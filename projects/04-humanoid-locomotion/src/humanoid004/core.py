"""Pure control/acceptance logic; coordinates in meters, radians, seconds."""
from dataclasses import dataclass
import math
import numpy as np


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def orientation(q):
    w, x, y, z = q
    norm = float(np.linalg.norm(q))
    if not np.isfinite(norm) or norm < 1e-8:
        raise ValueError('Invalid orientation quaternion')
    w, x, y, z = np.asarray(q) / norm
    yaw = math.atan2(2 * (w*z + x*y), 1 - 2 * (y*y + z*z))
    tilt = math.acos(float(np.clip(1 - 2 * (x*x + y*y), -1, 1)))
    return yaw, tilt


class CommandFilter:
    def __init__(self, limits, acceleration, timeout):
        self.limits = np.asarray(limits, dtype=float)
        self.acceleration = np.asarray(acceleration, dtype=float)
        self.timeout = timeout
        self.value = np.zeros(3)
        self.target = np.zeros(3)
        self.last_input = -math.inf

    def submit(self, target, now):
        target = np.asarray(target, dtype=float)
        if target.shape != (3,) or not np.all(np.isfinite(target)):
            raise ValueError('Velocity command must contain three finite numbers')
        self.target = np.clip(target, -self.limits, self.limits)
        self.last_input = now

    def step(self, now, dt):
        if dt <= 0 or not math.isfinite(dt):
            raise ValueError('Control timestep must be positive')
        desired = self.target if 0 <= now - self.last_input <= self.timeout else np.zeros(3)
        self.value += np.clip(desired - self.value, -self.acceleration * dt, self.acceleration * dt)
        return self.value.copy()


@dataclass
class Pose:
    x: float
    y: float
    yaw: float


def target_command(pose, goal):
    dx, dy = goal['x'] - pose.x, goal['y'] - pose.y
    distance = math.hypot(dx, dy)
    bearing = math.atan2(dy, dx)
    # Travel facing the target; close to the goal, regulate all three errors.
    yaw_error = wrap((bearing if distance > 0.45 else goal['yaw']) - pose.yaw)
    c, s = math.cos(pose.yaw), math.sin(pose.yaw)
    forward, sideways = c * dx + s * dy, -s * dx + c * dy
    if distance > 0.45:
        vx = min(0.45, 0.7 * distance) * max(0.0, math.cos(yaw_error))
        if abs(yaw_error) > 0.65:
            vx = 0.0
        return np.array([vx, 0.0, np.clip(1.2 * yaw_error, -0.5, 0.5)])
    return np.array([np.clip(0.9 * forward, -0.15, 0.25),
                     np.clip(0.9 * sideways, -0.15, 0.15),
                     np.clip(1.2 * yaw_error, -0.5, 0.5)])


class Route:
    def __init__(self, config, now=0.0):
        self.cfg = config
        self.index = 0
        self.phase = 'WARMUP'
        self.stage_start = now
        self.inside_since = None
        self.events = []
        self.reason = ''
        self.hold_samples = []

    @property
    def goal(self):
        return self.cfg['route'][min(self.index, len(self.cfg['route']) - 1)]

    def step(self, pose, now, speed=0.0):
        if self.phase in ('COMPLETED', 'FAILED'):
            return np.zeros(3)
        if self.phase == 'WARMUP':
            if now - self.stage_start < self.cfg['warmup_seconds']:
                return np.zeros(3)
            self.phase, self.stage_start = 'WALKING', now
        if now - self.stage_start > self.cfg['waypoint_timeout']:
            self.phase, self.reason = 'FAILED', 'WAYPOINT_TIMEOUT: ' + self.goal['name']
            return np.zeros(3)
        distance = math.hypot(pose.x - self.goal['x'], pose.y - self.goal['y'])
        heading = abs(wrap(self.goal['yaw'] - pose.yaw))
        inside = distance <= self.cfg['position_tolerance'] and heading <= math.radians(self.cfg['yaw_tolerance_degrees'])
        if inside:
            if self.inside_since is None:
                self.inside_since = now
                self.hold_samples = []
            self.hold_samples.append((distance, heading, speed))
            self.phase = 'HOLDING'
            if now - self.inside_since >= self.cfg['hold_seconds']:
                mean_speed = float(np.mean([s[2] for s in self.hold_samples]))
                if mean_speed > self.cfg.get('hold_mean_speed_limit', .15):
                    self.inside_since = now
                    self.hold_samples = []
                    return target_command(pose, self.goal)
                self.events.append({'goal': self.goal['name'], 'sim_time': now, 'position_error': distance,
                                    'yaw_error_degrees': math.degrees(heading), 'hold_seconds': now-self.inside_since,
                                    'hold_max_position_error': max(s[0] for s in self.hold_samples),
                                    'hold_max_yaw_error_degrees': math.degrees(max(s[1] for s in self.hold_samples)),
                                    'hold_mean_speed_mps': mean_speed})
                self.index += 1
                self.inside_since = None
                self.stage_start = now
                self.phase = 'COMPLETED' if self.index == len(self.cfg['route']) else 'WALKING'
        else:
            self.inside_since = None
            self.hold_samples = []
            self.phase = 'WALKING'
        return np.zeros(3) if self.phase == 'COMPLETED' else target_command(pose, self.goal)


class FallMonitor:
    def __init__(self, height, tilt_deg, debounce):
        self.height, self.tilt = height, math.radians(tilt_deg)
        self.debounce, self.bad_since = debounce, None

    def update(self, z, tilt, now):
        if not all(math.isfinite(v) for v in (z, tilt, now)):
            return True
        if z < self.height or tilt > self.tilt:
            if self.bad_since is None:
                self.bad_since = now
            return now - self.bad_since >= self.debounce
        self.bad_since = None
        return False
