#!/usr/bin/env python3
"""Measure the REAL stopping distance and stopping time of the robot in Gazebo.

Why this exists as a program rather than a number in a config: every safety margin in
this project is denominated in metres, and they are all currently guesses. "The robot
stops within 0.30 m at full speed" is either measured or it is a hope. P2's exit
criterion is that this has been measured, repeated, and frozen -- and that afterwards
a turning wheel has been shown not to clip a corner.

What it does, per speed:

    command the speed, wait until the robot is actually at it, cruise, command zero,
    then watch until the robot is genuinely below the stop threshold

and it records three numbers that are easy to confuse and must not be:

    zero_commanded_at        when zero was sent
    stop_confirmed_at        when the measured speed was first below the threshold
    stop_distance_m          ground truth travel between those two instants

"Commanded zero" is not "stopped". That distinction is the whole point of the
program: a report that quotes the commanded instant as the stopping time understates
the distance by however far the robot travelled while decelerating.

It commands through the SAFETY GATE, not around it, so the number it produces is the
number the real stack produces. If the gate refuses (no heartbeat, stale state, or a
latched estop) the run is reported as REFUSED with the gate's own reason instead of
being retried until it happens to work.

It also watches the lidar, and that is not optional. Commands go straight to the gate's
input, so nothing else is steering: the first version drove a robot into the east wall and
kept commanding, wheels turning against a wall while odometry integrated to 14 m. The
damage was not the wall, it was the corrupted odometry: the next navigation goal was
rejected with "Start Coordinates of (7.6029, -2.4390) was outside bounds", and a plausible
reading of that is a localisation bug rather than a test that drove into a wall. A run is
now refused before it starts if the forward clearance is short, and aborted mid-run if the
lidar closes below the margin.

Usage:
    ros2 run fleet_ros stop_distance_calibrator --ros-args \
        -p robot_id:=r01 -p speeds:="[0.10,0.20,0.35]"
Exit: 0 measured, 3 refused by the gate, 4 a measurement did not settle,
      6 no clear runway (the robot was left where a wall is in the way)
"""

from __future__ import annotations

import json
import math
import sys
import time
from dataclasses import dataclass, field
from pathlib import Path

import rclpy
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage

# NOT_RUN is the honest value: this program measures the plant, not a requirement.
NOT_RUN = "NOT_RUN"


class RunwayBlocked(RuntimeError):
    """The lidar says the runway ahead is no longer clear.

    Raised rather than returned so it cannot be ignored by a loop that only checks a
    status string: the only way to handle it is to stop commanding.
    """


def _qos(depth: int = 50) -> QoSProfile:
    return QoSProfile(
        reliability=QoSReliabilityPolicy.RELIABLE,
        durability=QoSDurabilityPolicy.VOLATILE,
        history=QoSHistoryPolicy.KEEP_LAST,
        depth=depth,
    )


@dataclass
class Sample:
    t: float
    speed: float
    x: float | None
    y: float | None
    source: str


@dataclass
class RunResult:
    commanded_mps: float
    achieved_mps: float = 0.0
    accel_distance_m: float = 0.0
    cruise_distance_m: float = 0.0
    stop_time_s: float = 0.0
    stop_distance_m: float = 0.0
    mean_decel_mps2: float = 0.0
    position_source: str = ""
    status: str = "NOT_RUN"
    note: str = ""
    gate_reasons: list[str] = field(default_factory=list)

    def as_dict(self) -> dict:
        return {
            "commanded_mps": round(self.commanded_mps, 4),
            "achieved_mps": round(self.achieved_mps, 4),
            "accel_distance_m": round(self.accel_distance_m, 4),
            "cruise_distance_m": round(self.cruise_distance_m, 4),
            "stop_time_s": round(self.stop_time_s, 4),
            "stop_distance_m": round(self.stop_distance_m, 4),
            "mean_decel_mps2": round(self.mean_decel_mps2, 4),
            "position_source": self.position_source,
            "status": self.status,
            "note": self.note,
            "gate_reasons": sorted(set(self.gate_reasons)),
        }


class StopDistanceCalibrator(Node):
    # phases
    ACCEL = "ACCEL"
    CRUISE = "CRUISE"
    DECEL = "DECEL"
    DONE = "DONE"

    def __init__(self) -> None:
        super().__init__("stop_distance_calibrator")

        self.declare_parameter("robot_id", "")
        self.declare_parameter("speeds", [0.10, 0.20, 0.35])
        self.declare_parameter("cmd_vel_topic", "cmd_vel_pre_gate")
        self.declare_parameter("odom_topic", "odom")
        # Empty by default, and that is deliberate. The per-robot ground-truth topic
        # this used to point at was removed: gz published gz.msgs.Pose while the bridge
        # subscribed as gz.msgs.Pose_V, so it never carried a message. Truth now arrives
        # on the world-level /world/<world>/dynamic_pose/info, which carries EVERY model,
        # so a consumer must filter by frame name before it means anything for one robot.
        # Rather than silently subscribe to a topic that cannot deliver, this defaults to
        # empty and the measurement falls back to odom and says so in its report.
        self.declare_parameter("ground_truth_topic", "")
        self.declare_parameter("gate_state_topic", "gate_state")
        self.declare_parameter("stop_speed_mps", 0.02)
        # How long to hold the target speed before stopping. Long enough that
        # acceleration transients are over, short enough not to hit a wall: at
        # 0.35 m/s this is 0.7 m.
        self.declare_parameter("cruise_s", 2.0)
        self.declare_parameter("reach_timeout_s", 8.0)
        self.declare_parameter("settle_timeout_s", 8.0)
        self.declare_parameter("settle_s", 2.0)
        # Wall protection. The commands bypass Nav2 and go straight to the gate, so
        # nothing else is steering; without these the node cheerfully drives into a wall
        # and corrupts odometry, which then looks like a localisation bug. The forward
        # sector is narrow because only the direction of travel matters.
        self.declare_parameter("scan_topic", "scan")
        self.declare_parameter("forward_sector_deg", 25.0)
        self.declare_parameter("min_forward_clearance_m", 0.35)
        self.declare_parameter("runway_margin_m", 0.25)
        self.declare_parameter("out_json", "")

        p = self.get_parameter
        self.robot_id = str(p("robot_id").value) or self.get_namespace().strip("/") or "r01"
        speeds = p("speeds").value
        self.speeds = [float(v) for v in (speeds if isinstance(speeds, list) else [speeds])]
        if not self.speeds:
            raise ValueError("speeds must not be empty")
        self.stop_speed = float(p("stop_speed_mps").value)
        self.cruise_s = float(p("cruise_s").value)
        self.reach_timeout_s = float(p("reach_timeout_s").value)
        self.settle_timeout_s = float(p("settle_timeout_s").value)
        self.settle_s = float(p("settle_s").value)
        self.forward_sector = math.radians(float(p("forward_sector_deg").value))
        self.min_clearance = float(p("min_forward_clearance_m").value)
        self.runway_margin = float(p("runway_margin_m").value)
        self.out_json = str(p("out_json").value)

        self.cmd_pub = self.create_publisher(
            TwistStamped, str(p("cmd_vel_topic").value), _qos())
        self.create_subscription(
            Odometry, str(p("odom_topic").value), self._on_odom, _qos())
        truth_topic = str(p("ground_truth_topic").value)
        if truth_topic:
            self.create_subscription(
                TFMessage, truth_topic, self._on_truth, _qos())
        self.create_subscription(
            String, str(p("gate_state_topic").value), self._on_gate, _qos())
        self.create_subscription(
            LaserScan, str(p("scan_topic").value), self._on_scan, _qos())

        self.speed = 0.0
        self.odom_xy: tuple[float, float] | None = None
        self.truth_xy: tuple[float, float] | None = None
        self.last_gate_reason = ""
        self.gate_reasons: list[str] = []
        self.forward_range: float | None = None
        self.scan_count = 0
        self.truth_is_positional = False
        self.t0 = time.monotonic()

        # Snapshot samples so a whole run can be reported, not just the endpoints.
        self.samples: list[Sample] = []

    # ------------------------------------------------------------------ #
    # subscriptions
    # ------------------------------------------------------------------ #

    def _now(self) -> float:
        return time.monotonic() - self.t0

    def _on_odom(self, msg: Odometry) -> None:
        lin = msg.twist.twist.linear
        self.speed = math.hypot(float(lin.x), float(lin.y))
        self.odom_xy = (
            float(msg.pose.pose.position.x), float(msg.pose.pose.position.y))

    def _on_truth(self, msg: TFMessage) -> None:
        """Ground truth, with the current channel's limitation made explicit.

        Two channel shapes are accepted, because both have existed here:

        * a NAME-bearing one (a per-model PosePublisher). Matched by suffix, not by exact
          name, because the published name carries a model instance prefix that differs
          per robot and an exact match would silently never fire.
        * the world-level /world/<world>/dynamic_pose/info that is actually wired up now.
          That mapping drops the frame names entirely, so the model pose can only be taken
          positionally -- entry 0. That is fine for one robot and WRONG for two, so it is
          flagged and carried into the report rather than being quietly assumed.
        """
        if not msg.transforms:
            return
        for tf in msg.transforms:
            child = (tf.child_frame_id or "").strip("/")
            if child.endswith("base_footprint") or child.endswith("base_link"):
                self.truth_xy = (float(tf.transform.translation.x),
                                 float(tf.transform.translation.y))
                self.truth_is_positional = False
                return
        first = msg.transforms[0].transform.translation
        self.truth_xy = (float(first.x), float(first.y))
        self.truth_is_positional = True

    def _on_gate(self, msg: String) -> None:
        try:
            state = json.loads(msg.data)
        except (ValueError, TypeError):
            return
        reason = str(state.get("reason", ""))
        if reason and reason != self.last_gate_reason:
            self.last_gate_reason = reason
            self.gate_reasons.append(reason)

    def _on_scan(self, msg: LaserScan) -> None:
        """Nearest return inside a narrow forward sector, in metres."""
        if not msg.ranges:
            return
        self.scan_count += 1
        n = len(msg.ranges)
        best: float | None = None
        for i, r in enumerate(msg.ranges):
            if not math.isfinite(r):
                continue
            ang = msg.angle_min + i * msg.angle_increment
            # wrap into (-pi, pi] so the sector test works whatever the scan layout is
            ang = math.atan2(math.sin(ang), math.cos(ang))
            if abs(ang) > self.forward_sector:
                continue
            if r < msg.range_min or r > msg.range_max:
                continue
            if best is None or r < best:
                best = r
        if best is not None:
            self.forward_range = best
        elif n:
            # A sector with nothing valid in it is NOT "clear": a lidar whose beam fell in
            # a gap must not be read as an open runway.
            self.forward_range = None

    # ------------------------------------------------------------------ #
    # plumbing
    # ------------------------------------------------------------------ #

    def _spin(self, duration_s: float, cmd: float) -> None:
        """Hold `cmd` and pump callbacks for `duration_s` wall seconds.

        Fails closed on the lidar: while a non-zero command is being held, a forward
        clearance below the margin stops the run. Reading a missing scan as "clear" is how
        a guard becomes decoration.
        """
        end = self._now() + duration_s
        msg = TwistStamped()
        msg.twist.linear.x = cmd
        while self._now() < end:
            if cmd > 1e-9 and self.scan_count:
                if self.forward_range is None or self.forward_range < self.min_clearance:
                    shown = ("no valid return in the forward sector"
                             if self.forward_range is None
                             else f"{self.forward_range:.3f} m")
                    raise RunwayBlocked(
                        f"forward clearance {shown}, margin {self.min_clearance:.3f} m, "
                        f"while commanding {cmd:.3f} m/s")
            self.cmd_pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.02)
            self.samples.append(Sample(self._now(), self.speed,
                                       *(self.truth_xy or self.odom_xy or (None, None)),
                                       "truth" if self.truth_xy else "odom"))

    def _runway_needed(self, target: float) -> float:
        """Distance the whole profile needs from here: accel + cruise + worst stop.

        Uses the plant's own acceleration limit rather than a guess, and adds the stop
        distance at THIS speed measured from v^2/2a, so the requirement scales the way the
        real profile does.
        """
        a = 0.5           # max_linear_acceleration in the SDF
        accel = target * target / (2 * a)
        cruise = target * self.cruise_s
        stop = target * target / (2 * a)
        return accel + cruise + stop + self.runway_margin

    def _position(self) -> tuple[tuple[float, float], str] | None:
        if self.truth_xy is not None:
            return self.truth_xy, "ground_truth"
        if self.odom_xy is not None:
            return self.odom_xy, "odom"
        return None

    # ------------------------------------------------------------------ #
    # one measurement
    # ------------------------------------------------------------------ #

    def measure(self, target: float) -> RunResult:
        res = RunResult(commanded_mps=target)
        log = self.get_logger()

        # --- is there room for the whole profile from here? -------------- #
        needed = self._runway_needed(target)
        if self.scan_count and (self.forward_range is None
                               or self.forward_range < needed):
            shown = ("no valid return in the forward sector"
                     if self.forward_range is None else f"{self.forward_range:.3f} m")
            res.status = "INSUFFICIENT_RUNWAY"
            res.note = (f"{shown} of clear floor ahead, but a {target:.2f} m/s profile "
                        f"needs {needed:.3f} m (accel + {self.cruise_s:.1f}s cruise + "
                        f"stop + {self.runway_margin:.2f} m margin). Move the robot to a "
                        "clear straight run; measuring here would drive it into a wall.")
            log.error(res.note)
            return res

        # --- reach the target speed ------------------------------------- #
        log.info(f"--- {target:.2f} m/s: accelerating "
                 f"(runway needed {needed:.2f} m, have "
                 f"{'?' if self.forward_range is None else f'{self.forward_range:.2f}'} m)")
        phase = self.ACCEL
        phase_start = self._now()
        accel_from = self._position()
        try:
            while phase == self.ACCEL:
                if self._now() - phase_start > self.reach_timeout_s:
                    res.status = "DID_NOT_SETTLE"
                    res.note = (f"never reached {target:.2f} m/s within "
                                f"{self.reach_timeout_s}s (best {self.speed:.3f} m/s)")
                    log.error(res.note)
                    return res
                self._spin(0.05, target)
                if self.speed >= 0.95 * target:
                    phase = self.CRUISE
                    phase_start = self._now()
                    cruise_from = self._position()
                    log.info(f"    reached {self.speed:.3f} m/s")

            # --- cruise -------------------------------------------------- #
            cruise_samples: list[float] = []
            end = self._now() + self.cruise_s
            while self._now() < end:
                self._spin(0.05, target)
                cruise_samples.append(self.speed)
        except RunwayBlocked as exc:
            self._spin(0.05, 0.0)
            res.status = "ABORTED_OBSTACLE"
            res.note = f"stopped early: {exc}"
            log.error(res.note)
            return res

        res.achieved_mps = sum(cruise_samples) / len(cruise_samples) if cruise_samples else 0.0
        log.info(f"    cruising at {res.achieved_mps:.3f} m/s "
                 f"(commanded {target:.2f})")

        before_zero = self._position()
        if before_zero is None:
            res.status = "NO_POSITION"
            res.note = "no odom and no ground truth arrived; cannot measure distance"
            log.error(res.note)
            return res
        cruise_pos, res.position_source = before_zero
        cruise_t = self._now()

        # --- command zero ------------------------------------------------ #
        self._spin(0.02, 0.0)
        zero_t = self._now()
        log.info(f"    zero commanded at t = {zero_t:.3f}s")

        # --- watch until genuinely stopped ------------------------------- #
        stop_t: float | None = None
        while stop_t is None:
            if self._now() - zero_t > self.settle_timeout_s:
                res.status = "DID_NOT_STOP"
                res.note = (f"still {self.speed:.4f} m/s {self.settle_timeout_s}s "
                            f"after zero was commanded")
                log.error(res.note)
                return res
            self._spin(0.02, 0.0)
            if self.speed <= self.stop_speed:
                stop_t = self._now()

        after = self._position()
        if after is None:
            res.status = "NO_POSITION"
            res.note = "position disappeared during the stop"
            return res
        stop_pos, src = after
        res.position_source = src

        res.stop_time_s = stop_t - zero_t
        res.stop_distance_m = math.dist(cruise_pos, stop_pos)
        if res.stop_time_s > 1e-6:
            res.mean_decel_mps2 = res.achieved_mps / res.stop_time_s
        if accel_from is not None:
            res.accel_distance_m = math.dist(accel_from[0], cruise_pos)
        res.cruise_distance_m = res.achieved_mps * (zero_t - cruise_t)
        res.status = "MEASURED"
        res.note = (f"{res.position_source} positions, threshold "
                    f"{self.stop_speed} m/s")

        log.info(f"    stop time {res.stop_time_s:.3f}s, stop distance "
                 f"{res.stop_distance_m:.4f} m over {res.position_source}")

        # Settle before the next speed, and let the robot come fully to rest so one
        # run's tail cannot contaminate the next run's distance.
        self._spin(self.settle_s, 0.0)
        return res

    # ------------------------------------------------------------------ #
    # driver
    # ------------------------------------------------------------------ #

    def run(self) -> int:
        log = self.get_logger()
        log.warning(
            "stop distance calibration: this node publishes velocity commands "
            "directly. Do NOT run it while Nav2 is navigating."
        )
        if self._foreign_publishers():
            log.error(
                "something else is already commanding "
                f"{self.get_name()}'s cmd_vel input ({self._foreign_publishers()}); "
                "measurements would be a mix of two controllers. Aborting."
            )
            return 5

        # Let odom, the scan and the gate state arrive. The scan is waited for because the
        # obstacle guard is only real if the lidar is actually being read.
        deadline = self._now() + 10.0
        while (self.odom_xy is None or self.truth_xy is None
               or self.scan_count == 0) and self._now() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
        log.info(
            f"inputs: odom={'yes' if self.odom_xy else 'NO'}, "
            f"ground_truth={'yes' if self.truth_xy else 'NO'}"
            + (" (positional: entry 0 of the message -- valid for ONE robot only)"
               if self.truth_xy and self.truth_is_positional else "")
            + f", scan={'yes' if self.scan_count else 'NO'} "
            f"({self.scan_count} msgs, forward clearance "
            f"{'?' if self.forward_range is None else f'{self.forward_range:.3f} m'})"
        )
        if self.odom_xy is None:
            log.error("no odom: start robot.launch.py first. Nothing measured.")
            return 4
        if not self.scan_count:
            log.error(
                "no lidar on the scan topic, so the obstacle guard cannot work. Refusing "
                "to command motion blind: this is how the first version drove a robot into "
                "the east wall and corrupted odometry. Check the bridge for scan."
            )
            return 6

        results: list[RunResult] = []
        for target in self.speeds:
            res = self.measure(target)
            results.append(res)
            if res.status == "MEASURED" and self.last_gate_reason in (
                    "NO_PERMIT", "PERMIT_EXPIRED", "LATCHED_ESTOP", "HEARTBEAT_LOST"):
                res.status = "REFUSED"
                res.note = (f"the gate refused motion ({self.last_gate_reason}); the "
                            "measurement is about the gate, not the plant")
            if res.status in ("REFUSED", "INSUFFICIENT_RUNWAY", "ABORTED_OBSTACLE"):
                # Every later speed needs MORE runway than this one did, so continuing
                # after a runway refusal would only produce more refusals -- and after an
                # obstacle abort the robot is closer to whatever it saw, not further.
                break

        report = {
            "robot_id": self.robot_id,
            "backend": "gazebo_nav2 (measured through the safety gate)",
            "speeds_commanded_mps": self.speeds,
            "stop_speed_threshold_mps": self.stop_speed,
            "runs": [r.as_dict() for r in results],
            "obstacle_guard": {
                "scan_topic": self.get_parameter("scan_topic").value,
                "forward_sector_deg": self.get_parameter("forward_sector_deg").value,
                "min_forward_clearance_m": self.min_clearance,
                "runway_margin_m": self.runway_margin,
                "clearance_at_end_m": (None if self.forward_range is None
                                       else round(self.forward_range, 4)),
                "note": (
                    "Commands go straight to the gate's input, so nothing else is "
                    "steering. Without this guard the run drives into a wall and corrupts "
                    "odometry, which afterwards looks like a localisation bug."),
            },
            "world_has_physics": True,
            "frozen_after_this_run": False,
            "note": (
                "These are SIMULATION figures from a specific gz/ODE build and a "
                "specific friction model. They are not transferable to hardware. "
                "Positions come from odom unless ground_truth_topic is set. The "
                "per-robot ground-truth topic was removed because it was silently "
                "empty; truth is now on the world-level dynamic_pose/info, which "
                "carries every model and needs frame filtering before any single "
                "robot's figure can be quoted from it. Truth is a verification "
                "channel and is never fed to control."
            ),
            "not_run": [
                "corner clipping by a turning wheel",
                "stopping accuracy on a real surface",
                "stop distance under payload",
            ],
        }

        measured = [r for r in results if r.status == "MEASURED"]
        refused = [r for r in results if r.status == "REFUSED"]
        blocked = [r for r in results
                   if r.status in ("INSUFFICIENT_RUNWAY", "ABORTED_OBSTACLE")]

        print("=" * 74)
        print(f"  009 stop distance calibration: {self.robot_id}")
        print("=" * 74)
        print(f"  {'speed':>7} {'achieved':>9} {'time(s)':>8} {'distance(m)':>12} "
              f"{'decel(m/s2)':>12}  status")
        for r in results:
            print(f"  {r.commanded_mps:7.2f} {r.achieved_mps:9.3f} "
                  f"{r.stop_time_s:8.3f} {r.stop_distance_m:12.4f} "
                  f"{r.mean_decel_mps2:12.3f}  {r.status}")
        print()
        if measured:
            worst = max(measured, key=lambda r: r.stop_distance_m)
            print(f"  worst measured stop distance: {worst.stop_distance_m:.4f} m "
                  f"at {worst.achieved_mps:.3f} m/s")
            print("  These are MEASURED figures for THIS build. Freeze them into "
                  "config only after repeating the run.")
        else:
            print("  NOTHING MEASURED. Physical stop distance remains NOT_RUN.")
        print("  Corner clipping by a turning wheel: NOT_RUN (separate test).")
        print("=" * 74)

        if self.out_json:
            path = Path(self.out_json)
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text(json.dumps(report, indent=2), encoding="utf-8")
            print(f"  wrote {path}")

        if refused:
            return 3
        if blocked:
            print("  The lidar refused the runway. No wall was driven into and odometry is")
            print("  intact; move the robot to a clear straight run and rerun.")
            return 6
        if len(measured) != len(self.speeds):
            return 4
        return 0

    def _foreign_publishers(self) -> list[str]:
        topic = self._resolve("cmd_vel_pre_gate")
        try:
            infos = self.get_publishers_info_by_topic(topic)
        except Exception:  # pragma: no cover
            return []
        return sorted({i.node_name for i in infos if i.node_name != self.get_name()})

    def _resolve(self, topic: str) -> str:
        ns = self.get_namespace().rstrip("/")
        return f"{ns}/{topic}" if ns else f"/{topic}"


def main(argv: list[str] | None = None) -> int:
    rclpy.init(args=argv)
    node = StopDistanceCalibrator()
    code = 1
    try:
        code = node.run()
    except KeyboardInterrupt:
        code = 130
    finally:
        for _ in range(5):
            node.cmd_pub.publish(TwistStamped())
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return code


if __name__ == "__main__":
    raise SystemExit(main())
