"""Hand targets and a measured, fail-closed manipulation state machine."""
import numpy as np

FINGERS = ('ff', 'mf', 'rf', 'th')
OPEN = np.array([0., 0., 0., 0.] * 3 + [.3, 0., 0., 0.])
GRASP = np.array([-.2684, .7374, 1.0586, 1.0175, 0., .8488, .9083, .7839,
                  .2684, .7374, 1.0586, 1.0175, 1.1135, .0672, 1.544, .0487])
# A finger-motion preset, not an independently validated pinch grasp.
PINCH = OPEN.copy()
PINCH[:4] = GRASP[:4]
PINCH[12:] = GRASP[12:]


def slew(current, target, rates, dt):
    return current + np.clip(target - current, -rates * dt, rates * dt)


def supported_grasp(obs):
    forces = obs['finger_forces']
    return forces['th'] > .02 and sum(forces[k] > .02 for k in FINGERS[:3]) >= 2


class Task:
    """No object pose writes, attachment constraints, or automatic resets."""
    def __init__(self, now, obs):
        self.phase = 'SETTLE'
        self.since = now
        self.start_z = obs['object'][2]
        self.stable_since = None
        self.grasp_lost_since = None
        self.events = [{'time': now, 'phase': self.phase}]
        self.result = None
        self.reason = ''
        self.max_lift = 0.
        self.hold_reference = None

    def enter(self, phase, now):
        self.phase, self.since, self.stable_since = phase, now, None
        self.events.append({'time': now, 'phase': phase})

    def finish(self, result, reason, now):
        self.result, self.reason = result, reason
        self.enter(result, now)

    def stable(self, condition, now, duration):
        if not condition:
            self.stable_since = None
        elif self.stable_since is None:
            self.stable_since = now
        return self.stable_since is not None and now - self.stable_since >= duration

    def command(self, now, obs):
        age = now - self.since
        hand, y, z = GRASP.copy(), 0., 0.
        self.max_lift = max(self.max_lift, obs['object'][2] - self.start_z)
        if self.result:
            return None
        if self.phase in ('LIFT', 'HOLD', 'TRANSFER', 'LOWER'):
            if not supported_grasp(obs):
                if self.grasp_lost_since is None:
                    self.grasp_lost_since = now
                if now - self.grasp_lost_since > .5:
                    self.finish('FAILED', 'LOST_FINGER_CONTACT', now)
                    return None
            else:
                self.grasp_lost_since = None
        if obs['object'][2] < .10:
            self.finish('FAILED', 'OBJECT_DROPPED', now)
            return None
        if self.phase == 'SETTLE':
            hand = OPEN.copy()
            if age >= 1.:
                self.enter('CLOSE', now)
        elif self.phase == 'CLOSE':
            hand = OPEN + np.clip(age / 2., 0, 1) * (GRASP - OPEN)
            if age >= 3.:
                self.enter('VERIFY_GRASP', now)
        elif self.phase == 'VERIFY_GRASP':
            if self.stable(supported_grasp(obs), now, .5):
                self.enter('LIFT', now)
            elif age > 3.:
                self.finish('FAILED', 'NO_OPPOSED_GRASP', now)
        elif self.phase == 'LIFT':
            z = .12 * min(age / 2.5, 1.)
            if age > 3.:
                if obs['object'][2] - self.start_z >= .09 and not obs['source_contact']:
                    self.hold_reference = np.array(obs['relative_object'])
                    self.enter('HOLD', now)
                else:
                    self.finish('FAILED', 'INSUFFICIENT_LIFT', now)
        elif self.phase == 'HOLD':
            z = .12
            slip = np.linalg.norm(np.array(obs['relative_object']) - self.hold_reference)
            if slip > .02:
                self.finish('FAILED', 'OBJECT_SLIPPED', now)
            elif age >= 2.:
                self.enter('TRANSFER', now)
        elif self.phase == 'TRANSFER':
            z, y = .12, .20 * min(age / 3., 1.)
            if age > 3.5:
                self.enter('LOWER', now)
        elif self.phase == 'LOWER':
            # Keep the fully open fingers clear of the receiving surface.
            y, z = .20, .10 + .02 * max(1. - age / 2.5, 0.)
            if age >= 3.:
                self.enter('RELEASE', now)
        elif self.phase == 'RELEASE':
            y, z = .20, .10 + .02 * min(age / 2., 1.)
            hand = GRASP + np.clip(age / 2., 0, 1) * (OPEN - GRASP)
            if age >= 2.5:
                self.enter('VERIFY_PLACE', now)
        elif self.phase == 'VERIFY_PLACE':
            hand, y, z = OPEN.copy(), .20, .12
            p = obs['object']
            placed = (abs(p[0] - .06) < .035 and abs(p[1] - .20) < .035
                      and abs(p[2] - .205) < .012 and obs['tray_contact']
                      and sum(obs['finger_forces'].values()) < .02 and obs['speed'] < .02)
            if self.stable(placed, now, 1.):
                self.finish('COMPLETED', 'LIFT_HOLD_TRANSFER_RELEASE_VERIFIED', now)
            elif age > 5.:
                self.finish('FAILED', 'PLACEMENT_NOT_STABLE', now)
        return np.r_[hand, y, z]
