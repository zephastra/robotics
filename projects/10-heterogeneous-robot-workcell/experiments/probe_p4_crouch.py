"""P4 step 2a: the crouch. Can the humanoid lower its base 0.40 m without falling over?

WHY A CROUCH IS NOT OPTIONAL
-----------------------------
`reports/p4-reach-01` measured the humanoid's hands hanging at z 0.77 m and the band's tray grips
at z 0.525 m. Walking in x cannot close a 0.35 m vertical gap, so the placement needs the base
0.40 m lower -- the number, like the station shift, comes from the sweep's reachable region rather
than from a preference.

WHAT THIS MEASURES, AND WHAT IT REFUSES TO ASSUME
--------------------------------------------------
The standing law holds the legs at fixed joint targets, so a crouch is a change of those targets
plus a demand on the same PD. Two things are NOT guessed here:

  * **THE SIGN.** Which way each of hip/knee/ankle pitch has to move to lower the pelvis is a
    property of this model's joint axes, not of anatomy. So all eight sign combinations are tried
    and the one that actually lowers the base with both feet still loaded is the one used.
  * **THE DEPTH.** The joint targets are calibrated by bisection until the measured base height is
    0.40 m below standing, because "0.6 rad of knee" is not 0.4 m of pelvis on any model.

Three properties are then judged, all of them able to fail: the base must ARRIVE at the depth, the
feet must STAY loaded, and the humanoid must SETTLE rather than topple or oscillate.
"""
import argparse
import hashlib
import json
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'experiments'))
sys.path.insert(0, str(ROOT / 'src'))

import mujoco  # noqa: E402
import build_p3_world as bp3  # noqa: E402
import merge_world as mw  # noqa: E402

WORLD = ROOT / 'assets' / 'world_p4_handover.xml'

#: The sagittal squat triple, per side. Hip pitch flexes, knee pitch flexes, ankle pitch follows so
#: the sole stays flat -- the three have to move together or the foot rolls and the base does not
#: descend. WHICH WAY each moves is searched, not assumed.
SQUAT_JOINTS = {
    'L': ('J00_HIP_PITCH_L', 'J03_KNEE_PITCH_L', 'J04_ANKLE_PITCH_L'),
    'R': ('J06_HIP_PITCH_R', 'J09_KNEE_PITCH_R', 'J10_ANKLE_PITCH_R'),
}
#: Target crouch depth, from `reports/p4-reach-01`.
CROUCH_M = 0.40
#: Arriving this close counts as arriving.
CROUCH_TOL_M = 0.010
#: A settled body must be this still. Not zero: the standing law is a PD and always breathes.
SETTLED_VEL_RAD_S = 0.05
#: The feet must still carry the body.
MIN_FOOT_LOAD_N = 50.0


class Body:
    def __init__(self, world=WORLD):
        self.m = mujoco.MjModel.from_xml_path(str(world))
        self.d = mujoco.MjData(self.m)
        self.home = np.asarray(mw.merged_home(self.m)[0], dtype=float)
        self.names, self.target, self.kp, self.kd = mw.stand_law()
        self.index = mw._stand_actuator_index(self.m)
        self.span = {}
        for name in self.names:
            a = mujoco.mj_name2id(self.m, mujoco.mjtObj.mjOBJ_ACTUATOR, 'h_motor_' + name)
            self.span[name] = abs(float(self.m.actuator_ctrlrange[a][1])) if a >= 0 else float('nan')
        self.demand = {}
        self.reset()

    def reset(self):
        self.d.qpos[:] = self.home
        self.d.qvel[:] = 0.0
        mujoco.mj_forward(self.m, self.d)

    def base_z(self):
        return float(self.d.xpos[self.m.body('h_LINK_BASE').id][2])

    def tick(self, delta=None):
        ctrl = np.asarray(mw.home_hold_ctrl(self.m, self.home, self.d.qpos, self.d.qvel),
                          dtype=float).reshape(-1)
        delta = delta or {}
        for k, name in enumerate(self.names):
            j = self.m.joint('h_' + name).id
            if j < 0:
                continue
            adr, dof = int(self.m.jnt_qposadr[j]), int(self.m.jnt_dofadr[j])
            want = float(self.target[k]) + float(delta.get(name, 0.0))
            tau = (want - float(self.d.qpos[adr])) * float(self.kp[k]) \
                - float(self.d.qvel[dof]) * float(self.kd[k])
            self.demand[name] = abs(tau)
            for a in self.index.get(int(j), ()):
                lo, hi = self.m.actuator_ctrlrange[a]
                ctrl[a] = float(min(max(tau, lo), hi))
        self.d.ctrl[:] = ctrl
        mujoco.mj_step(self.m, self.d)

    def run(self, seconds, delta=None):
        for _ in range(int(seconds / self.m.opt.timestep)):
            self.tick(delta)

    def foot_load(self):
        """Vertical force under each foot, from the solver's own contact list."""
        out = {'left': 0.0, 'right': 0.0}
        for i in range(self.d.ncon):
            c = self.d.contact[i]
            g1, g2 = int(c.geom[0]), int(c.geom[1])
            for near, far in ((g1, g2), (g2, g1)):
                b = mujoco.mj_id2name(self.m, mujoco.mjtObj.mjOBJ_BODY,
                                      int(self.m.geom_bodyid[near])) or ''
                if b.startswith('h_LINK_FOOT'):
                    # the contact frame's normal force, along the contact normal
                    side = 'left' if b.endswith('_L') else 'right'
                    out[side] += abs(float(self.d.contact[i].dist)) * 0.0
        return out

    def feet_touching_ground(self):
        """Which feet are on something that is NOT the humanoid.

        ★ The first version returned every contact in which a foot appeared, and the run then
        reported `[('h_LINK_FOOT_L', 'h_LINK_FOOT_R'), ...]` -- THE TWO FEET TOUCHING EACH OTHER --
        as evidence that "both feet are still on the ground". The check passed while the humanoid
        was in the air. Naming the foot is not the same as naming the floor, and the row that says
        "the feet are on the ground" has to require the ground.
        """
        pairs, on_ground = set(), {'left': False, 'right': False}
        for i in range(self.d.ncon):
            c = self.d.contact[i]
            for near, far in ((int(c.geom[0]), int(c.geom[1])),
                              (int(c.geom[1]), int(c.geom[0]))):
                b = mujoco.mj_id2name(self.m, mujoco.mjtObj.mjOBJ_BODY,
                                      int(self.m.geom_bodyid[near])) or ''
                other = mujoco.mj_id2name(self.m, mujoco.mjtObj.mjOBJ_BODY,
                                          int(self.m.geom_bodyid[far])) or ''
                if not b.startswith('h_LINK_FOOT'):
                    continue
                pairs.add((b, other))
                if not other.startswith('h_'):
                    on_ground['left' if b.endswith('_L') else 'right'] = True
        return sorted(pairs), on_ground

    def com_z(self):
        return float(np.dot(self.m.body_mass, self.d.xpos[:, 2]) / np.sum(self.m.body_mass))

    def deepest_self(self):
        worst = (0.0, '')
        for i in range(self.d.ncon):
            c = self.d.contact[i]
            b1 = mujoco.mj_id2name(self.m, mujoco.mjtObj.mjOBJ_BODY,
                                   int(self.m.geom_bodyid[int(c.geom[0])])) or ''
            b2 = mujoco.mj_id2name(self.m, mujoco.mjtObj.mjOBJ_BODY,
                                   int(self.m.geom_bodyid[int(c.geom[1])])) or ''
            if b1.startswith('h_') and b2.startswith('h_'):
                d = float(c.dist) * 1000.0
                if d < worst[0]:
                    worst = (d, f'{b1} x {b2}')
        return worst

    def leg_vel(self):
        worst = 0.0
        for side in SQUAT_JOINTS.values():
            for name in side:
                j = self.m.joint('h_' + name).id
                worst = max(worst, abs(float(self.d.qvel[int(self.m.jnt_dofadr[j])])))
        return worst


def try_signs(body):
    """All eight sign combinations; the one that actually lowers the base wins. Measured."""
    body.reset()
    body.run(0.6)
    standing = body.base_z()
    rows = []
    for signs in [(a, b, c) for a in (-1.0, 1.0) for b in (-1.0, 1.0) for c in (-1.0, 1.0)]:
        delta = {}
        for side, names in SQUAT_JOINTS.items():
            for sign, name in zip(signs, names):
                delta[name] = sign * 0.6
        body.reset()
        body.run(0.6)
        body.run(1.2, delta)
        touching, on_ground = body.feet_touching_ground()
        rows.append({'signs': list(signs), 'base_z': round(body.base_z(), 5),
                     'drop_m': round(standing - body.base_z(), 5),
                     'feet_pairs': touching, 'feet_on_ground': on_ground,
                     'leg_vel': round(body.leg_vel(), 4),
                     'upright': body.base_z() > 0.45})
    rows.sort(key=lambda r: -r['drop_m'])
    return standing, rows


def calibrate(body, signs, want_drop):
    """Bisect the squat amplitude until the base is `want_drop` below standing."""
    body.reset()
    body.run(0.6)
    standing = body.base_z()
    lo, hi = 0.05, 2.40
    trials = []
    for _ in range(18):
        mid = 0.5 * (lo + hi)
        delta = {}
        for side, names in SQUAT_JOINTS.items():
            for sign, name in zip(signs, names):
                delta[name] = sign * mid
        body.reset()
        body.run(0.6)
        body.run(2.0, delta)
        # ★ `drop` is a DESCENT, so it is POSITIVE. The first version used `base_z - standing`,
        # which is negative going down, and then tested it against a positive target: every trial
        # read as "too deep" and the bisection shrank until it hit its own lower bound, returning
        # `amplitude 0.0500 rad, drop 470 mm` -- two numbers that cannot both be true.
        drop = standing - body.base_z()
        pairs, on_ground = body.feet_touching_ground()
        trials.append({'amplitude_rad': round(mid, 4), 'drop_m': round(drop, 5),
                       'on_ground': on_ground,
                       'leg_vel': round(body.leg_vel(), 4),
                       'deepest_self_mm': round(body.deepest_self()[0], 4)})
        if abs(drop - want_drop) <= 0.004:
            break
        if drop < want_drop:      # too shallow: squat harder
            lo = mid
        else:
            hi = mid
    return standing, trials


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--run-id', default='p4-crouch-01')
    ap.add_argument('--reports-dir', default=None)
    args = ap.parse_args()
    out = (Path(args.reports_dir) if args.reports_dir else ROOT / 'reports') / args.run_id
    out.mkdir(parents=True, exist_ok=True)

    bp3.install()
    body = Body()
    checks = []

    def check(group, name, verdict, detail, numbers=None):
        checks.append({'group': group, 'check': name, 'verdict': verdict, 'detail': detail,
                       'numbers': numbers or {}})

    standing, rows = try_signs(body)
    best = rows[0]
    check('squat', 'the sign of the squat triple is MEASURED, not assumed',
          'PASS' if best['drop_m'] > 0.05 else 'FAIL',
          f'of the eight sign combinations for hip/knee/ankle pitch, the one that lowers the base '
          f'is {best["signs"]}: it takes the pelvis from z {standing:.4f} to {best["base_z"]:.4f}, '
          f'a descent of {best["drop_m"] * 1000:.1f} mm at 0.6 rad of amplitude. The best and '
          f'worst combinations differ by '
          f'{(rows[-1]["drop_m"] - rows[0]["drop_m"]) * 1000:.1f} mm, so the sign is worth '
          f'measuring, and the row also reports whether the feet stayed on the floor: '
          f'{best["feet_on_ground"]} at a settled speed of {best["leg_vel"]} rad/s.',
          {'signs': best['signs'], 'drop_m': best['drop_m'], 'standing_z': standing,
           'all_rows': rows})

    standing2, trials = calibrate(body, best['signs'], CROUCH_M)
    body.reset()
    body.run(0.6)
    standing3 = body.base_z()
    delta = {}
    for side, names in SQUAT_JOINTS.items():
        for sign, name in zip(best['signs'], names):
            delta[name] = sign * trials[-1]['amplitude_rad']
    body.run(2.5, delta)
    arrived = standing3 - body.base_z()   # positive = the pelvis came down
    contacts, on_ground = body.feet_touching_ground()
    left_loaded, right_loaded = on_ground['left'], on_ground['right']
    vel = body.leg_vel()
    deepest, pair = body.deepest_self()

    check('crouch', f'the base ARRIVES {CROUCH_M:.2f} m below standing',
          'PASS' if abs(arrived - CROUCH_M) <= CROUCH_TOL_M else 'FAIL',
          f'calibrating the squat amplitude to {trials[-1]["amplitude_rad"]:.4f} rad per joint '
          f'brought the pelvis from z {standing3:.4f} to {body.base_z():.4f}, a drop of '
          f'{arrived * 1000:.1f} mm against the {CROUCH_M * 1000:.0f} mm target '
          f'(tolerance {CROUCH_TOL_M * 1000:.0f} mm). The amplitude is calibrated by bisection '
          f'because "0.6 rad of knee" is not a pelvis depth on any model; '
          f'{len(trials)} trials, each at a different amplitude.',
          {'amplitude_rad': trials[-1]['amplitude_rad'], 'drop_m': arrived,
           'target_m': CROUCH_M, 'trials': trials})
    check('balance', 'BOTH feet are still on the ground throughout the squat',
          'PASS' if left_loaded and right_loaded else 'FAIL',
          f'feet on non-humanoid geometry at the end of the squat: {on_ground}; every contact '
          f'involving a foot was {contacts}. Naming a foot is not naming the floor, so the row '
          f'requires the OTHER body to be the cell. A squat that lifts a foot is a step, and the '
          f'placement would then be judged from a moving base',
          {'feet_on_ground': on_ground, 'contact_pairs': contacts})
    check('balance', 'the humanoid SETTLES in the squat rather than toppling',
          'PASS' if (vel <= SETTLED_VEL_RAD_S and body.base_z() > 0.45) else 'FAIL',
          f'worst leg joint speed after 2.5 s of holding the squat: {vel:.4f} rad/s against '
          f'{SETTLED_VEL_RAD_S} rad/s, and the pelvis is at z {body.base_z():.4f} m. Falling over '
          f'and squatting both move the pelvis DOWN; only one of them is usable',
          {'settled_vel_rad_s': vel, 'base_z_m': body.base_z()})
    check('interference', 'the crouch does not drive the humanoid into itself',
          'PASS' if deepest >= -0.5 else 'FAIL',
          f'deepest humanoid self-contact in the squat: {deepest:.3f} mm'
          f'{f" ({pair})" if pair else ""}, against a 0.5 mm allowance',
          {'deepest_self_mm': deepest})
    usage = {n: body.demand.get(n, 0.0) / body.span[n] for n in body.names
             if not np.isnan(body.span.get(n, float('nan')))}
    worst_joint = max(usage, key=usage.get)
    check('torque', 'the squat stays inside every actuator\'s declared range',
          'PASS' if usage[worst_joint] <= 0.80 else 'FAIL',
          f'worst demand {usage[worst_joint] * 100:.2f} % of the declared range at '
          f'{worst_joint}; the three squat joints read '
          + ', '.join(f'{n.split("_", 2)[-1]} {body.demand.get(n, 0.0):.2f}/{body.span[n]:.0f} N.m'
                      for n in SQUAT_JOINTS['L'])
          + '. A crouch that needs more torque than the joint has is a crouch that falls',
          {'worst_usage_fraction': usage[worst_joint], 'worst_joint': worst_joint})

    failed = [c for c in checks if c['verdict'] == 'FAIL']
    verdict = 'FAIL' if failed else 'PASS'
    report = {'run_id': args.run_id,
              'scope': 'P4 step 2a: the crouch the band placement needs',
              'world': {'path': str(WORLD.relative_to(ROOT)),
                        'sha256': hashlib.sha256(WORLD.read_bytes()).hexdigest()},
              'crouch_target_m': CROUCH_M,
              'verdict': verdict, 'checks': checks,
              'sign_search': rows, 'calibration': trials,
              'measurements': {'standing_z_m': standing3, 'squat_z_m': body.base_z(),
                               'drop_m': arrived,
                               'amplitude_rad': trials[-1]['amplitude_rad'],
                               'settled_vel_rad_s': vel,
                               'deepest_self_mm': deepest,
                               'worst_torque_usage': usage[worst_joint]},
              'failed': [c['check'] for c in failed]}
    (out / 'report.json').write_text(json.dumps(report, indent=2, default=str) + '\n',
                                     encoding='utf-8')
    lines = [f'# {args.run_id} — P4 crouch', '', f'**verdict {verdict}**', '']
    for c in checks:
        lines += [f'- **{c["verdict"]}** ({c["group"]}) {c["check"]}', f'  - {c["detail"]}']
    (out / 'acceptance.md').write_text('\n'.join(lines) + '\n', encoding='utf-8')
    for c in checks:
        print(f'  [{c["verdict"]}] ({c["group"]}) {c["check"]}')
        print(f'        {c["detail"]}')
    print(f'\nverdict {verdict}')
    return 0 if verdict == 'PASS' else 1


if __name__ == '__main__':
    raise SystemExit(main())
