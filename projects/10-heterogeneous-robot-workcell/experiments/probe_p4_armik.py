"""P4 step 3a: the arms SOLVE for the tray instead of being welded to a pose.

WHAT CHANGES FROM `probe_p4_ff2`
--------------------------------
`probe_p4_ff2`'s `solve_posture` OVERWRITES the 10 arm joint entries of the IK target with a frozen
carry pose. The consequence, measured last round with forward kinematics, is that the hands descend
with the pelvis: at a 0.20 m crouch the hands sit **0.2155 m** away from the tray's handles, which
are fixed in the world. Nothing in that probe checks the gap, so `arms='carry'` was, in the
handover's terms, a humanoid crouching EMPTY-HANDED.

Here the arms are solved to the handles on every control step, with a damped-least-squares IK over
the same 5 joints per side that `probe_p4_reach` declares. The elbows and shoulders are therefore
free to flex and retract exactly as the review describes, and the body does not have to lean
forward to close a gap that no longer exists.

THE GATE THE REVIEW ASKS FOR
----------------------------
`HAND_GAP_MAX_M = 0.002`. Both hands must stay within 2 mm of their handles for the WHOLE run --
not merely at the end -- or the depth FAILS. The gap is measured in the posture the CONTROLLER is
actually commanded to track (`q_cmd`), on the IK's own data, so it cannot be a restatement of the
solver's residual: if a solve is rejected and the previous posture is held, the two numbers differ
and only the commanded one decides whether the tray is held.

WHY THE IK HAS ITS OWN MjData
-----------------------------
`probe_p4_reach.ik` evaluates `d.site_xpos` on the rig it is handed. Called with a foreign rig after
`qpos` was written elsewhere it reads STALE sites, and reports a solution for a pose it never set --
which is how an earlier round concluded a single elbow flexion recovered 0.2165 m while the shoulder
"did not move". `ArmIK` owns its data and is seeded from the caller's qpos, so that cannot happen.

WHAT THIS DOES NOT CLAIM
------------------------
The tray is a separate free body; the handles are read from where it currently is. This is a
MEASUREMENT of holding, not a grip: there is no constraint between hand and handle, and the
fingertips are not actuated against the ball. "The hands are within 2 mm of the interface" is the
claim, and the run counts its own qpos writes to show it never moved the tray.
"""
import argparse
import json
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'experiments'))
sys.path.insert(0, str(ROOT / 'src'))

import mujoco  # noqa: E402
import build_p3_world as bp3  # noqa: E402

bp3.install()
import merge_world as mw  # noqa: E402
import probe_p4_balance as PB  # noqa: E402
import probe_p4_ff2 as F2  # noqa: E402
import probe_p4_reach as PR  # noqa: E402
import probe_p4_crouch2 as C2  # noqa: E402
from _p4_arm_ik import ArmIK  # noqa: E402  -- the solver, next to this file

WORLD = ROOT / 'assets' / 'world_p4_handover.xml'

#: The review's gate. 2 mm against a 35 mm handle ball: under a tenth of its radius, inside the
#: finger pads' own compliance, and the same number the reach study uses for "on the handle".
HAND_GAP_MAX_M = 0.002
#: How far the tray may drift from where it started. The tray is a free body with no constraint to
#: the hands, so this row is the honest form of "the tray was carried": if it moves, the run never
#: held it, whatever the hand-to-handle number says. 2 mm -- the same order as the grip allowance,
#: and far below the 0.84 m of drift that a stale handle reference once hid.
TRAY_SHIFT_MAX_M = 0.002
#: Which hand reaches which handle. Keyed by the SIGN OF y, because that is what decides it and the
#: merged model is not obliged to keep the tray source's own ordering.
SITE_OF_SIDE = {'+y': 'h_lh_grasp', '-y': 'h_rh_grasp'}


class GripRig(F2.FFRig):
    """`FFRig` with the arm lock replaced by an arm solve. The feed-forward is left untouched."""

    def __init__(self, world=WORLD):
        super().__init__(world)
        limits = {n: (float(self.m.jnt_range[self.m.joint('h_' + n).id][0]),
                      float(self.m.jnt_range[self.m.joint('h_' + n).id][1]))
                  for n in PR.ARM_JOINTS}
        self.arm_ik = ArmIK(self.m, PR.ARM_JOINTS, limits, SITE_OF_SIDE)
        self.handles = None            # side -> world point, from the tray's own handles
        self.handles_read_step = None  # what THIS step read; reused by measure_gap
        self.last_residual_m = None
        self.last_per_side_m = {}
        self.hand_gap_m = {}           # side -> gap in the COMMANDED posture
        self.tray_shift_m = 0.0        # how far the tray has moved since the handles were read
        self.tray_start = None
        self.arm_ik_iters = 0
        self.arm_ik_clamps = 0
        self.carry_pose = None         # deliberately unused: nothing overwrites the arms any more

    def read_handles(self):
        """The tray's grip interface, from the tray's CURRENT pose. Two geoms, or it is an error."""
        out = {}
        tray = self.m.body('c_payload').id
        for g in range(self.m.ngeom):
            if int(self.m.geom_bodyid[g]) != int(tray):
                continue
            name = mujoco.mj_id2name(self.m, mujoco.mjtObj.mjOBJ_GEOM, g) or ''
            if 'handle' in name and 'stem' not in name:
                p = np.array(self.d.geom_xpos[g], dtype=float)
                out['+y' if p[1] > 0 else '-y'] = p
        if len(out) != 2:
            raise RuntimeError('the tray has %d handle geoms; the interface is two' % len(out))
        return out

    def solve_posture(self, pelvis, seed):
        """Legs through the frozen solver, then ARMS SOLVED onto the handles.

        The distinction that matters: `FFRig` writes a FIXED arm pose here, so the hands go wherever
        the pelvis takes them. This solves the arm chain against the handles' actual world points,
        which is the whole point -- the arms absorb the descent instead of the body leaning for it.

        ★ THE HANDLES ARE RE-READ HERE, EVERY STEP, and that is not an optimisation detail.
        The first version of this probe read them ONCE at the top of `run` and reused that constant
        for all 1750 steps. The tray is a free body that nothing holds, so it leaves; the IK then
        chased a point frozen in space and converged to it beautifully, and the gate reported a
        1 um hand-to-handle gap while the tray was **0.84 m away**. Measured, before the fix:
        frozen handle x 4.34353, fresh handle x 5.11577, reported gap 1e-6 m.
        """
        if self.handles is not None:
            self.handles = self.read_handles()
            self.handles_read_step = self.handles
        q, resid, margins = super().solve_posture(pelvis, seed)
        if self.handles is not None:
            before = self.arm_ik.clamp_hits
            q, worst, per = self.arm_ik.solve(q, self.handles)
            self.arm_ik_clamps += self.arm_ik.clamp_hits - before
            # the solver's own residuals are kept, so `measure_gap` does not have to re-run the
            # forward passes to recover the same numbers
            self.last_residual_m = float(worst)
            self.last_per_side_m = {k: float(v) for k, v in per.items()}
        return q, resid, margins

    def measure_gap(self, q_cmd):
        """Hand-to-handle distance in the COMMANDED posture, plus how far the TRAY has moved.

        Both are needed, and the second is what makes the first mean anything. A hand-to-handle gap
        is only a statement about holding the tray if the handle is where the tray actually is; a
        gap measured against a handle position captured at t=0 will read zero while the tray leaves
        the world. So the tray's own displacement is reported next to it, and the gate on the tray
        is what turns "the solver converged" into "the tray was carried".
        """
        out = {}
        for side, target in (self.handles or {}).items():
            out[side] = float(np.linalg.norm(self.arm_ik.hand(side, q_cmd) - target))
        self.hand_gap_m = out
        if self.tray_start is not None:
            # reuse the handles THIS STEP already read; a second read would be the same numbers at
            # the same instant, and doing it twice made a 30 s run take over seven minutes
            fresh = self.handles_read_step or self.handles or {}
            self.tray_shift_m = max(
                (float(np.linalg.norm(fresh[s] - self.tray_start[s])) for s in fresh),
                default=0.0)
        return out

    def sample(self, *a, **k):
        s = super().sample(*a, **k)
        s['hand_gap_m'] = {side: round(v, 6) for side, v in self.hand_gap_m.items()}
        s['hand_gap_max_m'] = round(max(self.hand_gap_m.values()), 6) if self.hand_gap_m else None
        return s


def run(rig, depth, dx, *, correct, kp_bal, kd_bal, ramp_s=2.0, hold_s=1.5, record=0.05,
        hand_gap_max=HAND_GAP_MAX_M):
    """One crouch with the arms solving for the tray. Returns the trace plus its own bookkeeping."""
    rig.reset()
    rig.settle(1.0)
    rig.handles = rig.read_handles()
    # where the tray was when the run began -- the reference the tray-shift gate is measured against
    rig.tray_start = {k: np.array(v) for k, v in rig.handles.items()}
    rig.tray_shift_m = 0.0
    standing_z = float(rig.d.xpos[rig.m.body(PB.BASE).id][2])
    n_ramp = int(ramp_s / rig.m.opt.timestep)
    n_hold = int(hold_s / rig.m.opt.timestep)
    every = max(1, int(record / rig.m.opt.timestep))
    corr, rejects, targets, trace = 0.0, 0, [], []
    ungated, worst_gap = 0, 0.0
    worst_tray_shift = 0.0
    q_prev = np.array(rig.d.qpos, dtype=float)
    rig.prev_com, rig.prev_t = rig.humanoid_com(), float(rig.d.time)
    # ★ PER-RUN, not per-rig. `arm_ik_clamps` lives on the rig and is reused across depths, so a
    # sweep that does not reset it reports the CUMULATIVE total in every later row -- measured: the
    # 0.35 and 0.40 rows both read 13133, which is one number printed twice, not two measurements.
    rig.arm_ik_clamps = 0
    for i in range(n_ramp + n_hold):
        frac = min(1.0, (i + 1) / n_ramp)
        phase = 'ramp' if i < n_ramp else 'hold'
        commanded_z = float(rig.pelvis0[2]) - depth * frac
        measured_z = float(rig.d.xpos[rig.m.body(PB.BASE).id][2])
        if correct:
            corr = float(np.clip(C2.K_TASK * (commanded_z - measured_z),
                                 -C2.TASK_LIMIT_M, C2.TASK_LIMIT_M))
        eff_depth = max(0.0, depth * frac - corr) if correct else depth * frac
        target = np.array([rig.pelvis0[0] + C2.dx_for(eff_depth), rig.pelvis0[1],
                           float(rig.pelvis0[2]) - eff_depth], dtype=float)
        # ★ WARM START, and it is worth ~7x. Seeding from `rig.d.qpos` (the SIMULATED state, which
        # lags the commanded posture by a servo error) made the arm IK re-converge from scratch
        # every step: measured 37 ms/step in the full loop against 5.4 ms/step when seeded from the
        # previous solution, for 1750 steps. `probe_p4_reach.ik` warm-starts for the same reason --
        # its own comment records that cold starts land in local minima whose residual EXCEEDS the
        # start distance, which reads as "unreachable" for a target the arm can plainly reach.
        seed = np.array(q_prev)
        q_try, resid, _marg = rig.solve_posture(tuple(target), seed)
        if max(resid.values()) > PB.IK_ACCEPT_M:
            rejects += 1
            q_try = q_prev
        else:
            q_prev = q_try
        targets.append(float(corr))
        # ★ THE GATE. Both hands, the whole run, in the posture that will be commanded.
        gaps = rig.measure_gap(q_try)
        gap = max(gaps.values())
        worst_gap = max(worst_gap, gap)
        worst_tray_shift = max(worst_tray_shift, rig.tray_shift_m)
        if gap > hand_gap_max:
            ungated += 1
        com_ref = float(rig.posture_com(q_try)[0])
        com_vx = rig.com_x_velocity()
        rig.prev_com, rig.prev_t = rig.humanoid_com(), float(rig.d.time)
        rig.tick(q_try, balance=True, kp_bal=kp_bal, kd_bal=kd_bal, com_ref_x=com_ref,
                 com_vx=com_vx, gravity_ff=True)
        if i % every == 0 or i == n_ramp + n_hold - 1:
            tr = rig.sample(standing_z, -depth, com_ref, phase)
            tr['correction_m'] = round(corr, 5)
            tr['eff_depth_m'] = round(eff_depth, 5)
            tr['cmd_z'] = round(commanded_z, 5)
            tr['cmd_err_m'] = round(measured_z - commanded_z, 5)
            trace.append(tr)
    return {'trace': trace, 'standing_z': standing_z, 'rejects': rejects,
            'max_correction': max(abs(v) for v in targets),
            'ungated_steps': ungated, 'worst_gap_m': worst_gap,
            'worst_tray_shift_m': worst_tray_shift,
            'steps': n_ramp + n_hold,
            'arm_ik_clamps': rig.arm_ik_clamps}


def judge(r, depth, thresholds, hand_gap_max=HAND_GAP_MAX_M):
    tr = r['trace']
    hold = [s for s in tr if s['phase'] == 'hold']
    end = tr[-1]
    ratios = [s['load_over_weight'] for s in hold]
    self_all = [s['self_mm'] for s in tr]
    self_hold = [s['self_mm'] for s in hold]
    rows = {
        'arrives': abs(end['drop_m'] - depth) <= thresholds['crouch_tol_m'],
        'sole_flat': max(max(s['foot_pitch_deg'].values()) for s in tr)
        <= thresholds['foot_flat_max_deg'],
        # ★ the review's gate: EVERY step, not just the settled one
        'tray_held': r['worst_gap_m'] <= hand_gap_max,
        # ★ and the tray must STAY where it was. Without this row the gap above is measured against
        # a handle position that can be stale, and a stale reference makes the gate unfailable --
        # measured once: reported gap 1e-6 m while the tray was 0.84 m away.
        'tray_in_place': r['worst_tray_shift_m'] <= TRAY_SHIFT_MAX_M,
        'feet_loaded': min(min(s['load_N']['L'], s['load_N']['R']) / PB.WEIGHT_N
                           for s in hold) >= thresholds['min_foot_share'],
        'com_inside': min(min(s['com_margin_m']) for s in tr) >= thresholds['min_com_margin_m'],
        'com_tracks': max(abs(s['com_err_m']) for s in hold) <= thresholds['com_track_tol_m'],
        'settled': end['leg_vel'] <= thresholds['settled_vel_rad_s'],
        'torque_in_range': max(s['torque_usage'] for s in hold) <= 0.95,
        'load_adds_up': (min(ratios) >= thresholds['load_sum_band'][0]
                         and max(ratios) <= thresholds['load_sum_band'][1]),
        'no_self_contact': min(self_hold) >= PB.SELF_ALLOWANCE_MM,
        'ik_converged': r['rejects'] == 0,
    }
    return {'dz_m': depth,
            'drop_m': end['drop_m'], 'drop_err_m': end['drop_m'] - depth,
            'cmd_err_m': end['cmd_err_m'], 'correction_m': end['correction_m'],
            'ik_rejects': r['rejects'],
            'worst_foot_pitch_deg': max(max(s['foot_pitch_deg'].values()) for s in tr),
            'worst_gap_m': r['worst_gap_m'], 'ungated_steps': r['ungated_steps'],
            'worst_tray_shift_m': r['worst_tray_shift_m'],
            'steps': r['steps'],
            'ankle_Nm_end': end['ankle_Nm'],
            'peak_torque_usage': max(s['torque_usage'] for s in hold),
            'arm_ik_clamps': r['arm_ik_clamps'],
            'rows': {k: bool(v) for k, v in rows.items()}, 'holds': bool(all(rows.values()))}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--run-id', default='p4-armik-01')
    ap.add_argument('--reports-dir', default=None)
    ap.add_argument('--depths', default=None,
                    help='comma-separated; default is the frozen DEPTHS')
    args = ap.parse_args()
    out = (Path(args.reports_dir) if args.reports_dir else ROOT / 'reports') / args.run_id
    out.mkdir(parents=True, exist_ok=True)

    ref = json.loads((ROOT / 'reports' / 'p4-balance-04' / 'report.json').read_text(encoding='utf-8'))
    kp_bal, kd_bal = ref['controller']['kp_bal'], ref['controller']['kd_bal']

    rig = GripRig()
    rig.capture_standing()
    print('balance gains from p4-balance-04: kp %.3f kd %.3f' % (kp_bal, kd_bal))

    depths = ([float(x) for x in args.depths.split(',')] if args.depths else list(C2.DEPTHS))
    thresholds = {'crouch_tol_m': PB.CROUCH_TOL_M, 'foot_flat_max_deg': PB.FOOT_FLAT_MAX_DEG,
                  'min_foot_share': PB.MIN_FOOT_SHARE, 'min_com_margin_m': PB.MIN_COM_MARGIN_M,
                  'com_track_tol_m': PB.COM_TRACK_TOL_M, 'settled_vel_rad_s': PB.SETTLED_VEL_RAD_S,
                  'load_sum_band': list(PB.LOAD_SUM_BAND)}

    print('\n  depth | holds | arrival err | corr | pitch  | gap worst | ankle end | torque | fails')
    print('  ------+-------+-------------+------+--------+-----------+-----------+--------+------')
    sweep = []
    for depth in depths:
        r = run(rig, depth, C2.dx_for(depth), correct=True, kp_bal=kp_bal, kd_bal=kd_bal)
        e = judge(r, depth, thresholds)
        sweep.append(e)
        print('   %.2f | %5s | %+8.1f mm | %+4.0f | %5.2f° | %7.4f m | %9s | %4.0f%% | %s' %
              (depth, e['holds'], e['drop_err_m'] * 1000, e['correction_m'] * 1000,
               e['worst_foot_pitch_deg'], e['worst_gap_m'],
               '/'.join('%.1f' % v for v in e['ankle_Nm_end'].values()),
               e['peak_torque_usage'] * 100,
               [k for k, v in e['rows'].items() if not v] or 'none'))

    held = [e['dz_m'] for e in sweep if e['holds']]
    d_max = max(held) if held else None
    print('\n*** D_MAX with the arms solving for the tray = %s m' % d_max)

    # ★ FALSIFIABILITY. A gate that has only ever returned PASS is not evidence. The control arm
    # runs the SAME controller with the arms frozen at the carry pose (the previous behaviour), at
    # the deepest passing depth -- if the gate cannot fail there, it is not measuring the arms.
    control = None
    if d_max is not None:

        class Frozen(GripRig):
            """The PREVIOUS behaviour: legs solved, arms welded to a pose. The gate's control."""

            frozen_pose = None

            def solve_posture(self, pelvis, seed):
                q, resid, margins = PB.Rig.solve_posture(self, pelvis, seed)
                if self.frozen_pose is not None:
                    for name, angle in self.frozen_pose.items():
                        jid = self.m.joint('h_' + name).id
                        q[int(self.m.jnt_qposadr[jid])] = float(angle)
                return q, resid, margins

        f = Frozen()
        f.capture_standing()
        carrier = F2.FFRig()
        carrier.capture_standing()
        frozen_pose, _per = F2.carry_pose_for(carrier)
        f.frozen_pose = frozen_pose
        rc = run(f, d_max, C2.dx_for(d_max), correct=True, kp_bal=kp_bal, kd_bal=kd_bal)
        ec = judge(rc, d_max, thresholds)
        control = ec
        print('\n  CONTROL (arms frozen at the carry pose) at %.2f m: gap %.4f m, '
              'ungated %d/%d steps, tray_held=%s' %
              (d_max, ec['worst_gap_m'], ec['ungated_steps'], ec['steps'],
               ec['rows']['tray_held']))
        if ec['rows']['tray_held']:
            print('  ⚠ THE GATE DID NOT FAIL on the arm-locked control: it is not measuring the '
                  'arms, and this run proves nothing about holding the tray.')

    report = {'run_id': args.run_id,
              'scope': 'the arms SOLVE for the tray handle instead of being welded to a pose',
              'hand_gap_max_m': HAND_GAP_MAX_M,
              'controller': {'k_task': C2.K_TASK, 'task_limit_m': C2.TASK_LIMIT_M,
                             'kp_bal': kp_bal, 'kd_bal': kd_bal,
                             'arm_ik': 'damped least squares, own MjData, 2 passes/side',
                             'arm_ik_iters_cap': 300, 'arm_joints': list(PR.ARM_JOINTS)},
              'sweep': sweep, 'control_frozen_arms': control,
              'd_max_m': d_max}
    (out / 'report.json').write_text(json.dumps(report, indent=2, ensure_ascii=False,
                                                default=str), encoding='utf-8')
    print('\nwrote %s' % (out / 'report.json'))


if __name__ == '__main__':
    main()
