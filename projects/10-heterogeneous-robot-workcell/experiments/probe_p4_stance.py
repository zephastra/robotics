"""P4 step 2d: static stance torque in JOINT SPACE (hip pitch + knee), the ankle channel untouched.

WHY THIS EXISTS -- the measurement that motivated it
----------------------------------------------------
The proposal is to stop asking the ankle to do two jobs at once. Measured (`experiments/_dbg_p4_stance.py`),
the two jobs are not merely concurrent, they are OPPOSED:

    depth   ankle total    = PD       + qfrc_bias + balance term
    0.20 m  +1.974 N.m     = +38.035  + 0.002     - 36.063
    0.25 m  -25.871 N.m    = +14.695  - 0.006     - 40.560
    0.30 m  -20.464 N.m    = +12.543  + 0.001     - 33.008

The balance term drives the ankle at `kp_bal = 2537 N.m/m`, so a 14 mm CoM error asks for 36 N.m and
the posture PD -- whose job is to hold the flat sole -- answers with +38 N.m. The NET is small, which
is why the pose looks fine, but ~+/-37 N.m of antagonistic torque is flowing through one actuator, and
whatever asymmetry appears (0.30 m: PD +12.5 left against +5.6 right) breaks the cancellation and
ROLLS the foot. That is the `sole_flat` failure mode this step is about.

TWO PREMISES OF THE PROPOSAL, CHECKED BEFORE BUILDING ON THEM
-------------------------------------------------------------
1. "Let `sole_flat` be a pure kinematic rigid constraint." **ALREADY TRUE.** `solve_posture` drives a
   SIX-dimensional task -- foot position AND orientation -- on a 6-joint chain: measured IK target
   error **0.0000 mm position / 0.0000 deg rotation**, residual 0.0, commanded sole up-vector 0.093
   deg off vertical, at every depth tried. So the 3.87 deg of physical foot pitch in
   `reports/p4-ff2-01` is NOT a kinematics gap; the foot is pulled off an exactly flat target by its
   own control channel. Half the proposal is work already done, and the half that matters is the
   other half.
2. "Use a quasi-static rigid-body projection instead of a contact-Jacobian projection." **Right in
   intent** -- D094's contact projection came out asymmetric (L -29.99 against R +15.96 N.m). But the
   closed-form sagittal recursion I first wrote does NOT reproduce either reference, so it is
   REPORTED AND NOT USED; see `sagittal_tau` and the instrument table.

WHAT IS USED
------------
`qfrc_inverse` at the COMMANDED configuration with `qacc = 0` -- joint-space inverse dynamics on the
quasi-static stance, which is exactly "the torque each joint needs to carry the body" and involves no
contact Jacobian and no measured contact force. Two guards, both reported rather than silent:
  * a joint whose stance torque exceeds `GUARD_FRACTION` of its own actuator range is REJECTED and
    counted -- a 0.25 m crouch makes raw `mj_inverse` ask for -633 N.m of hip pitch against a +/-415
    N.m actuator, which is the `reports/p4-ff-01` blow-up family;
  * the applied set is hip pitch and knee ONLY. The ankle keeps its own channel, so this feed-forward
    cannot double-count the ankle strategy the way D094's projection did.
The SIGN is measured by an A/B arm, not taken from the derivation.
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

bp3.install()
import probe_p4_balance as PB  # noqa: E402
import probe_p4_crouch2 as C2  # noqa: E402
import probe_p4_ff2 as F2  # noqa: E402

WORLD = ROOT / 'assets' / 'world_p4_handover.xml'
#: Applied joints: SAGITTAL only. These are what place the fore/aft CoM; the ankle is deliberately
#: left out so its channel keeps only the two jobs it is good at -- hold the sole flat, carry the
#: static reaction.
APPLY = ('J00_HIP_PITCH_L', 'J03_KNEE_PITCH_L', 'J06_HIP_PITCH_R', 'J09_KNEE_PITCH_R')
REPORT_ONLY = ('J04_ANKLE_PITCH_L', 'J10_ANKLE_PITCH_R')
FEET = PB.FOOT
#: A stance torque above this share of the joint's own actuator range is not a stance torque.
GUARD_FRACTION = 0.60


def humanoid_mass(m):
    """The mass of the HUMANOID -- the subtree hanging off `h_LINK_BASE`. Not the scene's.

    `np.sum(m.body_mass)` is 251.16 kg here, the two vehicles, the table, the roller bands and the
    trays included; using it made F_z 2.9x too large in the first version of this file.
    """
    return float(m.body_subtreemass[m.body(PB.BASE).id])


def sagittal_tau(rig, q_cmd, *, share=0.5):
    """REPORTED, NOT USED. A closed-form per-leg sagittal statics recursion.

        tau_j = -(subtree_mass(j)*g*(subtree_com_x(j) - x_anchor(j))) + (x_foot - x_anchor(j))*F_z

    Symmetric between the legs by construction (measured gap 0.012 N.m against the contact
    projection's 30.4). It is NOT shipped because it does not reproduce either reference at the
    standing pose: the ankle comes out +0.36 N.m where the torque actually holding the robot is
    +12.35 N.m and `mj_inverse` says +13.68. The weak link is the DECLARATION of where the ground
    reaction acts -- the foot body's own subtree CoM sits almost under the ankle, so the lever arm
    collapses. Recorded because "I tried the closed form and it disagreed" is worth more than a
    silent choice.
    """
    m, sc = rig.m, rig.stance_data
    sc.qpos[:] = q_cmd
    sc.qvel[:] = 0.0
    mujoco.mj_forward(m, sc)
    g = abs(float(m.opt.gravity[2]))
    fz = share * humanoid_mass(m) * g
    out = {}
    for side in ('L', 'R'):
        x_c = float(sc.subtree_com[m.body(FEET[side]).id][0])
        for n in PB.LEG_CHAIN[side]:
            jid = m.joint('h_' + n).id
            bid = int(m.jnt_bodyid[jid])
            sub_m = float(m.body_subtreemass[bid])
            sub_x = float(sc.subtree_com[bid][0])
            xj = float(sc.xanchor[jid][0])
            out[n] = -(sub_m * g * (sub_x - xj)) - (-(x_c - xj) * fz)
    return out


def _actuator_span(rig, dof):
    span = 0.0
    for a in rig.dof_actuators.get(dof, ()):
        span = max(span, abs(float(rig.m.actuator_ctrlrange[a][1])))
    return span


def stance_tau(rig, q_cmd):
    """`qfrc_inverse` at the COMMANDED configuration with qacc = 0, with a range guard.

    Returns (dict joint -> N.m for the whole leg chain, list of rejected joint names).
    """
    m, sc = rig.m, rig.stance_data
    sc.qpos[:] = q_cmd
    sc.qvel[:] = 0.0
    sc.ctrl[:] = np.zeros(m.nu)
    mujoco.mj_forward(m, sc)
    sc.qacc[:] = 0.0
    mujoco.mj_inverse(m, sc)
    inv = np.array(sc.qfrc_inverse, dtype=float)
    out, rejected = {}, []
    for side in ('L', 'R'):
        for n in PB.LEG_CHAIN[side]:
            dof = int(m.jnt_dofadr[m.joint('h_' + n).id])
            val = float(inv[dof])
            span = _actuator_span(rig, dof)
            if span > 0 and abs(val) > GUARD_FRACTION * span:
                rejected.append(n)
                continue
            out[n] = val
    return out, rejected


class StanceRig(F2.FFRig):
    """The crouch rig with a joint-space stance feed-forward on hip pitch and knee."""

    def __init__(self, world=WORLD):
        super().__init__(world)
        self.q_cmd = None
        self.stance_all, self.stance_applied = {}, {}
        self.stance_sym_gap, self.stance_rejected = 0.0, []
        self.stance_data = mujoco.MjData(self.m)
        self.ff_sign = +1.0

    def solve_posture(self, pelvis, seed):
        q, resid, marg = super().solve_posture(pelvis, seed)
        self.q_cmd = np.array(q, dtype=float)
        return q, resid, marg

    def contact_ff(self):
        """The feed-forward: a joint-space static stance torque, not a contact projection."""
        if self.ff_mode != 'stance' or self.q_cmd is None:
            return {}
        t, rejected = stance_tau(self, self.q_cmd)
        self.stance_rejected = list(rejected)
        self.stance_all = {k: round(float(v), 4) for k, v in t.items()}
        # the instrument's OWN symmetry gap: by construction it should be ~0, unlike D094's 30.4.
        self.stance_sym_gap = max((abs(t[PB.LEG_CHAIN['L'][i]] - t[PB.LEG_CHAIN['R'][i]])
                                   for i in range(6) if PB.LEG_CHAIN['L'][i] in t
                                   and PB.LEG_CHAIN['R'][i] in t), default=0.0)
        applied = {n: self.ff_sign * v for n, v in t.items() if n in APPLY}
        self.stance_applied = {k: round(float(v), 4) for k, v in applied.items()}
        return {int(self.m.jnt_dofadr[self.m.joint('h_' + n).id]): float(v)
                for n, v in applied.items()}

    def sample(self, *a, **k):
        s = super().sample(*a, **k)
        s['stance_applied_Nm'] = dict(self.stance_applied)
        return s


def judge(run, depth, thresholds):
    e = F2.judge(run, depth, thresholds)
    hold = [s for s in run['trace'] if s['phase'] == 'hold']
    e['worst_com_err_mm'] = max(abs(s['com_err_m']) for s in hold) * 1000
    return e


def do_run(rig, depth, *, ff_mode, arms, kp_bal, kd_bal, sign=1.0, ramp_s=2.0, hold_s=1.5,
           balance_on=True):
    rig.ff_mode = ff_mode
    rig.ff_sign = float(sign)
    rig.balance_on = balance_on
    rig.ff_clamp_hits, rig.ff_peak_fraction = 0, 0.0
    rig.carry_pose = {'carry': rig.pose_locked, 'home': rig.pose_home, 'none': None}[arms]
    out = C2.run(rig, depth, C2.dx_for(depth), correct=True, kp_bal=kp_bal, kd_bal=kd_bal,
                 ramp_s=ramp_s, hold_s=hold_s)
    out.update(ff_mode=ff_mode, arms=arms, balance_on=balance_on, sign=float(sign))
    out['ff_clamp_hits'], out['ff_peak_fraction'] = rig.ff_clamp_hits, rig.ff_peak_fraction
    out['instrument'] = dict(rig.instrument)
    out['stance_rejected'] = list(rig.stance_rejected)
    out['stance_sym_gap_Nm'] = float(rig.stance_sym_gap)
    return out


def instrument_check(rig):
    """Three estimates of the same quantity at the SETTLED STANDING pose, plus the reference.

    A settled robot is in static equilibrium, so the torque holding it IS the torque its actuators
    apply there -- measured: knee -4.265 N.m, ankle +12.350 N.m. That is the reference. Everything
    else is compared against it, and every disagreement is reported rather than resolved silently.
    """
    q_stand = np.array(rig.d.qpos, dtype=float)
    inv, rejected = stance_tau(rig, q_stand)
    sag = sagittal_tau(rig, q_stand)
    applied = {}
    for name in rig.names:
        jid = rig.m.joint('h_' + name).id
        if jid < 0:
            continue
        applied[name] = sum(float(rig.d.ctrl[a]) for a in rig.index.get(int(jid), ()))
    cmp = {}
    for n in ('J00_HIP_PITCH_L', 'J03_KNEE_PITCH_L', 'J04_ANKLE_PITCH_L', 'J09_KNEE_PITCH_R'):
        cmp[n] = {'qfrc_inverse_Nm': round(float(inv.get(n, float('nan'))), 4),
                  'sagittal_closed_form_Nm': round(float(sag[n]), 4),
                  'applied_Nm': round(float(applied.get(n, float('nan'))), 4)}
    fit = [abs(cmp[n]['qfrc_inverse_Nm'] - cmp[n]['applied_Nm'])
           for n in ('J00_HIP_PITCH_L', 'J03_KNEE_PITCH_L') if n in inv]
    gap = max((abs(inv[PB.LEG_CHAIN['L'][i]] - inv[PB.LEG_CHAIN['R'][i]])
               for i in range(6) if PB.LEG_CHAIN['L'][i] in inv
               and PB.LEG_CHAIN['R'][i] in inv), default=0.0)
    return {'compare': cmp, 'rejected_at_standing': rejected,
            'symmetry_gap_Nm': round(float(gap), 6),
            'worst_sagittal_vs_applied_Nm': round(float(max(fit)), 4),
            'weight_N': humanoid_mass(rig.m) * abs(float(rig.m.opt.gravity[2])),
            'humanoid_mass_kg': humanoid_mass(rig.m)}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--run-id', default='p4-stance-01')
    ap.add_argument('--reports-dir', default=None)
    args = ap.parse_args()
    out = (Path(args.reports_dir) if args.reports_dir else ROOT / 'reports') / args.run_id
    out.mkdir(parents=True, exist_ok=True)

    ref = json.loads((ROOT / 'reports' / 'p4-balance-04' / 'report.json').read_text(encoding='utf-8'))
    kp_bal, kd_bal = ref['controller']['kp_bal'], ref['controller']['kd_bal']
    rig = StanceRig()
    rig.capture_standing()
    pose, per_arm = F2.carry_pose_for(rig)
    rig.pose_locked = pose
    import probe_p4_reach as PR  # noqa: E402
    rig.pose_home = {n: float(rig.target[k]) for k, n in enumerate(rig.names) if n in PR.ARM_JOINTS}

    inst = instrument_check(rig)
    print('stance-torque instrument, at the SETTLED STANDING pose:')
    print(f"   {'joint':>20} {'qfrc_inverse':>13} {'sagittal':>10} {'applied':>10}")
    for n, v in inst['compare'].items():
        print(f"   {n:>20} {v['qfrc_inverse_Nm']:13.3f} {v['sagittal_closed_form_Nm']:10.3f} "
              f"{v['applied_Nm']:10.3f}")
    print(f"   humanoid {inst['humanoid_mass_kg']:.3f} kg -> {inst['weight_N']:.2f} N; "
          f"L-R symmetry gap {inst['symmetry_gap_Nm']:.6f} N.m (D094's projection had 30.4); "
          f"rejected at standing {inst['rejected_at_standing']}")
    print(f"   worst sagittal |qfrc_inverse - applied| = "
          f"{inst['worst_sagittal_vs_applied_Nm']:.4f} N.m")

    thresholds = {'crouch_tol_m': PB.CROUCH_TOL_M, 'foot_flat_max_deg': PB.FOOT_FLAT_MAX_DEG,
                  'min_foot_share': PB.MIN_FOOT_SHARE, 'min_com_margin_m': PB.MIN_COM_MARGIN_M,
                  'com_track_tol_m': PB.COM_TRACK_TOL_M, 'settled_vel_rad_s': PB.SETTLED_VEL_RAD_S,
                  'load_sum_band': list(PB.LOAD_SUM_BAND)}

    ab, sweep = [], []
    for ff_mode, sign in (('off', 1.0), ('stance', +1.0), ('stance', -1.0)):
        r = do_run(rig, 0.25, ff_mode=ff_mode, arms='carry', kp_bal=kp_bal, kd_bal=kd_bal, sign=sign)
        e = judge(r, 0.25, thresholds)
        e['sign'] = sign
        ab.append(e)
        print(f'  A/B 0.25 ff={ff_mode:>6} sign={sign:+.0f}  arr {e["drop_err_m"] * 1000:+7.1f}mm '
              f'pitch {e["worst_foot_pitch_deg"]:6.2f}  com {e["worst_com_err_mm"]:6.2f}mm '
              f'ank {max(abs(v) for v in e["ankle_Nm_end"].values()):7.2f}  '
              f'rows {sum(e["rows"].values())}/10  '
              f'failed={[k for k, v in e["rows"].items() if not v] or "none"}')
    best = max(ab, key=lambda e: (sum(e['rows'].values()), -e['worst_com_err_mm']))
    print(f'\n  selected arm: ff={best["ff_mode"]} sign={best["sign"]:+.0f} '
          f'({sum(best["rows"].values())}/10 rows, CoM err {best["worst_com_err_mm"]:.2f} mm)')

    for depth in C2.DEPTHS:
        r = do_run(rig, depth, ff_mode=best['ff_mode'], arms='carry', kp_bal=kp_bal, kd_bal=kd_bal,
                   sign=best['sign'])
        e = judge(r, depth, thresholds)
        e['sign'] = best['sign']
        e['stance_rejected'] = r['stance_rejected']
        sweep.append(e)
        print(f'  {depth:.2f} holds={str(e["holds"]):>5} arr {e["drop_err_m"] * 1000:+7.1f}mm '
              f'pitch {e["worst_foot_pitch_deg"]:6.2f} com {e["worst_com_err_mm"]:6.2f}mm '
              f'ank {max(abs(v) for v in e["ankle_Nm_end"].values()):7.2f} '
              f'torque {e["peak_torque_usage"] * 100:5.1f}% '
              f'rej {len(e["stance_rejected"])} '
              f'failed={[k for k, v in e["rows"].items() if not v] or "none"}')
    held = [e['dz_m'] for e in sweep if e['holds']]
    print(f'\n*** D_MAX with the joint-space stance feed-forward = {max(held) if held else None} m '
          f'(was 0.20 m)')

    report = {'run_id': args.run_id,
              'scope': 'static stance torque in joint space (hip pitch + knee), ankle untouched',
              'instrument': inst, 'apply_joints': list(APPLY),
              'report_only_joints': list(REPORT_ONLY),
              'guard': f'reject any joint whose |stance torque| > {GUARD_FRACTION} of its own '
                       f'actuator range',
              'closed_form_note': 'sagittal_tau is computed and reported but NOT applied: it does '
                                  'not reproduce either reference at the standing pose',
              'world': {'path': 'assets/world_p4_handover.xml',
                        'sha256': hashlib.sha256(
                            (ROOT / 'assets' / 'world_p4_handover.xml').read_bytes()).hexdigest()},
              'controller': {'k_task': C2.K_TASK, 'task_limit_m': C2.TASK_LIMIT_M,
                             'kp_bal': kp_bal, 'kd_bal': kd_bal},
              'thresholds': thresholds, 'ab': ab, 'sweep': sweep,
              'selected_arm': {'ff_mode': best['ff_mode'], 'sign': best['sign'],
                               'rule': 'most rows at 0.25 m, ties by smaller CoM error'},
              'deepest_holding_depth_m': (max(held) if held else None),
              'previous_deepest_holding_depth_m': 0.20}
    (out / 'report.json').write_text(json.dumps(report, indent=2, default=str) + '\n',
                                     encoding='utf-8')
    lines = [f'# {args.run_id} — joint-space stance torque (hip pitch + knee), ankle untouched', '',
             f"**deepest holding depth {max(held) if held else None} m** (was 0.20 m)", '',
             '## the instrument at the settled standing pose', '',
             'A settled robot is in static equilibrium, so the torque holding it IS the torque its '
             'actuators apply. Everything else is compared against that.', '',
             '| joint | `qfrc_inverse` | sagittal closed form | applied (reference) |',
             '|---|---|---|---|']
    for n, v in inst['compare'].items():
        lines.append(f"| `{n}` | {v['qfrc_inverse_Nm']:+.4f} | "
                     f"{v['sagittal_closed_form_Nm']:+.4f} | {v['applied_Nm']:+.4f} |")
    lines += ['', f"leg-to-leg symmetry gap **{inst['symmetry_gap_Nm']:.6f} N·m** — the contact "
                  f"projection of D094 had **30.4**", '',
              '## A/B (the sign is measured, not taken from the derivation)', '',
              '| dz | ff | sign | arrival err | foot pitch | CoM err | ankle | rows | failed |',
              '|---|---|---|---|---|---|---|---|---|']
    for e in ab:
        lines.append(f"| {e['dz_m']:.2f} | {e['ff_mode']} | {e['sign']:+.0f} | "
                     f"{e['drop_err_m'] * 1000:+.1f} mm | {e['worst_foot_pitch_deg']:.2f}° | "
                     f"{e['worst_com_err_mm']:.2f} mm | "
                     f"{max(abs(v) for v in e['ankle_Nm_end'].values()):.2f} | "
                     f"{sum(e['rows'].values())}/10 | "
                     f"{', '.join(k for k, v in e['rows'].items() if not v) or '—'} |")
    lines += ['', '## depth sweep', '',
              '| dz | holds | arrival err | pitch | CoM err | torque | guard rejects | failed |',
              '|---|---|---|---|---|---|---|---|']
    for e in sweep:
        lines.append(f"| {e['dz_m']:.2f} | {e['holds']} | {e['drop_err_m'] * 1000:+.1f} mm | "
                     f"{e['worst_foot_pitch_deg']:.2f}° | {e['worst_com_err_mm']:.2f} mm | "
                     f"{e['peak_torque_usage'] * 100:.0f}% | {len(e.get('stance_rejected', []))} | "
                     f"{', '.join(k for k, v in e['rows'].items() if not v) or '—'} |")
    (out / 'acceptance.md').write_text('\n'.join(lines) + '\n', encoding='utf-8')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
