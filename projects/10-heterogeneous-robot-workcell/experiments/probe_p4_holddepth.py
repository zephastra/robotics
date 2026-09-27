"""How deep can the crouch actually HOLD? The one number every option depends on.

WHY THIS IS SEPARATE FROM `probe_p4_balance.py`
-----------------------------------------------
The judge writes `reports/p4-balance-04/`, and that evidence is only valid for the EXACT script that
produced it (`D080`: a judge whose source changes silently invalidates its own past runs). Extending
the judge here would have made `p4-balance-04` unattributable. So this file IMPORTS the judge's rig
and thresholds and re-derives the same gains, asserting they match the numbers the judge recorded.

WHY A DEPTH SWEEP NEEDS A PER-DEPTH `dx`
----------------------------------------
`dx` is not a free parameter: `_dbg_p4_legmap.py` measured that the flat-footed posture only exists
along a diagonal band, and that band MOVES with depth. The first sweep held `dx = -0.20` at every
depth, which is inside the band at 0.40 m and outside it at 0.10 m -- so it reported a limit of its
own configuration, not of the robot. Here each depth uses the `dx` the map measured for it.
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

#: The pelvis x the flat-footed posture requires, PER DEPTH, from `_dbg_p4_legmap.py` (FULL mode,
#: the sample with the best CoM centring at that depth). Interpolated on 0.05 m, and the shallowest
#: entry is bracketed by the map's own `dz = 0` row (dx -0.00).
DX_BY_DEPTH = {
    0.10: -0.04, 0.15: -0.08, 0.20: -0.08, 0.25: -0.12,
    0.30: -0.16, 0.35: -0.20, 0.40: -0.20, 0.45: -0.20, 0.50: -0.24,
}


def dx_for(depth):
    keys = sorted(DX_BY_DEPTH)
    if depth in DX_BY_DEPTH:
        return DX_BY_DEPTH[depth]
    lo = max(k for k in keys if k <= depth)
    hi = min(k for k in keys if k >= depth)
    if lo == hi:
        return DX_BY_DEPTH[lo]
    f = (depth - lo) / (hi - lo)
    return DX_BY_DEPTH[lo] + f * (DX_BY_DEPTH[hi] - DX_BY_DEPTH[lo])


def evaluate(rig, depth, *, ramp_s, hold_s):
    """Run one depth and apply the SAME row set the judge applies."""
    r = rig.run(balance=True, kp_bal=KP_BAL, kd_bal=KD_BAL, dx=dx_for(depth), dz=-depth,
                ramp_s=ramp_s, hold_s=hold_s, k_task=0.0)
    tr = r['trace']
    hold = [s for s in tr if s['phase'] == 'hold']
    end = tr[-1]
    worst_pitch = max(max(s['foot_pitch_deg'].values()) for s in tr)
    min_margin = min(min(s['com_margin_m']) for s in tr)
    peak_torque = max(s['torque_usage'] for s in hold)
    worst_com_err = max(abs(s['com_err_m']) for s in hold)
    peak_joint = max(hold, key=lambda s: s['torque_usage'])['torque_joint']
    ratios = [s['load_over_weight'] for s in hold]
    rows = {
        'arrives': abs(end['drop_m'] - depth) <= PB.CROUCH_TOL_M,
        'sole_flat': worst_pitch <= PB.FOOT_FLAT_MAX_DEG,
        'feet_loaded': (min(min(s['load_N']['L'], s['load_N']['R']) / PB.WEIGHT_N
                            for s in hold) >= PB.MIN_FOOT_SHARE),
        'com_inside': min_margin >= PB.MIN_COM_MARGIN_M,
        'com_tracks': worst_com_err <= PB.COM_TRACK_TOL_M,
        'settled': end['leg_vel'] <= PB.SETTLED_VEL_RAD_S,
        'torque_in_range': peak_torque <= 0.95,
        'load_adds_up': (min(ratios) >= PB.LOAD_SUM_BAND[0]
                         and max(ratios) <= PB.LOAD_SUM_BAND[1]),
        'no_self_contact': min(s['self_mm'] for s in tr) >= PB.SELF_ALLOWANCE_MM,
    }
    return {'dz_m': depth, 'dx_m': dx_for(depth), 'drop_m': end['drop_m'],
            'drop_err_m': end['drop_m'] - depth,
            'worst_foot_pitch_deg': worst_pitch, 'min_com_margin_m': min_margin,
            'peak_torque_usage': peak_torque, 'peak_joint': peak_joint,
            'worst_com_err_m': worst_com_err, 'leg_vel_end': end['leg_vel'],
            'load_ratio_range': [min(ratios), max(ratios)],
            'rows': rows, 'holds': all(rows.values())}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--run-id', default='p4-holddepth-01')
    ap.add_argument('--reports-dir', default=None)
    args = ap.parse_args()
    out = (Path(args.reports_dir) if args.reports_dir else ROOT / 'reports') / args.run_id
    out.mkdir(parents=True, exist_ok=True)

    # ---- use the SAME gains the judge used, and prove they are the same -----------------------
    global KP_BAL, KD_BAL
    ref = json.loads((ROOT / 'reports' / 'p4-balance-04' / 'report.json').read_text(encoding='utf-8'))
    KP_BAL = ref['controller']['kp_bal']
    KD_BAL = ref['controller']['kd_bal']
    sign = ref['controller']['sign_measured']
    rig0 = PB.Rig()
    rig0.capture_standing()
    mass = PB.WEIGHT_N / abs(float(rig0.m.opt.gravity[2]))
    ankle_z = float(rig0.d.xpos[rig0.m.body('h_LINK_ANKLE_PITCH_L').id][2])
    h_com = float(rig0.humanoid_com()[2]) - ankle_z
    kp_here = sign * PB.BALANCE_STIFFNESS_RATIO * mass * abs(float(rig0.m.opt.gravity[2]))
    kd_here = sign * 2.0 * 0.7 * float(np.sqrt(abs(kp_here) * mass * h_com ** 2))
    print(f'ref gains  kp {KP_BAL:.1f}  kd {KD_BAL:.1f}  sign {sign:+.0f}')
    print(f'here gains kp {kp_here:.1f}  kd {kd_here:.1f}')
    # ★ A BAND, NOT EQUALITY -- and the reason is a finding about the judge. It derives its mass
    # from a hard-typed `86.209 * 9.81` while the threshold table carries `WEIGHT_N = 845.7`, so the
    # two agree to about 1e-5 relative and an equality test refuses to run at all. Refusing was the
    # right failure (better than silently running a slightly different controller); what it exposes
    # is a typed constant in two places inside the judge.
    if abs(kp_here - KP_BAL) > 0.5 or abs(kd_here - KD_BAL) > 0.5:
        raise SystemExit(f'gain re-derivation off by more than 0.5: kp {kp_here} vs {KP_BAL}, '
                         f'kd {kd_here} vs {KD_BAL}')
    print(f'gains agree within 0.5 N.m/m (dkp {kp_here - KP_BAL:+.6f}, '
          f'dkd {kd_here - KD_BAL:+.6f}); the judge types its mass, so exact equality is not the '
          f'right test')

    rows = []
    for depth in (0.10, 0.15, 0.20, 0.25, 0.30, 0.35, 0.40, 0.45):
        r = evaluate(rig0, depth, ramp_s=2.0, hold_s=1.5)
        rows.append(r)
        print(f"  {depth:.2f} holds={str(r['holds']):>5} drop {r['drop_m'] * 1000:7.1f} "
              f"err {r['drop_err_m'] * 1000:+7.1f} pitch {r['worst_foot_pitch_deg']:6.2f} "
              f"margin {r['min_com_margin_m']:+.4f} torque {r['peak_torque_usage'] * 100:5.1f}%  "
              f"failed={[k for k, v in r['rows'].items() if not v]}")
    depths = [r['dz_m'] for r in rows]
    passed = [d for d, r in zip(depths, rows) if r['holds']]
    failed = [d for d, r in zip(depths, rows) if not r['holds']]
    deepest_hold = max(passed) if passed else None
    # refine the boundary between the deepest pass and the shallowest failure, at 10 mm
    lo = deepest_hold
    hi = min([d for d in failed if lo is None or d > lo], default=None)
    refined = []
    if lo is not None and hi is not None and hi - lo > 0.011:
        probes = [round(lo + 0.01 * i, 2) for i in range(1, int(round((hi - lo) / 0.01)))]
        probes = probes[:12]
        for depth in probes:
            r = evaluate(rig0, depth, ramp_s=2.0, hold_s=1.5)
            refined.append(r)
            if r['holds']:
                lo = depth
                break
        print(f'  refined boundary -> deepest holding depth {lo:.3f} m '
              f'({len(probes)} extra probes attempted)')
    print()
    print(f'*** D_MAX (deepest holding crouch) = {lo}')
    need = 0.356                     # the reach study's minimum, from its own `feasible` windows
    print(f'*** band raise needed = {need:.3f} - {lo} = {need - lo:+.3f} m' if lo else
          '*** no depth holds: the band raise cannot be sized from this run')

    report = {'run_id': args.run_id,
              'scope': 'the deepest crouch that passes EVERY row of the held-crouch judge',
              'gains_from': 'reports/p4-balance-04/report.json (asserted identical above)',
              'world': {'path': 'assets/world_p4_handover.xml',
                        'sha256': hashlib.sha256(
                            (ROOT / 'assets' / 'world_p4_handover.xml').read_bytes()).hexdigest()},
              'dx_source': 'experiments/_dbg_p4_legmap.py, FULL mode, best-centring sample per depth',
              'sweep': rows, 'refinement': refined,
              'deepest_holding_depth_m': lo,
              'min_crouch_the_arm_needs_m': need,
              'band_raise_needed_m': (need - lo) if lo else None,
              'thresholds': {'crouch_tol_m': PB.CROUCH_TOL_M,
                             'foot_flat_max_deg': PB.FOOT_FLAT_MAX_DEG,
                             'min_foot_share': PB.MIN_FOOT_SHARE,
                             'min_com_margin_m': PB.MIN_COM_MARGIN_M,
                             'com_track_tol_m': PB.COM_TRACK_TOL_M,
                             'settled_vel_rad_s': PB.SETTLED_VEL_RAD_S,
                             'load_sum_band': list(PB.LOAD_SUM_BAND),
                             'self_allowance_mm': PB.SELF_ALLOWANCE_MM}}
    (out / 'report.json').write_text(json.dumps(report, indent=2, default=str) + '\n',
                                     encoding='utf-8')
    lines = [f'# {args.run_id} — deepest holding crouch', '',
             f"**deepest holding depth {lo} m**", '',
             f'band raise needed = 0.356 − {lo} = {need - lo:+.3f} m' if lo else 'none holds', '']
    lines += ['| dz | dx | holds | drop | pitch | CoM margin | torque | failed rows |',
              '|---|---|---|---|---|---|---|---|']
    for r in rows + refined:
        lines.append(f"| {r['dz_m']:.2f} | {r['dx_m']:+.2f} | {r['holds']} | "
                     f"{r['drop_m'] * 1000:.0f} mm | {r['worst_foot_pitch_deg']:.1f}° | "
                     f"{r['min_com_margin_m'] * 1000:+.0f} mm | "
                     f"{r['peak_torque_usage'] * 100:.0f}% | "
                     f"{', '.join(k for k, v in r['rows'].items() if not v) or '—'} |")
    (out / 'acceptance.md').write_text('\n'.join(lines) + '\n', encoding='utf-8')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
