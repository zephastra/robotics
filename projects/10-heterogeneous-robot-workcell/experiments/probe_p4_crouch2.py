"""Close the posture loop on the PELVIS POSE, in z only, with a bounded correction.

THE DEFECT THIS FIXES (measured, `_dbg_p4_overshoot.py`)
--------------------------------------------------------
The crouch arrives ~17 % deeper than commanded, and it is NOT a reference bias (that is +0.008 mm)
and NOT a load-dependent sag (that would grow faster than linearly). It is a PD TRACKING ERROR:
the knee ends up **-0.150 rad (8.6 deg) more flexed than its target**, on both legs, because the
gravity feed-forward knows only the FREE-FLOATING gravity torque while the closed chain through the
feet needs more. The deficit is paid out of the PD's error budget.

WHY Z ONLY, AND WHY BOUNDED
---------------------------
The first attempt at an outer loop diverged: it integrated the full 3-D error with gain 1 and
clamped the correction at 0.25 m, which walked the IK target clear out of the feasible band -- the
leg IK then failed to converge (residual 1346.9 mm) and its "solution" was nonsense joint angles.
Two things are different here:
  * the correction is applied to **z only**, so `dx` stays on the value `_dbg_p4_legmap` measured,
    and the target cannot leave the band sideways;
  * it is clamped to `TASK_LIMIT_M` and any IK that fails to converge is REJECTED (the previous
    posture is held and the rejection is counted), so an unreachable target degrades to "hold what
    I had and say so" rather than to a fabricated posture.
Both arms are run: the corrected controller and the same controller with `k_task = 0`, because a
row that cannot fail says nothing.
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

WORLD = ROOT / 'assets' / 'world_p4_handover.xml'
#: pelvis x per depth -- the map's measurement, not a tuning choice
DX_BY_DEPTH = {0.10: -0.04, 0.15: -0.08, 0.20: -0.08, 0.25: -0.12,
               0.30: -0.16, 0.35: -0.20, 0.40: -0.20}
#: PROPORTIONAL ONLY. The first version was `corr += k * err`, a pure integrator, and with the
#: correction itself clamped the integrator kept accumulating and parked at the clamp: measured
#: `corr = -60.0 mm` at every depth, i.e. a constant offset with extra steps. A P term cannot wind
#: up. Gain 1.0 makes a 44 mm error ask for a 44 mm correction.
K_TASK = 1.0
TASK_LIMIT_M = 0.060
DEPTHS = (0.10, 0.15, 0.20, 0.25, 0.30, 0.35, 0.40)
#: The map's samples, as (depth, dx), INCLUDING the standing point. `_dbg_p4_legmap`'s row at
#: 0.05 m reads "no pose / CoM outside" at every grid sample, which looks like a GAP -- but its `dx`
#: grid was 40 mm coarse, so that row is a GRID ARTEFACT: the runs that ramped through 0.05 m with a
#: fixed `dx` report a worst IK residual of 0.001 mm, i.e. a posture existed all along. Treating that
#: row as a hole is what made the first correction snap to a discrete ladder -- and a snapped target
#: is DISCONTINUOUS in x, which is what rolled the sole to 17.42 deg at every depth.
#:
#: So: interpolate CONTINUOUSLY along the map, and do not snap. `dx` is a smooth function of the
#: target depth, which is what a control step needs.
DX_MAP = ((0.00, 0.00), (0.10, -0.04), (0.15, -0.08), (0.20, -0.08), (0.25, -0.12),
          (0.30, -0.16), (0.35, -0.20), (0.40, -0.20))


def dx_for(depth):
    depth = min(max(depth, DX_MAP[0][0]), DX_MAP[-1][0])
    for (d0, x0), (d1, x1) in zip(DX_MAP, DX_MAP[1:]):
        if d0 <= depth <= d1:
            if d1 == d0:
                return x0
            f = (depth - d0) / (d1 - d0)
            return x0 + f * (x1 - x0)
    return DX_MAP[-1][1]


def run(rig, depth, dx, *, correct, kp_bal, kd_bal, ramp_s=2.0, hold_s=1.5, record=0.05):
    """One crouch. Returns the trace plus the correction's own bookkeeping."""
    rig.reset()
    rig.settle(1.0)
    standing_z = float(rig.d.xpos[rig.m.body(PB.BASE).id][2])
    n_ramp = int(ramp_s / rig.m.opt.timestep)
    n_hold = int(hold_s / rig.m.opt.timestep)
    every = max(1, int(record / rig.m.opt.timestep))
    corr, rejects, targets, trace = 0.0, 0, [], []
    q_prev = np.array(rig.d.qpos, dtype=float)
    rig.prev_com, rig.prev_t = rig.humanoid_com(), float(rig.d.time)
    for i in range(n_ramp + n_hold):
        frac = min(1.0, (i + 1) / n_ramp)
        phase = 'ramp' if i < n_ramp else 'hold'
        commanded_z = float(rig.pelvis0[2]) - depth * frac
        measured_z = float(rig.d.xpos[rig.m.body(PB.BASE).id][2])
        if correct:
            # P only, and it corrects the TARGET DEPTH (so `dx` moves with it, staying on the band)
            corr = float(np.clip(K_TASK * (commanded_z - measured_z),
                                 -TASK_LIMIT_M, TASK_LIMIT_M))
        eff_depth = max(0.0, depth * frac - corr) if correct else depth * frac
        target = np.array([rig.pelvis0[0] + dx_for(eff_depth), rig.pelvis0[1],
                           float(rig.pelvis0[2]) - eff_depth], dtype=float)
        seed = np.array(rig.d.qpos, dtype=float)
        q_try, resid, _marg = rig.solve_posture(tuple(target), seed)
        if max(resid.values()) > PB.IK_ACCEPT_M:
            rejects += 1
            q_try = q_prev
        else:
            q_prev = q_try
        targets.append(float(corr))
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
            'max_correction': max(abs(v) for v in targets)}


def judge(r, depth, thresholds):
    tr, hold = r['trace'], [s for s in r['trace'] if s['phase'] == 'hold']
    end = tr[-1]
    ratios = [s['load_over_weight'] for s in hold]
    rows = {
        'arrives': abs(end['drop_m'] - depth) <= thresholds['crouch_tol_m'],
        'sole_flat': max(max(s['foot_pitch_deg'].values()) for s in tr)
        <= thresholds['foot_flat_max_deg'],
        'feet_loaded': min(min(s['load_N']['L'], s['load_N']['R']) / PB.WEIGHT_N
                           for s in hold) >= thresholds['min_foot_share'],
        'com_inside': min(min(s['com_margin_m']) for s in tr) >= thresholds['min_com_margin_m'],
        'com_tracks': max(abs(s['com_err_m']) for s in hold) <= thresholds['com_track_tol_m'],
        'settled': end['leg_vel'] <= thresholds['settled_vel_rad_s'],
        'torque_in_range': max(s['torque_usage'] for s in hold) <= 0.95,
        'load_adds_up': (min(ratios) >= thresholds['load_sum_band'][0]
                         and max(ratios) <= thresholds['load_sum_band'][1]),
        'no_self_contact': min(s['self_mm'] for s in tr) >= PB.SELF_ALLOWANCE_MM,
        'ik_converged': r['rejects'] == 0,
    }
    return {'dz_m': depth, 'dx_m': dx_for(depth), 'drop_m': end['drop_m'],
            'drop_err_m': end['drop_m'] - depth, 'cmd_err_m': end['cmd_err_m'],
            'correction_m': end['correction_m'], 'ik_rejects': r['rejects'],
            'worst_foot_pitch_deg': max(max(s['foot_pitch_deg'].values()) for s in tr),
            'min_com_margin_m': min(min(s['com_margin_m']) for s in tr),
            'peak_torque_usage': max(s['torque_usage'] for s in hold),
            'leg_vel_end': end['leg_vel'],
            'rows': rows, 'holds': all(rows.values())}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--run-id', default='p4-crouch2-01')
    ap.add_argument('--reports-dir', default=None)
    args = ap.parse_args()
    out = (Path(args.reports_dir) if args.reports_dir else ROOT / 'reports') / args.run_id
    out.mkdir(parents=True, exist_ok=True)

    ref = json.loads((ROOT / 'reports' / 'p4-balance-04' / 'report.json').read_text(encoding='utf-8'))
    kp_bal, kd_bal = ref['controller']['kp_bal'], ref['controller']['kd_bal']
    rig = PB.Rig()
    rig.capture_standing()
    thresholds = {'crouch_tol_m': PB.CROUCH_TOL_M, 'foot_flat_max_deg': PB.FOOT_FLAT_MAX_DEG,
                  'min_foot_share': PB.MIN_FOOT_SHARE, 'min_com_margin_m': PB.MIN_COM_MARGIN_M,
                  'com_track_tol_m': PB.COM_TRACK_TOL_M, 'settled_vel_rad_s': PB.SETTLED_VEL_RAD_S,
                  'load_sum_band': list(PB.LOAD_SUM_BAND)}
    print(f'gains from p4-balance-04: kp_bal {kp_bal:.1f} kd_bal {kd_bal:.1f}; '
          f'k_task {K_TASK}, limit {TASK_LIMIT_M * 1000:.0f} mm, z only')

    ab = []
    for depth in (0.25, 0.40):
        off = judge(run(rig, depth, dx_for(depth), correct=False, kp_bal=kp_bal, kd_bal=kd_bal),
                    depth, thresholds)
        on = judge(run(rig, depth, dx_for(depth), correct=True, kp_bal=kp_bal, kd_bal=kd_bal),
                   depth, thresholds)
        ab.append({'dz_m': depth, 'off': off, 'on': on})
        print(f'  A/B @ {depth:.2f} m: arrival error OFF {off["drop_err_m"] * 1000:+7.1f} mm '
              f'-> ON {on["drop_err_m"] * 1000:+7.1f} mm')

    sweep = []
    for depth in DEPTHS:
        r = run(rig, depth, dx_for(depth), correct=True, kp_bal=kp_bal, kd_bal=kd_bal)
        e = judge(r, depth, thresholds)
        sweep.append(e)
        print(f'  {depth:.2f} holds={str(e["holds"]):>5} drop {e["drop_m"] * 1000:7.1f} '
              f'err {e["drop_err_m"] * 1000:+7.1f}mm corr {e["correction_m"] * 1000:+6.1f}mm '
              f'pitch {e["worst_foot_pitch_deg"]:6.2f} torque {e["peak_torque_usage"] * 100:5.1f}% '
              f'rej {e["ik_rejects"]:3d} failed={[k for k, v in e["rows"].items() if not v]}')
    held = [e['dz_m'] for e in sweep if e['holds']]
    print(f'\n*** D_MAX with the corrected loop = {max(held) if held else None}')
    print(f'*** (was 0.11 m with the uncorrected loop; the physical torque wall is ~0.30 m)')

    report = {'run_id': args.run_id,
              'scope': 'bounded z-only task-space correction on the crouch, and the depth it unlocks',
              'defect_fixed': 'PD tracking error: knee -0.150 rad (8.6 deg) over-flexed, from the '
                              'free-floating gravity feed-forward not covering the closed chain',
              'controller': {'k_task': K_TASK, 'task_limit_m': TASK_LIMIT_M, 'axis': 'z only',
                             'kp_bal': kp_bal, 'kd_bal': kd_bal,
                             'ik_reject_rule': f'max residual > {PB.IK_ACCEPT_M} -> hold the '
                                               f'previous posture and count it'},
              'world': {'path': 'assets/world_p4_handover.xml',
                        'sha256': hashlib.sha256(
                            (ROOT / 'assets' / 'world_p4_handover.xml').read_bytes()).hexdigest()},
              'thresholds': thresholds, 'ab': ab, 'sweep': sweep,
              'deepest_holding_depth_m': (max(held) if held else None),
              'previous_deepest_holding_depth_m': 0.11}
    (out / 'report.json').write_text(json.dumps(report, indent=2, default=str) + '\n',
                                     encoding='utf-8')
    lines = [f'# {args.run_id} — crouch with a bounded z-only task loop', '',
             f"**deepest holding depth {max(held) if held else None} m** "
             f'(was 0.11 m uncorrected)', '', '## A/B on the arrival error', '',
             '| depth | correction OFF | correction ON |', '|---|---|---|']
    for a in ab:
        lines.append(f"| {a['dz_m']:.2f} m | {a['off']['drop_err_m'] * 1000:+.1f} mm | "
                     f"{a['on']['drop_err_m'] * 1000:+.1f} mm |")
    lines += ['', '## depth sweep, corrected', '',
              '| dz | holds | drop | arrival err | correction | pitch | torque | IK rejects | failed |',
              '|---|---|---|---|---|---|---|---|---|']
    for e in sweep:
        lines.append(f"| {e['dz_m']:.2f} | {e['holds']} | {e['drop_m'] * 1000:.0f} mm | "
                     f"{e['drop_err_m'] * 1000:+.1f} mm | {e['correction_m'] * 1000:+.1f} mm | "
                     f"{e['worst_foot_pitch_deg']:.2f}° | {e['peak_torque_usage'] * 100:.0f}% | "
                     f"{e['ik_rejects']} | "
                     f"{', '.join(k for k, v in e['rows'].items() if not v) or '—'} |")
    (out / 'acceptance.md').write_text('\n'.join(lines) + '\n', encoding='utf-8')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
