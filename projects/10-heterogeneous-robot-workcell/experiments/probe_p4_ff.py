"""Point 2 of the review: contact-consistent feed-forward, against the free-floating one.

THE DEFECT, RESTATED
--------------------
`qfrc_bias` at the measured state is the FREE-FLOATING gravity + Coriolis torque. A robot standing
on its feet is not free-floating: the static equilibrium is

    tau_joint = qfrc_bias_joint + (J_c^T f_c)_joint

and the second term -- the ground reaction projected into joint space -- is missing. The PD then
pays for it out of its error budget, which is what the measured **-0.150 rad / 8.6 deg** of knee
droop is.

THE FIX, WITHOUT ASSEMBLING `J^T F` BY HAND
-------------------------------------------
`mj_inverse` at the measured pose with `qvel = qacc = 0` returns `qfrc_inverse`: the torque required
to hold that state, **including the solver's own constraint forces**. One call, and it uses the same
contact forces the simulation actually resolved rather than a hand-rebuilt estimate.

The arms are left at the standing pose, as in the previous runs -- which is also why the 0.10 m row
failed (`_dbg_p4_entry.py`: the humanoid's own fingertip touches its own hip). That is a separate
finding and is NOT fixed here; this file changes exactly one thing.
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
import merge_world as mw  # noqa: E402
import probe_p4_balance as PB  # noqa: E402
import probe_p4_crouch2 as C2  # noqa: E402

WORLD = ROOT / 'assets' / 'world_p4_handover.xml'


def contact_consistent_ff(rig):
    """`tau` to hold the current state, contacts included. Returns (per-joint dict, qacc_input).

    `mj_forward` first: `mj_inverse` assumes the constraint data are already built, and building
    them from scratch here would be a second copy of the pipeline. Then `qacc = 0` makes it a STATIC
    inverse -- "what torque holds this pose" rather than "what torque produces this acceleration".
    """
    mujoco.mj_forward(rig.m, rig.d)
    saved_qvel = np.array(rig.d.qvel, dtype=float)
    rig.d.qvel[:] = 0.0
    rig.d.qacc[:] = 0.0
    mujoco.mj_inverse(rig.m, rig.d)
    out = {}
    for name in rig.names:
        jid = rig.m.joint('h_' + name).id
        if jid < 0:
            continue
        out[name] = float(rig.d.qfrc_inverse[int(rig.m.jnt_dofadr[jid])])
    rig.d.qvel[:] = saved_qvel          # `mj_step` needs the real velocity back
    return out


def tick(rig, q_target, *, mode, kp_bal, kd_bal, com_ref_x, com_vx):
    """One control step. `mode` picks the feed-forward: 'bias' (free-floating) or 'cc' (contacts)."""
    ctrl = np.asarray(mw.home_hold_ctrl(rig.m, rig.home, rig.d.qpos, rig.d.qvel), float)
    ff = contact_consistent_ff(rig) if mode == 'cc' else None
    for k, name in enumerate(rig.names):
        jid = rig.m.joint('h_' + name).id
        if jid < 0:
            continue
        adr, dof = int(rig.m.jnt_qposadr[jid]), int(rig.m.jnt_dofadr[jid])
        want = float(q_target[adr])
        tau = (want - float(rig.d.qpos[adr])) * float(rig.kp[k]) \
            - float(rig.d.qvel[dof]) * float(rig.kd[k])
        tau += float(rig.d.qfrc_bias[dof]) if mode == 'bias' else ff[name]
        for a in rig.index.get(int(jid), ()):
            lo, hi = rig.m.actuator_ctrlrange[a]
            ctrl[a] = float(min(max(tau, lo), hi))
    e = float(rig.humanoid_com()[0]) - float(com_ref_x)
    raw = kp_bal * e + kd_bal * com_vx
    for side, name in PB.ANKLE_PITCH.items():
        for a in rig.ankle_act[side]:
            lo, hi = rig.m.actuator_ctrlrange[a]
            ctrl[a] = float(min(max(ctrl[a] + raw, lo), hi))
    rig.d.ctrl[:] = ctrl
    mujoco.mj_step(rig.m, rig.d)


def run(rig, depth, *, mode, kp_bal, kd_bal, ramp_s=2.0, hold_s=1.5, record=0.05):
    rig.reset()
    rig.settle(1.0)
    standing_z = float(rig.d.xpos[rig.m.body(PB.BASE).id][2])
    n_ramp = int(ramp_s / rig.m.opt.timestep)
    every = max(1, int(record / rig.m.opt.timestep))
    corr, rejects, trace, ff_use = 0.0, 0, [], 0.0
    q_prev = np.array(rig.d.qpos, dtype=float)
    rig.prev_com, rig.prev_t = rig.humanoid_com(), float(rig.d.time)
    for i in range(n_ramp + int(hold_s / rig.m.opt.timestep)):
        frac = min(1.0, (i + 1) / n_ramp)
        phase = 'ramp' if i < n_ramp else 'hold'
        commanded_z = float(rig.pelvis0[2]) - depth * frac
        measured_z = float(rig.d.xpos[rig.m.body(PB.BASE).id][2])
        corr = float(np.clip(1.0 * (commanded_z - measured_z), -C2.TASK_LIMIT_M, C2.TASK_LIMIT_M))
        eff = max(0.0, depth * frac - corr)
        target = np.array([rig.pelvis0[0] + C2.dx_for(eff), rig.pelvis0[1],
                           float(rig.pelvis0[2]) - eff], dtype=float)
        seed = np.array(rig.d.qpos, dtype=float)
        q_try, resid, _m = rig.solve_posture(tuple(target), seed)
        if max(resid.values()) > PB.IK_ACCEPT_M:
            rejects += 1
            q_try = q_prev
        else:
            q_prev = q_try
        com_ref = float(rig.posture_com(q_try)[0])
        com_vx = rig.com_x_velocity()
        rig.prev_com, rig.prev_t = rig.humanoid_com(), float(rig.d.time)
        tick(rig, q_try, mode=mode, kp_bal=kp_bal, kd_bal=kd_bal, com_ref_x=com_ref, com_vx=com_vx)
        if mode == 'cc' and i % every == 0:
            f = contact_consistent_ff(rig)
            span = max((abs(f[n]) for n in ('J00_HIP_PITCH_L', 'J03_KNEE_PITCH_L',
                                            'J04_ANKLE_PITCH_L')), default=0.0)
            ff_use = max(ff_use, span)
        if i % every == 0 or i == n_ramp + int(hold_s / rig.m.opt.timestep) - 1:
            tr = rig.sample(standing_z, -depth, com_ref, phase)
            tr['correction_m'] = round(corr, 5)
            # the keys `C2.judge` reads: a consumer expecting a field the producer does not supply is
            # a silent interface mismatch waiting to happen, so they are produced here explicitly
            after = float(rig.d.xpos[rig.m.body(PB.BASE).id][2])
            tr['cmd_z'] = round(commanded_z, 5)
            tr['cmd_err_m'] = round(after - commanded_z, 5)
            tr['eff_depth_m'] = round(eff, 5)
            trace.append(tr)
    return {'trace': trace, 'standing_z': standing_z, 'rejects': rejects,
            'max_ff_torque_Nm': ff_use}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--run-id', default='p4-ff-01')
    ap.add_argument('--reports-dir', default=None)
    args = ap.parse_args()
    out = (Path(args.reports_dir) if args.reports_dir else ROOT / 'reports') / args.run_id
    out.mkdir(parents=True, exist_ok=True)

    ref = json.loads((ROOT / 'reports' / 'p4-balance-04' / 'report.json').read_text(encoding='utf-8'))
    kp_bal, kd_bal = ref['controller']['kp_bal'], ref['controller']['kd_bal']
    rig = PB.Rig()
    rig.capture_standing()
    th = {'crouch_tol_m': PB.CROUCH_TOL_M, 'foot_flat_max_deg': PB.FOOT_FLAT_MAX_DEG,
          'min_foot_share': PB.MIN_FOOT_SHARE, 'min_com_margin_m': PB.MIN_COM_MARGIN_M,
          'com_track_tol_m': PB.COM_TRACK_TOL_M, 'settled_vel_rad_s': PB.SETTLED_VEL_RAD_S,
          'load_sum_band': list(PB.LOAD_SUM_BAND)}

    ab = []
    for depth in (0.25, 0.40):
        bias = C2.judge(run(rig, depth, mode='bias', kp_bal=kp_bal, kd_bal=kd_bal), depth, th)
        cc = C2.judge(run(rig, depth, mode='cc', kp_bal=kp_bal, kd_bal=kd_bal), depth, th)
        ab.append({'dz_m': depth, 'bias': bias, 'cc': cc})
        print(f'  {depth:.2f} m | arrival err  bias {bias["drop_err_m"] * 1000:+7.1f} mm '
              f'-> cc {cc["drop_err_m"] * 1000:+7.1f} mm | '
              f'pitch {bias["worst_foot_pitch_deg"]:6.2f} -> {cc["worst_foot_pitch_deg"]:6.2f} deg | '
              f'torque {bias["peak_torque_usage"] * 100:5.1f}% -> {cc["peak_torque_usage"] * 100:5.1f}%')

    sweep = []
    for depth in C2.DEPTHS:
        r = run(rig, depth, mode='cc', kp_bal=kp_bal, kd_bal=kd_bal)
        e = C2.judge(r, depth, th)
        e['max_ff_torque_Nm'] = r['max_ff_torque_Nm']
        sweep.append(e)
        print(f'  cc {depth:.2f} holds={str(e["holds"]):>5} drop {e["drop_m"] * 1000:7.1f} '
              f'err {e["drop_err_m"] * 1000:+7.1f}mm pitch {e["worst_foot_pitch_deg"]:6.2f} '
              f'torque {e["peak_torque_usage"] * 100:5.1f}% ffmax {r["max_ff_torque_Nm"]:6.1f}Nm '
              f'failed={[k for k, v in e["rows"].items() if not v]}')
    held = [e['dz_m'] for e in sweep if e['holds']]
    print(f'\n*** D_MAX with contact-consistent feed-forward = {max(held) if held else None}')
    print('*** (0.20 m with the free-floating feed-forward)')

    report = {'run_id': args.run_id,
              'scope': 'contact-consistent static feed-forward (mj_inverse at qacc=0) vs the '
                       'free-floating qfrc_bias',
              'defect': 'qfrc_bias is the FREE-FLOATING gravity torque; the standing robot is a '
                        'closed chain through its feet, so tau_joint = qfrc_bias + (Jc^T fc) and the '
                        'second term was missing -- paid for by the PD as -0.150 rad of knee droop',
              'implementation': 'mujoco.mj_inverse with qvel=0, qacc=0; read qfrc_inverse per joint',
              'unchanged': 'the arms are still held at the standing pose (see the 0.10 m '
                           'self-contact finding in _dbg_p4_entry.py)',
              'world': {'path': 'assets/world_p4_handover.xml',
                        'sha256': hashlib.sha256(WORLD.read_bytes()).hexdigest()},
              'ab': ab, 'sweep_cc': sweep,
              'deepest_holding_depth_m': (max(held) if held else None),
              'previous_deepest_holding_depth_m': 0.20}
    (out / 'report.json').write_text(json.dumps(report, indent=2, default=str) + '\n',
                                     encoding='utf-8')
    lines = [f'# {args.run_id} — contact-consistent feed-forward', '',
             f"**deepest holding depth {max(held) if held else None} m** (was 0.20 m)", '',
             '## A/B, same controller, one term changed', '',
             '| depth | arrival err bias | arrival err cc | pitch bias | pitch cc | torque bias | torque cc |',
             '|---|---|---|---|---|---|---|']
    for a in ab:
        b, c = a['bias'], a['cc']
        lines.append(f"| {a['dz_m']:.2f} m | {b['drop_err_m'] * 1000:+.1f} mm | "
                     f"{c['drop_err_m'] * 1000:+.1f} mm | {b['worst_foot_pitch_deg']:.2f}° | "
                     f"{c['worst_foot_pitch_deg']:.2f}° | {b['peak_torque_usage'] * 100:.0f}% | "
                     f"{c['peak_torque_usage'] * 100:.0f}% |")
    lines += ['', '## depth sweep, contact-consistent', '',
              '| dz | holds | drop | arrival err | pitch | torque | max ff torque | failed |',
              '|---|---|---|---|---|---|---|---|']
    for e in sweep:
        lines.append(f"| {e['dz_m']:.2f} | {e['holds']} | {e['drop_m'] * 1000:.0f} mm | "
                     f"{e['drop_err_m'] * 1000:+.1f} mm | {e['worst_foot_pitch_deg']:.2f}° | "
                     f"{e['peak_torque_usage'] * 100:.0f}% | {e['max_ff_torque_Nm']:.1f} N·m | "
                     f"{', '.join(k for k, v in e['rows'].items() if not v) or '—'} |")
    (out / 'acceptance.md').write_text('\n'.join(lines) + '\n', encoding='utf-8')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
