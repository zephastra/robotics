"""Does an explicit sum(J^T f) reproduce MuJoCo's own qfrc_constraint, and where is the deficit?

Three questions, each answered by a number rather than an argument:

  Q1  SIGN + FRAME CONVENTION. `mj_contactForce` returns the force in the CONTACT frame and
      `contact.frame` is a 3x3 whose rows/columns are that frame's axes. Rather than assume which,
      all four combinations (frame @ f, frame.T @ f) x (f on body1, f on body2) are computed and
      compared against `qfrc_constraint`, which is MuJoCo's own answer in joint space. Whichever
      reproduces it is the convention -- measured, not assumed.

  Q2  IS THE DECOMPOSITION REAL. `qfrc_inverse == qfrc_bias - qfrc_constraint` at rest, per joint.

  Q3  WHERE IS THE DEFICIT, AND IS IT CIRCULAR. For every leg joint, print the bias torque, the
      constraint torque, the inverse-dynamics torque, and the torque the CURRENT controller already
      applies. If the inverse-dynamics answer merely echoes the applied torque it is not a
      feed-forward signal at all -- it is the present state read back -- and that has to be known
      before any of it is wired into a controller.
"""
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
import merge_world as mw  # noqa: E402


def explicit_contact_tau(m, d, *, frame_mode, sign, only_feet):
    """sum_k J_k^T f_k over contacts, in joint space. Returns (tau, used, pairs)."""
    tau = np.zeros(m.nv)
    jacp = np.zeros((3, m.nv))
    jacr = np.zeros((3, m.nv))
    force = np.zeros(6)
    used, pairs = 0, []
    for i in range(d.ncon):
        c = d.contact[i]
        g1, g2 = int(c.geom[0]), int(c.geom[1])
        b1, b2 = int(m.geom_bodyid[g1]), int(m.geom_bodyid[g2])
        n1 = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, b1) or ''
        n2 = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, b2) or ''
        if only_feet and not ('LINK_FOOT' in n1 or 'LINK_FOOT' in n2):
            continue
        # which side is the robot? a contact with the world body has it on exactly one side
        body, other = (b1, n2) if b1 != 0 else (b2, n1)
        if body == 0:
            continue
        mujoco.mj_contactForce(m, d, i, force)
        frame = np.array(c.frame, dtype=float).reshape(3, 3)
        f_local = np.array(force[:3], dtype=float)
        f_world = (frame @ f_local) if frame_mode == 'col' else (frame.T @ f_local)
        jacp[:] = 0.0
        jacr[:] = 0.0
        mujoco.mj_jac(m, d, jacp, jacr, np.array(c.pos, dtype=float), body)
        tau += jacp.T @ (sign * f_world)
        used += 1
        pairs.append((n1, n2))
    return tau, used, pairs


def decompose(rig, tag, ctrl_now):
    m, d0 = rig.m, rig.d
    sc = mujoco.MjData(m)
    sc.qpos[:] = d0.qpos
    sc.qvel[:] = 0.0
    sc.ctrl[:] = ctrl_now
    mujoco.mj_forward(m, sc)
    bias = np.array(sc.qfrc_bias, dtype=float)
    cons = np.array(sc.qfrc_constraint, dtype=float)
    ncon = int(sc.ncon)
    types = {}
    for k in range(len(sc.efc_type)):
        t = int(sc.efc_type[k])
        name = {0: 'contact', 1: 'limit', 2: 'frictionloss'}.get(t, f'type{t}')
        types[name] = types.get(name, 0) + 1

    print(f'\n===== {tag} =====')
    print(f'  contacts {ncon}, constraint rows by type {types}')

    best = None
    for frame_mode in ('col', 'row'):
        for sign in (+1.0, -1.0):
            tau, used, _pairs = explicit_contact_tau(
                m, sc, frame_mode=frame_mode, sign=sign, only_feet=False)
            err = float(np.max(np.abs(tau - cons)))
            scale = float(np.max(np.abs(cons))) or 1.0
            rel = err / scale
            print(f'  explicit J^T f  frame={frame_mode:>3} sign={sign:+.0f}  '
                  f'contacts_used {used:2d}  max|tau - qfrc_constraint| = {err:12.4f} N.m  '
                  f'rel {rel:8.4f}')
            if best is None or err < best[0]:
                best = (err, frame_mode, sign)
    print(f'  -> best convention: frame={best[1]}, sign={best[2]:+.0f}, residual {best[0]:.6f} N.m')

    # feet-only variant, which is what a feed-forward would actually use
    tau_feet, n_feet, pairs = explicit_contact_tau(
        m, sc, frame_mode=best[1], sign=best[2], only_feet=True)
    print(f'  feet-only: {n_feet} contacts {sorted(set(pairs))}')

    # mj_inverse, with qacc zeroed AFTER the forward pass so the constraint data is the static one
    sc.qacc[:] = 0.0
    mujoco.mj_inverse(m, sc)
    inv = np.array(sc.qfrc_inverse, dtype=float)

    print(f'\n  {"joint":>24} {"bias":>11} {"constraint":>11} {"bias-cons":>11} '
          f'{"qfrc_inv":>11} {"applied":>11} {"demand":>11}')
    for side in ('L', 'R'):
        for n in PB.LEG_CHAIN[side]:
            jid = m.joint('h_' + n).id
            dof = int(m.jnt_dofadr[jid])
            acts = list(rig.index.get(int(jid), ()))
            applied = 0.0
            for a in acts:
                applied += float(sc.ctrl[a])
            print(f'  {n:>24} {bias[dof]:11.4f} {cons[dof]:11.4f} {bias[dof] - cons[dof]:11.4f} '
                  f'{inv[dof]:11.4f} {applied:11.4f} {abs(bias[dof] - cons[dof]):11.4f}')
    print(f'  {"worst |bias-cons| (all nv)":>24} {np.max(np.abs(bias - cons)):11.4f}')
    print(f'  {"worst |inv-(bias-cons)|":>24} {np.max(np.abs(inv - (bias - cons))):11.4f}')
    return {'contacts': ncon, 'types': types, 'best': best,
            'worst_ident': float(np.max(np.abs(inv - (bias - cons))))}


def main():
    rig = PB.Rig()
    rig.capture_standing()
    ctrl_stand = np.asarray(mw.home_hold_ctrl(rig.m, rig.home, rig.d.qpos, rig.d.qvel), float)
    decompose(rig, 'STANDING (settled)', ctrl_stand)

    # actuator ranges on the leg joints, for the clamp the reviewer asks for
    print('\n===== actuator ctrl ranges, leg joints =====')
    for side in ('L', 'R'):
        for n in PB.LEG_CHAIN[side]:
            jid = rig.m.joint('h_' + n).id
            for a in rig.index.get(int(jid), ()):
                lo, hi = rig.m.actuator_ctrlrange[a]
                an = mujoco.mj_id2name(rig.m, mujoco.mjtObj.mjOBJ_ACTUATOR, a) or '?'
                print(f'  {n:>24}  {an:>30}  [{lo:+8.1f}, {hi:+8.1f}]')

    # now mid-crouch: drive the posture to 0.20 m and re-decompose
    print('\n===== driving to a 0.20 m crouch =====')
    import json  # noqa: E402

    import probe_p4_crouch2 as C2  # noqa: E402
    ref = json.loads((ROOT / 'reports' / 'p4-balance-04' / 'report.json').read_text(encoding='utf-8'))
    kp_bal, kd_bal = ref['controller']['kp_bal'], ref['controller']['kd_bal']
    r = C2.run(rig, 0.20, C2.dx_for(0.20), correct=True,
               kp_bal=kp_bal, kd_bal=kd_bal, ramp_s=2.0, hold_s=0.6)
    ctrl_crouch = np.asarray(rig.d.ctrl, dtype=float)
    print(f'  drop {r["trace"][-1]["drop_m"] * 1000:.1f} mm, rejects {r["rejects"]}')
    decompose(rig, 'HELD 0.20 m CROUCH', ctrl_crouch)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
