"""Is `tau_bias - J^T f_c` a FEED-FORWARD, or is it the torque already applied?

The reviewer's formula is `tau_ff = tau_bias - tau_contact` with `tau_contact = sum J^T f`, and the
plan is to read `f` from `mj_contactForce`. That is checkable, and the check matters: if the
projection of the MEASURED contact force reproduces the torque currently applied, then injecting it
as a feed-forward adds the current output to itself rather than supplying a missing term.

Four measurements, each able to settle it:

  A. CONVENTION. All four (frame-mode x sign) combinations against `qfrc_constraint`, plus an
     INDEPENDENT check that does not involve `qfrc_constraint` at all: the six FLOATING-BASE rows of
     `sum J^T f` must equal the TOTAL contact wrench (0, 0, m*g, ...). If they do, the frame and
     sign are right by Newton, not by fitting.

  B. THE IDENTITY. |(qfrc_bias - qfrc_constraint) - applied| per joint. If it is ~0, the quantity is
     an echo of the current output.

  C. mj_inverse vs the applied torque, same question.

  D. THE NON-CIRCULAR VERSION. Replace the measured force with a DESIRED one: the ground must carry
     the body weight, so f_desired = (0, 0, m*g / n_feet) applied at each foot's contact point. That
     number does not depend on the current actuator output, so it is a genuine feed-forward. Its
     size against the free-floating `qfrc_bias` at the same joints is the deficit it would fill.
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


def project(m, d, contacts, *, sign, apply_at):
    """sum_k J_k^T f_k, with the force read from `f_k` (a dict keyed by contact index)."""
    tau = np.zeros(m.nv)
    jacp = np.zeros((3, m.nv))
    jacr = np.zeros((3, m.nv))
    for i, (body, pos, f_world, _pair) in contacts.items():
        jacp[:] = 0.0
        jacr[:] = 0.0
        mujoco.mj_jac(m, d, jacp, jacr, np.asarray(pos if apply_at == 'point' else pos, float),
                      body)
        tau += jacp.T @ (sign * np.asarray(f_world, float))
    return tau


def gather(m, d, *, only_feet, frame_mode='row'):
    """Contact descriptors: (body on the robot side, application point, world-frame force, pair).

    The force returned here is the MEASURED one, rotated into the world with the convention
    `f_world = frame.T @ f_local`; the four-combination test in `decompose` selects the frame mode,
    and the DESIRED-force arm replaces the force with the weight split. One builder, three arms.
    """
    out = {}
    for i in range(d.ncon):
        c = d.contact[i]
        g1, g2 = int(c.geom[0]), int(c.geom[1])
        b1, b2 = int(m.geom_bodyid[g1]), int(m.geom_bodyid[g2])
        n1 = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, b1) or ''
        n2 = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, b2) or ''
        if only_feet and not ('LINK_FOOT' in n1 or 'LINK_FOOT' in n2):
            continue
        if b1 == 0 and b2 == 0:
            continue
        body = b1 if b1 != 0 else b2
        force = np.zeros(6)
        mujoco.mj_contactForce(m, d, i, force)
        frame = np.array(d.contact.frame[i], dtype=float).reshape(3, 3)
        f_local = np.array(force[:3], dtype=float)
        f_world = (frame @ f_local) if frame_mode == 'col' else (frame.T @ f_local)
        out[i] = (body, np.array(c.pos, dtype=float).copy(), f_world, (n1, n2))
    return out


def scratch_forward(rig, ctrl):
    m = rig.m
    sc = mujoco.MjData(m)
    sc.qpos[:] = rig.d.qpos
    sc.qvel[:] = 0.0
    sc.ctrl[:] = ctrl
    mujoco.mj_forward(m, sc)
    return sc


def decompose(rig, tag, ctrl):
    m = rig.m
    sc = scratch_forward(rig, ctrl)
    bias = np.array(sc.qfrc_bias, dtype=float)
    cons = np.array(sc.qfrc_constraint, dtype=float)
    nu_free = 6

    print(f'\n===== {tag} =====')
    print(f'  gravity {np.array(m.opt.gravity)}  nq {m.nq}  nv {m.nv}  ncon {sc.ncon}')

    # ---- A. convention: measured force, all four combinations --------------------------------
    best = None
    for frame_mode in ('row', 'col'):
        for sign in (+1.0, -1.0):
            con = gather(m, sc, only_feet=False, frame_mode=frame_mode)
            tau = project(m, sc, con, sign=sign, apply_at='point')
            err = float(np.max(np.abs(tau - cons)))
            print(f'  A  frame={frame_mode:>3} sign={sign:+.0f}  '
                  f'max|sum J^T f - qfrc_constraint| = {err:10.4f} N.m')
            if best is None or err < best[0]:
                best = (err, frame_mode, sign)

    # ---- A2. independent check: floating-base rows must equal the total contact wrench --------
    con = gather(m, sc, only_feet=False, frame_mode=best[1])
    f_total = np.zeros(3)
    for i, (b, p, f_world, pr) in con.items():
        f_total += f_world
    tau_all = project(m, sc, con, sign=best[2], apply_at='point')
    humanoid_mass = float(m.body_subtreemass[m.body(PB.BASE).id])
    print(f'  A2 floating-base rows of sum J^T f: {np.round(tau_all[:6], 3)}')
    print(f'     total contact force from mj_contactForce: {np.round(f_total, 3)} N')
    print(f'     humanoid subtree mass {humanoid_mass:.3f} kg -> weight '
          f'{humanoid_mass * 9.81:.2f} N')

    # ---- B. is (bias - cons) the torque already applied? --------------------------------------
    applied = np.zeros(m.nv)
    for k, name in enumerate(rig.names):
        jid = m.joint('h_' + name).id
        if jid < 0:
            continue
        for a in rig.index.get(int(jid), ()):
            applied[int(m.jnt_dofadr[jid])] += float(sc.ctrl[a])
    ident = bias - cons
    worst_j, worst_v = 0, 0.0
    for j in range(m.nv):
        if abs(ident[j] - applied[j]) > worst_v:
            worst_j, worst_v = j, abs(ident[j] - applied[j])
    jn = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_JOINT,
                           int(m.dof_jntid[worst_j])) or f'dof{worst_j}'
    print(f'  B  max_j |(bias - cons) - applied| over ALL {m.nv} dofs = {worst_v:.6f} N.m '
          f'(at {jn})')
    print(f'     -> {"IDENTICAL: the projection of the MEASURED force IS the current output" if worst_v < 1e-3 else "NOT identical: it carries extra information"}')

    # ---- C. mj_inverse -------------------------------------------------------------------------
    sc2 = scratch_forward(rig, ctrl)
    sc2.qacc[:] = 0.0
    mujoco.mj_inverse(m, sc2)
    inv = np.array(sc2.qfrc_inverse, dtype=float)
    d_inv_applied = float(np.max(np.abs(inv - applied)))
    d_inv_ident = float(np.max(np.abs(inv - ident)))
    print(f'  C  max |qfrc_inverse - applied| = {d_inv_applied:.4f} N.m;   '
          f'max |qfrc_inverse - (bias-cons)| = {d_inv_ident:.4f} N.m')

    # ---- D. the non-circular DESIRED-force version ---------------------------------------------
    feet = list(gather(m, sc, only_feet=True, frame_mode=best[1]).keys())
    weight = humanoid_mass * 9.81
    des = {}
    for i in feet:
        b, p, _f, pr = gather(m, sc, only_feet=True, frame_mode=best[1])[i]
        des[i] = (b, p, np.array([0.0, 0.0, weight / len(feet)]), pr)
    tau_des = project(m, sc, des, sign=best[2], apply_at='point')
    print(f'  D  DESIRED-GRF projection (weight {weight:.1f} N over {len(feet)} foot contacts):')
    print(f'     {"joint":>24} {"qfrc_bias":>11} {"-sum J^T f":>12} {"total ff":>10} '
          f'{"current applied":>16}')
    for side in ('L', 'R'):
        for n in PB.LEG_CHAIN[side]:
            jid = m.joint('h_' + n).id
            dof = int(m.jnt_dofadr[jid])
            print(f'     {n:>24} {bias[dof]:11.3f} {-tau_des[dof]:12.3f} '
                  f'{bias[dof] - tau_des[dof]:10.3f} {applied[dof]:16.3f}')
    worst_needed = max(abs(bias[m.jnt_dofadr[m.joint("h_" + n).id]])
                       for side in ('L', 'R') for n in PB.LEG_CHAIN[side])
    print(f'     worst |qfrc_bias| over the leg joints = {worst_needed:.3f} N.m;  '
          f'worst |sum J^T f| (desired) = '
          f'{max(abs(tau_des[m.jnt_dofadr[m.joint("h_" + n).id]]) for side in ("L", "R") for n in PB.LEG_CHAIN[side]):.3f} N.m')
    return {'convention': best, 'identity_residual': worst_v, 'inv_vs_applied': d_inv_applied}


def main():
    rig = PB.Rig()
    rig.capture_standing()
    ctrl_stand = np.asarray(mw.home_hold_ctrl(rig.m, rig.home, rig.d.qpos, rig.d.qvel), float)
    decompose(rig, 'STANDING (settled)', ctrl_stand)

    import json  # noqa: E402

    import probe_p4_crouch2 as C2  # noqa: E402
    ref = json.loads((ROOT / 'reports' / 'p4-balance-04' / 'report.json').read_text(encoding='utf-8'))
    r = C2.run(rig, 0.20, C2.dx_for(0.20), correct=True,
               kp_bal=ref['controller']['kp_bal'], kd_bal=ref['controller']['kd_bal'],
               ramp_s=2.0, hold_s=0.6)
    print(f'\n  [drove to a crouch: drop {r["trace"][-1]["drop_m"] * 1000:.1f} mm, '
          f'rejects {r["rejects"]}]')
    decompose(rig, 'HELD 0.20 m CROUCH', np.asarray(rig.d.ctrl, dtype=float))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
