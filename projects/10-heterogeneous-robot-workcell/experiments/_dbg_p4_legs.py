"""Diagnostic 2: the leg chain, the humanoid's OWN CoM, and whether a pelvis-pose crouch is
even kinematically available.

★ The global CoM is the wrong object. `data.subtree_com[0]` is rooted at the WORLD body, so it is
the CoM of the entire 251 kg scene (vehicle + a second assembly + this humanoid's 86 kg). It read
x 7.42 m while the humanoid's feet stood at x 3.93-4.22 m -- a 3 m "balance error" that belongs to
the model, not to the robot. The humanoid's CoM is the subtree rooted at `h_LINK_BASE`.

★ The second thing measured here is whether "pelvis 0.40 m lower with the feet where they are" is a
pose this leg can BE. If a 6-joint leg per side cannot hold the foot at its world pose while the
pelvis descends, then no amount of gain tuning will produce a 0.40 m crouch, and the alternative is
to change the *task*, not the controller.
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
from probe_p4_crouch import Body  # noqa: E402

BODY = mujoco.mjtObj.mjOBJ_BODY
JOINT = mujoco.mjtObj.mjOBJ_JOINT

BASE = 'h_LINK_BASE'
FOOT = {'L': 'h_LINK_FOOT_L', 'R': 'h_LINK_FOOT_R'}


def jname(m, j):
    return mujoco.mj_id2name(m, JOINT, j) or ''


def humanoid_com(body):
    """CoM of the humanoid's own subtree. Not the scene's."""
    mujoco.mj_comPos(body.m, body.d)
    bid = body.m.body(BASE).id
    return np.array(body.d.subtree_com[bid], dtype=float)


def humanoid_mass(body):
    m = body.m
    bid = m.body(BASE).id
    total, stack = 0.0, [bid]
    while stack:
        b = stack.pop()
        total += float(m.body_mass[b])
        for c in range(m.nbody):
            if int(m.body_parentid[c]) == b:
                stack.append(c)
    return total


def rot_err(Rc, Rt):
    R = Rt @ Rc.T
    cos = float(np.clip((np.trace(R) - 1.0) / 2.0, -1.0, 1.0))
    ang = float(np.arccos(cos))
    if ang < 1e-9:
        return np.zeros(3)
    ax = np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]]) / (2.0 * np.sin(ang))
    return ax * ang


#: The six joints of one leg, in chain order. DECLARED rather than found by suffix: `_L` also ends
#: the shoulder and elbow joints, so a suffix search returns an 11-joint "leg" and the IK then tries
#: to solve the arm as well.
LEG_CHAIN = {
    'L': ('J00_HIP_PITCH_L', 'J01_HIP_ROLL_L', 'J02_HIP_YAW_L',
          'J03_KNEE_PITCH_L', 'J04_ANKLE_PITCH_L', 'J05_ANKLE_ROLL_L'),
    'R': ('J06_HIP_PITCH_R', 'J07_HIP_ROLL_R', 'J08_HIP_YAW_R',
          'J09_KNEE_PITCH_R', 'J10_ANKLE_PITCH_R', 'J11_ANKLE_ROLL_R'),
}


def discover_legs(m):
    legs = {s: list(chain) for s, chain in LEG_CHAIN.items()}
    # cross-check the declaration against the model, so a renamed joint fails loudly here
    present = {jname(m, j) for j in range(m.njnt)}
    for side, names in legs.items():
        for n in names:
            if 'h_' + n not in present:
                raise RuntimeError(f'declared leg joint h_{n} is not in this model')
    return legs


def inventory(m):
    print('=== humanoid bodies (parent chain) ===')
    for b in range(m.nbody):
        n = mujoco.mj_id2name(m, BODY, b) or ''
        if not n.startswith('h_'):
            continue
        print(f'  body {n:24s} parent='
              f'{mujoco.mj_id2name(m, BODY, int(m.body_parentid[b])) or "world":24s} '
              f'mass={m.body_mass[b]:7.3f} pos={np.round(m.body_pos[b], 4)}')
    print()
    print('=== leg joints, in chain order ===')
    legs = discover_legs(m)
    for side, names in legs.items():
        print(f'  {side}: {names}')
    return legs


def leg_ik(body, side, chain, pelvis_qpos, foot_pos, foot_mat,
           iters=600, lam=0.02, cap=0.10):
    """Solve the 6 leg joints for the foot's pose, with the pelvis pose GIVEN.

    This is the honest form of "crouch": the pelvis pose is not a control input -- the free joint
    is driven by contacts and gravity -- so the reachable postures are the joint angles that are
    CONSISTENT with the pelvis being at the target while the feet stay where they are. Those joint
    angles ARE commandable, so `mj_forward` here computes targets, not a teleport.
    """
    m, d = body.m, body.d
    dofs = np.array([int(m.jnt_dofadr[m.joint('h_' + n).id]) for n in chain])
    qadrs = np.array([int(m.jnt_qposadr[m.joint('h_' + n).id]) for n in chain])
    lows = np.array([float(m.jnt_range[m.joint('h_' + n).id][0]) for n in chain])
    highs = np.array([float(m.jnt_range[m.joint('h_' + n).id][1]) for n in chain])
    fid = m.body(FOOT[side]).id
    q = np.array(pelvis_qpos, dtype=float)
    if q.shape != (m.nq,):
        raise ValueError(f'leg_ik needs the whole qpos ({m.nq},), got {q.shape}')
    jacp, jacr = np.zeros((3, m.nv)), np.zeros((3, m.nv))
    best_r, best_q, best_joints = None, None, None
    for _ in range(iters):
        d.qpos[:] = q
        mujoco.mj_forward(m, d)
        ep = np.asarray(foot_pos, dtype=float) - np.asarray(d.xpos[fid], dtype=float)
        er = rot_err(np.asarray(d.xmat[fid]).reshape(3, 3), np.asarray(foot_mat))
        err = np.concatenate([ep, er])
        r = float(np.linalg.norm(err))
        if best_r is None or r < best_r:
            best_r, best_q = r, np.array(q)
            best_joints = {n: float(q[a]) for n, a in zip(chain, qadrs)}
        # ★ THE JACOBIAN HAS TO BE COMPUTED. The first version allocated `jacp`/`jacr` and then
        # used them: a zero matrix, so `dq` was exactly zero, the loop returned its own seed, and
        # the reported residual was exactly the requested displacement -- 100.000 mm for a 100 mm
        # target, 60.000 mm for 60 mm. A number that looks like a measurement but is the input
        # echoed back, which is this project's most expensive recurring defect.
        mujoco.mj_jacBody(m, d, jacp, jacr, fid)
        J = np.vstack([jacp[:, dofs], jacr[:, dofs]])
        if not np.any(J):
            raise RuntimeError('mj_jacBody returned a zero Jacobian: the solver would move nothing')
        dq = J.T @ np.linalg.solve(J @ J.T + (lam ** 2) * np.eye(6), err)
        big = float(np.max(np.abs(dq)))
        if big > cap:
            dq = dq * (cap / big)
        q[qadrs] = np.clip(q[qadrs] + dq, lows, highs)
    # how close did any joint come to its own limit
    margin = {n: float(min(best_joints[n] - lo, hi - best_joints[n]))
              for n, lo, hi in zip(chain, lows, highs)}
    return best_r, best_q, best_joints, margin


def foot_print(m, d, side):
    """x/y extent of one foot's geoms in world space (the footprint)."""
    xs, ys = [], []
    for g in range(m.ngeom):
        bn = mujoco.mj_id2name(m, BODY, int(m.geom_bodyid[g])) or ''
        if bn != FOOT[side]:
            continue
        half = m.geom_size[g][:3]
        R = d.geom_xmat[g].reshape(3, 3)
        c = d.geom_xpos[g]
        for sx in (-1, 1):
            for sy in (-1, 1):
                for sz in (-1, 1):
                    p = c + R @ (sx * half[0], sy * half[1], sz * half[2])
                    xs.append(p[0])
                    ys.append(p[1])
    return min(xs), max(xs), min(ys), max(ys)


def main():
    body = Body()
    m, d = body.m, body.d
    legs = inventory(m)
    print(f'humanoid mass {humanoid_mass(body):.3f} kg '
          f'(whole scene {m.body_mass.sum():.3f} kg)')
    print()

    body.reset()
    body.run(1.0)
    c = humanoid_com(body)
    base = d.xpos[m.body(BASE).id]
    lx0, lx1, ly0, ly1 = foot_print(m, d, 'L')
    rx0, rx1, ry0, ry1 = foot_print(m, d, 'R')
    sup_x = (min(lx0, rx0), max(lx1, rx1))
    sup_y = (min(ly0, ry0), max(ly1, ry1))
    print('=== standing reference ===')
    print(f'  pelvis xyz      {np.round(base, 5)}')
    print(f'  humanoid CoM    {np.round(c, 5)}   (mass {humanoid_mass(body):.2f} kg)')
    print(f'  CoM - pelvis    {np.round(c - base, 4)}')
    print(f'  L footprint x [{lx0:.4f},{lx1:.4f}] y [{ly0:.4f},{ly1:.4f}]')
    print(f'  R footprint x [{rx0:.4f},{rx1:.4f}] y [{ry0:.4f},{ry1:.4f}]')
    print(f'  support   x [{sup_x[0]:.4f},{sup_x[1]:.4f}] y [{sup_y[0]:.4f},{sup_y[1]:.4f}] '
          f'(length {sup_x[1] - sup_x[0] * 1:.4f} m)')
    print(f'  CoM margin to support x: west {c[0] - sup_x[0]:+.4f} m, '
          f'east {sup_x[1] - c[0]:+.4f} m')
    for s, chain in legs.items():
        print(f'  {s} joints at stand: ' + ', '.join(
            f'{n} {float(d.qpos[int(m.jnt_qposadr[m.joint("h_" + n).id])]):+.4f}' for n in chain))
    print()

    # ---- pelvis pose targets: can the leg B E at the target with the feet pinned? ----
    foot_pos = {s: np.array(d.xpos[m.body(FOOT[s]).id], dtype=float) for s in ('L', 'R')}
    foot_mat = {s: np.array(d.xmat[m.body(FOOT[s]).id], dtype=float).reshape(3, 3)
                for s in ('L', 'R')}
    pelvis0 = np.array(d.qpos, dtype=float)      # the whole configuration; [:7] is the free joint
    print('=== pelvis-pose crouch feasibility (feet pinned at their standing pose) ===')
    print(f'  {"dz":>7} {"dx":>7} | {"resid_L":>9} {"resid_R":>9} | '
          f'{"CoM_x":>9} {"marg_west":>10} {"marg_east":>10} | tightest joint margin (rad)')
    rows = []
    for dz in (0.0, -0.10, -0.20, -0.30, -0.40, -0.45, -0.50, -0.55):
        for dx in (0.0, -0.06, 0.06):
            pelvis = np.array(pelvis0)
            pelvis[0] += dx
            pelvis[2] += dz
            res, q, joints, margin = leg_ik(body, 'L', legs['L'], pelvis,
                                            foot_pos['L'], foot_mat['L'])
            resR, qR, jointsR, marginR = leg_ik(body, 'R', legs['R'], pelvis,
                                                foot_pos['R'], foot_mat['R'])
            final = np.array(pelvis0)
            final[0] += dx
            final[2] += dz
            for n in legs['L']:
                final[int(m.jnt_qposadr[m.joint('h_' + n).id])] = joints[n]
            for n in legs['R']:
                final[int(m.jnt_qposadr[m.joint('h_' + n).id])] = jointsR[n]
            d.qpos[:] = final
            d.qvel[:] = 0.0
            mujoco.mj_forward(m, d)
            cc = humanoid_com(body)
            tight = min(min(margin.values()), min(marginR.values()))
            tight_name = min(list(margin.items()) + list(marginR.items()), key=lambda kv: kv[1])[0]
            print(f'  {dz:+.2f} {dx:+.2f} | {res * 1000:8.3f}mm {resR * 1000:8.3f}mm | '
                  f'{cc[0]:9.4f} {cc[0] - sup_x[0]:+10.4f} {sup_x[1] - cc[0]:+10.4f} | '
                  f'{tight:+.4f} @ {tight_name}')
            rows.append({'dz': dz, 'dx': dx, 'resid_L_m': res, 'resid_R_m': resR,
                         'com_x': float(cc[0]), 'mass_kg': humanoid_mass(body),
                         'margin_west_m': float(cc[0] - sup_x[0]),
                         'margin_east_m': float(sup_x[1] - cc[0]),
                         'tightest_joint_margin_rad': tight,
                         'tightest_joint': tight_name,
                         'joints_L': joints, 'joints_R': jointsR})
    print()
    best = [r for r in rows if r['dz'] == -0.40]
    if best:
        r = best[0]
        print(f'=== the -0.40 m row ===')
        print('  L joints ' + ', '.join(f'{k} {v:+.4f}' for k, v in r['joints_L'].items()))
        print('  R joints ' + ', '.join(f'{k} {v:+.4f}' for k, v in r['joints_R'].items()))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
