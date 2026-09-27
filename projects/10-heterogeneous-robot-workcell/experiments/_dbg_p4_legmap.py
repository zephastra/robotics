"""Diagnostic 3: the leg's own crouch envelope, with the feet pinned exactly where they stand.

The question is not "can a controller be tuned to get to 0.40 m" but "is a 0.40 m flat-footed
posture a pose this leg HAS". A 6-joint leg with the foot's full pose pinned is exactly determined,
so for each (dx, dz) of the pelvis there is at most one joint vector, and the residual says whether
it exists.

Two modes, because they have different support polygons:

  FULL   the foot's position AND orientation are pinned -- the sole stays flat on the floor, so the
         support polygon is the whole footprint. This is the stable one.
  POSN   only the foot's position is pinned; the sole is free to pitch, i.e. the heel lifts and the
         foot rolls onto its toe. Deeper, but the support polygon collapses to the toe strip.
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
from _dbg_p4_legs import LEG_CHAIN, FOOT, rot_err  # noqa: E402

BODY = mujoco.mjtObj.mjOBJ_BODY


def solve(body, side, q_seed, target_pos, target_mat, *, flat, iters=500, lam=0.01, cap=0.08):
    """DLS on either the full 6-D foot pose or its 3-D position alone.

    `flat=False` is not "no constraint on rotation": the ankle is left free to run to its own limit,
    which is exactly what a heel-lift IS. The row is judged by the resulting foot pitch, measured.
    """
    m, d = body.m, body.d
    chain = LEG_CHAIN[side]
    dofs = np.array([int(m.jnt_dofadr[m.joint('h_' + n).id]) for n in chain])
    qadrs = np.array([int(m.jnt_qposadr[m.joint('h_' + n).id]) for n in chain])
    lows = np.array([float(m.jnt_range[m.joint('h_' + n).id][0]) for n in chain])
    highs = np.array([float(m.jnt_range[m.joint('h_' + n).id][1]) for n in chain])
    fid = m.body(FOOT[side]).id
    q = np.array(q_seed, dtype=float)
    jacp, jacr = np.zeros((3, m.nv)), np.zeros((3, m.nv))
    best_r, best_q = None, None
    for _ in range(iters):
        d.qpos[:] = q
        mujoco.mj_forward(m, d)
        ep = np.asarray(target_pos, float) - np.asarray(d.xpos[fid], float)
        mujoco.mj_jacBody(m, d, jacp, jacr, fid)
        if flat:
            er = rot_err(np.asarray(d.xmat[fid]).reshape(3, 3), np.asarray(target_mat))
            err = np.concatenate([ep, er])
            J = np.vstack([jacp[:, dofs], jacr[:, dofs]])
        else:
            err = ep
            J = jacp[:, dofs]
        r = float(np.linalg.norm(err))
        if best_r is None or r < best_r:
            best_r, best_q = r, np.array(q)
        if r < 1e-7:
            break
        if not np.any(J):
            raise RuntimeError('zero Jacobian -- the solve would move nothing')
        dq = J.T @ np.linalg.solve(J @ J.T + (lam ** 2) * np.eye(J.shape[0]), err)
        big = float(np.max(np.abs(dq)))
        if big > cap:
            dq = dq * (cap / big)
        q[qadrs] = np.clip(q[qadrs] + dq, lows, highs)
    return best_r, best_q


def evaluate(body, q_base, pelvis_dx, pelvis_dz, *, flat, seeds, foot_pos, foot_mat):
    """Solve both legs for this pelvis target.

    `seeds` is a dict `{side: qpos or None}`, and it is checked per SIDE and for None: the first
    version passed a dict that was never itself None, so the "no seed" branch was unreachable, the
    seed stayed `np.array(None)` (a 0-dimensional array) and the solve died indexing it. A default
    that cannot be reached is the dead-branch family again.
    """
    m, d = body.m, body.d
    qp = np.array(q_base, dtype=float)
    qp[0] += pelvis_dx
    qp[2] += pelvis_dz
    resid = {}
    for side in ('L', 'R'):
        side_seed = None if seeds is None else seeds.get(side)
        start = np.array(qp if side_seed is None else side_seed, dtype=float)
        start[0], start[2] = qp[0], qp[2]
        r, best = solve(body, side, start, foot_pos[side], foot_mat[side], flat=flat)
        resid[side] = r
        for n in LEG_CHAIN[side]:
            qp[int(m.jnt_qposadr[m.joint('h_' + n).id])] = best[int(
                m.jnt_qposadr[m.joint('h_' + n).id])]
    d.qpos[:] = qp
    d.qvel[:] = 0.0
    mujoco.mj_forward(m, d)
    mujoco.mj_comPos(m, d)
    com = np.array(d.subtree_com[m.body('h_LINK_BASE').id], dtype=float)
    pitch = {}
    for side in ('L', 'R'):
        R = np.asarray(d.xmat[m.body(FOOT[side]).id]).reshape(3, 3)
        pitch[side] = float(np.degrees(np.arccos(np.clip(R[2, 2], -1, 1))))
    margin = {}
    for side in ('L', 'R'):
        for n in LEG_CHAIN[side]:
            j = m.joint('h_' + n).id
            lo, hi = (float(v) for v in m.jnt_range[j])
            v = float(d.qpos[int(m.jnt_qposadr[j])])
            margin[n] = min(v - lo, hi - v)
    worst = min(margin, key=margin.get)
    return {'resid_m': max(resid['L'], resid['R']), 'com_x': float(com[0]),
            'foot_pitch_deg': pitch, 'tightest': worst, 'tightest_margin': margin[worst],
            'qp': qp}


def main():
    body = Body()
    m, d = body.m, body.d
    body.reset()
    body.run(1.2)
    DX_GRID = tuple(round(v, 2) for v in np.arange(-0.40, 0.021, 0.04))
    DZ_GRID = tuple(round(v, 3) for v in np.arange(0.0, -0.501, -0.05))
    q_base = np.array(d.qpos, dtype=float)
    foot_pos = {s: np.array(d.xpos[m.body(FOOT[s]).id], float) for s in ('L', 'R')}
    # ★ COPY. `np.asarray` on a view of `d.xmat` does NOT copy, so the "target" orientation was a
    # LIVE VIEW of the current one: `rot_err(target, current)` was identically zero, the flat-sole
    # constraint was never applied, and this mode silently degenerated into the ankle-free one --
    # it printed `rot_err 0.000e+00` next to a foot pitched 11.74 deg. `np.array` copies; the
    # assertion below is what makes that a checked property rather than a comment.
    foot_mat = {s: np.array(d.xmat[m.body(FOOT[s]).id], dtype=float).reshape(3, 3)
                for s in ('L', 'R')}
    for s in ('L', 'R'):
        if np.shares_memory(foot_mat[s], d.xmat[m.body(FOOT[s]).id]):
            raise RuntimeError(f'foot_mat[{s}] aliases d.xmat: the target would move with the state')
    stand_z = body.base_z()
    sup_lo, sup_hi = 3.9334, 4.2217          # measured in _dbg_p4_legs, standing footprint
    print(f'standing pelvis z {stand_z:.5f}, support x [{sup_lo}, {sup_hi}]')
    print()

    for flat in (True, False):
        label = 'FULL (sole flat, support = whole footprint)' if flat \
            else 'POSN (heel free -- sole rolls onto the toe)'
        print(f'=== mode {label} ===')
        # ★ FEASIBLE MEANS THREE THINGS AT ONCE: the IK found a pose, the CoM is inside the support
        # polygon, and no joint is pinned at its own limit. The first version asked only whether the
        # residual was <= 1 mm, and so reported `deepest dz = -0.600` for poses whose CoM sat
        # 54 mm OUTSIDE the footprint -- a robot that has already fallen over is not crouching.
        print(f'  {"dz":>7} | ' + ' '.join(f'{dx:+5.2f}' for dx in DX_GRID))
        print('  ' + '-' * (11 + 7 * len(DX_GRID)))
        best_rows = {}
        for dz in DZ_GRID:
            seeds = None
            cells, feas = [], []
            for dx in DX_GRID:
                r = evaluate(body, q_base, float(dx), float(dz), flat=flat,
                             seeds=seeds, foot_pos=foot_pos, foot_mat=foot_mat)
                seeds = {'L': r['qp'], 'R': r['qp']}
                west = r['com_x'] - sup_lo
                east = sup_hi - r['com_x']
                ok = (r['resid_m'] <= 1e-3 and west > 0.0 and east > 0.0
                      and r['tightest_margin'] > 1e-4)
                cells.append('  ok  ' if ok else
                             (' r..  ' if r['resid_m'] > 1e-3 else
                              (' .B.  ' if not (west > 0 and east > 0) else ' .L.  ')))
                if ok:
                    feas.append((dx, r))
            print(f'  {dz:+.3f} | ' + ' '.join(cells))
            if feas:
                # widest-margin feasible sample at this depth
                pick = max(feas, key=lambda dr: min(dr[1]['com_x'] - sup_lo, sup_hi - dr[1]['com_x']))
                best_rows[dz] = pick
        print()
        if best_rows:
            print(f'  {"dz":>7} {"dx":>7} {"CoM_x":>9} {"marg_W":>8} {"marg_E":>8} '
                  f'{"pitchL":>7} {"pitchR":>7}  tightest joint')
            for dz, (dx, r) in sorted(best_rows.items(), key=lambda kv: kv[0]):
                print(f'  {dz:+.3f} {dx:+.3f} {r["com_x"]:9.4f} '
                      f'{r["com_x"] - sup_lo:+8.4f} {sup_hi - r["com_x"]:+8.4f} '
                      f'{r["foot_pitch_deg"]["L"]:6.2f} {r["foot_pitch_deg"]["R"]:6.2f}  '
                      f'{r["tightest"]} margin {r["tightest_margin"]:+.4f} rad')
            deepest = min(best_rows)
            print(f'  --> DEEPEST feasible flat-footed crouch (this grid): {deepest:+.3f} m')
        else:
            print('  --> no feasible sample on this grid')
        print()
    print('legend: ok = solvable AND CoM inside footprint AND no joint at its limit;')
    print('        r.. = no pose exists;  .B. = pose exists but the CoM is outside;')
    print('        .L. = pose exists, CoM inside, but a joint is pinned at its limit')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
