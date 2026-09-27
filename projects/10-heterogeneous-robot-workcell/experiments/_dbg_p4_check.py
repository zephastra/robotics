"""Verify the leg-map's own claim: if the foot's orientation is pinned, the sole pitch must stay at
its standing value. A mode that says "flat" while reporting 11.75 deg of pitch is measuring
something other than what it says -- so measure both errors at the FINAL configuration.
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
from _dbg_p4_legs import FOOT, rot_err  # noqa: E402
import _dbg_p4_legmap as LM  # noqa: E402

SUP_LO, SUP_HI = 3.9334, 4.2217


def main():
    body = Body()
    m, d = body.m, body.d
    body.reset()
    body.run(1.2)
    q_base = np.array(d.qpos, dtype=float)
    foot_pos = {s: np.array(d.xpos[m.body(FOOT[s]).id], float) for s in ('L', 'R')}
    foot_mat = {s: np.asarray(d.xmat[m.body(FOOT[s]).id]).reshape(3, 3) for s in ('L', 'R')}
    print(f'{"dz":>7} {"dx":>7} {"resid_reported":>15} {"pos_err_L":>11} {"rot_err_L":>11} '
          f'{"pos_err_R":>11} {"rot_err_R":>11} {"pitchL":>7} {"pitchR":>7} '
          f'{"CoM_x":>9} {"marg_W":>8} {"marg_E":>8}')
    for dz in (-0.20, -0.30, -0.40, -0.50):
        seeds = None
        for dx in (-0.24, -0.16, -0.12, -0.08, -0.04, 0.00):
            r = LM.evaluate(body, q_base, float(dx), float(dz), flat=True, seeds=seeds,
                            foot_pos=foot_pos, foot_mat=foot_mat)
            seeds = {'L': r['qp'], 'R': r['qp']}
            d.qpos[:] = r['qp']
            d.qvel[:] = 0.0
            mujoco.mj_forward(m, d)
            perr, rerr, pitch = {}, {}, {}
            for s in ('L', 'R'):
                fid = m.body(FOOT[s]).id
                perr[s] = float(np.linalg.norm(foot_pos[s] - np.asarray(d.xpos[fid], float)))
                rerr[s] = float(np.linalg.norm(
                    rot_err(np.asarray(d.xmat[fid]).reshape(3, 3), foot_mat[s])))
                R = np.asarray(d.xmat[fid]).reshape(3, 3)
                pitch[s] = float(np.degrees(np.arccos(np.clip(R[2, 2], -1, 1))))
            print(f'{dz:+.3f} {dx:+.3f} {r["resid_m"]:15.3e} {perr["L"]:11.3e} {rerr["L"]:11.3e} '
                  f'{perr["R"]:11.3e} {rerr["R"]:11.3e} {pitch["L"]:7.2f} {pitch["R"]:7.2f} '
                  f'{r["com_x"]:9.4f} {r["com_x"] - SUP_LO:+8.4f} {SUP_HI - r["com_x"]:+8.4f}')
        print()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
