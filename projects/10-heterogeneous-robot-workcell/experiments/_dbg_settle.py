"""Which dof is still moving after the abduction, and does it decay?

The P3 world gate's settle row went from PASS to FAIL when the arms were abducted: worst humanoid
|qvel| 0.1095 against a 0.10 limit. Two very different things produce that and they have different
remedies:

  * a DECAYING transient -- the arms are now free-hanging on a PD law (arm kp 120, kd 1) instead of
    resting on the hips, which damped them, so they swing and then settle. Then the answer is
    damping or time, and the peak is a settling artefact rather than a failure to settle;
  * a LIMIT CYCLE -- the arms keep oscillating. Then the pose change introduced a real problem and
    has to be revisited.

The limit is NOT touched here either way; this only measures which of the two it is, and names the
dof so the answer is about a joint rather than about "the humanoid".
"""
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'experiments'))
sys.path.insert(0, str(ROOT / 'src'))

import mujoco  # noqa: E402
import numpy as np  # noqa: E402

import build_p3_world as bp3  # noqa: E402
import merge_world as mw  # noqa: E402

SECONDS = 8.0


def jname(model, dof):
    return mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT,
                             int(model.dof_jntid[dof])) or f'dof{dof}'


def main():
    bp3.install()
    _t, layout, model, _s, _k, _n, _a = mw.build_text()
    data = mujoco.MjData(model)
    qpos, ctrl, _a2, _notes = mw.merged_home(model, layout)
    data.qpos[:] = qpos
    data.ctrl[:] = ctrl

    # the humanoid's dofs, so the trace is about that instance and not about the whole cell
    humanoid = [d for d in range(model.nv)
                if (mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY,
                                      int(model.dof_bodyid[d])) or '').startswith('h_')]
    steps = int(SECONDS / model.opt.timestep)
    print(f'{SECONDS} s, dt {model.opt.timestep}, {len(humanoid)} humanoid dofs')
    print(f"{'t s':>6} {'worst |qvel|':>13} {'limit pass':>11}  worst dof")
    peaks = {}
    trace = []
    for step in range(steps):
        data.ctrl[:] = np.asarray(mw.home_hold_ctrl(model, qpos, data.qpos, data.qvel),
                                  dtype=float).reshape(-1)
        mujoco.mj_step(model, data)
        if step % int(0.25 / model.opt.timestep) == 0:
            worst_dof, worst = max(((d, abs(float(data.qvel[d]))) for d in humanoid),
                                   key=lambda kv: kv[1])
            peaks[worst_dof] = max(peaks.get(worst_dof, 0.0), worst)
            trace.append((round(float(data.time), 3), round(worst, 5), jname(model, worst_dof)))
            print(f'{data.time:6.2f} {worst:13.5f} {"PASS" if worst <= 0.10 else "FAIL":>11}  '
                  f'{jname(model, worst_dof)}')
    print()
    print('dofs that ever held the maximum, with their own peak:')
    for dof, peak in sorted(peaks.items(), key=lambda kv: -kv[1]):
        print(f'   {jname(model, dof):<26} {peak:.5f}')
    tail = [t[1] for t in trace if t[0] >= 3.0]
    print()
    print(f'from 3 s onward the max is {max(tail):.5f} at {trace[0][0]}.. and the LAST value is '
          f'{trace[-1][1]:.5f}')
    if trace[-1][1] < max(tail) * 0.8:
        print('VERDICT: decaying. The 3 s window is catching a settling transient, not a failure '
              'to settle, and adding damping to the arm joints (a declared assembly adjustment, '
              'like the abduction) is the remedy that keeps every threshold as it is.')
    else:
        print('VERDICT: NOT decaying. The pose change introduced a real oscillation and the '
              'abduction has to be revisited rather than damped away.')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
