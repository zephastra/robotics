"""The self-penetration is a TRANSIENT, not a pose. Measure where it comes from.

Recorded as "the humanoid model at its home pose putting its own thumb tip inside its own knee".
Reproduced here, that sentence is wrong in a way that changes the fix: at the keyframe the cell has
ZERO deep contacts, and at the settled pose it has 0.044 mm. The 58.5 mm happens at STEP 60 of
settling, and the 25.1 mm elbow-into-table at STEP 97. So the arm is being driven THROUGH the knee
during the first 0.2 s and then the solver pushes it back out.

That distinction decides the remedy. A pose fix would move the keyframe and change nothing. What has
to be understood is what COMMANDS the arm during those 0.2 s, and this prints exactly that: per step,
the worst self pair and the joint values of the two bones involved.
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


def name(model, kind, index):
    return mujoco.mj_id2name(model, kind, index) or f'<{index}>'


def main():
    bp3.install()
    _text, layout, model, _step, _kin, _notes, _applied = mw.build_text()
    data = mujoco.MjData(model)
    qpos, ctrl, _a, home_notes = mw.merged_home(model, layout)
    data.qpos[:] = qpos

    print('world: assets/world_p3_cell.xml (rebound through build_p3_world)')
    print('home notes:', home_notes)
    print()

    # which actuators the standing law commands, and how hard, at t = 0
    ctrl0 = np.asarray(mw.home_hold_ctrl(model, qpos, data.qpos, data.qvel), dtype=float).reshape(-1)
    humanoid = [(a, name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, a))
                for a in range(model.nu)
                if (name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, a) or '').startswith('h_')]
    print(f'humanoid actuators driven by the standing law: {len(humanoid)}')
    hot = sorted(((abs(float(ctrl0[a])), n, float(ctrl0[a])) for a, n in humanoid), reverse=True)[:8]
    print('largest |ctrl| at t=0:')
    for mag, n, v in hot:
        print(f'   {n:<34} {v:+10.3f}   (ctrlrange {list(model.actuator_ctrlrange[a]) if False else ""})')
    print()

    # the target the law is driving the limb joints toward, versus where the keyframe put them
    print('=== keyframe vs what the law drives toward, worst 8 limb joints ===')
    deltas = []
    for a, n in humanoid:
        j = int(model.actuator_trnid[a][0])
        if model.actuator_trntype[a] != mujoco.mjtTrn.mjTRN_JOINT or j < 0:
            continue
        adr = int(model.jnt_qposadr[j])
        target = float(ctrl0[a])                      # for a position/muscle actuator the ctrl IS the target
        now = float(data.qpos[adr])
        deltas.append((abs(target - now), n, now, target))
    deltas.sort(reverse=True)
    for d, n, now, target in deltas[:8]:
        print(f'   {n:<34} keyframe {now:+8.4f}  target {target:+8.4f}  move {d:.4f}')
    print()

    # step and watch the worst self pair
    print('=== per step, the worst humanoid-self pair ===')
    print(f"{'step':>5} {'t':>7} {'depth mm':>10}  pair")
    worst_overall = (0.0, None, None)
    for step in range(int(1.2 / model.opt.timestep)):
        data.ctrl[:] = np.asarray(mw.home_hold_ctrl(model, qpos, data.qpos, data.qvel),
                                  dtype=float).reshape(-1)
        mujoco.mj_step(model, data)
        worst = (0.0, None, None)
        for i in range(data.ncon):
            g1, g2 = int(data.contact.geom[i][0]), int(data.contact.geom[i][1])
            b1 = int(model.geom_bodyid[g1])
            b2 = int(model.geom_bodyid[g2])
            n1 = name(model, mujoco.mjtObj.mjOBJ_BODY, b1)
            n2 = name(model, mujoco.mjtObj.mjOBJ_BODY, b2)
            if not (n1.startswith('h_') and n2.startswith('h_')):
                continue
            d = float(data.contact.dist[i])
            if d < worst[0]:
                worst = (d, n1, n2)
        if worst[0] < worst_overall[0]:
            worst_overall = (worst[0], worst[1], worst[2])
        if step % 10 == 0 or worst[0] < -0.002:
            print(f'{step:5d} {float(data.time):7.3f} {worst[0] * 1000:10.3f}  '
                  f'{worst[1]} x {worst[2]}')
    print()
    print(f'worst over the transit: {worst_overall[0] * 1000:.3f} mm  '
          f'({worst_overall[1]} x {worst_overall[2]})')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
