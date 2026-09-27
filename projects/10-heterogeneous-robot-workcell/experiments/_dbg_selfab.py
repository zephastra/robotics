"""A/B: is the 58.5 mm thumb-into-knee real, or is it an artefact of the audit's own controller?

THE SUSPICION, and why it is worth a measurement rather than an argument
-----------------------------------------------------------------------
`reports/w2-audit-05` reported the humanoid putting its thumb tip 58.5 mm inside its own knee at
step 60 and its elbow 25.1 mm inside its own table at step 97. Re-measuring the same pose with the
standing law evaluated EVERY STEP gives 0.044 mm settled and a worst transient of 3.4 mm. That is a
17x discrepancy on the same model, so one of the two measurements is describing something else.

The audit ran with `--no-drive`, and in the same round I found (and fixed) a defect in that probe:
`home_hold_ctrl` is a STATE-FEEDBACK law whose docstring says "evaluated every step", and the probe
computed it ONCE and reused it. Constant torque is not a standing law: the humanoid droops, and an
arm that droops while its hand is at its side goes THROUGH the hip and knee. So the prediction is
that the constant-ctrl arm reproduces the deep numbers and the per-step arm does not.

That is exactly what this measures. Same model, same seed state, same duration, ONE difference.
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

STEPS = 500                 # the audit's own window: 500 steps at dt 0.002
DT = None                   # read from the model


def bname(model, i):
    return mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, i) or f'<body{i}>'


def deepest(model, data, *, humanoid_only):
    worst = (0.0, None, None)
    for i in range(data.ncon):
        g1, g2 = int(data.contact.geom[i][0]), int(data.contact.geom[i][1])
        n1 = bname(model, int(model.geom_bodyid[g1]))
        n2 = bname(model, int(model.geom_bodyid[g2]))
        if humanoid_only and not (n1.startswith('h_') and n2.startswith('h_')):
            continue
        d = float(data.contact.dist[i])
        if d < worst[0]:
            worst = (d, n1, n2)
    return worst


def run(*, feedback):
    """One arm of the A/B. `feedback=True` re-evaluates the standing law every step."""
    bp3.install()
    _text, layout, model, _s, _k, _n, _a = mw.build_text()
    data = mujoco.MjData(model)
    qpos, ctrl, _a2, _notes = mw.merged_home(model, layout)
    data.qpos[:] = qpos
    data.ctrl[:] = ctrl
    dt = float(model.opt.timestep)

    worst_self = (0.0, None, None, None)
    worst_cross = (0.0, None, None, None)
    humanoid_height = []
    for step in range(STEPS):
        if feedback:
            data.ctrl[:] = np.asarray(mw.home_hold_ctrl(model, qpos, data.qpos, data.qvel),
                                      dtype=float).reshape(-1)
        # else: `ctrl` stays as the keyframe set it, which is what the audit did
        mujoco.mj_step(model, data)
        s = deepest(model, data, humanoid_only=True)
        if s[0] < worst_self[0]:
            worst_self = (s[0], s[1], s[2], step)
        c = deepest(model, data, humanoid_only=False)
        if c[0] < worst_cross[0]:
            worst_cross = (c[0], c[1], c[2], step)
        if step % 50 == 0:
            body = model.body('h_LINK_BASE').id
            humanoid_height.append(round(float(data.xpos[body][2]), 4))
    return {
        'feedback_each_step': feedback,
        'dt_s': dt, 'steps': STEPS, 'sim_s': round(STEPS * dt, 3),
        'worst_humanoid_self_mm': round(worst_self[0] * 1000, 3),
        'worst_self_pair': (worst_self[1], worst_self[2]),
        'worst_self_at_step': worst_self[3],
        'worst_any_mm': round(worst_cross[0] * 1000, 3),
        'worst_any_pair': (worst_cross[1], worst_cross[2]),
        'worst_any_at_step': worst_cross[3],
        'humanoid_base_z_trace': humanoid_height,
        'humanoid_base_z_final': humanoid_height[-1] if humanoid_height else None,
    }


def main():
    print('=== A: the audit\'s arm (constant ctrl, no feedback) ===')
    a = run(feedback=False)
    for k, v in a.items():
        print(f'   {k}: {v}')
    print()
    print('=== B: the corrected arm (standing law re-evaluated every step) ===')
    b = run(feedback=True)
    for k, v in b.items():
        print(f'   {k}: {v}')
    print()
    print('=== the difference ===')
    print(f"   worst humanoid self-contact: A {a['worst_humanoid_self_mm']} mm"
          f"  vs  B {b['worst_humanoid_self_mm']} mm")
    print(f"   humanoid base z:             A {a['humanoid_base_z_final']} m"
          f"  vs  B {b['humanoid_base_z_final']} m")
    print()
    if a['worst_humanoid_self_mm'] < -50.0 and b['worst_humanoid_self_mm'] > -10.0:
        print('   VERDICT: the deep figure belongs to the CONSTANT-CTRL arm. It reproduces the')
        print('   58.5 mm the audit reported, so the recorded finding is an artefact of that')
        print('   probe\'s own controller defect (a state-feedback law evaluated once), not a')
        print('   property of the humanoid at its home pose.')
    elif a['worst_humanoid_self_mm'] < -50.0 and b['worst_humanoid_self_mm'] < -50.0:
        print('   VERDICT: both arms show it. The finding is real and independent of the defect.')
    else:
        print('   VERDICT: neither arm reproduces the recorded figure; the audit measured')
        print('   something else again and the number must be re-attributed from scratch.')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
