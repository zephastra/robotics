"""Instrument ONE jam, instead of reasoning about it.

The W3 grid reports that some entries inside the declared catch never reach the window. Two
mechanisms are possible and they need different fixes:

  W1 a GEOMETRIC WEDGE -- the deck frame is yawed, its corner is wider than the gap, and the
     taper's face is a rigid world body, so the vehicle stalls. Diagnostic: the contact force on
     the frame rises while the chassis x stops advancing, and the frame's own yaw does NOT reduce.
  W2 A FORCE / FRICTION STALL -- the frame slides and the guide friction beats the taper's lateral
     thrust. Diagnostic: the frame's corner stays in contact and creeps, and the deck's lateral
     slide saturates against its travel limit.

Prints, per logged step: chassis x/y/yaw, the deck's own lateral and yaw joints, the frame centre,
and every contact on the frame (pair, penetration, normal force). The first contacts and the last
few steps are what decide it.
"""
import sys
import math
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'experiments'))
sys.path.insert(0, str(ROOT / 'src'))

import mujoco  # noqa: E402
import build_p3_world as bp3  # noqa: E402
import merge_world as mw  # noqa: E402
import build_w2_logistic_world as builder  # noqa: E402
import probe_w2_baseline as w2  # noqa: E402

WORLD = ROOT / 'assets' / 'world_w2_logistic.xml'
RADIUS = 0.04
KP = 6.0
TORQUE = 1.2
APPROACH_M = 1.15
SPEED = 0.25
TIMEOUT_S = 8.0


def names(model, index, obj):
    return mujoco.mj_id2name(model, obj, index) or f'?{index}'


def frame_contacts(model, data):
    body = model.body('c_deck').id
    subtree = {body}
    for b in range(model.nbody):
        if model.body_parentid[b] in subtree:
            subtree.add(b)
    out = []
    for i in range(data.ncon):
        c = data.contact[i]
        g1, g2 = int(c.geom[0]), int(c.geom[1])
        b1, b2 = int(model.geom_bodyid[g1]), int(model.geom_bodyid[g2])
        if b1 not in subtree and b2 not in subtree:
            continue
        force = np.zeros(6)
        mujoco.mj_contactForce(model, data, i, force)
        n = names(model, g1, mujoco.mjtObj.mjOBJ_GEOM)
        m = names(model, g2, mujoco.mjtObj.mjOBJ_GEOM)
        if 'w2_' not in n and 'w2_' not in m:
            continue                       # only the docking hardware, not the deck's own parts
        out.append((n, m, float(c.dist), abs(float(force[0]))))
    return sorted(out, key=lambda r: r[2])


def main(dy, dtheta_deg):
    bp3.install()
    model = mujoco.MjModel.from_xml_path(str(WORLD))
    data = mujoco.MjData(model)
    home, _c, _a, _n = mw.merged_home(model)
    home = np.asarray(home, dtype=float)
    mujoco.mj_resetDataKeyframe(model, data, 0)
    mujoco.mj_forward(model, data)

    chassis0 = float(data.xpos[model.body('n_base_link').id][0])
    home_slide = float(home[int(model.jnt_qposadr[model.joint('n_slide_x').id])])
    frame_x = float(data.geom_xpos[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM,
                                                     'c_deck_frame')][0])
    datum_face = float(data.geom_xpos[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM,
                                                       'w2_datum_face')][0]) - builder.STOP_HALF_X_M
    dock = chassis0 + (datum_face - (frame_x + float(builder.rig.DECK_FRAME_HALF[0])))
    start = dock - APPROACH_M

    data.qpos[:] = home
    data.qvel[:] = 0.0
    for joint, value in (('n_slide_x', home_slide + (start - chassis0)),
                         ('n_slide_y', dy),
                         ('n_yaw', math.radians(dtheta_deg))):
        data.qpos[int(model.jnt_qposadr[model.joint(joint).id])] = value
    mujoco.mj_forward(model, data)

    wheels = {s: w2.wheel_actuators(model, 'n_')[s] for s in ('left', 'right')}
    signs = {s: w2.wheel_forward_sign(model, data, f'n_wheel_{s}_joint', RADIUS) for s in wheels}
    print(f'wheel signs at this pose: {signs}')
    print(f'trial dy {dy * 1000:+.1f} mm  yaw {dtheta_deg:+.1f} deg  start chassis x {start:.6f}'
          f'  target {dock:.6f}')

    slide_adr = int(model.jnt_qposadr[model.joint('c_deck_slide_y').id])
    yaw_adr = int(model.jnt_qposadr[model.joint('c_deck_yaw').id])
    frame_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, 'c_deck_frame')
    steps = int(TIMEOUT_S / model.opt.timestep)
    seen = {}
    for step in range(steps):
        error = dock - float(data.xpos[model.body('n_base_link').id][0])
        command = float(np.clip(error / 0.4, -SPEED, SPEED))
        data.ctrl[:] = mw.home_hold_ctrl(model, home, data.qpos, data.qvel)
        for side, act in wheels.items():
            joint = model.joint(f'n_wheel_{side}_joint').id
            dof = int(model.jnt_dofadr[joint])
            want = (command / RADIUS) * signs[side]
            data.ctrl[act] = float(np.clip(KP * (want - float(data.qvel[dof])), -TORQUE, TORQUE))
        mujoco.mj_step(model, data)

        contacts = frame_contacts(model, data)
        for n, m, dist, fn in contacts:
            seen.setdefault((n, m), [step, dist, fn])
            seen[(n, m)] = [step, dist, fn]
        if step % 100 == 0 or (contacts and step % 20 == 0):
            ch = data.xpos[model.body('n_base_link').id]
            q = data.xquat[model.body('n_base_link').id]
            frame = data.geom_xpos[frame_id]
            rot = data.geom_xmat[frame_id].reshape(3, 3)
            fyaw = math.degrees(math.atan2(rot[1, 0], rot[0, 0]))
            print(f'  step {step:5d} t {data.time:5.3f}  ch x {float(ch[0]):.5f} y {float(ch[1]):+.5f} '
                  f'yaw {math.degrees(2 * math.atan2(q[3], q[0])):+7.2f}  '
                  f'frame y {float(frame[1]):+.5f} yaw {fyaw:+7.3f}  '
                  f'deck slid {float(data.qpos[slide_adr]) * 1000:+7.3f} mm '
                  f'yaw {math.degrees(float(data.qpos[yaw_adr])):+7.3f}')
            for n, m, dist, fn in contacts[:3]:
                print(f'          CONTACT {n} / {m}  dist {dist:+.6f}  fn {fn:9.3f}')
            if not contacts:
                print('          (no contact between the deck assembly and the docking hardware)')
        if abs(error) < 0.004:
            print(f'  reached the target at step {step}, t {data.time:.3f}')
            break

    print()
    print('=== every contact pair ever seen, with the LAST reading for each ===')
    for (n, m), (step, dist, fn) in sorted(seen.items(), key=lambda kv: kv[1][2]):
        print(f'  {n:26s} {m:26s} last step {step:5d} dist {dist:+.6f} fn {fn:9.3f}')
    final = float(data.xpos[model.body('n_base_link').id][0])
    print(f'  final chassis x {final:.6f}, travelled {final - start:.4f} of {APPROACH_M} m')


if __name__ == '__main__':
    main(float(sys.argv[1]) if len(sys.argv) > 1 else 0.030,
         float(sys.argv[2]) if len(sys.argv) > 2 else 0.5)
