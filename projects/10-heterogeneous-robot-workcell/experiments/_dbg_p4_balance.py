"""Diagnostic: WHY does the squat fall over? Nothing is judged here.

Three hypotheses, all distinguishable by measurement:

  H1 ROLL      -- the triple uses ONE amplitude for hip, knee and ankle (the probe passed a single
                  scalar to all three). For the sole to stay flat the three are not independent:
                  the foot's world pitch is the sum of their contributions, so an equal-amplitude
                  triple rolls the foot by twice the amplitude. A foot on its toe or heel has no
                  support polygon left, whatever the CoM does.
  H2 CoM       -- the CoM leaves the foot's footprint (backwards, since hip flexion carries the
                  pelvis back while the torso stays rigid on top of it).
  H3 TORQUE    -- the joints run out of actuator range and the PD simply cannot hold the pose.

The instrument that was supposed to answer H2/H3 was broken: `Body.foot_load` summed
`abs(contact.dist) * 0.0`, which is zero for every state -- a row that reads "0 N under both feet"
in every world, including one where the feet are carrying the whole body.
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
from probe_p4_crouch import Body, SQUAT_JOINTS  # noqa: E402

BODY = mujoco.mjtObj.mjOBJ_BODY
GEOM = mujoco.mjtObj.mjOBJ_GEOM
JOINT = mujoco.mjtObj.mjOBJ_JOINT


def name(m, obj, i):
    # `mj_id2name(m, type, id)` -- the MODEL is the first argument. Called without it, MuJoCo
    # raises a TypeError rather than returning None, which is the good kind of failure.
    return mujoco.mj_id2name(m, obj, i) or ''


def inventory(m):
    print('=== leg + foot inventory ===')
    for j in range(m.njnt):
        n = name(m, JOINT, j)
        if n.startswith('h_J') and ('J0' in n):
            print(f'  joint {n:28s} type={int(m.jnt_type[j])} '
                  f'qposadr={int(m.jnt_qposadr[j])} dofadr={int(m.jnt_dofadr[j])} '
                  f'range=[{m.jnt_range[j][0]:+.3f},{m.jnt_range[j][1]:+.3f}]')
    for b in range(m.nbody):
        n = name(m, BODY, b)
        if 'FOOT' not in n.upper():
            continue
        print(f'  body  {n}')
        for g in range(m.ngeom):
            if int(m.geom_bodyid[g]) == b:
                print(f'      geom {name(m, GEOM, g):34s} type={int(m.geom_type[g])} '
                      f'size={np.round(m.geom_size[g], 4)} pos={np.round(m.geom_pos[g], 4)}')
    print(f'  nq={m.nq} nv={m.nv} nu={m.nu} timestep={m.opt.timestep} gravity={m.opt.gravity}')
    # total mass, and the mass of each foot, so "load" has a scale to compare against
    print(f'  total mass {m.body_mass.sum():.4f} kg -> weight '
          f'{m.body_mass.sum() * abs(m.opt.gravity[2]):.2f} N')


def com(body):
    """Whole-body CoM. `subtree_com[0]` after mj_comPos is the CoM of the root subtree = the
    whole robot, and it accounts for bodies that are not simple rigid links."""
    mujoco.mj_comPos(body.m, body.d)
    return np.array(body.d.subtree_com[0], dtype=float)


FOOT_BOXES = {}


def foot_boxes(m):
    """The world-space x/y extent of every foot geom, as a box.

    The support polygon is the union of the feet's footprints, so its x-range IS the contact
    interval the CoM has to stay inside. Computed from the geom's own half-sizes and world
    rotation -- not from the contact list, which is empty exactly when the interesting thing
    (the foot leaving the floor) happens.
    """
    key = id(m)
    if key in FOOT_BOXES:
        return FOOT_BOXES[key]
    boxes = []
    for g in range(m.ngeom):
        bn = name(m, BODY, int(m.geom_bodyid[g]))
        if 'FOOT' not in bn.upper():
            continue
        boxes.append((bn, g))
    FOOT_BOXES[key] = boxes
    return boxes


def boxes_world(m, d):
    out = {}
    for bn, g in foot_boxes(m):
        half = m.geom_size[g][:3]
        R = d.geom_xmat[g].reshape(3, 3)
        c = d.geom_xpos[g]
        corners = []
        for sx in (-1, 1):
            for sy in (-1, 1):
                for sz in (-1, 1):
                    corners.append(c + R @ (sx * half[0], sy * half[1], sz * half[2]))
        corners = np.array(corners)
        key = 'L' if bn.endswith('_L') else 'R'
        if key not in out:
            out[key] = {'x': [corners[:, 0].min(), corners[:, 0].max()],
                        'y': [corners[:, 1].min(), corners[:, 1].max()],
                        'z': [corners[:, 2].min(), corners[:, 2].max()],
                        'normal': R[:, 2].copy(),
                        'names': [bn]}
        else:
            out[key]['x'][0] = min(out[key]['x'][0], corners[:, 0].min())
            out[key]['x'][1] = max(out[key]['x'][1], corners[:, 0].max())
            out[key]['y'][0] = min(out[key]['y'][0], corners[:, 1].min())
            out[key]['y'][1] = max(out[key]['y'][1], corners[:, 1].max())
            out[key]['z'][0] = min(out[key]['z'][0], corners[:, 2].min())
            out[key]['z'][1] = max(out[key]['z'][1], corners[:, 2].max())
            out[key]['names'].append(bn)
    return out


def foot_normal_force(m, d, box):
    """Vertical load under one foot, from the solver's contact forces.

    `mj_contactForce` gives the 6-D force in the CONTACT frame; component 0 is the normal force
    along the contact normal. That is the only place the number exists -- the contact list itself
    only carries the gap, and after the foot lifts the list is empty.
    """
    res = np.zeros(6, dtype=float)
    total, count = 0.0, 0
    for i in range(d.ncon):
        c = d.contact[i]
        for near, far in ((int(c.geom[0]), int(c.geom[1])), (int(c.geom[1]), int(c.geom[0]))):
            bn = name(m, BODY, int(m.geom_bodyid[near]))
            if bn not in box['names']:
                continue
            other = name(m, BODY, int(m.geom_bodyid[far]))
            if other.startswith('h_'):
                continue                       # foot-on-foot is not floor load
            mujoco.mj_contactForce(m, d, i, res)
            total += abs(float(res[0]))
            count += 1
    return total, count


def joint_state(m, d):
    out = {}
    for side, names in SQUAT_JOINTS.items():
        for n in names:
            j = m.joint('h_' + n).id
            out[n] = float(d.qpos[int(m.jnt_qposadr[j])])
    return out


def snapshot(body, standing, t):
    m, d = body.m, body.d
    c = com(body)
    boxes = boxes_world(m, d)
    load, counts = {}, {}
    for side, box in boxes.items():
        load[side], counts[side] = foot_normal_force(m, d, box)
    support_x = [min(b['x'][0] for b in boxes.values()), max(b['x'][1] for b in boxes.values())]
    support_y = [min(b['y'][0] for b in boxes.values()), max(b['y'][1] for b in boxes.values())]
    row = {'t': round(t, 3), 'base_z': round(body.base_z(), 5),
           'drop_mm': round((standing - body.base_z()) * 1000, 2),
           'com_x': round(float(c[0]), 5), 'com_y': round(float(c[1]), 5),
           'com_z': round(float(c[2]), 5),
           'support_x': [round(v, 4) for v in support_x],
           'support_y': [round(v, 4) for v in support_y],
           'margin_x': [round(float(c[0]) - support_x[0], 4), round(support_x[1] - float(c[0]), 4)],
           'margin_y': [round(float(c[1]) - support_y[0], 4), round(support_y[1] - float(c[1]), 4)],
           'load_N': {k: round(v, 1) for k, v in load.items()},
           'ncon': counts,
           'foot_pitch_deg': {k: round(float(np.degrees(np.arccos(
               np.clip(b['normal'][2], -1.0, 1.0)))), 2) for k, b in boxes.items()},
           'foot_z': {k: [round(v, 4) for v in b['z']] for k, b in boxes.items()},
           'joints': {k.split("_", 2)[-1]: round(v, 4) for k, v in joint_state(m, d).items()},
           'leg_vel': round(body.leg_vel(), 4),
           'self_mm': round(body.deepest_self()[0], 3)}
    return row


def main():
    body = Body()
    m = body.m
    inventory(m)
    print()

    standing = None
    body.reset()
    body.run(0.6)
    standing = body.base_z()
    print(f'standing pelvis z = {standing:.5f}')
    print('stand snapshot:', snapshot(body, standing, 0.0))
    print()

    signs = (-1.0, 1.0, 1.0)          # as measured by probe_p4_crouch
    for amp in (0.10, 0.217):
        delta = {}
        for names in SQUAT_JOINTS.values():
            for s, n in zip(signs, names):
                delta[n] = s * amp
        print(f'=== squat triple (hip,knee,ankle) signs {signs} amplitude {amp} rad ===')
        print(f'    delta per joint: { {k.split("_",2)[-1]: round(v,4) for k, v in delta.items()} }')
        body.reset()
        body.run(0.6)
        standing = body.base_z()
        n_steps = int(2.5 / m.opt.timestep)
        every = max(1, int(0.25 / m.opt.timestep))
        for i in range(n_steps):
            body.tick(delta)
            if i % every == 0:
                row = snapshot(body, standing, i * m.opt.timestep)
                print(f"  t={row['t']:.2f} drop={row['drop_mm']:+7.1f}mm "
                      f"com_x={row['com_x']:.4f} sup_x={row['support_x']} "
                      f"marg_x={row['margin_x']} load={row['load_N']} "
                      f"pitch={row['foot_pitch_deg']} foot_z={row['foot_z']} "
                      f"legvel={row['leg_vel']:.3f} self={row['self_mm']:.2f}")
        print()

    # ---- what each joint DOES to the foot pitch, so the flat-foot relation can be derived ----
    print('=== single-joint sensitivity: how much foot pitch does 0.1 rad of each joint buy? ===')
    body.reset()
    body.run(0.6)
    for n in SQUAT_JOINTS['L']:
        body.reset()
        body.run(0.6)
        delta = {n: 0.1}
        body.run(1.5, delta)
        boxes = boxes_world(m, body.d)
        pitch = {k: float(np.degrees(np.arccos(np.clip(b['normal'][2], -1.0, 1.0))))
                 for k, b in boxes.items()}
        print(f'  +0.1 rad on {n:22s} -> foot pitch {pitch}, '
              f'base_z {body.base_z():.4f}, com_x {com(body)[0]:.4f}')
    print()
    print('=== the same for the R side ===')
    for n in SQUAT_JOINTS['R']:
        body.reset()
        body.run(0.6)
        delta = {n: 0.1}
        body.run(1.5, delta)
        boxes = boxes_world(m, body.d)
        pitch = {k: float(np.degrees(np.arccos(np.clip(b['normal'][2], -1.0, 1.0))))
                 for k, b in boxes.items()}
        print(f'  +0.1 rad on {n:22s} -> foot pitch {pitch}, '
              f'base_z {body.base_z():.4f}, com_x {com(body)[0]:.4f}')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
