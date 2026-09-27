"""The fixed-arm pick-and-place rig, built on the upstream Franka Emika Panda model.

PROBE-A asks for one thing (`docs/TEST_AND_ACCEPTANCE.md`):

    | PROBE-A | 固定臂从桌面抓起并放下一个零件 |
    | 必需证据 | 夹爪接触、物体离台、释放后稳定；规划附着不能充当接触 |

Four rules are carried over from the C branch, because each of them cost rounds there:

1. **Derive, do not type.** Anything the model can tell us -- the grasp frame, the finger
   stroke, the home hand orientation -- is computed from the loaded model and asserted by a
   test. Two hand-computed derived quantities were wrong in the C branch, both times by
   dropping one term.
2. **Address joints through the model, not by position.** The first version of this file took
   the arm's Jacobian columns as `[:, :7]`. Once the payload's freejoint was declared before
   the arm, that silently addressed SIX PAYLOAD DOFS AND ONE ARM JOINT. Nothing crashed; the
   numbers were simply wrong. DOFs are now read from `jnt_dofadr`.
3. **The generated bodies go LAST in `<worldbody>`.** Not cosmetic: qpos ordering follows
   declaration order, so inserting the payload first shifts every arm joint and makes the
   upstream `home` keyframe land on the payload's quaternion instead of on the arm.
4. **A planned attach is not a grasp.** The payload is a free body: no equality, no weld, no
   runtime teleport. The one `<equality>` in the upstream model couples the two finger
   joints -- that is the gripper's own mechanism, so the anti-fake-attach check must be
   scoped to *the payload*, not to "there is no equality".
"""
import argparse
import hashlib
import math
import pathlib
import sys

import numpy as np
import mujoco

ROOT = pathlib.Path(__file__).resolve().parents[1]
PANDA_DIR = ROOT / 'assets' / 'panda'
PANDA_XML = PANDA_DIR / 'panda.xml'
WORLD = ROOT / 'assets' / 'world_arm_a.xml'
PANDA_MESH_SUBDIR = 'panda/assets'   # what `meshdir` must become, relative to assets/
UPSTREAM_MESHDIR = 'assets'

MODEL_NAME = 'panda_arm_a'
PAYLOAD = 'payload'
PAYLOAD_FREEJOINT = 'payload_free'

# --- the part under test ---------------------------------------------------------------
# A rectangular part, wider ALONG THE CLOSING AXIS than across it. That is not decoration:
# the gripper's closing force is a position servo, force = 100 * (target - blocked), so a
# wider part lets the fingers stop further from the servo target and therefore press harder.
# Measured: with the closing axis along world x, a 30 mm cube blocks the fingers at
# q = 0.0135 and yields only 1.35 N of tendon force, which is a thin margin; 40 mm gives
# 1.85 N. The pads are 17 x 17 mm, so the gripped faces (30 x 30 mm) cover them.
# x = the closing axis, y/z = the gripped faces. The Z half-extent is not free: the fingers
# reach 21.6 mm below the grasp frame (measured, `gripper_reach_below_grasp`), so a grasp
# centred on a short part drives the fingertip INTO the table, and the friction there is what
# stops the gripper closing -- measured, the whole CLOSE phase had the finger on the table in
# 125/126 to 174/175 samples, and at mu=1.0 the extra friction made the grip FAIL while mu=0.6
# passed. A 50 mm tall part lets the grasp sit at 0.225, which clears the surface by 3.4 mm.
# A test asserts `payload_rest_z() >= min_grasp_frame_z(...)` so this cannot silently recur.
PAYLOAD_HALF = (0.020, 0.015, 0.025)
PAYLOAD_MASS = 0.03

# --- the work surface ------------------------------------------------------------------
# The measured home pose puts the finger-length axis straight down, so this arm grasps from
# above natively; no reorientation is needed.
TABLE_TOP_Z = 0.20
TABLE_CENTRE_XY = (0.62, 0.0)
TABLE_HALF = (0.26, 0.24, 0.01)
PICK_XY = (0.55, -0.13)
PLACE_XY = (0.55, 0.13)

# Friction is a SWEPT parameter, not a declared constant. The C branch found the old tray
# asset declaring 1.0 with no material basis anywhere; rather than inherit that, the probe
# overwrites BOTH sides of every relevant interface and the judge checks that it did.
DEFAULT_FRICTION = 0.60
FRICTION_SWEEP = (0.30, 0.60, 1.00)

# --- gripper --------------------------------------------------------------------------
# actuator8 is a position servo on the `split` tendon: force = 100*(target - length), with
# ctrl in [0, 255] mapping to a tendon-length target in [0, 0.04]. The tendon is
# 0.5*q1 + 0.5*q2 and the joint equality keeps q1 == q2, so target length == target joint
# value; ctrl 255 is OPEN and ctrl 0 is CLOSED.
GRIPPER_CTRL_OPEN = 255.0
GRIPPER_CTRL_CLOSED = 0.0
GRIPPER_CTRL_RANGE = (0.0, 255.0)
GRIPPER_ACTUATOR = 'actuator8'

# --- state machine budget, in simulated seconds ----------------------------------------
STATE_TIMEOUT = {
    'APPROACH': 6.0, 'DESCEND': 5.0, 'CLOSE': 4.0, 'LIFT': 5.0,
    'CARRY': 8.0, 'LOWER': 5.0, 'RELEASE': 3.0, 'RETREAT': 6.0, 'HOLD': 4.0,
}
COORDINATE_PATH = ('APPROACH', 'DESCEND', 'CLOSE', 'LIFT', 'CARRY',
                   'LOWER', 'RELEASE', 'RETREAT', 'HOLD', 'DONE')
DELIVERY_STAGES = ('LIFT', 'CARRY', 'LOWER')
FAULT_STATES = ('GRIP_FAILED', 'PAYLOAD_LOST', 'TIMEOUT', 'STALLED')

# Heights, measured from the part's own rest height on the table.
PREGRASP_CLEARANCE = 0.10
GRASP_INSET = 0.0
LIFT_HEIGHT = 0.12
RETREAT_HEIGHT = 0.15
HOLD_SECONDS = 1.5

# Thresholds. DIAGNOSTIC values in the sense TEST_AND_ACCEPTANCE.md section 3 means: measured
# first, written down second, never inferred from one success.
MIN_CONTACT_SAMPLES = 60      # sustained finger<->payload contact while carrying
LIFT_MARGIN = 0.05            # payload must rise at least this far above its rest height
CARRIED_SLIP_MAX = 0.020      # payload must not slide more than this in the hand frame
PLACE_TOL_XY = 0.030          # where it ends up, versus the declared place target
RELEASE_DRIFT_MAX = 0.006     # displacement over the hold window after release
RELEASE_SPEED_MAX = 0.020     # peak speed over the hold window after release
TOUCHDOWN_TOL = 0.006         # final z versus table surface + half the part


def _skew(v):
    return np.array([[0.0, -v[2], v[1]],
                     [v[2], 0.0, -v[0]],
                     [-v[1], v[0], 0.0]])


def _rot_vector(R):
    """Rotation vector (axis * angle) of a rotation matrix, in the world frame."""
    c = (float(np.trace(R)) - 1.0) / 2.0
    c = max(-1.0, min(1.0, c))
    ang = math.acos(c)
    if ang < 1e-9:
        return np.zeros(3)
    if abs(math.pi - ang) < 1e-6:
        A = (R + np.eye(3)) / 2.0
        axis = np.sqrt(np.maximum(np.diag(A), 0.0))
        axis[1] *= 1.0 if R[0, 1] >= 0 else -1.0
        axis[2] *= 1.0 if R[0, 2] >= 0 else -1.0
        n = np.linalg.norm(axis)
        return np.zeros(3) if n < 1e-9 else axis / n * ang
    axis = np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]])
    return axis / (2.0 * math.sin(ang)) * ang


def table_centre_z():
    return TABLE_TOP_Z - TABLE_HALF[2]


def payload_rest_z():
    return TABLE_TOP_Z + PAYLOAD_HALF[2]


def gripper_min_world_z(model, data):
    """Lowest world z of any gripper geometry, from actual mesh vertices.

    Not a bounding sphere and not a number read off a drawing: the first estimate of this was
    15.5 mm from assuming the finger mesh ended at the pads, and the measurement is 21.6 mm.
    """
    bundle = ids(model)
    lo = None
    for g in bundle['gripper_geoms']:
        xpos = np.array(data.geom_xpos[g], dtype=float)
        xmat = np.array(data.geom_xmat[g], dtype=float).reshape(3, 3)
        if model.geom_type[g] == mujoco.mjtGeom.mjGEOM_MESH:
            did = int(model.geom_dataid[g])
            vadr, vnum = int(model.mesh_vertadr[did]), int(model.mesh_vertnum[did])
            verts = np.array(model.mesh_vert[vadr:vadr + vnum], dtype=float)
            z = float((verts @ xmat.T + xpos)[:, 2].min())
        else:
            # AABB half-extent of a box along world z: sum |R[2, i]| * size[i]
            half = float(np.abs(xmat[2, :]) @ np.asarray(model.geom_size[g], dtype=float))
            z = float(xpos[2] - half)
        lo = z if lo is None else min(lo, z)
    return lo


def gripper_reach_below_grasp(model, data):
    """How far the lowest gripper vertex sits below the grasp frame."""
    return float(grasp_point(model, data)[2] - gripper_min_world_z(model, data))


def min_grasp_frame_z(model, data):
    """The lowest grasp-frame height at which the fingers still clear the work surface."""
    return TABLE_TOP_Z + gripper_reach_below_grasp(model, data)


def payload_qpos_adr(model):
    return int(model.jnt_qposadr[model.joint(PAYLOAD_FREEJOINT).id])


# =======================================================================================
# scene generation
# =======================================================================================

def world_text():
    """The generated scene: upstream panda.xml with a work surface and one part added.

    Exactly two substitutions are made to the upstream text:

      * `meshdir="assets"` -> `meshdir="panda/assets"`, because the world file lands one
        level higher than the model. A world written into the wrong directory silently loses
        its meshes -- `make_world.py` records the same trap.
      * `model="panda"` -> `model="panda_arm_a"`, so a report cannot be mistaken for a run of
        the upstream model.

    Nothing inside the upstream `<worldbody>` is modified. The added bodies are inserted
    immediately BEFORE `</worldbody>` -- deliberately last, so the arm keeps qpos slots 0..8
    and the upstream `home` keyframe still means what it says.
    """
    src = PANDA_XML.read_text(encoding='utf-8')
    if src.count(f'meshdir="{UPSTREAM_MESHDIR}"') != 1:
        raise ValueError('upstream meshdir attribute is not where it was expected')
    src = src.replace(f'meshdir="{UPSTREAM_MESHDIR}"', f'meshdir="{PANDA_MESH_SUBDIR}"')
    if src.count('model="panda"') != 1:
        raise ValueError('upstream model attribute is not where it was expected')
    src = src.replace('model="panda"', f'model="{MODEL_NAME}"')

    hx, hy, hz = TABLE_HALF
    cx, cy = TABLE_CENTRE_XY
    px, py = PICK_XY
    qx, qy = PLACE_XY
    phx, phy, phz = PAYLOAD_HALF
    fr = f'{DEFAULT_FRICTION} 0.005 0.0001'

    insert = f'''
    <!-- ===== generated by experiments/arm_rig.py -- do not hand-edit ===== -->
    <geom name="floor" type="plane" size="3 3 0.05" pos="0 0 0" rgba="0.34 0.34 0.38 1"
          friction="{fr}"/>
    <body name="table" pos="{cx} {cy} {table_centre_z()}">
      <geom name="table_top" type="box" size="{hx} {hy} {hz}" rgba="0.55 0.44 0.29 1"
            friction="{fr}"/>
    </body>
    <body name="{PAYLOAD}" pos="{px} {py} {payload_rest_z()}">
      <!-- a SEPARATE part, free in the world: no weld, no equality, nothing holding it -->
      <freejoint name="{PAYLOAD_FREEJOINT}"/>
      <geom name="{PAYLOAD}_box" type="box" size="{phx} {phy} {phz}" mass="{PAYLOAD_MASS}"
            rgba="0.20 0.55 0.85 1" friction="{fr}"/>
    </body>
    <!-- the declared place target; a site, so it cannot collide with anything -->
    <site name="place_target" pos="{qx} {qy} {TABLE_TOP_Z}" size="0.012" rgba="0 0.8 0.2 0.5"/>
'''
    anchor = '</worldbody>'
    if src.count(anchor) != 1:
        raise ValueError('upstream </worldbody> is not where it was expected')
    return src.replace(anchor, insert + anchor)


def build(check=False):
    """Write (or verify) the generated world. Refuses if the upstream model has moved."""
    before = hashlib.sha256(PANDA_XML.read_bytes()).hexdigest()
    text = world_text()
    if check:
        if not WORLD.is_file():
            print(f'MISSING: {WORLD.relative_to(ROOT)} has not been generated')
            return 1
        if WORLD.read_text(encoding='utf-8') == text:
            print(f'OK: {WORLD.relative_to(ROOT)} matches its source '
                  f'(sha256 {hashlib.sha256(WORLD.read_bytes()).hexdigest()[:12]})')
            return 0
        print('STALE: the generated world differs from what its source produces')
        return 1
    WORLD.write_text(text, encoding='utf-8')
    if hashlib.sha256(PANDA_XML.read_bytes()).hexdigest() != before:
        raise RuntimeError('the upstream model changed while generating; it must not move')
    print(f'wrote {WORLD.relative_to(ROOT)}')
    print(f'  upstream {PANDA_XML.relative_to(ROOT)} untouched (sha256 {before[:12]})')
    print(f'  world sha256 {hashlib.sha256(WORLD.read_bytes()).hexdigest()[:12]}')
    return 0


# =======================================================================================
# model queries -- every geometry number the rest of the code uses comes from here
# =======================================================================================

def ids(model):
    """id bundle, looked up once. DOF addresses come from the model, never from position."""
    out = {
        'hand': model.body('hand').id,
        'left_finger': model.body('left_finger').id,
        'right_finger': model.body('right_finger').id,
        'payload': model.body(PAYLOAD).id,
        'table': model.body('table').id,
    }
    out['arm_joints'] = [model.joint(f'joint{i}').id for i in range(1, 8)]
    out['finger_joints'] = [model.joint(n).id for n in ('finger_joint1', 'finger_joint2')]
    out['arm_qadr'] = np.array([model.jnt_qposadr[j] for j in out['arm_joints']])
    out['arm_dof'] = np.array([model.jnt_dofadr[j] for j in out['arm_joints']])
    out['finger_qadr'] = np.array([model.jnt_qposadr[j] for j in out['finger_joints']])
    out['finger_dof'] = np.array([model.jnt_dofadr[j] for j in out['finger_joints']])
    out['payload_joint'] = model.joint(PAYLOAD_FREEJOINT).id
    out['payload_qadr'] = int(model.jnt_qposadr[out['payload_joint']])
    out['payload_dof'] = int(model.jnt_dofadr[out['payload_joint']])
    out['gripper_bodies'] = (out['hand'], out['left_finger'], out['right_finger'])
    out['payload_geoms'] = [g for g in range(model.ngeom)
                            if int(model.geom_bodyid[g]) == out['payload']]
    out['gripper_geoms'] = [g for g in range(model.ngeom)
                            if int(model.geom_bodyid[g]) in out['gripper_bodies']]
    out['table_geoms'] = [g for g in range(model.ngeom)
                          if int(model.geom_bodyid[g]) == out['table']]
    return out


def load(friction=DEFAULT_FRICTION, path=None):
    """Compile the scene and apply the swept friction to every interface that matters."""
    import mujoco
    model = mujoco.MjModel.from_xml_path(str(path or WORLD))
    data = mujoco.MjData(model)
    if friction is not None:
        apply_friction(model, friction)
    return model, data


def apply_friction(model, value):
    """Overwrite BOTH sides of every contact that can matter, so mu is unambiguous.

    Overwriting one side only leaves the effective coefficient as the pair maximum, which is
    how a "swept" parameter can quietly not be swept at all.
    """
    names = ('floor', 'table_top', f'{PAYLOAD}_box')
    touched = []
    for g in range(model.ngeom):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, g) or ''
        body = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY,
                                 int(model.geom_bodyid[g])) or ''
        if name in names or 'finger' in body:
            model.geom_friction[g][0] = value
            touched.append(name or body)
    if len(touched) < 6:
        raise RuntimeError(f'friction override reached only {len(touched)} geoms: {touched}')
    return touched


def _pad_geom(model, side):
    """The main pad of one finger: the largest box on that finger body."""
    import mujoco
    b = model.body(side).id
    boxes = [g for g in range(model.ngeom)
             if int(model.geom_bodyid[g]) == b
             and model.geom_type[g] == mujoco.mjtGeom.mjGEOM_BOX]
    if not boxes:
        raise RuntimeError(f'{side} has no box pads')
    return max(boxes, key=lambda g: float(np.prod(model.geom_size[g])))


def grasp_local_offset(model, data):
    """Pad-centre midpoint expressed in the HAND body frame.

    Computed from forward kinematics rather than by composing XML offsets by hand: the first
    version read `geom_pos` straight out of the model, which is the position in the FINGER
    body frame, and returned (0, 0.0055, 0.0445) instead of (0, 0, 0.1029). It is a constant
    of the model, so any pose will do -- a test asserts pose-independence.
    """
    import mujoco
    hand = model.body('hand').id
    mujoco.mj_forward(model, data)
    R = np.array(data.xmat[hand], dtype=float).reshape(3, 3)
    p = np.array(data.xpos[hand], dtype=float)
    centres = [np.array(data.geom_xpos[_pad_geom(model, s)])
               for s in ('left_finger', 'right_finger')]
    return R.T @ ((centres[0] + centres[1]) / 2.0 - p)


def finger_gap(model, data):
    """Distance between the two pad FACES, measured along the closing axis.

    The first version differenced world y coordinates and returned -15.78 mm, because the
    closing axis is the finger's own local y, which at the home pose maps to world x. So the
    axis is taken from the joint, and each pad's extent along it is projected out of the box.
    """
    left = _pad_geom(model, 'left_finger')
    right = _pad_geom(model, 'right_finger')
    lj = model.joint('finger_joint1')
    axis_local = np.array(model.jnt_axis[lj.id], dtype=float)
    b_left = model.body('left_finger').id
    d_world = np.array(data.xmat[b_left], dtype=float).reshape(3, 3) @ axis_local
    n = np.linalg.norm(d_world)
    if n < 1e-9:
        raise RuntimeError('the finger closing axis collapsed')
    d_world /= n
    separation = abs(float(np.dot(data.geom_xpos[left] - data.geom_xpos[right], d_world)))
    half_l = float(np.dot(np.asarray(model.geom_size[left]), np.abs(axis_local)))
    half_r = float(np.dot(np.asarray(model.geom_size[right]), np.abs(axis_local)))
    return separation - half_l - half_r


def home_hand_rot(model, data):
    """The hand body's world orientation in the `home` keyframe.

    Captured rather than typed: the IK holds this orientation, and "fingers pointing straight
    down" is a property of the model, not an assumption about it. This is only meaningful
    because the generated bodies are declared AFTER the arm.
    """
    import mujoco
    mujoco.mj_resetDataKeyframe(model, data, 0)
    mujoco.mj_forward(model, data)
    return np.array(data.xmat[model.body('hand').id], dtype=float).reshape(3, 3)


def grasp_point(model, data):
    """World position of the grasp frame, from the current state."""
    hand = model.body('hand').id
    R = np.array(data.xmat[hand], dtype=float).reshape(3, 3)
    return np.array(data.xpos[hand], dtype=float) + R @ grasp_local_offset(model, data)


def contacts_between(model, data, bodies_a, body_b):
    """Active contacts with EXACTLY ONE side in `bodies_a` and the other on `body_b`.

    Exactly-one-side is the predicate that matters. The equivalent helper in the C branch was
    first written as "one side in the set AND neither side in the set", which is
    unsatisfiable: it returned nothing every time, and "nothing is touching" read as a finding.
    """
    out = []
    for c in range(data.ncon):
        con = data.contact[c]
        b1 = int(model.geom_bodyid[con.geom1])
        b2 = int(model.geom_bodyid[con.geom2])
        if b1 in bodies_a and b2 == body_b:
            out.append((int(con.geom1), int(con.geom2)))
        elif b2 in bodies_a and b1 == body_b:
            out.append((int(con.geom2), int(con.geom1)))
    return out


# =======================================================================================
# inverse kinematics
# =======================================================================================

def ik(model, data, target_pos, target_rot=None, q_seed=None,
       iters=400, pos_tol=1.0e-4, rot_tol=1.0e-3, rot_weight=1.2):
    """Damped-least-squares IK for the 7 arm joints.

    Solves for the grasp frame's world position AND the hand's world orientation together.
    Position-only IK was not an option: the finger-length axis must stay vertical or the pads
    do not present flat faces to the part, and assuming that rather than solving it is exactly
    what the C branch kept getting wrong.

    Raises if it does not converge, rather than returning a bad answer that a later stage
    would silently treat as a plan.
    """
    import mujoco
    bundle = ids(model)
    hand = bundle['hand']
    arm_qadr = bundle['arm_qadr']
    arm_dof = bundle['arm_dof']
    if target_rot is None:
        target_rot = home_hand_rot(model, data)
    if q_seed is not None:
        data.qpos[arm_qadr] = q_seed
    mujoco.mj_forward(model, data)

    lo = np.array([model.jnt_range[j][0] for j in bundle['arm_joints']])
    hi = np.array([model.jnt_range[j][1] for j in bundle['arm_joints']])
    grip = grasp_local_offset(model, data)
    jacp = np.zeros((3, model.nv))
    jacr = np.zeros((3, model.nv))
    best = None

    for it in range(iters):
        mujoco.mj_jacBody(model, data, jacp, jacr, hand)
        R = np.array(data.xmat[hand], dtype=float).reshape(3, 3)
        r = R @ grip
        J = np.vstack([jacp - _skew(r) @ jacr, rot_weight * jacr])[:, arm_dof]
        pos_err = np.asarray(target_pos, dtype=float) - (np.array(data.xpos[hand]) + r)
        rot_err = rot_weight * _rot_vector(np.asarray(target_rot) @ R.T)
        err = np.concatenate([pos_err, rot_err])
        pe, re = float(np.linalg.norm(pos_err)), float(np.linalg.norm(rot_err)) / rot_weight
        if best is None or max(pe, re) < best[0]:
            best = (max(pe, re), np.array(data.qpos[arm_qadr]), pe, re)
        if pe < pos_tol and re < rot_tol:
            return np.array(data.qpos[arm_qadr]), pe, re
        lam = 1e-2 + 1e-2 * (1.0 - it / iters)
        dq = J.T @ np.linalg.solve(J @ J.T + lam * lam * np.eye(6), err)
        step = np.clip(dq, -0.15, 0.15)
        data.qpos[arm_qadr] = np.clip(np.array(data.qpos[arm_qadr]) + step, lo, hi)
        mujoco.mj_forward(model, data)

    raise RuntimeError(
        f'IK did not converge to {np.round(target_pos, 4)}: best pos {best[2]:.5f} m, '
        f'rot {best[3]:.5f} rad after {iters} iterations')


def planned_poses():
    """The grasp-frame waypoints of the sequence, as a name -> world position map."""
    z = payload_rest_z()
    px, py = PICK_XY
    qx, qy = PLACE_XY
    return {
        'pregrasp': (px, py, z + PREGRASP_CLEARANCE),
        'grasp': (px, py, z + GRASP_INSET),
        'lift': (px, py, z + GRASP_INSET + LIFT_HEIGHT),
        'above_place': (qx, qy, z + PREGRASP_CLEARANCE),
        'place': (qx, qy, z + GRASP_INSET),
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--check', action='store_true',
                        help='verify the generated world against its source; write nothing')
    args = parser.parse_args()
    return build(check=args.check)


if __name__ == '__main__':
    sys.exit(main())
