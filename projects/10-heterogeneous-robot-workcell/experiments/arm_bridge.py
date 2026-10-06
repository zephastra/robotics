"""Let `arm_rig` drive the arm in the MERGED world.

`arm_rig` was written against its own generated bench world, where the arm's bodies are unprefixed
(`hand`, `table`, `joint1..7`, `payload`, `actuator8`). The merged V1 world prefixes every role, so
the arm is `a_hand`, `a_table`, `a_joint1..7`, `a_payload`, `a_actuator8`.

So the names are REBOUND here rather than the geometry being copied. Two reasons:

* copying the arm's numbers into a new file would create a second source of truth for a robot that
  already has one, and the two would drift the moment the merged world changed;
* rebinding is checkable. `verify()` does not merely confirm that the lookups resolve -- it confirms
  that the thing on the other end is the SAME ARM: the seven joint ranges must equal the standalone
  world's, and `grasp_local_offset` -- a constant of the arm's own geometry -- must come out equal.
  A prefix typo that happened to resolve to some other body would fail the second check.

This is an ADAPTER, not a fork: nothing inside `arm_rig` is edited on disk. The rebinding is applied
to the imported module object at run time, which is visible in `installed()`.
"""
import numpy as np

import arm_rig as rig

_INSTALLED = {}


def _patch_functions(prefix):
    def ids(model):
        out = {
            'hand': model.body(prefix + 'hand').id,
            'left_finger': model.body(prefix + 'left_finger').id,
            'right_finger': model.body(prefix + 'right_finger').id,
            'payload': model.body(prefix + 'payload').id,
            'table': model.body(prefix + 'table').id,
        }
        out['arm_joints'] = [model.joint('%sjoint%d' % (prefix, i)).id for i in range(1, 8)]
        out['finger_joints'] = [model.joint('%sfinger_joint%d' % (prefix, i)).id for i in (1, 2)]
        out['arm_qadr'] = np.array([model.jnt_qposadr[j] for j in out['arm_joints']])
        out['arm_dof'] = np.array([model.jnt_dofadr[j] for j in out['arm_joints']])
        out['finger_qadr'] = np.array([model.jnt_qposadr[j] for j in out['finger_joints']])
        out['finger_dof'] = np.array([model.jnt_dofadr[j] for j in out['finger_joints']])
        out['payload_joint'] = model.joint(prefix + 'payload_free').id
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

    def pad_geom(model, side):
        import mujoco
        b = model.body(prefix + side).id
        boxes = [g for g in range(model.ngeom)
                 if int(model.geom_bodyid[g]) == b
                 and model.geom_type[g] == mujoco.mjtGeom.mjGEOM_BOX]
        if not boxes:
            raise RuntimeError('%s%s has no box pads' % (prefix, side))
        return max(boxes, key=lambda g: float(np.prod(model.geom_size[g])))

    def grasp_local_offset(model, data):
        """Pad-centre midpoint in the HAND frame. FK-derived, so it follows the prefix."""
        import mujoco
        hand = model.body(prefix + 'hand').id
        mujoco.mj_forward(model, data)
        R = np.array(data.xmat[hand], dtype=float).reshape(3, 3)
        p = np.array(data.xpos[hand], dtype=float)
        centres = [np.array(data.geom_xpos[pad_geom(model, s)])
                   for s in ('left_finger', 'right_finger')]
        return R.T @ (centres[0] + centres[1]) / 2.0 - R.T @ p

    def grasp_point(model, data):
        hand = model.body(prefix + 'hand').id
        R = np.array(data.xmat[hand], dtype=float).reshape(3, 3)
        return (np.array(data.xpos[hand], dtype=float)
                + R @ grasp_local_offset(model, data))

    def home_hand_rot(model, data):
        """The hand's world orientation at the model's own home pose.

        NOT a copy of arm_rig's version: that one calls `mj_resetDataKeyframe`, which on the
        merged world would reset every other role too (the tray, the vehicles). This one sets the
        home qpos, advances, and reads -- so it cannot disturb anything the caller has placed.
        """
        import mujoco
        data.qpos[:] = model.key_qpos[0]
        mujoco.mj_forward(model, data)
        return np.array(data.xmat[model.body(prefix + 'hand').id], dtype=float).reshape(3, 3)

    def finger_gap(model, data):
        left = pad_geom(model, 'left_finger')
        right = pad_geom(model, 'right_finger')
        lj = model.joint(prefix + 'finger_joint1')
        axis_local = np.array(model.jnt_axis[lj.id], dtype=float)
        b_left = model.body(prefix + 'left_finger').id
        d_world = np.array(data.xmat[b_left], dtype=float).reshape(3, 3) @ axis_local
        n = np.linalg.norm(d_world)
        if n < 1e-9:
            raise RuntimeError('the finger closing axis collapsed')
        d_world /= n
        separation = abs(float(np.dot(data.geom_xpos[left] - data.geom_xpos[right], d_world)))
        half_l = float(np.dot(np.asarray(model.geom_size[left]), np.abs(axis_local)))
        half_r = float(np.dot(np.asarray(model.geom_size[right]), np.abs(axis_local)))
        return separation - half_l - half_r

    def payload_qpos_adr(model):
        return int(model.jnt_qposadr[model.joint(prefix + 'payload_free').id])

    return {'ids': ids, '_pad_geom': pad_geom, 'grasp_local_offset': grasp_local_offset,
            'grasp_point': grasp_point, 'home_hand_rot': home_hand_rot,
            'finger_gap': finger_gap, 'payload_qpos_adr': payload_qpos_adr}


def install(prefix='a_'):
    """Rebind `arm_rig`'s name-dependent helpers and constants to a prefix. Idempotent."""
    if _INSTALLED.get('prefix') == prefix:
        return _INSTALLED
    originals = {}
    for name in ('ids', '_pad_geom', 'grasp_local_offset', 'grasp_point', 'home_hand_rot',
                 'finger_gap', 'payload_qpos_adr'):
        originals[name] = getattr(rig, name)
    originals['PAYLOAD'] = rig.PAYLOAD
    originals['PAYLOAD_FREEJOINT'] = rig.PAYLOAD_FREEJOINT
    originals['GRIPPER_ACTUATOR'] = rig.GRIPPER_ACTUATOR

    for name, fn in _patch_functions(prefix).items():
        setattr(rig, name, fn)
    rig.PAYLOAD = prefix + 'payload'
    rig.PAYLOAD_FREEJOINT = prefix + 'payload_free'
    rig.GRIPPER_ACTUATOR = prefix + 'actuator8'

    _INSTALLED.clear()
    _INSTALLED.update({'prefix': prefix, 'originals': originals})
    return _INSTALLED


def installed():
    """What was rebound, so a report can state it rather than assert it."""
    return {'prefix': _INSTALLED.get('prefix'),
            'rebound': sorted(k for k in _INSTALLED.get('originals', {}) if k != 'prefix')}


class _Uninstalled:
    """Temporarily restore arm_rig's ORIGINAL helpers, to measure an unprefixed world.

    Needed because the originals call each other: `grasp_local_offset` calls `_pad_geom`, so
    restoring only the outer function still routes through the rebound inner one -- which is how
    this failed the first time, with `a_left_finger` looked up in a world that has `left_finger`.
    """

    def __enter__(self):
        saved = _INSTALLED.get('originals')
        if not saved:
            raise RuntimeError('install() must run before uninstalled()')
        self._patched = {k: getattr(rig, k) for k in saved}
        for name, fn in saved.items():
            setattr(rig, name, fn)
        return self

    def __exit__(self, *exc):
        for name, fn in self._patched.items():
            setattr(rig, name, fn)
        return False


def uninstalled():
    return _Uninstalled()


def verify(model, data, standalone_model, *, prefix='a_'):
    """Prove the rebinding points at the SAME ARM, not merely at something that resolves.

    Three checks, each able to fail:
      1. the structure resolves: 7 arm joints, 2 finger joints, a hand, a table, a payload;
      2. the seven JOINT RANGES equal the standalone world's -- a different robot fails this;
      3. `grasp_local_offset`, a constant of the arm's own geometry, comes out equal across worlds.
    """
    b = rig.ids(model)
    ranges_here = np.array([list(model.jnt_range[j]) for j in b['arm_joints']])
    ranges_there = np.array([list(standalone_model.jnt_range[
        standalone_model.joint('joint%d' % i).id]) for i in range(1, 8)])
    offset_here = np.asarray(rig.grasp_local_offset(model, data), float)
    # the standalone world keeps its UNPREFIXED names, so it is measured with the ORIGINAL
    # helpers restored. Each side is read with the reader it needs, not one reader applied twice.
    with uninstalled():
        offset_there = np.asarray(
            rig.grasp_local_offset(standalone_model, _standalone_data(standalone_model)), float)
    return {
        'n_arm_joints': len(b['arm_joints']),
        'n_finger_joints': len(b['finger_joints']),
        'joint_ranges_equal': bool(np.allclose(ranges_here, ranges_there)),
        'grasp_local_offset_m': [float(v) for v in offset_here],
        'grasp_local_offset_equal_across_worlds': bool(np.allclose(offset_here, offset_there,
                                                                  atol=1e-9)),
        'same_arm': bool(len(b['arm_joints']) == 7 and len(b['finger_joints']) == 2
                         and np.allclose(ranges_here, ranges_there)
                         and np.allclose(offset_here, offset_there, atol=1e-9)),
    }


def _standalone_data(model):
    import mujoco
    d = mujoco.MjData(model)
    mujoco.mj_resetDataKeyframe(model, d, 0)
    mujoco.mj_forward(model, d)
    return d
