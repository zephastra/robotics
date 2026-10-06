"""Read-only shared-world geometry audit; ZERO physics steps, NOT Nav2 acceptance.

Only initialization geometry is inspected. Cargo truth is never a navigation
input. Spinning cylinders are static only when their occupied volume is invariant
under every ancestor hinge. Unsupported geometry is refused, never guessed.
"""
import argparse
import hashlib
import json
from pathlib import Path
import numpy as np
import mujoco


def bounds(position, rotation, size, kind):
    p, r, s = np.asarray(position, float), np.asarray(rotation, float), np.asarray(size, float)
    if (p.shape != (3,) or r.shape != (3, 3) or s.shape != (3,)
            or not all(np.all(np.isfinite(v)) for v in (p, r, s))
            or np.any(s < 0) or not np.allclose(r.T @ r, np.eye(3), atol=1e-8)
            or not np.isclose(np.linalg.det(r), 1., atol=1e-8)):
        raise ValueError('invalid primitive geometry')
    if kind == mujoco.mjtGeom.mjGEOM_BOX:
        half = np.abs(r) @ s
    elif kind == mujoco.mjtGeom.mjGEOM_SPHERE:
        half = np.full(3, s[0])
    elif kind == mujoco.mjtGeom.mjGEOM_CAPSULE:
        half = np.abs(r[:, 2]) * s[1] + s[0]
    elif kind == mujoco.mjtGeom.mjGEOM_CYLINDER:
        axis = r[:, 2]
        half = np.abs(axis) * s[1] + s[0] * np.sqrt(np.maximum(0., 1. - axis ** 2))
    else:
        raise ValueError('unsupported physical primitive type: %s' % kind)
    return p - half, p + half


def descendants(model, root):
    result = {int(root)}
    for body in range(1, model.nbody):
        if int(model.body_parentid[body]) in result:
            result.add(body)
    return result


def ancestor_joints(model, body):
    joints = []
    while body:
        first, number = int(model.body_jntadr[body]), int(model.body_jntnum[body])
        joints.extend(range(first, first + number))
        body = int(model.body_parentid[body])
    return joints


def invariant_under_joint(model, data, geom, joint):
    if model.jnt_type[joint] != mujoco.mjtJoint.mjJNT_HINGE:
        return False
    axis = data.xaxis[joint]
    offset = data.geom_xpos[geom] - data.xanchor[joint]
    # Center must lie on the rotation axis. A sphere is rotationally invariant;
    # a cylinder/capsule additionally needs its symmetry axis to match the hinge.
    if np.linalg.norm(np.cross(offset, axis)) > 1e-9:
        return False
    kind = model.geom_type[geom]
    if kind == mujoco.mjtGeom.mjGEOM_SPHERE:
        return True
    if kind in (mujoco.mjtGeom.mjGEOM_CYLINDER, mujoco.mjtGeom.mjGEOM_CAPSULE):
        symmetry = data.geom_xmat[geom].reshape(3, 3)[:, 2]
        return abs(float(np.dot(symmetry, axis))) >= 1. - 1e-9
    return False


def classify(model, data, robot_roots):
    robot_bodies = set()
    for name in robot_roots:
        root = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, name)
        if root < 0:
            raise ValueError('missing declared robot root: ' + name)
        robot_bodies.update(descendants(model, root))
    groups = {key: [] for key in ('static', 'moving', 'robot', 'visual_only', 'floor')}
    for geom in range(model.ngeom):
        name = model.geom(geom).name or '@geom%d' % geom
        if model.geom_contype[geom] == model.geom_conaffinity[geom] == 0:
            key = 'visual_only'
        elif int(model.geom_bodyid[geom]) in robot_bodies:
            key = 'robot'
        elif model.geom_type[geom] == mujoco.mjtGeom.mjGEOM_PLANE:
            key = 'floor' if not ancestor_joints(model, int(model.geom_bodyid[geom])) else 'moving'
        else:
            joints = ancestor_joints(model, int(model.geom_bodyid[geom]))
            key = 'static' if all(invariant_under_joint(model, data, geom, j) for j in joints) else 'moving'
        groups[key].append(name)
    return groups


def robot_bounds(model, data, root_name):
    root = model.body(root_name).id
    bodies = descendants(model, root)
    origin, rotation = data.xpos[root], data.xmat[root].reshape(3, 3)
    low, high = np.full(3, np.inf), np.full(3, -np.inf)
    for geom in range(model.ngeom):
        if (int(model.geom_bodyid[geom]) not in bodies
                or model.geom_contype[geom] == model.geom_conaffinity[geom] == 0):
            continue
        lo, hi = bounds(rotation.T @ (data.geom_xpos[geom] - origin),
                        rotation.T @ data.geom_xmat[geom].reshape(3, 3),
                        model.geom_size[geom], model.geom_type[geom])
        low, high = np.minimum(low, lo), np.maximum(high, hi)
    if not np.all(np.isfinite([low, high])):
        raise ValueError('robot has no finite collision envelope')
    return low, high


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run-id', required=True)
    args = parser.parse_args()
    if Path(args.run_id).name != args.run_id or args.run_id in ('.', '..'):
        parser.error('bare run ID required')
    root = Path(__file__).resolve().parents[1]
    world = root / 'assets/world_p5_candidate_v5.xml'
    out = root / 'reports' / args.run_id
    out.mkdir(parents=True, exist_ok=False)
    model = mujoco.MjModel.from_xml_path(str(world)); data = mujoco.MjData(model)
    mujoco.mj_resetDataKeyframe(model, data, 0); mujoco.mj_forward(model, data)
    groups = classify(model, data, ['n_base_link', 'n2_base_link', 'h_LINK_BASE', 'a_link0'])
    low, high = robot_bounds(model, data, 'n_base_link')
    static = []; unsupported = []
    for geom in range(model.ngeom):
        name = model.geom(geom).name or '@geom%d' % geom
        if name not in groups['static']: continue
        try:
            lo, hi = bounds(data.geom_xpos[geom], data.geom_xmat[geom].reshape(3, 3),
                            model.geom_size[geom], model.geom_type[geom])
            static.append(dict(name=name, body=model.body(model.geom_bodyid[geom]).name,
                               lo=lo.tolist(), hi=hi.tolist()))
        except ValueError as exc:
            unsupported.append(dict(name=name, reason=str(exc)))
    joints = []
    for joint in range(model.njnt):
        name = model.joint(joint).name or ''
        if not name.startswith('c_'): continue
        joints.append(dict(name=name, limited=bool(model.jnt_limited[joint]),
                           range=model.jnt_range[joint].tolist(),
                           initial=float(data.qpos[model.jnt_qposadr[joint]])))
    report = dict(scope='INITIAL_GEOMETRY_AUDIT_ZERO_PHYSICS_NOT_NAVIGATION',
        world_sha256=hashlib.sha256(world.read_bytes()).hexdigest(),
        source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        physics_steps=0, groups=groups, static_primitives=static, unsupported_static=unsupported,
        initialized_robot_bounds_base_frame=dict(lo=low.tolist(), hi=high.tolist()),
        mechanism_joints=joints, nav2='NOT_RUN', full_order='NOT_RUN', v1_complete=False,
        runtime_footprint='NOT_COMMISSIONED: requires declared slide/yaw/lateral posture and cargo catalog containment',
        map='NOT_GENERATED: static inventory only, not SLAM')
    (out / 'report.json').write_text(json.dumps(report, indent=2) + '\n')
    print(json.dumps(dict(physics_steps=0, group_counts={k: len(v) for k, v in groups.items()},
        robot_bounds=report['initialized_robot_bounds_base_frame'], unsupported_static=unsupported), indent=2))
    return 0 if not unsupported else 1


if __name__ == '__main__':
    raise SystemExit(main())
