"""Observe the existing cancel/stop, without bypassing its freeze or changing physics.

The entire trace is independent evaluation only. Primitive corner/enclosing-box
speeds provide a tighter conservative bound; finite differences are diagnostics,
not a substitute for the instantaneous stopped-speed contract.
"""
import argparse
import itertools
import json
import sys
from pathlib import Path

import mujoco
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT / 'experiments'), str(ROOT / 'src')]
import probe_transfer_recovery as recovery
import w4_plant as wp
from workcell.runtime_permit import RuntimePermit


def points(model, body):
    vertices = []
    for geom in range(model.ngeom):
        if model.geom_bodyid[geom] != body or not (
                model.geom_contype[geom] or model.geom_conaffinity[geom]):
            continue
        size, kind = model.geom_size[geom], model.geom_type[geom]
        if kind == mujoco.mjtGeom.mjGEOM_BOX:
            half = size.copy()
        elif kind == mujoco.mjtGeom.mjGEOM_SPHERE:
            half = np.full(3, size[0])
        elif kind == mujoco.mjtGeom.mjGEOM_CAPSULE:
            half = np.array([size[0], size[0], size[0] + size[1]])
        elif kind == mujoco.mjtGeom.mjGEOM_CYLINDER:
            half = np.array([size[0], size[0], size[1]])
        else:
            raise ValueError('unsupported physical primitive')
        rotation = np.empty(9)
        mujoco.mju_quat2Mat(rotation, model.geom_quat[geom])
        rotation = rotation.reshape(3, 3)
        for signs in itertools.product((-1, 1), repeat=3):
            vertices.append(model.geom_pos[geom] + rotation @ (half * signs))
    return np.asarray(vertices)


def measured_stop_window(trace, result, end):
    # Initial settling is not a measured stop reserve. A refused transfer must
    # not turn its earlier initialization trace into purported stop evidence.
    if not isinstance(result.get('brake'), dict):
        return []
    return [x for x in trace if x['time_s'] > end - wp.STOPPED_HOLD_S + 1e-9]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run-id', required=True)
    parser.add_argument('--initial-offset-x', type=float, default=0.0)
    parser.add_argument('--no-drive', action='store_true')
    parser.add_argument('--loaded', action='store_true')
    parser.add_argument('--initialize-loading', action='store_true',
                        help='independent episode starts at declared loading fixture')
    parser.add_argument('--settle-initial-s', type=float, default=0.0)
    parser.add_argument('--diagnostic-timestep', type=float)
    parser.add_argument('--world', default='world_w2_logistic.xml',
                        choices=['world_w2_logistic.xml', 'world_w2_support_v1.xml',
                                 'world_w2_support_v2.xml', 'world_p5_candidate_v3.xml'])
    args = parser.parse_args()
    out = ROOT / 'reports' / args.run_id
    out.mkdir(parents=True, exist_ok=False)
    wp.WORLD = ROOT / 'assets' / args.world
    loaded_probe = None
    if args.loaded:
        import probe_loaded_retention as loaded_probe
        plant, inventory = loaded_probe.build_loaded()
    else:
        plant, _ = recovery.build()
        inventory = []
    model, data = plant.model, plant.data
    original_timestep = float(model.opt.timestep)
    if args.diagnostic_timestep is not None:
        if not np.isfinite(args.diagnostic_timestep) or args.diagnostic_timestep <= 0:
            raise ValueError('diagnostic timestep must be finite and positive')
        # Separate diagnostic model only; no frozen XML or acceptance threshold changes.
        model.opt.timestep = args.diagnostic_timestep
    body = model.body('c_payload').id
    joint = int(model.body_jntadr[body])
    qadr, dof = int(model.jnt_qposadr[joint]), int(model.jnt_dofadr[joint])
    if not np.isfinite(args.initial_offset_x):
        raise ValueError('initial offset must be finite')
    # Declared independent initial condition BEFORE this episode's first physics step.
    # Never moves cargo to clear a running safety hold or to fake successful recovery.
    initial_offset = args.initial_offset_x
    if args.initialize_loading:
        initial_offset += float(data.geom_xpos[model.geom('c_fixed_roller_1_2').id][0]
                                - data.qpos[qadr])
    data.qpos[qadr] += initial_offset
    if args.loaded:
        for name, _, _ in loaded_probe.PARTS:
            part_joint = int(model.body_jntadr[model.body(name).id])
            data.qpos[int(model.jnt_qposadr[part_joint])] += initial_offset
    mujoco.mj_forward(model, data)
    initial_xyz = data.qpos[qadr:qadr+3].copy()
    local_points = points(model, body)
    trace, previous = [], [None]
    original_tick = plant._tick

    def tick(*a, **kw):
        original_tick(*a, **kw)
        quat_rotation = np.empty(9)
        mujoco.mju_quat2Mat(quat_rotation, data.qpos[qadr + 3:qadr + 7])
        rotation = quat_rotation.reshape(3, 3)
        offsets = local_points @ rotation.T
        xyz = offsets + data.qpos[qadr:qadr + 3]
        linear = data.qvel[dof:dof + 3].copy()
        angular_local = data.qvel[dof + 3:dof + 6].copy()
        angular_world = rotation @ angular_local
        velocities = linear + np.cross(angular_world, offsets)
        bound = float(np.max(np.linalg.norm(velocities, axis=1)))
        finite_difference = None if previous[0] is None else float(np.max(
            np.linalg.norm(xyz - previous[0], axis=1)) / model.opt.timestep)
        previous[0] = xyz.copy()
        contacts = []
        for contact in data.contact[:data.ncon]:
            if body in (model.geom_bodyid[contact.geom1], model.geom_bodyid[contact.geom2]):
                contacts.append(dict(geoms=[model.geom(g).name
                                           for g in (contact.geom1, contact.geom2)],
                                     depth_m=float(max(0, -contact.dist))))
        trace.append(dict(time_s=float(data.time), linear_world_mps=linear.tolist(),
            angular_local_radps=angular_local.tolist(), angular_world_radps=angular_world.tolist(),
            enclosing_primitive_speed_bound_mps=bound,
            finite_difference_point_speed_mps=finite_difference,
            old_spherical_speed_bound_mps=float(np.linalg.norm(linear)
                + np.linalg.norm(angular_local) * plant.tray_envelope_radius_m),
            tray_xyz=data.qpos[qadr:qadr + 3].tolist(), contacts=contacts))

    plant._tick = tick
    if not np.isfinite(args.settle_initial_s) or not 0 <= args.settle_initial_s <= 2:
        raise ValueError('initial settling duration must be in [0, 2] seconds')
    for _ in range(int(args.settle_initial_s / model.opt.timestep)):
        plant._tick(lambda side: plant._wheel_rate_torque(side, 0.0))
    transfer_start = float(data.time)
    sequence = [0]

    def observe():
        import time
        sequence[0] += 1
        return dict(owner='stop_diagnostic', epoch=1, generation=1,
                    sequence=sequence[0], observed_wall_s=time.monotonic(),
                    cancel_requested=args.no_drive or data.time >= transfer_start + .3,
                    zone_clear=True, stop_chain_healthy=True)

    plant.motion_guard = RuntimePermit(observe, owner='stop_diagnostic', epoch=1,
                                        generation=1, max_age_s=.5)
    result = plant.transfer(direction='onto_deck', timeout_s=5.)
    end = float(data.time)
    held = measured_stop_window(trace, result, end)
    if not held:
        report = dict(scope='STOP_MEASUREMENT_DIAGNOSTIC_ONLY', existing_stop=result,
                      diagnostic_result='NOT_RUN', reason='NO_STOP_WINDOW_RECORDED',
                      world=args.world, initialized_inventory=inventory,
                      full_order='NOT_RUN', v1_complete=False)
        (out / 'report.json').write_text(json.dumps(report, indent=2)+'\n')
        print(json.dumps(report, indent=2))
        return 1
    worst = max(held, key=lambda x: x['enclosing_primitive_speed_bound_mps'])
    report = dict(scope='STOP_MEASUREMENT_DIAGNOSTIC_ONLY', existing_stop=result,
        initial_condition=dict(offset_x_m=initial_offset, xyz=initial_xyz.tolist(),
                               declared_loading_initialization=args.initialize_loading,
                               pre_transfer_settle_s=transfer_start,
                               no_drive=args.no_drive,
                               extra_initialization_writes=1+len(inventory)),
        diagnostic_model=dict(original_timestep_s=original_timestep,
                              timestep_s=float(model.opt.timestep),
                              world=args.world,
                              frozen_asset_modified=False),
        initialized_inventory=inventory,
        final_inventory=(loaded_probe.inventory_truth(plant) if args.loaded else []),
        held_window=dict(start_s=end-wp.STOPPED_HOLD_S, end_s=end, samples=len(held),
            old_bound_peak_mps=max(x['old_spherical_speed_bound_mps'] for x in held),
            primitive_bound_peak_mps=worst['enclosing_primitive_speed_bound_mps'],
            finite_difference_peak_mps=max(x['finite_difference_point_speed_mps'] for x in held),
            worst_sample=worst), trace=trace, no_runtime_teleport=plant.qpos_writes.get('run', 0)==0,
        freeze_honored=bool(getattr(plant, 'safety_frozen', False)),
        diagnostic_result='RECORDED', full_order='NOT_RUN', v1_complete=False)
    (out / 'report.json').write_text(json.dumps(report, indent=2)+'\n')
    print(json.dumps({k:v for k,v in report.items() if k != 'trace'}, indent=2))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
