#!/usr/bin/env python3
"""Physical 2-red/1-blue retention diagnostic; NOT arm loading or an order.

Frozen W2 hardware is unchanged. Three free, colliding 30 g pieces are INITIAL
conditions. No weld, runtime qpos edits, visual oracle or order-count feedback.
Only the deck-band command differs between nominal and injected-fault arms.
"""
import argparse
import hashlib
import json
import sys
from pathlib import Path

import mujoco
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'experiments'))
sys.path.insert(0, str(ROOT / 'src'))
import w4_plant as wp
import probe_p4_belt as belt
from workcell.tray import derive_cells, cell_centre_world, FOOTPRINT_TOL_M

PARTS = (('loaded_red_0', 'red', 0), ('loaded_red_1', 'red', 1),
         ('loaded_blue_0', 'blue', 2))


def _json_scalar(value):
    if isinstance(value, np.generic):
        return value.item()
    raise TypeError('unsupported evidence value: %r' % type(value))


def build_loaded():
    """Adding explicit initial inventory does not modify the frozen hardware."""
    wp.bp3.install()
    baseline = mujoco.MjModel.from_xml_path(str(wp.WORLD))
    initial = mujoco.MjData(baseline)
    initial.qpos[:] = wp.mw.merged_home(baseline)[0]
    mujoco.mj_forward(baseline, initial)
    cells = derive_cells(baseline, initial)
    source_geom = baseline.geom('a_payload_box').id
    size = np.array(baseline.geom_size[source_geom], dtype=float)
    mass = float(baseline.body_mass[baseline.body('a_payload').id])
    spec = mujoco.MjSpec.from_file(str(wp.WORLD))
    # Role home is assembled by name, so added DOFs never receive zero-filled keys.
    for key in list(spec.keys):
        spec.delete(key)
    declarations = []
    for name, kind, cell in PARTS:
        pos = cell_centre_world(cells, cell, z=cells['deck']['top_z'] + size[2])
        body = spec.worldbody.add_body(name=name, pos=pos)
        body.add_freejoint(name=name + '_free')
        shape = (mujoco.mjtGeom.mjGEOM_BOX if kind == 'red'
                 else mujoco.mjtGeom.mjGEOM_CYLINDER)
        dimensions = size if kind == 'red' else [min(size[:2]), size[2], 0]
        body.add_geom(name=name + '_geom', type=shape, size=dimensions, mass=mass,
                      friction=[.6, .005, .001], condim=6, contype=1, conaffinity=1,
                      rgba=[1, .02, .02, 1] if kind == 'red' else [.02, .02, 1, 1])
        declarations.append({'name': name, 'class': kind, 'cell': cell,
                             'mass_kg': mass, 'size_m': list(map(float, dimensions)),
                             'initial_position_m': pos})
    model = spec.compile()
    data = mujoco.MjData(model)
    data.qpos[:] = wp.mw.merged_home(model)[0]  # the one episode initialisation
    mujoco.mj_forward(model, data)
    plant = wp.LogisticsPlant(model=model, data=data)
    plant._retention_cells = cells  # immutable initial local cuts, not world cuts after yaw
    return plant, declarations


def inventory_truth(plant):
    """Independent physical evaluator only; never passed into motor control."""
    m, d = plant.model, plant.data
    tray = m.body('c_payload').id
    rotation = d.xmat[tray].reshape(3, 3)
    cells = plant._retention_cells
    # Cell cuts are body-local offsets; derive_cells' world centres are unsuitable
    # after yaw, so use the frozen tray floor's local centre and sizes instead.
    floor = m.geom('c_tray_floor').id
    floor_pos, floor_size = m.geom_pos[floor], m.geom_size[floor]
    readings = []
    for name, kind, cell in PARTS:
        bid = m.body(name).id
        local = rotation.T @ (d.xpos[bid] - d.xpos[tray])
        bounds = cells['cells'][cell]
        gid = m.geom(name+'_geom').id
        half_z = m.geom_size[gid][2 if kind == 'red' else 1]
        inside = (abs(local[0] - floor_pos[0]) <= floor_size[0]
                  and bounds['lo'] <= local[1] - floor_pos[1] <= bounds['hi']
                  and abs(local[2] - (floor_pos[2] + floor_size[2] + half_z)) <= FOOTPRINT_TOL_M
                  and np.all(np.isfinite(local)))
        touches_tray = any(
            {int(m.geom_bodyid[int(c.geom[0])]), int(m.geom_bodyid[int(c.geom[1])])} == {bid, tray}
            for c in d.contact[:d.ncon])
        readings.append({'name': name, 'class': kind, 'expected_cell': cell,
                         'position_in_tray_m': list(map(float, local)),
                         'in_expected_cell': bool(inside),
                         'touches_tray': bool(touches_tray),
                         'mass_kg': float(m.body_mass[bid]),
                         'is_free_body': m.jnt_type[m.body_jntadr[bid]] == mujoco.mjtJoint.mjJNT_FREE})
    return readings


def run_arm(driven):
    plant, declarations = build_loaded()
    load = plant.transfer(direction='onto_deck', timeout_s=20.)
    plant.stop()
    before = inventory_truth(plant)
    reference = np.array(belt.tray_in_deck_frame(plant.model, plant.data,
                         plant.model.body('c_payload').id, plant.model.body('c_deck').id)[:2])
    brake_peak = [0.]
    original_tick = plant._tick

    def monitored_tick(*args, **kwargs):
        result = original_tick(*args, **kwargs)
        rel = np.array(belt.tray_in_deck_frame(plant.model, plant.data,
                       plant.model.body('c_payload').id, plant.model.body('c_deck').id)[:2])
        brake_peak[0] = max(brake_peak[0], float(np.linalg.norm(rel-reference)))
        return result

    plant._tick = monitored_tick
    transport = belt.drive_leg(plant, leg_m=belt.RETENTION_LEG_M,
                              deck_band_driven=driven, settle_first=False, deploy_blade=True)
    transport['peak_including_brake_m'] = max(transport['peak_rel_m'], brake_peak[0])
    return {'declarations': declarations, 'load': load, 'before': before,
            'after': inventory_truth(plant), 'transport': transport,
            'tray': plant.tray_state(), 'qpos_writes': plant.qpos_writes,
            'equality_constraints': int(plant.model.neq)}


def judge(report):
    normal, fault = report['nominal'], report['fault_deck_driven']

    def all_parts(readings):
        return (len(readings) == len(PARTS)
                and {(x['name'], x['class'], x['expected_cell']) for x in readings} == set(PARTS)
                and all(x['in_expected_cell'] and x['touches_tray'] for x in readings))

    checks = {
        'nominal_loaded': normal['load']['state'] == 'RECEIVED',
        'three_parts_initially_on_tray': all_parts(normal['before']),
        'three_parts_retained': all_parts(normal['after']),
        'parts_are_free': all(x['is_free_body'] for x in normal['after']),
        'normal_leg_reached': normal['transport']['leg_m'] >= .30,
        'normal_tray_retained': normal['tray']['supported_by'] == ['deck'],
        'normal_slip_within_budget': normal['transport']['peak_including_brake_m'] <= wp.DRIFT_LIMIT_M
            and normal['transport']['net_slip_m'] <= wp.DRIFT_LIMIT_M,
        'fault_actually_loaded': fault['load']['state'] == 'RECEIVED'
            and all_parts(fault['before']),
        'fault_leg_reached': fault['transport']['leg_m'] >= .30,
        'fault_detectably_slides': fault['transport']['net_slip_m'] > wp.DRIFT_LIMIT_M,
        'no_runtime_qpos_write': all(a['qpos_writes'].get('run', 0) == 0 for a in (normal, fault)),
    }
    return {'scope': 'THREE_FREE_PARTS_W2_RETENTION_DIAGNOSTIC',
            'checks': {k: 'PASS' if v else 'FAIL' for k, v in checks.items()},
            'diagnostic_result': 'PASS' if all(checks.values()) else 'FAIL',
            'arm_loading': 'NOT_RUN', 'full_order': 'NOT_RUN', 'v1_complete': False}


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--run-id', required=True)
    args = ap.parse_args()
    out = (ROOT / 'reports' / args.run_id).resolve()
    if not out.is_relative_to(ROOT / 'reports') or out.exists():
        ap.error('invalid or existing evidence directory')
    out.mkdir(parents=True)
    report = {'source_world_sha256': hashlib.sha256(wp.WORLD.read_bytes()).hexdigest(),
              'initial_inventory': '2 red boxes + 1 blue cylinder; each 30 g',
              'hardware_geometry_changed': False, 'slip_budget_m': wp.DRIFT_LIMIT_M}
    for field, driven in [('nominal', False), ('fault_deck_driven', True)]:
        report[field] = run_arm(driven)
        (out / 'report.json').write_text(json.dumps(report, indent=2, default=_json_scalar)+'\n')
        print(field, report[field]['transport']['net_slip_m'], flush=True)
    acceptance = judge(report)
    (out / 'acceptance.json').write_text(json.dumps(acceptance, indent=2)+'\n')
    print(json.dumps(acceptance, indent=2), flush=True)
    return 0 if acceptance['diagnostic_result'] == 'PASS' else 1


if __name__ == '__main__':
    raise SystemExit(main())
