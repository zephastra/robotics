"""Bounded physical transfer interruption; no order/transaction integration claim.

Frozen W2 hardware unchanged. Intruder is a DECLARED mocap fault object, not
an autonomous human model. Distance readings emulate ideal proximity sensing.
Wall-time permission expires independently of simulation progress.
"""
import argparse
import hashlib
import json
import sys
import time
from pathlib import Path

import mujoco
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'experiments'))
sys.path.insert(0, str(ROOT / 'src'))
import w4_plant as wp
from probe_p4_interlock import MARGIN_M, BUDGET_M
from workcell.runtime_permit import RuntimePermit


def build():
    wp.bp3.install()
    spec = mujoco.MjSpec.from_file(str(wp.WORLD))
    for key in list(spec.keys):
        spec.delete(key)
    # Initial x/z derive from the source fixture, not a runtime cargo pose.
    baseline = mujoco.MjModel.from_xml_path(str(wp.WORLD))
    bd = mujoco.MjData(baseline)
    mujoco.mj_forward(baseline, bd)
    crown = baseline.geom('c_fixed_roller_0_0').id
    pos = [float(bd.geom_xpos[crown][0]), -.8,
           float(bd.geom_xpos[crown][2]) + .10]
    body = spec.worldbody.add_body(name='fault_intruder', mocap=True, pos=pos)
    body.add_geom(name='fault_intruder_geom', type=mujoco.mjtGeom.mjGEOM_SPHERE,
                  size=[.03, 0, 0], rgba=[1, .7, 0, 1], contype=1, conaffinity=1)
    model = spec.compile()
    data = mujoco.MjData(model)
    data.qpos[:] = wp.mw.merged_home(model)[0]  # sole episode initialisation
    mujoco.mj_forward(model, data)
    return wp.LogisticsPlant(model=model, data=data), pos


def run_arm(kind):
    plant, initial = build()
    m, d = plant.model, plant.data
    sensor_geoms = [g for g in range(m.ngeom)
        if (mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_GEOM, g) or '').startswith('c_fixed_roller')
        and (m.geom_contype[g] or m.geom_conaffinity[g])]
    intruder = m.geom('fault_intruder_geom').id
    mocap = int(m.body_mocapid[m.body('fault_intruder').id])
    generation = [1]
    sequence = [0]
    injecting = [True]
    fault_writes = [0]
    started_wall = time.monotonic()
    distances = []

    def observe():
        sequence[0] += 1
        now = time.monotonic()
        if kind == 'intrusion' and injecting[0]:
            # Explicit fault trajectory, 0.4 m/s, terminated by the stop latch.
            d.mocap_pos[mocap] = [initial[0], initial[1] + min(.65, .4 * d.time), initial[2]]
            mujoco.mj_forward(m, d)
            fault_writes[0] += 1
        distance = min(mujoco.mj_geomDistance(m, d, intruder, g, BUDGET_M, None)
                       for g in sensor_geoms) if sensor_geoms else float('nan')
        distances.append(float(distance))
        return dict(owner='recovery_transfer', epoch=1, generation=generation[0],
                    sequence=sequence[0], observed_wall_s=(started_wall-1
                        if kind == 'expiry' and injecting[0] and d.time >= .3 else now),
                    cancel_requested=(kind == 'cancel' and injecting[0] and d.time >= .3),
                    zone_clear=bool(np.isfinite(distance) and distance >= MARGIN_M),
                    stop_chain_healthy=True)

    guard = RuntimePermit(observe, owner='recovery_transfer', epoch=1, generation=1,
                          max_age_s=.5)
    plant.motion_guard = guard
    first = plant.transfer(direction='onto_deck', timeout_s=.3 if kind == 'budget' else 5.)
    before_repeat = float(d.time)
    p0 = np.array(d.xpos[m.body('c_payload').id])
    repeat = plant.transfer(direction='onto_deck', timeout_s=5.)
    repeat_displacement = float(np.linalg.norm(d.xpos[m.body('c_payload').id] - p0))
    recovery = {'status': 'NOT_RUN'}
    if kind == 'cancel' and first.get('brake', {}).get('stopped_confirmed'):
        if plant.tray_support() == ['source_band']:
            injecting[0], generation[0] = False, 2
            authorized = guard.rearm(generation=2, stopped_confirmed=True)
            recovery = {'authorization': authorized,
                        'transfer': plant.transfer(direction='onto_deck', timeout_s=20.)}
    return {'kind': kind, 'first': first, 'repeat': repeat,
            'repeat_displacement_m': repeat_displacement,
            'repeat_elapsed_sim_s': float(repeat['end_s'] - before_repeat),
            'recovery': recovery, 'final_tray': plant.tray_state(),
            'runtime_qpos_writes': plant.qpos_writes.get('run', 0),
            'explicit_mocap_fault_writes': fault_writes[0],
            'minimum_distance_m': min(distances), 'guard_checks': guard.checks}


def judge(report):
    checks = {}
    safety_checks = {}
    expected = dict(cancel='REQUEST_CONFLICT', intrusion='RESOURCE_UNKNOWN',
                    expiry='STALE_OBSERVATION', budget='TRANSFER_TIMEOUT')
    for arm in report['arms']:
        key = arm['kind']
        first, repeat = arm['first'], arm['repeat']
        checks[key+'_interrupted'] = first.get('interrupted') is True
        checks[key+'_reason'] = first.get('reason_code') == expected[key]
        checks[key+'_actually_stopped'] = first.get('brake', {}).get('stopped_confirmed') is True
        checks[key+'_repeat_stays_latched'] = (repeat.get('interrupted') is True
                                              and repeat.get('state') != 'RECEIVED')
        checks[key+'_repeat_has_no_transfer'] = arm['repeat_displacement_m'] <= wp.DRIFT_LIMIT_M
        checks[key+'_no_teleport'] = arm['runtime_qpos_writes'] == 0
        safety_checks[key+'_runtime_interrupted'] = first.get('interrupted') is True
        safety_checks[key+'_no_unconfirmed_resume'] = (repeat.get('interrupted') is True
            and repeat.get('state') != 'RECEIVED')
        safety_checks[key+'_safe_hold'] = (first.get('brake', {}).get('stopped_confirmed') is True
            or (first.get('brake', {}).get('simulation_frozen') is True
                and repeat.get('simulation_frozen') is True
                and arm['repeat_elapsed_sim_s'] == 0
                and arm['repeat_displacement_m'] == 0))
    arms = {x['kind']: x for x in report['arms']}
    checks['complete_fault_set'] = set(arms) == set(expected)
    safety_checks['complete_fault_set'] = set(arms) == set(expected)
    checks['intrusion_was_measured'] = (arms['intrusion']['minimum_distance_m'] < MARGIN_M
        and arms['intrusion']['explicit_mocap_fault_writes'] > 0)
    checks['authorized_source_recovery'] = (arms['cancel']['recovery'].get('transfer', {})
                                           .get('state') == 'RECEIVED')
    return {'scope': 'W2_TRANSFER_RUNTIME_STOP_AND_EXPLICIT_RECOVERY_ONLY',
            'checks': {k:'PASS' if v else 'FAIL' for k,v in checks.items()},
            'diagnostic_result': 'PASS' if all(checks.values()) else 'FAIL',
            'safety_checks': {k:'PASS' if v else 'FAIL' for k,v in safety_checks.items()},
            'safety_result': 'PASS' if all(safety_checks.values()) else 'FAIL',
            'mechanical_stop_result': ('PASS' if all(checks[k+'_actually_stopped']
                for k in expected) else 'FAIL'),
            'transaction_recovery': 'NOT_RUN', 'full_order': 'NOT_RUN', 'v1_complete': False}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--run-id', required=True)
    parser.add_argument('--world', default='world_w2_logistic.xml',
                        choices=['world_w2_logistic.xml', 'world_w2_support_v2.xml'])
    args = parser.parse_args()
    wp.WORLD = ROOT / 'assets' / args.world
    out = (ROOT / 'reports' / args.run_id).resolve()
    if not out.is_relative_to(ROOT / 'reports'):
        raise ValueError('invalid report directory')
    out.mkdir(exist_ok=False)
    report = dict(source_world_sha256=hashlib.sha256(wp.WORLD.read_bytes()).hexdigest(),
        ideal_proximity_sensor=True, fault_trajectory='mocap sphere, 0.4 m/s', arms=[])
    for kind in ('cancel', 'intrusion', 'expiry', 'budget'):
        report['arms'].append(run_arm(kind))
        (out / 'report.json').write_text(json.dumps(report, indent=2)+'\n')
        print(kind, report['arms'][-1]['first'].get('state'), flush=True)
    result = judge(report)
    (out / 'acceptance.json').write_text(json.dumps(result, indent=2)+'\n')
    print(json.dumps(result, indent=2), flush=True)
    return 0 if result['diagnostic_result'] == 'PASS' else 1


if __name__ == '__main__':
    raise SystemExit(main())
