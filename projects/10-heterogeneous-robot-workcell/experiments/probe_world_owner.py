"""Candidate-wide final-writer interruption proof; NOT an order/recovery proof.

Fault flags emulate trusted external sensor/authorization evidence. Simulation
freeze is reported separately from physical stop; never resets cargo to recover.
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
sys.path[:0] = [str(ROOT/'experiments'), str(ROOT/'src')]
from workcell.runtime_permit import RuntimePermit
from w4_plant import LogisticsPlant
from world_owner import WorldOwner, WorldSafetyHold


def run(kind, real_humanoid=False):
    world = ROOT/'assets/world_p5_candidate_v3.xml'
    model = mujoco.MjModel.from_xml_path(str(world))
    data = mujoco.MjData(model)
    mujoco.mj_resetDataKeyframe(model, data, 0)  # declared sole episode initialization
    plant = LogisticsPlant(model=model, data=data)
    runtime = None
    if real_humanoid:
        from probe_h3_w5 import H1_runtime
        runtime = H1_runtime(world,model=model,data=data)
    fault = [False]
    sequence = [0]
    def observe():
        sequence[0] += 1
        now = time.monotonic()
        return dict(owner='shared_world', epoch=1, generation=1, sequence=sequence[0],
                    observed_wall_s=now-1 if fault[0] and kind=='expiry' else now,
                    cancel_requested=fault[0] and kind=='cancel',
                    zone_clear=not (fault[0] and kind=='intrusion'),
                    stop_chain_healthy=not (fault[0] and kind=='stop_chain'))
    permit = RuntimePermit(observe, owner='shared_world', epoch=1, generation=1, max_age_s=.5)
    owner = WorldOwner(model, data, permit)
    owner.attach(plant)
    if runtime is not None:
        runtime.step_owner=owner
        runtime.background_control=plant.park_control
    for _ in range(20):
        if runtime is None:
            plant._tick(lambda side: plant._wheel_rate_torque(side, 0.0))
        else:
            runtime.step(np.zeros(3),stationary=True)
    before = dict(time=float(data.time), qpos=data.qpos.copy(),
                  qvel=data.qvel.copy(), ctrl=data.ctrl.copy())
    fault[0] = True
    rejected = []
    for role in ('c_transfer', 'h', 'a', 'c', 'n', 'n2'):
        try:
            if role == 'c_transfer':
                plant.transfer(direction='onto_deck',timeout_s=.1)
            elif role == 'h' and runtime is not None:
                runtime.step(np.zeros(3),stationary=True)
            elif role == 'c':
                plant._tick(lambda side: 1.)
            else:
                owner.step(lambda: plant.park_control())
        except WorldSafetyHold:
            rejected.append(role)
    fault[0] = False  # spontaneous evidence recovery must NOT reopen the gate
    try:
        owner.step(lambda: plant.park_control())
        auto_resumed = True
    except WorldSafetyHold:
        auto_resumed = False
    return dict(kind=kind, rejected_roles=rejected, auto_resumed=auto_resumed,
        sim_time_unchanged=float(data.time)==before['time'],
        qpos_unchanged=bool(np.array_equal(data.qpos, before['qpos'])),
        qvel_unchanged=bool(np.array_equal(data.qvel, before['qvel'])),
        ctrl_unchanged=bool(np.array_equal(data.ctrl, before['ctrl'])),
        frozen=owner.frozen, final_steps=owner.steps,
        runtime_qpos_writes=plant.qpos_writes.get('run', 0),
        shared_model_and_data=plant.model is owner.model and plant.data is owner.data,
        real_humanoid_entry=runtime is not None)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run-id', required=True)
    parser.add_argument('--real-humanoid',action='store_true')
    args = parser.parse_args()
    out = ROOT/'reports'/args.run_id
    out.mkdir(parents=True, exist_ok=False)
    report = dict(scope='GLOBAL_WRITER_SIMULATION_FREEZE_ONLY',
        world_sha256=hashlib.sha256((ROOT/'assets/world_p5_candidate_v3.xml').read_bytes()).hexdigest(),
        ideal_external_fault_evidence=True, cases=[run(kind,args.real_humanoid) for kind in
            ('cancel', 'intrusion', 'expiry', 'stop_chain')])
    checks = {}
    for case in report['cases']:
        kind = case['kind']
        checks[kind+'_all_role_paths_refused'] = case['rejected_roles']==['c_transfer','h','a','c','n','n2']
        checks[kind+'_no_state_advance'] = all(case[k] for k in
            ('sim_time_unchanged','qpos_unchanged','qvel_unchanged','ctrl_unchanged'))
        checks[kind+'_latched'] = not case['auto_resumed']
        checks[kind+'_single_world'] = case['shared_model_and_data']
        checks[kind+'_not_mechanical_claim'] = case['frozen']['stopped_confirmed'] is False
        checks[kind+'_no_runtime_teleport'] = case['runtime_qpos_writes']==0
    acceptance = dict(scope=report['scope'], checks={k:'PASS' if v else 'FAIL' for k,v in checks.items()},
        diagnostic_result='PASS' if all(checks.values()) else 'FAIL',
        mechanical_stop='NOT_RUN', transaction_recovery='NOT_RUN',
        full_order='NOT_RUN', old_h3_migrated=False, v1_complete=False)
    (out/'report.json').write_text(json.dumps(report, indent=2)+'\n')
    (out/'acceptance.json').write_text(json.dumps(acceptance, indent=2)+'\n')
    print(json.dumps(acceptance, indent=2))
    return 0 if acceptance['diagnostic_result']=='PASS' else 1


if __name__ == '__main__':
    raise SystemExit(main())
