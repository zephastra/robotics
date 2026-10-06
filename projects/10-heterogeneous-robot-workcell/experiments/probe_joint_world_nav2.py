"""Empty v5 shared-world Nav2 commissioning, not loaded transport or V1.

Only WorldOwner advances physics. ROS sees scan/wheel odometry, never truth
pose. Independent truth is saved for arrival/stop judgement after execution.
"""
import argparse
import hashlib
import json
from pathlib import Path
import signal
import sys
import time
import mujoco
import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT/'src'), str(ROOT/'experiments')]
from w4_plant import LogisticsPlant
from world_owner import WorldOwner
from humanoid_posture_permit import HumanoidPosturePermit
from probe_h3_w5 import H1_runtime
from humanoid007.runtime import OPEN
from joint_world_nav_io import JointWorldNavIO
from make_joint_nav_profile import posture_check
from probe_n_nav import Processes


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run-id', required=True)
    parser.add_argument('--duration', type=float, default=60.)
    parser.add_argument('--parking-handoff', action='store_true')
    args = parser.parse_args()
    if Path(args.run_id).name != args.run_id or not np.isfinite(args.duration) or not 10<=args.duration<=90:
        parser.error('bare run ID and sim duration [10,90] required')
    out = ROOT/'reports'/args.run_id; out.mkdir(parents=True, exist_ok=False)
    cfg = yaml.safe_load((ROOT/'config/joint_world_v5.yaml').read_text())
    profile = json.loads((ROOT/'config/joint_world_v5.profile.json').read_text())
    world = ROOT/cfg['world']; owned = Processes(out); io = None; owner = None
    truth = []; stopping_at = None
    started = time.monotonic(); report = dict(scope=profile['scope'], status='ERROR',
        world_sha256=hashlib.sha256(world.read_bytes()).hexdigest(),
        source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        runtime_qpos_writes=0, loaded_navigation='NOT_RUN', full_order='NOT_RUN', v1_complete=False)
    def interrupted(signum, frame): raise KeyboardInterrupt('owned simulation interrupted')
    signal.signal(signal.SIGTERM, interrupted)
    try:
        if report['world_sha256'] != profile['world_sha256']: raise ValueError('profile world hash mismatch')
        model = mujoco.MjModel.from_xml_path(str(world)); data = mujoco.MjData(model)
        mujoco.mj_resetDataKeyframe(model, data, 0); mujoco.mj_forward(model, data)
        plant = LogisticsPlant(model=model, data=data, world=world)
        sequence = [0]
        def observe():
            sequence[0] += 1
            return dict(owner='world', epoch=1, generation=1, sequence=sequence[0],
                observed_wall_s=time.monotonic(), cancel_requested=False,
                zone_clear=True, stop_chain_healthy=True)
        permit = HumanoidPosturePermit(observe, model=model, data=data,
            owner='world', epoch=1, generation=1, max_age_s=.5)
        owner = WorldOwner(model, data, permit); owner.attach(plant)
        authority_token = None
        if args.parking_handoff:
            import vehicle_heading
            from wheel_authority import grant_navigation
            vehicle_heading.install(plant)
            authority_token = grant_navigation(plant)
        robot = H1_runtime(world, model=model, data=data)
        arm = robot.policy.default[robot.arm_ids].copy()
        io = JointWorldNavIO(plant, cfg)
        def compose():
            posture = posture_check(model, data)
            if not posture['allowed']:
                permit.stop(posture['reason_code'], str(posture))
                raise RuntimeError(posture['reason_code'])
            control = robot.control(np.zeros(3), arm, {s:OPEN for s in ('left','right')},
                stationary=False, stopping=True, base_control=io.control())
            robot.tick += 1
            return control
        # Fixed, validated strings only; own children have no physical simulator.
        for name, script, flags in (
            ('bridge', 'joint_world_ros_bridge.py', '--config config/joint_world_v5.yaml --command-source cmd_vel --duration '+str(args.duration+10)+' --wait-for-state 30'),
            ('worker', 'joint_nav_worker.py', '--wall-budget 300')):
            owned.spawn(name, ['bash','-lc','source scripts/env.sh\nexec /usr/bin/python3 experiments/'+script+' --run-id '+args.run_id+' '+flags])
        begin = float(data.time); wall_begin = time.monotonic()
        next_publish = begin; next_sample = begin; next_progress = begin
        worker_result = None
        worker_path = out/'nav2_worker_report.json'
        while data.time-begin < args.duration:
            if time.monotonic()-started > 380: raise TimeoutError('shared Nav2 wall budget')
            if owned.handles['bridge'].poll() is not None: raise RuntimeError('bridge exited during physics')
            if worker_path.exists() and worker_result is None:
                worker_result = json.loads(worker_path.read_text()); stopping_at = float(data.time)
            if owned.handles['worker'].poll() is not None and worker_result is None:
                raise RuntimeError('worker exited without durable report')
            if data.time >= next_publish:
                io.publish(); next_publish += 1/float(cfg['sim']['state_rate_hz'])
            owner.step(compose)
            if data.time >= next_sample:
                truth.append(dict(sim_s=float(data.time), xyz=data.body('n_base_link').xpos.tolist(),
                    velocity=[float(data.qvel[model.jnt_dofadr[model.joint(name).id]])
                              for name in ('n_slide_x','n_slide_y','n_slide_z','n_yaw')],
                    h_posture=permit.last_posture))
                next_sample += .1
            if data.time >= next_progress:
                print('shared Nav2 sim %.2f commands %d worker %s'%(data.time,io.received,
                    worker_result['status'] if worker_result else 'RUNNING'), flush=True)
                next_progress += 2.
            if stopping_at is not None and data.time-stopping_at >= 5.: break
            delay = (float(data.time)-begin)/float(cfg['sim']['realtime_factor'])-(time.monotonic()-wall_begin)
            if delay > 0: time.sleep(min(delay,.05))
        report.update(status='COMPLETED', nav2_worker=worker_result or {'status':'NOT_COMPLETED'},
            truth_judge_only=truth, states_sent=io.sent, commands_received=io.received,
            sim_s=float(data.time), owner_steps=owner.steps,
            same_model=robot.m is model and plant.model is model,
            same_data=robot.d is data and plant.data is data)
        stop = [row for row in truth if stopping_at is not None and row['sim_s'] >= stopping_at+1.]
        final = np.asarray(truth[-1]['xyz'])[:2]; goal = np.asarray(profile['goal'])[:2]
        drift = max((float(np.linalg.norm(np.asarray(row['xyz'])[:2]-np.asarray(stop[0]['xyz'])[:2])) for row in stop), default=float('inf'))
        speed = max((float(np.linalg.norm(row['velocity'][:3])) for row in stop), default=float('inf'))
        checks = dict(nav2_succeeded=worker_result is not None and worker_result['status']=='SUCCEEDED',
            fresh_motion_command=worker_result is not None and worker_result.get('nonzero_command_count',0)>0 and io.received>0,
            independent_arrival=float(np.linalg.norm(final-goal)) <= .25,
            physical_motion=float(np.linalg.norm(final-np.asarray(profile['start'])))>.1,
            physical_stop=speed<=.01 and drift<=.005, shared_world=report['same_model'] and report['same_data'],
            no_safety_freeze=owner.frozen is None)
        if args.parking_handoff:
            from wheel_authority import return_to_parking
            # Durable worker report is written AFTER its Nav2 launch exits.
            owned.handles['worker'].wait(timeout=5)
            nav_stopped = bool(worker_result and worker_result['processes']['nav2'].get('exit_code') is not None)
            io.close()
            return_to_parking(plant,authority_token,
                dict(stopped_confirmed=checks['physical_stop'],held_speed_mps=speed,drift_m=drift),
                nav_process_stopped=nav_stopped,ipc_closed=io.closed)
            parked_begin=float(data.time)
            def parked_control():
                result=robot.control(np.zeros(3),arm,{s:OPEN for s in ('left','right')},
                    stationary=False,stopping=True,base_control=plant.park_control())
                robot.tick+=1
                return result
            while data.time-parked_begin<1.:owner.step(parked_control)
            checks['parking_authority_returned']=plant.heading_feedback_installed and plant.navigation_authority is None
            report['authority_handoff']='PARKING_TO_NAV2_TO_CONFIRMED_STOP_TO_PARKING'
        report.update(checks={k:'PASS' if v else 'FAIL' for k,v in checks.items()},
            commissioning_result='PASS' if all(checks.values()) else 'FAIL',
            arrival_error_m=float(np.linalg.norm(final-goal)), stop_speed_mps=speed, stop_drift_m=drift)
    except (Exception, KeyboardInterrupt) as exc:
        report['status'] = 'ERROR'
        report['error'] = type(exc).__name__+': '+str(exc)
        report['commissioning_result'] = 'FAIL'
    finally:
        # Child worker first: its finally stops only its own Nav2 process group.
        owned.stop('worker','joint_nav_worker.py'); owned.stop('bridge','joint_world_ros_bridge.py'); owned.close()
        if io is not None: io.close()
        report.update(processes=owned.record, wall_s=time.monotonic()-started,
            truth_judge_only=truth, stopping_at_sim_s=stopping_at,
            simulation_frozen=owner.frozen if owner else None)
        (out/'report.json').write_text(json.dumps(report, indent=2)+'\n')
        print(json.dumps({k:v for k,v in report.items() if k!='truth_judge_only'},indent=2),flush=True)
    return 0 if report.get('commissioning_result')=='PASS' else 1


if __name__=='__main__': raise SystemExit(main())
