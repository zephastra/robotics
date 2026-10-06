"""Commission existing-world scan/clock/wheel IPC only; NOT Nav2 or order acceptance.

One existing LogisticsPlant and humanoid share one WorldOwner. Optional ROS child
is owned by this run and has no physics. No runtime position writes or resets.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
import time
import mujoco
import numpy as np
import yaml
ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/'src'),str(ROOT/'experiments')]
from w4_plant import LogisticsPlant
from world_owner import WorldOwner
from workcell.runtime_permit import RuntimePermit
from probe_h3_w5 import H1_runtime
from humanoid_supply import stationary_control
from humanoid007.runtime import OPEN
from joint_world_nav_io import JointWorldNavIO


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run-id',required=True)
    parser.add_argument('--duration',type=float,default=10.)
    parser.add_argument('--ros',action='store_true')
    args=parser.parse_args()
    if Path(args.run_id).name!=args.run_id or not np.isfinite(args.duration) or not 0<args.duration<=60:
        parser.error('bare run ID and bounded duration required')
    out=ROOT/'reports'/args.run_id;out.mkdir(parents=True,exist_ok=False)
    world=ROOT/'assets/world_p5_candidate_v5.xml'
    cfg=yaml.safe_load((ROOT/'config/n_probe.yaml').read_text())
    report=dict(scope='EXISTING_SHARED_WORLD_IPC_ONLY_NOT_NAV2_OR_ORDER',
        world_sha256=hashlib.sha256(world.read_bytes()).hexdigest(),
        source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        status='ERROR',runtime_qpos_writes=0,nav2='NOT_RUN',full_order='NOT_RUN',v1_complete=False)
    ros=None;log=None;io=None
    started=time.monotonic()
    try:
        model=mujoco.MjModel.from_xml_path(str(world));data=mujoco.MjData(model)
        mujoco.mj_resetDataKeyframe(model,data,0);mujoco.mj_forward(model,data)
        plant=LogisticsPlant(model=model,data=data,world=world)
        sequence=[0]
        def observe():
            sequence[0]+=1
            return dict(owner='world',epoch=1,generation=1,sequence=sequence[0],
                observed_wall_s=time.monotonic(),cancel_requested=False,
                zone_clear=True,stop_chain_healthy=True)
        owner=WorldOwner(model,data,RuntimePermit(observe,owner='world',epoch=1,generation=1,max_age_s=.5))
        owner.attach(plant)
        robot=H1_runtime(world,model=model,data=data)
        arm=robot.policy.default[robot.arm_ids].copy()
        io=JointWorldNavIO(plant,cfg)
        if args.ros:
            log=(out/'ros_stdout.log').open('w')
            # bash executes a fixed source/exec command; the run ID is validated above.
            command=['bash','-lc','source scripts/env.sh\nexec /usr/bin/python3 '
                'experiments/joint_world_ros_bridge.py --command-source cmd_vel '
                '--run-id '+args.run_id+' --duration '+str(args.duration)+' --wait-for-state 30']
            ros=subprocess.Popen(command,cwd=ROOT,stdout=log,stderr=subprocess.STDOUT)
            report['ros_child_pid']=ros.pid
            report['ros_child_start_ticks']=Path('/proc/%d/stat'%ros.pid).read_text().rsplit(')',1)[1].split()[19]
        truth=[];next_publish=float(data.time);next_sample=float(data.time)
        begin=float(data.time);wall_begin=time.monotonic()
        def compose():
            base=io.control()
            if robot.stand_weight==robot.arm_weight==1. and robot.stand_target is not None:
                result=stationary_control(robot,arm,base)
            else:
                result=robot.control(np.zeros(3),arm,{s:OPEN for s in ('left','right')},
                                     stationary=True,base_control=base)
            robot.tick+=1
            return result
        while data.time-begin<args.duration:
            if time.monotonic()-started>240:raise TimeoutError('owned probe wall deadline')
            if data.time>=next_publish:
                payload=io.publish();next_publish+=1/float(cfg['sim']['state_rate_hz'])
                report['last_scan_range_m']=[min(payload['ranges']),max(payload['ranges'])]
            owner.step(compose)
            if data.time>=next_sample:
                truth.append(dict(sim_s=float(data.time),xyz=data.body('n_base_link').xpos.tolist()))
                next_sample+=.5
            # Wall pacing keeps the ROS process from being outrun by physics.
            delay=(float(data.time)-begin)/float(cfg['sim']['realtime_factor'])-(time.monotonic()-wall_begin)
            if delay>0:time.sleep(min(delay,.05))
        io.publish()
        report.update(status='COMPLETED',sim_s=float(data.time),owner_steps=owner.steps,
            states_sent=io.sent,commands_received=io.received,truth_judge_only=truth,
            simulation_frozen=owner.frozen,
            same_model=robot.m is model and plant.model is model,
            same_data=robot.d is data and plant.data is data)
        if ros is not None:
            report['ros_exit_code']=ros.wait(timeout=10)
            ros_report=json.loads((out/'ros_report.json').read_text())
            report['ros_bridge_status']=ros_report['status']
            report['ros_counters']=ros_report['counters']
            if ros.returncode!=0 or ros_report['counters']['decode_refusals']!=0:
                report['status']='ROS_BRIDGE_FAILED'
    except Exception as exc:
        report['error']=type(exc).__name__+': '+str(exc)
    finally:
        if io is not None:io.close()
        if ros is not None and ros.poll() is None:
            # Direct child retained by Popen, not a global/pattern process kill.
            ros.terminate()
            try:ros.wait(timeout=5)
            except subprocess.TimeoutExpired:ros.kill();ros.wait()
        if log is not None:log.close()
        report['wall_s']=time.monotonic()-started
        (out/'report.json').write_text(json.dumps(report,indent=2)+'\n')
        print(json.dumps(report,indent=2),flush=True)
    return 0 if report['status']=='COMPLETED' else 1


if __name__=='__main__':raise SystemExit(main())
