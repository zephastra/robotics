"""Physical supply then long standing hold, with continuous original fall guard.

Isolates humanoid/controller stability; no rendering, arm loading, Nav2 or order.
"""
import argparse
import json
from pathlib import Path
import sys
import time
import mujoco
ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/'src'),str(ROOT/'experiments')]
from w4_plant import LogisticsPlant
from world_owner import WorldOwner
from humanoid_posture_permit import HumanoidPosturePermit,posture_status
from humanoid_supply import supply
import vehicle_heading


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run-id',required=True)
    parser.add_argument('--mode',choices=['static','policy'],required=True)
    parser.add_argument('--hold-duration',type=float,default=90.)
    args=parser.parse_args()
    if Path(args.run_id).name!=args.run_id or not 0<args.hold_duration<=120:
        parser.error('bare run ID and bounded duration required')
    out=ROOT/'reports'/args.run_id;out.mkdir(parents=True,exist_ok=False)
    world=ROOT/'assets/world_p5_candidate_v5.xml'
    report=dict(scope='H_SUPPLY_POST_EXIT_STABILITY_ONLY_NOT_LOADING_OR_ORDER',mode=args.mode,
        status='FAILED',full_order='NOT_RUN',v1_complete=False,physics_trace=[])
    owner=None;started=time.monotonic()
    try:
        model=mujoco.MjModel.from_xml_path(str(world));data=mujoco.MjData(model)
        mujoco.mj_resetDataKeyframe(model,data,0);mujoco.mj_forward(model,data)
        plant=LogisticsPlant(model=model,data=data,world=world)
        vehicle_heading.install(plant)
        sequence=[0]
        def observe():
            sequence[0]+=1
            return dict(owner='world',epoch=1,generation=1,sequence=sequence[0],
                observed_wall_s=time.monotonic(),cancel_requested=False,zone_clear=True,stop_chain_healthy=True)
        permit=HumanoidPosturePermit(observe,model=model,data=data,
            owner='world',epoch=1,generation=1,max_age_s=.5)
        owner=WorldOwner(model,data,permit);owner.attach(plant)
        initial=float(data.body('c_payload').xpos[0])
        supplied,compose=supply(model,data,plant,owner,world,initial,post_mode=args.mode)
        report['humanoid_supply']=supplied
        (out/'humanoid_supply.json').write_text(json.dumps(supplied,indent=2)+'\n')
        if supplied['status']!='SUCCEEDED':raise RuntimeError('SUPPLY_FAILED')
        begin=float(data.time);next_sample=begin
        while data.time-begin<args.hold_duration:
            owner.step(lambda:compose(plant.park_control()))
            if data.time>=next_sample:
                adr=permit.address
                report['physics_trace'].append(dict(sim_s=float(data.time),
                    posture=posture_status(float(data.qpos[adr+2]),data.qpos[adr+3:adr+7]),
                    robot_xyz=data.qpos[adr:adr+3].tolist(),
                    vehicle_xyz=data.body('n_base_link').xpos.tolist(),
                    vehicle_yaw_rad=vehicle_heading.measured_heading(plant)[0]))
                next_sample+=5.
                print('post-supply %s %.2f/%.2f sim s' % (args.mode,data.time-begin,args.hold_duration),flush=True)
        report.update(status='COMPLETED',sim_s=float(data.time),hold_s=float(data.time-begin))
    except Exception as exc:
        report['error']=type(exc).__name__+': '+str(exc)
    finally:
        report['wall_s']=time.monotonic()-started
        if owner is not None:
            report['writer_frozen']=owner.frozen
            report['last_posture']=owner.permit.last_posture
        (out/'report.json').write_text(json.dumps(report,indent=2)+'\n')
        print(json.dumps({k:v for k,v in report.items() if k not in ('physics_trace','humanoid_supply')},indent=2))
    return 0 if report['status']=='COMPLETED' else 1


if __name__=='__main__':raise SystemExit(main())
