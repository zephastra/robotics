"""Initialized cargo REST forensic probe. No ROS, nav, render, or order claim.

Uses the original physics timestep/contact coefficients. Cargo truth ONLY
records diagnostics; controllers read only their ordinary robot self-state.
"""
import argparse
import hashlib
import json
from pathlib import Path
import time
import numpy as np
import mujoco
from probe_vehicle_rgbd import scene,WORLD
from probe_vehicle_nav2 import json_value,exception_chain
from w4_plant import LogisticsPlant,collision_radius
from world_owner import WorldOwner
from humanoid_posture_permit import HumanoidPosturePermit
from probe_h3_w5 import H1_runtime
from humanoid007.runtime import OPEN

ROOT=Path(__file__).resolve().parents[1]
CARGO=('c_payload','optical_red_0','optical_red_1','optical_blue')


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--run-id',required=True)
    p.add_argument('--elliptic-cone',action='store_true',help='independent numerical contact candidate; original coefficients unchanged')
    p.add_argument('--impratio',type=float,help='independent friction/normal impedance ratio candidate, not a friction coefficient')
    p.add_argument('--multi-ccd',action='store_true',help='init-only independent multi-contact candidate; old model untouched')
    a=p.parse_args()
    if a.impratio is not None and (not a.elliptic_cone or not np.isfinite(a.impratio) or not 1<=a.impratio<=10):
        p.error('--impratio requires elliptic cone and finite value [1,10]')
    if Path(a.run_id).name!=a.run_id:p.error('bare run ID required')
    out=ROOT/'reports'/a.run_id;out.mkdir(exist_ok=False)
    started=time.monotonic();rows=[];contacts=[];report=dict(scope='INITIALIZED_CARGO_REST_DIAGNOSTIC',
        full_order='NOT_RUN',v1_complete=False,runtime_qpos_writes=0,physics_contact_changes=[],
        world_sha256=hashlib.sha256(WORLD.read_bytes()).hexdigest(),
        source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest())
    try:
        model,data,_,_,_=scene(shift=(-.15,0.),source_contacts=True)
        report['original_cone']=int(model.opt.cone)
        if a.elliptic_cone:model.opt.cone=mujoco.mjtCone.mjCONE_ELLIPTIC
        report['original_impratio']=float(model.opt.impratio)
        if a.impratio is not None:model.opt.impratio=a.impratio
        report['candidate_impratio']=float(model.opt.impratio)
        report['candidate_cone']=int(model.opt.cone)
        report['numerical_candidate']='ELLIPTIC_CONE' if a.elliptic_cone else 'BASELINE'
        if a.multi_ccd:
            from contact_model_candidate import enable_multiccd
            report['contact_candidate']=enable_multiccd(model)
        plant=LogisticsPlant(model=model,data=data,world=WORLD)
        plant.deploy_pusher(lift=.001,slide=.01324)
        for name,value in (('c_pusher_lift_joint',.001),('c_pusher_slide_joint',.01324)):
            data.qpos[model.jnt_qposadr[model.joint(name).id]]=value
        mujoco.mj_forward(model,data)
        sequence=[0]
        def observation():
            sequence[0]+=1
            return dict(owner='world',epoch=1,generation=1,sequence=sequence[0],
                observed_wall_s=time.monotonic(),cancel_requested=False,zone_clear=True,stop_chain_healthy=True)
        permit=HumanoidPosturePermit(observation,model=model,data=data,
            owner='world',epoch=1,generation=1,max_age_s=.5)
        owner=WorldOwner(model,data,permit);owner.attach(plant)
        robot=H1_runtime(WORLD,model=model,data=data);arm=robot.policy.default[robot.arm_ids].copy()
        def control():
            result=robot.control(np.zeros(3),arm,{s:OPEN for s in ('left','right')},
                stationary=False,stopping=True,base_control=plant.park_control())
            robot.tick+=1
            return result
        next_sample=0.;next_progress=0.
        blue=model.body('optical_blue').id
        while data.time<20:
            if time.monotonic()-started>150:raise TimeoutError('cargo rest wall budget')
            owner.step(control)
            if data.time>=next_sample:
                row=dict(sim_s=float(data.time),base_xyz=data.body('n_base_link').xpos.tolist(),cargo={})
                for name in CARGO:
                    bid=model.body(name).id;v=np.empty(6)
                    mujoco.mj_objectVelocity(model,data,mujoco.mjtObj.mjOBJ_BODY,bid,v,0)
                    row['cargo'][name]=dict(xyz=data.xpos[bid].tolist(),velocity_world=v.tolist(),
                        point_speed_bound_mps=float(np.linalg.norm(v[3:])+collision_radius(model,bid)*np.linalg.norm(v[:3])))
                rows.append(row)
                for i in range(data.ncon):
                    c=data.contact[i];pair=[int(g) for g in c.geom]
                    if blue not in [int(model.geom_bodyid[g]) for g in pair]:continue
                    force=np.empty(6);mujoco.mj_contactForce(model,data,i,force)
                    contacts.append(dict(sim_s=float(data.time),geoms=[model.geom(g).name for g in pair],
                        dimension=int(c.dim),friction=c.friction.tolist(),distance_m=float(c.dist),
                        force_contact_frame=force.tolist()))
                next_sample+=.05
            if data.time>=next_progress:
                print('cargo rest sim %.2f'%data.time,flush=True);next_progress+=2.
        held=[r for r in rows if r['sim_s']>=5]
        peaks={name:max(r['cargo'][name]['point_speed_bound_mps'] for r in held) for name in CARGO}
        report.update(status='COMPLETED',peaks_mps=peaks,
            rest_result='PASS' if all(v<=.01 for v in peaks.values()) else 'FAIL',
            timestep_s=float(model.opt.timestep),simulation_frozen=owner.frozen)
    except Exception as e:report.update(status='ERROR',error=exception_chain(e),rest_result='FAIL')
    report.update(truth_judge_only=rows,blue_contacts_judge_only=contacts,wall_s=time.monotonic()-started)
    (out/'report.json').write_text(json.dumps(report,indent=2,default=json_value)+'\n')
    print(json.dumps({k:v for k,v in report.items() if k not in ('truth_judge_only','blue_contacts_judge_only')},indent=2))
    return 0 if report['rest_result']=='PASS' else 1


if __name__=='__main__':raise SystemExit(main())
