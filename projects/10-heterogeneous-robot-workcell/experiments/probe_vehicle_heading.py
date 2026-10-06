"""Single-world parking/differential steering diagnostic, not logistics acceptance."""
import argparse
import hashlib
import json
import sys
import time
from pathlib import Path
import mujoco
import numpy as np
ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/'experiments'),str(ROOT/'src')]
from w4_plant import LogisticsPlant
from world_owner import WorldOwner
from workcell.runtime_permit import RuntimePermit
from humanoid_supply import stationary_control
from probe_h3_w5 import H1_runtime
from humanoid007.runtime import OPEN
import vehicle_heading


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--run-id',required=True)
    ap.add_argument('--yaw',type=float,default=0.)
    ap.add_argument('--feedback',action='store_true')
    ap.add_argument('--duration',type=float,default=30.)
    args=ap.parse_args()
    if not np.isfinite(args.yaw) or not 0<args.duration<=200:ap.error('finite bounded inputs required')
    world=ROOT/'assets/world_p5_candidate_v5.xml'
    out=ROOT/'reports'/args.run_id;out.mkdir(parents=True,exist_ok=False)
    m=mujoco.MjModel.from_xml_path(str(world));d=mujoco.MjData(m)
    mujoco.mj_resetDataKeyframe(m,d,0)
    # Declared initial fault only, before first step. No state writes thereafter.
    adr=int(m.jnt_qposadr[m.joint('n_yaw').id]);d.qpos[adr]=args.yaw
    mujoco.mj_forward(m,d)
    p=LogisticsPlant(model=m,data=d,world=world)
    seq=[0]
    def observe():
        seq[0]+=1
        return dict(owner='world',epoch=1,generation=1,sequence=seq[0],
            observed_wall_s=time.monotonic(),cancel_requested=False,zone_clear=True,stop_chain_healthy=True)
    owner=WorldOwner(m,d,RuntimePermit(observe,owner='world',epoch=1,generation=1,max_age_s=.5))
    owner.attach(p)
    if args.feedback:vehicle_heading.install(p)
    r=H1_runtime(world,model=m,data=d)
    default=r.policy.default[r.arm_ids].copy()
    samples=[];next_sample=0.;started=time.monotonic();start=float(d.time)
    def compose():
        base=p.park_control()
        if r.stand_weight==r.arm_weight==1. and r.stand_target is not None:
            ctrl=stationary_control(r,default,base)
        else:
            ctrl=r.control(np.zeros(3),default,{s:OPEN for s in ('left','right')},
                           stationary=True,base_control=base)
        r.tick+=1
        return ctrl
    while d.time-start<args.duration:
        owner.step(compose)
        if d.time-start>=next_sample:
            yaw,rate=vehicle_heading.measured_heading(p)
            frame=d.geom_xmat[m.geom('c_deck_frame').id].reshape(3,3)
            wheel_ids=list(p.wheel_actuators.values())
            cross_contacts=[]
            for c in d.contact[:d.ncon]:
                names=[m.geom(g).name or '' for g in (c.geom1,c.geom2)]
                moving=[name.startswith(('n_','c_deck','c_pusher')) for name in names]
                if any(moving) and not all(moving):
                    cross_contacts.append(dict(geoms=names,penetration_m=float(c.dist)))
            samples.append(dict(sim_s=float(d.time),base_yaw_rad=yaw,base_yaw_rate_radps=rate,
                wheel_control=d.ctrl[wheel_ids].tolist(),wheel_force=d.actuator_force[wheel_ids].tolist(),
                cross_contacts=cross_contacts,
                deck_relative_yaw_rad=float(d.qpos[m.jnt_qposadr[m.joint('c_deck_yaw').id]]),
                deck_world_yaw_rad=float(np.arctan2(frame[1,0],frame[0,0])),
                chassis_xyz=d.body('n_base_link').xpos.tolist(),
                wheel_rates=[float(d.qvel[m.jnt_dofadr[m.joint('n_wheel_'+s+'_joint').id]])
                             for s in ('left','right')]))
            next_sample+=1.
    report=dict(scope='INITIAL_FAULT_PARKING_ONLY_NOT_H_SUPPLY_OR_NAV2',feedback=args.feedback,
        initial_yaw_rad=args.yaw,sim_duration_s=float(d.time-start),samples=samples,
        wheel_signs=p._wheel_signs(),world_sha256=hashlib.sha256(world.read_bytes()).hexdigest(),
        runtime_qpos_writes=0,calibration_qpos_writes=p.qpos_writes,
        wall_s=time.monotonic()-started,source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest())
    (out/'report.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps({k:v for k,v in report.items() if k!='samples'},indent=2),flush=True)
    print(json.dumps(samples[-1]),flush=True)


if __name__=='__main__':main()
