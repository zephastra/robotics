"""Physical H2 supply in a caller-owned world; no initialization or actuator writer.

Restricted diagnostic, not full H/fleet acceptance. Uses H2's unchanged schedule,
clearance and position limits. Ideal touch/self-state are explicit instruments.
"""
import numpy as np
import mujoco
import probe_h3_w5 as H3
import probe_h2_w5 as H2
import probe_h_w5 as H1
from humanoid007.runtime import OPEN, GRASP, ExitPath
from humanoid007 import tray_task


def six_stage_passes(result):
    stages=result.get('stages',{})
    return (set(stages)==set(tray_task.STAGES) and result.get('failed')==[]
            and result.get('not_run')==[]
            and all(row.get('verdict')=='PASS' for row in stages.values()))


def stationary_control(runtime,default,base):
    """Same joint feedback once ALL walking outputs are completely overridden.

    Restricted to this supply helper's post-exit stationary mode. No neural
    history is advanced; this runtime must not resume walking afterward. Before
    full standing/arm blend, use the original controller unmodified.
    """
    if runtime.stand_target is None or runtime.stand_weight!=1. or runtime.arm_weight!=1.:
        return runtime.control(np.zeros(3),default,{s:OPEN for s in ('left','right')},
                               stationary=True,base_control=base)
    if runtime.tick%5==0:
        runtime.arm_target+=np.clip(default-runtime.arm_target,-.005,.005)
        runtime.policy.target[runtime.arm_ids]=runtime.arm_target
        runtime.policy.target[23:25]=runtime.head_target
        for side in runtime.arms:
            runtime.hand_target[side]+=np.clip(OPEN-runtime.hand_target[side],-.012,.012)
    torque=runtime.policy.torques(runtime.d.qpos[runtime.qa],runtime.d.qvel[runtime.va])
    torque[:13]=(runtime.stand_kp*(runtime.stand_target-runtime.d.qpos[runtime.qa[:13]])
        -runtime.stand_kd*runtime.d.qvel[runtime.va[:13]]+runtime.d.qfrc_bias[runtime.va[:13]])
    torque[runtime.arm_ids]+=(runtime.d.qfrc_bias[runtime.va[runtime.arm_ids]]
                               -3*runtime.d.qvel[runtime.va[runtime.arm_ids]])
    limits=runtime.m.jnt_actfrcrange[runtime.m.actuator_trnid[runtime.body_act,0]]
    ctrl=np.array(base,dtype=float,copy=True)
    ctrl[runtime.body_act]=np.clip(torque,limits[:,0],limits[:,1])
    for side,act in runtime.hand_act.items():ctrl[act]=runtime.hand_target[side]
    return ctrl


def supply(model,data,plant,owner,world,initial_tray_x,*,negative=False,post_mode='static'):
    if post_mode not in ('static','policy'):raise ValueError('unknown post-supply mode')
    runtime=H3.H1_runtime(world,model=model,data=data)
    runtime.step_owner=owner
    runtime.background_control=plant.park_control
    default=runtime.policy.default[runtime.arm_ids].copy()
    target_x=float(initial_tray_x) if negative else H2.PLACE_X
    path=ExitPath(runtime,H2.anchor_at(target_x,0.,H2.HANDLE_Y),default,
                  ExitPath.DEFAULT_VARIANT)
    lg,rg,tg=H1.hand_geom_sets(model,runtime.names)
    hg=lg|rg
    bg=[g for g in range(model.ngeom) if (model.geom(g).name or '').startswith(
        ('c_fixed_roller_','c_recv_roller_','c_deck'))]
    robot=[g for g in range(model.ngeom) if (model.body(model.geom_bodyid[g]).name or '').startswith('h_')]
    start=float(data.time);z0=float(data.body('c_payload').xpos[2])
    x0=plant.chassis_x();peak=z0;bilateral=False;trace=[];stage_samples=[];abnormal_total={}
    tray_dof=int(model.jnt_dofadr[model.body_jntadr[model.body('c_payload').id]])
    arms,hands=None,None
    next_progress=0.
    while data.time-start<H2.SEQUENCE_SECONDS:
        age=float(data.time-start)
        # Runtime consumes arm/hand targets only at tick % 5 == 0. Solving
        # full-world IK on the four discarded ticks adds cost, not commands.
        if runtime.tick%5==0:
            arms,hands=H2.stage_driver(runtime,float(initial_tray_x),age,default,OPEN,
                OPEN if negative else GRASP,path,H2.HANDLE_Y,H2.HAND_ROLL_BIAS,
                H2.PHASE_WINDOWS,None,target_x)
        if age>=next_progress:
            print('physical humanoid supply %.2f / %.2f sim seconds' %
                  (age,H2.SEQUENCE_SECONDS),flush=True)
            next_progress+=2.
        runtime.step(np.zeros(3),arms,hands,stationary=age>2)
        if runtime.tick%H2.SAMPLE_INTERVAL_TICKS==0:
            touch=[False,False]
            counts={'left':0,'right':0}
            for c in data.contact[:data.ncon]:
                for a,b in ((c.geom1,c.geom2),(c.geom2,c.geom1)):
                    if a in tg:
                        touch[0]|=b in lg;touch[1]|=b in rg
                        counts['left']+=int(b in lg);counts['right']+=int(b in rg)
            z=float(data.body('c_payload').xpos[2]);peak=max(peak,z)
            bilateral|=all(touch) and z-z0>=.05
            trace.append(dict(age_s=age,z_m=z,bilateral=all(touch),
                              chassis_x_m=plant.chassis_x()))
            feet,abnormal,_=H1.foot_and_collision_instruments(model,data,runtime.names,lg,rg,tg)
            for pair,n in abnormal.items():abnormal_total[pair]=abnormal_total.get(pair,0)+n
            stage_samples.append(dict(t=float(data.time-start),payload_z=z,
                payload_x=float(data.body('c_payload').xpos[0]),
                payload_y=float(data.body('c_payload').xpos[1]),
                payload_speed=float(np.linalg.norm(data.qvel[tray_dof:tray_dof+3])),
                hand_contacts=counts,arm_dev_rad=float(np.max(abs(data.qpos[
                    runtime.qa[runtime.arm_ids]]-default))),feet=feet))
    band_gap,_,hand_gap,_=H2.clearances(model,data,runtime.names,robot,tg,bg,hg)
    position=np.array(data.body('c_payload').xpos)
    checks=dict(bilateral_lift=bilateral,lift_at_least_50mm=peak-z0>=.05,
        source_supported=plant.tray_state()['zone']=='source_band',
        placement_window=abs(position[0]-H2.PLACE_X)<=H2.H2_THRESHOLDS['place_window_x_m']
            and abs(position[1])<=H2.H2_THRESHOLDS['place_window_y_m'],
        hand_clear=hand_gap>=H2.H2_THRESHOLDS['hand_clear_m'],
        robot_clear=band_gap>=H2.H2_THRESHOLDS['band_clear_m'],
        upright=H2.tray_tilt_deg(model,data,model.body('c_payload').id)<=H2.H2_THRESHOLDS['tray_tilt_max_deg'],
        vehicle_parked=abs(plant.chassis_x()-x0)<=H3.H3_THRESHOLDS['chassis_parked_drift_max_m'])
    driven={name for name,(_,end) in H2.PHASE_WINDOWS.items() if data.time-start>=end}
    stages=tray_task.evaluate(stage_samples,driven,z0)
    checks['original_six_stages']=six_stage_passes(stages)
    checks['no_abnormal_contact']=not abnormal_total
    result=dict(scope='RESTRICTED_H2_SUPPLY_WITH_ORIGINAL_SIX_STAGE_CONTACT_REQUALIFICATION',
        checks={k:'PASS' if v else 'FAIL' for k,v in checks.items()},
        status='SUCCEEDED' if all(checks.values()) and not negative else 'FAILED',
        initial_x_m=float(initial_tray_x),final_xyz_judge=position.tolist(),
        peak_lift_m=peak-z0,trace=trace,sim_s=float(data.time),
        original_six_stages=stages,stage_samples=stage_samples,
        abnormal_contacts=[dict(pair=list(pair),samples=n) for pair,n in sorted(abnormal_total.items())],
        feet_note='original read-only pitch/contact instrument; diagnostics, not newly invented safety thresholds',
        assumptions='ideal touch and robot self-state; fixture handles declared at initialization')
    # Keep recomputing the humanoid torque each subsequent shared period. Never
    # cache a torque vector; state/time changes while belts/arm/chassis execute.
    def compose(base):
        if post_mode=='policy':
            # Choose before any skipped neural history; no revival after static.
            ctrl=runtime.control(np.zeros(3),default,{s:OPEN for s in ('left','right')},
                                 stationary=False,stopping=True,base_control=base)
        else:
            ctrl=stationary_control(runtime,default,base)
        runtime.tick+=1
        return ctrl
    return result,compose
