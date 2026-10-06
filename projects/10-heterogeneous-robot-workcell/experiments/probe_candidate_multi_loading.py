"""Independent initialized tray: two red boxes + one blue cylinder physically loaded.

All three free bodies exist BEFORE the first physics step. RGB-D chooses stock;
order counts never feed the reader. No runtime cargo writes/attach/weld/reset.
This is not continuous H handover, transport, fleet or full-order acceptance.
"""
import argparse
import hashlib
import json
import sys
import time
from pathlib import Path

import mujoco
import numpy as np
from PIL import Image

ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/'experiments'),str(ROOT/'src')]
from probe_candidate_arm_loading import pv,rig,arm_bridge,WORLD,CONVENTION,STEPS
from world_owner import WorldOwner
from w4_plant import LogisticsPlant
from workcell.runtime_permit import RuntimePermit
from humanoid_posture_permit import HumanoidPosturePermit
from workcell.tray import derive_cells,cell_centre_world,cell_of
import tray_rgbd_reader as reader
import receiver_rgbd_reader as receiver_reader

SIZES={'red':np.array([.02,.015,.025]),'blue':np.array([.015,.015,.025])}
INVENTORY=('a_payload','p5_red_second','p5_blue_cylinder')
REQUIRED=('initial_observed_empty','three_physical_lifts','three_actual_cells',
          'all_parts_touch_real_tray','final_visual_counts','final_visual_cells',
          'missing_depth_unknown','no_runtime_qpos_write','one_world_writer',
          'all_collision_geometry_in_cells')


def aggregate_acceptance(report, checks, transport_required=False):
    """Requested optional stages are mandatory, including absent evidence."""
    judged={k:('PASS' if checks[k] else 'FAIL') if k in checks else 'NOT_RUN'
            for k in REQUIRED}
    if transport_required:
        judged['continuous_loaded_transport']=report.get('transport_diagnostic_result','NOT_RUN')
    if report.get('nav2_required'):
        judged['continuous_loaded_nav2']=report.get('continuous_nav2_result','NOT_RUN')
    if report.get('handover_required'):
        judged['continuous_humanoid_supply']=report.get('humanoid_supply_result','NOT_RUN')
    if report.get('error'):
        judged['execution_completed']='FAIL'
    return dict(scope=report['scope'],checks=judged,
        diagnostic_result='PASS' if all(v=='PASS' for v in judged.values()) else 'FAIL',
        continuous_handover=report.get('humanoid_supply_result','NOT_RUN'),
        transport=report.get('transport_diagnostic_result','NOT_RUN'),
        full_order='NOT_RUN',v1_complete=False)


def empty_tray_confirmed(observed):
    return (observed.get('status')=='RESOLVED'
            and observed.get('counts')=={'red':0,'blue':0})


def require_logistics_success(report, rows, expected_count):
    """Do not inspect an empty receiver after an earlier movement was refused."""
    if len(rows) != expected_count or any(row['result']['status']!='SUCCEEDED' for row in rows):
        failed=next((row for row in rows if row['result']['status']!='SUCCEEDED'), None)
        report['transport_diagnostic_result']='FAIL'
        report['first_logistics_failure']=failed or dict(reason_code='INCOMPLETE_SEQUENCE')
        raise RuntimeError('LOGISTICS_FAILED_BEFORE_RECEIVER: '+str(
            failed['result'].get('reason_code') if failed else 'INCOMPLETE_SEQUENCE'))


def write_progress(out,report,data,stage):
    """Durable component evidence; a watchdog exit is NEVER task success."""
    directory=out/'checkpoints';directory.mkdir(exist_ok=True)
    number=report.get('checkpoint_count',0)
    path=directory/('%03d.json'%number)
    if path.exists():raise RuntimeError('refuse checkpoint overwrite')
    keys=('scope','world_sha256','source_sha256','handover_required',
          'humanoid_supply_result','initial_count','empty_observation_attempts',
          'phases','live_pcl_stock_planes','transport_rows')
    saved={key:report[key] for key in keys if key in report}
    saved.update(stage=stage,sim_s=float(data.time),observed_wall_s=time.monotonic(),
                 full_order='NOT_RUN',v1_complete=False)
    path.write_text(json.dumps(saved,indent=2)+'\n')
    report['checkpoint_count']=number+1


def stock_observation(model,data,rgb,depth,bench_z,roi):
    """Multi-instance stock recognition, independent of requested quantities."""
    world,valid,_=pv.unproject(model,data,depth,**CONVENTION)
    x0,x1,y0,y1=roi
    inside=valid&(world[...,0]>x0)&(world[...,0]<x1)&(world[...,1]>y0)&(world[...,1]<y1)
    if np.sum(inside&(abs(world[...,2]-bench_z)<reader.FLOOR_BAND_M))<reader.MIN_VISIBLE_FLOOR_PIXELS:
        return dict(status='UNKNOWN',parts=[],reason='STOCK_NOT_OBSERVABLE')
    color=np.asarray(rgb,float)
    parts=[]
    for cls,ch in (('red',0),('blue',2)):
        others=[i for i in range(3) if i!=ch]
        # The stock is RESTING on the declared table. Elevated robot blue
        # surfaces are not stock candidates (same eligibility rule as P3).
        on_table_envelope=(world[...,2]<=bench_z+2*SIZES[cls][2]+pv.ON_BENCH_TOL)
        mask=np.all(color[...,others]<reader.COLOR_DOMINANCE*color[...,ch,None],axis=-1)&inside&on_table_envelope
        dep=np.where(mask,depth,0.)
        cloud,ok,_=pv.unproject(model,data,dep,**CONVENTION)
        comps=pv.segment_parts(cloud,dep,bench_z,ok,SIZES[cls],roi)
        pixels=int(np.sum(ok&(cloud[...,2]>bench_z+.5*float(min(SIZES[cls])))))
        if pixels!=sum(c['n_pixels'] for c in comps):
            return dict(status='UNKNOWN',parts=[],reason='UNRESOLVED_STOCK_FRAGMENT')
        for c in comps:
            err=float(np.max(abs(np.asarray(c['bbox_extent_m'])[:2]-2*SIZES[cls][:2])))
            if c['n_pixels']<pv.MIN_PART_PIXELS or err>reader.EXTENT_TOL_M or not (
                    pv.rests_on_bench(c['z_max'],bench_z,SIZES[cls])):
                return dict(status='UNKNOWN',parts=[],reason='STOCK_COMPONENT_UNRESOLVED',
                            rejected=dict(cls=cls,pixels=c['n_pixels'],size_error_m=err,
                                centroid=c['centroid'],extent=c['bbox_extent_m'],
                                z_min=c['z_min'],z_max=c['z_max']))
            parts.append(dict(cls=cls,centroid=c['centroid'],pixels=c['n_pixels'],size_error_m=err))
    return dict(status='RESOLVED',parts=parts,reason='RGBD_OBSERVED')


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--run-id',required=True)
    ap.add_argument('--transport',action='store_true',
                    help='continue SAME loaded world through legacy physical logistics diagnostic')
    ap.add_argument('--world',choices=['world_p5_candidate_v3.xml','world_p5_candidate_v4.xml',
                                     'world_p5_candidate_v5.xml'],
                    default=WORLD.name)
    ap.add_argument('--transactions',action='store_true',
                    help='gate actual transfers with occupied resources and the P2 custody ledger')
    ap.add_argument('--pcl',action='store_true',help='use already-built native PCL for live bench/receiver planes')
    ap.add_argument('--handover',action='store_true',help='physically supply original home tray with H2 before loading')
    ap.add_argument('--heading-feedback',action='store_true',
                    help='candidate wheel differential heading/continuous parking servo; ideal self-state, NOT Nav2')
    ap.add_argument('--negative-handover',action='store_true',help='open-hand failed supply; downstream must not start')
    ap.add_argument('--park-diagnostic',action='store_true',
                    help='record five seconds of loaded-scene wheel/contact control before logistics; not acceptance')
    ap.add_argument('--policy-balance',action='store_true',
                    help='candidate original zero-speed policy after H exit; not static holding, requires revalidation')
    ap.add_argument('--nav2',action='store_true',help='opt-in same-world actual loaded Nav2 transport candidate')
    ap.add_argument('--retained-motion',action='store_true',help='independent lower-acceleration loaded Nav2 control candidate')
    ap.add_argument('--multi-ccd',action='store_true',help='init-only multi-contact candidate in v5; requires physical revalidation')
    args=ap.parse_args()
    if args.nav2 and not (args.transport and args.heading_feedback and args.world=='world_p5_candidate_v5.xml'):
        ap.error('--nav2 requires --transport --heading-feedback and v5 world')
    if args.retained_motion and not args.nav2:ap.error('--retained-motion requires --nav2')
    if args.multi_ccd and args.world!='world_p5_candidate_v5.xml':ap.error('--multi-ccd requires v5 candidate world')
    if args.transactions and not args.transport:ap.error('--transactions requires --transport')
    if args.negative_handover and not args.handover:ap.error('--negative-handover requires --handover')
    world_path=WORLD.with_name(args.world)
    # Separate declared sampling experiment, NOT a shape-tolerance relaxation.
    # 30mm cylinder at 320x240 had 6.019mm extent deficit (> original 6mm).
    pv.W,pv.H=640,480
    out=ROOT/'reports'/args.run_id
    out.mkdir(parents=True,exist_ok=False)
    report=dict(scope=('CONTINUOUS_HUMANOID_SUPPLY_LOADING_LOGISTICS_DIAGNOSTIC'
                       if args.handover else 'INITIALIZED_TRAY_THREE_PARTS_VISUAL_PHYSICAL_LOADING'),phases=[],
        world=str(world_path.relative_to(ROOT)),
        world_sha256=hashlib.sha256(world_path.read_bytes()).hexdigest(),
        source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        image_resolution=[pv.W,pv.H],
        initial_inventory=list(INVENTORY),runtime_qpos_writes=0,
        handover_required=args.handover,
        nav2_required=args.nav2,
        assumptions=['fixed calibrated camera','known static loading fixture',
                     'ideal authorization evidence','restricted saturated red/blue shapes'])
    checks,view={},None
    try:
        base=mujoco.MjModel.from_xml_path(str(world_path))
        initial=mujoco.MjData(base)
        mujoco.mj_resetDataKeyframe(base,initial,0)
        mujoco.mj_forward(base,initial)
        loading_x=float(initial.geom_xpos[base.geom('c_fixed_roller_1_2').id][0])
        bench_z,roi=pv.bench_geometry(base,initial)
        table=np.array(initial.xpos[base.body('a_table').id])
        spec=mujoco.MjSpec.from_file(str(world_path))
        spec.body('a_payload').geoms[0].rgba=pv.RED_RGBA
        pos={ 'p5_red_second':table+np.array([.12,.04,.035]),
              'p5_blue_cylinder':table+np.array([-.06,.14,.035]) }
        for name,cls in (('p5_red_second','red'),('p5_blue_cylinder','blue')):
            body=spec.worldbody.add_body(name=name,pos=pos[name].tolist())
            body.add_freejoint(name=name+'_free')
            if cls=='red':
                body.add_geom(name=name+'_geom',type=mujoco.mjtGeom.mjGEOM_BOX,
                    size=SIZES[cls].tolist(),mass=.03,friction=[.6,.005,.0001],rgba=pv.RED_RGBA)
            else:
                body.add_geom(name=name+'_geom',type=mujoco.mjtGeom.mjGEOM_CYLINDER,
                    size=[.015,.025,0],mass=.03,friction=[.6,.005,.0001],rgba=pv.BLUE_RGBA)
        aim=[(loading_x+table[0])/2,-.235,bench_z+.02]
        cam_pos=[aim[0],aim[1]-.55,bench_z+1.]
        spec.worldbody.add_camera(name=pv.CAM,pos=cam_pos,
                                 quat=pv.look_at(cam_pos,aim),fovy=45.)
        source_aim=[loading_x-.40,0.,bench_z+.01]
        source_camera=[source_aim[0],-.55,bench_z+1.2]
        spec.worldbody.add_camera(name='source_cam',pos=source_camera,
            quat=pv.look_at(source_camera,source_aim),fovy=70.)
        receiver_geoms=[g for g in range(base.ngeom)
                        if (mujoco.mj_id2name(base,mujoco.mjtObj.mjOBJ_GEOM,g) or '').startswith('c_recv_roller')]
        receiver_x=sorted(float(initial.geom_xpos[g][0]) for g in receiver_geoms)
        receiver_top=max(float(initial.geom_xpos[g][2]+base.geom_size[g][0])
                         for g in receiver_geoms)
        receiver_aim=[receiver_x[0]+.15,0.,receiver_top+.01]
        receiver_cam=[receiver_aim[0],-.55,receiver_aim[2]+1.]
        spec.worldbody.add_camera(name='receiver_cam',pos=receiver_cam,
            quat=pv.look_at(receiver_cam,receiver_aim),fovy=45.)
        receiver_roi=(receiver_x[0]-.035,receiver_x[-1]+.035,-.245,.245)
        if args.nav2:
            from continuous_nav_session import add_camera
            add_camera(spec)
        model=spec.compile()
        if args.multi_ccd:
            from contact_model_candidate import enable_multiccd
            report['contact_candidate']=enable_multiccd(model)
        if args.nav2:
            # Declared optical candidate, not physical geometry/solver changes.
            model.vis.global_.offwidth=640;model.vis.global_.offheight=480
            model.vis.quality.offsamples=0
        data=mujoco.MjData(model)
        mujoco.mj_resetDataKeyframe(model,data,0)
        initial_tray_x=float(data.qpos[pv.free_addr(model,'c_payload')])
        if not args.handover:data.qpos[pv.free_addr(model,'c_payload')]=loading_x
        for name,xyz in pos.items():
            adr=pv.free_addr(model,name)
            data.qpos[adr:adr+3]=xyz
            data.qpos[adr+3:adr+7]=[1,0,0,0]
        mujoco.mj_forward(model,data)
        report['initial_freebody_writes']=2+int(not args.handover)
        fixture=derive_cells(model,data)  # BEFORE FIRST step, declared fixture
        # Catalog rim outline is visible; floor edges are hidden by the walls.
        floor_geom=model.geom('c_tray_floor').id
        rim_geom=model.geom('c_tray_wall_xp').id
        fixture['rim_height_m']=float(model.geom_pos[rim_geom][2]+model.geom_size[rim_geom][2]
            -model.geom_pos[floor_geom][2]-model.geom_size[floor_geom][2])
        plant=LogisticsPlant(model=model,data=data,world=world_path)
        report['calibrated_wheel_signs']=plant._wheel_signs()
        report['heading_feedback_requested']=args.heading_feedback
        if args.heading_feedback:
            import vehicle_heading
            vehicle_heading.install(plant)
            report['heading_feedback_scope']='ideal robot heading; bounded original wheel torque servo; NOT Nav2'
        sequence=[0]
        def observe():
            sequence[0]+=1
            return dict(owner='world',epoch=1,generation=1,sequence=sequence[0],
                observed_wall_s=time.monotonic(),cancel_requested=False,zone_clear=True,
                stop_chain_healthy=True)
        owner=WorldOwner(model,data,HumanoidPosturePermit(observe,model=model,data=data,owner='world',epoch=1,
            generation=1,max_age_s=.5))
        owner.attach(plant)
        def vehicle_diagnostic():
            import vehicle_heading
            yaw,rate=vehicle_heading.measured_heading(plant)
            frame=data.geom_xmat[model.geom('c_deck_frame').id].reshape(3,3)
            return dict(base_yaw_rad=yaw,base_yaw_rate_radps=rate,
                deck_relative_yaw_rad=float(data.qpos[model.jnt_qposadr[model.joint('c_deck_yaw').id]]),
                deck_world_yaw_rad=float(np.arctan2(frame[1,0],frame[0,0])))
        report['vehicle_heading_initial']=vehicle_diagnostic()
        arm_bridge.install('a_')
        ids=rig.ids(model)
        arm_ctrl=[model.actuator('a_actuator%d'%i).id for i in range(1,8)]
        grip_ctrl=model.actuator('a_actuator8').id
        arm=data.qpos[ids['arm_qadr']].copy()
        grip=[rig.GRIPPER_CTRL_OPEN]
        gains=np.asarray(model.actuator_gainprm[arm_ctrl,0])
        if np.any(gains<=0) or not np.allclose(model.actuator_gear[arm_ctrl,0],1.):
            raise RuntimeError('unsupported arm actuator transmission for bias compensation')
        report['arm_bias_compensation']='ideal robot model qfrc_bias / position gain, gear 1'
        def control():
            ctrl=plant.park_control()
            desired=arm+data.qfrc_bias[ids['arm_dof']]/gains
            for i,act in enumerate(arm_ctrl):
                if model.actuator_ctrllimited[act]:
                    desired[i]=np.clip(desired[i],*model.actuator_ctrlrange[act])
            ctrl[arm_ctrl]=desired
            ctrl[grip_ctrl]=grip[0]
            return ctrl
        plan=mujoco.MjData(model)
        rot=rig.home_hand_rot(model,plan)
        def move(point,n):
            q0=data.qpos[ids['arm_qadr']].copy()
            if point is None:
                # CLOSE/RELEASE must keep the last commanded waypoint. Holding
                # the measured (still lagging) joint pose silently abandons it.
                q=arm.copy()
                q0=q.copy()
            else:
                failures=[]
                for seed in (q0,model.key_qpos[0][ids['arm_qadr']]):
                    plan.qpos[:]=data.qpos
                    try:
                        q,pe,_=rig.ik(model,plan,np.array(point),target_rot=rot,q_seed=seed,iters=600)
                        break
                    except RuntimeError as failure:
                        failures.append(str(failure))
                else:
                    raise RuntimeError('IK both current/canonical seeds failed: '+str(failures))
                if failures:
                    report.setdefault('ik_seed_recovery',[]).append(dict(
                        target=np.asarray(point).tolist(),failed_attempts=failures,
                        selected='canonical_home_planning_seed',no_runtime_qpos_write=True))
                if pe>.001:raise RuntimeError('IK did not satisfy original 1mm criterion')
            for k in range(1,n+1):
                arm[:]=q0+(q-q0)*k/n
                owner.step(control)
            if point is not None:
                start=float(data.time)
                # Declared 2s controller reserve: large observation-to-stock
                # moves need measured settling, not a timer-only completion.
                while np.linalg.norm(rig.grasp_point(model,data)-np.asarray(point))>.005:
                    if data.time-start>=2.:
                        report['tracking_timeout']=dict(target=np.asarray(point).tolist(),
                            actual=rig.grasp_point(model,data).tolist(),
                            error_m=float(np.linalg.norm(rig.grasp_point(model,data)-point)),
                            actuator_forces=data.actuator_force[arm_ctrl].tolist())
                        raise RuntimeError('POSE_TRACKING_TIMEOUT: target not reached within 5mm')
                    owner.step(control)
            report.setdefault('tracking_diagnostics',[]).append(dict(
                sim_s=float(data.time),target=None if point is None else np.asarray(point).tolist(),
                grasp_xyz=rig.grasp_point(model,data).tolist(),
                joint_error_rad=float(np.max(abs(data.qpos[ids['arm_qadr']]-q))),
                grasp_error_m=None if point is None else float(np.linalg.norm(
                    rig.grasp_point(model,data)-np.asarray(point)))))
            write_progress(out,report,data,'arm_motion_completed')
        if args.handover:
            from humanoid_supply import supply
            result,humanoid_control=supply(model,data,plant,owner,world_path,
                initial_tray_x,negative=args.negative_handover,
                post_mode='policy' if args.policy_balance else 'static')
            report['humanoid_supply']=result
            report['vehicle_heading_after_supply']=vehicle_diagnostic()
            report['humanoid_supply_result']='PASS' if result['status']=='SUCCEEDED' else 'FAIL'
            (out/'humanoid_supply.json').write_text(json.dumps(result,indent=2)+'\n')
            if result['status']!='SUCCEEDED':
                owner.permit.stop('SUPPLY_UNCONFIRMED','humanoid did not release a supported tray')
                owner.step(control)
                raise RuntimeError('HUMANOID_SUPPLY_FAILED_NO_DOWNSTREAM')
            previous_hold=plant._hold
            plant._hold=lambda:humanoid_control(previous_hold())
        for _ in range(250):owner.step(control)
        # Diagnosed: returning above table centre places the wrist directly in
        # front of the blue cylinder (labeled visibility 283 -> 68 pixels).
        # A declared corner pose clears the optical ray without moving stock.
        base_pos=np.array(model.body('a_link0').pos)
        clear=[base_pos[0]-.10,base_pos[1]-.30,bench_z+.20]
        report['declared_observation_clear_pose']=clear
        move(clear,700)
        view=mujoco.Renderer(model,pv.H,pv.W)
        option=mujoco.MjvOption();option.geomgroup[5]=0
        frame=[0]
        def capture():
            view.disable_depth_rendering()
            view.disable_segmentation_rendering()
            view.update_scene(data,camera=pv.CAM,scene_option=option)
            rgb=view.render().copy()
            view.enable_depth_rendering()
            view.update_scene(data,camera=pv.CAM,scene_option=option)
            depth=view.render().copy()
            # Label rendering is DIAGNOSTIC JUDGE ONLY, never passed to readers.
            view.disable_depth_rendering()
            view.enable_segmentation_rendering()
            view.update_scene(data,camera=pv.CAM,scene_option=option)
            seg=view.render().copy()[:,:,0]
            view.disable_segmentation_rendering()
            name='rgb_%02d.png'%frame[0]
            Image.fromarray(rgb).save(out/name)
            bid=model.body('p5_blue_cylinder').id
            gids=[g for g in range(model.ngeom) if model.geom_bodyid[g]==bid]
            mask=np.isin(seg,gids)
            adr=pv.free_addr(model,'p5_blue_cylinder')
            report.setdefault('camera_diagnostics_judge_only',[]).append(dict(
                frame=frame[0],file=name,sim_s=float(data.time),
                blue_labeled_pixels=int(np.sum(mask)),
                blue_true_qpos=data.qpos[adr:adr+7].tolist()))
            frame[0]+=1
            return rgb,depth
        if args.handover:
            from source_staging import stage_to_loading
            previous_cam=pv.CAM
            try:
                pv.CAM='source_cam'
                def observe_source():
                    rgb,depth=capture()
                    cloud,valid,_=pv.unproject(model,data,depth,**CONVENTION)
                    return receiver_reader.locate(fixture,cloud,rgb,valid,
                        (loading_x-1.15,loading_x+.40,-.34,.34),bench_z+.01)
                positioned=stage_to_loading(plant,owner,observe_source,loading_x)
                report['source_positioning']={k:v for k,v in positioned.items() if k!='cells'}
                report['vehicle_heading_after_source_positioning']=vehicle_diagnostic()
                if positioned['status']!='RESOLVED':raise RuntimeError('LOADING_TRAY_POSITION_UNKNOWN')
                fixture=positioned['cells']
            finally:pv.CAM=previous_cam
        rgb,depth=capture()
        report['initial_count']=reader.read(model,data,rgb,depth,fixture,SIZES,CONVENTION)
        report['empty_observation_attempts']=[report['initial_count']]
        # Actual retreat/reobserve, not suppressing occluder pixels or relaxing
        # the content-height gate. Two declared candidates, then fail closed.
        observation_poses=([base_pos[0]-.15,base_pos[1]-.35,bench_z+.25],
                           [base_pos[0]-.20,base_pos[1]-.40,bench_z+.25])
        for candidate in observation_poses:
            if empty_tray_confirmed(report['initial_count']):break
            move(candidate,700)
            clear=list(candidate)
            rgb,depth=capture()
            report['initial_count']=reader.read(model,data,rgb,depth,fixture,SIZES,CONVENTION)
            report['empty_observation_attempts'].append(report['initial_count'])
        checks['initial_observed_empty']=empty_tray_confirmed(report['initial_count'])
        write_progress(out,report,data,'empty_tray_observed')
        if not checks['initial_observed_empty']:
            owner.permit.stop('EMPTY_TRAY_UNKNOWN','initial empty tray not independently confirmed')
            owner.step(control)
            raise RuntimeError('EMPTY_TRAY_UNCONFIRMED_NO_LOADING')
        lifts=[]
        for index,cls in enumerate(('red','red','blue')):
            rgb,depth=capture()
            measured_bench_z=bench_z
            if args.pcl:
                import pcl_plane_reader
                cloud,valid,_=pv.unproject(model,data,depth,**CONVENTION)
                plane=pcl_plane_reader.estimate(cloud,valid,roi,bench_z)
                report.setdefault('live_pcl_stock_planes',[]).append(plane)
                if plane['status']!='RESOLVED':raise RuntimeError('STOCK_PCL_PLANE_UNKNOWN')
                measured_bench_z=plane['floor_z_m']
            stock=stock_observation(model,data,rgb,depth,measured_bench_z,roi)
            report.setdefault('stock_observations',[]).append(stock)
            candidates=[p for p in stock['parts'] if p['cls']==cls]
            if stock['status']!='RESOLVED' or not candidates:
                raise RuntimeError('STOCK_UNKNOWN_OR_UNAVAILABLE: '+stock['reason'])
            # Commodity selection rule, NOT truth/name matching. Multiple red
            # candidates are valid inventory, selected by observed x then y.
            selected=min(candidates,key=lambda p:(p['centroid'][0],p['centroid'][1]))
            pick=np.array([*selected['centroid'][:2],measured_bench_z+SIZES[cls][2]])
            place=np.array(cell_centre_world(fixture,index,z=fixture['deck']['top_z']+SIZES[cls][2]))
            for phase,point,grip_value in (
                ('approach',pick+[0,0,rig.PREGRASP_CLEARANCE],rig.GRIPPER_CTRL_OPEN),
                ('descend',pick,None),('close',None,rig.GRIPPER_CTRL_CLOSED),
                ('lift',pick+[0,0,rig.LIFT_HEIGHT],None),
                ('carry',place+[0,0,rig.PREGRASP_CLEARANCE],None),
                ('lower',place,None),('release',None,rig.GRIPPER_CTRL_OPEN),
                ('retreat',place+[0,0,rig.RETREAT_HEIGHT],None),
                ('clear_view',clear,None)):
                if grip_value is not None:grip[0]=grip_value
                move(point,STEPS[phase])
                if phase=='close':
                    # Explicit IDEAL tactile assumption, not visual geom labels:
                    # both finger pads must contact the same free stock body.
                    contact_sets=[]
                    for finger in ('a_left_finger','a_right_finger'):
                        fb=model.body(finger).id
                        touched=set()
                        for contact in data.contact[:data.ncon]:
                            pair=[int(model.geom_bodyid[contact.geom1]),int(model.geom_bodyid[contact.geom2])]
                            if fb in pair:
                                other=pair[1] if pair[0]==fb else pair[0]
                                if other in [model.body(name).id for name in INVENTORY]:
                                    touched.add(other)
                        contact_sets.append(touched)
                    confirmed=bool(contact_sets[0]&contact_sets[1])
                    report.setdefault('ideal_tactile_close',[]).append(dict(item=index,
                        bilateral_common_stock_contact=confirmed))
                    if not confirmed:
                        owner.permit.stop('GRASP_CONTACT_UNCONFIRMED','bilateral stock touch absent')
                        owner.step(control)  # explicitly latch SIM freeze; no further movement
                row=dict(item=index,cls=cls,phase=phase,sim_s=float(data.time),
                         truth_for_judge={name:data.xpos[model.body(name).id].tolist()
                                          for name in INVENTORY})
                report['phases'].append(row)
                if phase=='lift':
                    lifts.append(any(x[2]>bench_z+SIZES[cls][2]+.05
                                     for name,x in row['truth_for_judge'].items()
                                     if (name=='p5_blue_cylinder')==(cls=='blue')))
            print('completed physical item '+str(index)+' '+cls,flush=True)
        final_cells=derive_cells(model,data)  # independent evaluator only
        final={name:dict(xyz=data.xpos[model.body(name).id].tolist(),
            cell=cell_of(final_cells,data.xpos[model.body(name).id])) for name in INVENTORY}
        tray=model.body('c_payload').id
        for name in INVENTORY:
            bid=model.body(name).id
            final[name]['tray_contact']=any({int(model.geom_bodyid[c.geom1]),int(
                model.geom_bodyid[c.geom2])}=={bid,tray} for c in data.contact[:data.ncon])
        report['final_truth_for_judge']=final
        report['vehicle_heading_after_loading']=vehicle_diagnostic()
        if args.park_diagnostic:
            # Preserve the exact scene as diagnostic artifacts, not inputs to task execution.
            mujoco.mj_saveModel(model,str(out/'loading_model.mjb'),None)
            flag=mujoco.mjtState.mjSTATE_INTEGRATION
            state=np.empty(mujoco.mj_stateSize(model,flag))
            mujoco.mj_getState(model,data,state,flag)
            np.savez(out/'loading_state.npz',state=state,flag=int(flag),arm_target=arm,
                     gripper_target=grip[0])
            diagnostic=[];begin=float(data.time);next_sample=begin
            while data.time-begin<5.:
                owner.step(control)
                if data.time>=next_sample:
                    cross=[]
                    for c in data.contact[:data.ncon]:
                        names=[model.geom(g).name or '' for g in (c.geom1,c.geom2)]
                        moving=[n.startswith(('n_','c_deck','c_pusher')) for n in names]
                        if any(moving) and not all(moving):
                            cross.append(dict(geoms=names,penetration_m=float(c.dist)))
                    diagnostic.append(dict(sim_s=float(data.time),heading=vehicle_diagnostic(),
                        wheel_controls={s:float(data.ctrl[a]) for s,a in plant.wheel_actuators.items()},
                        wheel_forces={s:float(data.actuator_force[a]) for s,a in plant.wheel_actuators.items()},
                        wheel_rates={s:float(data.qvel[model.jnt_dofadr[model.joint(
                            'n_wheel_'+s+'_joint').id]]) for s in ('left','right')},
                        cross_contacts=cross))
                    next_sample+=.5
            report['loaded_park_diagnostic']=diagnostic
            (out/'loaded_park_diagnostic.json').write_text(json.dumps(diagnostic,indent=2)+'\n')
        checks['three_physical_lifts']=len(lifts)==3 and all(lifts)
        checks['three_actual_cells']=sorted(v['cell'] for v in final.values()
            if v['cell'] is not None)==[0,1,2]
        checks['all_parts_touch_real_tray']=all(v['tray_contact'] for v in final.values())
        import cargo_footprint_judge
        report['loading_cargo_footprint_judge']=cargo_footprint_judge.judge(model,data,INVENTORY)
        checks['all_collision_geometry_in_cells']=report['loading_cargo_footprint_judge']['status']=='PASS'
        rgb,depth=capture()
        observed=reader.read(model,data,rgb,depth,fixture,SIZES,CONVENTION)
        report['final_visual_count']=observed
        checks['final_visual_counts']=observed['status']=='RESOLVED' and observed['counts']=={'red':2,'blue':1}
        checks['final_visual_cells']=sorted(p['cell'] for p in observed['parts'])==[0,1,2]
        missing=reader.read(model,data,rgb,np.zeros_like(depth),fixture,SIZES,CONVENTION)
        checks['missing_depth_unknown']=missing['status']=='UNKNOWN' and missing['counts'] is None
        checks['no_runtime_qpos_write']=report['runtime_qpos_writes']==0
        checks['one_world_writer']=owner.steps>0 and plant.step_owner is owner
        write_progress(out,report,data,'loading_judged')
        if args.transport:
            if not all(checks.get(key,False) for key in REQUIRED):
                raise RuntimeError('REFUSED_TRANSPORT: loading evidence incomplete')
            from workcell.adapters.sim import SkillAdapter
            import probe_w5_loop as loop
            # Persist the arm's approved hold target while the band/chassis run.
            # No arm/body reset and no second model/data after actual loading.
            original_hold=plant._hold
            def composed_hold():
                ctrl=original_hold()
                desired=arm+data.qfrc_bias[ids['arm_dof']]/gains
                for i,act in enumerate(arm_ctrl):
                    if model.actuator_ctrllimited[act]:
                        desired[i]=np.clip(desired[i],*model.actuator_ctrlrange[act])
                ctrl[arm_ctrl]=desired
                ctrl[grip_ctrl]=grip[0]
                return ctrl
            plant._hold=composed_hold
            navigator=None
            if args.nav2:
                from continuous_nav_session import ContinuousNavSession
                navigator=ContinuousNavSession(plant=plant,owner=owner,template=fixture,
                    inventory=INVENTORY,out=out,run_id=args.run_id,retained_motion=args.retained_motion)
            adapter=SkillAdapter(plant=plant,boot_id='loaded-diagnostic',stations=plant.stations,
                envelope=loop.ENVELOPE,transfers=loop.TRANSFERS,
                navigation_move=navigator.move if navigator else None)
            rows=[]
            transaction=None
            if args.transactions:
                from workcell.resources import ResourceTable
                from workcell.transfer import TransferLedger
                from workcell.physical_transfer_session import PhysicalTransferSession
                from workcell.orchestration.sequence import SkillSequence
                from workcell.safety import CommandGate
                order='loaded-diagnostic';epoch=1
                resources=ResourceTable(epoch=epoch,resources=[loop.SOURCE_RES,loop.DECK_RES,loop.RECEIVER_RES])
                ledger=TransferLedger(epoch=epoch)
                obs_dir=out/'obs';obs_dir.mkdir()
                journal_rows=[]
                def evidence(name,measurement):
                    ref='obs/'+name+'.json'
                    (out/ref).write_text(json.dumps(measurement,indent=2)+'\n')
                    return [ref]
                def journal(event):
                    event['sim_s']=float(data.time);event['wall_s']=time.monotonic()
                    journal_rows.append(event)
                    report['custody_transactions']=journal_rows
                    (out/'transactions.json').write_text(json.dumps(journal_rows,indent=2)+'\n')
                transaction=PhysicalTransferSession(ledger=ledger,resources=resources,journal=journal)
                state=plant.tray_state()
                if state['zone']!='source_band' or not state['on_crowns']:
                    raise RuntimeError('SOURCE_OCCUPANCY_UNKNOWN')
                references=evidence('initial_occupancy',dict(tray=state,
                    assumption='one declared physical tray; ideal solver contact occupancy sensor'))
                resources.initialise_occupied(loop.SOURCE_RES,owner=order,ttl_s=900.,
                                               now_s=time.monotonic(),evidence=references)
                for name,zone in ((loop.DECK_RES,'deck'),(loop.RECEIVER_RES,'receiver_band')):
                    if zone in plant.tray_support():raise RuntimeError('RECEIVER_NOT_EMPTY')
                    resources.mark_cleared(name,evidence=references,now_s=time.monotonic())
                skill_sequence=SkillSequence(adapter=adapter,gate=CommandGate(ttl_s=.5,silence_s=.5,
                    v_max=.6,w_max=1.2,source='safety_gate'),boot_id='loaded-diagnostic',
                    epoch=epoch,order_id=order,wall_clock=time.monotonic)
                contract=json.loads((ROOT/'config/docking_contract.json').read_text())
                def execute_request(request):
                    outcome=skill_sequence.submit(request,now_s=time.monotonic())
                    record=outcome['record']
                    return record.result or dict(status='FAILED',reason_code=record.refusal)
                def clear_source(transfer_id):
                    record=ledger.by_id[transfer_id]
                    zone=loop.TRANSFERS[transfer_id]['source_zone']
                    observed=plant.tray_state()
                    if zone in plant.tray_support():raise RuntimeError('SOURCE_CLEARANCE_UNKNOWN')
                    refs=evidence(transfer_id+'_source_clear',dict(tray=observed,supported_by=plant.tray_support()))
                    token=resources.snapshot(record.source)
                    resources.release(record.source,owner=transfer_id,generation=token['generation'],
                                      epoch=epoch,now_s=time.monotonic(),to_state='CLEARING')
                    resources.mark_cleared(record.source,evidence=refs,now_s=time.monotonic())
                    return refs
                def transfer_with_transaction(request):
                    tid=request['arguments']['transfer_id'];leg=loop.TRANSFERS[tid]
                    station='station_c' if leg['direction']=='onto_deck' else 'station_b'
                    residual=plant.dock_residuals(station);state=plant.tray_state()
                    dock_result=rows[-1]['result']['final_state']
                    token=resources.snapshot(leg['source'])
                    # Own live source cargo, empty receiver; no false FREE source.
                    prefix='c_fixed_roller' if station=='station_c' else 'c_recv_roller'
                    band_z=max(float(data.geom_xpos[g][2]+model.geom_size[g][0])
                        for g in range(model.ngeom) if (model.geom(g).name or '').startswith(prefix))
                    deck_z=max(float(data.geom_xpos[g][2]+model.geom_size[g][0])
                        for g in range(model.ngeom) if (model.geom(g).name or '').startswith('c_deck_roller'))
                    pre=dict(source_capacity_free=token['state']=='OCCUPIED',
                        receiver_capacity_free=resources.snapshot(leg['destination'])['state']=='FREE',
                        height_within_tolerance=abs(deck_z-band_z)<=contract['tolerance']['height_mismatch_m'],
                        lateral_within_tolerance=abs(residual['lateral_m'])<=loop.ENVELOPE['dock_lateral_m'],
                        yaw_within_tolerance=abs(residual['yaw_rad'])<=loop.ENVELOPE['dock_yaw_rad'],
                        clearance_ok=abs(residual['longitudinal_m'])<=loop.ENVELOPE['dock_longitudinal_m'],
                        amr_at_rest=bool(dock_result.get('stopped_confirmed')),
                        source_has_tray=state['zone']==leg['source_zone'] and state['on_crowns'],
                        receiver_empty=leg['destination_zone'] not in plant.tray_support(),
                        arms_retracted=bool(np.linalg.norm(rig.grasp_point(model,data)-clear)<=.005),
                        stop_chain_healthy=bool(owner.permit.check()['allowed']),
                        evidence_age_s=0.,max_evidence_age_s=contract['validity']['max_observation_age_s'],ttl_s=900.)
                    launch=evidence(tid+'_launch',dict(preconditions=pre,residual=residual,
                        sensor_assumptions='ideal interface height/body state, touch, wrist park and healthy external stop chain',
                        observed_wall_s=time.monotonic(),sim_s=float(data.time)))
                    previous='xfer_source_to_deck' if tid=='xfer_deck_to_receiver' else None
                    if previous:clear_source(previous)
                    grant=dict(owner=token['owner'],generation=token['generation'],epoch=epoch,evidence=launch)
                    def drive():
                        if previous:
                            ledger.advance(previous,evidence=launch)
                            journal(dict(event='previous_leg_released_after_occupied_handover',
                                         transaction=ledger.by_id[previous].to_dict()))
                        return execute_request(request)
                    def confirm(result):
                        stopped=plant._stop_transfer()
                        actual=plant.tray_state();support=plant.tray_support()
                        refs=evidence(tid+'_confirmation',dict(stop=stopped,tray=actual,support=support))
                        return dict(receiver_supported=leg['destination_zone'] in support and actual['on_crowns'],
                            source_cleared=leg['source_zone'] not in support,
                            stopped_confirmed=stopped['stopped_confirmed'],evidence=refs)
                    return transaction.execute(dict(transfer_id=tid,order_id=order,epoch=epoch,
                        source=leg['source'],receiver=leg['destination'],tray_id='c_payload',preconditions=pre),
                        now_s=time.monotonic(),launch_evidence=launch,source_grant=grant,drive=drive,confirm=confirm)
            for i,(skill,arguments) in enumerate(loop.SCRIPT):
                arguments=dict(arguments)
                if skill=='VERIFY_DELIVERY':arguments['order_id']='loaded-diagnostic'
                request=dict(schema_version=1,command_id='loaded-%03d'%i,
                    order_id='loaded-diagnostic',revision=i+1,expected_boot_id='loaded-diagnostic',
                    skill=skill,arguments=dict(arguments),resource_generation=0,
                    deadline_s=660. if args.nav2 and skill=='MOVE_TO_STATION' and arguments.get('station_id')=='station_b' else 120.)
                answer=(transfer_with_transaction(request) if transaction and skill=='START_TRANSFER'
                        else execute_request(request) if transaction else adapter.execute(request,now_s=time.monotonic()))
                rows.append(dict(skill=skill,result=answer))
                if navigator and skill=='MOVE_TO_STATION' and arguments.get('station_id')=='station_b':
                    report['continuous_nav2_result']='PASS' if answer['status']=='SUCCEEDED' else 'FAIL'
                report['transport_rows']=rows
                write_progress(out,report,data,'logistics_skill_'+skill)
                print(json.dumps(dict(skill=skill,status=answer['status'],
                    reason=answer.get('reason_code'),sim_s=float(data.time))),flush=True)
                if answer['status']!='SUCCEEDED':break
            require_logistics_success(report,rows,len(loop.SCRIPT))
            report['transport_scope']=dict(same_world=True,same_timeline=True,
                no_reload=True,custody_transactions='RUNNING' if transaction else 'NOT_RUN',
                nav2=report.get('continuous_nav2_result','NOT_RUN'),
                chassis_control=('Nav2 loaded approach; ideal fixture/self-state precision docking' if args.nav2
                                 else 'legacy ideal body-state diagnostic; NOT Nav2'),
                receiver_visual_count='PENDING')
            delivered_cells=derive_cells(model,data)  # independent judge only
            report['transport_final_truth']={name:dict(
                xyz=data.xpos[model.body(name).id].tolist(),
                cell=cell_of(delivered_cells,data.xpos[model.body(name).id])) for name in INVENTORY}
            report['transport_diagnostic_result']='PASS' if (
                len(rows)==len(loop.SCRIPT) and all(row['result']['status']=='SUCCEEDED' for row in rows)
                and sorted(p['cell'] for p in report['transport_final_truth'].values()
                    if p['cell'] is not None)==[0,1,2]) else 'FAIL'
            # Final commanded park is held, not reset; measure actual stop first.
            brake=plant._stop_transfer()
            report['receiver_stop_window']=brake
            tray_id=model.body('c_payload').id
            report['receiver_cargo_contact']={name:any(
                {int(model.geom_bodyid[c.geom1]),int(model.geom_bodyid[c.geom2])}==
                {model.body(name).id,tray_id} for c in data.contact[:data.ncon])
                for name in INVENTORY}
            # Fixed receiver camera uses only RGB-D and catalog calibration.
            previous_cam=pv.CAM
            try:
                pv.CAM='receiver_cam'
                rgb,depth=capture()
                cloud,valid,_=pv.unproject(model,data,depth,**CONVENTION)
                np.savez_compressed(out/'receiver_rgbd.npz',world=cloud,rgb=rgb,
                    depth=depth,valid=valid,roi=np.asarray(receiver_roi),floor_z=receiver_top+.01)
                receiver_floor_z=receiver_top+.01
                if args.pcl:
                    import pcl_plane_reader
                    plane=pcl_plane_reader.estimate(cloud,valid,receiver_roi,receiver_floor_z,
                                                   rgb=rgb,cyan_only=True)
                    report['live_pcl_receiver_plane']=plane
                    if plane['status']!='RESOLVED':raise RuntimeError('RECEIVER_PCL_PLANE_UNKNOWN')
                    receiver_floor_z=plane['floor_z_m']
                localized=receiver_reader.locate(fixture,cloud,rgb,valid,
                                                receiver_roi,receiver_floor_z)
                report['receiver_tray_localization']={k:v for k,v in localized.items() if k!='cells'}
                observed=(reader.read(model,data,rgb,depth,localized['cells'],SIZES,CONVENTION)
                          if localized['status']=='RESOLVED' else dict(
                              status='UNKNOWN',counts=None,parts=[],reason=localized['reason']))
                report['receiver_visual_count']=observed
                report['transport_scope']['receiver_visual_count']=observed['status']
                missing=receiver_reader.locate(fixture,cloud,rgb,np.zeros_like(valid),
                                               receiver_roi,receiver_top+.01)
                report['receiver_missing_depth_unknown']=missing['status']=='UNKNOWN'
            finally:
                pv.CAM=previous_cam
            extra=dict(receiver_stopped=brake['stopped_confirmed'],
                cargo_supported=all(report['receiver_cargo_contact'].values()),
                receiver_visual_count=observed['status']=='RESOLVED' and observed['counts']=={'red':2,'blue':1},
                receiver_visual_cells=sorted(p['cell'] for p in observed['parts'])==[0,1,2],
                missing_receiver_depth_unknown=report['receiver_missing_depth_unknown'])
            report['receiver_cargo_footprint_judge']=cargo_footprint_judge.judge(model,data,INVENTORY)
            extra['receiver_cargo_contained']=report['receiver_cargo_footprint_judge']['status']=='PASS'
            report['transport_required_checks']={k:'PASS' if v else 'FAIL' for k,v in extra.items()}
            if transaction:
                tid='xfer_deck_to_receiver'
                if all(extra.values()) and ledger.by_id[tid].stage=='COMMITTED':
                    refs=clear_source(tid)
                    current=resources.snapshot(loop.RECEIVER_RES)
                    resources.handover_occupied(loop.RECEIVER_RES,owner=tid,new_owner=order,
                        generation=current['generation'],epoch=epoch,ttl_s=900.,
                        now_s=time.monotonic(),evidence=refs+evidence('delivery_count',observed))
                    ledger.advance(tid,evidence=refs)
                    journal(dict(event='delivery_committed_buffer_remains_occupied',
                                 transaction=ledger.by_id[tid].to_dict()))
                report['transaction_final_records']={tid:rec.to_dict() for tid,rec in ledger.by_id.items()}
                report['resource_final_state']=resources.table(now_s=time.monotonic())
                extra['custody_transactions']=(len(ledger.by_id)==2 and all(
                    rec.stage=='RELEASED' for rec in ledger.by_id.values()) and
                    resources.snapshot(loop.RECEIVER_RES)['state']=='OCCUPIED' and
                    resources.snapshot(loop.RECEIVER_RES)['owner']==order)
                report['transport_required_checks']['custody_transactions']='PASS' if extra['custody_transactions'] else 'FAIL'
                report['transport_scope']['custody_transactions']=report['transport_required_checks']['custody_transactions']
            if not all(extra.values()):report['transport_diagnostic_result']='FAIL'
    except Exception as error:
        report['error']=type(error).__name__+': '+str(error)
        if args.nav2:report.setdefault('continuous_nav2_result','FAIL')
        if 'resources' in locals():
            report['resource_final_state']=resources.table(now_s=time.monotonic())
            report['transaction_final_records']={tid:rec.to_dict() for tid,rec in ledger.by_id.items()}
        if 'owner' in locals():report['writer_frozen']=owner.frozen
    finally:
        if view is not None:view.close()
    acceptance=aggregate_acceptance(report,checks,args.transport)
    (out/'report.json').write_text(json.dumps(report,indent=2)+'\n')
    (out/'acceptance.json').write_text(json.dumps(acceptance,indent=2)+'\n')
    print(json.dumps(acceptance,indent=2))
    return 0 if acceptance['diagnostic_result']=='PASS' else 1


if __name__=='__main__':raise SystemExit(main())
