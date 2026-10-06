"""Opt-in Nav2 guest on an already loaded world; no initialization/physics writer.

The source station is a declared localization prior, independently checked before
starting ROS. Camera calibration/catalog support height and ideal deck contact
are simulation assumptions. Truth logs judge arrival/retention/stop only.
This candidate does NOT provide general 3D swept collision protection or SLAM.
"""
import json
from pathlib import Path
import time
import mujoco
import numpy as np
import yaml

from joint_world_nav_io import JointWorldNavIO
from load_motion_gate import LoadMotionGate
from transport_posture_gate import TransportPostureGate
from state_publish_schedule import StatePublishSchedule
from make_joint_nav_profile import posture_check
from probe_n_nav import Processes
from probe_vehicle_rgbd import CAMERA, CAMERA_QUAT, SIZE
from probe_vehicle_nav2 import json_value, exception_chain
from vehicle_rgbd_reader import observe
from wheel_authority import grant_navigation, return_to_parking
from w4_plant import collision_radius
import probe_p3_vision as pv

ROOT = Path(__file__).resolve().parents[1]


def add_camera(spec):
    """Before compilation only. No cargo/geometry/control modifications."""
    spec.body('n_base_link').add_camera(name='vehicle_load_cam',
        pos=CAMERA['position_in_base'], quat=CAMERA_QUAT, fovy=CAMERA['fovy_deg'])


class ContinuousNavSession:
    def __init__(self, *, plant, owner, template, inventory, out, run_id, retained_motion=False):
        self.plant, self.owner = plant, owner
        self.model, self.data = plant.model, plant.data
        self.template, self.inventory = template, tuple(inventory)
        self.out, self.run_id = Path(out), run_id
        self.retained_motion=retained_motion
        self.cfg = yaml.safe_load((ROOT/'config/joint_world_v5.yaml').read_text())
        self.profile = json.loads((ROOT/'config/joint_world_v5.profile.json').read_text())
        self.contract = json.loads((ROOT/'config/joint_world_v5_centered.profile.json').read_text())['observation_contract']
        self.used = False

    def move(self, station):
        # The source has a zero approach leg. Never launch a loaded navigator
        # there or substitute this callback for precision docking.
        if station['id'] == 'station_c':
            return self.plant.drive_to(station['approach_x_m'],
                speed=station['approach_speed_mps'], timeout_s=station['approach_timeout_s'])
        if self.used or station['id'] != 'station_b':
            raise RuntimeError('CONTINUOUS_NAV_STATION_NOT_COMMISSIONED')
        self.used = True
        return self._loaded_move(station)

    def _loaded_move(self, station):
        plant, model, data, owner = self.plant, self.model, self.data, self.owner
        goal = [float(station['approach_x_m']), 0., 0.]
        base = model.body('n_base_link').id
        # An independent preflight validates the DECLARED source prior; runtime
        # pose is not sent to AMCL, the camera reader, or the goal planner.
        initial_xy = data.xpos[base,:2].copy()
        # Loading ends at the SOURCE DOCK declaration, not the original spawn.
        # Passive source support made those declared poses differ by 10mm.
        source_prior=[float(plant.stations['station_c']['dock_x_m']),0.,0.]
        heading = float(np.arctan2(data.xmat[base].reshape(3,3)[1,0],data.xmat[base].reshape(3,3)[0,0]))
        token = None
        owned = Processes(self.out); io = None; view = None
        latest = {}; frames = []; truth = []; worker = None
        gate = LoadMotionGate(self.contract)
        posture_gate = TransportPostureGate()
        started = time.monotonic(); begin = float(data.time); stop_at = None
        result = dict(scope='CONTINUOUS_ACTUAL_LOAD_NAV2_GUEST', status='RUNNING', goal=goal,
            retained_motion_candidate=self.retained_motion,
            declared_source_prior=source_prior,
            runtime_qpos_writes=0, same_world=True, full_order='NOT_RUN', v1_complete=False,
            assumptions=['declared source prior, verified while parked; wheel odometry thereafter',
                         'virtual calibrated RGBD; ideal deck contact; static map prior NOT SLAM',
                         'no general 3D swept-volume guard; precision dock remains ideal fixture sensing'])
        def save():
            result.update(truth_judge_only=truth, sensor_frames=frames, latest_sensor=latest,
                nav2_worker=worker, motion_refusal=gate.latched_reason, sim_s=float(data.time),
                posture_motion_refusal=posture_gate.latched_reason,
                wall_s=time.monotonic()-started)
            tmp=self.out/'continuous_nav.tmp'
            tmp.write_text(json.dumps(result,indent=2,default=json_value)+'\n')
            tmp.replace(self.out/'continuous_nav.json')
        def sensor():
            stamp=time.monotonic()
            for depth_mode in (False, True):
                if depth_mode:view.enable_depth_rendering()
                else:view.disable_depth_rendering()
                view.update_scene(data,camera='vehicle_load_cam')
                view.scene.flags[mujoco.mjtRndFlag.mjRND_SHADOW]=False
                view.scene.flags[mujoco.mjtRndFlag.mjRND_REFLECTION]=False
                if depth_mode:depth=view.render().copy()
                else:rgb=view.render().copy()
            # Pure reader has no simulator inputs. Restore unrelated fixed
            # camera globals even on failure; their 640x480 reader remains valid.
            old=(pv.W,pv.H)
            try:
                pv.W,pv.H=CAMERA['width'],CAMERA['height']
                observed=observe(rgb,depth,CAMERA,self.template,SIZE,
                    roi=(1.7,2.7,-.4,.4),floor_z=.78,
                    observed_sim_s=float(data.time),observed_wall_s=stamp)
            finally:pv.W,pv.H=old
            observed['count_verified']=observed.get('counts')=={'red':2,'blue':1}
            observed['support']='deck' if plant.tray_support()==['deck'] else 'UNKNOWN'
            frames.append(dict(sim_s=float(data.time),processing_wall_s=time.monotonic()-stamp,
                               observation=observed))
            return observed
        def measure():
            row=dict(sim_s=float(data.time),xyz=data.xpos[base].copy(),
                velocity=[float(data.qvel[model.jnt_dofadr[model.joint(n).id]])
                          for n in ('n_slide_x','n_slide_y','n_slide_z','n_yaw')],cargo={})
            for name in ('c_payload',)+self.inventory:
                bid=model.body(name).id
                speed=np.empty(6)
                mujoco.mj_objectVelocity(model,data,mujoco.mjtObj.mjOBJ_BODY,bid,speed,0)
                row['cargo'][name]=dict(xyz=data.xpos[bid].copy(),
                    point_speed_bound_mps=float(np.linalg.norm(speed[3:])+collision_radius(model,bid)*np.linalg.norm(speed[:3])),
                    relative_to_base=(data.xpos[bid]-data.xpos[base]) @ data.xmat[base].reshape(3,3),
                    relative_to_deck=(data.xpos[bid]-data.body('c_deck').xpos) @ data.body('c_deck').xmat.reshape(3,3))
            truth.append(row)
        try:
            if (not np.all(np.isfinite(initial_xy)) or not np.isfinite(heading)
                    or np.linalg.norm(initial_xy-np.asarray(source_prior[:2])) > .005 or abs(heading) > .01):
                raise RuntimeError('DECLARED_SOURCE_LOCALIZATION_PRIOR_UNCONFIRMED')
            if plant.tray_support() != ['deck']:
                raise RuntimeError('CONTINUOUS_LOAD_DECK_SUPPORT_UNCONFIRMED')
            token=grant_navigation(plant)
            io=JointWorldNavIO(plant,self.cfg)
            view=mujoco.Renderer(model,height=CAMERA['height'],width=CAMERA['width'])
            for name, script, flags in (
                ('bridge','joint_world_ros_bridge.py','--config config/joint_world_v5.yaml --command-source cmd_vel --duration '+str(begin+130)+' --wait-for-state 30'),
                ('worker','joint_nav_worker.py','--loaded --centered-load --wall-budget 600 --goal '+
                    ' '.join(map(str,goal))+' --start '+' '.join(map(str,source_prior))+
                    (' --retained-motion' if self.retained_motion else ''))):
                owned.spawn(name,['bash','-lc','source scripts/env.sh\nexec /usr/bin/python3 experiments/'+script+' --run-id '+self.run_id+' '+flags])
            next_frame=begin;next_sample=begin;next_save=begin
            publish_schedule=StatePublishSchedule(sim_period_s=1/float(self.cfg['sim']['state_rate_hz']))
            worker_path=self.out/'nav2_worker_report.json'
            def control():
                nonlocal stop_at
                posture=posture_check(model,data)
                posture_allowed=posture_gate.check(posture)
                if not posture_allowed:
                    result['control_failure_posture']=posture_gate.first_failure
                    if stop_at is None:stop_at=float(data.time)
                def allowed(requested):
                    authorization=gate.check(latest,now_sim_s=float(data.time),
                        now_wall_s=time.monotonic(),motion_requested=requested)
                    return posture_allowed and stop_at is None and authorization['motion_allowed']
                # Includes EXISTING post-supply humanoid and approved arm hold;
                # no new policy, controller reset, or second final writer.
                return io.control(allowed)
            while data.time-begin < 120:
                if time.monotonic()-started > 660:raise TimeoutError('continuous Nav2 wall budget')
                if owned.handles['bridge'].poll() is not None:raise RuntimeError('bridge exited')
                if data.time>=next_frame or (latest and time.monotonic()-latest['observed_wall_s']>=.25):
                    latest=sensor();next_frame=float(data.time)+.2
                if worker is None and worker_path.exists():
                    worker=json.loads(worker_path.read_text());stop_at=float(data.time)
                if gate.latched_reason and stop_at is None:stop_at=float(data.time)
                if owned.handles['worker'].poll() is not None and worker is None:
                    raise RuntimeError('worker exited without durable report')
                if publish_schedule.due(float(data.time),time.monotonic()):io.publish()
                owner.step(control)
                if data.time>=next_sample:measure();next_sample+=.1
                if data.time>=next_save:
                    save();print('continuous loaded Nav2 sim %.2f worker %s'%(data.time,worker['status'] if worker else 'RUNNING'),flush=True)
                    next_save+=2.
                if stop_at is not None and data.time-stop_at>=5:break
                delay=(float(data.time)-begin)/float(self.cfg['sim']['realtime_factor'])-(time.monotonic()-started)
                if delay>0:time.sleep(min(delay,.05))
            stop=[row for row in truth if stop_at is not None and row['sim_s']>=stop_at+1]
            if not stop:raise RuntimeError('NAV2_STOP_WINDOW_MISSING')
            drift=max(float(np.linalg.norm(row['xyz'][:2]-stop[0]['xyz'][:2])) for row in stop)
            speed=max(float(np.linalg.norm(row['velocity'][:3])) for row in stop)
            cargo_speed=max(row['cargo'][name]['point_speed_bound_mps'] for row in stop for name in row['cargo'])
            cargo_drift=max(float(np.linalg.norm(row['cargo'][name]['xyz']-stop[0]['cargo'][name]['xyz']))
                            for row in stop for name in row['cargo'])
            slip=max(float(np.linalg.norm(row['cargo']['c_payload']['relative_to_deck'][:2]-truth[0]['cargo']['c_payload']['relative_to_deck'][:2])) for row in truth)
            checks=dict(nav2_succeeded=bool(worker and worker['status']=='SUCCEEDED'),
                load_authorized=gate.latched_reason is None,
                transport_posture_authorized=posture_gate.latched_reason is None,
                arrived=float(np.linalg.norm(truth[-1]['xyz'][:2]-np.array(goal[:2])))<=.25,
                vehicle_stopped=speed<=.01 and drift<=.005,
                cargo_stopped=cargo_speed<=.01 and cargo_drift<=.005,
                tray_retained=slip<=.005,
                deck_support=plant.tray_support()==['deck'],
                visual_quantity=latest.get('status')=='RESOLVED' and latest.get('counts')=={'red':2,'blue':1})
            result.update(status='COMPLETED',checks={k:'PASS' if v else 'FAIL' for k,v in checks.items()},
                stop_speed_mps=speed,stop_drift_m=drift,cargo_stop_speed_mps=cargo_speed,
                cargo_stop_drift_m=cargo_drift,max_tray_relative_slip_m=slip,
                stop_at_sim_s=stop_at,states_sent=io.sent,commands_received=io.received)
            if not all(checks.values()):raise RuntimeError('CONTINUOUS_NAV_CHECK_FAILED')
            owned.handles['worker'].wait(timeout=5)
            io.close()
            return_to_parking(plant,token,dict(stopped_confirmed=True,held_speed_mps=speed,drift_m=drift),
                nav_process_stopped=worker['processes']['nav2'].get('exit_code') is not None,ipc_closed=io.closed)
            result['authority_returned']=True
            return dict(final_x_m=float(truth[-1]['xyz'][0]),error_x_m=float(truth[-1]['xyz'][0]-goal[0]),
                end_s=float(data.time),stopped_confirmed=True,held_speed_mps=speed,drift_m=drift,
                navigation_succeeded=True,travelled_m=float(np.linalg.norm(truth[-1]['xyz'][:2]-initial_xy)),
                nav_evidence='continuous_nav.json')
        except (Exception,KeyboardInterrupt) as exc:
            result['status']='ERROR';result['error']=type(exc).__name__+': '+str(exc)
            result['exception_chain']=exception_chain(exc)
            # On interruption the caller retains occupied cargo/resources.
            # Never silently return wheel authority without confirmed stop.
            if io is not None and owner.frozen is None:
                # Sensor/navigation failure revokes MOTION, not the brake.
                # This is a separately bounded attempt, never task recovery.
                brake_begin=float(data.time);brake_wall=time.monotonic()
                try:
                    stop_at=brake_begin
                    while data.time-brake_begin<5:
                        if time.monotonic()-brake_wall>30:
                            raise TimeoutError('failure brake wall budget')
                        owner.step(lambda:io.control(lambda requested:False))
                        if not truth or data.time-truth[-1]['sim_s']>=.1:measure()
                    result['failure_brake_attempt']='COMPLETED_NOT_INDEPENDENTLY_JUDGED'
                except Exception as brake_error:
                    result['failure_brake_error']=type(brake_error).__name__+': '+str(brake_error)
                    if owner.frozen is None:
                        owner.permit.stop('NAVIGATION_BRAKE_UNCONFIRMED','explicit simulation fallback, not mechanical stop')
                        try:owner.step(lambda:io.control(lambda requested:False))
                        except Exception:pass
            raise
        finally:
            owned.stop('worker','joint_nav_worker.py');owned.stop('bridge','joint_world_ros_bridge.py');owned.close()
            if io is not None:io.close()
            if view is not None:view.close()
            result['processes']=owned.record
            result['simulation_frozen']=owner.frozen
            save()
