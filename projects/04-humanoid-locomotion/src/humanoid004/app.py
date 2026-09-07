"""Physics-driven humanoid runner. Ground truth is used for waypoint feedback."""
import argparse
from contextlib import nullcontext
import csv
from datetime import datetime, timezone
import hashlib
from importlib.metadata import version
import json
import math
from pathlib import Path
import threading
import time
import traceback
import select
import sys
import termios
import tty

import mujoco
import numpy as np
import yaml

from .core import CommandFilter, FallMonitor, Pose, Route, orientation, target_command
from .robots import G1_JOINTS, get_robot, joint_addresses, manual_commands
from .manual_panel import ManualPanel, TerminalDecoder

ROOT = Path(__file__).resolve().parents[2]
EXPECTED_JOINTS = G1_JOINTS  # Backward-compatible G1 baseline checks.


class Keys:
    """Momentary key commands expire in wall time, including after viewer pauses."""
    def __init__(self, robot='g1'):
        self.commands = {ord(key): value for key, value in manual_commands(robot).items()}
        self.lock = threading.Lock()
        self.value = np.zeros(3)
        self.stamp = -math.inf
        self.cancel = False
        self.pause = False
        self.push = False

    def callback(self, key):
        with self.lock:
            commands = self.commands
            if key in commands:
                self.value = np.asarray(commands[key], dtype=float)
                self.stamp = time.monotonic()
            if key == ord('C'):
                self.cancel = True
            if key == ord('P'):
                self.pause = not self.pause
                self.stamp = -math.inf
            if key == ord('F'):
                self.push = True

    def read(self):
        with self.lock:
            cmd = self.value.copy() if time.monotonic()-self.stamp < 0.5 else np.zeros(3)
            push, self.push = self.push, False
            return cmd, self.cancel, self.pause, push

    def release(self):
        with self.lock:
            self.value[:] = 0
            self.stamp = -math.inf

    def held(self, codes):
        """UI heartbeat, not OS key repeat. A stalled UI still expires safely."""
        with self.lock:
            values = [self.commands[k] for k in codes if k in self.commands]
            if values and not self.pause and not self.cancel:
                bounds = np.asarray(list(self.commands.values()))
                self.value = np.clip(np.sum(values, axis=0), bounds.min(axis=0), bounds.max(axis=0))
                self.stamp = time.monotonic()
            else:
                self.value[:] = 0
                self.stamp = -math.inf


class TerminalKeys:
    """Terminal key repeat supplies a deadman input without GUI focus."""
    def __init__(self, keys, enabled, allow_movement=True):
        self.keys=keys
        self.enabled=enabled and sys.stdin.isatty()
        self.done=threading.Event()
        self.decoder=TerminalDecoder()
        self.allow_movement=allow_movement

    def __enter__(self):
        if self.enabled:
            self.fd=sys.stdin.fileno()
            self.previous=termios.tcgetattr(self.fd)
            tty.setcbreak(self.fd)
            self.thread=threading.Thread(target=self.loop,daemon=True)
            self.thread.start()
        return self

    def loop(self):
        while not self.done.is_set():
            if select.select([self.fd],[],[],.1)[0]:
                import os
                ch=os.read(self.fd,1)
                if not ch: break
                for key in self.decoder.feed(ch):
                    if self.allow_movement or key in (ord('C'), ord('X')):
                        self.keys.callback(key)

    def __exit__(self,*exc):
        if self.enabled:
            self.done.set()
            self.thread.join(timeout=.3)
            termios.tcsetattr(self.fd,termios.TCSADRAIN,self.previous)


def check_assets(robot='g1'):
    profile = get_robot(robot, ROOT)
    manifest = json.loads((ROOT / profile.manifest).read_text())
    for name, expected in manifest['sha256'].items():
        if hashlib.sha256((ROOT/name).read_bytes()).hexdigest() != expected:
            raise ValueError('Asset hash mismatch: '+name)
    return manifest


def add_markers(viewer, goal, position, phase, now, command, tilt, title='Unitree G1'):
    with viewer.lock():
        scene = viewer.user_scn
        scene.ngeom = 0
        if goal is not None:
            geom = scene.geoms[0]
            mujoco.mjv_initGeom(geom, mujoco.mjtGeom.mjGEOM_SPHERE, np.array([.08,.08,.08]),
                               np.array([goal['x'],goal['y'],.12]), np.eye(3).ravel(), np.array([1,.9,.1,1],dtype=np.float32))
            scene.ngeom = 1
        viewer.cam.lookat[:] = [position[0], position[1], .65]
    # Do not use viewer.set_texts here: the live MuJoCo 3.3.6 viewer was
    # observed stuck inside that call, starving simulation and Tk input.
    # Status remains available in the control panel and terminal instead.


def save_report(out, report, samples, cfg):
    report['config'] = cfg
    (out / 'report.json').write_text(json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False)+'\n')
    if samples:
        with (out/'trajectory.csv').open('w', newline='') as f:
            writer = csv.DictWriter(f, fieldnames=list(samples[0]))
            writer.writeheader()
            writer.writerows(samples)
        try:
            import matplotlib
            matplotlib.use('Agg')
            import matplotlib.pyplot as plt
            fig, axes = plt.subplots(1, 2, figsize=(11,4))
            axes[0].plot([s['x'] for s in samples], [s['y'] for s in samples], label='simulated base')
            for g in cfg['route']:
                axes[0].add_patch(plt.Circle((g['x'],g['y']), cfg['position_tolerance'], fill=False, color='green'))
                axes[0].text(g['x'],g['y'],g['name'])
            axes[0].set(xlabel='x [m]', ylabel='y [m]', title='Ground-truth trajectory', aspect='equal')
            axes[1].plot([s['t'] for s in samples], [s['tilt_deg'] for s in samples], label='body tilt [deg]')
            axes[1].set(xlabel='simulation time [s]', ylabel='body tilt [deg]', title=report['status'])
            speed_ax=axes[1].twinx()
            speed_ax.plot([s['t'] for s in samples], [s['speed'] for s in samples], color='orange',alpha=.6)
            speed_ax.set_ylabel('planar speed [m/s]',color='orange')
            for ax in axes:
                ax.grid(alpha=.3)
                ax.legend()
            fig.tight_layout()
            fig.savefig(out/'summary.png', dpi=140)
            plt.close(fig)
        except Exception as exc:
            (out/'plot_error.txt').write_text(str(exc))


def run(args):
    robot = get_robot(args.robot, ROOT)
    cfg = yaml.safe_load((ROOT/'config/lab.yaml').read_text())
    cfg.update(control_decimation=robot.decimation, fall_height=robot.fall_height)
    run_id = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S%fZ')+'-'+robot.name+'-'+args.mode+'-'+str(args.seed)
    out = ROOT/'reports'/run_id
    out.mkdir(parents=True)
    manifest = check_assets(args.robot)
    print('Report:', out, flush=True)
    policy = robot.policy(ROOT)
    if cfg['simulation_dt'] != policy.cfg['simulation_dt'] or cfg['control_decimation'] != policy.cfg['control_decimation']:
        raise ValueError('Control timing differs from the pinned policy baseline')
    model = mujoco.MjModel.from_xml_path(str(ROOT/robot.scene))
    data = mujoco.MjData(model)
    model.opt.timestep = cfg['simulation_dt']
    qadr, vadr = joint_addresses(model, robot)
    pelvis = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, robot.base)
    rng = np.random.default_rng(args.seed)
    # Initial conditions only. No root-coordinate edits are permitted after stepping begins.
    if robot.name == 't800':
        data.qpos[qadr] = policy.default
    if args.vary_initial:
        data.qpos[:2] += rng.uniform(-.04, .04, 2)
        initial_yaw = float(rng.uniform(-.08, .08))
        data.qpos[3:7] = [math.cos(initial_yaw/2),0,0,math.sin(initial_yaw/2)]
    mujoco.mj_forward(model, data)
    initial = data.qpos[:7].copy().tolist()
    if args.mode == 'stand':
        cfg['route'] = [dict(name='STAND',x=initial[0],y=initial[1],yaw=orientation(initial[3:7])[0])]
        cfg['hold_seconds'] = 30.0
    filt = CommandFilter(cfg['command_limits'],cfg['command_acceleration'],cfg['input_timeout'])
    fall = FallMonitor(cfg['fall_height'],cfg['fall_tilt_degrees'],cfg['fall_debounce_seconds'])
    route = Route(cfg)
    keys = Keys(robot.name)
    report = {'status':'RUNNING','reason':'','robot':robot.name,'mode':args.mode,'seed':args.seed,'initial_pose':initial,
              'localization':'MuJoCo ground truth, not SLAM or a hardware estimator',
              'upstream_commit':manifest['commit'],'policy_sha256':manifest['sha256'][robot.weight],
              'upstream_repository':manifest['repository'],'policy_file':robot.weight,
              'events':[],'pushes':[],'stop_requested_at':None,
              'arguments':vars(args),
              'source_sha256':{str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest() for p in sorted((ROOT/'src').rglob('*.py'))},
              'environment':{'mujoco':mujoco.__version__,'policy_device':'cpu',
                             'policy_runtime': 'MNN' if robot.name=='t800' else 'torch',
                             'policy_runtime_version': version('MNN' if robot.name=='t800' else 'torch')}}
    samples=[]
    viewer_context=nullcontext(None)
    panel_context=nullcontext(None)
    if args.mode=='manual':
        panel_context=ManualPanel(keys,robot.title)
        report['manual_input']='Dedicated Tk press/release panel; terminal and viewer only C/X; UI heartbeat watchdog 0.5 wall seconds'
    if not args.headless:
        from mujoco import viewer as mj_viewer
        def viewer_key(key):
            if args.mode!='manual' or key in (ord('C'), ord('X')):
                keys.callback(key)
        viewer_context=mj_viewer.launch_passive(model,data,key_callback=viewer_key)
    control_dt=cfg['simulation_dt']*cfg['control_decimation']
    command=np.zeros(3)
    wall_start=time.monotonic()
    next_tick=wall_start
    step=0
    last_phase=None
    scheduled_push=False
    push_until=-1.0
    cancel_start=None
    settle_pose=None
    cancel_goal=None
    manual_goal=None
    stop_positions=[]
    stop_speeds=[]
    max_tilt=0.0
    min_z=float(data.qpos[2])
    warning_count=np.array(data.warning.number).copy()
    foot_ids=[mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_BODY,s) for s in robot.feet]
    foot_min=np.full(2,np.inf)
    foot_max=np.full(2,-np.inf)
    max_displacement=0.0
    accumulated_yaw=0.0
    previous_yaw=orientation(data.qpos[3:7])[0]
    try:
        with TerminalKeys(keys,args.mode=='manual',allow_movement=False), viewer_context as viewer, panel_context as panel:
            if viewer:
                with viewer.lock():
                    viewer.cam.distance=3.8
                    viewer.cam.azimuth=135
                    viewer.cam.elevation=-18
                if args.mode=='manual':
                    print('Manual: click the separate 004 Controls window, then HOLD W/S A/D Q/E R/T, or hold a direction button. Release/focus loss stops movement. X/Space stop; C cancel; P pause; F push. Terminal/viewer movement keys are disabled; C/X remain available.',flush=True)
                else:
                    print('Viewer: C cancel; P pause; F push test. Use --mode manual for the dedicated keyboard/mouse control window.',flush=True)
            while data.time < args.duration:
                if panel:
                    panel.pump()
                if abs(float(data.time)-step*cfg['simulation_dt']) > 1e-5:
                    raise RuntimeError('Simulation time was externally reset/changed; refusing to mix episodes')
                if viewer and not viewer.is_running():
                    report.update(status='CANCELLED', reason='VIEWER_CLOSED: simulation terminated, not a verified physical stop')
                    break
                key_cmd, cancel, paused, manual_push=keys.read()
                if cancel and paused:
                    # A cancellation must not wait forever behind a simulation pause.
                    keys.pause=False
                    paused=False
                if paused:
                    if panel:
                        panel.status(float(data.time),'PAUSED - press P to resume',np.zeros(3))
                    if viewer:
                        viewer.sync()
                    time.sleep(.02)
                    next_tick=time.monotonic()
                    continue
                if step % cfg['control_decimation'] == 0:
                    now=float(data.time)
                    yaw, tilt=orientation(data.qpos[3:7])
                    pose=Pose(float(data.qpos[0]),float(data.qpos[1]),yaw)
                    if not np.all(np.isfinite(data.qpos)) or not np.all(np.isfinite(data.qvel)):
                        raise FloatingPointError('Nonfinite simulation state')
                    if np.any(np.array(data.warning.number)>warning_count):
                        raise FloatingPointError('MuJoCo numerical warning; terminating instead of continuing reset state')
                    if fall.update(float(data.qpos[2]),tilt,now):
                        report.update(status='FAILED',reason='FALL_DETECTED')
                        break
                    if (cancel or (args.cancel_at is not None and now >= args.cancel_at)) and cancel_start is None:
                        cancel_start=now
                        report['stop_requested_at']=now
                        report['events'].append({'event':'CANCEL_REQUESTED','sim_time':now})
                    if cancel_start is not None:
                        desired=np.zeros(3)
                        phase='DECELERATING'
                        if now-cancel_start >= 3.0:
                            phase='STOP_CHECK'
                            if settle_pose is None:
                                settle_pose=np.array([pose.x,pose.y])
                                cancel_goal=dict(name='STOP',x=pose.x,y=pose.y,yaw=pose.yaw)
                            desired=target_command(pose,cancel_goal)
                            stop_positions.append(float(np.linalg.norm(np.array([pose.x,pose.y])-settle_pose)))
                            stop_speeds.append(float(np.linalg.norm(data.qvel[:2])))
                        if now-cancel_start >= 13.0:
                            drift=max(stop_positions,default=math.inf)
                            mean_speed=float(np.mean(stop_speeds)) if stop_speeds else math.inf
                            passed=drift <= .30 and mean_speed <= .15
                            report.update(status='CANCELLED' if passed else 'FAILED',reason='CONTROLLED_STOP' if passed else 'STOP_DRIFT_EXCEEDED')
                            report['stop_check']={'passed':passed,'hold_seconds':10,'max_drift_m':drift,'mean_speed_mps':mean_speed}
                            break
                    elif args.mode in ('route','push','stand'):
                        desired=route.step(pose,now,float(np.linalg.norm(data.qvel[:2])))
                        phase=route.phase
                        if phase in ('COMPLETED','FAILED'):
                            report.update(status=phase,reason=route.reason)
                            break
                    elif args.mode=='manual':
                        desired=key_cmd if now>=cfg['warmup_seconds'] else np.zeros(3)
                        phase='MANUAL'
                        if np.linalg.norm(desired)<1e-6 and now>=cfg['warmup_seconds']:
                            if manual_goal is None:
                                manual_goal=dict(name='HOLD',x=pose.x,y=pose.y,yaw=pose.yaw)
                            desired=target_command(pose,manual_goal)
                            phase='MANUAL_HOLD'
                        else:
                            manual_goal=None
                    else:
                        desired=np.array(args.command) if now>=cfg['warmup_seconds'] else np.zeros(3)
                        phase='BASELINE'
                    filt.submit(desired,now)
                    command=filt.step(now,control_dt)
                    policy.infer(data.qpos[qadr],data.qvel[vadr],data.qpos[3:7],data.qvel[3:6],command,now)
                    if (args.mode=='push' and now >= args.push_at and not scheduled_push) or manual_push:
                        scheduled_push=True
                        push_until=now+args.push_duration
                        report['pushes'].append({'sim_time':now,'body':robot.base,'force_world_N':[0,args.push_force,0],
                                                'duration_seconds':args.push_duration,'impulse_Ns':args.push_force*args.push_duration})
                    max_tilt=max(max_tilt,math.degrees(tilt))
                    min_z=min(min_z,float(data.qpos[2]))
                    max_displacement=max(max_displacement,float(np.linalg.norm(data.qpos[:2]-initial[:2])))
                    accumulated_yaw+=math.atan2(math.sin(yaw-previous_yaw),math.cos(yaw-previous_yaw))
                    previous_yaw=yaw
                    if now>=cfg['warmup_seconds']:
                        heights=data.xpos[foot_ids,2]
                        foot_min=np.minimum(foot_min,heights)
                        foot_max=np.maximum(foot_max,heights)
                    if step % (cfg['control_decimation']*5)==0:
                        samples.append({'t':now,'x':pose.x,'y':pose.y,'z':float(data.qpos[2]),'yaw':yaw,
                                        'tilt_deg':math.degrees(tilt),'speed':float(np.linalg.norm(data.qvel[:2])),
                                        'cmd_vx':float(command[0]),'cmd_vy':float(command[1]),'cmd_wz':float(command[2]),'phase':phase,
                                        'input_vx':float(key_cmd[0]),'input_vy':float(key_cmd[1]),'input_wz':float(key_cmd[2])})
                    if panel:
                        panel.status(now,phase,command)
                    if phase != last_phase:
                        goal_label=route.goal['name'] if args.mode in ('route','stand','push') else args.mode
                        print(f'[{now:7.2f}s] {phase} goal={goal_label} pose=({pose.x:.2f},{pose.y:.2f},{math.degrees(yaw):.1f}deg)',flush=True)
                        last_phase=phase
                    if viewer and step % (cfg['control_decimation']*2)==0:
                        add_markers(viewer,route.goal if args.mode in ('route','push','stand') else None,data.qpos[:3],phase,now,command,tilt,robot.title)
                        viewer.sync()
                data.xfrc_applied[pelvis,:]=0
                if data.time < push_until:
                    data.xfrc_applied[pelvis,1]=args.push_force
                data.ctrl[:]=policy.torques(data.qpos[qadr],data.qvel[vadr])
                mujoco.mj_step(model,data)
                step+=1
                if viewer or args.realtime:
                    next_tick+=cfg['simulation_dt']
                    delay=next_tick-time.monotonic()
                    if delay>0:
                        time.sleep(delay)
                    elif delay < -.25:
                        next_tick=time.monotonic()
            else:
                report.update(status='COMPLETED' if args.mode=='baseline' and cancel_start is None else 'FAILED',
                              reason='BASELINE_DURATION_FINISHED' if args.mode=='baseline' and cancel_start is None else 'SIMULATION_TIMEOUT')
            # Capture the actual final simulated frame, without changing physical state.
            if args.snapshot:
                try:
                    cam=mujoco.MjvCamera()
                    cam.lookat[:]=data.qpos[:3]
                    cam.distance=3.5
                    cam.azimuth=135
                    cam.elevation=-15
                    with mujoco.Renderer(model,height=480,width=640) as renderer:
                        renderer.update_scene(data,camera=cam)
                        from PIL import Image
                        Image.fromarray(renderer.render()).save(out/'final-camera.png')
                except Exception as exc:
                    report['snapshot_error']=str(exc)
    except KeyboardInterrupt:
        report.update(status='CANCELLED',reason='PROCESS_INTERRUPTED: simulation terminated; controlled stop not tested')
    except Exception as exc:
        report.update(status='FAILED',reason=type(exc).__name__+': '+str(exc))
        (out/'exception.txt').write_text(traceback.format_exc())
    finally:
        data.xfrc_applied[:]=0
        report['events']+=route.events
        if args.mode=='manual':
            report['input_events']=panel_context.events
        report['sim_seconds']=float(data.time)
        report['wall_seconds']=time.monotonic()-wall_start
        report['max_tilt_degrees']=max_tilt
        report['min_pelvis_height_m']=min_z
        report['max_displacement_from_start_m']=max_displacement
        report['accumulated_yaw_degrees']=math.degrees(accumulated_yaw)
        report['foot_body_vertical_range_m']=[float(v) if np.isfinite(v) else None for v in foot_max-foot_min]
        report['final_pose']=[float(v) if np.isfinite(v) else None for v in data.qpos[:7]]
        report['net_displacement_m']=float(np.linalg.norm(data.qpos[:2]-np.asarray(initial[:2]))) if np.all(np.isfinite(data.qpos[:2])) else None
        save_report(out,report,samples,cfg)
        print(json.dumps({k:report[k] for k in ['status','reason','sim_seconds','wall_seconds','net_displacement_m']})+'\n'+str(out/'report.json'),flush=True)
    return 0 if report['status']=='COMPLETED' or report.get('stop_check',{}).get('passed') else 1


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--robot',choices=['g1','t800'],default='g1',help='Load matching dynamics and policy, not just a visual skin')
    parser.add_argument('--mode',choices=['baseline','manual','route','push','stand'],default='route')
    parser.add_argument('--headless',action='store_true')
    parser.add_argument('--realtime',action='store_true')
    parser.add_argument('--snapshot',action='store_true')
    parser.add_argument('--duration',type=float,default=300)
    parser.add_argument('--command',type=float,nargs=3,default=[0,0,0])
    parser.add_argument('--seed',type=int,default=0)
    parser.add_argument('--vary-initial',action='store_true')
    parser.add_argument('--cancel-at',type=float)
    parser.add_argument('--push-at',type=float,default=8)
    parser.add_argument('--push-force',type=float,default=30)
    parser.add_argument('--push-duration',type=float,default=.2)
    args=parser.parse_args()
    values=[args.duration,args.push_at,args.push_force,args.push_duration,*args.command]
    if not all(math.isfinite(v) for v in values) or args.duration<=0 or args.push_duration<=0 or args.push_at<0:
        parser.error('Arguments must be finite; duration positive and push time nonnegative')
    if args.cancel_at is not None and (not math.isfinite(args.cancel_at) or args.cancel_at<0):
        parser.error('cancel-at must be finite and nonnegative')
    if args.mode=='manual' and args.headless:
        parser.error('Manual mode needs the viewer for keyboard input')
    try:
        result=run(args)
    except Exception as exc:
        startup_dir=ROOT/'reports'/('startup-failed-'+datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S%fZ'))
        startup_dir.mkdir(parents=True)
        (startup_dir/'report.json').write_text(json.dumps({'status':'FAILED','reason':str(exc),'phase':'STARTUP','arguments':vars(args)},indent=2)+'\n')
        (startup_dir/'exception.txt').write_text(traceback.format_exc())
        print('Startup failed:',exc,'Report:',startup_dir,flush=True)
        result=1
    raise SystemExit(result)


if __name__=='__main__':
    main()
