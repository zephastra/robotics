"""Independent visual transport demo; truth is used only by the report evaluator."""
import argparse
from contextlib import ExitStack
from datetime import datetime,timezone
import hashlib
import json
import math
import sys
import time
import traceback
import tempfile
from pathlib import Path
import mujoco
import numpy as np
from PIL import Image
from .runtime import ROOT,Runtime,verify_assets
from .camera import HeadCamera
from .task import Mission,CFG
from .tactile import sense
from .diagnostics import ContactMonitor


def new_report_directory(reports):
    reports=Path(reports)
    reports.mkdir(parents=True,exist_ok=True)
    # Wall-clock resolution is not a uniqueness guarantee across WSL workers.
    return Path(tempfile.mkdtemp(prefix=datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S%fZ')+'-',dir=reports))


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--mode',choices=['vision','lift','transport'],default='transport')
    parser.add_argument('--headless',action='store_true')
    parser.add_argument('--keep-open',action='store_true')
    parser.add_argument('--snapshot',action='store_true')
    parser.add_argument('--duration',type=float,default=180.)
    parser.add_argument('--box-offset',type=float,nargs=2,default=[0.,0.])
    parser.add_argument('--target-offset',type=float,nargs=2,default=[0.,0.])
    parser.add_argument('--box-yaw',type=float,default=0.,help='Initial box yaw in degrees')
    parser.add_argument('--stance',choices=['auto','legacy'],default='auto',help='Automatic bounded stance search, or explicit legacy comparison')
    parser.add_argument('--hold-controller',choices=['walking','posture'],default='walking',help='Experimental measured-pose hold after stopping; walking policy is unchanged')
    args=parser.parse_args()
    if (not np.isfinite([args.duration,args.box_yaw,*args.box_offset,*args.target_offset]).all()
        or args.duration<=0 or abs(args.box_yaw)>10 or max(map(abs,args.box_offset))>.04
        or max(map(abs,args.target_offset))>.15):parser.error('Invalid or out-of-scope initial conditions')
    out=new_report_directory(ROOT/'reports')
    print('Report:',out,flush=True)
    report={'status':'ERROR','reason':'INITIALIZATION_FAILED','args':vars(args),'records':[]}
    report['source_sha256']={p.relative_to(ROOT).as_posix():hashlib.sha256(p.read_bytes()).hexdigest()
                             for p in (ROOT/'src').rglob('*.py')}
    report['config_sha256']={p.relative_to(ROOT).as_posix():hashlib.sha256(p.read_bytes()).hexdigest()
                             for p in (ROOT/'config').rglob('*') if p.is_file()}
    report['runtime_versions']=dict(python=sys.version,mujoco=mujoco.__version__,numpy=np.__version__)
    panel=None;r=None;camera=None;monitor=ContactMonitor();started=time.monotonic()
    try:
        report['manifest']=verify_assets()
        r=Runtime()
        # Initial scene variation only. Never passed to Mission or the detector.
        address=r.m.joint('payload_free').qposadr[0]
        r.d.qpos[address:address+2]+=args.box_offset
        angle=math.radians(args.box_yaw)/2
        r.d.qpos[address+3:address+7]=[math.cos(angle),0,0,math.sin(angle)]
        for name in ('destination_table','destination_leg','destination_marker','destination_marker_plate','destination_marker_post'):
            r.m.geom(name).pos[:2]+=args.target_offset
        mujoco.mj_forward(r.m,r.d)
        mission=Mission(r,args.mode,args.stance)
        camera=HeadCamera(r,CFG['camera_width'],CFG['camera_height'])
        with ExitStack() as stack:
            viewer=None;paused=False;cancel=False
            if not args.headless:
                import tkinter as tk
                from PIL import ImageTk
                import mujoco.viewer as mv
                panel=tk.Tk();panel.title('007 Active Vision — Head Camera and Controls')
                label=tk.StringVar()
                tk.Label(panel,textvariable=label,width=60,height=5).pack()
                view=tk.Label(panel);view.pack()
                def pause():
                    nonlocal paused
                    paused=not paused
                def stop():
                    nonlocal cancel
                    cancel=True
                tk.Button(panel,text='Pause / Resume',command=pause).pack(fill='x')
                tk.Button(panel,text='Cancel and freeze simulation',command=stop).pack(fill='x')
                panel.protocol('WM_DELETE_WINDOW',stop)
                viewer=stack.enter_context(mv.launch_passive(r.m,r.d))
                viewer.cam.lookat[:]=[.6,0,.95]
                viewer.cam.distance,viewer.cam.azimuth,viewer.cam.elevation=3.2,135,-15
            next_vision=0.;last=''
            while not mission.result and r.d.time<args.duration:
                frame_start=time.monotonic()
                if panel:
                    panel.update()
                    label.set(f'{"PAUSED | " if paused else ""}{mission.phase} | {r.d.time:.2f}s\n'
                              f'Box visible: {mission.box is not None and r.d.time-mission.box.time<.8}\n'
                              'Camera moves with the actual head joints.\nMagenta: box marker. Green: receiving area.')
                if cancel or (viewer and not viewer.is_running()):
                    mission.finish('CANCELLED','USER_CANCELLED_SIMULATION_FROZEN');break
                if not paused:
                    for _ in range(10):
                        if r.d.time>=next_vision:
                            frame=camera.capture();mission.perceive(frame)
                            next_vision=r.d.time+CFG['vision_period']
                            if panel:
                                photo=ImageTk.PhotoImage(Image.fromarray(frame.rgb))
                                view.configure(image=photo);view.image=photo
                        if r.tick%25==0:
                            touch=sense(r)
                            mission.update(touch)
                            if (mission.phase,mission.since)!=last:
                                print(f'{r.d.time:.2f}s {mission.phase} {mission.reason}',flush=True)
                                last=(mission.phase,mission.since)
                                if args.snapshot:Image.fromarray(camera.last.rgb).save(out/f'eyes-{mission.phase}.png')
                            if r.tick%250==0:
                                station_collisions=[]
                                for ci in range(r.d.ncon):
                                    contact=r.d.contact[ci]
                                    names=[r.m.geom(int(g)).name for g in (contact.geom1,contact.geom2)]
                                    bodies=[r.m.body(int(r.m.geom_bodyid[g])).name for g in (contact.geom1,contact.geom2)]
                                    if any(n in ('source_table','source_leg','destination_table','destination_leg') for n in names) and 'payload' not in bodies:
                                        station_collisions.append(dict(geoms=names,bodies=bodies))
                                truth=r.d.body('payload').xpos.copy()
                                detected=None if mission.box is None else mission.box.center-mission.box.normal*CFG['marker_height_above_center']
                                report['records'].append(dict(**r.snapshot(),phase=mission.phase,touch=touch,
                                    goal=mission.goal.tolist(),command=r.command.tolist(),
                                    arm_solution=None if mission.arm is None else mission.arm.tolist(),
                                    hold_anchor=None if mission.hold_anchor is None else mission.hold_anchor.tolist(),
                                    station_collisions=station_collisions,
                                    destination_estimate=None if mission.destination is None else mission.destination.center.tolist(),
                                    payload_truth=truth.tolist(),box_estimate=None if detected is None else detected.tolist(),
                                    detection_age=None if mission.box is None else float(r.d.time-mission.box.time)))
                        if mission.result:break
                        brake=(args.stance=='auto' and mission.phase in ('CARRY','APPROACH')
                               and (mission.phase!='CARRY' or not mission.route))
                        alignment=(args.stance=='auto' and mission.phase=='CARRY' and mission.retries>0)
                        control_goal=mission.goal if mission.hold_anchor is None else mission.hold_anchor
                        support_position=(r.support_midpoint()[:2]+mission.alignment_support_offset
                                          if alignment and mission.alignment_support_offset is not None else None)
                        command=(np.zeros(3) if mission.phase=='STOP' else
                                 r.hold_command(control_goal,precise=mission.phase in ('CARRY','APPROACH'),brake=brake,alignment=alignment,position=support_position))
                        r.step(command,
                               mission.arm,mission.hands,mission.head,
                               stationary=args.hold_controller=='posture' and mission.hold_anchor is not None,
                               stopping=mission.phase=='STOP')
                        monitor.sample(r,mission.phase)
                if viewer:
                    viewer.sync();time.sleep(max(0,.02-(time.monotonic()-frame_start)))
            if not mission.result:mission.finish('TIMEOUT','DURATION_LIMIT')
            report.update(status=mission.result,reason=mission.reason,events=mission.events,final=r.snapshot(),
                          stance_plans=mission.stance_plans,station_contact_diagnostics=monitor.summary())
            truth=r.d.body('payload').xpos.copy()
            target=r.m.geom('destination_table').pos.copy()+np.array([0,0,.047])
            velocity=np.zeros(6)
            mujoco.mj_objectVelocity(r.m,r.d,mujoco.mjtObj.mjOBJ_BODY,r.m.body('payload').id,velocity,0)
            touch=sense(r)
            report['evaluation']=dict(payload=truth.tolist(),target=target.tolist(),
                xy_error=float(np.linalg.norm(truth[:2]-target[:2])),height_error=float(abs(truth[2]-target[2])),
                speed=float(np.linalg.norm(velocity[3:])),touch=touch)
            if mission.box is not None:
                estimate=mission.box.center-mission.box.normal*CFG['marker_height_above_center']
                report['evaluation']['visual_position_error']=float(np.linalg.norm(estimate-truth))
                report['evaluation']['visual_age']=float(r.d.time-mission.box.time)
            if mission.result=='COMPLETED' and args.mode=='vision':
                ev=report['evaluation']
                if ev.get('visual_position_error',float('inf'))>.02 or ev.get('visual_age',float('inf'))>.8:
                    report.update(status='FAILED',reason='VISUAL_EVALUATION_REJECTED')
            if mission.result=='COMPLETED' and args.mode=='transport':
                ev=report['evaluation']
                if not (ev['xy_error']<.06 and ev['height_error']<.012 and ev['speed']<.025 and touch['destination']
                        and touch['left']<.02 and touch['right']<.02):
                    report.update(status='FAILED',reason='INDEPENDENT_EVALUATION_REJECTED')
            if args.snapshot:
                with mujoco.Renderer(r.m,720,960) as renderer:
                    cam=mujoco.MjvCamera();cam.lookat[:]=[.6,0,.95]
                    cam.distance,cam.azimuth,cam.elevation=3.2,135,-15
                    renderer.update_scene(r.d,camera=cam)
                    Image.fromarray(renderer.render()).save(out/'final.png')
            report['wall_seconds']=time.monotonic()-started
            (out/'report.json').write_text(json.dumps(report,indent=2)+'\n')
            print(report['status'],report['reason'],flush=True)
            while args.keep_open and viewer and viewer.is_running() and not cancel:
                label.set(f'{report["status"]}\n{report["reason"]}\nResult frozen; close to exit.')
                panel.update();viewer.sync();time.sleep(.03)
    except KeyboardInterrupt: report.update(status='CANCELLED',reason='PROCESS_INTERRUPTED')
    except Exception:
        report.update(status='ERROR',reason='EXCEPTION',traceback=traceback.format_exc());traceback.print_exc()
    finally:
        report['station_contact_diagnostics']=monitor.summary()
        if camera:camera.close()
        if panel:panel.destroy()
        report['wall_seconds']=time.monotonic()-started
        (out/'report.json').write_text(json.dumps(report,indent=2)+'\n')
    return 0 if report['status']=='COMPLETED' else 1


if __name__=='__main__': raise SystemExit(main())
