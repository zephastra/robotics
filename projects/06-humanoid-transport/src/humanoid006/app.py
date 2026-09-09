"""Local demonstrator with measured outcomes and independent simulation controls."""
import argparse
from contextlib import ExitStack
from datetime import datetime, timezone
import hashlib
import json
import time
import traceback

import numpy as np

from .runtime import ROOT, Runtime, verify_assets
from .task import Mission, observe


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--mode', choices=('lift', 'transport'), default='transport')
    parser.add_argument('--headless', action='store_true')
    parser.add_argument('--snapshot', action='store_true')
    parser.add_argument('--duration', type=float, default=120.)
    parser.add_argument('--payload-offset', type=float, nargs=2, default=(0.,0.), metavar=('X','Y'),
                        help='Initial object XY perturbation in meters; not applied during simulation')
    parser.add_argument('--keep-open', action='store_true', help='Keep the frozen result visible until closed')
    args = parser.parse_args()
    if not np.isfinite(args.duration) or args.duration <= 0:
        parser.error('--duration must be positive and finite')
    if not np.isfinite(args.payload_offset).all() or np.max(np.abs(args.payload_offset))>.005:
        parser.error('--payload-offset must be finite and within +/-0.005 meters')
    out = ROOT/'reports'/datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S%fZ')
    out.mkdir(parents=True)
    print('Report:', out, flush=True)
    report = dict(status='ERROR', reason='INITIALIZATION_FAILED', mode=args.mode,
                  payload_offset=list(args.payload_offset), records=[], events=[])
    report['source_sha256'] = {
        p.relative_to(ROOT).as_posix(): hashlib.sha256(p.read_bytes()).hexdigest()
        for p in (ROOT/'src').rglob('*.py')}
    started = time.monotonic()
    panel = viewer = r = mission = None
    try:
        report['manifest'] = verify_assets()
        r = Runtime()
        if any(args.payload_offset):
            import mujoco
            address = r.m.joint('payload_free').qposadr[0]
            r.d.qpos[address:address+2] += args.payload_offset
            mujoco.mj_forward(r.m,r.d)  # Initial conditions only, before the first step.
        mission = Mission(r, stop_after_lift=args.mode == 'lift')
        with ExitStack() as stack:
            paused, cancel = False, False
            if not args.headless:
                import tkinter as tk
                import mujoco.viewer
                panel = tk.Tk()
                panel.title('006 Humanoid Transport — Simulation Controls')
                label = tk.StringVar(value='Starting…')
                tk.Label(panel, textvariable=label, width=64, height=6).pack(padx=14, pady=8)
                def toggle():
                    nonlocal paused
                    paused = not paused
                def stop():
                    nonlocal cancel
                    cancel = True
                tk.Button(panel, text='Pause / Resume', command=toggle).pack(fill='x', padx=14)
                tk.Button(panel, text='Cancel and freeze simulation', command=stop).pack(fill='x', padx=14, pady=8)
                panel.protocol('WM_DELETE_WINDOW', stop)
                # No viewer text overlays or Python key callbacks: these caused GUI
                # problems in an earlier project. Controls live in this separate panel.
                viewer = stack.enter_context(mujoco.viewer.launch_passive(r.m, r.d))
                viewer.cam.lookat[:] = [.6, -.1, .95]
                viewer.cam.distance, viewer.cam.azimuth, viewer.cam.elevation = 3.3, 130, -15
            last = ''
            while not mission.result and r.d.time < args.duration:
                frame_start = time.monotonic()
                if panel:
                    panel.update()
                    label.set(f'{"PAUSED | " if paused else ""}{mission.phase} | {r.d.time:.2f} simulation seconds\n'
                              f'Mode: {args.mode}\nExperimental T800 + Allegro right hand\n'
                              'Close/cancel freezes simulation; not a hardware emergency stop.')
                if cancel or (viewer and not viewer.is_running()):
                    mission.finish('CANCELLED', 'USER_CANCELLED_SIMULATION_FROZEN')
                    break
                if not paused:
                    for _ in range(10):
                        if r.tick % 25 == 0:
                            mission.update()
                            obs = observe(r)
                            if mission.phase != last:
                                print(f'{r.d.time:6.2f}s {mission.phase} {mission.reason}', flush=True)
                                last = mission.phase
                            if r.tick % 250 == 0:
                                report['records'].append(dict(**r.snapshot(), phase=mission.phase,
                                    payload=obs['payload'].tolist(), forces=obs['forces'],
                                    target_contact=bool(obs['target']), payload_speed=obs['speed']))
                        if mission.result:
                            break
                        r.step(r.hold_command(mission.goal, precise=mission.phase == 'CARRY'), mission.arm, mission.hand)
                if viewer:
                    viewer.sync()
                    time.sleep(max(0., .02-(time.monotonic()-frame_start)))
            if not mission.result:
                mission.finish('TIMEOUT', 'DURATION_LIMIT')
            report.update(status=mission.result, reason=mission.reason, events=mission.events,
                          final=r.snapshot(), max_lift_m=mission.max_lift)
            obs = observe(r)
            report['payload_final'] = obs['payload'].tolist()
            report['target_contact'] = bool(obs['target'])
            report['payload_speed'] = obs['speed']
            if args.snapshot:
                import mujoco
                from PIL import Image
                with mujoco.Renderer(r.m, 720, 960) as renderer:
                    camera = mujoco.MjvCamera()
                    camera.lookat[:] = [.6, -.1, .95]
                    camera.distance, camera.azimuth, camera.elevation = 3.3, 130, -15
                    renderer.update_scene(r.d, camera=camera)
                    Image.fromarray(renderer.render()).save(out/'final.png')
            # Persist before a potentially long final-view wait.
            report['wall_seconds'] = time.monotonic()-started
            (out/'report.json').write_text(json.dumps(report, indent=2)+'\n')
            print(f'{mission.result}: {mission.reason}', flush=True)
            while args.keep_open and viewer and viewer.is_running() and not cancel:
                label.set(f'{mission.result}\n{mission.reason}\nResult frozen. Close either window to exit.')
                panel.update()
                viewer.sync()
                time.sleep(.03)
    except KeyboardInterrupt:
        report.update(status='CANCELLED', reason='PROCESS_INTERRUPTED_SIMULATION_FROZEN')
    except Exception:
        report.update(status='ERROR', reason='EXCEPTION', traceback=traceback.format_exc())
        traceback.print_exc()
    finally:
        if panel:
            panel.destroy()
        report['wall_seconds'] = time.monotonic()-started
        (out/'report.json').write_text(json.dumps(report, indent=2)+'\n')
    return 0 if report['status'] == 'COMPLETED' else 1


if __name__ == '__main__':
    raise SystemExit(main())
