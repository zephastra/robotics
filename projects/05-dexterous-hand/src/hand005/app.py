"""Run independent physical hand experiments with auditable outcomes."""
import argparse
import csv
from datetime import datetime, timezone
import hashlib
import json
import time

import mujoco
import numpy as np

from .core import Task, OPEN, slew
from .model import ROOT, load, observe, verify_assets


def capture(m, d, path):
    from PIL import Image
    with mujoco.Renderer(m, 720, 960) as renderer:
        camera = mujoco.MjvCamera()
        camera.lookat[:] = [.045, .08, .24]
        camera.distance, camera.azimuth, camera.elevation = .7, 120, -25
        renderer.update_scene(d, camera=camera)
        Image.fromarray(renderer.render()).save(path)


def run(args):
    manifest = verify_assets()
    m, d = load(args.seed, args.offset, args.mass)
    out = ROOT / 'reports' / datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S%fZ')
    out.mkdir(parents=True)
    report = dict(status='RUNNING', mode=args.mode, seed=args.seed, offset_m=args.offset,
                  object_mass_kg=args.mass, source_revision=manifest['revision'],
                  asset_sha256=manifest['sha256'],
                  limitations=['Simulated 2-axis wrist fixture; no arm or whole-body balance',
                               'Known sphere and scripted joint targets; no vision or learned policy',
                               'Contact force is simulation-derived, not hardware tactile sensing'],
                  events=[], trials=[], mujoco_version=mujoco.__version__)
    (out / 'report.json').write_text(json.dumps(report, indent=2) + '\n')
    print('Report:', out, flush=True)
    panel = viewer = None
    samples = []
    desired = np.r_[OPEN, 0., 0.]
    command = desired.copy()
    task = Task(0., observe(m, d)) if args.mode == 'demo' else None
    previous_phase = ''
    rates = np.r_[np.full(16, 1.2), .08, .06]
    wall_start = time.monotonic()
    next_tick = wall_start
    next_gui = 0.
    recorded = False
    try:
        if not args.headless:
            from mujoco import viewer as viewer_module
            from .panel import Panel
            # No Python key callback and no set_texts(): controls live in Tk.
            viewer = viewer_module.launch_passive(m, d)
            with viewer.lock():
                viewer.cam.lookat[:] = [.045, .08, .24]
                viewer.cam.distance, viewer.cam.azimuth, viewer.cam.elevation = .7, 120, -25
            panel = Panel(m.actuator_ctrlrange)
            print('Use the separate 005 Hand Controls window. Sliders set joint targets.', flush=True)
        step = 0
        while d.time < args.duration:
            now = float(d.time)
            wall_now = time.monotonic()
            if panel and wall_now >= next_gui:
                panel.pump()
                next_gui = wall_now + .02
                if panel.closed or not viewer.is_running():
                    report.update(status='CANCELLED', reason='WINDOW_CLOSED')
                    break
                request, panel.request = panel.request, None
                if request:
                    report['events'].append(dict(time=now, action=request))
                    if request == 'demo':
                        obs = observe(m, d)
                        if (np.linalg.norm(np.array(obs['object']) - [.06, 0., .205]) < .015
                                and obs['source_contact'] and obs['speed'] < .03):
                            if task and not recorded:
                                report['trials'].append(dict(result='CANCELLED', reason='RESTART_REQUEST', events=task.events))
                            task, recorded = Task(now, obs), False
                            report['events'].append(dict(time=now, action='DEMO_ACCEPTED'))
                        else:
                            report['events'].append(dict(time=now, action='DEMO_REJECTED_NOT_AT_SOURCE'))
                            print('Demo rejected: restart program for a fresh object at the source.', flush=True)
                    elif request in ('manual', 'hold'):
                        if task and not task.result:
                            task.finish('CANCELLED', 'MANUAL_OVERRIDE', now)
                        if request == 'manual':
                            desired = panel.target.copy()
                        else:
                            desired = d.qpos[m.jnt_qposadr[m.actuator_trnid[:, 0]]].copy()
                            panel.target[:] = desired
                            for var, value in zip(panel.scales, desired):
                                var.set(float(value))
                panel.display(now, task.phase if task else 'MANUAL', observe(m, d))
            if panel and panel.paused:
                viewer.sync()
                time.sleep(.01)
                next_tick = time.monotonic()
                continue
            if not np.all(np.isfinite(d.qpos)) or not np.all(np.isfinite(d.qvel)):
                report.update(status='FAILED', reason='NONFINITE_STATE')
                break
            if abs(now - step * m.opt.timestep) > 1e-6:
                report.update(status='FAILED', reason='EXTERNAL_TIME_RESET')
                break
            if step % 10 == 0:
                obs = observe(m, d)
                if task and not task.result:
                    target = task.command(now, obs)
                    if target is not None:
                        desired = target
                    if task.result == 'FAILED':
                        desired = d.qpos[m.jnt_qposadr[m.actuator_trnid[:, 0]]].copy()
                if task and task.phase != previous_phase:
                    print(f'[{now:6.2f}s] {task.phase} object={np.round(obs["object"], 3)}', flush=True)
                    previous_phase = task.phase
                    if args.snapshot and task.phase in ('HOLD', 'COMPLETED', 'FAILED'):
                        capture(m, d, out / (task.phase.lower() + '.png'))
                if task and task.result and not recorded:
                    report['trials'].append(dict(result=task.result, reason=task.reason, events=task.events,
                                                max_lift_m=task.max_lift))
                    report.update(status=task.result, reason=task.reason)
                    recorded = True
                    if args.mode == 'demo' and not args.keep_open:
                        break
                if step % 25 == 0:
                    samples.append(dict(time=now, phase=task.phase if task else 'MANUAL',
                                        x=obs['object'][0], y=obs['object'][1], z=obs['object'][2],
                                        speed=obs['speed'], source_contact=obs['source_contact'],
                                        tray_contact=obs['tray_contact'], **obs['finger_forces']))
            command = slew(command, np.clip(desired, m.actuator_ctrlrange[:, 0], m.actuator_ctrlrange[:, 1]), rates, m.opt.timestep)
            d.ctrl[:] = command
            mujoco.mj_step(m, d)
            step += 1
            if viewer and step % 16 == 0:
                viewer.sync()
            if not args.headless:
                next_tick += m.opt.timestep
                remaining = next_tick - time.monotonic()
                if remaining > 0:
                    time.sleep(remaining)
                elif remaining < -.25:
                    next_tick = time.monotonic()
        else:
            if report['status'] == 'RUNNING':
                report.update(status='TIMEOUT' if task else 'OBSERVED', reason='DURATION_LIMIT')
    except KeyboardInterrupt:
        report.update(status='CANCELLED', reason='KEYBOARD_INTERRUPT')
    except Exception as exc:
        report.update(status='ERROR', reason=f'{type(exc).__name__}: {exc}')
        raise
    finally:
        if task and not recorded:
            report['trials'].append(dict(result=report['status'] if report['status'] != 'RUNNING' else 'CANCELLED',
                                        reason=report.get('reason', 'SESSION_ENDED'), events=task.events,
                                        max_lift_m=task.max_lift))
        if panel:
            panel.destroy()
        if viewer:
            viewer.close()
        report.update(sim_seconds=float(d.time), wall_seconds=time.monotonic()-wall_start,
                      final_observation=observe(m, d),
                      source_sha256={str(p.relative_to(ROOT)): hashlib.sha256(p.read_bytes()).hexdigest()
                                     for p in sorted([*(ROOT/'src').rglob('*.py'), ROOT/'scripts/prepare_assets.py'])})
        (out/'report.json').write_text(json.dumps(report, indent=2, allow_nan=False)+'\n')
        if samples:
            with (out/'trajectory.csv').open('w', newline='') as f:
                writer = csv.DictWriter(f, fieldnames=samples[0])
                writer.writeheader()
                writer.writerows(samples)
        print(json.dumps({k: report.get(k) for k in ['status', 'reason', 'sim_seconds']}), flush=True)
    return report, out


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--mode', choices=['manual', 'demo'], default='manual')
    p.add_argument('--headless', action='store_true')
    p.add_argument('--keep-open', action='store_true')
    p.add_argument('--snapshot', action='store_true')
    p.add_argument('--duration', type=float, default=120.)
    p.add_argument('--seed', type=int, default=0)
    p.add_argument('--offset', type=float, default=0.)
    p.add_argument('--mass', type=float, default=.05)
    args = p.parse_args()
    if not np.isfinite(args.duration) or args.duration <= 0:
        p.error('--duration must be positive and finite')
    if not 0 <= args.offset <= .01 or not .01 <= args.mass <= .2:
        p.error('--offset must be 0..0.01 m; --mass must be 0.01..0.2 kg')
    if args.headless and (args.mode == 'manual' or args.keep_open):
        p.error('Headless mode requires --mode demo, without --keep-open')
    report, _ = run(args)
    raise SystemExit(0 if report['status'] in ('COMPLETED', 'OBSERVED', 'CANCELLED') else 1)


if __name__ == '__main__':
    main()
