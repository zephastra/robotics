"""Measure policy stability after adding a hand and transferring arm ownership."""
import argparse
from datetime import datetime, timezone
import json
from pathlib import Path
import sys
import time
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'src'))
from humanoid006.runtime import Runtime, OPEN, GRASP


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--baseline', action='store_true')
    p.add_argument('--arm', choices=['policy', 'neutral', 'reach'], default='policy')
    p.add_argument('--walk', action='store_true')
    p.add_argument('--duration', type=float, default=15.)
    p.add_argument('--snapshot', action='store_true')
    args = p.parse_args()
    r = Runtime(args.baseline)
    out = ROOT/'reports'/('probe-'+datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S%fZ'))
    out.mkdir(parents=True)
    records = []
    status = 'COMPLETED'
    started = time.monotonic()
    while r.d.time < args.duration:
        now = r.d.time
        arm = None
        if now > 2 and args.arm != 'policy':
            arm = r.policy.default[18:23].copy() if args.arm == 'neutral' else np.array([-.35, -.2, 0., -.9, 0.])
        goal = [1., 0., 0.] if args.walk and now > 5 else [0., 0., 0.]
        r.step(r.hold_command(goal), arm, OPEN)
        if r.tick % 250 == 0:
            sample = r.snapshot()
            records.append(sample)
            print(sample, flush=True)
            if sample['z'] < .65 or sample['tilt_deg'] > 50 or not np.isfinite(r.d.qpos).all():
                status = 'FAILED'
                break
    report = dict(status=status, scope='Empty-hand policy integration probe, NOT grasp/carry validation',
                  args=vars(args), model_mass=float(r.m.body_mass.sum()), wall_seconds=time.monotonic()-started,
                  records=records, final=r.snapshot())
    (out/'report.json').write_text(json.dumps(report, indent=2)+'\n')
    if args.snapshot:
        import mujoco
        from PIL import Image
        with mujoco.Renderer(r.m, 720, 960) as renderer:
            camera = mujoco.MjvCamera()
            camera.lookat[:] = [r.d.qpos[0], r.d.qpos[1], 1.0]
            camera.distance, camera.azimuth, camera.elevation = 3., 135, -15
            renderer.update_scene(r.d, camera=camera)
            Image.fromarray(renderer.render()).save(out/'final.png')
    print('Report:', out, flush=True)
    raise SystemExit(0 if status=='COMPLETED' else 1)


if __name__ == '__main__':
    main()
