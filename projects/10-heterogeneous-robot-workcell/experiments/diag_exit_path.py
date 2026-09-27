"""Measure and compare candidate H exit paths. Diagnostic only; never an acceptance run.

The exit path is `humanoid007.runtime.ExitPath` -- the same object `probe_h.py` drives, so a
comparison made here cannot drift away from what ships. This file adds what a comparison
needs and a run should not pay for: a per-tick trace of where the hands go, how close they
come to the object, and what the object is touching, so a candidate's tray travel can be
attributed to a segment instead of merely observed.

Why the object exists: commanding the default posture as soon as the exit stage begins made
`Runtime.step` interpolate in joint space, and the open hands swept the tray 12.0 mm against
a 5 mm limit (reports/p1-h-seq-07). Two guessed fixes measured worse (48.7 mm and 197.0 mm,
which knocked the tray off the table: p1-h-seq-05, -06). Hence measuring variants in one
place before choosing one.

Run with each variant so the table stays reproducible:

    python experiments/diag_exit_path.py --run-id p1-h-exitpath-line-04 --path line
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import sys
import time

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src'))
sys.path.insert(0, str(ROOT / 'experiments'))

DURATION = 50.0
FINE_EVERY = 10          # per-tick trace inside the exit window
DIST_MAX = 0.5           # metres; anything farther apart is reported as DIST_MAX
V1_TRAY = 'tray_v1'


def main():
    import mujoco
    from humanoid007 import tray_task
    from humanoid007.runtime import Runtime, OPEN, GRASP, ExitPath, verify_assets
    import probe_h as sequence

    parser = argparse.ArgumentParser()
    parser.add_argument('--run-id', required=True)
    parser.add_argument('--path', required=True, choices=list(ExitPath.VARIANTS))
    parser.add_argument('--object', default=V1_TRAY)
    args = parser.parse_args()
    if Path(args.run_id).name != args.run_id:
        raise ValueError('Invalid run id')

    out = ROOT / 'reports' / args.run_id
    out.mkdir(parents=True, exist_ok=False)
    started = time.monotonic()
    report = dict(scope='DIAGNOSTIC_ONLY', probe='H-exit-path', path=args.path,
                  pid=os.getpid(), command=sys.argv, status='ERROR', samples=[],
                  exit_trace=[],
                  source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest())
    try:
        manifest = verify_assets()
        report['assets_verified'] = {'entries': len(manifest['sha256']),
                                     'unrecorded': len(manifest['unrecorded'])}
        world = ROOT / 'assets' / f'world_{args.object}.xml'
        if not world.is_file():
            raise FileNotFoundError(f'no world for {args.object!r}: {world}')
        r = Runtime(world)
        report['model_sha256'] = hashlib.sha256(world.read_bytes()).hexdigest()
        report['free_base'] = int(r.m.jnt_type[0]) == int(mujoco.mjtJoint.mjJNT_FREE)
        report['equality_constraints'] = int(r.m.neq)

        anchors = {side: np.array([.23, sign * .30, .85]) for side, sign in
                   [('left', 1), ('right', -1)]}
        default_arms = r.policy.default[r.arm_ids].copy()
        exit_path = ExitPath(r, anchors, default_arms, args.path)
        report['default_hands'] = {s: v.tolist()
                                   for s, v in exit_path.default_hands.items()}
        report['segments'] = [{'kind': s['kind'], 'seconds': s['seconds'],
                               **({'targets': {k: np.asarray(v).tolist()
                                               for k, v in s['targets'].items()}}
                                  if s['kind'] == 'cart' else {})}
                              for s in exit_path.segments()]
        report['exit_start_s'] = sequence.EXIT_START

        payload_joint = int(r.m.body_jntadr[r.m.body('payload').id])
        payload_dof = int(r.m.jnt_dofadr[payload_joint])
        left_geoms, right_geoms, payload_geoms = sequence._hand_geom_sets(r.m)
        side_geoms = {'left': sorted(left_geoms), 'right': sorted(right_geoms)}
        payload_geoms = sorted(payload_geoms)
        report['payload_z_initial'] = float(r.d.body('payload').xpos[2])
        report['object'] = {'name': args.object, 'is_v1_tray': args.object == V1_TRAY,
                            'mass_kg': round(float(r.m.body_mass[r.m.body('payload').id]), 6)}

        def nearest(side):
            """Closest approach between one hand's geoms and the tray's geoms."""
            best, pair = DIST_MAX, None
            for ga in side_geoms[side]:
                for gb in payload_geoms:
                    distance = float(mujoco.mj_geomDistance(r.m, r.d, ga, gb,
                                                            DIST_MAX, None))
                    if distance < best:
                        best, pair = distance, (ga, gb)
            return best, pair

        def body_contacts():
            """Bodies the tray is touching, per mujoco's own contact list."""
            names = []
            for index in range(r.d.ncon):
                first, second = int(r.d.contact[index].geom1), int(r.d.contact[index].geom2)
                for near, far in ((first, second), (second, first)):
                    if near in payload_geoms:
                        names.append(r.m.body(int(r.m.geom_bodyid[far])).name or '?')
            return sorted(set(names))

        exit_baseline = None
        arms = None
        previous_fine = -1
        while r.d.time < DURATION:
            if time.monotonic() - started > sequence.WALL_DEADLINE_S:
                report['status'] = 'WALL_TIMEOUT'
                break
            now = r.d.time
            if now < sequence.EXIT_START:
                arms, hands = sequence._stage_driver(r, anchors, now, default_arms,
                                                     OPEN, GRASP)
            else:
                if exit_baseline is None:
                    exit_baseline = r.d.body('payload').xpos.copy()
                    report['exit_start_hands'] = {s: list(v) for s, v in
                                                  r.snapshot()['hands'].items()}
                arms = exit_path.command(r, now - sequence.EXIT_START)
                hands = {s: OPEN for s in anchors}
            r.step(np.zeros(3), arms, hands, stationary=now > 2)

            if r.tick % sequence.SAMPLE_INTERVAL_TICKS == 0:
                row = r.snapshot()
                row['payload'] = r.d.body('payload').xpos.tolist()
                row['payload_speed'] = float(np.linalg.norm(
                    r.d.qvel[payload_dof:payload_dof + 3]))
                counts = {'left': 0, 'right': 0}
                for name in body_contacts():
                    if name.startswith('lh_'):
                        counts['left'] += 1
                    elif name.startswith('rh_'):
                        counts['right'] += 1
                row['hand_contacts'] = counts
                row['arm_dev_rad'] = float(np.max(np.abs(
                    r.d.qpos[r.qa[r.arm_ids]] - default_arms)))
                report['samples'].append(row)
                if row['tilt_deg'] > 35 or row['base'][2] < .65:
                    report['status'] = 'BODY_FALL'
                    break

            if now >= sequence.EXIT_START and r.tick - previous_fine >= FINE_EVERY:
                previous_fine = r.tick
                row = r.snapshot()
                # snapshot() does not carry arm_dev_rad; only the sampling path adds it.
                deviation = float(np.max(np.abs(
                    r.d.qpos[r.qa[r.arm_ids]] - default_arms)))
                payload_now = r.d.body('payload').xpos.copy()
                left_d, left_pair = nearest('left')
                right_d, right_pair = nearest('right')
                commanded = None
                if exit_path.hands_target is not None:
                    commanded = {s: np.asarray(v).tolist()
                                 for s, v in exit_path.hands_target.items()}
                report['exit_trace'].append(dict(
                    t=float(now),
                    segment=int(exit_path.segment),
                    hands={s: list(v) for s, v in row['hands'].items()},
                    target=commanded,
                    payload=payload_now.tolist(),
                    payload_speed=float(np.linalg.norm(
                        r.d.qvel[payload_dof:payload_dof + 3])),
                    payload_shift_m=float(np.linalg.norm(payload_now[:2] - exit_baseline[:2])),
                    min_dist={'left': left_d, 'right': right_d},
                    nearest_pair={'left': left_pair, 'right': right_pair},
                    touching=body_contacts(),
                    arm_dev_rad=deviation,
                    arm_tracking_error=float(row['arm_tracking_error']),
                ))
        else:
            report['status'] = 'DIAGNOSTIC_COMPLETED'
        report['sim_seconds'] = float(r.d.time)

        driven = sequence._phases_driven(float(r.d.time))
        judged = [{'t': row['time'], 'payload_z': row['payload'][2],
                   'payload_x': row['payload'][0], 'payload_y': row['payload'][1],
                   'payload_speed': row['payload_speed'],
                   'hand_contacts': row['hand_contacts'],
                   'arm_dev_rad': row['arm_dev_rad']}
                  for row in report['samples']]
        result = tray_task.evaluate(judged, driven, report['payload_z_initial'])
        report['h_stages'] = result['stages']
        report['h_summary'] = tray_task.summary_line(result)
        report['full_H_acceptance'] = result['overall']

        if report['exit_trace']:
            trace = report['exit_trace']
            per_segment = {}
            for row in trace:
                key = str(row['segment'])
                per_segment[key] = max(per_segment.get(key, 0.0), row['payload_shift_m'])
            report['exit_final'] = {
                'payload_shift_m': trace[-1]['payload_shift_m'],
                'payload_speed_max': max(row['payload_speed'] for row in trace),
                'min_approach_m': min(min(row['min_dist'].values()) for row in trace),
                'min_arm_dev_rad': min(row['arm_dev_rad'] for row in trace),
                'final_arm_dev_rad': trace[-1]['arm_dev_rad'],
                'ik_worst_tracking_error_m': max(row['arm_tracking_error'] for row in trace),
                'duration_s': trace[-1]['t'] - trace[0]['t'],
                'payload_shift_by_segment_m': per_segment,
            }
    except Exception as exc:
        report['error'] = f'{type(exc).__name__}: {exc}'
        if report.get('status') in (None, 'ERROR'):
            report['status'] = 'POST_RUN_ERROR'
    finally:
        report['wall_seconds'] = time.monotonic() - started
        (out / 'report.json').write_text(json.dumps(report, indent=2) + '\n')
        head = {k: v for k, v in report.items()
                if k not in ('samples', 'exit_trace', 'h_stages', 'command')}
        print(json.dumps(head, indent=2, ensure_ascii=False), flush=True)
        for line in report.get('h_summary') or []:
            print('  ' + line, flush=True)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
