"""Bounded P1 mechanics diagnostic; never an autonomous vision acceptance test.

Two modes:

  --duration N [--lift]   settle (and optionally lift a fixed fixture). Diagnostic only.
  --sequence              drive the six H stages end to end:
                          approach -> grasp -> lift -> hold -> place -> release -> exit

Every H verdict is computed by `humanoid007.tray_task` from recorded samples. The old
hard-coded "NOT_RUN: criteria not implemented" string is gone: a stage the run did not
drive is NOT_RUN because the timeline says so, and a stage that was driven can fail.

Scope note: the object is the handled tray carried over from project 007, not the V1
tray model. `full_H_acceptance` therefore stays NOT_RUN even when all six stages pass.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src'))

#: On an idle machine the physics runs about 5x faster than the wall clock (p1-h-seq-01:
#: 50.00 s sim in 9.76 s wall). The 20.00 s sim / 20.09 s wall pair in p1-h-lift-01 was
#: measured while project 009 held the CPU, so it is a loaded-machine figure, not a
#: baseline. Budgets below are in sim seconds; the internal wall deadline stays at 90 s
#: and the external timeout at 120 s.
SEQUENCE_SECONDS = 50.0
MAX_DURATION_S = 60.0
WALL_DEADLINE_S = 90.0

#: Which sim-second window drives each stage. A stage counts as *driven* only when the
#: run reached the end of its window; that is what makes NOT_RUN honest.
PHASE_WINDOWS = {
    'grasp':   (9.5, 13.5),
    'lift':    (13.5, 22.0),
    'hold':    (22.0, 25.0),
    'place':   (25.0, 33.5),
    'release': (33.5, 37.0),
    'exit':    (41.0, 49.0),
}

#: The object already inline in assets/combined.xml, and the object this round builds.
DEFAULT_OBJECT = 'payload_007'
V1_TRAY = 'tray_v1'

#: Sim second at which `ExitPath` takes over from the driver's hold.
EXIT_START = PHASE_WINDOWS['exit'][0]

APPROACH_START = 2.0
LIFT_HEIGHT_M = 0.12
SAMPLE_INTERVAL_TICKS = 50


def _hand_geom_sets(model):
    left, right, payload = set(), set(), set()
    for geom in range(model.ngeom):
        body = model.body(int(model.geom_bodyid[geom])).name or ''
        if body.startswith('lh_'):
            left.add(geom)
        elif body.startswith('rh_'):
            right.add(geom)
        elif body == 'payload':
            payload.add(geom)
    return left, right, payload


def _stage_driver(runtime, anchors, now, default_arms, open_hand, grasp_hand,
                  exit_path=None):
    """Control targets for the six-stage sequence, as a function of sim time.

    Returns (arm_joint_targets, hand_targets). `None` arms leaves the policy's own
    posture alone, which is what the settle phase wants.
    """
    def ramp(start, end):
        return float(min(max((now - start) / (end - start), 0.0), 1.0))

    def reach(height):
        return runtime.arm_ik({side: point + [0.0, 0.0, height]
                               for side, point in anchors.items()})

    if now < APPROACH_START:
        return None, None
    if now < PHASE_WINDOWS['grasp'][0]:
        return reach(0.0), {side: open_hand for side in anchors}
    if now < PHASE_WINDOWS['grasp'][1]:
        blend = ramp(*PHASE_WINDOWS['grasp'])
        return reach(0.0), {side: open_hand + blend * (grasp_hand - open_hand)
                            for side in anchors}
    if now < PHASE_WINDOWS['lift'][1]:
        return reach(LIFT_HEIGHT_M * ramp(*PHASE_WINDOWS['lift'])), \
               {side: grasp_hand for side in anchors}
    if now < PHASE_WINDOWS['hold'][1]:
        return reach(LIFT_HEIGHT_M), {side: grasp_hand for side in anchors}
    if now < PHASE_WINDOWS['place'][1]:
        return reach(LIFT_HEIGHT_M * (1.0 - ramp(*PHASE_WINDOWS['place']))), \
               {side: grasp_hand for side in anchors}
    if now < PHASE_WINDOWS['release'][1]:
        blend = ramp(*PHASE_WINDOWS['release'])
        return reach(0.0), {side: grasp_hand + blend * (open_hand - grasp_hand)
                            for side in anchors}
    if now < PHASE_WINDOWS['exit'][0]:
        return reach(0.0), {side: open_hand for side in anchors}
    # Exit: hand over to ExitPath, open-handed.
    #
    # Commanding `default_arms` here instead made the rate limiter interpolate in joint
    # space, sweeping the open hands through the tray: 12.0 mm against a 5 mm limit
    # (reports/p1-h-seq-07). A Cartesian line alone kept the tray still but left the arm
    # 0.284 rad off the posture (reports/p1-h-exitpath-line-04). ExitPath does both stages
    # and is the object the measurements selected.
    if exit_path is None:
        return default_arms, {side: open_hand for side in anchors}
    return (exit_path.command(runtime, now - EXIT_START),
            {side: open_hand for side in anchors})


def _phases_driven(reached_sim_seconds):
    return {name for name, (_, end) in PHASE_WINDOWS.items() if reached_sim_seconds >= end}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--run-id', required=True)
    parser.add_argument('--duration', type=float, default=6)
    parser.add_argument('--lift', action='store_true')
    parser.add_argument('--sequence', action='store_true',
                        help=f'drive the six H stages over {SEQUENCE_SECONDS:.0f} sim s')
    parser.add_argument('--object', default=DEFAULT_OBJECT,
                        help='object fragment in assets/objects/ to place in the world; '
                             f'{DEFAULT_OBJECT} is the one already inside combined.xml')
    parser.add_argument('--exit-path', default=None,
                        help='which measured exit path variant to drive; the list and the '
                             'measured default come from ExitPath and are applied after the '
                             'runtime import. The comparison that chose the default is '
                             'reports/p1-h-exitpath-*-04')
    args = parser.parse_args()
    if args.sequence:
        args.duration = SEQUENCE_SECONDS
    if not 0 < args.duration <= MAX_DURATION_S or Path(args.run_id).name != args.run_id:
        raise ValueError('Invalid bounded run arguments')
    out = ROOT / 'reports' / args.run_id
    out.mkdir(parents=True, exist_ok=False)
    started = time.monotonic()
    report = dict(scope='DIAGNOSTIC_ONLY', probe='H', pid=os.getpid(),
                  command=sys.argv, status='ERROR', samples=[],
                  mode='sequence' if args.sequence else ('lift' if args.lift else 'settle'),
                  source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest())
    try:
        import mujoco
        import numpy as np
        from humanoid007 import tray_task
        from humanoid007.runtime import Runtime, OPEN, GRASP, ExitPath, verify_assets

        # The variant list lives in one place, runtime.ExitPath. That import is inside this
        # guard on purpose (a missing dependency must still write a report), so the option
        # is declared with the other options and validated here instead of through argparse
        # `choices` -- declaring it late hid it from --help and forced a second parse that
        # reset --duration, which is exactly the bug p1-h-seq-08/09 hit.
        args.exit_path = ExitPath.resolve_variant(args.exit_path)
        report['exit_path'] = args.exit_path

        # Assets first. A missing or drifted model makes every later number fiction.
        try:
            manifest = verify_assets()
        except Exception as exc:
            report['status'] = 'ASSET_MISMATCH'
            report['error'] = f'asset check refused the run: {type(exc).__name__}: {exc}'
            raise
        report['assets_verified'] = {
            'entries': len(manifest['sha256']),
            'not_carried_over': sorted(manifest.get('not_carried_over', {})),
        }

        # Which object this run is about is a property of the run, not of the code.
        world = (ROOT / 'assets/combined.xml' if args.object == DEFAULT_OBJECT
                 else ROOT / 'assets' / f'world_{args.object}.xml')
        fragment = ROOT / 'assets' / 'objects' / f'{args.object}.xml'
        if not world.is_file():
            raise FileNotFoundError(
                f'no world for object {args.object!r}: expected '
                f'{world.relative_to(ROOT)}. Build it with '
                f'experiments/make_world.py --object {args.object}')

        r = Runtime(world)
        report['mujoco'] = mujoco.__version__
        report['model_sha256'] = hashlib.sha256(world.read_bytes()).hexdigest()
        report['free_base'] = int(r.m.jnt_type[0]) == int(mujoco.mjtJoint.mjJNT_FREE)
        report['equality_constraints'] = int(r.m.neq)

        # Declared calibrated test fixture, not sensed object location.
        anchors = {side: np.array([.23, sign * .30, .85]) for side, sign in
                   [('left', 1), ('right', -1)]}
        payload_joint = int(r.m.body_jntadr[r.m.body('payload').id])
        payload_dof = int(r.m.jnt_dofadr[payload_joint])
        left_geoms, right_geoms, payload_geoms = _hand_geom_sets(r.m)
        default_arms = r.policy.default[r.arm_ids].copy()
        exit_path = (ExitPath(r, anchors, default_arms, args.exit_path)
                     if args.sequence else None)
        report['payload_z_initial'] = float(r.d.body('payload').xpos[2])
        object_body = int(r.m.body('payload').id)
        report['object'] = {
            'role': 'payload',
            'name': args.object,
            'world': str(world.relative_to(ROOT)),
            'world_sha256': hashlib.sha256(world.read_bytes()).hexdigest(),
            'fragment': str(fragment.relative_to(ROOT)) if fragment.is_file() else None,
            'fragment_sha256': (hashlib.sha256(fragment.read_bytes()).hexdigest()
                                if fragment.is_file() else None),
            'mass_kg': round(float(r.m.body_mass[object_body]), 6),
            'is_v1_tray': args.object == V1_TRAY,
        }

        arms = None
        while r.d.time < args.duration:
            if time.monotonic() - started > WALL_DEADLINE_S:
                report['status'] = 'WALL_TIMEOUT'
                break
            now = r.d.time
            if args.sequence:
                arms, hands = _stage_driver(r, anchors, now, default_arms, OPEN,
                                            GRASP, exit_path)
            else:
                if args.lift and now > 4 and r.tick % 25 == 0:
                    height = .12 * np.clip((now - 12) / 4, 0, 1)
                    arms = r.arm_ik({s: p + np.array([0, 0, height])
                                     for s, p in anchors.items()}, arms)
                fraction = np.clip((now - 8) / 3, 0, 1) if args.lift else 0
                hands = {s: OPEN + fraction * (GRASP - OPEN) for s in anchors}
            r.step(np.zeros(3), arms, hands, stationary=now > 2)
            if r.tick % SAMPLE_INTERVAL_TICKS == 0:
                row = r.snapshot()
                row['payload'] = r.d.body('payload').xpos.tolist()
                row['payload_speed'] = float(np.linalg.norm(r.d.qvel[payload_dof:payload_dof + 3]))
                row['contacts'] = int(r.d.ncon)
                counts = {'left': 0, 'right': 0}
                for index in range(r.d.ncon):
                    first, second = int(r.d.contact[index].geom1), int(r.d.contact[index].geom2)
                    for near, far in ((first, second), (second, first)):
                        if near in payload_geoms:
                            if far in left_geoms:
                                counts['left'] += 1
                            elif far in right_geoms:
                                counts['right'] += 1
                row['hand_contacts'] = counts
                row['arm_dev_rad'] = float(np.max(np.abs(
                    r.d.qpos[r.qa[r.arm_ids]] - default_arms)))
                report['samples'].append(row)
                if row['tilt_deg'] > 35 or row['base'][2] < .65:
                    report['status'] = 'BODY_FALL'
                    break
        else:
            report['status'] = 'DIAGNOSTIC_COMPLETED'
        report['sim_seconds'] = float(r.d.time)
        report['render'] = 'NOT_RUN'
        try:
            with mujoco.Renderer(r.m, height=120, width=160) as renderer:
                renderer.update_scene(r.d, camera='eyes')
                rgb = renderer.render()
                report['render'] = {'shape': list(rgb.shape), 'std': float(rgb.std())}
        except Exception as exc:
            report['render'] = {'error': str(exc)}

        # Judge. The samples are the evidence; the verdicts are derived from them.
        driven = _phases_driven(float(r.d.time)) if args.sequence else set()
        judged = [{'t': row['time'], 'payload_z': row['payload'][2],
                   'payload_x': row['payload'][0], 'payload_y': row['payload'][1],
                   'payload_speed': row['payload_speed'],
                   'hand_contacts': row['hand_contacts'],
                   'arm_dev_rad': row['arm_dev_rad']}
                  for row in report['samples']]
        result = tray_task.evaluate(judged, driven, report['payload_z_initial'])
        report['h_stages'] = result['stages']
        report['h_stages_driven'] = sorted(driven)
        report['h_summary'] = tray_task.summary_line(result)
        if result['failed'] or result['not_run']:
            report['full_H_acceptance'] = result['overall']
        elif args.object != V1_TRAY:
            report['full_H_acceptance'] = (
                f'NOT_RUN: all six stages passed, but on {args.object}, not the V1 tray '
                f'({V1_TRAY}); the V1 acceptance needs the V1 object')
        else:
            report['full_H_acceptance'] = (
                'DIAGNOSTIC_PASS: all six stages on the V1 tray, driven from a declared '
                'fixture pose with no vision and no pose perturbation; scope is '
                'DIAGNOSTIC_ONLY, so this is not the vision acceptance')
    except Exception as exc:
        # Record unconditionally. The first version only recorded while status was still
        # 'ERROR', so a failure after the run loop (the judging block, the render) left a
        # report that looked like a clean run with no verdicts at all and no error.
        report['error'] = f'{type(exc).__name__}: {exc}'
        if report.get('status') in (None, 'ERROR'):
            report['status'] = 'POST_RUN_ERROR'
    finally:
        report['wall_seconds'] = time.monotonic() - started
        (out / 'report.json').write_text(json.dumps(report, indent=2) + '\n')
        printable = {k: v for k, v in report.items() if k not in ('samples', 'h_stages')}
        print(json.dumps(printable, indent=2, ensure_ascii=False), flush=True)
        for line in report.get('h_summary') or []:
            print('  ' + line, flush=True)
    return 0 if report['status'] == 'DIAGNOSTIC_COMPLETED' else 1


if __name__ == '__main__':
    raise SystemExit(main())
