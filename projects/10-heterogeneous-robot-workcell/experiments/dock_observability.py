"""W3.3: the observability gate. Does the vehicle's EXISTING sensor set observe what DOCK needs?

WHY A GATE AND NOT AN OPINION
-----------------------------
The guidance's rule is that two range lines do not automatically give a planar pose, and that the
symbols, frames, blind zones and lateral/yaw coupling must be checked STATICALLY before anything is
built on them. This script does that with the project's own geometry:

  1. put the vehicle at the dock pose (measurement showed it IS the parked pose);
  2. cast the lidar's own fan at its own height through the ACTUAL assembled world;
  3. perturb the vehicle over a grid of (dx, dy, dtheta) and re-cast;
  4. ask three questions: does the perturbation change the scan at all; is the map from pose to scan
     INJECTIVE (are there two different poses with the same scan); and what is the SMALLEST step the
     noise floor can still see -- which is the resolution, and the only number that can be compared
     with the millimetre window C measured.

Two mistakes were made and fixed while writing this, and both are recorded in the output because
they are the reason to distrust a gate's first result:

  * the first version read the ray ORIGIN once, outside the grid, so every perturbation moved the
    body and the rays did not. Every sensitivity came out EXACTLY 0.0, and the uniform zero is the
    tell that a check cannot fail;
  * the second version reported "dx 15x" -- a ratio of scan changes taken at a 50 mm step, which
    says a 50 mm shift is detectable and NOT that the sensor resolves 2 mm. Quoting a ratio taken at
    a coarse step as a resolution is the same error as calling a typed constant derived.
"""
import argparse
import hashlib
import itertools
import json
import math
from pathlib import Path
import sys

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'experiments'))
sys.path.insert(0, str(ROOT / 'src'))

import mujoco  # noqa: E402
import build_p3_world  # noqa: E402

CELL = ROOT / 'assets' / 'world_p3_cell.xml'
SWEEP = ROOT / 'reports' / 'p2-c-dock-sweep-01' / 'report.json'
RAYS = 181
ANGLE_MIN_DEG, ANGLE_MAX_DEG = -90.0, 90.0
RANGE_MAX_M = 12.0
#: The declared noise floor of ONE range return. A sensitivity below this cannot be measured at all,
#: which is why it is the unit the grid has to beat.
NOISE_FLOOR_M = 0.005
DX_GRID_M = (-0.05, -0.02, -0.005, 0.0, 0.005, 0.02, 0.05)
DY_GRID_M = (-0.05, -0.02, -0.005, 0.0, 0.005, 0.02, 0.05)
DTHETA_GRID_DEG = (-1.0, -0.2, -0.05, 0.0, 0.05, 0.2, 1.0)


def find_lidar(model):
    for kind, count, obj in (('site', model.nsite, mujoco.mjtObj.mjOBJ_SITE),
                             ('geom', model.ngeom, mujoco.mjtObj.mjOBJ_GEOM)):
        for index in range(count):
            name = mujoco.mj_id2name(model, obj, index)
            if name and 'lidar' in name:
                return kind, index, name
    return None, None, None


def body_yaw(model, data):
    """The chassis' yaw from its own rotation matrix: the fan must turn WITH the vehicle, or a yaw
    perturbation would be measured as a translation."""
    matrix = np.asarray(data.body(model.body('n_base_link').id).xmat).reshape(3, 3)
    return math.atan2(float(matrix[1, 0]), float(matrix[0, 0]))


def scan(model, data, origin, yaw, *, rays=RAYS, range_max=RANGE_MAX_M):
    """One horizontal fan from `origin`, turned to `yaw`, through the actual world. No truth field
    is read anywhere: this is the sensor the vehicle really has."""
    angles = np.radians(np.linspace(ANGLE_MIN_DEG, ANGLE_MAX_DEG, rays)) + yaw
    directions = np.stack([np.cos(angles), np.sin(angles), np.zeros_like(angles)], axis=1)
    geomid = np.zeros(1, dtype=np.int32)
    out = np.empty(rays)
    for index in range(rays):
        distance = mujoco.mj_ray(model, data, np.asarray(origin, dtype=float), directions[index],
                                 None, 1, -1, geomid)
        out[index] = range_max if distance < 0 else min(float(distance), range_max)
    return out


def signature(returns, *, digits=3):
    return tuple(np.round(returns, digits))


def set_pose(model, data, home_qpos, *, dx, dy, dtheta_deg):
    """Move the CHASSIS by its own unlimited slides -- a static placement, declared as such.

    Only the chassis' qpos entries are touched, and only for this measurement: this claims nothing
    about transport, and `probe_w2_baseline.py` is what shows the wheels move it.
    """
    data.qpos[:] = home_qpos
    for joint, value in (('n_slide_x', dx), ('n_slide_y', dy)):
        index = model.joint(joint).id
        data.qpos[model.jnt_qposadr[index]] = value
    yaw = model.joint('n_yaw').id
    data.qpos[model.jnt_qposadr[yaw]] = math.radians(dtheta_deg)
    mujoco.mj_forward(model, data)


def measured_window():
    """C's own docking boundary, read from its sweep report rather than typed.

    The sweep is a set of discrete points, so what it establishes is a boundary BETWEEN the largest
    passing step and the smallest failing one -- not a continuous envelope.
    """
    doc = json.loads(SWEEP.read_text(encoding='utf-8'))
    passing, failing = [], []
    for row in doc['rows']:
        (passing if row['reached_received'] else failing).append(row)
    positive = max([r['dx'] for r in passing if r['dx'] > 0], default=None)
    negative = min([r['dx'] for r in passing if r['dx'] < 0], default=None)
    first_fail_positive = min([r['dx'] for r in failing if r['dx'] > 0], default=None)
    first_fail_negative = max([r['dx'] for r in failing if r['dx'] < 0], default=None)
    yaw_pass = max([r['dtheta_deg'] for r in passing if r['dtheta_deg'] > 0], default=0.0)
    yaw_fail = min([r['dtheta_deg'] for r in failing if r['dtheta_deg'] > 0], default=None)
    return {'largest_passing_dx_m': positive, 'first_failing_dx_m': first_fail_positive,
            'last_passing_negative_dx_m': negative,
            'first_failing_negative_dx_m': first_fail_negative,
            'largest_passing_yaw_deg': yaw_pass, 'first_failing_yaw_deg': yaw_fail,
            'source': str(SWEEP.relative_to(ROOT)),
            'basis': 'discrete sweep points: the true boundary lies between the last passing and the '
                     'first failing step, so this is a bracket, not an envelope'}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--run-id', default='w3-observability-01')
    ap.add_argument('--reports-dir', default=None)
    args = ap.parse_args()
    out = (Path(args.reports_dir) if args.reports_dir else ROOT / 'reports') / args.run_id
    out.mkdir(parents=True, exist_ok=True)
    checks = []

    def check(name, status, detail):
        checks.append({'check': name, 'status': status, 'detail': detail})

    build_p3_world.install()
    import merge_world as mw
    model = mujoco.MjModel.from_xml_path(str(CELL))
    data = mujoco.MjData(model)
    home_qpos = np.asarray(mw.merged_home(model)[0], dtype=float)
    set_pose(model, data, home_qpos, dx=0.0, dy=0.0, dtheta_deg=0.0)

    kind, index, name = find_lidar(model)
    if kind is None:
        check('the vehicle carries a lidar', 'FAIL', 'no site or geom named *lidar* in the cell')
        verdict = 'FAIL'
        (out / 'report.json').write_text(json.dumps(
            {'run_id': args.run_id, 'verdict': verdict, 'checks': checks}, indent=2) + '\n',
            encoding='utf-8')
        print(json.dumps({'run_id': args.run_id, 'verdict': verdict}, indent=2))
        return 1

    def sensor_pose():
        return (np.asarray(data.site_xpos[index]).copy() if kind == 'site'
                else np.asarray(data.geom_xpos[index]).copy())

    check('the vehicle carries a lidar, and the fan is re-read after every perturbation', 'PASS',
          f'{kind} {name!r} at world z {sensor_pose()[2]:.4f}; fan {RAYS} rays over '
          f'[{ANGLE_MIN_DEG}, {ANGLE_MAX_DEG}] deg, range_max {RANGE_MAX_M} m, noise floor '
          f'{NOISE_FLOOR_M * 1000:.0f} mm per return. The first version of this gate read the origin '
          f'once, outside the grid, and every sensitivity came out exactly zero.')

    scans = {}
    for dx in DX_GRID_M:
        for dy in DY_GRID_M:
            for dtheta in DTHETA_GRID_DEG:
                set_pose(model, data, home_qpos, dx=dx, dy=dy, dtheta_deg=dtheta)
                scans[(dx, dy, dtheta)] = signature(scan(model, data, sensor_pose(),
                                                         body_yaw(model, data)))
    baseline = scans[(0.0, 0.0, 0.0)]
    largest = max(float(np.max(np.abs(np.array(sig) - np.array(baseline))))
                  for sig in scans.values())
    check('the perturbation grid actually changed the scan',
          'PASS' if largest > 0.0 else 'FAIL',
          f'largest scan change anywhere in the grid {largest * 1000:.1f} mm over a range of 50 mm '
          f'of travel and 1 deg of yaw. Zero here would mean the whole gate is measuring nothing.')

    collisions = {}
    for key_a, key_b in itertools.combinations(sorted(scans), 2):
        if scans[key_a] == scans[key_b]:
            changed = tuple(('dx', 'dy', 'dtheta')[i] for i in range(3) if key_a[i] != key_b[i])
            collisions.setdefault(changed, []).append((key_a, key_b))
    check('no two distinct poses produce the same scan signature (the pose-to-scan map is injective)',
          'PASS' if not collisions else 'FAIL',
          'every distinct pose in the grid produced a distinct scan signature'
          if not collisions else
          f'{sum(len(v) for v in collisions.values())} colliding pairs over the grid '
          f'{sorted(collisions)}; a collision means those poses are indistinguishable FROM THIS '
          f'SENSOR and no controller can recover the difference')

    # ---- the decisive question: what is the SMALLEST change this sensor can see? ----
    resolution = {}
    for axis, grid, unit in (('dx', DX_GRID_M, 'm'), ('dy', DY_GRID_M, 'm'),
                             ('dtheta', DTHETA_GRID_DEG, 'deg')):
        steps = []
        for value in sorted({abs(v) for v in grid} - {0.0}):
            key = {'dx': (value, 0.0, 0.0), 'dy': (0.0, value, 0.0),
                   'dtheta': (0.0, 0.0, value)}[axis]
            change = float(np.max(np.abs(np.array(scans[key]) - np.array(baseline))))
            steps.append({'step': value, 'unit': unit, 'scan_change_m': change,
                          'clears_noise_floor': change > NOISE_FLOOR_M})
        audible = [s['step'] for s in steps if s['clears_noise_floor']]
        resolution[axis] = {'steps': steps,
                            'smallest_detectable': min(audible) if audible else None}
    check('the grid is fine enough to measure a resolution at all',
          'PASS' if resolution['dx']['smallest_detectable'] is not None else 'FAIL',
          'smallest step whose scan change clears the noise floor, per quantity: '
          + ', '.join(f"{axis} {resolution[axis]['smallest_detectable']}" for axis in resolution))

    window = measured_window()
    tight = abs(window['last_passing_negative_dx_m'] or 0.0)
    best_dx = resolution['dx']['smallest_detectable']
    ok = best_dx is not None and best_dx <= tight
    if best_dx is None:
        because = 'no longitudinal step in the grid clears the noise floor'
    elif ok:
        because = ('a dock that far out is still distinguishable from one on centre, so the tight '
                   'side of C\'s window is inside what this sensor can see')
    else:
        because = ('a dock 2 mm out CANNOT be distinguished from one on centre by ranging alone: the '
                   'coarse stage can be told "close" and the fine stage cannot be certified. That is '
                   'a stop-and-decide -- a guiding chamfer, a compliance mechanism, or a sensor that '
                   'measures the interface itself')
    detail = ('the smallest longitudinal step whose scan change clears the '
              f'{NOISE_FLOOR_M * 1000:.0f} mm noise floor is '
              + (f'{best_dx * 1000:.1f} mm' if best_dx is not None else 'not reached')
              + "; C's own sweep brackets its window at +" + f"{window['largest_passing_dx_m'] * 1000:.0f}"
              + ' mm / ' + f"{window['last_passing_negative_dx_m'] * 1000:.0f}" + ' mm longitudinal'
              + ' (first failures at +' + f"{window['first_failing_dx_m'] * 1000:.0f}" + ' mm / '
              + f"{window['first_failing_negative_dx_m'] * 1000:.0f}" + ' mm) and '
              + f"{window['largest_passing_yaw_deg']:.2f}" + ' deg of yaw, from '
              + f"{window['source']}. " + because)
    check('the sensor resolves the TIGHT side of the window C measured',
          'PASS' if ok else 'FAIL', detail)

    verdict = 'FAIL' if any(c['status'] == 'FAIL' for c in checks) else 'PASS'
    report = {'run_id': args.run_id, 'scope': 'W3_DOCK_OBSERVABILITY', 'verdict': verdict,
              'checks': checks, 'sensor': {'kind': kind, 'name': name,
                                          'origin_z_m': float(sensor_pose()[2])},
              'noise_floor_m': NOISE_FLOOR_M, 'resolution': resolution,
              'collisions': {','.join(k): v[:3] for k, v in collisions.items()},
              'measured_window': window,
              'grid': {'dx_m': list(DX_GRID_M), 'dy_m': list(DY_GRID_M),
                       'dtheta_deg': list(DTHETA_GRID_DEG), 'poses': len(scans)},
              'world_sha256': hashlib.sha256(CELL.read_bytes()).hexdigest(),
              'not_established': [
                  'anything dynamic: a static fan at one pose says nothing about the scan while the '
                  'vehicle rolls or the deck slides',
                  'injectivity in general: the grid proves it on ITS steps, and the true boundary '
                  'between the last passing and first failing step is a bracket',
                  'a real sensor: the vehicle has no camera, and P3\'s vision camera was a probe-side '
                  'addition calibrated with simulator geom labels',
                  'the rotation-centre conversion: C\'s sweep turned the chassis about ITS centre, and '
                  'the interface offset between that centre and the roller contact line is not '
                  'quantified, so its angular boundary cannot be quoted as an interface angle',
              ]}
    (out / 'report.json').write_text(json.dumps(report, indent=2) + '\n', encoding='utf-8')
    print(json.dumps({'run_id': args.run_id, 'verdict': verdict,
                      'failed': [c['check'] for c in checks if c['status'] == 'FAIL']}, indent=2))
    for c in checks:
        print(f"  [{c['status']}] {c['check']}")
        print(f'        {c["detail"]}')
    print(f'\nwrote {out.relative_to(ROOT)}/report.json')
    return 0 if verdict == 'PASS' else 1


if __name__ == '__main__':
    sys.exit(main())
