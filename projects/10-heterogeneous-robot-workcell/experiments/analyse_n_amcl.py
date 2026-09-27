"""Characterise a pose source against truth, in the coordinates that matter.

P1-N-07 measured the symptom: `amcl_pose` runs 0.24-0.31 m AHEAD of the truth along the route,
with the heading right to 0.013 rad, while wheel odometry is only 0.066 m out. "Ahead along the
route" is a statement about the projection of the error onto the path, and separating it from the
part perpendicular to the path is what discriminates the causes:

  * an **along-track** error is a distance/direction-of-travel error -- the estimate is
    systematically further along than the vehicle is, which is what a motion-model or an
    update-cadence problem looks like;
  * a **cross-track** error is a lateral error, which is what a map/model mismatch or a biased
    scan match looks like.

The same decomposition is applied to wheel odometry, because the question that decides where to
look next is whether AMCL *invents* the lead or *faithfully propagates an odometry that already
leads the truth*. AMCL's corner of the system cannot be blamed for an input error.

Read-only: it takes a run directory and prints numbers. Nothing here decides a verdict.
"""
import argparse
import json
import math
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'experiments'))

from evaluate_n_nav import path_length, percentile, truth_at, wrapped  # noqa: E402


def arc_lengths(samples):
    lengths = [0.0]
    for previous, current in zip(samples, samples[1:]):
        lengths.append(lengths[-1] + math.hypot(current[1] - previous[1],
                                                current[2] - previous[2]))
    return lengths


def closest_point_on_path(samples, lengths, pose):
    """The truth path point nearest `pose`: returns (offset_m, offset_t)."""
    best = None
    for index, row in enumerate(samples):
        distance = math.hypot(row[1] - pose[0], row[2] - pose[1])
        if best is None or distance < best[0]:
            best = (distance, index)
    index = best[1]
    return lengths[index], samples[index][0]


def analyse(pairs, samples, lengths, label):
    """pairs: [(stamp, x, y, yaw)]. Returns a summary dict, printing as it goes."""
    print('')
    print(f'--- {label} ---')
    print('   stamp     pose(x,y)            truth(x,y)         |e|    along    cross'
          '  lead_t   speed   dyaw')
    rows = []
    for stamp, x, y, yaw in pairs:
        reference = truth_at(samples, stamp)
        if reference is None:
            continue
        offset_m, offset_t = closest_point_on_path(samples, lengths, (x, y))
        dx, dy = x - reference[0], y - reference[1]
        along = dx * math.cos(reference[2]) + dy * math.sin(reference[2])
        cross = -dx * math.sin(reference[2]) + dy * math.cos(reference[2])
        window = [row for row in samples if stamp - 0.5 <= row[0] <= stamp]
        speed = (path_length(window) / (window[-1][0] - window[0][0])) if len(window) > 1 else 0.0
        rows.append({'stamp': stamp, 'error': math.hypot(dx, dy), 'along': along, 'cross': cross,
                     'lead_t': offset_t - stamp, 'speed': speed,
                     'yaw_error': wrapped(yaw - reference[2])})
        print(f'{stamp:8.2f}  ({x:+.3f},{y:+.3f})  ({reference[0]:+.3f},{reference[1]:+.3f})'
              f'  {rows[-1]["error"]:6.3f}  {along:+7.3f}  {cross:+7.3f}'
              f'  {offset_t - stamp:+7.3f}  {speed:6.3f}  {rows[-1]["yaw_error"]:+6.3f}')
    if not rows:
        print('  (no pairs)')
        return {'label': label, 'rows': [], 'summary': {}}
    summary = {
        'n': len(rows),
        'position_median_m': percentile([r['error'] for r in rows], 0.5),
        'position_p95_m': percentile([r['error'] for r in rows], 0.95),
        'along_median_m': percentile([abs(r['along']) for r in rows], 0.5),
        'along_p95_m': percentile([abs(r['along']) for r in rows], 0.95),
        'cross_median_m': percentile([abs(r['cross']) for r in rows], 0.5),
        'cross_p95_m': percentile([abs(r['cross']) for r in rows], 0.95),
        'yaw_median_rad': percentile([abs(r['yaw_error']) for r in rows], 0.5),
    }
    moving = [r for r in rows if r['speed'] > 0.05]
    if len(moving) >= 3:
        summary['lead_time_moving_median_s'] = percentile([r['lead_t'] for r in moving], 0.5)
        summary['lead_distance_moving_median_m'] = percentile(
            [r['speed'] * r['lead_t'] for r in moving], 0.5)
    print(f"  position   : median {summary['position_median_m']:.3f} m, "
          f"p95 {summary['position_p95_m']:.3f} m")
    print(f"  along-track: median |{summary['along_median_m']:.3f}| m, "
          f"p95 |{summary['along_p95_m']:.3f}| m   (positive = ahead of the truth)")
    print(f"  cross-track: median |{summary['cross_median_m']:.3f}| m, "
          f"p95 |{summary['cross_p95_m']:.3f}| m")
    print(f"  heading    : median |{summary['yaw_median_rad']:.3f}| rad")
    if 'lead_time_moving_median_s' in summary:
        print(f"  while moving: lead in time {summary['lead_time_moving_median_s']:+.3f} s, "
              f"lead x speed {summary['lead_distance_moving_median_m']:.3f} m")
    return {'label': label, 'rows': rows, 'summary': summary}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--run-id', required=True)
    parser.add_argument('--reports-dir', default=None)
    parser.add_argument('--label', default=None)
    parser.add_argument('--only', choices=['amcl', 'odom', 'both'], default='both')
    args = parser.parse_args()

    out = (Path(args.reports_dir) if args.reports_dir else ROOT / 'reports') / args.run_id
    sim = json.loads((out / 'sim_report.json').read_text())
    checker = json.loads((out / 'checker_nav_raw.json').read_text())
    samples = sim['truth_samples']
    lengths = arc_lengths(samples)
    amcl = checker['observed'].get('amcl_samples') or []
    odom = checker['observed'].get('odom_samples') or []

    print(f'=== {args.label or args.run_id} ===')
    print(f'truth: {len(samples)} samples, {samples[0][0]:.2f}..{samples[-1][0]:.2f} s sim, '
          f'{lengths[-1]:.3f} m travelled')
    print(f'amcl_pose samples: {len(amcl)};  odom samples: {len(odom)}')

    results = {}
    if args.only in ('odom', 'both'):
        results['odom'] = analyse(odom, samples, lengths,
                                  'wheel odometry vs truth (no AMCL involved)')
    if args.only in ('amcl', 'both'):
        results['amcl'] = analyse(amcl, samples, lengths, 'amcl_pose vs truth')
    if results.get('odom', {}).get('summary') and results.get('amcl', {}).get('summary'):
        om = results['odom']['summary']['position_median_m']
        am = results['amcl']['summary']['position_median_m']
        print('')
        print(f"VERDICT-INPUT: odometry median {om:.3f} m, AMCL median {am:.3f} m "
              f"-> AMCL is {am / max(om, 1e-6):.1f}x "
              f"{'better' if am < om else 'worse'} than its own odometry")

    (out / 'amcl_analysis.json').write_text(json.dumps(
        {'run_id': args.run_id, 'sources': results}, indent=2) + '\n')
    print(f"-> wrote {out / 'amcl_analysis.json'}")
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
