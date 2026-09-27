"""Measure the judge's arrival quantities on EVERY existing run, before changing anything.

The question: `at_distance` (distance at the instant nav2 reported success) printed the same
number as `final_distance` in 7 of 7 runs. Two possible reasons, and they mean different things:

  (a) `result_sim_s` lies at or past the last truth sample, so `truth_at` CLAMPS and returns the
      final pose -- the quantity never measured anything of its own; or
  (b) the run really is at rest by the time the result arrives, so both numbers are the same
      physical fact measured twice.

This script separates the two, and also computes the quantities that WOULD carry information:
the minimum distance ever reached and when, the first time truth entered the declared tolerance
(the latch proxy), and the drift from there to the end.
"""
import json
import math
import pathlib
import sys

ROOT = pathlib.Path('/home/ziling/projects/010_heterogeneous_robot_workcell')
sys.path.insert(0, str(ROOT / 'experiments'))

import evaluate_n_nav as ev  # noqa: E402

import yaml  # noqa: E402

RUNS = ['p1-n-nav-06', 'p1-n-nav-07', 'p1-n-nav-09', 'p1-n-nav-10', 'p1-n-nav-11',
        'p1-n-nav-12', 'p1-n-nav-13', 'p1-n-nav-14']


def goal_checker_of(params_path):
    """Everything the arrival number depends on, read out of the params file the run named."""
    doc = yaml.safe_load(pathlib.Path(params_path).read_text(encoding='utf-8'))
    node = doc['controller_server']['ros__parameters']
    plugins = list(node.get('goal_checker_plugins') or [])
    out = {'plugins': plugins}
    for name in plugins:
        block = node.get(name) or {}
        out[name] = {'stateful': block.get('stateful', False),
                     'xy': block.get('xy_goal_tolerance'),
                     'yaw': block.get('yaw_goal_tolerance')}
    return out


print(f'{"run":<15} {"result_s":>9} {"settled_s":>9} {"last_smp":>9} {"n":>5} '
      f'{"at_dist":>8} {"final":>8} {"min_ever":>9} {"t_min":>7} {"1st_in":>8} '
      f'{"d@1st":>7} {"drift":>7}  clamped')
for name in RUNS:
    out = ROOT / 'reports' / name
    plan = json.loads((out / 'nav_plan.json').read_text())
    nav = json.loads((out / 'nav_report.json').read_text())
    sim = json.loads((out / 'sim_report.json').read_text())
    samples = sim.get('truth_samples') or []
    strategy = nav.get('action') or {}
    goal = plan['goal']
    params = ROOT / plan['params']['path']
    gc = goal_checker_of(params)
    log = gc[gc['plugins'][0]]
    tol = log['xy']

    result_s = strategy.get('result_sim_s')
    last_s = samples[-1][0] if samples else None
    clamped = (result_s is not None and last_s is not None and result_s >= last_s)
    at = ev.truth_at(samples, result_s) if result_s is not None else None
    at_d = math.hypot(at[0] - goal['x'], at[1] - goal['y']) if at else None
    fp = samples[-1][1:]
    fin = math.hypot(fp[0] - goal['x'], fp[1] - goal['y'])
    dists = [(row[0], math.hypot(row[1] - goal['x'], row[2] - goal['y'])) for row in samples]
    t_min, min_d = min(dists, key=lambda kv: kv[1])
    inside = [(t, d) for t, d in dists if d <= tol]
    first = inside[0] if inside else None
    print(f'{name:<15} {result_s if result_s is None else round(result_s, 3):>9} '
          f'{strategy.get("settled_sim_s"):>9} {last_s:>9.3f} {len(samples):>5} '
          f'{at_d if at_d is None else round(at_d, 4):>8} {fin:>8.4f} {min_d:>9.4f} '
          f'{t_min:>7.2f} '
          f'{(first[0] if first else float("nan")):>8.2f} '
          f'{(first[1] if first else float("nan")):>7.4f} '
          f'{fin - (first[1] if first else float("nan")):>7.4f}  {clamped}')

print()
print('goal checker as the runs actually configured it (derived from each plan\'s params path):')
for name in RUNS:
    plan = json.loads((ROOT / 'reports' / name / 'nav_plan.json').read_text())
    gc = goal_checker_of(ROOT / plan['params']['path'])
    plug = gc['plugins'][0]
    print(f"  {name:<15} {plug:<22} stateful={gc[plug]['stateful']!s:<5} "
          f"xy={gc[plug]['xy']} yaw={gc[plug]['yaw']}  params={plan['params']['path']}")
    declared = plan.get('goal_tolerance') or {}
    print(f"{'':>18} plan DECLARES xy_m={declared.get('xy_m')} yaw_rad={declared.get('yaw_rad')} "
          f"stateful={declared.get('stateful', '<not recorded>')}")
