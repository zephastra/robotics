"""What was nav2 looking at when it decided it had arrived?

The truth data says the vehicle never entered 0.15 m of the goal in 8 of 8 runs -- `min_ever`
equals `final` equals 0.21..0.43 m, and the approach is monotonic. So the goal checker was
satisfied by something that is NOT the truth pose. The judge's own docstring already names the
candidate: nav2 compares the goal against `amcl_pose`, and a run once reported SUCCEEDED while
standing 0.43 m out "because AMCL believed it was 0.13 m away".

This measures, per run, the BELIEVED distance to the goal at the instant the action reported
success, the belief error at that same stamp, and the run's belief error distribution. If the
believed distance is inside the declared tolerance while the truth distance is not, then the
arrival number is a LOCALISATION number and the goal checker is doing what it was told.
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


def tol_of(plan):
    doc = yaml.safe_load((ROOT / plan['params']['path']).read_text(encoding='utf-8'))
    node = doc['controller_server']['ros__parameters']
    name = (node.get('goal_checker_plugins') or ['general_goal_checker'])[0]
    block = node.get(name) or {}
    return name, float(block.get('xy_goal_tolerance')), bool(block.get('stateful'))


print(f'{"run":<14} {"tol":>5} {"truth@res":>9} {"bel@res":>8} {"err@res":>8} '
      f'{"bel<=tol":>8} {"med_err":>8} {"p95_err":>8} {"yaw_err":>8} {"n_pair":>6}')
rows = []
for name in RUNS:
    out = ROOT / 'reports' / name
    plan = json.loads((out / 'nav_plan.json').read_text())
    nav = json.loads((out / 'nav_report.json').read_text())
    sim = json.loads((out / 'sim_report.json').read_text())
    checker = json.loads((out / 'checker_nav_raw.json').read_text())
    samples = sim.get('truth_samples') or []
    amcl = (checker.get('observed') or {}).get('amcl_samples') or []
    goal = plan['goal']
    plug, tol, stateful = tol_of(plan)
    result_s = (nav.get('action') or {}).get('result_sim_s')

    truth_at_res = ev.truth_at(samples, result_s)
    truth_d = math.hypot(truth_at_res[0] - goal['x'], truth_at_res[1] - goal['y'])

    # the amcl sample nearest the success instant
    if amcl:
        stamp, ax, ay, ayaw = min(amcl, key=lambda s: abs(s[0] - result_s))
        bel_d = math.hypot(ax - goal['x'], ay - goal['y'])
        ref = ev.truth_at(samples, stamp)
        err = math.hypot(ax - ref[0], ay - ref[1])
        yaw_err = ev.wrapped(ayaw - ref[2])
    else:
        stamp = bel_d = err = yaw_err = float('nan')

    errs = []
    for stamp_i, x, y, yaw in amcl:
        ref = ev.truth_at(samples, stamp_i)
        if ref is not None:
            errs.append(math.hypot(x - ref[0], y - ref[1]))
    med = ev.percentile(errs, 0.5)
    p95 = ev.percentile(errs, 0.95)

    print(f'{name:<14} {tol:>5.2f} {truth_d:>9.4f} {bel_d:>8.4f} {err:>8.4f} '
          f'{str(bel_d <= tol):>8} {med if med is None else round(med, 4):>8} '
          f'{p95 if p95 is None else round(p95, 4):>8} {yaw_err:>8.4f} {len(errs):>6}')
    rows.append((name, plug, stateful, truth_d, bel_d, err, med, p95))

print()
print('belief error at the success instant, vs the truth overshoot:')
for name, plug, stateful, truth_d, bel_d, err, med, p95 in rows:
    print(f'  {name:<14} truth {truth_d:.4f} - belief {bel_d:.4f} = {truth_d - bel_d:+.4f} m '
          f'| belief error {err:.4f} m | stateful={stateful}')

print()
print('is `truth - belief` explained by the belief error alone?  (it should be, if the belief is')
print('the only thing the goal checker saw and the vehicle stopped where it was told to):')
for name, plug, stateful, truth_d, bel_d, err, med, p95 in rows:
    print(f'  {name:<14} difference {truth_d - bel_d:+.4f} vs belief error {err:.4f} '
          f'-> ratio {(truth_d - bel_d) / err if err else float("nan"):+.3f}')
