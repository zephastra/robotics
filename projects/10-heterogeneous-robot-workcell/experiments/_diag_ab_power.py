"""Is the stateful A/B worth running? Decide it from data, not from argument.

Two things decide it:
  1. WHERE the believed distance sits at rest. If nav2 already comes to rest inside its own
     0.15 m tolerance at this goal, then flipping `stateful` has nothing to remove: the latch's
     only cost is the distance travelled between latching (belief <= 0.15) and stopping, and that
     is bounded by (believed distance at rest) - 0.15, floored at 0.
  2. the run-to-run spread at a FIXED goal with a FIXED config, which is what an effect has to
     beat to be visible. Runs 09/12/13/14 are the four current-plant samples at (2.6, 2.4).
"""
import json
import math
import pathlib
import sys

ROOT = pathlib.Path('/home/ziling/projects/010_heterogeneous_robot_workcell')
sys.path.insert(0, str(ROOT / 'experiments'))
import evaluate_n_nav as ev  # noqa: E402

SAME_GOAL = ['p1-n-nav-09', 'p1-n-nav-12', 'p1-n-nav-13', 'p1-n-nav-14']   # current plant
TOL = 0.15
DISTANCE_LIMIT = 0.25

print('=== the believed pose over the last few samples (is the vehicle still closing?) ===')
for name in ['p1-n-nav-09', 'p1-n-nav-10'] + SAME_GOAL[1:]:
    out = ROOT / 'reports' / name
    plan = json.loads((out / 'nav_plan.json').read_text())
    sim = json.loads((out / 'sim_report.json').read_text())
    checker = json.loads((out / 'checker_nav_raw.json').read_text())
    amcl = checker['observed']['amcl_samples']
    goal = plan['goal']
    print(f'  {name}  goal=({goal["x"]}, {goal["y"]})')
    for stamp, x, y, _yaw in amcl[-5:]:
        print(f'      sim {stamp:6.2f} s  believed distance {math.hypot(x - goal["x"], y - goal["y"]):.4f} m')

print()
print('=== believed distance at rest, and the latch cost it implies ===')
costs = []
for name in SAME_GOAL:
    out = ROOT / 'reports' / name
    plan = json.loads((out / 'nav_plan.json').read_text())
    checker = json.loads((out / 'checker_nav_raw.json').read_text())
    goal = plan['goal']
    _s, x, y, _y = checker['observed']['amcl_samples'][-1]
    bel = math.hypot(x - goal['x'], y - goal['y'])
    cost = max(0.0, bel - TOL)
    costs.append(cost)
    print(f'  {name}: believed {bel:.4f} m -> latch cost <= {cost:.4f} m')

print()
print('=== run-to-run spread at the fixed goal (the noise an effect must beat) ===')
final = []
for name in SAME_GOAL:
    out = ROOT / 'reports' / name
    plan = json.loads((out / 'nav_plan.json').read_text())
    sim = json.loads((out / 'sim_report.json').read_text())
    goal = plan['goal']
    row = sim['truth_samples'][-1]
    final.append(math.hypot(row[1] - goal['x'], row[2] - goal['y']))
mean = sum(final) / len(final)
var = sum((v - mean) ** 2 for v in final) / (len(final) - 1)
sd = math.sqrt(var)
print('  samples: ' + ', '.join(f'{v:.4f}' for v in final))
print(f'  mean {mean:.4f} m, sample sd {sd:.4f} m ({sd * 1000:.1f} mm)')

predicted = sum(costs) / len(costs)
print()
print('=== verdict on the A/B ===')
print(f'  predicted effect of stateful:false at this goal : {predicted * 1000:.1f} mm')
print(f'  run-to-run sd at this goal                      : {sd * 1000:.1f} mm')
if predicted > 0:
    z = 1.959963985 + 0.841621234
    n = math.ceil(2 * z * z * sd * sd / (predicted ** 2))
    print(f'  samples needed per arm for 80% power            : {n}')
else:
    print('  samples needed per arm                          : the effect is 0, so no n detects it')
print(f'  budget headroom: median arrival {sorted(final)[len(final) // 2]:.3f} m against a '
      f'{DISTANCE_LIMIT} m limit')
