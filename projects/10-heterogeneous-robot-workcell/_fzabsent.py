"""Which check catches an ABSENT freeze entry -- the new guard, or an older path?

The duplicate case reported itself clearly. The absent case returned rc=1 with no `coverage:` line,
so something else caught it first. Knowing WHICH is the difference between "I added a guard" and
"I added a guard that runs".
"""
import subprocess
from pathlib import Path

ROOT = Path('/home/ziling/projects/010_heterogeneous_robot_workcell')
P = ROOT / 'experiments' / 'make_p3_freeze.py'
PY = str(ROOT / '.venv' / 'bin' / 'python')

original = P.read_text(encoding='utf-8')
old = "    'experiments/probe_p4_holddepth.py',\n"
assert old in original
P.write_text(original.replace(old, old + "    'experiments/probe_does_not_exist.py',\n", 1),
             encoding='utf-8')
try:
    r = subprocess.run([PY, str(P), '--check'], capture_output=True, text=True, cwd=ROOT)
    out = (r.stdout + r.stderr).splitlines()
    print(f'rc={r.returncode}  lines={len(out)}')
    for ln in out:
        if any(k in ln for k in ('coverage', 'freeze', 'Traceback', 'Error', 'does_not_exist',
                                 'absent', 'missing')):
            print('  ', ln[:200])
    print('--- last 6 lines ---')
    for ln in out[-6:]:
        print('  ', ln[:200])
finally:
    P.write_text(original, encoding='utf-8')

r = subprocess.run([PY, str(P), '--check'], capture_output=True, text=True, cwd=ROOT)
print(f'reverted rc={r.returncode}  {(r.stdout + r.stderr).strip().splitlines()[-1][:160]}')
