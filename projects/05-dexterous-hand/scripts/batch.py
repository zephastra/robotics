"""Run every specified trial; preserve failures, never select only successes."""
import argparse
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--count', type=int, default=5)
    p.add_argument('--offset', type=float, default=.002)
    args = p.parse_args()
    if not 1 <= args.count <= 100 or not 0 <= args.offset <= .01:
        p.error('count: 1..100; offset: 0..0.01 m')
    out = ROOT/'reports'/('batch-'+datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S%fZ'))
    out.mkdir(parents=True)
    results = []
    env = dict(os.environ, PYTHONPATH=str(ROOT/'src'), PYTHONDONTWRITEBYTECODE='1')
    for seed in range(args.count):
        command = [sys.executable, '-m', 'hand005.app', '--mode', 'demo', '--headless',
                   '--seed', str(seed), '--offset', str(args.offset), '--duration', '35']
        try:
            result = subprocess.run(command, cwd=ROOT, env=env, capture_output=True, text=True, timeout=60)
            (out/f'{seed}.log').write_text(result.stdout+result.stderr)
            paths = [line.removeprefix('Report: ') for line in result.stdout.splitlines() if line.startswith('Report: ')]
            report = json.loads((Path(paths[0])/'report.json').read_text()) if paths else {}
            results.append(dict(seed=seed, status=report.get('status', 'ERROR'),
                                reason=report.get('reason'), exit_code=result.returncode,
                                report=str(Path(paths[0]).relative_to(ROOT)) if paths else None))
        except subprocess.TimeoutExpired:
            results.append(dict(seed=seed, status='ERROR', reason='WALL_TIMEOUT'))
        print(results[-1], flush=True)
    summary = dict(offset_m=args.offset, trials=results,
                   passed=sum(r['status']=='COMPLETED' for r in results), total=len(results),
                   scope='Fixed sphere, mass and nominal friction; bounded initial xy offsets, not hardware/generalization validation')
    (out/'summary.json').write_text(json.dumps(summary, indent=2)+'\n')
    print(out/'summary.json')
    raise SystemExit(0 if summary['passed']==summary['total'] else 1)


if __name__ == '__main__':
    main()
