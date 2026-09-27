"""Re-judge the P1-N runs with the CURRENT judge, into a NEW evidence id.

Why a new id instead of re-judging in place: a judge's `--run-id` is both "which run to read" and
"where to write the verdict" (`D042`), so re-judging in place destroys the verdict that was
recorded when the run happened. The originals keep their own acceptance files; this writes a
parallel set under `reports/<new-id>/cases/<run-id>/`.

What makes the parallel set comparable rather than a second opinion:

  * the five inputs are COPIED and their sha256 is asserted equal to the originals', so the two
    verdicts are about byte-identical evidence and any difference is the judge, not the run;
  * the earlier verdict is read from the original `reports/<run-id>/acceptance.json`, never typed;
  * the README and the summary are derived from the per-case results, so "7 of 8 unchanged" is
    computed rather than asserted.

Refuses to write into an existing id.
"""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
INPUTS = ('nav_plan.json', 'nav_report.json', 'sim_report.json', 'ros_report.json',
          'checker_nav_raw.json')
DEFAULT_RUNS = ('p1-n-nav-06', 'p1-n-nav-07', 'p1-n-nav-09', 'p1-n-nav-10', 'p1-n-nav-11',
                'p1-n-nav-12', 'p1-n-nav-13', 'p1-n-nav-14')
ARRIVAL = 'the vehicle arrived within tolerance of the goal, measured against truth'
BELIEF = 'nav2 stopped inside its own declared tolerance of the goal it believes in'


def sha256(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--run-id', required=True,
                        help='the NEW evidence id; must not already exist')
    parser.add_argument('--runs', nargs='*', default=list(DEFAULT_RUNS))
    parser.add_argument('--reason', default='the arrival row was decomposed and a belief row added '
                                            '(D062); the originals keep the verdicts recorded when '
                                            'they ran (D042)')
    args = parser.parse_args()
    if Path(args.run_id).name != args.run_id:
        raise ValueError('run id must be a bare name')

    out = ROOT / 'reports' / args.run_id
    if out.exists():
        raise SystemExit(f'{out} already exists; pick a new id (rerunning would replace verdicts)')
    cases = out / 'cases'

    summary = {'run_id': args.run_id, 'reason': args.reason, 'cases': {}}
    for source in args.runs:
        origin = ROOT / 'reports' / source
        if not (origin / 'nav_plan.json').is_file():
            raise SystemExit(f'{origin} is not a run directory')
        dest = cases / source
        dest.mkdir(parents=True)
        digests = {}
        for name in INPUTS:
            src, dst = origin / name, dest / name
            dst.write_bytes(src.read_bytes())
            # byte-identity is asserted, not assumed: this is what makes the two verdicts
            # comparable, and a copy that silently differs would make the comparison meaningless
            assert sha256(dst) == sha256(src), f'{name} copy differs for {source}'
            digests[name] = sha256(src)

        completed = subprocess.run(
            [sys.executable, str(ROOT / 'experiments' / 'evaluate_n_nav.py'),
             '--run-id', source, '--reports-dir', str(cases)],
            cwd=str(ROOT), capture_output=True, text=True)
        fresh = json.loads((dest / 'acceptance.json').read_text(encoding='utf-8'))
        before = json.loads((origin / 'acceptance.json').read_text(encoding='utf-8')) \
            if (origin / 'acceptance.json').is_file() else None
        rows = {item['check']: item for item in fresh['checks']}
        summary['cases'][source] = {
            'inputs_sha256': digests,
            'verdict_before': before['verdict'] if before else None,
            'verdict_now': fresh['verdict'],
            'changed': bool(before and before['verdict'] != fresh['verdict']),
            'judge_exit_code': completed.returncode,
            'arrival': rows.get(ARRIVAL),
            'belief': rows.get(BELIEF),
        }
        print(f'{source}: {summary["cases"][source]["verdict_before"]} -> '
              f'{fresh["verdict"]}  (judge rc={completed.returncode})')

    changed = [name for name, case in summary['cases'].items() if case['changed']]
    summary['changed_count'] = len(changed)
    summary['unchanged_count'] = len(summary['cases']) - len(changed)
    (out / 'summary.json').write_text(json.dumps(summary, indent=2) + '\n', encoding='utf-8')

    lines = [
        f'# Re-judged P1-N runs -- {args.run_id}', '',
        f'**{summary["unchanged_count"]} of {len(summary["cases"])} verdicts unchanged; '
        f'{summary["changed_count"]} changed: {changed or "none"}**', '',
        f'Why these exist: {args.reason}', '',
        'Each case is a byte-identical copy of the five input files of the run named, asserted by '
        'sha256, re-judged by `experiments/evaluate_n_nav.py`. The originals are untouched, so the '
        'two verdicts are comparable instead of one replacing the other.', '',
        '| run | verdict then | verdict now | changed |', '|---|---|---|---|',
    ]
    for name, case in summary['cases'].items():
        lines.append(f'| `{name}` | {case["verdict_before"]} | {case["verdict_now"]} | '
                     f'{"**yes**" if case["changed"] else "no"} |')
    lines += ['', '## The two arrival rows, per run', '']
    for name, case in summary['cases'].items():
        for label, row in (('truth', case['arrival']), ('belief', case['belief'])):
            if row:
                lines.append(f'- `{name}` **{label}** `{row["verdict"]}` -- {row["detail"]}')
        lines.append('')
    (out / 'README.md').write_text('\n'.join(lines) + '\n', encoding='utf-8')
    print(f'\nwrote {out.relative_to(ROOT)}/summary.json and README.md')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
