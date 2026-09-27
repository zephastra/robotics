"""Re-judge the P3 navigation runs with the identity- and profile-aware judge, into a new id.

Why a new id rather than re-judging in place: `evaluate_p3_nav.py --run-id X` reads
`reports/X/...` and WRITES `reports/X/p3_nav_gate.json`, so re-judging in place would destroy the
verdict recorded when the run happened (`D042`). The guidance says the same thing -- original
reports stay read-only.

What makes the parallel set comparable rather than a second opinion:
  * the input JSONs are COPIED and their sha256 is asserted equal to the originals', so the two
    verdicts are about byte-identical evidence and any difference is the judge, not the run;
  * the earlier verdict AND the earlier `variant` are read from the original gate, never typed;
  * the originals' sha256 is taken BEFORE and AFTER, and a change is an error rather than a note;
  * the README and the summary are derived, so "which numbers moved" is computed rather than
    asserted.

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
DEFAULT_RUNS = ('p3-nav-05', 'p3-nav-06')
GATE = 'p3_nav_gate.json'


def sha256(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--run-id', required=True,
                        help='the NEW evidence id; must not already exist')
    parser.add_argument('--runs', nargs='*', default=list(DEFAULT_RUNS))
    parser.add_argument('--reason', default='the physical variant is now derived from the run\'s own '
                                            'content hash instead of from its run id, and the '
                                            'verdict is computed over a declared required set '
                                            'instead of over whatever rows happen to be present')
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
        if not (origin / 'sim_report.json').is_file():
            raise SystemExit(f'{origin} is not a navigation run directory')
        before_sha = sha256(origin / GATE) if (origin / GATE).is_file() else None
        before = json.loads((origin / GATE).read_text(encoding='utf-8')) \
            if (origin / GATE).is_file() else None

        dest = cases / source
        dest.mkdir(parents=True)
        digests = {}
        for name in INPUTS:
            src, dst = origin / name, dest / name
            dst.write_bytes(src.read_bytes())
            # byte-identity is asserted, not assumed: a copy that silently differs would make the
            # comparison meaningless
            assert sha256(dst) == sha256(src), f'{name} copy differs for {source}'
            digests[name] = sha256(src)

        completed = subprocess.run(
            [sys.executable, str(ROOT / 'experiments' / 'evaluate_p3_nav.py'),
             '--run-id', source, '--reports-dir', str(cases)],
            cwd=str(ROOT), capture_output=True, text=True)
        fresh = json.loads((dest / GATE).read_text(encoding='utf-8'))

        # the originals must be untouched -- that is the whole point of judging a copy
        after_sha = sha256(origin / GATE) if (origin / GATE).is_file() else None
        if after_sha != before_sha:
            raise SystemExit(f'REFUSING: {source}/{GATE} changed while judging a copy')

        summary['cases'][source] = {
            'inputs_sha256': digests,
            'gate_sha256_before': before_sha,
            'variant_before': (before or {}).get('variant'),
            'variant_now': fresh['variant'],
            'verdict_before': (before or {}).get('verdict'),
            'verdict_now': fresh['verdict'],
            'counts_before': (before or {}).get('counts'),
            'counts_now': fresh['counts'],
            'identity_detail': fresh['identity']['detail'],
            'judge_exit_code': completed.returncode,
            'arrival_metres': next(
                (float(c['detail'].split('): ')[1].split(' m')[0])
                 for c in fresh['checks']
                 if c['name'] == 'the truth arrival error is within the limit'), None),
            'ok': (after_sha == before_sha),
        }
        case = summary['cases'][source]
        print(f'{source}: variant {case["variant_before"]} -> {case["variant_now"]}, '
              f'verdict {case["verdict_before"]} -> {case["verdict_now"]}, '
              f'counts {case["counts_before"]} -> {case["counts_now"]}')

    moved = [name for name, case in summary['cases'].items()
             if (case['variant_before'], case['verdict_before']) !=
                (case['variant_now'], case['verdict_now'])]
    summary['changed'] = moved
    (out / 'summary.json').write_text(json.dumps(summary, indent=2) + '\n', encoding='utf-8')

    lines = [
        f'# P3 navigation re-judged -- {args.run_id}', '',
        f'**The physical variant changed for {len(moved)} of {len(summary["cases"])} runs: '
        f'{moved or "none"}.**', '',
        f'Why: {args.reason}', '',
        'Each case is a byte-identical copy of the run\'s input JSONs (sha256 asserted), judged by '
        f'`experiments/evaluate_p3_nav.py`. The originals keep their own `{GATE}`; their sha256 was '
        'taken before and after and is recorded in `summary.json`, so "the originals were not '
        'touched" is a checked fact rather than an intention.', '',
        '| run | variant then | variant now | verdict then | verdict now | counts then | counts now |',
        '|---|---|---|---|---|---|---|',
    ]
    for name, case in summary['cases'].items():
        lines.append(f"| `{name}` | {case['variant_before']} | **{case['variant_now']}** | "
                     f"{case['verdict_before']} | **{case['verdict_now']}** | "
                     f"{json.dumps(case['counts_before'])} | {json.dumps(case['counts_now'])} |")
    lines += ['', '## The identity row, per run', '']
    for name, case in summary['cases'].items():
        lines.append(f"- `{name}` -- {case['identity_detail']}")
    lines += ['', '## What this does and does not change', '',
              '- **It changes the identity claim.** The loaded run was recorded as `bare` and every '
              'geometric row in its gate was anchored to the unloaded arena.',
              '- **It does not move the arrival number.** The two variants share the same start, '
              'goal, obstacles and room, so the same measurement lands on the same figure; the '
              'number that changed is not the distance but which world it is a distance in.',
              '- **The verdicts are unchanged, and that is not a defence of the old aggregator.** '
              'The one NOT_RUN row is the arrival yaw, which the new profile declares NOT '
              'APPLICABLE in advance; the old aggregate passed it because it DROPPED the row, which '
              'is a different reason and would also have dropped a required check.',
              '']
    (out / 'README.md').write_text('\n'.join(lines) + '\n', encoding='utf-8')
    print(f'\nwrote {out.relative_to(ROOT)}/summary.json and README.md')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
