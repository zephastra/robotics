#!/usr/bin/env python3
"""C: write the re-judged verdicts back into reports/.

WHY THIS IS A SEPARATE, BACKED-UP STEP
    The judge gained three corrections on 2026-09-20 (D-P17-28): a not-yet-placed slot no
    longer reads as a position, `UNKNOWN` is no longer "unowned", and a PASS with zero
    exposure is NOT_RUN. Every verdict in reports/ was produced by the previous judge, so
    every verdict now has to be replaced or the tree mixes two instruments. That is a rewrite
    of evidence, so it is done once, with a copy taken first and every change listed.

FAITHFULNESS
    `safety_outcome` is NOT recomputed here: `batch.safety_from_judgement` is imported and
    called, so the derivation cannot drift from the one the runner uses. `judge-<label>.json`
    is written with the same `indent=2, sort_keys=True` the runner uses, and `truth` is
    rebuilt with the same field list.

TRACEABILITY
    Each rewritten `summary.json` gains a `rejudged` block naming the date, the tool and the
    reason, so a reader can tell a rewritten verdict from an as-run one.
"""
from __future__ import annotations

import argparse
import json
import pathlib
import shutil
import sys
import time

ROOT = pathlib.Path('/home/ziling/projects/009_multi_robot_fleet')
BACKUP_ROOT = pathlib.Path('/mnt/c/Users/ZiLing/WorkBuddy/2026-09-11-18-34-08/_p009/p17')
sys.path.insert(0, str(ROOT / 'scripts'))
sys.path.insert(0, str(ROOT / 'src' / 'fleet_core'))
sys.path.insert(0, str(ROOT / 'src' / 'fleet_evaluation'))

import yaml                                                    # noqa: E402
import batch                                                   # noqa: E402
from fleet_core import validate_traffic_config                  # noqa: E402
from fleet_evaluation.judge import judge_case, load_spawns      # noqa: E402


def case_dirs() -> list[pathlib.Path]:
    out = []
    for pattern in ('batch_20260919T0*', 'batch_20260919T1*', 'batch_20260920T*'):
        for batch_dir in sorted((ROOT / 'reports').glob(pattern)):
            if batch_dir.name.startswith('_'):
                continue
            for case_dir in sorted(batch_dir.iterdir()):
                if case_dir.is_dir() and list(case_dir.glob('samples-*.jsonl')):
                    out.append(case_dir)
    return out


def label_of(case_dir: pathlib.Path) -> str:
    return next(case_dir.glob('samples-*.jsonl')).name[len('samples-'):-len('.jsonl')]


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument('--apply', action='store_true', help='write; without it, only report')
    args = ap.parse_args(argv)

    cfg = validate_traffic_config(
        yaml.safe_load((ROOT / 'config' / 'resources.yaml').read_text(encoding='utf-8')))
    spawns = load_spawns(ROOT / 'config' / 'spawns.yaml')

    dirs = case_dirs()
    print(f'{len(dirs)} case director(ies) with a recording in this generation')
    if not args.apply:
        print('DRY RUN -- nothing will be written')

    # ---- back up the two files that carry a verdict ------------------------- #
    stamp = time.strftime('%Y%m%dT%H%M%SZ', time.gmtime())
    backup = BACKUP_ROOT / f'BACKUP_reports_verdicts_{stamp}'
    if args.apply:
        n = 0
        for case_dir in dirs:
            label = label_of(case_dir)
            dest = backup / case_dir.parent.name / case_dir.name
            dest.mkdir(parents=True, exist_ok=True)
            for name in (f'judge-{label}.json', 'summary.json'):
                src = case_dir / name
                if src.is_file():
                    shutil.copy2(src, dest / name)
                    n += 1
        print(f'backed up {n} file(s) to {backup}')

    changed = []
    for case_dir in dirs:
        label = label_of(case_dir)
        summary_path = case_dir / 'summary.json'
        judge_path = case_dir / f'judge-{label}.json'
        try:
            record = json.loads(summary_path.read_text(encoding='utf-8'))
        except Exception as exc:
            print(f'  {label}: unreadable summary ({exc})')
            continue

        judged = judge_case(case_dir, cfg, spawns)
        outcome, why = batch.safety_from_judgement(judged)

        old = record.get('safety_outcome')
        prior = (record.get('truth') or {}).get('checks')
        # batch.py stores this as a flat {check: verdict} mapping; handle the list shape too
        # rather than assuming, because getting it wrong here silently reports "no change".
        if isinstance(prior, dict):
            old_checks = dict(prior)
        elif isinstance(prior, list):
            old_checks = {c['check']: c['verdict'] for c in prior if isinstance(c, dict)}
        else:
            old_checks = {}

        # A verdict can stay the same while the DETAIL changes -- `N07` keeps its FAIL but the
        # count is now split into "the book called FREE" and "the book said UNKNOWN". Skipping
        # those would leave the tree with two wordings for the same check, so the skip
        # condition is "the file on disk is already what this judge produces".
        try:
            on_disk = json.loads(judge_path.read_text(encoding='utf-8'))
        except Exception:
            on_disk = None
        if on_disk == judged and record.get('rejudged') and old == outcome:
            continue

        new_checks = {c['check']: c['verdict'] for c in judged.get('checks') or []}
        deltas = sorted(k for k in set(old_checks) | set(new_checks)
                        if old_checks.get(k) != new_checks.get(k))
        changed.append((case_dir.parent.name, label, old, outcome, deltas))

        if args.apply:
            judge_path.write_text(json.dumps(judged, indent=2, sort_keys=True),
                                  encoding='utf-8')
            record['safety_outcome'] = outcome
            record['safety_reason'] = why
            record['truth'] = {
                'judge_file': judge_path.name,
                'samples': judged.get('samples'),
                'samples_file': judged.get('samples_file'),
                'summary': judged.get('summary'),
                'robots_judged': judged.get('robots_judged'),
                'robots_absent': judged.get('robots_absent'),
                'truth_has_heading': judged.get('truth_has_heading'),
                'failed': judged.get('failed'),
                'not_run': judged.get('not_run'),
                'held': judged.get('held'),
                'caveats': judged.get('caveats'),
                'checks': new_checks,
            }
            record['rejudged'] = {
                'on': stamp,
                'tool': 'scripts/rejudge_writeback.py',
                'why': ('D-P17-28: unplaced slots no longer read as positions, UNKNOWN is not '
                        '"unowned", and a zero-exposure PASS is NOT_RUN. The verdicts in this '
                        'tree predated those corrections.'),
                'safety_outcome_before': old,
            }
            summary_path.write_text(json.dumps(record, indent=2, sort_keys=True),
                                    encoding='utf-8')

    print()
    print(f'{len(changed)} case(s) change')
    print(f'  {"batch":<24} {"label":<12} {"safety":<22} checks that moved')
    for batch_name, label, old, new, deltas in changed:
        print(f'  {batch_name:<24} {label:<12} {str(old) + " -> " + str(new):<22} '
              + ', '.join(deltas))

    if args.apply and changed:
        # The report.md files embed the OLD verdicts in a table. They cannot be regenerated
        # faithfully -- `write_report` wants the GLOBAL plan (38 / runnable / blocked) and
        # each report only ever saw its own slice -- so instead each touched report gets a
        # banner saying what it is: a snapshot from before the judge was corrected, with
        # summary.json authoritative. Labelling the stale artefact is the same move this
        # project uses for `superseded` recordings and `reports/_invalid/`.
        banner = (f'> **RE-JUDGED {stamp} (D-P17-28).** The table below was written by the '
                  f'runner at the time and records the verdicts of the judge as it then was:\n'
                  f'> a not-yet-placed slot counted as a position, `UNKNOWN` counted as '
                  f'unowned, and a PASS with no exposure counted as a PASS. This file is a '
                  f'snapshot; the authoritative verdicts are in each case\'s `summary.json` '
                  f'and `judge-*.json`, which were rewritten. Do not quote this table as '
                  f'current.')
        touched = {b for b, _l, _o, _n, _d in changed}
        for batch_name in sorted(touched):
            report = ROOT / 'reports' / batch_name / 'report.md'
            if not report.is_file():
                continue
            text = report.read_text(encoding='utf-8')
            if 'RE-JUDGED' in text:
                continue
            first, _, rest = text.partition('\n')
            report.write_text(f'{first}\n\n{banner}\n{rest}', encoding='utf-8')
        print(f'\nbanner added to {len(touched)} report.md file(s)')
        print()
        print('written. re-run scripts/tally38.py to regenerate the rollup.')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
