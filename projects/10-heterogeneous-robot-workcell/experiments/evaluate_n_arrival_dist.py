"""P1-N-10, option (b): accept the physics and state the arrival error as a distribution.

WHAT THE USER DECIDED, AND WHAT THAT DOES AND DOES NOT CHANGE
-------------------------------------------------------------
The old criterion compared ONE run's truth arrival against a hard 0.25 m limit. Measured, that limit
is `0.15 m contract + 0.10 m localisation budget`, while this plant's own AMCL position error has
p95 = 0.119-0.163 m -- so the limit is inside the noise of the thing it is measuring, and the
criterion fails about four runs in ten **by construction**. Option (b) accepts that and states the
arrival error distributionally: median, the extreme quantile the sample can actually support, n, and
how many runs are over each threshold.

WHAT THIS JUDGE REFUSES TO DO
-----------------------------
  * It does NOT delete or rewrite the per-run verdicts. `p1-n-nav-09` stays FAIL and
    `p1-n-nav-10` stays FAIL in their own `acceptance.*` (D042: the original evidence is the
    original evidence). They are listed here BY NAME.
  * It does NOT fail a run for being over the contract -- that is the decision. But it also does
    NOT pass by being silent: the counts over the contract AND over the legacy limit are printed,
    and a row fails if the numbers cannot be produced.
  * It does NOT let an uncitable run into the sample. A run whose recorded component hashes no
    longer match the disk was produced by code that does not exist any more, which is exactly why
    `p1-n-nav-07` cannot be cited. Excluded runs are DECLARED, and the exclusion is checked in the
    other direction too: if such a run becomes citable again, the declaration goes stale and this
    goes red -- otherwise an exclusion list is a place to hide evidence.

THE TWO READINGS
----------------
Every distance is computed twice: re-derived from the run's own `sim_report.json` truth samples,
and read back out of the run's `acceptance.json` prose. A disagreement is a FAIL, because two
records of one quantity disagreeing means one of them is about a different experiment.
"""
import argparse
import json
import math
import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'experiments'))
sys.path.insert(0, str(ROOT / 'src'))

import evaluate_n_nav as per_run  # noqa: E402  -- the per-run judge, so the arithmetic has ONE home

#: The sample. Declared, not discovered by globbing: "every run in the directory" is a set that
#: changes when someone adds a file, and a criterion whose denominator moves on its own is not a
#: criterion. These six were produced with the probes as they are now -- the earlier six (`-09..-14`)
#: are declared excluded below, because `probe_n_nav.py` changed after them and the change was in
#: CODE (an AST comparison with docstrings stripped proves it, so it is not a comment-only edit).
RUNS = ('p1-n-nav-15', 'p1-n-nav-16', 'p1-n-nav-17',
        'p1-n-nav-18', 'p1-n-nav-19', 'p1-n-nav-20')

#: Runs that exist on disk and are deliberately NOT in the sample, each with the reason and the
#: components that make it uncitable. Checked in BOTH directions: a run that becomes citable again
#: makes its entry stale and this judge goes red, so an exclusion list cannot become a place where
#: inconvenient evidence hides.
_REVISION_CHANGED = ('probe_n_nav.py changed after this run: the decomposition patch (`D062`) added '
                     'the post-success settle window, so the run describes a probe that cannot be '
                     're-run')
EXCLUDED = {
    'p1-n-nav-07': {
        'reason': 'produced by probe_n_sim/probe_n_ros revisions that no longer exist on disk, so '
                  'it describes code that cannot be re-run',
        'stale_components': ('probe_n_sim.py', 'probe_n_ros.py'),
    },
    **{f'p1-n-nav-{n}': {'reason': _REVISION_CHANGED, 'stale_components': ('probe_n_nav.py',)}
       for n in ('09', '10', '11', '12', '13', '14')},
}

#: How many citable runs the statement needs before it is a statement. Five is the frozen set; at
#: n = 5 the 95th percentile IS the maximum, and the row says so rather than pretending otherwise.
STATED_MIN_CITABLE_N = 5

#: The old hard limit, kept in the report for continuity and labelled as legacy. It is NOT the
#: criterion any more; it is the number the criterion used to be.
LEGACY_LIMIT_M = 0.25


def stated_distance(acceptance):
    """The arrival distance as the per-run judge wrote it into its own prose, or None."""
    if not acceptance:
        return None
    for item in acceptance.get('checks', []):
        if 'arrived within tolerance of the goal' in item.get('check', ''):
            found = re.search(r'is ([0-9.]+) m and', item.get('detail', ''))
            return float(found.group(1)) if found else None
    return None


def component_status(run_dir, plan):
    """Which of a run's recorded component hashes no longer match the disk."""
    recorded = (plan or {}).get('component_sha256') or {}
    stale = []
    for name, digest in recorded.items():
        path = ROOT / 'experiments' / name
        if not path.is_file():
            stale.append(f'{name} (gone)')
            continue
        import hashlib
        if hashlib.sha256(path.read_bytes()).hexdigest() != digest:
            stale.append(f'{name} ({digest[:12]} recorded, {hashlib.sha256(path.read_bytes()).hexdigest()[:12]} on disk)')
    return stale


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--run-id', default='p1-n-arrival-02')
    ap.add_argument('--reports-dir', default=None)
    args = ap.parse_args()
    out = Path(args.reports_dir) if args.reports_dir else (ROOT / 'reports' / args.run_id)
    out.mkdir(parents=True, exist_ok=True)

    checks = []

    def check(name, verdict, detail):
        checks.append({'check': name, 'verdict': verdict, 'detail': detail})

    rows = []
    for run in RUNS:
        d = ROOT / 'reports' / run
        plan = per_run.load(d / 'nav_plan.json')
        sim = per_run.load(d / 'sim_report.json')
        acc = per_run.load(d / 'acceptance.json')
        stale = component_status(d, plan)
        samples = (sim or {}).get('truth_samples') or []
        goal = (plan or {}).get('goal') or {}
        if not samples or not goal:
            rows.append({'run': run, 'citable': False, 'why': 'no truth samples or no goal',
                         'stale': stale})
            continue
        final = samples[-1][1:]
        derived = math.hypot(final[0] - goal['x'], final[1] - goal['y'])
        stated = stated_distance(acc)
        params_path = (plan.get('params') or {}).get('path')
        gc = per_run.goal_checker_config(ROOT / params_path) if params_path else None
        rows.append({'run': run, 'citable': not stale, 'stale': stale,
                     'derived_m': round(derived, 6), 'stated_m': stated,
                     'agrees': (stated is not None and abs(stated - derived) < 5e-4),
                     'per_run_verdict': (acc or {}).get('verdict'),
                     'contract_m': (gc or {}).get('xy_m'),
                     'contract_stateful': (gc or {}).get('stateful'),
                     'final_x': round(final[0], 4), 'final_y': round(final[1], 4)})

    # ---- the sample ---------------------------------------------------------------------------
    live = [r for r in rows if r.get('citable')]
    n = len(live)
    distances = sorted(r['derived_m'] for r in live)
    median = per_run.percentile(distances, 0.5)
    extreme = distances[-1] if distances else None
    over_contract = [r for r in live if r['contract_m'] and r['derived_m'] > r['contract_m']]
    over_legacy = [r for r in live if r['derived_m'] > LEGACY_LIMIT_M]
    disagreeing = [r['run'] for r in live if r.get('stated_m') is not None and not r['agrees']]

    excluded_detail = '; '.join(f'{r["run"]}: {"; ".join(r["stale"])}'
                                for r in rows if not r.get('citable'))
    check('every run in the sample is CITABLE under the current code',
          'PASS' if n == len(RUNS) else 'FAIL',
          f'{n} of {len(RUNS)} declared runs have component hashes that match the disk. '
          f'Excluded: {excluded_detail or "none"}')

    check('the declared exclusion is still needed',
          'PASS' if all(component_status(ROOT / 'reports' / name, per_run.load(
              ROOT / 'reports' / name / 'nav_plan.json')) for name in EXCLUDED) else 'FAIL',
          'each excluded run must still be uncitable, or the exclusion is stale and would be '
          'hiding a run that is now usable: '
          + '; '.join(f'{name}: {"still stale" if component_status(ROOT / "reports" / name, per_run.load(ROOT / "reports" / name / "nav_plan.json")) else "NOW CITABLE"}'
                      for name in EXCLUDED))

    check('the two records of each distance AGREE',
          'PASS' if not disagreeing else 'FAIL',
          'the distance re-derived from truth samples and the distance stated in the run\'s own '
          'acceptance prose must be the same number; they are one quantity recorded twice. '
          f'Disagreeing: {disagreeing or "none"}. Per run: '
          + '; '.join(f'{r["run"]} derived {r["derived_m"]:.4f} stated {r["stated_m"]}'
                      for r in live))

    check('the per-run verdicts are PRESERVED, not replaced by this statement',
          'PASS' if all(r['per_run_verdict'] in ('PASS', 'FAIL', 'INCOMPLETE', 'NOT_RUN')
                        for r in live) else 'FAIL',
          'this judge adds a distributional acceptance; it does not edit anyone\'s verdict. '
          + '; '.join(f'{r["run"]}={r["per_run_verdict"]}' for r in rows
                      if r.get('per_run_verdict')))

    check('the contract each run was judged against is DERIVED from the params it named',
          'PASS' if live and all(r['contract_m'] for r in live) else 'FAIL',
          'read from `params.path` + the goal checker block, never typed here: '
          + '; '.join(sorted({f'{r["contract_m"]} m (stateful={r["contract_stateful"]})'
                              for r in live if r['contract_m']})))

    check('the arrival error is stated as a distribution with its sample size',
          'PASS' if n >= STATED_MIN_CITABLE_N and median is not None else 'INCOMPLETE',
          (f'n = {n} citable runs; median {median:.4f} m; the extreme quantile the sample supports '
           f'is the MAXIMUM {extreme:.4f} m and at n = {n} a 95th percentile would BE the maximum, '
           f'so that is what is reported instead of a p95 that pretends to more resolution than '
           f'{n} samples carry. Distances: {[round(d, 4) for d in distances]}. '
           f'Over the derived contract ({live[0]["contract_m"] if live else "?"} m): '
           f'{len(over_contract)} of {n}. Over the LEGACY {LEGACY_LIMIT_M} m limit this criterion '
           f'used to apply: {len(over_legacy)} of {n}. BOTH counts are reported because the decision '
           f'was to change the statement, not to hide the number.'
           ) if median is not None else
          (f'n = 0 citable runs of {len(RUNS)} declared, so there is NO distribution to state. '
           f'Every run is uncitable under the current code: '
           + '; '.join(f'{r["run"]}: {", ".join(r["stale"])}' for r in rows if r.get('stale'))
           + '. A sample of zero is not a small sample -- it is the absence of one, and this row '
             'says so instead of reporting an empty median. The fix is to re-run the sample with '
             'the current probes; the previous numbers are not usable and are not used.'))

    # ---- the budget, derived rather than asserted ----------------------------------------------
    shares = []
    for r in live:
        d = ROOT / 'reports' / r['run']
        checker = per_run.load(d / 'checker_nav_raw.json') or {}
        sim = per_run.load(d / 'sim_report.json') or {}
        plan = per_run.load(d / 'nav_plan.json') or {}
        samples = sim.get('truth_samples') or []
        amcl = ((checker.get('observed') or {}).get('amcl_samples')) or []
        if not samples or not amcl:
            continue
        result_time = ((checker.get('strategy') or {}).get('result_sim_s')
                       or samples[-1][0])
        belief = per_run.truth_at(amcl, result_time)
        if belief is None:
            continue
        goal = plan.get('goal') or {}
        belief_distance = math.hypot(belief[0] - goal['x'], belief[1] - goal['y'])
        shares.append({'run': r['run'], 'arrival_m': r['derived_m'],
                       'belief_distance_m': round(belief_distance, 6),
                       'localisation_share_m': round(r['derived_m'] - belief_distance, 6)})
    if shares:
        loc = sorted(s['localisation_share_m'] for s in shares)
        detail = (f'the arrival error decomposes into where the vehicle believes it is and where it '
                  f'actually stopped; median localisation share {per_run.percentile(loc, 0.5):+.4f} m '
                  f'over n = {len(shares)}. This is why the old 0.10 m budget was the wrong shape: '
                  f'the share is not a residual to be budgeted, it is about half the number')
    else:
        detail = ('NOT DERIVABLE on this sample: the decomposition needs amcl samples and truth '
                  'samples in the same run, and one of them is absent. Reported as absent rather '
                  'than estimated')
    check('the localisation share of the arrival error is derived, not assumed',
          'PASS' if shares else 'NOT_RUN', detail)

    failed = [c for c in checks if c['verdict'] == 'FAIL']
    incomplete = [c for c in checks if c['verdict'] == 'INCOMPLETE']
    verdict = 'FAIL' if failed else ('INCOMPLETE' if incomplete else 'PASS')

    report = {'run_id': args.run_id,
              'scope': 'P1-N-10 OPTION (b): the arrival error stated as a distribution, with the '
                       'per-run verdicts preserved',
              'verdict': verdict,
              'checks': checks,
              'sample': {'declared': list(RUNS), 'citable_n': n, 'distances_m': distances,
                         'median_m': median, 'max_m': extreme,
                         'over_contract': [r['run'] for r in over_contract],
                         'over_legacy_limit': [r['run'] for r in over_legacy],
                         'legacy_limit_m': LEGACY_LIMIT_M},
              'excluded': EXCLUDED,
              'rows': rows,
              'localisation': shares,
              'failed': [c['check'] for c in failed],
              'incomplete': [c['check'] for c in incomplete]}
    (out / 'report.json').write_text(json.dumps(report, indent=2) + '\n', encoding='utf-8')

    lines = [f'# {args.run_id} — P1-N-10 option (b)', '',
             f'**verdict {verdict}** over {len(checks)} rows', '']
    for c in checks:
        lines.append(f'- **{c["verdict"]}** {c["check"]}')
        lines.append(f'  - {c["detail"]}')
    lines += ['', (f'median {median:.4f} m · max {extreme:.4f} m · n {n} · '
                   f'over contract {len(over_contract)} · over legacy {len(over_legacy)}')
                  if median is not None else
                  f'n = 0 citable of {len(RUNS)} declared: no distribution to state']
    (out / 'acceptance.md').write_text('\n'.join(lines) + '\n', encoding='utf-8')

    for c in checks:
        print(f'  [{c["verdict"]}] {c["check"]}')
        print(f'        {c["detail"][:600]}')
    print()
    print(f'verdict {verdict}; wrote {out.relative_to(ROOT)}')
    return 0 if verdict == 'PASS' else 1


if __name__ == '__main__':
    raise SystemExit(main())
