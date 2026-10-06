"""Independent expected-failure judgment; never rewrites the execution verdict."""
import argparse
import hashlib
import json
from pathlib import Path


def judge(report, execution):
    supply = report.get('humanoid_supply')
    supply = supply if isinstance(supply, dict) else {}
    supplied = supply.get('checks')
    supplied = supplied if isinstance(supplied, dict) else {}
    execution_checks = execution.get('checks')
    execution_checks = execution_checks if isinstance(execution_checks, dict) else {}
    frozen = report.get('writer_frozen')
    frozen = frozen if isinstance(frozen, dict) else {}
    facts = {
        'handover_requested': report.get('handover_required') is True,
        'supply_rejected': supply.get('status') == 'FAILED',
        'no_bilateral_lift': supplied.get('bilateral_lift') == 'FAIL',
        'no_50mm_lift': supplied.get('lift_at_least_50mm') == 'FAIL',
        'execution_not_success': execution.get('diagnostic_result') == 'FAIL',
        'aggregate_supply_failed': execution_checks.get('continuous_humanoid_supply') == 'FAIL',
        'no_loading_phases': report.get('phases') == [],
        'no_loading_observation': 'initial_count' not in report,
        'no_transport_started': report.get('transport_rows', []) == [],
        'writer_frozen': (frozen.get('simulation_frozen') is True
                          and frozen.get('allowed') is False
                          and frozen.get('stopped_confirmed') is False
                          and frozen.get('reason_code') == 'SUPPLY_UNCONFIRMED'
                          and frozen.get('hold_mode') == 'SIM_FROZEN_NO_PHYSICAL_STOP_PROOF'),
        'no_runtime_cargo_writes': type(report.get('runtime_qpos_writes')) is int
                                  and report['runtime_qpos_writes'] == 0,
        'no_completed_custody': not report.get('transaction_final_records')
                               and not report.get('custody_transactions'),
    }
    # These facts require an actual supply trace, not a hand-written failure string.
    trace = supply.get('trace')
    facts['physical_supply_recorded'] = (isinstance(trace, list) and len(trace) > 1
        and all(isinstance(row, dict) and 'age_s' in row and 'z_m' in row
                and type(row.get('bilateral')) is bool for row in trace))
    checks = {key: 'PASS' if value else 'FAIL' for key, value in facts.items()}
    return dict(scope='EXPECTED_SUPPLY_REJECTION_SIM_FREEZE_ONLY', checks=checks,
                safety_result='PASS' if all(facts.values()) else 'FAIL',
                task_outcome='FAILED', mechanical_stop='NOT_RUN',
                recovery='NOT_RUN', full_order='NOT_RUN', v1_complete=False)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--input-run', required=True)
    parser.add_argument('--run-id', required=True)
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[1]
    for name in (args.input_run, args.run_id):
        if Path(name).name != name or name in ('.', '..'):
            parser.error('run identifiers must be bare names')
    source = root / 'reports' / args.input_run
    paths = [source / 'report.json', source / 'acceptance.json']
    report, execution = [json.loads(path.read_text()) for path in paths]
    verdict = judge(report, execution)
    verdict['evidence'] = {str(path.relative_to(root)):
        hashlib.sha256(path.read_bytes()).hexdigest() for path in paths}
    verdict['source_sha256'] = hashlib.sha256(Path(__file__).read_bytes()).hexdigest()
    out = root / 'reports' / args.run_id
    out.mkdir(parents=True, exist_ok=False)
    (out / 'acceptance.json').write_text(json.dumps(verdict, indent=2) + '\n')
    print(json.dumps(verdict, indent=2))
    return 0 if verdict['safety_result'] == 'PASS' else 1


if __name__ == '__main__':
    raise SystemExit(main())
