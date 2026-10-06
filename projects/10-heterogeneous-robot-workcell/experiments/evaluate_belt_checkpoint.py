#!/usr/bin/env python3
"""Independent, explicitly LIMITED judgment of a belt diagnostic report.

This cannot accept P4 or an order. Full-load, interrupt recovery, shared-zone
interlocks and entire receiver containment are NOT_RUN until physically tested.
"""
import argparse
import hashlib
import json
import math
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
REQUIRED = ('chain_commands', 'load_supported', 'delivery_supported',
            'delivery_on_crowns',
            'repeat_preserves_delivery', 'long_repeat_preserves_delivery',
            'normal_transport_retention', 'driven_band_negative_control',
            'missing_drive_refusal', 'free_tray', 'no_run_teleport')
P4_GAPS = ('full_load_retention', 'mid_transfer_interrupt_recovery',
           'runtime_shared_zone_interlock', 'receiver_full_footprint_containment',
           'h085_retention_requalification')


def judge(r):
    rows = {}

    def check(name, condition):
        rows[name] = 'PASS' if condition else 'FAIL'

    try:
        budget = float(r['declared']['slide_budget_m'])
        if not math.isfinite(budget) or budget <= 0:
            raise ValueError('invalid declared budget')
        chain = r['chain']
        check('chain_commands', len(chain['rows']) == 11
              and all(x['status'] == 'SUCCEEDED' for x in chain['rows']))
        check('load_supported', chain['load_mark']['support'] == ['deck'])
        repeat = r['arc2_redundant_unload']
        check('delivery_supported', chain['end']['supported_by'] == ['receiver_band'])
        check('delivery_on_crowns', chain['end']['on_crowns'] is True)
        check('repeat_preserves_delivery', repeat['support_after'] == ['receiver_band']
              and 0 <= float(repeat['repeat_shift_m']) <= budget)
        check('long_repeat_preserves_delivery', repeat['runaway_support'] == ['receiver_band']
              and 0 <= float(repeat['runaway_shift_m']) <= budget)
        normal, negative = r['arc3_control'], r['arc3_retention']
        check('normal_transport_retention', normal['deck_band_driven'] is False
              and normal['tray_support_after'] == ['deck']
              and normal['leg_m'] >= 0.30 and 0 <= normal['net_slip_m'] <= budget
              and 0 <= normal['peak_rel_m'] <= budget)
        check('driven_band_negative_control', negative['deck_band_driven'] is True
              and negative['leg_m'] >= 0.30 and negative['net_slip_m'] > budget)
        check('missing_drive_refusal', r['arc1_band_cut']['support_after'] == ['source_band'])
        # Structural measurements belong to the existing probe. Missing rows remain UNKNOWN.
        structural = {x['name']: x['status'] for x in r['rows']}
        rows['free_tray'] = structural.get(
            'the tray is a free rigid body: no equality constraint touches it', 'UNKNOWN')
        rows['no_run_teleport'] = structural.get(
            'the tray was never teleported and never welded', 'UNKNOWN')
    except (KeyError, TypeError, ValueError):
        pass
    for name in REQUIRED:
        rows.setdefault(name, 'UNKNOWN')
    return {'scope': 'EMPTY_TRAY_W2_DIAGNOSTIC_CHECKPOINT', 'required_checks': list(REQUIRED),
            'checks': rows,
            'diagnostic_result': 'PASS' if all(rows[x] == 'PASS' for x in REQUIRED) else 'INCOMPLETE_OR_FAIL',
            'p4_required_unverified': {x: 'NOT_RUN' for x in P4_GAPS},
            'p4_exit_gate': 'INCOMPLETE', 'v1_complete': False}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source-run', required=True)
    parser.add_argument('--run-id', required=True)
    args = parser.parse_args(argv)
    source = (ROOT / 'reports' / args.source_run / 'report.json').resolve()
    out = (ROOT / 'reports' / args.run_id).resolve()
    if not source.is_relative_to(ROOT / 'reports') or not out.is_relative_to(ROOT / 'reports'):
        parser.error('report path escapes 010/reports')
    if out.exists() and any(out.iterdir()):
        parser.error('evidence already exists')
    data = json.loads(source.read_text())
    result = judge(data)
    result.update(source_run=args.source_run, source_sha256=hashlib.sha256(source.read_bytes()).hexdigest())
    out.mkdir(parents=True, exist_ok=True)
    (out / 'acceptance.json').write_text(json.dumps(result, indent=2) + '\n')
    print(json.dumps(result, indent=2))
    return 0 if result['diagnostic_result'] == 'PASS' else 1


if __name__ == '__main__':
    raise SystemExit(main())
