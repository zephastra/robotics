"""Loading inventory is explicit, and absent pieces must never vacuously PASS."""
import copy
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'experiments'))
import probe_loaded_retention as probe


def input_report():
    parts = [{'name': name, 'class': kind, 'expected_cell': cell,
              'in_expected_cell': True, 'touches_tray': True,
              'is_free_body': True} for name, kind, cell in probe.PARTS]
    arm = {'load': {'state': 'RECEIVED'}, 'before': parts, 'after': copy.deepcopy(parts),
           'transport': {'leg_m': .35, 'peak_including_brake_m': .004, 'net_slip_m': .004},
           'tray': {'supported_by': ['deck']}, 'qpos_writes': {'calibrate': 3}}
    fault = copy.deepcopy(arm)
    fault['transport']['net_slip_m'] = .30
    return {'nominal': arm, 'fault_deck_driven': fault}


def test_valid_retention_does_not_claim_order_or_arm_loading():
    result = probe.judge(input_report())
    assert result['diagnostic_result'] == 'PASS'
    assert result['full_order'] == 'NOT_RUN'
    assert result['arm_loading'] == 'NOT_RUN'
    assert result['v1_complete'] is False


@pytest.mark.parametrize('count', [0, 1, 2])
def test_missing_parts_are_not_vacuous_pass(count):
    report = input_report()
    report['nominal']['after'] = report['nominal']['after'][:count]
    assert probe.judge(report)['checks']['three_parts_retained'] == 'FAIL'


def test_wrong_identity_is_not_correct_quantity():
    report = input_report()
    report['nominal']['after'][0]['name'] = 'unregistered_part'
    assert probe.judge(report)['checks']['three_parts_retained'] == 'FAIL'


def test_good_endpoint_cannot_hide_braking_slide():
    report = input_report()
    report['nominal']['transport']['peak_including_brake_m'] = .02
    assert probe.judge(report)['checks']['normal_slip_within_budget'] == 'FAIL'


def test_floating_part_is_not_retained_inventory():
    report = input_report()
    report['nominal']['after'][0]['touches_tray'] = False
    assert probe.judge(report)['checks']['three_parts_retained'] == 'FAIL'
