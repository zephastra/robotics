"""Acceptance mutation checks: measurements, not a PASS label, decide the result."""
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'experiments'))
import evaluate_belt_checkpoint as judge


@pytest.fixture
def report():
    # Synthetic judge input: portable unit tests must not require ignored run reports.
    return {
        'declared': {'slide_budget_m': 0.005},
        'chain': {'rows': [{'status': 'SUCCEEDED'} for _ in range(11)],
                  'load_mark': {'support': ['deck']},
                  'end': {'supported_by': ['receiver_band'], 'on_crowns': True}},
        'arc2_redundant_unload': {'support_after': ['receiver_band'], 'repeat_shift_m': 0.0,
                                 'runaway_support': ['receiver_band'], 'runaway_shift_m': 0.0},
        'arc3_control': {'deck_band_driven': False, 'tray_support_after': ['deck'],
                         'leg_m': 0.35, 'net_slip_m': 0.004, 'peak_rel_m': 0.004},
        'arc3_retention': {'deck_band_driven': True, 'leg_m': 0.35, 'net_slip_m': 0.3},
        'arc1_band_cut': {'support_after': ['source_band']},
        'rows': [{'name': 'the tray is a free rigid body: no equality constraint touches it',
                  'status': 'PASS'},
                 {'name': 'the tray was never teleported and never welded', 'status': 'PASS'}],
    }


def test_all_diagnostic_checks_pass_but_p4_is_not_complete(report):
    answer = judge.judge(report)
    assert answer['diagnostic_result'] == 'PASS'
    assert answer['p4_exit_gate'] == 'INCOMPLETE'
    assert answer['v1_complete'] is False


@pytest.mark.parametrize('field', ['chain', 'arc2_redundant_unload', 'arc3_control', 'rows'])
def test_missing_evidence_never_passes(report, field):
    del report[field]
    assert judge.judge(report)['diagnostic_result'] != 'PASS'


def test_a_good_final_pose_cannot_hide_transient_slide(report):
    report['arc3_control']['net_slip_m'] = 0.0
    report['arc3_control']['peak_rel_m'] = 1.0
    assert judge.judge(report)['checks']['normal_transport_retention'] == 'FAIL'


def test_repeat_motion_and_lost_support_are_rejected(report):
    report['arc2_redundant_unload']['repeat_shift_m'] = 0.10
    report['arc2_redundant_unload']['runaway_support'] = []
    result = judge.judge(report)
    assert result['checks']['repeat_preserves_delivery'] == 'FAIL'
    assert result['checks']['long_repeat_preserves_delivery'] == 'FAIL'


def test_pass_label_cannot_override_missing_measurements(report):
    report['verdict'] = 'PASS'
    report['chain']['rows'][0]['status'] = 'FAILED'
    assert judge.judge(report)['checks']['chain_commands'] == 'FAIL'


def test_empty_report_is_incomplete():
    answer = judge.judge({})
    assert set(answer['checks']) == set(judge.REQUIRED)
    assert all(v == 'UNKNOWN' for v in answer['checks'].values())


def test_receiver_contact_cannot_hide_a_tilted_tray(report):
    report['chain']['end']['on_crowns'] = False
    answer = judge.judge(report)
    assert answer['checks']['delivery_on_crowns'] == 'FAIL'
    assert answer['diagnostic_result'] != 'PASS'
