import copy
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'experiments'))
from evaluate_receiver_envelope import judge


def nominal():
    return {'world_sha256': 'frozen', 'runtime_qpos_writes': 0,
            'chain_rows': [{'skill': 'VERIFY_DELIVERY', 'final_state': {
                'x_trailing_m': 1.1, 'x_leading_m': 1.8,
                'supported_by': ['receiver_band'], 'on_crowns': True}}]}


def test_only_longitudinal_diagnostic_can_pass():
    result = judge(nominal(), (1, 2), 'frozen')
    assert result['diagnostic_result'] == 'PASS'
    assert result['v1_complete'] is False
    assert result['loaded_delivery'] == 'NOT_RUN'


def test_contact_does_not_substitute_for_containment():
    report = nominal()
    report['chain_rows'][0]['final_state']['x_trailing_m'] = .99
    assert judge(report, (1, 2), 'frozen')['diagnostic_result'] == 'FAIL'


def test_missing_delivery_cannot_pass():
    report = nominal()
    report['chain_rows'] = []
    assert judge(report, (1, 2), 'frozen')['diagnostic_result'] == 'FAIL'


def test_wrong_world_and_nan_are_rejected():
    assert judge(nominal(), (1, 2), 'other')['diagnostic_result'] == 'FAIL'
    report = copy.deepcopy(nominal())
    report['chain_rows'][0]['final_state']['x_leading_m'] = float('nan')
    assert judge(report, (1, 2), 'frozen')['diagnostic_result'] == 'FAIL'
