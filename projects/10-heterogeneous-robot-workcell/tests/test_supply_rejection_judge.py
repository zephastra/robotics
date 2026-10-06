"""Fake evidence tests only; actual supply rejection needs its own physical run."""
import copy
import importlib.util
from pathlib import Path

import pytest

PATH = Path(__file__).resolve().parents[1] / 'experiments' / 'evaluate_supply_rejection.py'
spec = importlib.util.spec_from_file_location('supply_judge', PATH)
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


def records():
    return (dict(handover_required=True, humanoid_supply=dict(status='FAILED',
        checks=dict(bilateral_lift='FAIL', lift_at_least_50mm='FAIL'),
        trace=[dict(age_s=i, z_m=.85, bilateral=False) for i in (0, 1)]),
        phases=[], writer_frozen=dict(simulation_frozen=True,allowed=False,
            stopped_confirmed=False,reason_code='SUPPLY_UNCONFIRMED',
            hold_mode='SIM_FROZEN_NO_PHYSICAL_STOP_PROOF'),runtime_qpos_writes=0),
        dict(diagnostic_result='FAIL', checks=dict(continuous_humanoid_supply='FAIL')))


def test_expected_failure_does_not_become_task_success():
    result = module.judge(*records())
    assert result['safety_result'] == 'PASS'
    assert result['task_outcome'] == 'FAILED'
    assert result['mechanical_stop'] == result['recovery'] == 'NOT_RUN'
    assert result['v1_complete'] is False


@pytest.mark.parametrize('patch', [
    {'phases': [{'phase': 'close'}]}, {'initial_count': {'status': 'UNKNOWN'}},
    {'transport_rows': [{'skill': 'MOVE_TO_STATION'}]}, {'writer_frozen': False},
    {'writer_frozen': True}, {'writer_frozen': {'simulation_frozen': True}},
    {'runtime_qpos_writes': False}, {'runtime_qpos_writes': 1},
    {'transaction_final_records': {'leg': {'stage': 'COMMITTED'}}},
    {'humanoid_supply': {'status': 'FAILED', 'checks': {}, 'trace': []}},
    {'handover_required': False},
])
def test_incomplete_or_inconsistent_evidence_is_not_pass(patch):
    report, execution = records()
    report.update(copy.deepcopy(patch))
    assert module.judge(report, execution)['safety_result'] == 'FAIL'


def test_execution_pass_cannot_be_counted_as_expected_rejection():
    report, execution = records()
    execution['diagnostic_result'] = 'PASS'
    assert module.judge(report, execution)['safety_result'] == 'FAIL'
