"""Pure tests: an empty destination cannot replace the first physical failure."""
import sys
from pathlib import Path
import pytest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'experiments'))
from probe_candidate_multi_loading import require_logistics_success


def test_failed_docking_is_retained_before_any_receiver_read():
    report={}
    rows=[dict(skill='DOCK',result=dict(status='FAILED',reason_code='DOCK_OUT_OF_TOLERANCE'))]
    with pytest.raises(RuntimeError,match='DOCK_OUT_OF_TOLERANCE'):
        require_logistics_success(report,rows,11)
    assert report['first_logistics_failure'] is rows[0]
    assert report['transport_diagnostic_result']=='FAIL'


def test_incomplete_sequence_is_never_a_success():
    with pytest.raises(RuntimeError,match='INCOMPLETE_SEQUENCE'):
        require_logistics_success({},[],11)


def test_full_success_can_proceed_to_independent_receiver_judge():
    report={}
    require_logistics_success(report,[dict(result=dict(status='SUCCEEDED')) for _ in range(11)],11)
    assert report=={}
