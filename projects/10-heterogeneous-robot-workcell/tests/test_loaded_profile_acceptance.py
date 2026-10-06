"""No physical run: optional requested transport cannot disappear from verdict."""
import pytest
import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'experiments'))
from probe_candidate_multi_loading import REQUIRED, aggregate_acceptance


@pytest.mark.parametrize('transport', ['FAIL', 'UNKNOWN', 'NOT_RUN', None])
def test_requested_transport_without_pass_fails_top_level(transport):
    report={'scope':'test'}
    if transport is not None:
        report['transport_diagnostic_result']=transport
    result=aggregate_acceptance(report,dict.fromkeys(REQUIRED,True),True)
    assert result['diagnostic_result']=='FAIL'
    assert result['checks']['continuous_loaded_transport']!='PASS'


def test_complete_loading_and_transport_passes_only_diagnostic():
    result=aggregate_acceptance({'scope':'test','transport_diagnostic_result':'PASS'},
                                dict.fromkeys(REQUIRED,True),True)
    assert result['diagnostic_result']=='PASS'
    assert result['full_order']=='NOT_RUN'
    assert result['v1_complete'] is False


def test_loading_profile_does_not_require_unrequested_transport():
    assert aggregate_acceptance({'scope':'test'},dict.fromkeys(REQUIRED,True))[
        'diagnostic_result']=='PASS'


def test_missing_loading_check_is_not_erased_by_transport_pass():
    checks=dict.fromkeys(REQUIRED,True)
    del checks[REQUIRED[0]]
    assert aggregate_acceptance({'scope':'test','transport_diagnostic_result':'PASS'},
                                checks,True)['diagnostic_result']=='FAIL'
def test_required_handover_cannot_inherit_component_success():
    from probe_candidate_multi_loading import aggregate_acceptance,REQUIRED
    report=dict(scope='test',handover_required=True,transport_diagnostic_result='PASS')
    acceptance=aggregate_acceptance(report,{k:True for k in REQUIRED},True)
    assert acceptance['diagnostic_result']=='FAIL'
    assert acceptance['checks']['continuous_humanoid_supply']=='NOT_RUN'


def test_failed_handover_blocks_top_level_success():
    from probe_candidate_multi_loading import aggregate_acceptance,REQUIRED
    report=dict(scope='test',handover_required=True,humanoid_supply_result='FAIL',
                transport_diagnostic_result='PASS')
    assert aggregate_acceptance(report,{k:True for k in REQUIRED},True)['diagnostic_result']=='FAIL'
