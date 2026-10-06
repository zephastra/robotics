import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'experiments'))
from inspect_retained_nav import summarize


def test_forensics_does_not_promote_empty_evidence():
    result=summarize({})
    assert result['first_5mm_crossing'] is None
    assert result['original_report_result'] is None
    assert result['full_order']=='NOT_RUN' and result['v1_complete'] is False


def test_first_slip_is_separate_from_later_support_refusal():
    rows=[dict(sim_s=t,tray_in_deck_judge_only=[x,0,0])
        for t,x in ((0.,.12),(12.5,.114),(15.,.113))]
    frame=dict(sim_s=15.,observation=dict(status='RESOLVED',support='UNKNOWN'))
    result=summarize(dict(truth_judge_only=rows,sensor_frames=[frame],commissioning_result='FAIL'))
    assert result['first_5mm_crossing']==rows[1]
    assert result['first_sensor_refusal']==frame
    assert result['original_report_result']=='FAIL'
