import copy
from pathlib import Path
import sys
import pytest
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'experiments'))
from evaluate_continuous_nav import judge,CARGO,NODES


def evidence():
    chain=dict(handover_required=True,humanoid_supply_result='PASS',runtime_qpos_writes=0,
        humanoid_supply=dict(abnormal_contacts=[],checks={'original_six_stages':'PASS'},
            trace=[dict(z_m=.85,bilateral=False),dict(z_m=.92,bilateral=True)]),phases=[])
    for item,name in enumerate(CARGO[1:]):
        for phase,z in [('approach',.8),('lift',.86)]:
            chain['phases'].append(dict(item=item,phase=phase,truth_for_judge={name:[0,0,z]}))
    rows=[]
    for i in range(31):
        xyz=[5 if i>=10 else 4+.1*i,0,0]
        rows.append(dict(sim_s=i*.1,xyz=xyz,velocity=[0]*4,
            cargo={n:dict(xyz=xyz,relative_to_base=[2.2,0,.8],relative_to_deck=[.2,0,.04],
                         point_speed_bound_mps=.001) for n in CARGO}))
    nav=dict(status='COMPLETED',runtime_qpos_writes=0,goal=[5,0,0],stop_at_sim_s=1,
        authority_returned=True,commands_received=10,truth_judge_only=rows,motion_refusal=None,
        posture_motion_refusal=None,
        nav2_worker=dict(status='SUCCEEDED',goal_accepted=True,action_status=4,action_error_code=0,
            active={n:3 for n in NODES},cmd_vel_publishers=['collision_monitor'],nonzero_command_count=10),
        sensor_frames=[dict(observation=dict(source='RGBD',status='RESOLVED',counts={'red':2,'blue':1},
            count_verified=True,support='deck'))])
    return chain,nav


def test_complete_fixture_passes_only_its_bounded_scope():
    result=judge(*evidence())
    assert result['integration_result']=='PASS' and result['full_order']=='NOT_RUN'
    assert result['three_dimensional_safety']=='NOT_RUN' and result['v1_complete'] is False


def test_same_entity_lifted_twice_is_not_three_items():
    chain,nav=evidence()
    for row in chain['phases']:
        if row['item']==1:
            value=row['truth_for_judge'].pop(CARGO[2]);row['truth_for_judge'][CARGO[1]]=value
    assert judge(chain,nav)['checks']['actual_loading']=='FAIL'


def test_two_red_entities_may_be_picked_in_either_order():
    chain,nav=evidence()
    for row in chain['phases']:
        if row['item'] in (0,1):
            name=CARGO[row['item']+1];other=CARGO[2-row['item']]
            row['truth_for_judge'][other]=row['truth_for_judge'].pop(name)
    assert judge(chain,nav)['checks']['actual_loading']=='PASS'


@pytest.mark.parametrize('fault',['action','missing','cargo_speed','cargo_drift','slip','unknown',
    'supply','loading','write','authority','publishers','gap','short_stop','exception'])
def test_each_required_evidence_can_independently_fail(fault):
    chain,nav=evidence()
    if fault=='action':nav['nav2_worker']['action_status']=6
    elif fault=='missing':nav['truth_judge_only'][1]['cargo'].pop(CARGO[1])
    elif fault=='cargo_speed':nav['truth_judge_only'][-1]['cargo'][CARGO[1]]['point_speed_bound_mps']=.02
    elif fault=='cargo_drift':nav['truth_judge_only'][-1]['cargo'][CARGO[1]]['xyz']=[5,.006,0]
    elif fault=='slip':nav['truth_judge_only'][5]['cargo']['c_payload']['relative_to_deck']=[.206,0,.04]
    elif fault=='unknown':nav['sensor_frames'][-1]['observation']['status']='UNKNOWN'
    elif fault=='supply':chain['humanoid_supply']['trace'][1]['bilateral']=False
    elif fault=='loading':chain['phases']=[]
    elif fault=='write':nav['runtime_qpos_writes']=1
    elif fault=='authority':nav['authority_returned']=False
    elif fault=='publishers':nav['nav2_worker']['cmd_vel_publishers'].append('other')
    elif fault=='gap':nav['truth_judge_only'].pop(5)
    elif fault=='short_stop':nav['stop_at_sim_s']=2.8
    elif fault=='exception':nav['error']='late error'
    assert judge(chain,nav)['integration_result']=='FAIL'


def test_compliant_deck_motion_is_not_cargo_slip():
    chain,nav=evidence()
    nav['truth_judge_only'][5]['cargo']['c_payload']['relative_to_base']=[2.23,0,.8]
    result=judge(chain,nav)
    assert result['checks']['tray_retention']=='PASS'
    assert result['measurements']['peak_tray_relative_to_base_m']>.005


def test_missing_deck_frame_cannot_prove_retention():
    chain,nav=evidence()
    for row in nav['truth_judge_only']:row['cargo']['c_payload'].pop('relative_to_deck')
    result=judge(chain,nav)
    assert result['checks']['tray_retention']=='FAIL'
    assert result['measurements']['tray_retention_evidence']=='MISSING_DECK_FRAME_NOT_MEASURED'


def test_posture_refusal_cannot_be_promoted_by_other_pass_labels():
    chain,nav=evidence()
    nav['posture_motion_refusal']='TRANSPORT_POSTURE_UNCONFIRMED'
    assert judge(chain,nav)['integration_result']=='FAIL'
    del nav['posture_motion_refusal']
    assert judge(chain,nav)['checks']['transport_posture_authorized']=='FAIL'
