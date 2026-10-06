from copy import deepcopy
from pathlib import Path
import sys
import pytest

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'experiments'))
from evaluate_joint_nav import judge,NODES,REQUIRED


def evidence():
    report=dict(status='COMPLETED',world_sha256='hash',same_model=True,same_data=True,
        owner_steps=100,runtime_qpos_writes=0,commands_received=12,simulation_frozen=None,
        stopping_at_sim_s=2.,truth_judge_only=[
            dict(sim_s=t,xyz=[x,0.,0.],velocity=[0.,0.,0.,0.])
            for t,x in ((0.,0.),(1.,.5),(3.,.5),(4.,.5))])
    worker=dict(status='SUCCEEDED',goal_accepted=True,action_status=4,action_error_code=0,
        active={n:3 for n in NODES},cmd_vel_publishers=['collision_monitor'],nonzero_command_count=10)
    return report,worker,dict(world_sha256='hash',start=[0.,0.],goal=[.6,0.,0.])


def test_complete_evidence_passes_without_executor_pass_label():
    assert judge(*evidence())['commissioning_result']=='PASS'


@pytest.mark.parametrize('field',['truth_judge_only','stopping_at_sim_s','same_model','runtime_qpos_writes','simulation_frozen'])
def test_missing_mandatory_evidence_is_not_pass(field):
    r,w,p=evidence();del r[field]
    assert judge(r,w,p)['commissioning_result']=='FAIL'


@pytest.mark.parametrize('fault',['duplicate_time','nan','moving','drift','short_window','wrong_goal','second_writer','inactive','wrong_world','error_after_complete'])
def test_failures_cannot_be_hidden_by_pass_label(fault):
    r,w,p=deepcopy(evidence());r['commissioning_result']='PASS'
    if fault=='duplicate_time':r['truth_judge_only'][-1]['sim_s']=3.
    if fault=='nan':r['truth_judge_only'][-1]['xyz'][0]=float('nan')
    if fault=='moving':r['truth_judge_only'][-1]['velocity'][0]=.011
    if fault=='drift':r['truth_judge_only'][-1]['xyz'][0]+=.006
    if fault=='short_window':r['truth_judge_only'][-1]['sim_s']=3.1
    if fault=='wrong_goal':p['goal'][0]=10.
    if fault=='second_writer':w['cmd_vel_publishers'].append('other')
    if fault=='inactive':w['active']['amcl']=2
    if fault=='wrong_world':r['world_sha256']='different'
    if fault=='error_after_complete':r['error']='parking authority return refused'
    verdict=judge(r,w,p)
    assert tuple(verdict['checks'])==REQUIRED
    assert verdict['commissioning_result']=='FAIL'
