from pathlib import Path
import sys
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'experiments'))
from load_motion_gate import LoadMotionGate

CONTRACT=dict(centre_low=[1.9,-.125],centre_high=[2.47,.125],max_sim_age_s=.5,max_wall_age_s=.5)


def fresh():
    return dict(source='RGBD',status='RESOLVED',support='deck',count_verified=True,
        observed_sim_s=10.,observed_wall_s=20.,tray_in_base_xy=[2.2,0.],position_uncertainty_m=.01)


def test_fresh_load_grants_motion_not_stop_claim():
    result=LoadMotionGate(CONTRACT).check(fresh(),now_sim_s=10.1,now_wall_s=20.1,motion_requested=True)
    assert result['motion_allowed'] and not result['brake_required']
    assert 'stopped_confirmed' not in result


def test_stale_moving_frame_requires_brake_and_latches_task():
    gate=LoadMotionGate(CONTRACT)
    result=gate.check(fresh(),now_sim_s=10.1,now_wall_s=21.,motion_requested=True)
    assert not result['motion_allowed'] and result['brake_required']
    assert result['latched_reason']=='LOAD_ENVELOPE_UNKNOWN'
    new=fresh();new['observed_wall_s']=21.
    assert not gate.check(new,now_sim_s=10.1,now_wall_s=21.1,motion_requested=True)['motion_allowed']


def test_unknown_does_not_disable_braking_or_claim_recovery():
    gate=LoadMotionGate(CONTRACT)
    result=gate.check({},now_sim_s=10.,now_wall_s=20.,motion_requested=False)
    assert result['brake_required'] and not result['motion_allowed']
    assert result['latched_reason'] is None


def test_known_outside_region_refuses_movement():
    observation=fresh();observation['tray_in_base_xy']=[3.,0.]
    result=LoadMotionGate(CONTRACT).check(observation,now_sim_s=10.1,now_wall_s=20.1,motion_requested=True)
    assert result['latched_reason']=='LOAD_OUTSIDE_DECLARED_ENVELOPE'


def test_quantity_or_support_unknown_cannot_start_motion():
    for key,value in [('count_verified',False),('support','UNKNOWN')]:
        observation=fresh();observation[key]=value
        result=LoadMotionGate(CONTRACT).check(observation,now_sim_s=10.1,now_wall_s=20.1,motion_requested=True)
        assert result['brake_required'] and not result['motion_allowed']
