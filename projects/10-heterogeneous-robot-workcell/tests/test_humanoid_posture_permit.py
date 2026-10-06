"""Pure guards only, not physical braking, balance, or recovery evidence."""
import sys
from pathlib import Path
from types import SimpleNamespace
import numpy as np
import pytest
ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/'src'),str(ROOT/'experiments')]
from humanoid_posture_permit import posture_status,HumanoidPosturePermit
from world_owner import WorldOwner,WorldSafetyHold


def test_original_height_limit_not_relaxed():
    assert posture_status(.65,[1,0,0,0])['allowed']
    assert not posture_status(.649,[1,0,0,0])['allowed']


def test_original_tilt_limit_not_relaxed():
    angle=np.radians(36)
    assert posture_status(.9,[np.cos(angle/2),np.sin(angle/2),0,0])['reason_code']=='BODY_FALL'


@pytest.mark.parametrize('height,quat',[(np.nan,[1,0,0,0]),(.9,[0,0,0,0]),(.9,[1,0,0]),(.9,[np.nan,0,0,0])])
def test_invalid_posture_is_not_healthy(height,quat):
    assert posture_status(height,quat)['reason_code']=='POSTURE_UNKNOWN'


def test_post_supply_fall_revokes_without_automatic_recovery():
    model=SimpleNamespace(joint=lambda name:SimpleNamespace(id=0),jnt_qposadr=np.array([0]))
    data=SimpleNamespace(qpos=np.array([0.,0.,.9,1,0,0,0]))
    def observe():return dict(owner='world',epoch=1,generation=1,sequence=0,
        observed_wall_s=0.,cancel_requested=False,zone_clear=True,stop_chain_healthy=True)
    p=HumanoidPosturePermit(observe,model=model,data=data,owner='world',epoch=1,generation=1,max_age_s=.5,clock=lambda:0.)
    assert p.check()['allowed']
    data.qpos[2]=.358
    assert p.check()['reason_code']=='BODY_FALL'
    data.qpos[2]=.9
    assert not p.check()['allowed']


@pytest.mark.parametrize('fall_during_callback',[False,True])
def test_final_writer_rejects_fall_before_commit_and_never_steps(monkeypatch,fall_during_callback):
    model=SimpleNamespace(nq=7,nu=1,joint=lambda name:SimpleNamespace(id=0),jnt_qposadr=np.array([0]))
    data=SimpleNamespace(qpos=np.array([0.,0.,.9 if fall_during_callback else .4,1,0,0,0]),ctrl=np.zeros(1),time=0.)
    def observe():return dict(owner='world',epoch=1,generation=1,sequence=0,
        observed_wall_s=0.,cancel_requested=False,zone_clear=True,stop_chain_healthy=True)
    permit=HumanoidPosturePermit(observe,model=model,data=data,owner='world',epoch=1,generation=1,max_age_s=.5,clock=lambda:0.)
    owner=WorldOwner(model,data,permit)
    calls=[]
    def command():
        calls.append('producer')
        data.qpos[2]=.4  # Fake state fault; never a real cargo/controller write.
        return np.ones(1)
    monkeypatch.setattr('world_owner.mujoco.mj_step',lambda *args:pytest.fail('unsafe step'))
    with pytest.raises(WorldSafetyHold,match='BODY_FALL'):owner.step(command)
    assert len(calls)==int(fall_during_callback)
    assert owner.steps==0 and data.ctrl[0]==0.
    assert owner.frozen['simulation_frozen'] is True
    assert owner.frozen['stopped_confirmed'] is False
