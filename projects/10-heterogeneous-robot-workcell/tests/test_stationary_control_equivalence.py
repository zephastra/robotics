"""Fake-state algebraic checks, NOT standing or walking physical acceptance."""
import copy
import sys
from pathlib import Path
from types import SimpleNamespace
import numpy as np
import pytest
root=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(root/'src'),str(root/'experiments')]
from humanoid007.runtime import Runtime,OPEN
from humanoid_supply import stationary_control


class FakePolicy:
    def __init__(self,rng):
        self.default=rng.normal(size=25);self.target=rng.normal(size=25)
        self.kp=np.full(25,20.);self.kd=np.full(25,2.);self.inferences=0
    def infer(self,*args):
        self.inferences+=1
        # Deliberately different walking output: all these entries must be
        # overridden before a stationary shortcut can claim equivalence.
        self.target=np.linspace(-3.,3.,25)
    def torques(self,q,dq):return (self.target-q)*self.kp-dq*self.kd


def fake(seed,tick):
    rng=np.random.default_rng(seed);r=Runtime.__new__(Runtime)
    r.policy=FakePolicy(rng);r.qa=np.arange(7,32);r.va=np.arange(6,31)
    r.body_act=np.arange(25);r.arm_ids=np.arange(13,23)
    r.tick=tick;r.stand_target=rng.normal(size=13)
    r.stand_weight=1.;r.arm_weight=1.;r.command=np.zeros(3)
    r.stand_kp=np.full(13,30.);r.stand_kd=np.full(13,3.)
    r.arm_target=rng.normal(size=10);r.head_target=rng.normal(size=2)
    r.arms={'left':None,'right':None}
    r.hand_target={s:np.array(OPEN,float)+.02 for s in r.arms}
    n=len(OPEN);r.hand_act={'left':np.arange(25,25+n),'right':np.arange(25+n,25+2*n)}
    r.d=SimpleNamespace(qpos=rng.normal(size=32),qvel=rng.normal(size=31),
                        qfrc_bias=rng.normal(size=31),ctrl=np.zeros(25+2*n),time=1.)
    r.m=SimpleNamespace(actuator_trnid=np.column_stack([np.arange(25),np.zeros(25,int)]),
                         jnt_actfrcrange=np.tile([-100.,100.],(25,1)))
    return r


@pytest.mark.parametrize('tick',[0,1,4,5,123])
@pytest.mark.parametrize('seed',[1,2,3])
def test_full_blend_control_equals_existing_feedback(tick,seed):
    original=fake(seed,tick);shortcut=copy.deepcopy(original)
    default=original.policy.default[original.arm_ids].copy()
    base=np.arange(len(original.d.ctrl),dtype=float)
    expected=original.control(np.zeros(3),default,{s:OPEN for s in original.arms},
                              stationary=True,base_control=base)
    actual=stationary_control(shortcut,default,base)
    np.testing.assert_array_equal(actual,expected)
    np.testing.assert_array_equal(shortcut.arm_target,original.arm_target)
    assert shortcut.policy.inferences==0


def test_partial_blend_uses_original_controller():
    r=fake(3,0);r.stand_weight=.9
    stationary_control(r,r.policy.default[r.arm_ids],r.d.ctrl)
    assert r.policy.inferences==1
