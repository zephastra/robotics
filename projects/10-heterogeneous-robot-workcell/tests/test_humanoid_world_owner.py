"""Humanoid entry uses the same commit gate; no policy inference needed here."""
import sys
from pathlib import Path

import mujoco
import numpy as np
import pytest

ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/'experiments'),str(ROOT/'src')]
from humanoid007.runtime import Runtime
from world_owner import WorldOwner,WorldSafetyHold
from workcell.runtime_permit import RuntimePermit


def fixture():
    m=mujoco.MjModel.from_xml_string('<mujoco><worldbody><body><joint name="j"/>'
        '<geom size=".05"/></body></worldbody><actuator><motor joint="j"/>'
        '</actuator></mujoco>')
    d=mujoco.MjData(m)
    r=Runtime.__new__(Runtime)
    r.m,r.d,r.tick=m,d,0
    calls=[]
    r.control=lambda *a,**kw:calls.append(kw.get('base_control')) or np.ones(m.nu)
    fault=[False]
    sequence=[0]
    def observe():
        sequence[0]+=1
        return dict(owner='world',epoch=1,generation=1,sequence=sequence[0],
            observed_wall_s=10.,cancel_requested=fault[0],zone_clear=True,stop_chain_healthy=True)
    permit=RuntimePermit(observe,owner='world',epoch=1,generation=1,max_age_s=.5,
                         clock=lambda:10.)
    return r,WorldOwner(m,d,permit),fault,calls


def test_default_step_unchanged():
    r,owner,fault,calls=fixture()
    r.step(np.zeros(3))
    assert r.d.time>0 and r.tick==1 and len(calls)==1
    assert owner.steps==0


def test_composed_background_and_one_step():
    r,owner,fault,calls=fixture()
    r.step_owner=owner
    r.background_control=lambda:np.array([.25])
    r.step(np.zeros(3))
    assert owner.steps==1 and r.tick==1
    assert calls[0]==pytest.approx([.25])


def test_cancel_before_humanoid_inference_no_ctrl_or_tick_write():
    r,owner,fault,calls=fixture()
    r.step_owner=owner
    fault[0]=True
    before=r.d.ctrl.copy()
    with pytest.raises(WorldSafetyHold):r.step(np.zeros(3))
    assert r.tick==0 and r.d.time==0 and calls==[]
    assert np.array_equal(before,r.d.ctrl)


def test_wrong_world_cannot_use_owner():
    r,owner,fault,calls=fixture()
    r.step_owner=owner
    r.d=mujoco.MjData(r.m)
    with pytest.raises(ValueError):r.step(np.zeros(3))
    assert r.tick==0 and calls==[]
