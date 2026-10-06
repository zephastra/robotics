"""Fake/math/structural contracts only; actual shared-world ROS navigation NOT_RUN."""
import ast
import sys
from pathlib import Path
import pytest
import numpy as np
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'src'))
sys.path.insert(0,str(ROOT/'experiments'))
from joint_world_nav_io import WallGuardedCommand, bounded_ranges, JointWorldNavIO
from workcell_ipc import command,encode


def config():
    return dict(ttl_s=.5,silence_s=.5,v_max_mps=.6,w_max_radps=1.2)


def message(seq=0):return encode('command',command(seq=seq,issued_sim_time=1.,ttl_s=.5,v=.2,w=0.))


def test_paused_sim_cannot_preserve_motor_commands_after_wall_silence():
    wall=[0.];gate=WallGuardedCommand(config(),clock=lambda:wall[0])
    assert gate.offer(message(),1.)[0]
    assert gate.step(1.)[0]==.2
    wall[0]=.501
    assert gate.step(1.)[:2]==(0.,0.)


def test_invalid_packet_does_not_extend_wall_lease():
    wall=[0.];gate=WallGuardedCommand(config(),clock=lambda:wall[0])
    gate.offer(message(),1.)
    wall[0]=.4
    assert not gate.offer(b'not json',1.)[0]
    wall[0]=.6
    assert gate.step(1.)[0]==0.


def test_sim_clock_reversal_requires_authorization():
    gate=WallGuardedCommand(config(),clock=lambda:0.)
    gate.step(1.)
    with pytest.raises(RuntimeError,match='REAUTHORIZATION'):gate.step(.5)


def test_bridge_neither_resets_nor_steps_nor_writes_physical_state():
    tree=ast.parse((ROOT/'experiments/joint_world_nav_io.py').read_text())
    calls={ast.unparse(node.func) for node in ast.walk(tree) if isinstance(node,ast.Call)}
    assert not any('mj_step' in name or 'mj_reset' in name or 'MjData' in name for name in calls)
    for node in ast.walk(tree):
        if isinstance(node,ast.Assign):
            assert not any('.qpos[' in ast.unparse(t) or '.ctrl[' in ast.unparse(t) for t in node.targets)


def test_no_hit_is_maximum_not_false_near_obstacle():
    result=bounded_ranges([-1.,np.inf,np.nan,.05,9.,2.],[-1,-1,-1,2,3,4],.12,8.)
    np.testing.assert_array_equal(result,[8.,8.,8.,.12,8.,2.])


def test_heading_servo_cannot_compete_with_nav2():
    from types import SimpleNamespace
    cfg=dict(ipc={'host':'127.0.0.1'},ros={'domain_id':43})
    plant=SimpleNamespace(step_owner=object(),heading_feedback_installed=True)
    with pytest.raises(ValueError,match='removed before Nav2'):JointWorldNavIO(plant,cfg)
