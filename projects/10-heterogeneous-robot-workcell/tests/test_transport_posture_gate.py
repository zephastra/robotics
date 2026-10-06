"""Pure motion/brake separation and first-failure evidence contracts."""
from pathlib import Path
import sys
import pytest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'experiments'))
from transport_posture_gate import TransportPostureGate


def test_valid_posture_authorizes_motion():
    gate=TransportPostureGate()
    assert gate.check(dict(allowed=True,reason_code=None,failures=[]))
    assert gate.latched_reason is None


@pytest.mark.parametrize('observation',[None,{},dict(allowed=1,reason_code=None,failures=[]),
    dict(allowed=True,reason_code=None),dict(allowed=True,reason_code=None,failures=[{}])])
def test_unknown_or_inconsistent_posture_refuses_motion(observation):
    gate=TransportPostureGate()
    assert not gate.check(observation)
    assert gate.latched_reason=='TRANSPORT_POSTURE_UNKNOWN'


def test_failure_is_latched_and_evidence_is_immutable():
    gate=TransportPostureGate()
    failure=dict(allowed=False,reason_code='TRANSPORT_POSTURE_UNCONFIRMED',
                 failures=[dict(joint='c_deck_slide_y',value=.01001,allowed=[-.01,.01])])
    assert not gate.check(failure)
    failure['failures'][0]['value']=0
    assert gate.first_failure['failures'][0]['value']==.01001
    assert not gate.check(dict(allowed=True,reason_code=None,failures=[]))
    assert gate.latched_reason=='TRANSPORT_POSTURE_UNCONFIRMED'


def test_pure_gate_has_no_physics_or_reauthorization_interface():
    gate=TransportPostureGate()
    assert not hasattr(gate,'reset')
    source=(Path(__file__).resolve().parents[1]/'experiments/transport_posture_gate.py').read_text()
    assert 'mj_step' not in source and 'qpos' not in source


def test_posture_refusal_keeps_existing_single_writer_rate_brake():
    from types import SimpleNamespace
    from joint_world_nav_io import JointWorldNavIO
    io=JointWorldNavIO.__new__(JointWorldNavIO)
    io.closed=False;io.poll=lambda:None;io.data=SimpleNamespace(time=1.)
    io.gate=SimpleNamespace(step=lambda stamp:(.2,.5,'MOVING',None))
    io.radius=.1;io.track=.3;io.signs=dict(left=1.,right=1.)
    io.plant=SimpleNamespace(_wheel_rate_torque=lambda side,rate:rate,
        _control=lambda callback:[callback('left'),callback('right')])
    gate=TransportPostureGate()
    assert io.control(lambda requested:gate.check(None))==[0.,0.]
    assert gate.latched_reason=='TRANSPORT_POSTURE_UNKNOWN'
