"""Math checks only; no claims of real parking or docking success."""
import sys
from pathlib import Path
import math
import pytest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'experiments'))
from vehicle_heading import requested_rates


def test_straight_forward_preserves_declared_wheel_mapping():
    assert requested_rates(.04,0,0,{'left':-1.,'right':-1.})=={'left':-1.,'right':-1.}


def test_negative_yaw_requests_positive_turn_not_position_teleport():
    r=requested_rates(0,-.18,0,{'left':1.,'right':1.})
    assert r['left']<0<r['right']


def test_rate_feedback_brakes_rotation_at_zero_error():
    r=requested_rates(0,0,.2,{'left':1.,'right':1.})
    assert r['right']<0<r['left']


@pytest.mark.parametrize('value',[math.nan,math.inf,-math.inf])
def test_nonfinite_state_is_rejected(value):
    with pytest.raises(ValueError):requested_rates(0,value,0,{'left':1.,'right':1.})


def test_turn_request_remains_bounded():
    r=requested_rates(0,2,10,{'left':1.,'right':1.})
    assert max(abs(x) for x in r.values())<=.30*.15/.04


def test_install_brakes_belt_hold_and_preserves_original_servo(monkeypatch):
    import vehicle_heading
    import numpy as np
    from types import SimpleNamespace
    calls=[]
    def original(side,rate):
        calls.append((side,rate))
        return float(np.clip(rate,-1.2,1.2))
    plant=SimpleNamespace(_wheel_signs=lambda:{'left':1.,'right':1.},
        _wheel_rate_torque=original,_hold=lambda:np.array([8.,0.,0.]),
        wheel_actuators={'left':1,'right':2})
    monkeypatch.setattr(vehicle_heading,'measured_heading',lambda p:(-.18,0.))
    vehicle_heading.install(plant)
    held=plant._hold()
    assert held[0]==8. and held[1]<0<held[2]
    assert len(calls)==2
    calls.clear()
    plant._wheel_rate_torque('left',2.)
    assert calls[0][1] < 2.
    with pytest.raises(RuntimeError,match='already installed'):vehicle_heading.install(plant)
