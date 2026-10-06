from pathlib import Path
import sys
from types import SimpleNamespace
import pytest

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'experiments'))
from wheel_authority import grant_navigation,return_to_parking


def plant():
    raw=lambda *args:0.;park=lambda *args:1.
    return SimpleNamespace(_heading_feedback_base_servo=raw,_heading_feedback_parking_servo=park,
        _wheel_rate_torque=park,heading_feedback_installed=True)


STOP=dict(stopped_confirmed=True,held_speed_mps=.001,drift_m=.001)


def test_exclusive_controller_identity_both_directions():
    p=plant();token=grant_navigation(p)
    assert p._wheel_rate_torque is p._heading_feedback_base_servo and not p.heading_feedback_installed
    return_to_parking(p,token,STOP,nav_process_stopped=True,ipc_closed=True)
    assert p._wheel_rate_torque is p._heading_feedback_parking_servo and p.heading_feedback_installed


def test_no_second_grant():
    p=plant();grant_navigation(p)
    with pytest.raises(RuntimeError):grant_navigation(p)


@pytest.mark.parametrize('fault',['wrong_token','live_nav','open_ipc','no_stop','moving','drifting','nan','conflict'])
def test_failed_return_retains_navigation_owner(fault):
    p=plant();token=grant_navigation(p);stop=dict(STOP);nav=True;ipc=True;given=token
    if fault=='wrong_token':given=object()
    if fault=='live_nav':nav=False
    if fault=='open_ipc':ipc=False
    if fault=='no_stop':stop={}
    if fault=='moving':stop['held_speed_mps']=.011
    if fault=='drifting':stop['drift_m']=.006
    if fault=='nan':stop['held_speed_mps']=float('nan')
    if fault=='conflict':p.heading_feedback_installed=True
    with pytest.raises(RuntimeError):return_to_parking(p,given,stop,nav_process_stopped=nav,ipc_closed=ipc)
    assert p.navigation_authority is token
