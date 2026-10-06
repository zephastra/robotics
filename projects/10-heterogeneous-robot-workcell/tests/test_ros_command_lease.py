"""Offline leases only, not physical braking or Nav2 acceptance."""
import sys
from pathlib import Path
import pytest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'experiments'))
from ros_command_lease import ControllerLease


def test_repeat_bridge_transmission_cannot_renew_controller_lease():
    wall=[0.];g=ControllerLease(clock=lambda:wall[0])
    assert g.offer(.2,0.,1.,1.)[0]
    for value in (.1,.2,.4):
        wall[0]=value
        assert g.value(1.)==(.2,0.,1.)
    wall[0]=.501
    assert g.value(1.) is None


def test_replayed_stamp_cannot_extend_wall_lease():
    wall=[0.];g=ControllerLease(clock=lambda:wall[0])
    g.offer(.2,0.,1.,1.)
    wall[0]=.4
    assert not g.offer(.2,0.,1.,1.)[0]
    wall[0]=.6
    assert g.value(1.) is None


@pytest.mark.parametrize('stamp,sim',[(.4,1.),(1.1,1.),(float('nan'),1.)])
def test_stale_future_and_nonfinite_stamps_refused(stamp,sim):
    assert not ControllerLease().offer(.2,0.,stamp,sim)[0]


def test_valid_new_command_renews_and_limits_are_not_silently_clipped():
    wall=[0.];g=ControllerLease(clock=lambda:wall[0])
    g.offer(.2,0.,1.,1.)
    wall[0]=.4
    assert g.offer(.3,.1,1.4,1.4)[0]
    assert not g.offer(.61,0.,1.5,1.5)[0]
    wall[0]=.7
    assert g.value(1.7)==(.3,.1,1.4)


def test_clock_reversal_revokes_before_error():
    g=ControllerLease(clock=lambda:0.)
    g.offer(.2,0.,1.,1.)
    g.value(1.)
    with pytest.raises(RuntimeError,match='REAUTHORIZATION'):g.value(.9)
    assert g.accepted is None


def test_wall_clock_reversal_refuses_without_motion():
    wall=[1.];g=ControllerLease(clock=lambda:wall[0])
    g.offer(.2,0.,1.,1.)
    wall[0]=.9
    assert g.value(1.) is None
