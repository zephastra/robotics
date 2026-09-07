import math
import numpy as np
import pytest
from humanoid004.core import CommandFilter, FallMonitor, Pose, Route, orientation, target_command, wrap


@pytest.mark.parametrize('angle',[0,math.pi,-math.pi,5,-5,20])
def test_wrap(angle):
    assert -math.pi <= wrap(angle) <= math.pi
    assert math.sin(wrap(angle)) == pytest.approx(math.sin(angle))


@pytest.mark.parametrize('yaw',[-2,-.3,0,.7,2])
def test_orientation(yaw):
    actual,tilt=orientation([math.cos(yaw/2),0,0,math.sin(yaw/2)])
    assert actual == pytest.approx(yaw)
    assert tilt == pytest.approx(0)


def test_invalid_orientation():
    with pytest.raises(ValueError): orientation([0,0,0,0])


def test_commands_limit_ramp_and_timeout():
    f=CommandFilter([.5,.2,.6],[.4,.4,.8],.6)
    f.submit([9,9,9],0)
    assert f.target == pytest.approx([.5,.2,.6])
    assert f.step(0,.1) == pytest.approx([.04,.04,.08])
    assert f.step(1,.1) == pytest.approx([0,0,0])


@pytest.mark.parametrize('command',[[float('nan'),0,0],[float('inf'),0,0],[0,0]])
def test_reject_invalid_commands(command):
    f=CommandFilter([.5,.2,.6],[.4,.4,.8],.6)
    with pytest.raises(ValueError): f.submit(command,0)


def test_clock_regression_expires_command():
    f=CommandFilter([.5,.2,.6],[.4,.4,.8],.6)
    f.submit([.4,0,0],5)
    assert np.all(f.step(0,.1)==0)


def test_target_ahead_and_behind():
    p=Pose(0,0,0)
    assert target_command(p,dict(x=3,y=0,yaw=0))[0] > 0
    cmd=target_command(p,dict(x=-3,y=0,yaw=0))
    assert cmd[0] == 0
    assert abs(cmd[2]) > 0


def test_world_to_body():
    cmd=target_command(Pose(0,0,math.pi/2),dict(x=0,y=.2,yaw=math.pi/2))
    assert cmd[0] > 0
    assert cmd[1] == pytest.approx(0,abs=1e-12)


def config():
    return dict(route=[dict(name='A',x=1,y=0,yaw=0)],warmup_seconds=2,
                waypoint_timeout=90,position_tolerance=.3,yaw_tolerance_degrees=15,hold_seconds=10)


def test_route_requires_continuous_hold():
    r=Route(config())
    goal=Pose(1,0,0)
    r.step(goal,0)
    assert r.phase=='WARMUP'
    r.step(goal,2)
    r.step(Pose(2,0,0),11)
    r.step(goal,12)
    r.step(goal,21)
    assert r.phase!='COMPLETED'
    r.step(goal,22)
    assert r.phase=='COMPLETED'
    assert r.events[0]['hold_seconds']>=10


def test_route_rejects_wrong_heading():
    r=Route(config())
    r.step(Pose(1,0,math.pi),2)
    assert r.phase=='WALKING'


def test_route_timeout():
    r=Route(config())
    r.step(Pose(0,0,0),2)
    r.step(Pose(0,0,0),93)
    assert r.phase=='FAILED'


def test_fall_debounce_and_reset():
    f=FallMonitor(.45,60,.12)
    assert not f.update(.3,0,0)
    assert not f.update(.3,0,.1)
    assert not f.update(.8,0,.11)
    assert not f.update(.3,0,.2)
    assert f.update(.3,0,.33)


def test_fall_tilt_and_nan():
    f=FallMonitor(.45,60,.12)
    assert not f.update(.8,math.radians(70),0)
    assert f.update(.8,math.radians(70),.2)
    assert f.update(float('nan'),0,.3)
