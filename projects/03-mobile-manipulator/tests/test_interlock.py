"""Exercise the real velocity gate without launching ROS or a simulator."""
from types import SimpleNamespace
import pytest
from geometry_msgs.msg import Twist
from mobile_manipulator.runtime import Runtime


@pytest.mark.parametrize('fault',[None,'not_permitted','arm_extended','stale_permission','stale_joint','stale_velocity'])
def test_velocity_gate_is_fail_closed(monkeypatch, fault):
    monkeypatch.setattr('mobile_manipulator.runtime.time.monotonic',lambda:100.)
    sent=[]; cmd=Twist(); cmd.linear.x=.2
    node=SimpleNamespace(stow={'lift':.5,'elbow':2.5},joints={'lift':.5,'elbow':2.5},
                         permit=True,last_permit=99.9,last_joint=99.9,last_cmd=99.9,
                         cmd=cmd,pub=SimpleNamespace(publish=sent.append))
    if fault=='not_permitted': node.permit=False
    if fault=='arm_extended': node.joints['elbow']=0.
    if fault=='stale_permission': node.last_permit=98.
    if fault=='stale_joint': node.last_joint=98.
    if fault=='stale_velocity': node.last_cmd=98.
    Runtime.tick(node)
    assert len(sent)==1
    assert sent[0].linear.x == (.2 if fault is None else 0.)
