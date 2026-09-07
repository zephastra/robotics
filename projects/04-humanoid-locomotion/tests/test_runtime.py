import math
from pathlib import Path
import numpy as np
import pytest
from humanoid004.app import Keys, check_assets, EXPECTED_JOINTS
from humanoid004.core import Pose, Route
import mujoco


def test_assets_and_floating_base():
    root=Path(__file__).resolve().parents[1]
    manifest=check_assets()
    assert manifest['commit']=='276801e46c5d433564f24658bac64f254b7d2d4b'
    model=mujoco.MjModel.from_xml_path(str(root/'assets/g1/lab.xml'))
    assert (model.nq,model.nv,model.nu)==(19,18,12)
    assert model.jnt_type[0]==mujoco.mjtJoint.mjJNT_FREE
    assert np.all(model.jnt_actfrclimited[1:])
    names=[mujoco.mj_id2name(model,mujoco.mjtObj.mjOBJ_JOINT,int(j)) for j in model.actuator_trnid[:,0]]
    assert names==EXPECTED_JOINTS


def test_manual_deadman_and_pause(monkeypatch):
    import humanoid004.app as app
    now=[0.0]
    monkeypatch.setattr(app.time,'monotonic',lambda:now[0])
    keys=Keys()
    keys.callback(ord('W'))
    assert keys.read()[0][0]>.1
    now[0]=.7
    assert np.all(keys.read()[0]==0)
    keys.callback(ord('R'))
    assert keys.read()[0][0]>0 and keys.read()[0][2]>0
    keys.callback(ord('P'))
    assert keys.read()[2] is True
    assert np.all(keys.read()[0]==0)
    keys.callback(ord('P'))
    assert keys.read()[2] is False


def test_manual_cancel_and_push():
    k=Keys()
    k.callback(ord('C'))
    assert k.read()[1] is True
    k.callback(ord('F'))
    assert k.read()[3] is True
    assert k.read()[3] is False


def test_hold_rejects_high_speed():
    cfg=dict(route=[dict(name='A',x=0,y=0,yaw=0)],warmup_seconds=0,waypoint_timeout=90,
             position_tolerance=.3,yaw_tolerance_degrees=15,hold_seconds=10,hold_mean_speed_limit=.15)
    route=Route(cfg)
    route.step(Pose(0,0,0),0,speed=.5)
    route.step(Pose(0,0,0),10,speed=.5)
    assert route.phase!='COMPLETED'


def test_independent_runtime():
    root=Path(__file__).resolve().parents[1]
    for path in (root/'src').rglob('*.py'):
        source=path.read_text()
        assert '003_mobile_manipulator' not in source
        assert '002_amr_mission_executor' not in source
        assert 'rclpy' not in source
    assert 'source /opt/ros' not in (root/'run_demo.sh').read_text()


def test_terminal_key_input_and_restore(monkeypatch):
    import os
    import pty
    import termios
    import time
    import humanoid004.app as app
    master,slave=pty.openpty()
    terminal=os.fdopen(slave,'r',buffering=1)
    saved=termios.tcgetattr(terminal.fileno())
    monkeypatch.setattr(app.sys,'stdin',terminal)
    keys=Keys()
    try:
        with app.TerminalKeys(keys,True):
            os.write(master,b'w')
            deadline=time.monotonic()+2
            while keys.read()[0][0]==0 and time.monotonic()<deadline:
                time.sleep(.01)
            assert keys.read()[0][0]>.1
            os.write(master,b'c')
            deadline=time.monotonic()+2
            while not keys.read()[1] and time.monotonic()<deadline:
                time.sleep(.01)
            assert keys.read()[1]
        assert termios.tcgetattr(terminal.fileno())==saved
    finally:
        terminal.close()
        os.close(master)
