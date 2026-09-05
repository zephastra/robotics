import math
from pathlib import Path
import cv2
import numpy as np
import pytest
from mobile_manipulator.kinematics import inverse, forward, world_to_base, base_to_world, TRAY
from mobile_manipulator.vision import detect_box


@pytest.mark.parametrize('point',[(.70,0,.70),TRAY,(-.24,0,.85),(.50,.15,.75),(.30,-.50,.60),(.8200000000000001,0,.85)])
def test_ik_round_trip(point):
    assert forward(inverse(*point))[:3] == pytest.approx(point)


@pytest.mark.parametrize('point',[(1.,0,.7),(0.,0.,.7),(.5,0,.2),(.5,0,1.0),(float('nan'),0,.7)])
def test_unreachable_rejected(point):
    with pytest.raises(ValueError): inverse(*point)


def test_frame_round_trip():
    xy=base_to_world(.7,.1,1.5,3.,1.2)
    assert world_to_base(*xy,1.5,3.,1.2)==pytest.approx((.7,.1))


def test_pixel_projection():
    rgb=np.zeros((480,640,3),np.uint8); rgb[90:111,310:331]=[0,0,255]
    x,y,_=detect_box(rgb,400,400,320,240,.75)
    assert (x,y)==pytest.approx((.6675,0))


def test_red_marker():
    rgb=np.zeros((480,640,3),np.uint8); rgb[100:121,300:321]=[255,0,0]
    assert detect_box(rgb,400,400,320,240,.652,'red')[2]['pixels']==441


def test_home_marker_orientation():
    rgb=np.zeros((480,640,3),np.uint8); rgb[90:131,310:331]=[255,0,255]
    _,_,meta=detect_box(rgb,400,400,320,240,.003,'magenta')
    assert meta['yaw']==pytest.approx(0.)


def test_missing_and_ambiguous_target():
    rgb=np.zeros((480,640,3),np.uint8)
    with pytest.raises(ValueError): detect_box(rgb,400,400,320,240,.75)
    rgb[100:120,100:120]=[0,0,255]; rgb[200:220,200:220]=[0,0,255]
    with pytest.raises(ValueError): detect_box(rgb,400,400,320,240,.75)


def test_no_runtime_dependency_on_other_projects():
    root=Path(__file__).resolve().parents[1]
    for folder in ('src','scripts','launch','config'):
        for path in (root/folder).rglob('*'):
            if path.suffix not in ('.py','.sh','.yaml'): continue
            text=path.read_text()
            assert '002_amr_mission_executor' not in text
            assert 'projects/amr_slam' not in text
    assert 'set_pose' not in (root/'src/mobile_manipulator/mission.py').read_text()


def test_wheel_speed_not_limited_by_arm_default():
    import runpy
    import xml.etree.ElementTree as ET
    root=Path(__file__).resolve().parents[1]
    generator=runpy.run_path(str(root/'tools/generate_assets.py'))
    robot=ET.fromstring(generator['robot']())
    for side in ('left','right'):
        limit=float(robot.findtext(f'joint[@name="{side}_wheel_joint"]/axis/limit/velocity'))
        assert limit*.12 >= .4
    assert float(robot.findtext('joint[@name="shoulder"]/axis/limit/velocity')) <= .6
