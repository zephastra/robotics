import numpy as np
import pytest
from humanoid007.vision import Frame,detect


def frame(kind='box'):
    rgb=np.zeros((240,320,3),dtype=np.uint8)
    rgb[52:188,143:177]=[255,0,255] if kind=='box' else [0,255,0]
    return Frame(rgb,np.full((240,320),.5),np.array([.23,0,1.35]),
                 np.eye(3),65.,3.)


@pytest.mark.parametrize('kind',['box','destination'])
def test_rgb_depth_deprojection(kind):
    detection=detect(frame(kind),kind)
    assert detection is not None
    assert np.allclose(detection.center,[.23,0,.85],atol=.002)
    assert detection.normal[2]>.99
    assert detection.long_axis[1]>.99
    assert detection.time==3.


def test_absent_target_does_not_fabricate_position():
    f=frame();f.rgb[:]=0
    assert detect(f,'box') is None


def test_wrong_color_not_detected():
    assert detect(frame('destination'),'box') is None


def test_image_edge_crop_rejected():
    f=frame();f.rgb[0:10,143:177]=[255,0,255]
    assert detect(f,'box') is None


@pytest.mark.parametrize('bad',['depth','pose','fovy'])
def test_invalid_sensor_data_rejected(bad):
    f=frame()
    if bad=='depth':f.depth[:]=np.nan
    if bad=='pose':f.position[:]=np.nan
    if bad=='fovy':f.fovy=0.
    assert detect(f,'box') is None


def test_known_camera_translation_changes_estimate():
    f=frame();a=detect(f,'box')
    f.position+=np.array([.1,.2,0])
    b=detect(f,'box')
    assert np.allclose(b.center-a.center,[.1,.2,0])


def test_unknown_marker_class_fails_explicitly():
    with pytest.raises(ValueError):detect(frame(),'unknown')
