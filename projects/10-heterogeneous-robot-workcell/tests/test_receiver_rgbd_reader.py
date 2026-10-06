import sys
from pathlib import Path
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'experiments'))
from receiver_rgbd_reader import locate
from rgbd_geometry import image_components


def test_image_connectivity_keeps_detached_handle_patches_separate():
    mask=np.zeros((10,12),bool)
    mask[1:5,1:5]=True;mask[7:9,9:11]=True
    parts=list(image_components(mask))
    assert sorted(len(p) for p in parts)==[4,16]


def scene():
    x,y=np.meshgrid(np.linspace(12.435,12.565,50),np.linspace(-.23,.23,80))
    world=np.stack([x,y,np.full_like(x,.82)],axis=-1)
    template={'deck':{'centre':np.array([5.,0.,.815]),'size':np.array([.065,.23,.005])},
              'axis':1,'cells':[]}
    rgb=np.broadcast_to([40.,140.,220.],world.shape)
    return template,world,rgb,np.ones(x.shape,bool),(12.,14.,-.25,.25),.82


def test_localizes_transported_tray_not_old_template_pose():
    args=scene()
    result=locate(*args)
    assert result['status']=='RESOLVED'
    assert np.allclose(result['centre_xy_m'],[12.5,0.])
    assert args[0]['deck']['centre'][0]==5.


def test_missing_depth_is_unknown():
    args=list(scene());args[3][:]=False
    assert locate(*args)['cells'] is None


def test_partial_floor_cannot_be_declared_empty():
    args=list(scene());args[3][:,:25]=False
    assert locate(*args)['status']=='UNKNOWN'


def test_wrong_floor_color_is_unknown():
    args=list(scene());args[2]=np.zeros_like(args[2])
    assert locate(*args)['status']=='UNKNOWN'


def test_floor_outside_height_envelope_is_unknown():
    args=list(scene());args[1][...,2]+=.02
    assert locate(*args)['status']=='UNKNOWN'


def test_rotated_floor_uses_visual_orientation_not_world_aabb():
    args=list(scene())
    angle=.25;c,s=np.cos(angle),np.sin(angle)
    rotation=np.array([[c,-s],[s,c]])
    args[1][...,:2]=(args[1][...,:2]-[12.5,0.])@rotation.T+[12.5,0.]
    args[4]=(12.,14.,-.3,.3)
    result=locate(*args)
    assert result['status']=='RESOLVED'
    assert abs(result['observed_yaw_rad']-angle)<1e-9
    assert np.allclose(result['centre_xy_m'],[12.5,0.])


def test_catalog_rim_missing_is_unknown_not_fitted_from_partial_floor():
    args=list(scene());args[0]['rim_height_m']=.055
    result=locate(*args)
    assert result['status']=='UNKNOWN' and result['reason']=='TRAY_RIM_NOT_OBSERVABLE'


def test_visible_rim_fits_outer_size_even_when_floor_edges_are_hidden():
    args=list(scene());args[0]['rim_height_m']=.055
    args[1][:2,...,2]+=.055
    args[1][-2:,...,2]+=.055
    args[1][:,:2,2]+=.055
    args[1][:,-2:,2]+=.055
    assert locate(*args)['status']=='RESOLVED'
