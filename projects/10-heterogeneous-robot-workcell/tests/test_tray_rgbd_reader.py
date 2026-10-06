"""Synthetic reader contracts, NOT physical-camera acceptance."""
import sys
from pathlib import Path

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT/'experiments'),str(ROOT/'src')]
import tray_rgbd_reader as reader


@pytest.fixture
def scene(monkeypatch):
    h,w = reader.pv.H,reader.pv.W
    x,y = np.meshgrid(np.linspace(-.065,.065,w),np.linspace(-.23,.23,h))
    world = np.stack([x,y,np.full((h,w),.82)],axis=-1)
    def unproject(model,data,depth,**kwargs):
        return world.copy(),np.isfinite(depth)&(depth>0),(1.,1.)
    monkeypatch.setattr(reader.pv,'unproject',unproject)
    cells = dict(axis=1,deck=dict(centre=np.array([0.,0.,.815]),
        size=np.array([.065,.23,.005]),top_z=.82,x_span=(-.065,.065),y_span=(-.23,.23)),
        cells=[dict(index=i,lo=lo,hi=hi) for i,(lo,hi) in enumerate(
            ((-.23,-.0767),(-.0767,.0767),(.0767,.23)))])
    rgb = np.full((h,w,3),100.)
    depth = np.ones((h,w))
    def run(sizes=None):
        return reader.read(None,None,rgb,depth,cells,
                           np.array([.02,.015,.025]) if sizes is None else sizes,{})
    return world,rgb,depth,run


def test_observed_empty_not_missing(scene):
    world,rgb,depth,run = scene
    assert run()['counts']=={'red':0,'blue':0}
    depth[:]=0
    assert run()['status']=='UNKNOWN'
    assert run()['counts'] is None


def test_independently_rotated_part_retains_original_shape_threshold(scene):
    world,rgb,depth,_=scene
    rows,cols=slice(110,126),slice(100,199)
    x,y=np.meshgrid(np.linspace(-.02,.02,99),np.linspace(-.015,.015,16))
    angle=.4;c,s=np.cos(angle),np.sin(angle)
    world[rows,cols,0]=c*x-s*y
    world[rows,cols,1]=s*x+c*y
    world[rows,cols,2]=.87;rgb[rows,cols]=[240,10,10]
    cells=dict(axis=1,observed_yaw_rad=0.,deck=dict(centre=np.array([0.,0.,.815]),
        size=np.array([.065,.23,.005]),top_z=.82,x_span=(-.065,.065),y_span=(-.23,.23)),
        cells=[dict(index=i,lo=lo,hi=hi) for i,(lo,hi) in enumerate(
            ((-.23,-.0767),(-.0767,.0767),(.0767,.23)))])
    result=reader.read_cloud(rgb,depth,world,depth>0,cells,np.array([.02,.015,.025]))
    assert result['status']=='RESOLVED' and result['counts']=={'red':1,'blue':0}
    assert result['parts'][0]['size_error_m']<1e-9


def test_count_and_cell_without_labels(scene):
    world,rgb,depth,run = scene
    world[110:126,100:199,2]=.87
    rgb[110:126,100:199]=[240,10,10]
    answer = run()
    assert answer['status']=='RESOLVED'
    assert answer['counts']=={'red':1,'blue':0}
    assert answer['parts'][0]['cell']==1


def test_tiny_fragment_is_unknown_not_empty(scene):
    world,rgb,depth,run = scene
    world[110:113,100:103,2]=.87
    rgb[110:113,100:103]=[240,10,10]
    assert run()['reason']=='UNRESOLVED_COLORED_FRAGMENT'
    assert run()['counts'] is None


def test_occluded_cell_unknown_not_partial_count(scene):
    world,rgb,depth,run = scene
    world[world[...,1]<-.0767,2]=1.1
    assert run()['reason']=='OCCLUDER_ABOVE_CONTENT_ENVELOPE'
    assert run()['counts'] is None


def test_cyan_tray_is_not_blue_inventory(scene):
    world,rgb,depth,run = scene
    rgb[:]=[40,140,220]
    assert run()['counts']=={'red':0,'blue':0}


def test_malformed_input_unknown(scene):
    world,rgb,depth,run = scene
    rgb[0,0,0]=np.nan
    assert run()['reason']=='INVALID_RGB'
    assert reader.read(None,None,rgb,depth[:1],{},[],{})['reason']=='INVALID_FRAME_SHAPE'


def test_blue_class_specific_size_without_order_input(scene):
    world,rgb,depth,run=scene
    world[110:126,100:174,2]=.87
    rgb[110:126,100:174]=[10,10,240]
    answer=run({'red':np.array([.02,.015,.025]),'blue':np.array([.015,.015,.025])})
    assert answer['counts']=={'red':0,'blue':1}
    assert answer['parts'][0]['cell']==1


def test_multiple_same_class_instances_not_order_guess(scene):
    world,rgb,depth,run=scene
    for row in (40,180):
        world[row:row+16,100:199,2]=.87
        rgb[row:row+16,100:199]=[240,10,10]
    answer=run()
    assert answer['counts']=={'red':2,'blue':0}
    assert sorted(p['cell'] for p in answer['parts'])==[0,2]
