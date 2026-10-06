"""Restricted stock recognition contracts; synthetic, not physical acceptance."""
import sys
from pathlib import Path

import numpy as np
import pytest

ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/'experiments'),str(ROOT/'src')]
from probe_candidate_multi_loading import stock_observation,pv


@pytest.fixture
def stock(monkeypatch):
    h,w=pv.H,pv.W
    x,y=np.meshgrid(np.linspace(-.065,.065,w),np.linspace(-.23,.23,h))
    world=np.stack([x,y,np.full((h,w),.82)],axis=-1)
    rgb=np.full((h,w,3),100.)
    depth=np.ones((h,w))
    monkeypatch.setattr(pv,'unproject',lambda m,d,dep,**kw:(
        world.copy(),np.isfinite(dep)&(dep>0),(1.,1.)))
    def read():return stock_observation(None,None,rgb,depth,.82,(-.065,.065,-.23,.23))
    return world,rgb,depth,read


def test_multi_instance_stock_uses_pixels_not_requested_order(stock):
    world,rgb,depth,read=stock
    for row in (40,180):
        world[row:row+16,100:199,2]=.87
        rgb[row:row+16,100:199]=[240,10,10]
    result=read()
    assert result['status']=='RESOLVED'
    assert len(result['parts'])==2 and all(p['cls']=='red' for p in result['parts'])


def test_missing_stock_depth_unknown(stock):
    world,rgb,depth,read=stock
    depth[:]=0
    assert read()['status']=='UNKNOWN'


def test_elevated_blue_robot_surface_not_inventory(stock):
    world,rgb,depth,read=stock
    world[110:126,100:174,2]=1.3
    rgb[110:126,100:174]=[10,10,240]
    result=read()
    assert result['status']=='RESOLVED' and result['parts']==[]


def test_tiny_chromatic_stock_fragment_refused(stock):
    world,rgb,depth,read=stock
    world[110:113,100:103,2]=.87
    rgb[110:113,100:103]=[240,10,10]
    assert read()['status']=='UNKNOWN'


def test_shape_mismatch_unknown(stock):
    world,rgb,depth,read=stock
    world[110:126,100:174,2]=.87
    rgb[110:126,100:174]=[240,10,10]  # 30mm wide, NOT the declared 40mm red box
    assert read()['reason']=='STOCK_COMPONENT_UNRESOLVED'
