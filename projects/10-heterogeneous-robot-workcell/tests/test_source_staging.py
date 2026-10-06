"""Fake contract checks only; not a physical supply/positioning proof."""
import sys
from pathlib import Path
from types import SimpleNamespace
import pytest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'experiments'))
from source_staging import stage_to_loading


class Plant:
    def __init__(self):
        self.data=SimpleNamespace(time=0.)
        self.model=SimpleNamespace(nu=1,ngeom=1,geom_size=[[.035]],
            actuator=lambda i:SimpleNamespace(name='c_fixed_roller_drive'),
            geom=lambda i:SimpleNamespace(name='c_fixed_roller_0_0'))
        self.drives=[];self.stop_ok=True
    def _band_control(self,spin,speed):self.drives.append(speed);return [speed]
    def park_control(self):
        self.drives.append('park');return [0.]
    def _stop_transfer(self):return dict(stopped_confirmed=self.stop_ok)


class Owner:
    def __init__(self,plant):
        self.plant=plant;self.closed=False;self.commands=[]
        self.permit=SimpleNamespace(stop=self.stop)
    def stop(self,*args):self.closed=True
    def step(self,produce):
        if self.closed:raise RuntimeError('safety freeze')
        self.commands.append(produce());self.plant.data.time+=.01


def observation(x):return dict(status='RESOLVED',centre_xy_m=[x,0.],cells={'observed':True})


def test_observed_position_stops_without_driving():
    plant=Plant();owner=Owner(plant)
    result=stage_to_loading(plant,owner,lambda:observation(5.),5.)
    assert result['status']=='RESOLVED' and not plant.drives


def test_unknown_never_generates_motion():
    plant=Plant();owner=Owner(plant)
    with pytest.raises(RuntimeError,match='safety freeze'):
        stage_to_loading(plant,owner,lambda:dict(status='UNKNOWN'),5.)
    assert not plant.drives and plant.data.time==0.


def test_motion_sign_comes_from_observation_not_plant_truth():
    plant=Plant();owner=Owner(plant)
    observations=iter([observation(4.99),observation(5.),observation(5.)])
    assert stage_to_loading(plant,owner,lambda:next(observations),5.)['status']=='RESOLVED'
    assert plant.drives and all(s=='park' for s in plant.drives)
    assert all(command[0]>0 for command in owner.commands)


def test_failed_physical_stop_does_not_resolve_position():
    plant=Plant();plant.stop_ok=False;owner=Owner(plant)
    assert stage_to_loading(plant,owner,lambda:observation(5.),5.)['status']=='UNKNOWN'


def test_motion_during_brake_requires_rejection():
    plant=Plant();owner=Owner(plant)
    observations=iter([observation(5.),observation(5.02)])
    assert stage_to_loading(plant,owner,lambda:next(observations),5.)['status']=='UNKNOWN'
