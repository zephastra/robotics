import sys
from pathlib import Path
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'experiments'))
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'src'))
from cargo_footprint_judge import contained


def test_center_inside_does_not_prove_entire_cargo_inside():
    points=np.array([[-.02,-.02,0.],[.09,.02,.05]])
    assert contained(points,np.zeros(3),np.eye(3),np.array([.065,.23,.005]),-.075,.075)['status']=='FAIL'


def test_orientation_is_the_actual_tray_frame():
    angle=.4;c,s=np.cos(angle),np.sin(angle)
    rotation=np.array([[c,-s,0],[s,c,0],[0,0,1.]])
    centre=np.array([12.,.2,.82]);local=np.array([[-.02,-.02,0],[.02,.02,.05]])
    assert contained(local@rotation.T+centre,centre,rotation,np.array([.065,.23,.005]),-.075,.075)['status']=='PASS'


def test_missing_geometry_does_not_pass():
    assert contained([],np.zeros(3),np.eye(3),np.ones(3),-.1,.1)['status']=='UNKNOWN'


def test_invalid_rotation_does_not_pass():
    reflected=np.diag([-1.,1.,1.])
    assert contained([[0.,0.,0.]],np.zeros(3),reflected,np.ones(3),-.1,.1)['status']=='UNKNOWN'


def test_invalid_frame_dimensions_do_not_raise_or_pass():
    assert contained([[0.,0.,0.]],np.zeros(2),np.eye(3),np.ones(3),-.1,.1)['status']=='UNKNOWN'
