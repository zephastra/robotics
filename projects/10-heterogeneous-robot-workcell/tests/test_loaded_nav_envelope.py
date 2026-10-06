import json
from pathlib import Path
import sys
import numpy as np
import pytest

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'experiments'))
from loaded_nav_envelope import combined_envelope,observed_load_gate
from make_loaded_nav_profile import generate


def test_union_encloses_every_declared_centre_and_orientation():
    points=[[-.2,-.3],[2.5,-.3],[2.5,.3],[-.2,.3]]
    box=np.asarray(combined_envelope(points,[1.9,-.125],[2.47,.125],.335,.04))
    for x in (1.9,2.47):
        for y in (-.125,.125):
            for angle in np.linspace(0,2*np.pi,73):
                edge=np.array([x,y])+.375*np.array([np.cos(angle),np.sin(angle)])
                assert np.all(edge>=box.min(axis=0)-1e-12) and np.all(edge<=box.max(axis=0)+1e-12)
    assert np.all(box.min(axis=0)<=np.min(points,axis=0))


def evidence():
    return dict(source='RGBD',status='RESOLVED',support='deck',count_verified=True,
        observed_sim_s=10.,observed_wall_s=20.,tray_in_base_xy=[2.2,0.],position_uncertainty_m=.01)


CONTRACT=dict(centre_low=[1.9,-.125],centre_high=[2.47,.125],max_sim_age_s=.5,max_wall_age_s=.5)


def test_fresh_in_region_observation_is_allowed():
    assert observed_load_gate(evidence(),CONTRACT,now_sim_s=10.1,now_wall_s=20.1)['allowed']


@pytest.mark.parametrize('key,value',[('source','MUJOCO_TRUTH'),('status','UNKNOWN'),('support','source_band'),
    ('count_verified',False),('observed_sim_s',9.),('observed_wall_s',19.),('observed_wall_s',21.),
    ('tray_in_base_xy',[2.47,0.]),('position_uncertainty_m',float('nan')),('tray_in_base_xy',[2.2,float('nan')])])
def test_unsafe_or_stale_observation_cannot_grant_motion(key,value):
    observation=evidence();observation[key]=value
    assert not observed_load_gate(observation,CONTRACT,now_sim_s=10.1,now_wall_s=20.1)['allowed']


def test_missing_measurement_is_unknown_not_order_based_guess():
    observation=evidence();del observation['tray_in_base_xy'];observation['order_counts']={'red':2,'blue':1}
    assert observed_load_gate(observation,CONTRACT,now_sim_s=10.1,now_wall_s=20.1)['reason_code']=='LOAD_ENVELOPE_UNKNOWN'


def test_generated_loaded_profile_remains_not_run():
    files=generate();profile=json.loads(files['config/joint_world_v5_loaded.profile.json'])
    assert profile['loaded_navigation']=='NOT_RUN' and profile['retention']=='NOT_RUN'
    assert profile['runtime_gate'].endswith('NOT_WIRED') and profile['v1_complete'] is False
    assert profile['footprint'][0][1]<-.46
