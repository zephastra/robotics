from pathlib import Path
import sys
import json
import numpy as np
import yaml
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'experiments'))
from make_centered_load_profile import generate,ARTIFACTS
from loaded_nav_envelope import observed_load_gate


def test_centered_candidate_keeps_full_cargo_radius_and_original_profile_bytes():
    path=ROOT/'config/joint_world_v5_loaded.profile.json';before=path.read_bytes()
    result=json.loads(generate()[ARTIFACTS[0]]);parent=json.loads(before)
    assert before==path.read_bytes()
    assert result['cargo_radius_m']==parent['cargo_radius_m']
    assert result['mechanism_allowance_m']==parent['mechanism_allowance_m']
    assert result['observation_contract']['max_wall_age_s']==parent['observation_contract']['max_wall_age_s']
    contract=result['observation_contract']; box=np.asarray(result['footprint'])
    radius=result['cargo_radius_m']+result['mechanism_allowance_m']
    for x in (contract['centre_low'][0],contract['centre_high'][0]):
        for y in (-.06,.06):
            for angle in np.linspace(0,2*np.pi,73):
                point=np.array([x,y])+radius*np.array([np.cos(angle),np.sin(angle)])
                assert np.all(point>=box.min(axis=0)-1e-12) and np.all(point<=box.max(axis=0)+1e-12)


def test_both_costmaps_use_identical_centered_full_envelope():
    generated=generate(); config=yaml.safe_load(generated[ARTIFACTS[1]])
    result=json.loads(generated[ARTIFACTS[0]])
    for name in ('global_costmap','local_costmap'):
        assert config[name][name]['ros__parameters']['footprint']==str(result['footprint'])


def test_centred_contract_refuses_uncertainty_over_boundary():
    contract=json.loads(generate()[ARTIFACTS[0]])['observation_contract']
    observation=dict(source='RGBD',status='RESOLVED',support='deck',count_verified=True,
        observed_sim_s=10.,observed_wall_s=20.,tray_in_base_xy=[2.2,.055],position_uncertainty_m=.01)
    assert not observed_load_gate(observation,contract,now_sim_s=10.,now_wall_s=20.)['allowed']
    observation['tray_in_base_xy'][1]=.04
    assert observed_load_gate(observation,contract,now_sim_s=10.,now_wall_s=20.)['allowed']
