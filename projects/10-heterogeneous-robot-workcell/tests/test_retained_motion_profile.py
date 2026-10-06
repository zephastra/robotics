import copy
from pathlib import Path
import sys
import yaml
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'experiments'))
from make_retained_motion_profile import candidate


def test_candidate_is_independent_and_does_not_weaken_safety_contracts():
    root=Path(__file__).resolve().parents[1]
    original=yaml.safe_load((root/'config/nav2_joint_world_v5_centered.yaml').read_text())
    snapshot=copy.deepcopy(original);result=candidate(original)
    assert original==snapshot
    for name in original:
        if name not in ('controller_server','velocity_smoother'):assert result[name]==original[name]
    smooth=result['velocity_smoother']['ros__parameters']
    assert smooth['max_accel']==[.05,0.,.01]
    assert smooth['max_decel']==[-.05,0.,-.01]
    controller=result['controller_server']['ros__parameters']
    for name in controller['goal_checker_plugins']:
        assert controller[name]['xy_goal_tolerance']==.08<.12


def test_generator_does_not_claim_physical_qualification():
    source=(Path(__file__).resolve().parents[1]/'experiments/make_retained_motion_profile.py').read_text()
    assert 'acceptance_changes=[]' in source and 'qualified=False' in source
