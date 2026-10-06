"""Candidate generation and posture contracts, not navigation acceptance."""
import json
import sys
from pathlib import Path
from types import SimpleNamespace
import numpy as np
import pytest
import yaml
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'experiments'))
from make_joint_nav_profile import POSTURE, transport_bounds, posture_check, generate


def fake(value=0.):
    names = list(POSTURE)
    model = SimpleNamespace(joint=lambda name: SimpleNamespace(id=names.index(name)), jnt_qposadr=np.arange(5))
    data = SimpleNamespace(qpos=np.full(5, value))
    return model, data


def test_valid_candidate_posture():
    assert posture_check(*fake())['allowed'] is True


@pytest.mark.parametrize('index', range(5))
def test_every_posture_joint_can_refuse(index):
    model, data = fake(); data.qpos[index] = POSTURE[list(POSTURE)[index]][1] + .001
    result = posture_check(model, data)
    assert result['allowed'] is False and result['reason_code'] == 'TRANSPORT_POSTURE_UNCONFIRMED'


def test_nonfinite_posture_refused():
    assert not posture_check(*fake(np.nan))['allowed']


def test_conservative_guarded_envelope_contains_original_and_pusher_extension():
    low, high = np.array([-.19, -.3, -.04]), np.array([2.51, .3, .876])
    outer_low, outer_high = transport_bounds(low, high)
    assert np.all(outer_low < low) and np.all(outer_high > high)
    assert outer_high[0] > high[0] + .391


def test_generated_config_uses_full_footprint_and_known_spawn_not_origin():
    files = generate(); profile = json.loads(files['config/joint_world_v5.profile.json'])
    params = yaml.safe_load(files['config/nav2_joint_world_v5.yaml'])
    assert profile['start'][0] == pytest.approx(4.41372)
    assert profile['initialized_posture']['allowed'] is True
    assert max(point[0] for point in profile['footprint']) > 2.9
    for key in ('local_costmap', 'global_costmap'):
        section = params[key][key]['ros__parameters']
        assert 'robot_radius' not in section
        assert json.loads(section['footprint']) == profile['footprint']
    monitor = params['collision_monitor']['ros__parameters']
    assert monitor['FootprintApproach']['footprint_topic'] == 'local_costmap/published_footprint'
    assert profile['cargo_envelope'].startswith('NOT_COMMISSIONED')
