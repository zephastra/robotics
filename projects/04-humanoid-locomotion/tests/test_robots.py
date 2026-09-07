from pathlib import Path
import importlib.util
import mujoco
import numpy as np
import pytest
from humanoid004.app import check_assets
from humanoid004.robots import get_robot, joint_addresses

ROOT = Path(__file__).resolve().parents[1]


def test_unknown_robot_is_not_silently_g1():
    with pytest.raises(ValueError, match='Unknown robot'):
        get_robot('t800pro', ROOT)


def test_missing_t800_has_setup_hint(tmp_path):
    with pytest.raises(FileNotFoundError, match='setup_t800.sh'):
        get_robot('t800', tmp_path)
    assert get_robot('g1', tmp_path).weight == 'policies/motion.pt'


@pytest.fixture
def t800():
    if not (ROOT/'assets/manifest-t800.json').exists() or importlib.util.find_spec('MNN') is None:
        pytest.skip('Optional T800 resources not installed')
    return get_robot('t800', ROOT)


def test_t800_model_provenance_and_joint_addresses(t800):
    manifest = check_assets('t800')
    assert manifest['commit'] == '335c60e88772c26c7852d0abd6b3c7439037dd8f'
    assert manifest['sha256'][t800.weight] == 'cbcb90f86dbb2fde39bdc5a25c8d0530d5c79c7a8f84b1f90863d8c9065b6427'
    model = mujoco.MjModel.from_xml_path(str(ROOT/t800.scene))
    qadr, vadr = joint_addresses(model, t800)
    assert (model.nq, model.nv, model.nu) == (32, 31, 25)
    assert len(set(qadr)) == len(set(vadr)) == 25
    assert np.all(model.actuator_ctrllimited)
    with pytest.raises(ValueError, match='dimensions'):
        joint_addresses(model, get_robot('g1', ROOT))


def test_t800_history_and_command_layout(t800):
    policy = t800.policy(ROOT)
    assert list(policy.active) == list(range(12))+list(range(13, 23))
    q, dq, quat = policy.default.copy(), np.zeros(25), np.array([1., 0, 0, 0])
    obs = policy.observation(q, dq, quat, np.zeros(3), np.array([.2, .1, .3]))
    assert obs.shape == (1083,)
    np.testing.assert_allclose(obs[-3:], [.4, .2, .3])
    frames = obs[:-3].reshape(15, 72)
    np.testing.assert_allclose(frames, np.tile(frames[0], (15, 1)))
    np.testing.assert_allclose(frames[0, -3:], [0, 0, -1])
    q[0] += .2
    obs2 = policy.observation(q, dq, quat, np.zeros(3), np.zeros(3))
    frames2 = obs2[:-3].reshape(15, 72)
    assert frames2[-1, 0] == pytest.approx(.2)
    np.testing.assert_array_equal(frames2[:-1], frames[1:])


def test_t800_inference_inactive_joints_and_finite_guard(t800):
    policy = t800.policy(ROOT)
    q, dq, quat = policy.default.copy(), np.zeros(25), np.array([1., 0, 0, 0])
    policy.infer(q, dq, quat, np.zeros(3), np.zeros(3), 0)
    assert policy.last_action.shape == (22,)
    assert np.all(np.isfinite(policy.torques(q, dq)))
    np.testing.assert_array_equal(policy.target[[12, 23, 24]], policy.default[[12, 23, 24]])
    with pytest.raises(FloatingPointError):
        policy.infer(q, dq, quat, np.array([np.nan, 0, 0]), np.zeros(3), .01)


def test_manifests_are_separate(t800):
    g1 = check_assets('g1')
    t8 = check_assets('t800')
    assert not set(g1['sha256']).intersection(t8['sha256'])


def test_robot_specific_keys_preserve_g1():
    from humanoid004.app import Keys
    g1, t8 = Keys(), Keys('t800')
    for key, gvalue, tvalue, axis in [('S', -.12, -.3, 0), ('A', .4, .6, 2), ('Q', .12, .2, 1)]:
        g1.callback(ord(key))
        t8.callback(ord(key))
        assert g1.read()[0][axis] == gvalue
        assert t8.read()[0][axis] == tvalue
