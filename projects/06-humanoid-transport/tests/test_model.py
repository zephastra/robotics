import mujoco
import numpy as np
import pytest
from humanoid006.runtime import ROOT, verify_assets


@pytest.fixture(scope='module')
def model():
    if not (ROOT/'assets/combined.xml').exists():
        pytest.skip('Run setup to generate the physical model')
    return mujoco.MjModel.from_xml_path(str(ROOT/'assets/combined.xml'))


def test_payload_is_free_and_has_real_mass(model):
    assert model.joint('payload_free').type == mujoco.mjtJoint.mjJNT_FREE
    assert model.body('payload').mass[0] == pytest.approx(.05)
    assert model.geom('payload_geom').size[0] == pytest.approx(.035)
    assert model.body_mocapid[model.body('payload').id] == -1
    payload = model.body('payload').id
    for i in range(model.neq):
        if model.eq_type[i] == mujoco.mjtEq.mjEQ_WELD:
            assert payload not in (model.eq_obj1id[i], model.eq_obj2id[i])


def test_articulated_hand_and_floating_base(model):
    assert model.jnt_type[0] == mujoco.mjtJoint.mjJNT_FREE
    ids = [model.actuator('rh_'+p+'a'+str(i)).id for p in ('ff','mf','rf','th') for i in range(4)]
    assert len(set(ids)) == 16
    assert np.all(model.actuator_ctrlrange[ids,1] > model.actuator_ctrlrange[ids,0])


def test_assets_match_manifest(model):
    assert verify_assets()['hand_revision'] == '8161bba264d7fa7c99ca301e91e7fb44737676ad'
