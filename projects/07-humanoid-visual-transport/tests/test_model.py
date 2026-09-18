import mujoco
import numpy as np
import pytest
from humanoid007.runtime import ROOT,verify_assets


@pytest.fixture(scope='module')
def model():
    if not (ROOT/'assets/combined.xml').exists():pytest.skip('Run setup first')
    return mujoco.MjModel.from_xml_path(str(ROOT/'assets/combined.xml'))


def test_bilateral_hands_are_actuated(model):
    ids=[model.actuator(side+p+'a'+str(i)).id
         for side in ('lh_','rh_') for p in ('ff','mf','rf','th') for i in range(4)]
    assert len(set(ids))==32
    assert np.all(model.actuator_ctrlrange[ids,1]>model.actuator_ctrlrange[ids,0])


def test_camera_attached_to_moving_head(model):
    assert model.camera('eyes').bodyid==model.body('LINK_HEAD_YAW').id
    assert model.joint('J23_HEAD_PITCH').range[0]<0<model.joint('J23_HEAD_PITCH').range[1]
    assert model.joint('J24_HEAD_YAW').range[0]<0<model.joint('J24_HEAD_YAW').range[1]


def test_free_robot_and_payload(model):
    assert model.jnt_type[0]==mujoco.mjtJoint.mjJNT_FREE
    assert model.joint('payload_free').type==mujoco.mjtJoint.mjJNT_FREE
    assert model.body('payload').mass[0]==pytest.approx(.21)
    assert model.body_mocapid[model.body('payload').id]==-1
    for i in range(model.neq):
        if model.eq_type[i]==mujoco.mjtEq.mjEQ_WELD:
            assert model.body('payload').id not in (model.eq_obj1id[i],model.eq_obj2id[i])


def test_verified_assets(model):
    assert verify_assets()['hand_revision']=='8161bba264d7fa7c99ca301e91e7fb44737676ad'
