import argparse
import mujoco
import numpy as np
from hand005.app import run
from hand005.core import OPEN, GRASP
from hand005.model import load, verify_assets


def test_model_has_real_fingers_free_object_and_no_attachment():
    verify_assets()
    m, d = load()
    assert m.nu == 18
    assert m.neq == 0
    assert m.joint('object_free').type[0] == mujoco.mjtJoint.mjJNT_FREE
    for pose in [OPEN, GRASP]:
        assert np.all(pose >= m.actuator_ctrlrange[:16, 0])
        assert np.all(pose <= m.actuator_ctrlrange[:16, 1])


def test_offset_initialization_is_reproducible():
    m, first = load(seed=7, offset=.002)
    _, second = load(seed=7, offset=.002)
    np.testing.assert_array_equal(first.qpos, second.qpos)
    center = first.body('object').xpos
    assert np.all(np.abs(center[:2]-[.06, 0.]) <= .002)


def test_full_contact_manipulation():
    report, _ = run(argparse.Namespace(mode='demo', headless=True, keep_open=False,
                                       snapshot=False, duration=35., seed=0, offset=0., mass=.05))
    assert report['status'] == 'COMPLETED'
    trial = report['trials'][0]
    assert trial['max_lift_m'] >= .09
    assert [event['phase'] for event in trial['events']] == [
        'SETTLE', 'CLOSE', 'VERIFY_GRASP', 'LIFT', 'HOLD', 'TRANSFER',
        'LOWER', 'RELEASE', 'VERIFY_PLACE', 'COMPLETED']
    assert report['final_observation']['tray_contact']
    assert sum(report['final_observation']['finger_forces'].values()) < .02
