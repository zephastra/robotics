import copy
import numpy as np
import pytest
from hand005.core import Task, slew, supported_grasp


def observation():
    return dict(object=[.06, 0., .205], relative_object=[.06, 0., -.015],
                speed=0., source_contact=True, tray_contact=False,
                finger_forces=dict(ff=.1, mf=0., rf=.1, th=.1))


def test_slew_limits_each_joint():
    np.testing.assert_allclose(slew(np.zeros(2), np.array([1., -1.]), np.array([2., 3.]), .1), [.2, -.3])


@pytest.mark.parametrize('missing', ['ff', 'rf', 'th'])
def test_grasp_requires_thumb_and_two_fingers(missing):
    obs = observation()
    assert supported_grasp(obs)
    obs['finger_forces'][missing] = 0.
    assert not supported_grasp(obs)


def test_missing_contacts_fail_instead_of_lifting():
    obs = observation()
    obs['finger_forces'] = dict.fromkeys(obs['finger_forces'], 0.)
    task = Task(0., obs)
    task.enter('VERIFY_GRASP', 0.)
    task.command(3.1, obs)
    assert task.result == 'FAILED'
    assert task.reason == 'NO_OPPOSED_GRASP'


def test_dropped_object_fails_immediately():
    obs = observation()
    task = Task(0., obs)
    obs['object'][2] = .02
    task.command(.1, obs)
    assert task.result == 'FAILED'


def test_grasp_loss_during_transfer_fails():
    obs = observation()
    task = Task(0., obs)
    task.enter('TRANSFER', 0.)
    obs['finger_forces']['th'] = 0.
    task.command(.1, obs)
    task.command(.7, obs)
    assert task.reason == 'LOST_FINGER_CONTACT'


def test_insufficient_lift_fails():
    obs = observation()
    task = Task(0., obs)
    task.enter('LIFT', 0.)
    task.command(3.1, obs)
    assert task.reason == 'INSUFFICIENT_LIFT'


def test_place_requires_stable_tray_support_and_no_fingers():
    obs = observation()
    obs.update(object=[.06, .20, .205], source_contact=False, tray_contact=True)
    task = Task(0., obs)
    task.enter('VERIFY_PLACE', 0.)
    task.command(0., obs)
    task.command(1.1, obs)
    assert task.result is None
    obs['finger_forces'] = dict.fromkeys(obs['finger_forces'], 0.)
    task.command(1.2, obs)
    task.command(2.3, obs)
    assert task.result == 'COMPLETED'


def test_failed_condition_resets_stability_timer():
    task = Task(0., observation())
    assert not task.stable(True, 0., 1.)
    assert not task.stable(False, .9, 1.)
    assert not task.stable(True, 1., 1.)
    assert not task.stable(True, 1.5, 1.)
    assert task.stable(True, 2.1, 1.)


def test_task_does_not_mutate_observation():
    obs = observation()
    original = copy.deepcopy(obs)
    Task(0., obs).command(.1, obs)
    assert obs == original
