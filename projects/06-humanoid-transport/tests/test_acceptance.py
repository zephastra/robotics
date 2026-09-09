from types import SimpleNamespace
import numpy as np
import pytest
from humanoid006.task import DESTINATION, Mission, grasped, retained, placed


def observation():
    return dict(payload=DESTINATION.copy(), relative=np.zeros(3),
                forces=dict(ff=0., mf=0., rf=0., th=0.), target=True, speed=0.)


def test_grasp_requires_opposed_contacts():
    obs = observation()
    obs['forces'].update(mf=.3, rf=.3)
    assert not grasped(obs)
    obs['forces']['th'] = .1
    assert grasped(obs)


def test_retention_requires_contacts_and_local_slip_bound():
    obs = observation()
    obs['forces'].update(mf=.3, rf=.3)
    assert retained(obs, np.zeros(3))
    assert not retained(obs, None)
    obs['relative'][0] = .026
    assert not retained(obs, np.zeros(3))
    obs['relative'][:] = 0
    obs['forces']['mf'] = 0
    assert not retained(obs, np.zeros(3))


@pytest.mark.parametrize('failure', ['off_xy', 'off_z', 'no_support', 'held', 'moving'])
def test_placement_rejects_false_success(failure):
    obs = observation()
    assert placed(obs)
    if failure == 'off_xy': obs['payload'][0] += .061
    if failure == 'off_z': obs['payload'][2] += .013
    if failure == 'no_support': obs['target'] = False
    if failure == 'held': obs['forces']['mf'] = .1
    if failure == 'moving': obs['speed'] = .03
    assert not placed(obs)


def test_stability_timer_resets():
    mission = Mission.__new__(Mission)
    mission.r = SimpleNamespace(d=SimpleNamespace(time=0.))
    mission.stable_since = None
    assert not mission.stable(True, 1.)
    mission.r.d.time = .8
    assert not mission.stable(False, 1.)
    mission.r.d.time = 1.2
    assert not mission.stable(True, 1.)
    mission.r.d.time = 2.3
    assert mission.stable(True, 1.)


def test_finish_is_terminal():
    mission = Mission.__new__(Mission)
    mission.r = SimpleNamespace(d=SimpleNamespace(time=1.))
    mission.events = []
    mission.finish('FAILED', 'TEST_FAILURE')
    mission.update()  # Must not inspect physics after termination.
    assert mission.result == 'FAILED'
    assert mission.events[-1]['phase'] == 'FAILED'
