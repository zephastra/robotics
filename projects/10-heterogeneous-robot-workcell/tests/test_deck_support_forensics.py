import inspect
import pytest
import mujoco
import numpy as np
from deck_support_forensics import SeparationEpisodes, DeckSupportForensics, cylinder_plane_gap


def test_recovery_duration_and_signed_gap():
    tracker = SeparationEpisodes()
    tracker.sample(1., 2)
    tracker.sample(1.002, 0, -.0001)
    tracker.sample(1.004, 0, .0003)
    tracker.sample(1.006, 1, force=2.)
    row = tracker.summary()['episodes'][0]
    assert row['duration_until_recovery_s'] == pytest.approx(.004)
    assert row['peak_signed_gap_m'] == .0003
    assert row['min_signed_gap_m'] == -.0001
    assert row['censored'] is False


def test_open_episode_is_not_claimed_recovered():
    tracker = SeparationEpisodes()
    tracker.sample(2., 0, .002)
    tracker.sample(2.002, 0, .001)
    row = tracker.summary()['episodes'][0]
    assert row['censored'] and row['recovered_sim_s'] is None
    assert row['observed_missing_span_s'] == pytest.approx(.002)
    assert 'duration_until_recovery_s' not in row


@pytest.mark.parametrize('bad', [0., float('nan'), float('inf')])
def test_bad_time_refused(bad):
    tracker = SeparationEpisodes()
    tracker.sample(1., 1)
    with pytest.raises(ValueError): tracker.sample(bad, 0)


def test_instrument_has_no_physics_or_motor_writes():
    source = inspect.getsource(DeckSupportForensics)
    assert 'mj_step(' not in source and 'mj_forward(' not in source
    assert 'qpos[' not in source and 'ctrl[' not in source
    assert 'mj_geomDistance' in source
    assert tracker_permission_false()


def tracker_permission_false():
    return SeparationEpisodes().summary()['permission_output'] is False


def test_surface_distance_uses_geometry_not_body_height():
    model = mujoco.MjModel.from_xml_string('''<mujoco><worldbody>
      <body name="c_deck"><geom name="c_deck_roller_0" type="cylinder" size=".01 .2" quat=".707107 .707107 0 0"/></body>
      <body name="c_payload" pos="0 0 .023"><freejoint/>
        <geom name="c_tray_floor" type="box" size=".1 .1 .01"/>
      </body></worldbody></mujoco>''')
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)  # Fixture initialization only, not instrument.
    data.time = model.opt.timestep
    before = data.qpos.copy(), data.ctrl.copy(), float(data.time)
    instrument = DeckSupportForensics(model)
    instrument.sample(data)
    measured = instrument.summary()['first_missing']
    assert measured['min_signed_surface_gap_m'] == pytest.approx(.003)
    assert measured['analytic_plane_clearance_m'] == pytest.approx(.003)
    assert measured['cached_state_sim_s'] == 0.
    assert not measured['distance_censored']
    np.testing.assert_array_equal(before[0], data.qpos)
    np.testing.assert_array_equal(before[1], data.ctrl)
    assert before[2] == data.time


def test_analytic_cylinder_clearance_and_footprint():
    args = (np.array([0., 0., .023]), np.eye(3), np.array([.1, .1, .01]))
    assert cylinder_plane_gap(*args, [0, 0, 0], [0, 1, 0], .01, .2) == pytest.approx(.003)
    assert cylinder_plane_gap(*args, [.5, 0, 0], [0, 1, 0], .01, .2) is None
    assert cylinder_plane_gap(*args, [0, 0, .004], [0, 1, 0], .01, .2) == pytest.approx(-.001)
