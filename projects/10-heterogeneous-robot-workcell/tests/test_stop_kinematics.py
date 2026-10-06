import sys
from pathlib import Path

import mujoco
import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'experiments'))
from probe_stop_kinematics import points, measured_stop_window


def model_for(geoms):
    model = mujoco.MjModel.from_xml_string(
        '<mujoco><worldbody><body name="tray"><freejoint/>'
        + geoms + '</body></worldbody></mujoco>')
    return model, model.body('tray').id


def test_rotated_offset_box_and_visual_exclusion():
    model, body = model_for(
        '<geom type="box" size=".1 .2 .3" pos="1 2 3" '
        'quat=".7071067811865476 0 0 .7071067811865476"/>'
        '<geom type="box" size="10 10 10" mass="0" '
        'contype="0" conaffinity="0"/>')
    vertices = points(model, body)
    assert vertices.shape == (8, 3)
    np.testing.assert_allclose(vertices.min(axis=0), [.8, 1.9, 2.7])
    np.testing.assert_allclose(vertices.max(axis=0), [1.2, 2.1, 3.3])


@pytest.mark.parametrize('kind,size,expected', [
    ('sphere', '.2', [.2, .2, .2]),
    ('capsule', '.2 .4', [.2, .2, .6]),
    ('cylinder', '.2 .4', [.2, .2, .4]),
])
def test_round_primitive_enclosing_box(kind, size, expected):
    model, body = model_for(f'<geom type="{kind}" size="{size}"/>')
    vertices = points(model, body)
    np.testing.assert_allclose(vertices.max(axis=0), expected)
    np.testing.assert_allclose(vertices.min(axis=0), -np.asarray(expected))


def test_unsupported_geometry_is_not_silently_accepted():
    model, body = model_for('<geom type="ellipsoid" size=".1 .2 .3"/>')
    with pytest.raises(ValueError, match='unsupported physical primitive'):
        points(model, body)


def test_refused_transfer_cannot_reuse_initial_settling_as_stop():
    assert measured_stop_window([{'time_s': .1}, {'time_s': .2}],
                                {'state': 'UNKNOWN'}, .2) == []


def test_measurement_excludes_braking_before_held_window():
    trace = [{'time_s': t} for t in [.1, .8, .802, 1.3]]
    assert measured_stop_window(trace, {'brake': {'stopped_confirmed': False}}, 1.3) == trace[2:]
