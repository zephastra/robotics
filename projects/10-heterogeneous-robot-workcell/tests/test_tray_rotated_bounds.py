"""Actual collision envelope must follow orientation, not visual geometry."""
import sys
from pathlib import Path
from types import SimpleNamespace

import mujoco
import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'experiments'))
import w4_plant as wp


def test_rotation_envelope_ignores_visual_shapes_and_covers_offset_handles():
    model = mujoco.MjModel.from_xml_string('''<mujoco><worldbody>
      <body name="c_payload"><geom type="box" size=".05 .20 .02"/>
        <geom type="sphere" pos=".30 0 0" size=".035"/>
        <geom type="box" size="2 2 2" contype="0" conaffinity="0" mass="0"/>
      </body></worldbody></mujoco>''')
    assert wp.collision_radius(model, model.body('c_payload').id) == pytest.approx(.335)


def test_rotated_box_bounds_are_not_unrotated_sizes():
    model = mujoco.MjModel.from_xml_string('''<mujoco><worldbody>
      <body name="c_payload" quat="0.9238795325 0 0 0.3826834324">
        <geom type="box" size=".05 .20 .02"/>
        <geom type="box" size="2 2 2" contype="0" conaffinity="0" mass="0"/>
      </body></worldbody></mujoco>''')
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    instrument = SimpleNamespace(model=model, data=data, crown_z=-0.02,
                                 tray_support=lambda: ['receiver_band'],
                                 zone_of=lambda *a: 'receiver_band')
    state = wp.LogisticsPlant.tray_state(instrument)
    expected = (.05 + .20) / np.sqrt(2)
    assert state['x_trailing_m'] == pytest.approx(-expected, abs=1e-8)
    assert state['x_leading_m'] == pytest.approx(expected, abs=1e-8)
    assert state['on_crowns'] is True


@pytest.mark.parametrize('kind,size,expected', [
    ('sphere', '.04', [.04, .04, .04]),
    ('capsule', '.01 .20', [.21, .01, .01]),
    ('cylinder', '.03 .20', [.20, .03, .03]),
])
def test_rotated_primitive_extents(kind, size, expected):
    model = mujoco.MjModel.from_xml_string('''<mujoco><worldbody>
      <body name="c_payload"><geom type="%s" size="%s"
      quat="0.7071067811865476 0 0.7071067811865475 0"/>
      </body></worldbody></mujoco>''' % (kind, size))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    lo, hi = wp.collision_bounds(model, data, model.body('c_payload').id)
    assert hi == pytest.approx(expected, abs=1e-8)
    assert lo == pytest.approx(-np.asarray(expected), abs=1e-8)
