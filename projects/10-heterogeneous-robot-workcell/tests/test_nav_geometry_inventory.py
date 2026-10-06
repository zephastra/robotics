"""Static/model geometry tests, NOT commissioned Nav2 transport."""
import sys
from pathlib import Path
import numpy as np
import mujoco
import pytest
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'experiments'))
from nav_geometry_inventory import bounds, descendants, classify, robot_bounds


def test_rotated_box_bounds():
    rot = np.array([[0., -1., 0.], [1., 0., 0.], [0., 0., 1.]])
    lo, hi = bounds([1., 2., 3.], rot, [2., 1., .5], mujoco.mjtGeom.mjGEOM_BOX)
    np.testing.assert_allclose(lo, [0., 0., 2.5])
    np.testing.assert_allclose(hi, [2., 4., 3.5])


def test_cylinder_bounds_respect_horizontal_axis():
    rot = np.array([[0., 0., 1.], [0., 1., 0.], [-1., 0., 0.]])
    lo, hi = bounds([0., 0., 0.], rot, [.02, .25, 0.], mujoco.mjtGeom.mjGEOM_CYLINDER)
    np.testing.assert_allclose(hi, [.25, .02, .02])
    np.testing.assert_allclose(lo, -hi)


@pytest.mark.parametrize('rotation', [np.zeros((3, 3)), np.eye(3) * np.nan])
def test_invalid_orientation_refused(rotation):
    with pytest.raises(ValueError): bounds([0., 0., 0.], rotation, [1., 1., 1.], mujoco.mjtGeom.mjGEOM_BOX)


def model():
    m = mujoco.MjModel.from_xml_string('''<mujoco><worldbody>
      <geom name="floor" type="plane" size="10 10 .1"/>
      <body name="fixture" pos="2 0 0"><body name="nested"><geom name="leg" type="box" size=".1 .1 .5"/></body></body>
      <body name="roller" pos="3 0 0"><joint type="hinge" axis="0 0 1"/>
        <geom name="crown" type="cylinder" size=".1 .3"/></body>
      <body name="offaxis" pos="4 0 0"><joint type="hinge" axis="0 0 1"/>
        <geom name="offset_crown" pos=".2 0 0" type="cylinder" size=".1 .3"/></body>
      <body name="n_base_link"><joint type="slide" axis="1 0 0"/>
        <geom name="chassis" type="box" size=".19 .13 .04"/>
        <body name="deck" pos="1.92 0 .45"><geom name="deckframe" type="box" size=".32 .25 .02"/></body>
        <geom name="visual" contype="0" conaffinity="0" type="box" size="100 100 100" mass="0"/>
      </body><body name="cargo"><freejoint/><geom type="sphere" size=".05"/></body>
    </worldbody></mujoco>''')
    d = mujoco.MjData(m); mujoco.mj_forward(m, d)
    return m, d


def test_nested_fixture_and_spinning_cylinder_not_silently_omitted():
    m, d = model(); groups = classify(m, d, robot_roots=['n_base_link'])
    assert 'leg' in groups['static'] and 'crown' in groups['static']
    assert 'offset_crown' in groups['moving']
    assert 'deckframe' in groups['robot'] and 'visual' in groups['visual_only']


def test_attached_deck_extends_footprint_far_beyond_bare_radius():
    m, d = model(); base = m.body('n_base_link').id
    assert m.body('deck').id in descendants(m, base)
    lo, hi = robot_bounds(m, d, 'n_base_link')
    np.testing.assert_allclose(lo, [-.19, -.25, -.04])
    np.testing.assert_allclose(hi, [2.24, .25, .47])
    assert hi[0] > .26


def test_mesh_and_nonfinite_extent_not_guessed():
    with pytest.raises(ValueError): bounds([0., 0., 0.], np.eye(3), [1., 1., 1.], mujoco.mjtGeom.mjGEOM_MESH)
    with pytest.raises(ValueError): bounds([0., 0., 0.], np.eye(3), [1., np.nan, 1.], mujoco.mjtGeom.mjGEOM_BOX)
