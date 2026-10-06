import sys
from pathlib import Path
import xml.etree.ElementTree as ET

import pytest
import mujoco

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'experiments'))
import build_support_candidate as support


def source(pitch=.08):
    return f'''<mujoco><worldbody>
    <body name="c_fixed_roller_0_0" pos="0 0 .45">
      <joint name="j0" axis="0 1 0"/>
      <geom name="g0" type="cylinder" size=".035 .25"
        quat=".707107 .707107 0 0" mass=".4"/>
    </body>
    <body name="c_fixed_roller_0_1" pos="{pitch} 0 .45">
      <joint name="j1" axis="0 1 0"/>
      <geom name="g1" type="cylinder" size=".035 .25"
        quat=".707107 .707107 0 0" mass=".4"/>
    </body></worldbody></mujoco>'''


def test_passive_idler_preserves_original_geometries():
    original = ET.fromstring(source())
    text, pairs = support.build(source())
    new = ET.fromstring(text)
    for body in original.find('worldbody'):
        assert ET.tostring(new.find(f"worldbody/body[@name='{body.get('name')}']")) == ET.tostring(body)
    added = new.find("worldbody/body[@name='c_fixed_roller_idler_0']")
    assert added.find('joint').get('damping') == '0'
    assert added.find('joint').get('frictionloss') == '0'
    assert added.find('geom').get('contype', '1') != '0'
    assert pairs[0]['neighbour_clearance_m'] > 0
    assert pairs[0]['support_pitch_m'] == pytest.approx(.04)
    assert pairs[0]['crown_z_m'] == pytest.approx(.485)


def test_intersecting_idler_is_refused():
    with pytest.raises(ValueError, match='intersect'):
        support.build(source(.06))


def test_missing_source_row_is_refused():
    with pytest.raises(ValueError, match='missing'):
        support.build('<mujoco><worldbody/></mujoco>')


def test_split_support_has_two_independent_passive_contacts():
    text, pairs = support.build(source(), split=True)
    root = ET.fromstring(text)
    added = [b for b in root.find('worldbody') if 'idler' in b.get('name', '')]
    assert len(added) == len(pairs) == 2
    ys = sorted(float(b.get('pos').split()[1]) for b in added)
    assert ys == pytest.approx([-.125, .125])
    assert all(float(b.find('geom').get('size').split()[1]) == .0625 for b in added)


def test_integrated_key_preserves_old_joint_states_and_passive_home():
    original = source().replace('</mujoco>',
        '<keyframe><key name="home" qpos=".12 .34" qvel=".01 .02"/></keyframe></mujoco>')
    candidate, _ = support.build(original, split=True)
    mapped = support.preserve_keys(original, candidate)
    model = mujoco.MjModel.from_xml_string(mapped)
    assert model.nkey == 1
    for name, position, velocity in [('j0', .12, .01), ('j1', .34, .02)]:
        joint = model.joint(name).id
        assert model.key_qpos[0, model.jnt_qposadr[joint]] == pytest.approx(position)
        assert model.key_qvel[0, model.jnt_dofadr[joint]] == pytest.approx(velocity)
    for j in range(model.njnt):
        if 'idler' in model.joint(j).name:
            assert model.key_qpos[0, model.jnt_qposadr[j]] == 0
            assert model.key_qvel[0, model.jnt_dofadr[j]] == 0
