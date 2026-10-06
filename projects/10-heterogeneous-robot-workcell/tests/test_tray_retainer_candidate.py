import re
import xml.etree.ElementTree as ET
import numpy as np
import pytest
from build_tray_retainer_candidate import build, SOURCE, FORCE_LIMIT, RELEASE_TRAVEL
from build_p5_candidate_world import compile_text


@pytest.fixture(scope='module')
def built():
    raw = SOURCE.read_text()
    text, report = build(raw)
    return raw, text, report


def test_existing_geometries_and_contacts_unchanged(built):
    raw, text, report = built
    old = ET.fromstring(re.sub(r'<!--.*?-->', '', raw, flags=re.S))
    new = ET.fromstring(text)
    named = {e.get('name'): e.attrib for e in new.iter('geom') if e.get('name')}
    for geom in old.iter('geom'):
        if geom.get('name'): assert named[geom.get('name')] == geom.attrib
    assert report['qualified'] is False and report['physical_steps'] == 0
    assert not new.findall('equality/weld')


def test_keyframes_old_actuators_and_cargo_freedom_preserved(built):
    raw, text, report = built
    old, new = compile_text(raw), compile_text(text)
    np.testing.assert_array_equal(old.key_ctrl, new.key_ctrl[:, :old.nu])
    assert new.nu == old.nu+2 and new.nq == old.nq+2
    for name in report['added']:
        joint = new.joint(name+'_joint').id
        assert np.all(new.key_qpos[:, new.jnt_qposadr[joint]] == RELEASE_TRAVEL)
        actuator = new.actuator(name+'_drive').id
        np.testing.assert_array_equal(new.actuator_forcerange[actuator], [-FORCE_LIMIT, FORCE_LIMIT])
    assert new.jnt_type[new.joint('c_payload_free').id] == 0


def test_shoes_on_real_rim_and_released_clearance(built):
    _, text, report = built
    root = ET.fromstring(text)
    deck = next(b for b in root.iter('body') if b.get('name') == 'c_deck')
    for name in report['added']:
        shoe = deck.find(f"body[@name='{name}']")
        assert shoe is not None
        bottom = float(shoe.get('pos').split()[2])-float(shoe.find('geom').get('size').split()[2])
        assert bottom-report['rim_top_in_deck_m'] == pytest.approx(RELEASE_TRAVEL)
        assert shoe.find('joint').get('limited') == 'true'
