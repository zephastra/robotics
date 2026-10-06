import sys
from pathlib import Path
import xml.etree.ElementTree as ET

import pytest
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'experiments'))
import build_p5_candidate_world as candidate


@pytest.fixture(scope='module')
def built():
    original = candidate.SOURCE.read_bytes()
    text, declarations = candidate.build(original.decode(), stock_offset_x=0.50)
    assert candidate.SOURCE.read_bytes() == original
    return text, declarations


def test_candidate_is_static_only_and_preserves_source(built):
    text, declarations = built
    result = candidate.audit(text, declarations)
    assert result['diagnostic_result'] == 'PASS'
    assert result['v1_complete'] is False
    assert result['full_order'] == 'NOT_RUN'
    assert result['two_tray_buffer_delivery'] == 'NOT_RUN'
    assert result['dynamic_grasp'] == 'NOT_RUN'
    root = ET.fromstring(text)
    decks = [root.find(f"worldbody/body[@name='{chassis}']/body[@name='{deck}']")
             for chassis, deck in [('n_base_link', 'c_deck'), ('n2_base_link', 'c2_deck')]]
    assert ET.tostring(candidate.renamed_tree(decks[0], 'c_', 'c2_')) == ET.tostring(decks[1])


def test_missing_second_deck_actuator_cannot_pass(built):
    text, declarations = built
    root = ET.fromstring(text)
    acts = root.find('actuator')
    actuator = acts.find("general[@name='c2_deck_roller_0']")
    index = list(acts).index(actuator)
    acts.remove(actuator)
    key = root.find('keyframe/key')
    controls = key.get('ctrl').split()
    controls.pop(index)
    key.set('ctrl', ' '.join(controls))
    result = candidate.audit(ET.tostring(root, encoding='unicode'), declarations)
    assert result['checks']['second_deck_has_actuation'] == 'FAIL'
    assert result['diagnostic_result'] == 'FAIL'


def test_short_receiver_cannot_claim_double_capacity(built):
    text, declarations = built
    root = ET.fromstring(text)
    first = root.find("worldbody/body[@name='c_recv_roller_0']")
    start = candidate.vector(first)[0]
    for body in root.findall('worldbody/body'):
        if body.get('name', '').startswith('c_recv_roller_'):
            position = candidate.vector(body)
            position[0] = start + (position[0] - start) * 13 / 19
            candidate.setvector(body, position)
    result = candidate.audit(ET.tostring(root, encoding='unicode'), declarations)
    assert result['checks']['two_conservative_longitudinal_slots_fit'] == 'FAIL'
    assert result['diagnostic_result'] == 'FAIL'


def test_physical_marker_cannot_be_hidden_as_optical_annotation():
    original = candidate.SOURCE.read_text()
    changed = original.replace('contype="0" conaffinity="0"', 'contype="1" conaffinity="1"')
    assert changed != original
    with pytest.raises(ValueError, match='nonphysical'):
        candidate.build(changed)


def test_old_stock_layout_interference_is_rejected():
    text, declarations = candidate.build(candidate.SOURCE.read_text(), stock_offset_x=0.30)
    result = candidate.audit(text, declarations)
    assert result['checks']['stock_table_does_not_penetrate_arm_at_home'] == 'FAIL'
    assert result['diagnostic_result'] == 'FAIL'
