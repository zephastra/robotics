import xml.etree.ElementTree as ET
import numpy as np
from build_hinged_retainer_candidate import build, RELEASE_ANGLE, SOURCE
from build_p5_candidate_world import compile_text


def test_hinged_shoes_release_outside_tray_and_old_actuators_unchanged():
    raw = SOURCE.read_text(); text, report = build(raw)
    root = ET.fromstring(text)
    assert report['qualified'] is False
    old, new = compile_text(raw), compile_text(text)
    np.testing.assert_array_equal(old.key_ctrl, new.key_ctrl[:, :old.nu])
    for side in (-1, 1):
        name = f'c_retainer_{side}'
        body = next(b for b in root.iter('body') if b.get('name') == name)
        assert body.find('joint').get('type') == 'hinge'
        # At 90deg release, toe is at |y|=.255 rather than inside ±.23 tray.
        assert abs(float(body.get('pos').split()[1])) > .23
        joint = new.joint(name+'_joint').id
        assert np.all(new.key_qpos[:, new.jnt_qposadr[joint]] == RELEASE_ANGLE)
