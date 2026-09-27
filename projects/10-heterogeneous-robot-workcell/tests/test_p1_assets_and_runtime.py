"""Asset integrity and the humanoid runtime: both must be able to fail.

Structural checks only. These do not assert anything about physics.
"""
import hashlib
import json
from pathlib import Path
import sys

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src'))

from humanoid007 import runtime as rt  # noqa: E402


def build_tree(tmp_path, files, listed=None):
    """A miniature asset tree. `listed` defaults to every file given."""
    manifest = {'sha256': {}}
    for name, content in files.items():
        path = tmp_path / name
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(content, encoding='utf-8')
        if listed is None or name in listed:
            manifest['sha256'][name] = hashlib.sha256(content.encode()).hexdigest()
    (tmp_path / 'assets').mkdir(parents=True, exist_ok=True)
    (tmp_path / 'assets' / 'manifest.json').write_text(json.dumps(manifest), encoding='utf-8')
    return tmp_path


def test_verify_assets_accepts_the_real_tree():
    manifest = rt.verify_assets()
    assert len(manifest['sha256']) > 50
    assert set(manifest['not_carried_over']) == {'config/scene.json', 'config/task.json'}


def test_verify_assets_reports_what_it_did_not_record():
    """The real tree is expected to hold 010-authored integration files."""
    manifest = rt.verify_assets()
    assert isinstance(manifest['unrecorded'], list)


def test_verify_assets_fails_when_a_listed_file_is_missing(tmp_path):
    tree = build_tree(tmp_path, {'assets/x.txt': 'hello'})
    (tree / 'assets' / 'x.txt').unlink()
    with pytest.raises(ValueError, match='missing'):
        rt.verify_assets(tree)


def test_verify_assets_fails_when_a_hash_moved(tmp_path):
    tree = build_tree(tmp_path, {'assets/x.txt': 'hello'})
    (tree / 'assets' / 'x.txt').write_text('tampered', encoding='utf-8')
    with pytest.raises(ValueError, match='hash differs'):
        rt.verify_assets(tree)


def test_verify_assets_fails_when_a_not_carried_over_file_reappears(tmp_path):
    """The absent 007 configs are checked to stay absent, not merely ignored."""
    tree = build_tree(tmp_path, {'assets/x.txt': 'hello',
                                 'config/scene.json': '{}'})
    with pytest.raises(ValueError, match='not carried over'):
        rt.verify_assets(tree)


def test_verify_assets_lists_unrecorded_files_instead_of_failing(tmp_path):
    tree = build_tree(tmp_path, {'assets/x.txt': 'hello', 'assets/added.bin': 'zz'},
                      listed=['assets/x.txt'])
    manifest = rt.verify_assets(tree)
    assert manifest['unrecorded'] == ['assets/added.bin']


def test_runtime_loads_a_free_base_without_equality_constraints():
    import mujoco
    r = rt.Runtime()
    assert int(r.m.jnt_type[0]) == int(mujoco.mjtJoint.mjJNT_FREE)
    assert int(r.m.neq) == 0
    assert r.m.nq > 0 and r.m.nu > 0


def test_policy_dimensions_match_the_shipped_network():
    r = rt.Runtime()
    assert tuple(r.policy.input.getShape()) == (1, 1083)
    assert tuple(r.policy.output.getShape()) == (1, 22)
    assert len(r.policy.joints) == 25
    assert len(r.policy.active) == 22


def test_policy_rejects_a_nonfinite_observation():
    """A guard that cannot fire is not a guard; feed it the case it exists for."""
    import numpy as np
    r = rt.Runtime()
    with pytest.raises(FloatingPointError):
        r.policy.observation(np.full(len(r.policy.joints), np.nan),
                             np.zeros(len(r.policy.joints)),
                             np.array([1.0, 0.0, 0.0, 0.0]),
                             np.zeros(3), np.zeros(3))


def test_arm_ik_converges_and_stays_inside_the_joint_limits():
    import numpy as np
    r = rt.Runtime()
    target = {side: np.array([.23, sign * .30, .85]) for side, sign in
              [('left', 1), ('right', -1)]}
    joints = r.arm_ik(dict(target))
    assert r.arm_error(target, joints) < 0.01
    limits = r.m.jnt_range[r.m.actuator_trnid[r.body_act[r.arm_ids], 0]]
    assert np.all(joints >= limits[:, 0] - 1e-9)
    assert np.all(joints <= limits[:, 1] + 1e-9)


def test_arm_ik_on_an_unreachable_target_returns_without_raising():
    """Two metres out of reach: the solver must give up, not explode or loop."""
    import numpy as np
    r = rt.Runtime()
    far = {side: np.array([4.0, sign * .30, 0.85]) for side, sign in
           [('left', 1), ('right', -1)]}
    joints = r.arm_ik(dict(far))
    assert np.all(np.isfinite(joints))
    limits = r.m.jnt_range[r.m.actuator_trnid[r.body_act[r.arm_ids], 0]]
    assert np.all(joints >= limits[:, 0] - 1e-9)
    assert np.all(joints <= limits[:, 1] + 1e-9)


def test_the_superseded_exit_helper_is_deleted_rather_than_parked():
    """Measured four ways: p1-h-seq-04/05/06 and p1-h-exitpath-*-04.

    Straight-line joint return moved the tray 12.0 mm. The old waypoint helper as found
    moved it 48.7 mm, and 197.0 mm once its first waypoint was made purely vertical -- the
    tray left the table. All three swept the *descent* to the default posture, so no
    reordering of a retreat could fix them. Parking that helper "in case it is useful"
    invites exactly the silent re-use that produced the 197.0 mm run, so it is deleted; the
    measurements live in reports/ and docs/DECISIONS.md (D020, D022), and the replacement is
    `ExitPath`, whose default was chosen from those same measurements.
    """
    assert not hasattr(rt, 'withdrawal_waypoints'), (
        'the superseded exit helper is back in runtime.py; it was never used and measured '
        'worse than every alternative')
    probe = (ROOT / 'experiments/probe_h.py').read_text(encoding='utf-8')
    assert 'ExitPath(' in probe, 'the probe must drive the measured exit path'
    runtime_imports = [line for line in probe.splitlines()
                       if 'from humanoid007.runtime import' in line]
    assert runtime_imports, 'the probe should still import from humanoid007.runtime'
    assert 'ExitPath' in runtime_imports[0]
