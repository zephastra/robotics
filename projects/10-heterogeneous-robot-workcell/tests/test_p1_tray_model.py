"""The V1 tray model, the fragment extraction, and the generated world.

Structural and provenance checks only. Nothing here asserts anything about physics; the
six-stage criteria are in test_p1_tray_stages.py.

Regenerating the world is a side effect on the tree, so these tests use the generator's
`--check` mode: it rebuilds in memory and compares, writing nothing. That way the test
verifies the committed world matches its sources without editing it.
"""
import hashlib
from pathlib import Path
import subprocess
import sys

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src'))

import mujoco  # noqa: E402

from humanoid007 import runtime as rt  # noqa: E402

COMBINED = ROOT / 'assets/combined.xml'
TRAY_WORLD = ROOT / 'assets/world_tray_v1.xml'
TRAY_FRAGMENT = ROOT / 'assets/objects/tray_v1.xml'
PAYLOAD_FRAGMENT = ROOT / 'assets/objects/payload_007.xml'
MAKE_WORLD = ROOT / 'experiments/make_world.py'
HANDLE_GEOMS = ('handle_-1', 'handle_stem_-1', 'handle_1', 'handle_stem_1')


def make_world(*args):
    return subprocess.run([sys.executable, str(MAKE_WORLD), *args],
                          capture_output=True, text=True, cwd=str(ROOT))


def body_block(text, name='payload'):
    start = text.find(f'<body name="{name}"')
    end = text.find('</body>', start) + len('</body>')
    return text[start:end]


def load(path):
    return mujoco.MjModel.from_xml_path(str(path))


def geom_signature(model, name):
    gid = model.geom(name).id
    return (int(model.geom_type[gid]), tuple(model.geom_size[gid]),
            tuple(model.geom_pos[gid]))


def test_the_extracted_fragment_is_verbatim():
    """The extracted 007 object must be the original bytes, not a retyping."""
    from_combined = body_block(COMBINED.read_text(encoding='utf-8'))
    from_fragment = body_block(PAYLOAD_FRAGMENT.read_text(encoding='utf-8'))
    assert from_fragment == from_combined


def test_the_extractor_is_reproducible():
    result = make_world('--extract', '--check')
    assert result.returncode == 0, result.stdout + result.stderr


def test_the_generated_world_matches_its_sources():
    result = make_world('--object', 'tray_v1', '--check')
    assert result.returncode == 0, result.stdout + result.stderr


def test_the_generator_refuses_an_object_that_does_not_exist():
    result = make_world('--object', 'no_such_tray')
    assert result.returncode == 2
    assert 'ABORT' in result.stdout


def test_the_generator_does_not_touch_the_snapshot():
    """The plan requires the original model snapshot to survive this work."""
    before = hashlib.sha256(COMBINED.read_bytes()).hexdigest()
    make_world('--object', 'tray_v1', '--check')
    make_world('--object', 'no_such_tray')
    assert hashlib.sha256(COMBINED.read_bytes()).hexdigest() == before


def test_both_worlds_use_the_same_role_name():
    """The judges look the object up as `payload`; the role name must not change."""
    for path in (COMBINED, TRAY_WORLD):
        model = load(path)
        assert model.body('payload').id > 0


def test_v1_tray_is_lighter_than_the_007_tray():
    def mass(path):
        model = load(path)
        return float(model.body_mass[model.body('payload').id])
    assert mass(COMBINED) == pytest.approx(0.210, abs=1e-6)
    assert mass(TRAY_WORLD) == pytest.approx(0.098, abs=1e-6)
    assert mass(TRAY_WORLD) < mass(COMBINED)


def test_v1_tray_is_a_free_body_with_no_welds():
    model = load(TRAY_WORLD)
    payload = model.body('payload').id
    assert model.jnt_type[model.body_jntadr[payload]] == mujoco.mjtJoint.mjJNT_FREE
    assert int(model.neq) == 0


def test_v1_tray_has_three_pockets_and_two_handles():
    model = load(TRAY_WORLD)
    payload = model.body('payload').id
    names = [model.geom(model.body_geomadr[payload] + k).name
             for k in range(model.body_geomnum[payload])]
    assert len([n for n in names if n.startswith('tray_divider_')]) == 2, names
    for name in HANDLE_GEOMS:
        assert name in names, names
    assert 'tray_floor' in names
    assert len([n for n in names if n.startswith('tray_wall_')]) == 4, names


def test_the_grasp_interface_is_unchanged():
    """The claim in the tray's header comment, made checkable.

    The reach anchors and the hand-close preset were validated against the 007 handle
    geometry. If a future edit moves it, the reach plan silently stops being valid.
    """
    old, new = load(COMBINED), load(TRAY_WORLD)
    for name in HANDLE_GEOMS:
        assert geom_signature(old, name) == geom_signature(new, name), name


def test_runtime_loads_the_world_it_is_given():
    default = rt.Runtime()
    assert float(default.m.body_mass[default.m.body('payload').id]) == pytest.approx(
        0.210, abs=1e-6)

    tuned = rt.Runtime(TRAY_WORLD)
    assert float(tuned.m.body_mass[tuned.m.body('payload').id]) == pytest.approx(
        0.098, abs=1e-6)


def test_the_probe_exposes_the_object_switch():
    result = subprocess.run([sys.executable, str(ROOT / 'experiments/probe_h.py'), '--help'],
                            capture_output=True, text=True, cwd=str(ROOT))
    assert result.returncode == 0
    assert '--object' in result.stdout
