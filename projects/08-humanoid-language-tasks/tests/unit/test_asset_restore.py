"""Publication asset bootstrap: no network, physics or other project imports."""
import hashlib
from pathlib import Path
import runpy

import pytest

api = runpy.run_path(str(Path(__file__).resolve().parents[2] / "scripts/prepare_assets.py"))


@pytest.mark.parametrize("name", ["../escape", "/tmp/escape", "assets/../../escape", "assets\\escape"])
def test_rejects_unsafe_paths(name):
    with pytest.raises(ValueError):
        api["upstream_path"](name)


def test_pinned_upstream_mapping_and_project_files():
    assert api["upstream_path"]("assets/allegro/assets/base_link.stl") == (
        "menagerie", "wonik_allegro/assets/base_link.stl")
    assert api["upstream_path"]("assets/t800/robot/t800/xml/assets.xml") == (
        "engineai", "assets/resource/robot/t800/xml/assets.xml")
    assert api["upstream_path"]("assets/combined.xml") is None
    assert api["upstream_path"]("config/workcell.json") is None


def test_hash_mismatch_is_not_accepted():
    digest = hashlib.sha256(b"original").hexdigest()
    assert api["verified"](b"original", digest, "test") == b"original"
    with pytest.raises(ValueError):
        api["verified"](b"changed", digest, "test")
