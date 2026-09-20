"""P4.5 regression: the odom frame is not the map frame.

gz's DiffDrive publishes odometry whose origin is the robot's spawn pose. The
coordinator and the gate compare footprints against map-frame rectangles, so
without an explicit composition every robot reads as sitting at (0, 0) -- which is
inside the corridor -- and the gate refuses all motion.

That is exactly what the first P4.5 acceptance run produced:

    robots.r01.pose = [0.0, 0.0, 0.0]     while spawning at (-6.0, -2.0)
    sweep_done = false                     (a second, unrelated bug; see below)

These tests pin the composition itself and the consequence that mattered.
"""

from __future__ import annotations

import math
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
SRC = ROOT / "src"
for _pkg in ("fleet_core", "fleet_adapter"):
    _p = SRC / _pkg
    if str(_p) not in sys.path:
        sys.path.insert(0, str(_p))

from fleet_core import CrossingManager, load_traffic_config  # noqa: E402
from fleet_core.geometry import (  # noqa: E402
    Region,
    compose_2d,
    footprint_corners,
    wrap_angle,
)

CONFIG = ROOT / "config" / "resources.yaml"
L, W = 0.60, 0.45


@pytest.fixture(scope="module")
def tcfg():
    return load_traffic_config(CONFIG)


# --------------------------------------------------------------------------- #
# compose_2d
# --------------------------------------------------------------------------- #


def test_a_robot_at_its_spawn_odom_origin_lands_on_its_spawn():
    """The regression. r01 spawns at (-6, -2) and its odom reads (0, 0, 0)."""
    assert compose_2d((-6.0, -2.0, 0.0), (0.0, 0.0, 0.0)) == pytest.approx((-6.0, -2.0, 0.0))
    assert compose_2d((6.0, -2.0, math.pi), (0.0, 0.0, 0.0)) == pytest.approx((6.0, -2.0, math.pi))


def test_an_identity_origin_changes_nothing():
    local = (1.5, -0.25, 0.3)
    assert compose_2d((0.0, 0.0, 0.0), local) == pytest.approx(local)


def test_pure_translation_adds():
    assert compose_2d((2.0, 3.0, 0.0), (1.0, 1.0, 0.0)) == pytest.approx((3.0, 4.0, 0.0))


def test_a_yawed_origin_rotates_the_local_offset():
    """Forward 1 m in a frame rotated 90 deg must come out as +y in the parent."""
    x, y, yaw = compose_2d((0.0, 0.0, math.pi / 2), (1.0, 0.0, 0.0))
    assert (x, y) == pytest.approx((0.0, 1.0), abs=1e-12)
    assert yaw == pytest.approx(math.pi / 2)

    # 180 deg: forward becomes backward.
    x, y, _ = compose_2d((0.0, 0.0, math.pi), (1.0, 0.0, 0.0))
    assert (x, y) == pytest.approx((-1.0, 0.0), abs=1e-12)


def test_yaws_add_and_wrap():
    _, _, yaw = compose_2d((0.0, 0.0, 3.0), (0.0, 0.0, 3.0))
    assert yaw == pytest.approx(wrap_angle(6.0))
    assert -math.pi < yaw <= math.pi


def test_wrap_angle_folds_into_the_expected_range():
    assert wrap_angle(0.0) == pytest.approx(0.0)
    assert wrap_angle(2.0 * math.pi) == pytest.approx(0.0, abs=1e-12)
    assert wrap_angle(-2.0 * math.pi) == pytest.approx(0.0, abs=1e-12)
    assert wrap_angle(3.0 * math.pi) == pytest.approx(math.pi, abs=1e-12)


# --------------------------------------------------------------------------- #
# the consequence: the frame error turned into a total standstill
# --------------------------------------------------------------------------- #


def test_a_robot_at_spawn_odom_reads_as_inside_the_corridor(tcfg):
    """Documents why the bug stopped the fleet.

    Odometry alone puts r01 at (0,0), which is the middle of the gap. With the
    guard on, the gate then refuses every command -- safely, and uselessly.
    """
    raw = (0.0, 0.0, 0.0)
    region = Region("all", tuple(r for s in tcfg.resources.values() for r in s.rects),
                    margin_m=tcfg.footprint_margin_m)
    assert region.box_overlap(footprint_corners(*raw, L, W)), (
        "the raw odom pose should look like it is inside the crossing"
    )


def test_composing_with_the_spawn_moves_it_out(tcfg):
    """And the fix: the same odometry, placed in the map frame, is clear."""
    spawn = (-6.0, -2.0, 0.0)
    raw = (0.0, 0.0, 0.0)
    posed = compose_2d(spawn, raw)
    region = Region("all", tuple(r for s in tcfg.resources.values() for r in s.rects),
                    margin_m=tcfg.footprint_margin_m)
    assert not region.box_overlap(footprint_corners(*posed, L, W))


def test_the_crossing_is_still_found_from_a_correctly_placed_pose(tcfg):
    """The fix must not make the corridor unreachable, only correctly located."""
    mgr = CrossingManager(tcfg)
    # 6 m east of r01's spawn is exactly on the gap centre.
    posed = compose_2d((-6.0, -2.0, 0.0), (6.0, 2.0, 0.0))
    assert posed == pytest.approx((0.0, 0.0, 0.0))
    assert mgr.full_region().box_overlap(footprint_corners(*posed, L, W))


def test_spawn_poses_in_the_shipped_config_are_outside_the_region(tcfg):
    """Cross-checks config/spawns.yaml against the traffic geometry.

    If a spawn pose ever moved into the protected area, every robot would start
    inside a corridor it holds no permit for -- and the gate would hold it there.
    """
    import yaml

    table = yaml.safe_load((ROOT / "config" / "spawns.yaml").read_text(encoding="utf-8"))
    mgr = CrossingManager(tcfg)
    region = mgr.full_region()
    for robot, entry in (table.get("spawns") or {}).items():
        pose = (float(entry.get("x", 0.0)), float(entry.get("y", 0.0)),
                float(entry.get("yaw", 0.0)))
        assert region.fully_outside(footprint_corners(*pose, L, W)), (
            f"{robot} spawns at {pose} which is inside the protected crossing"
        )
