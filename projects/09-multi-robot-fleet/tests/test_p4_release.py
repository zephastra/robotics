"""P4.8: the release node must clear the bundle by the arrival tolerance.

``ConfirmClear`` is the last step of a crossing. The robot offers its pose; the
coordinator tests the whole footprint plus margin against every rectangle in the bundle
and refuses unless nothing overlaps. The release node is the only place a robot is
allowed to make that offer, so it has to be far enough out that a *typical* arrival
still clears.

The first version placed it at 3.5 m, which clears the exit buffer by 0.20 m for a
perfect arrival. The measured arrival error is 0.5051 m (worst case, P2 ground truth --
the same measurement that set ``node_reach_tolerance_m = 0.80``). So both robots, in
both directions, were refused with::

    CLEARANCE_NOT_PROVEN: footprint plus 0.15 m margin still overlaps ['pad']

and a sequence that had otherwise succeeded end to end -- wait, align, acquire, cross,
release -- never completed. ``both_completed`` was false for a run that worked.

The minimum is derived here rather than restated, so that moving a tolerance or
tightening a rectangle fails in this file instead of in a simulation run.
"""

from __future__ import annotations

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
from fleet_core.geometry import footprint_corners  # noqa: E402

CONFIG = ROOT / "config" / "resources.yaml"

# r01 / r02 footprint, from config/fleet.yaml.
L, W = 0.60, 0.45

DIRECTIONS = ("west_to_east", "east_to_west")


@pytest.fixture(scope="module")
def tcfg():
    return load_traffic_config(CONFIG)


def _release(tcfg, direction):
    """The region ConfirmClear actually tests, taken from the manager itself.

    It used to be built here as `region_of(..., margin_m=footprint_margin_m)`, which was the
    clearance boundary -- and it is the reason this file passed while a live run refused: it
    re-derived the wrong expression, faithfully. Asking the manager for its own boundary means
    this file cannot pass unless the thing that runs is the thing that is checked.
    """
    mgr = CrossingManager(tcfg)
    spec = mgr.direction_spec(direction)
    return mgr, mgr.clearance_region(mgr.bundle(direction)), mgr.node(spec.release_node)


def _minimum_clearing_x(tcfg, direction) -> float:
    """Smallest |x| on the release node's row at which the footprint clears."""
    mgr, region, release = _release(tcfg, direction)
    sign = 1.0 if release.x >= 0 else -1.0
    limit = abs(release.x) + 2.0
    x = abs(release.x) - 2.0
    while x <= limit:
        if not region.box_overlap(
                footprint_corners(sign * x, release.y, release.yaw, L, W)):
            return abs(x)
        x += 0.005
    raise AssertionError(f"{direction}: no clearing x found between 0 and {limit}")


def test_the_release_pose_does_not_overlap_its_own_bundle(tcfg):
    """The literal condition ``ConfirmClear`` tests, checked without a simulator."""
    for direction in DIRECTIONS:
        _, region, release = _release(tcfg, direction)
        corners = footprint_corners(release.x, release.y, release.yaw, L, W)
        hits = region.box_overlap(corners)
        assert not hits, (
            f"{direction}: the release node puts the footprint inside "
            f"{[h.name for h in hits]}. ConfirmClear would refuse, the permit would "
            "lapse, and a crossing that otherwise succeeded would never complete")


def test_the_release_node_clears_by_at_least_the_arrival_tolerance(tcfg):
    """Clearing for a perfect arrival is not clearing.

    The robot may stop anywhere inside ``node_reach_tolerance_m`` of the goal, so the
    node must clear the bundle by at least that much -- otherwise completion depends on
    how well Nav2 happened to do on the day.
    """
    for direction in DIRECTIONS:
        _, _, release = _release(tcfg, direction)
        needed = _minimum_clearing_x(tcfg, direction)
        slack = abs(release.x) - needed
        assert slack >= tcfg.node_reach_tolerance_m, (
            f"{direction}: the release node at |x|={abs(release.x):.2f} first clears the "
            f"bundle at |x|={needed:.2f}, leaving only {slack:.2f} m of slack, while the "
            f"robot may stop up to {tcfg.node_reach_tolerance_m:.2f} m short of the goal. "
            "Move the node out.")


def test_the_config_header_derivation_still_comes_out_right(tcfg):
    """The header states the arithmetic; this checks it, so the comment cannot rot.

        release |x| >= buffer_outer + stopping_reserve + footprint_margin
                                + max(half_length, half_width) + node_reach_tolerance_m

    The `stopping_reserve` term was added on 2026-09-17. It is the term the gate's refusal has
    always had and the clearance proof did not (`D-P10-01`), and its absence is why the nodes
    at 4.2 satisfied a boundary that was not the one being enforced.
    """
    buffer_outer = max(max(abs(r.x_min), abs(r.x_max))
                       for spec in tcfg.resources.values() for r in spec.rects)
    reserve = tcfg.stop.worst_case_envelope_m() + tcfg.stop.localization_margin_m
    expected = (buffer_outer + reserve + tcfg.footprint_margin_m
                + max(L, W) / 2.0 + tcfg.node_reach_tolerance_m)

    assert expected == pytest.approx(4.42, abs=0.02), (
        f"the derivation in config/resources.yaml no longer comes out at 4.42 but "
        f"{expected:.2f}; the comment and the nodes disagree")

    for direction in DIRECTIONS:
        _, _, release = _release(tcfg, direction)
        assert abs(release.x) >= expected, (
            f"{direction}: |x|={abs(release.x)} is below the derived minimum "
            f"{expected:.2f}")
