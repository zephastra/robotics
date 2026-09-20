"""P4 geometry: the containment predicate the gate depends on.

The interesting test here is ``test_sat_matches_reference_on_a_grid``. The
separating-axis routine is the one piece of arithmetic in P4 whose failure mode
is "the gate quietly says yes", so it is not tested against three hand-picked
points -- it is tested against an independent reference implementation over a
grid of poses and headings. A hand-picked test only proves the three cases I
happened to think of.
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

from fleet_core.geometry import (  # noqa: E402
    Rect,
    Region,
    StopModel,
    aabb_of,
    dominant_speed,
    footprint_corners,
    footprint_radius,
)


# --------------------------------------------------------------------------- #
# reference implementation, written independently of the module under test
# --------------------------------------------------------------------------- #


def _cross(o, a, b) -> float:
    return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])


def _on_segment(p, q, r) -> bool:
    return (
        min(p[0], r[0]) <= q[0] <= max(p[0], r[0])
        and min(p[1], r[1]) <= q[1] <= max(p[1], r[1])
    )


def _segments_cross(p1, p2, p3, p4) -> bool:
    d1, d2 = _cross(p3, p4, p1), _cross(p3, p4, p2)
    d3, d4 = _cross(p1, p2, p3), _cross(p1, p2, p4)
    if ((d1 > 0) != (d2 > 0)) and ((d3 > 0) != (d4 > 0)):
        return True
    if d1 == 0 and _on_segment(p3, p1, p4):
        return True
    if d2 == 0 and _on_segment(p3, p2, p4):
        return True
    if d3 == 0 and _on_segment(p1, p3, p2):
        return True
    if d4 == 0 and _on_segment(p1, p4, p2):
        return True
    return False


def _point_in_convex(p, poly) -> bool:
    signs = []
    for i in range(len(poly)):
        signs.append(_cross(poly[i], poly[(i + 1) % len(poly)], p))
    return all(s >= -1e-12 for s in signs) or all(s <= 1e-12 for s in signs)


def _reference_overlap(corners, rect: Rect) -> bool:
    """Exact convex-convex overlap: vertex inside, or any edge pair crossing."""
    rect_corners = (
        (rect.x_min, rect.y_min),
        (rect.x_max, rect.y_min),
        (rect.x_max, rect.y_max),
        (rect.x_min, rect.y_max),
    )
    for p in corners:
        if rect.contains_point(p):
            return True
    for p in rect_corners:
        if _point_in_convex(p, corners):
            return True
    for i in range(4):
        for j in range(4):
            if _segments_cross(
                corners[i], corners[(i + 1) % 4], rect_corners[j], rect_corners[(j + 1) % 4]
            ):
                return True
    return False


# --------------------------------------------------------------------------- #
# rectangles
# --------------------------------------------------------------------------- #


def test_rect_rejects_inverted_bounds():
    with pytest.raises(ValueError):
        Rect("bad", x_min=1.0, y_min=0.0, x_max=0.0, y_max=1.0)


def test_rect_expansion_is_symmetric():
    r = Rect("r", 0.0, 0.0, 1.0, 2.0).expanded(0.5)
    assert (r.x_min, r.y_min, r.x_max, r.y_max) == (-0.5, -0.5, 1.5, 2.5)
    assert r.width == pytest.approx(2.0)


def test_zero_expansion_is_a_no_op():
    r = Rect("r", 0.0, 0.0, 1.0, 1.0)
    assert r.expanded(0.0) == r


def test_negative_expansion_is_refused():
    with pytest.raises(ValueError):
        Rect("r", 0.0, 0.0, 1.0, 1.0).expanded(-0.1)


def test_region_needs_a_rectangle():
    with pytest.raises(ValueError):
        Region(name="empty", rects=())


# --------------------------------------------------------------------------- #
# footprint
# --------------------------------------------------------------------------- #


def test_footprint_at_zero_yaw_is_axis_aligned():
    corners = footprint_corners(0.0, 0.0, 0.0, 0.60, 0.45)
    lo, hi = aabb_of(corners)
    assert lo == pytest.approx((-0.30, -0.225))
    assert hi == pytest.approx((0.30, 0.225))


def test_footprint_quarter_turn_swaps_extents():
    corners = footprint_corners(0.0, 0.0, math.pi / 2, 0.60, 0.45)
    lo, hi = aabb_of(corners)
    assert hi[0] - lo[0] == pytest.approx(0.45)
    assert hi[1] - lo[1] == pytest.approx(0.60)


def test_footprint_rejects_zero_size():
    with pytest.raises(ValueError):
        footprint_corners(0.0, 0.0, 0.0, 0.0, 0.45)


def test_footprint_radius_is_the_circumscribed_circle():
    assert footprint_radius(0.60, 0.45) == pytest.approx(0.375, abs=1e-9)


# --------------------------------------------------------------------------- #
# the containment predicate
# --------------------------------------------------------------------------- #


def test_sat_matches_reference_on_a_grid():
    """Falsify the SAT routine against an independent exact test.

    Sweeps a small rectangle and a robot across positions and headings, and
    requires the two implementations to agree everywhere. A disagreement at any
    point means the gate's arithmetic is wrong, which is the one failure mode
    that does not announce itself.
    """
    rect = Rect("probe", 1.0, 1.0, 1.6, 1.6)
    mismatches = []
    for xi in range(-20, 21):
        for yi in range(-20, 21):
            x = xi * 0.05
            y = yi * 0.05
            for k in range(8):
                yaw = k * math.pi / 8.0
                corners = footprint_corners(x, y, yaw, 0.60, 0.45)
                got = _convex(corners, rect)
                want = _reference_overlap(corners, rect)
                if got != want:
                    mismatches.append((round(x, 3), round(y, 3), round(yaw, 4), got, want))
    assert not mismatches, f"{len(mismatches)} disagreements, first few: {mismatches[:5]}"


def _convex(corners, rect: Rect) -> bool:
    from fleet_core.geometry import _convex_overlap

    return _convex_overlap(corners, rect)


def test_aabb_alone_would_over_refuse():
    """The reason the SAT test exists: the AABB is strictly larger.

    A robot at 45 degrees near a rectangle corner has an AABB that touches it
    while the body does not. If the gate used the AABB, this legal pose would be
    refused; the test pins that the two answers differ *somewhere*, so removing
    the SAT re-check cannot pass unnoticed.
    """
    rect = Rect("probe", 1.0, 1.0, 1.6, 1.6)
    differs = 0
    for xi in range(-20, 21):
        for yi in range(-20, 21):
            x, y = xi * 0.05, yi * 0.05
            corners = footprint_corners(x, y, math.pi / 4, 0.60, 0.45)
            lo, hi = aabb_of(corners)
            if rect.overlaps_aabb(lo, hi) and not _convex(corners, rect):
                differs += 1
    assert differs > 0, "AABB and SAT never disagree here; the SAT re-check is dead code"


# --------------------------------------------------------------------------- #
# regions
# --------------------------------------------------------------------------- #


def test_region_margin_is_applied_per_rectangle():
    region = Region("r", (Rect("a", 0.0, 0.0, 1.0, 1.0),), margin_m=0.2)
    # A footprint centred at 1.30 with half-length 0.30 reaches x = 1.00 exactly.
    at_edge = footprint_corners(1.30, 0.5, 0.0, 0.60, 0.45)
    assert region.box_overlap(at_edge), "touching the un-margined edge must count"
    clear = footprint_corners(1.60, 0.5, 0.0, 0.60, 0.45)
    assert not region.box_overlap(clear), "0.10 m past the margin must be clear"


def test_region_fully_outside_is_the_inverse_of_box_overlap():
    region = Region("r", (Rect("a", 0.0, 0.0, 1.0, 1.0),), margin_m=0.15)
    inside = footprint_corners(0.5, 0.5, 0.0, 0.60, 0.45)
    assert region.box_overlap(inside)
    assert not region.fully_outside(inside)
    outside = footprint_corners(5.0, 5.0, 0.0, 0.60, 0.45)
    assert region.fully_outside(outside)


# --------------------------------------------------------------------------- #
# stopping maths
# --------------------------------------------------------------------------- #


def test_stop_envelope_is_zero_at_rest():
    m = StopModel(latency_s=0.2, a_stop_mps2=0.4, localization_margin_m=0.1)
    assert m.envelope_m(0.0) == 0.0


def test_stop_envelope_matches_closed_form():
    m = StopModel(latency_s=0.2, a_stop_mps2=0.4, localization_margin_m=0.1)
    # 0.35 * 0.2 + 0.35^2 / (2 * 0.4) = 0.07 + 0.153125
    assert m.envelope_m(0.35) == pytest.approx(0.223125)


def test_stop_envelope_covers_the_measured_stop_distance():
    """The model must be pessimistic about the runs P2 actually measured.

    Measured worst-case stop distance through the gate, ground truth:
    0.35 m/s -> 0.1381 m. The envelope at that speed must exceed it, or the gate
    would authorise motion it cannot stop.
    """
    m = StopModel(latency_s=0.2, a_stop_mps2=0.40, localization_margin_m=0.10)
    for speed, measured in ((0.15, 0.0292), (0.25, 0.0709), (0.35, 0.1381)):
        assert m.envelope_m(speed) > measured, f"{speed} m/s: envelope under the measured stop distance"


def test_negative_speed_is_treated_as_rest():
    m = StopModel(latency_s=0.2, a_stop_mps2=0.4, localization_margin_m=0.1)
    assert m.envelope_m(-1.0) == 0.0


def test_turning_in_place_is_not_treated_as_standing_still():
    """A spinning robot sweeps space, so yaw rate must appear as linear speed."""
    radius = footprint_radius(0.60, 0.45)
    assert dominant_speed((0.0, 0.0, 0.0), radius) == 0.0
    assert dominant_speed((0.0, 0.0, 1.0), radius) == pytest.approx(radius)
    assert dominant_speed((0.1, 0.0, 0.0), radius) == pytest.approx(0.1)
