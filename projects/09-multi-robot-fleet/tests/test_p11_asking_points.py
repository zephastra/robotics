"""A robot must be able to stand at — and arrive at — the place it is told to ask from.

The deadlock this pins
=====================

`config/resources.yaml` places `wait_*` and `align_*` as the only two points from which
`acquire` may be called. Measured on 2026-09-17, `align_west` was at |x| = 2.40 while the
gate's refusal boundary reached 2.523 at the fastest measured speed — so a robot
*standing* there was allowed and a robot *arriving* there was refused. The handshake
could never be opened, the robot could not move, its stopping envelope never shrank, and
the refusal was permanent. The driver reported `nav2 status=6 Failed to make progress`,
which is indistinguishable from a navigation fault and is not one. Rounds 6-9 chased it
as one.

What is asserted here, and why in this shape
============================================

Everything is asked of `CrossingManager.check_movement` — the same call the safety gate
makes — rather than recomputed. A test that re-derives the boundary it is checking can
only ever agree with itself, and would pass while the gate refused.

Three properties, for every direction:

  * the wait node is legal at `max_measured_speed_mps`;
  * the align node is legal at `max_measured_speed_mps`, **and so is every point within
    `node_reach_tolerance_m` of it toward the corridor** — because a robot counted as
    being "at" the node may be that far away, and the whole asking region has to be
    somewhere it may legally be;
  * the exit node is *refused* while the robot holds nothing — that is the reservation
    working, and it is asserted so the first two cannot be "fixed" by widening the
    boundary.

The speed used is the fastest the stop model was measured at, not the configured limit:
the boundary is monotone in speed, so clearing at that speed clears at every speed the
gate may authorise.
"""

from __future__ import annotations

import pathlib
import sys

import pytest

ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src" / "fleet_core"))

from fleet_core import CrossingManager, load_traffic_config  # noqa: E402

CONFIG = ROOT / "config" / "resources.yaml"

#: The robot every scenario in the corpus uses (config/fleet.yaml).
LENGTH_M = 0.60
WIDTH_M = 0.45


def _manager() -> CrossingManager:
    return CrossingManager(load_traffic_config(CONFIG))


def _session() -> dict:
    """A robot that holds nothing: the state that matters for asking."""
    return dict(task_id="", boot_id="", revision=0, generation=0, epoch=0)


def _check(manager, pose, *, speed):
    check = manager.check_movement(
        **_session(),
        wall_now=0.0,
        pose=pose,
        twist=(speed, 0.0, 0.0),
        length_m=LENGTH_M,
        width_m=WIDTH_M,
        localization_valid=True,
    )
    return check.allowed, check.reason.value


def _directions(manager):
    return sorted(manager.cfg.directions)


def test_the_wait_node_is_legal_to_arrive_at():
    """`wait` is documented as "a robot with no permit may drive here and no further"."""
    manager = _manager()
    vmax = manager.cfg.stop.max_measured_speed_mps
    bad = []
    for direction in _directions(manager):
        spec = manager.direction_spec(direction)
        node = manager.node(spec.wait_node)
        allowed, reason = _check(manager, (node.x, node.y, node.yaw), speed=vmax)
        if not allowed:
            bad.append(f"{direction}.{spec.wait_node} refused at {vmax} m/s: {reason}")
    assert not bad, (
        "a robot must be able to reach the waiting point with no permit: "
        + "; ".join(bad))


def test_the_align_node_and_its_whole_asking_region_are_legal_to_arrive_at():
    """The align node, and every point within the reach tolerance toward the corridor.

    That second part is the one that was false. A robot may be up to
    `node_reach_tolerance_m` from the node and still be counted as at it, so the node
    has to be far enough out that the *nearest* point of that region is legal too.
    """
    manager = _manager()
    vmax = manager.cfg.stop.max_measured_speed_mps
    tol = manager.cfg.node_reach_tolerance_m
    bad = []
    for direction in _directions(manager):
        spec = manager.direction_spec(direction)
        node = manager.node(spec.align_node)
        sign = float(spec.approach_sign)
        for label, x in (("the node itself", node.x),
                         (f"the nearest legal asking point (x={node.x + tol * sign:.3f}, "
                          f"+{tol} m toward the corridor)", node.x + tol * sign)):
            allowed, reason = _check(manager, (x, node.y, node.yaw), speed=vmax)
            if not allowed:
                bad.append(f"{direction}.{spec.align_node}: {label} refused at {vmax} "
                           f"m/s: {reason}")
    assert not bad, (
        "the asking point is inside the refusal boundary, so the handshake can never be "
        "opened and the deadlock is permanent (a stopped robot's envelope never shrinks). "
        "Move the node out; do not widen the boundary -- the boundary is what stops a "
        "robot whose stopping distance reaches the corridor. " + "; ".join(bad))


def test_the_exit_node_still_needs_the_grant():
    """The reservation must still bite: `exit` is inside the bundle.

    Without this, the two tests above could be satisfied by weakening the boundary.
    """
    manager = _manager()
    vmax = manager.cfg.stop.max_measured_speed_mps
    for direction in _directions(manager):
        spec = manager.direction_spec(direction)
        node = manager.node(spec.exit_node)
        allowed, reason = _check(manager, (node.x, node.y, node.yaw), speed=vmax)
        assert not allowed, (
            f"{direction}.{spec.exit_node} is inside the bundle and must not be reachable "
            f"without a permit")
        assert reason in ("RESOURCE_UNKNOWN", "ROUTE_REQUIRES_PERMIT", "RESOURCE_BUSY"), (
            f"{direction}.{spec.exit_node}: unexpected refusal reason {reason!r}")


def test_the_refusal_is_a_returned_reason_not_an_exception():
    """`check_movement` runs on a wall timer in the gate.

    An exception in that callback is a silently open door, so every refusal has to be a
    value. This asserts the shape, because the two tests above depend on it.
    """
    manager = _manager()
    vmax = manager.cfg.stop.max_measured_speed_mps
    spec = manager.direction_spec(_directions(manager)[0])
    node = manager.node(spec.exit_node)
    allowed, reason = _check(manager, (node.x, node.y, node.yaw), speed=vmax)
    assert allowed is False
    assert isinstance(reason, str) and reason


def test_the_align_node_clears_the_boundary_by_a_measured_amount():
    """Report the clearance, so a change that erodes it is visible rather than fatal.

    The node's |x| was derived as boundary + tolerance; this records how much room is
    left over, which is what a reader needs to judge a later edit. Deliberately not a
    threshold -- the threshold is the test above.
    """
    manager = _manager()
    vmax = manager.cfg.stop.max_measured_speed_mps
    tol = manager.cfg.node_reach_tolerance_m
    for direction in _directions(manager):
        spec = manager.direction_spec(direction)
        node = manager.node(spec.align_node)
        sign = float(spec.approach_sign)
        # Bisect for the boundary along the approach axis, using the same call as the gate.
        lo, hi = node.x, node.x + 3.0 * sign          # lo legal, hi refused
        assert _check(manager, (lo, node.y, node.yaw), speed=vmax)[0], (
            f"{direction}.{spec.align_node} itself is refused")
        assert not _check(manager, (hi, node.y, node.yaw), speed=vmax)[0], (
            "no refusal within 3 m of the align node toward the corridor: the bisection "
            "has nothing to bracket, so this test cannot measure anything")
        for _ in range(60):
            mid = 0.5 * (lo + hi)
            if _check(manager, (mid, node.y, node.yaw), speed=vmax)[0]:
                lo = mid
            else:
                hi = mid
        worst_asking_point = node.x + tol * sign
        clearance = abs(worst_asking_point - lo)
        assert clearance > 0, (
            f"{direction}: the worst legal asking point {worst_asking_point:.4f} is at or "
            f"inside the refusal boundary {lo:.4f}")
        print(f"{direction}.{spec.align_node} |x|={abs(node.x):.2f}: boundary {lo:.4f}, "
              f"worst asking point {worst_asking_point:.4f}, clearance {clearance:.4f} m")
