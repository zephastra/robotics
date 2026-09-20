"""P4 traffic: permits, occupancy, and the two ways a fleet puts two robots in
one corridor.

These tests load the **real** ``config/resources.yaml`` rather than a fixture
built to match it. A test config that drifts from the shipped one passes while
the vehicle is wrong, and the geometry in that file is the thing most likely to
be edited by hand.

Each test below is a specific naive implementation:

* free the resource when the TTL lapses               -> test_ttl_lapse_*
* treat "somebody holds it" as "somebody is inside"    -> test_enter_requires_*
* let a robot release while parked on the exit buffer  -> test_clearance_*
* check permissions per bundle instead of per resource -> test_other_exit_*
* accept a clearance flag instead of measuring         -> test_clearance_*_verified
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

from fleet_core import (  # noqa: E402
    ConfigError,
    CrossingManager,
    ReasonCode,
    ResourceState,
    StaleCommandError,
    load_traffic_config,
    validate_traffic_config,
)
from fleet_core.geometry import footprint_corners  # noqa: E402

CONFIG = ROOT / "config" / "resources.yaml"

# r01 / r02 footprint, from config/fleet.yaml.
L, W = 0.60, 0.45

WEST_TO_EAST = "west_to_east"
EAST_TO_WEST = "east_to_west"


# --------------------------------------------------------------------------- #
# helpers
# --------------------------------------------------------------------------- #


@pytest.fixture(scope="module")
def tcfg():
    return load_traffic_config(CONFIG)


@pytest.fixture
def mgr(tcfg):
    """A crossing whose resources have all been verified clear.

    Most of these tests are about the gate's decision, not about start-up, so the
    default is a healthy crossing. The two tests that care about the start-of-run
    UNKNOWN state build their own manager -- otherwise "unknown" would be baked
    into every expectation and a resource could be authorised before anyone
    looked at it without any test noticing.
    """
    return all_free(CrossingManager(tcfg))


def all_free(mgr, source="test: pre-run geometry check"):
    """Bring the crossing out of UNKNOWN the way the node does -- with a reason.

    Tests go through the same door as production. Poking the private state dict
    would let a change to the FREE precondition pass the whole suite.
    """
    for name in mgr.cfg.resources:
        mgr.mark_verified_free(name, source=source)
    return mgr


def node_pose(cfg, name):
    n = cfg.nodes[name]
    return (n.x, n.y, n.yaw)


def ask(mgr, direction, *, task="task-A", robot="r01", gen=1, at=None, wall_now=0.0, loc=True):
    spec = mgr.direction_spec(direction)
    pose = node_pose(mgr.cfg, at if at is not None else spec.wait_node)
    return mgr.acquire(
        direction,
        task_id=task,
        robot_id=robot,
        boot_id="b1",
        revision=0,
        generation=gen,
        epoch=0,
        now=0.0,
        wall_now=wall_now,
        pose=pose,
        length_m=L,
        width_m=W,
        localization_valid=loc,
    )


def move(mgr, *, task="task-A", gen=1, pose=(0.0, 0.0, 0.0), twist=(0.0, 0.0, 0.0), loc=True, wall_now=0.0):
    return mgr.check_movement(
        task_id=task, boot_id="b1", revision=0, generation=gen, epoch=0,
        wall_now=wall_now, pose=pose, twist=twist,
        length_m=L, width_m=W, localization_valid=loc,
    )


# --------------------------------------------------------------------------- #
# configuration
# --------------------------------------------------------------------------- #


def test_shipped_config_loads(tcfg):
    assert set(tcfg.resources) == {"mid", "mid_left", "mid_right"}
    assert set(tcfg.bundles) == {WEST_TO_EAST, EAST_TO_WEST}
    assert WEST_TO_EAST in tcfg.directions and EAST_TO_WEST in tcfg.directions


def test_corridor_rectangle_covers_the_physics_gap(tcfg):
    """The forbidden rectangle must contain the real gap, not sit inside it.

    warehouse.sdf leaves a hole over |y| < 0.6 (barrier_north covers y >= 0.6,
    barrier_south covers y <= -0.6). If the rectangle were narrower, a robot
    could straddle the mouth of the passage while showing as "outside".
    """
    gap = tcfg.rect_named("gap")
    assert gap is not None
    assert gap.y_min <= -0.60 and gap.y_max >= 0.60
    assert gap.x_min <= -1.50 and gap.x_max >= 1.50


def test_bundle_without_an_exit_buffer_is_rejected():
    raw = {
        "schema_version": 1,
        "resources": {"mid": {"kind": "CORRIDOR", "rects": [{"name": "g", "x_min": 0, "y_min": 0, "x_max": 1, "y_max": 1}]}},
        "bundles": {"west_to_east": ["mid"]},
        "nodes": {"w": {"x": 0.0, "y": 0.0}},
        "directions": {"west_to_east": {"wait_node": "w", "align_node": "w", "exit_node": "w", "release_node": "w", "entry_rect": "g"}},
        "traffic": {
            "footprint_margin_m": 0.15, "node_reach_tolerance_m": 0.35,
            "permit_ttl_s": 12.0, "renew_before_s": 4.0,
            "stop_model": {"latency_s": 0.2, "a_stop_mps2": 0.4, "localization_margin_m": 0.1},
        },
    }
    with pytest.raises(ConfigError, match="exit buffer"):
        validate_traffic_config(raw)


def test_renew_before_must_be_smaller_than_ttl():
    raw = {
        "schema_version": 1,
        "resources": {"mid": {"kind": "CORRIDOR", "rects": [{"name": "g", "x_min": 0, "y_min": 0, "x_max": 1, "y_max": 1}]}},
        "bundles": {"west_to_east": ["mid"]},
        "nodes": {"w": {"x": 0.0, "y": 0.0}},
        "directions": {"west_to_east": {"wait_node": "w", "align_node": "w", "exit_node": "w", "release_node": "w", "entry_rect": "g"}},
        "traffic": {
            "footprint_margin_m": 0.15, "node_reach_tolerance_m": 0.35,
            "permit_ttl_s": 4.0, "renew_before_s": 4.0,
            "stop_model": {"latency_s": 0.2, "a_stop_mps2": 0.4, "localization_margin_m": 0.1},
        },
    }
    with pytest.raises(ConfigError):
        validate_traffic_config(raw)


# --------------------------------------------------------------------------- #
# initial state
# --------------------------------------------------------------------------- #


def test_initial_state_is_unknown_not_free(tcfg):
    """CONTRACTS section 7: nothing is FREE until occupancy has been checked.

    Built from its own manager: the ``mgr`` fixture is a healthy crossing.
    """
    fresh = CrossingManager(tcfg)
    for name in fresh.cfg.resources:
        assert fresh.state_of(name) is ResourceState.UNKNOWN
    assert fresh.state_of("mid") is not ResourceState.FREE


def test_nobody_may_acquire_while_the_state_is_unknown(tcfg):
    """Granting into an unchecked corridor is the deadlock-on-first-boot bug.

    At boot the lease ledger is empty, so a naive "is anything held?" check says
    the corridor is available. It is not: nobody has looked at it yet.
    """
    fresh = CrossingManager(tcfg)
    res = ask(fresh, WEST_TO_EAST)
    assert res.granted is False
    assert res.reason is ReasonCode.RESOURCE_UNKNOWN
    assert fresh.book.owner_of(fresh.rid("mid")) is None

    all_free(fresh)
    assert ask(fresh, WEST_TO_EAST).granted is True


def test_marking_free_demands_a_reason(mgr):
    """The only route out of UNKNOWN is an argument, never a bare setter."""
    with pytest.raises(ValueError):
        mgr.mark_verified_free("mid", source="")


def test_occupied_resource_cannot_be_declared_free(mgr):
    all_free(mgr)
    assert ask(mgr, WEST_TO_EAST).granted is True
    mgr.enter(WEST_TO_EAST, task_id="task-A", boot_id="b1", revision=0, generation=1,
              epoch=0, wall_now=0.0, pose=(0.0, 0.0, 0.0), length_m=L, width_m=W)
    assert mgr.state_of("mid") is ResourceState.OCCUPIED
    with pytest.raises(RuntimeError):
        mgr.mark_verified_free("mid", source="just trust me")


# --------------------------------------------------------------------------- #
# acquire preconditions
# --------------------------------------------------------------------------- #


def test_acquire_from_the_wait_node_is_granted(mgr):
    all_free(mgr)
    res = ask(mgr, WEST_TO_EAST)
    assert res.granted is True
    assert mgr.state_of("mid") is ResourceState.RESERVED
    assert mgr.state_of("mid_right") is ResourceState.RESERVED
    assert mgr.book.owner_of(mgr.rid("mid_left")) is None, "the other side must not be granted"
    assert mgr.state_of("mid_left") is ResourceState.FREE, "and must not be marked RESERVED"
    assert mgr.book.owner_of(mgr.rid("mid")) == "task-A"


def test_acquire_from_a_random_pose_is_refused_and_not_queued(mgr):
    all_free(mgr)
    spec = mgr.direction_spec(WEST_TO_EAST)
    res = mgr.acquire(
        WEST_TO_EAST, task_id="task-A", robot_id="r01", boot_id="b1", revision=0,
        generation=1, epoch=0, now=0.0, wall_now=0.0, pose=(4.0, 4.0, 0.0),
        length_m=L, width_m=W, localization_valid=True,
    )
    assert res.granted is False
    assert res.reason is ReasonCode.PRECONDITION_NOT_REACHED
    assert mgr.book.queue(mgr.rid("mid")) == [], "an ineligible request must not hold a queue slot"
    assert mgr.book.owner_of(mgr.rid("mid")) is None
    assert spec.wait_node  # (documented: the only two legal places to ask)


def test_acquire_with_invalid_localization_is_refused(mgr):
    all_free(mgr)
    res = ask(mgr, WEST_TO_EAST, loc=False)
    assert res.granted is False
    assert res.reason is ReasonCode.LOCALIZATION_STALE


def test_alignment_node_is_a_legal_place_to_ask(mgr):
    all_free(mgr)
    res = ask(mgr, WEST_TO_EAST, at="align_west")
    assert res.granted is True


def test_being_at_the_node_is_not_enough_if_the_footprint_is_inside():
    """A robot straddling the mouth of the gap is not waiting, whatever the
    node's name says. Built from a config whose only node sits in the corridor,
    so the geometric second check is the only thing that can refuse it."""
    mgr = CrossingManager(validate_traffic_config(_raw(align_x=0.0)))
    all_free(mgr)
    res = mgr.acquire(
        "east_to_west", task_id="task-A", robot_id="r01", boot_id="b1", revision=0,
        generation=1, epoch=0, now=0.0, wall_now=0.0, pose=(0.0, 0.0, 0.0),
        length_m=L, width_m=W, localization_valid=True,
    )
    assert res.granted is False
    assert res.reason is ReasonCode.PRECONDITION_NOT_REACHED
    assert "gap" in res.blocked_by

    clear = CrossingManager(validate_traffic_config(_raw(align_x=-2.4)))
    clear._state.update({n: ResourceState.FREE for n in clear.cfg.resources})
    ok = clear.acquire(
        "east_to_west", task_id="task-A", robot_id="r01", boot_id="b1", revision=0,
        generation=1, epoch=0, now=0.0, wall_now=0.0, pose=(-2.4, 0.0, 0.0),
        length_m=L, width_m=W, localization_valid=True,
    )
    assert ok.granted is True, "the same code path must grant a genuinely clear pose"


def _raw(*, align_x: float) -> dict:
    return {
        "schema_version": 1,
        "resources": {
            "mid": {"kind": "CORRIDOR", "capacity": 1,
                    "rects": [{"name": "gap", "x_min": -1.75, "y_min": -0.65, "x_max": 1.75, "y_max": 0.65}]},
            "mid_left": {"kind": "EXIT_BUFFER", "capacity": 1,
                         "rects": [{"name": "pad", "x_min": -2.85, "y_min": -1.75, "x_max": -2.15, "y_max": -1.05}]},
        },
        "bundles": {"east_to_west": ["mid", "mid_left"]},
        "nodes": {"w": {"x": align_x, "y": 0.0, "yaw": 0.0}},
        "directions": {"east_to_west": {"wait_node": "w", "align_node": "w", "exit_node": "w",
                                        "release_node": "w", "entry_rect": "gap"}},
        "traffic": {
            "footprint_margin_m": 0.15, "node_reach_tolerance_m": 0.35,
            "permit_ttl_s": 12.0, "renew_before_s": 4.0,
            "stop_model": {"latency_s": 0.2, "a_stop_mps2": 0.4, "localization_margin_m": 0.1},
        },
    }


# --------------------------------------------------------------------------- #
# mutual exclusion and fairness
# --------------------------------------------------------------------------- #


def test_second_robot_queues_and_is_served_in_order(mgr):
    all_free(mgr)
    assert ask(mgr, WEST_TO_EAST, task="task-A", robot="r01").granted is True
    second = ask(mgr, EAST_TO_WEST, task="task-B", robot="r02")
    assert second.granted is False
    assert second.reason is ReasonCode.RESOURCE_BUSY
    assert mgr.book.queue(mgr.rid("mid")) == ["task-B"]

    outcome = mgr.confirm_clear(
        WEST_TO_EAST, task_id="task-A", generation=1,
        pose=node_pose(mgr.cfg, "release_east"), length_m=L, width_m=W,
    )
    assert outcome.cleared is True

    third = ask(mgr, EAST_TO_WEST, task="task-B", robot="r02")
    assert third.granted is True
    assert mgr.book.owner_of(mgr.rid("mid")) == "task-B"


def test_occupied_exit_buffer_blocks_the_bundle(mgr):
    """F03: never admit a robot to a corridor it cannot leave."""
    all_free(mgr)
    # r02 is parked on the west exit buffer, holding only that.
    parked = mgr.book.acquire(
        task_id="task-parked", robot_id="r02", resources=(mgr.rid("mid_left"),),
        now=0.0, wall_now=0.0, boot_id="b1", revision=0, generation=1, epoch=0,
    )
    assert parked.granted is True

    res = ask(mgr, EAST_TO_WEST, task="task-A", robot="r01")
    assert res.granted is False
    assert mgr.book.owner_of(mgr.rid("mid")) is None, "no half-granted bundle may exist"


def test_repeated_round_trips_do_not_leak_queue_entries(mgr):
    """Neither direction may be permanently locked out.

    Three full round trips each way. A released bundle that left a stale queue
    entry behind would make the *next* request from the other direction fail
    RESOURCE_BUSY forever -- a slow starvation that looks like a busy fleet.
    """
    all_free(mgr)
    for i in range(3):
        for direction, task, robot, release in (
            (WEST_TO_EAST, f"A{i}", "r01", "release_east"),
            (EAST_TO_WEST, f"B{i}", "r02", "release_west"),
        ):
            got = ask(mgr, direction, task=task, robot=robot, gen=i + 1)
            assert got.granted is True, f"{direction} starved on round {i}"
            outcome = mgr.confirm_clear(
                direction, task_id=task, generation=i + 1,
                pose=node_pose(mgr.cfg, release), length_m=L, width_m=W,
            )
            assert outcome.cleared is True
            assert mgr.book.total_queued() == 0, "a clearance must not leave a queue entry behind"


# --------------------------------------------------------------------------- #
# the gate's question
# --------------------------------------------------------------------------- #


def test_stationary_robot_without_a_permit_may_idle_at_the_wait_node(mgr):
    """Waiting must not be a refusal. Otherwise the fleet is deadlocked at boot,
    because the first thing every robot does is drive to a waiting point."""
    check = move(mgr, pose=node_pose(mgr.cfg, "wait_west"))
    assert check.allowed is True


def test_robot_inside_the_corridor_without_a_permit_is_stopped(mgr):
    check = move(mgr, pose=(0.0, 0.0, 0.0))
    assert check.allowed is False
    assert check.reason is ReasonCode.ROUTE_REQUIRES_PERMIT
    assert "mid" in check.stop_boundary_hit


def test_holder_may_drive_through_its_own_bundle(mgr):
    all_free(mgr)
    assert ask(mgr, WEST_TO_EAST).granted is True
    for pose in ((0.0, 0.0, 0.0), node_pose(mgr.cfg, "align_west"), node_pose(mgr.cfg, "exit_east")):
        assert move(mgr, pose=pose).allowed is True, f"permitted motion refused at {pose}"


def test_holder_may_not_drive_into_the_other_exit_buffer(mgr):
    """The per-resource provenance test.

    Holding ``[mid, mid_right]`` authorises the corridor and the east buffer. It
    does not authorise the west buffer. A bundle-level check -- "does the robot
    hold its bundle? yes" -- would let this through, and the two robots would
    then park on top of each other.
    """
    all_free(mgr)
    assert ask(mgr, WEST_TO_EAST).granted is True
    check = move(mgr, pose=node_pose(mgr.cfg, "exit_west"))
    assert check.allowed is False
    assert check.stop_boundary_hit == ("mid_left",)
    assert check.reason is ReasonCode.ROUTE_REQUIRES_PERMIT


def test_invalid_localization_stops_a_robot_that_holds_a_permit(mgr):
    """Fail closed: a permit is not a licence to keep moving blind."""
    all_free(mgr)
    assert ask(mgr, WEST_TO_EAST).granted is True
    check = move(mgr, pose=(0.0, 0.0, 0.0), loc=False)
    assert check.allowed is False
    assert check.reason is ReasonCode.LOCALIZATION_STALE


def test_superseded_generation_reports_a_stale_permit(mgr):
    all_free(mgr)
    assert ask(mgr, WEST_TO_EAST, gen=1).granted is True
    check = move(mgr, gen=2, pose=(0.0, 0.0, 0.0))
    assert check.allowed is False
    assert check.reason is ReasonCode.PERMIT_STALE


def test_faster_motion_shrinks_the_authorised_area(mgr):
    """The stop boundary must grow with speed, so the refusal comes earlier.

    Same pose, two speeds, placed so that the stationary case is legal and the
    moving one is not. If speed did not enter the test, they would agree.
    """
    pose = (-2.35, 0.0, 0.0)  # outside the gap rectangle, inside its stop boundary at speed
    still = move(mgr, pose=pose, twist=(0.0, 0.0, 0.0))
    fast = move(mgr, pose=pose, twist=(0.35, 0.0, 0.0))
    assert still.allowed is True
    assert fast.allowed is False
    assert fast.reason is ReasonCode.ROUTE_REQUIRES_PERMIT


def test_turning_in_place_counts_as_motion_too(mgr):
    pose = (-2.35, 0.0, 0.0)
    assert move(mgr, pose=pose, twist=(0.0, 0.0, 0.0)).allowed is True
    assert move(mgr, pose=pose, twist=(0.0, 0.0, 1.5)).allowed is False


# --------------------------------------------------------------------------- #
# occupancy transitions
# --------------------------------------------------------------------------- #


def test_enter_requires_actually_being_inside(mgr):
    """RESERVED means "may enter". Marking OCCUPIED from the waiting point would
    report the corridor busy while it is in fact still empty."""
    all_free(mgr)
    assert ask(mgr, WEST_TO_EAST).granted is True
    assert (
        mgr.enter(WEST_TO_EAST, task_id="task-A", boot_id="b1", revision=0, generation=1,
                  epoch=0, wall_now=0.0, pose=node_pose(mgr.cfg, "wait_west"), length_m=L, width_m=W)
        is False
    )
    assert mgr.state_of("mid") is ResourceState.RESERVED

    assert (
        mgr.enter(WEST_TO_EAST, task_id="task-A", boot_id="b1", revision=0, generation=1,
                  epoch=0, wall_now=0.0, pose=(0.0, 0.0, 0.0), length_m=L, width_m=W)
        is True
    )
    assert mgr.state_of("mid") is ResourceState.OCCUPIED


def test_enter_refuses_a_generation_that_holds_nothing(mgr):
    all_free(mgr)
    assert ask(mgr, WEST_TO_EAST, gen=1).granted is True
    assert (
        mgr.enter(WEST_TO_EAST, task_id="task-A", boot_id="b1", revision=0, generation=99,
                  epoch=0, wall_now=0.0, pose=(0.0, 0.0, 0.0), length_m=L, width_m=W)
        is False
    )


# --------------------------------------------------------------------------- #
# clearance: the rule that keeps the corridor honest
# --------------------------------------------------------------------------- #


def test_clearance_is_refused_while_parked_on_the_exit_buffer(mgr):
    """The exit buffer is INSIDE the bundle by design.

    A robot that stops there has not left. Releasing at that moment is exactly
    how the next robot is let into a corridor whose exit is still blocked.
    """
    all_free(mgr)
    assert ask(mgr, WEST_TO_EAST).granted is True
    outcome = mgr.confirm_clear(
        WEST_TO_EAST, task_id="task-A", generation=1,
        pose=node_pose(mgr.cfg, "exit_east"), length_m=L, width_m=W,
    )
    assert outcome.cleared is False
    assert outcome.reason is ReasonCode.CLEARANCE_NOT_PROVEN
    assert "pad" in outcome.still_inside
    assert mgr.state_of("mid") is ResourceState.CLEARING
    assert mgr.book.owner_of(mgr.rid("mid")) == "task-A", "the hold must survive a refused clearance"


def test_clearance_succeeds_at_the_release_node(mgr):
    all_free(mgr)
    assert ask(mgr, WEST_TO_EAST).granted is True
    outcome = mgr.confirm_clear(
        WEST_TO_EAST, task_id="task-A", generation=1,
        pose=node_pose(mgr.cfg, "release_east"), length_m=L, width_m=W,
    )
    assert outcome.cleared is True
    assert mgr.state_of("mid") is ResourceState.FREE
    assert mgr.state_of("mid_right") is ResourceState.FREE
    assert mgr.book.owner_of(mgr.rid("mid")) is None


def test_clearance_from_a_stale_generation_is_refused(mgr):
    all_free(mgr)
    assert ask(mgr, WEST_TO_EAST, gen=1).granted is True
    outcome = mgr.confirm_clear(
        WEST_TO_EAST, task_id="task-A", generation=7,
        pose=node_pose(mgr.cfg, "release_east"), length_m=L, width_m=W,
    )
    assert outcome.cleared is False
    assert outcome.reason is ReasonCode.PERMIT_STALE
    assert mgr.book.owner_of(mgr.rid("mid")) == "task-A"


def test_every_release_node_is_geometrically_outside_its_bundle(tcfg):
    """A release node that is inside the bundle can never produce a clearance.

    Checked here for all four directions x both robots' footprints, so a future
    edit to a coordinate fails the suite instead of producing a robot that drives
    to the release node and is told forever that it has not arrived.
    """
    for robot_len, robot_wid in ((0.60, 0.45), (0.70, 0.50)):
        for direction in tcfg.directions:
            mgr = CrossingManager(tcfg)
            spec = mgr.direction_spec(direction)
            pose = node_pose(tcfg, spec.release_node)
            names = mgr.bundle(direction)
            region = mgr.region_of(names)
            corners = footprint_corners(pose[0], pose[1], pose[2], robot_len, robot_wid)
            assert region.fully_outside(corners), (
                f"{direction} / footprint {robot_len}x{robot_wid}: release node "
                f"{spec.release_node} is not clear of {list(names)}"
            )


def test_every_wait_node_is_outside_the_whole_crossing(tcfg):
    """Waiting points must be outside *every* rectangle, not just the bundle.

    A robot waiting on the other side's exit buffer would be squatting inside the
    protected area while holding no permit for it.
    """
    for direction in tcfg.directions:
        mgr = CrossingManager(tcfg)
        spec = mgr.direction_spec(direction)
        pose = node_pose(tcfg, spec.wait_node)
        region = mgr.full_region()
        corners = footprint_corners(pose[0], pose[1], pose[2], 0.60, 0.45)
        assert region.fully_outside(corners), (
            f"{direction}: wait node {spec.wait_node} is inside the crossing"
        )


# --------------------------------------------------------------------------- #
# expiry
# --------------------------------------------------------------------------- #


def test_ttl_lapse_makes_the_resource_unknown_not_free(mgr):
    """F10: the lease lapsed. That is not evidence the corridor is empty."""
    all_free(mgr)
    assert ask(mgr, WEST_TO_EAST).granted is True
    touched = mgr.expire_due(wall_now=1e6)
    assert set(touched) == {"mid", "mid_right"}
    assert mgr.state_of("mid") is ResourceState.UNKNOWN
    assert mgr.state_of("mid") is not ResourceState.FREE
    assert mgr.block_reason("mid") is not None


def test_lapsed_holder_is_told_its_permit_expired(mgr):
    all_free(mgr)
    assert ask(mgr, WEST_TO_EAST, wall_now=0.0).granted is True
    mgr.expire_due(wall_now=1e6)
    check = move(mgr, pose=(0.0, 0.0, 0.0), wall_now=1e6)
    assert check.allowed is False
    assert check.reason is ReasonCode.PERMIT_EXPIRED


def test_a_stranger_is_told_the_resource_is_unknown(mgr):
    """Same event, different robot. It never held anything, so for it the corridor
    is simply not available yet -- and it is stopped before it can reach it."""
    all_free(mgr)
    assert ask(mgr, WEST_TO_EAST, task="task-A", wall_now=0.0).granted is True
    mgr.expire_due(wall_now=1e6)

    # Just outside the gap, creeping up on it.
    check = move(mgr, task="task-B", pose=(-2.05, 0.0, 0.0), wall_now=1e6)
    assert check.allowed is False
    assert check.reason is ReasonCode.RESOURCE_UNKNOWN


def test_renew_before_expiry_extends_the_lease(mgr):
    all_free(mgr)
    assert ask(mgr, WEST_TO_EAST, gen=1, wall_now=0.0).granted is True
    permit = mgr.renew(WEST_TO_EAST, task_id="task-A", boot_id="b1", revision=0, generation=1, epoch=0, wall_now=5.0)
    assert permit.expires_at > 5.0
    assert mgr.expire_due(wall_now=5.1) == ()


def test_renew_after_expiry_is_refused(mgr):
    """Otherwise a lapse could be laundered into a legal occupancy."""
    all_free(mgr)
    assert ask(mgr, WEST_TO_EAST, gen=1, wall_now=0.0).granted is True
    mgr.expire_due(wall_now=1e6)
    with pytest.raises(StaleCommandError, match="must be re-acquired"):
        mgr.renew(WEST_TO_EAST, task_id="task-A", boot_id="b1", revision=0, generation=1, epoch=0, wall_now=1e6)


def test_blocking_records_a_reason(mgr):
    mgr.block("mid", "operator inspecting spilled load")
    assert mgr.state_of("mid") is ResourceState.BLOCKED
    assert "spilled" in (mgr.block_reason("mid") or "")
    with pytest.raises(ValueError):
        mgr.block("mid", "")


def test_snapshot_names_owner_and_queue(mgr):
    all_free(mgr)
    assert ask(mgr, WEST_TO_EAST, task="task-A", robot="r01").granted is True
    ask(mgr, EAST_TO_WEST, task="task-B", robot="r02")
    snap = mgr.snapshot()
    assert snap["mid"]["owner"] == "task-A"
    assert snap["mid"]["state"] == ResourceState.RESERVED.value
    assert snap["mid_left"]["owner"] is None
    assert "task-B" in snap["mid"]["queue"]
