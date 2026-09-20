"""D-P14-02's second half: the retreat EMITTER, held to the four conditions that bound it.

`D-P4-12` proposed the retreat PERMIT -- a robot inside a rectangle may move iff the motion
strictly reduces its overlap. That is the safety half, and it is deliberately not implemented
yet. `D-P14-02` records why the emitter comes first: without a command to move, the widest
permit moves nothing, and `check_movement`'s refusal zeroes the velocity, which reads as a
navigation fault and is not one.

These tests pin the emitter's decisions. They are pure: no ROS, no simulator, no robot.

The four conditions each exist because leaving one out produced a wrong answer somewhere in
this project's history, so each has its own test naming the reason.
"""
from __future__ import annotations

import pathlib
import sys

import pytest

ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src/fleet_core"))

import fleet_core.retreat_policy as rp  # noqa: E402

# The exit pad actually used by config/resources.yaml: x in [-2.85, -2.15], y in [-1.75, -1.05]
PAD = {"pad": (-2.85, -1.75, -2.15, -1.05)}
STOPPED = (0.0, 0.0, 0.0)


# --------------------------------------------------------------------------- #
# condition 1: it must be inside, and we must measure HOW FAR in
# --------------------------------------------------------------------------- #


def test_outside_every_rectangle_needs_no_retreat():
    """Nothing to retreat from."""
    d = rp.needs_retreat((0.0, 0.0, 0.0), STOPPED, PAD, permittable=False)
    assert d.needed is False


def test_inside_an_unpermitted_rectangle_when_stopped_needs_a_retreat():
    """The measured situation: stopped inside the pad, refused, velocity zeroed."""
    d = rp.needs_retreat((-2.5, -1.4, 0.0), STOPPED, PAD, permittable=False)
    assert d.needed is True
    assert d.overlap_m > 0.0


def test_the_depth_is_the_shortest_way_out_not_the_distance_to_the_centre():
    """A robot just inside a corner is SHALLOW, not deep.

    Measuring from the centre calls a corner entry "deeply inside" and issues a retreat the
    robot does not need. The depth is what the retreat has to reduce, so it has to be the
    distance that actually has to be travelled.
    """
    # (-2.82, -1.72) is 0.03 m from two edges -> shallow, despite being far from the centre
    d = rp.needs_retreat((-2.82, -1.72, 0.0), STOPPED, PAD, permittable=False)
    assert d.overlap_m == pytest.approx(0.03, abs=1e-6), d
    # the centre is 0.35 m from every edge
    c = rp.needs_retreat((-2.5, -1.4, 0.0), STOPPED, PAD, permittable=False)
    assert c.overlap_m == pytest.approx(0.35, abs=1e-6), c


# --------------------------------------------------------------------------- #
# condition 2: the robot must be stopped
# --------------------------------------------------------------------------- #


def test_a_robot_still_moving_is_not_retreated():
    """Pre-empting a live navigation goal is what the 60 s `busy` budget was spent on.

    A robot whose own plan is running is not stuck, and issuing a competing goal gets it
    rejected for the whole budget -- measured, and the reason `busy_budget_s` exists.
    """
    d = rp.needs_retreat((-2.5, -1.4, 0.0), (0.20, 0.0, 0.0), PAD, permittable=False)
    assert d.needed is False
    assert "still moving" in d.reason


def test_the_moving_threshold_is_named_not_implied():
    """The boundary has to be a number in one place, so changing it is one edit."""
    assert rp.STOPPED_SPEED_MPS > 0.0
    just_under = rp.needs_retreat((-2.5, -1.4, 0.0),
                                  (rp.STOPPED_SPEED_MPS * 0.9, 0.0, 0.0),
                                  PAD, permittable=False)
    just_over = rp.needs_retreat((-2.5, -1.4, 0.0),
                                 (rp.STOPPED_SPEED_MPS * 1.1, 0.0, 0.0),
                                 PAD, permittable=False)
    assert just_under.needed is True
    assert just_over.needed is False


# --------------------------------------------------------------------------- #
# condition 3: a robot that COULD be granted the region must be granted it
# --------------------------------------------------------------------------- #


def test_a_permittable_robot_is_granted_the_region_not_driven_out_of_it():
    """This is a safety-path judgement, so it is asserted rather than left to the caller.

    Retreating a robot that had a legal path forward introduces motion on the safety-critical
    path that nobody asked for. Granting the resource is the answer; driving the robot out of
    a region it is entitled to enter is not.
    """
    d = rp.needs_retreat((-2.5, -1.4, 0.0), STOPPED, PAD, permittable=True)
    assert d.needed is False
    assert "grant" in d.reason


# --------------------------------------------------------------------------- #
# the command: which way, and how far
# --------------------------------------------------------------------------- #


def test_the_retreat_goes_out_through_the_nearest_edge():
    """Nearest edge, not the way it came in -- the entry heading is not recorded anywhere."""
    # near the x_max edge (x = -2.15) -> +x
    d = rp.retreat_command((-2.20, -1.40, 0.0), PAD)
    assert d.goal[0] > -2.20, d
    # near the y_min edge (y = -1.75) -> -y
    e = rp.retreat_command((-2.50, -1.70, 0.0), PAD)
    assert e.goal[1] < -1.70, e


def test_the_goal_lands_OUTSIDE_the_rectangle_with_room_to_be_remembered():
    """Stopping exactly on the edge is not outside it."""
    d = rp.retreat_command((-2.5, -1.4, 0.0), PAD)
    gx, gy = d.goal
    x_min, y_min, x_max, y_max = PAD["pad"]
    assert not (x_min <= gx <= x_max and y_min <= gy <= y_max), (
        "the retreat goal is still inside the rectangle it is retreating from"
    )


def test_the_distance_covers_the_depth_plus_the_configured_standoff():
    """The standoff is the part that makes the goal outside rather than on the boundary."""
    d = rp.retreat_command((-2.5, -1.4, 0.0), PAD, distance_m=1.20)
    assert d.distance_m == pytest.approx(0.35 + 1.20, abs=1e-6), d


def test_a_deeper_robot_is_sent_further():
    """The distance is derived from the depth, not a constant, or a deep robot stays inside."""
    shallow = rp.retreat_command((-2.80, -1.40, 0.0), PAD)   # 0.05 m deep
    deep = rp.retreat_command((-2.50, -1.40, 0.0), PAD)      # 0.35 m deep
    assert deep.distance_m > shallow.distance_m


def test_no_retreat_command_is_issued_when_none_is_needed():
    """The command path must not invent a goal for a robot that is outside."""
    d = rp.retreat_command((0.0, 0.0, 0.0), PAD)
    assert d.needed is False
    assert d.goal is None


def test_a_degenerate_rectangle_is_skipped_rather_than_used():
    """A zero-width rectangle has no nearest edge, so its direction would be arbitrary."""
    bad = {"bad": (-2.5, -1.4, -2.5, -1.4)}
    d = rp.needs_retreat((-2.5, -1.4, 0.0), STOPPED, bad, permittable=False)
    assert d.needed is False, (
        "a degenerate rectangle produced a retreat; its exit direction is undefined"
    )


def test_the_deepest_rectangle_is_the_one_retreated_from():
    """With two overlaps the robot must leave the one it is most inside."""
    two = {"shallow": (-2.90, -1.50, -2.70, -1.30),   # 0.10 m deep at (-2.80,-1.40)
           "deep": (-2.90, -1.80, -2.20, -1.10)}       # deeper
    d = rp.retreat_command((-2.80, -1.40, 0.0), two)
    assert "deep" in d.reason, d.reason


# --------------------------------------------------------------------------- #
# progress: the loop this module exists to stop
# --------------------------------------------------------------------------- #


def test_a_retreat_that_reduces_the_overlap_is_working():
    p = rp.progress(before_m=0.35, after_m=0.05)
    assert p["working"] is True
    assert p["reduced_by_m"] == pytest.approx(0.30)


def test_a_retreat_that_does_not_reduce_the_overlap_must_not_be_re_issued():
    """The `recoveries=8` shape: retrying a command that has been measured not to work.

    The fix for that loop was to notice and stop, and the same applies here. A retreat that
    changes nothing is a retreat that will change nothing next time either.
    """
    p = rp.progress(before_m=0.35, after_m=0.35)
    assert p["working"] is False
    assert "must not be re-issued" in p["detail"]


def test_a_reduction_smaller_than_the_epsilon_is_not_progress():
    """Numerical noise must not read as success, or the loop never exits."""
    p = rp.progress(before_m=0.350, after_m=0.349)
    assert p["working"] is False


def test_the_epsilon_is_a_named_constant():
    assert rp.PROGRESS_EPSILON_M > 0.0
