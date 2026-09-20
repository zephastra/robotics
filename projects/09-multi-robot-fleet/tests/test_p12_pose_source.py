"""`PoseSource` — one implementation of "which pose may I conclude from", two consumers.

`D-P11-02` was a bug in this bookkeeping, and it was only caught by a live run: the escape hatch
("a stale estimate is still true while the robot has not moved") could never open, because the
odometry pose from the estimate's instant was recorded as `None`. The node-level workaround is
pinned by an AST test; these tests pin the *behaviour*, in pure Python, so the next change to it
cannot pass review by looking plausible.

`D-P11-03` is why the class exists at all: the coordinator judged from a composed pose whose error
(1.5516 m median) was larger than the tolerance it was compared against (0.80 m). Two components
that must agree about where a robot is may not each keep their own copy of the answer.
"""

from __future__ import annotations

import pathlib
import sys

import pytest

ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src" / "fleet_core"))

from fleet_core.pose_source import (  # noqa: E402
    POSE_SOURCE_LOCALISER,
    POSE_SOURCE_SPAWN_ODOM,
    LocaliserPolicy,
    PoseSource,
)


def _spawn_odom(**kw) -> PoseSource:
    return PoseSource(source=POSE_SOURCE_SPAWN_ODOM, policy=LocaliserPolicy(), **kw)


def _localiser(**kw) -> PoseSource:
    return PoseSource(source=POSE_SOURCE_LOCALISER, policy=LocaliserPolicy(), **kw)


# --------------------------------------------------------------------------- #
# the parameter must not have a silent default
# --------------------------------------------------------------------------- #
def test_an_unknown_source_is_refused_at_construction():
    with pytest.raises(ValueError) as exc:
        PoseSource(source="odom_ish", policy=LocaliserPolicy())
    assert "not one of" in str(exc.value)
    # The refusal must name the alternatives, or an operator cannot fix it from the message.
    assert POSE_SOURCE_SPAWN_ODOM in str(exc.value)
    assert POSE_SOURCE_LOCALISER in str(exc.value)


# --------------------------------------------------------------------------- #
# spawn_odom: the previous behaviour, unchanged
# --------------------------------------------------------------------------- #
def test_spawn_odom_composes_the_spawn_with_odometry():
    source = _spawn_odom(spawn=(5.0, -1.0, 0.0), odom_timeout_s=1.5)
    composed = source.note_odom(raw=(1.0, 2.0, 0.0), wall=100.0)
    assert composed == (6.0, 1.0, 0.0)
    verdict = source.trusted(now=100.5)
    assert verdict.usable
    assert verdict.pose == (6.0, 1.0, 0.0)
    assert verdict.source == POSE_SOURCE_SPAWN_ODOM


def test_spawn_odom_refuses_when_odometry_stops():
    source = _spawn_odom(odom_timeout_s=1.0)
    source.note_odom(raw=(0.0, 0.0, 0.0), wall=0.0)
    assert source.trusted(now=0.9).usable
    later = source.trusted(now=1.1)
    assert not later.usable
    assert later.pose is None
    assert "no odometry within" in later.reason


# --------------------------------------------------------------------------- #
# localiser: the three outcomes
# --------------------------------------------------------------------------- #
def test_with_no_estimate_it_refuses_and_says_what_is_missing():
    verdict = _localiser().trusted(now=0.0)
    assert not verdict.usable
    assert verdict.pose is None
    assert "no localised pose has arrived" in verdict.reason


def test_a_fresh_estimate_is_used_directly():
    source = _localiser()
    source.note_localiser(pose=(1.0, 2.0, 0.5), wall=10.0)
    verdict = source.trusted(now=10.5)
    assert verdict.usable and verdict.pose == (1.0, 2.0, 0.5)
    assert verdict.source == POSE_SOURCE_LOCALISER


def test_a_stale_estimate_that_has_not_moved_is_still_used():
    """The hatch. Without it a stationary robot is refused for ever."""
    source = _localiser()
    source.note_odom(raw=(0.0, 0.0, 0.0), wall=9.9)
    source.note_localiser(pose=(1.0, 2.0, 0.0), wall=10.0)
    source.note_odom(raw=(0.01, 0.0, 0.0), wall=310.0)
    verdict = source.trusted(now=310.0)
    assert verdict.usable, verdict.reason
    assert "has moved only" in verdict.reason


def test_a_stale_estimate_that_has_moved_is_refused():
    source = _localiser()
    source.note_odom(raw=(0.0, 0.0, 0.0), wall=9.9)
    source.note_localiser(pose=(1.0, 2.0, 0.0), wall=10.0)
    # 20 s, not 2 s: the age bound is 2.0 s and `<=` is satisfied AT the bound, so a fixture that
    # lands on it tests the boundary rather than the stale branch -- and passes for the wrong
    # reason. (Round 10 learned the same thing about a 17.5 mm threshold.)
    source.note_odom(raw=(0.5, 0.0, 0.0), wall=20.0)
    verdict = source.trusted(now=20.0)
    assert not verdict.usable, verdict.reason
    assert verdict.pose is None, "a refused verdict must not hand back a pose to act on"
    assert "has moved 0.500 m" in verdict.reason


# --------------------------------------------------------------------------- #
# the loop: an estimate that beat odometry to the node
# --------------------------------------------------------------------------- #
def test_an_estimate_arriving_before_any_odometry_can_still_open_the_hatch():
    """`D-P11-02`, as a test.

    Live: AMCL publishes its initial pose the moment it activates and the odometry sample came
    after it, so the arrival snapshot was `None`, the displacement stayed unknown for ever, and the
    gate refused every tick. `amcl_pose` is motion-gated, so refusing motion kept it stale -- 1
    message and 0.000 m of travel over 640 s.
    """
    source = _localiser()
    source.note_localiser(pose=(1.0, 2.0, 0.0), wall=10.0)   # before any odometry
    assert source.moved_since_estimate() is None, "the fixture must reproduce the unknown case"

    source.note_odom(raw=(0.0, 0.0, 0.0), wall=10.02)         # the first sample, one period later
    assert source.moved_since_estimate() == 0.0

    source.note_odom(raw=(0.01, 0.0, 0.0), wall=300.0)        # still barely moving
    verdict = source.trusted(now=300.0)
    assert verdict.usable, (
        "an estimate that arrived before the first odometry sample must not disable the hatch "
        f"for ever: {verdict.reason}")
    assert verdict.pose == (1.0, 2.0, 0.0)


def test_the_unmeasured_window_is_one_odometry_period():
    """State the assumption rather than hide it: motion before the first sample is not seen."""
    source = _localiser()
    source.note_localiser(pose=(1.0, 2.0, 0.0), wall=10.0)
    # The robot moved 2 m before odometry was ever seen; the hatch cannot know, and must not
    # pretend the robot is still at the estimate -- it starts counting from the first sample.
    source.note_odom(raw=(2.0, 0.0, 0.0), wall=10.02)
    assert source.moved_since_estimate() == 0.0
    source.note_odom(raw=(2.0, 0.0, 0.0), wall=400.0)
    assert source.trusted(now=400.0).usable
    # At least the pose is reported as stale, so a reader can see the age.
    assert source.localiser_age_s(400.0) == pytest.approx(390.0)


# --------------------------------------------------------------------------- #
# reporting, which must never masquerade as deciding
# --------------------------------------------------------------------------- #
def test_the_disagreement_is_reported_and_never_used():
    source = _localiser(spawn=(0.0, 0.0, 0.0))
    source.note_odom(raw=(3.0, 4.0, 0.0), wall=1.0)     # composed = (3,4)
    source.note_localiser(pose=(0.0, 0.0, 0.0), wall=1.1)
    assert source.disagreement_m() == pytest.approx(5.0)
    # The decision is still the localiser, at the localiser's coordinates.
    assert source.trusted(now=1.2).pose == (0.0, 0.0, 0.0)
    assert source.composed == (3.0, 4.0, 0.0)


def test_the_message_count_is_kept_so_silence_is_visible():
    source = _localiser()
    assert source.localiser_messages == 0
    source.note_localiser(pose=(0.0, 0.0, 0.0), wall=1.0)
    source.note_localiser(pose=(0.0, 0.0, 0.0), wall=2.0)
    assert source.localiser_messages == 2
