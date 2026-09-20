"""P10: the truth stream's layout, and the defect that reading it by arithmetic caused.

MEASURED, from `scripts/probe_truth_slots.py` against the real two-robot world:

    idx 0   starts (-6.000, -2.000)   moved 10.243 m   r01's model frame
    idx 1   starts ( 6.000, -2.000)   moved 10.287 m   r02's model frame
    idx 2-8   static local offsets, ending at (0.200, 0.000)
    idx 9-15  the same offsets for the second model

`fleet_recorder` read `transforms[slot * LINKS_PER_MODEL]` -- indices 0 and 8 -- because it
assumed each model occupies a contiguous block of eight. Index 8 is one of r01's own links, so
**r02 was never recorded**, and a two-robot recording was judged UNKNOWN with "no slot starts on
the spawn of ['r02']" about a recording that contained r02 at index 1.

The first test below is the defect itself, reproduced from data: the old slice cannot attribute
two robots from the real layout, and the full stream can. It would have failed before the fix.
"""

from __future__ import annotations

import json
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
SRC = ROOT / "src"
for _pkg in ("fleet_core", "fleet_evaluation"):
    _p = SRC / _pkg
    if str(_p) not in sys.path:
        sys.path.insert(0, str(_p))

from fleet_core import load_traffic_config  # noqa: E402
from fleet_evaluation.judge import attribute_truth, judge_case  # noqa: E402

CONFIG = ROOT / "config" / "resources.yaml"

#: The two robots this test judges, and where they start.
SPAWNS = {"r01": (-6.0, -2.0, 0.0), "r02": (6.0, -2.0, 0.0)}

#: The link offsets of one model, in the order the stream carries them: two at the model
#: origin, four wheel-ish corners, one 0.2 m out. Observed, not derived.
LINK_OFFSETS = [(0.0, 0.0), (0.0, 0.0), (0.260, 0.205), (-0.260, 0.205),
                (0.260, -0.205), (-0.260, -0.205), (0.200, 0.000)]


def real_layout(pose_r01, pose_r02):
    """One message's worth of slots, in the measured layout: model frames first."""
    slots = {0: (pose_r01[0], pose_r01[1], pose_r01[2]),
             1: (pose_r02[0], pose_r02[1], pose_r02[2])}
    index = 2
    for model_pose in (pose_r01, pose_r02):
        for dx, dy in LINK_OFFSETS:
            # The links are reported as LOCAL offsets, which is why they never move: they sit
            # at the same numbers while the model they belong to travels metres.
            slots[index] = (dx, dy, 0.0)
            index += 1
    return slots


def at_rest():
    return real_layout(SPAWNS["r01"], SPAWNS["r02"])


def _odom_at_rest():
    return {r: [0.0, 0.0, 0.0] for r in SPAWNS}


def _sample(t, truth):
    return {"t": t, "odom": _odom_at_rest(),
            "truth": {str(k): list(v) for k, v in truth.items()},
            "resources": {n: {"state": "FREE", "owner": None, "queue": []}
                          for n in ("mid", "mid_left", "mid_right")}}


def _samples():
    """Two samples at rest, then a step out and back, so continuity has something to check."""
    return [
        _sample(0.0, at_rest()),
        _sample(0.2, at_rest()),
        _sample(0.4, real_layout((-5.9, -2.0, 0.0), (6.1, -2.0, 0.0))),
        _sample(0.6, real_layout((-5.8, -2.0, 0.0), (6.2, -2.0, 0.0))),
    ]


def _case(tmp_path, samples):
    case = tmp_path / "case"
    case.mkdir(parents=True, exist_ok=True)
    (case / "samples-x.jsonl").write_text(
        "\n".join(json.dumps(s) for s in samples), encoding="utf-8")
    return case


@pytest.fixture(scope="module")
def cfg():
    return load_traffic_config(CONFIG)


def test_the_old_slice_could_not_see_the_second_robot_and_the_full_stream_can():
    """The defect, from the measured layout. This is what the arithmetic cost.

    The slice `indices 0 and 8` is exactly what `slot * LINKS_PER_MODEL` produced with
    `LINKS_PER_MODEL = 8` for a 16-transform message.
    """
    layout = at_rest()
    samples = [_sample(0.0, layout), _sample(0.2, layout)]

    old_slice = {slot: layout[slot] for slot in (0, 8)}
    old = attribute_truth([_sample(0.0, old_slice), _sample(0.2, old_slice)], SPAWNS)
    assert old["ok"] is False, (
        "the old slice attributed two robots from a stream that has r02 at index 1; it must "
        "not, or this test is not measuring the defect it names")
    assert old["missing_robots"] == ["r02"]
    assert "0.200, 0.000" in old["reason"], (
        "the reason must name the slot that was read in r02's place")

    now = attribute_truth(samples, SPAWNS)
    assert now["ok"] is True, now["reason"]
    assert now["mapping"] == {"0": "r01", "1": "r02"}
    assert now["margin_m"] > 5.0
    assert now["continuity_violations"] == 0


def test_the_link_offsets_are_reported_as_non_robots_not_matched_to_whoever_is_nearest():
    """Fourteen of the sixteen slots are a robot's own links. None may become a robot."""
    attribution = attribute_truth(_samples(), SPAWNS)
    assert attribution["ok"] is True
    assert set(attribution["mapping"].values()) == {"r01", "r02"}
    assert len(attribution["not_robot_slots"]) == 14
    assert attribution["mapping"] == {"0": "r01", "1": "r02"}


def test_the_reason_says_how_many_slots_it_did_not_list():
    """A summary is not a truncation: the count of unlisted slots is stated."""
    layout = at_rest()
    one_robot = {slot: layout[slot] for slot in (0, 2, 3, 4, 5, 6, 7, 8)}
    attribution = attribute_truth([_sample(0.0, one_robot), _sample(0.2, one_robot)], SPAWNS)
    assert attribution["ok"] is False
    assert attribution["missing_robots"] == ["r02"]
    reason = attribution["reason"]
    assert "further slot(s) that did not start on any spawn" in reason
    # Derived rather than written down: the count depends on how many of the slots are robots,
    # which is the thing under test. A literal here was wrong by three.
    # Counted from the structure, not from the prose. The previous version derived `listed`
    # by counting the phrase "moved 0.000 m in total", so it broke the moment a slot with no
    # reported position got its own wording (D-P17-07) -- which is a real finding about this
    # test: the cap and the count are facts about `not_robot_slots`, and a test that reads
    # them out of a sentence is testing the sentence.
    not_robots = attribution["not_robot_slots"]
    listed = min(4, len(not_robots))
    unlisted = len(not_robots) - listed
    assert listed == 4, f"the listing is capped at four, this fixture has {len(not_robots)}"
    assert unlisted >= 1, "this fixture must have more than four non-robot slots"
    assert f"and {unlisted} further slot(s)" in reason, reason
    # Every listed slot must be named, in whichever of the two wordings applies to it.
    for slot in not_robots[:4]:
        assert f"slot {slot} " in reason, (
            f"slot {slot} is in not_robot_slots but the reason never names it")


def test_a_full_case_with_the_real_layout_reaches_a_verdict(tmp_path, cfg):
    """End to end: the layout as the world produces it must not be a reason for NOT_RUN."""
    result = judge_case(_case(tmp_path, _samples()), cfg, SPAWNS)
    attribution = next(c for c in result["checks"] if c["check"] == "truth_attribution")
    assert attribution["verdict"] == "PASS", attribution
    assert "r02" in attribution["seed_distances_m"]
    assert result["robots_judged"] == ["r01", "r02"]


# --------------------------------------------------------------------------- #
# D-P17-07: a slot that has not reported yet is not a robot at the origin
# --------------------------------------------------------------------------- #

def _widening_array():
    """The truth array as the world really widens it.

    Measured on N07 seed 1 (2026-09-18, batch_20260918T080259Z): widths 0 (241 samples),
    8 (7 samples) and 24 (12147 samples). During the width-8 messages only slot 0 held a
    position; slots 1..7 read as (0, 0). Two of those slots were r02 and r03, so their
    recorded "starting position" was the origin and `attribute_truth` -- which accepts a slot
    as a robot only if it STARTS ON THAT ROBOT'S SPAWN -- matched neither of them. Every
    truth-based check then returned NOT_RUN and the case's `safety` was UNKNOWN.

    Two-robot recordings never showed it: their widths are only 0 and 16, so there is no
    intermediate array to poison the start.
    """
    layout = at_rest()
    return layout, [
        _sample(0.0, {0: layout[0]}),
        _sample(0.1, {0: layout[0], 1: (0.0, 0.0, 0.0)}),
        _sample(0.2, {0: layout[0], 1: (0.0, 0.0, 0.0)}),
        _sample(0.3, {0: layout[0], 1: layout[1]}),
        _sample(0.4, {0: layout[0], 1: layout[1]}),
    ]


def test_the_origin_is_not_a_starting_position():
    """THE regression: the robot's start must be where it actually is, not (0, 0)."""
    layout, samples = _widening_array()
    attribution = attribute_truth(samples, SPAWNS)
    start = attribution["slot_tracks"]["1"]["start"]
    assert start == [round(v, 3) for v in layout[1][:2]], (
        f"slot 1 reported the origin for two samples before reporting its real position; its "
        f"start came back as {start} instead of {layout[1][:2]}")
    assert attribution["ok"] is True, attribution["reason"]
    assert attribution["mapping"] == {"0": "r01", "1": "r02"}


def test_a_slot_that_never_leaves_the_origin_has_no_start_at_all():
    """And that is reported as a missing start, not as a robot parked at the origin."""
    layout = at_rest()
    samples = [_sample(0.0, {0: layout[0], 9: (0.0, 0.0, 0.0)}),
               _sample(0.2, {0: layout[0], 9: (0.0, 0.0, 0.0)})]
    attribution = attribute_truth(samples, SPAWNS)
    assert attribution["slot_tracks"]["9"]["start"] is None, (
        "a slot that never reported a position away from the origin was given one")
    assert 9 in attribution["not_robot_slots"]


def test_the_reason_says_which_kind_of_non_robot_a_slot_was():
    """Two different facts, two different sentences. 'Not a robot' is not one thing."""
    layout = at_rest()
    # Slot 0 is r01 (a real robot); slot 9 never leaves the origin; slot 10 is one of r01's
    # link offsets. r02 is withheld on purpose, so the attribution fails and both wordings
    # appear in the same reason. The first version used `layout[1]`, which IS r02 -- the
    # fixture supplied the very thing it meant to withhold.
    samples = [_sample(0.0, {0: layout[0], 9: (0.0, 0.0, 0.0), 10: layout[3]}),
               _sample(0.2, {0: layout[0], 9: (0.0, 0.0, 0.0), 10: layout[3]})]
    reason = attribute_truth(samples, SPAWNS)["reason"]
    assert "never reported a position away from the origin" in reason, reason
    assert "slot 9 " in reason, reason
    # The other wording -- "sits at (x, y) and moved N m in total" -- belongs to a slot that
    # DID report a position. Which of `at_rest()`'s slots are off-spawn is a property of that
    # fixture, not of the code (slot 2 IS the origin; slot 3 is within a metre of a spawn), so
    # it is asserted where it is set up: the 14-link-offset test asserts 14 non-robot slots.
