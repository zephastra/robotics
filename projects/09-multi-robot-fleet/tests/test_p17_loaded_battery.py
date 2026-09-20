"""Round 17 item 8b: CONTRACTS section 5's LOADED half (D-P17-02).

The contract states the rule as two cases and only one was implemented:

  * not carrying cargo, low battery -> cancel the leg, generate a charge run.   [existed]
  * carrying cargo, low battery     -> do NOT auto-unload. Park at a legal, reachable
                                       position and raise NEEDS_ATTENTION.        [this]

`D-P17-02` records why the second half could not fire: `_service_battery` opened with
`if state.executing or not self._fresh(rid, now): continue`, so a robot mid-leg was never
considered -- and a robot mid-leg is the only robot that can be carrying cargo on a
critical battery while executing.

These tests are deliberately written so that each of the three parts of the rule can fail
ALONE. An order bug and a geometry bug look identical from the outside ("the robot went
somewhere it should not have"), and D-P17-03 is the record of what that costs.
"""
from __future__ import annotations

import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent / "src/fleet_core"))

from fleet_core.loaded_battery_policy import (  # noqa: E402
    ACTION_CANCEL_AND_CHARGE,
    ACTION_CONTINUE,
    ACTION_PARK_AND_ATTEND,
    LOADED_PARK_BANDS,
    PROTECTED_RECTS,
    decide,
    legal_park_point,
    ownership_preserved,
)

# --------------------------------------------------------------------------- #
# 1. the band
# --------------------------------------------------------------------------- #


def test_the_rule_acts_only_on_critical():
    """A named band, not an inlined string: F05 asserts the band and a band spelled in
    two files drifts."""
    assert LOADED_PARK_BANDS == ("CRITICAL",)


def test_above_the_band_is_continue_even_when_carrying():
    """The reason it is safe to call this every tick. A rule that acted on LOW would
    park a loaded robot a long way from the task it can still finish."""
    for band in ("NOMINAL", "LOW", "WARN"):
        d = decide(band, executing=True, carrying=True, has_charge_task=False)
        assert d.action == ACTION_CONTINUE, band
        assert d.carrying is True, "the flag is reported, not inferred"


# --------------------------------------------------------------------------- #
# 2. the ORDER of the two branches is the rule
# --------------------------------------------------------------------------- #


def test_carrying_on_critical_parks_instead_of_charging():
    """The loaded half. Not a charge run: the contract forbids auto-unload, and sending
    the cargo to a charger is the failure this branch exists to prevent."""
    d = decide("CRITICAL", executing=True, carrying=True, has_charge_task=False)
    assert d.action == ACTION_PARK_AND_ATTEND
    assert d.action != ACTION_CANCEL_AND_CHARGE
    assert d.carrying is True
    assert "auto-unload" in d.reason


def test_the_loaded_half_wins_over_the_unloaded_half():
    """The order test, stated as the thing it prevents.

    `has_charge_task=True` is the exact input that would take the unloaded branch if the
    order were reversed: a charge run is already outstanding, so the "cancel and charge"
    rule looks satisfied. On a LOADED robot that is wrong -- it means the cargo is on a
    robot the fleet believes is driving to a pad. The decision must not depend on it.
    """
    with_outstanding_run = decide(
        "CRITICAL", executing=True, carrying=True, has_charge_task=True
    )
    without = decide(
        "CRITICAL", executing=True, carrying=True, has_charge_task=False
    )
    assert with_outstanding_run.action == without.action == ACTION_PARK_AND_ATTEND, (
        "a loaded robot's outcome must not be moved by an unloaded robot's rule"
    )


def test_there_is_no_auto_unload_option_at_all():
    """The contract forbids it, and offering the option is how it comes back.

    Checked structurally rather than by asserting on the returned action, because the
    failure mode is a future edit adding a fourth constant -- not this function
    returning it today.
    """
    import fleet_core.loaded_battery_policy as mod

    names = [n for n in dir(mod) if n.startswith("ACTION_")]
    assert set(names) == {
        "ACTION_CANCEL_AND_CHARGE",
        "ACTION_CONTINUE",
        "ACTION_PARK_AND_ATTEND",
    }, f"a new action appeared: {sorted(names)}"


def test_unloaded_critical_executing_cancels_and_charges():
    """The first half, kept working. It already existed, but the skip meant it could only
    ever fire for an idle robot."""
    d = decide("CRITICAL", executing=True, carrying=False, has_charge_task=False)
    assert d.action == ACTION_CANCEL_AND_CHARGE
    assert d.carrying is False


def test_unloaded_critical_idle_also_charges():
    d = decide("CRITICAL", executing=False, carrying=False, has_charge_task=False)
    assert d.action == ACTION_CANCEL_AND_CHARGE


def test_decision_is_frozen_and_self_describing():
    """Every decision carries a sentence. `D-P17-03` is the record of a check that could
    report an outcome but not a reason."""
    d = decide("CRITICAL", executing=True, carrying=True, has_charge_task=False)
    assert d.reason and len(d.reason) > 20
    try:
        d.action = "something else"  # type: ignore[misc]
    except Exception:
        return
    raise AssertionError("LoadedBatteryDecision must be frozen")


# --------------------------------------------------------------------------- #
# 3. the parking geometry
# --------------------------------------------------------------------------- #

SAFE_POINT = (6.0, 3.0)          # far from every protected rectangle
IN_GAP = (0.0, 0.0)              # dead centre of the capacity-1 corridor
ON_PAD = (-2.5, -1.4)            # the exit pad centre


def test_no_legal_point_is_a_refusal_not_an_approxation():
    """If every candidate is illegal, the answer is "no". Returning the least-bad one is
    how a robot ends up parked in a capacity-1 resource, which is worse than the drive it
    replaced: the whole fleet queues behind it."""
    r = legal_park_point((0.0, 0.0, 0.0), [IN_GAP, ON_PAD])
    assert r["ok"] is False
    assert r["park_at"] is None
    assert r["legal_count"] == 0
    assert "capacity-1" in r["reason"]


def test_the_corridor_centres_are_protected():
    """Not a guess: these are the rectangles from `config/resources.yaml`. Asserted here
    so that changing the map without changing the policy fails a test."""
    assert PROTECTED_RECTS["gap"] == (-1.75, -0.65, 1.75, 0.65)
    assert PROTECTED_RECTS["pad_left"] == (-2.85, -1.75, -2.15, -1.05)
    assert PROTECTED_RECTS["pad_right"] == (2.15, -1.75, 2.85, -1.05)


def test_the_footprint_plus_margin_is_what_is_cleared():
    """A point just outside a rectangle is still illegal if the robot's body overlaps it.
    The gap is 1.20 m tall (y in [-0.60, 0.60]); a 0.30 m radius plus 0.15 m margin means
    a robot centred at y = 1.00 does not clear it (1.00 - 0.45 = 0.55 < 0.65)."""
    just_outside = (0.0, 1.00)
    r = legal_park_point((0.0, 3.0, 0.0), [just_outside], footprint_radius_m=0.30,
                         margin_m=0.15)
    assert r["ok"] is False, "0.45 m of clearance does not clear a 0.65 m half-height"

    far_enough = (0.0, 1.20)
    r2 = legal_park_point((0.0, 3.0, 0.0), [far_enough], footprint_radius_m=0.30,
                          margin_m=0.15)
    assert r2["ok"] is True


def test_a_smaller_robot_may_park_closer():
    """Proving the clearance is computed and not a hardcoded distance."""
    tight = (0.0, 0.90)
    big = legal_park_point((0.0, 3.0, 0.0), [tight], footprint_radius_m=0.30, margin_m=0.15)
    small = legal_park_point((0.0, 3.0, 0.0), [tight], footprint_radius_m=0.10,
                             margin_m=0.15)
    assert big["ok"] is False
    assert small["ok"] is True


def test_nearest_legal_point_wins_because_the_battery_is_the_constraint():
    near = (2.0, 3.0)
    far = (6.0, 3.0)
    r = legal_park_point((0.0, 3.0, 0.0), [far, near])
    assert r["ok"] is True
    assert r["park_at"] == near
    assert r["distance_m"] == 2.0


def test_an_unreachable_legal_point_is_refused_rather_than_promised():
    """The same discipline as `no_safe_charger`: a robot that cannot prove it can get
    there must not be told to drive there. Otherwise the promise is broken silently and
    the robot strands itself mid-drive."""
    legal_but_far = (6.0, 3.0)
    reachable = [(2.0, 3.0)]  # not in the candidate list at all -> filtered empty
    r = legal_park_point((0.0, 3.0, 0.0), [legal_but_far], reachable=reachable)
    assert r["ok"] is False
    assert "reachable" in r["reason"]


def test_reachability_only_filters_and_never_rescues():
    """A reachable-but-illegal point must not be chosen. Filtering the other way round
    is the bug: legality first, then reachability."""
    r = legal_park_point((0.0, 3.0, 0.0), [IN_GAP], reachable=[IN_GAP])
    assert r["ok"] is False


def test_the_choice_is_reported_with_its_reason():
    r = legal_park_point((0.0, 3.0, 0.0), [SAFE_POINT])
    assert r["ok"] is True
    assert "legal" in r["reason"]
    assert r["legal_count"] == 1


def test_a_point_inside_a_rect_is_illegal_not_merely_close():
    """Distance-to-rectangle is 0 inside, and 0 < need, so it must fail. An implementation
    that measured centre-to-centre would wrongly pass a point in the middle of the gap."""
    r = legal_park_point((0.0, 0.0, 0.0), [(0.0, 0.60)], footprint_radius_m=0.30,
                         margin_m=0.15)
    assert r["ok"] is False, "the gap's top edge is inside the gap"


# --------------------------------------------------------------------------- #
# 4. custody
# --------------------------------------------------------------------------- #


def test_ownership_preserved_when_the_same_robot_still_holds_it():
    r = ownership_preserved("r01", "r01")
    assert r["preserved"] is True


def test_a_re_homed_cargo_is_a_policy_failure_not_a_delivery():
    """F05's expected result says the cargo's ownership is kept. A check that only asked
    "was it delivered" passes here -- that is the failure the case exists to catch, and
    `D-P17-03` records that the runner could not decide this clause before."""
    r = ownership_preserved("r01", "r02")
    assert r["preserved"] is False
    assert "policy failure" in r["detail"]


def test_losing_the_cargo_is_not_preserved():
    r = ownership_preserved("r01", None)
    assert r["preserved"] is False


def test_no_cargo_at_all_is_not_preserved():
    """None -> None must not read as "preserved". A robot that was never loaded has no
    ownership to preserve, and calling that a pass would make the check vacuous on every
    case that does not carry anything."""
    r = ownership_preserved(None, None)
    assert r["preserved"] is False


# --------------------------------------------------------------------------- #
# 5. the wiring contract
# --------------------------------------------------------------------------- #


def test_the_node_no_longer_skips_executing_robots_before_reading_the_band():
    """The one-line change that makes the whole rule reachable.

    Checked as source text because the fault is a guard, and a guard is not observable
    from the policy module. `D-P17-02` names this line.
    """
    node = (
        pathlib.Path(__file__).resolve().parent.parent
        / "src/fleet_ros/fleet_ros/task_service_node.py"
    ).read_text(encoding="utf-8")

    assert "if state.executing or not self._fresh(rid, now):" not in node, (
        "the combined skip is back: an executing robot is never considered"
    )
    assert "self._service_loaded_battery(" in node
    assert "decide_loaded_battery(" in node
    # the loaded branch must be reached FROM the executing guard, not after it
    i = node.find("if state.executing:")
    j = node.find("self._service_loaded_battery(rid, state, band, now)")
    k = node.find("if self._has_charge_task(rid):")
    assert 0 < i < j < k if k > 0 else 0 < i < j, (
        "the loaded half must be called inside the executing guard and before the "
        "unloaded charging path"
    )
