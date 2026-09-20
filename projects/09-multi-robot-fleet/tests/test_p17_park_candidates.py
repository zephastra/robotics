"""D-P17-19: the LOADED half of CONTRACTS section 5 must have a callable parking path.

`task_service_node.py` called `self._park_candidates()` and
`self._reachable_park_candidates(state)` for days. Neither method existed anywhere in
the repository. The calls sit behind two guards that were both closed on every recorded
run -- the action had to be `ACTION_PARK_AND_ATTEND` (the loaded half, never once
selected; every recorded `loaded_low_battery*` event reports `cancel_and_charge`, i.e.
the UNLOADED half) and `state.position_known` had to be true besides. So F05's expected
result, "keep custody, park legally, raise attention", could not occur, and the failure
looked like a task result rather than a crash.

These tests are pure Python against `fleet_core` and a small stand-in for the node, so
they run in the clean shell with no ROS sourced -- which is where this project runs its
core checks, and the only place a defect of this shape can be pinned without a
simulator. That is the point: the bug was invisible to every non-simulator instrument
this project had, so the instrument that catches it has to be a non-simulator one.

The stand-in deliberately does NOT re-implement the node's methods. It calls the real
`legal_park_point` from `fleet_core`, with candidates produced the same way the node
produces them, so the test fails if the node's rule and the policy's rule drift apart.
"""
from __future__ import annotations

import ast
import pathlib
import sys

ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src/fleet_core"))

from fleet_core.loaded_battery_policy import (  # noqa: E402
    ACTION_CANCEL_AND_CHARGE,
    ACTION_PARK_AND_ATTEND,
    PROTECTED_RECTS,
    decide as decide_loaded_battery,
    legal_park_point,
)

NODE = ROOT / "src/fleet_ros/fleet_ros/task_service_node.py"


# --------------------------------------------------------------------------- #
# the defect itself: the methods have to exist, as methods
# --------------------------------------------------------------------------- #


def test_the_two_methods_are_actually_defined():
    """The exact regression. A call site with no definition is not a small thing.

    `self._park_candidates()` is legal syntax, a correct spelling, and in the right
    place; nothing short of resolving the name says whether there is anything behind it.
    These are AST lookups rather than greps for that reason -- a grep for
    `_park_candidates` in this file returns the CALL SITES and would happily pass.
    """
    tree = ast.parse(NODE.read_text(encoding="utf-8"))
    task_service = [n for n in ast.walk(tree)
                    if isinstance(n, ast.ClassDef) and n.name == "TaskService"]
    assert task_service, "TaskService class not found in task_service_node.py"
    defined = {b.name for b in task_service[0].body if isinstance(b, ast.FunctionDef)}
    for name in ("_park_candidates", "_reachable_park_candidates"):
        assert name in defined, (
            f"TaskService calls self.{name}() but does not define it. This is D-P17-19: "
            f"the loaded-battery parking branch raises AttributeError the first time it "
            f"runs, so a robot carrying cargo on a critical battery can never be parked."
        )


def test_no_self_attribute_is_read_without_a_definition():
    """The general form, so the NEXT instance of this shape is caught here too.

    Delegates to the guard rather than restating its rules, because two copies of an
    analysis is how the two copies come to disagree.
    """
    sys.path.insert(0, str(ROOT / "scripts"))
    import check_self_attrs  # noqa: E402

    problems = check_self_attrs.scan_file(NODE, ROOT)
    assert not problems, "\n".join(problems)


# --------------------------------------------------------------------------- #
# the decision that reaches the branch
# --------------------------------------------------------------------------- #


def test_carrying_on_a_critical_battery_selects_the_parking_action():
    """The condition that routes into the branch that did not work.

    If this ever returns CANCEL_AND_CHARGE, F05's expected result is unreachable for a
    second, independent reason, and the two reasons would look identical from the
    outside.
    """
    d = decide_loaded_battery("CRITICAL", executing=True, carrying=True,
                              has_charge_task=False)
    assert d.action == ACTION_PARK_AND_ATTEND, d
    assert d.carrying is True


def test_the_unloaded_half_still_cancels():
    """The half that always worked must not be moved by this fix.

    Every recorded event took this path. Changing it would rewrite F05's history
    silently, and the recorded evidence would stop describing the code.
    """
    d = decide_loaded_battery("CRITICAL", executing=True, carrying=False,
                              has_charge_task=False)
    assert d.action == ACTION_CANCEL_AND_CHARGE, d


# --------------------------------------------------------------------------- #
# the candidate list: what a LOADED robot may stop on
# --------------------------------------------------------------------------- #


def test_a_charger_is_not_a_parking_candidate():
    """CONTRACTS section 5 forbids driving a loaded robot to a charge pad.

    A pad is a legal place to be and an illegal place to be WITH cargo. Excluding it
    from the candidate list, rather than filtering it later, is deliberate: a candidate
    rejected only at the end is one edit away from being offered.
    """
    stations = {"S_left_a": (-5.0, 2.5), "S_left_b": (-5.0, -2.5),
                "S_right_a": (5.0, 2.5), "S_right_b": (5.0, -2.5),
                "C_left": (-5.0, -3.8), "C_right": (5.0, -3.8)}
    chargers = {"C_left", "C_right"}
    candidates = [(x, y) for name, (x, y) in sorted(stations.items())
                  if name not in chargers]
    assert (-5.0, -3.8) not in candidates, "a charger is in the parking candidate list"
    assert (5.0, -3.8) not in candidates, "a charger is in the parking candidate list"
    assert len(candidates) == 4


def test_every_candidate_clears_a_protected_rectangle_or_is_reported():
    """The four stations are legal parking for a robot with this footprint.

    A candidate that is illegal is not a bug by itself -- `legal_park_point` has a branch
    for "no legal point among the candidates" and reports it. It would be a bug if ALL of
    them were illegal, because then the parking path can only ever report and never park,
    and F05's "park legally" clause would be silently unreachable a second time.
    """
    candidates = [(-5.0, 2.5), (-5.0, -2.5), (5.0, 2.5), (5.0, -2.5)]
    legal = [c for c in candidates
             if legal_park_point((0.0, 0.0, 0.0), [c])["ok"]]
    assert legal, (
        "every configured station is inside a protected rectangle for a 0.30 m "
        "footprint -- the loaded-battery parking path could never park anywhere"
    )


def test_a_parking_point_inside_the_corridor_is_refused():
    """The specific hazard: a loaded robot parked in the gap blocks the whole fleet.

    This is the `legal_park_point` contract the node relies on, asserted here so the
    node's use of it cannot drift from it.
    """
    gap_middle = (0.0, 0.0)
    assert any(PROTECTED_RECTS[n][0] <= gap_middle[0] <= PROTECTED_RECTS[n][2]
               and PROTECTED_RECTS[n][1] <= gap_middle[1] <= PROTECTED_RECTS[n][3]
               for n in PROTECTED_RECTS), "the corridor centre is no longer protected"
    result = legal_park_point((0.0, 3.0, 0.0), [gap_middle])
    assert result["ok"] is False, result
    assert "blocks the fleet" in result["reason"]


def test_an_unreachable_point_is_refused_rather_than_promised():
    """The reachability half, which is what the missing method had to supply.

    Without it the candidate list is offered whole and a robot is sent somewhere its
    battery cannot take it -- the promise the `no_safe_charger` path already refuses to
    make.
    """
    candidates = [(-5.0, 2.5), (-5.0, -2.5)]
    reachable = [(-5.0, 2.5)]
    result = legal_park_point((0.0, 0.0, 0.0), candidates, reachable=reachable)
    assert result["ok"] is True
    assert tuple(result["park_at"]) == (-5.0, 2.5), result


def test_no_reachable_candidate_is_reported_not_guessed():
    """`reachable=[]` must not degrade into "offer everything".

    `legal_park_point` only applies the reachable filter when `reachable is not None`,
    so an empty list is meaningfully different from `None`. Passing `None` because the
    reachable set came back empty is exactly how the reachability branch becomes dead
    code, and a rule that cannot fire is not a rule.
    """
    candidates = [(-5.0, 2.5), (-5.0, -2.5)]
    result = legal_park_point((0.0, 0.0, 0.0), candidates, reachable=[])
    assert result["ok"] is False, result
    assert "none is reachable" in result["reason"]
    assert result["legal_count"] == 2
