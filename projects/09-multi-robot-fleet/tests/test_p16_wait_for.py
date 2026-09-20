"""The wait vocabulary: does a condition hold, and should the runner even keep waiting?

These are the tests for the thing that lets a case say "r01 asks first". The three properties
that matter are not "it returns True sometimes":

  * a condition that is true NOW holds, and one that is only true LATER does not;
  * a task that already reached a TERMINAL state without executing makes the condition
    IMPOSSIBLE, not slow -- otherwise a definite failure is reported as a timeout;
  * an unknown key is refused. A condition that silently never fires would make the case test
    nothing while its report says it ran.

The third one is a check on the CHECKER, and it is here because round 15's lesson was that an
assertion narrower than reality passes while the code is broken. So the refusals are tested by
feeding this module input that must be refused, not by reading its source.
"""

from __future__ import annotations

import json
import sys
from pathlib import Path

import pytest

SRC = Path(__file__).resolve().parents[1] / "src"
for _pkg in ("fleet_core",):
    _p = SRC / _pkg
    if str(_p) not in sys.path:
        sys.path.insert(0, str(_p))

from fleet_core import wait_for  # noqa: E402


def _snapshot(*, tasks=(), crossings=()):
    """Only the two fields the vocabulary reads, so a test says what it depends on."""
    return {
        "tasks": [dict(t) for t in tasks],
        "crossings": {rid: {} for rid in crossings},
    }


# --------------------------------------------------------------------------- #
# the task-state condition
# --------------------------------------------------------------------------- #


def test_a_task_at_executing_satisfies_a_bare_request_id():
    snap = _snapshot(tasks=[{"request_id": "r", "state": "EXECUTING"}])
    verdict = wait_for.condition_holds(snap, {"request_id": "r"})
    assert verdict.holds, verdict.detail


def test_a_later_progress_state_also_satisfies_an_earlier_one():
    """`ASSIGNED` is past `ACCEPTED`; a condition must not demand an exact instant."""
    snap = _snapshot(tasks=[{"request_id": "r", "state": "EXECUTING"}])
    assert wait_for.condition_holds(snap, {"request_id": "r", "state": "ACCEPTED"}).holds


def test_a_task_that_has_not_arrived_yet_is_not_yet_true_but_still_possible():
    snap = _snapshot(tasks=[{"request_id": "r", "state": "ACCEPTED"}])
    verdict = wait_for.condition_holds(snap, {"request_id": "r", "state": "EXECUTING"})
    assert not verdict.holds
    assert not verdict.impossible, "waiting is the right response to a task that is on its way"
    assert "ACCEPTED" in verdict.detail and "EXECUTING" in verdict.detail


def test_a_task_missing_from_the_ledger_is_not_yet_true():
    verdict = wait_for.condition_holds(_snapshot(), {"request_id": "r"})
    assert not verdict.holds
    assert not verdict.impossible


def test_a_failed_task_makes_the_condition_impossible_not_slow():
    """The distinction the module exists for.

    A task that went ACCEPTED -> FAILED never executed. A runner that keeps waiting reports a
    timeout, and the timeout says nothing about the refusal that actually happened.
    """
    snap = _snapshot(tasks=[{"request_id": "r", "state": "FAILED"}])
    verdict = wait_for.condition_holds(snap, {"request_id": "r", "state": "EXECUTING"})
    assert not verdict.holds
    assert verdict.impossible
    assert "never reached" in verdict.detail


def test_a_terminal_state_is_not_treated_as_further_along_than_executing():
    """FAILED must not satisfy `state: EXECUTING` just because it comes later in a ranking."""
    snap = _snapshot(tasks=[{"request_id": "r", "state": "FAILED"}])
    assert not wait_for.condition_holds(snap, {"request_id": "r"}).holds


def test_a_terminal_state_can_be_waited_for_explicitly():
    snap = _snapshot(tasks=[{"request_id": "r", "state": "SUCCEEDED"}])
    assert wait_for.condition_holds(snap, {"request_id": "r", "state": "SUCCEEDED"}).holds


# --------------------------------------------------------------------------- #
# the corridor condition
# --------------------------------------------------------------------------- #


def test_a_robot_inside_the_passage_satisfies_the_corridor_condition():
    snap = _snapshot(crossings=["r01"])
    assert wait_for.condition_holds(snap, {"robot_crossing": "r01"}).holds


def test_a_robot_that_has_left_the_passage_does_not():
    snap = _snapshot(crossings=["r02"])
    verdict = wait_for.condition_holds(snap, {"robot_crossing": "r01"})
    assert not verdict.holds
    assert not verdict.impossible, "r01 may enter later; that is what the case is arranging"


def test_two_keys_must_both_hold():
    both = _snapshot(tasks=[{"request_id": "r", "state": "EXECUTING"}], crossings=["r01"])
    assert wait_for.condition_holds(both, {"request_id": "r", "robot_crossing": "r01"}).holds

    no_corridor = _snapshot(tasks=[{"request_id": "r", "state": "EXECUTING"}], crossings=[])
    assert not wait_for.condition_holds(
        no_corridor, {"request_id": "r", "robot_crossing": "r01"}).holds

    no_task = _snapshot(tasks=[], crossings=["r01"])
    assert not wait_for.condition_holds(
        no_task, {"request_id": "r", "robot_crossing": "r01"}).holds


# --------------------------------------------------------------------------- #
# checks on the checker
# --------------------------------------------------------------------------- #


def test_an_unknown_key_is_refused_rather_than_ignored():
    """A typo must be loud. `{robot_crossings: r01}` (plural) would otherwise never fire."""
    verdict = wait_for.condition_holds(_snapshot(), {"robot_crossings": "r01"})
    assert not verdict.holds
    assert verdict.impossible
    assert "unknown wait key" in verdict.detail


def test_a_condition_that_is_not_a_mapping_is_refused():
    verdict = wait_for.condition_holds(_snapshot(), ["r01"])
    assert not verdict.holds and verdict.impossible


def test_an_empty_condition_holds_and_says_it_is_empty():
    """Not an error -- but it must not read as if something was checked."""
    verdict = wait_for.condition_holds(_snapshot(), {})
    assert verdict.holds
    assert verdict.detail == "an empty condition"


def test_the_vocabulary_is_closed():
    #: Extended in round 17 on purpose: `robot_carrying` is what F05 (low battery while
    #: carrying) and F08 (a robot carrying a payload goes offline) trigger on. The assertion
    #: stays an equality rather than becoming a superset check, because the property worth
    #: keeping is that a new key has to be written down here as well as in the module.
    assert wait_for.KNOWN_KEYS == frozenset(
        {"request_id", "state", "robot_crossing", "robot_carrying"})


def test_every_known_key_actually_changes_a_verdict():
    """A key in KNOWN_KEYS that no clause reads would be advertised and then ignored.

    Behavioural, not a source search: this project has already shipped an assertion that
    matched the spelling of a name in a docstring while the code did nothing.
    """
    empty = _snapshot()
    assert not wait_for.condition_holds(empty, {"request_id": "r"}).holds
    assert not wait_for.condition_holds(empty, {"robot_crossing": "r01"}).holds
    assert not wait_for.condition_holds(empty, {"robot_carrying": "r01"}).holds
    verdict = wait_for.condition_holds(empty, {"state": "EXECUTING"})
    assert not verdict.holds, "`state` without a subject must not fall through and hold"
    assert verdict.impossible


def test_progress_and_terminal_do_not_overlap():
    assert not (set(wait_for.PROGRESS) & set(wait_for.TERMINAL))
    assert len(wait_for.PROGRESS) == 4 and len(wait_for.TERMINAL) == 4


@pytest.mark.parametrize("state", ["SUCCEEDED", "CANCELED", "NEEDS_ATTENTION"])
def test_every_terminal_state_blocks_a_progress_wait(state):
    snap = _snapshot(tasks=[{"request_id": "r", "state": state}])
    verdict = wait_for.condition_holds(snap, {"request_id": "r", "state": "EXECUTING"})
    assert not verdict.holds and verdict.impossible, (
        f"{state} must be reported as impossible, not as something to keep waiting for")


def test_the_verdict_survives_a_json_round_trip_because_the_runner_records_it():
    verdict = wait_for.condition_holds(_snapshot(), {"request_id": "r"})
    row = {"holds": verdict.holds, "impossible": verdict.impossible, "detail": verdict.detail}
    assert json.loads(json.dumps(row)) == row
