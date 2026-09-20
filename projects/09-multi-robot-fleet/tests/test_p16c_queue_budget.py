"""A permit queue must not consume the driver's movement budget (D-P16-03).

N04 measured the defect from the driver's own report:

    stage_wait    38.924 s   reached wait_east
    stage_align   26.570 s   reached align_east
    acquire       42.868 s   granted east_to_west after 86 request(s)
    stage_cross   51.653 s   reached exit_west
    stage_release  2.620 s   FAILED -- "reached this driver's own budget"

42.9 s of a 165 s budget spent queued, and the robot ran out with its permit in hand. The
crossing protocol had behaved correctly; the budget was wrong.

The property that matters most here is the NEGATIVE one, and it is round 14's: **whatever the
queue, the driver must still stop before the dispatcher's kill**, because a killed driver leaves
its Nav2 goal behind and the next driver collides with it. So the queue credit is capped twice,
and the cap is asserted rather than the credit.
"""

from __future__ import annotations

import sys
from pathlib import Path

import pytest

SRC = Path(__file__).resolve().parents[1] / "src"
for _pkg in ("fleet_core",):
    _p = SRC / _pkg
    if str(_p) not in sys.path:
        sys.path.insert(0, str(_p))

from fleet_core.crossing_release import (  # noqa: E402
    DEFAULT_QUEUE_ALLOWANCE_S,
    ReleasePolicy,
    stop_is_early_enough,
)

#: N04's own numbers, from the driver report in reports/batch_20260917T191716Z.
N04_TIMEOUT_S = 300.0
N04_ALLOWANCE_S = 90.0
N04_QUEUED_S = 42.868


def _policy(**kw):
    return ReleasePolicy(**kw)


# --------------------------------------------------------------------------- #
# the default changes nothing
# --------------------------------------------------------------------------- #


def test_the_default_allowance_is_zero():
    assert DEFAULT_QUEUE_ALLOWANCE_S == 0.0
    assert ReleasePolicy().queue_allowance_s == 0.0


def test_with_no_allowance_the_deadline_is_where_it_always_was():
    """The seventeen round-14 tests keep their meaning, because this is the same number."""
    policy = _policy()
    started = 1000.0
    assert policy.run_deadline(started, 180.0) == policy.hard_deadline(started, 180.0)
    assert policy.run_deadline(started, 180.0) == started + 180.0 - 15.0


def test_a_zero_queue_credits_nothing_even_when_an_allowance_exists():
    policy = _policy(queue_allowance_s=90.0)
    deadline = policy.run_deadline(0.0, 300.0)
    assert policy.after_queue(deadline, 0.0, 0.0, 300.0) == deadline


# --------------------------------------------------------------------------- #
# the credit
# --------------------------------------------------------------------------- #


def test_the_allowance_is_held_back_so_it_can_be_given_back():
    """A driver that never queues must not get the allowance; it is reserved, not granted."""
    policy = _policy(queue_allowance_s=90.0)
    hard = policy.hard_deadline(0.0, 300.0)
    assert hard == 285.0
    assert policy.run_deadline(0.0, 300.0) == 195.0


def test_the_measured_queue_is_given_back():
    policy = _policy(queue_allowance_s=N04_ALLOWANCE_S)
    deadline = policy.run_deadline(0.0, N04_TIMEOUT_S)
    assert policy.after_queue(deadline, N04_QUEUED_S, 0.0, N04_TIMEOUT_S) == \
        pytest.approx(195.0 + N04_QUEUED_S)


def test_the_measured_work_fits_after_the_credit():
    """The arithmetic N04 needs: 38.9 + 26.6 + 51.7 + 23.4 of driving, plus the queue."""
    policy = _policy(queue_allowance_s=N04_ALLOWANCE_S)
    deadline = policy.run_deadline(0.0, N04_TIMEOUT_S)
    deadline = policy.after_queue(deadline, N04_QUEUED_S, 0.0, N04_TIMEOUT_S)
    driving_s = 38.924 + 26.570 + 51.653 + 23.389
    assert driving_s < deadline, "r02 must reach its release node inside the budget"


def test_a_queue_longer_than_the_allowance_is_capped():
    policy = _policy(queue_allowance_s=90.0)
    deadline = policy.run_deadline(0.0, 300.0)
    assert policy.after_queue(deadline, 5000.0, 0.0, 300.0) == 285.0


# --------------------------------------------------------------------------- #
# the property round 14 fought for: still inside the dispatcher's kill
# --------------------------------------------------------------------------- #


@pytest.mark.parametrize("queued_s", [0.0, 1.0, 42.868, 90.0, 120.0, 1e6])
@pytest.mark.parametrize("allowance_s", [0.0, 30.0, 90.0, 600.0])
def test_whatever_the_queue_the_driver_still_stops_before_the_dispatcher(
        queued_s, allowance_s):
    """The invariant whose violation costs the NEXT driver a refused goal.

    Round 14: the dispatcher killed a driver, the kill did not cancel the Nav2 goal, and the
    retry lost the race by 0.097 s. No amount of queueing may reopen that.
    """
    policy = _policy(queue_allowance_s=allowance_s)
    started = 1234.0
    deadline = policy.run_deadline(started, 300.0)
    deadline = policy.after_queue(deadline, queued_s, started, 300.0)
    assert stop_is_early_enough(deadline, started + 300.0)


@pytest.mark.parametrize("allowance_s", [0.0, 90.0, 600.0])
def test_the_hard_deadline_does_not_move_with_the_allowance(allowance_s):
    """The margin belongs to the dispatcher. An allowance that could move the hard deadline
    would be a scenario able to talk its way past its own kill."""
    assert _policy(queue_allowance_s=allowance_s).hard_deadline(0.0, 300.0) == 285.0


def test_a_deadline_is_never_moved_backwards_by_a_negative_queue():
    policy = _policy(queue_allowance_s=90.0)
    deadline = policy.run_deadline(0.0, 300.0)
    assert policy.after_queue(deadline, -100.0, 0.0, 300.0) == deadline


def test_a_tiny_budget_cannot_produce_a_deadline_in_the_past():
    """The margin is capped at half the budget, and the allowance must not eat the rest."""
    policy = _policy(queue_allowance_s=1000.0)
    started = 500.0
    deadline = policy.run_deadline(started, 10.0)
    assert deadline >= started
    assert stop_is_early_enough(deadline, started + 10.0)


def test_the_allowance_can_be_zero_even_with_a_queue_and_nothing_changes():
    """n01/n02/n03 declare no `crossing_queue_s`, so they queue for free as before."""
    policy = _policy()
    started = 0.0
    deadline = policy.run_deadline(started, 180.0)
    assert policy.after_queue(deadline, 42.868, started, 180.0) == deadline
