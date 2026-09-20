"""L1 §4, §5, §11: task state machine, two-phase cancel, stale commands.

Maps to TEST_AND_ACCEPTANCE L1:
  * 4  stale revision / boot_id / epoch rejected
  * 5  cancel accepted != stopped; a late success must not win
  * 11 low-battery prediction and held-low-battery -> attention
"""

from __future__ import annotations

import pytest

from fleet_core import Allocator, ReasonCode, TaskState
from fleet_core.domain import RobotState, StaleCommandError
from fleet_core.task_machine import IllegalTransition


# --------------------------------------------------------------------------- #
# happy path shows accepted is not success
# --------------------------------------------------------------------------- #


def test_accepted_is_not_success(ledger, machine, spec_factory):
    task, _ = ledger.submit(spec_factory("req-A"), now=1.0, epoch=0)
    after = machine.accept(task.task_id, now=2.0)
    assert after.state is TaskState.ACCEPTED
    assert after.state is not TaskState.SUCCEEDED


def test_full_lifecycle_requires_each_step(ledger, machine, spec_factory):
    task, _ = ledger.submit(spec_factory("req-A"), now=1.0, epoch=0)
    machine.accept(task.task_id, now=2.0)
    machine.assign(task.task_id, "r01", now=3.0)
    machine.begin_execution(task.task_id, "r01", now=4.0)
    done = machine.succeed(task.task_id, now=5.0)
    assert done.state is TaskState.SUCCEEDED


def test_terminal_states_cannot_be_revived(ledger, machine, spec_factory):
    task, _ = ledger.submit(spec_factory("req-A"), now=1.0, epoch=0)
    machine.accept(task.task_id, now=2.0)
    machine.fail(task.task_id, now=3.0, reason=ReasonCode.BLOCKED)
    with pytest.raises(IllegalTransition):
        machine.accept(task.task_id, now=4.0)


def test_cannot_execute_without_assignment(ledger, machine, spec_factory):
    task, _ = ledger.submit(spec_factory("req-A"), now=1.0, epoch=0)
    machine.accept(task.task_id, now=2.0)
    with pytest.raises(IllegalTransition):
        machine.begin_execution(task.task_id, "r01", now=3.0)


# --------------------------------------------------------------------------- #
# §4 stale commands
# --------------------------------------------------------------------------- #


def test_stale_generation_success_is_refused(ledger, machine, spec_factory):
    task, _ = ledger.submit(spec_factory("req-A"), now=1.0, epoch=0)
    machine.accept(task.task_id, now=2.0)
    machine.assign(task.task_id, "r01", now=3.0)
    machine.begin_execution(task.task_id, "r01", now=4.0)
    with pytest.raises(StaleCommandError):
        machine.succeed(task.task_id, now=5.0, generation_ok=False)
    assert ledger.get(task.task_id).state is TaskState.EXECUTING


def test_epoch_mismatch_is_stale(ledger, spec_factory):
    from fleet_core import TaskMachine

    task, _ = ledger.submit(spec_factory("req-A"), now=1.0, epoch=0)
    other_epoch = TaskMachine(ledger, epoch=7)
    with pytest.raises(StaleCommandError):
        other_epoch.accept(task.task_id, now=2.0)


def test_wrong_owner_is_stale(ledger, machine, spec_factory):
    task, _ = ledger.submit(spec_factory("req-A"), now=1.0, epoch=0)
    machine.accept(task.task_id, now=2.0)
    machine.assign(task.task_id, "r01", now=3.0)
    with pytest.raises(StaleCommandError):
        machine.transition(
            task.task_id, TaskState.EXECUTING, now=4.0, require_robot="r02"
        )


def test_revision_increases_on_every_write(ledger, machine, spec_factory):
    task, _ = ledger.submit(spec_factory("req-A"), now=1.0, epoch=0)
    r0 = ledger.get(task.task_id).revision
    machine.accept(task.task_id, now=2.0)
    r1 = ledger.get(task.task_id).revision
    machine.assign(task.task_id, "r01", now=3.0)
    r2 = ledger.get(task.task_id).revision
    assert r0 < r1 < r2


# --------------------------------------------------------------------------- #
# §5 two-phase cancel
# --------------------------------------------------------------------------- #


def test_cancel_accepted_is_not_stopped(ledger, machine, spec_factory):
    task, _ = ledger.submit(spec_factory("req-A"), now=1.0, epoch=0)
    machine.accept(task.task_id, now=2.0)
    machine.assign(task.task_id, "r01", now=3.0)
    machine.begin_execution(task.task_id, "r01", now=4.0)

    res = machine.cancel_requested(task.task_id, now=5.0)
    assert res.accepted is True
    assert res.stopped is False, "acknowledging a cancel is not proof of stopping"
    assert ledger.get(task.task_id).state is TaskState.EXECUTING


def test_cancel_only_completes_after_confirmation(ledger, machine, spec_factory):
    task, _ = ledger.submit(spec_factory("req-A"), now=1.0, epoch=0)
    machine.accept(task.task_id, now=2.0)
    machine.assign(task.task_id, "r01", now=3.0)
    machine.begin_execution(task.task_id, "r01", now=4.0)

    machine.cancel_requested(task.task_id, now=5.0)
    done = machine.cancel_confirmed(task.task_id, now=6.0)
    assert done.stopped is True
    assert ledger.get(task.task_id).state is TaskState.CANCELED


def test_late_success_after_cancel_does_not_win(ledger, machine, spec_factory):
    """I03: a stale success callback must not complete a cancelled task."""
    task, _ = ledger.submit(spec_factory("req-A"), now=1.0, epoch=0)
    machine.accept(task.task_id, now=2.0)
    machine.assign(task.task_id, "r01", now=3.0)
    machine.begin_execution(task.task_id, "r01", now=4.0)
    machine.cancel_requested(task.task_id, now=5.0)

    result = machine.succeed(task.task_id, now=6.0)
    assert result.state is TaskState.CANCELED
    assert result.reason is ReasonCode.CANCEL_NOT_CONFIRMED


def test_confirm_without_request_is_rejected(ledger, machine, spec_factory):
    task, _ = ledger.submit(spec_factory("req-A"), now=1.0, epoch=0)
    machine.accept(task.task_id, now=2.0)
    res = machine.cancel_confirmed(task.task_id, now=3.0)
    assert res.accepted is False
    assert ledger.get(task.task_id).state is TaskState.ACCEPTED


def test_cancel_terminal_task_is_not_accepted(ledger, machine, spec_factory):
    task, _ = ledger.submit(spec_factory("req-A"), now=1.0, epoch=0)
    machine.accept(task.task_id, now=2.0)
    machine.fail(task.task_id, now=3.0, reason=ReasonCode.BLOCKED)
    res = machine.cancel_requested(task.task_id, now=4.0)
    assert res.accepted is False
    assert res.stopped is True


def test_task_awaiting_cancel_confirmation_cannot_be_taken_over(ledger, machine, spec_factory):
    task, _ = ledger.submit(spec_factory("req-A"), now=1.0, epoch=0)
    machine.accept(task.task_id, now=2.0)
    machine.assign(task.task_id, "r01", now=3.0)
    machine.begin_execution(task.task_id, "r01", now=4.0)
    machine.cancel_requested(task.task_id, now=5.0)

    allowed, reason = machine.takeover_allowed(task.task_id)
    assert allowed is False
    assert reason is ReasonCode.CANCEL_NOT_CONFIRMED


# --------------------------------------------------------------------------- #
# §9 / §11 payload-aware takeover and low battery
# --------------------------------------------------------------------------- #


def test_task_holding_a_payload_cannot_be_taken_over(ledger, machine, spec_factory):
    task, _ = ledger.submit(spec_factory("req-A", payload_id="p1"), now=1.0, epoch=0)
    machine.accept(task.task_id, now=2.0)
    machine.assign(task.task_id, "r01", now=3.0)
    machine.begin_execution(task.task_id, "r01", now=4.0)
    ledger.pick("p1", task_id=task.task_id, robot_id="r01")

    allowed, reason = machine.takeover_allowed(task.task_id)
    assert allowed is False
    assert reason is ReasonCode.PAYLOAD_HELD_ELSEWHERE


def test_task_before_pick_can_be_taken_over(ledger, machine, spec_factory):
    """F07: an adapter death before pick-up is recoverable."""
    task, _ = ledger.submit(spec_factory("req-A", payload_id="p1"), now=1.0, epoch=0)
    machine.accept(task.task_id, now=2.0)
    machine.assign(task.task_id, "r01", now=3.0)
    machine.begin_execution(task.task_id, "r01", now=4.0)

    allowed, reason = machine.takeover_allowed(task.task_id)
    assert allowed is True
    assert reason is ReasonCode.OK


def test_held_low_battery_becomes_attention_not_a_dropped_task(ledger, machine, cfg, spec_factory):
    """F05: holding the payload, the robot cannot simply go charge or be re-homed."""
    task, _ = ledger.submit(spec_factory("req-A", payload_id="p1"), now=1.0, epoch=0)
    machine.accept(task.task_id, now=2.0)
    machine.assign(task.task_id, "r01", now=3.0)
    machine.begin_execution(task.task_id, "r01", now=4.0)
    ledger.pick("p1", task_id=task.task_id, robot_id="r01")

    alloc = Allocator(cfg)
    robots = {"r01": RobotState("r01", battery_wh=1.0), "r02": RobotState("r02", battery_wh=90.0)}
    low = alloc.low_battery_robots(robots)
    assert low == ["r01"]

    # Low battery while holding: needs attention, and the payload stays put.
    after = machine.needs_attention(task.task_id, now=6.0, reason=ReasonCode.INSUFFICIENT_BATTERY)
    assert after.state is TaskState.NEEDS_ATTENTION
    assert ledger.payload_state("p1")[1] == "r01"


def test_low_battery_prediction_excludes_robot_that_cannot_finish(ledger, cfg, spec_factory):
    """A near robot with too little charge is not a candidate."""
    task, _ = ledger.submit(spec_factory("req-A"), now=1.0, epoch=0)
    alloc = Allocator(cfg)
    robots = {
        "r01": RobotState("r01", x=5.0, y=2.4, battery_wh=5.5),   # near, nearly empty
        "r02": RobotState("r02", x=-4.0, y=2.4, battery_wh=90.0),  # far, healthy
    }
    alloc_result = alloc.allocate(task, robots, now=1.0)
    assert alloc_result.robot_id == "r02"
    assert "r01" not in [r for r, _ in alloc_result.candidates]
