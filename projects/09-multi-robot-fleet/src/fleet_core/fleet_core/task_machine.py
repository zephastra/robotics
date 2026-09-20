"""Task state machine.

Transitions are the only way a task's state changes, and every transition is
guarded. The guards are the point:

  * `accepted` is not success. SUBMITTED -> ACCEPTED only acknowledges receipt.
  * `cancel accepted` is not `stopped`. A cancel request sets a flag; the task
    reaches CANCELED only once the adapter confirms the motion actually ended,
    so a robot can never be handed a new task while the old motion continues.
  * A completion that arrives with a stale generation belongs to an older command
    and must be ignored -- otherwise a late callback from a cancelled run marks a
    live task SUCCEEDED.
"""

from __future__ import annotations

from dataclasses import dataclass

from .domain import (
    CancelResult,
    ReasonCode,
    Task,
    TaskState,
    StaleCommandError,
)
from .ledger import Ledger

_ALLOWED: dict[TaskState, frozenset[TaskState]] = {
    TaskState.SUBMITTED: frozenset({TaskState.ACCEPTED, TaskState.CANCELED, TaskState.FAILED}),
    TaskState.ACCEPTED: frozenset(
        {TaskState.ASSIGNED, TaskState.CANCELED, TaskState.FAILED, TaskState.NEEDS_ATTENTION}
    ),
    TaskState.ASSIGNED: frozenset(
        {TaskState.EXECUTING, TaskState.CANCELED, TaskState.FAILED, TaskState.NEEDS_ATTENTION}
    ),
    TaskState.EXECUTING: frozenset(
        {
            TaskState.SUCCEEDED,
            TaskState.FAILED,
            TaskState.CANCELED,
            TaskState.NEEDS_ATTENTION,
            TaskState.ASSIGNED,
        }
    ),
    TaskState.NEEDS_ATTENTION: frozenset(
        {TaskState.ASSIGNED, TaskState.FAILED, TaskState.CANCELED}
    ),
    TaskState.SUCCEEDED: frozenset(),
    TaskState.FAILED: frozenset(),
    TaskState.CANCELED: frozenset(),
    TaskState.NOT_STARTED: frozenset({TaskState.SUBMITTED, TaskState.FAILED}),
}


class IllegalTransition(Exception):
    pass


@dataclass
class TaskMachine:
    ledger: Ledger
    epoch: int = 0

    # ------------------------------------------------------------------ #

    def transition(
        self,
        task_id: str,
        to: TaskState,
        *,
        now: float,
        reason: ReasonCode = ReasonCode.OK,
        detail: str = "",
        robot_id: str | None = None,
        require_robot: str | None = None,
        generation_ok: bool = True,
    ) -> Task:
        task = self.ledger.get(task_id)

        if not generation_ok:
            # A late callback from a superseded command. Ignore it; do not let it
            # mutate the live task.
            raise StaleCommandError(
                f"{task_id}: refusing {task.state.value}->{to.value} from a stale generation"
            )
        if task.epoch != self.epoch:
            raise StaleCommandError(
                f"{task_id}: epoch {task.epoch} != current {self.epoch}"
            )
        if to not in _ALLOWED[task.state]:
            raise IllegalTransition(
                f"{task_id}: {task.state.value} -> {to.value} is not allowed"
            )
        if require_robot is not None and task.robot_id != require_robot:
            raise StaleCommandError(
                f"{task_id}: owned by {task.robot_id!r}, not {require_robot!r}"
            )
        if task.cancel_requested and to is TaskState.SUCCEEDED:
            # Cancelled mid-flight: a late success must not win.
            return self.ledger.set_state(
                task_id,
                TaskState.CANCELED,
                now=now,
                reason=ReasonCode.CANCEL_NOT_CONFIRMED,
                detail="success arrived after cancel was requested",
            )

        return self.ledger.set_state(
            task_id,
            to,
            now=now,
            reason=reason,
            detail=detail,
            robot_id=robot_id,
        )

    # ------------------------------------------------------------------ #
    # lifecycle helpers
    # ------------------------------------------------------------------ #

    def accept(self, task_id: str, *, now: float) -> Task:
        return self.transition(task_id, TaskState.ACCEPTED, now=now)

    def assign(self, task_id: str, robot_id: str, *, now: float) -> Task:
        return self.transition(
            task_id, TaskState.ASSIGNED, now=now, robot_id=robot_id
        )

    def begin_execution(self, task_id: str, robot_id: str, *, now: float) -> Task:
        return self.transition(
            task_id, TaskState.EXECUTING, now=now, robot_id=robot_id
        )

    def succeed(self, task_id: str, *, now: float, generation_ok: bool = True) -> Task:
        return self.transition(
            task_id, TaskState.SUCCEEDED, now=now, generation_ok=generation_ok
        )

    def fail(self, task_id: str, *, now: float, reason: ReasonCode, detail: str = "") -> Task:
        return self.transition(task_id, TaskState.FAILED, now=now, reason=reason, detail=detail)

    def needs_attention(
        self, task_id: str, *, now: float, reason: ReasonCode, detail: str = ""
    ) -> Task:
        # `detail` because a reason code alone cannot say WHICH robot is still holding
        # WHICH payload, and that is the first question anyone asks of a parked task.
        return self.transition(
            task_id, TaskState.NEEDS_ATTENTION, now=now, reason=reason, detail=detail
        )

    # ------------------------------------------------------------------ #
    # cancellation -- two-phase on purpose
    # ------------------------------------------------------------------ #

    def cancel_requested(self, task_id: str, *, now: float) -> CancelResult:
        """Phase 1: acknowledge the request. NOT a claim that motion stopped."""
        task = self.ledger.get(task_id)
        if task.state.terminal:
            return CancelResult(
                accepted=False,
                reason=ReasonCode.OK,
                stopped=True,
                detail=f"already {task.state.value}",
            )
        self.ledger.request_cancel(task_id, now=now)
        return CancelResult(
            accepted=True,
            reason=ReasonCode.OK,
            stopped=False,
            detail="cancel acknowledged; motion not yet confirmed stopped",
        )

    def cancel_confirmed(self, task_id: str, *, now: float) -> CancelResult:
        """Phase 2: the adapter proved the motion ended. Only now is it CANCELED."""
        task = self.ledger.get(task_id)
        if not task.cancel_requested:
            return CancelResult(
                accepted=False,
                reason=ReasonCode.INVALID_INPUT,
                stopped=False,
                detail="no cancel was requested",
            )
        if task.state.terminal:
            return CancelResult(accepted=True, reason=ReasonCode.OK, stopped=True)
        self.transition(task_id, TaskState.CANCELED, now=now, reason=ReasonCode.OK)
        return CancelResult(accepted=True, reason=ReasonCode.OK, stopped=True)

    # ------------------------------------------------------------------ #

    def takeover_allowed(self, task_id: str) -> tuple[bool, ReasonCode]:
        """Can another robot take this task over?

        Only when nothing is in hand. A task that has picked a payload must be
        reconciled, not silently re-homed -- otherwise the payload teleports.
        """
        task = self.ledger.get(task_id)
        if task.cancel_requested:
            return False, ReasonCode.CANCEL_NOT_CONFIRMED
        if task.spec.payload_id:
            state, holder = self.ledger.payload_state(task.spec.payload_id)
            from .domain import PayloadState

            if state is PayloadState.HELD and holder is not None:
                return False, ReasonCode.PAYLOAD_HELD_ELSEWHERE
        if task.state in (TaskState.SUCCEEDED, TaskState.FAILED, TaskState.CANCELED):
            return False, ReasonCode.OK
        return True, ReasonCode.OK
