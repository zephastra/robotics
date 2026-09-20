"""Fault takeover rules -- CONTRACTS section 5, as code.

The table in the contract is a set of permissions and prohibitions, and the
prohibitions are the interesting half:

| when the fault happens        | allowed                        | forbidden |
| unassigned                    | pick a different capable robot | duplicate the task |
| assigned, nothing picked yet  | bump revision after the old generation is *proven* stopped | let two robots race for the same payload because a heartbeat went quiet |
| payload already held          | mark NEEDS_ATTENTION, let unrelated tasks continue | hand the payload_id to a new robot as if it moved |
| lost inside the corridor      | block the resource, wait for clearance | declare the corridor empty when a TTL expires |
| delivered, ack lost           | idempotent answer from the ledger | unload or load a second time |

Every one of those prohibitions is a thing a reasonable-looking implementation
does by accident, so this module returns a decision *with an action string*
rather than a bool. A bool would let the caller re-derive the rule and get it
wrong in exactly the way the table warns about.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum

from .domain import PayloadState, ReasonCode, Task, TaskState


class FaultContext(str, Enum):
    UNASSIGNED = "UNASSIGNED"
    ASSIGNED_NOT_PICKED = "ASSIGNED_NOT_PICKED"
    PAYLOAD_HELD = "PAYLOAD_HELD"
    IN_CORRIDOR = "IN_CORRIDOR"
    DELIVERED_ACK_LOST = "DELIVERED_ACK_LOST"
    COMPLETED = "COMPLETED"


@dataclass(frozen=True)
class TakeoverDecision:
    allowed: bool
    reason: ReasonCode
    action: str
    detail: str

    @property
    def action_required(self) -> bool:
        return self.action in ("MARK_NEEDS_ATTENTION", "BLOCK_RESOURCE")


class InCorridorKnowledge(str, Enum):
    """Three states, because "we do not know" must not collapse into "no"."""

    YES = "YES"
    NO = "NO"
    UNKNOWN = "UNKNOWN"


def classify(
    task: Task,
    *,
    payload_state: PayloadState | None,
    payload_holder: str | None,
    in_corridor: InCorridorKnowledge = InCorridorKnowledge.UNKNOWN,
) -> FaultContext:
    """Where in the task's life the fault landed.

    ``UNKNOWN`` corridor knowledge is not ``NO``: if the fleet cannot say whether
    the failed robot is inside the shared region, the conservative answer is the
    in-corridor one, so an unknown is classified as a possible corridor loss.
    """
    if task.state in (TaskState.SUCCEEDED, TaskState.CANCELED, TaskState.FAILED):
        return FaultContext.COMPLETED
    if in_corridor is InCorridorKnowledge.YES or in_corridor is InCorridorKnowledge.UNKNOWN:
        if task.state in (TaskState.EXECUTING, TaskState.ASSIGNED):
            return FaultContext.IN_CORRIDOR
    if payload_state is PayloadState.HELD and payload_holder is not None:
        return FaultContext.PAYLOAD_HELD
    if payload_state is PayloadState.DELIVERED:
        return FaultContext.DELIVERED_ACK_LOST
    if task.robot_id is None:
        return FaultContext.UNASSIGNED
    return FaultContext.ASSIGNED_NOT_PICKED


def decide(
    task: Task,
    *,
    payload_state: PayloadState | None,
    payload_holder: str | None,
    stopped_confirmed: bool,
    in_corridor: InCorridorKnowledge = InCorridorKnowledge.UNKNOWN,
) -> TakeoverDecision:
    """Can this task move to another robot, and what must happen first?

    ``stopped_confirmed`` is the load-bearing input. "The heartbeat stopped" is
    not evidence that the old robot stopped, and using it as if it were is how
    two robots end up competing for one payload.
    """
    where = classify(
        task,
        payload_state=payload_state,
        payload_holder=payload_holder,
        in_corridor=in_corridor,
    )

    if where is FaultContext.COMPLETED:
        return TakeoverDecision(
            allowed=False,
            reason=ReasonCode.OK,
            action="REPORT_TERMINAL",
            detail=f"task is already {task.state.value}; nothing to take over",
        )

    if where is FaultContext.IN_CORRIDOR:
        return TakeoverDecision(
            allowed=False,
            reason=ReasonCode.BLOCKED,
            action="BLOCK_RESOURCE",
            detail="fault inside the protected region: block the resource and wait for "
            "clearance; a permit TTL expiring is not evidence that the region is free",
        )

    if where is FaultContext.PAYLOAD_HELD:
        return TakeoverDecision(
            allowed=False,
            reason=ReasonCode.PAYLOAD_HELD_ELSEWHERE,
            action="MARK_NEEDS_ATTENTION",
            detail=f"payload is HELD by {payload_holder!r}; a held payload is not a free "
            "resource, so unrelated tasks continue and this one is flagged",
        )

    if where is FaultContext.DELIVERED_ACK_LOST:
        return TakeoverDecision(
            allowed=False,
            reason=ReasonCode.OK,
            action="REPLY_FROM_LEDGER",
            detail="delivery is already recorded; answer idempotently instead of "
            "loading or unloading a second time",
        )

    if where is FaultContext.UNASSIGNED:
        return TakeoverDecision(
            allowed=True,
            reason=ReasonCode.OK,
            action="REASSIGN",
            detail="no robot held it and nothing was picked up; re-run the allocator",
        )

    if not stopped_confirmed:
        return TakeoverDecision(
            allowed=False,
            reason=ReasonCode.CANCEL_NOT_CONFIRMED,
            action="WAIT_FOR_STOP",
            detail="the previous robot's motion is not confirmed stopped, so a new "
            "assignment could start while the old command is still running",
        )

    return TakeoverDecision(
        allowed=True,
        reason=ReasonCode.OK,
        action="BUMP_REVISION_AND_REASSIGN",
        detail="previous generation proven stopped, nothing in hand: increase the "
        "assignment revision and re-allocate",
    )
