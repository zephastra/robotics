"""How a case expresses ARRIVAL ORDER without sleeping.

WHY THIS EXISTS
===============

`docs/LIMITATIONS.md` names the gap under `crossing_scenarios`:

    a way to express the ARRIVAL ORDER the case is about. The runner submits
    sequentially, so "r01 asks first" is not currently expressible

Sequential submission is not arrival order. Two requests sent back to back are picked up by
their robots at whatever moment each robot becomes free, so "r01 asked first" would be a race
the case happens to win. A case that passes by luck is worse than one that fails, because
nothing in the record says which of the two it was.

The obvious alternative is a sleep (`submit`, `sleep 20`, `submit`). That is the same shape as
the "sleep then kill" trigger this project already refuses elsewhere -- see the
`robot_kill_trigger` entry in the same register. A fixed sleep is a guess about how long
something takes, and it is wrong in both directions: too short and the condition has not
happened yet, too long and the case waits after it already has, so the next run starts from a
different state and the case stops being reproducible.

So a condition here is a PREDICATE OVER THE SNAPSHOT, evaluated until it holds. Nothing is
timed; the runner observes the state it is waiting for.

TWO KINDS OF FAILURE, KEPT APART
================================

A condition can fail to hold for two reasons that need different sentences:

  * it is not true YET -- the robot has not got there. Waiting is correct, and the runner
    should keep waiting until its declared budget runs out.
  * it can NEVER be true -- the task already reached a terminal state without ever executing.
    Waiting is useless, and a runner that keeps waiting turns a definite failure into a
    timeout with a vague message.

`WaitVerdict.impossible` carries the second case separately for exactly that reason.

WHAT A CONDITION MAY SAY
========================

A condition is a mapping and every key in it must hold:

    {request_id: <id>}                  that task reached EXECUTING (or later progress)
    {request_id: <id>, state: <STATE>}  that task reached <STATE>
    {robot_crossing: <rid>}             that robot is inside the passage right now
    {robot_carrying: <rid>}             that robot is holding a payload right now

An unknown key is refused rather than ignored: a condition that silently never fires would
make the case test nothing while reporting success, which is the failure this project has paid
for most often.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Mapping

#: The progress a task makes before it first moves, in order. A condition naming one of these
#: is satisfied when the task has reached it *or gone past it*.
PROGRESS: tuple[str, ...] = ("NOT_STARTED", "ACCEPTED", "ASSIGNED", "EXECUTING")

#: States from which no further progress happens. A condition naming a PROGRESS state can
#: never be satisfied once a task is in one of these without having passed through it.
TERMINAL: tuple[str, ...] = ("SUCCEEDED", "FAILED", "CANCELED", "NEEDS_ATTENTION")

#: The keys a condition may use.
KNOWN_KEYS: frozenset[str] = frozenset({
    "request_id",
    "state",
    "robot_crossing",
    #: Added in round 17 with the trigger vocabulary: F05 and F08 fault a robot
    #: BECAUSE it is carrying something, so "while it is carrying" has to be a
    #: state the runner can observe rather than a delay it hopes for.
    "robot_carrying",
})


@dataclass(frozen=True)
class WaitVerdict:
    """Whether a condition holds, and if not, whether waiting could ever help."""

    holds: bool
    impossible: bool = False
    detail: str = ""


def describe(condition: Mapping[str, Any]) -> str:
    """A condition, in the shape a person would say it in."""
    parts = []
    if "request_id" in condition:
        parts.append(f"task {condition['request_id']!r} is "
                     f"{condition.get('state', 'EXECUTING')}")
    if "robot_crossing" in condition:
        parts.append(f"{condition['robot_crossing']} is inside the passage")
    if "robot_carrying" in condition:
        parts.append(f"{condition['robot_carrying']} is holding a payload")
    return " and ".join(parts) if parts else "an empty condition"


def _satisfied(got: str, want: str) -> bool:
    """Is `got` at least `want`, in the ledger's own progression?"""
    if want in TERMINAL:
        return got == want
    if got in TERMINAL:
        # A terminal state is not "past EXECUTING": the task may never have executed at all,
        # and treating FAILED as further along than EXECUTING is how a case would wait for
        # something that has already been refused.
        return False
    if want not in PROGRESS or got not in PROGRESS:
        return got == want
    return PROGRESS.index(got) >= PROGRESS.index(want)


def condition_holds(snapshot: Mapping[str, Any], condition: Mapping[str, Any]) -> WaitVerdict:
    """Evaluate one condition against one snapshot. Never raises."""
    if not isinstance(condition, Mapping):
        return WaitVerdict(False, True, f"a wait condition must be a mapping, not "
                                         f"{type(condition).__name__}")
    unknown = sorted(set(condition) - KNOWN_KEYS)
    if unknown:
        return WaitVerdict(False, True,
                           f"unknown wait key(s) {unknown}; this build knows "
                           f"{sorted(KNOWN_KEYS)}")
    if "state" in condition and "request_id" not in condition:
        # `state` is a property of a named task, so on its own it has no subject. Left alone it
        # would fall through every clause and HOLD against an empty snapshot -- a condition
        # that is advertised, never evaluated, and always true.
        return WaitVerdict(False, True,
                           "`state` needs a `request_id` to say which task it is about; "
                           "on its own it would hold against every snapshot")

    rows = {row.get("request_id"): row for row in (snapshot.get("tasks") or [])}
    crossings = snapshot.get("crossings") or {}
    #: A payload is carried when the LEDGER says it is HELD and names a holder. Read
    #: from the ledger rather than from a robot's own report on purpose: "the payload
    #: is mine" is the party under test answering for itself, and this vocabulary is
    #: also used to decide when to fault that party.
    carriers = {str(row.get("holder")) for row in (snapshot.get("payloads") or [])
                if str(row.get("state")) == "HELD" and row.get("holder")}
    reasons: list[str] = []
    impossible = False

    if "request_id" in condition:
        wanted = str(condition["request_id"])
        want = str(condition.get("state", "EXECUTING"))
        row = rows.get(wanted)
        if row is None:
            reasons.append(f"task {wanted!r} is not in the ledger yet")
        else:
            got = str(row.get("state", "NOT_STARTED"))
            if not _satisfied(got, want):
                if got in TERMINAL and want in PROGRESS:
                    impossible = True
                    reasons.append(f"task {wanted!r} is {got} and never reached {want}, "
                                   f"so waiting cannot help")
                else:
                    reasons.append(f"task {wanted!r} is {got}, waiting for {want}")

    if "robot_crossing" in condition:
        wanted = str(condition["robot_crossing"])
        if wanted not in crossings:
            reasons.append(f"{wanted} is not inside the passage")

    if "robot_carrying" in condition:
        wanted = str(condition["robot_carrying"])
        if wanted not in carriers:
            reasons.append(f"{wanted} is not holding a payload")

    if reasons:
        return WaitVerdict(False, impossible, "; ".join(reasons))
    return WaitVerdict(True, False, describe(condition))
