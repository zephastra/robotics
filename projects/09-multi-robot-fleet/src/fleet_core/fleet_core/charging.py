"""Charger admission. Capacity is per pad; a second robot waits, it never stacks.

MASTER_PLAN section 9 (P5) asks for a simulated charging-area capacity, and the
handoff README's phase check says "充电位有人时排队，不叠在一起" -- a robot queues
when the pad is taken, and they do not pile up. That second half is a geometry
claim, not only a scheduling one, so this module refuses the second robot
outright instead of trusting the dispatcher to be polite.

Two design decisions worth stating:

  * **Capacity is per charger, not global.** ``config/fleet.yaml`` has two pads
    (``C_left``, ``C_right``) and a ``charger_capacity``. Treating the setting as
    a fleet-wide pool would let the scheduler admit two robots to one pad, which
    is exactly the pile-up the requirement forbids.
  * **A released pad does not auto-promote the queue head.** The waiting robot
    has to ask again, because between queueing and the pad freeing up its
    reachability may have changed. Auto-promotion would hand a pad to a robot
    that can no longer get there.
  * **A pad a robot cannot reach is not capacity for that robot.** Which pads those
    are is a fact about the site, so the caller supplies it (`reachable=`) rather than
    this module guessing: the allocator owns the policy, the caller owns the geometry.
    The first version of this class had no such notion, and the dispatcher asked for a
    pad with no preference, so every robot was sent to `sorted(chargers)[0]` -- which
    is on the far side of the barrier for half the fleet (D-P5-22).
"""

from __future__ import annotations

from dataclasses import dataclass, field

from .domain import FleetConfig, ReasonCode


def charge_request_id(robot_id: str, epoch: int, serial: int) -> str:
    """The idempotency key for a system-generated charge run.

    `epoch` is in the name for a reason that cost a run to learn: the serial alone
    restarts at 1 in every process, `request_id` is the ledger's dedup key, and the
    ledger is durable on purpose. So the first charge run of a new session reused the
    request_id of a dead one, `Ledger.submit` correctly reported `created=False`, and the
    caller returned early -- after the charge pad had already been granted. The pad was
    then held for the rest of the session by a robot that had no task to release it.

    Including the epoch makes the key unique per task-service generation while staying
    deterministic and readable, which matters because it ends up in the event log.
    """
    return f"auto-charge-{robot_id}-{int(epoch)}-{int(serial)}"


@dataclass(frozen=True)
class ChargeGrant:
    robot_id: str
    charger_id: str
    granted_at: float
    waited_s: float = 0.0


@dataclass
class _Queued:
    robot_id: str
    since: float
    preferred: str | None
    #: Pads this robot can be dispatched to. ``None`` means "not established", and then
    #: the entry competes for every pad exactly as it did before this field existed.
    #: Without it, a robot queued with no preference matched EVERY pad, so a west-side
    #: robot would have sat at the head of the east pad's queue and blocked the only
    #: robot that could use it -- a deadlock created by fixing a different bug.
    reachable: tuple[str, ...] | None = None


@dataclass
class ChargerAllocator:
    """FIFO per pad, one occupant per pad, one pad per robot."""

    cfg: FleetConfig
    capacity_override: int | None = None
    _occupant: dict[str, str] = field(default_factory=dict)
    _holder: dict[str, str] = field(default_factory=dict)
    _queue: list[_Queued] = field(default_factory=list)
    _granted_at: dict[str, float] = field(default_factory=dict)
    refusals: dict[str, int] = field(default_factory=dict)
    grants: int = 0
    releases: int = 0

    # ------------------------------------------------------------------ #

    @property
    def capacity(self) -> int:
        if self.capacity_override is not None:
            return self.capacity_override
        return int(self.cfg.charger_capacity or 1)

    @property
    def chargers(self) -> tuple[str, ...]:
        return tuple(sorted(self.cfg.chargers))

    def occupant(self, charger_id: str) -> str | None:
        return self._occupant.get(charger_id)

    def holder(self, robot_id: str) -> str | None:
        return self._holder.get(robot_id)

    def queue_for(self, charger_id: str) -> tuple[str, ...]:
        return tuple(q.robot_id for q in self._queue if self._admits(q, charger_id))

    def queue(self) -> tuple[str, ...]:
        return tuple(q.robot_id for q in self._queue)

    # ------------------------------------------------------------------ #

    def request(
        self,
        robot_id: str,
        *,
        now: float,
        preferred: str | None = None,
        reachable: "tuple[str, ...] | None" = None,
    ) -> ChargeGrant | None:
        """Ask for a pad.

        Returns a grant, or ``None`` after queueing. ``None`` is not an error:
        the caller re-asks next tick. Nothing here blocks.

        ``reachable`` is the set of pads the caller has established this robot can be
        dispatched to. Passing it is not optional in practice -- a charge run is pinned,
        so a pad the robot cannot drive to is not a long wait, it is a permanent
        refusal -- but it is optional in the signature so the core stays usable without
        a site layout.
        """
        if robot_id not in self.cfg.robots:
            self.refusals[ReasonCode.INVALID_INPUT.value] = (
                self.refusals.get(ReasonCode.INVALID_INPUT.value, 0) + 1
            )
            return None

        held = self._holder.get(robot_id)
        if held is not None:
            # Already charging. Re-granting would let one robot consume two pads
            # if it were ever dispatched twice in the same tick.
            self.refusals[ReasonCode.RESOURCE_BUSY.value] = (
                self.refusals.get(ReasonCode.RESOURCE_BUSY.value, 0) + 1
            )
            return None

        if preferred is not None and preferred not in self.cfg.chargers:
            preferred = None

        if reachable is None:
            admissible: tuple[str, ...] = self.chargers
        else:
            wanted = set(reachable)
            admissible = tuple(c for c in self.chargers if c in wanted)
            if not admissible:
                # Nothing this robot can drive to. Queueing would be worse than
                # refusing: the entry would hold a place at pads it cannot use, and the
                # caller would read a queue position as progress.
                self._dequeue(robot_id)
                self.refusals[ReasonCode.NO_SAFE_CHARGER.value] = (
                    self.refusals.get(ReasonCode.NO_SAFE_CHARGER.value, 0) + 1
                )
                return None

        # A preference that is not admissible is ignored rather than honoured: the
        # stated preference is a wish, the reachable set is a fact.
        order = ([preferred] if preferred in admissible else []) + [
            c for c in admissible if c != preferred
        ]
        for charger_id in order:
            occupant = self._occupant.get(charger_id)
            if occupant is not None and occupant != robot_id:
                continue
            # FIFO is decided by the *head* of this pad's queue, not by "is anyone
            # else waiting". Comparing against every other queued robot would let a
            # robot at the front be overtaken by the ones behind it: the front
            # robot asks, sees r04 also waiting, and re-queues for ever.
            waiting = [q for q in self._queue if self._admits(q, charger_id)]
            if waiting and waiting[0].robot_id != robot_id:
                self._enqueue(robot_id, now, preferred, reachable)
                self.refusals[ReasonCode.RESOURCE_BUSY.value] = (
                    self.refusals.get(ReasonCode.RESOURCE_BUSY.value, 0) + 1
                )
                return None
            # Read the queue entry BEFORE removing it. Reading afterwards always
            # finds nothing, so `waited_s` would be 0 for every grant -- a queue
            # statistic that reports "nobody ever waited" is worse than no
            # statistic, because it reads as evidence that the fleet is idle.
            queued = next((q for q in self._queue if q.robot_id == robot_id), None)
            self._dequeue(robot_id)
            self._occupant[charger_id] = robot_id
            self._holder[robot_id] = charger_id
            self._granted_at[robot_id] = now
            self.grants += 1
            waited = now - queued.since if queued else 0.0
            return ChargeGrant(robot_id, charger_id, now, waited)

        self._enqueue(robot_id, now, preferred, reachable)
        self.refusals[ReasonCode.CHARGER_BUSY.value] = (
            self.refusals.get(ReasonCode.CHARGER_BUSY.value, 0) + 1
        )
        return None

    def release(self, robot_id: str, *, now: float) -> bool:
        charger_id = self._holder.pop(robot_id, None)
        if charger_id is None:
            return False
        if self._occupant.get(charger_id) == robot_id:
            del self._occupant[charger_id]
        self._granted_at.pop(robot_id, None)
        self._dequeue(robot_id)
        self.releases += 1
        return True

    # ------------------------------------------------------------------ #

    def _admits(self, q: _Queued, charger_id: str) -> bool:
        """Does this queue entry compete for this pad?

        Both halves matter. A robot that stated a preference competes only for that pad;
        a robot whose reachable set was established competes only for pads in it. Getting
        the second half wrong is how one robot's queue entry blocks another robot's only
        usable pad, which reads as a full charging area with an empty pad in it.
        """
        if q.preferred is not None and q.preferred != charger_id:
            return False
        return q.reachable is None or charger_id in q.reachable

    def _enqueue(
        self,
        robot_id: str,
        now: float,
        preferred: str | None,
        reachable: "tuple[str, ...] | None" = None,
    ) -> None:
        for q in self._queue:
            if q.robot_id == robot_id:
                # Re-asking with a different preference keeps the place in the queue:
                # FIFO is about when the robot started waiting, not about what it has
                # asked for since. `since` is deliberately left alone.
                q.preferred = preferred
                q.reachable = reachable
                return
        self._queue.append(_Queued(robot_id, now, preferred, reachable))

    def _dequeue(self, robot_id: str) -> None:
        self._queue = [q for q in self._queue if q.robot_id != robot_id]

    def drop_offline(self, robot_id: str, *, now: float) -> bool:
        """A robot that vanished must not keep a pad reserved forever."""
        released = self.release(robot_id, now=now)
        before = len(self._queue)
        self._dequeue(robot_id)
        return released or len(self._queue) != before

    # ------------------------------------------------------------------ #

    def snapshot(self) -> dict:
        return {
            "capacity_per_charger": self.capacity,
            "chargers": list(self.chargers),
            "occupant": dict(sorted(self._occupant.items())),
            "holder": dict(sorted(self._holder.items())),
            "queue": [
                {"robot_id": q.robot_id, "since": q.since, "preferred": q.preferred,
                 # Reported so a run can show WHY this robot is waiting for this pad and
                 # not the empty one next to it.
                 "reachable": list(q.reachable) if q.reachable is not None else None}
                for q in self._queue
            ],
            "grants": self.grants,
            "releases": self.releases,
            "refusals": dict(sorted(self.refusals.items())),
        }
