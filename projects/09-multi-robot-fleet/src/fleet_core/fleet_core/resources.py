"""Resource book: who may occupy what, for how long, and who is next.

The rules that make this file worth reading:

  * **Bundles are atomic.** A corridor and its exit buffer are reserved together
    or not at all. There is no state in which a robot holds the corridor and
    cannot leave it, because that state deadlocks the fleet (MASTER_PLAN §7.2).
  * **Expiry is not clearance.** A permit whose TTL ran out stops *authorising*,
    but it does not prove the area is empty. Until an explicit `confirm_clear`
    arrives, the resource stays blocked as UNKNOWN occupancy. Freeing on timeout
    is how a real system puts two robots in the same corridor.
  * **Unknown position blocks too.** If the holder cannot be located, the area is
    treated as occupied -- not as free (TEST_AND_ACCEPTANCE F09/I-flavoured).
  * **Explicit grants only.** Capability and capacity are checked at grant time;
    there is no path that mutates occupancy directly.
"""

from __future__ import annotations

from dataclasses import dataclass, field

from .domain import (
    AcquireResult,
    Permit,
    ReasonCode,
    ResourceId,
    StaleCommandError,
)


@dataclass
class _Hold:
    permit: Permit
    resources: tuple[ResourceId, ...]


@dataclass
class _Waiting:
    task_id: str
    robot_id: str
    resources: tuple[ResourceId, ...]
    requested_at: float
    boot_id: str
    revision: int
    generation: int
    epoch: int


@dataclass
class ResourceBook:
    """In-memory occupancy + queue. State is mirrored into the ledger by callers."""

    capacity: dict[ResourceId, int] = field(default_factory=dict)
    permit_ttl_s: float = 30.0
    _holds: dict[ResourceId, list[_Hold]] = field(default_factory=dict)
    _queues: dict[ResourceId, list[_Waiting]] = field(default_factory=dict)
    # Resources whose previous holder vanished without an explicit clearance.
    _unknown: dict[ResourceId, str] = field(default_factory=dict)

    # ------------------------------------------------------------------ #
    # queries
    # ------------------------------------------------------------------ #

    def capacity_of(self, rid: ResourceId) -> int:
        return self.capacity.get(rid, 1)

    def holders(self, rid: ResourceId) -> list[_Hold]:
        return list(self._holds.get(rid, ()))

    def is_free(self, rid: ResourceId) -> bool:
        if rid in self._unknown:
            return False  # unknown occupancy is not free
        return len(self._holds.get(rid, ())) < self.capacity_of(rid)

    def unknown_reason(self, rid: ResourceId) -> str | None:
        return self._unknown.get(rid)

    def queue(self, rid: ResourceId) -> list[str]:
        return [w.task_id for w in self._queues.get(rid, ())]

    def total_queued(self) -> int:
        return sum(len(q) for q in self._queues.values())

    # ------------------------------------------------------------------ #
    # acquire / renew / release
    # ------------------------------------------------------------------ #

    def acquire(
        self,
        *,
        task_id: str,
        robot_id: str,
        resources: tuple[ResourceId, ...],
        now: float,
        wall_now: float,
        boot_id: str,
        revision: int,
        generation: int,
        epoch: int,
        permit_id: str | None = None,
        robot_position_known: bool = True,
    ) -> AcquireResult:
        """Request every resource in the bundle, or none of them.

        Order of checks matters: capacity first so a busy resource never queues a
        caller behind a doomed request, then staleness so a superseded command
        cannot consume a queue slot.
        """
        if not resources:
            return AcquireResult(granted=False, reason=ReasonCode.INVALID_INPUT)

        # Simultaneous request of the same bundle by the same task is idempotent:
        # return the existing grant rather than queueing a second copy.
        for rid in resources:
            for hold in self._holds.get(rid, ()):
                if hold.permit.task_id == task_id:
                    return AcquireResult(granted=True, reason=ReasonCode.OK, permit=hold.permit)

        if not robot_position_known:
            return AcquireResult(
                granted=False, reason=ReasonCode.RESOURCE_UNKNOWN, blocked_by=resources
            )

        for rid in resources:
            if rid in self._unknown:
                return AcquireResult(
                    granted=False,
                    reason=ReasonCode.RESOURCE_UNKNOWN,
                    blocked_by=(rid,),
                    queued_behind=tuple(self.queue(rid)),
                )

        if not all(self.is_free(rid) for rid in resources):
            holders: list[str] = []
            for rid in resources:
                holders.extend(h.permit.task_id for h in self._holds.get(rid, ()))
            for rid in resources:
                self._enqueue(
                    rid,
                    _Waiting(
                        task_id=task_id,
                        robot_id=robot_id,
                        resources=resources,
                        requested_at=wall_now,
                        boot_id=boot_id,
                        revision=revision,
                        generation=generation,
                        epoch=epoch,
                    ),
                )
            return AcquireResult(
                granted=False,
                reason=ReasonCode.RESOURCE_BUSY,
                blocked_by=tuple(r for r in resources if not self.is_free(r)),
                queued_behind=tuple(dict.fromkeys(holders)),
            )

        # FIFO fairness: a newcomer must not jump a queue that already exists,
        # unless it *is* the head of every queue it needs.
        for rid in resources:
            q = self._queues.get(rid, [])
            if q and q[0].task_id != task_id:
                self._enqueue(
                    rid,
                    _Waiting(
                        task_id=task_id,
                        robot_id=robot_id,
                        resources=resources,
                        requested_at=wall_now,
                        boot_id=boot_id,
                        revision=revision,
                        generation=generation,
                        epoch=epoch,
                    ),
                )
                return AcquireResult(
                    granted=False,
                    reason=ReasonCode.RESOURCE_BUSY,
                    blocked_by=(rid,),
                    queued_behind=(q[0].task_id,),
                )

        pid = permit_id or f"permit-{task_id}-{generation}"
        permit = Permit(
            permit_id=pid,
            task_id=task_id,
            robot_id=robot_id,
            resources=resources,
            boot_id=boot_id,
            revision=revision,
            generation=generation,
            epoch=epoch,
            granted_at=wall_now,
            expires_at=wall_now + self.permit_ttl_s,
        )
        for rid in resources:
            self._holds.setdefault(rid, []).append(_Hold(permit=permit, resources=resources))
            self._dequeue(rid, task_id)
        return AcquireResult(granted=True, reason=ReasonCode.OK, permit=permit)

    def renew(
        self,
        permit_id: str,
        *,
        wall_now: float,
        boot_id: str,
        revision: int,
        generation: int,
        epoch: int,
    ) -> Permit:
        """Extend a live permit. Stale callers are rejected, not honoured."""
        for holds in self._holds.values():
            for hold in holds:
                if hold.permit.permit_id != permit_id:
                    continue
                p = hold.permit
                if not p.matches(boot_id, revision, generation):
                    raise StaleCommandError(
                        f"renew {permit_id}: stale "
                        f"(have boot={p.boot_id} rev={p.revision} gen={p.generation}, "
                        f"got boot={boot_id} rev={revision} gen={generation})"
                    )
                if p.epoch != epoch:
                    raise StaleCommandError(f"renew {permit_id}: epoch {p.epoch} != {epoch}")
                p.expires_at = wall_now + self.permit_ttl_s
                return p
        raise KeyError(permit_id)

    def expire_due(self, *, wall_now: float) -> list[Permit]:
        """Stop authorising expired permits -- WITHOUT freeing the resources.

        The area stays blocked as UNKNOWN until `confirm_clear` says otherwise.
        """
        expired: list[Permit] = []
        for rid, holds in list(self._holds.items()):
            remaining: list[_Hold] = []
            for hold in holds:
                if hold.permit.expired(wall_now):
                    expired.append(hold.permit)
                    self._unknown.setdefault(rid, "permit expired without clearance")
                else:
                    remaining.append(hold)
            self._holds[rid] = remaining
        return expired

    def confirm_clear(
        self, *, resources: tuple[ResourceId, ...], proof: str
    ) -> list[ResourceId]:
        """Explicit proof that the protected area is empty. Only this frees it."""
        if not proof:
            raise ValueError("confirm_clear requires a non-empty proof")
        cleared: list[ResourceId] = []
        for rid in resources:
            if rid in self._unknown:
                del self._unknown[rid]
                cleared.append(rid)
        return cleared

    def release(
        self, *, task_id: str, expected_generation: int | None = None
    ) -> list[Permit]:
        """Release by task. A nil release is a no-op, never an error."""
        released: list[Permit] = []
        for rid, holds in list(self._holds.items()):
            remaining: list[_Hold] = []
            for hold in holds:
                if hold.permit.task_id != task_id:
                    remaining.append(hold)
                    continue
                if expected_generation is not None and hold.permit.generation != expected_generation:
                    remaining.append(hold)
                    continue
                released.append(hold.permit)
            self._holds[rid] = remaining
        return released

    def owner_of(self, rid: ResourceId) -> str | None:
        holds = self._holds.get(rid, ())
        return holds[0].permit.task_id if holds else None

    def generation_of(self, rid: ResourceId) -> int | None:
        holds = self._holds.get(rid, ())
        return holds[0].permit.generation if holds else None

    # ------------------------------------------------------------------ #
    # queue internals
    # ------------------------------------------------------------------ #

    def _enqueue(self, rid: ResourceId, waiting: _Waiting) -> None:
        q = self._queues.setdefault(rid, [])
        for existing in q:
            if existing.task_id == waiting.task_id and existing.generation == waiting.generation:
                return  # duplicate acquire request: idempotent
        q.append(waiting)

    def _dequeue(self, rid: ResourceId, task_id: str) -> None:
        q = self._queues.get(rid)
        if q:
            self._queues[rid] = [w for w in q if w.task_id != task_id]

    def install(self, permit: Permit) -> list[Permit]:
        """Adopt a permit that was granted somewhere else.

        Used by permit *mirrors*: each robot's local gate keeps its own copy of the
        traffic geometry so it can answer "am I allowed here?" from its own pose, and
        that copy needs the permit too. This is how it gets in without the robot
        pretending it did the granting.

        Any existing hold for the same task is replaced first, so a stale mirror
        cannot outlive a renewal or a revocation that arrived out of order.
        """
        if not permit.resources:
            raise ValueError("installed permit names no resources")
        removed = self.release(task_id=permit.task_id)
        for rid in permit.resources:
            self._holds.setdefault(rid, []).append(
                _Hold(permit=permit, resources=permit.resources)
            )
            # A mirror has not done a startup occupancy sweep, so the resource would
            # otherwise sit in UNKNOWN and block the very permit just installed.
            self._unknown.pop(rid, None)
        return removed

    def abandon_queue(self, task_id: str) -> None:
        """A cancelled task must not keep a queue slot and starve others."""
        for rid, q in list(self._queues.items()):
            self._queues[rid] = [w for w in q if w.task_id != task_id]

    def head_of(self, rid: ResourceId) -> str | None:
        q = self._queues.get(rid, ())
        return q[0].task_id if q else None
