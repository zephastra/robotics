"""Protected-crossing traffic logic: permits, occupancy, and clearance.

State machine (CONTRACTS section 7)::

    FREE -> RESERVED -> OCCUPIED -> CLEARING -> FREE
              |            |           |
              +--------- UNKNOWN / BLOCKED

Three rules this file exists to enforce, and each one is a specific way a fleet
puts two robots in the same corridor:

1. **A permit is not an occupancy.** ``RESERVED`` means "the ledger says this
   robot may go". It does not mean the corridor is empty. A resource becomes
   ``FREE`` only when all three hold: nobody in the ledger, a fresh enough
   observation that no robot is inside, and an explicit clearance proof.

2. **Expiry is not clearance.** When a permit lapses the resource becomes
   ``UNKNOWN``, never ``FREE``. It stays blocked -- possibly forever -- until
   someone proves it is empty. Freeing on timeout is exactly how a real system
   ends up with two robots in one passage.

3. **Leaving is a geometric claim the manager verifies itself.** A robot may not
   report ``clear=true``. It reports where it is; this module tests the whole
   footprint plus margin against the region and decides. A robot parked on the
   exit buffer is still inside the bundle and is refused (CONTRACTS section 8,
   step 5).

Permissions are evaluated **per resource**, not per bundle. Holding
``[mid, mid_right]`` authorises the corridor and the east exit buffer; it does
*not* authorise driving into ``mid_left``. A bundle-level check would miss that,
so the refusal test below is written against the resource each rectangle belongs
to.

Everything here is pure Python. The gate imports it, the ROS node imports it, and
the unit tests import it with no ROS present at all.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

from .domain import (
    AcquireResult,
    ClearanceOutcome,
    DirectionSpec,
    NodePose,
    Permit,
    PermitCheck,
    ReasonCode,
    ResourceId,
    ResourceState,
    StaleCommandError,
    TrafficConfig,
)
from .geometry import Rect, Region, StopModel, dominant_speed, footprint_corners, footprint_radius
from .resources import ResourceBook


def _region(rects: tuple[Rect, ...], *, name: str, margin_m: float) -> Region:
    # Region refuses an empty rectangle tuple, which is the right behaviour for
    # config but not here: "no rectangles" means "nothing is forbidden", and the
    # caller checks for that explicitly rather than catching an exception.
    if not rects:
        return Region(name=name, rects=(Rect(f"{name}:empty", 1e9, 1e9, 1e9 + 1e-6, 1e9 + 1e-6),), margin_m=margin_m)
    return Region(name=name, rects=rects, margin_m=margin_m)


def regions_overlapping(
    cfg: TrafficConfig,
    pose: tuple[float, float, float],
    *,
    length_m: float,
    width_m: float,
    margin_m: float | None = None,
) -> tuple[str, ...]:
    """Names of protected resources whose rectangle this body overlaps.

    The empty tuple means "clear of every resource". Uses the same rectangles and the
    same footprint expansion as the geofence, so a recovery decision that says "the
    failed robot is in the open" cannot drift from the thing it is a statement about.

    Why this exists: the dispatcher answered the in-corridor question with a hardcoded
    ``UNKNOWN``, and ``recovery.classify`` reads UNKNOWN as a possible corridor loss.
    Every fault anywhere in the world was therefore parked as BLOCK_RESOURCE, which made
    the contract's "payload already held" and "assigned, nothing picked yet" rows
    unreachable in production.
    """
    margin = cfg.footprint_margin_m if margin_m is None else margin_m
    corners = footprint_corners(pose[0], pose[1], pose[2], length_m, width_m)
    return tuple(
        sorted(
            name
            for name, spec in cfg.resources.items()
            if _region(spec.rects, name=name, margin_m=margin).box_overlap(corners)
        )
    )


@dataclass(frozen=True)
class ClearanceScan:
    """One complete occupancy observation, resource by resource.

    Every field describes the *same* instant, so a caller cannot combine a stale pose
    with a fresh verdict.

    ``unlocatable`` is deliberately not a boolean. A robot whose pose we do not have
    is not evidence that it is elsewhere, so a scan that could not locate everyone
    promotes nothing -- and it says so, rather than returning an empty list that
    reads like "nothing was clear".
    """

    freeable: tuple[str, ...] = ()
    occupied: tuple[tuple[str, tuple[str, ...]], ...] = ()
    held: tuple[str, ...] = ()
    unlocatable: tuple[str, ...] = ()

    @property
    def complete(self) -> bool:
        """True only when every configured robot was located."""
        return not self.unlocatable


@dataclass
class CrossingManager:
    """Owns the crossing's state machine and answers the gate's question.

    ``book`` is the P1 lease ledger (who holds what, FIFO order, idempotency).
    This class adds the parts the ledger deliberately does not have: the
    RESERVED/OCCUPIED/CLEARING distinction, the geometric clearance test, and
    the entry precondition (a robot must be stopped at a legal waiting point
    before it may ask).
    """

    cfg: TrafficConfig
    book: ResourceBook = field(default_factory=ResourceBook)

    # Runtime observation, not configuration -- kept out of TrafficConfig so a
    # config reload can never silently reset an occupancy belief.
    _state: dict[str, ResourceState] = field(default_factory=dict)
    _block_reason: dict[str, str] = field(default_factory=dict)
    # task_id -> the bundle it was last granted. Kept so a refusal can say
    # "your permit lapsed" rather than the useless "you have no permit" -- same
    # code path, very different diagnosis.
    _granted_bundle: dict[str, tuple[str, ...]] = field(default_factory=dict)
    # task_id -> the wall-clock deadline of the grant that bundle came from.
    # Kept separately from the permit because a mirror may never hold one that
    # this process granted, and 'it ran out' must not be reported as 'it was
    # never yours'.
    _granted_expiry: dict[str, float] = field(default_factory=dict)
    _last_inside: dict[str, bool] = field(default_factory=dict)

    def __post_init__(self) -> None:
        for spec in self.cfg.resources.values():
            self.book.capacity.setdefault(ResourceId(spec.kind, spec.name), spec.capacity)
            # Initial state is UNKNOWN, not FREE (CONTRACTS section 7): until the
            # configured robots have reported in and been checked geometrically,
            # nobody knows whether the corridor is empty.
            self._state.setdefault(spec.name, ResourceState.UNKNOWN)
        self.book.permit_ttl_s = self.cfg.permit_ttl_s

    # ------------------------------------------------------------------ #
    # identity helpers
    # ------------------------------------------------------------------ #

    def rid(self, name: str) -> ResourceId:
        spec = self.cfg.resources[name]
        return ResourceId(spec.kind, spec.name)

    def bundle(self, direction: str) -> tuple[str, ...]:
        return self.cfg.bundles[direction]

    def direction_spec(self, direction: str) -> DirectionSpec:
        return self.cfg.directions[direction]

    def node(self, name: str) -> NodePose:
        return self.cfg.nodes[name]

    def region_of(self, names: tuple[str, ...], *, margin_m: float | None = None) -> Region:
        rects: list[Rect] = []
        for n in names:
            rects.extend(self.cfg.resources[n].rects)
        return _region(
            tuple(rects),
            name="+".join(names),
            margin_m=self.cfg.footprint_margin_m if margin_m is None else margin_m,
        )

    def full_region(self) -> Region:
        return self.region_of(tuple(self.cfg.resources))

    # ------------------------------------------------------------------ #
    # state queries
    # ------------------------------------------------------------------ #

    def state_of(self, name: str) -> ResourceState:
        return self._state.get(name, ResourceState.UNKNOWN)

    def block_reason(self, name: str) -> str | None:
        return self._block_reason.get(name)

    def snapshot(self) -> dict[str, dict]:
        """Read-only view for reports and the dashboard. Never used for control."""
        out: dict[str, dict] = {}
        for name in self.cfg.resources:
            rid = self.rid(name)
            out[name] = {
                "state": self.state_of(name).value,
                "owner": self.book.owner_of(rid),
                "generation": self.book.generation_of(rid),
                "queue": self.book.queue(rid),
                "blocked_reason": self._block_reason.get(name),
            }
        return out

    # ------------------------------------------------------------------ #
    # the gate's question
    # ------------------------------------------------------------------ #

    def _stopping_reserve_m(self, *, speed_mps: float) -> float:
        return self.cfg.stop.envelope_m(speed_mps) + self.cfg.stop.localization_margin_m

    def protected_region(
        self, names: tuple[str, ...], *, speed_mps: float, name: str = "protected"
    ) -> Region:
        """THE boundary: the rectangles grown by the distance the robot cannot stop within,
        then by the footprint margin.

        One definition, because there used to be two. The gate's refusal had the stopping
        reserve and the clearance proof did not, so `cleared` was strictly weaker than `clear`
        and the two could disagree about the same pose in the same instant -- which they did,
        live, on `mid_right`. Two components that must agree about one boundary may not each
        compute it.
        """
        reserve = self._stopping_reserve_m(speed_mps=speed_mps)
        rects: list[Rect] = []
        for name_ in names:
            rects.extend(r.expanded(reserve) for r in self.cfg.resources[name_].rects)
        return _region(tuple(rects), name=name, margin_m=self.cfg.footprint_margin_m)

    def clearance_region(self, names: tuple[str, ...]) -> Region:
        """`protected_region` at the fastest speed the stop model was measured at.

        Monotone by construction -- `envelope_m` increases with speed -- so a pose outside this
        region is outside the refusal boundary at every speed the gate may authorise. A
        clearance proved at 0.10 m/s would not be, and that gap is the defect this closes.
        """
        return self.protected_region(
            names, speed_mps=self.cfg.stop.max_measured_speed_mps, name="clearance")

    def clearance_reserve_m(self) -> float:
        """The reserve the clearance proof used, so a message can name it."""
        return self._stopping_reserve_m(speed_mps=self.cfg.stop.max_measured_speed_mps)

    def stop_boundary(self, unheld: tuple[str, ...], *, speed_mps: float) -> Region:
        """The forbidden area grown by the distance the robot cannot stop within.

        CONTRACTS section 8: the gate must check the *conservatively reachable*
        area, not the current pose. A robot that cannot stop before the wall has
        already, in the only sense that matters, entered it.
        """
        return self.protected_region(unheld, speed_mps=speed_mps, name="stop_boundary")

    def _unpermitted_hits(
        self, unheld: tuple[str, ...], corners: tuple, *, speed_mps: float
    ) -> tuple[str, ...]:
        """Which *resources* this footprint is too close to, one by one.

        Per-resource rather than one merged region so the refusal can name the
        resource. A merged test would answer "something is in the way"; the
        operator needs "the west exit buffer is not yours".
        """
        hits: list[str] = []
        for name in unheld:
            # The same boundary `clearance_region` uses, from the same function, so
            # `cleared` implies `not refused` at every speed the gate may authorise.
            region = self.protected_region((name,), speed_mps=speed_mps, name=f"sb:{name}")
            if region.box_overlap(corners):
                hits.append(name)
        return tuple(hits)

    def check_movement(
        self,
        *,
        task_id: str,
        boot_id: str,
        revision: int,
        generation: int,
        epoch: int,
        wall_now: float,
        pose: tuple[float, float, float],
        twist: tuple[float, float, float],
        length_m: float,
        width_m: float,
        localization_valid: bool,
    ) -> PermitCheck:
        """May this robot keep moving right now?

        Never raises. The gate runs on a wall timer, and an exception in that
        callback is a silently open door, so every refusal is a returned value
        with a reason attached.
        """
        if not localization_valid:
            # Fail closed. CONTRACTS section 8: if we cannot bound where the robot
            # is, we cannot promise it will stop outside the boundary.
            return PermitCheck(
                allowed=False,
                reason=ReasonCode.LOCALIZATION_STALE,
                detail="localization invalid: cannot prove the stop boundary is respected",
            )

        held = self.held_resources(
            task_id=task_id, boot_id=boot_id, revision=revision,
            generation=generation, epoch=epoch, wall_now=wall_now,
        )
        unheld = tuple(n for n in self.cfg.resources if n not in held)

        x, y, yaw = pose
        corners = footprint_corners(x, y, yaw, length_m, width_m)
        speed = dominant_speed(twist, footprint_radius(length_m, width_m))

        hits = self._unpermitted_hits(unheld, corners, speed_mps=speed)
        if not hits:
            return PermitCheck(allowed=True, reason=ReasonCode.OK)

        reserve = self._stopping_reserve_m(speed_mps=speed)
        granted = self._granted_bundle.get(task_id)
        deadline = self._granted_expiry.get(task_id)
        lapsed = (deadline is not None and wall_now >= deadline) or any(
            "expired" in (self._block_reason.get(n) or "") for n in hits
        )
        if granted is not None and set(hits) <= set(granted):
            # The acting robot held exactly these and no longer does. Report its
            # own situation first: "your permit lapsed" is more actionable than
            # "the resource is unknown", and they are the same event.
            reason = ReasonCode.PERMIT_EXPIRED if lapsed else ReasonCode.PERMIT_STALE
        elif any(self.state_of(n) in (ResourceState.UNKNOWN, ResourceState.BLOCKED) for n in hits):
            reason = ReasonCode.RESOURCE_UNKNOWN
        else:
            # It never held these -- most often the *other* side's exit buffer.
            reason = ReasonCode.ROUTE_REQUIRES_PERMIT

        return PermitCheck(
            allowed=False,
            reason=reason,
            stop_boundary_hit=hits,
            detail=(
                f"speed {speed:.3f} m/s -> stopping envelope {self.cfg.stop.envelope_m(speed):.3f} m "
                f"+ localization margin {self.cfg.stop.localization_margin_m:.3f} m, "
                f"then footprint margin {self.cfg.footprint_margin_m:.3f} m; "
                f"unpermitted {list(hits)}, holdings {sorted(held) or 'none'} "
                f"(reserve {reserve:.3f} m)"
            ),
        )

    def held_resources(
        self, *, task_id: str, boot_id: str, revision: int, generation: int, epoch: int, wall_now: float
    ) -> tuple[str, ...]:
        """Which resources this robot may enter *right now*, ignoring expired ones."""
        held: list[str] = []
        for name in self.cfg.resources:
            rid = self.rid(name)
            for hold in self.book.holders(rid):
                p = hold.permit
                if p.task_id != task_id or p.expired(wall_now):
                    continue
                if not p.matches(boot_id, revision, generation) or p.epoch != epoch:
                    continue
                held.append(name)
        return tuple(held)

    # ------------------------------------------------------------------ #
    # acquire
    # ------------------------------------------------------------------ #

    def at_node(self, name: str, pose: tuple[float, float, float]) -> bool:
        n = self.node(name)
        return math.hypot(pose[0] - n.x, pose[1] - n.y) <= self.cfg.node_reach_tolerance_m

    def node_distance_text(self, pose: tuple[float, float, float], names) -> str:
        """Distances to candidate nodes, so a refusal states how far out it was."""
        parts = []
        for name in names:
            n = self.node(name)
            d = math.hypot(pose[0] - n.x, pose[1] - n.y)
            parts.append(f"{name} {d:.3f} m")
        return ", ".join(parts) + f" (pose x={pose[0]:.3f} y={pose[1]:.3f})"

    def acquire(
        self,
        direction: str,
        *,
        task_id: str,
        robot_id: str,
        boot_id: str,
        revision: int,
        generation: int,
        epoch: int,
        now: float,
        wall_now: float,
        pose: tuple[float, float, float],
        length_m: float,
        width_m: float,
        localization_valid: bool,
        permit_id: str | None = None,
    ) -> AcquireResult:
        """Ask for the whole bundle, from a legal waiting point, or not at all.

        A request from anywhere else is refused, not queued. Queueing it would
        let a robot that is already parked in the passage hold a claim on it
        (CONTRACTS section 7).
        """
        if direction not in self.cfg.directions:
            return AcquireResult(granted=False, reason=ReasonCode.INVALID_INPUT)
        if not localization_valid:
            return AcquireResult(granted=False, reason=ReasonCode.LOCALIZATION_STALE)

        # CONTRACTS section 7: at first boot every resource is UNKNOWN, and a
        # resource must not be granted before the configured robots have reported
        # in and been checked. Without this the *lease ledger* looks empty and the
        # grant succeeds -- the corridor only looks available because nobody has
        # looked at it yet.
        unchecked = [
            n for n in self.bundle(direction)
            if self.state_of(n) in (ResourceState.UNKNOWN, ResourceState.BLOCKED)
        ]
        if unchecked:
            return AcquireResult(
                granted=False,
                reason=ReasonCode.RESOURCE_UNKNOWN,
                blocked_by=tuple(self.rid(n) for n in unchecked),
            )

        names = self.bundle(direction)
        spec = self.direction_spec(direction)
        if not (self.at_node(spec.wait_node, pose) or self.at_node(spec.align_node, pose)):
            return AcquireResult(
                granted=False,
                reason=ReasonCode.PRECONDITION_NOT_REACHED,
                blocked_by=(self.rid(names[0]),),
                detail=(
                    f"not at a legal asking point: {self.node_distance_text(pose, [spec.wait_node, spec.align_node])}. "
                    f"Tolerance {self.cfg.node_reach_tolerance_m:.2f} m is set from the "
                    "stack's measured arrival error (see tests/test_p4_traffic.py), "
                    "not from convenience."
                ),
            )

        # Being *near* the node is not enough: the footprint must also clear every
        # rectangle. A robot whose tail is still in the gap is not waiting.
        region = self.full_region()
        corners = footprint_corners(pose[0], pose[1], pose[2], length_m, width_m)
        overlap = region.box_overlap(corners)
        if overlap:
            return AcquireResult(
                granted=False,
                reason=ReasonCode.PRECONDITION_NOT_REACHED,
                blocked_by=tuple(r.name for r in overlap),
            )

        res = self.book.acquire(
            task_id=task_id,
            robot_id=robot_id,
            resources=tuple(self.rid(n) for n in names),
            now=now,
            wall_now=wall_now,
            boot_id=boot_id,
            revision=revision,
            generation=generation,
            epoch=epoch,
            permit_id=permit_id,
            robot_position_known=localization_valid,
        )
        if res.granted:
            self._granted_bundle[task_id] = names
            self._granted_expiry[task_id] = res.permit.expires_at
            for n in names:
                self._state[n] = ResourceState.RESERVED
                self._block_reason.pop(n, None)
        return res

    # ------------------------------------------------------------------ #
    # occupancy transitions
    # ------------------------------------------------------------------ #

    def enter(
        self,
        direction: str,
        *,
        task_id: str,
        boot_id: str,
        revision: int,
        generation: int,
        epoch: int,
        wall_now: float,
        pose: tuple[float, float, float],
        length_m: float,
        width_m: float,
    ) -> bool:
        """RESERVED -> OCCUPIED. Called once the footprint is really inside.

        Refuses if it is not: marking OCCUPIED from the waiting point would make
        the resource look busy while still available, and the exit buffer would
        then be reported as occupied by nobody.
        """
        names = self.bundle(direction)
        if not set(names) <= set(
            self.held_resources(
                task_id=task_id, boot_id=boot_id, revision=revision,
                generation=generation, epoch=epoch, wall_now=wall_now,
            )
        ):
            return False
        region = self.region_of(names, margin_m=0.0)
        corners = footprint_corners(pose[0], pose[1], pose[2], length_m, width_m)
        if not region.box_overlap(corners):
            return False
        for n in names:
            self._state[n] = ResourceState.OCCUPIED
        return True

    def observe(self, pose: tuple[float, float, float], *, length_m: float, width_m: float) -> None:
        """Record whether *somebody* is inside. Feeds the clearance requirement.

        Returns nothing on purpose: believing "I saw nobody" is not evidence that
        the area is clear, and a value here would invite exactly that shortcut.
        """
        for name in self.cfg.resources:
            region = self.region_of((name,), margin_m=0.0)
            self._last_inside[name] = bool(
                region.box_overlap(footprint_corners(pose[0], pose[1], pose[2], length_m, width_m))
            )

    def inside_now(self, name: str) -> bool | None:
        """Last observation only. ``None`` means we have never looked."""
        return self._last_inside.get(name)

    def verified_clear(
        self,
        poses: dict[str, tuple[float, float, float] | None],
        sizes: dict[str, tuple[float, float]],
        *,
        only: tuple[str, ...] | None = None,
    ) -> ClearanceScan:
        """Which resources a complete observation proves empty.

        The single implementation of "verified clear" in this project: the start-up
        occupancy sweep and the continuous re-verification pass both call it, so the
        two can never disagree about what counts as evidence.

        A resource lands in ``freeable`` only when all three hold:

        * **nobody holds it.** A live permit authorises somebody to be inside, so
          poses alone cannot rule its occupancy out. Held resources are reported in
          ``held`` and are never freeable.
        * **every configured robot supplied a pose.** A robot missing from ``poses``,
          or present with ``None``, counts as unlocatable. When anything is
          unlocatable, ``freeable`` is empty.
        * **no supplied footprint overlaps the rectangle.**

        ``only`` restricts which resources are candidates; the robot set is always
        every configured robot, because an observation that skipped one of them would
        not be an observation.
        """
        names = tuple(only) if only is not None else tuple(self.cfg.resources)
        unlocatable = tuple(sorted(r for r in sizes if poses.get(r) is None))
        if unlocatable:
            return ClearanceScan(unlocatable=unlocatable)

        freeable: list[str] = []
        occupied: list[tuple[str, tuple[str, ...]]] = []
        held: list[str] = []
        for name in names:
            if self.book.holders(self.rid(name)):
                held.append(name)
                continue
            region = self.region_of((name,), margin_m=0.0)
            inside: list[str] = []
            for robot in sorted(sizes):
                pose = poses[robot]
                length_m, width_m = sizes[robot]
                corners = footprint_corners(pose[0], pose[1], pose[2], length_m, width_m)
                if region.box_overlap(corners):
                    inside.append(robot)
            if inside:
                occupied.append((name, tuple(inside)))
            else:
                freeable.append(name)
        return ClearanceScan(
            freeable=tuple(freeable),
            occupied=tuple(occupied),
            held=tuple(held),
        )

    # ------------------------------------------------------------------ #
    # renew / expiry
    # ------------------------------------------------------------------ #

    def renew(
        self,
        direction: str,
        *,
        task_id: str,
        boot_id: str,
        revision: int,
        generation: int,
        epoch: int,
        wall_now: float,
    ) -> Permit:
        """Extend the lease. Never resurrects an expired one.

        If the permit already lapsed the resource is UNKNOWN, and renewing would
        launder an unauthorised occupancy into a legal one, so it is refused.
        """
        names = self.bundle(direction)
        blocked = [n for n in names if self.state_of(n) in (ResourceState.UNKNOWN, ResourceState.BLOCKED)]
        if blocked:
            raise StaleCommandError(
                f"renew {direction}: {blocked} is {self.state_of(blocked[0]).value}; "
                "a lapsed permit must be re-acquired, not extended"
            )
        permit_id = self._permit_id_for(task_id, names)
        if permit_id is None:
            raise KeyError(f"renew {direction}: {task_id} holds nothing in {list(names)}")
        return self.book.renew(
            permit_id, wall_now=wall_now, boot_id=boot_id,
            revision=revision, generation=generation, epoch=epoch,
        )

    def _permit_id_for(self, task_id: str, names: tuple[str, ...]) -> str | None:
        for hold in self.book.holders(self.rid(names[0])):
            if hold.permit.task_id == task_id:
                return hold.permit.permit_id
        return None

    def expire_due(self, *, wall_now: float) -> tuple[str, ...]:
        """Lapse every overdue permit and drop the resource to UNKNOWN.

        Deliberately frees nothing. The area stays blocked until a verified
        clearance arrives (rule 2 at the top of this file).
        """
        expired = self.book.expire_due(wall_now=wall_now)
        touched: list[str] = []
        for permit in expired:
            for name, spec in self.cfg.resources.items():
                # A bundle shares one permit object across its resources, so the
                # ledger reports it once per resource. Dedupe by name.
                if name in touched:
                    continue
                if ResourceId(spec.kind, spec.name) in permit.resources:
                    self._state[name] = ResourceState.UNKNOWN
                    self._block_reason[name] = "permit expired without clearance"
                    touched.append(name)
        return tuple(touched)

    def block(self, name: str, reason: str) -> None:
        if not reason:
            raise ValueError("block requires a reason")
        self._state[name] = ResourceState.BLOCKED
        self._block_reason[name] = reason

    def mark_verified_free(self, name: str, *, source: str) -> None:
        """Promote a resource to FREE against an explicit argument.

        ``source`` is required and must be non-empty on purpose: the only way to
        reach FREE is to supply *why* the area is believed empty. A no-argument
        setter would become the standard way to clear a stuck corridor.

        It also clears the lease book's UNKNOWN mark, because a resource this manager
        calls FREE while the book still calls it unknown is not a resource that can be
        granted -- it is two answers to one question.
        """
        if not source:
            raise ValueError("mark_verified_free requires a source")
        if self.state_of(name) is ResourceState.OCCUPIED:
            raise RuntimeError(
                f"{name}: cannot mark FREE while OCCUPIED; wait for a clearance"
            )
        # Clear the lease book's UNKNOWN mark as well as this manager's state.
        # Leaving it set is not a harmless inconsistency: ResourceBook.acquire refuses
        # any resource still marked unknown, so a "verified free" resource would keep
        # refusing passage while reading as FREE -- two truths about one fact, which is
        # exactly the failure this module exists to prevent. The start-up sweep never
        # noticed, because at that point nothing has been marked unknown yet.
        self.book.confirm_clear(resources=(self.rid(name),), proof=source)
        self._state[name] = ResourceState.FREE
        self._block_reason.pop(name, None)

    def adopt_permit(
        self,
        *,
        permit_id: str,
        task_id: str,
        robot_id: str,
        direction: str,
        boot_id: str,
        revision: int,
        generation: int,
        epoch: int,
        expires_at_wall: float,
    ) -> None:
        """Install a mirrored permit granted by the coordinator.

        The robot does not mint permits; it only needs to know what it was given so
        its own gate can run the same geometry. ``expires_at_wall`` is a *receiver
        side* wall time: the coordinator's raw monotonic reading is meaningless in
        this process, and CONTRACTS section 7 forbids comparing them.
        """
        if direction not in self.cfg.directions:
            raise ValueError(f"adopt_permit: unknown direction {direction!r}")
        names = self.bundle(direction)
        permit = Permit(
            permit_id=permit_id,
            task_id=task_id,
            robot_id=robot_id,
            resources=tuple(self.rid(n) for n in names),
            boot_id=boot_id,
            revision=revision,
            generation=generation,
            epoch=epoch,
            granted_at=0.0,
            expires_at=expires_at_wall,
        )
        self.book.install(permit)
        self._granted_bundle[task_id] = names
        self._granted_expiry[task_id] = expires_at_wall
        for n in names:
            self._state[n] = ResourceState.RESERVED
            self._block_reason.pop(n, None)

    # ------------------------------------------------------------------ #
    # clearance
    # ------------------------------------------------------------------ #

    def confirm_clear(
        self,
        direction: str,
        *,
        task_id: str,
        generation: int,
        pose: tuple[float, float, float],
        length_m: float,
        width_m: float,
        source: str = "manager_geometry",
    ) -> ClearanceOutcome:
        """Release the bundle -- but only against verified geometry.

        The caller supplies a pose, not a verdict. The manager tests the whole
        footprint plus margin against every rectangle in the bundle and refuses
        if any part is still inside. Standing on the exit buffer is the classic
        mistake this rejects: the buffer is inside the bundle by design.
        """
        names = self.bundle(direction)
        region = self.clearance_region(names)
        reserve = self.clearance_reserve_m()
        corners = footprint_corners(pose[0], pose[1], pose[2], length_m, width_m)
        hits = region.box_overlap(corners)
        if hits:
            for n in names:
                if self.state_of(n) not in (ResourceState.UNKNOWN, ResourceState.BLOCKED):
                    self._state[n] = ResourceState.CLEARING
            return ClearanceOutcome(
                cleared=False,
                reason=ReasonCode.CLEARANCE_NOT_PROVEN,
                still_inside=tuple(h.name for h in hits),
                detail=(
                    f"footprint plus stopping reserve {reserve:.3f} m (stopping envelope "
                    f"{self.cfg.stop.worst_case_envelope_m():.3f} m at the fastest measured "
                    f"speed {self.cfg.stop.max_measured_speed_mps:.2f} m/s + localization "
                    f"margin {self.cfg.stop.localization_margin_m:.3f} m) plus footprint "
                    f"margin {self.cfg.footprint_margin_m:.2f} m still overlaps "
                    f"{[h.name for h in hits]}; this is the SAME boundary the gate refuses "
                    "against, so a robot parked on the exit buffer has not left the bundle -- "
                    "and one that clears here cannot be refused there"
                ),
            )

        if self.state_of(names[0]) in (ResourceState.UNKNOWN, ResourceState.BLOCKED):
            return ClearanceOutcome(
                cleared=False,
                reason=ReasonCode.RESOURCE_UNKNOWN,
                detail=(
                    f"resource is {self.state_of(names[0]).value}: "
                    f"{self._block_reason.get(names[0], 'unknown')}; "
                    "an explicit clearance after inspection is required"
                ),
            )

        released = self.book.release(task_id=task_id, expected_generation=generation)
        if not released:
            return ClearanceOutcome(
                cleared=False,
                reason=ReasonCode.PERMIT_STALE,
                detail=f"{task_id} generation {generation} holds nothing in {list(names)}",
            )
        self.book.confirm_clear(
            resources=tuple(self.rid(n) for n in names),
            proof=f"{source}: (x={pose[0]:.3f}, y={pose[1]:.3f}, yaw={pose[2]:.3f}) outside every rectangle",
        )
        for n in names:
            self._state[n] = ResourceState.FREE
            self._block_reason.pop(n, None)
        # The leg is over. Forgetting the grant here means a later incursion by
        # the same task is reported as 'no permit' rather than as a permit that
        # quietly ran out -- they are different failures.
        self._granted_bundle.pop(task_id, None)
        self._granted_expiry.pop(task_id, None)
        return ClearanceOutcome(cleared=True, reason=ReasonCode.OK, detail=f"released {list(names)}")

    # ------------------------------------------------------------------ #
    # convenience
    # ------------------------------------------------------------------ #

    def wait_node_for(self, direction: str) -> NodePose:
        return self.node(self.direction_spec(direction).wait_node)

    def release_node_for(self, direction: str) -> NodePose:
        return self.node(self.direction_spec(direction).release_node)

    def stop_model(self) -> StopModel:
        return self.cfg.stop
