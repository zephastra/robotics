"""Domain types for 009.

Every rule here encodes a constraint from AGENTS.md or CONTRACTS.md, in
particular:

  * accepted != succeeded            (TaskState separates them)
  * cancel accepted != stopped       (CancelResult carries `stopped`)
  * resource TTL expiry != proof the
    area is clear                    (Permit.expired is not `cleared`)
  * a task holding a payload cannot
    simply move to another robot     (PayloadState is owned, not re-derived)
  * stale command generations must be
    rejectable                       (Permit carries boot_id/revision/generation)
"""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
from typing import Any

from .geometry import Rect, StopModel


# --------------------------------------------------------------------------- #
# errors
# --------------------------------------------------------------------------- #


class StaleCommandError(Exception):
    """A command referred to a superseded boot, revision or generation."""


class ConflictError(Exception):
    """Same request_id, different content -- reject, do not overwrite."""


# --------------------------------------------------------------------------- #
# enums
# --------------------------------------------------------------------------- #


class TaskState(str, Enum):
    SUBMITTED = "SUBMITTED"
    ACCEPTED = "ACCEPTED"          # accepted != done
    ASSIGNED = "ASSIGNED"
    EXECUTING = "EXECUTING"
    SUCCEEDED = "SUCCEEDED"        # terminal
    FAILED = "FAILED"              # terminal
    CANCELED = "CANCELED"          # terminal
    NEEDS_ATTENTION = "NEEDS_ATTENTION"
    NOT_STARTED = "NOT_STARTED"

    @property
    def terminal(self) -> bool:
        return self in (
            TaskState.SUCCEEDED,
            TaskState.FAILED,
            TaskState.CANCELED,
        )


class PayloadState(str, Enum):
    FREE = "FREE"
    RESERVED = "RESERVED"          # claimed by a task, still in place
    HELD = "HELD"                  # in the robot's hands-equivalent (logical)
    DELIVERED = "DELIVERED"        # terminal


class ResourceKind(str, Enum):
    CORRIDOR = "CORRIDOR"          # the shared narrow passage
    EXIT_BUFFER = "EXIT_BUFFER"    # the slot just past the corridor
    STATION = "STATION"
    CHARGER = "CHARGER"


class Capability(str, Enum):
    CARRY = "CARRY"
    INSPECT = "INSPECT"


class ResourceState(str, Enum):
    """CONTRACTS section 7.

    The distinction that keeps robots out of each other's corridor is
    ``RESERVED`` vs ``OCCUPIED``: the first is a right to enter, the second is
    evidence that somebody did. And ``UNKNOWN`` is not a synonym for ``FREE`` --
    it means "we lost track and must not grant".
    """

    FREE = "FREE"              # ledger empty AND geometrically verified clear
    RESERVED = "RESERVED"      # a permit is live; entry expected, not observed
    OCCUPIED = "OCCUPIED"      # a robot's footprint is inside
    CLEARING = "CLEARING"      # leaving was attempted but not yet proven
    UNKNOWN = "UNKNOWN"        # no trustworthy occupancy knowledge -> block
    BLOCKED = "BLOCKED"        # link or operator event -> block until inspected


class ReasonCode(str, Enum):
    OK = "OK"
    INVALID_INPUT = "INVALID_INPUT"
    UNKNOWN_STATION = "UNKNOWN_STATION"
    DUPLICATE_ID = "DUPLICATE_ID"
    REQUEST_CONFLICT = "REQUEST_CONFLICT"
    NO_CAPABLE_ROBOT = "NO_CAPABLE_ROBOT"
    INSUFFICIENT_BATTERY = "INSUFFICIENT_BATTERY"
    RESOURCE_BUSY = "RESOURCE_BUSY"
    RESOURCE_UNKNOWN = "RESOURCE_UNKNOWN"
    EXIT_OCCUPIED = "EXIT_OCCUPIED"
    PERMIT_EXPIRED = "PERMIT_EXPIRED"
    PERMIT_STALE = "PERMIT_STALE"
    PAYLOAD_HELD_ELSEWHERE = "PAYLOAD_HELD_ELSEWHERE"
    # CONTRACTS section 12 lists this code and nothing emitted it. It is the one that
    # says "the task is over but the robot still has the goods", which is a different
    # statement from "someone else holds it" -- and the difference decides whether an
    # operator looks for a missing cargo or for a stuck robot.
    PAYLOAD_HELD_NEEDS_ATTENTION = "PAYLOAD_HELD_NEEDS_ATTENTION"
    CANCEL_NOT_CONFIRMED = "CANCEL_NOT_CONFIRMED"
    BLOCKED = "BLOCKED"
    PRECONDITION_NOT_REACHED = "PRECONDITION_NOT_REACHED"
    UNSUPPORTED = "UNSUPPORTED"

    # --- traffic / permits (CONTRACTS section 12) ----------------------- #
    ROUTE_REQUIRES_PERMIT = "ROUTE_REQUIRES_PERMIT"
    LOCALIZATION_STALE = "LOCALIZATION_STALE"
    CLEARANCE_NOT_PROVEN = "CLEARANCE_NOT_PROVEN"
    RESOURCE_TIMEOUT = "RESOURCE_TIMEOUT"

    # --- task layer (P5, CONTRACTS sections 2, 4 and 5) ---------------- #
    # Spelled the way the contract spells them, because a report that quotes its
    # own private vocabulary cannot be checked against the design document.
    UNSUPPORTED_PAYLOAD_MODE = "UNSUPPORTED_PAYLOAD_MODE"
    IDEMPOTENCY_CONFLICT = "IDEMPOTENCY_CONFLICT"
    WAIT_NO_ELIGIBLE_ROBOT = "WAIT_NO_ELIGIBLE_ROBOT"
    STALE_RESULT_IGNORED = "STALE_RESULT_IGNORED"
    CANCEL_UNCONFIRMED = "CANCEL_UNCONFIRMED"
    #: A task left open by a dead task service, parked at the next boot. Added
    #: because the code it used to borrow -- CANCEL_NOT_CONFIRMED -- describes
    #: something that never happened: nothing was cancelled. CONTRACTS section 12
    #: says "at least support these reason_code", so this is an addition within the
    #: contract; it is registered in docs/DECISIONS.md (D-P5-15) rather than passed
    #: off as one of the contract's own names.
    INTERRUPTED_BY_RESTART = "INTERRUPTED_BY_RESTART"
    CHARGER_BUSY = "CHARGER_BUSY"
    NO_SAFE_CHARGER = "NO_SAFE_CHARGER"
    NOT_ASSIGNED = "NOT_ASSIGNED"
    #: A task was assigned to a robot that is not executing it, and nothing in the
    #: dispatcher would ever look at it again. Added rather than reusing NAV_FAILED: the
    #: task never navigated, and 008 lost a week to a `fail_reason` column that was a
    #: constant, so a code that is merely "close enough" is worse than a new one.
    ORPHANED_ASSIGNMENT = "ORPHANED_ASSIGNMENT"
    PAYLOAD_NOT_AT_SOURCE = "PAYLOAD_NOT_AT_SOURCE"
    STATION_OCCUPIED = "STATION_OCCUPIED"
    UNKNOWN_TASK = "UNKNOWN_TASK"
    TASK_TERMINAL = "TASK_TERMINAL"
    NAV_FAILED = "NAV_FAILED"
    #: The robot's Nav2 executor had not reached ACTIVE, so the goal was refused before
    #: anything was attempted. Kept separate from NAV_FAILED on purpose: "Nav2 gave up"
    #: and "Nav2 is not up yet" need opposite responses, and the adapter's `nav_state`
    #: field exists to tell them apart -- a distinction this project has already paid for
    #: once (P6's `gate_reason` vs `nav_state`). CONTRACTS section 12 says "至少支持这些
    #: reason_code", so this is an addition inside the contract, registered as D-P5-23
    #: rather than presented as one of the contract's own names.
    NAV2_NOT_ACTIVE = "NAV2_NOT_ACTIVE"


# --------------------------------------------------------------------------- #
# task layer (P5)
# --------------------------------------------------------------------------- #


class TaskKind(str, Enum):
    """CONTRACTS section 2.

    VISIT_STATION is declared by the contract but deliberately not implemented.
    The planner refuses it with UNSUPPORTED rather than quietly treating a visit
    as a transfer that moves nothing. Declaring it here keeps the vocabulary
    faithful to the document instead of silently shrinking it.
    """

    STATION_TRANSFER = "station_transfer"
    VISIT_STATION = "visit_station"
    RETURN_TO_CHARGE = "return_to_charge"


class PayloadMode(str, Enum):
    """v1 moves payloads logically. `physical` must be refused, not downgraded."""

    LOGICAL = "logical"
    PHYSICAL = "physical"


class OperatingState(str, Enum):
    """CONTRACTS section 3.

    IDLE and CHARGING are distinct on purpose: a robot sitting on a charge pad is
    not available for work, and a report that shows it as IDLE makes the queue
    look shorter than it is.
    """

    OFFLINE = "OFFLINE"
    IDLE = "IDLE"
    EXECUTING = "EXECUTING"
    WAITING = "WAITING"
    CHARGING = "CHARGING"
    FAULT = "FAULT"
    ESTOP = "ESTOP"


class Band(str, Enum):
    """Battery band. The thresholds come from config, so a report can quote them."""

    OK = "OK"
    LOW = "LOW"
    CRITICAL = "CRITICAL"


@dataclass(frozen=True)
class BatteryConfig:
    """Simulated battery parameters. NOT measured, NOT a protection model.

    CONTRACTS section 5 fixes the fractions at 0.20 / 0.08 / 0.80 and says in as
    many words that they are demo rules. They live here rather than as literals
    so every run can state which numbers were actually in force.
    """

    charge_wh_per_s: float = 5.0
    low_fraction: float = 0.20
    critical_fraction: float = 0.08
    resume_fraction: float = 0.80
    simulated: bool = True

    def __post_init__(self) -> None:
        if not (0.0 < self.critical_fraction < self.low_fraction < self.resume_fraction <= 1.0):
            raise ValueError(
                "battery thresholds must be ordered 0 < critical < low < resume <= 1, "
                f"got {self.critical_fraction} / {self.low_fraction} / {self.resume_fraction}"
            )
        if self.charge_wh_per_s <= 0.0:
            raise ValueError("battery.charge_wh_per_s must be > 0")


# --------------------------------------------------------------------------- #
# identifiers
# --------------------------------------------------------------------------- #


@dataclass(frozen=True, order=True)
class ResourceId:
    kind: ResourceKind
    name: str

    def __str__(self) -> str:  # pragma: no cover - trivial
        return f"{self.kind.value}:{self.name}"


@dataclass(frozen=True)
class Capability2:  # placeholder kept for API symmetry
    name: str


# --------------------------------------------------------------------------- #
# specs
# --------------------------------------------------------------------------- #


@dataclass(frozen=True)
class RobotSpec:
    robot_id: str
    capabilities: frozenset[Capability]
    battery_capacity_wh: float
    discharge_wh_per_m: float
    discharge_wh_per_s: float
    max_speed_mps: float
    footprint_length_m: float
    footprint_width_m: float
    # Turning costs energy and a skid-steer robot turns a lot. Left at 0.0 by
    # default because P2 measured straight-line stop distance only; a nonzero
    # value here would be an invented number doing arithmetic in a safety check.
    turn_wh_per_rad: float = 0.0

    def __post_init__(self) -> None:
        if self.battery_capacity_wh <= 0:
            raise ValueError(f"{self.robot_id}: battery_capacity_wh must be > 0")
        if self.max_speed_mps <= 0:
            raise ValueError(f"{self.robot_id}: max_speed_mps must be > 0")
        if self.footprint_length_m <= 0 or self.footprint_width_m <= 0:
            raise ValueError(f"{self.robot_id}: footprint must be positive")


@dataclass(frozen=True)
class StationSpec:
    station_id: str
    x: float
    y: float
    yaw: float = 0.0


@dataclass(frozen=True)
class TaskSpec:
    """The immutable content of a submission. request_id dedupes by content."""

    request_id: str
    pick_station: str
    drop_station: str
    required_capability: Capability = Capability.CARRY
    payload_id: str | None = None
    # Additive and defaulted. `priority`, `max_attempts` and the two durations are
    # execution policy, not identity, so they are deliberately absent from
    # fingerprint(): two submissions differing only in retry budget are the same
    # request, and calling that a conflict would make a retried CLI call fail for
    # no reason.
    kind: TaskKind = TaskKind.STATION_TRANSFER
    payload_mode: PayloadMode = PayloadMode.LOGICAL
    priority: int = 10
    service_duration_s: float = 0.0
    max_attempts: int = 2
    timeout_sim_s: float = 180.0

    def fingerprint(self) -> str:
        return "|".join(
            [
                self.request_id,
                self.kind.value,
                self.pick_station,
                self.drop_station,
                self.required_capability.value,
                self.payload_mode.value,
                self.payload_id or "",
            ]
        )


@dataclass
class RobotState:
    robot_id: str
    x: float = 0.0
    y: float = 0.0
    battery_wh: float = 0.0
    online: bool = True
    position_known: bool = True
    boot_id: str = "boot-0"
    executing: str | None = None      # task_id currently executing
    generation: int = 0               # bumped on every new command
    # --- CONTRACTS section 3 ------------------------------------------------- #
    # All defaulted, so every existing construction site keeps working, and every
    # default is the *safe* reading: a caller that forgets to set
    # localization_valid gets True only because an unset flag must not silently
    # disable the gate -- the gate treats a missing state as absent, not as fine.
    # `sequence` starts at 0 so the first real sample always wins.
    sequence: int = 0
    stamp_sim: float = 0.0
    yaw: float = 0.0
    localization_valid: bool = True
    pose_age_s: float = 0.0
    operating_state: OperatingState = OperatingState.IDLE
    assignment_revision: int = 0
    battery_fraction: float = 1.0
    battery_valid: bool = True
    payload_id: str | None = None
    held_resources: tuple[str, ...] = ()
    permit_generation: int = 0
    nav_state: str = ""
    gate_reason: str = ""
    stopped_confirmed: bool = False
    fault_code: str = ""

    def distance_to(self, x: float, y: float) -> float:
        return ((self.x - x) ** 2 + (self.y - y) ** 2) ** 0.5


@dataclass
class Task:
    task_id: str
    spec: TaskSpec
    state: TaskState = TaskState.SUBMITTED
    robot_id: str | None = None
    revision: int = 0
    epoch: int = 0
    created_at: float = 0.0
    updated_at: float = 0.0
    reason: ReasonCode = ReasonCode.OK
    detail: str = ""
    cancel_requested: bool = False

    def is_open(self) -> bool:
        return not self.state.terminal


@dataclass
class Permit:
    """A granted right to occupy a set of resources for a bounded time."""

    permit_id: str
    task_id: str
    robot_id: str
    resources: tuple[ResourceId, ...]
    boot_id: str
    revision: int
    generation: int
    epoch: int
    granted_at: float
    expires_at: float
    cleared: bool = False          # explicit proof that the area is free

    def expired(self, now: float) -> bool:
        return now >= self.expires_at

    def matches(self, boot_id: str, revision: int, generation: int) -> bool:
        return (
            self.boot_id == boot_id
            and self.revision == revision
            and self.generation == generation
        )


@dataclass
class AcquireResult:
    granted: bool
    reason: ReasonCode
    permit: Permit | None = None
    blocked_by: tuple[ResourceId, ...] = ()
    queued_behind: tuple[str, ...] = ()
    # Human-readable reason. Optional because a refusal that only names its code
    # cannot be told apart from a different refusal with the same code, and
    # 'PRECONDITION_NOT_REACHED' covers both 'you are 0.5 m out' and 'you are
    # inside the corridor'.
    detail: str = ""


@dataclass
class CancelResult:
    accepted: bool
    reason: ReasonCode
    stopped: bool = False          # accepted does NOT imply stopped
    detail: str = ""


@dataclass
class Allocation:
    task_id: str
    robot_id: str
    cost: float
    reason: ReasonCode = ReasonCode.OK
    candidates: tuple[tuple[str, float], ...] = ()
    #: Every robot that was considered and refused, with the refusal that applied to it.
    #: `reason` answers "why is this task not assigned"; this answers "what did each robot
    #: say", and they are different questions. Without it a task refused for six causes
    #: looked like a task with one cause -- and the one reported was sometimes wrong
    #: (D-P5-20: a capable-but-busy fleet read as an incapable one).
    rejected: tuple[tuple[str, ReasonCode], ...] = ()
    detail: str = ""
    #: Every robot that was considered and refused, with the refusal that applied to it.
    #: `reason` answers "why is this task not assigned"; this answers "what did each robot
    #: say", and they are different questions. Without it a task refused for six causes
    #: looked like a task with one cause -- and the one reported was sometimes wrong
    #: (D-P5-20: a capable-but-busy fleet read as an incapable one).
    rejected: tuple[tuple[str, ReasonCode], ...] = ()
    detail: str = ""


@dataclass
class FleetConfig:
    robots: dict[str, RobotSpec] = field(default_factory=dict)
    stations: dict[str, StationSpec] = field(default_factory=dict)
    corridor: ResourceId | None = None
    exit_buffers: dict[str, ResourceId] = field(default_factory=dict)
    chargers: dict[str, ResourceId] = field(default_factory=dict)
    charger_capacity: int = 1
    permit_ttl_s: float = 30.0
    battery_reserve_wh: float = 5.0
    low_battery_wh: float = 15.0
    raw: dict[str, Any] = field(default_factory=dict)
    # The simulated energy model gets its own section rather than living under
    # `safety:`. CONTRACTS section 5 is explicit that it is a demonstration model,
    # and filing a demo model under "safety" is how it ends up being quoted as one.
    battery: BatteryConfig = field(default_factory=BatteryConfig)


# --------------------------------------------------------------------------- #
# P4 traffic: configuration and results
# --------------------------------------------------------------------------- #


@dataclass(frozen=True)
class NodePose:
    """A named pose the traffic logic may reference, in the `map` frame."""

    name: str
    x: float
    y: float
    yaw: float = 0.0

    def as_tuple(self) -> tuple[float, float, float]:
        return (self.x, self.y, self.yaw)


@dataclass(frozen=True)
class DirectionSpec:
    """Everything one direction of travel needs, resolved at load time.

    Node names are resolved here rather than at run time so a typo is a config
    error, not a robot that waits forever at a node nobody is going to grant.
    """

    direction: str
    wait_node: str
    align_node: str
    exit_node: str
    release_node: str
    entry_rect: str
    approach_axis: str = "x"
    approach_sign: float = 1.0


@dataclass(frozen=True)
class ResourceSpec:
    """One reservable area: the corridor, or one exit buffer."""

    name: str
    kind: ResourceKind
    capacity: int
    rects: tuple[Rect, ...]

    def __post_init__(self) -> None:
        if not self.rects:
            raise ValueError(f"resource {self.name!r}: needs at least one rectangle")
        if self.capacity < 1:
            raise ValueError(f"resource {self.name!r}: capacity must be >= 1")


@dataclass(frozen=True)
class TrafficConfig:
    """Validated content of config/resources.yaml."""

    resources: dict[str, ResourceSpec]
    bundles: dict[str, tuple[str, ...]]
    nodes: dict[str, NodePose]
    directions: dict[str, DirectionSpec]
    footprint_margin_m: float
    node_reach_tolerance_m: float
    permit_ttl_s: float
    renew_before_s: float
    stop: StopModel
    raw: dict[str, Any] = field(default_factory=dict)

    def __post_init__(self) -> None:
        if not self.resources:
            raise ValueError("traffic config: no resources")
        for direction, members in self.bundles.items():
            if not members:
                raise ValueError(f"bundle {direction!r}: empty")
            for m in members:
                if m not in self.resources:
                    raise ValueError(f"bundle {direction!r}: unknown resource {m!r}")
        for direction, spec in self.directions.items():
            if direction not in self.bundles:
                raise ValueError(f"directions.{direction}: no bundle declared")
            for role in ("wait_node", "align_node", "exit_node", "release_node"):
                ref = getattr(spec, role)
                if ref not in self.nodes:
                    raise ValueError(f"directions.{direction}.{role}: unknown node {ref!r}")

    def rect_named(self, name: str) -> Rect | None:
        """Look a rectangle up by its own name -- used to find the entry gate."""
        for spec in self.resources.values():
            for r in spec.rects:
                if r.name == name:
                    return r
        return None


@dataclass
class PermitCheck:
    """The gate's answer. ``allowed`` is never inferred from an absent reason."""

    allowed: bool
    reason: ReasonCode
    stop_boundary_hit: tuple[str, ...] = ()
    detail: str = ""


@dataclass
class ClearanceOutcome:
    """Result of a clearance request. ``cleared`` requires geometry, not words."""

    cleared: bool
    reason: ReasonCode
    still_inside: tuple[str, ...] = ()
    detail: str = ""
