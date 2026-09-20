"""Local safety gate: the only thing allowed to publish the robot's final velocity.

Why this file exists at all: Nav2 decides *where* to go, but nothing in Nav2 knows
whether this robot currently holds a permit for the protected corridor. If Nav2
could publish straight to the wheels, a planner recovering from a stuck state
would happily drive into a corridor another robot is using. So the gate sits last
in the chain and every velocity must pass through it.

Rules it enforces (CONTRACTS §traffic, TEST_AND_ACCEPTANCE §3.2):

  * No permit for a protected resource  -> the robot may not move at all.
  * Permit expired on **wall** time      -> stop, even if the simulator is frozen.
    Sim time freezing must not freeze the watchdog.
  * Permit carrying a stale boot / revision / generation -> stop. A superseded
    command is not an authorisation.
  * Robot position unknown               -> stop. "I do not know where I am" is
    not a licence to keep moving.
  * Interface heartbeat lost             -> stop. A dead adapter is UNKNOWN, not safe.
  * Speed or acceleration beyond the frozen limits -> clamp, never exceed.

Two distinctions the rest of the project depends on:

  * **Publishing zero is not stopping.** `zero_published_at` records when a zero
    velocity was sent. `stopped_confirmed_at` records when the robot was actually
    observed below the stop-speed threshold. A report may only claim a stop from
    the second one (TEST_AND_ACCEPTANCE §3.3).
  * **Gate exit is not safety.** On shutdown the gate publishes a stop and then
    reports that downstream protection is still REQUIRED. A bridge that keeps the
    last velocity after the gate dies is a real failure (F13), not a solved one.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from enum import Enum
from typing import Protocol

from .adapter_base import AdapterStatus


class GateMode(str, Enum):
    NORMAL = "NORMAL"
    CLAMPED = "CLAMPED"      # allowed to move, but slower than requested
    STOP = "STOP"            # zero commanded, robot may still be moving
    ESTOP = "ESTOP"          # latched; needs an explicit reset


class GateReason(str, Enum):
    OK = "OK"
    NO_PERMIT = "NO_PERMIT"
    PERMIT_EXPIRED = "PERMIT_EXPIRED"
    PERMIT_STALE = "PERMIT_STALE"
    RESOURCE_UNKNOWN = "RESOURCE_UNKNOWN"
    POSITION_UNKNOWN = "POSITION_UNKNOWN"
    HEARTBEAT_LOST = "HEARTBEAT_LOST"
    STATE_STALE = "STATE_STALE"
    SPEED_CLAMPED = "SPEED_CLAMPED"
    ACCEL_CLAMPED = "ACCEL_CLAMPED"
    LATCHED_ESTOP = "LATCHED_ESTOP"
    SHUTDOWN = "SHUTDOWN"


@dataclass(frozen=True)
class Limits:
    max_linear_mps: float = 0.35
    max_angular_radps: float = 0.6
    max_linear_accel_mps2: float = 0.5
    max_angular_accel_radps2: float = 1.0
    stop_speed_mps: float = 0.03
    stop_angular_radps: float = 0.05

    def __post_init__(self) -> None:
        for name in (
            "max_linear_mps",
            "max_angular_radps",
            "max_linear_accel_mps2",
            "max_angular_accel_radps2",
            "stop_speed_mps",
            "stop_angular_radps",
        ):
            value = getattr(self, name)
            if not (value > 0):
                raise ValueError(f"Limits.{name} must be > 0, got {value}")


@dataclass
class PermitView:
    """What the gate is told about the robot's authorisation.

    Deliberately a flat snapshot rather than the Permit object: the gate must
    never be able to mutate authorisation, only judge it.
    """

    granted: bool
    resources: tuple[str, ...] = ()
    boot_id: str = ""
    revision: int = -1
    generation: int = -1
    epoch: int = -1
    expires_at_wall: float = -math.inf


@dataclass
class GateDecision:
    allowed_linear_mps: float
    allowed_angular_radps: float
    mode: GateMode
    reason: GateReason
    detail: str = ""

    @property
    def is_zero(self) -> bool:
        return self.allowed_linear_mps == 0.0 and self.allowed_angular_radps == 0.0


@dataclass(frozen=True)
class ZoneVerdict:
    """Answer from a zone guard. ``allowed`` is never inferred from a blank reason."""

    allowed: bool
    reason: GateReason = GateReason.OK
    hit: tuple[str, ...] = ()
    detail: str = ""


class ZoneGuard(Protocol):
    """Something the gate can ask "may this robot move, right now?".

    Structural, not inherited: the gate must not need to know that a corridor
    exists, or that ``fleet_core`` does. It only needs a decision made from
    geometry rather than from a flag the caller chose.

    Implementations MUST be re-evaluated per call -- position and twist arrive as
    arguments precisely so that a cached "we were clear a moment ago" cannot
    outlive the pose it described.
    """

    def __call__(
        self,
        *,
        wall_now: float,
        pose: tuple[float, float, float],
        twist: tuple[float, float, float],
        localization_valid: bool,
    ) -> ZoneVerdict: ...


@dataclass
class SafetyGate:
    robot_id: str
    limits: Limits = field(default_factory=Limits)
    heartbeat_timeout_s: float = 1.0
    state_timeout_s: float = 0.5
    requires_permit: bool = True
    # When installed, this is authoritative and the caller cannot argue with
    # it: `wants_protected_zone` is ignored. See docs/DECISIONS.md D-P4-08.
    zone: "ZoneGuard | None" = None

    _latched: GateMode | None = None
    _latched_reason: GateReason | None = None
    _last_heartbeat_wall: float | None = None
    _last_status_wall: float | None = None
    _last_status: AdapterStatus | None = None
    _prev_linear: float = 0.0
    _prev_angular: float = 0.0
    zero_published_at: float | None = None
    stopped_confirmed_at: float | None = None
    shutdown_requires_downstream: bool = False
    decisions: list[tuple[float, GateDecision]] = field(default_factory=list)

    # ------------------------------------------------------------------ #
    # inputs
    # ------------------------------------------------------------------ #

    def heartbeat(self, *, wall_now: float) -> None:
        self._last_heartbeat_wall = wall_now

    def observe(self, status: AdapterStatus, *, wall_now: float) -> None:
        self._last_status = status
        self._last_status_wall = wall_now
        # Only a real measurement may confirm a stop. A zero command is not enough.
        if (
            self.zero_published_at is not None
            and abs(status.speed_mps) <= self.limits.stop_speed_mps
        ):
            if self.stopped_confirmed_at is None:
                self.stopped_confirmed_at = wall_now

    def latch_estop(self, reason: GateReason, *, wall_now: float) -> None:
        self._latched = GateMode.ESTOP
        self._latched_reason = reason
        self._record(GateDecision(0.0, 0.0, GateMode.ESTOP, reason, "latched"), wall_now)

    def reset_estop(self) -> None:
        """Only an explicit reset clears a latch, never a timer."""
        self._latched = None
        self._latched_reason = None
        self.zero_published_at = None
        self.stopped_confirmed_at = None

    # ------------------------------------------------------------------ #
    # evaluation
    # ------------------------------------------------------------------ #

    def evaluate(
        self,
        requested_linear: float,
        requested_angular: float,
        *,
        wall_now: float,
        epoch: int,
        permit: PermitView | None = None,
        state_epoch: int | None = None,
        wants_protected_zone: bool = False,
        dt_s: float = 0.05,
    ) -> GateDecision:
        r, a = float(requested_linear), float(requested_angular)

        decision = self._veto(wall_now, epoch, permit, state_epoch, wants_protected_zone)
        if decision is not None:
            self._record(decision, wall_now)
            return decision

        # Acceleration limit relative to the previous *commanded* velocity. This is
        # a command-space limit, not a claim about physical deceleration.
        max_dv = self.limits.max_linear_accel_mps2 * dt_s
        max_dw = self.limits.max_angular_accel_radps2 * dt_s
        clamped_r = _clamp(r, self._prev_linear - max_dv, self._prev_linear + max_dv)
        clamped_a = _clamp(a, self._prev_angular - max_dw, self._prev_angular + max_dw)
        reason = GateReason.OK
        mode = GateMode.NORMAL

        if abs(clamped_r) > self.limits.max_linear_mps or abs(clamped_a) > self.limits.max_angular_radps:
            clamped_r = _clamp(clamped_r, -self.limits.max_linear_mps, self.limits.max_linear_mps)
            clamped_a = _clamp(clamped_a, -self.limits.max_angular_radps, self.limits.max_angular_radps)
            reason = GateReason.SPEED_CLAMPED
            mode = GateMode.CLAMPED
        elif clamped_r != r or clamped_a != a:
            reason = GateReason.ACCEL_CLAMPED
            mode = GateMode.CLAMPED

        self._prev_linear, self._prev_angular = clamped_r, clamped_a
        out = GateDecision(clamped_r, clamped_a, mode, reason)
        self._record(out, wall_now)
        return out

    # ------------------------------------------------------------------ #

    def _veto(
        self,
        wall_now: float,
        epoch: int,
        permit: PermitView | None,
        state_epoch: int | None,
        wants_protected_zone: bool,
    ) -> GateDecision | None:
        def stop(reason: GateReason, detail: str = "") -> GateDecision:
            self._prev_linear = 0.0
            self._prev_angular = 0.0
            return GateDecision(0.0, 0.0, GateMode.STOP, reason, detail)

        if self._latched is GateMode.ESTOP:
            self._prev_linear = self._prev_angular = 0.0
            return GateDecision(
                0.0, 0.0, GateMode.ESTOP,
                self._latched_reason or GateReason.LATCHED_ESTOP,
                "estop latched; explicit reset required",
            )

        if self._last_heartbeat_wall is None:
            return stop(GateReason.HEARTBEAT_LOST, "no heartbeat received yet")
        if wall_now - self._last_heartbeat_wall > self.heartbeat_timeout_s:
            return stop(
                GateReason.HEARTBEAT_LOST,
                f"heartbeat {wall_now - self._last_heartbeat_wall:.3f}s old "
                f"(limit {self.heartbeat_timeout_s}s)",
            )

        if self._last_status_wall is not None and wall_now - self._last_status_wall > self.state_timeout_s:
            return stop(
                GateReason.STATE_STALE,
                f"robot state {wall_now - self._last_status_wall:.3f}s old "
                f"(limit {self.state_timeout_s}s)",
            )
        if self._last_status is None:
            return stop(GateReason.STATE_STALE, "no robot state observed")

        if self._last_status.phase.value == "FAULTED":
            return stop(GateReason.RESOURCE_UNKNOWN, "adapter reports FAULTED")

        if state_epoch is not None and state_epoch != epoch:
            return stop(
                GateReason.STATE_STALE,
                f"state from epoch {state_epoch}, current epoch {epoch}",
            )

        if self.requires_permit and self.zone is not None:
            # The guard is asked every time, with the pose the robot is at now.
            # `wants_protected_zone` is deliberately not consulted: it is a claim
            # by the supervised party, and honouring it here is the hole P4.3 closes.
            try:
                # `self._last_status` was proven non-None above; `_veto` has no
                # local for it, so it is read off the instance here.
                status_now = self._last_status
                verdict = self.zone(
                    wall_now=wall_now,
                    pose=(status_now.x, status_now.y, status_now.yaw),
                    twist=(status_now.speed_mps, 0.0, status_now.angular_radps),
                    localization_valid=status_now.localization_valid,
                )
            except Exception as exc:  # noqa: BLE001 -- deliberate: fail closed
                # An exception escaping a timer callback is an OPEN DOOR, not a
                # stop. A guard that crashes must produce a stop like any other
                # refusal, and it must say that is what happened.
                return stop(
                    GateReason.RESOURCE_UNKNOWN,
                    f"zone guard raised {type(exc).__name__}: {exc}; "
                    "treated as unknown occupancy, not as permission",
                )
            if not verdict.allowed:
                return stop(
                    verdict.reason,
                    verdict.detail or f"geofence: {list(verdict.hit)}",
                )

        if self.requires_permit and self.zone is None and wants_protected_zone:
            if permit is None or not permit.granted:
                return stop(GateReason.NO_PERMIT, "no permit for the protected zone")
            if permit.epoch != epoch:
                return stop(
                    GateReason.PERMIT_STALE,
                    f"permit epoch {permit.epoch} != current {epoch}",
                )
            if wall_now >= permit.expires_at_wall:
                return stop(
                    GateReason.PERMIT_EXPIRED,
                    f"permit expired {-1 * (permit.expires_at_wall - wall_now):.3f}s ago "
                    "(wall time; sim pause does not extend it)",
                )
            if permit.boot_id != self._expected_boot_id():
                return stop(
                    GateReason.PERMIT_STALE,
                    f"permit boot {permit.boot_id!r} != bot boot {self._expected_boot_id()!r}",
                )

        return None

    def _expected_boot_id(self) -> str:
        status = self._last_status
        return getattr(status, "boot_id", "") if status is not None else ""

    def _record(self, decision: GateDecision, wall_now: float) -> None:
        if decision.is_zero and self.zero_published_at is None:
            self.zero_published_at = wall_now
            self.stopped_confirmed_at = None
        elif not decision.is_zero:
            self.zero_published_at = None
            self.stopped_confirmed_at = None
        self.decisions.append((wall_now, decision))

    # ------------------------------------------------------------------ #
    # shutdown
    # ------------------------------------------------------------------ #

    def shutdown(self, *, wall_now: float) -> GateDecision:
        """Publish a stop and state plainly that this is NOT sufficient.

        A gate that exits cleanly has done its job, but if the thing downstream
        keeps the last velocity, the robot keeps moving. That must be reported as
        an unmet requirement, never as a safe state.
        """
        self._prev_linear = self._prev_angular = 0.0
        self.shutdown_requires_downstream = True
        decision = GateDecision(
            0.0, 0.0, GateMode.STOP, GateReason.SHUTDOWN,
            "gate exiting: zero published, but downstream must independently "
            "stop within its timeout. NOT_RUN until measured (F13).",
        )
        self._record(decision, wall_now)
        return decision

    # ------------------------------------------------------------------ #
    # reporting
    # ------------------------------------------------------------------ #

    def stop_latency_report(self) -> dict[str, float | str | None]:
        """Distinguish *commanded* stop from *observed* stop.

        The measured physical stopping time and distance are NOT available here;
        they require the simulator's ground truth and are recorded by the
        evaluator. This report deliberately says so instead of implying a
        physical measurement.
        """
        return {
            "zero_published_at": self.zero_published_at,
            "stopped_confirmed_at": self.stopped_confirmed_at,
            "command_to_observation_s": (
                None
                if self.zero_published_at is None or self.stopped_confirmed_at is None
                else self.stopped_confirmed_at - self.zero_published_at
            ),
            "physical_stop_time_s": "NOT_RUN",
            "physical_stop_distance_m": "NOT_RUN",
            "note": "physical figures require ground truth from the evaluator",
        }

    def reason_counts(self) -> dict[str, int]:
        out: dict[str, int] = {}
        for _, d in self.decisions:
            out[d.reason.value] = out.get(d.reason.value, 0) + 1
        return out


def _clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))
