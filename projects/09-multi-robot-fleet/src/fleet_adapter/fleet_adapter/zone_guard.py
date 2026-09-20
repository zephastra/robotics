"""Adapts fleet_core's CrossingManager to the safety gate's question.

The gate used to be told ``wants_protected_zone=True/False`` by whoever was
driving it. That flag is a **claim by the thing being supervised**, and a claim is
the one input a safety layer cannot accept. Forget to set it, or set it from a
route segment that no longer matches where the robot actually is, and the gate
opens -- silently, with every test still green.

``CrossingZoneGuard`` replaces the flag with a measurement. It answers the gate
from the robot's estimated pose, heading and commanded twist, using the same
``CrossingManager.check_movement`` that P4.1 already pins with 56 tests. The gate
no longer needs to know what a corridor is; it only needs someone it can ask.

It holds the robot's **identity** -- task, boot, revision, generation, epoch --
because a permit is scoped to those and not merely to a robot. ``reassign()`` is
how a new leg takes over. Without it, a superseded generation would keep its
authority, which is the failure CONTRACTS section 4 exists to prevent.

Nothing here imports ROS. The ROS node builds one of these per robot and hands it
to the gate; the tests build one directly.
"""

from __future__ import annotations

from dataclasses import dataclass

from fleet_core import CrossingManager, ReasonCode

from .safety_gate import GateReason, ZoneVerdict

# ReasonCode -> GateReason. Anything not listed maps to NO_PERMIT on purpose:
# an unrecognised refusal must never become an allowance.
_REASON_MAP: dict[ReasonCode, GateReason] = {
    ReasonCode.OK: GateReason.OK,
    ReasonCode.ROUTE_REQUIRES_PERMIT: GateReason.NO_PERMIT,
    ReasonCode.RESOURCE_BUSY: GateReason.NO_PERMIT,
    ReasonCode.PRECONDITION_NOT_REACHED: GateReason.NO_PERMIT,
    ReasonCode.PERMIT_EXPIRED: GateReason.PERMIT_EXPIRED,
    ReasonCode.PERMIT_STALE: GateReason.PERMIT_STALE,
    ReasonCode.RESOURCE_UNKNOWN: GateReason.RESOURCE_UNKNOWN,
    ReasonCode.RESOURCE_TIMEOUT: GateReason.RESOURCE_UNKNOWN,
    ReasonCode.CLEARANCE_NOT_PROVEN: GateReason.RESOURCE_UNKNOWN,
    ReasonCode.LOCALIZATION_STALE: GateReason.POSITION_UNKNOWN,
}


def map_reason(code: ReasonCode) -> GateReason:
    return _REASON_MAP.get(code, GateReason.NO_PERMIT)


@dataclass
class CrossingZoneGuard:
    """A callable the gate can ask: "may this robot move, right now?"

    Position and twist are *arguments*, not stored state: the guard must be
    re-evaluated against wherever the robot is now, and a cached pose would turn
    a moving robot into a stationary one.
    """

    manager: CrossingManager
    robot_id: str
    length_m: float
    width_m: float
    task_id: str = ""
    boot_id: str = ""
    revision: int = 0
    generation: int = 0
    epoch: int = 0

    def reassign(
        self,
        *,
        task_id: str,
        boot_id: str,
        revision: int,
        generation: int,
        epoch: int,
    ) -> None:
        """Point the guard at a new leg.

        Called when a command supersedes the previous one, and on adapter restart
        (new ``boot_id``). Deliberately not automatic: a guard that silently
        adopted whatever the last status said would follow a stale generation
        wherever it led.
        """
        self.task_id = task_id
        self.boot_id = boot_id
        self.revision = revision
        self.generation = generation
        self.epoch = epoch

    def __call__(
        self,
        *,
        wall_now: float,
        pose: tuple[float, float, float],
        twist: tuple[float, float, float],
        localization_valid: bool,
    ) -> ZoneVerdict:
        check = self.manager.check_movement(
            task_id=self.task_id,
            boot_id=self.boot_id,
            revision=self.revision,
            generation=self.generation,
            epoch=self.epoch,
            wall_now=wall_now,
            pose=pose,
            twist=twist,
            length_m=self.length_m,
            width_m=self.width_m,
            localization_valid=localization_valid,
        )
        return ZoneVerdict(
            allowed=check.allowed,
            reason=map_reason(check.reason),
            hit=tuple(check.stop_boundary_hit),
            detail=check.detail,
        )
