"""Loaded low battery: CONTRACTS section 5's second half, which nothing implemented.

THE CONTRACT, IN TWO LINES
--------------------------
CONTRACTS section 5 states the rule as two cases:

  * **not carrying cargo, low battery** -> cancel the leg, generate a charge run.
  * **carrying cargo, low battery**     -> do NOT auto-unload. Park at a legal, reachable
    position and raise NEEDS_ATTENTION.

The first half exists (`task_service_node._service_battery` creates the charge run). The second
does not, and `D-P17-02` records the three places that were checked before saying so:

  * `_service_battery` opens with `if state.executing or not self._fresh(...): continue`, so a
    robot mid-task is never even considered;
  * `nav2_adapter_node` simulates and reports the battery and takes no decision from it;
  * `gate_node` does not read the battery at all.

With no decision anywhere, a robot forced to critical mid-leg simply drives on. F05's expected
result -- 保持货物归属、合法停车 attention -- cannot happen, so the case was filed under
`low_battery_while_executing` rather than under a missing trigger.

WHY THIS MODULE IS PURE AND SEPARATE
------------------------------------
The rule has three parts that a reader has to be able to check one at a time, and burying them
in a node's timer callback makes all three invisible:

  1. **the decision** -- carrying cargo changes what low battery MEANS;
  2. **the position** -- "a legal reachable position" is a geometric claim, and an illegal
     parking spot is worse than the drive it replaced, because a robot parked in a protected
     rectangle blocks the passage for everyone;
  3. **the holding** -- the cargo's ownership must NOT be quietly re-homed. `D-P17-03` records
     that a check which only asks "was it delivered" passes on a fleet that re-homed the load,
     which is the failure this case exists to catch.

Nothing here publishes or subscribes. The caller owns actuation.
"""
from __future__ import annotations

from dataclasses import dataclass

#: The rule applies when the robot is at or below this band. Named rather than inlined because
#: F05 asserts the band, and a band spelled in two files drifts.
LOADED_PARK_BANDS = ("CRITICAL",)

#: What the fleet does about it. One value, because the contract names one outcome: park and
#: raise attention. There is deliberately no "unload first" option -- the contract forbids it,
#: and offering the option is how it would come back.
ACTION_PARK_AND_ATTEND = "park_and_attend"
ACTION_CONTINUE = "continue"
ACTION_CANCEL_AND_CHARGE = "cancel_and_charge"


@dataclass(frozen=True)
class LoadedBatteryDecision:
    """What should happen to this robot, and the sentence that says why."""

    action: str
    reason: str
    #: Set only when the action is PARK_AND_ATTEND: where it should stop.
    park_at: tuple[float, float] | None = None
    #: True when the decision was taken while the robot held cargo. Reported, not inferred:
    #: the whole difference between the two halves of the rule is this flag.
    carrying: bool = False


def decide(band: str, *, executing: bool, carrying: bool,
           has_charge_task: bool) -> LoadedBatteryDecision:
    """The rule, in one place.

    The ORDER of the two branches is the rule. Checked the other way round, a loaded robot is
    cancelled and sent to a charger with the cargo still on it, which the contract forbids --
    not because charging is unsafe, but because nobody has established that the cargo survives
    the trip, and a task that silently keeps its cargo while the fleet believes it is charging
    is a task nobody can account for.
    """
    if band not in LOADED_PARK_BANDS:
        return LoadedBatteryDecision(
            action=ACTION_CONTINUE,
            reason=f"battery band {band!r} is above the band this rule acts on",
            carrying=carrying,
        )

    # --- the loaded half comes FIRST ----------------------------------------- #
    if carrying:
        return LoadedBatteryDecision(
            action=ACTION_PARK_AND_ATTEND,
            reason=("carrying cargo on a CRITICAL battery: the contract forbids auto-unload, so "
                    "the robot stops at a legal reachable position and the task is raised for "
                    "attention. It is NOT sent to a charger, because nothing has established "
                    "that the cargo survives the trip and a task that keeps its cargo while "
                    "the fleet believes it is charging cannot be accounted for."),
            carrying=True,
        )

    # --- the unloaded half ---------------------------------------------------- #
    if not executing:
        return LoadedBatteryDecision(
            action=ACTION_CANCEL_AND_CHARGE,
            reason="idle on a CRITICAL battery: a charge run is generated",
            carrying=False,
        )
    if has_charge_task:
        return LoadedBatteryDecision(
            action=ACTION_CANCEL_AND_CHARGE,
            reason=("executing on a CRITICAL battery with a charge run already outstanding: "
                    "the leg is cancelled and the existing run is left to it"),
            carrying=False,
        )
    return LoadedBatteryDecision(
        action=ACTION_CANCEL_AND_CHARGE,
        reason="executing on a CRITICAL battery while unloaded: the leg is cancelled",
        carrying=False,
    )


#: Parking rectangles a loaded robot may NOT stop in. These are the protected rectangles from
#: `config/resources.yaml`: a robot parked in the corridor or on an exit pad occupies a
#: capacity-1 resource, and the whole fleet queues behind it.
PROTECTED_RECTS = {
    "gap":      (-1.75, -0.65, 1.75, 0.65),
    "pad_left": (-2.85, -1.75, -2.15, -1.05),
    "pad_right": (2.15, -1.75, 2.85, -1.05),
}


def legal_park_point(pose: tuple[float, float, float],
                     candidates: list[tuple[float, float]],
                     *, footprint_radius_m: float = 0.30,
                     margin_m: float = 0.15,
                     reachable: list[tuple[float, float]] | None = None) -> dict:
    """Pick a legal, reachable parking point. Returns the reason as well as the point.

    Three conditions, and the third is the one that is usually forgotten:

    * **legal** -- the footprint plus margin clears every protected rectangle. A robot parked
      in a capacity-1 resource blocks the passage for every other robot, which is worse than
      the drive it replaced.
    * **reachable** -- the robot must be able to GET there on the battery it has left. A
      parking spot the robot cannot reach is a promise it will break, which is the same fault
      the `no_safe_charger` path already refuses to make.
    * **nearest first** -- among the legal ones, the shortest drive, because the battery is
      the constraint.

    Returns a dict rather than a point: a caller that only gets coordinates has no way to
    report WHY that point, and `D-P17-03` is the record of what that costs.
    """
    legal: list[tuple[float, float, float]] = []
    for (cx, cy) in candidates:
        if _clears_every_rect(cx, cy, footprint_radius_m, margin_m, PROTECTED_RECTS):
            legal.append((cx, cy, (cx - pose[0]) ** 2 + (cy - pose[1]) ** 2))

    if not legal:
        return {
            "ok": False,
            "park_at": None,
            "reason": ("no legal parking point among the candidates: every one has the "
                       "footprint plus margin overlapping a protected rectangle, and parking "
                       "in a capacity-1 resource blocks the fleet behind it"),
            "legal_count": 0,
        }

    if reachable is not None:
        reach_set = {(round(x, 3), round(y, 3)) for x, y in reachable}
        filtered = [c for c in legal if (round(c[0], 3), round(c[1], 3)) in reach_set]
        if filtered:
            legal = filtered
        else:
            return {
                "ok": False,
                "park_at": None,
                "reason": ("legal parking points exist but none is reachable on the remaining "
                           "battery: the robot must stop where it is and report rather than be "
                           "promised a spot it cannot drive to"),
                "legal_count": len(legal),
            }

    legal.sort(key=lambda c: c[2])
    best = legal[0]
    return {
        "ok": True,
        "park_at": (round(best[0], 4), round(best[1], 4)),
        "distance_m": round(best[2] ** 0.5, 4),
        "reason": (f"nearest of {len(legal)} legal point(s), "
                   f"{best[2] ** 0.5:.2f} m away, clearing every protected rectangle by the "
                   f"footprint radius {footprint_radius_m:.2f} + margin {margin_m:.2f} m"),
        "legal_count": len(legal),
    }


def _clears_every_rect(x: float, y: float, radius_m: float, margin_m: float,
                       rects: dict[str, tuple[float, float, float, float]]) -> bool:
    need = radius_m + margin_m
    for rect in rects.values():
        x_min, y_min, x_max, y_max = rect
        # distance from the point to the rectangle; 0 inside
        dx = max(x_min - x, 0.0, x - x_max)
        dy = max(y_min - y, 0.0, y - y_max)
        if (dx * dx + dy * dy) ** 0.5 < need:
            return False
    return True


def ownership_preserved(carried_by_before: str | None,
                        carried_by_after: str | None) -> dict:
    """Did the cargo stay with the same robot?

    F05's expected result says 保持货物归属. A check that only asked "was it delivered" would
    pass on a fleet that quietly re-homed the load -- which is the failure the case exists to
    catch, and `D-P17-03` records that the runner could not decide this clause before.
    """
    same = carried_by_before == carried_by_after and carried_by_before is not None
    return {
        "preserved": same,
        "before": carried_by_before,
        "after": carried_by_after,
        "detail": ("the cargo is still held by the same robot"
                   if same else
                   f"cargo ownership moved from {carried_by_before!r} to {carried_by_after!r} "
                   f"or was lost; the contract forbids auto-unload on low battery, so a "
                   f"change here is a policy failure and not a delivery"),
    }
