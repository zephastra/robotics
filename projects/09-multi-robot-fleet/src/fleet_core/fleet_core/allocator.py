"""Allocator: choose which robot runs a task.

Deterministic on purpose. Every input is either a config value or an observed
state value, and every comparison ends in a total order, so two runs with the
same inputs pick the same robot. That is what makes a failure reproducible.

Cost model (CONTRACTS): travel distance at the robot's own speed, plus a
battery penalty so a nearly-empty robot is not chosen just because it is close.
Battery is a *hard* filter before it is ever a soft cost: a robot that cannot
finish the trip is not a candidate, however near it is.

Tie-break order, applied only when costs are equal within `tie_epsilon`:
  1. most remaining battery (finish-ability beats cleverness)
  2. robot_id ascending (stable, arbitrary but fixed)
Waiting tasks age: each second of waiting subtracts from the effective cost so a
task cannot be starved by a stream of fresher ones.
"""

from __future__ import annotations

from dataclasses import dataclass

from .domain import (
    Allocation,
    Capability,
    FleetConfig,
    ReasonCode,
    RobotState,
    Task,
)

TIE_EPSILON = 1e-9

#: Which refusal a task reports when different robots were refused for different reasons.
#:
#: Most actionable first. "There is a robot and it is occupied" tells an operator to wait;
#: "the robots that could take it need charging" explains an idle-looking fleet; "we cannot
#: say where anybody is" points at localization; and "nobody here has the capability" is
#: the only one that needs a human to change something.
#:
#: `NO_CAPABLE_ROBOT` is LAST on purpose, and that ordering is the whole of D-P5-20. The
#: old code reported it whenever no candidate survived -- including when the only reason
#: was RESOURCE_BUSY -- so a capable robot doing other work was reported as a robot that
#: does not exist. Declared here rather than left inside an if-chain so a test can pin the
#: order instead of re-deriving it from behaviour.
REASON_PRECEDENCE: tuple[ReasonCode, ...] = (
    ReasonCode.RESOURCE_BUSY,
    ReasonCode.INSUFFICIENT_BATTERY,
    ReasonCode.RESOURCE_UNKNOWN,
    ReasonCode.NO_CAPABLE_ROBOT,
)


def reported_reason(rejected: "list[tuple[str, ReasonCode]]") -> ReasonCode:
    """The single refusal to report for a task every robot refused.

    Falls back to `NO_CAPABLE_ROBOT` only when `rejected` is empty or names no known
    code, which is the conservative answer: it says "nobody can take this", not "wait".
    """
    for code in REASON_PRECEDENCE:
        if any(why is code for _, why in rejected):
            return code
    return ReasonCode.NO_CAPABLE_ROBOT

#: Which refusal a task reports when different robots were refused for different reasons.
#:
#: Most actionable first. "There is a robot and it is occupied" tells an operator to wait;
#: "the robots that could take it need charging" explains an idle-looking fleet; "we cannot
#: say where anybody is" points at localization; and "nobody here has the capability" is
#: the only one that needs a human to change something.
#:
#: `NO_CAPABLE_ROBOT` is LAST on purpose, and that ordering is the whole of D-P5-20. The
#: old code reported it whenever no candidate survived -- including when the only reason
#: was RESOURCE_BUSY -- so a capable robot doing other work was reported as a robot that
#: does not exist. Declared here rather than left inside an if-chain so a test can pin the
#: order instead of re-deriving it from behaviour.
REASON_PRECEDENCE: tuple[ReasonCode, ...] = (
    ReasonCode.RESOURCE_BUSY,
    ReasonCode.INSUFFICIENT_BATTERY,
    ReasonCode.RESOURCE_UNKNOWN,
    ReasonCode.NO_CAPABLE_ROBOT,
)


def reported_reason(rejected: "list[tuple[str, ReasonCode]]") -> ReasonCode:
    """The single refusal to report for a task every robot refused.

    Falls back to `NO_CAPABLE_ROBOT` only when `rejected` is empty or names no known
    code, which is the conservative answer: it says "nobody can take this", not "wait".
    """
    for code in REASON_PRECEDENCE:
        if any(why is code for _, why in rejected):
            return code
    return ReasonCode.NO_CAPABLE_ROBOT


@dataclass
class Candidate:
    robot_id: str
    cost: float
    distance_m: float
    battery_after_wh: float
    eta_s: float


class Allocator:
    def __init__(self, cfg: FleetConfig) -> None:
        self.cfg = cfg
        self.aging_per_s: float = 0.05

    # ------------------------------------------------------------------ #

    def estimate(self, task: Task, robot: RobotState, *, now: float) -> Candidate:
        spec = self.cfg.robots[robot.robot_id]
        pick = self.cfg.stations[task.spec.pick_station]
        drop = self.cfg.stations[task.spec.drop_station]

        leg1 = robot.distance_to(pick.x, pick.y)
        leg2 = ((pick.x - drop.x) ** 2 + (pick.y - drop.y) ** 2) ** 0.5
        total_m = leg1 + leg2

        travel_wh = total_m * spec.discharge_wh_per_m
        idle_s = 2.0 * (leg1 + leg2) / max(spec.max_speed_mps, 1e-6)
        idle_wh = idle_s * spec.discharge_wh_per_s
        battery_after = robot.battery_wh - travel_wh - idle_wh

        eta_s = total_m / max(spec.max_speed_mps, 1e-6)
        # A nearly-empty robot costs more than a far one: finishing matters more
        # than arriving early.
        battery_penalty = 0.0
        if battery_after > 0:
            battery_penalty = 1.0 / max(battery_after, 1e-6)
        cost = total_m + 10.0 * battery_penalty

        waited = max(0.0, now - task.created_at)
        cost -= self.aging_per_s * waited

        return Candidate(
            robot_id=robot.robot_id,
            cost=cost,
            distance_m=total_m,
            battery_after_wh=battery_after,
            eta_s=eta_s,
        )

    def candidates(
        self, task: Task, robots: dict[str, RobotState], *, now: float
    ) -> tuple[list[Candidate], list[tuple[str, ReasonCode]]]:
        ok: list[Candidate] = []
        rejected: list[tuple[str, ReasonCode]] = []

        for rid in sorted(robots):
            rs = robots[rid]
            if not rs.online:
                # Not NO_CAPABLE_ROBOT. An offline robot has the capability; what it
                # does not have is a working link, and reporting the capability as the
                # reason sends an operator to the wrong table. This is the same defect as
                # D-P5-20 through a different input, so it is fixed with it.
                rejected.append((rid, ReasonCode.RESOURCE_UNKNOWN))
                continue
            if not rs.position_known:
                rejected.append((rid, ReasonCode.RESOURCE_UNKNOWN))
                continue
            spec = self.cfg.robots[rid]
            if task.spec.required_capability not in spec.capabilities:
                rejected.append((rid, ReasonCode.NO_CAPABLE_ROBOT))
                continue
            if rs.executing is not None:
                rejected.append((rid, ReasonCode.RESOURCE_BUSY))
                continue
            cand = self.estimate(task, rs, now=now)
            if cand.battery_after_wh < self.cfg.battery_reserve_wh:
                rejected.append((rid, ReasonCode.INSUFFICIENT_BATTERY))
                continue
            ok.append(cand)

        ok.sort(key=lambda c: (round(c.cost, 12), -c.battery_after_wh, c.robot_id))
        return ok, rejected

    def allocate(
        self, task: Task, robots: dict[str, RobotState], *, now: float
    ) -> Allocation:
        ok, rejected = self.candidates(task, robots, now=now)
        if not ok:
            return Allocation(
                task_id=task.task_id,
                robot_id="",
                cost=float("inf"),
                reason=reported_reason(rejected),
                candidates=(),
                rejected=tuple(rejected),
                detail=", ".join(f"{rid}:{why.value}" for rid, why in rejected)
                or "no robots were offered",
            )

        # Only treat candidates as tied when they are genuinely indistinguishable;
        # TIE_EPSILON keeps float noise from silently reordering the fleet.
        best = ok[0]
        tied = [c for c in ok if abs(c.cost - best.cost) <= TIE_EPSILON]
        tied.sort(key=lambda c: (-c.battery_after_wh, c.robot_id))
        chosen = tied[0]

        return Allocation(
            task_id=task.task_id,
            robot_id=chosen.robot_id,
            cost=chosen.cost,
            reason=ReasonCode.OK,
            candidates=tuple((c.robot_id, c.cost) for c in ok),
            # Carried on success too: "who was considered and why not" is what makes a
            # choice auditable, and an auditor cannot reconstruct it from the winner.
            rejected=tuple(rejected),
        )

    # ------------------------------------------------------------------ #

    def low_battery_robots(
        self, robots: dict[str, RobotState], *, threshold_wh: float | None = None
    ) -> list[str]:
        limit = self.cfg.low_battery_wh if threshold_wh is None else threshold_wh
        return sorted(r for r, s in robots.items() if s.online and s.battery_wh <= limit)

    def has_capability(self, robot_id: str, capability: Capability) -> bool:
        spec = self.cfg.robots.get(robot_id)
        return bool(spec and capability in spec.capabilities)
