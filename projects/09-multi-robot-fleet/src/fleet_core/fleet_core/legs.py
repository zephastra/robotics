"""Task -> leg decomposition, and the one routing question that matters at P5.

A "leg" is one drive-and-serve unit: go somewhere, optionally stay there long
enough for the site service, and report. The dispatcher issues exactly one leg at
a time per robot (CONTRACTS section 4: one live command generation), so this
module has to answer a second question as well -- *does this leg require the
shared corridor, and in which direction?* -- because a leg that crosses the gap
may not be issued like an ordinary goal.

The crossing answer is derived from ``config/resources.yaml`` rather than
hard coded: a direction is "west to east" when its waiting node is on the
negative side and its exit node is on the positive side. A hand-written table of
station pairs would drift the moment a station moves, and the drift would show up
as a robot driving through the barrier rather than as a config error.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum

from .domain import FleetConfig, PayloadMode, ReasonCode, TaskKind, TaskSpec, TrafficConfig


class LegRole(str, Enum):
    TO_PICK = "TO_PICK"
    PICK_SERVICE = "PICK_SERVICE"
    TO_DROP = "TO_DROP"
    DROP_SERVICE = "DROP_SERVICE"
    TO_CHARGER = "TO_CHARGER"
    CHARGE_SERVICE = "CHARGE_SERVICE"


@dataclass(frozen=True)
class Leg:
    leg_id: str
    index: int
    role: LegRole
    target_name: str
    x: float
    y: float
    yaw: float
    service_duration_s: float = 0.0
    #: Direction key from the traffic config when this leg needs the corridor.
    corridor_direction: str | None = None

    @property
    def crosses_corridor(self) -> bool:
        return self.corridor_direction is not None


@dataclass(frozen=True)
class PlanError:
    reason: ReasonCode
    detail: str


def corridor_direction(from_x: float, to_x: float, tcfg: TrafficConfig) -> str | None:
    """Which declared direction crosses the gap for this move, if any.

    Geometry only: the barrier runs along x = 0, so a move that changes the sign
    of x has to use the gap. Whether the robot is *allowed* to is the traffic
    layer's business, not this function's.
    """
    if from_x == 0.0 or to_x == 0.0:
        return None
    west_to_east = from_x < 0.0 < to_x
    east_to_west = to_x < 0.0 < from_x
    if not (west_to_east or east_to_west):
        return None
    for key in sorted(tcfg.directions):
        spec = tcfg.directions[key]
        wait = tcfg.nodes.get(spec.wait_node)
        exit_ = tcfg.nodes.get(spec.exit_node)
        if wait is None or exit_ is None:
            continue
        if west_to_east and wait.x < 0.0 < exit_.x:
            return key
        if east_to_west and wait.x > 0.0 > exit_.x:
            return key
    return None


def target_of(name: str, cfg: FleetConfig, tcfg: TrafficConfig | None) -> tuple[float, float, float] | None:
    """Resolve a station, charger or lane node into a pose."""
    if name in cfg.stations:
        s = cfg.stations[name]
        return (s.x, s.y, s.yaw)
    if name in cfg.chargers:
        # A charger is a pad on the floor at a known place. The traffic config is
        # the only file that carries pad coordinates for non-station places.
        for prefix in (f"pad_{name.lower()}", f"pad_charge_{name.lower()}", name):
            if tcfg is not None and prefix in tcfg.nodes:
                n = tcfg.nodes[prefix]
                return (n.x, n.y, n.yaw)
        return None
    if tcfg is not None and name in tcfg.nodes:
        n = tcfg.nodes[name]
        return (n.x, n.y, n.yaw)
    return None


def plan_legs(
    spec: TaskSpec, cfg: FleetConfig, tcfg: TrafficConfig | None = None
) -> tuple[Leg, ...] | PlanError:
    """Decompose a task into legs, or refuse with a reason.

    PHYSICAL payload mode is refused here rather than downgraded: CONTRACTS
    section 2 requires ``UNSUPPORTED_PAYLOAD_MODE`` and forbids silently doing
    the logical thing and reporting success.
    """
    if spec.payload_mode is PayloadMode.PHYSICAL:
        return PlanError(
            ReasonCode.UNSUPPORTED_PAYLOAD_MODE,
            "payload_mode=physical is not implemented in v1; refusing rather than "
            "silently downgrading to a logical transfer",
        )
    try:
        TaskKind(spec.kind)
    except ValueError:
        return PlanError(ReasonCode.UNSUPPORTED, f"unknown task kind {spec.kind!r}")

    if spec.kind is TaskKind.VISIT_STATION:
        return PlanError(
            ReasonCode.UNSUPPORTED,
            "kind=visit_station is declared in CONTRACTS but not implemented; "
            "refusing rather than treating it as a transfer",
        )

    legs: list[Leg] = []

    def add(role: LegRole, name: str, service_s: float = 0.0) -> tuple[float, float, float] | None:
        pose = target_of(name, cfg, tcfg)
        if pose is None:
            return None
        legs.append(
            Leg(
                leg_id=f"{role.value.lower()}-{len(legs)}",
                index=len(legs),
                role=role,
                target_name=name,
                x=pose[0],
                y=pose[1],
                yaw=pose[2],
                service_duration_s=service_s,
            )
        )
        return pose

    if spec.kind is TaskKind.RETURN_TO_CHARGE:
        if spec.drop_station not in cfg.chargers:
            return PlanError(
                ReasonCode.UNKNOWN_STATION,
                f"return_to_charge destination {spec.drop_station!r} is not a known "
                f"charger (known: {sorted(cfg.chargers)})",
            )
        if add(LegRole.TO_CHARGER, spec.drop_station) is None:
            return PlanError(
                ReasonCode.UNKNOWN_STATION,
                f"charger {spec.drop_station!r} has no pad coordinates in the traffic "
                "config, so no goal can be generated for it",
            )
        add(LegRole.CHARGE_SERVICE, spec.drop_station, spec.service_duration_s)
        return tuple(legs)

    # A transfer. The service legs exist only when there is something to move:
    # a payload-free task is a drive-and-report, and adding empty service steps
    # would make it look like a transfer that moved nothing.
    carries = bool(spec.payload_id)
    if add(LegRole.TO_PICK, spec.pick_station) is None:
        return PlanError(
            ReasonCode.UNKNOWN_STATION, f"station {spec.pick_station!r} has no pose"
        )
    if carries:
        add(LegRole.PICK_SERVICE, spec.pick_station, spec.service_duration_s)
    if add(LegRole.TO_DROP, spec.drop_station) is None:
        return PlanError(
            ReasonCode.UNKNOWN_STATION, f"station {spec.drop_station!r} has no pose"
        )
    if carries:
        add(LegRole.DROP_SERVICE, spec.drop_station, spec.service_duration_s)

    if tcfg is not None:
        tagged: list[Leg] = []
        prev_pose: tuple[float, float, float] | None = None
        for leg in legs:
            if prev_pose is None:
                tagged.append(leg)
            else:
                direction = corridor_direction(prev_pose[0], leg.x, tcfg)
                tagged.append(
                    leg
                    if direction is None
                    else Leg(
                        leg_id=leg.leg_id,
                        index=leg.index,
                        role=leg.role,
                        target_name=leg.target_name,
                        x=leg.x,
                        y=leg.y,
                        yaw=leg.yaw,
                        service_duration_s=leg.service_duration_s,
                        corridor_direction=direction,
                    )
                )
            prev_pose = (leg.x, leg.y, leg.yaw)
        legs = tagged

    return tuple(legs)


def approach_direction(from_x: float, first_leg: Leg, tcfg: TrafficConfig) -> str | None:
    """Which corridor direction the APPROACH to a plan's first leg needs, if any.

    `plan_legs` tags the legs *between* targets, so a plan whose stations all sit in
    one half looks corridor-free -- and for a robot that starts in that half, it is.
    Handed to a robot in the OTHER half, the first leg silently becomes an untagged
    crossing: the dispatcher sends it as an ordinary goal, Nav2 cannot route through
    the barrier, and the task dies on a 180 s timeout instead of being refused.

    That is not hypothetical. A three-robot run assigned a west-side robot an
    east-side task exactly this way, and the failure read as `NAV_FAILED ... leg
    exceeded 180s`, which points at Nav2 rather than at the allocation.

    CONTRACTS section 6 lists route connectivity as an allocation filter, so this
    belongs to the decision about *which robot*, not to the plan.

    `tcfg is None` means this configuration has no corridor at all, so no move can require a
    passage. Returning None is not a shortcut: `corridor_direction` iterates `tcfg.directions` as
    soon as a move changes the sign of x, so without this guard a corridor-less configuration
    raises AttributeError the moment two stations sit on opposite sides -- which is exactly what
    the new `split_by_passage` test found, in a helper that had been called with a real config
    every time until then.
    """
    if tcfg is None:
        return None
    return corridor_direction(from_x, first_leg.x, tcfg)


def split_by_passage(
    poses: dict[str, float], first_leg: Leg, tcfg: TrafficConfig | None
) -> tuple[list[str], list[str]]:
    """Which of these robots can reach `first_leg` without a corridor passage, and which cannot.

    Returns `(same_side, needs_passage)`, both sorted for a stable report. A robot whose x is
    unknown is `needs_passage`: it is not a claim about the robot, it is the absence of one, and
    the caller's other gates (position, freshness) are what decide about it.

    This is the CONTRACTS section 6 route-connectivity question, split out so the ANSWER can be a
    preference. It used to be a refusal inside the dispatcher, which was correct while a crossing
    could not be driven -- and became a task that could never be assigned once one could, because
    the refusal sits upstream of the code that drives the crossing.
    """
    same_side: list[str] = []
    needs_passage: list[str] = []
    for rid in sorted(poses):
        x = poses[rid]
        if x is None or approach_direction(x, first_leg, tcfg) is not None:
            needs_passage.append(rid)
        else:
            same_side.append(rid)
    return same_side, needs_passage


def reachable_chargers(
    cfg: FleetConfig, tcfg: TrafficConfig | None, from_x: float
) -> tuple[str, ...]:
    """Pads a robot standing at ``from_x`` can actually be dispatched to.

    Two ways a pad is not reachable, and both are silent when the pad is picked
    without asking:

      * it has no pose anywhere, so `plan_legs` will refuse the whole task with
        UNKNOWN_STATION after the pad has already been granted; and
      * reaching it crosses the barrier, which turns the approach leg into an untagged
        crossing -- exactly what `approach_direction` exists to catch.

    A pad that is unreachable is not "busy". It is not a candidate for this robot, and
    handing the robot a pad it cannot drive to is how a charge run ends up refused on
    every tick for ever while the pad stays reserved for it (D-P5-22).

    This is the same `corridor_direction` the dispatcher's gate calls, deliberately:
    two implementations of "can this robot get there" would eventually disagree, and
    the disagreement would show up as a task that the allocator thinks is dispatchable
    and the gate refuses. There is a test that asserts the two agree over a grid of
    positions for exactly that reason.
    """
    out: list[str] = []
    for name in sorted(cfg.chargers):
        pose = target_of(name, cfg, tcfg)
        if pose is None:
            continue
        if tcfg is None or corridor_direction(from_x, pose[0], tcfg) is None:
            out.append(name)
    return tuple(out)


def total_distance_m(legs: tuple[Leg, ...], from_pose: tuple[float, float] | None) -> float:
    """Straight-line budget for a plan, used for the battery pre-check only.

    Straight-line is deliberately *less* than the real path, so a plan that
    fails this check fails for certain; one that passes still needs the measured
    arrival error as margin. It is a filter, not a guarantee.
    """
    total = 0.0
    cursor = from_pose
    for leg in legs:
        if cursor is not None:
            total += ((leg.x - cursor[0]) ** 2 + (leg.y - cursor[1]) ** 2) ** 0.5
        cursor = (leg.x, leg.y)
    return total
