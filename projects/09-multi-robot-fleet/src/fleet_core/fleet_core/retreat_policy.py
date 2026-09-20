"""The retreat command emitter: the missing half of D-P4-12.

WHAT D-P4-12 LEFT UNDONE
------------------------
`D-P4-12` proposed a "retreat permit": a robot already inside a rectangle it has no permit
for may move, if and only if the commanded motion STRICTLY reduces its overlap with the
protected region. That is the SAFETY half.

It is only half. `check_movement` takes a pose and a twist and answers "allowed, right now".
**If nobody issues a retreat command, the widest permit in the world moves nothing.** A robot
that has lapsed inside the bundle is refused, the refusal zeros its velocity, and it sits
there for ever -- which is what the project has observed and mis-read as a navigation fault.

So the second half is the POLICY half: a component that notices a robot is stopped inside a
region it cannot legally leave, and ISSUES a "back up N metres" goal.

WHY THIS MODULE EXISTS BEFORE THE PERMIT DOES
---------------------------------------------
`D-P14-02` fixes the order and the reason. Doing the permit first produces a robot that is
allowed to move and never told to; doing the emitter first produces a robot that is told to
move and not allowed to. The emitter is first because it is a POLICY change -- pure,
replayable, testable without a simulator -- while the permit is a SAFETY change that needs its
own design and a live run.

THIS MODULE ONLY DECIDES. IT DOES NOT DRIVE.
--------------------------------------------
It answers three questions and nothing else:

* is this robot in the situation the retreat is for?      -> ``needs_retreat``
* which way is out, and how far?                          -> ``retreat_command``
* is the retreat working?                                 -> ``progress``

Nothing here publishes, subscribes, or touches a robot. The caller owns the actuation, so
this can be tested against recorded poses with no simulator and reviewed as arithmetic.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

#: How far back the robot is asked to go in one retreat command. Long enough to clear a
#: 0.7 m pad plus the stopping envelope, short enough that the goal stays inside the legal
#: free space behind the robot. Derived from the geometry the resources actually use:
#: the widest protected rectangle is 3.5 m (the gap, x in [-1.75, 1.75]) but a retreat never
#: crosses it -- it leaves it -- so the binding number is the exit pad's 0.70 m plus the
#: measured stopping envelope at 0.35 m/s (0.1381 m) and the footprint half-length (0.30 m),
#: with the rest as room to be counted as "arrived" under node_reach_tolerance_m.
DEFAULT_RETREAT_DISTANCE_M = 1.20

#: The robot must be this close to stopped before a retreat is issued. A robot still moving
#: under its own plan is not stuck; interrupting it would be the interference this project
#: has already paid for once (a second goal rejected for 60 s while the first held the server).
STOPPED_SPEED_MPS = 0.02

#: The retreat is judged to be working if the overlap falls by this much. Below it, the
#: command is not achieving anything and repeating it is the loop this module exists to stop.
PROGRESS_EPSILON_M = 0.02


@dataclass(frozen=True)
class RetreatDecision:
    """Whether to retreat, and if so, where to."""

    needed: bool
    reason: str
    #: The goal as (x, y), in the same frame as the pose it was derived from.
    goal: tuple[float, float] | None = None
    distance_m: float = 0.0
    overlap_m: float = 0.0


def _overlap_depth(pose: tuple[float, float], rect: tuple[float, float, float, float]) -> float:
    """How deep inside `rect` the point is, in metres. 0.0 when outside.

    `rect` is (x_min, y_min, x_max, y_max). The depth is the SHORTEST distance to an edge,
    because that is the distance that has to be travelled to be out, and it is the quantity a
    retreat is trying to reduce. Using the distance to the centre instead would call a robot
    just inside a corner "deeply inside" and issue a retreat it does not need.
    """
    x, y = pose[0], pose[1]
    x_min, y_min, x_max, y_max = rect
    if not (x_min <= x <= x_max and y_min <= y <= y_max):
        return 0.0
    return min(x - x_min, x_max - x, y - y_min, y_max - y)


def _exit_direction(pose: tuple[float, float],
                    rect: tuple[float, float, float, float]) -> tuple[float, float]:
    """A unit vector pointing at the NEAREST edge -- the shortest way out.

    Nearest edge, not nearest corner, and not the way the robot came in. The robot may have
    arrived from any side, and the only thing that matters for legality is getting outside the
    rectangle; the shortest exit also minimises the distance driven while unpermitted.
    """
    x, y = pose[0], pose[1]
    x_min, y_min, x_max, y_max = rect
    candidates = [
        (x - x_min, (-1.0, 0.0)),
        (x_max - x, (1.0, 0.0)),
        (y - y_min, (0.0, -1.0)),
        (y_max - y, (0.0, 1.0)),
    ]
    return min(candidates, key=lambda c: c[0])[1]


def needs_retreat(pose: tuple[float, float, float],
                  twist: tuple[float, float, float],
                  unpermitted_rects: dict[str, tuple[float, float, float, float]],
                  *, permittable: bool) -> RetreatDecision:
    """Is this robot stuck inside a region it cannot legally leave?

    All four conditions have to hold, and each is here because leaving it out produced a
    wrong answer at some point in this project's history:

    * it is inside a rectangle it does not hold permission for -- otherwise there is nothing
      to retreat from;
    * it is essentially stopped -- otherwise its own plan is still running, and pre-empting a
      live navigation goal is what the 60 s `busy` budget was spent on;
    * it is NOT permittable -- a robot that could just be granted the region should be granted
      it, not driven out of it. Retreating a robot that had a legal path forward would be
      introducing motion nobody asked for, on the safety-critical path;
    * the rectangle is not unbounded -- a degenerate rectangle would make the exit direction
      meaningless.
    """
    if permittable:
        return RetreatDecision(
            needed=False,
            reason="the robot could be granted this region; granting it is the answer, "
                   "not driving it out",
        )
    speed = math.hypot(twist[0], twist[1])
    if speed > STOPPED_SPEED_MPS:
        return RetreatDecision(
            needed=False,
            reason=f"the robot is still moving ({speed:.3f} m/s); its own plan is live and "
                   f"a competing goal was already measured to cost 60 s of `busy`",
        )
    worst_name, worst_rect, worst_depth = None, None, 0.0
    for name, rect in unpermitted_rects.items():
        if rect[2] <= rect[0] or rect[3] <= rect[1]:
            continue
        depth = _overlap_depth(pose, rect)
        if depth > worst_depth:
            worst_name, worst_rect, worst_depth = name, rect, depth
    if worst_name is None:
        return RetreatDecision(
            needed=False, reason="the robot is outside every region it has no permit for")
    return RetreatDecision(
        needed=True,
        reason=(f"stopped {worst_depth:.3f} m inside {worst_name!r} with no permit for it and "
                f"no way to be granted one; the refusal zeroes its velocity, so it will not "
                f"move until something tells it to"),
        overlap_m=worst_depth,
    )


def retreat_command(pose: tuple[float, float, float],
                    unpermitted_rects: dict[str, tuple[float, float, float, float]],
                    *, distance_m: float = DEFAULT_RETREAT_DISTANCE_M) -> RetreatDecision:
    """Where to send the robot: straight out of the nearest edge, `distance_m` beyond it.

    The goal is placed OUTSIDE, at the edge plus the distance, rather than "N metres along the
    reverse of the entry heading". The robot's entry heading is not recorded anywhere (the
    recorder has no history), and reconstructing it would be a guess; the nearest edge is a
    fact about the current pose.
    """
    deep = needs_retreat(pose, (0.0, 0.0, 0.0), unpermitted_rects, permittable=False)
    if not deep.needed:
        return deep
    x, y = pose[0], pose[1]
    best_name, best_rect, best_depth = None, None, 0.0
    for name, rect in unpermitted_rects.items():
        if rect[2] <= rect[0] or rect[3] <= rect[1]:
            continue
        depth = _overlap_depth((x, y), rect)
        if depth > best_depth:
            best_name, best_rect, best_depth = name, rect, depth
    ux, uy = _exit_direction((x, y), best_rect)
    goal = (x + ux * (best_depth + distance_m), y + uy * (best_depth + distance_m))
    return RetreatDecision(
        needed=True,
        reason=(f"retreat {best_depth + distance_m:.3f} m out of {best_name!r} through its "
                f"nearest edge, so the robot stops being somewhere it has no permit for"),
        goal=(round(goal[0], 4), round(goal[1], 4)),
        distance_m=round(best_depth + distance_m, 4),
        overlap_m=round(best_depth, 4),
    )


def progress(before_m: float, after_m: float) -> dict[str, object]:
    """Did the last retreat reduce the overlap?

    A retreat that does not reduce it must stop being re-issued. Repeating a command that has
    been measured not to work is precisely the ``recoveries=8`` loop this project spent a
    round chasing, and the fix then was the same: notice, and stop.
    """
    delta = before_m - after_m
    return {
        "reduced_by_m": round(delta, 4),
        "working": delta >= PROGRESS_EPSILON_M,
        "detail": (f"overlap {before_m:.3f} m -> {after_m:.3f} m "
                   f"({'reduced' if delta > 0 else 'not reduced'})"
                   + ("" if delta >= PROGRESS_EPSILON_M else
                      "; a retreat that does not reduce the overlap must not be re-issued -- "
                      "that is the same unbounded retry loop as `recoveries=8`")),
    }
