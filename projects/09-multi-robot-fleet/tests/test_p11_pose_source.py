"""P11: the gate may conclude from the localiser, and must refuse rather than fall back.

THE MEASUREMENT (one instrumented N01, one process, one clock)

    the gate's pose IS spawn (+) odom     mean 0.12 mm       -- not a frame bug
    its error against truth               err = 0.0809 * distance travelled, R^2 = 0.923
                                          p95 1.725 m while moving
    AMCL's error against truth            p50 0.205, p95 0.348 m while moving
    AMCL's age while moving               p50 0.351 s, p95 0.853 s
    AMCL's age AT REST                    p95 242.7 s

The last line is why the freshness rule has two clauses rather than one, and the tests below pin
both directions: a single-clause rule either freezes the fleet at every stop (the latch this project
has removed three times) or authorises motion on a position metres out of date.
"""

from __future__ import annotations

import ast
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
for _pkg in ("fleet_core", "fleet_adapter"):
    _p = ROOT / "src" / _pkg
    if str(_p) not in sys.path:
        sys.path.insert(0, str(_p))

from fleet_core.pose_source import (  # noqa: E402
    POSE_SOURCE_LOCALISER,
    POSE_SOURCE_SPAWN_ODOM,
    VALID_SOURCES,
    LocaliserPolicy,
    judge_localiser,
)

GATE = ROOT / "src" / "fleet_ros" / "fleet_ros" / "gate_node.py"

#: The measured p95 age of the localiser's pose while the robot is moving, from the round-10
#: instrumentation. Named here so the policy's default can be checked against it rather than
#: against a feeling.
MEASURED_P95_AGE_MOVING_S = 0.853


def policy() -> LocaliserPolicy:
    return LocaliserPolicy()


def test_a_fresh_estimate_is_usable():
    verdict = judge_localiser(age_s=0.2, moved_since_estimate_m=0.4, policy=policy())
    assert verdict.usable is True
    assert "fresh" in verdict.reason
    assert verdict.age_s == 0.2


def test_a_stale_estimate_of_a_stationary_robot_is_still_usable():
    """The clause that stops the fix from re-creating the latch it is fixing.

    Measured: the localiser's age p95 is 242.7 s AT REST, because it stops publishing when the robot
    stops. A single-clause freshness rule would refuse every command at every stop.
    """
    verdict = judge_localiser(age_s=240.0, moved_since_estimate_m=0.002, policy=policy())
    assert verdict.usable is True
    assert "still describes where the robot is" in verdict.reason
    assert "0.002" in verdict.reason


def test_a_stale_estimate_of_a_moving_robot_is_not_usable_and_says_why():
    """The other direction: an old pose of a robot that has moved no longer bounds it."""
    verdict = judge_localiser(age_s=8.0, moved_since_estimate_m=2.8, policy=policy())
    assert verdict.usable is False
    # Both numbers, because "stale" alone does not say whether to widen the bound or fix the
    # localiser.
    assert "8.000" in verdict.reason
    assert "2.800" in verdict.reason
    assert "8.1%" not in verdict.reason  # the reason must be this rejection's numbers


def test_a_stale_estimate_with_unknown_displacement_is_not_usable():
    """Unknown is not a pass. If the estimate is stale and nothing says where the robot went,
    there is no bound to conclude from."""
    verdict = judge_localiser(age_s=8.0, moved_since_estimate_m=None, policy=policy())
    assert verdict.usable is False
    assert "unknown" in verdict.reason


def test_no_estimate_at_all_is_not_usable():
    verdict = judge_localiser(age_s=None, moved_since_estimate_m=0.0, policy=policy())
    assert verdict.usable is False
    assert "no localised pose has arrived" in verdict.reason


def test_the_age_bound_is_above_the_measured_p95_age():
    """A bound at or below the measured p95 would refuse a large share of moving samples.

    Derived from the measurement this feature exists for, so a future edit that tightens it has to
    deal with this test.
    """
    p = policy()
    assert p.max_age_s > MEASURED_P95_AGE_MOVING_S, (
        f"max_age_s {p.max_age_s} is not above the measured p95 age "
        f"{MEASURED_P95_AGE_MOVING_S} s; the gate would refuse while moving")
    assert p.max_age_s >= 2.0, (
        "the measured table gives 97.7% fresh at 1.0 s and 98.3% at 2.0 s; the default is the "
        "one that holds a fresh pose on essentially every moving sample")


def test_a_zero_age_bound_is_refused_at_construction():
    """A zero bound is not a strict setting, it is a stop command."""
    with pytest.raises(ValueError, match="positive"):
        LocaliserPolicy(max_age_s=0.0)


def test_the_two_sources_are_named_and_the_default_is_the_odometry_one():
    """`VALID_SOURCES` is what an unknown parameter value is checked against, so it must be the
    complete set -- and the default must stay the behaviour that needs no localiser."""
    assert VALID_SOURCES == (POSE_SOURCE_SPAWN_ODOM, POSE_SOURCE_LOCALISER)
    assert POSE_SOURCE_SPAWN_ODOM == "spawn_odom"


def _events_in_source_order(node: ast.AST):
    """(kind, name) for `self.<name>` accesses, in the order Python would evaluate them.

    Order is the whole point and it is why `ast.walk` cannot be used: that is breadth-first, so it
    would report every attribute of `__init__` as if the order did not exist. An assignment's
    right-hand side is evaluated BEFORE the target is bound, so a `self.x = self.x + 1` reads first.
    """

    def writes(target):
        if (isinstance(target, ast.Attribute) and isinstance(target.value, ast.Name)
                and target.value.id == "self"):
            yield ("write", target.attr)
            return
        for child in ast.iter_child_nodes(target):
            yield from writes(child)

    def walk(current):
        if isinstance(current, ast.Call):
            # Mark the callee so the attribute branch below can tell a method from an attribute.
            if isinstance(current.func, ast.Attribute):
                current.func._is_call_target = True
            for child in ast.iter_child_nodes(current):
                yield from walk(child)
            return
        if isinstance(current, ast.Assign):
            yield from walk(current.value)
            for target in current.targets:
                yield from writes(target)
            return
        if isinstance(current, ast.AnnAssign):
            if current.value is not None:
                yield from walk(current.value)
            yield from writes(current.target)
            return
        if isinstance(current, ast.AugAssign):
            yield from walk(current.target)
            yield from walk(current.value)
            return
        if (isinstance(current, ast.Attribute) and isinstance(current.value, ast.Name)
                and current.value.id == "self"):
            # A `self.x(...)` is a METHOD lookup on the class, not a read of an instance attribute
            # assigned in `__init__`. The first version of this check did not make that
            # distinction and flagged `declare_parameter`, `get_logger`, `create_subscription`
            # and every other inherited call -- a check that is mostly noise gets switched off.
            yield ("read" if not getattr(current, "_is_call_target", False) else "method",
                   current.attr)
            return
        for child in ast.iter_child_nodes(current):
            yield from walk(child)

    yield from walk(node)


def test_no_attribute_is_read_before_it_is_assigned_in_init():
    """A read before its assignment raises at construction, and nothing else can see it.

    Not hypothetical: the first version of the pose-source startup log read `self._pose_source`
    from the traffic-guard block, which runs before the state block that assigns it. `__init__`
    raised, **the gate node never started**, `cmd_vel` was never published, the robot never moved
    (odometry path 0.000 m for a whole run) and Nav2 aborted every goal with `error_code 105`.
    Nothing in the run said the gate was absent, because a node that never starts publishes
    nothing that could be absent.

    `check_undefined_names.py` cannot catch this: the name exists on `rclpy.node.Node`'s class
    layout or it does not, and either way it says nothing about when the instance gets it.
    """
    tree = ast.parse(GATE.read_text(encoding="utf-8"))
    init = next((n for n in ast.walk(tree)
                 if isinstance(n, ast.FunctionDef) and n.name == "__init__"), None)
    assert init is not None, "the gate has no __init__ to check"

    events = list(_events_in_source_order(init))
    # "Own" = assigned somewhere in this __init__. An inherited method is never assigned here, so
    # it is not own, so it is not this check's business -- which is what stops the check from
    # reporting `get_logger`, `declare_parameter` and every other call as a fault.
    own = {name for kind, name in events if kind == "write"}
    assigned: set[str] = set()
    problems: list[str] = []
    for kind, name in events:
        if kind == "write":
            assigned.add(name)
        elif kind == "read" and name in own and name not in assigned:
            problems.append(name)

    assert not problems, (
        f"__init__ reads {sorted(set(problems))} before assigning them. That raises at "
        "construction, so the node never starts and nothing it owns is ever published -- and the "
        "only witness to that is the node itself. Move the read after the assignment.")
    """The invariant that makes the change auditable: one answer to where the robot is.

    Two places constructing an `AdapterStatus` would mean two poses, and the geofence reading one
    while the payload reports the other.
    """
    tree = ast.parse(GATE.read_text(encoding="utf-8"))
    builders = []
    for node in ast.walk(tree):
        if isinstance(node, ast.FunctionDef):
            for inner in ast.walk(node):
                if (isinstance(inner, ast.Call) and isinstance(inner.func, ast.Name)
                        and inner.func.id == "AdapterStatus"):
                    builders.append(node.name)
    assert builders == ["_status"], (
        f"AdapterStatus is built in {builders}; the pose and the localisation-validity flag must "
        "be decided in one place, or two components can disagree about where the robot is")

    body = next(n for n in ast.walk(tree)
                if isinstance(n, ast.FunctionDef) and n.name == "_status")
    source = ast.dump(body)
    assert "trusted" in source, (
        "_status no longer routes the pose through _trusted_pose, so the configured source is "
        "decided somewhere else")
    assert "localization_valid" in source


# --------------------------------------------------------------------------- #
# The localiser deadlock, measured 2026-09-17
# --------------------------------------------------------------------------- #
#
# The policy's escape hatch -- "a stale estimate is still usable while the robot has not
# moved" -- needs the odometry pose from the instant the estimate arrived. `_on_localiser`
# captures `self._odom_raw`, which is None if no odometry has been seen yet, and live it
# was: AMCL publishes its initial pose the moment it activates and the gate's first odom
# message arrived after it. So the hatch could never open, the gate refused on every tick,
# the robot never moved, and because `amcl_pose` is motion-gated (update_min_d: 0.25 when
# this was measured; 0.10 since D-P17-06) the
# localiser never published again -- 1 message in 640 s, 0.000 m of travel.
#
# These two checks pin both halves: the policy DOES open the hatch when the displacement is
# known, and the node has a path that fills a missed arrival snapshot after the fact.


def test_a_stale_estimate_with_no_movement_still_bounds_the_robot():
    """The hatch itself: unknown displacement must not be the only option."""
    policy = LocaliserPolicy(max_age_s=2.0, max_moved_m=0.10)
    verdict = judge_localiser(age_s=300.0, moved_since_estimate_m=0.0, policy=policy)
    assert verdict.usable, (
        "a robot that has not moved is still where the last estimate put it; refusing here is "
        "what froze the fleet for 620 s")
    verdict = judge_localiser(age_s=300.0, moved_since_estimate_m=0.09, policy=policy)
    assert verdict.usable
    verdict = judge_localiser(age_s=300.0, moved_since_estimate_m=0.11, policy=policy)
    assert not verdict.usable


def test_an_unknown_displacement_is_refused_and_says_so():
    """The refusal is right. What was wrong was that it could never end."""
    policy = LocaliserPolicy(max_age_s=2.0, max_moved_m=0.10)
    verdict = judge_localiser(age_s=300.0, moved_since_estimate_m=None, policy=policy)
    assert not verdict.usable
    assert "displacement" in verdict.reason and "unknown" in verdict.reason


def test_the_gate_can_fill_a_missed_arrival_snapshot():
    """The node must have a path that makes the hatch live after the estimate arrived.

    Asserted on the AST rather than on a substring: the check is "this assignment exists in
    this function", and a substring search would be satisfied by the explanatory comment
    above it -- which is how four earlier assertions in this project were defeated.
    """
    import ast
    import pathlib

    src = pathlib.Path(__file__).resolve().parents[1] / "src" / "fleet_ros" / "fleet_ros" / "gate_node.py"
    tree = ast.parse(src.read_text(encoding="utf-8"))
    on_odom = None
    for node in ast.walk(tree):
        if isinstance(node, ast.FunctionDef) and node.name == "_on_odom":
            on_odom = node
    assert on_odom is not None, "gate_node.py has no _on_odom"

    assigns = []
    for node in ast.walk(on_odom):
        if isinstance(node, (ast.Assign, ast.AnnAssign)):
            targets = node.targets if isinstance(node, ast.Assign) else [node.target]
            for t in targets:
                if (isinstance(t, ast.Attribute)
                        and t.attr == "_localiser_odom_at_arrival"):
                    assigns.append(node.lineno)
    assert assigns, (
        "`_on_odom` never assigns `_localiser_odom_at_arrival`, so a localised estimate that "
        "arrives before the first odometry sample leaves the displacement unknown for ever "
        "and the gate refuses every tick -- measured: 1 amcl message, 0.000 m of travel")
