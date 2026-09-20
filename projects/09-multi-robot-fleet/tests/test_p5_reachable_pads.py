"""D-P5-22: a charge pad must be chosen for a robot that can drive to it.

The dispatcher used to ask for a pad with no preference, so `ChargerAllocator` walked
`sorted(cfg.chargers)` and always returned `C_left`. Charge runs are PINNED, so the
allocator never looks at a second robot: a pad on the far side of the barrier is a
permanent refusal of the run by its own route gate, while the pad stays reserved for the
robot that cannot use it. In the run that exposed it, two robots were each sent to the
pad across the wall, `ROUTE_REQUIRES_PERMIT` was counted 2087 times, the pads read
`grants=2 releases=0`, and three robots never left their spawns.

The first test below is the whole diagnosis, and it needs no simulator: the pad the old
code picked is unreachable by the approach gate for the robot it was picked for. That is
why this file exists -- the病因 was one function call away for a whole round while the
run's own evidence was being read as a queueing statistic.
"""

from __future__ import annotations

import ast
from pathlib import Path

import pytest
import yaml

from fleet_core import (
    ChargerAllocator,
    PlanError,
    ReasonCode,
    ResourceId,
    ResourceKind,
    approach_direction,
    load_traffic_config,
    parse_task_request,
    plan_legs,
    reachable_chargers,
    validate_fleet_config,
)

ROOT = Path(__file__).resolve().parents[1]
TSN = ROOT / "src" / "fleet_ros" / "fleet_ros" / "task_service_node.py"


# --------------------------------------------------------------------------- #
# the shipped config, not a synthetic one: a synthetic config would pass while
# the site layout is what actually decides reachability
# --------------------------------------------------------------------------- #


@pytest.fixture(scope="module")
def raw_fleet():
    return yaml.safe_load((ROOT / "config" / "fleet.yaml").read_text(encoding="utf-8"))


@pytest.fixture(scope="module")
def fleet(raw_fleet):
    return validate_fleet_config(raw_fleet)


@pytest.fixture(scope="module")
def traffic():
    return load_traffic_config(str(ROOT / "config" / "resources.yaml"))


@pytest.fixture(scope="module")
def spawns():
    """`config/spawns.yaml` carries the spawns under a top-level `spawns:` key."""
    body = yaml.safe_load((ROOT / "config" / "spawns.yaml").read_text(encoding="utf-8"))
    return body["spawns"]


def charge_leg(fleet, traffic, pad: str):
    """The first leg of the charge run the dispatcher would build for this pad."""
    spec = parse_task_request(
        {"request_id": f"auto-charge-test-{pad}", "kind": "return_to_charge",
         "destination_station": pad},
        fleet,
    )
    plan = plan_legs(spec, fleet, traffic)
    assert not isinstance(plan, PlanError), plan
    return plan[0]


# --------------------------------------------------------------------------- #
# 1. the diagnosis, offline
# --------------------------------------------------------------------------- #


def test_the_pad_the_allocator_picked_blind_is_unreachable_for_an_east_robot(
    fleet, traffic, spawns
):
    """This is D-P5-22 in four lines.

    `request()` with no `reachable=` is exactly what the dispatcher used to do. It
    returns `C_left` for everybody, because `sorted(chargers)[0]` is `C_left`. For a
    robot spawned on the east side, the approach to `C_left` needs the east_to_west
    crossing -- and an approach leg is untagged, so the route gate refuses it. For ever,
    because the run is pinned.
    """
    grant = ChargerAllocator(fleet).request("r02", now=0.0)
    assert grant is not None and grant.charger_id == "C_left", (
        "if this is no longer the first pad alphabetically the test below is measuring "
        "something else"
    )
    east_x = float(spawns["r02"]["x"])
    assert east_x > 0.0, "r02 is expected to spawn east of the barrier"

    direction = approach_direction(east_x, charge_leg(fleet, traffic, "C_left"), traffic)
    assert direction == "east_to_west", (
        "the pad the old code picked must be refused by the route gate for r02, or "
        "this test is not reproducing D-P5-22"
    )


def test_the_chosen_pad_is_the_one_the_approach_gate_accepts(fleet, traffic, spawns):
    """The fix, stated as the property the bug violated."""
    for rid, pose in sorted(spawns.items()):
        x = float(pose["x"])
        pads = reachable_chargers(fleet, traffic, x)
        assert pads, f"{rid} spawns at x={x} with no pad it can be dispatched to at all"
        for pad in pads:
            assert approach_direction(x, charge_leg(fleet, traffic, pad), traffic) is None, (
                f"{rid} at x={x}: reachable_chargers offers {pad} but the route gate "
                "would refuse it"
            )


# --------------------------------------------------------------------------- #
# 2. the two functions must not be able to disagree
# --------------------------------------------------------------------------- #


def test_reachability_and_the_route_gate_agree_on_every_position(fleet, traffic):
    """Two implementations of "can this robot get there" is one too many.

    `reachable_chargers` decides which pads the allocator may grant; `approach_direction`
    decides whether the dispatcher may issue the leg. If they ever disagree the fleet
    gets a task that is granted and then refused on every tick -- which is the shape of
    the bug, not a new one.
    """
    pads = sorted(fleet.chargers)
    positions = [-20.0, -6.0, -5.0, -2.5, -1.0, -0.001, 0.0, 0.001, 1.0, 2.5, 5.0, 6.0, 20.0]
    for x in positions:
        offered = set(reachable_chargers(fleet, traffic, x))
        for pad in pads:
            dispatchable = approach_direction(x, charge_leg(fleet, traffic, pad), traffic) is None
            assert (pad in offered) is dispatchable, (
                f"x={x}, pad={pad}: reachable_chargers says {pad in offered} and the "
                f"route gate says {dispatchable}"
            )


# --------------------------------------------------------------------------- #
# 3. the allocator's side of the contract
# --------------------------------------------------------------------------- #


def test_an_unreachable_pad_is_never_granted_even_when_it_is_free(fleet, traffic):
    west = reachable_chargers(fleet, traffic, -6.0)
    east = reachable_chargers(fleet, traffic, 6.0)
    assert west == ("C_left",) and east == ("C_right",)

    ca = ChargerAllocator(fleet)
    first = ca.request("r01", now=0.0, reachable=west)
    assert first is not None and first.charger_id == "C_left"
    second = ca.request("r02", now=0.0, reachable=east)
    assert second is not None and second.charger_id == "C_right"

    # Both pads are now taken by robots that can use them. A third, west-side robot must
    # wait for C_left rather than be handed anything else.
    assert ca.request("r03", now=0.0, reachable=west) is None
    assert ca.queue() == ("r03",)
    assert ca.occupant("C_right") == "r02", "a west robot must never land on the east pad"


def test_no_reachable_pad_is_refused_rather_than_queued(fleet):
    """Queueing at a pad you cannot drive to is not waiting, it is a reserved pad."""
    ca = ChargerAllocator(fleet)
    assert ca.request("r01", now=0.0, reachable=()) is None
    assert ca.queue() == (), "a robot with no usable pad must not take a queue place"
    assert ca.refusals[ReasonCode.NO_SAFE_CHARGER.value] == 1


def test_a_queued_robot_does_not_block_a_pad_it_cannot_reach(fleet, traffic):
    """The regression that the reachable set itself introduced.

    Queue matching used to be `q.preferred in (None, charger_id)`, and a robot with no
    preference therefore matched EVERY pad. Adding a reachable set while leaving that
    predicate alone would have put a west-side robot at the head of the east pad's queue,
    blocking the only robot that could use it -- a deadlock created by fixing a different
    bug, which is the kind of thing this file exists to catch.
    """
    west = reachable_chargers(fleet, traffic, -6.0)
    east = reachable_chargers(fleet, traffic, 6.0)

    ca = ChargerAllocator(fleet)
    assert ca.request("r01", now=0.0, reachable=west) is not None   # takes C_left
    assert ca.request("r03", now=0.0, reachable=west) is None       # waits for C_left
    assert ca.queue() == ("r03",)
    assert ca.queue_for("C_left") == ("r03",)
    assert ca.queue_for("C_right") == (), (
        "a robot waiting for the west pad must not appear in the east pad's queue"
    )

    grant = ca.request("r02", now=1.0, reachable=east)
    assert grant is not None and grant.charger_id == "C_right", (
        "r03 is ahead in the west queue and must not hold up the east pad"
    )


def test_a_pad_with_no_pose_is_not_a_candidate(fleet, raw_fleet, traffic):
    """A pad `plan_legs` cannot build a goal for must not be granted either.

    Granting it would reserve a pad, then refuse the whole task with UNKNOWN_STATION --
    the pad held by a robot that has no task, again.
    """
    cfg = validate_fleet_config(raw_fleet)
    cfg.chargers["C_ghost"] = ResourceId(ResourceKind.CHARGER, "C_ghost")
    assert set(reachable_chargers(cfg, traffic, -6.0)) == {"C_left"}


def test_a_stated_preference_does_not_override_reachability(fleet, traffic):
    """A preference is a wish; the reachable set is a fact."""
    ca = ChargerAllocator(fleet)
    grant = ca.request("r02", now=0.0, preferred="C_left",
                       reachable=reachable_chargers(fleet, traffic, 6.0))
    assert grant is not None and grant.charger_id == "C_right"


# --------------------------------------------------------------------------- #
# 4. the dispatcher must use it, and the watchdog must be reachable
# --------------------------------------------------------------------------- #


def _function(path: Path, name: str) -> ast.FunctionDef:
    tree = ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
    for node in ast.walk(tree):
        if isinstance(node, ast.FunctionDef) and node.name == name:
            return node
    raise AssertionError(f"{name} not found in {path}")


def _calls(node: ast.AST, attr: str) -> list[ast.Call]:
    return [
        n for n in ast.walk(node)
        if isinstance(n, ast.Call) and getattr(n.func, "attr", None) == attr
    ]


def test_the_service_tells_the_allocator_which_pads_are_reachable():
    fn = _function(TSN, "_create_charge_task")
    requests = _calls(fn, "request")
    assert requests, "the charge allocator is never asked for a pad"
    assert any(kw.arg == "reachable" for c in requests for kw in c.keywords), (
        "`request()` is called without `reachable=`, so the pad is chosen from the "
        "sorted charger list again -- which is exactly D-P5-22"
    )


def test_the_service_does_not_pick_a_pad_for_a_robot_it_cannot_locate():
    fn = _function(TSN, "_create_charge_task")
    guarded = [
        n for n in ast.walk(fn)
        if isinstance(n, ast.If) and isinstance(n.test, ast.UnaryOp)
        and isinstance(n.test.op, ast.Not) and isinstance(n.test.operand, ast.Attribute)
    ]
    assert any(n.test.operand.attr == "position_known" for n in guarded), (
        "a pad granted for a robot with no position is a pad reserved on a guess"
    )


def test_the_unassignable_watchdog_is_actually_called_every_tick():
    """A watchdog nobody calls is dead code, and this project has shipped that too."""
    tick = _function(TSN, "_tick")
    step = ast.dump(tick)  # a call on `self._watch_unassignable` inside the tick
    assert "_watch_unassignable" in step, (
        "_watch_unassignable exists but `_tick` never calls it, so it would never run"
    )


def test_the_watchdog_only_parks_refusals_that_ticking_cannot_clear():
    """`ACCEPTED` means "waiting". Parking a wait turns a recoverable pose into an alarm."""
    tree = ast.parse(TSN.read_text(encoding="utf-8"), filename=str(TSN))
    names: list[str] = []
    for node in ast.walk(tree):
        if isinstance(node, ast.Assign) and any(
            getattr(t, "id", None) == "PERMANENT_REFUSALS" for t in node.targets
        ):
            names = [n.attr for n in ast.walk(node.value) if isinstance(n, ast.Attribute)]
    assert names == ["ROUTE_REQUIRES_PERMIT"], (
        f"PERMANENT_REFUSALS is {names}; a stale pose, a busy robot and a battery rule "
        "are all waits that clear by themselves, and parking them would make this "
        "watchdog fire on healthy runs"
    )


def test_the_watchdog_writes_the_refusal_that_blocked_it_not_a_constant():
    fn = _function(TSN, "_watch_unassignable")
    parks = _calls(fn, "_park")
    assert parks, "the watchdog never parks anything"
    for call in parks:
        kwargs = {kw.arg: kw.value for kw in call.keywords}
        assert "reason" in kwargs, "_park requires an explicit reason"
        assert isinstance(kwargs["reason"], ast.Name), (
            "the reason must be the refusal that actually blocked the task. 008 lost a "
            "week to a fail_reason column that was a constant."
        )


def test_the_watchdog_releases_the_pad_it_was_holding_for_that_robot():
    """The pad is granted before the task exists; parking without releasing leaks it."""
    assert _calls(_function(TSN, "_watch_unassignable"), "release"), (
        "parking a pinned charge run must release the pad, or the pad is held for the "
        "rest of the session by a robot with no task"
    )
