"""P4.3: the gate must decide from geometry, not from a flag.

The gate already knew about permits. What it could not do was decide *whether the
robot was approaching a protected region* -- it was told, via
``wants_protected_zone``, by the very component it supervises.

That is not a safety check. It is a request for permission written by the party
being checked. Every test below is a way that arrangement fails:

* the caller says "not in the zone" while sitting in it   -> test_the_caller_...
* a flat PermitView is presented as authority              -> test_a_flat_permit_...
* the guard crashes                                       -> test_a_crashing_guard_...
* the check happens once and is cached                    -> test_the_geofence_is_re_...
* a superseded generation keeps its authority              -> test_reassign_...
* an adapter that never says anything about its localization -> test_a_status_without_...
"""

from __future__ import annotations

import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
SRC = ROOT / "src"
for _pkg in ("fleet_core", "fleet_adapter"):
    _p = SRC / _pkg
    if str(_p) not in sys.path:
        sys.path.insert(0, str(_p))

from fleet_adapter import (  # noqa: E402
    CrossingZoneGuard,
    GateMode,
    GateReason,
    PermitView,
    SafetyGate,
    ZoneVerdict,
)
from fleet_adapter.adapter_base import AdapterPhase, AdapterStatus  # noqa: E402
from fleet_adapter.fake_adapter import FakeAdapter  # noqa: E402
from fleet_core import CrossingManager, ReasonCode, load_traffic_config  # noqa: E402

CONFIG = ROOT / "config" / "resources.yaml"
L, W = 0.60, 0.45          # r01/r02 footprint, from config/fleet.yaml
BOOT = "boot-1"
NOW = 100.0


# --------------------------------------------------------------------------- #
# fixtures
# --------------------------------------------------------------------------- #


@pytest.fixture(scope="module")
def tcfg():
    return load_traffic_config(CONFIG)


def free_manager(tcfg) -> CrossingManager:
    mgr = CrossingManager(tcfg)
    for name in mgr.cfg.resources:
        mgr.mark_verified_free(name, source="test: pre-run geometry check")
    return mgr


def node_pose(cfg, name):
    n = cfg.nodes[name]
    return (n.x, n.y, n.yaw)


def guard_for(mgr, *, task_id="", boot_id=BOOT, revision=0, generation=1, epoch=0):
    return CrossingZoneGuard(
        manager=mgr, robot_id="r01", length_m=L, width_m=W,
        task_id=task_id, boot_id=boot_id, revision=revision,
        generation=generation, epoch=epoch,
    )


def arm(gate: SafetyGate, pose, *, speed=0.0, omega=0.0, loc=True, wall_now=NOW, boot_id=BOOT):
    """Make the health inputs fresh at this pose.

    Without this the heartbeat watchdog fires first and the test would be
    measuring that instead of the geofence.
    """
    gate.heartbeat(wall_now=wall_now)
    gate.observe(
        AdapterStatus(
            robot_id="r01", backend="fake", phase=AdapterPhase.MOVING, generation=1,
            x=pose[0], y=pose[1], yaw=pose[2], speed_mps=speed, angular_radps=omega,
            localization_valid=loc, boot_id=boot_id, epoch=0,
        ),
        wall_now=wall_now,
    )
    return gate


def granted_permit() -> PermitView:
    return PermitView(
        granted=True, resources=("CORRIDOR:mid",), boot_id=BOOT,
        revision=0, generation=1, epoch=0, expires_at_wall=1e9,
    )


def hold(tcfg, mgr, direction, *, task="task-A", boot_id=BOOT, revision=0, generation=1,
         epoch=0, wall_now=NOW, at=None):
    """Acquire a permit at the same wall clock the gate will be asked at.

    Defaulting this to ``NOW`` matters: the TTL is 12 s of wall time, so a permit
    taken at t=0 is already expired by the time the gate is evaluated at t=100.
    That is the gate being correct, not a broken test -- but it makes the test
    measure expiry instead of whatever it meant to measure.
    """
    spec = mgr.direction_spec(direction)
    pose = node_pose(tcfg, at if at is not None else spec.wait_node)
    return mgr.acquire(
        direction, task_id=task, robot_id="r01", boot_id=boot_id, revision=revision,
        generation=generation, epoch=epoch, now=0.0, wall_now=wall_now, pose=pose,
        length_m=L, width_m=W, localization_valid=True,
    )


# --------------------------------------------------------------------------- #
# the geofence decides, not the caller
# --------------------------------------------------------------------------- #


def test_a_robot_far_from_the_crossing_is_not_restricted(tcfg):
    """The guard must not turn into a global stop. Traffic logic that blocks
    everything outside the corridor is unusable, and would be switched off."""
    g = arm(SafetyGate(robot_id="r01", zone=guard_for(free_manager(tcfg))), (5.0, 3.0, 0.0), speed=0.3)
    d = g.evaluate(0.3, 0.0, wall_now=NOW, epoch=0, dt_s=1.0)
    assert not d.is_zero
    assert d.reason in (GateReason.OK, GateReason.ACCEL_CLAMPED, GateReason.SPEED_CLAMPED)


def test_no_permit_inside_the_crossing_stops_the_robot(tcfg):
    g = arm(SafetyGate(robot_id="r01", zone=guard_for(free_manager(tcfg))), (0.0, 0.0, 0.0))
    d = g.evaluate(0.3, 0.0, wall_now=NOW, epoch=0, dt_s=1.0)
    assert d.is_zero
    assert d.mode is GateMode.STOP
    assert d.reason is GateReason.NO_PERMIT


def test_the_caller_cannot_switch_the_geofence_off(tcfg):
    """The hole this stage exists to close.

    The robot is parked inside the gap rectangle. Whoever drives it declares
    ``wants_protected_zone=False``. Before P4.3 that declaration was believed and
    the robot drove on.
    """
    g = arm(SafetyGate(robot_id="r01", zone=guard_for(free_manager(tcfg))), (0.0, 0.0, 0.0))
    d = g.evaluate(
        0.3, 0.0, wall_now=NOW, epoch=0, wants_protected_zone=False, dt_s=1.0
    )
    assert d.is_zero, "the supervised party declared itself outside the zone and was believed"
    assert d.reason is GateReason.NO_PERMIT


def test_a_flat_permit_view_cannot_override_the_geofence(tcfg):
    """A PermitView says "somebody granted me something".

    The guard says where the robot is. When they disagree, geometry wins -- a
    permit minted from a stale route segment must not put the robot in the
    corridor.
    """
    g = arm(SafetyGate(robot_id="r01", zone=guard_for(free_manager(tcfg))), (0.0, 0.0, 0.0))
    d = g.evaluate(
        0.3, 0.0, wall_now=NOW, epoch=0,
        permit=granted_permit(), wants_protected_zone=True, dt_s=1.0,
    )
    assert d.is_zero
    assert d.reason is GateReason.NO_PERMIT


def test_the_geofence_is_re_evaluated_on_every_call(tcfg):
    """Position and twist are arguments, not cached state.

    A guard that remembered "clear a moment ago" would keep allowing motion after
    the robot had driven into the region."""
    g = SafetyGate(robot_id="r01", zone=guard_for(free_manager(tcfg)))

    arm(g, (5.0, 3.0, 0.0))
    assert not g.evaluate(0.3, 0.0, wall_now=NOW, epoch=0, dt_s=1.0).is_zero, "clear pose was refused"

    arm(g, (0.0, 0.0, 0.0))
    assert g.evaluate(0.3, 0.0, wall_now=NOW, epoch=0, dt_s=1.0).is_zero, "pose change was missed"

    arm(g, (5.0, 3.0, 0.0))
    assert not g.evaluate(0.3, 0.0, wall_now=NOW, epoch=0, dt_s=1.0).is_zero, "refusal did not clear"


def test_a_faster_robot_is_stopped_further_out(tcfg):
    """Same pose, two speeds. Only the moving one is refused.

    If speed did not reach the guard, these two would agree -- and the gate would
    authorise motion it cannot stop before the boundary.
    """
    pose = (-2.35, 0.0, 0.0)  # outside the gap rectangle
    g = SafetyGate(robot_id="r01", zone=guard_for(free_manager(tcfg)))

    arm(g, pose, speed=0.0)
    slow = g.evaluate(0.05, 0.0, wall_now=NOW, epoch=0, dt_s=1.0)
    arm(g, pose, speed=0.35)
    fast = g.evaluate(0.05, 0.0, wall_now=NOW, epoch=0, dt_s=1.0)

    assert not slow.is_zero
    assert fast.is_zero
    assert fast.reason is GateReason.NO_PERMIT


def test_turning_in_place_does_not_look_like_standing_still(tcfg):
    """A robot spinning next to the boundary still sweeps space."""
    pose = (-2.35, 0.0, 0.0)
    g = SafetyGate(robot_id="r01", zone=guard_for(free_manager(tcfg)))

    arm(g, pose, speed=0.0, omega=0.0)
    assert not g.evaluate(0.05, 0.0, wall_now=NOW, epoch=0, dt_s=1.0).is_zero

    arm(g, pose, speed=0.0, omega=1.5)
    d = g.evaluate(0.05, 0.0, wall_now=NOW, epoch=0, dt_s=1.0)
    assert d.is_zero
    assert d.reason is GateReason.NO_PERMIT


# --------------------------------------------------------------------------- #
# the guard cannot be crashed out of the way
# --------------------------------------------------------------------------- #


def test_a_crashing_guard_fails_closed(tcfg):
    """An exception escaping a timer callback is an open door, not a stop.

    So the gate converts it into a stop -- and says which one it was, because
    "unknown occupancy" and "a bug in the geometry" call for different responses.
    """

    class Boom:
        def __call__(self, **kwargs) -> ZoneVerdict:  # pragma: no cover - always raises
            raise RuntimeError("geometry exploded")

    g = arm(SafetyGate(robot_id="r01", zone=Boom()), (5.0, 3.0, 0.0))
    d = g.evaluate(0.3, 0.0, wall_now=NOW, epoch=0, dt_s=1.0)
    assert d.is_zero
    assert d.reason is GateReason.RESOURCE_UNKNOWN
    assert "geometry exploded" in d.detail
    assert "not as permission" in d.detail


def test_an_unknown_refusal_code_becomes_no_permit():
    """Fail closed on the mapping too: a reason the gate does not recognise must
    not turn into an allowance."""
    from fleet_adapter.zone_guard import map_reason

    assert map_reason(ReasonCode.ROUTE_REQUIRES_PERMIT) is GateReason.NO_PERMIT
    assert map_reason(ReasonCode.LOCALIZATION_STALE) is GateReason.POSITION_UNKNOWN
    assert map_reason(ReasonCode.PERMIT_EXPIRED) is GateReason.PERMIT_EXPIRED
    # Any code not in the table -- including ones added later -- is a refusal.
    assert map_reason(ReasonCode.UNSUPPORTED) is GateReason.NO_PERMIT


# --------------------------------------------------------------------------- #
# permits still have to be real
# --------------------------------------------------------------------------- #


def test_a_held_permit_lets_the_robot_cross(tcfg):
    mgr = free_manager(tcfg)
    assert hold(tcfg, mgr, "west_to_east", task="task-A").granted is True
    g = arm(SafetyGate(robot_id="r01", zone=guard_for(mgr, task_id="task-A")), (0.0, 0.0, 0.0))
    d = g.evaluate(0.3, 0.0, wall_now=NOW, epoch=0, dt_s=1.0)
    assert not d.is_zero, "a legitimately permitted crossing was refused"


def test_a_lapsed_permit_stops_the_robot(tcfg):
    mgr = free_manager(tcfg)
    assert hold(tcfg, mgr, "west_to_east", task="task-A", wall_now=0.0).granted is True
    mgr.expire_due(wall_now=1e6)
    g = arm(SafetyGate(robot_id="r01", zone=guard_for(mgr, task_id="task-A")),
            (0.0, 0.0, 0.0), wall_now=1e6)
    d = g.evaluate(0.3, 0.0, wall_now=1e6, epoch=0, dt_s=1.0)
    assert d.is_zero
    assert d.reason is GateReason.PERMIT_EXPIRED


def test_holding_one_side_does_not_authorise_the_other(tcfg):
    """Permitted for the corridor and the east buffer; stopped at the west one."""
    mgr = free_manager(tcfg)
    assert hold(tcfg, mgr, "west_to_east", task="task-A").granted is True
    g = arm(SafetyGate(robot_id="r01", zone=guard_for(mgr, task_id="task-A")),
            node_pose(tcfg, "exit_west"))
    d = g.evaluate(0.3, 0.0, wall_now=NOW, epoch=0, dt_s=1.0)
    assert d.is_zero
    assert d.reason is GateReason.NO_PERMIT


def test_reassign_drops_the_previous_generation(tcfg):
    """A new leg supersedes the old one. The old permit is not an authority."""
    mgr = free_manager(tcfg)
    assert hold(tcfg, mgr, "west_to_east", task="task-A", generation=1).granted is True
    guard = guard_for(mgr, task_id="task-A", generation=1)
    g = arm(SafetyGate(robot_id="r01", zone=guard), (0.0, 0.0, 0.0))
    assert not g.evaluate(0.3, 0.0, wall_now=NOW, epoch=0, dt_s=1.0).is_zero

    guard.reassign(task_id="task-A", boot_id=BOOT, revision=0, generation=2, epoch=0)
    arm(g, (0.0, 0.0, 0.0))
    d = g.evaluate(0.3, 0.0, wall_now=NOW, epoch=0, dt_s=1.0)
    assert d.is_zero
    assert d.reason is GateReason.PERMIT_STALE


def test_a_permit_from_another_boot_is_refused(tcfg):
    """A permit issued to a process that no longer exists proves nothing."""
    mgr = free_manager(tcfg)
    assert hold(tcfg, mgr, "west_to_east", task="task-A", boot_id="boot-from-a-previous-life").granted is True
    g = arm(SafetyGate(robot_id="r01", zone=guard_for(mgr, task_id="task-A", boot_id="boot-now")),
            (0.0, 0.0, 0.0))
    d = g.evaluate(0.3, 0.0, wall_now=NOW, epoch=0, dt_s=1.0)
    assert d.is_zero
    assert d.reason is GateReason.PERMIT_STALE


def test_unknown_localization_stops_through_the_geofence(tcfg):
    """The guard is asked before geometry, and refuses when the pose is untrusted."""
    g = arm(
        SafetyGate(robot_id="r01", zone=guard_for(free_manager(tcfg), task_id="task-A")),
        (0.0, 0.0, 0.0), loc=False,
    )
    d = g.evaluate(0.3, 0.0, wall_now=NOW, epoch=0, dt_s=1.0)
    assert d.is_zero
    assert d.reason is GateReason.POSITION_UNKNOWN


def test_estop_still_outranks_the_geofence(tcfg):
    mgr = free_manager(tcfg)
    assert hold(tcfg, mgr, "west_to_east", task="task-A").granted is True
    g = arm(SafetyGate(robot_id="r01", zone=guard_for(mgr, task_id="task-A")), (0.0, 0.0, 0.0))
    g.latch_estop(GateReason.RESOURCE_UNKNOWN, wall_now=NOW)
    d = g.evaluate(0.3, 0.0, wall_now=NOW, epoch=0, dt_s=1.0)
    assert d.is_zero
    assert d.mode is GateMode.ESTOP


# --------------------------------------------------------------------------- #
# the escape hatch, and the legacy path it replaces
# --------------------------------------------------------------------------- #


def test_requires_permit_false_disables_the_geofence_explicitly(tcfg):
    """A documented escape hatch for non-fleet runs.

    It has to exist for tests and for single-robot bring-up, and it has to be a
    named parameter rather than a side effect -- so that turning the geofence off
    is visible in a diff.
    """
    g = arm(
        SafetyGate(robot_id="r01", zone=guard_for(free_manager(tcfg)), requires_permit=False),
        (0.0, 0.0, 0.0),
    )
    assert not g.evaluate(0.3, 0.0, wall_now=NOW, epoch=0, dt_s=1.0).is_zero


def test_without_a_guard_the_legacy_flag_path_is_still_permissive():
    """States the old behaviour plainly, so nobody mistakes it for a geofence.

    With no guard installed the gate believes the caller's flag. That is exactly
    why production bring-up must install a guard -- and why the P4 acceptance run
    is not allowed to claim a corridor block from this path.
    """
    g = SafetyGate(robot_id="r01")
    arm(g, (0.0, 0.0, 0.0))
    assert g.evaluate(0.3, 0.0, wall_now=NOW, epoch=0, wants_protected_zone=True).is_zero
    assert not g.evaluate(0.3, 0.0, wall_now=NOW, epoch=0, wants_protected_zone=False).is_zero


# --------------------------------------------------------------------------- #
# the adapter contract has to carry what a geofence needs
# --------------------------------------------------------------------------- #


def test_a_status_without_localization_health_fails_closed():
    """The default is the safe one, so an adapter that stays silent is stopped."""
    bare = AdapterStatus(robot_id="r01", backend="fake", phase=AdapterPhase.MOVING, generation=1)
    assert bare.localization_valid is False, "the default must not be an optimistic one"


def test_the_fake_adapter_declares_its_localization_valid():
    """The fake has exact self-knowledge, so it says so -- explicitly, not by
    inheriting an optimistic default."""
    assert FakeAdapter(robot_id="r01").status().localization_valid is True


def test_the_gate_holds_a_fake_robot_sitting_in_the_corridor(tcfg):
    """End to end through the fake backend.

    ``FakeAdapter`` starts at (0, 0), which is inside the gap rectangle. With a
    guard installed the gate must refuse to forward the command, so the adapter
    never starts moving.
    """
    mgr = free_manager(tcfg)
    a = FakeAdapter(robot_id="r01")
    assert (a.x, a.y) == (0.0, 0.0)

    g = SafetyGate(robot_id="r01", zone=guard_for(mgr))
    g.heartbeat(wall_now=0.0)
    g.observe(a.status(), wall_now=0.0)

    d = g.evaluate(0.3, 0.0, wall_now=0.0, epoch=0, dt_s=1.0)
    assert d.is_zero
    assert d.reason is GateReason.NO_PERMIT
    assert a.status().phase is AdapterPhase.IDLE
    assert a.distance_travelled_m == 0.0
