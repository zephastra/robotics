"""L1/P2: Safety gate behaviour.

The gate is the last thing between a planner and the wheels, so these tests are
about the stops it must refuse to lift, not about the motion it permits.

Two ideas recur:

  * a *commanded* zero is not a *confirmed* stop, and the report must not confuse
    them (TEST_AND_ACCEPTANCE §3.3 requires physical figures from ground truth)
  * everything that makes authorisation unreliable -- expiry, staleness, a lost
    heartbeat, an unknown position -- resolves to STOP, never to "carry on"
"""

from __future__ import annotations

import pytest

from fleet_adapter.adapter_base import AdapterPhase, AdapterStatus
from fleet_adapter.fake_adapter import FakeAdapter
from fleet_adapter.safety_gate import (
    GateMode,
    GateReason,
    Limits,
    PermitView,
    SafetyGate,
)


def status(
    *,
    speed: float = 0.0,
    phase: AdapterPhase = AdapterPhase.MOVING,
    boot_id: str = "boot-1",
    epoch: int = 0,
) -> AdapterStatus:
    return AdapterStatus(
        robot_id="r01", backend="fake", phase=phase, generation=1,
        speed_mps=speed, boot_id=boot_id, epoch=epoch,
    )


def ready_gate(**kw) -> SafetyGate:
    g = SafetyGate(robot_id="r01", **kw)
    g.heartbeat(wall_now=100.0)
    g.observe(status(), wall_now=100.0)
    return g


def refresh(g: SafetyGate, wall_now: float, *, speed: float = 0.0) -> None:
    """Keep the health inputs live at `wall_now`.

    A robot that is actually there keeps heartbeating and publishing state. Tests
    that jump the wall clock must say so explicitly, otherwise they measure the
    heartbeat watchdog instead of whatever they meant to measure -- and the
    watchdog firing first is correct behaviour, not a bug.
    """
    g.heartbeat(wall_now=wall_now)
    g.observe(status(speed=speed), wall_now=wall_now)


def permit(
    *, expires_at_wall: float = 200.0, epoch: int = 0, boot_id: str = "boot-1"
) -> PermitView:
    return PermitView(
        granted=True, resources=("CORRIDOR:mid",), boot_id=boot_id,
        revision=0, generation=1, epoch=epoch, expires_at_wall=expires_at_wall,
    )


# --------------------------------------------------------------------------- #
# the happy path still passes through the gate
# --------------------------------------------------------------------------- #


def test_without_a_permit_the_robot_may_not_enter_the_protected_zone():
    g = ready_gate()
    d = g.evaluate(0.3, 0.0, wall_now=100.0, epoch=0, wants_protected_zone=True)
    assert d.is_zero
    assert d.mode is GateMode.STOP
    assert d.reason is GateReason.NO_PERMIT


def test_with_a_valid_permit_it_may_enter():
    g = ready_gate()
    d = g.evaluate(
        0.3, 0.0, wall_now=100.0, epoch=0,
        permit=permit(), wants_protected_zone=True, dt_s=1.0,
    )
    assert not d.is_zero
    assert d.mode in (GateMode.NORMAL, GateMode.CLAMPED)
    assert d.reason in (GateReason.OK, GateReason.ACCEL_CLAMPED, GateReason.SPEED_CLAMPED)


def test_inside_the_protected_zone_is_irrelevant_when_no_permit_is_needed():
    g = ready_gate(requires_permit=False)
    d = g.evaluate(0.2, 0.0, wall_now=100.0, epoch=0, wants_protected_zone=True, dt_s=1.0)
    assert not d.is_zero


# --------------------------------------------------------------------------- #
# expiry is wall-time based
# --------------------------------------------------------------------------- #


def test_expired_permit_stops_the_robot():
    g = ready_gate()
    refresh(g, 250.0)
    d = g.evaluate(
        0.3, 0.0, wall_now=250.0, epoch=0,
        permit=permit(expires_at_wall=200.0), wants_protected_zone=True,
    )
    assert d.is_zero
    assert d.reason is GateReason.PERMIT_EXPIRED


def test_expiry_uses_wall_time_so_a_frozen_simulator_still_stops():
    """F10/I04: pausing the world must not extend a lease.

    Wall time advanced 300 s while sim time never moved. The robot is still
    heartbeating, so health is fine -- the permit alone is what expired.
    """
    g = ready_gate()
    refresh(g, 400.0)
    d = g.evaluate(
        0.3, 0.0, wall_now=400.0, epoch=0,
        permit=permit(expires_at_wall=110.0), wants_protected_zone=True,
    )
    assert d.is_zero
    assert d.reason is GateReason.PERMIT_EXPIRED
    assert "sim pause does not extend it" in d.detail


def test_permit_just_inside_the_deadline_is_still_valid():
    g = ready_gate()
    refresh(g, 199.9)
    d = g.evaluate(
        0.3, 0.0, wall_now=199.9, epoch=0,
        permit=permit(expires_at_wall=200.0), wants_protected_zone=True, dt_s=1.0,
    )
    assert not d.is_zero


def test_lost_heartbeat_outranks_an_expired_permit():
    """Ordering matters for the *reason*, and health is the more fundamental one.

    If contact with the robot is gone we do not even know it stopped, so
    reporting "permit expired" would understate the problem.
    """
    g = ready_gate()
    # deliberately no refresh: heartbeat and state are now stale
    d = g.evaluate(
        0.3, 0.0, wall_now=400.0, epoch=0,
        permit=permit(expires_at_wall=110.0), wants_protected_zone=True,
    )
    assert d.is_zero
    assert d.reason is GateReason.HEARTBEAT_LOST


def test_permit_from_another_epoch_is_stale():
    g = ready_gate()
    d = g.evaluate(
        0.3, 0.0, wall_now=100.0, epoch=1,
        permit=permit(epoch=0), wants_protected_zone=True,
    )
    assert d.is_zero
    assert d.reason is GateReason.PERMIT_STALE


def test_permit_from_another_boot_is_stale():
    g = ready_gate()
    d = g.evaluate(
        0.3, 0.0, wall_now=100.0, epoch=0,
        permit=permit(boot_id="boot-from-a-previous-life"), wants_protected_zone=True,
    )
    assert d.is_zero
    assert d.reason is GateReason.PERMIT_STALE


# --------------------------------------------------------------------------- #
# health inputs
# --------------------------------------------------------------------------- #


def test_no_heartbeat_yet_stops():
    g = SafetyGate(robot_id="r01")
    g.observe(status(), wall_now=100.0)
    d = g.evaluate(0.3, 0.0, wall_now=100.0, epoch=0)
    assert d.is_zero
    assert d.reason is GateReason.HEARTBEAT_LOST


def test_stale_heartbeat_stops():
    g = ready_gate(heartbeat_timeout_s=1.0)
    d = g.evaluate(0.3, 0.0, wall_now=105.0, epoch=0)
    assert d.is_zero
    assert d.reason is GateReason.HEARTBEAT_LOST


def test_stale_robot_state_stops():
    g = ready_gate(state_timeout_s=0.5)
    g.heartbeat(wall_now=105.0)
    d = g.evaluate(0.3, 0.0, wall_now=105.0, epoch=0)
    assert d.is_zero
    assert d.reason is GateReason.STATE_STALE


def test_state_epoch_mismatch_stops():
    g = ready_gate()
    d = g.evaluate(0.3, 0.0, wall_now=100.0, epoch=3, state_epoch=2)
    assert d.is_zero
    assert d.reason is GateReason.STATE_STALE


def test_faulted_adapter_stops_and_is_not_treated_as_safe():
    g = ready_gate()
    g.observe(status(phase=AdapterPhase.FAULTED), wall_now=100.0)
    d = g.evaluate(0.3, 0.0, wall_now=100.0, epoch=0)
    assert d.is_zero
    assert d.reason is GateReason.RESOURCE_UNKNOWN


# --------------------------------------------------------------------------- #
# clamping
# --------------------------------------------------------------------------- #


def test_speed_beyond_the_limit_is_clamped_not_passed_through():
    g = ready_gate()
    d = g.evaluate(9.0, 9.0, wall_now=100.0, epoch=0, dt_s=100.0)
    assert d.allowed_linear_mps <= g.limits.max_linear_mps
    assert d.allowed_angular_radps <= g.limits.max_angular_radps
    assert d.reason is GateReason.SPEED_CLAMPED


def test_acceleration_is_limited_between_consecutive_commands():
    g = ready_gate()
    d = g.evaluate(0.30, 0.0, wall_now=100.0, epoch=0, dt_s=0.05)
    assert d.allowed_linear_mps == pytest.approx(0.5 * 0.05, abs=1e-9)
    assert d.reason is GateReason.ACCEL_CLAMPED


def test_command_within_limits_passes_unchanged():
    g = ready_gate()
    d = g.evaluate(0.1, 0.1, wall_now=100.0, epoch=0, dt_s=1.0)
    assert d.reason is GateReason.OK
    assert d.allowed_linear_mps == pytest.approx(0.1)


def test_limits_reject_nonsense_values():
    with pytest.raises(ValueError):
        Limits(max_linear_mps=0.0)
    with pytest.raises(ValueError):
        Limits(stop_speed_mps=-1.0)


# --------------------------------------------------------------------------- #
# commanded stop vs confirmed stop
# --------------------------------------------------------------------------- #


def test_publishing_zero_is_not_recorded_as_a_confirmed_stop():
    g = ready_gate()
    g.evaluate(0.0, 0.0, wall_now=100.0, epoch=0, wants_protected_zone=True)
    assert g.zero_published_at == 100.0
    assert g.stopped_confirmed_at is None, (
        "a zero command must not be mistaken for a measured stop"
    )


def test_stop_is_confirmed_only_when_the_robot_is_actually_slow():
    g = ready_gate()
    g.evaluate(0.0, 0.0, wall_now=100.0, epoch=0, wants_protected_zone=True)
    g.observe(status(speed=0.40), wall_now=100.1)
    assert g.stopped_confirmed_at is None
    g.observe(status(speed=0.01), wall_now=100.7)
    assert g.stopped_confirmed_at == 100.7


def test_motion_again_clears_the_stop_record():
    g = ready_gate()
    g.evaluate(0.0, 0.0, wall_now=100.0, epoch=0, wants_protected_zone=True)
    g.observe(status(speed=0.0), wall_now=100.5)
    assert g.stopped_confirmed_at is not None
    g.evaluate(0.3, 0.0, wall_now=101.0, epoch=0, permit=permit(), wants_protected_zone=True, dt_s=1.0)
    assert g.stopped_confirmed_at is None


def test_latency_report_refuses_to_invent_physical_numbers():
    g = ready_gate()
    g.evaluate(0.0, 0.0, wall_now=100.0, epoch=0, wants_protected_zone=True)
    g.observe(status(speed=0.0), wall_now=100.6)
    rep = g.stop_latency_report()
    assert rep["command_to_observation_s"] == pytest.approx(0.6)
    assert rep["physical_stop_time_s"] == "NOT_RUN"
    assert rep["physical_stop_distance_m"] == "NOT_RUN"


# --------------------------------------------------------------------------- #
# estop is latched
# --------------------------------------------------------------------------- #


def test_estop_latches_and_ignores_later_valid_commands():
    g = ready_gate()
    g.latch_estop(GateReason.RESOURCE_UNKNOWN, wall_now=100.0)
    d = g.evaluate(
        0.3, 0.0, wall_now=101.0, epoch=0, permit=permit(), wants_protected_zone=True,
    )
    assert d.is_zero
    assert d.mode is GateMode.ESTOP


def test_estop_clears_only_on_explicit_reset():
    g = ready_gate()
    g.latch_estop(GateReason.RESOURCE_UNKNOWN, wall_now=100.0)
    g.reset_estop()
    refresh(g, 101.0)
    d = g.evaluate(
        0.3, 0.0, wall_now=101.0, epoch=0, permit=permit(), wants_protected_zone=True,
        dt_s=1.0,
    )
    assert not d.is_zero


# --------------------------------------------------------------------------- #
# shutdown
# --------------------------------------------------------------------------- #


def test_shutdown_publishes_zero_and_declares_downstream_required():
    """F13: the gate exiting is not the same as the robot stopping."""
    g = ready_gate()
    d = g.shutdown(wall_now=110.0)
    assert d.is_zero
    assert d.reason is GateReason.SHUTDOWN
    assert g.shutdown_requires_downstream is True
    assert "NOT_RUN" in d.detail


def test_reason_counts_summarises_the_decisions():
    g = ready_gate()
    g.evaluate(0.3, 0.0, wall_now=100.0, epoch=0, wants_protected_zone=True)
    g.evaluate(0.3, 0.0, wall_now=100.0, epoch=0, wants_protected_zone=True)
    counts = g.reason_counts()
    assert counts.get("NO_PERMIT") == 2


# --------------------------------------------------------------------------- #
# the gate works with the fake adapter end to end
# --------------------------------------------------------------------------- #


def test_gate_blocks_a_fake_adapter_from_moving_without_a_permit():
    a = FakeAdapter(robot_id="r01", speed_mps=0.5)
    g = SafetyGate(robot_id="r01")
    g.heartbeat(wall_now=0.0)
    g.observe(a.status(), wall_now=0.0)

    d = g.evaluate(0.5, 0.0, wall_now=0.0, epoch=0, wants_protected_zone=True, dt_s=1.0)
    assert d.is_zero
    # Nothing was forwarded to the adapter, so it never started moving.
    assert a.status().phase is AdapterPhase.IDLE
    assert a.distance_travelled_m == 0.0


def test_gate_forwards_a_clamped_command_and_the_adapter_moves():
    a = FakeAdapter(robot_id="r01", speed_mps=0.5)
    g = SafetyGate(robot_id="r01")
    g.heartbeat(wall_now=0.0)
    g.observe(a.status(), wall_now=0.0)

    # The permit must name the boot that actually issued the status, otherwise the
    # gate is right to call it stale.
    p = permit(boot_id=a.boot_id)
    d = g.evaluate(
        0.5, 0.0, wall_now=0.0, epoch=0, permit=p, wants_protected_zone=True, dt_s=10.0
    )
    assert not d.is_zero
    assert d.allowed_linear_mps <= g.limits.max_linear_mps


def test_gate_refuses_a_permit_from_a_different_boot():
    a = FakeAdapter(robot_id="r01", boot_id="fake-boot")
    g = SafetyGate(robot_id="r01")
    g.heartbeat(wall_now=0.0)
    g.observe(a.status(), wall_now=0.0)

    d = g.evaluate(
        0.5, 0.0, wall_now=0.0, epoch=0,
        permit=permit(boot_id="some-other-boot"), wants_protected_zone=True, dt_s=10.0,
    )
    assert d.is_zero
    assert d.reason is GateReason.PERMIT_STALE
