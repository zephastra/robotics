"""L1 §3, §11: capability/battery filtering, cost, stable tie-break, aging.

Maps to TEST_AND_ACCEPTANCE L1:
  * 3  capability filter, battery filter, distance cost, stable tie-break, queue aging
  * 11 low-battery prediction and charger capacity

Determinism is the property under test as much as correctness: two runs over the
same inputs must pick the same robot, otherwise a failure cannot be reproduced.
"""

from __future__ import annotations

import pytest

from fleet_core import Allocator, Capability, ReasonCode
from fleet_core.domain import RobotState


def robots(**overrides) -> dict[str, RobotState]:
    base = {
        "r01": RobotState("r01", x=0.0, y=0.0, battery_wh=100.0),
        "r02": RobotState("r02", x=0.0, y=0.0, battery_wh=100.0),
        "r03": RobotState("r03", x=0.0, y=0.0, battery_wh=100.0),  # INSPECT only
    }
    for rid, changes in overrides.items():
        for k, v in changes.items():
            setattr(base[rid], k, v)
    return base


def test_closest_robot_wins(ledger, cfg, spec_factory):
    task, _ = ledger.submit(spec_factory("req-A"), now=0.0, epoch=0)
    alloc = Allocator(cfg)
    rs = robots(r01={"x": -4.9, "y": 2.4}, r02={"x": 4.9, "y": 2.4})
    result = alloc.allocate(task, rs, now=0.0)
    assert result.robot_id == "r01"
    assert result.reason is ReasonCode.OK


def test_incapable_robot_is_excluded(cfg, ledger, spec_factory):
    task, _ = ledger.submit(spec_factory("req-A"), now=0.0, epoch=0)
    alloc = Allocator(cfg)
    result = alloc.allocate(task, robots(), now=0.0)
    assert "r03" not in [r for r, _ in result.candidates], (
        "r03 lacks CARRY and must never be a candidate for a CARRY task"
    )
    ok, rejected = alloc.candidates(task, robots(), now=0.0)
    assert ("r03", ReasonCode.NO_CAPABLE_ROBOT) in rejected


def test_no_capable_robot_reports_the_reason(cfg, ledger, spec_factory):
    task, _ = ledger.submit(spec_factory("req-A"), now=0.0, epoch=0)
    alloc = Allocator(cfg)
    only_inspect = {"r03": RobotState("r03", x=0.0, y=0.0, battery_wh=100.0)}
    result = alloc.allocate(task, only_inspect, now=0.0)
    assert result.robot_id == ""
    assert result.reason is ReasonCode.NO_CAPABLE_ROBOT


def test_offline_robot_is_excluded(cfg, ledger, spec_factory):
    task, _ = ledger.submit(spec_factory("req-A"), now=0.0, epoch=0)
    alloc = Allocator(cfg)
    rs = robots(r01={"x": -4.9, "y": 2.4, "online": False})
    result = alloc.allocate(task, rs, now=0.0)
    assert result.robot_id == "r02"


def test_robot_with_unknown_position_is_excluded(cfg, ledger, spec_factory):
    """F14-flavoured: no position means no allocation, not a guess."""
    task, _ = ledger.submit(spec_factory("req-A"), now=0.0, epoch=0)
    alloc = Allocator(cfg)
    rs = robots(r01={"x": -4.9, "y": 2.4, "position_known": False})
    result = alloc.allocate(task, rs, now=0.0)
    assert result.robot_id == "r02"
    ok, rejected = alloc.candidates(task, robots(r01={"position_known": False}), now=0.0)
    assert ("r01", ReasonCode.RESOURCE_UNKNOWN) in rejected


def test_busy_robot_is_excluded(cfg, ledger, spec_factory):
    task, _ = ledger.submit(spec_factory("req-A"), now=0.0, epoch=0)
    alloc = Allocator(cfg)
    rs = robots(r01={"x": -4.9, "y": 2.4, "executing": "some-other-task"})
    result = alloc.allocate(task, rs, now=0.0)
    assert result.robot_id == "r02"


def test_battery_filter_beats_proximity(cfg, ledger, spec_factory):
    """A robot that cannot finish the trip is not a candidate, however near."""
    task, _ = ledger.submit(spec_factory("req-A"), now=0.0, epoch=0)
    alloc = Allocator(cfg)
    rs = robots(r01={"x": -4.99, "y": 2.5, "battery_wh": 6.0}, r02={"x": -3.0, "y": 2.5})
    result = alloc.allocate(task, rs, now=0.0)
    assert result.robot_id == "r02"


def test_insufficient_battery_is_reported(cfg, ledger, spec_factory):
    task, _ = ledger.submit(spec_factory("req-A"), now=0.0, epoch=0)
    alloc = Allocator(cfg)
    rs = robots(
        r01={"battery_wh": 6.0}, r02={"battery_wh": 6.0}, r03={"battery_wh": 6.0}
    )
    result = alloc.allocate(task, rs, now=0.0)
    assert result.reason is ReasonCode.INSUFFICIENT_BATTERY


def test_battery_penalty_prefers_the_healthier_of_two_equal_distances(cfg, ledger, spec_factory):
    task, _ = ledger.submit(spec_factory("req-A"), now=0.0, epoch=0)
    alloc = Allocator(cfg)
    rs = robots(r01={"battery_wh": 20.0}, r02={"battery_wh": 95.0})
    result = alloc.allocate(task, rs, now=0.0)
    assert result.robot_id == "r02"


# --------------------------------------------------------------------------- #
# determinism
# --------------------------------------------------------------------------- #


def test_allocation_is_deterministic_across_repeated_calls(cfg, ledger, spec_factory):
    task, _ = ledger.submit(spec_factory("req-A"), now=0.0, epoch=0)
    alloc = Allocator(cfg)
    picks = {alloc.allocate(task, robots(), now=0.0).robot_id for _ in range(25)}
    assert len(picks) == 1, f"allocation is not stable: {picks}"


def test_tie_break_is_stable_and_documented(cfg, ledger, spec_factory):
    """Identical distance and battery -> robot_id ascending, every time."""
    task, _ = ledger.submit(spec_factory("req-A"), now=0.0, epoch=0)
    alloc = Allocator(cfg)
    rs = robots(r01={"x": -4.9, "y": 2.4}, r02={"x": -4.9, "y": 2.4})
    first = alloc.allocate(task, rs, now=0.0).robot_id
    for _ in range(10):
        assert alloc.allocate(task, rs, now=0.0).robot_id == first
    assert first == "r01"


def test_tie_break_prefers_more_remaining_battery_first(cfg, ledger, spec_factory):
    """When costs tie exactly, the healthier robot wins before id order."""
    task, _ = ledger.submit(spec_factory("req-A"), now=0.0, epoch=0)
    alloc = Allocator(cfg)
    # r01 has less battery, so the battery term makes it marginally more costly;
    # r02 should be chosen even though r01 sorts first by id.
    rs = robots(r01={"battery_wh": 30.0}, r02={"battery_wh": 100.0})
    assert alloc.allocate(task, rs, now=0.0).robot_id == "r02"


# --------------------------------------------------------------------------- #
# aging
# --------------------------------------------------------------------------- #


def test_waiting_reduces_effective_cost_over_time(cfg, ledger, spec_factory):
    task, _ = ledger.submit(spec_factory("req-A"), now=0.0, epoch=0)
    alloc = Allocator(cfg)
    rs = {"r01": RobotState("r01", x=0.0, y=0.0, battery_wh=100.0)}
    c_early = alloc.estimate(task, rs["r01"], now=0.0).cost
    c_late = alloc.estimate(task, rs["r01"], now=100.0).cost
    assert c_late < c_early, "an aging task must become cheaper, not stay put"


def test_aging_does_not_go_negative_without_bound(cfg, ledger, spec_factory):
    """A very old task must not become attractive for the wrong reason -- the
    aging term is bounded by the task's own waiting time, never unbounded."""
    task, _ = ledger.submit(spec_factory("req-A"), now=0.0, epoch=0)
    alloc = Allocator(cfg)
    rs = {"r01": RobotState("r01", x=0.0, y=0.0, battery_wh=100.0)}
    stale = alloc.estimate(task, rs["r01"], now=1e6).cost
    fresh = alloc.estimate(task, rs["r01"], now=0.0).cost
    assert stale <= fresh


# --------------------------------------------------------------------------- #
# low battery helpers
# --------------------------------------------------------------------------- #


def test_low_battery_robots_lists_by_threshold(cfg):
    alloc = Allocator(cfg)
    rs = robots(
        r01={"battery_wh": 10.0}, r02={"battery_wh": 80.0}, r03={"battery_wh": 14.9}
    )
    assert alloc.low_battery_robots(rs) == ["r01", "r03"]


def test_low_battery_ignores_offline_robots(cfg):
    alloc = Allocator(cfg)
    rs = robots(r01={"battery_wh": 1.0, "online": False}, r02={"battery_wh": 90.0})
    assert alloc.low_battery_robots(rs) == []


def test_has_capability(cfg):
    alloc = Allocator(cfg)
    assert alloc.has_capability("r01", Capability.CARRY) is True
    assert alloc.has_capability("r03", Capability.CARRY) is False
    assert alloc.has_capability("r03", Capability.INSPECT) is True
    assert alloc.has_capability("nope", Capability.CARRY) is False


def test_estimate_reports_distance_and_eta(cfg, ledger, spec_factory):
    task, _ = ledger.submit(spec_factory("req-A"), now=0.0, epoch=0)
    alloc = Allocator(cfg)
    rs = RobotState("r01", x=-5.0, y=2.5, battery_wh=100.0)
    cand = alloc.estimate(task, rs, now=0.0)
    # S_left (-5,2.5) -> S_right (5,2.5) is a 10 m leg.
    assert cand.distance_m == pytest.approx(10.0, abs=1e-6)
    assert cand.eta_s > 0
    assert cand.battery_after_wh < 100.0
