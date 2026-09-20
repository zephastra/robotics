"""D-P5-20: the refusal must name the cause that actually applied.

The task service counted `NO_CAPABLE_ROBOT` while the only INSPECT-capable robot was
executing another task. No capability was missing. `allocate()` collected the per-robot
refusals and then reported `NO_CAPABLE_ROBOT` unless one of them happened to be
`INSUFFICIENT_BATTERY` -- a constant with one exception, which is the shape this project
has now paid for three times.

These tests are written against the DECLARED precedence rather than against the
if-chain, so tuning the order is a deliberate act with a failing test, not a silent
reordering.
"""

from __future__ import annotations

import ast
from pathlib import Path

from fleet_core import Allocator, Capability, ReasonCode, reported_reason
from fleet_core.allocator import REASON_PRECEDENCE
from fleet_core.domain import RobotState

ROOT = Path(__file__).resolve().parents[1]
ALLOCATOR = ROOT / "src" / "fleet_core" / "fleet_core" / "allocator.py"
TSN = ROOT / "src" / "fleet_ros" / "fleet_ros" / "task_service_node.py"


def robots(**overrides) -> dict[str, RobotState]:
    """r01/r02 carry, r03 inspects -- the same split the shared `cfg` fixture uses."""
    base = {
        "r01": RobotState("r01", x=0.0, y=0.0, battery_wh=100.0),
        "r02": RobotState("r02", x=0.0, y=0.0, battery_wh=100.0),
        "r03": RobotState("r03", x=0.0, y=0.0, battery_wh=100.0),
    }
    for rid, changes in overrides.items():
        for k, v in changes.items():
            setattr(base[rid], k, v)
    return base


def _submit(ledger, spec_factory, request_id: str, **kw):
    task, _ = ledger.submit(spec_factory(request_id, **kw), now=0.0, epoch=0)
    return task


# --------------------------------------------------------------------------- #
# the defect
# --------------------------------------------------------------------------- #


def test_a_busy_capable_robot_is_reported_as_busy_not_incapable(cfg, ledger, spec_factory):
    """The exact case that was reported wrong: capable, occupied, and named incapable."""
    task = _submit(ledger, spec_factory, "req-inspect",
                   required_capability=Capability.INSPECT)
    rs = {"r03": RobotState("r03", x=0.0, y=0.0, battery_wh=100.0,
                            executing="some-other-task")}
    result = Allocator(cfg).allocate(task, rs, now=0.0)
    assert result.robot_id == ""
    assert result.reason is ReasonCode.RESOURCE_BUSY, (
        "a capable robot doing other work is not an incapable robot; reporting "
        "NO_CAPABLE_ROBOT sends an operator to the capability table"
    )
    assert result.rejected == (("r03", ReasonCode.RESOURCE_BUSY),)


def test_a_busy_robot_beats_an_incapable_one_in_the_reported_reason(cfg, ledger, spec_factory):
    task = _submit(ledger, spec_factory, "req-A")
    rs = robots(r01={"executing": "some-other-task"}, r02={"executing": "another"},
                r03={})  # r03 lacks CARRY
    result = Allocator(cfg).allocate(task, rs, now=0.0)
    assert result.reason is ReasonCode.RESOURCE_BUSY
    assert dict(result.rejected)["r03"] is ReasonCode.NO_CAPABLE_ROBOT, (
        "r03's own refusal must still be recorded as the capability one"
    )


def test_an_offline_robot_is_not_reported_as_incapable(cfg, ledger, spec_factory):
    """Same defect, different input: it has the capability, not a link."""
    task = _submit(ledger, spec_factory, "req-A")
    rs = {"r01": RobotState("r01", x=0.0, y=0.0, battery_wh=100.0, online=False)}
    result = Allocator(cfg).allocate(task, rs, now=0.0)
    assert result.reason is ReasonCode.RESOURCE_UNKNOWN
    assert result.rejected == (("r01", ReasonCode.RESOURCE_UNKNOWN),)


def test_incapable_is_reported_when_that_is_the_only_reason(cfg, ledger, spec_factory):
    """Only a non-carrying robot exists: this is the one case that really is incapacity."""
    task = _submit(ledger, spec_factory, "req-A")
    only_inspect = {"r03": RobotState("r03", x=0.0, y=0.0, battery_wh=100.0)}
    result = Allocator(cfg).allocate(task, only_inspect, now=0.0)
    assert result.reason is ReasonCode.NO_CAPABLE_ROBOT


def test_battery_is_still_reported_ahead_of_incapable(cfg, ledger, spec_factory):
    """Behaviour that already existed and must not be lost in the reordering."""
    task = _submit(ledger, spec_factory, "req-A")
    rs = robots(r01={"battery_wh": 6.0}, r02={"battery_wh": 6.0}, r03={})
    result = Allocator(cfg).allocate(task, rs, now=0.0)
    assert result.reason is ReasonCode.INSUFFICIENT_BATTERY


# --------------------------------------------------------------------------- #
# the evidence, so the label can be checked rather than believed
# --------------------------------------------------------------------------- #


def test_every_considered_robot_keeps_its_own_refusal(cfg, ledger, spec_factory):
    task = _submit(ledger, spec_factory, "req-A")
    rs = robots(r01={"executing": "t"}, r02={"position_known": False},
                r03={"battery_wh": 1.0})
    result = Allocator(cfg).allocate(task, rs, now=0.0)
    reasons = dict(result.rejected)
    assert len(reasons) == 3, "each robot's refusal must survive, not just the winner's"
    assert reasons["r01"] is ReasonCode.RESOURCE_BUSY
    assert reasons["r02"] is ReasonCode.RESOURCE_UNKNOWN
    # r03 does not carry, so the capability filter fires before the battery one
    assert reasons["r03"] is ReasonCode.NO_CAPABLE_ROBOT
    assert result.detail, "the refusals must be readable without decoding the enum"
    assert "r01:RESOURCE_BUSY" in result.detail


def test_a_successful_allocation_still_carries_the_rejections(cfg, ledger, spec_factory):
    """A choice is auditable only if the alternatives are recorded with it."""
    task = _submit(ledger, spec_factory, "req-A")
    rs = robots(r01={"x": -4.9, "y": 2.4}, r03={})
    result = Allocator(cfg).allocate(task, rs, now=0.0)
    assert result.robot_id == "r01"
    assert result.reason is ReasonCode.OK
    assert ("r03", ReasonCode.NO_CAPABLE_ROBOT) in result.rejected


def test_an_empty_rejection_list_reports_no_capable_robot():
    """The conservative fallback: "nobody can take this" rather than "wait"."""
    assert reported_reason([]) is ReasonCode.NO_CAPABLE_ROBOT


# --------------------------------------------------------------------------- #
# the order must be a declaration somebody can read
# --------------------------------------------------------------------------- #


def test_the_precedence_is_declared_and_incapable_is_last():
    assert REASON_PRECEDENCE[-1] is ReasonCode.NO_CAPABLE_ROBOT, (
        "NO_CAPABLE_ROBOT must be the last resort: it is the only refusal that needs a "
        "human to change something, so it must not be used when a robot exists"
    )
    for code in (ReasonCode.RESOURCE_BUSY, ReasonCode.INSUFFICIENT_BATTERY,
                 ReasonCode.RESOURCE_UNKNOWN):
        assert code in REASON_PRECEDENCE
    assert len(set(REASON_PRECEDENCE)) == len(REASON_PRECEDENCE), "no duplicates"


def test_allocate_reports_through_the_declared_precedence():
    """A hardcoded reason in `allocate` is exactly what this record is about."""
    source = ALLOCATOR.read_text(encoding="utf-8")
    tree = ast.parse(source, filename=str(ALLOCATOR))
    body = None
    for node in ast.walk(tree):
        if isinstance(node, ast.FunctionDef) and node.name == "allocate":
            body = ast.get_source_segment(source, node)
    assert body is not None
    assert "reported_reason(rejected)" in body, (
        "allocate() must report through the declared precedence, not an if-chain"
    )
    assert "reason=ReasonCode.NO_CAPABLE_ROBOT" not in body, (
        "a hardcoded NO_CAPABLE_ROBOT is the defect itself"
    )


def test_the_dispatcher_writes_the_refusals_to_the_event_log():
    """The counter said 2087 and named nobody; the event must name who and why."""
    source = TSN.read_text(encoding="utf-8")
    tree = ast.parse(source, filename=str(TSN))
    note = None
    for node in ast.walk(tree):
        if isinstance(node, ast.FunctionDef) and node.name == "_note_no_allocation":
            note = ast.get_source_segment(source, node)
    assert note is not None, "_note_no_allocation is missing from the dispatcher"
    assert '"rejected"' in note, "the event must carry the per-robot refusals"

    assign = None
    for node in ast.walk(tree):
        if isinstance(node, ast.FunctionDef) and node.name == "_assign_ready":
            assign = ast.get_source_segment(source, node)
    assert assign is not None
    assert "_note_no_allocation" in assign, (
        "nothing calls it, so a task refused every tick would stay undiagnosable"
    )
