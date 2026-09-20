"""`_eligible_robots` must refuse a robot that cannot localise.

Structural, because the function needs a live node to run. The defect it pins was
observed live: four system-generated charge runs burnt inside the first 155 s of a run,
each one refused by the adapter with `LOCALIZATION_STALE` while the fleet was still
bringing up Nav2 and odometry.

The reason this matters for a *pinned* task in particular is that `_assign_ready` decides
`run.pinned_robot in self._eligible_robots(...)` and never calls `Allocator.candidates`
for a pinned run -- so the allocator's own `position_known` check is bypassed entirely on
the one path where the robot is chosen by construction rather than by comparison.
"""

from __future__ import annotations

import ast
import pathlib

ROOT = pathlib.Path(__file__).resolve().parents[1]
SERVICE = ROOT / "src" / "fleet_ros" / "fleet_ros" / "task_service_node.py"
ALLOCATOR = ROOT / "src" / "fleet_core" / "fleet_core" / "allocator.py"


def _function(path: pathlib.Path, name: str) -> ast.FunctionDef:
    tree = ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
    for node in ast.walk(tree):
        if isinstance(node, ast.FunctionDef) and node.name == name:
            return node
    raise AssertionError(f"{name} not found in {path.name}")


def _guarded_attributes(fn: ast.FunctionDef) -> set[str]:
    """Attributes appearing in a `not X.attr` test anywhere in the function."""
    out: set[str] = set()
    for node in ast.walk(fn):
        if isinstance(node, ast.If) and isinstance(node.test, ast.UnaryOp) \
                and isinstance(node.test.op, ast.Not):
            operand = node.test.operand
            if isinstance(operand, ast.Attribute):
                out.add(operand.attr)
    return out


def _attributes(fn: ast.FunctionDef) -> set[str]:
    """Every attribute read in the function, however it is combined.

    `guarded_attributes` only sees `not X.attr`; the pre-existing checks combine several
    attributes with `or` (`not fresh or state.fault_code or rid in self.parked`), so a
    test written against it alone reported that `fault_code` was unchecked. The helper was
    wrong, not the code.
    """
    return {n.attr for n in ast.walk(fn) if isinstance(n, ast.Attribute)}


def test_eligibility_refuses_a_robot_with_no_position():
    guarded = _guarded_attributes(_function(SERVICE, "_eligible_robots"))
    assert "position_known" in guarded, (
        "`_eligible_robots` does not check `position_known`, and for a PINNED task it is "
        "the only gate: the pinned branch of `_assign_ready` never reaches "
        "`Allocator.candidates`, so a system-generated charge run is offered to a robot "
        "whose own state says it has no position yet"
    )


def test_eligibility_still_refuses_the_things_it_already_did():
    """The tightening must not have replaced the earlier checks."""
    attrs = _attributes(_function(SERVICE, "_eligible_robots"))
    for name in ("fault_code", "operating_state", "needs_charge", "position_known"):
        assert name in attrs, f"`_eligible_robots` no longer looks at `{name}`"


def test_the_allocator_still_checks_it_too():
    """Both paths must check it: the allocator for ordinary tasks, the dispatcher for
    pinned ones. One check for a rule with two entry points is how the second entry point
    ends up unguarded."""
    assert "position_known" in _guarded_attributes(_function(ALLOCATOR, "candidates"))
