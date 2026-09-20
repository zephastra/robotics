"""P5 restart labelling: a parked task must say it was a restart.

`_reconcile_at_boot` moves every task left open by a dead task service to
NEEDS_ATTENTION. It used to record `CANCEL_NOT_CONFIRMED` for all of them, and
nothing had been cancelled. This file exists because that is the same shape as two
failures this project already paid for:

  * 008's `fail_reason` column, which was one constant for a week, so the record
    could not be used to tell causes apart;
  * P5's `_park`, whose reason was likewise hardcoded, and which is discussed in
    docs/LIMITATIONS.md section 4 ("a failure label that is a constant").

A text-level assertion would pass on the comment that explains the fix, so the check
below walks the AST and reads the actual keyword argument.
"""

from __future__ import annotations

import ast
import pathlib

ROOT = pathlib.Path(__file__).resolve().parents[1]
SERVICE = ROOT / "src" / "fleet_ros" / "fleet_ros" / "task_service_node.py"
DOMAIN = ROOT / "src" / "fleet_core" / "fleet_core" / "domain.py"


def _reason_constants() -> set[str]:
    tree = ast.parse(DOMAIN.read_text(encoding="utf-8"), filename=str(DOMAIN))
    for node in ast.walk(tree):
        if isinstance(node, ast.ClassDef) and node.name == "ReasonCode":
            return {
                t.targets[0].id
                for t in node.body
                if isinstance(t, ast.Assign) and isinstance(t.targets[0], ast.Name)
            }
    raise AssertionError("ReasonCode not found in domain.py")


def _function(name: str) -> ast.FunctionDef:
    tree = ast.parse(SERVICE.read_text(encoding="utf-8"), filename=str(SERVICE))
    for node in ast.walk(tree):
        if isinstance(node, ast.FunctionDef) and node.name == name:
            return node
    raise AssertionError(f"{name} not found in {SERVICE.name}")


def _reason_keywords(node: ast.AST) -> "list[str]":
    """Every `reason=ReasonCode.X` written anywhere inside `node`."""
    out: list[str] = []
    for sub in ast.walk(node):
        if not isinstance(sub, ast.Call):
            continue
        for kw in sub.keywords:
            if kw.arg != "reason":
                continue
            value = kw.value
            if isinstance(value, ast.Attribute):
                out.append(value.attr)
            elif isinstance(value, ast.Constant):
                out.append(repr(value.value))
    return out


def test_the_restart_code_exists():
    assert "INTERRUPTED_BY_RESTART" in _reason_constants()


def test_the_reconcile_path_labels_a_restart_as_a_restart():
    reasons = _reason_keywords(_function("_reconcile_at_boot"))
    assert reasons, "the reconcile path writes no reason at all"
    assert "CANCEL_NOT_CONFIRMED" not in reasons, (
        "a task parked by a restart is being labelled as an unconfirmed cancel; "
        "nothing was cancelled, and a constant label is what cost 008 a week"
    )
    assert "INTERRUPTED_BY_RESTART" in reasons


def test_the_restart_code_is_not_borrowed_by_the_cancel_path():
    """The cancel path must keep its own label.

    If `CANCEL_UNCONFIRMED` disappeared from `_enforce_cancel_deadlines` while
    `INTERRUPTED_BY_RESTART` appeared here, the two causes would have swapped, which
    is the same defect with the sign flipped.
    """
    reasons = _reason_keywords(_function("_enforce_cancel_deadlines"))
    assert any(r in ("CANCEL_UNCONFIRMED", "CANCEL_NOT_CONFIRMED") for r in reasons), (
        f"the cancel deadline path lost its cancel label: {reasons}"
    )
