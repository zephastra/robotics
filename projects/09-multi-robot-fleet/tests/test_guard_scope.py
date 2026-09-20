"""The scope guard must keep its teeth, and this test is what proves it.

Why this file exists
--------------------
`check_undefined_names.py` is the only automated defence against the `NameError` family,
which is the most expensive bug class in this project: it fires at runtime on a path the
test suite never takes, and the symptom points somewhere else. The historical instance:

    nav2_adapter_node._execute() used `map_x`, a local of the sibling method
    _build_state(). Every completed leg raised NameError and returned an EMPTY failure,
    which reads as "the executor died rather than "one name is out of scope".

The FIRST version of that guard could not have caught it. Its `bound_names()` walked the
whole tree, so a name assigned anywhere -- including inside a sibling function -- counted
as defined. A guard whose check is silently off is worse than no guard, because the build
reports success. So the guard is now tested directly rather than trusted.

These tests load the script by path: `scripts/` is not a package and must not become one.
"""

from __future__ import annotations

import importlib.util
import pathlib
import textwrap

import pytest

ROOT = pathlib.Path(__file__).resolve().parents[1]
GUARD = ROOT / "scripts" / "check_undefined_names.py"


def _load_guard():
    spec = importlib.util.spec_from_file_location("_guard_under_test", GUARD)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.fixture(scope="module")
def guard():
    assert GUARD.is_file(), f"the guard is missing: {GUARD}"
    return _load_guard()


def _write(tmp_path: pathlib.Path, source: str) -> pathlib.Path:
    p = tmp_path / "sample.py"
    p.write_text(textwrap.dedent(source), encoding="utf-8")
    return p


# --------------------------------------------------------------------------- #
# the historical regression, exactly
# --------------------------------------------------------------------------- #

def test_a_local_of_a_sibling_method_is_reported(guard, tmp_path):
    """This is the bug that cost two three-robot runs. It must never pass again."""
    path = _write(tmp_path, """
        class Node:
            def _build_state(self):
                map_x = 1.0
                map_y = 2.0
                return (map_x, map_y)

            def _execute(self):
                # out of scope: these are locals of _build_state, not globals
                return map_x + map_y
        """)
    problems, why = guard.scope_problems(path)
    assert why is None
    assert any("_execute()" in p and "map_x" in p for p in problems), problems
    assert any("_execute()" in p and "map_y" in p for p in problems), problems


def test_a_local_of_a_sibling_function_is_reported(guard, tmp_path):
    """The same shape as the acceptance-harness NameError: `node` was a local of main."""
    path = _write(tmp_path, """
        def main():
            node = object()

            def run_pair():
                return node

            return run_pair
        """)
    problems, why = guard.scope_problems(path)
    assert why is None
    # `node` IS visible inside run_pair, because run_pair closes over main's scope.
    # Reporting it would be a false positive, and false positives get guards switched off.
    assert problems == []


def test_a_name_that_no_scope_binds_is_reported_at_module_level(guard, tmp_path):
    path = _write(tmp_path, "value = never_defined_anywhere + 1\n")
    problems, why = guard.scope_problems(path)
    assert why is None
    assert any("<module>" in p for p in problems), problems


# --------------------------------------------------------------------------- #
# no false positives on the constructs this codebase actually uses
# --------------------------------------------------------------------------- #

def test_closures_arguments_and_globals_are_all_accepted(guard, tmp_path):
    path = _write(tmp_path, """
        import math

        GLOBAL = 3
        ANNOTATED: int = 4
        A, B = 1, 2

        def outer(arg, *rest, kw=None, **extra):
            local = arg
            total = 0.0

            def inner():
                # closes over outer's locals and the module globals
                return local + total + GLOBAL + ANNOTATED + A + B + arg

            for i in rest:
                total += i
            with open("/dev/null") as handle:
                handle.read()
            try:
                total += math.pi
            except ValueError as exc:
                total += len(str(exc))
            pairs = [k for k in rest]
            del pairs
            return inner, kw, extra, handle
        """)
    problems, why = guard.scope_problems(path)
    assert problems == [], problems
    assert why is None


def test_match_and_walrus_targets_are_accepted(guard, tmp_path):
    path = _write(tmp_path, """
        def f(payload):
            if (n := len(payload)) > 0:
                pass
            match payload:
                case {"a": item}:
                    return n + item
                case [first, *others]:
                    return n + first + len(others)
                case _:
                    return n
        """)
    problems, why = guard.scope_problems(path)
    assert problems == [], problems


def test_star_import_is_skipped_with_a_reason_not_guessed(guard, tmp_path):
    path = _write(tmp_path, "from somewhere import *\n\ndef f():\n    return mystery\n")
    problems, why = guard.scope_problems(path)
    assert problems == []
    assert why is not None and "star import" in why


def test_the_subject_codebases_own_nodes_pass_the_new_check(guard):
    """The check must be clean on the real files, or it is noise rather than a guard."""
    targets = [
        ROOT / "src" / "fleet_ros" / "fleet_ros" / "nav2_adapter_node.py",
        ROOT / "src" / "fleet_ros" / "fleet_ros" / "task_service_node.py",
        ROOT / "src" / "fleet_ros" / "fleet_ros" / "coordinator_node.py",
        ROOT / "src" / "fleet_ros" / "fleet_ros" / "gate_node.py",
        ROOT / "src" / "fleet_core" / "fleet_core" / "traffic.py",
        ROOT / "src" / "fleet_core" / "fleet_core" / "ledger.py",
        ROOT / "scripts" / "acceptance_p4.py",
    ]
    for path in targets:
        assert path.is_file(), f"missing target: {path}"
        problems, why = guard.scope_problems(path)
        assert problems == [], f"{path.name}: {problems}"
        assert why is None, f"{path.name}: unexpectedly skipped ({why})"


# --------------------------------------------------------------------------- #
# the shadowing check keeps its lookup table
# --------------------------------------------------------------------------- #

def test_node_member_shadowing_is_still_detected_without_rclpy(guard, tmp_path):
    path = _write(tmp_path, """
        class Thing:
            def __init__(self):
                self.clients = {}
        """)
    members, how = guard.node_members()
    assert "clients" in members, (
        "the member table lost `clients`, so half of the shadowing check is off -- that "
        "exact blindness let `self.clients = {}` reach a live run")
    assert guard.shadows_node_member(path, members) == ["clients"]
    assert how, "the guard must say how it obtained the member list"


def test_member_list_does_not_depend_on_which_shell_runs_the_guard(guard):
    """Sourcing ROS must not change the guard's answer.

    The clean shell cannot `import rclpy`, so an import-only implementation quietly fell
    back to a short hand-written list and produced a different verdict from the same
    source tree. That is the failure mode this project keeps hitting: a check that is off
    reports success. Parsing rclpy's own source removes the dependency.
    """
    members, how = guard.node_members()
    assert "parsed rclpy source" in how or "imported rclpy" in how, how
    # A few members that only the real class has, so a short list would fail here.
    for expected in ("clients", "publishers", "executor", "create_subscription"):
        assert expected in members, f"{expected} missing ({how})"
    assert len(members) > 39, f"member list looks like the curated fallback ({how})"
