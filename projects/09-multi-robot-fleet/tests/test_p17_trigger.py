"""Round 17: the `when <predicate> do <action>` vocabulary, and the guard that must agree with it.

TEST_AND_ACCEPTANCE section 5 states the rule this exists for:

    故障按状态触发而非某个固定 sleep ... 超时未到触发条件为 PRECONDITION_NOT_REACHED，
    不算故障测试通过

`fleet_core/trigger.py` is the vocabulary, `scripts/batch.py` is the only thing that carries a
step out, and `scripts/check_batch_manifest.py` is what keeps a manifest from naming an action,
a node or a behaviour check that does not exist. Three places, one vocabulary -- so this file
tests all three against each other as well as testing each on its own.

The mutation tests below break a COPY of the shipped manifest with the real contract, the real
limitations register and the real fleet config behind it, for the reason
`tests/test_p7_batch_manifest.py` gives: a guard that cannot fail is not a guard, and a test
that passes against a drifted fixture says nothing about the shipped one.
"""

from __future__ import annotations

import sys
from pathlib import Path

import pytest
import yaml

from fleet_core import trigger, wait_for

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))

from check_batch_manifest import (  # noqa: E402
    launch_node_names,
    load_manifest,
    runner_actions,
    runner_declared_checks,
    runner_wait_keys,
)

MANIFEST = ROOT / "config" / "scenarios" / "regression_v1.yaml"


# --------------------------------------------------------------------------- #
# helpers
# --------------------------------------------------------------------------- #


def _mutate(tmp_path: Path, change) -> Path:
    """Write a manifest that differs from the shipped one in exactly one way."""
    body = yaml.safe_load(MANIFEST.read_text(encoding="utf-8"))
    change(body)
    out = tmp_path / "regression_v1.yaml"
    out.write_text(yaml.safe_dump(body, sort_keys=False), encoding="utf-8")
    return out


def _refused(tmp_path: Path, change) -> str:
    """Run the guard on a mutated manifest and return what it complained about.

    The findings are printed, not raised: `SystemExit` carries only a count. Capture the
    output, not the exception.
    """
    import contextlib
    import io

    buffer = io.StringIO()
    path = _mutate(tmp_path, change)
    with contextlib.redirect_stdout(buffer), pytest.raises(SystemExit):
        load_manifest(path, ROOT)
    return buffer.getvalue()


def _steps(tmp_path: Path, steps: list) -> str:
    """The guard's answer to a case carrying exactly these steps."""
    return _refused(tmp_path, lambda body: body["cases"]["N02"].update({"steps": steps}))


def _accepted(tmp_path: Path, change) -> None:
    """Run the guard on a manifest that must be fine. Must NOT raise."""
    path = _mutate(tmp_path, change)
    load_manifest(path, ROOT)


GOOD_STEP = {"when": {"robot_carrying": "r01"},
             "do": {"action": "battery_inject", "runner": "r01", "fraction": 0.12}}


# --------------------------------------------------------------------------- #
# the vocabulary itself
# --------------------------------------------------------------------------- #


def test_the_four_actions_validate_cleanly():
    steps = [
        {"when": {"robot_carrying": "r01"},
         "do": {"action": "battery_inject", "runner": "r01", "fraction": 0.12}},
        {"when": {"request_id": "x", "state": "EXECUTING"},
         "do": {"action": "cancel_task", "request_id": "x"}},
        {"when": {"robot_crossing": "r01"},
         "do": {"action": "kill_process", "runner": "r01", "node": "all"}},
        {"do": {"action": "restart_process", "node": "fleet_task_service"}},
    ]
    for step in steps:
        assert trigger.validate(step, ["r01"]) == []


def test_a_step_needs_a_do_mapping():
    assert "needs a `do` mapping" in " ".join(trigger.validate({"when": {}}, ["r01"]))
    assert "must be a mapping" in " ".join(trigger.validate("kill r01", ["r01"]))


def test_an_unknown_action_is_refused_rather_than_ignored():
    findings = trigger.validate({"do": {"action": "teleport", "runner": "r01"}}, ["r01"])
    assert any("not implemented" in f for f in findings)


def test_battery_inject_without_a_runner_is_refused():
    findings = trigger.validate({"do": {"action": "battery_inject", "fraction": 0.2}}, ["r01"])
    assert any("needs a `runner`" in f for f in findings)


def test_killing_the_whole_namespace_without_a_runner_is_refused():
    """`all` with no namespace would address every node in the fleet."""
    findings = trigger.validate({"do": {"action": "kill_process", "node": "all"}}, ["r01"])
    assert any("every node in the fleet" in f for f in findings)


def test_a_step_naming_a_robot_the_case_does_not_run_is_refused():
    findings = trigger.validate(
        {"do": {"action": "kill_process", "runner": "r99", "node": "all"}}, ["r01"])
    assert any("r99" in f and "does not run" in f for f in findings)


def test_describe_names_the_action_and_the_condition():
    text = trigger.describe(GOOD_STEP)
    assert "battery_inject" in text and "r01" in text and "robot_carrying" in text
    assert "as soon as the fleet is ready" in trigger.describe(
        {"do": {"action": "cancel_task", "request_id": "x"}})


# --------------------------------------------------------------------------- #
# the predicate the two carrying faults need
# --------------------------------------------------------------------------- #


def _snapshot(payloads, tasks=(), crossings=()):
    return {
        "tasks": list(tasks),
        "crossings": {r: {} for r in crossings},
        "payloads": list(payloads),
    }


def test_robot_carrying_holds_only_while_the_ledger_says_held():
    held = _snapshot([{"payload_id": "c1", "state": "HELD", "holder": "r01"}])
    assert wait_for.condition_holds(held, {"robot_carrying": "r01"}).holds
    assert not wait_for.condition_holds(held, {"robot_carrying": "r02"}).holds

    delivered = _snapshot([{"payload_id": "c1", "state": "DELIVERED", "holder": "r01"}])
    assert not wait_for.condition_holds(delivered, {"robot_carrying": "r01"}).holds


def test_a_held_payload_with_no_holder_is_not_a_carrier():
    """A row that says HELD but names nobody must not satisfy "that robot is carrying it"."""
    loose = _snapshot([{"payload_id": "c1", "state": "HELD", "holder": ""}])
    assert not wait_for.condition_holds(loose, {"robot_carrying": "r01"}).holds


def test_an_unknown_key_is_still_refused():
    verdict = wait_for.condition_holds(_snapshot([]), {"robot_holding_permit": "r01"})
    assert not verdict.holds and verdict.impossible


def test_the_two_vocabularies_are_the_same_one():
    """The guard parses `KNOWN_KEYS` out of the file; it must see what the module defines."""
    found, keys = runner_wait_keys(ROOT)
    assert found
    assert set(keys) == set(wait_for.KNOWN_KEYS)


# --------------------------------------------------------------------------- #
# the guard's readers: a check that cannot read its vocabulary must refuse
# --------------------------------------------------------------------------- #


def test_the_guard_sees_the_actions_the_module_defines():
    found, actions, fields, target_all = runner_actions(ROOT)
    assert found
    assert set(actions) == set(trigger.KNOWN_ACTIONS)
    assert set(fields) == set(trigger.REQUIRED_FIELDS)
    assert target_all == trigger.TARGET_ALL


def test_the_guard_sees_every_declared_behaviour_check():
    found, names = runner_declared_checks(ROOT)
    assert found
    assert set(names) == set(batch_declared_checks())


def batch_declared_checks() -> "tuple[str, ...]":
    """`DECLARED_CHECKS` from `scripts/batch.py`, read without importing it (it needs rclpy)."""
    import ast

    tree = ast.parse((ROOT / "scripts" / "batch.py").read_text(encoding="utf-8"))
    for node in ast.walk(tree):
        if isinstance(node, ast.Assign) and any(
            isinstance(t, ast.Name) and t.id == "DECLARED_CHECKS" for t in node.targets
        ) and isinstance(node.value, ast.Tuple):
            return tuple(sorted(e.value for e in node.value.elts
                                if isinstance(e, ast.Constant)))
    raise AssertionError("scripts/batch.py declares no DECLARED_CHECKS")


def test_the_guard_sees_the_node_names_the_launches_start():
    found, names = launch_node_names(ROOT)
    assert found
    for expected in ("safety_gate", "gz_bridge", "amcl", "bt_navigator",
                     "nav2_adapter", "fleet_task_service", "fleet_coordinator"):
        assert expected in names, f"{expected} is started by a launch but the guard cannot see it"


# --------------------------------------------------------------------------- #
# the guard refuses a manifest that names something that does not exist
# --------------------------------------------------------------------------- #


def test_the_shipped_manifest_is_still_accepted():
    load_manifest(MANIFEST, ROOT)          # must not raise


def test_a_step_with_an_unimplemented_action_is_refused(tmp_path):
    output = _steps(tmp_path, [{"do": {"action": "teleport", "runner": "r01"}}])
    assert "not implemented" in output


def test_a_step_with_an_unknown_condition_key_is_refused(tmp_path):
    output = _steps(tmp_path, [{"when": {"robot_holding_permit": "r01"},
                                "do": {"action": "cancel_task", "request_id": "x"}}])
    assert "does not know" in output


def test_a_state_condition_without_a_request_id_is_refused(tmp_path):
    output = _steps(tmp_path, [{"when": {"state": "EXECUTING"},
                                "do": {"action": "cancel_task", "request_id": "x"}}])
    assert "needs a `request_id`" in output


def test_a_step_naming_a_node_no_launch_starts_is_refused(tmp_path):
    output = _steps(tmp_path, [{"do": {"action": "kill_process", "runner": "r01",
                                      "node": "hyperdrive"}}])
    assert "which no launch file starts" in output


def test_a_step_naming_a_robot_the_case_does_not_run_is_refused(tmp_path):
    output = _steps(tmp_path, [{"do": {"action": "kill_process", "runner": "r02",
                                      "node": "all"}}])
    assert "which the case does not run" in output


def test_a_missing_required_field_is_refused(tmp_path):
    output = _steps(tmp_path, [{"do": {"action": "battery_inject", "runner": "r01"}}])
    assert "needs `fraction`" in output


def test_an_unimplemented_behaviour_check_is_refused(tmp_path):
    output = _refused(tmp_path,
                      lambda body: body["cases"]["N02"].update(
                          {"behavior_checks": ["payload_not_rehomed", "vibes_are_good"]}))
    assert "vibes_are_good" in output and "not implemented" in output


def test_a_correctly_written_step_is_accepted(tmp_path):
    """The negative tests above are only meaningful if a good step gets through."""
    _accepted(tmp_path, lambda body: body["cases"]["N02"].update({"steps": [GOOD_STEP]}))
