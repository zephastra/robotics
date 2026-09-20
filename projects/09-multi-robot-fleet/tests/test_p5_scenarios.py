"""P5 scenario plumbing: the battery injection hook and the declared budgets.

No ROS, no simulator, following the same rule as the rest of tests/: the whole file
runs in well under a second, because these pin two things that used to be believed
rather than checked.

  1. **A declared budget is consumed.** CONTRACTS section 9 lets a scenario declare
     its own budgets and forbids adding one at runtime. The launch file used to
     forward only `tick_hz` and `pose_timeout_s`, so a scenario naming
     `leg_timeout_s` or `cancel_confirm_s` changed nothing while the run reported
     those numbers as being in force. `check_scenario_budgets.py` is the guard; these
     tests prove the guard is *falsifiable* rather than merely present, which is a
     distinction this project has paid for twice (a guard whose answer depended on
     the shell, and a guard reading a field nobody wrote).

  2. **The injection hook cannot empty a pack by accident.** The obvious design was
     a negative sentinel in a numeric field, where a caller that forgot to set it
     would ask for 0% state of charge. The shipped design is an explicit boolean, and
     the assertion below is structural: it reads the adapter's own AST and requires
     the early return, so removing the gate is a test failure rather than a silent
     behaviour change.
"""

from __future__ import annotations

import ast
import importlib.util
import pathlib
import sys

import pytest
import yaml

ROOT = pathlib.Path(__file__).resolve().parents[1]
LAUNCH = ROOT / "src" / "fleet_bringup" / "launch" / "fleet.launch.py"
ADAPTER = ROOT / "src" / "fleet_ros" / "fleet_ros" / "nav2_adapter_node.py"
SRV = ROOT / "src" / "fleet_interfaces" / "srv" / "FaultInject.srv"
SCENARIO_DIR = ROOT / "config" / "scenarios"


def load_guard():
    """Import scripts/check_scenario_budgets.py by path (it is not a package)."""
    path = ROOT / "scripts" / "check_scenario_budgets.py"
    spec = importlib.util.spec_from_file_location("_check_scenario_budgets", path)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


@pytest.fixture(scope="module")
def guard():
    return load_guard()


# --------------------------------------------------------------------------- #
# the guard actually sees the real files
# --------------------------------------------------------------------------- #


def test_the_guard_resolves_executables_to_the_module_that_declares_them(guard):
    """Both install mechanisms must be understood.

    `src/fleet_ros/setup.py` is a leftover ament_python scaffold and names only four
    of that package's six executables; the real installation is the package's
    CMakeLists.txt installing `scripts/*`. A resolver that read only setup.py made
    the guard announce that three executables did not exist -- the guard's own bug,
    reported as a project fault, which is the worst kind of failure a guard can have.
    """
    sources = guard.entry_points(ROOT)
    for exe in ("coordinator_node", "nav2_adapter_node", "task_service_node"):
        assert exe in sources, f"{exe} was not resolved to a source file"
    assert sources["nav2_adapter_node"] == ADAPTER
    assert "start_battery_fraction" in guard.declared_params(ADAPTER)


def test_the_guard_reads_the_launch_table_despite_the_annotation(guard):
    """The table is written as `NAME: "..." = {...}`, an annotated assignment."""
    table, robot_keys = guard.scenario_table(LAUNCH)
    assert sorted(table) == ["coordinator", "leg_executor", "task_service"]
    assert robot_keys == ("start_battery_fraction",)
    # The first element is the EXECUTABLE, not the node name. Writing the node name
    # here produced "this executable does not exist" for all three sections.
    assert table["coordinator"][0] == "coordinator_node"
    assert table["leg_executor"][0] == "nav2_adapter_node"
    assert "cancel_confirm_s" in table["task_service"][1]


def test_every_shipped_scenario_is_connected_end_to_end(guard):
    assert guard.main(["--root", str(ROOT)]) == 0


# --------------------------------------------------------------------------- #
# ... and that it can fail. A guard that cannot fail is not a guard.
# --------------------------------------------------------------------------- #


def test_the_value_checks_can_fail(guard):
    findings = guard._check_value("s.yaml", "budgets.task_service.tick_hz", "tick_hz", -1.0)
    assert findings, "a non-positive tick_hz must be a finding"
    findings = guard._check_value("s.yaml", "x", "start_battery_fraction", 3.0)
    assert findings, "a fraction outside [0,1] must be a finding"
    findings = guard._check_value("s.yaml", "x", "tick_hz", float("nan"))
    assert findings, "NaN must be refused; CONTRACTS section 2 refuses it in config"


def test_the_guard_refuses_a_scenario_that_names_an_unforwarded_section(guard, tmp_path):
    """End to end through main(): a bad file in a copied config must fail the run."""
    import shutil

    fake_root = tmp_path / "root"
    shutil.copytree(ROOT / "src", fake_root / "src", symlinks=True)
    (fake_root / "config" / "scenarios").mkdir(parents=True)
    (fake_root / "config" / "scenarios" / "bad.yaml").write_text(
        'schema_version: "1"\nbudgets:\n  no_such_node:\n    tick_hz: 2.0\n',
        encoding="utf-8",
    )
    assert guard.main(["--root", str(fake_root)]) == 1


# --------------------------------------------------------------------------- #
# the battery injection hook
# --------------------------------------------------------------------------- #


def _srv_request_fields(path: pathlib.Path) -> list[str]:
    """Field names in the request half of a .srv.

    Split on a line that IS `---`, not on the first occurrence of that text: the file
    contains `# --- state-of-charge override ...` in a comment, and `str.partition`
    happily split there and reported four fields for a service that has six.
    """
    fields: list[str] = []
    for line in path.read_text(encoding="utf-8").splitlines():
        stripped = line.strip()
        if stripped == "---":
            break
        if not stripped or stripped.startswith("#"):
            continue
        parts = stripped.split()
        if len(parts) >= 2:
            fields.append(parts[1])
    return fields


def test_the_service_declares_an_explicit_flag_rather_than_a_sentinel():
    fields = _srv_request_fields(SRV)
    assert "inject_battery" in fields, f"a boolean flag must gate the override; got {fields}"
    assert "battery_fraction" in fields
    text = SRV.read_text(encoding="utf-8")
    assert "bool inject_battery" in text
    # The flag has to come first, so a message built field-by-field cannot set the
    # value while leaving the gate defaulted off without noticing.
    assert fields.index("inject_battery") < fields.index("battery_fraction")
    assert "bool ok" in text and "string message" in text


def test_the_adapter_refuses_to_touch_the_pack_without_the_flag():
    """Structural, on the AST: the early return must be the first statement.

    Reading the source as text would match the comment that explains it. Walking the
    function body cannot.
    """
    tree = ast.parse(ADAPTER.read_text(encoding="utf-8"), filename=str(ADAPTER))
    fn = None
    for node in ast.walk(tree):
        if isinstance(node, ast.FunctionDef) and node.name == "_apply_battery_injection":
            fn = node
    assert fn is not None, "_apply_battery_injection is missing from the adapter"

    body = list(fn.body)
    # Skip the docstring: `body[0]` is an Expr holding the function's own explanation,
    # and asserting on it produced "expected an early return, got Expr".
    if body and isinstance(body[0], ast.Expr) and isinstance(body[0].value, ast.Constant) \
            and isinstance(body[0].value.value, str):
        body = body[1:]
    assert body, "the function has no statements after its docstring"

    first = body[0]
    assert isinstance(first, ast.If), f"expected an early return, got {type(first).__name__}"
    test = first.test
    assert isinstance(test, ast.UnaryOp) and isinstance(test.op, ast.Not)
    assert isinstance(test.operand, ast.Attribute)
    assert test.operand.attr == "inject_battery"
    returned = first.body[0]
    assert isinstance(returned, ast.Return) and returned.value is not None
    assert isinstance(returned.value, ast.Constant) and returned.value.value == ""


def test_the_adapter_undoes_the_override_when_a_duration_is_given():
    """A timed injection must be undoable, and the undo must happen on a tick.

    Without this half, a scenario could put a robot in the low band and never get it
    out again except by restarting the fleet, which makes the recovery half of the
    charge scenarios untestable.
    """
    text = ADAPTER.read_text(encoding="utf-8")
    assert "_expire_battery_injection" in text
    assert "self._expire_battery_injection(now)" in text, (
        "the restore is never called from _tick, so a duration would be ignored"
    )
    tree = ast.parse(text, filename=str(ADAPTER))
    names = {n.name for n in ast.walk(tree) if isinstance(n, ast.FunctionDef)}
    assert "_expire_battery_injection" in names


# --------------------------------------------------------------------------- #
# the shipped scenarios themselves
# --------------------------------------------------------------------------- #


def test_the_four_p5_scenarios_are_present_and_well_formed():
    expected = {"p5_capability", "p5_low_battery", "p5_charge_queue", "p5_restart"}
    found = {p.stem for p in SCENARIO_DIR.glob("p5_*.yaml")}
    assert expected <= found, f"missing scenario files: {sorted(expected - found)}"

    for path in sorted(SCENARIO_DIR.glob("p5_*.yaml")):
        body = yaml.safe_load(path.read_text(encoding="utf-8"))
        assert str(body.get("schema_version")) == "1", path.name
        assert body.get("description"), f"{path.name}: a scenario must say what it claims"
        assert body.get("budgets"), f"{path.name}: budgets must be declared up front"
        for rid, values in (body.get("robots") or {}).items():
            for key, val in values.items():
                assert key == "start_battery_fraction", f"{path.name}: {key}"
                assert 0.0 <= float(val) <= 1.0, f"{path.name}: {rid} {key}={val}"


def test_the_low_battery_scenario_sits_inside_the_low_band_and_not_in_critical():
    """The scenario is only meaningful if the thresholds in config actually bracket it.

    `low_fraction` is 0.20 and `critical_fraction` is 0.08 in config/fleet.yaml. A
    scenario written at 0.05 would take the reachability path instead, and would be
    testing a different claim than the one its description makes.
    """
    cfg = yaml.safe_load((ROOT / "config" / "fleet.yaml").read_text(encoding="utf-8"))
    battery = cfg["battery"]
    low, critical = float(battery["low_fraction"]), float(battery["critical_fraction"])

    for name in ("p5_low_battery", "p5_charge_queue"):
        body = yaml.safe_load((SCENARIO_DIR / f"{name}.yaml").read_text(encoding="utf-8"))
        for rid, values in (body.get("robots") or {}).items():
            fraction = float(values["start_battery_fraction"])
            assert critical < fraction <= low, (
                f"{name}: {rid} at {fraction} is not in (critical, low] = "
                f"({critical}, {low}]; the scenario would exercise a different rule"
            )
