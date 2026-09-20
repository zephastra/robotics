#!/usr/bin/env python3
"""Check that every declared scenario budget is actually consumed, and vice versa.

WHY THIS EXISTS
---------------
CONTRACTS section 9 says a scenario may declare its own budgets up front and may not
add them at runtime. It does not say the launch has to honour them -- and it did not.
Until this round, `fleet.launch.py` forwarded only `tick_hz` and `pose_timeout_s`, so
a scenario naming `leg_timeout_s`, `cancel_confirm_s`, `max_retries` or a starting
state of charge changed nothing at all, while the run's own report would have said
those budgets were in force. That is the same shape as every other expensive mistake
in this project: a value that is *believed* to be connected and is not.

So this guard closes the loop in both directions:

  * a key named in a scenario file that this launch does not forward  -> FAIL
  * a key the launch claims to forward that its node never declares   -> FAIL
  * a table entry whose executable cannot be resolved to a source file -> FAIL
  * a key listed in SCENARIO_NODES that the launch's own `parameters=[...]` never
    sets                                                              -> FAIL; the node would
    keep its default and a scenario naming the key would change nothing, while the run's
    report quoted the value as in force. Added after round 16c did exactly that with
    `crossing_queue_s` and this guard passed.
  * a threshold that is NaN, infinite, or non-positive where a duration is
    meant                                                             -> FAIL

It reads `SCENARIO_NODES` out of the launch file itself rather than repeating it, so
the table cannot drift away from what is checked.

Usage:
    scripts/check_scenario_budgets.py [--root DIR]
Exit code 0 if clean, 1 on any finding.
"""

from __future__ import annotations

import argparse
import ast
import math
import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]

#: executable name -> module, from a package's setup.py console_scripts.
_ENTRY = re.compile(r"['\"](\w+)\s*=\s*([\w.]+)\s*:\s*main['\"]")
#: `from fleet_ros.nav2_adapter_node import main` inside an ament_cmake wrapper.
_IMPORT_MAIN = re.compile(r"from\s+([\w.]+)\s+import\s+main")

#: Keys that must be a strictly positive duration/rate, not just a finite float.
MUST_BE_POSITIVE = {
    "tick_hz", "permit_rate_hz", "publish_hz", "pose_timeout_s",
    "occupancy_settle_s", "leg_timeout_s", "cancel_confirm_s", "orphan_grace_s",
    "pinned_grace_s",
}
#: Keys that must lie in [0, 1].
MUST_BE_FRACTION = {"start_battery_fraction"}


def entry_points(root: Path) -> dict[str, Path]:
    """executable name -> the source file that declares its parameters.

    Two installation mechanisms live in this workspace and both have to be understood,
    because reading only one of them made an earlier version of this guard report
    "the table and the packages have drifted apart" for three executables that were
    sitting in `src/fleet_ros/scripts/` and are installed by that package's
    CMakeLists.txt. `src/fleet_ros/setup.py` is a leftover from the ament_python
    scaffold and names only four of the six, so it is not the authority it looks like.

    Resolution order per executable:
      1. a `console_scripts` entry in some package's setup.py (ament_python), or
      2. a wrapper under `src/<pkg>/scripts/<name>`, followed through its
         `from <pkg>.<module> import main` line to the module that holds the policy.
    """
    found: dict[str, Path] = {}
    for setup in sorted(root.glob("src/*/setup.py")):
        text = setup.read_text(encoding="utf-8")
        for exe, dotted in _ENTRY.findall(text):
            parts = dotted.split(".")
            if len(parts) < 3:
                continue
            candidate = root / "src" / parts[0] / parts[1] / f"{parts[-1]}.py"
            if candidate.is_file():
                found[exe] = candidate

    for wrapper in sorted(root.glob("src/*/scripts/*")):
        # Installed executables have no suffix; `scripts/` also holds helpers that do.
        if not wrapper.is_file() or wrapper.suffix:
            continue
        pkg = wrapper.parent.parent.name
        text = wrapper.read_text(encoding="utf-8", errors="replace")
        candidates: list[Path] = []
        match = _IMPORT_MAIN.search(text)
        if match:
            dotted = match.group(1).split(".")
            if len(dotted) >= 2:
                candidates.append(root / "src" / pkg / dotted[0] / f"{dotted[-1]}.py")
        candidates.append(root / "src" / pkg / pkg / f"{wrapper.name}.py")
        for candidate in candidates:
            if candidate.is_file():
                found[wrapper.name] = candidate
                break
    return found


def declared_params(path: Path) -> set[str]:
    tree = ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
    out: set[str] = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.Call):
            fn = node.func
            name = getattr(fn, "attr", None) or getattr(fn, "id", None)
            if name == "declare_parameter" and node.args:
                arg = node.args[0]
                if isinstance(arg, ast.Constant) and isinstance(arg.value, str):
                    out.add(arg.value)
    return out


def scenario_table(launch: Path) -> tuple[dict[str, tuple[str, tuple[str, ...]]], tuple[str, ...]]:
    """Read SCENARIO_NODES / SCENARIO_ROBOT_KEYS out of the launch file.

    Handles both `NAME = {...}` and the annotated `NAME: "..." = {...}` form, because
    the launch file uses the annotated one and a reader that only understood plain
    assignment reported "could not read the table" while the table sat right there.
    """
    tree = ast.parse(launch.read_text(encoding="utf-8"), filename=str(launch))
    table: dict[str, tuple[str, tuple[str, ...]]] = {}
    robot_keys: tuple[str, ...] = ()
    for node in ast.walk(tree):
        target = None
        if isinstance(node, ast.Assign):
            names = [t.id for t in node.targets if isinstance(t, ast.Name)]
            target = names[0] if len(names) == 1 else None
        elif isinstance(node, ast.AnnAssign) and isinstance(node.target, ast.Name):
            target = node.target.id
        if target is None:
            continue
        value = node.value

        if target == "SCENARIO_NODES" and isinstance(value, ast.Dict):
            for key, val in zip(value.keys, value.values):
                if not (isinstance(key, ast.Constant) and isinstance(val, ast.Tuple)):
                    continue
                if not (len(val.elts) == 2 and isinstance(val.elts[1], ast.Tuple)):
                    continue
                exe = val.elts[0]
                keys = tuple(
                    e.value for e in val.elts[1].elts
                    if isinstance(e, ast.Constant) and isinstance(e.value, str)
                )
                if isinstance(exe, ast.Constant) and isinstance(exe.value, str):
                    table[key.value] = (exe.value, keys)
        elif target == "SCENARIO_ROBOT_KEYS" and isinstance(value, ast.Tuple):
            robot_keys = tuple(
                e.value for e in value.elts
                if isinstance(e, ast.Constant) and isinstance(e.value, str)
            )
    return table, robot_keys


def applied_params(launch: Path) -> dict[str, set[str]]:
    """executable name -> the keys its launch `Node` really puts in `parameters=[...]`.

    WHY THIS IS SEPARATE FROM THE TABLE CHECK
    -----------------------------------------
    `SCENARIO_NODES` says which keys a section MAY carry, and the node's `declare_parameter`
    calls say which it understands. Neither says the launch PASSES the value, because the
    launch builds each node's parameter dict by hand. A key can therefore be in the table,
    declared by the node, and still never applied -- and that is not hypothetical: round 16c
    added `crossing_queue_s` to the table and to the node, forgot this dict, and the driver
    reported `queue_allowance_s = 0.0` while the scenario declared 90 s.

    This is the half of the check the guard was missing, in the exact shape of its own
    purpose ("a value that is *believed* to be connected and is not"), so it is written as
    structure and not as a search for the name in the file.
    """
    tree = ast.parse(launch.read_text(encoding="utf-8"), filename=str(launch))
    out: dict[str, set[str]] = {}
    for node in ast.walk(tree):
        if not isinstance(node, ast.Call):
            continue
        func = getattr(node.func, "attr", None) or getattr(node.func, "id", None)
        if func != "Node":
            continue
        exe = None
        for kw in node.keywords:
            if kw.arg == "executable" and isinstance(kw.value, ast.Constant):
                exe = kw.value.value
        if not isinstance(exe, str):
            continue
        keys: set[str] = set()
        for kw in node.keywords:
            if kw.arg != "parameters" or not isinstance(kw.value, ast.List):
                continue
            for element in kw.value.elts:
                if not isinstance(element, ast.Dict):
                    continue
                for key in element.keys:
                    if isinstance(key, ast.Constant) and isinstance(key.value, str):
                        keys.add(key.value)
        out.setdefault(exe, set()).update(keys)
    return out


def main(argv: "list[str] | None" = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--root", default=str(ROOT))
    args = ap.parse_args(argv)

    root = Path(args.root).resolve()
    launch = root / "src" / "fleet_bringup" / "launch" / "fleet.launch.py"
    scen_dir = root / "config" / "scenarios"

    print("=" * 78)
    print("  009 scenario budget check (static)")
    print("=" * 78)

    findings: list[str] = []

    if not launch.is_file():
        print(f"  [FAIL] launch file not found: {launch}")
        return 1
    table, robot_keys = scenario_table(launch)
    if not table:
        print("  [FAIL] could not read SCENARIO_NODES from fleet.launch.py")
        return 1

    exes = entry_points(root)
    print(f"  launch table: {len(table)} node section(s) {sorted(table)}")
    print(f"  per-robot keys: {list(robot_keys)}")

    # ---- the launch side: every claimed key must exist on the node -------- #
    declared_by_section: dict[str, set[str]] = {}
    for section, (exe, keys) in sorted(table.items()):
        source = exes.get(exe)
        if source is None:
            findings.append(
                f"fleet.launch.py: SCENARIO_NODES[{section!r}] names executable "
                f"{exe!r}, which no package's setup.py declares. The table and the "
                "packages have drifted apart."
            )
            continue
        declared = declared_params(source)
        declared_by_section[section] = declared
        for key in keys:
            if key not in declared:
                findings.append(
                    f"fleet.launch.py: SCENARIO_NODES[{section!r}] forwards {key!r} to "
                    f"{exe!r}, but {source.relative_to(root)} never calls "
                    f"declare_parameter({key!r}). The node would ignore it."
                )

    # ---- the launch side, part two: does the launch actually PASS it? ----- #
    # `SCENARIO_ROBOT_KEYS` are applied per robot rather than in the node's own dict (that is
    # what makes them per-robot), and they are validated against the leg_executor list just
    # below. Every other key in the table has to appear in the dict or a scenario naming it
    # changes nothing while its report quotes the value as in force.
    applied = applied_params(launch)
    for section, (exe, keys) in sorted(table.items()):
        have = applied.get(exe)
        if have is None:
            findings.append(
                f"fleet.launch.py: no `Node(executable={exe!r})` was found, so nothing can be "
                f"proved about the keys SCENARIO_NODES[{section!r}] claims to forward."
            )
            continue
        for key in keys:
            if key in robot_keys:
                continue
            if key not in have:
                findings.append(
                    f"fleet.launch.py: SCENARIO_NODES[{section!r}] lists {key!r} and {exe!r} "
                    f"declares it, but that node's own `parameters=[...]` never sets it. A "
                    "scenario naming this key would change nothing while its report quoted the "
                    "value as in force -- the exact failure this guard exists to refuse."
                )

    leg_keys = table.get("leg_executor", ("", ()))[1]
    for key in robot_keys:
        if key not in leg_keys:
            findings.append(
                f"fleet.launch.py: SCENARIO_ROBOT_KEYS names {key!r}, which is not in "
                f"the leg_executor key list {list(leg_keys)}; a per-robot override of "
                "it could never be applied."
            )

    # ---- the scenario side: every named key must be forwarded ------------- #
    if not scen_dir.is_dir():
        print("  no config/scenarios/ directory: nothing to check on the file side")
        files: "list[Path]" = []
    else:
        files = sorted(scen_dir.glob("*.yaml")) + sorted(scen_dir.glob("*.yml"))
        print(f"  scenario files: {len(files)}")

    import yaml  # local import: keeps --help working without PyYAML

    for path in files:
        body = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
        rel = path.relative_to(root)
        if str(body.get("schema_version", "")) != "1":
            findings.append(f"{rel}: schema_version must be the string \"1\"")

        budgets = body.get("budgets") or {}
        for section, values in budgets.items():
            if section not in table:
                findings.append(
                    f"{rel}: budgets.{section} names no node this launch forwards to; "
                    f"known sections are {sorted(table)}"
                )
                continue
            allowed = table[section][1]
            for key, val in sorted(values.items()):
                if key not in allowed:
                    findings.append(
                        f"{rel}: budgets.{section}.{key} is not forwarded by "
                        f"fleet.launch.py (forwarded: {list(allowed)}). Setting it "
                        "would change nothing while the run reported it as in force."
                    )
                if key not in declared_by_section.get(section, set()):
                    continue
                findings.extend(_check_value(rel, f"budgets.{section}.{key}", key, val))

        for rid, values in (body.get("robots") or {}).items():
            for key, val in sorted(values.items()):
                if key not in robot_keys:
                    findings.append(
                        f"{rel}: robots.{rid}.{key} is not a per-robot scenario key "
                        f"({list(robot_keys)}); it would be ignored"
                    )
                    continue
                findings.extend(_check_value(rel, f"robots.{rid}.{key}", key, val))

    print()
    if findings:
        for f in findings:
            print(f"  [FAIL] {f}")
        print()
        print(f"  {len(findings)} finding(s) -- a scenario budget that is not connected "
              "end to end is a number the report must not quote.")
        return 1
    print("  every scenario budget is declared by its node AND forwarded by the launch")
    return 0


def _check_value(rel, where: str, key: str, val) -> "list[str]":
    out: list[str] = []
    if isinstance(val, bool):
        return out
    if not isinstance(val, (int, float)):
        return [f"{rel}: {where} = {val!r} is not a number"]
    if not math.isfinite(float(val)):
        return [f"{rel}: {where} = {val!r} is not finite; CONTRACTS section 2 refuses "
                "NaN and Infinity in config"]
    if key in MUST_BE_POSITIVE and float(val) <= 0.0:
        out.append(f"{rel}: {where} = {val} must be > 0")
    if key in MUST_BE_FRACTION and not (0.0 <= float(val) <= 1.0):
        out.append(f"{rel}: {where} = {val} must lie in [0, 1]")
    return out


if __name__ == "__main__":
    sys.exit(main())
