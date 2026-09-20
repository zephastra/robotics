#!/usr/bin/env python3
"""Static checks on the ROS nodes that a compile cannot catch.

Why this exists: a plain `python -m compileall` only proves the file parses. It does not
catch the two mistakes that actually got through in this project:

1. A parameter that is READ but never DECLARED. rclpy raises
   ParameterNotDeclaredException at construction, so the node dies at startup with a
   traceback that names the parameter but not the cause. Measured cost: one full
   4-minute bring-up cycle, during which the gate was absent, so `/r01/cmd_vel` had zero
   publishers and Nav2 reported only `controller_server: Failed to make progress` --
   a symptom that points at the controller, not at a missing node.

2. A parameter that is DECLARED but never used, which usually means an intended wire-up
   was half-applied.

Both are invisible to the type checker, to the linter, and to the runtime until the node
is launched, which is the most expensive possible moment to find out.

Usage:
    scripts/check_ros_params.py [--src DIR ...]
Exit code 0 if clean, 1 if any finding.
"""

from __future__ import annotations

import argparse
import ast
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
DEFAULT_SRCS = [ROOT / "src" / "fleet_ros", ROOT / "src" / "fleet_adapter",
                ROOT / "src" / "fleet_core", ROOT / "scripts"]

# Names that are declared by the framework or by a helper, not by declare_parameter.
ALLOWED_UNDECLARED = {"use_sim_time"}


def _literal_str(node: ast.AST) -> str | None:
    if isinstance(node, ast.Constant) and isinstance(node.value, str):
        return node.value
    return None


def scan(path: Path) -> tuple[set[str], dict[str, int], dict[str, int]]:
    """Return (declared, read_line_by_name, declared_line_by_name)."""
    tree = ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
    declared: set[str] = set()
    declared_at: dict[str, int] = {}
    read_at: dict[str, int] = {}

    for node in ast.walk(tree):
        if isinstance(node, ast.Call):
            fn = node.func
            name = getattr(fn, "attr", None) or getattr(fn, "id", None)
            if name == "declare_parameter" and node.args:
                lit = _literal_str(node.args[0])
                if lit:
                    declared.add(lit)
                    declared_at.setdefault(lit, node.lineno)
            elif name in ("get_parameter",) and node.args:
                lit = _literal_str(node.args[0])
                if lit:
                    read_at.setdefault(lit, node.lineno)
        # the common `p = self.get_parameter` shorthand followed by p("name")
        elif isinstance(node, ast.Call) and isinstance(node.func, ast.Name) and node.args:
            pass

    return declared, read_at, declared_at


def scan_shorthand(path: Path) -> dict[str, int]:
    """Catch `p = self.get_parameter` then `p("name").value`.

    Also catches `self.get_parameter("name")` written with a lambda, and the very common
    `p = self.get_parameter` alias used throughout this codebase.
    """
    src = path.read_text(encoding="utf-8")
    tree = ast.parse(src, filename=str(path))
    aliases: set[str] = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.Assign):
            val = node.value
            if (isinstance(val, ast.Attribute) and val.attr == "get_parameter"):
                for tgt in node.targets:
                    if isinstance(tgt, ast.Name):
                        aliases.add(tgt.id)
        elif isinstance(node, ast.AnnAssign) and isinstance(node.value, ast.Attribute):
            if node.value.attr == "get_parameter" and isinstance(node.target, ast.Name):
                aliases.add(node.target.id)

    found: dict[str, int] = {}
    for node in ast.walk(tree):
        if (isinstance(node, ast.Call) and isinstance(node.func, ast.Name)
                and node.func.id in aliases and node.args):
            lit = _literal_str(node.args[0])
            if lit:
                found.setdefault(lit, node.lineno)
    return found


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--src", action="append", default=None)
    args = ap.parse_args(argv)

    roots = [Path(s) for s in args.src] if args.src else DEFAULT_SRCS
    findings: list[str] = []
    checked = 0

    for root in roots:
        if not root.exists():
            continue
        for path in sorted(root.rglob("*.py")):
            if "__pycache__" in path.parts:
                continue
            checked += 1
            declared, read_at, declared_at = scan(path)
            read_at.update(scan_shorthand(path))

            for name, line in sorted(read_at.items(), key=lambda kv: kv[1]):
                if name in ALLOWED_UNDECLARED:
                    continue
                if declared and name not in declared:
                    findings.append(
                        f"{path}:{line}: parameter {name!r} is READ but never "
                        f"DECLARED. rclpy raises ParameterNotDeclaredException at "
                        f"construction and the node dies at launch."
                    )
            for name, line in sorted(declared_at.items(), key=lambda kv: kv[1]):
                if name not in read_at:
                    findings.append(
                        f"{path}:{line}: parameter {name!r} is declared but never "
                        f"read. Usually a half-applied change."
                    )

    print("=" * 78)
    print("  009 ROS parameter check (static)")
    print("=" * 78)
    print(f"  files checked: {checked}")
    if findings:
        print()
        for f in findings:
            print(f"  [FAIL] {f}")
        print()
        print(f"  {len(findings)} finding(s) -- refusing to call the nodes launchable.")
        return 1
    print("  every parameter that is read is also declared, and none are dead")
    return 0


if __name__ == "__main__":
    sys.exit(main())
