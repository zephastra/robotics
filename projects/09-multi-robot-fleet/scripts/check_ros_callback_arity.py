#!/usr/bin/env python3
"""Fail the build when a ROS callback is registered with the wrong number of parameters.

WHY THIS EXISTS
---------------
2026-09-18, round 17. A driver was given a navigation-feedback instrument:

    send_goal_async(goal, feedback_callback=self._on_feedback)
    def _on_feedback(self, _handle, feedback) -> None: ...

rclpy's ActionClient feeds that callback as

    await await_or_execute(self._feedback_callbacks[goal_uuid], feedback_msg)

i.e. with ONE argument. The definition wanted two. Every navigation attempt therefore died
on the first feedback message:

    TypeError: StagedCrossing._on_feedback() missing 1 required positional argument: 'feedback'

and the driver's report -- `complete: false, stopped_by: exception, stages: []` -- reads
exactly like a robot that refused to move. Four cases were run and discarded before anyone
opened a driver report, and the case whose safety verdict came back PASS had passed it
VACUOUSLY: a robot that never moves never enters a corridor.

NOTHING EXISTING COULD CATCH IT
    * the syntax is fine, so Python compiles it;
    * the name is defined, so check_undefined_names.py is satisfied -- that guard checks
      NAMES, and this is an ARITY;
    * no test calls it, because only the ROS executor ever does;
    * the correct shape was ALREADY in this repository at scripts/nav_goal.py:65.

So: for every callback registration in src/ and scripts/, resolve the target to its
definition and check that it can be called with the number of arguments ROS actually
passes.

Structural claims are checked structurally. Never by substring: the word `feedback` occurs
in the definition, in the registration and in the comment, so a substring test for it would
have passed on the broken code -- which is how this survived a full suite and seven guards.

Exit 0 when every resolved registration matches, 1 when any does not, 3 on an environment
problem. Registrations whose target cannot be resolved statically are reported as SKIPPED
rather than assumed good, so the output says what this guard actually knew.
"""

from __future__ import annotations

import ast
import pathlib
import sys

#: registration -> (keyword carrying the callback, positional index of the callback,
#: arguments ROS passes). The positions come from the rclpy signatures, not from memory:
#:   send_goal_async(goal, feedback_callback=None, goal_uuid=None)
#:   create_subscription(msg_type, topic, callback, qos_profile, ...)
#:   create_timer(timer_period_sec, callback, callback_group=None, ...)
#:   create_service(srv_type, srv_name, callback, ...)
REGISTRATIONS: dict[str, tuple[str, int, int]] = {
    "send_goal_async": ("feedback_callback", 1, 1),
    "create_subscription": ("callback", 2, 1),
    "create_timer": ("callback", 1, 0),
    "create_wall_timer": ("callback", 1, 0),
    "create_service": ("callback", 2, 2),
}

SCAN_DIRS = ("src", "scripts")
SKIP_DIRS = {"__pycache__", "build", "install", "log", "runtime"}


class Definition:
    """One function or method found in the tree, with what it can be called with."""

    __slots__ = ("file", "line", "qualname", "names", "defaults", "vararg", "required_kwonly")

    def __init__(self, file: str, line: int, qualname: str, names: list[str], defaults: int,
                 vararg: bool, required_kwonly: list[str]) -> None:
        self.file = file
        self.line = line
        self.qualname = qualname
        self.names = names
        self.defaults = defaults
        self.vararg = vararg
        self.required_kwonly = required_kwonly


def _describe(func: ast.FunctionDef | ast.AsyncFunctionDef | ast.Lambda, *,
              drop_first: bool) -> tuple[list[str], int, bool, list[str]]:
    args = func.args
    names = [a.arg for a in list(args.posonlyargs) + list(args.args)]
    defaults = len(args.defaults)
    if drop_first and names:
        names = names[1:]           # `self` is bound already; it is not an argument ROS passes
    star = args.vararg is not None
    required_kwonly = [a.arg for a, default in zip(args.kwonlyargs, args.kw_defaults)
                       if default is None]
    return names, defaults, star, required_kwonly


def _accepts(definition: Definition, count: int) -> bool:
    """Can `definition` be called with exactly `count` positional arguments?"""
    if definition.required_kwonly:
        return False                            # nothing would supply them
    required = len(definition.names) - definition.defaults
    if count < required:
        return False
    if definition.vararg:
        return True
    return count <= len(definition.names)


def _collect(tree: ast.Module, file: str) -> dict[str, Definition]:
    """Every function, method and lambda-with-a-name-position, keyed by qualified name."""
    found: dict[str, Definition] = {}
    for node in tree.body:
        if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef)):
            names, defaults, star, kwonly = _describe(node, drop_first=False)
            found[node.name] = Definition(file, node.lineno, node.name, names, defaults,
                                          star, kwonly)
        elif isinstance(node, ast.ClassDef):
            for item in node.body:
                if isinstance(item, (ast.FunctionDef, ast.AsyncFunctionDef)):
                    names, defaults, star, kwonly = _describe(item, drop_first=True)
                    found[f"{node.name}.{item.name}"] = Definition(
                        file, item.lineno, f"{node.name}.{item.name}",
                        names, defaults, star, kwonly)
    return found


def _enclosing_class(tree: ast.Module) -> dict[int, str]:
    """Map each node's id to the innermost class it sits in."""
    owner: dict[int, str] = {}

    def walk(node: ast.AST, current: str | None) -> None:
        for child in ast.iter_child_nodes(node):
            if isinstance(child, ast.ClassDef):
                owner[id(child)] = current or ""
                walk(child, child.name)
            else:
                owner[id(child)] = current or ""
                walk(child, current)

    walk(tree, None)
    return owner


def _resolve(call: ast.Call, spec: tuple[str, int, int], owner: dict[int, str],
             found: dict[str, Definition]) -> tuple[Definition | None, str]:
    """Find the callback expression and the definition it names. Second value is a reason."""
    keyword, position, _ = spec
    target: ast.expr | None = None
    for kw in call.keywords:
        if kw.arg == keyword:
            target = kw.value
            break
    if target is None and len(call.args) > position:
        target = call.args[position]
    if target is None:
        return None, "no callback argument at all"

    if isinstance(target, ast.Lambda):
        names, defaults, star, kwonly = _describe(target, drop_first=False)
        return (Definition("<lambda>", target.lineno, "<lambda>", names, defaults, star, kwonly),
                "")

    if isinstance(target, ast.Name):
        if target.id in found:
            return found[target.id], ""
        return None, f"`{target.id}` is not defined in this file (imported or built elsewhere)"

    if isinstance(target, ast.Attribute) and isinstance(target.value, ast.Name):
        cls = owner.get(id(target))
        if target.value.id == "self" and cls:
            key = f"{cls}.{target.attr}"
            if key in found:
                return found[key], ""
            return None, f"`{key}` is not defined in this file"
        return None, f"`{target.value.id}.{target.attr}` is not a local definition"

    if isinstance(target, ast.Call):
        return None, "the callback is the result of a call (partial/wrapper), not a definition"

    return None, f"cannot resolve `{ast.unparse(target)[:40]}`"


def _files() -> list[pathlib.Path]:
    out: list[pathlib.Path] = []
    for base in SCAN_DIRS:
        root = pathlib.Path(base)
        if not root.is_dir():
            continue
        for path in sorted(root.rglob("*.py")):
            if any(part in SKIP_DIRS for part in path.parts):
                continue
            out.append(path)
    return out


def main() -> int:
    if not pathlib.Path("src").is_dir():
        print("check_ros_callback_arity.py: run me from the repository root", file=sys.stderr)
        return 3

    failures: list[str] = []
    skips: list[str] = []
    checked = 0

    for path in _files():
        try:
            text = path.read_text(encoding="utf-8")
        except (OSError, UnicodeDecodeError) as exc:
            print(f"cannot read {path}: {exc}", file=sys.stderr)
            return 3
        if "import *" in text:
            skips.append(f"{path}: `from ... import *`; the file cannot be analysed")
            continue
        tree = ast.parse(text, filename=str(path))
        found = _collect(tree, str(path))
        owner = _enclosing_class(tree)

        for node in ast.walk(tree):
            if not isinstance(node, ast.Call):
                continue
            func = node.func
            if not isinstance(func, ast.Attribute) or func.attr not in REGISTRATIONS:
                continue
            spec = REGISTRATIONS[func.attr]
            _, _, expected = spec
            definition, why = _resolve(node, spec, owner, found)
            where = f"{path}:{node.lineno}"
            if definition is None:
                skips.append(f"{where} {func.attr}: {why}")
                continue
            checked += 1
            if _accepts(definition, expected):
                continue
            names = ", ".join(definition.names) or "(nothing)"
            failures.append(
                f"{where} {func.attr}({spec[0]}=...)\n"
                f"      ROS passes {expected} argument(s); {definition.qualname} "
                f"(declared at {definition.file}:{definition.line}) takes [{names}]"
                + (", all optional" if definition.defaults else "")
                + (f", and requires keyword-only [{', '.join(definition.required_kwonly)}]"
                   if definition.required_kwonly else "")
            )

    for line in skips:
        print(f"  [skip] {line}")
    for line in failures:
        print(f"  [FAIL] {line}")

    print(f"  {checked} registration(s) resolved and checked, {len(skips)} skipped")
    if failures:
        print(f"  FAILED: {len(failures)} registration(s) would raise TypeError when ROS calls "
              "them, on a path no unit test takes.")
        return 1
    print("  every resolved callback can be called with the arguments ROS passes")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
