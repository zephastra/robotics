#!/usr/bin/env python3
"""Fail the build on Python mistakes that neither the syntax nor the tests catch.

Three checks, each of which has already cost this project real time.

1. A CAPITALISED name that was never imported or defined. check_fleet_isolation.py was
   rewritten and lost `from rclpy.action import ActionClient` while still calling
   `ActionClient(...)`.

2. Assigning over a member that rclpy.node.Node already provides. `self.clients = {...}`
   looks harmless and raises
       AttributeError: property 'clients' of 'TaskService' object has no setter
   at construction time, i.e. in an unrelated-looking place.

3. A lower-case name LOADED inside a function that is not bound in that function, in any
   enclosing function, at module level, or in builtins. This is the `NameError` family,
   and it is the most expensive one here because it fires at runtime on a path the tests
   never take. Real example:
       nav2_adapter_node._execute() used `map_x`, a local of the sibling method
       `_build_state()`. Every completed leg therefore raised NameError and returned an
       EMPTY failure -- which reads as "the executor died", not as "one name is out of
       scope". It cost two full three-robot runs.

Check 3 needs the module scope computed properly: the old `bound_names()` walked the whole
tree, so a name assigned ANYWHERE (including inside a sibling function) counted as
defined, which would have hidden exactly this bug. `module_scope()` below only descends
into statements that really execute at module level.

Deliberately conservative, so that a false failure is unlikely:
  * only Name loads are checked; attributes (self.x, msg.field) are ignored
  * comprehension and match targets count as bound in the enclosing function
  * nested scopes are searched for closures before reporting
  * `from x import *` makes the file unanalysable, and is reported as skipped, not guessed

Exit 0 when clean, 1 when any check finds something.
"""

from __future__ import annotations

import ast
import builtins
import os
import pathlib
import sys

CAPITALISED_UNDEFINED_OK: set[str] = set()   # names that are legitimately dynamic

#: Module-level dunders Python injects; they are never imported.
MODULE_DUNDERS = {
    "__file__", "__name__", "__doc__", "__package__", "__spec__", "__loader__",
    "__builtins__", "__debug__",
}


def bound_names(tree: ast.AST) -> set[str]:
    """Everything the module makes available: builtins, imports, defs, assignments."""
    out: set[str] = set(dir(builtins)) | set(CAPITALISED_UNDEFINED_OK)
    for node in ast.walk(tree):
        if isinstance(node, (ast.Import, ast.ImportFrom)):
            for a in node.names:
                out.add((a.asname or a.name).split(".")[0])
        elif isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef, ast.ClassDef)):
            out.add(node.name)
        elif isinstance(node, ast.Name) and isinstance(node.ctx, ast.Store):
            out.add(node.id)
        elif isinstance(node, ast.arg):
            out.add(node.arg)
        elif isinstance(node, ast.ExceptHandler) and node.name:
            out.add(node.name)
        elif isinstance(node, ast.comprehension):
            for n in ast.walk(node.target):
                if isinstance(n, ast.Name):
                    out.add(n.id)
        elif isinstance(node, (ast.Global, ast.Nonlocal)):
            out.update(node.names)
    return out


def undefined(path: pathlib.Path) -> list[str]:
    tree = ast.parse(path.read_text(encoding="utf-8"))
    have = bound_names(tree)
    return sorted({n.id for n in ast.walk(tree)
                   if isinstance(n, ast.Name) and isinstance(n.ctx, ast.Load)
                   and n.id[:1].isupper() and n.id not in have})


# --------------------------------------------------------------------------- #
# module scope, computed correctly
# --------------------------------------------------------------------------- #

def _declare(node: ast.AST, out: set[str]) -> bool:
    """Add whatever `node` binds. Returns True if its children must NOT be walked.

    Functions and classes bind ONLY their own name at the outer level: their bodies run
    in their own scope, so descending into them would make every local of every function
    look like a module global -- which is precisely the blindness this rewrite exists to
    remove. (The first version of this guard had that bug, and it could not have caught
    the `map_x` NameError it was written for.)
    """
    if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef, ast.ClassDef)):
        out.add(node.name)
        return True
    if isinstance(node, ast.Lambda):
        return True
    if isinstance(node, (ast.Import, ast.ImportFrom)):
        for a in node.names:
            if a.name == "*":
                out.add("*")
            else:
                out.add((a.asname or a.name).split(".")[0])
        return True
    if isinstance(node, ast.Name) and isinstance(node.ctx, ast.Store):
        out.add(node.id)
    elif isinstance(node, ast.arg):
        out.add(node.arg)
    elif isinstance(node, ast.ExceptHandler) and node.name:
        out.add(node.name)
    elif isinstance(node, (ast.Global, ast.Nonlocal)):
        out.update(node.names)
    elif isinstance(node, ast.comprehension):
        for t in ast.walk(node.target):
            if isinstance(t, ast.Name):
                out.add(t.id)
    elif isinstance(node, ast.MatchAs) and node.name:
        out.add(node.name)
    elif isinstance(node, ast.MatchStar) and node.name:
        out.add(node.name)
    return False


def _binds_in_statement(stmt: ast.stmt) -> set[str]:
    """Names bound by one statement, WITHOUT descending into nested defs/classes."""
    out: set[str] = set()
    stack: list[ast.AST] = [stmt]
    while stack:
        node = stack.pop()
        if _declare(node, out):
            continue
        stack.extend(ast.iter_child_nodes(node))
    return out


def module_scope(tree: ast.AST) -> set[str]:
    """Names visible to module-level code, and to every function as globals.

    Returns the set, plus the sentinel "*" when the file has a star-import, which makes
    static scoping impossible.
    """
    have: set[str] = set(dir(builtins)) | set(CAPITALISED_UNDEFINED_OK) | MODULE_DUNDERS
    for stmt in tree.body:  # type: ignore[attr-defined]
        have |= _binds_in_statement(stmt)
    return have


# --------------------------------------------------------------------------- #
# function scope
# --------------------------------------------------------------------------- #

def _args_of(fn: ast.AST) -> set[str]:
    a = getattr(fn, "args", None)
    if a is None:
        return set()
    out = {x.arg for x in list(a.posonlyargs) + list(a.args) + list(a.kwonlyargs)}
    if a.vararg:
        out.add(a.vararg.arg)
    if a.kwarg:
        out.add(a.kwarg.arg)
    return out


def _locals_of(fn: ast.AST) -> set[str]:
    """Names bound in `fn`'s OWN scope.

    Deliberately does not descend into nested functions/classes: a name assigned only
    inside a nested `def` is not visible in the enclosing function, and pretending
    otherwise is how the previous version of this guard missed the `map_x` bug. Closures
    are handled the other way round, by `enclosing()`.
    """
    out = _args_of(fn)
    if isinstance(fn, ast.Lambda):
        seeds: list[ast.AST] = [fn.body]
    else:
        seeds = list(getattr(fn, "body", []))
    for seed in seeds:
        stack: list[ast.AST] = [seed]
        while stack:
            node = stack.pop()
            if _declare(node, out):
                continue
            stack.extend(ast.iter_child_nodes(node))
    return out


def _loads_in_own_scope(node: ast.AST) -> list[str]:
    """Loaded names in this node's own body, not inside a nested function/lambda."""
    out: list[str] = []

    def visit(cur: ast.AST, top: bool) -> None:
        for child in ast.iter_child_nodes(cur):
            if isinstance(
                child, (ast.FunctionDef, ast.AsyncFunctionDef, ast.ClassDef, ast.Lambda)
            ):
                continue
            if isinstance(child, ast.Name) and isinstance(child.ctx, ast.Load):
                out.append(child.id)
            visit(child, False)

    visit(node, True)
    return out


def _name_of(fn: ast.AST) -> str:
    name = getattr(fn, "name", None)
    return name or "<lambda>"


def scope_problems(path: pathlib.Path) -> tuple[list[str], str | None]:
    """Lower-case names loaded out of scope. Returns (problems, skipped_reason)."""
    tree = ast.parse(path.read_text(encoding="utf-8"))
    have = module_scope(tree)
    if "*" in have:
        return [], "star import makes static scoping impossible"

    parents: dict[int, ast.AST] = {}
    for parent in ast.walk(tree):
        for child in ast.iter_child_nodes(parent):
            parents[id(child)] = parent

    def enclosing(node: ast.AST) -> set[str]:
        out: set[str] = set()
        cur = parents.get(id(node))
        while cur is not None:
            if isinstance(cur, (ast.FunctionDef, ast.AsyncFunctionDef, ast.Lambda)):
                out |= _args_of(cur) | _locals_of(cur)
            cur = parents.get(id(cur))
        return out

    problems: set[str] = set()
    for node in ast.walk(tree):
        if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef, ast.Lambda)):
            allowed = set(have) | enclosing(node) | _args_of(node) | _locals_of(node)
            for name in _loads_in_own_scope(node):
                if name not in allowed:
                    problems.add(f"{_name_of(node)}(): {name}")
    # module-level code (a bare Name at module scope is a real NameError)
    for name in _loads_in_own_scope(tree):
        if name not in have:
            problems.add(f"<module>: {name}")
    return sorted(problems), None


def _members_from_source() -> set[str] | None:
    """Read rclpy's own source for the real member list, without importing it.

    Importing rclpy needs a sourced ROS environment, so in the clean shell where this
    project runs its checks the previous version silently fell back to a hand-written
    list. That made the guard's depth depend on which shell invoked it -- the same class
    of "the check was quietly off" that let `self.clients = {}` reach a live run.

    AST-parsing `node.py` / `node_base.py` off disk gives the same answer in both shells.
    Returns None when the files cannot be found, so the caller can say so out loud
    instead of implying it knows more than it does.
    """
    candidates: list[pathlib.Path] = []
    env_root = os.environ.get("ROS_DISTRO")
    patterns = ["/opt/ros/*/lib/python*/site-packages/rclpy",
                "/opt/ros/*/local/lib/python*/dist-packages/rclpy"]
    if env_root:
        patterns.append(f"/opt/ros/{env_root}/lib/python*/site-packages/rclpy")
    for pattern in patterns:
        candidates += sorted(pathlib.Path("/").glob(pattern.lstrip("/")))
    for entry in sys.path:
        candidate = pathlib.Path(entry) / "rclpy"
        if candidate.is_dir():
            candidates.append(candidate)

    members: set[str] = set()
    found = False
    for pkg in candidates:
        for name in ("node.py", "node_base.py"):
            src = pkg / name
            if not src.is_file():
                continue
            found = True
            try:
                tree = ast.parse(src.read_text(encoding="utf-8", errors="replace"))
            except SyntaxError:
                continue
            for cls in [n for n in ast.walk(tree) if isinstance(n, ast.ClassDef)]:
                for stmt in cls.body:
                    if isinstance(stmt, (ast.FunctionDef, ast.AsyncFunctionDef)):
                        members.add(stmt.name)
                    elif isinstance(stmt, ast.Assign):
                        for t in stmt.targets:
                            if isinstance(t, ast.Name):
                                members.add(t.id)
                    elif isinstance(stmt, ast.AnnAssign) and isinstance(stmt.target, ast.Name):
                        members.add(stmt.target.id)
                    elif isinstance(stmt, ast.Name) and isinstance(stmt.ctx, ast.Store):
                        members.add(stmt.id)
            # properties and `self.x = ...` inside methods are members too
            for node in ast.walk(tree):
                if (isinstance(node, ast.Attribute) and isinstance(node.ctx, ast.Store)
                        and isinstance(node.value, ast.Name) and node.value.id == "self"):
                    members.add(node.attr)
    if not found:
        return None
    return {m for m in members if not m.startswith("__")}


def node_members() -> tuple[set[str], str]:
    """Attribute names that rclpy.node.Node already provides, and how we know.

    Assigning one of these on a Node subclass is always a bug but never a syntax error:
    `self.clients = {...}` raises `AttributeError: property 'clients' ... has no setter`
    at construction time, so the failure surfaces as a crash in an unrelated-looking
    place. That happened here and cost a full P3 bring-up before any check ran.
    """
    fallback = {
        "clients", "publishers", "subscriptions", "services", "timers", "guards",
        "executor", "context", "handle", "parameters", "default_callback_group",
        "get_name", "get_namespace", "get_logger", "get_clock", "get_node_names",
        "create_publisher", "create_subscription", "create_client", "create_service",
        "create_timer", "create_rate", "destroy_node", "declare_parameter",
        "get_parameter", "set_parameters", "count_publishers", "count_subscribers",
        "resolve_topic_name", "resolve_service_name", "get_publishers_info_by_topic",
        "get_service_names_and_types", "get_topic_names_and_types", "destroy_timer",
        "destroy_publisher", "destroy_subscription", "create_guard_condition",
        "get_node_names_and_namespaces", "get_fully_qualified_name",
    }
    try:
        import rclpy.node  # noqa: PLC0415
        return {n for n in dir(rclpy.node.Node) if not n.startswith("__")}, "imported rclpy"
    except Exception:
        pass
    from_source = _members_from_source()
    if from_source:
        return from_source | fallback, "parsed rclpy source (no ROS env needed)"
    return fallback, "CURATED SHORT LIST ONLY -- rclpy source not found, check is partial"


def shadows_node_member(path: pathlib.Path, members: set[str]) -> list[str]:
    tree = ast.parse(path.read_text(encoding="utf-8"))
    hits: set[str] = set()
    for node in ast.walk(tree):
        if (isinstance(node, ast.Attribute) and isinstance(node.ctx, ast.Store)
                and node.attr in members
                and isinstance(node.value, ast.Name) and node.value.id == "self"):
            hits.add(node.attr)
    return sorted(hits)


def main(argv: list[str]) -> int:
    roots = [pathlib.Path(a) for a in (argv or ["scripts", "src"])]
    files: list[pathlib.Path] = []
    for r in roots:
        files += sorted(r.rglob("*.py")) if r.is_dir() else [r]
    members, how = node_members()
    bad = 0
    skipped: list[str] = []
    print(f"  rclpy.node.Node members known: {len(members)} ({how})")
    if "CURATED SHORT LIST" in how:
        print("  [WARN] the member-shadowing check is PARTIAL in this environment")
    for f in files:
        if "__pycache__" in f.parts:
            continue
        try:
            missing = undefined(f)
            shadowed = shadows_node_member(f, members)
            out_of_scope, why = scope_problems(f)
        except SyntaxError as exc:
            print(f"  [FAIL] {f}: not valid Python: {exc}")
            bad += 1
            continue
        if why:
            skipped.append(f"{f} ({why})")
        if missing:
            print(f"  [FAIL] {f}: used but never imported or defined: {missing}")
            bad += 1
        if shadowed:
            print(f"  [FAIL] {f}: assigns over rclpy Node member(s): {shadowed}")
            bad += 1
        if out_of_scope:
            print(f"  [FAIL] {f}: name used outside any scope that binds it: {out_of_scope}")
            bad += 1
    print(f"  files checked: {len(files)}")
    for s in skipped:
        print(f"  [skip] {s}")
    if bad:
        print(f"  RESULT: {bad} problem(s)")
        return 1
    print("  RESULT: no undefined names, no Node member shadowing, no out-of-scope names"
          " (exit 0)")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
