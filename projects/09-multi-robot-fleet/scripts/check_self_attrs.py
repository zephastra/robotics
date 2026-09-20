#!/usr/bin/env python3
"""Guard: a `self.<name>` read in a plain class must be assigned by some class in its file.

WHY THIS EXISTS
---------------
`staged_crossing.py:361` (then `:348`) read `self.movement_stages`. Nothing anywhere in
the repository assigned it. The read sat inside an `and` whose LEFT operand was
`self.expected_passage_s > 0.0`, so Python's short-circuit evaluation meant the
attribute was never touched on any recorded run -- and all 143 recorded crossing
reports declare `expected_passage_s` 0.0.

The bug was therefore invisible to every instrument this project has:

  * the test suite -- the tests never set `expected_passage_s` either, so they never
    entered the branch;
  * a live run -- the branch was never entered, so the run looked clean;
  * `py_compile` and every linter -- `self.x` is a legal attribute access;
  * code review -- it reads exactly like a field assigned in `__init__`.

Same shape as `D-P17-05` (`_on_feedback` taking two arguments when rclpy passes one):
a name that EXISTS, spelled correctly, in the right place, with nothing behind it.
Only EVALUATING the line reveals it, and nothing evaluated it. Same shape again as
`D-P17-15`, where the liveness check reported `live: 11 FROZEN: 0` while five of those
bodies were pinned, because it measured a quantity adjacent to the one it named.

The AST is authoritative here, not a text search. Grepping for `self.movement_stages`
finds the read and says nothing about whether an assignment exists elsewhere in the
file, in a sibling class, or in a method.

WHAT IS DELIBERATELY NOT FLAGGED, AND WHY
----------------------------------------
A guard that reports 40 lines on a clean tree is a guard that gets switched off, and
then it protects nothing. Three real sources of attributes are recognised:

  * **dataclass fields.** `@dataclass` turns annotations into `__init__` arguments and
    assigns them for you, so `RunResult.commanded_mps` is genuinely fine even though
    no line in the file says `self.commanded_mps = ...`;
  * **base classes outside this file.** `http.server.BaseHTTPRequestHandler` provides
    `self.wfile`, `self.path`, `self.send_response`; a class that inherits from an
    unknown name is skipped rather than guessed at;
  * **class-level constants.** `self.ACCEL`, where `ACCEL` is a plain assignment in
    the class body, resolves through the class namespace.

Inside a **dataclass**, unannotated `self.<name> = ...` assignments count as declared
attributes -- `__post_init__` sets computed fields that way, and listing them in the
annotation block is not required.

Exit codes: 0 clean, 1 problems found, 2 self-test failure.
"""
from __future__ import annotations

import ast
import pathlib
import sys

ROOT = pathlib.Path(__file__).resolve().parents[1]
SRC = ROOT / "src"

#: `rclpy.node.Node` members a node class may use without assigning them. Verified
#: against the installed Lyrical `rclpy.node.Node`. Keep this list SHORT: every entry
#: is a permanent blind spot, and one line of `self.foo = ...` is cheaper than that.
NODE_ALLOW = {
    "create_client", "create_service", "create_publisher", "create_subscription",
    "create_timer", "create_rate", "create_guard_condition",
    "get_logger", "get_clock", "get_name", "get_namespace", "get_parameters",
    "get_parameter", "get_parameter_or", "declare_parameter", "declare_parameters",
    "set_parameters", "set_parameters_atomically", "has_parameter",
    "destroy_node", "destroy_timer", "destroy_subscription", "destroy_client",
    "destroy_publisher", "destroy_service", "destroy_rate",
    "count_publishers", "count_subscribers", "get_publishers_info_by_topic",
    "get_subscriptions_info_by_topic", "resolve_topic_name", "resolve_service_name",
    "clock", "executor", "context", "handle", "default_callback_group",
    "get_node_names", "get_topic_names_and_types", "get_service_names_and_types",
}

#: Base classes whose members this guard cannot see. A class inheriting from one of
#: these is SKIPPED ENTIRELY rather than analysed with a hole in it: a guard that
#: guesses produces false positives, and false positives are how it gets turned off.
#: Every entry is a stdlib class whose provided attributes are documented.
UNKNOWN_BASE_ALLOW = {
    "BaseHTTPRequestHandler", "HTTPServer", "ThreadingHTTPServer", "TCPServer",
    "StreamRequestHandler", "Exception", "ValueError", "RuntimeError", "KeyError",
    "TypeError", "OSError", "TimeoutError", "BaseException", "Enum", "IntEnum",
    "Thread", "Process", "dict", "list", "str", "BaseModel", "NamedTuple",
    "SimpleHTTPRequestHandler", "ABC", "Protocol",
}


def _decorator_names(cls: ast.ClassDef) -> set[str]:
    out: set[str] = set()
    for d in cls.decorator_list:
        if isinstance(d, ast.Name):
            out.add(d.id)
        elif isinstance(d, ast.Attribute):
            out.add(d.attr)
        elif isinstance(d, ast.Call):
            f = d.func
            if isinstance(f, ast.Name):
                out.add(f.id)
            elif isinstance(f, ast.Attribute):
                out.add(f.attr)
    return out


def _base_names(cls: ast.ClassDef) -> set[str]:
    out: set[str] = set()
    for b in cls.bases:
        if isinstance(b, ast.Name):
            out.add(b.id)
        elif isinstance(b, ast.Attribute):
            out.add(b.attr)
    return out


def _annotated_names(cls: ast.ClassDef) -> set[str]:
    """Names declared as class-level or bare annotations (i.e. dataclass fields)."""
    out: set[str] = set()
    for node in ast.walk(cls):
        if isinstance(node, ast.AnnAssign) and isinstance(node.target, ast.Name):
            out.add(node.target.id)
    return out


def _class_constants(cls: ast.ClassDef) -> set[str]:
    """Names bound directly in the class body -- `ACCEL = 1.0` and friends."""
    out: set[str] = set()
    for st in cls.body:
        if isinstance(st, ast.Assign):
            for t in st.targets:
                if isinstance(t, ast.Name):
                    out.add(t.id)
        elif isinstance(st, ast.AnnAssign) and isinstance(st.target, ast.Name):
            out.add(st.target.id)
    return out


def _class_attrs(cls: ast.ClassDef) -> tuple[set[str], dict[str, int], set[str]]:
    """(assigned on self, read on self -> first lineno, method names)."""
    assigned: set[str] = set()
    read: dict[str, int] = {}
    for n in ast.walk(cls):
        if isinstance(n, ast.Attribute) and isinstance(n.value, ast.Name) \
                and n.value.id == "self":
            if isinstance(n.ctx, ast.Store):
                assigned.add(n.attr)
            elif isinstance(n.ctx, ast.Load):
                read.setdefault(n.attr, n.lineno)
    methods = {b.name for b in cls.body
               if isinstance(b, (ast.FunctionDef, ast.AsyncFunctionDef))}
    return assigned, read, methods


def scan_source(text: str, label: str) -> list[str]:
    try:
        tree = ast.parse(text)
    except SyntaxError:
        return []
    classes = [n for n in ast.walk(tree) if isinstance(n, ast.ClassDef)]
    if not classes:
        return []

    # Attributes assigned by ANY class in the file still count: a mixin or helper node
    # in the same module is a legitimate source, and denying that manufactures false
    # positives.
    file_assigned: set[str] = set()
    for c in classes:
        a, _, _ = _class_attrs(c)
        file_assigned |= a

    problems: list[str] = []
    for c in classes:
        bases = _base_names(c)
        if bases & UNKNOWN_BASE_ALLOW:
            continue
        decos = _decorator_names(c)
        assigned, read, methods = _class_attrs(c)
        known = (file_assigned | methods | _annotated_names(c)
                 | _class_constants(c) | NODE_ALLOW)
        # Inside a dataclass, `self.x = ...` in __post_init__ counts as a declared
        # attribute even without an annotation: dataclass does not require both.
        if "dataclass" in decos:
            known |= assigned
        for name, lineno in sorted(read.items(), key=lambda kv: kv[1]):
            if name in known:
                continue
            if name.startswith("__") and name.endswith("__"):
                continue
            problems.append(
                f"{label}:{lineno}: {c.name} reads self.{name}, which nothing in this "
                f"file assigns, annotates, or defines. Assign it (usually in __init__), "
                f"annotate it, or add the base class to UNKNOWN_BASE_ALLOW with the "
                f"reason."
            )
    return problems


def scan_file(path: pathlib.Path, root: pathlib.Path | None = None) -> list[str]:
    base = ROOT if root is None else root
    try:
        label = str(path.relative_to(base))
    except ValueError:
        label = str(path)
    return scan_source(path.read_text(encoding="utf-8"), label)


def main(argv: list[str] | None = None) -> int:
    argv = list(sys.argv[1:] if argv is None else argv)
    if argv and argv[0] == "--self-test":
        return self_test()

    targets = [p for p in sorted(SRC.rglob("*.py")) if "__pycache__" not in p.parts]
    problems: list[str] = []
    for p in targets:
        problems.extend(scan_file(p))

    if problems:
        print(f"FAIL: {len(problems)} attribute(s) read but never assigned")
        for line in problems:
            print("  " + line)
        print(
            "\nThis is the D-P17-18 shape: the name is spelled correctly, the access is "
            "legal, and only EVALUATING the line reveals that nothing is behind it. A "
            "short-circuited `and` or an unexercised branch can hide it indefinitely."
        )
        return 1
    print(f"OK: {len(targets)} source file(s) checked; every self.<name> read is assigned")
    return 0


def self_test() -> int:
    """Prove the guard CAN fail, before trusting its pass.

    A check that has only ever returned 0 has not been shown to work -- it has been
    shown to be quiet. `D-P17-15` records this lesson for the truth-liveness check,
    which reported `live: 11 FROZEN: 0` while five of those bodies were pinned. So the
    cases below include the exact shape that slipped through here, plus the three
    legitimate sources that must NOT be flagged -- else the guard would be unusable and
    consequently ignored.
    """
    cases = [
        ("read but never assigned -> must FAIL", """
class N:
    def __init__(self):
        self.good = 1
    def go(self):
        if self.good:
            return self.missing
""", 1),
        ("short-circuit hid it, the exact D-P17-18 shape -> must FAIL", """
class N:
    def __init__(self, expected):
        self.expected = expected
    def go(self):
        if self.expected > 0.0 and self.movement_stages > 0:
            return 1
        return 0
""", 1),
        ("assigned in __init__ -> must PASS", """
class N:
    def __init__(self):
        self.good = 1
    def go(self):
        return self.good
""", 0),
        ("method call -> must PASS", """
class N:
    def go(self):
        return self.helper()
    def helper(self):
        return 1
""", 0),
        ("rclpy Node member -> must PASS", """
class N:
    def go(self):
        return self.get_logger()
""", 0),
        ("dataclass field, never assigned in the file -> must PASS", """
from dataclasses import dataclass
@dataclass
class R:
    commanded_mps: float = 0.0
    def show(self):
        return self.commanded_mps
""", 0),
        ("class-level constant -> must PASS", """
class N:
    ACCEL = 1.5
    def go(self):
        return self.ACCEL
""", 0),
        ("unknown stdlib base -> must be SKIPPED, not flagged", """
import http.server
class H(http.server.BaseHTTPRequestHandler):
    def do_GET(self):
        return self.wfile
""", 0),
        ("assigned by a sibling class (mixin) -> must PASS", """
class Mixin:
    def setup(self):
        self.shared = 1
class N(Mixin):
    def go(self):
        return self.shared
""", 0),
        ("dataclass __post_init__ computed field -> must PASS", """
from dataclasses import dataclass
@dataclass
class R:
    x: float = 0.0
    def __post_init__(self):
        self.doubled = self.x * 2
    def show(self):
        return self.doubled
""", 0),
        ("plain class, __post_init__ but no @dataclass -> must FAIL", """
class R:
    def __post_init__(self):
        self.doubled = 1
    def show(self):
        return self.doubled, self.imagined
""", 1),
    ]
    failures = 0
    for label, body, want in cases:
        got = len(scan_source(body, "probe.py"))
        ok = got == want
        print(f"  {'OK  ' if ok else 'FAIL'} {label} (found {got}, want {want})")
        failures += 0 if ok else 1
    if failures:
        print(f"FAIL: guard self-test, {failures} of {len(cases)} case(s) wrong")
        return 2
    print(f"OK: guard self-test, {len(cases)} cases")
    return 0


if __name__ == "__main__":
    sys.exit(main())
