#!/usr/bin/env python3
"""Check that the frozen case list matches the contract, and that blocked cases are explained.

WHY THIS EXISTS
---------------
TEST_AND_ACCEPTANCE section 5 freezes 30 cases by id and section 5's strategy adds two extra
seeds for four of them. Nothing enforced either. A batch list is exactly the kind of artefact
that drifts quietly: a case gets renamed, one gets dropped because it was awkward, a case is
marked "not run" and the reason never written down. So this guard reads section 5's **own
table** out of the document and requires `config/scenarios/regression_v1.yaml` to cover
precisely those ids -- the manifest cannot add, drop or rename a case, and a case that cannot
run must name a prerequisite that is registered in `docs/LIMITATIONS.md`.

  * id set in the document  !=  id set in the manifest          -> FAIL
  * a case with neither (`submit` + `expect`) nor `requires`     -> FAIL
  * a case with both                                             -> FAIL
  * a `requires` key that is not declared under `prerequisites`  -> FAIL
  * a `requires` key absent from `docs/LIMITATIONS.md`           -> FAIL
  * a `scenario:` that is not a file under `config/scenarios/`    -> FAIL
  * a station, charger or capability a case names that the fleet
    config does not define                                      -> FAIL
  * `expect.safety: PASS` without a `truth:` source               -> FAIL, because a safety
    verdict needs a recording and the manifest has to say which one it expects
  * a `truth:` source the runner does not implement                -> FAIL, checked against
    `TRUTH_SOURCES` in `scripts/batch.py`, parsed rather than copied: a case may not name a
    collector that does not exist
  * `seed_repeats` that does not produce eight extra runs         -> FAIL
  * a case declared `runnable` whose scenario file fails the
    scenario-budget rules                                        -> FAIL (checked by
    `check_scenario_budgets.py`; this guard requires the file to exist and parse)
  * a `wait_for` key the runner does not implement                 -> FAIL, checked against
    `KNOWN_KEYS` in `fleet_core/wait_for.py`, parsed rather than copied: an unknown key would
    hold for ever, so the case would run with no order at all and then run out of budget
  * a `wait_for.state` with no `request_id` to attach it to         -> FAIL, for the same reason
  * a `crossing_order` naming a robot the case does not run         -> FAIL; that order can
    never be observed, so the case would fail after a full run instead of in the build
  * a `steps` entry with an action, a required field, a robot or a
    node the runner does not implement                              -> FAIL, checked against
    `fleet_core/trigger.py` and against the node names the launch files actually start. A node
    name no launch uses would match no process, and the runner reports that as "the fault was
    not delivered" -- a manifest error wearing a fleet finding's clothes
  * a `steps.when` key the runner does not implement                 -> FAIL, the same check as
    `wait_for`, because it is the same vocabulary
  * a `behavior_checks` name the runner does not implement            -> FAIL, checked against
    `DECLARED_CHECKS` in `scripts/batch.py`; an ignored check makes a case look stricter than
    it is

Usage:
    scripts/check_batch_manifest.py [--manifest PATH] [--root DIR]
Exit 0 clean, 2 the manifest is unusable, 1 on any finding.
"""

from __future__ import annotations

import argparse
import ast
import re
import sys
from collections.abc import Mapping
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[1]
DOC = Path("docs") / "TEST_AND_ACCEPTANCE.md"
DEFAULT_MANIFEST = Path("config") / "scenarios" / "regression_v1.yaml"

#: Section 1's vocabulary. A case that expects something else is not declaring an outcome.
TASK_OUTCOMES = frozenset({
    "SUCCEEDED", "FAILED", "CANCELED", "NEEDS_ATTENTION", "NOT_STARTED",
})
SAFETY_OUTCOMES = frozenset({"PASS", "FAIL", "UNKNOWN"})
BEHAVIOUR_OUTCOMES = frozenset({"PASS", "FAIL", "NOT_RUN"})


def runner_wait_keys(root: Path) -> "tuple[bool, tuple[str, ...]]":
    """The `wait_for` keys `fleet_core.wait_for` implements, parsed from it.

    Parsed rather than copied for the reason the truth-source check is parsed: a copied list
    drifts, and the drift is silent -- the guard would refuse a key the runner supports, or
    accept one it does not, and either way the manifest and the code would disagree while both
    looked correct. `found` is separate from an empty tuple so that a renamed constant reads as
    "I cannot check this" rather than as "the vocabulary is empty".

    Both `NAME = ...` and the annotated `NAME: "..." = ...` are handled. Round 15 shipped a
    reader that understood only the first form and reported "not found" for a constant sitting
    in the file with an annotation on it.
    """
    path = root / "src" / "fleet_core" / "fleet_core" / "wait_for.py"
    if not path.is_file():
        return False, ()
    tree = ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
    for node in ast.walk(tree):
        if isinstance(node, ast.Assign):
            names = [t.id for t in node.targets if isinstance(t, ast.Name)]
        elif isinstance(node, ast.AnnAssign) and isinstance(node.target, ast.Name):
            names = [node.target.id]
        else:
            continue
        if names != ["KNOWN_KEYS"] or not isinstance(node.value, ast.Call):
            continue
        for arg in node.value.args:
            if isinstance(arg, ast.Set):
                values = [e.value for e in arg.elts if isinstance(e, ast.Constant)]
                return True, tuple(sorted(str(v) for v in values))
    return False, ()

_ID = re.compile(r"^\|\s*([NFI]\d{2})\s*\|")


def _assigned(node: "ast.AST", name: str) -> "ast.AST | None":
    """The value assigned to a module-level `name`, in either assignment form.

    Both `NAME = ...` and the annotated `NAME: "..." = ...` are handled; round 15 shipped a
    reader that understood only the first form and reported "not found" for a constant sitting
    in the file with an annotation on it.
    """
    if isinstance(node, ast.Assign):
        names = [t.id for t in node.targets if isinstance(t, ast.Name)]
    elif isinstance(node, ast.AnnAssign) and isinstance(node.target, ast.Name):
        names = [node.target.id]
    else:
        return None
    return node.value if names == [name] else None


def _string_set(value: "ast.AST | None") -> "tuple[str, ...] | None":
    """`frozenset({...})` / `frozenset([...])` / a tuple literal -> its string members."""
    if isinstance(value, ast.Call):
        value = value.args[0] if value.args else None
    if isinstance(value, (ast.Set, ast.Tuple, ast.List)):
        return tuple(sorted(str(e.value) for e in value.elts
                            if isinstance(e, ast.Constant)))
    return None


def runner_actions(root: Path) -> "tuple[bool, tuple[str, ...], dict, str]":
    """(found, action names, action -> required fields, the "all" target), parsed from trigger.py."""
    path = root / "src" / "fleet_core" / "fleet_core" / "trigger.py"
    if not path.is_file():
        return False, (), {}, ""
    tree = ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
    actions: "tuple[str, ...] | None" = None
    fields: dict = {}
    target_all = ""
    for node in ast.walk(tree):
        value = _assigned(node, "KNOWN_ACTIONS")
        if value is not None:
            actions = _string_set(value) or actions
        value = _assigned(node, "TARGET_ALL")
        if isinstance(value, ast.Constant) and isinstance(value.value, str):
            target_all = value.value
        value = _assigned(node, "REQUIRED_FIELDS")
        if isinstance(value, ast.Dict):
            for key, item in zip(value.keys, value.values):
                if not (isinstance(key, ast.Constant) and isinstance(item, ast.Tuple)):
                    continue
                fields[str(key.value)] = tuple(str(e.value) for e in item.elts
                                               if isinstance(e, ast.Constant))
    if actions is None:
        return False, (), fields, target_all
    return True, actions, fields, target_all


def runner_declared_checks(root: Path) -> "tuple[bool, tuple[str, ...]]":
    """The `behavior_checks` names `scripts/batch.py` implements, parsed from it."""
    path = root / "scripts" / "batch.py"
    if not path.is_file():
        return False, ()
    tree = ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
    for node in ast.walk(tree):
        value = _assigned(node, "DECLARED_CHECKS")
        if value is None:
            continue
        names = _string_set(value)
        if names is not None:
            return True, names
    return False, ()


def runner_implemented_actions(root: Path) -> "tuple[bool, tuple[str, ...]]":
    """The step actions `scripts/batch.py` ACTUALLY dispatches.

    Parsed from the `if action == "..."` chain inside `fire_step`, because that chain IS the
    implementation. `KNOWN_ACTIONS` in `fleet_core/trigger.py` is the vocabulary the CONTRACT
    declares, and the two are not the same thing.

    They were treated as the same thing until 2026-09-19, and the difference was worth ten
    cases. The vocabulary named eighteen actions and `fire_step` dispatched four, so this
    file's own message -- "An action nobody runs would be reported as a fleet that survived
    its fault" -- was printed by a comparison that could never fire. Measured against the
    frozen list: F01, F02, F03, F10, I03, I04, I05, I06, I07 and I08 each named an action
    with nothing behind it, so none of their faults was ever delivered, and each of them
    would have been reported on as though it had been.

    The same idea was already applied correctly to `behavior_checks` (see
    `runner_declared_checks`, which reads `DECLARED_CHECKS` out of `batch.py`); it was simply
    missed for step actions.
    """
    path = root / "scripts" / "batch.py"
    if not path.is_file():
        return False, ()
    tree = ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
    fire_step = None
    for node in ast.walk(tree):
        if isinstance(node, ast.FunctionDef) and node.name == "fire_step":
            fire_step = node
    if fire_step is None:
        return False, ()
    found: "set[str]" = set()
    for node in ast.walk(fire_step):
        if not isinstance(node, ast.Compare):
            continue
        if not (isinstance(node.left, ast.Name) and node.left.id == "action"):
            continue
        for comparator in node.comparators:
            if isinstance(comparator, ast.Constant) and isinstance(comparator.value, str):
                found.add(comparator.value)
    return True, tuple(sorted(found))


def launch_node_names(root: Path) -> "tuple[bool, tuple[str, ...]]":
    """Every constant node name a launch file can start, parsed from the launch files.

    Names built from an f-string (`f"spawn_{robot}"`) are skipped rather than guessed. They are
    not addressable the way `kill_process` addresses a node anyway -- the spawn action runs once
    and exits -- and inventing a name here would be worse than leaving it out.
    """
    names: "set[str]" = set()
    found = False
    for rel in ("src/fleet_bringup/launch/robot.launch.py",
                "src/fleet_bringup/launch/fleet.launch.py"):
        path = root / rel
        if not path.is_file():
            continue
        found = True
        tree = ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
        for node in ast.walk(tree):
            if not (isinstance(node, ast.Call)
                    and isinstance(node.func, ast.Name) and node.func.id == "Node"):
                continue
            for kw in node.keywords:
                if (kw.arg == "name" and isinstance(kw.value, ast.Constant)
                        and isinstance(kw.value.value, str)):
                    names.add(kw.value.value)
    return found, tuple(sorted(names))


def runner_truth_sources(root: Path) -> "tuple[bool, tuple[str, ...]]":
    """The truth sources `scripts/batch.py` implements, parsed from it.

    Returns (found, names). `found` is separate from an empty tuple on purpose: a runner whose
    constant has been renamed is a different problem from a runner that attaches nothing, and
    the two need different messages.
    """
    path = root / "scripts" / "batch.py"
    if not path.is_file():
        return (False, ())
    tree = ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
    for node in ast.walk(tree):
        targets: list[str] = []
        if isinstance(node, ast.Assign):
            targets = [t.id for t in node.targets if isinstance(t, ast.Name)]
        elif isinstance(node, ast.AnnAssign) and isinstance(node.target, ast.Name):
            targets = [node.target.id]
        if "TRUTH_SOURCES" not in targets or not isinstance(node.value, ast.Tuple):
            continue
        names = tuple(e.value for e in node.value.elts
                      if isinstance(e, ast.Constant) and isinstance(e.value, str))
        return (True, names)
    return (False, ())


def contract_ids(doc: Path) -> "list[str]":
    """The ids of section 5's table, in order, read from the document itself."""
    text = doc.read_text(encoding="utf-8")
    start = text.index("## 5.")
    end = text.index("### 批测策略", start)
    return [m.group(1) for line in text[start:end].splitlines()
            if (m := _ID.match(line))]


def _fail(findings: "list[str]", msg: str) -> None:
    findings.append(msg)


def load_manifest(path: Path, root: Path) -> dict:
    """Parse and validate. Returns the manifest, or raises SystemExit with the findings.

    `batch.py` calls this, so the runner and the guard cannot disagree about what a case
    means: there is one reader.
    """
    doc = root / DOC
    if not doc.is_file():
        raise SystemExit(f"contract not found: {doc}")
    manifest_file = path if path.is_absolute() else root / path
    if not manifest_file.is_file():
        raise SystemExit(f"manifest not found: {manifest_file}")

    manifest = yaml.safe_load(manifest_file.read_text(encoding="utf-8")) or {}
    findings: list[str] = []

    if str(manifest.get("schema_version", "")) != "1":
        _fail(findings, f"{DOC.name}: manifest schema_version must be the string \"1\"")

    cases = manifest.get("cases") or {}
    if not isinstance(cases, dict) or not cases:
        raise SystemExit("manifest declares no cases")

    # ---- the id set, checked against the document ------------------------- #
    contract = contract_ids(doc)
    if not contract:
        raise SystemExit("could not read section 5's case table out of the contract")
    missing = [c for c in contract if c not in cases]
    extra = [c for c in cases if c not in contract]
    if missing:
        _fail(findings, f"the contract freezes {len(contract)} cases and the manifest is "
                        f"missing {missing}")
    if extra:
        _fail(findings, f"the manifest declares case(s) the contract does not: {extra}")

    # ---- the wait vocabulary ---------------------------------------------- #
    # Read once, before the case loop, so a manifest can be refused in the build rather than
    # after a fifteen-minute run that waited for a condition which could never fire.
    wait_found, wait_keys = runner_wait_keys(root)
    if not wait_found:
        _fail(findings, "could not read KNOWN_KEYS from "
                        "src/fleet_core/fleet_core/wait_for.py, so no `wait_for` condition can "
                        "be checked. A guard that cannot read its own vocabulary must refuse, "
                        "not pass.")
        print("  [FAIL] the `wait_for` vocabulary could not be read")
    else:
        print(f"  `wait_for` keys: {list(wait_keys)}")

    # ---- the trigger vocabulary ------------------------------------------- #
    actions_found, actions, action_fields, target_all = runner_actions(root)
    if actions_found:
        print(f"  `step` actions: {list(actions)}")
    else:
        _fail(findings, "could not read KNOWN_ACTIONS from "
                        "src/fleet_core/fleet_core/trigger.py, so no `steps` entry can be "
                        "checked. A guard that cannot read its own vocabulary must refuse, "
                        "not pass.")
        print("  [FAIL] the `steps` vocabulary could not be read")
    checks_found, declared_checks = runner_declared_checks(root)

    # ---- what actually runs ------------------------------------------------ #
    # The declaration above says what a case is ALLOWED to name. This says what the runner
    # will DO with it, and they are different questions. Until 2026-09-19 only the first was
    # asked, so ten cases named an action with no implementation and were still given
    # verdicts as though their fault had been delivered.
    impl_found, implemented = runner_implemented_actions(root)
    if not impl_found:
        _fail(findings, "could not read the step actions scripts/batch.py dispatches, so no "
                        "`steps` entry can be checked against what actually runs. A guard that "
                        "cannot read its own implementation must refuse, not pass.")
        print("  [FAIL] the runner's implemented actions could not be read")
    else:
        print(f"  `step` actions the runner IMPLEMENTS: {list(implemented)}")
        if actions_found:
            declared_only = [a for a in actions if a not in implemented]
            if declared_only:
                # A WARNING, not a failure, and the distinction is deliberate.
                #
                # KNOWN_ACTIONS is the vocabulary the CONTRACT permits; `implemented` is what
                # the runner can do. The contract legitimately lists capabilities that are
                # not built yet -- that is what the prerequisites register is for, and
                # section 5's own rule is that a case which cannot run is NOT_RUN rather than
                # a failure. Making this mismatch fatal would mean the batch could not run
                # AT ALL while any contract capability is unbuilt, which is the opposite of
                # "run what can be run and record the rest honestly".
                #
                # What must stay fatal is a case that USES one of these, because that is the
                # case whose fault silently never lands. That check is below, per step.
                print(f"  [WARN] the contract vocabulary is wider than the runner: "
                      f"{declared_only} are declared in KNOWN_ACTIONS and not implemented. "
                      f"No case may use them; any that does is refused below.")

    nodes_found, launch_nodes = launch_node_names(root)

    # ---- prerequisites ---------------------------------------------------- #
    prereqs = manifest.get("prerequisites") or {}
    if not isinstance(prereqs, dict):
        _fail(findings, "`prerequisites` must be a mapping")
        prereqs = {}
    limitations = (root / "docs" / "LIMITATIONS.md")
    lim_text = limitations.read_text(encoding="utf-8") if limitations.is_file() else ""

    # ---- per case --------------------------------------------------------- #
    station_names: set[str] = set()
    charger_names: set[str] = set()
    capability_names: set[str] = set()
    fleet = root / "config" / "fleet.yaml"
    if fleet.is_file():
        body = yaml.safe_load(fleet.read_text(encoding="utf-8")) or {}
        station_names = set((body.get("stations") or {}).keys())
        charger_names = set((body.get("chargers") or {}).keys())
        for spec in (body.get("robots") or {}).values():
            capability_names |= set(spec.get("capabilities") or [])

    truth_found, truth_names = runner_truth_sources(root)

    for cid, body in sorted(cases.items()):
        if not isinstance(body, dict):
            _fail(findings, f"{cid}: not a mapping")
            continue
        if not str(body.get("title", "")).strip():
            _fail(findings, f"{cid}: every case states what it is")
        requires = body.get("requires")
        runnable = "submit" in body or "expect" in body
        if requires and runnable:
            _fail(findings, f"{cid}: declares both `requires` and an expected outcome; a "
                            "blocked case cannot also claim a behaviour to check")
        if not requires and not runnable:
            _fail(findings, f"{cid}: declares neither `requires` nor (`submit` + `expect`)")
        if requires:
            if not isinstance(requires, list):
                _fail(findings, f"{cid}: `requires` must be a list of prerequisite keys")
                requires = []
            for key in requires:
                if key not in prereqs:
                    _fail(findings, f"{cid}: `requires: {key}` is not declared under "
                                    "`prerequisites`, so nobody knows what it is waiting for")
                if key and key not in lim_text:
                    _fail(findings, f"{cid}: prerequisite {key!r} does not appear in "
                                    f"docs/LIMITATIONS.md; a NOT_RUN case has to be explained "
                                    "in the register a reader already looks at")

        # ---- the crossing vocabulary ------------------------------------------ #
        runners = body.get("runners") or ["r01"]
        order = body.get("crossing_order")
        if order is not None:
            if not isinstance(order, list) or not order:
                _fail(findings, f"{cid}: `crossing_order` must be a non-empty list of robots")
            else:
                for named in order:
                    if str(named) not in [str(r) for r in runners]:
                        _fail(findings, f"{cid}: `crossing_order` names {named!r}, which the "
                                        f"case does not run ({list(runners)}). That order can "
                                        "never be observed, so the case would fail after a "
                                        "full-length run instead of here")
        if "crossing_exclusive" in body and not isinstance(body["crossing_exclusive"], bool):
            _fail(findings, f"{cid}: `crossing_exclusive` must be true or false")
        for index, spec in enumerate(body.get("submit") or []):
            if not isinstance(spec, dict):
                continue
            condition = spec.get("wait_for")
            if condition is None:
                continue
            if not isinstance(condition, dict):
                _fail(findings, f"{cid}: submit #{index + 1} `wait_for` must be a mapping")
                continue
            unknown = sorted(set(condition) - set(wait_keys))
            if unknown:
                _fail(findings, f"{cid}: submit #{index + 1} `wait_for` uses {unknown}, which "
                                f"the runner does not know ({list(wait_keys)}). An unknown key "
                                "would hold for ever, so the case would run with no order and "
                                "then run out of budget")
            if "state" in condition and "request_id" not in condition:
                _fail(findings, f"{cid}: submit #{index + 1} `wait_for.state` needs a "
                                "`request_id` to say which task it is about")

        # ---- the trigger vocabulary, per step --------------------------------- #
        case_runners = [str(r) for r in (body.get("runners") or ["r01"])]
        for index, step in enumerate(body.get("steps") or [], 1):
            if not isinstance(step, dict):
                _fail(findings, f"{cid}: step #{index} must be a mapping")
                continue
            condition = step.get("when")
            if condition is not None:
                if not isinstance(condition, Mapping):
                    _fail(findings, f"{cid}: step #{index} `when` must be a mapping")
                else:
                    unknown = sorted(set(condition) - set(wait_keys))
                    if unknown:
                        _fail(findings, f"{cid}: step #{index} `when` uses {unknown}, which the "
                                        f"runner does not know ({list(wait_keys)}). An unknown "
                                        "key would hold for ever, so the fault would never be "
                                        "delivered and the case would be reported as NOT_RUN")
                    if "state" in condition and "request_id" not in condition:
                        _fail(findings, f"{cid}: step #{index} `when.state` needs a "
                                        "`request_id` to say which task it is about")
            do = step.get("do")
            if not isinstance(do, Mapping):
                _fail(findings, f"{cid}: step #{index} needs a `do` mapping")
                continue
            action = str(do.get("action") or "")
            if not action:
                _fail(findings, f"{cid}: step #{index} declares no `do.action`")
            elif action not in actions or action not in implemented:
                _fail(findings, f"{cid}: step #{index} action {action!r} is not implemented "
                                f"(declared: {list(actions)}; the runner actually performs: "
                                f"{list(implemented)}). An action nobody runs would be reported "
                                "as a fleet that survived its fault")
            for field in action_fields.get(action, ()):
                if do.get(field) in (None, ""):
                    _fail(findings, f"{cid}: step #{index} action {action!r} needs `{field}`")
            runner = do.get("runner")
            if runner and str(runner) not in case_runners:
                _fail(findings, f"{cid}: step #{index} names runner {runner!r}, which the case "
                                f"does not run ({case_runners}). The fault would land in a "
                                "robot the case never described")
            node = str(do.get("node") or "")
            if action in ("kill_process", "restart_process"):
                if node and target_all and node == target_all and not runner:
                    _fail(findings, f"{cid}: step #{index} action {action!r} targets "
                                    f"{target_all!r} with no `runner`, which is every node in "
                                    "the fleet")
                if node and node != target_all:
                    if not nodes_found:
                        _fail(findings, f"{cid}: step #{index} names node {node!r} but no "
                                        "launch file could be read, so nothing can say whether "
                                        "that node exists")
                    elif node not in launch_nodes:
                        _fail(findings, f"{cid}: step #{index} names node {node!r}, which no "
                                        f"launch file starts ({list(launch_nodes)}). The "
                                        "pattern would match no process, and the runner "
                                        "reports that as a fault that was not delivered")

        for index, name in enumerate(body.get("behavior_checks") or [], 1):
            if not checks_found:
                _fail(findings, f"{cid}: declares behavior_checks #{index} but "
                                "scripts/batch.py declares no DECLARED_CHECKS, so nothing can "
                                "say whether that check exists")
            elif str(name) not in declared_checks:
                _fail(findings, f"{cid}: behavior check {name!r} is not implemented "
                                f"({list(declared_checks)}). An ignored check makes the case "
                                "look stricter than it is")

        expect = body.get("expect") or {}
        if runnable:
            if not isinstance(expect, dict):
                _fail(findings, f"{cid}: `expect` must be a mapping")
                expect = {}
            for key, allowed in (("task", TASK_OUTCOMES), ("safety", SAFETY_OUTCOMES),
                                 ("behavior", BEHAVIOUR_OUTCOMES)):
                if key not in expect:
                    _fail(findings, f"{cid}: `expect.{key}` is missing; section 1 requires "
                                    "all three outcomes per case")
                elif expect[key] not in allowed:
                    _fail(findings, f"{cid}: expect.{key}={expect[key]!r} is not one of "
                                    f"{sorted(allowed)}")
            truth = body.get("truth")
            if expect.get("safety") == "PASS" and not truth:
                _fail(findings, f"{cid}: declares `expect.safety: PASS` with no `truth:` "
                                "source. A safety verdict has to name the recording it is "
                                "based on; section 1 requires UNKNOWN otherwise")
            if truth:
                if not truth_found:
                    _fail(findings, f"{cid}: `truth: {truth}` but scripts/batch.py declares no "
                                    "TRUTH_SOURCES, so nothing can say whether that source "
                                    "exists")
                elif truth not in truth_names:
                    _fail(findings, f"{cid}: `truth: {truth}` is not implemented by the "
                                    f"runner (it attaches {list(truth_names)}). A case may "
                                    "not name a collector that does not exist")
            scenario = body.get("scenario")
            if scenario:
                if not (root / "config" / "scenarios" / f"{scenario}.yaml").is_file():
                    _fail(findings, f"{cid}: scenario {scenario!r} has no "
                                    f"config/scenarios/{scenario}.yaml")
            for spec in body.get("submit") or []:
                for field, pool, label in (("pick", station_names, "station"),
                                           ("drop", station_names, "station"),
                                           ("capability", capability_names, "capability")):
                    value = spec.get(field)
                    if value and pool and value not in pool:
                        _fail(findings, f"{cid}: submit.{field}={value!r} is not a known "
                                        f"{label} in config/fleet.yaml")
            for rid in body.get("runners") or []:
                if fleet.is_file() and rid not in (yaml.safe_load(
                        fleet.read_text(encoding="utf-8")) or {}).get("robots", {}):
                    _fail(findings, f"{cid}: runner {rid!r} is not in config/fleet.yaml")

    # ---- the seed repeats ------------------------------------------------- #
    repeats = manifest.get("seed_repeats") or {}
    repeat_cases = list(repeats.get("cases") or [])
    repeat_seeds = list(repeats.get("seeds") or [])
    for cid in repeat_cases:
        if cid not in cases:
            _fail(findings, f"seed_repeats names unknown case {cid!r}")
    extra = len(repeat_cases) * len(repeat_seeds)
    if extra != 8:
        _fail(findings, f"seed_repeats produces {extra} extra run(s); section 5 freezes two "
                        "extra seeds for four cases, which is eight")

    if findings:
        for f in findings:
            print(f"  [FAIL] {f}")
        raise SystemExit(f"{len(findings)} finding(s)")

    manifest["_root"] = str(root)
    manifest["_path"] = str(manifest_file)
    return manifest


def planned_runs(manifest: dict) -> "list[dict]":
    """The cases to run, expanded over the frozen seeds. Deterministic order."""
    repeats = manifest.get("seed_repeats") or {}
    repeat_cases = set(repeats.get("cases") or [])
    seeds = [1] + [int(s) for s in (repeats.get("seeds") or [])]
    out: list[dict] = []
    for cid, body in manifest["cases"].items():
        for seed in seeds if cid in repeat_cases else [1]:
            out.append({"case": cid, "seed": seed, "body": body})
    out.sort(key=lambda r: (r["case"], r["seed"]))
    return out


def main(argv: "list[str] | None" = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--manifest", default=str(DEFAULT_MANIFEST))
    ap.add_argument("--root", default=str(ROOT))
    args = ap.parse_args(argv)

    root = Path(args.root).resolve()
    path = Path(args.manifest)
    if not path.is_absolute():
        path = root / path

    print("=" * 78)
    print("  009 batch manifest check (static)")
    print("=" * 78)
    try:
        manifest = load_manifest(path, root)
    except SystemExit as exc:
        print()
        print(f"  manifest refused: {exc}")
        return 1

    runs = planned_runs(manifest)
    runnable = [r for r in runs if not r["body"].get("requires")]
    blocked = [r for r in runs if r["body"].get("requires")]
    print(f"  contract freezes {len(manifest['cases'])} case(s); the manifest covers all of them")
    print(f"  planned runs (with the frozen seed repeats): {len(runs)}")
    print(f"  runnable today: {len(runnable)}")
    print(f"  blocked, each with a prerequisite registered in docs/LIMITATIONS.md: "
          f"{len(blocked)}")
    if blocked:
        by_key: "dict[str, int]" = {}
        for r in blocked:
            for key in r["body"]["requires"]:
                by_key[key] = by_key.get(key, 0) + 1
        for key, count in sorted(by_key.items(), key=lambda kv: (-kv[1], kv[0])):
            print(f"    {count:>2} run(s) waiting on {key}")
    print()
    print("  the manifest matches the contract, and every blocked case is explained (exit 0)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
