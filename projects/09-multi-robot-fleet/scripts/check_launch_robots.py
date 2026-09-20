#!/usr/bin/env python3
"""Guard: a case's `runners` and the launch's `robots:=` must come from one source.

Round 17 wrote a live verification run by hand and omitted `robots:=`. The launch
default is `r01,r02`, so a two-robot world came up while the recorder audit still
said `robots = ['r01','r02','r03']`. Every odometry-derived check looked healthy,
because a robot with no body still has an odometry bridge, and the truth stream
carried 16 slots (2 models) against a 3-robot declaration.

Nothing in the tree could see that, and it produced a wrong written conclusion
("the r02 body is missing; ros_gz_sim/create segfaulted") that survived until the
slot count was compared against the declaration by hand.

This guard makes the mismatch impossible to reach silently:

  1. `scripts/batch.py` must derive `robots:=` from the case body's `runners`
     (it does: `runs = list(body.get("runners") or ["r01"])`, then
     `f"robots:={','.join(self.runs)}"`). Assert both statements still exist.
  2. Every case that declares `runners` must name robots `config/fleet.yaml`
     defines, with no duplicates.
  3. Any file that actually SPAWNS the launch -- i.e. builds an argv list whose
     first element is the string "ros2" and which contains "fleet.launch.py" --
     must also pass `robots:=`. A file that merely reads or mentions the launch
     file (a static checker, the manifest freezer) is not a launcher and is not
     an offender. That distinction is the whole point: a name-based check on
     "fleet.launch.py" flags the wrong files, which is the same class of mistake
     this repository keeps making.

Exit 0 when everything agrees, 1 otherwise.
"""

from __future__ import annotations

import ast
import pathlib
import re
import sys

import yaml

#: The tree to check. Defaults to this file's repository; FLEET009_ROOT points it
#: elsewhere so the guard's own tests can hand it a deliberately broken tree and
#: require a non-zero exit. A guard that can only agree is not a guard.
import os as _os

ROOT = pathlib.Path(_os.environ.get("FLEET009_ROOT")
                    or pathlib.Path(__file__).resolve().parent.parent)
BATCH = ROOT / "scripts/batch.py"
FLEET_CONFIG = ROOT / "config/fleet.yaml"
MANIFEST = ROOT / "config/scenarios/regression_v1.yaml"

failures: list[str] = []
notes: list[str] = []


# --------------------------------------------------------------------------- #
# 1. batch.py derives the robot list from the case, and only from the case
# --------------------------------------------------------------------------- #
def check_batch_source() -> None:
    if not BATCH.exists():
        failures.append(f"{BATCH.relative_to(ROOT)}: missing")
        return
    src = BATCH.read_text(encoding="utf-8")

    derive = re.search(
        r"^\s*runs\s*=\s*list\(\s*body\.get\(\s*[\"']runners[\"']\s*\)\s*or\s*\[[^\]]*\]\s*\)",
        src, re.MULTILINE)
    if not derive:
        failures.append(
            "scripts/batch.py: no `runs = list(body.get(\"runners\") or [...])`. "
            "The robot list MUST come from the case body, or the launch argument "
            "can disagree with the case's own declaration.")
    else:
        notes.append("batch.py derives runs from runners at line "
                     f"{src[:derive.start()].count(chr(10)) + 1}")

    if re.search(r"[\"']robots:=\{?[^\"']*[\"']", src):
        notes.append("batch.py passes robots:= built from a robot list")
    else:
        failures.append(
            "scripts/batch.py: Fleet._launch_args no longer passes `robots:=`")

    tree = ast.parse(src)
    for node in ast.walk(tree):
        if isinstance(node, ast.FunctionDef) and node.name == "_launch_args":
            arg_src = ast.get_source_segment(src, node) or ""
            other = re.findall(r"robots:=\{[^}]*\bself\.(\w+)\b", arg_src)
            bad = [o for o in other if o != "runs"]
            if bad:
                failures.append(
                    f"scripts/batch.py: _launch_args builds robots:= from "
                    f"self.{bad[0]}, not self.runs")


# --------------------------------------------------------------------------- #
# 2. every declared runner exists in the fleet config
# --------------------------------------------------------------------------- #
def check_runners_exist() -> None:
    if not MANIFEST.exists():
        failures.append(f"{MANIFEST.relative_to(ROOT)}: missing")
        return
    cfg = yaml.safe_load(FLEET_CONFIG.read_text(encoding="utf-8")) or {}
    known = set((cfg.get("robots") or {}).keys())

    body = yaml.safe_load(MANIFEST.read_text(encoding="utf-8")) or {}
    cases = body.get("cases") or {}
    if not isinstance(cases, dict):
        failures.append("regression_v1.yaml: `cases` is not a mapping")
        return

    n_three = 0
    for cid, case in sorted(cases.items()):
        runners = (case or {}).get("runners")
        if not runners:
            continue
        if len(runners) >= 3:
            n_three += 1
        unknown = [r for r in runners if r not in known]
        if unknown:
            failures.append(
                f"regression_v1.yaml: case {cid} names runner(s) {unknown} "
                f"that config/fleet.yaml does not define {sorted(known)}")
        if len(set(runners)) != len(runners):
            failures.append(
                f"regression_v1.yaml: case {cid} lists a duplicate runner: {runners}")
    notes.append(f"{len(cases)} case(s), {n_three} of them three-robot, "
                 f"all runners present in config/fleet.yaml {sorted(known)}")


# --------------------------------------------------------------------------- #
# 3. anything that SPAWNS the launch must pass robots:=
# --------------------------------------------------------------------------- #
def _launches_fleet(path: pathlib.Path) -> bool:
    """True only if this file builds an argv that actually starts the launch.

    A name-based search for "fleet.launch.py" flags static checkers and the
    manifest freezer, which read the file rather than run it. Requiring a
    "ros2" argv element alongside it separates a launcher from a reader.
    """
    try:
        tree = ast.parse(path.read_text(encoding="utf-8", errors="replace"))
    except SyntaxError:
        return False
    for node in ast.walk(tree):
        if not isinstance(node, (ast.List, ast.Tuple)):
            continue
        elts = [e.value for e in node.elts
                if isinstance(e, ast.Constant) and isinstance(e.value, str)]
        if "ros2" in elts and any("fleet.launch.py" in e for e in elts):
            return True
    return False


def check_no_hand_launch() -> None:
    offenders = []
    for py in sorted((ROOT / "scripts").glob("*.py")):
        if py.name in ("batch.py", "check_launch_robots.py"):
            continue
        if not _launches_fleet(py):
            continue
        src = py.read_text(encoding="utf-8", errors="replace")
        if "robots:=" not in src:
            offenders.append(py.name)
    if offenders:
        failures.append(
            "these scripts spawn fleet.launch.py without robots:= -- the launch "
            f"default is r01,r02, so they would silently run a two-robot world: "
            f"{offenders}")
    else:
        notes.append("every script that spawns fleet.launch.py passes robots:=")


def main() -> int:
    check_batch_source()
    check_runners_exist()
    check_no_hand_launch()

    print("=" * 78)
    print("  009 launch robot-set check (static)")
    print("=" * 78)
    for n in notes:
        print(f"    {n}")

    if failures:
        print()
        for f in failures:
            print(f"  [FAIL] {f}")
        print()
        print("A case's runners and the launch's robots:= must be the same list,")
        print("or a run can record a world that is missing a robot the case asked")
        print("for while every odometry-derived check still passes.")
        return 1

    print()
    print("    the case's runners and the launch's robots:= cannot disagree (exit 0)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
