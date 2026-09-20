#!/usr/bin/env python3
"""Fail the build when the localiser's age bound is not backed by a measurement.

WHY THIS EXISTS
---------------
`LocaliserPolicy` calls a pose usable when

    age <= max_age_s        (2.0 s, from the node parameter `localiser_timeout_s`)   OR
    moved <= max_moved_m    (0.100 m, from `localiser_move_margin_m`)

`check_pose_freshness.py` covers the SECOND condition: it asserts
`update_min_d <= max_moved_m` so the robot cannot cross the hatch between two estimates.

Nothing covered the FIRST one. `pose_source.py` justifies its 2.0 s like this:

    "`max_age_s` defaults to 2.0 s, which is 2.3x the measured p95 age while moving
     (0.853 s) ... Chosen from that table, not for convenience: the analyser prints the
     fraction of samples each candidate bound accepts."

That analyser was a one-off. The number stayed, the measurement did not, and in the
three-robot configuration the age now exceeds the bound: the gate logs

    localised pose unusable, refusing rather than composing odometry: localised pose is
    2.011 s old (> 2.00 s) and the robot has moved 0.124 m since it arrived (> 0.100 m)

-- the AGE condition, not the distance one -- 83 times in N07, 28 in N07_seed3, 11 in
N07_seed2. A guard that checks one of two conditions, and says "OK", is worse than no guard:
it is read as covering both.

WHAT THIS CHECKS, FROM THE RECORDINGS
    * per run: the age distribution while the robot is MOVING, and the fraction of moving
      samples each candidate bound would accept -- so the bound can be re-derived instead of
      re-quoted;
    * FAIL when `localiser_timeout_s` is less than `MIN_FACTOR` x the measured p95 age while
      moving, which is the relationship the docstring claims and nothing enforced;
    * FAIL when any sample carries an age-condition refusal.

Exit 0 when the bound is backed by the recordings, 1 when it is not, 3 on an environment
problem. `--self-test` proves the guard can fail.
"""

from __future__ import annotations

import argparse
import ast
import json
import pathlib
import shutil
import sys
import tempfile

ROOT = pathlib.Path(__file__).resolve().parents[1]
REPORTS = ROOT / "reports"
NODES = [
    ROOT / "src" / "fleet_ros" / "fleet_ros" / "gate_node.py",
    ROOT / "src" / "fleet_ros" / "fleet_ros" / "coordinator_node.py",
]
AGE_KEY = "localiser_timeout_s"
#: The factor `pose_source.py` claims ("2.3x the measured p95 age while moving"). A guard
#: cannot confirm 2.3 was the right choice; it can confirm the relationship still holds.
MIN_FACTOR = 2.3
#: What counts as moving. The gate's own measured speed, in m/s.
MOVING_MPS = 0.02
#: Bounds printed as the derivation table, so the next person re-derives rather than inherits.
CANDIDATES = (0.5, 0.75, 1.0, 1.5, 2.0, 3.0, 5.0)


def declared_age_bounds() -> dict[str, float]:
    """`localiser_timeout_s`'s declared default, per node that declares it."""
    found: dict[str, float] = {}
    for node in NODES:
        if not node.is_file():
            continue
        tree = ast.parse(node.read_text(encoding="utf-8"), filename=str(node))
        for call in ast.walk(tree):
            if not isinstance(call, ast.Call):
                continue
            func = call.func
            if not (isinstance(func, ast.Attribute) and func.attr == "declare_parameter"):
                continue
            if len(call.args) < 2:
                continue
            name, default = call.args[0], call.args[1]
            if not (isinstance(name, ast.Constant) and name.value == AGE_KEY):
                continue
            if isinstance(default, ast.Constant) and isinstance(default.value, (int, float)):
                found[str(node.relative_to(ROOT))] = float(default.value)
    return found


def moving_ages(path: pathlib.Path) -> tuple[list[float], int, list[str]]:
    """Ages while moving, the sample count examined, and any age-condition refusals."""
    ages: list[float] = []
    seen = 0
    refusals: list[str] = []
    for line in path.open(encoding="utf-8"):
        line = line.strip()
        if not line:
            continue
        try:
            sample = json.loads(line)
        except json.JSONDecodeError:
            continue
        for robot, gate in (sample.get("gate") or {}).items():
            if not gate or gate.get("localiser") is None:
                continue
            seen += 1
            age = gate.get("localiser_age_s")
            speed = gate.get("measured_speed_mps")
            verdict = str(gate.get("localiser_verdict") or "")
            if age is not None and isinstance(speed, (int, float)) and speed > MOVING_MPS:
                ages.append(float(age))
            if "old (> " in verdict:
                refusals.append(f"{path.parent.name}:{robot} t={sample.get('t')} {verdict[:80]}")
    return ages, seen, refusals


def _pct(values: list[float], q: float) -> float:
    if not values:
        return float("nan")
    ordered = sorted(values)
    return ordered[min(len(ordered) - 1, int(len(ordered) * q))]


def scan(root: pathlib.Path) -> tuple[dict[str, list[float]], int, list[str]]:
    per_run: dict[str, list[float]] = {}
    seen = 0
    refusals: list[str] = []
    for path in sorted(root.glob("batch_*/*/samples-*.jsonl")):
        if path.parent.parent.name.startswith("_"):
            continue
        ages, n, bad = moving_ages(path)
        seen += n
        refusals.extend(bad)
        if ages:
            per_run[path.parent.name] = ages
    return per_run, seen, refusals


def verdict(root: pathlib.Path, bounds: dict[str, float]) -> int:
    per_run, seen, refusals = scan(root)
    if not bounds:
        print(f"  [FAIL] no node declares {AGE_KEY}; the age bound has no declared default, "
              "so `LocaliserPolicy`'s default is whatever the caller passes")
        return 1
    values = set(bounds.values())
    for where, value in sorted(bounds.items()):
        print(f"  localiser age bound max_age_s = {value:.2f} s   (declared in {where})")
    if len(values) != 1:
        print(f"  [FAIL] the nodes disagree about the age bound: {sorted(values)}. The gate "
              "and the coordinator must make the same decision about the same subject.")
        return 1
    bound = values.pop()

    if not per_run:
        print("  [skip] no recording carries a gate payload with a localiser pose; nothing "
              "to measure. This is NOT a pass.")
        return 0

    all_ages = [a for ages in per_run.values() for a in ages]
    print(f"  measured from {len(per_run)} recording(s), {seen} gate-payload robot-samples, "
          f"{len(all_ages)} of them while moving (speed > {MOVING_MPS} m/s)")
    print()
    print(f"  {'run':<22} {'n':>7} {'p50':>7} {'p95':>7} {'max':>7}")
    for name, ages in sorted(per_run.items()):
        print(f"  {name:<22} {len(ages):>7} {_pct(ages, 0.50):>7.3f} {_pct(ages, 0.95):>7.3f} "
              f"{max(ages):>7.3f}")
    p95 = _pct(all_ages, 0.95)
    print()
    print(f"  pooled p50 {_pct(all_ages, 0.50):.3f} s   p95 {p95:.3f} s   max {max(all_ages):.3f} s")
    print()
    print("  the derivation table the docstring promises (fraction of moving samples each")
    print("  candidate bound would accept):")
    for cand in CANDIDATES:
        frac = sum(1 for a in all_ages if a <= cand) / len(all_ages)
        mark = "  <-- declared" if abs(cand - bound) < 1e-9 else ""
        print(f"    {cand:5.2f} s  {frac * 100:6.1f}%{mark}")
    print()

    needed = MIN_FACTOR * p95
    if bound < needed:
        print(f"  [FAIL] the declared bound {bound:.2f} s is {bound / p95:.2f}x the measured "
              f"p95 age while moving ({p95:.3f} s), but it is justified as {MIN_FACTOR}x.")
        print(f"         {MIN_FACTOR} x {p95:.3f} = {needed:.2f} s. A bound below its own")
        print(f"         justification is a bound nobody re-derived: the gate will refuse on")
        print(f"         the AGE condition, and the robot only gets moving again when the")
        print(f"         localiser happens to publish.")
        print(f"         Re-derive it from the table above (raise `{AGE_KEY}`, or raise AMCL's")
        print(f"         update rate so the ages fall), and correct the docstring in")
        print(f"         src/fleet_core/fleet_core/pose_source.py either way.")
        return 1

    if refusals:
        print(f"  [FAIL] {len(refusals)} sample(s) carry an age-condition refusal; the bound")
        print("         is being hit in real runs:")
        for row in refusals[:5]:
            print(f"           {row}")
        return 1

    print(f"  OK: bound {bound:.2f} s is {bound / p95:.2f}x the measured p95 age while moving "
          f"({p95:.3f} s), at least the {MIN_FACTOR}x it is justified by, and no sample "
          "carries an age-condition refusal")
    return 0


def self_test() -> int:
    """A guard that cannot be shown to fail is not a guard."""
    bounds = declared_age_bounds()
    if not bounds:
        print("self-test: cannot run, no declared age bound")
        return 3
    bound = max(bounds.values())
    tmp = pathlib.Path(tempfile.mkdtemp(prefix="ageguard-"))
    try:
        good = tmp / "batch_20260101T000000Z" / "GOOD"
        bad = tmp / "batch_20260101T000001Z" / "BAD"
        # NOTE: the loop variable must not be called `verdict` -- doing so shadowed the
        # function of that name and turned the self-test into a TypeError.
        for d, age, note in ((good, bound / 4.0, "localised pose is fresh"),
                             (bad, bound * 2.0, f"localised pose is {bound * 2.0:.3f} s old "
                                                f"(> {bound:.2f} s) and the robot has moved "
                                                f"0.124 m since it arrived (> 0.100 m)")):
            d.mkdir(parents=True)
            rows = [{"t": i * 0.05, "gate": {"r01": {
                "localiser": [0.0, 0.0, 0.0], "localiser_age_s": age,
                "measured_speed_mps": 0.25, "localiser_verdict": note}}}
                for i in range(40)]
            (d / "samples-X.jsonl").write_text(
                "\n".join(json.dumps(r) for r in rows), encoding="utf-8")

        only_good = tmp / "_only_good"
        only_good.mkdir()
        shutil.copytree(good.parent, only_good / good.parent.name)

        rc_good = verdict(only_good, bounds)
        rc_bad = verdict(tmp, bounds)
        ok = rc_good == 0 and rc_bad == 1
        print()
        print(f"  self-test: fresh-only tree -> rc {rc_good} (want 0); "
              f"with the stale run -> rc {rc_bad} (want 1) => {'OK' if ok else 'FAILED'}")
        return 0 if ok else 1
    finally:
        shutil.rmtree(tmp, ignore_errors=True)


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--self-test", action="store_true")
    args = ap.parse_args(argv)
    if args.self_test:
        return self_test()
    return verdict(REPORTS, declared_age_bounds())


if __name__ == "__main__":
    raise SystemExit(main())
