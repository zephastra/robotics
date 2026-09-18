"""Reproducible bounded initial-condition suite for the unsplit baseline (P4).

This is the "未拆分的固定任务回归": 30 cases (3 regression + 9 box-grid +
9 target-grid + 9 combined-yaw), each run through 008's own ``backend`` in its
own report directory. Failed cases are never discarded.

Run it only after ``bash scripts/setup.sh --profile sim``:

    MUJOCO_GL=egl .venv/bin/python scripts/batch.py --workers 1 --duration 120

It is intentionally NOT run by the test suite (it is minutes-to-hours of CPU).
"""

from __future__ import annotations

import argparse
import json
import random
import subprocess
import sys
import time
from collections import Counter
from concurrent.futures import ThreadPoolExecutor, as_completed
from datetime import datetime, timezone
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def cases(seed=7):
    rng = random.Random(seed)
    result = []

    def add(group, box=(0.0, 0.0), target=(0.0, 0.0), yaw=0.0):
        result.append(
            dict(
                id=f"{len(result) + 1:02d}-{group}",
                group=group,
                box=list(box),
                target=list(target),
                yaw=yaw,
            )
        )

    add("regression")
    add("regression", (0.02, 0.01), (0.05, 0.03))
    add("regression", (-0.02, -0.01), (-0.05, -0.03))
    for x in (-0.02, 0.0, 0.02):
        for y in (-0.01, 0.0, 0.01):
            add("box-grid", (x, y))
    for x in (-0.05, 0.0, 0.05):
        for y in (-0.03, 0.0, 0.03):
            add("target-grid", target=(x, y))
    for _ in range(9):
        add(
            "combined-yaw",
            tuple(round(rng.uniform(-v, v), 5) for v in (0.02, 0.01)),
            tuple(round(rng.uniform(-v, v), 5) for v in (0.05, 0.03)),
            round(rng.uniform(-5.0, 5.0), 3),
        )
    return result


def run_case(case, destination: Path, duration: float, timeout: float, split: bool = False,
             station: str = "B") -> dict:
    command = [
        sys.executable,
        "-m",
        "humanoid008.simulation.backend",
        "--headless",
        "--duration",
        str(duration),
        "--box-offset",
        *map(str, case["box"]),
        "--target-offset",
        *map(str, case["target"]),
        "--box-yaw",
        str(case["yaw"]),
    ]
    if split:
        command += ["--split", "--instruction", f"\u628a\u7bb1\u5b50\u642c\u5230 {station} \u53f0\u3002"]
    started = time.monotonic()
    timed_out = False
    try:
        proc = subprocess.run(
            command,
            cwd=ROOT,
            capture_output=True,
            text=True,
            timeout=timeout,
            env={**__import__("os").environ, "MUJOCO_GL": "egl"},
        )
        returncode = proc.returncode
        report_dir = _find_report_dir(proc.stdout)
    except subprocess.TimeoutExpired:
        returncode = None
        timed_out = True
        report_dir = None
    wall = time.monotonic() - started

    report = None
    if report_dir is not None:
        report_file = report_dir / "report.json"
        if report_file.is_file():
            report = json.loads(report_file.read_text())

    verdict = classify(returncode, report, timed_out, split)
    return dict(
        id=case["id"],
        group=case["group"],
        verdict=verdict,
        wall_s=round(wall, 1),
        report_dir=None if report_dir is None else report_dir.name,
    )


def _find_report_dir(stdout: str) -> Path | None:
    """The backend writes its run dir under reports/; find the newest one."""
    dirs = sorted((ROOT / "reports").glob("*"), key=lambda p: p.stat().st_mtime)
    if not dirs:
        return None
    return dirs[-1]


def classify(returncode, report, timed_out, split=False):
    if timed_out:
        return "WALL_TIMEOUT"
    if report is None:
        return "MISSING_REPORT"
    ok_status = "SUCCEEDED" if split else "COMPLETED"
    if returncode == 0 and report.get("status") == ok_status:
        if split and report.get("physical_outcome") != "PASS":
            return "EVALUATION_REJECTED"
        return "SUCCEEDED" if split else "COMPLETED"
    if report.get("status") == ok_status:
        return "INCONSISTENT_EXIT"
    return report.get("status", "INVALID_REPORT")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--seed", type=int, default=7)
    parser.add_argument("--duration", type=float, default=120.0)
    parser.add_argument("--timeout", type=float, default=900.0)
    parser.add_argument("--workers", type=int, default=1)
    parser.add_argument("--limit", type=int, default=0, help="run only the first N cases (0 = all)")
    parser.add_argument("--split", action="store_true", help="run the P5 skill-based task instead of the unsplit baseline")
    parser.add_argument(
        "--station",
        default="B",
        choices=["B", "C"],
        help="destination station for --split runs (default: B)",
    )
    args = parser.parse_args()

    suite = cases(args.seed)
    if args.limit:
        suite = suite[: args.limit]

    print(f"cases: {len(suite)}  duration={args.duration}s  workers={args.workers}  split={args.split}  station={args.station}")
    results = []
    with ThreadPoolExecutor(max_workers=args.workers) as pool:
        futures = {
            pool.submit(
                run_case, c, ROOT / "reports", args.duration, args.timeout,
                args.split, args.station,
            ): c
            for c in suite
        }
        for future in as_completed(futures):
            results.append(future.result())
            r = results[-1]
            print(f"  [{r['id']}] {r['verdict']}  ({r['wall_s']}s)", flush=True)

    results.sort(key=lambda r: r["id"])
    tally = Counter(r["verdict"] for r in results)
    task_kind = "skill_split" if args.split else "baseline_unsplit"
    summary = {
        "schema_version": "1.0",
        "task_kind": task_kind,
        "seed": args.seed,
        "duration": args.duration,
        "tally": dict(tally),
        "results": results,
        "finished_utc": datetime.now(timezone.utc).isoformat(),
    }
    prefix = "split" if args.split else "baseline"
    if args.station != "B":
        prefix = f"{prefix}-{args.station}"
    out = ROOT / "reports" / f"{prefix}-summary-{datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')}.json"
    out.write_text(json.dumps(summary, ensure_ascii=False, indent=2) + "\n")
    print(f"summary: {out.relative_to(ROOT)}")
    print(f"tally: {dict(tally)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
