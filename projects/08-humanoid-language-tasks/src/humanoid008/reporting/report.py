"""Run directory layout and JSON writers.

Contract (docs/MASTER_PLAN.md sections 16 and 25.4):

- every run gets its own unique directory; history is never overwritten;
- ``task_status``, ``physical_outcome`` and ``test_verdict`` are recorded on
  three separate axes and must never be collapsed into one "success" flag.
"""

from __future__ import annotations

import json
import platform
import uuid
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Iterable, Mapping

SCHEMA_VERSION = "1.0"

TASK_STATUSES = (
    "IDLE",
    "PLANNING",
    "WAITING_CLARIFICATION",
    "EXECUTING",
    "REPLANNING",
    "SUCCEEDED",
    "FAILED",
    "CANCELLED",
    "STOPPED",
)
PHYSICAL_OUTCOMES = ("PASS", "FAIL", "NOT_APPLICABLE", "NOT_EVALUATED")
TEST_VERDICTS = ("PASS", "FAIL", "NOT_RUN")


def utc_stamp() -> str:
    """Filesystem-safe UTC timestamp used as the run-directory prefix."""
    return datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")


def new_run_id() -> str:
    """Short unique run identifier."""
    return uuid.uuid4().hex[:12]


def create_run_directory(report_root: Path, run_id: str) -> Path:
    """Create ``<report_root>/<UTC>-<run_id>/``.

    Fails instead of reusing an existing directory, so a previous run can never
    be silently overwritten.
    """
    run_dir = Path(report_root) / f"{utc_stamp()}-{run_id}"
    run_dir.mkdir(parents=True, exist_ok=False)
    return run_dir


def write_json(path: Path, payload: Mapping[str, Any]) -> Path:
    path = Path(path)
    path.write_text(
        json.dumps(payload, ensure_ascii=False, indent=2),
        encoding="utf-8",
    )
    return path


def write_jsonl(path: Path, rows: Iterable[Mapping[str, Any]]) -> Path:
    """Append-oriented JSONL writer (rewrites the file, one object per line)."""
    path = Path(path)
    with path.open("w", encoding="utf-8") as handle:
        for row in rows:
            handle.write(json.dumps(row, ensure_ascii=False) + "\n")
    return path


def write_manifest(
    run_dir: Path,
    *,
    run_id: str,
    execution_backend: str,
    planner_backend: str,
    clock_mode: str,
    scene: str | None,
    seed: int | None,
) -> Path:
    manifest = {
        "schema_version": SCHEMA_VERSION,
        "run_id": run_id,
        "created_utc": datetime.now(timezone.utc).isoformat(),
        "execution_backend": execution_backend,
        "planner_backend": planner_backend,
        "clock_mode": clock_mode,
        "scene": scene,
        "seed": seed,
        "python_version": platform.python_version(),
        "platform": platform.platform(),
        "implemented_stages": ["P0", "P1", "P2", "P3"],
        "notes": (
            "fake backend: no physics, no rendering, no MuJoCo. "
            "physical_outcome is NOT_EVALUATED by design."
            if execution_backend == "fake"
            else "requested backend is not implemented yet; no task was executed."
        ),
    }
    return write_json(Path(run_dir) / "manifest.json", manifest)


def write_input(run_dir: Path, *, instruction: str | None, interactive: bool) -> Path:
    return write_json(
        Path(run_dir) / "input.json",
        {"instruction": instruction, "interactive": bool(interactive)},
    )


def write_report(
    run_dir: Path,
    *,
    task_status: str,
    physical_outcome: str,
    test_verdict: str,
    detail: Mapping[str, Any] | None = None,
) -> Path:
    """Write the final report, keeping the three status axes explicitly separate."""
    if task_status not in TASK_STATUSES:
        raise ValueError(f"unknown task_status: {task_status}")
    if physical_outcome not in PHYSICAL_OUTCOMES:
        raise ValueError(f"unknown physical_outcome: {physical_outcome}")
    if test_verdict not in TEST_VERDICTS:
        raise ValueError(f"unknown test_verdict: {test_verdict}")

    report = {
        "schema_version": SCHEMA_VERSION,
        "finished_utc": datetime.now(timezone.utc).isoformat(),
        "task_status": task_status,
        "physical_outcome": physical_outcome,
        "test_verdict": test_verdict,
        "detail": dict(detail or {}),
    }
    return write_json(Path(run_dir) / "report.json", report)
