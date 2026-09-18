"""Unit tests for the reporting layer.

Covers the two rules that matter most at this stage: a run directory is never
reused, and task_status / physical_outcome / test_verdict stay on separate axes.
"""

from __future__ import annotations

import json

import pytest

from humanoid008 import reporting


def test_run_directories_are_never_reused(tmp_path, monkeypatch):
    monkeypatch.setattr(reporting, "utc_stamp", lambda: "20260101T000000Z")
    created = reporting.create_run_directory(tmp_path, "abc123")
    assert created.is_dir()
    with pytest.raises(FileExistsError):
        reporting.create_run_directory(tmp_path, "abc123")


def test_report_keeps_three_status_axes_separate(tmp_path):
    run_dir = tmp_path / "run"
    run_dir.mkdir()
    path = reporting.write_report(
        run_dir,
        task_status="STOPPED",
        physical_outcome="NOT_APPLICABLE",
        test_verdict="PASS",
    )
    payload = json.loads(path.read_text(encoding="utf-8"))
    assert payload["task_status"] == "STOPPED"
    assert payload["physical_outcome"] == "NOT_APPLICABLE"
    assert payload["test_verdict"] == "PASS"


def test_report_rejects_unknown_status(tmp_path):
    run_dir = tmp_path / "run"
    run_dir.mkdir()
    with pytest.raises(ValueError):
        reporting.write_report(
            run_dir,
            task_status="DONE",
            physical_outcome="PASS",
            test_verdict="PASS",
        )


def test_event_log_is_append_only_jsonl(tmp_path):
    log = reporting.EventLog(tmp_path / "events.jsonl")
    log.record("run_started", run_id="abc123")
    log.record("note", text="hello")
    entries = log.read()
    assert [e["event"] for e in entries] == ["run_started", "note"]
    assert entries[0]["run_id"] == "abc123"


def test_manifest_records_backend_labels(tmp_path):
    path = reporting.write_manifest(
        tmp_path,
        run_id="abc123",
        execution_backend="fake",
        planner_backend="rule",
        clock_mode="fake_clock",
        scene=None,
        seed=7,
    )
    payload = json.loads(path.read_text(encoding="utf-8"))
    assert payload["execution_backend"] == "fake"
    assert payload["planner_backend"] == "rule"
    assert payload["seed"] == 7
