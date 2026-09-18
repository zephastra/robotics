"""Unit tests for CLI parsing, configuration validation and stage reporting.

These tests never touch MuJoCo, MNN or the network.
"""

from __future__ import annotations

import pytest

from humanoid008 import app


def _args(*extra: str):
    return app.build_parser().parse_args(["--instruction", "把箱子搬到 B 台", *extra])


def test_instruction_and_interactive_are_mutually_exclusive():
    args = app.build_parser().parse_args(["--instruction", "x", "--interactive"])
    with pytest.raises(app.ConfigError):
        app.validate_args(args)


def test_requires_instruction_or_interactive():
    args = app.build_parser().parse_args([])
    with pytest.raises(app.ConfigError):
        app.validate_args(args)


def test_headless_and_keep_open_contradict():
    args = _args("--headless", "--keep-open")
    with pytest.raises(app.ConfigError):
        app.validate_args(args)


def test_negative_seed_is_rejected():
    args = _args("--seed", "-1")
    with pytest.raises(app.ConfigError):
        app.validate_args(args)


def test_report_root_is_confined_to_project_reports(tmp_path):
    with pytest.raises(app.ConfigError):
        app.resolve_report_root(str(tmp_path))


def test_default_report_root_equals_project_reports():
    assert app.resolve_report_root(None) == (app.PROJECT_ROOT / "reports").resolve()


def test_unimplemented_message_names_the_blocking_stage():
    # fake + rule/replay/llm are all implemented as of P3; only mujoco blocks.
    assert app.describe_unimplemented(_args()) == ""
    assert app.describe_unimplemented(_args("--planner", "replay")) == ""
    assert app.describe_unimplemented(_args("--planner", "llm")) == ""
    assert "P4" in app.describe_unimplemented(_args("--backend", "mujoco"))


def test_main_returns_not_implemented_for_the_mujoco_backend():
    code = app.main(["--backend", "mujoco", "--instruction", "x"])
    assert code == app.EXIT_NOT_IMPLEMENTED


def test_replay_file_requires_replay_planner():
    args = app.build_parser().parse_args(
        ["--instruction", "x", "--planner", "rule", "--replay-file", "foo.jsonl"]
    )
    with pytest.raises(app.ConfigError):
        app.validate_args(args)


def test_main_rejects_bad_configuration_with_usage_code(tmp_path):
    code = app.main(["--instruction", "x", "--report-root", str(tmp_path)])
    assert code == app.EXIT_USAGE
