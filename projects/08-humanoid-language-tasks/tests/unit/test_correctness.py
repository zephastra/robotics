"""Regression tests for the external review's 5 correctness findings.

Each test first documents the bug (red now), then the implementation is fixed so
the test goes green. This keeps the "fail-first" discipline of section 17.
"""

from __future__ import annotations

from pathlib import Path

from humanoid008.contracts import ExecutionBackend, TaskStatus
from humanoid008.execution import (
    BudgetLedger,
    ControlToken,
    SkillExecutor,
    TaskManager,
)
from humanoid008.language import RuleVocabulary
from humanoid008.planners import ReplayPlanner
from humanoid008.skills import FakeRuntime, FakeScene, build_fake_registry

PROJECT_ROOT = Path(__file__).resolve().parents[2]
REGISTRY_CONFIG = PROJECT_ROOT / "config" / "skills" / "registry.yaml"
RULE_CONFIG = PROJECT_ROOT / "config" / "planners" / "rule.yaml"
SCENES = PROJECT_ROOT / "config" / "scenes"

DEFAULT_LIMITS = {
    "max_steps_per_task": 32,
    "model_calls_per_task": 20,
    "replans_per_task": 3,
    "retries_per_skill": 2,
    "stance_realign_per_task": 2,
}


def build_manager(*, planner):
    scene = FakeScene.load(SCENES / "single_target.yaml")
    runtime = FakeRuntime(scene)
    registry = build_fake_registry(REGISTRY_CONFIG, runtime)
    return TaskManager(
        runtime=runtime,
        registry=registry,
        planner=planner,
        executor=SkillExecutor(registry),
        vocabulary=RuleVocabulary.load(RULE_CONFIG),
        execution_backend=ExecutionBackend.FAKE,
        budgets=BudgetLedger(DEFAULT_LIMITS),
        control=ControlToken(),
    )


# --------------------------------------------------------------------------- #
# finding 1: empty planner must not be treated as goal-complete
# --------------------------------------------------------------------------- #


def test_empty_planner_must_not_report_transport_success():
    """An empty planner (returns None) currently yields SUCCEEDED with zero skills
    executed (task_manager.py treats `proposal is None` as goal-complete without
    checking postconditions). It must instead fail the transport task."""
    outcome = build_manager(planner=ReplayPlanner([])).run_instruction(
        "把箱子搬到 B 台。"
    )
    assert outcome.task_status is not TaskStatus.SUCCEEDED
    assert outcome.history == ()
