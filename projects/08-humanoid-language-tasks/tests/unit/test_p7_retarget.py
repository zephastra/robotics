"""P7 tests: authorised alternative targets, goal update, replan budget.

All fake-backend (no physics): they exercise the planning + goal-update logic
against the two_targets scene (station_b occupied, station_c free).
"""

from __future__ import annotations

from pathlib import Path

from humanoid008.contracts import (
    ExecutionBackend,
    ReasonCode,
    TaskStatus,
)
from humanoid008.execution import (
    BudgetLedger,
    ControlToken,
    SkillExecutor,
    TaskManager,
)
from humanoid008.language import RuleVocabulary
from humanoid008.planners import RulePlanner
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


def build_manager(*, scene="single_target", planner=None, budgets=None):
    s = FakeScene.load(SCENES / f"{scene}.yaml")
    runtime = FakeRuntime(s)
    registry = build_fake_registry(REGISTRY_CONFIG, runtime)
    return TaskManager(
        runtime=runtime,
        registry=registry,
        planner=planner or RulePlanner(registry),
        executor=SkillExecutor(registry),
        vocabulary=RuleVocabulary.load(RULE_CONFIG),
        execution_backend=ExecutionBackend.FAKE,
        budgets=budgets or BudgetLedger(DEFAULT_LIMITS),
        control=ControlToken(),
    )


def test_authorised_alternative_target_is_used_when_primary_occupied():
    """Section 15: when the primary destination is occupied and the user
    authorised an alternative, the task retargets to it and succeeds."""
    outcome = build_manager(scene="two_targets").run_instruction(
        "把箱子搬到 B 台，如果 B 台满了就放到 C 台。"
    )
    assert outcome.task_status is TaskStatus.SUCCEEDED
    assert outcome.goal is not None
    assert outcome.goal.target_id == "station_c"
    assert outcome.goal.goal_revision == 2  # retarget bumped the revision


def test_occupied_primary_without_authorisation_is_refused():
    """Without an authorised alternative, an occupied primary must be refused,
    never silently swapped (section 1.2 / 15)."""
    outcome = build_manager(scene="two_targets").run_instruction("把箱子搬到 B 台。")
    assert outcome.task_status is TaskStatus.FAILED
    assert outcome.reason_code is ReasonCode.NO_FEASIBLE_PLAN


def test_replan_budget_is_enforced_on_retarget():
    """Retargeting is a replan and must respect replans_per_task (section 13)."""
    limits = dict(DEFAULT_LIMITS, replans_per_task=0)
    outcome = build_manager(
        scene="two_targets", budgets=BudgetLedger(limits)
    ).run_instruction("把箱子搬到 B 台，如果 B 台满了就放到 C 台。")
    assert outcome.task_status is TaskStatus.FAILED
    assert outcome.reason_code is ReasonCode.BUDGET_EXHAUSTED
