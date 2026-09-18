"""P3 integration tests: the control loop under supervision.

These exercise the full TaskManager with the supervision layers, budgets and
cancellation wired in, and prove the section-13 gate:

- STOP still works while the model hangs (measured, not assumed);
- an illegal proposal cannot execute;
- a stale revision is discarded, not applied;
- a missing model provider is recorded as NOT_RUN_MISSING_PROVIDER;
- replay is labelled replay, never llm;
- budgets are enforced and cannot be reset.
"""

from __future__ import annotations

import threading
import time
from pathlib import Path

import pytest

from humanoid008.contracts import (
    ExecutionBackend,
    GoalSpec,
    Intent,
    ProposalKind,
    ReasonCode,
    SkillName,
    TaskStatus,
)
from humanoid008.execution import (
    BudgetLedger,
    ControlToken,
    SkillExecutor,
    TaskManager,
)
from humanoid008.language import RuleVocabulary
from humanoid008.planners import (
    BridgeConfig,
    HttpPlannerBridge,
    LlmPlanner,
    Planner,
    ReplayPlanner,
    RulePlanner,
)
from humanoid008.skills import FakeRuntime, FakeScene, build_fake_registry

PROJECT_ROOT = Path(__file__).resolve().parents[2]
REGISTRY_CONFIG = PROJECT_ROOT / "config" / "skills" / "registry.yaml"
RULE_CONFIG = PROJECT_ROOT / "config" / "planners" / "rule.yaml"
LLM_CONFIG = PROJECT_ROOT / "config" / "planners" / "llm.yaml"
SCENES = PROJECT_ROOT / "config" / "scenes"
REPLAY_FILE = PROJECT_ROOT / "tests" / "fixtures" / "model_responses" / "transport.jsonl"

TRANSPORT_SEQUENCE = [
    SkillName.OBSERVE_OBJECT,
    SkillName.OBSERVE_TARGET,
    SkillName.APPROACH_OBJECT,
    SkillName.GRASP_OBJECT,
    SkillName.LIFT_OBJECT,
    SkillName.CARRY_TO_TARGET,
    SkillName.PLACE_OBJECT,
    SkillName.VERIFY_RESULT,
]

DEFAULT_LIMITS = {
    "max_steps_per_task": 32,
    "model_calls_per_task": 20,
    "replans_per_task": 3,
    "retries_per_skill": 2,
    "stance_realign_per_task": 2,
}


def build_manager(*, planner=None, budgets=None, control=None, faults=()):
    scene = FakeScene.load(SCENES / "single_target.yaml")
    runtime = FakeRuntime(scene, faults=faults)
    registry = build_fake_registry(REGISTRY_CONFIG, runtime)
    return TaskManager(
        runtime=runtime,
        registry=registry,
        planner=planner or RulePlanner(registry),
        executor=SkillExecutor(registry),
        vocabulary=RuleVocabulary.load(RULE_CONFIG),
        execution_backend=ExecutionBackend.FAKE,
        budgets=budgets or BudgetLedger(DEFAULT_LIMITS),
        control=control or ControlToken(),
    )


# --------------------------------------------------------------------------- #
# planners for fault scenarios
# --------------------------------------------------------------------------- #


class BlockingPlanner(Planner):
    """A model that hangs forever, for the STOP test."""

    name = "blocking"

    def __init__(self):
        self.entered = threading.Event()

    def propose(self, *, goal, world, history, request_id):
        self.entered.set()
        threading.Event().wait()  # never returns
        raise AssertionError("unreachable")


class InvalidSkillPlanner(Planner):
    """Returns a proposal naming a skill that does not exist."""

    name = "invalid"

    def propose(self, *, goal, world, history, request_id):
        from humanoid008.contracts import ActionProposal

        return ActionProposal(
            request_id=request_id,
            task_id=goal.task_id,
            goal_revision=goal.goal_revision,
            world_revision=world.world_revision,
            kind=ProposalKind.SKILL,
            skill="EXEC_SHELL",
            reason="injected by test",
            args={},
        )


class StaleWorldPlanner(Planner):
    """Wraps the rule planner but forces a stale world_revision."""

    name = "stale_world"

    def __init__(self, registry):
        self._inner = RulePlanner(registry)

    def propose(self, *, goal, world, history, request_id):
        from humanoid008.contracts import ActionProposal

        proposal = self._inner.propose(
            goal=goal, world=world, history=history, request_id=request_id
        )
        if proposal is None or proposal.skill is None:
            return proposal
        return ActionProposal.for_skill(
            request_id=request_id,
            task_id=goal.task_id,
            goal_revision=goal.goal_revision,
            world_revision=world.world_revision + 999,
            skill=proposal.skill,
            reason=proposal.reason,
            args=proposal.args,
        )


# --------------------------------------------------------------------------- #
# gate 1: STOP pre-empts a hung model
# --------------------------------------------------------------------------- #


def test_stop_preempts_a_hung_model_within_200ms():
    planner = BlockingPlanner()
    control = ControlToken()
    manager = build_manager(planner=planner, control=control)

    outcome_box: dict = {}

    def run():
        outcome_box["outcome"] = manager.run_instruction("把箱子搬到 B 台。")

    thread = threading.Thread(target=run)
    thread.start()
    # Wait until the planner is actually inside propose(), then issue STOP.
    assert planner.entered.wait(timeout=2.0), "planner never entered propose()"
    t0 = time.monotonic()
    control.request_stop(reason="user stop", now_s=t0)
    thread.join(timeout=2.0)

    assert not thread.is_alive(), "the control loop did not stop"
    outcome = outcome_box["outcome"]
    assert outcome.task_status is TaskStatus.STOPPED
    assert outcome.reason_code is ReasonCode.CANCELLED
    assert control.latency_s is not None
    assert control.latency_s < 0.2, f"stop latency {control.latency_s:.4f}s exceeded 200ms"


# --------------------------------------------------------------------------- #
# gate 2: illegal proposals cannot execute
# --------------------------------------------------------------------------- #


def test_an_illegal_skill_proposal_is_rejected():
    outcome = build_manager(planner=InvalidSkillPlanner()).run_instruction(
        "把箱子搬到 B 台。"
    )
    assert outcome.task_status is TaskStatus.FAILED
    assert outcome.reason_code is ReasonCode.PRECONDITION_FAILED
    assert outcome.history == ()
    assert "rejected_proposal" in outcome.detail


# --------------------------------------------------------------------------- #
# gate 3: stale revision is discarded
# --------------------------------------------------------------------------- #


def test_stale_world_revision_is_discarded():
    scene = FakeScene.load(SCENES / "single_target.yaml")
    runtime = FakeRuntime(scene)
    registry = build_fake_registry(REGISTRY_CONFIG, runtime)
    outcome = build_manager(planner=StaleWorldPlanner(registry)).run_instruction(
        "把箱子搬到 B 台。"
    )
    assert outcome.task_status is TaskStatus.FAILED
    assert outcome.reason_code is ReasonCode.REVISION_MISMATCH
    assert outcome.history == ()


# --------------------------------------------------------------------------- #
# gate 4: missing provider is recorded, never faked
# --------------------------------------------------------------------------- #


def test_llm_without_provider_is_not_run_missing_provider():
    config = BridgeConfig.from_env(LLM_CONFIG, env={})
    bridge = HttpPlannerBridge(config)
    scene = FakeScene.load(SCENES / "single_target.yaml")
    runtime = FakeRuntime(scene)
    registry = build_fake_registry(REGISTRY_CONFIG, runtime)
    planner = LlmPlanner(bridge, registry)
    outcome = build_manager(planner=planner).run_instruction("把箱子搬到 B 台。")
    assert outcome.task_status is TaskStatus.FAILED
    assert outcome.reason_code is ReasonCode.NOT_RUN_MISSING_PROVIDER
    assert outcome.history == ()


# --------------------------------------------------------------------------- #
# gate 5: replay is labelled replay, never llm
# --------------------------------------------------------------------------- #


def test_replay_runs_the_recorded_sequence_and_is_not_llm():
    planner = ReplayPlanner.from_jsonl(REPLAY_FILE)
    assert planner.name == "replay"
    outcome = build_manager(planner=planner).run_instruction("把箱子搬到 B 台。")
    assert outcome.task_status is TaskStatus.SUCCEEDED
    assert [result.skill for result in outcome.history] == TRANSPORT_SEQUENCE


# --------------------------------------------------------------------------- #
# gate 6: budgets are enforced
# --------------------------------------------------------------------------- #


def test_step_budget_is_enforced_and_not_resettable():
    limits = dict(DEFAULT_LIMITS, max_steps_per_task=2)
    budgets = BudgetLedger(limits)
    outcome = build_manager(budgets=budgets).run_instruction("把箱子搬到 B 台。")
    assert outcome.task_status is TaskStatus.FAILED
    assert outcome.reason_code is ReasonCode.BUDGET_EXHAUSTED
    # Exactly two steps ran before the budget gate stopped the loop.
    assert [result.skill for result in outcome.history] == [
        SkillName.OBSERVE_OBJECT,
        SkillName.OBSERVE_TARGET,
    ]
