"""P6 tests: language-driven real simulation (planner selection + clock mode).

Fast tests cover ambiguity and the missing-provider path; the full replay-driven
transport is marked slow (~40 s, exercises the whole 8-skill chain).
"""

from __future__ import annotations

import argparse
import shutil
from datetime import datetime, timezone

import pytest

pytest.importorskip("mujoco", reason="simulation profile not installed (bash scripts/setup.sh --profile sim)")
pytest.importorskip("MNN", reason="simulation profile not installed")

from humanoid008.simulation import ROOT  # noqa: E402
from humanoid008.simulation.backend import run_skill_task  # noqa: E402


def _args(**overrides):
    base = dict(
        box_offset=[0.0, 0.0],
        target_offset=[0.0, 0.0],
        box_yaw=0.0,
        stance="auto",
        instruction="把箱子搬到 B 台。",
        split=True,
        planner="rule",
        replay_file=None,
        clock_mode=None,
    )
    base.update(overrides)
    return argparse.Namespace(**base)


def _run(args):
    run_dir = ROOT / "reports" / datetime.now(timezone.utc).strftime("test-p6-%Y%m%dT%H%M%S%fZ")
    run_dir.mkdir(parents=True, exist_ok=False)
    try:
        return run_skill_task(args, run_dir)
    finally:
        shutil.rmtree(run_dir, ignore_errors=True)


def test_llm_without_provider_records_not_run_missing_provider():
    """A real-model run without a configured endpoint must be recorded as
    NOT_RUN_MISSING_PROVIDER, never silently downgraded to rule (section 12)."""
    report = _run(_args(planner="llm"))
    assert report["planner_backend"] == "llm"
    assert report["clock_mode"] == "planning_pause"
    assert report["status"] == "FAILED"
    assert report["reason_code"] == "NOT_RUN_MISSING_PROVIDER"
    assert report["skills"] == []


def test_ambiguous_instruction_waits_for_clarification():
    """An ambiguous reference must ask, not guess -- in the real simulation too."""
    report = _run(_args(instruction="把那个搬过去。"))
    assert report["status"] == "WAITING_CLARIFICATION"


@pytest.mark.slow
def test_replay_planner_drives_real_mujoco_transport():
    """A recorded response sequence, replayed against real MuJoCo, completes the
    transport with a passing independent evaluation (rule and replay share the
    same executor + acceptance path, section 12)."""
    report = _run(_args(planner="replay"))
    assert report["planner_backend"] == "replay"
    assert report["clock_mode"] == "realtime"
    assert report["status"] == "SUCCEEDED"
    assert report["physical_outcome"] == "PASS"


@pytest.mark.slow
def test_stop_freeze_halts_real_mujoco_execution():
    """STOP must pre-empt the real MuJoCo loop: requesting a stop mid-task halts
    skill advancement and returns a stopped/cancelled outcome (section 13).

    Uses an auto-stop planner wrapper instead of a background thread, because
    MuJoCo's EGL rendering is not thread-safe.
    """
    from humanoid008.contracts import ExecutionBackend, TaskStatus
    from humanoid008.execution import (
        BudgetLedger,
        ControlToken,
        SkillExecutor,
        TaskManager,
        load_budget_limits,
    )
    from humanoid008.execution.planning_worker import PlanningWorker
    from humanoid008.language import RuleVocabulary
    from humanoid008.planners import RulePlanner
    from humanoid008.simulation.mujoco_world import MujocoWorld
    from humanoid008.skills.mujoco import build_mujoco_registry
    from humanoid008.supervision import (
        PreconditionSupervisor,
        ProposalValidator,
        RuntimeGuard,
        RuntimePolicy,
        SupervisionPolicy,
    )

    class AutoStopPlanner:
        name = "auto_stop"

        def __init__(self, inner, control, stop_after):
            self._inner = inner
            self._control = control
            self._stop_after = stop_after
            self._calls = 0

        def propose(self, *, goal, world, history, request_id):
            self._calls += 1
            if self._calls > self._stop_after:
                self._control.request_stop(reason="auto stop test")
            return self._inner.propose(
                goal=goal, world=world, history=history, request_id=request_id
            )

    world = MujocoWorld()
    try:
        registry = build_mujoco_registry(world)
        inner = RulePlanner(registry)
        control = ControlToken()
        planner = AutoStopPlanner(inner, control, stop_after=3)
        manager = TaskManager(
            runtime=world,
            registry=registry,
            planner=planner,
            executor=SkillExecutor(registry, max_ticks_per_skill=2400),
            vocabulary=RuleVocabulary.load(str(ROOT / "config" / "planners" / "rule.yaml")),
            execution_backend=ExecutionBackend.MUJOCO,
            validator=ProposalValidator(registry),
            precondition=PreconditionSupervisor(registry, SupervisionPolicy()),
            guard=RuntimeGuard(RuntimePolicy(), control),
            budgets=BudgetLedger(
                load_budget_limits(str(ROOT / "config" / "supervision" / "default.yaml"))
            ),
            control=control,
            planning_worker=PlanningWorker(planner),
        )
        outcome = manager.run_instruction("把箱子搬到 B 台。")
        assert outcome.task_status in (TaskStatus.STOPPED, TaskStatus.CANCELLED)
        assert len(outcome.history) < 8  # did not run the full 8-skill chain
    finally:
        world.close()
