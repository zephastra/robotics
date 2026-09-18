"""P3 unit tests: supervision layers, budgets and cancellation (section 13).

These test the building blocks in isolation, without any model or physics.
"""

from __future__ import annotations

from pathlib import Path

import pytest

from humanoid008.contracts import (
    ActionProposal,
    GoalSpec,
    Intent,
    ProposalKind,
    ReasonCode,
    SkillName,
    WorldState,
)
from humanoid008.execution import BudgetExhausted, BudgetLedger, ControlToken, StopKind
from humanoid008.skills import FakeRuntime, FakeScene, build_fake_registry
from humanoid008.skills.registry import SkillRegistry
from humanoid008.supervision import (
    PreconditionSupervisor,
    ProposalValidator,
    RuntimeGuard,
    RuntimePolicy,
    SupervisionPolicy,
)

PROJECT_ROOT = Path(__file__).resolve().parents[2]
REGISTRY_CONFIG = PROJECT_ROOT / "config" / "skills" / "registry.yaml"
SCENES = PROJECT_ROOT / "config" / "scenes"


def _registry() -> SkillRegistry:
    scene = FakeScene.load(SCENES / "single_target.yaml")
    return build_fake_registry(REGISTRY_CONFIG, FakeRuntime(scene))


def _goal() -> GoalSpec:
    return GoalSpec(
        task_id="task-0001",
        goal_revision=1,
        intent=Intent.TRANSPORT,
        object_id="box_01",
        source_id="station_a",
        target_id="station_b",
    )


def _world(**overrides) -> WorldState:
    data = {
        "schema_version": "1.0",
        "world_revision": 3,
        "snapshot_seq": 11,
        "sim_time_s": 10.0,
        "robot": {"holding_object_id": None, "current_skill": None, "hand_contact_n": 0.0},
        "entities": {
            "box_01": {
                "entity_id": "box_01",
                "category": "box",
                "x": 1.0,
                "y": 2.0,
                "visible": True,
                "observed_at_s": 9.9,
                "source": "fake_world",
            }
        },
        "stations": {
            "station_b": {
                "station_id": "station_b",
                "status": "FREE",
                "source": "fake_world",
                "observed_at_s": 9.9,
            }
        },
        "capabilities": {"available": ["OBSERVE_OBJECT"], "blocked": {}},
    }
    data.update(overrides)
    return WorldState.from_dict(data)


def _proposal(skill=SkillName.APPROACH_OBJECT, args=None, **overrides):
    kw = dict(request_id="req-1", task_id="task-0001", goal_revision=1, world_revision=3)
    kw.update(overrides)
    return ActionProposal.for_skill(
        **kw, skill=skill, reason="r", args=args or {"object_id": "box_01"}
    )


# --------------------------------------------------------------------------- #
# layer 1: ProposalValidator
# --------------------------------------------------------------------------- #


def test_validator_accepts_a_well_formed_skill_proposal():
    verdict = ProposalValidator(_registry()).check(
        _proposal(), goal=_goal(), world=_world()
    )
    assert verdict.ok


def test_validator_rejects_a_proposal_for_a_different_task():
    verdict = ProposalValidator(_registry()).check(
        _proposal(task_id="task-9999"), goal=_goal(), world=_world()
    )
    assert not verdict.ok
    assert verdict.reason_code is ReasonCode.PRECONDITION_FAILED


def test_validator_rejects_a_stale_goal_revision():
    verdict = ProposalValidator(_registry()).check(
        _proposal(goal_revision=7), goal=_goal(), world=_world()
    )
    assert not verdict.ok
    assert verdict.reason_code is ReasonCode.REVISION_MISMATCH


def test_validator_rejects_wrong_arguments():
    verdict = ProposalValidator(_registry()).check(
        _proposal(args={"wrong_arg": "x"}), goal=_goal(), world=_world()
    )
    assert not verdict.ok
    assert "arguments" in verdict.detail


def test_validator_rejects_an_unknown_entity_argument():
    verdict = ProposalValidator(_registry()).check(
        _proposal(args={"object_id": "box_99"}), goal=_goal(), world=_world()
    )
    assert not verdict.ok
    assert verdict.reason_code is ReasonCode.UNKNOWN_ENTITY


def test_validator_rejects_an_unregistered_skill():
    # A registry that knows only OBSERVE_OBJECT must refuse GRASP_OBJECT.
    full = _registry()
    declared = {SkillName.OBSERVE_OBJECT: full.argument_names(SkillName.OBSERVE_OBJECT)}
    narrow = SkillRegistry({name: (lambda: None) for name in declared}, declared)
    verdict = ProposalValidator(narrow).check(
        _proposal(skill=SkillName.GRASP_OBJECT), goal=_goal(), world=_world()
    )
    assert not verdict.ok
    assert "not registered" in verdict.detail


def test_validator_passes_non_action_kinds_without_further_checks():
    validator = ProposalValidator(_registry())
    clarify = ActionProposal.clarify(
        request_id="req-2",
        task_id="task-0001",
        goal_revision=1,
        world_revision=3,
        question="which box?",
        pending_fields=("object_id",),
        reason="ambiguous",
    )
    assert validator.check(clarify, goal=_goal(), world=_world()).ok

    refuse = ActionProposal.cannot_complete(
        request_id="req-3",
        task_id="task-0001",
        goal_revision=1,
        world_revision=3,
        reason="no plan",
    )
    assert validator.check(refuse, goal=_goal(), world=_world()).ok


# --------------------------------------------------------------------------- #
# layer 2: PreconditionSupervisor
# --------------------------------------------------------------------------- #


def test_precondition_flags_a_stale_object_observation():
    # object observed at 4.0, world time is 10.0 -> age 6.0 > 5.0 default.
    world = _world(
        entities={
            "box_01": {
                "entity_id": "box_01",
                "category": "box",
                "x": 1.0,
                "y": 2.0,
                "visible": True,
                "observed_at_s": 4.0,
                "source": "fake_world",
            }
        }
    )
    supervisor = PreconditionSupervisor(_registry(), SupervisionPolicy())
    verdict = supervisor.check(
        _proposal(), goal=_goal(), world=world, now_s=world.sim_time_s
    )
    assert not verdict.ok
    assert verdict.reason_code is ReasonCode.STALE_OBSERVATION


def test_precondition_flags_a_stale_station_observation():
    world = _world(
        stations={
            "station_b": {
                "station_id": "station_b",
                "status": "FREE",
                "source": "fake_world",
                "observed_at_s": 4.0,
            }
        }
    )
    supervisor = PreconditionSupervisor(_registry(), SupervisionPolicy())
    verdict = supervisor.check(
        _proposal(
            skill=SkillName.CARRY_TO_TARGET, args={"target_id": "station_b"}
        ),
        goal=_goal(),
        world=world,
        now_s=world.sim_time_s,
    )
    assert not verdict.ok
    assert verdict.reason_code is ReasonCode.STALE_OBSERVATION


def test_precondition_passes_a_fresh_proposal():
    supervisor = PreconditionSupervisor(_registry(), SupervisionPolicy())
    verdict = supervisor.check(
        _proposal(), goal=_goal(), world=_world(), now_s=_world().sim_time_s
    )
    assert verdict.ok


# --------------------------------------------------------------------------- #
# layer 3: RuntimeGuard
# --------------------------------------------------------------------------- #


def test_runtime_guard_stop_wins_immediately():
    control = ControlToken()
    guard = RuntimeGuard(RuntimePolicy(), control)
    control.request_stop("test stop", now_s=1.0)
    verdict = guard.check(now_s=1.001)
    assert not verdict.ok
    assert verdict.reason_code is ReasonCode.CANCELLED


def test_runtime_guard_flags_an_expired_command():
    guard = RuntimeGuard(RuntimePolicy(command_ttl_s=0.5), ControlToken())
    verdict = guard.check(now_s=2.0, command_issued_at_s=1.0)
    assert not verdict.ok
    assert "expired" in verdict.detail


def test_runtime_guard_flags_excessive_tilt():
    guard = RuntimeGuard(RuntimePolicy(max_tilt_rad=1.0), ControlToken())
    verdict = guard.check(now_s=1.0, tilt_rad=1.7)
    assert not verdict.ok
    assert "tilt" in verdict.detail


def test_runtime_guard_passes_when_idle():
    guard = RuntimeGuard(RuntimePolicy(), ControlToken())
    assert guard.check(now_s=1.0).ok


# --------------------------------------------------------------------------- #
# BudgetLedger
# --------------------------------------------------------------------------- #


def test_budget_spends_and_tracks_usage():
    ledger = BudgetLedger({"model_calls_per_task": 3})
    assert ledger.can_spend("model_calls_per_task")
    ledger.spend("model_calls_per_task")
    ledger.spend("model_calls_per_task")
    assert ledger.remaining("model_calls_per_task") == 1
    assert not ledger.can_spend("model_calls_per_task", amount=2)


def test_budget_exhaustion_raises():
    ledger = BudgetLedger({"retries_per_skill": 1})
    ledger.spend("retries_per_skill")
    with pytest.raises(BudgetExhausted):
        ledger.spend("retries_per_skill")


def test_budget_has_no_reset_method():
    """Section 13: budgets must not be resettable by replanning."""
    ledger = BudgetLedger({"replans_per_task": 3})
    assert not hasattr(ledger, "reset")


def test_budget_rejects_unknown_key():
    ledger = BudgetLedger({"replans_per_task": 3})
    with pytest.raises(KeyError):
        ledger.spend("does_not_exist")


# --------------------------------------------------------------------------- #
# ControlToken
# --------------------------------------------------------------------------- #


def test_stop_is_never_downgraded_by_cancel():
    control = ControlToken()
    control.request_stop("stop", now_s=1.0)
    control.request_cancel("cancel", now_s=2.0)
    assert control.kind is StopKind.STOP


def test_cancel_then_stop_yields_stop():
    control = ControlToken()
    control.request_cancel("cancel", now_s=1.0)
    control.request_stop("stop", now_s=2.0)
    assert control.kind is StopKind.STOP


def test_stop_latency_is_measured():
    control = ControlToken()
    control.request_stop("stop", now_s=10.0)
    control.mark_handled(10.05)
    assert control.latency_s == pytest.approx(0.05)
