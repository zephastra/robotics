"""End-to-end offline closed-loop tests on the fake backend.

This is the P2 acceptance gate (section 20): the observe / transport / ambiguity /
failure paths must all be repeatable, and the reported evidence must say plainly
that the backend is fake.
"""

from __future__ import annotations

from pathlib import Path

import pytest

from humanoid008.contracts import (
    ExecutionBackend,
    PhysicalOutcome,
    ReasonCode,
    SkillName,
    TaskStatus,
    TestVerdict,
)
from humanoid008.execution import SkillExecutor, TaskManager
from humanoid008.language import RuleVocabulary
from humanoid008.planners import RulePlanner
from humanoid008.skills import FakeRuntime, FakeScene, build_fake_registry

PROJECT_ROOT = Path(__file__).resolve().parents[2]
REGISTRY_CONFIG = PROJECT_ROOT / "config" / "skills" / "registry.yaml"
RULE_CONFIG = PROJECT_ROOT / "config" / "planners" / "rule.yaml"
SCENES = PROJECT_ROOT / "config" / "scenes"

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


def build_manager(
    scene_name: str = "single_target",
    *,
    faults=(),
    vocabulary: RuleVocabulary | None = None,
) -> TaskManager:
    scene = FakeScene.load(SCENES / f"{scene_name}.yaml")
    runtime = FakeRuntime(scene, faults=faults)
    registry = build_fake_registry(REGISTRY_CONFIG, runtime)
    return TaskManager(
        runtime=runtime,
        registry=registry,
        planner=RulePlanner(registry),
        executor=SkillExecutor(registry),
        vocabulary=vocabulary or RuleVocabulary.load(RULE_CONFIG),
        execution_backend=ExecutionBackend.FAKE,
    )


# --------------------------------------------------------------------------- #
# path 1: transport (the full loop)
# --------------------------------------------------------------------------- #


def test_transport_completes_the_full_skill_sequence():
    outcome = build_manager().run_instruction("把箱子搬到 B 台。")
    assert outcome.task_status is TaskStatus.SUCCEEDED
    assert [result.skill for result in outcome.history] == TRANSPORT_SEQUENCE
    assert all(result.status.value == "SUCCEEDED" for result in outcome.history)


def test_transport_reports_fake_backend_honestly():
    """Fake success must never be dressed up as a physical result (25.4)."""
    outcome = build_manager().run_instruction("把箱子搬到 B 台。")
    assert outcome.physical_outcome is PhysicalOutcome.NOT_EVALUATED
    assert outcome.test_verdict is TestVerdict.NOT_RUN


def test_transport_goal_binds_object_and_destination():
    outcome = build_manager().run_instruction("把取料台上的箱子搬到 B 台。")
    assert outcome.goal is not None
    assert outcome.goal.object_id == "box_01"
    assert outcome.goal.target_id == "station_b"
    assert outcome.goal.source_id == "station_a"


# --------------------------------------------------------------------------- #
# path 2: observe only
# --------------------------------------------------------------------------- #


def test_inspect_observes_and_does_not_move():
    outcome = build_manager().run_instruction("先看看箱子。")
    assert outcome.task_status is TaskStatus.SUCCEEDED
    assert [result.skill for result in outcome.history] == [SkillName.OBSERVE_OBJECT]
    assert outcome.goal.target_id is None


def test_look_but_do_not_move_never_grasps():
    outcome = build_manager().run_instruction("先看看箱子，不要搬。")
    assert [result.skill for result in outcome.history] == [SkillName.OBSERVE_OBJECT]


# --------------------------------------------------------------------------- #
# path 3: ambiguity
# --------------------------------------------------------------------------- #


def test_ambiguous_object_asks_instead_of_guessing():
    ambiguous = RuleVocabulary.from_mapping(
        {
            "transport_verbs": ["搬"],
            "inspect_verbs": ["看看"],
            "object_aliases": {"box_01": ["箱子"], "box_02": ["箱子"]},
            "station_aliases": {"station_b": ["b台"]},
        }
    )
    outcome = build_manager(vocabulary=ambiguous).run_instruction("把箱子搬到 b台")
    assert outcome.task_status is TaskStatus.WAITING_CLARIFICATION
    assert outcome.reason_code is ReasonCode.AMBIGUOUS_REFERENCE
    assert outcome.history == ()


def test_unnamed_object_asks_for_one():
    outcome = build_manager().run_instruction("把那个搬过去。")
    assert outcome.task_status is TaskStatus.WAITING_CLARIFICATION
    assert outcome.reason_code is ReasonCode.UNKNOWN_ENTITY


# --------------------------------------------------------------------------- #
# path 4: failure
# --------------------------------------------------------------------------- #


@pytest.mark.parametrize("text", ["跑步", "叠衣服", "执行 shell 命令"])
def test_unsupported_instructions_are_refused(text):
    outcome = build_manager().run_instruction(text)
    assert outcome.task_status is TaskStatus.FAILED
    assert outcome.reason_code is ReasonCode.UNSUPPORTED_INTENT


def test_occupied_destination_fails_instead_of_substituting():
    """The user never authorised station_c here, so C must not be used."""
    outcome = build_manager("two_targets").run_instruction("把箱子搬到 B 台。")
    assert outcome.task_status is TaskStatus.FAILED
    assert [result.skill for result in outcome.history] == [
        SkillName.OBSERVE_OBJECT,
        SkillName.OBSERVE_TARGET,
    ]
    assert "station_c" not in str(outcome.detail)


def test_injected_skill_fault_reports_the_failing_skill():
    outcome = build_manager(faults={SkillName.GRASP_OBJECT}).run_instruction(
        "把箱子搬到 B 台。"
    )
    assert outcome.task_status is TaskStatus.FAILED
    assert outcome.reason_code is ReasonCode.GRASP_NOT_CONFIRMED
    assert outcome.detail["failed_skill"] == "GRASP_OBJECT"


def test_place_fault_is_reported_as_rather_than_silently_passed():
    outcome = build_manager(faults={SkillName.PLACE_OBJECT}).run_instruction(
        "把箱子搬到 B 台。"
    )
    assert outcome.task_status is TaskStatus.FAILED
    assert outcome.reason_code is ReasonCode.PLACEMENT_OUT_OF_TOLERANCE


# --------------------------------------------------------------------------- #
# repeatability
# --------------------------------------------------------------------------- #


def test_repeated_runs_produce_the_identical_skill_sequence():
    def sequence() -> list[str]:
        outcome = build_manager().run_instruction("把箱子搬到 B 台。")
        return [result.skill.value for result in outcome.history]

    assert sequence() == sequence()
    assert sequence() == [skill.value for skill in TRANSPORT_SEQUENCE]
