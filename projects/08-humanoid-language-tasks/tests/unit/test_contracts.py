"""Contract tests: strict JSON, GoalSpec, ActionProposal, WorldState, SkillResult.

Covers items 1 and 2 of docs/MASTER_PLAN.md section 17.1.
"""

from __future__ import annotations

import pytest

from humanoid008 import contracts
from humanoid008.contracts import validation as v
from humanoid008.contracts.enums import Intent, ProposalKind, SkillName


# --------------------------------------------------------------------------- #
# strict JSON decoding (section 9.1)
# --------------------------------------------------------------------------- #


def test_strict_json_accepts_normal_payload():
    assert v.strict_json_loads('{"a": [1, 2]}') == {"a": [1, 2]}


def test_strict_json_rejects_duplicate_keys():
    with pytest.raises(v.ValidationError, match="duplicate key"):
        v.strict_json_loads('{"a": 1, "a": 2}')


def test_strict_json_rejects_nan():
    with pytest.raises(v.ValidationError):
        v.strict_json_loads('{"a": NaN}')


def test_strict_json_rejects_infinity():
    with pytest.raises(v.ValidationError):
        v.strict_json_loads('{"a": Infinity}')


def test_strict_json_rejects_oversized_payload():
    with pytest.raises(v.ValidationError, match="exceeds"):
        v.strict_json_loads('{"a": "' + "x" * 100 + '"}', max_bytes=16)


def test_strict_json_rejects_excessive_depth():
    payload = "1"
    for _ in range(20):
        payload = "[" + payload + "]"
    with pytest.raises(v.ValidationError, match="nesting depth"):
        v.strict_json_loads(payload)


def test_strict_json_rejects_malformed_json():
    with pytest.raises(v.ValidationError, match="invalid JSON"):
        v.strict_json_loads("{not json")


# --------------------------------------------------------------------------- #
# GoalSpec (sections 9.2 and 25.2)
# --------------------------------------------------------------------------- #


def goalspec_dict(**overrides) -> dict:
    base = {
        "schema_version": "1.0",
        "task_id": "task-0001",
        "goal_revision": 1,
        "intent": "transport",
        "object_id": "box_01",
        "source_id": "station_a",
        "target_id": "station_b",
        "allowed_alternative_targets": [],
        "constraints": {"allow_retry": True, "allow_retarget": False},
    }
    base.update(overrides)
    return base


def test_goalspec_transport_round_trip():
    goal = contracts.GoalSpec.from_dict(goalspec_dict())
    assert goal.intent is Intent.TRANSPORT
    assert goal.to_dict() == goalspec_dict()


def test_goalspec_inspect_needs_no_target():
    goal = contracts.GoalSpec.from_dict(
        goalspec_dict(intent="inspect", target_id=None)
    )
    assert goal.intent is Intent.INSPECT
    assert goal.target_id is None


def test_goalspec_transport_without_target_is_rejected():
    with pytest.raises(v.ValidationError, match="target_id is required"):
        contracts.GoalSpec.from_dict(goalspec_dict(target_id=None))


def test_goalspec_inspect_with_target_is_rejected():
    """We must not bolt a destination onto an inspect goal (section 25.2)."""
    with pytest.raises(v.ValidationError, match="must be null"):
        contracts.GoalSpec.from_dict(goalspec_dict(intent="inspect"))


def test_goalspec_rejects_unknown_field():
    with pytest.raises(v.ValidationError, match="unknown field"):
        contracts.GoalSpec.from_dict(goalspec_dict(extra="nope"))


def test_goalspec_rejects_unknown_schema_version():
    with pytest.raises(v.ValidationError, match="unsupported version"):
        contracts.GoalSpec.from_dict(goalspec_dict(schema_version="2.0"))


def test_goalspec_rejects_unknown_constraint():
    with pytest.raises(v.ValidationError, match="unknown field"):
        contracts.GoalSpec.from_dict(goalspec_dict(constraints={"ignore_rules": True}))


def test_goalspec_next_revision_increments():
    goal = contracts.GoalSpec.from_dict(goalspec_dict())
    assert goal.next_revision().goal_revision == 2


# --------------------------------------------------------------------------- #
# ActionProposal (section 9.4)
# --------------------------------------------------------------------------- #


def proposal_kwargs(**overrides) -> dict:
    base = {
        "request_id": "req-0001",
        "task_id": "task-0001",
        "goal_revision": 1,
        "world_revision": 5,
    }
    base.update(overrides)
    return base


def test_skill_proposal_round_trip():
    proposal = contracts.ActionProposal.for_skill(
        **proposal_kwargs(),
        skill=SkillName.APPROACH_OBJECT,
        reason="need to get close first",
        args={"object_id": "box_01"},
    )
    assert proposal.kind is ProposalKind.SKILL
    assert contracts.ActionProposal.from_dict(proposal.to_dict()) == proposal


def test_clarify_proposal_requires_pending_fields():
    with pytest.raises(v.ValidationError, match="pending_fields"):
        contracts.ActionProposal.clarify(
            **proposal_kwargs(),
            question="which box?",
            pending_fields=(),
            reason="ambiguous",
        ).to_dict()


def test_clarify_proposal_may_not_carry_a_skill():
    data = contracts.ActionProposal.clarify(
        **proposal_kwargs(),
        question="which box?",
        pending_fields=("object_id",),
        reason="ambiguous",
    ).to_dict()
    data["skill"] = "GRASP_OBJECT"
    with pytest.raises(v.ValidationError, match="must be null"):
        contracts.ActionProposal.from_dict(data)


def test_skill_proposal_may_not_carry_a_question():
    with pytest.raises(v.ValidationError, match="question is not allowed"):
        contracts.ActionProposal(
            **proposal_kwargs(),
            kind=ProposalKind.SKILL,
            reason="r",
            skill=SkillName.GRASP_OBJECT,
            question="why?",
        ).to_dict()


def test_proposal_rejects_unknown_skill():
    """An unknown skill arriving over the JSON boundary must be rejected."""
    data = contracts.ActionProposal.for_skill(
        **proposal_kwargs(), skill=SkillName.GRASP_OBJECT, reason="r"
    ).to_dict()
    data["skill"] = "FLY_TO_MOON"
    with pytest.raises(v.ValidationError, match="not one of"):
        contracts.ActionProposal.from_dict(data)


def test_proposal_rejects_unknown_field():
    data = contracts.ActionProposal.for_skill(
        **proposal_kwargs(), skill=SkillName.GRASP_OBJECT, reason="r"
    ).to_dict()
    data["code"] = "import os"
    with pytest.raises(v.ValidationError, match="unknown field"):
        contracts.ActionProposal.from_dict(data)


# --------------------------------------------------------------------------- #
# WorldState and SkillResult
# --------------------------------------------------------------------------- #


def worldstate_dict() -> dict:
    return {
        "schema_version": "1.0",
        "world_revision": 3,
        "snapshot_seq": 11,
        "sim_time_s": 4.5,
        "robot": {"holding_object_id": None, "current_skill": None, "hand_contact_n": 0.0},
        "entities": {
            "box_01": {
                "entity_id": "box_01",
                "category": "box",
                "x": 1.0,
                "y": 2.0,
                "visible": True,
                "observed_at_s": 4.4,
                "source": "fake_world",
            }
        },
        "stations": {
            "station_b": {
                "station_id": "station_b",
                "status": "FREE",
                "source": "fake_world",
                "observed_at_s": 4.4,
            }
        },
        "capabilities": {"available": ["OBSERVE_OBJECT"], "blocked": {}},
    }


def test_worldstate_round_trip():
    world = contracts.WorldState.from_dict(worldstate_dict())
    assert world.world_revision == 3
    assert world.entity("box_01") is not None
    assert world.station("station_b").status.value == "FREE"
    assert world.to_dict() == worldstate_dict()


def test_worldstate_rejects_bad_station_status():
    data = worldstate_dict()
    data["stations"]["station_b"]["status"] = "PROBABLY_FREE"
    with pytest.raises(v.ValidationError, match="not one of"):
        contracts.WorldState.from_dict(data)


def test_worldstate_rejects_user_assertion_source_typo():
    data = worldstate_dict()
    data["entities"]["box_01"]["source"] = "user_says_so"
    with pytest.raises(v.ValidationError, match="not one of"):
        contracts.WorldState.from_dict(data)


def test_skill_result_round_trip():
    result = contracts.SkillResult(
        skill_run_id="run-1",
        task_id="task-0001",
        skill=SkillName.OBSERVE_OBJECT,
        status=contracts.SkillStatus.SUCCEEDED,
        reason_code=contracts.ReasonCode.OK,
        started_at_s=0.0,
        finished_at_s=0.5,
    )
    assert result.succeeded
    assert contracts.SkillResult.from_dict(result.to_dict()) == result


def test_skill_result_rejects_unknown_status():
    data = contracts.SkillResult(
        skill_run_id="run-1",
        task_id="task-0001",
        skill=SkillName.OBSERVE_OBJECT,
        status=contracts.SkillStatus.SUCCEEDED,
        reason_code=contracts.ReasonCode.OK,
        started_at_s=0.0,
        finished_at_s=0.5,
    ).to_dict()
    data["status"] = "TOTALLY_DONE"
    with pytest.raises(v.ValidationError, match="not one of"):
        contracts.SkillResult.from_dict(data)
