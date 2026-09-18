"""Strict JSON decoding and schema checks for the 008 contracts.

Rules from docs/MASTER_PLAN.md section 9.1:

- ``schema_version`` must be ``1.0``; unknown versions are rejected.
- unknown fields are rejected (no silent tolerance).
- duplicate JSON keys are rejected.
- ``NaN`` / ``Infinity`` are rejected.
- oversized and over-deep payloads are rejected.

Every failure raises :class:`ValidationError`. Callers must never swallow it and
continue: a rejected payload means "do not act".
"""

from __future__ import annotations

import json
import math
from enum import Enum
from typing import Any, Iterable, Mapping, Sequence, Type

from .enums import (
    EvidenceSource,
    Intent,
    ProposalKind,
    ReasonCode,
    SkillName,
    SkillStatus,
    StationStatus,
)

SCHEMA_VERSION = "1.0"
MAX_BYTES = 64 * 1024
MAX_DEPTH = 12
MAX_STRING = 4096


class ValidationError(ValueError):
    """A contract violation."""


# --------------------------------------------------------------------------- #
# strict JSON decoding
# --------------------------------------------------------------------------- #


def _reject_duplicate_keys(pairs: list[tuple[str, Any]]) -> dict:
    seen: dict[str, Any] = {}
    for key, value in pairs:
        if key in seen:
            raise ValidationError(f"duplicate key: {key!r}")
        seen[key] = value
    return seen


def _reject_constant(name: str) -> Any:
    raise ValidationError(f"non-finite number is not allowed: {name}")


def strict_json_loads(
    text: str,
    *,
    max_bytes: int = MAX_BYTES,
    max_depth: int = MAX_DEPTH,
) -> Any:
    """Parse JSON under the 008 rules. Raises ValidationError on any violation."""
    if not isinstance(text, str):
        raise ValidationError("payload must be a string")
    if len(text.encode("utf-8")) > max_bytes:
        raise ValidationError(f"payload exceeds {max_bytes} bytes")
    try:
        value = json.loads(
            text,
            object_pairs_hook=_reject_duplicate_keys,
            parse_constant=_reject_constant,
        )
    except ValidationError:
        raise
    except json.JSONDecodeError as exc:
        raise ValidationError(f"invalid JSON: {exc}") from exc
    _check_depth(value, max_depth)
    _check_finite(value)
    return value


def _check_depth(value: Any, max_depth: int, depth: int = 0) -> None:
    if depth > max_depth:
        raise ValidationError(f"payload exceeds nesting depth {max_depth}")
    if isinstance(value, Mapping):
        for item in value.values():
            _check_depth(item, max_depth, depth + 1)
    elif isinstance(value, Sequence) and not isinstance(value, (str, bytes)):
        for item in value:
            _check_depth(item, max_depth, depth + 1)


def _check_finite(value: Any) -> None:
    if isinstance(value, float) and not math.isfinite(value):
        raise ValidationError("non-finite number is not allowed")
    if isinstance(value, Mapping):
        for item in value.values():
            _check_finite(item)
    elif isinstance(value, Sequence) and not isinstance(value, (str, bytes)):
        for item in value:
            _check_finite(item)


# --------------------------------------------------------------------------- #
# primitive assertions
# --------------------------------------------------------------------------- #


def require_mapping(value: Any, what: str) -> Mapping[str, Any]:
    if not isinstance(value, Mapping):
        raise ValidationError(f"{what} must be an object")
    return value


def require_list(value: Any, what: str) -> list:
    if not isinstance(value, list):
        raise ValidationError(f"{what} must be an array")
    return value


def require_keys(
    data: Mapping[str, Any],
    *,
    required: Iterable[str],
    optional: Iterable[str] = (),
    what: str,
) -> None:
    """Reject unknown fields and report missing ones."""
    required_set = set(required)
    allowed = required_set | set(optional)
    unknown = sorted(set(data) - allowed)
    if unknown:
        raise ValidationError(f"{what}: unknown field(s) {unknown}")
    missing = sorted(required_set - set(data))
    if missing:
        raise ValidationError(f"{what}: missing field(s) {missing}")


def require_str(value: Any, what: str, *, allow_empty: bool = False) -> str:
    if not isinstance(value, str):
        raise ValidationError(f"{what} must be a string")
    if len(value) > MAX_STRING:
        raise ValidationError(f"{what} exceeds {MAX_STRING} characters")
    if not allow_empty and not value.strip():
        raise ValidationError(f"{what} must not be empty")
    return value


def require_bool(value: Any, what: str) -> bool:
    if not isinstance(value, bool):
        raise ValidationError(f"{what} must be a boolean")
    return value


def require_int(value: Any, what: str, *, minimum: int | None = None) -> int:
    if isinstance(value, bool) or not isinstance(value, int):
        raise ValidationError(f"{what} must be an integer")
    if minimum is not None and value < minimum:
        raise ValidationError(f"{what} must be >= {minimum}")
    return value


def require_number(value: Any, what: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValidationError(f"{what} must be a number")
    if not math.isfinite(float(value)):
        raise ValidationError(f"{what} must be finite")
    return float(value)


def require_enum(value: Any, enum_cls: Type[Enum], what: str) -> Enum:
    if not isinstance(value, str):
        raise ValidationError(f"{what} must be a string")
    try:
        return enum_cls(value)
    except ValueError as exc:
        allowed = [member.value for member in enum_cls]
        raise ValidationError(f"{what}: {value!r} is not one of {allowed}") from exc


def require_schema_version(data: Mapping[str, Any], what: str) -> str:
    version = require_str(data.get("schema_version"), f"{what}.schema_version")
    if version != SCHEMA_VERSION:
        raise ValidationError(
            f"{what}.schema_version: unsupported version {version!r} "
            f"(expected {SCHEMA_VERSION!r})"
        )
    return version


# --------------------------------------------------------------------------- #
# contract schemas
# --------------------------------------------------------------------------- #

_GOALSPEC_KEYS = (
    "schema_version",
    "task_id",
    "goal_revision",
    "intent",
    "object_id",
    "source_id",
    "target_id",
    "allowed_alternative_targets",
    "constraints",
)


def validate_goalspec(data: Any) -> Mapping[str, Any]:
    """Validate a GoalSpec.

    Section 25.2: ``inspect`` needs only ``object_id``; ``transport`` needs both
    ``object_id`` and ``target_id``. We never invent a destination for inspect.
    """
    data = require_mapping(data, "GoalSpec")
    require_keys(data, required=_GOALSPEC_KEYS, what="GoalSpec")
    require_schema_version(data, "GoalSpec")
    require_str(data["task_id"], "GoalSpec.task_id")
    require_int(data["goal_revision"], "GoalSpec.goal_revision", minimum=1)
    intent = require_enum(data["intent"], Intent, "GoalSpec.intent")

    object_id = data["object_id"]
    if object_id is not None:
        require_str(object_id, "GoalSpec.object_id")
    for name in ("source_id", "target_id"):
        if data[name] is not None:
            require_str(data[name], f"GoalSpec.{name}")

    alternatives = require_list(data["allowed_alternative_targets"], "GoalSpec.allowed_alternative_targets")
    for item in alternatives:
        require_str(item, "GoalSpec.allowed_alternative_targets[]")

    constraints = require_mapping(data["constraints"], "GoalSpec.constraints")
    require_keys(
        constraints,
        required=(),
        optional=("allow_retry", "allow_retarget"),
        what="GoalSpec.constraints",
    )
    for key, value in constraints.items():
        require_bool(value, f"GoalSpec.constraints.{key}")

    if object_id is None:
        raise ValidationError("GoalSpec.object_id is required")
    if intent.value == "transport" and data["target_id"] is None:
        raise ValidationError("GoalSpec.target_id is required for intent 'transport'")
    if intent.value == "inspect" and data["target_id"] is not None:
        raise ValidationError("GoalSpec.target_id must be null for intent 'inspect'")

    return data


_PROPOSAL_KEYS = (
    "schema_version",
    "request_id",
    "task_id",
    "goal_revision",
    "world_revision",
    "kind",
    "skill",
    "args",
    "reason",
    "question",
    "pending_fields",
)


def validate_action_proposal(data: Any) -> Mapping[str, Any]:
    """Validate an ActionProposal.

    Section 9.4: the three kinds use mutually exclusive field sets. A ``clarify``
    proposal may not carry an action.
    """
    data = require_mapping(data, "ActionProposal")
    require_keys(data, required=_PROPOSAL_KEYS, optional=("target_id",), what="ActionProposal")
    require_schema_version(data, "ActionProposal")
    require_str(data["request_id"], "ActionProposal.request_id")
    require_str(data["task_id"], "ActionProposal.task_id")
    require_int(data["goal_revision"], "ActionProposal.goal_revision", minimum=1)
    require_int(data["world_revision"], "ActionProposal.world_revision", minimum=0)
    kind = require_enum(data["kind"], ProposalKind, "ActionProposal.kind")
    require_str(data["reason"], "ActionProposal.reason")

    if kind is ProposalKind.SKILL:
        require_enum(data["skill"], SkillName, "ActionProposal.skill")
        args = require_mapping(data["args"], "ActionProposal.args")
        for key, value in args.items():
            require_str(key, "ActionProposal.args key")
            require_str(value, f"ActionProposal.args.{key}")
        if data["question"] is not None:
            raise ValidationError("ActionProposal.question is not allowed for kind 'skill'")
        if list(data["pending_fields"]):
            raise ValidationError("ActionProposal.pending_fields is not allowed for kind 'skill'")

    elif kind is ProposalKind.CLARIFY:
        require_str(data["question"], "ActionProposal.question")
        pending = require_list(data["pending_fields"], "ActionProposal.pending_fields")
        if not pending:
            raise ValidationError("ActionProposal.pending_fields must not be empty for 'clarify'")
        for item in pending:
            require_str(item, "ActionProposal.pending_fields[]")
        if data["skill"] is not None:
            raise ValidationError("ActionProposal.skill must be null for kind 'clarify'")
        if dict(data["args"]):
            raise ValidationError("ActionProposal.args must be empty for kind 'clarify'")

    elif kind is ProposalKind.RETARGET:
        target_id = data.get("target_id")
        if not target_id:
            raise ValidationError("ActionProposal.target_id is required for kind 'retarget'")
        require_str(target_id, "ActionProposal.target_id")
        if data["skill"] is not None:
            raise ValidationError("ActionProposal.skill must be null for kind 'retarget'")
        if dict(data["args"]):
            raise ValidationError("ActionProposal.args must be empty for kind 'retarget'")
        if data["question"] is not None:
            raise ValidationError("ActionProposal.question must be null for kind 'retarget'")

    else:  # CANNOT_COMPLETE
        if data["skill"] is not None:
            raise ValidationError("ActionProposal.skill must be null for kind 'cannot_complete'")
        if dict(data["args"]):
            raise ValidationError("ActionProposal.args must be empty for kind 'cannot_complete'")
        if data["question"] is not None:
            raise ValidationError("ActionProposal.question must be null for kind 'cannot_complete'")

    return data


_ENTITY_KEYS = ("entity_id", "category", "x", "y", "visible", "observed_at_s", "source")
_STATION_KEYS = ("station_id", "status", "source", "observed_at_s")
_ROBOT_KEYS = ("holding_object_id", "current_skill", "hand_contact_n")
_WORLD_KEYS = (
    "schema_version",
    "world_revision",
    "snapshot_seq",
    "sim_time_s",
    "robot",
    "entities",
    "stations",
    "capabilities",
)


def validate_world_state(data: Any) -> Mapping[str, Any]:
    data = require_mapping(data, "WorldState")
    require_keys(data, required=_WORLD_KEYS, what="WorldState")
    require_schema_version(data, "WorldState")
    require_int(data["world_revision"], "WorldState.world_revision", minimum=0)
    require_int(data["snapshot_seq"], "WorldState.snapshot_seq", minimum=0)
    require_number(data["sim_time_s"], "WorldState.sim_time_s")

    robot = require_mapping(data["robot"], "WorldState.robot")
    require_keys(robot, required=_ROBOT_KEYS, what="WorldState.robot")
    if robot["holding_object_id"] is not None:
        require_str(robot["holding_object_id"], "WorldState.robot.holding_object_id")
    if robot["current_skill"] is not None:
        require_enum(robot["current_skill"], SkillName, "WorldState.robot.current_skill")
    require_number(robot["hand_contact_n"], "WorldState.robot.hand_contact_n")

    entities = require_mapping(data["entities"], "WorldState.entities")
    for entity_id, entity in entities.items():
        require_str(entity_id, "WorldState.entities key")
        entity = require_mapping(entity, f"WorldState.entities[{entity_id}]")
        require_keys(entity, required=_ENTITY_KEYS, what=f"WorldState.entities[{entity_id}]")
        require_str(entity["entity_id"], "entity_id")
        require_str(entity["category"], "category")
        for axis in ("x", "y"):
            if entity[axis] is not None:
                require_number(entity[axis], f"entity.{axis}")
        require_bool(entity["visible"], "entity.visible")
        require_number(entity["observed_at_s"], "entity.observed_at_s")
        require_enum(entity["source"], EvidenceSource, f"entity[{entity_id}].source")

    stations = require_mapping(data["stations"], "WorldState.stations")
    for station_id, station in stations.items():
        require_str(station_id, "WorldState.stations key")
        station = require_mapping(station, f"WorldState.stations[{station_id}]")
        require_keys(station, required=_STATION_KEYS, what=f"WorldState.stations[{station_id}]")
        require_str(station["station_id"], "station_id")
        require_enum(station["status"], StationStatus, f"station[{station_id}].status")
        require_number(station["observed_at_s"], "station.observed_at_s")
        require_enum(station["source"], EvidenceSource, f"station[{station_id}].source")

    capabilities = require_mapping(data["capabilities"], "WorldState.capabilities")
    require_keys(
        capabilities,
        required=("available", "blocked"),
        what="WorldState.capabilities",
    )
    for item in require_list(capabilities["available"], "WorldState.capabilities.available"):
        require_enum(item, SkillName, "WorldState.capabilities.available[]")
    blocked = require_mapping(capabilities["blocked"], "WorldState.capabilities.blocked")
    for skill, reason in blocked.items():
        require_enum(skill, SkillName, "WorldState.capabilities.blocked key")
        require_str(reason, f"WorldState.capabilities.blocked[{skill}]")

    return data


_RESULT_KEYS = (
    "schema_version",
    "skill_run_id",
    "task_id",
    "skill",
    "status",
    "reason_code",
    "started_at_s",
    "finished_at_s",
    "world_delta",
)


def validate_skill_result(data: Any) -> Mapping[str, Any]:
    data = require_mapping(data, "SkillResult")
    require_keys(data, required=_RESULT_KEYS, what="SkillResult")
    require_schema_version(data, "SkillResult")
    require_str(data["skill_run_id"], "SkillResult.skill_run_id")
    require_str(data["task_id"], "SkillResult.task_id")
    require_enum(data["skill"], SkillName, "SkillResult.skill")
    require_enum(data["status"], SkillStatus, "SkillResult.status")
    require_enum(data["reason_code"], ReasonCode, "SkillResult.reason_code")
    require_number(data["started_at_s"], "SkillResult.started_at_s")
    require_number(data["finished_at_s"], "SkillResult.finished_at_s")
    require_mapping(data["world_delta"], "SkillResult.world_delta")
    return data
