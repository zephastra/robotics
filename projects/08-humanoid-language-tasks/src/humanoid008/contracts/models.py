"""Typed value objects for the 008 contracts.

Every ``from_dict`` goes through :mod:`humanoid008.contracts.validation`, so an
invalid payload can never become a usable object. Every ``to_dict`` also
validates before returning, which makes it impossible to *emit* a malformed
contract either. The validation cost is irrelevant at P2 volumes and buys a
strong invariant.
"""

from __future__ import annotations

from dataclasses import dataclass, field, replace
from typing import Any, Mapping

from . import validation as v
from .enums import (
    EvidenceSource,
    Intent,
    ProposalKind,
    ReasonCode,
    SkillName,
    SkillStatus,
    StationStatus,
)

SCHEMA_VERSION = v.SCHEMA_VERSION


# --------------------------------------------------------------------------- #
# goal
# --------------------------------------------------------------------------- #


@dataclass(frozen=True)
class GoalSpec:
    """The trusted, validated statement of what the user wants (section 9.2)."""

    task_id: str
    goal_revision: int
    intent: Intent
    object_id: str
    source_id: str | None = None
    target_id: str | None = None
    allowed_alternative_targets: tuple[str, ...] = ()
    constraints: Mapping[str, bool] = field(default_factory=dict)

    def to_dict(self) -> dict:
        data = {
            "schema_version": SCHEMA_VERSION,
            "task_id": self.task_id,
            "goal_revision": self.goal_revision,
            "intent": self.intent.value,
            "object_id": self.object_id,
            "source_id": self.source_id,
            "target_id": self.target_id,
            "allowed_alternative_targets": list(self.allowed_alternative_targets),
            "constraints": dict(self.constraints),
        }
        v.validate_goalspec(data)
        return data

    @classmethod
    def from_dict(cls, data: Any) -> "GoalSpec":
        v.validate_goalspec(data)
        return cls(
            task_id=data["task_id"],
            goal_revision=data["goal_revision"],
            intent=Intent(data["intent"]),
            object_id=data["object_id"],
            source_id=data["source_id"],
            target_id=data["target_id"],
            allowed_alternative_targets=tuple(data["allowed_alternative_targets"]),
            constraints=dict(data["constraints"]),
        )

    def next_revision(self) -> "GoalSpec":
        """A new goal revision. Old responses must be discarded (section 14)."""
        return replace(self, goal_revision=self.goal_revision + 1)


# --------------------------------------------------------------------------- #
# world
# --------------------------------------------------------------------------- #


@dataclass(frozen=True)
class EntityObservation:
    entity_id: str
    category: str
    x: float | None
    y: float | None
    visible: bool
    observed_at_s: float
    source: EvidenceSource

    def to_dict(self) -> dict:
        return {
            "entity_id": self.entity_id,
            "category": self.category,
            "x": self.x,
            "y": self.y,
            "visible": self.visible,
            "observed_at_s": self.observed_at_s,
            "source": self.source.value,
        }

    @classmethod
    def from_dict(cls, data: Mapping[str, Any]) -> "EntityObservation":
        return cls(
            entity_id=data["entity_id"],
            category=data["category"],
            x=data["x"],
            y=data["y"],
            visible=data["visible"],
            observed_at_s=data["observed_at_s"],
            source=EvidenceSource(data["source"]),
        )


@dataclass(frozen=True)
class StationState:
    station_id: str
    status: StationStatus
    source: EvidenceSource
    observed_at_s: float

    def to_dict(self) -> dict:
        return {
            "station_id": self.station_id,
            "status": self.status.value,
            "source": self.source.value,
            "observed_at_s": self.observed_at_s,
        }

    @classmethod
    def from_dict(cls, data: Mapping[str, Any]) -> "StationState":
        return cls(
            station_id=data["station_id"],
            status=StationStatus(data["status"]),
            source=EvidenceSource(data["source"]),
            observed_at_s=data["observed_at_s"],
        )


@dataclass(frozen=True)
class RobotState:
    holding_object_id: str | None
    current_skill: SkillName | None
    hand_contact_n: float

    def to_dict(self) -> dict:
        return {
            "holding_object_id": self.holding_object_id,
            "current_skill": self.current_skill.value if self.current_skill else None,
            "hand_contact_n": self.hand_contact_n,
        }

    @classmethod
    def from_dict(cls, data: Mapping[str, Any]) -> "RobotState":
        skill = data["current_skill"]
        return cls(
            holding_object_id=data["holding_object_id"],
            current_skill=SkillName(skill) if skill else None,
            hand_contact_n=data["hand_contact_n"],
        )


@dataclass(frozen=True)
class CapabilityState:
    available: tuple[SkillName, ...]
    blocked: Mapping[str, str] = field(default_factory=dict)

    def to_dict(self) -> dict:
        return {
            "available": [skill.value for skill in self.available],
            "blocked": dict(self.blocked),
        }

    @classmethod
    def from_dict(cls, data: Mapping[str, Any]) -> "CapabilityState":
        return cls(
            available=tuple(SkillName(name) for name in data["available"]),
            blocked=dict(data["blocked"]),
        )


@dataclass(frozen=True)
class WorldState:
    world_revision: int
    snapshot_seq: int
    sim_time_s: float
    robot: RobotState
    entities: Mapping[str, EntityObservation]
    stations: Mapping[str, StationState]
    capabilities: CapabilityState

    def to_dict(self) -> dict:
        data = {
            "schema_version": SCHEMA_VERSION,
            "world_revision": self.world_revision,
            "snapshot_seq": self.snapshot_seq,
            "sim_time_s": self.sim_time_s,
            "robot": self.robot.to_dict(),
            "entities": {k: e.to_dict() for k, e in self.entities.items()},
            "stations": {k: s.to_dict() for k, s in self.stations.items()},
            "capabilities": self.capabilities.to_dict(),
        }
        v.validate_world_state(data)
        return data

    @classmethod
    def from_dict(cls, data: Any) -> "WorldState":
        v.validate_world_state(data)
        return cls(
            world_revision=data["world_revision"],
            snapshot_seq=data["snapshot_seq"],
            sim_time_s=data["sim_time_s"],
            robot=RobotState.from_dict(data["robot"]),
            entities={
                key: EntityObservation.from_dict(value)
                for key, value in data["entities"].items()
            },
            stations={
                key: StationState.from_dict(value)
                for key, value in data["stations"].items()
            },
            capabilities=CapabilityState.from_dict(data["capabilities"]),
        )

    def entity(self, entity_id: str) -> EntityObservation | None:
        return self.entities.get(entity_id)

    def station(self, station_id: str) -> StationState | None:
        return self.stations.get(station_id)


# --------------------------------------------------------------------------- #
# planning
# --------------------------------------------------------------------------- #


@dataclass(frozen=True)
class ActionProposal:
    """The only thing a planner may return: one skill, a question, or a refusal."""

    request_id: str
    task_id: str
    goal_revision: int
    world_revision: int
    kind: ProposalKind
    reason: str
    skill: SkillName | None = None
    args: Mapping[str, str] = field(default_factory=dict)
    question: str | None = None
    pending_fields: tuple[str, ...] = ()
    target_id: str | None = None

    def to_dict(self) -> dict:
        # Accept a raw string for kind/skill so that malformed construction
        # surfaces as a ValidationError below, not an AttributeError.
        kind = self.kind.value if isinstance(self.kind, ProposalKind) else self.kind
        skill = self.skill.value if isinstance(self.skill, SkillName) else self.skill
        data = {
            "schema_version": SCHEMA_VERSION,
            "request_id": self.request_id,
            "task_id": self.task_id,
            "goal_revision": self.goal_revision,
            "world_revision": self.world_revision,
            "kind": kind,
            "skill": skill,
            "args": dict(self.args),
            "reason": self.reason,
            "question": self.question,
            "pending_fields": list(self.pending_fields),
            "target_id": self.target_id,
        }
        v.validate_action_proposal(data)
        return data

    @classmethod
    def from_dict(cls, data: Any) -> "ActionProposal":
        v.validate_action_proposal(data)
        skill = data["skill"]
        return cls(
            request_id=data["request_id"],
            task_id=data["task_id"],
            goal_revision=data["goal_revision"],
            world_revision=data["world_revision"],
            kind=ProposalKind(data["kind"]),
            reason=data["reason"],
            skill=SkillName(skill) if skill else None,
            args=dict(data["args"]),
            question=data["question"],
            pending_fields=tuple(data["pending_fields"]),
            target_id=data.get("target_id"),
        )

    @classmethod
    def for_skill(
        cls,
        *,
        request_id: str,
        task_id: str,
        goal_revision: int,
        world_revision: int,
        skill: SkillName,
        reason: str,
        args: Mapping[str, str] | None = None,
    ) -> "ActionProposal":
        return cls(
            request_id=request_id,
            task_id=task_id,
            goal_revision=goal_revision,
            world_revision=world_revision,
            kind=ProposalKind.SKILL,
            reason=reason,
            skill=skill,
            args=dict(args or {}),
        )

    @classmethod
    def clarify(
        cls,
        *,
        request_id: str,
        task_id: str,
        goal_revision: int,
        world_revision: int,
        question: str,
        pending_fields: tuple[str, ...],
        reason: str,
    ) -> "ActionProposal":
        return cls(
            request_id=request_id,
            task_id=task_id,
            goal_revision=goal_revision,
            world_revision=world_revision,
            kind=ProposalKind.CLARIFY,
            reason=reason,
            question=question,
            pending_fields=pending_fields,
        )

    @classmethod
    def cannot_complete(
        cls,
        *,
        request_id: str,
        task_id: str,
        goal_revision: int,
        world_revision: int,
        reason: str,
    ) -> "ActionProposal":
        return cls(
            request_id=request_id,
            task_id=task_id,
            goal_revision=goal_revision,
            world_revision=world_revision,
            kind=ProposalKind.CANNOT_COMPLETE,
            reason=reason,
        )

    @classmethod
    def retarget(
        cls,
        *,
        request_id: str,
        task_id: str,
        goal_revision: int,
        world_revision: int,
        target_id: str,
        reason: str,
    ) -> "ActionProposal":
        """Propose switching to an authorised alternative destination (section 15)."""
        return cls(
            request_id=request_id,
            task_id=task_id,
            goal_revision=goal_revision,
            world_revision=world_revision,
            kind=ProposalKind.RETARGET,
            reason=reason,
            target_id=target_id,
        )


# --------------------------------------------------------------------------- #
# results
# --------------------------------------------------------------------------- #


@dataclass(frozen=True)
class SkillResult:
    skill_run_id: str
    task_id: str
    skill: SkillName
    status: SkillStatus
    reason_code: ReasonCode
    started_at_s: float
    finished_at_s: float
    world_delta: Mapping[str, Any] = field(default_factory=dict)

    def to_dict(self) -> dict:
        data = {
            "schema_version": SCHEMA_VERSION,
            "skill_run_id": self.skill_run_id,
            "task_id": self.task_id,
            "skill": self.skill.value,
            "status": self.status.value,
            "reason_code": self.reason_code.value,
            "started_at_s": self.started_at_s,
            "finished_at_s": self.finished_at_s,
            "world_delta": dict(self.world_delta),
        }
        v.validate_skill_result(data)
        return data

    @classmethod
    def from_dict(cls, data: Any) -> "SkillResult":
        v.validate_skill_result(data)
        return cls(
            skill_run_id=data["skill_run_id"],
            task_id=data["task_id"],
            skill=SkillName(data["skill"]),
            status=SkillStatus(data["status"]),
            reason_code=ReasonCode(data["reason_code"]),
            started_at_s=data["started_at_s"],
            finished_at_s=data["finished_at_s"],
            world_delta=dict(data["world_delta"]),
        )

    @property
    def succeeded(self) -> bool:
        return self.status is SkillStatus.SUCCEEDED
