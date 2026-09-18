"""008 data contracts: enumerations, typed value objects and strict validation.

This package depends on nothing else in ``humanoid008`` (section 5).
"""

from .enums import (
    ClockMode,
    EvidenceSource,
    ExecutionBackend,
    Intent,
    PhysicalOutcome,
    PlannerBackend,
    ProposalKind,
    ReasonCode,
    SkillName,
    SkillStatus,
    StationStatus,
    TaskStatus,
    TestVerdict,
)
from .models import (
    ActionProposal,
    CapabilityState,
    EntityObservation,
    GoalSpec,
    RobotState,
    SkillResult,
    StationState,
    WorldState,
)
from .validation import (
    SCHEMA_VERSION,
    ValidationError,
    strict_json_loads,
    validate_action_proposal,
    validate_goalspec,
    validate_skill_result,
    validate_world_state,
)

__all__ = [
    # enums
    "ClockMode",
    "EvidenceSource",
    "ExecutionBackend",
    "Intent",
    "PhysicalOutcome",
    "PlannerBackend",
    "ProposalKind",
    "ReasonCode",
    "SkillName",
    "SkillStatus",
    "StationStatus",
    "TaskStatus",
    "TestVerdict",
    # models
    "ActionProposal",
    "CapabilityState",
    "EntityObservation",
    "GoalSpec",
    "RobotState",
    "SkillResult",
    "StationState",
    "WorldState",
    # validation
    "SCHEMA_VERSION",
    "ValidationError",
    "strict_json_loads",
    "validate_action_proposal",
    "validate_goalspec",
    "validate_skill_result",
    "validate_world_state",
]
