"""Enumerations shared by every layer.

The *values* here are the wire strings used in JSON and in reports. They must
stay stable: reports and tests compare against them.
"""

from __future__ import annotations

from enum import Enum


class Intent(str, Enum):
    """What the user asked for. Deliberately tiny in V1 (section 9.2)."""

    INSPECT = "inspect"
    TRANSPORT = "transport"


class SkillName(str, Enum):
    """The only skills that may ever be proposed (section 10)."""

    OBSERVE_OBJECT = "OBSERVE_OBJECT"
    OBSERVE_TARGET = "OBSERVE_TARGET"
    APPROACH_OBJECT = "APPROACH_OBJECT"
    GRASP_OBJECT = "GRASP_OBJECT"
    LIFT_OBJECT = "LIFT_OBJECT"
    CARRY_TO_TARGET = "CARRY_TO_TARGET"
    PLACE_OBJECT = "PLACE_OBJECT"
    VERIFY_RESULT = "VERIFY_RESULT"


class SkillStatus(str, Enum):
    PENDING = "PENDING"
    RUNNING = "RUNNING"
    SUCCEEDED = "SUCCEEDED"
    FAILED = "FAILED"
    CANCELLED = "CANCELLED"


class TaskStatus(str, Enum):
    IDLE = "IDLE"
    PLANNING = "PLANNING"
    WAITING_CLARIFICATION = "WAITING_CLARIFICATION"
    EXECUTING = "EXECUTING"
    REPLANNING = "REPLANNING"
    SUCCEEDED = "SUCCEEDED"
    FAILED = "FAILED"
    CANCELLED = "CANCELLED"
    STOPPED = "STOPPED"


class ProposalKind(str, Enum):
    SKILL = "skill"
    CLARIFY = "clarify"
    CANNOT_COMPLETE = "cannot_complete"
    # Section 15: the primary destination is occupied but the user authorised an
    # alternative -- the planner proposes a retarget, the task manager applies it.
    RETARGET = "retarget"


class StationStatus(str, Enum):
    FREE = "FREE"
    OCCUPIED = "OCCUPIED"
    UNKNOWN = "UNKNOWN"


class EvidenceSource(str, Enum):
    """Where an observation came from (section 9.3).

    A user assertion must never be treated as visual evidence, and ``FAKE_WORLD``
    exists only so the P2 offline framework can label itself honestly.
    ``PROPRIOCEPTION`` is the robot's own body estimate (grasp sites / joint
    state), which is *not* visual evidence and must be labelled as such.
    """

    RGBD_MARKER = "rgbd_marker"
    KNOWN_FIXTURE_MAP = "known_fixture_map"
    PROPRIOCEPTION = "proprioception"
    TEST_INJECTION = "test_injection"
    USER_ASSERTION = "user_assertion"
    FAKE_WORLD = "fake_world"


class ReasonCode(str, Enum):
    OK = "OK"
    # language / grounding
    AMBIGUOUS_REFERENCE = "AMBIGUOUS_REFERENCE"
    UNKNOWN_ENTITY = "UNKNOWN_ENTITY"
    UNSUPPORTED_INTENT = "UNSUPPORTED_INTENT"
    # world
    TARGET_NOT_VISIBLE = "TARGET_NOT_VISIBLE"
    STALE_OBSERVATION = "STALE_OBSERVATION"
    TARGET_OCCUPIED = "TARGET_OCCUPIED"
    # execution
    GRASP_NOT_CONFIRMED = "GRASP_NOT_CONFIRMED"
    CONTACT_LOST = "CONTACT_LOST"
    PLACEMENT_OUT_OF_TOLERANCE = "PLACEMENT_OUT_OF_TOLERANCE"
    PRECONDITION_FAILED = "PRECONDITION_FAILED"
    NO_FEASIBLE_PLAN = "NO_FEASIBLE_PLAN"
    BASE_DRIFT = "BASE_DRIFT"
    # goal completion (a planner returning None is NOT, by itself, task success)
    GOAL_NOT_SATISFIED = "GOAL_NOT_SATISFIED"
    # control
    BUDGET_EXHAUSTED = "BUDGET_EXHAUSTED"
    CANCELLED = "CANCELLED"
    NOT_IMPLEMENTED = "NOT_IMPLEMENTED"
    # model interface / revision (P3)
    REVISION_MISMATCH = "REVISION_MISMATCH"
    MODEL_TIMEOUT = "MODEL_TIMEOUT"
    INVALID_OUTPUT = "INVALID_OUTPUT"
    NOT_RUN_MISSING_PROVIDER = "NOT_RUN_MISSING_PROVIDER"


class ExecutionBackend(str, Enum):
    FAKE = "fake"
    MUJOCO = "mujoco"


class PlannerBackend(str, Enum):
    RULE = "rule"
    REPLAY = "replay"
    LLM = "llm"


class ClockMode(str, Enum):
    FAKE_CLOCK = "fake_clock"
    PLANNING_PAUSE = "planning_pause"
    REALTIME = "realtime"


class PhysicalOutcome(str, Enum):
    """Kept separate from task status on purpose (section 25.4)."""

    PASS = "PASS"
    FAIL = "FAIL"
    NOT_APPLICABLE = "NOT_APPLICABLE"
    NOT_EVALUATED = "NOT_EVALUATED"


class TestVerdict(str, Enum):
    """Whether the *test case* passed. Kept separate from task status (25.4).

    ``__test__`` is False so pytest does not try to collect this enum as a test
    class (names beginning with "Test" are otherwise treated as test classes).
    """

    __test__ = False

    PASS = "PASS"
    FAIL = "FAIL"
    NOT_RUN = "NOT_RUN"
