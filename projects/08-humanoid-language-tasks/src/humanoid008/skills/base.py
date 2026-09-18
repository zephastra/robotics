"""Skill lifecycle (section 10).

``tick`` must be non-blocking: it never calls a planner or model, never sleeps,
and never runs a whole task by itself. All skills share one runtime, and the
executor guarantees that exactly one skill holds action-write authority at a
time.

A skill reports *evidence*, never a verdict: ``status``/``reason_code`` describe
what the skill observed, and the task manager decides what the task outcome is.
"""

from __future__ import annotations

from abc import ABC, abstractmethod
from typing import Any, Mapping, Sequence

from ..contracts import GoalSpec, ReasonCode, SkillName, SkillStatus, WorldState


class PreconditionError(Exception):
    """A skill cannot start. The message is safe to show to the user."""


class Skill(ABC):
    name: SkillName
    arg_names: tuple[str, ...] = ()

    def __init__(self) -> None:
        self._status = SkillStatus.PENDING
        self._reason = ReasonCode.OK

    # ------------------------------------------------------------------ #
    # lifecycle
    # ------------------------------------------------------------------ #

    @abstractmethod
    def check_preconditions(self, world: WorldState, args: Mapping[str, str]) -> None:
        """Raise :class:`PreconditionError` if this skill cannot start."""

    @abstractmethod
    def start(self, world: WorldState, args: Mapping[str, str]) -> None:
        """Begin the skill. Must not block."""

    @abstractmethod
    def tick(self, world: WorldState, dt_s: float) -> None:
        """Advance by ``dt_s``. Must not block, sleep or plan."""

    @abstractmethod
    def request_cancel(self, reason: str) -> None:
        """Stop as safely as the skill can, and record why."""

    @abstractmethod
    def world_delta(self) -> Mapping[str, Any]:
        """What this skill claims it changed. Recorded, not trusted blindly."""

    # ------------------------------------------------------------------ #
    # state
    # ------------------------------------------------------------------ #

    @property
    def status(self) -> SkillStatus:
        return self._status

    @property
    def reason_code(self) -> ReasonCode:
        return self._reason

    @property
    def is_finished(self) -> bool:
        return self._status in (
            SkillStatus.SUCCEEDED,
            SkillStatus.FAILED,
            SkillStatus.CANCELLED,
        )

    def _finish(self, status: SkillStatus, reason: ReasonCode) -> None:
        self._status = status
        self._reason = reason

    # ------------------------------------------------------------------ #
    # helpers
    # ------------------------------------------------------------------ #

    def arguments_for(self, goal: GoalSpec) -> dict[str, str]:
        """Default argument mapping, derived from the declared argument names."""
        return arguments_for(self.arg_names, goal)


def arguments_for(arg_names: Sequence[str], goal: GoalSpec) -> dict[str, str]:
    """Map declared argument names onto the goal.

    Single source of truth for how a goal becomes skill arguments, so a planner
    and a capability check can never disagree about them.
    """
    args: dict[str, str] = {}
    for name in arg_names:
        if name == "object_id":
            args[name] = goal.object_id
        elif name == "target_id":
            args[name] = goal.target_id or ""
    return args
