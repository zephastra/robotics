"""Planner interface.

A planner is the *only* component allowed to choose what happens next, and it may
only ever return one :class:`ActionProposal` -- or ``None`` to report that the
goal is already satisfied. A planner cannot execute anything, cannot touch the
simulator, and cannot change budgets or thresholds.

Every planner (rule / replay / llm) must share the same validation, execution and
acceptance path: the supervisor gives no planner a looser bound than another
(section 8).
"""

from __future__ import annotations

from abc import ABC, abstractmethod
from typing import Sequence

from ..contracts import ActionProposal, GoalSpec, SkillResult, WorldState


class Planner(ABC):
    """Base class for all planners."""

    name: str = "base"

    @abstractmethod
    def propose(
        self,
        *,
        goal: GoalSpec,
        world: WorldState,
        history: Sequence[SkillResult],
        request_id: str,
    ) -> ActionProposal | None:
        """Return the next proposal, or ``None`` when the goal is satisfied.

        ``history`` contains the finished :class:`SkillResult` objects for this
        task, oldest first.
        """
        raise NotImplementedError
