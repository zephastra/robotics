"""The rule planner -- the deterministic control group.

Section 12: it decides the next skill from the GoalSpec, the world state and what
has already succeeded. Its behaviour is the *reference*: any other planner must
return the same structure and be judged against this one, sharing the same
validation, execution and acceptance path.

Declared limitation (P2): authorised alternative targets are parsed, validated and
carried in the GoalSpec, but this planner does **not** fall back to them yet. An
occupied primary destination is reported as a failure rather than silently
swapped -- substituting a station without authorisation is exactly the behaviour
section 1.2 forbids.
"""

from __future__ import annotations

from typing import Mapping, Sequence

from ..contracts import (
    ActionProposal,
    GoalSpec,
    Intent,
    SkillName,
    SkillResult,
    SkillStatus,
    StationStatus,
    WorldState,
)
from ..skills.registry import SkillRegistry
from ..world import occupancy
from .base import Planner

_INSPECT_SEQUENCE: tuple[SkillName, ...] = (SkillName.OBSERVE_OBJECT,)

_TRANSPORT_SEQUENCE: tuple[SkillName, ...] = (
    SkillName.OBSERVE_OBJECT,
    SkillName.OBSERVE_TARGET,
    SkillName.APPROACH_OBJECT,
    SkillName.GRASP_OBJECT,
    SkillName.LIFT_OBJECT,
    SkillName.CARRY_TO_TARGET,
    SkillName.PLACE_OBJECT,
    SkillName.VERIFY_RESULT,
)

_REASONS: Mapping[SkillName, str] = {
    SkillName.OBSERVE_OBJECT: "need a fresh observation of the object first",
    SkillName.OBSERVE_TARGET: "need a fresh observation of the destination",
    SkillName.APPROACH_OBJECT: "object observed; moving to a workable stance",
    SkillName.GRASP_OBJECT: "in stance; establishing contact",
    SkillName.LIFT_OBJECT: "contact established; lifting clear of the source",
    SkillName.CARRY_TO_TARGET: "object held; walking to the destination",
    SkillName.PLACE_OBJECT: "at the destination; releasing into the target",
    SkillName.VERIFY_RESULT: "placement done; verifying the result independently",
}


class RulePlanner(Planner):
    name = "rule"

    def __init__(self, registry: SkillRegistry) -> None:
        self._registry = registry

    def propose(
        self,
        *,
        goal: GoalSpec,
        world: WorldState,
        history: Sequence[SkillResult],
        request_id: str,
    ) -> ActionProposal | None:
        sequence = (
            _INSPECT_SEQUENCE if goal.intent is Intent.INSPECT else _TRANSPORT_SEQUENCE
        )
        completed = {
            result.skill for result in history if result.status is SkillStatus.SUCCEEDED
        }

        for skill in sequence:
            if skill in completed:
                continue

            # Occupancy is only knowable after the destination has been observed,
            # so the refusal happens where a real system would notice it.
            if skill is SkillName.APPROACH_OBJECT and goal.intent is Intent.TRANSPORT:
                station = world.station(goal.target_id or "")
                if not occupancy.is_available_target(station):
                    alternative = self._authorised_alternative(goal, world)
                    if alternative is not None:
                        return ActionProposal.retarget(
                            request_id=request_id,
                            task_id=goal.task_id,
                            goal_revision=goal.goal_revision,
                            world_revision=world.world_revision,
                            target_id=alternative,
                            reason=(
                                f"destination {occupancy.describe(station)}; "
                                f"retargeting to authorised alternative {alternative}"
                            ),
                        )
                    return ActionProposal.cannot_complete(
                        request_id=request_id,
                        task_id=goal.task_id,
                        goal_revision=goal.goal_revision,
                        world_revision=world.world_revision,
                        reason=(
                            f"destination {occupancy.describe(station)}; refusing to "
                            "substitute another station without authorisation"
                        ),
                    )

            blocked = world.capabilities.blocked.get(skill.value)
            if blocked:
                return ActionProposal.cannot_complete(
                    request_id=request_id,
                    task_id=goal.task_id,
                    goal_revision=goal.goal_revision,
                    world_revision=world.world_revision,
                    reason=f"{skill.value} cannot start: {blocked}",
                )

            return ActionProposal.for_skill(
                request_id=request_id,
                task_id=goal.task_id,
                goal_revision=goal.goal_revision,
                world_revision=world.world_revision,
                skill=skill,
                reason=_REASONS.get(skill, ""),
                args=self._registry.arguments_for(skill, goal),
            )

        return None

    @staticmethod
    def _authorised_alternative(goal: GoalSpec, world: WorldState) -> str | None:
        """The first authorised alternative target that is not known-occupied.

        Section 15: swapping is allowed only to a user-authorised alternative,
        never on the planner's own initiative. An UNKNOWN alternative is also
        returned -- the retarget re-runs OBSERVE_TARGET, which observes it and
        re-checks occupancy before any placement.
        """
        for candidate in goal.allowed_alternative_targets:
            station = world.station(candidate)
            if station is None:
                continue
            if station.status is StationStatus.OCCUPIED:
                continue
            return candidate
        return None
