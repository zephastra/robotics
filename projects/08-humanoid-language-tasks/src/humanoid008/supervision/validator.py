"""Layer 1 supervision: structural checks on a proposal (section 13).

A proposal is **rejected, never repaired**. This layer answers, in order:

1. does it satisfy the contract schema at all (field whitelisting, finite values,
   size, mutually exclusive field sets)?
2. does it refer to this task and the goal revision currently in force?
3. if it proposes a skill, is that skill registered, are the arguments exactly the
   declared ones, and do the referenced entities exist in the world?

Rejections are recorded as their own category. This layer is *not* a safety
guarantee and must never be described as one.
"""

from __future__ import annotations

from dataclasses import dataclass

from ..contracts import (
    ActionProposal,
    GoalSpec,
    ProposalKind,
    ReasonCode,
    ValidationError,
    WorldState,
)
from ..skills.registry import SkillRegistry

ENTITY_ARGUMENTS = ("object_id", "target_id", "source_id")


@dataclass(frozen=True)
class Verdict:
    ok: bool
    reason_code: ReasonCode = ReasonCode.OK
    detail: str = ""


OK = Verdict(True)


class ProposalValidator:
    def __init__(self, registry: SkillRegistry) -> None:
        self._registry = registry

    def check(
        self, proposal: ActionProposal, *, goal: GoalSpec, world: WorldState
    ) -> Verdict:
        # 2. identity and revision, before anything expensive
        if proposal.task_id != goal.task_id:
            return Verdict(
                False,
                ReasonCode.PRECONDITION_FAILED,
                f"proposal belongs to task {proposal.task_id!r}, not {goal.task_id!r}",
            )
        if proposal.goal_revision != goal.goal_revision:
            return Verdict(
                False,
                ReasonCode.REVISION_MISMATCH,
                f"proposal uses goal_revision {proposal.goal_revision}, "
                f"current is {goal.goal_revision}",
            )

        # 1. schema
        try:
            proposal.to_dict()
        except ValidationError as exc:
            return Verdict(False, ReasonCode.PRECONDITION_FAILED, f"schema: {exc}")

        if proposal.kind is not ProposalKind.SKILL:
            # clarify / cannot_complete carry no action, so nothing else to check
            return OK

        # 3. skill and arguments
        if proposal.skill not in self._registry:
            return Verdict(
                False,
                ReasonCode.PRECONDITION_FAILED,
                f"skill {proposal.skill} is not registered",
            )
        declared = set(self._registry.argument_names(proposal.skill))
        supplied = set(proposal.args)
        if supplied != declared:
            return Verdict(
                False,
                ReasonCode.PRECONDITION_FAILED,
                f"arguments {sorted(supplied)} do not match declared {sorted(declared)}",
            )

        for name in ENTITY_ARGUMENTS:
            value = proposal.args.get(name)
            if not value:
                continue
            if name == "target_id":
                exists = world.station(value) is not None
            else:
                exists = world.entity(value) is not None
            if not exists:
                return Verdict(
                    False,
                    ReasonCode.UNKNOWN_ENTITY,
                    f"argument {name}={value!r} is not present in the world",
                )

        return OK
