"""Layer 2 supervision: may this skill start right now? (section 13)

Checks, in order:

1. every referenced entity and station must exist and its observation must be fresh
2. the skill's own declared preconditions must hold
3. the attempt budget must not already be exhausted

Occupancy is deliberately **not** re-implemented here: the skill's own
preconditions already encode "UNKNOWN is not FREE", and duplicating the rule in
two places is how the two copies drift apart.

Policy values come from config, so neither a planner nor a model can relax a
threshold at runtime.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any, Mapping

import yaml

from ..contracts import (
    ActionProposal,
    EvidenceSource,
    GoalSpec,
    ProposalKind,
    ReasonCode,
    WorldState,
)
from ..skills.base import PreconditionError
from ..skills.registry import SkillRegistry
from .validator import OK, Verdict


@dataclass(frozen=True)
class SupervisionPolicy:
    max_observation_age_s: float = 5.0

    @classmethod
    def from_mapping(cls, data: Mapping[str, Any]) -> "SupervisionPolicy":
        observation = data.get("observation") or {}
        return cls(
            max_observation_age_s=float(observation.get("max_age_s", 5.0)),
        )

    @classmethod
    def load(cls, path: str | Path) -> "SupervisionPolicy":
        raw = yaml.safe_load(Path(path).read_text(encoding="utf-8")) or {}
        return cls.from_mapping(raw)


def _timestamp_arg_ids(proposal: ActionProposal) -> list[str]:
    return [
        value
        for name in ("object_id", "source_id")
        if (value := proposal.args.get(name))
    ]


class PreconditionSupervisor:
    def __init__(self, registry: SkillRegistry, policy: SupervisionPolicy) -> None:
        self._registry = registry
        self._policy = policy

    def check(
        self,
        proposal: ActionProposal,
        *,
        goal: GoalSpec,
        world: WorldState,
        now_s: float,
        budget_available: bool = True,
    ) -> Verdict:
        if proposal.kind is not ProposalKind.SKILL:
            return OK

        for entity_id in _timestamp_arg_ids(proposal):
            entity = world.entity(entity_id)
            if entity is None:
                return Verdict(
                    False, ReasonCode.UNKNOWN_ENTITY, f"'{entity_id}' is not in the world"
                )
            age = now_s - entity.observed_at_s
            if age > self._policy.max_observation_age_s:
                return Verdict(
                    False,
                    ReasonCode.STALE_OBSERVATION,
                    f"'{entity_id}' was last observed {age:.2f}s ago "
                    f"(limit {self._policy.max_observation_age_s:.2f}s)",
                )

        target_id = proposal.args.get("target_id")
        if target_id:
            station = world.station(target_id)
            if station is None:
                return Verdict(
                    False, ReasonCode.UNKNOWN_ENTITY, f"'{target_id}' is not in the world"
                )
            # A static fixture's POSITION is map knowledge and must not be
            # freshness-checked like a dynamic observation; only its occupancy
            # would need re-confirmation (the skill's own preconditions already
            # encode "UNKNOWN is not FREE").
            if station.source is not EvidenceSource.KNOWN_FIXTURE_MAP:
                age = now_s - station.observed_at_s
                if age > self._policy.max_observation_age_s:
                    return Verdict(
                        False,
                        ReasonCode.STALE_OBSERVATION,
                        f"'{target_id}' occupancy was last observed {age:.2f}s ago",
                    )

        skill = self._registry.new_skill(proposal.skill)
        try:
            skill.check_preconditions(world, proposal.args)
        except PreconditionError as exc:
            return Verdict(False, ReasonCode.PRECONDITION_FAILED, str(exc))

        if not budget_available:
            return Verdict(
                False, ReasonCode.BUDGET_EXHAUSTED, "the skill-attempt budget is exhausted"
            )

        return OK
