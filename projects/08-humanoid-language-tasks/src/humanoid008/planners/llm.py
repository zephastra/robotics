"""Real-model planner (section 12).

Sends a **bounded** request through the provider-agnostic bridge and accepts only
a strict :class:`ActionProposal`.

Deliberate properties:

- the request carries the task, a bounded entity summary, the available skills,
  the recent results and the budgets -- never the repository, environment
  variables, secrets or arbitrary files;
- **no world coordinates are sent**: the model may only reference registered
  entity ids (section 9.1), so it cannot invent a position;
- the prompt is not a safety mechanism: the response goes through exactly the
  same contract validation and supervision as any other planner's output;
- there is **no fallback**. If no provider is configured this planner raises, and
  the run is recorded as ``NOT_RUN_MISSING_PROVIDER``.
"""

from __future__ import annotations

from typing import Any, Mapping, Sequence

from ..contracts import ActionProposal, GoalSpec, SkillResult, WorldState
from ..skills.registry import SkillRegistry
from .base import Planner
from .http_bridge import HttpPlannerBridge, MissingProviderError

RECENT_RESULTS_SENT = 5


def _bounded_world_summary(world: WorldState) -> dict[str, Any]:
    """A summary the model may see. No coordinates, no ground truth."""
    now = world.sim_time_s
    return {
        "world_revision": world.world_revision,
        "sim_time_s": now,
        "robot": {
            "holding_object_id": world.robot.holding_object_id,
            "current_skill": (
                world.robot.current_skill.value if world.robot.current_skill else None
            ),
        },
        "entities": [
            {
                "entity_id": entity.entity_id,
                "category": entity.category,
                "visible": entity.visible,
                "observed_age_s": round(now - entity.observed_at_s, 3),
                "source": entity.source.value,
            }
            for entity in world.entities.values()
        ],
        "stations": [
            {
                "station_id": station.station_id,
                "status": station.status.value,
                "observed_age_s": round(now - station.observed_at_s, 3),
                "source": station.source.value,
            }
            for station in world.stations.values()
        ],
        "capabilities": world.capabilities.to_dict(),
    }


class LlmPlanner(Planner):
    name = "llm"

    def __init__(self, bridge: HttpPlannerBridge, registry: SkillRegistry) -> None:
        self._bridge = bridge
        self._registry = registry

    @property
    def available(self) -> bool:
        return self._bridge.available

    def build_request(
        self,
        *,
        goal: GoalSpec,
        world: WorldState,
        history: Sequence[SkillResult],
        request_id: str,
    ) -> dict[str, Any]:
        return {
            "request_id": request_id,
            "goal": goal.to_dict(),
            "world": _bounded_world_summary(world),
            "skills": [
                {"name": name.value, "args": list(self._registry.argument_names(name))}
                for name in self._registry.names()
            ],
            "recent_results": [
                result.to_dict() for result in history[-RECENT_RESULTS_SENT:]
            ],
        }

    def propose(
        self,
        *,
        goal: GoalSpec,
        world: WorldState,
        history: Sequence[SkillResult],
        request_id: str,
    ) -> ActionProposal:
        if not self._bridge.available:
            raise MissingProviderError(
                "planner backend 'llm' was requested but no endpoint is configured"
            )
        payload = self.build_request(
            goal=goal, world=world, history=history, request_id=request_id
        )
        response = self._bridge.plan(payload)
        # Strict: an invalid response raises, and the caller applies its bounded
        # repair / failure policy. It is never silently replaced by a rule plan.
        return ActionProposal.from_dict(response)
