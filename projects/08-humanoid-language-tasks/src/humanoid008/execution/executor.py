"""Single-skill executor (sections 8, 10, 13).

Exactly one skill may hold action-write authority at a time. The executor runs a
skill to completion, records a :class:`SkillResult`, and refuses to start
anything while another skill is active.

The tick interval is explicit, so a fake clock and a real clock share one code
path instead of diverging.
"""

from __future__ import annotations

import uuid
from typing import Any, Callable, Mapping

from ..contracts import (
    ReasonCode,
    SkillName,
    SkillResult,
    SkillStatus,
    WorldState,
)
from ..skills.base import PreconditionError
from ..skills.registry import SkillRegistry


class ExecutionError(RuntimeError):
    """The executor was asked to do something it must refuse."""


class SkillExecutor:
    def __init__(
        self,
        registry: SkillRegistry,
        *,
        dt_s: float = 0.05,
        max_ticks_per_skill: int = 500,
    ) -> None:
        if dt_s <= 0.0:
            raise ValueError("dt_s must be positive")
        self._registry = registry
        self._dt = dt_s
        self._max_ticks = max_ticks_per_skill
        self._active: SkillName | None = None

    @property
    def active_skill(self) -> SkillName | None:
        """The skill currently holding action-write authority, if any."""
        return self._active

    def run(
        self,
        *,
        skill_name: SkillName,
        args: Mapping[str, str],
        world: WorldState,
        task_id: str,
        world_provider: Callable[[], WorldState] | None = None,
        control: Any = None,
    ) -> SkillResult:
        """Run one skill to completion.

        ``control`` is a duck-typed stop token (see
        :mod:`humanoid008.execution.cancellation`): if it has ``stop_requested``
        set while the skill is ticking, the skill is cancelled and a
        ``CANCELLED`` result is returned. ``None`` disables the check.
        """
        if self._active is not None:
            raise ExecutionError(
                f"{self._active.value} still holds action-write authority"
            )
        if skill_name not in self._registry:
            raise ExecutionError(f"skill {skill_name} is not registered")

        skill = self._registry.new_skill(skill_name)
        run_id = uuid.uuid4().hex[:12]
        started_at = world.sim_time_s
        self._active = skill_name
        try:
            try:
                skill.check_preconditions(world, args)
            except PreconditionError as exc:
                return self._build(
                    run_id, task_id, skill_name, SkillStatus.FAILED,
                    ReasonCode.PRECONDITION_FAILED, started_at, world,
                    {"precondition": str(exc)},
                )

            skill.start(world, args)
            ticks = 0
            while not skill.is_finished:
                if control is not None and control.stop_requested:
                    skill.request_cancel(str(control.reason))
                    break
                current = world_provider() if world_provider else world
                skill.tick(current, self._dt)
                ticks += 1
                if ticks > self._max_ticks:
                    skill.request_cancel("per-skill tick budget exhausted")
                    # Stage 0.2: merge the skill's own delta instead of dropping
                    # it. Diagnostics used to be discarded on exactly the path
                    # (budget exhaustion) where they matter most.
                    delta = {"ticks": ticks}
                    try:
                        delta.update(skill.world_delta() or {})
                    except Exception:  # noqa: BLE001
                        pass
                    return self._build(
                        run_id, task_id, skill_name, SkillStatus.FAILED,
                        ReasonCode.BUDGET_EXHAUSTED, started_at, current,
                        delta,
                    )

            latest = world_provider() if world_provider else world
            return self._build(
                run_id, task_id, skill_name, skill.status, skill.reason_code,
                started_at, latest, skill.world_delta(),
            )
        finally:
            self._active = None

    @staticmethod
    def _build(
        run_id: str,
        task_id: str,
        skill_name: SkillName,
        status: SkillStatus,
        reason: ReasonCode,
        started_at: float,
        world: WorldState,
        delta: Mapping[str, Any] | None,
    ) -> SkillResult:
        return SkillResult(
            skill_run_id=run_id,
            task_id=task_id,
            skill=skill_name,
            status=status,
            reason_code=reason,
            started_at_s=started_at,
            finished_at_s=world.sim_time_s,
            world_delta=dict(delta or {}),
        )
