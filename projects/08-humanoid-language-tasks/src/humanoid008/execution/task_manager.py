"""Task lifecycle: the one place that turns language into trusted action.

Section 25.1: parsing only produces a *draft*. This class performs validation,
entity binding and authorisation checks, and only then creates the trusted
GoalSpec. A draft is never executed directly.

The three status axes stay separate (section 25.4):

- ``task_status``      -- did the task logic finish, and how
- ``physical_outcome`` -- did the real world actually change (fake => NOT_EVALUATED)
- ``test_verdict``     -- did the test case pass (a plain demo run has no expectation)

They must never be collapsed into one "success" flag.

P3 adds the supervision pipeline around the loop (section 13):

1. structural validation of every proposal (layer 1),
2. preconditions -- freshness, declared preconditions, budget (layer 2),
3. a run-time guard -- stop, command expiry, body faults (layer 3),
4. budgets that replanning cannot reset,
5. stop/cancel handling, including a stop that pre-empts a hung model.

Planning runs off-loop (section 13) so the control loop never blocks on a model
and STOP always wins.
"""

from __future__ import annotations

import time
import uuid
from dataclasses import dataclass
from typing import Any, Callable, Mapping, Sequence

from ..contracts import (
    ActionProposal,
    ExecutionBackend,
    GoalSpec,
    Intent,
    PhysicalOutcome,
    ProposalKind,
    ReasonCode,
    SkillName,
    SkillResult,
    SkillStatus,
    TaskStatus,
    TestVerdict,
    ValidationError,
)
from ..language import ParseKind, RuleVocabulary, parse
from ..planners.base import Planner
from ..planners.http_bridge import MissingProviderError, PlannerTransportError
from ..skills.registry import SkillRegistry
from ..supervision import (
    PreconditionSupervisor,
    ProposalValidator,
    RuntimeGuard,
    RuntimePolicy,
    SupervisionPolicy,
)
from ..world import GroundingError, resolve_object_id, resolve_station_id
from .budgets import BudgetLedger
from .cancellation import ControlToken, StopKind
from .executor import SkillExecutor
from .planning_worker import PlanningWorker

DEFAULT_BUDGET_LIMITS: Mapping[str, int] = {
    "max_steps_per_task": 32,
    "model_calls_per_task": 20,
    "replans_per_task": 3,
    "retries_per_skill": 2,
    "stance_realign_per_task": 2,
}

# The skill whose SUCCEEDED status is *necessary* for the goal to count as done.
# A planner returning None is only "task complete" if this terminal skill really
# succeeded (section 25.4: skill success != task success; and an empty / silent
# planner must not be mistaken for a completed task).
_TERMINAL_SKILL: Mapping[Intent, SkillName] = {
    Intent.TRANSPORT: SkillName.VERIFY_RESULT,
    Intent.INSPECT: SkillName.OBSERVE_OBJECT,
}


@dataclass(frozen=True)
class TaskOutcome:
    task_status: TaskStatus
    physical_outcome: PhysicalOutcome
    test_verdict: TestVerdict
    reason_code: ReasonCode
    goal: GoalSpec | None
    history: tuple[SkillResult, ...]
    detail: Mapping[str, Any]


class TaskManager:
    def __init__(
        self,
        *,
        runtime,
        registry: SkillRegistry,
        planner: Planner,
        executor: SkillExecutor,
        vocabulary: RuleVocabulary,
        execution_backend: ExecutionBackend = ExecutionBackend.FAKE,
        events=None,
        validator: ProposalValidator | None = None,
        precondition: PreconditionSupervisor | None = None,
        guard: RuntimeGuard | None = None,
        budgets: BudgetLedger | None = None,
        control: ControlToken | None = None,
        now_s: Callable[[], float] = time.monotonic,
        planning_worker: PlanningWorker | None = None,
    ) -> None:
        self._runtime = runtime
        self._registry = registry
        self._planner = planner
        self._executor = executor
        self._vocabulary = vocabulary
        self._backend = execution_backend
        self._events = events
        self._now_s = now_s

        # P3 supervision pipeline (section 13). Each layer is independently
        # injectable so tests can exercise them in isolation.
        self._validator = validator or ProposalValidator(registry)
        self._precondition = precondition or PreconditionSupervisor(
            registry, SupervisionPolicy()
        )
        self._control = control or ControlToken()
        self._guard = guard or RuntimeGuard(RuntimePolicy(), self._control)
        self._budgets = budgets or BudgetLedger(DEFAULT_BUDGET_LIMITS)
        self._planning = planning_worker or PlanningWorker(planner)

    # ------------------------------------------------------------------ #
    # logging
    # ------------------------------------------------------------------ #

    def _record(self, event: str, **fields: Any) -> None:
        if self._events is not None:
            self._events.record(event, **fields)

    # ------------------------------------------------------------------ #
    # entry point
    # ------------------------------------------------------------------ #

    def run_instruction(self, instruction: str, *, task_id: str | None = None) -> TaskOutcome:
        task_id = task_id or f"task-{uuid.uuid4().hex[:8]}"
        self._record("instruction_received", task_id=task_id, instruction=instruction)

        parsed = parse(instruction, self._vocabulary)
        self._record(
            "instruction_parsed",
            task_id=task_id,
            kind=parsed.kind.value,
            normalized=parsed.normalized,
            reason_code=parsed.reason_code.value,
            detail=parsed.detail,
        )

        if parsed.kind is ParseKind.REJECT:
            self._record("instruction_refused", task_id=task_id, detail=parsed.detail)
            return self._outcome(
                TaskStatus.FAILED,
                ReasonCode.UNSUPPORTED_INTENT,
                None,
                (),
                {"refusal": parsed.detail},
            )

        if parsed.kind is ParseKind.CLARIFY:
            self._record(
                "clarification_requested", task_id=task_id, question=parsed.question
            )
            return self._outcome(
                TaskStatus.WAITING_CLARIFICATION,
                parsed.reason_code,
                None,
                (),
                {
                    "question": parsed.question,
                    "pending_fields": list(parsed.pending_fields),
                },
            )

        try:
            goal = self._bind(parsed.draft, task_id)
        except GroundingError as exc:
            self._record(
                "binding_failed", task_id=task_id, reason_code=exc.reason_code.value
            )
            status = (
                TaskStatus.WAITING_CLARIFICATION
                if exc.reason_code
                in (ReasonCode.AMBIGUOUS_REFERENCE, ReasonCode.UNKNOWN_ENTITY)
                else TaskStatus.FAILED
            )
            return self._outcome(status, exc.reason_code, None, (), {"binding_error": str(exc)})

        self._record("goal_created", task_id=task_id, goal=goal.to_dict())
        return self._execute(goal)

    # ------------------------------------------------------------------ #
    # binding (section 25.1 / 25.2)
    # ------------------------------------------------------------------ #

    def _bind(self, draft, task_id: str) -> GoalSpec:
        view = self._runtime.view()

        # Visibility is deliberately NOT required here. The object may be behind
        # something, or simply not looked at yet; section 15 allows bounded
        # re-observation, and OBSERVE_OBJECT is what establishes visibility.
        # Substituting initialisation ground truth would be the forbidden option.
        object_id = resolve_object_id(draft.object_candidates, view, require_visible=False)

        if draft.intent is Intent.INSPECT:
            goal = GoalSpec(
                task_id=task_id,
                goal_revision=1,
                intent=Intent.INSPECT,
                object_id=object_id,
            )
            goal.to_dict()  # validate before it becomes trusted
            return goal

        target_id = resolve_station_id(draft.target_candidates, view)
        source_id = (
            resolve_station_id(draft.source_candidates, view)
            if draft.source_candidates
            else None
        )
        alternatives = tuple(
            resolve_station_id((candidate,), view)
            for candidate in draft.alternative_target_candidates
        )

        goal = GoalSpec(
            task_id=task_id,
            goal_revision=1,
            intent=Intent.TRANSPORT,
            object_id=object_id,
            source_id=source_id,
            target_id=target_id,
            allowed_alternative_targets=alternatives,
            constraints={"allow_retry": draft.allow_retry, "allow_retarget": draft.allow_retarget},
        )
        goal.to_dict()
        return goal

    # ------------------------------------------------------------------ #
    # execution loop
    # ------------------------------------------------------------------ #

    def _execute(self, goal: GoalSpec) -> TaskOutcome:
        history: list[SkillResult] = []
        step = 0

        while True:
            # 0. STOP and budget gate before anything else.
            if self._control.stop_requested:
                return self._stopped_outcome(goal, history)

            budget_keys = ["max_steps_per_task"]
            if self._planner.name == "llm":
                budget_keys.append("model_calls_per_task")
            gate = self._spend_budgets(goal, history, *budget_keys)
            if gate is not None:
                return gate

            view = self._runtime.view()
            capabilities = self._registry.capabilities(view, goal)
            world = self._runtime.snapshot(capabilities)
            request_id = f"req-{step:02d}-{uuid.uuid4().hex[:6]}"
            step += 1

            self._record(
                "planning_requested",
                task_id=goal.task_id,
                request_id=request_id,
                goal_revision=goal.goal_revision,
                world_revision=world.world_revision,
            )

            # 1. Plan off-loop so a hung model cannot block STOP.
            self._planning.submit(
                goal=goal, world=world, history=history, request_id=request_id
            )
            planning = self._planning.wait(control=self._control, now_s=self._now_s)

            if planning.stopped:
                return self._outcome(
                    TaskStatus.STOPPED,
                    ReasonCode.CANCELLED,
                    goal,
                    history,
                    {"stop_latency_s": planning.stop_latency_s},
                )

            if planning.error is not None:
                reason = self._map_planner_error(planning.error)
                self._record(
                    "planner_error",
                    task_id=goal.task_id,
                    error_type=type(planning.error).__name__,
                    reason_code=reason.value,
                )
                return self._outcome(
                    TaskStatus.FAILED,
                    reason,
                    goal,
                    history,
                    {"planner_error": f"{type(planning.error).__name__}: {planning.error}"},
                )

            proposal = planning.proposal
            if proposal is None:
                # A planner returning None is NOT, by itself, task success. The
                # goal counts as done only if the intent's terminal skill really
                # succeeded (section 25.4). Otherwise it is an empty or silent
                # planner and the task must fail rather than be declared done.
                if self._goal_satisfied(goal, history):
                    self._record("goal_satisfied", task_id=goal.task_id, steps=step)
                    return self._outcome(
                        TaskStatus.SUCCEEDED,
                        ReasonCode.OK,
                        goal,
                        history,
                        {"steps": step, "skills_executed": len(history)},
                    )
                self._record(
                    "goal_not_satisfied",
                    task_id=goal.task_id,
                    steps=step,
                    skills_executed=len(history),
                )
                return self._outcome(
                    TaskStatus.FAILED,
                    ReasonCode.GOAL_NOT_SATISFIED,
                    goal,
                    history,
                    {"steps": step, "skills_executed": len(history)},
                )

            # 2. Layer 1: structural validation. A malformed proposal must never
            #    execute, and it is rejected, never repaired (section 13).
            verdict = self._validator.check(proposal, goal=goal, world=world)
            if not verdict.ok:
                self._record(
                    "proposal_rejected",
                    task_id=goal.task_id,
                    reason=verdict.detail,
                    reason_code=verdict.reason_code.value,
                )
                return self._outcome(
                    TaskStatus.FAILED,
                    verdict.reason_code,
                    goal,
                    history,
                    {"rejected_proposal": verdict.detail},
                )

            # 3. Semantic world_revision gate (section 25.3). Only a decision-
            #    relevant change bumps world_revision, so a response computed
            #    against an older world must not be applied.
            if proposal.world_revision != world.world_revision:
                self._record(
                    "proposal_discarded",
                    task_id=goal.task_id,
                    reason="stale world_revision",
                    proposal_revision=proposal.world_revision,
                    current_revision=world.world_revision,
                )
                return self._outcome(
                    TaskStatus.FAILED,
                    ReasonCode.REVISION_MISMATCH,
                    goal,
                    history,
                    {"revision": {"proposal": proposal.world_revision, "current": world.world_revision}},
                )

            self._record(
                "proposal_received",
                task_id=goal.task_id,
                request_id=proposal.request_id,
                kind=proposal.kind.value,
                skill=proposal.skill.value if proposal.skill else None,
                reason=proposal.reason,
            )

            # 4. Non-action kinds carry no skill to supervise further.
            if proposal.kind is ProposalKind.CLARIFY:
                return self._outcome(
                    TaskStatus.WAITING_CLARIFICATION,
                    ReasonCode.AMBIGUOUS_REFERENCE,
                    goal,
                    history,
                    {"question": proposal.question},
                )

            if proposal.kind is ProposalKind.CANNOT_COMPLETE:
                self._record(
                    "cannot_complete", task_id=goal.task_id, reason=proposal.reason
                )
                return self._outcome(
                    TaskStatus.FAILED,
                    ReasonCode.NO_FEASIBLE_PLAN,
                    goal,
                    history,
                    {"reason": proposal.reason},
                )

            if proposal.kind is ProposalKind.RETARGET:
                # Section 15: retargeting is allowed only to a user-authorised
                # alternative. A planner may not invent a new destination.
                if proposal.target_id not in goal.allowed_alternative_targets:
                    self._record(
                        "unauthorised_retarget",
                        task_id=goal.task_id,
                        target=proposal.target_id,
                    )
                    return self._outcome(
                        TaskStatus.FAILED,
                        ReasonCode.PRECONDITION_FAILED,
                        goal,
                        history,
                        {"rejected_retarget": proposal.target_id},
                    )
                # Retargeting is a replan: spend the replan budget (section 13).
                gate = self._spend_budgets(goal, history, "replans_per_task")
                if gate is not None:
                    return gate
                self._record(
                    "retargeted",
                    task_id=goal.task_id,
                    from_target=goal.target_id,
                    to_target=proposal.target_id,
                )
                # Switch the goal at a skill boundary (section 14): bump the
                # revision so any stale response is discarded, and re-observe
                # under the new target.
                goal = GoalSpec(
                    task_id=goal.task_id,
                    goal_revision=goal.goal_revision + 1,
                    intent=goal.intent,
                    object_id=goal.object_id,
                    source_id=goal.source_id,
                    target_id=proposal.target_id,
                    allowed_alternative_targets=goal.allowed_alternative_targets,
                    constraints=goal.constraints,
                )
                history = []
                continue

            # 5. Layer 2: preconditions. Freshness is compared against the
            #    simulation clock (observations are stamped in sim time), so
            #    wall-clock time must not be used here.
            verdict = self._precondition.check(
                proposal,
                goal=goal,
                world=world,
                now_s=world.sim_time_s,
                budget_available=True,
            )
            if not verdict.ok:
                self._record(
                    "proposal_rejected",
                    task_id=goal.task_id,
                    reason=verdict.detail,
                    reason_code=verdict.reason_code.value,
                )
                return self._outcome(
                    TaskStatus.FAILED,
                    verdict.reason_code,
                    goal,
                    history,
                    {"rejected_proposal": verdict.detail},
                )

            # 6. Layer 3: runtime guard (stop / command expiry / body faults).
            verdict = self._guard.check(now_s=self._now_s())
            if not verdict.ok:
                self._record(
                    "guard_triggered",
                    task_id=goal.task_id,
                    reason=verdict.detail,
                    reason_code=verdict.reason_code.value,
                )
                if verdict.reason_code is ReasonCode.CANCELLED:
                    return self._stopped_outcome(goal, history)
                return self._outcome(
                    TaskStatus.FAILED,
                    verdict.reason_code,
                    goal,
                    history,
                    {"guard": verdict.detail},
                )

            # 7. Execute (single writer, stop-aware).
            self._record(
                "skill_started",
                task_id=goal.task_id,
                skill=proposal.skill.value,
                args=dict(proposal.args),
            )
            result = self._executor.run(
                skill_name=proposal.skill,
                args=proposal.args,
                world=world,
                task_id=goal.task_id,
                world_provider=self._runtime.view,
                control=self._control,
            )
            history.append(result)
            self._record(
                "skill_finished",
                task_id=goal.task_id,
                skill=result.skill.value,
                status=result.status.value,
                reason_code=result.reason_code.value,
                skill_run_id=result.skill_run_id,
            )

            if result.status is SkillStatus.CANCELLED:
                return self._outcome(
                    TaskStatus.CANCELLED, ReasonCode.CANCELLED, goal, history, {}
                )
            if result.status is SkillStatus.FAILED:
                return self._outcome(
                    TaskStatus.FAILED,
                    result.reason_code,
                    goal,
                    history,
                    {"failed_skill": result.skill.value},
                )

        # The loop only exits via a return above; this line is unreachable but
        # kept so a future refactor of the loop condition cannot fall through.
        self._record("budget_exhausted", task_id=goal.task_id, max_steps=step)
        return self._outcome(
            TaskStatus.FAILED,
            ReasonCode.BUDGET_EXHAUSTED,
            goal,
            history,
            {"max_steps": step},
        )

    @staticmethod
    def _goal_satisfied(goal: GoalSpec, history: Sequence[SkillResult]) -> bool:
        """True only if the intent's terminal skill actually succeeded.

        This is the independent postcondition gate (section 25.4): a planner
        that returns None without the terminal skill having succeeded is not a
        completed task -- it is an empty or silent planner.
        """
        terminal = _TERMINAL_SKILL[goal.intent]
        return any(
            result.skill is terminal and result.status is SkillStatus.SUCCEEDED
            for result in history
        )

    # ------------------------------------------------------------------ #
    # planner error mapping (section 12)
    # ------------------------------------------------------------------ #

    def _spend_budgets(
        self, goal: GoalSpec, history: Sequence[SkillResult], *keys: str
    ) -> TaskOutcome | None:
        """Spend one unit of each named budget, or return a failure outcome.

        Budgets cannot be reset (section 13), so exhaustion is terminal for the
        task -- it is reported, never retried or worked around.
        """
        for key in keys:
            if not self._budgets.can_spend(key):
                self._record(
                    "budget_exhausted",
                    task_id=goal.task_id,
                    budget=self._budgets.snapshot(),
                    exhausted_key=key,
                )
                return self._outcome(
                    TaskStatus.FAILED,
                    ReasonCode.BUDGET_EXHAUSTED,
                    goal,
                    history,
                    {"budget": self._budgets.snapshot(), "exhausted_key": key},
                )
            self._budgets.spend(key)
        return None

    def _map_planner_error(self, error: BaseException) -> ReasonCode:
        if isinstance(error, MissingProviderError):
            return ReasonCode.NOT_RUN_MISSING_PROVIDER
        if isinstance(error, PlannerTransportError):
            return ReasonCode.MODEL_TIMEOUT
        if isinstance(error, ValidationError):
            return ReasonCode.INVALID_OUTPUT
        return ReasonCode.PRECONDITION_FAILED

    # ------------------------------------------------------------------ #
    # outcome assembly (section 25.4)
    # ------------------------------------------------------------------ #

    def _stopped_outcome(
        self, goal: GoalSpec, history: Sequence[SkillResult]
    ) -> TaskOutcome:
        self._control.mark_handled(self._now_s())
        status = (
            TaskStatus.STOPPED
            if self._control.kind is StopKind.STOP
            else TaskStatus.CANCELLED
        )
        self._record(
            "stop_handled",
            task_id=goal.task_id,
            status=status.value,
            reason=self._control.reason,
            stop_latency_s=self._control.latency_s,
        )
        return self._outcome(
            status,
            ReasonCode.CANCELLED,
            goal,
            history,
            {
                "stop": self._control.reason,
                "stop_latency_s": self._control.latency_s,
            },
        )

    def _outcome(
        self,
        task_status: TaskStatus,
        reason_code: ReasonCode,
        goal: GoalSpec | None,
        history: Sequence[SkillResult],
        detail: Mapping[str, Any],
    ) -> TaskOutcome:
        payload = dict(detail)
        self._record(
            "task_finished",
            task_id=goal.task_id if goal else None,
            task_status=task_status.value,
            reason_code=reason_code.value,
        )
        return TaskOutcome(
            task_status=task_status,
            physical_outcome=self._physical_outcome(),
            test_verdict=TestVerdict.NOT_RUN,
            reason_code=reason_code,
            goal=goal,
            history=tuple(history),
            detail=payload,
        )

    def _physical_outcome(self) -> PhysicalOutcome:
        """The fake backend can never claim a physical result (section 25.4)."""
        if self._backend is ExecutionBackend.FAKE:
            return PhysicalOutcome.NOT_EVALUATED
        return PhysicalOutcome.NOT_APPLICABLE
