"""008 command-line entry point.

Stage scope (docs/MASTER_PLAN.md section 20):

- P1 built the installable core: CLI, validation, run directory, reporting.
- P2 wires the offline closed loop: language -> GoalSpec -> rule planner ->
  fake skills -> sensor result -> next decision.
- P3 adds supervision (three layers), budgets, cancellation, and the
  provider-agnostic model interface (``replay`` + ``llm``).

``--backend mujoco`` remains explicitly unimplemented (P4): a missing backend
never silently falls back to fake. ``--planner llm`` without a configured
endpoint is recorded as ``NOT_RUN_MISSING_PROVIDER`` rather than quietly
downgrading to the rule planner (section 12).

Exit codes:
    0  task succeeded
    1  runtime error
    2  usage / configuration error
    3  explicitly not implemented at the current stage
    4  the task ran but did not complete (failure, refusal, missing provider, ...)
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

from . import SCHEMA_VERSION
from .contracts import ExecutionBackend, SkillName
from .execution import (
    BudgetLedger,
    ControlToken,
    SkillExecutor,
    TaskManager,
    load_budget_limits,
)
from .execution.planning_worker import PlanningWorker
from .language import RuleVocabulary
from .planners import (
    BridgeConfig,
    HttpPlannerBridge,
    LlmPlanner,
    ReplayPlanner,
    RulePlanner,
    load_system_prompt,
)
from .reporting import (
    EventLog,
    create_run_directory,
    new_run_id,
    write_input,
    write_jsonl,
    write_manifest,
    write_report,
)
from .skills import FakeRuntime, FakeScene, build_fake_registry
from .supervision import (
    PreconditionSupervisor,
    ProposalValidator,
    RuntimeGuard,
    RuntimePolicy,
    SupervisionPolicy,
)

PROJECT_ROOT = Path(__file__).resolve().parents[2]

EXIT_OK = 0
EXIT_RUNTIME_ERROR = 1
EXIT_USAGE = 2
EXIT_NOT_IMPLEMENTED = 3
EXIT_TASK_NOT_COMPLETED = 4

BACKENDS = ("fake", "mujoco")
PLANNERS = ("rule", "replay", "llm")
CLOCK_MODES = ("fake_clock", "planning_pause", "realtime")
DEFAULT_CLOCK_MODE = {"fake": "fake_clock", "mujoco": "planning_pause"}

DEFAULT_SCENE = "single_target"

REGISTRY_CONFIG = PROJECT_ROOT / "config" / "skills" / "registry.yaml"
RULE_CONFIG = PROJECT_ROOT / "config" / "planners" / "rule.yaml"
SUPERVISION_CONFIG = PROJECT_ROOT / "config" / "supervision" / "default.yaml"
LLM_CONFIG = PROJECT_ROOT / "config" / "planners" / "llm.yaml"
DEFAULT_REPLAY_FILE = (
    PROJECT_ROOT / "tests" / "fixtures" / "model_responses" / "transport.jsonl"
)


class ConfigError(Exception):
    """The requested run is internally inconsistent or outside the allowed scope."""


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="h008",
        description=(
            "008 - language-instruction-driven humanoid task execution "
            "(offline core; P1+P2+P3)."
        ),
    )
    parser.add_argument(
        "--backend",
        choices=BACKENDS,
        default="fake",
        help="execution backend (default: fake; mujoco arrives in P4)",
    )
    parser.add_argument(
        "--planner",
        choices=PLANNERS,
        default="rule",
        help="planner backend (default: rule; replay/llm are P3)",
    )
    parser.add_argument(
        "--replay-file",
        default=None,
        help=(
            "JSONL recording of proposals for --planner replay "
            f"(default: {DEFAULT_REPLAY_FILE.name} fixture)"
        ),
    )
    parser.add_argument(
        "--scene",
        default=None,
        help=f"registered fake scene name (default: {DEFAULT_SCENE})",
    )
    parser.add_argument(
        "--instruction",
        default=None,
        help="a single task instruction in natural language",
    )
    parser.add_argument(
        "--interactive",
        action="store_true",
        help="read task instructions interactively (excludes --instruction)",
    )
    parser.add_argument(
        "--clock-mode",
        choices=CLOCK_MODES,
        default=None,
        help="clock mode (default depends on backend/stage)",
    )
    parser.add_argument(
        "--seed",
        type=int,
        default=None,
        help="fixed integer seed, recorded in the manifest",
    )
    parser.add_argument(
        "--headless",
        action="store_true",
        help="run without a viewer (the fake backend has none)",
    )
    parser.add_argument(
        "--keep-open", action="store_true", help="keep the window open at the end"
    )
    parser.add_argument(
        "--fail-skill",
        action="append",
        default=None,
        metavar="SKILL",
        help=(
            "inject a deterministic failure into a skill (repeatable). "
            "Fake backend only; recorded in the manifest."
        ),
    )
    parser.add_argument(
        "--report-root", default=None, help="approved output directory for run reports"
    )
    parser.add_argument(
        "--version", action="version", version=f"humanoid008 {SCHEMA_VERSION}"
    )
    return parser


def validate_args(args: argparse.Namespace) -> None:
    if args.instruction is not None and args.interactive:
        raise ConfigError("--instruction and --interactive are mutually exclusive")
    if args.instruction is None and not args.interactive:
        raise ConfigError("one of --instruction or --interactive is required")
    if args.headless and args.keep_open:
        raise ConfigError("--headless and --keep-open contradict each other")
    if args.seed is not None and args.seed < 0:
        raise ConfigError("--seed must be a non-negative integer")
    if args.replay_file is not None and args.planner != "replay":
        raise ConfigError("--replay-file only applies to --planner replay")


def resolve_report_root(raw: str | None) -> Path:
    """Resolve the report root and refuse to write anywhere outside it."""
    allowed = (PROJECT_ROOT / "reports").resolve()
    root = Path(raw).expanduser().resolve() if raw else allowed
    if root != allowed:
        raise ConfigError(f"--report-root must be {allowed}; refusing to write elsewhere")
    return root


def resolve_scene_path(name: str | None) -> Path:
    scene = name or DEFAULT_SCENE
    if not scene.replace("-", "").replace("_", "").isalnum():
        raise ConfigError("--scene takes a registered scene name, not a path")
    path = PROJECT_ROOT / "config" / "scenes" / f"{scene}.yaml"
    if not path.is_file():
        raise ConfigError(f"unknown scene '{scene}' (expected {path})")
    return path


def describe_unimplemented(args: argparse.Namespace) -> str:
    """Name the first unimplemented stage that blocks this request."""
    if args.backend == "mujoco":
        return "execution backend 'mujoco' arrives in P4 (independent baseline import)"
    return ""


def _parse_faults(raw: list[str] | None) -> set[SkillName]:
    faults: set[SkillName] = set()
    for item in raw or []:
        try:
            faults.add(SkillName(item))
        except ValueError as exc:
            raise ConfigError(f"--fail-skill: unknown skill {item!r}") from exc
    return faults


def build_planner(name: str, registry, replay_file: str | None):
    """Build the requested planner. A real model is only attempted for 'llm'."""
    if name == "rule":
        return RulePlanner(registry)
    if name == "replay":
        path = Path(replay_file).expanduser() if replay_file else DEFAULT_REPLAY_FILE
        if not path.is_file():
            raise ConfigError(f"replay file not found: {path}")
        return ReplayPlanner.from_jsonl(path)
    if name == "llm":
        config = BridgeConfig.from_env(LLM_CONFIG)
        bridge = HttpPlannerBridge(config, system_prompt=load_system_prompt(LLM_CONFIG))
        return LlmPlanner(bridge, registry)
    raise ConfigError(f"unknown planner backend {name!r}")


def build_supervision(registry):
    """Assemble the three supervision layers, the budgets and the stop token."""
    policy = SupervisionPolicy.load(SUPERVISION_CONFIG)
    validator = ProposalValidator(registry)
    precondition = PreconditionSupervisor(registry, policy)
    control = ControlToken()
    guard = RuntimeGuard(RuntimePolicy.load(SUPERVISION_CONFIG), control)
    budgets = BudgetLedger(load_budget_limits(SUPERVISION_CONFIG))
    return validator, precondition, guard, budgets, control


def run_task(args: argparse.Namespace, run_dir: Path) -> int:
    """Run the offline fake closed loop and write the run's evidence."""
    events = EventLog(run_dir / "events.jsonl")
    faults = _parse_faults(args.fail_skill)

    scene = FakeScene.load(resolve_scene_path(args.scene))
    runtime = FakeRuntime(scene, faults=faults)
    registry = build_fake_registry(REGISTRY_CONFIG, runtime)
    planner = build_planner(args.planner, registry, args.replay_file)
    executor = SkillExecutor(registry)
    vocabulary = RuleVocabulary.load(RULE_CONFIG)
    validator, precondition, guard, budgets, control = build_supervision(registry)
    planning_worker = PlanningWorker(planner)

    events.record(
        "run_started",
        execution_backend=args.backend,
        planner_backend=args.planner,
        scene=scene.scene_id,
        injected_faults=sorted(skill.value for skill in faults),
    )

    manager = TaskManager(
        runtime=runtime,
        registry=registry,
        planner=planner,
        executor=executor,
        vocabulary=vocabulary,
        execution_backend=ExecutionBackend(args.backend),
        events=events,
        validator=validator,
        precondition=precondition,
        guard=guard,
        budgets=budgets,
        control=control,
        planning_worker=planning_worker,
    )
    outcome = manager.run_instruction(args.instruction, task_id=f"task-{run_dir.name}")

    executed = [result.skill.value for result in outcome.history]
    write_jsonl(run_dir / "skills.jsonl", [r.to_dict() for r in outcome.history])
    write_report(
        run_dir,
        task_status=outcome.task_status.value,
        physical_outcome=outcome.physical_outcome.value,
        test_verdict=outcome.test_verdict.value,
        detail={
            "reason_code": outcome.reason_code.value,
            "planner_backend": args.planner,
            "goal": outcome.goal.to_dict() if outcome.goal else None,
            "skills_executed": executed,
            "budget": budgets.snapshot(),
            "backend_note": (
                "fake backend: no physics, no rendering, no MuJoCo. "
                "physical_outcome is NOT_EVALUATED by design (section 25.4)."
            ),
            **outcome.detail,
        },
    )

    print(f"task_status: {outcome.task_status.value}")
    print(f"physical_outcome: {outcome.physical_outcome.value} (fake backend)")
    print(f"reason_code: {outcome.reason_code.value}")
    print(f"skills_executed: {executed}")
    for key, value in outcome.detail.items():
        print(f"  {key}: {value}")

    succeeded = outcome.task_status.value == "SUCCEEDED"
    return EXIT_OK if succeeded else EXIT_TASK_NOT_COMPLETED


def main(argv: list[str] | None = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)

    try:
        validate_args(args)
        report_root = resolve_report_root(args.report_root)
        unimplemented = describe_unimplemented(args)
        run_id = new_run_id()
        run_dir = create_run_directory(report_root, run_id)
        clock_mode = args.clock_mode or DEFAULT_CLOCK_MODE[args.backend]
        write_manifest(
            run_dir,
            run_id=run_id,
            execution_backend=args.backend,
            planner_backend=args.planner,
            clock_mode=clock_mode,
            scene=args.scene or DEFAULT_SCENE,
            seed=args.seed,
        )
        write_input(
            run_dir,
            instruction=args.instruction,
            interactive=bool(args.interactive),
        )
        if unimplemented:
            print(f"run directory: {run_dir}")
            print(
                f"NOT_IMPLEMENTED: {unimplemented}. No task was executed.",
                file=sys.stderr,
            )
            return EXIT_NOT_IMPLEMENTED
        return run_task(args, run_dir)
    except ConfigError as exc:
        print(f"configuration error: {exc}", file=sys.stderr)
        return EXIT_USAGE


def cli() -> None:
    raise SystemExit(main())


if __name__ == "__main__":
    cli()
