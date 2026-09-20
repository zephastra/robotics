"""Whole-fleet bring-up: world, every robot with the geofence ON, and the coordinator.

Why a separate file rather than a `robots:=[]` argument on robot.launch.py: the
coordinator is a *singleton* that owns the one reservation book, and each robot's
gate needs a permit source. Getting that ordering wrong is how a fleet boots with
every gate permissive -- which looks like a working system and is not one.

Differences from robot.launch.py, all of them deliberate:

  * ``traffic_guard`` is forced ON for every robot. A corridor claim made from a
    single-robot launch with the guard off is invalid (see D-P4-09), so the only
    place that can produce a valid one is here.
  * the coordinator starts FIRST, before any robot, so no gate is ever up with a
    coordinator it cannot reach.
  * ``start_nav2`` is on for every robot: the crossing is driven by Nav2 goals, and
    the gate sits between Nav2 and the wheels.

BUDGETS AND SCENARIOS (added for P5)
------------------------------------
CONTRACTS section 9: "场景可提前声明不同预算，不能运行中悄悄加" -- a scenario may
declare its own budgets up front, and must not add them silently at runtime.

Before this existed, the launch forwarded only `tick_hz` and `pose_timeout_s`, so
`leg_timeout_s`, `cancel_confirm_s`, `max_retries` and the start-of-charge
thresholds were whatever each node's source happened to default to. A scenario
"declaring a budget" therefore changed nothing, and the run reported budgets that
were never in force. `scenario:=<name>` fixes that: the file is read here, every
key is checked against the table below AND against the node's own
`declare_parameter` calls, and the effective value of every key is logged. An
unknown section or key is a hard error rather than a shrug.

`scripts/check_scenario_budgets.py` is the static half of the same claim, and it
runs in `build.sh`.
"""

from __future__ import annotations

import os
import sys
from pathlib import Path

import yaml
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

sys.path.insert(0, str(Path(__file__).resolve().parent))
from _common import default_models_dir, default_world  # noqa: E402


#: scenario section -> (EXECUTABLE it configures, the keys this launch forwards).
#:
#: The first element is the executable, because that is what `Node(executable=...)`
#: resolves and what `scripts/check_scenario_budgets.py` follows to find the file that
#: declares the parameters. It is NOT the node name: `coordinator_node` runs as
#: `fleet_coordinator`, and writing the node name here made the guard report that
#: three executables did not exist.
#:
#: This table is the contract between `config/scenarios/*.yaml` and this file, and the
#: guard checks it in BOTH directions. A key in a scenario that is not listed here is
#: a knob that silently does nothing; a key listed here that its node never declares
#: is a parameter the node ignores. This project has already shipped both mistakes
#: once each -- a declared-but-unread parameter, and a budget "passed" to a node that
#: had never declared it.
SCENARIO_NODES: "dict[str, tuple[str, tuple[str, ...]]]" = {
    "coordinator": (
        "coordinator_node",
        ("permit_rate_hz", "pose_timeout_s", "occupancy_settle_s"),
    ),
    "leg_executor": (
        "nav2_adapter_node",
        ("leg_timeout_s", "max_retries", "publish_hz", "start_battery_fraction"),
    ),
    "task_service": (
        "task_service_node",
        ("tick_hz", "leg_timeout_s", "cancel_confirm_s", "orphan_grace_s",
         "pinned_grace_s", "crossing_timeout_s", "crossing_queue_s"),
    ),
}

#: Keys that may be overridden PER ROBOT inside a scenario's `robots:` block.
#: Must be a subset of the `leg_executor` keys above; the guard enforces that.
SCENARIO_ROBOT_KEYS: "tuple[str, ...]" = ("start_battery_fraction",)


def _arg(context, name: str) -> str:
    return LaunchConfiguration(name).perform(context)


def _resolve_scenario_path(value: str) -> "Path | None":
    """Find a scenario by path, by stem, or by file name under config/scenarios/."""
    if not value:
        return None
    direct = Path(value)
    if direct.is_file():
        return direct
    root = os.environ.get("FLEET009_ROOT", "")
    bases = ([Path(root)] if root else []) + list(Path(__file__).resolve().parents)
    for base in bases:
        for name in (value, f"{value}.yaml"):
            candidate = base / "config" / "scenarios" / name
            if candidate.is_file():
                return candidate
    raise RuntimeError(
        f"fleet.launch.py: scenario {value!r} not found. Looked for it as a path and "
        "under config/scenarios/ from the project root; pass a path, a file name, or "
        "a file stem."
    )


def _load_scenario(value: str) -> "tuple[dict, dict, list]":
    """Return (budgets_by_section, per_robot_overrides, log lines)."""
    path = _resolve_scenario_path(value)
    if path is None:
        return {}, {}, []

    body = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
    budgets = body.get("budgets") or {}
    robots = body.get("robots") or {}
    if not isinstance(budgets, dict) or not isinstance(robots, dict):
        raise RuntimeError(f"{path}: `budgets` and `robots` must be mappings")

    for section, values in budgets.items():
        if section not in SCENARIO_NODES:
            raise RuntimeError(
                f"{path}: unknown section budgets.{section}; known sections are "
                f"{sorted(SCENARIO_NODES)}"
            )
        allowed = SCENARIO_NODES[section][1]
        for key in sorted(values):
            if key not in allowed:
                raise RuntimeError(
                    f"{path}: budgets.{section}.{key} is not forwarded to "
                    f"{SCENARIO_NODES[section][0]!r} by this launch file, so setting "
                    f"it would do nothing. Forwarded keys: {list(allowed)}"
                )
    for rid, values in robots.items():
        for key in sorted(values):
            if key not in SCENARIO_ROBOT_KEYS:
                raise RuntimeError(
                    f"{path}: robots.{rid}.{key} is not a per-robot scenario key; "
                    f"allowed: {list(SCENARIO_ROBOT_KEYS)}"
                )

    lines = [f"scenario {path}: {str(body.get('description') or '').strip()}"]
    for section in sorted(budgets):
        for key, val in sorted(budgets[section].items()):
            lines.append(f"  budgets.{section}.{key} = {val}")
    for rid in sorted(robots):
        for key, val in sorted(robots[rid].items()):
            lines.append(f"  robots.{rid}.{key} = {val}")
    return budgets, robots, lines


def _launch_setup(context, *args, **kwargs):
    share = Path(__file__).resolve().parent
    robots = [r.strip() for r in _arg(context, "robots").split(",") if r.strip()]
    if not robots:
        raise RuntimeError("fleet.launch.py: robots:= is empty")

    traffic_config = _arg(context, "traffic_config")
    scenario, robot_overrides, scenario_lines = _load_scenario(_arg(context, "scenario"))

    def budget(section: str, key: str, default):
        """Effective value of one scenario key, falling back to the launch default."""
        return (scenario.get(section) or {}).get(key, default)

    def robot_budget(robot: str, key: str, default):
        return (robot_overrides.get(robot) or {}).get(key, default)

    actions = []

    if scenario_lines:
        # Printed as one block so the run log states, in full, which budgets were in
        # force. A report that cannot quote this has to guess.
        actions.append(LogInfo(msg="\n".join(["", "=== scenario budgets in force ==="]
                                             + scenario_lines)))

    # ---- world first ---------------------------------------------------- #
    actions.append(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(share / "world.launch.py")),
        launch_arguments={
            "world": _arg(context, "world") or default_world(),
            "use_sim_time": _arg(context, "use_sim_time"),
            "models_dir": _arg(context, "models_dir") or default_models_dir(),
        }.items(),
    ))

    # ---- the coordinator, before any gate exists ------------------------- #
    actions.append(Node(
        package="fleet_ros",
        executable="coordinator_node",
        name="fleet_coordinator",
        output="screen",
        parameters=[{
            "robots": robots,
            "traffic_config": traffic_config,
            "permit_rate_hz": float(budget("coordinator", "permit_rate_hz", 5.0)),
            "pose_timeout_s": float(budget("coordinator", "pose_timeout_s", 1.5)),
            "occupancy_settle_s": float(budget(
                "coordinator", "occupancy_settle_s",
                _arg(context, "occupancy_settle_s"))),
            # This file always starts Nav2 for every robot (see the module docstring), so the
            # localiser is always there to use. The gate is given the same value in
            # robot.launch.py under the same condition: two components that must agree about
            # where a robot is may not disagree about how to find out (`D-P11-03`).
            "pose_source": "localiser",
            # Wall clock, never sim time: a permit must be able to expire while the
            # world is paused.
            "use_sim_time": False,
        }],
    ))

    # ---- one robot per entry, geofence on ------------------------------- #
    for robot in robots:
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(share / "robot.launch.py")),
            launch_arguments={
                "robot": robot,
                "traffic_guard": "true",
                "traffic_config": traffic_config,
                "permit_topic": _arg(context, "permit_topic"),
                "gate_requires_permit": "true",
                "start_nav2": "true",
                "use_sim_time": _arg(context, "use_sim_time"),
                "spawn_delay_s": _arg(context, "spawn_delay_s"),
                "nav2_log_level": _arg(context, "nav2_log_level"),
                # Forwarded explicitly, not left to robot.launch.py's own default. A fleet
                # run is the ONLY configuration that brings several Nav2 stacks up at once,
                # so it is the configuration whose load the timeout has to survive; letting
                # the two files hold separate values is how a fleet run silently gets the
                # single-robot timeout (D-P17-24).
                "nav2_service_timeout": _arg(context, "nav2_service_timeout"),
            }.items(),
        ))

    # ---- one leg executor per robot --------------------------------------- #
    #
    # Started here rather than inside robot.launch.py so that file's argument
    # machinery stays untouched: the executor belongs to the task layer, and
    # robot.launch.py must keep working for a corridor-only or single-robot run with
    # no task layer anywhere in the graph.
    if _arg(context, "start_adapters").lower() in ("1", "true", "yes"):
        fleet_config = _arg(context, "fleet_config")
        sim_time = _arg(context, "use_sim_time").lower() in ("1", "true", "yes")
        for robot in robots:
            actions.append(Node(
                package="fleet_ros",
                executable="nav2_adapter_node",
                name="nav2_adapter",
                namespace=robot,
                output="screen",
                parameters=[{
                    "robot": robot,
                    "fleet_config": fleet_config,
                    # This node integrates its battery model against sim time, because
                    # that is the clock its own odometry arrives on. Wall time here
                    # would drain the pack at a rate set by how fast the host happens
                    # to be, which is not a property of the robot.
                    "use_sim_time": sim_time,
                    # Same name as the gate's own default and as every other consumer. The
                    # launch may state the wiring, but it may not invent a third name for it.
                    "gate_state_topic": "gate_state",
                    "publish_hz": float(budget("leg_executor", "publish_hz", 10.0)),
                    "leg_timeout_s": float(budget("leg_executor", "leg_timeout_s", 180.0)),
                    "max_retries": int(budget("leg_executor", "max_retries", 2)),
                    # The one key that is per robot: a scenario about the low-battery
                    # band needs ONE robot under the threshold, not all three, and
                    # making it global would make every scenario a fleet-wide event.
                    "start_battery_fraction": float(robot_budget(
                        robot, "start_battery_fraction", 1.0)),
                }],
            ))

    # ---- the fleet task service: one, and only one ------------------------ #
    #
    # A second writer would mean two allocators handing the same task to two robots,
    # which is precisely the failure this project exists to prevent. The ledger
    # serialises within one process; it cannot arbitrate between two.
    #
    # db_path / events_path are left at the node's own defaults, which are relative to
    # the working directory. Run `ros2 launch` from the project root so runtime state
    # lands in runtime/ rather than somewhere surprising.
    if _arg(context, "start_task_service").lower() in ("1", "true", "yes"):
        actions.append(Node(
            package="fleet_ros",
            executable="task_service_node",
            name="fleet_task_service",
            output="screen",
            parameters=[{
                "robots": ",".join(robots),
                "fleet_config": _arg(context, "fleet_config"),
                "resources_config": traffic_config,
                # The ledger is durable ON PURPOSE -- restart-reconcile is a feature --
                # so it also survives between experiments, and a payload left HELD by an
                # earlier run refuses that robot every future pickup. Making the path an
                # argument lets a scenario start from its own ledger, and lets the
                # restart scenario reuse exactly one ledger, which is what it tests.
                "db_path": _arg(context, "db_path"),
                "events_path": _arg(context, "events_path"),
                "robot_state_dir": _arg(context, "robot_state_dir"),
                "tick_hz": float(budget("task_service", "tick_hz", 2.0)),
                "leg_timeout_s": float(budget("task_service", "leg_timeout_s", 180.0)),
                "cancel_confirm_s": float(budget("task_service", "cancel_confirm_s", 15.0)),
                "orphan_grace_s": float(budget("task_service", "orphan_grace_s", 30.0)),
                "pinned_grace_s": float(budget("task_service", "pinned_grace_s", 30.0)),
                # The crossing budget is the passage driver's timeout AND this node's
                # deadline, so the two cannot disagree about when a crossing failed.
                "crossing_timeout_s": float(
                    budget("task_service", "crossing_timeout_s", 150.0)),
                # How long a robot may be QUEUED for the permit before the wait counts
                # against its movement budget. Declared by the scenario, because CONTRACTS
                # section 9 says a case that expects a queue should say how long it is
                # prepared to wait. Round 16c added this key to SCENARIO_NODES and to the
                # node but NOT to this dict, so the node kept its default and the driver
                # reported `queue_allowance_s = 0.0` while the scenario said 90 s --
                # `check_scenario_budgets.py` now refuses that shape.
                "crossing_queue_s": float(
                    budget("task_service", "crossing_queue_s", 0.0)),
                # Wall clock, never sim time: the ledger's lease arithmetic and the
                # freshness test on incoming robot state must keep working while the
                # world is paused, and a paused sim clock would freeze both.
                "use_sim_time": False,
            }],
        ))

    return actions


DEFAULTS = {
    "robots": ("r01,r02", "Comma-separated robot ids. Two is the P4 acceptance set; "
                          "three is a P5 regression."),
    "traffic_config": ("", "Empty = config/resources.yaml from FLEET009_ROOT or the "
                           "source tree. All robots MUST use the same one: two different "
                           "corridor definitions is two different corridors."),
    "permit_topic": ("fleet/permit", "Relative permit topic; becomes <robot>/fleet/permit."),
    "occupancy_settle_s": (
        "5.0",
        "Seconds after the last robot first reports before the start-up occupancy "
        "sweep decides. No resource is grantable before it completes: a resource is "
        "not free just because nobody has looked at it. A scenario that names "
        "coordinator.occupancy_settle_s overrides this argument, and the effective "
        "value is printed either way."),
    "use_sim_time": ("true", "Use /clock. Must stay true: gz is the time source."),
    "world": ("", "World SDF; empty = the installed/source warehouse.sdf."),
    "models_dir": ("", "Rendered per-robot model directory."),
    "spawn_delay_s": ("3.0", "Seconds to wait for the gz create service."),
    "nav2_service_timeout": (
        "20.0",
        "Seconds each robot's Nav2 lifecycle manager waits for a lifecycle service reply. "
        "Declared here as well as in robot.launch.py because a fleet run is the only "
        "configuration that starts several Nav2 stacks at once, so it is the one whose load "
        "the timeout must survive. Stock 5.0 s aborts the bringup on this machine (D-P17-24)."),
    "nav2_log_level": (
        "info",
        "'debug' when the reason a goal aborted is needed; the interesting failures "
        "are not logged at info."),
    "fleet_config": (
        "",
        "Empty = config/fleet.yaml from FLEET009_ROOT or the source tree. The leg "
        "executor and the task service MUST agree on it: two different station lists "
        "is two different worlds, and that mismatch surfaces as an unknown station "
        "rather than as a config error."),
    "start_adapters": (
        "true",
        "Start one nav2 leg executor per robot. Off only for a corridor-only run: "
        "without them nothing can accept an ExecuteLeg goal."),
    "start_task_service": (
        "true",
        "Start the fleet task service, the single writer that turns submissions into "
        "motion. Off only for a corridor-only run that does not want tasks at all."),
    "db_path": (
        "runtime/fleet.sqlite",
        "SQLite ledger. Durable across restarts by design, so point a fresh experiment "
        "at a fresh file: a payload left HELD by an earlier session refuses that robot "
        "for every later pickup, because custody survives a restart on purpose."),
    "events_path": ("runtime/events.jsonl", "Append-only event log."),
    "robot_state_dir": ("runtime/robot_state", "Per-robot state snapshots."),
    "scenario": (
        "",
        "Name, file name or path of a file under config/scenarios/. Its `budgets:` "
        "block is applied to the nodes it names and the effective values are printed; "
        "an unknown section or key is an error, because a budget that is silently "
        "ignored makes a run report numbers that were never in force."),
}


def generate_launch_description() -> LaunchDescription:
    declared = [
        DeclareLaunchArgument(name, default_value=default, description=description)
        for name, (default, description) in DEFAULTS.items()
        if not (default == "" and name == "world")
    ]
    declared.append(DeclareLaunchArgument(
        "world", default_value=default_world(),
        description="World SDF; the world NAME is read from it for the spawn service."))
    return LaunchDescription(declared + [OpaqueFunction(function=_launch_setup)])
