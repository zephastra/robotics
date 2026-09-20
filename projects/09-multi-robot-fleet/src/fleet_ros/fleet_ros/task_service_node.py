"""Fleet task service: the single writer that turns submissions into motion.

This is the node the whole P5 phase was waiting for. Everything it does was
impossible before it existed: `fleet_core` had a task machine, an allocator and a
ledger with no ROS caller, and `nav2_adapter_node` had an action server with no
client. It owns the ledger, the task machine, the allocator, the simulated energy
model and the charge-pad allocation, and it is the only thing that decides which
robot runs which leg.

Four policies that are choices rather than consequences, so they are stated here:

  * **A restart does not resume in-flight work.** `TaskMachine.transition` refuses
    any task whose stored epoch differs from the current one, by design -- that is
    what stops a stale callback from a previous life mutating a live task. So after
    a restart every previously-open task is moved to NEEDS_ATTENTION with the
    reason recorded, and its payload (if one was held) stays with its holder.
    Nothing is silently re-homed and nothing is silently resumed. CONTRACTS
    section 11 asks for exactly this conservatism. Operationally: re-submit with a
    NEW request_id; the old request_id dedupes, as section 2 requires.
  * **A cross-corridor leg is driven through the passage protocol, not around the
    barrier.** `plan_legs` tags such a leg with a corridor direction and the dispatcher
    hands that crossing to `fleet_ros.staged_crossing` -- the same driver P4 was
    accepted with, run as a process and reaped in the tick. It is NOT a plain Nav2
    goal: the sequence is drive-to-wait-node, acquire the passage, drive through,
    confirm the corridor clear, drive out, and a goal to the far side would put the
    robot in the wall while it waits for a permit it has not been granted.
    Two things follow, and both are deliberate. The step is taken from GEOMETRY at
    issue time rather than from the leg's tag, because the tag records where the plan
    saw a sign change and not where the robot is. And a crossing that fails spends an
    attempt exactly as a failed leg does, so a robot that cannot get a passage is
    retried and then parked with a reason instead of being retried for ever.
  * **A low-battery robot is not given ordinary work.** The allocator's own filter
    only checks that the trip is finishable; it would still hand a task to a robot
    at 12% that happens to be close. The extra admission rule lives here.
  * **Charging completes on the measured state, not on a timer.** The charge task
    ends when the robot reports it has stopped charging, which the adapter only
    does once the simulated pack is above the resume threshold.

Threading: this node runs on a MultiThreadedExecutor and the tick callback does the
dispatching, so like the adapter it never nests a spin. Leg futures are polled.

Known gap, stated where it can be seen: robot state arrives over a topic, so it is
never authoritative. Every decision below re-checks freshness first, and a robot
that has gone quiet is treated as a fault, not as idle.
"""

from __future__ import annotations

import json
import os
import signal
import subprocess
import time
from dataclasses import dataclass
from pathlib import Path

import rclpy
import yaml
from fleet_core import (
    Allocator,
    BatteryModel,
    ChargerAllocator,
    Ledger,
    Leg,
    LegRole,
    PlanError,
    PayloadState,
    ReasonCode,
    RobotState as CoreRobotState,
    TaskMachine,
    TaskState,
    approach_direction,
    charge_request_id,
    load_traffic_config,
    plan_legs,
    reachable_chargers,
    split_by_passage,
    validate_fleet_config,
)

# Round 17 (D-P17-02): CONTRACTS section 5's second half. A robot carrying cargo on a
# critical battery must NOT be sent to a charger and must NOT be auto-unloaded; it parks
# legally and the task is raised. The decision is pure and lives in fleet_core so the
# three parts -- the branch order, the parking geometry, the custody -- are testable
# without a simulator. `decide` is aliased because this node already has one.
from fleet_core.loaded_battery_policy import (
    ACTION_CANCEL_AND_CHARGE,
    ACTION_PARK_AND_ATTEND,
    decide as decide_loaded_battery,
    legal_park_point,
    ownership_preserved,
)
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_srvs.srv import Trigger

from fleet_interfaces.action import ExecuteLeg
from fleet_interfaces.msg import RobotState
from fleet_interfaces.srv import CancelTask, SubmitTask

SCHEMA = "1"
#: A robot whose state has not been seen for this long is not merely quiet; it is
#: gone. The arrival time is this process's own monotonic clock -- CONTRACTS
#: section 3 forbids comparing independent monotonic timestamps across processes.
STATE_MAX_AGE_S = 3.0
CHARGE_TIMEOUT_S = 600.0
#: Wall-clock budget for a cancel to be confirmed. See the parameter comment for why this
#: is not the 3 s the contract names: the adapter's own settle window is 10 s.
CANCEL_CONFIRM_S = 15.0
#: Refusals that ticking cannot clear. `ACCEPTED` means "waiting", and a stale pose, a
#: busy robot or a battery rule is a wait. A pad on the far side of the barrier is
#: geometry: the same gate refuses it on every tick until something outside this process
#: changes. Those are the only ones `_watch_unassignable` may park.
PERMANENT_REFUSALS = frozenset({
    ReasonCode.ROUTE_REQUIRES_PERMIT,
})

#: Leg results that mean "not yet" rather than "failed". A robot whose Nav2 executor has
#: not reached ACTIVE has not failed to drive anywhere, and the task service creates a
#: charge run on its first tick -- 13-16 s before that activation. Counting those as
#: attempts let one bring-up race mark a whole charge run FAILED (D-P5-23).
#:
#: It is a `frozenset` because the answer to "which reasons are waits" must be readable in
#: one place; the alternative is an `if` in a result handler, which is how the refusal
#: label in D-P5-20 ended up naming the wrong cause.
WAIT_NOT_A_FAILURE = frozenset({
    ReasonCode.NAV2_NOT_ACTIVE,
})


def _resolve_config(value: str, relative: "Path", label: str) -> str:
    """Find a config file from an explicit path, FLEET009_ROOT, or the source tree.

    Mirrors coordinator_node's convention rather than inventing a second one: an
    empty parameter must mean "the shipped config", because a second resolution rule
    is how two nodes end up reading two different corridors. Walking up from
    ``__file__`` works because the workspace is built with ``--symlink-install``, so
    the installed module resolves back into the source tree.
    """
    if value:
        return value
    env = os.environ.get("FLEET009_ROOT")
    if env and (Path(env) / relative).is_file():
        return str(Path(env) / relative)
    for parent in Path(__file__).resolve().parents:
        if (parent / relative).is_file():
            return str(parent / relative)
    raise RuntimeError(
        f"task_service: cannot locate {relative} ({label}). Set FLEET009_ROOT or pass "
        f"--ros-args -p {label}:=/abs/path"
    )


@dataclass
class PendingLeg:
    robot_id: str
    task_id: str
    leg: Leg
    command_id: str
    future: object
    sent_at: float
    #: The accepted goal handle, kept so a cancel can be sent to the leg that is
    #: actually running. Without it the dispatcher can record a cancel and has no way
    #: to deliver one.
    handle: object | None = None


@dataclass
class PendingCrossing:
    """One passage driver process, while it is running for one robot.

    Deliberately not a `PendingLeg`. A leg has a goal handle and two futures to harvest; a
    crossing has a process to reap and a report file to read, and pretending they are the same
    shape is how a reaper ends up waiting on something that will never finish. There is also a
    hard invariant behind the separate type: at most one of these exists per robot, because
    CONTRACTS section 4 allows one live command generation per robot and a crossing IS the
    generation while it runs.
    """

    robot_id: str
    task_id: str
    leg: Leg
    command_id: str
    #: "<task_id>/<leg_id>" -- what `crossed` is keyed by.
    key: str
    direction: str
    proc: object
    report_path: str
    started_at: float
    deadline: float


@dataclass
class Run:
    """Per-task runtime state. Deliberately in memory, deliberately lost on restart."""

    spec: object
    legs: tuple = ()
    cursor: int = 0
    attempts: int = 0
    #: Re-asks granted because the leg reported "not yet" (a readiness wait), kept apart
    #: from `attempts` so a bring-up race does not look like a navigation failure in the
    #: ledger. It does NOT survive a restart, and neither does anything else here.
    waits: int = 0
    pinned_robot: str = ""
    charging_since: float | None = None


class TaskService(Node):
    def __init__(self) -> None:
        super().__init__("task_service")

        self.declare_parameter("fleet_config", "")
        self.declare_parameter("resources_config", "")
        self.declare_parameter("robots", "")
        self.declare_parameter("db_path", "runtime/fleet.sqlite")
        self.declare_parameter("events_path", "runtime/events.jsonl")
        self.declare_parameter("robot_state_dir", "runtime/robot_state")
        self.declare_parameter("tick_hz", 2.0)
        self.declare_parameter("leg_timeout_s", 180.0)
        #: Bounded, and declared rather than hardcoded, so a scenario can state its own
        #: budget instead of editing the module (CONTRACTS section 9). Two ticks is not
        #: enough to distinguish a hand-over from a stall; a minute is.
        self.declare_parameter("orphan_grace_s", 60.0)
        #: How long a PINNED run may sit unassigned before it is parked. Separate from
        #: `orphan_grace_s`: that one covers a task that was assigned and lost its
        #: executor, this one covers a task that was never offered to anybody. Declared
        #: rather than hardcoded so a scenario can state its own budget (CONTRACTS 9).
        self.declare_parameter("pinned_grace_s", 60.0)
        self.declare_parameter("crossing_queue_s", 0.0)
        # The scenario's OWN estimate of how long a passage takes.
        # CONTRACTS section 9: a scenario declares its budget rather than having one
        # inferred, so this reaches the driver as a declaration. 0.0 means "not
        # declared", which leaves the reserve check switched off rather than giving
        # it a number somebody guessed.
        self.declare_parameter("crossing_expected_s", 0.0)
        #: How many times one leg may be re-asked because the robot's executor was not up
        #: yet, before that stops being a wait and starts being the leg's failure. Bounded
        #: on purpose: an unbounded "not yet" is a task that never resolves, which is the
        #: spin `_poll_pending` deliberately refuses to do.
        self.declare_parameter("wait_budget_per_task", 4)
        # Wall-clock budget for a cancel to become CANCELED. CONTRACTS section 4 item 6
        # says 3 s, which this stack cannot meet: the adapter waits up to CANCEL_SETTLE_S
        # (10 s) to see the measured speed settle after Nav2 stops, so a 3 s budget would
        # mark every cancel CANCEL_UNCONFIRMED -- the letter of the rule with the opposite
        # of its intent. 15 s = that settle window plus margin. Recorded as a conflict in
        # docs/DECISIONS.md rather than quietly redefined.
        self.declare_parameter("cancel_confirm_s", CANCEL_CONFIRM_S)
        #: Wall-clock budget for ONE corridor crossing, handed to the passage driver AND used
        #: as this node's own deadline, so the two cannot disagree about when it failed. The
        #: default is generous because the driver waits for a permit: a crossing that queues
        #: behind another robot is slow and correct, and a short budget would report the queue
        #: as a fault. Declared rather than hardcoded (CONTRACTS section 9).
        self.declare_parameter("crossing_timeout_s", 150.0)
        #: How long the passage driver gets to exit after SIGTERM before it is killed. It has
        #: no samples to flush, so this is short; it exists so a hung driver cannot pin a robot
        #: for the rest of the session.
        self.declare_parameter("crossing_kill_grace_s", 10.0)

        fleet_path = _resolve_config(
            str(self.get_parameter("fleet_config").value),
            Path("config") / "fleet.yaml", "fleet_config",
        )
        traffic_path = _resolve_config(
            str(self.get_parameter("resources_config").value),
            Path("config") / "resources.yaml", "resources_config",
        )
        self.cfg = validate_fleet_config(yaml.safe_load(Path(fleet_path).read_text()))
        self.tcfg = load_traffic_config(traffic_path)
        self.leg_timeout_s = float(self.get_parameter("leg_timeout_s").value)
        self.cancel_confirm_s = float(self.get_parameter("cancel_confirm_s").value)
        self.orphan_grace_s = float(self.get_parameter("orphan_grace_s").value)
        self.pinned_grace_s = float(self.get_parameter("pinned_grace_s").value)
        #: Handed to the driver so a robot queued for the corridor is not charged for it.
        self.crossing_queue_s = float(self.get_parameter("crossing_queue_s").value)
        self.crossing_expected_s = float(
            self.get_parameter("crossing_expected_s").value)
        self.wait_budget = int(self.get_parameter("wait_budget_per_task").value)
        self.crossing_timeout_s = float(self.get_parameter("crossing_timeout_s").value)
        self.crossing_kill_grace_s = float(
            self.get_parameter("crossing_kill_grace_s").value)

        robots_raw = str(self.get_parameter("robots").value)
        self.robots = [r.strip() for r in robots_raw.split(",") if r.strip()]
        if not self.robots:
            raise RuntimeError("task_service: robots:= is empty")
        for rid in self.robots:
            if rid not in self.cfg.robots:
                raise RuntimeError(f"task_service: {rid!r} is not in {fleet_path}")

        db_path = Path(str(self.get_parameter("db_path").value))
        db_path.parent.mkdir(parents=True, exist_ok=True)
        self.events_path = Path(str(self.get_parameter("events_path").value))
        self.events_path.parent.mkdir(parents=True, exist_ok=True)
        self.state_dir = Path(str(self.get_parameter("robot_state_dir").value))
        self.state_dir.mkdir(parents=True, exist_ok=True)

        self.ledger = Ledger(db_path)
        # A fresh epoch per process, so every task from a previous life is
        # recognisably from a previous life rather than looking current.
        self._epoch = int.from_bytes(os.urandom(4), "big") % 2_000_000_000
        self.machine = TaskMachine(self.ledger, epoch=self._epoch)
        self.allocator = Allocator(self.cfg)
        self.battery = BatteryModel(self.cfg)
        self.chargers = ChargerAllocator(self.cfg)

        self.states: dict[str, CoreRobotState] = {
            rid: CoreRobotState(rid, battery_wh=self.cfg.robots[rid].battery_capacity_wh)
            for rid in self.robots
        }
        self.state_seen: dict[str, float] = {}
        self.runs: dict[str, Run] = {}
        self.pending: dict[str, PendingLeg] = {}
        #: One passage driver per robot, while it is crossing. See `PendingCrossing`.
        self.crossings: dict[str, PendingCrossing] = {}
        #: How often route connectivity was honoured as a preference -- a robot that would have
        #: needed a passage was dropped because another candidate did not -- and how often it
        #: was overridden because no candidate could avoid one. Both are reported: an
        #: uncounted cost is a cost nobody notices, and the second number is the interesting
        #: one (it is work the system does that it did not do before this change).
        self.crossing_avoided = 0
        self.crossing_admitted = 0
        #: "<task>/<leg>" -> when it crossed. A leg whose target is across the barrier is
        #: issued as "cross, then drive", and after the driver finishes the robot's BELIEVED
        #: position can still read as the near side for a tick or two (odometry lags the
        #: motion). Without this the dispatcher asks the same question of the same stale
        #: position and crosses the same leg again. Not an identity claim: it is only a
        #: memory of "this leg has already paid for its passage", pruned by age below.
        self.crossed: dict[str, float] = {}
        #: How many releases were refused because they would have cleared a live task's
        #: ownership. Above zero means a call site is releasing on behalf of the wrong task.
        self.ownership_refusals = 0
        #: task_id -> when it was first seen assigned to a robot that is not executing it.
        self.orphan_since: dict[str, float] = {}
        #: task_id -> (pinned robot, the refusal that blocked it, human detail). Filled
        #: only for PINNED tasks, because only there is an ineligible robot a stop: the
        #: allocator will never look for a second robot.
        self.ineligible: dict[str, tuple[str, ReasonCode, str]] = {}
        #: task_id -> signature of the last "no allocation" refusal reported, so the
        #: 2 Hz tick writes one event per CHANGE rather than two per second.
        self._no_allocation: dict[str, str] = {}
        #: task_id -> when it was first seen pinned-and-ineligible.
        self.unassignable_since: dict[str, float] = {}
        self.done_commands: set[str] = set()
        #: Robots taken out of service by this node. Kept here and NOT written into
        #: `self.states[rid].fault_code`, because that object is overwritten by the
        #: next state message -- a park that the next telemetry packet erases is a
        #: park that does not exist.
        self.parked: dict[str, str] = {}
        # Round 17 (D-P17-02): the loaded-battery decisions taken this run, keyed by
        # robot. Reported in the state snapshot so a case can assert the decision
        # instead of inferring it from the fact that the robot stopped.
        self.loaded_battery: dict[str, dict] = {}
        self.charge_serial = 0
        self.refusals: dict[str, int] = {}
        self.stale_results = 0
        #: task_id -> wall-clock deadline by which a cancel must be confirmed.
        #: A cancel with no deadline is a cancel that never times out, which is how
        #: p5-D took 306 s to reach CANCELED in the 08:26Z run.
        self._cancel_deadlines: dict[str, float] = {}
        #: Ticks that raised. Surfaced in the snapshot, because a dispatcher that has
        #: silently stopped dispatching must not be able to answer healthy: true.
        self.tick_errors = 0
        self._tick_error_kinds: set[str] = set()

        group = ReentrantCallbackGroup()
        self.leg_clients: dict[str, ActionClient] = {}
        for rid in self.robots:
            self.create_subscription(
                RobotState, f"/{rid}/fleet/state", self._mk_state_cb(rid), 20,
                callback_group=group,
            )
            # Absolute name: this node is a fleet-wide singleton, not namespaced per
            # robot, so a relative "fleet/execute_leg" would look under /fleet/.
            self.leg_clients[rid] = ActionClient(
                self, ExecuteLeg, f"/{rid}/fleet/execute_leg", callback_group=group
            )

        self.create_service(SubmitTask, "fleet/submit_task", self._srv_submit)
        self.create_service(CancelTask, "fleet/cancel_task", self._srv_cancel)
        self.create_service(Trigger, "fleet/tasks", self._srv_snapshot)

        hz = max(0.2, float(self.get_parameter("tick_hz").value))
        self.create_timer(1.0 / hz, self._tick)

        self.reconcile_report = self._reconcile_at_boot()
        self.get_logger().info(
            f"task_service up: epoch={self._epoch} ledger_boot={self.ledger.boot_id} "
            f"robots={self.robots} pads={sorted(self.cfg.chargers)}"
        )

    # ------------------------------------------------------------------ #
    # restart
    # ------------------------------------------------------------------ #

    def _reconcile_at_boot(self) -> dict:
        """Conservative recovery. Nothing is resumed; everything is reported.

        This is the ONE place that writes task state without going through
        `TaskMachine.transition`, and it has to be: that method refuses any task
        whose epoch is not the current one, which is precisely every task that
        existed before this process started. The escape hatch is narrow -- it only
        ever moves tasks OUT of service, never into it.
        """
        report = self.ledger.reconcile(now=0.0, epoch=self._epoch)
        now = time.monotonic()
        not_resumed: list[dict] = []
        for entry in report.get("open_tasks", []):
            task_id = entry["task_id"]
            try:
                task = self.ledger.get(task_id)
                held = ""
                if task.spec.payload_id:
                    state, holder = self.ledger.payload_state(task.spec.payload_id)
                    if state.value == "HELD" and holder:
                        held = holder
                if held:
                    detail = (
                        f"interrupted by a coordinator restart; NOT resumed and NOT "
                        f"re-homed, because the payload is HELD by {held}."
                    )
                else:
                    detail = (
                        f"interrupted by a coordinator restart (task epoch {task.epoch} "
                        f"!= {self._epoch}); NOT resumed. Re-submit with a new request_id."
                    )
                self.ledger.set_state(
                    task_id, TaskState.NEEDS_ATTENTION, now=now,
                    # NOT CANCEL_NOT_CONFIRMED: nothing was cancelled, and a record
                    # that says a task was parked for one reason while it was parked
                    # for another is the failure 008 spent a week on.
                    reason=ReasonCode.INTERRUPTED_BY_RESTART, detail=detail,
                )
                not_resumed.append({"task_id": task_id, "was": entry["state"], "held_by": held})
            except Exception as exc:  # keep recovery going for the other tasks
                self.get_logger().error(f"reconcile: could not park {task_id}: {exc}")
        report["not_resumed"] = not_resumed
        self._emit("coordinator_restart", "fleet", report)
        if not_resumed:
            self.get_logger().warning(
                f"reconcile: {len(not_resumed)} open task(s) parked as NEEDS_ATTENTION and "
                "NOT resumed; re-submit with a new request_id"
            )
        return report

    # ------------------------------------------------------------------ #
    # plumbing
    # ------------------------------------------------------------------ #

    def _mk_state_cb(self, rid: str):
        def _cb(msg: RobotState) -> None:
            core = self.states[rid]
            # Reject an out-of-order sample. A stale pose must not overwrite a newer
            # one -- an allocator working from a rewound position sends the nearest
            # robot, which may be the far one.
            if msg.sequence and msg.sequence < core.sequence:
                return
            core.sequence = int(msg.sequence)
            core.x = float(msg.x)
            core.y = float(msg.y)
            core.yaw = float(msg.yaw)
            core.battery_wh = float(msg.battery_wh)
            core.battery_fraction = float(msg.battery_fraction)
            core.online = True
            # Both names, both from the same message field. `position_known` is what
            # the snapshot reports; `localization_valid` is what the admission and
            # recovery rules read. Only the first was ever set, so every condition
            # that read the second was reading a default of True -- a guard that is
            # always satisfied. `pose_age_s` had the same defect: it stayed 0.0, so an
            # age test could never fire.
            core.position_known = bool(msg.localization_valid)
            core.localization_valid = bool(msg.localization_valid)
            core.pose_age_s = float(msg.pose_age_s)
            core.payload_id = msg.payload_id or None
            core.stopped_confirmed = bool(msg.stopped_confirmed)
            core.operating_state = msg.operating_state
            core.fault_code = msg.fault_code
            core.gate_reason = msg.gate_reason
            core.assignment_revision = int(msg.assignment_revision)
            self.state_seen[rid] = time.monotonic()
            (self.state_dir / f"{rid}.json").write_text(
                json.dumps(
                    {
                        "robot_id": rid,
                        "x": core.x, "y": core.y, "yaw": core.yaw,
                        "battery_fraction": core.battery_fraction,
                        "battery_wh": core.battery_wh,
                        "band": self.battery.band(rid, core.battery_wh).value,
                        "operating_state": core.operating_state,
                        "task_id": msg.task_id,
                        "stopped_confirmed": core.stopped_confirmed,
                        "gate_reason": core.gate_reason,
                        "fault_code": core.fault_code,
                        "parked": self.parked.get(rid, ""),
                    "loaded_battery": self.loaded_battery.get(rid),  # round 17, D-P17-02
                        "localization_valid": core.position_known,
                        "t_monotonic": self.state_seen[rid],
                    },
                    sort_keys=True,
                ),
                encoding="utf-8",
            )
        return _cb

    def _emit(self, kind: str, subject: str, data: dict) -> None:
        """Append one event. P6's timeline reads this file, not a status field."""
        row = {
            "t": time.time(),
            "mono": time.monotonic(),
            "epoch": self._epoch,
            "kind": kind,
            "subject": subject,
            "data": data,
        }
        with self.events_path.open("a", encoding="utf-8") as fh:
            fh.write(json.dumps(row, sort_keys=True) + "\n")

    def _refuse(self, code: ReasonCode) -> None:
        self.refusals[code.value] = self.refusals.get(code.value, 0) + 1

    def _fresh(self, rid: str, now: float) -> bool:
        seen = self.state_seen.get(rid)
        return seen is not None and (now - seen) <= STATE_MAX_AGE_S

    def _now(self) -> float:
        return time.monotonic()

    # ------------------------------------------------------------------ #
    # services
    # ------------------------------------------------------------------ #

    def _srv_submit(self, request: SubmitTask.Request, response: SubmitTask.Response):
        from fleet_core import ConfigError, parse_task_request

        response.accepted = False
        response.reason_code = ReasonCode.INVALID_INPUT.value
        raw = {
            "request_id": request.request_id,
            "kind": request.kind or "station_transfer",
            "pick_station": request.pick_station,
            "destination_station": request.destination_station,
            "required_capability": request.required_capability or "CARRY",
            "payload_mode": request.payload_mode or "logical",
            "payload_id": request.payload_id or None,
            "priority": int(request.priority or 10),
            "service_duration_s": float(request.service_duration_s),
            "max_attempts": int(request.max_attempts or 2),
            "timeout_sim_s": float(request.timeout_sim_s) or self.leg_timeout_s,
        }
        try:
            spec = parse_task_request(raw, self.cfg)
        except ConfigError as exc:
            response.message = str(exc)
            self._refuse(ReasonCode.INVALID_INPUT)
            self._emit("submit_refused", request.request_id, {"detail": str(exc)})
            return response

        plan = plan_legs(spec, self.cfg, self.tcfg)
        if isinstance(plan, PlanError):
            response.message = plan.detail
            response.reason_code = plan.reason.value
            self._refuse(plan.reason)
            self._emit("submit_refused", request.request_id,
                       {"reason": plan.reason.value, "detail": plan.detail})
            return response

        crossing = [leg for leg in plan if leg.crosses_corridor]
        if crossing:
            # ACCEPTED, and the crossing is paid for at dispatch time. It was refused here
            # while the task layer could not drive one; the dispatcher now hands the crossing
            # to the passage driver, so refusing would refuse work the system can do. The tag
            # is kept -- it is the plan's own record of the intended route, and the count is
            # reported below so a submission that needs three passages is visible as one.
            self.get_logger().info(
                f"submit {request.request_id}: plan needs {len(crossing)} corridor "
                f"passage(s), first {crossing[0].leg_id} "
                f"({crossing[0].corridor_direction!r})"
            )
            self._emit("submit_needs_passage", request.request_id,
                       {"legs": [leg.leg_id for leg in crossing],
                        "directions": [leg.corridor_direction for leg in crossing]})

        try:
            task, created = self.ledger.submit(
                spec, now=self._now(), epoch=self._epoch, task_id=request.request_id
            )
        except Exception as exc:  # ConflictError and friends
            response.message = str(exc)
            response.reason_code = ReasonCode.IDEMPOTENCY_CONFLICT.value
            self._refuse(ReasonCode.IDEMPOTENCY_CONFLICT)
            return response

        if created:
            self.runs[task.task_id] = Run(spec=spec, legs=plan)
            self.machine.accept(task.task_id, now=self._now())
            self._emit("submitted", task.task_id,
                       {"spec": raw, "legs": [leg.leg_id for leg in plan]})
        task = self.ledger.get(task.task_id)
        response.accepted = True
        response.task_id = task.task_id
        response.state = task.state.value
        response.reason_code = task.reason.value
        response.message = (
            "received; acceptance is not completion"
            if created
            else f"idempotent replay of request_id {spec.request_id!r}; nothing new created"
        )
        self.get_logger().info(
            f"submit {task.task_id}: created={created} state={task.state.value}"
        )
        return response

    def _srv_cancel(self, request: CancelTask.Request, response: CancelTask.Response):
        try:
            task = self.ledger.get(request.task_id)
        except KeyError:
            response.accepted = False
            response.reason_code = ReasonCode.UNKNOWN_TASK.value
            response.message = f"no such task {request.task_id!r}"
            self._refuse(ReasonCode.UNKNOWN_TASK)
            return response
        if task.state.terminal:
            response.accepted = False
            response.state = task.state.value
            response.reason_code = ReasonCode.TASK_TERMINAL.value
            response.message = f"task is already {task.state.value}"
            return response

        result = self.machine.cancel_requested(task.task_id, now=self._now())
        response.accepted = result.accepted
        response.reason_code = result.reason.value
        response.state = self.ledger.get(task.task_id).state.value
        response.stopped = False
        response.message = (
            "cancel accepted, NOT stopped: the leg must end and measured speed must "
            "settle before this becomes CANCELED (CONTRACTS section 4 item 3)"
        )
        self._emit("cancel_requested", task.task_id, {"reason": result.reason.value})

        # TRANSMIT it. Recording a cancel in the ledger and telling nobody is not a
        # cancel: the leg keeps running, retrying and aborting on its own schedule, and
        # the dispatcher only finalises when it returns. p5-D took 306 s that way.
        self._cancel_deadlines[task.task_id] = self._now() + self.cancel_confirm_s
        rid = task.robot_id
        pending = self.pending.get(rid) if rid else None
        if pending is not None and pending.handle is not None:
            try:
                pending.handle.cancel_goal_async()
                self.get_logger().warning(
                    f"{task.task_id}: cancel sent to {rid} for leg "
                    f"{pending.command_id}"
                )
            except Exception as exc:
                # Not fatal: the deadline below still bounds this. But say so, because a
                # cancel that silently failed to send is the original defect.
                self.get_logger().error(
                    f"{task.task_id}: cancel_goal_async raised: {type(exc).__name__}: {exc}"
                )
        crossing = self.crossings.get(rid) if rid else None
        if crossing is not None:
            # A crossing is driven by a process this node owns, so it is this node's to stop.
            # Leaving it running would keep the robot moving through a corridor for a task
            # that has been cancelled -- the same defect as recording a cancel without sending
            # one, one level out.
            self.get_logger().warning(
                f"{task.task_id}: stopping the passage driver for {rid} "
                f"(direction {crossing.direction})"
            )
            self._kill_crossing(crossing)
        elif rid:
            self.get_logger().warning(
                f"{task.task_id}: cancel accepted but {rid} has no running leg to cancel"
            )
        return response

    def _srv_snapshot(self, _request, response: Trigger.Response):
        response.success = True
        response.message = json.dumps(self.snapshot(), sort_keys=True)
        return response

    # ------------------------------------------------------------------ #
    # the tick
    # ------------------------------------------------------------------ #

    def _tick(self) -> None:
        now = self._now()
        try:
            self._poll_pending(now)
            self._poll_crossings(now)
            self._enforce_cancel_deadlines(now)
            self._detect_faults(now)
            self._service_battery(now)
            self._assign_ready(now)
            self._issue_legs(now)
            self._watch_orphans(now)
            self._watch_unassignable(now)
            self._complete_charging(now)
        except Exception as exc:  # a dispatching bug must not silently stop the fleet
            # The blanket catch is right -- one bad tick must not kill the node -- but
            # swallowing it whole is not. P5.3 shipped a `ClientGoalHandle` that had no
            # `is_cancel_requested`, every tick raised, and the only trace was one line
            # per tick while five dispatching steps quietly stopped happening.
            import traceback

            self.tick_errors += 1
            kind = type(exc).__name__
            if kind not in self._tick_error_kinds:
                self._tick_error_kinds.add(kind)
                self.get_logger().error(
                    f"tick raised {kind}: {exc}\n{traceback.format_exc()}"
                )
            else:
                self.get_logger().error(
                    f"tick raised {kind}: {exc} (occurrence {self.tick_errors}; the "
                    "whole dispatch loop is skipped while this happens)"
                )

    def _active(self) -> list[tuple[str, str]]:
        return [(rid, s.executing) for rid, s in self.states.items() if s.executing]

    # ------------------------------------------------------------------ #
    # the corridor passage
    # ------------------------------------------------------------------ #

    def _crossing_dir(self, rid: str, leg: Leg) -> str | None:
        """Which direction this robot needs in order to REACH this leg, if any.

        Asked of geometry, through the helper the eligibility filter already uses. The leg's
        `corridor_direction` tag is the plan's own record of where it noticed a sign change and
        is not consulted here: it cannot say where the robot is now, which is the only thing
        this question is about.
        """
        state = self.states.get(rid)
        if state is None or not state.position_known:
            return None
        return approach_direction(state.x, leg, self.tcfg)

    def _crossing_key(self, task_id: str, leg: Leg) -> str:
        return f"{task_id}/{leg.leg_id}"

    def _start_crossing(self, rid: str, task_id: str, leg: Leg, direction: str,
                        now: float) -> "PendingCrossing | None":
        """Hand one crossing to the passage driver, as a process.

        Returns None when the driver could not even be started, which is a dependency failure
        and not a navigation one -- and it must NOT be answered by issuing the leg as a plain
        goal, because that drives the robot into the barrier. The caller parks the task with
        the reason instead.
        """
        run = self.runs.get(task_id)
        attempt = run.attempts if run is not None else 0
        command_id = f"{task_id}/r{attempt}/{leg.leg_id}"
        safe = task_id.replace("/", "_")
        reports = self.state_dir.parent / "crossings"
        reports.mkdir(parents=True, exist_ok=True)
        report_path = reports / f"{safe}-{leg.leg_id}.json"
        log_path = reports / f"{safe}-{leg.leg_id}.log"
        cmd = [
            "ros2", "run", "fleet_ros", "staged_crossing",
            "--robot", rid,
            "--direction", direction,
            "--task-id", task_id,
            # The generation is this process's epoch, so a driver left over from a previous
            # life is recognisable by its own report as belonging to one.
            "--generation", str(int(self._epoch)),
            "--timeout-s", str(self.crossing_timeout_s),
            "--queue-allowance-s", str(self.crossing_queue_s),
            "--expected-passage-s", str(self.crossing_expected_s),
            "--report", str(report_path),
        ]
        log = log_path.open("w", encoding="utf-8")
        try:
            proc = subprocess.Popen(cmd, stdout=log, stderr=subprocess.STDOUT, text=True,
                                    start_new_session=True)
        except OSError as exc:
            log.close()
            self.get_logger().error(
                f"{rid}: the passage driver could not be started: {type(exc).__name__}: {exc}")
            self._emit("crossing_unavailable", task_id,
                       {"robot": rid, "direction": direction, "error": str(exc)})
            return None
        log.close()

        item = PendingCrossing(
            robot_id=rid, task_id=task_id, leg=leg, command_id=command_id,
            key=self._crossing_key(task_id, leg), direction=direction, proc=proc,
            report_path=str(report_path), started_at=now,
            deadline=now + self.crossing_timeout_s,
        )
        self.crossings[rid] = item
        if self.ledger.get(task_id).state is TaskState.ASSIGNED:
            self.machine.begin_execution(task_id, rid, now=now)
        self._emit("crossing_started", task_id,
                   {"robot": rid, "leg": leg.leg_id, "direction": direction,
                    "command_id": command_id, "target": leg.target_name,
                    "pid": proc.pid})
        self.get_logger().info(
            f"{rid}: crossing the corridor {direction} for {task_id} ({leg.leg_id} -> "
            f"{leg.target_name}); the passage driver is pid {proc.pid}")
        return item

    def _kill_crossing(self, item: PendingCrossing) -> None:
        """Stop a driver, politely then not. Idempotent, and never raises."""
        try:
            os.killpg(os.getpgid(item.proc.pid), signal.SIGTERM)
        except Exception:
            try:
                item.proc.terminate()
            except Exception:
                pass
        try:
            item.proc.wait(timeout=self.crossing_kill_grace_s)
        except Exception:
            try:
                os.killpg(os.getpgid(item.proc.pid), signal.SIGKILL)
            except Exception:
                pass

    def _finish_crossing(self, item: PendingCrossing, detail: str,
                         now: float, reason: ReasonCode = ReasonCode.NAV_FAILED) -> None:
        """A crossing that did not succeed spends an attempt, exactly as a failed leg does."""
        self.done_commands.add(item.command_id)
        run = self.runs.get(item.task_id)
        if run is None:
            return
        run.attempts += 1
        self._emit("crossing_failed", item.task_id,
                   {"robot": item.robot_id, "direction": item.direction,
                    "attempt": run.attempts, "reason": reason.value, "detail": detail})
        if run.attempts < max(1, run.spec.max_attempts):
            self.get_logger().warning(
                f"{item.robot_id}: crossing {item.direction} for {item.task_id} failed "
                f"({detail}); attempt {run.attempts} of {run.spec.max_attempts}")
            return
        self.machine.fail(item.task_id, now=now, reason=reason,
                          detail=f"{item.leg.leg_id}: {detail}")
        self._emit("failed", item.task_id,
                   {"leg": item.leg.leg_id, "reason": reason.value, "detail": detail})
        self.get_logger().error(f"{item.task_id}: parked after a failed crossing: {detail}")
        # The task is terminal, so ownership is no longer load-bearing. Released explicitly
        # rather than left to the watchdog: a robot held by a task that has given up is a
        # robot that never gets the next task, which is the shape of D-P5-19's silent no-op.
        self._release_robot(item.robot_id, item.task_id)

    def _poll_crossings(self, now: float) -> None:
        """Reap finished passage drivers. Never waits on one."""
        for rid in list(self.crossings):
            item = self.crossings[rid]
            code = item.proc.poll()
            if code is None:
                if now > item.deadline:
                    self._kill_crossing(item)
                    del self.crossings[rid]
                    self._finish_crossing(
                        item,
                        f"the passage driver did not finish within "
                        f"{self.crossing_timeout_s:.0f} s", now)
                continue

            del self.crossings[rid]
            body: dict = {}
            try:
                body = json.loads(Path(item.report_path).read_text(encoding="utf-8"))
            except Exception:
                body = {}
            stages = [s for s in (body.get("stages") or []) if isinstance(s, dict)]
            failed = [s for s in stages if not s.get("ok")]

            task = self.ledger.get(item.task_id)
            # A task that is already being stopped: either it reached a terminal state, or a
            # cancel has been asked for and not yet confirmed. The second half is a LEDGER FIELD,
            # not a state -- CONTRACTS section 4 item 6 leaves such a task in EXECUTING with
            # `cancel_requested=1` until the leg ends, so no `TaskState` member can answer it.
            # `TaskState.CANCEL_REQUESTED` was read here and has never existed; it raised on
            # every tick that harvested a crossing, which the blanket handler in `_tick` turned
            # into "one line, eight dispatch steps skipped", and `crossing_complete` was never
            # emitted as a result. See docs/DECISIONS.md D-P7-01.
            if task.state.terminal or task.cancel_requested:
                # The task is already being stopped, so this crossing is a consequence of the
                # stop rather than a new failure. Counting it as one would put a FAILED
                # crossing into the record of a cancelled task, which reads like a fault.
                self._emit("crossing_reaped", item.task_id,
                           {"robot": rid, "exit_code": code, "state": task.state.value})
                if task.state.terminal:
                    self._release_robot(rid, item.task_id)
                continue

            if code == 0 and body.get("complete"):
                self.crossed[item.key] = now
                self._emit("crossing_complete", item.task_id,
                           {"robot": rid, "direction": item.direction,
                            "leg": item.leg.leg_id, "renewals": body.get("renewals"),
                            "target": item.leg.target_name})
                self.get_logger().info(
                    f"{rid}: crossed {item.direction} for {item.task_id}; the leg to "
                    f"{item.leg.target_name} is issued next")
                continue

            first = failed[0] if failed else {}
            detail = (f"the passage driver exited {code}"
                      + (f"; stage {first.get('name')!r}: {first.get('detail')}"
                         if first else "; its report carries no failed stage"))
            # A passage that could not be ACQUIRED is a routing admission problem, not a
            # navigation one, and the two need different responses: one is "the corridor is
            # busy or the geometry refuses you", the other is "Nav2 could not get there".
            acquired = [s for s in failed if s.get("name") == "acquire"]
            self._finish_crossing(
                item, detail, now,
                reason=(ReasonCode.ROUTE_REQUIRES_PERMIT if acquired
                        else ReasonCode.NAV_FAILED))

        # Prune the crossing memory. Entries older than two leg budgets cannot belong to a leg
        # still in flight, and keeping them for ever would be a slow leak of string keys that
        # nothing ever reads again.
        horizon = now - (self.leg_timeout_s * 2)
        for key in [k for k, when in self.crossed.items() if when < horizon]:
            del self.crossed[key]

    def _poll_pending(self, now: float) -> None:
        """Two stages, because a goal has two futures and they are not the same one.

        `send_goal_async` resolves when the SERVER ACCEPTS the goal, and its value is
        a goal handle -- not a result. Treating that future as the outcome makes every
        leg look like an instant failure with an empty message, which is exactly what
        the first three-robot run produced: `NAV_FAILED` with `detail=""` about 300 ms
        after the goal was sent. The result is a second future obtained from the
        handle, and until it exists the leg has not finished.

        Non-acceptance is handled here rather than by re-issuing silently: a goal the
        executor refuses must consume an attempt, or the dispatcher would spin on the
        same leg for ever, defeating the timeout that exists to catch exactly that.
        """
        for rid in list(self.pending):
            item = self.pending[rid]
            if now - item.sent_at > (self.leg_timeout_s * 2):
                self.get_logger().error(
                    f"{rid}: leg {item.command_id} never completed; releasing the robot "
                    "and parking the task"
                )
                del self.pending[rid]
                self._park(item.task_id, "leg never completed",
                           reason=ReasonCode.NAV_FAILED)
                self._release_robot(rid)
                continue

            if getattr(item, "stage", "SEND") == "SEND":
                if not item.future.done():
                    continue
                try:
                    handle = item.future.result()
                except Exception as exc:
                    handle = None
                    self.get_logger().error(f"{rid}: send_goal_async raised: {exc}")
                item.handle = handle
                if handle is None or not getattr(handle, "accepted", False):
                    del self.pending[rid]
                    self.done_commands.add(item.command_id)
                    run = self.runs.get(item.task_id)
                    if run is not None:
                        run.attempts += 1
                        if run.attempts >= max(1, run.spec.max_attempts):
                            self.machine.fail(
                                item.task_id, now=now, reason=ReasonCode.NAV_FAILED,
                                detail=f"{item.leg.leg_id}: the executor did not accept "
                                       "the goal",
                            )
                            self._emit("failed", item.task_id,
                                       {"leg": item.leg.leg_id, "reason": "NAV_FAILED",
                                        "detail": "goal not accepted by the executor"})
                            self._release_robot(rid)
                    self.get_logger().warning(
                        f"{rid}: executor did not accept {item.command_id}"
                    )
                    continue
                item.result_future = handle.get_result_async()
                item.stage = "RESULT"
                continue

            if not item.result_future.done():
                continue
            del self.pending[rid]
            self.done_commands.add(item.command_id)
            try:
                wrapped = item.result_future.result()
            except Exception as exc:
                wrapped = None
                self.get_logger().error(f"{rid}: get_result_async raised: {exc}")
            self._handle_leg_result(rid, item, getattr(wrapped, "result", None), now)

    def _handle_leg_result(self, rid: str, item: PendingLeg, result, now: float) -> None:
        run = self.runs.get(item.task_id)
        if run is None:
            return
        task = self.ledger.get(item.task_id)
        if task.state.terminal:
            return
        ok = bool(getattr(result, "ok", False))
        detail = str(getattr(result, "message", ""))
        reason_raw = str(getattr(result, "reason_code", ""))
        try:
            reason = ReasonCode(reason_raw) if reason_raw else ReasonCode.NAV_FAILED
        except ValueError:
            reason = ReasonCode.NAV_FAILED
        stopped = bool(getattr(result, "stopped_confirmed", False))

        if task.cancel_requested:
            # The leg ended after a cancel was requested. CANCELED only once the
            # motion is confirmed over; otherwise the payload and any held resource
            # stay exactly where they are.
            if stopped:
                self.machine.cancel_confirmed(item.task_id, now=now)
                self._emit("canceled", item.task_id, {"robot": rid})
            else:
                self._park(item.task_id, "cancel requested but stop not confirmed",
                           reason=ReasonCode.CANCEL_UNCONFIRMED)
            self._release_robot(rid)
            return

        if not ok and reason in WAIT_NOT_A_FAILURE and run.waits < self.wait_budget:
            # The leg was refused before anything was attempted: the robot's Nav2 had not
            # reached ACTIVE. Re-ask, spend no attempt, and record it -- a wait that is
            # counted is diagnosable, and one that is not is a hang that looks like work.
            run.waits += 1
            self._emit("leg_deferred", item.task_id,
                       {"leg": item.leg.leg_id, "reason": reason.value, "detail": detail,
                        "waits": run.waits, "budget": self.wait_budget})
            self.get_logger().warning(
                f"{item.task_id}: leg {item.leg.leg_id} deferred ({reason.value}: "
                f"{detail}); wait {run.waits} of {self.wait_budget}, no attempt spent"
            )
            return            # re-issued by _issue_legs on the next tick

        if not ok:
            run.attempts += 1
            if run.attempts < max(1, run.spec.max_attempts):
                self.get_logger().warning(
                    f"{item.task_id}: leg {item.leg.leg_id} failed ({reason_raw}: {detail}); "
                    f"attempt {run.attempts} of {run.spec.max_attempts}"
                )
                return  # same leg re-issued by _issue_legs
            self.machine.fail(item.task_id, now=now, reason=reason,
                              detail=f"{item.leg.leg_id}: {detail}")
            self._emit("failed", item.task_id,
                       {"leg": item.leg.leg_id, "reason": reason.value, "detail": detail})
            self._release_robot(rid)
            return

        role = item.leg.role
        if role is LegRole.PICK_SERVICE and run.spec.payload_id:
            if not self.ledger.pick(run.spec.payload_id, task_id=item.task_id, robot_id=rid):
                self.machine.fail(
                    item.task_id, now=now, reason=ReasonCode.PAYLOAD_HELD_ELSEWHERE,
                    detail=f"payload {run.spec.payload_id!r} is held by someone else",
                )
                self._emit("pick_refused", item.task_id, {"payload": run.spec.payload_id})
                self._release_robot(rid)
                return
            self._emit("picked", item.task_id,
                       {"payload": run.spec.payload_id, "robot": rid})
        elif role is LegRole.DROP_SERVICE and run.spec.payload_id:
            if not self.ledger.deliver(run.spec.payload_id, task_id=item.task_id, robot_id=rid):
                self.machine.fail(item.task_id, now=now, reason=ReasonCode.UNSUPPORTED,
                                  detail="delivery refused by the ledger")
                self._release_robot(rid)
                return
            self._emit("delivered", item.task_id,
                       {"payload": run.spec.payload_id, "robot": rid})

        run.cursor += 1
        run.attempts = 0
        run.waits = 0
        if run.cursor >= len(run.legs):
            if task.spec.kind.value == "return_to_charge":
                # The drive is done; charging itself is observed, not commanded.
                run.charging_since = now
                return
            self.machine.succeed(item.task_id, now=now)
            self._emit("succeeded", item.task_id, {"robot": rid})
            self._release_robot(rid)

    def _release_robot(self, rid: str, task_id: str = "") -> None:
        """Free a robot -- but not from a live task that is not the caller's.

        `executing` used to be cleared unconditionally. A release arriving for an older task
        could therefore clear a NEWER task's ownership, and since `_active()` yields only
        robots whose `executing` is set, that newer task was never looked at again: it stayed
        ASSIGNED in the ledger with a robot_id, a planned leg, `attempts: 0` and `reason: OK`
        for as long as the process lived. Measured: three tasks assigned to r03 within eight
        seconds while `_assign_ready` refuses an already-executing robot -- which requires
        ownership to have been freed twice -- and the third then sat there for 213 s.

        The guard reads the LEDGER rather than trusting the caller, so it protects all twelve
        call sites without editing any of them. That is deliberate: twelve mechanical edits
        are twelve chances to miss one, and a missed one brings the bug back silently.

        `task_id` is optional and only tightens the check -- a caller that knows which task it
        is releasing passes it, and then a matching owner is released even if non-terminal.
        An unknown owner (no ledger row) is treated as releasable: a task that does not exist
        cannot be orphaned.
        """
        if rid in self.crossings:
            # The robot is mid-passage. Clearing `executing` here would not stop the driver --
            # nothing else looks at it -- and would let the allocator hand this robot a second
            # task while the first one is still driving it through the corridor.
            self.ownership_refusals += 1
            if self.ownership_refusals == 1 or self.ownership_refusals % 20 == 0:
                self.get_logger().warning(
                    f"{rid}: refused to release"
                    + (f" for {task_id}" if task_id else "")
                    + f": it is crossing the corridor for "
                      f"{self.crossings[rid].task_id} ({self.ownership_refusals} "
                      "refusal(s) so far)"
                )
            return

        owner = self.states[rid].executing
        if owner and owner != task_id:
            try:
                owner_live = not self.ledger.get(owner).state.terminal
            except KeyError:
                owner_live = False
            if owner_live:
                self.ownership_refusals += 1
                if self.ownership_refusals == 1 or self.ownership_refusals % 20 == 0:
                    self.get_logger().warning(
                        f"{rid}: refused to release"
                        + (f" for {task_id}" if task_id else "")
                        + f": it is executing {owner}, which is still live "
                        f"({self.ownership_refusals} refusal(s) so far). Clearing it here "
                        "would orphan that task in ASSIGNED with nothing left to look at it."
                    )
                return
        self.states[rid].executing = None
        self.chargers.release(rid, now=self._now())

    def _watch_orphans(self, now: float) -> None:
        """Park a task that is assigned to a robot which is not executing it.

        MASTER_PLAN section 9 requires a DIAGNOSABLE wait policy, and a task in this state is
        not waiting for anything -- no loop looks at it again. Before this it stayed ASSIGNED
        for ever with `reason: OK` and `attempts: 0`, which is indistinguishable from a task
        that is about to start. The grace period exists so that the normal hand-over between
        two ticks is not mistaken for a stall; it is a parameter so a scenario can declare a
        shorter one instead of editing it.
        """
        tracked = {s.executing for s in self.states.values() if s.executing}
        for task in self.ledger.open_tasks():
            if task.state is not TaskState.ASSIGNED or task.task_id in tracked:
                if task.task_id in self.orphan_since:
                    del self.orphan_since[task.task_id]
                continue
            if self.runs.get(task.task_id) is None:
                # No plan in this process: a task from a previous life. Reconcile owns it.
                continue
            since = self.orphan_since.setdefault(task.task_id, now)
            waited = now - since
            if waited < self.orphan_grace_s:
                continue
            del self.orphan_since[task.task_id]
            why = (f"assigned to {task.robot_id or '(nobody)'} but no robot is executing "
                   f"it after {waited:.1f}s; no dispatcher loop would look at it again")
            self._park(task.task_id, why, reason=ReasonCode.ORPHANED_ASSIGNMENT)
            self.get_logger().error(
                f"{task.task_id}: orphaned in ASSIGNED for {waited:.1f}s -> "
                f"{ReasonCode.ORPHANED_ASSIGNMENT.value}"
            )

    def _watch_unassignable(self, now: float) -> None:
        """Park a pinned run whose own robot can never be offered it, and free the pad.

        A charge run is pinned, so the allocator never looks for a second robot: an
        ineligible pinned robot means the task is refused on every tick for ever. It
        reads ACCEPTED, which is indistinguishable from "about to start", while the pad
        it reserved stays reserved for it. One run ended exactly there -- two pads held,
        `grants=2 releases=0`, three robots at their spawns for the whole run, and the
        only trace was 2087 counts in a counter that named no robot.

        Only PERMANENT_REFUSALS are parked. A stale pose, a busy robot or a battery rule
        is a wait, and `ACCEPTED` means "waiting"; parking a wait would turn a
        recoverable condition into an operator call-out. The reason written into the
        ledger is the refusal that actually blocked it, never a constant.
        """
        if len(self.ineligible) > 64:
            live = {t.task_id for t in self.ledger.open_tasks()}
            for stale in [k for k in self.ineligible if k not in live]:
                del self.ineligible[stale]

        for task in self.ledger.open_tasks():
            blocked: "tuple[str, ReasonCode, str] | None" = None
            if task.state is TaskState.ACCEPTED:
                run = self.runs.get(task.task_id)
                if run is not None and run.pinned_robot:
                    candidate = self.ineligible.get(task.task_id)
                    if candidate is not None and candidate[1] in PERMANENT_REFUSALS:
                        blocked = candidate
            if blocked is None:
                self.unassignable_since.pop(task.task_id, None)
                continue
            since = self.unassignable_since.setdefault(task.task_id, now)
            waited = now - since
            if waited < self.pinned_grace_s:
                continue
            del self.unassignable_since[task.task_id]
            rid, code, detail = blocked
            why = (f"pinned to {rid}, which has been ineligible for {waited:.1f}s: "
                   f"{detail}")
            self._park(task.task_id, why, reason=code)
            # The pad was granted when the run was created, so the robot holding it has
            # no task left that would ever release it. Third time this project has
            # shipped that combination; this is the line that breaks it.
            self.chargers.release(rid, now=now)
            self.get_logger().error(
                f"{task.task_id}: pinned to {rid} but ineligible for {waited:.1f}s "
                f"({code.value}) -> parked and the pad released"
            )

    def _note_no_allocation(self, task_id: str, allocation) -> None:
        """Record why a task is not being assigned, once per change of reason.

        The dispatcher refuses at 2 Hz, so an event per tick would write the same line
        twice a second for as long as the task waits and bury the one line that matters.
        One event per change keeps the evidence readable while still naming which robot
        was refused for what -- the part the refusal counter never had, and the reason a
        capable-but-busy fleet could be read as an incapable one for a whole round.
        """
        rejected = [[rid, code.value] for rid, code in allocation.rejected]
        signature = allocation.reason.value + "|" + ";".join(f"{r}:{c}" for r, c in rejected)
        if self._no_allocation.get(task_id) == signature:
            return
        self._no_allocation[task_id] = signature
        if len(self._no_allocation) > 64:
            live = {t.task_id for t in self.ledger.open_tasks()}
            for stale in [k for k in self._no_allocation if k not in live]:
                del self._no_allocation[stale]
        self._emit("no_allocation", task_id,
                   {"reason": allocation.reason.value, "detail": allocation.detail,
                    "rejected": rejected, "candidates": len(allocation.candidates)})

    def _park(self, task_id: str, why: str, *, reason: ReasonCode) -> None:
        """Park a task, recording why it was actually parked.

        `reason` used to be hardcoded to CANCEL_NOT_CONFIRMED at every call site, so a
        navigation timeout, a blocked protected region and a charging timeout all wrote
        the same machine-readable reason into the ledger. That is the shape that cost 008
        a week: a `fail_reason` column that was a constant, and a report nobody could
        trust. The caller now supplies it and the human text goes into `detail`.
        """
        try:
            self.machine.needs_attention(
                task_id, now=self._now(), reason=reason, detail=why
            )
        except Exception:
            self.ledger.set_state(task_id, TaskState.NEEDS_ATTENTION, now=self._now(),
                                  reason=reason, detail=why)
        self._emit("needs_attention", task_id, {"why": why, "reason": reason.value})

    def _park_robot(self, rid: str, why: str) -> None:
        self.parked[rid] = why
        self.get_logger().error(f"{rid} PARKED: {why}")

    def _corridor_knowledge(self, rid: str, state, now: float):
        """Is this robot inside a protected region? YES / NO / UNKNOWN, never guessed.

        `UNKNOWN` is not `NO`: `recovery.classify` treats it as a possible corridor loss,
        and that is the correct default when the fleet cannot say. The bug was that it was
        the ONLY value ever passed, so every fault anywhere in the world was classified as
        a corridor loss and parked as BLOCK_RESOURCE. The contract's section 5 rows for a
        held payload and for an unpicked assignment were unreachable in production.

        The pose used here is the robot's own estimate, already composed into the map
        frame by its adapter. It is only trusted when the sample is fresh, the robot says
        it is localised, and the robot's own reported pose age agrees. Anything else
        stays UNKNOWN, so the fail-closed behaviour is unchanged where it matters.
        """
        from fleet_core import InCorridorKnowledge, regions_overlapping

        if not self._fresh(rid, now) or not state.localization_valid:
            return InCorridorKnowledge.UNKNOWN
        if state.pose_age_s > STATE_MAX_AGE_S:
            return InCorridorKnowledge.UNKNOWN
        spec = self.cfg.robots.get(rid)
        if spec is None:
            return InCorridorKnowledge.UNKNOWN
        hits = regions_overlapping(
            self.tcfg,
            (state.x, state.y, state.yaw),
            length_m=spec.footprint_length_m,
            width_m=spec.footprint_width_m,
        )
        return InCorridorKnowledge.YES if hits else InCorridorKnowledge.NO

    def _enforce_cancel_deadlines(self, now: float) -> None:
        """A cancel that is not confirmed in time becomes CANCEL_UNCONFIRMED.

        CONTRACTS section 4 item 6. Without this a cancel has no upper bound at all: the
        task sits in EXECUTING with `cancel_requested=1` until the leg happens to end,
        which in one run was 306 seconds. The task is parked rather than failed, because
        the robot may still be moving and its cargo is still in its hands -- and parking
        is what keeps the dispatcher from handing that robot more work.
        """
        for task_id, deadline in list(self._cancel_deadlines.items()):
            if now < deadline:
                continue
            task = self.ledger.get(task_id)
            if task.state.terminal:
                del self._cancel_deadlines[task_id]
                continue
            if not task.cancel_requested:
                # It was confirmed and then reopened, or the request was withdrawn.
                del self._cancel_deadlines[task_id]
                continue
            rid = task.robot_id or ""
            # Give the leg one more chance to report a confirmed stop before parking:
            # the adapter needs its own settle window, and it reports through the result.
            pending = self.pending.get(rid) if rid else None
            # A CLIENT-side goal handle cannot be asked "was my cancel requested" --
            # `is_cancel_requested` lives on the server handle, which is what the
            # adapter uses inside its execute callback. Asking anyway raised
            # AttributeError on every tick and killed the whole dispatcher loop, so
            # the cancel still had no deadline and five other steps stopped running.
            # Sending a cancel twice is harmless; guessing at the client API is not.
            if pending is not None and pending.handle is not None:
                try:
                    pending.handle.cancel_goal_async()
                except Exception:
                    pass
            detail = (
                f"cancel not confirmed within {self.cancel_confirm_s:.0f}s "
                f"(wall); the robot may still be moving and keeps its cargo"
            )
            self._park(task_id, detail, reason=ReasonCode.CANCEL_UNCONFIRMED)
            if rid:
                self._park_robot(rid, f"CANCEL_UNCONFIRMED: {detail}")
                self._release_robot(rid)
            del self._cancel_deadlines[task_id]

    def _detect_faults(self, now: float) -> None:
        from fleet_core import InCorridorKnowledge, decide

        for rid in self.robots:
            state = self.states[rid]
            fresh = self._fresh(rid, now)
            if fresh and not state.fault_code:
                continue
            task_id = state.executing
            if not task_id:
                continue
            try:
                task = self.ledger.get(task_id)
            except KeyError:
                continue
            payload_state = None
            payload_holder = None
            if task.spec.payload_id:
                payload_state, payload_holder = self.ledger.payload_state(task.spec.payload_id)
            # `stopped_confirmed` comes from the robot's own measurement. A quiet
            # heartbeat is NOT evidence of a stopped robot, and this is the single
            # input that keeps two robots from racing for one payload.
            decision = decide(
                task,
                payload_state=payload_state,
                payload_holder=payload_holder,
                stopped_confirmed=state.stopped_confirmed,
                in_corridor=self._corridor_knowledge(rid, state, now),
            )
            self._emit("fault_decision", task_id,
                       {"robot": rid, "fresh": fresh, "fault": state.fault_code,
                        "action": decision.action, "reason": decision.reason.value,
                        "detail": decision.detail})
            if decision.action_required:
                # If the robot is still holding the goods, the ledger must say so with the
                # contract's own code: "someone else holds it" and "you are parked and
                # you still have it" are different, and the difference decides whether
                # an operator looks for a missing cargo or for a stuck robot.
                park_reason = decision.reason
                if payload_state is PayloadState.HELD and payload_holder is not None:
                    park_reason = ReasonCode.PAYLOAD_HELD_NEEDS_ATTENTION
                self._park(task_id, f"{decision.action}: {decision.detail}",
                           reason=park_reason)
                # A vanished robot must not hold a charge pad or a payload forever.
                self.chargers.drop_offline(rid, now=now)
                self.done_commands.clear()
                self.states[rid].executing = None

    def _service_battery(self, now: float) -> None:
        """Low battery -> a system-generated return_to_charge, capacity permitting.

        Two rules live here and they are NOT the same rule (CONTRACTS section 5):

          * **unloaded** -- cancel the leg, generate a charge run. This is what the body
            of the loop below does, and it needs an idle robot.
          * **carrying cargo** -- do NOT auto-unload. Park at a legal, reachable position
            and raise the task. `D-P17-02` records that nothing implemented this, and the
            reason is the `state.executing` guard: it skipped every mid-leg robot before
            the band was even read, so a loaded robot could never reach the decision.

        The guard is therefore split. An executing robot is still not given a charge run
        -- driving a loaded robot to a pad is the thing the contract forbids -- but it IS
        asked what to do when it is loaded.
        """
        for rid in self.allocator.low_battery_robots(self.states):
            if rid in self.parked:
                continue
            state = self.states[rid]
            if not self._fresh(rid, now):
                continue
            band = self.battery.band(rid, state.battery_wh).value
            if state.executing:
                # The loaded half. It is checked first for the same reason the policy
                # checks it first: getting the order wrong sends a loaded robot to a
                # charger.
                if band == "CRITICAL":
                    self._service_loaded_battery(rid, state, band, now)
                continue
            if band == "CRITICAL":
                reach = self.battery.can_reach_charger(
                    rid, charge_wh=state.battery_wh,
                    distance_m=self._nearest_charger_distance(rid), turnaround_rad=0.0,
                )
                if not reach.ok:
                    # Report rather than promise: a robot that cannot prove it can
                    # reach a pad must not be told to drive to one.
                    self._emit("no_safe_charger", rid,
                               {"charge_wh": state.battery_wh, "margin_wh": reach.margin_wh,
                                "detail": "critical and cannot prove it can reach a pad; "
                                          "stopping and reporting instead of promising"})
                    self._park_robot(rid, "INSUFFICIENT_BATTERY: no provable charger route")
                    continue
            if self._has_charge_task(rid):
                continue
            self._create_charge_task(rid, now)

    def _carried_by(self, rid: str) -> str | None:
        """The id of the cargo this robot holds, or None.

        Read from the ledger, which is the single writer for payload state, rather than
        from a local field. A local field would drift the moment a pick or a drop was
        refused, and this value is what `ownership_preserved` compares.
        """
        for payload_id, holder in sorted(self.ledger.payload_holders().items()):
            if holder == rid:
                return payload_id
        return None

    def _service_loaded_battery(self, rid: str, state, band: str, now: float) -> None:
        """CONTRACTS section 5, loaded half. Called for a robot that is EXECUTING.

        Before this existed, `_service_battery` skipped every executing robot, so a robot
        forced to critical mid-leg drove on with the cargo and F05's expected result --
        keep custody, park legally, raise attention -- could not happen at all.
        """
        carried = self._carried_by(rid)
        decision = decide_loaded_battery(
            band, executing=bool(state.executing), carrying=carried is not None,
            has_charge_task=self._has_charge_task(rid),
        )
        # The legal parking geometry needs a candidate list and, when the robot knows its
        # own position, the reachable set. Without a position there is nothing to choose
        # between, so the decision is reported and the robot stops where it is.
        park = None
        if decision.action == ACTION_PARK_AND_ATTEND and state.position_known:
            # Both calls below are inside an `and` chain, and the outer condition is the
            # LOADED half of CONTRACTS section 5 -- the half that had never been selected
            # before 2026-09-19. Until then these two methods did not exist and the whole
            # branch would have raised AttributeError on its first execution, which the
            # tick handler would have logged as `tick raised AttributeError` once per
            # second while `_emit("loaded_low_battery_parked")` never happened. That is
            # D-P17-19, and it is why F05's expected result could not occur.
            # `scripts/check_self_attrs.py` now finds this shape by AST.
            park = legal_park_point(
                (state.x, state.y, 0.0),
                self._park_candidates(),
                footprint_radius_m=float(self.cfg.footprint_margin_m or 0.30),
                reachable=self._reachable_park_candidates(state),
            )

        record = {
            "action": decision.action,
            "band": band,
            "carrying": carried,
            "carried_by": carried,
            "reason": decision.reason,
            "park_at": list(park["park_at"]) if park and park.get("park_at") else None,
            "park_reason": park["reason"] if park else "no position fix; parking in place",
        }
        self.loaded_battery[rid] = record

        if decision.action == ACTION_PARK_AND_ATTEND:
            # Parking is what actually stops the drive. `parked` is consulted by the
            # driver dispatch path, so setting it here is the difference between
            # "the fleet decided to stop" and "the robot stopped".
            self._park_robot(
                rid,
                "CRITICAL battery while carrying "
                f"{carried}: CONTRACTS section 5 forbids auto-unload, "
                f"{'parking at ' + str(park['park_at']) if park and park.get('park_at') else 'holding position'}"
                " and raising the task for attention",
            )
            self._emit("loaded_low_battery_parked", rid, record)
            return

        if decision.action == ACTION_CANCEL_AND_CHARGE:
            # An executing, UNLOADED robot on a critical battery: the leg is cancelled and
            # a charge run is requested. This is the first half of the rule, which already
            # existed -- but it could only fire for an idle robot before, because of the
            # same skip. Cancelling is left to the normal path so the ledger stays the
            # single writer.
            self._emit("loaded_low_battery_cancel", rid, record)

    def _park_candidates(self) -> list[tuple[float, float]]:
        """Points a LOADED robot may stop at, as (x, y).

        The configured stations, minus the chargers.

        Why stations: they are the points the rest of the system already names and the
        dispatcher can already route a robot to. Generating a grid here would invent a
        second, unreviewed notion of where a robot may stand, and the first thing that
        would happen is the two notions disagreeing.

        Why not the chargers: CONTRACTS section 5 forbids driving a loaded robot to a
        charge pad. A pad is a legal place to be and an illegal place to be *with cargo*,
        so it is excluded from the candidate list rather than filtered later -- a
        candidate that is only rejected at the end is one edit away from being offered.

        Whether each point actually clears a protected rectangle is `legal_park_point`'s
        question, answered against `PROTECTED_RECTS` by the same `_clears_every_rect` the
        parking policy uses. This method deliberately does not re-implement that test:
        two copies of a clearance rule is how the gate and the driver came to disagree by
        exactly the stopping reserve (D-P10-01).
        """
        stations = getattr(self.cfg, "stations", {}) or {}
        chargers = set(getattr(self.cfg, "chargers", {}) or {})
        return [
            (float(spec.x), float(spec.y))
            for name, spec in sorted(stations.items())
            if name not in chargers
        ]

    def _reachable_park_candidates(self, state) -> list[tuple[float, float]]:
        """The subset of `_park_candidates()` this robot can still pay for.

        Uses `can_reach_charger`, the same reachability model the `no_safe_charger`
        path already uses. "Reachable" has to mean one thing in this file: the alternative
        is a robot told to drive somewhere on a battery that cannot get it there, which is
        the promise `no_safe_charger` already refuses to make and `legal_park_point`
        already has a branch for ("legal parking points exist but none is reachable on the
        remaining battery ... report rather than be promised a spot").

        Returning the full candidate list when the battery model cannot answer would make
        the reachability branch of `legal_park_point` dead code, which is how a rule stops
        being a rule. So the question is asked per candidate and the answer is used.
        """
        rid = getattr(state, "robot_id", None)
        if rid is None:
            # `RobotState` is a dataclass whose first field is `robot_id`, so this is
            # unreachable for the type the node actually stores. It is here because a
            # caller passing a duck-typed stand-in should get an EMPTY reachable set --
            # "no spot can be shown to be reachable" -- rather than an AttributeError or,
            # worse, the whole candidate list with the check silently skipped.
            return []
        out: list[tuple[float, float]] = []
        for (cx, cy) in self._park_candidates():
            reach = self.battery.can_reach_charger(
                rid, charge_wh=state.battery_wh,
                distance_m=state.distance_to(cx, cy), turnaround_rad=0.0,
            )
            if reach.ok:
                out.append((cx, cy))
        return out

    def _nearest_charger_distance(self, rid: str) -> float:
        state = self.states[rid]
        reachable = [
            state.distance_to(self.cfg.stations[c].x, self.cfg.stations[c].y)
            for c in self.cfg.chargers
            if c in self.cfg.stations
        ]
        return min(reachable) if reachable else 0.0

    def _has_charge_task(self, rid: str) -> bool:
        for task_id, run in self.runs.items():
            if run.pinned_robot != rid or run.spec.kind.value != "return_to_charge":
                continue
            try:
                if not self.ledger.get(task_id).state.terminal:
                    return True
            except KeyError:
                continue
        return False

    def _create_charge_task(self, rid: str, now: float) -> None:
        from fleet_core import parse_task_request

        # Ask for a pad BEFORE creating the task. Admitting a robot to a charge run
        # it cannot finish would put it on a pad it never reaches, or on top of one
        # that is already occupied.
        state = self.states[rid]
        if not state.position_known:
            # There is no way to tell which pad this robot can drive to, and a pad
            # granted on a guess is a pad reserved for a robot that may never be able to
            # reach it. Wait for the first fix: this runs every tick, so the run is
            # created on the tick after odometry arrives.
            self._refuse(ReasonCode.LOCALIZATION_STALE)
            return
        reachable = reachable_chargers(self.cfg, self.tcfg, state.x)
        # Ask for a pad this robot can actually be dispatched to (D-P5-22). The
        # allocator never looks for a second robot -- charge runs are pinned -- so a pad
        # on the far side of the barrier is not a shorter wait, it is a permanent
        # refusal with the pad reserved for the robot that cannot use it.
        grant = self.chargers.request(rid, now=now, reachable=reachable)
        if grant is None:
            if reachable:
                # An admissible pad is occupied or has somebody ahead of us in its
                # queue. That is a wait, and the allocator has counted it already.
                self._refuse(ReasonCode.CHARGER_BUSY)
            snapshot = self.chargers.snapshot()
            # Which pads were even considered, and why the rest were not. Without this
            # the event says "queued" for a robot that has nowhere to queue for.
            snapshot["reachable_for"] = {"robot": rid, "pads": list(reachable)}
            self._emit("charger_queued" if reachable else "no_reachable_pad", rid, snapshot)
            return
        self.charge_serial += 1
        request_id = charge_request_id(rid, self._epoch, self.charge_serial)
        spec = parse_task_request(
            {"request_id": request_id, "kind": "return_to_charge",
             "destination_station": grant.charger_id},
            self.cfg,
        )
        plan = plan_legs(spec, self.cfg, self.tcfg)
        if isinstance(plan, PlanError):
            self.chargers.release(rid, now=now)
            self._emit("charge_plan_refused", rid, {"detail": plan.detail})
            return
        task, created = self.ledger.submit(spec, now=now, epoch=self._epoch, task_id=request_id)
        if not created:
            # The pad was granted a few lines above. Returning here without releasing it
            # leaves a charger held for the session by a robot that has no task to ever
            # release it, and the status page still shows a plausible occupant.
            self.chargers.release(rid, now=now)
            self._emit("charge_not_created", rid,
                       {"request_id": request_id, "existing_task": task.task_id,
                        "detail": "the request_id already existed, so no new charge run "
                                  "was created; the pad has been released rather than "
                                  "held by a robot with no task"})
            self.get_logger().warning(
                f"{rid}: charge run {request_id} already existed in the ledger; "
                "released the pad instead of holding it without a task"
            )
            return
        self.runs[task.task_id] = Run(spec=spec, legs=plan, pinned_robot=rid)
        self.machine.accept(task.task_id, now=now)
        self._emit("charge_created", task.task_id,
                   {"robot": rid, "charger": grant.charger_id,
                    "battery_wh": self.states[rid].battery_wh})
        self.get_logger().info(
            f"{rid} low ({state.battery_wh:.1f} Wh) -> {task.task_id} to "
            f"{grant.charger_id} (x={state.x:.1f}, reachable={list(reachable)})"
        )

    def _assign_ready(self, now: float) -> None:
        for task in self.ledger.open_tasks():
            if task.state is not TaskState.ACCEPTED:
                continue  # NEEDS_ATTENTION is deliberate; only an operator unparks it
            run = self.runs.get(task.task_id)
            if run is None:
                continue

            if run.pinned_robot:
                if run.pinned_robot not in self._eligible_robots(task, now):
                    continue
                chosen = run.pinned_robot
            else:
                allocation = self.allocator.allocate(
                    task, self._eligible_robots(task, now), now=now
                )
                if not allocation.robot_id:
                    self._refuse(allocation.reason)
                    self._note_no_allocation(task.task_id, allocation)
                    continue
                self._no_allocation.pop(task.task_id, None)
                chosen = allocation.robot_id

            if self.states[chosen].executing:
                # Already running something. For a pinned charge task the robot may
                # have been given work between the pad grant and this tick.
                continue
            self.machine.assign(task.task_id, chosen, now=now)
            self.states[chosen].executing = task.task_id
            self._emit("assigned", task.task_id,
                       {"robot": chosen, "pinned": bool(run.pinned_robot)})
            self.get_logger().info(f"{task.task_id} -> {chosen}")

    def _eligible_robots(self, task, now: float) -> dict[str, CoreRobotState]:
        """Robots the allocator may consider, with the extra admissions applied.

        The allocator checks capability and finishability. It does not know a robot
        on a charge pad is unavailable, nor that a low-battery robot should go to a
        pad instead of taking more work -- those rules live here, where the energy
        model is.
        """
        out: dict[str, CoreRobotState] = {}
        charging_task = task.spec.kind.value == "return_to_charge"
        run = self.runs.get(task.task_id)
        first_leg = run.legs[0] if run is not None and run.legs else None
        # Why each robot was refused, per robot, in addition to the running count. A
        # count says "something was refused 2087 times"; this says WHICH gate refused
        # WHICH robot -- and that difference is the whole diagnosis. The 2087 refusals in
        # one charge-queue run were read as a queueing statistic when they were a
        # permanent refusal of a pinned robot by its own gate.
        refusals: dict[str, tuple[ReasonCode, str]] = {}

        def refuse(rid: str, code: ReasonCode, detail: str) -> None:
            self._refuse(code)
            refusals.setdefault(rid, (code, detail))
        # Custody. A robot that already holds a payload may not be given a task that
        # will load a second one. CONTRACTS section 5 forbids handing a payload_id to
        # another robot "as if it moved"; stacking two on one robot is the same mistake
        # with a quieter symptom -- both rows read HELD, the ledger looks consistent, and
        # the first cargo is never delivered. Observed in the 07:57Z run: r02 was parked
        # by a fault with cargo_C, took p5-D, and picked cargo_D.
        carrying = set(self.ledger.payload_holders().values())
        for rid, state in self.states.items():
            if not self._fresh(rid, now) or state.fault_code or rid in self.parked:
                continue
            if not state.position_known:
                # A robot that cannot say where it is cannot be dispatched to. This is
                # load-bearing for a PINNED task specifically: `_assign_ready` decides
                # `run.pinned_robot in self._eligible_robots(...)`, which is the ONLY
                # gate for a system-generated charge run, because the pinned branch never
                # calls `Allocator.candidates` and therefore never reaches its
                # `position_known` check. Without this line the dispatcher offers a leg to
                # a robot with no odometry, the adapter correctly refuses with
                # LOCALIZATION_STALE (CONTRACTS section 3), and the attempt is spent --
                # observed as four charge runs burnt inside the first 155 s of one run.
                refuse(rid, ReasonCode.LOCALIZATION_STALE, "no position reported yet")
                continue
            if state.operating_state == "CHARGING":
                continue
            if task.spec.payload_id and rid in carrying:
                # It is already carrying something. Refusing is the conservative
                # reading of the takeover table, and the alternative is a robot
                # holding two cargos with no rule that ever frees the first.
                refuse(rid, ReasonCode.PAYLOAD_HELD_ELSEWHERE,
                       f"already holding a payload ({sorted(carrying)})")
                continue
            if first_leg is not None:
                # Route connectivity, CONTRACTS section 6 -- as a PREFERENCE, not a refusal.
                #
                # It was a refusal while a crossing could not be driven: the approach leg would
                # have become an untagged crossing, Nav2 cannot route through the barrier, and
                # the task died on a timeout that pointed at navigation instead of at the
                # allocation. Now the dispatcher crosses before issuing a leg -- first legs
                # included -- so refusing here would refuse work the system can do, and it would
                # refuse it BEFORE `_issue_legs` is ever reached, which is where the crossing is
                # started. The whole set is decided below, from the candidates that survived the
                # other gates: a robot that would need a passage is dropped only if some
                # candidate would not.
                pass
            if not charging_task and self.battery.needs_charge(rid, state.battery_wh):
                # Hard admission rule: below the low threshold the next thing this
                # robot does is charge, not carry.
                refuse(rid, ReasonCode.INSUFFICIENT_BATTERY,
                       f"{state.battery_wh:.1f} Wh is at or below the low threshold")
                continue
            out[rid] = state
        # Route connectivity as a preference, applied to the surviving candidates together.
        #
        # Per-robot it cannot be a preference, only a refusal: one robot cannot know whether
        # another could do the job from its own side. So the split happens here, on the set.
        if first_leg is not None and len(out) > 1:
            same_side, needs_passage = split_by_passage(
                {rid: state.x for rid, state in out.items()}, first_leg, self.tcfg)
            if same_side:
                for rid in needs_passage:
                    del out[rid]
                self.crossing_avoided += len(needs_passage)
            elif needs_passage:
                # Nobody can reach the first station without a passage. That is not a reason to
                # leave the task unassigned: it is a crossing, and the dispatcher can drive one.
                self.crossing_admitted += len(needs_passage)
                self.get_logger().info(
                    f"{task.task_id}: every eligible robot needs a corridor passage to reach "
                    f"{first_leg.target_name} ({needs_passage}); admitting one rather than "
                    "leaving the task unassigned")

        pin = run.pinned_robot if run is not None else ""
        if pin and pin in refusals:
            code, detail = refusals[pin]
            self.ineligible[task.task_id] = (pin, code, detail)
        else:
            # Either the pin is eligible, or this is not a pinned task. Both mean the
            # watchdog must not be counting down on it.
            self.ineligible.pop(task.task_id, None)
        return out

    def _issue_legs(self, now: float) -> None:
        for rid, task_id in self._active():
            if rid in self.pending or rid in self.crossings:
                # A crossing IS this robot's live command generation while it runs, so no leg
                # may be issued beside it (CONTRACTS section 4).
                continue
            run = self.runs.get(task_id)
            if run is None:
                # A task from a previous life, or one parked during reconcile.
                self._release_robot(rid)
                continue
            if run.charging_since is not None:
                # Parked on a pad and waiting for the pack to fill. The leg is done;
                # releasing the robot here would also release the pad it is sitting on.
                continue
            if run.cursor >= len(run.legs):
                continue
            leg = run.legs[run.cursor]

            # Cross first if this leg's target is on the other side of the barrier.
            #
            # Before the goal is built, and instead of it: the passage protocol is a sequence
            # of motions and service calls that this node cannot express as one Nav2 goal, and
            # sending one anyway is how a robot ends up pressed against the barrier waiting for
            # a permit. `self.crossed` is consulted so a leg that has already crossed is not
            # crossed again while the robot's own position estimate catches up.
            direction = self._crossing_dir(rid, leg)
            key = self._crossing_key(task_id, leg)
            if direction is not None and key not in self.crossed:
                started = self._start_crossing(rid, task_id, leg, direction, now)
                if started is None:
                    # The driver is not runnable in this environment. Parking with the reason
                    # is the only honest outcome: falling through would drive the barrier.
                    self.machine.fail(
                        task_id, now=now, reason=ReasonCode.NAV_FAILED,
                        detail=f"{leg.leg_id}: the corridor passage driver is not available, "
                               "and this leg's target is across the barrier")
                    self._release_robot(rid, task_id)
                continue

            if not self.leg_clients[rid].wait_for_server(timeout_sec=1.0):
                self.get_logger().warning(f"{rid}: execute_leg action server absent")
                continue
            command_id = f"{task_id}/r{run.attempts}/{leg.leg_id}"
            goal = ExecuteLeg.Goal()
            goal.schema_version = SCHEMA
            goal.task_id = task_id
            goal.assignment_revision = int(self.ledger.get(task_id).revision)
            goal.leg_id = leg.leg_id
            goal.command_id = command_id
            goal.expected_robot_boot_id = ""
            goal.dispatcher_epoch = int(self._epoch)
            goal.goal_station = leg.target_name
            goal.route_segment_id = leg.corridor_direction or ""
            goal.resource_bundle_id = ""
            goal.permit_generation = 0
            goal.timeout_sim_s = float(run.spec.timeout_sim_s) or self.leg_timeout_s
            goal.allowed_retries = int(run.spec.max_attempts)

            if self.ledger.get(task_id).state is TaskState.ASSIGNED:
                self.machine.begin_execution(task_id, rid, now=now)

            future = self.leg_clients[rid].send_goal_async(goal)
            self.pending[rid] = PendingLeg(rid, task_id, leg, command_id, future, now)
            self._emit("leg_issued", task_id,
                       {"robot": rid, "leg": leg.leg_id, "command_id": command_id,
                        "station": leg.target_name, "attempt": run.attempts})

    def _complete_charging(self, now: float) -> None:
        for rid, task_id in self._active():
            run = self.runs.get(task_id)
            if run is None or run.charging_since is None:
                continue
            state = self.states[rid]
            if state.operating_state != "CHARGING" and \
                    self.battery.may_resume_service(rid, state.battery_wh):
                self.machine.succeed(task_id, now=now)
                self._emit("charge_succeeded", task_id,
                           {"robot": rid, "battery_wh": state.battery_wh,
                            "battery_fraction": state.battery_fraction})
                self._release_robot(rid)
                continue
            if now - run.charging_since > CHARGE_TIMEOUT_S:
                self._park(task_id, f"charging did not finish within {CHARGE_TIMEOUT_S:.0f}s",
                           reason=ReasonCode.RESOURCE_TIMEOUT)
                self._release_robot(rid)

    # ------------------------------------------------------------------ #
    # reporting
    # ------------------------------------------------------------------ #

    def snapshot(self) -> dict:
        now = self._now()
        return {
            "schema_version": SCHEMA,
            "epoch": self._epoch,
            "ledger_boot": self.ledger.boot_id,
            "ledger_healthy": self.ledger.healthy,
            "backend": "gazebo_nav2",
            "battery_model": self.battery.snapshot(),
            "payload_mode": (
                "logical: pick and deliver update the ledger only. There is no "
                "mechanical handling in v1 and no physical transfer may be claimed."
            ),
            "robots": {
                rid: {
                    "x": round(s.x, 4), "y": round(s.y, 4), "yaw": round(s.yaw, 4),
                    "battery_wh": round(s.battery_wh, 4),
                    "battery_fraction": round(s.battery_fraction, 4),
                    "band": self.battery.band(rid, s.battery_wh).value,
                    "operating_state": s.operating_state,
                    "task_id": s.executing or "",
                    "fresh": self._fresh(rid, now),
                    "stopped_confirmed": s.stopped_confirmed,
                    "gate_reason": s.gate_reason,
                    "fault_code": s.fault_code,
                    "parked": self.parked.get(rid, ""),
                    "loaded_battery": self.loaded_battery.get(rid),  # round 17, D-P17-02
                    "localization_valid": s.position_known,
                }
                for rid, s in sorted(self.states.items())
            },
            "tasks": [
                {
                    "task_id": t.task_id,
                    "request_id": t.spec.request_id,
                    "kind": t.spec.kind.value,
                    "state": t.state.value,
                    "robot_id": t.robot_id or "",
                    "leg": self._current_leg(t.task_id),
                    "attempts": self.runs[t.task_id].attempts if t.task_id in self.runs else 0,
                    "pinned_robot": self.runs[t.task_id].pinned_robot if t.task_id in self.runs else "",
                    "reason": t.reason.value,
                    "detail": t.detail,
                    "payload_id": t.spec.payload_id or "",
                }
                for t in self.ledger.all_tasks()
            ],
            "payloads": self._payload_rows(),
            "chargers": self.chargers.snapshot(),
            "pending": {rid: item.command_id for rid, item in sorted(self.pending.items())},
            #: Reported because a crossing is the one place this node hands motion to another
            #: process. "The task is EXECUTING" and "the robot is actually moving" are not the
            #: same statement, and the second one is only visible here.
            "crossings": {
                rid: {
                    "task_id": item.task_id,
                    "leg": item.leg.leg_id,
                    "direction": item.direction,
                    "target": item.leg.target_name,
                    "age_s": round(now - item.started_at, 3),
                    "budget_s": self.crossing_timeout_s,
                }
                for rid, item in sorted(self.crossings.items())
            },
            "crossed_legs": sorted(self.crossed),
            "route_connectivity": {
                "preferred_same_side": self.crossing_avoided,
                "admitted_a_passage": self.crossing_admitted,
            },
            "refusals": dict(sorted(self.refusals.items())),
            "stale_results": self.stale_results,
            "ownership_refusals": self.ownership_refusals,
            "orphaned_tasks": sorted(self.orphan_since),
            #: A pinned run that cannot be offered to its own robot, with the refusal that
            #: blocked it. Reported and not only logged: the counter said "2087 refusals"
            #: and nothing about which robot or which gate.
            "ineligible_pinned": {
                task_id: {"robot": robot, "reason": code.value, "detail": detail}
                for task_id, (robot, code, detail) in sorted(self.ineligible.items())
            },
            #: Pinned runs that have been unassignable for a while, so a stalled run can
            #: be watched filling up rather than only read after it has been parked.
            "unassignable_tasks": sorted(self.unassignable_since),
            #: Tasks refused because no robot survived the filter. The per-robot reasons
            #: are in the `no_allocation` event; this is the list of what is waiting.
            "awaiting_allocation": sorted(self._no_allocation),
            #: Non-zero means the dispatcher has been raising. It is reported, not
            #: hidden: a task service that cannot dispatch is not a healthy one.
            "tick_errors": self.tick_errors,
            "reconcile": self.reconcile_report,
        }

    def _current_leg(self, task_id: str) -> str:
        run = self.runs.get(task_id)
        if run is None or run.cursor >= len(run.legs):
            return ""
        return run.legs[run.cursor].leg_id

    def _payload_rows(self) -> list[dict]:
        rows: list[dict] = []
        for task in self.ledger.all_tasks():
            if not task.spec.payload_id:
                continue
            try:
                state, holder = self.ledger.payload_state(task.spec.payload_id)
            except KeyError:
                continue
            rows.append(
                {"payload_id": task.spec.payload_id, "state": state.value,
                 "holder": holder or "", "task_id": task.task_id}
            )
        return rows


def main(argv=None) -> None:
    rclpy.init(args=argv)
    node = TaskService()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        try:
            node.ledger.close()
        except Exception:
            pass
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
