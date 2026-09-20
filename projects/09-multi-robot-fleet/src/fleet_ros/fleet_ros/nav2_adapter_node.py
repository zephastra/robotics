"""Per-robot leg executor: one node, one robot, one live command generation.

This is the piece MASTER_PLAN assigns to P3 ("接真实 ExecuteLeg；从 CLI 到账本再到
停靠的完整链路") and which P4 did not need, because the corridor crossing had its
own single-purpose driver. It is the only place in the fleet that calls
``/rXX/navigate_to_pose``.

What it refuses to do, and why each one matters:

  * **It never falls back to a fake backend.** CONTRACTS section 4 item 7: a
    missing action server, a type mismatch or missing localization is a refusal,
    not an excuse to move the robot some other way. A silent downgrade would put
    "simulated" motion into a report that says Gazebo.
  * **It never treats "goal accepted" as arrival, or "cancel accepted" as
    stopped.** `stopped_confirmed` comes from measured odometry being at rest for
    a continuous window -- not from the command being zero, and not from the gate
    having zeroed the command. If odometry goes quiet the answer is False, because
    a lost sensor is not a stopped robot.
  * **It rejects a stale instruction on its own.** dispatcher_epoch, assignment
    revision and expected boot id travel with the command, so a message queued
    across a coordinator restart is refused here rather than acted upon.
  * **A late result from a superseded goal cannot complete the new leg.** It is
    recorded as STALE_RESULT_IGNORED and dropped.

Concurrency, stated once because it is easy to get wrong here: this node runs on a
MultiThreadedExecutor and the leg runs *inside* an action callback, so nothing in
here may call `spin_until_future_complete`. Nesting a spin on a node that an
executor is already spinning is how this kind of node deadlocks -- quietly, and
only under load. Futures are polled with a sleep instead, which is safe precisely
because other executor threads keep servicing odometry, the timer and cancel
requests while this thread waits.

Energy is integrated here because this is the only node that knows how far the
robot actually drove. The model is simulated; see BatteryConfig.
"""

from __future__ import annotations

import json
import math
import os
import threading
import time
from pathlib import Path

import rclpy
import yaml
from fleet_adapter import Nav2Readiness, should_wait_for_recovery
from fleet_core import BatteryModel, ReasonCode, compose_2d, validate_fleet_config
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient, ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String

from fleet_interfaces.action import ExecuteLeg
from fleet_interfaces.msg import RobotState
from fleet_interfaces.srv import FaultInject
from lifecycle_msgs.srv import GetState

SCHEMA = "1"

#: Odometry must be at or below this speed, continuously, before motion may be
#: called stopped. P2 measured that this cart falls below 0.01 m/s within one
#: control period of a zero command, so this is a "has it settled" test rather
#: than a tight threshold pretending to be a measurement.
AT_REST_MPS = 0.02
AT_REST_WINDOW_S = 1.0
ODOM_MAX_AGE_S = 1.5
CANCEL_SETTLE_S = 10.0
#: The label this node puts in `nav_state` (and therefore in the leg result's reason) when
#: its Nav2 executor has not reached ACTIVE. A string rather than the enum because it
#: travels in a message field that predates the reason code, and a reader comparing the two
#: must see the same words.
NAV2_NOT_ACTIVE_LABEL = "NAV2_NOT_ACTIVE"

#: GoalStatus values from action_msgs.
STATUS_SUCCEEDED = 4
STATUS_CANCELED = 5


def _resolve_config(value: str, relative: "Path", label: str) -> str:
    """Find the fleet config from an explicit path, FLEET009_ROOT, or the source tree.

    Same convention as coordinator_node and task_service_node, for the same reason:
    two resolution rules is how two nodes end up disagreeing about which stations and
    which charge pads exist. The source-tree walk works because the workspace is built
    with ``--symlink-install``, so the installed module resolves back into ``src/``.
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
        f"nav2_adapter: cannot locate {relative} ({label}). Set FLEET009_ROOT or pass "
        f"--ros-args -p {label}:=/abs/path"
    )


def _yaw_of(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class Nav2Adapter(Node):
    def __init__(self) -> None:
        super().__init__("nav2_adapter")

        self.declare_parameter("robot", "r01")
        self.declare_parameter("fleet_config", "")
        self.declare_parameter("publish_hz", 10.0)
        self.declare_parameter("leg_timeout_s", 180.0)
        self.declare_parameter("max_retries", 2)
        # The gate publishes on `<ns>/gate_state` -- its own default, and the name four other
        # consumers already use (acceptance_p4.py, probe_odom_truth.py,
        # check_fleet_isolation.py's REQUIRED_TOPICS, stop_distance_calibrator.py). This
        # default used to read "fleet/gate_state" and the launch override agreed with it, so
        # this node subscribed to a topic with no publisher while the gate published to a
        # topic with no subscriber. Nothing errored: a subscription nobody feeds is silent.
        # Measured live: /r01/fleet/gate_state publishers=0 subscribers=2;
        # /r01/gate_state publishers=1 subscribers=0. See docs/DECISIONS.md D-P8-01.
        self.declare_parameter("gate_state_topic", "gate_state")
        self.declare_parameter("start_battery_fraction", 1.0)
        self.declare_parameter("initial_x", 0.0)
        self.declare_parameter("initial_y", 0.0)
        #: How long a leg will wait for its own Nav2 executor to reach ACTIVE before it
        #: reports NAV2_NOT_ACTIVE. Declared rather than hardcoded so a scenario can state
        #: its own patience (CONTRACTS section 9). 30 s is about twice the measured
        #: bring-up (13-16 s) and a sixth of the leg timeout, so a robot that is merely
        #: slow to activate cannot consume the leg's own budget.
        self.declare_parameter("nav2_ready_timeout_s", 30.0)

        self.robot = str(self.get_parameter("robot").value)
        self.leg_timeout_s = float(self.get_parameter("leg_timeout_s").value)
        self.max_retries = int(self.get_parameter("max_retries").value)
        self.nav2_ready_timeout_s = float(
            self.get_parameter("nav2_ready_timeout_s").value
        )

        cfg_path = _resolve_config(
            str(self.get_parameter("fleet_config").value),
            Path("config") / "fleet.yaml", "fleet_config",
        )
        self.cfg = validate_fleet_config(
            yaml.safe_load(Path(cfg_path).read_text(encoding="utf-8"))
        )
        if self.robot not in self.cfg.robots:
            raise RuntimeError(f"nav2_adapter: {self.robot!r} is not in {cfg_path}")
        self.spec = self.cfg.robots[self.robot]
        self.battery = BatteryModel(self.cfg)

        # --- where this robot's odom frame sits in the map ------------------- #
        #
        # Gazebo's odometry origin is the SPAWN POSE, not the map origin, so raw odom
        # reads (0,0) for a robot parked where it started. Everything downstream --
        # station distances, the allocator, the route-connectivity filter -- compares
        # against MAP-frame coordinates, and comparing the two directly made all three
        # robots look like they were standing at the origin: an east-side task went to
        # a west-side robot and vice versa, which then read as a navigation timeout.
        # The spawn pose is composed in once, here, at the source.
        #
        # Read from spawns.yaml rather than taken as a parameter, because spawns.yaml
        # is already the single source of truth for it -- validate_assets.py proves
        # these poses are clear of every pad, wall and charger, and the launch file
        # spawns the robot from the same numbers. A second copy in the launch plumbing
        # is a second thing to drift.
        spawns_path = Path(cfg_path).parent / "spawns.yaml"
        if not spawns_path.is_file():
            raise RuntimeError(
                f"nav2_adapter: {spawns_path} is missing. Without the spawn pose this "
                "node cannot convert odometry into map-frame coordinates, and every "
                "pose it publishes would be measured from the wrong origin."
            )
        spawns = (yaml.safe_load(spawns_path.read_text(encoding="utf-8")) or {}).get("spawns") or {}
        if self.robot not in spawns:
            raise RuntimeError(f"nav2_adapter: {self.robot!r} has no entry in {spawns_path}")
        self._spawn = (
            float(spawns[self.robot].get("x", 0.0)),
            float(spawns[self.robot].get("y", 0.0)),
            float(spawns[self.robot].get("yaw", 0.0)),
        )

        # --- Nav2 action type, imported rather than fabricated -------------- #
        # A hard import failure is reported as a refusal. Treating "Nav2 is not
        # here" as "do nothing and report success" is the exact thing the
        # contract forbids.
        self.nav_type = None
        self.nav_import_error = ""
        try:
            from nav2_msgs.action import NavigateToPose  # type: ignore

            self.nav_type = NavigateToPose
        except ImportError as exc:  # pragma: no cover - environment dependent
            self.nav_import_error = f"{type(exc).__name__}: {exc}"

        # --- state ---------------------------------------------------------- #
        self._x = float(self.get_parameter("initial_x").value)
        self._y = float(self.get_parameter("initial_y").value)
        self._yaw = 0.0
        self._speed = 0.0
        self._ang = 0.0
        self._odom_stamp = None
        self._odom_seen = 0
        self._sequence = 0
        self._boot_id = f"{self.robot}-adapter-{int(time.time())}"
        self._battery_wh = self.spec.battery_capacity_wh * float(
            self.get_parameter("start_battery_fraction").value
        )
        self._payload_id = ""
        self._task_id = ""
        self._leg_id = ""
        self._revision = 0
        self._gate_reason = ""
        self._fault_code = ""
        self._charging_charger = ""
        #: A state-of-charge override injected through FaultInject, held as
        #: (when to undo, what to restore) rather than as a bare mutation. A scenario
        #: needs to put one robot into the low band and then let it recover; a
        #: permanent overwrite would make the recovery half of the scenario
        #: impossible without restarting the whole fleet.
        self._battery_restore_at: float | None = None
        self._battery_restore_wh: float | None = None
        self._last_tick = None
        self._last_xy = (self._x, self._y)
        self._epoch = 0
        self._live_command = ""
        self._done_commands: dict[str, str] = {}
        #: Highest revision seen PER TASK. A single global mark is wrong: revisions are
        #: per-task, so a new task legitimately starts lower than the one before it, and
        #: a global mark refused the first leg of every task after the first.
        self._revision_of: dict[str, int] = {}
        self._at_rest_since = None
        self.cancel_unconfirmed = 0

        group = ReentrantCallbackGroup()
        self.create_subscription(Odometry, "odom", self._on_odom, 20, callback_group=group)
        self.create_subscription(
            String,
            str(self.get_parameter("gate_state_topic").value),
            self._on_gate_state,
            10,
            callback_group=group,
        )
        self.pub_state = self.create_publisher(RobotState, "fleet/state", 10)
        self.create_service(FaultInject, "fleet/inject_fault", self._srv_inject_fault)
        self.server = ActionServer(
            self,
            ExecuteLeg,
            "fleet/execute_leg",
            execute_callback=self._execute,
            callback_group=group,
        )
        self.nav_client = None
        if self.nav_type is not None:
            # Relative name so it resolves under this robot's namespace. A relative
            # name on an unnamespaced node looks for /navigate_to_pose, and the
            # symptom is "action server absent" on a robot whose Nav2 is healthy.
            self.nav_client = ActionClient(
                self, self.nav_type, "navigate_to_pose", callback_group=group
            )

        # Readiness, asked of the lifecycle rather than guessed from the action client.
        # The action server exists while its node is INACTIVE, so the client answers
        # "available" for a node that will refuse every goal -- that is why
        # `wait_for_server` returning true is not evidence, and why the retry loop spent
        # its whole budget on a bring-up race (D-P5-23).
        self.readiness = Nav2Readiness()
        #: Legs that ended in NAV2_NOT_ACTIVE. Reported, because a robot that keeps
        #: waiting for its own executor is a fact about the run and not a private detail.
        self.nav2_not_ready = 0
        #: One ask at a time across three threads: the leg callback, the 10 Hz state
        #: publisher and (in principle) anything else. A second `call_async` would
        #: overwrite the first future and neither answer would ever be harvested.
        self._lc_lock = threading.Lock()
        self._lc_future = None
        self._lc_client = self.create_client(
            GetState, "bt_navigator/get_state", callback_group=group
        )

        hz = max(1.0, float(self.get_parameter("publish_hz").value))
        self.create_timer(1.0 / hz, self._tick)

        self.get_logger().info(
            f"nav2_adapter up for {self.robot}: boot={self._boot_id} "
            f"battery={self._battery_wh:.1f}/{self.spec.battery_capacity_wh:.1f}Wh "
            f"caps={sorted(c.value for c in self.spec.capabilities)} "
            f"nav2={'available' if self.nav_client else 'MISSING ' + self.nav_import_error}"
        )

    # ------------------------------------------------------------------ #
    # inputs
    # ------------------------------------------------------------------ #

    def _on_odom(self, msg: Odometry) -> None:
        pose = msg.pose.pose
        self._x = pose.position.x
        self._y = pose.position.y
        self._yaw = _yaw_of(pose.orientation)
        self._speed = abs(msg.twist.twist.linear.x)
        self._ang = abs(msg.twist.twist.angular.z)
        self._odom_stamp = time.monotonic()
        self._odom_seen += 1

    def _on_gate_state(self, msg: String) -> None:
        """Best-effort diagnostic only.

        The gate's reason is copied into the published state so a report can say
        *why* a robot was held. Nothing here may influence motion: the gate is the
        authority on stopping, and re-deriving its decision from its own log would
        make the log part of the control path.
        """
        try:
            body = json.loads(msg.data)
        except (ValueError, TypeError):
            return
        if isinstance(body, dict):
            self._gate_reason = str(body.get("reason", "") or "")

    # ------------------------------------------------------------------ #
    # state publication
    # ------------------------------------------------------------------ #

    def _localization_valid(self) -> tuple[bool, float]:
        if self._odom_stamp is None:
            return (False, float("inf"))
        age = time.monotonic() - self._odom_stamp
        return (age <= ODOM_MAX_AGE_S, age)

    def _stopped(self) -> bool:
        """Measured rest, not a zero command.

        `_at_rest_since` only advances while odometry is fresh AND slow, so a
        sensor dropout resets the window instead of looking like stillness.
        """
        if not self._localization_valid()[0]:
            return False
        if self._at_rest_since is None:
            return False
        return (time.monotonic() - self._at_rest_since) >= AT_REST_WINDOW_S

    def _operating_state(self) -> str:
        if not self._localization_valid()[0] or self._fault_code:
            return "FAULT"
        if self._charging_charger:
            return "CHARGING"
        if self._live_command:
            return "EXECUTING"
        return "IDLE"

    def _tick(self) -> None:
        now = time.monotonic()
        dt = 0.0 if self._last_tick is None else now - self._last_tick
        self._last_tick = now
        self._expire_battery_injection(now)
        if dt > 0.0:
            travelled = math.dist((self._x, self._y), self._last_xy)
            self._last_xy = (self._x, self._y)
            self._integrate_energy(dt, travelled)

        if self._speed <= AT_REST_MPS and self._localization_valid()[0]:
            if self._at_rest_since is None:
                self._at_rest_since = now
        else:
            self._at_rest_since = None

        self.pub_state.publish(self._build_state())

    def _integrate_energy(self, dt: float, travelled: float) -> None:
        if self._charging_charger:
            step = self.battery.charge(self.robot, charge_wh=self._battery_wh, dt_s=dt)
            self._battery_wh = step.charge_wh
            if self.battery.may_resume_service(self.robot, self._battery_wh):
                self.get_logger().info(
                    f"released {self._charging_charger}: battery at "
                    f"{self.battery.fraction(self.robot, self._battery_wh):.2f} of capacity"
                )
                self._charging_charger = ""
            return
        before = self._battery_wh
        step = self.battery.simulate(
            self.robot, charge_wh=self._battery_wh, dt_s=dt, distance_m=travelled
        )
        if step.clamped_empty and before > 0.0:
            self.get_logger().warning(
                f"battery model clamped to empty: this step demanded "
                f"{step.consumed_wh:.3f} Wh with {before:.3f} Wh remaining. The model "
                "is simulated; treat the depletion time as indicative only."
            )
        self._battery_wh = step.charge_wh

    def _build_state(self) -> RobotState:
        msg = RobotState()
        msg.schema_version = SCHEMA
        msg.robot_id = self.robot
        msg.robot_boot_id = self._boot_id
        self._sequence += 1
        msg.sequence = self._sequence
        sec, nanosec = divmod(int(self.get_clock().now().nanoseconds), 1_000_000_000)
        msg.stamp_sim.sec = sec
        msg.stamp_sim.nanosec = nanosec
        loc_ok, age = self._localization_valid()
        # Publish in the MAP frame, which is what every consumer compares against.
        map_x, map_y, map_yaw = compose_2d(self._spawn, (self._x, self._y, self._yaw))
        msg.x = float(map_x)
        msg.y = float(map_y)
        msg.yaw = float(map_yaw)
        msg.speed_mps = float(self._speed)
        msg.angular_radps = float(self._ang)
        msg.localization_valid = bool(loc_ok)
        msg.pose_age_s = 0.0 if age == float("inf") else float(age)
        msg.operating_state = self._operating_state()
        msg.task_id = self._task_id
        msg.assignment_revision = int(self._revision)
        msg.leg_id = self._leg_id
        msg.capabilities = sorted(c.value for c in self.spec.capabilities)
        msg.battery_fraction = float(self.battery.fraction(self.robot, self._battery_wh))
        msg.battery_wh = float(self._battery_wh)
        msg.battery_valid = True
        msg.payload_id = self._payload_id
        msg.held_resources = []
        msg.permit_generation = 0
        msg.nav_state = self._nav_state_label()
        msg.gate_reason = self._gate_reason
        msg.stopped_confirmed = bool(self._stopped())
        msg.fault_code = self._fault_code
        return msg

    # ------------------------------------------------------------------ #
    # fault injection (the CLI is the only intended caller)
    # ------------------------------------------------------------------ #

    def _srv_inject_fault(self, request: FaultInject.Request, response: FaultInject.Response):
        if request.robot_id and request.robot_id != self.robot:
            response.ok = False
            response.message = f"this adapter is {self.robot!r}, not {request.robot_id!r}"
            return response
        self._fault_code = str(request.fault_code)
        if self._fault_code:
            self.get_logger().warning(
                f"FAULT INJECTED: {self._fault_code} (duration {request.duration_s}s)"
            )
        else:
            self.get_logger().info("fault cleared")
        soc_note = self._apply_battery_injection(request)
        response.ok = True
        response.message = f"{self.robot}: fault_code={self._fault_code!r}{soc_note}"
        return response

    def _apply_battery_injection(self, request: FaultInject.Request) -> str:
        """Set the state of charge from a FaultInject request. Returns a note.

        This exists because P5's low-battery and charge-queue scenarios could not be
        run at all without it: the simulated pack spends 0.5 Wh/m, so reaching the
        20% threshold naturally needs about 160 m of driving -- roughly thirty legs
        and half an hour of wall clock per run. Injecting the STATE is not the same
        as faking the DECISION: the low/critical bands, the allocator's
        finishability test and the charge admission all read the same BatteryModel
        afterwards, so the policy under test is the shipped one.

        ``inject_battery`` is a separate flag rather than a negative sentinel in
        ``battery_fraction``, so a caller that forgets to set the fraction cannot
        empty the pack by accident.
        """
        if not request.inject_battery:
            return ""
        capacity = self.spec.battery_capacity_wh
        previous = self._battery_wh
        fraction = max(0.0, min(1.0, float(request.battery_fraction)))
        self._battery_wh = fraction * capacity
        if request.duration_s > 0.0:
            self._battery_restore_at = time.monotonic() + float(request.duration_s)
            self._battery_restore_wh = previous
            restore_note = (
                f"; restored to {previous:.3f} Wh after {request.duration_s:.1f} s"
            )
        else:
            self._battery_restore_at = None
            self._battery_restore_wh = None
            restore_note = "; no restore, this lasts for the session"
        note = (
            f" battery={self._battery_wh:.3f}Wh"
            f" ({self.battery.fraction(self.robot, self._battery_wh):.3f} of "
            f"{capacity:.1f}Wh, band "
            f"{self.battery.band(self.robot, self._battery_wh).value})"
            + restore_note
        )
        self.get_logger().warning("BATTERY INJECTED:" + note)
        return note

    def _expire_battery_injection(self, now: float) -> None:
        """Undo a timed injection. Logged, because a state that changes by itself
        without a line in the log is indistinguishable from a bug."""
        if self._battery_restore_at is None or now < self._battery_restore_at:
            return
        restored = self._battery_restore_wh
        self._battery_wh = 0.0 if restored is None else float(restored)
        self.get_logger().info(
            f"battery injection expired: charge restored to {self._battery_wh:.3f} Wh "
            f"({self.battery.fraction(self.robot, self._battery_wh):.3f} of capacity)"
        )
        self._battery_restore_at = None
        self._battery_restore_wh = None

    # ------------------------------------------------------------------ #
    # the leg
    # ------------------------------------------------------------------ #

    def _await(self, future, timeout_s: float):
        """Poll a future without nesting a spin. Returns None on timeout."""
        deadline = time.monotonic() + timeout_s
        while not future.done():
            if time.monotonic() > deadline:
                return None
            time.sleep(0.05)
        try:
            return future.result()
        except Exception:  # a failed future is a refusal, not a crash
            return None

    # ------------------------------------------------------------------ #
    # Nav2 readiness (D-P5-23)
    # ------------------------------------------------------------------ #

    def _nav2_ready(self) -> "bool | None":
        """Is this robot's `bt_navigator` ACTIVE? True / False / None (= cannot tell).

        Two-phase on purpose. `_execute` runs inside an action callback on a
        MultiThreadedExecutor, and the response to a service request is delivered by that
        same executor -- so blocking on the future here would be the nested-wait deadlock
        this node's own docstring warns about, one layer down. Issue, return, harvest
        later.
        """
        with self._lc_lock:
            if self._lc_future is not None:
                if not self._lc_future.done():
                    return self.readiness.active()
                try:
                    self.readiness.record(int(self._lc_future.result().current_state.id))
                except Exception:
                    # A failed ask is not evidence of a stopped executor: record "no
                    # longer known", which every caller reads as "carry on".
                    self.readiness.record(None)
                self._lc_future = None
                return self.readiness.active()

            if not self.readiness.due():
                return self.readiness.active()
            if not self._lc_client.service_is_ready():
                # Nobody to ask. Keep the last answer, note that we tried, and let the
                # leg proceed: a missing lifecycle service must not become a stopped
                # fleet.
                self.readiness.asked_without_an_answer()
                return self.readiness.active()

            self.readiness.asks += 1
            self._lc_future = self._lc_client.call_async(GetState.Request())
            return self.readiness.active()

    def _wait_for_nav2_ready(self, deadline: float, handle) -> bool:
        """Wait, bounded, for ACTIVE. Returns whether the leg may proceed.

        An unknown answer does NOT wait: the only value that can hold a leg back is an
        explicit `False`. Waiting is also interruptible, because a cancel has to be able
        to end a leg that is waiting rather than driving.
        """
        while True:
            if self._nav2_ready() is not False:
                return True
            if handle.is_cancel_requested or time.monotonic() >= deadline:
                return False
            time.sleep(0.1)

    def _nav_state_label(self) -> str:
        """This adapter's own view of Nav2, for the state message and the leg result.

        The field used to say only whether the action CLIENT existed, which is true while
        the server's node is still INACTIVE. So the one field that exists to distinguish
        "Nav2 is up and refusing" from "Nav2 gave up" said nothing about the case that
        actually happened.
        """
        if self.nav_client is None:
            return "NO_NAV2_ACTION_SERVER"
        if self._nav2_ready() is False:
            return NAV2_NOT_ACTIVE_LABEL
        return ""

    def _reject(self, handle, reason: ReasonCode, message: str) -> ExecuteLeg.Result:
        result = ExecuteLeg.Result()
        result.ok = False
        result.terminal_state = "REJECTED"
        result.reason_code = reason.value
        result.message = message
        result.nav_state = self._nav_state_label()
        result.stopped_confirmed = bool(self._stopped())
        self.get_logger().warning(f"leg refused ({reason.value}): {message}")
        handle.abort()
        return result

    def _execute(self, handle) -> ExecuteLeg.Result:
        goal = handle.request

        # A different coordinator generation, not a later one. The epoch is a random
        # per-process value, so comparing magnitudes is meaningless: a restarted
        # coordinator would be refused for having the smaller number. What it is good
        # for is telling generations apart -- and when it changes, the revision marks
        # recorded under the old one do not apply to the new one.
        if int(goal.dispatcher_epoch) != self._epoch:
            self.get_logger().info(
                f"dispatcher epoch {self._epoch} -> {goal.dispatcher_epoch}: clearing "
                "per-task revision marks"
            )
            self._epoch = int(goal.dispatcher_epoch)
            self._revision_of.clear()

        if goal.expected_robot_boot_id and goal.expected_robot_boot_id != self._boot_id:
            return self._reject(
                handle,
                ReasonCode.PERMIT_STALE,
                f"command targets boot {goal.expected_robot_boot_id!r}; this adapter is "
                f"{self._boot_id!r}",
            )
        seen = self._revision_of.get(goal.task_id, 0)
        if int(goal.assignment_revision) < seen:
            return self._reject(
                handle,
                ReasonCode.PERMIT_STALE,
                f"task {goal.task_id!r}: assignment revision "
                f"{goal.assignment_revision} is older than {seen} already seen for THIS "
                "task",
            )
        if goal.command_id and goal.command_id in self._done_commands:
            # Already handled. Re-sending would move the robot twice for one
            # command, which is the definition of a non-idempotent retry.
            previous = self._done_commands[goal.command_id]
            result = ExecuteLeg.Result()
            result.ok = previous == ReasonCode.OK.value
            result.terminal_state = "REPLAYED"
            result.reason_code = previous
            result.message = "this command_id was already handled; no new goal sent"
            result.stopped_confirmed = bool(self._stopped())
            handle.succeed()
            return result
        if not self._localization_valid()[0]:
            return self._reject(
                handle,
                ReasonCode.LOCALIZATION_STALE,
                "no fresh odometry: refusing to start a leg on stale localization "
                "(CONTRACTS section 3)",
            )
        if self.nav_client is None:
            return self._reject(
                handle,
                ReasonCode.NAV_FAILED,
                "the navigate_to_pose action type is unavailable "
                f"({self.nav_import_error}); refusing rather than falling back to a "
                "non-Nav2 motion path",
            )

        target = self._resolve_target(goal)
        if target is None:
            return self._reject(
                handle,
                ReasonCode.UNKNOWN_STATION,
                f"goal_station {goal.goal_station!r} has no pose in the fleet config",
            )

        self._live_command = goal.command_id
        self._task_id = goal.task_id
        self._leg_id = goal.leg_id
        self._revision = int(goal.assignment_revision)
        self._revision_of[goal.task_id] = max(seen, int(goal.assignment_revision))
        reason = ReasonCode.OK
        nav_state = ""
        detail = ""
        t0 = time.monotonic()
        stopped = False

        try:
            if not self.nav_client.wait_for_server(timeout_sec=30.0):
                return self._reject(
                    handle,
                    ReasonCode.NAV_FAILED,
                    "navigate_to_pose action server absent for this robot",
                )
            ok, nav_state, detail = self._drive(handle, target, t0)
            if ok:
                reason = ReasonCode.OK
            elif nav_state == NAV2_NOT_ACTIVE_LABEL:
                # Not a navigation failure. "Nav2 gave up" and "Nav2 is not up yet" need
                # opposite responses, and the dispatcher does not spend an attempt on the
                # second (see WAIT_NOT_A_FAILURE in the task service).
                reason = ReasonCode.NAV2_NOT_ACTIVE
            else:
                reason = ReasonCode.NAV_FAILED
            stopped = self._wait_for_rest()
        except Exception as exc:  # a failure must land inside the result
            reason = ReasonCode.NAV_FAILED
            nav_state = "EXCEPTION"
            detail = f"{type(exc).__name__}: {exc}"
            self.get_logger().error(f"leg {goal.command_id} raised: {detail}")
        finally:
            self._live_command = ""
            self._done_commands[goal.command_id] = reason.value
            if goal.goal_station in self.cfg.chargers and reason is ReasonCode.OK:
                if self.battery.may_resume_service(self.robot, self._battery_wh):
                    self.get_logger().info(
                        f"parked on {goal.goal_station} but already above the resume "
                        "threshold; not entering CHARGING"
                    )
                else:
                    self._charging_charger = goal.goal_station
                    self.get_logger().info(
                        f"parked on {goal.goal_station}: simulated charging until "
                        f"{self.cfg.battery.resume_fraction:.2f} of capacity"
                    )

        result = ExecuteLeg.Result()
        result.ok = reason is ReasonCode.OK
        result.terminal_state = "SUCCEEDED" if result.ok else "ABORTED"
        result.reason_code = reason.value
        result.message = detail
        # The map-frame pose is computed HERE rather than borrowed from _build_state:
        # that method's locals are not in scope, and reaching for them raised NameError
        # on every completed leg. The symptom was an empty NAV_FAILED about a second
        # after the goal was sent -- which reads as a dead executor, not as a mistake in
        # this one line, and cost two runs to find.
        map_x, map_y, map_yaw = compose_2d(self._spawn, (self._x, self._y, self._yaw))
        result.final_state_json = json.dumps(
            {
                # Report the map-frame pose and SAY so. The raw odometry is measured
                # from the spawn pose, so a bare "x" here would be a number that looks
                # like a map coordinate and is not one.
                "frame_id": "map",
                "x": round(map_x, 4),
                "y": round(map_y, 4),
                "yaw": round(map_yaw, 4),
                "odom_x": round(self._x, 4),
                "odom_y": round(self._y, 4),
                "spawn": [round(v, 4) for v in self._spawn],
                "battery_wh": round(self._battery_wh, 4),
                "battery_fraction": round(
                    self.battery.fraction(self.robot, self._battery_wh), 4
                ),
                "backend": "gazebo_nav2",
                "battery_model": "simulated",
            },
            sort_keys=True,
        )
        result.elapsed_s = float(time.monotonic() - t0)
        result.nav_state = nav_state
        result.stopped_confirmed = bool(stopped)
        if result.ok:
            handle.succeed()
        else:
            handle.abort()
        return result

    def _resolve_target(self, goal) -> tuple[float, float, float] | None:
        """Resolve a station name to a pose, or None.

        The pose deliberately comes from trusted config rather than from the
        command's explicit x/y. CONTRACTS section 4: an ordinary task submitter
        must not be able to pass execution parameters, and honouring a raw pose
        here would be exactly that while looking harmless.
        """
        if goal.goal_station:
            if goal.goal_station in self.cfg.stations:
                s = self.cfg.stations[goal.goal_station]
                return (s.x, s.y, s.yaw)
            return None
        if goal.goal_x or goal.goal_y or goal.goal_yaw:
            return (goal.goal_x, goal.goal_y, goal.goal_yaw)
        return None

    def _drive(self, handle, target, t0) -> tuple[bool, str, str]:
        """Send one goal, wait for its RESULT, and translate what happened."""
        timeout = float(handle.request.timeout_sim_s) or self.leg_timeout_s
        # Wait for the executor BEFORE spending an attempt. A charge run is created on the
        # task service's first tick, 13-16 s before Nav2 reaches ACTIVE, and a goal sent
        # into an inactive executor is answered `accepted=False` -- which the loop below
        # used to spend its entire budget on.
        ready_deadline = time.monotonic() + min(self.nav2_ready_timeout_s, timeout)
        if not self._wait_for_nav2_ready(ready_deadline, handle):
            self.nav2_not_ready += 1
            return (
                False,
                NAV2_NOT_ACTIVE_LABEL,
                f"bt_navigator was not ACTIVE within "
                f"{min(self.nav2_ready_timeout_s, timeout):.0f}s; nothing was attempted, "
                "so the dispatcher may re-ask without spending an attempt",
            )
        attempt = 0
        while attempt < self.max_retries:
            attempt += 1
            nav_goal = self.nav_type.Goal()
            nav_goal.pose.header.frame_id = "map"
            # Zero stamp: Nav2 runs on /clock and this node may not. A wall-clock
            # stamp on a sim-time goal makes AMCL treat it as being in the future,
            # and the symptom is an unexplained extrapolation abort.
            nav_goal.pose.header.stamp.sec = 0
            nav_goal.pose.header.stamp.nanosec = 0
            nav_goal.pose.pose.position.x = float(target[0])
            nav_goal.pose.pose.position.y = float(target[1])
            nav_goal.pose.pose.orientation.z = float(math.sin(target[2] / 2.0))
            nav_goal.pose.pose.orientation.w = float(math.cos(target[2] / 2.0))

            nav_handle = self._await(self.nav_client.send_goal_async(nav_goal), 10.0)
            if nav_handle is None or not getattr(nav_handle, "accepted", False):
                # "another navigator is still processing one" is transient, not a
                # verdict. Retrying costs nothing; treating it as a refusal once
                # cost an entire P4 acceptance run.
                #
                # From here "the executor is not ACTIVE" looks identical to "the executor
                # is busy" -- so it is ASKED about instead of assumed, and a refusal
                # explained by readiness is a wait: the attempt is given back.
                if should_wait_for_recovery(self.readiness.active()):
                    if not self._wait_for_nav2_ready(ready_deadline, handle):
                        self.nav2_not_ready += 1
                        return (
                            False,
                            NAV2_NOT_ACTIVE_LABEL,
                            "bt_navigator went back to inactive (or never finished "
                            "activating) and the goal was refused; no attempt spent",
                        )
                    attempt -= 1
                    continue
                self.get_logger().warning(f"goal not accepted (attempt {attempt})")
                time.sleep(1.0)
                continue

            result_future = nav_handle.get_result_async()
            deadline = time.monotonic() + timeout
            while not result_future.done():
                if handle.is_cancel_requested:
                    nav_handle.cancel_goal_async()
                    settled = self._wait_for_rest(CANCEL_SETTLE_S)
                    if not settled:
                        self.cancel_unconfirmed += 1
                        self.get_logger().error(
                            "CANCEL_UNCONFIRMED: cancel requested but measured speed "
                            "never settled; the payload and any held resource stay with "
                            "this task (CONTRACTS section 4 item 6)"
                        )
                    handle.canceled()
                    return (
                        False,
                        "CANCELED",
                        "cancel requested; see stopped_confirmed in the result",
                    )
                if time.monotonic() > deadline:
                    nav_handle.cancel_goal_async()
                    return (False, "TIMEOUT", f"leg exceeded {timeout:.0f}s")
                time.sleep(0.1)

            wrapped = result_future.result()
            status = getattr(wrapped, "status", None)
            if status not in (STATUS_SUCCEEDED, STATUS_CANCELED):
                if attempt < self.max_retries:
                    self.get_logger().warning(
                        f"nav2 status={status} on attempt {attempt}; retrying"
                    )
                    time.sleep(1.0)
                    continue
                return (False, f"STATUS_{status}", f"nav2 status={status}")

            # A result that arrives after this command was superseded must not
            # complete the newer leg. That is the contract's STALE_RESULT_IGNORED,
            # and it is why `_live_command` is checked here.
            if self._live_command != handle.request.command_id:
                self.get_logger().warning(
                    f"STALE_RESULT_IGNORED: result for {handle.request.command_id} "
                    f"arrived while {self._live_command!r} was live"
                )
                return (
                    False,
                    "STALE_RESULT_IGNORED",
                    "a superseded command's result was discarded",
                )
            return (True, "SUCCEEDED", f"reached {handle.request.goal_station}")

        return (False, "RETRIES_EXHAUSTED", f"{self.max_retries} attempts rejected")

    def _wait_for_rest(self, timeout_s: float = CANCEL_SETTLE_S) -> bool:
        """Wait for measured rest. Returns whether it actually happened."""
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            if self._stopped():
                return True
            time.sleep(0.1)
        return False


def main(argv=None) -> None:
    rclpy.init(args=argv)
    node = Nav2Adapter()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
