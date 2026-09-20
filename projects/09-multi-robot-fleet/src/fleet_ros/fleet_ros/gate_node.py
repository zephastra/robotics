#!/usr/bin/env python3
"""The safety gate as a ROS node: the only publisher of the robot's cmd_vel.

Policy lives in fleet_adapter.safety_gate. This node does four things and nothing
else:

  1. turns `<ns>/odom` into the AdapterStatus the gate judges,
  2. turns `<ns>/cmd_vel_pre_gate` into the requested velocity it judges,
  3. publishes the gate's verdict on `<ns>/cmd_vel`,
  4. refuses to run if anything else is also publishing `<ns>/cmd_vel`.

Point 4 is the one worth explaining. "The gate is the only publisher" is normally a
convention written in a document, and conventions are not enforced at runtime. Here it
is a check: shortly after start the node asks DDS who else publishes the output topic,
and if anyone does, it latches an emergency stop and says so. A second publisher is
exactly how a recovering planner drives into a corridor another robot has reserved,
and it is invisible in a normal topic echo because both publishers look identical.

Three things this node does NOT do, and says so out loud in its startup log:

  * It does not replace downstream protection. If this process is killed outright it
    cannot publish a stop. gz's DiffDrive `<timeout>` is what covers that case, and
    the physical stop distance is measured separately (NOT_RUN until then).
  * In self_heartbeat mode it does not detect a dead upstream commander. The gate's
    heartbeat check is designed to notice that whoever is commanding it has stopped
    talking. With no fleet core running there is nobody to talk, so this node beats
    for itself. What still protects the robot in that mode is the command input
    timeout and the plant timeout.
  * It does not grant permits. Authorisation is injected. Without one, motion inside
    the protected zone is refused, not merely discouraged.

Topics and services, all relative to the node's namespace:

    SUB  cmd_vel_pre_gate   geometry_msgs/Twist      requested velocity
    SUB  odom               nav_msgs/Odometry        measured state (speed, pose)
    SUB  heartbeat          std_msgs/Empty          optional, from a fleet core
    SUB  fleet/permit       fleet_interfaces/ResourcePermit  mirrored grant
    PUB  cmd_vel            geometry_msgs/Twist      the ONE velocity that moves it
    PUB  gate_state         std_msgs/String          JSON, for the evaluator/logs
    SRV  estop              std_srvs/SetBool         true latches, false clears
    SRV  estop_reset        std_srvs/Trigger         explicit clear
    SRV  gate_report        std_srvs/Trigger         JSON of the stop latency report
"""

from __future__ import annotations

import json
import math
import time
from typing import Any

import rclpy
from geometry_msgs.msg import Twist, TwistStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import Empty, String
from std_srvs.srv import Empty as EmptySrv
from std_srvs.srv import SetBool, Trigger
from geometry_msgs.msg import PoseWithCovarianceStamped

from pathlib import Path

import yaml
from fleet_adapter import (
    AdapterPhase,
    AdapterStatus,
    CrossingZoneGuard,
    GateMode,
    GateReason,
    Limits,
    PermitView,
    SafetyGate,
)
from fleet_core import CrossingManager, validate_traffic_config
from fleet_core.geometry import compose_2d
from fleet_core.pose_source import (
    POSE_SOURCE_LOCALISER,
    POSE_SOURCE_SPAWN_ODOM,
    VALID_SOURCES,
    LocaliserPolicy,
    needs_refresh,
    judge_localiser,
)
from fleet_interfaces.msg import ResourcePermit


def _yaw_from_quaternion(q) -> float:
    """Planar yaw. Roll and pitch are ignored, not silently applied.

    The geofence tests the footprint rectangle, so a missing heading is not a
    cosmetic omission: it changes which part of the map the body is judged to occupy.
    """
    siny = 2.0 * (float(q.w) * float(q.z) + float(q.x) * float(q.y))
    cosy = 1.0 - 2.0 * (float(q.y) * float(q.y) + float(q.z) * float(q.z))
    return math.atan2(siny, cosy)


def _find_repo_root():
    import os
    from pathlib import Path as _Path

    env = os.environ.get("FLEET009_ROOT")
    if env and (_Path(env) / "config" / "resources.yaml").is_file():
        return _Path(env)
    for parent in _Path(__file__).resolve().parents:
        if (parent / "config" / "resources.yaml").is_file():
            return parent
    return None


def _qos(depth: int = 10) -> QoSProfile:
    return QoSProfile(
        reliability=QoSReliabilityPolicy.RELIABLE,
        durability=QoSDurabilityPolicy.VOLATILE,
        history=QoSHistoryPolicy.KEEP_LAST,
        depth=depth,
    )


class GateNode(Node):
    def __init__(self) -> None:
        super().__init__("safety_gate")

        # ---- parameters ---------------------------------------------------- #
        self.declare_parameter("robot_id", "")
        self.declare_parameter("publish_rate_hz", 20.0)
        self.declare_parameter("cmd_vel_in_topic", "cmd_vel_pre_gate")
        self.declare_parameter("cmd_vel_out_topic", "cmd_vel")
        # Message type of the INPUT topic. Not cosmetic: it is the difference between the
        # gate working and the gate being silently deaf.
        #
        # Nav2's velocity chain moved from geometry_msgs/Twist to
        # geometry_msgs/TwistStamped. Both are valid messages and both are called
        # "cmd_vel", so a mismatch is invisible in every name in the graph: the
        # subscriber exists, the publisher exists, no message is delivered, and DDS
        # reports it as a QoS mismatch ("Last incompatible policy: INVALID") rather than
        # as a type error -- which sends you looking at QoS instead of at the type.
        #
        # Measured cost of getting this wrong: with the gate on Twist and
        # collision_monitor publishing TwistStamped, EVERY Nav2 command was dropped.
        # The robot could be driven by our own drivers (which published Twist) and could
        # not be driven by Nav2 at all, and the only visible symptom on the Nav2 side was
        # `controller_server: Failed to make progress`.
        self.declare_parameter("cmd_vel_in_is_stamped", True)
        self.declare_parameter("odom_topic", "odom")
        self.declare_parameter("gate_state_topic", "gate_state")

        self.declare_parameter("max_linear_mps", 0.35)
        self.declare_parameter("max_angular_radps", 0.6)
        self.declare_parameter("max_linear_accel_mps2", 0.5)
        self.declare_parameter("max_angular_accel_radps2", 1.0)
        self.declare_parameter("stop_speed_mps", 0.03)
        self.declare_parameter("stop_angular_radps", 0.05)

        self.declare_parameter("heartbeat_timeout_s", 1.0)
        self.declare_parameter("state_timeout_s", 0.5)
        # No command for this long and the requested velocity is treated as zero. This
        # is the protection that survives when upstream dies silently.
        self.declare_parameter("input_timeout_s", 0.5)

        self.declare_parameter("requires_permit", True)
        self.declare_parameter("wants_protected_zone", False)
        self.declare_parameter("self_heartbeat", True)

        # P4.3 traffic geofence. See the block after SafetyGate(...) below.
        self.declare_parameter("traffic_guard", False)
        self.declare_parameter("traffic_config", "")
        self.declare_parameter("permit_topic", "fleet/permit")
        self.declare_parameter("robot_length_m", 0.60)
        self.declare_parameter("robot_width_m", 0.45)
        self.declare_parameter("pose_timeout_s", 1.0)
        # Which pose this gate is allowed to conclude from. `spawn_odom` composes the spawn pose
        # with raw odometry and its error is a RATE (measured 8.1% of distance travelled, p95
        # 1.725 m on a 21.5 m route); `localiser` uses the localised pose (measured p95 0.348 m,
        # age p95 0.853 s while moving). See fleet_core.pose_source.
        self.declare_parameter("pose_source", POSE_SOURCE_SPAWN_ODOM)
        self.declare_parameter("localiser_topic", "amcl_pose")
        # 2.0 s is 2.3x the measured p95 age while moving and holds a fresh pose on 98.3% of
        # moving samples; 0.10 m is the displacement below which a stale estimate is still true
        # of the robot's position, which is what keeps a stopped robot from freezing the gate.
        self.declare_parameter("localiser_timeout_s", 2.0)
        self.declare_parameter("localiser_move_margin_m", 0.10)
        # Ask the localiser for an estimate without motion, instead of waiting for motion
        # that the refusal is preventing. The NAME is a hint, not a fact: the same control
        # was a topic in older nav2 and a service in this build, so the node looks for any
        # service under its own namespace whose name contains `nomotion` and reports what it
        # found. A wrong name that silently does nothing is the kind of fault that has cost
        # this project several runs.
        self.declare_parameter("nomotion_service_hint", "request_nomotion_update")
        self.declare_parameter("nomotion_refresh_s", 2.0)
        # The unusable-pose warning used to be printed on every tick: 20 Hz for 355 s is
        # ~7100 identical lines, and a log that repeats a fact is a log that hides the next
        # new one. Printed on the reason changing, and then at most every N seconds.
        self.declare_parameter("unusable_log_period_s", 10.0)
        # The origin of this robot's odom frame. Without it the geofence would
        # compare odom coordinates against map rectangles and conclude that a
        # robot sitting at its spawn pose is inside the corridor.
        self.declare_parameter("spawn_x", 0.0)
        self.declare_parameter("spawn_y", 0.0)
        self.declare_parameter("spawn_yaw", 0.0)

        self.declare_parameter("verify_solo_publisher", True)
        self.declare_parameter("solo_check_delay_s", 5.0)
        self.declare_parameter("solo_check_period_s", 5.0)

        p = self.get_parameter
        self.robot_id = str(p("robot_id").value) or self.get_namespace().strip("/") or "unknown"
        rate = float(p("publish_rate_hz").value)
        if rate <= 0.0:
            raise ValueError("publish_rate_hz must be > 0")
        self.dt_nominal = 1.0 / rate

        self.out_topic = str(p("cmd_vel_out_topic").value)
        self.self_heartbeat = bool(p("self_heartbeat").value)
        self.requires_permit = bool(p("requires_permit").value)
        self._wants_protected_zone = bool(p("wants_protected_zone").value)
        self.input_timeout_s = float(p("input_timeout_s").value)

        self.gate = SafetyGate(
            robot_id=self.robot_id,
            limits=Limits(
                max_linear_mps=float(p("max_linear_mps").value),
                max_angular_radps=float(p("max_angular_radps").value),
                max_linear_accel_mps2=float(p("max_linear_accel_mps2").value),
                max_angular_accel_radps2=float(p("max_angular_accel_radps2").value),
                stop_speed_mps=float(p("stop_speed_mps").value),
                stop_angular_radps=float(p("stop_angular_radps").value),
            ),
            heartbeat_timeout_s=float(p("heartbeat_timeout_s").value),
            state_timeout_s=float(p("state_timeout_s").value),
            requires_permit=self.requires_permit,
        )

        # ---- P4.3 geofence ------------------------------------------- #
        # Built unconditionally when the flag is on, and REFUSED rather than
        # degraded when the geometry cannot be loaded. A gate that silently falls
        # back to the caller-supplied `wants_protected_zone` flag is worse than
        # one that does not start: it looks protected and is not.
        # Declared BEFORE the geofence block assigns it. Do not reset this further
        # down: an annotated `= None` placed after the guard is built wipes it while
        # `self.gate.zone` keeps the object, so the geofence goes on judging from its
        # own geometry against an empty permit mirror and refuses every motion as
        # RESOURCE_UNKNOWN, with no log line to say why. That happened twice.
        self._zguard: CrossingZoneGuard | None = None
        #: The `nomotion` service this robot's localiser offers, discovered from the graph.
        self._nomotion_service = ""
        self._nomotion_client = None
        self._nomotion_last_wall: float | None = None
        self._nomotion_requests = 0
        self._nomotion_need = ""
        self._nomotion_missing_logged = False
        #: The last unusable reason, and when it was last printed.
        self._unusable_reason = ""
        self._unusable_log_wall: float | None = None
        #: Read from the parameter, not written inline: the reference used to point at an
        #: attribute nothing assigned, which `check_ros_params.py` reports as a declared
        #: parameter that is never read -- the same shape as a half-applied change.
        self._unusable_log_period_s = float(
            self.get_parameter("unusable_log_period_s").value)
        self.traffic_guard = bool(p("traffic_guard").value)
        if self.traffic_guard:
            cfg_text = str(p("traffic_config").value)
            if cfg_text:
                cfg_path = Path(cfg_text)
            else:
                root = _find_repo_root()
                if root is None:
                    raise RuntimeError(
                        "traffic_guard is on but config/resources.yaml was not "
                        "found. Set FLEET009_ROOT or pass traffic_config:=<path>."
                    )
                cfg_path = root / "config" / "resources.yaml"
            tcfg = validate_traffic_config(
                yaml.safe_load(cfg_path.read_text(encoding="utf-8")))
            self._zguard = CrossingZoneGuard(
                manager=CrossingManager(tcfg),
                robot_id=self.robot_id,
                length_m=float(p("robot_length_m").value),
                width_m=float(p("robot_width_m").value),
                task_id="", boot_id="",
            )
            self.gate.zone = self._zguard
            permit_topic = str(p("permit_topic").value)
            self.create_subscription(
                ResourcePermit, permit_topic, self._on_permit, _qos())
            # Named, resolved, at startup. A mirror subscribed to the wrong topic
            # produces the same log as one subscribed correctly that never receives a
            # grant, and only one of those is a bug.
            self.get_logger().info(
                f"traffic geofence ON: permit mirror subscribed on "
                f"'{self.get_namespace().rstrip('/')}/{permit_topic}' "
                f"(config {cfg_path}); this robot's own geometry is authoritative"
            )

        # ---- state --------------------------------------------------------- #
        # Wall clock, not sim time. A permit must expire while the simulator is
        # frozen, so the gate's clock must be the one that keeps running. Using
        # use_sim_time here would make a paused world pause the watchdog too.
        self._wall_start = time.monotonic()
        self._last_input_wall: float | None = None
        self._requested = (0.0, 0.0)
        self._speed = 0.0
        self._omega = 0.0
        self._pose = (0.0, 0.0, 0.0)
        self._odom_raw: tuple[float, float, float] | None = None
        self._have_odom = False
        # ---- which pose the verdict may rest on ---------------------------------------- #
        self._pose_source = str(p("pose_source").value)
        if self._pose_source not in VALID_SOURCES:
            raise RuntimeError(
                f"pose_source {self._pose_source!r} is not one of {list(VALID_SOURCES)}. "
                "An unknown source is not a default: a gate that silently composes odometry "
                "while the operator believes it is using the localiser is the failure this "
                "parameter exists to prevent.")
        self._localiser_topic = str(p("localiser_topic").value)
        self._localiser_policy = LocaliserPolicy(
            max_age_s=float(p("localiser_timeout_s").value),
            max_moved_m=float(p("localiser_move_margin_m").value),
        )
        self._localiser_pose: tuple[float, float, float] | None = None
        self._localiser_wall: float | None = None
        self._localiser_odom_at_arrival: tuple[float, float, float] | None = None
        self._localiser_seen = 0
        self._localiser_verdict = None
        # AFTER the assignments above, deliberately. The first version of this line sat in the
        # traffic-guard block, which runs earlier, and read `self._pose_source` before it existed:
        # `__init__` raised, the gate node never started, and a whole validation run measured a
        # robot that could not move -- with nothing in the run saying the gate was absent, because
        # a node that never starts publishes nothing to be absent from. There is now a check for
        # the class of fault (`tests/test_p11_pose_source.py`).
        self.get_logger().info(
            f"pose source {self._pose_source}: "
            + (f"the localiser on '{self._localiser_topic}'; "
               f"{self._localiser_policy.describe()}; an unusable localised pose is a REFUSAL, "
               "never a fall back to composed odometry"
               if self._pose_source == POSE_SOURCE_LOCALISER else
               "spawn (+) raw odometry; its error is a RATE (measured 8.1% of distance "
               "travelled), so this is only correct where no localiser exists")
        )
        self._permit: PermitView | None = None
        self._odom_wall: float | None = None
        self._pose_timeout_s = float(p("pose_timeout_s").value)
        self._spawn = (
            float(p("spawn_x").value),
            float(p("spawn_y").value),
            float(p("spawn_yaw").value),
        )
        self._permits_seen = 0
        self._granted_seen = 0
        self._adopted_once = False
        self._solo_checked = False
        # Tripwire for the trap this code fell into: assert the guard built above is
        # still here. A gate that starts with traffic_guard on and no guard silently
        # refuses the whole corridor and reads as a traffic/reservation bug, which is
        # where two runs and most of a day went. Tests/test_p4_gate_wiring.py also
        # pins the declaration order, so a re-introduction fails the suite rather
        # than a simulation run.
        if self.traffic_guard and self._zguard is None:
            raise RuntimeError(
                "traffic_guard is on but `self._zguard is None` during __init__: "
                "something reset it after the guard was built. The permit mirror "
                "would stay empty and every motion would be refused as "
                "RESOURCE_UNKNOWN, with nothing in the log to say why."
            )
        self._solo_violation = False
        self._published_zero = False

        # ---- interfaces ---------------------------------------------------- #
        in_topic = str(p("cmd_vel_in_topic").value)
        self._in_topic_name = in_topic
        self.cmd_in_is_stamped = bool(p("cmd_vel_in_is_stamped").value)
        if self.cmd_in_is_stamped:
            self.create_subscription(
                TwistStamped, in_topic, self._on_cmd_stamped, _qos())
        else:
            self.create_subscription(Twist, in_topic, self._on_cmd, _qos())
        self.create_subscription(
            Odometry, str(p("odom_topic").value), self._on_odom, _qos())
        # Resolved, relative, and named at startup, for the same reason the permit mirror is: a
        # subscriber on the wrong topic and a localiser that never publishes produce the same
        # silence, and only one of them is a bug.
        self.create_subscription(
            PoseWithCovarianceStamped, self._localiser_topic, self._on_localiser, _qos())
        self.create_subscription(Empty, "heartbeat", self._on_heartbeat, _qos())

        self.pub = self.create_publisher(Twist, self.out_topic, _qos())
        self.pub_state = self.create_publisher(String, str(p("gate_state_topic").value), _qos())

        self.create_service(SetBool, "estop", self._srv_estop)
        self.create_service(Trigger, "estop_reset", self._srv_reset)
        self.create_service(Trigger, "gate_report", self._srv_report)

        self.timer = self.create_timer(self.dt_nominal, self._tick)

        if bool(p("verify_solo_publisher").value):
            self.create_timer(
                float(p("solo_check_period_s").value), self._check_solo_publisher)
            self._solo_delay = float(p("solo_check_delay_s").value)

        self._announce_startup()

    # ------------------------------------------------------------------ #
    # startup honesty
    # ------------------------------------------------------------------ #

    def _announce_startup(self) -> None:
        self.get_logger().info(
            f"009 safety gate up for {self.robot_id!r}; publishing {self.out_topic} "
            f"at {1.0 / self.dt_nominal:.0f} Hz, listening on "
            f"{self._in_topic_name} as "
            f"{'TwistStamped' if self.cmd_in_is_stamped else 'Twist'}"
        )
        self.get_logger().info(
            "limits: "
            f"{self.gate.limits.max_linear_mps} m/s, "
            f"{self.gate.limits.max_angular_radps} rad/s, "
            f"accel {self.gate.limits.max_linear_accel_mps2} m/s2 / "
            f"{self.gate.limits.max_angular_accel_radps2} rad/s2  "
            "(frozen by P2 calibration, NOT_YET_CALIBRATED)"
        )

        if self.self_heartbeat:
            self.get_logger().warning(
                "self_heartbeat=true: this node beats for itself, so a DEAD UPSTREAM "
                "COMMANDER IS NOT DETECTED by the heartbeat check. Remaining "
                "protections are the command input timeout "
                f"({self.input_timeout_s}s) and the plant timeout in gz. Any report "
                "about the adapter-death case must be marked NOT_RUN."
            )
        if not self.requires_permit:
            self.get_logger().warning(
                "requires_permit=false: motion is allowed without a corridor permit. "
                "Correct for a single-robot smoke test; INVALID for any run that "
                "claims to exercise the reservation rules."
            )
        self.get_logger().warning(
            "gate exit is NOT safety. If this process is killed it cannot publish a "
            "stop, and the last command may persist downstream (F13). The physical "
            "stop distance is NOT_RUN until measured."
        )

    # ------------------------------------------------------------------ #
    # inputs
    # ------------------------------------------------------------------ #

    def _now(self) -> float:
        """Wall seconds since this node started.

        Monotonic, never sim time: a permit must be able to expire while the
        simulator is paused (CONTRACTS, traffic expiry).
        """
        return time.monotonic() - self._wall_start

    def _on_cmd(self, msg: Twist) -> None:
        self._requested = (float(msg.linear.x), float(msg.angular.z))
        self._last_input_wall = self._now()

    def _on_cmd_stamped(self, msg: TwistStamped) -> None:
        """Accept the stamped form used by this Nav2 generation.

        Only the twist is used. The header stamp is deliberately ignored: this node's
        watchdogs run on the wall clock (see `_now`), so mixing a sim-time stamp into
        them would let a paused simulator extend a permit.
        """
        self._on_cmd(msg.twist)

    def _on_odom(self, msg: Odometry) -> None:
        lin = msg.twist.twist.linear
        ang = msg.twist.twist.angular
        self._speed = math.hypot(float(lin.x), float(lin.y))
        self._omega = float(ang.z)
        # odom frame -> map frame. See fleet_core.geometry.compose_2d for why this
        # is a composition with the spawn pose rather than a TF lookup.
        #
        # The RAW pose is kept as well as the composition, because it is the frame in which the
        # displacement since a localised estimate arrived is measured -- and that displacement,
        # not the message's age, is what decides whether a stale estimate is still true.
        self._odom_raw = (
            float(msg.pose.pose.position.x),
            float(msg.pose.pose.position.y),
            _yaw_from_quaternion(msg.pose.pose.orientation),
        )
        self._pose = compose_2d(self._spawn, self._odom_raw)
        self._have_odom = True
        self._odom_wall = self._now()

        # The first odometry sample after a localised estimate arrived, if the estimate
        # beat odometry to this node. `_on_localiser` captures the odom pose at the
        # estimate's instant, and it captures `None` when odom has not been seen yet --
        # which happened live: AMCL publishes its initial pose the moment it activates,
        # and this node's first odom message came after it. `_moved_since_estimate()`
        # then returned None for ever, so the policy's escape hatch ("the robot has not
        # moved, so a stale estimate still describes it") could never open.
        #
        # That mattered because `amcl_pose` is motion-gated: nav2_params.yaml sets
        # update_min_d (0.10 since D-P17-06; 0.25 before that) and update_min_a: 0.2, so
        # AMCL only publishes when the robot
        # has moved. A refusal that stops the robot therefore stops the localiser and the
        # refusal stays live: measured, `localiser_messages: 1` over a 640 s run and
        # `odom path 0.000 m`.
        #
        # The window this leaves unmeasured is one odometry period -- 20 ms at the
        # recorder's 50 Hz, 7 mm at the 0.35 m/s limit -- which is why it is filled from
        # odom rather than from a guess about where the robot was.
        if self._localiser_wall is not None and self._localiser_odom_at_arrival is None:
            self._localiser_odom_at_arrival = self._odom_raw

    def _on_localiser(self, msg: PoseWithCovarianceStamped) -> None:
        """Record the localised pose, when it arrived, and where odometry was at that moment."""
        p = msg.pose.pose.position
        self._localiser_pose = (
            float(p.x),
            float(p.y),
            _yaw_from_quaternion(msg.pose.pose.orientation),
        )
        self._localiser_wall = self._now()
        self._localiser_odom_at_arrival = self._odom_raw
        self._localiser_seen += 1

    def _moved_since_estimate(self) -> float | None:
        """Displacement in the odom frame since the last localised estimate arrived."""
        if self._localiser_odom_at_arrival is None or self._odom_raw is None:
            return None
        return math.hypot(
            self._odom_raw[0] - self._localiser_odom_at_arrival[0],
            self._odom_raw[1] - self._localiser_odom_at_arrival[1],
        )

    def _trusted_pose(self) -> tuple[tuple[float, float, float], bool, str]:
        """The pose this gate may conclude from, whether it may, and which source it is.

        Three outcomes, and only three:

        * `spawn_odom` configured -> the composed pose, subject to the odometry being fresh. This is
          the previous behaviour, unchanged, and it is what a run without a localiser uses.
        * `localiser` configured and usable -> the localised pose.
        * `localiser` configured and NOT usable -> the composed pose is returned for reporting but
          `usable` is False, so the traffic layer refuses with LOCALIZATION_STALE. The composed pose
          is deliberately NOT substituted: bounding motion with a pose whose error is 8.1% of
          distance travelled is the defect, not the fallback.
        """
        if self._pose_source != POSE_SOURCE_LOCALISER:
            fresh_odom = (self._odom_wall is not None
                          and (self._now() - self._odom_wall) <= self._pose_timeout_s)
            return self._pose, fresh_odom, POSE_SOURCE_SPAWN_ODOM

        verdict = judge_localiser(
            age_s=(None if self._localiser_wall is None
                   else self._now() - self._localiser_wall),
            moved_since_estimate_m=self._moved_since_estimate(),
            policy=self._localiser_policy,
        )
        self._localiser_verdict = verdict
        if verdict.usable and self._localiser_pose is not None:
            self._unusable_reason = ""
            return self._localiser_pose, True, POSE_SOURCE_LOCALISER
        # Once per reason change, then at most every `unusable_log_period_s`. The first line
        # of a repeated fault carries the information; the next seven thousand do not.
        now = self._now()
        if (verdict.reason != self._unusable_reason
                or self._unusable_log_wall is None
                or (now - self._unusable_log_wall) >= self._unusable_log_period_s):
            self._unusable_reason = verdict.reason
            self._unusable_log_wall = now
            self.get_logger().warning(
                f"localised pose unusable, refusing rather than composing odometry: "
                f"{verdict.reason}")
        return self._pose, False, POSE_SOURCE_LOCALISER

    # ------------------------------------------------------------------ #
    # asking the localiser for an estimate without motion
    # ------------------------------------------------------------------ #
    def _find_nomotion_service(self) -> str:
        """The `nomotion` service this robot's localiser actually offers, or "".

        Discovered rather than assumed. This control is a service in this nav2 build and was
        a topic in older ones; a hardcoded name that matches nothing produces silence that
        reads exactly like "the localiser has nothing to say".
        """
        ns = self.get_namespace().rstrip("/")
        prefix = f"{ns}/" if ns and ns != "/" else "/"
        found = [name for name, _types in self.get_service_names_and_types()
                 if "nomotion" in name.lower() and name.startswith(prefix)]
        if not found:
            hint = str(self.get_parameter("nomotion_service_hint").value).lstrip("/")
            return ""
        return sorted(found)[0]

    def _ask_for_an_estimate(self, now: float) -> None:
        """Ask for a no-motion estimate, if the policy says the pose is not fresh."""
        if self._pose_source != POSE_SOURCE_LOCALISER:
            return
        need = needs_refresh(
            age_s=(None if self._localiser_wall is None else now - self._localiser_wall),
            policy=self._localiser_policy,
            since_last_request_s=(None if self._nomotion_last_wall is None
                                  else now - self._nomotion_last_wall),
            min_interval_s=float(self.get_parameter("nomotion_refresh_s").value),
        )
        self._nomotion_need = need.reason
        if not need.needed:
            return
        if not self._nomotion_service:
            # Re-look: the localiser may have activated after this gate did.
            self._nomotion_service = self._find_nomotion_service()
        if not self._nomotion_service:
            self._nomotion_need = (
                "a no-motion update is needed but no `nomotion` service is offered under "
                f"{self.get_namespace()}; the localiser cannot be refreshed while stopped, so "
                "the stale-pose refusal will persist")
            if not self._nomotion_missing_logged:
                self._nomotion_missing_logged = True
                self.get_logger().error(self._nomotion_need)
            return
        if self._nomotion_client is None:
            self._nomotion_client = self.create_client(EmptySrv, self._nomotion_service)
        if not self._nomotion_client.service_is_ready():
            return
        self._nomotion_client.call_async(EmptySrv.Request())
        self._nomotion_last_wall = now
        self._nomotion_requests += 1
        # The first one says what the fix is; every fiftieth says it is still working.
        # Logging ONLY the first made "how many times did the gate ask" unobservable --
        # the count is in the payload, but a log that stops after one line is a log that
        # cannot distinguish "asked once and never needed again" from "asked 300 times".
        if self._nomotion_requests == 1 or self._nomotion_requests % 50 == 0:
            self.get_logger().info(
                f"asked the localiser for an estimate without motion on "
                f"'{self._nomotion_service}' (request {self._nomotion_requests}) -- the "
                f"refusal stops the localiser (`update_min_d` is a motion gate), so waiting "
                f"for motion would wait for ever")

    def _pose_fields(self) -> dict:
        """The pose half of the state payload: both sources, both ages, and the disagreement.

        Kept as one dict so the payload cannot report one source's age beside the other source's
        position -- a shape that reads as a single fact and is two.
        """
        trusted, usable, source = self._trusted_pose()
        disagreement = (None if self._localiser_pose is None
                        else round(math.hypot(self._localiser_pose[0] - self._pose[0],
                                              self._localiser_pose[1] - self._pose[1]), 4))
        moved = self._moved_since_estimate()
        return {
            "pose_source": source,
            "pose_usable": usable,
            "pose_trusted": [round(v, 4) for v in trusted],
            # Reported, never used to decide: the disagreement is the measurement that motivated
            # the switch, and it would be invisible if only the chosen pose were published.
            "localiser": (None if self._localiser_pose is None
                          else [round(v, 4) for v in self._localiser_pose]),
            "localiser_age_s": (None if self._localiser_wall is None
                                else round(self._now() - self._localiser_wall, 3)),
            "localiser_moved_m": None if moved is None else round(moved, 4),
            "localiser_messages": self._localiser_seen,
            "localiser_verdict": (None if self._localiser_verdict is None
                                  else self._localiser_verdict.reason),
            "pose_disagreement_m": disagreement,
            # The refresh half of the same question, reported so a run says whether the loop
            # was broken by asking or was left to sustain itself.
            "nomotion_service": self._nomotion_service or None,
            "nomotion_requests": self._nomotion_requests,
            "nomotion_need": self._nomotion_need or None,
        }

    def _on_heartbeat(self, _msg: Empty) -> None:
        self.gate.heartbeat(wall_now=self._now())

    def _on_permit(self, msg: ResourcePermit) -> None:
        """Adopt a mirrored permit so the local gate judges for itself.

        The gate does not hand its judgement to the coordinator, and the coordinator
        does not hand its judgement to the gate. Both run the same geometry; the
        permit only tells this robot what it was granted.

        ``valid_for_wall_s`` is a DURATION. It becomes a deadline on the clock of
        THIS process at arrival, so a replayed or delayed message cannot inherit a
        longer life than it was given, and the raw monotonic reading of the
        coordinator (meaningless here) is never compared.
        """
        self._permits_seen += 1
        if self._permits_seen == 1:
            # One line, on the first message only. If this never appears, the
            # problem is the subscription/topic; if it appears but the mirror
            # stays empty, the problem is adoption.
            self.get_logger().info(
                f"first permit message received: granted={msg.granted} "
                f"direction={msg.direction!r} permit_id={msg.permit_id!r} "
                f"valid_for_wall_s={msg.valid_for_wall_s:.3f}"
            )
        if self._zguard is None:
            # Counted above, so `permit_messages_seen` still explains a gate whose
            # guard is missing -- those two facts must not collapse into one number.
            return
        if not msg.granted:
            # A refusal carries no authority to adopt. The guard refuses from
            # geometry on its own, so there is nothing to do here.
            return
        # Counted separately from `_permits_seen`: a gate that receives only refusals
        # and a gate that receives grants but cannot adopt them are different bugs,
        # and one number cannot tell them apart.
        self._granted_seen += 1
        wall = self._now()
        expires_at_wall = wall + float(msg.valid_for_wall_s)
        if expires_at_wall <= wall:
            self.get_logger().warning(
                "permit arrived with no remaining life; ignoring it")
            return
        if msg.direction not in self._zguard.manager.cfg.directions:
            self.get_logger().warning(
                f"permit names unknown direction {msg.direction!r}; ignoring")
            return
        try:
            self._zguard.manager.adopt_permit(
                permit_id=msg.permit_id,
                task_id=msg.task_id,
                robot_id=msg.robot_id,
                direction=msg.direction,
                boot_id=msg.boot_id,
                revision=int(msg.revision),
                generation=int(msg.generation),
                epoch=int(msg.epoch),
                expires_at_wall=expires_at_wall,
            )
        except Exception as exc:
            self.get_logger().warning(f"could not adopt permit: {exc}")
            return
        if not self._adopted_once:
            self._adopted_once = True
            self.get_logger().info(
                f"adopted permit {msg.permit_id} for {msg.direction}: the gate now "
                f"holds {list(msg.resources)} until wall "
                f"{expires_at_wall:.2f} (receiver-side deadline)"
            )
        self._zguard.reassign(
            task_id=msg.task_id, boot_id=msg.boot_id, revision=int(msg.revision),
            generation=int(msg.generation), epoch=int(msg.epoch))
        # Keep the legacy PermitView path consistent with what was just adopted, so
        # the two paths cannot disagree about what this robot holds.
        self._permit = PermitView(
            granted=True, resources=tuple(msg.resources), boot_id=msg.boot_id,
            revision=int(msg.revision), generation=int(msg.generation),
            epoch=int(msg.epoch), expires_at_wall=expires_at_wall)

    def _status(self) -> AdapterStatus:
        phase = AdapterPhase.MOVING if self._speed > self.gate.limits.stop_speed_mps \
            else AdapterPhase.STOPPED
        # THE single place the pose and the localisation-validity flag are decided. Everything
        # downstream -- the geofence, the occupancy observation, the published verdict -- reads
        # this, so there is one answer to "where does the gate believe the robot is".
        trusted, usable, source = self._trusted_pose()
        return AdapterStatus(
            robot_id=self.robot_id,
            backend="ros_nav2",
            phase=phase,
            generation=0,
            x=trusted[0],
            y=trusted[1],
            yaw=trusted[2],
            speed_mps=self._speed,
            angular_radps=self._omega,
            message=f"pose_source={source}",
            stopped_confirmed=self._speed <= self.gate.limits.stop_speed_mps,
            arrived=False,
            boot_id=f"gate-{self._wall_start:.3f}",
            epoch=0,
            # Fail closed. With the composed pose that means fresh odometry; with the localiser it
            # means an estimate that still describes where the robot is -- and when it does not,
            # this flag is False and the traffic layer refuses with LOCALIZATION_STALE rather than
            # falling back to a pose whose error is a rate.
            localization_valid=usable,
        )

    # ------------------------------------------------------------------ #
    # the tick
    # ------------------------------------------------------------------ #

    def _tick(self) -> None:
        wall = self._now()

        if self.self_heartbeat:
            # Only beat once the robot state is known: beating earlier would let the
            # gate believe it is healthy while it still has no idea where the robot is.
            if self._have_odom:
                self.gate.heartbeat(wall_now=wall)

        if self._have_odom:
            self.gate.observe(self._status(), wall_now=wall)

        # The refusal stops the localiser, and a stopped localiser sustains the refusal.
        # Asking for a no-motion estimate is what breaks that loop; see pose_source.
        self._ask_for_an_estimate(wall)

        req_lin, req_ang = self._requested
        if self._last_input_wall is None:
            # No command has ever arrived. Zero, and say why once.
            if not self._published_zero:
                self.get_logger().info(
                    "no command received yet; publishing zero until one arrives")
            req_lin = req_ang = 0.0
        elif wall - self._last_input_wall > self.input_timeout_s:
            req_lin = req_ang = 0.0

        decision = self.gate.evaluate(
            req_lin,
            req_ang,
            wall_now=wall,
            epoch=0,
            permit=self._permit,
            state_epoch=None,
            wants_protected_zone=self._wants_protected_zone,
            dt_s=self.dt_nominal,
        )

        msg = Twist()
        msg.linear.x = decision.allowed_linear_mps
        msg.angular.z = decision.allowed_angular_radps
        self.pub.publish(msg)

        if decision.is_zero:
            self._published_zero = True
        self._publish_state(wall, decision)

    def _publish_state(self, wall: float, decision) -> None:
        state = {
            "robot_id": self.robot_id,
            "wall_s": round(wall, 4),
            "mode": decision.mode.value,
            "reason": decision.reason.value,
            "detail": decision.detail,
            "commanded_linear_mps": round(decision.allowed_linear_mps, 5),
            "commanded_angular_radps": round(decision.allowed_angular_radps, 5),
            "measured_speed_mps": round(self._speed, 5),
            "zero_published_at": self.gate.zero_published_at,
            "stopped_confirmed_at": self.gate.stopped_confirmed_at,
            "zero_is_not_stopped": True,
            "physical_stop_distance_m": "NOT_RUN",
            "solo_publisher_violation": self._solo_violation,
            # Permit-path observability. Without these four, a stalled robot is
            # equally consistent with "no grant ever arrived" and "the grant arrived
            # and this gate refused on its own geometry" -- different bugs, and the
            # wrong one gets fixed. `guard_installed` also catches the guard being
            # silently absent, which is what happened here.
            "guard_installed": self._zguard is not None,
            "permit_messages_seen": self._permits_seen,
            "permit_grants_seen": self._granted_seen,
            "permit_adopted": self._adopted_once,
            # The pose this verdict was computed from, and how old it was. Without it a reader
            # has to reconstruct the gate's frame from the recorder, whose odometry has its
            # origin at the spawn pose -- comparing that to a map-frame rectangle translates
            # the whole fleet by one spawn (measured: 6.325 m) and reads as a localisation
            # fault. `spawn` makes the frame self-documenting rather than implied.
            "pose": [round(v, 4) for v in self._pose],
            **self._pose_fields(),
            "pose_age_s": (
                None if self._odom_wall is None else round(self._now() - self._odom_wall, 4)
            ),
            "spawn": [round(float(v), 4) for v in self._spawn],
        }
        self.pub_state.publish(String(data=json.dumps(state)))

    # ------------------------------------------------------------------ #
    # the "gate is the sole publisher" check
    # ------------------------------------------------------------------ #

    def _check_solo_publisher(self) -> None:
        # Once a violation is latched there is nothing left to learn; keep the estop on
        # and stop polling.
        if self._solo_violation:
            return

        topic = self.out_topic_resolved()
        try:
            infos = self.get_publishers_info_by_topic(topic)
        except Exception as exc:  # pragma: no cover - discovery is best effort
            self.get_logger().warning(
                f"could not enumerate publishers of {topic}: {exc}. "
                "The sole-publisher rule is UNVERIFIED for this run."
            )
            self._solo_checked = True
            return

        others = sorted({i.node_name for i in infos if i.node_name != self.get_name()})
        self._solo_checked = True
        if others:
            self._solo_violation = True
            self.gate.latch_estop(GateReason.RESOURCE_UNKNOWN, wall_now=self._now())
            self.get_logger().error(
                f"SOLE PUBLISHER VIOLATION on {topic}: also published by {others}. "
                "Latched ESTOP. Two publishers on cmd_vel is how a recovering planner "
                "bypasses a corridor reservation."
            )
        else:
            self.get_logger().info(
                f"sole publisher check passed on {topic} "
                f"({len(infos)} publisher(s) visible, none of them foreign)"
            )

    def out_topic_resolved(self) -> str:
        ns = self.get_namespace().rstrip("/")
        return f"{ns}/{self.out_topic}" if ns else f"/{self.out_topic}"

    # ------------------------------------------------------------------ #
    # services
    # ------------------------------------------------------------------ #

    def _srv_estop(self, request: SetBool.Request, response: SetBool.Response):
        if request.data:
            self.gate.latch_estop(GateReason.LATCHED_ESTOP, wall_now=self._now())
            response.success = True
            response.message = "estop latched; only an explicit reset clears it"
            self.get_logger().error(response.message)
        else:
            self.gate.reset_estop()
            response.success = True
            response.message = "estop cleared"
            self.get_logger().warning(response.message)
        return response

    def _srv_reset(self, _request: Trigger.Request, response: Trigger.Response):
        self.gate.reset_estop()
        response.success = True
        response.message = "estop cleared by explicit reset"
        self.get_logger().warning(response.message)
        return response

    def _srv_report(self, _request: Trigger.Request, response: Trigger.Response):
        report: dict[str, Any] = dict(self.gate.stop_latency_report())
        report["reason_counts"] = self.gate.reason_counts()
        report["robot_id"] = self.robot_id
        report["decisions_total"] = len(self.gate.decisions)
        response.success = True
        response.message = json.dumps(report)
        return response

    # ------------------------------------------------------------------ #
    # shutdown
    # ------------------------------------------------------------------ #

    def destroy_node(self) -> bool:
        decision = self.gate.shutdown(wall_now=self._now())
        for _ in range(5):
            self.pub.publish(Twist())
        self.get_logger().warning(decision.detail)
        return super().destroy_node()


def main(argv: list[str] | None = None) -> int:
    rclpy.init(args=argv)
    node = GateNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
