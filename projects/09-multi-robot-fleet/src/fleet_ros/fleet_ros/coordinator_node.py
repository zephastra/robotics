#!/usr/bin/env python3
"""The fleet coordinator as a ROS node: the only grantor of passage permits.

Policy lives in ``fleet_core.traffic``. This node does five things:

  1. owns the single ``CrossingManager`` -- the reservation book;
  2. observes each robot's pose from that robot's own odometry;
  3. runs the start-up occupancy sweep before any resource may be granted;
  4. serves ``/fleet/acquire_passage``, ``/fleet/renew_permit`` and
     ``/fleet/confirm_clear``;
  5. publishes each robot's current permit so that robot's gate can re-derive the
     same decision locally.

Three things it deliberately does NOT do, each for a specific reason:

* **It does not trust the pose in a request.** The pose fields on the services are
  diagnostic. Every judgement uses the pose this node observed from ``/<robot>/odom``.
  A supervised party that could assert its own position could assert its way past the
  entry precondition -- which is the same class of hole as the caller-supplied
  ``wants_protected_zone`` flag that P4.3 removed from the gate.

* **It does not publish cmd_vel.** No central component may move a robot
  (MASTER_PLAN section 5). It grants permission; the robot's own gate decides what
  is actually sent to the wheels.

* **It does not free a resource on timeout.** ``expire_due`` drops a lapsed resource
  to UNKNOWN, and UNKNOWN stays blocked until an explicit geometric clearance. That
  is the rule that keeps two robots out of one corridor, and this node is where it
  would be easiest to "fix" wrongly.

Clock discipline (CONTRACTS section 9): permits and heartbeats run on this process's
**wall** clock, so pausing the simulator does not extend a lease. Sim time is used
only for logging. ``use_sim_time`` must therefore stay false here.
"""

from __future__ import annotations

import json
import math
import time
from dataclasses import dataclass, field
from pathlib import Path

import rclpy
import yaml
from fleet_core import (
    CrossingManager,
    ReasonCode,
    ResourceState,
    validate_fleet_config,
    validate_traffic_config,
)
from fleet_core.geometry import compose_2d, footprint_corners
from fleet_core.pose_source import (
    POSE_SOURCE_SPAWN_ODOM,
    LocaliserPolicy,
    PoseSource,
)
from geometry_msgs.msg import PoseWithCovarianceStamped
from fleet_interfaces.msg import ResourcePermit
from fleet_interfaces.srv import AcquirePassage, ConfirmClear, RenewPermit
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from std_srvs.srv import Trigger


def _qos(depth: int = 10) -> QoSProfile:
    return QoSProfile(
        reliability=QoSReliabilityPolicy.RELIABLE,
        durability=QoSDurabilityPolicy.VOLATILE,
        history=QoSHistoryPolicy.KEEP_LAST,
        depth=depth,
    )


def _yaw_from_quaternion(q) -> float:
    """Planar yaw from a quaternion. Roll/pitch are ignored, not silently applied."""
    siny = 2.0 * (float(q.w) * float(q.z) + float(q.x) * float(q.y))
    cosy = 1.0 - 2.0 * (float(q.y) * float(q.y) + float(q.z) * float(q.z))
    return math.atan2(siny, cosy)


def _repo_root() -> Path | None:
    import os

    env = os.environ.get("FLEET009_ROOT")
    if env and (Path(env) / "config" / "resources.yaml").is_file():
        return Path(env)
    for parent in Path(__file__).resolve().parents:
        if (parent / "config" / "resources.yaml").is_file():
            return parent
    return None


@dataclass
class RobotLink:
    """Everything the coordinator knows about one robot."""

    robot_id: str
    length_m: float
    width_m: float
    # Where this robot was spawned == the origin of its odom frame. Rotating and
    # translating by it is what turns an odom pose into a map pose.
    spawn: tuple[float, float, float] = (0.0, 0.0, 0.0)
    # The composed observation, kept for REPORTING. It is deliberately not what this node
    # judges with: that is `source` below, so there is one place to change the answer and one
    # place to read it.
    pose: tuple[float, float, float] | None = None
    pose_wall: float | None = None
    ever_seen: bool = False
    # Which pose this node may conclude from, and when it may not. One per robot.
    source: "PoseSource | None" = None
    # The current leg. Set when a passage is granted; the gate scope travels with it.
    task_id: str = ""
    boot_id: str = ""
    revision: int = 0
    generation: int = 0
    epoch: int = 0
    direction: str = ""
    permit_id: str = ""
    last_reason: str = ""
    last_detail: str = ""

    def pose_age_s(self, wall_now: float) -> float | None:
        if self.pose_wall is None:
            return None
        return wall_now - self.pose_wall


@dataclass
class RequestLog:
    """A bounded record of what was asked and what was decided, for the report."""

    entries: list[dict] = field(default_factory=list)
    limit: int = 500

    def add(self, **kwargs) -> None:
        self.entries.append(kwargs)
        if len(self.entries) > self.limit:
            del self.entries[: len(self.entries) - self.limit]


class FleetCoordinator(Node):
    def __init__(self) -> None:
        super().__init__("fleet_coordinator")

        self.declare_parameter("robots", [""])
        self.declare_parameter("traffic_config", "")
        self.declare_parameter("fleet_config", "")
        self.declare_parameter("permit_topic", "fleet/permit")
        self.declare_parameter("permit_rate_hz", 5.0)
        self.declare_parameter("pose_timeout_s", 1.5)
        # Which pose this node may JUDGE from. `spawn_odom` composes the spawn pose with raw
        # odometry and its error is a RATE: measured on N01 (2026-09-17) at 1.5516 m median /
        # 2.6825 m p95, while `at_node`'s tolerance is 0.80 m -- so the asking-point precondition
        # could not be satisfied once odometry had accumulated, and the robot that Nav2 had
        # delivered to the node was refused for not being there (`D-P11-03`). `localiser` uses
        # the localised pose (0.1866 m median on the same run). See fleet_core.pose_source.
        #
        # This does NOT weaken CONTRACTS section 7. The coordinator still judges from a pose it
        # observed ITSELF on `/{robot}/amcl_pose`; the pose in a request stays diagnostic.
        self.declare_parameter("pose_source", POSE_SOURCE_SPAWN_ODOM)
        self.declare_parameter("localiser_topic", "amcl_pose")
        self.declare_parameter("localiser_timeout_s", 2.0)
        self.declare_parameter("localiser_move_margin_m", 0.10)
        # How long to wait after the last robot first reports before deciding the
        # initial occupancy of every resource.
        self.declare_parameter("occupancy_settle_s", 5.0)
        self.declare_parameter("renew_margin_s", 0.0)

        p = self.get_parameter
        root = _repo_root()
        if root is None and not str(p("traffic_config").value):
            raise RuntimeError(
                "cannot locate config/resources.yaml. Set FLEET009_ROOT or pass "
                "traffic_config:=<path>. Refusing to start without the traffic geometry: "
                "a coordinator with no corridor definition would grant everything."
            )

        traffic_path = Path(str(p("traffic_config").value)) if str(p("traffic_config").value) \
            else root / "config" / "resources.yaml"
        fleet_path = Path(str(p("fleet_config").value)) if str(p("fleet_config").value) \
            else root / "config" / "fleet.yaml"

        self.tcfg = validate_traffic_config(yaml.safe_load(traffic_path.read_text(encoding="utf-8")))
        self.fcfg = validate_fleet_config(yaml.safe_load(fleet_path.read_text(encoding="utf-8")))
        self.manager = CrossingManager(self.tcfg)

        names = [r for r in (p("robots").value or []) if r]
        # The spawn table is read even when `robots` is given on the command line,
        # because it is also the origin of each robot's odometry frame. Reading it
        # only as a name fallback is how the frame offset got lost.
        self._spawn_table: dict[str, dict] = {}
        _spawns_path = root / "config" / "spawns.yaml" if root else None
        if _spawns_path and _spawns_path.is_file():
            self._spawn_table = (
                yaml.safe_load(_spawns_path.read_text(encoding="utf-8")) or {}
            ).get("spawns", {}) or {}
        if not names:
            names = sorted(self._spawn_table)
        if not names:
            raise RuntimeError("no robots: pass robots:=['r01','r02'] or provide config/spawns.yaml")
        missing = [r for r in names if r not in self.fcfg.robots]
        if missing:
            raise RuntimeError(f"robots {missing} are not in {fleet_path}")

        self.robots: dict[str, RobotLink] = {}
        for r in names:
            spec = self.fcfg.robots[r]
            spawn = self._spawn_of(r)
            self.robots[r] = RobotLink(
                robot_id=r,
                length_m=spec.footprint_length_m,
                width_m=spec.footprint_width_m,
                spawn=spawn,
                # Built here rather than lazily on the first message: a source that appears when
                # the first odometry arrives is a source that is absent exactly when the first
                # judgement is made. An unknown `pose_source` raises here, at start-up.
                source=PoseSource(
                    source=str(p("pose_source").value),
                    policy=LocaliserPolicy(
                        max_age_s=float(p("localiser_timeout_s").value),
                        max_moved_m=float(p("localiser_move_margin_m").value),
                    ),
                    spawn=spawn,
                    odom_timeout_s=float(p("pose_timeout_s").value),
                ),
            )

        self.pose_timeout_s = float(p("pose_timeout_s").value)
        self.occupancy_settle_s = float(p("occupancy_settle_s").value)
        self.renew_margin_s = float(p("renew_margin_s").value)
        self._wall_start = time.monotonic()
        self._sweep_done = False
        # How many times UNKNOWN state was lifted on fresh poses. Reported so a
        # run cannot silently depend on it without saying so.
        self._reverify_count = 0
        self._epoch = 1
        # Last grant state published per robot, so the permit path is
        # observable without logging at 5 Hz. Silence on both ends made the
        # first instrumented run ambiguous: 'never published' and 'never
        # received' looked identical.
        self._published_granted: dict[str, bool | None] = {}
        self._request_seq = 0
        self.requests = RequestLog()
        self.startup_notes: list[str] = []

        permit_topic = str(p("permit_topic").value)
        localiser_topic = str(p("localiser_topic").value)
        for robot in self.robots:
            self.create_subscription(
                Odometry, f"/{robot}/odom", self._odom_cb(robot), _qos())
            # Absolute and per-robot: this node is not namespaced (it is `/fleet_coordinator`),
            # so a relative name would silently resolve to `/amcl_pose` and there would be no
            # localised pose at all -- which reads exactly like a localiser that never publishes.
            self.create_subscription(
                PoseWithCovarianceStamped, f"/{robot}/{localiser_topic}",
                self._localiser_cb(robot), _qos())
        self.pubs = {
            robot: self.create_publisher(ResourcePermit, f"/{robot}/{permit_topic}", _qos())
            for robot in self.robots
        }

        self.create_service(AcquirePassage, "/fleet/acquire_passage", self._srv_acquire)
        self.create_service(RenewPermit, "/fleet/renew_permit", self._srv_renew)
        self.create_service(ConfirmClear, "/fleet/confirm_clear", self._srv_clear)
        # Read-only snapshot for the acceptance harness and for a human asking
        # "why is nothing moving?". It cannot change state.
        self.create_service(Trigger, "/fleet/status", self._srv_status)
        self.create_service(Trigger, "/fleet/reverify", self._srv_reverify)

        rate = float(p("permit_rate_hz").value)
        if rate <= 0:
            raise ValueError("permit_rate_hz must be > 0")
        self.create_timer(1.0 / rate, self._publish_permits)
        self.create_timer(0.2, self._expire_and_sweep)
        # A separate timer for re-verification, with its own exception guard: if the
        # promotion pass ever faults it must not be able to take expiry down with it.
        # A coordinator whose permits stop lapsing is a worse failure than one that
        # stops promoting.
        self.create_timer(0.5, self._auto_reverify)

        self._announce()

    # ------------------------------------------------------------------ #
    # startup honesty
    # ------------------------------------------------------------------ #

    def _announce(self) -> None:
        log = self.get_logger()
        log.info(
            f"fleet_coordinator up: robots={sorted(self.robots)} "
            f"permit_ttl={self.tcfg.permit_ttl_s}s epoch={self._epoch}"
        )
        log.info(
            f"pose source: {self.get_parameter('pose_source').value!r}; localiser on "
            f"'/<robot>/{self.get_parameter('localiser_topic').value}', "
            + LocaliserPolicy(
                max_age_s=float(self.get_parameter("localiser_timeout_s").value),
                max_moved_m=float(self.get_parameter("localiser_move_margin_m").value),
              ).describe()
            + ". The safety gate makes the same decision on the same subject; see "
              "fleet_core.pose_source for the measurement it rests on."
        )
        log.info(
            "map-frame poses are composed as spawn (+) odom. gz DiffDrive publishes "
            "odometry with its origin at the spawn pose. Its error is a RATE -- measured on N01 "
            "(2026-09-17) at 1.5516 m median and 2.6825 m p95 against ground truth while the "
            "localiser was 0.1866 m -- so the composed pose is kept for REPORTING and is not what "
            "this node judges with unless pose_source is spawn_odom. The assumption is recorded, "
            "not hidden."
        )
        for r, link in sorted(self.robots.items()):
            log.info(f"  {r}: spawn {link.spawn} footprint "
                     f"{link.length_m}x{link.width_m} m")
        log.info(
            "every resource starts UNKNOWN. No passage is granted until the start-up "
            "occupancy sweep has observed every robot and found none inside a "
            "protected rectangle."
        )
        log.info(
            "wall clock only: use_sim_time is irrelevant here, so pausing the simulator "
            "does not extend a permit."
        )
        log.info(
            "UNKNOWN resources are re-verified against every robot's fresh odometry, and "
            "on demand via /fleet/reverify. Without that pass a single aborted crossing "
            "would leave the corridor unusable for the rest of the session: ConfirmClear "
            "refuses on an UNKNOWN resource by design, and the start-up sweep runs once."
        )
        log.warning(
            "NOT covered by this node: it publishes no cmd_vel; it does not verify that a "
            "robot actually stopped; and the pose it judges is not ground truth. Which pose it "
            "judges from is stated above, and it is the same decision the safety gate makes -- "
            "see fleet_core.pose_source."
        )

    # ------------------------------------------------------------------ #
    # observation
    # ------------------------------------------------------------------ #

    def _now(self) -> float:
        return time.monotonic() - self._wall_start

    def _spawn_of(self, robot: str) -> tuple[float, float, float]:
        entry = self._spawn_table.get(robot)
        if not entry:
            self.get_logger().error(
                f"no spawn pose for {robot} in config/spawns.yaml. Its odom poses "
                "cannot be placed in the map frame, so every geometric judgement "
                f"about it would be wrong. Treating it as spawned at (0,0,0) only "
                "so the node still starts; expect refusals."
            )
            return (0.0, 0.0, 0.0)
        return (float(entry.get("x", 0.0)), float(entry.get("y", 0.0)),
                float(entry.get("yaw", 0.0)))

    def _odom_cb(self, robot: str):
        def _cb(msg: Odometry) -> None:
            link = self.robots[robot]
            # The message is in the ODOM frame, whose origin is this robot's spawn
            # pose. The traffic rectangles are in the map frame, so the two are
            # composed here rather than compared directly -- see compose_2d.
            composed = link.source.note_odom(
                raw=(
                    float(msg.pose.pose.position.x),
                    float(msg.pose.pose.position.y),
                    _yaw_from_quaternion(msg.pose.pose.orientation),
                ),
                wall=self._now(),
            )
            link.pose = composed
            link.pose_wall = self._now()
            link.ever_seen = True

        return _cb

    def _localiser_cb(self, robot: str):
        def _cb(msg: PoseWithCovarianceStamped) -> None:
            q = msg.pose.pose.position
            self.robots[robot].source.note_localiser(
                pose=(float(q.x), float(q.y),
                      _yaw_from_quaternion(msg.pose.pose.orientation)),
                wall=self._now(),
            )

        return _cb

    def _posed(self, robot: str):
        """The pose this node may judge with, and whether it may judge at all."""
        return self.robots[robot].source.trusted(now=self._now())

    def _pose_or_none(self, robot: str) -> tuple[float, float, float] | None:
        """Ask the source, not the composed observation.

        `link.pose` is the composed pose kept for reporting. Judging from it is what produced
        `D-P11-03`: the coordinator held a robot 1.25 m from a node Nav2 had reported it reached,
        because the composed pose's error (1.5516 m median) is larger than the 0.80 m tolerance it
        was compared against.
        """
        return self._posed(robot).pose

    # ------------------------------------------------------------------ #
    # start-up occupancy sweep
    # ------------------------------------------------------------------ #

    def _expire_and_sweep(self) -> None:
        touched = self.manager.expire_due(wall_now=self._now())
        if touched:
            self.get_logger().warning(
                f"permit(s) lapsed on {sorted(set(touched))}: resource(s) now UNKNOWN and "
                "stay BLOCKED until an explicit geometric clearance. Expiry is not "
                "evidence that the area is empty."
            )
        if not self._sweep_done:
            self._try_sweep()

    def _pose_table(self) -> dict:
        """One pose per configured robot, or None where the pose is too old to judge.

        ``None`` is passed through rather than dropped, because "we do not know where
        r02 is" is not the same fact as "r02 is somewhere else", and that difference is
        the entire safety argument for promoting a resource to FREE.
        """
        return {r: self._pose_or_none(r) for r in self.robots}

    def _size_table(self) -> dict:
        return {r: (link.length_m, link.width_m) for r, link in self.robots.items()}

    def _try_sweep(self) -> None:
        """Decide the initial occupancy of every resource, exactly once.

        Shares `CrossingManager.verified_clear` with re-verification, so there is one
        implementation of "verified clear" in the project and the two callers cannot
        drift apart.
        """
        unseen = [r for r, link in self.robots.items() if not link.ever_seen]
        if unseen:
            return
        # Give the robots a moment to settle at their spawn poses before believing
        # where they are; a first sample taken mid-drop is not a position.
        newest = max(link.pose_wall or 0.0 for link in self.robots.values())
        if self._now() - newest < 0.0:
            return
        # Elapsed, not absolute: `_wall_start` is a monotonic instant and `_now()` is
        # a duration, so adding them was true forever and the sweep never ran.
        if self.occupancy_settle_s > self._now():
            return

        scan = self.manager.verified_clear(self._pose_table(), self._size_table())

        if not scan.complete:
            note = (f"start-up sweep: {list(scan.unlocatable)} have no fresh pose; resources "
                    "stay UNKNOWN. A resource is not granted just because nobody has looked "
                    "at it yet.")
            self.startup_notes.append(note)
            self.get_logger().warning(note)
            self._sweep_done = True
            return

        for name, who in scan.occupied:
            self.manager.block(name, f"start-up sweep: {list(who)} inside {name}")
        if scan.occupied:
            self.get_logger().warning(
                f"start-up sweep: {[(n, list(w)) for n, w in scan.occupied]} occupied; "
                "those resources stay BLOCKED")

        for name in scan.freeable:
            self.manager.mark_verified_free(
                name,
                source=(
                    "start-up sweep: every configured robot reported a fresh pose and "
                    f"none was inside any protected rectangle (t={self._now():.1f}s)"
                ),
            )
        note = (f"start-up sweep: {len(scan.freeable)} verified clear, "
                f"{len(scan.occupied)} occupied, {len(scan.held)} held at "
                f"t={self._now():.1f}s")
        self.startup_notes.append(note)
        self.get_logger().info(note)
        self._sweep_done = True

    # ------------------------------------------------------------------ #
    # re-verification
    # ------------------------------------------------------------------ #

    def _reverify_unknown(self, *, trigger: str) -> dict:
        """Promote resources out of UNKNOWN on a complete, fresh observation.

        Without this pass a single aborted crossing bricks the fleet for the rest of
        the session. A lapsed permit drops its bundle to UNKNOWN; ``ConfirmClear``
        refuses on an UNKNOWN resource by design, because the robot whose permit
        lapsed is the least credible witness that the area is now empty; and the
        start-up sweep runs once. Nothing else can lift it.

        The evidence is not the robot's word. It is every configured robot's fresh
        odometry, judged here against the rectangle with the same strictness as the
        start-up sweep: one missing or stale pose promotes nothing.
        """
        candidates = tuple(
            n for n in self.tcfg.resources
            if self.manager.state_of(n) in (ResourceState.UNKNOWN, ResourceState.BLOCKED)
        )
        if not candidates:
            return {"candidates": [], "promoted": [], "unlocatable": [], "occupied": []}

        scan = self.manager.verified_clear(
            self._pose_table(), self._size_table(), only=candidates)
        if not scan.complete:
            return {"candidates": list(candidates), "promoted": [],
                    "unlocatable": list(scan.unlocatable), "occupied": []}

        for name in scan.freeable:
            self.manager.mark_verified_free(
                name,
                source=(f"{trigger}: every configured robot reported a fresh pose and none "
                        f"overlaps {name} (t={self._now():.1f}s)"),
            )
        if scan.freeable:
            self._reverify_count += 1
            self.get_logger().info(
                f"{trigger}: promoted {sorted(scan.freeable)} from UNKNOWN to FREE on fresh "
                "poses, with no permit held on them")
        return {"candidates": list(candidates), "promoted": sorted(scan.freeable),
                "unlocatable": [], "occupied": [(n, list(w)) for n, w in scan.occupied]}

    def _auto_reverify(self) -> None:
        """Timer wrapper that never raises into the executor.

        Kept separate from ``_expire_and_sweep`` on purpose: if this pass ever faults,
        expiry must keep running.

        It does nothing until the start-up sweep has finished. The sweep waits
        ``occupancy_settle_s`` for the robots to settle, and that delay exists because a
        first sample taken while a model is still dropping is not a position. This timer
        starts immediately, so without the guard below re-verification became the thing
        that first declared a corridor free, using evidence the sweep deliberately
        declines. A live run did exactly that: promoted at t=7.3 s, sweep at t=7.8 s.
        """
        if not self._sweep_done:
            return
        try:
            self._reverify_unknown(trigger="re-verify (automatic)")
        except Exception as exc:  # noqa: BLE001 -- timer safety, reported not swallowed
            self.get_logger().error(
                f"re-verification pass raised {type(exc).__name__}: {exc}; expiry is "
                "unaffected and every resource keeps its current state")

    # ------------------------------------------------------------------ #
    # permit publication
    # ------------------------------------------------------------------ #

    def _publish_permits(self) -> None:
        wall = self._now()
        for robot, link in self.robots.items():
            msg = ResourcePermit()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.robot_id = robot
            msg.task_id = link.task_id
            msg.boot_id = link.boot_id
            msg.revision = link.revision
            msg.generation = link.generation
            msg.epoch = self._epoch

            permit = self._live_permit(robot)
            if permit is None:
                msg.granted = False
                msg.reason_code = link.last_reason or ReasonCode.ROUTE_REQUIRES_PERMIT.value
                msg.detail = link.last_detail or "no live permit for this robot"
            else:
                msg.granted = True
                msg.permit_id = permit.permit_id
                msg.direction = link.direction
                msg.resources = [str(r) for r in permit.resources]
                msg.valid_for_wall_s = max(0.0, permit.expires_at - wall)
                msg.reason_code = ReasonCode.OK.value
                msg.detail = f"expires in {msg.valid_for_wall_s:.2f}s"
            self.pubs[robot].publish(msg)
            if self._published_granted.get(robot) != bool(msg.granted):
                self._published_granted[robot] = bool(msg.granted)
                if msg.granted:
                    self.get_logger().info(
                        f"permit live for {robot}: {msg.permit_id} "
                        f"direction={msg.direction} "
                        f"valid_for_wall_s={msg.valid_for_wall_s:.2f} "
                        f"task={msg.task_id} generation={msg.generation}"
                    )
                else:
                    self.get_logger().info(
                        f"no live permit for {robot} "
                        f"({msg.reason_code}); publishing granted=false"
                    )

    def _live_permit(self, robot: str):
        link = self.robots[robot]
        if not link.task_id or not link.direction:
            return None
        wall = self._now()
        for name in self.manager.bundle(link.direction):
            for hold in self.manager.book.holders(self.manager.rid(name)):
                permit = hold.permit
                if permit.task_id != link.task_id:
                    continue
                if permit.generation != link.generation or permit.epoch != self._epoch:
                    continue
                if permit.expired(wall):
                    return None
                return permit
        return None

    # ------------------------------------------------------------------ #
    # services
    # ------------------------------------------------------------------ #

    def _srv_acquire(self, req: AcquirePassage.Request, resp: AcquirePassage.Response):
        wall = self._now()
        link = self.robots.get(req.robot_id)
        if link is None:
            resp.reason_code = ReasonCode.INVALID_INPUT.value
            resp.detail = f"unknown robot {req.robot_id!r}; known: {sorted(self.robots)}"
            return resp
        if req.direction not in self.tcfg.directions:
            resp.reason_code = ReasonCode.INVALID_INPUT.value
            resp.detail = f"unknown direction {req.direction!r}"
            return resp
        if not req.localization_valid:
            resp.reason_code = ReasonCode.LOCALIZATION_STALE.value
            resp.detail = "caller reports its localization is not valid"
            return resp
        if not self._sweep_done:
            resp.reason_code = ReasonCode.RESOURCE_UNKNOWN.value
            resp.detail = "start-up occupancy sweep has not completed; nothing is grantable yet"
            return resp

        pose = self._pose_or_none(req.robot_id)
        if pose is None:
            resp.reason_code = ReasonCode.LOCALIZATION_STALE.value
            resp.detail = (
                f"no usable pose for {req.robot_id}: {self._posed(req.robot_id).reason}. "
                "The pose in the request is logged, not trusted."
            )
            return resp

        self._request_seq += 1
        res = self.manager.acquire(
            req.direction,
            task_id=req.task_id,
            robot_id=req.robot_id,
            boot_id=req.boot_id,
            revision=int(req.revision),
            generation=int(req.generation),
            # The coordinator's own epoch, not the caller's. An epoch chosen by the
            # supervised party cannot detect a restarted supervisor -- it can only
            # switch the check off, which is exactly what happened here: the driver
            # sent 0, every permit was minted under 0, and _live_permit matched
            # nothing because it compares against self._epoch. See DECISIONS D-P4-10.
            epoch=self._epoch,
            now=wall,
            wall_now=wall,
            pose=pose,
            length_m=link.length_m,
            width_m=link.width_m,
            localization_valid=True,
        )
        if res.granted:
            link.task_id = req.task_id
            link.boot_id = req.boot_id
            link.revision = int(req.revision)
            link.generation = int(req.generation)
            link.epoch = self._epoch
            link.direction = req.direction
            link.permit_id = res.permit.permit_id
            link.last_reason = ReasonCode.OK.value
            link.last_detail = ""
            resp.granted = True
            resp.permit_id = res.permit.permit_id
            resp.task_id = req.task_id
            resp.direction = req.direction
            resp.resources = [str(r) for r in res.permit.resources]
            resp.valid_for_wall_s = max(0.0, res.permit.expires_at - wall)
            resp.reason_code = ReasonCode.OK.value
            # Echoed so the client can scope every later renew and clear to the
            # epoch this grant was actually made under. Without it the response
            # carries the srv default of 0 while the permit carries this node's
            # epoch, and every renewal is refused as stale.
            resp.epoch = self._epoch
            resp.detail = (
                f"granted {req.direction}; requested pose x={req.x:.3f} y={req.y:.3f} "
                f"yaw={req.yaw:.3f} was logged, judged against observed "
                f"x={pose[0]:.3f} y={pose[1]:.3f} yaw={pose[2]:.3f}"
            )
        else:
            link.last_reason = res.reason.value
            link.last_detail = f"acquire refused: {res.reason.value}"
            resp.granted = False
            resp.reason_code = res.reason.value
            resp.direction = req.direction
            resp.queued_behind = list(res.queued_behind)
            resp.detail = (
                f"{res.reason.value}; blocked_by={[str(b) for b in res.blocked_by]} "
                f"observed x={pose[0]:.3f} y={pose[1]:.3f}"
                + (f"; {res.detail}" if res.detail else "")
            )

        self.requests.add(
            wall=round(wall, 4), seq=self._request_seq, robot=req.robot_id,
            request_id=req.request_id, direction=req.direction, task_id=req.task_id,
            granted=bool(res.granted), reason=res.reason.value,
            observed_pose=[round(v, 4) for v in pose],
            reported_pose=[round(float(req.x), 4), round(float(req.y), 4), round(float(req.yaw), 4)],
        )
        return resp

    def _srv_renew(self, req: RenewPermit.Request, resp: RenewPermit.Response):
        wall = self._now()
        link = self.robots.get(req.robot_id)
        resp.epoch = self._epoch
        if link is None:
            resp.reason_code = ReasonCode.INVALID_INPUT.value
            resp.detail = f"unknown robot {req.robot_id!r}"
            return resp
        if req.direction not in self.tcfg.directions:
            resp.reason_code = ReasonCode.INVALID_INPUT.value
            resp.detail = f"unknown direction {req.direction!r}"
            return resp
        try:
            permit = self.manager.renew(
                req.direction,
                task_id=req.task_id,
                boot_id=req.boot_id,
                revision=int(req.revision),
                generation=int(req.generation),
                epoch=int(req.epoch),
                wall_now=wall,
            )
        except Exception as exc:  # StaleCommandError / KeyError, both ordinary refusals
            resp.granted = False
            resp.reason_code = (
                ReasonCode.PERMIT_STALE.value
                if type(exc).__name__ == "StaleCommandError"
                else ReasonCode.PERMIT_EXPIRED.value
            )
            resp.detail = f"renew refused: {exc}"
            link.last_reason = resp.reason_code
            link.last_detail = resp.detail
            return resp
        resp.granted = True
        resp.valid_for_wall_s = max(0.0, permit.expires_at - wall)
        resp.reason_code = ReasonCode.OK.value
        resp.detail = f"renewed for {resp.valid_for_wall_s:.2f}s"
        return resp

    def _srv_clear(self, req: ConfirmClear.Request, resp: ConfirmClear.Response):
        wall = self._now()
        link = self.robots.get(req.robot_id)
        if link is None:
            resp.reason_code = ReasonCode.INVALID_INPUT.value
            resp.detail = f"unknown robot {req.robot_id!r}"
            return resp
        if req.direction not in self.tcfg.directions:
            resp.reason_code = ReasonCode.INVALID_INPUT.value
            resp.detail = f"unknown direction {req.direction!r}"
            return resp

        pose = self._pose_or_none(req.robot_id)
        if pose is None:
            resp.reason_code = ReasonCode.LOCALIZATION_STALE.value
            resp.detail = (
                f"no usable pose for {req.robot_id}: {self._posed(req.robot_id).reason}. "
                "Clearance cannot be proven, so the bundle stays held. The pose in the request "
                "is logged, not trusted."
            )
            return resp

        outcome = self.manager.confirm_clear(
            req.direction,
            task_id=req.task_id,
            generation=int(req.generation),
            pose=pose,
            length_m=link.length_m,
            width_m=link.width_m,
            source=f"coordinator observed odom at t={wall:.2f}s",
        )
        resp.cleared = outcome.cleared
        resp.reason_code = outcome.reason.value
        resp.detail = outcome.detail
        resp.still_inside = list(outcome.still_inside)
        if outcome.cleared:
            link.task_id = ""
            link.direction = ""
            link.permit_id = ""
            link.last_reason = ReasonCode.OK.value
            link.last_detail = ""
        else:
            link.last_reason = outcome.reason.value
            link.last_detail = outcome.detail
        self.requests.add(
            wall=round(wall, 4), seq=self._request_seq, robot=req.robot_id,
            request_id="", direction=req.direction, task_id=req.task_id,
            granted=bool(outcome.cleared), reason=outcome.reason.value,
            observed_pose=[round(v, 4) for v in pose],
            reported_pose=[round(float(req.x), 4), round(float(req.y), 4), round(float(req.yaw), 4)],
        )
        return resp

    # ------------------------------------------------------------------ #

    def _srv_reverify(self, _request: Trigger.Request, response: Trigger.Response):
        """Force one re-verification pass and report what it did.

        Read-mostly on purpose: it can only promote a resource to FREE against the
        same complete observation the start-up sweep uses. It cannot grant, extend,
        release or unblock a permit, and it cannot free a resource that is held or
        that any robot's footprint overlaps. Unlike `_auto_reverify`, an exception
        here is reported to the caller rather than swallowed by a timer.
        """
        if not self._sweep_done:
            # Fail closed rather than promote on pre-settle evidence. Saying so is the
            # point: a caller that gets an empty promotion list cannot tell "nothing to
            # promote" from "not ready yet".
            response.success = False
            response.message = json.dumps({
                "refused": "start-up occupancy sweep has not completed",
                "note": "promoting now would use pre-settle poses that the sweep "
                        "deliberately waits past; retry once sweep_done is true",
            })
            return response
        outcome = self._reverify_unknown(trigger="re-verify (requested)")
        response.success = True
        response.message = json.dumps(outcome)
        return response

    def _srv_status(self, _request: Trigger.Request, response: Trigger.Response):
        """Read-only. Returns the whole scheduling state as one JSON string."""
        response.success = True
        response.message = json.dumps(self.status_report())
        return response

    def status_report(self) -> dict:
        snap = self.manager.snapshot()
        return {
            "epoch": self._epoch,
            "sweep_done": self._sweep_done,
            "reverify_count": self._reverify_count,
            "startup_notes": list(self.startup_notes),
            "resources": snap,
            "robots": {
                r: {
                    "task_id": link.task_id,
                    "direction": link.direction,
                    "permit_id": link.permit_id,
                    "pose": None if link.pose is None else [round(v, 4) for v in link.pose],
                    "pose_age_s": None if link.pose_age_s(self._now()) is None
                    else round(link.pose_age_s(self._now()), 3),
                    "ever_seen": link.ever_seen,
                    "last_reason": link.last_reason,
                }
                for r, link in self.robots.items()
            },
            "requests": list(self.requests.entries),
            "resource_states": {n: self.manager.state_of(n).value for n in self.tcfg.resources},
        }


def main(argv: list[str] | None = None) -> int:
    rclpy.init(args=argv)
    node = FleetCoordinator()
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
