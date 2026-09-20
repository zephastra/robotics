#!/usr/bin/env python3
"""Drives one robot through the staged crossing: P4.4.

The sequence, and why each stage exists (CONTRACTS section 8):

    1. drive to the WAIT node            -- an unpermitted robot may go no further
    2. AcquirePassage                    -- the atomic bundle, asked for from a legal place
    3. drive to the EXIT node            -- the crossing leg, now authorised
    4. drive to the RELEASE node         -- still under the same permit, but outside
    5. ConfirmClear                      -- evidence, verified by the coordinator
    6. release the permit locally        -- only now, and only on a verified clear

Two things this file deliberately does NOT do:

* **It never publishes cmd_vel.** It sends Nav2 action goals. Movement still passes
  through that robot's safety gate, which holds its own copy of the geometry and can
  refuse a goal this program is convinced is fine. If the two disagree, the gate wins
  -- which is the point of having the gate.

* **It does not treat "goal accepted" as "arrived".** Every stage waits for the action
  result and reports the Nav2 status. A goal accepted and a goal reached are different
  facts and only the second one advances the sequence.

Exit codes: 0 all stages completed, 4 the sequence stopped part-way (the JSON says
where and why), 3 the prerequisites for running at all were missing.
"""

from __future__ import annotations

import json
import math
import signal
import time
from dataclasses import asdict, dataclass, field
from pathlib import Path

import rclpy
import yaml
from fleet_core import (
    STOP_BUSY,
    STOP_NAV_ABSENT,
    STOP_RETREAT_REFUSED,
    STOP_RUN_DEADLINE,
    STOP_INFRA_TIMEOUT,
    STOP_SIGNAL,
    ReleasePolicy,
    needs_retreat,
    retreat_command,
    validate_traffic_config,
)
from fleet_interfaces.srv import AcquirePassage, ConfirmClear, RenewPermit
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)

#: How many of this driver's stages move the robot. `expected_passage_s` describes the
#: PASSAGE, and the per-stage share that `classify_stop` compares against is the passage
#: divided by this. It is a named constant rather than `self.movement_stages` because
#: that attribute was READ at line 348 and never assigned anywhere: the read sits inside
#: an `and` whose left operand is `self.expected_passage_s > 0.0`, so Python's
#: short-circuit meant it was never evaluated on any recorded run -- all 143 of them
#: declare `expected_passage_s` 0.0. The bug was armed, not harmless: the first scenario
#: to declare an expected passage (which `reserve_ok` and the INFRA_TIMEOUT split both
#: need) would have raised AttributeError inside the timeout branch, and the driver's
#: handler reports that as `stopped_by: "exception"` -- "the driver is broken" -- when
#: CONTRACTS section 9 wants "the box was slow".
MOVEMENT_STAGES = 4


def _load_traffic():
    import os

    env = os.environ.get("FLEET009_ROOT")
    root = Path(env) if env else None
    if root is None:
        for parent in Path(__file__).resolve().parents:
            if (parent / "config" / "resources.yaml").is_file():
                root = parent
                break
    if root is None:
        raise RuntimeError("cannot find config/resources.yaml; set FLEET009_ROOT")
    return validate_traffic_config(
        yaml.safe_load((root / "config" / "resources.yaml").read_text(encoding="utf-8")))


@dataclass
class Stage:
    name: str
    ok: bool
    detail: str = ""
    elapsed_s: float = 0.0
    nav_status: str = ""
    observed_pose: list[float] = field(default_factory=list)

    #: What Nav2's own feedback last said, and how many times it said anything.
    #:
    #: Added 2026-09-18 because ELAPSED TIME ALONE CANNOT DISTINGUISH the four things that make
    #: a stage long: driving slowly, recovering (Nav2 unable to make progress), not being
    #: commanded at all, and not having driven in the first place. Measured: one driver spent
    #: 192.9 s reaching a node 4 m away and reported only "reached this driver's own budget",
    #: which reads as a budget problem and may be three other things.
    #:
    #: -1 means "never said", which is deliberately not the same as 0.
    distance_remaining_m: float = -1.0
    recoveries: int = -1
    feedback_samples: int = 0
    #: Feedback messages that ARRIVED but could not be read. Deliberately not folded into
    #: `feedback_samples`: "Nav2 said nothing" and "we could not understand Nav2" are
    #: different facts, and only one of them is Nav2's fault.
    feedback_unreadable: int = 0


class StagedCrossing(Node):
    def __init__(self, *, robot: str, direction: str, task_id: str, generation: int,
                 timeout_s: float, request_delay_s: float,
                 queue_allowance_s: float = 0.0,
                 expected_passage_s: float = 0.0) -> None:
        super().__init__("staged_crossing", namespace=robot)
        self.cfg = _load_traffic()
        if direction not in self.cfg.directions:
            raise ValueError(f"unknown direction {direction!r}; known {sorted(self.cfg.directions)}")
        self.dspec = self.cfg.directions[direction]
        self.robot = robot
        self.direction = direction
        self.task_id = task_id
        self.generation = generation
        self.revision = 0
        # The coordinator owns the epoch and returns it on every grant; this is only
        # the value echoed back on renew and clear. It starts at 0 because nothing has
        # been granted yet -- inventing one locally is what made every grant invisible
        # to the coordinator's own permit publisher.
        self.epoch = 0
        self.timeout_s = timeout_s
        self.request_delay_s = request_delay_s
        self.queue_allowance_s = queue_allowance_s
        # Declared by the scenario, never inferred here. 0.0 means "no expectation
        # declared", which disables the reserve check rather than inventing one --
        # a driver that guessed its own passage time would be measuring its own
        # opinion and refusing on it.
        self.expected_passage_s = expected_passage_s

        self.boot_id = f"staged-{int(time.time())}"
        self.stages: list[Stage] = []
        self.permit_id = ""
        self.acquire_tries = 0
        self.queued_behind: list[str] = []
        # A crossing leg takes longer than one permit TTL, so the permit has to be
        # renewed while the robot is moving. The first version of this driver never
        # renewed, and the consequence was a textbook self-inflicted failure: the
        # permit lapsed mid-corridor, the coordinator dropped the resource to UNKNOWN,
        # the robot's own gate stopped it, Nav2 reported "Failed to make progress" over
        # and over, and the goal was aborted. Every component behaved correctly except
        # this one.
        self.renew_every_s = max(1.0, (self.cfg.permit_ttl_s - self.cfg.renew_before_s) / 2.0)
        self.renewals = 0
        self.renew_failures: list[str] = []

        #: Why this driver stopped, when it stopped itself. Empty means it ran to a
        #: verdict. See `fleet_core.crossing_release`: a dispatcher kill neither cancels
        #: this driver's nav goal nor lets it write a report, so the driver has to stop
        #: on its own while there is still time.
        self.stopped_by = ""
        self.policy = ReleasePolicy(queue_allowance_s=queue_allowance_s)
        self.started_at = 0.0
        self.run_deadline = 0.0
        #: Every rejected goal's total wait, in seconds. Reported, because "the server
        #: was busy for 8 s" and "the server was absent" want different next actions.
        self.busy_waits: list[float] = []
        self._stop = False
        #: The latest Nav2 feedback for the stage in flight. Cleared by `_set_stage`, so each
        #: stage reports its own numbers rather than the previous stage's.
        self._feedback: dict = {}

        # Imported lazily, and imported rather than looked up by name.
        #
        # Two reasons. nav2_msgs lives in an optional source overlay (/opt/nav2), so a
        # module-level import would make this file unimportable wherever that overlay
        # is not sourced -- including in this project's own test process. And
        # `rosidl_runtime_py.utilities.get_message` rejects the `pkg/action/Name` form
        # outright:
        #
        #     ValueError: Expected the full name of a message, got
        #                 'nav2_msgs/action/NavigateToPose'
        #
        # That is what stopped every driver in the first two P4.5 acceptance runs. The
        # message blames "a bad name" rather than "the wrong lookup function", so it
        # reads like a typo and sends you looking in the wrong place.
        try:
            from nav2_msgs.action import NavigateToPose
        except ImportError as exc:
            raise RuntimeError(
                "nav2_msgs is not importable. Source the Nav2 overlay (/opt/nav2) before "
                "running the staged crossing: this driver moves robots by sending Nav2 "
                "action goals and has no other mechanism."
            ) from exc

        self.NavigateToPose = NavigateToPose
        # Relative to this node's namespace, which is the robot id -- so this
        # resolves to /<robot>/navigate_to_pose. A relative name on an unnamespaced
        # node would look for /navigate_to_pose, and "action server absent" is then
        # reported for a Nav2 stack that is running perfectly.
        self.nav_client = ActionClient(self, self.NavigateToPose, "navigate_to_pose")
        self.acquire_cli = self.create_client(AcquirePassage, "/fleet/acquire_passage")
        self.renew_cli = self.create_client(RenewPermit, "/fleet/renew_permit")
        self.clear_cli = self.create_client(ConfirmClear, "/fleet/confirm_clear")

        # ---- the retreat emitter's view of the world (D-P14-02) ---------------- #
        # Located at the same topic and the same QoS the safety gate trusts for its own
        # pose: `amcl_pose` inside this robot's namespace, resolved RELATIVE to the
        # namespace so it becomes /<robot>/amcl_pose. An absolute name would subscribe to
        # a topic no one publishes and produce silence, and "the localiser never spoke"
        # and "we listened to the wrong topic" are indistinguishable from the outside --
        # which is why the gate names its subscriber at startup too.
        #
        # This driver previously had NO pose source at all: it drove by issuing goals to
        # named nodes and never asked where the robot was. That is fine for driving, and
        # useless for the retreat question, which is entirely about where the robot
        # actually is.
        self._pose_topic = "amcl_pose"
        self._latest_pose: tuple[float, float, float] | None = None
        self._pose_msgs = 0
        self._pose_sub = None
        #: The overlap depth the last issued retreat was trying to reduce, or 0.0 if
        #: none has been issued. `retreat_policy.progress` is consulted before a SECOND
        #: retreat, because a retreat measured not to work must not be re-issued -- that
        #: is the same unbounded retry loop as `recoveries=8`.
        self._retreat_overlap_m = 0.0
        #: The resource names the grant actually named, so "unpermitted" is answered
        #: against the permit in this driver's hand rather than against an assumption
        #: about which bundle it should have. Set on a successful `acquire`.
        self._granted_resources: tuple[str, ...] = ()

    def _ensure_pose_subscription(self) -> None:
        """Subscribe to this robot's localised pose, once, on first need.

        Lazy on purpose: the ordinary crossing path never asks where the robot is, so it
        must not pay for a subscription. It is created on the first failure that needs
        the geometry, and the first message may take a moment to arrive -- so
        `_latest_pose` stays None until one does, and every consumer treats None as
        "cannot answer" rather than as the origin.
        """
        if self._pose_sub is not None:
            return
        qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
        )
        self._pose_sub = self.create_subscription(
            PoseWithCovarianceStamped, self._pose_topic, self._on_pose, qos)

    def _on_pose(self, msg: PoseWithCovarianceStamped) -> None:
        """Record the latest localised pose. Decides nothing.

        ROS calls this with ONE argument and the payload is the message itself --
        `create_subscription` hands the callback the deserialised message. The gate does
        the same thing with its own `_on_localiser`. Getting the arity wrong here is
        invisible from outside: the process dies on the first message and the retreat
        stage reports "no pose", which reads exactly like a localiser that never
        published. `scripts/check_ros_callback_arity.py` is the guard for that.
        """
        q = msg.pose.pose.orientation
        # Yaw from the quaternion, the same way the gate does it: this is a planar robot
        # and only the heading matters. `atan2` on (z, w) is the z-axis rotation for a
        # unit quaternion with roll and pitch zero, which is what AMCL produces.
        yaw = math.atan2(2.0 * (q.w * q.z), 1.0 - 2.0 * (q.z * q.z))
        self._latest_pose = (
            float(msg.pose.pose.position.x),
            float(msg.pose.pose.position.y),
            float(yaw),
        )
        self._pose_msgs += 1

    def _unpermitted_rects(self, pose: tuple[float, float, float]) -> dict:
        """Which protected rectangles this robot overlaps and has no permit for.

        The rectangles come from `config/resources.yaml` through the same
        `TrafficConfig` the gate uses, so "unpermitted" means one thing in this process
        and in the gate. A second copy of the geometry is how the two came to disagree by
        exactly the stopping reserve on 2026-09-17 (`D-P10-01`).

        "Held" is asked of the permit this driver actually obtained: the bundle the grant
        named. A robot that still holds its bundle is not inside an unpermitted
        rectangle, and one whose permit has lapsed is -- which is the case the retreat
        exists for, and the reason the held set is taken from the GRANT rather than
        re-derived. If the permit lapsed, the grant is stale, and a stale grant must not
        make a rectangle look permitted.
        """
        held = set(self._granted_resources)
        x, y, _yaw = pose
        out: dict[str, tuple[float, float, float, float]] = {}
        for name, spec in self.cfg.resources.items():
            if name in held:
                continue
            for rect in spec.rects:
                if rect.x_min <= x <= rect.x_max and rect.y_min <= y <= rect.y_max:
                    out[name] = (rect.x_min, rect.y_min, rect.x_max, rect.y_max)
                    break
        return out

    def retreat_if_stuck(self, reason: str) -> Stage | None:
        """Notice a robot that cannot be where it is, and tell it to move. D-P14-02.

        Called with the reason the stage in front of it failed, which is what makes this
        a POLICY rather than a poll: the driver has just been told "you could not reach
        X", and this asks the one question that distinguishes the two things that look
        identical afterwards -- could not get there, or got there and could not stay.

        Returns `None` when no retreat is needed, which is the ordinary case. Returns a
        `Stage` when one was decided, so the JSON says what happened either way.
        """
        self._ensure_pose_subscription()
        if self._latest_pose is None:
            # "No localised pose has arrived" is not "the robot is outside every
            # rectangle". Reporting nothing here would be claiming to have checked.
            return self._set_stage(
                "retreat", False,
                f"after {reason}: no localised pose has arrived on {self._pose_topic} "
                f"in this robot's namespace, so the retreat geometry cannot be evaluated. "
                f"This is an INSTRUMENT result and not a finding about the robot: "
                f"D-P17-12 measured `no localised pose has arrived` as the dominant "
                f"reason the gate cannot trust a pose, so the two share a cause.",
                time.monotonic())

        pose = self._latest_pose
        rects = self._unpermitted_rects(pose)
        if not rects:
            self.get_logger().info(
                f"retreat: not needed after {reason} -- the robot at "
                f"({pose[0]:.2f}, {pose[1]:.2f}) is outside every region it has no "
                f"permit for")
            return None

        decision = needs_retreat(pose, (0.0, 0.0, 0.0), rects, permittable=False)
        if not decision.needed:
            self.get_logger().info(f"retreat: not needed after {reason} -- {decision.reason}")
            return None

        if self._retreat_overlap_m > 0.0 and decision.overlap_m >= self._retreat_overlap_m:
            # The loop this policy exists to prevent, in its new costume. A retreat was
            # already issued for at least this much overlap and the overlap did not fall,
            # so issuing another one is the same unbounded retry as `recoveries=8`.
            return self._set_stage(
                "retreat", False,
                f"a retreat was already issued for {self._retreat_overlap_m:.3f} m of "
                f"overlap and the overlap is now {decision.overlap_m:.3f} m -- it did not "
                f"reduce it, so it must not be re-issued. This is the same shape as "
                f"`recoveries=8`: a command that has been measured not to work.",
                time.monotonic())

        command = retreat_command(pose, rects)
        if command.goal is None:
            return self._set_stage("retreat", False, command.reason, time.monotonic())
        self._retreat_overlap_m = float(command.overlap_m)

        self.get_logger().warning(
            f"RETREAT ISSUED after {reason}: {command.reason}; goal {command.goal}, "
            f"overlap {command.overlap_m:.3f} m, distance {command.distance_m:.3f} m. "
            "Sent as an ordinary Nav2 goal, so the robot's own gate judges it: if it is "
            "refused, the PERMIT half of D-P14-02 is what is missing.")

        t0 = time.monotonic()
        x, y = command.goal
        goal = self.NavigateToPose.Goal()
        goal.pose.header.frame_id = "map"
        # Stamp left at zero, for the same reason `drive_to` leaves it at zero: this node
        # runs on the wall clock while Nav2 runs on /clock, and a wall-clock stamp makes
        # Nav2 report "Lookup would require extrapolation", which points at TF instead of
        # at the stamp that caused it.
        goal.pose.pose.position.x = float(x)
        goal.pose.pose.position.y = float(y)
        goal.pose.pose.orientation.w = 1.0

        send = self.nav_client.send_goal_async(goal, feedback_callback=self._on_feedback)
        rclpy.spin_until_future_complete(self, send, timeout_sec=10.0)
        handle = send.result()
        if handle is None or not handle.accepted:
            self.stopped_by = STOP_RETREAT_REFUSED
            return self._set_stage(
                "retreat", False,
                f"the retreat goal to ({x:.2f}, {y:.2f}) was REFUSED by the navigator, "
                f"not reached: {command.reason}. The PERMIT half of D-P14-02 is what is "
                f"missing -- a robot inside a region it may not occupy may be permitted "
                f"to move only if the motion STRICTLY reduces its overlap, and that rule "
                f"is not implemented, so the gate can only refuse. This is the state the "
                f"project has been reading as a navigation fault.",
                t0)

        res = handle.get_result_async()
        deadline = min(t0 + 30.0,
                       self.run_deadline if self.run_deadline else t0 + 30.0)
        while not res.done() and time.monotonic() < deadline:
            rclpy.spin_until_future_complete(self, res, timeout_sec=0.2)
        if not res.done():
            cancel = handle.cancel_goal_async()
            rclpy.spin_until_future_complete(self, cancel, timeout_sec=5.0)
            self.stopped_by = STOP_RUN_DEADLINE
            return self._set_stage(
                "retreat", False,
                f"the retreat goal to ({x:.2f}, {y:.2f}) did not finish within "
                f"{time.monotonic() - t0:.1f}s and was cancelled: {command.reason}",
                t0)
        status = str(res.result().status) if res.result() is not None else "UNKNOWN"
        ok = res.result() is not None and res.result().status == 4
        return self._set_stage(
            "retreat", ok,
            (f"retreated out of {command.overlap_m:.3f} m of unpermitted overlap: "
             f"{command.reason} (nav status {status})" if ok else
             f"the retreat returned nav status {status}: {command.reason}"),
            t0, status)

    # ------------------------------------------------------------------ #

    def node_pose(self, name: str) -> tuple[float, float, float]:
        n = self.cfg.nodes[name]
        return (n.x, n.y, n.yaw)

    def _wait(self, client, label: str, timeout_s: float = 30.0) -> bool:
        self.get_logger().info(f"waiting for {label} ...")
        return client.wait_for_service(timeout_sec=timeout_s)

    def _on_feedback(self, msg) -> None:
        """Record the latest Nav2 feedback. Decides nothing; see `Stage`'s new fields.

        ROS calls this with ONE argument. `send_goal_async(feedback_callback=...)` is fed by
        rclpy/action/client.py as `await_or_execute(cb, feedback_msg)`, so what arrives is the
        ActionFeedback WRAPPER and the payload is at `msg.feedback`. Both halves matter, and
        both were already got right elsewhere in this repository: scripts/nav_goal.py.

        Getting either half wrong is invisible from outside. A driver that dies on the first
        feedback message writes a report -- `complete: false, stopped_by: exception,
        stages: []` -- that reads exactly like a robot that refused to move, and the case's
        safety verdict comes back PASS because a robot that never moves never enters a
        corridor.

        A message that ARRIVED but could not be read is counted apart from one that never
        arrived: "the instrument recorded nothing" and "nothing was said" are two facts, and
        this project has already read one of them as the other.
        """
        payload = getattr(msg, "feedback", None)
        if payload is None:
            self._feedback["unreadable"] = int(self._feedback.get("unreadable", 0)) + 1
            return
        self._feedback["distance_remaining_m"] = round(
            float(getattr(payload, "distance_remaining", -1.0)), 3)
        self._feedback["recoveries"] = int(getattr(payload, "number_of_recoveries", -1))
        self._feedback["feedback_samples"] = int(self._feedback.get("feedback_samples", 0)) + 1

    def _set_stage(self, name: str, ok: bool, detail: str = "", t0: float = 0.0,
                   nav_status: str = "") -> Stage:
        stage = Stage(name=name, ok=ok, detail=detail,
                      elapsed_s=round(time.monotonic() - t0, 3) if t0 else 0.0,
                      nav_status=nav_status,
                      distance_remaining_m=float(self._feedback.get("distance_remaining_m", -1.0)),
                      recoveries=int(self._feedback.get("recoveries", -1)),
                      feedback_samples=int(self._feedback.get("feedback_samples", 0)),
                      feedback_unreadable=int(self._feedback.get("unreadable", 0)))
        self._feedback.clear()
        self.stages.append(stage)
        # The numbers go in the TEXT log as well as the JSON. A fact that exists only in a
        # recording is a fact nobody greps for, and this project has already read one of those
        # as "it does not exist".
        if stage.feedback_samples:
            nav_note = (f" [nav: {stage.distance_remaining_m:.2f} m left, "
                        f"{stage.recoveries} recoveries, {stage.feedback_samples} feedback]")
        elif stage.feedback_unreadable:
            nav_note = (f" [nav: {stage.feedback_unreadable} feedback message(s) arrived but "
                        "could not be read -- the message shape is not the one this driver "
                        "expects, so this stage's numbers are MISSING, not zero]")
        else:
            nav_note = " [nav: no feedback -- Nav2 said nothing during this stage]"
        self.get_logger().info(f"[{'OK ' if ok else 'STOP'}] {name}: {detail}"
                               + (f" ({stage.elapsed_s:.1f}s)" if stage.elapsed_s else "")
                               + nav_note)
        return stage

    # ------------------------------------------------------------------ #

    def drive_to(self, name: str, label: str) -> Stage:
        """Send one NavigateToPose goal and wait for its RESULT, not its acceptance."""
        x, y, yaw = self.node_pose(name)
        t0 = time.monotonic()
        goal = self.NavigateToPose.Goal()
        goal.pose.header.frame_id = "map"
        # Stamp deliberately left at zero, meaning "latest available transform".
        #
        # This node runs on the wall clock (it has no reason to use sim time), while
        # every Nav2 node runs on /clock. Putting a wall-clock stamp on a sim-time
        # system is a request to extrapolate years into the future, and the error it
        # produces -- "Lookup would require extrapolation" -- points at TF rather than
        # at the stamp that caused it.
        goal.pose.pose.position.x = float(x)
        goal.pose.pose.position.y = float(y)
        goal.pose.pose.orientation.z = float(math.sin(yaw / 2.0))
        goal.pose.pose.orientation.w = float(math.cos(yaw / 2.0))

        # A rejected goal is not a refusal to travel -- but the wait for the server to
        # free up is measured in SECONDS, not in attempts.
        #
        # The first version of this loop allowed three attempts 3 s apart, i.e. nine
        # seconds of patience. In the run of 2026-09-17 the previous driver had been
        # killed at the dispatcher's budget without its nav goal being cancelled; the
        # orphan lived 8 s and this loop gave up 0.1 s before it released -- twice, with
        # the same timings. The count was the wrong unit: the delay is chosen here, so
        # "three attempts" describes nothing about how long the server may stay busy.
        handle = None
        busy_since = time.monotonic()
        attempts = 0
        while True:
            if self._stop or time.monotonic() >= self.run_deadline:
                self.stopped_by = STOP_SIGNAL if self._stop else STOP_RUN_DEADLINE
                return self._set_stage(
                    label, False,
                    self.policy.stopped_detail(self.stopped_by, label,
                                               time.monotonic() - t0), t0)
            attempts += 1
            send = self.nav_client.send_goal_async(goal, feedback_callback=self._on_feedback)
            rclpy.spin_until_future_complete(self, send, timeout_sec=10.0)
            handle = send.result()
            if handle is not None and handle.accepted:
                break
            if self.policy.busy_exhausted(busy_since, time.monotonic()):
                self.stopped_by = STOP_BUSY
                self.busy_waits.append(round(time.monotonic() - busy_since, 3))
                return self._set_stage(
                    label, False,
                    self.policy.busy_detail(name, time.monotonic() - busy_since, attempts))
            self.get_logger().warning(
                f"{label}: goal to {name} not accepted (attempt {attempts}); another "
                f"navigator holds this robot's server. Waiting up to "
                f"{self.policy.busy_budget_s:.0f}s in total, not a fixed number of tries.")
            time.sleep(self.policy.retry_delay_s)

        res = handle.get_result_async()
        # Wait for the RESULT, renewing the permit as we go.
        #
        # spin_until_future_complete with a short timeout returns without completing
        # the future, so calling it in a loop is the ordinary pattern. Renewing from
        # inside the same loop (rather than from a second thread) keeps all rclpy work
        # on one thread, which is what rclpy expects.
        stage_started = time.monotonic()
        deadline = self.policy.stage_deadline(stage_started, self.timeout_s,
                                              self.run_deadline)
        next_renew = time.monotonic() + self.renew_every_s
        while not res.done() and time.monotonic() < deadline:
            rclpy.spin_until_future_complete(self, res, timeout_sec=0.2)
            if self._stop:
                # Release before exiting. This is the whole point: a killed process
                # cannot cancel its goal, and Nav2 then refuses the next driver's.
                self.stopped_by = STOP_SIGNAL
                cancel = handle.cancel_goal_async()
                rclpy.spin_until_future_complete(self, cancel, timeout_sec=5.0)
                return self._set_stage(
                    label, False,
                    self.policy.stopped_detail(STOP_SIGNAL, label,
                                               time.monotonic() - stage_started))
            if self.permit_id and time.monotonic() >= next_renew:
                self.renew_quiet()
                next_renew = time.monotonic() + self.renew_every_s

        if not res.done():
            over_run_budget = time.monotonic() >= self.run_deadline
            # WHICH budget ran out decides the word. A stage that overran its own declared
            # share while the interval simply ran out is an infrastructure result, and
            # CONTRACTS section 9 requires it to be reported as one: the reader's next action
            # for a slow box ("re-run it") is the opposite of the one for a stuck controller
            # ("debug it"). Before this, both printed "timeout after Ns" and the batch could
            # not tell them apart.
            stage_expected_s = 0.0
            if self.expected_passage_s > 0.0 and MOVEMENT_STAGES > 0:
                stage_expected_s = self.expected_passage_s / MOVEMENT_STAGES
            if over_run_budget:
                self.stopped_by = self.policy.classify_stop(
                    label, time.monotonic() - t0, stage_expected_s,
                    self.run_deadline, time.monotonic())
            self.get_logger().warning(f"{label}: timed out after {self.timeout_s}s; cancelling")
            cancel = handle.cancel_goal_async()
            rclpy.spin_until_future_complete(self, cancel, timeout_sec=5.0)
            if over_run_budget:
                if self.stopped_by == STOP_INFRA_TIMEOUT:
                    return self._set_stage(
                        label, False,
                        self.policy.stopped_detail_infra(label, time.monotonic() - t0,
                                                         stage_expected_s), t0)
                return self._set_stage(
                    label, False,
                    self.policy.stopped_detail(STOP_RUN_DEADLINE, label,
                                               time.monotonic() - t0), t0)
            return self._set_stage(label, False, f"timeout after {self.timeout_s}s", t0,
                                   "TIMEOUT")

        status = str(res.result().status) if res.result() is not None else "UNKNOWN"
        # 4 == STATUS_SUCCEEDED in action_msgs/GoalStatus.
        ok = res.result() is not None and res.result().status == 4
        return self._set_stage(label, ok, f"reached {name} (nav status {status})", t0, status)

    # ------------------------------------------------------------------ #

    def acquire(self) -> Stage:
        t0 = time.monotonic()
        if self.request_delay_s:
            self.get_logger().info(
                f"delaying the request by {self.request_delay_s}s to change ARRIVAL ORDER. "
                "The delta only changes who asks first; what happens next is decided by "
                "the reservation book, not by this sleep.")
            time.sleep(self.request_delay_s)
        if not self._wait(self.acquire_cli, "acquire_passage service"):
            return self._set_stage("acquire", False, "acquire_passage service absent", t0)

        deadline = self.policy.stage_deadline(time.monotonic(), self.timeout_s,
                                              self.run_deadline)
        refusal = ""
        while time.monotonic() < deadline:
            if self._stop:
                self.stopped_by = STOP_SIGNAL
                return self._set_stage(
                    "acquire", False,
                    self.policy.stopped_detail(STOP_SIGNAL, "acquire",
                                               time.monotonic() - t0), t0)
            self.acquire_tries += 1
            req = AcquirePassage.Request()
            req.robot_id = self.robot
            req.boot_id = self.boot_id
            req.revision = self.revision
            req.generation = self.generation
            req.task_id = self.task_id
            req.direction = self.direction
            req.request_id = f"{self.task_id}-{self.acquire_tries}"
            req.localization_valid = True
            fut = self.acquire_cli.call_async(req)
            rclpy.spin_until_future_complete(self, fut, timeout_sec=10.0)
            resp = fut.result()
            if resp is None:
                # `resp is None` means the request went unanswered inside a 10 s window. It is
                # NOT the same as a refusal, and the count is what tells them apart: forty
                # iterations in twenty seconds were forty refusals, one per 0.5 s backoff, while
                # a request that never answers costs 10 s on its first try.
                return self._set_stage(
                    "acquire", False,
                    f"no response from acquire_passage after {self.acquire_tries} request(s); "
                    f"the last ANSWERED request was refused: "
                    f"{refusal or '(none answered)'}", t0)
            if resp.granted:
                self.permit_id = resp.permit_id
                # Scope every later renew and clear to the epoch this grant was made
                # under. Renewing under a locally invented epoch is refused as stale.
                self.epoch = int(resp.epoch)
                self.queued_behind = []
                # What the grant named, kept so the retreat geometry can tell a
                # rectangle this robot is entitled to be in from one it is not.
                # `resp.resources` is the bundle the coordinator actually reserved; if
                # the response does not carry it, the configured bundle for this
                # direction is the next best reading, and falling back to it is
                # deliberate rather than silent -- see the retreat report, which names
                # the rectangles it used.
                named = getattr(resp, "resources", None)
                if named:
                    self._granted_resources = tuple(str(n) for n in named)
                else:
                    self._granted_resources = tuple(
                        self.cfg.bundles.get(self.direction, ()) or ())
                return self._set_stage(
                    "acquire", True,
                    f"granted {resp.direction} for {resp.valid_for_wall_s:.2f}s "
                    f"after {self.acquire_tries} request(s)", t0)
            refusal = f"{resp.reason_code}: {resp.detail}"
            self.queued_behind = list(resp.queued_behind)
            # Logged, not just remembered. A queue that is refused forty times and reports only
            # "still queued" hides which gate refused it, and that is the difference between a
            # busy corridor and a corridor that will never be granted to anybody.
            if self.acquire_tries == 1 or self.acquire_tries % 10 == 0:
                self.get_logger().warning(
                    f"acquire {self.acquire_tries}: refused -- {refusal}"
                    + (f"; queued_behind={self.queued_behind}" if self.queued_behind else ""))
            # Retry every refusal except a malformed request. The first version
            # treated only RESOURCE_BUSY as retryable and stopped on everything else,
            # which turned two transient states into a lost crossing in a live run:
            #
            #   LOCALIZATION_STALE  -- one 1.9 s odometry gap on a loaded host, seen
            #                          after 197 legitimate queue retries, against a
            #                          1.5 s freshness limit. The robot was parked.
            #   RESOURCE_UNKNOWN    -- a lapsed bundle, now lifted by the coordinator's
            #                          re-verification pass.
            #
            # Asking again causes no motion: the coordinator judges the request and an
            # unanswered request grants nothing. So a refusal costs only time, and
            # stopping early threw away the rest of the budget for no benefit.
            # INVALID_INPUT is different -- it means this request is malformed, and
            # repeating it cannot help.
            if resp.reason_code == "INVALID_INPUT":
                return self._set_stage("acquire", False, f"refused: {refusal}", t0)
            time.sleep(0.5)
        return self._set_stage(
            "acquire", False,
            f"still queued after {self.timeout_s}s and {self.acquire_tries} request(s); "
            f"last refusal {refusal}; queued_behind={self.queued_behind}", t0)

    def renew(self) -> Stage:
        t0 = time.monotonic()
        req = RenewPermit.Request()
        req.robot_id = self.robot
        req.boot_id = self.boot_id
        req.revision = self.revision
        req.generation = self.generation
        req.epoch = self.epoch
        req.task_id = self.task_id
        req.direction = self.direction
        req.permit_id = self.permit_id
        fut = self.renew_cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=10.0)
        resp = fut.result()
        if resp is None or not resp.granted:
            detail = "" if resp is None else f"{resp.reason_code}: {resp.detail}"
            return self._set_stage("renew", False, f"renew refused: {detail}", t0)
        return self._set_stage("renew", True, f"renewed for {resp.valid_for_wall_s:.2f}s", t0)

    def renew_quiet(self) -> None:
        """Renew without adding a stage, so the report stays readable.

        Failures ARE recorded, because a renewal that quietly stopped working looks
        exactly like a robot that stopped moving for no reason.
        """
        if not self.permit_id:
            return
        req = RenewPermit.Request()
        req.robot_id = self.robot
        req.boot_id = self.boot_id
        req.revision = self.revision
        req.generation = self.generation
        req.epoch = self.epoch
        req.task_id = self.task_id
        req.direction = self.direction
        req.permit_id = self.permit_id
        fut = self.renew_cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=5.0)
        resp = fut.result()
        if resp is None or not resp.granted:
            detail = "no response" if resp is None else f"{resp.reason_code}: {resp.detail}"
            self.renew_failures.append(detail)
            self.get_logger().warning(f"permit renewal failed: {detail}")
            return
        self.renewals += 1
        self.get_logger().info(
            f"permit renewed ({self.renewals}) for {resp.valid_for_wall_s:.2f}s")

    def confirm_clear(self) -> Stage:
        t0 = time.monotonic()
        if self._stop:
            self.stopped_by = STOP_SIGNAL
            return self._set_stage(
                "confirm_clear", False,
                self.policy.stopped_detail(STOP_SIGNAL, "confirm_clear", 0.0))
        if not self._wait(self.clear_cli, "confirm_clear service"):
            return self._set_stage("confirm_clear", False, "confirm_clear service absent", t0)
        req = ConfirmClear.Request()
        req.robot_id = self.robot
        req.boot_id = self.boot_id
        req.revision = self.revision
        req.generation = self.generation
        req.epoch = self.epoch
        req.task_id = self.task_id
        req.direction = self.direction
        fut = self.clear_cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=10.0)
        resp = fut.result()
        if resp is None:
            return self._set_stage("confirm_clear", False, "no response", t0)
        ok = bool(resp.cleared)
        return self._set_stage(
            "confirm_clear", ok,
            resp.detail if ok else f"{resp.reason_code}: {resp.detail} "
                                   f"still_inside={list(resp.still_inside)}", t0)

    # ------------------------------------------------------------------ #

    def request_stop(self) -> None:
        """Ask the driver to stop at the next safe point. Signal-handler safe.

        Sets a flag and returns. No rclpy call is made from here on purpose: the only
        thread allowed to touch rclpy is the one inside `spin_until_future_complete`,
        and cancelling a goal from a signal handler would race it.
        """
        self._stop = True

    def run(self) -> dict:
        self.started_at = time.monotonic()
        # Stop on our own, before the dispatcher's kill. A kill leaves the nav goal
        # running on a server the next driver needs, and loses this report.
        self.run_deadline = self.policy.run_deadline(self.started_at, self.timeout_s)
        budget = max(1.0, min(60.0, self.run_deadline - time.monotonic()))
        if not self.nav_client.wait_for_server(timeout_sec=budget):
            self.stopped_by = STOP_NAV_ABSENT
            self._set_stage("nav2", False, "navigate_to_pose action server absent")
            return self.report()
        self._set_stage("nav2", True, "navigate_to_pose action server available")

        for label, name in (
            ("stage_wait", self.dspec.wait_node),
            ("stage_align", self.dspec.align_node),
        ):
            if not self.drive_to(name, label).ok:
                return self.report()

        acquired = self.acquire()
        if not acquired.ok:
            return self.report()
        # A permit queue is not this driver's movement budget. Measured on N04 (2026-09-18):
        # r02 spent 42.9 s and 86 requests waiting while r01 held the corridor, and then ran out
        # of its 165 s budget 2.6 s into `stage_release` with the permit in its hand -- having
        # done nothing wrong. The granted budget is still a hard ceiling, so the driver stays
        # `release_margin_s` inside the dispatcher's kill either way.
        self.run_deadline = self.policy.after_queue(
            self.run_deadline, acquired.elapsed_s, self.started_at, self.timeout_s)

        # ---- may we still afford the passage? (D-P17-08) ------------------------ #
        # AFTER the grant: the queue credit is applied above, so this is the first moment the
        # remaining budget is the real one. BEFORE stage_cross: the decision is whether to
        # enter a capacity-1 resource, and a robot that enters without the time to leave takes
        # the queue down with it. Refusing here costs a retry; refusing later costs everyone.
        if self.expected_passage_s > 0.0 and not self.policy.reserve_ok(
                self.expected_passage_s, time.monotonic(), self.run_deadline):
            self.stopped_by = STOP_RUN_DEADLINE
            self.insufficient_reserve = True
            return self._set_stage(
                "reserve", False,
                self.policy.reserve_detail(self.expected_passage_s, time.monotonic(),
                                           self.run_deadline, "stage_cross"), t0=0.0)

        for label, name in (
            ("stage_cross", self.dspec.exit_node),
            ("stage_release", self.dspec.release_node),
        ):
            stage = self.drive_to(name, label)
            if stage.ok:
                continue
            # The leg failed. At this point the driver has been told "you could not
            # reach X" and there are two things that produce that sentence, which look
            # identical in the report afterwards: the robot could not get there, or it
            # got there and could not STAY. `retreat_policy` answers the second, and
            # this is the only moment the question can be asked -- the robot is stopped,
            # the permit is in its hand, and the geometry is current (D-P14-02).
            #
            # Only for stage_cross: `stage_release` drives to a node OUTSIDE every
            # rectangle, so a robot that fails it is not inside anything and there is
            # nothing to retreat from.
            if label == "stage_cross":
                retreat = self.retreat_if_stuck(
                    f"{label} could not reach {name} ({stage.detail})")
                if retreat is not None and not retreat.ok:
                    return self.report()
            return self.report()

        self.confirm_clear()
        return self.report()

    def report(self) -> dict:
        return {
            "robot": self.robot,
            "direction": self.direction,
            "task_id": self.task_id,
            "generation": self.generation,
            "permit_id": self.permit_id,
            "acquire_tries": self.acquire_tries,
            "queued_behind": self.queued_behind,
            "renewals": self.renewals,
            "renew_failures": self.renew_failures,
            # `complete: false` never said whether the driver was interrupted, ran out of
            # budget, or found the server busy. Those want different next actions.
            "stopped_by": self.stopped_by,
            "busy_waits_s": list(self.busy_waits),
            "budget_s": round(self.timeout_s, 3),
            "queue_allowance_s": round(self.queue_allowance_s, 3),
            "expected_passage_s": round(self.expected_passage_s, 3),
            "insufficient_reserve": bool(self.insufficient_reserve),
            # The retreat emitter's own record (D-P14-02). Reported even when no retreat
            # was issued, because "we looked and the robot was fine" and "we never looked"
            # are different facts and only one of them is a clean run.
            "retreat_pose_messages": int(self._pose_msgs),
            "retreat_overlap_m": round(float(self._retreat_overlap_m), 4),
            "granted_resources": list(self._granted_resources),
            "run_deadline_s": (round(self.run_deadline - self.started_at, 3)
                               if self.started_at else 0.0),
            "complete": bool(self.stages) and all(s.ok for s in self.stages),
            "stages": [asdict(s) for s in self.stages],
        }


def main(argv: list[str] | None = None) -> int:
    import argparse

    parser = argparse.ArgumentParser(description="Drive one staged corridor crossing.")
    parser.add_argument("--robot", required=True)
    parser.add_argument("--direction", required=True,
                        choices=["west_to_east", "east_to_west"])
    parser.add_argument("--task-id", required=True)
    parser.add_argument("--generation", type=int, default=1)
    parser.add_argument("--timeout-s", type=float, default=90.0)
    parser.add_argument("--expected-passage-s", type=float, default=0.0,
                        help="the scenario's own estimate of how long the passage "
                             "takes; a driver with less budget left than this "
                             "refuses to enter. 0.0 disables the check.")
    parser.add_argument("--request-delay-s", type=float, default=0.0)
    parser.add_argument("--queue-allowance-s", type=float, default=0.0,
                        help="how much permit queueing is excused from the movement "
                             "budget; the scenario declares it")
    parser.add_argument("--report", default="", help="write the JSON report here too")
    args = parser.parse_args(argv)

    rclpy.init(args=None)
    node = None

    def _on_signal(_signum, _frame):
        # Ask the node to stop at its next safe point; do NO rclpy work here.
        #
        # Why a handler at all: SIGTERM's default action ends the process, which (a)
        # leaves the Nav2 goal active for the next driver to collide with and (b) skips
        # the report write below -- so a killed run left no evidence that it had run.
        if node is not None:
            node.request_stop()

    signal.signal(signal.SIGTERM, _on_signal)
    signal.signal(signal.SIGINT, _on_signal)

    try:
        node = StagedCrossing(
            robot=args.robot, direction=args.direction, task_id=args.task_id,
            generation=args.generation, timeout_s=args.timeout_s,
            request_delay_s=args.request_delay_s,
            queue_allowance_s=args.queue_allowance_s,
            expected_passage_s=args.expected_passage_s,
        )
        report = node.run()
    except Exception as exc:
        # A crash is still a run, and it still owes a report: "the driver died" and
        # "the driver never started" are different facts and only one of them is a bug
        # in this file.
        report = {"robot": args.robot, "direction": args.direction,
                  "complete": False, "stages": [], "stopped_by": "exception",
                  "error": f"{type(exc).__name__}: {exc}"}
        text = json.dumps(report, indent=2)
        print(text)
        if args.report:
            Path(args.report).write_text(text, encoding="utf-8")
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        return 3

    text = json.dumps(report, indent=2)
    print(text)
    if args.report:
        Path(args.report).write_text(text, encoding="utf-8")
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
    return 0 if report["complete"] else 4


if __name__ == "__main__":
    raise SystemExit(main())
