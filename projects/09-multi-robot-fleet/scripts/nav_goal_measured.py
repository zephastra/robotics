#!/usr/bin/env python3
"""Send a NavigateToPose goal and measure the arrival in the MAP frame, two ways.

Why this and not just the action result: an action result of SUCCEEDED means Nav2's own
goal checker was satisfied. That is a real result, but it is Nav2 marking its own
homework, computed in the same frames and with the same belief state that produced the
motion. So the arrival is measured independently:

  * action result   -- Nav2's opinion
  * amcl_pose       -- the estimator's opinion, in map, read straight off its topic
  * ground truth    -- the simulator's model pose, which is neither of the above

The third one is the only one that can catch a localisation error. Without it, a robot
that mis-locates itself and then drives to where it wrongly believes the goal is will
report a small arrival error and be entirely wrong. Truth is read for VERIFICATION and is
never published to any control topic (AGENTS.md rule 13).

Two reader details, both of which have already cost a run each:

 * Belief comes from the amcl_pose TOPIC, not from a tf2 Buffer. An earlier version used
   a Buffer, which returned None for a whole 60 s window -- "map -> base_link never
   resolved" -- while the same chain resolved instantly in the next process. A 2019-era
   Buffer lookup is a fine tool; it is not a measurement.
 * Every lookup is allowed to return None and is checked before use. Formatting a None
   into a log line raises TypeError, so a lost transform used to crash the run rather than
   being reported as a missing transform.

The truth channel's limitation is stated rather than hidden: the world-level
/world/<world>/dynamic_pose/info delivers every dynamic model with the frame NAMES
dropped, so the model is taken positionally (entry 0). That is valid for ONE robot and
wrong for two, and the report says which one it used.

Exit codes: 0 goal reached, 1 not reached, 2 no action server or no pose, 3 timed out.
"""

from __future__ import annotations

import argparse
import math
import sys
import time

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer, TransformListener

RESULT_NAMES = {
    0: "STATUS_UNKNOWN", 1: "ACCEPTED", 2: "EXECUTING",
    3: "CANCELING", 4: "SUCCEEDED", 5: "CANCELED", 6: "ABORTED",
}


def fmt(p: tuple[float, float] | None) -> str:
    return "None" if p is None else f"({p[0]:+.4f}, {p[1]:+.4f})"


class MeasuredGoal(Node):
    def __init__(self, robot: str, world: str, watch_scan: bool = False) -> None:
        super().__init__("nav_goal_measured",
                         parameter_overrides=[Parameter("use_sim_time", value=True)])
        self.robot = robot
        self.base = f"{robot}/base_link"
        qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, depth=20)

        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.client = ActionClient(self, NavigateToPose, f"/{robot}/navigate_to_pose")
        self._feedback = None

        self.amcl_pose: tuple[float, float] | None = None
        self.amcl_count = 0
        self.create_subscription(
            PoseWithCovarianceStamped, f"/{robot}/amcl_pose", self._on_amcl, qos)

        self.truth: tuple[float, float] | None = None
        self.truth_count = 0
        self.truth_is_positional = False
        self.create_subscription(
            TFMessage, f"/world/{world}/dynamic_pose/info", self._on_truth, qos)

        # Minimum lidar range seen during the run. This is the closest the robot's skin
        # came to any wall, measured by the robot itself rather than inferred from a
        # footprint. `range_min` is 0.12 m; a reading at or near it means something
        # touched the sensor plane, so it is a usable lower bound on clearance even
        # though it is not a contact sensor.
        self.watch_scan = watch_scan
        self.min_range: float | None = None
        self.min_range_at: tuple[float, float] | None = None
        if watch_scan:
            self.create_subscription(
                LaserScan, f"/{robot}/scan", self._on_scan,
                QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, depth=5))

    # ---------------------------------------------------------------- #

    def _on_amcl(self, msg: PoseWithCovarianceStamped) -> None:
        p = msg.pose.pose.position
        self.amcl_pose = (float(p.x), float(p.y))
        self.amcl_count += 1

    def _on_truth(self, msg: TFMessage) -> None:
        if not msg.transforms:
            return
        self.truth_count += 1
        for tf in msg.transforms:
            child = (tf.child_frame_id or "").strip("/")
            if child.endswith("base_footprint") or child.endswith("base_link"):
                self.truth = (float(tf.transform.translation.x),
                              float(tf.transform.translation.y))
                self.truth_is_positional = False
                return
        t = msg.transforms[0].transform.translation
        self.truth = (float(t.x), float(t.y))
        self.truth_is_positional = True

    def _on_scan(self, msg: LaserScan) -> None:
        vals = [r for r in msg.ranges if msg.range_min <= r <= msg.range_max]
        if not vals:
            return
        lo = min(vals)
        if self.min_range is None or lo < self.min_range:
            self.min_range = lo
            self.min_range_at = self.belief()

    # ---------------------------------------------------------------- #

    def belief(self) -> tuple[float, float] | None:
        """Estimator's pose in map. Topic first; the tf buffer is only a fallback."""
        if self.amcl_pose is not None:
            return self.amcl_pose
        for frame in (self.base, f"{self.robot}/base_footprint"):
            try:
                tr = self.buffer.lookup_transform(
                    "map", frame, Time(), timeout=Duration(seconds=0.3))
            except Exception:
                continue
            t = tr.transform.translation
            return (float(t.x), float(t.y))
        return None

    def map_to_odom(self) -> tuple[float, float] | None:
        try:
            tr = self.buffer.lookup_transform(
                "map", f"{self.robot}/odom", Time(), timeout=Duration(seconds=0.5))
        except Exception:
            return None
        t = tr.transform.translation
        return (float(t.x), float(t.y))

    def settle(self, seconds: float) -> None:
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.1)

    def on_feedback(self, msg) -> None:
        self._feedback = msg

    def run(self, x: float, y: float, yaw: float, timeout_s: float,
            settle_s: float, settle_after: float) -> int:
        log = self.get_logger()

        log.info(f"waiting up to {settle_s:.0f}s for a pose and the action server")
        t0 = time.monotonic()
        pose = None
        while time.monotonic() - t0 < settle_s:
            rclpy.spin_once(self, timeout_sec=0.2)
            pose = self.belief()
            if pose is not None and self.client.wait_for_server(timeout_sec=0.0):
                break
        if pose is None:
            log.error(
                "no pose: amcl_pose never arrived and map -> base_link never resolved. "
                f"NOT_RUN (amcl_pose messages: {self.amcl_count}). This is a measurement "
                "failure, not a navigation failure."
            )
            return 2
        if not self.client.wait_for_server(timeout_sec=5.0):
            log.error(f"no action server at /{self.robot}/navigate_to_pose. NOT_RUN.")
            return 2

        mo = self.map_to_odom()
        log.info(f"before: belief = {fmt(pose)}   truth = {fmt(self.truth)}   "
                 f"map->odom = {fmt(mo)}")
        log.info(f"goal  : ({x:+.4f}, {y:+.4f}) yaw {yaw:+.3f}")

        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(x)
        goal.pose.pose.position.y = float(y)
        goal.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.pose.orientation.w = math.cos(yaw / 2.0)

        fut = self.client.send_goal_async(goal, feedback_callback=self.on_feedback)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=15.0)
        handle = fut.result() if fut.done() else None
        if handle is None or not handle.accepted:
            log.error("goal NOT accepted")
            return 1
        log.info("goal accepted")

        started = time.monotonic()
        result_fut = handle.get_result_async()
        next_log = 0.0
        while not result_fut.done():
            rclpy.spin_once(self, timeout_sec=0.1)
            el = time.monotonic() - started
            if el > timeout_s:
                log.error(f"no result after {timeout_s:.0f}s; the robot may still be "
                          "moving. Cancelling -- this is not a stop.")
                handle.cancel_goal_async()
                return 3
            if el >= next_log:
                fb = self._feedback.feedback if self._feedback else None
                dr = getattr(fb, "distance_remaining", None) if fb else None
                log.info(f"  t={el:5.1f}s  belief={fmt(self.belief())}  "
                         f"truth={fmt(self.truth)}  remaining={dr}")
                next_log += 10.0

        wrapper = result_fut.result() if result_fut.done() else None
        status = wrapper.status if wrapper is not None else 0
        name = RESULT_NAMES.get(status, f"STATUS_{status}")
        log.info(f"action result: {name} after {time.monotonic() - started:.1f}s")

        self.settle(settle_after)
        after_b = self.belief()
        after_t = self.truth
        mo_after = self.map_to_odom()
        log.info(f"after : belief = {fmt(after_b)}   truth = {fmt(after_t)}   "
                 f"map->odom = {fmt(mo_after)}")

        if mo is not None and mo_after is not None:
            d = math.dist(mo, mo_after)
            log.info(f"AMCL moved map->odom by {d:.4f} m during the run"
                     + ("  (AMCL IS correcting)" if d > 1e-6
                        else "  (AMCL never corrected during the run)"))

        if after_b is None:
            log.error("belief lost after the run. NOT_RUN.")
            return 2

        err_b = math.dist(after_b, (x, y))
        log.info("=" * 66)
        log.info(f"ARRIVAL ERROR (estimator's own belief): {err_b:.4f} m   "
                 f"(dx {after_b[0] - x:+.4f}, dy {after_b[1] - y:+.4f})")
        log.info(f"  ACTION RESULT: {name}")
        if after_t is not None:
            err_t = math.dist(after_t, (x, y))
            log.info(f"ARRIVAL ERROR (ground truth):           {err_t:.4f} m   "
                     f"(dx {after_t[0] - x:+.4f}, dy {after_t[1] - y:+.4f})")
            log.info(f"  belief minus truth at the end: "
                     f"{math.dist(after_b, after_t):.4f} m"
                     + ("   <-- the estimator is the larger error"
                        if math.dist(after_b, after_t) > max(err_b, err_t) else ""))
            log.info("  The truth figure is the one to quote: the belief figure cannot "
                     "detect a localisation error, because the robot drove to where it "
                     "believed the goal was.")
            if self.truth_is_positional:
                log.info("  Truth was read POSITIONALLY (entry 0 of the world-level "
                         "topic, which drops frame names). Valid for one robot only.")
        else:
            log.info("ARRIVAL ERROR (ground truth):           NOT_RUN -- no truth on the "
                     "world-level dynamic_pose/info topic")
            log.info("  So this run cannot distinguish 'arrived' from 'mis-located'.")
        if self.watch_scan:
            if self.min_range is None:
                log.info("  CLOSEST APPROACH: no scan data received -- NOT_RUN")
            else:
                log.info(f"  CLOSEST APPROACH: {self.min_range:.4f} m "
                         f"(lidar range_min 0.12 m) at map {fmt(self.min_range_at)}")
                log.info("    This bounds clearance from the robot's own sensor, not from "
                         "a contact sensor, so a wheel clipping a corner below the scan "
                         "plane would not appear here.")
        log.info("=" * 66)
        return 0 if (name == "SUCCEEDED" and err_b < 0.5) else 1


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--robot", default="r01")
    ap.add_argument("--world", default="warehouse")
    ap.add_argument("--x", type=float, required=True)
    ap.add_argument("--y", type=float, required=True)
    ap.add_argument("--yaw", type=float, default=0.0)
    ap.add_argument("--timeout", type=float, default=180.0)
    ap.add_argument("--settle", type=float, default=60.0)
    ap.add_argument("--settle-after", type=float, default=4.0)
    ap.add_argument("--watch-scan", action="store_true",
                    help="track the minimum lidar range during the run (closest approach)")
    args, ros_args = ap.parse_known_args(argv)

    rclpy.init(args=ros_args)
    node = MeasuredGoal(args.robot, args.world, watch_scan=args.watch_scan)
    code = 1
    try:
        code = node.run(args.x, args.y, args.yaw, args.timeout, args.settle,
                        args.settle_after)
    except KeyboardInterrupt:
        code = 3
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return code


if __name__ == "__main__":
    raise SystemExit(main())
