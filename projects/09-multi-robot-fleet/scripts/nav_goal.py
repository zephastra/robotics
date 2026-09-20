#!/usr/bin/env python3
"""Send one NavigateToPose goal and report what actually happened.

Why a script rather than `ros2 action send_goal`: on this machine the ROS CLI tools are
not reliable for driving anything. `ros2 topic pub` printed "publishing #1..#6" while the
graph showed `Publisher count: 0` and no subscriber received a message. A real rclpy
node has no such problem, and the same distinction matters for goals: a goal that was
"sent" is not a goal that was accepted, and an accepted goal is not a goal that was
reached. This program reports all three separately.

Exit codes:
    0  goal reached (the action server reported SUCCEEDED)
    1  the action server rejected or aborted the goal
    2  no action server (Nav2 not running, or the namespace is wrong)
    3  timed out waiting for a result -- the robot may still be moving
"""

from __future__ import annotations

import argparse
import math
import sys
import time

import rclpy
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node

# action_msgs/GoalStatus codes. The first version of this file had 1=SUCCEEDED, which
# is STATUS_ACCEPTED: an ABORTED goal (6) was reported as "STATUS_6" instead of
# "ABORTED", and a genuinely accepted-but-still-running goal would have been announced
# as a success. Getting these wrong is how a navigation test lies about the result.
RESULT_NAMES = {
    0: "STATUS_UNKNOWN",
    1: "ACCEPTED",
    2: "EXECUTING",
    3: "CANCELING",
    4: "SUCCEEDED",
    5: "CANCELED",
    6: "ABORTED",
}


class NavGoal(Node):
    def __init__(self, robot: str) -> None:
        super().__init__("nav_goal")
        self.robot = robot
        # Absolute action name, built from --robot and nothing else.
        #
        # A RELATIVE name resolves against this node's namespace, which is the root, so
        # it would look for /navigate_to_pose while the server lives at
        # /r01/navigate_to_pose. The symptom is "waiting for the action server" and then
        # a timeout, with nothing in it to suggest a namespace mismatch -- which is
        # exactly how the first attempt at this step failed.
        self.action_name = f"/{robot}/navigate_to_pose"
        self.client = ActionClient(self, NavigateToPose, self.action_name)
        # Feedback arrives through a callback, not through the goal handle:
        # rclpy's ClientGoalHandle exposes accept/cancel/result but has no `feedback`
        # attribute. Reading handle.feedback raises AttributeError, which is how the
        # first attempt at this step died AFTER the goal had been accepted -- so the
        # robot was navigating while the program that asked for it had already crashed.
        self._last_feedback = None

    def _on_feedback(self, msg) -> None:
        self._last_feedback = msg

    def _progress(self) -> str:
        if self._last_feedback is None:
            return "no feedback yet"
        fb = self._last_feedback.feedback
        bits = []
        for name in ("distance_remaining", "navigation_time", "estimated_time_remaining",
                     "number_of_recoveries"):
            if hasattr(fb, name):
                bits.append(f"{name}={getattr(fb, name)}")
        return ", ".join(bits) if bits else "feedback received"

    def send(self, x: float, y: float, yaw: float, timeout_s: float) -> int:
        log = self.get_logger()

        log.info(f"waiting for action server {self.action_name} ...")
        if not self.client.wait_for_server(timeout_sec=20.0):
            log.error(
                f"no action server at {self.action_name}. Either Nav2 is not running, or "
                "it is in a different namespace. NOT_RUN -- this does not mean the robot "
                "is unable to navigate."
            )
            return 2

        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(x)
        goal.pose.pose.position.y = float(y)
        goal.pose.pose.position.z = 0.0
        goal.pose.pose.orientation.z = math.sin(float(yaw) / 2.0)
        goal.pose.pose.orientation.w = math.cos(float(yaw) / 2.0)

        log.info(f"goal -> ({x:.3f}, {y:.3f}) yaw {yaw:.3f} rad, frame 'map'")

        send_future = self.client.send_goal_async(goal, feedback_callback=self._on_feedback)
        rclpy.spin_until_future_complete(self, send_future, timeout_sec=20.0)
        handle = send_future.result()
        if handle is None or not handle.accepted:
            log.error("goal was NOT accepted by the action server")
            return 1
        log.info("goal accepted")

        started = time.monotonic()
        result_future = handle.get_result_async()

        # Poll rather than block, so a stall is visible while it is happening rather
        # than only as a timeout at the end.
        next_report = 0.0
        while not result_future.done():
            rclpy.spin_once(self, timeout_sec=0.2)
            elapsed = time.monotonic() - started
            if elapsed > timeout_s:
                log.error(
                    f"no result after {timeout_s:.0f}s. The robot may STILL BE MOVING; "
                    "this is not a stop. Cancelling the goal."
                )
                handle.cancel_goal_async()
                return 3
            if elapsed >= next_report:
                log.info(f"  t={elapsed:5.1f}s  {self._progress()}")
                next_report += 10.0

        wrapper = result_future.result()
        status = wrapper.status if wrapper is not None else 0
        name = RESULT_NAMES.get(status, f"STATUS_{status}")
        log.info(f"result: {name} after {time.monotonic() - started:.1f}s")

        if name == "SUCCEEDED":
            log.info("GOAL REACHED")
            return 0
        log.error(f"goal did not succeed: {name}")
        return 1


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--robot", default="r01", help="robot namespace, e.g. r01")
    ap.add_argument("--x", type=float, required=True)
    ap.add_argument("--y", type=float, required=True)
    ap.add_argument("--yaw", type=float, default=0.0)
    ap.add_argument("--timeout", type=float, default=120.0,
                    help="seconds to wait for a result")
    args, ros_args = ap.parse_known_args(argv)

    rclpy.init(args=ros_args)
    node = NavGoal(args.robot)
    code = 2
    try:
        code = node.send(args.x, args.y, args.yaw, args.timeout)
    except KeyboardInterrupt:
        code = 3
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return code


if __name__ == "__main__":
    raise SystemExit(main())
