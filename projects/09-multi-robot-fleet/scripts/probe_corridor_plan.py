#!/usr/bin/env python3
"""Ask the planner directly whether the 1.2 m gap is plannable, from fixed start poses.

The corridor crossings both failed with
    GridBasedplugin failed to plan from (-0.10, 0.54) ... to (4.00, -2.00):
    "Failed to create plan with tolerance of: 0.500000"
which is ambiguous: it could mean the gap is blocked in the costmap, or that the
robot's BELIEVED pose happened to be inside the wall footprint so the start was invalid.

Those two have completely different fixes, so guessing is expensive. This removes the
ambiguity by asking the planner for paths from explicit start poses that are known to be
in free space, using ComputePathToPose's `start` field rather than the robot's pose:

  A  (0.0, 0.0) -> (2.5, 0.0)     straight through the middle of the gap
  B  (-3.0, 0.0) -> (3.0, 0.0)    the full crossing, aligned with the gap
  C  (-3.0, -2.0) -> (3.0, -2.0)  the crossing a naive planner would attempt, off-axis:
                                  it must find the gap because nothing else connects
  D  (-3.0, -3.0) -> (-1.0, -3.0) control: plans entirely on one side. If D fails, the
                                  problem is not the gap at all.

A succeeding A/B/C means the costmap allows it and the earlier failure was about the
robot's pose. A failing A means the gap is not passable in the costmap as configured --
which is a configuration or world-geometry answer, not a control-tuning answer.

Read-only: this asks for paths and moves nothing.
"""

from __future__ import annotations

import argparse
import math
import sys
import time

import rclpy
from nav2_msgs.action import ComputePathToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import Parameter

CASES = [
    ("A  gap centre, aligned", (0.0, 0.0), (2.5, 0.0)),
    ("B  full crossing, aligned", (-3.0, 0.0), (3.0, 0.0)),
    ("C  full crossing, off-axis", (-3.0, -2.0), (3.0, -2.0)),
    ("D  control: same side only", (-3.0, -3.0), (-1.0, -3.0)),
    ("E  control: gap from 0.55 m off-centre", (-1.0, 0.55), (1.0, 0.55)),
]


class PlannerProbe(Node):
    def __init__(self, robot: str) -> None:
        super().__init__("planner_probe",
                         parameter_overrides=[Parameter("use_sim_time", value=True)])
        self.client = ActionClient(self, ComputePathToPose,
                                   f"/{robot}/compute_path_to_pose")

    def plan(self, start: tuple[float, float], goal: tuple[float, float],
             timeout: float = 25.0) -> tuple[bool, str]:
        goal_msg = ComputePathToPose.Goal()
        goal_msg.use_start = True
        goal_msg.start.header.frame_id = "map"
        goal_msg.start.pose.position.x = start[0]
        goal_msg.start.pose.position.y = start[1]
        goal_msg.start.pose.orientation.w = 1.0
        goal_msg.goal.header.frame_id = "map"
        goal_msg.goal.pose.position.x = goal[0]
        goal_msg.goal.pose.position.y = goal[1]
        goal_msg.goal.pose.orientation.w = 1.0

        fut = self.client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=timeout)
        handle = fut.result()
        if handle is None or not handle.accepted:
            return False, "goal not accepted"
        res_fut = handle.get_result_async()
        rclpy.spin_until_future_complete(self, res_fut, timeout_sec=timeout)
        wrapper = res_fut.result()
        if wrapper is None:
            return False, "no result (timed out)"
        status = wrapper.status
        if status != 4:
            return False, f"status {status}"
        poses = wrapper.result.path.poses if wrapper.result is not None else []
        if not poses:
            return False, "plan has 0 poses"
        lens = 0.0
        for a, b in zip(poses, poses[1:]):
            lens += math.dist((a.pose.position.x, a.pose.position.y),
                              (b.pose.position.x, b.pose.position.y))
        return True, f"{len(poses)} poses, {lens:.3f} m"


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--robot", default="r01")
    args, ros_args = ap.parse_known_args(argv)

    rclpy.init(args=ros_args)
    node = PlannerProbe(args.robot)
    if not node.client.wait_for_server(timeout_sec=10.0):
        print(f"no compute_path_to_pose server under /{args.robot}. NOT_RUN.")
        return 2

    # Give the costmap a moment; it publishes with transient_local durability.
    end = time.monotonic() + 5.0
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.2)

    print("=" * 78)
    print("  is the 1.2 m gap plannable?  (explicit start poses, robot does not move)")
    print("=" * 78)
    ok_count = 0
    for label, start, goal in CASES:
        ok, detail = node.plan(start, goal)
        ok_count += 1 if ok else 0
        flag = "PLAN OK " if ok else "NO PLAN "
        print(f"  [{flag}] {label:44s} from {start} to {goal}")
        print(f"             -> {detail}")
    print()
    print(f"  {ok_count}/{len(CASES)} plannable")
    print("  A/B failing  = the gap is NOT passable in the costmap as configured.")
    print("  A/B passing  = the gap IS passable; the crossing failures were about the")
    print("                 robot's believed pose, not the costmap.")

    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
