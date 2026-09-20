#!/usr/bin/env python3
"""Print the robot's pose from odom and from AMCL at full precision. READ-ONLY.

Both sources matter and they answer different questions:

  odom  -- where the wheels think they are. Always moving, never corrected.
  amcl  -- where the map thinks they are. Only changes if AMCL actually fused a scan.

Comparing the two across two samples is how "AMCL is dead" is told apart from "AMCL is
alive but the base never moved". Full precision is deliberate: a rounded printout hides
exactly the sub-millimetre movement that proves a filter update happened.
"""

from __future__ import annotations

import argparse
import math
import sys
import time

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node


class Sampler(Node):
    def __init__(self, robot: str) -> None:
        super().__init__("pose_sampler")
        self.odom = None
        self.amcl = None
        self.amcl_stamp = None
        self.create_subscription(Odometry, f"/{robot}/odom", self._on_odom, 10)
        self.create_subscription(
            PoseWithCovarianceStamped, f"/{robot}/amcl_pose", self._on_amcl, 10)

    def _on_odom(self, msg: Odometry) -> None:
        p = msg.pose.pose.position
        self.odom = (p.x, p.y)

    def _on_amcl(self, msg: PoseWithCovarianceStamped) -> None:
        p = msg.pose.pose.position
        self.amcl = (p.x, p.y)
        self.amcl_stamp = msg.header.stamp

    def spin_for(self, seconds: float) -> None:
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.1)


def fmt(v) -> str:
    return "None" if v is None else f"({v[0]:.9f}, {v[1]:.9f})"


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--robot", default="r01")
    ap.add_argument("--samples", type=int, default=2)
    ap.add_argument("--gap", type=float, default=3.0)
    args, ros_args = ap.parse_known_args(argv)

    rclpy.init(args=ros_args)
    node = Sampler(args.robot)

    seen: list[tuple] = []
    for i in range(args.samples):
        node.spin_for(6.0 if i == 0 else args.gap)
        seen.append((node.odom, node.amcl))
        print(f"  sample {i + 1}:  odom={fmt(node.odom)}   amcl_pose={fmt(node.amcl)}")

    print()
    if len(seen) >= 2 and seen[0][1] is not None and seen[-1][1] is not None:
        d = math.dist(seen[0][1], seen[-1][1])
        do = math.dist(seen[0][0], seen[-1][0]) if seen[0][0] and seen[-1][0] else 0.0
        print(f"  amcl_pose moved {d:.9f} m across samples; odom moved {do:.9f} m")
        if d == 0.0 and do == 0.0:
            print("  -> BOTH frozen. The base did not move, so nothing here proves "
                  "anything about AMCL. Move the base first.")
        elif d == 0.0 and do > 0.0:
            print("  -> odom moved but amcl_pose did NOT: AMCL is not updating. REAL BUG.")
        elif d > 0.0:
            print("  -> amcl_pose changed: AMCL IS updating. Localisation works.")
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
