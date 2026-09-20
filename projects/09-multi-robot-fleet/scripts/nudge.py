#!/usr/bin/env python3
"""Drive the robot a short distance through the safety gate. READ-ONLY on the repo.

Purpose is diagnostic, not calibration: it answers "is the base able to move at all
right now, and does AMCL react once it moves?". AMCL only runs a filter update after
the robot has moved at least `update_min_d` (0.10 m in this deployment, D-P17-06; stock's
0.25 m is larger than the safety gate can trust) or turned
`update_min_a`; a stationary robot therefore keeps republishing its initial pose
unchanged, which is easy to misread as "AMCL is dead". Moving the base on purpose
separates those two cases.

Publishes on the gate's INPUT topic (cmd_vel_pre_gate), never on cmd_vel, so the gate
still owns the final topic and still applies its own limits. This program is not a
publisher of cmd_vel and cannot bypass the gate.

It publishes a real command stream at 20 Hz rather than shelling out to
`ros2 topic pub`, because on this machine `ros2 topic pub` prints "publishing #N"
while the graph reports Publisher count: 0 and no subscriber ever receives a message.
"""

from __future__ import annotations

import argparse
import math
import sys
import time

import rclpy
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node


class Nudge(Node):
    def __init__(self, robot: str) -> None:
        super().__init__("nudge")
        # TwistStamped, because that is what the gate listens for on this Nav2
        # generation. Publishing plain Twist here would put a second, different type on
        # the same topic and the gate would ignore whichever one it is not bound to.
        self.pub = self.create_publisher(TwistStamped, f"/{robot}/cmd_vel_pre_gate", 10)
        self.odom = None
        self.create_subscription(
            Odometry, f"/{robot}/odom", self._on_odom, 10)

    def _on_odom(self, msg: Odometry) -> None:
        p = msg.pose.pose.position
        self.odom = (p.x, p.y)


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--robot", default="r01")
    ap.add_argument("--speed", type=float, default=0.15)
    ap.add_argument("--seconds", type=float, default=3.0)
    ap.add_argument("--settle", type=float, default=2.5,
                    help="seconds to command zero afterwards, so the stop is measured")
    args, ros_args = ap.parse_known_args(argv)

    rclpy.init(args=ros_args)
    node = Nudge(args.robot)

    t0 = time.monotonic()
    while node.odom is None and time.monotonic() - t0 < 15.0:
        rclpy.spin_once(node, timeout_sec=0.2)
    print(f"odom before: {node.odom}")
    if node.odom is None:
        print("NO ODOM -- cannot measure. Is the bridge up?")
        node.destroy_node()
        rclpy.shutdown()
        return 2
    # odom is relative to where the robot started, so distance travelled is the
    # difference between two odom readings -- not an offset from the spawn x.
    before = node.odom

    print(f"commanding {args.speed:.3f} m/s on /{args.robot}/cmd_vel_pre_gate "
          f"for {args.seconds:.1f}s")
    twist = TwistStamped()
    twist.twist.linear.x = float(args.speed)
    tick = 1.0 / 20.0
    end = time.monotonic() + args.seconds
    while time.monotonic() < end:
        node.pub.publish(twist)
        rclpy.spin_once(node, timeout_sec=0.0)
        time.sleep(tick)

    print(f"commanding zero for {args.settle:.1f}s")
    zero = TwistStamped()
    end = time.monotonic() + args.settle
    while time.monotonic() < end:
        node.pub.publish(zero)
        rclpy.spin_once(node, timeout_sec=0.0)
        time.sleep(tick)

    print(f"odom after : {node.odom}")
    if node.odom is not None:
        # odom is relative to where the robot started, so this is distance travelled,
        # NOT a map coordinate. Comparing it against the spawn x would be wrong.
        print(f"travelled {math.dist(node.odom, before):.4f} m in the odom frame")
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
