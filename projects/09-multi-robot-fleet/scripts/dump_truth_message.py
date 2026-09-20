#!/usr/bin/env python3
"""Dump one raw message from the bridged world pose topic. Diagnostic only.

The bridge now maps gz.msgs.Pose_V -> tf2_msgs/msg/TFMessage for
/world/<world>/dynamic_pose/info, but every child_frame_id arrives empty, so the names
are somewhere else in the message. Guessing which field holds them costs another 80 s
bring-up per guess, so this prints the whole first message once and stops guessing.
"""

from __future__ import annotations

import argparse
import math
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy
from tf2_msgs.msg import TFMessage


class Dumper(Node):
    def __init__(self, topic: str) -> None:
        super().__init__("truth_dump",
                         parameter_overrides=[Parameter("use_sim_time", value=True)])
        self.got = None
        self.create_subscription(TFMessage, topic, self._on, 
                                 QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                                            depth=10))

    def _on(self, msg: TFMessage) -> None:
        if self.got is None:
            self.got = msg


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--topic", default="/world/warehouse/dynamic_pose/info")
    ap.add_argument("--seconds", type=float, default=20.0)
    args, ros_args = ap.parse_known_args(argv)

    rclpy.init(args=ros_args)
    node = Dumper(args.topic)
    end = time.monotonic() + args.seconds
    while node.got is None and time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.2)

    print("=" * 78)
    print(f"  raw dump of {args.topic}")
    print("=" * 78)
    if node.got is None:
        print("  NO MESSAGE in the window -- the topic is not bridged or not publishing")
        return 1

    msg = node.got
    print(f"  transforms in message: {len(msg.transforms)}")
    for i, tr in enumerate(msg.transforms[:6]):
        print(f"  --- transform[{i}] ---")
        print(f"    header.frame_id   : {tr.header.frame_id!r}")
        print(f"    child_frame_id    : {tr.child_frame_id!r}")
        print(f"    header.stamp      : {tr.header.stamp.sec}.{tr.header.stamp.nanosec}")
        t = tr.transform.translation
        r = tr.transform.rotation
        print(f"    translation       : ({t.x:+.4f}, {t.y:+.4f}, {t.z:+.4f})")
        # Rotation is printed raw, not as a yaw. Whether this mapping carries rotation at
        # all has to be established before any claim about the robot turning can be made:
        # a constant yaw of 0.00 is either a robot that never turns or an extractor that
        # reads nothing, and those are opposite conclusions.
        print(f"    rotation (quat)   : ({r.x:+.6f}, {r.y:+.6f}, {r.z:+.6f}, {r.w:+.6f})")
        yaw = math.degrees(math.atan2(2.0 * (r.w * r.z + r.x * r.y),
                                      1.0 - 2.0 * (r.y * r.y + r.z * r.z)))
        print(f"    -> yaw            : {yaw:+.3f} deg")
    if len(msg.transforms) > 6:
        print(f"  ... {len(msg.transforms) - 6} more")
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
