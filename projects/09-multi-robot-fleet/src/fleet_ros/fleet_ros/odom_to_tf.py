#!/usr/bin/env python3
"""Publish `<ns>/odom -> <ns>/base_footprint` from the bridged odometry.

WHY THIS NODE EXISTS
--------------------
The obvious way to get robot odometry into the ROS transform tree is to bridge
gz's own pose topic:

    /<ns>/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V

That is what this project did, and on one robot it is fine. On three robots it is
not, and the failure is severe enough to be worth this file.

WHAT GOES WRONG
---------------
`ros_gz_bridge` is launched once per robot, each instance with its OWN gz node,
its OWN subscription to its OWN `/model/<robot>/tf` topic, and its own callback
queue. All of them publish into the ONE shared `/tf` topic, because TF is a single
tree. Three independent producers, each stamping its message from the gz clock at
the moment its own callback runs, therefore arrive at every consumer in an order
that is not their timestamp order. Under load the skew is milliseconds. That is
enough: tf2 requires monotonic arrival per child frame, and when it sees a message
older than one it has already accepted it does not merely drop it —

    [tf2_buffer]: Detected jump back in time. Clearing TF buffer.

it discards its entire cache. The old edge is gone. A listener that then asks for
`<ns>/odom`, which was only ever published by that stream, is told

    Invalid frame ID "<ns>/odom" passed to canTransform argument target_frame
        - frame does not exist

and a `tf2_ros::MessageFilter` waiting on it throws

    tf2::LookupException: Static cache is empty, when looking up transform from
        frame [<ns>/laser_link/scan] to frame [<ns>/odom]

Nothing in nav2 catches that exception, so the process calls std::terminate and
aborts with signal 6. Measured on this machine, 3 robots brought up together and
driven for 30 s:

    attempt 1   0 crashes
    attempt 2   6 crashed nodes: amcl x3, controller_server x2, planner_server x1
    attempt 3   4 crashed nodes: amcl x3, controller_server x1

The victims are not only the localiser. Any tf2 consumer can be the one holding
the stale request when the buffer is cleared. Losing amcl means `amcl_pose` never
arrives, the safety gate is left with no localised pose to judge from, and the run
is worthless even though the robots themselves are fine.

THE FIX
-------
Do not bridge the gz transform stream at all. `/<ns>/odom` is already bridged as
`nav_msgs/Odometry`, and that message carries the pose, the child frame and the
stamp. This node subscribes to it and republishes the one edge

    <ns>/odom -> <ns>/base_footprint

which is the only edge that stream ever produced (the URDF has no `odom` link, so
`robot_state_publisher` cannot publish it; it belongs to the odometry source). One
robot, one publisher, one edge, one clock. The ordering hazard disappears because
there is no longer more than one writer to interleave.

WHAT THIS NODE DELIBERATELY DOES NOT DO
---------------------------------------
* It does not publish `map -> odom`. That is AMCL's, and AMCL keeps it.
* It does not use `tf_broadcast` or any transform cache. It copies a pose.
* It does not interpolate or extrapolate. The stamp is the odometry stamp; if a
  consumer wants a time in between it can ask tf2, which now has a well-ordered
  history to interpolate within.

Frames are taken from the incoming message header rather than from parameters
wherever possible: `header.frame_id` and `child_frame_id` are already the right
strings (gz's DiffDrive is configured with `<frame_id>__ROBOT__/odom</frame_id>`
and `<child_frame_id>__ROBOT__/base_footprint</child_frame_id>`), and reading them
back means the launch file cannot drift out of step with the model SDF. The
parameters exist to override, not to be the source of truth.
"""

from __future__ import annotations

import sys
from typing import Any

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from tf2_msgs.msg import TFMessage


class OdomToTf(Node):
    """Republish odometry as the single `odom -> base_footprint` edge."""

    def __init__(self) -> None:
        super().__init__("odom_to_tf")

        # Defaults are best-effort only; both are overridden from the message.
        self.declare_parameter("odom_topic", "odom")
        self.declare_parameter("parent_frame", "")
        self.declare_parameter("child_frame", "")
        self.declare_parameter("tf_topic", "/tf")

        self._parent_param = str(self.get_parameter("parent_frame").value)
        self._child_param = str(self.get_parameter("child_frame").value)

        odom_topic = str(self.get_parameter("odom_topic").value)
        tf_topic = str(self.get_parameter("tf_topic").value)

        # Sensor-data QoS on the way in: gz's bridge publishes odometry as a streaming
        # best-effort sensor topic, and a RELIABLE subscription would still work but
        # would add retransmission machinery for data that is worthless once superseded.
        odom_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
        )
        # Reliable on the way out. TF is not sensor data to be dropped; it is the
        # transform tree, and a consumer that misses an edge looks exactly like a
        # consumer whose tree is broken -- which is the failure this file removes.
        tf_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=100,
        )

        self._pub = self.create_publisher(TFMessage, tf_topic, tf_qos)
        self._sub = self.create_subscription(Odometry, odom_topic, self._on_odom, odom_qos)

        self._count = 0
        self._last_parent = ""
        self._last_child = ""
        # A rate-limited heartbeat, because a node that silently publishes nothing is
        # indistinguishable from a node that was never started. Printed once at startup
        # and then only when the frames it is using change.
        self.get_logger().info(
            f"odom_to_tf: {odom_topic} -> {tf_topic} "
            f"(parent={self._parent_param or '<from message>'}, "
            f"child={self._child_param or '<from message>'})"
        )

    def _on_odom(self, msg: Odometry) -> None:
        parent = self._parent_param or msg.header.frame_id
        child = self._child_param or msg.child_frame_id

        if not parent or not child:
            # Loud, once per change, rather than a silent no-op. An odometry message
            # with empty frame names cannot produce a usable edge, and finding that out
            # from a downstream "frame does not exist" costs an afternoon.
            self.get_logger().error(
                f"odom_to_tf: refusing to publish an edge with empty frame names "
                f"(parent={parent!r}, child={child!r})"
            )
            return

        if (parent, child) != (self._last_parent, self._last_child):
            self.get_logger().info(f"odom_to_tf: publishing {parent} -> {child}")
            self._last_parent, self._last_child = parent, child

        t = TransformStamped()
        t.header.stamp = msg.header.stamp
        t.header.frame_id = parent
        t.child_frame_id = child
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z
        t.transform.rotation = msg.pose.pose.orientation

        out = TFMessage()
        out.transforms = [t]
        self._pub.publish(out)
        self._count += 1


def main(argv: list[str] | None = None) -> int:
    rclpy.init(args=argv)
    node = OdomToTf()
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
    sys.exit(main())
