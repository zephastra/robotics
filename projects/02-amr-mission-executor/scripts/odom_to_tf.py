#!/usr/bin/env python3
"""Publish odom -> base_link from Gazebo's Odometry message."""

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from tf2_ros import TransformBroadcaster


class OdomToTF(Node):
    def __init__(self) -> None:
        super().__init__("warehouse_odom_to_tf")
        self.broadcaster = TransformBroadcaster(self)
        self.create_subscription(Odometry, "/odom", self._publish_tf, 20)

    def _publish_tf(self, message: Odometry) -> None:
        transform = TransformStamped()
        transform.header.stamp = message.header.stamp
        transform.header.frame_id = message.header.frame_id or "odom"
        transform.child_frame_id = "base_link"
        transform.transform.translation.x = message.pose.pose.position.x
        transform.transform.translation.y = message.pose.pose.position.y
        transform.transform.translation.z = message.pose.pose.position.z
        transform.transform.rotation = message.pose.pose.orientation
        self.broadcaster.sendTransform(transform)


def main() -> None:
    rclpy.init()
    node = OdomToTF()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    main()
