#!/usr/bin/env python3
"""
odom_to_tf：把 /odom（Odometry）转成 odom -> base_link 的 TF。
ros_gz_bridge 只转发消息、不发布 TF，所以需要这个小节点补上 TF 链。
TF 链：map -> odom（slam_toolbox 发布）-> base_link（本节点）-> laser_link（静态）
"""
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped


class OdomToTF(Node):
    def __init__(self):
        super().__init__('odom_to_tf')
        self.tf_broadcaster = TransformBroadcaster(self)
        self.sub = self.create_subscription(Odometry, '/odom', self.cb, 10)

    def cb(self, msg):
        t = TransformStamped()
        t.header.stamp = msg.header.stamp
        t.header.frame_id = msg.header.frame_id if msg.header.frame_id else 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z
        t.transform.rotation = msg.pose.pose.orientation
        self.tf_broadcaster.sendTransform(t)


def main():
    rclpy.init()
    node = OdomToTF()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
