"""Odom TF and fail-closed velocity interlock; no simulation truth for navigation."""
import json
import time
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
from nav_msgs.msg import Odometry, OccupancyGrid
from visualization_msgs.msg import MarkerArray
from sensor_msgs.msg import JointState, LaserScan
from geometry_msgs.msg import TransformStamped, Twist
from std_msgs.msg import String
from tf2_ros import TransformBroadcaster
from .kinematics import inverse, forward, JOINT_NAMES, TRAY
from .scan_filter import filter_self_returns
from .map_visual import map_markers


class Runtime(Node):
    def __init__(self):
        super().__init__('mm_runtime', parameter_overrides=[rclpy.parameter.Parameter('use_sim_time', value=True)])
        self.tf = TransformBroadcaster(self)
        self.joints = {}; self.last_joint = 0.; self.last_permit = 0.; self.permit = False
        self.cmd = Twist(); self.last_cmd = 0.
        self.stow = dict(zip(('shoulder','elbow','lift','wrist'), inverse(TRAY[0], TRAY[1], .85)))
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.scan_pub = self.create_publisher(LaserScan, '/scan', qos_profile_sensor_data)
        map_qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.map_pub=self.create_publisher(MarkerArray,'/mm/map_visual',map_qos)
        self.create_subscription(OccupancyGrid,'/map',lambda m:self.map_pub.publish(map_markers(m)),map_qos)
        self.create_subscription(LaserScan, '/scan_raw', self.scan, qos_profile_sensor_data)
        self.create_subscription(Odometry, '/odom', self.odom, qos_profile_sensor_data)
        self.create_subscription(JointState, '/joint_states', self.joint, qos_profile_sensor_data)
        self.create_subscription(String, '/mm/drive_permit', self.permission, 10)
        self.create_subscription(Twist, '/nav_cmd_vel', self.velocity, 10)
        self.create_timer(.05, self.tick, clock=rclpy.clock.Clock(clock_type=rclpy.clock.ClockType.STEADY_TIME))

    def odom(self, m):
        t = TransformStamped(); t.header = m.header; t.child_frame_id = m.child_frame_id
        t.transform.translation.x = m.pose.pose.position.x
        t.transform.translation.y = m.pose.pose.position.y
        t.transform.translation.z = m.pose.pose.position.z
        t.transform.rotation = m.pose.pose.orientation; self.tf.sendTransform(t)

    def joint(self, m):
        self.joints = dict(zip(m.name, m.position)); self.last_joint = time.monotonic()

    def scan(self, m):
        tool=forward([self.joints[n] for n in JOINT_NAMES[:4]]) if all(n in self.joints for n in JOINT_NAMES[:4]) else None
        m.ranges=filter_self_returns(m.ranges,m.angle_min,m.angle_increment,tool)
        self.scan_pub.publish(m)

    def permission(self, m):
        self.permit = m.data == 'drive'; self.last_permit = time.monotonic()

    def velocity(self, m):
        self.cmd = m; self.last_cmd = time.monotonic()

    def tick(self):
        now = time.monotonic()
        ready = all(abs(self.joints.get(n, 999)-v) < .035 for n,v in self.stow.items())
        okay = self.permit and ready and now-self.last_permit < .7 and now-self.last_joint < .7 and now-self.last_cmd < .5
        self.pub.publish(self.cmd if okay else Twist())


def main():
    rclpy.init(); node = Runtime()
    try: rclpy.spin(node)
    except (KeyboardInterrupt,ExternalShutdownException): pass
    finally:
        if rclpy.ok(): node.pub.publish(Twist())
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()


if __name__ == '__main__': main()
