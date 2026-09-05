#!/usr/bin/env python3
"""Read-only topic inspection; optionally save actual RGB camera image."""
import json
import math
import sys
import time
from pathlib import Path
import cv2
from cv_bridge import CvBridge
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, CameraInfo, JointState, LaserScan
from std_msgs.msg import String
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry

rclpy.init(); node=rclpy.create_node('mm_inspect'); data={}; images=[]
node.create_subscription(String,'/mm/physics',lambda m:data.update(physics=json.loads(m.data)),10)
node.create_subscription(String,'/mm/status',lambda m:data.update(status=json.loads(m.data)),10)
node.create_subscription(PoseWithCovarianceStamped,'/amcl_pose',lambda m:data.update(amcl=[m.pose.pose.position.x,m.pose.pose.position.y]),qos_profile_sensor_data)
node.create_subscription(Odometry,'/odom',lambda m:data.update(odom=[m.pose.pose.position.x,m.pose.pose.position.y]),qos_profile_sensor_data)
node.create_subscription(JointState,'/joint_states',lambda m:data.update(joints=dict(zip(m.name,m.position))),qos_profile_sensor_data)
node.create_subscription(CameraInfo,'/camera/camera_info',lambda m:data.update(camera_k=list(m.k)),qos_profile_sensor_data)
node.create_subscription(Image,'/camera/image_raw',lambda m:images.append(m) if not images else images.__setitem__(0,m),qos_profile_sensor_data)
node.create_subscription(LaserScan,'/scan',lambda m:data.update(scan={'frame':m.header.frame_id,'rays':len(m.ranges),'min_finite':min((r for r in m.ranges if math.isfinite(r)),default=None)}),qos_profile_sensor_data)
deadline=time.monotonic()+5.
while time.monotonic()<deadline: rclpy.spin_once(node,timeout_sec=.1)
if images:
    data['image']={'width':images[0].width,'height':images[0].height,'encoding':images[0].encoding,'stamp':[images[0].header.stamp.sec,images[0].header.stamp.nanosec]}
    if len(sys.argv)>1:
        out=Path(sys.argv[1]); out.parent.mkdir(parents=True,exist_ok=True)
        cv2.imwrite(str(out),CvBridge().imgmsg_to_cv2(images[0],desired_encoding='bgr8'))
print(json.dumps(data,indent=2))
node.destroy_node(); rclpy.shutdown()
