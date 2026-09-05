#!/usr/bin/env python3
"""Check platform packages only; never installs or changes system settings."""
import importlib
import os
from pathlib import Path
import sys
from ament_index_python.packages import get_package_prefix

errors=[]
for package in ('ros_gz_bridge','robot_state_publisher','rviz2','nav2_map_server','nav2_amcl',
                'nav2_controller','nav2_planner','nav2_behaviors','nav2_bt_navigator',
                'nav2_navfn_planner','nav2_regulated_pure_pursuit_controller','nav2_lifecycle_manager'):
    try: get_package_prefix(package)
    except Exception as exc: errors.append(f'{package}: {exc}')
for module in ('rclpy','cv2','numpy','yaml','pytest','cv_bridge','tf2_ros'):
    try: importlib.import_module(module)
    except ImportError as exc: errors.append(str(exc))
for path in ('/opt/ros/lyrical/opt/gz_sim_vendor/lib/cmake/gz-sim',
             '/opt/ros/lyrical/opt/gz_plugin_vendor/lib/cmake/gz-plugin'):
    if not Path(path).is_dir(): errors.append('Missing Gazebo development dependency: '+path)
if errors:
    print('\n'.join(errors),file=sys.stderr)
    raise SystemExit(1)
print(f'Environment OK: ROS {os.environ.get("ROS_DISTRO")}, Python {sys.version.split()[0]}, domain {os.environ.get("ROS_DOMAIN_ID")}')
print('No physical hardware or other project workspace is required.')
