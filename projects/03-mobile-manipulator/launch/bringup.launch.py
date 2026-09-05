from pathlib import Path
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node


def generate_launch_description():
    root = Path(__file__).resolve().parents[1]
    params = str(root/'config/nav2.yaml')
    actions = [
        Node(package='ros_gz_bridge', executable='parameter_bridge',
             parameters=[{'config_file': str(root/'config/bridge.yaml'), 'use_sim_time': True}]),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': (root/'description/robot.urdf').read_text(), 'use_sim_time': True}]),
        ExecuteProcess(cmd=['python3', '-m', 'mobile_manipulator.runtime'], output='screen'),
    ]
    lifecycle = []
    for package, executable in [('nav2_map_server','map_server'),('nav2_amcl','amcl'),
                                ('nav2_controller','controller_server'),('nav2_planner','planner_server'),
                                ('nav2_behaviors','behavior_server'),('nav2_bt_navigator','bt_navigator')]:
        extra = {'yaml_filename': str(root/'maps/lab.yaml')} if executable == 'map_server' else {}
        actions.append(Node(package=package, executable=executable, name=executable,
                            parameters=[params, extra], output='screen',
                            remappings=[('cmd_vel', '/nav_cmd_vel')]))
        lifecycle.append(executable)
    actions.append(Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
                        name='lifecycle_manager', parameters=[{'use_sim_time': True,
                        'autostart': True, 'node_names': lifecycle, 'bond_timeout': 10.0}]))
    return LaunchDescription(actions)
