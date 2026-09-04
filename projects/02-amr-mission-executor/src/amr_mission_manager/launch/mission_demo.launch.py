from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
from pathlib import Path


def generate_launch_description():
    package_share = Path(get_package_share_directory("amr_mission_manager"))
    default_mission = str(package_share / "config" / "warehouse_demo.yaml")

    mission_file = LaunchConfiguration("mission_file")
    report_directory = LaunchConfiguration("report_directory")
    battery_initial = LaunchConfiguration("battery_initial_percentage")
    battery_drain = LaunchConfiguration("battery_drain_per_second")

    return LaunchDescription(
        [
            DeclareLaunchArgument("mission_file", default_value=default_mission),
            DeclareLaunchArgument(
                "report_directory", default_value="/tmp/amr_mission_reports"
            ),
            DeclareLaunchArgument("battery_initial_percentage", default_value="100.0"),
            DeclareLaunchArgument("battery_drain_per_second", default_value="0.05"),
            Node(
                package="amr_mission_manager",
                executable="battery_simulator",
                name="battery_simulator",
                output="screen",
                parameters=[
                    {
                        "initial_percentage": ParameterValue(
                            battery_initial, value_type=float
                        ),
                        "drain_per_second": ParameterValue(
                            battery_drain, value_type=float
                        ),
                    }
                ],
            ),
            Node(
                package="amr_mission_manager",
                executable="mission_manager",
                name="mission_manager",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": True,
                        "mission_file": mission_file,
                        "report_directory": report_directory,
                    }
                ],
            ),
        ]
    )
