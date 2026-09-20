cmake_minimum_required(VERSION 3.16)
project(fleet_ros)

# ament_cmake, not ament_python, so the executables land where ROS looks for them.
#
# On this machine ament_python installed the console scripts into <prefix>/bin and added
# that to PATH. `ros2 run fleet_ros gate_node`, `Node(package='fleet_ros',
# executable='gate_node')` in a launch file, and `ros2 pkg executables fleet_ros` all
# resolve under <prefix>/lib/<package>, so all three failed with "No executable found"
# even though the scripts existed and were runnable by name.
#
# install(PROGRAMS ... DESTINATION lib/${PROJECT_NAME}) states the layout directly, which
# removes the dependence on a convention that differs between colcon releases.

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(fleet_interfaces REQUIRED)

ament_python_install_package(${PROJECT_NAME})

# fleet_ros is now: one gate per robot, one leg executor per robot, the corridor
# coordinator (singleton), the fleet task service (singleton), two tools, and one
# odometry-to-transform publisher per robot. The two singletons are singletons because
# the reservation book and the task ledger must each have exactly one writer; running two
# of either is the failure this project is built to avoid.
#
# odom_to_tf is deliberately NOT a singleton: each robot owns the transform from its own
# odom frame to its own body frame, so there is one per robot and they publish different
# edges. What must be a singleton is any single EDGE, and this is what makes each one so.
install(
  PROGRAMS
    scripts/gate_node
    scripts/stop_distance_calibrator
    scripts/coordinator_node
    scripts/staged_crossing
    scripts/nav2_adapter_node
    scripts/task_service_node
    scripts/odom_to_tf
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
