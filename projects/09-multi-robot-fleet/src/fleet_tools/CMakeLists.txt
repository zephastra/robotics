cmake_minimum_required(VERSION 3.16)
project(fleet_tools)

# Same ament_cmake choice as the other packages: the executable layout is stated
# explicitly rather than inherited from a colcon convention.
#
# No fleet behaviour depends on this package. `fleet_cli` is the only supported way
# for a human to change fleet state, `fleet_dashboard` is read-only by construction,
# and `fleet_cli events` works with no ROS at all so a timeline can be read after the
# fleet has been torn down.

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(fleet_interfaces REQUIRED)
find_package(visualization_msgs REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(
  PROGRAMS
    scripts/fleet_cli
    scripts/fleet_dashboard
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
