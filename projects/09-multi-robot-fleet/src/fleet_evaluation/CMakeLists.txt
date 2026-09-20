cmake_minimum_required(VERSION 3.16)
project(fleet_evaluation)

# The evaluator is split so the half that MATTERS does not need ROS.
#
# `judge.py` -- the actual judging -- is pure Python: no rclpy, no simulator. It runs on a
# run directory produced earlier, on a machine that has no ROS installed, by somebody who
# was not there when the run happened. A judge that needs the thing it is judging in order
# to run is not independent, so this is a structural property and not a preference.
#
# `recorder.py` does need ROS. It is a separate module reached only through its own entry
# point, so the rclpy dependency cannot leak into the judging path.
#
# rclpy is declared here because the recorder lives in this package. It is NOT imported by
# fleet_evaluation/__init__.py or by judge.py; keep it that way.

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(fleet_core REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(
  PROGRAMS
    scripts/fleet_eval
    scripts/fleet_recorder
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
