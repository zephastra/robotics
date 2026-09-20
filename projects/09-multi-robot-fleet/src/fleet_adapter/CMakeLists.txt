cmake_minimum_required(VERSION 3.16)
project(fleet_adapter)

# ament_cmake for the same reason as fleet_core: the executable layout must be explicit
# rather than dependent on the install convention of whichever colcon release is used.
#
# This package has no executables. It is installed so the ROS nodes can import the
# safety gate policy without a second copy of the source in the tree.

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

ament_package()
