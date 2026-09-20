cmake_minimum_required(VERSION 3.16)
project(fleet_core)

# Built with ament_cmake rather than ament_python on purpose.
#
# ament_python on this machine installs console scripts into <prefix>/bin and relies on
# PATH. `ros2 run`, `ros2 launch` (Node(package=..., executable=...)) and `ros2 pkg
# executables` all resolve executables under <prefix>/lib/<package> and therefore find
# nothing. Rather than depend on which of the two conventions a given colcon release
# implements, the layout is stated explicitly here.
#
# ament_python_install_package places the importable package into the interpreter's
# site-packages inside this install prefix, which is what the ROS nodes import.

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

ament_package()
