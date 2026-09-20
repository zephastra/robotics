cmake_minimum_required(VERSION 3.16)
project(fleet_interfaces)

# Standard rosidl generation. The interfaces are generated, so this package has no
# install(PROGRAMS) section and no Python package of its own.
#
# Not ament_python: rosidl needs the build type ament_cmake, and this machine has
# already produced one confusing failure from a package whose declared build type
# did not match its build files.
#
# The P4 protocol (ResourcePermit + AcquirePassage/RenewPermit/ConfirmClear) and
# the P5 task surface (SubmitTask/CancelTask/FaultInject + ExecuteLeg) live in the
# same package on purpose: they are one interface contract for one fleet, and
# splitting them would let the two halves drift into different schema versions.

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(unique_identifier_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ResourcePermit.msg"
  "msg/RobotState.msg"
  "srv/AcquirePassage.srv"
  "srv/RenewPermit.srv"
  "srv/ConfirmClear.srv"
  "srv/SubmitTask.srv"
  "srv/CancelTask.srv"
  "srv/FaultInject.srv"
  "action/ExecuteLeg.action"
  DEPENDENCIES builtin_interfaces std_msgs unique_identifier_msgs
)

ament_export_dependencies(rosidl_default_runtime builtin_interfaces)

ament_package()
