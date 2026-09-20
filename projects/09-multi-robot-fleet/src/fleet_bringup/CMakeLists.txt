cmake_minimum_required(VERSION 3.16)
project(fleet_bringup NONE)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch config urdf
  DESTINATION share/${PROJECT_NAME}
)

# The world and the map are assets, not launch files, but the launch file needs to
# find them in the install space: a launch file that only works when run from the
# source tree is the kind of thing that breaks the first time it is run from
# somewhere else. Installed under their own names so the paths stay readable.
#
# NOTE the path depth: this file lives at src/fleet_bringup/, so the repository's
# assets/ directory is two levels up, not one. A single "../assets" resolves to
# src/assets, which does not exist, and --symlink-install reports it as a missing
# file rather than as a wrong relative path.
install(
  FILES ../../assets/worlds/warehouse.sdf
  DESTINATION share/${PROJECT_NAME}/worlds
)
install(
  FILES ../../assets/maps/warehouse.pgm ../../assets/maps/warehouse.yaml
  DESTINATION share/${PROJECT_NAME}/maps
)

# The per-robot model template is installed too, so a rendered model can be
# produced from an installed workspace and not only from the source tree.
install(
  FILES ../../assets/models/amr_4wd/model.sdf.in ../../assets/models/amr_4wd/model.config
  DESTINATION share/${PROJECT_NAME}/model_template
)

ament_package()
