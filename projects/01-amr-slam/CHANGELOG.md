# Changelog

## Unreleased

- Reframed the project as a progressive ROS 2 AMR mapping and navigation tutorial.
- Added a Gazebo Sim and RViz mapping screenshot to the project README.
- Added an automatic AMCL initial pose for the fixed simulation spawn point.
- Staged Nav2 startup so navigation begins only after `map -> odom` is available.
- Corrected RViz Map and LaserScan QoS settings.
- Added the Navigation 2 panel and Nav2 goal tool required to send navigation actions.

## v0.1.0 — 2026-09-02

- Added a complete Gazebo Sim differential-drive AMR world.
- Added ROS–Gazebo bridges for clock, scan, odometry, and velocity commands.
- Added the `map → odom → base_link → laser_link` TF chain.
- Added SLAM Toolbox mapping, RViz visualization, and map saving.
- Added Nav2 localization and navigation configuration.
- Corrected wheel geometry and turning behavior.
- Improved scan density and reduced default angular speed to prevent rotated map duplication.
- Added process cleanup, WSLg settings, Cyclone DDS defaults, and troubleshooting documentation.
- Added a non-destructive preflight check that prevents conflicting simulation sessions.
