# Third-party notices

009 runs on other people's software. This file records what, and what this project does and
does not claim about it.

## Runtime dependencies

| component | version on this machine | licence | how it is used |
|---|---|---|---|
| ROS 2 Lyrical | `lyrical` (the only distribution installed here) | Apache-2.0 | middleware, nodes, launch, `rosidl` interface generation |
| Gazebo Sim | `gz sim 10.4.0` (Rotary nightly) | Apache-2.0 | the world, the physics, the sensors |
| Nav2 | source overlay at `/opt/nav2` | Apache-2.0 | `navigate_to_pose`, costmaps, AMCL, controller |
| CycloneDDS | system | Eclipse Public License 2.0 / Eclipse Distribution License 1.0 | DDS implementation, pinned to loopback |
| Python | 3.14.4 | PSF-2.0 | everywhere |
| colcon / ament | system | Apache-2.0 | build |
| pytest | dev only | MIT | the core test suite (no count here: a number in a notice file goes stale, and this one already had) |

## What this project does not redistribute

No third-party source or binary is vendored in this repository. `assets/models/amr_4wd/` is
this project's own SDF template, and `assets/worlds/warehouse.sdf` is written for this
project — it is not an upstream Gazebo world. There is no upstream sample world or robot
description copied in, so there is nothing here that carries an upstream attribution
requirement.

If that changes — if an upstream model, mesh or world is brought in — it must be listed here
with its licence and the required attribution, and any CC BY material needs the attribution
line kept with it.

## Version sensitivity that matters

**This machine is not a typical ROS 2 setup, and the documentation does not match it.**

- Ubuntu **26.04**, ROS 2 **Lyrical** only. There is no Jazzy and no Humble here. Guides that
  assume either will produce commands whose packages do not exist.
- Gazebo is **`gz sim` 10.4.0**, which is the *Rotary* generation, **not Harmonic (8)**. This
  matters for API and config questions: consult 10.x documentation. A concrete consequence
  found here: `LIDAR` is not a supported sensor type and `gpu_lidar` must be used instead.
- There is **no NVIDIA Vulkan ICD**, so the default path is headless. The GUI path is not
  verified, and no test in this project depends on it.
- Nav2 and MoveIt 2 are **source overlays**, not binaries. A `ros2 pkg` lookup finds them
  only after `scripts/env.sh` has been sourced.

## Data and network

Nothing here talks to the network. The dashboard binds **loopback only** and refuses to start
on any other address; it has no authentication because it is not intended to be reachable.
The DDS configuration pins this fleet to the `lo` interface so that a WSL2 host's other
interfaces cannot produce partial discovery.
