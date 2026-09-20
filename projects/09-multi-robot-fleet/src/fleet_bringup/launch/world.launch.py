#!/usr/bin/env python3
"""Start the Gazebo server for the 009 warehouse world, plus the clock bridge.

Deliberately server only. This machine has no NVIDIA Vulkan ICD, so a GUI is not
available and pretending otherwise would just fail later; P2 runs headless and the GUI
path is recorded as NOT_RUN rather than assumed.

What it starts:

  1. `gz sim -r -s` on assets/worlds/warehouse.sdf
  2. `/clock` bridged from gz to ROS
  3. the ground-truth pose channel, bridged from gz to ROS

Why the clock bridge AND the truth bridge live here rather than in robot.launch.py: both
are properties of the world, not of a robot. Two robots each bridging /clock would put two
publishers on one topic and make `use_sim_time` jitter, and two robots each bridging
/world/<name>/dynamic_pose/info would deliver every model's pose twice, so the evaluator
would see each robot in two places.

The truth channel is gz's own dynamic_pose/info from the SceneBroadcaster that every world
already runs. Three variants were tried and two failed silently; see the note at the end of
assets/models/amr_4wd/model.sdf.in. It carries every dynamic model, so consumers identify a
robot by matching its pose, and it drops names and stamps -- a coarse cross-check, recorded
as such, not the name-bearing channel the plan asks for.

AGENTS.md rule 13: this channel exists for INITIALISATION, FAULT INJECTION and
VERIFICATION. Nothing in the control path may subscribe to it.

Path resolution lives in _common.py so this file and robot.launch.py cannot disagree
about where the world is.
"""

from __future__ import annotations

import os
import sys
from pathlib import Path

# `ros2 launch` does not promise to put this file's directory on sys.path, so the
# sibling helper is imported explicitly rather than hoped for.
sys.path.insert(0, str(Path(__file__).resolve().parent))

from _common import default_models_dir, default_world, world_name  # noqa: E402

from launch import LaunchDescription  # noqa: E402
from launch.actions import (  # noqa: E402
    DeclareLaunchArgument,
    ExecuteProcess,
    OpaqueFunction,
    SetEnvironmentVariable,
)
from launch.substitutions import LaunchConfiguration  # noqa: E402
from launch_ros.actions import Node  # noqa: E402


def _launch_setup(context, *args, **kwargs):
    def arg(name: str) -> str:
        return LaunchConfiguration(name).perform(context)

    world = arg("world")
    models_dir = arg("models_dir")
    verbosity = arg("verbosity")
    headless = arg("headless").lower() in ("1", "true", "yes")
    start_clock = arg("clock_bridge").lower() in ("1", "true", "yes")
    truth_bridge = arg("truth_bridge").lower() in ("1", "true", "yes")

    if not world or not Path(world).is_file():
        raise RuntimeError(
            f"world.launch.py: world file not found: {world!r}. "
            "Pass world:=<path to an .sdf>."
        )

    actions = []

    # gz resolves models through GZ_SIM_RESOURCE_PATH. Rendered per-robot models are
    # generated at runtime, so they cannot come from an install directory. This is
    # APPENDED to whatever is already there: overwriting would hide the system models.
    if models_dir:
        existing = os.environ.get("GZ_SIM_RESOURCE_PATH", "")
        joined = models_dir if not existing else models_dir + os.pathsep + existing
        actions.append(SetEnvironmentVariable("GZ_SIM_RESOURCE_PATH", joined))

    gz_cmd = ["gz", "sim", "-r", "-s", "-v", verbosity]
    if headless:
        gz_cmd.append("--headless-rendering")
    gz_cmd.append(world)

    actions.append(ExecuteProcess(cmd=gz_cmd, output="screen", name="gz_sim_server"))

    if start_clock:
        actions.append(Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            name="clock_bridge",
            arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
            output="screen",
        ))

    if truth_bridge:
        # The topic name is derived from the world's own <world name=...> so it cannot
        # drift from the file it describes. Getting this wrong is silent: the ROS topic
        # exists with a publisher and never carries a message, which reads exactly like a
        # robot that never moves.
        wn = world_name(world)
        actions.append(Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            name="truth_bridge",
            arguments=[f"/world/{wn}/dynamic_pose/info"
                       "@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V"],
            output="screen",
        ))

    return actions


def generate_launch_description() -> LaunchDescription:
    declared = [
        DeclareLaunchArgument(
            "world", default_value=default_world(),
            description="World SDF to load (default: the installed 009 warehouse)."),
        DeclareLaunchArgument(
            "models_dir", default_value=default_models_dir(),
            description="Directory of rendered per robot models, appended to "
                        "GZ_SIM_RESOURCE_PATH."),
        DeclareLaunchArgument(
            "headless", default_value="true",
            description="Pass --headless-rendering. Must stay true on this machine: "
                        "there is no NVIDIA Vulkan ICD, so the GUI path is unavailable "
                        "and is recorded as NOT_RUN."),
        DeclareLaunchArgument(
            "verbosity", default_value="2", description="gz -v level (0..4)."),
        DeclareLaunchArgument(
            "clock_bridge", default_value="true",
            description="Bridge gz /clock to ROS. One per world, never one per robot."),
        DeclareLaunchArgument(
            "truth_bridge", default_value="true",
            description="Bridge gz /world/<name>/dynamic_pose/info to ROS for the "
                        "evaluator. One per world, never one per robot: a second bridge "
                        "would deliver every model's pose twice. Verification only."),
    ]
    return LaunchDescription(declared + [OpaqueFunction(function=_launch_setup)])
