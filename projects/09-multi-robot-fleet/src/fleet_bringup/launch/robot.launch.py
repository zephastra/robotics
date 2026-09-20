#!/usr/bin/env python3
"""Bring up ONE robot: spawn it, describe it, bridge it, gate it, and optionally
localise and navigate it.

Everything here is parameterised on one name, `robot`, so P3 is a fleet.launch.py that
calls this file once per robot with no other change. Nothing hardcodes r01 and nothing
depends on how many robots there are.

Ordering, and why:

  1. spawn                  gz needs the model in the world before anything observes it
  2. robot_state_publisher  publishes base_footprint -> base_link -> laser_link
  3. bridge                 gz <-> ROS: scan, odom, tf, joint_states, truth, cmd_vel
  4. safety gate            the ONLY publisher of <ns>/cmd_vel
  5. amcl + nav2            publishes cmd_vel_nav, never cmd_vel

The velocity chain, which is the part that is easy to get subtly wrong:

    controller_server  --cmd_vel_nav-->|
    behavior_server    --cmd_vel_nav-->|---> velocity_smoother
        --cmd_vel_smoothed--> collision_monitor
        --cmd_vel_pre_gate--> SAFETY GATE
        --cmd_vel--> bridge --> gz DiffDrive

controller_server AND behavior_server are both remapped OFF cmd_vel, and the gate's
output is ON cmd_vel. Two reasons, both learned the hard way:

  * If the controller also published cmd_vel, velocity_smoother would subscribe to the
    gate's own output and the loop would feed itself.
  * behavior_server publishes cmd_vel as well (spin, backup, drive_on_heading). Left
    alone, recovery behaviours reach the wheels without passing the smoother, the
    collision monitor or the gate. That is not hypothetical: on the first Nav2 bringup
    the gate latched ESTOP with "SOLE PUBLISHER VIOLATION ... also published by
    ['behavior_server']".

The gate is therefore the single publisher of cmd_vel by construction rather than by
convention.

Params files. config/nav2_params.yaml contains __NS__ in every frame name, __MAP__ in the
map path and __INIT_X__ / __INIT_Y__ / __INIT_YAW__ in AMCL's initial pose, and its top
level keys are unqualified. This file substitutes all of them and prefixes the keys, so
each robot gets its own generated params file. Every substitution is verified afterwards:
a surviving token, an unqualified key or a bare frame name means a robot that silently
never activates or that localises itself six metres from where it is.
"""

from __future__ import annotations

import re
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

# Explicit sys.path insert: `ros2 launch` does not promise to add this file's
# directory, and the sibling helper is needed at import time.
sys.path.insert(0, str(Path(__file__).resolve().parent))

from _common import (  # noqa: E402
    as_float_text,
    default_map,
    default_models_dir,
    default_world,
    runtime_dir,
    share_dir,
    spawn_pose,
    world_name,
)

from ament_index_python.packages import get_package_share_directory  # noqa: E402
from launch import LaunchDescription  # noqa: E402
from launch.actions import (  # noqa: E402
    DeclareLaunchArgument,
    OpaqueFunction,
    TimerAction,
)
from launch.substitutions import LaunchConfiguration  # noqa: E402
from launch_ros.actions import Node  # noqa: E402

ROBOT_RE = re.compile(r"^[A-Za-z][A-Za-z0-9_]{0,15}$")

# Top level keys in a Nav2 params file, prefixed with the namespace so the file matches
# nodes launched as /<ns>/<node>.
TOP_LEVEL_KEY_RE = re.compile(r"^([a-z][a-z0-9_]*):(\s*)$", re.MULTILINE)

BARE_FRAME_RE = re.compile(
    r"^\s*(robot_base_frame|base_frame_id|odom_frame_id|local_frame|global_frame):"
    r'\s*"?(odom|base_link|base_footprint)"?\s*$',
    re.MULTILINE,
)

# nav2's own launch files substitute $(find-pkg-share <pkg>) inside a YAML parameter
# file. That is a NAV2 convention, not a ROS one: hand the same file straight to a node
# and the literal string survives. Stock nav2_params.yaml relies on it for
#
#     bt_search_directories:
#       - $(find-pkg-share nav2_bt_navigator)/behavior_trees
#
# and the failure it produces is misleading:
#
#     Exception registering behavior trees: filesystem error: directory iterator
#     cannot open directory: No such file or directory
#     [$(find-pkg-share nav2_bt_navigator)/behavior_trees]
#
# which reads like a missing install rather than an unsubstituted variable.
FIND_PKG_SHARE_RE = re.compile(r"\$\(find-pkg-share\s+([A-Za-z0-9_]+)\)")

# TF is ONE shared tree, and every consumer in a namespace must be bound to it.
#
# A node inside namespace /r01 resolves the relative topic name `tf` to /r01/tf. But the
# tree is published on the global /tf (gz odometry via the bridge, remapped on purpose)
# and /tf_static (robot_state_publisher). Nothing publishes /r01/tf, so a namespaced
# consumer sees an EMPTY transform tree.
#
# That produces no error message anywhere. It produces this chain instead:
#
#     AMCL cannot transform a scan -> it drops every scan
#       -> it never publishes map -> odom
#       -> the frame `map` never exists
#       -> global_costmap cannot activate, and later global planning cannot run
#       -> the behaviour tree aborts the goal in 0.1 s with no explanation in the log
#
# The observed symptoms were exactly those: amcl_pose frozen at the initial pose across
# samples, planner_server failing with "transform from r01/base_link to map did not
# become available", and every NavigateToPose goal returning ABORTED almost immediately.
#
# Relative names in a remap rule are expanded against the node's own namespace, so these
# two entries bind every node to the shared tree without hardcoding r01.
TF_REMAPS: list[tuple[str, str]] = [("tf", "/tf"), ("tf_static", "/tf_static")]


def _resolve_find_pkg_share(text: str) -> str:
    """Resolve every $(find-pkg-share <pkg>) so the generated file is self-contained.

    Resolving here rather than leaving it to whatever launched us also means a package
    that cannot be found raises, instead of producing a path that does not exist and a
    node that quietly refuses to activate.
    """
    missing: list[str] = []

    def repl(match: re.Match[str]) -> str:
        pkg = match.group(1)
        try:
            return str(get_package_share_directory(pkg))
        except Exception:
            missing.append(pkg)
            return match.group(0)

    out = FIND_PKG_SHARE_RE.sub(repl, text)
    if missing:
        raise BringupError(
            "these packages are referenced by the nav2 params but are not on the ament "
            f"prefix path: {sorted(set(missing))}. Source the Nav2 overlay first."
        )
    leftover = re.findall(r"\$\([^)]*\)", out)
    if leftover:
        raise BringupError(
            f"unsubstituted launch-style variables survived in the nav2 params: "
            f"{sorted(set(leftover))}. They would be passed to the nodes literally."
        )
    return out


class BringupError(RuntimeError):
    pass


def _qualify(text: str, robot: str) -> str:
    """Prefix top level Nav2 params keys with the robot namespace.

    Nav2 matches parameter blocks against the fully qualified node name. A block called
    `amcl:` never matches a node called `/r01/amcl`, and the failure is quiet: the node
    starts, activates, and runs entirely on defaults.
    """
    out, n = TOP_LEVEL_KEY_RE.subn(lambda m: f"{robot}/{m.group(1)}:{m.group(2)}", text)
    if n == 0:
        raise BringupError("nav2 params have no top level keys to qualify: wrong file?")
    return out


def _render_nav2_params(share: Path, robot: str, map_yaml: str, runtime: Path,
                        spawn_x: str, spawn_y: str, spawn_yaw: str) -> str:
    src = share / "config" / "nav2_params.yaml"
    if not src.is_file():
        raise BringupError(f"nav2 params not found: {src}")

    text = src.read_text(encoding="utf-8")
    text = text.replace("__NS__", robot).replace("__MAP__", map_yaml)
    # AMCL's initial pose must be the SPAWN pose. Leaving it at 0 0 0 starts the belief
    # at the map origin, six metres from where the robot actually is, and the result
    # looks like a broken localiser rather than a missing parameter.
    text = (text.replace("__INIT_X__", spawn_x)
                .replace("__INIT_Y__", spawn_y)
                .replace("__INIT_YAW__", spawn_yaw))
    for token in ("__NS__", "__MAP__", "__INIT_X__", "__INIT_Y__", "__INIT_YAW__"):
        if token in text:
            raise BringupError(f"{token} survived substitution for {robot}")
    text = _resolve_find_pkg_share(text)
    text = _qualify(text, robot)

    # Prove no bare frame name survived. Two robots share one TF tree, so a bare `odom`
    # here would silently put both robots in the same frame.
    bare = [
        f"line {text[:m.start()].count(chr(10)) + 1}: {m.group(0).strip()}"
        for m in BARE_FRAME_RE.finditer(text)
    ]
    if bare:
        raise BringupError(
            f"unprefixed frame names in the params for {robot}: " + "; ".join(bare)
        )

    # Prove AMCL's initial pose is still typed as a float. A whole Nav2 stack failed to
    # activate because a launch format string turned -6.0 into -6, YAML read it as an
    # integer, and AMCL refused to configure -- and the error that surfaced much later was
    # "no navigate_to_pose action server". Catching it here names the real cause.
    if "set_initial_pose: true" not in text:
        raise BringupError(
            f"the params for {robot} do not set an initial pose, so AMCL would start "
            "uninitialised at the map origin. Set it from the spawn pose."
        )
    # Bound the search to the four lines that follow `initial_pose:` rather than to the
    # next blank line, so an unrelated `z:` far below in the file cannot trip this.
    after = text.split("initial_pose:", 1)[1].splitlines()[:8] \
        if "initial_pose:" in text else []
    ints = [
        ln.strip() for ln in after
        if re.match(r"^\s*(x|y|z|yaw):\s*-?\d+\s*$", ln)
    ]
    if ints:
        raise BringupError(
            "AMCL's initial pose contains integer-typed values "
            f"({'; '.join(ints)}). YAML reads `-6` as an int and AMCL declares these as "
            "doubles, so it fails to configure and the visible symptom is a missing "
            "action server. Format them with as_float_text()."
        )

    out_dir = runtime / "params"
    out_dir.mkdir(parents=True, exist_ok=True)
    out = out_dir / f"nav2_params_{robot}.yaml"
    out.write_text(text, encoding="utf-8")
    return str(out)


def _bridge_arguments(robot: str, bridge_joint_states: bool = False) -> list[str]:
    """gz <-> ROS topics for one robot. Absolute names only.

    joint_states is OPT-IN and off by default. It is NOT a fix for the stale-pose
    problem -- that was tested and this made no difference. Here is what is actually
    known, because both halves of it matter:

    WHAT IT IS. gz's JointStatePublisher has no rate parameter: it publishes in
    PostUpdate, so its rate IS the physics step rate. This world steps at 0.001 s, so
    joint_states arrives at 1000 Hz -- one hundred times the lidar, fifty times odometry.
    Nothing in the P2 control path reads wheel joint angles; they exist for RViz, and P2
    is headless.

    WHAT IT IS NOT, AND WHY THIS PARAGRAPH EXISTS. An earlier run of probe_pipeline_lag.py
    appeared to show every topic on the localisation path collapsing to ~3.8 Hz while the
    robot moved, with ages growing to 4.8 s and then overshooting their configured rate
    once it stopped. That looked like a transport backlog, so this flag was added to
    remove the 1000 Hz stream. The collapse did not change, and the reason is that it was
    never real: the probe paced its own loop and called spin_once once per 50 ms tick, so
    it could drain only 20 callbacks per second while scan + odom + tf deliver about 50.
    It was throttling itself, and only in the phases that used the sleeping loop.
    Re-measured with the executor on its own thread, and with a self-check comparing the
    delivered rate against the rate the message stamps imply, every input is fresh:

        phase      sim/wall   scan rate   scan age   odom rate   odom age
        static        0.999     10.0 Hz    0.059 s     19.9 Hz    0.000 s
        driving       0.990     10.0 Hz    0.063 s     20.0 Hz    0.000 s
        after         0.947      9.9 Hz    0.100 s     19.9 Hz    0.000 s

    gz's own /world/warehouse/stats agrees: real_time_factor 1.000 and 1000 physics steps
    per sim second, world-only, robot idle and robot driving alike. There was no overload
    anywhere. Do not re-test any of this as a lag fix.

    WHY IT IS STILL OFF: it is unused. gz's JointStatePublisher has no rate parameter --
    it publishes in PostUpdate, so its rate IS the physics step rate, which here is
    0.001 s, i.e. 1000 Hz of wheel angles that nothing in the P2 control path reads. They
    exist for RViz and P2 is headless. Re-enable with bridge_joint_states:=true for a run
    that genuinely needs them, e.g. probe_odom_truth.py, which uses wheel angles to prove
    the wheels turn rather than slip.
    """
    args = [
        # gz -> ROS
        f"/{robot}/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
        f"/{robot}/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry",
        # NOT here any more: f"/{robot}/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V".
        # That bridge gave the odom -> base_footprint edge one publisher PER ROBOT, all
        # writing into the single shared /tf. Three independent producers stamping from
        # the same gz clock arrive at consumers out of stamp order under load; tf2 then
        # does not drop the late message, it clears its whole buffer ("Detected jump back
        # in time. Clearing TF buffer."), the odom frame ceases to exist, and the
        # tf2_ros::MessageFilter inside AMCL throws an uncaught tf2::LookupException and
        # aborts. Measured with 3 robots driven for 30 s: 0 / 6 / 4 nodes crashed across
        # three attempts -- amcl and controller_server and planner_server all took turns.
        # The edge is published instead by fleet_ros' odom_to_tf node, from the odometry
        # bridge immediately above, so it has one writer and one clock.
        # ROS -> gz
        f"/{robot}/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist",
        # NOT here: the ground-truth topic. It is per WORLD, not per robot, and is bridged
        # by world.launch.py exactly once. Bridging it from each robot's launch file would
        # put N publishers on one topic and deliver every model's pose N times, so the
        # evaluator would see each robot in N places -- a wrong answer that looks like a
        # data problem rather than a wiring problem.
    ]
    if bridge_joint_states:
        args.append(f"/{robot}/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model")
    return args


def _launch_setup(context, *args, **kwargs):
    def arg(name: str) -> str:
        return LaunchConfiguration(name).perform(context)

    robot = arg("robot")
    if not ROBOT_RE.match(robot):
        raise BringupError(
            f"invalid robot name {robot!r}: it becomes a topic name and a TF frame "
            f"prefix, so it must match {ROBOT_RE.pattern}"
        )

    use_sim_time = arg("use_sim_time").lower() in ("1", "true", "yes")
    start_nav2 = arg("start_nav2").lower() in ("1", "true", "yes")
    autostart = arg("autostart").lower() in ("1", "true", "yes")
    # Off by default: see _bridge_arguments for the measured reason (1000 Hz of wheel
    # angles starves the bridge and makes the lidar arrive seconds late).
    bridge_joint_states = arg("bridge_joint_states").lower() in ("1", "true", "yes")
    spawn_delay = float(arg("spawn_delay_s"))
    # Seconds the Nav2 lifecycle manager waits for a lifecycle service reply. Read here
    # rather than inlined so the value appears once, is assertable, and cannot silently
    # diverge from what a guard checks.
    nav2_service_timeout = float(arg("nav2_service_timeout"))
    if nav2_service_timeout <= 0.0:
        raise BringupError(
            f"nav2_service_timeout must be positive, got {nav2_service_timeout!r}. A "
            f"zero or negative value would make the lifecycle manager give up instantly "
            f"on every request.")

    share = share_dir()
    if share is None:
        raise BringupError("fleet_bringup is not on the ament prefix path; source the "
                           "workspace (source scripts/env.sh) before launching")
    runtime = runtime_dir()

    world = arg("world")
    wname = world_name(world)

    # Spawn pose from config/spawns.yaml unless explicitly overridden. Both the spawn
    # call AND AMCL's initial pose read these four values, so they cannot disagree.
    # spawns.yaml is only consulted for the fields that were left empty, so a caller who
    # passes all four explicitly does not need the file to exist at all.
    given = {k: arg(k) for k in ("x", "y", "z", "yaw")}
    if not all(given.values()):
        pose = spawn_pose(robot)
    else:
        pose = {}
    # as_float_text, NOT %g: these values are substituted into a YAML params file where
    # `-6` is an integer and `-6.0` is a double, and AMCL refuses to configure on the
    # former. See the note on as_float_text.
    spawn_x = given["x"] or as_float_text(pose["x"])
    spawn_y = given["y"] or as_float_text(pose["y"])
    spawn_z = given["z"] or as_float_text(pose["z"])
    spawn_yaw = given["yaw"] or as_float_text(pose["yaw"])
    if not given["x"] or not given["y"]:
        print(f"009 launch: spawn pose for {robot} from config/spawns.yaml: "
              f"({spawn_x}, {spawn_y}) yaw {spawn_yaw}", flush=True)

    models_dir = Path(arg("models_dir"))
    model_sdf = models_dir / f"amr_4wd_{robot}" / "model.sdf"
    if not model_sdf.is_file():
        raise BringupError(
            f"rendered model not found: {model_sdf}\n"
            "Render it first, then relaunch:\n"
            f"    python3 scripts/render_model.py --robot {robot}\n"
            "Launching without it fails inside gz with a message about a missing file "
            "rather than about the missing render step."
        )

    urdf_path = share / "urdf" / "amr_4wd.urdf"
    if not urdf_path.is_file():
        raise BringupError(f"robot description not found: {urdf_path}")
    robot_description = urdf_path.read_text(encoding="utf-8")

    map_yaml = arg("map") or default_map()

    actions = []

    # 1. spawn. -allow_renaming false so a name clash fails loudly: gz would otherwise
    #    create `r01_0`, whose topics belong to nobody and whose silence looks like a
    #    bridge fault rather than a duplicate spawn.
    spawn = Node(
        package="ros_gz_sim",
        executable="create",
        name=f"spawn_{robot}",
        output="screen",
        arguments=[
            "-world", wname,
            "-name", robot,
            "-file", str(model_sdf),
            "-x", spawn_x, "-y", spawn_y, "-z", spawn_z, "-Y", spawn_yaw,
            "-allow_renaming", "false",
        ],
    )
    actions.append(TimerAction(period=spawn_delay, actions=[spawn]))

    # 2. TF for the robot's own links. frame_prefix adds "<robot>/" once; the URDF uses
    #    bare link names, so the prefix cannot be applied twice.
    actions.append(Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        namespace=robot,
        output="screen",
        # Bind to the shared /tf and /tf_static. Without this, the robot description
        # lands on /r01/tf_static while the gz odometry lands on /tf, and the tree is
        # split in two: each half is individually plausible and neither supports a
        # single lookup from map to the laser.
        remappings=TF_REMAPS,
        parameters=[{
            "robot_description": robot_description,
            "frame_prefix": f"{robot}/",
            "use_sim_time": use_sim_time,
        }],
    ))

    # 2b. Sensor frame alias.
    #
    # gz names a sensor's frame as <link>/<sensor>, so the lidar publishes
    #     frame_id: r01/laser_link/scan
    # while the URDF (and therefore TF) only has
    #     r01/laser_link
    # The two conventions do not meet, and the consequence is not a warning:
    # AMCL cannot transform a single scan, so it never publishes map -> odom, so
    # global_costmap cannot activate, so the lifecycle manager aborts the whole bringup.
    # The failure presents as
    #     "Failed to activate global_costmap because transform from r01/base_link to map
    #      did not become available before timeout"
    # which points at TF in general rather than at the scan frame.
    #
    # The sensor sits at the link origin (pose 0 0 0 in the model), so the alias is the
    # identity. Declaring an alias frame is the standard ROS answer to a driver that
    # names frames its own way; the alternative, rewriting the header of every scan,
    # would add a node and latency and would have to be repeated per sensor.
    #
    # Deliberately in the SAME namespace as robot_state_publisher, so whatever topic
    # convention static transforms use, the two agree with each other and with the
    # consumers (AMCL, both costmaps) which live in the same namespace.
    actions.append(Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="sensor_frame_alias",
        namespace=robot,
        output="screen",
        arguments=[
            "--frame-id", f"{robot}/laser_link",
            "--child-frame-id", f"{robot}/laser_link/scan",
        ],
        remappings=TF_REMAPS,
        parameters=[{"use_sim_time": use_sim_time}],
    ))

    # 3. bridge. Everything stays inside the robot's namespace now: the gz tf topic is
    #    no longer bridged at all (see _bridge_arguments), so there is no /tf remap here
    #    either. Leaving a remap for a topic that is not in the argument list would be a
    #    rule that looks enforced and is not.
    actions.append(Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="gz_bridge",
        namespace=robot,
        output="screen",
        arguments=_bridge_arguments(robot, bridge_joint_states=bridge_joint_states),
        parameters=[{"use_sim_time": use_sim_time}],
    ))

    # 3b. odom -> base_footprint, from the odometry that step 3 just bridged.
    #
    # This is the replacement for bridging gz's Pose_V. The edge needs exactly one
    # publisher, and in a fleet there is exactly one thing that knows it: this robot's own
    # odometry stream. Frames come from the message header (gz's DiffDrive is configured
    # with frame_id __ROBOT__/odom and child_frame_id __ROBOT__/base_footprint) so this
    # file cannot drift away from the model SDF; the parameters are an override, not the
    # source of truth.
    #
    # It is deliberately NOT namespaced-through-remap: /tf is one global tree, so this
    # node publishes to the absolute /tf. Relative names would resolve to /r01/tf and the
    # edge would exist for nobody.
    actions.append(Node(
        package="fleet_ros",
        executable="odom_to_tf",
        name="odom_to_tf",
        namespace=robot,
        output="screen",
        remappings=TF_REMAPS,
        parameters=[{
            "odom_topic": f"/{robot}/odom",
            "tf_topic": "/tf",
            "use_sim_time": use_sim_time,
        }],
    ))

    # 4. safety gate: the only publisher of <ns>/cmd_vel.
    actions.append(Node(
        package="fleet_ros",
        executable="gate_node",
        name="safety_gate",
        namespace=robot,
        output="screen",
        parameters=[{
            "robot_id": robot,
            "use_sim_time": use_sim_time,
            "publish_rate_hz": 20.0,
            "cmd_vel_in_topic": "cmd_vel_pre_gate",
            "cmd_vel_out_topic": "cmd_vel",
            # This Nav2 generation publishes TwistStamped on its velocity topics.
            # Getting this wrong makes the gate silently deaf to Nav2 while every topic
            # name in the graph still looks correct -- DDS reports it as a QoS
            # mismatch, not as a type mismatch. See gate_node.py.
            "cmd_vel_in_is_stamped": True,
            "odom_topic": "odom",
            "max_linear_mps": 0.35,
            "max_angular_radps": 0.6,
            # false only when a robot is driven without the reservation service, e.g.
            # the P2 single robot smoke test. Must be true for any run that claims to
            # exercise the corridor rules.
            "requires_permit": arg("gate_requires_permit").lower() in
                                ("1", "true", "yes"),
            # P4.3: when true the gate decides whether this robot may enter a
            # protected crossing from its OWN pose, and ignores the legacy
            # wants_protected_zone flag. It must stay true for any run that
            # claims to exercise the corridor rules -- the whole point is that
            # the supervised party no longer declares its own permission.
            "traffic_guard": arg("traffic_guard").lower() in ("1", "true", "yes"),
            "traffic_config": arg("traffic_config"),
            # Consumed as a RELATIVE name, so each robot listens on its own
            # namespace: /r01/fleet/permit, /r02/fleet/permit.
            "permit_topic": arg("permit_topic"),
            "robot_length_m": float(arg("robot_footprint_length_m")),
            "robot_width_m": float(arg("robot_footprint_width_m")),
            # The origin of this robot's odom frame. The geofence compares against
            # map-frame rectangles, so it has to translate first; without this a robot
            # at its spawn pose reads as being inside the corridor.
            "spawn_x": float(spawn_x),
            "spawn_y": float(spawn_y),
            "spawn_yaw": float(spawn_yaw),
            # Which pose the gate concludes from. The localiser-based source needs a localiser, so
            # it is selected exactly when one is being started -- and the gate REFUSES rather than
            # falling back if it goes quiet, so a missing localiser is a stop, not a silent
            # downgrade. See docs/DECISIONS.md D-P11-01 for the measurement that put it here: the
            # composed pose's error is a RATE (0.0809 x distance travelled, p95 1.725 m), while
            # AMCL's is 0.348 m p95 with a 0.853 s p95 age.
            "pose_source": "localiser" if start_nav2 else "spawn_odom",
        }],
    ))

    if start_nav2:
        params = _render_nav2_params(share, robot, map_yaml, runtime,
                                     spawn_x, spawn_y, spawn_yaw)
        # Nav2's default log level is info, and at info the interesting failures are
        # invisible: AMCL does not announce that it rejected a scan, and the behaviour
        # tree does not announce why it aborted. Raising the level for one diagnostic run
        # is how those reasons become visible, so it is a launch argument rather than
        # something to hand-edit each time.
        nav2_log_args = ["--log-level", arg("nav2_log_level")]

        common = {
            "parameters": [params, {"use_sim_time": use_sim_time}],
            "namespace": robot,
            "output": "screen",
            "arguments": nav2_log_args,
            # Every nav2 node reads the shared transform tree. See TF_REMAPS above for
            # what happens without this: AMCL silently drops every scan.
            "remappings": list(TF_REMAPS),
        }

        actions += [
            Node(package="nav2_map_server", executable="map_server",
                 name="map_server", **common),
            Node(package="nav2_amcl", executable="amcl", name="amcl", **common),
            # The controller publishes cmd_vel_nav, NOT cmd_vel. See the chain note in
            # the module docstring: this remap is what stops the smoother from
            # consuming the gate's own output.
            Node(package="nav2_controller", executable="controller_server",
                 name="controller_server",
                 remappings=TF_REMAPS + [("cmd_vel", "cmd_vel_nav")],
                 **{k: v for k, v in common.items() if k != "remappings"}),
            Node(package="nav2_planner", executable="planner_server",
                 name="planner_server", **common),
            # behavior_server publishes cmd_vel too, for spin / backup /
            # drive_on_heading. Without this remap the recovery behaviours bypass the
            # smoother, the collision monitor AND the safety gate, and reach the wheels
            # directly. The gate caught exactly that on the first Nav2 run:
            #     SOLE PUBLISHER VIOLATION on /r01/cmd_vel: also published by
            #     ['behavior_server']. Latched ESTOP.
            # Routing them to cmd_vel_nav puts recovery motion through the same chain as
            # planned motion, so there is one entry point for everything Nav2 can do
            # rather than two with different protections.
            Node(package="nav2_behaviors", executable="behavior_server",
                 name="behavior_server",
                 remappings=TF_REMAPS + [("cmd_vel", "cmd_vel_nav")],
                 **{k: v for k, v in common.items() if k != "remappings"}),
            Node(package="nav2_bt_navigator", executable="bt_navigator",
                 name="bt_navigator", **common),
            Node(package="nav2_velocity_smoother", executable="velocity_smoother",
                 name="velocity_smoother",
                 remappings=TF_REMAPS + [("cmd_vel", "cmd_vel_nav")],
                 **{k: v for k, v in common.items() if k != "remappings"}),
            Node(package="nav2_collision_monitor", executable="collision_monitor",
                 name="collision_monitor", **common),
        ]

        lifecycle_nodes = [
            "map_server", "amcl", "controller_server", "planner_server",
            "behavior_server", "bt_navigator", "velocity_smoother", "collision_monitor",
        ]
        actions.append(Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="lifecycle_manager",
            namespace=robot,
            output="screen",
            parameters=[{
                "use_sim_time": use_sim_time,
                "autostart": autostart,
                "node_names": lifecycle_nodes,
                # Too short and nodes are torn down under load so the stack flaps; too
                # long and a genuinely dead node goes unnoticed. 4 s matches stock.
                "bond_timeout": 4.0,
                # How long the manager waits for a lifecycle service reply. Stock is 5.0 s
                # and was NOT enough on this machine: bringing three Nav2 stacks up at once
                # (three robots x eight managed nodes, plus gz emulating three robots) makes
                # map_server's reply to get_state late, and the manager then aborts the whole
                # bringup. Measured bringups are in docs/DECISIONS.md (D-P17-24); the value
                # here is the measured worst case with margin, not a guess.
                "service_timeout": nav2_service_timeout,
                "attempt_respawn_reconnection": True,
            }],
        ))

    return actions


DEFAULTS = {
    "robot": ("r01", "Robot name. Becomes the namespace, the TF frame prefix and the "
                     "topic prefix."),
    # Empty means "read config/spawns.yaml". That file is the single source of truth:
    # validate_assets.py proves each pose is clear of every pad, wall, charger and the
    # protected corridor, and the spawn and AMCL's initial pose both come from here. Two
    # places holding the same number is how a robot ends up localising perfectly in the
    # wrong place.
    "x": ("", "Spawn x in the world frame. Empty = config/spawns.yaml."),
    "y": ("", "Spawn y in the world frame. Empty = config/spawns.yaml."),
    "z": ("", "Spawn z; a small lift so the robot settles instead of starting "
              "interpenetrated with the floor. Empty = config/spawns.yaml."),
    "yaw": ("", "Spawn yaw, radians. Empty = config/spawns.yaml."),
    "use_sim_time": ("true", "Use /clock. Must stay true while gz is the time source."),
    "start_nav2": ("true", "Start AMCL and the Nav2 stack for this robot."),
    "bridge_joint_states": (
        "false",
        "Bridge gz wheel joint states to ROS. OFF by default: gz publishes them at the "
        "physics step rate, which here is 1000 Hz, and that stream starves the bridge so "
        "the lidar arrives seconds late. Enable only for a run that needs wheel angles "
        "(probe_odom_truth.py). Nothing in the control path reads them."),
    "autostart": ("true", "Let the lifecycle manager configure and activate nodes."),
    "traffic_guard": (
        "false",
        "Install the P4.3 geometric geofence in this robot's gate. Default false "
        "because a single-robot launch has no coordinator to issue permits, and "
        "with the guard on and no permits the robot can never enter the corridor. "
        "fleet.launch.py sets it true for every robot; any corridor claim made "
        "with it false is invalid."),
    "traffic_config": (
        "",
        "Path to config/resources.yaml. Empty = derived from FLEET009_ROOT / the "
        "source tree. The node refuses to start if it cannot be loaded while "
        "traffic_guard is true."),
    "permit_topic": (
        "fleet/permit",
        "Relative topic this robot's gate reads permits from; the coordinator "
        "publishes <robot>/fleet/permit."),
    "robot_footprint_length_m": ("0.60", "Body length incl. wheel overhang."),
    "robot_footprint_width_m": ("0.45", "Body width incl. wheel overhang."),
    "gate_requires_permit": (
        "false",
        "Require a corridor permit before any motion. false is correct ONLY for a "
        "single-robot smoke test and invalidates any corridor claim."),
    "spawn_delay_s": ("3.0", "Seconds to wait for the gz create service to appear."),
    "nav2_service_timeout": (
        "20.0",
        "Seconds the Nav2 lifecycle manager waits for a lifecycle service reply "
        "(get_state / change_state). Stock nav2 uses 5.0 s, which is NOT enough on this "
        "machine: three robots x eight managed nodes brought up at once make map_server's "
        "reply late, and the manager then aborts the WHOLE bringup for that robot, so "
        "every case in the run records NOT_RUN. The failure message is "
        "'<node>/get_state service client: async_send_request failed', which reads like a "
        "failed send but is thrown when the REPLY is late. See D-P17-24 for the measured "
        "distribution this default is derived from."),
    "nav2_log_level": (
        "info",
        "Log level for the Nav2 nodes. 'debug' is needed to see why AMCL rejects a "
        "scan or why the behaviour tree aborts a goal; those reasons are not logged "
        "at info."),
    "map": ("", "Map yaml for AMCL and the static layer (default: the installed map)."),
    "world": ("", "World SDF; the world NAME is read from it for the spawn service."),
    "models_dir": ("", "Directory of rendered per-robot models."),
}


def generate_launch_description() -> LaunchDescription:
    declared = []
    for name, (default, description) in DEFAULTS.items():
        if default == "" and name == "world":
            default = default_world()
        elif default == "" and name == "models_dir":
            default = default_models_dir()
        declared.append(
            DeclareLaunchArgument(name, default_value=default, description=description)
        )
    return LaunchDescription(declared + [OpaqueFunction(function=_launch_setup)])
