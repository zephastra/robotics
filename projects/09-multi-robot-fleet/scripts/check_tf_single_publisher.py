#!/usr/bin/env python3
"""Guard: exactly one publisher per TF edge, and no edge is bridged from gz's pose stream.

THE DEFECT THIS PREVENTS
------------------------
gz's DiffDrive plugin emits `odom -> base_footprint` on a gz topic. The obvious way to get
that into the ROS tree is to bridge it:

    /<ns>/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V

That bridge is launched once PER ROBOT. All of them publish into the one shared `/tf`,
because TF is a single tree and that is the point of it. So the edge `odom ->
base_footprint` -- one edge per robot, but one edge each -- acquires three independent
publishers with three independent callback queues, each stamping from the same gz clock at
the moment its own callback happens to run.

tf2 requires arrival order to match stamp order per child frame. When it is violated it
does not drop the late message; it discards its whole buffer:

    [tf2_buffer]: Detected jump back in time. Clearing TF buffer.

The `odom` frame then does not exist. The next consumer to ask for it gets

    Invalid frame ID "<ns>/odom" passed to canTransform argument target_frame
        - frame does not exist

and the `tf2_ros::MessageFilter` inside AMCL throws

    tf2::LookupException: Static cache is empty, when looking up transform from
        frame [<ns>/laser_link/scan] to frame [<ns>/odom]

which nav2 does not catch, so the process terminates and aborts on signal 6.

MEASURED, 3 robots brought up together and driven for 30 s, this machine:

    before    attempt 1: 0 nodes crashed
              attempt 2: 6 nodes crashed -- amcl x3, controller_server x2, planner_server x1
              attempt 3: 4 nodes crashed -- amcl x3, controller_server x1
    after     attempt 1: 0    attempt 2: 0    attempt 3: 0

The victims are not only the localiser: any tf2 consumer holding a stale request when the
buffer is cleared can be the one that dies. Losing amcl is the worst case because
`amcl_pose` then never arrives and the safety gate is left judging from no localised pose.

WHAT THIS GUARD CHECKS
----------------------
1. `robot.launch.py`'s `_bridge_arguments` must NOT bridge a gz transform topic
   (`gz.msgs.Pose_V` onto a `tf2_msgs/msg/TFMessage`). A comment mentioning the old form
   is fine and expected -- this reads the ARGUMENT LIST, not the file text, because a
   name-based search over the whole file flags the very comment that explains the fix.
2. Some node must actually publish each edge the tree needs. `odom_to_tf` must be
   launched, must be given the odometry topic of its own robot, and must publish to the
   absolute `/tf` -- a relative `tf` inside namespace `/r01` resolves to `/r01/tf`, which
   nobody reads, and that failure is completely silent.
3. `tf2_msgs` must be a declared dependency of whatever package owns the publisher, so the
   guard cannot pass while the runtime import would fail.
4. Self-test: the checker must return non-zero on a deliberately broken tree. A guard that
   can only agree has been shown to be quiet, not to work (`D-P17-10`).

Exit 0 when the wiring is right, 1 otherwise.
"""

from __future__ import annotations

import ast
import os
import pathlib
import re
import sys

ROOT = pathlib.Path(os.environ.get("FLEET009_ROOT")
                    or pathlib.Path(__file__).resolve().parent.parent)
LAUNCH = ROOT / "src/fleet_bringup/launch/robot.launch.py"
WORLD_LAUNCH = ROOT / "src/fleet_bringup/launch/world.launch.py"
PKG_XML = ROOT / "src/fleet_ros/package.xml"
CMAKE = ROOT / "src/fleet_ros/CMakeLists.txt"
SCRIPT = ROOT / "src/fleet_ros/scripts/odom_to_tf"
NODE = ROOT / "src/fleet_ros/fleet_ros/odom_to_tf.py"

failures: list[str] = []
notes: list[str] = []


def _literal_strings(node: ast.AST) -> list[str]:
    """Every string constant inside a node, including inside f-strings' literals."""
    out: list[str] = []
    for sub in ast.walk(node):
        if isinstance(sub, ast.Constant) and isinstance(sub.value, str):
            out.append(sub.value)
        elif isinstance(sub, ast.JoinedStr):
            for part in sub.values:
                if isinstance(part, ast.Constant) and isinstance(part.value, str):
                    out.append(part.value)
    return out


def _literal_strings_in_values(values: list[ast.AST]) -> list[str]:
    """Same, for a list of keyword-value nodes rather than a single wrapped node.

    The launch file builds its strings with f-strings, so `f"/{robot}/odom"` is a
    JoinedStr whose literal parts are  "/" and "/odom". Matching on those fragments is
    how this guard sees the robot name without evaluating the expression.
    """
    out: list[str] = []
    for v in values:
        out.extend(_literal_strings(v))
    return out


def check_bridge_does_not_carry_tf() -> None:
    """The gz pose stream must not be bridged as TF."""
    if not LAUNCH.exists():
        failures.append(f"{LAUNCH.relative_to(ROOT)}: missing")
        return
    src = LAUNCH.read_text(encoding="utf-8")
    tree = ast.parse(src)

    fn = next((n for n in ast.walk(tree)
               if isinstance(n, ast.FunctionDef) and n.name == "_bridge_arguments"), None)
    if fn is None:
        failures.append("robot.launch.py: _bridge_arguments is gone; this guard cannot "
                        "verify what the bridge carries")
        return

    strings = _literal_strings(fn)
    offenders = [s for s in strings
                 if "tf2_msgs/msg/TFMessage" in s and "Pose_V" in s]
    if offenders:
        failures.append(
            "robot.launch.py: _bridge_arguments still bridges gz's pose stream as TF: "
            f"{offenders}. That gives each `odom -> base_footprint` edge one publisher "
            "PER ROBOT on the shared /tf; the arrival order breaks, tf2 clears its buffer, "
            "the odom frame disappears, and AMCL aborts with tf2::LookupException. "
            "Publish the edge from the bridged odometry instead (see fleet_ros "
            "odom_to_tf).")
    else:
        notes.append("_bridge_arguments does not bridge any gz transform stream")

    # The odom bridge is what odom_to_tf consumes. If it disappears, the replacement
    # source of the edge disappears with it and the tree goes quiet for a different
    # reason -- so assert it is still there rather than only asserting the removal.
    if any("nav_msgs/msg/Odometry" in s for s in strings):
        notes.append("_bridge_arguments still bridges nav_msgs/Odometry (odom_to_tf's input)")
    else:
        failures.append(
            "robot.launch.py: _bridge_arguments no longer bridges nav_msgs/msg/Odometry. "
            "That topic is now the ONLY source of the odom -> base_footprint edge, so "
            "removing it leaves the transform tree without an odometry edge at all.")


def check_odom_to_tf_launched() -> None:
    """odom_to_tf must be launched, per robot, onto the absolute /tf."""
    if not LAUNCH.exists():
        return
    src = LAUNCH.read_text(encoding="utf-8")
    tree = ast.parse(src)

    node_calls = [n for n in ast.walk(tree)
                  if isinstance(n, ast.Call)
                  and isinstance(n.func, ast.Name) and n.func.id == "Node"]
    found = None
    for call in node_calls:
        kwargs = {k.arg: k.value for k in call.keywords if k.arg}
        pkg = kwargs.get("package")
        exe = kwargs.get("executable")
        if (isinstance(pkg, ast.Constant) and pkg.value == "fleet_ros"
                and isinstance(exe, ast.Constant) and exe.value == "odom_to_tf"):
            found = kwargs
            break

    if found is None:
        failures.append(
            "robot.launch.py: no Node(package='fleet_ros', executable='odom_to_tf'). "
            "Nothing else publishes odom -> base_footprint now.")
        return

    # All string literals inside the odom_to_tf Node(...) call, at any depth.
    all_strings = _literal_strings_in_values(list(found.values()))

    # tf_topic must be the absolute /tf. A relative name inside namespace /r01 resolves
    # to /r01/tf, which no consumer reads -- silent, and identical in symptom to the
    # defect this guard exists for.
    tf_arg = [s for s in all_strings if s.strip("/") == "tf"]
    if not tf_arg:
        failures.append("robot.launch.py: odom_to_tf is launched without a tf_topic; "
                        "without it the edge goes to a name no consumer reads")
    elif "/tf" not in tf_arg:
        failures.append(f"robot.launch.py: odom_to_tf's tf_topic is not absolute "
                        f"(found {tf_arg}); inside namespace /r01 a relative name "
                        f"resolves to /r01/tf and the edge is invisible fleet-wide")
    else:
        notes.append("odom_to_tf publishes to the absolute /tf")

    # The odom topic must be absolute and must name this robot, so one odom_to_tf cannot
    # silently consume another robot's odometry and publish a wrong edge.
    odom_abs = [s for s in all_strings if s.startswith("/") and "odom" in s]
    if not odom_abs:
        failures.append("robot.launch.py: odom_to_tf has no absolute odom_topic; "
                        "it would subscribe to a topic in its own namespace that "
                        "nothing publishes")
    else:
        # Report the source expression, not the f-string fragments: "/{robot}/odom"
        # decomposes into "/" + "{robot}" + "/odom", and reporting "/odom" twice tells a
        # reader nothing about whether the robot name is in there.
        seg = ast.get_source_segment(src, found.get("parameters")) or ""
        joined = " ".join(seg.split())
        notes.append(f"odom_to_tf odom_topic expression: {joined[:120]}")

    # And it must be given the sim clock, or its stamps come from the wall clock and every
    # consumer rejects them as future data.
    if not any("use_sim_time" in s for s in all_strings):
        failures.append("robot.launch.py: odom_to_tf is not given use_sim_time; its "
                        "stamps would come from the wall clock and every consumer "
                        "would see them as future data")
    else:
        notes.append("odom_to_tf receives use_sim_time")


def check_world_launch_unchanged() -> None:
    """The truth channel is a separate topic and must not be confused with /tf."""
    if not WORLD_LAUNCH.exists():
        failures.append(f"{WORLD_LAUNCH.relative_to(ROOT)}: missing")
        return
    src = WORLD_LAUNCH.read_text(encoding="utf-8")
    if "dynamic_pose/info" in src:
        notes.append("world.launch.py still bridges the truth channel on its own topic "
                     "(not /tf)")
    else:
        notes.append("world.launch.py: no truth bridge found")


def check_dependency_and_entry_point() -> None:
    """The publisher must be installable and its dependency declared."""
    if not NODE.exists():
        failures.append(f"{NODE.relative_to(ROOT)}: missing -- the node itself is gone")
        return
    if not SCRIPT.exists():
        failures.append(f"{SCRIPT.relative_to(ROOT)}: missing -- without the wrapper the "
                        f"launch file's executable= cannot resolve")
        return

    cmake = CMAKE.read_text(encoding="utf-8") if CMAKE.exists() else ""
    if "scripts/odom_to_tf" not in cmake:
        failures.append(
            "src/fleet_ros/CMakeLists.txt: scripts/odom_to_tf is not in install(PROGRAMS "
            "...). This package installs executables via that list, not via setup.py's "
            "console_scripts, so the node would build and still not be launchable.")
    else:
        notes.append("scripts/odom_to_tf is installed via install(PROGRAMS ...)")

    xml = PKG_XML.read_text(encoding="utf-8") if PKG_XML.exists() else ""
    if "<depend>tf2_msgs</depend>" not in xml:
        failures.append("src/fleet_ros/package.xml: tf2_msgs is not declared, but "
                        "odom_to_tf imports TFMessage from it")
    else:
        notes.append("fleet_ros declares its tf2_msgs dependency")


def check_self_test() -> None:
    """Prove the checker can fail, by running it against a deliberately broken tree."""
    import shutil
    import subprocess
    import tempfile

    with tempfile.TemporaryDirectory() as tmp:
        tmp_path = pathlib.Path(tmp) / "009"
        for rel in ("src/fleet_bringup/launch", "src/fleet_ros/scripts",
                    "src/fleet_ros/fleet_ros"):
            (tmp_path / rel).mkdir(parents=True, exist_ok=True)
        shutil.copy(LAUNCH, tmp_path / "src/fleet_bringup/launch/robot.launch.py")
        shutil.copy(WORLD_LAUNCH, tmp_path / "src/fleet_bringup/launch/world.launch.py")
        shutil.copy(NODE, tmp_path / "src/fleet_ros/fleet_ros/odom_to_tf.py")
        shutil.copy(SCRIPT, tmp_path / "src/fleet_ros/scripts/odom_to_tf")
        shutil.copy(CMAKE, tmp_path / "src/fleet_ros/CMakeLists.txt")
        shutil.copy(PKG_XML, tmp_path / "src/fleet_ros/package.xml")

        # Re-introduce the defect: bridge the gz pose stream as TF again.
        broken = tmp_path / "src/fleet_bringup/launch/robot.launch.py"
        text = broken.read_text(encoding="utf-8")
        text = text.replace(
            '        f"/{robot}/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry",',
            '        f"/{robot}/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry",\n'
            '        f"/{robot}/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V",', 1)
        broken.write_text(text, encoding="utf-8")

        env = dict(os.environ)
        env["FLEET009_ROOT"] = str(tmp_path)
        # --no-self-test is mandatory here, not an optimisation: without it the child
        # re-enters check_self_test and forks again, and the guard never returns. The
        # self-test proves the OTHER checks can fail; it does not need to prove itself.
        proc = subprocess.run([sys.executable, __file__, "--no-self-test"],
                              env=env, capture_output=True, text=True, timeout=60)
        if proc.returncode == 0:
            failures.append(
                "self-test FAILED: with the gz pose bridge re-added by hand, this guard "
                "still exited 0. A guard that cannot fail is not a guard.")
        else:
            notes.append("self-test: detects a re-added gz pose bridge (exit "
                         f"{proc.returncode} as required)")


def main() -> int:
    check_bridge_does_not_carry_tf()
    check_odom_to_tf_launched()
    check_world_launch_unchanged()
    check_dependency_and_entry_point()
    if "--no-self-test" not in sys.argv:
        check_self_test()

    print("=" * 78)
    print("  009 single-publisher TF edge check (static)")
    print("=" * 78)
    for n in notes:
        print(f"    {n}")

    if failures:
        print()
        for f in failures:
            print(f"  [FAIL] {f}")
        print()
        print("Every transform edge must have exactly one publisher. When an edge has")
        print("several, their arrival order races, tf2 clears its buffer on the loser,")
        print("and every tf2 consumer in the fleet can die on the uncaught exception.")
        return 1

    print()
    print("    odom -> base_footprint has one publisher per robot, onto the shared /tf "
          "(exit 0)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
