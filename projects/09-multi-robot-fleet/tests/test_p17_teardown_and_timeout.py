"""Two fleet-only defects, and the checks that must not be able to miss them again.

Both defects share a shape that has cost this project several runs: **they only exist when
more than one robot is involved, and each file involved can hold its own value for the one
quantity they disagree about.**

  1. The Nav2 lifecycle manager's `service_timeout` was left at nav2's 5.0 s default. That
     is fine for one robot and aborts a three-robot bringup, because three Nav2 stacks come
     up at once and a reply arrives late. Measured: 0 of 6 three-robot bringups reached
     "Managed nodes are active" before the parameter was raised; 3 of 3 after.

  2. `Fleet.stop` swept for leftover processes with patterns that matched none of
     `parameter_bridge`, `robot_state_publisher` or `static_transform_publisher`, and
     skipped its process-group kill whenever the launcher had already exited. Three leaked
     generations of those left the host at load average 20, and with more than one live
     `clock_bridge` every consumer's clock jumps -- which is not a leak but a correctness
     fault, since a jumping clock makes tf2 discard its buffer and AMCL abort.

A guard that only ever agrees with the tree cannot catch either, so these tests feed the
checks a tree that is WRONG and require them to object.
"""
from __future__ import annotations

import ast
import importlib.util
import os
import pathlib
import subprocess
import sys
import textwrap
import time

import pytest

ROOT = pathlib.Path(__file__).resolve().parent.parent
TIMEOUT_GUARD = ROOT / "scripts/check_nav2_lifecycle_timeout.py"
BATCH = ROOT / "scripts/batch.py"


def _load_batch():
    spec = importlib.util.spec_from_file_location("_batch_under_test", BATCH)
    mod = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(mod)
    return mod


# --------------------------------------------------------------------------- #
# 1. the lifecycle timeout guard
# --------------------------------------------------------------------------- #
def _run_guard(root: pathlib.Path) -> subprocess.CompletedProcess:
    env = dict(os.environ, FLEET009_ROOT=str(root))
    return subprocess.run([sys.executable, str(TIMEOUT_GUARD), "--no-self-test"],
                          cwd=root, capture_output=True, text=True, env=env)


def _tree(tmp_path: pathlib.Path, *, timeout_value: str = "20.0",
          forward: bool = True, manager_line: bool = True) -> pathlib.Path:
    """A miniature FLEET009_ROOT holding only the two launch files."""
    launch = tmp_path / "src/fleet_bringup/launch"
    launch.mkdir(parents=True, exist_ok=True)

    manager = (
        '                "service_timeout": nav2_service_timeout,\n'
        if manager_line else ""
    )
    (launch / "robot.launch.py").write_text(
        "def _launch_setup():\n"
        "    actions = []\n"
        "    actions.append(Node(\n"
        "        package='nav2_lifecycle_manager',\n"
        "        parameters=[{\n"
        '            "bond_timeout": 4.0,\n'
        f"{manager}"
        "        }],\n"
        "    ))\n"
        "\n\n"
        "DEFAULTS = {\n"
        f'    "nav2_service_timeout": ("{timeout_value}", "doc"),\n'
        "}\n",
        encoding="utf-8")

    forwarded = (
        '                "nav2_service_timeout": _arg(context, "nav2_service_timeout"),\n'
        if forward else ""
    )
    (launch / "fleet.launch.py").write_text(
        "def _launch_setup():\n"
        "    actions = []\n"
        "    actions.append(IncludeLaunchDescription(\n"
        "        source,\n"
        "        launch_arguments={\n"
        '            "robot": robot,\n'
        f"{forwarded}"
        "        }.items(),\n"
        "    ))\n"
        "\n\n"
        "DEFAULTS = {\n"
        f'    "nav2_service_timeout": ("{timeout_value}", "doc"),\n'
        "}\n",
        encoding="utf-8")
    return tmp_path


def test_real_tree_passes_the_timeout_guard():
    proc = subprocess.run([sys.executable, str(TIMEOUT_GUARD), "--no-self-test"],
                          cwd=ROOT, capture_output=True, text=True)
    assert proc.returncode == 0, proc.stdout + proc.stderr


def test_guard_rejects_a_missing_service_timeout(tmp_path):
    """Stock behaviour: the parameter is simply absent. Must be refused."""
    root = _tree(tmp_path, manager_line=False)
    proc = _run_guard(root)
    assert proc.returncode == 1
    assert "NOT given `service_timeout`" in proc.stdout


def test_guard_rejects_an_unforwarded_timeout(tmp_path):
    """The fleet launch keeps its own copy and never passes it. Must be refused."""
    root = _tree(tmp_path, forward=False)
    proc = _run_guard(root)
    assert proc.returncode == 1
    assert "not forwarded" in proc.stdout


def test_guard_rejects_a_value_below_the_measured_floor(tmp_path):
    """Stock 5.0 s is exactly the value that aborts a three-robot bringup."""
    root = _tree(tmp_path, timeout_value="5.0")
    proc = _run_guard(root)
    assert proc.returncode == 1
    assert "below the measured floor" in proc.stdout


def test_guard_accepts_a_bounded_value(tmp_path):
    root = _tree(tmp_path, timeout_value="20.0")
    proc = _run_guard(root)
    assert proc.returncode == 0, proc.stdout + proc.stderr


# --------------------------------------------------------------------------- #
# 2. the teardown sweep
# --------------------------------------------------------------------------- #
#: Real command lines, copied from the process table during a three-robot run on
#: 2026-09-19. They are the fixture because the defect was exactly this: the sweep looked
#: for "gz sim", "nav2_" and "amcl", and NONE of the six lines below contains any of them.
#: Note the `-r __node:=...` suffix is a ROS argument, not argv[0]: the process that
#: publishes /clock is called `parameter_bridge`, and its command line says so.
REAL_CMDLINES = (
    "/opt/ros/lyrical/lib/ros_gz_bridge/parameter_bridge "
    "/r01/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan "
    "/r01/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry "
    "/r01/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist --ros-args -r __node:=gz_bridge -r __ns:=/r01",
    "/opt/ros/lyrical/lib/ros_gz_bridge/parameter_bridge "
    "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock --ros-args -r __node:=clock_bridge",
    "/opt/ros/lyrical/lib/ros_gz_bridge/parameter_bridge "
    "/world/warehouse/dynamic_pose/info@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V "
    "--ros-args -r __node:=truth_bridge",
    "/opt/ros/lyrical/lib/robot_state_publisher/robot_state_publisher "
    "--ros-args -r __node:=robot_state_publisher -r __ns:=/r01 -r tf:=/tf",
    "/opt/ros/lyrical/lib/tf2_ros/static_transform_publisher "
    "--frame-id r01/laser_link --child-frame-id r01/laser_link/scan "
    "--ros-args -r __node:=sensor_frame_alias -r __ns:=/r01",
    "/opt/nav2/nav2_amcl/lib/nav2_amcl/amcl --log-level info "
    "--ros-args -r __node:=amcl -r __ns:=/r01",
    "/opt/nav2/nav2_controller/lib/nav2_controller/controller_server "
    "--ros-args -r __node:=controller_server -r __ns:=/r01",
    "/home/ziling/projects/009_multi_robot_fleet/install/fleet_ros/lib/fleet_ros/gate_node "
    "--ros-args -r __node:=safety_gate -r __ns:=/r01",
    "gz sim --headless-rendering -r -v 2 warehouse.sdf",
)


@pytest.mark.parametrize("cmdline", REAL_CMDLINES)
def test_sweep_covers_every_real_process_class(cmdline):
    """Every class the tree spawns must match at least one sweep pattern.

    Parametrized over the actual command lines rather than over a list of words, because
    the failure mode being guarded against is precisely a word that is not in the command
    line: `clock_bridge` is a `parameter_bridge`, and `nav2_controller` is not `nav2_`.
    """
    fleet = _load_batch().Fleet
    matched = [p for p in fleet.LEFTOVER_PATTERNS if p in cmdline]
    assert matched, (
        f"no teardown pattern matches this real process: {cmdline!r}. It would survive the "
        f"sweep and pollute the next case.")


def test_leftovers_does_not_count_the_process_doing_the_counting():
    """The old `ps | grep -c <pattern>` read 1 on an IDLE machine.

    grep does not exclude itself, and its own command line contains the pattern. The
    replacement reads /proc/<pid>/cmdline and skips its own pid, so this asserts the skip
    directly, in a child whose OWN argv carries the marker. Asserting "an idle machine reads
    0" instead would have made this test depend on nothing else running -- and it promptly
    failed the first time the suite ran beside a live fleet.
    """
    script = textwrap.dedent(
        """
        import importlib.util, pathlib, sys
        spec = importlib.util.spec_from_file_location("b", sys.argv[2])
        mod = importlib.util.module_from_spec(spec); spec.loader.exec_module(mod)
        mod.Fleet.LEFTOVER_PATTERNS = (sys.argv[1],)
        print(mod.Fleet.leftovers()[sys.argv[1]])
        """
    )
    marker = "zz_marker_for_the_self_count_test_zz"
    proc = subprocess.run([sys.executable, "-c", script, marker, str(BATCH)],
                          capture_output=True, text=True, timeout=60)
    assert proc.returncode == 0, proc.stderr
    assert proc.stdout.strip() == "0", (
        f"the counter counted the process running it: {proc.stdout.strip()!r}")


def test_leftovers_detects_a_real_process():
    """A counter that can only read 0 is as useless as one that can only read 1."""
    fleet = _load_batch().Fleet
    marker = "zz_marker_for_the_real_process_test_zz"
    proc = subprocess.Popen(
        ["bash", "-c", f"exec -a {marker} sleep 20"],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    try:
        time.sleep(0.5)
        original = fleet.LEFTOVER_PATTERNS
        fleet.LEFTOVER_PATTERNS = (marker,)
        try:
            counts = fleet.leftovers()
        finally:
            fleet.LEFTOVER_PATTERNS = original
        assert counts[marker] >= 1, counts
    finally:
        proc.kill()
        proc.wait(timeout=10)


def test_group_kill_is_not_gated_on_the_launcher_being_alive():
    """The leak: a tree whose `ros2 launch` parent died still has live children.

    Structural, because the alternative is starting a real fleet inside a unit test. If
    `os.killpg` sits inside an `if <...>.poll() is None:` branch, then exactly the case
    that leaks -- parent gone, children alive -- is the case that skips the group kill.
    """
    src = BATCH.read_text(encoding="utf-8")
    tree = ast.parse(src)
    stop = None
    for node in ast.walk(tree):
        if isinstance(node, ast.ClassDef) and node.name == "Fleet":
            for sub in node.body:
                if isinstance(sub, ast.FunctionDef) and sub.name == "stop":
                    stop = sub
    assert stop is not None, "Fleet.stop is gone; this test cannot check anything"

    offenders = []
    for node in ast.walk(stop):
        if not isinstance(node, ast.If):
            continue
        test_src = ast.get_source_segment(src, node.test) or ""
        if ".poll()" not in test_src:
            continue
        for inner in ast.walk(node):
            if (isinstance(inner, ast.Call) and isinstance(inner.func, ast.Attribute)
                    and inner.func.attr == "killpg"):
                offenders.append(test_src.strip())
    assert not offenders, (
        f"Fleet.stop only signals the process group inside `if {offenders[0]}`. A launcher "
        f"that has already exited leaves its children running, and that is the case this "
        f"condition skips.")
