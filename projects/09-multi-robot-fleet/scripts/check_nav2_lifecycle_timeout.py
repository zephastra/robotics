#!/usr/bin/env python3
"""Guard: a fleet run must not bring Nav2 up with a single-robot service timeout.

THE DEFECT THIS PREVENTS
------------------------
`nav2_lifecycle_manager` waits a bounded time for each lifecycle service reply. The
parameter is `service_timeout` (nav2 default 5.0 s) and it is used for both halves of
every state change:

    LifecycleManager::changeStateForNode(node, transition):
        node_map_[node]->change_state(transition, -1ms, service_timeout_)
        node_map_[node]->get_state(service_timeout_)

and `get_state` reaches

    nav2::ServiceClient::invoke(request, timeout)
      -> spin_until_complete(future, timeout)
      -> rclcpp::spin_until_future_complete(node_base_interface_, future, timeout)

whose timeout is WALL time. When it expires the library throws

    "<node>/get_state service client: async_send_request failed"

which reads like a failed send and is not one: the request was sent, and the REPLY was
late. The manager logs `Failed to change state for node: <node>` and then
`Failed to bring up all requested nodes. Aborting bringup.`

On this machine, bringing up three robots at once (three Nav2 stacks x eight managed
nodes, plus gz emulating three robots on CPU only) makes those replies late. Measured
before the fix, three three-robot runs: bringup never once reached "Managed nodes are
active" within 150 s, and `map_server` was the node that timed out.

WHY THAT MATTERS MORE THAN ONE ABORTED START
--------------------------------------------
An aborted bringup does not produce a failing test result. It produces a case with no
fresh state at all, which `batch.py` records as NOT_RUN. So a machine-speed problem
destroys the evidence for every case in the run instead of being reported as the
infrastructure fault it is -- the opposite of what CONTRACTS section 9 asks for.

WHAT THIS GUARD CHECKS
----------------------
1. `robot.launch.py`'s lifecycle_manager Node is given `service_timeout` explicitly.
   Leaving it to the library default is how this defect existed unnoticed: stock 5.0 s
   looks perfectly reasonable in isolation.
2. The value is the launch argument `nav2_service_timeout`, not a bare literal. A literal
   cannot be raised for one experiment without editing a launch file, and cannot be
   checked by this guard against the other file's copy.
3. The argument is DECLARED in `robot.launch.py` and in `fleet.launch.py`, and
   `fleet.launch.py` FORWARDS it into each robot's include. Point 3 is the same lesson as
   `check_launch_robots.py` and `D-P17-09`: two files holding their own value for one
   quantity is how a fleet run silently gets the single-robot setting, because the fleet
   launch never passes it and the robot launch falls back to its own default.
4. The value is bounded on BOTH sides. Below the floor, replies on this machine are late
   and the bringup aborts. Above the ceiling, the parameter would be hiding a node that
   never answers -- note that the ceiling can be generous, because a genuinely dead node
   is detected by the BOND (bond_timeout, 4.0 s), not by this timeout.
5. Self-test: with the forwarding removed by hand, this guard must exit non-zero. A guard
   that can only agree has been shown to be quiet, not to work (D-P17-10).

Exit 0 when the wiring is right, 1 otherwise.
"""

from __future__ import annotations

import ast
import os
import pathlib
import shutil
import subprocess
import sys
import tempfile

ROOT = pathlib.Path(os.environ.get("FLEET009_ROOT")
                    or pathlib.Path(__file__).resolve().parent.parent)
ROBOT = ROOT / "src/fleet_bringup/launch/robot.launch.py"
FLEET = ROOT / "src/fleet_bringup/launch/fleet.launch.py"

ARG = "nav2_service_timeout"

#: Measured floor. Below this the three-robot bringup does not complete on this machine;
#: see D-P17-24 for the run-by-run numbers. Kept as a floor rather than an equality so the
#: value can be tuned upward for a heavier fleet without a false failure.
MIN_S = 10.0

#: Ceiling. Not tight, because bond_timeout (4.0 s) is what catches a node that never
#: answers; this timeout only has to outlast a slow reply, and must not be so large that a
#: typo (e.g. 2000 instead of 20) goes unnoticed.
MAX_S = 120.0

failures: list[str] = []
notes: list[str] = []


def _parse(path: pathlib.Path) -> tuple[str, ast.Module]:
    src = path.read_text(encoding="utf-8")
    return src, ast.parse(src)


def _defaults_table(tree: ast.Module) -> dict[str, ast.AST]:
    """The module-level DEFAULTS mapping, {name: value node}."""
    for node in tree.body:
        if isinstance(node, ast.Assign):
            for target in node.targets:
                if isinstance(target, ast.Name) and target.id == "DEFAULTS":
                    if isinstance(node.value, ast.Dict):
                        return {
                            k.value: v
                            for k, v in zip(node.value.keys, node.value.values)
                            if isinstance(k, ast.Constant) and isinstance(k.value, str)
                        }
    return {}


def _dict_keys(node: ast.AST) -> dict[str, ast.AST]:
    """Every string key in every Dict under `node`, mapped to its value node."""
    out: dict[str, ast.AST] = {}
    for sub in ast.walk(node):
        if isinstance(sub, ast.Dict):
            for k, v in zip(sub.keys, sub.values):
                if isinstance(k, ast.Constant) and isinstance(k.value, str):
                    out.setdefault(k.value, v)
    return out


def check_lifecycle_manager_is_given_it() -> tuple[ast.AST | None, ast.AST | None]:
    if not ROBOT.exists():
        failures.append(f"{ROBOT.relative_to(ROOT)}: missing")
        return None, None
    src, tree = _parse(ROBOT)

    manager_params = None
    for call in ast.walk(tree):
        if not (isinstance(call, ast.Call) and isinstance(call.func, ast.Name)
                and call.func.id == "Node"):
            continue
        kw = {k.arg: k.value for k in call.keywords if k.arg}
        pkg = kw.get("package")
        if isinstance(pkg, ast.Constant) and pkg.value == "nav2_lifecycle_manager":
            manager_params = kw.get("parameters")

    if manager_params is None:
        failures.append("robot.launch.py: no Node(package='nav2_lifecycle_manager') found")
        return None, None

    keys = _dict_keys(manager_params)
    if "service_timeout" not in keys:
        failures.append(
            "robot.launch.py: the lifecycle_manager is NOT given `service_timeout`. It "
            "will use nav2's default of 5.0 s, which on this machine is not enough for a "
            "three-robot bringup: the manager times out on map_server/get_state and aborts "
            "the whole bringup, and every case in that run records NOT_RUN.")
        return manager_params, None

    value = keys["service_timeout"]
    if not (isinstance(value, ast.Name) and value.id == ARG):
        seg = ast.get_source_segment(src, value) or "?"
        failures.append(
            f"robot.launch.py: service_timeout is set to `{seg}` instead of the launch "
            f"argument `{ARG}`. A bare literal cannot be A/B'd without editing this file, "
            f"and this guard cannot compare it against fleet.launch.py's copy.")
        return manager_params, None

    notes.append(f"lifecycle_manager is given service_timeout = {ARG}")
    return manager_params, value


def check_declared_and_forwarded() -> None:
    """Declared in both files, forwarded by the fleet launch, and bounded."""
    robot_tree = ast.parse(ROBOT.read_text(encoding="utf-8"))
    fleet_tree = ast.parse(FLEET.read_text(encoding="utf-8"))

    robot_defaults = _defaults_table(robot_tree)
    fleet_defaults = _defaults_table(fleet_tree)

    for name, table, label in (("robot.launch.py", robot_defaults, ROBOT),
                               ("fleet.launch.py", fleet_defaults, FLEET)):
        if ARG not in table:
            failures.append(
                f"{name}: `{ARG}` is not declared in DEFAULTS. The launch file that reads "
                f"an argument must also declare it, or the value silently becomes whatever "
                f"the other file happens to hold.")

    # The fleet launch must PASS the argument into each robot include. Without this the
    # fleet run inherits robot.launch.py's default and the two can diverge again.
    forwarded = False
    for call in ast.walk(fleet_tree):
        if not (isinstance(call, ast.Call) and isinstance(call.func, ast.Name)
                and call.func.id == "IncludeLaunchDescription"):
            continue
        kw = {k.arg: k.value for k in call.keywords if k.arg}
        args = kw.get("launch_arguments")
        if args is not None and ARG in _dict_keys(args):
            forwarded = True
    if forwarded:
        notes.append(f"fleet.launch.py forwards {ARG} into each robot include")
    else:
        failures.append(
            f"fleet.launch.py: `{ARG}` is not forwarded in the per-robot "
            f"IncludeLaunchDescription. A fleet run is the ONLY configuration that starts "
            f"several Nav2 stacks at once, so it is exactly the run whose timeout must be "
            f"this value; leaving it unforwarded gives the fleet run robot.launch.py's "
            f"default instead.")

    # Bounds, taken from the value a FLEET run actually uses.
    value_node = fleet_defaults.get(ARG) or robot_defaults.get(ARG)
    if value_node is None:
        return
    # DEFAULTS maps a name to a (default, description) TUPLE, not to the bare value.
    # Reading `.value` off the tuple is an AttributeError, and catching it as "not a
    # number" would have made this guard report the wrong problem on every run.
    default_literal: ast.AST = value_node
    if isinstance(value_node, ast.Tuple) and value_node.elts:
        default_literal = value_node.elts[0]
    if not (isinstance(default_literal, ast.Constant)
            and isinstance(default_literal.value, str)):
        failures.append(
            f"{ARG}: default is not a string literal I can read: "
            f"{ast.dump(default_literal)[:80]}. Launch argument defaults must be strings.")
        return
    try:
        val = float(default_literal.value)
    except ValueError:
        failures.append(f"{ARG}: default {default_literal.value!r} is not a number")
        return

    if val < MIN_S:
        failures.append(
            f"{ARG} default is {val}s, below the measured floor of {MIN_S}s. Three-robot "
            f"bringup does not complete at stock 5.0 s on this machine (D-P17-24).")
    elif val > MAX_S:
        failures.append(
            f"{ARG} default is {val}s, above the ceiling of {MAX_S}s. A value this large "
            f"stops distinguishing 'slow reply' from 'no reply'.")
    else:
        notes.append(f"{ARG} default = {val}s (within [{MIN_S}, {MAX_S}])")


def check_self_test() -> None:
    """Prove the checker can fail: remove the forwarding and require non-zero."""
    with tempfile.TemporaryDirectory() as tmp:
        root = pathlib.Path(tmp) / "009"
        for rel in ("src/fleet_bringup/launch",):
            (root / rel).mkdir(parents=True, exist_ok=True)
        shutil.copy(ROBOT, root / "src/fleet_bringup/launch/robot.launch.py")
        broken = root / "src/fleet_bringup/launch/fleet.launch.py"
        text = FLEET.read_text(encoding="utf-8")
        # Drop the forwarding line, keeping the dict syntactically valid.
        text = text.replace(f'"{ARG}": _arg(context, "{ARG}"),', "")
        broken.write_text(text, encoding="utf-8")

        env = dict(os.environ)
        env["FLEET009_ROOT"] = str(root)
        proc = subprocess.run([sys.executable, __file__, "--no-self-test"],
                              env=env, capture_output=True, text=True, timeout=60)
        if proc.returncode == 0:
            failures.append(
                "self-test FAILED: with the fleet launch's forwarding removed, this guard "
                "still exited 0. A guard that cannot fail is not a guard.")
        else:
            notes.append(f"self-test: detects a removed forward (exit {proc.returncode})")


def main() -> int:
    if not FLEET.exists():
        failures.append(f"{FLEET.relative_to(ROOT)}: missing")

    check_lifecycle_manager_is_given_it()
    if FLEET.exists():
        check_declared_and_forwarded()
    if "--no-self-test" not in sys.argv:
        check_self_test()

    print("=" * 78)
    print("  009 Nav2 lifecycle service-timeout check (static)")
    print("=" * 78)
    for n in notes:
        print(f"    {n}")

    if failures:
        print()
        for f in failures:
            print(f"  [FAIL] {f}")
        print()
        print("A fleet run brings up several Nav2 stacks at once. If the lifecycle")
        print("manager's service timeout is sized for one robot, the manager aborts the")
        print("bringup, and the run loses its evidence instead of reporting a slow host.")
        return 1

    print()
    print("    the fleet's Nav2 lifecycle timeout is single-sourced, forwarded and bounded "
          "(exit 0)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
