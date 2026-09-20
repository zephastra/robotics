#!/usr/bin/env python3
"""Assert two robots in one world do not interfere. The P3 gate.

Two robots sharing a world is exactly the configuration where things fail in ways that
look like something else: a shared topic name means one robot drives both, a shared TF
frame means one robot's pose overwrites the other's, and a namespace typo means a robot
silently listens to a topic nobody publishes. None of those produce an error.

So this checks, from inside the process:

  1. every expected per-robot topic exists, under that robot's namespace, and no bare
     global topic of the same name exists (a bare /cmd_vel is the failure mode)
  2. each /<robot>/cmd_vel has EXACTLY ONE publisher, and it is THAT robot's safety_gate
  3. no topic is published from more than one namespace
  4. each robot's TF chain map -> <robot>/base_link resolves, and lands near its OWN
     spawn pose (config/spawns.yaml), not the other robot's
  5. a goal sent to one robot moves THAT robot and does not move the other

Point 2 is load bearing. The whole safety argument for 009 rests on the gate being the
single publisher of the wheels' velocity topic; with two robots that claim has to hold
per robot, and "Publisher count: 1" is not enough on its own -- it must be 1 AND in the
right namespace.

Point 5 needs a caveat about the truth channel, because it is the weak link. The world
level /world/<world>/dynamic_pose/info does carry every dynamic model, but the
Pose_V -> TFMessage mapping drops the pose NAME and leaves the stamp at zero, so entries
cannot be addressed by name. They CAN be addressed by their position in the message,
which is stable for a session, and the identity of each index is then established from
geometry -- see identify(). That identification is printed, with its margins, and the
run fails if it is ambiguous rather than guessing.

Uses in-process rclpy discovery rather than the ros2 CLI on purpose: on this machine the
CLI reports "Unknown topic" for topics that demonstrably carry data, and a partial graph
looks exactly like a broken system.

The executor runs on its own thread. That is not style: a sampling loop that calls
spin_once once per tick and then sleeps throttles its own callback delivery, which in
this project once produced a completely fake "the transport collapses while driving"
result (see 009_INSTRUMENT_ARTIFACT.md). Nothing here may pace delivery.
"""

from __future__ import annotations

import argparse
import math
import pathlib
import sys
import threading
import time

import rclpy
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer, TransformListener

# Topics every robot must own. joint_states is deliberately NOT here: it is off by
# default now (bridge_joint_states:=false) because nothing in the control path reads
# wheel angles, so requiring it would be a false failure.
REQUIRED_TOPICS = ["cmd_vel", "cmd_vel_pre_gate", "scan", "odom", "plan", "gate_state"]
OPTIONAL_TOPICS = ["joint_states", "amcl_pose"]

# Topics that are SUPPOSED to come from more than one namespace. Kept as a guard for
# anything whose LEAF name happens to match a per-robot suffix.
#
# The first version of phase 3 flagged ANY topic published from two namespaces, which
# failed on /tf, /diagnostics and /parameter_events. All three are shared by design: TF is
# one tree, every lifecycle manager publishes diagnostics, and every ROS node publishes
# parameter events. That produced a "isolation is NOT established" verdict on a system
# where every real property held -- which is the worst kind of check, because it teaches
# you to skim past its failures.
#
# So phase 3 is now driven by the LEAF NAME instead: only a topic whose last path segment
# is one of the per-robot suffixes is checked for cross-robot traffic. Infrastructure
# topics are skipped automatically, without maintaining an allowlist of every topic ROS
# happens to create.
SHARED_TOPICS = {"/tf", "/tf_static", "/clock", "/rosout", "/rosout_agg"}
SHARED_PREFIXES = ("/world/",)


class IsolationCheck(Node):
    def __init__(self, robots: list[str], world: str) -> None:
        super().__init__("isolation_check",
                         parameter_overrides=[Parameter("use_sim_time", value=True)])
        self.robots = robots
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)

        # Truth is tracked PER MESSAGE INDEX, not by name: the bridge drops the name, so a
        # name-keyed dict would put every model under one empty key and silently compare
        # the wrong things. The index is stable within a session because the models are
        # spawned once.
        self.tracks: dict[int, list[tuple[float, float, float]]] = {}
        self.truth_msgs = 0
        self.empty_names = 0

        self.create_subscription(
            TFMessage, f"/world/{world}/dynamic_pose/info", self._on_truth,
            QoSProfile(reliability=ReliabilityPolicy.RELIABLE, depth=50))
        # NOT `self.clients`: rclpy.node.Node already has a `clients` property (it returns
        # the node's service clients) and it has no setter, so assigning it raises
        # AttributeError while the node is being constructed. That cost a full P3 run
        # before anything was checked.
        self.nav_clients = {
            r: ActionClient(self, NavigateToPose, f"/{r}/navigate_to_pose")
            for r in robots
        }

    # ------------------------------------------------------------------ #

    def _on_truth(self, msg: TFMessage) -> None:
        now = self.sim()
        self.truth_msgs += 1
        for i, tr in enumerate(msg.transforms):
            if not (tr.child_frame_id or "").strip():
                self.empty_names += 1
            t = tr.transform.translation
            self.tracks.setdefault(i, []).append((now, float(t.x), float(t.y)))

    def sim(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def positions(self) -> dict[int, tuple[float, float]]:
        return {i: (h[-1][1], h[-1][2]) for i, h in self.tracks.items() if h}

    # ---- discovery (in process, never the CLI) ------------------------------- #

    def topics(self) -> list[str]:
        return [n for n, _ in self.get_topic_names_and_types()]

    def pubs(self, topic: str) -> list[tuple[str, str]]:
        try:
            infos = self.get_publishers_info_by_topic(topic)
        except Exception:
            return []
        return sorted({((i.node_namespace or "/").rstrip("/") or "/", i.node_name)
                       for i in infos})

    def belief(self, robot: str) -> tuple[float, float] | None:
        for frame in (f"{robot}/base_link", f"{robot}/base_footprint"):
            for t in (Time(), Time(seconds=0.0)):
                try:
                    tr = self.buffer.lookup_transform("map", frame, t,
                                                      timeout=Duration(seconds=0.3))
                except Exception:
                    continue
                p = tr.transform.translation
                return (float(p.x), float(p.y))
        return None

    # ---- identification ------------------------------------------------------ #

    def identify(self, spawns: dict[str, tuple[float, float]]) -> tuple[dict[str, int],
                                                                        list[str]]:
        """Map robot -> transform index, from geometry, and refuse to guess.

        Two independent sources of identity are available and both are used as a
        cross-check: each robot's spawn pose from config/spawns.yaml, and each robot's
        own belief (map -> <robot>/base_link) from its AMCL/TF chain. They must agree on
        the same index. If the two robots are too close to tell apart, or two robots map
        to one index, this reports a problem instead of picking one.
        """
        problems: list[str] = []
        pos = self.positions()
        if not pos:
            return {}, ["truth channel carried no transforms -- cannot identify models"]
        if len(pos) < len(self.robots):
            problems.append(
                f"truth channel has {len(pos)} dynamic model(s) but {len(self.robots)} "
                "robot(s) are expected; identification would be guesswork")

        def nearest(target: tuple[float, float]) -> tuple[int, float, float]:
            ranked = sorted(((i, math.dist(target, p)) for i, p in pos.items()),
                            key=lambda kv: kv[1])
            i, d = ranked[0]
            runner = ranked[1][1] if len(ranked) > 1 else float("inf")
            return i, d, runner

        mapping: dict[str, int] = {}
        print(f"    {'robot':<6}{'by spawn':>10}{'d':>8}{'2nd':>8}   "
              f"{'by belief':>12}{'d':>8}{'2nd':>8}")
        for r in self.robots:
            row = f"    {r:<6}"
            idx_s = idx_b = None
            if r in spawns:
                idx_s, d, runner = nearest(spawns[r])
                row += f"{idx_s:>10}{d:>8.3f}{runner:>8.3f}"
                if runner - d < 0.5:
                    problems.append(
                        f"{r}: spawn match is ambiguous (index {idx_s} at {d:.3f} m vs "
                        f"next at {runner:.3f} m)")
            else:
                row += f"{'n/a':>10}{'-':>8}{'-':>8}"
            b = self.belief(r)
            if b is not None:
                idx_b, d2, runner2 = nearest(b)
                row += f"{idx_b:>12}{d2:>8.3f}{runner2:>8.3f}"
                if runner2 - d2 < 0.5:
                    problems.append(
                        f"{r}: belief match is ambiguous (index {idx_b} at {d2:.3f} m vs "
                        f"next at {runner2:.3f} m)")
            else:
                row += f"{'n/a':>12}{'-':>8}{'-':>8}"
            print(row)

            if idx_s is None and idx_b is None:
                problems.append(f"{r}: no way to identify its model")
                continue
            if idx_s is not None and idx_b is not None and idx_s != idx_b:
                problems.append(
                    f"{r}: spawn says index {idx_s} but its own belief says index "
                    f"{idx_b} -- the two robots' geometry is not self-consistent")
                continue
            mapping[r] = idx_s if idx_s is not None else idx_b

        if len(set(mapping.values())) != len(mapping):
            problems.append(f"two robots resolved to the same model index: {mapping}")
        return mapping, problems


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--robots", default="r01,r02")
    ap.add_argument("--world", default="warehouse")
    ap.add_argument("--spawns", default="", help="config/spawns.yaml")
    ap.add_argument("--move-test", action="store_true",
                    help="phase 5: goal for the second robot, watch the first")
    ap.add_argument("--warmup", type=float, default=70.0)
    ap.add_argument("--goal-timeout", type=float, default=90.0)
    args, ros_args = ap.parse_known_args(argv)
    robots = [r.strip() for r in args.robots.split(",") if r.strip()]

    def read_spawns(path: pathlib.Path) -> dict[str, tuple[float, float]]:
        """The spawn table, parsed locally.

        _common.py lives in the launch directory and is not importable from here; the
        file's shape is fixed (spawns: -> robot: -> x/y), so a strict local parser is
        clearer than a sys.path trick. yaml itself is not a declared dependency of this
        package and a hand parser keeps the checker runnable standalone.
        """
        out: dict[str, tuple[float, float]] = {}
        cur: str | None = None
        in_spawns = False
        for lineno, raw in enumerate(path.read_text(encoding="utf-8").splitlines(), 1):
            line = raw.split("#", 1)[0].rstrip()
            if not line.strip():
                continue
            indent = len(line) - len(line.lstrip())
            key, _, val = line.strip().partition(":")
            val = val.strip()
            if indent == 0:
                in_spawns = key == "spawns"
                continue
            if not in_spawns:
                continue
            if indent == 2:
                cur = key
                out[cur] = (0.0, 0.0)
            elif indent == 4 and cur is not None and key in ("x", "y"):
                x, y = out[cur]
                try:
                    v = float(val)
                except ValueError as exc:
                    raise SystemExit(f"{path}:{lineno}: {val!r} is not a number") from exc
                out[cur] = (v, y) if key == "x" else (x, v)
        if not out:
            raise SystemExit(f"{path}: parsed to nothing")
        return out

    spawns = read_spawns(pathlib.Path(args.spawns)) if args.spawns else {}

    print("=" * 78)
    print(f"  009 fleet isolation check: {', '.join(robots)}")
    print("=" * 78)

    rclpy.init(args=ros_args)
    n = IsolationCheck(robots, args.world)
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(n)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    problems: list[str] = []
    print(f"  warming up for up to {args.warmup:.0f} s ...")
    deadline = time.monotonic() + args.warmup
    while time.monotonic() < deadline:
        time.sleep(0.2)
        if n.truth_msgs and all(n.belief(r) for r in robots):
            break
    time.sleep(2.0)
    print(f"  truth messages received: {n.truth_msgs}   "
          f"transforms with an EMPTY child_frame_id: {n.empty_names}")

    seen = set(n.topics())

    # ---- 1. topics ---------------------------------------------------------- #
    print()
    print("1. per-robot topics present, namespaced, and no bare global twin")
    for r in robots:
        missing = [f"/{r}/{t}" for t in REQUIRED_TOPICS if f"/{r}/{t}" not in seen]
        if missing:
            problems.append(f"/{r}: missing topics {missing}")
        else:
            print(f"  [ ok ] /{r}: all {len(REQUIRED_TOPICS)} required topics present")
        # "appears in the graph" is NOT the same as "carries data": a topic exists as soon
        # as something subscribes to it, and robot_state_publisher subscribes to
        # joint_states whether or not anything publishes it. The first version of this
        # line reported joint_states as "present" while its bridge was disabled, which is
        # exactly the kind of half-true line that gets quoted later. So report publishers.
        opt = [(t, len(n.pubs(f"/{r}/{t}"))) for t in OPTIONAL_TOPICS]
        print("         optional: " + ", ".join(
            f"{t}={'data' if c else 'no publisher'}" for t, c in opt))
    # A bare /cmd_vel (or /scan, ...) is the failure this catches: it means something is
    # publishing outside every namespace, so both robots could subscribe to it.
    bare = sorted(t for t in seen
                  if t.count("/") == 1 and t.lstrip("/") in REQUIRED_TOPICS + OPTIONAL_TOPICS)
    if bare:
        problems.append(f"bare global topics exist alongside the namespaced ones: {bare}")
    else:
        print("  [ ok ] no bare global topic shadowing a per-robot name")

    # ---- 2. sole publisher per robot ---------------------------------------- #
    print()
    print("2. sole publisher of each cmd_vel, and it is the gate of THAT robot")
    for r in robots:
        topic = f"/{r}/cmd_vel"
        pubs = n.pubs(topic)
        if len(pubs) != 1:
            problems.append(f"{topic}: {len(pubs)} publishers {pubs} (want exactly 1)")
        elif pubs[0] != (f"/{r}", "safety_gate"):
            problems.append(f"{topic}: publisher is {pubs[0][0]}/{pubs[0][1]}, "
                            f"want /{r}/safety_gate")
        else:
            print(f"  [ ok ] {topic}: exactly 1 publisher, /{r}/safety_gate")

    # ---- 3. no cross-namespace publisher ------------------------------------ #
    print()
    print("3. no per-robot topic carries two robots' traffic")
    per_robot_leaves = set(REQUIRED_TOPICS) | set(OPTIONAL_TOPICS)
    crossed: list[tuple[str, list[tuple[str, str]]]] = []
    checked = 0
    skipped = 0
    for t in sorted(seen):
        leaf = t.rsplit("/", 1)[-1]
        if leaf not in per_robot_leaves:
            # /tf, /diagnostics, /parameter_events and friends: not a per-robot name, so
            # never cross-talk by construction.
            skipped += 1
            continue
        if t in SHARED_TOPICS or t.startswith(SHARED_PREFIXES):
            skipped += 1
            continue
        checked += 1
        p = n.pubs(t)
        if len({ns for ns, _ in p}) > 1:
            crossed.append((t, p))
    if crossed:
        for t, p in crossed:
            problems.append(f"{t} published from {p}")
    else:
        print(f"  [ ok ] {checked} per-robot topic(s) checked, every one has publishers "
              f"from exactly one namespace ({skipped} infrastructure topic(s) skipped)")

    # ---- 4. TF and pose ----------------------------------------------------- #
    print()
    print("4. TF map -> <robot>/base_link, and where each robot believes it is")
    for r in robots:
        got = n.belief(r)
        if got is None:
            problems.append(f"map -> {r}/base_link does not resolve")
            continue
        if r in spawns:
            sx, sy = spawns[r]
            d = math.dist(got, (sx, sy))
            if d > 1.0:
                problems.append(
                    f"{r} believes it is {d:.3f} m from its own spawn "
                    f"({sx:+.2f}, {sy:+.2f})")
            else:
                print(f"  [ ok ] map -> {r}/base_link = ({got[0]:+.3f}, {got[1]:+.3f}), "
                      f"{d:.3f} m from its spawn ({sx:+.2f}, {sy:+.2f})")
        else:
            print(f"  [ ok ] map -> {r}/base_link = ({got[0]:+.3f}, {got[1]:+.3f})")
        for o in robots:
            if o != r and o in spawns and math.dist(got, spawns[o]) < 0.5:
                problems.append(f"{r} believes it is AT {o}'s spawn {spawns[o]}")

    # ---- 4b. identify the truth entries ------------------------------------- #
    print()
    print("4b. which truth transform index is which robot (names are dropped by the bridge)")
    mapping: dict[str, int] = {}
    if len(robots) >= 2:
        mapping, id_problems = n.identify(spawns)
        problems += id_problems
        if mapping:
            print(f"    resolved: {mapping}")
        if not id_problems:
            print("  [ ok ] every robot identified, unambiguously, by two independent "
                  "geometric sources")
    else:
        print("    single robot: identification not needed")

    # ---- 5. motion isolation ------------------------------------------------ #
    if args.move_test and len(robots) >= 2:
        mover, watcher = robots[1], robots[0]
        print()
        print(f"5. motion isolation: goal accepted by {mover}, watch {watcher}")
        if mover not in mapping or watcher not in mapping:
            problems.append("cannot run the motion test: models not identified")
        else:
            mi, wi = mapping[mover], mapping[watcher]
            before = n.positions()
            client = n.nav_clients[mover]
            if not client.wait_for_server(timeout_sec=25.0):
                problems.append(f"no navigate_to_pose action server for {mover}")
            else:
                gx, gy = spawns.get(mover, (6.0, -2.0))
                goal = NavigateToPose.Goal()
                goal.pose.header.frame_id = "map"
                goal.pose.header.stamp = n.get_clock().now().to_msg()
                # Two metres toward the middle of the mover's OWN half, so this is a
                # within-half move that needs no corridor reservation (that is P4).
                goal.pose.pose.position.x = gx - 2.0 if gx > 0 else gx + 2.0
                goal.pose.pose.position.y = gy
                goal.pose.pose.orientation.w = 1.0
                print(f"    goal: ({goal.pose.pose.position.x:+.2f}, "
                      f"{goal.pose.pose.position.y:+.2f}) for {mover} at spawn "
                      f"({gx:+.2f}, {gy:+.2f})")

                fut = client.send_goal_async(goal)
                t_end = time.monotonic() + 25.0
                while not fut.done() and time.monotonic() < t_end:
                    time.sleep(0.1)
                handle = fut.result() if fut.done() else None
                if handle is None or not handle.accepted:
                    problems.append(f"{mover}'s goal was not accepted")
                else:
                    res = handle.get_result_async()
                    t_end = time.monotonic() + args.goal_timeout
                    while not res.done() and time.monotonic() < t_end:
                        time.sleep(0.1)
                    status = "TIMED OUT (the robot may still be moving)"
                    if res.done():
                        status = {
                            4: "SUCCEEDED", 5: "CANCELED", 6: "ABORTED",
                        }.get(res.result().status, f"status {res.result().status}")
                    else:
                        handle.cancel_goal_async()
                    print(f"    {mover} goal result: {status}")

                    time.sleep(3.0)          # let the last of the motion land
                    after = n.positions()
                    b_m, a_m = before.get(mi), after.get(mi)
                    b_w, a_w = before.get(wi), after.get(wi)
                    if not all((b_m, a_m, b_w, a_w)):
                        problems.append("truth positions missing across the motion test")
                    else:
                        dm = math.dist(b_m, a_m)
                        dw = math.dist(b_w, a_w)
                        print(f"    {mover} (model index {mi}) moved {dm:.4f} m")
                        print(f"    {watcher} (model index {wi}) moved {dw:.4f} m")
                        if dm < 0.20:
                            problems.append(
                                f"{mover} was commanded 2 m but its TRUE pose moved only "
                                f"{dm:.4f} m -- the goal did not reach the plant")
                        else:
                            print(f"  [ ok ] {mover} moved as commanded ({dm:.3f} m)")
                        if dw > 0.05:
                            problems.append(
                                f"{watcher} moved {dw:.4f} m while only {mover} was "
                                "given a goal -- the two are coupled")
                        else:
                            print(f"  [ ok ] {watcher} did not move "
                                  f"(<0.05 m) while {mover} drove")

    # ---- verdict ------------------------------------------------------------ #
    print()
    print("=" * 78)
    if problems:
        print(f"  VERDICT: {len(problems)} problem(s). Isolation is NOT established.")
        for p in problems:
            print(f"    - {p}")
    else:
        print("  VERDICT: isolation holds for every property checked here.")
        print("  NOT CHECKED: whether either robot can complete a corridor crossing")
        print("  alongside the other, collision avoidance between them, and any")
        print("  physical or safety claim. Corridor arbitration is P4.")
    print("=" * 78)
    executor.shutdown()
    n.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
    return 1 if problems else 0


if __name__ == "__main__":
    sys.exit(main())
