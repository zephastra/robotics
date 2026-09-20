#!/usr/bin/env python3
"""Dump the ACTUAL costmap that the planner refused to plan through.

Item 2 of the round-17 work list: "把「规划失败」的完整代价地图快照取出来（不是猜）".
The planner said, four separate times:

    GridBasedplugin failed to plan from (2.27, 0.05) ... to (-2.50, -1.40):
    "Failed to create plan with tolerance of: 0.500000"

Four recordings, 34/19/19 attempts, always to (-2.50, -1.40) -- `exit_west` in
resources.yaml, which the file itself says is "still INSIDE the bundle". This
script does not guess why. It saves the costmap.

It subscribes with TRANSIENT_LOCAL durability (Nav2 costmaps latch), waits for
one map on each of the topic's layers, and writes:

    <out>/costmap-<robot>-<topic>.pgm   the raw occupancy bytes, P5
    <out>/costmap-<robot>-<topic>.yaml  metadata (resolution, origin, size)
    <out>/snapshot.json                 everything above plus the caller's pose
                                        and a plan attempt to exit_west

It ALSO asks for the path to exit_west from the robot's current pose, so the
snapshot and the refusal are recorded in the same breath.

Usage (inside a sourced ROS shell, while a run is up):
    /usr/bin/python3 scripts/dump_costmap.py --robot r01 --out /tmp/cm

Read-only with respect to the fleet: it subscribes and asks for a path. It
publishes nothing and moves nothing.
"""

from __future__ import annotations

import argparse
import json
import math
import pathlib
import sys
import time

import rclpy
from nav2_msgs.action import ComputePathToPose
from nav_msgs.msg import OccupancyGrid, Path
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy)

#: The goal the planner refused, four recordings running. From config/resources.yaml.
EXIT_WEST = (-2.50, -1.40)

#: Nav2 latches costmaps; anything other than TRANSIENT_LOCAL gets nothing.
LATCHED = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=1,
)

TOPICS = {
    "global": "global_costmap/costmap",
    "local": "local_costmap/costmap",
    "global_updates": "global_costmap/costmap_updates",
    "global_raw": "global_costmap/costmap_raw",
}


def write_pgm(path: pathlib.Path, grid: OccupancyGrid) -> None:
    """Write an OccupancyGrid as a P5 PGM. Values as Nav2 publishes them."""
    w, h = grid.info.width, grid.info.height
    data = bytes((255 if v < 0 else (0 if v >= 99 else int(255 * (100 - v) / 100)))
                 for v in grid.data)
    with open(path, "wb") as fh:
        fh.write(b"P5\n")
        fh.write(f"# {grid.header.frame_id} {w}x{h} res={grid.info.resolution}\n".encode())
        fh.write(f"{w} {h}\n255\n".encode())
        fh.write(data)


class CostmapDumper(Node):
    def __init__(self, robot: str, out: pathlib.Path) -> None:
        super().__init__("costmap_dumper",
                         parameter_overrides=[Parameter("use_sim_time", value=True)])
        self.robot = robot
        self.out = out
        self.grids: dict[str, OccupancyGrid] = {}
        self.plan_result: dict = {}
        self.pose: dict = {}

        for name, topic in TOPICS.items():
            full = f"/{robot}/{topic}"
            self.create_subscription(
                OccupancyGrid, full,
                lambda msg, n=name: self.grids.__setitem__(n, msg),
                LATCHED)

        self.plan_client = ActionClient(self, ComputePathToPose,
                                        f"/{robot}/compute_path_to_pose")
        self.create_subscription(Path, f"/{robot}/plan", self._on_plan, 10)
        self.plan_seen = None

    def _on_plan(self, msg: Path) -> None:
        self.plan_seen = len(msg.poses)

    # ------------------------------------------------------------------ helpers
    def spin_for(self, seconds: float) -> None:
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.1)

    def ask_for_path(self, goal_xy: tuple[float, float]) -> dict:
        """Ask for a path to `goal_xy` from the robot's believed pose."""
        if not self.plan_client.wait_for_server(timeout_sec=10.0):
            return {"attempted": False, "reason": "no compute_path_to_pose server"}
        g = ComputePathToPose.Goal()
        g.use_start = False                      # let Nav2 use the robot's own pose
        g.goal.header.frame_id = "map"
        g.goal.pose.position.x = goal_xy[0]
        g.goal.pose.position.y = goal_xy[1]
        g.goal.pose.orientation.w = 1.0

        fut = self.plan_client.send_goal_async(g)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=20.0)
        handle = fut.result()
        if handle is None or not handle.accepted:
            return {"attempted": True, "accepted": False}
        res_fut = handle.get_result_async()
        rclpy.spin_until_future_complete(self, res_fut, timeout_sec=25.0)
        wrapper = res_fut.result()
        if wrapper is None:
            return {"attempted": True, "accepted": True, "result": "timeout"}
        poses = wrapper.result.path.poses if wrapper.result is not None else []
        length = 0.0
        for a, b in zip(poses, poses[1:]):
            length += math.dist((a.pose.position.x, a.pose.position.y),
                                (b.pose.position.x, b.pose.position.y))
        return {
            "attempted": True,
            "accepted": True,
            "status": wrapper.status,
            "poses": len(poses),
            "length_m": round(length, 3),
        }


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--robot", default="r01")
    ap.add_argument("--out", default="/tmp/cm")
    ap.add_argument("--wait", type=float, default=20.0,
                    help="seconds to wait for a latched costmap")
    args, ros_args = ap.parse_known_args(argv)

    out = pathlib.Path(args.out)
    out.mkdir(parents=True, exist_ok=True)

    rclpy.init(args=ros_args)
    node = CostmapDumper(args.robot, out)

    end = time.monotonic() + args.wait
    while time.monotonic() < end and "global" not in node.grids:
        rclpy.spin_once(node, timeout_sec=0.2)

    summary: dict = {"robot": args.robot, "layers": {}, "goal": list(EXIT_WEST)}

    for name, grid in sorted(node.grids.items()):
        w, h = grid.info.width, grid.info.height
        vals = list(grid.data)
        occupied = sum(1 for v in vals if v >= 99)
        unknown = sum(1 for v in vals if v < 0)
        free = w * h - occupied - unknown
        summary["layers"][name] = {
            "width": w, "height": h,
            "resolution": grid.info.resolution,
            "origin": [grid.info.origin.position.x, grid.info.origin.position.y],
            "frame": grid.header.frame_id,
            "occupied": occupied, "free": free, "unknown": unknown,
            "max_cost": max(vals) if vals else None,
        }
        pgm = out / f"costmap-{args.robot}-{name}.pgm"
        yml = out / f"costmap-{args.robot}-{name}.yaml"
        write_pgm(pgm, grid)
        yml.write_text(
            f"image: {pgm.name}\n"
            f"resolution: {grid.info.resolution}\n"
            f"origin: [{grid.info.origin.position.x}, {grid.info.origin.position.y}, 0.0]\n"
            f"negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n",
            encoding="utf-8")
        print(f"  wrote {pgm}  ({w}x{h}, {occupied} occupied, {unknown} unknown)")

        # What does the cell AT exit_west say?
        gx = int((EXIT_WEST[0] - grid.info.origin.position.x) / grid.info.resolution)
        gy = int((EXIT_WEST[1] - grid.info.origin.position.y) / grid.info.resolution)
        if 0 <= gx < w and 0 <= gy < h:
            summary["layers"][name]["exit_west_cell"] = {
                "ix": gx, "iy": gy, "cost": vals[gy * w + gx],
            }

    if "global" not in node.grids:
        summary["error"] = "no latched global costmap arrived; is the run up?"

    summary["plan_to_exit_west"] = node.ask_for_path(EXIT_WEST)
    summary["plan_topic_last_len"] = node.plan_seen

    (out / "snapshot.json").write_text(
        json.dumps(summary, indent=2), encoding="utf-8")
    print(json.dumps(summary, indent=2))

    node.destroy_node()
    rclpy.shutdown()
    return 0 if "global" in node.grids else 2


if __name__ == "__main__":
    sys.exit(main())
