#!/usr/bin/env python3
"""Dump every transform in the world-level truth message, so nothing is inferred.

Why this exists: a corridor-crossing probe read "ground truth" as entry 0 of
/world/<world>/dynamic_pose/info and produced a trajectory that is physically impossible
in places -- the value sat frozen at the spawn for seconds and then jumped 2.1 m in one
step while Nav2's own distance_remaining had not moved. Either the robot teleported or that
entry is not always the robot.

The channel drops frame names and stamps, so the ONLY way to tell which entry is the robot
is to look at the translations themselves. This prints all of them, with the message rate,
while whatever the caller commands is happening. No inference, no selection: if entry 0 is
not the robot, this shows it.

Read-only: subscribes and prints. Publishes nothing.
"""

from __future__ import annotations

import argparse
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy
from tf2_msgs.msg import TFMessage


class TruthDump(Node):
    def __init__(self, world: str) -> None:
        super().__init__("truth_dump",
                         parameter_overrides=[Parameter("use_sim_time", value=True)])
        self.count = 0
        self.rows: list[list[tuple[float, float, float]]] = []
        self.qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, depth=50)
        self.create_subscription(
            TFMessage, f"/world/{world}/dynamic_pose/info", self._on, self.qos)

    def _on(self, msg: TFMessage) -> None:
        self.count += 1
        row = [(float(t.transform.translation.x),
                float(t.transform.translation.y),
                float(t.transform.translation.z)) for t in msg.transforms]
        self.rows.append(row)
        if len(self.rows) > 4000:
            self.rows.pop(0)


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--world", default="warehouse")
    ap.add_argument("--seconds", type=float, default=12.0)
    ap.add_argument("--every", type=float, default=1.0,
                    help="seconds between printed snapshots")
    args, ros_args = ap.parse_known_args(argv)

    rclpy.init(args=ros_args)
    n = TruthDump(args.world)

    print("=" * 78)
    print("  raw dump of the world-level truth topic")
    print("=" * 78)
    t0 = time.monotonic()
    nxt = 0.0
    while time.monotonic() - t0 < args.seconds:
        rclpy.spin_once(n, timeout_sec=0.1)
        el = time.monotonic() - t0
        if el >= nxt and n.rows:
            row = n.rows[-1]
            print(f"\n  t={el:5.1f}s  transforms in this message: {len(row)}")
            for i, (x, y, z) in enumerate(row):
                mark = "   <-- entry 0" if i == 0 else ""
                print(f"    [{i}] ({x:+9.4f}, {y:+9.4f}, {z:+9.4f}){mark}")
            nxt += args.every

    rate = n.count / max(1e-9, args.seconds)
    print()
    print(f"  messages received: {n.count}  ({rate:.1f} Hz)")
    if n.rows:
        counts = {len(r) for r in n.rows}
        print(f"  transforms per message: {sorted(counts)}"
              + ("   <-- NOT constant, the set changes" if len(counts) > 1 else ""))
        # Does entry 0 ever sit at more than one place while the robot is still? Track the
        # distinct positions entry 0 took, rounded, with how long each lasted.
        seen: dict[tuple[float, float], int] = {}
        for r in n.rows:
            if r:
                key = (round(r[0][0], 3), round(r[0][1], 3))
                seen[key] = seen.get(key, 0) + 1
        print(f"  distinct entry-0 positions seen: {len(seen)}")
        for key, cnt in sorted(seen.items(), key=lambda kv: -kv[1])[:8]:
            print(f"    {key}  in {cnt} messages")
        if len(seen) == 1:
            print("  => entry 0 never moved during this window.")
        print()
        print("  READING:")
        if len(seen) > 1 and n.count and max(seen.values()) / n.count > 0.5:
            print("    entry 0 is ONE stable entity among several: it holds the same")
            print("    position for most messages, which is what a model pose looks like.")
            print("    If it ever reported a position no robot could reach, that entry was")
            print("    a different entity and the positional read is NOT safe.")
        else:
            print("    entry 0 moved freely, consistent with a single moving model.")
    n.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
