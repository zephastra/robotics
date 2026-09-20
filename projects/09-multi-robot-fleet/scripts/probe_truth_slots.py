#!/usr/bin/env python3
"""Which entity is at which index in the truth stream? Decided, not assumed.

WHY
---
`fleet_recorder` reads the world's `dynamic_pose/info` as ``transforms[slot * LINKS_PER_MODEL]``
with ``LINKS_PER_MODEL = 8``, i.e. it assumes every model contributes exactly eight links AND
that the model order is fixed. Two measurements say both assumptions are wrong here:

  * in `reports/batch_20260917T082919Z/N01` the stream carried **1, 2 and 3 models at different
    times within one run** (raw widths 8/16/24), so an index is not a stable identity;
  * in `reports/batch_20260916T151619Z/N03` -- a TWO-robot case -- the stream carried two slots,
    one starting on r01's spawn and one at (0.200, 0.000) that never moved, while **r02's own
    odometry shows 9.902 m of travel**. r02 has no truth slot, and until that is explained the
    20 blocked two-robot runs cannot be judged.

This probe answers it directly: subscribe to the truth topic, and for every message record the
number of transforms and where every transform index is. A robot's spawn appearing at an index
that is NOT a multiple of 8 is the whole answer.

It publishes nothing, calls no service, and reads for verification only (rule 13). Its own
failure mode is that it saw nothing, which is why it prints its counts on exit.

Usage (through the wrapper):
    scripts/probe_truth_slots.sh --world warehouse --spawns 6.0,-2.0 --duration 60 --out <dir>

Exit codes: 0 fine, 3 no rclpy (wrong interpreter), 4 nothing seen, 5 seen but a named spawn
never appeared at any index.
"""
from __future__ import annotations

import argparse
import json
import pathlib
import sys
import time

try:
    import rclpy  # noqa: F401
except Exception as exc:  # pragma: no cover
    print(f"PREFLIGHT FAILED: cannot import rclpy ({type(exc).__name__}: {exc}).\n"
          "Run this through scripts/probe_truth_slots.sh, which picks /usr/bin/python3.",
          file=sys.stderr)
    raise SystemExit(3)

from rclpy.executors import ExternalShutdownException  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.qos import QoSProfile, ReliabilityPolicy  # noqa: E402
from tf2_msgs.msg import TFMessage  # noqa: E402


class TruthSlots(Node):
    def __init__(self, topic: str, spawns: dict[str, tuple[float, float]], out: pathlib.Path):
        super().__init__("truth_slots")
        self.spawns = spawns
        self.out = out
        self.messages = 0
        self.widths: dict[int, int] = {}
        # index -> the positions seen there, so an index that changes hands is visible
        self.positions: dict[int, list[tuple[float, float]]] = {}
        self.names: dict[int, set[str]] = {}
        self.started = time.monotonic()
        self.create_subscription(
            TFMessage, topic, self._on_msg,
            QoSProfile(reliability=ReliabilityPolicy.RELIABLE, depth=50))

    def _on_msg(self, msg: TFMessage) -> None:
        self.messages += 1
        width = len(msg.transforms)
        self.widths[width] = self.widths.get(width, 0) + 1
        for index, tr in enumerate(msg.transforms):
            t = tr.transform.translation
            self.positions.setdefault(index, []).append((float(t.x), float(t.y)))
            # The bridge drops the pose name, but if a build keeps it in child_frame_id it is
            # the strongest possible answer, so it is recorded when present rather than
            # assumed absent.
            name = (tr.child_frame_id or "").strip()
            if name:
                self.names.setdefault(index, set()).add(name)
        self._write_raw(msg)

    def _write_raw(self, msg: TFMessage) -> None:
        row = {"t": round(time.monotonic() - self.started, 3), "width": len(msg.transforms),
               "poses": [[round(float(tr.transform.translation.x), 4),
                          round(float(tr.transform.translation.y), 4),
                          (tr.child_frame_id or "")] for tr in msg.transforms]}
        with (self.out / "truth_slots_raw.jsonl").open("a", encoding="utf-8") as fh:
            fh.write(json.dumps(row) + "\n")

    def report(self) -> int:
        print(f"truth messages: {self.messages}")
        print("transforms per message: "
              + ", ".join(f"{k}x{v}" for k, v in sorted(self.widths.items())))
        if not self.messages:
            print("NOTHING SEEN -- this probe says nothing about the world.")
            return 4
        print()
        print(f"  {'idx':>4} {'n':>6}  {'first':>22}  {'last':>22}  {'moved_m':>8}  name")
        import math
        found: dict[str, int] = {}
        for index in sorted(self.positions):
            pts = self.positions[index]
            first, last = pts[0], pts[-1]
            moved = sum(math.hypot(b[0] - a[0], b[1] - a[1])
                        for a, b in zip(pts, pts[1:]))
            names = ",".join(sorted(self.names.get(index, ())))
            print(f"  {index:>4} {len(pts):>6}  ({first[0]:9.3f},{first[1]:9.3f})  "
                  f"({last[0]:9.3f},{last[1]:9.3f})  {moved:8.3f}  {names}")
            for robot, (sx, sy) in self.spawns.items():
                if robot not in found and math.hypot(first[0] - sx, first[1] - sy) <= 1.0:
                    found[robot] = index

        print()
        print("every index a spawn first appeared at (LINKS_PER_MODEL is 8, so a base link is "
              "expected at a multiple of 8):")
        for robot in sorted(self.spawns):
            sx, sy = self.spawns[robot]
            idx = found.get(robot)
            if idx is None:
                print(f"  {robot}: spawn ({sx:.3f}, {sy:.3f}) -- NO INDEX EVER STARTED THERE")
            else:
                step = "a multiple of 8" if idx % 8 == 0 else "NOT a multiple of 8"
                print(f"  {robot}: spawn ({sx:.3f}, {sy:.3f}) first seen at index {idx} -- "
                      f"{step}")

        missing = [r for r in sorted(self.spawns) if r not in found]
        if missing:
            print(f"\nMISSING: {missing} never appeared in this stream at any index. That is "
                  "the fact that makes a two-robot safety verdict unobtainable, and it is a "
                  "property of the world/bridge, not of the judge.")
            return 5
        return 0


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--world", default="warehouse")
    ap.add_argument("--spawns", default="",
                    help="robot=x,y pairs separated by ';', e.g. 'r01=-6,-2;r02=6,-2'")
    ap.add_argument("--out", default=".")
    ap.add_argument("--duration", type=float, default=0.0)
    args = ap.parse_args(argv)

    spawns: dict[str, tuple[float, float]] = {}
    for chunk in [c for c in args.spawns.split(";") if c.strip()]:
        robot, _, xy = chunk.partition("=")
        xs, _, ys = xy.partition(",")
        spawns[robot.strip()] = (float(xs), float(ys))

    out = pathlib.Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    (out / "truth_slots_raw.jsonl").write_text("", encoding="utf-8")

    rclpy.init()
    node = TruthSlots(f"/world/{args.world}/dynamic_pose/info", spawns, out)
    print(f"subscribed: /world/{args.world}/dynamic_pose/info; spawns {spawns}")
    started = time.monotonic()
    try:
        while rclpy.ok():
            try:
                rclpy.spin_once(node, timeout_sec=0.2)
            except ExternalShutdownException:
                break
            if args.duration and (time.monotonic() - started) >= args.duration:
                break
    finally:
        code = node.report()
        node.destroy_node()
        rclpy.shutdown()
    return code


if __name__ == "__main__":
    raise SystemExit(main())
