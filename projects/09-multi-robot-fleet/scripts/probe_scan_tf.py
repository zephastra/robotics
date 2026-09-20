#!/usr/bin/env python3
"""Resolve the laser scan's TF chain with a real tf2 Buffer. READ-ONLY.

The CLI tools disagree with each other about whether `map` and `r01/odom` exist
(tf2_monitor listed them, tf2_echo said "frame does not exist"), so neither can be
trusted to answer "where does the chain break". This uses the same library AMCL
uses, with the same clock, and reports the first link that fails and the exact
exception.

If the chain to r01/base_footprint resolves but map does not, AMCL is fine and the
problem is that AMCL itself never produced an update. If a lower link fails, that
link is the real blocker.
"""

from __future__ import annotations

import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener


CHAIN = [
    ("r01/base_footprint", "r01/base_link"),
    ("r01/base_link", "r01/laser_link"),
    ("r01/laser_link", "r01/laser_link/scan"),
    ("r01/odom", "r01/base_footprint"),
    ("map", "r01/odom"),
]


class Probe(Node):
    def __init__(self) -> None:
        super().__init__("scan_tf_probe", parameter_overrides=[
            Parameter("use_sim_time", value=True)])
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.scan = None
        self.scan_stamp = None
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST, depth=5)
        self.create_subscription(LaserScan, "/r01/scan", self._on_scan, qos)

    def _on_scan(self, msg: LaserScan) -> None:
        self.scan = msg.header.frame_id
        self.scan_stamp = msg.header.stamp


def _stamp(t) -> float:
    return t.sec + t.nanosec * 1e-9


def main() -> int:
    rclpy.init()
    node = Probe()
    t0 = time.monotonic()
    while node.scan is None and time.monotonic() - t0 < 40.0:
        rclpy.spin_once(node, timeout_sec=0.2)

    print(f"sim clock now              : {node.get_clock().now().nanoseconds * 1e-9:.3f}")
    print(f"scan frame_id              : {node.scan!r}")
    if node.scan_stamp is not None:
        print(f"scan stamp (sim)           : {_stamp(node.scan_stamp):.3f}")

    # let TF accumulate a little so a zero-time lookup is not just "not yet"
    t1 = time.monotonic()
    while time.monotonic() - t1 < 8.0:
        rclpy.spin_once(node, timeout_sec=0.2)

    print(f"tf buffer frames           : {sorted(node.buffer.all_frames_as_yaml().splitlines())[:2]}")
    try:
        yaml_txt = node.buffer.all_frames_as_yaml()
        frames = [ln.split(":")[0] for ln in yaml_txt.splitlines() if ln and not ln.startswith(" ")]
        print(f"frames in buffer ({len(frames)})      : {sorted(frames)}")
    except Exception as exc:
        print(f"could not enumerate frames : {exc}")

    print()
    print("--- chain lookups (zero time = latest available, then at scan stamp) ---")
    verdict = None
    for parent, child in CHAIN:
        for label, t in (("latest", rclpy.time.Time()), ("scan stamp", node.scan_stamp)):
            if t is None:
                continue
            try:
                tr = node.buffer.lookup_transform(parent, child, t, timeout=rclpy.duration.Duration(seconds=1.0))
                v = tr.transform.translation
                print(f"  OK   {parent:20s} -> {child:22s} [{label:10s}] "
                      f"t=({v.x:+.3f},{v.y:+.3f},{v.z:+.3f})")
                break
            except Exception as exc:
                if label == "scan stamp":
                    print(f"  FAIL {parent:20s} -> {child:22s} [{label:10s}] {type(exc).__name__}: {exc}")
                    if verdict is None:
                        verdict = (parent, child, str(exc))

    print()
    if verdict is None:
        print("VERDICT: the full chain to map resolves. AMCL's input is fine, so the "
              "blocker is inside AMCL's update logic, not TF.")
    else:
        p, c, msg = verdict
        print(f"VERDICT: first broken link is {p} -> {c}")
        print(f"         {msg}")

    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
