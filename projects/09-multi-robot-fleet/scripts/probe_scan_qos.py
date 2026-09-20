#!/usr/bin/env python3
"""Decisive QoS probe for the 009 AMCL blocker. READ-ONLY.

AMCL subscribes to the laser with `SensorDataQoS()` -- in this rclpy that is
RELIABLE / VOLATILE / KEEP_LAST(5). If the ros_gz bridge publishes with a QoS
that cannot match it, the subscription is created but silently receives nothing,
and AMCL then simply never localises. Nothing in the log says "I got no scans".

So: subscribe the SAME way AMCL does and report whether anything arrives. If
this gets messages, the QoS hypothesis is DEAD and the problem is inside AMCL's
frame handling instead. If it gets nothing while `ros2 topic echo` (which uses
its own default QoS) does get messages, the QoS hypothesis is CONFIRMED.

This script subscribes only. It publishes nothing, writes nothing, and exits.
"""

from __future__ import annotations

import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from sensor_msgs.msg import LaserScan


class ScanProbe(Node):
    def __init__(self) -> None:
        super().__init__("scan_qos_probe")
        # Exactly what nav2_amcl uses.
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )
        # The bridge's likely QoS, for comparison.
        best_effort_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )
        self.reliable_count = 0
        self.best_effort_count = 0
        self.first_frame = None

        self.create_subscription(
            LaserScan, "/r01/scan", self._on_reliable, sensor_qos)
        self.create_subscription(
            LaserScan, "/r01/scan", self._on_best_effort, best_effort_qos)

    def _on_reliable(self, msg: LaserScan) -> None:
        self.reliable_count += 1
        if self.first_frame is None:
            self.first_frame = msg.header.frame_id

    def _on_best_effort(self, msg: LaserScan) -> None:
        self.best_effort_count += 1


def main() -> int:
    rclpy.init()
    node = ScanProbe()
    try:
        import time
        deadline = time.monotonic() + 20.0
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.2)
    finally:
        print(f"RELIABLE subscriber  (what AMCL uses) : {node.reliable_count} msgs")
        print(f"BEST_EFFORT subscriber (bridge guess) : {node.best_effort_count} msgs")
        if node.first_frame is not None:
            print(f"frame_id seen: {node.first_frame!r}")
        if node.reliable_count == 0 and node.best_effort_count > 0:
            print("VERDICT: QoS MISMATCH CONFIRMED -- reliable subscription gets "
                  "nothing while best-effort gets data. AMCL is deaf by QoS.")
        elif node.reliable_count > 0:
            print("VERDICT: QoS is FINE -- AMCL receives scans. The failure is "
                  "inside AMCL's frame/transform handling, not the transport.")
        elif node.reliable_count == 0 and node.best_effort_count == 0:
            print("VERDICT: NO SCAN DATA AT ALL on /r01/scan -- the bridge or the "
                  "sensor is the problem, not QoS.")
        node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
