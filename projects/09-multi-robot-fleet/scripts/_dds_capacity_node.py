#!/usr/bin/env python3
"""Hold one rclpy node, in its own process, so a caller can measure how many fit.

Diagnostic for a real defect found by the P3 fleet run: with two full Nav2 stacks up,
`ros_gz_sim create` for the second robot died with

    create: Failed to find a free participant index for domain 42
    rmw_cyclonedds_cpp: rmw_create_node: failed to create domain, error Error
    terminate called after throwing an instance of 'rclcpp::exceptions::RCLError'

so the second robot was never spawned, had no odom transform, and its Nav2 bringup was
aborted. The isolation checker then failed to start for the same reason.

Participant count is per PROCESS, not per Node -- rclpy shares one rmw participant across
every Node in a process -- so a ceiling has to be measured by processes, which is what
this program is for: start N of them, count how many reach "UP", and read the error text
from the first one that does not.

Prints UP on success, or FAIL plus the exception, and exits 9 on failure.
"""

from __future__ import annotations

import sys
import time

import rclpy


def main() -> int:
    name = sys.argv[1] if len(sys.argv) > 1 else "dds_capacity"
    hold = float(sys.argv[2]) if len(sys.argv) > 2 else 30.0
    try:
        rclpy.init()
        from rclpy.node import Node
        Node(name)
    except Exception as exc:                      # noqa: BLE001 - report, do not raise
        print(f"FAIL {type(exc).__name__}: {exc}", flush=True)
        return 9
    print("UP", flush=True)
    try:
        time.sleep(hold)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
