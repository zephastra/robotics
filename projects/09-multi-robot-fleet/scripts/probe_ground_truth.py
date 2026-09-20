#!/usr/bin/env python3
"""Coarse ground-truth cross-check: robot's TRUE model pose vs AMCL's belief.

Read this for what it is. The available channel is /world/<world>/dynamic_pose/info,
bridged as gz.msgs.Pose_V -> tf2_msgs/msg/TFMessage. That mapping DROPS the pose name and
leaves the stamp at zero, so what arrives is a list of unnamed, unstamped transforms and
the model has to be identified by array position (entry 0 is the model, verified by its
value matching the spawn pose).

Consequences, stated rather than hidden:
  * this works for ONE robot and is not robust once several models share the world
  * there is no stamp, so truth and estimate are sampled together in time rather than
    aligned by timestamp
  * it is therefore a COARSE cross-check, not the name-bearing verification channel the
    plan requires. A name-bearing channel needs a small gz-transport node or the
    ros_gz_interfaces route, and is recorded as open work.

Why it is worth having anyway: the corridor failures left the robot's believed pose
inside a barrier wall, which is physically impossible. That has two opposite explanations
-- AMCL is wrong, or the map is wrong -- and only the simulator's own pose separates them.

AGENTS.md rule 13 is respected: read for VERIFICATION, never fed to control.
"""

from __future__ import annotations

import argparse
import math
import sys
import time

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer, TransformListener


class TruthProbe(Node):
    def __init__(self, robot: str, world: str) -> None:
        super().__init__("truth_probe",
                         parameter_overrides=[Parameter("use_sim_time", value=True)])
        self.robot = robot
        self.truth: tuple[float, float] | None = None
        self.truth_count = 0
        self.xform_count = 0
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, depth=20)
        self.create_subscription(
            TFMessage, f"/world/{world}/dynamic_pose/info", self._on_truth, qos)

    def _on_truth(self, msg: TFMessage) -> None:
        self.truth_count += 1
        self.xform_count = len(msg.transforms)
        if not msg.transforms:
            return
        # Entry 0 is the model itself; the rest are its links. See the module docstring.
        t = msg.transforms[0].transform.translation
        self.truth = (float(t.x), float(t.y))

    def belief(self) -> tuple[float, float] | None:
        try:
            tr = self.buffer.lookup_transform(
                "map", f"{self.robot}/base_link", Time(),
                timeout=Duration(seconds=0.5))
        except Exception:
            return None
        t = tr.transform.translation
        return (float(t.x), float(t.y))


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--robot", default="r01")
    ap.add_argument("--world", default="warehouse")
    ap.add_argument("--seconds", type=float, default=15.0)
    args, ros_args = ap.parse_known_args(argv)

    print("=" * 78)
    print("  COARSE ground truth vs AMCL belief   (verification only, never control)")
    print("=" * 78)

    rclpy.init(args=ros_args)
    node = TruthProbe(args.robot, args.world)

    # Warm up before sampling. The transform tree is assembled asynchronously and
    # map -> base_link needs AMCL's map -> odom, which only starts once AMCL has its
    # initial pose. Sampling immediately returns "not yet" and looks like a broken
    # localiser, which is exactly the misreading this whole channel exists to prevent.
    warm = time.monotonic() + 60.0
    while time.monotonic() < warm:
        rclpy.spin_once(node, timeout_sec=0.2)
        if node.truth is not None and node.belief() is not None:
            break
    print(f"  warm-up: truth={'yes' if node.truth is not None else 'no'} "
          f"belief={'yes' if node.belief() is not None else 'no'}")

    end = time.monotonic() + args.seconds
    rows: list[tuple[tuple[float, float] | None, tuple[float, float] | None]] = []
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.2)
        rows.append((node.truth, node.belief()))

    print(f"  truth messages received: {node.truth_count} "
          f"({node.xform_count} transforms each)")
    paired = [(t, b) for t, b in rows if t and b]
    print(f"  samples with both truth and belief: {len(paired)}")
    if not paired:
        print()
        print("  VERDICT: could not pair truth with belief.")
        if node.truth_count == 0:
            print("           No truth messages at all. The bridge entry for")
            print("           /world/<world>/dynamic_pose/info is missing or the world")
            print("           name does not match.")
        else:
            print("           Truth arrived but map -> base_link never resolved.")
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        return 1

    errs = [math.dist(t, b) for t, b in paired]
    print()
    print(f"  {'truth (x, y)':>30s}   {'AMCL belief (x, y)':>30s}   error")
    for t, b in paired[:: max(1, len(paired) // 8)]:
        print(f"  ({t[0]:+9.4f}, {t[1]:+9.4f})   ({b[0]:+9.4f}, {b[1]:+9.4f})"
              f"   {math.dist(t, b):.4f} m")
    print()
    err = sum(errs) / len(errs)
    print(f"  error: min {min(errs):.4f}  mean {err:.4f}  max {max(errs):.4f} m")
    print()
    print("  CAVEAT: the channel carries no names and no stamps (see the module")
    print("          docstring), so this is a coarse single-robot cross-check.")
    if max(errs) < 0.15:
        print("  VERDICT: AMCL agrees with the simulator to within 0.15 m. Localisation")
        print("           is NOT the problem; a believed pose inside a wall would mean")
        print("           the MAP disagrees with the world.")
    else:
        print(f"  VERDICT: AMCL disagrees with the simulator by up to {max(errs):.2f} m.")
        print("           Localisation error is real and is a candidate cause for the")
        print("           corridor failures.")
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
