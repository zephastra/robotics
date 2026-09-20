#!/usr/bin/env python3
"""Measure the EFFECTIVE rolling radius, so the 2.7% odometry scale question is data.

The wheel collision cylinder is 0.077 m while the visual wheel -- and the radius the
DiffDrive plugin integrates -- is 0.075 m. That 2 mm was added deliberately, because with
the collision exactly tangent to the floor no contact was generated at all and the robot
had zero traction. The cost is a possible 2.7% scale error: if the wheel truly rolls on a
0.077 m circle then a commanded 0.20 m/s travels 0.2053 m/s, and odometry (which
multiplies joint angle by 0.075) under-reports by the same 2.7%.

Whether that is real is a question about one number:

    effective radius = (distance the robot ACTUALLY travelled) / (wheel angle turned)

so measure exactly that, three ways, in one straight run:
  * commanded distance   -- what was asked for
  * odom distance        -- what the plugin believes, using r = 0.075
  * truth distance       -- what the simulator did, from the model pose

If effective_r lands on 0.075 there is nothing to compensate and the 2 mm is free.
If it lands on 0.077 the scale error is real and the fix belongs in the SDF (set
<wheel_radius> to match the contact radius, or remove the 2 mm and fix the contact
instead), not in a fudge factor downstream.

AGENTS.md rule 13: truth is read here for MEASUREMENT. It is never fed to control; this
program only publishes a straight-line velocity command, exactly as a driver would.
"""

from __future__ import annotations

import argparse
import math
import sys
import time

import rclpy
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from tf2_msgs.msg import TFMessage

VISUAL_R = 0.075
COLLISION_R = 0.077


def yaw_of(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class ScaleProbe(Node):
    def __init__(self, robot: str, world: str) -> None:
        super().__init__("scale_probe",
                         parameter_overrides=[Parameter("use_sim_time", value=True)])
        self.robot = robot
        self.odom: tuple[float, float] | None = None
        self.truth: tuple[float, float] | None = None
        self.joints: dict[str, float] = {}
        qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, depth=20)
        self.create_subscription(Odometry, f"/{robot}/odom", self._on_odom, qos)
        self.create_subscription(
            TFMessage, f"/world/{world}/dynamic_pose/info", self._on_truth, qos)
        self.create_subscription(JointState, f"/{robot}/joint_states", self._on_joints, qos)
        # straight into the gate, so this measures the same path a driver uses
        self.pub = self.create_publisher(TwistStamped, f"/{robot}/cmd_vel_pre_gate", 10)

    def _on_odom(self, m: Odometry) -> None:
        p = m.pose.pose.position
        self.odom = (float(p.x), float(p.y))

    def _on_truth(self, m: TFMessage) -> None:
        if m.transforms:
            t = m.transforms[0].transform.translation
            self.truth = (float(t.x), float(t.y))

    def _on_joints(self, m: JointState) -> None:
        for n, p in zip(m.name, m.position):
            self.joints[n] = float(p)

    def spin_for(self, seconds: float, cmd: float | None) -> None:
        msg = TwistStamped()
        if cmd:
            msg.twist.linear.x = cmd
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            if cmd is not None:
                self.pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.0)
            time.sleep(1.0 / 20.0)


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--robot", default="r01")
    ap.add_argument("--world", default="warehouse")
    ap.add_argument("--speed", type=float, default=0.20)
    ap.add_argument("--seconds", type=float, default=4.0)
    args, ros_args = ap.parse_known_args(argv)

    rclpy.init(args=ros_args)
    n = ScaleProbe(args.robot, args.world)

    warm = time.monotonic() + 60.0
    while time.monotonic() < warm:
        rclpy.spin_once(n, timeout_sec=0.2)
        if n.odom and n.truth and n.joints:
            break
    print("=" * 74)
    print("  009 effective rolling radius -- is the 2.7% scale error real?")
    print("=" * 74)
    print(f"  warm-up: odom={n.odom is not None} truth={n.truth is not None} "
          f"joints={bool(n.joints)}")
    if not (n.odom and n.truth and n.joints):
        print("  cannot measure -- NOT_RUN")
        n.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        return 2

    print(f"  joints visible: {sorted(n.joints)}")
    print()
    print(f"  driving straight at {args.speed:.2f} m/s for {args.seconds:.1f} s, "
          "through the gate")

    n.spin_for(1.0, None)                       # settle, nothing commanded
    o0 = tuple(n.odom)
    t0 = tuple(n.truth)
    j0 = dict(n.joints)
    wall0 = time.monotonic()
    n.spin_for(args.seconds, args.speed)
    wall_cmd = time.monotonic() - wall0
    n.spin_for(0.5, None)                       # let it coast to a halt
    o1 = tuple(n.odom)
    t1 = tuple(n.truth)
    j1 = dict(n.joints)

    d_cmd = args.speed * args.seconds
    d_odom = math.hypot(o1[0] - o0[0], o1[1] - o0[1])
    d_truth = math.hypot(t1[0] - t0[0], t1[1] - t0[1])

    # AUDIT THE INSTRUMENT BEFORE READING IT.
    #
    # The first version of this probe reported "commanded distance 1.2000 m" for a 6 s run
    # at 0.20 m/s and then compared odometry (1.0059 m) against it. But the odometry figure
    # implies only 5.03 s of motion and the truth figure 5.74 s, so the window was not the
    # 6 s the arithmetic assumed: the ramp, the settle and the publish loop all eat into it.
    # Every percentage this probe prints is relative to `d_cmd`, so an unverified window
    # makes every percentage wrong. The effective radius ratio (truth / wheel angle) is the
    # only window-invariant quantity here, and even that is confounded with wheel slip.
    print()
    print(f"  window audit: {wall_cmd:.2f} s of commanded motion "
          f"(asked for {args.seconds:.2f} s)")
    print(f"    odom implies  {d_odom / args.speed:.2f} s at the commanded speed")
    print(f"    truth implies {d_truth / args.speed:.2f} s at the commanded speed")
    # The start-up transient has to be a small part of the window or the comparison is
    # mostly a measurement of the ramp. At 0.5 m/s^2 reaching 0.2 m/s takes 0.4 s.
    ramp_s = args.speed / 0.5
    if ramp_s / wall_cmd > 0.05:
        print(f"    WARNING: the {ramp_s:.2f} s ramp is {100 * ramp_s / wall_cmd:.0f}% of "
              "the window, so these percentages measure the ramp more than the plant."
              " Use --seconds >= 20 for a scale figure.")
        print("             Treat every number below as INDICATIVE ONLY.")
    if abs(d_odom / args.speed - wall_cmd) > 0.15 * wall_cmd:
        print("    WARNING: odometry's implied duration is far from the commanded duration,")
        print("             so the commanded distance is NOT the right denominator. The")
        print("             percentages below are NOT trustworthy.")
    angles = [j1[k] - j0[k] for k in j0 if k in j1]
    mean_abs = sum(abs(a) for a in angles) / len(angles) if angles else 0.0

    print()
    print(f"  {'joint':34s} {'angle (rad)':>12s}")
    for k in sorted(j0):
        if k in j1:
            print(f"  {k:34s} {j1[k] - j0[k]:+12.3f}")
    print()
    print(f"  mean |wheel angle|      : {mean_abs:.4f} rad")
    print(f"  commanded distance      : {d_cmd:.4f} m")
    print(f"  odom distance           : {d_odom:.4f} m   "
          f"({d_odom / d_cmd * 100 - 100:+.1f}% vs commanded)")
    print(f"  truth distance          : {d_truth:.4f} m   "
          f"({d_truth / d_cmd * 100 - 100:+.1f}% vs commanded)")
    print()
    if mean_abs > 1e-6:
        eff = d_truth / mean_abs
        print(f"  effective radius        : {eff:.5f} m   "
              f"(truth distance / mean |angle|)")
        print(f"    visual / plugin radius: {VISUAL_R:.3f} m")
        print(f"    collision radius      : {COLLISION_R:.3f} m")
        print(f"    delta vs plugin radius: {(eff - VISUAL_R) * 1000:+.2f} mm "
              f"({(eff / VISUAL_R - 1) * 100:+.2f}%)")
        print()
        if abs(eff - VISUAL_R) < 0.0015:
            print("  VERDICT: the wheel rolls on ~0.075 m. The 2 mm collision padding does")
            print("           NOT produce a scale error worth compensating -- the contact")
            print("           compresses to the visual radius. Nothing to fix; record that")
            print("           the concern was checked and is closed.")
        else:
            print(f"  VERDICT: the wheel rolls on ~{eff:.4f} m, not 0.075. The scale error")
            print("           IS real. Fix it in the SDF (set the DiffDrive <wheel_radius>")
            print(f"           to {eff:.4f}), not with a downstream fudge factor.")
    print()
    print(f"  odom vs truth scale     : {d_odom / d_truth * 100 - 100:+.2f}% "
          "(what a controller would be fooled by)")
    n.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
