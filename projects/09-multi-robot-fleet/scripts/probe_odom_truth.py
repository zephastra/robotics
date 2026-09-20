#!/usr/bin/env python3
"""Isolate one question: does the SIMULATOR'S ODOMETRY drift, or does AMCL's correction?

The crossing probe showed AMCL's belief diverging from the simulator's true pose by up to
3.73 m, and the robot never actually reached the corridor. That leaves the drift in one of
two places, and they need different fixes:

  belief = (map -> odom, published by AMCL)  composed with  (odom -> base, from gz)

    odom -> base drifts   => the PLANT's odometry is lying. This robot is four fixed wheels
                             on a skid-steer base: the wheels scrub sideways in every turn
                             and the DiffDrive plugin integrates joint velocities, so it
                             over-reports turning. Nothing in Nav2 can fix that.
    map -> odom wanders   => AMCL's own correction is moving, i.e. the estimator is
                             reacting to a bad scan match.

Nothing about Nav2 is needed to tell these apart, and running Nav2 at the same time only
adds variables. So this drives a fixed, simple scripted motion straight through the safety
gate and compares, sample by sample:

    what odom says the robot did      vs      what the simulator says it did

Phases: straight, pause, turn in place, pause. Slip shows up in the turn; a bad wheel
radius or scale error shows up in the straight line.

Publishes on cmd_vel_pre_gate (the gate's INPUT), never on cmd_vel, so the gate stays the
only publisher downstream and still applies its own limits. Truth is read for verification
and is never fed to control (AGENTS.md rule 13).
"""

from __future__ import annotations

import argparse
import math
import sys
import time

import rclpy
from geometry_msgs.msg import Twist, TwistStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage


def yaw_of(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class OdomTruth(Node):
    def __init__(self, robot: str, world: str) -> None:
        super().__init__("odom_truth",
                         parameter_overrides=[Parameter("use_sim_time", value=True)])
        self.robot = robot
        self.pub = self.create_publisher(TwistStamped,
                                         f"/{robot}/cmd_vel_pre_gate", 10)
        self.odom: tuple[float, float, float] | None = None   # x, y, yaw
        self.truth: tuple[float, float, float] | None = None  # x, y, yaw
        # What the GATE actually published, and why. Without these two, "the robot did not
        # move" cannot be told apart from "the gate refused to move it" or "the command
        # never arrived", and those are three different problems.
        self.gate_out: tuple[float, float] | None = None
        self.gate_reason = ""
        # Wheel joint ANGLES straight from gz. If odometry reports motion but these do not
        # turn, the wheels are not rotating and the robot is sliding -- a drivetrain fact
        # that no amount of reading odom can establish.
        self.joints: dict[str, float] = {}
        qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, depth=20)
        self.create_subscription(Odometry, f"/{robot}/odom", self._on_odom, qos)
        self.create_subscription(
            TFMessage, f"/world/{world}/dynamic_pose/info", self._on_truth, qos)
        self.create_subscription(Twist, f"/{robot}/cmd_vel", self._on_gate_out, qos)
        self.create_subscription(String, f"/{robot}/gate_state", self._on_gate_state, qos)
        self.create_subscription(JointState, f"/{robot}/joint_states", self._on_joints, qos)

    def _on_joints(self, msg: JointState) -> None:
        for name, pos in zip(msg.name, msg.position):
            self.joints[name] = float(pos)

    def _on_gate_out(self, msg: Twist) -> None:
        self.gate_out = (float(msg.linear.x), float(msg.angular.z))

    def _on_gate_state(self, msg: String) -> None:
        self.gate_reason = msg.data

    def _on_odom(self, msg: Odometry) -> None:
        p = msg.pose.pose.position
        self.odom = (float(p.x), float(p.y), yaw_of(msg.pose.pose.orientation))

    def _on_truth(self, msg: TFMessage) -> None:
        if not msg.transforms:
            return
        tr = msg.transforms[0].transform
        self.truth = (float(tr.translation.x), float(tr.translation.y),
                      yaw_of(tr.rotation))

    def drive(self, lin: float, ang: float, seconds: float,
              rows: list, label: str, out) -> None:
        tick = 1.0 / 20.0
        end = time.monotonic() + seconds
        msg = TwistStamped()
        msg.twist.linear.x = float(lin)
        msg.twist.angular.z = float(ang)
        # Per-phase tally of what the gate actually forwarded. `requested` is what we
        # asked for; if the gate published zero for most ticks, the robot not moving is a
        # GATE result and nothing about odometry or Nav2 is being measured.
        seen: dict[str, int] = {}
        n = 0
        zero_ticks = 0
        j0 = dict(self.joints)
        while time.monotonic() < end:
            self.pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.0)
            n += 1
            if self.gate_out is not None:
                if abs(self.gate_out[0]) < 1e-9 and abs(self.gate_out[1]) < 1e-9:
                    zero_ticks += 1
            if self.gate_reason:
                seen[self.gate_reason] = seen.get(self.gate_reason, 0) + 1
            if self.odom and self.truth:
                rows.append((label, self.odom, self.truth))
            out.write(f"    {label} odom={self._f(self.odom)} "
                      f"truth={self._f(self.truth)} gate={self.gate_out}\n")
            time.sleep(tick)
        pct = 100.0 * zero_ticks / max(1, n)
        print(f"    requested: lin {lin:+.3f} m/s, ang {ang:+.3f} rad/s for "
              f"{seconds:.1f}s")
        print(f"    gate forwarded zero for {zero_ticks}/{n} ticks ({pct:.0f}%)")
        dj = {k: self.joints[k] - j0[k] for k in j0 if k in self.joints}
        if dj:
            # Print the FULL joint name. Truncating to a fixed width collapsed
            # wheel_front_left and wheel_front_right onto one key, which silently hid the
            # left/right split -- and for a skid-steer robot the left/right split is the
            # whole question: an in-place turn needs the two sides EQUAL AND OPPOSITE, and
            # a 3:1 ratio is the signature of a wheel that cannot scrub.
            for name in sorted(dj):
                print(f"    {name:34s} {dj[name]:+.3f} rad")
            left = [v for k, v in dj.items() if k.endswith("_left_joint")]
            right = [v for k, v in dj.items() if k.endswith("_right_joint")]
            mean_ang = sum(abs(v) for v in dj.values()) / len(dj)
            print(f"    left side total  : {sum(left):+.3f} rad")
            print(f"    right side total : {sum(right):+.3f} rad")
            print(f"    mean |angle| {mean_ang:.3f} rad -> implied rolling distance at "
                  f"r=0.075: {mean_ang * 0.075:.4f} m per wheel")
            if abs(ang) > 1e-6 and left and right:
                # A differential turn wants left = -right. |left + right| is therefore the
                # part of the motion that is NOT turning, i.e. how much the robot slid.
                drift = abs(sum(left) + sum(right))
                turn = abs(sum(right) - sum(left))
                print(f"    differential test: |L+R| {drift:.3f} (slide, want ~0) vs "
                      f"|R-L| {turn:.3f} (turn, want large)")
            if mean_ang < 0.05 and abs(lin) > 1e-6 and abs(ang) < 1e-6:
                print("    >> THE WHEELS DID NOT TURN despite a non-zero linear command")
        else:
            print("    no joint_states received -- cannot tell whether the wheels turned")

    @staticmethod
    def _f(p) -> str:
        return "None" if p is None else f"({p[0]:+7.3f},{p[1]:+7.3f},{p[2]:+6.3f})"


def rel(a, b) -> tuple[float, float, float]:
    """b relative to a, in a's own frame: dx, dy (rotated), dyaw."""
    dx, dy = b[0] - a[0], b[1] - a[1]
    c, s = math.cos(-a[2]), math.sin(-a[2])
    return (c * dx - s * dy, s * dx + c * dy, math.atan2(math.sin(b[2] - a[2]),
                                                         math.cos(b[2] - a[2])))


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--robot", default="r01")
    ap.add_argument("--world", default="warehouse")
    ap.add_argument("--straight-speed", type=float, default=0.20)
    ap.add_argument("--straight-s", type=float, default=3.0)
    ap.add_argument("--turn-rate", type=float, default=0.50)
    ap.add_argument("--turn-s", type=float, default=3.0)
    ap.add_argument("--out", default="")
    args, ros_args = ap.parse_known_args(argv)

    out = open(args.out, "w", encoding="utf-8") if args.out else sys.stdout
    rclpy.init(args=ros_args)
    node = OdomTruth(args.robot, args.world)

    t0 = time.monotonic()
    while (node.odom is None or node.truth is None) and time.monotonic() - t0 < 60.0:
        rclpy.spin_once(node, timeout_sec=0.2)
    if node.odom is None or node.truth is None:
        print(f"NOT_RUN: odom={'ok' if node.odom else 'missing'} "
              f"truth={'ok' if node.truth else 'missing'}")
        return 1

    print("=" * 78)
    print("  odometry fidelity: what odom reports vs what the simulator did")
    print("=" * 78)
    print(f"  spawn truth = {node._f(node.truth)}")
    print(f"  spawn odom  = {node._f(node.odom)}   (odom starts at the origin by "
          "definition)")

    rows: list[tuple] = []
    print()
    print("  phase 1: straight, no rotation")
    a_o, a_t = node.odom, node.truth
    node.drive(args.straight_speed, 0.0, args.straight_s, rows, "straight", out)
    b_o, b_t = node.odom, node.truth
    d_o = rel(a_o, b_o)
    d_t = rel(a_t, b_t)
    print(f"    odom  moved: forward {d_o[0]:+.4f} m, lateral {d_o[1]:+.4f} m, "
          f"yaw {math.degrees(d_o[2]):+.2f} deg")
    print(f"    truth moved: forward {d_t[0]:+.4f} m, lateral {d_t[1]:+.4f} m, "
          f"yaw {math.degrees(d_t[2]):+.2f} deg")
    print(f"    >> straight-line odometry error: "
          f"{math.dist((d_o[0], d_o[1]), (d_t[0], d_t[1])):.4f} m, "
          f"yaw error {math.degrees(d_o[2] - d_t[2]):+.2f} deg")

    print()
    print("  phase 2: turn in place")
    a_o, a_t = node.odom, node.truth
    node.drive(0.0, args.turn_rate, args.turn_s, rows, "turn", out)
    node.drive(0.0, 0.0, 2.0, rows, "turn-stop", out)
    b_o, b_t = node.odom, node.truth
    d_o = rel(a_o, b_o)
    d_t = rel(a_t, b_t)
    print(f"    odom  turned {math.degrees(d_o[2]):+8.2f} deg, "
          f"and reported translation ({d_o[0]:+.4f}, {d_o[1]:+.4f}) m")
    print(f"    truth turned {math.degrees(d_t[2]):+8.2f} deg, "
          f"actual translation ({d_t[0]:+.4f}, {d_t[1]:+.4f}) m")
    print(f"    >> turn-rate odometry error: "
          f"{math.degrees(d_o[2] - d_t[2]):+.2f} deg  "
          f"({100.0 * (d_o[2] - d_t[2]) / d_t[2]:+.1f}% of the true turn)"
          if abs(d_t[2]) > 1e-6 else "    >> no turn registered")

    print()
    print("  phase 3: straight again, now on a rotated heading")
    a_o, a_t = node.odom, node.truth
    node.drive(args.straight_speed, 0.0, args.straight_s, rows, "straight2", out)
    b_o, b_t = node.odom, node.truth
    d_o = rel(a_o, b_o)
    d_t = rel(a_t, b_t)
    print(f"    odom  moved: forward {d_o[0]:+.4f} m, lateral {d_o[1]:+.4f} m")
    print(f"    truth moved: forward {d_t[0]:+.4f} m, lateral {d_t[1]:+.4f} m")

    node.drive(0.0, 0.0, 1.5, rows, "stop", out)

    # cumulative drift, expressed as "where odom thinks we are" vs "where we are",
    # both measured from the spawn point in the spawn frame
    if not rows:
        print()
        print("  NOT_RUN: no sample had both odom and truth; nothing to compare.")
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        return 1
    tot_o = rel(rows[0][1], node.odom)
    tot_t = rel(rows[0][2], node.truth)
    print()
    print("=" * 78)
    print("  CUMULATIVE (whole scripted motion)")
    print(f"    odom  believes it travelled to ({tot_o[0]:+.4f}, {tot_o[1]:+.4f}) "
          f"yaw {math.degrees(tot_o[2]):+.2f} deg")
    print(f"    truth says it travelled to      ({tot_t[0]:+.4f}, {tot_t[1]:+.4f}) "
          f"yaw {math.degrees(tot_t[2]):+.2f} deg")
    print(f"    >> ODOMETRY DRIFT: {math.dist((tot_o[0], tot_o[1]), (tot_t[0], tot_t[1])):.4f} m"
          f"  yaw {math.degrees(tot_o[2] - tot_t[2]):+.2f} deg")
    print("=" * 78)
    if math.dist((tot_o[0], tot_o[1]), (tot_t[0], tot_t[1])) > 0.10:
        print("  VERDICT: the plant's odometry drifts on its own, with no Nav2 and no")
        print("           AMCL involved. Localisation cannot be better than its odometry")
        print("           input, so this is the first thing to fix.")
    else:
        print("  VERDICT: odometry is honest for this motion. The crossing drift")
        print("           therefore comes from AMCL's correction, not from the plant.")

    if args.out:
        out.close()
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
