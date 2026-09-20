#!/usr/bin/env python3
"""Why does the estimate lag during motion? Separate a late message from a stale value.

Measured fact: amcl_pose disagreed with the simulator's true pose by up to 3.5 m and the
gap grew monotonically through a 40 s crossing, while agreeing to 0.19-0.33 m whenever the
robot was stationary or settled. That is a defect in the thing the controller steers on,
and it has two opposite explanations that need opposite fixes:

  (a) THE MESSAGE IS LATE. AMCL computes a correct pose, but the message sits in a queue or
      the transport delays it, so the consumer reads history.
  (b) THE VALUE IS OLD. AMCL keeps publishing, on time, a pose it computed seconds ago --
      which happens if it REJECTS the scans in between and simply re-sends its last
      estimate. nav2_amcl does exactly that when it cannot resolve odom -> base_link at the
      scan's timestamp ("Failed to compute odom pose, skipping scan").

The way to tell them apart is the message's own header stamp, compared against the
simulator clock. Note the trap: nav2_amcl republishes with a FRESH stamp when it chooses
not to update, so a small stamp age does not by itself rule out (b). That is why this also
records the odom pose and map->odom separately:

    map_pose = map->odom (x) odom_pose

so a stale map->odom with a fresh odom pose reproduces exactly the lag, and shows up as
map->odom sitting still while odom advances. Truth is the referee for both.

Read-only with respect to control: it subscribes and publishes one straight-line velocity
command, exactly as a driver would. Ground truth is read for VERIFICATION and never fed
back (AGENTS.md rule 13).

Exit: 0 measured, 2 could not measure (missing inputs).
"""

from __future__ import annotations

import argparse
import json
import math
import pathlib
import statistics
import sys
import threading
import time

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, TwistStamped
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from rosgraph_msgs.msg import Clock
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer, TransformListener


def yaw_of(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class PoseLag(Node):
    def __init__(self, robot: str, world: str) -> None:
        super().__init__("pose_lag",
                         parameter_overrides=[Parameter("use_sim_time", value=True)])
        self.robot = robot
        qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, depth=50)

        self.sim_now: float | None = None
        self.amcl: tuple[float, float] | None = None
        self.amcl_stamp: float | None = None
        self.amcl_count = 0
        self.amcl_arrivals: list[float] = []      # wall arrival times
        self.amcl_stamps: list[float] = []        # sim stamps
        self.odom: tuple[float, float, float] | None = None
        self.odom_count = 0
        self.odom_stamps: list[float] = []
        self.truth: tuple[float, float] | None = None
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)

        self.create_subscription(Clock, "/clock", self._on_clock, qos)
        self.create_subscription(
            PoseWithCovarianceStamped, f"/{robot}/amcl_pose", self._on_amcl, qos)
        self.create_subscription(Odometry, f"/{robot}/odom", self._on_odom, qos)
        self.create_subscription(
            TFMessage, f"/world/{world}/dynamic_pose/info", self._on_truth, qos)
        self.pub = self.create_publisher(TwistStamped, f"/{robot}/cmd_vel_pre_gate", 10)

    def _on_clock(self, m: Clock) -> None:
        self.sim_now = m.clock.sec + m.clock.nanosec * 1e-9

    def _on_amcl(self, m: PoseWithCovarianceStamped) -> None:
        p = m.pose.pose.position
        self.amcl = (float(p.x), float(p.y))
        s = m.header.stamp
        self.amcl_stamp = s.sec + s.nanosec * 1e-9
        self.amcl_count += 1
        self.amcl_arrivals.append(time.monotonic())
        self.amcl_stamps.append(self.amcl_stamp)

    def _on_odom(self, m: Odometry) -> None:
        p = m.pose.pose.position
        self.odom = (float(p.x), float(p.y), yaw_of(m.pose.pose.orientation))
        self.odom_count += 1
        s = m.header.stamp
        self.odom_stamps.append(s.sec + s.nanosec * 1e-9)

    def _on_truth(self, m: TFMessage) -> None:
        if m.transforms:
            t = m.transforms[0].transform.translation
            self.truth = (float(t.x), float(t.y))

    def map_to_odom(self) -> tuple[float, float] | None:
        for frame in (f"{self.robot}/odom",):
            try:
                tr = self.buffer.lookup_transform(
                    "map", frame, Time(), timeout=Duration(seconds=0.3))
            except Exception:
                continue
            t = tr.transform.translation
            return (float(t.x), float(t.y))
        return None

    def _row(self, t0: float) -> dict:
        return {
            "t": time.monotonic() - t0,
            "sim": self.sim_now,
            "truth": self.truth,
            "amcl": self.amcl,
            "amcl_stamp": self.amcl_stamp,
            "odom_xy": None if self.odom is None else self.odom[:2],
            "map_odom": self.map_to_odom(),
        }

    def drive(self, speed: float, seconds: float, hz: float,
              rows: list[dict]) -> None:
        """Pace the command WITHOUT touching callback delivery.

        An earlier version of this loop called spin_once once per tick and then slept, so
        the node could handle at most `hz` callbacks per second however many arrived.
        That is the defect that made probe_pipeline_lag.py report a fake transport
        collapse, and this probe had the same one: with the sample loop throttling
        delivery, a sample pairs a stale estimate with a current truth and reports a lag
        that is really the probe's own backlog. The executor now runs on its own thread
        (see main), so this loop only paces and publishes.
        """
        tick = 1.0 / hz
        msg = TwistStamped()
        msg.twist.linear.x = float(speed)
        t0 = time.monotonic()
        while time.monotonic() - t0 < seconds:
            self.pub.publish(msg)
            rows.append(self._row(t0))
            time.sleep(tick)
        # stop and let everything drain
        end = time.monotonic() + 6.0
        while time.monotonic() < end:
            self.pub.publish(TwistStamped())
            rows.append(self._row(t0))
            time.sleep(tick)


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--robot", default="r01")
    ap.add_argument("--world", default="warehouse")
    ap.add_argument("--speed", type=float, default=0.25)
    ap.add_argument("--seconds", type=float, default=14.0)
    ap.add_argument("--hz", type=float, default=5.0)
    ap.add_argument("--out", default="")
    args, ros_args = ap.parse_known_args(argv)

    rclpy.init(args=ros_args)
    n = PoseLag(args.robot, args.world)

    # The executor runs on its own thread so the sampling loop inside drive() cannot
    # throttle message delivery. Before this, the same node reported a lag that was
    # actually its own backlog; see drive().
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(n)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    print("=" * 78)
    print("  does amcl_pose arrive late, or arrive on time carrying an old value?")
    print("=" * 78)

    warm = time.monotonic() + 45.0
    while time.monotonic() < warm and not (
            n.sim_now is not None and n.truth is not None and n.odom is not None
            and n.amcl_count > 0):
        time.sleep(0.2)
    print(f"  warm-up: clock={n.sim_now is not None} truth={n.truth is not None} "
          f"odom={n.odom is not None} amcl_msgs={n.amcl_count}")
    if n.sim_now is None or n.truth is None or n.odom is None:
        print("  cannot measure -- NOT_RUN")
        executor.shutdown()
        n.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        return 2

    a0 = n.amcl_count
    o0 = n.odom_count
    rows: list[dict] = []
    print(f"  driving straight at {args.speed} m/s for {args.seconds:.0f} s, then 6 s "
          "stationary")
    n.drive(args.speed, args.seconds, args.hz, rows)
    elapsed = rows[-1]["t"]

    paired = [r for r in rows if r["truth"] and r["amcl"]]
    print()
    print(f"  samples {len(rows)} over {elapsed:.1f} s   "
          f"amcl_pose messages during the run: {n.amcl_count - a0}")
    if not paired:
        print("  no paired sample -- NOT_RUN")
        executor.shutdown()
        n.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        return 2

    # ---- message age: how old is the stamp relative to the clock -------------- #
    ages = [r["sim"] - r["amcl_stamp"] for r in rows
            if r["sim"] is not None and r["amcl_stamp"] is not None]
    # ---- value error: how wrong is the position ------------------------------- #
    errs = [math.dist(r["truth"], r["amcl"]) for r in paired]
    # ---- map->odom motion vs odom motion ------------------------------------- #
    mo = [r["map_odom"] for r in rows if r["map_odom"]]
    od = [r["odom_xy"] for r in rows if r["odom_xy"]]

    print()
    print(f"  MESSAGE AGE (sim clock minus the message's own stamp):")
    if ages:
        print(f"    min {min(ages):.3f}  mean {sum(ages) / len(ages):.3f}  "
              f"max {max(ages):.3f} s")
    print(f"  VALUE ERROR (amcl_pose position vs simulator truth):")
    print(f"    min {min(errs):.3f}  mean {sum(errs) / len(errs):.3f}  "
          f"max {max(errs):.3f} m")
    if mo:
        print(f"  map->odom moved {math.dist(mo[0], mo[-1]):.3f} m over the run")
    if od:
        print(f"  odom pose  moved {math.dist(od[0], od[-1]):.3f} m over the run")

    if n.amcl_stamps and len(n.amcl_stamps) > 1:
        gaps = [b - a for a, b in zip(n.amcl_stamps, n.amcl_stamps[1:]) if b > a]
        if gaps:
            gaps.sort()
            print(f"  amcl_pose stamp spacing: min {gaps[0]:.3f}  "
                  f"median {gaps[len(gaps) // 2]:.3f}  max {gaps[-1]:.3f} s")
        arr = [b - a for a, b in zip(n.amcl_arrivals, n.amcl_arrivals[1:]) if b > a]
        if arr:
            arr.sort()
            print(f"  amcl_pose arrival spacing (wall): min {arr[0]:.3f}  "
                  f"median {arr[len(arr) // 2]:.3f}  max {arr[-1]:.3f} s")

    # ---- can this probe keep up? ----------------------------------------------- #
    # The failure this guards against: if the sampling loop throttles callback delivery,
    # every sample pairs a stale estimate with a current truth, and the error that gets
    # reported is this probe's backlog rather than AMCL's. That is not hypothetical -- the
    # same defect in probe_pipeline_lag.py produced a complete fake transport collapse.
    # So compare the delivered count against the rate the message stamps imply. They must
    # agree, or the numbers below describe the instrument.
    def rate_check(count: int, stamps: list[float]) -> str:
        if count < 4 or len(stamps) < 4:
            return "too few messages to check (indicative only)"
        gaps = sorted(b - a for a, b in zip(stamps, stamps[1:]) if b > a)
        span = stamps[-1] - stamps[0]
        if span <= 0.0:
            return "no stamp span"
        implied = 1.0 / gaps[len(gaps) // 2]
        actual = (count - 1) / span
        ratio = actual / implied if implied else float("nan")
        verdict = "OK" if 0.75 <= ratio <= 1.30 else "PROBE LOST MESSAGES"
        return (f"stamp {implied:6.2f} Hz   delivered {actual:6.2f} Hz   "
                f"ratio {ratio:4.2f}   {verdict}")

    print()
    print("  INSTRUMENT SELF-CHECK (must say OK, or the error below is this probe's):")
    print(f"    amcl_pose  {rate_check(n.amcl_count - a0, n.amcl_stamps[a0:])}")
    print(f"    odom       {rate_check(n.odom_count - o0, n.odom_stamps[o0:])}")

    print()
    print("  trajectory (every ~2 s):")
    print(f"    {'t':>6} {'truth (x,y)':>20} {'amcl (x,y)':>20} {'err':>7} "
          f"{'stamp age':>10}  {'map->odom (x,y)':>20} {'odom (x,y)':>20}")
    step = max(1, len(rows) // 12)
    for r in rows[::step]:
        if not (r["truth"] and r["amcl"]):
            continue
        age = (r["sim"] - r["amcl_stamp"]) if (r["sim"] is not None
                                              and r["amcl_stamp"] is not None) else None
        t_xy = f"({r['truth'][0]:+8.3f},{r['truth'][1]:+8.3f})"
        a_xy = f"({r['amcl'][0]:+8.3f},{r['amcl'][1]:+8.3f})"
        err = math.dist(r["truth"], r["amcl"])
        age_s = "       n/a" if age is None else f"{age:10.3f}"
        mo = r["map_odom"]
        mo_s = "                 n/a" if not mo else f"({mo[0]:+8.3f},{mo[1]:+8.3f})"
        od = r["odom_xy"]
        od_s = "                 n/a" if not od else f"({od[0]:+8.3f},{od[1]:+8.3f})"
        print(f"    {r['t']:6.1f} {t_xy:>20} {a_xy:>20} {err:7.3f} {age_s}  "
              f"{mo_s:>20} {od_s:>20}")

    # Did map->odom sit still while odom advanced? That is the signature of AMCL not
    # updating: the correction term is frozen, so map_pose = frozen_correction + live_odom
    # reproduces the lag exactly.
    mo_rows = [(r["t"], r["map_odom"], r["odom_xy"]) for r in rows
               if r["map_odom"] and r["odom_xy"]]
    if len(mo_rows) > 10:
        mo_move = math.dist(mo_rows[0][1], mo_rows[-1][1])
        od_move = math.dist(mo_rows[0][2], mo_rows[-1][2])
        print()
        print(f"  map->odom travelled {mo_move:.3f} m while odom travelled {od_move:.3f} m")
        if od_move > 1.0 and mo_move < 0.1:
            print("  => map->odom is NOT tracking the robot's motion: AMCL's correction")
            print("     term is frozen, so the published map pose is `frozen + live odom`.")
            print("     That is the stale-value case, not the late-message case.")
        elif od_move > 1.0 and mo_move > 0.5 * od_move:
            print("  => map->odom is moving with the robot, so AMCL IS correcting during")
            print("     motion. The residual error then needs a different explanation.")

    print()
    print("  VERDICT:")
    max_age = max(ages) if ages else float("nan")
    max_err = max(errs)
    if max_age < 0.3 and max_err > 0.8:
        print("    Messages arrive ON TIME but carry an OLD value. AMCL is not updating")
        print("    its filter during motion -- it republishes its last estimate with a")
        print("    fresh stamp. Look for AMCL skipping scans in its log:")
        print("      grep -iE 'skip|Failed to compute odom|no laser' <robot log>")
    elif max_age >= 0.8:
        print(f"    Messages themselves are up to {max_age:.2f} s old. The lag is in")
        print("    delivery, not in the filter: check the subscriber's queue depth and")
        print("    whether this process spins often enough.")
    else:
        print("    Neither a late message nor a stale value stands out; the residual")
        print(f"    {max_err:.2f} m needs a different explanation.")

    if args.out:
        path = pathlib.Path(args.out)
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(json.dumps({
            "rows": [{k: v for k, v in r.items()} for r in rows],
            "amcl_stamps": n.amcl_stamps,
            "amcl_arrivals": n.amcl_arrivals,
        }, indent=2), encoding="utf-8")
        print(f"  raw rows written to {path}")

    executor.shutdown()
    n.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
