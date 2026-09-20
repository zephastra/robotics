#!/usr/bin/env python3
"""Record the TRUE pose and AMCL's belief, side by side, through one corridor crossing.

Why this and not another look at the log: the crossing fails with the robot's BELIEVED
pose inside the barrier wall, which is physically impossible. That has two opposite
explanations and they need opposite fixes:

  (a) AMCL drifts during the crossing, so the belief wanders into the wall while the robot
      is somewhere else. Fix: localisation -- and this world is a prime suspect, because it
      is MIRROR SYMMETRIC about x = 0, so near the gap two hypotheses explain the same scan.
  (b) AMCL is right and the robot really is against the wall. Fix: control -- the approach
      to a 1.2 m gap is not being tracked.

The action result cannot tell them apart, and neither can any amount of log reading. The
simulator's own model pose can, so this samples both together at 5 Hz for the whole run.

It also records the global path's crossing point, i.e. where the PLAN says the robot should
pass x = 0, because a plan that aims at the gap edge would explain (b) without any
localisation story at all.

AGENTS.md rule 13: truth is read for VERIFICATION here and is never fed to control. This
program subscribes and sends one goal; it publishes no velocity command.

Channel caveat, repeated because it matters: the available truth topic carries no names and
no stamps (see probe_ground_truth.py), so entry 0 of each message is taken as the model.
That is fine for a single robot and is not a general-purpose verification channel.
"""

from __future__ import annotations

import argparse
import math
import pathlib
import sys
import time

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Path
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer, TransformListener

RESULT_NAMES = {
    0: "STATUS_UNKNOWN", 1: "ACCEPTED", 2: "EXECUTING",
    3: "CANCELING", 4: "SUCCEEDED", 5: "CANCELED", 6: "ABORTED",
}

GAP_HALF_WIDTH = 0.6   # the barrier spans |y| > 0.6 on x = 0


class CrossingProbe(Node):
    def __init__(self, robot: str, world: str) -> None:
        super().__init__("crossing_probe",
                         parameter_overrides=[Parameter("use_sim_time", value=True)])
        self.robot = robot
        self.truth: tuple[float, float] | None = None
        # Belief comes from the amcl_pose TOPIC, not from a tf2 buffer.
        #
        # The first version of this probe read map -> base_link through a tf2 Buffer and
        # got None for an entire 76 s crossing -- while the same stack resolved the same
        # chain instantly in the next process. The truth track was in the log the whole
        # time but every sample was thrown away because the pairing demanded both halves,
        # so a working measurement was lost to a flaky reader. Reading the topic AMCL
        # actually publishes removes the reader; the buffer is kept as a fallback.
        self.belief_topic: tuple[float, float] | None = None
        self.belief_count = 0
        # Kept SEPARATE. Truth is the decisive evidence for "did it go through the gap" and
        # must survive even when the estimate is unavailable.
        self.truth_rows: list[tuple[float, float, float]] = []
        self.belief_rows: list[tuple[float, float, float]] = []
        self.plan: list[tuple[float, float]] = []
        self._feedback = None
        # Where to write the raw samples, so the summary can be re-derived by someone who
        # does not trust the summary.
        self.out_path = ""
        # Main-loop sampler accounting, kept separately from the 3 s settle tail so the
        # rate audit measures the loop that was asked to run at a given rate.
        self.main_samples = 0
        self.main_span_s = 0.0

        qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, depth=20)
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.create_subscription(
            TFMessage, f"/world/{world}/dynamic_pose/info", self._on_truth, qos)
        self.create_subscription(
            PoseWithCovarianceStamped, f"/{robot}/amcl_pose", self._on_amcl_pose, qos)
        self.create_subscription(
            Path, f"/{robot}/plan", self._on_plan,
            QoSProfile(reliability=ReliabilityPolicy.RELIABLE, depth=2))
        self.client = ActionClient(self, NavigateToPose, f"/{robot}/navigate_to_pose")

    # ------------------------------------------------------------------ #

    def _on_truth(self, msg: TFMessage) -> None:
        if not msg.transforms:
            return
        t = msg.transforms[0].transform.translation
        self.truth = (float(t.x), float(t.y))

    def _on_amcl_pose(self, msg: PoseWithCovarianceStamped) -> None:
        p = msg.pose.pose.position
        self.belief_topic = (float(p.x), float(p.y))
        self.belief_count += 1

    def _on_plan(self, msg: Path) -> None:
        self.plan = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]

    def belief(self, allow_tf: bool = True) -> tuple[float, float] | None:
        """AMCL's believed pose in map: the topic first, then the tf buffer.

        `allow_tf=False` is what the sampling loop uses. A tf lookup can block for up to
        0.3 s per attempt, four attempts deep, and the previous version called it from
        inside the sampling loop -- which made the loop's cadence unpredictable and left
        its own sample counts impossible to reconcile with the elapsed time. A sampler that
        cannot account for its own samples is not a measurement.

        Note the estimate is published only on FILTER UPDATES, and `update_min_d: 0.25`
        means a stationary robot produces none. So the belief is a snapshot that can be up
        to 0.25 m of travel old, which is why the truth figures are the ones to quote.
        """
        if self.belief_topic is not None:
            return self.belief_topic
        if not allow_tf:
            return None
        for frame in (f"{self.robot}/base_link", f"{self.robot}/base_footprint"):
            for t in (Time(), Time(seconds=0.0)):
                try:
                    tr = self.buffer.lookup_transform(
                        "map", frame, t, timeout=Duration(seconds=0.3))
                except Exception:
                    continue
                p = tr.transform.translation
                return (float(p.x), float(p.y))
        return None

    def plan_crossing(self) -> tuple[float, float] | None:
        """Where the plan crosses x = 0, interpolated. None if it never does."""
        pts = self.plan
        for a, b in zip(pts, pts[1:]):
            if (a[0] <= 0.0 <= b[0]) or (b[0] <= 0.0 <= a[0]):
                if abs(b[0] - a[0]) < 1e-9:
                    return (0.0, a[1])
                f = (0.0 - a[0]) / (b[0] - a[0])
                return (0.0, a[1] + f * (b[1] - a[1]))
        return None

    def on_feedback(self, msg) -> None:
        self._feedback = msg

    # ------------------------------------------------------------------ #

    def run(self, x: float, y: float, timeout_s: float, hz: float) -> int:
        log = self.get_logger()

        warm = time.monotonic() + 70.0
        while time.monotonic() < warm:
            rclpy.spin_once(self, timeout_sec=0.2)
            # Deliberately NOT gated on belief being available. The first version waited
            # for it, so a run whose estimate reader was broken spent the whole warm-up
            # waiting and then reported "no paired samples" -- losing a perfectly good
            # truth track. Truth and the action server are what must be ready; the
            # estimate is reported alongside whatever it manages to do.
            if self.truth is not None and self.client.wait_for_server(timeout_sec=0.0):
                break
        log.info(f"warm-up done: truth={self.truth is not None} "
                 f"belief={self.belief() is not None} "
                 f"amcl_pose_msgs={self.belief_count}")

        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(x)
        goal.pose.pose.position.y = float(y)
        goal.pose.pose.orientation.w = 1.0

        fut = self.client.send_goal_async(goal, feedback_callback=self.on_feedback)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=20.0)
        handle = fut.result()
        if handle is None or not handle.accepted:
            log.error("goal NOT accepted")
            return 2
        log.info(f"goal accepted: ({x:+.2f}, {y:+.2f}) from truth {self.truth}")

        tick = 1.0 / hz
        started = time.monotonic()
        result_fut = handle.get_result_async()
        next_report = 0.0

        while not result_fut.done():
            rclpy.spin_once(self, timeout_sec=0.0)
            now = time.monotonic()
            el = now - started
            # allow_tf=False: never block the sampler on a transform lookup
            b = self.belief(allow_tf=False)
            if self.truth is not None:
                self.truth_rows.append((el, self.truth[0], self.truth[1]))
                self.main_samples += 1
                self.main_span_s = el
            if b is not None:
                self.belief_rows.append((el, b[0], b[1]))
            if el > timeout_s:
                log.error(f"no result after {timeout_s:.0f}s; cancelling "
                          "(the robot may still be moving -- this is not a stop)")
                handle.cancel_goal_async()
                break
            if el >= next_report:
                pc = self.plan_crossing()
                err = math.dist(self.truth, b) if (self.truth and b) else float("nan")
                fb = self._feedback.feedback if self._feedback else None
                rem = getattr(fb, "distance_remaining", None) if fb else None
                log.info(f"  t={el:5.1f}s truth={self._fmt(self.truth)} "
                         f"belief={self._fmt(b)} err={err:.3f} rem={rem} "
                         f"plan@x0={self._fmt(pc)}")
                next_report += 2.0
            time.sleep(tick)

        wrapper = result_fut.result() if result_fut.done() else None
        status = wrapper.status if wrapper is not None else 0
        name = RESULT_NAMES.get(status, f"STATUS_{status}")

        # settle and let the last of the motion land in the samples.
        #
        # Rate limited, and that is not a nicety. `spin_once(timeout_sec=0.1)` returns as
        # soon as ANY callback fires, and the truth topic runs at ~55 Hz, so the loop body
        # ran about a thousand times a second. It appended ~3000 samples in 3 s, which both
        # inflated the sample count to a number that could not be reconciled with the run
        # time and silently over-weighted the final stationary seconds in every average.
        # A busy loop is not a sampler.
        end = time.monotonic() + 3.0
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.02)
            el = time.monotonic() - started
            if self.truth is not None:
                self.truth_rows.append((el, self.truth[0], self.truth[1]))
            b = self.belief(allow_tf=False)
            if b is not None:
                self.belief_rows.append((el, b[0], b[1]))
            time.sleep(tick)

        print()
        print("=" * 78)
        print(f"  crossing result: {name} after {time.monotonic() - started:.1f}s")
        print("=" * 78)
        tr_rows = self.truth_rows
        if not tr_rows:
            print("  no truth samples at all -- NOT_RUN")
            return 1

        print(f"  truth samples : {len(tr_rows)}   belief samples: "
              f"{len(self.belief_rows)}   (amcl_pose messages: {self.belief_count})")
        # Audit the sampler against itself, on the MAIN loop only. An earlier run reported
        # 3588 samples for a 34.6 s window, which cannot be reconciled with the requested
        # rate: the 3 s settle tail was a busy loop appending ~1000 times a second, so the
        # total was inflated and every average was over-weighted towards the final
        # stationary seconds. A sampler that cannot account for its own samples is not a
        # measurement, so the achieved rate is printed next to the requested one.
        achieved = (self.main_samples / self.main_span_s
                    if self.main_span_s > 0 else float("nan"))
        print(f"  sampler        : main loop {achieved:.1f} Hz over "
              f"{self.main_span_s:.1f} s (requested {1.0 / tick:.1f} Hz); "
              f"{len(tr_rows)} truth samples including the settle tail")
        if not (0.3 * (1.0 / tick) <= achieved <= 3.0 * (1.0 / tick)):
            print("  WARNING: achieved rate is far from the requested rate, so the loop was")
            print("           not running on its scheduled cadence and these statistics are")
            print("           NOT trustworthy. Treat the verdicts below as indicative only.")
        if not self.belief_rows:
            print("  NOTE: no estimate samples. The truth track below is still valid and is")
            print("        the decisive evidence for whether the gap was used; only the")
            print("        drift question is unanswerable this run.")

        # --- did the robot ACTUALLY go through the gap? from TRUTH only ---- #
        west = [r for r in tr_rows if r[1] < 0.0]
        east = [r for r in tr_rows if r[1] > 0.0]
        inmouth = [r for r in tr_rows if abs(r[1]) < 0.7]
        worst = min((abs(r[2]) for r in inmouth), default=None)
        print()
        print(f"  TRUTH west of x=0 : {len(west)} samples"
              + (f", min |y| {min(abs(r[2]) for r in west):.3f}" if west else ""))
        print(f"  TRUTH east of x=0 : {len(east)} samples"
              + (f", min |y| {min(abs(r[2]) for r in east):.3f}" if east else ""))
        print(f"  samples inside the gap mouth (|x| < 0.7): {len(inmouth)}"
              + (f", min |y| = {worst:.3f}" if worst is not None else ""))
        print()
        if west and east and worst is not None and worst < GAP_HALF_WIDTH:
            print(f"  => PROVEN from truth: the robot passed x=0 INSIDE the gap mouth, at")
            print(f"     |y| = {worst:.3f} m against a {GAP_HALF_WIDTH} m half width.")
            print(f"     The barrier spans |y| > {GAP_HALF_WIDTH}, and a flood fill of the map")
            print(f"     already proved there is no other route, so this is a real crossing.")
        elif west and east:
            print("  => the robot reached both sides, but no sample caught it inside the gap")
            print("     mouth. With the flood fill that still means it used the gap, but the")
            print("     clearance it actually kept is UNMEASURED -- do not claim one.")
        else:
            print(f"  => the robot's TRUE pose never reached the far side "
                  f"(west {len(west)}, east {len(east)}).")

        # --- plan ---------------------------------------------------------- #
        pc = self.plan_crossing()
        print()
        print(f"  final TRUTH  : {self._fmt(self.truth)}")
        print(f"  final BELIEF : {self._fmt(self.belief())}")
        print(f"  final plan@x=0: {self._fmt(pc)}"
              + ("   <-- plan aims OUTSIDE the gap!" if pc and abs(pc[1]) > GAP_HALF_WIDTH
                 else ""))

        # --- drift --------------------------------------------------------- #
        if self.belief_rows:
            # pair each belief sample with the truth sample nearest in time
            errs = []
            for tb, bx, by in self.belief_rows:
                t0, tx, ty = min(tr_rows, key=lambda r: abs(r[0] - tb))
                errs.append((tb, math.dist((tx, ty), (bx, by))))
            print()
            print(f"  truth-vs-belief error over {len(errs)} paired samples: "
                  f"min {min(e for _, e in errs):.3f}  "
                  f"mean {sum(e for _, e in errs) / len(errs):.3f}  "
                  f"max {max(e for _, e in errs):.3f} m")
            for thresh in (0.10, 0.25, 0.50):
                first = next((e for e in errs if e[1] > thresh), None)
                if first is None:
                    print(f"    never exceeded {thresh:.2f} m")
                else:
                    print(f"    first exceeded {thresh:.2f} m at t={first[0]:.1f}s "
                          f"(error {first[1]:.3f} m)")
            print()
            print("  READING:")
            mx = max(e for _, e in errs)
            if mx < 0.20:
                print("    the estimate tracked truth throughout => localisation is NOT")
                print("    the limiting factor for this crossing.")
            else:
                print(f"    the estimate diverged from truth by up to {mx:.2f} m. Check")
                print("    whether the divergence starts before or after x = 0: a drift that")
                print("    begins at the gap is the mirror-symmetric-world ambiguity, one")
                print("    that begins earlier is odometry.")
        # Raw samples, so the numbers above can be re-derived independently. The
        # world-level truth channel carries no stamp and amcl_pose is emitted only on
        # filter updates, so the pairing is by nearest sample time -- keeping the samples
        # lets a reader redo that pairing instead of trusting this file's summary.
        if self.out_path:
            import csv
            # pathlib.Path, spelled out. `Path` is already taken in this module by
            # nav_msgs.msg.Path, so the bare name constructs a ROS message and raises
            # "Path.__init__() takes 1 positional argument but 2 were given" -- an error
            # that names neither the file being written nor the shadowing import.
            path = pathlib.Path(self.out_path)
            path.parent.mkdir(parents=True, exist_ok=True)
            with path.open("w", newline="", encoding="utf-8") as fh:
                w = csv.writer(fh)
                w.writerow(["kind", "t_s", "x", "y"])
                for t, x, y in tr_rows:
                    w.writerow(["truth", f"{t:.4f}", f"{x:.6f}", f"{y:.6f}"])
                for t, x, y in self.belief_rows:
                    w.writerow(["belief", f"{t:.4f}", f"{x:.6f}", f"{y:.6f}"])
            print(f"  raw samples written to {path} "
                  f"({len(tr_rows)} truth, {len(self.belief_rows)} belief)")
        return 0

    @staticmethod
    def _fmt(p: tuple[float, float] | None) -> str:
        return "None" if p is None else f"({p[0]:+7.3f},{p[1]:+7.3f})"


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--robot", default="r01")
    ap.add_argument("--world", default="warehouse")
    ap.add_argument("--x", type=float, default=4.0)
    ap.add_argument("--y", type=float, default=-2.0)
    ap.add_argument("--timeout", type=float, default=240.0)
    ap.add_argument("--hz", type=float, default=5.0)
    ap.add_argument("--out", default="",
                    help="write raw truth/belief samples here as CSV")
    args, ros_args = ap.parse_known_args(argv)

    rclpy.init(args=ros_args)
    node = CrossingProbe(args.robot, args.world)
    node.out_path = args.out
    code = 1
    try:
        code = node.run(args.x, args.y, args.timeout, args.hz)
    except KeyboardInterrupt:
        code = 3
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return code


if __name__ == "__main__":
    sys.exit(main())
