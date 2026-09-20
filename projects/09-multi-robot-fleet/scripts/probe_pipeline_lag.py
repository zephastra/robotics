"""How stale is each input when it arrives?  READ-ONLY measurement.

Why this exists, and what it already got wrong once.

The controller steers on a pose measured to lag the simulator, so the question is which
input is stale and why. This probe reports, per topic on the localisation path:

  arrival age -- sim_now - stamp, taken in the callback
  sampled age -- sim_now - stamp of the latest message held, on a 10 Hz timer
  rate        -- arrivals per second of SIM time
  growth      -- median age over the last third of a phase minus the first third

THE BUG THIS FILE WAS REWRITTEN TO REMOVE. The first version paced its own loop and
called spin_once once per iteration with `time.sleep(1/20)` in the driving phase. That
caps the node's callback throughput at 20/s, while scan (10 Hz) + odom (20 Hz) + tf
(20 Hz) deliver about 50/s. The probe could not drain its own queue, so:

  * delivered rates fell to 3.8 Hz in the driving phase and then OVERSHOT their
    configured rate once driving stopped (scan 16.9 > 10, odom 26.8 > 20),
  * ages grew to 4.8 s and kept growing,
  * and every topic collapsed TOGETHER, which is the tell: independent producers do not
    fail in lockstep, and the delivery stalls were exactly the phases that used the
    sleeping loop.

gz itself was measured healthy throughout that run -- real_time_factor 1.000 and 1000
physics steps per sim second, stationary and driving alike, read from its own
/world/<world>/stats with no ROS in the path. The scan stamps were spaced exactly
0.1000 s, i.e. the lidar WAS publishing at 10 Hz while the probe reported 3.8 Hz
delivered. Instrument, not transport.

So the executor now runs on its own thread and the pacing loop never touches it. The
report ends with an INSTRUMENT SELF-CHECK that compares the delivered rate against the
rate implied by the message stamps: if those disagree, the number is about the probe and
the verdict must be ignored. That check is the point of this file, not an extra.

The only thing published is the drive command, on the gate's INPUT, so motion still goes
through the safety gate. No ground-truth value is read here and none is fed anywhere.
"""

from __future__ import annotations

import argparse
import statistics
import sys
import threading
import time

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, TwistStamped
from nav_msgs.msg import Odometry
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState, LaserScan
from tf2_msgs.msg import TFMessage


def stamp_s(obj) -> float | None:
    """header.stamp as float seconds, or None if the message has no header."""
    h = getattr(obj, "header", None)
    if h is None:
        return None
    return float(h.stamp.sec) + float(h.stamp.nanosec) * 1e-9


class PipelineLag(Node):
    def __init__(self, robot: str) -> None:
        super().__init__("pipeline_lag",
                         parameter_overrides=[Parameter("use_sim_time", value=True)])
        self.robot = robot
        qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, depth=100)

        self.count: dict[str, int] = {}
        self.last_stamp: dict[str, float] = {}
        self.samples: list[tuple[float, str, float]] = []      # sim_now, topic, age
        self.arrivals: list[tuple[float, str]] = []            # sim_now, topic
        self.stamps: dict[str, list[float]] = {}               # topic -> stamps
        self.marks: list[tuple[str, float, float]] = []        # label, wall, sim
        self.unstamped: set[str] = set()

        self.create_subscription(LaserScan, f"/{robot}/scan",
                                 lambda m: self._note("scan", stamp_s(m)), qos)
        self.create_subscription(Odometry, f"/{robot}/odom",
                                 lambda m: self._note("odom", stamp_s(m)), qos)
        self.create_subscription(JointState, f"/{robot}/joint_states",
                                 lambda m: self._note("joint_states", stamp_s(m)), qos)
        self.create_subscription(TFMessage, f"/{robot}/tf", self._on_tf, qos)
        # Present only when nav2 is up; absence is tolerated and reported.
        self.create_subscription(PoseWithCovarianceStamped, f"/{robot}/amcl_pose",
                                 lambda m: self._note("amcl_pose", stamp_s(m)), qos)

        self.pub = self.create_publisher(TwistStamped,
                                         f"/{robot}/cmd_vel_pre_gate", 10)
        self.create_timer(0.1, self._sample)

    # ------------------------------------------------------------------ #

    def sim(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _note(self, topic: str, stamp: float | None) -> None:
        now = self.sim()
        self.count[topic] = self.count.get(topic, 0) + 1
        self.arrivals.append((now, topic))
        if stamp is None or stamp == 0.0:
            # A zero stamp means the producer did not fill the header in. That is not
            # "no lag", it is "unmeasurable", and it must not be averaged in as 0 lag.
            self.unstamped.add(topic)
            return
        self.last_stamp[topic] = stamp
        self.samples.append((now, topic, now - stamp))
        self.stamps.setdefault(topic, []).append(stamp)

    def _on_tf(self, msg: TFMessage) -> None:
        """Record the odom -> base transform specifically.

        tf carries several transforms; the one that matters to AMCL is the odom ->
        base_link pair the DiffDrive plugin publishes. Match by suffix so a model
        instance prefix does not break it. If names are empty (the Pose_V bridge can
        drop them) fall back to transform[0] and say so.
        """
        named = False
        for tr in msg.transforms:
            child = (tr.child_frame_id or "").strip("/")
            if child.endswith("base_footprint") or child.endswith("base_link"):
                self._note("tf(odom->base)", stamp_s(tr))
                named = True
                break
        if not named and msg.transforms:
            self._note("tf[0]?unnamed", stamp_s(msg.transforms[0]))

    def _sample(self) -> None:
        now = self.sim()
        for topic, st in self.last_stamp.items():
            self.samples.append((now, topic, now - st))

    # ------------------------------------------------------------------ #

    def begin(self, label: str) -> None:
        self.marks.append((label, time.monotonic(), self.sim()))

    def pace(self, seconds: float, publish: TwistStamped | None = None) -> None:
        """Pace for `seconds` of wall time without touching the executor.

        Callbacks are drained by the executor thread, so this loop's cadence cannot
        throttle message delivery. That is the whole reason the executor moved off this
        thread: when it did not, this loop's 20 Hz tick capped the node at 20 callbacks
        per second and produced a fake transport collapse.
        """
        tick = 1.0 / 20.0
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            if publish is not None:
                self.pub.publish(publish)
            time.sleep(tick)

    # ------------------------------------------------------------------ #

    def window(self, label: str) -> tuple[float, float] | None:
        for i, (lab, _, t0) in enumerate(self.marks):
            if lab != label or i + 1 >= len(self.marks):
                continue
            return t0, self.marks[i + 1][2]
        return None

    def wall_window(self, label: str) -> tuple[float, float] | None:
        for i, (lab, w0, _) in enumerate(self.marks):
            if lab != label or i + 1 >= len(self.marks):
                continue
            return w0, self.marks[i + 1][1]
        return None

    def stats(self, label: str) -> dict[str, dict[str, float]]:
        win = self.window(label)
        out: dict[str, dict[str, float]] = {}
        if win is None:
            return out
        t0, t1 = win
        span = t1 - t0
        if span <= 0.2:
            return out
        for topic in sorted(set(self.count) | set(self.last_stamp)):
            ages = [(t, a) for t, tp, a in self.samples if tp == topic and t0 <= t < t1]
            if not ages:
                continue
            vals = sorted(a for _, a in ages)
            msgs = sum(1 for t, tp in self.arrivals if tp == topic and t0 <= t < t1)
            third = max(1, len(ages) // 3)
            out[topic] = {
                "msgs": msgs,
                "rate": msgs / span,
                "min": vals[0],
                "median": statistics.median(vals),
                "max": vals[-1],
                "growth": (statistics.median(a for _, a in ages[-third:])
                           - statistics.median(a for _, a in ages[:third])),
            }
        return out

    def instrument_table(self) -> tuple[list[str], bool]:
        """Delivered rate vs the rate the stamps imply, per phase and topic.

        If a producer emits at 10 Hz and the node only records 3.8 Hz, the node lost
        messages or stalled -- the number describes the probe, not the system. This is
        the check that would have caught the first version of this file.
        """
        rows: list[str] = []
        ok = True
        for label, _, _ in self.marks[:-1]:
            win = self.window(label)
            if win is None:
                continue
            t0, t1 = win
            span = t1 - t0
            if span <= 0.2:
                continue
            for topic in sorted(self.stamps):
                ss = sorted(x for x in self.stamps[topic] if t0 <= x < t1)
                if len(ss) < 4:
                    continue
                gaps = sorted(b - a for a, b in zip(ss, ss[1:]) if b > a)
                produced = 1.0 / gaps[len(gaps) // 2]
                msgs = sum(1 for t, tp in self.arrivals if tp == topic and t0 <= t < t1)
                ratio = msgs / span / produced if produced else float("nan")
                flag = "" if 0.75 <= ratio <= 1.30 else "   <-- NODE LOST MESSAGES"
                if flag:
                    ok = False
                rows.append(f"    {label:<8}{topic:<16}stamp {produced:7.2f} Hz   "
                            f"delivered {msgs / span:7.2f} Hz   ratio {ratio:5.2f}{flag}")
        return rows, ok

    def report(self) -> None:
        print()
        print("=" * 78)
        print("  input staleness   age = sim clock at delivery - the message's own stamp")
        print("=" * 78)
        per_phase: dict[str, dict[str, dict[str, float]]] = {}
        for label, _, _ in self.marks[:-1]:
            st = self.stats(label)
            per_phase[label] = st
            sw = self.window(label)
            ww = self.wall_window(label)
            print()
            if sw is None:
                continue
            sim_span = sw[1] - sw[0]
            wall_span = (ww[1] - ww[0]) if ww else float("nan")
            rtf = sim_span / wall_span if wall_span else float("nan")
            print(f"  --- phase {label!r}: sim {sim_span:.2f} s, wall {wall_span:.2f} s, "
                  f"sim/wall {rtf:.3f} ---")
            if not st:
                print("      sim clock barely advanced, or no data. NOT_RUN.")
                continue
            print(f"    {'topic':<18}{'msgs':>6}{'rate':>9}{'min':>9}{'median':>9}"
                  f"{'max':>9}{'growth':>9}")
            for topic, s in sorted(st.items(), key=lambda kv: -kv[1]["median"]):
                print(f"    {topic:<18}{s['msgs']:>6.0f}{s['rate']:>8.1f}H"
                      f"{s['min']:>9.3f}{s['median']:>9.3f}{s['max']:>9.3f}"
                      f"{s['growth']:>+9.3f}")

        if self.unstamped:
            print()
            print(f"  topics arriving with a ZERO stamp (unmeasurable, not zero lag): "
                  f"{sorted(self.unstamped)}")

        print()
        print("  --- producer cadence from the stamps (not from arrival counts) ---")
        for topic in sorted(self.stamps):
            ss = sorted(self.stamps[topic])
            gaps = sorted(b - a for a, b in zip(ss, ss[1:]) if b > a)
            if not gaps:
                continue
            print(f"    {topic:<18} n={len(ss):<6} median gap "
                  f"{gaps[len(gaps) // 2]:.4f} s   => "
                  f"{1.0 / gaps[len(gaps) // 2]:.1f} Hz")

        rows, instrument_ok = self.instrument_table()
        print()
        print("  --- INSTRUMENT SELF-CHECK: can this probe keep up? ---")
        for r in rows:
            print(r)
        if not rows:
            print("    not enough stamped samples to check. Treat numbers as indicative.")
        elif instrument_ok:
            print("    every topic: delivered rate matches the rate its stamps imply,")
            print("    so the ages below describe the SYSTEM, not this probe.")
        else:
            print("    MISMATCH: this probe lost messages. The ages above describe the")
            print("    PROBE. Fix the instrument before drawing any conclusion.")

        # ---- verdict ---------------------------------------------------- #
        print()
        print("  --- verdict ---")
        if not instrument_ok:
            print("    WITHHELD: the instrument did not keep up, so its rate numbers are")
            print("    not about the system. Re-run before interpreting anything.")
            return
        drive = per_phase.get("drive") or {}
        static = per_phase.get("static") or {}
        if not drive:
            print("    no driving phase data: NOT_RUN")
            return
        worst = max(drive.items(), key=lambda kv: kv[1]["median"])
        print(f"    worst topic while driving: {worst[0]}, median age "
              f"{worst[1]['median']:.3f} s, growth {worst[1]['growth']:+.3f} s")
        big = {t: s for t, s in drive.items() if s["median"] > 0.5}
        if not big:
            print("    every input is fresh (<0.5 s) while driving.")
        elif len(big) >= 3:
            print(f"    {len(big)} of {len(drive)} topics stale by >0.5 s at once: "
                  f"{sorted(big)}")
            print("    => a shared cause, not one slow sensor.")
        else:
            print(f"    only {sorted(big)} stale => that producer is the lag.")
        growing = {t: s["growth"] for t, s in drive.items() if s["growth"] > 0.3}
        if growing:
            print(f"    age GROWS during motion on {sorted(growing)}")
        elif big:
            print("    age is flat during motion => fixed latency, not a growing backlog.")
        for topic in ("scan", "odom"):
            d, s = drive.get(topic), static.get(topic)
            if d and s:
                print(f"    {topic}: static median {s['median']:.3f} s -> driving median "
                      f"{d['median']:.3f} s")


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--robot", default="r01")
    ap.add_argument("--static", type=float, default=8.0)
    ap.add_argument("--drive", type=float, default=16.0)
    ap.add_argument("--after", type=float, default=8.0)
    ap.add_argument("--speed", type=float, default=0.25)
    args, ros_args = ap.parse_known_args(argv)

    rclpy.init(args=ros_args)
    node = PipelineLag(args.robot)

    # The executor gets its own thread. See the module docstring: when this probe paced
    # its own callbacks, its 20 Hz loop throttled delivery and looked like a transport
    # collapse. Nothing below may block callback delivery.
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    print("=" * 78)
    print("  input staleness against the sim clock  (read-only)")
    print("=" * 78)
    deadline = time.monotonic() + 90.0
    while time.monotonic() < deadline and not (
            "scan" in node.count and "odom" in node.count):
        time.sleep(0.2)
    print(f"  topics seen during warm-up: {sorted(node.count)}")
    if "scan" not in node.count or "odom" not in node.count:
        print("  scan and/or odom never arrived. NOT_RUN -- start the stack first.")
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        return 1

    node.begin("static")
    node.pace(args.static)
    node.begin("drive")
    print(f"  driving straight at {args.speed:.2f} m/s for {args.drive:.0f} s")
    drive_msg = TwistStamped()
    drive_msg.twist.linear.x = float(args.speed)
    node.pace(args.drive, publish=drive_msg)
    node.begin("after")
    node.pace(args.after)
    node.begin("end")

    node.report()
    executor.shutdown()
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
