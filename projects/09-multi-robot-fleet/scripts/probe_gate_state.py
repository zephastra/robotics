#!/usr/bin/env python3
"""Read-only probe: what does the safety gate SAY while the robot is not moving?

WHY THIS EXISTS
---------------
`reports/batch_20260917T071455Z/N01` and `_20260917T073309Z/N01` both leave r01 motionless for
the rest of the run -- truth and raw odometry frozen together, controller aborting with
`error_code 105 'Failed to make progress'` 32 times, and even `backup` (0.15 m, open loop)
timing out. That is a statement about the base, and there are two ways to get it:

  (a) the gate withheld the command -- it is the SOLE publisher of `<ns>/cmd_vel`, and
      `gate_node.py` has three paths that end in a zeroed command, none of which logs anything
      after the first one; or
  (b) the gate passed the command through and the base did not apply it.

The gate publishes its full verdict at 20 Hz on `<ns>/fleet/gate_state` -- `mode`, `reason`,
`detail`, `commanded_linear_mps`, `measured_speed_mps`, `zero_published_at`,
`permit_messages_seen`, `permit_grants_seen`. Nothing records it. This probe records it.

A disclaimer that is also a design constraint: this subscribes, prints and writes. It publishes
nothing, calls no service, and would be the same instrument if the fleet were replaced by a
recording. Its own failure mode is that it saw nothing, which is why it prints the message count
on exit rather than only the transitions.

Usage (through the wrapper, which is the only supported way -- it sets the ROS environment and
picks /usr/bin/python3):

    scripts/probe_gate_state.sh --robots r01 --out <dir> [--duration 0]

THE SECOND MEASUREMENT (added after the latch was found)
-------------------------------------------------------
The gate latched `STOP`/`PERMIT_EXPIRED` for 815 s and refused every command. Its own pose at
that moment was 1.526 m from the simulator's truth -- but that number came from aligning two
separate processes' clocks by hand, and the question it does NOT answer is which of two very
different defects produced it:

  * a CONSTANT offset -- the odom frame's origin, the spawn it is composed with, or a frame
    convention; the fix is at the frame, and it is one number; or
  * ACCUMULATED drift -- the base's odometry over-reports (four fixed wheels scrub in every
    turn), so the error grows with distance travelled; the fix is at the plant, and no amount
    of frame accounting touches it.

Those are told apart by plotting the error against distance travelled, which needs the gate's
pose, the raw odometry, AMCL's belief and the simulator's truth in ONE process so the samples
are simultaneous rather than aligned afterwards. So this probe now also records:

    <ns>/odom                                the plant's raw odometry (origin = spawn)
    <ns>/amcl_pose                           what Nav2 steers on
    /world/<world>/dynamic_pose/info         Gazebo's own poses -- the arbiter

and writes `pose_track.jsonl` beside its log. Truth is read for verification and is never fed
to control (rule 13); this probe still publishes nothing.

Exit codes: 0 fine, 3 the interpreter cannot import rclpy (wrong python), 4 no gate_state topic
ever seen, 5 the gate was seen but the pose comparison could not be made (no truth or no odom).
"""
from __future__ import annotations

import argparse
import json
import math
import pathlib
import signal
import sys
import time

# The P5.6 preflight. A probe that reports "the system is silent" because it was launched with an
# interpreter that has no rclpy is an instrument lying about the system, and this project has paid
# for that mistake once already (5 NOT_RUN verdicts from one ModuleNotFoundError).
try:
    import rclpy  # noqa: F401
except Exception as exc:  # pragma: no cover - depends on the environment, not the code
    print(f"PREFLIGHT FAILED: cannot import rclpy ({type(exc).__name__}: {exc}).\n"
          "This script must run with /usr/bin/python3, through scripts/probe_gate_state.sh.",
          file=sys.stderr)
    raise SystemExit(3)

from geometry_msgs.msg import PoseWithCovarianceStamped  # noqa: E402
from nav_msgs.msg import Odometry  # noqa: E402
from rclpy.executors import ExternalShutdownException  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.qos import (QoSHistoryPolicy, QoSProfile,  # noqa: E402
                       QoSReliabilityPolicy, QoSDurabilityPolicy)
from std_msgs.msg import String  # noqa: E402
from tf2_msgs.msg import TFMessage  # noqa: E402

#: The fields whose CHANGE is the event. `measured_speed_mps` is not one of them: it changes
#: continuously and would flood the log, which is how a log stops being read.
KEY_FIELDS = ("mode", "reason", "detail", "commanded_linear_mps",
              "commanded_angular_radps", "zero_published_at", "solo_publisher_violation")


def _pct(values: list[float], q: float) -> float | None:
    """A percentile without a statistics import, and None for no data.

    None rather than 0.0: "the channel never delivered" and "it delivered instantly" are not the
    same fact, and a default of zero would read as the second.
    """
    if not values:
        return None
    ordered = sorted(values)
    index = min(len(ordered) - 1, max(0, int(round(q / 100.0 * (len(ordered) - 1)))))
    return round(ordered[index], 3)


def qos() -> QoSProfile:
    """Match the gate's publisher, which is RELIABLE/VOLATILE/KEEP_LAST(10)."""
    return QoSProfile(
        reliability=QoSReliabilityPolicy.RELIABLE,
        durability=QoSDurabilityPolicy.VOLATILE,
        history=QoSHistoryPolicy.KEEP_LAST,
        depth=50,
    )


class GateStateProbe(Node):
    def __init__(self, robots: list[str], out: pathlib.Path, heartbeat_s: float,
                 world: str = "warehouse", track_s: float = 2.0) -> None:
        super().__init__("gate_state_probe")
        self.robots = robots
        self.out = out
        self.heartbeat_s = heartbeat_s
        self.seen = {r: 0 for r in robots}
        self.last_key: dict[str, str] = {}
        self.last_beat = time.monotonic()
        self.started = time.monotonic()

        # ---- the pose comparison ---------------------------------------------------- #
        self.world = world
        self.track_s = track_s
        self.last_track = time.monotonic()
        self.track_path = out.parent / "pose_track.jsonl"
        # A stale file would make one artefact hold two runs and turn every "over time"
        # reading into a comparison of two different experiments.
        try:
            self.track_path.unlink()
        except FileNotFoundError:
            pass
        self.last_body: dict[str, dict] = {}
        self.last_pose: dict[str, tuple[float, float]] = {}
        self.spawn: dict[str, tuple[float, float, float]] = {}
        self.odom: dict[str, tuple[float, float]] = {}
        self.odom_path: dict[str, float] = {}
        self.odom_msgs: dict[str, int] = {}
        self.amcl: dict[str, tuple[float, float]] = {}
        self.amcl_msgs: dict[str, int] = {}
        # Arrival times, not just counts. `amcl_messages: 115` over 947 s says the localiser
        # runs at ~0.12 Hz, and a rate alone does not say whether that is a steady 8 s cadence
        # or bursts with long silences -- which is the difference between a usable gate input
        # and one that must never be one.
        self.odom_last_wall: dict[str, float] = {}
        self.amcl_last_wall: dict[str, float] = {}
        self.odom_gaps: dict[str, list[float]] = {}
        self.amcl_gaps: dict[str, list[float]] = {}
        # Truth arrives as a list of unnamed poses: the bridge drops the model names, so the
        # slot ORDER carries the identity. The whole dict is kept rather than index 0, because
        # choosing an index by convention is a choice, and the two-robot case already showed
        # that this convention is not the identity.
        self.truth: dict[int, tuple[float, float]] = {}
        self.truth_path: dict[int, float] = {}
        self.truth_msgs = 0
        self.truth_slots = 0
        for rid in robots:
            # `<ns>/gate_state`, not `<ns>/fleet/gate_state`: the second was the name this
            # probe first used, taken from the adapter, and the adapter was wrong. The probe
            # reported `n=0` for nine minutes, which was true and useless -- so it now also
            # counts publishers, and a silent topic and a wrong topic stop looking the same.
            self.create_subscription(
                String, f"/{rid}/gate_state",
                lambda msg, r=rid: self._on_state(r, msg), qos())
            self.create_subscription(
                Odometry, f"/{rid}/odom",
                lambda msg, r=rid: self._on_odom(r, msg), qos())
            self.create_subscription(
                PoseWithCovarianceStamped, f"/{rid}/amcl_pose",
                lambda msg, r=rid: self._on_amcl(r, msg), qos())
        # gz's own poses, the channel fleet_recorder treats as the arbiter. Bridged by
        # fleet_bringup/launch/world.launch.py; bridged nowhere else on purpose.
        self.create_subscription(
            TFMessage, f"/world/{world}/dynamic_pose/info", self._on_truth, qos())
        self.get_logger().info(
            f"subscribed: {[f'/{r}/gate_state' for r in robots]}; "
            f"heartbeat every {heartbeat_s:.0f}s; writing {out}")

    def _emit(self, line: str) -> None:
        print(line, flush=True)

    def _on_state(self, rid: str, msg: String) -> None:
        self.seen[rid] += 1
        try:
            body = json.loads(msg.data)
        except Exception:
            self._emit(f"  t={time.monotonic() - self.started:8.1f} {rid} UNPARSEABLE: "
                       f"{msg.data[:160]}")
            return
        # Refreshed on every message. `pose` is deliberately NOT a change key (it moves
        # continuously, so making it one would flood the log), which means a copy taken only
        # when something ELSE changed would be a stale pose -- the exact fault this probe
        # exists to detect, reproduced inside the detector.
        self.last_body[rid] = body
        pose = body.get("pose")
        if isinstance(pose, list) and len(pose) >= 2:
            self.last_pose[rid] = (float(pose[0]), float(pose[1]))
        spawn = body.get("spawn")
        if isinstance(spawn, list) and len(spawn) >= 3:
            self.spawn[rid] = (float(spawn[0]), float(spawn[1]), float(spawn[2]))
        key = json.dumps({k: body.get(k) for k in KEY_FIELDS}, sort_keys=True)
        if key == self.last_key.get(rid):
            return
        first = rid not in self.last_key
        self.last_key[rid] = key
        wall = body.get("wall_s")
        self._emit(
            f"{'FIRST ' if first else 'CHANGE'} t={time.monotonic() - self.started:8.1f} "
            f"wall={wall} {rid} mode={body.get('mode')} reason={body.get('reason')} "
            f"cmd=({body.get('commanded_linear_mps')}, {body.get('commanded_angular_radps')}) "
            f"measured={body.get('measured_speed_mps')} "
            f"pose={body.get('pose')}/{body.get('pose_age_s')}s "
            f"zero_at={body.get('zero_published_at')} "
            f"permits={body.get('permit_messages_seen')}/"
            f"{body.get('permit_grants_seen')}/adopted={body.get('permit_adopted')} "
            # NOT truncated. The first version cut this to 80 characters and the cut removed
            # "unpermitted [...], holdings [...]" -- the two lists that name WHY the gate
            # refused. The probe had recorded the answer and thrown it away, which is the
            # same fault as a subscriber on the wrong topic: the instrument decided what
            # could be known. `detail` is the gate's own sentence and it is short.
            f"| {body.get('detail')}"
            + self._companion(rid))
        self._write_track(rid, body, "state")

    # ---- the pose comparison, in one process ----------------------------------------- #
    # Round 9's number (1.526 m) was correct and still not enough: it had to reconstruct WHEN
    # the latch happened by aligning two processes' clocks, and its first attempt got both the
    # alignment and the parse wrong. Everything below is simultaneous by construction.
    def _on_odom(self, rid: str, msg) -> None:
        self.odom_msgs[rid] = self.odom_msgs.get(rid, 0) + 1
        now = time.monotonic()
        last = self.odom_last_wall.get(rid)
        if last is not None:
            self.odom_gaps.setdefault(rid, []).append(now - last)
        self.odom_last_wall[rid] = now
        p = msg.pose.pose.position
        xy = (float(p.x), float(p.y))
        last = self.odom.get(rid)
        if last is not None:
            # Distance TRAVELLED, not distance from the origin: accumulated drift scales with
            # the former, and a straight line to the origin cannot tell the two apart.
            self.odom_path[rid] = self.odom_path.get(rid, 0.0) + math.hypot(
                xy[0] - last[0], xy[1] - last[1])
        self.odom[rid] = xy

    def _on_amcl(self, rid: str, msg) -> None:
        self.amcl_msgs[rid] = self.amcl_msgs.get(rid, 0) + 1
        now = time.monotonic()
        last = self.amcl_last_wall.get(rid)
        if last is not None:
            self.amcl_gaps.setdefault(rid, []).append(now - last)
        self.amcl_last_wall[rid] = now
        p = msg.pose.pose.position
        self.amcl[rid] = (float(p.x), float(p.y))

    def _on_truth(self, msg) -> None:
        self.truth_msgs += 1
        slots: dict[int, tuple[float, float]] = {}
        for slot, tr in enumerate(msg.transforms):
            t = tr.transform.translation
            xy = (float(t.x), float(t.y))
            prev = self.truth.get(slot)
            if prev is not None:
                self.truth_path[slot] = self.truth_path.get(slot, 0.0) + math.hypot(
                    xy[0] - prev[0], xy[1] - prev[1])
            slots[slot] = xy
        self.truth = slots
        self.truth_slots = len(slots)

    def _truth_robot(self) -> tuple[int, tuple[float, float]] | None:
        """Which slot is the robot? The one that has MOVED.

        Convention says index 0. Convention is a hypothesis: in the two-robot world the truth
        stream carried one robot and one stationary model, and an index-based reader called the
        stationary one a robot. Movement is the property being measured, so it is also the
        property used to identify the thing being measured.
        """
        if not self.truth:
            return None
        moved = max(self.truth_path.items(), key=lambda kv: kv[1]) if self.truth_path else None
        if moved is not None and moved[1] > 0.5 and moved[0] in self.truth:
            return moved[0], self.truth[moved[0]]
        first = min(self.truth)
        return first, self.truth[first]

    def _composition_error(self, rid: str) -> float | None:
        """||gate_pose - (spawn (+) odom)||: zero by construction, or the patch is wrong.

        The gate translates odom into the map frame by composing it with the spawn pose, on the
        stated ground that `map -> odom` is nearly constant in this stack. If this number is
        not ~0, then the 1.5 m is a frame bookkeeping bug and nothing else needs explaining.
        """
        gate, odom, spawn = self.last_pose.get(rid), self.odom.get(rid), self.spawn.get(rid)
        if gate is None or odom is None or spawn is None:
            return None
        c, s = math.cos(spawn[2]), math.sin(spawn[2])
        ex = spawn[0] + odom[0] * c - odom[1] * s
        ey = spawn[1] + odom[0] * s + odom[1] * c
        return math.hypot(gate[0] - ex, gate[1] - ey)

    def _companion(self, rid: str) -> str:
        bits = []
        odom = self.odom.get(rid)
        if odom is not None:
            bits.append(f"odom=({odom[0]:.3f}, {odom[1]:.3f})")
        comp = self._composition_error(rid)
        if comp is not None:
            bits.append(f"composition_error={comp:.4f}")
        tr = self._truth_robot()
        if tr is not None:
            slot, xy = tr
            bits.append(f"truth[slot {slot}/{self.truth_slots}]=({xy[0]:.3f}, {xy[1]:.3f})")
            gate = self.last_pose.get(rid)
            if gate is not None:
                bits.append(f"|gate-truth|={math.hypot(gate[0] - xy[0], gate[1] - xy[1]):.3f}")
            amcl = self.amcl.get(rid)
            if amcl is not None:
                bits.append(f"|amcl-truth|={math.hypot(amcl[0] - xy[0], amcl[1] - xy[1]):.3f}")
        elif self.truth_msgs == 0:
            bits.append("NO TRUTH MESSAGE YET")
        bits.append(f"path={self.odom_path.get(rid, 0.0):.2f}m")
        return " " + " ".join(bits)

    def _write_track(self, rid: str, body: dict, why: str) -> None:
        tr = self._truth_robot()
        row: dict = {
            "why": why,
            "t_s": round(time.monotonic() - self.started, 3),
            "robot": rid,
            "mode": body.get("mode"),
            "reason": body.get("reason"),
            "detail": body.get("detail"),
            "gate_pose": body.get("pose"),
            # The pose the gate DECIDED from, which is not `gate_pose` above: that one is
            # `self._pose` (spawn (+) odom), published for reference beside the chosen one.
            # The gate has published all of these since D-P10-02; this probe dropped them
            # and the analysis then read "the gate is still concluding from odometry"
            # while the gate's own start-up line said `pose source localiser`. An
            # instrument that carries one of two facts makes the missing one look absent.
            "pose_source": body.get("pose_source"),
            "pose_usable": body.get("pose_usable"),
            # The refresh half. The gate publishes these three; a whitelist that carried
            # only `pose_usable` is why "did the loop get broken by asking, and how often"
            # was unanswerable in the run of 2026-09-17T18:15:39Z -- the payload had the
            # answer and the instrument dropped it.
            "nomotion_service": body.get("nomotion_service"),
            "nomotion_requests": body.get("nomotion_requests"),
            "nomotion_need": body.get("nomotion_need"),
            "pose_trusted": body.get("pose_trusted"),
            "localiser": body.get("localiser"),
            "localiser_age_s": body.get("localiser_age_s"),
            "localiser_moved_m": body.get("localiser_moved_m"),
            "localiser_messages": body.get("localiser_messages"),
            "localiser_verdict": body.get("localiser_verdict"),
            "pose_disagreement_m": body.get("pose_disagreement_m"),
            "spawn": body.get("spawn"),
            "odom": self.odom.get(rid),
            "amcl": self.amcl.get(rid),
            # The ages are the point: `|amcl - truth|` means nothing without them, because a
            # stale pose of a stationary robot is exact and a fresh pose of a moving one is not.
            "amcl_age_s": (None if rid not in self.amcl_last_wall
                           else round(time.monotonic() - self.amcl_last_wall[rid], 3)),
            "odom_age_s": (None if rid not in self.odom_last_wall
                           else round(time.monotonic() - self.odom_last_wall[rid], 3)),
            "amcl_count": self.amcl_msgs.get(rid, 0),
            "odom_count": self.odom_msgs.get(rid, 0),
            # The gate's own view of whether it is moving, so the analysis can split by phase
            # without inferring motion from a position difference.
            "cmd_linear": body.get("commanded_linear_mps"),
            "measured_speed": body.get("measured_speed_mps"),
            "truth_slots": self.truth_slots,
            "truth_robot_slot": tr[0] if tr else None,
            "truth_all": {str(k): list(v) for k, v in sorted(self.truth.items())},
            "truth_path": {str(k): round(v, 4) for k, v in sorted(self.truth_path.items())},
            "odom_path_m": round(self.odom_path.get(rid, 0.0), 4),
            "composition_error": (round(self._composition_error(rid), 4)
                                  if self._composition_error(rid) is not None else None),
        }
        gate = row["gate_pose"]
        row["err_gate_truth"] = (round(math.hypot(gate[0] - tr[1][0], gate[1] - tr[1][1]), 4)
                                 if (tr and gate) else None)
        amcl = row["amcl"]
        row["err_amcl_truth"] = (round(math.hypot(amcl[0] - tr[1][0], amcl[1] - tr[1][1]), 4)
                                 if (tr and amcl) else None)
        # The number the whole question turns on: how far the pose the gate acted on was
        # from the simulator. `err_gate_truth` measures the reported composed pose and
        # cannot answer it.
        trusted = row["pose_trusted"]
        row["err_trusted_truth"] = (
            round(math.hypot(trusted[0] - tr[1][0], trusted[1] - tr[1][1]), 4)
            if (tr and trusted) else None)
        try:
            with self.track_path.open("a", encoding="utf-8") as fh:
                fh.write(json.dumps(row, sort_keys=True) + "\n")
        except Exception as exc:  # pragma: no cover - a write failure must be visible
            self._emit(f"  ! track write failed: {type(exc).__name__}: {exc}")

    def beat(self) -> None:
        now = time.monotonic()
        if now - self.last_beat < self.heartbeat_s:
            return
        self.last_beat = now
        parts = []
        for rid in self.robots:
            key = self.last_key.get(rid)
            body = json.loads(key) if key else {}
            # The publisher count is the difference between "the gate is silent" and "I am
            # listening to the wrong topic", and those two produced the same n=0 for nine
            # minutes in this probe's first run. An instrument that cannot tell them apart
            # is reporting about itself.
            pub = self.count_publishers(f"/{rid}/gate_state")
            parts.append(f"{rid}: n={self.seen[rid]} pubs={pub} mode={body.get('mode')} "
                         f"cmd={body.get('commanded_linear_mps')} reason={body.get('reason')}")
        self._emit(f"  . t={now - self.started:8.1f}  " + " | ".join(parts))
        # The gate's own lines only fire on a CHANGE, and during a stall nothing changes --
        # which is precisely the window that has to be sampled. This tick is what makes the
        # error-over-distance curve exist at all.
        if now - self.last_track >= self.track_s:
            self.last_track = now
            for rid in self.robots:
                self._write_track(rid, self.last_body.get(rid, {}), "tick")
        for rid in self.robots:
            # Only while this robot's topic has NEVER delivered. The warning exists to
            # separate "the gate is silent" from "I subscribed to the wrong name", and once
            # messages have arrived the second is disproved -- after which the only thing a
            # zero publisher count means is that the fleet has been torn down. It fired at the
            # end of every run, which is how an alarm trains its reader to ignore it.
            if self.seen[rid] == 0 and self.count_publishers(f"/{rid}/gate_state") == 0:
                self._emit(f"  ! {rid}: NO PUBLISHER on /{rid}/gate_state and no state has "
                           "arrived -- this run says nothing about the gate, only about the "
                           "topic name")


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--robots", nargs="+", default=["r01"])
    ap.add_argument("--out", default=".")
    ap.add_argument("--duration", type=float, default=0.0,
                    help="seconds; 0 means until told to stop")
    ap.add_argument("--heartbeat-s", type=float, default=5.0)
    ap.add_argument("--world", default="warehouse",
                    help="gz world name; the truth topic is /world/<name>/dynamic_pose/info")
    ap.add_argument("--track-s", type=float, default=2.0,
                    help="seconds between pose-comparison rows in pose_track.jsonl")
    args = ap.parse_args(argv)

    out_dir = pathlib.Path(args.out)
    out_dir.mkdir(parents=True, exist_ok=True)
    log_path = out_dir / "gate_state_probe.txt"

    stop = {"flag": False, "count": 0}

    def _handler(signum, _frame):
        # Flush during a signal is what cost a whole recorder run: a second signal during the
        # flush killed the write. Count the signals and let the main loop exit; ignore repeats.
        stop["count"] += 1
        if stop["count"] > 1:
            return
        stop["flag"] = True

    # `rclpy.init()` installs its own SIGINT handler, so the handlers are installed AFTER it:
    # that is version-independent, and the alternative -- asking init not to via an option member
    # -- cost this probe a run, because `SignalHandlerOptions.NONE` does not exist in this
    # build and the name is only ever read at run time. (See docs/DECISIONS.md D-P7-01, which is
    # about exactly this fault; the author of that entry then committed it here.)
    #
    # The window between init and the two lines below is one instant in which rclpy's handler is
    # in charge. Nothing is subscribed yet, so the worst case is a clean rclpy shutdown.
    rclpy.init()
    signal.signal(signal.SIGINT, _handler)
    signal.signal(signal.SIGTERM, _handler)

    node = GateStateProbe(args.robots, log_path, args.heartbeat_s,
                         world=args.world, track_s=args.track_s)
    started = time.monotonic()
    try:
        while rclpy.ok() and not stop["flag"]:
            try:
                rclpy.spin_once(node, timeout_sec=0.2)
            except ExternalShutdownException:
                # Something outside this process closed the context. That is a fact about the
                # run, not a crash: stop, and let the finally-block write down what was seen --
                # including, if it comes to that, that nothing was.
                break
            node.beat()
            if args.duration and (time.monotonic() - started) >= args.duration:
                break
    finally:
        summary = {
            "gate_state_messages": dict(node.seen),
            "distinct_states": {r: (json.loads(k) if k else None)
                                for r, k in node.last_key.items()},
            "seconds": round(time.monotonic() - started, 1),
            "stopped_by_signal": stop["count"],
            # An instrument that reports only the channel that worked is reporting about
            # itself. These three are the ones that can be silently absent.
            "truth_messages": node.truth_msgs,
            "truth_slots_last": node.truth_slots,
            "odom_messages": dict(node.odom_msgs),
            "amcl_messages": dict(node.amcl_msgs),
            "amcl_gap_p50_s": {r: _pct(node.amcl_gaps.get(r, []), 50)
                               for r in node.amcl_gaps},
            "amcl_gap_p95_s": {r: _pct(node.amcl_gaps.get(r, []), 95)
                               for r in node.amcl_gaps},
            "amcl_max_gap_s": {r: (round(max(node.amcl_gaps[r]), 3) if node.amcl_gaps.get(r)
                                   else None) for r in node.amcl_gaps},
            "odom_gap_p95_s": {r: _pct(node.odom_gaps.get(r, []), 95)
                               for r in node.odom_gaps},
            "track_file": str(node.track_path),
        }
        with log_path.open("a", encoding="utf-8") as fh:
            fh.write("\n" + json.dumps(summary, indent=2, sort_keys=True) + "\n")
            fh.flush()
        node.destroy_node()
        rclpy.shutdown()
        print(json.dumps(summary, indent=2, sort_keys=True), flush=True)

    if not any(node.seen.values()):
        print("NO GATE STATE SEEN -- the probe saw nothing, which says nothing about the gate.",
              file=sys.stderr)
        return 4
    # A HALF-WORKING instrument is more dangerous than a silent one: the gate half answers, the
    # pose comparison silently yields None, and an analysis that averages Nones reports a clean
    # run. Exit 5 says which half was missing.
    absent = []
    if node.truth_msgs == 0:
        absent.append(f"/world/{args.world}/dynamic_pose/info")
    for rid in args.robots:
        if not node.odom_msgs.get(rid):
            absent.append(f"/{rid}/odom")
    if absent:
        print(f"PARTIAL INSTRUMENT: no messages on {absent}. The gate half is recorded; the "
              "pose comparison is not, so this run cannot answer the pose question.",
              file=sys.stderr)
        return 5
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
