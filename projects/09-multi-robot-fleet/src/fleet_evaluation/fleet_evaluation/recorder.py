"""Recorder: sample a live run into a file the evaluator can judge later.

This node is a **witness, not a participant**. It publishes nothing and subscribes only.
That is a structural requirement, not tidiness: a recorder that published would be part of
the system it is recording, and its numbers would then be about itself.

WHAT IT RECORDS, AND WHY EACH PART IS SEPARATED
-----------------------------------------------
* `truth`     -- Gazebo's own `dynamic_pose` stream. The arbiter. Its slots carry no names
                 (measured: 2816 of 2816 `child_frame_id` values empty), so they are
                 identified later by geometry. Yaw IS recorded, unlike the P4 sampler, so a
                 future judgement can use the real oriented footprint instead of a
                 conservative circle.
* `odom`      -- each robot's estimate, in the odom frame (origin at the spawn pose).
                 This is the CLAIM. It is stored separately so that "what the robot thinks"
                 and "what is true" can never be silently averaged together.
* `claim`     -- the task service's own status snapshot: task states, battery, refusals.
                 A claim. The evaluator cross-checks it; it is never evidence.
* `resources` -- the reservation book as the COORDINATOR reports it, which is a different
                 service from the one behind `claim`. The judge decides "unauthorised entry"
                 by asking whether truth puts a robot inside a region its own book called
                 unowned, and it reads that from `sample["resources"]`. The task service's
                 snapshot has no reservation book in it at all, so a recorder that polls only
                 `/fleet/tasks` makes that property NOT_RUN for every recording it ever
                 produces -- silent, and indistinguishable from a clean run.
* `gate`      -- the safety gate's OWN pose payload, verbatim, per robot. This is the pose the
                 gate ACTED ON, and it is a different fact from `odom`: the gate may be
                 concluding from the localiser (`pose_source: localiser`) or from composed
                 odometry, and it may be concluding from a pose it considers UNUSABLE. Reading
                 `odom` and calling it "the gate's pose" is how the safety margin came to rest
                 on a quantity nobody had measured. Recorded with its own arrival time, because
                 the question "was that pose fresh when the verdict was taken" needs the age
                 the gate saw, not the age this recorder saw.

Run it beside a live fleet:

    ros2 run fleet_evaluation fleet_recorder --robots r01,r02,r03 \
        --out reports/whatever --label run --duration 300
"""

from __future__ import annotations

import argparse
import json
import math
import pathlib
import sys
import threading
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_msgs.msg import TFMessage

#: One model in the world stream occupies this many links. Verified rather than assumed:
#: the recorder reports the observed widths, and the evaluator refuses to attribute slots
#: when the count does not match the number of robots.
#: Transforms per model in this world, used for nothing but "how many models were in the
#: message" (``width // LINKS_PER_MODEL``). It is NOT an address: a model's frame is not at
#: ``index * LINKS_PER_MODEL``. See `_on_truth`.
LINKS_PER_MODEL = 8

DEFAULT_TRUTH_TOPIC = "/world/warehouse/dynamic_pose/info"

#: Depth for the truth subscription. See the module docstring's diagnosis and
#: `docs/DECISIONS.md` D-P17-08. The default profile (`depth=10`) is sized for a
#: control topic at control rate; `dynamic_pose/info` carries every link of every
#: model at physics rate (the world steps at 1 ms), so with three robots the queue is
#: overrun between two 5 Hz samples and all but the last few poses are discarded --
#: which reads downstream as a robot frozen at its spawn. Measured: 1-2 robots give
#: 0.3-0.5 samples per truth message with the default, 3 robots give 7.2-152.3. Raised
#: far above any plausible per-sample arrival count so the witness cannot be the thing
#: that drops evidence.
TRUTH_QOS_DEPTH = 2000


def truth_qos(depth: int = TRUTH_QOS_DEPTH) -> QoSProfile:
    """The profile the truth subscription uses. RELIABLE + deep KEEP_LAST.

    RELIABLE rather than BEST_EFFORT on purpose: the bridge publishes reliably, and a
    witness should not introduce a second, independent dropping policy. The history
    depth is what was wrong, not the reliability.
    """
    return QoSProfile(
        reliability=QoSReliabilityPolicy.RELIABLE,
        durability=QoSDurabilityPolicy.VOLATILE,
        history=QoSHistoryPolicy.KEEP_LAST,
        depth=depth,
    )





def yaw_from_quaternion(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class Recorder(Node):
    def __init__(self, robots: list[str], truth_topic: str, out_dir: pathlib.Path,
                 label: str, sample_hz: float,
                 gate_state_topics: dict[str, str] | None = None) -> None:
        super().__init__("fleet_recorder")
        self.robots = robots
        self.out_dir = out_dir
        self.label = label
        self.out_dir.mkdir(parents=True, exist_ok=True)

        self.odom: dict[str, tuple[float, float, float]] = {}
        self.truth: dict[int, tuple[float, float, float]] = {}
        self.truth_widths: list[int] = []
        self.truth_messages = 0
        self.claim: dict | None = None
        self.claim_failures = 0
        self.claim_age_s: float | None = None
        self.book: dict | None = None
        self.book_failures = 0
        self.book_age_s: float | None = None
        #: The gate's own payload per robot, with the wall time it arrived. Kept as a dict
        #: per robot and never merged with `odom`: they are two facts about one robot and
        #: averaging them would invent a third. ``gate_state_topics`` empty means "no gate
        #: topics requested", and the samples then carry ``gate: {}`` rather than a
        #: half-filled map that reads as "the gate said nothing".
        self.gate: dict[str, dict] = {}
        self.gate_arrival: dict[str, float] = {}
        self.gate_messages = 0
        self.gate_parse_failures = 0
        self.gate_state_topics = dict(gate_state_topics or {})
        #: Outstanding request futures, one per source. Harvested on the NEXT tick rather
        #: than waited on in this one: waiting inside a timer callback cannot complete when
        #: the node's callback group is MutuallyExclusive (P6, measured).
        self._outstanding: dict[str, object] = {}

        self.create_subscription(TFMessage, truth_topic, self._on_truth, truth_qos())
        for robot in robots:
            self.create_subscription(
                Odometry, f"/{robot}/odom", self._mk_odom(robot), 10)
        for robot, topic in self.gate_state_topics.items():
            self.create_subscription(
                String, topic, self._mk_gate_state(robot), 10)
        # Absolute: this node is a fleet-wide singleton, not namespaced per robot.
        self.claims = self.create_client(Trigger, "/fleet/tasks")
        # The reservation book lives in the coordinator, not in the task service. Reading it
        # from the wrong service is defect (1) in this file's header.
        self.books = self.create_client(Trigger, "/fleet/status")

        self.samples: list[dict] = []
        self.t0 = time.monotonic()
        self.create_timer(1.0 / max(0.2, sample_hz), self._record)
        self.get_logger().info(
            f"fleet_recorder: robots={robots} truth={truth_topic} out={out_dir}/{label}")

    def _mk_odom(self, robot: str):
        def _cb(msg: Odometry) -> None:
            p = msg.pose.pose
            self.odom[robot] = (p.position.x, p.position.y,
                                yaw_from_quaternion(p.orientation))
        return _cb

    def _mk_gate_state(self, robot: str):
        def _cb(msg: String) -> None:
            self._on_gate_state(robot, msg)
        return _cb

    def _on_gate_state(self, robot: str, msg: String) -> None:
        """File one gate payload, stamped with when it ARRIVED.

        Three outcomes, counted apart, because they are three different faults:

        * a payload that will not parse       -> ``gate_parse_failures``
        * a payload that is not an object     -> ``gate_parse_failures``
        * a good payload                      -> ``gate_messages`` and a stored copy

        The previous shape of this node had no gate subscription at all, so "the gate
        published nothing" and "the gate published something nobody kept" were the same
        silence. They no longer are.
        """
        try:
            body = json.loads(msg.data)
        except (json.JSONDecodeError, TypeError):
            self.gate_parse_failures += 1
            return
        if not isinstance(body, dict):
            self.gate_parse_failures += 1
            return
        self.gate_messages += 1
        self.gate[robot] = body
        self.gate_arrival[robot] = time.monotonic()

    def _on_truth(self, msg: TFMessage) -> None:
        """Record EVERY transform's pose. No index arithmetic decides identity here.

        The previous version read `transforms[slot * LINKS_PER_MODEL]`, i.e. indices 0 and 8,
        on the belief that a model occupies a contiguous block of eight. Measured against the
        two-robot world (`scripts/probe_truth_slots.py`), the stream is 16 transforms laid out
        as two model frames FIRST -- index 0 starting on r01's spawn and index 1 on r02's --
        followed by each model's seven local link offsets. Index 8 was therefore one of r01's
        own links, frozen at (0.200, 0.000), and r02 was never read at all. That is why a
        two-robot recording was judged UNKNOWN with "no slot starts on the spawn of ['r02']"
        while the recording did contain r02.

        `slot` instead of `slot * 8` would happen to be right for this world. It would encode
        a second layout guess in the same place, so instead the slicing is gone: identity is
        decided downstream by geometry (`judge.attribute_truth` accepts a slot as a robot only
        when it STARTED on that robot's spawn, reports the rest by position, refuses when the
        assignment is not decisive, and counts continuity violations). An assumption that can
        be checked beats one that is encoded -- and the cost is 16 positions per sample.
        """
        self.truth_messages += 1
        width = len(msg.transforms)
        self.truth_widths.append(width)
        for slot, transform in enumerate(msg.transforms):
            t = transform.transform.translation
            self.truth[slot] = (
                t.x,
                t.y,
                yaw_from_quaternion(transform.transform.rotation),
            )

    #: The two subscriptions this recorder reads claims from. Each is (client attribute,
    #: name used in the audit, callback that files the decoded body). Kept as data so adding
    #: a source cannot accidentally add a fourth polling style.
    def _issue(self, name: str, client) -> None:
        """Send one request if none is outstanding for this source."""
        if name in self._outstanding:
            return
        if not client.service_is_ready():
            self._count_failure(name)
            return
        self._outstanding[name] = client.call_async(Trigger.Request())

    def _harvest(self, name: str) -> None:
        """File the answer of the request issued on an earlier tick, if it has one."""
        future = self._outstanding.get(name)
        if future is None or not future.done():
            return
        del self._outstanding[name]
        result = future.result()
        if result is None or not result.success:
            self._count_failure(name)
            return
        try:
            body = json.loads(result.message)
        except json.JSONDecodeError:
            self._count_failure(name)
            return
        self._file(name, body)

    def _count_failure(self, name: str) -> None:
        if name == "claim":
            self.claim_failures += 1
        else:
            self.book_failures += 1

    def _file(self, name: str, body: dict) -> None:
        if name == "claim":
            self.claim = body
            self.claim_age_s = 0.0
            return
        # The book is stored in the shape the judge reads (`state`/`owner`/`queue` per
        # region) rather than as the coordinator's whole status: a sample has to be
        # comparable with the P4 recordings, and that is the shape they carry.
        regions = body.get("resources") or {}
        self.book = {
            str(region): {
                "state": (entry or {}).get("state"),
                "owner": (owner if (owner := (entry or {}).get("owner")) else None),
                "queue": list((entry or {}).get("queue") or []),
            }
            for region, entry in regions.items()
        }
        self.book_age_s = 0.0

    def _poll_claims(self) -> None:
        """Issue and harvest both sources. Never waits."""
        for name, client in (("claim", self.claims), ("book", self.books)):
            self._harvest(name)
            self._issue(name, client)

    def _record(self) -> None:
        self._poll_claims()
        self.samples.append({
            "t": round(time.monotonic() - self.t0, 3),
            "odom": {r: self.odom.get(r) for r in self.robots},
            "truth": {str(k): list(v) for k, v in sorted(self.truth.items())},
            "claim": self.claim,
            # Read by judge.no_unauthorised_entry_from_truth. The coordinator's book, not
            # the task service's snapshot.
            "resources": self.book,
            # The gate's own pose payload, plus how long ago THIS recorder saw it. The age
            # is included so the reader never has to line two timestamps up by hand, and it
            # is the recorder's age -- the gate's own `localiser_age_s` inside the payload
            # is the one the gate judged on, and both are kept.
            "gate": {
                r: ({**self.gate[r],
                     "gate_age_s": round(time.monotonic() - self.gate_arrival[r], 3)}
                    if r in self.gate else None)
                for r in self.robots
            },
        })

    def write(self) -> dict:
        path = self.out_dir / f"samples-{self.label}.jsonl"
        path.write_text("\n".join(json.dumps(s) for s in self.samples), encoding="utf-8")
        widths = sorted(set(self.truth_widths))
        audit = {
            "samples": len(self.samples),
            "truth_messages": self.truth_messages,
            # The RAW message widths, because a width changing within one run is itself the
            # evidence that an index is not a stable identity (measured: 8/16/24 in one run).
            "truth_widths": widths,
            "truth_slots_recorded": max((len(self.truth), 0)),
            "truth_qos_depth": TRUTH_QOS_DEPTH,
            # `links_per_model` used to be reported here. It asserted a model's link count as
            # a fact and the fact was wrong (two models are 16 transforms with both model
            # frames first), so reporting it invited the arithmetic that caused the fault.
            "truth_note": (
                "every transform index is recorded; identity is decided by judge."
                "attribute_truth from where each slot started, never from the index"),
            "robots": self.robots,
            "claim_failures": self.claim_failures,
            "claim_available": self.claim is not None,
            "book_failures": self.book_failures,
            "book_available": self.book is not None,
            "book_regions": sorted(self.book or {}),
            # The gate instrument, audited the same way the others are. A recording whose
            # gate payloads never arrived must not be judgeable as if they had.
            "gate_topics": dict(self.gate_state_topics),
            "gate_messages": self.gate_messages,
            "gate_parse_failures": self.gate_parse_failures,
            "gate_available": bool(self.gate),
            "gate_robots_seen": sorted(self.gate),
            "label": self.label,
            "file": str(path),
        }
        (self.out_dir / f"recorder-{self.label}.json").write_text(
            json.dumps(audit, indent=2), encoding="utf-8")
        return audit


def _install_stop_handlers(on_stop) -> None:
    """Make SIGTERM mean the same thing as SIGINT, and make one stop mean one stop.

    `ros2 run` handles SIGINT itself *and* forwards it, so the child sees it twice: the first
    raises KeyboardInterrupt out of the sleep, the second lands inside the flush and takes the
    whole recording with it (measured: exit 254, no file). SIGTERM was not handled at all, so
    a supervisor that used it got no file either. Both are fixed by setting a flag and letting
    the main loop leave in an orderly way, and by ignoring further stops while flushing.
    """
    import signal

    def _stop(signum, frame):  # noqa: ANN001 - signal handler signature
        on_stop(signum)

    for name in ("SIGINT", "SIGTERM"):
        if hasattr(signal, name):
            signal.signal(getattr(signal, name), _stop)


def _ignore_stops() -> None:
    """Called before the flush. From here on nothing can interrupt the write."""
    import signal

    for name in ("SIGINT", "SIGTERM"):
        if hasattr(signal, name):
            signal.signal(getattr(signal, name), signal.SIG_IGN)


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(prog="fleet_recorder")
    ap.add_argument("--robots", default="r01,r02,r03")
    ap.add_argument("--out", required=True)
    ap.add_argument("--label", default="run")
    ap.add_argument("--truth-topic", default=DEFAULT_TRUTH_TOPIC)
    ap.add_argument("--sample-hz", type=float, default=5.0)
    ap.add_argument("--gate-state-topic", default="{robot}/gate_state",
                    help="topic pattern for each gate's JSON state; {robot} is substituted. "
                         "Recorded per robot and stored in every sample under `gate`.")
    ap.add_argument("--duration", type=float, default=0.0,
                    help="0 means run until interrupted")
    args, ros_args = ap.parse_known_args(argv)

    robots = [r.strip() for r in args.robots.split(",") if r.strip()]
    rclpy.init(args=ros_args)
    gate_topics = {r: args.gate_state_topic.format(robot=r) for r in robots}
    node = Recorder(robots, args.truth_topic, pathlib.Path(args.out), args.label,
                    args.sample_hz, gate_state_topics=gate_topics)
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()

    stopped = {"by": 0}

    def _on_stop(signum: int) -> None:
        stopped["by"] = signum
        # Leave the sleep from the handler rather than polling: the loop below has to notice
        # within one tick whatever the caller sent.
        raise KeyboardInterrupt

    _install_stop_handlers(_on_stop)
    try:
        if args.duration > 0:
            time.sleep(args.duration)
        else:
            while True:
                time.sleep(0.25)
    except KeyboardInterrupt:
        pass
    finally:
        # Nothing may interrupt the flush. This is the whole difference between a recording
        # and an empty directory.
        _ignore_stops()
        audit = node.write()
        audit["stopped_by_signal"] = stopped["by"]
        print(json.dumps(audit, indent=2))
        # Leave without rclpy's teardown: DDS shutdown segfaults in this environment and
        # turns a finished recording into exit 139. The data is flushed above.
        sys.stdout.flush()
        sys.stderr.flush()
        import os

        os._exit(0 if audit["samples"] else 4)


if __name__ == "__main__":
    raise SystemExit(main())
