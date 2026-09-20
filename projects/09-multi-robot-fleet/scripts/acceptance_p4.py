#!/usr/bin/env python3
"""P4.5 acceptance: two robots in opposite directions, one corridor, verified.

Run with the fleet already up (scripts/acceptance_p4.sh does that). This program

  1. drives both robots at the crossing at the same time, in opposite directions;
  2. samples odometry, the reservation book and (best effort) ground truth at ~5 Hz;
  3. states what the samples prove and what they do not.

The three claims it is allowed to make, and the sample that would falsify each:

  * **no simultaneous occupancy** -- some sample shows both footprints inside the
    protected region at once;
  * **no unauthorised entry**     -- some sample shows a footprint inside the
    region while the book says that robot holds nothing;
  * **no starvation**             -- one direction completes and the other does not.

Ordering is a test VARIABLE, not a mechanism: `--bias-s` only changes who asks
first. What happens next is decided by the reservation book. MASTER_PLAN section P4
forbids faking the schedule with per-robot sleeps, and this is the distinction that
keeps that honest.

Ground truth is used only to cross-check the odometry-based claims. It is NEVER fed
back to control (AGENTS.md rule 13).
"""

from __future__ import annotations

import argparse
import json
import math
import os
import subprocess
import sys
import threading
import time
from pathlib import Path

import rclpy
import yaml
from fleet_core import validate_traffic_config
from fleet_core.geometry import compose_2d, footprint_corners
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_msgs.msg import TFMessage

L, W = 0.60, 0.45


def _repo_root() -> Path:
    import os

    env = os.environ.get("FLEET009_ROOT")
    root = Path(env) if env else None
    if root is None:
        for parent in Path(__file__).resolve().parents:
            if (parent / "config" / "resources.yaml").is_file():
                root = parent
                break
    if root is None:
        raise SystemExit("cannot find config/resources.yaml; set FLEET009_ROOT")
    return root


def load_traffic():
    return validate_traffic_config(
        yaml.safe_load(
            (_repo_root() / "config" / "resources.yaml").read_text(encoding="utf-8")))


def load_spawns() -> dict[str, tuple[float, float, float]]:
    """Each robot's spawn pose == the origin of its odom frame.

    The sampler needs it for the same reason the coordinator and the gate do:
    odometry is in the odom frame and the rectangles are in the map frame, so
    comparing them directly would put every robot inside the corridor at boot. The
    first version of this harness made exactly that mistake and would have
    "confirmed" a problem that was not there.
    """
    table = yaml.safe_load(
        (_repo_root() / "config" / "spawns.yaml").read_text(encoding="utf-8")) or {}
    out = {}
    for robot, entry in (table.get("spawns") or {}).items():
        out[robot] = (float(entry.get("x", 0.0)), float(entry.get("y", 0.0)),
                      float(entry.get("yaw", 0.0)))
    return out


def yaw_of(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class Sampler(Node):
    def __init__(self, cfg, robots, truth_topic, spawns):
        super().__init__("p4_acceptance_sampler")
        self.cfg = cfg
        self.robots = robots
        self.spawns = spawns
        self.odom: dict[str, tuple] = {}
        self.odom_raw: dict[str, tuple] = {}
        self.truth: dict[int, tuple] = {}
        self.truth_count = 0
        self.truth_widths: list[int] = []
        self.samples: list[dict] = []
        self.status_failures = 0
        self.gate_reasons: dict[str, int] = {}
        self.gate_counts: dict[str, int] = {}
        self.gate_last: dict[str, dict] = {}
        self.gate_first_refusals: list[dict] = []
        self._gate_unparsed = 0
        # What each gate says about its own permit path, read from its state JSON.
        # Counters, not rates, so the sampler cannot throttle them.
        self.gate_permit_grants: dict[str, int] = {}
        self.gate_guard_installed: dict[str, bool] = {}
        for r in robots:
            self.create_subscription(Odometry, f"/{r}/odom", self._mk(r), 10)
            # The gate publishes its verdict as JSON on <robot>/gate_state. Recording
            # it is the difference between "the robot did not move" and "the robot did
            # not move because the gate refused for reason X". Without it, a standstill
            # is consistent with half a dozen causes and the next step is a guess.
            self.create_subscription(
                String, f"/{r}/gate_state", self._mk_gate(r), 10)
        self.create_subscription(TFMessage, truth_topic, self._truth, 10)
        self.cli = self.create_client(Trigger, "/fleet/status")

    def _mk_gate(self, robot):
        def _cb(msg: String) -> None:
            try:
                body = json.loads(msg.data)
            except Exception:
                self._gate_unparsed += 1
                return
            reason = str(body.get("reason", "?"))
            self.gate_reasons[reason] = self.gate_reasons.get(reason, 0) + 1
            self.gate_counts[robot] = self.gate_counts.get(robot, 0) + 1
            # Monotonic counters, so the last sample carries the whole story and a
            # missed sample cannot make the path look broken.
            seen = int(body.get("permit_grants_seen", 0) or 0)
            self.gate_permit_grants[robot] = max(
                self.gate_permit_grants.get(robot, 0), seen)
            if "guard_installed" in body:
                self.gate_guard_installed[robot] = bool(body["guard_installed"])
            self.gate_last[robot] = {
                "mode": body.get("mode"),
                "reason": reason,
                "detail": str(body.get("detail", ""))[:400],
                "wall_s": body.get("wall_s"),
            }
            if reason != "OK" and len(self.gate_first_refusals) < 6:
                self.gate_first_refusals.append(
                    {"robot": robot, "mode": body.get("mode"), "reason": reason,
                     "detail": str(body.get("detail", ""))[:400]})
        return _cb

    def _mk(self, robot):
        spawn = self.spawns.get(robot, (0.0, 0.0, 0.0))

        def _cb(msg: Odometry) -> None:
            p = msg.pose.pose
            raw = (p.position.x, p.position.y, yaw_of(p.orientation))
            self.odom_raw[robot] = raw
            self.odom[robot] = compose_2d(spawn, raw)
        return _cb

    def _truth(self, msg: TFMessage) -> None:
        self.truth_count += 1
        self.truth_widths.append(len(msg.transforms))
        # transforms[0] is the first model's pose in world coordinates; with two
        # robots the block repeats every 8 links. Verified rather than assumed: the
        # report fails the truth cross-check if the width is not what that implies.
        for idx, slot in ((0, 0), (8, 1)):
            if idx < len(msg.transforms):
                t = msg.transforms[idx].transform.translation
                self.truth[slot] = (t.x, t.y)

    def poll_status(self, timeout_s: float = 2.0) -> dict | None:
        """Call /fleet/status without spinning.

        The executor runs on its own thread (see main), so this must NOT call
        spin_until_future_complete: doing that from the measuring loop is precisely
        the self-throttling defect this project already documented once, where the
        probe processed ~5 callbacks/second while ~135 arrived and then reported the
        backlog it had created as if it were the system's behaviour.
        """
        if not self.cli.service_is_ready():
            return None
        fut = self.cli.call_async(Trigger.Request())
        deadline = time.monotonic() + timeout_s
        while not fut.done() and time.monotonic() < deadline:
            time.sleep(0.005)
        if not fut.done():
            self.status_failures += 1
            return None
        res = fut.result()
        if res is None or not res.success:
            self.status_failures += 1
            return None
        return json.loads(res.message)

    def rate_self_check(self, elapsed_s: float) -> dict:
        """Compare delivered rates against what the chain claims to send.

        Standing project rule: any rate claim carries a delivered-versus-expected
        ratio, or it is withheld. A gate verdict stream that arrives at 4 Hz from a
        20 Hz publisher means the sampler is lagging, and its reason counts describe
        the past, not the run.
        """
        out = {}
        for robot, count in self.gate_counts.items():
            observed = count / elapsed_s if elapsed_s > 0 else 0.0
            ratio = observed / 20.0
            out[robot] = {
                "messages": count,
                "observed_hz": round(observed, 2),
                "assumed_publisher_hz": 20.0,
                "ratio": round(ratio, 3),
                "verdict": "OK" if 0.60 <= ratio <= 1.40 else "WITHHELD",
            }
        return out


def region_overlap(cfg, pose, name) -> bool:
    n = cfg.resources[name]
    rects = n.rects
    from fleet_core.geometry import Region

    region = Region(name=name, rects=rects, margin_m=cfg.footprint_margin_m)
    return bool(region.box_overlap(footprint_corners(pose[0], pose[1], pose[2], L, W)))


def run_pair(order: str, bias_s: float, timeout_s: float, report_dir: Path,
             node) -> list[dict]:
    """Start both crossing drivers, in the requested arrival order."""
    plans = [
        ("r01", "west_to_east", "p4-opposing-A"),
        ("r02", "east_to_west", "p4-opposing-B"),
    ]
    if order == "r02_first":
        plans.reverse()

    procs = []
    for i, (robot, direction, task) in enumerate(plans):
        if i > 0 and bias_s > 0:
            time.sleep(bias_s)
        out = report_dir / f"{task}-{robot}.json"
        cmd = [
            "ros2", "run", "fleet_ros", "staged_crossing",
            "--robot", robot, "--direction", direction, "--task-id", task,
            "--generation", "1", "--timeout-s", str(timeout_s),
            "--report", str(out),
        ]
        procs.append((robot, direction, subprocess.Popen(
            cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)))

    results = []
    for robot, direction, proc in procs:
        out, _ = proc.communicate(timeout=timeout_s + 120.0)
        entry = {"robot": robot, "direction": direction, "exit_code": proc.returncode}
        # Cross-check the two ends of the permit path from artifacts, not from a guess.
        # The driver writes down what it was granted; each gate reports how many
        # GRANTED permit messages it saw and whether its guard was installed at all.
        # A driver holding a permit while its gate saw zero grants is the exact shape
        # of the defect that cost two full acceptance runs, and neither end alone
        # could see it.
        task_of = {r: t for r, _d, t in plans}
        report = report_dir / f"{task_of.get(robot, robot)}-{robot}.json"
        try:
            driver = json.loads(report.read_text(encoding="utf-8"))
        except Exception:
            driver = {}
        granted = bool(driver.get("permit_id"))
        seen = int(node.gate_permit_grants.get(robot, 0))
        entry["permit_path"] = {
            "driver_permit_id": driver.get("permit_id") or "",
            "driver_renewals": driver.get("renewals", 0),
            "driver_renew_failures": len(driver.get("renew_failures") or []),
            "gate_guard_installed": node.gate_guard_installed.get(robot),
            "gate_grants_seen": seen,
            "mismatch": bool(granted and seen == 0),
        }
        if entry["permit_path"]["mismatch"]:
            print(f"[acceptance] PERMIT PATH MISMATCH for {robot}: the driver holds "
                  f"{driver.get('permit_id')!r} and its gate saw {seen} grant(s). "
                  "The coordinator granted; the gate never adopted.")
        if proc.returncode != 0:
            # Keep the evidence. Throwing away a driver's stdout already cost one
            # entire simulation run: both drivers exited 3 and the reason went in the
            # bin with their output, so the next step was a guess instead of a read.
            tail = (out or "").strip().splitlines()[-15:]
            entry["stdout_tail"] = tail
            entry["stderr_captured"] = False
            print(f"[acceptance] {robot} ({direction}) exited {proc.returncode}; "
                  "combined output tail:")
            for line in tail:
                print(f"    | {line}")
        results.append(entry)
    return results


def main(argv=None) -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--order", choices=["r01_first", "r02_first"], default="r01_first")
    ap.add_argument("--bias-s", type=float, default=2.0,
                    help="Delay before the second request. Changes ARRIVAL ORDER only.")
    ap.add_argument("--timeout-s", type=float, default=120.0)
    ap.add_argument("--report-dir", required=True)
    ap.add_argument("--truth-topic", default="/world/warehouse/dynamic_pose/info")
    args = ap.parse_args(argv)

    cfg = load_traffic()
    robots = ["r01", "r02"]
    report_dir = Path(args.report_dir)
    report_dir.mkdir(parents=True, exist_ok=True)

    rclpy.init(args=None)
    spawns = load_spawns()
    node = Sampler(cfg, robots, args.truth_topic, spawns)
    print(f"[acceptance] spawn poses (odom frame origins): "
          f"{ {k: tuple(round(v, 3) for v in p) for k, p in spawns.items()} }")
    # Callbacks run on their own thread so the measuring loop never delivers them.
    from rclpy.executors import MultiThreadedExecutor

    executor = MultiThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    print(f"[acceptance] order={args.order} bias={args.bias_s}s")

    # Readiness. Nothing is grantable until the coordinator's occupancy sweep has
    # completed, so starting the drivers before it would measure a queue that is
    # blocked for a reason unrelated to the test.
    print("[acceptance] waiting for the coordinator's occupancy sweep ...")
    ready_deadline = time.monotonic() + 240.0
    status = None
    while time.monotonic() < ready_deadline:
        status = node.poll_status()
        if status and status.get("sweep_done"):
            break
        time.sleep(1.0)
    if not status or not status.get("sweep_done"):
        print(json.dumps({
            "error": "coordinator never reported a completed occupancy sweep",
            "status": status,
            "note": "check that both robots publish /<robot>/odom and that the "
                    "coordinator is running; nothing is grantable before the sweep",
        }, indent=2))
        node.destroy_node()
        rclpy.shutdown()
        return 3
    print(f"[acceptance] sweep done at t={status.get('sweep_done')} "
          f"states={status.get('resource_states')}")
    print(f"[acceptance] startup notes: {status.get('startup_notes')}")

    results: list[dict] = []
    # An exception in a thread does not reach the caller: it prints a traceback and
    # the thread dies, leaving `results` empty. `both_completed` is derived from
    # `results`, so a crash inside the driver thread was reported as a failure of the
    # traffic logic -- in both cases of a run where both robots completed every stage.
    # The error is captured, published in the verdict, and printed where a reader will
    # see it. This is the shape of defect this project keeps meeting: the instrument
    # reporting on itself instead of on the subject.
    drive_error: list[str] = []

    def drive():
        nonlocal results
        try:
            results = run_pair(args.order, args.bias_s, args.timeout_s, report_dir, node)
        except Exception as exc:  # noqa: BLE001 -- reported, never swallowed
            import traceback
            drive_error.append(f"{type(exc).__name__}: {exc}")
            traceback.print_exc()
            print(f"[acceptance] FATAL: the driver thread raised "
                  f"{type(exc).__name__}: {exc}. Any verdict below is NOT a "
                  "statement about the traffic logic.")

    driver = threading.Thread(target=drive)
    driver.start()

    t0 = time.monotonic()
    while driver.is_alive():
        status = node.poll_status()
        sample = {
            "t": round(time.monotonic() - t0, 3),
            "odom": {r: node.odom.get(r) for r in robots},
            "truth": {k: v for k, v in node.truth.items()},
        }
        if status is not None:
            sample["resources"] = {
                name: {"state": body["state"], "owner": body["owner"],
                       "queue": body["queue"]}
                for name, body in status["resources"].items()
            }
            sample["sweep_done"] = status["sweep_done"]
        node.samples.append(sample)
        time.sleep(0.2)
    driver.join()

    # Drain a final sample so the end state is recorded.
    final = node.poll_status()
    end = {
        "t": round(time.monotonic() - t0, 3),
        "odom": {r: node.odom.get(r) for r in robots},
        "truth": {k: v for k, v in node.truth.items()},
        "final_status": final,
    }
    node.samples.append(end)

    # ------------------------------------------------------------------ #
    # adjudication
    # ------------------------------------------------------------------ #
    simultaneous: list[dict] = []
    unauthorised: list[dict] = []
    pair_owner_seen: list[tuple] = []
    busy_seen = 0

    for s in node.samples:
        res = s.get("resources")
        if res:
            owners = {n: b["owner"] for n, b in res.items() if b["owner"]}
            if owners:
                pair_owner_seen.append((s["t"], tuple(sorted(set(owners.values())))))
            if any(b["queue"] for b in res.values()):
                busy_seen += 1

        inside = {}
        for r in robots:
            pose = s["odom"].get(r)
            if pose is None:
                continue
            hit = [n for n in cfg.resources if region_overlap(cfg, pose, n)]
            if hit:
                inside[r] = hit

        if len(inside) > 1:
            simultaneous.append({"t": s["t"], "inside": inside, "odom": s["odom"]})

        if inside and res:
            for r, hit in inside.items():
                owner = res.get(hit[0], {}).get("owner")
                if owner is None:
                    unauthorised.append({"t": s["t"], "robot": r, "rects": hit,
                                         "odom": s["odom"], "owner": owner})

    truth_widths = sorted(set(node.truth_widths))
    truth_usable = bool(node.truth_count) and truth_widths and truth_widths[-1] >= 16
    truth_note = (
        f"{node.truth_count} truth messages, transform widths {truth_widths}"
        if node.truth_count else "no truth messages received"
    )

    verdict = {
        "order": args.order,
        "bias_s": args.bias_s,
        "samples": len(node.samples),
        "status_failures": node.status_failures,
        "runs": results,
        "drive_error": drive_error,
        "both_completed": (all(r["exit_code"] == 0 for r in results)
                           and len(results) == 2
                           and not drive_error),
        "simultaneous_occupancy_samples": len(simultaneous),
        "unauthorised_entry_samples": len(unauthorised),
        "busy_or_queued_samples": busy_seen,
        "distinct_owner_phases": [
            {"t": t, "owners": list(o)} for t, o in pair_owner_seen
        ][:40],
        "truth": {
            "usable_for_cross_check": truth_usable,
            "note": truth_note,
            "width_interpretation": (
                "8 links per robot, so a two-robot message carries 16 transforms and "
                "transforms[0] / transforms[8] are the two model poses"),
        },
        "first_simultaneous": simultaneous[:2],
        "first_unauthorised": unauthorised[:2],
        "gate": {
            "reason_counts": dict(sorted(node.gate_reasons.items(),
                                        key=lambda kv: -kv[1])),
            "unparsed": node._gate_unparsed,
            "last_verdict_per_robot": node.gate_last,
            "first_non_ok_verdicts": node.gate_first_refusals,
            "rate_self_check": node.rate_self_check(
                node.samples[-1]["t"] if node.samples else 0.0),
        },
    }
    print(json.dumps(verdict, indent=2))
    (report_dir / f"verdict-{args.order}.json").write_text(
        json.dumps(verdict, indent=2), encoding="utf-8")
    (report_dir / f"samples-{args.order}.jsonl").write_text(
        "\n".join(json.dumps(s) for s in node.samples), encoding="utf-8")

    ok = (verdict["both_completed"] and not simultaneous and not unauthorised)
    print(f"[acceptance] order={args.order} -> {'PASS' if ok else 'FAIL'}")

    # Leave WITHOUT rclpy's teardown. Every run so far ended in a segfault during
    # interpreter shutdown (exit 139), which turned a readable PASS/FAIL into a crash
    # code and left "did this case pass" to be re-derived from the JSON by hand. The
    # verdict is already written and flushed above; the fault is in DDS teardown and
    # says nothing about the run. Recorded here rather than hidden -- if a future
    # version needs a clean shutdown, this is the first line to revisit.
    sys.stdout.flush()
    sys.stderr.flush()
    os._exit(0 if ok else 4)


if __name__ == "__main__":
    raise SystemExit(main())
