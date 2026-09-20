"""fleet_tools: the operator entry point. Read, submit, cancel, inject faults.

MASTER_PLAN section 9 (P6) is explicit that the CLI -- not the web page -- is where
submission, cancellation and fault injection live, and that the web page is
read-only. This module is therefore the only supported way to change fleet state by
hand, and it is deliberately small: every option maps to one field of one service,
and nothing here decides anything.

Two habits it enforces on the operator:

  * **`wait` reports the terminal state the ledger actually holds**, never an
    inference from "the robot looks like it arrived". A task that has been driven
    to the right station but not yet delivered is not done, and a tool that says
    otherwise is how a report ends up claiming a transfer that never happened.
  * **`fault` prints whether the injection was acknowledged.** A fault that was
    requested and never took effect turns a test into a silent no-op, and the
    handoff's P5 instructions require the expected fault to be shown as having
    fired.
"""

from __future__ import annotations

import argparse
import json
import sys
import time

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger

from fleet_interfaces.srv import CancelTask, FaultInject, SubmitTask

TERMINAL = {"SUCCEEDED", "FAILED", "CANCELED"}


class Cli(Node):
    def __init__(self) -> None:
        super().__init__("fleet_cli")
        self.submit_cli = self.create_client(SubmitTask, "/fleet/submit_task")
        self.cancel_cli = self.create_client(CancelTask, "/fleet/cancel_task")
        self.tasks_cli = self.create_client(Trigger, "/fleet/tasks")
        self.traffic_cli = self.create_client(Trigger, "/fleet/status")

    # -- generic call ---------------------------------------------------- #

    def call(self, client, request, timeout_s: float = 8.0, wait_s: float = 25.0):
        """Wait for the service to be discoverable, then call it, then retry once.

        A single `wait_for_service(timeout_sec=5)` is not enough on this graph. With
        three robots the fleet is ~55 DDS participants, and discovery routinely takes
        longer than five seconds. The symptom is `no <service>` reported while the node
        is demonstrably alive, and that cost an entire three-robot run every reading it
        was supposed to produce. So: poll `service_is_ready` with a real budget, and if
        the call still comes back empty, try once more before giving up.
        """
        deadline = time.monotonic() + wait_s
        while time.monotonic() < deadline and not client.service_is_ready():
            time.sleep(0.2)
        if not client.service_is_ready():
            return None
        for attempt in range(2):
            future = client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=timeout_s)
            result = future.result()
            if result is not None:
                return result
            if attempt == 0:
                time.sleep(0.5)
        return None

    def snapshot(self) -> dict | None:
        result = self.call(self.tasks_cli, Trigger.Request())
        if result is None or not result.success:
            return None
        return json.loads(result.message)

    def traffic(self) -> dict | None:
        result = self.call(self.traffic_cli, Trigger.Request())
        if result is None or not result.success:
            return None
        try:
            return json.loads(result.message)
        except ValueError:
            return {"raw": result.message}


# --------------------------------------------------------------------------- #
# commands
# --------------------------------------------------------------------------- #


def cmd_status(cli: Cli, args) -> int:
    snap = cli.snapshot()
    if snap is None:
        print("no task service on /fleet/tasks", file=sys.stderr)
        return 2
    if args.json:
        print(json.dumps(snap, indent=2, sort_keys=True))
        return 0

    print(f"epoch {snap['epoch']}  ledger_boot {snap['ledger_boot']}  "
          f"backend {snap['backend']}  healthy {snap['ledger_healthy']}")
    bm = snap["battery_model"]
    print(f"battery model: SIMULATED  charge={bm['charge_wh_per_s']} Wh/s  "
          f"low={bm['low_fraction']} critical={bm['critical_fraction']} "
          f"resume={bm['resume_fraction']}")
    print()
    print(f"{'robot':6} {'state':10} {'band':9} {'batt':>6} {'fresh':6} {'task':22} notes")
    for rid, r in snap["robots"].items():
        notes = []
        if not r["fresh"]:
            notes.append("STALE")
        if r["parked"]:
            notes.append(f"PARKED:{r['parked']}")
        if r["fault_code"]:
            notes.append(f"FAULT:{r['fault_code']}")
        if not r["localization_valid"]:
            notes.append("NO-LOC")
        if r["stopped_confirmed"]:
            notes.append("stopped")
        if r["gate_reason"]:
            notes.append(f"gate={r['gate_reason']}")
        print(f"{rid:6} {r['operating_state']:10} {r['band']:9} "
              f"{r['battery_fraction']:6.2f} {str(r['fresh']):6} {r['task_id']:22} "
              f"{' '.join(notes)}")
    print()
    print("tasks:")
    for t in snap["tasks"]:
        print(f"  {t['task_id']:26} {t['state']:16} {t['kind']:16} "
              f"robot={t['robot_id'] or '-':5} leg={t['leg'] or '-':16} "
              f"attempts={t['attempts']}  {t['detail'][:60]}")
    if snap["payloads"]:
        print()
        print("payloads (logical, no mechanical handling):")
        for p in snap["payloads"]:
            print(f"  {p['payload_id']:14} {p['state']:11} holder={p['holder'] or '-'}")
    print()
    print("chargers:", json.dumps(snap["chargers"], sort_keys=True))
    if snap["refusals"]:
        print("refusals:", json.dumps(snap["refusals"], sort_keys=True))
    not_resumed = snap.get("reconcile", {}).get("not_resumed", [])
    if not_resumed:
        print(f"restart parked {len(not_resumed)} task(s) and resumed none: "
              f"{[e['task_id'] for e in not_resumed]}")
    return 0


def cmd_submit(cli: Cli, args) -> int:
    req = SubmitTask.Request()
    req.schema_version = "1"
    req.request_id = args.request_id
    req.kind = args.kind
    req.pick_station = args.pick or ""
    req.destination_station = args.drop
    req.required_capability = args.capability
    req.payload_mode = args.payload_mode
    req.payload_id = args.payload or ""
    req.priority = int(args.priority)
    req.service_duration_s = float(args.service_duration)
    req.max_attempts = int(args.max_attempts)
    req.timeout_sim_s = float(args.timeout_sim)
    result = cli.call(cli.submit_cli, req)
    if result is None:
        print("no task service on /fleet/submit_task", file=sys.stderr)
        return 2
    print(f"accepted={result.accepted} task_id={result.task_id!r} "
          f"state={result.state!r} reason={result.reason_code!r}")
    print(f"  {result.message}")
    # `accepted` means received. Saying so here, at the one place an operator looks,
    # is cheaper than a report that confuses the two.
    if result.accepted:
        print("  note: accepted != succeeded; use `fleet_cli wait` to see the outcome")
    return 0 if result.accepted else 1


def cmd_cancel(cli: Cli, args) -> int:
    req = CancelTask.Request()
    req.task_id = args.task_id
    result = cli.call(cli.cancel_cli, req)
    if result is None:
        print("no task service on /fleet/cancel_task", file=sys.stderr)
        return 2
    print(f"accepted={result.accepted} stopped={result.stopped} "
          f"state={result.state!r} reason={result.reason_code!r}")
    print(f"  {result.message}")
    return 0 if result.accepted else 1


def cmd_wait(cli: Cli, args) -> int:
    deadline = time.monotonic() + args.timeout
    last = ""
    while time.monotonic() < deadline:
        snap = cli.snapshot()
        if snap is None:
            print("no task service on /fleet/tasks", file=sys.stderr)
            return 2
        match = [t for t in snap["tasks"] if t["task_id"] == args.task_id]
        if not match:
            print(f"unknown task {args.task_id!r}", file=sys.stderr)
            return 2
        task = match[0]
        line = (f"{task['state']:16} leg={task['leg'] or '-':16} "
                f"attempts={task['attempts']} robot={task['robot_id'] or '-'}")
        if line != last:
            print(f"[{time.monotonic() - (deadline - args.timeout):6.1f}s] {line}")
            last = line
        if task["state"] in TERMINAL:
            print(f"terminal: {task['state']} reason={task['reason']} {task['detail']}")
            return 0 if task["state"] == "SUCCEEDED" else 1
        time.sleep(args.interval)
    print(f"timed out after {args.timeout}s; last seen {last or 'nothing'}", file=sys.stderr)
    return 3


def cmd_fault(cli: Cli, args) -> int:
    from fleet_interfaces.srv import FaultInject as FI

    client = cli.create_client(FI, f"/{args.robot}/fleet/inject_fault")
    req = FI.Request()
    req.schema_version = "1"
    req.robot_id = args.robot
    req.fault_code = args.code
    req.duration_s = float(args.duration)
    result = cli.call(client, req)
    if result is None:
        print(f"no adapter on /{args.robot}/fleet/inject_fault", file=sys.stderr)
        return 2
    print(f"ok={result.ok} {result.message}")
    if not result.ok:
        print("  the fault was NOT injected; anything that follows proves nothing",
              file=sys.stderr)
    return 0 if result.ok else 1


def cmd_traffic(cli: Cli, args) -> int:
    data = cli.traffic()
    if data is None:
        print("no coordinator on /fleet/status", file=sys.stderr)
        return 2
    print(json.dumps(data, indent=2, sort_keys=True))
    return 0


def cmd_events(cli: Cli, args) -> int:
    from pathlib import Path

    path = Path(args.path)
    if not path.is_file():
        print(f"no event log at {path}", file=sys.stderr)
        return 2
    shown = 0
    for line in path.read_text(encoding="utf-8").splitlines():
        try:
            row = json.loads(line)
        except ValueError:
            continue
        if args.kind and row.get("kind") != args.kind:
            continue
        if args.subject and row.get("subject") != args.subject:
            continue
        print(f"{row['mono']:12.3f} {row['kind']:20} {row['subject']:26} "
              f"{json.dumps(row['data'], sort_keys=True)[:110]}")
        shown += 1
        if args.limit and shown >= args.limit:
            break
    return 0


# --------------------------------------------------------------------------- #


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="fleet_cli",
        description="009 fleet operator tool. Read, submit, cancel, inject faults.",
    )
    sub = parser.add_subparsers(dest="command", required=True)

    p = sub.add_parser("status", help="one snapshot of every robot, task and pad")
    p.add_argument("--json", action="store_true")
    p.set_defaults(func=cmd_status)

    p = sub.add_parser("submit", help="submit a task")
    p.add_argument("--request-id", required=True)
    p.add_argument("--kind", default="station_transfer")
    p.add_argument("--pick", default="")
    p.add_argument("--drop", required=True)
    p.add_argument("--payload", default="")
    p.add_argument("--capability", default="CARRY")
    p.add_argument("--payload-mode", default="logical")
    p.add_argument("--priority", type=int, default=10)
    p.add_argument("--service-duration", type=float, default=2.0)
    p.add_argument("--max-attempts", type=int, default=2)
    p.add_argument("--timeout-sim", type=float, default=180.0)
    p.set_defaults(func=cmd_submit)

    p = sub.add_parser("cancel", help="request cancellation of a task")
    p.add_argument("task_id")
    p.set_defaults(func=cmd_cancel)

    p = sub.add_parser("wait", help="poll a task until it reaches a terminal state")
    p.add_argument("task_id")
    p.add_argument("--timeout", type=float, default=300.0)
    p.add_argument("--interval", type=float, default=1.0)
    p.set_defaults(func=cmd_wait)

    p = sub.add_parser("fault", help="inject a fault into one robot's adapter")
    p.add_argument("robot")
    p.add_argument("code")
    p.add_argument("--duration", type=float, default=0.0)
    p.set_defaults(func=cmd_fault)

    p = sub.add_parser("traffic", help="the corridor coordinator's view")
    p.set_defaults(func=cmd_traffic)

    p = sub.add_parser("events", help="read the append-only event timeline")
    p.add_argument("--path", default="runtime/events.jsonl")
    p.add_argument("--kind", default="")
    p.add_argument("--subject", default="")
    p.add_argument("--limit", type=int, default=0)
    p.set_defaults(func=cmd_events)
    return parser


def main(argv=None) -> int:
    args = build_parser().parse_args(argv)
    if args.command == "events":
        # Reading a file needs no ROS at all, and starting a node for it would add a
        # DDS participant to a run for no reason.
        return cmd_events(None, args)

    rclpy.init()
    cli = Cli()
    try:
        return args.func(cli, args)
    finally:
        cli.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    sys.exit(main())
