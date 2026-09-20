#!/usr/bin/env python3
"""Ask the coordinator for a passage, and print exactly what it answers.

WHY A PROBE AND NOT A REASONED GUESS
------------------------------------
Round 5 concluded from a live run that `/fleet/acquire_passage` "never answered". The numbers in
that run's own driver log say otherwise: 41 attempts in 20 s is forty refusals at the 0.5 s
backoff plus one request that was interrupted by my own `kill`. A refusal IS an answer. So the
open question is not "does the coordinator answer" but "what is it refusing with, and why".

That question has a one-call answer, and this is the instrument that asks it. It prints:

  * `/fleet/status` -- `sweep_done` and the reservation book. The coordinator refuses EVERY
    acquire with `RESOURCE_UNKNOWN` while its start-up occupancy sweep is unfinished, and the
    sweep runs once; if it never completed, no crossing in this configuration can ever be
    granted, which is the difference between "the corridor is busy" and "the corridor is dead".
  * the full response of `/fleet/acquire_passage`, for a named robot and direction, retried.

It publishes nothing and moves nothing. It is a client, a witness and a question.
"""

from __future__ import annotations

import argparse
import json
import sys
import time

import rclpy
from fleet_interfaces.srv import AcquirePassage
from rclpy.node import Node
from std_srvs.srv import Trigger


class Prober(Node):
    def __init__(self) -> None:
        super().__init__("acquire_probe")
        self.status = self.create_client(Trigger, "/fleet/status")
        self.acquire = self.create_client(AcquirePassage, "/fleet/acquire_passage")

    def call(self, client, request, timeout: float = 10.0):
        if not client.wait_for_service(timeout_sec=timeout):
            return None
        future = client.call_async(request)
        deadline = time.monotonic() + timeout
        while not future.done() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
        if not future.done():
            return None
        return future.result()

    def coordinator_status(self) -> dict:
        resp = self.call(self.status, Trigger.Request())
        if resp is None or not resp.success:
            return {}
        try:
            return json.loads(resp.message)
        except json.JSONDecodeError:
            return {}


def main(argv: "list[str] | None" = None) -> int:
    ap = argparse.ArgumentParser(prog="probe_acquire")
    ap.add_argument("--robot", default="r01")
    ap.add_argument("--direction", default="west_to_east")
    ap.add_argument("--task-id", default="probe-acquire")
    ap.add_argument("--tries", type=int, default=4)
    ap.add_argument("--delay", type=float, default=2.0)
    ap.add_argument("--status-timeout", type=float, default=60.0)
    args, ros_args = ap.parse_known_args(argv)

    rclpy.init(args=ros_args)
    node = Prober()
    try:
        print(f"probe: waiting up to {args.status_timeout:.0f}s for /fleet/status ...")
        deadline = time.monotonic() + args.status_timeout
        status: dict = {}
        while time.monotonic() < deadline:
            status = node.coordinator_status()
            if status:
                break
            time.sleep(1.0)
        if not status:
            print("NO_COORDINATOR_STATUS: /fleet/status did not answer; nothing can be judged")
            return 3

        print(f"  sweep_done   : {status.get('sweep_done')}")
        resources = status.get("resources") or {}
        for name in sorted(resources):
            body = resources[name] or {}
            print(f"  resource {name:10s} state={body.get('state')} owner={body.get('owner')!r} "
                  f"queue={body.get('queue')}")
        robots = status.get("robots") or {}
        for rid in sorted(robots):
            body = robots[rid] or {}
            print(f"  robot {rid}: fresh={body.get('fresh')} pose_age_s="
                  f"{body.get('pose_age_s')} inside={body.get('inside')}")
        print(f"  (whole status: {json.dumps(status, sort_keys=True)[:400]}...)")

        if status.get("sweep_done") is not True:
            print("")
            print("FINDING: sweep_done is not true. Every acquire is refused RESOURCE_UNKNOWN")
            print("         until it is, so no crossing in this configuration can be granted.")

        print("")
        print(f"probe: asking for a passage {args.tries} time(s), {args.delay:.1f}s apart")
        answers = 0
        reasons: list[str] = []
        for attempt in range(1, args.tries + 1):
            req = AcquirePassage.Request()
            req.robot_id = args.robot
            req.boot_id = ""
            req.revision = 0
            req.generation = 1
            req.task_id = args.task_id
            req.direction = args.direction
            req.request_id = f"{args.task_id}-probe-{attempt}"
            req.localization_valid = True
            resp = node.call(node.acquire, req)
            if resp is None:
                print(f"  try {attempt}: NO RESPONSE within 10 s")
            else:
                answers += 1
                reasons.append(resp.reason_code)
                print(f"  try {attempt}: granted={resp.granted} reason={resp.reason_code!r} "
                      f"detail={resp.detail!r} permit={resp.permit_id!r} "
                      f"valid_for={resp.valid_for_wall_s:.2f}s epoch={getattr(resp, 'epoch', None)}")
            time.sleep(args.delay)

        print("")
        print(f"SUMMARY: {answers}/{args.tries} answered; reasons={sorted(set(reasons))}")
        if answers == 0:
            print("VERDICT: the service did not answer any request")
            return 4
        if any(r == "OK" for r in reasons):
            print("VERDICT: a passage WAS granted -- the coordinator is granting in this "
                  "configuration")
            return 0
        print(f"VERDICT: it answers, and refuses with {sorted(set(reasons))} -- that refusal is "
              "the thing to fix, not the service")
        return 5
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
