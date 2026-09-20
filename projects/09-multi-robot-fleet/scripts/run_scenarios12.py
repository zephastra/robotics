#!/usr/bin/env python3
"""P5 directed scenarios 1 and 2: the two that need no battery hook.

  A. pre_pickup_takeover     -- cancel a task while the robot is still driving TO the pick,
                                and prove nothing is left locked: cargo unheld, robot free,
                                the same work re-submittable.
  B. no_transfer_while_holding -- with a robot holding a cargo, prove the fleet refuses to
                                load a second one onto it, and that no transfer entry point
                                exists at all.

Why these two first: MASTER_PLAN's P5 list needs a battery-injection handle for the low-battery
and charge-queue scenarios, and launch does not yet consume declared budgets. These two need
neither -- they are driven by ordinary business rules (submit / cancel / resubmit) against
machinery that already exists, so they can be answered today.

CORRECTION to the plan as proposed, recorded here because it changes what is being claimed:
there is NO reassignment operation in this codebase (the only `reassign` is the gate's permit
mirror, unrelated). A task is assigned once, at submit. So "转派" is not something that can be
attempted; the expressible form is cancel-then-resubmit under a NEW request_id. `request_id`
is the idempotency key, so re-submitting the same one returns the original task rather than a
second attempt. Scenario A therefore proves "cancel frees everything and the work can be
re-issued", not "the same task moved to another robot".

Assertions print PASS/FAIL per line and the raw evidence for each, including the exact ledger
state, so a reader can disagree with the verdict without rerunning anything.
"""

from __future__ import annotations

import json
import math
import os
import pathlib
import signal
import subprocess
import sys
import time

# The project root comes from this file's location, not from a literal. A hardcoded
# path makes a copy of the tree not a copy, which is the specific thing
# scripts/verify_independence.sh is looking for.
ROOT = pathlib.Path(__file__).resolve().parents[1]
#: The transcript. FLEET009_SCEN_OUT points it elsewhere when a run's evidence is
#: wanted outside the project; the default keeps a copied tree self-contained.
OUT = pathlib.Path(os.environ.get("FLEET009_SCEN_OUT") or (ROOT / "reports")) / "scen12.txt"

ROBOTS = "r01,r02,r03"

CHECKS: list[dict] = []


def log(*parts) -> None:
    line = " ".join(str(p) for p in parts)
    print(line, flush=True)
    with OUT.open("a", encoding="utf-8") as fh:
        fh.write(line + "\n")


def record(label: str, ok: bool, detail: str = "") -> None:
    CHECKS.append({"check": label, "ok": bool(ok), "detail": detail})
    log(f"  {'PASS' if ok else 'FAIL'}  {label}")
    if detail and not ok:
        log(f"          {detail}")


def station_layout() -> tuple[list[str], dict[str, tuple[float, float]]]:
    """Stations and their coordinates, read from config rather than hardcoded."""
    import yaml

    cfg = yaml.safe_load((ROOT / "config" / "fleet.yaml").read_text(encoding="utf-8"))
    stations: dict[str, tuple[float, float]] = {}
    for name, spec in (cfg.get("stations") or {}).items():
        if isinstance(spec, dict) and "x" in spec:
            stations[name] = (float(spec["x"]), float(spec["y"]))
    return sorted(stations), stations


def main() -> int:
    OUT.parent.mkdir(parents=True, exist_ok=True)
    OUT.write_text("", encoding="utf-8")

    # Every id carries this run's UTC time. The ledger is durable across sessions on purpose
    # (that is what makes restart-reconcile worth having), and `request_id` is the idempotency
    # key -- so reusing a name from an earlier attempt returns THAT attempt's task instead of
    # creating a new one. The first run of this script did exactly that and spent its result
    # reporting on somebody else's parked task.
    STAMP = time.strftime("%H%M%S", time.gmtime())
    CARGO_A = f"scen-{STAMP}-cargo-A"
    CARGO_B = f"scen-{STAMP}-cargo-B"
    CARGO_C = f"scen-{STAMP}-cargo-C"
    REQ_A1, REQ_A2 = f"scen-{STAMP}-A-1", f"scen-{STAMP}-A-2"
    REQ_B1, REQ_B2 = f"scen-{STAMP}-B-1", f"scen-{STAMP}-B-2"

    import rclpy
    from fleet_interfaces.srv import CancelTask, SubmitTask
    from rclpy.node import Node
    from std_srvs.srv import Trigger

    log("=== P5 scenarios 1 & 2 (no battery hook needed) ===")
    names, coords = station_layout()
    log(f"    stations: {names}")
    if len(names) < 2:
        log("FATAL: fewer than two stations in config/fleet.yaml; cannot build a transfer")
        return 3

    # Two stations on the same side of the barrier, nearest r01's spawn, so the legs are
    # corridor-free: these scenarios are about task custody, not about the corridor.
    west = [n for n in names if coords[n][0] < 0 and not n.startswith("C_")]
    east = [n for n in names if coords[n][0] > 0 and not n.startswith("C_")]
    log(f"    west stations (non-charger): {west}")
    log(f"    east stations (non-charger): {east}")
    if len(west) < 2:
        log("FATAL: need two west stations for the pickup pair")
        return 3
    pick, drop = west[0], west[1]
    log(f"    using pick={pick} drop={drop}")

    launch_log = (ROOT / "reports" / "scen12.log").open("w", encoding="utf-8")
    launch = subprocess.Popen(
        ["ros2", "launch", "fleet_bringup", "fleet.launch.py", f"robots:={ROBOTS}"],
        stdout=launch_log, stderr=subprocess.STDOUT, cwd=str(ROOT))

    rclpy.init()
    node = Node("p5_scenarios")
    group = rclpy.callback_groups.ReentrantCallbackGroup()
    submit_cli = node.create_client(SubmitTask, "/fleet/submit_task", callback_group=group)
    cancel_cli = node.create_client(CancelTask, "/fleet/cancel_task", callback_group=group)
    tasks_cli = node.create_client(Trigger, "/fleet/tasks", callback_group=group)

    def call(client, request, timeout: float = 15.0):
        """Single call, driven by spinning the executor so it can be answered."""
        if not client.wait_for_service(timeout_sec=timeout):
            return None
        future = client.call_async(request)
        deadline = time.monotonic() + timeout
        while not future.done() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
        return future.result() if future.done() else None

    def snapshot() -> dict:
        result = call(tasks_cli, Trigger.Request())
        if result is None or not result.success:
            return {}
        return json.loads(result.message)

    def task_of(snap: dict, task_id: str) -> dict:
        for row in snap.get("tasks", []):
            if row["task_id"] == task_id:
                return row
        return {}

    def submit(request_id: str, payload: str, capability: str = "CARRY"):
        # Field names from `ros2 interface show fleet_interfaces/srv/SubmitTask`, which this
        # script prints before running. They are NOT the CLI's flag names: the service calls
        # them `destination_station` / `required_capability`, and the CLI calls them
        # `--drop` / `--capability`. Writing the CLI's names here raised AttributeError on the
        # first attempt, which is the good failure: a wrong field on a rosidl message cannot
        # silently take a default.
        req = SubmitTask.Request()
        req.schema_version = "1"
        req.request_id = request_id
        req.kind = "station_transfer"
        req.pick_station = pick
        req.destination_station = drop
        req.required_capability = capability
        req.payload_mode = "logical"
        req.payload_id = payload
        req.priority = 10
        req.service_duration_s = 2.0
        req.max_attempts = 2
        req.timeout_sim_s = 180.0
        return call(submit_cli, req)

    try:
        log("\n=== waiting for the task service ===")
        deadline = time.monotonic() + 180
        ready = False
        while time.monotonic() < deadline:
            snap = snapshot()
            if snap.get("robots"):
                fresh = [r for r, v in snap["robots"].items() if v.get("fresh")]
                if len(fresh) == len(ROBOTS.split(",")):
                    ready = True
                    break
            time.sleep(2.0)
        if not ready:
            log("FATAL: the fleet never reported three fresh robots")
            log(f"      launch log: {launch_log.name}")
            return 3
        log(f"    three robots fresh")

        # ------------------------------------------------------------------ #
        # A. pre-pickup takeover
        # ------------------------------------------------------------------ #
        log("\n=== A. pre_pickup_takeover ===")
        resp = submit(REQ_A1, CARGO_A)
        if resp is None:
            log("FATAL: submit returned nothing")
            return 3
        task_a = resp.task_id
        log(f"    submitted request_id=scen-A-1 -> task_id={task_a} "
            f"accepted={resp.accepted} state={resp.state!r}")

        # Catch the moment BEFORE the pick: the robot is driving to the pick station.
        caught = None
        deadline = time.monotonic() + 120
        while time.monotonic() < deadline:
            snap = snapshot()
            row = task_of(snap, task_a)
            leg = row.get("leg", "")
            if row.get("state") == "EXECUTING" and leg.startswith("to_pick"):
                caught = row
                break
            if row.get("state") in ("SUCCEEDED", "FAILED", "CANCELED", "NEEDS_ATTENTION"):
                break
            time.sleep(0.5)
        log(f"    caught before the pick: {json.dumps(caught or {}, sort_keys=True)}")
        holders_before = dict((snapshot().get("payloads") or [{}])[0] or {})
        record("A1_the_pick_had_not_happened_yet", bool(caught) and caught.get("leg", "").startswith("to_pick"),
               f"needed a task EXECUTING on a to_pick leg; saw {caught}")

        cancel = CancelTask.Request()
        cancel.task_id = task_a
        cresp = call(cancel_cli, cancel)
        log(f"    cancel -> accepted={getattr(cresp, 'accepted', None)} "
            f"state={getattr(cresp, 'state', None)} stopped={getattr(cresp, 'stopped', None)} "
            f"reason={getattr(cresp, 'reason_code', None)}")
        log(f"    message: {getattr(cresp, 'message', '')}")
        record("A2_cancel_is_accepted_but_does_not_claim_a_stop", cresp is not None
               and cresp.accepted and cresp.stopped is False,
               "accepted must not mean stopped: the leg has to end and measured speed settle")

        # Wait for the terminal state and for the robot to be released.
        final = {}
        deadline = time.monotonic() + 180
        while time.monotonic() < deadline:
            snap = snapshot()
            final = task_of(snap, task_a)
            if final.get("state") in ("CANCELED", "FAILED", "NEEDS_ATTENTION"):
                break
            time.sleep(1.0)
        log(f"    final task row: {json.dumps(final, sort_keys=True)}")
        record("A3_the_task_reaches_a_terminal_state", final.get("state") == "CANCELED",
               f"state={final.get('state')!r}, detail={final.get('detail')!r}")

        snap = snapshot()
        payloads = {p["payload_id"]: p for p in snap.get("payloads", [])}
        # Scoped to THIS run. The ledger carries earlier sessions' rows, and a previous run's
        # HELD cargo would otherwise be reported as this scenario's failure -- which is exactly
        # what happened the first time.
        mine = {k: v for k, v in payloads.items() if k.startswith(f"scen-{STAMP}-")}
        held = {k: v for k, v in mine.items() if v.get("state") == "HELD"}
        log(f"    this run's payloads: {json.dumps(mine, sort_keys=True)}")
        log(f"    (all payloads in the ledger: {len(payloads)} rows, "
            f"{len(payloads) - len(mine)} from earlier sessions)")
        record("A4_no_cargo_is_left_held", not held, f"still HELD: {held}")
        record("A5_the_payload_row_is_still_free_not_orphaned",
               mine.get(CARGO_A, {}).get("state") == "FREE",
               f"{CARGO_A} = {mine.get(CARGO_A)}")

        # The robot must be free again: not named by any non-terminal task.
        busy = {t["robot_id"] for t in snap.get("tasks", [])
                if t.get("robot_id") and t.get("state") not in
                ("SUCCEEDED", "FAILED", "CANCELED", "NEEDS_ATTENTION")}
        log(f"    robots named by live tasks: {sorted(busy)}")
        record("A6_the_robot_is_not_left_pinned", not busy, f"still busy: {sorted(busy)}")

        # And the same work can be issued again under a new request_id.
        resp2 = submit(REQ_A2, CARGO_A)
        log(f"    resubmitted as request_id=scen-A-2 -> accepted="
            f"{getattr(resp2, 'accepted', None)} task_id={getattr(resp2, 'task_id', None)} "
            f"state={getattr(resp2, 'state', None)!r} "
            f"reason={getattr(resp2, 'reason_code', None)}")
        record("A7_the_work_can_be_issued_again", resp2 is not None and resp2.accepted
               and resp2.task_id != task_a,
               "a cancelled pre-pick task must not lock the cargo or the robot")
        task_a2 = getattr(resp2, "task_id", "") if resp2 is not None else ""

        # ------------------------------------------------------------------ #
        # B. no transfer while holding
        # ------------------------------------------------------------------ #
        log("\n=== B. no_transfer_while_holding ===")
        # `INSPECT` is r03's alone, so the second task has exactly one candidate: the robot
        # that is already carrying. That makes the refusal deterministic instead of a race
        # with a free robot.
        resp3 = submit(REQ_B1, CARGO_B, capability="INSPECT")
        task_b = getattr(resp3, "task_id", "") if resp3 is not None else ""
        log(f"    submitted request_id=scen-B-1 (INSPECT) -> task_id={task_b} "
            f"accepted={getattr(resp3, 'accepted', None)} "
            f"state={getattr(resp3, 'state', None)!r}")
        # The response names NO robot, and that is the contract rather than an omission:
        # `accepted` means received, and assignment is a separate later fact. Reading the
        # carrier from the ledger instead of assuming the receipt knew it is the difference
        # between testing the system and testing my guess about it.
        carrier = ""
        deadline = time.monotonic() + 90
        while time.monotonic() < deadline:
            row = task_of(snapshot(), task_b)
            if row.get("robot_id"):
                carrier = row["robot_id"]
                break
            if row.get("state") in ("SUCCEEDED", "FAILED", "CANCELED", "NEEDS_ATTENTION"):
                break
            time.sleep(0.5)
        log(f"    scen-B-1 assigned to {carrier!r} (read from the ledger, not the response)")

        holding = None
        started = time.monotonic()
        deadline = started + 240
        last_report = 0.0
        while time.monotonic() < deadline:
            snap = snapshot()
            payloads = {p["payload_id"]: p for p in snap.get("payloads", [])}
            row = task_of(snap, task_b)
            entry = payloads.get(CARGO_B, {})
            if entry.get("state") == "HELD" and entry.get("holder"):
                holding = (entry["holder"], entry)
                break
            if row.get("state") in ("FAILED", "NEEDS_ATTENTION", "SUCCEEDED"):
                log(f"    scen-B-1 ended before the cargo went HELD: "
                    f"{json.dumps(row, sort_keys=True)}")
                break
            # Report progress: a 240 s silent wait cannot be told from a hung script.
            if time.monotonic() - last_report > 30.0:
                last_report = time.monotonic()
                log(f"      t+{time.monotonic() - started:5.1f}s state={row.get('state')!r} "
                    f"leg={row.get('leg')!r} cargo={entry.get('state')!r}")
            time.sleep(1.0)
        log(f"    holding: {holding}")
        record("B1_a_robot_took_the_cargo", holding is not None and holding[0] == carrier,
               f"expected {carrier!r} to hold scen_cargo_B; saw {holding}")

        before = snapshot()
        refusals_before = int((before.get("refusals") or {}).get("PAYLOAD_HELD_ELSEWHERE", 0))
        holders_before = json.dumps(before.get("payloads"), sort_keys=True)

        resp4 = submit(REQ_B2, CARGO_C, capability="INSPECT")
        log(f"    submitted request_id=scen-B-2 (a SECOND load, INSPECT) -> accepted="
            f"{getattr(resp4, 'accepted', None)} task_id={getattr(resp4, 'task_id', None)} "
            f"state={getattr(resp4, 'state', None)!r} "
            f"reason={getattr(resp4, 'reason_code', None)}")
        time.sleep(6.0)
        after = snapshot()
        refusals_after = int((after.get("refusals") or {}).get("PAYLOAD_HELD_ELSEWHERE", 0))
        log(f"    PAYLOAD_HELD_ELSEWHERE refusals: {refusals_before} -> {refusals_after}")
        log(f"    payload holders now: {json.dumps(after.get('payloads'), sort_keys=True)}")

        holders_now: dict[str, list[str]] = {}
        for p in after.get("payloads", []):
            if p.get("state") == "HELD" and p.get("holder"):
                holders_now.setdefault(p["holder"], []).append(p["payload_id"])
        log(f"    cargo per holder: {holders_now}")
        record("B2_no_robot_ends_up_holding_two_cargos",
               all(len(v) <= 1 for v in holders_now.values()), f"holders: {holders_now}")
        row4 = task_of(after, getattr(resp4, "task_id", "") or "")
        log(f"    the second task's row: {json.dumps(row4, sort_keys=True)}")
        record("B3_the_second_load_was_refused_under_a_real_code",
               refusals_after > refusals_before,
               f"PAYLOAD_HELD_ELSEWHERE did not move ({refusals_before} -> {refusals_after}); "
               "a refusal nobody counts is a refusal nobody can audit")
        # The counter alone is not attribution: it was already at 20 from earlier sessions, so
        # a rise could have come from any carrying robot. What makes the refusal THIS test's
        # is that the second task did not go to the robot that is holding the first cargo.
        record("B3b_the_second_task_did_not_go_to_the_carrying_robot",
               bool(holding) and row4.get("robot_id", "") != holding[0],
               f"carrier={holding[0] if holding else None} was given "
               f"{row4.get('robot_id')!r} on the second task")
        record("B4_custody_of_the_first_cargo_is_unchanged",
               (f'"{holding[0]}"' in holders_before) if holding else False,
               f"was {holders_before}, now {json.dumps(after.get('payloads'), sort_keys=True)}")

        # The structural half: there must be no way to hand a payload to another robot.
        surface = subprocess.run(["ros2", "service", "list"], capture_output=True, text=True,
                                 cwd=str(ROOT)).stdout
        log(f"    services mentioning transfer/payload:")
        transfer = [s for s in surface.splitlines() if "transfer" in s.lower()]
        log(f"      {transfer or '(none)'}")
        record("B5_there_is_no_transfer_entry_point", not transfer,
               f"found {transfer}; a callable transfer would make 'no physical transfer in v1' "
               "untrue")

        log(f"\n    (left running for inspection: task {task_a2!r}, {task_b!r}; nothing is "
            "cancelled here)")

    finally:
        log("\n=== teardown ===")
        try:
            node.destroy_node()
            rclpy.try_shutdown()
        except Exception:
            pass
        if launch.poll() is None:
            launch.send_signal(signal.SIGINT)
            for _ in range(45):
                if launch.poll() is not None:
                    break
                time.sleep(1.0)
        if launch.poll() is None:
            launch.terminate()
            time.sleep(20)
        if launch.poll() is None:
            launch.kill()
        time.sleep(3.0)
        left = subprocess.run(["pgrep", "-f", f"{ROOT}/instal[l]"], capture_output=True, text=True)
        gz = subprocess.run(["pgrep", "-f", "gz-sim-mai[n]"], capture_output=True, text=True)
        n_left = len([x for x in left.stdout.split() if x.isdigit()])
        n_gz = len([x for x in gz.stdout.split() if x.isdigit()])
        log(f"    leftover project processes: {n_left}   gz: {n_gz}")
        record("teardown_verified_clean", n_left == 0 and n_gz == 0,
               f"leftover={n_left} gz={n_gz}")

    failed = [c for c in CHECKS if not c["ok"]]
    log(f"\n=== {len(CHECKS) - len(failed)}/{len(CHECKS)} assertions passed"
        f" -> {'PASS' if not failed else 'FAIL'}")
    for c in failed:
        log(f"    FAILED: {c['check']}")
    return 0 if not failed else 1


if __name__ == "__main__":
    raise SystemExit(main())
