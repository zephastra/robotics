#!/usr/bin/env python3
"""P5 directed scenarios 3-6, the four that need the launch to consume budgets.

  C. capability_filtering   -- an INSPECT task has exactly one eligible robot, and the
                               other two are never candidates for it.
  D. low_battery_admission  -- a robot under the low threshold is not given ordinary
                               work and is sent to charge instead.
  E. charge_queue           -- three low robots, two pads: the third queues, and no pad
                               is ever occupied by two robots.
  F. restart_recovery       -- the single writer is killed mid-task; the task is parked
                               as NEEDS_ATTENTION with a STATED reason, nothing is
                               silently resumed, and the fleet takes new work.

Run one case per process, because each needs its own boot: D and E put robots into the
low band through `start_battery_fraction`, and C needs every robot healthy.

THREE THINGS THIS FILE LEARNED THE HARD WAY (version 1 got all three wrong):

  1. **Readiness is not "the state topic is publishing".** Version 1 waited for three
     fresh `RobotState` messages and then submitted. r03's `bt_navigator` comes up
     about seven seconds after r01's, and a goal sent into that window is *rejected*
     (the action server exists, the lifecycle node is not active yet), so the first leg
     burned both retries and the task failed with NAV_FAILED. The fleet was fine; the
     instrument started too early. This version waits for the Nav2 lifecycle state to
     be ACTIVE on every robot.
  2. **A canned failure explanation printed as evidence reads as a failure.** Version 1
     printed the same detail string whether a check passed or failed, so a PASS next to
     "neither counter moved" was indistinguishable from a FAIL. `record()` now takes
     evidence (always printed, factual) and `why` (printed only on failure).
  3. **A durable ledger is a shared resource.** Version 1 reused `runtime/fleet.sqlite`
     and inherited two payloads left HELD by earlier sessions, which made every robot
     that had ever carried something ineligible. The run reported a capability failure
     that was really a contaminated environment. Each case now gets its own ledger, and
     the boot state is asserted.

Every assertion prints the raw evidence that decided it, so a reader can disagree with
the verdict without re-running anything.
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

ROOT = pathlib.Path(__file__).resolve().parents[1]
OUT_DIR = pathlib.Path(os.environ.get("FLEET009_SCEN_OUT") or (ROOT / "reports" / "scen3456"))
ROBOTS = "r01,r02,r03"
CHECKS: list[dict] = []
LOG_FH = None

#: Pads are 10 m apart, so "the same pad" is unambiguous at this radius. It is the
#: footprint half-diagonal (0.375 m from P2's measured numbers) plus a localiser margin.
PAD_RADIUS_M = 1.2

#: Nav2 lifecycle state ids (lifecycle_msgs/msg/State).
ACTIVE = 3

#: Per-case runtime paths, filled in by main(). `case_restart` reuses the same ones so
#: that its restart genuinely recovers the same ledger rather than starting a new one.
RUNTIME_ARGS: "dict[str, dict]" = {}


def log(*parts) -> None:
    line = " ".join(str(p) for p in parts)
    print(line, flush=True)
    if LOG_FH is not None:
        LOG_FH.write(line + "\n")
        LOG_FH.flush()


def record(label: str, ok: bool, evidence: str = "", why: str = "") -> None:
    """One assertion. `evidence` is factual and always printed; `why` only on failure.

    Keeping them apart is the point: a detail string written as a failure explanation
    is a lie when the check passes, and it reads as one.
    """
    CHECKS.append({"check": label, "ok": bool(ok), "evidence": evidence, "why": why})
    log(f"  {'PASS' if ok else 'FAIL'}  {label}")
    if evidence:
        log(f"          {evidence}")
    if not ok and why:
        log(f"          why it matters: {why}")


def dump(label: str, snap: dict) -> None:
    """Print the parts of a snapshot a verdict actually rests on."""
    log(f"    --- {label} ---")
    robots = snap.get("robots") or {}
    for rid in sorted(robots):
        v = robots[rid]
        log(f"      {rid}: band={v.get('band')} wh={v.get('battery_wh')} "
            f"frac={v.get('battery_fraction')} op={v.get('operating_state')} "
            f"task={v.get('task_id')!r} fresh={v.get('fresh')} "
            f"at=({v.get('x')},{v.get('y')})")
    for t in snap.get("tasks") or []:
        log(f"      task {t.get('task_id')}: kind={t.get('kind')} "
            f"state={t.get('state')} robot={t.get('robot_id')!r} "
            f"pinned={t.get('pinned_robot')!r} leg={t.get('leg')!r} "
            f"attempts={t.get('attempts')} reason={t.get('reason')} "
            f"detail={t.get('detail')!r}")
    held = [p for p in (snap.get("payloads") or []) if p.get("state") == "HELD"]
    log(f"      payloads HELD: {held}")
    ch = snap.get("chargers") or {}
    log(f"      chargers: occupant={ch.get('occupant')} queue={ch.get('queue')} "
        f"grants={ch.get('grants')} releases={ch.get('releases')} "
        f"capacity={ch.get('capacity_per_charger')}")
    log(f"      refusals: {json.dumps(snap.get('refusals') or {}, sort_keys=True)}")
    log(f"      tick_errors={snap.get('tick_errors')} "
        f"stale_results={snap.get('stale_results')} "
        f"ownership_refusals={snap.get('ownership_refusals')} "
        f"orphaned={snap.get('orphaned_tasks')} epoch={snap.get('epoch')}")


def load_config() -> dict:
    import yaml

    return yaml.safe_load((ROOT / "config" / "fleet.yaml").read_text(encoding="utf-8"))


def main(argv) -> int:
    global LOG_FH
    cases = ("capability", "low_battery", "charge_queue", "restart")
    if len(argv) != 2 or argv[1] not in cases:
        print("usage: run_scenarios3456.py {capability|low_battery|charge_queue|restart}",
              file=sys.stderr)
        return 2
    case = argv[1]
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    stamp = time.strftime("%Y%m%dT%H%M%SZ", time.gmtime())
    LOG_FH = (OUT_DIR / f"{case}_{stamp}.txt").open("w", encoding="utf-8")

    log(f"=== P5 scenario {case!r}  run {stamp} ===")
    log(f"    backend: gazebo_nav2     host: {os.uname().nodename}")
    log(f"    scenario file: config/scenarios/p5_{case}.yaml")
    log("    instrument version: 2 (Nav2-lifecycle readiness, separate evidence/why)")

    import rclpy
    from fleet_interfaces.srv import SubmitTask
    from lifecycle_msgs.srv import GetState
    from rclpy.node import Node
    from std_srvs.srv import Trigger

    cfg = load_config()
    stations = {n: (float(s["x"]), float(s["y"])) for n, s in cfg["stations"].items()
                if isinstance(s, dict) and "x" in s}
    west = sorted(n for n, (x, _y) in stations.items() if x < 0 and not n.startswith("C_"))
    chargers = {n: stations[n] for n in cfg.get("chargers", {}) if n in stations}
    log(f"    west stations: {west}")
    log(f"    chargers: {chargers}  capacity/charger={cfg.get('charger_capacity')}")
    if len(west) < 2:
        log("FATAL: need two west stations")
        return 3
    pick, drop = west[0], west[1]

    run_id = time.strftime("%H%M%S", time.gmtime())
    req = lambda tag: f"sc3456-{run_id}-{case}-{tag}"  # noqa: E731
    cargo = lambda tag: f"sc3456-{run_id}-{case}-{tag}"  # noqa: E731

    runtime = ROOT / "runtime" / "scen3456"
    runtime.mkdir(parents=True, exist_ok=True)
    # Stamped with the RUN, not just the case. Isolating cases from each other is not
    # enough: a second attempt at the same case inherits the first attempt's rows, and a
    # leftover charge row with the same generated request_id makes the new charge run
    # impossible to create -- which reads as a scheduling failure and is leftover state.
    RUNTIME_ARGS[case] = {
        "db_path": runtime / f"{case}_{stamp}.sqlite",
        "events_path": runtime / f"{case}_{stamp}.jsonl",
        "robot_state_dir": runtime / f"{case}_{stamp}_state",
    }
    launch_log = (ROOT / "reports" / f"scen3456_{case}.log").open("w", encoding="utf-8")
    launch = subprocess.Popen(
        ["ros2", "launch", "fleet_bringup", "fleet.launch.py",
         f"robots:={ROBOTS}", f"scenario:=p5_{case}",
         f"db_path:={RUNTIME_ARGS[case]['db_path']}",
         f"events_path:={RUNTIME_ARGS[case]['events_path']}",
         f"robot_state_dir:={RUNTIME_ARGS[case]['robot_state_dir']}"],
        stdout=launch_log, stderr=subprocess.STDOUT, cwd=str(ROOT))

    rclpy.init()
    node = Node("p5_scenarios3456")
    group = rclpy.callback_groups.ReentrantCallbackGroup()
    submit_cli = node.create_client(SubmitTask, "/fleet/submit_task", callback_group=group)
    tasks_cli = node.create_client(Trigger, "/fleet/tasks", callback_group=group)
    lifecycle_clis = {
        f"{rid}/{name}": node.create_client(
            GetState, f"/{rid}/{name}/get_state", callback_group=group)
        for rid in ROBOTS.split(",") for name in ("bt_navigator", "controller_server")
    }

    def call(client, request, timeout: float = 15.0):
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
        try:
            return json.loads(result.message)
        except (ValueError, TypeError):
            return {}

    def task_of(snap: dict, task_id: str) -> dict:
        for row in snap.get("tasks", []):
            if row.get("task_id") == task_id:
                return row
        return {}

    def submit(request_id: str, payload: str, capability: str = "CARRY") -> object:
        msg = SubmitTask.Request()
        msg.schema_version = "1"
        msg.request_id = request_id
        msg.kind = "station_transfer"
        msg.pick_station = pick
        msg.destination_station = drop
        msg.required_capability = capability
        msg.payload_mode = "logical"
        msg.payload_id = payload
        msg.priority = 10
        msg.service_duration_s = 2.0
        msg.max_attempts = 2
        msg.timeout_sim_s = 240.0
        return call(submit_cli, msg)

    def wait_for_fleet(deadline_s: float = 240.0) -> bool:
        """Wait until every robot reports fresh state AND its Nav2 nodes are ACTIVE.

        The second half is not optional. A goal sent to a bt_navigator that is up but
        not yet active is rejected, not queued, and the leg burns its retries.
        """
        end = time.monotonic() + deadline_s
        last = ""
        fresh_ok_at = None
        while time.monotonic() < end:
            snap = snapshot()
            fresh = [r for r, v in (snap.get("robots") or {}).items() if v.get("fresh")]
            if len(fresh) == len(ROBOTS.split(",")):
                if fresh_ok_at is None:
                    fresh_ok_at = time.monotonic()
                    log("    three robots fresh; waiting for Nav2 lifecycles")
                states = {}
                for key, client in lifecycle_clis.items():
                    resp = call(client, GetState.Request(), timeout=3.0)
                    states[key] = resp.current_state.id if resp is not None else None
                row = json.dumps(states, sort_keys=True)
                if row != last:
                    last = row
                    log(f"      lifecycles: {row}")
                if all(v == ACTIVE for v in states.values()):
                    log(f"    Nav2 active on all robots "
                        f"({time.monotonic() - fresh_ok_at:.1f}s after the state topic)")
                    return True
            else:
                row = json.dumps({r: v.get("fresh") for r, v in
                                  (snap.get("robots") or {}).items()}, sort_keys=True)
                if row != last:
                    last = row
                    log(f"      waiting: fresh={row}")
            time.sleep(2.0)
        log("FATAL: the fleet never became ready (fresh state AND active Nav2)")
        return False

    def teardown() -> None:
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
        left = subprocess.run(["pgrep", "-f", f"{ROOT}/instal[l]"],
                              capture_output=True, text=True)
        gz = subprocess.run(["pgrep", "-f", "gz-sim-mai[n]"], capture_output=True, text=True)
        n_left = len([x for x in left.stdout.split() if x.isdigit()])
        n_gz = len([x for x in gz.stdout.split() if x.isdigit()])
        log(f"    leftover project processes: {n_left}   gz: {n_gz}")
        record("teardown_verified_clean", n_left == 0 and n_gz == 0,
               f"leftover={n_left} gz={n_gz}",
               "a case that cannot get a clean world makes the NEXT case's numbers "
               "uninterpretable")

    try:
        if not wait_for_fleet():
            return 3

        boot = snapshot()
        boot_held = [p for p in (boot.get("payloads") or []) if p.get("state") == "HELD"]
        record("environment_ledger_has_no_stale_custody", not boot_held,
               f"payloads HELD at boot: {boot_held}",
               "a HELD payload survives a restart by design, so the robot named as its "
               "holder is refused for every later task that carries a payload. A run "
               "that inherits one reports a scheduling failure that is really a "
               "contaminated environment")
        boot_refusals = boot.get("refusals") or {}
        boot_tick_errors = int(boot.get("tick_errors") or 0)
        log(f"    boot refusals: {json.dumps(boot_refusals, sort_keys=True)}  "
            f"tick_errors={boot_tick_errors}")

        if case == "capability":
            case_capability(snapshot, task_of, submit, req, cargo, cfg, boot_refusals)
        elif case == "low_battery":
            case_low_battery(snapshot, task_of, submit, req, cargo, boot_refusals)
        elif case == "charge_queue":
            case_charge_queue(snapshot, submit, req, chargers)
        elif case == "restart":
            case_restart(snapshot, task_of, submit, req, cargo, launch_log)
    finally:
        teardown()

    failed = [c for c in CHECKS if not c["ok"]]
    log(f"\n=== {len(CHECKS) - len(failed)}/{len(CHECKS)} assertions passed"
        f" -> {'PASS' if not failed else 'FAIL'}")
    for c in failed:
        log(f"    FAILED: {c['check']}")
    return 0 if not failed else 1


def _delta(after: dict, before: dict) -> str:
    """Render a refusal-counter delta, with the unit stated.

    These counters advance once per TICK, not once per distinct refusal: one task that
    no robot can serve at 2 Hz produces ~90 increments in 45 s. Read without that, "91
    refusals" sounds like 91 separate events. The unit is printed so it does not have to
    be remembered.
    """
    keys = sorted(set(after) | set(before))
    moved = ", ".join(f"{k}: {int(before.get(k, 0))} -> {int(after.get(k, 0))}"
                      for k in keys
                      if int(before.get(k, 0)) != int(after.get(k, 0)))
    return (moved + "  [refusal EVALUATIONS, one per tick, not distinct events]"
            if moved else "(no counter moved)")


# --------------------------------------------------------------------------- #
# C. capability filtering
# --------------------------------------------------------------------------- #


def case_capability(snapshot, task_of, submit, req, cargo, cfg, boot_refusals) -> None:
    log("\n=== C. capability_filtering ===")
    inspect_robots = sorted(r for r, spec in cfg["robots"].items()
                            if "INSPECT" in (spec.get("capabilities") or []))
    log(f"    robots declaring INSPECT in config: {inspect_robots}")
    record("C0_exactly_one_robot_declares_the_capability", len(inspect_robots) == 1,
           f"capable robots in config: {inspect_robots}",
           "if two robots had the capability, one assignment would not show the filter works")

    resp = submit(req("C-1"), cargo("C-1"), capability="INSPECT")
    if resp is None:
        record("C1_the_inspect_task_is_accepted", False, "submit returned nothing")
        return
    task_a = resp.task_id
    log(f"    submitted request_id={req('C-1')} -> task_id={task_a} "
        f"accepted={resp.accepted} state={resp.state!r}")

    seen: list[str] = []
    assigned = ""
    end = time.monotonic() + 120
    while time.monotonic() < end:
        row = task_of(snapshot(), task_a)
        rid = row.get("robot_id") or ""
        if rid and rid not in seen:
            seen.append(rid)
        if rid:
            assigned = rid
            break
        if row.get("state") in ("FAILED", "NEEDS_ATTENTION", "CANCELED"):
            break
        time.sleep(0.5)
    record("C1_the_inspect_task_is_accepted", bool(resp.accepted),
           f"accepted={resp.accepted} state={resp.state!r}")
    record("C2_the_task_went_to_the_only_capable_robot", assigned == inspect_robots[0]
           if inspect_robots else False,
           f"assigned={assigned!r}, capable={inspect_robots}")
    record("C3_no_incapable_robot_was_ever_named", all(r in inspect_robots for r in seen),
           f"robots ever named for this task: {seen}")

    # Occupy the only capable robot, then ask for a second INSPECT task. Nothing can
    # serve it while r03 is busy, and the point is that neither of the other two is
    # quietly used instead.
    before = snapshot().get("refusals") or {}
    resp2 = submit(req("C-2"), cargo("C-2"), capability="INSPECT")
    task_b = getattr(resp2, "task_id", "") if resp2 is not None else ""
    wrong: list[str] = []
    # Sample the counters while the second task waits. The refusal changes cause during this
    # window -- the capable robot is first busy and then holding -- and one end-of-window delta
    # cannot tell those apart. This is the D-P5-20 question asked properly: not "which counter
    # moved" but "did the label keep up with the cause".
    phases: list[dict] = []
    end = time.monotonic() + 45
    while time.monotonic() < end:
        snap_now = snapshot()
        row = task_of(snap_now, task_b)
        rid = row.get("robot_id") or ""
        if rid and rid not in inspect_robots and rid not in wrong:
            wrong.append(rid)
        phases.append({
            "t": round(time.monotonic() - (end - 45), 1),
            "RESOURCE_BUSY": int((snap_now.get("refusals") or {}).get("RESOURCE_BUSY", 0)),
            "NO_CAPABLE_ROBOT": int((snap_now.get("refusals") or {}).get("NO_CAPABLE_ROBOT", 0)),
            "PAYLOAD_HELD_ELSEWHERE": int(
                (snap_now.get("refusals") or {}).get("PAYLOAD_HELD_ELSEWHERE", 0)),
            "carrying": bool((snap_now.get("payloads") or [])
                             and any(p.get("state") == "HELD" for p in snap_now["payloads"])),
        })
        time.sleep(1.0)
    after = snapshot().get("refusals") or {}
    moved = _delta(after, before)
    first_busy = next((p for p in phases if p["RESOURCE_BUSY"] > int(before.get("RESOURCE_BUSY", 0))), None)
    first_custody = next((p for p in phases if p["PAYLOAD_HELD_ELSEWHERE"]
                          > int(before.get("PAYLOAD_HELD_ELSEWHERE", 0))), None)
    first_incapable = next((p for p in phases if p["NO_CAPABLE_ROBOT"]
                            > int(before.get("NO_CAPABLE_ROBOT", 0))), None)
    log(f"    phase order: busy at {first_busy['t'] if first_busy else None}s, "
        f"custody at {first_custody['t'] if first_custody else None}s, "
        f"no-capable at {first_incapable['t'] if first_incapable else None}s "
        f"(carrying={[p['carrying'] for p in phases]})")
    log(f"    second INSPECT task: task_id={task_b} "
        f"accepted={getattr(resp2, 'accepted', None)}; refusals {moved}")
    record("C4_the_second_task_never_went_to_an_incapable_robot", not wrong,
           f"incapable robots seen on the second task: {wrong}")
    # D-P5-20, checked live. The old assertion here accepted NO_CAPABLE_ROBOT and therefore
    # passed for a whole round while the only capable robot was merely `RESOURCE_BUSY` -- the
    # instrument had the defect written into it as expected behaviour.
    busy = int(after.get("RESOURCE_BUSY", 0)) - int(before.get("RESOURCE_BUSY", 0))
    incapable = int(after.get("NO_CAPABLE_ROBOT", 0)) - int(before.get("NO_CAPABLE_ROBOT", 0))
    record("C5_the_refusal_names_the_cause_that_applied", busy > 0,
           f"RESOURCE_BUSY delta={busy}; refusal counters: {moved}",
           "the capable robot is BUSY, not missing. Reporting NO_CAPABLE_ROBOT here is "
           "D-P5-20: the decision was right and the label sent an operator to the capability "
           "table")
    # The label must follow the cause. "A capable robot exists and is occupied" may not be
    # reported as "nobody has the capability"; once the capable robot is HOLDING a payload the
    # custody rule (D-P5-14) takes it out of the candidate set, and then NO_CAPABLE_ROBOT is the
    # honest answer -- the two phases must not be blamed on each other.
    custody_first = (first_custody is not None and first_incapable is not None
                     and first_custody["t"] <= first_incapable["t"])
    record("C6_the_label_follows_the_cause",
           first_busy is not None and (first_incapable is None or custody_first),
           f"first RESOURCE_BUSY at {first_busy['t'] if first_busy else None}s; first "
           f"PAYLOAD_HELD_ELSEWHERE at {first_custody['t'] if first_custody else None}s; first "
           f"NO_CAPABLE_ROBOT at {first_incapable['t'] if first_incapable else None}s; "
           f"NO_CAPABLE_ROBOT delta={incapable} after {busy} busy refusal(s)",
           "the refusal must name the cause that applies AT THE TIME. Reporting a busy robot as "
           "capability-less was D-P5-20; reporting a payload-holding robot as merely busy would "
           "be the same defect with the two labels swapped")
    dump("capability, end of case", snapshot())


# --------------------------------------------------------------------------- #
# D. low-battery admission
# --------------------------------------------------------------------------- #


def case_low_battery(snapshot, task_of, submit, req, cargo, boot_refusals) -> None:
    log("\n=== D. low_battery_admission ===")
    snap = snapshot()
    robots = snap.get("robots") or {}
    for rid in sorted(robots):
        log(f"    {rid}: wh={robots[rid].get('battery_wh')} "
            f"fraction={robots[rid].get('battery_fraction')} band={robots[rid].get('band')}")
    record("D1_r01_started_in_the_low_band", (robots.get("r01") or {}).get("band") == "LOW",
           f"r01 band={(robots.get('r01') or {}).get('band')!r} "
           f"fraction={(robots.get('r01') or {}).get('battery_fraction')}",
           "the injection hook has to reach the shipped BatteryModel, not bypass it")
    record("D2_the_other_two_stayed_healthy",
           all((robots.get(r) or {}).get("band") == "OK" for r in ("r02", "r03")),
           f"r02={robots.get('r02', {}).get('band')} r03={robots.get('r03', {}).get('band')}")

    # The fleet should generate a charge run without being asked. Any
    # return_to_charge task counts; whether it is pinned yet is a separate fact.
    charge_task = ""
    end = time.monotonic() + 90
    while time.monotonic() < end:
        rows = [t for t in (snapshot().get("tasks") or [])
                if t.get("kind") == "return_to_charge"]
        if rows:
            charge_task = rows[-1]["task_id"]
            log(f"    charge task: {json.dumps(rows[-1], sort_keys=True)}")
            break
        time.sleep(2.0)
    if not charge_task:
        dump("D3 failed: no charge task appeared", snapshot())
    record("D3_a_charge_run_was_generated_for_it", bool(charge_task),
           f"return_to_charge task ids: {charge_task!r}",
           "a robot that is under the low threshold and given no way to charge is the "
           "rule this scenario exists to check")

    before = snapshot().get("refusals") or {}
    resp = submit(req("D-1"), cargo("D-1"))
    task_id = getattr(resp, "task_id", "") if resp is not None else ""
    log(f"    submitted an ordinary CARRY task -> task_id={task_id} "
        f"accepted={getattr(resp, 'accepted', None)}")
    samples: list[tuple[str, str, str]] = []
    charge_states: list[str] = []
    r01_fractions: list[float] = []
    end = time.monotonic() + 90
    while time.monotonic() < end:
        snap = snapshot()
        row = task_of(snap, task_id)
        band = ((snap.get("robots") or {}).get("r01") or {}).get("band", "")
        samples.append((row.get("robot_id") or "", band, row.get("state") or ""))
        if charge_task:
            charge_states.append(task_of(snap, charge_task).get("state") or "")
        frac = ((snap.get("robots") or {}).get("r01") or {}).get("battery_fraction")
        if frac is not None:
            r01_fractions.append(float(frac))
        # Stop early once the charge run has finished and a clean reading is in hand;
        # the assertions below are about the window, so a longer wait adds nothing.
        if charge_states and charge_states[-1] in ("SUCCEEDED", "FAILED", "NEEDS_ATTENTION"):
            break
        time.sleep(2.0)
    after = snapshot().get("refusals") or {}
    bad = sorted({s for s in samples if s[0] == "r01" and s[1] == "LOW"})
    record("D4_it_was_never_given_the_task_while_low", not bad,
           f"samples where r01 was named AND low: {bad}; "
           f"{len(samples)} samples taken, e.g. {samples[:6]}",
           "below the threshold the next thing this robot does is charge, not carry")
    record("D5_the_refusal_was_counted",
           int(after.get("INSUFFICIENT_BATTERY", 0))
           > int(before.get("INSUFFICIENT_BATTERY", 0)),
           f"refusal counters: {_delta(after, before)}")

    final = snapshot()
    observed = sorted(set(charge_states))
    record("D6_r01_executed_the_charge_run",
           any(s in ("EXECUTING", "SUCCEEDED") for s in observed),
           f"states observed for {charge_task!r} over the window: {observed}",
           "asking this at one instant is how a finished charge run reads as a missing "
           "one: by the end r01 was already back to IDLE above the resume threshold")
    rose = (max(r01_fractions) - min(r01_fractions)) if len(r01_fractions) >= 2 else 0.0
    record("D7_r01_actually_charged", rose > 0.10,
           f"r01 battery fraction {min(r01_fractions) if r01_fractions else None} -> "
           f"{max(r01_fractions) if r01_fractions else None} (rise {rose:.3f}); "
           f"{len(r01_fractions)} samples; pad grants="
           f"{(final.get('chargers') or {}).get('grants')}",
           "a robot that reaches a pad and does not charge is the failure D-P5-19 hid")
    dump("low battery, end of case", final)


# --------------------------------------------------------------------------- #
# E. charge-pad capacity and queueing
# --------------------------------------------------------------------------- #


def case_charge_queue(snapshot, submit, req, chargers) -> None:
    log("\n=== E. charge_queue ===")
    deadline = time.monotonic() + 330
    stacked: list[dict] = []
    over_capacity: list[dict] = []
    occupants_seen: list[dict] = []
    queue_seen: list[list] = []
    charge_states: list[str] = []
    moved_to_a_pad: set[str] = set()
    all_low = False
    while time.monotonic() < deadline:
        snap = snapshot()
        robots = snap.get("robots") or {}
        if len(robots) == 3 and all(v.get("band") == "LOW" for v in robots.values()):
            all_low = True
        ch = snap.get("chargers") or {}
        occupants = ch.get("occupant") or {}
        if occupants:
            occupants_seen.append(occupants)
        if len(occupants) > len(chargers):
            over_capacity.append(occupants)
        queue = [e.get("robot_id") for e in (ch.get("queue") or [])]
        if queue:
            queue_seen.append(queue)
        for t in (snap.get("tasks") or []):
            if t.get("kind") == "return_to_charge":
                charge_states.append(t.get("state") or "")
        # The grant counter is bookkeeping. A robot standing inside a pad's radius is the
        # claim the requirement actually makes, and it is the one that was missing when
        # `grants=2` sat next to three robots that never left their spawns.
        for rid, v in robots.items():
            for pad, pos in chargers.items():
                if v.get("x") is not None and math.dist((v["x"], v["y"]), pos) <= PAD_RADIUS_M:
                    moved_to_a_pad.add(rid)

        # The geometric half of "they do not stack": two robots inside the same pad's
        # radius. The allocator's bookkeeping cannot show this.
        for pad, pos in chargers.items():
            near = [rid for rid, v in robots.items()
                    if v.get("x") is not None and math.dist((v["x"], v["y"]), pos)
                    <= PAD_RADIUS_M]
            if len(near) > 1:
                stacked.append({"pad": pad, "robots": near})
        # Do not stop at the first queue reading. The assertions below are about a robot
        # ARRIVING at a pad, and an early break turned "not yet observed" into "never
        # happened" -- which is how the earlier run reported a granted pad as charging.
        if all_low and queue_seen and moved_to_a_pad:
            break
        time.sleep(2.0)

    log(f"    occupancy samples: {len(occupants_seen)}   queue samples: {len(queue_seen)}")
    log(f"    last occupancy: {occupants_seen[-1] if occupants_seen else None}")
    log(f"    first few queues: {queue_seen[:6]}")
    counts = [len(o) for o in occupants_seen]
    record("E1_all_three_robots_were_in_the_low_band", all_low,
           f"pads={len(chargers)}; occupancy samples={len(occupants_seen)}")
    record("E2_no_pad_was_ever_over_its_capacity", not over_capacity,
           f"samples whose occupant count exceeded the pad count: {over_capacity[:3]}",
           "one pad must never hold two robots: that is the pile-up the requirement "
           "forbids")
    record("E3_at_most_two_pads_were_occupied_at_once",
           all(c <= len(chargers) for c in counts),
           f"max simultaneous occupants: {max(counts) if counts else 0} of {len(chargers)}")
    record("E4_the_surplus_robot_waited_rather_than_being_sent", bool(queue_seen),
           f"queue snapshots: {len(queue_seen)}, e.g. {queue_seen[:4]}",
           "with three low robots and two pads the third has nowhere to go but the queue")
    record("E5_two_robots_were_never_inside_one_pad", not stacked,
           f"pads holding more than one robot at once: {stacked[:3]}",
           "the allocator refusing the second robot is a scheduling claim; this is the "
           "geometric one")
    snap = snapshot()
    ch = snap.get("chargers") or {}
    refusals = ch.get("refusals") or {}
    record("E6_a_pad_was_granted", bool(ch.get("grants")),
           f"grants={ch.get('grants')} releases={ch.get('releases')}",
           "a queue test in which nobody was ever admitted tests nothing about capacity")
    exec_states = sorted(set(charge_states))
    record("E8_a_charge_run_was_actually_executed",
           any(s in ("EXECUTING", "SUCCEEDED") for s in exec_states),
           f"states seen for return_to_charge tasks: {exec_states}",
           "a granted pad with no executing charge run is a pad held by a robot that has "
           "no task, which is exactly the leak a colliding request_id produced")
    record("E9_a_robot_physically_reached_a_pad", bool(moved_to_a_pad),
           f"robots observed within {PAD_RADIUS_M} m of a pad: {sorted(moved_to_a_pad)}; "
           f"final positions: "
           f"{ {r: (v.get('x'), v.get('y')) for r, v in (snap.get('robots') or {}).items()} }",
           "the allocator admitting a robot is a scheduling claim; the robot arriving is "
           "the physical one, and this project has already been caught reporting the "
           "first as if it were the second")
    record("E7_the_queue_refusal_is_auditable", int(refusals.get("CHARGER_BUSY", 0)) > 0,
           f"charger refusals: {json.dumps(refusals, sort_keys=True)}")
    dump("charge queue, end of case", snap)


# --------------------------------------------------------------------------- #
# F. restart of the single writer
# --------------------------------------------------------------------------- #


def _task_service_pids() -> list[int]:
    out = subprocess.run(["pgrep", "-f", "fleet_ros/lib/fleet_ros/task_service_node"],
                         capture_output=True, text=True)
    return sorted(int(x) for x in out.stdout.split() if x.isdigit())


def case_restart(snapshot, task_of, submit, req, cargo, launch_log) -> None:
    log("\n=== F. restart_recovery ===")
    resp = submit(req("F-1"), cargo("F-1"))
    if resp is None:
        record("F1_the_task_is_running_before_the_kill", False, "submit returned nothing")
        return
    task_id = resp.task_id
    log(f"    submitted request_id={req('F-1')} -> task_id={task_id}")

    running = {}
    end = time.monotonic() + 150
    while time.monotonic() < end:
        row = task_of(snapshot(), task_id)
        if row.get("state") == "EXECUTING" and row.get("robot_id"):
            running = row
            break
        if row.get("state") in ("FAILED", "NEEDS_ATTENTION", "CANCELED", "SUCCEEDED"):
            break
        time.sleep(1.0)
    record("F1_the_task_is_running_before_the_kill",
           running.get("state") == "EXECUTING" and bool(running.get("robot_id")),
           f"state={running.get('state')!r} robot={running.get('robot_id')!r}",
           "without a live task a restart proves nothing about recovery")

    before = snapshot()
    held_before = {p["payload_id"]: p.get("holder")
                   for p in before.get("payloads", [])
                   if p.get("state") == "HELD" and p["payload_id"] == cargo("F-1")}
    epoch_before = before.get("epoch")
    log(f"    epoch before kill: {epoch_before}; this task's custody: {held_before}")

    pids = _task_service_pids()
    log(f"    task service pids: {pids}")
    if not pids:
        record("F2_the_single_writer_is_actually_killed", False,
               "no task_service_node process found to kill")
        return
    for pid in pids:
        try:
            os.kill(pid, signal.SIGKILL)
        except ProcessLookupError:
            pass
    gone = False
    end = time.monotonic() + 60
    while time.monotonic() < end:
        if not snapshot():
            gone = True
            break
        time.sleep(1.0)
    record("F2_the_single_writer_is_actually_killed", gone,
           f"SIGKILL sent to {pids}; status service stopped answering: {gone}",
           "if the writer is still up, nothing below is a recovery test")

    paths = RUNTIME_ARGS.get("restart", {})
    restarted = subprocess.Popen(
        ["ros2", "run", "fleet_ros", "task_service_node", "--ros-args",
         "-p", f"robots:={ROBOTS}",
         "-p", "tick_hz:=2.0",
         "-p", "leg_timeout_s:=240.0",
         "-p", "cancel_confirm_s:=15.0",
         "-p", "orphan_grace_s:=45.0",
         "-p", "use_sim_time:=false",
         # The SAME ledger, deliberately: the claim is that it recovers the one it was
         # already writing, not that it can make a new one.
         "-p", f"db_path:={paths.get('db_path', 'runtime/fleet.sqlite')}",
         "-p", f"events_path:={paths.get('events_path', 'runtime/events.jsonl')}",
         "-p", f"robot_state_dir:={paths.get('robot_state_dir', 'runtime/robot_state')}"],
        stdout=launch_log, stderr=subprocess.STDOUT, cwd=str(ROOT))

    after = {}
    end = time.monotonic() + 90
    while time.monotonic() < end:
        snap = snapshot()
        if snap.get("robots"):
            after = snap
            break
        time.sleep(2.0)
    record("F3_the_writer_comes_back", bool(after),
           f"restarted pid={restarted.pid}; snapshot received={bool(after)}")
    if not after:
        return

    row = task_of(after, task_id)
    record("F4_the_task_is_parked_for_attention", row.get("state") == "NEEDS_ATTENTION",
           f"state={row.get('state')!r} detail={row.get('detail')!r}")
    record("F5_the_reason_names_the_restart",
           row.get("reason") == "INTERRUPTED_BY_RESTART",
           f"reason={row.get('reason')!r}",
           "a task parked by a restart and labelled as a cancel is the same defect as "
           "008's constant fail_reason")
    not_resumed = [e.get("task_id") for e in (after.get("reconcile") or {}).get("not_resumed", [])]
    record("F6_the_reconcile_report_names_it", task_id in not_resumed,
           f"{len(not_resumed)} task(s) in not_resumed; ours present={task_id in not_resumed}")
    record("F7_the_epoch_changed", after.get("epoch") != epoch_before,
           f"epoch {epoch_before} -> {after.get('epoch')}")
    # NOT `ledger_boot == ledger_boot`: `boot-{hash(path, sim_t, wall_t)}` includes wall
    # time, so it identifies the ledger INSTANCE and changes on every open. Asserting on
    # it reads as a database identity check and is not one. What actually shows the same
    # FILE was reopened is that rows written before the kill are still there.
    before_ids = {t.get("task_id") for t in before.get("tasks", [])}
    after_ids = {t.get("task_id") for t in after.get("tasks", [])}
    missing = sorted(before_ids - after_ids)
    record("F8_the_same_ledger_file_was_reopened", not missing,
           f"{len(before_ids)} task row(s) before, {len(after_ids)} after; missing: "
           f"{missing[:5]}  (instance id {before.get('ledger_boot')} -> "
           f"{after.get('ledger_boot')}, which is expected to change)",
           "a ledger that lost its rows would mean the database was thrown away, and "
           "clearing the database is not recovery (CONTRACTS section 11)")
    holders = {p["payload_id"]: p.get("holder")
               for p in after.get("payloads", []) if p.get("state") == "HELD"}
    record("F9_custody_is_unchanged",
           holders.get(cargo("F-1")) == held_before.get(cargo("F-1")),
           f"{held_before} -> {holders}")
    still_executing = {t.get("task_id") for t in after.get("tasks", [])
                       if t.get("state") == "EXECUTING"}
    record("F10_nothing_was_dragged_back_into_service", task_id not in still_executing,
           f"EXECUTING after restart: {sorted(still_executing)}")

    resp2 = submit(req("F-2"), cargo("F-2"))
    record("F11_the_fleet_accepts_new_work",
           resp2 is not None and bool(getattr(resp2, "accepted", False)),
           f"new task accepted={getattr(resp2, 'accepted', None)} "
           f"task_id={getattr(resp2, 'task_id', None)} "
           f"reason={getattr(resp2, 'reason_code', None)}",
           "a restart that leaves the fleet unable to take work is not a recovery")
    dump("restart, after recovery", snapshot())

    try:
        restarted.send_signal(signal.SIGINT)
        restarted.wait(timeout=20)
    except Exception:
        restarted.kill()


if __name__ == "__main__":
    raise SystemExit(main(sys.argv))
