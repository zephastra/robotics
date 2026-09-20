#!/usr/bin/env python3
"""P6 against a LIVE fleet: does the display go grey when the stream stops?

Run it through `scripts/check_p6_live.sh`, which sources the ROS environment and refuses to
start while another fleet is up.

Why this exists next to `check_p6_dashboard.sh`: that script asserts "no fleet => not green",
which is the disconnected case. It has never seen a fleet, so it cannot tell you whether the
page goes grey when a *running* fleet stops answering -- and that is the failure P6 is named
for. The two cases are different code paths: with no fleet the snapshot is `None` and every
list is empty; with a frozen stream a complete-but-old snapshot still exists and the display
has to decide, on its own, not to quote it.

Three interruptions, all real, none faked by a hook:

  1. SIGSTOP the task service. The process is alive but answers nothing -- the "stream
     stopped" case. The page must degrade LIVE -> AGING -> STALE and must keep being served
     while it does. A page that disappears instead of going grey is also wrong.
  2. SIGCONT it. The grey must LIFT. A display that latches to grey is a different bug and
     would otherwise pass every check above.
  3. SIGKILL one robot's adapter. The task service is still alive, so the page must stay
     LIVE and keep showing the other two robots -- while that one robot is shown as not
     fresh with its pose and battery withheld. This is the case a real fleet spends its time
     in, and the one nothing has tested.

Deliberately NOT done: any test that a stale page shows an *error*. Grey is the requirement;
a red banner is a design choice. Asserting on the exact banner text would pin cosmetics.

Exit codes: 0 = all checks passed, 1 = a check failed, 3 = environment problem, 4 = abort.

Nothing here imports rclpy. It talks HTTP to the dashboard and signals to processes, so it
cannot accidentally become part of the system it is judging.
"""

from __future__ import annotations

import json
import os
import pathlib
import signal
import subprocess
import sys
import time
import urllib.error
import urllib.request

ROOT = pathlib.Path(__file__).resolve().parents[1]
PORT = int(os.environ.get("P6_LIVE_PORT", "8123"))
ROBOTS = os.environ.get("P6_LIVE_ROBOTS", "r01,r02,r03")
DEAD_ROBOT = os.environ.get("P6_LIVE_DEAD_ROBOT", "r03")
BASE = f"http://127.0.0.1:{PORT}"

#: The task service stops being trusted about a robot after this long (STATE_MAX_AGE_S).
#: Read from the source rather than copied, so this check cannot drift out of step with it.
TASK_SERVICE_SRC = ROOT / "src" / "fleet_ros" / "fleet_ros" / "task_service_node.py"
DISPLAY_SRC = ROOT / "src" / "fleet_tools" / "fleet_tools" / "display.py"

CHECKS: list[dict] = []
READINGS: dict[str, list[dict]] = {}


# --------------------------------------------------------------------------- #
# recording
# --------------------------------------------------------------------------- #


def record(label: str, ok: bool, detail: str = "") -> None:
    CHECKS.append({"check": label, "ok": bool(ok), "detail": detail})
    print(f"  {'PASS' if ok else 'FAIL'}  {label}" + (f"\n          {detail}" if detail and not ok
                                                      else ""))


def declared_number(path: pathlib.Path, name: str) -> float:
    """Read `NAME = 12.0` out of a source file.

    The thresholds are asserted against the source rather than hard-coded here: a check that
    carries its own copy of the number it is checking passes after somebody changes the real
    one. That has already happened once in this project with a marker string.
    """
    for line in path.read_text(encoding="utf-8").splitlines():
        stripped = line.strip()
        if stripped.startswith(f"{name} ="):
            return float(stripped.split("=", 1)[1].split("#")[0].strip())
    raise SystemExit(f"check_p6_live: cannot find {name} in {path}")


# --------------------------------------------------------------------------- #
# http + processes
# --------------------------------------------------------------------------- #


def fetch(path: str) -> tuple[int, bytes]:
    request = urllib.request.Request(BASE + path, method="GET")
    try:
        with urllib.request.urlopen(request, timeout=8) as response:
            return response.status, response.read()
    except urllib.error.HTTPError as exc:
        return exc.code, exc.read()
    except Exception as exc:  # refused, reset, timed out
        return 0, repr(exc).encode("utf-8")


def read_page() -> dict:
    """One reading, reduced to the fields the checks need, plus enough to diagnose."""
    status, body = fetch("/api/snapshot")
    row: dict = {"http": status}
    if status != 200:
        row["error"] = body.decode("utf-8", "replace")[:200]
        return row
    try:
        payload = json.loads(body)
    except Exception as exc:
        row["error"] = f"not JSON: {exc}"
        return row
    live = payload.get("liveness") or {}
    robots = payload.get("robots") or []
    row.update({
        "state": live.get("state"),
        "age_s": None if live.get("age_s") is None else round(float(live["age_s"]), 2),
        "may_show_ok": live.get("may_show_ok"),
        "n_robots": len(robots),
        "robots": [
            {
                "robot_id": r.get("robot_id"),
                "state": r.get("display_state"),
                "pose": r.get("position") is not None,
                "battery": r.get("battery_fraction") is not None,
                "task_id": r.get("task_id") or "",
                "notes": r.get("notes") or [],
            }
            for r in robots
        ],
        "n_tasks": len(payload.get("tasks") or []),
        "n_payloads": len(payload.get("payloads") or []),
        "n_resources": len(payload.get("resources") or []),
        "backend": payload.get("backend"),
        "traffic_age_s": payload.get("traffic_age_s"),
    })
    return row


def procs(pattern: str) -> list[tuple[int, str]]:
    out = subprocess.run(["pgrep", "-af", pattern], capture_output=True, text=True)
    rows: list[tuple[int, str]] = []
    for line in out.stdout.splitlines():
        pid, _, cmd = line.partition(" ")
        if pid.isdigit():
            rows.append((int(pid), cmd.strip()))
    return rows


def poll(phase: str, seconds: float, interval: float = 1.0) -> list[dict]:
    readings: list[dict] = []
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        reading = read_page()
        reading["t"] = round(time.monotonic() - SESSION_START, 1)
        readings.append(reading)
        time.sleep(interval)
    READINGS[phase] = readings
    return readings


def wait_for(phase: str, predicate, seconds: float, interval: float = 1.5) -> list[dict]:
    """Poll until `predicate(reading)` holds, or the budget runs out. Returns every reading."""
    readings: list[dict] = []
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        reading = read_page()
        reading["t"] = round(time.monotonic() - SESSION_START, 1)
        readings.append(reading)
        if predicate(reading):
            break
        time.sleep(interval)
    READINGS[phase] = readings
    return readings


# --------------------------------------------------------------------------- #
# the flight recorder, so a failure can be diagnosed without a rerun
# --------------------------------------------------------------------------- #


def summarise_phase(phase: str) -> str:
    readings = READINGS.get(phase) or []
    if not readings:
        return f"{phase}: no readings"
    states = []
    for reading in readings:
        if reading.get("http") != 200:
            states.append(f"http{reading['http']}")
        else:
            states.append(f"{reading['state']}({reading.get('age_s')})")
    return f"{phase}: {' '.join(states)}"


SESSION_START = time.monotonic()


# --------------------------------------------------------------------------- #
# main
# --------------------------------------------------------------------------- #


def main() -> int:
    if not (ROOT / "install" / "fleet_bringup").is_dir():
        print("check_p6_live: install/ is missing. Run scripts/build.sh first.", file=sys.stderr)
        return 3
    if subprocess.run(["pgrep", "-f", f"{ROOT}/instal[l]"], capture_output=True).returncode == 0:
        print("check_p6_live: a fleet is already running under this project.", file=sys.stderr)
        print("  Stop it first: bash scripts/stop_demo.sh", file=sys.stderr)
        return 3

    stamp = time.strftime("%Y%m%dT%H%M%SZ", time.gmtime())
    run_dir = ROOT / "reports" / f"p6_live_{stamp}"
    run_dir.mkdir(parents=True, exist_ok=True)

    stale_s = declared_number(DISPLAY_SRC, "STALE_S")
    fresh_s = declared_number(DISPLAY_SRC, "FRESH_S")
    state_age_s = declared_number(TASK_SERVICE_SRC, "STATE_MAX_AGE_S")
    print(f"=== P6 live check === port {PORT}  robots {ROBOTS}  victim {DEAD_ROBOT}")
    print(f"    windows: fresh {fresh_s}s  stale {stale_s}s  task-service freshness {state_age_s}s")
    print(f"    run dir: {run_dir}")

    launch_log = (run_dir / "fleet_launch.log").open("w", encoding="utf-8")
    dash_log = (run_dir / "dashboard.log").open("w", encoding="utf-8")
    launch = subprocess.Popen(
        ["ros2", "launch", "fleet_bringup", "fleet.launch.py", f"robots:={ROBOTS}"],
        stdout=launch_log, stderr=subprocess.STDOUT, cwd=str(ROOT),
    )
    dash = subprocess.Popen(
        ["ros2", "run", "fleet_tools", "fleet_dashboard", "--ros-args", "-p", f"port:={PORT}"],
        stdout=dash_log, stderr=subprocess.STDOUT, cwd=str(ROOT),
    )
    stopped: list[int] = []

    try:
        # ---- the page must come up and serve ---------------------------------- #
        deadline = time.monotonic() + 60.0
        served = False
        while time.monotonic() < deadline:
            if fetch("/healthz")[0] == 200:
                served = True
                break
            time.sleep(1.0)
        if not served:
            print("check_p6_live: the dashboard never served /healthz. See "
                  f"{run_dir}/dashboard.log", file=sys.stderr)
            return 3

        # ---- P0: baseline, a live fleet --------------------------------------- #
        print("\n--- P0 baseline: the page must be green with three healthy robots ---")
        p0 = wait_for(
            "P0_baseline",
            lambda r: (r.get("state") == "LIVE" and r.get("n_robots") == len(ROBOTS.split(","))
                       and all(x["state"] == "ok" and x["pose"] and x["battery"]
                               for x in (r.get("robots") or []))),
            seconds=150.0, interval=2.0,
        )
        last = p0[-1]
        record("baseline_page_is_live_with_every_robot_green",
               last.get("state") == "LIVE" and last.get("may_show_ok") is True
               and last.get("n_robots") == len(ROBOTS.split(","))
               and all(x["state"] == "ok" for x in last.get("robots") or []),
               f"last reading: {json.dumps(last, sort_keys=True)}")
        record("baseline_shows_every_robots_pose_and_battery",
               all(x["pose"] and x["battery"] for x in last.get("robots") or []),
               f"rows: {json.dumps(last.get('robots'), sort_keys=True)}")
        record("baseline_is_not_green_because_it_has_not_heard_otherwise",
               last.get("backend") not in (None, "UNKNOWN") and last.get("n_tasks") is not None,
               f"backend={last.get('backend')!r} tasks={last.get('n_tasks')}")

        # ---- find the task service BEFORE stopping anything -------------------- #
        service = procs("fleet_ros/lib/fleet_ros/task_service_node")
        if len(service) != 1:
            print(f"\ncheck_p6_live: expected exactly one task service, found {len(service)}:",
                  file=sys.stderr)
            for pid, cmd in service:
                print(f"    {pid}  {cmd}", file=sys.stderr)
            return 3
        service_pid = service[0][0]
        print(f"\n    task service pid {service_pid}")

        # ---- P1: freeze the stream -------------------------------------------- #
        print("\n--- P1 freeze: SIGSTOP the task service; the page must go grey, not vanish ---")
        os.kill(service_pid, signal.SIGSTOP)
        stopped.append(service_pid)
        p1 = wait_for("P1_frozen", lambda r: r.get("state") == "STALE", seconds=45.0, interval=1.0)
        print(f"    {summarise_phase('P1_frozen')}")

        record("page_stays_served_while_the_stream_is_frozen",
               all(r.get("http") == 200 for r in p1),
               "the page must degrade to grey, not stop answering: "
               + json.dumps([r.get("http") for r in p1][:12]))
        record("degradation_is_graded_not_a_jump",
               any(r.get("state") == "AGING" for r in p1),
               f"states seen: {[r.get('state') for r in p1]}")
        record("page_reaches_stale_and_stops_claiming_ok",
               p1[-1].get("state") == "STALE" and p1[-1].get("may_show_ok") is False,
               f"last: {json.dumps(p1[-1], sort_keys=True)}")
        # Monotonicity is a property of the FROZEN period only. Before the freeze the age
        # legitimately falls again each time a read lands, and the earlier version of this
        # assertion failed on exactly those samples -- a check bug reported as a display bug.
        freeze_from = next((i for i, r in enumerate(p1) if r.get("state") != "LIVE"), None)
        frozen = p1[freeze_from:] if freeze_from is not None else []
        record("page_age_grows_monotonically_while_frozen",
               len(frozen) >= 3 and all(
                   a.get("age_s") is not None and b.get("age_s") is not None
                   and b["age_s"] >= a["age_s"] for a, b in zip(frozen, frozen[1:])),
               f"from the first non-LIVE reading: {[r.get('age_s') for r in frozen]} "
               f"(whole phase: {[r.get('age_s') for r in p1]})")

        # Every reading from the first non-live one onward must show nothing green.
        first_grey = next((i for i, r in enumerate(p1) if r.get("may_show_ok") is False), None)
        after = p1[first_grey:] if first_grey is not None else []
        green_after = [
            (r.get("t"), x["robot_id"], x["state"])
            for r in after for x in (r.get("robots") or []) if x["state"] == "ok"
        ]
        record("nothing_is_shown_green_once_the_page_is_not_live",
               not green_after, f"green rows after going grey: {green_after[:6]}")
        record("backend_is_withheld_once_the_page_is_not_live",
               all(r.get("backend") == "UNKNOWN" for r in after),
               f"backends: {[r.get('backend') for r in after][:8]}")

        final = p1[-1]
        rows = final.get("robots") or []
        record("pose_and_battery_are_withheld_when_the_stream_is_frozen",
               bool(rows) and all(not x["pose"] and not x["battery"] for x in rows),
               f"rows: {json.dumps(rows, sort_keys=True)}")
        record("the_task_list_is_withheld_when_the_stream_is_frozen",
               final.get("n_tasks") == 0 and final.get("n_payloads") == 0,
               f"tasks={final.get('n_tasks')} payloads={final.get('n_payloads')}")
        # The same claim, in the other place it is made. The task LIST is emptied above;
        # the per-robot assignment must be withheld for exactly the same reason, or the
        # page quotes one stale task while refusing to quote the list it came from.
        leaked = [x for x in rows if x["task_id"]]
        record("per_robot_task_assignment_is_withheld_when_the_stream_is_frozen",
               not leaked,
               f"stale rows still naming a task: {json.dumps(leaked, sort_keys=True)}")
        # The corridor view comes from the COORDINATOR, which is a different process and was
        # not touched. The page must not throw that good data away because an unrelated
        # source died -- and it must not pass it off as part of the task view either, which
        # is why the traffic age is reported separately. Both halves are asserted.
        record("a_frozen_task_service_does_not_erase_the_live_corridor_view",
               final.get("traffic_age_s") is not None and final.get("n_resources") > 0,
               f"traffic_age_s={final.get('traffic_age_s')} "
               f"n_resources={final.get('n_resources')} "
               "(the coordinator was never stopped, so its view is still good and must "
               "still be shown, labelled with its own age)")

        # ---- P2: resume ------------------------------------------------------- #
        print("\n--- P2 resume: SIGCONT; the grey must lift ---")
        os.kill(service_pid, signal.SIGCONT)
        stopped.remove(service_pid)
        p2 = wait_for(
            "P2_resumed",
            lambda r: r.get("state") == "LIVE" and all(
                x["state"] == "ok" for x in (r.get("robots") or [])),
            seconds=60.0, interval=1.5,
        )
        print(f"    {summarise_phase('P2_resumed')}")
        record("the_page_returns_to_live_after_the_stream_resumes",
               p2[-1].get("state") == "LIVE" and p2[-1].get("may_show_ok") is True
               and all(x["pose"] and x["battery"] for x in p2[-1].get("robots") or []),
               f"last: {json.dumps(p2[-1], sort_keys=True)}")

        # ---- P3: one robot's adapter dies ------------------------------------- #
        print(f"\n--- P3 one robot dies: SIGKILL {DEAD_ROBOT}'s adapter ---")
        victims = [row for row in procs("nav2_adapter_node") if DEAD_ROBOT in row[1]]
        for pid, cmd in procs("nav2_adapter_node"):
            print(f"    adapter pid {pid}: {cmd[:120]}")
        if len(victims) != 1:
            print(f"check_p6_live: expected exactly one adapter for {DEAD_ROBOT}, "
                  f"found {len(victims)}", file=sys.stderr)
            return 3
        victim_pid = victims[0][0]
        os.kill(victim_pid, signal.SIGKILL)
        print(f"    killed pid {victim_pid}")
        def exactly_one_lost(reading: dict) -> bool:
            """One robot not trustworthy, the other two fully good.

            The previous predicate fired on the mere existence of a robot without a pose,
            which was true on the first reading after the freeze -- while the task service was
            still catching up from being SIGSTOPped, so EVERY robot read as not-fresh. The
            check then asserted that the survivors kept their values using a sample in which
            nobody had any. It failed for a reason unrelated to the display, and worse, the
            P3 evidence from that run could not distinguish "the display handles a dead robot"
            from "the display distrusts everything" -- so it is treated as a weak run, not
            merely a mis-reported one.

            Waiting for the discriminating state makes the assertion mean what it says.
            """
            rows = reading.get("robots") or []
            if len(rows) != len(ROBOTS.split(",")):
                return False
            good = [x for x in rows if x["pose"] and x["battery"] and x["state"] == "ok"]
            lost = [x for x in rows if not x["pose"] and not x["battery"]]
            return len(good) == len(rows) - 1 and len(lost) == 1

        p3 = wait_for("P3_robot_dead", exactly_one_lost,
                      seconds=max(30.0, state_age_s * 6), interval=1.0)
        print(f"    {summarise_phase('P3_robot_dead')}")
        for reading in p3:
            rows = [{k: v for k, v in x.items() if k != "notes"}
                    for x in reading.get("robots") or []]
            print(f"      t={reading.get('t')}s {json.dumps(rows, sort_keys=True)}")
        p3_last = p3[-1]
        rows3 = p3_last.get("robots") or []
        dead = [x for x in rows3 if x["robot_id"] == DEAD_ROBOT]
        alive = [x for x in rows3 if x["robot_id"] != DEAD_ROBOT]

        record("the_page_stays_live_when_only_one_robot_dies",
               p3_last.get("state") == "LIVE" and p3_last.get("may_show_ok") is True,
               f"last: {json.dumps({k: v for k, v in p3_last.items() if k != 'robots'}, sort_keys=True)}")
        record("the_dead_robot_is_still_listed_but_not_green",
               len(dead) == 1 and dead[0]["state"] != "ok",
               f"dead row: {json.dumps(dead, sort_keys=True)}")
        record("the_dead_robot_says_why_it_is_not_trusted",
               bool(dead) and any("fresh" in note for note in dead[0]["notes"]),
               f"notes: {dead[0]['notes'] if dead else None}")
        record("the_dead_robots_pose_and_battery_are_withheld",
               bool(dead) and not dead[0]["pose"] and not dead[0]["battery"],
               f"dead row: {json.dumps(dead, sort_keys=True)}")
        record("the_other_robots_keep_their_values",
               len(alive) == len(ROBOTS.split(",")) - 1
               and all(x["pose"] and x["battery"] and x["state"] == "ok" for x in alive),
               f"alive rows: {json.dumps(alive, sort_keys=True)}")

    finally:
        print("\n--- teardown ---")
        for pid in stopped:
            try:
                os.kill(pid, signal.SIGCONT)
            except ProcessLookupError:
                pass
        for name, proc in (("dashboard", dash), ("launch", launch)):
            if proc.poll() is not None:
                continue
            proc.send_signal(signal.SIGINT)
            for _ in range(45):
                if proc.poll() is not None:
                    break
                time.sleep(1.0)
            if proc.poll() is None:
                print(f"    {name} still alive after 45 s of SIGINT; escalating")
                proc.terminate()
                for _ in range(30):
                    if proc.poll() is not None:
                        break
                    time.sleep(1.0)
            if proc.poll() is None:
                proc.kill()
        time.sleep(3.0)
        pattern = f"{ROOT}/instal[l]"
        leftover = subprocess.run(["pgrep", "-f", pattern], capture_output=True, text=True)
        gz = subprocess.run(["pgrep", "-f", "gz-sim-mai[n]"], capture_output=True, text=True)
        n_left = len([x for x in leftover.stdout.split() if x.isdigit()])
        n_gz = len([x for x in gz.stdout.split() if x.isdigit()])
        clean = n_left == 0 and n_gz == 0
        print(f"    leftover project processes: {n_left}   gz: {n_gz}")
        if not clean:
            print("    WARNING: the world is still up; the next run's numbers would be "
                  "contaminated. Not cleaning up globally on purpose.")
            print(f"    inspect with: pgrep -a -f '{ROOT}'")
        else:
            print("    teardown verified clean")
        record("teardown_verified_clean", clean,
               f"leftover={n_left} gz={n_gz}")

    failures = [c for c in CHECKS if not c["ok"]]
    verdict = {
        "port": PORT,
        "robots": ROBOTS,
        "victim_robot": DEAD_ROBOT,
        "windows": {"fresh_s": fresh_s, "stale_s": stale_s, "state_max_age_s": state_age_s},
        "checks_total": len(CHECKS),
        "checks_passed": len(CHECKS) - len(failures),
        "failed": [c["check"] for c in failures],
        "summary": "PASS" if not failures else "FAIL",
        "checks": CHECKS,
        "phase_summaries": {p: summarise_phase(p) for p in READINGS},
        "readings": READINGS,
    }
    (run_dir / "p6_live_verdict.json").write_text(
        json.dumps(verdict, indent=2, sort_keys=True), encoding="utf-8")

    print(f"\n=== {len(CHECKS) - len(failures)}/{len(CHECKS)} checks passed -> {verdict['summary']}")
    for phase, text in verdict["phase_summaries"].items():
        print(f"    {text}")
    print(f"    verdict: {run_dir / 'p6_live_verdict.json'}")
    for check in failures:
        print(f"    FAILED: {check['check']}")
        print(f"            {check['detail']}")
    return 0 if not failures else 1


if __name__ == "__main__":
    raise SystemExit(main())
