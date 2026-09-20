#!/usr/bin/env python3
"""Run the frozen case list of TEST_AND_ACCEPTANCE section 5, one case at a time.

    scripts/batch.py --manifest config/scenarios/regression_v1.yaml --dry-run
    scripts/batch.py --manifest config/scenarios/regression_v1.yaml --only N02,I01
    scripts/batch.py --manifest config/scenarios/regression_v1.yaml

WHAT THIS IS FOR
----------------
Section 5 freezes 30 cases and section 5's strategy adds two extra seeds for four of them. A
case list that lives in a script cannot be checked against the contract, so the list is data
(`config/scenarios/regression_v1.yaml`) and this runner only executes it. Two consequences
that matter:

  * a case that cannot run is a first-class outcome, not a skip. It reports
    `NOT_RUN` with the prerequisite it is waiting for, and the prerequisite has to be written
    down in `docs/LIMITATIONS.md` -- `scripts/check_batch_manifest.py` enforces both.
  * nothing is overwritten. Every case gets its own directory under
    `reports/batch_<UTC stamp>/`, so a re-run is a second record rather than a replacement of
    the first. Section 5: "重跑单独记录，不覆盖首次结果".

GROUND TRUTH, AND WHY IT IS A SEPARATE PROCESS
---------------------------------------------
Every case runs with `fleet_evaluation`'s recorder beside it, writing `samples-<case>.jsonl`
into the case directory, and is then judged by `fleet_evaluation.judge` from that recording.
That is what makes a safety verdict possible at all: section 1 wants the simulator's own body
poses, not the robot's estimate of them.

The recorder is a separate PROCESS, not a thread in this one, and that is a requirement rather
than a preference: this runner blocks on service calls, and an instrument sharing a thread with
the thing it measures records its own queue instead of the fleet. P4 measured exactly that
shape with a probe that limited its own rate.

The judge runs in-process because it is pure Python and reads only files -- it cannot touch the
run it is judging, so sharing an interpreter with the driver costs nothing. Its independence is
structural (no ROS, asserted by a test), not a matter of which process it happens to be in.

`safety_outcome` comes from the two safety properties the judge derives from truth, not from
the judge's summary as a whole: the summary also carries pose error and the claim
cross-checks, which are not safety properties, and folding them into a safety verdict would
report a localisation number as a collision.

STEPS AND FAULTS
----------------
A case may declare `steps:`, an ordered list of `when <predicate> do <action>`. That is the
vocabulary `docs/LIMITATIONS.md` names under `batch_driver_extended`, and the rule behind it
is `docs/TEST_AND_ACCEPTANCE.md` section 5:

    故障按状态触发而非某个固定 sleep ... 超时未到触发条件为 PRECONDITION_NOT_REACHED，
    不算故障测试通过

So a fault fires on a STATE the case names, and a case whose trigger never arrived is refused
rather than reported as a run. The predicate vocabulary is `fleet_core.wait_for`'s, evaluated
against the same snapshot the rest of the case reads -- one vocabulary, one implementation.

The steps are evaluated BEFORE the loop's own exit condition. Those two can become true in the
same tick, and a case that finished without its fault having been delivered would be a normal
case wearing a fault case's name.

A case whose claim covers work the SERVICE generates -- N08's charge queue -- says so with
`await_charge: true`, and does not stop until those runs have finished as well.

Exit codes follow section 9: 0 all cases behaved as declared, 2 input error, 3 dependency or
startup problem, 4 a case's task or behaviour outcome failed, 5 infrastructure / sampling /
budget -- which includes the safety-UNKNOWN above, and any NOT_RUN.
"""

from __future__ import annotations

import argparse
import json
import os
import signal
import subprocess
import sys
import time
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))
from check_batch_manifest import load_manifest, planned_runs  # noqa: E402

MANIFEST_DEFAULT = ROOT / "config" / "scenarios" / "regression_v1.yaml"

#: lifecycle_msgs/msg/State.ACTIVE
LC_ACTIVE = 3
#: How long a case may wait for the fleet before it is a startup problem.
READY_BUDGET_S = 120.0

#: How long a case may wait for its own declared arrival order before the case is declared
#: unable to start at all. A case that cannot arrange the order it is about has not run, and
#: reporting it as a timeout afterwards measures the wrong thing.
WAIT_BUDGET_S = 240.0

#: How often the arrival-order predicate is re-read. It reads the same snapshot the rest of the
#: case reads, so what it waits for and what it later asserts come from one source.
WAIT_POLL_S = 0.5

#: How long the runner waits after SIGTERMing a node before it reads the fleet again. Not a
#: trigger -- the fault has been delivered either way -- but a process that has been ASKED to
#: stop has not stopped, and reading the snapshot inside that window reads the fleet the step
#: is in the middle of changing.
FAULT_SETTLE_S = 2.0

#: The behaviour checks a case may name in `behavior_checks`, enumerated here because a name
#: that is silently ignored makes a case look stricter than it is -- and this project has
#: already shipped a guard that read a field nobody wrote. The manifest guard parses this
#: tuple, so a case cannot name a check that does not exist.
DECLARED_CHECKS = (
    "no_duplicate_task",
    "charger_capacity_respected",
    "charger_contended",
    "payload_not_rehomed",
)

#: How long the recorder gets to flush after being asked to stop. Measured: a clean stop takes
#: under two seconds, but the recorder also flushes on SIGTERM now, and the budget has to
#: outlive a slow write rather than the other way round.
RECORDER_FLUSH_S = 30.0
#: How often the recorder samples. Section 1 wants at least 50 sim Hz for the *world* stream;
#: this is the rate of the JUDGED recording, and the recorder reports the rate it achieved so
#: the judge can hold it rather than assume it.
RECORDER_HZ = 20.0

#: Truth sources this runner can attach. A case may name one in its `truth:` key, and
#: `scripts/check_batch_manifest.py` parses this tuple and refuses a name that is not here --
#: so a manifest cannot claim a truth source the runner does not implement.
TRUTH_SOURCES = ("fleet_recorder",)

#: The judge's checks that ARE safety properties. `safety_outcome` is derived from these alone.
SAFETY_CHECKS = (
    "no_simultaneous_occupancy_from_truth",
    "no_unauthorised_entry_from_truth",
)


class CaseFailed(Exception):
    """A case could not be run at all (as opposed to running and misbehaving)."""


def _say(msg: str, indent: int = 0) -> None:
    print(" " * indent + msg, flush=True)


# --------------------------------------------------------------------------- #
# the run directory: what section 7 requires a run to save
# --------------------------------------------------------------------------- #


class CaseDir:
    def __init__(self, base: Path, case: str, seed: int) -> None:
        self.name = case if seed == 1 else f"{case}_seed{seed}"
        self.path = base / self.name
        self.path.mkdir(parents=True, exist_ok=False)
        self.stdout = (self.path / "stdout.log").open("w", encoding="utf-8")
        self.stderr = (self.path / "stderr.log").open("w", encoding="utf-8")

    def inputs(self, manifest_path: Path, case_body: dict) -> None:
        (self.path / "inputs").mkdir(exist_ok=True)
        (self.path / "inputs" / "case.yaml").write_text(
            yaml.safe_dump(case_body, sort_keys=True), encoding="utf-8"
        )
        (self.path / "inputs" / "manifest_ref.txt").write_text(
            f"{manifest_path.relative_to(ROOT)}\n", encoding="utf-8"
        )

    def close(self) -> None:
        self.stdout.close()
        self.stderr.close()


# --------------------------------------------------------------------------- #
# the fleet
# --------------------------------------------------------------------------- #


class Fleet:
    """One bring-up of one case. Owns its processes and its runtime paths."""

    #: Every process class the launch tree spawns, as a substring of a real command line.
    #: It is a class constant so a test can require that the sweep covers all of them; a
    #: literal inside `stop()` cannot be checked, and the previous literal was missing the
    #: three classes whose command lines contain none of the words it did look for:
    #:
    #:   /opt/ros/lyrical/lib/ros_gz_bridge/parameter_bridge /r01/scan@...
    #:   /opt/ros/lyrical/lib/robot_state_publisher/robot_state_publisher ...
    #:   /opt/ros/lyrical/lib/tf2_ros/static_transform_publisher ...
    #:
    #: None of those contains "gz sim", "nav2_" or "amcl", so all three survived the sweep.
    #: A second `clock_bridge` in particular is not a leak but a correctness fault: two
    #: publishers on /clock give every consumer a clock that jumps, and a jumping clock makes
    #: tf2 discard its whole buffer. Measured 2026-09-19: three leaked generations left the
    #: host at load average 20 and made two of three later launches abort.
    LEFTOVER_PATTERNS = (
        "fleet_bringup/fleet.launch.py",
        "gz sim",
        "parameter_bridge",
        "robot_state_publisher",
        "static_transform_publisher",
        "nav2_",
        "amcl",
        "fleet_ros/lib/fleet_ros",
    )

    def __init__(self, root: Path, runs: list[str], scenario: str | None,
                 case_dir: CaseDir) -> None:
        self.root = root
        self.runs = runs
        self.scenario = scenario
        self.dir = case_dir
        self.proc: "subprocess.Popen | None" = None
        self.runtime = case_dir.path / "runtime"
        self.runtime.mkdir(exist_ok=True)

    def _launch_args(self) -> list[str]:
        args = [
            "ros2", "launch", "fleet_bringup", "fleet.launch.py",
            f"robots:={','.join(self.runs)}",
            f"db_path:={self.runtime / 'fleet.sqlite'}",
            f"events_path:={self.runtime / 'events.jsonl'}",
            f"robot_state_dir:={self.runtime / 'robot_state'}",
        ]
        if self.scenario:
            args.append(f"scenario:={self.scenario}")
        return args

    def start(self) -> None:
        self.proc = subprocess.Popen(
            self._launch_args(), cwd=str(self.root),
            stdout=self.dir.stdout, stderr=self.dir.stderr,
            start_new_session=True,
        )

    def stop(self) -> "dict[str, int]":
        """Stop the fleet and say what was left behind.

        A leftover simulator turns the NEXT case's evidence into fiction, and this project
        has already read a real robot's silence as a scheduling failure once. So the sweep is
        part of the case, not a courtesy.

        Two things this must not get wrong (D-P17-25):

        * The process-GROUP kill must not be gated on the launcher still being alive. A tree
          whose `ros2 launch` parent has already exited still has live children, and that is
          precisely the case where skipping the group kill leaks them. The pgid is captured
          and signalled unconditionally; a dead group raises ProcessLookupError and that is
          the normal, quiet outcome.
        * The sweep must cover what the tree actually spawns, which is why the patterns are
          the LEFTOVER_PATTERNS constant above rather than a list written here.
        """
        if self.proc is not None:
            try:
                pgid = os.getpgid(self.proc.pid)
            except ProcessLookupError:
                pgid = None
            if pgid is not None:
                try:
                    os.killpg(pgid, signal.SIGINT)
                except ProcessLookupError:
                    pass
                try:
                    self.proc.wait(timeout=20)
                except subprocess.TimeoutExpired:
                    try:
                        os.killpg(pgid, signal.SIGKILL)
                    except ProcessLookupError:
                        pass
                    try:
                        self.proc.wait(timeout=10)
                    except subprocess.TimeoutExpired:
                        # The direct child is stuck. The pkill sweep below still runs, so
                        # say nothing here and let the leftover count be the evidence.
                        pass
        for pattern in self.LEFTOVER_PATTERNS:
            subprocess.run(["pkill", "-f", pattern], capture_output=True, check=False)
        time.sleep(3)
        return self.leftovers()

    @classmethod
    def leftovers(cls) -> "dict[str, int]":
        """Live processes per pattern, counted WITHOUT letting the counter match itself.

        `ps | grep -c <pattern>` reports 1 on an idle machine: the grep process's own command
        line contains the pattern and grep does not exclude itself. (Measured 2026-09-19 --
        that exact metric was used to judge a cleanup and read 1 before and after.) Reading
        /proc/<pid>/cmdline and skipping our own pid cannot do that.
        """
        counts = {p: 0 for p in cls.LEFTOVER_PATTERNS}
        mine = {os.getpid(), os.getppid()}
        for entry in os.listdir("/proc"):
            if not entry.isdigit() or int(entry) in mine:
                continue
            try:
                with open(f"/proc/{entry}/cmdline", "rb") as fh:
                    cmd = fh.read().replace(b"\x00", b" ").decode("utf-8", "replace")
            except OSError:
                continue
            for pattern in counts:
                if pattern in cmd:
                    counts[pattern] += 1
        return counts


# --------------------------------------------------------------------------- #
# the ground-truth recorder
# --------------------------------------------------------------------------- #


class TruthRecorder:
    """One `fleet_recorder` process per case. Starts before the fleet is ready, stops after
    the last observation, and is expected to leave a recording behind.

    Stopped with SIGTERM rather than SIGINT. That is not arbitrary: SIGINT into a process
    group reaches the recorder twice (because `ros2 run` handles it AND forwards it) and the
    second one used to land inside the flush and destroy the recording outright -- measured,
    exit 254 and zero files. `scripts/probe_recorder_stop.py` holds that measurement, and
    SIGTERM now means the same thing as SIGINT inside the recorder.
    """

    def __init__(self, case_dir: "CaseDir", runs: list[str], label: str) -> None:
        self.dir = case_dir.path
        self.runs = runs
        self.label = label
        self.proc: "subprocess.Popen | None" = None
        self.audit: dict = {}
        self.stop_reason = ""
        self.stdout_path = self.dir / "recorder-stdout.log"

    def start(self) -> None:
        self.dir.mkdir(parents=True, exist_ok=True)
        cmd = [
            "ros2", "run", "fleet_evaluation", "fleet_recorder",
            "--robots", ",".join(self.runs),
            "--out", str(self.dir),
            "--label", self.label,
            "--sample-hz", str(RECORDER_HZ),
            "--duration", "0",
        ]
        self.out = self.stdout_path.open("w", encoding="utf-8")
        self.proc = subprocess.Popen(cmd, stdout=self.out, stderr=subprocess.STDOUT,
                                     text=True, start_new_session=True)

    def stop(self) -> dict:
        """Stop it and read what it wrote. Idempotent: teardown calls it too."""
        if self.proc is not None and self.proc.poll() is None:
            self.proc.terminate()
            try:
                self.proc.wait(timeout=RECORDER_FLUSH_S)
                self.stop_reason = f"terminated, exit {self.proc.returncode}"
            except subprocess.TimeoutExpired:
                # It is not going to flush. Say so rather than pretend the recording is fine.
                os.killpg(os.getpgid(self.proc.pid), signal.SIGKILL)
                self.proc.wait(timeout=10)
                self.stop_reason = "had to be SIGKILLed; the recording is not trusted"
        if self.proc is not None:
            try:
                self.out.close()
            except Exception:
                pass

        audit_file = self.dir / f"recorder-{self.label}.json"
        if audit_file.is_file():
            try:
                self.audit = json.loads(audit_file.read_text(encoding="utf-8"))
            except json.JSONDecodeError as exc:
                self.stop_reason = f"the audit file is unreadable: {exc}"
        return self.audit

    def samples_file(self) -> Path | None:
        found = sorted(self.dir.glob(f"samples-{self.label}.jsonl"))
        return found[0] if found else None


def harvest_crossing_claims(case_dir: "CaseDir") -> list[str]:
    """File the passage drivers' own reports where the judge looks for driver claims.

    One claim is one corridor crossing: the driver states `robot`, `direction` and `complete`,
    and the judge decides whether truth agrees -- `complete` is corroborated only if the robot
    actually crossed and moved at least 2 m. The runner files the driver's report; it does not
    paraphrase it. A claim rewritten by the party being checked is not a claim.

    Non-crossing cases have no such report, so `driver_claims_corroborated_by_truth` stays
    `NOT_RUN` for them, which is the correct answer: it was not performed.
    """
    source = case_dir.path / "runtime" / "crossings"
    filed: list[str] = []
    if not source.is_dir():
        return filed
    for n, path in enumerate(sorted(source.glob("*.json")), 1):
        try:
            body = json.loads(path.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError):
            continue
        if not isinstance(body, dict) or "robot" not in body or "direction" not in body:
            # Not a driver claim. `load_case` collects on exactly these two keys, so filing
            # anything else under this name would create a claim out of a log line.
            continue
        target = case_dir.path / f"driver-crossing-{n}.json"
        target.write_text(json.dumps(body, indent=2, sort_keys=True), encoding="utf-8")
        filed.append(target.name)
    return filed


def judge_case_dir(root: Path, case_dir: "CaseDir") -> dict:
    """Judge one case directory from truth. Pure Python; reads files only."""
    import yaml

    from fleet_core import validate_traffic_config
    from fleet_evaluation.judge import judge_case, load_spawns

    cfg = validate_traffic_config(
        yaml.safe_load((root / "config" / "resources.yaml").read_text(encoding="utf-8")))
    spawns = load_spawns(root / "config" / "spawns.yaml")
    return judge_case(case_dir.path, cfg, spawns)


def safety_from_judgement(result: dict) -> tuple[str, str]:
    """The two safety properties, and nothing else, become `safety_outcome`."""
    by_name = {c["check"]: c for c in result.get("checks") or []}
    verdicts = []
    for name in SAFETY_CHECKS:
        check = by_name.get(name)
        if check is None:
            return ("UNKNOWN", f"the judge did not report {name}; a missing safety property "
                               "is not a passing one")
        verdicts.append((name, check["verdict"], check.get("detail", "")))
    failed = [v for v in verdicts if v[1] == "FAIL"]
    if failed:
        return ("FAIL", "; ".join(f"{n}: {d}" for n, _v, d in failed))
    not_run = [v for v in verdicts if v[1] != "PASS"]
    if not_run:
        return ("UNKNOWN", "; ".join(f"{n} is {v}" for n, v, _d in not_run))
    return ("PASS", "; ".join(f"{n} PASS" for n, _v, _d in verdicts))


# --------------------------------------------------------------------------- #
# the ROS side
# --------------------------------------------------------------------------- #


class Driver:
    """Service clients. Runs in the main thread, so spinning here is not a nested spin."""

    def __init__(self, runs: list[str]) -> None:
        import rclpy
        from fleet_interfaces.srv import CancelTask, FaultInject, SubmitTask
        from lifecycle_msgs.srv import GetState
        from rclpy.callback_groups import ReentrantCallbackGroup
        from rclpy.node import Node

        self.rclpy = rclpy
        rclpy.init()
        self.node = Node("p7_batch")
        group = ReentrantCallbackGroup()
        self.submit = self.node.create_client(SubmitTask, "/fleet/submit_task",
                                             callback_group=group)
        self.cancel = self.node.create_client(CancelTask, "/fleet/cancel_task",
                                             callback_group=group)
        self.snapshot = self.node.create_client(__import__(
            "std_srvs.srv", fromlist=["Trigger"]).Trigger, "/fleet/tasks",
            callback_group=group)
        self.inject = {
            rid: self.node.create_client(FaultInject, f"/{rid}/fleet/inject_fault",
                                        callback_group=group)
            for rid in runs
        }
        self.lifecycle = {
            rid: self.node.create_client(GetState, f"/{rid}/bt_navigator/get_state",
                                        callback_group=group)
            for rid in runs
        }

    def call(self, client, request, timeout: float = 15.0):
        if not client.wait_for_service(timeout_sec=timeout):
            return None
        future = client.call_async(request)
        deadline = time.monotonic() + timeout
        while not future.done():
            if time.monotonic() > deadline:
                return None
            self.rclpy.spin_once(self.node, timeout_sec=0.05)
        try:
            return future.result()
        except Exception:
            return None

    def snap(self) -> dict:
        from std_srvs.srv import Trigger
        resp = self.call(self.snapshot, Trigger.Request(), timeout=10.0)
        if resp is None or not resp.success:
            return {}
        return json.loads(resp.message)

    def all_nav2_active(self) -> bool:
        for client in self.lifecycle.values():
            resp = self.call(client, __import__(
                "lifecycle_msgs.srv", fromlist=["GetState"]).GetState.Request(),
                timeout=3.0)
            if resp is None or int(resp.current_state.id) != LC_ACTIVE:
                return False
        return True

    def close(self) -> None:
        self.node.destroy_node()
        self.rclpy.try_shutdown()


def wait_for_ready(driver: Driver, log) -> bool:
    """Fresh state for every robot AND every robot's Nav2 ACTIVE, or give up.

    The lifecycle half is not optional: an action server exists while its node is INACTIVE,
    so a goal sent too early is refused, and this is the same readiness question D-P5-23 was
    about. Waiting here is cheaper than diagnosing it later.
    """
    from std_srvs.srv import Trigger

    deadline = time.monotonic() + READY_BUDGET_S
    while time.monotonic() < deadline:
        snap = driver.snap()
        robots = snap.get("robots") or {}
        if robots and all(v.get("fresh") for v in robots.values()):
            if driver.all_nav2_active():
                return True
        time.sleep(2.0)
    return False


def inject_battery(driver: Driver, rid: str, fraction: float) -> bool:
    from fleet_interfaces.srv import FaultInject

    req = FaultInject.Request()
    req.schema_version = "1"
    req.robot_id = rid
    req.inject_battery = True
    req.battery_fraction = float(fraction)
    req.duration_s = 0.0
    resp = driver.call(driver.inject[rid], req, timeout=10.0)
    return bool(resp is not None and resp.ok)


def wait_until(driver: Driver, condition: dict, *, budget_s: float) -> "tuple[bool, str]":
    """Wait for a predicate over the fleet snapshot. Returns (held, why not).

    NOT A SLEEP. `docs/LIMITATIONS.md`'s `crossing_scenarios` entry names the gap this closes:
    the runner submits sequentially, so "r01 asks first" was not expressible. Sequential
    submission is not arrival order -- two requests sent back to back reach their robots at
    whatever moment each robot becomes free -- so a case relying on it would pass or fail by
    luck, and nothing in the record would say which.

    A sleep would be worse and is the same shape as the "sleep then kill" trigger this project
    refuses elsewhere: a fixed sleep is a guess about how long something takes, and it is wrong
    in both directions. Here the case states the state it is waiting for and this observes it.
    """
    from fleet_core import wait_for

    deadline = time.monotonic() + budget_s
    last = "the snapshot has not been read yet"
    while time.monotonic() < deadline:
        snapshot = driver.snap()
        if not snapshot:
            last = "the fleet snapshot did not answer"
        else:
            verdict = wait_for.condition_holds(snapshot, condition)
            if verdict.holds:
                return True, ""
            if verdict.impossible:
                # Waiting cannot help, and saying so now is the whole difference between "the
                # case could not start" and "the case ran out of time": different findings,
                # different next steps.
                return False, f"can never hold -- {verdict.detail}"
            last = verdict.detail
        time.sleep(WAIT_POLL_S)
    return False, f"still not true after {budget_s:.0f}s -- {last}"


def _acquire_requests(case_dir: Path) -> dict:
    """How many requests each driver needed for its permit, per crossing report.

    The passage driver writes `granted ... after N request(s)` into its acquire stage, so this
    is the cheapest honest answer to "did the corridor make anybody wait". Read only; nothing
    is judged from it here.
    """
    out: dict = {}
    for path in sorted(case_dir.glob("driver-crossing-*.json")):
        try:
            body = json.loads(path.read_text(encoding="utf-8"))
        except (OSError, ValueError):
            continue
        robot = body.get("robot", path.stem)
        for stage in (body.get("stages") or []):
            if stage.get("name") != "acquire":
                continue
            detail = str(stage.get("detail", ""))
            requests = None
            if "after " in detail and " request" in detail:
                tail = detail.split("after ", 1)[1]
                requests = int("".join(c for c in tail.split(" request")[0]
                                       if c.isdigit()) or 0)
            out.setdefault(robot, []).append(
                {"ok": bool(stage.get("ok")), "requests": requests, "detail": detail})
    return out


def _crossing_ledger():
    """`fleet_core.crossing_ledger`, imported where it is used.

    Deferred for the same reason as the `fleet_core` import in `judge_case_dir`: `batch.py`
    runs under the system interpreter with ROS sourced, where the built packages are
    importable, and a module-scope import would break `--help` and the manifest guard in a
    bare interpreter.
    """
    from fleet_core import crossing_ledger

    return crossing_ledger


# --------------------------------------------------------------------------- #
# steps: `when <predicate> do <action>`
# --------------------------------------------------------------------------- #


def _trigger():
    """`fleet_core.trigger`, imported where it is used -- same reason as `_crossing_ledger`."""
    from fleet_core import trigger

    return trigger


def _wait_for_module():
    """`fleet_core.wait_for`, imported where it is used. One predicate vocabulary."""
    from fleet_core import wait_for

    return wait_for


def _pending_steps(body: dict, runs: list[str]) -> list[dict]:
    """The case's steps, validated before anything is launched.

    Validated here as well as in the manifest guard because a manifest can be edited after the
    build. A step that would fault a robot the case does not run is refused, not defaulted:
    the fault would land somewhere else and the case would then be evidence about a fleet it
    never described.
    """
    steps = list(body.get("steps") or [])
    trigger = _trigger()
    findings: list[str] = []
    for index, step in enumerate(steps, 1):
        findings += [f"step #{index}: {f}" for f in trigger.validate(step, runs)]
    if findings:
        raise CaseFailed("the case's step list is not usable: " + "; ".join(findings))
    return steps


def process_pattern(runner: str, node: str) -> str:
    """The `pgrep -f` pattern that addresses exactly one robot's copy of one node.

    Built from the two argv tokens `ros2 launch` writes, measured with `ps` on a live fleet:

        python3 .../lib/fleet_ros/nav2_adapter_node --ros-args -r __node:=nav2_adapter -r __ns:=/r01

    A node's NAME is a runtime registration and does not appear in `ps`; its argv does, and the
    namespace is part of it. That is what makes one robot's gate addressable and the other
    robot's gate not, without a pid file or a name the launch does not use.
    """
    trigger = _trigger()
    node = str(node or "")
    if node == trigger.TARGET_ALL:
        if not runner:
            raise ValueError("node 'all' needs a runner, or it addresses every node in the "
                             "fleet")
        return f"__ns:=/{runner}"
    if runner:
        return f"__node:={node}.*__ns:=/{runner}"
    return f"__node:={node}"


def find_processes(pattern: str) -> list[int]:
    """Pids whose argv matches. No shell: a shell would expand the pattern it is handed, and
    `pgrep -f` matching the invoking command line is a mistake this project has made three
    times already."""
    out = subprocess.run(["pgrep", "-f", pattern], capture_output=True, text=True,
                         check=False).stdout
    pids: list[int] = []
    for token in out.split():
        if token.isdigit():
            pids.append(int(token))
    return pids


def kill_process(runner: str, node: str) -> "tuple[bool, str]":
    """SIGTERM one node, or every node in one robot's namespace. Returns (ok, what happened)."""
    trigger = _trigger()
    pattern = process_pattern(runner, node)
    pids = find_processes(pattern)
    if not pids:
        # NOT "the fleet survived it": the fault was never delivered, and section 5 says a case
        # whose trigger did not arrive is NOT_RUN rather than a pass.
        return False, f"no process matched {pattern!r}; the fault was not delivered"
    if len(pids) > 1 and str(node) != trigger.TARGET_ALL:
        # Refused rather than guessed. Killing the wrong one of two puts the fault in a robot
        # the case never named.
        return False, (f"{len(pids)} processes matched {pattern!r} ({pids}); refusing to guess "
                       "which one the case meant")
    for pid in pids:
        try:
            os.kill(pid, signal.SIGTERM)
        except ProcessLookupError:
            continue
    time.sleep(FAULT_SETTLE_S)
    return True, f"SIGTERM to pid(s) {pids} ({pattern})"


def restart_process(runner: str, node: str, log) -> "tuple[bool, str]":
    """Stop one node and start it again from the argv it was started with.

    WHY /proc AND NOT A RE-LAUNCH. `ros2 launch` owns this node: its argv (including the
    generated `--params-file /tmp/launch_params_*`), its working directory and its environment
    were all assembled by the launch, and none of them is reproducible from the manifest. A
    restart that invented its own argv would start a node with default parameters and then
    report that the fleet recovered -- a different process wearing the same name. So all three
    are read out of /proc BEFORE the kill, while they still exist.

    Section 6: 中央重启保留 SQLite，不能新建空数据库. Nothing here chooses a database; the
    node is restarted with the arguments it had, so the same `--db_path` comes back.
    """
    pattern = process_pattern(runner, node)
    pids = find_processes(pattern)
    if not pids:
        return False, f"no process matched {pattern!r}; nothing was restarted"
    if len(pids) > 1:
        return False, (f"{len(pids)} processes matched {pattern!r} ({pids}); a restart names one "
                       "node, not a namespace")
    pid = pids[0]
    try:
        argv = [a.decode(errors="replace")
                for a in Path(f"/proc/{pid}/cmdline").read_bytes().split(b"\0") if a]
        cwd = os.readlink(f"/proc/{pid}/cwd")
        env: "dict[str, str]" = {}
        for entry in Path(f"/proc/{pid}/environ").read_bytes().split(b"\0"):
            if b"=" in entry:
                key, _, value = entry.partition(b"=")
                env[key.decode(errors="replace")] = value.decode(errors="replace")
    except OSError as exc:
        return False, f"could not read pid {pid} before killing it: {exc}"
    if not argv:
        return False, f"pid {pid} has an empty argv, so there is nothing to start again"
    os.kill(pid, signal.SIGTERM)
    time.sleep(FAULT_SETTLE_S)
    subprocess.Popen(argv, cwd=cwd, env=env or None, stdout=log, stderr=subprocess.STDOUT,
                     start_new_session=False)
    return True, f"restarted pid {pid} as {argv[0]} ({pattern})"


def cancel_task(driver: Driver, request_id: str) -> "tuple[bool, str]":
    """Ask for a cancel and report whether the MOTION ended, not whether the request was heard.

    `CancelTask.srv` keeps those apart on purpose: a cancel that cannot be confirmed must not
    let a new task be handed to a robot that is still moving. So `stopped` decides this
    function's answer and `accepted` is only recorded.
    """
    from fleet_interfaces.srv import CancelTask

    snapshot = driver.snap()
    row = next((t for t in (snapshot.get("tasks") or [])
                if t.get("request_id") == request_id), None)
    if row is None:
        return False, f"no task with request_id {request_id!r} is in the ledger"
    task_id = str(row.get("task_id") or "")
    if not task_id:
        return False, f"task {request_id!r} has no task_id yet"
    request = CancelTask.Request()
    request.task_id = task_id
    response = driver.call(driver.cancel, request, timeout=15.0)
    if response is None:
        return False, f"the cancel service did not answer for {task_id}"
    return bool(response.stopped), (f"cancel {task_id}: accepted={response.accepted} "
                                    f"stopped={response.stopped} state={response.state} "
                                    f"{response.reason_code}")


# --------------------------------------------------------------------------- #
# stage-2 actuators: the gz world, obstacle injection, and one Nav2 goal
# --------------------------------------------------------------------------- #
#
# THE DEFECT THESE EXIST TO CLOSE (D-P17-26). `fleet_core.trigger.KNOWN_ACTIONS`
# declares eighteen step actions and this file dispatched four, so ten cases named a fault
# with nothing behind it and were still given verdicts. Round 17 declared the stage-2
# capabilities and wrote the policy layer for them (`fleet_core.stage2_capabilities` has
# `pause_precondition`, `clock_jump_precondition`, `PauseObservation`) but never wrote the
# part that touches the simulator. These functions are that part.

#: How long to wait for a gz transport service reply. gz returns a Boolean.
GZ_TIMEOUT_MS = 5000

#: Height of an injected obstacle. The case declares only a radius; the height is fixed so
#: that the obstacle is taller than the lidar plane (laser_link sits ~0.30 m up) and
#: therefore visible to Nav2's costmap and to the localisation, which is the whole point.
OBSTACLE_HEIGHT_M = 0.80


def _world_name() -> str:
    """The gz world's name, read from the SDF the launch will load.

    Read from the FILE rather than hardcoded, for the reason `_common.world_name` gives: a
    hardcoded name that no longer matches the file produces "service not available", which
    reads like a broken Gazebo install rather than a stale constant. The path comes from
    FLEET_REPO_ROOT, which `scripts/env.sh` exports and this process inherits, so the runner
    and the launch resolve the same file by construction rather than by agreement.
    """
    import xml.etree.ElementTree as ET

    base = os.environ.get("FLEET_REPO_ROOT") or str(ROOT)
    path = Path(base) / "assets" / "worlds" / "warehouse.sdf"
    try:
        world = ET.parse(path).getroot().find("world")
        name = world.get("name") if world is not None else None
    except (OSError, ET.ParseError) as exc:
        raise CaseFailed(f"cannot read the world name from {path}: {exc}") from exc
    if not name:
        raise CaseFailed(f"{path} has no <world name=...>, so no gz service can be built")
    return name


def _gz_service(service: str, reqtype: str, reptype: str, req: str) -> "tuple[bool, str]":
    """Call one gz transport service through the CLI, in THIS process's partition.

    Through the CLI rather than through a ROS service client because gz's own services are
    not bridged on this project: `/world/<name>/control`, `/create` and `/remove` are gz
    transport endpoints, and `ros2 service list` shows none of them. The gz CLI shares the
    partition through `GZ_PARTITION`, which `env.sh` sets once and `ros2 launch` passes
    through to the simulator, so the runner reaches the same simulator the fleet is in.

    A reply of `true` is required. gz answers a malformed request with `false` rather than
    an error, and treating that as success is how a fault that never landed gets reported as
    one that did.
    """
    cmd = ["gz", "service", "-s", service, "--reqtype", reqtype, "--reptype", reptype,
           "--timeout", str(GZ_TIMEOUT_MS), "--req", req]
    try:
        proc = subprocess.run(cmd, capture_output=True, text=True, timeout=15)
    except (OSError, subprocess.SubprocessError) as exc:
        return False, f"could not run the gz CLI: {exc}"
    out = f"{proc.stdout or ''}{proc.stderr or ''}".strip().replace("\n", " ")
    if "true" not in out.lower():
        return False, f"{service} replied {out[:160]!r} (expected true)"
    return True, f"{service} accepted"


def spawn_obstacle(name: str, x: float, y: float, radius_m: float) -> "tuple[bool, str]":
    """Put a cylinder in the world under `name`.

    A cylinder rather than a box because the case declares a radius, and blocking a 1.2 m
    gap is then a statement about geometry a reader can check: F02 asks for 0.65 m, whose
    diameter 1.30 m exceeds the gap, which is what "blocked for good" means.

    `-allow_renaming false` so a name collision is an error rather than a second obstacle
    with a suffixed name that `despawn_obstacle` would then fail to remove.
    """
    world = _world_name()
    z = OBSTACLE_HEIGHT_M / 2.0
    sdf = (
        '<sdf version="1.10"><model name="%s"><static>true</static>'
        '<link name="link">'
        '<collision name="collision"><geometry><cylinder>'
        '<radius>%.4f</radius><length>%.4f</length>'
        '</cylinder></geometry></collision>'
        '<visual name="visual"><geometry><cylinder>'
        '<radius>%.4f</radius><length>%.4f</length>'
        '</cylinder></geometry></visual>'
        '</link></model></sdf>'
    ) % (name, radius_m, OBSTACLE_HEIGHT_M, radius_m, OBSTACLE_HEIGHT_M)
    cmd = ["ros2", "run", "ros_gz_sim", "create", "-world", world, "-name", name,
           "-string", sdf, "-x", repr(float(x)), "-y", repr(float(y)), "-z", repr(z),
           "-allow_renaming", "false"]
    try:
        proc = subprocess.run(cmd, capture_output=True, text=True, timeout=30)
    except (OSError, subprocess.SubprocessError) as exc:
        return False, f"could not spawn {name!r}: {exc}"
    blob = f"{proc.stdout or ''}{proc.stderr or ''}"
    if "error" in blob.lower() or "failed" in blob.lower():
        return False, f"{name!r} was not created: {blob.strip()[:160]!r}"
    time.sleep(FAULT_SETTLE_S)
    # The diameter is in the note because the diameter is the claim: F02 asks for 0.65 m and
    # says the corridor is blocked for good, and that is checkable only against the 1.2 m gap,
    # which is 2 * radius. Recording the radius alone leaves the reader to do the arithmetic.
    return True, (f"obstacle {name!r} spawned at ({x:.2f}, {y:.2f}) r={radius_m:.2f} m "
                  f"(d={2.0 * radius_m:.2f} m), h={OBSTACLE_HEIGHT_M:.2f} m")


def despawn_obstacle(name: str) -> "tuple[bool, str]":
    """Remove `name` from the world.

    Measured 2026-09-19 (F01's second step): `ros2 run ros_gz_sim delete_entity` HUNG and
    was killed by its own 30 s timeout, while `gz service` on the same world answered at
    once -- `pause_world` uses that path and works. So the world's own remove service is
    tried first and the ros_gz_sim CLI is kept as a FALLBACK rather than as the primary,
    and the note says which path answered so a reader can tell them apart.
    """
    if not name:
        return False, "despawn_obstacle needs a name; an empty name addresses nothing"
    ok, note = _gz_service(
        f"/world/{_world_name()}/remove", "gz.msgs.Entity", "gz.msgs.Boolean",
        f'name: "{name}", type: 2')
    if ok:
        time.sleep(FAULT_SETTLE_S)
        return True, f"obstacle {name!r} removed via the world's remove service"
    cmd = ["ros2", "run", "ros_gz_sim", "delete_entity", "--name", name, "--type", "6"]
    try:
        proc = subprocess.run(cmd, capture_output=True, text=True, timeout=30)
    except (OSError, subprocess.SubprocessError) as exc:
        return False, (f"{name!r} was not deleted: the world's remove service answered "
                       f"{note!r} and the ros_gz_sim CLI answered {exc}")
    blob = f"{proc.stdout or ''}{proc.stderr or ''}"
    if "error" in blob.lower() or "failed" in blob.lower():
        return False, f"{name!r} was not deleted: {blob.strip()[:160]!r}"
    time.sleep(FAULT_SETTLE_S)
    return True, f"obstacle {name!r} removed via the ros_gz_sim CLI"


#: The sim time sampled immediately before the world was last paused, or None.
#:
#: `/clock` stops publishing while the world is paused, so a step that needs the clock after
#: a pause cannot read it. The clock cannot move during a pause either, so the last value
#: before freezing IS the current value -- this is where that value is kept. It is cleared
#: on resume, because after a resume a stale remember would be a wrong number.
_FROZEN_AT: "float | None" = None


def set_world_paused(paused: bool) -> "tuple[bool, str]":
    """Freeze or unfreeze the simulator.

    The precondition is checked by the CALLER against a policy that already exists
    (`fleet_core.pause_precondition`), because whether the world is currently paused is a
    fact the case's own snapshot can answer and this function cannot: a second pause is
    indistinguishable from one applied once, so "the fleet survived the pause" would be a
    statement about a world that never stopped.
    """
    global _FROZEN_AT
    what = "pause" if paused else "unpause"
    if paused:
        # BEFORE the freeze: after it /clock goes quiet.
        _FROZEN_AT = _sim_now()
    ok, note = _gz_service(
        f"/world/{_world_name()}/control", "gz.msgs.WorldControl", "gz.msgs.Boolean",
        f"{what}: true")
    if not ok:
        _FROZEN_AT = None if paused else _FROZEN_AT
        return False, note
    if not paused:
        # The clock is moving again, so a remembered value is now stale by definition.
        _FROZEN_AT = None
    remembered = (f"; the clock was last read at {_FROZEN_AT:.3f} s, sampled just before "
                  f"it was frozen") if paused and _FROZEN_AT is not None else ""
    return True, f"world {'paused' if paused else 'resumed'} ({note}){remembered}"


def jump_clock(seconds: float) -> "tuple[bool, str]":
    """Move the sim clock, and say what it actually did.

    FORWARD is exact: gz's WorldControl has `run_to_sim_time`, a simulation time to run to
    and then pause, so a positive request is honoured to the millisecond.

    BACKWARD IS NOT, and this is the honest limit rather than a bug. This gz build has no
    "set the clock to T" and `run_to_sim_time` is documented as a time in the FUTURE, so an
    arbitrary earlier time cannot be requested. The only backwards movement available is a
    time reset, which takes the clock to zero. So a negative request is served by resetting
    time, and the note states the magnitude that ACTUALLY happened, measured before and
    after. I05 is titled "the clock resets mid-run" and its declared -30.0 s is a direction,
    not a promise this layer can keep to the tenth; a reader who needs the exact number has
    it in the note.
    """
    if seconds == 0.0:
        return False, "a zero-second jump does not move the clock, so it is not a fault"
    gz_now = _sim_now()
    source = "read live"
    if gz_now is None and _FROZEN_AT is not None:
        # The world is paused, so /clock is not publishing. The pause step sampled the clock
        # just before it froze, and the clock cannot move during a freeze, so that sample IS
        # the value now. The note says so, because a remembered number and a read number are
        # not the same kind of evidence.
        gz_now = _FROZEN_AT
        source = "the value sampled just before the world was frozen"
    if gz_now is None:
        return False, ("the sim clock could not be read, so nothing can be said about a "
                       "jump: /clock stops publishing while the world is paused, and this "
                       "run did not sample it before pausing. The magnitude is therefore "
                       "not measurable and is not claimed.")
    if seconds > 0.0:
        target = gz_now + float(seconds)
        ok, note = _gz_service(
            f"/world/{_world_name()}/control", "gz.msgs.WorldControl", "gz.msgs.Boolean",
            f"run_to_sim_time: {{sec: {int(target)}, nsec: 0}}, pause: true")
        if not ok:
            return False, note
        return True, (f"clock run to {target:.3f} s (a forward jump of {seconds:+.3f} s "
                      f"from {gz_now:.3f} s, {source}), then paused")
    ok, note = _gz_service(
        f"/world/{_world_name()}/control", "gz.msgs.WorldControl", "gz.msgs.Boolean",
        "reset: { time_only: true }")
    if not ok:
        return False, note
    after = _sim_now()
    if after is None:
        # Expected while paused: the reset is accepted and the value cannot be re-read. The
        # discontinuity is still in the recording's own time base, so it is not unobserved.
        return True, (f"clock reset from {gz_now:.3f} s ({source}), requested "
                      f"{seconds:+.3f} s. The post-reset value is not sampled because "
                      f"/clock is silent while the world is paused; gz has no arbitrary "
                      f"backwards set, so a reset is the only backwards move available.")
    return True, (f"clock reset: {gz_now:.3f} s -> {after:.3f} s, an actual jump of "
                  f"{after - gz_now:+.3f} s against a requested {seconds:+.3f} s "
                  f"(pre-reset value {source})")


def _sim_now() -> "float | None":
    """The simulator's current time, from `/clock`. None when it cannot be read.

    ONE reader on purpose, and the second one that was tried here is recorded as WITHDRAWN
    rather than merely absent.

    Measured 2026-09-19: `/clock` over ROS goes silent the moment the world is paused, and
    I05 pauses the world before it asks for a jump, so this returns None for that step. A
    `gz topic -e -t /clock` fallback was added to cover it and DID produce a number --
    `1789832190.000 s -> 1789832191.000 s` -- which is a pair of WALL-CLOCK stamps: the
    first `sec:` line of that dump belongs to a header, not to the Clock message. The
    recorder's own samples carry `t` of 23.5 / 76.3 / 857.7 for the same kind of run, which
    is what sim seconds look like here.

    A confident wrong number in the record is worse than a refusal, so the step refuses.
    If a future reader wants to restore it, parse the Clock message's own fields and check
    them against the recorder's `t` before trusting the magnitude.
    """
    try:
        proc = subprocess.run(
            ["ros2", "topic", "echo", "--once", "--field", "clock.sec", "/clock"],
            capture_output=True, text=True, timeout=15)
        return float((proc.stdout or "").strip().splitlines()[-1])
    except (OSError, subprocess.SubprocessError, ValueError, IndexError):
        return None


def drive_to_pose(runner: str, x: float, y: float) -> "tuple[bool, str]":
    """Drive one robot to (x, y) by sending it a Nav2 goal.

    Shells out to `scripts/nav_goal.py` rather than re-implementing the action client.
    That file already distinguishes the three things this must not conflate -- sent,
    accepted, reached -- and reports a missing action server as its own exit code, so a
    goal that never had a server is not recorded as a goal the robot failed.
    """
    if not runner:
        return False, "drive_to_pose needs a runner; without one there is no robot to drive"
    script = ROOT / "scripts" / "nav_goal.py"
    cmd = ["/usr/bin/python3", str(script), "--robot", runner,
           "--x", repr(float(x)), "--y", repr(float(y)), "--timeout", "120"]
    try:
        proc = subprocess.run(cmd, capture_output=True, text=True, timeout=180)
    except (OSError, subprocess.SubprocessError) as exc:
        return False, f"could not drive {runner} to ({x}, {y}): {exc}"
    codes = {0: "reached", 1: "rejected or aborted", 2: "no action server",
             3: "timed out, the robot may still be moving"}
    verdict = codes.get(proc.returncode, f"exit {proc.returncode}")
    return proc.returncode == 0, f"{runner} -> ({x:.2f}, {y:.2f}): {verdict}"


def fire_step(driver: Driver, step: dict, case_dir: "CaseDir") -> "tuple[bool, str]":
    """Carry out one step's action. Returns (ok, note).

    The note is what goes in the record, so it says what was DONE rather than what was asked
    for: "SIGTERM to pid(s) [1824]" and "battery of r01 forced to 0.12" can both be checked
    against the run, where "inject battery" cannot.
    """
    do = step.get("do") or {}
    action = str(do.get("action") or "")
    runner = str(do.get("runner") or "")
    if action == "battery_inject":
        ok = inject_battery(driver, runner, float(do["fraction"]))
        return ok, (f"battery of {runner} forced to {do['fraction']}" if ok
                    else f"the battery injection for {runner} was not acknowledged")
    if action == "cancel_task":
        return cancel_task(driver, str(do["request_id"]))
    if action == "kill_process":
        return kill_process(runner, str(do.get("node") or _trigger().TARGET_ALL))
    if action == "restart_process":
        log = (case_dir.path / f"restarted-{do.get('node')}.log").open("a", encoding="utf-8")
        return restart_process(runner, str(do.get("node") or ""), log)
    # ---- stage 2 (D-P17-26) ------------------------------------------------ #
    if action == "spawn_obstacle":
        # `name` is optional in the declared vocabulary, so a case that omits it gets one
        # derived from its own label rather than an empty string: `-name ""` builds a model
        # called "" that `despawn_obstacle` could never address.
        name = str(do.get("name") or f"obstacle-{case_dir.name}")
        return spawn_obstacle(name, float(do["x"]), float(do["y"]), float(do["radius_m"]))
    if action == "despawn_obstacle":
        return despawn_obstacle(str(do.get("name") or ""))
    if action == "pause_world":
        return set_world_paused(True)
    if action == "resume_world":
        return set_world_paused(False)
    if action == "jump_clock":
        return jump_clock(float(do["seconds"]))
    if action == "drive_to_pose":
        return drive_to_pose(runner, float(do["x"]), float(do["y"]))
    return False, f"action {action!r} is not implemented"


def _payload_holders(snapshot: dict) -> dict:
    """Who is holding what, right now. Read from the ledger, never from the robot's own word:
    "the payload is mine" is the party under test answering for itself."""
    return {str(p.get("payload_id")): str(p.get("holder") or "")
            for p in (snapshot.get("payloads") or [])
            if str(p.get("state")) == "HELD"}


def _charger_occupancy(snapshot: dict) -> "tuple[int, dict]":
    """The busiest single charger in one snapshot, and which robots are on which pad.

    Read from the charging book's own `occupant` map rather than inferred from positions: which
    pad a robot occupies is a reservation, and a robot standing NEXT TO a charger is not a
    capacity violation.
    """
    charges = snapshot.get("chargers") or {}
    per: dict = {}
    for robot, pad in (charges.get("occupant") or {}).items():
        per[pad] = per.get(pad, 0) + 1
    return (max(per.values()) if per else 0), per


def _declared_checks(names: list, *, final: dict, submitted: list[dict],
                     worst_charger_use: int, worst_charger_queue: int,
                     custodian_at_fault: dict, record: dict) -> "list[bool]":
    """The checks a case names in `behavior_checks`, evaluated from the state the run left.

    Each appends a sentence to `record["notes"]` BEFORE returning its verdict, so a failure
    says which reading it came from instead of only that it failed.
    """
    out: "list[bool]" = []
    for name in names:
        if name == "no_duplicate_task":
            wanted = [s["request_id"] for s in submitted]
            counts: dict = {}
            for task in (final.get("tasks") or []):
                rid = str(task.get("request_id"))
                if rid in wanted:
                    counts[rid] = counts.get(rid, 0) + 1
            dupes = {k: v for k, v in counts.items() if v > 1}
            out.append(not dupes)
            record["notes"].append(f"ledger rows per request_id: {counts}"
                                   + (f"; DUPLICATES {dupes}" if dupes else ""))
        elif name == "charger_capacity_respected":
            capacity = (final.get("chargers") or {}).get("capacity_per_charger")
            ok = bool(capacity) and worst_charger_use <= int(capacity)
            out.append(ok)
            record["notes"].append(
                f"busiest single charger held {worst_charger_use} robot(s) against a declared "
                f"capacity of {capacity}")
        elif name == "charger_contended":
            # Cleared by the busiest queue seen at any poll, not by the queue at the end: a
            # queue that formed and drained is exactly the thing being asked about, and at the
            # end of a successful run there is nothing left to see.
            out.append(worst_charger_queue >= 1)
            record["notes"].append(
                f"longest charger queue seen: {worst_charger_queue} robot(s)")
        elif name == "payload_not_rehomed":
            now = _payload_holders(final)
            moved = {pid: (held_by, now.get(pid))
                     for pid, held_by in custodian_at_fault.items() if now.get(pid) != held_by}
            out.append(not moved)
            record["notes"].append(
                f"custody at the fault {custodian_at_fault}, at the end {now}"
                + (f"; CHANGED HANDS {moved}" if moved else ""))
        else:
            # Fail closed. A name the runner does not implement must not read as a pass.
            out.append(False)
            record["notes"].append(f"behavior check {name!r} is not implemented by this runner")
    return out


def submit(driver: Driver, spec: dict, request_id: str, *, repeat: int = 1) -> list[dict]:
    from fleet_interfaces.srv import SubmitTask

    out = []
    for _ in range(max(1, repeat)):
        req = SubmitTask.Request()
        req.schema_version = "1"
        req.request_id = request_id
        req.kind = spec.get("kind", "station_transfer")
        req.pick_station = spec.get("pick", "")
        req.destination_station = spec.get("drop", "")
        req.required_capability = spec.get("capability", "CARRY")
        req.payload_mode = spec.get("payload_mode", "logical")
        req.payload_id = spec.get("cargo") or ""
        req.priority = int(spec.get("priority", 10))
        req.service_duration_s = float(spec.get("service_duration_s", 0.0))
        req.max_attempts = int(spec.get("max_attempts", 2))
        req.timeout_sim_s = float(spec.get("timeout_sim_s", 180.0))
        resp = driver.call(driver.submit, req, timeout=15.0)
        out.append({
            "request_id": request_id,
            "accepted": None if resp is None else bool(resp.accepted),
            "task_id": "" if resp is None else resp.task_id,
            "state": "" if resp is None else resp.state,
            "reason_code": "" if resp is None else resp.reason_code,
            "message": "" if resp is None else resp.message,
        })
    return out


# --------------------------------------------------------------------------- #
# one case
# --------------------------------------------------------------------------- #


def launch_error_tail(case_dir: "CaseDir", lines: int = 4) -> str:
    """The last few errors from a launch, for a case that could not start.

    F04 first failed as "the fleet did not report fresh state with Nav2 ACTIVE within 120s",
    which is true and useless: the cause was two lines up in the log
    (`Failed to change state for node: planner_server ... Aborting bringup`). A startup
    failure has to carry its cause, or the batch reports an environment problem as a case.
    """
    try:
        case_dir.stdout.flush()
        text = (case_dir.path / "stdout.log").read_text(encoding="utf-8", errors="replace")
    except OSError:
        return ""
    errors = [ln.strip() for ln in text.splitlines() if "[ERROR]" in ln or "Aborting" in ln]
    return " | ".join(errors[-lines:])


def _stop_and_judge(root: Path, case_dir: "CaseDir", recorder: TruthRecorder,
                    record: dict) -> None:
    """Stop the recorder, judge the case, and write both verdicts into `record`.

    Never raises. A recording that did not happen is a fact about the RUN, and it is recorded
    as UNKNOWN with its reason -- not thrown as an exception, which would replace whatever the
    case established with "the runner crashed".
    """
    audit = recorder.stop()
    source = recorder.samples_file()
    record["recorder"] = {
        "stop": recorder.stop_reason,
        "samples": audit.get("samples"),
        "truth_messages": audit.get("truth_messages"),
        "truth_widths": audit.get("truth_widths"),
        "claim_available": audit.get("claim_available"),
        "claim_failures": audit.get("claim_failures"),
        "book_available": audit.get("book_available"),
        "book_failures": audit.get("book_failures"),
        "stopped_by_signal": audit.get("stopped_by_signal"),
    }
    if source is None:
        record["task_outcome"] = record.get("task_outcome") or "NOT_RUN"
        record["safety_outcome"] = "UNKNOWN"
        record["safety_reason"] = (
            f"the recorder left no recording ({recorder.stop_reason or 'no samples file'}); "
            "section 1: no truth means safety is UNKNOWN, and UNKNOWN is not a pass")
        record["notes"].append("recorder produced no samples-*.jsonl")
        return

    record["claims"] = harvest_crossing_claims(case_dir)
    if record["claims"]:
        record["notes"].append(
            f"filed {len(record['claims'])} passage driver claim(s) for cross-checking: "
            + ", ".join(record["claims"]))

    try:
        judged = judge_case_dir(root, case_dir)
    except Exception as exc:  # a judge that dies must not read as a clean case
        record["safety_outcome"] = "UNKNOWN"
        record["safety_reason"] = f"the judge raised {type(exc).__name__}: {exc}"
        record["notes"].append(f"judge raised {type(exc).__name__}")
        return

    label = judge_path(case_dir, recorder.label)
    label.write_text(json.dumps(judged, indent=2, sort_keys=True), encoding="utf-8")
    outcome, why = safety_from_judgement(judged)
    record["safety_outcome"] = outcome
    record["safety_reason"] = why
    record["truth"] = {
        "judge_file": label.name,
        "samples": judged.get("samples"),
        "samples_file": judged.get("samples_file"),
        "summary": judged.get("summary"),
        "robots_judged": judged.get("robots_judged"),
        "robots_absent": judged.get("robots_absent"),
        "truth_has_heading": judged.get("truth_has_heading"),
        "failed": judged.get("failed"),
        "not_run": judged.get("not_run"),
        "held": judged.get("held"),
        "caveats": judged.get("caveats"),
        "checks": {c["check"]: c["verdict"] for c in judged.get("checks") or []},
    }
    record["notes"].append(
        f"judged {judged.get('samples')} samples: safety {outcome}, judge summary "
        f"{judged.get('summary')}")


def judge_path(case_dir: "CaseDir", label: str) -> Path:
    """Where this judge's own output goes.

    NOT `verdict-*.json`: those names belong to the drivers' self-reports, which
    `judge.load_case` reads as published claims to cross-check. Writing the judge's verdict
    under that name would make its next reading of the directory cite itself as a claim.
    """
    return case_dir.path / f"judge-{label}.json"


def run_case(case: str, body: dict, seed: int, base: Path, root: Path,
             manifest_path: Path) -> dict:
    case_dir = CaseDir(base, case, seed)
    case_dir.inputs(manifest_path, body)
    record: dict = {
        "case": case,
        "seed": seed,
        "title": body.get("title", ""),
        "task_outcome": "NOT_STARTED",
        "safety_outcome": "UNKNOWN",
        "safety_reason": "the case was not run, so no recording exists to judge",
        "expected_behavior_outcome": "NOT_RUN",
        "expected_safety": (body.get("expect") or {}).get("safety", "UNKNOWN"),
        "truth_source": body.get("truth", ""),
        "steps_planned": [],
        "steps_fired": [],
        "notes": [],
    }

    if body.get("requires"):
        record["task_outcome"] = "NOT_RUN"
        record["expected_behavior_outcome"] = "NOT_RUN"
        record["safety_outcome"] = "UNKNOWN"
        record["safety_reason"] = "the case was not run"
        record["blocked_on"] = list(body["requires"])
        case_dir.close()
        (case_dir.path / "summary.json").write_text(
            json.dumps(record, indent=2, sort_keys=True), encoding="utf-8")
        return record

    runs = list(body.get("runners") or ["r01"])
    fleet = Fleet(root, runs, body.get("scenario"), case_dir)
    driver = None
    recorder = None
    try:
        driver = Driver(runs)
        _say(f"    {case}: launching ({','.join(runs)}, scenario={body.get('scenario')})")
        fleet.start()
        # Before readiness, not after: the readiness window is part of the run, and a
        # recording that starts once everything is already fine cannot show what went wrong
        # on the way there.
        recorder = TruthRecorder(case_dir, runs, case_dir.name)
        recorder.start()
        _say(f"    {case}: recorder attached (label {case_dir.name}, {RECORDER_HZ:g} Hz)")
        if not wait_for_ready(driver, case_dir.stdout):
            raise CaseFailed("the fleet did not report fresh state with Nav2 ACTIVE within "
                             f"{READY_BUDGET_S:.0f}s")

        for fault in body.get("faults") or []:
            kind = fault.get("kind")
            if kind != "battery_inject":
                raise CaseFailed(f"fault trigger {kind!r} is not implemented by this runner")
            ok = inject_battery(driver, fault["runner"], float(fault["fraction"]))
            record["notes"].append(f"battery_inject {fault['runner']}: ok={ok}")
            if not ok:
                raise CaseFailed("the battery injection was not acknowledged")

        steps = _pending_steps(body, runs)
        record["steps_planned"] = [_trigger().describe(s) for s in steps]
        if steps:
            _say(f"    {case}: {len(steps)} step(s) waiting on a state")

        submitted: list[dict] = []
        pairs: "list[tuple[dict, list[dict]]]" = []
        for index, spec in enumerate(body.get("submit") or []):
            # A case may name its own request_id: I02 needs the SAME one twice with different
            # content, and a derived id per submission turned that case into two unrelated
            # requests that between them tested nothing.
            request_id = spec.get("request_id") or f"p7-{case}-s{seed}-{index + 1}"
            condition = spec.get("wait_for")
            if condition:
                budget = float(spec.get("wait_for_timeout_s",
                                       body.get("wait_for_timeout_s", WAIT_BUDGET_S)))
                held, why = wait_until(driver, condition, budget_s=budget)
                record["notes"].append(
                    f"wait_for {condition}: " + ("held" if held else f"NOT REACHED -- {why}"))
                if not held:
                    # The case's declared starting condition never happened, so continuing
                    # would run a different experiment from the one the case describes.
                    raise CaseFailed(f"the case could not arrange its arrival order: {why}")
                _say(f"    {case}: arrival order reached ({condition})")
            responses = submit(driver, spec, request_id, repeat=int(spec.get("repeat", 1)))
            pairs.append((spec, responses))
            submitted += responses
        record["submissions"] = submitted

        expect = body.get("expect") or {}
        deadline = time.monotonic() + float(body.get("observe_s", 300.0))
        final: dict = {}
        custodian_at_fault: dict = {}
        worst_charger_use = 0
        worst_charger_queue = 0
        while time.monotonic() < deadline:
            final = driver.snap()
            worst_charger_use = max(worst_charger_use, _charger_occupancy(final)[0])
            worst_charger_queue = max(worst_charger_queue, len(
                (final.get("chargers") or {}).get("queue") or []))
            # Steps BEFORE the exit condition, and that order is load-bearing: the two can
            # become true in the same tick, and a case that finished without its fault having
            # been delivered would be an ordinary run wearing a fault case's name.
            for step in list(steps):
                condition = step.get("when")
                if condition is not None:
                    verdict = _wait_for_module().condition_holds(final, condition)
                    if not verdict.holds:
                        continue
                do = step["do"]
                if do["action"] == "restart_process":
                    # Captured before the kill: what is in service right now is what the boot
                    # reconciliation will have to account for.
                    record["epoch_before_restart"] = final.get("epoch")
                    record["open_tasks_at_restart"] = {
                        str(t.get("request_id")): str(t.get("state"))
                        for t in (final.get("tasks") or [])
                        if str(t.get("state")) not in
                        ("SUCCEEDED", "FAILED", "CANCELED", "NEEDS_ATTENTION")
                    }
                custodian_at_fault.update(_payload_holders(final))
                ok, note = fire_step(driver, step, case_dir)
                record["steps_fired"].append({"note": note, "ok": ok})
                _say(f"    {case}: step fired -- {note}")
                if not ok:
                    raise CaseFailed(f"a declared fault could not be delivered: {note}")
                if do["action"] == "kill_process" and \
                        str(do.get("node")) == _trigger().TARGET_ALL:
                    record.setdefault("stopped_robots", []).append(str(do.get("runner")))
                steps.remove(step)
                # The fleet is not the one just read: a step that restarted a node, or stopped
                # one, has changed it. Re-read before deciding anything from a snapshot.
                final = driver.snap()
                break
            tasks = {t["request_id"]: t for t in (final.get("tasks") or [])}
            settled = all(
                tasks.get(s["request_id"], {}).get("state") in
                ("SUCCEEDED", "FAILED", "CANCELED", "NEEDS_ATTENTION")
                for s in submitted
            )
            if not submitted:
                # Nothing was submitted, so the case has to say what it is watching for. The
                # charge run is GENERATED from the battery state, so there is nothing to name:
                # it becomes evidence once it has actually charged. SUCCEEDED, not EXECUTING --
                # being assigned is not arriving, which this project has been caught blurring.
                charges = [t for t in (final.get("tasks") or [])
                           if t.get("kind") == "return_to_charge"]
                settled = (body.get("observe") == "charge_run"
                           and any(t.get("state") == "SUCCEEDED" for t in charges))
            elif body.get("await_charge"):
                # N08's "队列有推进" covers the charge runs as well, and those are generated by
                # the service rather than submitted here. A case that stopped as soon as its own
                # submissions were done would never observe the queue it is about -- and "no
                # charge run was ever created" is not a queue that advanced.
                charges = [t for t in (final.get("tasks") or [])
                           if t.get("kind") == "return_to_charge"]
                settled = settled and bool(charges) and all(
                    t.get("state") == "SUCCEEDED" for t in charges)
            if settled:
                break
            time.sleep(2.0)
        if steps:
            # Section 5: 超时未到触发条件为 PRECONDITION_NOT_REACHED，不算故障测试通过. A case
            # that did not deliver its fault is not a fault case, and saying so is the whole
            # difference between "the fleet survived it" and "it never happened".
            pending = "; ".join(_trigger().describe(s) for s in steps)
            raise CaseFailed("the case's trigger never arrived, so its fault was never "
                             f"delivered: {pending}")
        record["custodian_at_fault"] = custodian_at_fault
        record["worst_charger_use"] = worst_charger_use
        record["worst_charger_queue"] = worst_charger_queue
        (case_dir.path / "summary_state.json").write_text(
            json.dumps(final, indent=2, sort_keys=True), encoding="utf-8")

        # Stop recording before the teardown, so the recording covers the run and not the
        # shutdown, and so a case that has finished cannot have its window extended by an
        # unrelated failure afterwards.
        if recorder is not None:
            _stop_and_judge(root, case_dir, recorder, record)
            recorder = None

        tasks = {t["request_id"]: t for t in (final.get("tasks") or [])}
        observed = [tasks.get(s["request_id"], {}) for s in submitted]
        states = [t.get("state", "") for t in observed]
        record["observed_states"] = states
        record["chargers"] = final.get("chargers")
        record["refusals"] = final.get("refusals")
        record["tick_errors"] = final.get("tick_errors")

        # The task outcome: the declared expectation, checked per submission.
        wanted = expect.get("task", "SUCCEEDED")
        if not submitted:
            # Nothing was submitted, so the case has to say what to look for. F04's charge run
            # is GENERATED by the task service from the battery state; writing an outcome here
            # without observing anything would be a label with no evidence, which is the exact
            # shape this project has paid for three times.
            observe = body.get("observe")
            if observe != "charge_run":
                raise CaseFailed("a case with nothing to submit must declare "
                                 "`observe: charge_run` so the runner knows what to check")
            charges = [t for t in (final.get("tasks") or [])
                       if t.get("kind") == "return_to_charge"]
            executed = [t for t in charges
                        if t.get("state") in ("EXECUTING", "SUCCEEDED")]
            record["notes"].append(
                f"charge runs: {[(t['task_id'], t['state']) for t in charges]}")
            record["task_outcome"] = "SUCCEEDED" if executed else "NOT_STARTED"
        elif wanted == "NEEDS_ATTENTION":
            record["task_outcome"] = (
                "SUCCEEDED" if "SUCCEEDED" in states
                else "NEEDS_ATTENTION" if "NEEDS_ATTENTION" in states
                else "FAILED" if "FAILED" in states else "NOT_STARTED")
        else:
            bad = [s for s in states if s not in ("SUCCEEDED",)]
            record["task_outcome"] = "SUCCEEDED" if not bad else (bad[0] or "NOT_STARTED")

        # The behaviour outcome: the things the case exists to observe. Each check is a
        # property the case's own declared expectation names, evaluated from the state the
        # run left behind -- never from the fact that a task said SUCCEEDED.
        behaviour = []
        # Only cargos whose submission the service ACCEPTED. A refused submission (I02's
        # conflicting request) never created a payload, so counting it made a correct refusal
        # look like a missing delivery.
        accepted_specs = [
            spec for spec, responses in pairs
            if responses and responses[0].get("accepted") is True
        ]
        declared_cargo = [s["cargo"] for s in accepted_specs if s.get("cargo")]
        if declared_cargo:
            # "正确领取/交付，没有实体装卸声明": every cargo ends DELIVERED, none is left held.
            payloads = {p["payload_id"]: p for p in (final.get("payloads") or [])}
            delivered = [c for c in declared_cargo
                         if payloads.get(c, {}).get("state") == "DELIVERED"]
            still_held = [c for c in declared_cargo
                          if payloads.get(c, {}).get("state") == "HELD"]
            behaviour.append(len(delivered) == len(declared_cargo) and not still_held)
            record["notes"].append(
                f"cargo delivered {len(delivered)}/{len(declared_cargo)}, held at the end: "
                f"{still_held}")
        wanted_runners = [s.get("runner") for s in (body.get("submit") or []) if s.get("runner")]
        if wanted_runners:
            # "并发运动，控制不串线": each task ran on the robot the case named for it, and the
            # robots are distinct. An allocator that crossed them over would still report two
            # SUCCEEDED tasks.
            got = [t.get("robot_id", "") for t in observed]
            ok = got == wanted_runners and len(set(wanted_runners)) == len(wanted_runners)
            behaviour.append(ok)
            record["notes"].append(f"tasks ran on {got}; the case named {wanted_runners}")
        ledger = _crossing_ledger()
        intervals = ledger.crossing_intervals(
            ledger.read_events(case_dir.path / "runtime" / "events.jsonl"))
        record["crossings_observed"] = ledger.for_recording(intervals)
        if body.get("crossing_exclusive"):
            # Section 5's "ordered use, no rear-ending" for N04/N05/N06 and the capacity-1
            # claim for N07. Read from the event stream rather than the snapshot: the
            # snapshot's maps are last-writer state, and "who went first" is gone by the end of
            # the run. Neither property here carries a threshold -- an overlap is an
            # inequality between two timestamps from one file, so there is no margin to widen.
            ok, detail = ledger.exclusive(intervals)
            behaviour.append(ok)
            record["notes"].append(f"corridor exclusivity: {detail}")
        if body.get("crossing_order"):
            wanted = [str(r) for r in body["crossing_order"]]
            got = ledger.first_crossing_order(intervals)
            behaviour.append(got == wanted)
            record["notes"].append(
                f"first crossing order: {got}; the case declared {wanted}")
        if body.get("crossing_exclusive") or body.get("crossing_order"):
            # Recorded and not asserted. "r02 waited" is evident from the acquire stage's own
            # request count, and a case that asserted it would be asserting the driver's
            # internal retry policy -- but a reader asking "did the corridor actually make
            # anybody wait" should not have to open four JSON files to find out.
            record["crossing_acquire_requests"] = _acquire_requests(case_dir.path)

        for spec, responses in pairs:
            if spec.get("expect") in ("IDEMPOTENCY_CONFLICT", "REQUEST_CONFLICT"):
                # A conflicting request_id is refused IN THE RESPONSE; it never reaches the
                # ledger, so there is no row to read a reason from.
                # CONTRACTS section 2 defines the code as IDEMPOTENCY_CONFLICT; the message
                # mentions the ledger's REQUEST_CONFLICT exception, which is where the first
                # version of this check looked by mistake.
                code = responses[-1].get("reason_code") if responses else ""
                behaviour.append(bool(responses) and responses[-1].get("accepted") is False
                                 and code == "IDEMPOTENCY_CONFLICT")
                if responses:
                    record["notes"].append(f"conflicting submit answered {code!r}")
        if len(body.get("submit") or []) == 1 and body["submit"][0].get("repeat"):
            # I01: ten identical submissions must produce exactly one task row.
            same = [t for t in (final.get("tasks") or [])
                    if t["request_id"] == submitted[0]["request_id"]]
            behaviour.append(len(same) == 1)
            record["notes"].append(f"I01: {len(same)} ledger row(s) for ten submissions")
        if not submitted and body.get("observe") == "charge_run":
            # The case declares this observation AS its behaviour claim, so it is judged here
            # rather than left as NOT_RUN while the note next to it says the run succeeded.
            behaviour.append(record["task_outcome"] == "SUCCEEDED")
        if body.get("behavior_checks"):
            behaviour += _declared_checks(list(body["behavior_checks"]), final=final,
                                          submitted=submitted,
                                          worst_charger_use=worst_charger_use,
                                          worst_charger_queue=worst_charger_queue,
                                          custodian_at_fault=custodian_at_fault,
                                          record=record)

        for stopped in record.get("stopped_robots") or []:
            # F06's claim, "不再分配，其他车执行可行任务", stated so it can fail: a robot whose
            # processes were stopped must not be named for a task afterwards. The pool has to
            # notice, and the snapshot is where that is visible.
            named = [t.get("task_id") for t in (final.get("tasks") or [])
                     if str(t.get("robot_id")) == stopped]
            behaviour.append(not named)
            record["notes"].append(f"tasks on the stopped robot {stopped} at the end: {named}")

        if "epoch_before_restart" in record:
            # F11's claim, "新 epoch 对账，旧任务不重复执行". Both halves are read from the
            # service's own report: the epoch is a fresh value per process, so a value that did
            # not change means the process did not come back; and every task that was in
            # service when it died must be accounted for -- parked BY THE RECONCILIATION, with
            # that reason, or legitimately finished.
            after = final.get("epoch")
            rows = {str(t.get("request_id")): t for t in (final.get("tasks") or [])}
            accounted = {rid: (str((rows.get(rid) or {}).get("state") or state),
                               str((rows.get(rid) or {}).get("reason") or ""))
                         for rid, state in (record.get("open_tasks_at_restart") or {}).items()}
            unnoticed = {rid: pair for rid, pair in accounted.items()
                         if pair[1] != "INTERRUPTED_BY_RESTART" and pair[0] != "SUCCEEDED"}
            behaviour.append(after != record["epoch_before_restart"] and not unnoticed)
            record["notes"].append(
                f"task-service epoch {record['epoch_before_restart']} -> {after}; in service at "
                f"the restart: {accounted}" + (f"; NOT ACCOUNTED FOR {unnoticed}" if unnoticed
                                               else ""))

        record["expected_behavior_outcome"] = (
            "PASS" if behaviour and all(behaviour) else
            "NOT_RUN" if not behaviour else "FAIL")

        leftovers = fleet.stop()
        record["teardown"] = leftovers
        record["notes"].append(f"teardown leftover: {leftovers}")
    except CaseFailed as exc:
        record["task_outcome"] = "NOT_RUN"
        record["expected_behavior_outcome"] = "NOT_RUN"
        record["notes"].append(f"could not run: {exc}")
        tail = launch_error_tail(case_dir)
        if tail:
            record["notes"].append(f"launch said: {tail}")
            record["startup_error"] = tail
    except Exception as exc:  # a batch must record a failure, not die in it
        record["task_outcome"] = "NOT_RUN"
        record["expected_behavior_outcome"] = "NOT_RUN"
        record["notes"].append(f"{type(exc).__name__}: {exc}")
    finally:
        if recorder is not None:
            # The case failed before it got to the judging step. Still stop it -- a recorder
            # left running outlives the case and would poll the NEXT case's services.
            try:
                _stop_and_judge(root, case_dir, recorder, record)
            except Exception as exc:
                record["notes"].append(f"recorder teardown failed: {type(exc).__name__}")
        if driver is not None:
            try:
                driver.close()
            except Exception:
                pass
        if fleet.proc is not None:
            try:
                fleet.stop()
            except Exception:
                pass
        case_dir.close()
        (case_dir.path / "summary.json").write_text(
            json.dumps(record, indent=2, sort_keys=True), encoding="utf-8")
    return record


# --------------------------------------------------------------------------- #
# the batch
# --------------------------------------------------------------------------- #


def write_report(base: Path, records: list[dict], stamp: str, plan: dict) -> None:
    lines = [
        f"# 009 regression_v1 batch {stamp}",
        "",
        f"Cases planned: **{plan['total']}** "
        f"({plan['runnable']} runnable, {plan['blocked']} blocked)",
        "",
        "| case | seed | task | safety | behaviour | note |",
        "|---|---|---|---|---|---|",
    ]
    for r in records:
        note = "; ".join(r.get("notes") or [])[:110]
        if r.get("blocked_on"):
            note = "blocked on " + ", ".join(r["blocked_on"])
        lines.append(
            f"| {r['case']} | {r['seed']} | {r['task_outcome']} | {r['safety_outcome']} | "
            f"{r['expected_behavior_outcome']} | {note} |"
        )
    unresolved = [r for r in records if r["safety_outcome"] == "UNKNOWN"
                  and r["task_outcome"] != "NOT_RUN"]
    lines += [
        "",
        f"**Claims filed for cross-checking**: "
        f"{sum(len(r.get('claims') or []) for r in records)} passage driver report(s); a case "
        "with none has `driver_claims_corroborated_by_truth: NOT_RUN`, which is what a check "
        "that was not performed should say.",
        "",
        f"**Safety**: {sum(1 for r in records if r['safety_outcome'] == 'PASS')} PASS, "
        f"{sum(1 for r in records if r['safety_outcome'] == 'FAIL')} FAIL, "
        f"{sum(1 for r in records if r['safety_outcome'] == 'UNKNOWN')} UNKNOWN.",
        "",
        "Every case that ran carries a `samples-<case>.jsonl` written by"
        " `fleet_evaluation`'s recorder and a `judge-<case>.json` written from it by"
        " `fleet_evaluation.judge`, which reads the simulator's own body poses rather than"
        " the robots' estimates. `safety_outcome` is derived from the judge's two safety"
        " properties only; pose error and the claim cross-checks are in `truth.checks` where"
        " they can be read as what they are.",
        "",
        ("A safety verdict that could not be established is an infrastructure result, not a"
         " clean one: " + ", ".join(f"{r['case']} ({r['safety_reason'][:60]})"
                                    for r in unresolved)
         if unresolved else
         "Every case that ran has a safety verdict, so section 1's UNKNOWN condition is met"
         " for this batch."),
        "",
    ]
    (base / "report.md").write_text("\n".join(lines), encoding="utf-8")


def main(argv: "list[str] | None" = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--manifest", default=str(MANIFEST_DEFAULT))
    ap.add_argument("--root", default=str(ROOT))
    ap.add_argument("--only", default="", help="comma-separated case ids")
    ap.add_argument("--seeds", default="",
                    help="comma-separated seed numbers; run ONLY those seeds for the "
                         "selected cases. Seed 1 is normally implied for every case and "
                         "the manifest's seed_repeats adds 2 and 3, which makes one case "
                         "a ~21 minute run. On a host that can be shut down or have its "
                         "client killed at any moment, a run that long cannot be "
                         "completed, so this overrides the seed set instead of extending "
                         "it. It changes nothing about what a seed means.")
    ap.add_argument("--dry-run", action="store_true",
                    help="resolve and print the plan; starts nothing")
    args = ap.parse_args(argv)

    root = Path(args.root).resolve()
    manifest_path = Path(args.manifest)
    if not manifest_path.is_absolute():
        manifest_path = root / manifest_path

    _say("=" * 78)
    _say("  009 regression_v1 -- TEST_AND_ACCEPTANCE section 5")
    _say("=" * 78)
    try:
        import rclpy  # noqa: F401
    except ModuleNotFoundError:
        # Caught here rather than per case. The first version of this runner recorded FIVE
        # `NOT_RUN` verdicts from one wrong interpreter, and a case verdict that is really an
        # environment error is a lie about the system under test.
        _say("  cannot import rclpy: this runner drives ROS services.")
        _say("  run it as `bash scripts/batch.sh ...`, or `source scripts/env.sh` first and")
        _say("  use /usr/bin/python3. The project .venv is deliberately ROS-free.")
        return 3
    try:
        manifest = load_manifest(manifest_path, root)
    except SystemExit as exc:
        _say(f"  manifest refused: {exc}")
        return 2

    runs = planned_runs(manifest)
    wanted = [c.strip() for c in args.only.split(",") if c.strip()]
    if wanted:
        unknown = [c for c in wanted if c not in {r["case"] for r in runs}]
        if unknown:
            _say(f"  --only names unknown case(s): {unknown}")
            return 2
        runs = [r for r in runs if r["case"] in wanted]

    seeds_wanted = {int(s) for s in args.seeds.split(",") if s.strip()}
    if seeds_wanted:
        # An override, not an extension: a caller who names one seed wants one run.
        seeds_in_plan = {r["seed"] for r in runs}
        absent = sorted(seeds_wanted - seeds_in_plan)
        if absent:
            _say(f"  --seeds names seed(s) the plan does not contain: {absent}")
            _say(f"  seeds this plan has: {sorted(seeds_in_plan)}")
            return 2
        runs = [r for r in runs if r["seed"] in seeds_wanted]

    runnable = [r for r in runs if not r["body"].get("requires")]
    plan = {"total": len(runs), "runnable": len(runnable),
            "blocked": len(runs) - len(runnable)}
    _say(f"  cases: {plan['total']}  runnable: {plan['runnable']}  "
         f"blocked: {plan['blocked']}")

    for r in runs:
        if r["body"].get("requires"):
            _say(f"    [NOT_RUN] {r['case']:>4} seed {r['seed']}  needs "
                 f"{', '.join(r['body']['requires'])}")
        else:
            _say(f"    [  plan ] {r['case']:>4} seed {r['seed']}  "
                 f"{r['body'].get('title', '')}")

    if args.dry_run:
        _say("")
        _say("  dry run: nothing was started")
        return 0

    stamp = time.strftime("%Y%m%dT%H%M%SZ", time.gmtime())
    base = root / "reports" / f"batch_{stamp}"
    base.mkdir(parents=True, exist_ok=False)
    (base / "manifest.json").write_text(json.dumps({
        "stamp": stamp,
        "manifest": str(manifest_path.relative_to(root)),
        "backend": manifest.get("backend", "gazebo_nav2"),
        "plan": plan,
        "machine": os.uname().nodename,
    }, indent=2, sort_keys=True), encoding="utf-8")

    _say("")
    records = []
    for r in runs:
        _say(f"  --- {r['case']} seed {r['seed']} " + "-" * 40)
        records.append(run_case(r["case"], r["body"], r["seed"], base, root, manifest_path))

    (base / "summary.json").write_text(json.dumps(records, indent=2, sort_keys=True),
                                       encoding="utf-8")
    write_report(base, records, stamp, plan)

    ran = [r for r in records if r["task_outcome"] != "NOT_RUN"]
    failed = [r for r in ran if r["task_outcome"] not in ("SUCCEEDED", "NEEDS_ATTENTION")]
    bad_behaviour = [r for r in ran if r["expected_behavior_outcome"] == "FAIL"]
    blocked = [r for r in records if r["task_outcome"] == "NOT_RUN"]
    # A declared safety expectation that did not hold is a case failure (section 9, exit 4).
    # A safety property that could not be judged at all is a sampling problem (exit 5).
    # Collapsing the two would let "we could not look" read as "we looked and it was fine",
    # which is the one confusion section 1 exists to prevent.
    safety_contradicted = [r for r in ran
                           if r["safety_outcome"] != r.get("expected_safety", "UNKNOWN")]
    safety_unjudged = [r for r in safety_contradicted if r["safety_outcome"] == "UNKNOWN"]

    _say("")
    _say(f"  ran {len(ran)} of {len(records)} planned run(s)"
         + (f"; {len(blocked)} NOT_RUN" if blocked else ""))
    _say(f"  safety: " + ", ".join(f"{r['case']}={r['safety_outcome']}"
                                   for r in records if r["task_outcome"] != "NOT_RUN"))
    _say(f"  report: {base.relative_to(root)}/report.md")
    if failed or bad_behaviour:
        _say(f"  RESULT: a case's declared outcome did not happen "
             f"({len(failed)} task, {len(bad_behaviour)} behaviour) -- exit 4")
        return 4
    hard_safety = [r for r in safety_contradicted if r not in safety_unjudged]
    if hard_safety:
        _say(f"  RESULT: safety contradicted its declaration in "
             f"{len(hard_safety)} case(s) -- exit 4")
        return 4
    if safety_unjudged:
        _say(f"  RESULT: {len(safety_unjudged)} case(s) declared a safety verdict the "
             "recording could not establish -- exit 5")
        return 5
    if blocked:
        _say(f"  RESULT: no case contradicted its declaration, but {len(blocked)} case(s) are "
             "NOT_RUN -- the batch is not accepted (exit 5)")
        return 5
    _say("  RESULT: every case ran, every declared outcome held, and safety was judged from "
         "truth -- exit 0")
    return 0


if __name__ == "__main__":
    sys.exit(main())
