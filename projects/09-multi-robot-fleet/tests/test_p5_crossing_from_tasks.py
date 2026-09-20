"""A crossing must be driven by the passage protocol, and the dispatcher must survive losing one.

WHAT THIS FILE CAN AND CANNOT TEST
----------------------------------
Everything added for the corridor crossing lives in `fleet_ros`'s dispatcher, which imports
rclpy, and the core suite runs in a clean shell with no ROS on purpose (that is what makes the
"fleet_core does not depend on ROS" invariant provable). So the checks below are structural,
in the same style as `test_p5_nav2_readiness.py`, and they are honest about being structural:
they pin the SHAPE that matters and the live run is what establishes the behaviour. Two of them
are not structural at all -- the dataclass and the attempt accounting are exercised through the
code path directly, because they are pure Python:

  * a failed crossing spends exactly one attempt, and parks the task on the last one
  * the direction question is asked of geometry, not read from the leg's tag

The structural ones exist because the failure they prevent is silent: a dispatcher that falls
through to a plain Nav2 goal for a crossing leg looks completely healthy and drives the robot
into the barrier. That is a property worth pinning in a test that runs in two seconds.
"""

from __future__ import annotations

import ast
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
NODE = ROOT / "src" / "fleet_ros" / "fleet_ros" / "task_service_node.py"

TREE = ast.parse(NODE.read_text(encoding="utf-8"), filename=str(NODE))


def _function(name: str) -> ast.FunctionDef:
    for node in ast.walk(TREE):
        if isinstance(node, ast.FunctionDef) and node.name == name:
            return node
    raise AssertionError(f"{name} is missing from {NODE.name}")


def _calls(node: ast.AST, name: str) -> list[ast.Call]:
    out = []
    for call in [n for n in ast.walk(node) if isinstance(n, ast.Call)]:
        target = call.func
        label = getattr(target, "attr", None) or getattr(target, "id", None)
        if label == name:
            out.append(call)
    return out


def _source(node: ast.AST) -> str:
    return ast.unparse(node)


# --------------------------------------------------------------------------- #
# the crossing is a process, and it is the passage driver
# --------------------------------------------------------------------------- #


def test_the_driver_is_the_one_p4_was_accepted_with():
    """Not a re-implementation. `staged_crossing` already renews its permit mid-crossing, and
    a second implementation of that sequence would drift until a robot is stopped inside the
    corridor by its own gate."""
    start = _function("_start_crossing")
    dump = _source(start)
    assert "'staged_crossing'" in dump
    for flag in ("--robot", "--direction", "--task-id", "--report"):
        assert f"'{flag}'" in dump, f"the driver invocation is missing {flag}"
    assert "--timeout-s" in dump, (
        "the driver's own timeout must be the dispatcher's budget, or the two disagree about "
        "when the crossing failed")


def test_the_driver_gets_its_own_process_group():
    """Cancellation kills the group. Without a new session, SIGTERM to the driver's group is
    either impossible or reaches this node too."""
    popen = _calls(_function("_start_crossing"), "Popen")
    assert popen, "the driver must be started as a process"
    flags = {kw.arg for kw in popen[0].keywords}
    assert "start_new_session" in flags


def test_a_missing_driver_does_not_become_a_plain_goal():
    """The whole point of the refusal this replaced.

    When the passage driver cannot be started at all, issuing the leg as an ordinary Nav2 goal
    sends the robot at the barrier. The task must be parked with a reason instead, and the leg
    must not be sent.
    """
    issue = _function("_issue_legs")
    # Find the branch that handles "the driver did not start".
    branches = [n for n in ast.walk(issue)
                if isinstance(n, ast.If) and "_start_crossing" in _source(n)]
    assert branches, "_issue_legs does not consult _start_crossing"
    guarded = [n for n in ast.walk(issue) if isinstance(n, ast.If) and "started is None" in _source(n)]
    assert guarded, "_issue_legs ignores the failure to start the driver"
    body = _source(guarded[0])
    assert "fail(" in body, "the task must be parked with a reason"
    assert "send_goal_async" not in body, (
        "falling through to a plain goal for a crossing leg drives the robot into the barrier")


def test_a_leg_that_already_crossed_is_not_crossed_again():
    """The robot's believed position lags the motion.

    After the driver finishes, this node's own view of the robot can still read as the near
    side, so asking the same question again answers "cross" and would send it straight back.
    """
    issue = _source(_function("_issue_legs"))
    assert "self.crossed" in issue, "the dispatcher has no memory of a paid-for crossing"
    poll = _source(_function("_poll_crossings"))
    assert "self.crossed[item.key] = now" in poll, "a successful crossing is not remembered"
    assert "horizon" in poll, "the crossing memory is never pruned"


# --------------------------------------------------------------------------- #
# the crossing is the robot's live command generation
# --------------------------------------------------------------------------- #


def test_a_crossing_blocks_a_second_leg():
    """CONTRACTS section 4: one live command generation per robot. While the driver is moving
    the robot, this node must not issue a leg to it as well."""
    issue = _source(_function("_issue_legs"))
    assert "rid in self.crossings" in issue
    assert "rid in self.pending" in issue


def test_a_crossing_pins_its_robot_against_release():
    """A release arriving while the driver is running would let the allocator hand this robot a
    second task and leave the first one driving it through the corridor."""
    release = _source(_function("_release_robot"))
    assert "rid in self.crossings" in release
    assert "ownership_refusals" in release


def test_cancel_stops_the_driver():
    """The defect this project paid for once already: recording a cancel and telling nobody.

    A crossing is driven by a process this node owns, so nothing else can stop it.
    """
    cancel = _source(_function("_srv_cancel"))
    assert "self.crossings" in cancel
    assert "_kill_crossing" in cancel


def test_a_crossing_is_reaped_in_the_tick_not_waited_on():
    """The same rule as the leg futures: poll, never wait. A wait inside the tick callback is
    how P6's display deadlocked, and how a probe in this project measured its own queue."""
    tick = _source(_function("_tick"))
    assert "self._poll_crossings(now)" in tick
    poll = _source(_function("_poll_crossings"))
    assert ".poll()" in poll, "the driver is not reaped with poll()"
    for forbidden in ("wait(", "communicate("):
        assert forbidden not in poll, f"_poll_crossings calls {forbidden}; it must not wait"


def test_the_snapshot_reports_what_is_crossing():
    """`EXECUTING` and "the robot is moving" are different statements, and only one of them is
    visible anywhere else."""
    snapshot = _source(_function("snapshot"))
    assert "'crossings'" in snapshot or '"crossings"' in snapshot
    assert "self.crossings" in snapshot


# --------------------------------------------------------------------------- #
# attempts, and the failure that is not a navigation failure
# --------------------------------------------------------------------------- #


def _signature_arity(fn: ast.FunctionDef) -> tuple[int, int, int]:
    """(positional parameters including self, of those, how many have defaults, kw-only)."""
    args = fn.args
    positional = len(args.posonlyargs) + len(args.args)
    # Defaults attach to the LAST n positional parameters, which is why `now` being required and
    # `reason` being optional is not something a count of parameters can tell you.
    return positional, len(args.defaults), len(args.kwonlyargs)


def _call_arity(call: ast.Call) -> tuple[int, int]:
    return len([a for a in call.args if not isinstance(a, ast.Starred)]), len(call.keywords)


def test_every_crossing_call_matches_its_signature():
    """A test for the family, not the instance.

    The live run died in a 180-second restart loop because one call site passed one argument
    too few. Every other test in this file checks that a name is PRESENT somewhere, which is
    why none of them noticed: a signature is not a name, and a call that cannot run is not a
    call. This walks the calls of the crossing methods and requires each to be satisfiable.
    """
    tree = TREE
    for owner in ("_finish_crossing", "_kill_crossing", "_start_crossing", "_crossing_dir"):
        fn = _function(owner)
        positional, defaults, kwonly = _signature_arity(fn)
        required = positional - 1 - defaults   # minus self, minus the optional ones
        for call in [n for n in ast.walk(tree)
                     if isinstance(n, ast.Call)
                     and getattr(n.func, "attr", None) == owner]:
            args, kwargs = _call_arity(call)
            assert args <= positional, (
                f"{owner} is called with {args} positional argument(s); its signature takes "
                f"{positional} including self")
            assert args + kwargs >= required + kwonly, (
                f"{owner} is called with {args} positional and {kwargs} keyword argument(s), "
                f"but {required + kwonly} are required -- this is the exact shape of the bug "
                "that put the dispatcher into a 180-second restart loop")


def test_a_failed_crossing_spends_one_attempt_and_then_parks():
    finish = _source(_function("_finish_crossing"))
    assert "run.attempts += 1" in finish
    assert "run.spec.max_attempts" in finish
    assert "self.machine.fail(" in finish
    assert "self._release_robot(" in finish, (
        "a parked task must not keep holding its robot; that is the shape of D-P5-19's silent "
        "no-op")


def test_a_passage_that_cannot_be_acquired_is_not_reported_as_a_navigation_failure():
    """"The corridor is busy or the geometry refuses you" and "Nav2 could not get there" need
    opposite responses, and the reason code is what an operator reads."""
    poll = _source(_function("_poll_crossings"))
    assert "ROUTE_REQUIRES_PERMIT" in poll
    assert "'acquire'" in poll


def test_a_cancelled_task_does_not_get_a_failed_crossing_written_into_it():
    """"A cancel is pending" is a ledger FIELD, not a task state.

    This test used to assert that the literal `CANCEL_REQUESTED` appeared in `_poll_crossings`.
    `TaskState` has no such member, so the assertion pinned a name whose only possible effect at
    run time is an AttributeError -- and it passed for two rounds while every harvest of a
    crossing raised one. A substring assertion cannot falsify an attribute access; asserting the
    predicate that answers the question can.
    """
    poll = _source(_function("_poll_crossings"))
    assert "task.cancel_requested" in poll
    assert "crossing_reaped" in poll


def test_no_code_reads_a_TaskState_member_that_does_not_exist():
    """Every `TaskState.<member>` in the module must exist on the enum.

    The failure this prevents is invisible to every other check the project has: the name is
    defined, so `check_undefined_names.py` is satisfied; the branch is only reached when a live
    task is being harvested, so no test that runs in two seconds reaches it; and a substring
    assertion over the source finds the spelling it was written against. It took two live runs
    for the AttributeError to surface, and the only trace was a line the blanket handler prints.
    """
    members = {n.attr for n in ast.walk(TREE)
               if isinstance(n, ast.Attribute)
               and isinstance(n.value, ast.Name) and n.value.id == "TaskState"}
    assert members, ("no `TaskState.<member>` is read anywhere in the module; this check has "
                     "drifted away from the code it is supposed to guard")
    import fleet_core
    from fleet_core.domain import TaskState
    assert fleet_core  # the import path the rest of this file uses, kept explicit
    missing = sorted(m for m in members if not hasattr(TaskState, m))
    assert not missing, (
        f"{missing} are read as TaskState members and do not exist. The branch that reads them "
        "raises the first time it is taken -- which is when a live task is being harvested, the "
        "least convenient moment available")


def _fields(cls: str) -> set[str]:
    """The field names of a dataclass, read from its annotations.

    NOT a substring search over the class source. `PendingCrossing`'s docstring says a leg has
    "a goal handle and two futures" and that this type deliberately has neither, so a search
    for those words finds them in a comment about their absence -- and the assertion fails
    against code that is exactly right.
    """
    node = next(n for n in ast.walk(TREE)
                if isinstance(n, ast.ClassDef) and n.name == cls)
    return {t.id for child in node.body if isinstance(child, ast.AnnAssign)
            for t in [child.target] if isinstance(t, ast.Name)}


def test_the_crossing_state_record_is_not_a_leg():
    """A leg has a goal handle and two futures; a crossing has a process and a report file.
    Sharing one type is how a reaper ends up waiting on something that never completes."""
    crossing = _fields("PendingCrossing")
    leg = _fields("PendingLeg")
    assert {"proc", "report_path", "direction"} <= crossing
    assert {"handle", "future"} <= leg, "PendingLeg no longer looks like a leg"
    assert not ({"handle", "future"} & crossing), (
        "a crossing reaped like a leg would wait for a future that does not exist")
    assert "proc" not in leg and "report_path" not in leg


# --------------------------------------------------------------------------- #
# route connectivity is a preference, and the preference is testable without ROS
# --------------------------------------------------------------------------- #


def _corridor_fixture():
    """A west half, an east half, and one station on each side."""
    import yaml

    from fleet_core import validate_traffic_config
    from fleet_core.legs import Leg, LegRole

    traffic = validate_traffic_config(
        yaml.safe_load((ROOT / "config" / "resources.yaml").read_text(encoding="utf-8")))
    west = Leg(leg_id="to_pick-0", index=0, role=LegRole.TO_PICK, target_name="S_left_a",
               x=-5.0, y=2.5, yaw=0.0)
    east = Leg(leg_id="to_pick-0", index=0, role=LegRole.TO_PICK, target_name="S_right_a",
               x=5.0, y=2.5, yaw=0.0)
    return traffic, west, east


def test_the_split_prefers_the_side_the_robot_is_already_on():
    from fleet_core import split_by_passage

    traffic, west_target, _east_target = _corridor_fixture()
    same, needs = split_by_passage({"r01": -6.0, "r02": 6.0}, west_target, traffic)
    assert same == ["r01"], "the robot already on the target's side needs no passage"
    assert needs == ["r02"], "the robot on the far side would have to cross"


def test_the_split_is_symmetric():
    from fleet_core import split_by_passage

    traffic, _west_target, east_target = _corridor_fixture()
    same, needs = split_by_passage({"r01": -6.0, "r02": 6.0}, east_target, traffic)
    assert same == ["r02"]
    assert needs == ["r01"]


def test_a_robot_with_no_position_is_counted_as_needing_a_passage():
    """Not a claim about the robot: the absence of one.

    `None` means "this node does not know where it is", and pretending it is on the near side
    would be the odometry-origin bug -- which this project has made four times -- wearing a
    different hat.
    """
    from fleet_core import split_by_passage

    traffic, west_target, _ = _corridor_fixture()
    same, needs = split_by_passage({"r01": None, "r02": -6.0}, west_target, traffic)
    assert same == ["r02"]
    assert needs == ["r01"]


def test_the_split_with_no_traffic_config_prefers_nobody():
    """Without a corridor there is no passage to avoid, so nothing may be dropped either."""
    from fleet_core import split_by_passage

    _traffic, west_target, _ = _corridor_fixture()
    same, needs = split_by_passage({"r01": -6.0, "r02": 6.0}, west_target, None)
    assert same == ["r01", "r02"], "no corridor means both are equally reachable"
    assert needs == []


def test_the_dispatcher_no_longer_refuses_on_route_connectivity():
    """The refusal that made a first-leg crossing unreachable.

    It sat UPSTREAM of `_issue_legs`, which is where a crossing is started, so a task whose
    first station is across the barrier could never be assigned and the crossing code never ran.
    Measured on the first N01 attempt: 816 refusals, the task stuck in ACCEPTED.
    """
    body = _source(_function("_eligible_robots"))
    assert "untagged approach leg" not in body, (
        "the route-connectivity refusal is still in the eligibility path")
    assert "split_by_passage(" in body, "the preference is not applied"
    assert "crossing_admitted" in body, (
        "nothing records the case where a passage had to be admitted instead of preferred away")


def test_a_preference_computed_per_robot_cannot_be_a_preference():
    """The split is applied to the surviving SET, not to one robot at a time.

    One robot cannot know whether another could do the job from its own side, so a per-robot rule
    can only refuse. The count of dropped robots is what makes the preference observable.
    """
    body = _source(_function("_eligible_robots"))
    assert "self.crossing_avoided +=" in body
    assert "if same_side:" in body, (
        "without this the robots that need a passage are dropped even when nobody else can reach "
        "the station, which is the old refusal with extra steps")


def test_the_snapshot_reports_what_the_preference_cost():
    snapshot = _source(_function("snapshot"))
    assert "route_connectivity" in snapshot
    assert "admitted_a_passage" in snapshot


def test_the_node_imports_the_rule_it_applies():
    """The guard caught this, not a test: `split_by_passage` was called and never imported.

    The patch's own assertion looked for the NAME anywhere in the file, which a call site
    satisfies. `check_undefined_names.py` looks for a name that no scope binds, and failed the
    build on it -- in the round that added it. Asserting on the import list is the difference
    between "this name appears" and "this name resolves".
    """
    text = NODE.read_text(encoding="utf-8")
    block = text[text.index("from fleet_core import ("):]
    block = block[:block.index(")")]
    assert "split_by_passage" in block, (
        "split_by_passage is used in this node and is not imported from fleet_core")


def test_every_name_in_fleet_core_all_resolves():
    """An `__all__` entry that does not exist is worse than a missing export.

    `from fleet_core import *` raises on it, and nothing else looks. This round produced the same
    mistake three ways -- a name called but not imported (the guard caught it), a name in `__all__`
    but not imported (the tests caught it), a name in a docstring but not in the fields (an
    assertion caught it) -- so it is worth one test of the invariant rather than three of the
    instances.
    """
    import fleet_core

    missing = [name for name in fleet_core.__all__ if not hasattr(fleet_core, name)]
    assert not missing, f"fleet_core.__all__ names that do not resolve: {missing}"


def test_the_split_is_exported_the_way_the_node_uses_it():
    """The node does `from fleet_core import split_by_passage`, which is the only path that
    matters -- being defined in `fleet_core.legs` is not the same thing."""
    import fleet_core

    assert hasattr(fleet_core, "split_by_passage")
    assert "split_by_passage" in fleet_core.__all__


# --------------------------------------------------------------------------- #
# the runner files claims; it does not author them
# --------------------------------------------------------------------------- #


def _crossing_claim(robot: str = "r01", direction: str = "west_to_east",
                    complete: bool = True) -> dict:
    """The shape `fleet_ros.staged_crossing` writes. Copied from a real report."""
    return {
        "robot": robot, "direction": direction, "complete": complete,
        "permit_id": "permit-abc", "renewals": 3, "renew_failures": [],
        "stages": [{"name": "stage_align", "ok": True, "detail": "reached align_west",
                    "elapsed_s": 8.294, "nav_status": "4", "observed_pose": []}],
    }


def test_the_harvested_claim_is_the_driver_shape_the_judge_collects(tmp_path):
    """`judge.load_case` collects `*.json` carrying `robot` AND `direction`.

    That is the whole interface. A runner that wrote a claim in its own words would be a party
    rewording the evidence it is being checked against.
    """
    from fleet_evaluation.judge import load_case

    case = tmp_path / "N01"
    case.mkdir()
    (case / "samples-N01.jsonl").write_text(
        json.dumps({"t": 0.0, "odom": {"r01": [0.0, 0.0, 0.0]}, "truth": {}, "claim": None,
                    "resources": None}) + "\n", encoding="utf-8")
    (case / "driver-crossing-1.json").write_text(
        json.dumps(_crossing_claim()), encoding="utf-8")

    loaded = load_case(case)
    assert len(loaded["drivers"]) == 1, "the claim was not collected as a driver report"
    driver = loaded["drivers"][0]
    assert driver["robot"] == "r01"
    assert driver["direction"] == "west_to_east"
    assert driver["complete"] is True
    assert driver["_file"] == "driver-crossing-1.json"


def test_the_judges_own_output_is_not_read_back_as_a_claim(tmp_path):
    """`judge-*.json` is this judge's verdict, not a driver's claim.

    Filing a claim under that name -- or under `verdict-*.json`, which belongs to the drivers --
    would let the next run cite its own previous output as independent evidence.
    """
    from fleet_evaluation.judge import load_case

    case = tmp_path / "N01"
    case.mkdir()
    (case / "samples-N01.jsonl").write_text(
        json.dumps({"t": 0.0, "odom": {"r01": [0.0, 0.0, 0.0]}, "truth": {}, "claim": None,
                    "resources": None}) + "\n", encoding="utf-8")
    (case / "judge-N01.json").write_text(json.dumps({
        "case": "N01", "samples": 1, "summary": "PASS",
        "checks": [{"check": "x", "verdict": "PASS", "detail": ""}],
    }), encoding="utf-8")

    loaded = load_case(case)
    assert loaded["drivers"] == [], "the judge read its own verdict as a driver claim"


def test_the_harvest_only_takes_files_that_carry_both_keys(tmp_path):
    """A log line is not a claim. `load_case`'s interface is `robot` AND `direction`."""
    import importlib.util

    spec = importlib.util.spec_from_file_location("batch_under_test", ROOT / "scripts" / "batch.py")
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)

    case_dir = tmp_path / "N02"
    (case_dir / "runtime" / "crossings").mkdir(parents=True)
    (case_dir / "runtime" / "crossings" / "good.json").write_text(
        json.dumps(_crossing_claim()), encoding="utf-8")
    (case_dir / "runtime" / "crossings" / "not-a-claim.json").write_text(
        json.dumps({"robot": "r01", "note": "a log line without a direction"}), encoding="utf-8")

    filed = module.harvest_crossing_claims(type("D", (), {"path": case_dir})())
    assert filed == ["driver-crossing-1.json"], (
        f"expected only the report carrying both keys to be filed, got {filed}")
