"""Event-triggered steps: `when <predicate> do <action>`.

WHY THIS EXISTS
===============

`docs/LIMITATIONS.md` names the gap under `batch_driver_extended`:

    the batch runner's sequencing and trigger vocabulary does not cover the case:
    it needs a bespoke multi-phase driver or an event trigger the runner does not
    implement

and `docs/TEST_AND_ACCEPTANCE.md` states the rule it is really about:

    故障按状态触发而非某个固定 sleep ... 超时未到触发条件为 PRECONDITION_NOT_REACHED，
    不算故障测试通过

A fault is triggered by a STATE, not by a delay. A case that says "submit, sleep 20, kill" has
two ways to be wrong and no way to say which one happened: the sleep may fire before the robot
ever reached the state the fault is about, or long after, and either way the run is measuring
the sleep. This project already refuses that shape once, in `fleet_core/wait_for`, for the
arrival-order vocabulary, and the argument is the same one.

So a step is a mapping with two keys:

    when: <predicate>     how the runner knows the moment has arrived. Optional; a step
                          without one is taken as soon as the fleet is ready.
    do:   <action>        what happens at that moment.

The predicate vocabulary is `fleet_core.wait_for`'s, evaluated against the same fleet
snapshot. One vocabulary, one implementation: a second one would drift, and the drift would
look like a case waiting for something the runner never checks.

WHAT IS DECIDABLE, AND WHAT IS NOT
==================================

An action here changes the world; it does not decide whether the case passed. Whether the
fleet behaved correctly is judged afterwards, from the recording and from the ledger. The one
thing a step itself has to report is that it FIRED -- a fault that was declared and never
triggered must not be read as a fault that was survived, which is the same confusion
`FaultInject.srv` refuses at the single-robot level ("a fault that was merely requested and
never took effect turns a pass into a silent no-op").

ACTIONS ARE ENUMERATED, NOT DISCOVERED
======================================

`KNOWN_ACTIONS` is parsed by `scripts/check_batch_manifest.py`, so a manifest cannot name an
action the runner does not implement. That check is parsed rather than copied for the same
reason `KNOWN_KEYS` is: a copied list drifts, and the drift is silent -- the guard would
accept an action that does nothing, and a case that injected nothing would report that the
fleet survived it.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Mapping

# Kept imported rather than dropped: `tests/test_stage2_capabilities.py` asserts that
# these three agree with the literals above, which is the drift check the literal
# spelling needs. `fleet_core.trigger` itself no longer reads them.
from .stage2_capabilities import (  # noqa: F401
    ACTION_FIELDS as _STAGE2_FIELDS,
    ACTION_NEEDS_RUNNER as _STAGE2_RUNNERS,
    NEW_ACTIONS as _STAGE2_ACTIONS,
)

#: The actions `scripts/batch.py` implements.
#: The actions this module owns directly.
_BASE_ACTIONS: frozenset[str] = frozenset({
    "battery_inject",
    "cancel_task",
    "kill_process",
    "restart_process",
})

#: Round 17 (stage 2): the six capabilities the last eleven cases were waiting for add
#: thirteen more actions. `NEW_ACTIONS` in `fleet_core.stage2_capabilities` is the single
#: source of that half, and `tests/test_stage2_capabilities.py` asserts that
#: `KNOWN_ACTIONS == _BASE_ACTIONS | _STAGE2_ACTIONS`.
#:
#: It is spelled out as a LITERAL rather than computed, and that is not redundancy:
#: `scripts/check_batch_manifest.py` parses this name with `ast` and refuses the whole
#: manifest if it cannot read it as a literal set of strings. A computed union is invisible
#: to that parse, and the failure mode is a guard that reports "could not read its own
#: vocabulary" -- or, worse, one that has been "fixed" to fall back to an empty set, which
#: would make every step in the manifest pass. The test is what catches drift now.
KNOWN_ACTIONS: frozenset[str] = frozenset({
    # the four that predate round 17
    "battery_inject",
    "cancel_task",
    "kill_process",
    "restart_process",
    # stage 2: obstacle_injection (F01, F02)
    "spawn_obstacle",
    "despawn_obstacle",
    # stage 2: world_pause_control (I04, I05)
    "pause_world",
    "resume_world",
    "jump_clock",
    # stage 2: truth_collector_control (I06)
    "stop_truth_collector",
    "start_truth_collector",
    # stage 2: db_fault_injection (I07)
    "fail_ledger_writes",
    "heal_ledger_writes",
    # stage 2: permit_renewal_interrupt (F10)
    "suspend_permit_renewal",
    "resume_permit_renewal",
    # stage 2: batch_driver_extended (F03, I03, I08)
    "drive_to_pose",
    "inject_stale_goal_result",
    "terminate_run",
})

#: Actions that act on one robot and cannot mean anything without naming it. `kill_process`
#: and `restart_process` are NOT in here, because `fleet_task_service` runs once for the whole
#: fleet with no namespace and F11 is about restarting exactly that. They require a `runner`
#: only when the target is `all`, where "no namespace" would mean "every node in the fleet".
NEEDS_RUNNER: frozenset[str] = frozenset({
    "battery_inject",
    #: Round 17 (stage 2). The only new action that acts on ONE robot.
    "drive_to_pose",
})

#: Actions that address a process by node identity.
PROCESS_ACTIONS: frozenset[str] = frozenset({
    "kill_process", "restart_process",
})

#: `kill_process` / `restart_process` targets. `all` is every node in that robot's namespace;
#: anything else is matched as `__node:=<name>` alongside `__ns:=/<runner>`. Those two argv
#: tokens are how `ros2 launch` passes a node's identity, measured from `ps` on a live fleet:
#:
#:     python3 .../lib/fleet_ros/nav2_adapter_node --ros-args -r __node:=nav2_adapter -r __ns:=/r01
#:
#: Matching on the argv rather than on the node name is deliberate. A node's NAME is a
#: runtime registration and is not visible to `ps`; its argv is, and the namespace is part of
#: it, so `pkill -f` can address exactly one robot's copy of a node and nothing else.
TARGET_ALL = "all"

#: Extra fields each action requires, beyond `action` itself.
REQUIRED_FIELDS: "dict[str, tuple[str, ...]]" = {
    "battery_inject": ("fraction",),
    "cancel_task": ("request_id",),
    "kill_process": ("node",),
    "restart_process": ("node",),
    #: Round 17 (stage 2). Spelled out for the same reason KNOWN_ACTIONS is: this name is
    #: read with `ast` by `scripts/check_batch_manifest.py`, and a `**unpacking` inside the
    #: literal is invisible to that walk. `test_p17_trigger.py`'s
    #: `test_the_guard_sees_the_actions_the_module_defines` caught exactly that disagreement
    #: when these were merged -- the guard saw 4 fields where the runtime had 18.
    "spawn_obstacle": ("x", "y", "radius_m"),
    "despawn_obstacle": ("name",),
    "pause_world": (),
    "resume_world": (),
    "jump_clock": ("seconds",),
    "stop_truth_collector": (),
    "start_truth_collector": (),
    "fail_ledger_writes": ("mode",),
    "heal_ledger_writes": (),
    "suspend_permit_renewal": (),
    "resume_permit_renewal": (),
    "drive_to_pose": ("x", "y"),
    "inject_stale_goal_result": ("request_id",),
    "terminate_run": ("second_run_dir",),
}


@dataclass(frozen=True)
class Fired:
    """One action, and what the runner did with it."""

    action: str
    detail: str
    ok: bool = True

    def as_note(self) -> str:
        return f"{self.action}: {self.detail}" if self.detail else self.action


def _mapping(value: Any) -> "Mapping[str, Any] | None":
    return value if isinstance(value, Mapping) else None


def validate(step: Any, runners: "list[str] | tuple[str, ...]" = ()) -> "list[str]":
    """What is wrong with one step. An empty list means it is well formed.

    Checked here as well as in the manifest guard, because a case can also be run with a
    manifest edited after the build. The guard's job is to refuse it before a fifteen-minute
    run; this one's job is to refuse it before it faults the wrong robot.
    """
    findings: "list[str]" = []
    step_map = _mapping(step)
    if step_map is None:
        return [f"a step must be a mapping, not {type(step).__name__}"]

    condition = step_map.get("when")
    if condition is not None:
        if not isinstance(condition, Mapping):
            findings.append(f"`when` must be a mapping, not {type(condition).__name__}")
    do = _mapping(step_map.get("do"))
    if do is None:
        findings.append("every step needs a `do` mapping saying what happens")
        return findings

    action = str(do.get("action") or "")
    if not action:
        findings.append("`do.action` is missing")
        return findings
    if action not in KNOWN_ACTIONS:
        findings.append(f"action {action!r} is not implemented; known actions are "
                        f"{sorted(KNOWN_ACTIONS)}")
        return findings

    for field in REQUIRED_FIELDS.get(action, ()):
        if do.get(field) in (None, ""):
            findings.append(f"action {action!r} needs `{field}`")

    runner = do.get("runner")
    if action in NEEDS_RUNNER:
        if not runner:
            findings.append(f"action {action!r} needs a `runner` to act on")
    if action in PROCESS_ACTIONS:
        if str(do.get("node")) == TARGET_ALL and not runner:
            findings.append(f"action {action!r} with node {TARGET_ALL!r} needs a `runner`: "
                            "without a namespace it would address every node in the fleet")
    if runner and runners and str(runner) not in [str(r) for r in runners]:
        findings.append(f"step names runner {runner!r}, which this case does not run "
                        f"({list(runners)}); the fault would target a robot that is not there "
                        "and then report that the fleet survived it")

    return findings


def describe(step: Mapping[str, Any]) -> str:
    """One step, in the shape a person would say it in."""
    do = step.get("do") or {}
    action = str(do.get("action") or "?")
    bits = []
    if do.get("runner"):
        bits.append(f"on {do['runner']}")
    if do.get("node"):
        bits.append(f"node {do['node']}")
    if do.get("request_id"):
        bits.append(f"task {do['request_id']}")
    if do.get("fraction") is not None:
        bits.append(f"fraction {do['fraction']}")
    where = f"when {dict(step['when'])}" if step.get("when") else "as soon as the fleet is ready"
    return f"{action} {' '.join(bits)} -- {where}".replace("  ", " ")
