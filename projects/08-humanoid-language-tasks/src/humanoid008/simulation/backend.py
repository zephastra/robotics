"""The unsplit baseline task (section 20 P4).

This reproduces the imported 007 baseline's **monolithic** visual-transport
task against 008's own ``simulation`` modules, so 008 can run it without 007.
It is deliberately *not* the skill-based task system (that is P5/P6): the whole
state machine runs as one unit, and 007's ``task.py``/``app.py`` are used only
as a reference (section 6.1).

Scene coordinates and payload truth are absent from the controller; the
``truth_evaluator`` reads terminal state only for the independent verdict.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import time
from datetime import datetime, timezone
from pathlib import Path

import mujoco
import numpy as np

from humanoid008.contracts import ExecutionBackend, Intent
from humanoid008.execution import BudgetLedger, ControlToken, SkillExecutor, TaskManager, load_budget_limits
from humanoid008.execution.planning_worker import PlanningWorker
from humanoid008.language import RuleVocabulary
from humanoid008.planners import (
    BridgeConfig,
    HttpPlannerBridge,
    LlmPlanner,
    ReplayPlanner,
    RulePlanner,
    load_system_prompt,
)
from humanoid008.skills.mujoco import build_mujoco_registry
from humanoid008.supervision import (
    PreconditionSupervisor,
    ProposalValidator,
    RuntimeGuard,
    RuntimePolicy,
    SupervisionPolicy,
)

from . import ROOT
from .camera import HeadCamera
from .mujoco_world import MujocoWorld
from .robot_runtime import GRASP, OPEN, Runtime, orientation, withdrawal_waypoints, verify_assets
from .stance import StancePlanner
from .tactile import sense
from .truth_evaluator import evaluate_transport, transport_accepted
from .vision import detect

CFG = json.loads((ROOT / "config" / "task.json").read_text())

DEFAULT_REPLAY_FILE = ROOT / "tests" / "fixtures" / "model_responses" / "transport.jsonl"
LLM_CONFIG_PATH = ROOT / "config" / "planners" / "llm.yaml"
# Section 13: a real model run pauses at skill boundaries to wait for the model
# (planning_pause); rule/replay run the simulation continuously (realtime).
DEFAULT_CLOCK_MODE = {"rule": "realtime", "replay": "realtime", "llm": "planning_pause"}


def corridor_recovery_ready(origin, goal, position):
    """Intermediate waypoint only: reached its cross-section inside a bounded corridor."""
    segment = np.asarray(goal[:2]) - np.asarray(origin[:2])
    length = np.linalg.norm(segment)
    if length < 0.2:
        return False
    direction = segment / length
    remaining = np.asarray(goal[:2]) - np.asarray(position[:2])
    along = float(remaining @ direction)
    cross = abs(float(remaining[0] * direction[1] - remaining[1] * direction[0]))
    return -0.05 <= along <= 0.025 and cross <= 0.14


class BaselineTask:
    """The unsplit visual-transport state machine (reproduced, not copied)."""

    def __init__(self, r, mode="transport", stance="auto"):
        self.r = r
        self.mode = mode
        self.phase = "STAND"
        self.since = 0.0
        self.stable_since = None
        self.result = None
        self.reason = ""
        self.events = []
        self.arm = None
        self.hands = {s: OPEN.copy() for s in ("left", "right")}
        self.head = np.array([0.1, 0.0])
        self.goal = np.zeros(3)
        self.box = None
        self.destination = None
        self.destination_marker = None
        self.next_frame = 0.0
        self.last_box_time = -1.0
        self.loss_since = None
        self.box_center = None
        self.axis = np.array([0.0, 1.0, 0.0])
        self.targets = None
        self.retries = 0
        self.carry_origin = None
        self.waypoint_recoveries = 0
        self.stance_mode = stance
        self.stance_plans = []
        self.approach_attempts = 0
        self.hold_anchor = None

    def choose_stance(self, center, operation, hand_offsets=None):
        planner = StancePlanner(self.r, self.destination.center)
        seed = self.r.policy.default[self.r.arm_ids] if self.arm is None else self.arm
        plan = planner.select(center, self.axis, seed, operation, hand_offsets=hand_offsets)
        plan["time"] = float(self.r.d.time)
        self.stance_plans.append(plan)
        return plan["selected"]

    def enter(self, phase):
        self.phase = phase
        self.since = float(self.r.d.time)
        self.stable_since = None
        if phase == "CARRY":
            self.carry_origin = self.r.d.qpos[:2].copy()
        if phase in ("CARRY", "APPROACH"):
            self.hold_anchor = None
        self.events.append({"time": self.since, "phase": phase})

    def finish(self, result, reason):
        self.result = result
        self.reason = reason
        self.enter(result)

    def stable(self, condition, duration):
        now = self.r.d.time
        if not condition:
            self.stable_since = None
        elif self.stable_since is None:
            self.stable_since = now
        return self.stable_since is not None and now - self.stable_since >= duration

    def perceive(self, frame):
        box = detect(frame, "box")
        target = detect(frame, "destination")
        if box is not None:
            self.box = box
            self.last_box_time = frame.time
        if target is not None:
            self.destination_marker = target.center.copy()
            target.center = target.center - CFG["destination_marker_offset_y"] * target.long_axis
            self.destination = target

    def update(self, touch):
        if self.result:
            return
        r = self.r
        now = float(r.d.time)
        age = now - self.since
        if not np.isfinite(r.d.qpos).all() or not np.isfinite(r.d.qvel).all():
            self.finish("FAILED", "NONFINITE_STATE")
            return
        if r.d.qpos[2] < 0.65 or orientation(r.d.qpos[3:7])[1] > math.radians(45):
            self.finish("FAILED", "BODY_FALL")
            return
        if age > (45.0 if self.phase == "CARRY" else CFG["phase_timeout"]):
            self.finish("FAILED", "PHASE_TIMEOUT_" + self.phase)
            return
        fresh = self.box is not None and now - self.box.time < CFG["max_detection_age"]
        speed = np.linalg.norm(r.d.qvel[:2])
        if self.hold_anchor is not None and np.linalg.norm(r.d.qpos[:2] - self.hold_anchor[:2]) > 0.12:
            self.finish("FAILED", "STATIONARY_BASE_DRIFT")
            return
        if self.phase in ("LIFT", "HOLD", "CARRY", "STOP", "INSPECT_DESTINATION", "INSPECT_BOX", "LOWER"):
            if min(touch["left"], touch["right"]) < 0.04:
                if self.loss_since is None:
                    self.loss_since = now
                if now - self.loss_since > 0.6:
                    self.finish("FAILED", "BILATERAL_CONTACT_LOST")
                    return
            else:
                self.loss_since = None
        if self.phase == "STAND":
            if now > 3 and self.stable(speed < 0.08, 0.8):
                self.enter("SEARCH")
        elif self.phase == "SEARCH":
            self.head = np.array([0.45 if int(age / 2) % 2 else 0.0, 0.25 * math.sin(age * 0.6)])
            if fresh and self.destination is not None:
                self.enter("OBSERVE")
        elif self.phase == "OBSERVE":
            if self.box is not None:
                self.head = r.gaze(self.box.center)
            if fresh and age > 1 and self.stable(speed < 0.08, 0.5):
                self.box_center = self.box.center - self.box.normal * CFG["marker_height_above_center"]
                self.axis = self.box.long_axis.copy()
                if abs(self.axis[2]) > 0.1:
                    self.finish("FAILED", "BOX_TOO_TILTED")
                    return
                if self.mode == "vision":
                    self.finish("COMPLETED", "VISUAL_LOCALIZATION_ONLY")
                    return
                delta = self.box_center[:2] - r.d.qpos[:2] - np.array([0.23, 0.0])
                if self.stance_mode == "auto":
                    selected = self.choose_stance(self.box_center, "grasp")
                    if selected is None:
                        self.finish("FAILED", "NO_FEASIBLE_GRASP_STANCE")
                        return
                    delta = np.asarray(selected["xy"]) - r.d.qpos[:2]
                if np.linalg.norm(delta) > (1e-6 if self.stance_mode == "auto" else 0.07):
                    self.approach_attempts += 1
                    if self.approach_attempts > 2:
                        self.finish("FAILED", "GRASP_STANCE_ALIGNMENT_LIMIT")
                        return
                    self.goal[:2] = r.d.qpos[:2] + delta
                    self.enter("APPROACH")
                else:
                    self.targets = {
                        s: self.box_center + sign * self.axis * CFG["handle_y"]
                        for s, sign in [("left", 1), ("right", -1)]
                    }
                    self.arm = r.policy.default[r.arm_ids].copy()
                    self.enter("REACH")
        elif self.phase == "APPROACH":
            if self.box is not None:
                self.head = r.gaze(self.box.center)
            if self.stable(np.linalg.norm(r.d.qpos[:2] - self.goal[:2]) < 0.06 and speed < 0.08, 0.8):
                self.goal[:2] = r.d.qpos[:2]
                self.enter("OBSERVE")
        elif self.phase == "REACH":
            self.head = r.gaze(self.box.center)
            if not fresh:
                self.finish("FAILED", "TARGET_LOST_BEFORE_GRASP")
                return
            self.arm = r.arm_ik(self.targets, self.arm)
            errors = [
                np.linalg.norm(r.d.site(p + "grasp").xpos - self.targets[s])
                for s, p in [("left", "lh_"), ("right", "rh_")]
            ]
            if self.stable(max(errors) < 0.012 and speed < 0.08, 0.6):
                self.enter("CLOSE")
        elif self.phase == "CLOSE":
            self.arm = r.arm_ik(self.targets, self.arm)
            self.hands = {s: OPEN + min(age / 3, 1) * (GRASP - OPEN) for s in self.hands}
            if age > 4 and self.stable(min(touch["left"], touch["right"]) > 0.15, 0.3):
                self.lift_start = {s: p.copy() for s, p in self.targets.items()}
                self.initial_box_z = float(self.box_center[2])
                self.enter("LIFT")
            elif age > 7:
                self.finish("FAILED", "NO_BILATERAL_GRASP")
        elif self.phase == "LIFT":
            self.targets = {
                s: p + np.array([0, 0, CFG["lift_height"] * min(age / 4, 1)])
                for s, p in self.lift_start.items()
            }
            self.arm = r.arm_ik(self.targets, self.arm)
            if fresh:
                self.head = r.gaze(self.box.center)
            if age > 5:
                if (
                    fresh
                    and self.box.center[2] - CFG["marker_height_above_center"] > self.initial_box_z + 0.08
                    and not touch["source"]
                ):
                    self.enter("HOLD")
                else:
                    self.finish("FAILED", "LIFT_NOT_VISUALLY_CONFIRMED")
        elif self.phase == "HOLD":
            self.arm = r.arm_ik(self.targets, self.arm)
            if age > 1:
                if self.mode == "lift":
                    self.finish("COMPLETED", "VISUAL_BIMANUAL_LIFT")
                else:
                    self.arm = r.d.qpos[r.qa[r.arm_ids]].copy()
                    self.carry_offset = (
                        np.mean([r.d.site(p + "grasp").xpos for p in ("lh_", "rh_")], axis=0)
                        - r.d.qpos[:3]
                    )
                    final_xy = self.destination.center[:2] - self.carry_offset[:2]
                    if self.stance_mode == "auto":
                        selected = self.choose_stance(
                            self.destination.center + self.destination.normal * CFG["known_box_halfheight"],
                            "place",
                        )
                        if selected is None:
                            self.finish("FAILED", "NO_FEASIBLE_PLACE_STANCE")
                            return
                        final_xy = np.asarray(selected["xy"])
                        final_xy = final_xy - np.array([0.08, 0.0])
                        self.stance_plans[-1]["approach_xy"] = final_xy.tolist()
                    side_y = min(float(r.d.qpos[1]), float(final_xy[1])) - 0.65
                    self.route = [
                        np.array([r.d.qpos[0], side_y]),
                        np.array([final_xy[0], side_y]),
                        final_xy,
                    ]
                    self.goal[:2] = self.route.pop(0)
                    self.enter("CARRY")
        elif self.phase == "CARRY":
            self.head = r.gaze(self.destination.center)
            tolerance = 0.03 if self.stance_mode == "auto" and self.retries > 0 and not self.route else 0.08
            reached = np.linalg.norm(r.d.qpos[:2] - self.goal[:2]) < tolerance
            if self.stance_mode == "auto" and not self.route:
                reached = reached and speed < 0.08
            recovery = (
                not reached
                and bool(self.route)
                and self.carry_origin is not None
                and age > 8
                and speed < 0.03
                and corridor_recovery_ready(self.carry_origin, self.goal, r.d.qpos)
            )
            if self.stable(reached or recovery, 0.5 if recovery else 0.1):
                if recovery:
                    self.waypoint_recoveries += 1
                    if self.waypoint_recoveries > 2:
                        self.finish("FAILED", "WAYPOINT_RECOVERY_LIMIT")
                        return
                    self.events.append(
                        {
                            "time": now,
                            "phase": "CARRY",
                            "recovery": "CORRIDOR_CROSS_SECTION",
                            "attempt": self.waypoint_recoveries,
                        }
                    )
                if self.route:
                    self.goal[:2] = self.route.pop(0)
                    self.enter("CARRY")
                else:
                    if self.stance_mode == "legacy":
                        self.goal[:2] = r.d.qpos[:2]
                    self.enter("STOP")
        elif self.phase == "STOP":
            self.head = r.gaze(self.destination_marker)
            if age > 1 and self.stable(speed < 0.08, 0.6):
                if self.stance_mode == "auto":
                    self.hold_anchor = np.r_[r.d.qpos[:2], orientation(r.d.qpos[3:7])[0]]
                self.enter("INSPECT_DESTINATION")
        elif self.phase == "INSPECT_DESTINATION":
            self.head = r.gaze(self.destination_marker)
            if self.stable(now - self.destination.time < 0.5 and age > 1.0, 0.5):
                self.placement_target = (
                    self.destination.center + self.destination.normal * CFG["known_box_halfheight"]
                )
                self.enter("INSPECT_BOX")
        elif self.phase == "INSPECT_BOX":
            center = np.mean([r.d.site(p + "grasp").xpos for p in ("lh_", "rh_")], axis=0)
            self.head = r.gaze(center)
            if fresh and age > 1 and self.stable(speed < 0.08, 0.5):
                target = self.placement_target
                box = self.box.center - self.box.normal * CFG["marker_height_above_center"]
                start = {s: r.d.site(p + "grasp").xpos.copy() for s, p in [("left", "lh_"), ("right", "rh_")]}
                delta = target - box + np.array([0, 0, 0.004])
                final_targets = {s: p + delta for s, p in start.items()}
                solution = r.arm_ik(final_targets, self.arm)
                clearance_ok = True
                if self.stance_mode == "auto":
                    planner = StancePlanner(r, self.destination.center)
                    check = planner.evaluate(r.d.qpos[:2], final_targets, self.arm, "place")
                    self.stance_plans.append(dict(operation="arrival_check", time=now, **check))
                    clearance_ok = check["feasible"]
                if (
                    not clearance_ok
                    or np.linalg.norm(box[:2] - target[:2]) > 0.18
                    or r.arm_error(final_targets, solution) > 0.01
                ):
                    self.retries += 1
                    if self.retries > 2:
                        self.finish("FAILED", "PLACEMENT_ALIGNMENT_FAILED")
                        return
                    self.goal[:2] = r.d.qpos[:2] + target[:2] - box[:2]
                    if self.stance_mode == "auto":
                        offsets = {s: p - box + np.array([0, 0, 0.004]) for s, p in start.items()}
                        selected = self.choose_stance(target, "place", hand_offsets=offsets)
                        if selected is None:
                            self.finish("FAILED", "NO_FEASIBLE_PLACE_STANCE")
                            return
                        self.goal[:2] = selected["xy"]
                    self.route = []
                    self.enter("CARRY")
                    return
                self.lower_start = start
                self.lower_delta = delta
                self.manipulation_reference = self.arm.copy()
                self.enter("LOWER")
        elif self.phase == "LOWER":
            self.arm = r.arm_ik(
                {s: p + min(age / 4, 1) * self.lower_delta for s, p in self.lower_start.items()},
                self.arm,
                safeguarded=self.stance_mode == "auto",
                posture_reference=self.manipulation_reference,
            )
            if fresh:
                self.head = r.gaze(self.box.center)
            if age > 5 and touch["destination"]:
                self.release_start = {s: h.copy() for s, h in self.hands.items()}
                self.release_arm_start = {
                    s: r.d.site(p + "grasp").xpos.copy() for s, p in [("left", "lh_"), ("right", "rh_")]
                }
                self.enter("RELEASE")
        elif self.phase == "RELEASE":
            self.hands = {s: h + min(age / 4, 1) * (OPEN - h) for s, h in self.release_start.items()}
            if self.stance_mode == "auto":
                self.arm = r.arm_ik(
                    {
                        s: p + np.array([0, sign * 0.025, 0.015]) * min(age / 4, 1)
                        for (s, p), sign in zip(self.release_arm_start.items(), (1, -1))
                    },
                    self.arm,
                    safeguarded=True,
                    posture_reference=self.manipulation_reference,
                )
            if age > 5:
                self.retract_start = {
                    s: r.d.site(p + "grasp").xpos.copy() for s, p in [("left", "lh_"), ("right", "rh_")]
                }
                if self.stance_mode == "auto":
                    self.retract_clear, self.retract_target = withdrawal_waypoints(self.release_arm_start)
                else:
                    self.retract_target = {
                        s: p + np.array([0, sign * 0.08, 0.14])
                        for (s, p), sign in zip(self.retract_start.items(), (1, -1))
                    }
                self.enter("RETRACT")
        elif self.phase == "RETRACT":
            start, end, fraction = self.retract_start, self.retract_target, min(age / 4, 1)
            if self.stance_mode == "auto":
                start, end, fraction = (
                    (self.retract_start, self.retract_clear, min(age / 2, 1))
                    if age < 2
                    else (self.retract_clear, self.retract_target, min((age - 2) / 2, 1))
                )
            self.arm = r.arm_ik(
                {s: p + (end[s] - p) * fraction for s, p in start.items()},
                self.arm,
                safeguarded=self.stance_mode == "auto",
                posture_reference=self.manipulation_reference,
            )
            if age > 5:
                self.enter("VERIFY")
        elif self.phase == "VERIFY":
            if self.box is not None:
                self.head = r.gaze(self.box.center)
            good = (
                fresh
                and touch["destination"]
                and touch["left"] < 0.02
                and touch["right"] < 0.02
                and np.linalg.norm(self.box.center[:2] - self.placement_target[:2]) < 0.06
            )
            if self.stable(good, 1.0):
                self.finish("COMPLETED", "VISUAL_BIMANUAL_TRANSPORT")


def _fingerprint(paths) -> dict:
    result = {}
    for p in paths:
        if p.is_file() and "__pycache__" not in p.parts:
            result[p.relative_to(ROOT).as_posix()] = hashlib.sha256(p.read_bytes()).hexdigest()
    return result


def run_episode(args, run_dir: Path) -> dict:
    """Run one unsplit baseline episode and return the physical report dict."""
    started = time.monotonic()
    report = {
        "schema_version": "1.0",
        "execution_backend": "mujoco",
        "task_kind": "baseline_unsplit",
        "planner_backend": "baseline",
        "status": "ERROR",
        "reason": "INITIALIZATION_FAILED",
        "args": vars(args),
        "records": [],
    }
    report["source_sha256"] = _fingerprint((ROOT / "src").rglob("*.py"))
    report["config_sha256"] = _fingerprint((ROOT / "config").rglob("*"))
    report["asset_manifest"] = verify_assets()
    report["runtime_versions"] = dict(
        python=__import__("sys").version,
        mujoco=mujoco.__version__,
        numpy=np.__version__,
    )

    r = Runtime()
    # Initial scene variation only; never passed to the task or detector.
    address = r.m.joint("payload_free").qposadr[0]
    r.d.qpos[address : address + 2] += args.box_offset
    angle = math.radians(args.box_yaw) / 2
    r.d.qpos[address + 3 : address + 7] = [math.cos(angle), 0, 0, math.sin(angle)]
    for name in (
        "destination_table",
        "destination_leg",
        "destination_marker",
        "destination_marker_plate",
        "destination_marker_post",
    ):
        r.m.geom(name).pos[:2] += args.target_offset
    mujoco.mj_forward(r.m, r.d)

    task = BaselineTask(r, args.mode, args.stance)
    camera = HeadCamera(r, CFG["camera_width"], CFG["camera_height"])
    next_vision = 0.0
    try:
        while not task.result and r.d.time < args.duration:
            for _ in range(10):
                if r.d.time >= next_vision:
                    frame = camera.capture()
                    task.perceive(frame)
                    next_vision = r.d.time + CFG["vision_period"]
                if r.tick % 25 == 0:
                    touch = sense(r)
                    task.update(touch)
                if task.result:
                    break
                brake = args.stance == "auto" and task.phase in ("CARRY", "APPROACH") and (
                    task.phase != "CARRY" or not task.route
                )
                alignment = args.stance == "auto" and task.phase == "CARRY" and task.retries > 0
                if alignment:
                    brake = False
                control_goal = task.goal if task.hold_anchor is None else task.hold_anchor
                command = (
                    np.zeros(3)
                    if args.stance == "auto" and task.phase == "STOP"
                    else r.hold_command(
                        control_goal,
                        precise=task.phase in ("CARRY", "APPROACH"),
                        brake=brake,
                        alignment=alignment,
                    )
                )
                r.step(command, task.arm, task.hands, task.head)
        if not task.result:
            task.finish("TIMEOUT", "DURATION_LIMIT")
        report.update(
            status=task.result,
            reason=task.reason,
            events=task.events,
            final=r.snapshot(),
            stance_plans=task.stance_plans,
        )
        report["evaluation"] = evaluate_transport(r)
        if task.result == "COMPLETED" and args.mode == "transport":
            if not transport_accepted(report["evaluation"]):
                report.update(status="FAILED", reason="INDEPENDENT_EVALUATION_REJECTED")
    finally:
        camera.close()
        report["wall_seconds"] = time.monotonic() - started

    (run_dir / "report.json").write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n")
    return report


def _build_planner(name: str, registry, replay_file: str | None):
    """Build the requested planner for the split (skill-based) backend.

    A real model is only attempted for 'llm'; without a configured endpoint the
    planner raises MissingProviderError at propose time, which the task manager
    records as NOT_RUN_MISSING_PROVIDER (section 12: never silently downgrade).
    """
    if name == "rule":
        return RulePlanner(registry)
    if name == "replay":
        path = Path(replay_file).expanduser() if replay_file else DEFAULT_REPLAY_FILE
        if not path.is_file():
            raise FileNotFoundError(f"replay file not found: {path}")
        return ReplayPlanner.from_jsonl(path)
    if name == "llm":
        config = BridgeConfig.from_env(LLM_CONFIG_PATH)
        bridge = HttpPlannerBridge(config, system_prompt=load_system_prompt(LLM_CONFIG_PATH))
        return LlmPlanner(bridge, registry)
    raise ValueError(f"unknown planner backend {name!r}")


def run_skill_task(args, run_dir: Path) -> dict:
    """Run the SPLIT (skill-based) transport via TaskManager + RulePlanner (P5).

    The eight MuJoCo skills are driven one at a time through the same pipeline
    as the fake backend, sharing one robot. The independent truth evaluator is
    still read-only and reported separately.
    """
    from .truth_evaluator import evaluate_transport, transport_accepted

    started = time.monotonic()
    world = MujocoWorld()
    # Initial scene variation only (same as the unsplit baseline); never passed
    # to the task or detector as ground truth.
    address = world.runtime.m.joint("payload_free").qposadr[0]
    world.runtime.d.qpos[address : address + 2] += args.box_offset
    angle = math.radians(args.box_yaw) / 2
    world.runtime.d.qpos[address + 3 : address + 7] = [math.cos(angle), 0, 0, math.sin(angle)]
    for name in (
        "destination_table",
        "destination_leg",
        "destination_marker",
        "destination_marker_plate",
        "destination_marker_post",
        "destination_c_table",
        "destination_c_leg",
        "destination_c_marker",
        "destination_c_marker_plate",
        "destination_c_marker_post",
    ):
        world.runtime.m.geom(name).pos[:2] += args.target_offset
    mujoco.mj_forward(world.runtime.m, world.runtime.d)
    registry = build_mujoco_registry(world)
    planner = _build_planner(args.planner, registry, args.replay_file)
    clock_mode = args.clock_mode or DEFAULT_CLOCK_MODE[args.planner]
    # Section 13: "技能执行时限：保留导入基线". The imported baseline runs with
    # --duration 120 (120 s sim). The P2 default of 500 ticks (25 s) is too tight
    # for the longest skill (PLACE, which can recover its stance a few times), so
    # we align the per-skill budget to the baseline horizon (120 s / 0.05 s = 2400).
    executor = SkillExecutor(registry, max_ticks_per_skill=2400)
    vocabulary = RuleVocabulary.load(str(ROOT / "config" / "planners" / "rule.yaml"))
    validator = ProposalValidator(registry)
    precondition = PreconditionSupervisor(registry, SupervisionPolicy())
    control = ControlToken()
    guard = RuntimeGuard(RuntimePolicy(), control)
    budgets = BudgetLedger(load_budget_limits(str(ROOT / "config" / "supervision" / "default.yaml")))

    manager = TaskManager(
        runtime=world,
        registry=registry,
        planner=planner,
        executor=executor,
        vocabulary=vocabulary,
        execution_backend=ExecutionBackend.MUJOCO,
        validator=validator,
        precondition=precondition,
        guard=guard,
        budgets=budgets,
        control=control,
        planning_worker=PlanningWorker(planner),
    )

    outcome = manager.run_instruction(
        args.instruction or "把箱子搬到 B 台。", task_id=f"task-{run_dir.name}"
    )

    report = {
        "schema_version": "1.0",
        "execution_backend": "mujoco",
        "task_kind": "skill_split",
        "planner_backend": args.planner,
        "clock_mode": clock_mode,
        "status": outcome.task_status.value,
        "reason_code": outcome.reason_code.value,
        "skills": [r.skill.value for r in outcome.history],
        "skill_results": [r.to_dict() for r in outcome.history],
        "args": vars(args),
    }

    physical_outcome = "NOT_APPLICABLE"
    evaluation = None
    if outcome.task_status.value == "SUCCEEDED":
        intent = outcome.goal.intent if outcome.goal is not None else None
        if intent is Intent.TRANSPORT:
            # Only a transport task is judged by transport acceptance (section
            # 25.4). An inspect task that succeeded must NOT be re-judged as a
            # failed transport (review finding 5).
            evaluation = evaluate_transport(world.runtime, target_id=outcome.goal.target_id or "station_b")
            physical_outcome = "PASS" if transport_accepted(evaluation) else "FAIL"
            if not transport_accepted(evaluation):
                report["status"] = "FAILED"
                report["reason_code"] = "INDEPENDENT_EVALUATION_REJECTED"
        # else: inspect (and any other non-transport intent) keeps
        # physical_outcome = NOT_APPLICABLE -- no transport acceptance applies.
    world.close()
    report["physical_outcome"] = physical_outcome
    report["evaluation"] = evaluation
    report["wall_seconds"] = time.monotonic() - started

    (run_dir / "report.json").write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n")
    return report


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="008 MuJoCo transport (unsplit baseline or skill split)")
    parser.add_argument("--mode", choices=["vision", "lift", "transport"], default="transport")
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--duration", type=float, default=180.0)
    parser.add_argument("--box-offset", type=float, nargs=2, default=[0.0, 0.0])
    parser.add_argument("--target-offset", type=float, nargs=2, default=[0.0, 0.0])
    parser.add_argument("--box-yaw", type=float, default=0.0)
    parser.add_argument("--stance", choices=["auto", "legacy"], default="auto")
    parser.add_argument("--split", action="store_true", help="run the P5 skill-based task via TaskManager + RulePlanner")
    parser.add_argument("--instruction", default=None, help="language instruction for --split")
    parser.add_argument(
        "--planner",
        choices=["rule", "replay", "llm"],
        default="rule",
        help="planner backend for --split (default: rule; llm needs a configured endpoint)",
    )
    parser.add_argument(
        "--replay-file",
        default=None,
        help=f"JSONL recording for --planner replay (default: {DEFAULT_REPLAY_FILE.name})",
    )
    parser.add_argument(
        "--clock-mode",
        choices=["realtime", "planning_pause"],
        default=None,
        help="override the planner's default clock mode",
    )
    return parser


def main() -> int:
    parser = build_parser()
    args = parser.parse_args()
    if (
        not np.isfinite([args.duration, args.box_yaw, *args.box_offset, *args.target_offset]).all()
        or args.duration <= 0
        or abs(args.box_yaw) > 10
        or max(map(abs, args.box_offset)) > 0.04
        or max(map(abs, args.target_offset)) > 0.15
    ):
        parser.error("Invalid or out-of-scope initial conditions")
    run_dir = ROOT / "reports" / datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S%fZ")
    run_dir.mkdir(parents=True, exist_ok=False)
    if args.split:
        report = run_skill_task(args, run_dir)
        print(report["status"], report["physical_outcome"], flush=True)
        return 0 if report["status"] == "SUCCEEDED" else 1
    report = run_episode(args, run_dir)
    print(report["status"], report["reason"], flush=True)
    return 0 if report["status"] == "COMPLETED" else 1


if __name__ == "__main__":
    raise SystemExit(main())
