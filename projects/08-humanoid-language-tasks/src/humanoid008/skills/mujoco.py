"""Real MuJoCo skills (P5).

Ports the unsplit baseline task's phases into the :class:`~humanoid008.skills.base.Skill`
lifecycle, so the P2/P3 TaskManager + RulePlanner can drive the transport one
skill at a time. All eight skills share a single :class:`~humanoid008.simulation.mujoco_world.MujocoWorld`
(the robot is never rebuilt and ``qpos`` is never reset between skills, section 10).

Consolidates docs/MASTER_PLAN.md section 5's ``observe/approach/grasp/lift/carry/
place/verify`` modules into one file because the skills share a private control
loop and the cross-skill manipulation context.
"""

from __future__ import annotations

import math
import os

import numpy as np

from ..contracts import ReasonCode, SkillName, SkillStatus, WorldState
from ..simulation.mujoco_world import MujocoWorld
from ..simulation.robot_runtime import GRASP, OPEN, orientation
from ..simulation.stance import StancePlanner
from .base import PreconditionError, Skill
from .registry import SkillRegistry

CONTROL_STEPS = 25  # robot steps per tick (~0.25 s sim at control_decimation 5)

# Corridor pre-offset applied to the stance the planner already chose as
# reachable. The original code hard-coded -0.08 m here; it is now a parameter
# so it can be swept, but THE DEFAULT MUST STAY -0.08.
#
# WHY: measured 2026-09-13 with the 30-case acceptance batch
# (scripts/batch.py --split, station B):
#     default -0.08  (original behaviour)          -> 26 SUCCEEDED / 4 FAILED
#     default +0.03  (with yaw fix ON)             -> 14 SUCCEEDED / 16 FAILED
# +0.03 looked like the only working value when sweeping the single unperturbed
# case, but it collapses across the perturbed suite. Never retune this without
# running the full batch. See docs/C_STATION_ROOTCAUSE.md.
CORRIDOR_PRE_OFFSET_X = float(os.environ.get("HUMANOID008_CORRIDOR_OFFSET", "-0.08"))

# Stage 0.2 diagnostics: cap on recorded placement-alignment failures per run.
# Bounded so that a long failure loop can neither grow memory nor slow the
# control loop; recording never advances the simulation or touches RNG state.
_PLACE_FAILURE_LIMIT = 64


def _place_failure_detail(check, box, target, final_targets, solution, world, runtime) -> dict:
    """Best-effort numbers for a failed placement alignment check.

    Purely additive diagnostics. The caller decides that the check FAILED using
    the original short-circuit order, so nothing here can change control flow;
    every field is guarded for the same reason.
    """
    detail: dict = {}
    try:
        detail["t"] = float(runtime.d.time)
    except Exception:  # noqa: BLE001
        detail["t"] = None
    try:
        detail["box_offset"] = round(float(np.linalg.norm(box[:2] - target[:2])), 4)
    except Exception:  # noqa: BLE001
        detail["box_offset"] = None
    try:
        detail["arm_error"] = round(float(runtime.arm_error(final_targets, solution)), 4)
    except Exception:  # noqa: BLE001
        detail["arm_error"] = None
    try:
        detail["body_clearance"] = check.get("body_clearance")
    except Exception:  # noqa: BLE001
        detail["body_clearance"] = None
    try:
        detail["perr"] = round(float(np.linalg.norm(runtime.d.qpos[:2] - world.goal[:2])), 4)
    except Exception:  # noqa: BLE001
        detail["perr"] = None
    # Signed error vector: where the base AIMED vs where it ACTUALLY was.
    # A repeatable bias points to over-aiming; scatter points at variance.
    try:
        aim = np.asarray(world.goal[:2], dtype=float)
        got = np.asarray(runtime.d.qpos[:2], dtype=float)
        detail["aim_xy"] = [round(float(v), 4) for v in aim]
        detail["achieved_xy"] = [round(float(v), 4) for v in got]
        detail["err_xy"] = [round(float(v), 4) for v in (got - aim)]
    except Exception:  # noqa: BLE001
        detail["aim_xy"] = None
        detail["achieved_xy"] = None
        detail["err_xy"] = None
    try:
        detail["yaw_deg"] = round(math.degrees(orientation(runtime.d.qpos[3:7])[0]), 3)
    except Exception:  # noqa: BLE001
        detail["yaw_deg"] = None
    # --- stage-2 additive diagnostics ------------------------------------
    # Requested by the staged-action review: record the ACTUAL body pose and
    # foot placement so the stance/clearance question is answered from data
    # rather than inference. Guarded; cannot change control flow.
    try:
        detail["pelvis_xyz"] = [round(float(v), 4) for v in runtime.d.qpos[:3]]
    except Exception:  # noqa: BLE001
        detail["pelvis_xyz"] = None
    try:
        _feet = {}
        for _side, _site in (
            ("left", "force_sensor_left_foot"),
            ("right", "force_sensor_right_foot"),
        ):
            _feet[_side] = [round(float(v), 4) for v in runtime.d.site(_site).xpos]
        detail["feet_xyz"] = _feet
        _mid = (
            np.asarray(_feet["left"], dtype=float)
            + np.asarray(_feet["right"], dtype=float)
        ) / 2.0
        detail["foot_mid_xy"] = [round(float(v), 4) for v in _mid[:2]]
        try:
            detail["foot_to_target"] = round(
                float(np.linalg.norm(_mid[:2] - np.asarray(target[:2], dtype=float))), 4
            )
        except Exception:  # noqa: BLE001
            detail["foot_to_target"] = None
    except Exception:  # noqa: BLE001
        detail["feet_xyz"] = None
        detail["foot_mid_xy"] = None
        detail["foot_to_target"] = None
    try:
        detail["target_xy"] = [round(float(v), 4) for v in target[:2]]
    except Exception:  # noqa: BLE001
        detail["target_xy"] = None
    return detail

# Phases that MUST be holding the box. RELEASE / RETRACT deliberately let go, so
# contact dropping there is expected and must NOT trip CONTACT_LOST (this matches
# the unsplit baseline's phase list exactly).
_CONTACT_PHASES = (
    "LIFT",
    "HOLD",
    "CARRY",
    "STOP",
    "INSPECT_DESTINATION",
    "INSPECT_BOX",
    "LOWER",
)


class _MujocoSkill(Skill):
    """Shared control loop: capture vision, step the shared robot, then update."""

    def __init__(self, world: MujocoWorld) -> None:
        super().__init__()
        self._world = world
        self._phase = "INIT"
        self._since = 0.0
        self._stable_since = None
        self._target_id = "station_b"

    # ------------------------------------------------------------------ #
    # lifecycle
    # ------------------------------------------------------------------ #

    def start(self, world: WorldState, args) -> None:
        self._args = dict(args)
        self._target_id = args.get("target_id") or "station_b"
        self._status = SkillStatus.RUNNING
        self._enter(self._first_phase())

    def tick(self, world: WorldState, dt_s: float) -> None:
        if self.is_finished:
            return
        for _ in range(CONTROL_STEPS):
            self._world.capture()
            self._world.step(
                self._locomotion(),
                self._world.arm,
                self._world.hands,
                self._world.head,
                stationary=False,
            )
        self._world.sense()
        self._update()

    def request_cancel(self, reason: str) -> None:
        if not self.is_finished:
            self._finish(SkillStatus.CANCELLED, ReasonCode.CANCELLED)

    def world_delta(self):
        return {}

    # ------------------------------------------------------------------ #
    # shared helpers
    # ------------------------------------------------------------------ #

    def _enter(self, phase: str) -> None:
        self._phase = phase
        self._since = float(self._world.runtime.d.time)
        self._stable_since = None

    @property
    def _age(self) -> float:
        return float(self._world.runtime.d.time) - self._since

    def _stable(self, condition: bool, duration: float) -> bool:
        now = float(self._world.runtime.d.time)
        if not condition:
            self._stable_since = None
        elif self._stable_since is None:
            self._stable_since = now
        return self._stable_since is not None and now - self._stable_since >= duration

    def _locomotion(self) -> np.ndarray:
        """Port of the baseline's per-step locomotion command."""
        world = self._world
        phase = self._phase
        if phase == "STOP":
            return np.zeros(3)
        brake = phase in ("CARRY", "APPROACH") and (phase != "CARRY" or not world.route)
        alignment = phase == "CARRY" and world.retries > 0
        if alignment:
            brake = False
        control_goal = world.goal if world.hold_anchor is None else world.hold_anchor
        return world.runtime.hold_command(
            control_goal,
            precise=phase in ("CARRY", "APPROACH"),
            brake=brake,
            alignment=alignment,
        )

    def _guard(self) -> bool:
        """Return True if the robot is still healthy.

        Mirrors the unsplit baseline's protective checks (section 10 / 25.5):
        nonfinite state, body fall, stationary base drift, and bilateral contact
        loss while carrying. The latter two were dropped during the P5 skill
        split and are restored here for behavioural equivalence.
        """
        r = self._world.runtime
        w = self._world
        if not np.isfinite(r.d.qpos).all() or not np.isfinite(r.d.qvel).all():
            self._finish(SkillStatus.FAILED, ReasonCode.PRECONDITION_FAILED)
            return False
        if r.d.qpos[2] < 0.65 or orientation(r.d.qpos[3:7])[1] > math.radians(45):
            self._finish(SkillStatus.FAILED, ReasonCode.PRECONDITION_FAILED)
            return False
        # stationary base drift (baseline parity)
        if w.hold_anchor is not None and np.linalg.norm(r.d.qpos[:2] - w.hold_anchor[:2]) > 0.12:
            self._finish(SkillStatus.FAILED, ReasonCode.BASE_DRIFT)
            return False
        # bilateral contact loss while carrying (baseline parity). Only phases
        # that must be holding count; RELEASE/RETRACT let go on purpose.
        if self._phase in _CONTACT_PHASES:
            if min(w.touch["left"], w.touch["right"]) < 0.04:
                if w.contact_loss_since is None:
                    w.contact_loss_since = float(r.d.time)
                if float(r.d.time) - w.contact_loss_since > 0.6:
                    self._finish(SkillStatus.FAILED, ReasonCode.CONTACT_LOST)
                    return False
            else:
                w.contact_loss_since = None
        return True

    def _fresh_box(self) -> bool:
        w = self._world
        return w.box is not None and w.runtime.d.time - w.box.time < _cfg()["max_detection_age"]

    def _dest(self):
        """The visually-localised destination for this skill's target station."""
        return self._world.destination_for(self._target_id)

    def _dest_marker(self):
        """The destination marker centre for this skill's target station."""
        return self._world.destination_marker_for(self._target_id)

    def _finish_ok(self) -> None:
        self._finish(SkillStatus.SUCCEEDED, ReasonCode.OK)

    # subclasses override ------------------------------------------------- #

    def _first_phase(self) -> str:
        raise NotImplementedError

    def _update(self) -> None:
        raise NotImplementedError


class ObserveObject(_MujocoSkill):
    name = SkillName.OBSERVE_OBJECT

    def check_preconditions(self, world, args):
        _require(bool(args.get("object_id")), "no object_id was supplied")

    def _first_phase(self):
        return "STAND"

    def _update(self):
        if not self._guard():
            return
        w = self._world
        r = w.runtime
        speed = np.linalg.norm(r.d.qvel[:2])
        fresh = self._fresh_box()
        if self._phase == "STAND":
            if r.d.time > 3 and self._stable(speed < 0.08, 0.8):
                self._enter("SEARCH")
        elif self._phase == "SEARCH":
            w.head = np.array([0.45 if int(self._age / 2) % 2 else 0.0, 0.25 * math.sin(self._age * 0.6)])
            if fresh:
                self._enter("OBSERVE")
        elif self._phase == "OBSERVE":
            if w.box is not None:
                w.head = r.gaze(w.box.center)
            if fresh and self._age > 1 and self._stable(speed < 0.08, 0.5):
                w.box_center = w.box.center - w.box.normal * _cfg()["marker_height_above_center"]
                w.axis = w.box.long_axis.copy()
                if abs(w.axis[2]) > 0.1:
                    self._finish(SkillStatus.FAILED, ReasonCode.PRECONDITION_FAILED)
                    return
                selected = _choose_stance(w, w.box_center, "grasp")
                if selected is None:
                    self._finish(SkillStatus.FAILED, ReasonCode.PRECONDITION_FAILED)
                    return
                w.goal[:2] = selected["xy"]
                self._finish_ok()


class ObserveTarget(_MujocoSkill):
    name = SkillName.OBSERVE_TARGET

    def check_preconditions(self, world, args):
        _require(bool(args.get("target_id")), "no target_id was supplied")

    def _first_phase(self):
        return "SCAN"

    def _update(self):
        if not self._guard():
            return
        w = self._world
        r = w.runtime
        if self._phase == "SCAN":
            # Orient the head toward the target station's known marker bearing
            # (section 11: fixture position from a known map; exact pose from the
            # visual marker). This reliably brings the marker into view, unlike a
            # blind sweep which the vision throttle can miss.
            w.head = r.gaze(w.marker_bearing(self._target_id))
            if self._dest() is not None:
                self._enter("HOLD")
        elif self._phase == "HOLD":
            if self._dest() is not None and self._stable(True, 0.5):
                self._finish_ok()


class ApproachObject(_MujocoSkill):
    name = SkillName.APPROACH_OBJECT

    def check_preconditions(self, world, args):
        _require(world.entity(args.get("object_id", "")) is not None, "object not in world")

    def _first_phase(self):
        return "APPROACH"

    def _update(self):
        if not self._guard():
            return
        w = self._world
        r = w.runtime
        speed = np.linalg.norm(r.d.qvel[:2])
        if self._phase == "APPROACH":
            if w.box is not None:
                w.head = r.gaze(w.box.center)
            if self._stable(np.linalg.norm(r.d.qpos[:2] - w.goal[:2]) < 0.06 and speed < 0.08, 0.8):
                w.goal[:2] = r.d.qpos[:2]
                self._finish_ok()


class GraspObject(_MujocoSkill):
    name = SkillName.GRASP_OBJECT

    def check_preconditions(self, world, args):
        _require(bool(args.get("object_id")), "no object_id was supplied")

    def start(self, world, args):
        # Grasp targets are computed from the latest localisation (the baseline
        # re-localises right before REACH); the arm resets to the policy default.
        w = self._world
        w.targets = {
            s: w.box_center + sign * w.axis * _cfg()["handle_y"]
            for s, sign in [("left", 1), ("right", -1)]
        }
        w.arm = w.runtime.policy.default[w.runtime.arm_ids].copy()
        super().start(world, args)

    def _first_phase(self):
        return "REACH"

    def _update(self):
        if not self._guard():
            return
        w = self._world
        r = w.runtime
        speed = np.linalg.norm(r.d.qvel[:2])
        fresh = self._fresh_box()
        if self._phase == "REACH":
            w.head = r.gaze(w.box.center) if w.box is not None else w.head
            if not fresh:
                self._finish(SkillStatus.FAILED, ReasonCode.PRECONDITION_FAILED)
                return
            w.arm = r.arm_ik(w.targets, w.arm)
            errors = [
                np.linalg.norm(r.d.site(p + "grasp").xpos - w.targets[s])
                for s, p in [("left", "lh_"), ("right", "rh_")]
            ]
            if self._stable(max(errors) < 0.012 and speed < 0.08, 0.6):
                self._enter("CLOSE")
        elif self._phase == "CLOSE":
            w.arm = r.arm_ik(w.targets, w.arm)
            w.hands = {s: OPEN + min(self._age / 3, 1) * (GRASP - OPEN) for s in w.hands}
            if self._age > 4 and self._stable(min(w.touch["left"], w.touch["right"]) > 0.15, 0.3):
                w.lift_start = {s: p.copy() for s, p in w.targets.items()}
                w.initial_box_z = float(w.box_center[2])
                self._finish_ok()
            elif self._age > 7:
                self._finish(SkillStatus.FAILED, ReasonCode.GRASP_NOT_CONFIRMED)


class LiftObject(_MujocoSkill):
    name = SkillName.LIFT_OBJECT

    def check_preconditions(self, world, args):
        _require(bool(args.get("object_id")), "no object_id was supplied")
        _require(bool(args.get("target_id")), "no target_id was supplied")

    def _first_phase(self):
        return "LIFT"

    def _update(self):
        if not self._guard():
            return
        w = self._world
        r = w.runtime
        fresh = self._fresh_box()
        cfg = _cfg()
        if self._phase == "LIFT":
            w.targets = {
                s: p + np.array([0, 0, cfg["lift_height"] * min(self._age / 4, 1)])
                for s, p in w.lift_start.items()
            }
            w.arm = r.arm_ik(w.targets, w.arm)
            if fresh:
                w.head = r.gaze(w.box.center)
            if self._age > 5:
                if (
                    fresh
                    and w.box.center[2] - cfg["marker_height_above_center"] > w.initial_box_z + 0.08
                    and not w.touch["source"]
                ):
                    self._enter("HOLD")
                else:
                    self._finish(SkillStatus.FAILED, ReasonCode.GRASP_NOT_CONFIRMED)
        elif self._phase == "HOLD":
            w.arm = r.arm_ik(w.targets, w.arm)
            if self._age > 1:
                w.holding = "box_01"
                w.arm = r.d.qpos[r.qa[r.arm_ids]].copy()
                w.carry_offset = (
                    np.mean([r.d.site(p + "grasp").xpos for p in ("lh_", "rh_")], axis=0)
                    - r.d.qpos[:3]
                )
                final_xy = self._dest().center[:2] - w.carry_offset[:2]
                selected = _choose_stance(
                    w,
                    self._dest().center + self._dest().normal * _cfg()["known_box_halfheight"],
                    "place",
                    target_id=self._target_id,
                )
                if selected is None:
                    self._finish(SkillStatus.FAILED, ReasonCode.PRECONDITION_FAILED)
                    return
                final_xy = np.asarray(selected["xy"]) + np.array(
                    [CORRIDOR_PRE_OFFSET_X, 0.0]
                )
                # The corridor must clear the y=0 tables (source + B) with margin.
                # It must NOT overshoot below a negative-y destination: the old
                # `min(y, final_y) - 0.65` gave -1.15 for C (0.65 m PAST the -0.5
                # destination), forcing ~1.8 m of lateral walking that drifted the
                # heading to -8.3 deg and collapsed the placement reach window
                # (which fails past ~+-10 deg). Capping the corridor at the
                # destination y cut that to ~0.8 m. (A variant that walked straight
                # at the destination y was tried and hangs CARRY with
                # BUDGET_EXHAUSTED, so the capped corridor is kept.)
                side_y = min(float(final_xy[1]), -0.65)
                w.route = [
                    np.array([r.d.qpos[0], side_y]),
                    np.array([final_xy[0], side_y]),
                    final_xy,
                ]
                w.goal[:2] = w.route.pop(0)
                self._finish_ok()


class CarryToTarget(_MujocoSkill):
    name = SkillName.CARRY_TO_TARGET

    def check_preconditions(self, world, args):
        _require(bool(args.get("target_id")), "no target_id was supplied")

    def start(self, world, args):
        # Record the carry origin for corridor recovery (baseline parity): the
        # unsplit baseline sets this when CARRY begins; the split path must too,
        # otherwise the recovery branch (carry_origin is not None) never fires.
        self._world.carry_origin = self._world.runtime.d.qpos[:2].copy()
        super().start(world, args)

    def _first_phase(self):
        return "CARRY"

    def _update(self):
        if not self._guard():
            return
        w = self._world
        r = w.runtime
        speed = np.linalg.norm(r.d.qvel[:2])
        if self._phase == "CARRY":
            if self._dest() is not None:
                w.head = r.gaze(self._dest().center)
            tolerance = 0.03 if w.retries > 0 and not w.route else 0.08
            reached = np.linalg.norm(r.d.qpos[:2] - w.goal[:2]) < tolerance
            if not w.route:
                reached = reached and speed < 0.08
            recovery = (
                not reached
                and bool(w.route)
                and w.carry_origin is not None
                and self._age > 8
                and speed < 0.03
                and _corridor_ready(w.carry_origin, w.goal, r.d.qpos)
            )
            if self._stable(reached or recovery, 0.5 if recovery else 0.1):
                # stage-2 additive: log the arrival residual at the instant the
                # skill declares arrival. Tests whether the 0.08 m tolerance is
                # what stops the robot short. Guarded; no control-flow effect.
                try:
                    _arr = getattr(w, "carry_arrivals", None)
                    if _arr is None:
                        _arr = w.carry_arrivals = []
                    if len(_arr) < 24:
                        _arr.append(
                            {
                                "t": round(float(r.d.time), 3),
                                "tolerance": float(tolerance),
                                "arrival_err": round(
                                    float(np.linalg.norm(r.d.qpos[:2] - w.goal[:2])), 4
                                ),
                                "speed": round(float(speed), 4),
                                "retries": int(w.retries),
                                "goal_xy": [round(float(v), 4) for v in w.goal[:2]],
                                "actual_xy": [
                                    round(float(v), 4) for v in r.d.qpos[:2]
                                ],
                                "recovery": bool(recovery),
                            }
                        )
                except Exception:  # noqa: BLE001
                    pass
                if recovery:
                    w.waypoint_recoveries = getattr(w, "waypoint_recoveries", 0) + 1
                    if w.waypoint_recoveries > 2:
                        self._finish(SkillStatus.FAILED, ReasonCode.PRECONDITION_FAILED)
                        return
                if w.route:
                    w.goal[:2] = w.route.pop(0)
                    self._enter("CARRY")
                else:
                    self._enter("STOP")
        elif self._phase == "STOP":
            if self._dest_marker() is not None:
                w.head = r.gaze(self._dest_marker())
            if self._age > 1 and self._stable(speed < 0.08, 0.6):
                w.hold_anchor = np.r_[r.d.qpos[:2], orientation(r.d.qpos[3:7])[0]]
                self._finish_ok()


class PlaceObject(_MujocoSkill):
    name = SkillName.PLACE_OBJECT

    def check_preconditions(self, world, args):
        _require(bool(args.get("object_id")), "no object_id was supplied")
        _require(bool(args.get("target_id")), "no target_id was supplied")

    def _first_phase(self):
        return "INSPECT_DESTINATION"

    def world_delta(self):
        """Stage 0.2 + stage-2: placement-alignment failure diagnostics."""
        out = {"place_failures": list(getattr(self, "_place_failures", []))}
        # stage-2: dump the LAST stance plan's candidate table. This is what
        # distinguishes "no feasible stance exists" from "the search window was
        # too narrow" -- the question the lateral-window experiment could not
        # answer from pass rates alone.
        try:
            plans = [
                p
                for p in getattr(self._world, "stance_plans", [])
                if p.get("candidates")
            ]
            if plans:
                plan = plans[-1]
                out["stance_operation"] = plan.get("operation")
                out["stance_geometry_source"] = plan.get("geometry_source")
                out["stance_candidates"] = [
                    {
                        "xy": [round(float(v), 4) for v in c["xy"]],
                        "feasible": bool(c.get("feasible")),
                        "reason": c.get("reason"),
                        "body_clearance": (
                            round(float(c["body_clearance"]), 5)
                            if c.get("body_clearance") is not None
                            else None
                        ),
                        "error": (
                            round(float(c["error"]), 5)
                            if c.get("error") is not None
                            else None
                        ),
                    }
                    for c in plan["candidates"]
                ]
                out["stance_feasible_count"] = sum(
                    1 for c in plan["candidates"] if c.get("feasible")
                )
                out["stance_candidate_count"] = len(plan["candidates"])
        except Exception:  # noqa: BLE001
            pass
        try:
            out["carry_arrivals"] = list(
                getattr(self._world, "carry_arrivals", [])
            )
        except Exception:  # noqa: BLE001
            pass
        return out

    def _update(self):
        if not self._guard():
            return
        w = self._world
        r = w.runtime
        speed = np.linalg.norm(r.d.qvel[:2])
        fresh = self._fresh_box()
        cfg = _cfg()
        if self._phase == "INSPECT_DESTINATION":
            if self._dest_marker() is not None:
                w.head = r.gaze(self._dest_marker())
            if self._stable(r.d.time - self._dest().time < 0.5 and self._age > 1.0, 0.5):
                w.placement_target = self._dest().center + self._dest().normal * cfg["known_box_halfheight"]
                self._enter("INSPECT_BOX")
        elif self._phase == "INSPECT_BOX":
            center = np.mean([r.d.site(p + "grasp").xpos for p in ("lh_", "rh_")], axis=0)
            w.head = r.gaze(center)
            if fresh and self._age > 1 and self._stable(speed < 0.08, 0.5):
                target = w.placement_target
                box = w.box.center - w.box.normal * cfg["marker_height_above_center"]
                start = {s: r.d.site(p + "grasp").xpos.copy() for s, p in [("left", "lh_"), ("right", "rh_")]}
                delta = target - box + np.array([0, 0, 0.004])
                final_targets = {s: p + delta for s, p in start.items()}
                solution = r.arm_ik(final_targets, w.arm)
                clearance_ok = True
                planner = StancePlanner(r, self._dest().center, station=self._target_id)
                check = planner.evaluate(r.d.qpos[:2], final_targets, w.arm, "place")
                w.stance_plans.append(dict(operation="arrival_check", time=float(r.d.time), **check))
                clearance_ok = check["feasible"]
                # Stage 0.2: identical condition, identical order, identical
                # short-circuit. This only NAMES which of the three checks
                # fired, so the pass/fail decision is unchanged.
                if not clearance_ok:
                    fail_reason = "CLEARANCE"
                elif np.linalg.norm(box[:2] - target[:2]) > 0.18:
                    fail_reason = "BOX_OFFSET"
                elif r.arm_error(final_targets, solution) > 0.01:
                    fail_reason = "ARM_ERROR"
                else:
                    fail_reason = None
                if fail_reason is not None:
                    w.retries += 1
                    failures = getattr(self, "_place_failures", None)
                    if failures is None:
                        failures = self._place_failures = []
                    if len(failures) < _PLACE_FAILURE_LIMIT:
                        record = _place_failure_detail(
                            check, box, target, final_targets, solution, w, r
                        )
                        record["fail_reason"] = fail_reason
                        record["retries"] = int(w.retries)
                        failures.append(record)
                    if w.retries > 2:
                        self._finish(SkillStatus.FAILED, ReasonCode.PLACEMENT_OUT_OF_TOLERANCE)
                        return
                    w.goal[:2] = r.d.qpos[:2] + target[:2] - box[:2]
                    offsets = {s: p - box + np.array([0, 0, 0.004]) for s, p in start.items()}
                    selected = _choose_stance(w, target, "place", offsets, target_id=self._target_id)
                    if selected is None:
                        self._finish(SkillStatus.FAILED, ReasonCode.PRECONDITION_FAILED)
                        return
                    w.goal[:2] = selected["xy"]
                    w.route = []
                    # Release the stale hold anchor: the locomotion layer prefers
                    # hold_anchor over goal, so a leftover anchor would keep the
                    # base pinned at the old position instead of walking to the
                    # new alignment goal (review finding 4).
                    w.hold_anchor = None
                    self._enter("CARRY")
                    return
                w.lower_start = start
                w.lower_delta = delta
                w.manipulation_reference = w.arm.copy()
                self._enter("LOWER")
        elif self._phase == "CARRY":
            # recovery: walk the alignment delta, then re-inspect.
            # NOTE: tightening this radius (tried 0.025) makes CARRY hang with
            # BUDGET_EXHAUSTED -- the policy can *hold* a pose precisely (~0.004 m)
            # but cannot *walk* onto a sub-centimetre target (it overshoots). The
            # wider reach window / better approach remains an open item for the C
            # station (see docs/IMPLEMENTATION_STATUS.md 3.12).
            speed = np.linalg.norm(r.d.qvel[:2])
            if self._stable(np.linalg.norm(r.d.qpos[:2] - w.goal[:2]) < 0.06 and speed < 0.08, 0.8):
                self._enter("INSPECT_DESTINATION")
        elif self._phase == "LOWER":
            w.arm = r.arm_ik(
                {s: p + min(self._age / 4, 1) * w.lower_delta for s, p in w.lower_start.items()},
                w.arm,
                safeguarded=True,
                posture_reference=w.manipulation_reference,
            )
            if fresh:
                w.head = r.gaze(w.box.center)
            if self._age > 5 and w.touch["destination"]:
                w.release_start = {s: h.copy() for s, h in w.hands.items()}
                w.release_arm_start = {
                    s: r.d.site(p + "grasp").xpos.copy() for s, p in [("left", "lh_"), ("right", "rh_")]
                }
                self._enter("RELEASE")
        elif self._phase == "RELEASE":
            w.hands = {s: h + min(self._age / 4, 1) * (OPEN - h) for s, h in w.release_start.items()}
            w.arm = r.arm_ik(
                {
                    s: p + np.array([0, sign * 0.025, 0.015]) * min(self._age / 4, 1)
                    for (s, p), sign in zip(w.release_arm_start.items(), (1, -1))
                },
                w.arm,
                safeguarded=True,
                posture_reference=w.manipulation_reference,
            )
            if self._age > 5:
                w.retract_start = {
                    s: r.d.site(p + "grasp").xpos.copy() for s, p in [("left", "lh_"), ("right", "rh_")]
                }
                w.retract_clear, w.retract_target = _withdrawal(w.release_arm_start)
                self._enter("RETRACT")
        elif self._phase == "RETRACT":
            start, end, fraction = w.retract_start, w.retract_target, min(self._age / 4, 1)
            if self._age < 2:
                start, end, fraction = w.retract_start, w.retract_clear, min(self._age / 2, 1)
            else:
                start, end, fraction = w.retract_clear, w.retract_target, min((self._age - 2) / 2, 1)
            w.arm = r.arm_ik(
                {s: p + (end[s] - p) * fraction for s, p in start.items()},
                w.arm,
                safeguarded=True,
                posture_reference=w.manipulation_reference,
            )
            if self._age > 5:
                w.holding = None
                w.placed_target = self._target_id
                self._finish_ok()


class VerifyResult(_MujocoSkill):
    name = SkillName.VERIFY_RESULT

    def check_preconditions(self, world, args):
        _require(bool(args.get("object_id")), "no object_id was supplied")
        _require(bool(args.get("target_id")), "no target_id was supplied")

    def _first_phase(self):
        return "VERIFY"

    def _update(self):
        if not self._guard():
            return
        w = self._world
        r = w.runtime
        if w.box is not None:
            w.head = r.gaze(w.box.center)
        fresh = self._fresh_box()
        good = (
            fresh
            and w.touch["destination"]
            and w.touch["left"] < 0.02
            and w.touch["right"] < 0.02
            and np.linalg.norm(w.box.center[:2] - w.placement_target[:2]) < 0.06
        )
        if self._stable(good, 1.0):
            self._finish_ok()


def _require(value: bool, message: str) -> None:
    if not value:
        raise PreconditionError(message)


def _cfg() -> dict:
    import json
    from pathlib import Path

    return json.loads((Path(__file__).resolve().parents[3] / "config" / "task.json").read_text())


def _corridor_ready(origin, goal, position) -> bool:
    from ..simulation.backend import corridor_recovery_ready

    return corridor_recovery_ready(origin, goal, position)


def _choose_stance(world, center, operation, offsets=None, target_id="station_b"):
    destination = world.destination_for(target_id)
    planner = StancePlanner(world.runtime, destination.center, station=target_id)
    seed = world.runtime.policy.default[world.runtime.arm_ids] if world.arm is None else world.arm
    plan = planner.select(center, world.axis, seed, operation, hand_offsets=offsets)
    plan["time"] = float(world.runtime.d.time)
    world.stance_plans.append(plan)
    return plan["selected"]


def _withdrawal(anchors):
    from ..simulation.robot_runtime import withdrawal_waypoints

    return withdrawal_waypoints(anchors)


def build_mujoco_registry(world: MujocoWorld) -> SkillRegistry:
    """The closed set of real MuJoCo skills for exactly the declared registry config."""
    from ..simulation import ROOT
    from .registry import load_skill_declarations

    declared = load_skill_declarations(str(ROOT / "config" / "skills" / "registry.yaml"))
    by_name = {
        SkillName.OBSERVE_OBJECT: ObserveObject,
        SkillName.OBSERVE_TARGET: ObserveTarget,
        SkillName.APPROACH_OBJECT: ApproachObject,
        SkillName.GRASP_OBJECT: GraspObject,
        SkillName.LIFT_OBJECT: LiftObject,
        SkillName.CARRY_TO_TARGET: CarryToTarget,
        SkillName.PLACE_OBJECT: PlaceObject,
        SkillName.VERIFY_RESULT: VerifyResult,
    }
    factories = {name: (lambda cls=cls: cls(world)) for name, cls in by_name.items() if name in declared}
    return SkillRegistry(factories, {name: declared[name] for name in factories})
