"""Bounded, forward-facing bilateral stance search in a known fixture workspace.

The copied collision model uses the explicit source map and visual destination.
Payload simulator pose is never queried. Sampled geometric checks are not a
continuous collision proof or a dynamic balance guarantee.

Adapted from the 007 baseline (section 6.1).
"""

import copy
import json

import mujoco
import numpy as np

from . import ROOT
from .robot_runtime import GRASP, OPEN, forward_kinematics, withdrawal_waypoints


def candidate_positions(center, current, operation="grasp"):
    result = []
    if 0.20 <= center[0] - current[0] <= 0.40 and abs(center[1] - current[1]) <= 0.08:
        result.append(np.asarray(current[:2]).copy())
    # NOTE (C-station investigation): the real placement window along the approach
    # axis is ~[0.22, 0.34] m (clearance-limited below by the hip, reach-limited
    # above), i.e. ~0.14 m wide -- wider than the 0.08 m covered by this 0.04 m
    # stride. Densifying the stride to 0.02 m was tried and did NOT fix the C
    # placement (it turned a clean PLACEMENT_OUT_OF_TOLERANCE into a CARRY
    # BUDGET_EXHAUSTED hang), so the original stride is kept. See
    # docs/IMPLEMENTATION_STATUS.md 3.12/3.13.
    distances = (0.24, 0.28, 0.32, 0.36, 0.40, 0.44) if operation == "place" else (0.24, 0.28, 0.32, 0.36)
    result.extend(
        np.array([center[0] - distance, center[1] + side])
        for distance in distances
        for side in (0.0, -0.04, 0.04)
    )
    return result


class StancePlanner:
    def __init__(self, r, destination, station="station_b"):
        self.r = copy.copy(r)
        self.r._ik_scratch = self.r._error_scratch = self.r._gaze_scratch = None
        self.r.m = copy.copy(r.m)
        self.r.d = mujoco.MjData(self.r.m)
        self.r.d.qpos[:] = r.d.qpos
        known = json.loads((ROOT / "config" / "workcell.json").read_text())
        source = np.asarray(known["source_table_center"])
        target = np.asarray(destination) - np.array([0.0, 0.0, 0.007])
        # The source fixture always sits at its known map position; the current
        # destination fixture is moved to its visually-localised position. The
        # *other* destination stays at its fixed model pose (it is an obstacle).
        self.r.m.geom("source_table").pos[:] = source
        self.r.m.geom("source_leg").pos[:2] = source[:2] + [0.08, 0.0]
        if station == "station_c":
            table, leg = "destination_c_table", "destination_c_leg"
            plate, post = "destination_c_marker_plate", "destination_c_marker_post"
        else:
            table, leg = "destination_table", "destination_leg"
            plate, post = "destination_marker_plate", "destination_marker_post"
        # Per-station SIGNED marker installation offset (review finding 4): the
        # marker plate/post stand at table_centre + offset * long_axis, and the
        # sign differs per station (B: +y, C: -y). This must be the SAME value
        # the vision conversion and the scene geometry use.
        offset = known.get("marker_offset_y", {}).get(station, 0.75)
        self.r.m.geom(table).pos[:] = target
        self.r.m.geom(leg).pos[:2] = target[:2] + [0.08, 0.0]
        self.r.m.geom(plate).pos[:2] = target[:2] + [0.0, offset]
        self.r.m.geom(post).pos[:2] = target[:2] + [0.0, offset]
        self.stations = [
            self.r.m.geom(n).id
            for n in (
                "source_table",
                "source_leg",
                "destination_table",
                "destination_leg",
                "destination_marker_plate",
                "destination_marker_post",
                "destination_c_table",
                "destination_c_leg",
                "destination_c_marker_plate",
                "destination_c_marker_post",
            )
        ]
        self.robot = [
            g
            for g in range(r.m.ngeom)
            if r.m.geom_bodyid[g] != 0
            and r.m.body(int(r.m.geom_bodyid[g])).name != "payload"
            and (r.m.geom_contype[g] or r.m.geom_conaffinity[g])
        ]
        self.original = r.d.qpos.copy()

    def separation_bound(self, a, b):
        r = self.r
        boxes = []
        for g in (a, b):
            rotation = r.d.geom_xmat[g].reshape(3, 3)
            local = r.m.geom_aabb[g]
            boxes.append((r.d.geom_xpos[g] + rotation @ local[:3], np.abs(rotation) @ local[3:]))
        gap = np.maximum(np.abs(boxes[0][0] - boxes[1][0]) - boxes[0][1] - boxes[1][1], 0.0)
        return float(np.linalg.norm(gap))

    def clearance(self):
        r = self.r
        forward_kinematics(r.m, r.d)
        body = hand = 0.2
        self.closest = {}
        for g in self.robot:
            is_hand = r.m.body(int(r.m.geom_bodyid[g])).name.startswith(("lh_", "rh_"))
            for station in self.stations:
                bound = self.separation_bound(g, station)
                distance = (
                    bound
                    if bound > 0.03
                    else max(bound, mujoco.mj_geomDistance(r.m, r.d, g, station, 0.2, None))
                )
                key = "hand" if is_hand else "body"
                if distance < (hand if is_hand else body):
                    self.closest[key] = [
                        g,
                        r.m.body(int(r.m.geom_bodyid[g])).name,
                        r.m.geom(station).name,
                        float(distance),
                    ]
                if is_hand:
                    hand = min(hand, float(distance))
                else:
                    body = min(body, float(distance))
        return body, hand

    def evaluate(self, xy, targets, seed, operation):
        r = self.r
        r.d.qpos[:] = self.original
        r.d.qpos[:2] = xy
        joints = np.asarray(seed).copy()
        minimum = 0.2
        error = 0.0
        solve_options = (
            dict(safeguarded=True, posture_reference=np.asarray(seed).copy())
            if operation == "place"
            else {}
        )
        stages = (
            [targets, {s: p + [0, 0, 0.12] for s, p in targets.items()}]
            if operation == "grasp"
            else [targets, *withdrawal_waypoints(targets)]
        )
        for stage_index, stage in enumerate(stages):
            solution = r.arm_ik(stage, joints, **solve_options)
            error = max(error, r.arm_error(stage, solution))
            if error > 0.012:
                return dict(feasible=False, reason="BILATERAL_REACH", error=error, stage_index=stage_index)
            for fraction in np.linspace(0, 1, 6):
                r.d.qpos[r.qa[r.arm_ids]] = joints + fraction * (solution - joints)
                body, hand = self.clearance()
                minimum = min(minimum, body)
                if body < 0.025 or hand < 0.003:
                    return dict(
                        feasible=False,
                        reason="FIXTURE_CLEARANCE",
                        body_clearance=body,
                        hand_clearance=hand,
                        closest=self.closest,
                        fraction=float(fraction),
                    )
            joints = solution
            if stage_index == 0:
                desired = GRASP if operation == "grasp" else OPEN
                addresses = {
                    s: r.m.jnt_qposadr[r.m.actuator_trnid[act, 0]]
                    for s, act in r.hand_act.items()
                }
                starts = {s: r.d.qpos[q].copy() for s, q in addresses.items()}
                for fraction in np.linspace(0, 1, 6):
                    if operation == "place":
                        release_targets = {
                            s: p + np.array([0, sign * 0.025, 0.015]) * fraction
                            for (s, p), sign in zip(stage.items(), (1, -1))
                        }
                        joints = r.arm_ik(release_targets, joints, **solve_options)
                        error = max(error, r.arm_error(release_targets, joints))
                        if error > 0.012:
                            return dict(feasible=False, reason="RELEASE_REACH", error=error)
                        r.d.qpos[r.qa[r.arm_ids]] = joints
                    for side, q in addresses.items():
                        r.d.qpos[q] = starts[side] + fraction * (desired - starts[side])
                    body, hand = self.clearance()
                    minimum = min(minimum, body)
                    if body < 0.025 or hand < 0.003:
                        return dict(
                            feasible=False,
                            reason="FINGER_SWEEP_CLEARANCE",
                            body_clearance=body,
                            hand_clearance=hand,
                        )
        return dict(feasible=True, error=error, body_clearance=minimum)

    def select(self, center, axis, seed, operation, hand_offsets=None):
        targets = {
            s: np.asarray(center)
            + (
                np.asarray(hand_offsets[s])
                if hand_offsets is not None
                else sign * np.asarray(axis) * 0.30
            )
            for s, sign in [("left", 1), ("right", -1)]
        }
        candidates = []
        for xy in candidate_positions(center, self.original[:2], operation):
            result = self.evaluate(xy, targets, seed, operation)
            result["xy"] = xy.tolist()
            if result["feasible"]:
                weight = 0.02 if operation == "place" else 0.002
                reach_cost = 20.0 * result["error"] if operation == "place" else 0.0
                result["score"] = float(
                    np.linalg.norm(xy - self.original[:2])
                    + weight / (result["body_clearance"] + 0.01)
                    + reach_cost
                )
            candidates.append(result)
        feasible = [c for c in candidates if c["feasible"]]
        # Robustness (review finding 4): prefer a stance whose *neighbourhood* is
        # also feasible -- the interior of the reach window -- rather than an edge
        # that merely happens to be closest. The robot's arrival carries ~0.05 m
        # position error and a few degrees of heading drift, so an edge stance
        # lands outside the feasible set while an interior stance absorbs the
        # error. This reuses the already-evaluated candidate grid (no extra IK).
        for c in feasible:
            xy = np.asarray(c["xy"])
            c["robustness"] = sum(
                1
                for o in feasible
                if o is not c and np.linalg.norm(np.asarray(o["xy"]) - xy) <= 0.05
            )
            c["score"] += 0.20 * max(0, 4 - c["robustness"])
        chosen = min(feasible, key=lambda c: c["score"]) if feasible else None
        return dict(
            operation=operation,
            candidates=candidates,
            selected=chosen,
            geometry_source="known source map + visually located destination fixture",
            scope="measured orientation, sampled arm paths; no global path or balance proof",
        )
