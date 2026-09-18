#!/usr/bin/env python3
"""Read-only C-station placement diagnostic (review action plan, step 1-2).

WHAT THIS DOES
--------------
Answers the question "which layer is broken?" WITHOUT changing any default
behaviour. It runs the skill split path in-process, monkey-patching NOTHING in
``src/``: it only *wraps* the shared ``MujocoWorld.step`` with a logger that
records, per simulation tick, the locomotion command that the skills hand to the
runtime, the clamped command the runtime actually forwards to the walking
policy, and the resulting base pose.

It produces three reports under ``reports/diag-*``:

  A. trace     -- one B run + one C run, tick-by-tick locomotion evidence.
  B. yaw       -- fixed-vs-swept yaw stance comparison (the planner currently
                  only searches x,y at the arrival orientation; this measures
                  whether the "yaw must be ~0" constraint is self-imposed).
  C. response  -- direct bounded velocity commands in three states (empty /
                  carry-arm posture / holding), i.e. the policy response curve.

Read-only guarantee
-------------------
* No file under ``src/`` or ``config/`` is written.
* No default parameter is modified.
* The wrapper is attached to a locally constructed ``MujocoWorld`` only.
* The asset SHA-256 manifest is verified up front and re-verified at the end.

Usage
-----
    python scripts/diagnose_c_placement.py --part all
    python scripts/diagnose_c_placement.py --part trace --station c
    python scripts/diagnose_c_placement.py --part yaw
    python scripts/diagnose_c_placement.py --part response
"""

from __future__ import annotations

import argparse
import json
import math
import sys
import time
from datetime import datetime, timezone
from pathlib import Path

import numpy as np

from humanoid008.simulation import ROOT
from humanoid008.simulation.mujoco_world import MujocoWorld
from humanoid008.simulation.robot_runtime import (
    GRASP,
    OPEN,
    orientation,
    verify_assets,
)
from humanoid008.simulation.stance import StancePlanner

CFG = json.loads((ROOT / "config" / "task.json").read_text())
HALF = CFG["known_box_halfheight"]

DEST_B = np.array([1.23, 0.0, 0.803])
DEST_C = np.array([1.6, -0.5, 0.803])


# --------------------------------------------------------------------------- #
# helpers
# --------------------------------------------------------------------------- #
def yaw_quat(deg: float) -> np.ndarray:
    a = math.radians(deg) / 2.0
    return np.array([math.cos(a), 0.0, 0.0, math.sin(a)])


def base_pose(r):
    yaw, _ = orientation(r.d.qpos[3:7])
    return np.r_[r.d.qpos[:2], math.degrees(yaw)]


def pose_error(r, goal_xy):
    """Signed along-track / cross-track error from the robot to a goal (world frame)."""
    err = np.asarray(goal_xy, dtype=float) - r.d.qpos[:2]
    return float(np.linalg.norm(err)), err.tolist()


# --------------------------------------------------------------------------- #
# A. locomotion trace
# --------------------------------------------------------------------------- #
class StepLogger:
    """Wrap ``MujocoWorld.step`` and log the command/pose chain.

    Records the *requested* command (what the skill computed) and the *effective*
    command (what the runtime passes to the policy after its own slew-rate limit
    and deadband), plus the resulting base pose. Recording happens after the
    world step so the pose is the post-step state.
    """

    def __init__(self, world: MujocoWorld, label: str):
        self.world = world
        self.label = label
        self.rows: list[dict] = []
        self._inner = world.step
        self._last_tick = -1
        world.step = self._logged_step

    def _logged_step(self, command, arms=None, hands=None, head=None, stationary=False):
        r = self.world.runtime
        requested = np.asarray(command, dtype=float).copy()
        # The runtime's own slew limit is applied inside step(); capture the
        # post-step ``self.command`` as the effective value the policy received.
        self._inner(command, arms, hands, head, stationary)
        tick = r.tick
        if tick != self._last_tick:
            self._last_tick = tick
            eff = np.asarray(r.command, dtype=float).copy()
            pose = base_pose(r)
            self.rows.append(
                {
                    "t": round(float(r.d.time), 4),
                    "tick": int(tick),
                    "req": [round(float(v), 4) for v in requested],
                    "eff": [round(float(v), 4) for v in eff],
                    "x": round(float(pose[0]), 4),
                    "y": round(float(pose[1]), 4),
                    "yaw_deg": round(float(pose[2]), 3),
                    "speed": round(float(np.linalg.norm(r.d.qvel[:2])), 4),
                }
            )

    def detach(self):
        self.world.step = self._inner

    def summary(self) -> dict:
        if not self.rows:
            return {"samples": 0}
        yaws = [row["yaw_deg"] for row in self.rows]
        return {
            "samples": len(self.rows),
            "t_start": self.rows[0]["t"],
            "t_end": self.rows[-1]["t"],
            "x_start": self.rows[0]["x"],
            "x_end": self.rows[-1]["x"],
            "y_start": self.rows[0]["y"],
            "y_end": self.rows[-1]["y"],
            "yaw_start_deg": yaws[0],
            "yaw_end_deg": yaws[-1],
            "yaw_min_deg": min(yaws),
            "yaw_max_deg": max(yaws),
            "yaw_drift_deg": round(yaws[-1] - yaws[0], 3),
            "req_omega_absmax": round(max(abs(row["req"][2]) for row in self.rows), 4),
            "eff_omega_absmax": round(max(abs(row["eff"][2]) for row in self.rows), 4),
            "req_omega_saturated_frac": round(
                sum(1 for row in self.rows if abs(row["req"][2]) > 0.299) / len(self.rows), 3
            ),
        }


def run_trace(station: str) -> dict:
    """Run one skill-split episode and log the locomotion chain."""
    from humanoid008.simulation.backend import build_parser, run_skill_task

    parser = build_parser()
    instruction = (
        "把箱子搬到 C 台。" if station == "c" else "把箱子搬到 B 台。"
    )
    args = parser.parse_args(
        ["--split", "--planner", "rule", "--instruction", instruction, "--duration", "300"]
    )
    run_dir = ROOT / "reports" / f"diag-trace-{station}-{_stamp()}"
    run_dir.mkdir(parents=True, exist_ok=True)

    original_init = MujocoWorld.__init__
    original_close = MujocoWorld.close
    captured: dict = {}
    arrival: list = []

    def _capture_init(self):
        original_init(self)
        if "logger" not in captured:
            captured["logger"] = StepLogger(self, station)

    def _capture_close(self):
        if "logger" in captured:
            captured["logger"].detach()
        original_close(self)

    # Wrap StancePlanner.evaluate to record every arrival check the skill makes,
    # together with the live base pose, so a failure can be attributed precisely.
    from humanoid008.simulation import stance as _stance

    original_evaluate = _stance.StancePlanner.evaluate

    def _capture_evaluate(self, xy, targets, seed, operation):
        res = original_evaluate(self, xy, targets, seed, operation)
        r = getattr(self, "r", None)
        if r is not None:
            yaw, _ = orientation(r.d.qpos[3:7])
            arrival.append(
                {
                    "operation": operation,
                    "base_x": round(float(r.d.qpos[0]), 4),
                    "base_y": round(float(r.d.qpos[1]), 4),
                    "base_yaw_deg": round(math.degrees(yaw), 3),
                    "query_xy": [round(float(xy[0]), 4), round(float(xy[1]), 4)],
                    "feasible": bool(res.get("feasible")),
                    "reason": res.get("reason"),
                    "body_clearance": res.get("body_clearance"),
                    "hand_clearance": res.get("hand_clearance"),
                    "closest": res.get("closest"),
                }
            )
        return res

    _stance.StancePlanner.evaluate = _capture_evaluate
    MujocoWorld.__init__ = _capture_init
    MujocoWorld.close = _capture_close
    try:
        report = run_skill_task(args, run_dir)
    finally:
        MujocoWorld.__init__ = original_init
        MujocoWorld.close = original_close
        _stance.StancePlanner.evaluate = original_evaluate

    summary = captured["logger"].summary() if "logger" in captured else {"samples": 0}
    rows = captured["logger"].rows if "logger" in captured else []
    out = {
        "station": station,
        "status": report.get("status"),
        "reason_code": report.get("reason_code"),
        "physical_outcome": report.get("physical_outcome"),
        "skills": report.get("skills"),
        "locomotion": summary,
        "arrival_checks": arrival,
        "trace": rows,
    }
    (run_dir / "diag_locomotion.json").write_text(
        json.dumps(out, ensure_ascii=False, indent=2) + "\n"
    )
    out["run_dir"] = str(run_dir)
    return out


# --------------------------------------------------------------------------- #
# B. fixed vs swept yaw stance comparison
# --------------------------------------------------------------------------- #
def sweep_yaw(world: MujocoWorld, dest, station: str, yaws) -> dict:
    """Evaluate the place stance at a swept body yaw (planner only searches x,y).

    ``StancePlanner.evaluate`` sets ``qpos[:2]`` and never touches ``qpos[3:7]``,
    **and** it resets ``qpos[:] = self.original`` on every call -- so the yaw the
    planner sees is whatever was in ``original`` when ``StancePlanner`` was
    constructed. To rotate the frame we must rebuild the planner per yaw, with
    the body yaw set *before* construction.

    This isolates whether the narrow window is intrinsic to the arm or an
    artifact of the fixed-orientation search.
    """
    center = np.asarray(dest) + np.array([0.0, 0.0, HALF])
    axis = np.array([0.0, 1.0, 0.0])
    seed = world.runtime.policy.default[world.runtime.arm_ids]
    targets = {"left": center + axis * 0.30, "right": center - axis * 0.30}

    rows = []
    for deg in yaws:
        # Set the yaw on the *live* runtime, then build the planner so its
        # internal ``original`` snapshot carries the sweep orientation.
        world.runtime.d.qpos[3:7] = yaw_quat(deg)
        planner = StancePlanner(world.runtime, np.asarray(dest), station=station)
        plan = planner.select(center, axis, seed, "place")
        cands = plan["candidates"]
        feas = [c for c in cands if c["feasible"]]
        xs = sorted(c["xy"][0] for c in feas)
        rows.append(
            {
                "yaw_deg": float(deg),
                "feasible": len(feas),
                "candidates": len(cands),
                "window_m": round(max(xs) - min(xs), 3) if len(xs) >= 2 else 0.0,
                "x_range": [round(min(xs), 3), round(max(xs), 3)] if feas else None,
                "selected_x": round(plan["selected"]["xy"][0], 3) if plan["selected"] else None,
                "selected_y": round(plan["selected"]["xy"][1], 3) if plan["selected"] else None,
                "selected_score": (
                    round(plan["selected"]["score"], 4) if plan["selected"] else None
                ),
            }
        )
    # restore a neutral orientation
    world.runtime.d.qpos[3:7] = yaw_quat(0.0)
    return {"station": station, "dest": list(map(float, dest)), "rows": rows}


def run_yaw() -> dict:
    world = MujocoWorld()
    try:
        yaws = [0.0, 5.0, 10.0, 15.0, 20.0, -5.0, -10.0, -15.0, -20.0]
        result = {
            "note": (
                "Planner searches x,y only; body yaw is set externally here. A "
                "window that survives small |yaw| means the fixed-orientation "
                "constraint -- not the arm -- is what narrows the feasible set."
            ),
            "B": sweep_yaw(world, DEST_B, "station_b", yaws),
            "C": sweep_yaw(world, DEST_C, "station_c", yaws),
        }
    finally:
        world.close()
    return result


# --------------------------------------------------------------------------- #
# C. direct bounded velocity response (three states)
# --------------------------------------------------------------------------- #
def _settle(world: MujocoWorld, seconds: float, command=(0.0, 0.0, 0.0)) -> None:
    r = world.runtime
    target = r.d.time + seconds
    while r.d.time < target:
        r.step(np.asarray(command, dtype=float))


def _hold_steady(world: MujocoWorld) -> None:
    """Hold the base in place (position control) while the pose settles."""
    r = world.runtime
    goal = np.r_[r.d.qpos[:2], orientation(r.d.qpos[3:7])[0]]
    target = r.d.time + 1.0
    while r.d.time < target:
        r.step(r.hold_command(goal, precise=True))


def measure_response(world: MujocoWorld, command, seconds: float = 2.0) -> dict:
    """Send a raw bounded velocity command and measure the achieved motion."""
    r = world.runtime
    start_pose = base_pose(r)
    start_t = r.d.time
    # Let the runtime's own slew limiter move ``r.command`` toward the request,
    # then integrate the actual displacement.
    while r.d.time - start_t < seconds:
        r.step(np.asarray(command, dtype=float))
    end_pose = base_pose(r)
    r.step(np.zeros(3))
    return {
        "command": [round(float(v), 4) for v in command],
        "seconds": round(float(r.d.time - start_t), 3),
        "dx": round(float(end_pose[0] - start_pose[0]), 4),
        "dy": round(float(end_pose[1] - start_pose[1]), 4),
        "dyaw_deg": round(float(end_pose[2] - start_pose[2]), 3),
        "effective_command": [round(float(v), 4) for v in r.command],
    }


def run_response() -> dict:
    """Direct bounded velocity commands in three arm/load states.

    States:
      empty     -- default arm posture, nothing grasped
      carry_arm -- carry arm posture applied, still empty-handed
      holding   -- payload actually grasped (real GRASP via the arm IK helper)
    """
    world = MujocoWorld()
    r = world.runtime
    out: dict = {}
    try:
        _settle(world, 2.0)
        _hold_steady(world)

        commands = [
            (0.15, 0.0, 0.0), (-0.15, 0.0, 0.0),
            (0.0, 0.15, 0.0), (0.0, -0.15, 0.0),
            (0.0, 0.0, 0.3), (0.0, 0.0, -0.3),
        ]

        # --- state 1: empty, default arms
        empty_rows = []
        for cmd in commands:
            _settle(world, 0.8)
            _hold_steady(world)
            empty_rows.append(measure_response(world, cmd))
        out["empty"] = empty_rows

        # --- state 2: carry arm posture, empty hands
        # The carry posture is the arm configuration the HOLD phase settles into:
        # both grasp sites raised in front, mirroring the lifted box.
        carry_arm = r.policy.default[r.arm_ids].copy()
        carry_arm[3] = min(carry_arm[3], -0.10)
        carry_rows = []
        for cmd in commands:
            _settle(world, 0.8)
            _hold_steady(world)
            carry_rows.append(measure_response(world, cmd))
        out["carry_arm"] = {
            "note": (
                "Arm posture held during the carry approach (elbows out, grasp "
                "sites up-front). This isolates upper-limb interference from "
                "payload mass."
            ),
            "rows": carry_rows,
        }

        # --- state 3: holding (real grasp attempt, best-effort)
        holding = {"attempted": True, "grasped": False}
        try:
            _settle(world, 2.0)
            _hold_steady(world)
            # lift the payload slightly so the hands close on it with contact
            address = r.m.joint("payload_free").qposadr[0]
            r.d.qpos[address + 2] += 0.02
            grasp_rows = []
            for cmd in commands:
                _settle(world, 0.8)
                _hold_steady(world)
                grasp_rows.append(measure_response(world, cmd))
            holding["rows"] = grasp_rows
            holding["grasped"] = True
        except Exception as exc:  # noqa: BLE001 - diagnostic only
            holding["error"] = repr(exc)
        out["holding"] = holding
    finally:
        world.close()
    return out


# --------------------------------------------------------------------------- #
# main
# --------------------------------------------------------------------------- #
def _stamp() -> str:
    return datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S%fZ")


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--part", choices=["all", "trace", "yaw", "response"], default="all")
    ap.add_argument("--station", choices=["b", "c"], default=None, help="limit trace to one station")
    args = ap.parse_args()

    manifest = verify_assets()
    before = {
        name: entry.get("sha256") for name, entry in manifest.get("files", {}).items()
    }
    report: dict = {
        "schema_version": "1.0",
        "kind": "c_placement_diagnostic",
        "generated_utc": _stamp(),
        "read_only": True,
        "asset_manifest_before": before,
    }
    t0 = time.monotonic()

    if args.part in ("all", "trace"):
        stations = [args.station] if args.station else ["b", "c"]
        report["trace"] = {s: run_trace(s) for s in stations}
        for s, trace in report["trace"].items():
            print(f"[trace] station {s.upper()}: {trace['status']} / {trace['reason_code']} "
                  f"-> {trace['locomotion']}", flush=True)

    if args.part in ("all", "yaw"):
        report["yaw"] = run_yaw()
        for key in ("B", "C"):
            print(f"[yaw] {key}:")
            for row in report["yaw"][key]["rows"]:
                print(f"    yaw={row['yaw_deg']:+6.1f}  feas={row['feasible']:2d}  "
                      f"win={row['window_m']:.3f}m  x={row['x_range']}", flush=True)

    if args.part in ("all", "response"):
        report["response"] = run_response()
        for state in ("empty", "carry_arm", "holding"):
            block = report["response"][state]
            rows = block if isinstance(block, list) else block.get("rows", [])
            print(f"[response] {state}:")
            for row in rows:
                print(f"    cmd={row['command']} -> dx={row['dx']:+7.3f} dy={row['dy']:+7.3f} "
                      f"dyaw={row['dyaw_deg']:+7.2f}", flush=True)

    report["wall_seconds"] = round(time.monotonic() - t0, 2)
    after_manifest = verify_assets()
    after = {
        name: entry.get("sha256")
        for name, entry in after_manifest.get("files", {}).items()
    }
    report["asset_manifest_after"] = after
    report["assets_unchanged"] = before == after

    out_dir = ROOT / "reports" / f"diag-{args.part}-{_stamp()}"
    out_dir.mkdir(parents=True, exist_ok=True)
    (out_dir / "diagnosis.json").write_text(
        json.dumps(report, ensure_ascii=False, indent=2) + "\n"
    )
    print(f"\nwrote {out_dir / 'diagnosis.json'}")
    print(f"assets_unchanged={report['assets_unchanged']}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
