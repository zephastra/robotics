"""Offline stance diagnostic: isolate why C-station placement reach window is
narrower than B's (review finding — the "side reach halves" claim must be tested,
not asserted).

Read-only: does not modify src/ or config/. Builds the model once and runs the
SAME place selection (same seed, same axis, same table height, same hand targets
relative to the placement center) for several destination layouts.

Reported per layout:
  - pure IK failures  (BILATERAL_REACH / RELEASE_REACH)
  - collision failures (FIXTURE_CLEARANCE / FINGER_SWEEP_CLEARANCE), with the
    specific robot body part and fixture part involved

Phase 1 controls body orientation (all face +x). Phase 2 sweeps body yaw to test
whether the fixed-yaw stance search is the real culprit (the stance planner only
searches x,y at the robot's *current* orientation — it never rotates the body).
"""

import json

import numpy as np

from humanoid008.simulation import ROOT
from humanoid008.simulation.mujoco_world import MujocoWorld
from humanoid008.simulation.stance import StancePlanner

CFG = json.loads((ROOT / "config" / "task.json").read_text())
HALF = CFG["known_box_halfheight"]  # 0.04

B = np.array([1.23, 0.0, 0.803])
C = np.array([1.6, -0.5, 0.803])
C_Y0 = np.array([1.6, 0.0, 0.803])

B_GEOMS = {"destination_table", "destination_leg",
           "destination_marker_plate", "destination_marker_post"}


def yaw_quat(deg):
    a = np.deg2rad(deg) / 2.0
    return np.array([np.cos(a), 0.0, 0.0, np.sin(a)])


def run_case(world, table_center, station, yaw_deg, drop_b_obstacle=False):
    # Set body orientation (yaw about +z). Base x,y is left at the default
    # standing pose; candidate positions are generated relative to the center.
    world.runtime.d.qpos[3:7] = yaw_quat(yaw_deg)

    center = np.asarray(table_center) + np.array([0.0, 0.0, HALF])
    seed = world.runtime.policy.default[world.runtime.arm_ids]
    axis = np.array([0.0, 1.0, 0.0])

    planner = StancePlanner(world.runtime, np.asarray(table_center), station=station)
    if drop_b_obstacle:
        planner.stations = [
            g for g in planner.stations if planner.r.m.geom(g).name not in B_GEOMS
        ]
    plan = planner.select(center, axis, seed, "place")

    cands = plan["candidates"]
    feas = [c for c in cands if c["feasible"]]
    ik = [c for c in cands if not c["feasible"]
          and c.get("reason") in ("BILATERAL_REACH", "RELEASE_REACH")]
    col = [c for c in cands if not c["feasible"]
           and c.get("reason") in ("FIXTURE_CLEARANCE", "FINGER_SWEEP_CLEARANCE")]

    xs = sorted(c["xy"][0] for c in feas)
    window = round(max(xs) - min(xs), 3) if len(xs) >= 2 else 0.0

    reasons = {}
    for c in cands:
        if not c["feasible"]:
            r = c.get("reason", "?")
            reasons[r] = reasons.get(r, 0) + 1

    pairs = {}
    for c in col:
        closest = c.get("closest") or {}
        for key in ("hand", "body"):
            entry = closest.get(key)
            if entry:
                pairs[(entry[1], entry[2])] = pairs.get((entry[1], entry[2]), 0) + 1

    return {
        "candidates": len(cands),
        "feasible": len(feas),
        "window": window,
        "x_range": [round(min(xs), 2), round(max(xs), 2)] if feas else None,
        "ik": len(ik),
        "coll": len(col),
        "reasons": reasons,
        "coll_pairs": dict(sorted(pairs.items(), key=lambda kv: -kv[1])[:6]),
    }


def print_case(label, r):
    xr = r["x_range"] if r["x_range"] else "-"
    print(f"{label:22s} cand={r['candidates']:2d} feas={r['feasible']:2d} "
          f"win={r['window']:.2f}m x_range={xr} ik={r['ik']} coll={r['coll']} "
          f"reasons={r['reasons']}")
    if r["coll_pairs"]:
        print(f"{'':22s}   collision_pairs(body -> fixture)={r['coll_pairs']}")


def main():
    world = MujocoWorld()
    try:
        print("=== Phase 1: controlled orientation (body faces +x) ===")
        print_case("B (1.23, 0)", run_case(world, B, "station_b", 0.0))
        print_case("C (1.6, -0.5)", run_case(world, C, "station_c", 0.0))
        print_case("C@y0 (1.6, 0)", run_case(world, C_Y0, "station_c", 0.0))
        print_case("C no-B-obstacle", run_case(world, C, "station_c", 0.0, drop_b_obstacle=True))

        print()
        print("=== Phase 2: yaw sweep for C (1.6, -0.5) ===")
        for deg in (0.0, 10.0, 20.0, 30.0, 45.0, -10.0, -20.0, -30.0):
            r = run_case(world, C, "station_c", deg)
            xr = r["x_range"] if r["x_range"] else "-"
            print(f"yaw={deg:+5.0f}deg  feas={r['feasible']:2d} win={r['window']:.2f}m "
                  f"x_range={xr} ik={r['ik']} coll={r['coll']}")
    finally:
        world.close()


if __name__ == "__main__":
    main()
