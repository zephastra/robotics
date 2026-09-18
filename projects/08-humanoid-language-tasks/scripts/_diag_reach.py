"""Reach-window diagnostic (review finding 4, first half).

Sweeps the placement reach window along the approach axis (table_centre - d)
with a fixed orientation, and reports -- per distance -- whether the two-arm IK
is feasible and why not. This shows whether the ~0.08 m window is bounded by the
arm's maximum reach, by a minimum stand-off, or by something else, i.e. whether
"widen the window" is even possible before trying seeds/postures.
"""

import json

import numpy as np

from humanoid008.simulation import ROOT
from humanoid008.simulation.mujoco_world import MujocoWorld
from humanoid008.simulation.stance import StancePlanner

CFG = json.loads((ROOT / "config" / "task.json").read_text())
HALF = CFG["known_box_halfheight"]

DEST_B = np.array([1.23, 0.0, 0.803])
DEST_C = np.array([1.6, -0.5, 0.803])


def sweep(world, dest, station, seed, label):
    center = dest + np.array([0.0, 0.0, HALF])
    axis = np.array([0.0, 1.0, 0.0])
    targets = {"left": center + axis * 0.30, "right": center - axis * 0.30}
    planner = StancePlanner(world.runtime, dest, station=station)
    feas = []
    print(f"  {label}:")
    for d in np.arange(0.10, 0.50, 0.02):
        xy = np.array([center[0] - d, center[1]])
        res = planner.evaluate(xy, targets, seed, "place")
        ok = bool(res["feasible"])
        if ok:
            feas.append(round(float(d), 2))
        print(f"    d={d:.2f}  feasible={ok!s:5s}  reason={res.get('reason','')!s:20s} "
              f"err={res.get('error') if res.get('error') is None else round(res['error'],4)}")
    width = 0.0 if not feas else round(max(feas) - min(feas) + 0.02, 3)
    print(f"    -> feasible d in {feas}  width={width} m")


def main():
    world = MujocoWorld()
    try:
        seed = world.runtime.policy.default[world.runtime.arm_ids]
        print("=== B (1.23, 0), yaw=0, default seed ===")
        sweep(world, DEST_B, "station_b", seed, "B default")
        print("=== C (1.6, -0.5), yaw=0, default seed ===")
        sweep(world, DEST_C, "station_c", seed, "C default")
    finally:
        world.close()


if __name__ == "__main__":
    main()
