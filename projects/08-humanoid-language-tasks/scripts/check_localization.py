"""Static localisation check (review finding 4 / 5).

Read-only diagnostic. For each station, run OBSERVE_OBJECT + OBSERVE_TARGET and
compare the *vision-converted table centre* (marker -> table, using the
per-station signed marker offset) against the model's ground-truth table centre.

Ground truth is used ONLY for this independent check and is never fed to the
controller (section 25.5). This verifies, before trusting any full transport,
that B/C are localised to the correct table.
"""

import json

import numpy as np

from humanoid008.contracts import SkillName
from humanoid008.simulation import ROOT
from humanoid008.simulation.mujoco_world import MujocoWorld
from humanoid008.skills.mujoco import build_mujoco_registry

STATIONS = [
    ("station_b", "destination_table", "destination_marker"),
    ("station_c", "destination_c_table", "destination_c_marker"),
]


def observe(world, registry, target_id):
    for name, args in [
        (SkillName.OBSERVE_OBJECT, {"object_id": "box_01"}),
        (SkillName.OBSERVE_TARGET, {"target_id": target_id}),
    ]:
        skill = registry.new_skill(name)
        skill.start(world.view(), args)
        ticks = 0
        while not skill.is_finished and ticks < 4000:
            skill.tick(world.view(), 0.05)
            ticks += 1
        if skill.status.value != "SUCCEEDED":
            return None, name.value
    return world.destination_for(target_id), None


def main():
    print(f"{'station':10s} {'vision_xy':>20s} {'truth_xy':>20s} "
          f"{'pos_err_m':>10s} {'marker_seen':>12s}")
    for target_id, table_geom, marker_geom in STATIONS:
        world = MujocoWorld()
        registry = build_mujoco_registry(world)
        try:
            det, failed_at = observe(world, registry, target_id)
            truth = world.runtime.m.geom(table_geom).pos.copy()
            if det is None:
                print(f"{target_id:10s} {'-':>20s} {np.round(truth[:2],3)!s:>20s} "
                      f"{'FAIL@'+failed_at:>10s} {'no':>12s}")
                continue
            err = float(np.linalg.norm(np.asarray(det.center)[:2] - truth[:2]))
            marker_truth = world.runtime.m.geom(marker_geom).pos[:2]
            seen = np.linalg.norm(np.asarray(det.center)[:2] - marker_truth) > 0.01
            print(f"{target_id:10s} {np.round(det.center[:2],3)!s:>20s} "
                  f"{np.round(truth[:2],3)!s:>20s} {err:10.4f} {str(seen):>12s}")
        finally:
            world.close()


if __name__ == "__main__":
    main()
