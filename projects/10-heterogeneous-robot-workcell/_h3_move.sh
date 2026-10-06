set -u
P=/home/ziling/projects/010_heterogeneous_robot_workcell
cd "$P" || exit 1
timeout 1200 ./.venv/bin/python - <<'PY'
import sys, numpy as np, time
sys.path.insert(0,"experiments"); sys.path.insert(0,"src")
import mujoco, build_p3_world as bp3
bp3.install()
import w4_plant as wp
from humanoid007.runtime import Runtime, OPEN

WORLD = "assets/world_w5_h085_loop.xml"
m = mujoco.MjModel.from_xml_path(WORLD)
d = mujoco.MjData(m)
d.qpos[:] = m.key_qpos[0]
mujoco.mj_forward(m, d)

plant = wp.LogisticsPlant(world=WORLD, model=m, data=d)
print("owns_world", plant._owns_world, "writes", plant.qpos_writes)
print("chassis_at_home %.5f" % plant.chassis_at_home)
print("deck_row[0] %.5f  source_band[-1] %.5f  pitch %.5f" % (plant.deck_row[0], plant.source_band[-1], plant.pitch))
print("dock_x_source %.5f  dock_x_receiver %.5f" % (plant.dock_x_source, plant.dock_x_receiver))
print("station_c", plant.stations["station_c"])
print("residual at home station_c", plant.dock_residuals("station_c"))
print()

nb = lambda i: mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, i) or ''

# --- TEST A: does the vehicle reach station_c's approach pose with NO humanoid controller? ---
target = plant.stations["station_c"]["approach_x_m"]
print("=== TEST A: drive_to %.5f, humanoid limp (no controller) ===" % target)
t0 = time.time()
out = plant.drive_to(target, speed=wp.DRIVE_SPEED_MPS, timeout_s=wp.DRIVE_TIMEOUT_S)
print("  %s  (%.1fs wall)" % ({k: (round(v,5) if isinstance(v,float) else v) for k,v in out.items()}, time.time()-t0))
print("  chassis now %.5f  humanoid base %s" % (
    plant.chassis_x(), np.round(d.xpos[m.body('h_LINK_BASE').id],4).tolist()))
print("  contacts touching the chassis:")
for i in range(d.ncon):
    c = d.contact[i]
    n1, n2 = nb(int(m.geom_bodyid[c.geom1])), nb(int(m.geom_bodyid[c.geom2]))
    if 'n_' in n1 or 'n_' in n2:
        if not (n1.startswith('n_') and n2.startswith('n_')) and 'cell_ground' not in (n1,n2):
            print("     %-24s <-> %-24s dist %+0.5f" % (n1, n2, float(c.dist)))
PY
