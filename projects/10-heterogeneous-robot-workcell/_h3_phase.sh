set -u
P=/home/ziling/projects/010_heterogeneous_robot_workcell
cd "$P" || exit 1
timeout 2400 ./.venv/bin/python - <<'PY'
import sys, numpy as np, time
sys.path.insert(0,"experiments"); sys.path.insert(0,"src")
import mujoco, build_p3_world as bp3
bp3.install()
import w4_plant as wp
import probe_h2_w5 as H2
from humanoid007.runtime import Runtime, OPEN, GRASP, ExitPath

WORLD = "assets/world_w5_h085_loop.xml"
m = mujoco.MjModel.from_xml_path(WORLD)
d = mujoco.MjData(m)
d.qpos[:] = m.key_qpos[0]
mujoco.mj_forward(m, d)
TRAY = "c_payload"
r = Runtime(world=WORLD, prefix="h_", object_body=TRAY, model=m, data=d)
plant = wp.LogisticsPlant(world=WORLD, model=m, data=d)
nb = lambda i: mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, i) or ''

tray0 = np.array(d.body(TRAY).xpos, float)
default_arms = r.policy.default[r.arm_ids].copy()
exit_path = ExitPath(r, H2.anchor_at(H2.PLACE_X, 0.0, H2.HANDLE_Y), default_arms,
                     ExitPath.DEFAULT_VARIANT)

print("=== running the humanoid phase exactly as H3 does ===")
t0=time.time()
while d.time < H2.SEQUENCE_SECONDS:
    now = d.time
    arms, hands = H2.stage_driver(r, float(tray0[0]), now, default_arms, OPEN, GRASP,
                                  exit_path, H2.HANDLE_Y, H2.HAND_ROLL_BOOT if False else H2.HAND_ROLL_BIAS,
                                  dict(H2.PHASE_WINDOWS))
    r.step(np.zeros(3), arms, hands, stationary=now > 2)
print("  humanoid phase done t=%.3f (%.0fs wall)" % (d.time, time.time()-t0))
print("  humanoid base   ", np.round(d.xpos[m.body('h_LINK_BASE').id],4).tolist())
print("  tray            ", np.round(d.body(TRAY).xpos,4).tolist())
print("  chassis x %.5f" % float(d.xpos[m.body('n_base_link').id][0]))
print("  ctrl nonzero count", int(np.count_nonzero(d.ctrl)))

# --- what does the plant's hold law put on the humanoid's arms? ---
import merge_world as mw
ctrl_hold = np.asarray(mw.home_hold_ctrl(m, plant._home, d.qpos, d.qvel), float).reshape(-1)
arm_act = np.array([int(i) for i in r.body_act], int)
print("  plant _hold() ctrl on the humanoid's 25 policy actuators: nonzero %d / %d, max %.3f"
      % (int(np.count_nonzero(ctrl_hold[arm_act])), len(arm_act), float(np.max(np.abs(ctrl_hold[arm_act])))))
print("  runtime's own last ctrl on those:                          max %.3f"
      % float(np.max(np.abs(d.ctrl[arm_act]))))

# --- now the chain phase, exactly as H3 does: plant._tick only ---
print()
print("=== driving to station_c's approach, plant hold law only ===")
for probe in (5, 20, 60, 120):
    pass
target = plant.stations["station_c"]["approach_x_m"]
out = plant.drive_to(target, speed=wp.DRIVE_SPEED_MPS, timeout_s=wp.DRIVE_TIMEOUT_S)
print("  drive_to %.5f ->" % target, {k:(round(v,5) if isinstance(v,float) else v) for k,v in out.items()})
print("  chassis now %.5f" % plant.chassis_x())
print("  humanoid base ", np.round(d.xpos[m.body('h_LINK_BASE').id],4).tolist())
print("  humanoid base z drift from 1.03: %+.5f" % (float(d.xpos[m.body('h_LINK_BASE').id][2]) - 1.03))
print("  contacts not involving n_/world:")
for i in range(d.ncon):
    c = d.contact[i]
    n1, n2 = nb(int(m.geom_bodyid[c.geom1])), nb(int(m.geom_bodyid[c.geom2]))
    if (n1.startswith('h_') != n2.startswith('h_')) or ('h_' in (n1,n2) and 'cell_ground' not in (n1,n2)):
        print("     %-24s <-> %-24s dist %+0.5f" % (n1, n2, float(c.dist)))
print()
print("=== is the tray still supported? ===")
print("  tray", np.round(d.body(TRAY).xpos,4).tolist())
sup=[]
for i in range(d.ncon):
    c=d.contact[i]
    b1,b2 = int(m.geom_bodyid[c.geom1]), int(m.geom_bodyid[c.geom2])
    if 124 in (b1,b2):
        other = b2 if b1==124 else b1
        sup.append(nb(other))
print("  tray contact bodies:", sorted(set(sup)))
PY
