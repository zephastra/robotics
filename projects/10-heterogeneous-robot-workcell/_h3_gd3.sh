set -u
P=/home/ziling/projects/010_heterogeneous_robot_workcell
cd "$P" || exit 1
timeout 1800 ./.venv/bin/python - <<'PY'
import sys, numpy as np
sys.path.insert(0,"experiments"); sys.path.insert(0,"src")
import mujoco, build_p3_world as bp3
bp3.install()
import w4_plant as wp, probe_h2_w5 as H2
from humanoid007.runtime import Runtime, OPEN, GRASP, ExitPath

WORLD = "assets/world_w5_h085_loop.xml"
m = mujoco.MjModel.from_xml_path(WORLD)
d = mujoco.MjData(m); d.qpos[:] = m.key_qpos[0]; mujoco.mj_forward(m, d)
TRAY="c_payload"
r = Runtime(world=WORLD, prefix="h_", object_body=TRAY, model=m, data=d)
plant = wp.LogisticsPlant(world=WORLD, model=m, data=d)
nb = lambda i: mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, i) or ''
ng = lambda i: mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_GEOM, i) or ''
en = lambda g: bool(m.geom_contype[g]) or bool(m.geom_conaffinity[g])
hum = [g for g in range(m.ngeom) if nb(int(m.geom_bodyid[g])).startswith('h_') and en(g)]
band = [g for g in range(m.ngeom)
        if (nb(int(m.geom_bodyid[g])).startswith(('c_fixed_roller_','c_recv_roller_'))
            or nb(int(m.geom_bodyid[g]))=='c_deck') and en(g)]

# check the roller joint type -- a roller that can TRANSLATE would explain this
for j in range(m.njnt):
    n = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_JOINT, j) or ''
    if n.startswith('c_fixed_roller_2_5'):
        print("joint %s type %d (0=free,1=ball,2=slide,3=hinge)" % (n, int(m.jnt_type[j])))
print("nq=%d nv=%d" % (m.nq, m.nv))

tray0 = np.array(d.body(TRAY).xpos, float)
dflt = r.policy.default[r.arm_ids].copy()
ep = ExitPath(r, H2.anchor_at(H2.PLACE_X,0.0,H2.HANDLE_Y), dflt, ExitPath.DEFAULT_VARIANT)

print()
print("=== walking the humanoid phase to t=15, tracking the global min and t in [6.0,7.0] ===")
gmin = (9e9, None, None, None)
while d.time < 15.0:
    now = d.time
    arms, hands = H2.stage_driver(r, float(tray0[0]), now, dflt, OPEN, GRASP, ep,
                                  H2.HANDLE_Y, H2.HAND_ROLL_BIAS, dict(H2.PHASE_WINDOWS))
    d.ctrl[:] = plant.park_control()
    r.step(np.zeros(3), arms, hands, stationary=now > 2)
    if r.tick % 50 == 0:
        best = (9e9, None, None)
        zeros = 0
        for a in hum:
            for b in band:
                dd = float(mujoco.mj_geomDistance(m, d, a, b, 2.0, None))
                if dd == 0.0:
                    zeros += 1
                if dd < best[0]:
                    best = (dd, a, b)
        if best[0] < gmin[0]:
            gmin = (best[0], now, best[1], best[2])
        if 6.0 <= now <= 7.0:
            a, b = best[1], best[2]
            print("  t=%6.3f min %9.5f zeros=%d  %s(g%d) %s vs %s(g%d) %s" % (
                now, best[0], zeros, nb(int(m.geom_bodyid[a])), a,
                np.round(d.geom_xpos[a],4).tolist(),
                nb(int(m.geom_bodyid[b])), b, np.round(d.geom_xpos[b],4).tolist()))
print("  GLOBAL MIN %9.5f at t=%s between %s(g%s) and %s(g%s)" % (
    gmin[0], gmin[1], nb(int(m.geom_bodyid[gmin[2]])), gmin[2],
    nb(int(m.geom_bodyid[gmin[3]])), gmin[3]))
if gmin[2] is not None:
    print("   xpos A", np.round(d.geom_xpos[gmin[2]],4).tolist())
    print("   xpos B", np.round(d.geom_xpos[gmin[3]],4).tolist())
print()
print("humanoid base final", np.round(d.xpos[m.body('h_LINK_BASE').id],4).tolist())
print("c_fixed_roller_2_5 body xpos", np.round(d.xpos[m.body('c_fixed_roller_2_5').id],4).tolist())
PY
