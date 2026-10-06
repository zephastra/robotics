set -u
P=/home/ziling/projects/010_heterogeneous_robot_workcell
cd "$P" || exit 1
timeout 600 ./.venv/bin/python - <<'PY'
import sys, numpy as np
sys.path.insert(0,"experiments"); sys.path.insert(0,"src")
import mujoco, build_p3_world as bp3
bp3.install()

m = mujoco.MjModel.from_xml_path("assets/world_w5_h085_loop.xml")
d = mujoco.MjData(m); d.qpos[:] = m.key_qpos[0]; mujoco.mj_forward(m, d)
nb = lambda i: mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, i) or ''
ng = lambda i: mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_GEOM, i) or ''

tgt = 'c_fixed_roller_2_5'
gb = [g for g in range(m.ngeom) if ng(g) == tgt]
gbd = [b for b in range(m.nbody) if nb(b) == tgt]
print("geom ids named %s: %s" % (tgt, gb))
print("body ids named %s: %s" % (tgt, gbd))
for g in gb:
    print("   geom %d type %d body %s contype %d conaffinity %d" % (
        g, int(m.geom_type[g]), nb(int(m.geom_bodyid[g])), int(m.geom_contype[g]), int(m.geom_conaffinity[g])))
    print("   geom_xpos", np.round(d.geom_xpos[g],4).tolist(), "size", np.round(m.geom_size[g],4).tolist())
for b in gbd:
    print("   body %d xpos %s" % (b, np.round(d.xpos[b],4).tolist()))

sh = [g for g in range(m.ngeom) if nb(int(m.geom_bodyid[g])) == 'h_LINK_SHOULDER_ROLL_R'
      and (int(m.geom_contype[g]) or int(m.geom_conaffinity[g]))]
print()
print("shoulder collision-enabled geom(s):", sh)
for g in sh:
    print("   xpos", np.round(d.geom_xpos[g],4).tolist(), "size", np.round(m.geom_size[g],4).tolist())
    for t in gb:
        print("   vs %s -> mj_geomDistance(2.0)=%.6f  (9.0)=%.6f  centre dist=%.4f" % (
            tgt, float(mujoco.mj_geomDistance(m,d,g,t,2.0,None)),
            float(mujoco.mj_geomDistance(m,d,g,t,9.0,None)),
            float(np.linalg.norm(d.geom_xpos[g]-d.geom_xpos[t]))))
print()
print("=== the other c_fixed_roller_2_* geom positions ===")
for g in range(m.ngeom):
    n = ng(g)
    if n.startswith('c_fixed_roller_2_'):
        print("   %-24s x %8.5f y %8.5f z %8.5f" % (n, d.geom_xpos[g][0], d.geom_xpos[g][1], d.geom_xpos[g][2]))
PY
