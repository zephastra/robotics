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

# the shoulder geom of interest, and the band geoms
sh = [g for g in range(m.ngeom) if nb(int(m.geom_bodyid[g])) == 'h_LINK_SHOULDER_ROLL_R']
band = [g for g in range(m.ngeom)
        if nb(int(m.geom_bodyid[g])).startswith(('c_fixed_roller_','c_recv_roller_'))
        or nb(int(m.geom_bodyid[g])) == 'c_deck']
print("shoulder geoms:", [(ng(g) or '?', int(m.geom_type[g]), int(m.geom_contype[g]),
                          int(m.geom_conaffinity[g])) for g in sh])
print("band geoms:", len(band))
print()
print("=== every shoulder geom vs the CLOSEST band geom, with positions ===")
for g in sh:
    best = (9e9, None)
    for b in band:
        dist = float(mujoco.mj_geomDistance(m, d, g, b, 2.0, None))
        if dist < best[0]:
            best = (dist, b)
    b = best[1]
    dx = np.round(d.geom_xpos[g] - d.geom_xpos[b], 4).tolist()
    print("  %-22s type %d dist %.6f  to %-24s type %d  dx %s" % (
        ng(g) or '?', int(m.geom_type[g]), best[0], ng(b) or '?', int(m.geom_type[b]), dx))
print()
print("=== control: two obviously far geoms ===")
far = None
for a in (sh[0],):
    for b in band:
        if abs(float(d.geom_xpos[a][0]) - float(d.geom_xpos[b][0])) > 3.0:
            far = b; break
print("  distmax=2.0 ->", float(mujoco.mj_geomDistance(m, d, sh[0], far, 2.0, None)))
print("  distmax=9.0 ->", float(mujoco.mj_geomDistance(m, d, sh[0], far, 9.0, None)))
print()
print("=== is mj_geomDistance 0 because the pair is CONTACT-EXCLUDED? ===")
print("  shoulder contype/conaffinity:", int(m.geom_contype[sh[0]]), int(m.geom_conaffinity[sh[0]]))
b = None
for bb in band:
    if float(mujoco.mj_geomDistance(m, d, sh[0], bb, 2.0, None)) == 0.0:
        b = bb; break
print("  a band geom reporting 0.0:", ng(b) if b is not None else None,
      (int(m.geom_contype[b]), int(m.geom_conaffinity[b])) if b is not None else None)
if b is not None:
    print("  their xpos:", np.round(d.geom_xpos[sh[0]],4).tolist(), np.round(d.geom_xpos[b],4).tolist())
    print("  TRUE centre distance: %.4f" % float(np.linalg.norm(d.geom_xpos[sh[0]] - d.geom_xpos[b])))
    print("  mj_geomDistance with distmax 9.0:", float(mujoco.mj_geomDistance(m, d, sh[0], b, 9.0, None)))
    print("  ncon at this state:", d.ncon)
PY
