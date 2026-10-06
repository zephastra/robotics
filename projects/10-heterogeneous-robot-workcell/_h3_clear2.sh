set -u
P=/home/ziling/projects/010_heterogeneous_robot_workcell
cd "$P" || exit 1
./.venv/bin/python - <<'PY'
import mujoco, numpy as np, sys
sys.path.insert(0, 'experiments')
import build_p3_world as bp3
bp3.install()

m = mujoco.MjModel.from_xml_path('assets/world_w5_h085.xml')
d = mujoco.MjData(m)
nm_b = lambda i: mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, i) or ''
nm_g = lambda i: mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_GEOM, i) or ''
def label(g):
    return '%s/%s' % (nm_b(int(m.geom_bodyid[g])), nm_g(g) or '<unnamed>')

d.qpos[:] = m.key_qpos[0].copy(); d.qpos[0] = 4.13
mujoco.mj_forward(m, d)

h_geoms = [g for g in range(m.ngeom) if nm_b(int(m.geom_bodyid[g])).startswith('h_')]
loop = [g for g in range(m.ngeom) if nm_b(int(m.geom_bodyid[g])).startswith(('n_','c_'))]
ft = np.zeros(6); rows = []
for a in h_geoms:
    for b in loop:
        dist = mujoco.mj_geomDistance(m, d, a, b, 5.0, ft)
        if dist < 0.06:
            rows.append((float(dist), label(a), label(b)))
rows.sort()
seen = set()
print('--- distinct penetrating/near pairs (deduped) ---')
for dist, a, b in rows:
    key = (a, b)
    if key in seen: continue
    seen.add(key)
    print('  %+9.5f  %-42s %s' % (dist, a, b))
print()
# Where exactly is the humanoid geometry that hits n_chassis?
hits = [a for dist,a,b in rows if b == 'n_base_link/n_chassis' and dist < 0.02]
print('humanoid geoms within 20 mm of n_chassis:', sorted(set(hits)))
for label_a in sorted(set(hits)):
    g = next(g for g in h_geoms if label(g) == label_a)
    print('   %-42s xyz=%s size=%s' % (label_a, np.round(d.geom_xpos[g],5).tolist(), np.round(m.geom_size[g],5).tolist()))
gch = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, 'n_chassis')
print('   n_chassis xyz=%s size=%s' % (np.round(d.geom_xpos[gch],5).tolist(), np.round(m.geom_size[gch],5).tolist()))
print()
print('=== tray geometry: which h_ geoms touch c_tray_floor at the H2 station? ===')
for dist,a,b in rows:
    if b == 'c_payload/c_tray_floor' and dist <= 0.0001:
        print('   X', a, dist)
print()
print('=== how far is c_payload west edge from the humanoid base? ===')
tp = d.xpos[m.body('c_payload').id]
print('   tray x %.5f  west edge %.5f   humanoid base x %.5f  -> gap %.5f' % (
    tp[0], tp[0]-0.065, d.xpos[m.body('h_LINK_BASE').id][0],
    (tp[0]-0.065) - d.xpos[m.body('h_LINK_BASE').id][0]))
PY
