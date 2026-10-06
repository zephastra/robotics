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
nb = lambda i: mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, i) or ''
ng = lambda i: mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_GEOM, i) or ''
q = m.key_qpos[0].copy()
q[0] = 4.09
d.qpos[:] = q
mujoco.mj_forward(m, d)

print('=== every humanoid geom that comes within 1 mm of c_payload at station 4.09 ===')
tan = [g for g in range(m.ngeom) if nb(int(m.geom_bodyid[g])) == 'c_payload']
hum = [g for g in range(m.ngeom) if nb(int(m.geom_bodyid[g])).startswith('h_')]
ft = np.zeros(6)
rows = []
for a in hum:
    for b in tan:
        dist = float(mujoco.mj_geomDistance(m, d, a, b, 5.0, ft))
        if dist < 0.001:
            rows.append((dist, nb(int(m.geom_bodyid[a])), ng(a) or '?', ng(b) or '?'))
for dist, hb, hg, tg in sorted(rows):
    print('   %+9.6f  %-24s %-14s -> c_payload/%s' % (dist, hb, hg, tg))
print()
print('=== the hip geoms, and where they actually are ===')
for body in ('h_LINK_HIP_YAW_L','h_LINK_HIP_YAW_R','h_LINK_HIP_PITCH_L','h_LINK_HIP_ROLL_R'):
    bi = m.body(body).id
    for g in range(m.ngeom):
        if int(m.geom_bodyid[g]) == bi:
            print('   %-22s %-12s xyz=%s size=%s' % (body, ng(g) or '?',
                  np.round(d.geom_xpos[g],5).tolist(), np.round(m.geom_size[g],5).tolist()))
print()
print('=== c_payload geoms, x extents ===')
for g in tan:
    z = float(d.geom_xpos[g][0]); s = float(m.geom_size[g][0])
    print('   %-24s x %.5f +- %.5f  -> [%.5f, %.5f]  body=%s' % (
        ng(g) or '?', z, s, z-s, z+s, nb(int(m.geom_bodyid[g]))))
print()
print('=== is the humanoid OVERLAPPING ITSELF at the hips/tray region? ===')
# self-contact among h_ geoms, to establish whether 0.0 is a tray interference or a robot-internal one
hits = 0
for i, a in enumerate(hum):
    for b in hum[i+1:]:
        dist = float(mujoco.mj_geomDistance(m, d, a, b, 5.0, ft))
        if dist < 0.0:
            hits += 1
            if hits <= 10:
                print('   self %+8.5f %s/%s <-> %s/%s' % (dist,
                    nb(int(m.geom_bodyid[a])), ng(a) or '?', nb(int(m.geom_bodyid[b])), ng(b) or '?'))
print('   total self-penetrating pairs: %d' % hits)
PY
