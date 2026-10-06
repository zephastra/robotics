set -u
P=/home/ziling/projects/010_heterogeneous_robot_workcell
cd "$P" || exit 1
./.venv/bin/python - <<'PY'
import mujoco, numpy as np, sys
sys.path.insert(0, 'experiments')
import build_p3_world as bp3
bp3.install()

m = mujoco.MjModel.from_xml_path('assets/world_w5_h085.xml')
nb = lambda i: mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, i) or ''
ng = lambda i: mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_GEOM, i) or ''

# Which h_ geoms are CONTACT-ENABLED? contype/conaffinity decide whether physics sees them at all.
print('=== geometry class of the hip/tray geoms (contype, conaffinity, group) ===')
for g in range(m.ngeom):
    b = nb(int(m.geom_bodyid[g]))
    if b in ('h_LINK_HIP_YAW_L','h_LINK_HIP_YAW_R') or b == 'c_payload':
        print('   %-20s %-18s type=%d contype=%d conaffinity=%d group=%d' % (
            b, ng(g) or '<unnamed>', int(m.geom_type[g]), int(m.geom_contype[g]),
            int(m.geom_conaffinity[g]), int(m.geom_group[g])))
print()
print('=== how many h_ geoms are collision-enabled at all? ===')
h_all = [g for g in range(m.ngeom) if nb(int(m.geom_bodyid[g])).startswith('h_')]
h_col = [g for g in h_all if int(m.geom_contype[g]) or int(m.geom_conaffinity[g])]
print('   h_ geoms %d, collision-enabled %d' % (len(h_all), len(h_col)))
print('   enabled bodies:', sorted({nb(int(m.geom_bodyid[g])) for g in h_col}))
print()
print('=== THE PHYSICS TEST: settle at each station and read data.ncon ===')
for x in (0.660132, 4.13, 4.11, 4.09, 4.07, 4.05):
    d = mujoco.MjData(m)
    q = m.key_qpos[0].copy(); q[0] = x
    d.qpos[:] = q
    mujoco.mj_forward(m, d)
    # let it settle under gravity briefly so contact forces are real
    for _ in range(400):
        mujoco.mj_step(m, d)
    pairs = {}
    for c in range(d.ncon):
        con = d.contact[c]
        a = '%s/%s' % (nb(int(m.geom_bodyid[con.geom1])), ng(con.geom1) or '?')
        b = '%s/%s' % (nb(int(m.geom_bodyid[con.geom2])), ng(con.geom2) or '?')
        key = tuple(sorted((a, b)))
        pairs[key] = pairs.get(key, 0) + 1
    inter = {k: v for k, v in pairs.items() if ('h_' in k[0]) != ('h_' in k[1])}
    print('  station %.6f  drift_x %+8.5f  ncon %3d  cross-entity pairs %d' % (
        x, float(d.qpos[0]) - x, d.ncon, len(inter)))
    for k, v in sorted(inter.items()):
        print('        %-58s %d' % (' <-> '.join(k), v))
PY
