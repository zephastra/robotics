set -u
P=/home/ziling/projects/010_heterogeneous_robot_workcell
cd "$P" || exit 1
# Can the humanoid stand at the H2 station (4.13) in candidate B, and does it collide with the
# W5 loop's AMT chassis parked at n_base_link x=4.41372?
./.venv/bin/python - <<'PY'
import mujoco, numpy as np, sys
sys.path.insert(0, 'experiments'); sys.path.insert(0, 'src')
import build_p3_world as bp3
bp3.install()

m = mujoco.MjModel.from_xml_path('assets/world_w5_h085.xml')
d = mujoco.MjData(m)
k = m.key_qpos[0].copy()
nm_b = lambda i: mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, i) or ''
nm_g = lambda i: mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_GEOM, i) or ''

# place humanoid base at the H2 station x, everything else at home
q = k.copy()
q[0] = 4.13
d.qpos[:] = q
mujoco.mj_forward(m, d)
print('humanoid base   ', np.round(d.xpos[m.body('h_LINK_BASE').id],5).tolist())
print('chassis base    ', np.round(d.xpos[m.body('n_base_link').id],5).tolist())

# distance from every humanoid geom to every n_/c_ geom
h_geoms = [g for g in range(m.ngeom) if nm_b(int(m.geom_bodyid[g])).startswith('h_')]
loop_geoms = [g for g in range(m.ngeom)
              if nm_b(int(m.geom_bodyid[g])).startswith(('n_','c_'))]
print('h_ geoms %d, n_/c_ geoms %d' % (len(h_geoms), len(loop_geoms)))
worst = []
fromto = np.zeros(6)
for a in h_geoms:
    for b in loop_geoms:
        dist = mujoco.mj_geomDistance(m, d, a, b, 5.0, fromto)
        if dist < 0.06:
            worst.append((round(float(dist),5), nm_g(a), nm_g(b)))
worst.sort()
print('closest %d pairs under 60 mm:' % len(worst))
for row in worst[:15]:
    print('   ', row)
PY
