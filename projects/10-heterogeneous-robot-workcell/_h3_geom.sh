set -u
P=/home/ziling/projects/010_heterogeneous_robot_workcell
cd "$P" || exit 1
./.venv/bin/python - <<'PY'
import mujoco, numpy as np, sys
sys.path.insert(0, 'experiments')
import build_p3_world as bp3
bp3.install()

def dump(f):
    m = mujoco.MjModel.from_xml_path(f)
    d = mujoco.MjData(m)
    # home from keyframe if present
    if m.nkey:
        k = m.key_qpos[0]
        d.qpos[:] = k
    mujoco.mj_forward(m, d)
    nm = lambda o,i: mujoco.mj_id2name(m,o,i) or ''
    print('==', f, 'nq=%d nbody=%d ngem=%d' % (m.nq, m.nbody, m.ngeom))
    for b in ('h_LINK_BASE','n_base_link','c_deck','c_payload','a_payload','w5_h085_station'):
        i = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, b)
        if i >= 0:
            print('   body %-18s xyz=%s' % (b, np.round(d.xpos[i],5).tolist()))
        g = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, b)
        if g >= 0:
            print('   geom %-18s xyz=%s size=%s' % (b, np.round(d.geom_xpos[g],5).tolist(), np.round(m.geom_size[g],5).tolist()))
    # source band extent
    xs = []
    for g in range(m.ngeom):
        n = nm(mujoco.mjtObj.mjOBJ_GEOM, g)
        if n and n.startswith('c_fixed_roller'):
            xs.append((float(d.geom_xpos[g][0]), n, float(m.geom_size[g][1]), float(d.geom_xpos[g][2])))
    xs.sort()
    print('   source rollers: %d, x %.5f..%.5f, half_len %.3f, z %.5f' % (
        len(xs), xs[0][0], xs[-1][0], xs[0][2], xs[0][3]))
    # deck rollers
    dxs = []
    for g in range(m.ngeom):
        n = nm(mujoco.mjtObj.mjOBJ_GEOM, g)
        if n and n.startswith('c_deck_roller'):
            dxs.append(float(d.geom_xpos[g][0]))
    if dxs:
        print('   deck rollers: %d, x %.5f..%.5f' % (len(dxs), min(dxs), max(dxs)))
    # recv
    rxs = []
    for g in range(m.ngeom):
        n = nm(mujoco.mjtObj.mjOBJ_GEOM, g)
        if n and n.startswith('c_recv_roller'):
            rxs.append(float(d.geom_xpos[g][0]))
    if rxs:
        print('   recv rollers: %d, x %.5f..%.5f' % (len(rxs), min(rxs), max(rxs)))

for f in ('assets/world_w2_logistic.xml','assets/world_w5_h085.xml','assets/world_p4_handover.xml'):
    dump(f)
PY
