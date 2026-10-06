set -u
P=/home/ziling/projects/010_heterogeneous_robot_workcell
cd "$P" || exit 1
echo "=== sha256 of the three worlds ==="
sha256sum assets/world_w2_logistic.xml assets/world_w5_h085.xml assets/world_p4_handover.xml
echo
echo "=== h_LINK_BASE line ==="
for f in assets/world_w2_logistic.xml assets/world_w5_h085.xml assets/world_p4_handover.xml; do
  printf '%-34s ' "$f"
  grep -o '<body name="h_LINK_BASE" pos="[^"]*"' "$f"
done
echo
echo "=== structure + free joints + qpos0 ==="
./.venv/bin/python - <<'PY'
import mujoco, numpy as np
for f in ('assets/world_w2_logistic.xml','assets/world_w5_h085.xml','assets/world_p4_handover.xml'):
    m = mujoco.MjModel.from_xml_path(f)
    print(f, 'nq=%d nv=%d nu=%d nbody=%d ngeom=%d neq=%d nkey=%d' % (m.nq,m.nv,m.nu,m.nbody,m.ngeom,m.neq,m.nkey))
    names = {}
    for j in range(m.njnt):
        if int(m.jnt_type[j]) == int(mujoco.mjtJoint.mjJNT_FREE):
            nm = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_JOINT, j)
            names[nm] = int(m.jnt_qposadr[j])
            print('    free %-22s qposadr=%3d dofadr=%3d' % (nm, int(m.jnt_qposadr[j]), int(m.jnt_dofadr[j])))
    print('    qpos0[0:7] =', np.round(m.qpos0[0:7],6).tolist())
PY
echo
echo "=== plant hard requirements: n_ wheel/pusher actuators ==="
./.venv/bin/python - <<'PY'
import mujoco
for f in ('assets/world_w2_logistic.xml','assets/world_w5_h085.xml'):
    m = mujoco.MjModel.from_xml_path(f)
    want = ('n_wheel_left_motor','n_wheel_right_motor','c_pusher_lift','c_pusher_slide')
    got = {}
    for w in want:
        i = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_ACTUATOR, w)
        got[w] = i
    print(f, got)
PY
