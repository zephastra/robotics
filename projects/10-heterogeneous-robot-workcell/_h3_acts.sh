set -u
P=/home/ziling/projects/010_heterogeneous_robot_workcell
cd "$P" || exit 1
timeout 600 ./.venv/bin/python - <<'PY'
import sys, numpy as np
sys.path.insert(0,"experiments"); sys.path.insert(0,"src")
import mujoco, build_p3_world as bp3
bp3.install()
from humanoid007.runtime import Runtime

WORLD = "assets/world_w5_h085_loop.xml"
m = mujoco.MjModel.from_xml_path(WORLD)
d = mujoco.MjData(m)
r = Runtime(world=WORLD, prefix="h_", object_body="c_payload", model=m, data=d)
human = sorted(set(int(i) for i in r.body_act) | {int(i) for s in r.hand_act.values() for i in s})
other = [a for a in range(m.nu) if a not in set(human)]
name = lambda a: mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_ACTUATOR, a) or '?'
print("nu total %d ; humanoid %d ; plant %d ; overlap %d" % (m.nu, len(human), len(other),
      len(human) + len(other) - m.nu))
print()
print("plant actuators (%d):" % len(other))
for a in other:
    jid = int(m.actuator_trnid[a,0])
    jn = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_JOINT, jid) or '?'
    bias = int(m.actuator_biastype[a])
    print("   %-22s -> %-26s biastype %d biasprm %s ctrlrange %s" % (
        name(a), jn, bias, np.round(m.actuator_biasprm[a],2).tolist(),
        np.round(m.actuator_ctrlrange[a],2).tolist()))
print()
print("chassis driving joints:")
for j in ("n_slide_x","n_slide_y","n_wheel_left_joint","n_wheel_right_joint"):
    jid = m.joint(j).id
    acts = [a for a in range(m.nu) if int(m.actuator_trnid[a,0]) == jid]
    print("   %-22s actuators %s" % (j, [name(a) for a in acts]))
PY
