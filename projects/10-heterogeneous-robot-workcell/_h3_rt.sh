set -u
P=/home/ziling/projects/010_heterogeneous_robot_workcell
cd "$P" || exit 1
./.venv/bin/python - <<'PY'
import mujoco, numpy as np, sys, time
sys.path.insert(0, 'experiments'); sys.path.insert(0, 'src')
import build_p3_world as bp3
bp3.install()
from humanoid007.runtime import Runtime, OPEN

# THE HUMAN CONTROL: does this humanoid hold a stand at all, for 300 ticks, driven by its own
# controller? If it does not, the station question is unanswerable here and H3 cannot start.
for world, x, label in (('assets/world_w5_h085.xml', 0.660132, 'candidate-B home (unplaced)'),
                        ('assets/world_w5_h085.xml', 4.13, 'H2 station'),
                        ('assets/world_w5_h085.xml', 4.09, 'pulled west'),
                        ('assets/world_w5_h085.xml', 4.20, 'pushed east (into chassis)')):
    r = Runtime(world=world, prefix='h_', object_body='c_payload')
    nb = lambda i: mujoco.mj_id2name(r.m, mujoco.mjtObj.mjOBJ_BODY, i) or ''
    ng = lambda i: mujoco.mj_id2name(r.m, mujoco.mjtObj.mjOBJ_GEOM, i) or ''
    adr = int(r.m.jnt_qposadr[r.m.joint('h_base_free').id])
    r.d.qpos[adr:adr+7] = [x, 0, 1.03, 1, 0, 0, 0]
    mujoco.mj_forward(r.m, r.d)
    b0 = np.array(r.d.xpos[r.m.body('h_LINK_BASE').id], float)
    t0 = time.time()
    for tick in range(400):
        r.step(np.zeros(3), arms=r.arm_target.copy(), hands={s: OPEN for s in r.arms},
               stationary=True)
    b1 = np.array(r.d.xpos[r.m.body('h_LINK_BASE').id], float)
    pairs = {}
    for c in range(r.d.ncon):
        con = r.d.contact[c]
        a = '%s/%s' % (nb(int(r.m.geom_bodyid[con.geom1])), ng(con.geom1) or '?')
        b = '%s/%s' % (nb(int(r.m.geom_bodyid[con.geom2])), ng(con.geom2) or '?')
        k = tuple(sorted((a, b)))
        pairs[k] = pairs.get(k, 0) + 1
    inter = {k: v for k, v in pairs.items() if ('h_' in k[0]) != ('h_' in k[1])}
    stood = float(b1[2]) - 1.03
    print('%-28s x %.5f  %-22s dz %+0.5f  ncon %2d  cross %d  (%.1fs)' % (
        label, x, 'drift ' + str(np.round(b1 - b0, 4).tolist()), stood, r.d.ncon, len(inter),
        time.time() - t0))
    for k, v in sorted(inter.items()):
        print('        %-58s %d' % (' <-> '.join(k), v))
PY
