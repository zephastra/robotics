"""Margin around the candidate mount (fast: ONE seg capture per reading).

Also checks the two parts still on the BENCH, because 02b must perceive the part it is asked to pick.
"""
import os
import sys

import numpy as np

ROOT = os.path.expanduser('~/projects/010_heterogeneous_robot_workcell')
sys.path.insert(0, os.path.join(ROOT, 'src'))
sys.path.insert(0, os.path.join(ROOT, 'experiments'))

import mujoco  # noqa: E402
import probe_p3_vision as PV  # noqa: E402
import p4_fixture as FIX  # noqa: E402
from probe_p4_vision import decide  # noqa: E402
from workcell.tray import cell_centre_world  # noqa: E402

AIM = (3.5123, -0.03)
BENCH_Z = 0.20
SWEEP = [(1.00, 0.55), (1.00, 0.48), (1.00, 0.64), (0.88, 0.55), (1.12, 0.55)]
SETTLE = 400


def free_addr(model, body):
    bid = model.body(body).id
    jid = int(model.body_jntadr[bid])
    return int(model.jnt_qposadr[jid]), int(model.jnt_dofadr[jid])


def build(h, back):
    pos = [AIM[0], AIM[1] - back, BENCH_Z + h]
    model, bench_z, blue_pose = FIX.scene(cam_pos=pos, cam_quat=PV.look_at(pos, [AIM[0], AIM[1], 0.22]))
    data = mujoco.MjData(model)
    PV.prepare(model, data, poses={PV.BLUE: blue_pose})
    if model.nkey > 0 and model.key_ctrl.shape[0] > 0:
        data.ctrl[:] = model.key_ctrl[0]
    g = np.asarray(PV.part_size(model), float)
    cells = FIX.cells(model, data)
    view = PV.View(model)
    depth0 = np.asarray(view.capture(data, 'depth'), float)
    seg0 = view.capture(data, 'seg')
    _e, conv, _r = PV.calibrate(model, data, depth0, seg0, bench_z, g, PV.pose_of(model, data, PV.RED))
    return model, data, view, conv, g, cells, float(cells['deck']['top_z'])


def reading(model, data, view, conv, g, bench_z):
    """One capture set -> (decide, why, per-component dominant geom names)."""
    rgb = view.capture(data, 'rgb')
    dep = np.asarray(view.capture(data, 'depth'), float)
    seg = view.capture(data, 'seg')
    per = PV.analyze(model, data, rgb, dep, bench_z, g, conv)
    dec, why = decide(per, g)
    doms = []
    for c in per['comps']:
        ids = seg[[y for y, x in c['pixels']], [x for y, x in c['pixels']]]
        uniq, cnt = np.unique(ids, return_counts=True)
        doms.append((c['n_pixels'], c['cls'], FIX.geom_name(model, int(uniq[int(np.argmax(cnt))]))))
    return dec, why, doms


for h, back in SWEEP:
    model, data, view, conv, g, cells, deck_top = build(h, back)
    bench_z = BENCH_Z
    dec, why, doms = reading(model, data, view, conv, g, bench_z)
    bench_red = 'OK' if dec.get('red') is not None else 'UNKNOWN'
    bench_blue = 'OK' if dec.get('blue') is not None else 'UNKNOWN'
    out = []
    for ci in (0, 1, 2):
        rq, rd = free_addr(model, PV.RED)
        data.qpos[rq:rq + 3] = cell_centre_world(cells, ci, z=deck_top + float(g[2]))
        data.qpos[rq + 3:rq + 7] = [1.0, 0.0, 0.0, 0.0]
        data.qvel[rd:rd + 6] = 0.0
        mujoco.mj_forward(model, data)
        for _ in range(SETTLE):
            mujoco.mj_step(model, data)
        dec2, why2, doms2 = reading(model, data, view, conv, g, bench_z)
        reds = [d for d in doms2 if d[1] == 'red']
        merges = [d for d in doms2 if d[2] and d[2].startswith('p_fixture') and d[0] > 600]
        out.append('c%d:%s%s' % (ci, 'OK' if dec2.get('red') is not None else 'FAIL',
                                 ('/merge%s' % (merges[0],)) if (not reds and merges) else ''))
    view.close()
    print('h=%.2f back=%.2f  bench red=%s blue=%s | %s' % (h, back, bench_red, bench_blue,
                                                           '  '.join(out)), flush=True)
print('DONE')
