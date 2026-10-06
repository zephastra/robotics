"""Which pixel bridges the part to the fixture's wall ring? Print the image region as text.

For the rows around the parked part, print each contiguous run of MASK pixels with the geom name it
came from. A text picture of the bridge, rather than a theory about it.
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
from workcell.tray import cell_centre_world  # noqa: E402

model, bench_z, blue_pose = FIX.scene()
data = mujoco.MjData(model)
PV.prepare(model, data, poses={PV.BLUE: blue_pose})
if model.nkey > 0 and model.key_ctrl.shape[0] > 0:
    data.ctrl[:] = model.key_ctrl[0]
g = np.asarray(PV.part_size(model), float)
cells = FIX.cells(model, data)
deck_top = float(cells['deck']['top_z'])

view = PV.View(model)
depth0 = np.asarray(view.capture(data, 'depth'), float)
seg0 = view.capture(data, 'seg')
_e, conv, _r = PV.calibrate(model, data, depth0, seg0, bench_z, g, PV.pose_of(model, data, PV.RED))

bid = model.body(PV.RED).id
jid = int(model.body_jntadr[bid])
rq, rd = int(model.jnt_qposadr[jid]), int(model.jnt_dofadr[jid])
data.qpos[rq:rq + 3] = cell_centre_world(cells, 1, z=deck_top + float(g[2]))
data.qpos[rq + 3:rq + 7] = [1.0, 0.0, 0.0, 0.0]
data.qvel[rd:rd + 6] = 0.0
mujoco.mj_forward(model, data)
for _ in range(500):
    mujoco.mj_step(model, data)

rgb = view.capture(data, 'rgb')
dep = np.asarray(view.capture(data, 'depth'), float)
seg = view.capture(data, 'seg')
per = PV.analyze(model, data, rgb, dep, bench_z, g, conv)
world = per['world']
_t, roi = PV.bench_geometry(model, data)
in_roi = ((world[..., 0] > roi[0] - 0.02) & (world[..., 0] < roi[1] + 0.02)
          & (world[..., 1] > roi[2] - 0.02) & (world[..., 1] < roi[3] + 0.02))
plane = FIX.segmentation_plane(model, bench_z)
mask = per['ok'] & (world[..., 2] > plane) & in_roi

red_gid = PV._geom_of_body(model, PV.RED)
rows = np.argwhere((seg == red_gid) & mask)
print('part mask rows %d..%d cols %d..%d'
      % (rows[:, 0].min(), rows[:, 0].max(), rows[:, 1].min(), rows[:, 1].max()))
print()


def short(gi):
    nm = FIX.geom_name(model, gi) or ('g%d' % gi)
    return nm.replace('p_fixture_', 'fx_').replace('a_payload_blue_geom', 'BLUE')


lo, hi = max(0, rows[:, 0].min() - 14), min(239, rows[:, 0].max() + 14)
for r in range(lo, hi + 1):
    runs = []
    c = 0
    while c < 320:
        if mask[r, c]:
            c0 = c
            while c < 320 and mask[r, c]:
                c += 1
            gids = seg[r, c0:c]
            uniq, cnt = np.unique(gids, return_counts=True)
            who = ','.join('%s(%d)' % (short(int(u)), int(n)) for u, n in
                           sorted(zip(uniq, cnt), key=lambda t: -t[1])[:2])
            runs.append('c%d-%d[%s]' % (c0, c - 1, who))
        else:
            c += 1
    if runs:
        print('row %3d: %s' % (r, '  '.join(runs)))
