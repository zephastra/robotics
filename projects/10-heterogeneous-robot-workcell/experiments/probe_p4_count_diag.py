"""02c diagnostic: the part is IN the cell but is not a component. Why?

Prints, for the exact state 02c's case 1 reads: the fixture plane, the mask size, every geom's
segmentation pixels, whether the red part's OWN pixels survive the plane+ROI filter, and the
components with the geoms they came from.
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
from workcell.tray import cell_of, cell_centre_world  # noqa: E402

SETTLE = 500

model, bench_z, blue_pose = FIX.scene()
data = mujoco.MjData(model)
PV.prepare(model, data, poses={PV.BLUE: blue_pose})
if model.nkey > 0 and model.key_ctrl.shape[0] > 0:
    data.ctrl[:] = model.key_ctrl[0]

g = np.asarray(PV.part_size(model), float)
cells = FIX.cells(model, data)
deck_top = float(cells['deck']['top_z'])
plane = FIX.segmentation_plane(model, bench_z)

view = PV.View(model)
depth0 = np.asarray(view.capture(data, 'depth'), float)
seg0 = view.capture(data, 'seg')
_e, conv, _r = PV.calibrate(model, data, depth0, seg0, bench_z, g, PV.pose_of(model, data, PV.RED))

# park the red part in cell 1, exactly as 02c does
rq, rd = None, None
bid = model.body(PV.RED).id
jid = int(model.body_jntadr[bid])
rq, rd = int(model.jnt_qposadr[jid]), int(model.jnt_dofadr[jid])
target = cell_centre_world(cells, 1, z=deck_top + float(g[2]))
data.qpos[rq:rq + 3] = target
data.qpos[rq + 3:rq + 7] = [1.0, 0.0, 0.0, 0.0]
data.qvel[rd:rd + 6] = 0.0
mujoco.mj_forward(model, data)
for _ in range(SETTLE):
    mujoco.mj_step(model, data)

truth = np.asarray(data.xpos[bid], float)
print('bench_z %.5f  plane %.5f  fixture floor top %.5f' % (bench_z, plane, deck_top))
print('red parked: target %s -> actual %s' % (np.round(target, 5), np.round(truth, 5)))
print('red body top would be %.5f (needs > plane by %.5f)'
      % (truth[2] + float(g[2]), truth[2] + float(g[2]) - plane))
print()

rgb = view.capture(data, 'rgb')
dep = np.asarray(view.capture(data, 'depth'), float)
seg = view.capture(data, 'seg')
per = PV.analyze(model, data, rgb, dep, bench_z, g, conv)
world = per['world']
_ok = per['ok']
_t, roi = PV.bench_geometry(model, data)
in_roi = ((world[..., 0] > roi[0] - 0.02) & (world[..., 0] < roi[1] + 0.02)
          & (world[..., 1] > roi[2] - 0.02) & (world[..., 1] < roi[3] + 0.02))
above = per['ok'] & (world[..., 2] > plane)
print('mask: above-plane %d px ; above-plane AND in ROI %d px' % (int(above.sum()),
                                                                 int((above & in_roi).sum())))
print('roi x[%.4f, %.4f] y[%.4f, %.4f]' % roi)
print()

red_gid = PV._geom_of_body(model, PV.RED)
print('red geom id %d: seg px %d ; of those, above-plane %d ; above-plane AND in ROI %d'
      % (red_gid, int((seg == red_gid).sum()),
         int(((seg == red_gid) & above).sum()),
         int(((seg == red_gid) & above & in_roi).sum())))
rows = np.argwhere((seg == red_gid) & above & in_roi)
if rows.size:
    print('   its unprojected z: min %.4f max %.4f' % (world[..., 2][rows[:, 0], rows[:, 1]].min(),
                                                       world[..., 2][rows[:, 0], rows[:, 1]].max()))
    print('   image rows %d..%d, cols %d..%d'
          % (rows[:, 0].min(), rows[:, 0].max(), rows[:, 1].min(), rows[:, 1].max()))
print()

counts = {}
for gi in range(model.ngeom):
    n = int((seg == gi).sum())
    if n:
        counts[gi] = n
print('geoms with segmentation pixels (top 14):')
for gi, n in sorted(counts.items(), key=lambda t: -t[1])[:14]:
    nm = FIX.geom_name(model, gi) or ('g%d' % gi)
    ab = int(((seg == gi) & above & in_roi).sum())
    print('   %-24s seg=%-6d above+roi=%d  %s' % (nm, n, ab, 'ON BENCH' if ab else ''))
print()
print('components: %d' % len(per['comps']))
for c in sorted(per['comps'], key=lambda c: -c['n_pixels']):
    ids = seg[[y for y, x in c['pixels']], [x for y, x in c['pixels']]]
    uniq, cnt = np.unique(ids, return_counts=True)
    top = int(uniq[int(np.argmax(cnt))])
    rr = [y for y, x in c['pixels']]
    cc = [x for y, x in c['pixels']]
    print('   n=%-6d cls=%-5s on_bench=%-5s rows %d..%d cols %d..%d z[%.4f,%.4f] dominant=%s (%d%%)'
          % (c['n_pixels'], c['cls'], c['on_bench'], min(rr), max(rr), min(cc), max(cc),
             c['z_min'], c['z_max'], FIX.geom_name(model, top), 100 * cnt.max() // c['n_pixels']))
    print('        from: %s' % ', '.join(
        '%s:%d' % (FIX.geom_name(model, int(uniq[i])), int(cnt[i]))
        for i in np.argsort(-cnt)[:4]))
dec, why = decide(per, g)
print()
print('decide -> %s' % {k: (None if v is None else np.round(v, 4).tolist()) for k, v in dec.items()})
print('why    -> %s' % why)
print('cell_of(red truth) = %s' % cell_of(cells, truth))
