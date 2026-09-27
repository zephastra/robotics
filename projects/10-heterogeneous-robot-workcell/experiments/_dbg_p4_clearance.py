"""Is the station cap MY assumption or the model's geometry?

`reports/p4-reach-01` capped the station shift with `shift_max = band_first_roller -
(table_east_edge_offset) - base_x0`. That is a claim in ONE dimension: "the table may not pass the
band". But the table's top plate is at z 0.810 and the band's crown at 0.485 -- the table OVERHANGS
the band by 0.325 m of height, and an overhang collides with nothing.

If the real 3-D clearance allows the shift the arms actually want (3.56 m at a 0.28 m crouch), then
no geometry change is needed at all and the only thing wrong was my cap.

`mj_geomDistance` measures the true 3-D distance between two geoms regardless of what the contact
filter would allow, which is what this question needs -- `data.ncon` would answer "are they
colliding" with the contact tables' permissions mixed in.
"""
import sys
from pathlib import Path

import numpy as np

ROOT = Path('/home/ziling/projects/010_heterogeneous_robot_workcell')
sys.path.insert(0, str(ROOT / 'experiments'))
sys.path.insert(0, str(ROOT / 'src'))

import mujoco  # noqa: E402
import build_p3_world as bp3  # noqa: E402

bp3.install()
import merge_world as mw  # noqa: E402

WORLD = ROOT / 'assets' / 'world_p4_handover.xml'
DESIGN_SHIFT = 3.4184          # the P4 world was built at this station shift

model = mujoco.MjModel.from_xml_path(str(WORLD))
data = mujoco.MjData(model)
home = np.asarray(mw.merged_home(model)[0], dtype=float)
data.qpos[:] = home
mujoco.mj_forward(model, data)

base_id = model.body('h_LINK_BASE').id
base_x_design = float(data.xpos[base_id][0])

# ★ `h_source_table` is a GEOM, not a body: the model has no body by that name, and looking one
# up raises. Its parent body decides how it moves -- a geom on the world body is moved by editing
# `model.geom_pos`, one on a named body by moving the body.
table_geoms = [g for g in range(model.ngeom)
               if 'source_table' in (mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, g) or '')]
if not table_geoms:
    raise SystemExit('no geom named *source_table* in this world')
table_bodies = sorted({int(model.geom_bodyid[g]) for g in table_geoms})
print('table geom ids       :', table_geoms)
print('table parent bodies  :',
      [mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, b) or f'<world {b}>' for b in table_bodies])
table_geom_pos0 = {g: np.array(model.geom_pos[g], dtype=float) for g in table_geoms}
band_geoms = [g for g in range(model.ngeom)
              if 'c_fixed_roller' in (mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, g) or '')]
print(f'base x at the design point : {base_x_design:.6f}')
print(f'table geoms {len(table_geoms)}, band crown geoms {len(band_geoms)}')
tz = np.array(data.geom_xpos[table_geoms[0]], dtype=float)
print(f'table first geom world xyz : {np.round(tz, 5)}')
print(f'band crown z range         : '
      f'{min(float(data.geom_xpos[g][2]) for g in band_geoms):.4f} .. '
      f'{max(float(data.geom_xpos[g][2]) for g in band_geoms):.4f}')
print(f'band crown x range         : '
      f'{min(float(data.geom_xpos[g][0]) for g in band_geoms):.4f} .. '
      f'{max(float(data.geom_xpos[g][0]) for g in band_geoms):.4f}')
print()

# which humanoid geoms? for a body-vs-band check as well
humanoid_geoms = [g for g in range(model.ngeom)
                  if (mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY,
                                        int(model.geom_bodyid[g])) or '').startswith('h_')]

fromto = np.zeros(6, dtype=float)
print(f'{"shift":>7} {"base_x":>9} {"table_x_lo":>10} {"table_x_hi":>10} {"tab-vs-band":>12} '
      f'{"human-vs-band":>13}')
for shift in (3.4184, 3.45, 3.50, 3.55, 3.56, 3.60, 3.65, 3.70, 3.80):
    dx = shift - DESIGN_SHIFT
    for g in table_geoms:
        model.geom_pos[g] = table_geom_pos0[g] + np.array([dx, 0.0, 0.0])
    q = np.array(home)
    q[0] += dx                                   # the base is a free joint: its x is qpos[0]
    data.qpos[:] = q
    mujoco.mj_forward(model, data)

    txs = [float(data.geom_xpos[g][0]) for g in table_geoms]
    tmin, hmin = np.inf, np.inf
    for g in table_geoms:
        for b in band_geoms:
            d = float(mujoco.mj_geomDistance(model, data, g, b, 1.0, fromto))
            tmin = min(tmin, d)
    for g in humanoid_geoms:
        for b in band_geoms:
            d = float(mujoco.mj_geomDistance(model, data, g, b, 1.0, fromto))
            hmin = min(hmin, d)
    print(f'{shift:7.4f} {float(data.xpos[base_id][0]):9.5f} {min(txs):10.4f} {max(txs):10.4f} '
          f'{tmin * 1000:11.2f}mm {hmin * 1000:12.2f}mm')

print()
print('Reach study, minimum station shift the ARMS need, per crouch depth:')
for dz, need in ((-0.24, 3.66), (-0.28, 3.56), (-0.30, 3.52), (-0.32, 3.48), (-0.34, 3.44),
                 (-0.36, 3.42)):
    print(f'   crouch {dz:+.2f} m -> shift >= {need:.2f} m')
print()
print('For each of those shifts, the table-vs-band clearance is in the table above.')
print('If the clearance stays positive where the arms need it, the cap in `p4-reach-01` is an')
print('assumption about ONE dimension and not a property of the geometry.')
