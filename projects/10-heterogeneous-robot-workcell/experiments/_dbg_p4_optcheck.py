"""Audit my own option list before acting on it.

Two numbers decide whether the "move the station / the table" lever exists at all:

  1. how far the TRAY must be carried to get from its start pose onto the source band, versus the
     tray-carry capability `reports/p4-reach-01` measured (its `carry_m`);
  2. the station shift the presentation table allows.

If (1) is already at its limit, moving the tray or the table west buys nothing, and the only lever
left on the height axis is the band's own height.

★ These are read from the EXISTING authoritative report, not re-run: `p4-reach-01` is the evidence
for the reach, and re-deriving it here would make a second, unpublishable source of the same number.
"""
import json
from pathlib import Path

ROOT = Path('/home/ziling/projects/010_heterogeneous_robot_workcell')
rep = json.loads((ROOT / 'reports' / 'p4-reach-01' / 'report.json').read_text(encoding='utf-8'))
m = rep['measurements']

band_x = m['band_first_roller_x']
tray_x = m['tray_start_x'] if m['tray_start_x'] > 1 else None
print('band_first_roller_x      :', band_x)
print('report tray_start_x      :', m['tray_start_x'], '(this run used the UN-migrated world)')

# The P4 world's tray origin, from the builder's own declaration.
builder = (ROOT / 'experiments' / 'build_p4_handover_world.py').read_text(encoding='utf-8')
import re
val = re.search(r'BASE_X_M\s*=\s*([0-9.]+)', builder)
print('builder BASE_X_M         :', val.group(1) if val else 'not found')
p4 = json.loads((ROOT / 'reports' / 'p4-crouch-01' / 'report.json').read_text(encoding='utf-8'))
print('p4 world sha             :', p4['world']['sha256'][:16])

# Measure the two directly from the compiled P4 world: the tray's origin and the band's first crown.
import sys
sys.path.insert(0, str(ROOT / 'experiments'))
sys.path.insert(0, str(ROOT / 'src'))
import numpy as np                      # noqa: E402
import mujoco                           # noqa: E402
import build_p3_world as bp3            # noqa: E402
bp3.install()
import merge_world as mw                # noqa: E402

WORLD = ROOT / 'assets' / 'world_p4_handover.xml'
model = mujoco.MjModel.from_xml_path(str(WORLD))
data = mujoco.MjData(model)
data.qpos[:] = np.asarray(mw.merged_home(model)[0], dtype=float)
mujoco.mj_forward(model, data)

tray_bid = model.body('c_payload').id
tray_origin = np.array(data.xpos[tray_bid], dtype=float)

band_geoms = [g for g in range(model.ngeom)
              if 'c_fixed_roller' in (mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, g) or '')]
roller_x = sorted(float(data.geom_xpos[g][0]) for g in band_geoms)
tray_half = None
for g in range(model.ngeom):
    if int(model.geom_bodyid[g]) == tray_bid:
        pass
# the tray's x half-extent, from the union of its own geoms
xs = []
for g in range(model.ngeom):
    bn = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, int(model.geom_bodyid[g])) or ''
    if bn != 'c_payload':
        continue
    half = model.geom_size[g][:3]
    R = np.array(data.geom_xmat[g], dtype=float).reshape(3, 3)
    c = np.array(data.geom_xpos[g], dtype=float)
    for sx in (-1, 1):
        for sy in (-1, 1):
            for sz in (-1, 1):
                xs.append((c + R @ (sx * half[0], sy * half[1], sz * half[2]))[0])
tray_half = (max(xs) - min(xs)) / 2.0

print()
print('--- measured in the P4 world ---')
print('tray origin (c_payload)  :', np.round(tray_origin, 5))
print(f'tray footprint x [{min(xs):.4f}, {max(xs):.4f}]  half {tray_half:.4f}')
print(f'source band first crown x: {roller_x[0]:.5f}  ({len(band_geoms)} crown geoms)')
target_x = roller_x[0] + tray_half
required = target_x - float(tray_origin[0])
print(f'target tray centre x     : {target_x:.5f}  (west edge flush with the first crown)')
print()
print(f'*** required tray carry  : {required * 1000:.2f} mm')
print(f'*** carry capability     : {m["carry_m"] * 1000:.2f} mm   (reports/p4-reach-01)')
print(f'*** slack                : {(m["carry_m"] - required) * 1000:.2f} mm')
print()
gap = band_x - (tray_origin[0] + tray_half)
print(f'gap from the tray to the first crown: {gap * 1000:.2f} mm')
