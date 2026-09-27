"""Is the knee-vs-band intersection a real COLLISION, or two geoms the filter keeps apart?

`mj_geomDistance` deliberately ignores `contype`/`conaffinity`, so it reports an overlap between
geoms that would never touch in the simulation. A penetration that no contact model acts on is a
modelling artefact, not a wall. This prints the filter words of the geoms involved, on both sides.
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

model = mujoco.MjModel.from_xml_path(str(ROOT / 'assets' / 'world_p4_handover.xml'))
data = mujoco.MjData(model)
data.qpos[:] = np.asarray(mw.merged_home(model)[0], dtype=float)
mujoco.mj_forward(model, data)

GEOM = mujoco.mjtObj.mjOBJ_GEOM
BODY = mujoco.mjtObj.mjOBJ_BODY


def show(tag, pred):
    print(f'--- {tag} ---')
    for g in range(model.ngeom):
        n = mujoco.mj_id2name(model, GEOM, g) or '<unnamed>'
        if not pred(g, n):
            continue
        bn = mujoco.mj_id2name(model, BODY, int(model.geom_bodyid[g])) or '<world>'
        print(f'  geom {n:26s} body {bn:24s} contype={int(model.geom_contype[g]):3d} '
              f'conaffinity={int(model.geom_conaffinity[g]):3d} '
              f'group={int(model.geom_group[g])} condim={int(model.geom_condim[g])}')


show('the band crowns the knee hits', lambda g, n: 'c_fixed_roller' in n)
show('humanoid knee / hip geoms', lambda g, n: (
    mujoco.mj_id2name(model, BODY, int(model.geom_bodyid[g])) or '') in
    ('h_LINK_KNEE_PITCH_L', 'h_LINK_KNEE_PITCH_R', 'h_LINK_HIP_YAW_L', 'h_LINK_HIP_YAW_R'))

print()
print('--- does the filter let ANY humanoid geom collide with ANY band geom? ---')
h = ['h_LINK_KNEE_PITCH_L', 'h_LINK_KNEE_PITCH_R', 'h_LINK_HIP_YAW_L', 'h_LINK_HIP_YAW_R']
pairs_tested = 0
allowed = 0
for g in range(model.ngeom):
    gb = mujoco.mj_id2name(model, BODY, int(model.geom_bodyid[g])) or ''
    if gb not in h:
        continue
    for k in range(model.ngeom):
        kb = mujoco.mj_id2name(model, GEOM, k) or ''
        if 'c_fixed_roller' not in kb:
            continue
        pairs_tested += 1
        c1, a1 = int(model.geom_contype[g]), int(model.geom_conaffinity[g])
        c2, a2 = int(model.geom_contype[k]), int(model.geom_conaffinity[k])
        if (c1 & a2) or (c2 & a1):
            allowed += 1
print(f'  {pairs_tested} (knee/hip geom, crown geom) pairs; {allowed} would generate a contact')
print()
print('--- and the table, for contrast (the cap that turned out to be mine) ---')
show('the presentation table', lambda g, n: 'source_table' in n)
