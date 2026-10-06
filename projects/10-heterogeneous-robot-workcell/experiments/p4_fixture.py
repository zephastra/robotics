#!/usr/bin/env python3
"""The P4 world, and where its open loading fixture's cells are.

WHY THIS FILE EXISTS AT ALL
`experiments/probe_p3_vision.py` is FROZEN and hard-codes its world path, and `assets/world_p3_cell.xml`
is frozen too. The P4 steps need the DERIVED world that `experiments/build_p4_cell_world.py` writes, so
exactly one override has to exist somewhere. It is here, it is named, and every report carries both
world hashes (see `world_provenance`) instead of quietly loading whatever path was lying around.

`scene()` also refuses to return a model that has no `p_fixture`, so an override that failed to take is
a stopped probe rather than a run that measures the wrong world and says PASS.
"""
import hashlib
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src'))
sys.path.insert(0, str(ROOT / 'experiments'))

import mujoco  # noqa: E402
import numpy as np  # noqa: E402

import probe_p3_vision as PV  # noqa: E402
from workcell.tray import derive_cells  # noqa: E402

SOURCE_WORLD = ROOT / 'assets' / 'world_p3_cell.xml'
WORLD = ROOT / 'assets' / 'world_p4_cell.xml'
FIXTURE_BODY = 'p_fixture'


def world_provenance():
    """Both worlds, pinned by hash, so a report cannot be read without knowing which one ran."""
    out = {
        'p4_world': WORLD.name, 'p4_world_sha256': _sha(WORLD), 'p4_world_bytes': WORLD.stat().st_size,
        'built_from': SOURCE_WORLD.name, 'built_from_sha256': _sha(SOURCE_WORLD),
        'builder': 'experiments/build_p4_cell_world.py',
        'why_derived': 'the frozen world places the tray 2.2x beyond the arm\'s measured reach and its '
                       'lid plates over all three cells; see D115 and reports/p4-count-01',
    }
    if _LAST_MOUNT:
        out['camera'] = dict(_LAST_MOUNT)
    return out


#: DECLARED camera mount: height above the bench, and standoff back along -y from the aim point.
#:
#: It is declared because `probe_p3_vision.build_scene()`'s DEFAULT mount cannot read this fixture:
#: measured, the far rib (`p_fixture_rib_1`, y +0.0767) projects to image rows 90..93 while the part
#: in the same cell projects to rows 94..108, so the 4-connected flood fill in `segment_parts` walks
#: from the part into the fixture's wall ring and the component's mean colour becomes the FIXTURE's --
#: `decide()` then reports "0 candidate(s) for a red part". The mechanism is foreshortening: at the
#: default 47 degrees from vertical, 0.45 m of depth lands in ~90 image rows, so anything within about
#: 60 mm BEHIND a part is one hop away. A steeper view removes it.
#:
#: Chosen by measuring 5 mounts x 3 cells (`_diag/out_mount3.txt`): at this mount all three cells AND
#: both parts on the bench resolve, and so do all four neighbours (+-0.12 m in height, +-0.09 m in
#: standoff). A mount whose neighbourhood also fails would be luck.
CAM_ABOVE_BENCH_M = 1.00
CAM_BACK_M = 0.55
CAM_ABOVE_AIM_M = 0.02
_LAST_MOUNT = {}


def mount():
    """The mount, DERIVED from the world: aimed at the midpoint of the pick area and the fixture."""
    m = mujoco.MjModel.from_xml_path(str(WORLD))
    d = mujoco.MjData(m)
    d.qpos[:] = m.key_qpos[0]
    mujoco.mj_forward(m, d)
    bench_top, _roi = PV.bench_geometry(m, d)
    red = np.asarray(d.xpos[m.body('a_payload').id], float)
    fixture = np.asarray(m.body(FIXTURE_BODY).pos, float)
    aim = [0.5 * (red[0] + fixture[0]), 0.5 * (red[1] + fixture[1]), bench_top + CAM_ABOVE_AIM_M]
    pos = [aim[0], aim[1] - CAM_BACK_M, bench_top + CAM_ABOVE_BENCH_M]
    return {'pos': [float(v) for v in pos], 'aim': [float(v) for v in aim],
            'above_bench_m': CAM_ABOVE_BENCH_M, 'back_m': CAM_BACK_M,
            'why': 'the default mount merges a part with the fixture\'s far rib in the segmentation; '
                   'measured margin at this one: all 3 cells plus 4 neighbouring mounts pass'}


def scene(*, cam_pos=None, cam_quat=None):
    """`probe_p3_vision.build_scene()` pointed at the derived P4 world. THE one override."""
    global _LAST_MOUNT
    PV.WORLD = WORLD
    if cam_pos is None:
        mount_spec = mount()
        cam_pos = mount_spec['pos']
        cam_quat = PV.look_at(cam_pos, mount_spec['aim'])
        _LAST_MOUNT = mount_spec
    model, bench_z, blue_pose = PV.build_scene(cam_pos=cam_pos, cam_quat=cam_quat)
    if model.body(FIXTURE_BODY) is None:
        raise SystemExit('REFUSED: the loaded world has no %r, so the override did not take'
                         % FIXTURE_BODY)
    return model, bench_z, blue_pose


def cells(model, data):
    """The fixture's cells, derived from the fixture's own geometry -- never typed, never the tray's."""
    return derive_cells(model, data, FIXTURE_BODY)


def fixture_geoms(model):
    bid = model.body(FIXTURE_BODY).id
    return [g for g in range(model.ngeom) if int(model.geom_bodyid[g]) == bid]


def geom_name(model, gid):
    return mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, gid)


def segmentation_plane(model, bench_z):
    """The height `probe_p3_vision.segment_parts` keeps points above, for THESE parts.

    Copied from the frozen line 431 (`world[..., 2] > bench_z + 0.5 * float(min(g_size))`) so a probe
    can test its own fixture against the plane instead of assuming it clears it. The copy is not
    trusted: the builder refuses to write a world whose floor is above it, and each report states the
    margin in metres.
    """
    g = np.asarray(PV.part_size(model), float)
    return float(bench_z) + 0.5 * float(np.min(g))


def _sha(p):
    return hashlib.sha256(p.read_bytes()).hexdigest()
