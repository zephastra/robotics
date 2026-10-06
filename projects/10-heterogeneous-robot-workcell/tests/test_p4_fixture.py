"""The P4 fixture world becomes a permanent gate, not a one-off measurement.

Four invariants, each able to fail:
  * the derived world is REPRODUCIBLE (`build_p4_cell_world.py --check`),
  * its cells follow the TRAY's convention (derived twice from two worlds),
  * the fixture floor is below the segmenter's plane -- the fact that makes a part readable at all,
  * the fixture is inside the arm's measured reach, and its floor above the bench.
"""
import subprocess
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src'))
sys.path.insert(0, str(ROOT / 'experiments'))

import mujoco  # noqa: E402
import probe_p3_vision as PV  # noqa: E402
import p4_fixture as FIX  # noqa: E402
from workcell.tray import cell_centre_world, derive_cells  # noqa: E402

#: The IK boundary MEASURED in reports/p4-arm-01: converges at 0.780 m from a_link0, fails at 0.820 m.
#: Cited here so a fixture that drifts out of reach breaks the suite instead of a probe run.
REACH_BOUNDARY_M = 0.78


def _home(path):
    model = mujoco.MjModel.from_xml_path(str(path))
    data = mujoco.MjData(model)
    data.qpos[:] = model.key_qpos[0]
    mujoco.mj_forward(model, data)
    return model, data


def test_the_derived_world_is_reproducible():
    r = subprocess.run(
        [str(ROOT / '.venv' / 'bin' / 'python'),
         str(ROOT / 'experiments' / 'build_p4_cell_world.py'), '--check'],
        capture_output=True, text=True, cwd=str(ROOT))
    assert r.returncode == 0, 'the builder disagrees with the asset on disk:\n%s\n%s' % (r.stdout,
                                                                                         r.stderr)
    assert 'on-disk == rebuilt: True' in r.stdout


def test_the_fixture_cells_follow_the_trays_convention():
    src, src_data = _home(FIX.SOURCE_WORLD)
    tray = derive_cells(src, src_data, 'c_payload')
    p4, p4_data = _home(FIX.WORLD)
    fixture = derive_cells(p4, p4_data, FIX.FIXTURE_BODY)
    assert fixture['axis'] == tray['axis'], 'the two fixtures are divided on different axes'
    assert ([round(float(c['width']), 6) for c in fixture['cells']]
            == [round(float(c['width']), 6) for c in tray['cells']]), \
        'the fixture no longer uses the tray\'s cell layout'


def test_the_fixture_floor_is_below_the_segmentation_plane():
    model, data = _home(FIX.WORLD)
    bench_top, _roi = PV.bench_geometry(model, data)
    plane = FIX.segmentation_plane(model, bench_top)
    floor_top = float(derive_cells(model, data, FIX.FIXTURE_BODY)['deck']['top_z'])
    assert floor_top > bench_top, 'the fixture floor is below the bench, so there is no fixture'
    assert floor_top < plane, (
        'the fixture floor top %.5f is not below the segmenter\'s plane %.5f: a part standing on it '
        'would 4-connect to the floor and its colour would never reach the classifier'
        % (floor_top, plane))


def test_the_fixture_is_inside_the_arms_measured_reach():
    model, data = _home(FIX.WORLD)
    arm = np.asarray(data.xpos[model.body('a_link0').id], float)
    cells = derive_cells(model, data, FIX.FIXTURE_BODY)
    radii = [float(np.hypot(float(cell_centre_world(cells, i)[0]) - arm[0],
                            float(cell_centre_world(cells, i)[1]) - arm[1]))
             for i in range(len(cells['cells']))]
    assert max(radii) < REACH_BOUNDARY_M, (
        'a fixture cell sits %.4f m from a_link0, beyond the measured boundary %.3f' %
        (max(radii), REACH_BOUNDARY_M))


def test_the_declared_mount_still_agrees_with_the_world():
    spec = FIX.mount()
    model, data = _home(FIX.WORLD)
    bench_top, _roi = PV.bench_geometry(model, data)
    assert abs(spec['pos'][2] - (bench_top + spec['above_bench_m'])) < 1e-9
    # the aim is the midpoint of the pick area and the fixture, so it must lie between them
    red = np.asarray(data.xpos[model.body('a_payload').id], float)
    fixture = np.asarray(model.body(FIX.FIXTURE_BODY).pos, float)
    assert min(red[0], fixture[0]) < spec['aim'][0] < max(red[0], fixture[0])
