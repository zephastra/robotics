"""Tests for `P3-VISION-03`: the part classifier, the bench test, and the probe's own contracts.

The probe carries its own falsifiability contract (12 judged checks, 9 probes that must each make
one go red). These tests cover what that contract cannot: the PURE predicates, the derived
geometry, and the cross-artefact claims. Every one of them would fail if the rule it names were
reverted.

| property under test                                                     | test |
|--------------------------------------------------------------------------|------|
| 灰色不是零件颜色（旧规则会把它读成蓝的）                                     | `test_a_grey_blob_is_not_a_part_colour` |
| 旧规则真的会把那块灰读成蓝 —— 这就是被替换掉的那条回归                          | `test_the_ordering_only_rule_really_did_call_that_grey_blue` |
| 零件颜色按色度分类，不按亮度                                                | `test_the_part_colours_classify_by_chroma_and_not_by_brightness` |
| 台面的棕色也不是零件颜色                                                    | `test_the_bench_colour_is_not_a_part_colour` |
| 落在台面上的 blob 算零件，浮在 0.4 m 高的手不算                                | `test_a_blob_on_the_bench_is_a_part_and_the_hand_is_not` |
| 台面高度是从 geom 派生的，不是打上去的常量                                    | `test_the_bench_top_is_derived_from_the_geom` |
| 第二个零件的位置从间隙派生，不是打上去的                                       | `test_the_second_part_pose_is_derived_from_the_clearance` |
| keyframe 会把后加的自由关节钉在原点 —— `prepare()` 存在的理由                  | `test_the_keyframe_pins_a_later_free_joint_at_the_origin` |
| 只有自由关节能通过 qpos 摆位，否则要显式报错                                   | `test_a_body_without_a_free_joint_cannot_be_placed` |
| 探针不修改被判过的世界                                                      | `test_the_judged_world_is_byte_identical_after_building_the_probe` |
| 可失败性契约的键表与结构性命名单一致                                          | `test_the_check_key_table_matches_the_structural_declaration` |
| 结构性命名的检查都写了理由                                                   | `test_every_structural_check_carries_a_written_reason` |
| 冻结清单里确实有这个探针，且覆盖率没有缺口                                     | `test_the_freeze_freezes_this_probe_and_reports_no_coverage_hole` |
"""
from __future__ import annotations

import hashlib
import json
import pathlib
import sys

import pytest

ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'experiments'))
sys.path.insert(0, str(ROOT / 'src'))

import mujoco  # noqa: E402
import numpy as np  # noqa: E402

import probe_p3_vision as pv  # noqa: E402

_SCENE = {}


def scene():
    """Build the probe's world ONCE. Compiling the five-instance cell is not free."""
    if 'm' not in _SCENE:
        _SCENE['m'], _SCENE['bz'], _SCENE['blue'] = pv.build_scene()
    return _SCENE['m'], _SCENE['bz'], _SCENE['blue']


# ---------------------------------------------------------------------------------------------
# colour
# ---------------------------------------------------------------------------------------------
def test_a_grey_blob_is_not_a_part_colour():
    assert pv.chroma_class(pv.HAND_RGB) == 'grey'
    assert pv.chroma_class(np.array([0.5, 0.5, 0.5])) == 'grey'
    assert pv.chroma_class(np.array([255.0, 255.0, 255.0])) == 'grey'


def test_the_ordering_only_rule_really_did_call_that_grey_blue():
    """The regression the chroma test replaced, asserted as a FACT rather than described.

    `order_only_class` is kept in the probe for exactly this: it is the old rule, and the old rule
    classifies the arm's hand -- measured at RGB [93.5, 93.7, 93.8] -- as a BLUE PART.
    """
    assert pv.order_only_class(pv.HAND_RGB) == 'blue'
    assert pv.chroma_class(pv.HAND_RGB) == 'grey'


def test_the_part_colours_classify_by_chroma_and_not_by_brightness():
    # the two colours the probe actually measured
    assert pv.chroma_class(np.array([152.7, 11.2, 10.5])) == 'red'
    assert pv.chroma_class(np.array([14.1, 12.4, 90.9])) == 'blue'
    # a DARK but saturated colour is still a class colour: the rule is a ratio, not a level
    assert pv.chroma_class(np.array([9.0, 2.0, 2.0])) == 'red'
    assert pv.chroma_class(np.array([2.0, 2.0, 9.0])) == 'blue'
    # just inside / just outside the declared ratio, on the red side
    assert pv.chroma_class(np.array([100.0, 100.0 / pv.CHROMA_RATIO, 0.0])) == 'red'
    assert pv.chroma_class(np.array([100.0, 100.0 / (pv.CHROMA_RATIO * 0.9), 0.0])) == 'grey'


def test_the_bench_colour_is_not_a_part_colour():
    # a_table's geom rgba is [0.55, 0.44, 0.29, 1.0] -> brownish, and it must classify as neither
    bench = np.array([0.55, 0.44, 0.29]) * 255.0
    assert pv.chroma_class(bench) == 'grey'


# ---------------------------------------------------------------------------------------------
# the bench-contact test
# ---------------------------------------------------------------------------------------------
def test_a_blob_on_the_bench_is_a_part_and_the_hand_is_not():
    bench_z = 0.2
    half = 0.025
    # a part standing on the bench has its top at bench_z + 2*half
    assert pv.rests_on_bench(bench_z + 2 * half, bench_z, np.array([0.02, 0.015, half]))
    # the arm's hand, measured 0.4022 m above the bench top
    assert not pv.rests_on_bench(bench_z + 0.4022, bench_z, np.array([0.02, 0.015, half]))
    # and the margin is orders of magnitude, not a nudge
    assert 0.4022 >= 10 * pv.ON_BENCH_TOL


# ---------------------------------------------------------------------------------------------
# derived geometry
# ---------------------------------------------------------------------------------------------
def test_the_bench_top_is_derived_from_the_geom():
    model, bench_z, _blue = scene()
    gid = pv._geom_of_body(model, pv.TABLE)
    d = pv.prepare(model, mujoco.MjData(model))
    independent = float(d.geom_xpos[gid][2] + model.geom_size[gid][2])
    assert abs(bench_z - independent) < 1e-12
    # and it is NOT the value an earlier version typed into a constant
    assert abs(bench_z - 0.24) > 1e-3, 'the typed BENCH_TOP = 0.24 has come back'


def test_the_second_part_pose_is_derived_from_the_clearance():
    model, bench_z, blue_pose = scene()
    d = pv.prepare(model, mujoco.MjData(model))
    g = pv.part_size(model)
    red = pv.pose_of(model, d, pv.RED)
    expected, _g = pv.intended_blue_pose(model, d, bench_z)
    assert abs(expected[1] - (red[1] + 2.0 * g[1] + pv.PART_CLEARANCE)) < 1e-12
    assert abs(expected[2] - (bench_z + g[2])) < 1e-12
    assert np.allclose(blue_pose, expected)
    # face-to-face gap is exactly the declared clearance
    assert abs((blue_pose[1] - g[1]) - (red[1] + g[1]) - pv.PART_CLEARANCE) < 1e-12


# ---------------------------------------------------------------------------------------------
# state: why `prepare` exists at all
# ---------------------------------------------------------------------------------------------
def test_the_keyframe_pins_a_later_free_joint_at_the_origin():
    """`prepare()` is not decoration.

    A `<keyframe>` in the merged world pins EVERY joint, and MuJoCo pads the qpos of a free joint
    added afterwards with ZERO. So applying the keyframe alone parks the second part at the world
    origin, and a `body.pos` write cannot fix it. This test fails if that mechanism ever goes away
    -- which is worth knowing, because the hand-written truth it once hid was 30 mm wrong too.
    """
    model, bench_z, blue_pose = scene()
    d = pv.prepare(model, mujoco.MjData(model))
    parked = pv.pose_of(model, d, pv.BLUE)
    assert np.allclose(parked, 0.0, atol=1e-9), 'the keyframe no longer zeroes the added free joint'
    assert not np.allclose(parked, blue_pose, atol=1e-6)
    # and placing it through the qpos does work, and is asserted to have landed
    d = pv.prepare(model, mujoco.MjData(model), poses={pv.BLUE: blue_pose})
    assert np.allclose(pv.pose_of(model, d, pv.BLUE), blue_pose, atol=1e-9)
    # the bench top is unchanged by the placement
    assert abs(pv.bench_geometry(model, d)[0] - bench_z) < 1e-12


def test_a_body_without_a_free_joint_cannot_be_placed():
    model, _bz, _blue = scene()
    with pytest.raises(SystemExit):
        pv.prepare(model, mujoco.MjData(model), poses={pv.TABLE: [0.0, 0.0, 0.0]})


# ---------------------------------------------------------------------------------------------
# the judged world must not move
# ---------------------------------------------------------------------------------------------
def test_the_judged_world_is_byte_identical_after_building_the_probe():
    before = hashlib.sha256(pv.WORLD.read_bytes()).hexdigest()
    pv.build_scene()
    after = hashlib.sha256(pv.WORLD.read_bytes()).hexdigest()
    assert before == after, 'the probe rewrote the artefact P3-WORLD-01 is judged against'


# ---------------------------------------------------------------------------------------------
# the probe's own contracts, and the freeze
# ---------------------------------------------------------------------------------------------
def test_the_check_key_table_matches_the_structural_declaration():
    assert len(pv.KEYS) == 12
    assert len(set(pv.KEYS.values())) == 12, 'two checks share a name'
    assert set(pv.STRUCTURAL) <= set(pv.KEYS), 'a structural key is not a declared check'
    assert len(pv.KEYS) - len(pv.STRUCTURAL) == 9, 'the falsifiable count changed'


def test_every_structural_check_carries_a_written_reason():
    for key, reason in pv.STRUCTURAL.items():
        assert reason and len(reason) > 40, f'{key} is declared structural with no real reason'


def test_the_freeze_freezes_this_probe_and_reports_no_coverage_hole():
    frozen = json.loads((ROOT / 'config' / 'p3_freeze.json').read_text(encoding='utf-8'))
    paths = {e['path'] for e in frozen['code']}
    assert 'experiments/probe_p3_vision.py' in paths, (
        'the freeze does not hash the only judge for P3-VISION-03; that was D058')
    cov = frozen['code_coverage']
    assert cov['missing_from_code'] == [], (
        f'a P3 row names a script the freeze does not cover: {cov["missing_from_code"]}')
    assert cov['exempt_but_no_longer_named'] == []
    assert 'p3_vision.LOC_TOL' in {t['name'] for t in frozen['thresholds']}
