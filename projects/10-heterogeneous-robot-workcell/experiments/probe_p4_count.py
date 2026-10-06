#!/usr/bin/env python3
"""P4-ARM-02c: kind / count / cell verification, with UNKNOWN.

`docs/MASTER_PLAN.md` section 2 step 5, verbatim:

    检查种类/数量/料位；看不清返回 UNKNOWN，**不读取订单反推实际数量**。

The clause doing the work is the last one. A "verification" that compares the order against itself
always agrees; the thing worth building is a reader whose answer can DISAGREE with the order, and a
row that shows it disagreeing.

WHAT THIS READS
`probe_p4_vision.decide()` gives the parts it can see, each with a class and a position. Each
detected part is assigned to a cell using `workcell.tray.cell_of` -- the DERIVED cell table -- and
the result is an occupancy map. Nothing in that path consults the order, and a row below proves it
by making the order wrong.

WHICH FIXTURE IT READS, AND WHY THAT CHANGED
The first version read the tray, and could not: the tray plates over its own cells and its floor
sits above the segmenter's plane (`reports/p4-count-01`, both measured). `D115` answers that with
an OPEN loading fixture inside the arm's reach, authored in a derived world. So this probe now
reads the fixture, and two workarounds are gone: no in-memory geometry edit, no tray teleport.

PLACEMENT IS NOT THIS PROBE'S SUBJECT
The part is put into its cell KINEMATICALLY here. `P4-ARM-02b` (`reports/p4-arm-01`) is what places
it with the arm; mixing the two would leave the failure mode "the count is wrong" with two possible
causes. One probe, one subject.

DECLARED, NOT FROZEN.
"""
import argparse
import json
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src'))
sys.path.insert(0, str(ROOT / 'experiments'))

import mujoco  # noqa: E402
import numpy as np  # noqa: E402

import probe_p3_vision as PV  # noqa: E402
from probe_p4_vision import decide  # noqa: E402
import p4_fixture as FIX  # noqa: E402
from workcell.tray import cell_of, cell_centre_world  # noqa: E402

#: The fixture is AUTHORED IN THE WORLD (`experiments/build_p4_cell_world.py`, `D115`), so this
#: probe neither moves the tray nor edits geometry in memory. Both of those used to happen here.
ARM_ORIGIN_BODY = 'a_link0'

RED, BLUE = PV.RED, PV.BLUE
#: The order the verification is measured against. DELIBERATELY WRONG for the first check: the
#: point is that the reader can report a mismatch instead of agreeing.
ORDER_AS_CLAIMED = {'red': 2, 'blue': 1}
SETTLE_STEPS = 500


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--run-id', default='p4-count-02')
    args = ap.parse_args(argv)

    out = ROOT / 'reports' / args.run_id
    if out.exists() and any(out.iterdir()):
        raise SystemExit('REFUSED: %s already holds evidence; pick a new --run-id' % out)
    out.mkdir(parents=True, exist_ok=True)

    started = time.monotonic()
    report = {'probe': 'P4-ARM-02c: kind / count / cell verification that can disagree with the '
                       'order and can say UNKNOWN',
              'order_claimed': dict(ORDER_AS_CLAIMED), 'status': 'ERROR'}

    # -- the DERIVED P4 world: the frozen common world + the OPEN loading fixture -----------------
    # The two workarounds this probe used to carry are GONE, and their absence is the evidence:
    #   * it used to shrink `c_visual_marker` in memory because the tray plates over its own cells --
    #     the fixture has no lid, so no geometry is edited at all;
    #   * it used to teleport `c_payload` into reach -- the fixture is authored where it belongs.
    model, bench_z, blue_pose = FIX.scene()
    data = mujoco.MjData(model)
    PV.prepare(model, data, poses={PV.BLUE: blue_pose})
    if model.nkey > 0 and model.key_ctrl.shape[0] > 0:
        data.ctrl[:] = model.key_ctrl[0]     # or the arm's servos drag it to zero (see p4-arm-01)
    report['world'] = FIX.world_provenance()
    report['in_memory_geometry_edits'] = 0

    origin = np.asarray(data.xpos[model.body(ARM_ORIGIN_BODY).id], float)

    # -- the fixture's cells, derived from the fixture's own geometry ---------------------------------
    cells = FIX.cells(model, data)
    deck_top = float(cells['deck']['top_z'])
    report['fixture'] = {
        'placement': 'AUTHORED in %s by experiments/build_p4_cell_world.py; the probe teleports '
                     'nothing and edits no geometry to create it' % FIX.WORLD.name,
        'origin_world': [float(v) for v in data.xpos[model.body(FIX.FIXTURE_BODY).id]],
        'floor_top_z': deck_top,
        'divided_along': 'xy'[cells['axis']],
        'segmentation_plane_z': FIX.segmentation_plane(model, bench_z),
        'cells': [{'index': c['index'], 'centre': float(c['centre']), 'width': float(c['width'])}
                  for c in cells['cells']],
    }
    report['fixture']['floor_margin_below_plane_m'] = (float(report['fixture']['segmentation_plane_z'])
                                                       - deck_top)

    view = PV.View(model)
    depth0 = np.asarray(view.capture(data, 'depth'), float)
    seg0 = view.capture(data, 'seg')
    g_part = np.asarray(PV.part_size(model), float)
    _e, conv, _r = PV.calibrate(model, data, depth0, seg0, bench_z, g_part,
                                PV.pose_of(model, data, RED))

    #: The part's OWN start pose, read from the model's keyframe: the world's own definition of
    #: "not in the fixture". Case 2 puts it back there and the read must stop counting it.
    red_home = np.asarray(PV.pose_of(model, data, PV.RED), float)
    rq, rd = _free_addr(model, RED)
    bq, bd = _free_addr(model, BLUE)

    def park_part(qadr, dof, xyz):
        data.qpos[qadr:qadr + 3] = xyz
        data.qpos[qadr + 3:qadr + 7] = [1.0, 0.0, 0.0, 0.0]
        data.qvel[dof:dof + 6] = 0.0
        mujoco.mj_forward(model, data)

    def read_occupancy():
        """Depth + colour -> ({cell: {class: count}}, unknowns, raw components).

        The order is NOT an argument. That is the whole point, and the row that makes the order
        wrong is what shows it.
        """
        rgb = view.capture(data, 'rgb')
        dep = np.asarray(view.capture(data, 'depth'), float)
        seg_img = view.capture(data, 'seg')
        per = PV.analyze(model, data, rgb, dep, bench_z, g_part, conv)
        dec, why = decide(per, g_part)
        occ, unknown = {}, []
        for cls in ('red', 'blue'):
            if dec.get(cls) is None:
                unknown.append({'class': cls, 'why': why.get(cls)})
                continue
            idx = cell_of(cells, np.asarray(dec[cls], float))
            if idx is None:
                unknown.append({'class': cls,
                                'why': 'UNKNOWN: the part is not inside any derived cell'})
                continue
            occ.setdefault(idx, {})
            occ[idx][cls] = occ[idx].get(cls, 0) + 1
        comps = []
        for c in per['comps']:
            ids = seg_img[[y for y, x in c['pixels']], [x for y, x in c['pixels']]]
            uniq, cnt = np.unique(ids, return_counts=True)
            top = int(uniq[int(np.argmax(cnt))])
            comps.append({'n_pixels': int(c['n_pixels']), 'cls': c['cls'],
                          'on_bench': bool(c['on_bench']),
                          'dominant_geom_id': top, 'dominant_geom': FIX.geom_name(model, top)})
        return occ, unknown, comps

    def totals(occ):
        out = {}
        for cell_map in occ.values():
            for cls, n in cell_map.items():
                out[cls] = out.get(cls, 0) + n
        return out

    # -- case 1: one red part in cell 1 --------------------------------------------------------
    c1 = cells['cells'][1]
    park_part(rq, rd, cell_centre_world(cells, 1, z=deck_top + float(g_part[2])))
    for _ in range(SETTLE_STEPS):
        mujoco.mj_step(model, data)
    occ1, unk1, comps1 = read_occupancy()
    t1 = totals(occ1)
    report['case_one_part'] = {'occupancy': {str(k): v for k, v in occ1.items()},
                               'totals': t1, 'unknown': unk1,
                               'part_world': [float(v) for v in data.xpos[model.body(RED).id]],
                               'read_back_cell': cell_of(
                                   cells, np.asarray(data.xpos[model.body(RED).id], float))}

    # the order claims 2 red + 1 blue; does the reader report the MISMATCH, or agree with it?
    mismatch = any(t1.get(cls, 0) != n for cls, n in ORDER_AS_CLAIMED.items())
    report['order_check'] = {'claimed': dict(ORDER_AS_CLAIMED), 'verified': t1,
                             'reported_mismatch': bool(mismatch)}

    # -- case 2 (falsifiability): take the part OUT of the fixture; the count must change ---------
    # ★ THE FIRST INSTRUMENT WAS WRONG, AND IT IS RECORDED HERE. It lifted the part 0.30 m and then
    # ran 500 settle steps -- and the part simply FELL BACK into the cell: measured, the verified
    # count stayed at 1. "Lift and let it settle" cannot take anything out of a cell. The part is
    # returned to its own start pose instead, which is a state the WORLD defines (the keyframe), and
    # it stays there because the bench is under it.
    park_part(rq, rd, red_home)
    for _ in range(SETTLE_STEPS):
        mujoco.mj_step(model, data)
    occ2, unk2, comps2 = read_occupancy()
    t2 = totals(occ2)
    report['case_out_of_the_fixture'] = {'occupancy': {str(k): v for k, v in occ2.items()},
                                         'totals': t2, 'unknown': unk2,
                                         'parked_at': [float(v) for v in red_home]}

    # -- case 3: a part in a DIFFERENT cell is reported in that cell ---------------------------
    park_part(rq, rd, cell_centre_world(cells, 2, z=deck_top + float(g_part[2])))
    for _ in range(SETTLE_STEPS):
        mujoco.mj_step(model, data)
    occ3, unk3, comps3 = read_occupancy()
    report['case_other_cell'] = {'occupancy': {str(k): v for k, v in occ3.items()},
                                 'totals': totals(occ3), 'unknown': unk3}

    # -- rows ---------------------------------------------------------------------------------
    rows = []

    def add(name, ok, detail, falsified_by=None):
        rows.append({'name': name, 'status': 'PASS' if ok else 'FAIL', 'detail': detail,
                     'falsified_by': falsified_by})

    read_cell = report['case_one_part']['read_back_cell']
    add('the_occupancy_map_is_read_not_remembered',
        read_cell == 1 and occ1.get(1, {}).get('red') == 1,
        'a part parked in cell 1 is read back as being in cell 1, with one red part counted there '
        '(occupancy %s, totals %s). The map is built from the image plus the DERIVED cell table '
        '(%d cells); no placement record is consulted'
        % ({str(k): v for k, v in occ1.items()}, t1, len(cells['cells'])),
        falsified_by='the next row removes the part and the count changes')

    add('the_order_does_not_decide_the_count',
        mismatch and not all(t1.get(c, 0) == n for c, n in ORDER_AS_CLAIMED.items()),
        'the order claims %s; the reader reports %s and therefore a MISMATCH. A verifier that '
        'compared the order against itself would agree by construction -- this one disagrees, so '
        'it is reading something. MASTER_PLAN step 5 forbids exactly the other behaviour: '
        '\u4e0d\u8bfb\u53d6\u8ba2\u5355\u53cd\u63a8\u5b9e\u9645\u6570\u91cf'
        % (ORDER_AS_CLAIMED, t1),
        falsified_by='making the order match the truth would remove the mismatch, so the row would '
                     'fail -- it is measuring agreement, not existence')

    add('taking_the_part_out_of_the_fixture_changes_the_verified_count',
        t2.get('red', 0) == 0 and t1.get('red', 0) == 1,
        '[FALSIFIABILITY] putting the part back at its own start pose on the bench takes the verified '
        'red count from %d to %d, and the reader reports it as %s. So the count follows the world and '
        'cannot be a constant, a copy of the order, or a record of where the part was PUT. The first '
        'instrument here lifted the part 0.30 m instead and settled it -- it fell back into the cell '
        'and the count stayed at %d, which is why the instrument was replaced rather than the row'
        % (t1.get('red', 0), t2.get('red', 0), unk2[0]['why'] if unk2 else 'no unknown recorded',
           t1.get('red', 0)),
        falsified_by='the reader returning the cell it was told to place into would keep reporting 1')

    add('a_part_in_another_cell_is_reported_in_that_cell',
        occ3.get(2, {}).get('red') == 1 and occ3.get(1, {}).get('red') is None,
        'moving the part to cell 2 reports it in cell 2 and NOT in cell 1 (occupancy %s), so the '
        'cell assignment is positional rather than sticky'
        % {str(k): v for k, v in occ3.items()},
        falsified_by='a sticky assignment would keep reporting cell 1')

    add('a_part_outside_the_cells_is_unknown_never_a_silent_zero',
        bool(unk2) and t2.get('red', 0) == 0
        and any('not inside any derived cell' in (u.get('why') or '') for u in unk2),
        'the part is put where the reader cannot place it, and the reader says so instead of counting '
        'zero: %s. Note WHY this proves the part was SEEN: that message is only reachable after '
        '`decide()` resolved the class, so the reader had a position and still refused to put it in a '
        'cell. A silent zero would leave this list empty, and "no part there" and "could not place '
        'what I saw" would become the same record -- which is the failure this row exists to forbid'
        % [u['why'] for u in unk2],
        falsified_by='the in-cell cases below return a count for the same class instead of an UNKNOWN')

    report['components_at_the_reading'] = comps3
    fixtures = set(FIX.fixture_geoms(model))
    fixture_comps = [c for c in comps1 if c['dominant_geom_id'] in fixtures]
    misread = [c for c in fixture_comps if c['cls'] in ('red', 'blue')]
    report['fixture_components'] = fixture_comps

    add('the_fixture_floor_is_below_the_segmentation_plane',
        deck_top < float(report['fixture']['segmentation_plane_z']),
        'the fixture floor top is %.5f and `segment_parts` keeps only points above '
        'bench_z + 0.5*min(part) = %.5f, so the floor is %.5f m below the plane and is NOT in the '
        'mask. That is why a part standing in a cell is its own component: the tray\'s own floor sits '
        '2.5 mm ABOVE the same plane and a part on it 4-connects to the floor (measured in '
        'reports/p4-count-01: one 3772 px blob whose mean colour is the tray\'s)'
        % (deck_top, report['fixture']['segmentation_plane_z'],
           report['fixture']['floor_margin_below_plane_m']),
        falsified_by='the tray in the same world, whose floor is above the plane, yields that blob')

    add('the_fixture_is_never_mistaken_for_a_part',
        bool(fixture_comps) and not misread,
        'the fixture\'s walls and ribs stand above the plane and ARE in the mask, so this row is not '
        'vacuous: %s. All are classified %s, never a part candidate -- grey by construction, since '
        '`chroma_class` needs a 1.5x channel ratio'
        % ([{'geom': c['dominant_geom'], 'n_pixels': c['n_pixels'], 'cls': c['cls']}
            for c in fixture_comps], sorted({c['cls'] for c in fixture_comps})),
        falsified_by='painting a wall the part\'s colour would put it in `decide()`\'s candidate list')

    report['rows'] = rows
    report['counts'] = {'pass': sum(1 for r in rows if r['status'] == 'PASS'),
                        'fail': sum(1 for r in rows if r['status'] == 'FAIL')}
    report['falsifiable_rows'] = sum(1 for r in rows if r['falsified_by'] is not None)
    report['verdict'] = ('PASS: all %d judged rows' % len(rows)
                         if report['counts']['fail'] == 0
                         else 'FAIL: %d of %d judged rows'
                              % (report['counts']['fail'], len(rows)))
    report['wall_s'] = time.monotonic() - started
    report['status'] = 'DIAGNOSTIC_COMPLETED'
    (out / 'report.json').write_text(json.dumps(report, indent=2) + '\n', encoding='utf-8')

    print('order claimed %s | verified %s | mismatch reported: %s'
          % (ORDER_AS_CLAIMED, t1, report['order_check']['reported_mismatch']))
    print('one part  -> occupancy %s' % {str(k): v for k, v in occ1.items()})
    print('out of fx -> occupancy %s  unknown %s' % ({str(k): v for k, v in occ2.items()},
                                                      [u['why'] for u in unk2]))
    print('other cell-> occupancy %s' % {str(k): v for k, v in occ3.items()})
    print()
    for r in rows:
        print('%-4s [%-4s] %-52s %s' % ('ok' if r['status'] == 'PASS' else 'FAIL',
                                        r['status'], r['name'], r['detail'][:110]))
    print()
    print(report['verdict'])
    print('report -> %s' % (out / 'report.json'))
    return 0 if report['counts']['fail'] == 0 else 1


def _free_addr(model, body):
    bid = model.body(body).id
    jid = int(model.body_jntadr[bid])
    if int(model.jnt_type[jid]) != int(mujoco.mjtJoint.mjJNT_FREE):
        raise SystemExit('REFUSED: %s is not free-jointed' % body)
    return int(model.jnt_qposadr[jid]), int(model.jnt_dofadr[jid])


if __name__ == '__main__':
    raise SystemExit(main())
