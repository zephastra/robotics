#!/usr/bin/env python3
"""P4-ARM-02b/02c prerequisite: the tray's cells, derived and judged.

WHY THIS IS A SEPARATE PROBE
Both remaining pieces of `P4-ARM-02` need the same thing: a way to name a tray cell.
`02b` needs it as a PLACE TARGET; `02c` needs it as a READ ("which cell is this part in"). Neither
can hard-code it, because the tray moves -- in the H3 chain the tray is carried from the source band
to the deck, and `P4-BELT-03` will move it again.

THE FIRST ANSWER WAS WRONG AND IS ON THE RECORD
The first derivation classified boundaries by the X axis and produced three "cells" of widths
4 mm / 122 mm / 4 mm. Those 4 mm slivers are the tray's END WALLS, not cells. The partitions are
thin in Y and run along X, so the tray is divided in Y. Both attempts return "3 cells", which is why
the wrong one looked right -- so the row that checks the axis is not decoration.

Rows carry FALSIFIABILITY: row `the_derivation_reads_the_geometry` perturbs the partition geoms and
requires the cell count to collapse to 1. A derivation that returned 3 regardless would fail it.
"""
import argparse
import copy
import hashlib
import json
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src'))
sys.path.insert(0, str(ROOT / 'experiments'))

import mujoco  # noqa: E402
import numpy as np  # noqa: E402

from workcell.tray import (derive_cells, cell_of, cell_centre_world,  # noqa: E402
                           boundaries_along)

#: The worlds the tray appears in. Deriving the SAME cells from both is itself a check: a property of
#: the tray must not depend on which file it was read from.
WORLDS = ('assets/world_w5_h085_loop.xml', 'assets/world_p3_cell.xml')
TRAY = 'c_payload'
#: The part used for the place/read test. A box (`a_payload`), because a cylinder rolls and the row
#: would then be measuring the settle, not the derivation.
PART = 'a_payload'
#: How many steps to let a placed part settle before reading where it is.
SETTLE_STEPS = 400


def load(world):
    model = mujoco.MjModel.from_xml_path(str(ROOT / world))
    data = mujoco.MjData(model)
    data.qpos[:] = np.array(model.key_qpos[0], dtype=float)
    mujoco.mj_forward(model, data)
    return model, data


def free_qadr(model, body):
    bid = model.body(body).id
    jid = int(model.body_jntadr[bid])
    if int(model.jnt_type[jid]) != int(mujoco.mjtJoint.mjJNT_FREE):
        raise SystemExit('REFUSED: %s is not free-jointed' % body)
    return int(model.jnt_qposadr[jid]), int(model.jnt_dofadr[jid])


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--run-id', default='p4-cells-01')
    args = ap.parse_args(argv)

    out = ROOT / 'reports' / args.run_id
    if out.exists() and any(out.iterdir()):
        raise SystemExit('REFUSED: %s already holds evidence; pick a new --run-id' % out)
    out.mkdir(parents=True, exist_ok=True)

    started = time.monotonic()
    report = {'probe': 'P4-ARM-02b/02c prerequisite: tray cells derived from the tray geometry',
              'worlds': {}, 'status': 'ERROR'}

    derived = {}
    for w in WORLDS:
        model, data = load(w)
        cells = derive_cells(model, data, TRAY)
        derived[w] = cells
        report['worlds'][w] = {
            'sha256': hashlib.sha256((ROOT / w).read_bytes()).hexdigest(),
            'deck': {'centre': [float(v) for v in cells['deck']['centre']],
                     'size': [float(v) for v in cells['deck']['size']],
                     'top_z': float(cells['deck']['top_z'])},
            'divided_along': 'xy'[cells['axis']],
            'partitions': [float(v) for v in cells['partitions']],
            'walls': [float(v) for v in cells['walls']],
            'cells': [{'index': c['index'], 'lo': float(c['lo']), 'hi': float(c['hi']),
                       'centre': float(c['centre']), 'width': float(c['width'])}
                      for c in cells['cells']],
            'tiles_the_deck': bool(cells['tiles_the_deck']),
        }

    # -- falsifiability: flatten ONLY the interior partitions; the cell count must collapse to 1 --
    model, data = load(WORLDS[0])
    cells_before = derive_cells(model, data, TRAY)
    flat = copy.deepcopy(model)
    axis = cells_before['axis']
    other = 1 - axis
    # Only the INTERIOR partitions are flattened -- the tray's outer walls are left standing, so the
    # deck still exists and still has edges. If the derivation merely counted walls it would still
    # report 3; it must report 1.
    parts, walls = boundaries_along(model, data, axis, TRAY)
    partition_gids = [int(gid) for _rel, gid in parts]
    for g in partition_gids:
        flat.geom_size[g][other] = 0.0
    cells_flat = derive_cells(flat, data, TRAY)
    report['falsifiability'] = {
        'axis': 'xy'[axis],
        'partition_geoms_flattened': partition_gids,
        'wall_geoms_left_standing': [int(gid) for _rel, gid in walls],
        'cells_before': len(cells_before['cells']),
        'cells_after': len(cells_flat['cells']),
        'partition_offsets': [float(r) for r, _g in parts],
    }

    # -- the read that 02c will need: place a part in each cell, settle, read it back ----------
    model, data = load(WORLDS[0])
    cells = derive_cells(model, data, TRAY)
    qadr, dofadr = free_qadr(model, PART)
    part_half_z = float(model.geom_size[
        next(g for g in range(model.ngeom)
             if int(model.geom_bodyid[g]) == model.body(PART).id)][2])
    reads = []
    for c in cells['cells']:
        tgt = cell_centre_world(cells, c['index'], z=float(cells['deck']['top_z']) + part_half_z)
        data.qpos[qadr:qadr + 3] = tgt
        data.qpos[qadr + 3:qadr + 7] = [1.0, 0.0, 0.0, 0.0]
        data.qvel[dofadr:dofadr + 6] = 0.0
        mujoco.mj_forward(model, data)
        for _ in range(SETTLE_STEPS):
            mujoco.mj_step(model, data)
        where = np.asarray(data.xpos[model.body(PART).id], float)
        reads.append({'placed_in_cell': c['index'], 'target': [float(v) for v in tgt],
                      'settled_at': [float(v) for v in where],
                      'read_back_cell': cell_of(cells, where),
                      'offset_m': float(np.linalg.norm(where[:2] - np.asarray(tgt)[:2]))})
    report['cell_reads'] = reads

    # -- rows -----------------------------------------------------------------------------------
    counts = {w: len(report['worlds'][w]['cells']) for w in WORLDS}
    axes = {w: report['worlds'][w]['divided_along'] for w in WORLDS}
    rows = []

    def add(name, ok, detail, falsified_by=None):
        rows.append({'name': name, 'status': 'PASS' if ok else 'FAIL', 'detail': detail,
                     'falsified_by': falsified_by})

    add('cells_are_derived_from_the_tray_geometry',
        all(counts[w] >= 2 for w in WORLDS),
        'the same derivation run on two different worlds gives %s cells (per world: %s), so the '
        'structure is a property of the TRAY and not of one file. Partitions at y %s relative to the '
        'deck centre, found from the geoms themselves'
        % (sorted(set(counts.values())), counts,
           [round(v, 5) for v in report['worlds'][WORLDS[0]]['partitions']]),
        falsified_by='flattening the partitions collapses the count to 1, which the next row shows')

    add('the_cell_count_matches_the_declared_layout',
        all(counts[w] == 3 for w in WORLDS),
        'MASTER_PLAN section 1 gives the V1 tray "3 个有隔挡的料位"; the derivation finds exactly 3 '
        'in every world that has this tray (%s)' % counts,
        falsified_by='a derivation that always answered 3 would also pass this row, which is why the '
                     'axis is checked separately')

    add('the_tray_is_divided_along_the_axis_the_geometry_says',
        all(axes[w] == 'y' for w in WORLDS),
        'the division is along %s. The FIRST attempt used x and produced three "cells" of widths '
        '4 mm / 122 mm / 4 mm -- those slivers are the tray END WALLS. Partitions are thin in y and '
        'run along x, so x was wrong while still yielding a plausible count of 3'
        % axes,
        falsified_by='the 4 mm widths are the detectable symptom of the x-axis misreading')

    add('cells_tile_the_deck_without_gap_or_overlap',
        all(report['worlds'][w]['tiles_the_deck'] for w in WORLDS),
        'the derived widths sum to the deck span exactly in every world (%s), so no region of the '
        'deck belongs to two cells or to none'
        % {w: [round(c['width'], 5) for c in report['worlds'][w]['cells']] for w in WORLDS},
        falsified_by='a gap or an overlap makes the sum differ and the derivation raises')

    add('the_derivation_reads_the_geometry', report['falsifiability']['cells_after'] == 1,
        '[FALSIFIABILITY] flattening ONLY the %d INTERIOR partitions at y %s (the %d outer walls '
        'are left standing, so the deck still has edges) collapses the cell count from %d to %d. '
        'So the derivation reads the geometry rather than returning a constant; a function that '
        'always answered 3 would fail this row'
        % (len(report['falsifiability']['partition_geoms_flattened']),
           [round(v, 5) for v in report['falsifiability']['partition_offsets']],
           len(report['falsifiability']['wall_geoms_left_standing']),
           report['falsifiability']['cells_before'], report['falsifiability']['cells_after']),
        falsified_by=None)

    read_ok = all(r['read_back_cell'] == r['placed_in_cell'] for r in reads)
    add('a_part_placed_in_a_cell_is_read_back_in_that_cell', read_ok,
        'a part placed at each cell centre and allowed to settle is read back into the cell it was '
        'placed in, at every one of the %d cells (offsets %s m). This is the read the count '
        'verification needs: "which cell is this part in" answered from the part\'s own position'
        % (len(reads), [round(r['offset_m'], 5) for r in reads]),
        falsified_by='the settle offsets are small and the cells are 149-153 mm wide, so a part read '
                     'back as the wrong cell would mean the derivation and the physics disagree')

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

    for w in WORLDS:
        print('%s  divided along %s  cells %s'
              % (w, report['worlds'][w]['divided_along'],
                 [round(c['width'], 4) for c in report['worlds'][w]['cells']]))
    print('falsifiability: cells %d -> %d after flattening %d interior partitions (walls kept: %d)'
          % (report['falsifiability']['cells_before'], report['falsifiability']['cells_after'],
             len(report['falsifiability']['partition_geoms_flattened']),
             len(report['falsifiability']['wall_geoms_left_standing'])))
    print('cell reads: %s' % [(r['placed_in_cell'], r['read_back_cell']) for r in reads])
    print()
    for r in rows:
        print('%-4s [%-4s] %-52s %s' % ('ok' if r['status'] == 'PASS' else 'FAIL',
                                        r['status'], r['name'], r['detail'][:110]))
    print()
    print(report['verdict'])
    print('report -> %s' % (out / 'report.json'))
    return 0 if report['counts']['fail'] == 0 else 1


if __name__ == '__main__':
    raise SystemExit(main())
