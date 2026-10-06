"""Tray cells, derived from the tray's own geometry.

`docs/MASTER_PLAN.md` section 1 gives the V1 tray "3 个有隔挡的料位" -- three baffled cells. The
cells are not named in any world: recon (2026-09-29) found the only `cell_*` bodies are the ROOM's
walls and floor (`cell_ground`, `cell_wall_east|north|south|west`). So a cell target has to be
DERIVED, and this module derives it.

WHY DERIVE RATHER THAN TYPE
A typed cell centre is correct until the tray moves, and this project has recorded the same failure
four times (`P3-VISION-03`'s `BENCH_TOP = 0.24` against a derived 0.2000; the H2 `ROLLER_HALF_LENGTH`
scale; the plant's non-home derivation; the frozen `pusher_position` name). A derived interval
cannot go stale, and deriving it twice from two different worlds is itself a check.

THE RULE, AND A FIRST ATTEMPT THAT WAS WRONG
For each horizontal axis, take the geoms that are THIN along that axis, EXTENDED along the other
and TALLER than the deck. Those are the tray's walls and partitions. Along an axis, the two
OUTERMOST are the tray's walls; any remaining ones are PARTITIONS, and they cut the deck into cells.

The first attempt classified by the X axis only, and produced three "cells" of widths 4 mm, 122 mm
and 4 mm -- because the geoms thin in x are the tray's END WALLS, not partitions. The partitions are
thin in Y and run along x, so the tray is divided in Y. That is recorded here because the wrong
answer looked plausible: a 3-cell result came out either way.
"""
import numpy as np

#: A geom is "thin" along an axis at or below this half-extent.
THIN_HALF_M = 0.010
#: and "extended" along the other at or above this one.
EXTENDED_HALF_M = 0.030
#: and must stand above the deck, so the base plate is never mistaken for a partition.
TALL_HALF_M = 0.010
#: How far inside the deck a boundary must sit to count as an interior partition rather than a wall.
INSIDE_MARGIN_M = 0.002


def _tray_geoms(model, data, body):
    bid = model.body(body).id
    return [(g, np.asarray(model.geom_size[g], float), np.asarray(data.geom_xpos[g], float))
            for g in range(model.ngeom) if int(model.geom_bodyid[g]) == bid]


def deck(model, data, body='c_payload'):
    """The tray's deck plate: the widest, flattest geom. Returns its world centre and half-extents."""
    geoms = _tray_geoms(model, data, body)
    if not geoms:
        raise ValueError('body %r has no geoms' % body)
    gid, size, centre = max(geoms, key=lambda t: (t[1][0] * t[1][1], -t[1][2]))
    return {'geom': gid, 'centre': centre, 'size': size,
            'x_span': (centre[0] - size[0], centre[0] + size[0]),
            'y_span': (centre[1] - size[1], centre[1] + size[1]),
            'top_z': centre[2] + size[2]}


def boundaries_along(model, data, axis, body='c_payload'):
    """Offsets (relative to the deck centre) of every wall/partition crossing `axis`.

    `axis` is 0 for x, 1 for y. Walls are the two outermost; anything between them is a partition.
    """
    d = deck(model, data, body)
    other = 1 - axis
    centre = d['centre']
    hits = []
    for gid, size, c in _tray_geoms(model, data, body):
        if size[axis] > THIN_HALF_M:
            continue                      # not thin along this axis: cannot be a wall/partition
        if size[other] < EXTENDED_HALF_M:
            continue                      # not extended: a post, not a wall
        if size[2] < TALL_HALF_M:
            continue                      # flat on the deck: part of the plate
        rel = float(c[axis] - centre[axis])
        if abs(rel) < 1e-9:
            continue                      # centred: the deck's own middle, not a boundary
        hits.append((rel, gid))
    hits.sort()
    # the two outermost on each side are the tray's walls; the rest are interior partitions
    if len(hits) <= 2:
        return [], hits
    lo = [h for h in hits if h[0] < 0]
    hi = [h for h in hits if h[0] > 0]
    walls, parts = [], []
    if lo:
        walls.append(lo[0])
        parts.extend(lo[1:])
    if hi:
        walls.append(hi[-1])
        parts.extend(hi[:-1])
    parts.sort()
    return parts, sorted(walls)


def derive_cells(model, data, body='c_payload'):
    """The tray's cells, as intervals relative to the deck centre, plus how they were found.

    Returns {'axis': 0|1, 'cells': [{'index','lo','hi','centre','width'}...], 'partitions': [...],
    'walls': [...], 'deck': {...}}. `axis` is the axis the tray is DIVIDED along.
    """
    d = deck(model, data, body)
    best = None
    for axis in (0, 1):
        parts, walls = boundaries_along(model, data, axis, body)
        if not parts:
            continue
        half = d['size'][axis]
        edges = [(-half, None)] + [(p[0], p[1]) for p in parts] + [(half, None)]
        cells = []
        for i in range(len(edges) - 1):
            lo, hi = edges[i][0], edges[i + 1][0]
            cells.append({'index': i, 'lo': lo, 'hi': hi, 'centre': (lo + hi) / 2.0,
                          'width': hi - lo})
        cand = {'axis': axis, 'cells': cells, 'partitions': [p for p, _ in parts],
                'walls': [w for w, _ in walls], 'deck': d,
                'deck_half': half, 'whole_span_m': 2.0 * half}
        if best is None or len(cells) > len(best['cells']):
            best = cand
    if best is None:
        # an undivided deck: one cell, and say so rather than inventing structure
        axis = 0
        half = d['size'][axis]
        best = {'axis': axis,
                'cells': [{'index': 0, 'lo': -half, 'hi': half, 'centre': 0.0,
                           'width': 2.0 * half}],
                'partitions': [], 'walls': [w for w, _ in
                                            boundaries_along(model, data, axis, body)[1]],
                'deck': d, 'deck_half': half, 'whole_span_m': 2.0 * half}
    # a sanity check that the derivation actually tiles the deck: no gaps, no overlaps
    span = sum(c['width'] for c in best['cells'])
    if abs(span - best['whole_span_m']) > 1e-9:
        raise ValueError('the derived cells do not tile the deck: %.6f vs %.6f'
                         % (span, best['whole_span_m']))
    best['tiles_the_deck'] = True
    return best


#: How far outside the deck plate a point may still count as "in" a cell. Declared: a part resting
#: on the deck may overhang a little, but a part standing on the BENCH 0.15 m away must not.
FOOTPRINT_TOL_M = 0.010


def cell_of(cells, point):
    """Which cell a world point falls in, or None.

    ★ BOTH horizontal axes are checked, and the first version checked only the divided one. The
    symptom was measured in `reports/p4-count-01`: a part standing on the BENCH beside the tray --
    same y, 0.15 m away in x -- was reported as being in cell 1, because cell 1 was the y-interval
    the point happened to fall in. A cell is a region of the DECK, not a slab of the world.
    """
    axis = cells['axis']
    other = 1 - axis
    centre = cells['deck']['centre']
    half = cells['deck']['size']
    if abs(float(point[other]) - float(centre[other])) > float(half[other]) + FOOTPRINT_TOL_M:
        return None
    rel = float(point[axis]) - float(centre[axis])
    for c in cells['cells']:
        if c['lo'] <= rel <= c['hi']:
            return c['index']
    return None


def cell_centre_world(cells, index, z=None):
    """A world point at the centre of cell `index`, at the deck's top unless `z` is given."""
    c = cells['cells'][index]
    out = list(cells['deck']['centre'])
    out[cells['axis']] = float(cells['deck']['centre'][cells['axis']]) + c['centre']
    out[2] = float(cells['deck']['top_z']) if z is None else float(z)
    return out
