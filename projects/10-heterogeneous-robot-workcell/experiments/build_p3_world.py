"""Build the P3 formal cell: the V1 configuration, with TWO AMR instances.

WHY THIS FILE IS THIN
---------------------
`merge_world.py` already owns the hard parts -- station derivation from measured footprints,
prefixing, mesh-path rebasing, keyframe rebuilding, the stepping and kinematics verification.
Re-implementing any of that here would be a second opinion on machinery that is already checked.

So this module does exactly one thing: it hands `merge_world` a five-role table instead of four
and a different output path, then lets the existing builder run. Everything the P1 gate proved
about the assembly (names follow ownership, no cross-instance leakage, one time base, the scene
settles) then applies to the P3 cell by construction rather than by assertion.

    h   humanoid + V1 tray
    a   fixed arm + part + table
    c   the transfer fixture (fixed roller sections + receiving row + tray) at its station
    n   AMR INSTANCE 1 -- carries C's deck (the P1 assembly, unchanged)
    n2  AMR INSTANCE 2 -- a second, identical chassis

WHAT IS DELIBERATELY *NOT* DONE HERE, AND WHY
---------------------------------------------
**Instance 2 has no deck.** V1's configuration says "two AMRs with short conveyor decks", and
that is what the cell should eventually look like. This build gives the deck to instance 1 only.

That is a scope decision, not an oversight, and it is recorded rather than left to be discovered:

  * P3's exit gate (`MASTER_PLAN` section 5) is "single-vehicle localisation and navigation,
    coordinate/time/sensor calibration, and the LOADED docking measurement". One decked vehicle
    is what that gate needs.
  * The second deck is what instance 2 needs to *carry* anything, which belongs with the
    two-order work (P6), not with the world/nav/vision work (P3).
  * Duplicating the deck is a real change -- `decompose_c_xml` splits C into a vehicle piece and
    a world piece, and a second deck means deciding whether the fixture is shared or duplicated
    too. That decision has no evidence behind it yet.

What P3 needs from the second instance is **independence**: its own names, its own frames, its own
command path, and no coupling to instance 1. A bare chassis exercises all of that.

STATIONS
--------
`station_layout` derives each station from the role's own measured footprint plus a declared
clearance, so adding a role widens the corridor automatically. The two AMRs therefore end up at
two separate stations, not in one shared driving area -- which is right for an isolation test and
is NOT the final cell layout. Recorded.
"""
from __future__ import annotations

import argparse
import pathlib
import sys

ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'experiments'))
sys.path.insert(0, str(ROOT / 'src'))

import merge_world as mw  # noqa: E402

#: The P3 output. A DIFFERENT file from the P1 gate's world -- the P1 gate's evidence must keep
#: pointing at the artefact it judged, so P3 does not overwrite `world_p1_cell.xml`.
P3_TARGET = mw.ASSETS / 'world_p3_cell.xml'

#: N's own arena furniture, reused for instance 2 so both AMRs are built from one source.
_N_FURNITURE = tuple(next(r.furniture for r in mw.ROLES if r.key == 'n'))

P3_ROLES = tuple(mw.ROLES) + (
    mw.Role('n2', 'second diff-drive chassis (no deck -- see the module docstring)',
            'assets/worlds/world_n_probe.xml', furniture=_N_FURNITURE),
)


def install():
    """Hand `merge_world` the P3 role table and target. Idempotent."""
    mw.ROLES = P3_ROLES
    mw.TARGET = P3_TARGET
    return mw


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--check', action='store_true',
                    help='compare with what is on disk; write nothing')
    ap.add_argument('--layout', action='store_true')
    args = ap.parse_args()

    install()
    if args.layout:
        for k, v in mw.station_layout().items():
            print(f'  {k}: {v}')
        print(f'  bounds: {mw.cell_bounds(mw.station_layout())}')
        return 0
    return mw.build(check=args.check)


if __name__ == '__main__':
    raise SystemExit(main())
