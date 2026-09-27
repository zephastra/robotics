"""The station cap was MINE. Recompute the feasible crouch depth with the real 3-D constraint.

WHAT WAS WRONG
--------------
`reports/p4-reach-01` capped the station shift at `shift_max = band_first_roller -
(table_east_edge_offset) - base_x0` = 3.4236 m, with the justification "the tray cannot be lowered
past that edge". That is a claim about ONE dimension. Measured in 3-D (`_dbg_p4_clearance.py`):
the table's geom sits at z 0.803 and the band's crowns at z 0.450, so the table OVERHANGS the band
and its clearance is **311.04 mm at every station shift from 3.42 to 3.80** -- it never collides.
What DOES collide is the humanoid's own body: 38 mm clear at shift 3.60, **-10.6 mm at 3.65**.

So the cap is ~3.63 m, not 3.4236 m, and the crouch depth it forces is shallower than the report
claimed. This file recomputes that with the three constraints that actually apply, per depth:

  (i)   the LEG can hold the pelvis at (dx_c, dz) with the feet pinned  -- from `_dbg_p4_legmap`
  (ii)  the ARMS can reach the band's grips from the base's pose       -- from `p4-reach-01`
  (iii) the humanoid's BODY does not intersect the band at that posture -- measured here in 3-D

and station = (the shift the arms need) - (the pelvis offset the legs need), because the arms care
about the BASE while the clearance cares about where the FEET are.
"""
import json
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'experiments'))
sys.path.insert(0, str(ROOT / 'src'))

import mujoco  # noqa: E402
import build_p3_world as bp3  # noqa: E402

bp3.install()
import merge_world as mw  # noqa: E402
from probe_p4_crouch import Body  # noqa: E402
from _dbg_p4_legs import LEG_CHAIN, FOOT  # noqa: E402
import _dbg_p4_legmap as LM  # noqa: E402

WORLD = ROOT / 'assets' / 'world_p4_handover.xml'
DESIGN_SHIFT = 3.4184


def main():
    reach = json.loads((ROOT / 'reports' / 'p4-reach-01' / 'report.json').read_text(encoding='utf-8'))
    windows = {round(w['dz_m'], 3): w['reachable_dx_m'] for w in reach['measurements']['station_windows']}

    body = Body()
    m, d = body.m, body.d
    body.reset()
    body.run(1.2)
    q_stand = np.array(d.qpos, dtype=float)
    base_x_design = float(d.xpos[m.body('h_LINK_BASE').id][0])
    base_x_unmigrated = base_x_design - DESIGN_SHIFT
    foot_pos = {s: np.array(d.xpos[m.body(FOOT[s]).id], dtype=float) for s in ('L', 'R')}
    foot_mat = {s: np.array(d.xmat[m.body(FOOT[s]).id], dtype=float).reshape(3, 3) for s in ('L', 'R')}
    for s in ('L', 'R'):
        if np.shares_memory(foot_mat[s], d.xmat[m.body(FOOT[s]).id]):
            raise RuntimeError('foot_mat aliases d.xmat')

    band = [g for g in range(m.ngeom)
            if 'c_fixed_roller' in (mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_GEOM, g) or '')]
    humanoid = [g for g in range(m.ngeom)
                if (mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY,
                                      int(m.geom_bodyid[g])) or '').startswith('h_')]
    fromto = np.zeros(6, dtype=float)

    print(f'base x at the design point {base_x_design:.6f};  shift {DESIGN_SHIFT} -> '
          f'un-migrated base x {base_x_unmigrated:.6f}')
    print(f'band crowns {len(band)}, humanoid geoms {len(humanoid)}')
    print()
    print(f'{"crouch":>7} {"arm needs shift":>15} {"leg dx_c":>9} {"=> station":>11} '
          f'{"body-vs-band":>13} {"verdict":>9}')

    rows = []
    for dz in (-0.10, -0.15, -0.20, -0.22, -0.24, -0.26, -0.28, -0.30, -0.32, -0.34, -0.36, -0.40):
        key = round(dz, 3)
        win = windows.get(key)
        if win is None:
            # the reach sweep sampled every 20 mm from -0.24; interpolate onto its own grid
            ks = sorted(windows)
            lo = max([k for k in ks if k <= key], default=None)
            hi = min([k for k in ks if k >= key], default=None)
            if lo is None or hi is None:
                continue
            f = (key - lo) / (hi - lo) if hi != lo else 0.0
            arm_shift = windows[lo][0] + f * (windows[hi][0] - windows[lo][0])
        else:
            arm_shift = win[0]
        leg_dx = LM_DX.get(round(-dz, 2), None)
        if leg_dx is None:
            leg_dx = LM_DX.get(min(LM_DX, key=lambda k: abs(k + dz)))
        # FEET at `station`, so the base is at station + the pelvis offset the legs need
        station = arm_shift - leg_dx
        q = np.array(q_stand, dtype=float)
        q[0] = base_x_unmigrated + station + leg_dx
        q[2] = float(q_stand[2]) + dz
        tgt = {s: np.array([foot_pos[s][0] + (station - DESIGN_SHIFT), foot_pos[s][1],
                            foot_pos[s][2]], dtype=float) for s in ('L', 'R')}
        worst = 0.0
        for s in ('L', 'R'):
            r, best = LM.solve(body, s, q, tgt[s], foot_mat[s], flat=True)
            worst = max(worst, r)
            q = np.array(best, dtype=float)
        d.qpos[:] = q
        d.qvel[:] = 0.0
        mujoco.mj_forward(m, d)
        clear, pair, active_pairs, pen_from_inactive = np.inf, '', 0, 0
        for g in humanoid:
            for b in band:
                # \u2605 ONLY PAIRS THE FILTER WOULD ACT ON. `mj_geomDistance` ignores contype and
                # conaffinity, so it also reports overlaps between a collision geom and a VISUAL-only
                # one (the humanoid's knee carries both a group-2 visual geom and group-3 collision
                # geoms). A penetration no contact model acts on is not a wall, so the pair has to be
                # filtered before the number means anything. Counted, not assumed: 144 of 240
                # knee/crown pairs are active.
                c1, a1 = int(m.geom_contype[g]), int(m.geom_conaffinity[g])
                c2, a2 = int(m.geom_contype[b]), int(m.geom_conaffinity[b])
                if not ((c1 & a2) or (c2 & a1)):
                    continue
                active_pairs += 1
                dd = float(mujoco.mj_geomDistance(m, d, g, b, 1.0, fromto))
                if dd < clear:
                    clear = dd
                    pair = (f'{mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, int(m.geom_bodyid[g]))}'
                            f' x {mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_GEOM, b)}')
        if not active_pairs:
            raise RuntimeError('no ACTIVE humanoid-vs-band geom pair: the constraint would be vacuous')
        # ★ THE TWO FAILURE MODES ARE REPORTED SEPARATELY. The first version ANDed them into one
        # verdict, so a row that failed because the LEG IK had not converged was indistinguishable
        # from a row that failed because the body touched the band -- two completely different
        # findings wearing the same word.
        ik_ok = worst <= 1e-3
        clear_ok = clear >= 0.0
        ok = ik_ok and clear_ok
        print(f'{dz:+.2f} {arm_shift:15.2f} {leg_dx:9.2f} {station:11.2f} '
              f'{clear * 1000:12.2f}mm {"FEASIBLE" if ok else "no":>9}  '
              f'ik_resid {worst * 1000:8.4f}mm {"ok" if ik_ok else "FAILED"}  '
              f'[{active_pairs} active pairs]  {pair}')
        rows.append({'dz_m': dz, 'arm_shift_m': arm_shift, 'leg_dx_m': leg_dx,
                     'station_m': station, 'leg_ik_resid_m': worst,
                     'leg_ik_converged': bool(ik_ok),
                     'body_vs_band_mm': clear * 1000, 'closest_pair': pair,
                     'clearance_ok': bool(clear_ok), 'feasible': bool(ok),
                     'active_pairs': active_pairs})

    feas = [r['dz_m'] for r in rows if r['feasible']]
    print()
    print(f'*** deepest crouch reachable with the REAL constraint: {min(feas) if feas else None}')
    print(f'*** the old cap (3.4236 m, my 1-D assumption) forced: 0.36 m')
    out = ROOT / 'reports' / 'p4-clearance-01'
    out.mkdir(parents=True, exist_ok=True)
    (out / 'report.json').write_text(json.dumps(
        {'run_id': 'p4-clearance-01',
         'scope': 'recompute the station cap in 3-D and the crouch depth it forces',
         'cap_claimed_by_p4_reach_01_m': 3.4236,
         'cap_measured_m': None,
         'table_vs_band_mm_at_every_shift': 311.04,
         'deepest_feasible_crouch_m': (min(feas) if feas else None),
         'rows': rows}, indent=2) + '\n', encoding='utf-8')
    return 0


#: pelvis x per depth, from `_dbg_p4_legmap` (FULL mode, best CoM centring). Depth is positive here.
LM_DX = {0.10: -0.04, 0.15: -0.08, 0.20: -0.08, 0.25: -0.12,
         0.30: -0.16, 0.35: -0.20, 0.40: -0.20, 0.45: -0.20, 0.50: -0.24}

if __name__ == '__main__':
    raise SystemExit(main())
