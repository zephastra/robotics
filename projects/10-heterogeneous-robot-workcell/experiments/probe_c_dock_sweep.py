"""The docking tolerance sweep: does C's transfer still work when the AMR parks imperfectly?

THE DEBT
--------
`D041` says it in one line: bolting the deck to the chassis turned the deck-to-receiver spacing
from a design constant into `(where the AMR parked) - (where the receiver is)` -- a DOCKING
TOLERANCE, i.e. a different claim. The gate then parks the chassis at the exact standoff the old
constants assumed (`dock_offset +3.9e-06`) and its own `not_established` admits the consequence:

    "that C's docking tolerance is acceptable (the chassis is parked at the exact standoff the old
     constants assumed -- the tolerance has not been swept)"

This file sweeps it.

HOW THE PARK IS PERTURBED
-------------------------
The park is baked into `n_base_link`'s BODY POSE, not into a joint: `n_slide_x/y/z` all have range
`[0, 0]`, so the chassis cannot move during a simulation at all. So the perturbation is applied at
MODEL BUILD time, through `MjSpec`, before compiling:

    n_base_link.pos  += (dx, 0, 0)
    n_base_link.quat  = yaw(dtheta)

That is a rigid shift of the whole vehicle (chassis + deck + pusher). The fixture and the tray are
world-level and stay put, so the deck-to-fixture relationship changes -- which is exactly the
docking error a real AMR would present.

CONTROL: the `dx = 0, dtheta = 0` row must reproduce the shipped model's `dock_offset` (+3.9e-06).
If the spec round-trip moved anything, that row would say so, and the whole sweep would be void.

TWO PHYSICAL READOUTS PER ROW, so the perturbation can be trusted:
  * `dock_offset_m`  -- deck world x minus (C's station + DECK_X0), the gate's own headline number
  * `deck_gap_m`     -- the gap between the source row's last crown and the deck's first crown.
    At the nominal park the crowns nearly touch (2r = 0.07), so this gap IS the docking error in
    the units C's own interlock measures.

WHAT THIS DOES NOT DO
---------------------
It does NOT reproduce C's ALIGN window, so configurations C's interlock would REFUSE are admitted
and run here. That is deliberate: the question is whether the MECHANISM tolerates the error, which
is a different (and prerequisite) question from whether the gate should admit it. And it does not
run the pusher, the interlock or PLATFORM_CLEAR, so it is NOT C's acceptance.
"""
from __future__ import annotations

import json
import math
import pathlib
import sys

ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'experiments'))
sys.path.insert(0, str(ROOT / 'src'))

import mujoco  # noqa: E402
import numpy as np  # noqa: E402

import roller_rig as rig  # noqa: E402
import probe_c_full_chain as fc  # noqa: E402

MERGED = ROOT / 'assets' / 'world_p1_cell.xml'
CHASSIS = 'n_base_link'

# dx in metres, dtheta in degrees. 0 appears once; the halo is the tail of the list.
# The first grid jumped straight to +-5 mm and +-0.5 deg, which is already past the failure for
# everything on the "too close" / "rotated" side. This one brackets the boundary.
DX_SWEEP = (0.0, 0.002, 0.005, 0.010, 0.020, 0.040, -0.001, -0.002, -0.003, -0.005)
DTHETA_SWEEP = (0.0, 0.05, 0.1, 0.2, 0.5)


#: The chassis is parked by a NON-ZERO `n_slide_x` qpos, and body_pos is the joint's zero.
#: Measured: body_pos_x 9.03174 with the chassis sitting at 4.41372 -> the slide carries the park
#: offset L = 4.61802 m. MuJoCo expresses a slide joint's axis in the CHILD body's frame, so
#: rotating the chassis rotates the slide direction too and the park offset SWINGS with it:
#:
#:     parked(quat) = body_pos + qpos * R(theta) * axis
#:
#: With qpos = -L and axis = +x this puts the chassis at body_pos - L*(cos t, sin t, 0). Rotating
#: the body alone therefore ALSO translates the robot by -L*sin(t) in y -- measured 4.0 mm at
#: 0.05 deg, i.e. a lateral error 3.4x larger than the intended 1.92*sin(t) swing of the deck
#: relative to the fixture, which is why the first angular sweep looked 1.4x too strong.
#:
#: This cancels it, so the perturbation is a pure yaw about the PARKED origin:
#:     body_pos <- body_pos + L*(cos t - 1, sin t, 0)
def _park_lever(m):
    d = mujoco.MjData(m)
    # AT HOME. Without this the chassis sits at body_pos (qpos 0) and the difference is 0 --
    # which is exactly what an earlier version of this helper returned, silently disabling the
    # compensation it exists to provide.
    if m.nkey > 0:
        d.qpos[:] = m.key_qpos[0]
    mujoco.mj_forward(m, d)
    ch = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, CHASSIS)
    return float(m.body_pos[ch][0]) - float(d.xpos[ch][0])


_LEVER = _park_lever(mujoco.MjModel.from_xml_path(str(MERGED)))


def build(dx=0.0, dtheta_deg=0.0):
    spec = mujoco.MjSpec.from_file(str(MERGED))
    b = spec.body(CHASSIS)
    if b is None:
        raise SystemExit(f'no body {CHASSIS!r} in {MERGED.name}')
    th = math.radians(dtheta_deg)
    b.pos = [float(b.pos[0]) + dx + _LEVER * (math.cos(th) - 1.0),
             float(b.pos[1]) + _LEVER * math.sin(th),
             float(b.pos[2])]
    if dtheta_deg:
        h = th / 2.0
        b.quat = [math.cos(h), 0.0, 0.0, math.sin(h)]
    return spec.compile()


def geometry(m):
    d = mujoco.MjData(m)
    if m.nkey > 0:
        d.qpos[:] = m.key_qpos[0]
    mujoco.mj_forward(m, d)
    g = lambda n: float(d.xpos[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, n)][0])
    deck = g('c_deck')
    dock_offset = deck - (rig.DECK_X0 + g('c_fixed_roller_0_0'))
    deck_gap = g('c_deck_roller_0') - g('c_fixed_roller_2_7')
    # LATERAL readout: a yaw about the chassis origin barely moves the deck in x but swings it in y,
    # so for the angular rows `deck_y` is the quantity that means anything.
    deck_y = float(d.xpos[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, 'c_deck')][1])
    ch = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, CHASSIS)
    mat = np.zeros(9)
    mujoco.mju_quat2Mat(mat, d.xquat[ch])
    return {'deck_offset_from_source_row_m': round(dock_offset, 9),
            'deck_gap_m': round(deck_gap, 9), 'deck_world_x': round(deck, 6),
            'deck_y_m': round(deck_y, 9),
            'chassis_x_m': round(float(d.xpos[ch][0]), 9),
            'chassis_y_m': round(float(d.xpos[ch][1]), 9),
            'chassis_yaw_deg': round(math.degrees(math.atan2(float(mat[3]), float(mat[0]))), 6)}


def main():
    rows = []

    # --- control: the spec round-trip with no perturbation must reproduce the shipped model ---
    shipped = mujoco.MjModel.from_xml_path(str(MERGED))
    ship_geo = geometry(shipped)
    ctrl_geo = geometry(build(0.0, 0.0))
    print(f'park lever L = {_LEVER:.6f} m (the slide offset the yaw compensation cancels)')
    print('CONTROL (spec round-trip, no perturbation):')
    print(f"  shipped   {ship_geo}")
    print(f"  rebuilt   {ctrl_geo}")
    same = (abs(ship_geo['deck_offset_from_source_row_m']
                - ctrl_geo['deck_offset_from_source_row_m']) < 1e-9
            and abs(ship_geo['deck_gap_m'] - ctrl_geo['deck_gap_m']) < 1e-9
            and abs(ship_geo['deck_y_m'] - ctrl_geo['deck_y_m']) < 1e-9)
    print(f"  -> {'IDENTICAL' if same else 'DIFFERENT -- THE SWEEP WOULD BE VOID'}")
    if not same:
        return 2

    configs = [('dx=%+.3f m' % dx, dx, 0.0) for dx in DX_SWEEP]
    configs += [('dtheta=%+.2f deg' % th, 0.0, th) for th in DTHETA_SWEEP if th]

    print('\n' + '=' * 100)
    print('DOCKING TOLERANCE SWEEP -- does the transfer still finish?')
    print('=' * 100)
    print(f"{'config':16s} {'ch_yaw':>8s} {'ch_y':>9s} {'deck_y':>9s} {'deck_gap':>9s} "
          f"{'final':>10s} {'seated yaw':>11s} {'fell':>5s}")
    print('-' * 100)
    for tag, dx, th in configs:
        m = build(dx, th)
        geo = geometry(m)
        r = fc.run_chain(m, tag, payload='c_payload', deck='c_deck',
                         deck_roller='c_deck_roller_', fixed_roller='c_fixed_roller_',
                         deck_slide='c_deck_slide')
        seated = (r['marks'].get('SETTLE_end') or {}).get('yaw')
        end = (r['marks'].get('RECEIVED') or r['marks'].get('UNLOAD_STALLED')
               or r['marks'].get('EMBARK_end') or {})
        reached = 'RECEIVED' in r['marks']
        rows.append({'config': tag, 'dx': dx, 'dtheta_deg': th, **geo,
                     'reached_received': reached, 'final_state': r['final_state'],
                     'fell': r['fell'], 'seated_yaw': seated, 'yaw_at_end': end.get('yaw'),
                     'x_at_end_local': end.get('x_local'),
                     'deck_travel': (r['marks'].get('EMBARK_end') or {}).get('deck_travel')})
        print(f"{tag:16s} {geo['chassis_yaw_deg']:+8.3f} {geo['chassis_y_m']:+9.5f} "
              f"{geo['deck_y_m']:+9.5f} {geo['deck_gap_m']:9.6f} "
              f"{'RECEIVED' if reached else r['final_state']:>10s} "
              f"{(seated if seated is not None else float('nan')):11.3f} "
              f"{str(r['fell']):>5s}")

    ok = [r for r in rows if r['reached_received'] and not r['fell']]
    print(f"\n{len(ok)}/{len(rows)} configurations reached RECEIVED without falling.")
    print('  reached : ' + (', '.join(r['config'] for r in ok) or 'none'))
    bad = [r for r in rows if not (r['reached_received'] and not r['fell'])]
    print('  failed  : ' + (', '.join(f"{r['config']}({'fell' if r['fell'] else r['final_state']})"
                                      for r in bad) or 'none'))
    pos = [r for r in rows if r['dx'] > 0 and r['reached_received'] and not r['fell']]
    neg = [r for r in rows if r['dx'] < 0 and r['reached_received'] and not r['fell']]
    ang = [r for r in rows if r['dtheta_deg'] > 0 and r['reached_received'] and not r['fell']]
    print(f"  deck FURTHER away (dx>0): {len(pos)} work, up to dx {max((r['dx'] for r in pos), default=float('nan')):+.3f} m")
    print(f"  deck TOO CLOSE   (dx<0): {len(neg)} work")
    print(f"  deck ROTATED  (dth>0): {len(ang)} work, up to dtheta {max((r['dtheta_deg'] for r in ang), default=float('nan')):+.2f} deg")

    print('\n⚠️ The interlock is NOT reproduced here, so a configuration that C\'s ALIGN window would\n'
          '   REFUSE is admitted and run. This measures the MECHANISM\'s tolerance, not the gate\'s.')
    print('⚠️ NOT C\'s acceptance: the pusher, the interlock and PLATFORM_CLEAR are not run.')

    out = ROOT / 'reports' / 'p2-c-dock-sweep-01'
    out.mkdir(parents=True, exist_ok=True)
    (out / 'report.json').write_text(json.dumps({
        'control': {'shipped': ship_geo, 'rebuilt': ctrl_geo, 'identical': same},
        'dx_sweep_m': list(DX_SWEEP), 'dtheta_sweep_deg': list(DTHETA_SWEEP),
        'rows': rows,
        'reached_received_clean': [r['config'] for r in ok],
    }, indent=1), encoding='utf-8')
    print(f'\nwrote {out}/report.json')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
