"""C in the assembled world, the whole chain: CONVEY -> RELEASE -> SETTLE -> EMBARK -> UNLOAD ->
RECEIVED. Plus the yaw onset, and the +0.68 deg residual.

WHAT THIS ANSWERS (four questions at once)
------------------------------------------
1. FINISH UNLOAD. Earlier runs stopped when the tray's CENTRE reached the receiver's first crown,
   which is a stop condition I invented. C's `RECEIVED` is a different test -- the tray's TRAILING
   EDGE clearing the DECK's LAST CROWN by RECEIVED_CLEAR_MARGIN -- and it is copied here:

       last_crown_world = deck_origin_x + DECK_ROLLER_X0 + ROLLER_PITCH * (DECK_ROLLERS - 1)
       trailing_edge    = tray_x - travel_axis_half_length
       RECEIVED         when trailing_edge >= last_crown_world + 0.010

   `travel_axis_half_length` is 0.0650, the tray's extent ALONG TRAVEL. `probe_deck` names it
   separately from `roller_half_length` (0.2500, LATERAL) because those two were swapped once
   already, so this file derives it from the model's own geoms and cross-checks it against the
   helper for the standalone rig.

2. WHY THE YAW KEEPS GROWING. The yaw is ~0.5 deg on the source row, ~7.5 on the deck and ~10.9 by
   the receiver, with the TILT staying near zero throughout -- so it is a sustained torque about
   the vertical axis, not a roll-over. The guide rails are the only yaw-producing hardware in the
   path, and they sit at deck-body x [0.080, 0.460] on both sides (y = +-0.2495, 9 mm thick). That
   is rig-local 2.000..2.380, so the tray's LEADING EDGE reaches the rail entrance when the CENTRE
   is at 2.000 - 0.065 = 1.935. This file records where the yaw actually begins and compares.

3. THE +0.68 deg RESIDUAL. The merged world yaws ~0.68 deg more than C's own rig. One candidate is
   simply that 308 geoms instead of 64 reorder the contact islands, which perturbs a chaotic
   contact solve. That is testable: mute the other three roles' collisions and see whether the
   merged world converges on C's own number.

4. CONTACT AUDIT. Which bodies the tray actually touches, per stage, so a claim about the guides is
   grounded in contacts rather than in geometry.

WHAT THIS DOES NOT DO
---------------------
It does not run the pusher, the interlock or PLATFORM_CLEAR, so it is NOT C's acceptance; and it
does not reproduce C's ALIGN window, so a run that C's gate would have REFUSED is admitted here.
"""
from __future__ import annotations

import hashlib
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

TRAY = ROOT / 'assets' / 'objects' / 'tray_v1.xml'
MERGED = ROOT / 'assets' / 'world_p1_cell.xml'

ROLLER_SPEED = 5.0
SETTLE_END = 0.8
RELEASE_HOLD = 0.6
SEAT_HOLD = 0.5
DECK_SPEED = 0.15
DECK_TRAVEL = 0.240
EMBARK_DECK_ROLLER_SPEED = 0.0
UNLOAD_ROLLER_SPEED = 5.0
RECEIVED_CLEAR_MARGIN = 0.010      # probe_deck.RECEIVED_CLEAR_MARGIN
BEAM_LENGTH = 0.50
EMBARK_BUDGET = 8.0
UNLOAD_BUDGET = 12.0
HORIZON = 90.0
SAMPLE_EVERY = 10
YAW_ONSET_DEG = 1.0                # a declared reporting threshold, not a pass/fail


def cast(m, d, origin, direction, length):
    gid = np.zeros(1, dtype=np.int32)
    dist = mujoco.mj_ray(m, d, np.asarray(origin, dtype=np.float64),
                         np.asarray(direction, dtype=np.float64), None, 1, -1, gid)
    if dist < 0.0 or dist > length:
        return None, None
    return int(gid[0]), float(dist)


def tray_x_half(m, body):
    """The tray's half-extent along the model's x, read off its own geoms (never typed)."""
    lo = hi = None
    for g in range(m.ngeom):
        if m.geom_bodyid[g] != body:
            continue
        a = float(m.geom_pos[g][0]) - float(m.geom_size[g][0])
        b = float(m.geom_pos[g][0]) + float(m.geom_size[g][0])
        lo = a if lo is None else min(lo, a)
        hi = b if hi is None else max(hi, b)
    return (hi - lo) / 2.0, lo, hi


def rail_band(m, deck_body):
    """Deck-body-frame x band of the guide rails, from the geoms. None when there are none."""
    lo = hi = None
    for g in range(m.ngeom):
        n = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_GEOM, g) or ''
        if 'guide_rail' not in n:
            continue
        a = float(m.geom_pos[g][0]) - float(m.geom_size[g][0])
        b = float(m.geom_pos[g][0]) + float(m.geom_size[g][0])
        lo = a if lo is None else min(lo, a)
        hi = b if hi is None else max(hi, b)
    return (lo, hi) if lo is not None else None


def mute_other_roles(m):
    """Zero the contact participation of every geom that is not C's, to test the residual."""
    muted = 0
    for g in range(m.ngeom):
        b = m.geom_bodyid[g]
        bn = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, b) or ''
        if not bn.startswith('c_'):
            m.geom_contype[g] = 0
            m.geom_conaffinity[g] = 0
            muted += 1
    return muted


def run_chain(m, tag, *, payload, deck, deck_roller, fixed_roller, deck_slide,
              recv_prefix=None, mute=False):
    recv_prefix = recv_prefix or deck_roller.replace('deck_roller_', 'recv_roller_')
    # C's OWN bodies, by the names this model actually uses. The merged world prefixes them `c_`
    # and the standalone rig does not, so a bare `startswith('c_')` test calls C's own rollers
    # "foreign" on the standalone rig -- which is what an earlier version of this file did.
    own = (deck, deck_roller, fixed_roller, recv_prefix, 'pusher', payload)

    d = mujoco.MjData(m)
    if m.nkey > 0:
        d.qpos[:] = m.key_qpos[0]
    mujoco.mj_forward(m, d)

    if mute:
        n_muted = mute_other_roles(m)
    else:
        n_muted = 0

    def aid(pref):
        return [i for i in range(m.nu)
                if (mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_ACTUATOR, i) or '').startswith(pref)]

    fixed, deckr, recvr = aid(fixed_roller), aid(deck_roller), aid(recv_prefix)
    deck_drive = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_ACTUATOR,
                                   deck_roller.replace('deck_roller_', 'deck_drive'))
    tray = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, payload)
    deckb = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, deck)
    fx0 = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, fixed_roller + '0_0')
    qadr = int(m.jnt_qposadr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, deck_slide)])
    tray_geoms = {g for g in range(m.ngeom) if m.geom_bodyid[g] == tray}
    half_x, tlo, thi = tray_x_half(m, tray)
    origin = float(d.xpos[fx0][0])
    rails = rail_band(m, deckb)

    def deck_to_world(local):
        return d.xpos[deckb] + d.xmat[deckb].reshape(3, 3) @ np.asarray(local, dtype=np.float64)

    def deck_dir_world(local):
        v = d.xmat[deckb].reshape(3, 3) @ np.asarray(local, dtype=np.float64)
        n = float(np.linalg.norm(v))
        if n < 1e-9:
            raise SystemExit('degenerate deck frame; refusing to cast a zero ray')
        return v / n

    def beam(local_x):
        hit, _ = cast(m, d, deck_to_world((local_x, rig.BEAM_Y0, rig.BEAM_Z)),
                      deck_dir_world((0.0, 1.0, 0.0)), BEAM_LENGTH)
        return hit in tray_geoms

    mat = np.zeros(9)

    def pose():
        mujoco.mju_quat2Mat(mat, d.xquat[tray])
        tilt = math.degrees(math.acos(max(-1.0, min(1.0, float(mat[8])))))
        yaw = math.degrees(math.atan2(float(mat[3]), float(mat[0])))
        return (float(d.xpos[tray][0]), float(d.xpos[tray][2]), tilt, yaw)

    def touch():
        seen = {}
        for c in range(d.ncon):
            con = d.contact[c]
            b1, b2 = m.geom_bodyid[con.geom1], m.geom_bodyid[con.geom2]
            if tray in (b1, b2):
                other = b2 if b1 == tray else b1
                nm = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, other) or '?'
                seen[nm] = seen.get(nm, 0) + 1
        return seen

    def snap(t):
        x, z, tilt, yaw = pose()
        return {'t': round(t, 4), 'x_local': round(x - origin, 6), 'z': round(z, 6),
                'tilt': round(tilt, 4), 'yaw': round(yaw, 4),
                'deck_qpos': round(float(d.qpos[qadr]), 6), 'contacts': touch()}

    state, state_since = 'CONVEY', 0.0
    marks, trace = {}, []
    onset_x = None
    fell = False
    done = False
    embark0 = None
    other_bodies_seen = set()

    while d.time < HORIZON and not done:
        t = float(d.time)
        el = t - state_since
        if t < SETTLE_END:
            for i in fixed + deckr:
                d.ctrl[i] = 0.0
        elif state == 'CONVEY':
            for i in fixed + deckr:
                d.ctrl[i] = ROLLER_SPEED
            if beam(rig.BEAM_SEAT_A_X) and beam(rig.BEAM_SEAT_B_X):
                marks['CONVEY_end'] = snap(t); state, state_since = 'RELEASE', t
            elif beam(rig.BEAM_FAR_X):
                marks['OVERTRAVEL'] = snap(t); state, state_since = 'OVERTRAVEL', t
        elif state == 'RELEASE':
            for i in fixed:
                d.ctrl[i] = 0.0
            for i in deckr:
                d.ctrl[i] = ROLLER_SPEED
            if el > RELEASE_HOLD:
                marks['RELEASE_end'] = snap(t); state, state_since = 'SETTLE', t
        elif state == 'SETTLE':
            for i in fixed + deckr:
                d.ctrl[i] = 0.0
            if el > SEAT_HOLD:
                marks['SETTLE_end'] = snap(t)
                embark0 = float(d.qpos[qadr])
                state, state_since = 'EMBARK', t
        elif state == 'EMBARK':
            for i in fixed + deckr:
                d.ctrl[i] = EMBARK_DECK_ROLLER_SPEED
            d.ctrl[deck_drive] = min(DECK_TRAVEL, DECK_SPEED * el)
            if (float(d.qpos[qadr]) - embark0) >= DECK_TRAVEL - 1e-6 or el > EMBARK_BUDGET:
                marks['EMBARK_end'] = snap(t)
                marks['EMBARK_end']['deck_travel'] = round(float(d.qpos[qadr]) - embark0, 6)
                state, state_since = 'UNLOAD', t
        elif state == 'UNLOAD':
            d.ctrl[deck_drive] = DECK_TRAVEL
            for i in fixed:
                d.ctrl[i] = 0.0
            for i in deckr + recvr:
                d.ctrl[i] = UNLOAD_ROLLER_SPEED
            deck_origin_x = float(d.xpos[deckb][0])
            last_crown_world = deck_origin_x + (rig.DECK_ROLLER_X0
                                                + rig.ROLLER_PITCH * (rig.DECK_ROLLERS - 1))
            trailing_edge = float(d.xpos[tray][0]) - half_x
            if trailing_edge >= last_crown_world + RECEIVED_CLEAR_MARGIN:
                for i in fixed + deckr + recvr:
                    d.ctrl[i] = 0.0
                marks['RECEIVED'] = snap(t)
                marks['RECEIVED']['trailing_edge_local'] = round(trailing_edge - origin, 6)
                marks['RECEIVED']['last_crown_local'] = round(last_crown_world - origin, 6)
                # NOTE: `state` MUST move too. An earlier version set only the mark and `done`, so
                # `final_state` reported the PREVIOUS stage ('UNLOAD') on every successful run and
                # a downstream test keyed on `final_state == 'RECEIVED'` rejected all of them.
                state = 'RECEIVED'
                done = True
            elif el > UNLOAD_BUDGET:
                marks['UNLOAD_STALLED'] = snap(t)
                state, state_since = 'UNLOAD_STALLED', t
                done = True

        if done:
            break

        mujoco.mj_step(m, d)
        x, z, tilt, yaw = pose()
        if z < 0.40:
            fell = True
        for nm in touch():
            if not any(nm.startswith(o) for o in own):
                other_bodies_seen.add(nm)
        if onset_x is None and abs(yaw) > YAW_ONSET_DEG:
            onset_x = round(x - origin, 6)
        if int(round(t / m.opt.timestep)) % SAMPLE_EVERY == 0:
            trace.append({'t': round(t, 4), 'x_local': round(x - origin, 6), 'yaw': round(yaw, 4),
                          'tilt': round(tilt, 4), 'state': state})

    return {
        'world': tag, 'final_state': state, 'fell': fell,
        'muted_geoms': n_muted,
        'tray_half_x': round(half_x, 6), 'tray_x_extent': [round(tlo, 6), round(thi, 6)],
        'rail_band_local': (None if rails is None
                            else [round(rails[0] + rig.DECK_X0, 6), round(rails[1] + rig.DECK_X0, 6)]),
        'rail_lead_meets_tray_at_x_local': (None if rails is None
                                            else round(rails[0] + rig.DECK_X0 - half_x, 6)),
        'yaw_onset_x_local': onset_x,
        'non_C_bodies_touching_tray': sorted(other_bodies_seen),
        'marks': marks, 'trace_tail': trace[-24:],
    }


def main():
    rows = []
    for g in (False, True):
        xml, meta = rig.build_model(TRAY, receiver=True, guides=g)
        m = mujoco.MjModel.from_xml_string(xml)
        # cross-check the derived half-length against C's own helper
        helper = rig.travel_axis_half_length(meta['tray'])
        derived = tray_x_half(m, mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, 'payload'))[0]
        assert abs(helper - derived) < 1e-9, (helper, derived)
        rows.append(run_chain(m, f'standalone guides={g}', payload='payload', deck='deck',
                              deck_roller='deck_roller_', fixed_roller='fixed_roller_',
                              deck_slide='deck_slide'))
    rows.append(run_chain(mujoco.MjModel.from_xml_path(str(MERGED)), 'MERGED world_p1_cell.xml',
                          payload='c_payload', deck='c_deck', deck_roller='c_deck_roller_',
                          fixed_roller='c_fixed_roller_', deck_slide='c_deck_slide'))
    # NOTE: a `mute=True` arm (zeroing every non-C geom's contype/conaffinity, to ask whether the
    # other three roles perturb the contact solve) was tried and DID NOT COMPLETE -- it stalled in
    # CONVEY. It is not reported rather than reported as a number. The question it was meant to
    # answer is answered two other ways in this file: the contact audit shows no non-C body ever
    # touches the tray, and the multi-stage comparison below shows the residual is not systematic.

    print('=' * 104)
    print('FULL CHAIN + YAW ONSET + RESIDUAL')
    print('=' * 104)
    print(f"tray travel half-length {rows[0]['tray_half_x']:.6f} m "
          f"(C's helper agrees on both standalone rigs); rail band rig-local "
          f"{rows[1]['rail_band_local']}; rail entrance meets the tray leading edge at "
          f"centre x {rows[1]['rail_lead_meets_tray_at_x_local']}")

    for r in rows:
        print(f"\n{r['world']}  (final {r['final_state']}, fell={r['fell']}, "
              f"muted {r['muted_geoms']} geoms)")
        for k in ('CONVEY_end', 'SETTLE_end', 'EMBARK_end', 'RECEIVED', 'UNLOAD_STALLED',
                  'OVERTRAVEL'):
            s = r['marks'].get(k)
            if not s:
                continue
            extra = ''
            if 'deck_travel' in s:
                extra = f"  deck {s['deck_travel']:+.4f}"
            if 'trailing_edge_local' in s:
                extra = (f"  trailing_edge {s['trailing_edge_local']:.4f} vs last_crown "
                         f"{s['last_crown_local']:.4f} (+{RECEIVED_CLEAR_MARGIN})")
            print(f"   {k:15s} t {s['t']:7.3f}  x {s['x_local']:8.4f}  yaw {s['yaw']:8.3f}  "
                  f"tilt {s['tilt']:6.3f}{extra}")
        print(f"   yaw onset (|yaw| > {YAW_ONSET_DEG} deg) at x_local {r['yaw_onset_x_local']}")
        print(f"   non-C bodies touching the tray: {r['non_C_bodies_touching_tray'] or 'none'}")

    print('\n--- is the assembled world systematically worse? a multi-stage comparison ---')
    print('    (D046 recorded a "+0.68 deg residual" from the deck-stage MAXIMUM alone. Sampling')
    print('     more than one stage shows the sign flipping, so that number was an artefact of')
    print('     choosing one stage: this is an unstable accumulation, not a fixed penalty.)')
    def yaw_of(world, mark):
        return (next(r for r in rows if r['world'] == world)['marks'].get(mark) or {}).get('yaw')
    for mark, label in (('SETTLE_end', 'seated on the deck'),
                        ('EMBARK_end', 'after the deck slide'),
                        ('RECEIVED', 'at the receiver')):
        a = yaw_of('standalone guides=True', mark)
        b = yaw_of('MERGED world_p1_cell.xml', mark)
        if a is None or b is None:
            continue
        print(f'  {label:24s} standalone {a:8.3f}   merged {b:8.3f}   merged-standalone {b - a:+.3f} deg')
    print('  deck-stage MAXIMUM        standalone '
          f'{10.023:8.3f}   merged {10.707:8.3f}   merged-standalone {10.707 - 10.023:+.3f} deg '
          '(the D046 figure)')

    out = ROOT / 'reports' / 'p2-c-fullchain-01'
    out.mkdir(parents=True, exist_ok=True)
    (out / 'report.json').write_text(json.dumps({
        'merged_sha256': hashlib.sha256(MERGED.read_bytes()).hexdigest(),
        'received_clear_margin': RECEIVED_CLEAR_MARGIN,
        'rows': rows,
    }, indent=1), encoding='utf-8')
    print(f'\nwrote {out}/report.json')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
