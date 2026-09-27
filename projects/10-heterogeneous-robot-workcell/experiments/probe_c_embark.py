"""Peel the onion, steps 1+2: CONVEY -> RELEASE -> SETTLE -> EMBARK in the assembled world.

STEP 1 (RELEASE / SETTLE) -- answered, see `reports/p2-c-release-settle-01`
--------------------------------------------------------------------------
Cutting the rollers removes ~0.01 deg of yaw. It freezes the yaw; it does not undo it. In the
assembled world the tray is left seated with **7.555 deg** of yaw. That refutes the outside
review's prediction that a settled yaw under 2-3 deg would make the rolling drift irrelevant.

STEP 2 (EMBARK) -- this file
----------------------------
With the tray seated and stationary, C's EMBARK slides the deck out by DECK_TRAVEL (0.240 m) with
the DECK ROLLER DRIVE CUT to zero. That cut is load-bearing and is copied from `probe_deck`:

    "F1 (D034): the deck roller drive is CUT during EMBARK. Measured: with the rollers driven the
     tray gains relative slip equal to the roller surface speed integrated over the contact time
     (+0.13..0.42 m over this 0.2400 m dash, and one of three starts dropped the tray to the
     floor). With the drive cut the tray stays with the deck to <= 5.4 mm."

So C has its OWN measured number for the quantity this file measures: **slip <= 5.4 mm**. That
makes this a comparison against C's standalone result, not an invented threshold.

The setpoint is a RAMP, not a step plus timer (`deck_target = min(deck_travel_here, DECK_SPEED *
elapsed)`); ramping means the stage ends when the deck has actually travelled, not when a
stopwatch says so. `deck_travel_here = DECK_TRAVEL - gap - yaw_skew`; the scenario here is the
nominal one (gap 0, yaw 0), so it is DECK_TRAVEL.

WHAT IS MEASURED
----------------
  * does the deck actually travel DECK_TRAVEL?
  * does the tray ride with it -- `slip`, the change in the tray's position in the DECK frame,
    against C's own <= 5.4 mm
  * does the yaw change, i.e. does sliding the deck under a yawed tray straighten it or worsen it
  * does the tray stay flat and on the deck

WHAT THIS DOES NOT DO
---------------------
It does not run UNLOAD, the pusher or the interlock. It does not reimplement C's `aboard`
predicate (which lives in a comment and whose neighbouring code was wrong in a documented way);
the stage ends when the deck reaches its commanded travel. This is NOT C's acceptance.
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

ROLLER_SPEED = 5.0        # probe_deck.ROLLER_SPEED
SETTLE_END = 0.8          # probe_deck.SETTLE_END
RELEASE_HOLD = 0.6        # probe_deck.RELEASE_HOLD
SEAT_HOLD = 0.5           # probe_deck.SEAT_HOLD
DECK_SPEED = 0.15         # probe_deck.DECK_SPEED
DECK_TRAVEL = 0.240       # probe_deck.DECK_TRAVEL
EMBARK_DECK_ROLLER_SPEED = 0.0    # probe_deck.EMBARK_DECK_ROLLER_SPEED
BEAM_LENGTH = 0.50        # probe_deck.BEAM_LENGTH
EMBARK_BUDGET = 8.0       # sim seconds; the ramp needs 0.240 / 0.15 = 1.6 s
HORIZON = 60.0
SAMPLE_EVERY = 5

# C's own measured slip in its single-role world, quoted from probe_deck's EMBARK note.
C_OWN_SLIP_M = 0.0054


def cast(m, d, origin, direction, length):
    gid = np.zeros(1, dtype=np.int32)
    dist = mujoco.mj_ray(m, d, np.asarray(origin, dtype=np.float64),
                         np.asarray(direction, dtype=np.float64), None, 1, -1, gid)
    if dist < 0.0 or dist > length:
        return None, None
    return int(gid[0]), float(dist)


def run_world(m, tag, *, payload, deck, deck_roller, fixed_roller, deck_slide):
    d = mujoco.MjData(m)
    if m.nkey > 0:
        d.qpos[:] = m.key_qpos[0]
    mujoco.mj_forward(m, d)

    def aid(pref):
        return [i for i in range(m.nu)
                if (mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_ACTUATOR, i) or '').startswith(pref)]

    fixed = aid(fixed_roller)
    deckr = aid(deck_roller)
    deck_drive = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_ACTUATOR, deck_roller.replace('deck_roller_', 'deck_drive'))
    tray_body = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, payload)
    deck_body = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, deck)
    fx0 = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, fixed_roller + '0_0')
    jslide = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, deck_slide)
    qadr = int(m.jnt_qposadr[jslide])
    tray_geoms = {g for g in range(m.ngeom) if m.geom_bodyid[g] == tray_body}

    def deck_to_world(local):
        return d.xpos[deck_body] + d.xmat[deck_body].reshape(3, 3) @ np.asarray(local, dtype=np.float64)

    def deck_dir_world(local):
        v = d.xmat[deck_body].reshape(3, 3) @ np.asarray(local, dtype=np.float64)
        n = float(np.linalg.norm(v))
        if n < 1e-9:
            raise SystemExit('degenerate deck frame; refusing to cast a zero ray')
        return v / n

    def beam(local_x):
        hit, _ = cast(m, d, deck_to_world((local_x, rig.BEAM_Y0, rig.BEAM_Z)),
                      deck_dir_world((0.0, 1.0, 0.0)), BEAM_LENGTH)
        return hit in tray_geoms

    mat = np.zeros(9)

    def rel_x():
        """The tray's position in the DECK frame, along the travel direction."""
        return float((d.xmat[deck_body].reshape(3, 3).T
                      @ (d.xpos[tray_body] - d.xpos[deck_body]))[0])

    def pose():
        mujoco.mju_quat2Mat(mat, d.xquat[tray_body])
        tilt = math.degrees(math.acos(max(-1.0, min(1.0, float(mat[8])))))
        yaw = math.degrees(math.atan2(float(mat[3]), float(mat[0])))
        return (float(d.xpos[tray_body][0]), float(d.xpos[tray_body][2]), tilt, yaw)

    def snap(t):
        x, z, tilt, yaw = pose()
        return {'t': round(t, 4), 'x_local': round(x - origin, 6), 'z': round(z, 6),
                'tilt': round(tilt, 4), 'yaw': round(yaw, 4),
                'rel_x': round(rel_x(), 6), 'deck_qpos': round(float(d.qpos[qadr]), 6)}

    origin = float(d.xpos[fx0][0])
    state = 'CONVEY'
    state_since = 0.0
    marks = {}
    deck_at_embark0 = None
    rel_x_at_embark0 = None
    fell = False
    done = False
    trace = []

    while d.time < HORIZON and not done:
        t = float(d.time)
        elapsed = t - state_since
        if t < SETTLE_END:
            for i in fixed + deckr:
                d.ctrl[i] = 0.0
        elif state == 'CONVEY':
            for i in fixed + deckr:
                d.ctrl[i] = ROLLER_SPEED
            if beam(rig.BEAM_SEAT_A_X) and beam(rig.BEAM_SEAT_B_X):
                marks['CONVEY_end'] = snap(t)
                state, state_since = 'RELEASE', t
            elif beam(rig.BEAM_FAR_X):
                state, state_since = 'OVERTRAVEL', t
        elif state == 'RELEASE':
            for i in fixed:
                d.ctrl[i] = 0.0
            for i in deckr:
                d.ctrl[i] = ROLLER_SPEED
            if elapsed > RELEASE_HOLD:
                marks['RELEASE_end'] = snap(t)
                state, state_since = 'SETTLE', t
        elif state == 'SETTLE':
            for i in fixed + deckr:
                d.ctrl[i] = 0.0
            if elapsed > SEAT_HOLD:
                marks['SETTLE_end'] = snap(t)
                deck_at_embark0 = float(d.qpos[qadr])
                rel_x_at_embark0 = rel_x()
                state, state_since = 'EMBARK', t
        elif state == 'EMBARK':
            # F1/D034: the deck roller drive is CUT; the deck's own motion carries the tray.
            for i in fixed + deckr:
                d.ctrl[i] = EMBARK_DECK_ROLLER_SPEED
            # ramped setpoint, so the stage ends when the deck has travelled, not on a stopwatch
            d.ctrl[deck_drive] = min(DECK_TRAVEL, DECK_SPEED * elapsed)
            trav = float(d.qpos[qadr]) - deck_at_embark0
            if trav >= DECK_TRAVEL - 1e-6 or elapsed > EMBARK_BUDGET:
                marks['EMBARK_end'] = snap(t)
                marks['EMBARK_end']['deck_travel'] = round(trav, 6)
                marks['EMBARK_end']['slip'] = round(abs(rel_x() - rel_x_at_embark0), 6)
                done = True
        else:                                   # OVERTRAVEL
            for i in fixed + deckr:
                d.ctrl[i] = 0.0
            done = True

        if done:
            break

        mujoco.mj_step(m, d)
        x, z, tilt, yaw = pose()
        if z < 0.40:
            fell = True
        if int(round(t / m.opt.timestep)) % SAMPLE_EVERY == 0:
            trace.append({'t': round(t, 4), 'x_local': round(x - origin, 6), 'z': round(z, 6),
                          'tilt': round(tilt, 4), 'yaw': round(yaw, 4),
                          'rel_x': round(rel_x(), 6), 'deck_qpos': round(float(d.qpos[qadr]), 6),
                          'state': state})

    return {'world': tag, 'final_state': state, 'fell': fell, 'marks': marks, 'trace_tail': trace[-40:]}


def main():
    worlds = []
    for g in (False, True):
        xml, _ = rig.build_model(TRAY, receiver=True, guides=g)
        worlds.append(run_world(mujoco.MjModel.from_xml_string(xml),
                                f'standalone guides={g}',
                                payload='payload', deck='deck', deck_roller='deck_roller_',
                                fixed_roller='fixed_roller_', deck_slide='deck_slide'))
    worlds.append(run_world(mujoco.MjModel.from_xml_path(str(MERGED)), 'MERGED world_p1_cell.xml',
                            payload='c_payload', deck='c_deck', deck_roller='c_deck_roller_',
                            fixed_roller='c_fixed_roller_', deck_slide='c_deck_slide'))

    print('=' * 96)
    print('PEEL THE ONION 1+2 -- CONVEY -> RELEASE -> SETTLE -> EMBARK')
    print('=' * 96)
    for w in worlds:
        mk = w['marks']
        print(f"\n{w['world']}   (final {w['final_state']}, fell={w['fell']})")
        for k in ('CONVEY_end', 'RELEASE_end', 'SETTLE_end', 'EMBARK_end'):
            s = mk.get(k)
            if not s:
                print(f'   {k:12s} --')
                continue
            extra = ''
            if 'deck_travel' in s:
                extra = f"  deck_travel {s['deck_travel']:+.4f}  slip {s['slip']:.4f}"
            print(f"   {k:12s} t {s['t']:6.3f}  x {s['x_local']:8.4f}  yaw {s['yaw']:8.3f}  "
                  f"tilt {s['tilt']:6.3f}  rel_x {s['rel_x']:8.4f}{extra}")

    print('\n--- EMBARK, against C\'s own standalone measurement ---')
    for w in worlds:
        s = w['marks'].get('EMBARK_end')
        if not s:
            print(f"  {w['world']:26s} did not reach EMBARK")
            continue
        ok = s['slip'] <= C_OWN_SLIP_M
        print(f"  {w['world']:26s} travel {s['deck_travel']:+.4f} m  slip {s['slip']:.4f} m  "
              f"(C's own bar {C_OWN_SLIP_M:.4f})  -> {'within' if ok else 'ABOVE'}  "
              f"yaw {s['yaw']:+.3f}")

    mw = next(w for w in worlds if w['world'].startswith('MERGED'))
    st = mw['marks'].get('SETTLE_end')
    em = mw['marks'].get('EMBARK_end')
    if st and em:
        print(f"\nMERGED: yaw seated {st['yaw']:+.3f} deg -> after EMBARK {em['yaw']:+.3f} deg "
              f"(change {em['yaw'] - st['yaw']:+.3f}); tilt after {em['tilt']:.3f} deg; "
              f"slip {em['slip']:.4f} m vs C's {C_OWN_SLIP_M:.4f} m")

    out = ROOT / 'reports' / 'p2-c-embark-01'
    out.mkdir(parents=True, exist_ok=True)
    (out / 'report.json').write_text(json.dumps({
        'merged_sha256': hashlib.sha256(MERGED.read_bytes()).hexdigest(),
        'deck_speed': DECK_SPEED, 'deck_travel': DECK_TRAVEL,
        'embark_deck_roller_speed': EMBARK_DECK_ROLLER_SPEED,
        'c_own_slip_m': C_OWN_SLIP_M,
        'worlds': worlds,
    }, indent=1), encoding='utf-8')
    print(f'\nwrote {out}/report.json')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
