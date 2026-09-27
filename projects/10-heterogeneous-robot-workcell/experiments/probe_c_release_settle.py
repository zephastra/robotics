"""RELEASE / SETTLE in the assembled world: what is the yaw once the tray has STOPPED?

THE QUESTION, AND WHERE IT COMES FROM
-------------------------------------
`D045`/`D046` measured a ~10.7 deg yaw while the tray is being conveyed across the deck, and
attributed it to C's own side guides (C's standalone rig does 10.02 deg with them, 1.28 deg
without). The obvious next question, and the one an outside review raised:

    "if the SETTLED yaw (after the rollers are cut) is < 2-3 deg, the 10 deg rolling drift
     does not matter and the guides do not need touching."

That is a prediction, not a result, and it is NOT obvious: stopping the rollers does not UNDO a
yaw the tray has already acquired -- it freezes whatever it had. So the settled value should
depend on WHERE the tray is when it is told to stop, which is exactly what C's seat beams decide.

WHY THE STOP POINT IS C'S OWN, NOT MINE
---------------------------------------
C's state machine is copied rather than paraphrased, because the answer depends entirely on where
the stop happens:

    t < SETTLE_END (0.8 s)   every actuator at 0            -- the scene settles first
    CONVEY                   fixed + deck rollers at ROLLER_SPEED
                             -> leaves when beam_seat_a AND beam_seat_b  (tray seated on the deck)
                             -> OVERTRAVEL if beam_far (leading edge ran past the seat window)
    RELEASE                  fixed = 0, deck rollers STILL at ROLLER_SPEED, for RELEASE_HOLD 0.6 s
    SETTLE                   everything 0, for SEAT_HOLD 0.5 s   <-- MEASUREMENT POINT

The beams are cast exactly as `probe_deck` casts them -- a +y ray from the deck frame at
(local_x, BEAM_Y0, BEAM_Z), hit tested against the tray's own geoms -- so this file cannot
disagree with C about "seated" by re-implementing the sensor differently.

THE ROLLERS ARE POSITION SERVOS, so `ctrl = 0` genuinely BRAKES them (biastype mjBIAS_AFFINE).
RELEASE is a real stop, not a free coast.

WORLDS, SAME AS THE ATTRIBUTION RUN
-----------------------------------
    standalone guides=False / guides=True / MERGED world_p1_cell.xml
so the settled yaw can be attributed the same way the rolling yaw was: if disabling the guides
also collapses the SETTLED yaw, the guides are decisively implicated.

WHAT THIS DOES NOT DO
---------------------
It stops at SETTLE. It does not run EMBARK (the deck slide), UNLOAD, the pusher or the interlock,
so it says nothing about the rest of the transfer and is NOT C's acceptance.
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

ROLLER_SPEED = 5.0                      # probe_deck.ROLLER_SPEED
SETTLE_END = 0.8                        # probe_deck.SETTLE_END
RELEASE_HOLD = 0.6                      # probe_deck.RELEASE_HOLD
SEAT_HOLD = 0.5                         # probe_deck.SEAT_HOLD
BEAM_LENGTH = 0.50                      # probe_deck.BEAM_LENGTH
HORIZON = 40.0                          # sim-second guard
SAMPLE_EVERY = 10

# The outside review's stated expectation, recorded so the result can be read against it.
PREDICTED_SETTLED_YAW_DEG = 3.0


def cast(m, d, origin, direction, length):
    gid = np.zeros(1, dtype=np.int32)
    dist = mujoco.mj_ray(m, d, np.asarray(origin, dtype=np.float64),
                         np.asarray(direction, dtype=np.float64), None, 1, -1, gid)
    if dist < 0.0 or dist > length:
        return None, None
    return int(gid[0]), float(dist)


def run_world(m, tag, *, payload, deck, deck_roller, fixed_roller):
    d = mujoco.MjData(m)
    if m.nkey > 0:                       # the merged world's parking lives in its `home` keyframe
        d.qpos[:] = m.key_qpos[0]
    mujoco.mj_forward(m, d)

    def aid(pref):
        return [i for i in range(m.nu)
                if (mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_ACTUATOR, i) or '').startswith(pref)]

    fixed = aid(fixed_roller)
    deckr = aid(deck_roller)
    tray_body = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, payload)
    deck_body = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, deck)
    fx0 = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, fixed_roller + '0_0')
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

    def pose():
        mujoco.mju_quat2Mat(mat, d.xquat[tray_body])
        tilt = math.degrees(math.acos(max(-1.0, min(1.0, float(mat[8])))))
        yaw = math.degrees(math.atan2(float(mat[3]), float(mat[0])))
        return (float(d.xpos[tray_body][0]), float(d.xpos[tray_body][2]), tilt, yaw)

    def snap(t):
        x, z, tilt, yaw = pose()
        return {'t': round(t, 4), 'x_local': round(x - origin, 6), 'z': round(z, 6),
                'tilt': round(tilt, 4), 'yaw': round(yaw, 4)}

    origin = float(d.xpos[fx0][0])

    state = 'CONVEY'
    state_since = 0.0
    at_release = at_settle = None
    max_yaw_convey = 0.0
    fell = False
    done = False
    trace = []

    while d.time < HORIZON and not done:
        t = float(d.time)
        # ---- command, by C's own stage table ----
        if t < SETTLE_END:
            for i in fixed + deckr:
                d.ctrl[i] = 0.0
        elif state == 'CONVEY':
            for i in fixed + deckr:
                d.ctrl[i] = ROLLER_SPEED
            if beam(rig.BEAM_SEAT_A_X) and beam(rig.BEAM_SEAT_B_X):
                at_release = snap(t)
                state, state_since = 'RELEASE', t
            elif beam(rig.BEAM_FAR_X):
                state, state_since = 'OVERTRAVEL', t
        elif state == 'RELEASE':
            for i in fixed:
                d.ctrl[i] = 0.0
            for i in deckr:
                d.ctrl[i] = ROLLER_SPEED
            if t - state_since > RELEASE_HOLD:
                state, state_since = 'SETTLE', t
        elif state == 'SETTLE':
            for i in fixed + deckr:
                d.ctrl[i] = 0.0
            if t - state_since > SEAT_HOLD:
                at_settle = snap(t)
                done = True
        else:                            # OVERTRAVEL
            for i in fixed + deckr:
                d.ctrl[i] = 0.0
            done = True

        if done:
            break

        mujoco.mj_step(m, d)
        x, z, tilt, yaw = pose()
        if state == 'CONVEY':
            max_yaw_convey = max(max_yaw_convey, abs(yaw))
        if z < 0.40:
            fell = True
        if int(round(t / m.opt.timestep)) % SAMPLE_EVERY == 0:
            trace.append({'t': round(t, 4), 'x_local': round(x - origin, 6), 'z': round(z, 6),
                          'tilt': round(tilt, 4), 'yaw': round(yaw, 4), 'state': state})

    return {
        'world': tag,
        'final_state': state,
        'fell': fell,
        'max_yaw_during_convey': round(max_yaw_convey, 4),
        'at_release': at_release,
        'at_settle': at_settle,
        'yaw_change_over_release_settle': (
            None if not (at_release and at_settle)
            else round(abs(at_settle['yaw']) - abs(at_release['yaw']), 4)),
        'trace_tail': trace[-30:],
    }


def main():
    worlds = []
    xml, _ = rig.build_model(TRAY, receiver=True, guides=False)
    worlds.append(run_world(mujoco.MjModel.from_xml_string(xml), 'standalone guides=False',
                            payload='payload', deck='deck', deck_roller='deck_roller_',
                            fixed_roller='fixed_roller_'))
    xml, _ = rig.build_model(TRAY, receiver=True, guides=True)
    worlds.append(run_world(mujoco.MjModel.from_xml_string(xml), 'standalone guides=True',
                            payload='payload', deck='deck', deck_roller='deck_roller_',
                            fixed_roller='fixed_roller_'))
    worlds.append(run_world(mujoco.MjModel.from_xml_path(str(MERGED)), 'MERGED world_p1_cell.xml',
                            payload='c_payload', deck='c_deck', deck_roller='c_deck_roller_',
                            fixed_roller='c_fixed_roller_'))

    print('=' * 100)
    print('RELEASE / SETTLE -- the yaw the tray is left with once the rollers are cut')
    print('=' * 100)
    print(f"{'world':26s} {'final':11s} {'x@rel':>8s} {'yaw@rel':>8s} {'x@set':>8s} "
          f"{'yaw@set':>8s} {'tilt@set':>9s} {'d|yaw|':>8s} {'fell':>5s}")
    print('-' * 100)
    for w in worlds:
        a, b = w['at_release'] or {}, w['at_settle'] or {}
        ch = w['yaw_change_over_release_settle']
        print(f"{w['world']:26s} {w['final_state']:11s} "
              f"{a.get('x_local', float('nan')):8.4f} {a.get('yaw', float('nan')):8.3f} "
              f"{b.get('x_local', float('nan')):8.4f} {b.get('yaw', float('nan')):8.3f} "
              f"{b.get('tilt', float('nan')):9.3f} "
              f"{(ch if ch is not None else float('nan')):8.3f} {str(w['fell']):>5s}")

    print('\nmax |yaw| while conveying (for scale): '
          + ', '.join(f"{w['world']}={w['max_yaw_during_convey']:.3f}" for w in worlds))

    merged_settled = next((w['at_settle'] or {}).get('yaw')
                          for w in worlds if w['world'].startswith('MERGED'))
    print(f'\noutside review predicted a settled yaw under {PREDICTED_SETTLED_YAW_DEG} deg, which '
          f'would mean the rolling drift does not matter and the guides need not be touched.')
    if merged_settled is None:
        verdict = 'NO SETTLEMENT REACHED -- the tray never seated, so the prediction is untestable here'
    elif abs(merged_settled) < PREDICTED_SETTLED_YAW_DEG:
        verdict = (f'PREDICTION HELD: settled yaw {merged_settled:+.3f} deg is inside '
                   f'{PREDICTED_SETTLED_YAW_DEG} deg')
    else:
        verdict = (f'PREDICTION FAILED: settled yaw {merged_settled:+.3f} deg is NOT inside '
                   f'{PREDICTED_SETTLED_YAW_DEG} deg -- cutting the rollers does not undo the yaw')
    print(f'  => {verdict}')

    out = ROOT / 'reports' / 'p2-c-release-settle-01'
    out.mkdir(parents=True, exist_ok=True)
    (out / 'report.json').write_text(json.dumps({
        'merged_sha256': hashlib.sha256(MERGED.read_bytes()).hexdigest(),
        'roller_speed_rad_s': ROLLER_SPEED,
        'settle_end_s': SETTLE_END, 'release_hold_s': RELEASE_HOLD, 'seat_hold_s': SEAT_HOLD,
        'predicted_settled_yaw_deg': PREDICTED_SETTLED_YAW_DEG,
        'worlds': worlds,
        'verdict': verdict,
    }, indent=1), encoding='utf-8')
    print(f'\nwrote {out}/report.json')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
