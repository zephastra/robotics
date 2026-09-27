"""Attribute the tray's deck yaw: is it C's own mechanism, or the assembly?

WHY THIS FILE EXISTS
--------------------
`reports/p2-c-in-cell-02` (and `D045`) measured that in the ASSEMBLED world the tray rolls straight
along the source rollers (|yaw| <= 0.474 deg) and then develops yaw the moment it crosses onto the
deck, reaching ~10.7 deg by the deck's far end. That measurement cannot say whether the deck yaw is

    (a) inherent to C's rig  -- the deck roller row simply steers the tray, or
    (b) introduced by the assembly -- the deck is now bolted to a chassis.

Only a like-for-like run in C's OWN single-role world separates them. That is this file.

WHY THE COMPARISON IS CLEAN
---------------------------
`roller_rig.build_model` at qpos0 and the merged world differ by an EXACT TRANSLATION: the merged
world's C station is +4.41372 m in x. Measured on both, every marker agrees to the millimetre:

    marker          standalone        merged          delta
    payload           0.200000        4.613720        +4.41372
    deck              1.920000        6.333720        +4.41372
    deck_roller_0     1.910000        6.323720        +4.41372
    deck_roller_7     2.470000        6.883720        +4.41372
    fixed_roller_0_0  0.000000        4.413720        +4.41372
    recv_roller_0     2.790000        7.203720        +4.41372

So all x values are compared in RIG-LOCAL coordinates (x - fixed_roller_0_0.x) and the two runs are
directly comparable. The roller contact friction is also identical in both ([0.25, 0.005, 0.001]).

THE CONTROL, AND WHY THERE IS NO SECOND ARM HERE
------------------------------------------------
`p2-c-in-cell-02` already ran the control arm (every actuator at 0) and got a 0.0007 m drift; that
question is answered. The control inside THIS file is built into the trajectory: the SAME tray,
under the SAME command, on the SAME rig, rolls straight on the source row and then diverges on the
deck. The source row is therefore its own control -- the instrument is demonstrably live (the tray
moves) and demonstrably not inventing yaw (it reads ~0 before the seam). A "no motion" failure mode
is excluded by reporting the advance.

THREE WORLDS, ONE FROZEN VARIABLE EACH
--------------------------------------
    standalone, guides=False   C as its builder's default
    standalone, guides=True    C with the side rails (this is what the merged world carries)
    merged world_p1_cell.xml   the assembly

standalone/guides=True is the like-for-like twin of the merged world; guides=False is included
because the obvious third hypothesis ("the fixed section has side rails and the deck does not") is
directly testable, and measuring it costs one run.

WHAT THIS FILE DOES NOT DO
--------------------------
It does not re-run C's acceptance (108 runs over friction x gap x step x yaw, with an interlock and
a bypass arm). It holds the deck slide, the receiver rollers and the pusher at zero in every world,
so the tray runs off the deck's far end in all three -- that is the declared outcome from `D045`,
and the stage-wise table stops at the last deck crown precisely so it cannot be read as a failure.
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
RUN_S = 15.0
SETTLE_S = 0.40
SAMPLE_EVERY = 25


def _ids(m, prefix):
    return [i for i in range(m.nu)
            if (mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_ACTUATOR, i) or '').startswith(prefix)]


def solver_settings(m):
    """The merged world has ONE solver for four roles -- a documented coupling, so report it."""
    return {
        'timestep': m.opt.timestep,
        'integrator': int(m.opt.integrator),
        'cone': int(m.opt.cone),
        'tolerance': float(m.opt.tolerance),
        'iterations': int(m.opt.iterations),
        'ls_iterations': int(m.opt.ls_iterations),
        'o_solref': [round(float(v), 6) for v in m.opt.o_solref],
        'o_solimp': [round(float(v), 6) for v in m.opt.o_solimp],
    }


def run(model, tag, *, payload='payload', deck_prefix='deck_roller_',
        fixed_prefix='fixed_roller_', recv_prefix='recv_roller_'):
    m = model
    d = mujoco.MjData(m)
    # The merged world's parked chassis lives in its `home` KEYFRAME; qpos0 there is the
    # un-parked pose, where the deck sits ~4.6 m from the fixture and the tray rolls into
    # nothing and flips. C's standalone rig has no keyframe, so qpos0 IS its home. Getting this
    # wrong is silent: the run still completes and still reports numbers -- an earlier version of
    # this file did exactly that and produced a "merged" row of 179.8 deg tilt.
    if m.nkey > 0:
        d.qpos[:] = m.key_qpos[0]
    mujoco.mj_forward(m, d)

    fixed = _ids(m, fixed_prefix)
    deckr = _ids(m, deck_prefix)
    recv = _ids(m, recv_prefix)
    slide = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_ACTUATOR,
                              deck_prefix.replace('deck_roller_', 'deck_drive'))

    tray = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, payload)
    d0 = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, f'{deck_prefix}0')
    dN = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, f'{deck_prefix}{rig.DECK_ROLLERS - 1}')
    fx0 = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, f'{fixed_prefix}0_0')

    mat = np.zeros(9)

    def pose():
        mujoco.mju_quat2Mat(mat, d.xquat[tray])
        tilt = math.degrees(math.acos(max(-1.0, min(1.0, float(mat[8])))))
        yaw = math.degrees(math.atan2(float(mat[3]), float(mat[0])))
        return (float(d.xpos[tray][0]), float(d.xpos[tray][2]), tilt, yaw)

    for _ in range(int(SETTLE_S / m.opt.timestep)):
        mujoco.mj_step(m, d)

    origin = float(d.xpos[fx0][0])            # rig-local frame
    x0, z0, _, _ = pose()
    first_deck = float(d.xpos[d0][0]) - origin
    last_deck = float(d.xpos[dN][0]) - origin

    trace = []
    for k in range(int(RUN_S / m.opt.timestep)):
        for i in fixed + deckr:
            d.ctrl[i] = ROLLER_SPEED
        d.ctrl[slide] = 0.0
        for i in recv:
            d.ctrl[i] = 0.0
        mujoco.mj_step(m, d)
        if k % SAMPLE_EVERY == 0:
            x, z, tilt, yaw = pose()
            trace.append({'t': round(k * m.opt.timestep, 4), 'x_local': round(x - origin, 6),
                          'z': round(z, 6), 'tilt': round(tilt, 4), 'yaw': round(yaw, 4)})

    x, z, tilt_end, yaw_end = pose()

    src = [s for s in trace if s['x_local'] < first_deck]
    dek = [s for s in trace if first_deck <= s['x_local'] <= last_deck]

    def worst(rows, key):
        return round(max((abs(r[key]) if key == 'yaw' else r[key] for r in rows), default=0.0), 4)

    return {
        'world': tag,
        'solver': solver_settings(m),
        'rig_local': {'first_deck_crown': round(first_deck, 6),
                      'last_deck_crown': round(last_deck, 6),
                      'tray_start': round(x0 - origin, 6)},
        'advance_m': round((x - origin) - (x0 - origin), 6),
        'end_x_local': round(x - origin, 6),
        'z_dev_supported': round(max((abs(s['z'] - z0) for s in src + dek), default=0.0), 6),
        'source_samples': len(src),
        'deck_samples': len(dek),
        'source_max_abs_yaw': worst(src, 'yaw'),
        'deck_max_abs_yaw': worst(dek, 'yaw'),
        'source_max_tilt': worst(src, 'tilt'),
        'deck_max_tilt': worst(dek, 'tilt'),
        'trace': trace,
    }


def main():
    worlds = []

    xml, _meta = rig.build_model(TRAY, receiver=True, guides=False)
    worlds.append(run(mujoco.MjModel.from_xml_string(xml),
                      'standalone guides=False'))

    xml, _meta = rig.build_model(TRAY, receiver=True, guides=True)
    worlds.append(run(mujoco.MjModel.from_xml_string(xml),
                      'standalone guides=True'))

    worlds.append(run(mujoco.MjModel.from_xml_path(str(MERGED)),
                      'MERGED world_p1_cell.xml',
                      payload='c_payload', deck_prefix='c_deck_roller_',
                      fixed_prefix='c_fixed_roller_', recv_prefix='c_recv_roller_'))

    print('=' * 92)
    print('YAW ATTRIBUTION -- same tray, same command, same rig-local geometry; only the WORLD differs')
    print('=' * 92)
    hdr = f"{'world':26s} {'advance':>9s} {'src n':>6s} {'deck n':>7s} {'src |yaw|':>10s} " \
          f"{'deck |yaw|':>11s} {'src tilt':>9s} {'deck tilt':>10s}"
    print(hdr)
    print('-' * len(hdr))
    for w in worlds:
        print(f"{w['world']:26s} {w['advance_m']:9.4f} {w['source_samples']:6d} "
              f"{w['deck_samples']:7d} {w['source_max_abs_yaw']:10.4f} "
              f"{w['deck_max_abs_yaw']:11.4f} {w['source_max_tilt']:9.4f} {w['deck_max_tilt']:10.4f}")

    print('\nsolver settings per world (the merged world must share ONE for four roles):')
    for w in worlds:
        s = w['solver']
        print(f"  {w['world']:26s} tol {s['tolerance']:.2e} it {s['iterations']:<4d} "
              f"ls {s['ls_iterations']:<4d} cone {s['cone']} integ {s['integrator']} "
              f"dt {s['timestep']}")

    base = worlds[1]['deck_max_abs_yaw']      # the like-for-like twin of the merged world
    merged = worlds[2]['deck_max_abs_yaw']
    src_base = worlds[1]['source_max_abs_yaw']
    verdict = ('C ITSELF YAWS ON THE DECK -- not an assembly effect'
               if base > 5.0 else
               'ASSEMBLY INTRODUCED THE DECK YAW -- C alone does not do this')
    print(f"\nlike-for-like twins (both carry the guide rails):")
    print(f"  standalone deck |yaw| {base:.4f} deg   merged deck |yaw| {merged:.4f} deg")
    print(f"  standalone source |yaw| {src_base:.4f} deg")
    print(f"\n=> {verdict}")

    out = ROOT / 'reports' / 'p2-c-yaw-attrib-01'
    out.mkdir(parents=True, exist_ok=True)
    (out / 'report.json').write_text(json.dumps({
        'merged_sha256': hashlib.sha256(MERGED.read_bytes()).hexdigest(),
        'roller_speed_rad_s': ROLLER_SPEED,
        'run_s': RUN_S,
        'worlds': worlds,
        'like_for_like': {'standalone_guides_true_deck_yaw': base,
                          'merged_deck_yaw': merged,
                          'standalone_source_yaw': src_base},
        'verdict': verdict,
    }, indent=1), encoding='utf-8')
    print(f'\nwrote {out}/report.json')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
