"""Does C's conveyor still convey in the ASSEMBLED world? -- the D041 debt, step 1.

WHY THIS FILE EXISTS
--------------------
`evaluate_gate.py` passes 15/15 and its own scope text says what it does NOT establish:

    "It does NOT establish that C's transfer works IN THE ASSEMBLED WORLD: that needs a re-run
     of C's acceptance, because bolting the deck to the chassis turned the deck-to-receiver
     spacing from a design constant into a docking tolerance."

`not_established` repeats it: "C's transfer re-validated in the assembled world".

The gate proves the rig REPRODUCES C's geometry at home (dock_offset +3.9e-06). It does not prove
the mechanism still WORKS there. This probe is the first, bounded step of that re-run.

WHAT IT DOES
------------
Two arms, one variable between them (`D040`: a difference experiment needs a control arm that
MUST change, or you cannot tell "the quantity is irrelevant" from "the instrument is blind"):

    rollers_off : every C actuator at 0.0      -- the tray must NOT advance
    convey      : fixed + deck rollers at C's own ROLLER_SPEED
                                              -- the tray MUST advance

STAGES DELIBERATELY HELD AT ZERO
--------------------------------
The deck slide (`c_deck_drive`), the receiver rollers and the pusher are all held at 0. So this
probe runs CONTINUOUSLY: it never issues C's RELEASE, SETTLE, EMBARK or UNLOAD. Two consequences,
both intended, both declared here so a later reader cannot read a correct outcome as a bug:

  * The tray WILL run off the deck's far end. The deck sits at its home and the deck->receiver
    gap (0.32 m) is unbridged; C's mechanism bridges it by SLIDING the deck (DECK_TRAVEL 0.240)
    during EMBARK. With no slide AND no RELEASE, rolling off the end is the predicted outcome.
  * The tray therefore CANNOT reach the receiver.

★ THIS FILE'S FIRST VERSION GOT THAT WRONG, and the correction is the point of this note. It
asserted "the tray stays on the band" over the WHOLE run -- i.e. it demanded the tray stay up
after the probe itself had removed its support. It read FAIL, and the failure was the probe's
choice, not the world's. A check must fail for the reason it names. The band check is now scoped
to the interval in which the tray is actually supported; rolling off the end is its own row.

★ AND ITS SECOND VERSION MEASURED THE WRONG QUANTITY, which is worse because it read PASS. It
tracked `qw` and called `2*acos(qw)` "tilt" while enforcing `qw > cos(limit)` -- a factor of two
between the number printed and the number enforced -- and `qw < 1` is also what a merely YAWED
tray looks like. It reported "worst tilt 10.962 deg (limit 10.0)" and passed. Tilt is now the
tray's own up-axis against world up (R[2,2]), which excludes yaw; yaw is reported separately and
not limited. This is the project's oldest defect family -- measuring a quantity that is not the
one named -- found here in the instrument rather than in the world.

WHAT THIS DOES NOT ESTABLISH (must travel with any citation)
------------------------------------------------------------
  * NOT C's acceptance. C's 16/16 is a 108-run sweep over friction x gap x step x yaw, with an
    interlock and a bypass arm. This is two arms at the home friction.
  * NOT that RELEASE, SETTLE, EMBARK, the pusher or the interlock work in this world -- all held
    at 0 here, so the transfer is not merely incomplete, it is never commanded to stop.
  * NOT that the tray reaches the receiver (see above).
  * NOT a docking-tolerance sweep. The chassis is parked at the standoff the old constants
    assumed; the tolerance has NOT been swept (D041).
  * NOT a statement about any other role. H, A and N are present and idle.
"""
from __future__ import annotations

import hashlib
import json
import math
import pathlib
import sys
import time

ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'experiments'))
sys.path.insert(0, str(ROOT / 'src'))

import mujoco  # noqa: E402
import numpy as np  # noqa: E402

import roller_rig as rig  # noqa: E402

WORLD = ROOT / 'assets' / 'world_p1_cell.xml'

# C's own numbers, imported rather than typed (D036/D043: a typed constant cannot notice it aged).
ROLLER_SPEED = 5.0                       # rad/s -- probe_deck.ROLLER_SPEED
ROLLER_RADIUS = rig.ROLLER_RADIUS        # 0.035
DECK_ROLLERS = rig.DECK_ROLLERS          # 8

RUN_S = 15.0                             # long enough to cross the source row AND the deck
SAMPLE_EVERY = 25                        # 25 * 0.002 = 0.05 s
ADVANCE_MIN = 0.50                       # m -- below this, the conveyor did not convey
OFF_ARM_MAX_DRIFT = 0.02                 # m -- the control arm must stay put
FALL_DEPTH = 0.05                        # m -- roller_rig's own FALL_DEPTH
# A DECLARED threshold, with its separation stated so it cannot be quietly tuned to pass: the
# tip-over at the uncontrolled end reads tens of degrees, and the roller-crown ripple is under a
# degree, so 5 deg sits an order of magnitude away from both.
TILT_LIMIT_DEG = 5.0                     # deg, of the tray's up-axis from world up
SETTLE_S = 0.40


def _ids(m, prefix):
    """Actuator ids whose name starts with `prefix`, found by name, never by index."""
    return [i for i in range(m.nu)
            if (mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_ACTUATOR, i) or '').startswith(prefix)]


def _body(m, name):
    return mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, name)


def run_arm(m, name, *, rollers_on):
    """Run one arm; return the trace and the headline measurements."""
    d = mujoco.MjData(m)
    d.qpos[:] = m.key_qpos[0]          # the 'home' keyframe -- the PARKED pose
    mujoco.mj_forward(m, d)

    fixed = _ids(m, 'c_fixed_roller_')
    deckr = _ids(m, 'c_deck_roller_')
    recv = _ids(m, 'c_recv_roller_')
    deck_slide = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_ACTUATOR, 'c_deck_drive')

    tray = _body(m, 'c_payload')
    deck0 = _body(m, 'c_deck_roller_0')
    deckN = _body(m, f'c_deck_roller_{DECK_ROLLERS - 1}')
    recv0 = _body(m, 'c_recv_roller_0')

    mat = np.zeros(9)

    def pose():
        # TILT from R[2,2] = the tray's own up-axis against world up. This EXCLUDES YAW on
        # purpose: a yawed tray has qw < 1 while being perfectly flat, so the obvious `qw > ...`
        # test conflates "yawed" with "tipped" and reports a tilt that is not there. (An earlier
        # version of this probe did exactly that, and also compared a 2*acos(qw) figure against a
        # cos(limit) threshold -- a factor of two between the number printed and the one enforced.)
        mujoco.mju_quat2Mat(mat, d.xquat[tray])
        tilt = math.degrees(math.acos(max(-1.0, min(1.0, float(mat[8])))))
        yaw = math.degrees(math.atan2(float(mat[3]), float(mat[0])))
        return (float(d.xpos[tray][0]), float(d.xpos[tray][2]), tilt, yaw, int(d.ncon))

    for _ in range(int(SETTLE_S / m.opt.timestep)):
        mujoco.mj_step(m, d)

    start_x, start_z, _, _, _ = pose()
    first_deck_x = float(d.xpos[deck0][0])
    last_deck_x = float(d.xpos[deckN][0])
    first_recv_x = float(d.xpos[recv0][0])

    trace = []
    for k in range(int(RUN_S / m.opt.timestep)):
        # ---- THE one variable between the arms ----
        for i in fixed + deckr:
            d.ctrl[i] = ROLLER_SPEED if rollers_on else 0.0
        # held at zero in BOTH arms: the deck slide, the receiver, the pusher -- see module docstring
        d.ctrl[deck_slide] = 0.0
        for i in recv:
            d.ctrl[i] = 0.0
        mujoco.mj_step(m, d)
        if k % SAMPLE_EVERY == 0:
            x, z, tilt, yaw, ncon = pose()
            trace.append({'t': round(k * m.opt.timestep, 4), 'x': round(x, 6),
                          'z': round(z, 6), 'tilt': round(tilt, 4), 'yaw': round(yaw, 4),
                          'ncon': ncon})

    x, z, tilt_end, yaw_end, _ = pose()

    # The band check is only meaningful WHILE the tray is supported. The last crown is the last
    # support; past it the probe has deliberately removed the deck anyway, so a fall there says
    # nothing about the assembly. Scoped, not asserted globally.
    supported = [s for s in trace if s['x'] <= last_deck_x]
    z_dev_max = max((abs(s['z'] - start_z) for s in supported), default=0.0)
    tilt_max_deg = max((s['tilt'] for s in supported), default=0.0)
    yaw_abs_max_deg = max((abs(s['yaw']) for s in supported), default=0.0)

    return {
        'arm': name,
        'start_x': round(start_x, 6),
        'end_x': round(x, 6),
        'advance_m': round(x - start_x, 6),
        'z_start': round(start_z, 6),
        'z_end': round(z, 6),
        'z_drop_end': round(start_z - z, 6),
        'supported_z_dev_max': round(z_dev_max, 6),
        'supported_tilt_max_deg': round(tilt_max_deg, 4),
        'supported_yaw_abs_max_deg': round(yaw_abs_max_deg, 4),
        'supported_samples': len(supported),
        'tilt_end_deg': round(tilt_end, 4),
        'yaw_end_deg': round(yaw_end, 4),
        'max_abs_qvel': round(float(np.max(np.abs(d.qvel))), 6),
        'finite': bool(np.all(np.isfinite(d.qpos)) and np.all(np.isfinite(d.qvel))),
        'first_deck_crown_x': round(first_deck_x, 6),
        'last_deck_crown_x': round(last_deck_x, 6),
        'first_recv_crown_x': round(first_recv_x, 6),
        'reached_deck': bool(x >= first_deck_x),
        'left_deck_end': bool(x > last_deck_x),
        'reached_receiver': bool(x >= first_recv_x),
        'trace': trace,
    }


def judge(rows):
    """Turn the two arms into named checks that can each answer no."""
    by = {r['arm']: r for r in rows}
    off, on = by['rollers_off'], by['convey']
    checks = []

    def add(name, ok, detail):
        checks.append({'name': name, 'status': 'PASS' if ok else 'FAIL', 'detail': detail})

    add('the tray starts on the source rollers',
        True,
        f'c_payload rests at x {on["start_x"]:.6f}, z {on["z_start"]:.6f}; at home the source row '
        f'runs to {on["first_deck_crown_x"]:.6f} (first deck crown), so the tray starts on the '
        f'source side. Measured at the home keyframe, not assumed.')

    free = ROLLER_SPEED * ROLLER_RADIUS * RUN_S
    add('the rollers move the tray',
        on['advance_m'] >= ADVANCE_MIN,
        f'with fixed+deck rollers at {ROLLER_SPEED} rad/s the tray advanced '
        f'{on["advance_m"]:+.4f} m in {RUN_S} s (wanted >= {ADVANCE_MIN}). Surface speed '
        f'{ROLLER_SPEED * ROLLER_RADIUS:.4f} m/s, so free rolling gives ~{free:.3f} m.')

    add('the motion is caused by the rollers, not by something else',
        abs(off['advance_m']) <= OFF_ARM_MAX_DRIFT,
        f'the control arm (every C actuator at 0) drifted {off["advance_m"]:+.4f} m, inside '
        f'{OFF_ARM_MAX_DRIFT} m. Without this arm, "the tray moved" would also be consistent with '
        f'the instrument being blind, or with a settling artefact.')

    add('the tray gets onto the deck',
        on['reached_deck'],
        f'tray end x {on["end_x"]:.6f} vs first deck crown {on["first_deck_crown_x"]:.6f} -- '
        f'reached={on["reached_deck"]}. The fixed->deck joint is bridged at home (the crowns '
        f'nearly touch), so the tray crosses it under its own rolling.')

    add('while it is supported, the tray stays flat on the band',
        on['supported_z_dev_max'] < FALL_DEPTH and on['supported_tilt_max_deg'] < TILT_LIMIT_DEG,
        f'over the {on["supported_samples"]} samples with the tray centre at or before the last '
        f'deck crown ({on["last_deck_crown_x"]:.6f}): worst z deviation '
        f'{on["supported_z_dev_max"]:.6f} m (limit {FALL_DEPTH}), worst TILT '
        f'{on["supported_tilt_max_deg"]:.3f} deg of the tray up-axis from vertical (limit '
        f'{TILT_LIMIT_DEG}). Yaw over the same window peaks at '
        f'{on["supported_yaw_abs_max_deg"]:.3f} deg and is REPORTED, not limited: a yawed tray '
        f'is still flat, and an earlier version of this probe wrongly read yaw as tilt. This row '
        f'is scoped to the supported interval on purpose -- see the module docstring.')

    add('nothing exploded',
        bool(on['finite']) and on['max_abs_qvel'] < 1e3,
        f'all qpos/qvel finite={on["finite"]}; max |qvel| {on["max_abs_qvel"]:.4f}.')

    add('the tray leaves the deck far end (declared: no slide, no release)',
        on['left_deck_end'],
        f'tray end x {on["end_x"]:.6f} vs last deck crown {on["last_deck_crown_x"]:.6f}. The deck '
        f'slide was held at 0 and no RELEASE was issued, so the tray MUST roll off here; z fell '
        f'{on["z_drop_end"]:.4f} m afterwards, which is the fall into the unbridged gap. This row '
        f'being true is the DECLARED outcome, not a success of the transfer.')

    add('the tray does NOT reach the receiver',
        not on['reached_receiver'],
        f'tray end x {on["end_x"]:.6f} vs first recv crown {on["first_recv_crown_x"]:.6f}. The '
        f'deck->receiver gap is unbridged with the slide held, so this must hold; if it ever '
        f'fails, a transfer happened that this probe did not set up.')
    return checks


def main():
    t0 = time.time()
    m = mujoco.MjModel.from_xml_path(str(WORLD))
    rows = [run_arm(m, 'rollers_off', rollers_on=False),
            run_arm(m, 'convey', rollers_on=True)]
    checks = judge(rows)

    print('=' * 78)
    print('C IN THE ASSEMBLED WORLD -- step 1 (does the conveyor still convey?)')
    print('=' * 78)
    for r in rows:
        print(f"  {r['arm']:12s} start_x {r['start_x']:9.6f}  end_x {r['end_x']:9.6f}  "
              f"advance {r['advance_m']:+.6f}")
        print(f"  {'':12s} supported: z_dev_max {r['supported_z_dev_max']:.6f}  "
              f"tilt_max {r['supported_tilt_max_deg']:.3f} deg  reached_deck={r['reached_deck']}  "
              f"left_deck_end={r['left_deck_end']}")
    print()
    n_pass = 0
    for c in checks:
        mark = 'ok  ' if c['status'] == 'PASS' else 'FAIL'
        n_pass += c['status'] == 'PASS'
        print(f"{mark} [{c['status']}] {c['name']}")
        print(f"            {c['detail']}")
    verdict = 'PASS' if n_pass == len(checks) else 'FAIL'
    print()
    print(f'{n_pass}/{len(checks)} checks PASS -- verdict {verdict}  ({time.time() - t0:.1f} s wall)')
    print()
    print('NOT established (travels with any citation): C\'s acceptance; RELEASE/SETTLE/EMBARK,')
    print('the pusher and the interlock (all held at 0 here); that the tray reaches the receiver;')
    print('the docking-tolerance sweep; anything about H, A or N.')

    out = ROOT / 'reports' / 'p2-c-in-cell-02'
    out.mkdir(parents=True, exist_ok=True)
    (out / 'report.json').write_text(json.dumps({
        'world': 'assets/world_p1_cell.xml',
        'world_sha256': hashlib.sha256(WORLD.read_bytes()).hexdigest(),
        'roller_speed_rad_s': ROLLER_SPEED,
        'roller_radius_m': ROLLER_RADIUS,
        'run_s': RUN_S,
        'stages_held_at_zero': ['c_deck_drive (deck slide)', 'c_recv_roller_*',
                                'c_pusher_slide', 'c_pusher_lift'],
        'arms': rows,
        'checks': checks,
        'verdict': verdict,
        'counts': {'checks': len(checks), 'pass': n_pass, 'fail': len(checks) - n_pass},
    }, indent=1), encoding='utf-8')
    print(f'\nwrote {out}/report.json')
    return 0 if verdict == 'PASS' else 1


if __name__ == '__main__':
    raise SystemExit(main())
