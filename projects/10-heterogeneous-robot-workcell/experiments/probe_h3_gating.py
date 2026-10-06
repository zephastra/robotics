#!/usr/bin/env python3
"""Gating audit: what is still allowed to move after a handoff is REFUSED?

WHY THIS EXISTS
---------------
The accepted negative H3 run (`reports/p4-h3-w5-neg-01`) records that every transfer and verify step
REFUSED, yet the tray travelled 0.1924 m. That report's own `chain_rows` localises the motion: not
spread over the chain, but **0.18071 m of it (94%) inside `START_TRANSFER`** -- the very command that
then reports `FAILED / TRANSFER_TIMEOUT`. So "refused" does not mean "nothing moved": the refusal is
a TIMEOUT THAT FIRES AFTER THE ACTUATION, not a gate that precedes it.

That is the question a reviewer raised before we wire the arm in: *after a handoff is refused, which
mechanisms are still allowed to act, and what state does the refusal leave behind?*

WHAT IS MEASURED
----------------
1. `tray_motion_is_the_band` -- the tray moves under `transfer()` while its SUPPORT is unchanged, so
   the rollers moved it and the source band still carries it.
2. `refusal_is_a_timeout_after_the_motion` -- the distance scales with the timeout at a constant
   rate, which is what "spin the band for a bounded time" predicts, and it is the command that
   reports `FAILED` that produced the motion.
3. `aborted_transfer_reports_the_abort` -- the abort returns `TRANSFERRING`, not `RECEIVED`.
4. `nothing_left_commanded` -- every actuator is back on the plant's hold value.
5. `pusher_state_is_observable` -- **FOUND BROKEN, THEN FIXED (2026-09-29).**
   `LogisticsPlant.pusher_position()` looked the joints up as `pusher_lift_joint` /
   `pusher_slide_joint`, but the merged world names them `c_pusher_lift_joint` /
   `c_pusher_slide_joint`. `mj_name2id` returned -1, the branch was skipped, and the function
   returned `{}` SILENTLY -- and `transfer()` handed that empty dict back as its own `pusher` field,
   so the retention device's state was never written into any report. Same family as `D106`'s
   `runtime_qpos_writes = null` and the `TransferLedger` that was built and never driven: a field
   that is reported but never actually filled. The mechanism was always measurable; the READBACK
   was dead. Fixed by reaching the joints through the pusher's own VALIDATED actuators, by
   ownership, rather than by hard-coding the `c_` prefix (which is the same fragility one rename
   away). This row is what caught it and is what verifies the fix.
6. `the_retention_blade_can_reach_its_command` -- **FAILS.** The lift joint's whole range is 1 mm
   (`[0, 0.001]`). Commanded its full value and allowed to settle, it sits at **-0.000215 m**, i.e.
   BELOW its own lower limit, and the blade body rises 0.12 mm. So the step the module documents as
   "the blade goes UP before anything is asked to move: it is what stops the tray walking back off
   the deck" is not achievable by the mechanism as built. The retention claim is therefore not
   supported by a deployment, only asserted.

A HYPOTHESIS THIS FILE REFUTED, KEPT ON THE RECORD
--------------------------------------------------
Reading `transfer()` suggested a second hazard: the blade is lowered only `if direction ==
'onto_receiver'`, while the invariant the module's own comment states is about an EMPTY DECK
("a retention device left deployed on an empty deck would block the next load"). An aborted
`onto_deck` should therefore leave the blade up over nothing. **Measurement refuted it**: the blade
never rises in the first place, so it cannot be left raised. The branch is still keyed on the
attempt's direction rather than on whether the deck received a tray -- a latent inconsistency -- but
it is inert while finding 6 stands. Row 7 records the measurement that refutes it.

A DEFECT IN THIS FILE'S OWN FIRST VERSION, ALSO KEPT ON THE RECORD
------------------------------------------------------------------
The first version scored `no_blade_left_up_on_an_empty_deck` as a PASS on a comparison it could not
make: the "before" lift was `0.0`, and the message used `x or float('nan')`, so `0.0` printed as
`nan` and the guard `is not None` waved the row through. **A row that cannot fail is worse than no
row.** It is recorded here because it is the same defect family this whole audit is about.

WHAT THIS FILE DOES NOT DO
--------------------------
It does not rerun the chain and does not touch `probe_h3_w5.py`, whose hash the accepted reports
cite. It reconstructs the abort by calling `LogisticsPlant.transfer()` with a short timeout, so the
finding reproduces in seconds and the accepted evidence stays valid.

DECLARED, NOT FROZEN. This is a diagnostic: it measures and reports. Turning any of it into a judged
H3 row is a separate decision, recorded rather than assumed.
"""
import argparse
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

import w4_plant as wp  # noqa: E402

WORLD = ROOT / 'assets' / 'world_w5_h085_loop.xml'
TRAY = 'c_payload'
BLADE = 'c_pusher_blade'
LIFT_JOINT = 'c_pusher_lift_joint'
SLIDE_JOINT = 'c_pusher_slide_joint'

#: The tray pose the negative H3 run left at the handoff (`tray_at_handoff` in that report). Using
#: the recorded value rather than an invented one is what makes this an audit of THAT abort.
HANDOFF_XYZ = (4.368744467296106, 0.00016008128176809574, 0.8499474925330587)

#: DECLARED. The real run spent 21.5 s inside `START_TRANSFER` and still did not cross; two short
#: timeouts are used so the rate can be shown to be constant, which is what identifies the mechanism.
TIMEOUTS_S = (3.0, 6.0)
SETTLE_STEPS = 600


def joint_qpos(model, data, name):
    jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
    if jid < 0:
        raise SystemExit('REFUSED: the world has no joint named %r' % name)
    return float(data.qpos[int(model.jnt_qposadr[jid])])


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--run-id', default='p4-h3-gating-01')
    args = ap.parse_args(argv)

    out = ROOT / 'reports' / args.run_id
    if out.exists() and any(out.iterdir()):
        raise SystemExit('REFUSED: %s already holds evidence; pick a new --run-id' % out)
    out.mkdir(parents=True, exist_ok=True)

    started = time.monotonic()
    report = {
        'probe': 'gating: what still acts after a REFUSED handoff',
        'world': WORLD.name,
        'world_sha256': hashlib.sha256(WORLD.read_bytes()).hexdigest(),
        'timeouts_s': list(TIMEOUTS_S),
        'tray_at_handoff': list(HANDOFF_XYZ),
        'status': 'ERROR',
    }

    model = mujoco.MjModel.from_xml_path(str(WORLD))
    data = mujoco.MjData(model)
    home_qpos = np.array(model.key_qpos[0], dtype=float)
    data.qpos[:] = home_qpos
    mujoco.mj_forward(model, data)

    plant = wp.LogisticsPlant(world=str(WORLD), model=model, data=data)
    residual = plant.dock_residuals('station_c')['longitudinal_m']
    if abs(residual) > 0.6:
        raise SystemExit('plant derived its geometry from a non-home state (%.4f m)' % residual)

    tbody = model.body(TRAY).id
    jid = int(model.body_jntadr[tbody])
    qadr = int(model.jnt_qposadr[jid])
    dofadr = int(model.jnt_dofadr[jid])
    blade_body = model.body(BLADE).id
    lift_full = float(plant.pusher_limits['lift'][1])
    report['declared'] = {
        'lift_full_m': lift_full,
        'lift_ctrlrange': list(plant.pusher_limits['lift']),
        'slide_ctrlrange': list(plant.pusher_limits['slide']),
        'lift_joint_range': [float(v) for v in model.jnt_range[
            mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, LIFT_JOINT)]],
        'world_lift_joint': LIFT_JOINT,
        'api_looks_up': 'pusher_lift_joint',
    }

    def put_tray_at_handoff():
        data.qpos[:] = home_qpos
        data.qvel[:] = 0.0
        data.qpos[qadr:qadr + 3] = HANDOFF_XYZ
        data.qpos[qadr + 3:qadr + 7] = [1.0, 0.0, 0.0, 0.0]
        mujoco.mj_forward(model, data)

    def snapshot():
        ts = plant.tray_state()
        return {
            'tray_x_m': float(ts['x_m']),
            'tray_zone': ts['zone'],
            'support': list(plant.tray_support()),
            'lift_qpos_m': joint_qpos(model, data, LIFT_JOINT),
            'slide_qpos_m': joint_qpos(model, data, SLIDE_JOINT),
            'blade_z_m': float(data.xpos[blade_body][2]),
            'pusher_position_api': plant.pusher_position(),
        }

    # -- 1. the blade's response to its own command (findings 5 and 6) --------------------------
    def_blade = {}
    for label, lift in (('commanded_down', 0.0), ('commanded_full', lift_full)):
        put_tray_at_handoff()
        plant.deploy_pusher(lift=lift, slide=0.0)
        hold = plant._hold()
        for _ in range(SETTLE_STEPS):
            data.ctrl[:] = hold
            mujoco.mj_step(model, data)
        def_blade[label] = {
            'lift_command': lift,
            'lift_qpos_m': joint_qpos(model, data, LIFT_JOINT),
            'slide_qpos_m': joint_qpos(model, data, SLIDE_JOINT),
            'blade_z_m': float(data.xpos[blade_body][2]),
        }
    report['blade_deployment'] = def_blade
    blade_travel = (def_blade['commanded_full']['blade_z_m']
                    - def_blade['commanded_down']['blade_z_m'])

    # -- 2. the abort, at two timeouts, to show the rate ----------------------------------------
    aborts = []
    for timeout_s in TIMEOUTS_S:
        put_tray_at_handoff()
        before = snapshot()
        left = plant.transfer(direction='onto_deck', timeout_s=timeout_s)
        after = snapshot()
        ctrl = np.asarray(data.ctrl, dtype=float)
        hold = np.asarray(plant._hold(), dtype=float)
        off = np.flatnonzero(np.abs(ctrl - hold) > 1e-12)
        aborts.append({
            'timeout_s': timeout_s,
            'returned_state': left['state'],
            'driven_actuators': left['driven_actuators'],
            'roller_speed': left['roller_speed'],
            'seconds': left['end_s'] - left['start_s'],
            'before': before, 'after': after,
            'tray_dx_m': after['tray_x_m'] - before['tray_x_m'],
            'rate_mps': ((after['tray_x_m'] - before['tray_x_m'])
                         / max(1e-9, left['end_s'] - left['start_s'])),
            #: `seconds` above includes the fixed seating stroke that runs AFTER the timeout, during
            #: which the band is also spun. Normalising by elapsed therefore dilutes the short case.
            #: The mechanism is "spin for `timeout_s`", so the timeout is the right normaliser for
            #: the time-driven claim. Both are reported so the difference is visible.
            'rate_over_timeout_mps': ((after['tray_x_m'] - before['tray_x_m'])
                                      / max(1e-9, timeout_s)),
            'n_actuators_differing_from_hold': int(off.size),
            'leftover_names': [mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, int(i))
                               for i in off],
            'pusher_returned_by_transfer': left['pusher'],
        })
    report['aborts'] = aborts

    # -- the rows --------------------------------------------------------------------------------
    a1, a2 = aborts
    rates = [a['rate_over_timeout_mps'] for a in aborts]
    rate_spread = abs(rates[0] - rates[1]) / max(rates)
    api_complete = (isinstance(a1['after']['pusher_position_api'], dict)
                    and set(a1['after']['pusher_position_api']) == {'lift', 'slide'})
    lift_reached = abs(a1['after']['lift_qpos_m'] - lift_full) < 1e-6

    rows = []

    def add(name, ok, detail):
        rows.append({'name': name, 'status': 'PASS' if ok else 'FAIL', 'detail': detail})

    add('tray_motion_is_the_band',
        a1['tray_dx_m'] > 0.005 and a1['after']['support'] == a1['before']['support'],
        'the tray moved %+.5f m in x under transfer() while its SUPPORT was unchanged at %s, so the '
        'rollers moved it and the source band still carried it'
        % (a1['tray_dx_m'], a1['after']['support']))

    add('the_motion_is_time_driven_not_a_delivery',
        rate_spread < 0.10 and a1['tray_dx_m'] > 0.005 and a2['tray_dx_m'] > a1['tray_dx_m'],
        'timeout %.1f s -> tray %+.5f m; timeout %.1f s -> tray %+.5f m. Per second OF TIMEOUT that '
        'is %.5f and %.5f m, a spread of %.1f%%, so the distance is the timeout multiplied by a band '
        'rate and it grows when the timeout grows. It is not a delivered object: the command '
        'returned %r. (Measured elapsed is %.2f / %.2f s, because a fixed seating stroke runs after '
        'the timeout and spins the band too -- which is why elapsed is the wrong normaliser.)'
        % (a1['timeout_s'], a1['tray_dx_m'], a2['timeout_s'], a2['tray_dx_m'],
           rates[0], rates[1], rate_spread * 100, a1['returned_state'],
           a1['seconds'], a2['seconds']))

    add('aborted_transfer_reports_the_abort',
        a1['returned_state'] == 'TRANSFERRING' and 'deck' not in a1['after']['support'],
        'an aborted onto_deck returns state %r (not RECEIVED) and the deck holds a tray=%s, so the '
        'caller is told it failed'
        % (a1['returned_state'], 'deck' in a1['after']['support']))

    add('nothing_left_commanded',
        a1['n_actuators_differing_from_hold'] == 0,
        'after the call %d of %d actuators differ from the plant hold value%s'
        % (a1['n_actuators_differing_from_hold'], int(model.nu),
           (' -- %s' % a1['leftover_names']) if a1['leftover_names'] else ''))

    add('pusher_state_is_observable',
        api_complete,
        'pusher_position() returned %r, and transfer() handed %r back as its own `pusher` field. '
        'A complete readback must carry BOTH keys; the world names the joints %s / %s while the '
        'API historically looked up %s / %s, which returned -1 and produced a silent {} that '
        'travelled into every report as a field with no value. The blade is measurable directly '
        '(lift qpos %.6f m)'
        % (a1['after']['pusher_position_api'], a1['pusher_returned_by_transfer'],
           LIFT_JOINT, SLIDE_JOINT, 'pusher_lift_joint', 'pusher_slide_joint',
           a1['after']['lift_qpos_m']))

    add('the_retention_blade_can_reach_its_command',
        lift_reached,
        'commanded down (0.000000) the lift joint sits at %.6f m; commanded its full %0.6f m it sits '
        'at %.6f m -- i.e. %.6f m BELOW its own lower limit of %.6f -- and the blade body moves '
        '%.6f m (%.2f mm) in z. The module documents this step as "the blade goes UP before anything '
        'is asked to move: it is what stops the tray walking back off the deck", so the retention '
        'claim rests on a deployment the mechanism does not perform'
        % (def_blade['commanded_down']['lift_qpos_m'], lift_full,
           def_blade['commanded_full']['lift_qpos_m'],
           def_blade['commanded_full']['lift_qpos_m'], report['declared']['lift_joint_range'][0],
           blade_travel, blade_travel * 1000))

    add('blade_is_not_left_raised_over_an_empty_deck',
        not ('deck' not in a1['after']['support'] and lift_reached),
        'the REFUTED hypothesis, kept as a measurement: an aborted onto_deck leaves the deck empty '
        '(tray on %s) and the blade was commanded up, but the blade never reaches its command '
        '(qpos %.6f vs %.6f), so it cannot be left raised. The branch that lowers the blade is still '
        'keyed on direction == onto_receiver rather than on whether the deck received a tray; that '
        'inconsistency is latent while the lift is inoperative'
        % (a1['after']['support'], a1['after']['lift_qpos_m'], lift_full))

    report['rows'] = rows
    report['counts'] = {'pass': sum(1 for r in rows if r['status'] == 'PASS'),
                        'fail': sum(1 for r in rows if r['status'] == 'FAIL')}
    report['verdict'] = ('PASS: all %d gating rows' % len(rows)
                         if report['counts']['fail'] == 0
                         else 'FAIL: %d of %d gating rows'
                              % (report['counts']['fail'], len(rows)))
    report['wall_s'] = time.monotonic() - started
    report['status'] = 'DIAGNOSTIC_COMPLETED'

    (out / 'report.json').write_text(json.dumps(report, indent=2) + '\n', encoding='utf-8')

    for r in rows:
        print('%-4s [%-4s] %-42s %s' % ('ok' if r['status'] == 'PASS' else 'FAIL',
                                        r['status'], r['name'], r['detail']))
    print()
    print(report['verdict'])
    print('world %s (%s)' % (report['world'], report['world_sha256'][:16]))
    print('report -> %s' % (out / 'report.json'))
    return 0 if report['counts']['fail'] == 0 else 1


if __name__ == '__main__':
    raise SystemExit(main())
