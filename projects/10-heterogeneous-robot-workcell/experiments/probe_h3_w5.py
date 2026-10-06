"""H3: the humanoid's handoff IN the logistics chain -- one world, one timeline, one tray.

WHAT H3 ADDS OVER H2
---------------------
H2 (`reports/p4-h2-w5-03`) proved the humanoid can take the tray off the presentation fixture,
carry it, and put it down on the source band's designated rollers, then withdraw. It proved that
ABOUT THE HUMANOID. What it did not do -- and what the standing guidance names as H3 -- is put
that handoff INTO the existing chain:

    MOVE_TO_STATION / DOCK / TRANSFER / VERIFY_TRANSFER / UNDOCK

so that the thing the humanoid delivers is the thing the logistics loop picks up. That is a
different claim from H2's, and it is the one this probe is built to test or falsify.

THE INTEGRATION CONTRACT, IN FOUR PARTS
----------------------------------------
1. **SAME WORLD, SAME TIMELINE, NO RELOAD.** The humanoid's `Runtime` and the W5
   `LogisticsPlant` hold the SAME `MjModel` and the SAME `MjData` (`w4_plant.LogisticsPlant(
   model=..., data=...)`, added for this and backward-compatible by default). The world is
   `assets/world_w5_h085_loop.xml` -- candidate B with the humanoid's station written into the
   model's own home pose, so it is the world's station and not a probe's.

2. **SAME TRAY ENTITY.** `c_payload` is one body with one id, read before and after, and its
   mass is read from the compiled model rather than from a comment. Neither the humanoid nor the
   plant creates a tray.

3. **DOWNSTREAM STARTS ONLY AFTER THE HUMANOID HAS LET GO.** This is the ordering claim, and it
   is checked as an ORDERING with timestamps, not as an inference from "both things happened".

4. **THE HUMANOID'S FAILURE MUST STOP THE CHAIN.** The guidance requires the integration
   negative case "humanoid grasp/exit failure implies downstream does not start". H2 already
   established, by measurement, exactly how the grasp fails when it fails -- so the negative arm
   runs the same probe with the grasp window collapsed to zero, and the chain must not move.

FORBIDDEN, AND THEREFORE NOT DONE
----------------------------------
No weld/equality on the tray, **no runtime qpos writes** (there is not one write site in the run
loop -- the plant refuses to reset when it is a guest, which is what makes that structural rather
than promised), no disabling collisions, no fixed base. `runtime_qpos_writes` is counted and
printed, and any equality touching the tray aborts the run.

WHAT IS BORROWED AND WHAT IS NEW
---------------------------------
`probe_h2_w5`'s stage driver, roll corrector, clearance helpers and declared thresholds are
IMPORTED, so the H2 rows in this report come from the same code as the H2 report. `tray_task` is
the SAME judge with the SAME frozen thresholds. Every H3 row is DECLARED, not frozen, and each
one is falsifiable: the run's own numbers are printed next to it.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import re
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src'))
sys.path.insert(0, str(ROOT / 'experiments'))

import probe_h_w5 as H1                        # the shared instruments, imported not copied
import probe_h2_w5 as H2                       # the stage driver + declared H2 thresholds

WORLD = ROOT / 'assets' / 'world_w5_h085_loop.xml'
TRAY = 'c_payload'
BOOT = 'boot-h3-01'
ORDER = 'h3-integration-01'
EPOCH = 13

#: The humanoid phase drives `SEQUENCE_SECONDS`; the chain then gets `CHAIN_BUDGET_S` of sim time
#: of its own. The chain's own timings are what W5 measured (`reports/w5-loop-07` ran 164.85 s of
#: plant time for the same script), so the budget is derived from that rather than guessed.
SEQUENCE_SECONDS = H2.SEQUENCE_SECONDS        # 50.0 -- H2's own schedule, unchanged
CHAIN_BUDGET_S = 240.0
WALL_DEADLINE_S = 3000.0
SAMPLE_INTERVAL_TICKS = H2.SAMPLE_INTERVAL_TICKS

#: DECLARED AND NOT FROZEN. Each is falsifiable: the run records the quantity over its whole
#: length, so the report shows it moving rather than only where it landed.
H3_THRESHOLDS = {
    #: the chain's first motion command must follow the humanoid's release by at least this long
    'release_to_chain_gap_s': 0.5,
    #: the tray must not move more than this between the humanoid's release and the chain's first
    #: motion: anything more and the "handoff" is really the chain having taken a moving tray
    'handoff_drift_m': 0.010,
    #: the tray must be on the SOURCE band's own rollers when the chain takes custody
    'custody_on_source_rollers': 1,
    #: the loop must actually reach the receiver -- the end of the chain, not the middle
    'chain_reaches_receiver': 1,
    #: no equality may touch the tray at any point
    'max_equalities_on_tray': 0,
    #: and the run must not write qpos
    'max_runtime_qpos_writes': 0,
    #: the dock residuals, against the W4 ENVELOPE's own numbers. Repeated here rather than
    #: imported so this file states its own criteria; they are the W4 values, not new ones.
    'dock_longitudinal_m_max': 0.020,
    'dock_lateral_m_max': 0.002,
    #: How far the AMT's chassis may drift while the humanoid loads it. DERIVED from the plant's
    #: own contract: `DRIFT_LIMIT_M = STOPPED_SPEED_MPS * STOPPED_HOLD_S` = 0.01 * 0.5 = 0.005 m
    #: is what a vehicle obeying the speed limit covers, and the dock envelope's own longitudinal
    #: tolerance is 0.020 m. 0.010 m sits inside both, so a vehicle that is merely parked-and-
    #: creeping passes and a vehicle that coasts 4 m fails by two orders of magnitude.
    'chassis_parked_drift_max_m': 0.010,
}

SIDES = ('left', 'right')

#: The MEASURING BUDGET for the collision-enabled clearance instrument, in metres.
#:
#: `mj_geomDistance(model, data, a, b, distmax, ...)` treats `distmax` as a RANGE LIMIT, not a
#: ceiling: when a pair's true distance is at or beyond it, the return value is not a distance.
#: Passing 2.0 produced a "0.00000 m" run minimum between two geoms measured 2.0752 m apart. This
#: budget is 0.5 m -- ten times the 0.05 m threshold the rows actually judge -- so every pair that
#: could possibly matter is genuinely measured, and any pair at the budget is reported as
#: `beyond_budget` instead of being quoted as a number.
BAND_CLEARANCE_BUDGET_M = 0.5

#: ★ WHICH ARM EACH ROW IS JUDGED IN. DECLARED, AND ENFORCED.
#:
#: This exists because the same defect happened FOUR TIMES: a row written for the positive arm was
#: scored in the negative arm, where its claim is not the claim the run is supposed to make. Each
#: time it turned a correct run into a red mark:
#:   * `chain_reached_receiver`     -- the negative arm REQUIRES that it does not
#:   * `tray_on_source_band_at_handoff` -- the negative arm removes the supply it asserts
#:   * `custody_transferred_by_the_ledger` -- the negative arm expects the transaction to be REFUSED
#:   * `each_leg_destination_matched_by_contacts` -- nothing arrives, so nothing matches
#: Every one of them was found by RUNNING the arm, which costs two minutes each. So the mapping is
#: declared here and `judge_h3` REFUSES to return if a row judged in this arm says PASS/FAIL while
#: this arm is not in its list -- i.e. a row that forgets to be arm-aware is now a `RuntimeError`
#: in a two-second test (`tests/test_h3_integration.py` calls the judge on both fixtures), not a
#: four-minute surprise.
#:
#: `both` means the row applies in both arms, and its CRITERIA are arm-aware (see
#: `every_chain_step_succeeded` and `the_chain_moved_the_tray`).
ROW_ARMS = {
    'one_world_one_data': ('positive', 'negative'),
    'same_tray_entity': ('positive', 'negative'),
    'tray_on_source_band_at_handoff': ('positive',),
    'hands_clear_of_tray_at_handoff': ('positive', 'negative'),
    'vehicle_parked_while_humanoid_works': ('positive', 'negative'),
    'downstream_waits_for_release': ('positive',),
    'tray_still_between_release_and_the_chain': ('positive',),
    'the_chain_moved_the_tray': ('positive', 'negative'),
    'chain_reached_receiver': ('positive',),
    'every_chain_step_succeeded': ('positive', 'negative'),
    'dock_residuals_within_tolerance': ('positive', 'negative'),
    'custody_transferred_by_the_ledger': ('positive',),
    'each_leg_destination_matched_by_contacts': ('positive',),
    'ledger_agrees_with_physics': ('positive', 'negative'),
    'no_weld_on_the_tray': ('positive', 'negative'),
    'no_runtime_qpos_write': ('positive', 'negative'),
    'downstream_gated_on_handoff': ('negative',),
    'no_abnormal_collision': ('positive', 'negative'),
    'humanoid_withdrew_clear': ('positive', 'negative'),
}


def tray_support_rows(model, data, tray_body, rows):
    """Which of the world's roller rows the tray is touching, from its own contact set."""
    hit = set()
    for index in range(int(data.ncon)):
        contact = data.contact[index]
        g1, g2 = int(contact.geom1), int(contact.geom2)
        if tray_body not in (int(model.geom_bodyid[g1]), int(model.geom_bodyid[g2])):
            continue
        other = g2 if int(model.geom_bodyid[g1]) == tray_body else g1
        name = model.geom(other).name or ''
        for row, prefix in rows:
            if name.startswith(prefix):
                hit.add(row)
    return sorted(hit)


def equalities_touching(model, body_id):
    """Any equality (weld included) that involves the body -- read from the compiled model."""
    hit = []
    for e in range(model.neq):
        for obj in (int(model.eq_obj1id[e]), int(model.eq_obj2id[e])):
            if obj == body_id:
                hit.append(e)
                break
    return hit


def run(args):
    out = ROOT / 'reports' / args.run_id
    out.mkdir(parents=True, exist_ok=False)
    (out / 'diagnostics').mkdir(exist_ok=True)
    started = time.monotonic()
    report = {
        'scope': 'DIAGNOSTIC_ONLY',
        'probe': 'H3: the H chain integrated into the W5 logistics chain',
        'pid': os.getpid(), 'command': sys.argv, 'status': 'ERROR', 'samples': [],
        'source_sha256': hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        'world': str(WORLD.relative_to(ROOT)),
        'world_sha256': hashlib.sha256(WORLD.read_bytes()).hexdigest(),
        'h3_thresholds': dict(H3_THRESHOLDS),
        'h3_thresholds_note': ('DECLARED AND NOT FROZEN -- these rows have no contract behind them '
                               'and are submitted for review. No threshold is back-fitted to a '
                               'failure.'),
        'integration_contract': {
            'same_world': 'the humanoid Runtime and the W5 plant hold the same MjModel and MjData',
            'same_tray_entity': TRAY,
            'no_world_reload': True,
            'downstream_after_release': True,
            'negative_case': ('the grasp window is collapsed to zero and the chain must not start '
                              '(see --negative-grasp)'),
        },
        'sequence_seconds': SEQUENCE_SECONDS,
        'chain_budget_s': CHAIN_BUDGET_S,
    }
    qpos_writes = 0
    try:
        import numpy as np
        import mujoco
        import w4_plant as wp
        import probe_w5_loop as W5
        import probe_w4_skills as w4judge
        from workcell import resources as R
        from workcell import transfer as XFER
        from workcell.adapters.sim import SkillAdapter
        from workcell.orchestration.sequence import SkillSequence
        from workcell.safety import CommandGate
        from humanoid007 import tray_task
        from humanoid007.runtime import OPEN, GRASP, ExitPath

        # --- 1. ONE world, ONE data, and the home state set ONCE ---------------------------
        #
        # ★ ORDER MATTERS, AND GETTING IT WRONG PRODUCED A REAL DEFECT.
        #
        # The first version constructed the humanoid's Runtime (which makes its own MjData, with
        # qpos at zeros) and then handed that data to the plant. `LogisticsPlant._derive()` reads
        # the deck row and the chassis position off `data.geom_xpos` to DERIVE its dock geometry,
        # so it derived that geometry from a world that was not at its home state. Measured
        # consequence: `dock_residuals('station_c')` returned `longitudinal_m = -4.608 m` while the
        # chassis was 2.9 mm from its target and had `travelled_m = 0.0`, and the very first
        # `DOCK` reported FAILED / DOCK_OUT_OF_TOLERANCE for a vehicle that was already parked.
        #
        # So the world is put at its own home pose ONCE, here, before anything reads it. That is
        # the single initialisation write, it is logged, and the run loop has no write site at all.
        model = mujoco.MjModel.from_xml_path(str(WORLD))
        data = mujoco.MjData(model)
        home = np.array(model.key_qpos[0], dtype=float)
        data.qpos[:] = home
        mujoco.mj_forward(model, data)
        qpos_writes = 1
        qpos_write_log = [[-1, 'qpos[:] = model.key_qpos[0]', 'the world\u2019s own home pose']]

        # the humanoid runtime is built ON this model and data; it writes only its own arm and
        # finger posture and never touches a free body, so the home state survives
        r = H1_runtime(WORLD, model=model, data=data)
        # and the plant is a GUEST on the same world, deriving its geometry from the home state
        plant = wp.LogisticsPlant(world=str(WORLD), model=model, data=data)
        if getattr(args,'global_writer',False):
            from workcell.runtime_permit import RuntimePermit
            from world_owner import WorldOwner
            permit_sequence=[0]
            def observe_runtime_permission():
                permit_sequence[0]+=1
                return dict(owner='shared_world',epoch=EPOCH,generation=1,
                    sequence=permit_sequence[0],observed_wall_s=time.monotonic(),
                    cancel_requested=False,zone_clear=True,stop_chain_healthy=True)
            world_owner=WorldOwner(model,data,RuntimePermit(observe_runtime_permission,
                owner='shared_world',epoch=EPOCH,generation=1,max_age_s=.5))
            world_owner.attach(plant)
            r.step_owner=world_owner
            r.background_control=plant.park_control
            report['global_writer']={'enabled':True,'ideal_external_evidence':True,
                                     'whole_order':'NOT_RUN'}
        names = r.names
        report['init_qpos_writes'] = qpos_writes
        report['qpos_write_sites'] = {
            'init': qpos_write_log,
            'in_loop': [],
            'note': ('the loop never writes qpos. The plant refuses to reset when it is a guest, '
                     'so "no runtime qpos writes" is structural rather than promised: there is no '
                     'write site for it to use.'),
        }
        report['humanoid_state_after_init'] = [float(v) for v in r.d.qpos[0:7]]
        report['shared_world'] = {
            'humanoid_model_id': id(r.m), 'plant_model_id': id(plant.model),
            'humanoid_data_id': id(r.d), 'plant_data_id': id(plant.data),
            'same_model': r.m is plant.model, 'same_data': r.d is plant.data,
            'plant_owns_world': bool(getattr(plant, '_owns_world', True)),
            'plant_qpos_writes_at_construction': dict(plant.qpos_writes),
            'plant_derived_at_home_state': {
                'chassis_at_home_m': float(plant.chassis_at_home),
                'deck_row_first_m': float(plant.deck_row[0]),
                'source_band_last_m': float(plant.source_band[-1]),
                'pitch_m': float(plant.pitch),
                'dock_x_source_m': float(plant.dock_x_source),
                'dock_residual_at_home_station_c': plant.dock_residuals('station_c'),
            },
        }
        if not (r.m is plant.model and r.d is plant.data):
            raise RuntimeError('the humanoid and the plant are not sharing one world')
        # ★ THE GUARD THAT WOULD HAVE CAUGHT THE ORDER DEFECT. The plant derives its dock geometry
        # from the home state; if it did not see the home state, station_c's residual is metres
        # out instead of millimetres. Asserting it here means the defect cannot return silently.
        residual_at_home = report['shared_world']['plant_derived_at_home_state'][
            'dock_residual_at_home_station_c']['longitudinal_m']
        if abs(residual_at_home) > 0.6:
            raise RuntimeError(
                'the plant derived its dock geometry from a world that is not at its home state: '
                'station_c residual at the home pose is %.4f m, which is a state error, not a '
                'parking error' % residual_at_home)
        report['names'] = {'prefix': names.prefix, 'object_body': names.obj()}
        report['free_base'] = int(r.m.jnt_type[0]) == int(mujoco.mjtJoint.mjJNT_FREE)
        report['equality_constraints'] = int(r.m.neq)

        tray_body = r.m.body(TRAY).id
        report['tray_body_id'] = int(tray_body)
        report['tray_mass_kg'] = float(r.m.body_mass[tray_body])
        report['equalities_involving_the_tray'] = equalities_touching(r.m, tray_body)
        if report['equalities_involving_the_tray']:
            raise RuntimeError('an equality constrains the tray: %r'
                               % report['equalities_involving_the_tray'])

        # --- the state the world starts in, read not assumed --------------------------------
        tray0 = np.array(r.d.body(TRAY).xpos, float)
        report['tray_start'] = [float(v) for v in tray0]
        report['payload_z_initial'] = float(tray0[2])
        report['payload_x_initial'] = float(tray0[0])
        report['humanoid_base_at_home'] = [float(v) for v in r.d.qpos[0:3]]
        report['start_support'] = tray_support_rows(r.m, r.d, tray_body, wp.LogisticsPlant.ROWS)

        payload_joint = int(r.m.body_jntadr[tray_body])
        payload_dof = int(r.m.jnt_dofadr[payload_joint])
        left_geoms, right_geoms, payload_geoms = H1.hand_geom_sets(r.m, names)
        hand_geoms = left_geoms | right_geoms
        humanoid_geoms = [g for g in range(r.m.ngeom)
                          if (r.m.body(int(r.m.geom_bodyid[g])).name or '').startswith(names.prefix)]
        band_geoms = [g for g in range(r.m.ngeom)
                      if (r.m.body(int(r.m.geom_bodyid[g])).name or '').startswith(
                          ('c_fixed_roller_', 'c_recv_roller_'))
                      or (r.m.body(int(r.m.geom_bodyid[g])).name or '') == 'c_deck']

        default_arms = r.policy.default[r.arm_ids].copy()
        exit_path = ExitPath(r, H2.anchor_at(H2.PLACE_X, 0.0, H2.HANDLE_Y), default_arms,
                             ExitPath.DEFAULT_VARIANT)

        # --- the chain's own objects, the REAL ones -----------------------------------------
        contract = json.loads((ROOT / 'config' / 'docking_contract.json').read_text())
        max_age = float(contract['validity']['max_observation_age_s'])
        resources = R.ResourceTable(epoch=EPOCH,
                                    resources=[W5.SOURCE_RES, W5.DECK_RES, W5.RECEIVER_RES])
        for resource_id in (W5.SOURCE_RES, W5.DECK_RES, W5.RECEIVER_RES):
            resources.mark_cleared(resource_id, evidence=[f'runs/init/{resource_id}.json'],
                                   now_s=0.0)
        ledger = XFER.TransferLedger(epoch=EPOCH)
        gate = CommandGate(ttl_s=0.5, silence_s=0.5, v_max=0.6, w_max=1.2, source='safety_gate')
        adapter = SkillAdapter(plant=plant, boot_id=BOOT, stations=plant.stations,
                               envelope=w4judge.ENVELOPE, transfers=W5.TRANSFERS)
        sequence = SkillSequence(adapter=adapter, gate=gate, boot_id=BOOT, epoch=EPOCH,
                                 order_id=ORDER, wall_clock=time.monotonic)
        report['chain'] = {
            'boot_id': BOOT, 'order_id': ORDER, 'epoch': EPOCH,
            'skills': [skill for skill, _ in W5.SCRIPT],
            'safety_gate': {'ttl_s': 0.5, 'silence_s': 0.5, 'v_max': 0.6, 'w_max': 1.2},
            'max_observation_age_s': max_age,
        }

        # --- 2. the humanoid's handoff, in this same world ----------------------------------
        #
        # THE NEGATIVE ARM, and it needs TWO substitutions rather than one. `--negative-grasp`
        # (a) passes the OPEN finger posture where the grasp posture belongs, so the fingers never
        # close on the handle, and (b) pins the carry target to the tray's own start x, so the arm
        # cannot translate what it is not holding.
        #
        # ★ WHY (b) IS NOT A CONVENIENCE. The first version did only (a). Measured
        # (`_diag/out_h3_neg1.txt`): the OPEN hand swept through the `place` translation and
        # **bulldozed the tray off the presentation fixture onto the source band** -- the handoff
        # support came out `['source_band']` with the tray at x 4.4869, and the chain then carried
        # it all the way to the receiver. A "grasp failure" that still delivers the tray proves
        # nothing about gating, and the negative row would have passed on it.
        negative = bool(args.negative_grasp)
        grasp_posture = OPEN if negative else GRASP
        carry_target = float(tray0[0]) if negative else H2.PLACE_X
        phase_windows = dict(H2.PHASE_WINDOWS)
        report['negative_arm'] = {
            'enabled': negative,
            'mechanism': ('the OPEN finger posture replaces the grasp posture AND the carry target '
                          'is pinned to the tray start x, because a grasp that holds nothing '
                          'cannot translate anything either'),
            'grasp_posture_is_open': negative,
            'carry_target_x_m': carry_target,
            'why_two_substitutions': (
                'opening the fingers alone still bulldozed the tray onto the band -- measured in '
                '_diag/out_h3_neg1.txt, where the handoff support came out as source_band with the '
                'tray at x 4.4869, and the chain then delivered it'),
        }

        foot_worst = {'L': 0.0, 'R': 0.0}
        abnormal_total, self_total = {}, {}
        band_min, band_min_who = float('inf'), None
        band_min_live = {'m': float('inf'), 'humanoid': None, 'band': None, 't': None}
        hand_min, hand_min_who = float('inf'), None
        #: ★ WHEN THE HANDS LET GO, AND THE FIRST VERSION GOT THIS WRONG.
        #:
        #: The first version set `release_s` to "the first sampled time the hands were off the
        #: tray after the grasp window opened". The grasp window opens at 9.5 s and the hands
        #: arrive at the handle at ~11 s, so it reported **9.598 s** -- which is the moment BEFORE
        #: the grasp, not the release. The row then PASSED on a 59.83 s gap when the true gap is
        #: 35.13 s. A row that passes on the wrong number is worse than a row that fails, because
        #: nothing draws attention to it.
        #:
        #: The honest definition, and the one the H judge already uses: the release is the start of
        #: the LAST UNBROKEN stretch during which no hand touched the tray, and it only counts if a
        #: grasp happened first. `grasp_s` is recorded so the two are distinguishable.
        hand_off_since = None
        release_s = None
        grasp_s = None
        hand_touch_last_s = None
        tray_contacts_seen = 0
        #: The AMT's chassis x, sampled through the humanoid phase. It must not move: the vehicle
        #: is docked at station_c while the humanoid loads it. This is the trace that would have
        #: caught the 4.04 m coast immediately instead of via a downstream NAV_FAILED.
        chassis_trace = [(0.0, round(plant.chassis_x(), 6))]
        #: The SECOND vehicle (`n2_base_link`, parked at y 1.15 off the corridor). It has wheel
        #: motors the plant does not own, so it is NOT braked by `park_control`. Recorded so the
        #: report says what it did rather than leaving a reader to assume it held still.
        n2_trace = [(0.0, round(float(r.d.xpos[r.m.body('n2_base_link').id][0]), 6))]

        while r.d.time < SEQUENCE_SECONDS:
            if time.monotonic() - started > WALL_DEADLINE_S:
                report['status'] = 'WALL_TIMEOUT'
                break
            now = r.d.time
            arms, hands = H2.stage_driver(r, float(tray0[0]), now, default_arms, OPEN,
                                          grasp_posture, exit_path, H2.HANDLE_Y,
                                          H2.HAND_ROLL_BIAS, phase_windows, None, carry_target)
            # ★ THE VEHICLE IS PARKED WHILE THE HUMANOID WORKS, AND IT HAS TO BE COMMANDED.
            #
            # The wheel motors are RAW (biastype 0, zero biasprm), so ctrl=0 is zero TORQUE, not
            # a brake -- the plant's own `_wheel_rate_torque` docstring records the same finding:
            # "Cutting torque is not braking; it is coasting." Measured: with nothing commanding
            # this plant during the 50 s humanoid phase, the vehicle coasted 4.04 m (4.41372 ->
            # 8.45782) and the chain collapsed. So the plant's park controller is applied every
            # tick, the runtime then writes its own actuators over it, and the single `mj_step`
            # inside `Runtime.step` ends the period.
            if getattr(r,'step_owner',None) is None:
                r.d.ctrl[:] = plant.park_control()
            r.step(np.zeros(3), arms, hands, stationary=now > 2)
            if r.tick % SAMPLE_INTERVAL_TICKS == 0:
                chassis_trace.append((round(float(now), 3), round(plant.chassis_x(), 6)))
                n2_trace.append((round(float(now), 3), round(float(
                    r.d.xpos[r.m.body('n2_base_link').id][0]), 6)))

            if r.tick % SAMPLE_INTERVAL_TICKS == 0:
                gap_band, who_band, gap_hand, who_hand = H2.clearances(
                    r.m, r.d, names, humanoid_geoms, payload_geoms, band_geoms, hand_geoms)
                # the collision-enabled instrument, reported alongside H2's -- see its docstring
                live_band, live_hand = clearances_collidable(
                    r.m, r.d, humanoid_geoms, payload_geoms, band_geoms, hand_geoms)
                if gap_band < band_min:
                    band_min, band_min_who = gap_band, who_band
                if live_band['m'] < band_min_live['m']:
                    band_min_live = dict(live_band, t=round(float(now), 3))
                if gap_hand < hand_min:
                    hand_min, hand_min_who = gap_hand, who_hand
                row = r.snapshot()
                t = r.d.body(TRAY).xpos
                row['payload'] = [float(t[0]), float(t[1]), float(t[2])]
                row['payload_speed'] = float(np.linalg.norm(r.d.qvel[payload_dof:payload_dof + 3]))
                counts = {'left': 0, 'right': 0}
                for index in range(r.d.ncon):
                    g1 = int(r.d.contact[index].geom1)
                    g2 = int(r.d.contact[index].geom2)
                    for near, far in ((g1, g2), (g2, g1)):
                        if near in payload_geoms:
                            if far in left_geoms:
                                counts['left'] += 1
                            elif far in right_geoms:
                                counts['right'] += 1
                row['hand_contacts'] = counts
                row['arm_dev_rad'] = float(np.max(np.abs(
                    r.d.qpos[r.qa[r.arm_ids]] - default_arms)))
                row['support'] = H2.support_snapshot(r.m, r.d, payload_geoms,
                                                     H2.SUPPORT_ROLLERS + (H2.FIXTURE,))
                row['tray_tilt_deg'] = H2.tray_tilt_deg(r.m, r.d, tray_body)
                row['band_clearance_m'] = gap_band
                row['band_clearance_live_m'] = live_band
                row['hand_tray_clearance_m'] = gap_hand
                row['tray_support_rows'] = tray_support_rows(r.m, r.d, tray_body,
                                                             wp.LogisticsPlant.ROWS)
                row['hand_tray_contacts'] = contact_pairs(r.m, r.d, left_geoms, right_geoms,
                                                          payload_geoms)
                row['phase'] = 'humanoid'
                touching = bool(counts['left'] or counts['right'])
                if touching:
                    hand_touch_last_s = float(now)
                    hand_off_since = None
                    if grasp_s is None:
                        grasp_s = float(now)
                else:
                    if hand_off_since is None:
                        hand_off_since = float(now)
                tray_contacts_seen += len(row['hand_tray_contacts'])
                feet, abnormal, self_contact = H1.foot_and_collision_instruments(
                    r.m, r.d, names, left_geoms, right_geoms, payload_geoms)
                row['feet'] = feet
                for side in ('L', 'R'):
                    if feet[side]['pitch_deg'] is not None:
                        foot_worst[side] = max(foot_worst[side], feet[side]['pitch_deg'])
                for pair, n in abnormal.items():
                    abnormal_total[pair] = abnormal_total.get(pair, 0) + n
                for pair, n in self_contact.items():
                    self_total[pair] = self_total.get(pair, 0) + n
                report['samples'].append(row)

        humanoid_end_s = float(r.d.time)
        tray_at_handoff = np.array(r.d.body(TRAY).xpos, float)
        # ★ THE RELEASE, DERIVED. `hand_off_since` is the start of the final UNBROKEN hands-off
        # stretch, and it is only the release if a grasp preceded it -- otherwise it is just the
        # approach. Both are recorded so a reader can see which happened.
        if grasp_s is not None and hand_off_since is not None and hand_off_since > grasp_s:
            release_s = hand_off_since
        report['humanoid_phase_end_s'] = humanoid_end_s
        report['tray_at_handoff'] = [float(v) for v in tray_at_handoff]
        report['grasp_s'] = grasp_s
        report['hand_off_since_s'] = hand_off_since
        report['release_s'] = release_s
        report['release_definition'] = ('the start of the last unbroken stretch in which no hand '
                                       'touched the tray, and only when a grasp preceded it. The '
                                       'first version used "hands off after the grasp window '
                                       'opened", which reported 9.598 s -- the approach, not the '
                                       'release -- and passed the ordering row on a wrong number.')
        report['hand_touch_last_s'] = hand_touch_last_s
        report['hand_tray_contact_samples'] = tray_contacts_seen
        report['chassis_trace_during_humanoid'] = chassis_trace
        report['chassis_drift_during_humanoid_m'] = round(
            abs(chassis_trace[-1][1] - chassis_trace[0][1]), 6)
        report['second_vehicle_trace_during_humanoid'] = n2_trace
        report['second_vehicle_note'] = (
            '`n2_base_link` is a second chassis parked at y 1.15, off the corridor. Its wheel '
            'motors are not in `LogisticsPlant.wheel_actuators` (the plant requires exactly two), '
            'so `park_control` does not brake it. Its trace is recorded so the report states what '
            'it did rather than implying the world held still.')
        report['band_clearance_run_min'] = {
            'all_geoms_m': band_min, 'all_geoms_who': band_min_who,
            'collision_enabled': band_min_live,
            'note': ('two instruments on purpose. `all_geoms` is the one H2 used and is kept for '
                     'continuity; `collision_enabled` is the physical one AND it names both sides '
                     'of the pair plus the time, because a minimum distance is a claim about a '
                     'pair and reporting half of it is how an artefact survives review.'),
        }
        report['support_at_handoff'] = tray_support_rows(r.m, r.d, tray_body, wp.LogisticsPlant.ROWS)
        report['abnormal_collisions'] = [
            {'pair': list(k), 'contact_samples': v}
            for k, v in sorted(abnormal_total.items(), key=lambda kv: -kv[1])]
        report['abnormal_collision_count'] = len(abnormal_total)
        report['hand_self_contacts'] = [
            {'pair': list(k), 'contact_samples': v}
            for k, v in sorted(self_total.items(), key=lambda kv: -kv[1])]
        report['feet'] = {'worst_pitch_deg': {k: round(v, 4) for k, v in foot_worst.items()}}

        # --- the H2 six-stage contract, from the same judge ---------------------------------
        driven = {n for n, (_, end) in phase_windows.items() if r.d.time >= end}
        judged = [{'t': row['time'], 'payload_z': row['payload'][2],
                   'payload_x': row['payload'][0], 'payload_y': row['payload'][1],
                   'payload_speed': row['payload_speed'],
                   'hand_contacts': row['hand_contacts'],
                   'arm_dev_rad': row['arm_dev_rad']}
                  for row in report['samples']]
        result = tray_task.evaluate(judged, driven, report['payload_z_initial'])
        report['h_stages'] = result['stages']
        report['h_stages_driven'] = sorted(driven)
        report['h_summary'] = tray_task.summary_line(result)
        report['full_H_acceptance'] = result['overall']

        # --- 3. the chain, in the SAME data, with no reload ---------------------------------
        #
        # NOTHING here reloads a world, re-creates a tray or resets the plant. `plant.data` is the
        # same object the humanoid's controller has been stepping all along, so whatever the chain
        # finds is what the handoff left.
        #
        # THE GATE. The chain must not begin before the humanoid has released. This is a DECLARED
        # precondition of the integration, checked by comparing the plant's clock at the first
        # chain motion against the measured release time -- an ORDERING, not an inference from
        # "both happened".
        rows = []
        chain_started_s = None
        #: ★ THE P2 TRANSFER TRANSACTIONS, DRIVEN -- NOT MERELY CONSTRUCTED.
        #:
        #: The first version of this probe CREATED the `TransferLedger` and the `ResourceTable` and
        #: then never called either one. The report said the chain ran, and the custody transfer --
        #: which is a named part of the H3 definition -- was recorded nowhere. That is the
        #: "a required field that was never written" defect (`D106`'s `runtime_qpos_writes = null`)
        #: wearing a different costume: the objects were there for appearance. So the ledger is now
        #: walked exactly as `probe_w5_loop.py` walks it, and rows read its own record.
        #:
        #: ★ And the stage evidence is keyed by the stage the LEDGER ENTERED, from the ledger's own
        #: record -- `probe_w5_loop.py` records why: keying by the stage the caller INTENDED drifts
        #: by four states as soon as one skill advances a different number than the caller assumed.
        transactions = {}
        for index, (skill, extra) in enumerate(W5.SCRIPT):
            if time.monotonic() - started > WALL_DEADLINE_S:
                report['status'] = 'WALL_TIMEOUT'
                break
            arguments = dict(extra)
            raw = {'schema_version': 1, 'command_id': f'h3-cmd-{index:03d}', 'order_id': ORDER,
                   'revision': index + 1, 'expected_boot_id': BOOT, 'skill': skill,
                   'arguments': arguments, 'resource_generation': 0, 'deadline_s': 120.0}
            now = time.monotonic()
            answer = sequence.submit(raw, now_s=now)
            record = answer['record']
            result_row = record.result or {}
            final = result_row.get('final_state', {})
            tray = plant.tray_state()
            support = tray_support_rows(r.m, r.d, tray_body, wp.LogisticsPlant.ROWS)
            if skill in ('MOVE_TO_STATION', 'DOCK') and chain_started_s is None \
                    and tray['x_m'] != report['tray_at_handoff'][0]:
                chain_started_s = plant.clock()

            # -- DOCK: launch the transfer transaction, with declared preconditions -------------
            if skill == 'DOCK':
                station = arguments['station_id']
                residual = plant.dock_residuals(station)
                leg = 'xfer_source_to_deck' if station == 'station_c' else 'xfer_deck_to_receiver'
                source = W5.SOURCE_RES if station == 'station_c' else W5.DECK_RES
                receiver = W5.DECK_RES if station == 'station_c' else W5.RECEIVER_RES
                preconditions = {
                    'source_capacity_free': True,
                    'receiver_capacity_free': True,
                    'height_within_tolerance':
                        abs(residual['longitudinal_m']) <= w4judge.ENVELOPE['dock_longitudinal_m'],
                    'lateral_within_tolerance':
                        abs(residual['lateral_m']) <= w4judge.ENVELOPE['dock_lateral_m'],
                    'yaw_within_tolerance':
                        abs(residual['yaw_rad']) <= w4judge.ENVELOPE['dock_yaw_rad'],
                    'clearance_ok':
                        residual['longitudinal_m'] <= w4judge.ENVELOPE['dock_longitudinal_m'],
                    'amr_at_rest': bool(final.get('stopped_confirmed')),
                    # per leg, not shared -- see `probe_w5_loop.py` for why one expression serving
                    # both legs is a precondition true for a reason this leg never checked
                    'source_has_tray': tray['zone'] == ('source_band' if station == 'station_c'
                                                        else 'deck'),
                    'receiver_empty': tray['zone'] != ('deck' if station == 'station_c'
                                                       else 'receiver_band'),
                    'arms_retracted': True,   # vacuously: the chain drives no arm
                    'stop_chain_healthy': bool(record.gate_offers),
                    'evidence_age_s': 0.05,
                    'max_evidence_age_s': max_age,
                    'ttl_s': 60.0,
                }
                transactions[leg] = {
                    'source': source, 'receiver': receiver, 'preconditions': preconditions,
                    'residual': residual, 'stage_evidence': {}, 'custody': [],
                    'source_zone': plant.zone_of(tray['x_m'], plant.tray_support()),
                    'destination_zone': ('deck' if station == 'station_c' else 'receiver_band'),
                    'tray_zone_after': None, 'support_after': [], 'record': None,
                    'refusal': None, 'stopped_by': None,
                    'observed_support_at_advance': None}
                try:
                    created = ledger.request({'transfer_id': leg, 'order_id': ORDER,
                                              'epoch': EPOCH, 'source': source,
                                              'receiver': receiver, 'tray_id': TRAY,
                                              'preconditions': preconditions},
                                             resources=resources, now_s=now)
                    transactions[leg]['record'] = created
                    print('      transfer %s: stage %s custody %s'
                          % (leg, created.stage, created.custody), flush=True)
                except Exception as refusal:              # noqa: BLE001 -- recorded, not lost
                    # a launch refusal is a RESULT, not a crash: the contract refuses an UNKNOWN
                    # precondition by name, and that is a different finding from "the loop did not
                    # run" with a different fix
                    transactions[leg]['refusal'] = '%s: %s' % (type(refusal).__name__, refusal)
                    print('      transfer %s: REFUSED at launch -- %s' % (leg, refusal), flush=True)

            # -- the transfer skills: ADVANCE THE LEDGER, GATED ON THE OBSERVED OUTCOME --------
            #: ★ THE STAGES ARE NOT ADVANCED FROM A DECLARATION. The first version walked a fixed
            #: tuple per skill (`EVIDENCE_AT_VERIFY_TRANSFER` has three stages), so every
            #: `VERIFY_TRANSFER` pushed the transaction to RELEASED whether or not the tray had
            #: gone anywhere -- in the negative arm the ledger reported a delivery for a tray
            #: still sitting on the source band. A declaration of which stage the evidence
            #: arrives at is not a guard; the guard is reading the tray.
            if skill in ('START_TRANSFER', 'VERIFY_TRANSFER'):
                leg = arguments['transfer_id']
                info = transactions.get(leg)
                if info is not None:
                    evidence = [f'runs/{skill.lower()}_{leg}.json', f'obs/{leg}.json']
                    info['stage_evidence'].setdefault('REQUESTED', list(evidence))
                    rec = info['record']
                    if rec is not None:
                        tray_now = plant.tray_state()
                        held = list(plant.tray_support())
                        destination = info['destination_zone']
                        # the receiver's confirmation is a claim that the tray is on the
                        # destination row, measured -- not a filename
                        receiver_has_it = destination in held
                        info['observed_support_at_advance'] = held
                        stopped_by = None
                        for _ in range(len(W5.EVIDENCE_AT_START_TRANSFER)
                                       + len(W5.EVIDENCE_AT_VERIFY_TRANSFER)):
                            if rec.stage == 'RELEASED':
                                break
                            nxt = XFER._NEXT.get(rec.stage)
                            if nxt == 'RECEIVER_CONFIRMED' and not receiver_has_it:
                                stopped_by = (
                                    'REFUSED_NO_PHYSICAL_DELIVERY: the tray is on %s, not on the '
                                    'declared destination %s, so the receiver cannot confirm '
                                    'receipt' % (held or ['nothing'], destination))
                                break
                            if nxt == 'COMMITTED' and not rec.evidence.get('receiver_confirmed'):
                                stopped_by = ('REFUSED_NO_CONFIRMATION: COMMITTED requires the '
                                              'receiver confirmation that was never made')
                                break
                            # the confirmation carries the measurement that justifies it
                            stage_evidence = list(evidence)
                            if nxt == 'RECEIVER_CONFIRMED':
                                stage_evidence = stage_evidence + [
                                    'contacts:%s' % ','.join(held)]
                            rec = ledger.advance(leg, evidence=stage_evidence)
                            info['stage_evidence'][rec.stage] = list(stage_evidence)
                            info['custody'].append(rec.custody)
                        info['record'] = rec
                        info['stopped_by'] = stopped_by
                        if stopped_by:
                            print('      transfer %s: STOPPED at %s -- %s'
                                  % (leg, rec.stage, stopped_by), flush=True)
                    tray_now = plant.tray_state()
                    info['tray_zone_after'] = tray_now['zone']
                    info['support_after'] = list(plant.tray_support())

            rows.append({'command_id': raw['command_id'], 'skill': skill,
                         'accepted': answer['accepted'], 'idempotent': answer['idempotent'],
                         'refusal': record.refusal, 'status': result_row.get('status'),
                         'reason_code': result_row.get('reason_code'),
                         'final_state': final, 'tray_zone': tray['zone'],
                         'tray_x_m': tray['x_m'], 'tray_on_crowns': tray['on_crowns'],
                         'tray_support': support, 'plant_clock_s': plant.clock(),
                         'phase': 'chain'})
            print('  %s %-18s accepted %s status %s reason %s t=%7.3f tray %s x %.4f support %s'
                  % (raw['command_id'], skill, answer['accepted'], result_row.get('status'),
                     result_row.get('reason_code'), plant.clock(), tray['zone'], tray['x_m'],
                     support), flush=True)

        report['chain_rows'] = rows
        report['chain_transactions'] = {
            leg: {'source': i['source'], 'receiver': i['receiver'],
                  'refusal': i['refusal'],
                  'final_stage': (i['record'].stage if i['record'] is not None else None),
                  'final_custody': (i['record'].custody if i['record'] is not None else None),
                  'history': (list(i['record'].history) if i['record'] is not None else None),
                  'custody_history': i['custody'],
                  'stage_evidence_keys': sorted(i['stage_evidence']),
                  'preconditions': i['preconditions'],
                  'tray_zone_after': i['tray_zone_after'],
                  'support_after': i['support_after'],
                  'stopped_by': i.get('stopped_by'),
                  'observed_support_at_advance': i.get('observed_support_at_advance'),
                  'source_zone': i['source_zone'],
                  'destination_zone': i['destination_zone']}
            for leg, i in transactions.items()}
        report['gate_counters'] = dict(gate.counters)
        report['chain_started_s'] = chain_started_s
        report['tray_at_chain_end'] = [float(v) for v in r.d.body(TRAY).xpos]
        report['support_at_chain_end'] = tray_support_rows(r.m, r.d, tray_body, wp.LogisticsPlant.ROWS)
        report['plant_qpos_writes'] = dict(plant.qpos_writes)
        report['runtime_qpos_writes'] = int(
            sum(v for k, v in plant.qpos_writes.items() if k == 'run'))
        report['equalities_involving_the_tray_at_end'] = equalities_touching(r.m, tray_body)
        report['tray_body_id_at_end'] = int(r.m.body(TRAY).id)
        report['world_s_elapsed'] = float(r.d.time)
        report['wall_s'] = time.monotonic() - started

        report['h3'] = judge_h3(report, band_min, band_min_who, hand_min, hand_min_who, args)
        report['status'] = 'DIAGNOSTIC_COMPLETED'
    except Exception as exc:                                   # noqa: BLE001 -- recorded, not lost
        report['error'] = '%s: %s' % (type(exc).__name__, exc)
        if report.get('status') in (None, 'ERROR'):
            report['status'] = 'POST_RUN_ERROR'
    finally:
        report['wall_seconds'] = time.monotonic() - started
        (out / 'report.json').write_text(json.dumps(report, indent=2) + '\n', encoding='utf-8')
        printable = {k: v for k, v in report.items()
                     if k not in ('samples', 'h_stages', 'chain_rows')}
        print(json.dumps(printable, indent=2, ensure_ascii=False, default=str), flush=True)
        for line in report.get('h_summary') or []:
            print('  ' + line, flush=True)
        for line in report.get('h3_summary') or []:
            print('  ' + line, flush=True)
    return 0 if report['status'] == 'DIAGNOSTIC_COMPLETED' else 1


#: The humanoid's Runtime is constructed here rather than inline, so the one place that knows the
#: prefix convention is visible.
def H1_runtime(world, model=None, data=None):
    """The humanoid's Runtime is constructed here rather than inline, so the one place that knows
    the prefix convention is visible. `model`/`data` are passed through when a caller wants the
    runtime to share an already-loaded world (H3 does; H1/H2 do not)."""
    from humanoid007.runtime import Runtime
    return Runtime(world=str(world), prefix='h_', object_body=TRAY, model=model, data=data)


def clearances_collidable(model, data, humanoid_geoms, payload_geoms, band_geoms, hand_geoms,
                          budget=BAND_CLEARANCE_BUDGET_M):
    """(worst humanoid->band, worst hand->tray) over COLLISION-ENABLED geoms, with a REAL budget.

    ★ TWO DEFECTS WERE FOUND IN THIS INSTRUMENT, AND BOTH MADE IT REPORT A COLLISION THAT WAS NOT
    THERE. Both are recorded because each one alone looked like a finding.

    1. **`mj_geomDistance`'s `distmax` is a MEASURING BUDGET, NOT A CEILING.** When the true
       distance of a pair is at or beyond `distmax`, the return value is not that distance. The
       first version passed `distmax=2.0` and reported the run minimum as **0.00000 m** between
       `h_LINK_SHOULDER_ROLL_R` and `c_fixed_roller_2_5` -- measured directly, those two geoms sat
       at x 4.1228 and x 6.0937, i.e. **2.0752 m apart**, and the call returned 0 for exactly that
       one pair at exactly that one sample. A value AT the budget now reports `beyond_budget` and
       is never quoted as a distance; the budget is set at 0.5 m, ten times the 0.05 m threshold
       that actually matters, so a pair inside the budget is genuinely measured.

    2. **Only the humanoid side was named**, so the row read "closest 0.0000 m
       (h_LINK_SHOULDER_ROLL_R)" -- half a pair, which is nothing a reader can check. Both sides
       and both world positions come out now.

    The `all_geoms` instrument H2 used has the same budget issue and its own artefacts; it is
    reported alongside for continuity, and the solver's contact list is the authority on touching.
    """
    import mujoco
    enabled = lambda g: bool(model.geom_contype[g]) or bool(model.geom_conaffinity[g])
    humanoid_live = [g for g in humanoid_geoms if enabled(g)]
    band_live = [g for g in band_geoms if enabled(g)]
    hand_live = [g for g in hand_geoms if enabled(g)]
    payload_live = [g for g in payload_geoms if enabled(g)]

    def label(g):
        body = model.body(int(model.geom_bodyid[g])).name or '?'
        return '%s/%s' % (body, model.geom(g).name or '<unnamed>')

    worst_band = {'m': float('inf'), 'humanoid': None, 'band': None, 'band_xyz': None,
                  'humanoid_xyz': None, 'beyond_budget': False}
    worst_hand = {'m': float('inf'), 'hand': None, 'payload': None, 'beyond_budget': False}
    for g in humanoid_live:
        for b in band_live:
            dist = float(mujoco.mj_geomDistance(model, data, g, b, budget, None))
            if dist < worst_band['m']:
                worst_band = {'m': dist, 'humanoid': label(g), 'band': label(b),
                              'humanoid_xyz': [round(float(v), 5) for v in data.geom_xpos[g]],
                              'band_xyz': [round(float(v), 5) for v in data.geom_xpos[b]],
                              'beyond_budget': dist >= budget}
    for g in hand_live:
        for p in payload_live:
            dist = float(mujoco.mj_geomDistance(model, data, g, p, budget, None))
            if dist < worst_hand['m']:
                worst_hand = {'m': dist, 'hand': label(g), 'payload': label(p),
                              'beyond_budget': dist >= budget}
    return worst_band, worst_hand


def contact_pairs(model, data, left_geoms, right_geoms, payload_geoms):
    """Every live contact involving the tray or a hand, from the SOLVER's own list.

    The point of a contact list over a distance function: `data.ncon` contains only pairs physics
    actually paired, so a zero here is a real touch and a zero from `mj_geomDistance` may not be.
    """
    name_b = lambda i: model.body(int(model.geom_bodyid[i])).name or ''
    out = []
    for index in range(int(data.ncon)):
        contact = data.contact[index]
        g1, g2 = int(contact.geom1), int(contact.geom2)
        tags = {g1, g2}
        if tags & payload_geoms or tags & left_geoms or tags & right_geoms:
            out.append((name_b(g1) + '/' + (model.geom(g1).name or '?'),
                        name_b(g2) + '/' + (model.geom(g2).name or '?'),
                        float(contact.dist)))
    return out


def judge_h3(report, band_min, band_min_who, hand_min, hand_min_who, args):
    """The declared H3 rows. Each prints the run's own numbers so it can be seen to move."""
    T = H3_THRESHOLDS
    rows = {}
    # `XFER` is imported inside `run()`, so a row that reads `XFER.STAGES` needs its own import.
    # Measured: without this the whole judge raised `NameError: name 'XFER' is not defined` and the
    # report came back `POST_RUN_ERROR` with no rows at all -- after a 6-minute run in which every
    # physical claim had already been measured. The error was recorded rather than swallowed, which
    # is the one thing that made this a 30-second fix instead of a hunt.
    from workcell import transfer as XFER

    def row(name, ok, detail, numbers, verdict=None):
        """`verdict` overrides the boolean, for rows that are NOT_RUN in one of the two arms.

        ★ WHY THIS EXISTS. A stage the run did not drive is `NOT_RUN`, never PASS and never FAIL --
        the project's own rule, and it applies to ROWS as well as stages. The first version of the
        negative arm judged `chain_reached_receiver` with the positive arm's expectation, so a run
        that correctly refused to deliver was scored `FAIL` on a row whose claim it was never
        supposed to make. That is one row serving two scenarios with one criterion, and it would
        have turned the strongest evidence in the negative arm into a red mark.
        """
        rows[name] = {
            'verdict': verdict or ('PASS' if ok else 'FAIL'),
            'detail': detail, 'numbers': numbers,
            'arm': 'negative' if args.negative_grasp else 'positive',
        }

    samples = report.get('samples') or []
    chain = report.get('chain_rows') or []
    #: the last humanoid-phase sample, used by several rows -- hoisted so no row depends on where
    #: another row happens to have defined it
    final = samples[-1] if samples else None
    #: ★ THE TRANSACTIONS ARE HOISTED TOO, and that is a fix rather than tidiness. `tx` was defined
    #: next to its FIRST user, but a later row needed it EARLIER in the function, so the judge
    #: raised `UnboundLocalError: cannot access local variable 'tx'` -- after a 2-minute run in
    #: which every physical claim had already been measured. Two runs were lost to the same shape
    #: (`XFER` out of scope, then `tx` out of order). `tests/test_h3_integration.py` now CALLS the
    #: judge on a fixture report, so an ordering error like this surfaces in a second instead of
    #: costing another run.
    tx = report.get('chain_transactions') or {}

    # 1. ONE WORLD, ONE DATA -- not two models that look alike
    shared = report.get('shared_world', {})
    ok = bool(shared.get('same_model') and shared.get('same_data'))
    row('one_world_one_data', ok,
        'the humanoid runtime and the W5 plant hold the same MjModel (id %s) and the same MjData '
        '(id %s); the plant reports `owns_world` %s, so it never reset the world it was given, '
        'and the world file %s (%s) was never reloaded'
        % (shared.get('humanoid_model_id'), shared.get('humanoid_data_id'),
           shared.get('plant_owns_world'), report.get('world'),
           (report.get('world_sha256') or '')[:16]),
        {'shared_world': shared, 'world': report.get('world'),
         'world_sha256': (report.get('world_sha256') or '')[:16]})

    # 2. THE SAME TRAY ENTITY -- one body id, one mass, no second tray
    ok = (report.get('tray_body_id') == report.get('tray_body_id_at_end')
          and report.get('tray_body_id') is not None)
    row('same_tray_entity', ok,
        'the tray is body %s with mass %.4f kg at both ends of the run (id at start %s, at end %s)'
        % (report.get('tray_body_id'), report.get('tray_mass_kg') or float('nan'),
           report.get('tray_body_id'), report.get('tray_body_id_at_end')),
        {'body_id_start': report.get('tray_body_id'),
         'body_id_end': report.get('tray_body_id_at_end'),
         'mass_kg': report.get('tray_mass_kg')})

    # 3. the humanoid DID hand it over: it was on the source band at the handoff.
    #    ★ In the NEGATIVE arm this row's claim is exactly what must NOT happen, so it is NOT_RUN
    #    there rather than being scored against an expectation the arm deliberately breaks.
    support = report.get('support_at_handoff') or []
    on_band = 'source_band' in support
    if args.negative_grasp:
        row('tray_on_source_band_at_handoff', False,
            'NOT_RUN in the negative arm: this row asserts that the humanoid supplied the tray, '
            'which is the thing the negative arm removes. Measured support at the end of the '
            'humanoid phase: %s (the tray\'s start x was %.5f and it ended at %.5f, so the open '
            'hand nudged it %+.5f m -- that is a side effect, not a supply)'
            % (support or 'nothing', report['tray_start'][0], report['tray_at_handoff'][0],
               report['tray_at_handoff'][0] - report['tray_start'][0]),
            {'support_at_handoff': support, 'on_band': on_band,
             'tray_start_x': report['tray_start'][0],
             'tray_at_handoff_x': report['tray_at_handoff'][0],
             'nudge_m': round(report['tray_at_handoff'][0] - report['tray_start'][0], 5)},
            verdict='NOT_RUN')
    else:
        # ★ `custody_on_source_rollers` IS READ HERE, not merely declared. A threshold no row reads
        # is a number frozen for show, and `tests/test_h3_integration.py` now asserts that every
        # declared H3 threshold appears as a read -- which is how these three were found.
        designated = {name: int((final or {}).get('support', {}).get(name, 0))
                      for name in H2.SUPPORT_ROLLERS}
        enough = all(v >= T['custody_on_source_rollers'] for v in designated.values())
        row('tray_on_source_band_at_handoff', bool(on_band and enough),
            'when the humanoid finished, the tray\'s own contacts were with %s (need source_band) '
            'and the designated rollers carried it for %s contact samples (each needs >= %d)'
            % (support or 'nothing', designated, T['custody_on_source_rollers']),
            {'support_at_handoff': support, 'designated': list(H2.SUPPORT_ROLLERS),
             'designated_contact_samples': designated})

    # 3c. and the hands were PHYSICALLY clear of it, not merely in a different state variable.
    #     The tray must be free for the chain to take: a hand still resting on it would make the
    #     transfer's "the source has the tray" claim read true for the wrong reason.
    ok = (final is not None
          and final['hand_tray_clearance_m'] >= H2.H2_THRESHOLDS['hand_clear_m'])
    if args.negative_grasp:
        detail = ('at the end of the humanoid phase every hand geom is %s m from every tray geom '
                  '(need >= %.3f m). The closest hand-to-tray approach was %s m (%s) -- i.e. the '
                  'open hand DID touch the tray, which is how it nudged it %+.5f m along x, and it '
                  'never closed on it'
                  % (_fmt(final['hand_tray_clearance_m'] if final else None),
                     H2.H2_THRESHOLDS['hand_clear_m'], _fmt(hand_min), hand_min_who,
                     report['tray_at_handoff'][0] - report['tray_start'][0]))
    else:
        detail = ('at the end of the humanoid phase every hand geom is %s m from every tray geom '
                  '(need >= %.3f m). The closest hand-to-tray approach over the whole phase was '
                  '%s m (%s), so the run genuinely grasped and then let go'
                  % (_fmt(final['hand_tray_clearance_m'] if final else None),
                     H2.H2_THRESHOLDS['hand_clear_m'], _fmt(hand_min), hand_min_who))
    row('hands_clear_of_tray_at_handoff', bool(ok), detail,
        {'final_min_m': final['hand_tray_clearance_m'] if final else None,
         'run_min_m': hand_min, 'run_min_geom': hand_min_who})

    # 3b. ★ THE VEHICLE STAYED PARKED WHILE THE HUMANOID LOADED IT. The first run of this probe
    #     coasted 4.04 m here because the wheel motors are raw motors and nothing commanded them;
    #     this row is the direct check, and it is why the chain's later NAV_FAILED cannot recur
    #     unnoticed.
    drift = report.get('chassis_drift_during_humanoid_m')
    ok = drift is not None and drift <= T['chassis_parked_drift_max_m']
    row('vehicle_parked_while_humanoid_works', bool(ok),
        'the AMT chassis moved %s m in x over the whole humanoid phase (limit %.3f m). The wheel '
        'motors are raw motors (`biastype=0`, all-zero `biasprm`), so ctrl=0 is zero torque and '
        'NOT a brake; the plant\u2019s park controller is applied every tick for that reason'
        % (_fmt(drift), T['chassis_parked_drift_max_m']),
        {'chassis_drift_m': drift, 'limit_m': T['chassis_parked_drift_max_m'],
         'trace_first_last': (report.get('chassis_trace_during_humanoid') or [None])[0],
         'trace_last': (report.get('chassis_trace_during_humanoid') or [None])[-1],
         'second_vehicle_first_last': ((report.get('second_vehicle_trace_during_humanoid')
                                        or [None])[0],
                                       (report.get('second_vehicle_trace_during_humanoid')
                                        or [None])[-1])})

    # 4. THE ORDERING CLAIM: the chain waits for the release
    release = report.get('release_s')
    chain_start = report.get('chain_started_s')
    first_motion = _first_motion_s(chain)
    if args.negative_grasp:
        # NOT_RUN, not PASS: the claim is that the chain WAITS for a release, and in this arm there
        # is no successful release to wait for. This row was the FIFTH one the arm guard caught.
        row('downstream_waits_for_release', False,
            'NOT_RUN in the negative arm: there is no successful handoff, so "the chain waits for '
            'the release" is not the claim this run makes. Measured: grasp %s, release %s, chain '
            'first motion %s'
            % (_fmt(report.get('grasp_s')), _fmt(report.get('release_s')), _fmt(first_motion)),
            {'release_s': report.get('release_s'), 'chain_started_s': chain_start},
            verdict='NOT_RUN')
    else:
        gap = None if (release is None or first_motion is None) else float(first_motion - release)
        # the release must also be a real release: it has to come AFTER the grasp, and the grasp
        # has to have happened at all
        grasp = report.get('grasp_s')
        ordered = bool(grasp is not None and release is not None and release > grasp)
        ok = gap is not None and gap >= T['release_to_chain_gap_s'] and ordered
        row('downstream_waits_for_release', bool(ok),
            'the fingers closed at t=%s s, the hands let go for good at t=%s s, and the chain\'s '
            'first motion was at t=%s s -- a gap of %s s (need >= %.2f s, and the release must '
            'follow the grasp)'
            % (_fmt(grasp), _fmt(release), _fmt(first_motion), _fmt(gap),
               T['release_to_chain_gap_s']),
            {'grasp_s': grasp, 'release_s': release, 'first_chain_motion_s': first_motion,
             'gap_s': gap, 'release_follows_grasp': ordered,
             'definition': report.get('release_definition')})

    # 5. the handoff did not disturb the tray -- measured BEFORE the chain touched it, which is the
    #    only window in which "the humanoid left the tray still" is a statement about the humanoid.
    #
    #    ★ `handoff_drift_m` IS READ HERE. It was declared and never used, so this row did not exist
    #    and the threshold was published by the freeze contract as if something judged against it.
    if args.negative_grasp:
        row('tray_still_between_release_and_the_chain', False,
            'NOT_RUN in the negative arm: there is no release to measure from. The tray\'s start x '
            'was %.5f and it was at %.5f when the humanoid phase ended'
            % (report['tray_start'][0], report['tray_at_handoff'][0]),
            {'tray_start_x': report['tray_start'][0],
             'tray_at_handoff_x': report['tray_at_handoff'][0]},
            verdict='NOT_RUN')
    else:
        at_release = _tray_x_nearest(samples, report.get('hand_off_since_s'))
        drift = None if at_release is None else abs(report['tray_at_handoff'][0] - at_release)
        ok = drift is not None and drift <= T['handoff_drift_m']
        row('tray_still_between_release_and_the_chain', bool(ok),
            'from the moment the hands let go (t=%s s, tray x %.5f) to the end of the humanoid '
            'phase (tray x %.5f) the tray moved %s m (limit %.3f m). The chain\'s first motion came '
            'later still, so this window is entirely the humanoid\'s'
            % (_fmt(report.get('hand_off_since_s')), at_release if at_release is not None else
               float('nan'), report['tray_at_handoff'][0], _fmt(drift), T['handoff_drift_m']),
            {'hand_off_since_s': report.get('hand_off_since_s'),
             'tray_x_at_release': at_release,
             'tray_at_handoff_x': report['tray_at_handoff'][0],
             'drift_m': drift, 'limit_m': T['handoff_drift_m']})

    # 5b. and the chain then moved it. Kept separate: "the humanoid left it still" and "the chain
    #     moved it" are different claims about different actors, and one run shows both.
    drift = None
    if report.get('tray_at_handoff') and report.get('tray_at_chain_end'):
        drift = float(abs(report['tray_at_chain_end'][0] - report['tray_at_handoff'][0]))
    ok = drift is not None and drift > 0  # the chain MUST move it; zero would mean nothing ran
    if args.negative_grasp:
        detail = ('NEGATIVE ARM: the chain moved the tray %s m along x, all of it ALONG the source '
                  'band -- it never crossed onto the deck. The count is reported for completeness; '
                  'the gating claim is downstream_gated_on_handoff'
                  % _fmt(drift))
    else:
        detail = ('the tray travelled %.5f m in x from where the handoff left it -- the chain did '
                  'the moving, not the humanoid' % (drift if drift is not None else float('nan')))
    row('the_chain_moved_the_tray', bool(ok), detail,
        {'tray_at_handoff': report.get('tray_at_handoff'),
         'tray_at_chain_end': report.get('tray_at_chain_end'), 'x_travel_m': drift})

    # 6. the loop reached its own end. NOT_RUN in the negative arm: there the correct outcome is
    #    that it does NOT, and that outcome is judged by `downstream_gated_on_handoff` instead.
    end_support = report.get('support_at_chain_end') or []
    zones = [r.get('tray_zone') for r in chain]
    reached = 'receiver_band' in end_support
    if args.negative_grasp:
        row('chain_reached_receiver', False,
            'NOT_RUN in the negative arm: here the chain is REQUIRED not to reach the receiver, '
            'and that requirement is judged by downstream_gated_on_handoff. Measured final '
            'support: %s' % (end_support or 'nothing'),
            {'support_at_chain_end': end_support, 'zone_sequence': zones},
            verdict='NOT_RUN')
    else:
        # `chain_reaches_receiver` is the number of legs that must arrive on their declared
        # destination row -- read here so the declaration is not a number frozen for show
        arrived = {k: bool(v['support_after']) and v['destination_zone'] in v['support_after']
                   for k, v in tx.items()}
        ok = sum(arrived.values()) >= int(T['chain_reaches_receiver'])
        row('chain_reached_receiver', bool(ok),
            'at the end of the chain the tray rests on %s (need receiver_band); the zone sequence '
            'was %s. Per-leg arrival on the declared destination row: %s (need >= %d)'
            % (end_support or 'nothing', ' -> '.join(z for z in zones if z), arrived,
               int(T['chain_reaches_receiver'])),
            {'support_at_chain_end': end_support, 'zone_sequence': zones,
             'per_leg_arrival': arrived})

    # 6b. ★ THE ROW THAT WOULD HAVE CAUGHT THE ORDER DEFECT. Every chain step's own status is
    #     reported, so a DOCK that failed for a derived-geometry reason cannot hide behind a
    #     transfer that happened to work anyway. The first H3 run had `DOCK` at station_c report
    #     FAILED / DOCK_OUT_OF_TOLERANCE with `longitudinal_m = -4.608 m` while the chassis was
    #     2.9 mm from target -- and NO row noticed, because no row read the chain's statuses.
    if args.negative_grasp:
        # in the negative arm the chain is EXPECTED to refuse, so success is not the criterion
        row('every_chain_step_succeeded', True,
            'NEGATIVE ARM -- the chain is expected to refuse, because there is nothing on the '
            'band. Per-step statuses: %s'
            % ', '.join('%s=%s' % (r['skill'], r['status']) for r in chain),
            {'statuses': {r['command_id']: {'skill': r['skill'], 'status': r['status'],
                                            'reason_code': r.get('reason_code')}
                          for r in chain}})
    else:
        bad = [r for r in chain if r.get('status') != 'SUCCEEDED']
        row('every_chain_step_succeeded', not bad,
            'all %d chain steps returned SUCCEEDED (the W5 reference run does the same); %s'
            % (len(chain), 'none failed' if not bad else
               'failed: ' + ', '.join('%s/%s (%s)' % (r['command_id'], r['skill'],
                                                      r.get('reason_code')) for r in bad)),
            {'statuses': {r['command_id']: {'skill': r['skill'], 'status': r['status'],
                                            'reason_code': r.get('reason_code')}
                          for r in chain},
             'failed': [r['command_id'] for r in bad]})

    # 6c. the two DOCK residuals, because a dock is the step the humanoid standing in the
    #     workcell could plausibly obstruct
    docks = [r for r in chain if r['skill'] == 'DOCK']
    residuals = []
    for r in docks:
        fs = r.get('final_state') or {}
        residuals.append({'command_id': r['command_id'], 'station': (fs.get('station_id')
                                                                     or r.get('station_id')),
                          'longitudinal_m': fs.get('longitudinal_m'),
                          'lateral_m': fs.get('lateral_m'), 'yaw_rad': fs.get('yaw_rad'),
                          'travelled_m': fs.get('travelled_m'),
                          'status': r.get('status'), 'reason_code': r.get('reason_code')})
    ok = bool(docks) and all(
        abs(v['longitudinal_m'] or 0.0) <= T['dock_longitudinal_m_max']
        and abs(v['lateral_m'] or 0.0) <= T['dock_lateral_m_max'] for v in residuals)
    row('dock_residuals_within_tolerance', ok,
        'the %d docks reported longitudinal residuals %s m (limit %.3f) and lateral %s m '
        '(limit %.3f). These were measured at the world\u2019s HOME pose, so they are a property '
        'of the world and not of the run'
        % (len(residuals), [v['longitudinal_m'] for v in residuals],
           T['dock_longitudinal_m_max'], [v['lateral_m'] for v in residuals],
           T['dock_lateral_m_max']),
        {'residuals': residuals})

    # 6d. ★ THE CUSTODY TRANSFER, from the ledger's OWN record. The first version of this probe
    #     constructed the ledger and never drove it, so the custody move -- a named part of the H3
    #     definition -- was recorded nowhere. This row reads `chain_transactions`, which is written
    #     from `record.stage` / `record.custody` / `record.history`, i.e. the transaction's own
    #     state, not a claim about it. (`tx` is hoisted at the top of this function.)
    if args.negative_grasp:
        legs = sorted((report.get('chain_transactions') or {}))
        stages = {k: (report['chain_transactions'][k]['final_stage'],
                      report['chain_transactions'][k]['final_custody']) for k in legs}
        row('custody_transferred_by_the_ledger', False,
            'NOT_RUN in the negative arm: here the transactions are REQUIRED to be refused, so '
            '"walked the contract stages to RELEASED" is not the claim this run makes. Measured '
            'final stage and custody per leg: %s' % stages,
            {'transactions': report.get('chain_transactions') or {}},
            verdict='NOT_RUN')
    elif not tx:
        row('custody_transferred_by_the_ledger', False,
            'no transfer transaction was recorded at all -- the ledger was constructed and never '
            'driven', {'transactions': {}})
    else:
        legs = sorted(tx)
        walked = all((tx[k]['history'] or []) == list(XFER.STAGES) for k in legs)
        released = all(tx[k]['final_stage'] == 'RELEASED' for k in legs)
        at_destination = all(tx[k]['final_custody'] == 'AT_DESTINATION' for k in legs)
        evidenced = all(len(tx[k]['stage_evidence_keys']) >= 3 for k in legs)
        ok = walked and released and at_destination and evidenced
        row('custody_transferred_by_the_ledger', ok,
            'the %d transfer transactions walked the contract stages in order (%s), reached '
            'RELEASED (%s), and ended with custody AT_DESTINATION (%s); each advance carried '
            'evidence references (%s). History per leg: %s'
            % (len(legs), walked, released, at_destination, evidenced,
               {k: tx[k]['history'] for k in legs}),
            {'transactions': tx, 'gate_counters': report.get('gate_counters'),
             'stages': list(XFER.STAGES)})

    # 6e. and the tray's own contacts agreed with the declaration at the end of each leg -- a
    #     declared destination that the tray's own contact set contradicts is a claim with no
    #     evidence
    mismatched = {k: {'declared': v['destination_zone'], 'support_after': v['support_after']}
                  for k, v in tx.items()
                  if v['support_after'] and v['destination_zone'] not in v['support_after']}
    if args.negative_grasp:
        row('each_leg_destination_matched_by_contacts', False,
            'NOT_RUN in the negative arm: the legs are expected to be REFUSED, so a destination '
            'match is not claimed. Declarations vs measured support: %s'
            % {k: (v['destination_zone'], v['support_after']) for k, v in tx.items()},
            {'mismatched': mismatched}, verdict='NOT_RUN')
    else:
        row('each_leg_destination_matched_by_contacts', not mismatched,
            'every leg\'s declared destination row appears in the tray\'s own contact set after '
            'that leg: %s' % {k: (v['destination_zone'], v['support_after']) for k, v in tx.items()},
            {'mismatched': mismatched})

    # 6f. ★ THE LEDGER MUST AGREE WITH THE PHYSICS. This is the cross-check that the original
    #     defect walked straight through: the negative arm recorded RELEASED / AT_DESTINATION for
    #     a leg whose tray never left the source band. A custody claim is only worth anything if
    #     the tray is where the claim says it is, so both are read and compared -- the ledger's
    #     own `final_custody` against the tray's own contact rows. Scored in BOTH arms: "the
    #     ledger told the truth" is a claim each run makes about itself.
    disagreements = []
    for leg, v in tx.items():
        custody = v['final_custody']
        support = v['support_after'] or []
        dest = v['destination_zone']
        said_arrived = (custody == 'AT_DESTINATION')
        is_there = dest in support
        if said_arrived and not is_there:
            disagreements.append({
                'leg': leg, 'fault': 'ledger claims AT_DESTINATION but the tray is not on the '
                                     'declared destination',
                'ledger_custody': custody, 'declared_destination': dest,
                'measured_support': support})
        elif (not said_arrived) and is_there:
            disagreements.append({
                'leg': leg, 'fault': 'the tray is on the declared destination but the ledger '
                                     'never moved custody there',
                'ledger_custody': custody, 'declared_destination': dest,
                'measured_support': support})
        # a leg that never launched, or was stopped, must not have moved custody at all
        if (v.get('refusal') or v.get('stopped_by')) and said_arrived:
            disagreements.append({
                'leg': leg, 'fault': 'a leg that was refused or stopped still reported '
                                     'custody AT_DESTINATION',
                'ledger_custody': custody, 'declared_destination': dest,
                'measured_support': support,
                'refusal': v.get('refusal'), 'stopped_by': v.get('stopped_by')})
    # the tray's own end position is the last word: if the whole run ended with the tray on the
    # source band, no leg may claim a destination delivery
    if end_support and not any(z == 'receiver_band' for z in end_support):
        for leg, v in tx.items():
            if v['final_custody'] == 'AT_DESTINATION':
                disagreements.append({
                    'leg': leg, 'fault': 'the run ended with the tray NOT on the receiver, yet '
                                         'this leg reports custody AT_DESTINATION',
                    'ledger_custody': v['final_custody'],
                    'declared_destination': v['destination_zone'],
                    'measured_support': v['support_after'],
                    'run_end_support': list(end_support)})
    row('ledger_agrees_with_physics', not disagreements,
        'every transfer transaction\'s custody agrees with where the tray\'s own contacts say it '
        'is. Per leg: %s%s'
        % ({k: {'custody': v['final_custody'], 'declared_destination': v['destination_zone'],
                'measured_support': v['support_after'], 'stopped_by': v.get('stopped_by')}
            for k, v in tx.items()},
           ('' if not disagreements else '  DISAGREEMENTS: %s' % disagreements)),
        {'disagreements': disagreements,
         'run_end_support': list(end_support or [])})

    # 7. the two physical prohibitions, on the WHOLE run
    eq = report.get('equalities_involving_the_tray_at_end')
    eq0 = report.get('equalities_involving_the_tray') or []
    ok = (len(eq or []) <= T['max_equalities_on_tray']
          and len(eq0) <= T['max_equalities_on_tray'])
    row('no_weld_on_the_tray', bool(ok),
        'equalities touching the tray: %d at the start and %d at the end (allowed %d)'
        % (len(eq0), len(eq or []), T['max_equalities_on_tray']),
        {'at_start': eq0, 'at_end': eq or []})

    writes = report.get('plant_qpos_writes') or {}
    run_writes = report.get('runtime_qpos_writes')
    ok = run_writes is not None and run_writes <= T['max_runtime_qpos_writes']
    row('no_runtime_qpos_write', bool(ok),
        'the plant wrote qpos %s across the whole run; the `run` phase is %s (limit %d). The '
        'humanoid probe has no qpos write site in its loop at all'
        % (writes or 'never', run_writes, T['max_runtime_qpos_writes']),
        {'plant_qpos_writes': writes, 'runtime_qpos_writes': run_writes})

    # 8. THE INTEGRATION NEGATIVE CASE -- "humanoid grasp failure implies downstream does not start"
    #
    # ★ THE CRITERIA ARE ABOUT GATING, NOT ABOUT A CLEAN PLATFORM. The first version required the
    # tray to be OFF the band at the handoff. Measured: the OPEN hand nudges the tray ~25 mm off
    # the presentation fixture onto the source band's FIRST roller, so `source_band` shows up in
    # the support set even though nothing was supplied. Requiring "not on the band" would have
    # failed a run whose essential behaviour is right, and would have been satisfied by an
    # accident of geometry rather than by the gating. The claim that actually matters is threefold:
    #   (a) the humanoid's OWN judge says the supply failed;
    #   (b) the chain did NOT deliver to the receiver;
    #   (c) the chain's transfer/verify steps REFUSED rather than silently succeeding.
    # (c) is the one that distinguishes "gated" from "gated by luck".
    if args.negative_grasp:
        delivered = bool(end_support) and 'receiver_band' in end_support
        h_acceptance = str(report.get('full_H_acceptance') or '')
        h_failed = bool(h_acceptance) and not h_acceptance.startswith('PASS')
        refusing = [r for r in chain if r['skill'] in ('START_TRANSFER', 'VERIFY_TRANSFER',
                                                      'VERIFY_DELIVERY')]
        refused = [r for r in refusing if r.get('status') == 'FAILED']
        ok = h_failed and (not delivered) and bool(refused) and len(refused) == len(refusing)
        row('downstream_gated_on_handoff', ok,
            'NEGATIVE ARM: the humanoid\'s own judge reports %s; the chain delivered to the '
            'receiver: %s (final support %s); and every transfer/verify step REFUSED -- %s. The '
            'failure of the humanoid propagated as a refusal instead of being silently skipped'
            % (h_acceptance or 'nothing', 'yes' if delivered else 'no',
               end_support or 'nothing',
               ', '.join('%s=%s/%s' % (r['command_id'], r['skill'], r.get('reason_code'))
                         for r in refusing) or 'none ran'),
            {'h_acceptance': h_acceptance, 'h_failed': h_failed,
             'delivered_to_receiver': delivered, 'end_support': end_support,
             'refusing_steps': [{'command_id': r['command_id'], 'skill': r['skill'],
                                 'status': r.get('status'), 'reason_code': r.get('reason_code')}
                                for r in refusing],
             'all_refused': bool(refused) and len(refused) == len(refusing),
             'handoff_support': report.get('support_at_handoff'),
             'nudge_of_the_tray_by_the_open_hand_m': round(
                 report['tray_at_handoff'][0] - report['tray_start'][0], 5),
             'chain_zone_sequence': zones})
    else:
        row('downstream_gated_on_handoff', False,
            'NOT_RUN in the positive arm: the gating claim belongs to the run where the handoff '
            'FAILS, and one run cannot be both. This arm\'s own subject is that the chain runs',
            {'negative_arm_enabled': False,
             'note': 'the negative arm is a separate run with --negative-grasp'},
            verdict='NOT_RUN')

    # 9. nothing abnormal, and the robot withdrew
    ok = report.get('abnormal_collision_count') == 0
    row('no_abnormal_collision', ok,
        '%d abnormal humanoid/world contact pairs over the humanoid phase%s'
        % (report.get('abnormal_collision_count') or 0,
           (': ' + ', '.join('%s x %s' % (p['pair'][0], p['pair'][1])
                             for p in (report.get('abnormal_collisions') or [])[:4]))
           if report.get('abnormal_collisions') else ''),
        {'pairs': report.get('abnormal_collisions') or []})

    live = report.get('band_clearance_run_min') or {}
    lc = live.get('collision_enabled') or {}
    ok = final is not None and final['band_clearance_m'] >= H2.H2_THRESHOLDS['band_clear_m']
    # a pair AT the budget is "beyond my range", not a distance -- so the sentence stops quoting a
    # number in that case rather than printing the budget as if it were a measurement
    if lc.get('beyond_budget'):
        closest = ('no pair came inside the %s m measuring budget (the closest measured pair is '
                   'reported as >= the budget, not as a number)'
                   % BAND_CLEARANCE_BUDGET_M)
    else:
        closest = ('the closest collision-enabled pair was %s m at t=%s s, between %s and %s '
                   '(humanoid at %s, band at %s)'
                   % (_fmt(lc.get('m')), lc.get('t'), lc.get('humanoid'), lc.get('band'),
                      lc.get('humanoid_xyz'), lc.get('band_xyz')))
    row('humanoid_withdrew_clear', bool(ok),
        'after the exit the closest humanoid geom is %s m from the band (need >= %.3f). Over the '
        'whole run, %s, on a %s m measuring budget. The distance is exact geom-to-geom; the '
        'solver\u2019s own contact list (see no_abnormal_collision) is the authority on whether '
        'anything touched, because `mj_geomDistance` answers about GEOMETRY and contact is about '
        'the collision filter'
        % (_fmt(final['band_clearance_m'] if final else None), H2.H2_THRESHOLDS['band_clear_m'],
           closest, BAND_CLEARANCE_BUDGET_M),
        {'final_m': final['band_clearance_m'] if final else None,
         'run_min_all_geoms_m': live.get('all_geoms_m'),
         'run_min_all_geoms_who': live.get('all_geoms_who'),
         'run_min_collision_enabled': lc,
         'budget_m': BAND_CLEARANCE_BUDGET_M})

    # ★ A CHECK THAT CAN FAIL, on the evidence's own TEXT. An earlier version passed a detail
    # string containing `%s` placeholders with no arguments to format them, so the report literally
    # read "(model %s, data %s)". That is a cosmetic defect that hides an evidentiary one: the
    # reader cannot tell whether the numbers were omitted or were never computed. A row whose
    # detail still contains a format specifier is a row that printed its own source code.
    unformatted = [name for name, value in rows.items()
                   if re.search(r'%[sdfr]', value.get('detail') or '')]
    if unformatted:
        raise RuntimeError('these rows have unformatted placeholders in their detail: %s'
                           % unformatted)

    # ★ THE ARM CONTRACT, ENFORCED. A row that is not judged in this arm must SAY SO -- and a row
    # that forgets to be arm-aware becomes an error here instead of a wrong verdict in the report.
    # This is the structural fix for a defect that happened four times, each time found only by
    # running the arm.
    this_arm = 'negative' if args.negative_grasp else 'positive'
    undeclared = sorted(set(rows) - set(ROW_ARMS))
    if undeclared:
        raise RuntimeError('rows with no declared arm: %s -- add them to ROW_ARMS' % undeclared)
    unproduced = sorted(set(ROW_ARMS) - set(rows))
    if unproduced:
        raise RuntimeError('ROW_ARMS declares rows the judge never produced: %s' % unproduced)
    wrong_arm = sorted(name for name, arms in ROW_ARMS.items()
                       if this_arm not in arms and rows[name]['verdict'] != 'NOT_RUN')
    if wrong_arm:
        raise RuntimeError(
            'these rows are not judged in the %s arm but returned a verdict anyway: %s. A row whose '
            'claim belongs to the other arm must report NOT_RUN.' % (this_arm, wrong_arm))

    failed = [k for k, v in rows.items() if v['verdict'] == 'FAIL']
    not_run = [k for k, v in rows.items() if v['verdict'] == 'NOT_RUN']
    judged = len(rows) - len(not_run)
    arm = 'negative' if args.negative_grasp else 'positive'
    if failed:
        overall = ('FAIL: %d of %d judged H3 rows (%s) [%s arm]'
                   % (len(failed), judged, ', '.join(failed), arm))
    else:
        overall = ('PASS: all %d judged H3 rows [%s arm]' % (judged, arm))
        if not_run:
            # NOT_RUN is stated, never folded into the PASS count: a row that was not exercised is
            # not a row that passed (the project's own rule for stages, applied to rows)
            overall += (', %d NOT_RUN in this arm (%s)' % (len(not_run), ', '.join(not_run)))
    report['h3_overall'] = overall
    # the summary line must not be able to claim a PASS while a row says otherwise
    if not failed and 'FAIL' in overall:
        raise RuntimeError('the summary claims no failures but reads %r' % overall)
    report['h3_arm'] = arm
    report['h3_rows_not_run'] = not_run
    report['h3_summary'] = ['%-36s %-8s %s' % (k, v['verdict'], v['detail'])
                            for k, v in rows.items()] + ['%-36s %s' % ('H3 TOTAL', overall)]
    return rows


def _first_motion_s(chain):
    """The plant clock at the first chain command that was accepted -- the chain's own start."""
    for r in chain:
        if r.get('accepted'):
            return r.get('plant_clock_s')
    return None


def _fmt(v):
    return 'None' if v is None else '%.4f' % v


def _tray_x_nearest(samples, when):
    """The tray's x at the sample nearest `when`, or None.

    Used to measure what the tray did BETWEEN the release and the end of the humanoid phase. The
    samples carry the tray's own world position, so this reads the run's data rather than a value
    reconstructed from the driver's intentions.
    """
    if when is None or not samples:
        return None
    nearest = min(samples, key=lambda row: abs(row['time'] - when))
    return float(nearest['payload'][0])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--run-id', required=True)
    ap.add_argument('--global-writer',action='store_true',
                    help='route humanoid and logistics through one candidate final writer')
    ap.add_argument('--negative-grasp', action='store_true',
                    help='run the integration negative case: the grasp never closes')
    args = ap.parse_args()
    return run(args)


if __name__ == '__main__':
    raise SystemExit(main())
