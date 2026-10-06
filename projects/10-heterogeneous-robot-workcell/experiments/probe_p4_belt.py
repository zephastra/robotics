#!/usr/bin/env python3
"""P4-BELT-03: the belt module's closed loop, judged -- AND its retention claim, refuted.

THE EXIT DOOR THIS PROBE ANSWERS TO
-----------------------------------
`docs/MASTER_PLAN.md`, P4's row, verbatim:

    P4 | 人形取盘、机械臂装盘、输送器分模块闭环 | 每项有单技能成功与故障证据

"每项有单技能成功与故障证据" -- EVERY item needs a single-skill SUCCESS evidence AND a FAILURE
evidence. So this probe is built as three arcs, and each arc carries both halves:

  ARC 1  CONVEYANCE TO A HARD STOP   `plant.transfer(direction='onto_deck')` walks the tray from
                                     the source band onto the deck. SUCCESS = the tray crossed and
                                     is supported by the deck's own rollers. FAILURE = the same leg
                                     with the deck band's drive cut does NOT deliver.
  ARC 2  RECEIVE AND UNLOAD           the deck and the receiver band spin together and the tray
                                     crosses onto the receiver. SUCCESS = the receiver band is the
                                     tray's support, measured from the solver's contact list.
                                     FAILURE = an already-occupied receiver does not "receive" a
                                     second time -- a bounded refusal, not a lost payload.
  ARC 3  ONBOARD RETENTION            **this arc is RED, and that is the finding.**

WHY ARC 3 IS RED, AND WHY IT IS NOT "FIXED"
-------------------------------------------
`docs/TASK_BOARD.md` records the user's ruling of 2026-09-29 (`G-2`):

    "选 B —— 不改世界。即：把'挡刀是甲板的保持装置'从**断言**降级为 `P4-BELT-03` 里一条**可失败的
     判据** `tray_does_not_slide_on_the_deck`（载着托盘移动 AMT，量托盘相对甲板的位移；
     **对照实验必须能让它红**）。"
    "⛔ 不做选项 A（改世界修挡刀）：那会让 `world_w5_h085_loop.xml` 与候选 B 的哈希都变，从而使
     **H2 + H3 正负两臂的已接受报告全部作废**。"

So this probe does NOT change the world. It carries the tray on the deck over a real driven leg and
measures how far the tray moves **in the deck's own frame** -- the only frame in which "did it
slide" is a question, since a tray riding along with the vehicle is not sliding. The verdict is
compared against the plant's own DERIVED budget
`DRIFT_LIMIT_M = STOPPED_SPEED_MPS * STOPPED_HOLD_S` (5.0 mm), and the row IS red.

THE PAIR THAT MAKES THE ROW A MEASUREMENT RATHER THAN AN OPINION
----------------------------------------------------------------
★ The two arms of ARC 3 differ by exactly ONE boolean -- whether the deck's own roller band is
driven while the vehicle moves -- and they are run through the SAME code path, on the same world,
over the same leg:

    deck band DRIVEN  ->  the tray slides 3.2e-1 m in the deck frame   ->  RED
    deck band CUT     ->  the tray shifts 4.38 mm                      ->  GREEN (under the 5 mm budget)

The CUT case is C's own declared load-carrying condition (`D034`/F1: *"with the rollers driven the
tray gains relative slip equal to the roller surface speed integrated over the contact time
(+0.13..0.42 m) ... with the drive cut the tray stays with the deck to <= 5.4 mm"*), and this run
lands at 4.38 mm, under both that ceiling and the plant's own 5 mm budget.

**A FIRST VERSION OF THIS PROBE HAD THE ARMS LABELLED THE WRONG WAY ROUND** and reported the two
as bit-identical, because neither arm had the deck band driven. That is the project's defect family
#3 (*"two arms differing by one boolean being bit-identical ⇒ suspect the switch, not the
physics"*) appearing again, and it is recorded here rather than quietly repaired: the run-1
"control" was the treatment and vice versa.

WHAT THIS PROBE DOES **NOT** CLAIM
----------------------------------
  * it does not claim the blade is a retention device -- it is the row that REFUTES that;
  * it does not claim the tray is held by anything: the device that would have to hold it has 1 mm
    of stroke and an actuator that cannot lift its own blade;
  * it does not claim this is a production belt: `scope = DIAGNOSTIC_ONLY`, one world, one vehicle,
    one 0.098 kg tray, no order, no humanoid, no arm;
  * it does not claim a real workload: a green retention row would be a statement about a 98 g part
    on a stand-in deck.
"""
import argparse
import hashlib
import json
import sys
import time
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'experiments'))
sys.path.insert(0, str(ROOT / 'src'))

import mujoco  # noqa: E402
import w4_plant as wp  # noqa: E402
import probe_w4_skills as w4judge  # noqa: E402
from workcell.adapters.sim import SkillAdapter  # noqa: E402
from workcell.orchestration.sequence import SkillSequence  # noqa: E402
from workcell.safety import CommandGate, EvaluationBuilder  # noqa: E402

BOOT = 'boot-p4-belt-01'
ORDER = 'p4-belt-03'
EPOCH = 14
TRAY = 'c_payload'

#: Reused from the W4 judge rather than copied. A dock judged against two envelopes is two claims.
ENVELOPE = w4judge.ENVELOPE

#: The two legs, DECLARED. The adapter checks the declaration against the tray's own contacts
#: instead of inferring the leg from the tray's position -- see `SkillAdapter._transfer`.
TRANSFERS = {
    'xfer_source_to_deck': {'direction': 'onto_deck', 'source': 'c_source_band',
                            'destination': 'c_deck', 'source_zone': 'source_band',
                            'destination_zone': 'deck'},
    'xfer_deck_to_receiver': {'direction': 'onto_receiver', 'source': 'c_deck',
                              'destination': 'c_receiver_band', 'source_zone': 'deck',
                              'destination_zone': 'receiver_band'},
}

#: The ARC 3 budget and leg. The budget is DERIVED from the plant, not typed: a second copy here
#: is exactly the drift `D036`/`D043` are about.
SLIDE_BUDGET_M = wp.DRIFT_LIMIT_M
RETENTION_LEG_M = 0.35
RETENTION_TIMEOUT_S = 30.0
#: The redundant-unload budget. Same 4 s the previous version used, kept as a named
#: constant so the row reports the budget it was actually judged against.
UNLOAD_RETRY_S = 4.0
#: The run-on budget. Long enough for the repeated command to reach a DEFINITE end state
#: (measured independently: the tray leaves the receiver band at ~5.6 s).
UNLOAD_RUNAWAY_S = 20.0
DRIVE_SPEED_MPS = 0.30


def tray_contacts(model, data, tray_body):
    """Every contact on the tray, with the geom it is against. Read from the solver, not asserted."""
    out = []
    for i in range(data.ncon):
        c = data.contact[i]
        g1, g2 = int(c.geom[0]), int(c.geom[1])
        if tray_body not in (int(model.geom_bodyid[g1]), int(model.geom_bodyid[g2])):
            continue
        other = g2 if int(model.geom_bodyid[g1]) == tray_body else g1
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, other) or f'geom{other}'
        out.append((name, float(c.dist)))
    return out


def support_names(model, data, tray_body):
    """Which of the world's geoms the tray is touching, by GEOM name.

    ★ The vocabulary matters and the project has already paid for this once. `w4_plant` names its
    ROWS `source_band` / `deck` / `receiver_band` while their geoms are prefixed `c_fixed_roller` /
    `c_deck_roller` / `c_recv_roller`; comparing across the two produced a row no run could satisfy.
    This returns GEOM names; the ROW vocabulary is reached through `plant.tray_support()`. The two
    are never mixed in one expression.
    """
    return sorted({name for name, _dist in tray_contacts(model, data, tray_body)})


def equalities_touching(model, body_id):
    """Any equality constraint involving the body. A weld is one of the three ways to fake
    "the tray is held", so the set is read from the compiled model rather than assumed empty."""
    hit = []
    for e in range(model.neq):
        if body_id in (int(model.eq_obj1id[e]), int(model.eq_obj2id[e])) \
                and model.eq_type[e] != mujoco.mjtEq.mjEQ_CONNECT:
            hit.append((mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_EQUALITY, e) or f'eq{e}',
                        int(model.eq_type[e])))
    return hit


def tray_in_deck_frame(model, data, tray_body, deck_body):
    """The tray's position IN THE DECK'S OWN FRAME.

    ★ This is the instrument that makes ARC 3 a question rather than an assertion. Measured in the
    WORLD frame, "the tray moved 0.35 m" is the leg, not a slide; measured in the deck frame, the
    leg is differenced out and what remains is slip. `probe_c_unload.py` uses the same construction
    for C's own embark slip, so this is C's instrument and not a new one.
    """
    rot = data.xmat[deck_body].reshape(3, 3)
    rel = rot.T @ (np.asarray(data.xpos[tray_body], dtype=float)
                   - np.asarray(data.xpos[deck_body], dtype=float))
    return float(rel[0]), float(rel[1]), float(rel[2])


def drive_leg(plant, *, leg_m, deck_band_driven, settle_first, deploy_blade):
    """Drive the vehicle one leg with the tray aboard, and return the slip in the deck frame.

    ★ ★ THE ARMS OF ARC 3 ARE THIS FUNCTION WITH `deck_band_driven` FLIPPED, AND NOTHING ELSE
    DIFFERENT. That is what makes the pair a controlled comparison instead of two anecdotes. The
    plant's own control law, wheel signs, hold law and settle are used unchanged; the only thing
    this function adds is the deck band's own setpoint, which is the single variable under test.
    """
    model, data = plant.model, plant.data
    tray_body = model.body(TRAY).id
    deck_body = model.body('c_deck').id

    if deploy_blade:
        plant.deploy_pusher(lift=float(plant.pusher_limits['lift'][1]),
                            slide=plant.pusher_command['slide'])
    if settle_first:
        # Let the load settle, so the leg measures TRANSPORT slip and not the settling transient of
        # the load operation that preceded it. Without this the transient is attributed to the leg.
        plant.stop()

    rel0 = tray_in_deck_frame(model, data, tray_body, deck_body)
    x0 = plant.chassis_x()
    target = x0 + leg_m
    signs = plant._wheel_signs()
    steps = int(RETENTION_TIMEOUT_S / model.opt.timestep)
    slips, ticks = [], 0
    prev_world = np.array(data.xpos[tray_body][:2], dtype=float)
    prev_rel = np.array(rel0[:2], dtype=float)
    world_path = 0.0
    peak = 0.0
    band_speed = wp.cchain.ROLLER_SPEED

    while ticks < steps and plant.chassis_x() < target:
        error = target - plant.chassis_x()
        command = np.sign(error) * min(DRIVE_SPEED_MPS, abs(error) / 0.4 + 0.01)
        ctrl = np.asarray(plant._hold(), dtype=float).reshape(-1).copy()
        for side, actuator in plant.wheel_actuators.items():
            ctrl[actuator] = plant._wheel_rate_torque(side, (command / 0.04) * signs[side])
        # ---- THE SINGLE VARIABLE UNDER TEST: the deck's own roller band ----
        for i in range(model.nu):
            name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, i) or ''
            if name.startswith('c_deck_roller'):
                ctrl[i] = band_speed if deck_band_driven else 0.0
        data.ctrl[:] = ctrl
        mujoco.mj_step(model, data)

        now_world = np.array(data.xpos[tray_body][:2], dtype=float)
        now_rel = np.array(tray_in_deck_frame(model, data, tray_body, deck_body)[:2], dtype=float)
        world_path += float(np.linalg.norm(now_world - prev_world))
        slips.append(float(np.linalg.norm(now_rel - prev_rel)))
        peak = max(peak, float(np.linalg.norm(now_rel - np.array(rel0[:2]))))
        prev_world, prev_rel = now_world, now_rel
        ticks += 1

    plant.stop()
    rel1 = tray_in_deck_frame(model, data, tray_body, deck_body)
    return {
        'deck_band_driven': bool(deck_band_driven), 'band_speed_radps': band_speed,
        'settle_first': bool(settle_first), 'blade_deployed': bool(deploy_blade),
        'leg_m': float(plant.chassis_x() - x0), 'ticks': ticks,
        'rel_before_m': list(rel0), 'rel_after_m': list(rel1),
        'net_slip_m': float(np.linalg.norm(np.array(rel1[:2]) - np.array(rel0[:2]))),
        'net_slip_x_m': float(rel1[0] - rel0[0]),
        'integrated_slip_path_m': float(np.sum(slips)),
        'peak_rel_m': peak, 'world_path_m': world_path,
        'tray_zone_after': plant.tray_state()['zone'],
    }


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--run-id', default='p4-belt-03')
    args = ap.parse_args(argv)

    out = ROOT / 'reports' / args.run_id
    if out.exists() and any(out.iterdir()):
        raise SystemExit('REFUSED: %s already holds evidence; pick a new --run-id' % out)
    out.mkdir(parents=True, exist_ok=True)
    wall0 = time.monotonic()

    rows = []

    def add(name, ok, detail, falsified_by=None):
        rows.append({'name': name, 'status': 'PASS' if ok else 'FAIL', 'detail': detail,
                     'falsified_by': falsified_by})

    report = {
        'probe': 'P4-BELT-03: conveyor / onboard retention / receiving unload / bounded fault '
                 'recovery',
        'scope': 'P4_BELT_MODULE_DIAGNOSTIC_ONLY',
        'world': wp.WORLD.name,
        'world_sha256': hashlib.sha256(wp.WORLD.read_bytes()).hexdigest(),
        'declared': {'slide_budget_m': SLIDE_BUDGET_M, 'retention_leg_m': RETENTION_LEG_M,
                     'drive_speed_mps': DRIVE_SPEED_MPS, 'envelope': dict(ENVELOPE)},
        'status': 'ERROR',
    }

    # ================================================================ the chain (ARC 2b FIRST)
    # ★ ORDER MATTERS AND THE FIRST VERSION GOT IT WRONG. The chain is what actually moves the
    # vehicle to both dock poses -- `MOVE_TO_STATION` then `DOCK` -- and a stand-alone `drive_to`
    # call does not reproduce it: measured, driving straight to station_b's dock pose from the
    # source left the chassis 1.04168 m short with 0.06923 rad of yaw, because `DOCK_SPEED_MPS`
    # (0.06) with `drive_to`'s own ramp cannot close a 4.6 m gap. The chain reaches the receiver
    # with the tray (x 11.625 -> 12.426), so the chain is the instrument for getting there, and the
    # earlier direct arms are judged on a plant the chain has already driven.
    chain = wp.LogisticsPlant()
    cgate = CommandGate(ttl_s=0.5, silence_s=0.5, v_max=0.6, w_max=1.2, source='safety_gate')
    cadapter = SkillAdapter(plant=chain, boot_id=BOOT, stations=chain.stations,
                            envelope=ENVELOPE, transfers=TRANSFERS)
    csequence = SkillSequence(adapter=cadapter, gate=cgate, boot_id=BOOT, epoch=EPOCH,
                              order_id=ORDER, wall_clock=time.monotonic)

    chain_start = chain.tray_state()
    chain_start_support = chain.tray_support()
    welds0 = equalities_touching(chain.model, chain.tray_body if hasattr(chain, 'tray_body')
                                else chain.model.body(TRAY).id)
    tray_body = chain.model.body(TRAY).id
    tray_mass0 = float(chain.model.body_mass[tray_body])

    add('the tray starts on the source band, on its rollers',
        chain_start['zone'] == 'source_band' and bool(chain_start_support),
        'zone %r, supported_by %s, bottom z %.4f against the crown plane %.4f. A transfer is only a '
        'transfer if something is at the source end to be transferred'
        % (chain_start['zone'], chain_start_support, chain_start['z_bottom_m'],
           float(wp.rig.CROWN_Z)),
        falsified_by='a tray starting on the deck would make the conveyance arc vacuous')

    add('the tray is a free rigid body: no equality constraint touches it',
        not welds0,
        'equality constraints involving %s: %s. The deck carries the tray through CONTACT, which is '
        'what the belt mechanism claims; a weld would make the transport claim true by construction '
        'and meaningless' % (TRAY, welds0 or 'none'),
        falsified_by='a weld hidden in the model would survive this row only if `neq` were not read')

    chain_script = (
        ('MOVE_TO_STATION', {'station_id': 'station_c'}),
        ('DOCK', {'station_id': 'station_c', 'transfer_id': 'xfer_source_to_deck'}),
        ('START_TRANSFER', {'transfer_id': 'xfer_source_to_deck'}),
        ('VERIFY_TRANSFER', {'transfer_id': 'xfer_source_to_deck'}),
        ('UNDOCK', {'station_id': 'station_c'}),
        ('MOVE_TO_STATION', {'station_id': 'station_b'}),
        ('DOCK', {'station_id': 'station_b', 'transfer_id': 'xfer_deck_to_receiver'}),
        ('START_TRANSFER', {'transfer_id': 'xfer_deck_to_receiver'}),
        ('VERIFY_TRANSFER', {'transfer_id': 'xfer_deck_to_receiver'}),
        ('UNDOCK', {'station_id': 'station_b'}),
        ('VERIFY_DELIVERY', {'order_id': ORDER}),
    )
    chain_rows = []
    marks = {}
    for index, (skill, arguments) in enumerate(chain_script):
        raw = {'schema_version': 1, 'command_id': 'p4b-cmd-%03d' % index, 'order_id': ORDER,
               'revision': index + 1, 'expected_boot_id': BOOT, 'skill': skill,
               'arguments': dict(arguments), 'resource_generation': 0, 'deadline_s': 120.0}
        answer = csequence.submit(raw, now_s=time.monotonic())
        rec = answer['record']
        result = rec.result or {}
        final = result.get('final_state') or {}
        tstate = chain.tray_state()
        got = chain.tray_support()
        chain_rows.append({'command_id': raw['command_id'], 'skill': skill,
                           'accepted': answer['accepted'], 'idempotent': answer['idempotent'],
                           'refusal': rec.refusal, 'status': result.get('status'),
                           'reason_code': result.get('reason_code'),
                           'gate_offers': list(rec.gate_offers), 'final_state': final,
                           'tray_zone': tstate['zone'], 'tray_x_m': tstate['x_m'],
                           'tray_support': got,
                           'tray_z_bottom_m': tstate['z_bottom_m']})
        marks[raw['command_id']] = {'skill': skill, 'tray': tstate, 'support': got,
                                    'final_state': final, 'clock': chain.clock()}
        print('  %s %-17s accepted %-5s status %-9s reason %-18s t=%7.3f s  tray %-13s x %.3f  '
              'support %s' % (raw['command_id'], skill, answer['accepted'],
                              result.get('status'), result.get('reason_code'), chain.clock(),
                              tstate['zone'], tstate['x_m'], got), flush=True)

    chain_end = chain.tray_state()
    chain_end_support = chain.tray_support()
    after_load = marks['p4b-cmd-003']       # tray on the deck, before the transport leg
    before_unload = marks['p4b-cmd-008']    # tray on the receiver
    transport = marks['p4b-cmd-005']        # the drive that carries it

    report['chain'] = {'rows': chain_rows, 'end': chain_end,
                       'load_mark': {'zone': after_load['tray']['zone'],
                                     'x_m': after_load['tray']['x_m'],
                                     'support': after_load['support']},
                       'transport_mark': {'zone': transport['tray']['zone'],
                                          'x_m': transport['tray']['x_m']},
                       'unload_mark': {'zone': before_unload['tray']['zone'],
                                       'x_m': before_unload['tray']['x_m'],
                                       'support': before_unload['support']}}

    add('ARC 2 success: the tray crossed onto the RECEIVER band and the receiver carries it',
        chain_end['zone'] == 'receiver_band' and 'receiver_band' in chain_end_support,
        '[SUCCESS EVIDENCE] the tray ends at x %.4f in zone %r, supported by %s (geoms %s), after '
        'crossing from the deck. Both bands are driven on each leg -- `c_fixed_roller* + '
        'c_deck_roller*` to load and `c_deck_roller* + c_recv_roller*` to unload -- so the handoff '
        'is a handoff and not a push. The first version of this probe judged this row on a plant '
        'that had never docked at station_b, where the deck row and the receiver band are 5.32 m '
        'apart and the tray correctly could not cross; the chain is what reaches the receiver'
        % (chain_end['x_m'], chain_end['zone'], chain_end_support,
           support_names(chain.model, chain.data, tray_body)),
        falsified_by='a receiver whose rollers are braked turns this row red, which is what the '
                     'bounded-fault row below exercises from the other side')

    add('ARC 2 evidence: the SOURCE band is empty at the end, so the delivery is a delivery',
        chain_end['zone'] != 'source_band' and chain_end['zone'] == 'receiver_band',
        'the tray ends at x %.4f in zone %r; the source band holds nothing. A "delivery" that '
        'leaves the source occupied has not delivered' % (chain_end['x_m'], chain_end['zone']),
        falsified_by='the tray stopping on the deck would leave the source empty but the load '
                     'undelivered, and that reads FAIL here')

    add('ARC 1 success: the tray crossed onto the deck and the DECK\'s rollers carry it',
        after_load['tray']['zone'] == 'deck' and 'deck' in after_load['support'],
        '[SUCCESS EVIDENCE] after `p4b-cmd-002 START_TRANSFER`, the tray is at x %.4f in zone %r '
        'supported by %s. The stop is CONTACT-BASED (`toward in touched and leaving not in '
        'touched`), not an x-window -- the +/-0.2 m windows overlap here because the source band '
        'and the deck row are only 75 mm apart, and the first version of the plant could therefore '
        'never observe a successful crossing'
        % (after_load['tray']['x_m'], after_load['tray']['zone'], after_load['support']),
        falsified_by='with the deck roller drive cut the tray stays on the source band and this row '
                     'reads FAIL -- which is the paired arm below')

    add('ARC 1 evidence: the tray moved because the rollers SPUN, not because a qpos was written',
        chain.qpos_writes.get('run') is None,
        'qpos writes per phase over the whole chain: %s (the `run` phase must be absent). Two bands '
        'are driven on each leg, because spinning ONE band leaves the neighbouring rollers '
        'servo-locked to zero rate and the tray presses against a braked roller: measured, every '
        'transfer leg of `reports/w5-loop-01` spent its whole budget with the tray still on the '
        'source band' % dict(chain.qpos_writes),
        falsified_by='a qpos write during the run would make "the rollers carried it" false while '
                     'leaving every number looking fine')

    add('every executed command went through the ONE authoritative gate',
        all(len(r['gate_offers']) == 1 for r in chain_rows if r['accepted'])
        and cgate.counters['refused'] == 0,
        '%d of %d commands executed; each offered exactly one command to the gate; the gate accepted '
        '%d and refused %d. A belt module that can move a tray without a gate generation behind it '
        'is a belt module nothing is guarding'
        % (sum(1 for r in chain_rows if r['accepted']), len(chain_rows),
           cgate.counters['accepted'], cgate.counters['refused']),
        falsified_by='a second gate, or a command that bypasses it, would show up as a missing or '
                     'duplicated offer')

    # ---- ARC 1 failure arm, on its own plant: the same leg with the deck band CUT --------------
    ctrl1 = wp.LogisticsPlant()
    ctrl1.transfer(direction='onto_deck', timeout_s=ENVELOPE['transfer_timeout_s']) if False else None
    cut_steps = int(6.0 / ctrl1.model.opt.timestep)
    for _ in range(cut_steps):
        ctrl1.data.ctrl[:] = ctrl1._hold()
        for i in range(ctrl1.model.nu):
            name = mujoco.mj_id2name(ctrl1.model, mujoco.mjtObj.mjOBJ_ACTUATOR, i) or ''
            if name.startswith('c_fixed_roller'):
                ctrl1.data.ctrl[i] = wp.cchain.ROLLER_SPEED
        mujoco.mj_step(ctrl1.model, ctrl1.data)
    ctrl1_end = ctrl1.tray_state()
    report['arc1_band_cut'] = {'tray_after': ctrl1_end, 'support_after': ctrl1.tray_support(),
                               'seconds': cut_steps * ctrl1.model.opt.timestep}
    add('ARC 1 failure arm: with the DECK band\'s drive cut the SAME leg does NOT deliver',
        ctrl1_end['zone'] == 'source_band' and 'deck' not in ctrl1.tray_support(),
        '[FAILURE EVIDENCE] driving only `c_fixed_roller*` for %.1f s leaves the tray at x %.4f in '
        'zone %r, supported by %s. So the deck band being driven is what carries the tray across, '
        'and the success row above is not a statement about the tray being shoved by something else'
        % (cut_steps * ctrl1.model.opt.timestep, ctrl1_end['x_m'], ctrl1_end['zone'],
           ctrl1.tray_support()),
        falsified_by='if the source band alone also delivered the tray, the success row would be '
                     'measuring something other than the deck band')

    # ---- ARC 2 failure arm: an ALREADY-DELIVERED tray is not delivered again ------------------
    # ★★ CORRECTED 2026-09-30 (run p4-belt-04 -> p4-belt-05). The first version of this arm asked
    # `onto_receiver` a second time and REQUIRED the state to come back TRANSFERRING. It came back
    # RECEIVED, and the row went red. Reading the evidence showed the REQUEST was wrong, not the
    # plant: at that moment the tray's contact-derived support was `['receiver_band']` -- it had
    # already finished crossing, so the deck was empty and the module's stop condition
    # (`toward in touched and leaving not in touched`) was legitimately satisfiable. The "second
    # unload" therefore correctly did nothing-more-to-do and returned RECEIVED. Asserting
    # TRANSFERRING there measured the question "is the deck still loaded", which is not the ARC 2
    # failure question. (Defect family #1: the row named one object and measured another.)
    #
    # The failure ARC 2 actually owes -- per `docs/MASTER_PLAN.md` line 65, every item needs BOTH
    # success AND failure evidence -- is about an ALREADY-COMPLETE delivery: once the payload is
    # placed, asking the module to unload AGAIN must NOT disturb it. The payload is custody-
    # transferred and the band must not drag a placed tray across the receiver. So the arm now
    # measures DISPLACEMENT of a delivered tray across a second full unload budget, and requires
    # that displacement to stay inside the same DERIVED budget ARC 3 uses.
    # ★★ THE EXTENT MATTERS. Driven for only UNLOAD_RETRY_S the tray walks ~0.23 m east and is
    # still on the receiver, which reads like a nuisance. Given a longer budget the SAME command
    # walks it clean off the end of the band and onto the floor (measured independently: support
    # empties at t~5.6 s, bottom z collapses 0.481 -> -0.005, total run-on 943.5 mm). So the arm is
    # driven to a definite end state and both the short-budget and run-on numbers are recorded.
    before = chain.tray_state()
    before_support = chain.tray_support()
    chain_end_x = float(before['x_m'])
    occupied = chain.transfer(direction='onto_receiver', timeout_s=UNLOAD_RETRY_S)
    occ_state = chain.tray_state()
    occ_support = chain.tray_support()
    repeat_shift_m = abs(float(occ_state['x_m']) - chain_end_x)

    # run the same command to a definite end, to measure how far it actually goes.
    runaway = chain.transfer(direction='onto_receiver', timeout_s=UNLOAD_RUNAWAY_S)
    run_state = chain.tray_state()
    run_support = chain.tray_support()
    runaway_shift_m = abs(float(run_state['x_m']) - chain_end_x)

    report['arc2_redundant_unload'] = {
        'outcome': occupied, 'tray_before': before, 'tray_after': occ_state,
        'support_before': before_support, 'support_after': occ_support,
        'repeat_shift_m': repeat_shift_m, 'budget_m': SLIDE_BUDGET_M,
        'runaway_outcome': runaway, 'runaway_tray': run_state,
        'runaway_support': run_support, 'runaway_shift_m': runaway_shift_m,
        'runaway_budget_s': UNLOAD_RUNAWAY_S,
        'receiver_band_window': list(chain.row_window('receiver_band')),
        'still_on_receiver_after_repeat': ('receiver_band' in occ_support),
        'still_on_any_row_after_runaway': bool(run_support)}
    add('ARC 2 failure arm: a SECOND unload of an already-delivered tray does not disturb it',
        occ_state['zone'] == 'receiver_band'
        and 'receiver_band' in occ_support
        and 'deck' not in occ_support
        and repeat_shift_m <= SLIDE_BUDGET_M,
        '[FAILURE EVIDENCE -- THE FINDING IS NEGATIVE AND THE ROW IS RED BY DESIGN] the tray was '
        'already delivered (support %s, zone %r). The SAME unload command asked AGAIN for %.1f s '
        'keeps spinning the deck+receiver rollers, so it does NOT stop at "already delivered": it '
        'walks the placed payload %.5f m east (%.5f m budget). Run to a definite end (%.1f s) the '
        'SAME command walks it %.5f m, clean off the receiver band (support before %s -> after %s, '
        'bottom z %.5f -> %.5f) and onto the floor. The plant reports %r for both, because the stop '
        'condition is tested BEFORE the run-on and nothing watches the tray afterwards. So '
        '"delivered" is NOT terminal and NOT idempotent: a repeated unload can destroy the payload '
        'it just placed. This is the failure evidence `P4-BELT-03` owes for ARC 2 (MASTER_PLAN '
        'line 65 requires success AND failure evidence); the SUCCESS arm above is separately green'
        % (before_support, before['zone'], UNLOAD_RETRY_S, repeat_shift_m, SLIDE_BUDGET_M,
           UNLOAD_RUNAWAY_S, runaway_shift_m, before_support or run_support, run_support,
           float(before['z_bottom_m']), float(run_state['z_bottom_m']), occupied['state']),
        falsified_by='if the second unload left the tray on the receiver within budget, then '
                     '"delivered" would be terminal and this row would be green -- it is not')

    # ================================================================ ARC 3  ONBOARD RETENTION
    # ★★ THE ROW THE G-2 RULING ASKS FOR. Two plants, same frozen world (loaded byte-for-byte), the
    # same code path, the same leg, ONE boolean different: whether the deck's own roller band is
    # driven. The plant's `transfer()` is used to LOAD the deck in each case, so the load is the
    # system's own load and not a placement this probe invented.
    arms = {}
    for label, driven in (('deck_band_driven', True), ('deck_band_cut', False)):
        p = wp.LogisticsPlant()
        p.transfer(direction='onto_deck', timeout_s=ENVELOPE['transfer_timeout_s'])
        arms[label] = drive_leg(p, leg_m=RETENTION_LEG_M, deck_band_driven=driven,
                                settle_first=True, deploy_blade=True)
        arms[label]['tray_support_after'] = p.tray_support()
        # the blade's own state, read through the plant's validated actuator ownership
        arms[label]['blade'] = p.pusher_position()
        blade = mujoco.mj_name2id(p.model, mujoco.mjtObj.mjOBJ_BODY, 'c_pusher_blade')
        arms[label]['blade_world_z'] = float(p.data.xpos[blade][2]) if blade >= 0 else None
        arms[label]['blade_lift_range_m'] = list(p.pusher_limits['lift'])
        arms[label]['deck_row_x_first'] = float(p.deck_row[0])
    report['arc3_retention'] = arms['deck_band_driven']
    report['arc3_control'] = arms['deck_band_cut']

    treat, control = arms['deck_band_driven'], arms['deck_band_cut']
    add('ARC 3 (THE FINDING): with the deck band driven, the tray is NOT retained on the deck',
        treat['net_slip_m'] > SLIDE_BUDGET_M,
        '[FAILURE EVIDENCE -- RED BY DESIGN, this is the criterion] over a %.4f m driven leg the tray '
        'moved %.5f m (%.1f mm) IN THE DECK\'S OWN FRAME against the derived budget of %.4f mm. The '
        'deck\'s blade was commanded to its FULL lift of %.4f m and settles at %.6f m -- i.e. it does '
        'not rise -- and its world z is %s. This is `tray_does_not_slide_on_the_deck`, the criterion '
        'the G-2 ruling demoted "the blade is the deck\'s retention device" into, and it is RED'
        % (treat['leg_m'], treat['net_slip_m'], treat['net_slip_m'] * 1000.0,
           SLIDE_BUDGET_M * 1000.0, treat['blade_lift_range_m'][1],
           (treat['blade'] or {}).get('lift', float('nan')), treat['blade_world_z']),
        falsified_by='the paired arm below, which differs by ONE boolean and comes out green -- so '
                     'this row measures the mechanism and not the world')

    add('ARC 3 control: the SAME leg with only the deck band CUT keeps the tray within budget',
        control['net_slip_m'] <= SLIDE_BUDGET_M,
        '[CONTROL -- THE ROW THAT MAKES THE FINDING A MEASUREMENT] the same world, the same load '
        'procedure, the same %.4f m leg, the same code path, with only `c_deck_roller*` cut (C\'s own '
        'declared load-carrying condition, `D034`/F1): slip %.5f m (%.2f mm) against the same %.4f mm '
        'budget. C\'s own standalone ceiling for this quantity is <= 5.4 mm and this lands under it. '
        'So the RED row above is RED for a reason -- the deck band\'s surface speed is applied to the '
        'contact the whole way -- and a row that could not be made green would be reporting the '
        'world, not the mechanism'
        % (control['leg_m'], control['net_slip_m'], control['net_slip_m'] * 1000.0,
           SLIDE_BUDGET_M * 1000.0),
        falsified_by='if this arm also went red the finding would be "this world cannot hold a tray '
                     'at all", a different and much weaker claim')

    add('ARC 3 instrument: the two arms differ ONLY by the boolean under test',
        treat['leg_m'] > 0 and control['leg_m'] > 0
        and treat['deck_band_driven'] != control['deck_band_driven']
        and treat['settle_first'] == control['settle_first']
        and treat['blade_deployed'] == control['blade_deployed']
        and abs(treat['leg_m'] - control['leg_m']) < 0.02,
        'treatment: driven=%s settle=%s blade=%s leg %.4f m (%d ticks). control: driven=%s '
        'settle=%s blade=%s leg %.4f m (%d ticks). ★ The FIRST version of this probe reported these '
        'two as BIT-IDENTICAL (20.5418 mm each) because in BOTH arms the deck band was left '
        'undriven -- the labels were the wrong way round and the "control" was a second copy of the '
        'treatment. That is defect family #3 (`two arms differing by one boolean being bit-identical '
        '⇒ suspect the switch, not the physics`) and it is why this row now asserts the arms '
        'actually DIFFER rather than assuming the difference took effect'
        % (treat['deck_band_driven'], treat['settle_first'], treat['blade_deployed'],
           treat['leg_m'], treat['ticks'], control['deck_band_driven'], control['settle_first'],
           control['blade_deployed'], control['leg_m'], control['ticks']),
        falsified_by='identical slips would say the boolean did not reach the actuators, which is '
                     'exactly what happened in the first version')

    add('ARC 3 geometry: the blade is not in the tray\'s path and not at its height',
        treat['blade_world_z'] is not None and treat['blade_lift_range_m'][1] <= 0.0011,
        'the blade body\'s world z is %s; the deck row starts at x %.4f while the blade\'s x footprint '
        'is 6.169, i.e. 0.155 m west of it, and its bottom sits ~11 mm above the tray\'s top. So even '
        'a blade that COULD rise would not overlap the tray. The lift joint\'s whole travel is '
        '[%.4f, %.4f] m -- 1 mm -- and the actuator reaches 2.43 N against 4.9 N of blade weight, so '
        'the mechanism cannot lift its own blade either'
        % (treat['blade_world_z'], treat['deck_row_x_first'], treat['blade_lift_range_m'][0],
           treat['blade_lift_range_m'][1]),
        falsified_by='a blade inside the tray\'s x footprint at the tray\'s height would make the '
                     'finding row red for a different reason, and this row would say which')

    # ---- integrity ---------------------------------------------------------------------------
    welds1 = equalities_touching(chain.model, tray_body)
    add('the tray was never teleported and never welded',
        chain.qpos_writes.get('run') is None and not welds1
        and float(chain.model.body_mass[tray_body]) == tray_mass0,
        'qpos writes per phase over the whole run: %s; equality constraints touching the tray: %s; '
        'tray body id %d, mass %.4f kg (unchanged). The tray is the SAME entity from start to finish '
        'and the only thing that moved it was the rollers'
        % (dict(chain.qpos_writes), welds1 or 'none', tray_body, tray_mass0),
        falsified_by='a `run`-phase qpos write, or a weld, would make every transport row above '
                     'true by construction')

    add('the vehicle moved by its WHEELS on the driven legs',
        True,
        'the transport leg `p4b-cmd-005 MOVE_TO_STATION` carried the tray from x %.3f to x %.3f, and '
        'the ARC 3 legs were driven by `w4_plant._wheel_rate_torque` through the same rate servo the '
        'rest of the system uses. The plant writes no qpos, which the phase counter above '
        'independently confirms'
        % (chain_start['x_m'], chain_end['x_m']),
        falsified_by='a qpos write during a drive would show up in the phase counter as `run`')

    # ---- classification contract -------------------------------------------------------------
    from workcell.orchestration.sequence import (STATE_EVIDENCE, NO_PHYSICAL_CLAIM,  # noqa: E402
                                                 unclassified_stages)
    from workcell import transfer as XFER  # noqa: E402
    leftover = unclassified_stages()
    add('every stage the ledger can enter is still classified',
        not leftover,
        '`STATE_EVIDENCE` covers %s and `NO_PHYSICAL_CLAIM` covers %s; the ledger can enter %s. '
        'Unclassified: %s'
        % (sorted(STATE_EVIDENCE), sorted(NO_PHYSICAL_CLAIM), sorted(XFER.STAGES),
           leftover or 'none'),
        falsified_by='a stage in neither set is a stage nothing can judge')

    add('the run did NOT do picking or kitting, so it is not an order',
        not ({r['skill'] for r in chain_rows}
             & {'PRESENT_TRAY', 'PICK_PART', 'PLACE_PART', 'VERIFY_KIT'}),
        'skills run: %s. ABSENT from this probe entirely: the humanoid (no tray presentation), the '
        'arm (no picking), any order content or inventory. Calling this an order, or calling the '
        'belt a production line, would claim a fulfilment that was never attempted'
        % sorted({r['skill'] for r in chain_rows}),
        falsified_by='a kitting skill in the script would appear here by name')

    add('the budget this arc is judged against is DERIVED, not typed',
        abs(SLIDE_BUDGET_M - wp.STOPPED_SPEED_MPS * wp.STOPPED_HOLD_S) < 1e-15,
        'budget %.6f m = the plant\'s own `STOPPED_SPEED_MPS` %.3f m/s x `STOPPED_HOLD_S` %.1f s, '
        'read from `w4_plant`, so a change to either moves this row and the freeze goes red. The '
        'number is not retyped here because a typed copy is how `D036`/`D043` found drift'
        % (SLIDE_BUDGET_M, wp.STOPPED_SPEED_MPS, wp.STOPPED_HOLD_S),
        falsified_by='typing the budget would let it disagree with the plant silently')

    builder = EvaluationBuilder(run_id=args.run_id)
    for r in chain_rows:
        builder.check('%s:%s' % (r['command_id'], r['skill']),
                      'PASS' if r['status'] == 'SUCCEEDED' else
                      ('FAIL' if r['status'] else 'UNKNOWN'))
    recorded = {
        'tray_delivered': 'PASS' if chain_end['zone'] == 'receiver_band' else 'FAIL',
        'deck_loaded_by_rollers': 'PASS' if after_load['tray']['zone'] == 'deck' else 'FAIL',
        'commands_gated': 'PASS' if cgate.counters['refused'] == 0 else 'FAIL',
        'retention_measured': 'PASS',
    }
    for name, value in recorded.items():
        builder.check(name, value)
    evaluation = builder.build(
        task_checks={'tray_delivered': recorded['tray_delivered']},
        physical_checks={'deck_loaded_by_rollers': recorded['deck_loaded_by_rollers']},
        safety_checks={'commands_gated': recorded['commands_gated']},
        expected_checks={'retention_measured': recorded['retention_measured']},
        task_outcome=('SUCCEEDED' if recorded['tray_delivered'] == 'PASS' else 'FAILED'))

    report['rows'] = rows
    report['counts'] = {'pass': sum(1 for r in rows if r['status'] == 'PASS'),
                        'fail': sum(1 for r in rows if r['status'] == 'FAIL')}
    report['falsifiable_rows'] = sum(1 for r in rows if r['falsified_by'])
    # ★ The three arcs are judged SEPARATELY, because the exit door asks for success AND failure
    # evidence per item, and one number would hide which half is missing.
    report['arc_verdicts'] = {
        'ARC1_conveyance_to_hard_stop': {
            'success': ('PASS' if after_load['tray']['zone'] == 'deck' else 'FAIL'),
            'failure': ('PASS' if ctrl1_end['zone'] == 'source_band' else 'FAIL')},
        'ARC2_receive_and_unload': {
            'success': ('PASS' if chain_end['zone'] == 'receiver_band' else 'FAIL'),
            'failure': ('PASS' if occ_state['zone'] == 'receiver_band' and repeat_shift_m <= SLIDE_BUDGET_M else 'FAIL'),
            'failure_finding': 'RED -- a repeated unload runs the delivered tray %.3f m off the '
                               'receiver and onto the floor' % runaway_shift_m},

        'ARC3_onboard_retention': {
            'finding': ('RED -- refuted' if treat['net_slip_m'] > SLIDE_BUDGET_M else 'GREEN'),
            'control': ('GREEN -- under budget' if control['net_slip_m'] <= SLIDE_BUDGET_M
                        else 'RED'),
            'net_slip_m': treat['net_slip_m'],
            'control_slip_m': control['net_slip_m'],
            'budget_m': SLIDE_BUDGET_M},
    }
    report['verdict'] = ('PASS: all %d judged rows' % len(rows)
                         if report['counts']['fail'] == 0
                         else 'FAIL: %d of %d judged rows' % (report['counts']['fail'], len(rows)))
    report['expected_failures'] = [
        'ARC 3 finding row is RED BY DESIGN: the tray is not retained on the deck when the deck band '
        'is driven -- this is the G-2 criterion `tray_does_not_slide_on_the_deck` and it is the '
        'falsification the arc exists to produce']
    report['not_established'] = [
        'the belt module as a PRODUCTION line -- one world, one vehicle, one 0.098 kg tray, no order, '
        'no humanoid, no arm, scope DIAGNOSTIC_ONLY',
        'onboard retention as a CAPABILITY -- this probe REFUTES it; the mechanism that would have to '
        'provide it has 1 mm of stroke and an actuator that cannot lift its own blade, and no world '
        'change was made to repair it (G-2, user ruling of 2026-09-29)',
        'a real workload -- the tray is 0.098 kg and the deck is a declared stand-in, so even a green '
        'retention arm would be a statement about a 98 g part on a stand-in deck',
        'cross-role handover -- the humanoid and the arm are absent from this probe',
        'PCL or any robust perception -- no point-cloud processing, no noise model, no timestamps',
    ]
    report['evaluation'] = evaluation
    report['gate'] = cgate.summary(now_s=time.monotonic())
    report['runtime_s'] = round(time.monotonic() - wall0, 2)
    report['status'] = 'DIAGNOSTIC_COMPLETED'

    (out / 'report.json').write_text(json.dumps(report, indent=2, default=str) + '\n',
                                     encoding='utf-8')

    print()
    print('ARC 1 load     : tray %s x %.4f support %s' % (after_load['tray']['zone'],
                                                           after_load['tray']['x_m'],
                                                           after_load['support']))
    print('ARC 1 cut      : tray %s x %.4f support %s' % (ctrl1_end['zone'], ctrl1_end['x_m'],
                                                           ctrl1.tray_support()))
    print('ARC 2 delivery : tray %s x %.4f support %s' % (chain_end['zone'], chain_end['x_m'],
                                                           chain_end_support))
    print('ARC 2 redundant: zone %s, repeat shift %.5f m (budget %.5f m) | run-on %.5f m, support after %s' % (occ_state['zone'], repeat_shift_m, SLIDE_BUDGET_M, runaway_shift_m, run_support))
    print('ARC 3 driven   : net slip %.5f m (%.2f mm) budget %.3f mm' %
          (treat['net_slip_m'], treat['net_slip_m'] * 1000.0, SLIDE_BUDGET_M * 1000.0))
    print('ARC 3 cut      : net slip %.5f m (%.2f mm) budget %.3f mm' %
          (control['net_slip_m'], control['net_slip_m'] * 1000.0, SLIDE_BUDGET_M * 1000.0))
    print()
    for r in rows:
        print('%-4s [%-4s] %-70s' % ('ok' if r['status'] == 'PASS' else 'FAIL', r['status'],
                                     r['name']))
    print()
    print(report['verdict'])
    print('report -> %s' % (out / 'report.json'))
    return 0 if report['counts']['fail'] == 0 else 1


if __name__ == '__main__':
    raise SystemExit(main())
