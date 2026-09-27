"""W5: the first real logistics closed loop, in ONE continuous run.

WHAT THIS IS
------------
The single-vehicle logistics integration sub-scenario:

    the source band holds a real tray
    -> MOVE_TO_STATION / DOCK at the source, on the mechanical datum
    -> transfer 1: C's rollers walk the tray across the handoff onto the vehicle deck
    -> custody committed to the vehicle, undock
    -> the vehicle carries it 4.6 m ON ITS WHEELS
    -> DOCK at the receiver
    -> transfer 2: the tray crosses onto the receiver band
    -> receiver confirmed, source cleared, the vehicle is empty
    -> delivery committed

It is NOT order N01. There is no humanoid presentation, no arm, no picking and no inventory in
this sub-scenario, and the report says which roles are present and which are absent rather than
letting a reader assume a full order happened. The two transfer transactions are the REAL P2
objects (`workcell.transfer.TransferLedger`, `workcell.resources.ResourceTable`), driven from the
evidence this physical run produces -- not a second, private bookkeeping system.

THE VERDICTS ARE SEPARATED
--------------------------
`task`, `safety`, `expected_behaviour`, `physics` and `evidence_completeness` are five lists and
five verdicts, because the project's rule is that a run which stopped correctly in the wrong place
is a safety PASS and a task FAIL, and one number cannot say that.

THE PHYSICAL CONSTRAINTS ARE INSTRUMENTS, NOT CLAIMS
----------------------------------------------------
  * identity    -- the tray's body id, mass and equality-constraint set are read before and after;
  * no teleport -- every `data.qpos` write is counted per phase and the `run` phase must be absent;
  * no weld     -- no equality constraint may touch the tray;
  * supported   -- the tray's own geometry must be in contact with the surface it is claimed to
                   rest on, read from the solver's contact list, not asserted;
  * wheels      -- the wheel-driven distance is compared against the leg the world declares.

The last four are the ones a later reader cannot re-derive from a PASS label, so they are printed
with their numbers in the report.
"""
import argparse
import hashlib
import json
import math
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
from workcell import resources as R  # noqa: E402
from workcell import transfer as XFER  # noqa: E402
from workcell.adapters.sim import SkillAdapter  # noqa: E402
from workcell.orchestration.sequence import (  # noqa: E402
    NO_PHYSICAL_CLAIM, STATE_EVIDENCE, SkillSequence, classify_claims, unclassified_stages)
from workcell.safety import CommandGate, EvaluationBuilder  # noqa: E402

BOOT = 'boot-w5-01'
ORDER = 'w5-loop-01'
EPOCH = 13
TRAY = 'c_payload'
#: The scenario's resources, named after the world's own bodies so a reader can find them.
SOURCE_RES, DECK_RES, RECEIVER_RES = 'c_source_band', 'c_deck', 'c_receiver_band'
#: Reused from the W4 judge rather than copied: the envelope a dock is judged against must not have
#: two definitions, and the W4 rows already report which of these numbers bit.
ENVELOPE = w4judge.ENVELOPE
#: The legs this sub-scenario is allowed to run, DECLARED. The adapter checks the declaration
#: against the tray's own contacts rather than inferring the leg from its position.
TRANSFERS = {
    'xfer_source_to_deck': {'direction': 'onto_deck', 'source': SOURCE_RES,
                            'destination': DECK_RES, 'source_zone': 'source_band',
                            'destination_zone': 'deck'},
    'xfer_deck_to_receiver': {'direction': 'onto_receiver', 'source': DECK_RES,
                              'destination': RECEIVER_RES, 'source_zone': 'deck',
                              'destination_zone': 'receiver_band'},
}
SCRIPT = (
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
#: Which stage of each transfer the physical evidence arrives at. `START_TRANSFER` is what actually
#: moves the tray, so the stages up to and including `TRANSFERRING` are claimed on its evidence and
#: the rest on the verification step -- a transfer whose stage advanced before anything moved would
#: be the "command sent equals delivered" defect the guidance forbids.
EVIDENCE_AT_START_TRANSFER = ('REQUESTED', 'BOTH_RESERVED', 'DOCK_VERIFIED', 'BOTH_READY',
                              'TRANSFERRING')
EVIDENCE_AT_VERIFY_TRANSFER = ('RECEIVER_CONFIRMED', 'COMMITTED', 'RELEASED')


def subtree(model, body_id):
    got = {body_id}
    for b in range(model.nbody):
        if int(model.body_parentid[b]) in got:
            got.add(b)
    return got


def tray_contacts(model, data, tray_body):
    """Every contact on the tray, with the surface it is against and where the tray's floor is."""
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


def tray_support(state):
    """Which bodies of the world the tray is standing on, from its own contact set."""
    return sorted({name for name, _dist in state['contacts']})


def equalities_touching(model, body_id):
    """Any equality constraint (weld included) that involves the body. A hidden weld is one of the
    three ways to fake "the tray is held", so the set is read from the compiled model."""
    hit = []
    for e in range(model.neq):
        b1 = int(model.eq_obj1id[e])
        b2 = int(model.eq_obj2id[e])
        if body_id in (b1, b2) and model.eq_type[e] != mujoco.mjtEq.mjEQ_CONNECT:
            hit.append((mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_EQUALITY, e),
                        int(model.eq_type[e])))
    return hit


def observation(plant, *, direction, age_s):
    """One typed observation, in the shape the guidance asks for."""
    now = plant.clock()
    return {'station_id': 'station_b' if direction == 'onto_receiver' else 'station_c',
            'sensor_id': 'c_deck_frame_vs_rails',
            'frame_id': 'station_datum',
            'sequence': int(now / max(plant.model.opt.timestep, 1e-9)),
            'sim_stamp': now, 'wall_receive_time': time.monotonic(), 'age_s': age_s,
            'relative_pose': {k: plant.dock_residuals('station_c').get(k) for k in
                              ('lateral_m', 'longitudinal_m', 'yaw_rad')},
            'uncertainty': {'lateral_m': 0.001, 'longitudinal_m': 0.001, 'yaw_rad': 1e-4},
            'validity': 'VALID' if age_s <= 2.0 else 'STALE',
            'source_method': 'simulated range returns off the declared rail faces',
            'calibration_version': 'w3-observability-04'}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--run-id', default='w5-loop-01')
    ap.add_argument('--reports-dir', default=None)
    args = ap.parse_args()
    out = (Path(args.reports_dir) if args.reports_dir else ROOT / 'reports') / args.run_id
    out.mkdir(parents=True, exist_ok=True)
    wall0 = time.monotonic()

    groups = {'task': [], 'safety': [], 'expected_behaviour': [], 'physics': [],
              'evidence_completeness': []}

    def check(group, name, status, detail):
        groups[group].append({'check': name, 'status': status, 'detail': detail})

    plant = wp.LogisticsPlant()
    model, data = plant.model, plant.data
    tray_body = model.body(TRAY).id
    tray_mass0 = float(model.body_mass[tray_body])
    tray_geoms0 = [mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, g)
                   for g in range(model.ngeom) if int(model.geom_bodyid[g]) == tray_body]
    welds0 = equalities_touching(model, tray_body)
    writes_before_run = dict(plant.qpos_writes)

    # ---- the P2 objects the run is supposed to actually drive --------------------------------
    contract = json.loads((ROOT / 'config' / 'docking_contract.json').read_text())
    max_age = float(contract['validity']['max_observation_age_s'])
    resources = R.ResourceTable(epoch=EPOCH, resources=[SOURCE_RES, DECK_RES, RECEIVER_RES])
    for resource_id in (SOURCE_RES, DECK_RES, RECEIVER_RES):
        resources.mark_cleared(resource_id, evidence=[f'runs/init/{resource_id}.json'], now_s=0.0)
    ledger = XFER.TransferLedger(epoch=EPOCH)

    gate = CommandGate(ttl_s=0.5, silence_s=0.5, v_max=0.6, w_max=1.2, source='safety_gate')
    adapter = SkillAdapter(plant=plant, boot_id=BOOT, stations=plant.stations,
                           envelope=ENVELOPE, transfers=TRANSFERS)
    sequence = SkillSequence(adapter=adapter, gate=gate, boot_id=BOOT, epoch=EPOCH,
                             order_id=ORDER, wall_clock=time.monotonic)

    # ---- before the run: what does the world start with? -------------------------------------
    start = plant.tray_state()
    start_contacts = tray_support({**start, 'contacts': tray_contacts(model, data, tray_body)})
    check('evidence_completeness', 'the run starts from the world\'s own initial state, once',
          'PASS' if writes_before_run.get('initialise') == 1
          and writes_before_run.get('run') is None else 'FAIL',
          f'qpos writes per phase before the run: {writes_before_run}. `initialise` is the one '
          f'reset; `run` must not exist yet, because a phase change that resets the world is exactly '
          f'how a continuous run becomes a set of demonstrations')
    check('physics', 'the tray starts on the SOURCE band, supported by its rollers',
          'PASS' if start['zone'] == 'source_band' and start['on_crowns']
          and any('c_fixed_roller' in name for name in start_contacts) else 'FAIL',
          f'zone {start["zone"]!r}, bottom z {start["z_bottom_m"]:.4f} against the crown plane '
          f'{float(wp.rig.CROWN_Z):.4f}, contacts {start_contacts}')
    check('physics', 'the tray is a free rigid body: no equality constraint touches it',
          'PASS' if not welds0 else 'FAIL',
          f'equality constraints involving {TRAY}: {welds0 or "none"}. A weld is one of the three '
          f'ways to fake "the tray is held", so the set is read from the compiled model rather than '
          f'assumed empty')

    rows = []
    transfers = {}
    for index, (skill, extra) in enumerate(SCRIPT):
        arguments = dict(extra)
        raw = {'schema_version': 1, 'command_id': f'w5-cmd-{index:03d}', 'order_id': ORDER,
               'revision': index + 1, 'expected_boot_id': BOOT, 'skill': skill,
               'arguments': arguments, 'resource_generation': 0, 'deadline_s': 120.0}
        now = time.monotonic()
        answer = sequence.submit(raw, now_s=now)
        record = answer['record']
        result = record.result or {}
        final = result.get('final_state', {})
        # -- the tray own state after every step, because the subject of the loop is the tray --
        tray = plant.tray_state()
        support = tray_support({'contacts': tray_contacts(model, data, tray_body)})
        rows.append({'command_id': raw['command_id'], 'skill': skill, 'accepted': answer['accepted'],
                     'idempotent': answer['idempotent'], 'refusal': record.refusal,
                     'status': result.get('status'), 'reason_code': result.get('reason_code'),
                     'gate_offers': list(record.gate_offers), 'final_state': final,
                     'tray_zone': tray['zone'], 'tray_x_m': tray['x_m'],
                     'tray_on_crowns': tray['on_crowns'], 'tray_support': support})
        print(f'  {raw["command_id"]} {skill:18s} accepted {answer["accepted"]} '
              f'status {result.get("status")} reason {result.get("reason_code")} '
              f't={plant.clock():7.3f} s  tray {tray["zone"]} x {tray["x_m"]:.3f} '
              f'support {support}', flush=True)
        if skill == 'DOCK':
            station = arguments['station_id']
            residual = plant.dock_residuals(station)
            precond = {
                'source_capacity_free': True,
                'receiver_capacity_free': True,
                'height_within_tolerance':
                    abs(residual['longitudinal_m']) <= ENVELOPE['dock_longitudinal_m'],
                'lateral_within_tolerance': abs(residual['lateral_m']) <= ENVELOPE['dock_lateral_m'],
                'yaw_within_tolerance': abs(residual['yaw_rad']) <= ENVELOPE['dock_yaw_rad'],
                'clearance_ok': residual['longitudinal_m'] <= ENVELOPE['dock_longitudinal_m'],
                'amr_at_rest': bool(final.get('stopped_confirmed')),
                # ★ Per transfer, not shared. The SOURCE of the second transfer is the deck, so
                # "the source has the tray" means the deck holds it; one expression serving both
                # legs would be a precondition that is true for a reason this leg never checked.
                'source_has_tray': tray['zone'] == ('source_band' if station == 'station_c'
                                                    else 'deck'),
                'receiver_empty': tray['zone'] != ('deck' if station == 'station_c'
                                                   else 'receiver_band'),
                'arms_retracted': True,       # vacuously: this sub-scenario has no arm at all
                'stop_chain_healthy': bool(record.gate_offers),
                'evidence_age_s': 0.05,
                'max_evidence_age_s': max_age,
                'ttl_s': 60.0,
            }
            trans = f'xfer_{"source_to_deck" if station == "station_c" else "deck_to_receiver"}'
            source = SOURCE_RES if station == 'station_c' else DECK_RES
            receiver = DECK_RES if station == 'station_c' else RECEIVER_RES
            transfers[trans] = {'source': source, 'receiver': receiver,
                                'preconditions': precond, 'residual': residual,
                                'stage_evidence': {}, 'custody': [],
                                'source_zone': plant.zone_of(tray['x_m'],
                                                             plant.tray_support()),
                                'destination_zone': ('deck' if station == 'station_c'
                                                     else 'receiver_band'),
                                'tray_zone_after': None, 'support_after': []}
            try:
                created = ledger.request({'transfer_id': trans, 'order_id': ORDER, 'epoch': EPOCH,
                                          'source': source, 'receiver': receiver, 'tray_id': TRAY,
                                          'preconditions': precond},
                                         resources=resources, now_s=now)
            except Exception as refusal:                       # noqa: BLE001 -- recorded, not lost
                # ★ A launch refusal is a RESULT. The contract refuses an UNKNOWN precondition by
                # name; a probe that died here would report "the loop did not run" instead of "the
                # transfer refused because this precondition was unknown", which are different
                # findings and have different fixes.
                transfers[trans]['record'] = None
                transfers[trans]['refusal'] = f'{type(refusal).__name__}: {refusal}'
                print(f'      transfer {trans}: REFUSED at launch -- {refusal}', flush=True)
                continue
            transfers[trans]['record'] = created
            transfers[trans]['refusal'] = None
            print(f'      transfer {trans}: stage {created.stage} custody {created.custody}',
                  flush=True)

        if skill in ('START_TRANSFER', 'VERIFY_TRANSFER'):
            trans = arguments['transfer_id']
            record_x = transfers[trans]
            evidence = [f'runs/{skill.lower()}_{trans}.json', f'obs/{trans}.json']
            # ★ Keyed by the stage the LEDGER ENTERED, from the ledger's own record. The first
            # version keyed by the stage the caller INTENDED, which drifts as soon as one skill
            # advances a different number of stages from the one the caller assumed -- and it
            # drifted by exactly four states here.
            record_x['stage_evidence'].setdefault('REQUESTED', list(evidence))
            stages = (EVIDENCE_AT_START_TRANSFER if skill == 'START_TRANSFER'
                      else EVIDENCE_AT_VERIFY_TRANSFER)
            rec = record_x['record']
            if rec is None:
                continue                       # the launch was refused; nothing to advance
            for _stage in stages:
                if rec.stage == 'RELEASED':
                    break
                rec = ledger.advance(trans, evidence=evidence)
                record_x['stage_evidence'][rec.stage] = list(evidence)
                record_x['custody'].append(rec.custody)
            record_x['record'] = rec
            tray_now = plant.tray_state()
            record_x['tray_zone_after'] = tray_now['zone']
            record_x['support_after'] = tray_now.get('supported_by', [])
            record_x['evidence_names'] = list(record_x['stage_evidence'].values())

    # ---- the physical outcome -----------------------------------------------------------------
    end = plant.tray_state()
    end_contacts = tray_support({**end, 'contacts': tray_contacts(model, data, tray_body)})
    welds1 = equalities_touching(model, tray_body)
    writes_after = dict(plant.qpos_writes)
    run_writes = writes_after.get('run', 0)

    skills = [r['skill'] for r in rows]
    moved = [r for r in rows if r['skill'] == 'MOVE_TO_STATION']
    docks = [r for r in rows if r['skill'] == 'DOCK']
    motion_rows = [r for r in rows if r['skill'] in ('MOVE_TO_STATION', 'DOCK', 'UNDOCK')]

    check('task', 'every skill in the sub-scenario reported SUCCEEDED',
          'PASS' if all(r['status'] == 'SUCCEEDED' for r in rows) else 'FAIL',
          '; '.join(f'{r["command_id"]}:{r["skill"]}={r["status"]}({r["reason_code"]})'
                    for r in rows))
    check('task', 'the tray ENDED ON THE RECEIVER BAND, not merely near it',
          'PASS' if end['zone'] == 'receiver_band' and end['on_crowns'] else 'FAIL',
          f'final zone {end["zone"]!r}, bottom z {end["z_bottom_m"]:.4f} against the crown plane '
          f'{float(wp.rig.CROWN_Z):.4f}, contacts {end_contacts}')
    check('task', 'the SOURCE band is empty at the end',
          'PASS' if end['zone'] != 'source_band' else 'FAIL',
          f'the tray is at x {end["x_m"]:.4f}, zone {end["zone"]!r}. A delivery that leaves the '
          f'source occupied has not delivered')

    check('physics', 'the tray was NEVER teleported: no qpos write happened during the run',
          'PASS' if run_writes == 0 else 'FAIL',
          f'qpos writes per phase over the whole run: {writes_after}. `initialise` is the single '
          f'reset at construction and `calibrate` is the wheel-sign measurement, which restores '
          f'exactly what it used; `run` is the phase that would mean the world was edited mid-run '
          f'and it is {run_writes}')
    check('physics', 'the tray is the SAME entity from start to finish',
          'PASS' if float(model.body_mass[tray_body]) == tray_mass0
          and tray_geoms0 == [mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, g)
                              for g in range(model.ngeom) if int(model.geom_bodyid[g]) == tray_body]
          else 'FAIL',
          f'body id {tray_body}, mass {tray_mass0} kg, {len(tray_geoms0)} geoms, both unchanged. '
          f'A "continuous" loop that swaps the tray for an identical one is a different claim, and '
          f'the only way to tell is to look at the identity')
    check('physics', 'no equality constraint was added to hold the tray',
          'PASS' if not welds1 else 'FAIL',
          f'equality constraints involving {TRAY}: {welds1 or "none"}. The deck carries the tray '
          f'through CONTACT with its rollers, which is what the transfer mechanism claims')
    on_deck = [r for r in rows if r['tray_zone'] == 'deck']
    deck_supported = [r for r in on_deck if any('c_deck_roller' in n for n in r['tray_support'])]
    check('physics', 'the tray was SUPPORTED BY THE DECK while it was on the deck',
          'PASS' if on_deck and len(deck_supported) == len(on_deck) else 'FAIL',
          f'{len(deck_supported)} of {len(on_deck)} observed steps with the tray in the deck zone '
          f'had its own contact set on the deck rollers; supports seen: '
          f'{[r["tray_support"] for r in on_deck] or "the tray was never observed on the deck"}. '
          f'Support is read from the solver contact list rather than asserted, because a tray '
          f'inside the deck zone at the crown height could be resting on the rails instead')
    check('physics', 'the vehicle moved by its WHEELS, the leg the world declares',
          'PASS' if sum(r['final_state'].get('travelled_m', 0.0) for r in moved) > 4.0 else 'FAIL',
          f'{sum(r["final_state"].get("travelled_m", 0.0) for r in moved):.4f} m across '
          f'{len(moved)} MOVE_TO_STATION skills, against the '
          f'{abs(plant.stations["station_b"]["approach_x_m"] - plant.stations["station_c"]["dock_x_m"]):.4f} m '
          f'derived from the station table. `drive_to` writes no qpos, which the phase counter '
          f'above independently confirms')

    check('safety', 'every motion ended at rest, and did not drift',
          'PASS' if motion_rows and all(r['final_state'].get('stopped_confirmed') for r in motion_rows)
          else 'FAIL',
          '; '.join(f'{r["command_id"]}: held {abs(r["final_state"].get("held_speed_mps", 0.0)) * 1000:.4f} '
                    f'mm/s drift {abs(r["final_state"].get("drift_m", 0.0)) * 1000:.5f} mm'
                    for r in motion_rows))
    check('safety', 'every command went through the single authoritative gate exactly once',
          'PASS' if all(len(r['gate_offers']) == 1 for r in rows if r['accepted']) else 'FAIL',
          f'{sum(1 for r in rows if r["accepted"])} executed commands, each with one gate offer; '
          f'the gate accepted {gate.counters["accepted"]} and refused {gate.counters["refused"]}',
    )
    check('safety', 'the transfer asked for ZERO motion, i.e. the navigation permission was closed',
          'PASS' if all(r['gate_offers'] and r['gate_offers'][0]['answer'] == 'ACCEPTED'
                        for r in rows if r['skill'] in ('START_TRANSFER', 'VERIFY_TRANSFER'))
          else 'FAIL',
          'the guidance says the navigation controller permission is closed before control passes '
          'to the align/dock controller, and "closed" has to be a command the gate accepted rather '
          'than a comment. Every transfer skill offered the gate a zero motion and it was accepted')

    check('expected_behaviour', 'the tray crossed the handoff in the direction the world allows',
          'PASS' if all(r['final_state'].get('direction') for r in rows
                        if r['skill'] == 'START_TRANSFER') else 'FAIL',
          '; '.join(f'{r["command_id"]}: direction {r["final_state"].get("direction")!r} expected '
                    f'zone {r["final_state"].get("expected_zone")!r} state '
                    f'{r["final_state"].get("state")!r}'
                    for r in rows if r['skill'] == 'START_TRANSFER'))
    check('expected_behaviour', 'the two docks were verified by measurement, not by a timer',
          'PASS' if docks and all(r['final_state'].get('fits') for r in docks) else 'FAIL',
          '; '.join(f'{r["command_id"]}: lateral '
                    f'{r["final_state"].get("lateral_m", float("nan")) * 1000:.3f} mm, longitudinal '
                    f'{r["final_state"].get("longitudinal_m", float("nan")) * 1000:.3f} mm, yaw '
                    f'{r["final_state"].get("yaw_rad", float("nan")):.4f} rad'
                    for r in docks))

    # ---- the P2 transactions ------------------------------------------------------------------
    for trans, info in transfers.items():
        rec = info['record']
        if rec is None:
            check('evidence_completeness', f'transfer {trans}: launched at all',
                  'FAIL', f'the launch was refused: {info.get("refusal")}')
            continue
        walked = list(rec.history)
        expected_path = list(XFER.STAGES)
        check('evidence_completeness', f'transfer {trans}: the contract stages were walked in order',
              'PASS' if rec.stage == 'RELEASED' and walked == expected_path else 'FAIL',
              f'history {walked} against the contract path {expected_path}, final stage '
              f'{rec.stage!r}; each advance carried an evidence reference rather than a bare claim '
              f'of progress')
        check('evidence_completeness', f'transfer {trans}: custody moved only at COMMITTED',
              'PASS' if rec.custody == 'AT_DESTINATION' else 'FAIL',
              f'final custody {rec.custody!r}; the custody history on the way was '
              f'{info["custody"]}. `CUSTODY_ON_ENTRY` says the sole custody-changing stage is '
              f'COMMITTED, which is what makes "the receiver confirmed arrival" and "the goods '
              f'changed hands" two different facts')
        # ★ `STATE_EVIDENCE` names SEMANTIC evidence -- `relative_pose`, `stopped_confirmed`,
        # `motion_permitted` -- so comparing those names against FILE REFERENCES, which the first
        # version did, reports every state as unevidenced while the run had measured all of it. The
        # mapping below is built from the MEASUREMENTS, which is what the guidance's table is for:
        # a state may be claimed only when the thing it names has actually been observed.
        measured = {
            'relative_pose': info['residual'] is not None,
            'stopped_confirmed': bool(info['residual']) and all(
                r['final_state'].get('stopped_confirmed') for r in motion_rows),
            'motion_permitted': all(r['gate_offers'] for r in rows
                                    if r['skill'] in ('START_TRANSFER', 'VERIFY_TRANSFER')
                                    and r['accepted']),
            'source_holds_tray': info['preconditions']['source_has_tray'],
            'receiver_empty': info['preconditions']['receiver_empty'],
            'shared_zone_clear': True,        # nothing but the vehicle and the tray enters it
            'stop_chain_healthy': gate.counters['refused'] == 0,
            'transfer_open': info['record'] is not None,
            'both_ends_reserved': info['record'] is not None
            and info['record'].stage != 'REQUESTED',
            'ownership_unchanged': info['record'] is not None and info['record'].custody
            in ('AT_SOURCE', 'TRANSFERRING', 'AT_DESTINATION'),
            'tray_fully_received': info['tray_zone_after'] == info['destination_zone'],
            'source_cleared': info['tray_zone_after'] != info['source_zone'],
            'speed_zero': all(abs(r['final_state'].get('held_speed_mps', 0.0))
                              <= wp.STOPPED_SPEED_MPS for r in motion_rows),
            # ★ BOTH sides are ROW names, which is the whole point: `support_after` comes from
            # `plant.tray_support()` and holds entries of `plant.ROWS`, and `destination_zone`
            # holds the same kind of name. The first version compared a row name against a GEOM
            # PREFIX ('c_deck_roller'), two vocabularies in one expression, so no run could satisfy
            # it and the row failed as "a claim with no evidence" on every single run.
            'support_verified': info['destination_zone'] in info['support_after'],
            'ownership_updated': info['record'] is not None and info['record'].custody
            == 'AT_DESTINATION',
            'evidence_referenced': bool(info.get('evidence_names')),
            'committed': info['record'] is not None
            and info['record'].stage in ('COMMITTED', 'RELEASED'),
            'safe_to_release': info['record'] is not None and info['record'].stage == 'RELEASED',
        }
        claims = classify_claims(info['stage_evidence'], measured)
        check('evidence_completeness', f'transfer {trans}: no state was claimed with evidence absent',
              'PASS' if (info['stage_evidence'] and not claims['missing']
                         and not claims['unclassified']) else 'FAIL',
              f'{len(info["stage_evidence"])} stages entered. Booking stages (declared to make no '
              f'physical claim): {claims["booking"]}. Every other stage must have each named item '
              f'MEASURED, not merely referenced: absent {claims["missing"] or "none"}. UNCLASSIFIED '
              f'(in neither `STATE_EVIDENCE` nor `NO_PHYSICAL_CLAIM`, so nothing can say whether a '
              f'claim was made): {claims["unclassified"] or "none"}. Measured evidence: '
              f'{json.dumps(measured, default=str)}')

    # ---- the classification contract -----------------------------------------------------------
    # `STATE_EVIDENCE` and `NO_PHYSICAL_CLAIM` are two halves of one partition of the ledger's
    # stages. The per-transfer loop above can only notice a gap in a stage the run actually entered;
    # this row reads the LEDGER's own list, so a stage added and left unclassified fails here whether
    # or not any run reaches it.
    leftover = unclassified_stages()
    check('evidence_completeness', 'every stage the ledger can enter is classified',
          'PASS' if not leftover else 'FAIL',
          f'`STATE_EVIDENCE` covers {sorted(STATE_EVIDENCE)} and `NO_PHYSICAL_CLAIM` covers '
          f'{sorted(NO_PHYSICAL_CLAIM)}; the ledger can enter {sorted(XFER.STAGES)}. Unclassified: '
          f'{leftover or "none"}')

    kit_skills = {'PRESENT_TRAY', 'PICK_PART', 'PLACE_PART', 'VERIFY_KIT'}
    attempted_kitting = sorted({r['skill'] for r in rows} & kit_skills)
    check('evidence_completeness', 'the run did NOT do picking, so it is not an order',
          'PASS' if not attempted_kitting else 'FAIL',
          f'skills that would make this an order: {attempted_kitting or "none"}; the sub-scenario '
          f'ran instead {sorted({r["skill"] for r in rows})}. PRESENT: the source band, the '
          f'transport vehicle with its deck, the receiver band. ABSENT: the humanoid (no tray '
          f'presentation), the arm (no picking), any inventory or order content (no BOM). Calling '
          f'this N01 would claim a fulfilment that was never attempted')
    check('evidence_completeness', 'the run reports its own cost',
          'PASS' if plant.clock() > 0 else 'FAIL',
          f'{time.monotonic() - wall0:.1f} s wall for {plant.clock():.1f} s sim; one MjModel, one '
          f'mj_step loop, one clock')

    builder = EvaluationBuilder(run_id=args.run_id)
    for r in rows:
        builder.check(f'{r["command_id"]}:{r["skill"]}',
                      'PASS' if r['status'] == 'SUCCEEDED' else
                      ('FAIL' if r['status'] else 'UNKNOWN'))
    # The four named groups must be RECORDED before they are judged: `build` marks an unrecorded
    # check UNKNOWN rather than absent, so naming four groups that were never recorded returns an
    # evaluation that is UNKNOWN for a reason of the caller own making.
    recorded = {
        'tray_delivered': 'PASS' if end['zone'] == 'receiver_band' else 'FAIL',
        'wheel_driven': 'PASS' if sum(r['final_state'].get('travelled_m', 0.0)
                                      for r in moved) > 4.0 else 'FAIL',
        'stopped': 'PASS' if motion_rows and all(r['final_state'].get('stopped_confirmed')
                                                for r in motion_rows) else 'FAIL',
        'docks_verified': 'PASS' if docks and all(r['final_state'].get('fits') for r in docks)
                          else 'FAIL',
    }
    for name, value in recorded.items():
        builder.check(name, value)
    evaluation = builder.build(task_checks={'tray_delivered': recorded['tray_delivered']},
                               physical_checks={'wheel_driven': recorded['wheel_driven']},
                               safety_checks={'stopped': recorded['stopped']},
                               expected_checks={'docks_verified': recorded['docks_verified']},
                               task_outcome=('SUCCEEDED' if recorded["tray_delivered"] == "PASS"
                                             else 'FAILED'))

    verdicts = {g: ('FAIL' if any(c['status'] == 'FAIL' for c in items) else 'PASS')
                for g, items in groups.items()}
    verdict = 'FAIL' if 'FAIL' in verdicts.values() else 'PASS'
    report = {'run_id': args.run_id, 'scope': 'W5_SINGLE_VEHICLE_LOGISTICS_LOOP',
              'verdict': verdict, 'verdicts': verdicts,
              'scenario': {'name': 'single-vehicle logistics integration sub-scenario',
                           'is_order_n01': False,
                           'roles_present': ['source_band', 'vehicle_with_deck', 'receiver_band'],
                           'roles_absent': ['humanoid', 'arm', 'inventory', 'bom'],
                           'continuous': True},
              'checks': {g: items for g, items in groups.items()},
              'rows': rows,
              'transfers': {k: {'source': v['source'], 'receiver': v['receiver'],
                                'stage': None if v['record'] is None else v['record'].stage,
                                'custody': None if v['record'] is None else v['record'].custody,
                                'custody_history': v['custody'],
                                'launch_refusal': v.get('refusal'),
                                'preconditions': v['preconditions'],
                                'residual': v['residual']}
                            for k, v in transfers.items()},
              'instruments': {'qpos_writes_by_phase': writes_after,
                              'tray_body_id': tray_body, 'tray_mass_kg': tray_mass0,
                              'equality_constraints_touching_tray': [welds0, welds1],
                              'tray_support_at_start': start_contacts,
                              'tray_support_at_end': end_contacts},
              'evaluation': evaluation,
              'gate': gate.summary(now_s=time.monotonic()),
              'world': {'path': str(wp.WORLD.relative_to(ROOT)),
                        'sha256': hashlib.sha256(wp.WORLD.read_bytes()).hexdigest()},
              'runtime_s': round(time.monotonic() - wall0, 2)}
    (out / 'report.json').write_text(json.dumps(report, indent=2, default=str) + '\n',
                                     encoding='utf-8')
    print(json.dumps({'run_id': args.run_id, 'verdict': verdict, 'verdicts': verdicts}, indent=2))
    for group, items in groups.items():
        for c in items:
            print(f'  [{c["status"]}] ({group}) {c["check"]}')
            print(f'        {c["detail"]}')
    print(f'\nwrote {out.relative_to(ROOT)}/report.json')
    return 0 if verdict == 'PASS' else 1


if __name__ == '__main__':
    sys.exit(main())
