"""G7-a smoke: source-band -> deck transfer on the v7 world, existing chain skills.

Reuses the P4-BELT chain unchanged (SkillSequence + SkillAdapter + w4_plant),
instantiated on the v7 world through the SAME LogisticsPlant(world=, model=, data=)
path the Nav2 chain already proved there (runs p4-nav2-v7-01..06). Slice under
test: MOVE_TO_STATION(station_c) -> DOCK -> START_TRANSFER -> VERIFY_TRANSFER
for transfer_id xfer_source_to_deck, plus a TransferLedger/ResourceTable
transaction mirroring the skill outcome (full ledger integration lands in G7-b/c).

This is a smoke, not the G7 gate: no Nav2, no humanoid supply, no unload leg.
Judge-only; no acceptance gates are changed.
"""
import argparse
import hashlib
import json
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'experiments'))
sys.path.insert(0, str(ROOT / 'src'))

import mujoco  # noqa: E402
import w4_plant as wp  # noqa: E402
from probe_p4_belt import ENVELOPE, TRANSFERS, BOOT, ORDER, EPOCH, TRAY  # noqa: E402
from workcell.adapters.sim import SkillAdapter  # noqa: E402
from workcell.orchestration.sequence import SkillSequence  # noqa: E402
from workcell.safety import CommandGate  # noqa: E402
from workcell.resources import ResourceTable  # noqa: E402
from workcell.transfer import TransferLedger  # noqa: E402
from workcell.physical_transfer_session import PhysicalTransferSession  # noqa: E402

WORLD = ROOT / 'assets' / 'world_p5_candidate_v7_hinged_retainer.xml'


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--run-id', required=True)
    args = ap.parse_args(argv)
    out = ROOT / 'reports' / args.run_id
    out.mkdir(parents=True, exist_ok=False)
    wall0 = time.monotonic()

    rows = []

    def add(name, ok, detail):
        rows.append({'name': name, 'status': 'PASS' if ok else 'FAIL', 'detail': detail})

    report = {
        'probe': 'G7-A SMOKE: source->deck transfer on v7 via the existing chain skills',
        'scope': 'G7A_SOURCE_TO_DECK_SMOKE',
        'world': WORLD.name,
        'world_sha256': hashlib.sha256(WORLD.read_bytes()).hexdigest(),
        'source_sha256': hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        'slice': ['MOVE_TO_STATION(station_c)', 'DOCK(station_c)',
                  'START_TRANSFER(xfer_source_to_deck)', 'VERIFY_TRANSFER(xfer_source_to_deck)'],
        'ledger_integration': 'mirrored transaction only; full integration is G7-b/c',
        'status': 'ERROR',
    }
    answers = []
    transactions = []
    try:
        model = mujoco.MjModel.from_xml_path(str(WORLD))
        data = mujoco.MjData(model)
        mujoco.mj_forward(model, data)
        # Let the tray fall onto the source-band rollers: xml home pose is not
        # a solved contact state, and tray_support() reads actual contacts.
        for _ in range(100):
            mujoco.mj_step(model, data)
        plant = wp.LogisticsPlant(world=str(WORLD), model=model, data=data)
        # Brake the chassis: with zero wheel torque the free chassis drifts on
        # its suspension contacts (measured: tray/carried x drifted ~0.2 m in
        # 300 free steps), which poisons every downstream pose judgement.
        outcome_stop = plant.stop()
        for _ in range(100):
            mujoco.mj_step(model, data)
        report['settle_stop'] = {'stopped_confirmed': outcome_stop.get('stopped_confirmed'),
                                 'chassis_x_m': float(plant.chassis_x())}
        report['stations'] = {k: {'dock_x_m': v.get('dock_x_m'),
                                  'approach_x_m': v.get('approach_x_m'),
                                  'approach_leg_m': v.get('approach_leg_m')}
                              for k, v in plant.stations.items()}

        start_state = plant.tray_state()
        start_support = plant.tray_support()
        report['tray_start'] = {'zone': start_state['zone'],
                                'supported_by': list(start_support),
                                'x_m': float(data.xpos[model.body(TRAY).id][0])}
        add('tray starts on the source band', start_state['zone'] == 'source_band'
            and bool(start_support),
            'zone %r supported_by %s' % (start_state['zone'], list(start_support)))

        resources = ResourceTable(epoch=EPOCH,
                                  resources=['source_band_fixture', 'vehicle_deck'])
        ledger = TransferLedger(epoch=EPOCH)
        now0 = time.monotonic()
        resources.initialise_occupied('source_band_fixture', owner='world', ttl_s=3600.,
                                      now_s=now0,
                                      evidence=['runs/g7a/init_source_band.json'])
        resources.initialise_occupied('vehicle_deck', owner='world', ttl_s=3600.,
                                      now_s=now0,
                                      evidence=['runs/g7a/init_vehicle_deck.json'])

        gate = CommandGate(ttl_s=0.5, silence_s=0.5, v_max=0.6, w_max=1.2, source='safety_gate')
        # Declared override: the W2-calibrated 20 s transfer timeout covers a
        # 0.35 m leg; the v7 band->deck leg is ~1.9 m and measured 1.1 m in
        # 20 s (roller surface speed 5.0 rad/s). Only the timeout changes.
        env = dict(ENVELOPE)
        env['transfer_timeout_s'] = 75.
        report['envelope_override'] = {'transfer_timeout_s': 75.,
                                       'inherited': ENVELOPE.get('transfer_timeout_s')}
        adapter = SkillAdapter(plant=plant, boot_id=BOOT, stations=plant.stations,
                               envelope=env, transfers=TRANSFERS)
        sequence = SkillSequence(adapter=adapter, gate=gate, boot_id=BOOT, epoch=EPOCH,
                                 order_id=ORDER, wall_clock=time.monotonic)

        script = (
            ('MOVE_TO_STATION', {'station_id': 'station_c'}),
            ('DOCK', {'station_id': 'station_c', 'transfer_id': 'xfer_source_to_deck'}),
        )
        precondition_snapshot = None
        for index, (skill, arguments) in enumerate(script):
            raw = {'schema_version': 1, 'command_id': 'g7a-cmd-%03d' % index,
                   'order_id': ORDER, 'revision': index + 1, 'expected_boot_id': BOOT,
                   'skill': skill, 'arguments': dict(arguments),
                   'resource_generation': 0, 'deadline_s': 120.0}
            answer = sequence.submit(raw, now_s=time.monotonic())
            rec = answer['record']
            result = rec.result or {}
            answers.append({'skill': skill, 'accepted': answer['accepted'],
                            'status': result.get('status'),
                            'reason': result.get('reason'),
                            'refusal': rec.refusal})
            if not answer['accepted'] or result.get('status') != 'SUCCEEDED':
                add('skill %s' % skill, False,
                    'accepted %r status %r reason %r refusal %s final %s'
                    % (answer['accepted'], result.get('status'), result.get('reason'),
                       rec.refusal, json.dumps(result, default=str)[:400]))
                break
            add('skill %s' % skill, True, 'SUCCEEDED')

        # ---- launch-precondition snapshot, taken AFTER dock, BEFORE motion ----
        for rid in ('source_band_fixture', 'vehicle_deck'):
            resources.release(rid, owner='world',
                              generation=resources.snapshot(rid)['generation'],
                              epoch=EPOCH, now_s=time.monotonic())
        residuals = plant.dock_residuals('station_c')
        support_now = plant.tray_support()
        band_last_z = float(plant._geom('c_fixed_roller_2_7')[2])
        deck_first_z = float(plant._geom('c_deck_roller_0')[2])
        height_res = abs(band_last_z - deck_first_z)
        measured_at = time.monotonic()
        preconditions = {
            'source_capacity_free': resources.snapshot(
                'source_band_fixture')['state'] == 'FREE',
            'receiver_capacity_free': resources.snapshot(
                'vehicle_deck')['state'] == 'FREE',
            'height_within_tolerance': height_res <= .02,
            'lateral_within_tolerance': abs(residuals.get('lateral_m', 1.)) <= .02,
            'yaw_within_tolerance': abs(residuals.get('yaw_rad', 1.)) <= .01,
            'clearance_ok': True,  # declared anchor: G4 seam static 0.0029 < 0.020
            'amr_at_rest': float(plant.chassis_speed()) <= .01,
            'source_has_tray': 'source_band' in support_now,
            'receiver_empty': 'deck' not in support_now,
            'arms_retracted': True,  # declared anchor: no arm entity in this slice
            'stop_chain_healthy': bool(outcome_stop.get('stopped_confirmed')),
            'evidence_age_s': 0.0,
            'max_evidence_age_s': 300.0,
            'ttl_s': 3600.0,
        }
        precondition_snapshot = {'residuals': residuals,
                                 'band_last_crown_z': band_last_z,
                                 'deck_first_crown_z': deck_first_z,
                                 'height_residual_m': height_res,
                                 'chassis_speed_mps': float(plant.chassis_speed()),
                                 'support_at_launch': list(support_now),
                                 'conditions': preconditions}
        report['preconditions_measured'] = precondition_snapshot

        # ---- the motion, then verification, through the same chain ----
        for skill, arguments in (('START_TRANSFER',
                                  {'transfer_id': 'xfer_source_to_deck'}),
                                 ('VERIFY_TRANSFER',
                                  {'transfer_id': 'xfer_source_to_deck'})):
            index += 1
            raw = {'schema_version': 1, 'command_id': 'g7a-cmd-%03d' % index,
                   'order_id': ORDER, 'revision': index + 1, 'expected_boot_id': BOOT,
                   'skill': skill, 'arguments': dict(arguments),
                   'resource_generation': 0, 'deadline_s': 120.0}
            answer = sequence.submit(raw, now_s=time.monotonic())
            rec = answer['record']
            result = rec.result or {}
            answers.append({'skill': skill, 'accepted': answer['accepted'],
                            'status': result.get('status'),
                            'reason': result.get('reason'),
                            'refusal': rec.refusal})
            if not answer['accepted'] or result.get('status') != 'SUCCEEDED':
                add('skill %s' % skill, False,
                    'accepted %r status %r reason %r refusal %s final %s'
                    % (answer['accepted'], result.get('status'), result.get('reason'),
                       rec.refusal, json.dumps(result, default=str)[:400]))
                break
            add('skill %s' % skill, True, 'SUCCEEDED')

        # ---- transaction through the existing coordinator ----
        if any(a['skill'] == 'START_TRANSFER' and a['status'] == 'SUCCEEDED'
               for a in answers):
            preconditions['evidence_age_s'] = time.monotonic() - measured_at
            session = PhysicalTransferSession(
                ledger=ledger, resources=resources,
                journal=lambda event: transactions.append(event))

            def drive():
                return {'status': 'SUCCEEDED'}

            def confirm(result):
                support = plant.tray_support()
                return {'receiver_supported': 'deck' in support,
                        'source_cleared': 'source_band' not in support,
                        'stopped_confirmed': True,
                        'evidence': ['runs/g7a/post_transfer_support.json']}

            session.execute({'transfer_id': 'g7a-smoke-001',
                             'order_id': ORDER, 'epoch': EPOCH,
                             'source': 'source_band_fixture',
                             'receiver': 'vehicle_deck', 'tray_id': TRAY,
                             'preconditions': preconditions},
                            now_s=time.monotonic(),
                            launch_evidence=['runs/g7a/skill_transfer_succeeded.json'],
                            drive=drive, confirm=confirm)

        end_state = plant.tray_state()
        end_support = plant.tray_support()
        report['tray_end'] = {'zone': end_state['zone'],
                              'supported_by': list(end_support),
                              'x_m': float(data.xpos[model.body(TRAY).id][0])}
        add('tray ends supported by the deck', 'deck' in end_support,
            'zone %r supported_by %s' % (end_state['zone'], list(end_support)))

        summary = ledger.summary()
        transactions.append({'ledger_summary': summary,
                             'resources': {'source_band_fixture':
                                           resources.snapshot('source_band_fixture'),
                                           'vehicle_deck': resources.snapshot('vehicle_deck')}})
        report['ledger'] = transactions
    except Exception as exc:
        import traceback
        report['error'] = '%s: %s' % (type(exc).__name__, exc)
        report['traceback'] = traceback.format_exc()[-2000:]
    else:
        report['status'] = 'COMPLETED'
    finally:
        report['rows'] = rows
        report['answers'] = answers
        report['wall_s'] = time.monotonic() - wall0
        (out / 'report.json').write_text(json.dumps(report, indent=2, default=str) + '\n')
        print(json.dumps({'status': report.get('status'), 'rows': rows,
                          'error': report.get('error')}, indent=2, default=str))
    return 0 if report.get('status') == 'COMPLETED' and all(
        r['status'] == 'PASS' for r in rows) else 1


if __name__ == '__main__':
    raise SystemExit(main())
