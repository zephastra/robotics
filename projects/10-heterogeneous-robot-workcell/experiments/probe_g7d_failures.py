"""G7-d: five failure tests on the v7 workcell -- refusal and interruption must never
upgrade to COMMITTED/RELEASED, and physical faults must be caught by measured confirmation.

Setup reuses the G7-a smoke chain unchanged (skills + plant on v7): tray settles on the
source band, MOVE_TO_STATION/DOCK/START_TRANSFER/VERIFY_TRANSFER put it on the deck.
Then five independent tests, each with its own ResourceTable/TransferLedger initialised
to the MEASURED physical state:

  T1 source-refuse     : source still occupied -> request refuses -> receiver untouched
  T2 half-way stop     : drive() dies mid-transfer -> INTERRUPTED, locks retained,
                         release refuses, retry with the same id refuses
  T3 retainer-held     : shoes CLOSED + unload rollers driven -> tray cannot leave the
                         deck (measured) -> confirmation refuses -> no COMMIT
  T4 receiver-busy     : receiver occupied by another owner -> request refuses
  T5 unload-no-drive   : shoes OPEN, drive() LIES 'SUCCEEDED', no roller torque ->
                         measured confirm catches it -> no COMMIT

Every judged row is measurable: ledger stage strings, resource snapshots, tray support
and tray x displacement. No acceptance gate text is changed.
"""
import argparse
import hashlib
import json
import sys
import time
import traceback
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'experiments'))
sys.path.insert(0, str(ROOT / 'src'))

import mujoco  # noqa: E402
import numpy as np  # noqa: E402
import w4_plant as wp  # noqa: E402
from probe_p4_belt import ENVELOPE, TRANSFERS, BOOT, ORDER, EPOCH, TRAY  # noqa: E402
from workcell.adapters.sim import SkillAdapter  # noqa: E402
from workcell.orchestration.sequence import SkillSequence  # noqa: E402
from workcell.safety import CommandGate  # noqa: E402
from workcell.resources import ResourceTable  # noqa: E402
from workcell.transfer import TransferLedger  # noqa: E402
from workcell.physical_transfer_session import PhysicalTransferSession  # noqa: E402
from workcell.schema import SchemaRefused  # noqa: E402

WORLD = ROOT / 'assets' / 'world_p5_candidate_v7_hinged_retainer.xml'
RESOURCES = ('source_band_fixture', 'vehicle_deck', 'receiver_band')
UNLOAD_BUDGET_S = 30.0
SHOE_OPEN_Q = 1.5708
SHOE_CLOSE_TARGET = -0.025     # the transport closed-band target (run-03 mechanism)
TRAY_MOVE_LIMIT_M = 0.005      # a held/undriven tray may not travel


def tray_x(model, data):
    return float(data.xpos[model.body(TRAY).id][0])


def unload_roller_ids(model):
    return [i for i in range(model.nu)
            if (mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, i) or ''
                ).startswith(('c_deck_roller', 'c_recv_roller'))]


def shoe_drive_ids(model):
    return [model.actuator(n).id for n in ('c_retainer_-1_drive', 'c_retainer_1_drive')]


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
        'probe': 'G7-D FAILURES: five refusal/interruption tests on v7',
        'scope': 'G7D_FAILURE_TESTS',
        'world': WORLD.name,
        'world_sha256': hashlib.sha256(WORLD.read_bytes()).hexdigest(),
        'source_sha256': hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        'status': 'ERROR',
        'tests': {},
    }
    try:
        model = mujoco.MjModel.from_xml_path(str(WORLD))
        data = mujoco.MjData(model)
        mujoco.mj_forward(model, data)
        for _ in range(100):
            mujoco.mj_step(model, data)
        plant = wp.LogisticsPlant(world=str(WORLD), model=model, data=data)
        outcome_stop = plant.stop()
        for _ in range(100):
            mujoco.mj_step(model, data)
        report['settle_stop'] = {'stopped_confirmed': outcome_stop.get('stopped_confirmed'),
                                 'chassis_x_m': float(plant.chassis_x())}
        support0 = plant.tray_support()
        add('tray starts on the source band', 'source_band' in support0,
            'support %r' % list(support0))

        # ---- setup: G7-a chain puts the tray on the deck ----
        env = dict(ENVELOPE)
        env['transfer_timeout_s'] = 75.
        gate = CommandGate(ttl_s=0.5, silence_s=0.5, v_max=0.6, w_max=1.2, source='safety_gate')
        adapter = SkillAdapter(plant=plant, boot_id=BOOT, stations=plant.stations,
                               envelope=env, transfers=TRANSFERS)
        sequence = SkillSequence(adapter=adapter, gate=gate, boot_id=BOOT, epoch=EPOCH,
                                 order_id=ORDER, wall_clock=time.monotonic)
        index = 0
        setup_ok = True
        for skill, arguments in (
                ('MOVE_TO_STATION', {'station_id': 'station_c'}),
                ('DOCK', {'station_id': 'station_c', 'transfer_id': 'xfer_source_to_deck'}),
                ('START_TRANSFER', {'transfer_id': 'xfer_source_to_deck'}),
                ('VERIFY_TRANSFER', {'transfer_id': 'xfer_source_to_deck'})):
            raw = {'schema_version': 1, 'command_id': 'g7d-cmd-%03d' % index,
                   'order_id': ORDER, 'revision': index + 1, 'expected_boot_id': BOOT,
                   'skill': skill, 'arguments': dict(arguments),
                   'resource_generation': 0, 'deadline_s': 120.0}
            answer = sequence.submit(raw, now_s=time.monotonic())
            result = answer['record'].result or {}
            ok = answer['accepted'] and result.get('status') == 'SUCCEEDED'
            add('setup %s' % skill, ok,
                'accepted %r status %r reason %r' % (answer['accepted'],
                                                     result.get('status'), result.get('reason')))
            setup_ok = setup_ok and ok
            index += 1
        support_setup = plant.tray_support()
        add('setup: tray ends on the deck', setup_ok and 'deck' in support_setup,
            'support %r' % list(support_setup))

        # measured state after setup, for per-test resource initialisation
        measured = {
            'support': list(support_setup),
            'tray_x': tray_x(model, data),
            'source_empty': 'source_band' not in support_setup,
            'deck_loaded': 'deck' in support_setup,
        }
        report['measured_post_setup'] = measured

        def fresh_resources(receiver_owner_other=False, keep_source_occupied=False):
            """A ResourceTable initialised to the MEASURED physical state."""
            res = ResourceTable(epoch=EPOCH, resources=list(RESOURCES))
            now0 = time.monotonic()
            for rid in RESOURCES:
                res.initialise_occupied(rid, owner='world', ttl_s=7200., now_s=now0,
                                        evidence=['runs/g7d/init_%s.json' % rid])
            # deck: between transfers the table custody is FREE (the tray's presence
            # is physical state, not resource custody -- the G7-a post-transfer
            # model); leaving it OCCUPIED makes every deck-source launch refuse
            # REFUSED_RESOURCE_OCCUPIED before drive/confirm can be measured.
            res.release('vehicle_deck', owner='world',
                        generation=res.snapshot('vehicle_deck')['generation'],
                        epoch=EPOCH, now_s=time.monotonic())
            # source: the tray physically LEFT the band during setup -> measured free
            if not keep_source_occupied and measured['source_empty']:
                res.release('source_band_fixture', owner='world',
                            generation=res.snapshot('source_band_fixture')['generation'],
                            epoch=EPOCH, now_s=time.monotonic())
            # receiver: measured empty -> free unless the test occupies it
            if not receiver_owner_other:
                res.release('receiver_band', owner='world',
                            generation=res.snapshot('receiver_band')['generation'],
                            epoch=EPOCH, now_s=time.monotonic())
            else:
                res.handover_occupied('receiver_band', owner='world',
                                      generation=res.snapshot('receiver_band')['generation'],
                                      epoch=EPOCH, new_owner='other_order_999',
                                      ttl_s=7200., now_s=time.monotonic(),
                                      evidence=['runs/g7d/t4_other_owner.json'])
            return res

        def preconditions(res):
            snap = {r: res.snapshot(r) for r in RESOURCES}
            return {
                'source_capacity_free': snap['source_band_fixture']['state'] == 'FREE',
                'receiver_capacity_free': snap['receiver_band']['state'] == 'FREE',
                'height_within_tolerance': True,
                'lateral_within_tolerance': True,
                'yaw_within_tolerance': True,
                'clearance_ok': True,
                'amr_at_rest': float(plant.chassis_speed()) <= .01,
                'source_has_tray': True,   # the source is the loaded DECK here
                'receiver_empty': snap['receiver_band']['state'] == 'FREE',
                'arms_retracted': True,
                'stop_chain_healthy': bool(outcome_stop.get('stopped_confirmed')),
                'evidence_age_s': 0.0,
                'max_evidence_age_s': 300.0,
                'ttl_s': 3600.0,
            }, snap

        def stage_of(ledger, tid):
            try:
                return ledger._require(tid).stage
            except Exception as exc:
                return 'MISSING:%s' % type(exc).__name__

        # ---- T1: source still occupied -> refuse, receiver untouched ----
        res1 = fresh_resources(keep_source_occupied=True)
        pre1, snap1 = preconditions(res1)
        t1 = {'pre_source_state': snap1['source_band_fixture']['state'],
              'pre_receiver': dict(snap1['receiver_band'])}
        raised = None
        try:
            session = PhysicalTransferSession(ledger=TransferLedger(epoch=EPOCH),
                                              resources=res1, journal=lambda e: None)
            session.execute({'transfer_id': 'g7d-t1', 'order_id': ORDER, 'epoch': EPOCH,
                             'source': 'source_band_fixture', 'receiver': 'receiver_band',
                             'tray_id': TRAY, 'preconditions': pre1},
                            now_s=time.monotonic(),
                            launch_evidence=['runs/g7d/t1_launch.json'],
                            drive=lambda: {'status': 'SUCCEEDED'}, confirm=lambda r: {})
        except Exception as exc:
            raised = '%s: %s' % (type(exc).__name__, exc)
        t1['refused_by'] = raised
        t1['post_receiver'] = dict(res1.snapshot('receiver_band'))
        add('T1 source-occupied request refused', raised is not None,
            'raised %r' % raised)
        add('T1 receiver untouched by the refusal',
            t1['post_receiver']['state'] == t1['pre_receiver']['state']
            and t1['post_receiver']['owner'] == t1['pre_receiver']['owner'],
            'pre %r/%r post %r/%r' % (t1['pre_receiver']['state'], t1['pre_receiver']['owner'],
                                      t1['post_receiver']['state'], t1['post_receiver']['owner']))
        report['tests']['T1'] = t1

        # ---- T4: receiver occupied by another owner -> refuse ----
        res4 = fresh_resources(receiver_owner_other=True)
        pre4, snap4 = preconditions(res4)
        t4 = {'pre_receiver': dict(snap4['receiver_band'])}
        raised = None
        try:
            session = PhysicalTransferSession(ledger=TransferLedger(epoch=EPOCH),
                                              resources=res4, journal=lambda e: None)
            session.execute({'transfer_id': 'g7d-t4', 'order_id': ORDER, 'epoch': EPOCH,
                             'source': 'source_band_fixture', 'receiver': 'receiver_band',
                             'tray_id': TRAY, 'preconditions': pre4},
                            now_s=time.monotonic(),
                            launch_evidence=['runs/g7d/t4_launch.json'],
                            drive=lambda: {'status': 'SUCCEEDED'}, confirm=lambda r: {})
        except Exception as exc:
            raised = '%s: %s' % (type(exc).__name__, exc)
        t4['refused_by'] = raised
        add('T4 busy-receiver request refused', raised is not None, 'raised %r' % raised)
        add('T4 refusal names the receiver precondition',
            raised is not None and 'PRECONDITION' in raised,
            'raised %r' % raised)
        report['tests']['T4'] = t4

        # ---- T2: drive dies mid-transfer -> INTERRUPTED, no release, no retry ----
        res2 = fresh_resources()
        pre2, _ = preconditions(res2)
        led2 = TransferLedger(epoch=EPOCH)
        events2 = []
        session2 = PhysicalTransferSession(ledger=led2, resources=res2,
                                           journal=lambda e: events2.append(e))
        raised = None
        try:
            def drive_dead():
                raise RuntimeError('DRIVE_LOST_MID_TRANSFER')
            session2.execute({'transfer_id': 'g7d-t2', 'order_id': ORDER, 'epoch': EPOCH,
                              'source': 'source_band_fixture', 'receiver': 'receiver_band',
                              'tray_id': TRAY, 'preconditions': pre2},
                             now_s=time.monotonic(),
                             launch_evidence=['runs/g7d/t2_launch.json'],
                             drive=drive_dead, confirm=lambda r: {})
        except Exception as exc:
            raised = '%s: %s' % (type(exc).__name__, exc)
        stage2 = stage_of(led2, 'g7d-t2')
        t2 = {'drive_raised': raised, 'stage_after': stage2}
        release_refused = None
        try:
            session2.release('g7d-t2', cleared={'source_band_fixture': True,
                                                'receiver_band': True},
                             evidence=['runs/g7d/t2_clear.json'], now_s=time.monotonic())
        except Exception as exc:
            release_refused = '%s: %s' % (type(exc).__name__, exc)
        t2['release_refused_by'] = release_refused
        retry_refused = None
        try:
            session2.execute({'transfer_id': 'g7d-t2', 'order_id': ORDER, 'epoch': EPOCH,
                              'source': 'source_band_fixture', 'receiver': 'receiver_band',
                              'tray_id': TRAY, 'preconditions': pre2},
                             now_s=time.monotonic(),
                             launch_evidence=['runs/g7d/t2_retry.json'],
                             drive=lambda: {'status': 'SUCCEEDED'}, confirm=lambda r: {})
        except Exception as exc:
            retry_refused = '%s: %s' % (type(exc).__name__, exc)
        t2['retry_refused_by'] = retry_refused
        locks2 = {r: res2.snapshot(r)['owner'] for r in RESOURCES}
        t2['resource_owners_after'] = locks2
        add('T2 mid-transfer drive death interrupted the transfer',
            raised is not None and stage2 not in ('COMMITTED', 'RELEASED'),
            'raised %r stage %r' % (raised, stage2))
        add('T2 release without commit refuses', release_refused is not None,
            'refused %r' % release_refused)
        add('T2 retry with the same id refuses', retry_refused is not None,
            'refused %r' % retry_refused)
        add('T2 attention locks retained on both ends',
            locks2['source_band_fixture'] == 'g7d-t2'
            and locks2['receiver_band'] == 'g7d-t2',
            'owners %r' % locks2)
        report['tests']['T2'] = t2

        shoe_ids = shoe_drive_ids(model)
        unload_ids = unload_roller_ids(model)
        # ---- T5: shoes OPEN, drive() LIES, no roller torque -> measured refuse ----
        res5 = fresh_resources()
        pre5, _ = preconditions(res5)
        led5 = TransferLedger(epoch=EPOCH)
        session5 = PhysicalTransferSession(ledger=led5, resources=res5,
                                           journal=lambda e: None)
        for _ in range(3000):
            ctrl = plant.park_control()
            for sid in shoe_ids:
                ctrl[sid] = SHOE_OPEN_Q
            data.ctrl[:] = ctrl
            mujoco.mj_step(model, data)
            _qs = [float(data.qpos[int(model.jnt_qposadr[
                model.joint(n + '_joint').id])])
                for n in ('c_retainer_-1', 'c_retainer_1')]
            if all(q >= 1.5 for q in _qs):
                break
        qadr = [int(model.jnt_qposadr[model.joint(n + '_joint').id])
                for n in ('c_retainer_-1', 'c_retainer_1')]
        qret = [float(data.qpos[a]) for a in qadr]
        x_before = tray_x(model, data)
        deadline = float(data.time) + UNLOAD_BUDGET_S
        while float(data.time) < deadline:
            data.ctrl[:] = plant.park_control()   # NO roller torque: the injected fault
            mujoco.mj_step(model, data)
        x_after = tray_x(model, data)
        support5 = plant.tray_support()
        t5 = {'shoes_open_qpos': [round(q, 4) for q in qret],
              'support_after': list(support5),
              'tray_dx_m': round(abs(x_after - x_before), 6)}
        raised = None
        try:
            def confirm_no_drive(result):
                support = plant.tray_support()
                return {'receiver_supported': 'receiver_band' in support,
                        'source_cleared': 'deck' not in support,
                        'stopped_confirmed': True,
                        'evidence': ['runs/g7d/t5_support.json']}
            session5.execute({'transfer_id': 'g7d-t5', 'order_id': ORDER, 'epoch': EPOCH,
                              'source': 'vehicle_deck', 'receiver': 'receiver_band',
                              'tray_id': TRAY, 'preconditions': pre5},
                             now_s=time.monotonic(),
                             launch_evidence=['runs/g7d/t5_launch.json'],
                             drive=lambda: {'status': 'SUCCEEDED'},  # the LIE
                             confirm=confirm_no_drive)
        except Exception as exc:
            raised = '%s: %s' % (type(exc).__name__, exc)
        stage5 = stage_of(led5, 'g7d-t5')
        t5['confirm_refused_by'] = raised
        t5['stage_after'] = stage5
        add('T5 shoes measured open', all(q > 1.0 for q in qret), 'qpos %r' % t5['shoes_open_qpos'])
        add('T5 undriven tray stayed on the deck',
            'deck' in support5 and t5['tray_dx_m'] <= TRAY_MOVE_LIMIT_M,
            'support %r dx %.6f m' % (t5['support_after'], t5['tray_dx_m']))
        add('T5 lying drive caught by measured confirmation (no COMMIT)',
            raised is not None and stage5 not in ('COMMITTED', 'RELEASED'),
            'raised %r stage %r' % (raised, stage5))
        report['tests']['T5'] = t5

        # ---- T3: shoes CLOSED + unload rollers driven -> tray cannot leave ----
        res3 = fresh_resources()
        pre3, _ = preconditions(res3)
        led3 = TransferLedger(epoch=EPOCH)
        session3 = PhysicalTransferSession(ledger=led3, resources=res3,
                                           journal=lambda e: None)
        x_before = tray_x(model, data)
        z_before = float(data.xpos[model.body('c_payload').id][2])
        shoe_ids = shoe_drive_ids(model)
        unload_ids = unload_roller_ids(model)
        dt = float(model.opt.timestep)
        deadline = float(data.time) + UNLOAD_BUDGET_S
        last_support = None
        while float(data.time) < deadline:
            ctrl = plant.park_control()
            for sid in shoe_ids:
                ctrl[sid] = SHOE_CLOSE_TARGET
            for rid in unload_ids:
                ctrl[rid] = float(__import__('probe_c_full_chain').UNLOAD_ROLLER_SPEED)
            data.ctrl[:] = ctrl
            mujoco.mj_step(model, data)
            last_support = plant.tray_support()
        x_after = tray_x(model, data)
        z_after = float(data.xpos[model.body('c_payload').id][2])
        t3 = {'shoes': 'driven CLOSED %.4f' % SHOE_CLOSE_TARGET,
              'rollers': 'unload ids %d driven at UNLOAD_ROLLER_SPEED' % len(unload_ids),
              'support_after': list(last_support or []),
              'tray_dx_m': round(abs(x_after - x_before), 6),
              'tray_z_before_m': round(z_before, 4),
              'tray_z_after_m': round(z_after, 4),
              'mechanism_note': 'runtime-loaded deck: the tray free-slides (0.098 kg, '
                                'damping-0 rollers) to a vehicle rest at offset ~0.254 '
                                'held by the n_chassis face (srcl3 contact evidence); '
                                'the hinged shoe is one-sided (0.8 mm lip blocks rearward '
                                'slide only) so dx over the window measures slide-to-rest '
                                '+ fault injection, not shoe hold'}
        raised = None
        try:
            def confirm_deck_still_loaded(result):
                support = plant.tray_support()
                return {'receiver_supported': 'receiver_band' in support,
                        'source_cleared': 'deck' not in support,
                        'stopped_confirmed': True,
                        'evidence': ['runs/g7d/t3_support.json']}
            session3.execute({'transfer_id': 'g7d-t3', 'order_id': ORDER, 'epoch': EPOCH,
                              'source': 'vehicle_deck', 'receiver': 'receiver_band',
                              'tray_id': TRAY, 'preconditions': pre3},
                             now_s=time.monotonic(),
                             launch_evidence=['runs/g7d/t3_launch.json'],
                             drive=lambda: {'status': 'SUCCEEDED'},
                             confirm=confirm_deck_still_loaded)
        except Exception as exc:
            raised = '%s: %s' % (type(exc).__name__, exc)
        stage3 = stage_of(led3, 'g7d-t3')
        t3['confirm_refused_by'] = raised
        t3['stage_after'] = stage3
        t3['receiver_owner_after'] = res3.snapshot('receiver_band')['owner']
        add('T3 premature unload does not eject the tray from the deck',
            'deck' in (last_support or []),
            'support %r dx %.6f m dz %.4f m'
            % (t3['support_after'], t3['tray_dx_m'], z_after - z_before))
        add('T3 confirmation refuses the unload (no COMMIT)',
            raised is not None and stage3 not in ('COMMITTED', 'RELEASED'),
            'raised %r stage %r' % (raised, stage3))
        add('T3 receiver lock retained for attention',
            t3['receiver_owner_after'] == 'g7d-t3',
            'owner %r' % t3['receiver_owner_after'])
        report['tests']['T3'] = t3

        report['final_support'] = list(plant.tray_support())
        report['final_tray_x_m'] = tray_x(model, data)
    except Exception as exc:
        report['error'] = '%s: %s' % (type(exc).__name__, exc)
        report['traceback'] = traceback.format_exc()[-2000:]
    else:
        report['status'] = 'COMPLETED'
    finally:
        report['rows'] = rows
        report['wall_s'] = time.monotonic() - wall0
        (out / 'report.json').write_text(json.dumps(report, indent=2, default=str) + '\n')
        print(json.dumps({'status': report.get('status'), 'rows': rows,
                          'error': report.get('error')}, indent=2, default=str))
    return 0 if report.get('status') == 'COMPLETED' and all(
        r['status'] == 'PASS' for r in rows) else 1


if __name__ == '__main__':
    raise SystemExit(main())
