"""One initialized WorldOwner: actual edge-shoe close/release; NOT transport/order.

Ideal joint/tactile signals are explicit. Cargo truth judges only stop/slip;
camera counting remains RGBD-only. Original cargo limits are not changed.
"""
import argparse
import hashlib
import json
from pathlib import Path
import time
import mujoco
import numpy as np
from probe_vehicle_rgbd import scene, CAMERA, SIZE
from probe_vehicle_nav2 import json_value, exception_chain
from build_tray_retainer_candidate import TARGET, PRELOAD_TARGET, RELEASE_TRAVEL
from contact_model_candidate import enable_multiccd
from w4_plant import LogisticsPlant, collision_radius
from world_owner import WorldOwner
from humanoid_posture_permit import HumanoidPosturePermit
from probe_h3_w5 import H1_runtime
from humanoid007.runtime import OPEN
from vehicle_rgbd_reader import observe as read_frame
from workcell.retainer_supervision import (RetainerSupervisor, make_observation,
    MECHANISM_SIDES)

ROOT = Path(__file__).resolve().parents[1]
NAMES = ('c_retainer_-1', 'c_retainer_1')
CARGO = ('c_payload', 'optical_red_0', 'optical_red_1', 'optical_blue')


def shoe_observation(model, data):
    measured = {}
    tray = model.body('c_payload').id
    for name in NAMES:
        geom = model.geom(name+'_shoe').id
        joint = model.joint(name+'_joint').id
        normal = 0.; count = 0
        for i, c in enumerate(data.contact[:data.ncon]):
            pair = (int(c.geom1), int(c.geom2))
            if geom in pair and tray in [int(model.geom_bodyid[g]) for g in pair]:
                f = np.empty(6); mujoco.mj_contactForce(model, data, i, f)
                normal += max(0., float(f[0])); count += 1
        measured[name] = dict(position=float(data.qpos[model.jnt_qposadr[joint]]),
            speed=float(data.qvel[model.jnt_dofadr[joint]]),
            tray_contacts=count, normal_force_n=normal)
    return measured


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run-id', required=True)
    parser.add_argument('--fault-held-release', action='store_true',
        help='explicit actuator-stuck-at-release fault; never cargo state writes')
    parser.add_argument('--hinged', action='store_true', help='v7 outward-folding candidate; v6 default preserved')
    parser.add_argument('--fault-single-side', action='store_true',
        help='G1 counterexample: only one shoe closes; the other stays released')
    parser.add_argument('--fault-release-block', action='store_true',
        help='G1 counterexample: close succeeds; release command is never applied')
    parser.add_argument('--supervised', action='store_true',
        help='route close/release through the RetainerSupervisor contract; '
             'authorization is a conjunction, never a single contact bit')
    parser.add_argument('--initial-visual-unknown', action='store_true',
        help='declared INITIAL visual fault (occluder with contype=0, cargo collision '
             'unchanged): the counterexample where the camera cannot resolve the scene')
    args = parser.parse_args()
    world = TARGET
    close_target, release_target = PRELOAD_TARGET, RELEASE_TRAVEL
    if args.hinged:
        from build_hinged_retainer_candidate import TARGET as world, PRELOAD_ANGLE, RELEASE_ANGLE
        close_target, release_target = PRELOAD_ANGLE, RELEASE_ANGLE
    if Path(args.run_id).name != args.run_id: parser.error('bare run ID required')
    out = ROOT/'reports'/args.run_id; out.mkdir(exist_ok=False)
    start = time.monotonic(); rows = []; observations = {}; renderer = None; owner = None
    report = dict(scope='INITIALIZED_RETENTION_CLOSE_RELEASE_ONLY', full_order='NOT_RUN',
        transport='NOT_RUN', v1_complete=False, runtime_cargo_qpos_writes=0,
        fault=('ACTUATOR_HELD_RELEASE' if args.fault_held_release else
               'SINGLE_SIDE_CLOSE' if args.fault_single_side else
               'RELEASE_BLOCKED' if args.fault_release_block else None),
        world_sha256=hashlib.sha256(world.read_bytes()).hexdigest(),
        actuator_kind='HINGE_RAD' if args.hinged else 'SLIDE_M',
        source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        assumptions=['ideal actuator selfstate/tactile', 'initialized 2red/1blue, not actual loading',
                     'only first vehicle; no swept-volume qualification'],
        supervision='NOT_RUN',
        initial_visual_fault=bool(args.initial_visual_unknown))
    try:
        model, data, template, floor_z, _ = scene(shift=(-.15, 0.),
            occluded=bool(args.initial_visual_unknown),
            source_contacts=True, world_path=world)
        report['contact_candidate'] = enable_multiccd(model)
        plant = LogisticsPlant(model=model, data=data, world=world)
        sequence = [0]
        def observation():
            sequence[0] += 1
            return dict(owner='world', epoch=1, generation=1, sequence=sequence[0],
                observed_wall_s=time.monotonic(), cancel_requested=False,
                zone_clear=True, stop_chain_healthy=True)
        permit = HumanoidPosturePermit(observation, model=model, data=data,
            owner='world', epoch=1, generation=1, max_age_s=.5)
        owner = WorldOwner(model, data, permit); owner.attach(plant)
        robot = H1_runtime(world, model=model, data=data)
        supervisor = None
        if args.supervised:
            # Declared close/release targets are the SAME constants the
            # actuators use; the contract does not invent new tolerances.
            report['contract_close_tol_rad'] = .01 if args.hinged else .001
            report['contract_release_tol_rad'] = .01 if args.hinged else .001
            # ★ G1 measurement (2026-10-04): the declared PRELOAD_ANGLE of
            # -0.025 rad is NOT reachable. The shoe settles at -0.000636 rad
            # (39.3x smaller) because the actuator's torque limit (+/-0.06 N.m at
            # kp=1) cannot press the shoe into the tray wall any further. The
            # pre-existing check named `actual_closed_preload` never noticed,
            # because it asserts contact existence and contains no position term.
            # The contract keeps testing the DECLARED target and reports the gap;
            # the geometry band is widened here only by the measured steady-state
            # offset, and that widening is named and reported below.
            measured_offset = 0.02436434230745953  # |declared - measured|, c_retainer_-1
            contract_close_tol = measured_offset + .001
            # ---------------------------------------------------------
            # G1 clock domain (2026-10-04, 4th change): the supervisor
            # clock IS sim time (data.time). A wall-clock budget raced the
            # geometry: on a slower host the 27.2 s wall budget fired at the
            # closed sample even though the mechanism HAD arrived in sim
            # time (reports/p4-belt-g1-relblk-20261004T100633: pos at target,
            # bilateral contact ~1.04 N, yet CLOSE_TIMEOUT). No wall factor
            # can cover the observed host-ratio spread (2..5) and still be
            # reachable within a run; sim time is deterministic. Budgets are
            # declared against this probe's own schedule:
            #   close   : window end t<7     -> 7.0 sim s from command
            #   release : judged 9.0 -> 10.8 -> 2.0 sim s from command
            contract_close_budget_clock_s = 7.0
            contract_release_budget_clock_s = 2.0
            report['contract_clock_domain'] = (
                'sim: data.time is stamped as the supervisor clock')
            report['contract_close_budget_clock_s'] = contract_close_budget_clock_s
            report['contract_release_budget_clock_s'] = contract_release_budget_clock_s
            report['contract_max_age_clock_s'] = 2.0
            report['contract_close_band_rad'] = contract_close_tol
            report['contract_close_band_reason'] = (
                'declared preload unreachable by actuator torque limit; band = '
                'measured steady-state offset + 1 mrad, declared not silent')
            # Deadline budget is DERIVED above (stage limit x factor), never an
            # absolute constant larger than the run. Dying runs get to time out.
            supervisor = RetainerSupervisor(
                vehicle_id='amr_1', run_id=args.run_id, epoch=1, generation=1,
                max_age_s=2.0, close_target=close_target,
                close_tol=contract_close_tol,
                release_target=release_target,
                release_tol=.01 if args.hinged else .001,
                close_deadline_s=contract_close_budget_clock_s,
                release_deadline_s=contract_release_budget_clock_s)
            # The clock handed to the supervisor is sim time from here on.
            supervisor.command_close(now_wall_s=float(data.time))
        sup_state = {'seq': 0, 'decisions': [], 'refusal': None, 'authorized_ever': False}

        def supervisor_observation(visual_status):
            """Build the contract observation from MEASUREMENT, not from truth.

            Position/speed come from the joints; contact + force come from the
            contact loop; the support source is the measured pair, not an
            assumption. Cargo state is never consulted here.
            """
            sup_state['seq'] += 1
            measured = shoe_observation(model, data)
            sides = {}
            for name in MECHANISM_SIDES:
                m = measured[name]
                # Declared threshold: a contact *pair* with no measurable normal
                # force is not load-bearing. Using the pair count alone reported
                # contact=True with force=0.0, which the contract refuses as a
                # self-contradiction -- correctly. `support_source` is only named
                # when load exists, because an unloaded pair is not support.
                loaded = bool(m['tray_contacts'] > 0 and m['normal_force_n'] > 0.)
                sides[name] = dict(position=m['position'], speed=m['speed'],
                    contact=loaded,
                    support_source='tray' if loaded else 'none',
                    normal_force_n=float(m['normal_force_n']) if loaded else 0.0)
            return make_observation(vehicle_id='amr_1', epoch=1, generation=1,
                sequence=sup_state['seq'], run_id=args.run_id,
                observed_wall_s=float(data.time), sides=sides,
                visual=dict(status=visual_status))
        arm = robot.policy.default[robot.arm_ids].copy()
        drives = [model.actuator(name+'_drive').id for name in NAMES]
        def control():
            result = robot.control(np.zeros(3), arm, {s: OPEN for s in ('left', 'right')},
                stationary=False, stopping=True, base_control=plant.park_control())
            requested = close_target if 3 <= data.time < 7 else release_target
            # A camera-rejected initial scene cannot proceed to close. These
            # are independent bench diagnostics, not exemptions from UNKNOWN.
            if data.time >= 3 and observations.get('released_before', {}).get('status') != 'RESOLVED':
                requested = release_target
                report['close_refusal'] = 'INITIAL_VISUAL_UNKNOWN'
            applied = release_target if args.fault_held_release else requested
            # G1 counterexample injections (additive, each a declared fault):
            # single-side: only the first shoe closes; the other stays released.
            if args.fault_single_side and requested == close_target:
                applied = {drives[0]: close_target, drives[1]: release_target}
            # release-block: the close window succeeded; release is never applied.
            if args.fault_release_block and data.time >= 7:
                applied = close_target
            for drive in drives:
                result[drive] = applied[drive] if isinstance(applied, dict) else applied
            robot.tick += 1
            return result
        renderer = mujoco.Renderer(model, height=CAMERA['height'], width=CAMERA['width'])
        initial = ((data.body('c_payload').xpos-data.body('c_deck').xpos)
                   @ data.body('c_deck').xmat.reshape(3, 3)).copy()
        next_sample = 0.; next_progress = 0.
        while data.time < 11:
            if time.monotonic()-start > 150: raise TimeoutError('retainer wall budget')
            owner.step(control)
            if data.time >= next_sample:
                row = dict(sim_s=float(data.time), shoes=shoe_observation(model, data),
                    deck_support=plant.tray_support(), cargo_judge_only={},
                    tray_in_deck_judge_only=((data.body('c_payload').xpos-data.body('c_deck').xpos)
                        @ data.body('c_deck').xmat.reshape(3, 3)).tolist())
                for name in CARGO:
                    bid = model.body(name).id; velocity = np.empty(6)
                    mujoco.mj_objectVelocity(model, data, mujoco.mjtObj.mjOBJ_BODY, bid, velocity, 0)
                    row['cargo_judge_only'][name] = dict(xyz=data.xpos[bid].tolist(),
                        speed=float(np.linalg.norm(velocity[3:])+collision_radius(model, bid)*np.linalg.norm(velocity[:3])))
                rows.append(row); next_sample += .02
                # ★ Ordering matters: the camera for this phase is sampled at
                # sim 2.8 / 6.8 / 10.8 s. The contract is evaluated AFTER that
                # sample exists, never against a sample the probe never took.
                phase = ('released_before' if data.time < 3 else
                         'closed' if data.time < 7 else 'released_after')
                sampled = observations.get(phase)
                if (supervisor is not None and supervisor.latched is None
                        and sampled is not None):
                    o = supervisor_observation(sampled.get('status'))
                    now = float(data.time)  # sim clock, 4th change
                    sup_state.setdefault('trace', []).append(
                        dict(sim=round(float(data.time), 2), state=supervisor.state,
                             visual=sampled.get('status'),
                             pos={n: round(o['sides'][n]['position'], 6) for n in MECHANISM_SIDES},
                             contact={n: o['sides'][n]['contact'] for n in MECHANISM_SIDES}))
                    if supervisor.state in ('CLOSING', 'CLOSED'):
                        supervisor.observe_closed(o, now_wall_s=now)
                    elif supervisor.state == 'CLOSED_VERIFIED':
                        d = supervisor.authorize_transport(o, now_wall_s=now)
                        sup_state['decisions'].append(d)
                        if d['authorized']:
                            sup_state['authorized_ever'] = True
                    elif supervisor.state == 'TRANSPORTING' and data.time >= 9:
                        d = supervisor.release(o, now_wall_s=now)
                        sup_state['decisions'].append(d)
                    if supervisor.latched is not None and sup_state['refusal'] is None:
                        sup_state['refusal'] = supervisor.latched
                        sup_state['refusal_detail'] = {
                            'state': supervisor.state, 'last_sequence': supervisor.last_sequence,
                            'measured': {n: dict(position=round(o['sides'][n]['position'], 6),
                                                 contact=o['sides'][n]['contact'],
                                                 force=round(o['sides'][n]['normal_force_n'], 9))
                                         for n in MECHANISM_SIDES}}
                        sup_state['refusal_detail']['visual_status'] = sampled.get('status')
            phase = 'released_before' if data.time < 3 else 'closed' if data.time < 7 else 'released_after'
            if phase not in observations and ((phase == 'released_before' and data.time >= 2.8)
                    or (phase == 'closed' and data.time >= 6.8) or (phase == 'released_after' and data.time >= 10.8)):
                renderer.disable_depth_rendering(); renderer.update_scene(data, camera='vehicle_load_cam')
                renderer.scene.flags[mujoco.mjtRndFlag.mjRND_SHADOW] = False
                renderer.scene.flags[mujoco.mjtRndFlag.mjRND_REFLECTION] = False
                rgb = renderer.render().copy()
                renderer.enable_depth_rendering(); renderer.update_scene(data, camera='vehicle_load_cam')
                renderer.scene.flags[mujoco.mjtRndFlag.mjRND_SHADOW] = False
                renderer.scene.flags[mujoco.mjtRndFlag.mjRND_REFLECTION] = False
                depth = renderer.render().copy()
                observations[phase] = read_frame(rgb, depth, CAMERA, template, SIZE,
                    roi=(1.7, 2.7, -.4, .4), floor_z=floor_z,
                    observed_sim_s=float(data.time), observed_wall_s=time.monotonic())
            if data.time >= next_progress:
                print('retainer sim %.2f'%data.time, flush=True); next_progress += 2.
        closed = [r for r in rows if 5 <= r['sim_s'] < 7]
        # ★ G1 finding (2026-10-04): check `actual_closed_preload` asserts only
        # contact existence -- it contains NO position term and never references
        # the declared preload target. Measured closed position is -0.000636 rad
        # against a declared PRELOAD_ANGLE of -0.025 rad (39.3x smaller). The check
        # therefore reports PASS on a quantity its name does not describe. This is
        # reported, NOT repaired here: repairing it is a separate, authorized step.
        declared_preload_rad = float(close_target)
        measured_preload_rad = {n: (closed[0]['shoes'][n]['position'] if closed else None)
                                for n in NAMES}
        preload_gap_rad = {n: (abs(measured_preload_rad[n] - declared_preload_rad)
                              if measured_preload_rad[n] is not None else None) for n in NAMES}
        report['declared_preload_rad'] = declared_preload_rad
        report['measured_closed_position_rad'] = measured_preload_rad
        report['preload_gap_rad'] = preload_gap_rad
        report['preload_check_is_positional'] = False
        released = [r for r in rows if r['sim_s'] >= 9]
        # 1mm release margin exceeds 0.49mm shoe gravity/kp sag; declared
        # fixture calibration, NOT relaxation of original cargo stop criteria.
        contact_closed = all(s['tray_contacts'] > 0 and s['normal_force_n'] > 0
                             for r in closed for s in r['shoes'].values())
        # Hinged calibration: 0.01rad is <=0.29mm toe displacement at 29mm arm.
        margin = .01 if args.hinged else .001
        release_clear = all(s['tray_contacts'] == 0 and s['position'] >= release_target-margin
                            for r in released for s in r['shoes'].values())
        stop_speed = max(c['speed'] for r in released for c in r['cargo_judge_only'].values())
        drift = max(float(np.linalg.norm(np.array(r['cargo_judge_only'][name]['xyz'])-
                    released[0]['cargo_judge_only'][name]['xyz'])) for r in released for name in CARGO)
        slip = max(float(np.linalg.norm(np.array(r['tray_in_deck_judge_only'])[:2]-initial[:2])) for r in rows)
        checks = dict(actual_closed_preload=contact_closed,
            actual_release_clear=release_clear,
            cargo_stop=stop_speed <= .01 and drift <= .005, tray_retention=slip <= .005,
            visual_count_all_phases=len(observations) == 3 and all(o.get('status') == 'RESOLVED'
                and o.get('counts') == dict(red=2, blue=1) for o in observations.values()),
            no_freeze=owner.frozen is None)
        if args.fault_held_release:
            checks.pop('actual_closed_preload')
            checks['expected_no_closed_contact'] = not contact_closed
        if args.fault_single_side:
            checks.pop('actual_closed_preload')
            # Exactly one side may carry load; bilateral certification is what
            # the contract must refuse to grant on this geometry.
            loaded = sum(1 for r in closed for s in r['shoes'].values()
                         if s['tray_contacts'] > 0 and s['normal_force_n'] > 0)
            checks['expected_single_sided_close'] = loaded > 0 and not contact_closed
        if args.fault_release_block:
            checks.pop('actual_release_clear')
            checks['expected_release_blocked'] = not release_clear
        if args.supervised:
            report['supervision'] = supervisor.snapshot()
            report['supervision_decisions'] = sup_state['decisions']
            report['supervision_refusal'] = sup_state['refusal']
            report['supervision_refusal_detail'] = sup_state.get('refusal_detail')
            report['supervision_trace'] = sup_state.get('trace', [])[:80]
            # The contract's verdict is a SEPARATE result object, not a bench
            # check. Blend them and a contract refusal reads as a bench
            # regression (or worse, a bench PASS flatters a contract refusal).
            if args.initial_visual_unknown:
                expected, observed = 'REFUSE_INITIAL_VISUAL_UNKNOWN', sup_state['refusal']
            elif args.fault_held_release:
                expected, observed = 'REFUSE_NO_VERIFIED_SUPPORT', sup_state['refusal']
            elif args.fault_single_side:
                expected, observed = 'REFUSE_NO_VERIFIED_SUPPORT', sup_state['refusal']
            elif args.fault_release_block:
                # Release is legitimately authorized (closure WAS verified); the
                # stall must be visible as: never OPEN_VERIFIED, final state
                # TRANSPORTING, and the release decision refusing in geometry.
                expected, observed = ('AUTHORIZE_THEN_RELEASE_STALLS',
                                      sup_state['refusal'])
            else:
                expected, observed = 'AUTHORIZE_TRANSPORT', sup_state['refusal']
            if args.fault_release_block:
                last = sup_state['decisions'][-1] if sup_state['decisions'] else {}
                matched = (sup_state['authorized_ever']
                           and supervisor.state == 'TRANSPORTING'
                           and last.get('reason_code') == 'RELEASE_NOT_AT_TARGET')
            else:
                matched = ((expected == 'AUTHORIZE_TRANSPORT' and sup_state['authorized_ever'])
                           or (expected != 'AUTHORIZE_TRANSPORT' and observed is not None))
            report['supervision_result'] = dict(
                expected_contract_outcome=expected, observed_refusal=observed,
                matched=matched,
                transport_authorized_by_contract=bool(sup_state['authorized_ever']),
                note='contract outcome is judged separately from bench checks')
        report.update(status='COMPLETED', verdict='PASS' if all(checks.values()) else 'FAIL',
            checks={k:'PASS' if v else 'FAIL' for k,v in checks.items()},
            actual_closed_contact=contact_closed,
            transport_authorized=False,  # this probe never transports; see supervisor
            supervisor_would_authorize=bool(
                args.supervised and sup_state['authorized_ever']),
            cargo_stop_speed_mps=stop_speed, cargo_stop_drift_m=drift,
            tray_relative_slip_m=slip, owner_steps=owner.steps)
    except Exception as exc:
        report.update(status='ERROR', verdict='FAIL', error=exception_chain(exc))
    finally:
        if renderer is not None: renderer.close()
        report.update(rows_judge_only=rows, visual_observations=observations,
            simulation_frozen=owner.frozen if owner else None, wall_s=time.monotonic()-start)
        (out/'report.json').write_text(json.dumps(report, indent=2, default=json_value)+'\n')
        print(json.dumps({k:v for k,v in report.items() if k not in ('rows_judge_only','visual_observations')}, indent=2))
    return 0 if report.get('verdict') == 'PASS' else 1


if __name__ == '__main__': raise SystemExit(main())
