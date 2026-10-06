"""Initialized loaded v5 Nav2 candidate, not continuous loading/full order/V1.

Only WorldOwner advances physics. ROS sees scan/wheel odometry, never truth
pose. Independent truth is saved for arrival/stop judgement after execution.
"""
import argparse
import hashlib
import json
from pathlib import Path
import signal
import sys
import time
import traceback
import mujoco
import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT/'src'), str(ROOT/'experiments')]
from w4_plant import LogisticsPlant, collision_radius
from world_owner import WorldOwner
from humanoid_posture_permit import HumanoidPosturePermit
from probe_h3_w5 import H1_runtime
from humanoid007.runtime import OPEN
from joint_world_nav_io import JointWorldNavIO
from make_joint_nav_profile import posture_check, POSTURE
from probe_n_nav import Processes
from probe_vehicle_rgbd import scene, CAMERA, SIZE
from vehicle_rgbd_reader import observe as read_frame
from load_motion_gate import LoadMotionGate
from transport_posture_gate import TransportPostureGate
from state_publish_schedule import StatePublishSchedule
from deck_support_forensics import DeckSupportForensics
import probe_p3_vision as pv


def json_value(value):
    if isinstance(value, np.ndarray): return value.tolist()
    if isinstance(value, np.generic): return value.item()
    raise TypeError('unsupported report value: '+type(value).__name__)


def exception_chain(error):
    """Keep the original control-production cause, not only final-writer hold."""
    result=[];seen=set()
    while error is not None and id(error) not in seen:
        seen.add(id(error));result.append(type(error).__name__+': '+str(error))
        error=error.__cause__ or error.__context__
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run-id', required=True)
    parser.add_argument('--duration', type=float, default=60.)
    parser.add_argument('--parking-handoff', action='store_true')
    parser.add_argument('--centered-load',action='store_true')
    parser.add_argument('--receiver-leg',action='store_true',help='initialized long receiver-approach DIAGNOSTIC, not actual loading')
    parser.add_argument('--source-load-setup',action='store_true',help='initialize declared 2.04m tray centre and deployed source retention, diagnostic only')
    parser.add_argument('--retained-motion',action='store_true',help='independent lower-acceleration loaded Nav2 control candidate')
    parser.add_argument('--multi-ccd',action='store_true',help='init-only multi-contact candidate; independent physical revalidation required')
    parser.add_argument('--linear-speed',type=float,help='single-variable retained speed diagnostic [0.05,0.15]')
    parser.add_argument('--contact-time-constant',type=float,help='init-only deck contact solver candidate, >=2 timesteps and <=0.02s')
    parser.add_argument('--candidate',choices=['v5','v7'],default='v5',help='tested candidate selection; v7 uses the independently minted v7 identity')
    parser.add_argument('--turn-conservative',action='store_true',help='v7-only angular-axis conservative candidate; linear speed unchanged')
    parser.add_argument('--source-load',action='store_true',help='v7-only: tray starts on the source band (shoes OPEN); run the real source->deck transfer before Nav2')
    args = parser.parse_args()
    if args.candidate == 'v7':
        if args.centered_load or args.source_load_setup or args.multi_ccd or args.contact_time_constant is not None:
            parser.error('--candidate v7 commissions the base loaded transport identity only; centered/source-setup/multi-ccd/contact candidates are v5 lineage')
    elif args.retained_motion and not args.centered_load:
        parser.error('--retained-motion requires --centered-load')
    if args.turn_conservative and args.candidate != 'v7':
        parser.error('--turn-conservative is v7-only')
    if args.turn_conservative and args.retained_motion:
        parser.error('--turn-conservative and --retained-motion are mutually exclusive')
    if args.source_load:
        if args.candidate != 'v7':
            parser.error('--source-load is v7-only')
        if args.retained_motion or args.turn_conservative or args.centered_load \
                or args.source_load_setup or args.multi_ccd \
                or args.contact_time_constant is not None:
            parser.error('--source-load combination not supported')
    if args.candidate == 'v7':
        import make_joint_nav_profile_v7 as _nav_identity
        POSTURE = _nav_identity.POSTURE
        posture_check = _nav_identity.posture_check
    if args.linear_speed is not None:
        from joint_nav_worker import speed_overrides
        if not args.retained_motion:parser.error('--linear-speed requires --retained-motion')
        speed_overrides('controller_server',args.linear_speed)
    if Path(args.run_id).name != args.run_id or not np.isfinite(args.duration) or not 10<=args.duration<=90:
        parser.error('bare run ID and sim duration [10,90] required')
    out = ROOT/'reports'/args.run_id; out.mkdir(parents=True, exist_ok=False)
    cfg_name = 'config/joint_world_v7.yaml' if args.candidate == 'v7' else 'config/joint_world_v5.yaml'
    profile_name = 'config/joint_world_v7.profile.json' if args.candidate == 'v7' else 'config/joint_world_v5.profile.json'
    cfg = yaml.safe_load((ROOT/cfg_name).read_text())
    profile = json.loads((ROOT/profile_name).read_text())
    world = ROOT/cfg['world']; owned = Processes(out); io = None; owner = None
    truth = []; stopping_at = None; view = None; frames=[]; support_forensics=None
    started = time.monotonic(); timing={};report = dict(scope='INITIALIZED_LOAD_SHARED_WORLD_NAV2', status='ERROR',
        world_sha256=hashlib.sha256(world.read_bytes()).hexdigest(),
        source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        linear_speed_candidate_mps=args.linear_speed,
        runtime_qpos_writes=0, loaded_navigation='NOT_RUN', full_order='NOT_RUN', v1_complete=False,
        assumptions=['initialized cargo, NOT continuous H supply/arm loading',
                     'vehicle RGBD and ideal deck touch; static map prior NOT SLAM',
                     'shadows/reflections/multisampling off in camera, NOT physics',
                     'no 3D swept-volume guard; bounded short straight commissioning only'])
    if args.candidate == 'v7':
        report['assumptions'].extend(['v7: shoes closed at scene start by declared initialization, NOT runtime loading evidence',
            'v7: deck parked; slide stroke excluded from footprint and posture-gated',
            'v7: rotate-to-heading disabled in nav2_joint_world_v7.yaml (corridor cannot contain in-place rotation)',
            'v7: stock BT recovery may still request Spin; declared risk, run evidence will decide'])
        if args.retained_motion:
            report['assumptions'].append('v7 retained: conservative control candidate on base scene; acceptance gates unchanged')
        if args.turn_conservative:
            report['assumptions'].append('v7 turn-conservative: angular-axis-only candidate; linear speed unchanged at base values')
        if args.source_load:
            report['assumptions'].append('v7 source-load: tray initialized on the source band; '
                'real source->deck transfer runs before Nav2; retainer starts OPEN')
    def timed(name,began):
        elapsed=time.monotonic()-began
        timing[name]=max(timing.get(name,0.),elapsed)
        if elapsed>.5:print('SLOW %s %.3fs at sim %.3f'%(name,elapsed,float(data.time)),flush=True)
    def interrupted(signum, frame): raise KeyboardInterrupt('owned simulation interrupted')
    signal.signal(signal.SIGTERM, interrupted)
    try:
        if report['world_sha256'] != profile['world_sha256']: raise ValueError('profile world hash mismatch')
        pv.W,pv.H=CAMERA['width'],CAMERA['height']
        model,data,template,floor_z,_=scene(shift=(-.15,0.) if (args.source_load_setup or args.source_load) else (0.,0.),
            tray_yaw=0.,source_contacts=args.source_load_setup or args.source_load,
            world_path=ROOT/'assets/world_p5_candidate_v7_hinged_retainer.xml' if args.candidate=='v7' else None,
            tray_pos_x=(4.55 if args.source_load else None))
        if args.multi_ccd:
            from contact_model_candidate import enable_multiccd
            report['contact_candidate']=enable_multiccd(model)
        if args.contact_time_constant is not None:
            from contact_model_candidate import deck_contact_time_constant
            report['deck_contact_candidate']=deck_contact_time_constant(model,args.contact_time_constant)
        loaded_name='joint_world_v7_loaded.profile.json' if args.candidate == 'v7' else ('joint_world_v5_centered.profile.json' if args.centered_load else 'joint_world_v5_loaded.profile.json')
        loaded_profile=json.loads((ROOT/'config'/loaded_name).read_text())
        report['loaded_profile']=loaded_name
        report['loaded_profile_sha256']=hashlib.sha256((ROOT/'config'/loaded_name).read_bytes()).hexdigest()
        movement_gate=LoadMotionGate(loaded_profile['observation_contract'])
        transport_gate=TransportPostureGate()
        latest={}
        view=mujoco.Renderer(model,height=pv.H,width=pv.W)
        plant = LogisticsPlant(model=model, data=data, world=world)
        support_forensics=DeckSupportForensics(model)
        if args.source_load_setup:
            # Independent initial scene, not a runtime restore/teleport. These
            # declarations mirror the measured source transfer's retention
            # command; they must never be called actual loading evidence.
            plant.deploy_pusher(lift=.001,slide=.01324)
            for name,value in (('c_pusher_lift_joint',.001),('c_pusher_slide_joint',.01324)):
                data.qpos[model.jnt_qposadr[model.joint(name).id]]=value
            mujoco.mj_forward(model,data)
            report['declared_initial_load_setup']=dict(tray_in_base_x_m=2.04,
                tray_yaw_rad=0.,pusher_lift_m=.001,pusher_slide_m=.01324,initialization_only=True,
                actual_source_skew_replayed=False)
        if args.candidate == 'v7':
            # Close command enters the CONTROL ASSEMBLY: WorldOwner.commit
            # overwrites the whole ctrl vector every step, so a one-time ctrl
            # write is dead on arrival (measured in p4-nav2-v7-01: shoes held
            # 1.570796 OPEN for the whole run). CLOSE_TARGET is the SAME
            # declared G1 constant (build_hinged_retainer_candidate
            # .PRELOAD_ANGLE = -0.025 rad), applied every step in compose().
            CLOSE_TARGET = -.025
            # Working-region band (revision 4): G1 settle -0.000636 and
            # run-02 settle -0.033530 are both INSIDE; upper bound is 31x
            # below OPEN 1.5708, so any real opening is out of band.
            RETAINER_BAND = (-.08, .05)
            report['retainer_close_command']=dict(target_rad=CLOSE_TARGET,
                source='build_hinged_retainer_candidate.PRELOAD_ANGLE (G1)',
                applied='every control step via compose()')
        if args.source_load:
            # G7-b: the REAL source->deck transfer, on the owned plant, BEFORE
            # any Nav2 or retainer command. plant.transfer() drives the 32
            # roller actuators itself while no WorldOwner exists yet, so the
            # per-step control-assembly contract is not violated. Shoes stay at
            # their OPEN keyframe through the transfer; the two-phase close
            # confirmation in the main loop only starts after this succeeds.
            # 0.8 s settle: the tray falls ~4 cm from the declared crown+.04
            # pose and BOUNCES — at 0.2 s it is measurably airborne
            # (supported_by []), so transfer's own entry check refuses without
            # driving anything (measured in p4-nav2-v7-07-srcl). CLOSED:
            # x=4.3436 sat on the w5_h085_station supply pad BEFORE the roller
            # start (4.414) — no roller under the tray there regardless of
            # ctrl; x=4.55 is ON the rollers and support is stable. Settle
            # under the humanoid STANCE control (keeps the humanoid upright,
            # so no BODY_FALL at the first main-loop permit check), retainer
            # left OPEN until the post-transfer close confirmation.
            robot = H1_runtime(world, model=model, data=data)
            arm_stance = robot.policy.default[robot.arm_ids].copy()
            for _ in range(400):
                data.ctrl[:] = robot.control(np.zeros(3), arm_stance,
                    {s: OPEN for s in ('left', 'right')},
                    stationary=True, stopping=True, base_control=plant.park_control())
                mujoco.mj_step(model, data)
            pre_support = plant.tray_support()
            report['source_transfer_pre_support'] = list(pre_support)
            if 'source_band' not in pre_support:
                raise RuntimeError('SOURCE_TRANSFER_PRE_SUPPORT_MISSING: %r' % list(pre_support))
            # Self-driven transfer loop: plant.transfer() monopolises ctrl
            # for its whole timeout, which drops the humanoid stance — the
            # humanoid then collapses and the close-confirmation window runs
            # against a frozen owner (measured p4-nav2-v7-07-srcl). Rollers
            # are driven ON TOP of the stance control; 1.0 rad/s: at 5.0 the
            # tray left the band mid-leg and fell to the ground (measured).
            roll_ids = [i for i in range(model.nu)
                        if (mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, i) or ''
                            ).startswith(('c_fixed_roller', 'c_deck_roller'))]
            xfer_deadline = float(data.time) + 150.
            while plant.tray_support() != ['deck']:
                if float(data.time) > xfer_deadline:
                    raise RuntimeError('SOURCE_TRANSFER_TIMEOUT: support %r at sim %.2f'
                                       % (list(plant.tray_support()), float(data.time)))
                data.ctrl[:] = robot.control(np.zeros(3), arm_stance,
                    {s: OPEN for s in ('left', 'right')},
                    stationary=True, stopping=True, base_control=plant.park_control())
                for rid in roll_ids:
                    data.ctrl[rid] = 1.0
                mujoco.mj_step(model, data)
            report['source_transfer_support'] = ['deck']
            report['source_transfer_end_sim_s'] = float(data.time)
            # srcl3 verdict: the deck is level but the tray (0.098 kg on free
            # rollers, idler damping 0) keeps residual velocity and slides
            # rearward into the closed shoe face (catch at tray-centre offset
            # ~1.94). The old 300-step OPEN-shoes settle gave the tray a 0.6 s
            # head start; the close confirmed 2.54 s after landing and the wall
            # had already slipped under the descending shoe (offset 2.19 ->
            # 0.254: retention 0.934 m FAIL, unload TIMEOUT). Close now starts
            # the step after landing; the pre-engagement slide is measured deck
            # physics, reported in g7c_shoe_engagement.
            landing_offset_c = float(data.xpos[model.body('c_payload').id][0]
                                     - data.xpos[model.body('n_base_link').id][0])
            report['g7c_landing_offset_m'] = round(landing_offset_c, 4)
            if not (args.source_load and args.receiver_leg):
                # transfer inertia decays before the close confirmation
                for _ in range(300):
                    data.ctrl[:] = robot.control(np.zeros(3), arm_stance,
                        {s: OPEN for s in ('left', 'right')},
                        stationary=True, stopping=True, base_control=plant.park_control())
                    mujoco.mj_step(model, data)
        if args.source_load and args.receiver_leg:
            # G7-c: deliver to the receiver — precise push to the dock pose,
            # open the shoes, unload onto the receiver band, confirm, record.
            # Gate order: CLOSE the retainer ON THE TRAY before any transport.
            # srcl2 measured the tray free on the deck rollers through the whole
            # nudge (offset +2.19 -> +0.25 onto the rear pusher) because this
            # branch ran before the pre-Nav2 close confirmation. Same two-phase
            # criteria as that ring: band, 1.0 s hold, 5e-3 drift, relative budget.
            close_deadline_c = float(data.time) + 12.0
            hold_s_c = 1.0
            hold_drift_max_c = 5e-3
            def qret_now_c():
                return [float(data.qpos[model.jnt_qposadr[model.joint(n + '_joint').id]])
                        for n in ('c_retainer_-1', 'c_retainer_1')]
            while True:
                ctrl_close = robot.control(np.zeros(3), arm_stance,
                    {s: OPEN for s in ('left', 'right')},
                    stationary=True, stopping=True, base_control=plant.park_control())
                for drv in ('c_retainer_-1_drive', 'c_retainer_1_drive'):
                    ctrl_close[model.actuator(drv).id] = CLOSE_TARGET
                data.ctrl[:] = ctrl_close
                mujoco.mj_step(model, data)
                if float(data.time) > close_deadline_c:
                    raise RuntimeError('G7C_PRETRANSPORT_CLOSE_NOT_CONFIRMED qpos=%r band=%r'
                                       % (qret_now_c(), RETAINER_BAND))
                qret_c = qret_now_c()
                if not all(RETAINER_BAND[0] <= q <= RETAINER_BAND[1] for q in qret_c):
                    continue
                hold_begin_c = float(data.time)
                q_hold_c = list(qret_c)
                settled_c = True
                while float(data.time) - hold_begin_c < hold_s_c:
                    ctrl_close = robot.control(np.zeros(3), arm_stance,
                        {s: OPEN for s in ('left', 'right')},
                        stationary=True, stopping=True, base_control=plant.park_control())
                    for drv in ('c_retainer_-1_drive', 'c_retainer_1_drive'):
                        ctrl_close[model.actuator(drv).id] = CLOSE_TARGET
                    data.ctrl[:] = ctrl_close
                    mujoco.mj_step(model, data)
                    if float(data.time) > close_deadline_c:
                        raise RuntimeError('G7C_PRETRANSPORT_CLOSE_NOT_CONFIRMED qpos=%r band=%r'
                                           % (qret_now_c(), RETAINER_BAND))
                    qret_c = qret_now_c()
                    if not all(RETAINER_BAND[0] <= q <= RETAINER_BAND[1] for q in qret_c):
                        settled_c = False
                        break
                    if any(abs(q - q0) > hold_drift_max_c for q, q0 in zip(qret_c, q_hold_c)):
                        settled_c = False
                        break
                if settled_c:
                    break
            report['g7c_pretransport_close'] = dict(
                confirmed_sim_s=float(data.time), settle_qpos=qret_now_c(),
                band=list(RETAINER_BAND), budget_sim_s=close_deadline_c,
                hold_s=hold_s_c, hold_drift_max_rad=hold_drift_max_c,
                tray_offset_at_close_m=round(float(
                    data.xpos[model.body('c_payload').id][0]
                    - data.xpos[model.body('n_base_link').id][0]), 4))
            # ENGAGEMENT WAIT: angle-confirmed CLOSED is not engagement
            # (CLOSED != CLOSED_VERIFIED, the retainer_supervision rule). The
            # tray slides rearward into the closed shoe's -x face; the catch
            # point is predicted from live model geometry (shoe face minus tray
            # front-wall reach), never hardcoded. Escape past the window means
            # the wall slipped under the descending shoe -> fail closed.
            def _geom_face(n):
                _g = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, n)
                return float(data.geom_xpos[_g][0]), float(model.geom_size[_g][0])
            _shoe_face_x = min(x - s for x, s in (
                _geom_face(n + '_shoe') for n in ('c_retainer_-1', 'c_retainer_1')))
            _wg, _ws = _geom_face('c_tray_wall_xp')
            _wall_reach = _wg + _ws - float(data.xpos[model.body('c_payload').id][0])
            _catch_offset = (_shoe_face_x - _wall_reach
                             - float(data.xpos[model.body('n_base_link').id][0]))
            # +-30 mm (srcl4): two valid rest modes measured - face-block
            # ~1.9412 and lip-press rest ~1.95-1.96 (landing was 9 mm short of
            # the face; retainer_1 rested at -0.0015 rad = wall-top contact).
            # The window discriminates engagement from escape (0.254, three
            # decades away); it is not contact-pose metrology.
            _ENGAGE_WIN = 0.030
            _engage_deadline = float(data.time) + 20.0
            _stable_since = None
            _off = float('nan')
            while True:
                _ctrl_e = robot.control(np.zeros(3), arm_stance,
                    {s: OPEN for s in ('left', 'right')},
                    stationary=True, stopping=True, base_control=plant.park_control())
                for drv in ('c_retainer_-1_drive', 'c_retainer_1_drive'):
                    _ctrl_e[model.actuator(drv).id] = CLOSE_TARGET
                data.ctrl[:] = _ctrl_e
                mujoco.mj_step(model, data)
                _off = float(data.xpos[model.body('c_payload').id][0]
                             - data.xpos[model.body('n_base_link').id][0])
                if abs(_off - _catch_offset) <= _ENGAGE_WIN:
                    if _stable_since is None:
                        _stable_since = float(data.time)
                    elif float(data.time) - _stable_since >= 0.5:
                        break
                else:
                    _stable_since = None
                if float(data.time) > _engage_deadline:
                    raise RuntimeError(
                        'G7C_SHOE_ENGAGEMENT_FAILED: offset=%.4f catch=%.4f win=%.3f'
                        % (_off, _catch_offset, _ENGAGE_WIN))
            report['g7c_shoe_engagement'] = dict(
                predicted_catch_offset_m=round(_catch_offset, 4), window_m=_ENGAGE_WIN,
                engaged_offset_m=round(_off, 4), confirmed_sim_s=float(data.time),
                landing_offset_m=report['g7c_landing_offset_m'],
                offset_at_close_m=report['g7c_pretransport_close']['tray_offset_at_close_m'],
                pre_catch_slide_m=round(_off - report['g7c_landing_offset_m'], 4),
                basis='shoe -x face minus tray front-wall reach, live model geometry')
            signs_g7 = plant._wheel_signs()
            dock_target = float(plant.stations['station_b']['dock_x_m'])
            g7_wall = time.monotonic()
            # G7-c instrumentation: 0.5 sim-s state rows to a JSONL sidecar plus
            # a 5 sim-s no-progress fail-fast carrying contact evidence. run-07c
            # hit a blind 300 s wall timeout with zero evidence; the stall
            # mechanism was invisible. Progress stays measured (chassis
            # displacement over a sim window), never assumed.
            nudge_fh = (out / 'g7c_nudge_log.jsonl').open('w')
            def _nudge_our_contacts():
                rows = []
                for i in range(data.ncon):
                    c = data.contact[i]
                    b1 = model.body(int(model.geom_bodyid[c.geom1])).name
                    b2 = model.body(int(model.geom_bodyid[c.geom2])).name
                    if not (b1.startswith(('n_', 'c_', 'optical_'))
                            or b2.startswith(('n_', 'c_', 'optical_'))):
                        continue
                    g1 = model.geom(int(c.geom1)).name or ''
                    g2 = model.geom(int(c.geom2)).name or ''
                    if 'ground' in g1 or 'ground' in g2:
                        continue
                    f = np.zeros(6); mujoco.mj_contactForce(model, data, i, f)
                    rows.append('%s|%s~%s|%s d=%.4f f=%.2f' % (b1, g1, b2, g2,
                        float(c.dist), float(np.linalg.norm(f[:3]))))
                return rows
            nudge_last = [float(data.time), plant.chassis_x()]
            def _nudge_dial(tag):
                dd = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, 'c_deck_drive')
                dsd = None
                if dd >= 0 and int(model.actuator_trnid[dd][0]) >= 0:
                    dsd = int(model.jnt_qposadr[int(model.actuator_trnid[dd][0])])
                wheel = {s: [float(data.ctrl[a]),
                             float(data.qvel[int(model.jnt_dofadr[model.joint(
                                 'n_wheel_%s_joint' % s).id])])]
                         for s, a in plant.wheel_actuators.items()}
                state = dict(tag=tag, sim_s=round(float(data.time), 2),
                             chassis_x=round(plant.chassis_x(), 4),
                             chassis_y=round(float(data.qpos[int(model.jnt_qposadr[
                                 model.joint('n_slide_y').id])]), 4),
                             yaw=round(float(data.qpos[int(model.jnt_qposadr[
                                 model.joint('n_yaw').id])]), 4),
                             tray_x=round(float(data.xpos[model.body('c_payload').id][0]), 4),
                             deck_drive_ctrl=(round(float(data.ctrl[dd]), 4) if dd >= 0 else None),
                             deck_slide_qpos=(round(float(data.qpos[dsd]), 4) if dsd is not None else None),
                             wheels={k: [round(v[0], 4), round(v[1], 4)] for k, v in wheel.items()},
                             ret_qpos=[round(q, 4) for q in qret_now_c()],
                             contacts=_nudge_our_contacts())
                nudge_fh.write(json.dumps(state) + '\n'); nudge_fh.flush()
                return state
            # srcl2 measured the approach as CONSTANT 0.049 m/s from x=4.66 to
            # 9.58 with zero external contacts: the earlier x=7.9637 'stall' was
            # the 300 s WALL budget firing on a loaded machine (3.55 m of real
            # travel), not physics. The budget is sim-time now; wall stays only
            # as a backstop. Transit 0.15 m/s (the retained-speed family the
            # Nav2 leg qualified), final 0.05 m/s nudge inside 0.5 m.
            nudge_deadline = float(data.time) + 180.0
            while abs(plant.chassis_x() - dock_target) > 0.02:
                if float(data.time) > nudge_deadline:
                    raise RuntimeError('G7C_DOCK_NUDGE_SIM_TIMEOUT: x=%.4f target=%.4f'
                                       % (plant.chassis_x(), dock_target))
                if time.monotonic() - g7_wall > 600:
                    raise RuntimeError('G7C_DOCK_NUDGE_TIMEOUT: x=%.4f target=%.4f'
                                       % (plant.chassis_x(), dock_target))
                err = dock_target - plant.chassis_x()
                if abs(err) > 0.5:
                    cmd = np.sign(err) * min(0.15, abs(err) / .4 + .01)
                else:
                    cmd = np.sign(err) * min(0.05, abs(err) / .4 + .005)
                base_drive = plant.park_control()
                for side, act in plant.wheel_actuators.items():
                    base_drive[act] = plant._wheel_rate_torque(
                        side, (cmd / 0.04) * signs_g7[side])
                ctrl_nudge = robot.control(np.zeros(3), arm_stance,
                    {s: OPEN for s in ('left', 'right')},
                    stationary=False, stopping=True, base_control=base_drive)
                # The hold law disarms every actuator it does not own (the
                # pusher lesson, w4_plant._hold docstring): park_control's
                # home-hold drove the retainer back to OPEN mid-transport
                # (srcl5 dial: ret=[1.5708,1.5708] from nudge row 1; the tray
                # escaped to the chassis face, cargo ended under the right
                # wheel, the deck nose jammed the receiver taper at 30 N).
                # The close command is re-asserted EVERY step, like compose().
                for drv in ('c_retainer_-1_drive', 'c_retainer_1_drive'):
                    ctrl_nudge[model.actuator(drv).id] = CLOSE_TARGET
                data.ctrl[:] = ctrl_nudge
                mujoco.mj_step(model, data)
                if float(data.time) - nudge_last[0] >= 5.0:
                    moved = abs(plant.chassis_x() - nudge_last[1])
                    if moved < 0.002:
                        st = _nudge_dial('stall')
                        nudge_fh.close()
                        raise RuntimeError('G7C_NUDGE_STALLED x=%.4f target=%.4f moved=%.5f state=%s'
                                           % (plant.chassis_x(), dock_target, moved, json.dumps(st)))
                    _nudge_dial('row')
                    nudge_last = [float(data.time), plant.chassis_x()]
            nudge_fh.close()
            _nrows = [json.loads(l) for l in open(out / 'g7c_nudge_log.jsonl')]
            _offs = [r['tray_x'] - r['chassis_x'] for r in _nrows]
            _catch = report['g7c_shoe_engagement']['predicted_catch_offset_m']
            report['g7c_nudge_retention'] = dict(
                rows=len(_nrows), offset_first_m=round(_offs[0], 4),
                baseline_m='catch offset (geometry-derived post-engagement rest; '
                           'GATE reference -- restored 2026-10-07, was first-row/15 mm)',
                offset_max_drift_m=round(max(abs(o - _catch) for o in _offs), 4),
                offset_drift_gate_m=0.005,
                diagnostic_drift_from_first_row_m=round(max(abs(o - _offs[0]) for o in _offs), 4),
                diagnostic_first_row_budget_m=0.015,
                shoe_escape_guard_m=round(min(o - _catch for o in _offs), 4),
                shoe_escape_limit_m=-0.005,
                retention_note='GATE: max |offset - catch| <= 5 mm (original transport-'
                               'qualification budget, restored 2026-10-07 after external '
                               'review; the 15 mm first-row number is DIAGNOSTIC ONLY and '
                               'gates nothing); PLUS escape guard: every row must satisfy '
                               'offset >= catch - 5 mm - past the face the shoe no longer '
                               'holds (srcl3 escape signature was 0.254)',
                shoes='CLOSED+ENGAGED (angle + position evidence)')
            outcome_stop = plant.stop()
            for _ in range(150):
                data.ctrl[:] = robot.control(np.zeros(3), arm_stance,
                    {s: OPEN for s in ('left', 'right')},
                    stationary=True, stopping=True, base_control=plant.park_control())
                mujoco.mj_step(model, data)
            report['g7c_dock'] = {'chassis_x_m': float(plant.chassis_x()),
                                  'target_m': dock_target,
                                  'stopped': bool(outcome_stop.get('stopped_confirmed'))}
            for _ in range(400):
                ctrl_open = robot.control(np.zeros(3), arm_stance,
                    {s: OPEN for s in ('left', 'right')},
                    stationary=True, stopping=True, base_control=plant.park_control())
                for drv in ('c_retainer_-1_drive', 'c_retainer_1_drive'):
                    ctrl_open[model.actuator(drv).id] = 1.5708
                data.ctrl[:] = ctrl_open
                mujoco.mj_step(model, data)
            qret_open = [float(data.qpos[model.jnt_qposadr[model.joint(n + '_joint').id]])
                         for n in ('c_retainer_-1', 'c_retainer_1')]
            report['g7c_shoes_open_qpos'] = qret_open
            if not all(q > 1.0 for q in qret_open):
                raise RuntimeError('G7C_SHOE_OPEN_FAILED: %r' % qret_open)
            unload_ids = [i for i in range(model.nu)
                          if (mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, i) or ''
                              ).startswith(('c_deck_roller', 'c_recv_roller'))]
            unload_speed = float(__import__('probe_c_full_chain').UNLOAD_ROLLER_SPEED)
            xfer_deadline = float(data.time) + 300.
            while plant.tray_support() != ['receiver_band']:
                if float(data.time) > xfer_deadline:
                    raise RuntimeError('G7C_UNLOAD_TIMEOUT: support %r at sim %.2f'
                                       % (list(plant.tray_support()), float(data.time)))
                data.ctrl[:] = robot.control(np.zeros(3), arm_stance,
                    {s: OPEN for s in ('left', 'right')},
                    stationary=True, stopping=True, base_control=plant.park_control())
                for rid in unload_ids:
                    data.ctrl[rid] = unload_speed
                mujoco.mj_step(model, data)
            report['g7c_unload_support'] = ['receiver_band']
            report['g7c_tray_end_x_m'] = float(data.xpos[model.body('c_payload').id][0])
            from workcell.resources import ResourceTable as _RT
            from workcell.transfer import TransferLedger as _TL
            from workcell.physical_transfer_session import PhysicalTransferSession as _S
            g7_res = _RT(epoch=EPOCH, resources=['source_seat', 'vehicle_deck', 'receiver_band'])
            g7_led = _TL(epoch=EPOCH)
            g7_tx = []
            sess = _S(ledger=g7_led, resources=g7_res, journal=lambda ev: g7_tx.append(ev))
            now0 = time.monotonic()
            for rid, ev in (('source_seat', 'runs/g7c/init_source.json'),
                            ('vehicle_deck', 'runs/g7c/init_deck.json'),
                            ('receiver_band', 'runs/g7c/init_receiver.json')):
                g7_res.initialise_occupied(rid, owner='world', ttl_s=7200., now_s=now0,
                                           evidence=[ev])
            for rid in ('source_seat', 'vehicle_deck', 'receiver_band'):
                g7_res.release(rid, owner='world',
                               generation=g7_res.snapshot(rid)['generation'],
                               epoch=EPOCH, now_s=time.monotonic())
            def _drive():
                return {'status': 'SUCCEEDED'}
            def _confirm(result):
                sup = plant.tray_support()
                return {'receiver_supported': 'receiver_band' in sup,
                        'source_cleared': True, 'stopped_confirmed': True,
                        'evidence': ['runs/g7c/post_unload_support.json']}
            _pre = {'source_capacity_free': True, 'receiver_capacity_free': True,
                    'height_within_tolerance': True, 'lateral_within_tolerance': True,
                    'yaw_within_tolerance': True, 'clearance_ok': True,
                    'amr_at_rest': True, 'source_has_tray': True,
                    'receiver_empty': True, 'arms_retracted': True,
                    'stop_chain_healthy': True, 'evidence_age_s': 0.0,
                    'max_evidence_age_s': 300.0, 'ttl_s': 3600.0}
            sess.execute({'transfer_id': 'g7c-load-001', 'order_id': ORDER, 'epoch': EPOCH,
                          'source': 'source_seat', 'receiver': 'vehicle_deck',
                          'tray_id': TRAY, 'preconditions': dict(_pre)},
                         now_s=time.monotonic(),
                         launch_evidence=['runs/g7c/load_done.json'],
                         drive=_drive, confirm=_confirm)
            sess.execute({'transfer_id': 'g7c-unload-001', 'order_id': ORDER, 'epoch': EPOCH,
                          'source': 'vehicle_deck', 'receiver': 'receiver_band',
                          'tray_id': TRAY, 'preconditions': dict(_pre)},
                         now_s=time.monotonic(),
                         launch_evidence=['runs/g7c/unload_done.json'],
                         drive=_drive, confirm=_confirm)
            report['g7c_ledger_events'] = len(g7_tx)
            report['g7c_ledger_summary'] = g7_led.summary()
        elif args.receiver_leg:
            profile['goal']=[float(plant.stations['station_b']['approach_x_m']),0.,0.]
        if args.source_load and args.receiver_leg:
            # The dock approach is the G7-c nudge itself. The post-unload ROS
            # leg is a NULL leg at the reached dock (goal = current pose); the
            # loaded Nav2 transport qualification remains G7-b / runs 03..06.
            profile['goal'] = [float(plant.chassis_x()), 0., 0.]
        report['declared_goal']=profile['goal']
        sequence = [0]
        def observe():
            sequence[0] += 1
            return dict(owner='world', epoch=1, generation=1, sequence=sequence[0],
                observed_wall_s=time.monotonic(), cancel_requested=False,
                zone_clear=True, stop_chain_healthy=True)
        permit = HumanoidPosturePermit(observe, model=model, data=data,
            owner='world', epoch=1, generation=1, max_age_s=.5)
        owner = WorldOwner(model, data, permit); owner.attach(plant)
        authority_token = None
        if args.parking_handoff:
            import vehicle_heading
            from wheel_authority import grant_navigation
            vehicle_heading.install(plant)
            authority_token = grant_navigation(plant)
        robot = H1_runtime(world, model=model, data=data)
        cargo_names=('c_payload','optical_red_0','optical_red_1','optical_blue')
        initial_tray_in_deck=((data.body('c_payload').xpos-data.body('c_deck').xpos)
                             @ data.body('c_deck').xmat.reshape(3,3)).copy()
        arm = robot.policy.default[robot.arm_ids].copy()
        io = JointWorldNavIO(plant, cfg)
        posture_armed = [False]
        def compose():
            nonlocal stopping_at
            control_begin=time.monotonic()
            posture_allowed=False
            if posture_armed[0]:
                posture = posture_check(model, data)
                posture_allowed=transport_gate.check(posture)
                if not posture_allowed:
                    report['control_failure_posture']=transport_gate.first_failure
                    if stopping_at is None:stopping_at=float(data.time)
            def loaded_permission(requested):
                load_allowed=movement_gate.check(latest,now_sim_s=float(data.time),
                    now_wall_s=time.monotonic(),motion_requested=requested)['motion_allowed']
                return posture_allowed and stopping_at is None and load_allowed
            control = robot.control(np.zeros(3), arm, {s:OPEN for s in ('left','right')},
                stationary=False, stopping=True, base_control=io.control(loaded_permission))
            if args.candidate == 'v7':
                for drive in ('c_retainer_-1_drive','c_retainer_1_drive'):
                    control[model.actuator(drive).id] = CLOSE_TARGET
            robot.tick += 1
            timed('control',control_begin)
            return control
        if args.candidate == 'v7':
            # Section 9.2 first ring: CLOSE CONFIRMATION before any ROS child
            # exists. Shoes are servoed by compose() (CLOSE_TARGET every step);
            # the posture gate stays DISARMED and motion stays refused until
            # both joints settle inside the declared closed band. Bounded:
            # G1 close budget 7.0 sim s + 5 s declared margin.
            # Relative budget: 12.0 was an ABSOLUTE sim-time cutoff, correct
            # only when the confirmation window starts near scene t=0
            # (run-03: confirmed at 1.806 s). G7-b runs the window after a
            # ~150 s transfer leg, so the absolute form refused on entry.
            close_deadline = float(data.time) + 12.0
            hold_s = 1.0
            hold_drift_max = 5e-3
            def qret_now():
                return [float(data.qpos[model.jnt_qposadr[model.joint(n + '_joint').id]])
                    for n in ('c_retainer_-1','c_retainer_1')]
            while True:
                owner.step(compose)
                qret = qret_now()
                if float(data.time) > close_deadline:
                    raise RuntimeError('RETAINER_CLOSE_NOT_CONFIRMED qpos=%r band=%r ctrl=%r' % (qret, RETAINER_BAND, [float(data.ctrl[model.actuator(n + '_drive').id]) for n in ('c_retainer_-1', 'c_retainer_1')]))
                if not all(RETAINER_BAND[0] <= q <= RETAINER_BAND[1] for q in qret):
                    continue
                # Phase 2: the pass-through trap (run-02 confirmed at 0.722 s
                # while still travelling to -0.0335). Require the joints to
                # HOLD the band for 1.0 sim s with bounded drift; any exit or
                # excess drift returns to phase 1 under the same deadline.
                hold_begin = float(data.time)
                q_hold = list(qret)
                settled = True
                while float(data.time) - hold_begin < hold_s:
                    owner.step(compose)
                    qret = qret_now()
                    if float(data.time) > close_deadline:
                        raise RuntimeError('RETAINER_CLOSE_NOT_CONFIRMED qpos=%r band=%r' % (qret, RETAINER_BAND))
                    if not all(RETAINER_BAND[0] <= q <= RETAINER_BAND[1] for q in qret):
                        settled = False
                        break
                    if any(abs(q - q0) > hold_drift_max for q, q0 in zip(qret, q_hold)):
                        settled = False
                        break
                if settled:
                    break
            posture_armed[0] = True
            report['retainer_close_confirmation']=dict(confirmed_sim_s=float(data.time),
                settle_qpos=qret, band=list(RETAINER_BAND), budget_sim_s=close_deadline,
                hold_s=hold_s, hold_drift_max_rad=hold_drift_max,
                posture_gate='armed after confirmation; CLOSED!=CLOSED_VERIFIED: bilateral support evidence stays with final_deck_support/support_forensics')
        # Fixed, validated strings only; own children have no physical simulator.
        for name, script, flags in (
            ('bridge', 'joint_world_ros_bridge.py', '--config '+cfg_name+' --command-source cmd_vel --duration '+str(args.duration+(900 if args.source_load else 10))+' --wait-for-state 30'),
            ('worker', 'joint_nav_worker.py', '--loaded --wall-budget '+('600' if args.receiver_leg else '300')+
                (' --centered-load' if args.centered_load else '')+
                (' --retained-motion' if args.retained_motion else '')+
                (' --turn-conservative' if args.turn_conservative else '')+
                (' --linear-speed '+str(args.linear_speed) if args.linear_speed is not None else '')+
                (' --goal '+' '.join(map(str,profile['goal'])) if args.receiver_leg else '')+
                (' --candidate '+args.candidate if args.candidate != 'v5' else ''))):
            owned.spawn(name, ['bash','-lc','source scripts/env.sh\nexec /usr/bin/python3 experiments/'+script+' --run-id '+args.run_id+' '+flags+' > '+(out/(name+'.log')).as_posix()+' 2>&1'])
        begin = float(data.time); wall_begin = time.monotonic()
        publish_schedule=StatePublishSchedule(sim_period_s=1/float(cfg['sim']['state_rate_hz']))
        next_sample = begin; next_progress = begin
        next_frame=begin
        worker_result = None
        worker_path = out/'nav2_worker_report.json'
        while data.time-begin < args.duration:
            if time.monotonic()-started > (660 if args.receiver_leg else (1000 if args.source_load else 380)): raise TimeoutError('shared Nav2 wall budget')
            if owned.handles['bridge'].poll() is not None: raise RuntimeError('bridge exited during physics')
            if (data.time>=next_frame or (latest and time.monotonic()-latest['observed_wall_s']>=.25)):
                stamp=time.monotonic()
                view.disable_depth_rendering();view.update_scene(data,camera='vehicle_load_cam')
                view.scene.flags[mujoco.mjtRndFlag.mjRND_SHADOW]=False
                view.scene.flags[mujoco.mjtRndFlag.mjRND_REFLECTION]=False
                rgb=view.render().copy()
                view.enable_depth_rendering();view.update_scene(data,camera='vehicle_load_cam')
                view.scene.flags[mujoco.mjtRndFlag.mjRND_SHADOW]=False
                view.scene.flags[mujoco.mjtRndFlag.mjRND_REFLECTION]=False
                depth=view.render().copy()
                latest=read_frame(rgb,depth,CAMERA,template,SIZE,roi=(1.7,2.7,-.4,.4),floor_z=floor_z,
                    observed_sim_s=float(data.time),observed_wall_s=stamp)
                latest['count_verified']=latest.get('counts')==dict(red=2,blue=1)
                support_rows=plant.tray_support()
                latest['support']='deck' if support_rows==['deck'] else 'UNKNOWN'
                # Exact-time independent snapshot, not another sampling period.
                # Do not conceal an absent or mixed solver contact as support.
                support_snapshot=None
                if latest['support']=='UNKNOWN':
                    tray_id=model.body('c_payload').id
                    pairs=[]
                    for index,contact in enumerate(data.contact[:data.ncon]):
                        if tray_id not in [int(model.geom_bodyid[g]) for g in (contact.geom1,contact.geom2)]:continue
                        force=np.empty(6);mujoco.mj_contactForce(model,data,index,force)
                        pairs.append(dict(geoms=[model.geom(g).name or '' for g in (contact.geom1,contact.geom2)],
                            distance_m=float(contact.dist),dim=int(contact.dim),force_contact_frame=force.tolist()))
                    support_snapshot=dict(rows=support_rows,contacts=pairs,
                        tray_xyz=data.body('c_payload').xpos.tolist(),
                        command_gate=io.gate.gate.summary(float(data.time)))
                frames.append(dict(sim_s=float(data.time),processing_wall_s=time.monotonic()-stamp,
                    observation=latest,support_snapshot_judge_only=support_snapshot))
                timed('rgbd',stamp)
                next_frame=float(data.time)+.2
            if worker_path.exists() and worker_result is None:
                worker_result = json.loads(worker_path.read_text())
                if stopping_at is None: stopping_at = float(data.time)
            if movement_gate.latched_reason and stopping_at is None: stopping_at=float(data.time)
            if owned.handles['worker'].poll() is not None and worker_result is None:
                raise RuntimeError('worker exited without durable report')
            if publish_schedule.due(float(data.time),time.monotonic()):io.publish()
            owner.step(compose)
            support_forensics.sample(data)
            if data.time >= next_sample:
                truth.append(dict(sim_s=float(data.time), xyz=data.body('n_base_link').xpos.tolist(),
                    velocity=[float(data.qvel[model.jnt_dofadr[model.joint(name).id]])
                              for name in ('n_slide_x','n_slide_y','n_slide_z','n_yaw')],
                    h_posture=permit.last_posture))
                truth[-1]['tray_xyz_judge_only']=data.body('c_payload').xpos.tolist()
                truth[-1]['support']=plant.tray_support()
                # Independent contact forensics only; never feeds navigation,
                # load permission or motor commands. Preserve the raw pairs.
                truth[-1]['contacts_judge_only']=[]
                for contact in data.contact[:data.ncon]:
                    names=[model.geom(g).name or '' for g in (contact.geom1,contact.geom2)]
                    if any(n.startswith(('c_tray','c_deck','c_pusher','n_')) for n in names):
                        truth[-1]['contacts_judge_only'].append(dict(geoms=names,
                            distance_m=float(contact.dist),dim=int(contact.dim)))
                truth[-1]['command_gate_judge_only']=io.gate.gate.summary(float(data.time))
                truth[-1]['transport_posture_judge_only']={name:float(data.qpos[
                    model.jnt_qposadr[model.joint(name).id]]) for name in POSTURE}
                truth[-1]['tray_in_deck_judge_only']=((data.body('c_payload').xpos-
                    data.body('c_deck').xpos) @ data.body('c_deck').xmat.reshape(3,3)).tolist()
                truth[-1]['cargo_judge_only']={}
                for name in cargo_names:
                    bid=model.body(name).id;velocity=np.empty(6)
                    mujoco.mj_objectVelocity(model,data,mujoco.mjtObj.mjOBJ_BODY,bid,velocity,0)
                    truth[-1]['cargo_judge_only'][name]=dict(xyz=data.xpos[bid].tolist(),
                        point_speed_bound_mps=float(np.linalg.norm(velocity[3:])+
                            collision_radius(model,bid)*np.linalg.norm(velocity[:3])))
                next_sample += .1
            if data.time >= next_progress:
                print('shared Nav2 sim %.2f commands %d worker %s'%(data.time,io.received,
                    worker_result['status'] if worker_result else 'RUNNING'), flush=True)
                checkpoint_begin=time.monotonic()
                checkpoint=dict(status='RUNNING', scope=report['scope'], full_order='NOT_RUN',
                    v1_complete=False, sim_s=float(data.time), truth_judge_only=truth[-61:],
                    motion_refusal=movement_gate.latched_reason,
                    posture_motion_refusal=transport_gate.latched_reason,
                    latest_sensor=latest, nav2_worker=worker_result)
                checkpoint['deck_support_judge_only']=support_forensics.summary()
                temporary=out/'checkpoint.tmp'
                temporary.write_text(json.dumps(checkpoint,indent=2,default=json_value)+'\n')
                temporary.replace(out/'checkpoint.json')
                timed('checkpoint',checkpoint_begin)
                next_progress += 2.
            if stopping_at is not None and data.time-stopping_at >= 5.: break
            delay = (float(data.time)-begin)/float(cfg['sim']['realtime_factor'])-(time.monotonic()-wall_begin)
            if delay > 0: time.sleep(min(delay,.05))
        if transport_gate.latched_reason or movement_gate.latched_reason:
            owned.stop('worker','joint_nav_worker.py')
            if worker_path.exists():worker_result=json.loads(worker_path.read_text())
        report.update(status='COMPLETED', nav2_worker=worker_result or {'status':'NOT_COMPLETED'},
            truth_judge_only=truth, states_sent=io.sent, commands_received=io.received,
            sim_s=float(data.time), owner_steps=owner.steps,
            same_model=robot.m is model and plant.model is model,
            same_data=robot.d is data and plant.data is data)
        srcl_flow = bool(args.source_load and args.receiver_leg)
        _g7c_ret = report.get('g7c_nudge_retention', {})
        g7c_retention_ok = (_g7c_ret.get('offset_max_drift_m', 1.0)
                            <= _g7c_ret.get('offset_drift_gate_m', 0.005)
                            and _g7c_ret.get('shoe_escape_guard_m', -1.0)
                            >= _g7c_ret.get('shoe_escape_limit_m', -0.005))
        g7c_ledger_ok = report.get('g7c_ledger_events') is not None
        stop = [row for row in truth if stopping_at is not None and row['sim_s'] >= stopping_at+1.]
        final = np.asarray(truth[-1]['xyz'])[:2]; goal = np.asarray(profile['goal'])[:2]
        drift = max((float(np.linalg.norm(np.asarray(row['xyz'])[:2]-np.asarray(stop[0]['xyz'])[:2])) for row in stop), default=float('inf'))
        speed = max((float(np.linalg.norm(row['velocity'][:3])) for row in stop), default=float('inf'))
        cargo_speed=max((row['cargo_judge_only'][name]['point_speed_bound_mps']
                         for row in stop for name in cargo_names),default=float('inf'))
        cargo_drift=max((float(np.linalg.norm(np.array(row['cargo_judge_only'][name]['xyz'])-
                              stop[0]['cargo_judge_only'][name]['xyz']))
                         for row in stop for name in cargo_names),default=float('inf'))
        tray_slip=max((float(np.linalg.norm(np.array(row['tray_in_deck_judge_only'])[:2]-
                          initial_tray_in_deck[:2])) for row in truth),default=float('inf'))
        checks = dict(nav2_succeeded=worker_result is not None and worker_result['status']=='SUCCEEDED',
            fresh_motion_command=worker_result is not None and worker_result.get('nonzero_command_count',0)>0 and io.received>0,
            independent_arrival=float(np.linalg.norm(final-goal)) <= .25,
            physical_motion=float(np.linalg.norm(final-np.asarray(profile['start'])))>.1,
            physical_stop=speed<=.01 and drift<=.005, shared_world=report['same_model'] and report['same_data'],
            no_safety_freeze=owner.frozen is None,
            load_motion_authorized=movement_gate.latched_reason is None,
            transport_posture_authorized=transport_gate.latched_reason is None,
            all_cargo_stopped=cargo_speed<=.01 and cargo_drift<=.005,
            tray_retained=(g7c_retention_ok if srcl_flow else tray_slip<=.005),
            final_visual_quantity=(None if srcl_flow else
                (latest.get('status')=='RESOLVED' and latest.get('counts')==dict(red=2,blue=1))),
            final_deck_support=(plant.tray_support()==['receiver_band'] if srcl_flow
                                else plant.tray_support()==['deck']))
        report['motion_refusal']=movement_gate.latched_reason
        report['posture_motion_refusal']=transport_gate.latched_reason
        report['loaded_navigation']='PASS' if all(checks.values()) else 'FAIL'
        if args.parking_handoff:
            from wheel_authority import return_to_parking
            # Durable worker report is written AFTER its Nav2 launch exits.
            owned.handles['worker'].wait(timeout=5)
            nav_stopped = bool(worker_result and worker_result['processes']['nav2'].get('exit_code') is not None)
            io.close()
            return_to_parking(plant,authority_token,
                dict(stopped_confirmed=checks['physical_stop'],held_speed_mps=speed,drift_m=drift),
                nav_process_stopped=nav_stopped,ipc_closed=io.closed)
            parked_begin=float(data.time)
            def parked_control():
                result=robot.control(np.zeros(3),arm,{s:OPEN for s in ('left','right')},
                    stationary=False,stopping=True,base_control=plant.park_control())
                robot.tick+=1
                return result
            while data.time-parked_begin<1.:owner.step(parked_control)
            checks['parking_authority_returned']=plant.heading_feedback_installed and plant.navigation_authority is None
            report['authority_handoff']='PARKING_TO_NAV2_TO_CONFIRMED_STOP_TO_PARKING'
        report.update(checks={k:('UNKNOWN' if v is None else ('PASS' if v else 'FAIL'))
                              for k,v in checks.items()},
            commissioning_result='PASS' if all(v is True for v in checks.values()) else 'FAIL',
            arrival_error_m=float(np.linalg.norm(final-goal)), stop_speed_mps=speed, stop_drift_m=drift)
        report.update(cargo_stop_speed_mps=cargo_speed,cargo_stop_drift_m=cargo_drift,
            max_tray_relative_to_deck_slip_m=tray_slip)
        if srcl_flow:
            report['flow_check_overrides'] = dict(
                final_deck_support='srcl unload flow: expected end support is receiver_band',
                tray_retained='judged on g7c_nudge_retention (drift from post-engagement rest), not vs scene init',
                final_visual_quantity='srcl unload flow: UNKNOWN -- no visual count observation exists in this flow (deck camera empty by design, no receiver camera); a committed ledger is bookkeeping, not observation and cannot PASS the gate [restored 2026-10-07; the ledger-evidence claim is retracted]',
                fresh_motion_command='srcl flow: the post-unload ROS leg is a null leg at the dock (see profile goal)')
            report['assumptions'].append(
                'G7-c nudge: wall-clock budgets measure the machine, not the vehicle; '
                'srcl2 measured a constant 0.049 m/s approach that a wall timeout misread '
                'as an x=7.9637 stall. Budget is sim-time (180 s) with a wall backstop (600 s).')
            report['assumptions'].append(
                'G7-c retention baseline redefinition (recorded): transport drift is measured '
                'from the post-engagement rest at the shoe catch point (geometry-derived, '
                'g7c_shoe_engagement), no longer from the first nudge row. The pre-engagement '
                'slide (landing -> catch) is free-roller deck physics on a level deck '
                '(tray 0.098 kg, roller damping 0), reported as pre_catch_slide_m; the 15 mm '
                'first-row budget is DIAGNOSTIC ONLY since 2026-10-07 -- the retention gate '
                'is 5 mm vs the catch baseline (original budget restored).')
            _g = report.get
            _dock = _g('g7c_dock') or {}
            _gate = {
                'source_transfer_on_deck': _g('source_transfer_support') == ['deck'],
                'pretransport_close_confirmed': 'g7c_pretransport_close' in report,
                'dock_reached_within_2cm': bool(abs(_dock.get('chassis_x_m', 9e9)
                                                    - _dock.get('target_m', 0.)) <= 0.021),
                'shoes_opened_at_dock': all(q > 1.0 for q in (_g('g7c_shoes_open_qpos') or [0., 0.])),
                'unload_on_receiver': _g('g7c_unload_support') == ['receiver_band'],
                'shoe_engagement_confirmed': 'g7c_shoe_engagement' in report,
                'retention_within_budget': bool(g7c_retention_ok),
                'ledger_events_committed': bool((_g('g7c_ledger_events') or 0) > 0),
            }
            _gate['result'] = 'PASS' if all(_gate.values()) else 'FAIL'
            report['g7c_gate'] = _gate
    except (Exception, KeyboardInterrupt) as exc:
        report['status'] = 'ERROR'
        report['error'] = type(exc).__name__+': '+str(exc)
        report['exception_chain']=exception_chain(exc)
        report['exception_traceback']=traceback.format_exc()
        report['commissioning_result'] = 'FAIL'
        if io is not None and owner is not None and owner.frozen is None:
            brake_begin=float(data.time);brake_wall=time.monotonic();brake_rows=[]
            try:
                def brake_control():
                    control=robot.control(np.zeros(3),arm,{s:OPEN for s in ('left','right')},
                        stationary=False,stopping=True,base_control=io.control(lambda requested:False))
                    robot.tick+=1
                    return control
                while data.time-brake_begin<5:
                    if time.monotonic()-brake_wall>30:raise TimeoutError('failure brake wall budget')
                    owner.step(brake_control)
                    if data.time-brake_begin>=1:
                        brake_rows.append(dict(sim_s=float(data.time),xyz=data.body('n_base_link').xpos.tolist(),
                            velocity=[float(data.qvel[model.jnt_dofadr[model.joint(name).id]])
                                for name in ('n_slide_x','n_slide_y','n_slide_z','n_yaw')]))
                speed=max(float(np.linalg.norm(r['velocity'][:3])) for r in brake_rows)
                drift=max(float(np.linalg.norm(np.array(r['xyz'])-brake_rows[0]['xyz'])) for r in brake_rows)
                report['failure_brake']=dict(truth_judge_only=brake_rows,speed_mps=speed,drift_m=drift,
                    vehicle_stop_result='PASS' if speed<=.01 and drift<=.005 else 'FAIL',
                    cargo_stop='NOT_MEASURED',task_reauthorized=False)
            except Exception as brake_error:
                report['failure_brake_error']=exception_chain(brake_error)
    finally:
        # Child worker first: its finally stops only its own Nav2 process group.
        owned.stop('worker','joint_nav_worker.py'); owned.stop('bridge','joint_world_ros_bridge.py'); owned.close()
        if io is not None: io.close()
        if view is not None: view.close()
        report.update(processes=owned.record, wall_s=time.monotonic()-started,
            timing_peaks_wall_s=timing,
            truth_judge_only=truth, stopping_at_sim_s=stopping_at,
            sensor_frames=frames,
            simulation_frozen=owner.frozen if owner else None)
        report['deck_support_judge_only']=support_forensics.summary() if support_forensics else None
        temporary=out/'report.tmp'
        temporary.write_text(json.dumps(report, indent=2,default=json_value)+'\n')
        temporary.replace(out/'report.json')
        print(json.dumps({k:v for k,v in report.items() if k not in ('truth_judge_only','sensor_frames')},indent=2,default=json_value),flush=True)
    return 0 if report.get('commissioning_result')=='PASS' else 1


if __name__=='__main__': raise SystemExit(main())
