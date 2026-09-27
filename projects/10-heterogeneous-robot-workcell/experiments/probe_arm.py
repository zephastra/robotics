"""PROBE-A: a fixed arm grasps a separate part off a table, lifts it, carries it, and sets it
down, with the grasp held by CONTACT ONLY.

The whole point of this probe is the sentence in `docs/TEST_AND_ACCEPTANCE.md`:

    | PROBE-A | 固定臂从桌面抓起并放下一个零件 |
    | 必需证据 | 夹爪接触、物体离台、释放后稳定；规划附着不能充当接触 |

so the machine is allowed to do exactly one thing to the world: write actuator commands. It
never touches `qpos`, never adds an equality, never welds. If the pads do not touch the part,
the part does not move -- and the run says GRIP_FAILED rather than quietly succeeding.

Lessons from earlier branches are built in rather than re-learned:

  * **Motion is interpolated in CARTESIAN space, with IK at every waypoint.** The H experiment
    lost a round to this: commanding a joint-space target let a rate limiter interpolate along
    a path nobody chose, and the fingers swept the tray.
  * **A lost payload latches.** The C branch recorded `fell_at` and read it for nothing, so 211
    runs drove on for a total of 79 minutes after dropping their load. Here a drop sets
    `PAYLOAD_LOST`, holds the arm and ends the run.
  * **"Exhausted" and "not yet planned" are different states.** The first version of the motion
    step tested `if not waypoints` to decide whether to plan, which re-planned on every tick
    after the list emptied, so the machine never reached its settle branch and timed out while
    standing still. The sentinel is `None`.
  * **The payload's initial pose comes from `qpos0`, not from a keyframe.**
    `mj_resetDataKeyframe` pads a 9-value keyframe out to nq=16, and the padding lands on the
    payload's free joint: the part started on the floor at the world origin.
"""
import argparse
import hashlib
import json
import math
import os
import pathlib
import sys
import time

import numpy as np

HERE = pathlib.Path(__file__).resolve().parent
ROOT = HERE.parent
sys.path.insert(0, str(HERE))
import arm_rig as rig                                                  # noqa: E402

import mujoco                                                          # noqa: E402

REPORTS = ROOT / 'reports'
WORLD = rig.WORLD

PHYSICS_DT = 0.002
CTRL_EVERY = 10                 # physics steps per control step -> 50 Hz control
CTRL_DT = PHYSICS_DT * CTRL_EVERY
MAX_STEP_M = 0.004              # grasp-frame travel per control step -> 0.20 m/s
SETTLE_QVEL = 0.05              # rad/s per arm joint
SETTLE_PAYLOAD_SPEED = 0.01     # m/s
CLOSE_SECONDS = 1.0
RELEASE_SECONDS = 0.6
FALL_DEPTH = 0.020              # how far the part must sink to count as dropped

# The arm actuators are position servos with a gravity-induced steady-state error: IK asks
# for the grasp frame at z = 0.215 and the servo settles at 0.2098, about 5 mm low. That is
# not a rounding detail -- it put the finger shell into the table (measured contact
# `table|left_finger`) and the friction against the table was what stopped the gripper
# closing. So each motion state now CLOSES THE LOOP: measure where the grasp frame actually
# ended up, and re-aim by the residual. Asking a servo for a position and assuming you got it
# is the same class of mistake as reading an encoder value as a fact.
GRASP_SEEK_TOL = 0.002          # m, acceptable residual at a waypoint
GRASP_SEEK_MAX = 60             # control steps spent correcting per state
# After the close ramp finishes, keep holding and wait for contact before declaring failure.
# The first version judged contact the instant the ramp ended, so it called GRIP_FAILED on a
# gripper that was still closing and made contact 1.2 s later.
CLOSE_CONTACT_WAIT = 2.5
SIM_SECONDS_MAX = 90.0

MOTION_STATES = ('APPROACH', 'DESCEND', 'LIFT', 'CARRY', 'LOWER', 'RETREAT')
NEXT_STATE = {'APPROACH': 'DESCEND', 'DESCEND': 'CLOSE', 'LIFT': 'CARRY',
              'CARRY': 'LOWER', 'LOWER': 'RELEASE', 'RETREAT': 'HOLD'}
# which waypoint each motion state travels to
MOTION_TARGET = {'APPROACH': 'pregrasp', 'DESCEND': 'grasp', 'LIFT': 'lift',
                 'CARRY': 'above_place', 'LOWER': 'place', 'RETREAT': None}

# The set of part placements inside which this skill is CLAIMED to work, at EVERY swept
# friction. It is a LIST, not a half-range, because the limit is not monotonic.
#
# DERIVED BY RUNNING THE JUDGE over 18 candidate placements (`reports/p1-a-envscan`), not by
# reading `final_state == DONE`. Two earlier attempts made that mistake and credited placements
# the judge rejects: a +10 mm y shift reaches DONE but places the part 33 mm from the target,
# and the (4, -6) mm diagonal slides 25.6 mm in the hand at mu=0.30. **"DONE" is the state
# machine finishing, not the skill working.** Eight of the eighteen survive every content
# criterion at all three frictions; those are listed here.
#
# The closing-axis direction (+x) is the interesting one: +2/+3/+4 mm pass, **+5 and +6 mm
# FAIL**, and +8 mm passes again. A parallel gripper does not self-centre the part it grasps,
# so an off-axis grip can push the part out of the jaws; at 8 mm the pad contact has moved past
# the part's corner and the part is captured differently. A half-range would therefore be a lie
# in both directions, which is why this is enumerated.
PART_PLACEMENT_ENVELOPE = ((0.0, 0.0), (0.002, 0.0), (0.003, 0.0), (0.004, 0.0),
                           (0.008, 0.0), (0.0, 0.005), (0.0, -0.005), (0.003, 0.006))

SCENARIOS = (
    # the gripper is commanded fully shut; the part blocks it, and that blocked error IS the
    # normal force. This is the case that must work.
    {'name': 'nominal', 'close_ctrl': 0.0, 'expect': 'DONE'},
    # ---- inside the declared envelope, exercised point by point. Ground truth is used for
    #      INITIALISATION here, which TEST_AND_ACCEPTANCE explicitly permits; it never reaches
    #      the controller, which only ever sees joint positions and commands.
    {'name': 'part_shift_x_p4', 'close_ctrl': 0.0, 'part_offset': (0.004, 0.0),
     'expect': 'DONE'},
    {'name': 'part_shift_x_p8', 'close_ctrl': 0.0, 'part_offset': (0.008, 0.0),
     'expect': 'DONE'},
    {'name': 'part_shift_y_p5', 'close_ctrl': 0.0, 'part_offset': (0.0, 0.005),
     'expect': 'DONE'},
    {'name': 'part_shift_y_m5', 'close_ctrl': 0.0, 'part_offset': (0.0, -0.005),
     'expect': 'DONE'},
    {'name': 'part_shift_xy', 'close_ctrl': 0.0, 'part_offset': (0.003, 0.006),
     'expect': 'DONE'},
    # ---- OUTSIDE the declared envelope. Still run, still reported: a limit that is never
    #      exercised is an assumption. These are simply not CLAIMED.
    # The gripper is commanded to stop fully open, so the pads never reach the part. This MUST
    # fail, and it is what lets the judge be shown a real failure as well as successes.
    {'name': 'grip_never_closes', 'close_ctrl': 255.0, 'expect': 'GRIP_FAILED',
     'in_envelope': False},
    # The measured HOLE: completes at mu 0.30/0.60, GRIP_FAILED at mu 1.00.
    {'name': 'part_shift_x_p5', 'close_ctrl': 0.0, 'part_offset': (0.005, 0.0),
     'expect': 'DONE', 'in_envelope': False},
    # The y boundary: the pads overhang the part's edge, so it slides in the hand and lands off
    # target.
    {'name': 'part_shift_y_p8', 'close_ctrl': 0.0, 'part_offset': (0.0, 0.008),
     'expect': 'DONE', 'in_envelope': False},
    # The diagonal that was claimed first and then measured out.
    {'name': 'part_shift_xy_out', 'close_ctrl': 0.0, 'part_offset': (0.004, -0.006),
     'expect': 'DONE', 'in_envelope': False},
)


def _log(msg):
    print(msg, flush=True)


def stages_in_source():
    """Required stages this file can actually emit, read out of its own source.

    Derived for the same reason the C branch derived `full_C_acceptance`: a typed claim cannot
    notice that it has gone stale. If a stage is added to the requirement list but not to the
    machine, this shrinks and the claim is withheld by itself.
    """
    src = pathlib.Path(__file__).resolve().read_text(encoding='utf-8')
    return sorted({s for s in rig.COORDINATE_PATH if f"'{s}'" in src})


def _path(start, end):
    """Cartesian waypoints from start to end, at most MAX_STEP_M apart."""
    start = np.asarray(start, dtype=float)
    end = np.asarray(end, dtype=float)
    d = end - start
    n = max(1, int(math.ceil(float(np.linalg.norm(d)) / MAX_STEP_M)))
    return [start + d * (i + 1) / n for i in range(n)]


def _body_name(model, body_id):
    return mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, body_id) or f'body{body_id}'


def _record(records, model, data, state, b, payload_qadr, payload_dof,
            grip_ctrl_val, bodies_fingers, body_hand, body_payload, body_table):
    # A GRASP means the FINGERS hold the part. The palm is not a grasp: counting it let the
    # "gripper never closes" case -- jaws commanded fully open, 83 mm apart, with a 40 mm part
    # between them -- "succeed", because the palm happened to rest on the part's top face.
    grip = rig.contacts_between(model, data, bodies_fingers, body_payload)
    palm = rig.contacts_between(model, data, {body_hand}, body_payload)
    table = rig.contacts_between(model, data, {body_table}, body_payload)
    floor = rig.contacts_between(model, data, {0}, body_payload)
    rec = {
        't': float(data.time),
        'state': state,
        'payload': [float(v) for v in data.qpos[payload_qadr:payload_qadr + 3]],
        'payload_quat': [float(v) for v in data.qpos[payload_qadr + 3:payload_qadr + 7]],
        'payload_speed': float(np.linalg.norm(data.qvel[payload_dof:payload_dof + 3])),
        'grasp': [float(v) for v in rig.grasp_point(model, data)],
        'arm_q': [float(v) for v in data.qpos[b['arm_qadr']]],
        'finger_q': [float(v) for v in data.qpos[b['finger_qadr']]],
        'gap': float(rig.finger_gap(model, data)),
        'grip_ctrl': float(grip_ctrl_val),
        # NAMES, not a count. The C branch had a bare `contacts` count that reported 8 while
        # the tray floated 0.46 m in the air, because it summed every geom of the body.
        'grip_contacts': sorted({_body_name(model, int(model.geom_bodyid[g])) for g, _ in grip}),
        'palm_contact_n': len(palm),
        'table_contact_n': len(table),
        'floor_contact_n': len(floor),
        # the arm itself touching the work surface. Not part of PROBE-A's required evidence,
        # but it was the actual cause of a failed grip and it must not be invisible again.
        'finger_table_contact_n': len(rig.contacts_between(model, data,
                                                           bodies_fingers + (body_hand,),
                                                           body_table)),
    }
    records.append(rec)
    return rec


def run_scenario(scenario):
    """One transfer attempt at the default friction. Returns a dict; writes nothing."""
    t_wall = time.time()
    friction = float(scenario.get('friction', rig.DEFAULT_FRICTION))
    # Defined here, not next to `close_ctrl` further down: the INITIAL STATE block below uses
    # it, and defining it lower made the initialisation raise UnboundLocalError. The comment is
    # longer than the fix on purpose -- "defined after use" is invisible until the line runs.
    part_offset = tuple(float(v) for v in scenario.get('part_offset', (0.0, 0.0)))
    model, data = rig.load(friction)
    b = rig.ids(model)
    plan = mujoco.MjData(model)                 # IK runs here, so it cannot perturb the sim
    arm_ctrl_idx = [model.actuator(f'actuator{i}').id for i in range(1, 8)]
    grip_ctrl_idx = model.actuator(rig.GRIPPER_ACTUATOR).id
    payload_qadr = b['payload_qadr']
    payload_dof = b['payload_dof']
    arm_qadr = b['arm_qadr']
    rest_z = rig.payload_rest_z()
    bodies_fingers = (b['left_finger'], b['right_finger'])
    body_hand = b['hand']
    body_payload = b['payload']
    body_table = b['table']

    # ---- initial state: payload from qpos0 (its declared pose), arm from the home keyframe
    mujoco.mj_resetData(model, data)
    data.qpos[arm_qadr] = model.key_qpos[0][:7]
    data.qpos[b['finger_qadr']] = model.key_qpos[0][7:9]
    # displace the part before the run: its declared start IS the pick target plus this offset
    data.qpos[payload_qadr] = float(rig.PICK_XY[0]) + part_offset[0]
    data.qpos[payload_qadr + 1] = float(rig.PICK_XY[1]) + part_offset[1]
    data.ctrl[:] = model.key_ctrl[0]
    data.ctrl[grip_ctrl_idx] = rig.GRIPPER_CTRL_OPEN
    mujoco.mj_forward(model, data)
    start = np.array(data.qpos[payload_qadr:payload_qadr + 3], dtype=float)
    want = np.array([rig.PICK_XY[0] + part_offset[0],
                     rig.PICK_XY[1] + part_offset[1], rest_z])
    if float(np.linalg.norm(start - want)) > 1e-6:
        raise RuntimeError(f'the payload did not start at its declared pose: '
                           f'{np.round(start, 5)} vs {np.round(want, 5)}')

    # settle on the table before commanding anything
    for _ in range(1000):
        mujoco.mj_step(model, data)

    # The hand keeps the home orientation throughout: the finger-length axis stays vertical so
    # the pads present flat faces to the part. Captured from the model, not typed.
    grasp_rot = rig.home_hand_rot(model, plan)
    poses = rig.planned_poses()
    retreat_target = np.array(poses['place']) + np.array([0.0, 0.0, rig.RETREAT_HEIGHT])
    close_ctrl = float(scenario['close_ctrl'])

    def plan_move(target_pos):
        """IK along a straight line from the current pose to target_pos."""
        here = rig.grasp_point(model, data)
        seed = np.array(data.qpos[arm_qadr], dtype=float)
        out = []
        for pt in _path(here, target_pos):
            plan.qpos[:] = data.qpos
            q, _, _ = rig.ik(model, plan, pt, target_rot=grasp_rot, q_seed=seed)
            seed = q
            out.append(q)
        return out

    def command(q_arm, grip_ctrl):
        data.ctrl[arm_ctrl_idx] = q_arm
        data.ctrl[grip_ctrl_idx] = grip_ctrl

    def settled():
        qv = float(np.abs(data.qvel[b['arm_dof']]).max())
        pv = float(np.linalg.norm(data.qvel[payload_dof:payload_dof + 3]))
        return qv < SETTLE_QVEL and pv < SETTLE_PAYLOAD_SPEED

    state = 'APPROACH'
    state_since = float(data.time)
    waypoints = None            # None = not planned yet; [] = planned and exhausted
    seek = 0
    records = []
    fell_at = None
    faulted = False
    close_contact_samples = 0
    payload_contact_samples = 0
    post_release_contact_samples = 0
    transitions = [{'state': state, 't': float(data.time)}]

    def goto(new_state):
        nonlocal state, state_since, waypoints, seek
        state, state_since, waypoints, seek = new_state, float(data.time), None, 0
        transitions.append({'state': new_state, 't': float(data.time)})

    while data.time < SIM_SECONDS_MAX:
        pz = float(data.qpos[payload_qadr + 2])
        grip_now = float(data.ctrl[grip_ctrl_idx])

        # ---- the fall latch runs BEFORE the state machine, every tick -------------------
        if fell_at is None and pz < rest_z - FALL_DEPTH and state not in ('APPROACH', 'DESCEND'):
            fell_at = float(data.time)
            faulted = True
            command(np.array(data.qpos[arm_qadr], dtype=float), rig.GRIPPER_CTRL_OPEN)
            goto('PAYLOAD_LOST')
            _record(records, model, data, state, b, payload_qadr, payload_dof,
                    rig.GRIPPER_CTRL_OPEN, bodies_fingers, body_hand, body_payload,
                    body_table)
            break

        rec = _record(records, model, data, state, b, payload_qadr, payload_dof,
                      grip_now, bodies_fingers, body_hand, body_payload, body_table)
        if rec['grip_contacts']:
            payload_contact_samples += 1
            if state == 'CLOSE':
                close_contact_samples += 1
            if state in ('RELEASE', 'RETREAT', 'HOLD'):
                post_release_contact_samples += 1

        elapsed = float(data.time) - state_since
        if elapsed > rig.STATE_TIMEOUT.get(state, 10.0):
            faulted = True
            # "stop acting" means HOLD the arm. Zeroing ctrl on a position-servo arm produces
            # a violent move to an all-zero pose, which is the opposite of stopping.
            command(np.array(data.qpos[arm_qadr], dtype=float), grip_now)
            goto('TIMEOUT')
            break

        # ---- actions -------------------------------------------------------------------
        if state in MOTION_STATES:
            # RETREAT belongs on the OPEN side of this choice. It is a retreat AFTER
            # release: the first version closed the gripper again here, so the arm re-grasped
            # the part in mid-air, carried it up 165 mm, and then dropped it when HOLD opened
            # the jaws -- which the release-stability check reported, correctly, as a
            # 165 mm drift. The bug was here, not in the physics.
            grip = (rig.GRIPPER_CTRL_OPEN
                    if state in ('APPROACH', 'DESCEND', 'RETREAT') else close_ctrl)
            if waypoints is None:
                seek = 0
                target = retreat_target if MOTION_TARGET[state] is None \
                    else np.asarray(poses[MOTION_TARGET[state]], dtype=float)
                waypoints = plan_move(target)
            if waypoints:
                command(waypoints.pop(0), grip)
            else:
                residual = np.asarray(target, dtype=float) - rig.grasp_point(model, data)
                if float(np.linalg.norm(residual)) > GRASP_SEEK_TOL and seek < GRASP_SEEK_MAX:
                    # re-aim by the measured residual; this is the servo's droop, not a plan
                    plan.qpos[:] = data.qpos
                    q_corr, _, _ = rig.ik(model, plan,
                                          np.asarray(target, dtype=float) + residual,
                                          target_rot=grasp_rot,
                                          q_seed=np.array(data.qpos[arm_qadr], dtype=float))
                    command(q_corr, grip)
                    seek += 1
                elif seek < GRASP_SEEK_MAX or settled():
                    command(np.array(data.qpos[arm_qadr], dtype=float), grip)
                    if settled():
                        goto(NEXT_STATE[state])

        elif state == 'CLOSE':
            frac = min(1.0, elapsed / CLOSE_SECONDS)
            grip = rig.GRIPPER_CTRL_OPEN + (close_ctrl - rig.GRIPPER_CTRL_OPEN) * frac
            command(np.array(data.qpos[arm_qadr], dtype=float), grip)
            if close_contact_samples > 0:
                goto('LIFT')
            elif frac >= 1.0 and elapsed > CLOSE_SECONDS + CLOSE_CONTACT_WAIT:
                faulted = True
                goto('GRIP_FAILED')

        elif state == 'RELEASE':
            frac = min(1.0, elapsed / RELEASE_SECONDS)
            grip = close_ctrl + (rig.GRIPPER_CTRL_OPEN - close_ctrl) * frac
            command(np.array(data.qpos[arm_qadr], dtype=float), grip)
            if frac >= 1.0:
                goto('RETREAT')

        elif state == 'HOLD':
            command(np.array(data.qpos[arm_qadr], dtype=float), rig.GRIPPER_CTRL_OPEN)
            if elapsed >= rig.HOLD_SECONDS:
                goto('DONE')
                break

        else:                                        # a fault state: hold and stop acting
            command(np.array(data.qpos[arm_qadr], dtype=float), grip_now)
            break

        for _ in range(CTRL_EVERY):
            mujoco.mj_step(model, data)

    final = _record(records, model, data, state, b, payload_qadr, payload_dof,
                    float(data.ctrl[grip_ctrl_idx]), bodies_fingers, body_hand,
                    body_payload, body_table)

    return {
        'scenario': {k: v for k, v in scenario.items()},
        'part_offset': list(part_offset),
        'friction': friction,
        'final_state': state,
        'faulted': bool(faulted),
        'fell_at': fell_at,
        'close_ctrl': close_ctrl,
        'max_close_contact_run': close_contact_samples,
        'payload_contact_samples': payload_contact_samples,
        'post_release_contact_samples': post_release_contact_samples,
        'transitions': transitions,
        'stages_executed': sorted({r['state'] for r in records}),
        'records': records,
        'final': final,
        'n_samples': len(records),
        'sim_seconds': float(data.time),
        'wall_seconds': time.time() - t_wall,
        'initial_payload': [float(v) for v in start],
    }


SCOPE_CLAIM = ('A FIXED-ARM COMPONENT; a single skill on a stand-in work surface, not a '
               'production pick station. One part, one grasp configuration, one pick and one '
               'place pose; no vision, no pose disturbance, no part variation.')


def full_a_state(runs):
    """`CLAIMED` only when every required stage exists in the source AND actually ran.

    Deliberately derived, not typed: the C branch shipped a typed "NOT_RUN" that sat beside a
    computed "0 stages never executed", i.e. one line asserting two contradictory things.
    """
    required = set(rig.COORDINATE_PATH)
    in_source = set(stages_in_source())
    executed = set()
    for run in runs:
        executed |= set(run.get('stages_executed') or ())
    if required <= in_source and required <= executed:
        return 'CLAIMED'
    return 'NOT_RUN'


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--run-id', required=True)
    parser.add_argument('--reports-dir', default=str(REPORTS))
    parser.add_argument('--friction', type=float, default=None,
                        help='single friction; default is the whole sweep')
    parser.add_argument('--scenario', default=None, help='single scenario name')
    parser.add_argument('--no-traces', action='store_true',
                        help='do not write per-run trace files (they are large)')
    args = parser.parse_args()

    frictions = [args.friction] if args.friction is not None else list(rig.FRICTION_SWEEP)
    scenarios = [s for s in SCENARIOS if args.scenario is None or s['name'] == args.scenario]
    if not scenarios:
        _log(f'no scenario named {args.scenario!r}; known: {[s["name"] for s in SCENARIOS]}')
        return 2

    # Generate the scene from arm_rig's constants before running anything. Skipping this is how
    # a run ends up simulating a world that no longer matches the code: the part height changed
    # and the probe happily loaded the previous 30 mm scene, then failed its own start-pose
    # assertion. A test asserts `arm_rig.build(check=True) == 0`, so the committed file is also
    # verified against its source rather than only being regenerated here.
    _log(f'arm_rig.build() rc={rig.build()}')

    out = pathlib.Path(args.reports_dir) / args.run_id
    out.mkdir(parents=True, exist_ok=True)
    (out / 'world.xml').write_text(WORLD.read_text(encoding='utf-8'), encoding='utf-8')

    runs = []
    for sc in scenarios:
        for mu in frictions:
            r = run_scenario({**sc, 'friction': mu})
            _log(f'  {sc["name"]:<20} mu={mu:<5} -> {r["final_state"]:<14} '
                 f'{r["sim_seconds"]:6.2f} s sim, {r["wall_seconds"]:5.2f} s wall, '
                 f'{r["n_samples"]} samples, grip-contact samples {r["payload_contact_samples"]}')
            trace_name = f'trace_{sc["name"]}_mu{mu}.json'
            if not args.no_traces:
                (out / trace_name).write_text(
                    json.dumps(r, separators=(',', ':')), encoding='utf-8')
            entry = {k: v for k, v in r.items() if k != 'records'}
            # The judge reads the trace by name rather than getting a copy of every sample
            # inside report.json, so there is exactly one authoritative record of the run.
            entry['trace_file'] = trace_name
            runs.append(entry)

    report = {
        'run_id': args.run_id,
        'world': str(WORLD.relative_to(ROOT)),
        'world_sha256': hashlib.sha256(WORLD.read_bytes()).hexdigest(),
        'panda_asset_sha256': hashlib.sha256(rig.PANDA_XML.read_bytes()).hexdigest(),
        'panda_upstream_revision': '822c2d8f877dd166c5b7d3c9f7e3c3b6589473b7',
        'mujoco': mujoco.__version__,
        'pid': os.getpid(),
        'physics_dt': PHYSICS_DT,
        'control_dt': CTRL_DT,
        'max_step_m': MAX_STEP_M,
        'frictions': frictions,
        'part_offsets_swept': sorted({tuple(r['part_offset'])
                                      for r in runs if r.get('part_offset')}),
        'scenarios': [dict(s) for s in scenarios],
        'payload_half': list(rig.PAYLOAD_HALF),
        'payload_mass': rig.PAYLOAD_MASS,
        'table_top_z': rig.TABLE_TOP_Z,
        'pick_xy': list(rig.PICK_XY),
        'place_xy': list(rig.PLACE_XY),
        'planned_poses': {k: list(v) for k, v in rig.planned_poses().items()},
        'gripper_ctrl_range': list(rig.GRIPPER_CTRL_RANGE),
        'requires': list(rig.COORDINATE_PATH),
        'stages_in_source': stages_in_source(),
        'fault_states': list(rig.FAULT_STATES),
        'full_A_acceptance': full_a_state(runs),
        'scope_claim': SCOPE_CLAIM,
        'runs': runs,
        'thresholds': {
            'min_contact_samples': rig.MIN_CONTACT_SAMPLES,
            'lift_margin_m': rig.LIFT_MARGIN,
            'carried_slip_max_m': rig.CARRIED_SLIP_MAX,
            'place_tol_xy_m': rig.PLACE_TOL_XY,
            'release_drift_max_m': rig.RELEASE_DRIFT_MAX,
            'release_speed_max_mps': rig.RELEASE_SPEED_MAX,
            'touchdown_tol_m': rig.TOUCHDOWN_TOL,
            'fall_depth_m': FALL_DEPTH,
        },
    }
    (out / 'report.json').write_text(json.dumps(report, indent=1), encoding='utf-8')
    _log(json.dumps({'run_id': args.run_id, 'runs': len(runs)}))
    return 0


if __name__ == '__main__':
    sys.exit(main())
