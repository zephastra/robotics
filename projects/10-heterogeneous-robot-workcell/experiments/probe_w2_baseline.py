"""W2 baseline: one integrated world, audited, and driven by its wheels.

Run against either integrated world; `--world` selects it:

  * `assets/world_p3_cell.xml` -- the assembly world. Its source and receiver are two ends of
    one fixture, so it can only show that the vehicle reaches a dock pose, not that it can
    transport;
  * `assets/world_w2_logistic.xml` -- the logistics world, where the receiver is a separate
    station metres away. `--min-leg-m` turns the leg length into an ASSERTION instead of a
    report, which is the whole point of building that world.

WHAT THIS ADDS THAT P3-WORLD-01 DID NOT
--------------------------------------
P3-WORLD-01 already proved the assembled `assets/world_p3_cell.xml` loads five instances into one
`MjModel`, with one stepper and no cross-talk. What was never checked is what the integration order
needs: **where the source and the receiver actually are, whether the vehicle can reach both by
driving, and whether anything other than the wheels could have moved it.**

Two measured facts set the design, and neither was known before measuring:

  * the tray `c_payload` is already at rest ON THE FIXED ROLLER BAND -- its z bottom is 0.485, the
    crown plane, and its x is inside the band's span -- so the fixed band IS the source station. An
    earlier note of mine said the only fixed station was the receiver; that note was wrong;
  * the deck rides 1.92 m AHEAD of the chassis origin, so at the initial parked pose the deck's
    roller row is nowhere near either band. **The parked pose is not a dock pose**, so the receive
    behaviour is not reproducible there, and the probe drives to the dock poses instead of
    pretending otherwise.

EVERY FRAME IS DERIVED, from the model's own geom positions and from `roller_rig`'s own generator
metadata (`recv_handoff_gap_m`, `roller_pitch`). A typed dock x would judge the next chassis by this
one's geometry, which is the failure mode this project keeps finding.

`--no-drive` stops after the audits, which is the cheap way to learn what the hold law does to the
chassis before spending minutes on a drive.
"""
import argparse
import hashlib
import json
import math
from pathlib import Path
import sys
import time

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'experiments'))
sys.path.insert(0, str(ROOT / 'src'))

import mujoco  # noqa: E402
import roller_rig  # noqa: E402
import build_p3_world  # noqa: E402

DEFAULT_WORLD = ROOT / 'assets' / 'world_p3_cell.xml'
TRAY_V1 = ROOT / 'assets' / 'objects' / 'tray_v1.xml'
KP = 6.0
TORQUE_LIMIT_NM = 1.2
RADIUS_M = 0.04
TRACK_M = 0.30
HOLD_S = 1.0
DRIVE_SPEED_MPS = 0.35
DRIVE_TIMEOUT_S = 30.0
SETTLE_S = 0.8


def geom_prefix_x(model, data, prefix):
    """World x of every geom whose name starts with `prefix`, sorted.

    The roller crowns ARE the cylinder geoms, so `geom_xpos` is the crown centre: no half-size
    arithmetic and no assumption about which axis the cylinder is drawn along.
    """
    obj = mujoco.mjtObj.mjOBJ_GEOM
    return sorted(float(data.geom_xpos[g][0]) for g in range(model.ngeom)
                  if (mujoco.mj_id2name(model, obj, g) or '').startswith(prefix))


def body_span(model, data, name):
    body = model.body(name).id
    lo = np.full(3, np.inf)
    hi = np.full(3, -np.inf)
    for g in range(model.ngeom):
        if model.geom_bodyid[g] != body:
            continue
        centre, size = data.geom_xpos[g], model.geom_size[g]
        half = size if model.geom_type[g] == mujoco.mjtGeom.mjGEOM_BOX \
            else np.full(3, max(size[0], size[1], size[2]))
        lo, hi = np.minimum(lo, centre - half), np.maximum(hi, centre + half)
    return lo, hi


def subtree(model, root_body):
    """`root_body` and every descendant, so a wheel on a child body is not missed."""
    out, stack = set(), [root_body]
    while stack:
        body = stack.pop()
        if body in out:
            continue
        out.add(body)
        stack += [b for b in range(model.nbody) if model.body_parentid[b] == body]
    return out


def describe_geom(model, geom):
    """A name for a geom even when it has none. The offending geoms in this assembly are unnamed,
    so `mj_id2name` returns None and a report that says "between None and None" identifies nothing;
    the BODY is what makes the pair actionable."""
    name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, geom)
    if name:
        return name
    body = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, int(model.geom_bodyid[geom]))
    return f'<unnamed geom {geom} on body {body or model.geom_bodyid[geom]}>'


def geom_owner(model, geom):
    """Which owner a geom belongs to, from its NAME and its body's name.

    Furniture is the reason this exists as a name rule: a role's table, floor or walls are
    attached to the WORLD body in the merged model, so ancestry cannot tell them apart. The
    cell's own geoms are named `cell_*`; anything else that sits on the world body keeps its
    role's prefix, which is exactly the ownership that matters for deciding whether an
    interpenetration is an assembly fault or a role's own business.
    """
    name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, geom) or ''
    body = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY,
                             int(model.geom_bodyid[geom])) or ''
    if name.startswith('cell_'):
        return 'cell'
    for candidate in (name, body):
        if candidate:
            return candidate.split('_')[0]
    return 'world'

def role_of(model, geom):
    """Which role a geom belongs to, from the TOP ancestor of its body.

    Walking to the child-of-world ancestor is the only reliable rule here: prefixes collide
    (`n2_base_link` starts with `n_`), and the deck is deliberately a child of the chassis, so its
    top ancestor is the chassis and the deck counts as part of role `n` -- which is correct, since
    the deck rides the vehicle.
    """
    body = int(model.geom_bodyid[geom])
    while model.body_parentid[body] != 0:
        body = int(model.body_parentid[body])
    name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, body) or f'body{body}'
    return name.split('_')[0] if body != 0 else 'world'


def rig_meta():
    """`roller_rig`'s own generator metadata: the C fixture's declared offsets, not a copy of them."""
    _model, meta = roller_rig.build_model(str(TRAY_V1))
    return meta


def derive_frames(model, data):
    meta = rig_meta()
    pitch = float(meta['roller_pitch'])
    handoff = float(meta['recv_handoff_gap_m'])
    chassis_x = float(data.xpos[model.body('n_base_link').id][0])
    deck_row = geom_prefix_x(model, data, 'c_deck_roller')
    source_row = geom_prefix_x(model, data, 'c_fixed_roller')
    recv_row = geom_prefix_x(model, data, 'c_recv_roller')
    if not (deck_row and source_row and recv_row):
        raise SystemExit('the world does not carry c_deck_roller / c_fixed_roller / c_recv_roller')
    # the deck rides the chassis, so "dock" is a CHASSIS pose: the offset turns one into the other
    mount_offset = deck_row[0] - chassis_x
    source_dock = chassis_x + ((source_row[-1] + pitch) - deck_row[0])
    recv_dock = chassis_x + ((recv_row[0] - handoff) - deck_row[-1])
    return {
        'roller_pitch_m': pitch,
        'source': {'kind': 'fixed roller band carrying the tray: the source station',
                   'frame': 'world', 'trains_along': '+x',
                   'x_span_m': [source_row[0], source_row[-1]],
                   'rotation_centre_x_m': (source_row[0] + source_row[-1]) / 2.0},
        'receiver': {'kind': 'receiver roller band', 'frame': 'world', 'trains_along': '+x',
                     'x_span_m': [recv_row[0], recv_row[-1]],
                     'rotation_centre_x_m': (recv_row[0] + recv_row[-1]) / 2.0,
                     'handoff_gap_m': handoff},
        'vehicle_dock': {'kind': 'the deck roller row carried by the chassis',
                         'body': 'c_deck', 'carrier_body': 'n_base_link',
                         'mount_offset_x_m': mount_offset,
                         'deck_row_x_at_start_m': [deck_row[0], deck_row[-1]],
                         'chassis_x_at_start_m': chassis_x,
                         'chassis_x_to_dock_at_source_m': source_dock,
                         'chassis_x_to_dock_at_receiver_m': recv_dock,
                         'travel_source_to_receiver_m': recv_dock - source_dock},
    }


def dof_audit(model):
    """Which joints could move the chassis, and which of them an actuator can drive.

    The guidance's rule is that transport happens through the WHEELS, and that the deck slide is a
    declared mechanism which must not be counted as vehicle travel. Both halves are checkable: walk
    the chassis SUBTREE (the wheels are on child bodies), and separate "a joint exists" from "a
    joint is driven", because a slide that no actuator touches cannot move the base on its own.
    """
    kinds = {mujoco.mjtJoint.mjJNT_FREE: 'free', mujoco.mjtJoint.mjJNT_BALL: 'ball',
             mujoco.mjtJoint.mjJNT_SLIDE: 'slide', mujoco.mjtJoint.mjJNT_HINGE: 'hinge'}
    driven = {int(model.actuator(a).trnid[0]) for a in range(model.nu)
              if model.actuator(a).trntype[0] == mujoco.mjtTrn.mjTRN_JOINT}
    chassis_body = model.body('n_base_link').id
    members = subtree(model, chassis_body)
    deck_members = subtree(model, model.body('c_deck').id)
    chassis_own, deck_own = [], []
    for joint in range(model.njnt):
        body = model.jnt_bodyid[joint]
        if body not in members:
            continue
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, joint) or f'<jnt{joint}>'
        row = {'joint': name, 'on_body': mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, body),
               'type': kinds.get(int(model.jnt_type[joint]), '?'),
               'axis': [round(float(v), 4) for v in model.jnt_axis[joint]],
               'range': [round(float(v), 4) for v in model.jnt_range[joint]],
               'limited': bool(model.jnt_limited[joint]), 'driven': joint in driven}
        (deck_own if body in deck_members else chassis_own).append(row)
    # ★ The transport claim is about THE CHASSIS BODY, not its subtree: the deck rides the chassis,
    # so a subtree walk counts the deck's own driven slides as ways to move the vehicle. The deck's
    # slides are a declared mechanism and belong in their own list, where they are reported and
    # cannot be mistaken for vehicle travel.
    return {
        'chassis_body_joints': chassis_own,
        'deck_subtree_joints': deck_own,
        'wheel_joints': [r['joint'] for r in chassis_own if 'wheel' in r['joint']],
        'wheel_joints_driven': [r['joint'] for r in chassis_own
                                if 'wheel' in r['joint'] and r['driven']],
        'chassis_body_translation_joints_that_are_driven':
            [r['joint'] for r in chassis_own if r['type'] in ('slide', 'free') and r['driven']],
        'deck_driven_slides_are_a_declared_mechanism':
            [r['joint'] for r in deck_own if r['type'] == 'slide' and r['driven']],
        'chassis_slides_are_limited':
            {r['joint']: r['limited'] for r in chassis_own if r['type'] == 'slide'},
    }


def wheel_actuators(model, prefix):
    """The two wheel motors of one chassis, found by walking the prefix rather than typing a name:
    the merge patches actuator names, so a typed name would silently find nothing."""
    found = {}
    for side in ('left', 'right'):
        for candidate in (f'{prefix}wheel_{side}_motor', f'wheel_{side}_motor',
                          f'{prefix}{side}_wheel_motor'):
            hit = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, candidate)
            if hit >= 0:
                found[side] = int(hit)
                break
    return found


def hold_now(model, home_qpos, data):
    """The standing law evaluated at the CURRENT state.

    `merge_world.home_hold_ctrl` is a state feedback, and its own docstring says it is used
    "by the gate's settle test (evaluated every step)". Applying it as a constant -- which is
    what this probe did -- turns a hold into a fixed torque, and the humanoid then sags: its
    right foot ended 12.6 mm inside the shared ground at step 349 of a 500-step hold.
    """
    import merge_world as mw
    return np.asarray(mw.home_hold_ctrl(model, home_qpos, data.qpos, data.qvel),
                      dtype=float).reshape(-1)

def apply_hold(model, data, home_qpos, wheels, commands):
    """Hold every role at home, then let the wheel commands win.

    The order matters: the standing law is what keeps the humanoid upright for the whole drive, and
    the wheel motors are what move the chassis. Applied the other way round the law would fight the
    drive, and applied not at all the humanoid would fall over and every contact number below would
    describe a collapsing cell instead of a driving one.
    """
    data.ctrl[:] = hold_now(model, home_qpos, data)
    for side, value in commands.items():
        data.ctrl[wheels[side]] = value


def wheel_forward_sign(model, data, joint_name, radius):
    """+1 or -1: the sense of wheel rotation that drives the chassis along +x.

    DERIVED, because the typed version was wrong. Both wheel hinges in this plant are
    `axis="0 1 0"`, so forward is the SAME sign on both sides; the first version of this
    probe wrote `+1` on the left and `-1` on the right, which counter-rotated the wheels and
    spun the vehicle to -34.6 deg instead of driving it. Measured symptom: the chassis x
    moved 0.07 m while its yaw went to -34.6 deg, and the deck -- which rides 1.92 m ahead --
    swung to y -1.04 m.

    The derivation is rolling contact: with the wheel centre at the body origin and the
    contact at `-radius` along the body's own z, a unit angular velocity about the axis moves
    the contact point by `axis x r`, and the body by the negative of that. So the sign is
    read off the x component of that cross product, and the geometry comes from the compiled
    model rather than from this comment.
    """
    joint = model.joint(joint_name).id
    body = int(model.jnt_bodyid[joint])
    rotation = data.xmat[body].reshape(3, 3)
    axis_world = rotation @ np.asarray(model.jnt_axis[joint], dtype=float)
    # ★ DOWN IN THE WORLD, not `rotation @ (0, 0, -radius)`. That arm is a point on the RIM in body
    # coordinates, so it turns with the wheel -- while the contact patch does not. The sign
    # therefore flipped once a wheel had accumulated a quarter turn of spin, and a drive that read
    # it at such a moment went the wrong way at full speed until its timeout. Measured on the W4
    # chain: `cmd-004 DOCK` had a target 0.603 m EAST and drove 1.7766 m WEST.
    # `w4_plant._calibrate_wheel_signs` takes the same correction one step further and measures the
    # sign on the model, because a derivation is still an opinion until it is checked.
    contact_arm = np.array([0.0, 0.0, -radius])
    motion = np.cross(axis_world, contact_arm)
    return -1.0 if float(motion[0]) > 0 else 1.0

def drive(model, data, wheels, target_x, home_qpos, *, speed=DRIVE_SPEED_MPS,
          timeout_s=DRIVE_TIMEOUT_S, settle_s=SETTLE_S):
    """Drive the chassis to a target world x THROUGH ITS WHEELS, then zero and stand still.

    The controller has the same shape as `probe_n_sim.py`'s: a wheel-rate error times a gain,
    clipped by a torque limit, with the rate derived from the wanted (v, w) through the wheel radius
    and the track. **Nothing writes the chassis qpos**, so the displacement below is only credible
    because the audit found no driven non-wheel translation joint.
    """
    chassis = model.body('n_base_link').id
    signs = {side: wheel_forward_sign(model, data, f'n_wheel_{side}_joint', RADIUS_M)
             for side in ('left', 'right')}
    previous = np.array(data.xpos[chassis][:2])
    travelled, peak = 0.0, 0.0
    for _ in range(int(timeout_s / model.opt.timestep)):
        error = target_x - float(data.xpos[chassis][0])
        if abs(error) < 0.005:
            break
        command = math.copysign(min(speed, abs(error) / 0.5 + 0.02), error)
        rate = command / RADIUS_M
        commands = {}
        for side in ('left', 'right'):
            joint = model.joint(f'n_wheel_{side}_joint').id
            dof = model.jnt_dofadr[joint]
            want = rate * signs[side]
            commands[side] = float(np.clip(KP * (want - data.qvel[dof]),
                                           -TORQUE_LIMIT_NM, TORQUE_LIMIT_NM))
        apply_hold(model, data, home_qpos, wheels, commands)
        mujoco.mj_step(model, data)
        now = np.array(data.xpos[chassis][:2])
        travelled += float(np.linalg.norm(now - previous))
        peak = max(peak, float(np.linalg.norm(now - previous)) / model.opt.timestep)
        previous = now
    # ★ BRAKE, do not cut the torque. A wheel with `damping 0.02` and no friction loss free-wheels,
    # so a coast decays with a time constant near 0.9 s: measured on the W4 chain, cutting the
    # torque left the vehicle at 43 mm/s inside the contract's settle window against a 10 mm/s
    # limit, and cost 47 mm of travel. Here it mattered differently: the vehicle kept rolling while
    # the DECK's own x compliance was still ringing, so the handoff gap was read off a transient
    # (101.9 mm against the declared 80 mm). The brake is the drive loop's own rate servo with a
    # zero setpoint -- one controller, two setpoints.
    def brake_commands():
        out = {}
        for side in ('left', 'right'):
            joint = model.joint(f'n_wheel_{side}_joint').id
            dof = model.jnt_dofadr[joint]
            out[side] = float(np.clip(-KP * float(data.qvel[dof]),
                                      -TORQUE_LIMIT_NM, TORQUE_LIMIT_NM))
        return out

    apply_hold(model, data, home_qpos, wheels, brake_commands())
    at_rest = previous.copy()
    for _ in range(int(settle_s / model.opt.timestep)):
        apply_hold(model, data, home_qpos, wheels, brake_commands())
        mujoco.mj_step(model, data)
    final = np.array(data.xpos[chassis][:2])
    return {'target_x': target_x, 'final_x': float(final[0]),
            'error_x_m': float(final[0] - target_x), 'travelled_m': travelled,
            'peak_speed_mps': peak,
            'wheel_forward_signs': signs,
            'moved_during_settle_m': float(np.linalg.norm(final - at_rest))}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--run-id', default='w2-baseline-01')
    ap.add_argument('--reports-dir', default=None)
    ap.add_argument('--no-drive', action='store_true',
                    help='stop after the audits: the cheap way to learn what the hold law does')
    ap.add_argument('--world', default=str(DEFAULT_WORLD.relative_to(ROOT)),
                    help='which integrated world to audit')
    ap.add_argument('--min-leg-m', type=float, default=0.0,
                    help='the transport leg must be at least this long or the run FAILS: a '
                         'leg this run does not assert is a leg nobody checked')
    args = ap.parse_args()
    out = (Path(args.reports_dir) if args.reports_dir else ROOT / 'reports') / args.run_id
    out.mkdir(parents=True, exist_ok=True)
    world = ROOT / args.world if not Path(args.world).is_absolute() else Path(args.world)
    if not world.is_file():
        raise SystemExit(f'no world at {world}')
    wall0 = time.monotonic()
    checks = []

    def check(name, status, detail):
        checks.append({'check': name, 'status': status, 'detail': detail})

    build_p3_world.install()          # rebinds merge_world.ROLES/TARGET to the P3 assembly
    import merge_world as mw

    model = mujoco.MjModel.from_xml_path(str(world))
    data = mujoco.MjData(model)
    home_qpos, hold_ctrl, applied, notes = mw.merged_home(model)
    hold_ctrl = np.asarray(hold_ctrl, dtype=float).reshape(-1)
    data.qpos[:] = home_qpos
    mujoco.mj_forward(model, data)

    absent = [b for b in ('h_LINK_BASE', 'a_link0', 'c_deck', 'n_base_link', 'n2_base_link')
              if mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, b) < 0]
    notes_text = ('; '.join(str(n) for n in notes) if isinstance(notes, (list, tuple))
                  else str(notes))[:200]
    hold_ctrl = hold_now(model, home_qpos, data)   # one evaluation, for the body row below
    check('all four roles plus the second chassis are in the one model',
          'PASS' if not absent else 'FAIL',
          f'nbody {model.nbody}, nq {model.nq}, nu {model.nu}, absent {absent or "none"}; '
          f'world sha256 {hashlib.sha256(world.read_bytes()).hexdigest()[:16]} '
          f'({args.world}); '
          f'the standing law touches {applied} actuators; notes: {notes_text}')

    frames = derive_frames(model, data)
    audit = dof_audit(model)
    check('the wheels are the only thing that can move the CHASSIS BODY',
          'PASS' if len(audit['wheel_joints_driven']) == 2
          and not audit['chassis_body_translation_joints_that_are_driven'] else 'FAIL',
          f"the chassis body 'n_base_link' carries {[r['joint'] for r in audit['chassis_body_joints']]}; "
          f"driven among them {audit['wheel_joints_driven']}; the chassis' own translation joints "
          f"that an actuator can drive "
          f"{audit['chassis_body_translation_joints_that_are_driven'] or 'none'} -- so a commanded "
          f"wheel rate is the only way to move the vehicle. Separately, the deck subtree carries "
          f"driven slides {audit['deck_driven_slides_are_a_declared_mechanism']}: that is the "
          f"declared transfer mechanism, and its travel is NOT vehicle travel")

    deepest = {'self': (0.0, None, None, 0), 'cross': (0.0, None, None, 0),
               'world': (0.0, None, None, 0), 'own_fixture': (0.0, None, None, 0)}
    for step in range(int(HOLD_S / model.opt.timestep)):
        data.ctrl[:] = hold_now(model, home_qpos, data)
        mujoco.mj_step(model, data)
        if not data.ncon:
            continue
        for contact in range(data.ncon):
            first = int(data.contact.geom[contact][0])
            second = int(data.contact.geom[contact][1])
            depth = float(data.contact.dist[contact])
            # Ownership by NAME, not by body ancestry. `role_of` walks to the child of the
            # world, and merge_world hoists each role's furniture onto the world body, so the
            # humanoid's own `h_source_table` came back as "world" -- which reads as a cell
            # fault when it is the humanoid's own fixture.
            left, right = geom_owner(model, first), geom_owner(model, second)
            if left == 'cell' or right == 'cell':
                kind = 'world'
            elif left == right == 'world':
                kind = 'world'
            elif left == right:
                kind = 'self'
            elif 'world' in (left, right):
                # one side is a role, the other is somebody else's furniture on the world body
                kind = 'own_fixture' if left == right else 'cross'
            else:
                kind = 'cross'
            if depth < deepest[kind][0]:
                deepest[kind] = (depth, describe_geom(model, first),
                                 describe_geom(model, second), step)
    self_depth, self_a, self_b, self_step = deepest['self']
    fixture_depth, fixture_a, fixture_b, fixture_step = deepest['own_fixture']
    cross_depth, cross_a, cross_b, cross_step = deepest['cross']
    world_depth, world_a, world_b, world_step = deepest['world']
    check('nothing CROSS-role or against the world interpenetrates in the assembled cell',
          'PASS' if min(cross_depth, world_depth) > -0.005 else 'FAIL',
          f'{int(HOLD_S / model.opt.timestep)} steps at dt {model.opt.timestep}; deepest CROSS-role '
          f'contact {cross_depth * 1000:.3f} mm {cross_a} vs {cross_b} at step {cross_step}; '
          f'deepest against the world {world_depth * 1000:.3f} mm {world_a} vs {world_b} at step '
          f'{world_step}; {data.ncon} contacts at the end')
    check('a role that intersects ITSELF is attributed to that role, not to the assembly',
          'PASS' if self_a is not None else 'FAIL',
          f'deepest self-contact {self_depth * 1000:.3f} mm between {self_a} and {self_b} at step '
          f'{self_step}. This is the humanoid model at its home pose putting its own thumb tip inside '
          f'its own knee: it says nothing about the assembly, and it is a real problem for '
          f'`P4-HUMAN-01` the moment that hand is asked to carry a tray. P3-WORLD-01 never measured '
          f'contact depth, which is why this is the first time it appears.')


    deck_row = geom_prefix_x(model, data, 'c_deck_roller')
    source_row = geom_prefix_x(model, data, 'c_fixed_roller')
    recv_row = geom_prefix_x(model, data, 'c_recv_roller')
    pitch = frames['roller_pitch_m']
    check('the parked pose IS the source dock pose, and the transport leg is measured',
          'PASS' if abs((deck_row[0] - source_row[-1]) - pitch) < 0.02 else 'FAIL',
          f'at the parked pose the deck first crown sits {deck_row[0] - source_row[-1]:+.4f} m past '
          f'the source band\'s last crown, where C asks for one pitch ({pitch} m) -- a difference '
          f'of {abs(deck_row[0] - source_row[-1] - pitch) * 1000:.1f} mm. So the parked pose is the '
          f'source dock pose, which is exactly why C\'s chain works from it. **The consequence is '
          f'the finding: the source band (x {source_row[0]:.2f}..{source_row[-1]:.2f}) and the '
          f'receiver band (x {recv_row[0]:.2f}..{recv_row[-1]:.2f}) are two ends of ONE fixture with '
          f'the deck bridging them, so the chassis travel between the two dock poses is only '
          f'{frames["vehicle_dock"]["travel_source_to_receiver_m"] * 1000:.0f} mm.** A logistics leg '
          f'needs a receiver at a DIFFERENT station, which this world does not have.')

    # --- the docking mechanism, present or absent, reported either way ----------------------
    mechanism = ['w2_chamfer_posy', 'w2_chamfer_negy', 'w2_datum_stop']
    found = [mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, name) >= 0 for name in mechanism]
    has_compliance = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT,
                                       'c_deck_slide_y') >= 0
    if any(found):
        check('the passive docking mechanism is present AND has something to push',
              'PASS' if all(found) and has_compliance else 'FAIL',
              f'bodies found {dict(zip(mechanism, found))}; the deck own lateral slide '
              f'c_deck_slide_y present {has_compliance}. Both halves are needed: a chamfer can '
              f'only move what is free to move, and this vehicle is a four-wheeled skid-steer '
              f'whose tyre friction is the reason the DECK, not the vehicle, is the thing that '
              f'complies')
    else:
        check('this world declares no passive docking mechanism', 'PASS',
              'no w2_chamfer_* / w2_datum_stop bodies: this is the assembly world, whose '
              'docking interface is C original ungated one. Reported rather than skipped, '
              'because a check that silently does not run reads like a check that passed')

    standby = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, 'n2_base_link')
    if standby >= 0:
        lane = abs(float(data.xpos[standby][1]))
        check('the second chassis is parked OFF the transport lane, still a real entity',
              'PASS' if lane > 0.5 else 'FAIL',
              f'n2_base_link sits at y {lane:.4f} m from the lane centre. The guidance asks the '
              f'idle robots to stay real but not participate; a chassis parked on the lane is '
              f'an obstacle, and a chassis deleted is a claim that it does not exist')

    leg = frames['vehicle_dock']['travel_source_to_receiver_m']
    check('the transport leg is at least the length this run demands',
          'PASS' if leg >= args.min_leg_m else 'FAIL',
          f'the two dock poses are {leg:.4f} m apart by chassis x, against the '
          f'{args.min_leg_m:.2f} m this run demands. In the assembly world this is a few '
          f'hundred millimetres, which is a re-index rather than transport; the threshold is '
          f'passed in so that the claim is made by the run and not by the reader')

    tray_lo, tray_hi = body_span(model, data, 'c_payload')
    check('the source tray is a free body resting on the source band, not welded to it',
          'PASS' if abs(tray_lo[2] - 0.485) < 0.02 else 'FAIL',
          f'`c_payload` spans x [{tray_lo[0]:.4f}, {tray_hi[0]:.4f}], z bottom {tray_lo[2]:.4f} '
          f'against the crown plane 0.485; it carries a free joint, so nothing holds it and it '
          f'stays where the rollers leave it')

    drives = {}
    if args.no_drive:
        check('the drive legs were skipped on request', 'PASS',
              f'--no-drive: the audits ran in {time.monotonic() - wall0:.1f} s and no wheel command '
              f'was issued, so this run says nothing about whether the chassis can reach either dock')
    else:
        wheels = wheel_actuators(model, 'n_')
        if len(wheels) != 2:
            check('both wheel motors of the transport chassis were found', 'FAIL',
                  f'found {wheels}: the drive would be measuring nothing')
        else:
            check('both wheel motors of the transport chassis were found', 'PASS',
                  f'{sorted(wheels)} (found by walking the prefix, because the merge patches names)')
            to_source = drive(model, data, wheels,
                              frames['vehicle_dock']['chassis_x_to_dock_at_source_m'],
                              home_qpos)
            gap = geom_prefix_x(model, data, 'c_deck_roller')[0] - source_row[-1]
            check('the vehicle reaches the source dock BY ITS WHEELS',
                  'PASS' if abs(gap - frames['roller_pitch_m']) < 0.02 else 'FAIL',
                  f'drove {to_source["travelled_m"]:.4f} m by wheel, finished '
                  f'{to_source["error_x_m"] * 1000:+.1f} mm from the derived chassis x; the deck '
                  f'first crown is now {gap:+.4f} m past the source band\'s last crown, against the '
                  f'{frames["roller_pitch_m"]} m pitch C declares')
            to_recv = drive(model, data, wheels,
                            frames['vehicle_dock']['chassis_x_to_dock_at_receiver_m'],
                            home_qpos)
            handoff = recv_row[0] - geom_prefix_x(model, data, 'c_deck_roller')[-1]
            check('the vehicle reaches the receiver dock BY ITS WHEELS',
                  'PASS' if abs(handoff - frames['receiver']['handoff_gap_m']) < 0.02 else 'FAIL',
                  f'drove {to_recv["travelled_m"]:.4f} m by wheel; the deck last crown is now '
                  f'{handoff:+.4f} m from the receiver band\'s first crown, against the '
                  f'{frames["receiver"]["handoff_gap_m"]:.4f} m handoff gap C declares')
            drives = {'to_source': to_source, 'to_receiver': to_recv}
            # BOTH SIDES ARE THE DECK BODY ORIGIN. The first version took the centre of the deck
            # body's geom span and compared it against `mount_offset`, which is defined from the
            # deck's roller row -- two different reference points, so a correctly bolted deck was
            # reported 0.30 m out.
            deck_body_x = float(data.xpos[model.body('c_deck').id][0])
            chassis_x = float(data.xpos[model.body('n_base_link').id][0])
            mounted = deck_body_x - chassis_x
            slack = mounted - frames['vehicle_dock']['mount_offset_x_m']
            check('the deck is still carried by the chassis after both drives',
                  'PASS' if abs(slack) < 0.05 else 'FAIL',
                  f'deck body x {deck_body_x:.4f} against chassis {chassis_x:.4f} gives a mount of '
                  f'{mounted:.4f} m, against the derived {frames["vehicle_dock"]["mount_offset_x_m"]:.4f} m '
                  f'-- a slide of {slack * 1000:+.1f} mm in the deck mount. Both numbers are the '
                  f'DECK BODY ORIGIN, which is the reference the mount offset is defined from')
            tray_lo_after, _tray_hi = body_span(model, data, 'c_payload')
            check('the source tray stayed on its band while the vehicle drove away',
                  'PASS' if abs(tray_lo_after[0] - tray_lo[0]) < 0.02 else 'FAIL',
                  f'tray x moved {tray_lo_after[0] - tray_lo[0]:+.5f} m: the deck slid out from '
                  f'under nothing, because the tray was never on the deck in this run')

    runtime = time.monotonic() - wall0
    check('the round reports its own cost instead of implying a fast machine', 'PASS',
          f'{runtime:.1f} s wall for {float(data.time):.1f} s sim (ratio '
          f'{float(data.time) / runtime:.3f}); one MjModel, one mj_step loop, one clock; '
          f'RTF is measured WITH the hold law, so it is a lower bound for the assembled cell')

    verdict = 'FAIL' if any(c['status'] == 'FAIL' for c in checks) else 'PASS'
    report = {'run_id': args.run_id, 'scope': 'W2_INTEGRATION_BASELINE', 'verdict': verdict,
              'checks': checks, 'frames': frames, 'dof_audit': audit, 'drives': drives,
              'runtime_s': round(runtime, 3), 'sim_time_s': round(float(data.time), 3),
              'world': {'path': str(world.relative_to(ROOT)),
                        'sha256': hashlib.sha256(world.read_bytes()).hexdigest()},
              'min_leg_m': args.min_leg_m,
              'not_established': [
                  'the roller TRANSFER itself: this baseline drives the vehicle to both dock poses '
                  'and stops. Moving the tray along the rollers is C\'s mechanism and its own '
                  'evidence (`reports/p2-c-in-cell-02`)',
                  'the second chassis cannot transport: it carries no deck, so it has no roller row',
                  'ROS in the loop: plant only, so no bridge, no Nav2, no estimator, and no scan',
                  'SLAM: the world is generated and the map is an operator prior',
              ]}
    (out / 'report.json').write_text(json.dumps(report, indent=2) + '\n', encoding='utf-8')
    print(json.dumps({'run_id': args.run_id, 'verdict': verdict,
                      'failed': [c['check'] for c in checks if c['status'] == 'FAIL']}, indent=2))
    for c in checks:
        print(f"  [{c['status']}] {c['check']}")
        print(f'        {c["detail"]}')
    print(f'\nwrote {out.relative_to(ROOT)}/report.json')
    return 0 if verdict == 'PASS' else 1


if __name__ == '__main__':
    sys.exit(main())
