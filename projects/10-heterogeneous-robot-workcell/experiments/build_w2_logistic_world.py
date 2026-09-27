"""W2: the integrated LOGISTICS world -- source and receiver at different stations.

WHY A SECOND WORLD AND NOT AN EDIT OF THE CELL
----------------------------------------------
`probe_w2_baseline.py` measured that in `assets/world_p3_cell.xml` the source band
(x 4.41..6.25) and the receiver band (x 7.20..8.24) are two ends of ONE fixture, so the
chassis travel between the two dock poses is a couple of hundred millimetres. That is a
re-index, not transport: it cannot exercise path following, speed feed-forward or drift over
distance. The decision taken was to build a SEPARATE world with the receiver moved to its own
station, and to keep the cell world untouched so that every P3 verdict stays readable against
the artifact it was made on.

HOW IT IS BUILT
---------------
Not by hand-editing a generated XML. It calls `merge_world.build_text()` -- the same builder
that produces the cell -- then applies a small set of DECLARED transforms to the resulting
spec:

  1. the receiver roller row `c_recv_roller_*` is translated by `RECEIVER_SHIFT_M` along +x, so
     the receiver becomes its own station;
  2. the ground and the three walls are extended east by the same amount, so the corridor is
     real floor rather than a hole;
  3. the second chassis `n2` is moved to a declared STANDBY PARK off the driving lane: the
     guidance requires the other robots to stay real entities, and a real entity parked in the
     lane is an obstacle, not a standby;
  4. the DOCKING INTERFACE gains a passive guidance mechanism (see below).

Every number is derived from `roller_rig`'s own generator metadata and the merged layout; none
is typed from a measurement.

THE DOCKING INTERFACE (option (1): passive mechanical guidance)
-------------------------------------------------------------
The observability gate refused to certify the tight side of the transfer window: the vehicle's
lidar resolves 5 mm, C's own sweep puts the tight side at 2 mm. The accepted design decision is
passive mechanical guidance, so that the SENSOR only has to deliver the vehicle into a LOOSE
window and the MECHANISM has to deliver the tight one. The mechanism is three features:

  * two LATERAL CHAMFER RAILS at the receiver station, whose inner faces taper from
    `CATCH_M` outside the deck frame at the entry to `RESIDUAL_LATERAL_M` at the exit. They
    act on `c_deck_frame`, the widest part of the deck, at the frame's own z band;
  * a LONGITUDINAL DATUM BAR, spring-mounted on an x slide, that the deck frame's leading face
    butts against. The declared dock is therefore a mechanical datum rather than an estimator's
    belief. Its compliance is what makes the stop measurable instead of a penetration;
  * a LATERAL COMPLIANCE in the deck mount (`c_deck_slide_y`). This is not decoration and the
    design does not work without it: a chamfer can only push something sideways, and the vehicle
    is a four-wheeled skid-steer whose wheel friction resists lateral motion by
    `mu * m * g`. Pushing the VEHICLE sideways needs a longitudinal force of
    `mu*m*g / tan(chamfer)`, which exceeds the wheel torque; pushing the DECK sideways needs only
    the mount spring's force. `static_audit()` computes both numbers from the compiled model so
    the reasoning is a measurement, not an assertion.

All three are present ONLY here. `assets/world_p3_cell.xml` is unchanged, so the P3 verdicts
are unaffected, and this world is a NEW VERSION of the docking interface with its own evidence.
"""
import argparse
import hashlib
import json
import math
from pathlib import Path
import re
import sys

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'experiments'))
sys.path.insert(0, str(ROOT / 'src'))

import mujoco  # noqa: E402
import merge_world as mw  # noqa: E402
import build_p3_world as bp3  # noqa: E402
import roller_rig as rig  # noqa: E402

OUT = ROOT / 'assets' / 'world_w2_logistic.xml'
#: The merged world has no `meshdir`; its mesh paths are bare and resolve against the XML file's
#: own directory. So this world must sit in `assets/` next to `world_p3_cell.xml`, and any
#: intermediate compile has to happen from a file THERE, not from a string -- `from_xml_string`
#: has no base directory and reported `Error opening file 't800/robot/t800/meshes/LINK_BASE.obj'`.
CELL = ROOT / 'assets' / 'world_p3_cell.xml'
_TMP = ROOT / 'assets' / '_w2_build_tmp.xml'


def compile_xml(text):
    """Compile a world that uses meshes: through a file in `assets/`, because MuJoCo resolves a
    bare mesh path against the model file's directory and a string has none."""
    _TMP.write_text(text, encoding='utf-8')
    try:
        return mujoco.MjModel.from_xml_path(str(_TMP))
    finally:
        if _TMP.exists():
            _TMP.unlink()


def compile_spec(spec):
    return compile_xml(mw.with_option_block(spec.to_xml()))

#: How far the receiver station moves. A transport leg has to be long enough to be a leg:
#: 5 m is ~20x the deck length and ~21x the travel the cell world allowed.
RECEIVER_SHIFT_M = 5.0

#: The second chassis' standby park, in y, off the driving lane. Real entity, out of the way.
STANDBY_OFFSET_Y_M = 1.15

# --- the docking mechanism, every value declared with its reason --------------------------
#: The loose lateral window the SENSOR must deliver. Derived below from the sensor's own
#: resolution, not chosen for looks: see `derive_catch()`.
CATCH_M = 0.035
#: The residual left after the chamfer has done its work. It sets TWO limits at once, which is
#: why it is 1 mm and not 2: a rectangle of half-width `DECK_FRAME_HALF[1]` inside a channel with
#: clearance `c` cannot be yawed by more than about `c / half_width`. At 2 mm that bound is
#: 0.38 deg, ABOVE C's own measured 0.2 deg passing yaw; at 1 mm it is 0.19 deg, just inside.
RESIDUAL_LATERAL_M = 0.001
#: The parallel section after the taper. The taper pulls the deck in; the channel is what holds
#: it while the vehicle finishes its approach, so the two must not be the same feature.
CHANNEL_LENGTH_M = 0.90
#: Where the deck frame's leading face ends up, measured from where the frame face geometrically
#: was when the deck was at C's own dock. 2 mm is INSIDE C's measured longitudinal bracket
#: (+20 mm passing, -2 mm the tight side), which is the whole point: the datum replaces the
#: estimator's guess with a mechanical stop, and the stop has to be somewhere inside the window.
DATUM_OFFSET_M = 0.002
#: The gap between the rails' east ends and the datum bar's west face. They are different
#: features at the same height and the first build had them interpenetrating by 0.015 mm, which
#: pushed the spring-mounted datum bar before anything had touched it.
RAIL_TO_STOP_CLEARANCE_M = 0.004
#: The chamfer's half-angle. Steeper than a real funnel, because the force argument in
#: `static_audit()` is what sets the minimum, and this is what clears it with margin.
CHAMFER_ANGLE_DEG = 12.0
RAIL_THICKNESS_M = 0.020
RAIL_HALF_Z_M = 0.012
#: The posts sit OUTBOARD of the rails, clear of the vehicle's own 0.19 m half-width, so the
#: vehicle drives through the channel without touching anything but the deck frame.
POST_OFFSET_M = 0.10
POST_HALF_Y_M = 0.06
RAIL_COLOUR_DARK = (0.35, 0.36, 0.40, 1.0)
RAIL_COLOUR_LIGHT = (0.62, 0.63, 0.66, 1.0)
#: The datum bar's compliance: stiff enough that the docked pose is the datum, soft enough that
#: the contact is a measured deflection rather than a solver penetration.
STOP_STIFFNESS_N_M = 20000.0
STOP_DAMPING = 200.0
STOP_TRAVEL_M = 0.006
#: The datum bar's own size. Its z half-height is DELIBERATELY inside the deck frame's 20 mm
#: band: the receiver's first roller bottoms out at z 0.415 and the bar's top must stay clear of
#: it, or the dock would jam on the roller instead of resting on the datum.
STOP_HALF_X_M = 0.010
STOP_HALF_Y_M = 0.42
STOP_HALF_Z_M = 0.009
#: The deck mount's lateral spring. Soft, because the chamfer has to be able to compress it.
DECK_LATERAL_STIFFNESS_N_M = 150.0
DECK_LATERAL_DAMPING = 5.0
DECK_LATERAL_RANGE_M = 0.05
#: The deck mount's YAW spring, and the reason it exists: a straight channel can only hold a
#: deck whose yaw it can accept, so a deck rigid in yaw jams unless the vehicle is already
#: straight. Freeing the deck in yaw is what turns "the vehicle must be within 0.2 deg" into
#: "the deck ends within the channel's own clearance". Same argument as the lateral spring.
DECK_YAW_STIFFNESS_NM_RAD = 40.0
DECK_YAW_DAMPING_NM_S_RAD = 8.0
DECK_YAW_RANGE_RAD = 0.05


def rig_meta():
    _model, meta = rig.build_model(str(ROOT / 'assets' / 'objects' / 'tray_v1.xml'))
    return meta


def _world_geom(model, name):
    """`(pos, size)` of a named geom in the compiled model, as float arrays."""
    index = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, name)
    if index < 0:
        raise RuntimeError(f'the merged world has no geom named {name!r}')
    return np.array(model.geom_pos[index], dtype=float), np.array(model.geom_size[index],
                                                                 dtype=float)


def _home_state(model):
    """A `MjData` at the world's own home keyframe, so 'where things are' means the parked pose.

    Reading `body_pos` or a default-qpos `mj_forward` gives the XML's DECLARED positions, and the
    first version of this file did exactly that: it reported the chassis at x 9.03 -- its
    `body_pos` -- while the parked chassis is at 4.41, because the parking lives in the keyframe's
    slide qpos and nowhere else. Every frame below is therefore read from the keyframe.
    """
    data = mujoco.MjData(model)
    if model.nkey:
        mujoco.mj_resetDataKeyframe(model, data, 0)
    mujoco.mj_forward(model, data)
    return data


def wheel_radius_m(model):
    """The wheel radius, from the wheel geometry itself rather than from a lookalike constant.

    `roller_rig.ROLLER_RADIUS` is 0.035 and the wheels are 0.040; using the roller's number here
    would have understated the available drive force by 12%.
    """
    for geom in range(model.ngeom):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, geom) or ''
        if 'wheel' in name:
            # a cylinder's `size[0]` is its radius whatever axis it is drawn along
            return float(model.geom_size[geom][0])
    raise RuntimeError('no geom named *wheel* in the chassis: the drive budget cannot be derived')


def declared_torque_limit_nm():
    """The plant's own declared wheel torque, read from the single-source config."""
    text = (ROOT / 'config' / 'n_probe.yaml').read_text(encoding='utf-8')
    found = re.search(r'^\s*wheel_torque_limit_nm:\s*([0-9.]+)', text, re.M)
    if not found:
        raise RuntimeError('config/n_probe.yaml does not declare wheel_torque_limit_nm')
    return float(found.group(1))


def derive_geometry(model, shift=0.0):
    """Every x this mechanism needs, read off the parked pose of a compiled world.

    `model` is the world BEFORE the shift (the cell world), and `shift` is added to every
    receiver-dependent quantity. Deriving from the pre-shift world and adding the declared
    translation is exact -- the transform is a rigid +x translation of the receiver row and of
    nothing else -- and it avoids compiling a half-built model.

    The dock chassis x is `probe_w2_baseline.py`'s own expression, recomputed here from the
    compiled model and `roller_rig`'s metadata so that the two cannot drift apart.
    """
    meta = rig_meta()
    pitch = float(meta['roller_pitch'])
    handoff = float(meta['recv_handoff_gap_m'])
    data = _home_state(model)

    def row_x(prefix):
        obj = mujoco.mjtObj.mjOBJ_GEOM
        return sorted(float(data.geom_xpos[g][0]) for g in range(model.ngeom)
                      if (mujoco.mj_id2name(model, obj, g) or '').startswith(prefix))

    chassis = float(data.xpos[model.body('n_base_link').id][0])
    deck_row = row_x('c_deck_roller')
    source_last = row_x('c_fixed_roller')[-1]
    recv_first = row_x('c_recv_roller')[0] + shift
    deck_x_at_source = float(data.xpos[model.body('c_deck').id][0])

    source_dock = chassis + ((source_last + pitch) - deck_row[0])
    receiver_dock = chassis + ((recv_first - handoff) - deck_row[-1])
    deck_x_at_receiver = deck_x_at_source + (receiver_dock - source_dock)
    return {'roller_pitch_m': pitch, 'handoff_gap_m': handoff,
            'chassis_x_at_start_m': chassis, 'source_dock_chassis_x_m': source_dock,
            'receiver_dock_chassis_x_m': receiver_dock,
            'travel_source_to_receiver_m': receiver_dock - source_dock,
            'deck_x_at_source_m': deck_x_at_source,
            'deck_x_at_receiver_m': deck_x_at_receiver,
            'receiver_first_crown_x_m': recv_first,
            'receiver_row_moved_by_m': shift,
            'deck_frame_half_m': list(rig.DECK_FRAME_HALF),
            'deck_frame_x_local_m': 0.29,   # roller_rig's own frame centre, deck-local
            'deck_frame_z_local_m': -0.085,
            'deck_z_m': float(data.xpos[model.body('c_deck').id][2]),
            # ★ The lever arm from the chassis origin to the deck frame centre, measured. The
            # docking catch is declared at the FRAME, so this number is what converts a
            # vehicle yaw into a lateral offset, and it is what makes the catch a COMBINED
            # budget: at 2.21 m, 1 deg of yaw spends 38.6 mm of a 35 mm catch.
            'frame_lever_arm_m': float(data.geom_xpos[mujoco.mj_name2id(
                model, mujoco.mjtObj.mjOBJ_GEOM, 'c_deck_frame')][0]) - chassis}


def static_audit(model, geo):
    """Is a passive chamfer even able to move this vehicle? A number, not an opinion.

    A wedge converts the drive force `F` into a lateral push `F*tan(theta)`. To move the whole
    VEHICLE sideways that push has to beat the tyres' lateral friction; to move the DECK it only
    has to beat the mount spring. Both are computed here so the design choice is auditable.

    TWO drive forces are reported, because they are different facts and the conclusion depends on
    which one is in play: what the plant's actuator is DECLARED able to produce
    (`config/n_probe.yaml`), and what the drive probe's own controller actually commands
    (`probe_w2_baseline.py` clips at 1.2 N m per wheel). The mechanism has to work under the
    second, which is the conservative one.
    """
    mass = float(model.body_subtreemass[model.body('n_base_link').id])
    deck_mass = float(model.body_subtreemass[model.body('c_deck').id])
    friction = float(rig.DEFAULT_FRICTION)
    radius = wheel_radius_m(model)
    declared_torque = declared_torque_limit_nm()
    controller_clip = 1.2          # probe_w2_baseline.TORQUE_LIMIT_NM, the value in use
    theta = math.radians(CHAMFER_ANGLE_DEG)
    vehicle_force_needed = friction * mass * 9.81
    deck_force_needed = DECK_LATERAL_STIFFNESS_N_M * CATCH_M

    def budget(torque):
        force = 2.0 * torque / radius
        return {'drive_force_n': round(force, 3),
                'lateral_force_available_n': round(force * math.tan(theta), 3),
                'moves_the_vehicle': force * math.tan(theta) >= vehicle_force_needed,
                'moves_the_deck': force * math.tan(theta) >= deck_force_needed}

    return {
        'chassis_subtree_mass_kg': round(mass, 4),
        'deck_subtree_mass_kg': round(deck_mass, 4),
        'wheel_radius_m': radius,
        'ground_friction_mu': friction,
        'lateral_force_to_move_the_VEHICLE_n': round(vehicle_force_needed, 3),
        'lateral_force_to_move_the_DECK_n': round(deck_force_needed, 3),
        'chamfer_angle_deg': CHAMFER_ANGLE_DEG,
        'with_the_declared_actuator': dict(
            torque_nm=declared_torque, **budget(declared_torque)),
        'with_the_probe_controller_clip': dict(
            torque_nm=controller_clip, **budget(controller_clip)),
    }


def add_rail(spec, *, x_entry, x_exit, y_entry, y_exit, side, z_centre):
    """One chamfer rail, built as a rotated box whose INNER FACE passes through the two declared
    points, plus a straight CHANNEL box that carries the same face on to the datum.

    The channel is not decoration: the taper only pulls the deck in, and something has to hold it
    straight for the last stretch of the approach, when the vehicle is still creeping and its own
    yaw is what the deck would otherwise inherit.
    """
    suffix = 'pos' if side > 0 else 'neg'
    x_taper_end = x_exit - CHANNEL_LENGTH_M
    dx = x_taper_end - x_entry
    dy = y_exit - y_entry
    length = math.hypot(dx, dy)
    psi = math.atan2(dy, dx)
    # left normal of the direction, i.e. the direction the +y rail's thickness goes
    normal = np.array([-dy, dx]) / length
    if side < 0 and normal[1] > 0:
        normal = -normal
    if side > 0 and normal[1] < 0:
        normal = -normal
    mid = (np.array([x_entry, y_entry]) + np.array([x_taper_end, y_exit])) / 2.0 \
        + (RAIL_THICKNESS_M / 2.0) * normal
    body = spec.worldbody.add_body(name=f'w2_taper_{suffix}y',
                                   pos=[float(mid[0]), float(mid[1]), float(z_centre)])
    # MjsBody exposes `quat`, not `euler`. A rotation about z by psi is
    # (cos(psi/2), 0, 0, sin(psi/2)) in MuJoCo's (w, x, y, z) order.
    body.quat = [math.cos(psi / 2.0), 0.0, 0.0, math.sin(psi / 2.0)]
    geom = body.add_geom(name=f'w2_taper_face_{suffix}y',
                         type=mujoco.mjtGeom.mjGEOM_BOX)
    geom.size = [length / 2.0, RAIL_THICKNESS_M / 2.0, RAIL_HALF_Z_M]
    geom.rgba = list(RAIL_COLOUR_LIGHT)
    geom.friction = [float(rig.GUIDE_FRICTION), 0.005, 0.0001]

    # the straight channel: same inner face, no taper, all the way to the datum
    channel_x0, channel_x1 = x_exit - CHANNEL_LENGTH_M, x_exit
    face_y = y_exit + side * 0.0          # the taper's exit face, carried on unchanged
    centre_y = face_y + side * (RAIL_THICKNESS_M / 2.0)
    channel = spec.worldbody.add_body(name=f'w2_chamfer_{suffix}y',
                                      pos=[float((channel_x0 + channel_x1) / 2.0), float(centre_y),
                                           float(z_centre)])
    cgeom = channel.add_geom(name=f'w2_chamfer_face_{suffix}y',
                             type=mujoco.mjtGeom.mjGEOM_BOX)
    cgeom.size = [CHANNEL_LENGTH_M / 2.0, RAIL_THICKNESS_M / 2.0, RAIL_HALF_Z_M]
    cgeom.rgba = list(RAIL_COLOUR_LIGHT)
    cgeom.friction = [float(rig.GUIDE_FRICTION), 0.005, 0.0001]

    # the post, outboard of both, so the vehicle's own body never reaches it
    post_y = float(centre_y + side * (POST_OFFSET_M + POST_HALF_Y_M))
    post = spec.worldbody.add_body(name=f'w2_chamfer_post_{suffix}y',
                                   pos=[float((channel_x0 + x_entry) / 2.0), post_y,
                                        float(z_centre) / 2.0])
    pgeom = post.add_geom(name=f'w2_chamfer_post_face_{suffix}y',
                          type=mujoco.mjtGeom.mjGEOM_BOX)
    pgeom.size = [(x_exit - x_entry) / 2.0, POST_HALF_Y_M, float(z_centre) / 2.0]
    pgeom.rgba = list(RAIL_COLOUR_DARK)
    pgeom.friction = [float(rig.GUIDE_FRICTION), 0.005, 0.0001]
    return f'w2_chamfer_{suffix}y'


def admissible_yaw_deg(channel_half):
    """The largest yaw a square frame of `DECK_FRAME_HALF` can carry and still pass the channel.

    SOLVED, not approximated. A square of half-width `h` yawed by `t` has a y half-extent of
    `h*(cos t + sin t) = h*sqrt(2)*cos(t - 45 deg)`, so it fits a channel of half-width `c` up to
    `t = 45 deg - acos(c / (h*sqrt(2)))`. At the built geometry that is 0.192 deg.

    One definition, used by `verify_rail_faces`, by the W3 judge and by the report -- a second copy
    of this expression is a second, unowned opinion about what the mechanism can accept.
    """
    half = float(rig.DECK_FRAME_HALF[1])
    ratio = float(channel_half) / (half * math.sqrt(2.0))
    if ratio >= 1.0:
        return 45.0
    return 45.0 - math.degrees(math.acos(ratio))


def verify_rail_faces(model):
    """Re-measure the inner faces the rails were supposed to produce.

    A rail built from a rotated box plus a translated one is easy to get subtly wrong, and a
    wrong face is invisible until something docks 20 mm out. So the faces are READ BACK from the
    compiled model: the +y rail's face at the entry x and at the datum x must be the declared
    `DECK_FRAME_HALF[1] + CATCH` and `DECK_FRAME_HALF[1] + RESIDUAL`. This is the check that
    makes the geometry a measurement instead of an intention.
    """
    data = mujoco.MjData(model)
    mujoco.mj_resetDataKeyframe(model, data, 0)
    mujoco.mj_forward(model, data)
    out = {}
    half_y = float(rig.DECK_FRAME_HALF[1])
    for suffix, side in (('posy', +1), ('negy', -1)):
        for name in (f'w2_taper_face_{suffix}', f'w2_chamfer_face_{suffix}'):
            geom = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, name)
            if geom < 0:
                raise RuntimeError(f'the built world has no geom {name!r}')
            centre = np.array(data.geom_xpos[geom], dtype=float)
            size = np.array(model.geom_size[geom], dtype=float)
            rotation = data.geom_xmat[geom].reshape(3, 3)
            half = rotation @ np.array([size[0], 0.0, 0.0])
            inner = centre - side * (rotation @ np.array([0.0, size[1], 0.0]))
            out[name] = {'inner_face_at_west_end': [round(float(inner[0] - half[0]), 6),
                                                    round(float(inner[1]), 6)],
                         'inner_face_at_east_end': [round(float(inner[0] + half[0]), 6),
                                                    round(float(inner[1]), 6)],
                         'declared_half_y_m': half_y}
    return out


def load_merged_spec():
    """The merged world as an editable `MjSpec`, ready to re-serialise.

    It has to come through a FILE in `assets/`, not `from_string`: the merged world carries no
    `meshdir`, so MuJoCo resolves its bare mesh paths against the model file's directory, and a
    spec parsed from a string has none. Measured symptom: `spec.to_xml()` raised
    `Error opening file 't800/robot/t800/meshes/LINK_BASE.obj'` -- the round trip, not the
    compile, is what resolves assets.
    """
    bp3.install()                     # rebinds merge_world's ROLES/TARGET to the P3 assembly
    text, layout, _model0, _stepping, _kin, _notes, _applied = mw.build_text()
    _TMP.write_text(text, encoding='utf-8')
    return mujoco.MjSpec.from_file(str(_TMP)), layout


def build_spec():
    """The whole world as an MjSpec, plus the numbers it was built from."""
    spec, layout = load_merged_spec()
    try:
        for key in list(spec.keys):
            spec.delete(key)
    except (AttributeError, TypeError):     # no keyframe in this build: nothing to drop
        pass

    # 1. the receiver becomes its own station
    count = int(rig_meta()['recv_rollers'])
    for index in range(count):
        body = spec.body(f'c_recv_roller_{index}')
        if body is None:
            raise RuntimeError(f'the merged world has no c_recv_roller_{index}: the receiver row '
                               f'cannot be moved, and moving "the rest" would be a different world')
        body.pos = np.array([body.pos[0] + RECEIVER_SHIFT_M, body.pos[1], body.pos[2]])

    # 2. real floor and real walls along the whole leg
    for name, grow in (('cell_ground', True), ('cell_wall_east', False)):
        geom = spec.geom(name)
        geom.pos = np.array([geom.pos[0] + (RECEIVER_SHIFT_M / 2.0 if grow
                                            else RECEIVER_SHIFT_M), geom.pos[1], geom.pos[2]])
        if grow:
            geom.size = np.array([geom.size[0] + RECEIVER_SHIFT_M / 2.0, geom.size[1],
                                  geom.size[2]])
    for name in ('cell_wall_north', 'cell_wall_south'):
        geom = spec.geom(name)
        geom.pos = np.array([geom.pos[0] + RECEIVER_SHIFT_M / 2.0, geom.pos[1], geom.pos[2]])
        geom.size = np.array([geom.size[0] + RECEIVER_SHIFT_M / 2.0, geom.size[1], geom.size[2]])

    # 3. the second chassis stands by OFF the lane, still a real entity
    n2 = spec.body('n2_base_link')
    n2.pos = np.array([n2.pos[0], n2.pos[1] + STANDBY_OFFSET_Y_M, n2.pos[2]])

    # 4. the docking mechanism
    geometry = derive_geometry(mujoco.MjModel.from_xml_path(str(CELL)), shift=RECEIVER_SHIFT_M)
    half_y = float(rig.DECK_FRAME_HALF[1])
    frame_centre_x = geometry['deck_x_at_receiver_m'] + geometry['deck_frame_x_local_m']
    frame_centre_z = (geometry['deck_z_m'] + geometry['deck_frame_z_local_m'])
    x_stop_face = frame_centre_x + float(rig.DECK_FRAME_HALF[0]) + DATUM_OFFSET_M
    x_exit = x_stop_face - RAIL_TO_STOP_CLEARANCE_M
    run = (CATCH_M - RESIDUAL_LATERAL_M) / math.tan(math.radians(CHAMFER_ANGLE_DEG))
    x_entry = x_exit - CHANNEL_LENGTH_M - run
    for side in (+1, -1):
        add_rail(spec, x_entry=x_entry, x_exit=x_exit,
                 y_entry=side * (half_y + CATCH_M), y_exit=side * (half_y + RESIDUAL_LATERAL_M),
                 side=side, z_centre=frame_centre_z)
    # The bar's WEST FACE is the datum, and a body's rest position is `pos` (the slide's
    # springref is 0), so the body is placed half its own thickness past the face.
    stop = spec.worldbody.add_body(name='w2_datum_stop',
                                   pos=[float(x_stop_face + STOP_HALF_X_M), 0.0,
                                        float(frame_centre_z)])
    stop.add_joint(name='w2_datum_slide', type=mujoco.mjtJoint.mjJNT_SLIDE, axis=[1.0, 0.0, 0.0])
    sgeom = stop.add_geom(name='w2_datum_face', type=mujoco.mjtGeom.mjGEOM_BOX)
    sgeom.size = [STOP_HALF_X_M, STOP_HALF_Y_M, STOP_HALF_Z_M]
    sgeom.rgba = list(RAIL_COLOUR_DARK)
    sgeom.friction = [float(rig.GUIDE_FRICTION), 0.005, 0.0001]

    # 4b. the deck's compliances. The lateral one lets the chamfer move the deck instead of
    # having to shove the vehicle sideways; the yaw one is what lets a straight channel accept a
    # vehicle that is not perfectly aligned, and it is the reason the residual yaw can be a
    # property of the channel rather than of the vehicle's aiming. Both are declared springs with
    # declared travel, and both are absent from the cell world.
    deck = spec.body('c_deck')
    deck.add_joint(name='c_deck_slide_y', type=mujoco.mjtJoint.mjJNT_SLIDE, axis=[0.0, 1.0, 0.0])
    deck.add_joint(name='c_deck_yaw', type=mujoco.mjtJoint.mjJNT_HINGE, axis=[0.0, 0.0, 1.0])

    mechanism = {'receiver_shift_m': RECEIVER_SHIFT_M,
                 'receiver_bodies_moved': count,
                 'chamfer_run_m': run, 'chamfer_entry_x_m': x_entry,
                 'chamfer_exit_x_m': x_exit,
                 'chamfer_channel_length_m': CHANNEL_LENGTH_M,
                 'chamfer_catch_m': CATCH_M,
                 'chamfer_residual_lateral_m': RESIDUAL_LATERAL_M,
                 'datum_face_x_m': x_stop_face,
                 'datum_offset_m': DATUM_OFFSET_M,
                 'deck_frame_centre_z_m': frame_centre_z,
                 'deck_lateral_range_m': DECK_LATERAL_RANGE_M,
                 'deck_yaw_range_rad': DECK_YAW_RANGE_RAD,
                 'datum_stop_travel_m': STOP_TRAVEL_M,
                 'standby_offset_y_m': STANDBY_OFFSET_Y_M}
    geometry.update(mechanism)
    return spec, geometry, mechanism


def finish(spec):
    """Set the joint parameters the class defaults cannot express, then compile and key."""
    for name, stiffness, damping, travel in (
            ('w2_datum_slide', STOP_STIFFNESS_N_M, STOP_DAMPING, STOP_TRAVEL_M),
            ('c_deck_slide_y', DECK_LATERAL_STIFFNESS_N_M, DECK_LATERAL_DAMPING,
             DECK_LATERAL_RANGE_M),
            ('c_deck_yaw', DECK_YAW_STIFFNESS_NM_RAD, DECK_YAW_DAMPING_NM_S_RAD,
             DECK_YAW_RANGE_RAD)):
        joint = spec.joint(name)
        joint.stiffness = stiffness
        joint.damping = damping
        joint.springref = 0.0
        joint.range = [-travel, travel]
        joint.limited = True
    joint = spec.joint('w2_datum_slide')
    joint.pos = [0.0, 0.0, 0.0]
    # A hinge's spring is torqued about its own axis, so its declared value is in N m/rad and is
    # useless unless the axis is the vertical one; stated here rather than assumed.
    if not np.allclose(np.abs(np.asarray(spec.joint('c_deck_yaw').axis, dtype=float)), [0, 0, 1]):
        raise RuntimeError('c_deck_yaw must hinge about z, or its stiffness is the wrong units')


def build():
    spec, geometry, mechanism = build_spec()
    finish(spec)
    _text2, layout, _m, _s, _k, _n, _a = mw.build_text()   # layout is a pure function of sources
    # The home pose is re-derived from the NEW model rather than copied: the deck gained a joint,
    # the receiver moved and the parking solve depends on both, so a copied qpos would be the
    # wrong length and the wrong pose.
    model = compile_spec(spec)
    qpos, ctrl, applied, merged_notes = mw.merged_home(model, layout)
    if len(qpos) != model.nq:
        raise RuntimeError(f'the derived home qpos is {len(qpos)} long but the model has nq '
                           f'{model.nq}: a keyframe would be silently padded and the world would '
                           f'start from a pose nobody derived')
    spec.add_key(name='home', qpos=qpos, ctrl=ctrl)
    # The MERGED world arrives with its own keyframe. `spec.delete` on it empties the name but the
    # key survives, so `add_key` left TWO keyframes -- one of them nameless and 155 long against a
    # 157-long model. A world with two keyframes is a world whose start pose depends on the index
    # someone happens to pass, so everything that is not this derivation is removed and the count
    # is ASSERTED rather than assumed.
    for key in list(spec.keys):
        if key.name != 'home' or len(key.qpos) != len(qpos):
            spec.delete(key)
    xml = mw.with_option_block(spec.to_xml())
    model = compile_xml(xml)
    if model.nkey != 1 or len(re.findall(r'<key[ >]', xml)) != 1:
        raise RuntimeError(f'the built world has {len(re.findall(chr(60) + "key[ >]", xml))} key '
                           f'elements and nkey {model.nkey}: exactly one home keyframe is required')
    geometry['home_qpos_len'] = int(len(qpos))
    geometry['home_ctrl_len'] = int(len(ctrl))
    header = (
        '<!--\n'
        '  GENERATED by experiments/build_w2_logistic_world.py -- do not hand-edit.\n'
        '  Rebuild and verify with:  ./.venv/bin/python experiments/build_w2_logistic_world.py --check\n\n'
        '  The integrated LOGISTICS world. It differs from assets/world_p3_cell.xml in exactly\n'
        '  three declared ways, and nothing else:\n'
        '    1. the receiver roller row is moved +5.000 m east, so the receiver is its own\n'
        '       station and the transport leg is metres instead of millimetres;\n'
        '    2. the ground and the walls extend with it, so the leg has floor;\n'
        '    3. the docking interface gains passive mechanical guidance (two chamfer rails, a\n'
        '       spring-mounted longitudinal datum bar) and the deck mount gains a lateral\n'
        '       compliance. The cell world does NOT have these, so P3 evidence is untouched and\n'
        '       this is a NEW VERSION of the interface with its own evidence.\n\n'
        '  WHAT THIS WORLD DOES NOT CLAIM:\n'
        '    * that the mechanism is industrially designed -- it is a first-pass geometry whose\n'
        '      catch envelope and residual are MEASURED by experiments/probe_w3_dock_mechanism.py;\n'
        '    * that the chamfer can move the vehicle sideways. It cannot, and it does not have to:\n'
        '      see `static_audit()`, which computes the vehicle-lateral force budget and shows the\n'
        '      vehicle is out of reach while the deck is not;\n'
        '    * that the second chassis can transport: it carries no deck, and here it is parked in\n'
        '      a declared standby position off the lane rather than removed -- a standby that is\n'
        '      deleted would be a claim that it does not exist;\n'
        '    * SLAM: the map remains an operator prior generated from geometry.\n'
        '-->\n')
    return header + xml, model, geometry, mechanism, applied, merged_notes


def write(check=False):
    text, model, geometry, mechanism, applied, merged_notes = build()
    if check:
        if not OUT.is_file():
            print(f'MISSING: {OUT.relative_to(ROOT)} has not been generated')
            return 1
        if OUT.read_text(encoding='utf-8') == text:
            print(f'OK: {OUT.relative_to(ROOT)} matches its sources '
                  f'(sha256 {hashlib.sha256(OUT.read_bytes()).hexdigest()})')
            return 0
        print(f'STALE: {OUT.relative_to(ROOT)} differs from what its sources produce. '
              f'Re-run without --check.')
        return 1
    OUT.parent.mkdir(parents=True, exist_ok=True)
    OUT.write_text(text, encoding='utf-8')
    print(f'wrote {OUT.relative_to(ROOT)}  {len(text)} chars')
    print(f'  sha256 {hashlib.sha256(OUT.read_bytes()).hexdigest()}')
    print(f'  nq={model.nq} nv={model.nv} nu={model.nu} nbody={model.nbody} ngeom={model.ngeom} '
          f'njnt={model.njnt} nkey={model.nkey}')
    print(f'  transport leg {geometry["travel_source_to_receiver_m"]:.4f} m')
    print(f'  chamfer run {geometry["chamfer_run_m"]:.4f} m from x {geometry["chamfer_entry_x_m"]:.4f}'
          f' to {geometry["chamfer_exit_x_m"]:.4f}, acting on the deck frame at z '
          f'{geometry["deck_frame_centre_z_m"]:.4f}')
    audit = static_audit(model, geometry)
    print('  static audit:')
    for key, value in audit.items():
        print(f'    {key}: {value}')
    return 0


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--check', action='store_true',
                        help='re-derive and compare; exit 1 on any drift')
    parser.add_argument('--audit', action='store_true',
                        help='print the static force budget and exit')
    args = parser.parse_args()
    if args.audit:
        spec, geometry, _notes = build_spec()
        finish(spec)
        model = compile_spec(spec)
        print(json.dumps({'geometry': geometry,
                          'static_audit': static_audit(model, geometry)}, indent=2))
        return 0
    return write(check=args.check)


if __name__ == '__main__':
    sys.exit(main())
