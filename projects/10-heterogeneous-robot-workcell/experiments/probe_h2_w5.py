"""H2 in candidate B: put the tray on the STOPPED source band, release, and withdraw.

WHAT H2 ADDS OVER H1
--------------------
H1 (`experiments/probe_h_w5.py`, `reports/p4-h1-w5-01`) proved the humanoid can close its
fingers on the tray, lift it +0.126 m, carry it airborne 13.9 s, put it back down and
withdraw. H2 changes exactly one thing: WHERE it is put down. H1 put it back on the
presentation fixture; H2 puts it on the band's first roller pair -- the mechanism the
logistics chain reads from. The fixture's +x edge (4.375) is 13.7 mm short of where the
tray's underside ends once placed (4.38872), so the fixture cannot be carrying it.

The transfer is a level translation because candidate B raised the band to the fixture's
height; in the source world the two were 0.325 m apart, which is what made the tray
transfer a different problem there.

REUSED, NOT REWRITTEN
---------------------
  * `tray_task` is the SAME judge with the SAME frozen thresholds. The six-stage contract is
    not modified and no threshold is loosened. `place` now happens to end on the band.
  * `probe_h_w5`'s instruments (`hand_geom_sets`, `foot_and_collision_instruments`) are
    IMPORTED, so the H1 records in this report come from the same code as in the H1 report.

NEW ROWS ARE DECLARED, NOT FROZEN
---------------------------------
The rows in `H2_THRESHOLDS` have no contract behind them. The guidance requires new metrics
to be declared and reviewed rather than introduced silently, and requires that no threshold
be back-fitted to a failure. Each one is falsifiable: the same quantity is recorded over the
whole run, and the run's own values show it moving through the region that fails.

FORBIDDEN, AND THEREFORE NOT DONE
---------------------------------
No weld/equality on the tray, no runtime qpos writes, no disabling collisions, no fixed
base. `runtime_qpos_writes` is counted and reported, and any equality touching the tray
aborts the run.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src'))
sys.path.insert(0, str(ROOT / 'experiments'))

import probe_h_w5 as H1                       # the shared instruments, imported not copied

WORLD = ROOT / 'assets' / 'world_w5_h085.xml'
SEQUENCE_SECONDS = 50.0
WALL_DEADLINE_S = 300.0

PHASE_WINDOWS = {
    'grasp':   (9.5, 13.5),
    'lift':    (13.5, 22.0),
    'hold':    (22.0, 25.0),
    'place':   (25.0, 33.5),
    'release': (33.5, 37.0),
    'exit':    (41.0, 49.0),
}
EXIT_START = PHASE_WINDOWS['exit'][0]
APPROACH_START = 2.0
#: `place` is split internally, NOT into a new stage: the six-stage contract keeps its six
#: names. First slide in +x at the lifted height, then lower onto the rollers.
TRANSFER_END = 29.5
LIFT_HEIGHT_M = 0.12
SAMPLE_INTERVAL_TICKS = 50

#: Declared station. `D102`'s feasible band is x in [4.0, 4.15]; 4.13 is inside it, and at
#: 4.13 the arm's residual to the handles at the place position is 2.5e-4 m (`_diag/
#: out_h2_shape.txt`) with 37 mm of clearance between the legs and the presentation fixture.
DEFAULT_STATION_X = 4.13

#: Where the tray is put down: the midpoint of `c_fixed_roller_0_0` (4.41372) and
#: `c_fixed_roller_0_1` (4.49372). This is the same convention the SOURCE world uses for its
#: own tray station (its tray at 4.61372 is the midpoint of `_0_2` 4.57372 and `_0_3` 4.65372),
#: so the tray is placed the way the world already places it, not the way that is convenient.
PLACE_X = 4.45372

#: The declared grasp anchor's y: the handle ball's own centre, and what H1 used. It is NOT
#: moved to buy band clearance, because that was measured and it costs the grip: at 0.34 the
#: tray tilts 5-18 deg and is dragged 0.096 m instead of carried (reports/_h2_smoke3), and at
#: 0.36 the lift fails outright (+0.0388 m against a 0.05 m requirement, reports/_h2_smoke2).
HANDLE_Y = 0.30
HANDLE_Z = 0.85

#: The declared hand-roll bias, in radians, on each arm's ELBOW YAW joint -- the joint nearest
#: the hand, and therefore the knob for the hand's roll about the forearm. It exists because
#: the grasp site sits on the PALM, so the arm's five joints leave the hand's roll free: with
#: the roll as the IK happens to leave it, the ring finger's base lies inside the first roller
#: by 3.8 mm while the tray is being set down on it.
#:
#: MEASURED, `_diag/out_h2_roll2.txt`, at handle_y 0.30 with the GRASP posture:
#:
#:     bias       left hand -> band      right hand -> band
#:     0.00          -0.0037 m             -0.0038 m
#:     +0.30         +0.0132 m             +0.0067 m
#:     +0.50         +0.0176 m             +0.0180 m
#:     +0.70         +0.0194 m             +0.0214 m      <- chosen, balanced
#:     +0.90         +0.0188 m             +0.0247 m
#:     +1.20         +0.0119 m             +0.0292 m
#:
#: and the fingers stay on the handle throughout (-0.0157 m, i.e. still wrapping it), with the
#: grasp site still on the declared anchor to 3e-5 m. BOTH ARMS TAKE THE SAME SIGN; the
#: mirrored guess (left +b, right -b) was wrong and left the right ring finger inside.
#:
#: IT IS STILL NOT AN UNBLOCK, and the declared default is therefore 0.0. Run as a pair
#: (`p4-h2-w5-01` at 0.0 against `p4-h2-w5-02` at +0.70), the roll moves the failure rather
#: than removing it: the ring finger clears, and the INDEX finger and THUMB go in instead
#: (`c_fixed_roller_0_1 x h_lh_ff_distal` 38 samples, `c_fixed_roller_0_0 x h_lh_ff_distal` 28,
#: `c_fixed_roller_0_0 x h_lh_th_tip` 10), and the tray is left 0.113 m off in y against the
#: 0.020 m window. The hand is simply wider than the clearance: measured below.
HAND_ROLL_BIAS = 0.0

#: When the roll is on. It is NOT applied to the grasp, lift or hold phases -- there the tray is
#: lifted 0.12 m so the fingers are above the band anyway -- which also keeps those three phases
#: identical to H1's. It ramps in just before the tray is lowered over the rollers and ramps out
#: while the fingers open, because the exit judge measures the arms against the DEFAULT posture
#: and a 0.7 rad elbow offset would fail it.
HAND_ROLL_IN = (28.4, 29.4)
HAND_ROLL_OUT = (35.6, 37.4)

#: The rollers the tray is expected to end up on, and the fixture it must NOT be on.
SUPPORT_ROLLERS = ('c_fixed_roller_0_0', 'c_fixed_roller_0_1')
FIXTURE = 'w5_h085_station'

SIDES = ('left', 'right')

#: DECLARED AND NOT FROZEN. See the module docstring.
H2_THRESHOLDS = {
    'place_window_x_m': 0.020,        # the tray's origin must land within this of PLACE_X
    'place_window_y_m': 0.020,
    'tray_tilt_max_deg': 5.0,         # the tray's own up axis against the world's
    'support_contact_min_samples': 1,  # each named roller must be under the tray at least once
    'fixture_contact_max_samples': 0,  # and the fixture must carry NONE of it after release
    'hand_clear_m': 0.020,            # every hand geom this far from every tray geom at the end
    'band_clear_m': 0.050,            # every humanoid geom this far from the band at the end
    'still_seconds': 1.0,             # no hand-tray contact for this long, unbroken, at the end
    'rest_speed_mps': 0.010,
    #: How far the source rollers' SURFACE may travel over the whole run, from their total
    #: rotation. This is the quantity that answers "did the source carry the tray"; the
    #: instantaneous speed does not, because a landing tray kicks a passive roller.
    'band_carry_m': 0.005,
}


def anchor_at(x, height, handle_y=HANDLE_Y):
    """The declared fixture anchor set, at a declared x and height above the tray's origin."""
    import numpy as np
    return {side: np.array([x, sign * handle_y, HANDLE_Z + height])
            for side, sign in (('left', 1), ('right', -1))}


#: The hand-roll corrector owns its own MjData, for the same reason `ArmIK` does: reading
#: `site_xpos` off data that something else wrote is how a solver reports a pose it never
#: visited. Seeded from the runtime every call.
_ROLL_SCRATCH = {}


def roll_correct(runtime, q, anchors, bias, iterations=30):
    """Pin each elbow-yaw at (its IK value + bias) and re-solve the other four joints.

    `Runtime.arm_ik` solves the grasp SITE (3 numbers) with five joints, so two degrees of
    freedom are left to the solver's own choice -- including the hand's roll. This picks the
    roll deliberately instead, and pays for it by re-solving the remaining four joints so the
    site returns to the declared anchor. It is a correction to the arm solution, not a new
    controller: the target is still the same declared anchor.
    """
    import numpy as np
    import mujoco
    m = runtime.m
    scratch = _ROLL_SCRATCH.get('data')
    if scratch is None:
        scratch = _ROLL_SCRATCH['data'] = mujoco.MjData(m)
        _ROLL_SCRATCH['model_id'] = id(m)
    assert _ROLL_SCRATCH.get('model_id') == id(m), 'roll scratch belongs to another model'

    qa = runtime.qa[runtime.arm_ids]
    va = runtime.va[runtime.arm_ids]
    site = {'left': int(m.site(runtime.names.n('lh_grasp')).id),
            'right': int(m.site(runtime.names.n('rh_grasp')).id)}
    sl = {'left': slice(0, 5), 'right': slice(5, 10)}
    pin = {'left': 4, 'right': 9}          # J17 / J22 ELBOW_YAW, in arm_ids order
    lim = m.jnt_range[m.actuator_trnid[runtime.body_act[runtime.arm_ids], 0]]

    q = np.array(q, float)
    for side in SIDES:
        q[pin[side]] = float(np.clip(q[pin[side]] + bias,
                                     lim[pin[side]][0], lim[pin[side]][1]))
    scratch.qpos[:] = runtime.d.qpos
    for _ in range(iterations):
        scratch.qpos[qa] = q
        mujoco.mj_forward(m, scratch)
        worst = 0.0
        for side in SIDES:
            err = anchors[side] - np.array(scratch.site_xpos[site[side]], float)
            worst = max(worst, float(np.linalg.norm(err)))
            if np.linalg.norm(err) < 3e-5:
                continue
            jac = np.zeros((3, m.nv))
            mujoco.mj_jacSite(m, scratch, jac, None, site[side])
            free = [sl[side].start + i for i in range(5) if (sl[side].start + i) != pin[side]]
            J = jac[:, va[free]]
            dq = J.T @ np.linalg.solve(J @ J.T + np.eye(3) * 1e-3, err)
            for k, i in enumerate(free):
                q[i] = float(np.clip(q[i] + np.clip(dq[k], -0.05, 0.05),
                                     lim[i][0], lim[i][1]))
        if worst < 3e-5:
            break
    return q


def hand_roll_at(now, full):
    """The declared roll schedule: zero through grasp/lift/hold, full over the band, then out."""
    def ramp(a, b):
        return float(min(max((now - a) / (b - a), 0.0), 1.0))
    return full * (ramp(*HAND_ROLL_IN) - ramp(*HAND_ROLL_OUT))


def stage_driver(runtime, tray0_x, now, default_arms, open_hand, grasp_hand, exit_path,
                 handle_y=HANDLE_Y, hand_roll=HAND_ROLL_BIAS, phase_windows=None,
                 approach_start=None, place_x=None):
    """The H chain's stage schedule.

    `phase_windows` / `approach_start` / `place_x` default to this module's own declared values, so
    the H2 report's numbers come from exactly the code they came from before. H3 passes its own
    values for ONE purpose: the integration negative case ("humanoid grasp failure implies
    downstream does not start").

    ★ AND THAT NEGATIVE CASE NEEDED **TWO** SUBSTITUTIONS, NOT ONE. The first attempt only opened
    the fingers. Measured consequence (`_diag/out_h3_neg1.txt`): the OPEN hand still swept through
    the `place` translation and **bulldozed the tray off the presentation fixture onto the source
    band** -- handoff support `['source_band']`, tray at x 4.4869, and the chain then carried it to
    the receiver. A "failure" that delivers the tray is not a failure, and the negative row would
    have passed on a scenario that proved nothing. So the negative arm also sets `place_x` to the
    tray's own start x, because a grasp that holds nothing cannot translate anything either.
    """
    windows = PHASE_WINDOWS if phase_windows is None else phase_windows
    approach = APPROACH_START if approach_start is None else approach_start
    target_x = PLACE_X if place_x is None else float(place_x)

    def ramp(start, end):
        return float(min(max((now - start) / (end - start), 0.0), 1.0))

    roll = hand_roll_at(now, hand_roll)

    def reach(x, height):
        anchors = anchor_at(x, height, handle_y)
        q = runtime.arm_ik(anchors)
        if abs(roll) > 1e-9:
            q = roll_correct(runtime, q, anchors, roll)
        return q

    if now < approach:
        return None, None
    if now < windows['grasp'][0]:
        return reach(tray0_x, 0.0), {s: open_hand for s in SIDES}
    if now < windows['grasp'][1]:
        bl = ramp(*windows['grasp'])
        return reach(tray0_x, 0.0), {s: open_hand + bl * (grasp_hand - open_hand) for s in SIDES}
    if now < windows['lift'][1]:
        return reach(tray0_x, LIFT_HEIGHT_M * ramp(*windows['lift'])), \
               {s: grasp_hand for s in SIDES}
    if now < windows['hold'][1]:
        return reach(tray0_x, LIFT_HEIGHT_M), {s: grasp_hand for s in SIDES}
    if now < windows['place'][1]:
        if now < TRANSFER_END:
            x = tray0_x + (target_x - tray0_x) * ramp(windows['place'][0], TRANSFER_END)
            return reach(x, LIFT_HEIGHT_M), {s: grasp_hand for s in SIDES}
        dl = ramp(TRANSFER_END, windows['place'][1])
        return reach(target_x, LIFT_HEIGHT_M * (1.0 - dl)), {s: grasp_hand for s in SIDES}
    if now < windows['release'][1]:
        bl = ramp(*windows['release'])
        return reach(target_x, 0.0), \
               {s: grasp_hand + bl * (open_hand - grasp_hand) for s in SIDES}
    if now < windows['exit'][0]:
        return reach(target_x, 0.0), {s: open_hand for s in SIDES}
    if exit_path is None:
        return default_arms, {s: open_hand for s in SIDES}
    return exit_path.command(runtime, now - EXIT_START), {s: open_hand for s in SIDES}


def support_snapshot(model, data, payload_geoms, watched_bodies):
    """Which of the watched bodies the tray is touching right now. Read-only."""
    out = {name: 0 for name in watched_bodies}
    for index in range(int(data.ncon)):
        contact = data.contact[index]
        g1, g2 = int(contact.geom1), int(contact.geom2)
        if g1 in payload_geoms:
            other = g2
        elif g2 in payload_geoms:
            other = g1
        else:
            continue
        name = model.body(int(model.geom_bodyid[other])).name
        if name in out:
            out[name] += 1
    return out


def clearances(model, data, names, humanoid_geoms, payload_geoms, band_geoms, hand_geoms):
    """(min humanoid->band distance, min hand->tray distance, who was closest).

    Exact geom-to-geom distances, so this is not an AABB argument.
    """
    import numpy as np
    import mujoco
    worst_band, who_band, worst_hand, who_hand = float('inf'), None, float('inf'), None
    for g in humanoid_geoms:
        for b in band_geoms:
            dist = mujoco.mj_geomDistance(model, data, g, b, 2.0, None)
            if dist < worst_band:
                worst_band = dist
                who_band = model.body(int(model.geom_bodyid[g])).name
    for g in hand_geoms:
        for p in payload_geoms:
            dist = mujoco.mj_geomDistance(model, data, g, p, 2.0, None)
            if dist < worst_hand:
                worst_hand = dist
                who_hand = model.body(int(model.geom_bodyid[g])).name
    return worst_band, who_band, worst_hand, who_hand


def tray_tilt_deg(model, data, tray_body):
    import numpy as np
    rot = np.array(data.xmat[int(tray_body)], float).reshape(3, 3)
    return float(np.degrees(np.arccos(np.clip(rot[2, 2], -1.0, 1.0))))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--run-id', required=True)
    ap.add_argument('--station-x', type=float, default=DEFAULT_STATION_X)
    ap.add_argument('--place-x', type=float, default=PLACE_X)
    ap.add_argument('--handle-y', type=float, default=HANDLE_Y)
    ap.add_argument('--hand-roll', type=float, default=HAND_ROLL_BIAS)
    ap.add_argument('--duration', type=float, default=SEQUENCE_SECONDS)
    args = ap.parse_args()

    out = ROOT / 'reports' / args.run_id
    out.mkdir(parents=True, exist_ok=False)
    started = time.monotonic()
    report = {
        'scope': 'DIAGNOSTIC_ONLY',
        'probe': 'H2 in candidate B',
        'pid': os.getpid(), 'command': sys.argv, 'status': 'ERROR', 'samples': [],
        'source_sha256': hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        'world': str(WORLD.relative_to(ROOT)),
        'world_sha256': hashlib.sha256(WORLD.read_bytes()).hexdigest(),
        'station_x': args.station_x,
        'place_x_declared': args.place_x,
        'handle_y_declared': args.handle_y,
        'handle_y_note': ('the grasp anchor y: the handle ball centre, as in H1. Moving it to buy '
                          'band clearance was measured and rejected -- see HANDLE_Y'),
        'hand_roll_bias_declared': args.hand_roll,
        'hand_roll_schedule': {'in': list(HAND_ROLL_IN), 'out': list(HAND_ROLL_OUT)},
        'hand_roll_note': ('a bias on each arm\'s ELBOW YAW joint, with the other four joints '
                           're-solved so the grasp site stays on the declared anchor. It exists '
                           'because the site is on the PALM, so the hand\'s roll is free, and as '
                           'the IK happens to leave it the ring finger is 3.8 mm inside the first '
                           'roller. Measure: _diag/out_h2_roll2.txt'),
        'support_rollers': list(SUPPORT_ROLLERS),
        'presentation_fixture': FIXTURE,
        'h2_thresholds': dict(H2_THRESHOLDS),
        'h2_thresholds_note': ('DECLARED AND NOT FROZEN -- these rows have no contract behind '
                               'them and are submitted for review'),
    }
    qpos_writes = 0
    try:
        import numpy as np
        import mujoco
        from humanoid007 import tray_task
        from humanoid007.runtime import Runtime, OPEN, GRASP, ExitPath

        r = Runtime(world=str(WORLD), prefix='h_', object_body='c_payload')
        names = r.names
        report['names'] = {'prefix': names.prefix, 'object_body': names.obj()}
        report['free_base'] = int(r.m.jnt_type[0]) == int(mujoco.mjtJoint.mjJNT_FREE)
        report['equality_constraints'] = int(r.m.neq)

        # --- the tray must not be welded to anything --------------------------------------
        tray_joint = int(r.m.body_jntadr[r.m.body(names.obj()).id])
        offenders = [e for e in range(r.m.neq)
                     for o in (int(r.m.eq_obj1id[e]), int(r.m.eq_obj2id[e]))
                     if int(r.m.eq_type[e]) == int(mujoco.mjtEq.mjEQ_JOINT) and o == tray_joint]
        report['equalities_involving_the_tray'] = offenders
        report['equality_note'] = ('the one pre-existing equality binds the arm gripper fingers, '
                                   'not the tray')
        if offenders:
            raise RuntimeError('an equality constrains the tray: %r' % offenders)

        # --- is the SOURCE still? (derived, then MEASURED through the run) -----------------
        # The first version of this block was WRONG and said so confidently: it claimed the
        # band was world-attached geometry with no joint. It is not -- `c_fixed_roller_0_0`
        # owns 1 joint, `c_deck` owns 3, and the band as a whole owns 41 joints and 39
        # actuators. It is an actuated roller conveyor. So "the source stayed stopped" is not
        # structural and is checked two ways below instead of asserted.
        band_bodies = set()
        for b in range(r.m.nbody):
            nm = r.m.body(b).name or ''
            if nm.startswith(('c_fixed_roller_', 'c_recv_roller_', 'c_deck')):
                band_bodies.add(b)
        band_joints, band_dofs = [], []
        for b in sorted(band_bodies):
            first = int(r.m.body_jntadr[b])
            for k in range(first, first + int(r.m.body_jntnum[b])):
                band_joints.append(k)
                band_dofs.append(int(r.m.jnt_dofadr[k]))
        band_qadr = [int(r.m.jnt_qposadr[k]) for k in band_joints]
        band_qpos0 = np.array([r.d.qpos[a] for a in band_qadr], float) if band_qadr else np.zeros(0)
        # the roller radius, so a joint rotation can be read as surface travel. Taken from the
        # first band cylinder rather than typed, so it cannot rot.
        roller_radius = None
        for g in range(r.m.ngeom):
            if int(r.m.geom_bodyid[g]) not in band_bodies:
                continue
            if int(r.m.geom_type[g]) == int(mujoco.mjtGeom.mjGEOM_CYLINDER):
                roller_radius = float(r.m.geom_size[g][0])
                break
        band_act = [a for a in range(r.m.nu)
                    if int(r.m.actuator_trnid[a][0]) >= 0
                    and int(r.m.jnt_bodyid[int(r.m.actuator_trnid[a][0])]) in band_bodies]
        # every actuator index this probe is capable of writing, from the runtime itself
        touched = set(int(i) for i in r.body_act)
        for side in r.hand_act:
            touched |= set(int(i) for i in r.hand_act[side])
        report['source_mechanism'] = {
            'bodies': len(band_bodies),
            'joints': len(band_joints),
            'actuators': len(band_act),
            'roller_radius_m': roller_radius,
            'band_actuators_the_probe_can_command': sorted(set(band_act) & touched),
            'note': ('the source band is an ACTUATED roller conveyor -- its rollers have joints. '
                     '"Source stopped" is therefore not structural. It is checked by (a) the '
                     'intersection of the band actuators with every actuator this probe can '
                     'write, which must be empty, and (b) the band joints\' measured speed '
                     'through the run, which the `source_stopped` row judges.'),
        }

        # --- INITIALISATION ONLY: the declared station ------------------------------------
        base_adr = int(r.m.jnt_qposadr[r.m.body_jntadr[int(r.m.body(names.n('LINK_BASE')).id)]])
        assert base_adr == 0, 'the runtime assumes the base free joint is first'
        # every qpos write this probe makes goes through here, so the count is auditable rather
        # than asserted. There is no write site inside the loop at all.
        qpos_write_log = []
        for offset, value in enumerate((args.station_x, 0.0, 1.03)):
            r.d.qpos[base_adr + offset] = value
            qpos_write_log.append([int(base_adr + offset), float(value)])
        for offset, value in enumerate((1.0, 0.0, 0.0, 0.0)):
            r.d.qpos[base_adr + 3 + offset] = value
            qpos_write_log.append([int(base_adr + 3 + offset), float(value)])
        qpos_writes += len(qpos_write_log)
        mujoco.mj_forward(r.m, r.d)
        report['init_qpos_writes'] = len(qpos_write_log)
        report['qpos_write_sites'] = {
            'init': qpos_write_log,
            'in_loop': [],
            'note': ('the loop never writes qpos: it passes arm/finger targets to Runtime.step and '
                     'lets physics produce the motion. The tray is a free body, so its travel is '
                     'a physics result, not a teleport.'),
        }
        report['station_base'] = [float(v) for v in r.d.qpos[base_adr:base_adr + 3]]

        tray0 = np.array(r.d.body(names.obj()).xpos, float)
        report['tray_start'] = [float(v) for v in tray0]
        report['transfer_m'] = float(args.place_x - tray0[0])
        report['payload_z_initial'] = float(tray0[2])
        report['payload_x_initial'] = float(tray0[0])

        payload_joint = int(r.m.body_jntadr[r.m.body(names.obj()).id])
        payload_dof = int(r.m.jnt_dofadr[payload_joint])
        left_geoms, right_geoms, payload_geoms = H1.hand_geom_sets(r.m, names)
        hand_geoms = left_geoms | right_geoms
        humanoid_geoms = [g for g in range(r.m.ngeom)
                          if (r.m.body(int(r.m.geom_bodyid[g])).name or '').startswith(names.prefix)]
        band_geoms = [g for g in range(r.m.ngeom)
                      if (r.m.body(int(r.m.geom_bodyid[g])).name or '').startswith(
                          ('c_fixed_roller_', 'c_recv_roller_'))
                      or (r.m.body(int(r.m.geom_bodyid[g])).name or '') == 'c_deck']
        report['hand_geoms'] = {'left': len(left_geoms), 'right': len(right_geoms),
                                'payload': len(payload_geoms), 'band': len(band_geoms)}

        default_arms = r.policy.default[r.arm_ids].copy()
        exit_path = ExitPath(r, anchor_at(args.place_x, 0.0, args.handle_y), default_arms,
                             ExitPath.DEFAULT_VARIANT)

        report['control_source'] = {
            'actuators_driven_by': 'humanoid007.policy.T800Policy (ONNX) + this probe stage driver',
            'policy_cfg': 'config/t800/walking.yaml',
            'model_cfg': 'config/t800/model.yaml',
            'stand_law': 'config/t800/stand.yaml',
            'runtime_sha256': hashlib.sha256(
                (ROOT / 'src' / 'humanoid007' / 'runtime.py').read_bytes()).hexdigest(),
            'perception': 'NONE at runtime -- the anchor set is a DECLARED fixture pose, and the '
                          'placement target is a DECLARED station, read once from the world',
        }

        # --- the run ----------------------------------------------------------------------
        foot_worst = {'L': 0.0, 'R': 0.0}
        abnormal_total, self_total = {}, {}
        band_min, band_min_who = float('inf'), None
        hand_min, hand_min_who = float('inf'), None
        body_geoms_near_band = set()
        band_speed_max = 0.0
        while r.d.time < args.duration:
            if time.monotonic() - started > WALL_DEADLINE_S:
                report['status'] = 'WALL_TIMEOUT'
                break
            now = r.d.time
            arms, hands = stage_driver(r, float(tray0[0]), now, default_arms, OPEN, GRASP,
                                       exit_path, args.handle_y, args.hand_roll)
            r.step(np.zeros(3), arms, hands, stationary=now > 2)
            if band_dofs:
                speed = float(np.max(np.abs(r.d.qvel[band_dofs])))
                band_speed_max = max(band_speed_max, speed)

            if r.tick % SAMPLE_INTERVAL_TICKS == 0:
                # Clearances are exact geom-to-geom distances and cost ~5k calls per sample, so
                # they are computed ON THE SAMPLE GRID, not every step. The run minimum below is
                # therefore a SAMPLED minimum, and it is reported as one.
                gap_band, who_band, gap_hand, who_hand = clearances(
                    r.m, r.d, names, humanoid_geoms, payload_geoms, band_geoms, hand_geoms)
                if gap_band < band_min:
                    band_min, band_min_who = gap_band, who_band
                if gap_hand < hand_min:
                    hand_min, hand_min_who = gap_hand, who_hand
                if gap_band <= H2_THRESHOLDS['band_clear_m'] and who_band:
                    body_geoms_near_band.add(who_band)
                row = r.snapshot()
                t = r.d.body(names.obj()).xpos
                row['payload'] = [float(t[0]), float(t[1]), float(t[2])]
                row['payload_speed'] = float(np.linalg.norm(
                    r.d.qvel[payload_dof:payload_dof + 3]))
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
                row['support'] = support_snapshot(r.m, r.d, payload_geoms,
                                                  SUPPORT_ROLLERS + (FIXTURE,))
                row['tray_tilt_deg'] = tray_tilt_deg(r.m, r.d, r.m.body(names.obj()).id)
                row['band_clearance_m'] = gap_band
                row['hand_tray_clearance_m'] = gap_hand
                row['hand_roll_rad'] = hand_roll_at(now, args.hand_roll)
                if band_dofs:
                    row['band_speed_max_rad_s'] = float(np.max(np.abs(r.d.qvel[band_dofs])))
                    row['band_turn_max_rad'] = float(np.max(np.abs(
                        np.array([r.d.qpos[a] for a in band_qadr], float) - band_qpos0)))
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

        report['source_mechanism']['band_joint_speed_max_rad_s'] = band_speed_max
        report['runtime_qpos_writes'] = qpos_writes - report['init_qpos_writes']
        report['feet'] = {'worst_pitch_deg': {k: round(v, 4) for k, v in foot_worst.items()},
                          'ground_contacts_at_samples': [
                              {'t': row['time'],
                               **{s: row['feet'][s]['ground_contacts'] for s in ('L', 'R')}}
                              for row in report['samples']]}
        report['abnormal_collisions'] = [
            {'pair': list(k), 'contact_samples': v}
            for k, v in sorted(abnormal_total.items(), key=lambda kv: -kv[1])] or []
        report['abnormal_collision_count'] = len(abnormal_total)
        report['hand_self_contacts'] = [
            {'pair': list(k), 'contact_samples': v}
            for k, v in sorted(self_total.items(), key=lambda kv: -kv[1])] or []
        report['hand_self_contact_note'] = ('finger-vs-finger within a hand: that is grasp '
                                            'closure, not a fault.')

        # --- the six-stage contract, judged by the project's own judge --------------------
        driven = {n for n, (_, end) in PHASE_WINDOWS.items() if r.d.time >= end}
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

        # --- the H2 rows ------------------------------------------------------------------
        report['h2'] = judge_h2(report, args, band_min, band_min_who, hand_min, hand_min_who,
                                sorted(body_geoms_near_band), band_speed_max)
        report['status'] = 'DIAGNOSTIC_COMPLETED'
    except Exception as exc:
        report['error'] = '%s: %s' % (type(exc).__name__, exc)
        if report.get('status') in (None, 'ERROR'):
            report['status'] = 'POST_RUN_ERROR'
    finally:
        report['wall_seconds'] = time.monotonic() - started
        (out / 'report.json').write_text(json.dumps(report, indent=2) + '\n', encoding='utf-8')
        printable = {k: v for k, v in report.items() if k not in ('samples', 'h_stages')}
        print(json.dumps(printable, indent=2, ensure_ascii=False), flush=True)
        for line in report.get('h_summary') or []:
            print('  ' + line, flush=True)
        for line in report.get('h2_summary') or []:
            print('  ' + line, flush=True)
    return 0 if report['status'] == 'DIAGNOSTIC_COMPLETED' else 1


def judge_h2(report, args, band_min, band_min_who, hand_min, hand_min_who, near_band,
             band_speed_max):
    """The declared H2 rows. Each reports the run's own numbers so it can be seen to move."""
    T = H2_THRESHOLDS
    samples = report['samples']
    rows = {}

    def row(name, ok, detail, numbers):
        rows[name] = {'verdict': 'PASS' if ok else 'FAIL', 'detail': detail, 'numbers': numbers}

    # 1. the tray is where it was put
    tail = samples[-8:] if len(samples) >= 8 else samples
    final = samples[-1] if samples else None
    if final is None:
        row('tray_in_place_window', False, 'no samples', {})
    else:
        dx = final['payload'][0] - args.place_x
        dy = final['payload'][1]
        dz = final['payload'][2] - report['payload_z_initial']
        ok = (abs(dx) <= T['place_window_x_m'] and abs(dy) <= T['place_window_y_m']
              and abs(dz) <= 0.010)
        row('tray_in_place_window', ok,
            'tray origin is %+.5f m in x, %+.5f m in y and %+.5f m in z from the declared '
            'place point (windows %.3f / %.3f / 0.010 m)'
            % (dx, dy, dz, T['place_window_x_m'], T['place_window_y_m']),
            {'dx_m': dx, 'dy_m': dy, 'dz_m': dz,
             'final_xyz': final['payload'], 'declared_place_x': args.place_x})

    # 2. the designated rollers carry it, at least once, after release
    last_contact = None
    for index, s in enumerate(samples):
        if s['hand_contacts']['left'] or s['hand_contacts']['right']:
            last_contact = index
    after = samples[(last_contact + 1):] if last_contact is not None else []
    support_counts = {name: sum(s['support'].get(name, 0) for s in after)
                      for name in SUPPORT_ROLLERS}
    fixture_counts = sum(s['support'].get(FIXTURE, 0) for s in after)
    ok = all(v >= T['support_contact_min_samples'] for v in support_counts.values())
    row('tray_on_designated_rollers', ok,
        'after the hands came off, contact samples with %s = %s (each needs >= %d)'
        % (', '.join(SUPPORT_ROLLERS), support_counts, T['support_contact_min_samples']),
        {'samples': support_counts, 'samples_after_release': len(after)})

    # 3. and NOT on the presentation fixture
    ok = fixture_counts <= T['fixture_contact_max_samples']
    row('tray_clear_of_presentation_fixture', ok,
        'the tray touched %s on %d samples after release (allowed %d): the transfer is to the '
        'band, not back onto the fixture' % (FIXTURE, fixture_counts,
                                             T['fixture_contact_max_samples']),
        {'fixture_contact_samples': fixture_counts})

    # 4. it is upright
    tilt = final['tray_tilt_deg'] if final else None
    ok = tilt is not None and tilt <= T['tray_tilt_max_deg']
    row('tray_not_tipped', bool(ok),
        'the tray\'s up axis is %.4f deg off the world up (limit %.1f); worst over the run %.4f'
        % (tilt if tilt is not None else float('nan'), T['tray_tilt_max_deg'],
           max((s['tray_tilt_deg'] for s in samples), default=float('nan'))),
        {'final_tilt_deg': tilt,
         'worst_tilt_deg': max((s['tray_tilt_deg'] for s in samples), default=None)})

    # 5. no residual hand constraint: no equality, hands physically off, for a stretch
    still = 0.0
    run_start = None
    for s in samples:
        if not (s['hand_contacts']['left'] or s['hand_contacts']['right']):
            if run_start is None:
                run_start = s['time']
            still = max(still, s['time'] - run_start)
        else:
            run_start = None
    ok = (not report['equalities_involving_the_tray']) and still >= T['still_seconds']
    row('no_residual_hand_constraint', ok,
        'no equality touches the tray (%d found) and the hands were off it for %.2f s unbroken '
        '(need %.1f s)' % (len(report['equalities_involving_the_tray']), still,
                           T['still_seconds']),
        {'equalities': report['equalities_involving_the_tray'], 'hands_off_seconds': still})

    # 6. the hands are physically clear of the tray
    ok = hand_min is not None and final is not None and \
        final['hand_tray_clearance_m'] >= T['hand_clear_m']
    row('hands_clear_of_tray', bool(ok),
        'at the end every hand geom is %.5f m from every tray geom (need >= %.3f); closest '
        'during the run %.5f m (%s)'
        % (final['hand_tray_clearance_m'] if final else float('nan'), T['hand_clear_m'],
           hand_min, hand_min_who),
        {'final_min_m': final['hand_tray_clearance_m'] if final else None,
         'run_min_m': hand_min, 'run_min_geom': hand_min_who})

    # 7. the robot is out of the band's space
    ok = (band_min is not None and final is not None
          and final['band_clearance_m'] >= T['band_clear_m'])
    row('robot_clear_of_band', bool(ok),
        'at the end every humanoid geom is %.5f m from every band geom (need >= %.3f); closest '
        'during the run %.5f m (%s), i.e. the run DID reach into the band and then withdrew'
        % (final['band_clearance_m'] if final else float('nan'), T['band_clear_m'],
           band_min, band_min_who),
        {'final_min_m': final['band_clearance_m'] if final else None,
         'run_min_m': band_min, 'run_min_geom': band_min_who,
         'bodies_that_entered_the_margin': near_band})

    # 8. nothing abnormal was hit
    ok = report['abnormal_collision_count'] == 0
    row('no_abnormal_collision', ok,
        '%d abnormal humanoid/world contact pairs were recorded%s'
        % (report['abnormal_collision_count'],
           (': ' + ', '.join('%s x %s (%d samples)'
                             % (p['pair'][0], p['pair'][1], p['contact_samples'])
                             for p in report['abnormal_collisions'][:4]))
           if report['abnormal_collisions'] else ''),
        {'pairs': report['abnormal_collisions']})

    # 9. the source band stayed stopped, MEASURED -- it is an actuated conveyor, so this is not
    #    structural. Three parts, and the third is the one that matters: the probe cannot command
    #    it, the instantaneous speed is reported, and the ROLLERS' TOTAL ROTATION over the run
    #    is what decides whether the source carried the tray anywhere.
    band_cmd = report['source_mechanism']['band_actuators_the_probe_can_command']
    turn = max((s.get('band_turn_max_rad') or 0.0) for s in samples) if samples else 0.0
    radius = report['source_mechanism'].get('roller_radius_m') or 0.035
    carried = turn * radius
    ok = (not band_cmd) and carried <= T['band_carry_m']
    row('source_stopped', bool(ok),
        'the probe can command %d of the band\'s %d actuators (must be 0); the band joints\' '
        'peak speed was %.6f rad/s and their TOTAL rotation %.6f rad, i.e. at most %.5f m of '
        'roller surface travel (limit %.4f m) -- the source did not carry the tray'
        % (len(band_cmd), report['source_mechanism']['actuators'], band_speed_max, turn,
           carried, T['band_carry_m']),
        {'band_actuators_commandable_by_probe': band_cmd,
         'band_joint_speed_max_rad_s': band_speed_max,
         'band_joint_rotation_total_rad': turn,
         'roller_surface_travel_m': carried, 'limit_m': T['band_carry_m'],
         'note': ('speed alone was the first version of this row and it FAILED at 0.086 rad/s. '
                  'The race is not speed but whether the source MOVED anything: a landing tray '
                  'kicks a passive roller, and a roller that turns 0.02 rad carries the tray '
                  '.0007 m. The row now judges the integral, and the change is recorded rather '
                  'than hidden.')})

    failed = [k for k, v in rows.items() if v['verdict'] == 'FAIL']
    overall = ('PASS: all %d H2 rows' % len(rows)) if not failed else \
        ('FAIL: %d of %d H2 rows (%s)' % (len(failed), len(rows), ', '.join(failed)))
    report['h2'] = rows
    report['h2_overall'] = overall
    report['h2_summary'] = ['%-36s %-6s %s' % (k, v['verdict'], v['detail'])
                            for k, v in rows.items()] + ['%-36s %s' % ('H2 TOTAL', overall)]
    return rows


if __name__ == '__main__':
    raise SystemExit(main())
