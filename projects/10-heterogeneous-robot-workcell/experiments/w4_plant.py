"""W4's physical plant: the logistics world behind the adapter's plant interface.

It exists in `experiments/` and not in `src/`, because `src/` must not import the simulator. The
adapter knows what a skill means; this file knows how the world answers.

Every frame is derived from the built world and from `roller_rig`'s own metadata -- the station
approach poses, the dock poses, the tray's zone, all of them. The only typed numbers are the
driving speeds and timeouts, and those are control choices with their reasons written next to them.

THE TRANSFER IS C'S OWN MECHANISM, SPUN BY ITS OWN SPEEDS
---------------------------------------------------------
The tray moves along rollers, so a transfer is "spin the band". The speeds come from
`probe_c_full_chain.py`, which is where C's chain was validated, rather than being retyped here.
"""
import math
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'experiments'))
sys.path.insert(0, str(ROOT / 'src'))

import mujoco  # noqa: E402
import build_p3_world as bp3  # noqa: E402
import merge_world as mw  # noqa: E402
import roller_rig as rig  # noqa: E402
import build_w2_logistic_world as builder  # noqa: E402
import probe_c_full_chain as cchain  # noqa: E402

WORLD = ROOT / 'assets' / 'world_w2_logistic.xml'
#: Control choices, each with the measurement that set it.
DRIVE_SPEED_MPS = 0.30          # below the 0.60 m/s the plant's own config declares as v_max
DOCK_SPEED_MPS = 0.06           # creeping into a datum that has 1 mm of clearance per side
DRIVE_TIMEOUT_S = 30.0        # the longest command is the 4.62 m approach to the receiver
                              # at DRIVE_SPEED_MPS, i.e. ~15.4 s, and at 15.0 s the first
                              # version cut that leg 89.8 mm short
#: Seating the tray on the deck with the pusher: slow, and bounded by where the deck's own
#: roller row starts. `SEAT_SPEED_MPS` is a control choice, `SEAT_MARGIN_M` is C's own
#: `RECEIVED_CLEAR_MARGIN`.
SEAT_SPEED_MPS = 0.02
SEAT_S = 1.5
SEAT_MARGIN_M = cchain.RECEIVED_CLEAR_MARGIN
STOPPED_SPEED_MPS = 0.01        # the contract's own `stopped_speed_mps`
STOPPED_HOLD_S = 0.5            # the contract's own `settle_duration_s`
#: The drift a still vehicle may accumulate over the settle window. DERIVED, not chosen:
#: `stopped_speed_mps * settle_duration_s` is the distance a vehicle obeying the speed limit
#: would cover, so this row catches the case the speed row cannot -- a vehicle whose every
#: instantaneous sample is under the limit but which nevertheless keeps moving.
DRIFT_LIMIT_M = STOPPED_SPEED_MPS * STOPPED_HOLD_S
WHEEL_KP = 6.0
WHEEL_TORQUE_LIMIT_NM = 1.2


def _band(model, data, prefix):
    obj = mujoco.mjtObj.mjOBJ_GEOM
    return sorted(float(data.geom_xpos[g][0]) for g in range(model.ngeom)
                  if (mujoco.mj_id2name(model, obj, g) or '').startswith(prefix))


def collision_bounds(model, data, body):
    """World AABB of physical geoms; visual-only shapes cannot prove containment.

    Exact extents for the tray's primitives, including rotated capsules and boxes.
    Unsupported types fail explicitly rather than underestimating a safety envelope.
    """
    lo, hi = np.full(3, np.inf), np.full(3, -np.inf)
    for g in range(model.ngeom):
        if (model.geom_bodyid[g] != body
                or (model.geom_contype[g] == 0 and model.geom_conaffinity[g] == 0)):
            continue
        rotation = data.geom_xmat[g].reshape(3, 3)
        size, kind = model.geom_size[g], model.geom_type[g]
        if kind == mujoco.mjtGeom.mjGEOM_BOX:
            half = np.abs(rotation) @ size
        elif kind == mujoco.mjtGeom.mjGEOM_SPHERE:
            half = np.full(3, size[0])
        elif kind == mujoco.mjtGeom.mjGEOM_CAPSULE:
            half = np.abs(rotation[:, 2]) * size[1] + size[0]
        elif kind == mujoco.mjtGeom.mjGEOM_CYLINDER:
            axis = rotation[:, 2]
            half = np.abs(axis) * size[1] + size[0] * np.sqrt(np.maximum(0, 1 - axis**2))
        else:
            raise ValueError('unsupported physical envelope geom type: %s' % kind)
        centre = data.geom_xpos[g]
        lo, hi = np.minimum(lo, centre - half), np.maximum(hi, centre + half)
    if not np.all(np.isfinite(lo)) or not np.all(np.isfinite(hi)):
        raise ValueError('body has no finite physical envelope')
    return lo, hi


def collision_radius(model, body):
    """Conservative body-origin radius valid after any rotation, physical geoms only."""
    radii = []
    for g in range(model.ngeom):
        if (model.geom_bodyid[g] != body
                or (model.geom_contype[g] == 0 and model.geom_conaffinity[g] == 0)):
            continue
        size, kind = model.geom_size[g], model.geom_type[g]
        if kind == mujoco.mjtGeom.mjGEOM_BOX:
            radius = np.linalg.norm(size)
        elif kind == mujoco.mjtGeom.mjGEOM_SPHERE:
            radius = size[0]
        elif kind == mujoco.mjtGeom.mjGEOM_CAPSULE:
            radius = size[0] + size[1]
        elif kind == mujoco.mjtGeom.mjGEOM_CYLINDER:
            radius = np.hypot(size[0], size[1])
        else:
            raise ValueError('unsupported physical envelope geom type: %s' % kind)
        radii.append(float(np.linalg.norm(model.geom_pos[g]) + radius))
    if not radii or not np.all(np.isfinite(radii)):
        raise ValueError('body has no finite physical envelope')
    return max(radii)


class LogisticsPlant:
    """The plant object `workcell.adapters.sim.SkillAdapter` drives."""

    def __init__(self, world=WORLD, model=None, data=None):
        """`model`/`data` let a caller hand in an ALREADY-LOADED world, so the plant can share one
        `MjData` with another controller.

        H3 needs this and nothing else in this file changes for it: the humanoid's runtime and this
        plant must act on the SAME world, in the SAME continuous run, on the SAME tray entity. Two
        separately-constructed models would be two worlds that look alike, which is exactly what
        the H3 definition forbids.

        DEFAULT UNCHANGED. With `model=None` the plant loads `world` itself, byte for byte as
        before, so W2/W3/W4/W5 evidence is untouched. When a model is supplied, `world` is only
        used as a label: the caller's model is authoritative, and this does NOT reload anything.
        """
        self.world = Path(world)
        bp3.install()
        _loaded = model is None
        self.model = mujoco.MjModel.from_xml_path(str(self.world)) if _loaded else model
        self.data = mujoco.MjData(self.model) if _loaded or data is None else data
        #: True when this plant owns the model and may reset it freely; False when it is a guest
        #: on a world someone else drives. Read by `reset()` below.
        self._owns_world = bool(_loaded and data is None)
        self._home, _ctrl, _applied, _notes = mw.merged_home(self.model)
        self._home = np.asarray(self._home, dtype=float)
        #: Every write to `data.qpos`, counted per phase. See `_set_state`.
        self.qpos_writes = {}
        if self._owns_world:
            self.reset()
        else:
            # A GUEST does not reset: the owner has already established the home state, and doing
            # it again would be a qpos write H3 must not have. `mj_forward` is read-only bookkeeping.
            #
            # `initialise` is left ABSENT rather than set to 0. A zero would say "an initialise
            # happened and wrote nothing", which is a different and false statement; absent says
            # "this plant never wrote qpos at all", which is the truth. The W5 judge reads
            # `writes_before_run.get('initialise') == 1`, so a guest world reports that check as
            # FAIL -- correctly: the guest did not do the reset, the owner did.
            mujoco.mj_forward(self.model, self.data)
        self._derive()
        self.wheel_actuators = {}
        for side in ('left', 'right'):
            for candidate in (f'n_wheel_{side}_motor', f'wheel_{side}_motor'):
                index = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, candidate)
                if index >= 0:
                    self.wheel_actuators[side] = int(index)
                    break
        if len(self.wheel_actuators) != 2:
            raise RuntimeError('the transport chassis has no wheel motors')
        # The deck's own rear pusher, as a retention device. C's rig made it a two-DOF
        # mechanism on purpose (a rigidly mounted pusher was measured infeasible), and the deck
        # it belongs to is the one that carries the tray, so this is the world's retention
        # mechanism rather than one invented here.
        self.pusher_command = {'lift': 0.0, 'slide': 0.0}
        self.pusher_actuators = {}
        for key, name in (('lift', 'c_pusher_lift'), ('slide', 'c_pusher_slide')):
            index = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, name)
            if index >= 0:
                self.pusher_actuators[key] = int(index)
        if len(self.pusher_actuators) != 2:
            raise RuntimeError('the deck has no rear pusher: no retention mechanism exists, '
                               'and a tray on free rollers does not stay on')
        self.pusher_limits = {'lift': list(self.model.actuator_ctrlrange
                                          [self.pusher_actuators['lift']]),
                              'slide': list(self.model.actuator_ctrlrange
                                           [self.pusher_actuators['slide']])}
        # Measured, not reasoned: see `_calibrate_wheel_signs`.
        self._calibrate_wheel_signs()

    # -- state ---------------------------------------------------------------

    def _set_state(self, qpos, qvel=None, time_s=None, *, phase):
        """The ONLY way this plant writes `data.qpos`, tagged with the phase it happened in.

        ★ Why an instrument and not a promise: the closed loop has to be able to say "the tray was
        never teleported", and "we do not write qpos" is a claim about code that nobody re-checks.
        Counting the writes per phase turns it into a number a judge can assert against zero.

        Phases: `initialise` (the one reset, before the run), `calibrate` (the wheel-sign
        measurement, which restores exactly what it used), and `run` -- which must stay absent.
        """
        self.data.qpos[:] = np.asarray(qpos, dtype=float)
        if qvel is not None:
            self.data.qvel[:] = np.asarray(qvel, dtype=float)
        if time_s is not None:
            self.data.time = float(time_s)
        self.qpos_writes[phase] = self.qpos_writes.get(phase, 0) + 1

    def reset(self):
        """Put the world at its home state. THE ONE `initialise` WRITE.

        When this plant is a GUEST on someone else's world (`_owns_world` False) the reset is
        refused rather than performed: writing `_home` over a world the humanoid's runtime is
        midway through would be a runtime qpos write, which H3 must have zero of. The caller is
        told instead of being quietly obeyed.
        """
        if getattr(self, 'safety_frozen', False):
            raise RuntimeError('safety hold cannot be cleared by resetting cargo state')
        if not getattr(self, '_owns_world', True):
            raise RuntimeError('this plant does not own the world: reset() would be a runtime '
                               'qpos write. The owner sets the home state once, before the run.')
        self._set_state(self._home, np.zeros(self.model.nv), phase='initialise')
        mujoco.mj_forward(self.model, self.data)

    def _derive(self):
        data = self.data
        meta = rig.build_model(str(ROOT / 'assets' / 'objects' / 'tray_v1.xml'))[1]
        pitch = float(meta['roller_pitch'])
        handoff = float(meta['recv_handoff_gap_m'])
        self.pitch = pitch
        self.handoff = handoff
        self.source_band = _band(self.model, data, 'c_fixed_roller')
        crown_geom = self.model.geom('c_fixed_roller_0_0').id
        self.crown_z = float(data.geom_xpos[crown_geom][2]
                             + self.model.geom_size[crown_geom][0])
        self.receiver_band = _band(self.model, data, 'c_recv_roller')
        self.tray_envelope_radius_m = collision_radius(self.model,
                                                       self.model.body('c_payload').id)
        self.deck_row = _band(self.model, data, 'c_deck_roller')
        chassis = float(data.xpos[self.model.body('n_base_link').id][0])
        self.chassis_at_home = chassis
        self.mount = float(data.xpos[self.model.body('c_deck').id][0]) - chassis
        datum_face = float(self._geom('w2_datum_face')[0]) - builder.STOP_HALF_X_M
        frame_centre = float(self._geom('c_deck_frame')[0])
        half = float(rig.DECK_FRAME_HALF[0])
        # the pose whose deck-frame leading face is `DATUM_OFFSET_M` short of the datum
        self.dock_x_receiver = chassis - (frame_centre + half - (datum_face - builder.DATUM_OFFSET_M))
        self.dock_x_source = chassis - (self.deck_row[0] - (self.source_band[-1] + pitch))
        # ★ The SOURCE station has NO approach leg, and that is a property of the world rather
        # than a convenience: the vehicle is parked with its deck already at the source dock, and
        # driving west from there is blocked within ~15 mm because the deck's roller row meets the
        # source band's roller row at the same height (both at CROWN_Z). Measured: the first
        # version asked for an approach 0.60 m west and the vehicle travelled 0.0143 m. So the
        # approach pose IS the dock pose and the leg is declared zero, which is what the run
        # reports. The RECEIVER station does have a real approach leg, because the vehicle arrives
        # there from the west.
        zero = {'approach_x_m': self.dock_x_source, 'approach_speed_mps': DRIVE_SPEED_MPS,
                'approach_timeout_s': DRIVE_TIMEOUT_S, 'dock_timeout_s': DRIVE_TIMEOUT_S}
        self.stations = {
            'station_c': {'id': 'station_c', 'dock_x_m': self.dock_x_source,
                          'approach_leg_m': 0.0, **zero},
            'station_b': {'id': 'station_b', 'approach_x_m': self.dock_x_receiver - 0.60,
                          'dock_x_m': self.dock_x_receiver, 'approach_leg_m': 0.60,
                          'approach_speed_mps': DRIVE_SPEED_MPS,
                          'approach_timeout_s': DRIVE_TIMEOUT_S, 'dock_timeout_s': DRIVE_TIMEOUT_S},
        }

    def _geom(self, name):
        index = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_GEOM, name)
        if index < 0:
            raise RuntimeError(f'the world has no geom {name!r}')
        return np.array(self.data.geom_xpos[index], dtype=float)

    def _hold(self):
        """The standing law, with the deck's pusher command layered on top.

        ★ `home_hold_ctrl` commands every actuator it does not own to zero, and the pusher is one of
        them -- so the blade was retracted again the moment the vehicle moved. A retention device
        that the hold law disarms as soon as the vehicle moves is not a retention device, and the
        measurement said so: the tray left the deck during transport.
        """
        ctrl = np.asarray(mw.home_hold_ctrl(self.model, self._home, self.data.qpos,
                                            self.data.qvel), dtype=float).reshape(-1)
        for key, index in self.pusher_actuators.items():
            ctrl[index] = float(self.pusher_command[key])
        return ctrl

    def deploy_pusher(self, *, lift, slide):
        """Command the deck's rear pusher. Declared as the deck's retention + push mechanism."""
        low, high = self.pusher_limits['lift']
        self.pusher_command['lift'] = float(min(max(lift, low), high))
        low, high = self.pusher_limits['slide']
        self.pusher_command['slide'] = float(min(max(slide, low), high))
        return dict(self.pusher_command)

    def pusher_position(self):
        """The pusher's own joint positions, reached through its ACTUATORS.

        ★ This used to look the joints up BY NAME -- `pusher_lift_joint` / `pusher_slide_joint`
        -- while the merged world names them `c_pusher_lift_joint` / `c_pusher_slide_joint`.
        `mj_name2id` returned -1, the branch was skipped, and the function returned `{}` SILENTLY;
        `transfer()` then handed that empty dict back as its own `pusher` field, so the retention
        device's state was never written into any report. Measured and reported in
        `reports/p4-h3-gating-01` (row `pusher_state_is_observable`).

        The fix is NOT to hard-code the `c_` prefix -- that is the same fragility one rename away.
        The plant already resolves and VALIDATES its two pusher actuators in `__init__` (it raises
        unless there are exactly two), so the joints are reached through them, by ownership. If the
        lookup cannot resolve, say so instead of returning a short dict.
        """
        out = {}
        for key, actuator in self.pusher_actuators.items():
            trnid = int(self.model.actuator_trnid[actuator][0])
            if trnid < 0:
                raise RuntimeError(
                    'the %s pusher actuator drives no joint, so the pusher position cannot be '
                    'reported; returning a short dict here is how this readback was dead before'
                    % key)
            out[key] = float(self.data.qpos[int(self.model.jnt_qposadr[trnid])])
        if set(out) != {'lift', 'slide'}:
            raise RuntimeError('the pusher position readback is incomplete: %r' % sorted(out))
        return out

    def _calibrate_wheel_signs(self):
        """Which sign of wheel rate moves the chassis +x -- MEASURED on this model, once.

        ★ The previous version returned
              -1 if cross(R @ jnt_axis, R @ (0, 0, -radius))[0] > 0 else 1
        and that arm is a point on the RIM in body coordinates. The rim turns with the wheel; the
        contact patch does not. So the lever arm rotated with the spin angle, and the returned sign
        FLIPPED whenever a wheel had accumulated a quarter turn of spin. Measured consequence: in
        `reports/w4-skills-04`, `cmd-004 DOCK` had a target 0.603 m to the EAST and drove 1.7766 m
        WEST at full dock speed until its 30 s timeout, and the `UNDOCK` after it drove back east
        -- because between those two commands the accumulated spin phase had crossed the flip.

        So this does not reason about MuJoCo's conventions. It commands each wheel ALONE and looks
        at which way the chassis actually went. Each wheel is probed separately because a pair
        wired oppositely would cancel, and that is a thing to DETECT rather than to average over:
        a wheel that cannot move the chassis raises, instead of silently yielding a sign.
        """
        save = (np.array(self.data.qpos), np.array(self.data.qvel), float(self.data.time))
        probe_rate = 1.0                      # rad/s: enough to see, not enough to disturb anything
        signs = {}
        for side in sorted(self.wheel_actuators):
            self._set_state(save[0], save[1], phase='calibrate')
            mujoco.mj_forward(self.model, self.data)
            start = self.chassis_x()
            for _ in range(20):
                self._tick(lambda s, _side=side: self._wheel_rate_torque(s, probe_rate)
                           if s == _side else 0.0)
            moved = self.chassis_x() - start
            if abs(moved) < 1e-7:
                raise RuntimeError(
                    f'the {side} wheel moved the chassis {moved:.3e} m when driven alone: a drive '
                    f'direction cannot be inferred from a wheel that does nothing, and returning '
                    f'one anyway is how the sign defect stayed hidden')
            signs[side] = 1.0 if moved > 0 else -1.0
        self._set_state(save[0], save[1], save[2], phase='calibrate')
        mujoco.mj_forward(self.model, self.data)
        self._signs = signs
        return dict(signs)

    def _wheel_signs(self):
        """The MEASURED signs, cached at construction.

        Deliberately not re-measured per call: the measurement moves the model, and a sign that can
        change between two calls is the defect this replaced. The value is a property of the
        vehicle, not of the moment.
        """
        if not hasattr(self, '_signs'):
            self._calibrate_wheel_signs()
        return dict(self._signs)

    def clock(self):
        """The plant's own time base. A skill result's start_s/end_s live in ONE domain,
        and mixing the coordinator's wall clock with the simulator's clock is a real defect
        the schema caught: `end_s 1.0 < start_s 100.0`."""
        return float(self.data.time)

    def chassis_x(self):
        return float(self.data.xpos[self.model.body('n_base_link').id][0])

    def chassis_speed(self):
        dof = int(self.model.jnt_dofadr[self.model.joint('n_slide_x').id])
        return float(self.data.qvel[dof])

    # -- motion --------------------------------------------------------------

    def _wheel_rate_torque(self, side, want_rate):
        """THE wheel controller. There is exactly one, and it is a rate servo.

        `drive_to` asks it for a moving setpoint; the brake asks it for zero. That is the whole
        difference between driving and stopping, and making it one function is what stops the two
        from drifting apart.

        ★ Why a brake was needed at all (measured, `_dbg_creep.py`, 2026-09-26): the wheel joints
        carry `damping 0.02`, `frictionloss 0.0`, `armature 0.004`, and the actuator's force range
        is unlimited -- so setting the torque to zero leaves a FREE wheel, and the vehicle coasts.
        The measured coast decays with a time constant of about 0.9 s: 33.8 mm/s falling to
        3.8 mm/s over two seconds, i.e. still 24.8 mm/s inside the contract's 0.5 s settle window
        and so over the 10 mm/s limit. Cutting torque is not braking; it is coasting.
        """
        dof = int(self.model.jnt_dofadr[self.model.joint(f'n_wheel_{side}_joint').id])
        return float(np.clip(WHEEL_KP * (float(want_rate) - float(self.data.qvel[dof])),
                             -WHEEL_TORQUE_LIMIT_NM, WHEEL_TORQUE_LIMIT_NM))

    def _control(self, wheel_rate_of_side):
        """One control VECTOR: the hold law, then the wheels. No physics step.

        Split out of `_tick` for H3, which has to command this plant AND the humanoid's runtime
        within the same control period and step exactly once. Duplicating "the hold law plus the
        wheel rates" into a second place is how the two would drift apart, so there is one.
        """
        ctrl = np.asarray(self._hold(), dtype=float).reshape(-1).copy()
        for side, actuator in self.wheel_actuators.items():
            ctrl[actuator] = wheel_rate_of_side(side)
        return ctrl

    def _tick(self, wheel_rate_of_side):
        """One control period: the hold law, then the wheels, then one physics step."""
        if getattr(self, 'safety_frozen', False):
            raise RuntimeError('SIMULATION_SAFETY_HOLD: external owner must not advance physics')
        if getattr(self, 'step_owner', None) is not None:
            return self.step_owner.step(lambda: self._control(wheel_rate_of_side))
        self.data.ctrl[:] = self._control(wheel_rate_of_side)
        mujoco.mj_step(self.model, self.data)

    def _commit_control(self, produce_control, *, advance=False):
        """Every transfer write/step uses the same gate; legacy default unchanged."""
        if getattr(self,'safety_frozen',False):
            raise RuntimeError('SIMULATION_SAFETY_HOLD: no control commit')
        owner=getattr(self,'step_owner',None)
        if owner is not None:
            return owner.commit(produce_control,advance=advance)
        self.data.ctrl[:]=produce_control()
        if advance:
            mujoco.mj_step(self.model,self.data)
        return float(self.data.time)

    def _band_control(self,spin,speed,deck_drive=None):
        ctrl=np.asarray(self._hold(),dtype=float).copy()
        for index in spin:ctrl[index]=speed
        if deck_drive is not None and deck_drive>=0:ctrl[deck_drive]=0.
        return ctrl

    def park_control(self):
        """The PARKED control vector for one period, WITHOUT stepping.

        ★ WHY H3 NEEDS THIS, AND THE MEASUREMENT THAT FORCED IT. The wheel motors are
        `biastype=0` (mjBIAS_NONE) with an all-zero `biasprm` -- they are RAW MOTORS, so
        `ctrl = 0` is ZERO TORQUE, not a brake. `_wheel_rate_torque`'s own docstring records the
        same thing from 2026-09-26: *"Cutting torque is not braking; it is coasting."*

        Measured consequence in H3's first correct-geometry run: for the 50 s during which the
        humanoid supplies the tray, nothing commanded this plant, so its wheels sat at zero torque
        and the vehicle **coasted from x 4.41372 to 8.45782 -- 4.04 m**. The chain's next
        `MOVE_TO_STATION` then had 4 m to recover and reported `NAV_FAILED`, and the whole
        downstream sequence collapsed into `DOCK_OUT_OF_TOLERANCE` / `TRANSFER_TIMEOUT` /
        `PAYLOAD_LOST`.

        The vehicle being parked while the humanoid loads it is not an H3 convenience, it is the
        scenario: a docked AMR waits. So this exposes the plant's own brake ("`drive_to` asks it
        for a moving setpoint; the brake asks it for zero") as one parked control period, and H3
        calls it every tick of the humanoid phase and then lets the runtime write its own
        actuators on top before the single `mj_step`.
        """
        return self._control(lambda side: self._wheel_rate_torque(side, 0.0))

    def drive_to(self, target_x, *, speed, timeout_s, reverse=False):
        if getattr(self, 'safety_frozen', False):
            return {'final_x_m': self.chassis_x(),
                    'error_x_m': self.chassis_x()-float(target_x), 'travelled_m': 0.0,
                    'peak_speed_mps': 0.0, 'stopped_confirmed': False,
                    'simulation_frozen': True, 'end_s': float(self.data.time)}
        signs = self._wheel_signs()
        steps = int(timeout_s / self.model.opt.timestep)
        travelled, peak = 0.0, 0.0
        previous = np.array(self.data.xpos[self.model.body('n_base_link').id][:2])
        for _ in range(steps):
            error = float(target_x) - self.chassis_x()
            if abs(error) < 0.003:
                break
            command = math.copysign(min(speed, abs(error) / 0.4 + 0.01), error)
            if reverse:
                command = -abs(command)
            rate = command / 0.04
            self._tick(lambda side: self._wheel_rate_torque(side, rate * signs[side]))
            now = np.array(self.data.xpos[self.model.body('n_base_link').id][:2])
            travelled += float(np.linalg.norm(now - previous))
            peak = max(peak, float(np.linalg.norm(now - previous)) / self.model.opt.timestep)
            previous = now
        # brake, then verify it STAYS stopped: a vehicle that is still creeping has not stopped,
        # and "stopped" is evidence a dock is allowed to use.
        brake = self._settle()
        return {'final_x_m': self.chassis_x(), 'error_x_m': self.chassis_x() - float(target_x),
                'travelled_m': travelled, 'peak_speed_mps': peak,
                'stopped_confirmed': brake['stopped_confirmed'],
                'held_speed_mps': brake['held_speed_mps'],
                'drift_m': brake['drift_m'],
                'brake_transient_peak_mps': brake['brake_transient_peak_mps'],
                'legacy_window_peak_mps': brake['legacy_window_peak_mps'],
                'end_s': float(self.data.time)}

    def _settle(self):
        """BRAKE, then verify it STAYS stopped. Two phases, and they are different claims.

        Phase 1 is the stop itself, and it is what `settle_duration_s` is FOR: a brake takes time,
        and a controller that is judged during its own transient is judged on the wrong thing.
        Phase 2 is the contract's real question -- *is it at rest?* -- and that is the JUDGED
        window. Both are reported, so a reader can see the transient as well as the verdict.

        `legacy_window_peak_mps` is the number the PREVIOUS version judged (max |v| over the second
        half of phase 1). It is still reported, so that the improvement can be attributed to the
        brake rather than to the window moving: with the old controller that same quantity is what
        failed, and with the brake it collapses. Reporting it costs one max() and removes the
        question.
        """
        if getattr(self, 'safety_frozen', False):
            return {'stopped_confirmed': False, 'held_speed_mps': abs(self.chassis_speed()),
                    'drift_m': 0.0, 'brake_transient_peak_mps': 0.0,
                    'legacy_window_peak_mps': 0.0, 'simulation_frozen': True}
        steps = int(STOPPED_HOLD_S / self.model.opt.timestep)
        half = steps // 2
        brake_peak = 0.0
        legacy = 0.0
        for step in range(steps):
            self._tick(lambda side: self._wheel_rate_torque(side, 0.0))
            speed = abs(self.chassis_speed())
            brake_peak = max(brake_peak, speed)
            if step >= half:
                legacy = max(legacy, speed)
        start = self.chassis_x()
        held = 0.0
        for _ in range(steps):
            self._tick(lambda side: self._wheel_rate_torque(side, 0.0))
            held = max(held, abs(self.chassis_speed()))
        drift = abs(self.chassis_x() - start)
        return {'brake_transient_peak_mps': brake_peak, 'held_speed_mps': held,
                'drift_m': drift, 'legacy_window_peak_mps': legacy,
                'stopped_confirmed': held <= STOPPED_SPEED_MPS and drift <= DRIFT_LIMIT_M}

    def stop(self):
        brake = self._settle()
        return {'stopped_confirmed': brake['stopped_confirmed'],
                'held_speed_mps': brake['held_speed_mps'],
                'drift_m': brake['drift_m'],
                'drift_limit_m': DRIFT_LIMIT_M,
                'brake_transient_peak_mps': brake['brake_transient_peak_mps'],
                'legacy_window_peak_mps': brake['legacy_window_peak_mps'],
                'end_s': float(self.data.time)}

    # -- measurement ---------------------------------------------------------

    def dock_residuals(self, station_id):
        frame = self._geom('c_deck_frame')
        rotation = self.data.geom_xmat[
            mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_GEOM, 'c_deck_frame')].reshape(3, 3)
        half = float(rig.DECK_FRAME_HALF[0])
        if station_id == 'station_b':
            channel_y = float((self._geom('w2_chamfer_face_posy')[1]
                               + self._geom('w2_chamfer_face_negy')[1]) / 2.0)
            datum = float(self._geom('w2_datum_face')[0]) - builder.STOP_HALF_X_M
            return {'lateral_m': float(frame[1]) - channel_y,
                    'longitudinal_m': float(frame[0]) + half - datum,
                    'yaw_rad': math.atan2(rotation[1, 0], rotation[0, 0])}
        # The source station has no mechanism, so its residual is measured against C's own
        # criterion -- and that criterion is about the deck's FIRST crown against the band's LAST
        # one, one pitch apart. The first version used the LAST crown against the handoff gap,
        # which is the RECEIVER's criterion, and reported a 550 mm error for a correctly parked
        # vehicle.
        from dock_measurements import source_residual
        # Cached crown x + local slide ignores the free chassis translation and
        # yaw; lateral=0 is not a measurement. Use current first-crown state.
        source=[self.source_band[-1],self._geom('c_fixed_roller_0_0')[1]]
        return source_residual(source,self._geom('c_deck_roller_0')[:2],self.pitch,rotation)

    def _hold_deck_slide(self):
        joint = self.model.joint('c_deck_slide').id
        return float(self.data.qpos[int(self.model.jnt_qposadr[joint])])

    #: Which roller row each band's geoms belong to, by geometry-name prefix. Derived from the
    #: world's own naming rather than from a typed list of body names.
    ROWS = (('source_band', 'c_fixed_roller'), ('deck', 'c_deck_roller'),
            ('receiver_band', 'c_recv_roller'))

    def tray_support(self):
        """Which roller rows the tray is actually standing on, from the solver contact list.

        ★ The first version answered the different question "which station is the tray NEAR", from a
        +/-0.2 m x window. In this world the source band and the deck row are 75 mm apart, so those
        windows OVERLAP: the tray was still reported as being on the source band well after it had
        left it, and because the transfer stop condition was "the tray is in the deck zone", a
        successful crossing could never be observed. Being ON a row and being NEAR a station are
        different questions and this answers the first one, from contacts rather than from x.
        """
        tray = self.model.body('c_payload').id
        touched = set()
        for i in range(self.data.ncon):
            contact = self.data.contact[i]
            g1, g2 = int(contact.geom[0]), int(contact.geom[1])
            if tray not in (int(self.model.geom_bodyid[g1]), int(self.model.geom_bodyid[g2])):
                continue
            other = g2 if int(self.model.geom_bodyid[g1]) == tray else g1
            name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_GEOM, other) or ''
            for row, prefix in self.ROWS:
                if name.startswith(prefix):
                    touched.add(row)
        return sorted(touched)

    def tray_state(self):
        body = self.model.body('c_payload').id
        lo, hi = collision_bounds(self.model, self.data, body)
        x, z_bottom = float((lo[0] + hi[0]) / 2.0), float(lo[2])
        touched = self.tray_support()
        return {'zone': self.zone_of(x, touched), 'supported_by': touched, 'x_m': x,
                'x_trailing_m': float(lo[0]), 'x_leading_m': float(hi[0]),
                'z_bottom_m': z_bottom,
                'on_crowns': abs(z_bottom - self.crown_z) < 0.02,
                'end_s': float(self.data.time)}

    def row_window(self, row):
        """The band's own x window, in world coordinates, from the current chassis position."""
        if row == 'source_band':
            return self.source_band[0], self.source_band[-1]
        if row == 'receiver_band':
            return self.receiver_band[0], self.receiver_band[-1]
        shift = self.chassis_x() - self.chassis_at_home
        return min(self.deck_row) + shift, max(self.deck_row) + shift

    def zone_of(self, x, touched=None):
        """Which row the tray has reached.

        Taken from the rows it is IN CONTACT with, and only when it touches nothing (it is in
        flight between two rows) from the x windows. `source_band` wins whenever it is still
        touched, because the only direction this scenario moves is east: if the tray is still on
        the band it came from, the leg is not finished, whatever else it is also touching.
        """
        rows = self.tray_support() if touched is None else list(touched)
        if 'source_band' in rows:
            return 'source_band'
        if len(rows) == 1:
            return rows[0]
        if rows:
            return max(rows, key=lambda row: self.row_window(row)[1])
        for row, _prefix in self.ROWS:
            low, high = self.row_window(row)
            if low - 0.2 <= x <= high + 0.2:
                return row
        return 'unknown'

    def observe(self):
        return {'chassis_x_m': self.chassis_x(), 'chassis_speed_mps': self.chassis_speed(),
                'tray': self.tray_state(), 'end_s': float(self.data.time)}

    # -- the transfer --------------------------------------------------------

    def _stop_transfer(self):
        """Zero band setpoints and measure vehicle AND full tray motion after braking.

        Two existing STOPPED_HOLD_S windows: brake first, then judge the held
        window. This is an additional declared emergency-stop reserve, not a
        successful transfer and not proof that cargo custody changed.
        """
        if getattr(self, 'safety_frozen', False):
            return dict(self._frozen_brake)
        body = self.model.body('c_payload').id
        joint = int(self.model.body_jntadr[body])
        dof = int(self.model.jnt_dofadr[joint])
        steps = int(STOPPED_HOLD_S / self.model.opt.timestep)
        for _ in range(steps):
            self._tick(lambda side: self._wheel_rate_torque(side, 0.0))
        tray_start = np.array(self.data.xpos[body], dtype=float)
        chassis_start = self.chassis_x()
        tray_peak, vehicle_peak = 0.0, 0.0
        linear_peak, angular_peak, window_motion = 0.0, 0.0, 0.0
        for _ in range(steps):
            self._tick(lambda side: self._wheel_rate_torque(side, 0.0))
            v = self.data.qvel[dof:dof+6]
            linear_peak = max(linear_peak, float(np.linalg.norm(v[:3])))
            angular_peak = max(angular_peak, float(np.linalg.norm(v[3:])))
            window_motion = max(window_motion, float(np.linalg.norm(self.data.xpos[body]-tray_start)))
            tray_peak = max(tray_peak, float(np.linalg.norm(v[:3])
                            + np.linalg.norm(v[3:]) * self.tray_envelope_radius_m))
            vehicle_peak = max(vehicle_peak, abs(self.chassis_speed()))
        tray_drift = float(np.linalg.norm(self.data.xpos[body] - tray_start))
        vehicle_drift = abs(self.chassis_x() - chassis_start)
        confirmed = (tray_peak <= STOPPED_SPEED_MPS and vehicle_peak <= STOPPED_SPEED_MPS
                     and tray_drift <= DRIFT_LIMIT_M and vehicle_drift <= DRIFT_LIMIT_M)
        result = {'stopped_confirmed': bool(confirmed), 'tray_held_speed_bound_mps': tray_peak,
                'vehicle_held_speed_mps': vehicle_peak, 'tray_drift_m': tray_drift,
                'tray_linear_peak_mps': linear_peak, 'tray_angular_peak_radps': angular_peak,
                'tray_window_motion_m': window_motion, 'tray_joint_dof_address': dof,
                'vehicle_drift_m': vehicle_drift,
                'stop_reserve_sim_s': 2 * STOPPED_HOLD_S,
                'end_s': float(self.data.time)}
        if not confirmed:
            # This controller stops advancing its world, not a hardware e-stop.
            # A guest world's external owner must also honor this hold request.
            self.safety_frozen = True
            result['simulation_frozen'] = True
            result['hold_mode'] = 'SIM_FROZEN_NO_PHYSICAL_STOP_PROOF'
            self._frozen_brake = dict(result)
        return result

    def transfer(self, *, direction, timeout_s):
        """Spin the declared band the tray has to cross, for a bounded time, then report.

        The three speeds are C's own, imported from the chain that validated them. Nothing here
        teleports a body or writes a qpos: what moves the tray is the rollers, which is the claim
        C's mechanism makes.
        """
        if direction not in ('onto_deck', 'onto_receiver'):
            raise ValueError('unknown transfer direction: %r' % direction)
        if (not math.isfinite(timeout_s)
                or timeout_s < float(self.model.opt.timestep)):
            raise ValueError('transfer budget must be finite and allow at least one step')
        if getattr(self, 'safety_frozen', False):
            return {'state': 'UNKNOWN', 'interrupted': True,
                    'reason_code': 'CANCEL_UNCONFIRMED',
                    'simulation_frozen': True, 'brake': dict(self._frozen_brake),
                    'direction': direction, 'pusher_seated': False,
                    'tray': self.tray_state(),
                    'start_s': float(self.data.time), 'end_s': float(self.data.time)}

        def actuators(prefix):
            return [i for i in range(self.model.nu)
                    if (mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, i) or ''
                        ).startswith(prefix)]

        fixed = actuators('c_fixed_roller')
        deck_rollers = actuators('c_deck_roller')
        receiver = actuators('c_recv_roller')
        deck_drive = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, 'c_deck_drive')
        # ★ BOTH bands are driven, which is what makes a transfer a transfer. C's own chain spins
        # `fixed + deck_roller` to load and `deck_roller + recv_roller` to unload; the first version
        # of this function spun ONE band and left the neighbours servo-locked to zero rate, so the
        # tray pressed against a braked roller and could not cross. Measured: every transfer leg of
        # `reports/w5-loop-01` spent its whole budget with the tray still on the source band.
        if direction == 'onto_deck':
            spin, speed, toward, leaving = (fixed + deck_rollers, cchain.ROLLER_SPEED, 'deck',
                                            'source_band')
        else:
            spin, speed, toward, leaving = (deck_rollers + receiver, cchain.UNLOAD_ROLLER_SPEED,
                                            'receiver_band', 'deck')
        started = float(self.data.time)
        touched = self.tray_support()
        # A delivered tray is terminal: do not spin a single additional tick. Previously
        # the already-satisfied arrival test ran AFTER a driven step, then a 1.5 s
        # seating loop ran even for unloading, pushing delivered goods off the receiver.
        # Only destination-exclusive physical support permits this no-op result.
        receiver_target = self.row_window(toward)[0] + 1.5 * self.pitch
        row_start, row_end = self.row_window(toward)
        if direction == 'onto_receiver':
            # Actual-yaw clearing alone failed after UNDOCK: the tray rotated
            # and its handles returned over the entrance. Reserve its complete
            # rotation envelope plus the existing stopped-window drift budget.
            receiver_target = max(receiver_target, row_start
                                  + self.tray_envelope_radius_m
                                  + SEAT_MARGIN_M + DRIFT_LIMIT_M)

        def positioned_on_receiver(observation):
            # Longitudinal clearing, including handles at the actual yaw. This
            # is NOT lateral containment: the design permits handle overhang.
            return (observation['x_m'] >= receiver_target
                    and observation['x_trailing_m'] >= row_start + SEAT_MARGIN_M
                    and observation['x_leading_m'] <= row_end - SEAT_MARGIN_M)

        arrival = self.tray_state()
        safely_received = (touched == [toward] and arrival['on_crowns']
                           and (direction == 'onto_deck'
                                or positioned_on_receiver(arrival)))
        if touched == [toward] or leaving not in touched:
            LogisticsPlant._commit_control(self,lambda:self._hold())
            return {'state': 'RECEIVED' if safely_received else 'UNKNOWN',
                    'direction': direction, 'expected_zone': toward,
                    'left_the_row': leaving, 'driven_actuators': 0,
                    'roller_speed': 0.0, 'pusher_seated': False,
                    'pusher': self.pusher_position(),
                    'pusher_command': dict(self.pusher_command),
                    'tray': self.tray_state(), 'start_s': started, 'end_s': started,
                    'already_received': safely_received}
        steps = int(timeout_s / self.model.opt.timestep)
        state = 'TRANSFERRING'
        guard = getattr(self, 'motion_guard', None)
        interruption = None
        # The deck's blade is DEPLOYED before anything is asked to move, on the design intent that
        # it stops the tray walking back off the deck.
        #
        # ⛔ MEASURED 2026-09-29, AND THE INTENT IS NOT REALISED (`reports/p4-h3-gating-01`,
        # rows `the_retention_blade_can_reach_its_command` and `pusher_state_is_observable`):
        #   * `c_pusher_lift_joint` has a range of **[0, 0.001] m -- 1 mm** of travel;
        #   * commanded its FULL value the joint settles at **-0.000215 m**, i.e. below its own
        #     lower limit, and the blade body rises **0.12 mm**;
        #   * the force balance says why: `qfrc_actuator` reaches **2.43 N** against **4.9 N** of
        #     gravity on the 0.5 kg blade, with NO contact on the blade -- the actuator cannot lift
        #     its own blade;
        #   * and geometrically it could not retain anything anyway: the blade's bottom (z 0.885) is
        #     **~11 mm above** the tray's top (~0.874) and its x footprint (6.169) is **0.155 m west**
        #     of the deck row (which starts at 6.324), so blade and tray never overlap.
        # So the deck's retention is NOT provided by this blade. What actually keeps the tray in
        # place during transport is UNVERIFIED -- that is `P4-BELT-03`'s judged row
        # `tray_does_not_slide_on_the_deck`, which measures the tray's shift while the vehicle moves.
        # This comment is corrected rather than the geometry changed, because changing the geometry
        # would alter `assets/world_w5_h085_loop.xml` and invalidate the accepted H2/H3 reports.
        lift_full = float(self.pusher_limits['lift'][1])
        if guard is not None:
            decision = guard.check()
            if not decision['allowed']:
                interruption = decision
        if interruption is None:
            self.deploy_pusher(lift=lift_full, slide=self.pusher_command['slide'])
        for step in range(steps):
            if interruption is not None:
                break
            if guard is not None and step > 0:
                decision = guard.check()
                if not decision['allowed']:
                    interruption = decision
                    break
            LogisticsPlant._commit_control(self,
                lambda:LogisticsPlant._band_control(self,spin,speed,deck_drive),advance=True)
            # ★ The stop condition is CONTACT-BASED: the tray has arrived when it is standing on
            # the destination row and has left the row it came from. The first version tested a
            # zone computed from overlapping +/-0.2 m windows, which in this world never flips.
            touched = self.tray_support()
            arrival = self.tray_state()
            # First receiver contact is not stable delivery. Stop above interior
            # crowns, not at the unsupported entrance edge. This target derives
            # from the frozen roller pitch; geometry and acceptance stay unchanged.
            positioned = (direction == 'onto_deck'
                          or positioned_on_receiver(arrival))
            if (toward in touched and leaving not in touched and positioned
                    and arrival['on_crowns']
                    and abs(self.chassis_speed()) <= STOPPED_SPEED_MPS):
                state = 'RECEIVED'
                break
        if guard is not None and state != 'RECEIVED' and interruption is None:
            interruption = guard.stop('TRANSFER_TIMEOUT', 'transfer driving budget exhausted')
        # Then STROKE the blade east, slowly, to seat the tray against the deck's forward limit.
        # This is the deck's own mechanism doing the seating, and it is also what leaves the tray
        # behind a raised blade for the transport leg.
        seated = False
        row = self.row_window(toward)
        # Seating is a LOADING operation only, never an unloading run-on or a
        # post-timeout motion. Cut roller setpoints as soon as unloading arrives.
        stroke_steps = (int(SEAT_S / self.model.opt.timestep)
                        if state == 'RECEIVED' and direction == 'onto_deck' else 0)
        slide = self.pusher_command['slide']
        for _ in range(stroke_steps):
            if guard is not None:
                decision = guard.check()
                if not decision['allowed']:
                    interruption = decision
                    break
            if state == 'RECEIVED' and direction == 'onto_deck':
                trailing = self.tray_state()['x_trailing_m']
                if trailing >= row[0] + SEAT_MARGIN_M:
                    seated = True
                    break
                slide = min(slide + SEAT_SPEED_MPS * self.model.opt.timestep,
                            float(self.pusher_limits['slide'][1]))
            self.deploy_pusher(lift=lift_full, slide=slide)
            LogisticsPlant._commit_control(self,
                lambda:LogisticsPlant._band_control(self,spin,speed),advance=True)
        if direction == 'onto_receiver':
            # The tray is on the receiver, so the deck is empty and the blade goes back down: a
            # retention device left deployed on an empty deck would block the next load.
            self.deploy_pusher(lift=0.0, slide=0.0)
        LogisticsPlant._commit_control(self,lambda:self._hold())
        if interruption is not None:
            brake = self._stop_transfer()
            return {'state': 'STOPPED' if brake['stopped_confirmed'] else 'UNKNOWN',
                    'interrupted': True, 'reason_code': interruption['reason_code'],
                    'runtime_permission': interruption, 'brake': brake,
                    'direction': direction, 'expected_zone': toward,
                    'left_the_row': leaving, 'tray': self.tray_state(),
                    'pusher': self.pusher_position(), 'pusher_seated': False,
                    'start_s': started, 'end_s': float(self.data.time)}
        return {'state': state, 'direction': direction, 'expected_zone': toward,
                'left_the_row': leaving, 'driven_actuators': len(spin), 'roller_speed': speed,
                'pusher_seated': seated, 'pusher': self.pusher_position(),
                'pusher_command': dict(self.pusher_command),
                'tray': self.tray_state(), 'start_s': started, 'end_s': float(self.data.time)}
