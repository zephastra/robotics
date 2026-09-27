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


class LogisticsPlant:
    """The plant object `workcell.adapters.sim.SkillAdapter` drives."""

    def __init__(self, world=WORLD):
        self.world = Path(world)
        bp3.install()
        self.model = mujoco.MjModel.from_xml_path(str(self.world))
        self.data = mujoco.MjData(self.model)
        self._home, _ctrl, _applied, _notes = mw.merged_home(self.model)
        self._home = np.asarray(self._home, dtype=float)
        #: Every write to `data.qpos`, counted per phase. See `_set_state`.
        self.qpos_writes = {}
        self.reset()
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
        self.receiver_band = _band(self.model, data, 'c_recv_roller')
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
        out = {}
        for key, joint in (('lift', 'pusher_lift_joint'), ('slide', 'pusher_slide_joint')):
            index = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, joint)
            if index >= 0:
                out[key] = float(self.data.qpos[int(self.model.jnt_qposadr[index])])
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

    def _tick(self, wheel_rate_of_side):
        """One control period: the hold law, then the wheels, then one physics step."""
        self.data.ctrl[:] = self._hold()
        for side, actuator in self.wheel_actuators.items():
            self.data.ctrl[actuator] = wheel_rate_of_side(side)
        mujoco.mj_step(self.model, self.data)

    def drive_to(self, target_x, *, speed, timeout_s, reverse=False):
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
        slide = float(self._hold_deck_slide())
        deck_first = float(self.deck_row[0]) + slide
        return {'lateral_m': 0.0,
                'longitudinal_m': (self.source_band[-1] + self.pitch) - deck_first,
                'yaw_rad': math.atan2(rotation[1, 0], rotation[0, 0])}

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
        lo = np.full(3, np.inf)
        hi = np.full(3, -np.inf)
        for g in range(self.model.ngeom):
            if self.model.geom_bodyid[g] != body:
                continue
            centre = self.data.geom_xpos[g]
            size = self.model.geom_size[g]
            half = size if self.model.geom_type[g] == mujoco.mjtGeom.mjGEOM_BOX \
                else np.full(3, max(size[0], size[1], size[2]))
            lo, hi = np.minimum(lo, centre - half), np.maximum(hi, centre + half)
        x, z_bottom = float((lo[0] + hi[0]) / 2.0), float(lo[2])
        touched = self.tray_support()
        return {'zone': self.zone_of(x, touched), 'supported_by': touched, 'x_m': x,
                'x_trailing_m': float(lo[0]), 'x_leading_m': float(hi[0]),
                'z_bottom_m': z_bottom,
                'on_crowns': abs(z_bottom - float(rig.CROWN_Z)) < 0.02,
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

    def transfer(self, *, direction, timeout_s):
        """Spin the declared band the tray has to cross, for a bounded time, then report.

        The three speeds are C's own, imported from the chain that validated them. Nothing here
        teleports a body or writes a qpos: what moves the tray is the rollers, which is the claim
        C's mechanism makes.
        """
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
        steps = int(timeout_s / self.model.opt.timestep)
        state = 'TRANSFERRING'
        # The deck's blade goes UP before anything is asked to move: it is what stops the tray
        # walking back off the deck, and it has to be deployed before the rollers can push the tray
        # against it.
        lift_full = float(self.pusher_limits['lift'][1])
        self.deploy_pusher(lift=lift_full, slide=self.pusher_command['slide'])
        for _ in range(steps):
            self.data.ctrl[:] = self._hold()
            for i in spin:
                self.data.ctrl[i] = speed
            if deck_drive >= 0:
                self.data.ctrl[deck_drive] = 0.0
            mujoco.mj_step(self.model, self.data)
            # ★ The stop condition is CONTACT-BASED: the tray has arrived when it is standing on
            # the destination row and has left the row it came from. The first version tested a
            # zone computed from overlapping +/-0.2 m windows, which in this world never flips.
            touched = self.tray_support()
            if toward in touched and leaving not in touched and abs(self.chassis_speed()) < 1.0:
                state = 'RECEIVED'
                break
        # Then STROKE the blade east, slowly, to seat the tray against the deck's forward limit.
        # This is the deck's own mechanism doing the seating, and it is also what leaves the tray
        # behind a raised blade for the transport leg.
        seated = False
        row = self.row_window(toward)
        stroke_steps = int(SEAT_S / self.model.opt.timestep)
        slide = self.pusher_command['slide']
        for _ in range(stroke_steps):
            if state == 'RECEIVED' and direction == 'onto_deck':
                trailing = self.tray_state()['x_trailing_m']
                if trailing >= row[0] + SEAT_MARGIN_M:
                    seated = True
                    break
                slide = min(slide + SEAT_SPEED_MPS * self.model.opt.timestep,
                            float(self.pusher_limits['slide'][1]))
            self.deploy_pusher(lift=lift_full, slide=slide)
            self.data.ctrl[:] = self._hold()
            for i in spin:
                self.data.ctrl[i] = speed
            mujoco.mj_step(self.model, self.data)
        if direction == 'onto_receiver':
            # The tray is on the receiver, so the deck is empty and the blade goes back down: a
            # retention device left deployed on an empty deck would block the next load.
            self.deploy_pusher(lift=0.0, slide=0.0)
        self.data.ctrl[:] = self._hold()
        return {'state': state, 'direction': direction, 'expected_zone': toward,
                'left_the_row': leaving, 'driven_actuators': len(spin), 'roller_speed': speed,
                'pusher_seated': seated, 'pusher': self.pusher_position(),
                'pusher_command': dict(self.pusher_command),
                'tray': self.tray_state(), 'start_s': started, 'end_s': float(self.data.time)}
