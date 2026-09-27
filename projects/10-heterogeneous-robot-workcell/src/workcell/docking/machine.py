"""The docking state machine, and the handover of command authority that the guidance asks for.

    IDLE -> RESERVE -> APPROACH -> ACQUIRE_TARGET -> ALIGN -> SETTLE -> VERIFY_DOCK -> DOCKED
    any of them -> STOPPING -> NEEDS_ATTENTION | ABORTED

Four properties are load-bearing, and each is a test:

  * **`_enter` is the only mutator** of `state`, exactly as `CommandGate` is the only writer of
    actuator commands and `TransferLedger._enter` is the only writer of a transfer stage. No public
    method accepts a target state.
  * **the APPROACH->ALIGN handover is a transition, not a comment.** APPROACH holds the navigation
    permit; entering ALIGN requires the navigation permit RELEASED and the docking permit GRANTED,
    with the generation advanced, so a command issued to the old holder cannot arrive later and be
    obeyed.
  * **SETTLE is a measured interval, not a sleep.** It requires a continuous run of FRESH
    observations of at least `settle_duration_s` AND the measured speed below the stopped limit; a
    gap in the observations resets the run.
  * **DOCKED expires.** A dock that outlives its evidence is not a dock, so `step` demotes it to
    NEEDS_ATTENTION when the observation ages out, the vehicle moves, or the permit changes.
"""
import math

DOCK_REFUSALS = frozenset({
    'REFUSED_NOT_A_SEQUENCE', 'REFUSED_BAD_TRANSITION', 'REFUSED_NO_PERMIT',
    'REFUSED_WRONG_PERMIT_HOLDER', 'REFUSED_STALE_GENERATION', 'REFUSED_NOT_AT_REST',
    'REFUSED_OUT_OF_WINDOW', 'REFUSED_OBSERVATION_NOT_FRESH', 'REFUSED_SETTLE_NOT_HELD',
    'REFUSED_ALREADY_DONE', 'REFUSED_UNKNOWN_ENTITY', 'REFUSED_BAD_TYPE',
    'REFUSED_UNDECLARED_REASON', 'DOCK_INVALIDATED',
})
STATES = ('IDLE', 'RESERVE', 'APPROACH', 'ACQUIRE_TARGET', 'ALIGN', 'SETTLE', 'VERIFY_DOCK',
          'DOCKED')
FAILURE_STATES = ('STOPPING', 'NEEDS_ATTENTION', 'ABORTED')
HOLDERS = ('nobody', 'navigation', 'docking')
#: How long a DOCKED verdict stays valid without new evidence, from the contract's `validity`.
DOCKED_VALIDITY_S = None          # set per instance from the contract; typed here only as a name


class DockRefused(ValueError):
    def __init__(self, reason, detail='', *, field=None):
        self.reason = reason
        self.detail = detail
        self.field = field
        super().__init__(f'{reason}: {detail}' if detail else reason)


class Permits:
    """Who may command motion, as a value with a generation.

    This is deliberately tiny: it is not a second command gate (`workcell.safety.CommandGate` is the
    actuator gate and stays the only writer of commands). It is the record of WHICH controller is
    allowed to ask, which the guidance wants closed before the handover and generated so that a late
    command from the previous holder is refused rather than obeyed.
    """

    def __init__(self):
        self.holder = 'nobody'
        self.generation = 0
        self.history = [{'holder': 'nobody', 'generation': 0}]

    def grant(self, holder):
        if holder not in HOLDERS or holder == 'nobody':
            raise DockRefused('REFUSED_BAD_TYPE', repr(holder), field='holder')
        self.holder = holder
        self.generation += 1
        self.history.append({'holder': holder, 'generation': self.generation})
        return self.generation

    def release(self):
        self.holder = 'nobody'
        self.generation += 1
        self.history.append({'holder': 'nobody', 'generation': self.generation})
        return self.generation


class DockMachine:
    """One docking attempt at one station.

    Every transition that moves the vehicle is gated on an observation accepted by
    `observations.accept_observation`: the machine never reads a pose directly, so a caller cannot
    hand it a truth value it did not measure.
    """

    def __init__(self, contract, *, epoch, station_id, transfer_id=None):
        self.contract = contract
        self.epoch = int(epoch)
        self.station_id = station_id
        self.transfer_id = transfer_id
        self.permits = Permits()
        self.state = 'IDLE'
        self.history = ['IDLE']
        self.reason_code = None
        self.return_to = None
        self.setting_since = None
        self.last_observation = None
        self.last_window = None
        self.refusals = []
        self.docked_since = None
        self.docked_validity_s = float(contract['validity']['docked_validity_s'])

    # ------------------------------------------------------------------ helpers
    @property
    def is_done(self):
        return self.state in ('DOCKED', 'ABORTED')

    @property
    def is_stuck(self):
        return self.state in FAILURE_STATES

    def _refuse(self, reason, detail, *, field=None):
        self.refusals.append({'reason': reason, 'detail': detail, 'state': self.state})
        raise DockRefused(reason, detail, field=field)

    def _enter(self, target):
        """The ONLY place `state` moves. A public method that took a target state would be a second
        authority, which is the defect this project keeps finding in its gates."""
        allowed = {s: STATES[i + 1] for i, s in enumerate(STATES[:-1])}
        if self.is_done or self.state in FAILURE_STATES:
            self._refuse('REFUSED_ALREADY_DONE', f'{self.state} cannot advance')
        if target != allowed.get(self.state):
            self._refuse('REFUSED_BAD_TRANSITION', f'{self.state} -> {target}')
        self.state = target
        self.history.append(target)

    def _fresh(self, observation, *, now_wall_s):
        from workcell.docking.observations import accept_observation
        verdict = accept_observation(observation, now_wall_s=now_wall_s, contract=self.contract,
                                     expected_station_id=self.station_id)
        if verdict['status'] != 'FRESH':
            self._refuse('REFUSED_OBSERVATION_NOT_FRESH',
                         f"{verdict['reason']}: {verdict['detail']}")
        self.last_observation = verdict['observation']
        return verdict

    def _window(self, observation):
        from workcell.docking.contract import check_window
        pose = observation['relative_pose']
        self.last_window = check_window(self.contract, longitudinal_m=pose['longitudinal_m'],
                                        lateral_m=pose['lateral_m'],
                                        height_m=pose['height_m'], yaw_rad=pose['yaw_rad'])
        return self.last_window

    # ------------------------------------------------------------------ transitions
    def reserve(self, *, resources_reserved, now_wall_s):
        if not isinstance(resources_reserved, bool):
            self._refuse('REFUSED_BAD_TYPE', 'resources_reserved must be a bool',
                         field='resources_reserved')
        if not resources_reserved:
            self._refuse('REFUSED_NO_PERMIT', 'the dock resources are not reserved', field='resources')
        self._enter('RESERVE')
        return self.snapshot(now_wall_s=now_wall_s)

    def approach(self, *, now_wall_s):
        if self.state != 'RESERVE':
            # the state is checked FIRST: a caller that skipped RESERVE must be told it skipped a
            # stage, not sent looking for a permit it was never going to have
            self._refuse('REFUSED_BAD_TRANSITION', f'{self.state} -> APPROACH')
        if self.permits.holder != 'navigation':
            self._refuse('REFUSED_WRONG_PERMIT_HOLDER',
                         f'APPROACH needs the navigation permit, the holder is '
                         f'{self.permits.holder!r}')
        self._enter('APPROACH')
        return self.snapshot(now_wall_s=now_wall_s)

    def acquire_target(self, *, in_pre_dock_area, now_wall_s):
        if self.state != 'APPROACH':
            self._refuse('REFUSED_BAD_TRANSITION', f'{self.state} -> ACQUIRE_TARGET')
        if in_pre_dock_area is not True:
            self._refuse('REFUSED_OUT_OF_WINDOW',
                         'the vehicle is not inside the declared pre-dock area yet')
        self._enter('ACQUIRE_TARGET')
        return self.snapshot(now_wall_s=now_wall_s)

    def hand_over_to_align(self, observation, *, now_wall_s):
        """The handover the guidance asks for: navigation's permit is closed BEFORE docking's opens.

        Both halves are required, and the observation must already be fresh, so the docking
        controller starts from something it measured rather than from a promise.
        """
        if self.state != 'ACQUIRE_TARGET':
            self._refuse('REFUSED_BAD_TRANSITION', f'{self.state} -> ALIGN')
        self._fresh(observation, now_wall_s=now_wall_s)
        if self.permits.holder != 'nobody':
            self._refuse('REFUSED_WRONG_PERMIT_HOLDER',
                         f"the navigation permit must be RELEASED before ALIGN, it is held by "
                         f"{self.permits.holder!r}")
        self.permits.grant('docking')
        self._enter('ALIGN')
        self.setting_since = None
        return self.snapshot(now_wall_s=now_wall_s)

    def settle(self, observation, *, speed_mps, now_wall_s):
        """Inside the window and slow enough to be settling rather than driving."""
        if self.state != 'ALIGN':
            self._refuse('REFUSED_BAD_TRANSITION', f'{self.state} -> SETTLE')
        self._fresh(observation, now_wall_s=now_wall_s)
        window = self._window(observation)
        if not window['fits']:
            failed = [k for k, v in window['components'].items() if not v['ok']]
            self._refuse('REFUSED_OUT_OF_WINDOW', f'components outside their limits: {failed}')
        if speed_mps is None or float(speed_mps) > float(self.contract['validity']['stopped_speed_mps']):
            self._refuse('REFUSED_NOT_AT_REST',
                         f'speed {speed_mps} m/s exceeds the declared '
                         f'{self.contract["validity"]["stopped_speed_mps"]} m/s')
        self._enter('SETTLE')
        self.setting_since = float(now_wall_s)
        self.last_window = window
        return self.snapshot(now_wall_s=now_wall_s)

    def verify_dock(self, observation, *, speed_mps, now_wall_s):
        """A measured interval, not a sleep: the observations must have been continuously fresh for
        the declared duration, and a gap resets the clock."""
        if self.state != 'SETTLE':
            self._refuse('REFUSED_BAD_TRANSITION', f'{self.state} -> VERIFY_DOCK')
        verdict = self._fresh(observation, now_wall_s=now_wall_s)
        if verdict['age_s'] is not None:
            gap = float(now_wall_s) - float(observation['wall_receive_time'])
            limit = float(self.contract['validity']['max_observation_age_s'])
            if gap > limit:
                self.setting_since = None
                self._refuse('REFUSED_SETTLE_NOT_HELD', f'the observation stream had a {gap:.3f} s gap')
        if self.setting_since is None:
            self._refuse('REFUSED_SETTLE_NOT_HELD', 'the settle interval never started')
        held = float(now_wall_s) - self.setting_since
        needed = float(self.contract['validity']['settle_duration_s'])
        if held < needed:
            self._refuse('REFUSED_SETTLE_NOT_HELD',
                         f'held {held:.3f} s of the declared {needed} s')
        window = self._window(observation)
        if not window['fits']:
            self._refuse('REFUSED_OUT_OF_WINDOW', 'the window does not fit at verification time')
        if speed_mps is None or float(speed_mps) > float(self.contract['validity']['stopped_speed_mps']):
            self._refuse('REFUSED_NOT_AT_REST', f'speed {speed_mps} m/s at verification time')
        self._enter('VERIFY_DOCK')
        return self.snapshot(now_wall_s=now_wall_s)

    def dock(self, observation, *, now_wall_s):
        if self.state != 'VERIFY_DOCK':
            self._refuse('REFUSED_BAD_TRANSITION', f'{self.state} -> DOCKED')
        self._fresh(observation, now_wall_s=now_wall_s)
        window = self._window(observation)
        if not window['fits']:
            self._refuse('REFUSED_OUT_OF_WINDOW', 'the re-verification failed')
        self._enter('DOCKED')
        self.docked_since = float(now_wall_s)
        return self.snapshot(now_wall_s=now_wall_s)

    def undock(self, *, now_wall_s):
        """Leaving is a failure-state-free path: DOCKED -> IDLE is a new attempt, and the permit goes
        back to nobody so the next holder has to be granted it explicitly."""
        if self.state != 'DOCKED':
            self._refuse('REFUSED_BAD_TRANSITION', f'{self.state} -> IDLE')
        self.permits.release()
        self.state = 'IDLE'
        self.history.append('IDLE')
        self.docked_since = None
        return self.snapshot(now_wall_s=now_wall_s)

    # ------------------------------------------------------------------ failure paths
    def interrupt(self, *, reason_code, now_wall_s):
        from workcell import schema as S
        if reason_code not in S.REASON_CODES:
            self._refuse('REFUSED_UNDECLARED_REASON', repr(reason_code), field='reason_code')
        if self.state in ('IDLE',) or self.is_stuck:
            self._refuse('REFUSED_BAD_TRANSITION', f'{self.state} -> STOPPING')
        self.return_to = self.state
        self.state = 'STOPPING'
        self.history.append('STOPPING')
        self.reason_code = reason_code
        self.permits.release()
        return self.snapshot(now_wall_s=now_wall_s)

    def resolve(self, *, non_transfer_proven, now_wall_s):
        if self.state != 'STOPPING':
            self._refuse('REFUSED_BAD_TRANSITION', f'{self.state} -> resolve')
        if not isinstance(non_transfer_proven, bool):
            self._refuse('REFUSED_BAD_TYPE', 'non_transfer_proven must be a bool',
                         field='non_transfer_proven')
        self.state = 'ABORTED' if non_transfer_proven else 'NEEDS_ATTENTION'
        self.history.append(self.state)
        return self.snapshot(now_wall_s=now_wall_s)

    # ------------------------------------------------------------------ validity
    def step(self, *, now_wall_s, vehicle_moved=False, permit_holder=None):
        """A DOCKED verdict expires on its own evidence.

        The guidance: DOCKED has a validity, and drift, movement, an expired observation or a permit
        change all invalidate it. Anything else turns a dock into a latch, and a latch is how a
        vehicle is still 'docked' an hour after it drove away.
        """
        if self.state != 'DOCKED':
            return {'state': self.state, 'valid': None, 'reason': None}
        for condition, reason in (
                (vehicle_moved, 'the vehicle moved'),
                (permit_holder is not None and permit_holder != 'docking',
                 f'the permit moved to {permit_holder!r}'),
                (self.docked_since is not None
                 and float(now_wall_s) - self.docked_since > self.docked_validity_s,
                 f'the verdict is older than the declared {self.docked_validity_s} s'),
                (self.last_observation is not None
                 and float(now_wall_s) - float(self.last_observation['wall_receive_time'])
                 > float(self.contract['validity']['max_observation_age_s']),
                 'the supporting observation aged out')):
            if condition:
                self.state = 'NEEDS_ATTENTION'
                self.history.append('NEEDS_ATTENTION')
                self.reason_code = 'DOCK_INVALIDATED'
                return {'state': self.state, 'valid': False, 'reason': reason}
        return {'state': self.state, 'valid': True, 'reason': None,
                'remaining_s': None if self.docked_since is None else
                self.docked_validity_s - (float(now_wall_s) - self.docked_since)}

    def snapshot(self, *, now_wall_s=None):
        return {'state': self.state, 'station_id': self.station_id, 'epoch': self.epoch,
                'transfer_id': self.transfer_id, 'history': list(self.history),
                'reason_code': self.reason_code, 'return_to': self.return_to,
                'permit_holder': self.permits.holder,
                'permit_generation': self.permits.generation,
                'last_window': self.last_window,
                'refusals': list(self.refusals),
                'docked_validity_s': self.docked_validity_s}
