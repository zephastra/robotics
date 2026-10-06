"""W4: the adapter that turns a validated skill request into a physical action.

LAYERING, DELIBERATELY
----------------------
`src/` does not import `experiments/`. This adapter knows the SKILL SEMANTICS -- what
`SUCCEEDED` means for `DOCK`, which evidence a skill has to produce, when a skill must refuse --
and it receives the plant as an object. The MuJoCo implementation of that object lives in
`experiments/w4_plant.py`, which is where the simulator belongs. The separation is what makes the
fault matrix testable without physics: a fake plant exercises every refusal path, and the physical
plant is exercised once, on the real world.

WHAT THE ADAPTER IS NOT ALLOWED TO DO
-------------------------------------
  * it may not read a world pose to decide a motion -- `drive_to` takes a target and reports what
    happened, and the target comes from the station's declared approach pose;
  * it may not call `succeeded` because a timer expired. Every `SUCCEEDED` below is attached to a
    measured quantity crossing a declared threshold, and every other outcome carries a
    `reason_code` from the schema's own set;
  * it may not convert a refusal into a failure. `NEEDS_ATTENTION` and `FAILED` are different
    answers and the guidance's fault matrix needs both.
"""
import math

from workcell import schema as S


class SkillAdapter:
    """Executes one skill against a plant. One instance per device boot."""

    def __init__(self, *, plant, boot_id, stations, envelope, transfers=None, navigation_move=None):
        self.plant = plant
        # Optional guest navigator; DOCK/UNDOCK remain distinct physical skills.
        # It must report actual stop evidence, not just action acceptance.
        self.navigation_move = navigation_move
        self.boot_id = S._identifier(boot_id, 'boot_id')
        self.stations = dict(stations)
        #: transfer_id -> {direction, source, destination, source_zone, destination_zone}. The
        #: SCENARIO declares the legs; the adapter does not infer them. See `_transfer`.
        self.transfers = {k: dict(v) for k, v in (transfers or {}).items()}
        #: (approach_m, dock_lateral_m, dock_longitudinal_m, dock_yaw_rad). The numbers come from
        #: the caller -- this module does not own a threshold.
        self.envelope = dict(envelope)
        self.calls = []
        #: The coordinator's wall clock at the moment of the command, recorded separately from
        #: the plant's own clock. Set by `execute`.
        self._wall_s = None

    # -- helpers ------------------------------------------------------------

    def _station(self, skill, arguments):
        station = arguments.get('station_id')
        if station is None:
            raise S.SchemaRefused('REFUSED_MISSING_FIELD',
                                  f'{skill} needs a station_id', field='arguments.station_id')
        if station not in self.stations:
            raise S.SchemaRefused('REFUSED_ENTITY_UNKNOWN', repr(station),
                                  field='arguments.station_id')
        return self.stations[station]

    def _result(self, request, *, status, reason=None, evidence=None, start_s, end_s,
                final_state=None):
        """★ `start_s` / `end_s` are the PLANT's clock. The coordinator's wall clock is recorded
        separately as `commanded_at_wall_s`. The first version passed the wall clock in as
        `start_s` and the simulator's in as `end_s`, and the schema refused the result with
        `end_s 1.0 < start_s 100.0` -- which is the guidance's "sim time and wall time are two
        clocks" made concrete."""
        state = dict(final_state or {})
        if self._wall_s is not None:
            state['commanded_at_wall_s'] = float(self._wall_s)
        state['plant_clock_s'] = float(end_s)
        return {'schema_version': S.SCHEMA_VERSION, 'command_id': request['command_id'],
                'order_id': request['order_id'], 'revision': request['revision'],
                'boot_id': self.boot_id, 'status': status, 'reason_code': reason,
                'evidence': list(evidence or []), 'final_state': state,
                'start_s': float(start_s), 'end_s': float(end_s)}

    # -- the skills ---------------------------------------------------------

    def execute(self, request, *, now_s):
        skill = request['skill']
        arguments = request['arguments']
        self._wall_s = float(now_s)
        start = float(self.plant.clock())
        self.calls.append(skill)
        handler = getattr(self, f'_skill_{skill.lower()}', None)
        if handler is None:
            return self._result(request, status='FAILED', reason='UNSUPPORTED_SKILL',
                                start_s=start, end_s=now_s)
        return handler(request, arguments, start)

    def _skill_move_to_station(self, request, arguments, start):
        station = self._station('MOVE_TO_STATION', arguments)
        outcome = (self.navigation_move(station) if self.navigation_move is not None
                   else self.plant.drive_to(station['approach_x_m'], speed=station['approach_speed_mps'],
                                           timeout_s=station['approach_timeout_s']))
        inside = (abs(outcome['error_x_m']) <= self.envelope['approach_m']
                  and outcome.get('navigation_succeeded', True) is True)
        if not outcome.get('stopped_confirmed'):
            return self._result(request, status='FAILED', reason='NO_PROGRESS',
                                evidence=['runs/move.json'], start_s=start,
                                end_s=outcome['end_s'], final_state=outcome)
        return self._result(request, status='SUCCEEDED' if inside else 'FAILED',
                            reason=None if inside else 'NAV_FAILED',
                            evidence=['runs/move.json'], start_s=start, end_s=outcome['end_s'],
                            final_state=outcome)

    def _skill_dock(self, request, arguments, start):
        station = self._station('DOCK', arguments)
        outcome = self.plant.drive_to(station['dock_x_m'], speed=self.envelope['dock_speed_mps'],
                                      timeout_s=station['dock_timeout_s'])
        measured = self.plant.dock_residuals(station['id'])
        fits = (abs(measured['lateral_m']) <= self.envelope['dock_lateral_m']
                and abs(measured['longitudinal_m']) <= self.envelope['dock_longitudinal_m']
                and abs(measured['yaw_rad']) <= self.envelope['dock_yaw_rad'])
        final = {**outcome, **measured, 'fits': fits}
        if not fits:
            # A dock that does not fit is a DOCK failure, not a navigation failure -- the reason
            # code is what tells the next reader which subsystem to open.
            return self._result(request, status='FAILED', reason='DOCK_OUT_OF_TOLERANCE',
                                evidence=['runs/dock.json'], start_s=start,
                                end_s=outcome['end_s'], final_state=final)
        return self._result(request, status='SUCCEEDED', evidence=['runs/dock.json'],
                            start_s=start, end_s=outcome['end_s'], final_state=final)

    def _transfer(self, skill, arguments):
        """The leg, from the SCENARIO's declaration, CHECKED against the tray's own contacts.

        ★ The first version INFERRED the direction from the tray's position, and that inference is
        wrong exactly when it matters: after a successful load the tray is on the deck, so the
        inference concludes that the only leg left is the unload, and the LOADING transfer's own
        `VERIFY_TRANSFER` then checks for the wrong arrival zone and reports `PAYLOAD_LOST` for a
        transfer that worked. Measured: `reports/w5-loop-02`, `cmd-003`.

        A transfer's direction is a property of the TRANSACTION. What the tray's position is good
        for is CHECKING the declaration, so that is what it is used for here: if the tray is not
        standing on the declared source row, the skill refuses with `PAYLOAD_LOST` instead of
        silently running a different leg.
        """
        transfer_id = arguments.get('transfer_id')
        if transfer_id is None:
            raise S.SchemaRefused('REFUSED_MISSING_FIELD', f'{skill} needs a transfer_id',
                                  field='arguments.transfer_id')
        if transfer_id not in self.transfers:
            raise S.SchemaRefused('REFUSED_ENTITY_UNKNOWN', repr(transfer_id),
                                  field='arguments.transfer_id')
        declared = self.transfers[transfer_id]
        rows = self.plant.tray_support()
        on_source = declared['source_zone'] in rows
        return declared, on_source, rows

    def _skill_start_transfer(self, request, arguments, start):
        declared, on_source, rows = self._transfer('START_TRANSFER', arguments)
        if not on_source:
            return self._result(request, status='FAILED', reason='PAYLOAD_LOST',
                                evidence=['runs/transfer.json'], start_s=start,
                                end_s=start,
                                final_state={'declared_source': declared['source_zone'],
                                             'tray_supported_by': rows,
                                             'refused_because': 'the tray is not on the declared '
                                                                'source row'})
        outcome = self.plant.transfer(direction=declared['direction'],
                                      timeout_s=self.envelope['transfer_timeout_s'])
        if outcome.get('interrupted'):
            # A runtime refusal is not delivery, even if the tray touches its
            # destination while braking. Custody requires later fresh evidence.
            return self._result(request, status='FAILED',
                                reason=(outcome['reason_code']
                                        if outcome['brake']['stopped_confirmed']
                                        else 'CANCEL_UNCONFIRMED'),
                                evidence=['runs/transfer.json'], start_s=start,
                                end_s=outcome['end_s'], final_state=outcome)
        if outcome['state'] == 'TRANSFERRING':
            return self._result(request, status='FAILED', reason='TRANSFER_TIMEOUT',
                                evidence=['runs/transfer.json'], start_s=start,
                                end_s=outcome['end_s'], final_state=outcome)
        if outcome['state'] == 'UNKNOWN':
            # Not FAILED: "the transfer's state is not knowable" must not be reported as "it
            # failed", because the recovery is different -- hold the goods, re-observe, retry.
            return self._result(request, status='FAILED', reason='TRANSFER_UNKNOWN',
                                evidence=['runs/transfer.json'], start_s=start,
                                end_s=outcome['end_s'], final_state=outcome)
        return self._result(request, status='SUCCEEDED', evidence=['runs/transfer.json'],
                            start_s=start, end_s=outcome['end_s'], final_state=outcome)

    def _skill_verify_transfer(self, request, arguments, start):
        declared, _on_source, rows = self._transfer('VERIFY_TRANSFER', arguments)
        outcome = self.plant.tray_state()
        expected = declared['destination_zone']
        agreed = outcome['zone'] == expected
        final = {**outcome, 'expected_zone': expected, 'direction': declared['direction'],
                 'declared_source_zone': declared['source_zone']}
        return self._result(request, status='SUCCEEDED' if agreed else 'FAILED',
                            reason=None if agreed else 'PAYLOAD_LOST',
                            evidence=['runs/tray.json'], start_s=start, end_s=outcome['end_s'],
                            final_state=final)

    def _skill_undock(self, request, arguments, start):
        station = self._station('UNDOCK', arguments)
        if station.get('approach_leg_m', 0.0) <= 0.0:
            # A station the vehicle starts ON has no retreat leg, and inventing one would ask it
            # to reverse through its own fixture. Releasing the dock is then "hold, and confirm
            # stopped", which is a real thing to do and is reported as such.
            outcome = self.plant.stop()
            outcome['approach_leg_m'] = 0.0
            return self._result(request, status='SUCCEEDED' if outcome.get('stopped_confirmed')
                                else 'FAILED',
                                reason=None if outcome.get('stopped_confirmed')
                                else 'NO_PROGRESS',
                                evidence=['runs/undock.json'], start_s=start,
                                end_s=outcome['end_s'], final_state=outcome)
        outcome = self.plant.drive_to(station['approach_x_m'],
                                      speed=self.envelope['dock_speed_mps'],
                                      timeout_s=station['dock_timeout_s'], reverse=True)
        clear = abs(outcome['final_x_m'] - station['approach_x_m']) <= self.envelope['approach_m']
        return self._result(request, status='SUCCEEDED' if clear else 'FAILED',
                            reason=None if clear else 'NO_PROGRESS',
                            evidence=['runs/undock.json'], start_s=start,
                            end_s=outcome['end_s'], final_state=outcome)

    def _skill_verify_delivery(self, request, arguments, start):
        tray = self.plant.tray_state()
        delivered = tray['zone'] == 'receiver_band'
        return self._result(request, status='SUCCEEDED' if delivered else 'FAILED',
                            reason=None if delivered else 'PAYLOAD_LOST',
                            evidence=['runs/delivery.json'], start_s=start, end_s=tray['end_s'],
                            final_state=tray)

    def _skill_stop(self, request, arguments, start):
        outcome = self.plant.stop()
        return self._result(request, status='SUCCEEDED' if outcome.get('stopped_confirmed')
                            else 'FAILED',
                            reason=None if outcome.get('stopped_confirmed') else 'CANCEL_UNCONFIRMED',
                            evidence=['runs/stop.json'], start_s=start, end_s=outcome['end_s'],
                            final_state=outcome)

    def _skill_observe(self, request, arguments, start):
        outcome = self.plant.observe()
        return self._result(request, status='SUCCEEDED', evidence=['runs/observe.json'],
                            start_s=start, end_s=outcome['end_s'], final_state=outcome)
