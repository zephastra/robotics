"""A scriptable plant for the fault matrix. It answers; it does not simulate.

WHY IT IS ALLOWED TO EXIST
--------------------------
The guidance's W4 says the protocol faults are to be covered with SHORT TESTS FIRST and the ones
that need physics afterwards, and it says out loud what a fake cannot prove: "fake 不能证明停止距
离、支撑连续性". So this plant is used for the protocol rows and its results are reported as
protocol rows -- never as a physical claim. The physical rows come from `w4_plant.py`, which owns
real bodies.
"""
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src'))


class FakePlant:
    """A plant whose every answer can be dictated by the test."""

    def __init__(self, *, stations, tray_zone='source_band', drive_error=0.0,
                 stopped=True, dock=(0.0, 0.0, 0.0), transfer_state='RECEIVED'):
        self.stations = stations
        self.tray_zone = tray_zone
        self.drive_error = drive_error
        self.stopped = stopped
        self.dock = dock
        self.transfer_state = transfer_state
        self.calls = []
        self.time = 0.0

    def clock(self):
        return self.time

    def _tick(self, seconds=1.0):
        self.time += seconds
        return self.time

    def drive_to(self, target_x, *, speed, timeout_s, reverse=False):
        self.calls.append(('drive_to', target_x, reverse))
        return {'final_x_m': target_x + self.drive_error, 'error_x_m': self.drive_error,
                'travelled_m': abs(self.drive_error), 'peak_speed_mps': speed,
                'stopped_confirmed': self.stopped, 'held_speed_mps': 0.0,
                'end_s': self._tick()}

    def stop(self):
        self.calls.append(('stop',))
        return {'stopped_confirmed': self.stopped, 'held_speed_mps': 0.0, 'end_s': self._tick()}

    def dock_residuals(self, station_id):
        self.calls.append(('dock_residuals', station_id))
        lateral, longitudinal, yaw = self.dock
        return {'lateral_m': lateral, 'longitudinal_m': longitudinal, 'yaw_rad': yaw}

    def tray_state(self):
        self.calls.append(('tray_state',))
        return {'zone': self.tray_zone, 'supported_by': self.tray_support(), 'x_m': 0.0,
                'z_bottom_m': 0.485, 'on_crowns': True, 'end_s': self._tick()}

    def tray_support(self):
        """Which rows the tray stands on. The real plant reads this from the solver's contacts;
        the fake says what the test told it to say, which is the whole point of a fake."""
        self.calls.append(('tray_support',))
        return [] if self.tray_zone == 'unknown' else [self.tray_zone]

    def transfer(self, *, direction, timeout_s):
        self.calls.append(('transfer', direction))
        return {'state': self.transfer_state, 'direction': direction,
                'expected_zone': 'deck' if direction == 'onto_deck' else 'receiver_band',
                'tray': self.tray_state(), 'start_s': 0.0, 'end_s': self._tick()}

    def observe(self):
        self.calls.append(('observe',))
        return {'chassis_x_m': 0.0, 'chassis_speed_mps': 0.0, 'tray': self.tray_state(),
                'end_s': self._tick()}


FAKE_STATIONS = {
    'station_c': {'id': 'station_c', 'approach_x_m': 4.0, 'dock_x_m': 4.6, 'approach_leg_m': 0.6,
                  'approach_speed_mps': 0.3, 'approach_timeout_s': 5.0, 'dock_timeout_s': 5.0},
    'station_b': {'id': 'station_b', 'approach_x_m': 9.0, 'dock_x_m': 9.6, 'approach_leg_m': 0.6,
                  'approach_speed_mps': 0.3, 'approach_timeout_s': 5.0, 'dock_timeout_s': 5.0},
}

#: The legs the tests are allowed to run, declared rather than inferred. The adapter checks the
#: declaration against the tray's contacts and refuses when the tray is not on the declared
#: source, so every transfer in these tests has to name one.
FAKE_TRANSFERS = {
    't1': {'direction': 'onto_deck', 'source': 'dock_a', 'destination': 'deck_b',
           'source_zone': 'source_band', 'destination_zone': 'deck'},
    't2': {'direction': 'onto_receiver', 'source': 'deck_b', 'destination': 'recv_a',
           'source_zone': 'deck', 'destination_zone': 'receiver_band'},
}

FAKE_ENVELOPE = {'approach_m': 0.05, 'dock_speed_mps': 0.06, 'dock_lateral_m': 0.002,
                 'dock_longitudinal_m': 0.02, 'dock_yaw_rad': 0.0035, 'transfer_timeout_s': 20.0}

#: The legs this scenario DECLARES, named with the same vocabulary the W5 scenario uses so a reader
#: sees one set of names. The adapter does not infer a direction from the tray's position: a leg is
#: a property of the transaction, and an undeclared id is refused (`REFUSED_ENTITY_UNKNOWN`). The
#: three protocol tests below were written against the earlier inference and passed `'t1'`; the
#: refusal is the contract working, so the tests were updated to declare the leg they exercise.
FAKE_TRANSFERS = {
    'xfer_source_to_deck': {'direction': 'onto_deck', 'source': 'c_source_band',
                            'destination': 'c_deck', 'source_zone': 'source_band',
                            'destination_zone': 'deck'},
    'xfer_deck_to_receiver': {'direction': 'onto_receiver', 'source': 'c_deck',
                              'destination': 'c_receiver_band', 'source_zone': 'deck',
                              'destination_zone': 'receiver_band'},
}
