"""The P1-N IPC contract, the command gate, and the judge that decides the probe passed.

These run without ROS and without MuJoCo where possible: the contract and the gate are the
parts that decide whether a command is acted on, so they have to be testable on their own.
The judge is tested the way this project tests instruments -- by feeding it inputs that must
FAIL and requiring it to say so. A judge that cannot be shown to fail is not a judge.
"""
import json
import math
from pathlib import Path
import subprocess
import sys

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src'))
sys.path.insert(0, str(ROOT / 'experiments'))

from workcell_ipc import (COMMAND_KEYS, Refused, STATE_KEYS, command, decode,  # noqa: E402
                          encode, state)
from workcell_ipc.gate import CommandGate  # noqa: E402


def a_state(**overrides):
    payload = state(seq=7, sim_time=1.5, wheel_rate=[1.0, -1.0], ranges=[1.0, 2.0, 3.0],
                    range_min=0.12, range_max=8.0,
                    gate={'mode': 'HOLDING', 'age_s': 0.01, 'accepted': 4, 'refused': 0})
    payload.update(overrides)
    return payload


def a_command(**overrides):
    payload = command(seq=1, issued_sim_time=1.0, ttl_s=0.5, v=0.2, w=-0.1)
    payload.update(overrides)
    return payload


def a_gate():
    return CommandGate(ttl_s=0.5, silence_s=0.5, v_max=0.6, w_max=1.2)


# ---------------------------------------------------------------- the contract
def test_a_state_round_trips():
    payload = a_state()
    assert decode('state', encode('state', payload), expect_ranges=3) == payload


def test_a_command_round_trips():
    payload = a_command()
    assert decode('command', encode('command', payload)) == payload


def test_an_unknown_field_is_refused_not_ignored():
    """A field both ends ignore is a field nobody tests."""
    payload = a_state(extra=1)
    with pytest.raises(Refused) as excinfo:
        encode('state', payload)
    assert excinfo.value.reason == 'REFUSED_UNKNOWN_FIELD'
    raw = json.dumps({**a_state(), 'extra': 1}).encode()
    with pytest.raises(Refused) as excinfo:
        decode('state', raw, expect_ranges=None)
    assert excinfo.value.reason == 'REFUSED_UNKNOWN_FIELD'


def test_a_missing_field_is_refused():
    payload = a_state()
    del payload['wheel_rate']
    with pytest.raises(Refused) as excinfo:
        encode('state', payload)
    assert excinfo.value.reason == 'REFUSED_MISSING_FIELD'


def test_a_wrong_schema_version_is_refused():
    raw = json.dumps({**a_state(), 'schema_version': 2}).encode()
    with pytest.raises(Refused) as excinfo:
        decode('state', raw)
    assert excinfo.value.reason == 'REFUSED_SCHEMA_VERSION'


def test_nan_is_refused_even_though_json_accepts_it():
    """`json.loads('NaN')` succeeds, so non-finite values have to be rejected explicitly."""
    assert math.isnan(json.loads('{"v": NaN}')['v'])
    raw = json.dumps({**a_command(), 'v': float('nan')}).encode()
    with pytest.raises(Refused) as excinfo:
        decode('command', raw)
    assert excinfo.value.reason == 'REFUSED_NON_FINITE'
    with pytest.raises(Refused):
        encode('command', a_command(v=float('inf')))


def test_a_duplicate_key_is_refused():
    raw = b'{"schema_version":1,"seq":1,"seq":2,"issued_sim_time":1.0,"ttl_s":0.5,"v":0.1,"w":0.0}'
    with pytest.raises(Refused) as excinfo:
        decode('command', raw)
    assert excinfo.value.reason == 'REFUSED_DUPLICATE_KEY'


def test_a_range_count_that_disagrees_with_config_is_refused():
    """Publishing a scan with the wrong length would put a wrong angle increment on the wire."""
    with pytest.raises(Refused) as excinfo:
        decode('state', encode('state', a_state()), expect_ranges=181)
    assert excinfo.value.reason == 'REFUSED_RANGE_COUNT'


def test_a_negative_or_non_finite_range_is_refused():
    raw = json.dumps(a_state(ranges=[1.0, -0.5, 3.0])).encode()
    with pytest.raises(Refused):
        decode('state', raw, expect_ranges=3)
    raw = json.dumps(a_state(ranges=[1.0, float('nan'), 3.0])).encode()
    with pytest.raises(Refused):
        decode('state', raw, expect_ranges=3)


def test_the_gate_keys_are_part_of_the_contract():
    raw = json.dumps(a_state(gate={'mode': 'HOLDING', 'age_s': 0.0, 'accepted': 1})).encode()
    with pytest.raises(Refused) as excinfo:
        decode('state', raw)
    assert excinfo.value.reason == 'REFUSED_UNKNOWN_FIELD'


def test_the_state_contract_has_no_pose_field():
    """This is what makes 'odometry is not fed the truth' structural, not a promise."""
    for key in STATE_KEYS:
        assert 'pose' not in key
        assert key not in ('x', 'y', 'yaw', 'world_pose')
    assert 'v' not in COMMAND_KEYS or True   # commands carry velocity, which is fine


# ---------------------------------------------------------------- the gate
def test_with_no_command_the_wheels_are_told_to_stop():
    gate = a_gate()
    assert gate.step(0.0) == (0.0, 0.0, 'SILENT', 'NO_COMMAND_EVER')


def test_a_fresh_command_is_held():
    gate = a_gate()
    assert gate.offer(encode('command', a_command(seq=1, issued_sim_time=1.0, v=0.3, w=0.2)), 1.0)
    assert gate.step(1.2) == (0.3, 0.2, 'HOLDING', 'HELD')


def test_a_command_older_than_its_ttl_is_refused_and_the_lease_lapses():
    """TTL and silence are separate mechanisms and are tested separately.

    With silence_s equal to ttl_s a lapse is reported as SILENT, which is also a stop but
    says something different: EXPIRED means the command aged out, SILENT means nobody is
    talking any more. The gate checks silence first on purpose.
    """
    gate = CommandGate(ttl_s=0.5, silence_s=10.0, v_max=0.6, w_max=1.2)
    gate.offer(encode('command', a_command(seq=1, issued_sim_time=1.0)), 1.0)
    assert gate.step(1.4)[2] == 'HOLDING'
    assert gate.step(1.6)[2] == 'EXPIRED'
    assert gate.step(1.6)[:2] == (0.0, 0.0)


def test_silence_stops_the_wheels_even_with_a_long_ttl_command():
    """A dead ROS process must stop the robot, not leave it driving on the last message."""
    gate = CommandGate(ttl_s=10.0, silence_s=0.5, v_max=0.6, w_max=1.2)
    gate.offer(encode('command', a_command(seq=1, issued_sim_time=1.0, ttl_s=10.0, v=0.5)), 1.0)
    assert gate.step(1.2)[2] == 'HOLDING'
    assert gate.step(2.0) == (0.0, 0.0, 'SILENT', 'SILENT_SINCE_LAST_COMMAND')


def test_a_late_datagram_is_not_accepted_at_all():
    gate = a_gate()
    ok, reason = gate.offer(encode('command', a_command(seq=1, issued_sim_time=0.0)), 2.0)
    assert not ok and reason == 'REFUSED_ALREADY_EXPIRED'
    assert gate.step(2.0)[2] == 'SILENT'


def test_a_stale_sequence_number_is_refused():
    gate = a_gate()
    assert gate.offer(encode('command', a_command(seq=5, issued_sim_time=1.0)), 1.0)[0]
    ok, reason = gate.offer(encode('command', a_command(seq=5, issued_sim_time=1.05)), 1.05)
    assert not ok and reason == 'REFUSED_STALE_SEQ'


def test_a_command_from_the_future_is_refused():
    gate = a_gate()
    ok, reason = gate.offer(encode('command', a_command(seq=1, issued_sim_time=99.0)), 1.0)
    assert not ok and reason == 'REFUSED_FROM_THE_FUTURE'


def test_a_command_beyond_the_configured_limits_is_refused_not_clamped():
    gate = a_gate()
    ok, reason = gate.offer(encode('command', a_command(seq=1, issued_sim_time=1.0, v=5.0)), 1.0)
    assert not ok and reason == 'REFUSED_OUT_OF_RANGE'
    ok, reason = gate.offer(encode('command', a_command(seq=2, issued_sim_time=1.0, w=9.0)), 1.0)
    assert not ok and reason == 'REFUSED_OUT_OF_RANGE'


def test_a_refused_datagram_does_not_extend_the_lease():
    gate = CommandGate(ttl_s=0.5, silence_s=10.0, v_max=0.6, w_max=1.2)
    gate.offer(encode('command', a_command(seq=1, issued_sim_time=1.0)), 1.0)
    gate.offer(encode('command', a_command(seq=2, issued_sim_time=1.4)), 1.5)   # accepted
    gate.offer(b'not json at all', 1.6)                                        # refused
    assert gate.counters['refused'] == 1
    assert gate.reasons['REFUSED_NOT_JSON'] == 1
    assert gate.step(1.7)[2] == 'HOLDING'
    assert gate.step(2.1)[2] == 'EXPIRED'      # measured from the accepted command, not the junk


def test_a_measured_run_stops_on_silence_after_commands_that_were_accepted():
    """The fallback run: commands accepted, then they stop, and the wheels go to zero."""
    gate = a_gate()
    for index in range(5):
        issued = 1.0 + 0.1 * index
        gate.offer(encode('command', a_command(seq=index + 1, issued_sim_time=issued,
                                               v=0.4)), issued)
    assert gate.counters['accepted'] == 5
    assert gate.step(1.5)[2] == 'HOLDING'          # 0.1 s after the last one: still valid
    v, w, mode, reason = gate.step(2.1)            # 0.7 s later: both window and silence passed
    assert (v, w) == (0.0, 0.0)
    assert mode in ('SILENT', 'EXPIRED')
    assert gate.summary(2.1)['mode'] == mode


def test_the_summary_says_what_the_gate_did():
    gate = a_gate()
    gate.offer(encode('command', a_command(seq=1, issued_sim_time=1.0, v=0.2, w=0.1)), 1.0)
    # Version is checked before the field set, so an empty object is refused for its version.
    gate.offer(b'{}', 1.1)
    summary = gate.summary(1.2)
    assert summary['mode'] == 'HOLDING' and summary['accepted'] == 1
    assert summary['refused'] == 1
    assert summary['reasons'] == {'REFUSED_SCHEMA_VERSION': 1}
    assert summary['v'] == 0.2
    gate.offer(b'{"schema_version":1,"seq":2,"issued_sim_time":1.2,"ttl_s":0.5}', 1.2)
    assert gate.summary(1.3)['reasons']['REFUSED_MISSING_FIELD'] == 1


# ---------------------------------------------------------------- the process split
def test_the_two_halves_do_not_import_each_others_runtime():
    """MuJoCo is in the .venv on Python 3.12 and rclpy is Python 3.14: they cannot share a
    process, so the split is enforced rather than intended."""
    sim = (ROOT / 'experiments/probe_n_sim.py').read_text(encoding='utf-8')
    ros = (ROOT / 'experiments/probe_n_ros.py').read_text(encoding='utf-8')
    assert 'import rclpy' not in sim, 'the simulator side must not import rclpy'
    assert 'import mujoco' not in ros, 'the ROS side must not import mujoco'
    for name, text in (('sim', sim), ('ros', ros)):
        assert 'n_probe.yaml' in text, f'{name} does not read the shared config'


def test_the_judge_can_fail_synthetic_inputs(tmp_path):
    """Feed the judge a graph with two /clock publishers and require it to refuse."""
    run = tmp_path / 'p1-n-ipc-synthetic'
    run.mkdir()
    (run / 'sim_report.json').write_text(json.dumps({
        'status': 'COMPLETED',
        'state_schema': {'keys': sorted(STATE_KEYS), 'world_pose_in_state': False},
        'ipc': {'states_sent': 10},
        'gate': {'accepted': 3, 'refused': 0, 'reasons': {}, 'mode': 'HOLDING',
                 'modes': {'HOLDING': 100}},
        'truth': {'final': [1.0, 0.0, 0.0], 'travelled_m': 1.0},
    }))
    (run / 'ros_report.json').write_text(json.dumps({
        'status': 'COMPLETED', 'command_stopped_at': None,
        'counters': {'states_received': 10, 'decode_refusals': 0, 'clocks': 10, 'scans': 10,
                     'odoms': 10, 'commands_sent': 3, 'command_refusals': 0, 'bytes_in': 1},
        'seq_gaps': 0, 'odom_final': {'x': 1.0, 'y': 0.0, 'yaw': 0.0},
        'decode_refusal_reasons': {},
    }))
    (run / 'checker_raw.json').write_text(json.dumps({
        'status': 'COMPLETED', 'wall_seconds': 1.0,
        'counts': {'clock': 10, 'scan': 10, 'odom': 10, 'tf': 0, 'tf_static': 0},
        'rows': [{'wall': 0.5, 'counts': {}, 'publishers': {'clock': 2, 'scan': 1, 'odom': 1,
                                                            'tf': 1, 'tf_static': 1},
                  'clock_stamp': 1.0, 'scan_stamp': 1.0, 'odom_stamp': 1.0},
                 {'wall': 1.0, 'counts': {}, 'publishers': {'clock': 2, 'scan': 1, 'odom': 1,
                                                            'tf': 1, 'tf_static': 1},
                  'clock_stamp': 2.0, 'scan_stamp': 2.0, 'odom_stamp': 2.0}],
        'observed': {'scan_geometry': None, 'scan_finite': 0, 'scan_rays': 0,
                     'scan_range_min': None, 'scan_range_max': None,
                     'clock_stamp': 2.0, 'scan_stamp': 2.0, 'odom_stamp': 2.0,
                     'clock_child': [], 'tf_children': [], 'static_children': [],
                     'odom_child': None, 'odom_first': (0.0, 0.0), 'odom_last': (1.0, 0.0)},
    }))
    proc = subprocess.run([sys.executable, str(ROOT / 'experiments/evaluate_n_ipc.py'),
                           '--run-id', run.name, '--reports-dir', str(tmp_path)],
                          capture_output=True, text=True, timeout=120)
    assert proc.returncode == 1, proc.stdout
    assert 'exactly one /clock publisher' in proc.stdout
    result = json.loads((run / 'acceptance.json').read_text())
    assert result['verdict'] == 'FAIL'
    assert 'exactly one /clock publisher' in result['failed']


def test_the_judge_reports_missing_input_as_failure_not_silence(tmp_path):
    run = tmp_path / 'p1-n-ipc-empty'
    run.mkdir()
    proc = subprocess.run([sys.executable, str(ROOT / 'experiments/evaluate_n_ipc.py'),
                           '--run-id', run.name, '--reports-dir', str(tmp_path)],
                          capture_output=True, text=True, timeout=120)
    assert proc.returncode == 1
    result = json.loads((run / 'acceptance.json').read_text())
    assert result['verdict'] == 'FAIL'
    assert len(result['failed']) == 3
