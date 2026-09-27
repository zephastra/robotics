"""What may cross the sim/ROS process boundary in the P1-N probe, and what is refused.

010's two halves cannot share a process: MuJoCo lives in the project .venv (Python 3.12) and
rclpy is the system interpreter (Python 3.14). So the world state and the velocity commands
cross a loopback UDP socket, and this module is the contract for what may cross it.

It is stdlib-only on purpose: both interpreters must be able to import it.

The rules come from docs/CONTRACTS.md section 1 (reject unknown fields, duplicate keys, and
non-finite numbers) and from the P1-N scope ("length-bounded local IPC: sequence number, sim
timestamp, command validity, and a zero/stop fallback; no arbitrary network commands").
Refusing beats coercing throughout: a coerced command looks exactly like a working link, and
a NaN that gets silently clamped reads as a healthy run.

Two properties make the states N cares about structural rather than promised:

  * There is no world-pose field in the state datagram. The ROS side cannot feed the truth
    into odometry because it is never sent it.
  * Decoding is done against an exact key set, so a field added at one end is refused at the
    other instead of being ignored -- a field both ends ignore is a field nobody tests.
"""
import json

SCHEMA_VERSION = 1

#: Exact key sets. Not "required subset": an extra key is refused.
STATE_KEYS = frozenset({
    'schema_version', 'seq', 'sim_time', 'wheel_rate', 'ranges',
    'range_min', 'range_max', 'gate',
})
GATE_KEYS = frozenset({'mode', 'age_s', 'accepted', 'refused'})
COMMAND_KEYS = frozenset({
    'schema_version', 'seq', 'issued_sim_time', 'ttl_s', 'v', 'w',
})

KINDS = {
    'state': STATE_KEYS,
    'command': COMMAND_KEYS,
}

STATE_LIMITS = {
    'sim_time': (0.0, 1.0e6),
    'range_min': (0.0, 1.0e3),
    'range_max': (0.0, 1.0e3),
}


class Refused(ValueError):
    """A datagram that must not be acted on, carrying a machine-readable reason."""

    def __init__(self, reason, detail=''):
        super().__init__(f'{reason}: {detail}' if detail else reason)
        self.reason = reason
        self.detail = detail


def _pairs_hook(pairs):
    """Reject duplicate keys instead of silently keeping the last one."""
    seen = {}
    for key, value in pairs:
        if key in seen:
            raise Refused('REFUSED_DUPLICATE_KEY', key)
        seen[key] = value
    return seen


def _finite(value, name):
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise Refused('REFUSED_BAD_TYPE', f'{name} is {type(value).__name__}')
    if value != value or value in (float('inf'), float('-inf')):
        raise Refused('REFUSED_NON_FINITE', name)
    return float(value)


def encode(kind, payload):
    """Serialize a state or command datagram. NaN cannot leave this function."""
    if kind not in KINDS:
        raise Refused('REFUSED_UNKNOWN_KIND', kind)
    expected = KINDS[kind]
    missing = expected - set(payload)
    extra = set(payload) - expected
    if missing:
        raise Refused('REFUSED_MISSING_FIELD', ','.join(sorted(missing)))
    if extra:
        raise Refused('REFUSED_UNKNOWN_FIELD', ','.join(sorted(extra)))
    try:
        text = json.dumps(payload, separators=(',', ':'), allow_nan=False)
    except ValueError as exc:
        raise Refused('REFUSED_NON_FINITE', str(exc)) from exc
    return text.encode('utf-8')


def decode(kind, raw, *, expect_ranges=None, expect_gate=True):
    """Parse and validate a datagram, or raise `Refused`.

    `expect_ranges` fixes the scan length: a state whose range count differs from what the
    ROS side is configured to publish as a LaserScan is a configuration mismatch, and
    publishing it anyway would put a wrong angle increment on the wire.
    """
    if kind not in KINDS:
        raise Refused('REFUSED_UNKNOWN_KIND', kind)
    expected = KINDS[kind]
    try:
        text = raw.decode('utf-8')
    except UnicodeDecodeError as exc:
        raise Refused('REFUSED_NOT_UTF8', str(exc)) from exc
    try:
        payload = json.loads(text, object_pairs_hook=_pairs_hook)
    except Refused:
        raise
    except json.JSONDecodeError as exc:
        raise Refused('REFUSED_NOT_JSON', str(exc)) from exc
    if not isinstance(payload, dict):
        raise Refused('REFUSED_NOT_OBJECT', type(payload).__name__)

    version = payload.get('schema_version')
    if version != SCHEMA_VERSION:
        raise Refused('REFUSED_SCHEMA_VERSION', repr(version))
    missing = expected - set(payload)
    extra = set(payload) - expected
    if missing:
        raise Refused('REFUSED_MISSING_FIELD', ','.join(sorted(missing)))
    if extra:
        raise Refused('REFUSED_UNKNOWN_FIELD', ','.join(sorted(extra)))

    sequence = payload['seq']
    if isinstance(sequence, bool) or not isinstance(sequence, int) or sequence < 0:
        raise Refused('REFUSED_BAD_SEQ', repr(sequence))

    if kind == 'state':
        for name, (low, high) in STATE_LIMITS.items():
            value = _finite(payload[name], name)
            if not low <= value <= high:
                raise Refused('REFUSED_OUT_OF_RANGE', f'{name}={value}')
        rates = payload['wheel_rate']
        if not isinstance(rates, list) or len(rates) != 2:
            raise Refused('REFUSED_BAD_TYPE', 'wheel_rate must be two numbers')
        for value in rates:
            if abs(_finite(value, 'wheel_rate')) > 200.0:
                raise Refused('REFUSED_OUT_OF_RANGE', f'wheel_rate={value}')
        ranges = payload['ranges']
        if not isinstance(ranges, list):
            raise Refused('REFUSED_BAD_TYPE', 'ranges must be a list')
        if expect_ranges is not None and len(ranges) != expect_ranges:
            raise Refused('REFUSED_RANGE_COUNT', f'{len(ranges)} != {expect_ranges}')
        for value in ranges:
            if not isinstance(value, (int, float)) or isinstance(value, bool):
                raise Refused('REFUSED_BAD_TYPE', 'range entry')
            if value != value:
                raise Refused('REFUSED_NON_FINITE', 'range entry')
            if not 0.0 <= float(value) <= payload['range_max'] * 1.0001:
                raise Refused('REFUSED_OUT_OF_RANGE', f'range={value}')
        gate = payload['gate']
        if expect_gate:
            if not isinstance(gate, dict):
                raise Refused('REFUSED_BAD_TYPE', 'gate')
            if set(gate) != GATE_KEYS:
                raise Refused('REFUSED_UNKNOWN_FIELD',
                              'gate keys ' + ','.join(sorted(set(gate) ^ GATE_KEYS)))
    else:
        for name in ('issued_sim_time', 'ttl_s', 'v', 'w'):
            _finite(payload[name], name)
        if payload['ttl_s'] <= 0.0:
            raise Refused('REFUSED_OUT_OF_RANGE', f"ttl_s={payload['ttl_s']}")
    return payload


def state(**kwargs):
    """Build a state payload; `schema_version` is filled in so callers cannot forget it."""
    kwargs.setdefault('schema_version', SCHEMA_VERSION)
    return kwargs


def command(**kwargs):
    """Build a command payload."""
    kwargs.setdefault('schema_version', SCHEMA_VERSION)
    return kwargs
