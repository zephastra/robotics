"""Strict validation for everything that enters the 010 workcell's order plane.

`docs/CONTRACTS.md` section 1 names one rule that this module exists to enforce:

    Reject unknown fields, duplicate keys, NaN/Infinity, illegal entity/count/path.
    Input text and model output are not execution permission.

That last sentence is why this file is hostile to its input. A workcell that accepts a
loosely-typed order is a workcell whose failure appears three stages later, in a place that
looks nothing like the order. So the shape of every check here is the same as
`workcell_ipc`'s: refuse, never coerce. A coerced value looks exactly like a working link.

Four decisions worth stating, because each one is a thing that could quietly go wrong:

* **Exact key sets, not "required subsets".** An extra key is refused. A field that both ends
  ignore is a field nobody tests, and it is how a stale field survives a rename.
* **`bool` is not `int`.** Python makes `True == 1`, so an order with `"count": true` would
  otherwise pass an integer check and mean "one item". It is refused as `REFUSED_BAD_TYPE`.
* **Numbers must be finite and are range-checked per field.** `json.loads` accepts `Infinity`
  and `NaN` by default, and both compare strangely: `NaN != NaN`, and `Infinity > limit` is
  true but `Infinity - Infinity` is not a duration.
* **Duplicate keys are refused, not last-wins.** `{"count": 2, "count": 1}` has two readings
  and the standard library silently picks the second.

Nothing here knows about MuJoCo, ROS, or the ledger. It is stdlib-only so that both the
`.venv` interpreter and the system `/usr/bin/python3` can import it, for the same reason
`workcell_ipc` is: the two halves of 010 run under different Pythons.
"""
import json
import math
import re

SCHEMA_VERSION = 1

#: Units are fixed by the contract; these strings exist so a payload can declare them and be
#: checked, rather than having the unit live in a comment somewhere.
UNITS = ('m', 's', 'rad', 'N', 'kg')

# ---------------------------------------------------------------------------
# Identity formats
# ---------------------------------------------------------------------------

#: ID format is fixed here rather than "whatever the caller sends", because the contract says
#: "ID 格式/长度 ... 在 schema 中固定". Bounded length matters: an unbounded ID travels into
#: ledgers, filenames and log lines.
ID_PATTERN = re.compile(r'^[A-Za-z0-9][A-Za-z0-9_.:-]{0,63}$')

#: Every kind of identifier the plane uses, so a mistake is caught at the field that made it
#: rather than at the ledger key that swallowed it.
ID_MAX = 64

#: Evidence references are paths inside a report directory (`run-001/trace.json`), so they
#: need a different rule from an identifier. It is deliberately a *whitelist* of shapes rather
#: than a blacklist of bad ones: no leading slash (absolute), no `..` segment (traversal), no
#: backslash, bounded depth and length. An evidence ref ends up as a filename in a report, so
#: it is input that reaches the filesystem.
EVIDENCE_PATTERN = re.compile(r'^(?!/)(?!.*\.\.)(?!.*\\)[A-Za-z0-9._-]+(/[A-Za-z0-9._-]+){0,7}$')
EVIDENCE_REF_MAX = 200
REQUEST_ID_MAX = ID_MAX
ORDER_ID_MAX = ID_MAX

#: Site/entity references are drawn from a closed vocabulary. A free-form destination string is
#: how a typo becomes a station that exists nowhere and a task that waits forever.
STATION_IDS = frozenset({'station_a', 'station_b', 'station_c', 'station_home'})
PART_TYPES = frozenset({'red_block', 'blue_block', 'green_block', 'red_cylinder', 'blue_cylinder'})
TRAY_IDS = frozenset({'tray_v1_0', 'tray_v1_1', 'tray_v1_2'})
SLOT_IDS = frozenset({'slot_0', 'slot_1', 'slot_2'})

#: Contract section 2 fixes these. They are limits, not defaults: exceeding one is a refusal,
#: because a 10^6-line order is not a big order, it is a memory-exhaustion attempt.
MAX_ITEMS_PER_ORDER = 16
MAX_ITEM_COUNT = 99
MAX_TOTAL_PARTS = 256
MAX_PRIORITY = 1000

#: Transfer/quantity magnitudes. A path with a 10^9 m segment is not a long path.
MAX_DISTANCE_M = 1.0e4
MAX_DURATION_S = 1.0e6

# --- skills -----------------------------------------------------------------
#: Contract section 3's whitelist. `RETURN_TO_CHARGE` is explicitly reserved and NOT
#: registered: the contract says "不注册未实现的技能", so putting it here would be the same
#: defect as a typed constant -- a name that claims a capability nobody implemented.
SKILLS = frozenset({
    'OBSERVE', 'PRESENT_TRAY', 'PICK_PART', 'PLACE_PART', 'VERIFY_KIT',
    'MOVE_TO_STATION', 'DOCK', 'START_TRANSFER', 'VERIFY_TRANSFER',
    'UNDOCK', 'VERIFY_DELIVERY', 'STOP',
})
RESERVED_SKILLS = frozenset({'RETURN_TO_CHARGE'})

#: A device's `capabilities` list is NOT the skill whitelist. Some entries describe what the
#: device *is* (it has a battery, it can hold cargo) rather than what it can be *asked to do*.
#: Conflating the two is why this needed a fix: a vehicle declaring "battery" was refused with
#: REFUSED_BAD_ENUM because "battery" is not a skill. The distinction matters because the
#: battery entry is exactly the one that makes the battery reading mandatory below.
RESOURCE_CAPABILITIES = frozenset({'battery', 'cargo_hold', 'localization'})
CAPABILITIES = SKILLS | RESOURCE_CAPABILITIES

#: Arguments a skill may carry. Deliberately NOT a free-form dict: contract section 3 says
#: parameters may only reference registered items/stations/slots, and that paths and control
#: quantities are generated by a trusted adapter. A skill request that can carry `joint_targets`
#: or `shell` is a skill request that can carry anything.
SKILL_ARG_KEYS = {
    'OBSERVE': frozenset({'target_id'}),
    'PRESENT_TRAY': frozenset({'tray_id'}),
    'PICK_PART': frozenset({'part_id', 'slot_id'}),
    'PLACE_PART': frozenset({'part_id', 'slot_id'}),
    'VERIFY_KIT': frozenset({'tray_id', 'expected_bom'}),
    'MOVE_TO_STATION': frozenset({'station_id'}),
    'DOCK': frozenset({'station_id', 'transfer_id'}),
    'START_TRANSFER': frozenset({'transfer_id'}),
    'VERIFY_TRANSFER': frozenset({'transfer_id'}),
    'UNDOCK': frozenset({'station_id'}),
    'VERIFY_DELIVERY': frozenset({'order_id'}),
    'STOP': frozenset(),
}

#: Argument names that must never appear, whatever the skill. These are the contract's
#: "用户不能填写控制阈值、任意关节目标、shell命令或仿真坐标" made structural -- the refusal
#: happens by field name, so it cannot be forgotten for a newly added skill.
FORBIDDEN_ARG_NAMES = frozenset({
    'joint_targets', 'joint_target', 'qpos', 'qvel', 'ctrl', 'torque', 'force',
    'shell', 'cmd', 'command_line', 'argv', 'exec', 'eval', 'import',
    'x', 'y', 'z', 'pose', 'position', 'world_pose', 'sim_x', 'sim_y', 'sim_z',
    'threshold', 'thresholds', 'tolerance', 'ttl', 'timeout', 'deadline_s',
})

# --- enums the rest of P2 shares --------------------------------------------
ORDER_STATES = (
    'QUEUED', 'PREPARING_TRAY', 'KITTING', 'VERIFYING_KIT', 'WAITING_TRANSPORT',
    'LOADING', 'IN_TRANSIT', 'UNLOADING', 'VERIFYING_DELIVERY', 'SUCCEEDED',
    # constrained states: always entered with a `return_to` and a reason
    'WAITING', 'CANCELING', 'FAILED', 'NEEDS_ATTENTION',
    # terminal, reached only through a confirmed stop (see TERMINAL_ORDER_STATES)
    'CANCELLED',
)
#: The forward path only. Kept separate from ORDER_STATES so a transition table can be derived
#: from it instead of being typed out twice.
ORDER_FORWARD_PATH = (
    'QUEUED', 'PREPARING_TRAY', 'KITTING', 'VERIFYING_KIT', 'WAITING_TRANSPORT',
    'LOADING', 'IN_TRANSIT', 'UNLOADING', 'VERIFYING_DELIVERY', 'SUCCEEDED',
)
#: Terminal states: the order is done and nothing advances it. `CANCELLED` is here next to
#: `SUCCEEDED` because `CANCELING` needs somewhere to land -- an order parked in `CANCELING`
#: forever makes "we asked it to stop" indistinguishable from "it has stopped". Neither
#: `FAILED` nor `NEEDS_ATTENTION` is terminal: both are resumable or cancellable by design.
TERMINAL_ORDER_STATES = frozenset({'SUCCEEDED', 'CANCELLED'})
#: Constrained states are reachable from anywhere and must remember where they came from.
CONSTRAINED_ORDER_STATES = frozenset({'WAITING', 'CANCELING', 'FAILED', 'NEEDS_ATTENTION'})

CUSTODY_STATES = frozenset({
    'AT_SOURCE', 'ON_WORKCELL', 'TRANSFERRING', 'ON_AMR', 'AT_DESTINATION', 'UNKNOWN',
})
RESOURCE_STATES = frozenset({
    'FREE', 'RESERVED', 'OCCUPIED', 'CLEARING', 'UNKNOWN', 'BLOCKED',
})
TRANSFER_STATES = (
    'REQUESTED', 'BOTH_RESERVED', 'DOCK_VERIFIED', 'BOTH_READY',
    'TRANSFERRING', 'RECEIVER_CONFIRMED', 'COMMITTED', 'RELEASED',
)
TRANSFER_FAILURE_STATES = ('STOPPING', 'NEEDS_ATTENTION', 'ABORTED')

TASK_OUTCOMES = frozenset({'SUCCEEDED', 'FAILED', 'NEEDS_ATTENTION', 'CANCELLED'})
TRI_STATE = frozenset({'PASS', 'FAIL', 'UNKNOWN'})
PHYSICAL_RESULTS = frozenset({'PASS', 'FAIL', 'UNKNOWN', 'NOT_APPLICABLE'})

#: Contract section 7's reason list. A closed set, because "不要把所有失败写成
#: BUDGET_EXHAUSTED" is only enforceable if the writer must pick from a list.
REASON_CODES = frozenset({
    'NO_ELIGIBLE_DEVICE', 'OUT_OF_STOCK', 'TARGET_NOT_VISIBLE', 'STALE_OBSERVATION',
    'NO_FEASIBLE_GRASP', 'CONTACT_LOST', 'KIT_MISMATCH', 'DOCK_OUT_OF_TOLERANCE',
    'RECEIVER_OCCUPIED', 'TRANSFER_TIMEOUT', 'TRANSFER_UNKNOWN', 'PAYLOAD_LOST',
    'RESOURCE_UNKNOWN', 'PERMIT_EXPIRED', 'LOCALIZATION_INVALID', 'NAV_FAILED',
    'CANCEL_UNCONFIRMED', 'DEVICE_OFFLINE', 'CLOCK_RESET', 'EVIDENCE_MISSING',
    'BUDGET_EXHAUSTED', 'REQUEST_CONFLICT', 'INVALID_REQUEST', 'UNSUPPORTED_SKILL',
    'OUT_OF_TOLERANCE', 'NO_PROGRESS', 'MANUAL_INTERVENTION_REQUIRED',
})

#: Reason codes that are produced by the schema layer itself. Kept separate so a test can
#: assert the validator's own refusals are distinguishable from a business-level failure.
REFUSAL_REASONS = frozenset({
    'REFUSED_NOT_JSON', 'REFUSED_NOT_OBJECT', 'REFUSED_DUPLICATE_KEY',
    'REFUSED_UNKNOWN_FIELD', 'REFUSED_MISSING_FIELD', 'REFUSED_BAD_TYPE',
    'REFUSED_NON_FINITE', 'REFUSED_OUT_OF_RANGE', 'REFUSED_BAD_ID',
    'REFUSED_BAD_ENUM', 'REFUSED_SCHEMA_VERSION', 'REFUSED_TOO_MANY_ITEMS',
    'REFUSED_FORBIDDEN_FIELD', 'REFUSED_ENTITY_UNKNOWN', 'REFUSED_CONFLICT',
    'REFUSED_BAD_QUATERNION', 'REFUSED_BAD_TRANSITION', 'REFUSED_BAD_COUNT',
    # resource layer (P2-RES-02): each of these names a different reason a reservation or a
    # renewal was refused, because "the reservation failed" is not actionable.
    'REFUSED_RESOURCE_UNKNOWN', 'REFUSED_RESOURCE_BLOCKED', 'REFUSED_RESOURCE_OCCUPIED',
    'REFUSED_RESOURCE_CLEARING', 'REFUSED_RESOURCE_HELD_BY_OTHER',
    'REFUSED_RESOURCE_NOT_CLEARABLE', 'REFUSED_STALE_EPOCH',
    'REFUSED_STALE_GENERATION', 'REFUSED_NO_LIVE_GRANT', 'REFUSED_GRANT_OWNED_BY_OTHER',
    'REFUSED_GRANT_EXPIRED',
})


class SchemaRefused(ValueError):
    """A payload that must not be acted on, carrying a machine-readable reason.

    Same shape as `workcell_ipc.Refused`, on purpose: both halves of 010 raise the same
    kind of object for the same kind of problem, and the reason string is what a report
    counts. A refusal that says only "invalid input" cannot be counted, and a failure that
    cannot be counted is how a baseline stays red.
    """

    def __init__(self, reason, detail='', *, field=None):
        self.reason = reason
        self.detail = detail
        self.field = field
        text = f'{reason}: {detail}' if detail else reason
        if field:
            text = f'{reason}[{field}]: {detail}' if detail else f'{reason}[{field}]'
        super().__init__(text)


# ---------------------------------------------------------------------------
# primitives
# ---------------------------------------------------------------------------

def _pairs_hook(pairs):
    """Reject duplicate keys instead of silently keeping the last one."""
    seen = {}
    for key, value in pairs:
        if key in seen:
            raise SchemaRefused('REFUSED_DUPLICATE_KEY', key)
        seen[key] = value
    return seen


def _is_bool(value):
    return isinstance(value, bool)


def _finite(value, field):
    """A real number. `bool` is excluded even though `isinstance(True, int)` is true."""
    if _is_bool(value) or not isinstance(value, (int, float)):
        raise SchemaRefused('REFUSED_BAD_TYPE', f'expected a number, got {type(value).__name__}',
                            field=field)
    value = float(value)
    if not math.isfinite(value):
        # NaN compares false against everything including itself, so a NaN slip past here
        # would make every later `>` guard silently take the else branch.
        raise SchemaRefused('REFUSED_NON_FINITE', repr(value), field=field)
    return value


def _number_in(value, field, low, high):
    value = _finite(value, field)
    if not low <= value <= high:
        raise SchemaRefused('REFUSED_OUT_OF_RANGE', f'{value} not in [{low}, {high}]',
                            field=field)
    return value


def _int_in(value, field, low, high):
    if _is_bool(value) or not isinstance(value, int):
        # A NaN or an infinity is a float, so without this the caller is told "expected an int"
        # and never learns that the problem is that the value is not a number at all. Both
        # reasons are true; NON_FINITE is the one that says what to fix.
        if isinstance(value, float) and not math.isfinite(value):
            raise SchemaRefused('REFUSED_NON_FINITE', repr(value), field=field)
        raise SchemaRefused('REFUSED_BAD_TYPE', f'expected an int, got {type(value).__name__}',
                            field=field)
    if not low <= value <= high:
        raise SchemaRefused('REFUSED_OUT_OF_RANGE', f'{value} not in [{low}, {high}]',
                            field=field)
    return value


def _text(value, field, *, max_len=ID_MAX):
    if not isinstance(value, str):
        raise SchemaRefused('REFUSED_BAD_TYPE', f'expected a string, got {type(value).__name__}',
                            field=field)
    if not value:
        raise SchemaRefused('REFUSED_BAD_ID', 'empty', field=field)
    if len(value) > max_len:
        raise SchemaRefused('REFUSED_BAD_ID', f'length {len(value)} > {max_len}', field=field)
    return value


def _identifier(value, field, *, max_len=ID_MAX):
    text = _text(value, field, max_len=max_len)
    if not ID_PATTERN.match(text):
        raise SchemaRefused('REFUSED_BAD_ID', f'{text!r} is not a legal id', field=field)
    return text


def _evidence_ref(value, field):
    """An evidence path. Not an identifier: it has slashes. Still hostile to traversal."""
    text = _text(value, field, max_len=EVIDENCE_REF_MAX)
    if not EVIDENCE_PATTERN.match(text):
        # An absolute path or a `..` segment here would be read later as "the file named by
        # the evidence", so this is the field where it has to stop.
        raise SchemaRefused('REFUSED_BAD_ID',
                            f'{text!r} is not a legal evidence reference', field=field)
    return text


def _enum(value, field, allowed):
    if not isinstance(value, str) or value not in allowed:
        raise SchemaRefused('REFUSED_BAD_ENUM', f'{value!r} not in {sorted(allowed)}',
                            field=field)
    return value


def _mapping(value, field):
    if not isinstance(value, dict):
        raise SchemaRefused('REFUSED_BAD_TYPE',
                            f'expected an object, got {type(value).__name__}', field=field)
    return value


def _exact_keys(value, field, allowed, *, required=None):
    """Exact key set check. `required or allowed` means "all of allowed are required"."""
    if required is None:
        required = allowed
    allowed = frozenset(allowed)
    required = frozenset(required)
    missing = required - set(value)
    extra = set(value) - allowed
    if missing:
        raise SchemaRefused('REFUSED_MISSING_FIELD', ','.join(sorted(missing)), field=field)
    if extra:
        raise SchemaRefused('REFUSED_UNKNOWN_FIELD', ','.join(sorted(extra)), field=field)
    return value


def _load(raw):
    """Parse JSON text or bytes, refusing duplicates, non-objects and non-finite numbers."""
    if isinstance(raw, (bytes, bytearray)):
        try:
            raw = raw.decode('utf-8')
        except UnicodeDecodeError as exc:
            raise SchemaRefused('REFUSED_NOT_JSON', 'not utf-8') from exc
    if isinstance(raw, dict):
        return raw
    if not isinstance(raw, str):
        raise SchemaRefused('REFUSED_BAD_TYPE', f'expected str/bytes/object, got '
                            f'{type(raw).__name__}')
    try:
        payload = json.loads(raw, object_pairs_hook=_pairs_hook)
    except SchemaRefused:
        raise
    except json.JSONDecodeError as exc:
        raise SchemaRefused('REFUSED_NOT_JSON', str(exc)) from exc
    if not isinstance(payload, dict):
        raise SchemaRefused('REFUSED_NOT_OBJECT', type(payload).__name__)
    return payload


def _schema_version(payload):
    version = payload.get('schema_version')
    if _is_bool(version) or version != SCHEMA_VERSION:
        raise SchemaRefused('REFUSED_SCHEMA_VERSION', repr(version), field='schema_version')


# ---------------------------------------------------------------------------
# Order
# ---------------------------------------------------------------------------

ORDER_KEYS = frozenset({
    'schema_version', 'request_id', 'order_id', 'destination_id', 'items', 'priority',
})
ITEM_KEYS = frozenset({'type', 'count'})


def validate_order(raw):
    """Validate an order as described in `docs/CONTRACTS.md` section 2.

    Returns a normalised dict. Raises `SchemaRefused` with a countable reason otherwise.

    Normalisation is a copy with the values coerced to their declared types (counts to int,
    priority to int) -- never a *widening*: a string count is refused, not parsed.
    """
    payload = _load(raw)
    _schema_version(payload)
    _exact_keys(payload, 'order', ORDER_KEYS)

    request_id = _identifier(payload['request_id'], 'request_id', max_len=REQUEST_ID_MAX)
    order_id = _identifier(payload['order_id'], 'order_id', max_len=ORDER_ID_MAX)
    destination_id = _identifier(payload['destination_id'], 'destination_id')
    if destination_id not in STATION_IDS:
        raise SchemaRefused('REFUSED_ENTITY_UNKNOWN',
                            f'{destination_id!r} not in {sorted(STATION_IDS)}',
                            field='destination_id')

    items_raw = payload['items']
    if not isinstance(items_raw, list):
        # A dict here would let `{"red_block": 2}` read as one item named "red_block".
        raise SchemaRefused('REFUSED_BAD_TYPE', 'items must be a list', field='items')
    if not items_raw:
        raise SchemaRefused('REFUSED_OUT_OF_RANGE', 'items must not be empty', field='items')
    if len(items_raw) > MAX_ITEMS_PER_ORDER:
        raise SchemaRefused('REFUSED_TOO_MANY_ITEMS',
                            f'{len(items_raw)} > {MAX_ITEMS_PER_ORDER}', field='items')

    items = []
    seen_types = set()
    total = 0
    for index, entry in enumerate(items_raw):
        where = f'items[{index}]'
        _exact_keys(_mapping(entry, where), where, ITEM_KEYS)
        part_type = entry['type']
        _enum(part_type, f'{where}.type', PART_TYPES)
        if part_type in seen_types:
            # Two lines for one type have two readings -- "2 total" or "2+3". Refuse rather
            # than pick one; a merge that guesses is a stock level that drifts.
            raise SchemaRefused('REFUSED_BAD_COUNT',
                                f'{part_type!r} appears twice; merge into one line',
                                field=f'{where}.type')
        seen_types.add(part_type)
        count = _int_in(entry['count'], f'{where}.count', 1, MAX_ITEM_COUNT)
        total += count
        items.append({'type': part_type, 'count': count})

    if total > MAX_TOTAL_PARTS:
        raise SchemaRefused('REFUSED_TOO_MANY_ITEMS',
                            f'{total} parts > {MAX_TOTAL_PARTS}', field='items')

    priority = _int_in(payload['priority'], 'priority', 0, MAX_PRIORITY)

    return {
        'schema_version': SCHEMA_VERSION,
        'request_id': request_id,
        'order_id': order_id,
        'destination_id': destination_id,
        'items': items,
        'priority': priority,
    }


def content_fingerprint(order):
    """A stable fingerprint of an order's *content*, for the same-request_id rule.

    Contract section 2: "同 request_id 同内容返回原结果；不同内容拒绝冲突." So the ledger must
    compare content, and it must compare it in a way that does not depend on key order or on
    which of two equal-but-differently-written values arrived first.

    The fingerprint deliberately excludes nothing. A field added to `validate_order` later
    enters the fingerprint automatically -- the alternative (a hand-written field list) is the
    typed-constant defect: it would keep saying "same content" after the content changed.
    """
    canonical = json.dumps(order, sort_keys=True, separators=(',', ':'), allow_nan=False)
    return canonical


# ---------------------------------------------------------------------------
# DeviceState / SkillRequest / SkillResult
# ---------------------------------------------------------------------------

DEVICE_STATE_KEYS = frozenset({
    'schema_version', 'device_id', 'boot_id', 'sequence', 'sim_stamp', 'capabilities',
    'operating_state', 'active_command', 'pose', 'observation_age_s', 'stopped_confirmed',
    'held_payload_id', 'resources', 'fault', 'battery',
})
POSE_KEYS = frozenset({'frame', 'x', 'y', 'z', 'quaternion'})
OPERATING_STATES = frozenset({'IDLE', 'BUSY', 'DEGRADED', 'FAULT', 'OFFLINE', 'UNKNOWN'})

SKILL_REQUEST_KEYS = frozenset({
    'schema_version', 'command_id', 'order_id', 'revision', 'expected_boot_id',
    'skill', 'arguments', 'resource_generation', 'deadline_s',
})

SKILL_RESULT_KEYS = frozenset({
    'schema_version', 'command_id', 'order_id', 'revision', 'boot_id',
    'status', 'reason_code', 'evidence', 'final_state', 'start_s', 'end_s',
})
SKILL_RESULT_STATUS = frozenset({'ACCEPTED', 'RUNNING', 'SUCCEEDED', 'FAILED', 'CANCELED'})

FRAMES = frozenset({'map', 'odom', 'base_link', 'world'})


def validate_pose(raw):
    """A pose whose quaternion order is explicitly declared, because xyzw vs wxyz is silent.

    A quaternion given in the wrong order is a valid unit quaternion in the wrong frame -- it
    compiles, it runs, and it points somewhere else. The contract says the order is declared
    per interface, so this accepts exactly one spelling and says so in the key name.
    """
    payload = _load(raw)
    _exact_keys(payload, 'pose', POSE_KEYS)
    frame = _enum(payload['frame'], 'pose.frame', FRAMES)
    x = _number_in(payload['x'], 'pose.x', -MAX_DISTANCE_M, MAX_DISTANCE_M)
    y = _number_in(payload['y'], 'pose.y', -MAX_DISTANCE_M, MAX_DISTANCE_M)
    z = _number_in(payload['z'], 'pose.z', -MAX_DISTANCE_M, MAX_DISTANCE_M)
    quat = payload['quaternion']
    if not isinstance(quat, list) or len(quat) != 4:
        raise SchemaRefused('REFUSED_BAD_QUATERNION', 'must be [x, y, z, w]',
                            field='pose.quaternion')
    # Everything about a malformed quaternion is reported as BAD_QUATERNION, including a
    # non-finite component: NON_FINITE would be true but less useful, because the caller's
    # problem is that this field is not a rotation, not that numbers in general are bad.
    values = []
    for i, v in enumerate(quat):
        if _is_bool(v) or not isinstance(v, (int, float)) or not math.isfinite(float(v)):
            raise SchemaRefused('REFUSED_BAD_QUATERNION',
                                f'component {i} is not a finite number', field='pose.quaternion')
        values.append(float(v))
    norm = math.sqrt(sum(v * v for v in values))
    if not 0.9 <= norm <= 1.1:
        raise SchemaRefused('REFUSED_BAD_QUATERNION',
                            f'norm {norm:.6f} is not a rotation', field='pose.quaternion')
    return {'frame': frame, 'x': x, 'y': y, 'z': z,
            'quaternion': values, 'quaternion_order': 'xyzw'}


def validate_device_state(raw):
    """A device's self-report. `battery` is required only when `capabilities` lists it.

    The conditional is the interesting part: `battery（适用时）` in the contract is a
    *conditional requirement*, and a validator that ignores the condition accepts a vehicle
    with no battery field, which reads downstream as "battery fine".
    """
    payload = _load(raw)
    _schema_version(payload)
    _exact_keys(payload, 'device_state', DEVICE_STATE_KEYS)

    device_id = _identifier(payload['device_id'], 'device_id')
    boot_id = _identifier(payload['boot_id'], 'boot_id')
    sequence = _int_in(payload['sequence'], 'sequence', 0, 2 ** 62)
    sim_stamp = _number_in(payload['sim_stamp'], 'sim_stamp', 0.0, MAX_DURATION_S)
    age = _number_in(payload['observation_age_s'], 'observation_age_s', 0.0, MAX_DURATION_S)
    stopped = payload['stopped_confirmed']
    if not _is_bool(stopped):
        raise SchemaRefused('REFUSED_BAD_TYPE', 'stopped_confirmed must be a bool',
                            field='stopped_confirmed')

    caps = payload['capabilities']
    if not isinstance(caps, list) or not caps:
        raise SchemaRefused('REFUSED_BAD_TYPE', 'capabilities must be a non-empty list',
                            field='capabilities')
    for index, capability in enumerate(caps):
        _enum(capability, f'capabilities[{index}]', CAPABILITIES)

    operating = _enum(payload['operating_state'], 'operating_state', OPERATING_STATES)

    active = payload['active_command']
    if active is not None:
        active = _identifier(active, 'active_command')

    held = payload['held_payload_id']
    if held is not None:
        held = _identifier(held, 'held_payload_id')

    resources = payload['resources']
    if not isinstance(resources, list):
        raise SchemaRefused('REFUSED_BAD_TYPE', 'resources must be a list', field='resources')
    checked_resources = []
    for index, entry in enumerate(resources):
        where = f'resources[{index}]'
        entry = _mapping(entry, where)
        _exact_keys(entry, where, frozenset({'resource_id', 'state'}))
        checked_resources.append({
            'resource_id': _identifier(entry['resource_id'], f'{where}.resource_id'),
            'state': _enum(entry['state'], f'{where}.state', RESOURCE_STATES),
        })

    fault = payload['fault']
    if fault is not None:
        _enum(fault, 'fault', REASON_CODES)

    battery = payload['battery']
    if battery is None:
        if 'battery' in caps:
            # Declaring the capability and then omitting the reading is how a flat battery
            # presents as a healthy vehicle.
            raise SchemaRefused('REFUSED_MISSING_FIELD',
                                'capability battery declared but no reading given',
                                field='battery')
        battery_out = None
    else:
        battery_out = _number_in(battery, 'battery', 0.0, 1.0)

    return {
        'schema_version': SCHEMA_VERSION,
        'device_id': device_id,
        'boot_id': boot_id,
        'sequence': sequence,
        'sim_stamp': sim_stamp,
        'capabilities': list(caps),
        'operating_state': operating,
        'active_command': active,
        'pose': validate_pose(payload['pose']),
        'observation_age_s': age,
        'stopped_confirmed': stopped,
        'held_payload_id': held,
        'resources': checked_resources,
        'fault': fault,
        'battery': battery_out,
    }


def _check_arguments(skill, arguments):
    """Arguments may only reference registered entities, and only the ones the skill takes."""
    arguments = _mapping(arguments, 'skill.arguments')
    allowed = SKILL_ARG_KEYS[skill]
    extra = set(arguments) - allowed
    if extra:
        forbidden = sorted(extra & FORBIDDEN_ARG_NAMES)
        if forbidden:
            # Named separately: this is not a typo, it is an attempt to pass a control
            # quantity. The reason code differs so it is countable apart from a misspelling.
            raise SchemaRefused(
                'REFUSED_FORBIDDEN_FIELD', ','.join(forbidden), field='skill.arguments')
        raise SchemaRefused('REFUSED_UNKNOWN_FIELD', ','.join(sorted(extra)),
                            field='skill.arguments')
    for key, value in arguments.items():
        if key == 'expected_bom':
            _validate_bom(value, 'skill.arguments.expected_bom')
        elif key in ('part_id', 'tray_id', 'transfer_id'):
            _identifier(value, f'skill.arguments.{key}')
        elif key == 'slot_id':
            _enum(value, 'skill.arguments.slot_id', SLOT_IDS)
        elif key == 'station_id':
            _enum(value, 'skill.arguments.station_id', STATION_IDS)
        elif key == 'order_id':
            _identifier(value, 'skill.arguments.order_id')
        elif key == 'target_id':
            _identifier(value, 'skill.arguments.target_id')
        else:  # pragma: no cover -- SKILL_ARG_KEYS and this branch are kept in step by a test
            raise SchemaRefused('REFUSED_UNKNOWN_FIELD', key, field='skill.arguments')
    required = REQUIRED_SKILL_ARGUMENTS.get(skill, frozenset())
    missing = required - set(arguments)
    if missing:
        # Refuse rather than default. An absent `station_id` filled in from a default is a
        # command sent somewhere the caller never named.
        raise SchemaRefused('REFUSED_MISSING_FIELD', ','.join(sorted(missing)),
                            field='skill.arguments')
    return dict(arguments)


#: Which arguments each skill cannot run without. Separate from `SKILL_ARG_KEYS` (which is the
#: *allowed* set) because "may carry" and "must carry" are different questions, and collapsing
#: them would make every optional argument mandatory.
REQUIRED_SKILL_ARGUMENTS = {
    'OBSERVE': frozenset(),
    'PRESENT_TRAY': frozenset({'tray_id'}),
    'PICK_PART': frozenset({'part_id'}),
    'PLACE_PART': frozenset({'part_id', 'slot_id'}),
    'VERIFY_KIT': frozenset({'tray_id', 'expected_bom'}),
    'MOVE_TO_STATION': frozenset({'station_id'}),
    'DOCK': frozenset({'station_id', 'transfer_id'}),
    'START_TRANSFER': frozenset({'transfer_id'}),
    'VERIFY_TRANSFER': frozenset({'transfer_id'}),
    'UNDOCK': frozenset({'station_id'}),
    'VERIFY_DELIVERY': frozenset({'order_id'}),
    'STOP': frozenset(),
}


def _validate_bom(value, field):
    """An expected bill of materials: a list of {type, count} with no duplicate types."""
    if not isinstance(value, list) or not value:
        raise SchemaRefused('REFUSED_BAD_TYPE', 'expected_bom must be a non-empty list',
                            field=field)
    if len(value) > MAX_ITEMS_PER_ORDER:
        raise SchemaRefused('REFUSED_TOO_MANY_ITEMS', f'{len(value)}',
                            field=field)
    seen = set()
    for index, entry in enumerate(value):
        where = f'{field}[{index}]'
        _exact_keys(_mapping(entry, where), where, ITEM_KEYS)
        part_type = _enum(entry['type'], f'{where}.type', PART_TYPES)
        if part_type in seen:
            raise SchemaRefused('REFUSED_BAD_COUNT', f'{part_type!r} appears twice', field=where)
        seen.add(part_type)
        _int_in(entry['count'], f'{where}.count', 1, MAX_ITEM_COUNT)
    return value


def validate_skill_request(raw):
    """A request to run one whitelisted skill against registered entities."""
    payload = _load(raw)
    _schema_version(payload)
    _exact_keys(payload, 'skill_request', SKILL_REQUEST_KEYS)

    command_id = _identifier(payload['command_id'], 'command_id')
    order_id = _identifier(payload['order_id'], 'order_id')
    revision = _int_in(payload['revision'], 'revision', 0, 2 ** 31)
    expected_boot_id = _identifier(payload['expected_boot_id'], 'expected_boot_id')

    skill = payload['skill']
    if not isinstance(skill, str) or skill not in SKILLS:
        if isinstance(skill, str) and skill in RESERVED_SKILLS:
            raise SchemaRefused('REFUSED_BAD_ENUM',
                                f'{skill!r} is reserved and not registered', field='skill')
        raise SchemaRefused('REFUSED_BAD_ENUM', f'{skill!r} not in {sorted(SKILLS)}',
                            field='skill')

    resource_generation = _int_in(payload['resource_generation'], 'resource_generation',
                                  0, 2 ** 62)
    deadline_s = _number_in(payload['deadline_s'], 'deadline_s', 0.0, MAX_DURATION_S)

    return {
        'schema_version': SCHEMA_VERSION,
        'command_id': command_id,
        'order_id': order_id,
        'revision': revision,
        'expected_boot_id': expected_boot_id,
        'skill': skill,
        'arguments': _check_arguments(skill, payload['arguments']),
        'resource_generation': resource_generation,
        'deadline_s': deadline_s,
    }


def validate_skill_result(raw):
    """What came back from one skill call.

    A result carries `boot_id` and `revision` so the ledger can tell "this is the answer to a
    question I am still asking" from "this is the answer to a question I asked before the
    device restarted". Contract section 3: "旧 revision/boot_id 结果不得改变现任务."
    """
    payload = _load(raw)
    _schema_version(payload)
    _exact_keys(payload, 'skill_result', SKILL_RESULT_KEYS)

    status = _enum(payload['status'], 'status', SKILL_RESULT_STATUS)
    command_id = _identifier(payload['command_id'], 'command_id')
    order_id = _identifier(payload['order_id'], 'order_id')
    revision = _int_in(payload['revision'], 'revision', 0, 2 ** 31)
    boot_id = _identifier(payload['boot_id'], 'boot_id')

    reason = payload['reason_code']
    if status in ('FAILED', 'CANCELED'):
        if reason is None:
            # A failure with no reason cannot be counted, and an uncountable failure is the
            # shape of "everything becomes BUDGET_EXHAUSTED".
            raise SchemaRefused('REFUSED_MISSING_FIELD',
                                f'{status} requires a reason_code', field='reason_code')
        _enum(reason, 'reason_code', REASON_CODES)
    elif reason is not None:
        _enum(reason, 'reason_code', REASON_CODES)

    evidence = payload['evidence']
    if not isinstance(evidence, list):
        raise SchemaRefused('REFUSED_BAD_TYPE', 'evidence must be a list', field='evidence')
    for index, ref in enumerate(evidence):
        _evidence_ref(ref, f'evidence[{index}]')

    start_s = _number_in(payload['start_s'], 'start_s', 0.0, MAX_DURATION_S)
    end_s = _number_in(payload['end_s'], 'end_s', 0.0, MAX_DURATION_S)
    if end_s < start_s:
        raise SchemaRefused('REFUSED_OUT_OF_RANGE',
                            f'end_s {end_s} < start_s {start_s}', field='end_s')

    final_state = payload['final_state']
    if final_state is not None:
        final_state = _mapping(final_state, 'final_state')

    return {
        'schema_version': SCHEMA_VERSION,
        'command_id': command_id,
        'order_id': order_id,
        'revision': revision,
        'boot_id': boot_id,
        'status': status,
        'reason_code': reason,
        'evidence': list(evidence),
        'final_state': final_state,
        'start_s': start_s,
        'end_s': end_s,
    }


# ---------------------------------------------------------------------------
# evaluation aggregation (contract section 7)
# ---------------------------------------------------------------------------

EVALUATION_KEYS = frozenset({
    'schema_version', 'run_id',
    'task_outcome', 'physical_result', 'safety_result', 'expected_behavior',
    'evidence_complete', 'missing',
})


def validate_evaluation(raw):
    """The five separate result fields, and the rule that governs the top-level verdict.

    Contract section 7: "正常任务全部必需检查 PASS 且数据完整才能顶层 ACCEPTED；
    缺项 UNKNOWN/NOT_RUN 则不能 ACCEPTED."

    This module does not decide ACCEPTED -- it checks that the fields are *well formed*, and
    refuses the two ways of smuggling a pass in: a missing field defaulted to PASS, and an
    incomplete run whose gaps are not listed.
    """
    payload = _load(raw)
    _schema_version(payload)
    _exact_keys(payload, 'evaluation', EVALUATION_KEYS)

    run_id = _identifier(payload['run_id'], 'run_id', max_len=200)
    task_outcome = _enum(payload['task_outcome'], 'task_outcome', TASK_OUTCOMES)
    physical_result = _enum(payload['physical_result'], 'physical_result', PHYSICAL_RESULTS)
    # safety_result deliberately excludes NOT_APPLICABLE: "we did not check safety" is not a
    # third option, it is UNKNOWN.
    safety_result = _enum(payload['safety_result'], 'safety_result', TRI_STATE)
    expected_behavior = _enum(payload['expected_behavior'], 'expected_behavior', TRI_STATE)

    complete = payload['evidence_complete']
    if not _is_bool(complete):
        raise SchemaRefused('REFUSED_BAD_TYPE', 'evidence_complete must be a bool',
                            field='evidence_complete')
    missing = payload['missing']
    if not isinstance(missing, list):
        raise SchemaRefused('REFUSED_BAD_TYPE', 'missing must be a list', field='missing')
    for index, item in enumerate(missing):
        _text(item, f'missing[{index}]', max_len=200)
    if not complete and not missing:
        raise SchemaRefused('REFUSED_MISSING_FIELD',
                            'evidence_complete is false but nothing is listed as missing',
                            field='missing')
    if complete and missing:
        raise SchemaRefused('REFUSED_CONFLICT',
                            'evidence_complete is true but gaps are listed', field='missing')

    return {
        'schema_version': SCHEMA_VERSION,
        'run_id': run_id,
        'task_outcome': task_outcome,
        'physical_result': physical_result,
        'safety_result': safety_result,
        'expected_behavior': expected_behavior,
        'evidence_complete': complete,
        'missing': list(missing),
    }


def top_level_accepted(evaluation):
    """Whether an evaluation may be reported as ACCEPTED, and if not, what is missing.

    Deliberately a function over a *validated* evaluation, and deliberately conservative:
    every field must be positively good and the evidence must be complete. A single UNKNOWN
    anywhere means no ACCEPTED, which is the contract's rule and also the only reading that
    does not let "we didn't look" become "fine".
    """
    blockers = []
    if not evaluation['evidence_complete']:
        blockers.append('EVIDENCE_INCOMPLETE')
    if evaluation['task_outcome'] != 'SUCCEEDED':
        blockers.append(f"TASK_{evaluation['task_outcome']}")
    if evaluation['physical_result'] != 'PASS':
        blockers.append(f"PHYSICAL_{evaluation['physical_result']}")
    if evaluation['safety_result'] != 'PASS':
        blockers.append(f"SAFETY_{evaluation['safety_result']}")
    if evaluation['expected_behavior'] != 'PASS':
        blockers.append(f"EXPECTED_{evaluation['expected_behavior']}")
    return (not blockers), blockers


def accepted_or_refused(evaluation):
    """Raise unless the evaluation may be reported ACCEPTED. Returns None on success."""
    ok, blockers = top_level_accepted(evaluation)
    if not ok:
        raise SchemaRefused('REFUSED_CONFLICT',
                            f'not ACCEPTED: {",".join(blockers)}', field='evaluation')
    return None


__all__ = [
    'SCHEMA_VERSION', 'UNITS', 'SKILLS', 'RESERVED_SKILLS', 'SKILL_ARG_KEYS',
    'REQUIRED_SKILL_ARGUMENTS', 'FORBIDDEN_ARG_NAMES', 'ORDER_STATES',
    'ORDER_FORWARD_PATH', 'TERMINAL_ORDER_STATES', 'CONSTRAINED_ORDER_STATES',
    'CUSTODY_STATES', 'RESOURCE_STATES', 'TRANSFER_STATES', 'TRANSFER_FAILURE_STATES',
    'TASK_OUTCOMES', 'TRI_STATE', 'PHYSICAL_RESULTS', 'REASON_CODES', 'REFUSAL_REASONS',
    'CAPABILITIES', 'RESOURCE_CAPABILITIES', 'EVIDENCE_PATTERN', 'EVIDENCE_REF_MAX',
    'STATION_IDS', 'PART_TYPES', 'TRAY_IDS', 'SLOT_IDS', 'MAX_ITEMS_PER_ORDER',
    'MAX_ITEM_COUNT', 'MAX_TOTAL_PARTS', 'MAX_PRIORITY', 'SchemaRefused',
    'validate_order', 'content_fingerprint', 'validate_pose', 'validate_device_state',
    'validate_skill_request', 'validate_skill_result', 'validate_evaluation',
    'top_level_accepted', 'accepted_or_refused',
]
