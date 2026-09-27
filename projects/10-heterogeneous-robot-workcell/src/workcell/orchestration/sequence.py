"""W4: the layer where the P2 core stops being a library and starts commanding skills.

WHAT WAS MISSING
----------------
`src/workcell` already had the schema, the order ledger, the resource table, the transfer state
machine and the command gate, and none of them had ever commanded anything. The guidance's W4 is
exactly that gap, and it names the ways it must not be closed:

  * no second order/resource system that leaves P2 as decoration;
  * "the timer ran out" is not `succeeded`;
  * "the command was sent" is not "the delivery happened";
  * wiping the ledger and starting a new one is not recovery.

WHAT THIS MODULE IS
-------------------
The PROTOCOL layer: it validates a request against `workcell.schema`, enforces one-command-one-
physical-action by `command_id`, refuses stale boots and stale revisions, puts EVERY motion
through the single authoritative gate (`workcell.safety.CommandGate`, per `D049`), and records
which real evidence each transaction state requires. It does not touch a simulator: the plant
arrives as an adapter, which is what lets the fault matrix be tested without physics and what
keeps "the protocol is right" and "the physics is right" as separate claims.

THE EVIDENCE TABLE IS DATA, NOT PROSE
-------------------------------------
`STATE_EVIDENCE` is the guidance's own mapping. A state may only be claimed when every named
piece of evidence is present AND fresh; `missing_evidence` returns what is absent rather than
letting a caller assert the state anyway. That is the difference between "DOCK_VERIFIED" as a
label and as a fact.
"""
import math

from workcell import schema as S
from workcell.safety import CommandGate, EvaluationBuilder

#: The single-vehicle integration sub-scenario, in order. Each entry is
#: (skill, station) where the station is None for skills that are not addressed to one.
SKILL_SCRIPT = (
    ('MOVE_TO_STATION', 'station_c'),
    ('DOCK', 'station_c'),
    ('START_TRANSFER', 'station_c'),
    ('VERIFY_TRANSFER', 'station_c'),
    ('UNDOCK', 'station_c'),
    ('MOVE_TO_STATION', 'station_r'),
    ('DOCK', 'station_r'),
    ('START_TRANSFER', 'station_r'),
    ('VERIFY_TRANSFER', 'station_r'),
    ('UNDOCK', 'station_r'),
    ('VERIFY_DELIVERY', None),
)

#: The guidance's transaction-state -> required real evidence table, as data. A state is claimed
#: only when every item is present; `missing_evidence` names what is absent.
STATE_EVIDENCE = {
    'DOCK_VERIFIED': ('relative_pose', 'stopped_confirmed', 'motion_permitted'),
    'BOTH_READY': ('source_holds_tray', 'receiver_empty', 'shared_zone_clear', 'stop_chain_healthy'),
    'TRANSFERRING': ('transfer_open', 'both_ends_reserved', 'ownership_unchanged'),
    'RECEIVER_CONFIRMED': ('tray_fully_received', 'source_cleared', 'speed_zero', 'support_verified'),
    'COMMITTED': ('ownership_updated', 'evidence_referenced'),
    'RELEASED': ('committed', 'safe_to_release'),
}

#: Stages a transaction legitimately enters WITHOUT making a physical claim, and why. The two
#: sets are a partition of the ledger's stages, checked by `classify_claims` against `STAGES`.
#: A state in neither is UNCLASSIFIED, which is reported rather than skipped: the first version of
#: the W5 judge called `missing_evidence` on every entered stage, crashed on `REQUESTED`, and would
#: have let any stage added later pass unjudged.
NO_PHYSICAL_CLAIM = {
    'REQUESTED': 'the transaction is booked and nothing has been measured yet',
    'BOTH_RESERVED': 'the two ends are reserved; the dock has NOT been verified',
}

#: Which skills need the vehicle's MOTION permission from the gate, and what they ask for. A
#: transfer skill asks for zero: the guidance says the navigation controller's permission is
#: closed before control passes to the align/dock controller, and "closed" has to be a command
#: the gate accepted, not a comment.
MOTION_INTENT = {
    'MOVE_TO_STATION': (0.25, 0.0),
    'DOCK': (0.05, 0.0),
    'START_TRANSFER': (0.0, 0.0),
    'VERIFY_TRANSFER': (0.0, 0.0),
    'UNDOCK': (0.0, 0.0),
    'VERIFY_DELIVERY': (0.0, 0.0),
    'STOP': (0.0, 0.0),
    'OBSERVE': (0.0, 0.0),
}

TIMEOUT_ORDER = frozenset({'REFUSED_STALE_EPOCH', 'REFUSED_STALE_GENERATION'})
STALE_BOOT = 'REFUSED_STALE_BOOT'
STALE_REVISION = 'REFUSED_STALE_REVISION'


class Record:
    """One execution record. Returned for the first submit AND for every repeat of it."""

    def __init__(self, *, command_id, order_id, revision, boot_id, skill, arguments):
        self.command_id = command_id
        self.order_id = order_id
        self.revision = revision
        self.boot_id = boot_id
        self.skill = skill
        self.arguments = arguments
        self.attempts = 0
        self.result = None
        self.refusal = None
        self.gate_offers = []

    @property
    def executed(self):
        return self.attempts > 0

    def to_dict(self):
        return {'command_id': self.command_id, 'order_id': self.order_id,
                'revision': self.revision, 'boot_id': self.boot_id, 'skill': self.skill,
                'arguments': dict(self.arguments), 'attempts': self.attempts,
                'refusal': self.refusal, 'gate_offers': list(self.gate_offers),
                'status': (self.result or {}).get('status') if self.result else None}


class SkillSequence:
    """Submit validated skill requests; execute each PHYSICAL action exactly once.

    ★ The invariant that matters: a repeated `command_id` returns the SAME record and does not
    call the plant and does not offer the gate. A retry is a protocol event; a second transfer is
    a physical one, and conflating them is how a lost acknowledgement becomes two deliveries.
    """

    def __init__(self, *, adapter, gate, boot_id, epoch, order_id, ttl_s=0.5, wall_clock=None):
        if not isinstance(gate, CommandGate):
            raise S.SchemaRefused('REFUSED_BAD_TYPE',
                                  'the sequence takes the authoritative CommandGate, not a duck',
                                  field='gate')
        self.adapter = adapter
        self.gate = gate
        self.boot_id = S._identifier(boot_id, 'boot_id')
        self.epoch = gate.bind_epoch(epoch)
        self.order_id = S._identifier(order_id, 'order_id')
        self.ttl_s = float(ttl_s)
        self.records = {}
        self.order = []
        self._seq = 0
        self._highest_revision = -1
        self.wall_clock = wall_clock or _default_clock()

    # -- submitting ---------------------------------------------------------

    def submit(self, raw, *, now_s):
        request = S.validate_skill_request(raw)
        command_id = request['command_id']
        existing = self.records.get(command_id)
        if existing is not None:
            # ★ IDEMPOTENT BY COMMAND ID. Same answer, no second action.
            return {'record': existing, 'idempotent': True, 'accepted': existing.refusal is None}

        record = Record(command_id=command_id, order_id=request['order_id'],
                        revision=request['revision'], boot_id=request['expected_boot_id'],
                        skill=request['skill'], arguments=request['arguments'])
        self.records[command_id] = record
        self.order.append(command_id)

        refusal = self._guard(request)
        if refusal is not None:
            record.refusal = refusal
            return {'record': record, 'idempotent': False, 'accepted': False}

        allowed, gate_answer = self._offer_motion(request, now_s=now_s)
        record.gate_offers.append({'generation': self.gate.generation, 'answer': gate_answer})
        if not allowed:
            # The gate is the single authority, so a refusal here is the end of it. Note that the
            # record is NOT marked executed: nothing physical happened.
            record.refusal = f'GATE_{gate_answer}'
            return {'record': record, 'idempotent': False, 'accepted': False}

        record.attempts += 1
        record.result = self.adapter.execute(request, now_s=now_s)
        S.validate_skill_result(record.result)
        self._highest_revision = max(self._highest_revision, request['revision'])
        return {'record': record, 'idempotent': False, 'accepted': True}

    def _guard(self, request):
        """Everything that must be true BEFORE a physical action, each with its own reason."""
        if request['expected_boot_id'] != self.boot_id:
            return STALE_BOOT
        if request['order_id'] != self.order_id:
            return 'REFUSED_OTHER_ORDER'
        if request['revision'] < self._highest_revision:
            # Contract section 3: a stale revision must not change the current task. Refusing is
            # the only safe reading -- "apply it if it looks harmless" is how a late callback
            # rewrites a finished task.
            return STALE_REVISION
        if request['revision'] == self._highest_revision and self._highest_revision >= 0:
            return STALE_REVISION
        return None

    def _offer_motion(self, request, *, now_s):
        v, w = MOTION_INTENT.get(request['skill'], (0.0, 0.0))
        self._seq += 1
        command = {'source': 'safety_gate', 'epoch': self.epoch,
                   'generation': self.gate.generation, 'seq': self._seq,
                   'issued_s': float(now_s), 'ttl_s': self.ttl_s, 'v': v, 'w': w}
        return self.gate.offer(command, now_s=now_s)

    # -- what the run actually did -----------------------------------------

    def evaluations(self, *, run_id):
        """One `EvaluationBuilder` over the whole sequence, so nothing is judged per-skill only."""
        builder = EvaluationBuilder(run_id=run_id)
        stops = [r for r in self.records.values() if r.refusal and r.refusal.startswith('GATE_')]
        failures = [r for r in self.records.values()
                    if r.executed and (r.result or {}).get('status') == 'FAILED']
        not_run = [r for r in self.records.values() if not r.executed]
        builder.check('every submitted command was either executed or refused with a reason',
                      'PASS' if all(r.executed or r.refusal for r in self.records.values())
                      else 'FAIL')
        builder.check('the gate was offered exactly once per executed command',
                      'PASS' if all(len(r.gate_offers) <= 1 for r in self.records.values())
                      else 'FAIL')
        builder.check('a repeated command_id did not repeat a physical action',
                      'PASS' if all(r.attempts <= 1 for r in self.records.values()) else 'FAIL')
        if not_run:
            builder.miss(f'{len(not_run)} commands never executed: '
                         f'{[r.command_id for r in not_run][:4]}')
        builder.check('no skill failed', 'PASS' if not failures else 'FAIL')
        builder.check('nothing was stopped by the gate', 'PASS' if not stops else 'FAIL')
        return builder

    def summary(self):
        return {'order_id': self.order_id, 'boot_id': self.boot_id, 'epoch': self.epoch,
                'submitted': len(self.order),
                'executed': sum(1 for r in self.records.values() if r.executed),
                'refused': {r.command_id: r.refusal for r in self.records.values() if r.refusal},
                'records': [self.records[c].to_dict() for c in self.order],
                'gate': self.gate.summary(now_s=self.wall_clock())}


def missing_evidence(state, present):
    """Which of a state's required evidence items are absent. Claiming a state is not enough.

    Raises for a stage in neither declared set: an undeclared stage is a contract bug, and the
    caller that wants to tolerate it must say so through `classify_claims` rather than through a
    bare `try`.
    """
    if state not in STATE_EVIDENCE:
        raise S.SchemaRefused('REFUSED_BAD_ENUM', repr(state), field='state')
    return [item for item in STATE_EVIDENCE[state] if not present.get(item)]


def classify_claims(states, present):
    """Sort the stages a run ENTERED into books / claims-with-gaps / unclassified.

    Returns a dict with `booking` (declared, no claim to evidence), `missing` (state -> the evidence
    items it needed and did not have) and `unclassified` (in neither set). Splitting this out of the
    judge is deliberate: as inline code it was a branch no test could reach, and the bug it hid was
    a crash on the very first stage.
    """
    booking, missing, unclassified = [], {}, []
    for state in states:
        if state in NO_PHYSICAL_CLAIM:
            booking.append(state)
            continue
        if state not in STATE_EVIDENCE:
            unclassified.append(state)
            continue
        gap = missing_evidence(state, present)
        if gap:
            missing[state] = gap
    return {'booking': sorted(booking), 'missing': missing, 'unclassified': sorted(unclassified)}


def unclassified_stages():
    """Which of the ledger's own stages neither set covers. Empty is the only correct answer."""
    from workcell import transfer as T
    declared = set(STATE_EVIDENCE) | set(NO_PHYSICAL_CLAIM)
    return sorted(set(T.STAGES) - declared)


def _default_clock():
    import time
    return time.monotonic
