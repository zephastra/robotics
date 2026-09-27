"""The order ledger: one order, one state machine, idempotent by `request_id`.

`docs/CONTRACTS.md` section 2 fixes the forward path and the constrained states:

    QUEUED -> PREPARING_TRAY -> KITTING -> VERIFYING_KIT -> WAITING_TRANSPORT -> LOADING
           -> IN_TRANSIT -> UNLOADING -> VERIFYING_DELIVERY -> SUCCEEDED

    任意阶段可进入受约束 WAITING、CANCELING、FAILED、NEEDS_ATTENTION；保留原阶段和具体原因。

Every design choice here comes from a failure this project has already paid for, or from a
rule the contract states outright. The three that shape the file:

* **The forward path is derived from `schema.ORDER_FORWARD_PATH`, not retyped.** Hard lesson 8:
  a constant typed into a new place cannot notice that the old place changed. If the contract's
  path grows a stage, the transition table here grows it too, and there is a test asserting the
  two agree.

* **A constrained state remembers where it came from.** "保留原阶段" is not decoration: an order
  in `NEEDS_ATTENTION` must be resumable, and a state machine that only records "not running"
  cannot tell a stuck `KITTING` from a stuck `IN_TRANSIT`. So `return_to` is mandatory on entry
  and is what `resume()` reads.

* **Idempotency is by content, not by arrival order.** "同 request_id 同内容返回原结果；
  不同内容拒绝冲突." The second half is the interesting one: a *different* payload under a
  `request_id` already in the ledger is not a retry, it is a conflict, and must be refused
  rather than overwrite the first one.

The ledger is deliberately in-memory and pure. It does not know about time, devices, or
sensors; those arrive in P2-RES-02 and P2-XFER-03 as inputs to the transitions, not as
dependencies of them.
"""
from . import schema as S
from .schema import SchemaRefused

#: Derived, never retyped: the next stage of the happy path, built from the contract's list.
_NEXT = {a: b for a, b in zip(S.ORDER_FORWARD_PATH, S.ORDER_FORWARD_PATH[1:])}

#: Which constrained states may be entered from any stage. The contract names exactly four.
CONSTRAINED = S.CONSTRAINED_ORDER_STATES

#: States from which no further transition is possible at all. `SUCCEEDED` is the only one:
#: `FAILED` and `NEEDS_ATTENTION` are resumable or cancellable by design, because an order that
#: reached the end of its useful life is a decision, not a state the ledger can infer.
DEAD_ENDS = S.TERMINAL_ORDER_STATES


class OrderRecord:
    """The ledger's row for one order. Mutable, but only through the ledger's transitions.

    `history` is the ordered list of transitions, and it is the thing that makes a stuck order
    diagnosable: contract section 7 asks for "最后有效进展、谁在等谁和首个拒绝原因", and all three
    are answerable from the history but not from the current state alone.
    """

    def __init__(self, order, *, state='QUEUED', order_id=None):
        self.order = order
        self.order_id = order_id or order['order_id']
        self.request_id = order['request_id']
        self.fingerprint = S.content_fingerprint(order)
        self.state = state
        self.revision = 0
        self.return_to = None           # set when a constrained state is entered
        self.reason_code = None         # set when a constrained state is entered
        self.history = [{'revision': 0, 'from': None, 'to': state,
                         'reason_code': None, 'return_to': None}]
        self.last_progress = state      # the last *forward* stage actually reached

    # -- queries -----------------------------------------------------------
    def is_terminal(self):
        return self.state in DEAD_ENDS

    def is_constrained(self):
        return self.state in CONSTRAINED

    def to_dict(self):
        return {
            'order_id': self.order_id,
            'request_id': self.request_id,
            'state': self.state,
            'revision': self.revision,
            'return_to': self.return_to,
            'reason_code': self.reason_code,
            'last_progress': self.last_progress,
            'history': [dict(h) for h in self.history],
            'order': self.order,
        }

    def __repr__(self):  # pragma: no cover -- debugging aid
        return (f'<OrderRecord {self.order_id} state={self.state} rev={self.revision} '
                f'return_to={self.return_to} reason={self.reason_code}>')


def _advance_allowed(current, target):
    """Whether `current -> target` is a legal forward step.

    Only the declared next stage is legal. Allowing "skip ahead" would let an order reach
    `IN_TRANSIT` without ever having been kitted, which is the shape of a state machine that
    reports the destination of a journey it never took.
    """
    return _NEXT.get(current) == target


class OrderLedger:
    """Holds orders and the only legal ways to change their state.

    Two indexes, not one: `by_request` is what makes idempotency checkable, and `by_order` is
    what `advance` resolves against. A single index on one of them would silently accept a
    second submission under the same request_id with a different order_id -- which is exactly
    the "different content, same request" case the contract says to refuse.
    """

    def __init__(self):
        self.by_request = {}    # request_id -> order_id
        self.by_order = {}      # order_id   -> OrderRecord
        self.refusals = []      # every refusal, as (reason, detail) -- countable, per contract 7

    # -- submission --------------------------------------------------------

    def submit(self, raw):
        """Accept an order, or return the previously accepted one, or refuse a conflict.

        Returns `(record, created)` where `created` is False when this was a replay. The
        replay returns the *original* record rather than a fresh copy, because the caller's
        next question is always "what happened to it", not "what did I send".

        Refuses, with distinct reasons:
          * a malformed order                    -> whatever `validate_order` says
          * same request_id, same content        -> replay, `created=False` (not an error)
          * same request_id, different content   -> REFUSED_CONFLICT
          * same order_id,  different request_id -> REFUSED_CONFLICT
        """
        order = S.validate_order(raw)
        request_id = order['request_id']
        order_id = order['order_id']
        fingerprint = S.content_fingerprint(order)

        existing_order_id = self.by_request.get(request_id)
        if existing_order_id is not None:
            existing = self.by_order[existing_order_id]
            if existing.fingerprint == fingerprint:
                return existing, False
            # Same request, different content. Silently updating would make the ledger's
            # answer depend on which copy arrived last, and a retry after a dropped reply is
            # indistinguishable from an edited order.
            self._refuse('REFUSED_CONFLICT',
                         f'request_id {request_id!r} already used with different content')
            raise SchemaRefused('REFUSED_CONFLICT',
                                f'request_id {request_id!r} already carries different content',
                                field='request_id')

        holder = self.by_order.get(order_id)
        if holder is not None and holder.request_id != request_id:
            self._refuse('REFUSED_CONFLICT',
                         f'order_id {order_id!r} already belongs to {holder.request_id!r}')
            raise SchemaRefused('REFUSED_CONFLICT',
                                f'order_id {order_id!r} already belongs to '
                                f'{holder.request_id!r}', field='order_id')

        record = OrderRecord(order)
        self.by_request[request_id] = order_id
        self.by_order[order_id] = record
        return record, True

    # -- transitions -------------------------------------------------------

    def advance(self, order_id, target=None):
        """Move one stage forward, or to `target` if it is exactly the declared next stage.

        Refuses a skip and a backwards move. Neither is a "nice to have" guard: a backwards
        move is what a replayed event looks like, and accepting it would make the ledger's
        revision counter go up while the order went back to `KITTING`.
        """
        record = self._require(order_id)
        if record.is_constrained():
            raise SchemaRefused(
                'REFUSED_BAD_TRANSITION',
                f'order is in {record.state}; resume() it to {record.return_to} first',
                field='state')
        if record.is_terminal():
            raise SchemaRefused('REFUSED_BAD_TRANSITION',
                                f'order is {record.state}; there is no next stage',
                                field='state')

        if target is None:
            target = _NEXT.get(record.state)
            if target is None:
                raise SchemaRefused('REFUSED_BAD_TRANSITION',
                                    f'{record.state} has no declared next stage',
                                    field='state')
        elif not _advance_allowed(record.state, target):
            # Both a skip and a backwards step land here, and the message says which.
            expected = _NEXT.get(record.state)
            raise SchemaRefused(
                'REFUSED_BAD_TRANSITION',
                f'{record.state} -> {target} is not the declared step'
                + (f' (expected {expected})' if expected else ' (no forward step exists)'),
                field='state')

        self._move(record, target, reason_code=None, return_to=None)
        return record

    def constrain(self, order_id, state, reason_code):
        """Enter one of the four constrained states, recording where to come back to.

        `reason_code` is mandatory and must come from the contract's list. A constrained order
        with no reason is an order nobody can act on, and "an uncountable failure" is the
        defect the contract names explicitly ("不要把所有失败写成 BUDGET_EXHAUSTED").
        """
        state = S._enum(state, 'state', CONSTRAINED)
        if reason_code is None or reason_code not in S.REASON_CODES:
            raise SchemaRefused('REFUSED_BAD_ENUM',
                                f'{reason_code!r} is not a contract reason code',
                                field='reason_code')
        record = self._require(order_id)
        if record.is_terminal():
            raise SchemaRefused('REFUSED_BAD_TRANSITION',
                                f'order is {record.state}; cannot enter {state}',
                                field='state')
        if record.is_constrained():
            # Re-constraining an already constrained order is how a stuck order loses the
            # stage it was actually stuck at.
            raise SchemaRefused(
                'REFUSED_BAD_TRANSITION',
                f'already {record.state} (return_to={record.return_to}); resume or cancel first',
                field='state')
        self._move(record, state, reason_code=reason_code, return_to=record.state)
        return record

    def resume(self, order_id):
        """Return a constrained order to the stage it came from.

        This is the half that makes `return_to` load-bearing. Resuming to "somewhere sensible"
        would be a second, hidden transition table.
        """
        record = self._require(order_id)
        if not record.is_constrained():
            raise SchemaRefused('REFUSED_BAD_TRANSITION',
                                f'{record.state} is not a constrained state', field='state')
        target = record.return_to
        if target is None:
            # Only reachable if a record was constructed by hand without a return_to; the
            # transitions above cannot produce it, so this is a load-bearing assertion.
            raise SchemaRefused('REFUSED_BAD_TRANSITION',
                                'constrained order has no return_to', field='return_to')
        self._move(record, target, reason_code=None, return_to=None)
        return record

    def cancel(self, order_id, reason_code='CANCEL_UNCONFIRMED'):
        """Request cancellation. This is not the same as being cancelled.

        Contract section 3: "取消必须确认停稳或明确冻结/attention". So this moves the order to
        `CANCELING` and nothing else -- reaching a terminal cancelled state requires an
        explicit `confirm_canceled`, which is the thing a caller must not be able to skip.
        `CANCEL_UNCONFIRMED` is the default reason precisely because the honest starting
        position is "we asked, we have not seen it stop".
        """
        return self.constrain(order_id, 'CANCELING', reason_code)

    def confirm_canceled(self, order_id, *, stopped_confirmed):
        """Finish a cancellation, and only if stopping was actually observed.

        `stopped_confirmed` is a required keyword with no default. A default of True would make
        every cancellation succeed, which is the failure mode the contract's "必须确认停稳"
        exists to prevent; a default of False would make the parameter ceremonial.
        """
        record = self._require(order_id)
        if record.state != 'CANCELING':
            raise SchemaRefused('REFUSED_BAD_TRANSITION',
                                f'order is {record.state}, not CANCELING', field='state')
        if stopped_confirmed is not True:
            # Stay in CANCELING and say so. This is the "明确冻结/attention" half: the order
            # does not silently become cancelled, it becomes something a person must look at.
            #
            # This uses an explicit move rather than `constrain()`, because an order that was
            # already constrained (it is in CANCELING) must not go through the re-constrain
            # guard. `return_to` is carried through to CANCELING's own return target, so
            # resuming from the frozen state still lands on the stage the order was running at
            # -- the first version of this method called `constrain()` and was refused by its
            # own guard, which is how "the frozen branch was unreachable" was found.
            self._move(record, 'NEEDS_ATTENTION', reason_code='CANCEL_UNCONFIRMED',
                       return_to=record.return_to)
            return record
        self._move(record, 'CANCELLED', reason_code='CANCELED_BY_REQUEST', return_to=None)
        return record

    # -- internals ---------------------------------------------------------

    def _require(self, order_id):
        try:
            return self.by_order[order_id]
        except KeyError:
            raise SchemaRefused('REFUSED_ENTITY_UNKNOWN',
                                f'no such order {order_id!r}', field='order_id') from None

    def _move(self, record, target, *, reason_code, return_to):
        previous = record.state
        record.state = target
        record.reason_code = reason_code
        record.return_to = return_to
        record.revision += 1
        if target in S.ORDER_FORWARD_PATH and target != previous:
            record.last_progress = target
        record.history.append({'revision': record.revision, 'from': previous, 'to': target,
                               'reason_code': reason_code, 'return_to': return_to})

    def _refuse(self, reason, detail):
        self.refusals.append((reason, detail))

    # -- reporting ---------------------------------------------------------

    def why_stuck(self, order_id):
        """Contract section 7's three questions, answered from the record alone.

        "记录最后有效进展、谁在等谁和首个拒绝原因" -- all three are here, and all three are
        derived from the history rather than stored separately. Two stored copies of one fact
        is how they disagree.
        """
        record = self._require(order_id)
        first_refusal = next(
            (h for h in record.history if h['reason_code'] is not None), None)
        return {
            'order_id': record.order_id,
            'state': record.state,
            'last_progress': record.last_progress,
            'return_to': record.return_to,
            'reason_code': record.reason_code,
            'first_refusal': first_refusal,
            'waiting_on': self._waiting_on(record),
            'revision': record.revision,
        }

    @staticmethod
    def _waiting_on(record):
        """Who the order is waiting for, derived from the stage rather than from a flag.

        A separate `waiting_on` field would be a second place to keep in step with `state`.
        """
        if record.state in ('QUEUED', 'PREPARING_TRAY', 'KITTING', 'VERIFYING_KIT'):
            return 'source_workcell'
        if record.state in ('WAITING_TRANSPORT', 'LOADING'):
            return 'transport'
        if record.state in ('IN_TRANSIT', 'UNLOADING', 'VERIFYING_DELIVERY'):
            return 'destination_workcell'
        if record.state in ('SUCCEEDED', 'CANCELLED', 'FAILED'):
            return None
        if record.state in ('WAITING', 'CANCELING', 'NEEDS_ATTENTION'):
            return f'operator:{record.reason_code}'
        return 'unknown'


__all__ = ['OrderRecord', 'OrderLedger', 'CONSTRAINED', 'DEAD_ENDS']
