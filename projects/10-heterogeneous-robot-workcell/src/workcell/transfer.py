"""TransferTransaction: the eight-stage handover, and the honest end of a failed one.

`docs/CONTRACTS.md` section 4 and 5:

    货物状态：AT_SOURCE、ON_WORKCELL、TRANSFERRING、ON_AMR、AT_DESTINATION、UNKNOWN。
    只有稳定交接确认才能改变唯一归属；跨设备期间用 TRANSFERRING 明确占两端资源。
    交接中断不得直接重新分配托盘；保持两端停止、保留事务、请求有证据恢复或人工处理。

    REQUESTED → BOTH_RESERVED → DOCK_VERIFIED → BOTH_READY
              → TRANSFERRING → RECEIVER_CONFIRMED → COMMITTED → RELEASED
    任一异常 → STOPPING → NEEDS_ATTENTION / ABORTED（仅证明未发生转移时）

    启动条件：双方同 transfer_id/epoch、空余容量、相对高度/横向/偏航/间隙合格、AMR停稳、
    源端有盘、接收端无盘、机械臂/人形退出、证据新鲜、停止链健康。任一 UNKNOWN 拒绝启动。

Three ideas shape this file, and each one is a rule that a naive implementation gets wrong in a
way that looks like success:

* **Custody is single-valued and it never goes `UNKNOWN` in a way that lets someone else take
  the tray.** While a transfer is in flight the tray is `TRANSFERRING`, which is a claim that
  *neither* end has it. That is deliberately not a claim of partial possession: the moment the
  system says "the receiver has 60% of a tray", the next stage of the plan will act on it. So
  the only custody changes this machine makes are `AT_SOURCE → TRANSFERRING → AT_DESTINATION`,
  and every one of them is gated on the stage that authorises it.

* **`ABORTED` means "I have proven no transfer happened"; `NEEDS_ATTENTION` means "I cannot
  prove either way".** Getting this backwards is how a system writes off a tray that is
  physically half way between two machines. The contract's parenthetical -- 仅证明未发生转移时
  -- is the whole rule, so this module has one function whose only job is to decide that, and
  it must be fed a proof rather than asked to assume one.

* **A launch precondition that is `UNKNOWN` refuses the start; it does not get defaulted.**
  Nine preconditions, and each is a separate reason so a report can say *which* one was missing.
  A precondition defaulted to "fine" is the same defect family as a resource defaulting to
  `FREE`: the first action happens before anyone looked.

The module is pure and clock-injected, like `ledger` and `resources`, so a test (or the
restart-reconciliation path) can replay a transaction from its record without wall time.
"""
from . import schema as S
from .resources import ResourceTable
from .schema import SchemaRefused

#: The eight happy-path stages, imported rather than retyped -- the contract owns the names.
STAGES = S.TRANSFER_STATES
#: The three ways a transfer can end badly.
FAILURE_STAGES = S.TRANSFER_FAILURE_STATES

#: Derived, never retyped: the forward edge for each stage is "the next stage in the contract's
#: own tuple", so inserting a stage in the contract cannot leave this map quietly wrong.
_NEXT = {a: b for a, b in zip(STAGES, STAGES[1:])}

#: The stage at which each custody change becomes legal. `None` means "this stage changes no
#: custody", which is the honest answer for the four setup stages: reserving a resource, seeing
#: a dock, and declaring both sides ready all happen *before* anything moves.
CUSTODY_ON_ENTRY = {
    'REQUESTED': None,
    'BOTH_RESERVED': None,
    'DOCK_VERIFIED': None,
    'BOTH_READY': None,
    'TRANSFERRING': 'TRANSFERRING',
    'RECEIVER_CONFIRMED': None,      # the receiver confirms arrival; custody moves at COMMITTED
    'COMMITTED': 'AT_DESTINATION',
    'RELEASED': None,
}

STOPPABLE = frozenset({'REQUIRED'})


def _verify(precond, field, *, low=None, high=None):
    """Read one precondition strictly: `None` is UNKNOWN, and UNKNOWN is not a pass.

    The contract says 任一 UNKNOWN 拒绝启动, so a missing key and an explicit `null` are the
    same failure -- and the caller learns which precondition by name, not just "rejected".

    Returns `(value, reason)` where `reason` is a refusal reason string or `None`. It does NOT
    raise: the ledger must *record* which condition killed a start, and a helper that raises
    directly would leave the refusal uncounted -- the same defect as a check nobody can audit.
    """
    if field not in precond:
        return None, 'REFUSED_MISSING_FIELD'
    value = precond[field]
    if value is None:
        return None, 'REFUSED_PRECONDITION_UNKNOWN'
    if isinstance(value, bool):
        return value, None
    if low is not None:
        return S._number_in(value, field, low, high), None
    return value, None


class TransferRecord:
    """One transfer, from request to whatever end it reached.

    The record is the durable thing: the contract says 中央重启保留事务与账本. So everything a
    restart needs in order to reconcile is *in here* -- the stage, the custody it has declared,
    which end is holding, and why it stopped if it stopped.
    """

    __slots__ = ('transfer_id', 'order_id', 'epoch', 'source', 'receiver', 'tray_id',
                 'stage', 'history', 'reason_code', 'return_to', 'stopped_confirmed',
                 'evidence', 'custody')

    def __init__(self, transfer_id, order_id, epoch, source, receiver, tray_id, *,
                 custody='AT_SOURCE'):
        self.transfer_id = transfer_id
        self.order_id = order_id
        self.epoch = epoch
        self.source = source
        self.receiver = receiver
        self.tray_id = tray_id
        self.stage = 'REQUESTED'
        self.custody = custody
        self.history = ['REQUESTED']
        self.reason_code = None
        self.return_to = None
        self.stopped_confirmed = None
        self.evidence = {}

    # -- queries -----------------------------------------------------------

    @property
    def is_done(self):
        return self.stage in ('RELEASED', 'ABORTED')

    @property
    def is_stuck(self):
        """Stuck means "a human must decide", not "it stopped".

        `STOPPING` is not stuck: the stop has been requested and the machine is waiting for the
        two confirmations that close it. Only a terminal bad end is stuck.
        """
        return self.stage in ('NEEDS_ATTENTION', 'ABORTED')

    @property
    def holds_both_ends(self):
        """True while no other transfer may take either end.

        This is the contract's 跨设备期间…明确占两端资源, and it deliberately includes
        `STOPPING`: 交接中断…保持两端停止 -- a stopped transfer must keep both ends, because the
        interruption is precisely when somebody else must *not* be allowed to pull up and take
        the tray. Excluding `STOPPING` would make the stop release the scene it is protecting.
        """
        return self.stage in ('BOTH_READY', 'TRANSFERRING', 'RECEIVER_CONFIRMED', 'STOPPING')

    def to_dict(self):
        return {
            'transfer_id': self.transfer_id,
            'order_id': self.order_id,
            'epoch': self.epoch,
            'source': self.source,
            'receiver': self.receiver,
            'tray_id': self.tray_id,
            'stage': self.stage,
            'custody': self.custody,
            'history': list(self.history),
            'reason_code': self.reason_code,
            'return_to': self.return_to,
            'stopped_confirmed': self.stopped_confirmed,
            # ★ `to_dict` is the durable form, so it carries only what `_restore` accepts.
            # The derived properties (`holds_both_ends`, `is_stuck`) are deliberately NOT here:
            # they are functions of `stage`/`custody`, and a serialiser that also wrote them
            # would create a second source of truth that a hand-edited log could contradict.
            'evidence': dict(sorted(self.evidence.items())),
        }

    def __repr__(self):  # pragma: no cover -- debugging aid
        return (f'<TransferRecord {self.transfer_id} {self.stage} custody={self.custody} '
                f'holds_both={self.holds_both_ends}>')


class TransferLedger:
    """All live transfers, keyed by id, plus the refusals that stopped a start.

    Refusals are recorded rather than only raised, because the contract's launch conditions are
    nine separate claims and a report has to be able to say how many starts died of *which* one.
    """

    def __init__(self, *, epoch):
        self.epoch = int(epoch)
        self.by_id = {}
        self.by_order = {}
        self.refusals = []
        self.started = 0
        self.committed = 0
        self.aborted = 0
        self.attention = 0

    # -- helpers -----------------------------------------------------------

    def _require(self, transfer_id):
        S._identifier(transfer_id, 'transfer_id')
        if transfer_id not in self.by_id:
            raise SchemaRefused('REFUSED_ENTITY_UNKNOWN',
                                f'no such transfer {transfer_id!r}', field='transfer_id')
        return self.by_id[transfer_id]

    def _refuse_start(self, reason, detail, *, field=None):
        """Record the refusal and raise it.

        `field` is carried so a caller can learn *which* launch condition failed by reading the
        exception, not by parsing the message. Both the recording and the raising happen here so
        the two can never drift: a refusal that is raised but not counted is invisible to a
        report, and that is the defect this signature is shaped to prevent.
        """
        self.refusals.append({'reason': reason, 'detail': detail, 'field': field})
        raise SchemaRefused(reason, detail, field=field)

    # -- launch ------------------------------------------------------------

    def request(self, raw, *, resources, now_s):
        """Open a transfer, or refuse it and say which launch condition failed.

        The nine conditions are checked in the order that fails fastest and most specifically,
        and each gets its own reason so "the transfer would not start" is never the whole answer.
        """
        payload = S._load(raw)
        S._mapping(payload, 'transfer request')
        S._exact_keys(payload, 'transfer request',
                      ('transfer_id', 'order_id', 'epoch', 'source', 'receiver', 'tray_id',
                       'preconditions'))
        transfer_id = S._identifier(payload['transfer_id'], 'transfer_id')
        if transfer_id in self.by_id:
            raise SchemaRefused('REFUSED_CONFLICT',
                                f'transfer {transfer_id!r} already exists', field='transfer_id')
        order_id = S._identifier(payload['order_id'], 'order_id')
        epoch = S._int_in(payload['epoch'], 'epoch', 0, 2 ** 62)
        source = S._identifier(payload['source'], 'source')
        receiver = S._identifier(payload['receiver'], 'receiver')
        tray_id = S._identifier(payload['tray_id'], 'tray_id')
        now_s = S._number_in(now_s, 'now_s', 0.0, S.MAX_DURATION_S)

        # -- 0. both ends must be distinct devices; a transfer to yourself is not a transfer
        if source == receiver:
            self._refuse_start('REFUSED_BAD_ID',
                               f'source and receiver are the same device {source!r}')
        # -- 1. same epoch as this coordinator
        if epoch != self.epoch:
            self._refuse_start('REFUSED_STALE_EPOCH',
                               f'transfer epoch {epoch} != coordinator epoch {self.epoch}')

        pre = payload['preconditions']
        if not isinstance(pre, dict) or isinstance(pre, bool):
            raise SchemaRefused('REFUSED_BAD_TYPE',
                                'preconditions must be a mapping', field='preconditions')

        # -- 2. the two resources must exist and be free; a transfer cannot share a dock
        if source not in resources.state:
            self._refuse_start('REFUSED_ENTITY_UNKNOWN', f'no resource {source!r} (source)')
        if receiver not in resources.state:
            self._refuse_start('REFUSED_ENTITY_UNKNOWN', f'no resource {receiver!r} (receiver)')

        # -- 3..9. the nine launch preconditions, each with its own field name.
        #    A missing key, an explicit null, and a false boolean are three different answers,
        #    and the first two are both "we do not know", which the contract refuses outright.
        S._mapping(pre, 'preconditions')
        BOOLEAN_CONDITIONS = (
            ('source_capacity_free', 'REFUSED_PRECONDITION_UNKNOWN'),
            ('receiver_capacity_free', 'REFUSED_PRECONDITION_UNKNOWN'),
            ('height_within_tolerance', 'DOCK_OUT_OF_TOLERANCE'),
            ('lateral_within_tolerance', 'DOCK_OUT_OF_TOLERANCE'),
            ('yaw_within_tolerance', 'DOCK_OUT_OF_TOLERANCE'),
            ('clearance_ok', 'DOCK_OUT_OF_TOLERANCE'),
            ('amr_at_rest', 'REFUSED_PRECONDITION_UNKNOWN'),
            ('source_has_tray', 'REFUSED_PRECONDITION_UNKNOWN'),
            ('receiver_empty', 'REFUSED_PRECONDITION_UNKNOWN'),
            ('arms_retracted', 'REFUSED_PRECONDITION_UNKNOWN'),
            ('stop_chain_healthy', 'REFUSED_PRECONDITION_UNKNOWN'),
        )
        for field, when_false in BOOLEAN_CONDITIONS:
            value, reason = _verify(pre, field)
            if reason is not None:
                self._refuse_start('REFUSED_PRECONDITION_UNKNOWN',
                                   f'launch precondition {field!r} is UNKNOWN ({reason})',
                                   field=field)
            if not value:
                if when_false == 'DOCK_OUT_OF_TOLERANCE':
                    self._refuse_start('DOCK_OUT_OF_TOLERANCE',
                                       f'{field} is outside the frozen tolerance', field=field)
                self._refuse_start('REFUSED_PRECONDITION_UNKNOWN',
                                   f'launch precondition {field!r} is not satisfied',
                                   field=field)

        #    evidence age, in seconds. Absent or null is again UNKNOWN, not zero: a missing
        #    timestamp defaulting to 0 would make the staleness check pass by omission.
        age_s, reason = _verify(pre, 'evidence_age_s', low=0.0, high=S.MAX_DURATION_S)
        if reason is not None:
            self._refuse_start('REFUSED_PRECONDITION_UNKNOWN',
                               f'evidence_age_s is UNKNOWN ({reason})')
        max_age_s, reason = _verify(pre, 'max_evidence_age_s', low=0.0, high=S.MAX_DURATION_S)
        if reason is not None:
            self._refuse_start('REFUSED_PRECONDITION_UNKNOWN',
                               f'max_evidence_age_s is UNKNOWN ({reason})')
        if age_s > max_age_s:
            self._refuse_start('STALE_OBSERVATION',
                               f'evidence is {age_s} s old, limit {max_age_s} s')

        # All nine hold. Reserve both ends atomically: if the second reservation fails, the
        # first must not be left standing, or a failed start would leak a held dock.
        owner = transfer_id
        ttl_s = S._number_in(pre.get('ttl_s'), 'ttl_s', 0.0, S.MAX_DURATION_S)
        reserved = []
        try:
            for resource in (source, receiver):
                resources.reserve(resource, owner=owner, ttl_s=ttl_s, now_s=now_s)
                reserved.append(resource)
        except SchemaRefused as refusal:
            for resource in reserved:
                resources.release(resource, owner=owner,
                                  generation=resources.snapshot(resource)['generation'],
                                  epoch=resources.epoch, now_s=now_s)
            self._refuse_start(refusal.reason, refusal.detail, field='resources')

        record = TransferRecord(transfer_id, order_id, epoch, source, receiver, tray_id)
        record.evidence['launch'] = {
            'evidence_age_s': age_s, 'max_evidence_age_s': max_age_s, 'ttl_s': ttl_s,
        }
        self.by_id[transfer_id] = record
        self.by_order[order_id] = transfer_id
        self.started += 1
        self._enter(record, 'BOTH_RESERVED')
        return record

    # -- progress ----------------------------------------------------------

    def _enter(self, record, stage):
        """Move one stage and apply the custody that stage authorises, if any."""
        if CUSTODY_ON_ENTRY.get(stage) is not None:
            new_custody = CUSTODY_ON_ENTRY[stage]
            if new_custody != record.custody:
                record.custody = new_custody
                record.evidence.setdefault('custody_changes', []).append(
                    {'at_stage': stage, 'to': new_custody})
        record.stage = stage
        record.history.append(stage)

    def advance(self, transfer_id, *, evidence=None):
        """Advance one stage along the contract's own path.

        There is no "jump to stage" argument, on purpose: a caller that can name the stage it
        wants can skip `DOCK_VERIFIED` and the machine would have no way to say no. The only
        legal move is the next one.
        """
        record = self._require(transfer_id)
        if record.stage in FAILURE_STAGES:
            raise SchemaRefused('REFUSED_CONFLICT',
                                f'transfer is {record.stage}; it must be resumed',
                                field='stage')
        if record.stage == 'RELEASED':
            raise SchemaRefused('REFUSED_CONFLICT', 'transfer is already RELEASED',
                                field='stage')
        target = _NEXT[record.stage]
        if target == 'RECEIVER_CONFIRMED':
            # The confirmation is a claim that something moved, so it needs evidence.
            if not evidence:
                raise SchemaRefused('REFUSED_MISSING_FIELD',
                                    'RECEIVER_CONFIRMED requires evidence', field='evidence')
            record.evidence['receiver_confirmed'] = list(evidence)
        elif evidence:
            record.evidence[target.lower()] = list(evidence)
        self._enter(record, target)
        if target == 'COMMITTED':
            self.committed += 1
        return record

    # -- the bad end -------------------------------------------------------

    def interrupt(self, transfer_id, *, reason_code, evidence=None):
        """An abnormality: stop both ends, keep the transaction, and do NOT reassign the tray.

        This is the contract's 交接中断不得直接重新分配托盘, made structural. The stage becomes
        `STOPPING` and the only way out is `resolve()`. Nothing here touches custody: at this
        point nobody knows where the tray is.
        """
        record = self._require(transfer_id)
        if record.stage in ('STOPPING',) or record.is_stuck:
            raise SchemaRefused('REFUSED_CONFLICT',
                                f'transfer is already {record.stage}', field='stage')
        if reason_code not in S.REASON_CODES:
            raise SchemaRefused('REFUSED_BAD_ENUM', repr(reason_code), field='reason_code')
        record.return_to = record.stage
        record.reason_code = reason_code
        record.evidence['interrupt'] = {'from': record.stage, 'reason_code': reason_code,
                                        'evidence': list(evidence or [])}
        self._enter(record, 'STOPPING')
        return record

    def resolve(self, transfer_id, *, non_transfer_proven, evidence=None):
        """Close a stopped transfer as either `ABORTED` or `NEEDS_ATTENTION`.

        ★ The contract's parenthetical is 仅证明未发生转移时 -- ABORTED only when it has been
        *proven* that no transfer happened. So this method does not decide that; it is given a
        proof and it refuses to accept a shrug. The two values are not degrees of the same
        thing, they are different epistemic states:

          * `ABORTED`      -- we know the tray is still at the source. Safe to retry.
          * `NEEDS_ATTENTION` -- we cannot say where the tray is. A human looks before anything
                                 else touches it.

        Getting this backwards writes off a tray that is physically between two machines, which
        is why the parameter is required and keyword-only, with no default.
        """
        record = self._require(transfer_id)
        if record.stage != 'STOPPING':
            raise SchemaRefused('REFUSED_CONFLICT',
                                f'transfer is {record.stage}, not STOPPING', field='stage')
        if non_transfer_proven is not True and non_transfer_proven is not False:
            raise SchemaRefused('REFUSED_BAD_TYPE',
                                'non_transfer_proven must be a bool', field='non_transfer_proven')
        record.evidence['resolution'] = {
            'non_transfer_proven': bool(non_transfer_proven),
            'evidence': list(evidence or []),
        }
        if non_transfer_proven:
            # Proven not to have moved, so custody is still the source's -- and the record says
            # so rather than leaving the reader to infer it from the stage.
            record.custody = 'AT_SOURCE'
            self._enter(record, 'ABORTED')
            self.aborted += 1
        else:
            self._enter(record, 'NEEDS_ATTENTION')
            self.attention += 1
        return record

    # -- restart -----------------------------------------------------------

    def reconcile(self, records, *, epoch, resources):
        """Rebuild live transfers after a coordinator restart, and report what cannot be trusted.

        The contract says 中央重启保留事务与账本，先对账再派发，不清空数据库冒充恢复. So this
        method does not "recover" anything by fiat: it re-loads the records, and for each one it
        states the fact that decides what may happen next --

          * a transfer from a previous epoch is flagged, because nothing it holds is still valid;
          * a transfer that was mid-flight when the process died is `NEEDS_ATTENTION`, never
            `ABORTED`: a crash is exactly the case where nobody proved the tray did not move;
          * a transfer already at a terminal stage is carried over untouched.

        Returns `(rebuilt_by_id, findings)` where a finding is a machine-readable statement, not
        prose, so a report can count them.
        """
        self.by_id = {}
        self.by_order = {}
        findings = []
        for raw in records:
            record = self._restore(raw)
            if record.transfer_id in self.by_id:
                raise SchemaRefused('REFUSED_CONFLICT',
                                    f'duplicate transfer {record.transfer_id!r} in the log',
                                    field='records')
            self.by_id[record.transfer_id] = record
            self.by_order[record.order_id] = record.transfer_id

            if record.epoch != epoch:
                findings.append({'transfer_id': record.transfer_id, 'kind': 'STALE_EPOCH',
                                 'detail': f'epoch {record.epoch} != {epoch}'})
            if record.stage in ('STOPPING', 'RELEASED', 'ABORTED', 'NEEDS_ATTENTION'):
                continue
            if record.holds_both_ends or record.stage == 'BOTH_RESERVED':
                record.return_to = record.stage
                record.reason_code = record.reason_code or 'TRANSFER_UNKNOWN'
                self._enter(record, 'NEEDS_ATTENTION')
                # ★ The summary must count what the reconcile produced, not only what
                # `resolve()` produced. A restart that parked a transfer in NEEDS_ATTENTION
                # while the summary reported zero is exactly how "the log looks clean" and
                # "a tray is missing" coexist.
                self.attention += 1
                findings.append({
                    'transfer_id': record.transfer_id, 'kind': 'INTERRUPTED_IN_FLIGHT',
                    'detail': f'was {record.return_to} at restart; cannot be ABORTED without a '
                              f'proof that the tray did not move',
                })
        return self.by_id, findings

    def _restore(self, raw):
        S._mapping(raw, 'transfer record')
        S._exact_keys(raw, 'transfer record',
                      ('transfer_id', 'order_id', 'epoch', 'source', 'receiver', 'tray_id',
                       'stage', 'custody', 'history', 'reason_code', 'return_to',
                       'stopped_confirmed', 'evidence'))
        stage = S._enum(raw['stage'], 'stage', frozenset(STAGES) | set(FAILURE_STAGES))
        custody = S._enum(raw['custody'], 'custody', S.CUSTODY_STATES)
        record = TransferRecord(
            S._identifier(raw['transfer_id'], 'transfer_id'),
            S._identifier(raw['order_id'], 'order_id'),
            S._int_in(raw['epoch'], 'epoch', 0, 2 ** 62),
            S._identifier(raw['source'], 'source'),
            S._identifier(raw['receiver'], 'receiver'),
            S._identifier(raw['tray_id'], 'tray_id'),
            custody=custody)
        record.stage = stage
        record.history = [S._enum(h, 'history[]', frozenset(STAGES) | set(FAILURE_STAGES))
                          for h in raw['history']]
        if not record.history or record.history[-1] != stage:
            raise SchemaRefused('REFUSED_CONFLICT',
                                f'history ends at {record.history[-1] if record.history else None!r}'
                                f' but stage is {stage!r}', field='history')
        record.reason_code = raw['reason_code']
        record.return_to = raw['return_to']
        record.stopped_confirmed = raw['stopped_confirmed']
        record.evidence = dict(raw['evidence'])
        return record

    # -- reporting ---------------------------------------------------------

    def summary(self):
        return {
            'epoch': self.epoch,
            'transfers': len(self.by_id),
            'started': self.started,
            'committed': self.committed,
            'aborted': self.aborted,
            'needs_attention': self.attention,
            'refusals': dict(sorted(
                {r: sum(1 for x in self.refusals if x['reason'] == r)
                 for r in {x['reason'] for x in self.refusals}}.items())),
        }


#: Reasons this module refuses with, declared so a report can count them apart from the generic
#: validator reasons it inherits by re-using `schema._identifier` / `schema._number_in`.
TRANSFER_REFUSALS = frozenset({
    'REFUSED_PRECONDITION_UNKNOWN', 'REFUSED_STALE_EPOCH', 'REFUSED_ENTITY_UNKNOWN',
    # A transfer to yourself is not a transfer, and this is the reason that says so.
    'REFUSED_BAD_ID',
    # The start is aborted when the second reservation fails, and the reason is the one the
    # resource table gave (held-by-other, unknown, ...), recorded under its own name.
    'REFUSED_RESOURCE_HELD_BY_OTHER', 'REFUSED_RESOURCE_UNKNOWN',
    'REFUSED_RESOURCE_OCCUPIED', 'REFUSED_RESOURCE_BLOCKED', 'REFUSED_RESOURCE_CLEARING',
    'REFUSED_NO_LIVE_GRANT', 'REFUSED_GRANT_EXPIRED',
})

__all__ = ['TransferLedger', 'TransferRecord', 'STAGES', 'FAILURE_STAGES', 'CUSTODY_ON_ENTRY',
           'TRANSFER_REFUSALS']
