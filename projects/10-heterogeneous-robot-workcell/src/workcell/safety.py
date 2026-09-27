"""Safety: the command gate, the watchdog, and the aggregation of a verdict.

`docs/CONTRACTS.md` section 7 and section 6's device-loss clause:

    分别输出 task_outcome、physical_result、safety_result、expected_behavior、evidence_complete。
    正常任务全部必需检查 PASS 且数据完整才能顶层 ACCEPTED；缺项 UNKNOWN/NOT_RUN 则不能 ACCEPTED。
    故障测试可以"正确停车"为预期通过，但不能计为配送完成。
    不要把所有失败写成 BUDGET_EXHAUSTED；记录最后有效进展、谁在等谁和首个拒绝原因。

    设备失联：命令门和驱动下游保护生效；持盘车不能自动让另一台接管货物。

Two ideas, and both are about *not* letting a convenient answer stand in for a real one:

* **The gate is the only thing that decides what the hardware is told, and silence is a
  stop.** A watchdog that fires turns the output to zero and says why. The failure this
  prevents is the one that reads as success in a log: the controller died, so no new command
  arrived, so the last command kept being honoured -- and every line in the log is consistent
  with a robot that was driving correctly.

* **A failed delivery that stopped correctly is a *safety pass* and a *task failure*, and both
  are reported.** 故障测试可以"正确停车"为预期通过，但不能计为配送完成 is the whole clause: if
  the two were collapsed into one field, either a stopped robot would count as a delivery, or a
  correct stop would count as a fault. So the aggregation here builds all five fields and
  refuses to let a caller reach `ACCEPTED` with a hedge in any of them.

The gate is shaped after `src/workcell_ipc/gate.py` (009's P1-N gate) and hardened in the two
places that template was weak: the reason vocabulary is closed and declared, and the watchdog
is an explicit transition with its own evidence rather than an arithmetic comparison that a
future caller can skip.
"""
from . import schema as S
from .schema import SchemaRefused

#: Every reason the gate itself can refuse a command with. Closed and declared, so a report can
#: count them and so a new refusal cannot appear that nothing can label.
GATE_REFUSALS = frozenset({
    'REFUSED_NOT_A_MAPPING', 'REFUSED_UNKNOWN_FIELD', 'REFUSED_MISSING_FIELD',
    'REFUSED_NON_FINITE', 'REFUSED_OUT_OF_RANGE', 'REFUSED_STALE_SEQ',
    'REFUSED_FROM_THE_FUTURE', 'REFUSED_ALREADY_EXPIRED', 'REFUSED_STALE_GENERATION',
    'REFUSED_STALE_EPOCH', 'REFUSED_WRONG_SOURCE',
})

#: The sources allowed to steer. A gate that accepts a command from any topic is a gate that
#: cannot answer "who told it to do that", which is the first question after any incident.
COMMAND_SOURCES = frozenset({'safety_gate', 'replay'})

#: What the actuator is being told, and why. The mode is the answer; the reason explains it.
GATE_MODES = frozenset({'HOLDING', 'SILENT', 'EXPIRED', 'STOPPED'})


class CommandGate:
    """The single writer of the actuator command, and the watchdog over it.

    Invariant, checked by `audit()`: at most one source may be accepted, every acceptance
    bumps the generation, and the output is `(0, 0)` in every mode that is not `HOLDING`.
    """

    def __init__(self, *, ttl_s, silence_s, v_max, w_max, source):
        if source not in COMMAND_SOURCES:
            raise SchemaRefused('REFUSED_BAD_ENUM', repr(source), field='source')
        self.ttl_s = S._number_in(ttl_s, 'ttl_s', 0.0, S.MAX_DURATION_S)
        self.silence_s = S._number_in(silence_s, 'silence_s', 0.0, S.MAX_DURATION_S)
        self.v_max = S._number_in(v_max, 'v_max', 0.0, 1.0e6)
        self.w_max = S._number_in(w_max, 'w_max', 0.0, 1.0e6)
        self.source = source
        self.epoch = None
        self.generation = 0
        self.active = None
        self.accepted_at_s = None
        self.highest_seq = -1
        self.stopped_confirmed = False
        self.counters = {'accepted': 0, 'refused': 0, 'watchdog_fired': 0}
        self.reasons = {}

    # -- helpers -----------------------------------------------------------

    def _refuse(self, reason):
        self.counters['refused'] += 1
        self.reasons[reason] = self.reasons.get(reason, 0) + 1
        return False, reason

    def bind_epoch(self, epoch):
        """Tell the gate which coordinator run it belongs to.

        Until this is called the gate has no epoch and accepts nothing: a command that predates
        the coordinator's own run is the one case where "the wheels moved and nobody knows why"
        is the true story, so the gate starts closed rather than open.
        """
        self.epoch = S._int_in(epoch, 'epoch', 0, 2 ** 62)
        return self.epoch

    # -- acceptance --------------------------------------------------------

    def offer(self, raw, *, now_s):
        """Try to accept one command. Returns `(accepted, reason_or_ACCEPTED)`.

        A refused command leaves the active one untouched: it neither extends nor cancels it.
        That is deliberate -- a malformed datagram must not be able to prolong a lease, because
        the whole point of the lease is that it expires on its own.
        """
        if self.epoch is None:
            return self._refuse('REFUSED_STALE_EPOCH')
        if not isinstance(raw, dict) or isinstance(raw, bool):
            return self._refuse('REFUSED_NOT_A_MAPPING')
        allowed = {'source', 'epoch', 'generation', 'seq', 'issued_s', 'ttl_s', 'v', 'w'}
        missing = allowed - set(raw)
        if missing:
            return self._refuse('REFUSED_MISSING_FIELD')
        extra = set(raw) - allowed
        if extra:
            return self._refuse('REFUSED_UNKNOWN_FIELD')
        if raw['source'] not in COMMAND_SOURCES:
            return self._refuse('REFUSED_WRONG_SOURCE')
        # ★ The source is fixed at construction, so a gate built for the safety layer cannot be
        # driven by the replay tool: "two writers disagreed" must be impossible, not handled.
        if raw['source'] != self.source:
            return self._refuse('REFUSED_WRONG_SOURCE')

        try:
            epoch = S._int_in(raw['epoch'], 'epoch', 0, 2 ** 62)
            generation = S._int_in(raw['generation'], 'generation', 0, 2 ** 62)
            seq = S._int_in(raw['seq'], 'seq', 0, 2 ** 62)
            issued_s = S._number_in(raw['issued_s'], 'issued_s', 0.0, S.MAX_DURATION_S)
            ttl_s = S._number_in(raw['ttl_s'], 'ttl_s', 0.0, S.MAX_DURATION_S)
            v = S._number_in(raw['v'], 'v', -self.v_max, self.v_max)
            w = S._number_in(raw['w'], 'w', -self.w_max, self.w_max)
        except SchemaRefused as exc:
            return self._refuse(exc.reason)

        if epoch != self.epoch:
            return self._refuse('REFUSED_STALE_EPOCH')
        # ★ The generation guard is what makes a replayed command harmless: an old generation is
        # not an old-but-valid command, it is a command from a world that no longer exists.
        if generation < self.generation:
            return self._refuse('REFUSED_STALE_GENERATION')
        if seq <= self.highest_seq:
            return self._refuse('REFUSED_STALE_SEQ')
        # A command dated in the future means the clocks disagree, and acting on it means acting
        # on a timestamp nothing else shares. Refused rather than clamped.
        if issued_s > now_s + 0.05:
            return self._refuse('REFUSED_FROM_THE_FUTURE')
        if now_s - issued_s > min(ttl_s, self.ttl_s):
            return self._refuse('REFUSED_ALREADY_EXPIRED')

        # Accepting a command re-arms the gate: a confirmed stop is history the moment the
        # operator asks for motion again, and it must be re-earned.
        self.stopped_confirmed = False
        self.highest_seq = seq
        self.generation = generation
        self.active = {'v': v, 'w': w, 'issued_s': issued_s, 'ttl_s': min(ttl_s, self.ttl_s),
                       'seq': seq, 'generation': generation}
        self.accepted_at_s = now_s
        self.counters['accepted'] += 1
        return True, 'ACCEPTED'

    # -- the stop chain ----------------------------------------------------

    def stop(self, *, reason_code, now_s):
        """A deliberate stop, with a contract reason code. Deterministic, not time-based."""
        if reason_code not in S.REASON_CODES:
            raise SchemaRefused('REFUSED_BAD_ENUM', repr(reason_code), field='reason_code')
        self.active = None
        self.accepted_at_s = None
        self.stopped_confirmed = False
        return {'mode': 'STOPPED', 'reason': reason_code, 'v': 0.0, 'w': 0.0}

    def confirm_stopped(self, *, stopped_confirmed):
        """Record that the hardware has *confirmed* the stop, so a cancellation can land.

        ★ Keyword-only with no default, for the same reason `ledger.confirm_canceled` is: "did
        it actually stop" is the question that decides whether a transfer may be declared
        cancelled, and a default would answer it by omission.
        """
        if stopped_confirmed is not True and stopped_confirmed is not False:
            raise SchemaRefused('REFUSED_BAD_TYPE', 'must be a bool',
                                field='stopped_confirmed')
        self.stopped_confirmed = bool(stopped_confirmed)
        return self.stopped_confirmed

    def step(self, *, now_s):
        """What the hardware is told this step: `(v, w, mode, reason)`.

        ★ The watchdog is here and nowhere else. Three independent ways to be silent, and the
        output is zero in all of them:

          * no command has ever been accepted,
          * nothing has been accepted for `silence_s` -- the ROS process died and the last
            command must not keep being honoured,
          * the active command is older than its own lease.
        """
        if self.stopped_confirmed and self.active is None:
            return 0.0, 0.0, 'STOPPED', 'STOP_CONFIRMED'
        if self.active is None:
            return 0.0, 0.0, 'SILENT', 'NO_COMMAND_EVER'
        if self.accepted_at_s is not None and now_s - self.accepted_at_s > self.silence_s:
            self.counters['watchdog_fired'] += 1
            return 0.0, 0.0, 'SILENT', 'SILENT_SINCE_LAST_COMMAND'
        age = now_s - self.active['issued_s']
        if age > self.active['ttl_s']:
            return 0.0, 0.0, 'EXPIRED', 'TTL_EXPIRED'
        return self.active['v'], self.active['w'], 'HOLDING', 'HELD'

    # -- the audit ---------------------------------------------------------

    def audit(self, *, now_s):
        """Re-derive the invariant from the gate's own state. Raises if it does not hold.

        ★ A gate is the one component whose output is a physical force, so "it looks right" is
        not the standard. This walks the actual state and refuses to return if any of three
        properties is false:

          * the mode is one the vocabulary declares;
          * the mode agrees with the gate's own stored state -- `HOLDING` and `EXPIRED` both
            require a live command to be about, and `STOPPED` requires that there is none;
          * the gate cannot be both stopped-confirmed and holding a live command.

        ★ The second property used to read "every mode that is not HOLDING outputs exactly
        zero", which `step` guarantees by construction, so the guard was unreachable and could
        not fail. See the comment at the guard itself.

        It is a method rather than a test so that the simulator can call it every step, which is
        the difference between an invariant and a claim about an invariant.
        """
        v, w, mode, _reason = self.step(now_s=now_s)
        if mode not in GATE_MODES:
            raise SchemaRefused('REFUSED_BAD_ENUM', f'gate mode {mode!r}', field='mode')
        # ★★ This guard replaces one that could never fire. The previous version was
        #
        #       if mode != 'HOLDING' and (v != 0.0 or w != 0.0):  raise ...
        #
        #   and it is dead: `step` returns a literal `0.0, 0.0` from every non-HOLDING branch,
        #   so the conjunction is unsatisfiable. A 168-state enumeration over (accepted,
        #   stopped_confirmed, clock, live command, stored ttl) produced **zero** states where
        #   the mode was non-holding and the output non-zero -- the same "a check that cannot be
        #   shown to fail is not a check" defect as the deleted `mark_cleared` branch.
        #
        #   What the guard was *trying* to say is checkable, and is checked below instead: a
        #   non-HOLDING mode must be consistent with the gate's own stored state. The pair
        #   (mode, active) is the real invariant -- a gate that says it is expiring while a live
        #   command sits in its hand, or that says it is stopped while one is still loaded, is
        #   the inconsistency a report would otherwise call healthy.
        self._check_mode_agrees_with_state(mode)
        return {'mode': mode, 'v': v, 'w': w}

    def _check_mode_agrees_with_state(self, mode):
        """The invariant, in one place, callable with a *stated* mode.

        ★ Separated out so a test can hand it a mode the gate would not currently compute.
        A guard that can only be reached through the code path that produces its input cannot be
        shown to fail -- which is exactly how the guard this replaced became dead code. Taking
        the mode as an argument is what makes the check demonstrable.
        """
        live = self.active is not None
        if mode == 'HOLDING' and not live:
            raise SchemaRefused('REFUSED_CONFLICT',
                                'mode HOLDING with no active command', field='mode')
        if mode == 'EXPIRED' and not live:
            raise SchemaRefused('REFUSED_CONFLICT',
                                'mode EXPIRED with no active command to expire', field='mode')
        if mode == 'STOPPED' and live:
            raise SchemaRefused('REFUSED_CONFLICT',
                                'mode STOPPED while still holding a command', field='mode')
        if self.stopped_confirmed and live:
            raise SchemaRefused('REFUSED_CONFLICT',
                                'the gate is stopped-confirmed and still holding a command',
                                field='stopped_confirmed')
        return mode

    def summary(self, *, now_s):
        v, w, mode, reason = self.step(now_s=now_s)
        return {
            'source': self.source,
            'epoch': self.epoch,
            'generation': self.generation,
            'mode': mode,
            'reason': reason,
            'v': v,
            'w': w,
            'highest_seq': self.highest_seq,
            'stopped_confirmed': self.stopped_confirmed,
            'accepted': self.counters['accepted'],
            'refused': self.counters['refused'],
            'watchdog_fired': self.counters['watchdog_fired'],
            'refusals': dict(sorted(self.reasons.items())),
        }


class EvaluationBuilder:
    """Assemble a verdict, and refuse the two ways of smuggling a pass into it.

    ★ 故障测试可以"正确停车"为预期通过，但不能计为配送完成. That sentence is why this class
    exists: the two facts ("it stopped correctly" and "it delivered") are separate fields, and
    the builder will not let one stand for the other. A fault scenario that stopped correctly is
    `safety_result=PASS` **and** `task_outcome=FAILED` (or `NEEDS_ATTENTION`), never `SUCCEEDED`.

    It also refuses the third smuggling route: a run with missing evidence quietly reported as
    `evidence_complete=True`, which would make "we did not look" indistinguishable from "fine".
    """

    def __init__(self, *, run_id):
        self.run_id = S._identifier(run_id, 'run_id', max_len=200)
        self.checks = {}
        self.missing = []
        self.reason_code = None
        self.last_progress = None
        self.waiting_on = None
        self.first_refusal = None

    def check(self, name, result):
        """Record one named check. `UNKNOWN` is a legal value and is *not* a pass."""
        if not isinstance(name, str) or not name:
            raise SchemaRefused('REFUSED_BAD_ID', repr(name), field='check')
        if result not in ('PASS', 'FAIL', 'UNKNOWN', 'NOT_APPLICABLE'):
            raise SchemaRefused('REFUSED_BAD_ENUM', repr(result), field=f'checks.{name}')
        self.checks[name] = result
        return self

    def miss(self, what):
        """Declare one piece of evidence that is absent. Declaring is what makes it countable."""
        S._text(what, 'missing[]', max_len=200)
        self.missing.append(what)
        return self

    def note(self, *, reason_code=None, last_progress=None, waiting_on=None,
             first_refusal=None):
        """The three things section 7 requires beyond the verdict, all optional and all recorded.

        ★ 记录最后有效进展、谁在等谁和首个拒绝原因. `first_refusal` is deliberately the FIRST and
        not the latest: the latest refusal is usually a downstream symptom, and the first one is
        the cause. Overwriting it would destroy the only trace of what actually went wrong.
        """
        if reason_code is not None:
            if reason_code not in S.REASON_CODES:
                raise SchemaRefused('REFUSED_BAD_ENUM', repr(reason_code), field='reason_code')
            self.reason_code = reason_code
        if last_progress is not None:
            S._text(last_progress, 'last_progress', max_len=200)
            self.last_progress = last_progress
        if waiting_on is not None:
            S._text(waiting_on, 'waiting_on', max_len=200)
            self.waiting_on = waiting_on
        if first_refusal is not None:
            S._text(first_refusal, 'first_refusal', max_len=200)
            # Kept as the first, never replaced by a later one.
            if self.first_refusal is None:
                self.first_refusal = first_refusal
        return self

    def _aggregate(self, results, *, worst='FAIL'):
        """`PASS` only if every required check is `PASS`; `UNKNOWN` if any is unknown."""
        values = [self.checks.get(name, 'UNKNOWN') for name in results]
        if any(v == 'UNKNOWN' for v in values):
            return 'UNKNOWN'
        return 'PASS' if all(v == 'PASS' for v in values) else worst

    def _aggregate_outcome(self, results, *, worst='FAILED', declared=None):
        """The task group's verdict, in `TASK_OUTCOMES` and not in the tri-state.

        ★ The tri-state (`PASS`/`FAIL`/`UNKNOWN`) answers "was this check satisfied". The task
        outcome answers "what happened to the order" -- and it is drawn from a different
        vocabulary (`SUCCEEDED`/`FAILED`/`NEEDS_ATTENTION`/`CANCELLED`). Reusing `_aggregate`
        here produced the literal string `'PASS'` in an outcome field, which the validator
        refused. That refusal is the right outcome: a check value is not an outcome, and the
        fact that the two look interchangeable is exactly why the mapping is written once, here,
        rather than inlined at the call site.

        The three-way mapping, in order:

          * any required task check `UNKNOWN` -> `NEEDS_ATTENTION`. We did not look, so we may
            not claim either the delivery or its failure; a human decides.
          * every required check `PASS` -> `SUCCEEDED`.
          * otherwise -> `worst` (`FAILED`), because a check that ran and did not pass is a
            delivery that did not happen.
        """
        values = [self.checks.get(name, 'UNKNOWN') for name in results]
        if any(v == 'UNKNOWN' for v in values):
            return 'NEEDS_ATTENTION'
        if all(v == 'PASS' for v in values):
            return 'SUCCEEDED'
        return worst

    def build(self, *, task_checks, physical_checks, safety_checks, expected_checks,
              task_outcome=None):
        """Produce the five fields, with the required-check sets supplied by the caller.

        `task_outcome` is only honoured when the task's own checks all passed: a caller cannot
        *declare* a delivery, because the delivery is the checks. Everything else is derived.
        """
        for group in (task_checks, physical_checks, safety_checks, expected_checks):
            for name in group:
                if name not in self.checks:
                    # ★ A required check nobody ran is UNKNOWN, not absent. Defaulting it to a
                    # missing key would let a report omit the one check that would have failed.
                    self.checks[name] = 'UNKNOWN'

        derived_outcome = self._aggregate_outcome(task_checks, worst='FAILED')
        if task_outcome is not None:
            if task_outcome not in S.TASK_OUTCOMES:
                raise SchemaRefused('REFUSED_BAD_ENUM', repr(task_outcome),
                                    field='task_outcome')
            # ★ The two must agree, and the comparison is outcome-to-outcome. A caller that
            # reports SUCCEEDED while a required task check is not PASS is claiming a delivery the
            # evidence does not support. The check is on the *derived outcome* rather than on the
            # raw check values, so it stays correct if the mapping above ever gains a third
            # outcome: this line cannot drift away from `_aggregate_outcome`.
            if task_outcome == 'SUCCEEDED' and derived_outcome != 'SUCCEEDED':
                raise SchemaRefused(
                    'REFUSED_CONFLICT',
                    f'SUCCEEDED declared but the task checks derive {derived_outcome}',
                    field='task_outcome')
            derived_outcome = task_outcome

        physical = self._aggregate(physical_checks, worst='FAIL')
        safety = self._aggregate(safety_checks, worst='FAIL')
        expected = self._aggregate(expected_checks, worst='FAIL')

        complete = not self.missing
        # ★ This dict is exactly `EVALUATION_KEYS` and nothing more. The validator is the
        # authority on what a verdict is, so the builder does not get to widen it -- an extra key
        # here would be refused, which is the correct outcome and is asserted by a test.
        raw = {
            'schema_version': S.SCHEMA_VERSION,
            'run_id': self.run_id,
            'task_outcome': derived_outcome,
            'physical_result': physical,
            'safety_result': safety,
            'expected_behavior': expected,
            'evidence_complete': complete,
            'missing': list(self.missing),
        }
        validated = S.validate_evaluation(raw)
        # `top_level_accepted` reads only the eight fields above, so it is called on the
        # validated dict before the annotations are attached.
        ok, blockers = S.top_level_accepted(validated)
        # The section-7 annotations and the derived verdict ride alongside the five fields. They
        # are attached after validation because they are *reporting*, not part of the verdict's
        # declared shape -- and `evidence_complete` is the field that says whether the verdict
        # itself is fully supported.
        validated['checks'] = dict(sorted(self.checks.items()))
        validated['reason_code'] = self.reason_code
        validated['last_progress'] = self.last_progress
        validated['waiting_on'] = self.waiting_on
        validated['first_refusal'] = self.first_refusal
        validated['accepted'] = ok
        validated['blockers'] = blockers
        return validated


def correctly_stopped_is_not_a_delivery(*, builder, task_checks, physical_checks,
                                        safety_checks, expected_checks, reason_code):
    """The fault-scenario shape: a delivery that failed but stopped correctly.

    Exists as a *function* rather than as guidance, because this is the case the contract singles
    out (故障测试可以"正确停车"为预期通过，但不能计为配送完成) and the one most likely to be got
    backwards. The result is a safety pass, an expected-behaviour pass, and a task failure --
    three fields disagreeing on purpose, and `ACCEPTED` is False precisely because they do.
    """
    builder.note(reason_code=reason_code)
    return builder.build(
        task_checks=task_checks,
        physical_checks=physical_checks,
        safety_checks=safety_checks,
        expected_checks=expected_checks,
        task_outcome='FAILED',
    )


__all__ = ['CommandGate', 'EvaluationBuilder', 'correctly_stopped_is_not_a_delivery',
           'GATE_REFUSALS', 'COMMAND_SOURCES', 'GATE_MODES']
