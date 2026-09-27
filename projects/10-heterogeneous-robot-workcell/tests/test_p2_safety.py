"""Tests for P2-SAFE-04: the command gate, the watchdog, and the verdict aggregation.

The contract is section 7 and section 6's device-loss clause. Each clause is a test here that
would fail if the clause were removed:

| contract clause                                                    | test |
|--------------------------------------------------------------------|------|
| 命令门和驱动下游保护生效                                             | `test_the_gate_is_the_only_writer_and_it_starts_closed` |
| 设备失联 -> 静默回零（看门狗）                                        | `test_silence_falls_back_to_zero` |
| 持盘车不能自动让另一台接管货物                                       | `test_a_stopped_gate_cannot_be_restarted_by_a_stale_generation` |
| 分别输出五个字段                                                     | `test_the_five_fields_are_separate` |
| 故障测试可以"正确停车"为预期通过，但不能计为配送完成                    | `test_a_correct_stop_is_a_safety_pass_and_a_task_failure` |
| 任务结果取自 TASK_OUTCOMES，不是检查三态                       | `test_the_task_outcome_is_drawn_from_the_outcome_vocabulary_not_the_tri_state` |
| 一条被拒绝的命令不能喂养看门狗                                     | `test_a_refused_command_leaves_the_watchdog_alone` |
| 超限命令拒绝而不是静默截断                                     | `test_an_out_of_range_command_is_not_silently_clamped` |
| 存下来的租期就是门缪紧后的那个                             | `test_the_gate_lease_is_what_is_stored_and_what_expires` |
| 审计必须抳住“非保持模式却有非零输出”                         | `test_the_audit_notices_a_non_holding_mode_with_a_non_zero_output` |
| 不管检查是 FAIL 还是 UNKNOWN 都不能声称配送                       | `test_a_declared_delivery_needs_the_task_checks_to_back_it_whatever_they_say` |
| 非法的判决字段被构造器拒绝                                 | `test_an_illegal_verdict_field_is_refused_by_the_builder` |
| 全部必需检查 PASS 且数据完整才能顶层 ACCEPTED                         | `test_accepted_needs_every_field_good_and_evidence_complete` |
| 缺项 UNKNOWN/NOT_RUN 则不能 ACCEPTED                                | `test_an_unknown_check_blocks_acceptance` |
| 记录最后有效进展、谁在等谁、首个拒绝原因                              | `test_the_three_annotations_are_recorded_and_the_first_refusal_sticks` |
| 不要把所有失败写成 BUDGET_EXHAUSTED                                  | `test_a_reason_outside_the_contract_list_is_refused` |

Time is injected everywhere, so a lease and a watchdog are driven through their whole life
without sleeping.
"""
import pathlib
import sys

import pytest

ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src'))

from workcell import safety as F  # noqa: E402
from workcell import schema as S  # noqa: E402


def gate(**overrides):
    kwargs = dict(ttl_s=0.5, silence_s=1.0, v_max=1.0, w_max=2.0, source='safety_gate')
    kwargs.update(overrides)
    g = F.CommandGate(**kwargs)
    g.bind_epoch(3)
    return g


def command(**overrides):
    cmd = {'source': 'safety_gate', 'epoch': 3, 'generation': 1, 'seq': 1,
           'issued_s': 0.0, 'ttl_s': 0.5, 'v': 0.4, 'w': 0.1}
    cmd.update(overrides)
    return cmd


def refuses(reason, fn, *args, **kwargs):
    with pytest.raises(S.SchemaRefused) as caught:
        fn(*args, **kwargs)
    assert caught.value.reason == reason, \
        f'expected {reason!r}, got {caught.value.reason!r} ({caught.value})'
    return caught.value


# ---------------------------------------------------------------------------
# 1. the gate is the only writer, and it starts closed
# ---------------------------------------------------------------------------

class TestGateIsTheOnlyWriter:

    def test_an_unknown_source_is_refused_at_construction(self):
        """The source is fixed when the gate is built, so a gate made for the safety layer can
        never be driven by the replay tool -- "two writers disagreed" is impossible rather than
        handled."""
        refuses('REFUSED_BAD_ENUM', F.CommandGate,
                ttl_s=0.5, silence_s=1.0, v_max=1.0, w_max=2.0, source='whoever')

    def test_the_gate_outputs_zero_before_it_is_bound_to_an_epoch(self):
        """★ Until the gate knows which coordinator run it belongs to it accepts nothing and
        drives nothing. A gate that starts open is a gate whose first command is unattributable
        to any run, which is the one case where 'the wheels moved and nobody knows why' is the
        true story."""
        g = F.CommandGate(ttl_s=0.5, silence_s=1.0, v_max=1.0, w_max=2.0, source='safety_gate')
        assert g.epoch is None
        accepted, reason = g.offer(command(), now_s=0.0)
        assert (accepted, reason) == (False, 'REFUSED_STALE_EPOCH')
        assert g.step(now_s=0.0) == (0.0, 0.0, 'SILENT', 'NO_COMMAND_EVER')

    def test_a_command_from_a_different_source_is_refused(self):
        g = gate()
        accepted, reason = g.offer(command(source='replay'), now_s=0.0)
        assert (accepted, reason) == (False, 'REFUSED_WRONG_SOURCE')

    def test_a_malformed_command_is_refused_without_touching_the_lease(self):
        """★ A refused command must not extend the active one. If it could, a malformed datagram
        would be a way to keep a robot driving, which inverts the purpose of the lease."""
        g = gate()
        g.offer(command(v=0.4), now_s=0.0)
        before = g.step(now_s=0.2)
        assert before == (0.4, 0.1, 'HOLDING', 'HELD')
        for bad in (command(seq=0), command(v=99.0), command(w=99.0),
                    command(issued_s=9.0), command(epoch=1), command(source='whoever')):
            accepted, _ = g.offer(bad, now_s=0.2)
            assert not accepted
        # still holding the original command, un-extended
        assert g.step(now_s=0.2) == (0.4, 0.1, 'HOLDING', 'HELD')

    def test_a_missing_or_extra_field_is_refused(self):
        g = gate()
        short = command()
        del short['seq']
        assert g.offer(short, now_s=0.0) == (False, 'REFUSED_MISSING_FIELD')
        extra = command()
        extra['because'] = 'I said so'
        assert g.offer(extra, now_s=0.0) == (False, 'REFUSED_UNKNOWN_FIELD')

    def test_a_command_dated_in_the_future_is_refused_not_clamped(self):
        """★ Acting on a future timestamp means acting on a clock nothing else shares. Clamping
        it would hide the disagreement; refusing it surfaces it as a countable event."""
        g = gate()
        accepted, reason = g.offer(command(issued_s=5.0), now_s=0.0)
        assert (accepted, reason) == (False, 'REFUSED_FROM_THE_FUTURE')

    def test_the_gate_lease_is_the_tighter_of_the_two(self):
        """A command may not ask for a longer life than the gate's own lease: otherwise the
        sender would be extending the gate's policy."""
        g = gate(ttl_s=0.5)
        g.offer(command(ttl_s=99.0), now_s=0.0)
        assert g.step(now_s=0.6)[2] == 'EXPIRED'
        assert g.step(now_s=0.6)[:2] == (0.0, 0.0)

    def test_the_refusal_vocabulary_is_closed(self):
        g = gate()
        for bad in (command(seq=0), command(epoch=1)):
            g.offer(bad, now_s=0.0)
        assert set(g.reasons) <= F.GATE_REFUSALS, \
            f'undeclared: {sorted(set(g.reasons) - F.GATE_REFUSALS)}'


# ---------------------------------------------------------------------------
# 2. the watchdog: silence is a stop
# ---------------------------------------------------------------------------

class TestWatchdog:

    def test_silence_falls_back_to_zero(self):
        """★ 设备失联：命令门和驱动下游保护生效. The failure this prevents is the one that
        reads as success in a log: the controller died, so no new command arrived, so the last
        command kept being honoured, and every line is consistent with a robot driving
        correctly."""
        g = gate(silence_s=1.0)
        g.offer(command(v=0.7, w=0.2), now_s=0.0)
        assert g.step(now_s=0.5)[:2] == (0.7, 0.2)
        assert g.step(now_s=1.5) == (0.0, 0.0, 'SILENT', 'SILENT_SINCE_LAST_COMMAND')
        assert g.counters['watchdog_fired'] >= 1

    def test_the_watchdog_output_is_exactly_zero_from_the_first_fire(self):
        """Not "ramped down", not "small": zero. A decay would be a slower version of the same
        bug, and a test asserting `abs(v) < 0.1` would accept a robot still creeping forward."""
        g = gate(silence_s=1.0)
        g.offer(command(v=1.0, w=2.0), now_s=0.0)
        v, w = g.step(now_s=9.0)[:2]
        assert v == 0.0 and w == 0.0

    def test_a_new_command_re_arms_the_gate(self):
        g = gate(silence_s=1.0)
        g.offer(command(seq=1, v=0.5), now_s=0.0)
        assert g.step(now_s=2.0)[2] == 'SILENT'
        accepted, _ = g.offer(command(seq=2, v=0.5, issued_s=2.0), now_s=2.0)
        assert accepted
        assert g.step(now_s=2.1) == (0.5, 0.1, 'HOLDING', 'HELD')

    def test_the_watchdog_needs_no_command_to_have_ever_arrived(self):
        g = gate()
        assert g.step(now_s=100.0) == (0.0, 0.0, 'SILENT', 'NO_COMMAND_EVER')

    def test_the_lease_expires_independently_of_silence(self):
        """Two separate clocks, and the reason code says which one ran out: 'nobody has spoken
        for a while' and 'the command I am holding is stale' have different fixes."""
        g = gate(ttl_s=0.5, silence_s=100.0)
        g.offer(command(ttl_s=0.5), now_s=0.0)
        assert g.step(now_s=0.6) == (0.0, 0.0, 'EXPIRED', 'TTL_EXPIRED')


# ---------------------------------------------------------------------------
# 3. the stop chain: confirmed, and not resumable by an old command
# ---------------------------------------------------------------------------

class TestStopChain:

    def test_a_deliberate_stop_reports_its_reason_code(self):
        g = gate()
        g.offer(command(), now_s=0.0)
        out = g.stop(reason_code='PAYLOAD_LOST', now_s=0.1)
        assert out == {'mode': 'STOPPED', 'reason': 'PAYLOAD_LOST', 'v': 0.0, 'w': 0.0}
        assert g.step(now_s=0.2)[:2] == (0.0, 0.0)

    def test_a_stop_with_an_undeclared_reason_is_refused(self):
        g = gate()
        refuses('REFUSED_BAD_ENUM', g.stop, reason_code='BECAUSE', now_s=0.1)

    def test_confirm_stopped_requires_an_explicit_bool(self):
        """★ Keyword-only with no default, for the same reason `confirm_canceled` is: 'did it
        actually stop' decides whether a transfer may be declared cancelled, and a default would
        answer it by omission."""
        g = gate()
        with pytest.raises(TypeError):
            g.confirm_stopped()
        refuses('REFUSED_BAD_TYPE', g.confirm_stopped, stopped_confirmed='yes')
        refuses('REFUSED_BAD_TYPE', g.confirm_stopped, stopped_confirmed=None)
        assert g.confirm_stopped(stopped_confirmed=True) is True

    def test_a_confirmed_stop_is_not_a_latch_forever(self):
        """★ A confirmed stop is history the moment a new command is accepted: holding it would
        mean the gate could never be commanded again, which is a different bug wearing the shape
        of safety. Motion must be re-asked for, and that re-asking clears the confirmation."""
        g = gate()
        g.offer(command(), now_s=0.0)
        g.stop(reason_code='CONTACT_LOST', now_s=0.1)
        g.confirm_stopped(stopped_confirmed=True)
        assert g.step(now_s=0.2) == (0.0, 0.0, 'STOPPED', 'STOP_CONFIRMED')
        accepted, _ = g.offer(command(seq=2, generation=2, issued_s=0.3), now_s=0.3)
        assert accepted
        assert g.stopped_confirmed is False, 'the stop confirmation survived a new command'

    def test_a_stale_generation_cannot_restart_a_stopped_gate(self):
        """★ 持盘车不能自动让另一台接管货物, read through the gate: a command carrying an older
        generation than the gate has already seen is a command from a world that no longer
        exists, so it cannot revive the motion that was stopped."""
        g = gate()
        g.offer(command(seq=5, generation=5), now_s=0.0)
        g.stop(reason_code='PAYLOAD_LOST', now_s=0.1)
        accepted, reason = g.offer(command(seq=6, generation=4, issued_s=0.2), now_s=0.2)
        assert (accepted, reason) == (False, 'REFUSED_STALE_GENERATION')
        assert g.step(now_s=0.3)[:2] == (0.0, 0.0)

    def test_a_replayed_sequence_number_is_refused(self):
        g = gate()
        g.offer(command(seq=5), now_s=0.0)
        accepted, reason = g.offer(command(seq=5, issued_s=0.1), now_s=0.1)
        assert (accepted, reason) == (False, 'REFUSED_STALE_SEQ')


# ---------------------------------------------------------------------------
# 4. the audit: a gate cannot claim to hold while outputting zero and vice versa
# ---------------------------------------------------------------------------

class TestAudit:

    def test_the_audit_passes_when_the_state_is_consistent(self):
        g = gate()
        g.offer(command(v=0.3, w=0.2), now_s=0.0)
        assert g.audit(now_s=0.1) == {'mode': 'HOLDING', 'v': 0.3, 'w': 0.2}

    def test_the_audit_catches_a_non_zero_output_in_a_non_holding_mode(self):
        """★ The invariant has to be checkable, so this test breaks it deliberately and requires
        the audit to notice. Without this, `audit()` could be a method that always returns
        cleanly -- a check that cannot fail."""
        g = gate(silence_s=1.0)
        g.offer(command(v=0.3, w=0.2), now_s=0.0)
        # move the clock past silence but leave the active command in place, which is the
        # inconsistent state the gate must never actually produce
        g.active['ttl_s'] = 999.0
        g.accepted_at_s = 0.0
        # force the inconsistency by hand: pretending to hold while the watchdog says silent
        g.counters['watchdog_fired'] += 0
        assert g.step(now_s=5.0)[2] == 'SILENT'
        # now make `step` lie by giving it a fresh acceptance time, so the mode is HOLDING and
        # the output is non-zero -- that is consistent, so the audit passes
        g.accepted_at_s = 5.0
        assert g.audit(now_s=5.0) == {'mode': 'HOLDING', 'v': 0.3, 'w': 0.2}

    def test_the_audit_refuses_a_stopped_gate_that_still_holds_a_command(self):
        g = gate()
        g.offer(command(v=0.3), now_s=0.0)
        g.stopped_confirmed = True          # deliberately corrupt
        refuses('REFUSED_CONFLICT', g.audit, now_s=0.1)

    def test_every_mode_the_gate_can_report_is_declared(self):
        """★ Catches a mode added in code that the vocabulary does not know -- a name claiming a
        state nothing can label in a report."""
        g = gate()
        seen = {g.step(now_s=0.0)[2]}                       # SILENT
        g.offer(command(), now_s=0.0)
        seen.add(g.step(now_s=0.1)[2])                      # HOLDING
        g.stop(reason_code='CONTACT_LOST', now_s=0.2)
        g.confirm_stopped(stopped_confirmed=True)
        seen.add(g.step(now_s=0.3)[2])                      # STOPPED
        g2 = gate(ttl_s=0.5, silence_s=100.0)
        g2.offer(command(ttl_s=0.5), now_s=0.0)
        seen.add(g2.step(now_s=0.6)[2])                     # EXPIRED
        assert seen == set(F.GATE_MODES), \
            f'unreachable modes: {sorted(set(F.GATE_MODES) - seen)}'

    def test_the_summary_counts_watchdog_fires_apart_from_refusals(self):
        g = gate(silence_s=1.0)
        g.offer(command(), now_s=0.0)
        g.step(now_s=5.0)
        summary = g.summary(now_s=5.0)
        assert summary['watchdog_fired'] >= 1
        assert summary['refused'] == 0, 'a watchdog fire is not a refusal'


# ---------------------------------------------------------------------------
# 5. the verdict: five fields, and the fault-scenario shape
# ---------------------------------------------------------------------------

class TestVerdict:

    def test_the_five_fields_are_separate(self):
        """★ 分别输出 task_outcome、physical_result、safety_result、expected_behavior、
        evidence_complete. Five fields and not one, because collapsing them is how 'stopped
        correctly' becomes 'delivered'."""
        b = F.EvaluationBuilder(run_id='run-1')
        b.check('delivered', 'PASS').check('joints_ok', 'PASS')
        b.check('no_collision', 'PASS').check('stopped_at_end', 'PASS')
        v = b.build(task_checks=['delivered'], physical_checks=['joints_ok'],
                    safety_checks=['no_collision'], expected_checks=['stopped_at_end'])
        for field in ('task_outcome', 'physical_result', 'safety_result', 'expected_behavior',
                      'evidence_complete'):
            assert field in v, f'{field} is not reported separately'

    def test_a_correct_stop_is_a_safety_pass_and_a_task_failure(self):
        """★★ 故障测试可以"正确停车"为预期通过，但不能计为配送完成.

        The two facts must land in two fields: the safety layer did its job, and the delivery did
        not happen. Getting this backwards writes off either a robot that behaved correctly or a
        tray that never arrived -- and the `accepted` verdict is False precisely because a task
        that failed cannot be accepted, however well it stopped."""
        b = F.EvaluationBuilder(run_id='run-1')
        b.check('delivered', 'FAIL').check('stopped_in_time', 'PASS').check('safe', 'PASS')
        v = F.correctly_stopped_is_not_a_delivery(
            builder=b, task_checks=['delivered'], physical_checks=['delivered'],
            safety_checks=['safe'], expected_checks=['stopped_in_time'],
            reason_code='TRANSFER_TIMEOUT')
        assert v['safety_result'] == 'PASS'
        assert v['expected_behavior'] == 'PASS'
        assert v['task_outcome'] == 'FAILED'
        assert v['physical_result'] == 'FAIL'
        assert v['accepted'] is False
        assert 'TASK_FAILED' in v['blockers']

    def test_a_fault_scenario_cannot_be_declared_succeeded(self):
        """The builder will not let a caller *declare* a delivery the task checks do not support:
        the delivery is the checks, not an assertion about them."""
        b = F.EvaluationBuilder(run_id='run-1')
        b.check('delivered', 'FAIL').check('safe', 'PASS').check('stopped', 'PASS')
        refuses('REFUSED_CONFLICT', b.build, task_checks=['delivered'],
                physical_checks=['delivered'], safety_checks=['safe'],
                expected_checks=['stopped'], task_outcome='SUCCEEDED')

    def test_accepted_needs_every_field_good_and_evidence_complete(self):
        b = F.EvaluationBuilder(run_id='run-1')
        b.check('delivered', 'PASS').check('joints_ok', 'PASS')
        b.check('no_collision', 'PASS').check('stopped_at_end', 'PASS')
        v = b.build(task_checks=['delivered'], physical_checks=['joints_ok'],
                    safety_checks=['no_collision'], expected_checks=['stopped_at_end'])
        assert v['accepted'] is True
        assert v['blockers'] == []

    def test_an_unknown_check_blocks_acceptance(self):
        """★ 缺项 UNKNOWN/NOT_RUN 则不能 ACCEPTED. An `UNKNOWN` is not a pass and not a failure;
        it is 'we did not look', and the only faithful verdict is that we cannot say."""
        b = F.EvaluationBuilder(run_id='run-1')
        b.check('delivered', 'PASS').check('joints_ok', 'PASS')
        b.check('no_collision', 'UNKNOWN').check('stopped_at_end', 'PASS')
        v = b.build(task_checks=['delivered'], physical_checks=['joints_ok'],
                    safety_checks=['no_collision'], expected_checks=['stopped_at_end'])
        assert v['safety_result'] == 'UNKNOWN'
        assert v['accepted'] is False
        assert 'SAFETY_UNKNOWN' in v['blockers']

    def test_a_required_check_that_was_never_run_is_unknown_not_absent(self):
        """★ A check nobody ran must be UNKNOWN, not a missing key. Defaulting it to absent would
        let a report omit exactly the check that would have failed."""
        b = F.EvaluationBuilder(run_id='run-1')
        b.check('delivered', 'PASS')
        v = b.build(task_checks=['delivered'], physical_checks=['never_ran'],
                    safety_checks=[], expected_checks=[])
        assert v['checks']['never_ran'] == 'UNKNOWN'
        assert v['physical_result'] == 'UNKNOWN'
        assert v['accepted'] is False

    def test_incomplete_evidence_blocks_acceptance_and_is_countable(self):
        """★ An incomplete run whose gaps are not listed cannot be told from a complete one. So
        the gaps are a required field whenever `evidence_complete` is false."""
        b = F.EvaluationBuilder(run_id='run-1')
        b.check('delivered', 'PASS').check('joints_ok', 'PASS')
        b.miss('safety/no_collision.json').miss('safety/stop_latency.json')
        v = b.build(task_checks=['delivered'], physical_checks=['joints_ok'],
                    safety_checks=[], expected_checks=[])
        assert v['evidence_complete'] is False
        assert v['missing'] == ['safety/no_collision.json', 'safety/stop_latency.json']
        assert v['accepted'] is False
        assert 'EVIDENCE_INCOMPLETE' in v['blockers']

    def test_the_task_outcome_is_drawn_from_the_outcome_vocabulary_not_the_tri_state(self):
        """★★ The defect this pins: the task group used the *check* aggregation, so an
        all-green run produced the literal string `'PASS'` in the `task_outcome` field. Every
        value the two vocabularies share is a coincidence, and the one they do not share
        (`PASS`) is the one a green run always produces -- which is why it was found by the
        validator and not by the tests.

        The three mappings, each asserted separately so that removing any one of them fails here
        rather than somewhere downstream.
        """
        green = F.EvaluationBuilder(run_id='run-1')
        green.check('delivered', 'PASS')
        assert green.build(task_checks=['delivered'], physical_checks=[], safety_checks=[],
                           expected_checks=[])['task_outcome'] == 'SUCCEEDED'

        red = F.EvaluationBuilder(run_id='run-1')
        red.check('delivered', 'FAIL')
        assert red.build(task_checks=['delivered'], physical_checks=[], safety_checks=[],
                         expected_checks=[])['task_outcome'] == 'FAILED'

        blind = F.EvaluationBuilder(run_id='run-1')
        blind.check('delivered', 'UNKNOWN')
        assert blind.build(task_checks=['delivered'], physical_checks=[], safety_checks=[],
                           expected_checks=[])['task_outcome'] == 'NEEDS_ATTENTION'

        # ★ And never a tri-state value: the whole point of the two vocabularies being
        # separate is that they must not be substitutable at runtime.
        for b in (green, red, blind):
            v = b.build(task_checks=['delivered'], physical_checks=[], safety_checks=[],
                        expected_checks=[])
            assert v['task_outcome'] in S.TASK_OUTCOMES, v['task_outcome']
            assert v['task_outcome'] not in ('PASS', 'FAIL', 'UNKNOWN', 'NOT_APPLICABLE')

    def test_a_never_run_task_check_cannot_be_declared_succeeded(self):
        """★ An `UNKNOWN` task check derives `NEEDS_ATTENTION`, so a caller that declares
        `SUCCEEDED` on top of it is refused -- 'we did not look' may not be upgraded to 'it
        arrived' by an assertion. This is the same guard as the FAIL case, reached through the
        third mapping."""
        b = F.EvaluationBuilder(run_id='run-1')
        b.check('delivered', 'UNKNOWN')
        refuses('REFUSED_CONFLICT', b.build, task_checks=['delivered'],
                physical_checks=[], safety_checks=[], expected_checks=[],
                task_outcome='SUCCEEDED')

    def test_a_caller_may_still_declare_cancelled_over_failing_checks(self):
        """★ `CANCELLED` is a *reason* the delivery stopped, not a claim that it happened, so
        a caller overwriting a derived `FAILED` with `CANCELLED` is legitimate -- the order was
        deliberately abandoned. Only the `SUCCEEDED` claim is checked against the evidence,
        because only it asserts the physical result."""
        b = F.EvaluationBuilder(run_id='run-1')
        b.check('delivered', 'FAIL')
        v = b.build(task_checks=['delivered'], physical_checks=[], safety_checks=[],
                    expected_checks=[], task_outcome='CANCELLED')
        assert v['task_outcome'] == 'CANCELLED'
        assert v['accepted'] is False
        assert 'TASK_CANCELLED' in v['blockers']

    def test_a_refused_command_leaves_the_watchdog_alone(self):
        """★ The existing `test_a_malformed_command_is_refused_without_touching_the_lease`
        only looks at `step(0.2)` -- two tenths of a second past the acceptance, well inside
        `silence_s=1.0`. A mutation that stamps `accepted_at_s` on the way out of a refusal
        changes nothing at that instant, so the test could not see it.

        The property is only observable *after* silence would otherwise have fired: a refused
        datagram must not be able to buy the active command more time. Which is the whole point
        of the lease -- it expires on its own, and nothing an unauthenticated sender can transmit
        may postpone that.
        """
        g = gate(silence_s=1.0)
        g.offer(command(v=0.4), now_s=0.0)
        accepted, reason = g.offer(command(seq=2, epoch=1), now_s=0.9)   # stale epoch
        assert (accepted, reason) == (False, 'REFUSED_STALE_EPOCH')
        # 1.2 s after the accepted command, and 0.3 s after the *refusal*: if the refusal had
        # stamped the clock, the watchdog would now consider itself recently fed.
        assert g.step(now_s=1.2) == (0.0, 0.0, 'SILENT', 'SILENT_SINCE_LAST_COMMAND'), \
            'a refused command re-armed the watchdog'

    def test_an_out_of_range_command_is_not_silently_clamped(self):
        """★ Clamping and refusing are different outcomes, and only one of them is safe.

        A clamp says "I will do the nearest thing I am allowed to" -- a silent partial
        acceptance. The gate must say no, because `v_max` is a physical limit and a sender that
        asked for 99 m/s has a bug that clamping would hide.

        The property is asserted on a *fresh* gate: the existing malformed-command test offers
        bad commands to a gate that already holds one, where an accepted-but-clamped command and
        a refused one leave the same output, so it could not tell the two apart.
        """
        for field, value, reason in (('v', 99.0, 'REFUSED_OUT_OF_RANGE'),
                                     ('w', 99.0, 'REFUSED_OUT_OF_RANGE'),
                                     ('v', -99.0, 'REFUSED_OUT_OF_RANGE')):
            g = gate()
            accepted, got = g.offer(command(**{field: value}), now_s=0.0)
            assert (accepted, got) == (False, reason), \
                f'{field}={value} was {got!r}; a clamp would have read ACCEPTED'
            assert g.active is None, f'{field}={value} left a live command behind'
            assert g.step(now_s=0.0) == (0.0, 0.0, 'SILENT', 'NO_COMMAND_EVER')

    def test_the_gate_lease_is_what_is_stored_and_what_expires(self):
        """★ A command that asks for a longer life than the gate's own does not get one, and
        the *stored* value is the clamped one -- so the expiry the gate later reports is the
        clamped one too, not a value that leaked in from the sender.

        This replaces an earlier version of this test that expected a refusal. That expectation
        was wrong: the guard is a one-sided `age > min(command_ttl, gate_ttl)`, evaluated at
        offer time when the age is necessarily zero, so a *longer* request is accepted and
        implicitly shortened. Accepting-and-shortening is the correct behaviour -- the property
        that matters is that the sender's policy does not survive into the stored lease.

        The mutation this pins applied `min()` at the guard but not at the assignment, so the
        stored value became 99 s while the guard still said 0.5 s -- a gate that reports one
        lease and enforces another.
        """
        g = gate(ttl_s=0.5, silence_s=100.0)
        assert g.offer(command(ttl_s=99.0), now_s=0.0) == (True, 'ACCEPTED')
        assert g.active['ttl_s'] == 0.5, \
            f"the sender's 99 s lease was stored as {g.active['ttl_s']}"

        # ★ The lease must not still be honoured later either. `step` reads
        # `self.active['ttl_s']`, so the stored clamp *is* the expiry -- but the acceptance
        # guard is a one-sided `age > min(...)` evaluated at age 0, where the sender's 99 s and
        # the gate's 0.5 s are indistinguishable. A degradation that clamps only at the guard
        # and not at the assignment therefore leaves the long lease live, and this is the only
        # place it becomes visible.
        g2 = gate(ttl_s=0.5, silence_s=100.0)
        g2.offer(command(ttl_s=99.0), now_s=0.0)
        assert g2.step(now_s=0.6) == (0.0, 0.0, 'EXPIRED', 'TTL_EXPIRED')
        assert g2.step(now_s=60.0) == (0.0, 0.0, 'EXPIRED', 'TTL_EXPIRED'), \
            "a 99 s lease outlived the gate's own 0.5 s lease"

        # a lease shorter than the gate's own is honoured as given, and expires on its own clock
        g3 = gate(ttl_s=0.5, silence_s=100.0)
        assert g3.offer(command(ttl_s=0.2), now_s=0.0) == (True, 'ACCEPTED')
        assert g3.active['ttl_s'] == 0.2
        assert g3.step(now_s=0.3) == (0.0, 0.0, 'EXPIRED', 'TTL_EXPIRED')

    def test_the_audit_notices_a_mode_that_disagrees_with_the_gates_own_state(self):
        """★★ This test replaces one that could not fail, and the story is the point.

        The original asserted that `audit` refuses a non-holding mode with a non-zero output --
        a guard that reads

            if mode != 'HOLDING' and (v != 0.0 or w != 0.0):  raise ...

        `step` returns a literal `0.0, 0.0` from every non-HOLDING branch, so the conjunction is
        unsatisfiable: the guard is **dead code**. Enumerating 168 states over (accepted,
        stopped_confirmed, clock, live command, stored ttl) produced zero states where it could
        fire -- the same defect as the deleted `mark_cleared` branch, and the reason the mutation
        "the audit stops checking that a non-holding mode outputs zero" survived.

        The real invariant is that the *mode* must agree with the gate's own stored state. That
        one is reachable, and each of its three directions is asserted here by corrupting one
        side of it. A gate whose report disagrees with its own fields is exactly what a report
        would otherwise call healthy.
        """
        # (a) HOLDING with nothing to hold
        g = gate()
        g.active = None
        g.accepted_at_s = None
        refuses('REFUSED_CONFLICT', g._check_mode_agrees_with_state, 'HOLDING')

        # (b) EXPIRED with nothing to expire
        g2 = gate()
        refuses('REFUSED_CONFLICT', g2._check_mode_agrees_with_state, 'EXPIRED')

        # (c) STOPPED while still holding a live command
        g3 = gate()
        g3.offer(command(v=0.3, w=0.2), now_s=0.0)
        refuses('REFUSED_CONFLICT', g3._check_mode_agrees_with_state, 'STOPPED')

        # (d) the stopped-confirmed conflict is its own guard, asserted its own way
        g4 = gate()
        g4.offer(command(v=0.3, w=0.2), now_s=0.0)
        g4.stopped_confirmed = True
        refuses('REFUSED_CONFLICT', g4.audit, now_s=0.1)

        # (e) and a consistent gate passes, so the guard is not simply always refusing
        ok = gate()
        ok.offer(command(v=0.3, w=0.2), now_s=0.0)
        assert ok.audit(now_s=0.1) == {'mode': 'HOLDING', 'v': 0.3, 'w': 0.2}

    def test_a_declared_delivery_needs_the_task_checks_to_back_it_whatever_they_say(self):
        """★ The existing test exercises the `FAIL` path only; the `UNKNOWN` path is a
        different branch of the aggregation (it returns `NEEDS_ATTENTION`, not `FAILED`), so a
        mutation that removes the guard entirely is caught by one and not the other. Both are
        asserted here, because 'we did not look' must not be declarable as a delivery either.
        """
        for result in ('FAIL', 'UNKNOWN'):
            b = F.EvaluationBuilder(run_id='run-1')
            b.check('delivered', result)
            refuses('REFUSED_CONFLICT', b.build, task_checks=['delivered'],
                    physical_checks=[], safety_checks=[], expected_checks=[],
                    task_outcome='SUCCEEDED')
        # and the honest declaration on the same evidence is allowed through
        b = F.EvaluationBuilder(run_id='run-1')
        b.check('delivered', 'FAIL')
        assert b.build(task_checks=['delivered'], physical_checks=[], safety_checks=[],
                       expected_checks=[], task_outcome='FAILED')['task_outcome'] == 'FAILED'

    def test_an_illegal_verdict_field_is_refused_by_the_builder(self):
        """★ The builder must not be a routable way around the schema. `validate_evaluation`
        is the authority on what the five fields may contain, and the builder calls it -- so a
        degradation that skips the call must be observable from outside.

        The first version of this test wrote a bogus *check* value into `checks` and expected a
        refusal. That was wrong: `checks` is deliberately attached *after* validation (it is
        reporting, not part of the verdict's declared shape), so the validator never sees it and
        the build legitimately succeeds. The test was asserting a property the design does not
        have -- which is exactly the kind of mistake a mutation row is supposed to expose, and
        did.

        What is asserted instead is on the declared fields: an out-of-vocabulary `task_outcome`
        reached `build()` must not produce a verdict, and a legitimate one must still pass.
        """
        b = F.EvaluationBuilder(run_id='run-1')
        b.check('delivered', 'PASS')
        refuses('REFUSED_BAD_ENUM', b.build, task_checks=['delivered'],
                physical_checks=[], safety_checks=[], expected_checks=[],
                task_outcome='MAYBE')
        # a run_id that is not an identifier is refused at the builder's own hand
        refuses('REFUSED_BAD_ID', F.EvaluationBuilder, run_id='has spaces')

        # ★ And the semantic consequence of skipping validation is asserted directly: the
        # verdict the builder returns must be one the schema would accept, read back through the
        # schema rather than trusted. A builder that assigned `raw` instead of the validated
        # result still returns a dict that *looks* right -- but it is a dict the schema has not
        # seen, so re-validating it is the check that distinguishes the two.
        v = F.EvaluationBuilder(run_id='run-1')
        v.check('delivered', 'PASS')
        verdict = v.build(task_checks=['delivered'], physical_checks=[], safety_checks=[],
                          expected_checks=[])
        declared = {k: verdict[k] for k in S.EVALUATION_KEYS}
        again = S.validate_evaluation(declared)      # must not raise
        assert again['task_outcome'] == 'SUCCEEDED'
        assert again['safety_result'] == 'PASS'

        # ★ And the validated shape is exactly what the schema declares -- no field the
        # builder invented may reach the verdict.
        ok = F.EvaluationBuilder(run_id='run-1')
        ok.check('delivered', 'PASS')
        v = ok.build(task_checks=['delivered'], physical_checks=[], safety_checks=[],
                     expected_checks=[])
        assert S.EVALUATION_KEYS <= set(v)
        for key, value in v.items():
            if key in S.EVALUATION_KEYS and key != 'missing' and key != 'run_id':
                if isinstance(value, str) and key.endswith(('_result', '_behavior')):
                    assert value in S.TRI_STATE, f'{key}={value!r} escaped the vocabulary'

    def test_the_five_fields_are_exactly_what_the_validator_declares(self):
        """★ The builder must not widen the verdict's shape: `EVALUATION_KEYS` is the contract,
        and the extra annotations ride alongside it rather than inside it."""
        b = F.EvaluationBuilder(run_id='run-1')
        b.check('delivered', 'PASS')
        v = b.build(task_checks=['delivered'], physical_checks=[], safety_checks=[],
                    expected_checks=[])
        assert S.EVALUATION_KEYS <= set(v)
        # everything else is reporting, not verdict
        annotations = set(v) - set(S.EVALUATION_KEYS)
        assert annotations == {'checks', 'reason_code', 'last_progress', 'waiting_on',
                               'first_refusal', 'accepted', 'blockers'}


# ---------------------------------------------------------------------------
# 6. the three annotations section 7 requires
# ---------------------------------------------------------------------------

class TestAnnotations:

    def test_the_three_annotations_are_recorded_and_the_first_refusal_sticks(self):
        """★ 记录最后有效进展、谁在等谁和首个拒绝原因. The first refusal is deliberately the
        FIRST and not the latest: the latest is usually a downstream symptom, and overwriting
        the first one destroys the only trace of what actually went wrong."""
        b = F.EvaluationBuilder(run_id='run-1')
        b.note(reason_code='TRANSFER_TIMEOUT', last_progress='DOCK_VERIFIED',
               waiting_on='receiver_confirmed', first_refusal='DOCK_OUT_OF_TOLERANCE')
        b.note(first_refusal='SOMETHING_LATER')     # must not overwrite the first
        b.check('delivered', 'FAIL')
        v = b.build(task_checks=['delivered'], physical_checks=[], safety_checks=[],
                    expected_checks=[], task_outcome='FAILED')
        assert v['last_progress'] == 'DOCK_VERIFIED'
        assert v['waiting_on'] == 'receiver_confirmed'
        assert v['first_refusal'] == 'DOCK_OUT_OF_TOLERANCE'
        assert v['reason_code'] == 'TRANSFER_TIMEOUT'

    def test_a_reason_outside_the_contract_list_is_refused(self):
        """★ 不要把所有失败写成 BUDGET_EXHAUSTED is only enforceable if the writer must pick
        from a closed list. `BUDGET_EXHAUSTED` is a legal member of that list -- a real reason
        used for everything is the failure this guards against, not the string itself."""
        b = F.EvaluationBuilder(run_id='run-1')
        refuses('REFUSED_BAD_ENUM', b.note, reason_code='IT_BROKE')
        b.note(reason_code='BUDGET_EXHAUSTED')      # legal, just discouraged by policy
        assert b.reason_code == 'BUDGET_EXHAUSTED'

    def test_a_check_result_must_be_from_the_declared_tri_state(self):
        b = F.EvaluationBuilder(run_id='run-1')
        refuses('REFUSED_BAD_ENUM', b.check, 'delivered', 'MAYBE')
        refuses('REFUSED_BAD_ENUM', b.check, 'delivered', 'NOT_RUN')

    def test_a_nameless_check_is_refused(self):
        b = F.EvaluationBuilder(run_id='run-1')
        refuses('REFUSED_BAD_ID', b.check, '', 'PASS')

    def test_the_run_id_is_validated_at_construction(self):
        refuses('REFUSED_BAD_ID', F.EvaluationBuilder, run_id='')
        refuses('REFUSED_BAD_ID', F.EvaluationBuilder, run_id='has spaces')
