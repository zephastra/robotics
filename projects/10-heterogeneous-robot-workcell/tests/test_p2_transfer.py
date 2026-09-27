"""Tests for P2-XFER-03: the transfer transaction, its launch gate, and its honest bad end.

The contract is section 4 and section 5 of `docs/CONTRACTS.md`. Each clause it states is a test
here that would fail if the clause were removed:

| contract clause                                          | test |
|----------------------------------------------------------|------|
| 八阶段路径 REQUESTED→…→RELEASED                            | `test_the_eight_stages_are_walked_in_order` |
| 只有稳定交接确认才能改变唯一归属                            | `test_custody_moves_only_at_the_two_stages_that_prove_it` |
| 跨设备期间用 TRANSFERRING 明确占两端资源                     | `test_the_middle_stages_hold_both_ends` |
| 交接中断不得直接重新分配托盘                                | `test_an_interrupt_touches_no_custody` |
| 任一异常 → STOPPING → NEEDS_ATTENTION/ABORTED（仅证明未发生转移时）| `test_aborted_requires_a_proof_and_needs_attention_does_not` |
| 启动条件：任一 UNKNOWN 拒绝启动                            | `test_every_launch_precondition_must_be_known` |
| 中央重启保留事务与账本，先对账再派发                        | `test_reconcile_never_invents_an_aborted_transfer` |

Time is injected throughout, so a transaction is driven end to end without sleeping.
"""
import pathlib
import sys

import pytest

ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src'))

from workcell import resources as R  # noqa: E402
from workcell import schema as S  # noqa: E402
from workcell import transfer as T  # noqa: E402

DOCK = 'dock_a'
DECK = 'deck_b'

#: The nine launch preconditions the contract lists, all satisfied. Each launch-refusal test
#: below removes exactly one, so no test can pass by breaking something else.
GOOD_PRECONDITIONS = {
    'source_capacity_free': True,
    'receiver_capacity_free': True,
    'height_within_tolerance': True,
    'lateral_within_tolerance': True,
    'yaw_within_tolerance': True,
    'clearance_ok': True,
    'amr_at_rest': True,
    'source_has_tray': True,
    'receiver_empty': True,
    'arms_retracted': True,
    'stop_chain_healthy': True,
    'evidence_age_s': 0.4,
    'max_evidence_age_s': 2.0,
    'ttl_s': 60.0,
}


def table(epoch=7, *, cleared=True):
    t = R.ResourceTable(epoch=epoch, resources=[DOCK, DECK])
    if cleared:
        t.mark_cleared(DOCK, evidence=['init/clear_dock.json'], now_s=0.0)
        t.mark_cleared(DECK, evidence=['init/clear_deck.json'], now_s=0.0)
    return t


def request_payload(**overrides):
    payload = {
        'transfer_id': 'xfer_1',
        'order_id': 'order_1',
        'epoch': 7,
        'source': DOCK,
        'receiver': DECK,
        'tray_id': 'tray_v1_01',
        'preconditions': dict(GOOD_PRECONDITIONS),
    }
    payload.update(overrides)
    return payload


def live(epoch=7, *, now_s=1.0):
    """One transfer walked to `BOTH_RESERVED`."""
    t = table(epoch=epoch)
    led = T.TransferLedger(epoch=epoch)
    rec = led.request(request_payload(epoch=epoch), resources=t, now_s=now_s)
    return t, led, rec


def refuses(reason, fn, *args, **kwargs):
    with pytest.raises(S.SchemaRefused) as caught:
        fn(*args, **kwargs)
    assert caught.value.reason == reason, \
        f'expected {reason!r}, got {caught.value.reason!r} ({caught.value})'
    return caught.value


# ---------------------------------------------------------------------------
# 1. the eight-stage path
# ---------------------------------------------------------------------------

class TestHappyPath:

    def test_the_stage_list_is_the_contracts_own(self):
        """Imported, not retyped: inserting a stage in the contract cannot leave this module
        quietly walking a shorter path."""
        assert T.STAGES is S.TRANSFER_STATES

    def test_the_forward_edges_are_derived_from_the_stage_list(self):
        """★ The edge map must be the contract's list zipped with itself, so it cannot drift.
        A hand-written map is a second copy of the path, and the two copies disagree silently."""
        assert T._NEXT == {a: b for a, b in zip(S.TRANSFER_STATES,
                                                S.TRANSFER_STATES[1:])}
        for stage in S.TRANSFER_STATES[:-1]:
            assert stage in T._NEXT, f'no forward edge from {stage}'

    def test_the_eight_stages_are_walked_in_order(self):
        t, led, rec = live()
        # `live()` returns at BOTH_RESERVED, so REQUESTED is already behind us. Take the walk
        # from the record's own history rather than re-deriving it, so the first stage is
        # included and the assertion covers all eight.
        walked = list(rec.history)
        assert walked == list(S.TRANSFER_STATES[:2])
        while rec.stage != 'RELEASED':
            rec = led.advance('xfer_1', evidence=['obs/p.json'])
            walked.append(rec.stage)
        assert walked == list(S.TRANSFER_STATES)
        assert rec.history == list(S.TRANSFER_STATES)

    def test_a_transfer_cannot_skip_a_stage(self):
        """★ There is no "jump to stage" argument, on purpose: a caller that can name the stage
        it wants could skip DOCK_VERIFIED and the machine would have no way to say no. So the
        only steerable thing is "next", and this asserts that one call moves exactly one stage."""
        t, led, rec = live()
        before = rec.stage
        rec = led.advance('xfer_1')
        assert rec.stage == T._NEXT[before]
        assert rec.history == ['REQUESTED', 'BOTH_RESERVED', 'DOCK_VERIFIED']

    def test_the_receiver_confirmation_needs_evidence(self):
        """★ 只有稳定交接确认才能改变唯一归属 -- a confirmation is a claim that something
        moved, so it must carry the observation that says so. Without evidence it is a default
        pretending to be a measurement."""
        t, led, rec = live()
        for _ in range(3):
            rec = led.advance('xfer_1')
        assert rec.stage == 'TRANSFERRING'
        refuses('REFUSED_MISSING_FIELD', led.advance, 'xfer_1')
        refuses('REFUSED_MISSING_FIELD', led.advance, 'xfer_1', evidence=[])
        rec = led.advance('xfer_1', evidence=['obs/receiver_photo.json'])
        assert rec.stage == 'RECEIVER_CONFIRMED'
        assert rec.evidence['receiver_confirmed'] == ['obs/receiver_photo.json']

    def test_advancing_a_released_transfer_is_refused(self):
        t, led, rec = live()
        while rec.stage != 'RELEASED':
            rec = led.advance('xfer_1', evidence=['o.json'])
        refuses('REFUSED_CONFLICT', led.advance, 'xfer_1')

    def test_an_unknown_transfer_is_refused(self):
        _, led, _ = live()
        refuses('REFUSED_ENTITY_UNKNOWN', led.advance, 'xfer_nope')


# ---------------------------------------------------------------------------
# 2. custody: single-valued, and only moved by proof
# ---------------------------------------------------------------------------

class TestCustody:

    def test_custody_moves_only_at_the_two_stages_that_prove_it(self):
        """★ 只有稳定交接确认才能改变唯一归属, made structural. Four setup stages prove nothing
        about where the tray is; two stages do. The custody values in between are the whole
        point: `TRANSFERRING` claims *neither* end has it, which is why no later plan step can
        act as if one end does."""
        t, led, rec = live()
        # `live()` returns at BOTH_RESERVED, so both of the first two stages are already behind
        # us and both left custody at the source. Walk the rest and record one custody per
        # stage, so the assertion is over exactly eight entries for eight stages.
        assert rec.stage == 'BOTH_RESERVED' and rec.custody == 'AT_SOURCE'
        trace = [('REQUESTED', 'AT_SOURCE'), ('BOTH_RESERVED', 'AT_SOURCE')]
        while rec.stage != 'RELEASED':
            rec = led.advance('xfer_1', evidence=['o.json'])
            trace.append((rec.stage, rec.custody))
        assert trace == [
            ('REQUESTED', 'AT_SOURCE'),
            ('BOTH_RESERVED', 'AT_SOURCE'),
            ('DOCK_VERIFIED', 'AT_SOURCE'),
            ('BOTH_READY', 'AT_SOURCE'),
            ('TRANSFERRING', 'TRANSFERRING'),
            ('RECEIVER_CONFIRMED', 'TRANSFERRING'),
            ('COMMITTED', 'AT_DESTINATION'),
            ('RELEASED', 'AT_DESTINATION'),
        ]
        # the two changes are attributable to exactly two stages
        changes = [(c['at_stage'], c['to']) for c in rec.evidence['custody_changes']]
        assert changes == [('TRANSFERRING', 'TRANSFERRING'), ('COMMITTED', 'AT_DESTINATION')]

    def test_the_middle_stages_hold_both_ends(self):
        """★ 跨设备期间用 TRANSFERRING 明确占两端资源 -- the window in which one tray is
        legitimately claimed by two devices at once is exactly three stages wide."""
        t, led, rec = live()
        holds = [('BOTH_RESERVED', rec.holds_both_ends)]
        while rec.stage != 'RELEASED':
            rec = led.advance('xfer_1', evidence=['o.json'])
            holds.append((rec.stage, rec.holds_both_ends))
        assert holds == [('BOTH_RESERVED', False), ('DOCK_VERIFIED', False),
                         ('BOTH_READY', True), ('TRANSFERRING', True),
                         ('RECEIVER_CONFIRMED', True), ('COMMITTED', False),
                         ('RELEASED', False)]

    def test_a_transfer_never_declares_custody_unknown(self):
        """The custody a transfer declares is drawn from a known set, and `UNKNOWN` is not one it
        may land on as a side effect: 'we do not know where the tray is' must be a deliberate
        verdict from `resolve()`, not something that happens on the way through."""
        t, led, rec = live()
        while rec.stage != 'RELEASED':
            rec = led.advance('xfer_1', evidence=['o.json'])
            assert rec.custody in ('AT_SOURCE', 'TRANSFERRING', 'AT_DESTINATION')
        assert rec.custody in S.CUSTODY_STATES


# ---------------------------------------------------------------------------
# 3. the launch gate: nine conditions, and UNKNOWN is not a pass
# ---------------------------------------------------------------------------

class TestLaunchConditions:

    def test_a_complete_request_starts_and_reserves_both_ends(self):
        t, led, rec = live()
        assert rec.stage == 'BOTH_RESERVED'
        assert t.snapshot(DOCK)['state'] == 'RESERVED'
        assert t.snapshot(DECK)['state'] == 'RESERVED'
        # both ends are held by the transfer, so no other owner can take either
        refuses('REFUSED_RESOURCE_HELD_BY_OTHER', t.reserve,
                DOCK, owner='amr_9', ttl_s=5.0, now_s=2.0)

    def test_every_launch_precondition_must_be_known(self):
        """★ 任一 UNKNOWN 拒绝启动. Each of the nine is tested by removing it, so a single
        forgotten check cannot hide behind the others -- and the refusal names the field, so a
        report can say which condition was missing rather than "the transfer did not start"."""
        numeric = {'evidence_age_s', 'max_evidence_age_s', 'ttl_s'}
        for field in GOOD_PRECONDITIONS:
            if field in numeric:
                continue
            payload = request_payload()
            payload['preconditions'] = dict(GOOD_PRECONDITIONS)
            payload['preconditions'][field] = None
            t = table()
            led = T.TransferLedger(epoch=7)
            e = refuses('REFUSED_PRECONDITION_UNKNOWN', led.request, payload,
                        resources=t, now_s=1.0)
            assert e.field == field, f'wrong field reported for {field}'
            # ★ The missing key is refused with the SAME reason as the explicit null, and that
            # is deliberate: "the key is absent" and "the key says unknown" are the same
            # epistemic state -- we do not know -- and the contract refuses both. Reporting them
            # as two different reasons would invite a caller to treat one as recoverable.
            payload['preconditions'] = {k: v for k, v in GOOD_PRECONDITIONS.items()
                                        if k != field}
            e2 = refuses('REFUSED_PRECONDITION_UNKNOWN', led.request, payload,
                         resources=t, now_s=1.0)
            assert e2.field == field, f'missing key reported the wrong field for {field}'

    def test_a_false_non_geometry_precondition_is_refused(self):
        """★ The complement of `test_a_single_geometry_violation_is_refused_with_its_own_reason`.

        Found by mutation: killing `if not value:` left the suite green except for the geometry
        test, so nothing was checking that a *non-geometry* condition reported `False` is
        refused. Every one of the eleven booleans is exercised, because a single guard covers
        them all and a single test would have let ten of them rot unnoticed."""
        geometric = {'height_within_tolerance', 'lateral_within_tolerance',
                     'yaw_within_tolerance', 'clearance_ok'}
        for field in GOOD_PRECONDITIONS:
            if field in geometric or not isinstance(GOOD_PRECONDITIONS[field], bool):
                continue
            payload = request_payload()
            payload['preconditions'] = dict(GOOD_PRECONDITIONS)
            payload['preconditions'][field] = False
            t = table()
            led = T.TransferLedger(epoch=7)
            e = refuses('REFUSED_PRECONDITION_UNKNOWN', led.request, payload,
                        resources=t, now_s=1.0)
            assert e.field == field, f'wrong field reported for {field}'
            assert led.summary()['started'] == 0
            # and nothing was reserved on the way to the refusal
            assert t.snapshot(DOCK)['state'] == 'FREE'
            assert t.snapshot(DECK)['state'] == 'FREE'

    def test_all_the_boolean_preconditions_are_counted(self):
        """A guard against the previous test silently shrinking: the contract lists nine
        boolean launch conditions, so this fails if one is dropped from the table."""
        booleans = {k for k, v in GOOD_PRECONDITIONS.items() if isinstance(v, bool)}
        assert booleans == {
            'source_capacity_free', 'receiver_capacity_free',
            'height_within_tolerance', 'lateral_within_tolerance', 'yaw_within_tolerance',
            'clearance_ok', 'amr_at_rest', 'source_has_tray', 'receiver_empty',
            'arms_retracted', 'stop_chain_healthy',
        }, 'the launch-condition table no longer matches its test'

    def test_a_single_geometry_violation_is_refused_with_its_own_reason(self):
        for field in ('height_within_tolerance', 'lateral_within_tolerance',
                      'yaw_within_tolerance', 'clearance_ok'):
            payload = request_payload()
            payload['preconditions'] = dict(GOOD_PRECONDITIONS)
            payload['preconditions'][field] = False
            t = table()
            led = T.TransferLedger(epoch=7)
            refuses('DOCK_OUT_OF_TOLERANCE', led.request, payload, resources=t, now_s=1.0)

    def test_stale_evidence_is_refused(self):
        payload = request_payload()
        payload['preconditions'] = dict(GOOD_PRECONDITIONS)
        payload['preconditions']['evidence_age_s'] = 3.0
        t = table()
        led = T.TransferLedger(epoch=7)
        refuses('STALE_OBSERVATION', led.request, payload, resources=t, now_s=1.0)

    def test_an_uncleared_resource_cannot_be_the_source(self):
        """A resource nobody has looked at is not a resource a transfer may start on -- the
        same default-UNKNOWN rule as `resources`, reached through this door."""
        t = table(cleared=False)
        led = T.TransferLedger(epoch=7)
        refuses('REFUSED_RESOURCE_UNKNOWN', led.request, request_payload(),
                resources=t, now_s=1.0)

    def test_a_stale_epoch_is_refused(self):
        t = table()
        led = T.TransferLedger(epoch=9)
        refuses('REFUSED_STALE_EPOCH', led.request, request_payload(epoch=7),
                resources=t, now_s=1.0)

    def test_a_transfer_to_the_same_device_is_refused(self):
        t = table()
        led = T.TransferLedger(epoch=7)
        refuses('REFUSED_BAD_ID', led.request,
                request_payload(source=DOCK, receiver=DOCK), resources=t, now_s=1.0)

    def test_a_reused_transfer_id_is_a_conflict(self):
        t, led, _ = live()
        refuses('REFUSED_CONFLICT', led.request, request_payload(), resources=t, now_s=2.0)

    def test_a_failed_start_leaks_no_reservation(self):
        """★ The atomicity that matters at this layer: both ends are reserved, so if the second
        reservation cannot happen the first must not be left standing. A leaked dock is a
        transfer that failed *and* quietly took a resource with it."""
        t = table()
        # someone else already holds the receiver, so the second reservation must fail
        t.reserve(DECK, owner='amr_9', ttl_s=100.0, now_s=0.5)
        led = T.TransferLedger(epoch=7)
        refuses('REFUSED_RESOURCE_HELD_BY_OTHER', led.request, request_payload(),
                resources=t, now_s=1.0)
        assert t.snapshot(DOCK)['state'] == 'FREE', 'the failed start leaked the source'
        assert t.snapshot(DOCK)['owner'] is None
        assert t.snapshot(DECK)['owner'] == 'amr_9'

    def test_the_refusal_history_is_countable(self):
        t = table()
        led = T.TransferLedger(epoch=7)
        bad = request_payload()
        bad['preconditions'] = dict(GOOD_PRECONDITIONS)
        bad['preconditions']['amr_at_rest'] = None
        for _ in range(3):
            refuses('REFUSED_PRECONDITION_UNKNOWN', led.request, bad,
                    resources=t, now_s=1.0)
        assert led.summary()['refusals'] == {'REFUSED_PRECONDITION_UNKNOWN': 3}
        assert led.summary()['started'] == 0


# ---------------------------------------------------------------------------
# 4. the bad end: STOPPING, then a verdict that must be earned
# ---------------------------------------------------------------------------

class TestInterruption:

    def test_an_interrupt_touches_no_custody(self):
        """★ 交接中断不得直接重新分配托盘. At the moment of an abnormality nobody knows where
        the tray is, so the interrupt must not move custody in either direction -- not to the
        receiver (optimistic) and not back to the source (a write-off)."""
        t, led, rec = live()
        rec = led.advance('xfer_1')   # DOCK_VERIFIED
        rec = led.advance('xfer_1')   # BOTH_READY
        rec = led.advance('xfer_1')   # TRANSFERRING
        assert rec.stage == 'TRANSFERRING' and rec.custody == 'TRANSFERRING'
        rec = led.interrupt('xfer_1', reason_code='CONTACT_LOST',
                            evidence=['obs/contact_lost.json'])
        assert rec.stage == 'STOPPING'
        assert rec.custody == 'TRANSFERRING', 'the interrupt reassigned the tray'
        assert rec.return_to == 'TRANSFERRING'
        assert rec.reason_code == 'CONTACT_LOST'
        assert rec.holds_both_ends, 'the stop must keep both ends reserved'

    def test_aborted_requires_a_proof_and_needs_attention_does_not(self):
        """★★ 仅证明未发生转移时 -- this is the clause that decides whether a tray gets
        retried or gets a human. The two outcomes are not degrees of confidence: `ABORTED` is a
        proof the tray never moved, `NEEDS_ATTENTION` is the absence of one. So the same stopped
        transfer, resolved two ways, must land in two different places."""
        # proven not to have moved -> ABORTED, and custody is back at the source *because it was
        # proven*, not because a stop happened
        _, led_a, rec_a = live()
        rec_a = led_a.advance('xfer_1')   # DOCK_VERIFIED
        rec_a = led_a.advance('xfer_1')   # BOTH_READY
        rec_a = led_a.advance('xfer_1')   # TRANSFERRING -- identical to the other arm
        assert rec_a.custody == 'TRANSFERRING'
        led_a.interrupt('xfer_1', reason_code='TRANSFER_TIMEOUT')
        rec_a = led_a.resolve('xfer_1', non_transfer_proven=True,
                              evidence=['obs/source_still_full.json'])
        assert rec_a.stage == 'ABORTED'
        assert rec_a.custody == 'AT_SOURCE'
        assert rec_a.is_done

        # not proven -> NEEDS_ATTENTION, and custody stays where the interrupt left it
        _, led_b, rec_b = live()
        rec_b = led_b.advance('xfer_1')   # DOCK_VERIFIED
        rec_b = led_b.advance('xfer_1')   # BOTH_READY
        rec_b = led_b.advance('xfer_1')   # TRANSFERRING -- the state this branch is about
        assert rec_b.custody == 'TRANSFERRING'
        led_b.interrupt('xfer_1', reason_code='TRANSFER_UNKNOWN')
        rec_b = led_b.resolve('xfer_1', non_transfer_proven=False)
        assert rec_b.stage == 'NEEDS_ATTENTION'
        assert rec_b.custody == 'TRANSFERRING', 'an unproven stop claimed the tray moved back'
        assert not rec_b.is_done
        assert rec_b.is_stuck

    def test_resolve_requires_an_explicit_verdict(self):
        """The parameter is keyword-only with no default, so 'was it proven' cannot be answered
        by omission. A default here would be the mechanism by which a half-transferred tray gets
        written off."""
        _, led, rec = live()
        rec = led.advance('xfer_1')
        led.interrupt('xfer_1', reason_code='TRANSFER_TIMEOUT')
        with pytest.raises(TypeError):
            led.resolve('xfer_1')                     # no verdict at all
        refuses('REFUSED_BAD_TYPE', led.resolve, 'xfer_1', non_transfer_proven='yes')
        refuses('REFUSED_BAD_TYPE', led.resolve, 'xfer_1', non_transfer_proven=None)

    def test_resolving_something_that_is_not_stopping_is_refused(self):
        _, led, rec = live()
        refuses('REFUSED_CONFLICT', led.resolve, 'xfer_1', non_transfer_proven=True)

    def test_an_interrupt_on_a_terminal_transfer_is_refused(self):
        _, led, rec = live()
        led.interrupt('xfer_1', reason_code='CONTACT_LOST')
        led.resolve('xfer_1', non_transfer_proven=True)
        refuses('REFUSED_CONFLICT', led.interrupt, 'xfer_1', reason_code='CONTACT_LOST')

    def test_a_stopped_transfer_cannot_be_advanced(self):
        """A stopped transfer must be *resolved*, not pushed forward: otherwise `STOPPING` would
        be a suggestion and the resolution verdict could be bypassed entirely."""
        _, led, rec = live()
        led.interrupt('xfer_1', reason_code='CONTACT_LOST')
        refuses('REFUSED_CONFLICT', led.advance, 'xfer_1')

    def test_an_interrupt_with_an_undeclared_reason_is_refused(self):
        _, led, rec = live()
        refuses('REFUSED_BAD_ENUM', led.interrupt, 'xfer_1', reason_code='BECAUSE')

    def test_the_outcomes_are_counted_apart(self):
        _, led, _ = live()
        led.interrupt('xfer_1', reason_code='CONTACT_LOST')
        led.resolve('xfer_1', non_transfer_proven=False)
        summary = led.summary()
        assert summary['needs_attention'] == 1
        assert summary['aborted'] == 0, 'an unproven stop was counted as an abort'


# ---------------------------------------------------------------------------
# 5. restart: reconcile, do not "recover"
# ---------------------------------------------------------------------------

class TestRestartReconciliation:

    def snapshot(self, led):
        return [rec.to_dict() for rec in led.by_id.values()]

    def test_a_record_round_trips_through_its_dict(self):
        t, led, rec = live()
        while rec.stage != 'RELEASED':
            rec = led.advance('xfer_1', evidence=['o.json'])
        restored = T.TransferLedger(epoch=7)
        restored.reconcile(self.snapshot(led), epoch=7, resources=t)
        assert restored.by_id['xfer_1'].stage == 'RELEASED'
        assert restored.by_id['xfer_1'].custody == 'AT_DESTINATION'
        assert restored.by_id['xfer_1'].history == list(S.TRANSFER_STATES)

    def test_reconcile_never_invents_an_aborted_transfer(self):
        """★★ 中央重启保留事务与账本，先对账再派发，不清空数据库冒充恢复.

        A crash is exactly the case in which nobody proved the tray did not move, so a mid-flight
        transfer must come back as `NEEDS_ATTENTION` and never as `ABORTED`. This is the same
        rule as `ABORTED` needing a proof, reached through the restart door -- and getting it
        wrong is how a system that crashes mid-handover declares the tray safe to move again."""
        t, led, rec = live()
        rec = led.advance('xfer_1')          # DOCK_VERIFIED
        rec = led.advance('xfer_1')          # BOTH_READY
        rec = led.advance('xfer_1')          # TRANSFERRING -- the tray is genuinely in flight
        assert rec.holds_both_ends
        snapshot = self.snapshot(led)

        restored = T.TransferLedger(epoch=7)
        by_id, findings = restored.reconcile(snapshot, epoch=7, resources=t)
        assert restored.by_id['xfer_1'].stage == 'NEEDS_ATTENTION'
        assert restored.by_id['xfer_1'].stage != 'ABORTED'
        assert restored.by_id['xfer_1'].return_to == 'TRANSFERRING'
        assert [f['kind'] for f in findings] == ['INTERRUPTED_IN_FLIGHT']
        assert restored.summary()['aborted'] == 0
        assert restored.summary()['needs_attention'] == 1

    def test_a_bare_reservation_is_also_not_aborted_at_restart(self):
        """★ `BOTH_RESERVED` has not moved anything, so it is tempting to call it `ABORTED` at
        restart. It must not be: a restart that returns it to a clean state still has to re-check
        the dock, because the crash may have happened during the docking move that the stage
        record had not yet caught up with. The conservative reading is the only safe one."""
        t, led, rec = live()
        assert rec.stage == 'BOTH_RESERVED' and not rec.holds_both_ends
        restored = T.TransferLedger(epoch=7)
        by_id, findings = restored.reconcile(self.snapshot(led), epoch=7, resources=t)
        assert restored.by_id['xfer_1'].stage == 'NEEDS_ATTENTION'
        assert [f['kind'] for f in findings] == ['INTERRUPTED_IN_FLIGHT']

    def test_a_stale_epoch_is_reported_at_reconcile(self):
        """★ After a restart nothing the old epoch held is still valid, and that has to be a
        countable finding rather than a silent acceptance."""
        t, led, rec = live(epoch=7)
        snapshot = self.snapshot(led)
        restored = T.TransferLedger(epoch=8)
        by_id, findings = restored.reconcile(snapshot, epoch=8, resources=t)
        assert 'STALE_EPOCH' in [f['kind'] for f in findings]
        finding = [f for f in findings if f['kind'] == 'STALE_EPOCH'][0]
        assert finding['transfer_id'] == 'xfer_1'

    def test_a_terminal_transfer_is_carried_over_untouched(self):
        t, led, rec = live()
        led.interrupt('xfer_1', reason_code='CONTACT_LOST')
        rec = led.resolve('xfer_1', non_transfer_proven=True)
        restored = T.TransferLedger(epoch=7)
        by_id, findings = restored.reconcile(self.snapshot(led), epoch=7, resources=t)
        assert restored.by_id['xfer_1'].stage == 'ABORTED'
        assert restored.by_id['xfer_1'].custody == 'AT_SOURCE'
        assert findings == []

    def test_a_record_whose_history_disagrees_with_its_stage_is_refused(self):
        """★ A corrupt log must stop the reconcile, not produce a plausible-looking record: a
        `stage` that the history never reached is exactly the shape of a hand-edited database
        pretending to be a recovery."""
        t, led, rec = live()
        snapshot = self.snapshot(led)
        snapshot[0]['stage'] = 'COMMITTED'      # history still ends at BOTH_RESERVED
        restored = T.TransferLedger(epoch=7)
        refuses('REFUSED_CONFLICT', restored.reconcile, snapshot, epoch=7, resources=t)

    def test_a_duplicated_transfer_in_the_log_is_refused(self):
        t, led, rec = live()
        snapshot = self.snapshot(led)
        restored = T.TransferLedger(epoch=7)
        refuses('REFUSED_CONFLICT', restored.reconcile, snapshot + snapshot,
                epoch=7, resources=t)


# ---------------------------------------------------------------------------
# 6. this module's own vocabulary must match the contract's
# ---------------------------------------------------------------------------

class TestVocabularyConsistency:

    def test_the_failure_stages_are_the_schema_ones(self):
        assert T.FAILURE_STAGES is S.TRANSFER_FAILURE_STATES

    def test_every_stage_has_a_custody_entry_or_is_a_failure_stage(self):
        """★ Catches a stage added to the contract that this module silently never enters: a
        name claiming a moment that nothing implements."""
        for stage in S.TRANSFER_STATES:
            assert stage in T.CUSTODY_ON_ENTRY, f'{stage} has no custody-on-entry decision'
        for stage in S.TRANSFER_FAILURE_STATES:
            assert stage not in T.CUSTODY_ON_ENTRY, \
                f'{stage} is a failure stage but has a custody-on-entry decision'

    def test_only_the_stages_that_prove_a_move_change_custody(self):
        """The policy, asserted as a value rather than described in a comment: exactly two
        stages change custody, and they are the two the contract names."""
        changing = {k for k, v in T.CUSTODY_ON_ENTRY.items() if v is not None}
        assert changing == {'TRANSFERRING', 'COMMITTED'}

    def test_every_custody_entry_is_a_declared_custody_state(self):
        for stage, custody in T.CUSTODY_ON_ENTRY.items():
            if custody is not None:
                assert custody in S.CUSTODY_STATES, f'{stage} -> undeclared {custody!r}'

    def test_every_refusal_reason_this_module_raises_is_declared(self):
        """★ Hard lesson 2 applied to the module's own vocabulary: walk the source for the
        literals and require each to be declared, so a new refusal cannot appear that a report
        cannot count."""
        import inspect
        import re
        src = inspect.getsource(T)
        found = set(re.findall(r"'(REFUSED_[A-Z_]+)'", src))
        assert found, 'the scan found no refusal literals -- the scan itself is broken'

        declared = set(T.TRANSFER_REFUSALS) | set(S.REFUSAL_REASONS)
        undeclared = found - declared
        assert not undeclared, f'undeclared: {sorted(undeclared)}'

        raised_here = set(re.findall(r"_refuse_start\('(REFUSED_[A-Z_]+)'", src))
        assert raised_here, 'no _refuse_start() call sites found'
        assert raised_here <= set(T.TRANSFER_REFUSALS), (
            f'TRANSFER_REFUSALS is missing what the module raises: '
            f'{sorted(raised_here - set(T.TRANSFER_REFUSALS))}')

    def test_the_business_failures_this_module_raises_are_declared_reason_codes(self):
        """DOCK_OUT_OF_TOLERANCE / STALE_OBSERVATION are business-level failures, not validator
        refusals, so they must live in the contract's reason list and not in the refusal
        vocabulary -- a report counting them must not find them in the wrong column."""
        for reason in ('DOCK_OUT_OF_TOLERANCE', 'STALE_OBSERVATION', 'TRANSFER_UNKNOWN'):
            assert reason in S.REASON_CODES
            assert reason not in S.REFUSAL_REASONS
