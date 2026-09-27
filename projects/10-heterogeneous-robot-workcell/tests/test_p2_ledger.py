"""Tests for the P2 order ledger.

The ledger's job is to refuse. Three classes of refusal matter, and each has tests here that
would fail if the refusal were removed (the mutation harness at the end of `tests/` proves the
suite can go red):

  * **illegal transitions** -- skipping a stage, going backwards, re-constraining a stuck order;
  * **idempotency vs conflict** -- the same request is a replay, the same request with different
    content is a conflict, and neither may overwrite the other;
  * **unconfirmed cancellation** -- an order that was asked to stop is not an order that stopped.

The forward path is not retyped here. It is read from `schema.ORDER_FORWARD_PATH`, because a
test that hard-codes the ten stages would keep passing after the contract's list changed.
"""
import pathlib
import sys

import pytest

ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src'))

from workcell import ledger as L  # noqa: E402
from workcell import schema as S  # noqa: E402


def order(**over):
    base = {
        'schema_version': 1,
        'request_id': 'request-001',
        'order_id': 'order-001',
        'destination_id': 'station_b',
        'items': [{'type': 'red_block', 'count': 2}],
        'priority': 10,
    }
    base.update(over)
    return base


def refuses(reason, fn, *args, **kwargs):
    with pytest.raises(S.SchemaRefused) as caught:
        fn(*args, **kwargs)
    assert caught.value.reason == reason, \
        f'expected {reason!r}, got {caught.value.reason!r} ({caught.value})'
    return caught.value


def walk_to_end(ledger, order_id):
    """Advance one declared step at a time until the order reaches the end of the path.

    Deliberately *not* a `state -> target` jump: the point of these tests is that only the
    declared next step is legal, so the helper that drives them must take real steps.
    """
    record = ledger.by_order[order_id]
    while L._NEXT.get(record.state) is not None:
        record = ledger.advance(order_id)
    return record


# ---------------------------------------------------------------------------
# 1. submission: accept, replay, conflict
# ---------------------------------------------------------------------------

class TestSubmission:

    def test_a_valid_order_is_created_queued(self):
        led = L.OrderLedger()
        record, created = led.submit(order())
        assert created is True
        assert record.state == 'QUEUED'
        assert record.revision == 0
        assert record.last_progress == 'QUEUED'

    def test_same_request_same_content_is_a_replay_not_an_error(self):
        """★ '同 request_id 同内容返回原结果' -- and it must be the *same object*, because the
        caller's next question is what happened to it, not what they sent."""
        led = L.OrderLedger()
        first, created_a = led.submit(order())
        led.advance('order-001')          # move it, so a fresh copy would look different
        second, created_b = led.submit(order())
        assert created_a is True and created_b is False
        assert second is first
        assert second.state == 'PREPARING_TRAY', 'the replay lost the order\'s progress'
        assert led.refusals == [], 'a replay is not a refusal'

    def test_same_request_different_content_is_a_conflict(self):
        """★ The half that matters: a retry after a dropped reply and an edited order are
        indistinguishable by request_id alone, so content decides."""
        led = L.OrderLedger()
        led.submit(order())
        e = refuses('REFUSED_CONFLICT', led.submit,
                    order(items=[{'type': 'red_block', 'count': 3}]))
        assert 'request-001' in e.detail
        assert led.by_order['order-001'].order['items'] == [{'type': 'red_block', 'count': 2}], \
            'the original order was overwritten by the conflicting one'
        assert led.refusals and led.refusals[0][0] == 'REFUSED_CONFLICT'

    def test_same_order_id_under_a_new_request_is_a_conflict(self):
        led = L.OrderLedger()
        led.submit(order())
        refuses('REFUSED_CONFLICT', led.submit,
                order(request_id='request-002'))

    def test_two_orders_may_coexist_under_different_ids(self):
        led = L.OrderLedger()
        led.submit(order())
        led.submit(order(request_id='request-002', order_id='order-002'))
        assert set(led.by_order) == {'order-001', 'order-002'}

    def test_a_malformed_order_never_enters_the_ledger(self):
        led = L.OrderLedger()
        refuses('REFUSED_UNKNOWN_FIELD', led.submit, order(urgency='high'))
        assert led.by_order == {} and led.by_request == {}
        assert led.refusals == [], 'a validation failure is not a ledger conflict'

    def test_the_replay_index_and_the_record_index_agree(self):
        """★ Structural: a divergence between these two would make one order reachable under a
        request_id whose record says a different order_id."""
        led = L.OrderLedger()
        led.submit(order())
        led.submit(order(request_id='request-002', order_id='order-002'))
        for request_id, order_id in led.by_request.items():
            assert led.by_order[order_id].request_id == request_id


# ---------------------------------------------------------------------------
# 2. the forward path
# ---------------------------------------------------------------------------

class TestForwardPath:

    def test_the_whole_declared_path_is_walkable(self):
        """★ Read from schema, never retyped -- so a stage added to the contract is walked here
        automatically, and a test that hard-coded ten stages could not hide the change."""
        led = L.OrderLedger()
        led.submit(order())
        seen = ['QUEUED']
        for stage in S.ORDER_FORWARD_PATH[1:]:
            led.advance('order-001')
            seen.append(led.by_order['order-001'].state)
        assert seen == list(S.ORDER_FORWARD_PATH)
        assert led.by_order['order-001'].state == 'SUCCEEDED'
        assert led.by_order['order-001'].last_progress == 'SUCCEEDED'

    def test_each_step_bumps_the_revision_once(self):
        led = L.OrderLedger()
        led.submit(order())
        for expected in range(1, len(S.ORDER_FORWARD_PATH)):
            led.advance('order-001')
            assert led.by_order['order-001'].revision == expected

    def test_skipping_a_stage_is_refused(self):
        """★ A skipped stage would let an order reach IN_TRANSIT without ever being kitted."""
        led = L.OrderLedger()
        led.submit(order())
        e = refuses('REFUSED_BAD_TRANSITION', led.advance, 'order-001', 'KITTING')
        assert 'PREPARING_TRAY' in e.detail, 'the refusal must name the step that was expected'
        assert led.by_order['order-001'].state == 'QUEUED'

    def test_going_backwards_is_refused(self):
        """A backwards move is what a replayed event looks like; accepting it would raise the
        revision while the order went back a stage."""
        led = L.OrderLedger()
        led.submit(order())
        led.advance('order-001')
        led.advance('order-001')
        assert led.by_order['order-001'].state == 'KITTING'
        refuses('REFUSED_BAD_TRANSITION', led.advance, 'order-001', 'PREPARING_TRAY')

    def test_advancing_a_finished_order_is_refused(self):
        led = L.OrderLedger()
        led.submit(order())
        for _ in range(len(S.ORDER_FORWARD_PATH) - 1):
            led.advance('order-001')
        assert led.by_order['order-001'].state == 'SUCCEEDED'
        refuses('REFUSED_BAD_TRANSITION', led.advance, 'order-001')

    def test_an_unknown_order_id_is_refused(self):
        led = L.OrderLedger()
        refuses('REFUSED_ENTITY_UNKNOWN', led.advance, 'order-999')

    def test_the_forward_path_has_no_cycles(self):
        """★ Structural: if a stage appeared twice, `walk_to`-style loops would not terminate."""
        assert len(set(S.ORDER_FORWARD_PATH)) == len(S.ORDER_FORWARD_PATH)
        seen = set()
        state = S.ORDER_FORWARD_PATH[0]
        while state in L._NEXT:
            assert state not in seen
            seen.add(state)
            state = L._NEXT[state]
        assert state == 'SUCCEEDED'


# ---------------------------------------------------------------------------
# 3. constrained states and the return path
# ---------------------------------------------------------------------------

class TestConstrainedStates:

    @pytest.mark.parametrize('state', sorted(S.CONSTRAINED_ORDER_STATES))
    def test_every_constrained_state_can_be_entered_from_a_running_order(self, state):
        if state == 'CANCELING':
            pytest.skip('CANCELING is entered through cancel(), covered separately')
        led = L.OrderLedger()
        led.submit(order())
        led.advance('order-001')                       # PREPARING_TRAY
        led.constrain('order-001', state, 'NAV_FAILED')
        record = led.by_order['order-001']
        assert record.state == state
        assert record.return_to == 'PREPARING_TRAY', 'the stage it came from was not kept'
        assert record.reason_code == 'NAV_FAILED'

    def test_resume_returns_to_the_stage_it_came_from(self):
        """★ The half that makes `return_to` load-bearing: resuming "somewhere sensible" would
        be a second, hidden transition table."""
        led = L.OrderLedger()
        led.submit(order())
        led.advance('order-001')                       # PREPARING_TRAY
        led.advance('order-001')                       # KITTING
        led.constrain('order-001', 'NEEDS_ATTENTION', 'CONTACT_LOST')
        led.resume('order-001')
        assert led.by_order['order-001'].state == 'KITTING'
        assert led.by_order['order-001'].return_to is None
        assert led.by_order['order-001'].reason_code is None

    def test_advancing_a_constrained_order_is_refused_with_the_return_target_named(self):
        led = L.OrderLedger()
        led.submit(order())
        led.advance('order-001')
        led.constrain('order-001', 'WAITING', 'RESOURCE_UNKNOWN')
        e = refuses('REFUSED_BAD_TRANSITION', led.advance, 'order-001')
        assert 'PREPARING_TRAY' in e.detail, 'the caller is not told how to get unstuck'

    def test_re_constraining_a_stuck_order_is_refused(self):
        """★ Re-constraining is how a stuck order loses the stage it was actually stuck at."""
        led = L.OrderLedger()
        led.submit(order())
        led.advance('order-001')
        led.constrain('order-001', 'NEEDS_ATTENTION', 'CONTACT_LOST')
        refuses('REFUSED_BAD_TRANSITION', led.constrain,
                'order-001', 'WAITING', 'STALE_OBSERVATION')
        assert led.by_order['order-001'].return_to == 'PREPARING_TRAY'

    def test_a_constrained_state_needs_a_contract_reason(self):
        """★ Contract 7's list, enforced at the transition. A stuck order with no reason is an
        order nobody can act on."""
        led = L.OrderLedger()
        led.submit(order())
        led.advance('order-001')
        refuses('REFUSED_BAD_ENUM', led.constrain, 'order-001', 'WAITING', 'IT_BROKE')
        refuses('REFUSED_BAD_ENUM', led.constrain, 'order-001', 'WAITING', None)

    def test_an_unknown_constrained_state_is_refused(self):
        led = L.OrderLedger()
        led.submit(order())
        refuses('REFUSED_BAD_ENUM', led.constrain, 'order-001', 'PAUSED', 'NAV_FAILED')

    def test_resuming_a_running_order_is_refused(self):
        led = L.OrderLedger()
        led.submit(order())
        refuses('REFUSED_BAD_TRANSITION', led.resume, 'order-001')

    def test_a_failed_order_can_still_be_resumed(self):
        """FAILED is not terminal: it is resumable by design, which is why it is not in
        TERMINAL_ORDER_STATES."""
        led = L.OrderLedger()
        led.submit(order())
        led.advance('order-001')
        led.constrain('order-001', 'FAILED', 'KIT_MISMATCH')
        led.resume('order-001')
        assert led.by_order['order-001'].state == 'PREPARING_TRAY'
        assert 'FAILED' not in S.TERMINAL_ORDER_STATES


# ---------------------------------------------------------------------------
# 4. cancellation: asked to stop is not stopped
# ---------------------------------------------------------------------------

class TestCancellation:

    def test_cancel_moves_to_canceling_not_to_cancelled(self):
        """★ Contract 3: '取消必须确认停稳或明确冻结/attention'."""
        led = L.OrderLedger()
        led.submit(order())
        led.advance('order-001')
        record = led.cancel('order-001')
        assert record.state == 'CANCELING'
        assert record.reason_code == 'CANCEL_UNCONFIRMED'
        assert record.return_to == 'PREPARING_TRAY'
        assert record.state != 'CANCELLED'

    def test_confirmed_stop_finishes_the_cancellation(self):
        led = L.OrderLedger()
        led.submit(order())
        led.cancel('order-001')
        record = led.confirm_canceled('order-001', stopped_confirmed=True)
        assert record.state == 'CANCELLED'
        assert record.is_terminal()

    def test_an_unconfirmed_stop_becomes_attention_not_a_cancellation(self):
        """★ This is the '明确冻结/attention' half. Without it, a vehicle still driving would
        read as a cancelled order.

        Also a bug this test found: the first version of `confirm_canceled` reached the frozen
        branch by calling `constrain()`, which its own re-constrain guard refused because the
        order was already in CANCELING -- so the frozen branch was unreachable and the order
        stayed in CANCELING forever.
        """
        led = L.OrderLedger()
        led.submit(order())
        led.cancel('order-001')
        record = led.confirm_canceled('order-001', stopped_confirmed=False)
        assert record.state == 'NEEDS_ATTENTION'
        assert record.reason_code == 'CANCEL_UNCONFIRMED'
        assert record.state != 'CANCELLED'
        assert record.state != 'CANCELING', 'the unconfirmed-cancel branch did not move at all'

    def test_a_frozen_cancellation_remembers_the_stage_to_resume_to(self):
        """★ The bug above had a second consequence: the frozen order must still know where it
        was, or 'resume the order someone asked to cancel' has no answer."""
        led = L.OrderLedger()
        led.submit(order())
        led.advance('order-001')                       # PREPARING_TRAY
        led.advance('order-001')                       # KITTING
        led.cancel('order-001')
        record = led.confirm_canceled('order-001', stopped_confirmed=False)
        assert record.return_to == 'KITTING', 'the cancelled order forgot its stage'
        led.resume('order-001')
        assert led.by_order['order-001'].state == 'KITTING'

    def test_a_cancelled_order_can_be_resumed_from_the_frozen_state(self):
        """Because CANCELING depends on the stage it came from, the frozen path must carry it."""
        led = L.OrderLedger()
        led.submit(order())
        led.cancel('order-001')
        led.confirm_canceled('order-001', stopped_confirmed=False)
        assert led.by_order['order-001'].return_to == 'QUEUED'

    def test_stopped_confirmed_has_no_default(self):
        """★ A default of True would make every cancellation succeed; a default of False would
        make the parameter ceremonial. So it is a required keyword."""
        import inspect
        sig = inspect.signature(L.OrderLedger.confirm_canceled)
        param = sig.parameters['stopped_confirmed']
        assert param.default is inspect.Parameter.empty
        assert param.kind == inspect.Parameter.KEYWORD_ONLY

    def test_confirming_cancel_on_a_running_order_is_refused(self):
        led = L.OrderLedger()
        led.submit(order())
        refuses('REFUSED_BAD_TRANSITION', led.confirm_canceled,
                'order-001', stopped_confirmed=True)

    def test_a_cancelled_order_advances_nowhere(self):
        led = L.OrderLedger()
        led.submit(order())
        led.cancel('order-001')
        led.confirm_canceled('order-001', stopped_confirmed=True)
        refuses('REFUSED_BAD_TRANSITION', led.advance, 'order-001')
        refuses('REFUSED_BAD_TRANSITION', led.constrain, 'order-001', 'WAITING', 'NAV_FAILED')


# ---------------------------------------------------------------------------
# 5. history and the stuck report
# ---------------------------------------------------------------------------

class TestHistoryAndWhyStuck:

    def test_every_transition_is_recorded_in_order(self):
        led = L.OrderLedger()
        led.submit(order())
        led.advance('order-001')
        led.constrain('order-001', 'NEEDS_ATTENTION', 'CONTACT_LOST')
        led.resume('order-001')
        history = led.by_order['order-001'].history
        assert [h['to'] for h in history] == [
            'QUEUED', 'PREPARING_TRAY', 'NEEDS_ATTENTION', 'PREPARING_TRAY']
        assert history[0]['from'] is None
        for index, entry in enumerate(history):
            assert entry['revision'] == index, 'revision and history index disagree'

    def test_why_stuck_answers_the_three_contract_questions(self):
        """★ Contract 7 asks for the last real progress, who is being waited on, and the first
        refusal reason. All three must be answerable, and all three from one record."""
        led = L.OrderLedger()
        led.submit(order())
        led.advance('order-001')                       # PREPARING_TRAY
        led.advance('order-001')                       # KITTING
        led.advance('order-001')                       # VERIFYING_KIT
        led.constrain('order-001', 'NEEDS_ATTENTION', 'KIT_MISMATCH')

        why = led.why_stuck('order-001')
        assert why['last_progress'] == 'VERIFYING_KIT'      # last real forward progress
        assert why['waiting_on'] == 'operator:KIT_MISMATCH'  # who is being waited on
        assert why['first_refusal']['reason_code'] == 'KIT_MISMATCH'
        assert why['state'] == 'NEEDS_ATTENTION'
        assert why['return_to'] == 'VERIFYING_KIT'

    def test_the_first_refusal_is_the_first_not_the_latest(self):
        """An order stuck twice must report where it first went wrong, per the contract."""
        led = L.OrderLedger()
        led.submit(order())
        led.advance('order-001')
        led.constrain('order-001', 'WAITING', 'RESOURCE_UNKNOWN')
        led.resume('order-001')
        led.constrain('order-001', 'NEEDS_ATTENTION', 'CONTACT_LOST')
        why = led.why_stuck('order-001')
        assert why['first_refusal']['reason_code'] == 'RESOURCE_UNKNOWN'
        assert why['reason_code'] == 'CONTACT_LOST', 'the current reason is still reported'

    def test_waiting_on_is_derived_from_the_stage(self):
        """★ Derived, not stored: a separate field would be a second place to keep in step."""
        expected = {
            'QUEUED': 'source_workcell',
            'PREPARING_TRAY': 'source_workcell',
            'KITTING': 'source_workcell',
            'VERIFYING_KIT': 'source_workcell',
            'WAITING_TRANSPORT': 'transport',
            'LOADING': 'transport',
            'IN_TRANSIT': 'destination_workcell',
            'UNLOADING': 'destination_workcell',
            'VERIFYING_DELIVERY': 'destination_workcell',
            'SUCCEEDED': None,
        }
        led = L.OrderLedger()
        led.submit(order())
        for stage in S.ORDER_FORWARD_PATH:
            assert L.OrderLedger._waiting_on(led.by_order['order-001']) == expected[stage], stage
            if stage != S.ORDER_FORWARD_PATH[-1]:
                led.advance('order-001')

    def test_to_dict_is_a_copy_not_a_window(self):
        led = L.OrderLedger()
        led.submit(order())
        snapshot = led.by_order['order-001'].to_dict()
        led.advance('order-001')
        assert snapshot['state'] == 'QUEUED', 'the snapshot changed under the caller'
        assert snapshot['history'] == [{'revision': 0, 'from': None, 'to': 'QUEUED',
                                        'reason_code': None, 'return_to': None}]


# ---------------------------------------------------------------------------
# 6. the table must agree with the contract it implements
# ---------------------------------------------------------------------------

class TestTableConsistency:

    def test_the_ledger_next_table_is_derived_from_schema(self):
        """★ Hard lesson 8: the forward path must not be typed twice."""
        assert L._NEXT == {a: b for a, b in zip(S.ORDER_FORWARD_PATH, S.ORDER_FORWARD_PATH[1:])}

    def test_every_forward_stage_has_a_successor_except_the_last(self):
        for stage in S.ORDER_FORWARD_PATH[:-1]:
            assert stage in L._NEXT, f'{stage} has no declared next stage'
        assert S.ORDER_FORWARD_PATH[-1] not in L._NEXT

    def test_constrained_states_and_dead_ends_are_schema_sets(self):
        assert L.CONSTRAINED is S.CONSTRAINED_ORDER_STATES
        assert L.DEAD_ENDS is S.TERMINAL_ORDER_STATES

    def test_no_forward_stage_is_also_constrained_or_terminal(self):
        assert not (set(S.ORDER_FORWARD_PATH) & S.CONSTRAINED_ORDER_STATES), \
            'a stage that is both forward and constrained would be walkable and stuck at once'
        assert not (set(S.ORDER_FORWARD_PATH) & S.TERMINAL_ORDER_STATES - {'SUCCEEDED'})
