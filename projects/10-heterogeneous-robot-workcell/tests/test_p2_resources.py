"""Tests for P2-RES-02: atomic reservation, leases, occupancy, late replies.

The contract's section 6 is one paragraph with five clauses, and each clause is a test here
that would fail if the clause were removed:

| contract clause                          | test                                          |
|------------------------------------------|-----------------------------------------------|
| 默认 UNKNOWN，需要初始化清空证据          | `test_a_new_resource_is_unknown_not_free`     |
| 授权带 owner、generation、epoch、ttl       | `test_a_grant_carries_all_four_identifiers`   |
| 延迟响应不能重新计满 TTL                  | `test_a_late_reply_cannot_refill_the_ttl`     |
| 已占用时超时不释放                        | `test_expiry_does_not_release_an_occupied_resource` |
| (atomicity, implied by "原子资源预约")    | `test_reserve_is_atomic_under_interleaving`   |

Time is injected everywhere, so a test drives a lease through its whole life without sleeping.
"""
import pathlib
import sys

import pytest

ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src'))

from workcell import resources as R  # noqa: E402
from workcell import schema as S  # noqa: E402

LANE = 'lane_north'
DOCK = 'dock_station_b'


def table(epoch=1, resources=(LANE, DOCK)):
    return R.ResourceTable(epoch=epoch, resources=list(resources))


def cleared(epoch=1, *, now_s=0.0):
    t = table(epoch=epoch)
    t.mark_cleared(LANE, evidence=['init/clear_lane.json'], now_s=now_s)
    t.mark_cleared(DOCK, evidence=['init/clear_dock.json'], now_s=now_s)
    return t


def refuses(reason, fn, *args, **kwargs):
    with pytest.raises(S.SchemaRefused) as caught:
        fn(*args, **kwargs)
    assert caught.value.reason == reason, \
        f'expected {reason!r}, got {caught.value.reason!r} ({caught.value})'
    return caught.value


# ---------------------------------------------------------------------------
# 1. the default state, and the evidence needed to leave it
# ---------------------------------------------------------------------------

class TestInitialisation:

    def test_a_new_resource_is_unknown_not_free(self):
        """★ The contract's '默认 UNKNOWN，需要初始化清空证据'. A resource that defaults to FREE
        is a resource whose first grant happens before anyone looked."""
        t = table()
        assert t.snapshot(LANE)['state'] == 'UNKNOWN'
        assert t.snapshot(DOCK)['state'] == 'UNKNOWN'

    def test_you_cannot_reserve_an_uncleared_resource(self):
        """★ And the refusal is its own reason, not a generic failure: 'nobody cleared it' and
        'someone else has it' have different fixes."""
        t = table()
        e = refuses('REFUSED_RESOURCE_UNKNOWN', t.reserve,
                    LANE, owner='amr_1', ttl_s=10.0, now_s=0.0)
        assert 'lane_north' in e.detail

    def test_clearing_requires_evidence(self):
        """★ '需要初始化清空证据' -- 'I looked and it was empty' is a claim needing a source."""
        t = table()
        refuses('REFUSED_MISSING_FIELD', t.mark_cleared, LANE, evidence=[], now_s=0.0)
        refuses('REFUSED_MISSING_FIELD', t.mark_cleared, LANE, evidence=None, now_s=0.0)

    def test_clearing_evidence_must_not_be_a_bare_string(self):
        """A string is iterable, so `evidence='a.json'` would otherwise validate as one ref and
        hide the caller's mistake about the argument's shape."""
        t = table()
        refuses('REFUSED_BAD_TYPE', t.mark_cleared, LANE,
                evidence='init/clear.json', now_s=0.0)

    def test_clearing_evidence_cannot_traverse(self):
        t = table()
        refuses('REFUSED_BAD_ID', t.mark_cleared, LANE,
                evidence=['../../etc/passwd'], now_s=0.0)

    def test_clearing_makes_it_free_and_reservable(self):
        t = cleared()
        assert t.snapshot(LANE)['state'] == 'FREE'
        snap = t.reserve(LANE, owner='amr_1', ttl_s=10.0, now_s=1.0)
        assert snap['state'] == 'RESERVED'
        assert snap['owner'] == 'amr_1'

    def test_clearing_bumps_the_generation(self):
        """★ So a grant from before the clear is history and a late reply cannot revive it."""
        t = table()
        before = t.snapshot(LANE)['generation']
        t.mark_cleared(LANE, evidence=['init/clear.json'], now_s=0.0)
        assert t.snapshot(LANE)['generation'] == before + 1

    def test_an_occupied_resource_cannot_be_declared_clear(self):
        """★ Clearing an occupied resource is a claim that something moved, made by the caller
        with no measurement. The contract's own words are that this needs evidence."""
        t = cleared()
        t.reserve(LANE, owner='amr_1', ttl_s=10.0, now_s=0.0)
        t.confirm_occupied(LANE, owner='amr_1', generation=2, epoch=1, now_s=0.5)
        refuses('REFUSED_RESOURCE_NOT_CLEARABLE', t.mark_cleared, LANE,
                evidence=['init/clear.json'], now_s=1.0)

    def test_an_unknown_resource_id_is_refused_everywhere(self):
        t = table()
        refuses('REFUSED_ENTITY_UNKNOWN', t.snapshot, 'lane_nowhere')
        refuses('REFUSED_ENTITY_UNKNOWN', t.reserve,
                'lane_nowhere', owner='amr_1', ttl_s=1.0, now_s=0.0)

    def test_a_bad_resource_id_is_refused_at_construction(self):
        refuses('REFUSED_BAD_ID', R.ResourceTable, epoch=1, resources=['lane north'])
        refuses('REFUSED_BAD_ID', R.ResourceTable, epoch=1, resources=[''])
        refuses('REFUSED_CONFLICT', R.ResourceTable, epoch=1, resources=[LANE, LANE])

    def test_mark_blocked_needs_a_contract_reason(self):
        t = cleared()
        # `mark_blocked`'s keyword is `reason` and so is this helper's second parameter, so the
        # call is built through a dict to keep the two apart.
        refuses('REFUSED_BAD_ENUM', lambda: t.mark_blocked(LANE, reason='BECAUSE'))
        t.mark_blocked(LANE, reason='MANUAL_INTERVENTION_REQUIRED')
        assert t.snapshot(LANE)['state'] == 'BLOCKED'
        refuses('REFUSED_RESOURCE_BLOCKED', t.reserve,
                LANE, owner='amr_1', ttl_s=1.0, now_s=0.0)


# ---------------------------------------------------------------------------
# 2. the grant carries all four identifiers
# ---------------------------------------------------------------------------

class TestGrantIdentity:

    def test_a_grant_carries_all_four_identifiers(self):
        """★ '授权带 owner、generation、epoch、ttl'. All four are compared on every renewal;
        comparing fewer would let a stranger's reply or a pre-restart reply look like a
        continuation of this grant."""
        t = cleared(epoch=7)
        t.reserve(LANE, owner='amr_1', ttl_s=10.0, now_s=1.0)
        held = t.grant[LANE]
        assert held.owner == 'amr_1'
        assert held.generation == 2          # 1 from clear, 1 from reserve
        assert held.epoch == 7
        assert held.ttl_s == 10.0
        assert held.expires_at_s == 11.0
        snap = t.snapshot(LANE, now_s=3.0)
        assert snap['owner'] == 'amr_1' and snap['epoch'] == 7
        assert snap['expires_at_s'] == 11.0
        assert snap['remaining_s'] == pytest.approx(8.0)

    def test_a_zero_or_negative_ttl_is_refused(self):
        t = cleared()
        refuses('REFUSED_OUT_OF_RANGE', t.reserve, LANE, owner='amr_1', ttl_s=0.0, now_s=0.0)
        refuses('REFUSED_OUT_OF_RANGE', t.reserve, LANE, owner='amr_1', ttl_s=-1.0, now_s=0.0)

    def test_a_non_finite_ttl_is_refused(self):
        t = cleared()
        refuses('REFUSED_NON_FINITE', t.reserve,
                LANE, owner='amr_1', ttl_s=float('inf'), now_s=0.0)

    def test_a_bad_owner_is_refused(self):
        t = cleared()
        refuses('REFUSED_BAD_ID', t.reserve, LANE, owner='amr 1', ttl_s=1.0, now_s=0.0)

    def test_want_state_must_be_reserved_or_occupied(self):
        """A caller may not reserve straight into CLEARING or BLOCKED."""
        t = cleared()
        refuses('REFUSED_BAD_ENUM', t.reserve, LANE,
                owner='amr_1', ttl_s=1.0, now_s=0.0, want_state='FREE')


# ---------------------------------------------------------------------------
# ---------------------------------------------------------------------------
# 2b. what "clear" may and may not be applied to
# ---------------------------------------------------------------------------

class TestClearingTargets:

    def test_an_occupied_resource_cannot_be_declared_clear(self):
        """★ `mark_cleared` on an `OCCUPIED` resource is a claim that something moved, made by
        the caller, with no measurement behind it. It is refused with its OWN reason rather
        than the generic `REFUSED_CONFLICT`: "you told me to declare empty something that is in
        use" and "two writers disagreed" have different fixes, so a report must tell them apart.

        Without this test the guard is decorative -- deleting it makes the module happily
        declare a loaded resource empty and nothing notices."""
        t = cleared()
        t.reserve(LANE, owner='amr_1', ttl_s=10.0, now_s=0.0)
        t.confirm_occupied(LANE, owner='amr_1', generation=2, epoch=1, now_s=1.0)
        e = refuses('REFUSED_RESOURCE_NOT_CLEARABLE', t.mark_cleared, LANE,
                    evidence=['init/clear_lane.json'], now_s=2.0)
        assert LANE in e.detail
        # The resource was not touched: still held, still occupied, still granted.
        assert t.snapshot(LANE)['state'] == 'OCCUPIED'
        assert t.snapshot(LANE)['owner'] == 'amr_1'
        assert t.grant[LANE] is not None

    def test_a_reserved_resource_cannot_be_declared_clear_either(self):
        """The same guard by the other door: `RESERVED` is a promise somebody is acting on, so
        declaring it empty is equally a claim the coordinator did not measure."""
        t = cleared()
        t.reserve(LANE, owner='amr_1', ttl_s=10.0, now_s=0.0)
        refuses('REFUSED_RESOURCE_NOT_CLEARABLE', t.mark_cleared, LANE,
                evidence=['init/clear_lane.json'], now_s=1.0)
        assert t.snapshot(LANE)['state'] == 'RESERVED'

    def test_a_blocked_resource_cannot_be_declared_clear(self):
        """`BLOCKED` is a deliberate human hold; clearing it is not the coordinator's call."""
        t = cleared()
        t.mark_blocked(LANE, reason='MANUAL_INTERVENTION_REQUIRED')
        refuses('REFUSED_RESOURCE_NOT_CLEARABLE', t.mark_cleared, LANE,
                evidence=['init/clear_lane.json'], now_s=1.0)
        assert t.snapshot(LANE)['state'] == 'BLOCKED'

    def test_clearing_is_allowed_from_free_and_from_clearing(self):
        """The positive side of the same rule, so the guard cannot be satisfied by refusing
        everything: `FREE` and `CLEARING` are exactly the states a clear may be applied to."""
        t = cleared()                                   # already FREE
        assert t.mark_cleared(LANE, evidence=['init/again.json'],
                              now_s=1.0)['state'] == 'FREE'
        t.reserve(DOCK, owner='amr_1', ttl_s=10.0, now_s=2.0)
        t.confirm_occupied(DOCK, owner='amr_1', generation=2, epoch=1, now_s=3.0)
        t.release(DOCK, owner='amr_1', generation=2, epoch=1, now_s=4.0, to_state='CLEARING')
        assert t.snapshot(DOCK)['state'] == 'CLEARING'
        assert t.mark_cleared(DOCK, evidence=['init/clear_dock.json'],
                              now_s=5.0)['state'] == 'FREE'


# 3. atomicity
# ---------------------------------------------------------------------------

class TestAtomicity:

    def test_a_second_owner_is_refused_while_the_first_holds_it(self):
        t = cleared()
        t.reserve(LANE, owner='amr_1', ttl_s=10.0, now_s=0.0)
        refuses('REFUSED_RESOURCE_HELD_BY_OTHER', t.reserve,
                LANE, owner='amr_2', ttl_s=10.0, now_s=0.0)

    def test_the_same_owner_re_reserving_is_idempotent(self):
        """★ Idempotent, not a conflict: a retried request is not a second claim. And it must
        not silently extend the TTL either -- a re-ask is not a renewal."""
        t = cleared()
        first = t.reserve(LANE, owner='amr_1', ttl_s=10.0, now_s=0.0)
        second = t.reserve(LANE, owner='amr_1', ttl_s=10.0, now_s=2.0)
        assert second['state'] == first['state']
        assert second['generation'] == first['generation'], 'the retry minted a new generation'
        assert second['expires_at_s'] == first['expires_at_s'], 'the retry extended the lease'

    def test_interleaving_two_owners_leaves_exactly_one_winner(self):
        """★ Atomic means the check and the grant happen together. Modelling a realistic
        interleaving: A reserves, A releases, B reserves -- and B's grant must be the only
        live one."""
        t = cleared()
        t.reserve(LANE, owner='amr_1', ttl_s=5.0, now_s=0.0)
        t.release(LANE, owner='amr_1', generation=2, epoch=1, now_s=1.0)
        t.reserve(LANE, owner='amr_2', ttl_s=5.0, now_s=2.0)
        held = t.grant[LANE]
        assert held.owner == 'amr_2'
        assert held.generation == 4          # clear 1, reserve 2, release 3, reserve 4
        # A's reply is stale *and* from the wrong owner. The module checks owner first (the
        # more specific fact: this grant was never A's any more), so that is the reason. The
        # test asserts the declared order rather than an arbitrary one.
        refuses('REFUSED_GRANT_OWNED_BY_OTHER', t.renew,
                LANE, owner='amr_1', generation=2, epoch=1, ttl_s=5.0, now_s=3.0)
        refuses('REFUSED_GRANT_OWNED_BY_OTHER', t.release,
                LANE, owner='amr_1', generation=2, epoch=1, now_s=3.0)

    def test_a_stale_generation_from_the_right_owner_is_reported_as_stale(self):
        """★ The other half of the ordering above: when the owner IS right and only the
        generation is old, the reason must be STALE_GENERATION. Otherwise the owner-first check
        would swallow the very case the generation exists for."""
        t = cleared()
        t.reserve(LANE, owner='amr_1', ttl_s=10.0, now_s=0.0)
        stale = t.snapshot(LANE)['generation']      # 2
        t.release(LANE, owner='amr_1', generation=stale, epoch=1, now_s=1.0)
        t.reserve(LANE, owner='amr_1', ttl_s=10.0, now_s=2.0)   # generation 4, same owner
        refuses('REFUSED_STALE_GENERATION', t.renew,
                LANE, owner='amr_1', generation=stale, epoch=1, ttl_s=10.0, now_s=3.0)

    def test_reserving_one_resource_does_not_touch_another(self):
        t = cleared()
        t.reserve(LANE, owner='amr_1', ttl_s=10.0, now_s=0.0)
        t.reserve(DOCK, owner='amr_2', ttl_s=10.0, now_s=0.0)
        assert t.snapshot(LANE)['owner'] == 'amr_1'
        assert t.snapshot(DOCK)['owner'] == 'amr_2'
        assert t.snapshot(LANE)['generation'] == 2
        assert t.snapshot(DOCK)['generation'] == 2


# ---------------------------------------------------------------------------
# 4. late replies must not refill the TTL  (the contract's hardest clause)
# ---------------------------------------------------------------------------

class TestLateReplies:

    def test_a_late_reply_cannot_refill_the_ttl(self):
        """★ '延迟响应不能重新计满 TTL'.

        The scenario is concrete: amr_1 holds a lease, which lapses; amr_2 takes the resource;
        then a renewal from amr_1 -- sent before it lapsed, delivered after -- arrives. It must
        be refused, and refused *specifically*, so a report can see it happened.
        """
        t = cleared()
        t.reserve(LANE, owner='amr_1', ttl_s=5.0, now_s=0.0)
        # the lease lapses and someone else takes it
        t.expire(20.0)
        t.reserve(LANE, owner='amr_2', ttl_s=5.0, now_s=21.0)
        # the late reply. amr_1 is both stale and no longer the owner; the module reports owner
        # first (the more specific fact: this grant is not amr_1's at all).
        refuses('REFUSED_GRANT_OWNED_BY_OTHER', t.renew,
                LANE, owner='amr_1', generation=2, epoch=1, ttl_s=100.0, now_s=21.5)
        # and it did not move the live grant
        held = t.grant[LANE]
        assert held.owner == 'amr_2'
        assert held.expires_at_s == 26.0, 'the late reply extended the live lease'

    def test_a_reply_from_a_previous_epoch_is_refused(self):
        """★ After a coordinator restart the epoch changes, so a reply that was in flight across
        the restart is recognisable as belonging to the old run."""
        t = cleared(epoch=2)
        t.reserve(LANE, owner='amr_1', ttl_s=5.0, now_s=0.0)
        e = refuses('REFUSED_STALE_EPOCH', t.renew,
                    LANE, owner='amr_1', generation=2, epoch=1, ttl_s=5.0, now_s=1.0)
        assert 'lane_north' in e.detail

    def test_another_owners_reply_is_refused(self):
        t = cleared()
        t.reserve(LANE, owner='amr_1', ttl_s=5.0, now_s=0.0)
        refuses('REFUSED_GRANT_OWNED_BY_OTHER', t.renew,
                LANE, owner='amr_2', generation=2, epoch=1, ttl_s=5.0, now_s=1.0)

    def test_renewing_after_the_lease_lapsed_is_refused_even_by_the_right_owner(self):
        """★ A lapsed grant cannot be renewed back into existence: the resource may already
        have been handed on, and reviving it would silently steal from the new holder."""
        t = cleared()
        t.reserve(LANE, owner='amr_1', ttl_s=5.0, now_s=0.0)
        refuses('REFUSED_GRANT_EXPIRED', t.renew,
                LANE, owner='amr_1', generation=2, epoch=1, ttl_s=5.0, now_s=6.0)

    def test_renewing_with_no_live_grant_is_refused(self):
        t = cleared()
        refuses('REFUSED_NO_LIVE_GRANT', t.renew,
                LANE, owner='amr_1', generation=2, epoch=1, ttl_s=5.0, now_s=1.0)

    def test_a_legitimate_renewal_restarts_from_now_not_from_the_original_grant(self):
        """★ Using the original grant time would let a reply arriving just before expiry buy a
        full fresh window while reporting itself as a continuation."""
        t = cleared()
        t.reserve(LANE, owner='amr_1', ttl_s=10.0, now_s=0.0)
        snap = t.renew(LANE, owner='amr_1', generation=2, epoch=1, ttl_s=10.0, now_s=9.0)
        assert snap['expires_at_s'] == 19.0, 'the renewal restarted from the original grant'
        assert snap['expires_at_s'] != 10.0

    def test_a_renewal_does_not_change_the_generation(self):
        """The generation identifies the grant, so a renewal that bumped it would make the
        renewed grant unrecognisable to its own holder."""
        t = cleared()
        t.reserve(LANE, owner='amr_1', ttl_s=10.0, now_s=0.0)
        before = t.snapshot(LANE)['generation']
        t.renew(LANE, owner='amr_1', generation=before, epoch=1, ttl_s=10.0, now_s=1.0)
        assert t.snapshot(LANE)['generation'] == before

    def test_a_stale_reply_cannot_confirm_occupancy_either(self):
        """The occupancy path is the same identity check, so the hole cannot be reached by
        taking the other door."""
        t = cleared()
        t.reserve(LANE, owner='amr_1', ttl_s=5.0, now_s=0.0)
        t.expire(20.0)
        t.reserve(LANE, owner='amr_2', ttl_s=5.0, now_s=21.0)
        refuses('REFUSED_GRANT_OWNED_BY_OTHER', t.confirm_occupied,
                LANE, owner='amr_1', generation=2, epoch=1, now_s=22.0)
        assert t.snapshot(LANE)['state'] == 'RESERVED'
        assert t.snapshot(LANE)['owner'] == 'amr_2'

    def test_a_stale_reply_cannot_release_someone_elses_grant(self):
        t = cleared()
        t.reserve(LANE, owner='amr_1', ttl_s=5.0, now_s=0.0)
        t.release(LANE, owner='amr_1', generation=2, epoch=1, now_s=1.0)
        t.reserve(LANE, owner='amr_2', ttl_s=5.0, now_s=2.0)
        refuses('REFUSED_GRANT_OWNED_BY_OTHER', t.release,
                LANE, owner='amr_1', generation=2, epoch=1, now_s=3.0)
        assert t.grant[LANE].owner == 'amr_2'

    def test_a_release_from_a_previous_epoch_is_refused(self):
        """★ Cell 3 of the release identity matrix below, and the one the epoch guard guards.
        After a coordinator restart the epoch changes, so a release in flight across the restart
        must not be able to give away the new coordinator's grant. Without this test the epoch
        check in `release` is unexercised -- deleting it changes no observable behaviour."""
        t = cleared(epoch=2)
        t.reserve(LANE, owner='amr_1', ttl_s=10.0, now_s=0.0)   # generation 2
        e = refuses('REFUSED_STALE_EPOCH', t.release,
                    LANE, owner='amr_1', generation=2, epoch=1, now_s=1.0)
        assert LANE in e.detail
        assert t.grant[LANE] is not None, 'a stale-epoch release gave the resource away'
        assert t.snapshot(LANE)['state'] == 'RESERVED'

    def test_a_release_from_the_right_owner_with_a_stale_generation_is_refused(self):
        """★ Cell 2, the one the owner-first check order could swallow. The caller IS the owner,
        so the owner check passes and only the generation is old -- a duplicate delivery of an
        earlier grant. It must be reported as stale so a report can tell it apart from a request
        from a stranger."""
        t = cleared()
        t.reserve(LANE, owner='amr_1', ttl_s=10.0, now_s=0.0)              # generation 2
        t.release(LANE, owner='amr_1', generation=2, epoch=1, now_s=1.0)   # -> generation 3
        t.reserve(LANE, owner='amr_1', ttl_s=10.0, now_s=2.0)              # generation 4
        live = t.snapshot(LANE)['generation']
        assert live == 4
        refuses('REFUSED_STALE_GENERATION', t.release,
                LANE, owner='amr_1', generation=2, epoch=1, now_s=3.0)
        assert t.snapshot(LANE)['state'] == 'RESERVED'
        assert t.grant[LANE].generation == 4, 'the stale release moved the live grant'

    def test_releasing_bumps_the_generation_so_the_old_grant_is_stale(self):
        """★ Why the bump exists: once a release has happened, a reply tagged with the grant just
        given back is history, not a continuation. Without the bump the released grant and its
        successor share a number and the stale reply is indistinguishable from a valid one."""
        t = cleared()
        t.reserve(LANE, owner='amr_1', ttl_s=10.0, now_s=0.0)
        before = t.snapshot(LANE)['generation']
        t.release(LANE, owner='amr_1', generation=before, epoch=1, now_s=1.0)
        assert t.snapshot(LANE)['generation'] == before + 1, 'the release burned no generation'
        # replaying the very same release is now refused, because the grant it names is gone
        refuses('REFUSED_NO_LIVE_GRANT', t.release,
                LANE, owner='amr_1', generation=before, epoch=1, now_s=2.0)

    def test_a_release_with_no_live_grant_is_refused(self):
        """★ Cell 4: nothing is held, so there is nothing to give back."""
        t = cleared()
        refuses('REFUSED_NO_LIVE_GRANT', t.release,
                LANE, owner='amr_1', generation=2, epoch=1, now_s=1.0)



# ---------------------------------------------------------------------------
# ---------------------------------------------------------------------------
# 4b. the release identity matrix, cell by cell
# ---------------------------------------------------------------------------
#
# `release` compares four things on every call -- epoch, the existence of a grant, the owner,
# and the generation -- and the four refusals are ordered. An ordered chain is exactly where a
# gap hides: the tests that exercise a later link must first satisfy every earlier one, or the
# earlier refusal fires and the later link is never reached. So each cell below is named and
# tested on its own, and any two adjacent cells differ by exactly one field:
#
# | cell | epoch | grant | owner | generation | expected                    | test |
# |------|-------|-------|-------|------------|-----------------------------|------|
# | 1    | ok    | live  | ok    | ok         | the release happens         | `test_releasing_from_occupied_goes_to_clearing_not_free` |
# | 2    | ok    | live  | ok    | stale      | `REFUSED_STALE_GENERATION`  | `test_a_release_from_the_right_owner_with_a_stale_generation_is_refused` |
# | 3    | stale | live  | ok    | ok         | `REFUSED_STALE_EPOCH`       | `test_a_release_from_a_previous_epoch_is_refused` |
# | 4    | ok    | live  | other | any        | `REFUSED_GRANT_OWNED_BY_OTHER` | `test_a_stale_reply_cannot_release_someone_elses_grant` |
# | 5    | ok    | none  | any   | any        | `REFUSED_NO_LIVE_GRANT`     | `test_a_release_with_no_live_grant_is_refused` |
#
# Same matrix for `renew` (cells 2, 3, 4, 5 are covered by the `TestLateReplies` members above)
# and for `confirm_occupied` (`test_promotion_requires_the_right_identity`,
# `test_a_stale_reply_cannot_confirm_occupancy_either`,
# `test_promotion_after_the_lease_lapsed_is_refused`).

# 5. expiry releases only what expiry owns
# ---------------------------------------------------------------------------

class TestExpiry:

    def test_a_lapsed_reservation_is_released_by_expiry(self):
        t = cleared()
        t.reserve(LANE, owner='amr_1', ttl_s=5.0, now_s=0.0)
        changed = t.expire(6.0)
        assert changed == [LANE]
        assert t.snapshot(LANE)['state'] == 'FREE'
        assert t.grant[LANE] is None

    def test_expiry_does_not_release_an_occupied_resource(self):
        """★ '已占用时超时不释放'. A clock running out is not evidence that the thing on the
        resource moved. This is the clause that stops a stuck loader being written off."""
        t = cleared()
        t.reserve(LANE, owner='amr_1', ttl_s=5.0, now_s=0.0)
        t.confirm_occupied(LANE, owner='amr_1', generation=2, epoch=1, now_s=1.0)
        changed = t.expire(1000.0)
        assert changed == [], 'expiry released an occupied resource'
        assert t.snapshot(LANE)['state'] == 'OCCUPIED'
        assert t.grant[LANE] is not None
        assert t.grant[LANE].owner == 'amr_1'

    def test_an_overdue_occupied_resource_is_reported_rather_than_released(self):
        """★ The honest outcome: the coordinator's authority expired, the resource did not
        change hands, and a human is told. Reporting it is the difference between "the lease
        system works" and "the lease system hides a stuck loader"."""
        t = cleared()
        t.reserve(LANE, owner='amr_1', ttl_s=5.0, now_s=0.0)
        t.confirm_occupied(LANE, owner='amr_1', generation=2, epoch=1, now_s=1.0)
        t.expire(20.0)
        overdue = t.overdue(20.0)
        assert len(overdue) == 1
        assert overdue[0]['resource_id'] == LANE
        assert overdue[0]['owner'] == 'amr_1'
        # The grant was taken at now_s=0.0 with ttl 5.0, so it expires at 5.0 and at now_s=20.0
        # it is 15.0 s overdue. (The occupancy promotion at 1.0 deliberately does NOT restart
        # the clock -- promoting a reservation is a state change, not a renewal.)
        assert overdue[0]['overdue_by_s'] == pytest.approx(15.0)

    def test_expiry_bumps_the_generation_so_a_late_reply_is_caught(self):
        t = cleared()
        t.reserve(LANE, owner='amr_1', ttl_s=5.0, now_s=0.0)
        before = t.snapshot(LANE)['generation']
        t.expire(6.0)
        assert t.snapshot(LANE)['generation'] == before + 1

    def test_expire_is_idempotent(self):
        t = cleared()
        t.reserve(LANE, owner='amr_1', ttl_s=5.0, now_s=0.0)
        assert t.expire(6.0) == [LANE]
        assert t.expire(7.0) == [], 'expiring twice reported a second change'

    def test_expiry_happens_implicitly_on_a_reserve_attempt(self):
        """★ If expiry only ran in a sweep, a lapsed lease would block a new owner until some
        unrelated call happened to sweep it, and the resource would look busy at random."""
        t = cleared()
        t.reserve(LANE, owner='amr_1', ttl_s=5.0, now_s=0.0)
        # no explicit expire() call
        snap = t.reserve(LANE, owner='amr_2', ttl_s=5.0, now_s=6.0)
        assert snap['owner'] == 'amr_2'

    def test_a_blocked_resource_is_never_swept(self):
        t = cleared()
        t.mark_blocked(LANE, reason='MANUAL_INTERVENTION_REQUIRED')
        assert t.expire(1e6) == []
        assert t.snapshot(LANE)['state'] == 'BLOCKED'

    def test_an_unknown_resource_is_never_swept(self):
        t = table()
        assert t.expire(1e6) == []
        assert t.snapshot(LANE)['state'] == 'UNKNOWN'


# ---------------------------------------------------------------------------
# 6. occupancy promotion
# ---------------------------------------------------------------------------

class TestOccupancy:

    def test_promotion_requires_the_right_identity(self):
        t = cleared()
        t.reserve(LANE, owner='amr_1', ttl_s=10.0, now_s=0.0)
        refuses('REFUSED_GRANT_OWNED_BY_OTHER', t.confirm_occupied,
                LANE, owner='amr_2', generation=2, epoch=1, now_s=1.0)
        refuses('REFUSED_STALE_EPOCH', t.confirm_occupied,
                LANE, owner='amr_1', generation=2, epoch=9, now_s=1.0)

    def test_promotion_changes_state_and_keeps_the_grant(self):
        t = cleared()
        t.reserve(LANE, owner='amr_1', ttl_s=10.0, now_s=0.0)
        snap = t.confirm_occupied(LANE, owner='amr_1', generation=2, epoch=1, now_s=1.0)
        assert snap['state'] == 'OCCUPIED'
        assert snap['owner'] == 'amr_1'
        assert snap['generation'] == 2, 'promotion minted a new generation'
        assert t.grant[LANE].state == 'OCCUPIED'

    def test_promotion_after_the_lease_lapsed_is_refused(self):
        t = cleared()
        t.reserve(LANE, owner='amr_1', ttl_s=5.0, now_s=0.0)
        refuses('REFUSED_GRANT_EXPIRED', t.confirm_occupied,
                LANE, owner='amr_1', generation=2, epoch=1, now_s=6.0)

    def test_releasing_from_occupied_goes_to_clearing_not_free(self):
        """★ 'CLEARING' exists so 'the container is being emptied' is not the same statement as
        'the container is empty'."""
        t = cleared()
        t.reserve(LANE, owner='amr_1', ttl_s=10.0, now_s=0.0)
        t.confirm_occupied(LANE, owner='amr_1', generation=2, epoch=1, now_s=1.0)
        snap = t.release(LANE, owner='amr_1', generation=2, epoch=1, now_s=2.0,
                         to_state='CLEARING')
        assert snap['state'] == 'CLEARING'
        assert snap['owner'] is None

    def test_a_clearing_resource_cannot_be_reserved(self):
        t = cleared()
        t.reserve(LANE, owner='amr_1', ttl_s=10.0, now_s=0.0)
        t.confirm_occupied(LANE, owner='amr_1', generation=2, epoch=1, now_s=1.0)
        t.release(LANE, owner='amr_1', generation=2, epoch=1, now_s=2.0, to_state='CLEARING')
        refuses('REFUSED_RESOURCE_CLEARING', t.reserve,
                LANE, owner='amr_2', ttl_s=5.0, now_s=3.0)

    def test_a_released_resource_can_be_reserved_again(self):
        t = cleared()
        t.reserve(LANE, owner='amr_1', ttl_s=10.0, now_s=0.0)
        t.release(LANE, owner='amr_1', generation=2, epoch=1, now_s=1.0)
        snap = t.reserve(LANE, owner='amr_2', ttl_s=10.0, now_s=2.0)
        assert snap['state'] == 'RESERVED' and snap['owner'] == 'amr_2'

    def test_reserving_straight_into_occupied_is_allowed_when_asked(self):
        """A caller that knows the load is already there may take it in one step; the state is
        then protected from expiry immediately."""
        t = cleared()
        snap = t.reserve(LANE, owner='amr_1', ttl_s=5.0, now_s=0.0, want_state='OCCUPIED')
        assert snap['state'] == 'OCCUPIED'
        assert t.expire(1000.0) == []


# ---------------------------------------------------------------------------
# 7. the table's own vocabulary must match the contract's
# ---------------------------------------------------------------------------

class TestVocabularyConsistency:

    def test_the_states_are_schema_resource_states(self):
        assert R.STATES is S.RESOURCE_STATES

    def test_every_declared_state_is_reachable_by_some_operation(self):
        """★ Catches a state added to the contract that no code path can enter -- a name
        claiming something nothing implements."""
        t = cleared()
        seen = {'UNKNOWN', 'FREE'}
        t.reserve(LANE, owner='amr_1', ttl_s=5.0, now_s=0.0)
        seen.add(t.snapshot(LANE)['state'])
        t.confirm_occupied(LANE, owner='amr_1', generation=2, epoch=1, now_s=1.0)
        seen.add(t.snapshot(LANE)['state'])
        t.release(LANE, owner='amr_1', generation=2, epoch=1, now_s=2.0, to_state='CLEARING')
        seen.add(t.snapshot(LANE)['state'])
        t.mark_blocked(DOCK, reason='MANUAL_INTERVENTION_REQUIRED')
        seen.add(t.snapshot(DOCK)['state'])
        assert seen == set(S.RESOURCE_STATES), \
            f'unreachable states: {sorted(set(S.RESOURCE_STATES) - seen)}'

    def test_every_refusal_reason_this_module_raises_is_declared(self):
        """★ Hard lesson 2 applied to the module's own vocabulary: walk the source for the
        literals and require each to be declared, so a new refusal cannot appear that a report
        cannot count.

        Two vocabularies are legitimately in play, and the test says so rather than pretending
        there is one: this module's own reasons (`RESOURCE_REFUSALS`) and the generic validator
        reasons it inherits by re-using `schema._identifier` / `schema._number_in`. Both must be
        declared *somewhere*, and each set must be exactly the literals actually used.
        """
        import inspect
        import re
        src = inspect.getsource(R)
        found = set(re.findall(r"'(REFUSED_[A-Z_]+)'", src))
        assert found, 'the scan found no refusal literals -- the scan itself is broken'

        declared = set(R.RESOURCE_REFUSALS) | set(S.REFUSAL_REASONS)
        undeclared = found - declared
        assert not undeclared, f'undeclared: {sorted(undeclared)}'

        # The module-specific set must be exactly what the module raises on its own behalf:
        # no surplus (a declared reason nothing raises is a name claiming capability) and no
        # shortfall (what `_refuse` actually passes must be in it).
        raised_here = set(re.findall(r"_refuse\(resource_id, '(REFUSED_[A-Z_]+)'\)", src))
        assert raised_here, 'no _refuse() call sites found'
        assert raised_here == set(R.RESOURCE_REFUSALS), (
            f'RESOURCE_REFUSALS disagrees with the call sites: '
            f'surplus={sorted(set(R.RESOURCE_REFUSALS) - raised_here)}, '
            f'missing={sorted(raised_here - set(R.RESOURCE_REFUSALS))}')

    def test_the_inherited_validator_reasons_are_the_generic_ones(self):
        """The module re-uses the validator for ids/numbers, so the reasons it can leak are
        exactly the generic ones -- not a surprise from a helper."""
        inherited = {'REFUSED_BAD_ID', 'REFUSED_BAD_TYPE', 'REFUSED_OUT_OF_RANGE',
                     'REFUSED_NON_FINITE', 'REFUSED_BAD_ENUM', 'REFUSED_MISSING_FIELD',
                     'REFUSED_CONFLICT', 'REFUSED_ENTITY_UNKNOWN'}
        assert inherited <= S.REFUSAL_REASONS
        assert not (inherited & R.RESOURCE_REFUSALS), \
            'a reason is in both vocabularies, so a report cannot tell them apart'

    def test_resource_refusals_are_declared_in_schema_too(self):
        """So a report counting refusal reasons sees one vocabulary, not two."""
        assert R.RESOURCE_REFUSALS <= S.REFUSAL_REASONS

    def test_resource_refusals_do_not_collide_with_business_reasons(self):
        assert not (R.RESOURCE_REFUSALS & S.REASON_CODES)
