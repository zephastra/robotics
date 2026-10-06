"""G1 pure contract tests. No physics, no world files, no MuJoCo.

Each test is a *counterexample*: it asserts the contract REFUSES. A test that
passes because the guard is dead is the failure mode this file exists to catch,
so every guard here is exercised by an input that must trip it.

Also asserts the module cannot mutate physics: no mujoco import, no qpos writes.
"""
from pathlib import Path
import math
import sys

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src'))
sys.path.insert(0, str(ROOT / 'experiments'))

from workcell.retainer_supervision import (  # noqa: E402
    ContractRefused, MECHANISM_SIDES, STATES, FAULT_REASONS, RetainerSupervisor,
    make_observation, validate_observation, transport_authorization,
)

VEHICLE, RUN = 'amr_1', 'p4-belt-g1-unit-01'
CLOSE_T, CLOSE_TOL = -0.025, 0.01
RELEASE_T, RELEASE_TOL = math.pi / 2, 0.01
MAX_AGE = 0.5


def sides(pos=CLOSE_T, contact=True, source='tray', force=2.0, speed=0.0):
    return {n: dict(position=pos, speed=speed, contact=contact,
                    support_source=source, normal_force_n=force) for n in MECHANISM_SIDES}


def obs(*, seq=1, wall=100.0, vehicle=VEHICLE, epoch=1, generation=1, run=RUN,
        visual='RESOLVED', s=None, mechanism='retainer'):
    return make_observation(
        vehicle_id=vehicle, epoch=epoch, generation=generation, sequence=seq,
        run_id=run, observed_wall_s=wall, sides=s or sides(), visual=dict(status=visual))


def fresh_obs(now, *, seq, **kw):
    """An observation stamped at `now`, so age checks do not mask deadlines."""
    return obs(seq=seq, wall=now, **kw)


def supervisor(close_deadline_s=None, release_deadline_s=None):
    return RetainerSupervisor(vehicle_id=VEHICLE, run_id=RUN, epoch=1, generation=1,
                              max_age_s=MAX_AGE, close_target=CLOSE_T, close_tol=CLOSE_TOL,
                              release_target=RELEASE_T, release_tol=RELEASE_TOL,
                              close_deadline_s=close_deadline_s,
                              release_deadline_s=release_deadline_s)


def bring_to_verified(sup):
    """Drive OPEN -> CLOSING -> CLOSED -> CLOSED_VERIFIED, asserting each step."""
    assert sup.command_close(now_wall_s=100.0)['admitted'] is True
    assert sup.state == 'CLOSING'
    r = sup.observe_closed(obs(seq=1), now_wall_s=100.0)
    assert r['state'] == 'CLOSED', r
    r = sup.observe_closed(obs(seq=2), now_wall_s=100.0)
    assert r['state'] == 'CLOSED_VERIFIED', r
    return sup


# --------------------------------------------------------------------------
# 1. Baseline: the honest positive path still works (no regression on concept)
# --------------------------------------------------------------------------

def test_verified_path_authorizes_transport():
    sup = bring_to_verified(supervisor())
    d = sup.authorize_transport(obs(seq=3), now_wall_s=100.0)
    assert d['allowed'] is True and d['authorized'] is True
    assert sup.state == 'TRANSPORTING'
    assert sup.snapshot()['transport_authorized'] is True


def test_close_geometry_reaching_target_is_not_yet_verified():
    """The core defect this module prevents: position != genuine retention."""
    sup = supervisor()
    sup.command_close()
    r = sup.observe_closed(obs(seq=1), now_wall_s=100.0)
    assert r['state'] == 'CLOSED'
    assert r['authorized'] is False
    # One sample showing geometry + contact is still ONE sample. It must not be
    # promoted to CLOSED_VERIFIED, and it must not authorize transport.
    assert sup.state == 'CLOSED'
    d = sup.authorize_transport(obs(seq=2), now_wall_s=100.0)
    assert d['authorized'] is False
    assert d['reason_code'] == 'COMMAND_CONFLICT'
    assert sup.snapshot()['transport_authorized'] is False


def test_illegal_state_jump_refused_immediately():
    """CLOSED -> TRANSPORTING is not a declared transition at all."""
    sup = supervisor()
    sup.command_close()
    sup.observe_closed(obs(seq=1), now_wall_s=100.0)
    assert sup.state == 'CLOSED'
    with pytest.raises(ContractRefused) as exc:
        sup._enter('TRANSPORTING')
    assert exc.value.reason == 'COMMAND_CONFLICT'


# --------------------------------------------------------------------------
# 2. State jumps: illegal transitions are refused, not silently accepted
# --------------------------------------------------------------------------

@pytest.mark.parametrize('target', ['TRANSPORTING', 'OPEN_VERIFIED', 'CLOSED_VERIFIED'])
def test_illegal_state_jump_refused(target):
    sup = supervisor()  # state == OPEN
    with pytest.raises(ContractRefused) as exc:
        sup._enter(target)
    assert exc.value.reason == 'COMMAND_CONFLICT'


def test_release_before_transport_is_refused():
    sup = supervisor()
    d = sup.release(obs(seq=1), now_wall_s=100.0)
    assert d['authorized'] is False
    assert d['reason_code'] == 'COMMAND_CONFLICT'
    assert sup.latched == 'COMMAND_CONFLICT'


def test_single_side_loss_latches_and_is_absorbing():
    sup = bring_to_verified(supervisor())
    one = sides()
    one[MECHANISM_SIDES[0]].update(contact=False, normal_force_n=0.0)
    d = sup.authorize_transport(obs(seq=4, s=one), now_wall_s=100.0)
    assert d['reason_code'] == 'SINGLE_SIDE_SUPPORT', d
    # A recovery must NOT be granted inside the same run, even with perfect input.
    again = sup.authorize_transport(obs(seq=5), now_wall_s=100.0)
    assert again['authorized'] is False and again['reason_code'] == 'SINGLE_SIDE_SUPPORT'


def test_stale_observation_latches_and_is_absorbing():
    sup = bring_to_verified(supervisor())
    d = sup.authorize_transport(obs(seq=4, wall=99.0), now_wall_s=100.0)
    assert d['reason_code'] == 'STALE_OBSERVATION', d
    again = sup.authorize_transport(obs(seq=5), now_wall_s=100.0)
    assert again['authorized'] is False and again['reason_code'] == 'STALE_OBSERVATION'
    assert sup.state == 'UNKNOWN'


def test_fault_state_is_absorbing_for_every_entry_point():
    sup = bring_to_verified(supervisor())
    one = sides()
    one[MECHANISM_SIDES[0]].update(contact=False, normal_force_n=0.0)
    sup.authorize_transport(obs(seq=4, s=one), now_wall_s=100.0)
    assert sup.latched == 'SINGLE_SIDE_SUPPORT'
    # Every public entry point must return the latch, not re-derive.
    for call in (lambda: sup.observe_closed(obs(seq=5), now_wall_s=100.0),
                 lambda: sup.release(obs(seq=6), now_wall_s=100.0),
                 lambda: sup.cancel(obs(seq=7), now_wall_s=100.0),
                 lambda: sup.command_close()):
        d = call()
        assert d['reason_code'] == 'SINGLE_SIDE_SUPPORT', d
        assert d['authorized'] is False


# --------------------------------------------------------------------------
# 3. Observation invalidation: every required field is load-bearing
# --------------------------------------------------------------------------

@pytest.mark.parametrize('drop', ['vehicle_id', 'mechanism', 'epoch', 'generation',
                                  'sequence', 'run_id', 'observed_wall_s', 'sides', 'visual'])
def test_missing_field_is_refused_not_defaulted(drop):
    o = obs(seq=3)
    del o[drop]
    ok, reason, _ = validate_observation(o, expect_vehicle=VEHICLE, expect_epoch=1,
                                         expect_generation=1, expect_run_id=RUN,
                                         max_age_s=MAX_AGE, now_wall_s=100.0,
                                         last_sequence=2)
    assert ok is False and reason == 'OBSERVATION_INVALID'


def test_partial_side_fields_are_refused():
    o = obs(seq=3)
    del o['sides'][MECHANISM_SIDES[0]]['normal_force_n']
    ok, reason, _ = validate_observation(o, expect_vehicle=VEHICLE, expect_epoch=1,
                                         expect_generation=1, expect_run_id=RUN,
                                         max_age_s=MAX_AGE, now_wall_s=100.0)
    assert ok is False and reason == 'OBSERVATION_INVALID'


def test_contradictory_force_and_contact_is_refused():
    contradictory = sides(contact=False, force=5.0)
    o = obs(seq=3, s=contradictory)
    ok, reason, _ = validate_observation(o, expect_vehicle=VEHICLE, expect_epoch=1,
                                         expect_generation=1, expect_run_id=RUN,
                                         max_age_s=MAX_AGE, now_wall_s=100.0)
    assert ok is False and reason == 'OBSERVATION_INVALID'


def test_contact_without_measurable_force_is_refused():
    o = obs(seq=3, s=sides(contact=True, force=0.0))
    ok, reason, _ = validate_observation(o, expect_vehicle=VEHICLE, expect_epoch=1,
                                         expect_generation=1, expect_run_id=RUN,
                                         max_age_s=MAX_AGE, now_wall_s=100.0)
    assert ok is False and reason == 'OBSERVATION_INVALID'


def test_nan_position_is_refused():
    s = sides(); s[MECHANISM_SIDES[1]]['position'] = float('nan')
    o = obs(seq=3, s=s)
    ok, reason, _ = validate_observation(o, expect_vehicle=VEHICLE, expect_epoch=1,
                                         expect_generation=1, expect_run_id=RUN,
                                         max_age_s=MAX_AGE, now_wall_s=100.0)
    assert ok is False and reason == 'OBSERVATION_INVALID'


# --------------------------------------------------------------------------
# 4. Single-side contact
# --------------------------------------------------------------------------

def test_single_side_contact_grants_no_transport():
    sup = bring_to_verified(supervisor())
    one_side = sides()
    one_side[MECHANISM_SIDES[0]]['contact'] = False
    one_side[MECHANISM_SIDES[0]]['normal_force_n'] = 0.0
    d = sup.authorize_transport(obs(seq=3, s=one_side), now_wall_s=100.0)
    assert d['authorized'] is False
    assert d['reason_code'] == 'SINGLE_SIDE_SUPPORT'
    assert MECHANISM_SIDES[0] in d['detail']  # names the side that lost support


def test_support_from_something_other_than_the_tray_is_refused():
    sup = bring_to_verified(supervisor())
    d = sup.authorize_transport(obs(seq=3, s=sides(source='deck')), now_wall_s=100.0)
    assert d['authorized'] is False
    assert d['reason_code'] == 'OBSERVATION_INVALID'


def test_direct_authorization_call_still_requires_measurable_force():
    """The conjunction is a public function; it must not rely on a prior validate.

    The supervisor path can never reach the force branch, because the validator
    refuses contact-without-force first. Called directly on a raw mapping that
    claims contact with zero force, the conjunction must still refuse -- that is
    the only way this guard is load-bearing, and the mutation harness flags it as
    DEAD without this test.
    """
    raw = obs(seq=1)  # shape-valid; we then inject the contradiction bypassing make_observation
    raw['sides'][MECHANISM_SIDES[0]]['contact'] = True
    raw['sides'][MECHANISM_SIDES[0]]['normal_force_n'] = 0.0
    d = transport_authorization(raw, expect_vehicle=VEHICLE, expect_epoch=1,
                                expect_generation=1, expect_run_id=RUN,
                                now_wall_s=100.0, max_age_s=MAX_AGE, last_sequence=-1,
                                close_target=CLOSE_T, close_tol=CLOSE_TOL,
                                release_target=RELEASE_T, release_tol=RELEASE_TOL,
                                state='CLOSED_VERIFIED')
    assert d['allowed'] is False
    assert d['reason_code'] == 'OBSERVATION_INVALID'


# --------------------------------------------------------------------------
# 5. Wrong vehicle identity
# --------------------------------------------------------------------------

def test_wrong_vehicle_id_grants_nothing():
    sup = bring_to_verified(supervisor())
    d = sup.authorize_transport(obs(seq=3, vehicle='amr_2'), now_wall_s=100.0)
    assert d['authorized'] is False and d['reason_code'] == 'WRONG_VEHICLE'


def test_wrong_run_id_grants_nothing():
    sup = bring_to_verified(supervisor())
    d = sup.authorize_transport(obs(seq=3, run='some-other-run'), now_wall_s=100.0)
    assert d['authorized'] is False and d['reason_code'] == 'RUN_IDENTITY_MISMATCH'


# --------------------------------------------------------------------------
# 6. Stale epoch / stale observation / regressed sequence
# --------------------------------------------------------------------------

def test_old_epoch_callback_is_refused():
    sup = bring_to_verified(supervisor())
    d = sup.authorize_transport(obs(seq=3, epoch=0), now_wall_s=100.0)
    assert d['authorized'] is False and d['reason_code'] == 'STALE_EPOCH'


def test_old_generation_is_refused():
    sup = bring_to_verified(supervisor())
    d = sup.authorize_transport(obs(seq=3, generation=0), now_wall_s=100.0)
    assert d['authorized'] is False and d['reason_code'] == 'STALE_EPOCH'


def test_expired_observation_is_refused():
    sup = bring_to_verified(supervisor())
    d = sup.authorize_transport(obs(seq=3, wall=99.0), now_wall_s=100.0)
    assert d['authorized'] is False and d['reason_code'] == 'STALE_OBSERVATION'


def test_future_observation_is_refused():
    sup = bring_to_verified(supervisor())
    d = sup.authorize_transport(obs(seq=3, wall=101.0), now_wall_s=100.0)
    assert d['authorized'] is False and d['reason_code'] == 'OBSERVATION_INVALID'


def test_regressed_sequence_is_refused():
    sup = bring_to_verified(supervisor())  # last_sequence == 2
    d = sup.authorize_transport(obs(seq=2), now_wall_s=100.0)
    assert d['authorized'] is False and d['reason_code'] == 'SEQUENCE_REGRESSED'


# --------------------------------------------------------------------------
# 7. Close timeout / release timeout / cancel / duplicate
# --------------------------------------------------------------------------

def test_close_geometry_failure_is_distinct_from_close_timeout():
    sup = supervisor()
    sup.command_close(now_wall_s=100.0)
    d = sup.observe_closed(obs(seq=1, s=sides(pos=0.9)), now_wall_s=100.0)
    assert d['state'] == 'CLOSING' and d['admitted'] is False
    d = sup.authorize_transport(obs(seq=2, s=sides(pos=0.9)), now_wall_s=100.0)
    assert d['authorized'] is False and d['reason_code'] == 'CLOSE_NOT_AT_TARGET'


def test_close_deadline_is_a_declared_budget_that_actually_expires():
    """CLOSE_TIMEOUT must be raisable by a clock, not only by missing geometry.

    Without a deadline the reason existed but nothing could reach it: the
    mutation harness proved that. This test pins the deadline behaviour.
    """
    sup = supervisor(close_deadline_s=2.0)
    sup.command_close(now_wall_s=100.0)
    # Geometry arrives, so no geometry-based timeout; but the budget runs out.
    d = sup.observe_closed(fresh_obs(101.0, seq=1, s=sides(pos=0.9)), now_wall_s=101.0)
    assert d['state'] == 'CLOSING'
    d = sup.observe_closed(fresh_obs(102.5, seq=2, s=sides(pos=0.9)), now_wall_s=102.5)
    assert d['reason_code'] == 'CLOSE_TIMEOUT', d
    assert sup.state == 'FAULT'


def test_close_deadline_does_not_fire_inside_budget():
    sup = supervisor(close_deadline_s=10.0)
    sup.command_close(now_wall_s=100.0)
    d = sup.observe_closed(fresh_obs(101.0, seq=1, s=sides(pos=0.9)), now_wall_s=101.0)
    assert d['reason_code'] is None and sup.state == 'CLOSING'


def test_release_deadline_is_a_declared_budget_that_actually_expires():
    sup = supervisor(release_deadline_s=1.0)
    bring_to_verified(sup)
    sup.authorize_transport(obs(seq=3), now_wall_s=100.0)
    assert sup.command_release(now_wall_s=100.0)['admitted'] is True
    d = sup.release(fresh_obs(100.4, seq=4, s=sides(pos=0.2)), now_wall_s=100.4)
    assert d['reason_code'] == 'RELEASE_NOT_AT_TARGET'  # inside budget, not at target
    d = sup.release(fresh_obs(101.5, seq=5, s=sides(pos=0.2)), now_wall_s=101.5)
    assert d['reason_code'] == 'RELEASE_TIMEOUT', d


def test_deadlines_are_validated_at_construction():
    for bad in (0, -1.0, float('nan'), float('inf')):
        with pytest.raises(ContractRefused):
            supervisor(close_deadline_s=bad)


def test_release_geometry_failure_is_distinct_from_release_timeout():
    """Two different causes must carry two different reason codes.

    Sharing one code made them indistinguishable: disabling the deadline still
    reported RELEASE_TIMEOUT for a purely geometric failure.
    """
    sup = bring_to_verified(supervisor())  # no release deadline at all
    sup.authorize_transport(obs(seq=3), now_wall_s=100.0)
    assert sup.command_release(now_wall_s=100.0)['admitted'] is True
    d = sup.release(obs(seq=4, s=sides(pos=0.2)), now_wall_s=100.0)
    assert d['authorized'] is False
    assert d['reason_code'] == 'RELEASE_NOT_AT_TARGET'


def test_release_timeout_requires_a_declared_deadline():
    """Without a declared budget, RELEASE_TIMEOUT must be unreachable."""
    sup = bring_to_verified(supervisor())  # release_deadline_s is None
    sup.authorize_transport(obs(seq=3), now_wall_s=100.0)
    assert sup.command_release(now_wall_s=100.0)['admitted'] is True
    # Release geometry is correct and clear, just observed much later.
    clear = sides(pos=RELEASE_T, contact=False, force=0.0)
    d = sup.release(fresh_obs(1999.0, seq=4, s=clear), now_wall_s=1999.0)
    assert d['reason_code'] is None and d['state'] == 'OPEN_VERIFIED'


def test_release_not_cleared_while_still_in_contact():
    sup = bring_to_verified(supervisor())
    sup.authorize_transport(obs(seq=3), now_wall_s=100.0)
    assert sup.command_release(now_wall_s=100.0)['admitted'] is True
    d = sup.release(obs(seq=4, s=sides(pos=RELEASE_T)), now_wall_s=100.0)
    assert d['authorized'] is False and d['reason_code'] == 'RELEASE_NOT_CLEARED'
    assert sup.state == 'TRANSPORTING'  # a failed release does not silently advance


def test_release_verified_when_clear():
    sup = bring_to_verified(supervisor())
    sup.authorize_transport(obs(seq=3), now_wall_s=100.0)
    assert sup.command_release(now_wall_s=100.0)['admitted'] is True
    clear = sides(pos=RELEASE_T, contact=False, force=0.0)
    d = sup.release(obs(seq=4, s=clear), now_wall_s=100.0)
    assert d['admitted'] is True and d['state'] == 'OPEN_VERIFIED'
    assert sup.snapshot()['transport_authorized'] is False


def test_cancel_grants_nothing_and_latches():
    sup = bring_to_verified(supervisor())
    d = sup.cancel(obs(seq=3), now_wall_s=100.0)
    assert d['admitted'] is False and d['authorized'] is False
    assert d['reason_code'] == 'CANCEL_REQUESTED'
    assert sup.state == 'FAULT'
    assert sup.snapshot()['transport_authorized'] is False


def test_duplicate_close_command_is_a_named_fault():
    sup = supervisor()
    assert sup.command_close(now_wall_s=100.0)['admitted'] is True
    d = sup.command_close(now_wall_s=100.0)
    assert d['admitted'] is False and d['reason_code'] == 'DUPLICATE_COMMAND'
    assert sup.latched == 'DUPLICATE_COMMAND'


def test_close_from_wrong_state_is_a_fault():
    sup = bring_to_verified(supervisor())
    d = sup.command_close()
    assert d['admitted'] is False
    # 'close' was already issued and the mechanism already moved: still a refusal,
    # and it must name a declared reason.
    assert d['reason_code'] in ('COMMAND_CONFLICT', 'DUPLICATE_COMMAND')
    assert sup.snapshot()['transport_authorized'] is False


# --------------------------------------------------------------------------
# 8. Initial visual occlusion -> UNKNOWN, never PASS
# --------------------------------------------------------------------------

def test_initial_visual_unknown_refuses_and_documents_as_unknown():
    sup = supervisor()
    sup.command_close()
    sup.observe_closed(obs(seq=1), now_wall_s=100.0)     # CLOSED
    d = sup.observe_closed(obs(seq=2, visual='UNKNOWN'), now_wall_s=100.0)
    assert d['state'] == 'UNKNOWN'
    assert d['reason_code'] == 'INITIAL_VISUAL_UNKNOWN'
    assert sup.snapshot()['transport_authorized'] is False


def test_visual_unknown_never_becomes_pass_via_authorization():
    sup = bring_to_verified(supervisor())
    d = sup.authorize_transport(obs(seq=3, visual='UNKNOWN'), now_wall_s=100.0)
    assert d['authorized'] is False
    assert d['reason_code'] == 'INITIAL_VISUAL_UNKNOWN'
    assert d['reason_code'] != 'PASS'


def test_visual_status_outside_vocabulary_is_refused():
    o = obs(seq=3, visual='MAYBE')
    ok, reason, _ = validate_observation(o, expect_vehicle=VEHICLE, expect_epoch=1,
                                         expect_generation=1, expect_run_id=RUN,
                                         max_age_s=MAX_AGE, now_wall_s=100.0)
    assert ok is False and reason == 'OBSERVATION_INVALID'


# --------------------------------------------------------------------------
# 9. Every failure reason is reachable and distinguishable
# --------------------------------------------------------------------------

def test_all_declared_fault_reasons_are_distinct():
    assert len(set(FAULT_REASONS)) == len(FAULT_REASONS)
    assert len(set(STATES)) == len(STATES)


def test_observations_are_self_describing_units():
    o = obs(seq=1)
    assert o['force_units'] == 'N' and o['position_units'] == 'rad'


# --------------------------------------------------------------------------
# 10. The contract cannot touch physics
# --------------------------------------------------------------------------

def test_module_never_touches_physics():
    source = (ROOT / 'src/workcell/retainer_supervision.py').read_text()
    for forbidden in ('import mujoco', 'mj_step', 'qpos[', 'qvel[', 'MjData('):
        assert forbidden not in source, forbidden
