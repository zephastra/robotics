"""Prove the P2 core suite can fail: mutate the modules, each mutation must break tests.

Hard lesson 2: a check that cannot be shown to fail is not a check. The suite passing says
nothing until I have seen it go red for each property it claims. Each entry below removes
exactly one guarantee and names the tests that must notice.

Run:  .venv/bin/python <this>       (from the project root)
Exit code 0 means every mutation was caught.
"""
import ast
import inspect
import os
import re
import subprocess
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SCHEMA = os.path.join(ROOT, "src/workcell/schema.py")
LEDGER = os.path.join(ROOT, "src/workcell/ledger.py")
RESOURCES = os.path.join(ROOT, "src/workcell/resources.py")
TRANSFER = os.path.join(ROOT, "src/workcell/transfer.py")
SAFETY = os.path.join(ROOT, "src/workcell/safety.py")

TEST_FILES = ["tests/test_p2_schema.py", "tests/test_p2_ledger.py",
              "tests/test_p2_resources.py", "tests/test_p2_transfer.py",
              "tests/test_p2_safety.py"]


def _path(which):
    return {"schema": SCHEMA, "ledger": LEDGER, "resources": RESOURCES,
            "transfer": TRANSFER, "safety": SAFETY}[which]

def check_anchors_unique():
    """Every anchor must occur EXACTLY once in the file it mutates. Raises on any violation.

    ★ This guard exists because of a defect in the harness itself, not in the modules. Thirteen
    anchors in the first version of this file contained a literal `//n` where `\n` was intended,
    so `str.replace` changed nothing, and those rows mutated nothing while still reporting a
    status. `write_source`'s "the mutation is on disk" assertion could not see it: it compares
    the mutated text against the returned text, and both were the same unchanged string.

    Two ways an anchor can be wrong, and both look exactly like a passing row:

      * **zero occurrences** -- the mutation is a no-op; the suite is green and the guarantee
        was never removed. This is "a check that cannot be shown to fail".
      * **two or more occurrences** -- the mutation changes several guarantees at once, so a
        CAUGHT row does not say which one the tests are pinning.

    Checked *before* the suite runs and treated as fatal: a harness whose rows do not do what
    they claim is worse than no harness, because it produces confident output.
    """
    tree = ast.parse(inspect.getsource(sys.modules[__name__]))
    for node in tree.body:
        if isinstance(node, ast.Assign) and getattr(node.targets[0], 'id', '') == 'MUTATIONS':
            literal = node.value.elts
            break
    else:
        raise AssertionError('MUTATIONS not found while checking anchors')

    #: Anchors that legitimately occur more than once because the guard they remove is written
    #: at two call sites that are both covered by the same test. Declared here explicitly rather
    #: than tolerated silently: an unexplained `2` is indistinguishable from a mutation that
    #: changes two guarantees while only one of them is being tested.
    #: Each entry: (label, which, count) -- verified by hand, one line of justification each.
    MULTI_OK = {
        ("resume() forgets the return target", "ledger", 2):
            'both `advance` and `resume` end in the same `_move(...)`; '
            'test_resume_returns_to_the_stage_it_came_from notices either',
        ("a lapsed grant can be renewed back into existence", "resources", 2):
            'the lapsed-grant guard sits in both `renew` and `release`; '
            'the renewal tests cover both',
    }

    problems = []
    seen_multi = []
    for elt in literal:
        label = elt.elts[0].value
        which = elt.elts[1].value
        body = elt.elts[2].body
        if not (isinstance(body, ast.Call) and isinstance(body.func, ast.Attribute)
                and body.func.attr == 'replace'):
            problems.append((label, which, 'NOT-A-REPLACE'))
            continue
        anchor = _anchor_value(body.args[0])
        n = originals[_path(which)].count(anchor)
        if n == 1:
            continue
        if n == 0:
            # ★ The dangerous one: the mutation is a no-op, the row still reports a status, and
            # "COULD NOT APPLY" is the only thing standing between that and a confident lie.
            problems.append((label, which, n))
        elif (label, which, n) in MULTI_OK:
            seen_multi.append((label, which, n))
        else:
            problems.append((label, which, n))

    for label, which, n in seen_multi:
        print(f'   anchor occurs {n}x by design in {which}: {label}')
        print(f'     why: {MULTI_OK[(label, which, n)]}')
    if problems:
        for label, which, n in problems:
            print(f'!! anchor occurs {n} time(s) in {which}: {label}')
        raise AssertionError(f'{len(problems)} mutation anchor(s) do not occur exactly once')
    return len(literal)


def _anchor_value(node):
    """The exact string a `s.replace(A, B)` mutation searches for.

    ★ The values are joined from the AST rather than read with `literal_eval` off the source
    text: several anchors are adjacent string literals, and reading the *source* form makes the
    anchor depend on how the escape happened to be typed -- which is precisely how the `//n`
    defect got in. The node values cannot be corrupted that way.
    """
    if isinstance(node, ast.Constant):
        return node.value
    return ''.join(_anchor_value(part) for part in node.values)




#: (label, which file, mutate-the-source, [test names that must fail])
MUTATIONS = [
    # ---- schema.py -------------------------------------------------------
    ("extra keys accepted (drop unknown-field refusal)",
     "schema",
     lambda s: s.replace(
         "    if extra:\n"
         "        raise SchemaRefused('REFUSED_UNKNOWN_FIELD', ','.join(sorted(extra)), field=field)\n",
         "    if extra:\n        pass  # MUTATION\n"),
     ["test_unknown_top_level_field", "test_item_unknown_field"]),

    ("bool treated as int (remove the bool guard from _int_in)",
     "schema",
     lambda s: s.replace(
         "def _int_in(value, field, low, high):\n"
         "    if _is_bool(value) or not isinstance(value, int):",
         "def _int_in(value, field, low, high):\n"
         "    if not isinstance(value, int):  # MUTATION\n"),
     ["test_true_is_not_one", "test_sequence_must_be_a_non_negative_int"]),

    ("NaN allowed through _finite (drop the isfinite guard)",
     "schema",
     lambda s: s.replace(
         "    if not math.isfinite(value):", "    if False:  # MUTATION\n"),
     ["test_a_non_finite_coordinate_is_refused", "test_nan_and_infinity"]),

    ("duplicate JSON keys no longer refused",
     "schema",
     lambda s: s.replace(
         "        if key in seen:\n"
         "            raise SchemaRefused('REFUSED_DUPLICATE_KEY', key)",
         "        if key in seen and False:  # MUTATION\n"
         "            raise SchemaRefused('REFUSED_DUPLICATE_KEY', key)"),
     ["test_duplicate_key_is_not_last_wins"]),

    ("a missing required skill argument is defaulted",
     "schema",
     lambda s: s.replace(
         "    missing = required - set(arguments)\n    if missing:",
         "    missing = required - set(arguments)\n    if False:  # MUTATION\n"),
     ["test_a_required_argument_missing_is_refused_not_defaulted"]),

    ("the battery conditional requirement is ignored",
     "schema",
     lambda s: s.replace(
         "        if 'battery' in caps:", "        if False:  # MUTATION\n"),
     ["test_declaring_the_capability_without_a_reading_is_refused"]),

    ("a failure may arrive without a reason",
     "schema",
     lambda s: s.replace(
         "        if reason is None:", "        if False:  # MUTATION\n"),
     ["test_a_failure_without_a_reason_is_refused"]),

    ("a duplicate part row is merged instead of refused",
     "schema",
     lambda s: s.replace(
         "        if part_type in seen_types:",
         "        if False:  # MUTATION\n"),
     ["test_duplicate_type_in_two_lines_is_refused"]),

    ("the incomplete-evidence rule is dropped",
     "schema",
     lambda s: s.replace(
         "    if not complete and not missing:",
         "    if False:  # MUTATION\n"),
     ["test_incomplete_must_list_what_is_missing"]),

    ("an evidence ref may traverse out of the report",
     "schema",
     lambda s: s.replace(
         "    if not EVIDENCE_PATTERN.match(text):",
         "    if False:  # MUTATION\n"),
     ["test_evidence_cannot_traverse_out_of_the_report"]),

    ("control quantities are no longer refused by name",
     "schema",
     lambda s: s.replace(
         "        if forbidden:\n",
         "        if False:  # MUTATION: forbidden names allowed\n"),
     ["test_control_quantities_are_refused_by_name"]),

    ("safety_result regains NOT_APPLICABLE",
     "schema",
     lambda s: s.replace(
         "    safety_result = _enum(payload['safety_result'], 'safety_result', TRI_STATE)",
         "    safety_result = _enum(payload['safety_result'], 'safety_result',\n"
         "                          TRI_STATE | {'NOT_APPLICABLE'})  # MUTATION"),
     ["test_safety_has_no_not_applicable"]),

    # ---- ledger.py -------------------------------------------------------
    ("stage skipping becomes legal",
     "ledger",
     lambda s: s.replace(
         "    return _NEXT.get(current) == target",
         "    return target in _NEXT.values() or target == current  # MUTATION"),
     ["test_skipping_a_stage_is_refused", "test_going_backwards_is_refused"]),

    ("a replayed request returns a fresh record (idempotency broken)",
     "ledger",
     lambda s: s.replace(
         "            if existing.fingerprint == fingerprint:\n"
         "                return existing, False",
         "            if existing.fingerprint == fingerprint:\n"
         "                return OrderRecord(order), False  # MUTATION"),
     ["test_same_request_same_content_is_a_replay_not_an_error"]),

    ("same request with different content is accepted as an update",
     "ledger",
     lambda s: s.replace(
         "            if existing.fingerprint == fingerprint:",
         "            if True:  # MUTATION: conflicts treated as replays\n"),
     ["test_same_request_different_content_is_a_conflict"]),

    ("resume() forgets the return target",
     "ledger",
     lambda s: s.replace(
         "        self._move(record, target, reason_code=None, return_to=None)",
         "        self._move(record, 'QUEUED', reason_code=None, return_to=None)  # MUTATION"),
     ["test_resume_returns_to_the_stage_it_came_from"]),

    ("constrained states no longer record where they came from",
     "ledger",
     lambda s: s.replace(
         "        self._move(record, state, reason_code=reason_code, return_to=record.state)",
         "        self._move(record, state, reason_code=reason_code, return_to=None)  # MUTATION"),
     ["test_resume_returns_to_the_stage_it_came_from",
      "test_a_frozen_cancellation_remembers_the_stage_to_resume_to"]),

    ("an unconfirmed cancel is treated as a completed cancellation",
     "ledger",
     lambda s: s.replace(
         "        if stopped_confirmed is not True:",
         "        if False:  # MUTATION\n"),
     ["test_an_unconfirmed_stop_becomes_attention_not_a_cancellation"]),

    ("a constrained state needs no reason code",
     "ledger",
     lambda s: s.replace(
         "        if reason_code is None or reason_code not in S.REASON_CODES:",
         "        if False:  # MUTATION\n"),
     ["test_a_constrained_state_needs_a_contract_reason"]),

    ("the first refusal is overwritten by the latest",
     "ledger",
     lambda s: s.replace(
         "        first_refusal = next(\n"
         "            (h for h in record.history if h['reason_code'] is not None), None)",
         "        first_refusal = next(\n"
         "            (h for h in reversed(record.history) if h['reason_code'] is not None),\n"
         "            None)  # MUTATION: latest, not first"),
     ["test_the_first_refusal_is_the_first_not_the_latest"]),

    ("re-constraining a stuck order becomes legal",
     "ledger",
     lambda s: s.replace(
         "        if record.is_constrained():\n"
         "            # Re-constraining an already constrained order is how a stuck order loses the\n"
         "            # stage it was actually stuck at.\n"
         "            raise SchemaRefused(",
         "        if False:  # MUTATION\n"
         "            raise SchemaRefused("),
     ["test_re_constraining_a_stuck_order_is_refused"]),

    ("last_progress tracks every state, including constrained ones",
     "ledger",
     lambda s: s.replace(
         "        if target in S.ORDER_FORWARD_PATH and target != previous:",
         "        if target != previous:  # MUTATION: constrained states count as progress\n"),
     ["test_why_stuck_answers_the_three_contract_questions"]),

    ("waiting_on stops being derived from the stage",
     "ledger",
     lambda s: s.replace(
         "        if record.state in ('WAITING_TRANSPORT', 'LOADING'):\n"
         "            return 'transport'",
         "        if False:  # MUTATION\n            return 'transport'"),
     ["test_waiting_on_is_derived_from_the_stage"]),

    # ---- resources.py ----------------------------------------------------
    ("a new resource starts FREE instead of UNKNOWN",
     "resources",
     lambda s: s.replace(
         "            self.state[resource_id] = 'UNKNOWN'",
         "            self.state[resource_id] = 'FREE'  # MUTATION"),
     ["test_a_new_resource_is_unknown_not_free",
      "test_you_cannot_reserve_an_uncleared_resource"]),

    ("clearing needs no evidence",
     "resources",
     lambda s: s.replace(
         "        if not evidence:\n"
         "            raise SchemaRefused('REFUSED_MISSING_FIELD',\n"
         "                                'clearing a resource requires evidence', field='evidence')",
         "        if False:  # MUTATION: no evidence required to clear\n"
         "            raise SchemaRefused('REFUSED_MISSING_FIELD', 'x', field='evidence')"),
     ["test_clearing_requires_evidence"]),

    ("clearing does not bump the generation",
     "resources",
     lambda s: s.replace(
         "        self.state[resource_id] = 'FREE'\n"
         "        # Clearing bumps the generation: any grant from before the clear is now history, and a\n"
         "        # late reply carrying the old generation will be refused.\n"
         "        self.generation[resource_id] += 1",
         "        self.state[resource_id] = 'FREE'  # MUTATION: generation not bumped"),
     ["test_clearing_bumps_the_generation"]),

    ("a resource in any state may be declared empty",
     "resources",
     # ★ The anchor must name the ONE guard carrying the rule. An earlier version of this row
     # replaced `if current == 'OCCUPIED':`, which appeared twice and was a strict subset of the
     # `not in` guard below -- so removing it changed nothing, and the row read "NO TEST FAILED"
     # while the suite was in fact fine. That subset guard has been deleted from the module as
     # dead code; this now removes the guard that is load-bearing.
     lambda s: s.replace(
         "        if current not in ('UNKNOWN', 'FREE', 'CLEARING'):",
         "        if False:  # MUTATION: anything may be declared empty\n"),
     ["test_an_occupied_resource_cannot_be_declared_clear",
      "test_a_reserved_resource_cannot_be_declared_clear_either",
      "test_a_blocked_resource_cannot_be_declared_clear"]),

    ("expiry releases an occupied resource",
     "resources",
     lambda s: s.replace(
         "        if self.state[resource_id] == 'OCCUPIED':",
         "        if False:  # MUTATION: occupied resources are released by the clock\n"),
     ["test_expiry_does_not_release_an_occupied_resource"]),

    ("expiry never runs (no implicit sweep)",
     "resources",
     lambda s: s.replace(
         "        if now_s <= held.expires_at_s:\n            return False\n",
         "        if True:  # MUTATION: never treat anything as lapsed\n            return False\n"),
     ["test_a_lapsed_reservation_is_released_by_expiry",
      "test_expiry_happens_implicitly_on_a_reserve_attempt"]),

    ("a second owner can take a held resource",
     "resources",
     lambda s: s.replace(
         "            if held.owner != owner:\n"
         "                self._refuse(resource_id, 'REFUSED_RESOURCE_HELD_BY_OTHER')",
         "            if False:  # MUTATION: anyone may take an existing reservation\n"
         "                self._refuse(resource_id, 'REFUSED_RESOURCE_HELD_BY_OTHER')"),
     ["test_a_second_owner_is_refused_while_the_first_holds_it"]),

    ("re-reserving by the same owner restarts the clock",
     "resources",
     lambda s: s.replace(
         "        if current in ('RESERVED',) and self.grant[resource_id] is not None:",
         "        if False:  # MUTATION: a repeat request is treated as a fresh grant\n"),
     ["test_the_same_owner_re_reserving_is_idempotent"]),

    ("a lapsed grant can be renewed back into existence",
     "resources",
     lambda s: s.replace(
         "        if now_s > held.expires_at_s:",
         "        if False:  # MUTATION: expiry is not enforced on renewal\n"),
     ["test_renewing_after_the_lease_lapsed_is_refused_even_by_the_right_owner"]),

    ("a late reply may refill the TTL (generation not compared)",
     "resources",
     lambda s: s.replace(
         "        if generation != held.generation:\n"
         "            self._refuse(resource_id, 'REFUSED_STALE_GENERATION')\n"
         "        if now_s > held.expires_at_s:",
         "        if False:  # MUTATION: generation not compared\n"
         "            self._refuse(resource_id, 'REFUSED_STALE_GENERATION')\n"
         "        if now_s > held.expires_at_s:"),
     ["test_a_stale_generation_from_the_right_owner_is_reported_as_stale",
      "test_a_late_reply_cannot_refill_the_ttl"]),

    ("renewal computes the new expiry from the original grant time",
     "resources",
     lambda s: s.replace(
         "        held.ttl_s = ttl_s\n        held.granted_at_s = now_s",
         "        held.ttl_s = ttl_s  # MUTATION: granted_at_s not moved to now_s"),
     ["test_a_legitimate_renewal_restarts_from_now_not_from_the_original_grant"]),

    ("the epoch is not checked on release",
     "resources",
     lambda s: s.replace(
         "        to_state = S._enum(to_state, 'to_state', frozenset({'FREE', 'CLEARING'}))\n"
         "        held = self.grant[resource_id]\n"
         "        if epoch != self.epoch:",
         "        to_state = S._enum(to_state, 'to_state', frozenset({'FREE', 'CLEARING'}))\n"
         "        held = self.grant[resource_id]\n"
         "        if False:  # MUTATION\n"),
     ["test_a_release_from_a_previous_epoch_is_refused"]),

    ("a release from a non-owner is honoured",
     "resources",
     lambda s: s.replace(
         "        if held.owner != owner:\n"
         "            self._refuse(resource_id, 'REFUSED_GRANT_OWNED_BY_OTHER')\n"
         "        if generation != held.generation:\n"
         "            self._refuse(resource_id, 'REFUSED_STALE_GENERATION')\n\n"
         "        self.generation[resource_id] += 1\n"
         "        self.state[resource_id] = to_state",
         "        if False:  # MUTATION: anyone may release\n"
         "            self._refuse(resource_id, 'REFUSED_GRANT_OWNED_BY_OTHER')\n"
         "        if generation != held.generation:\n"
         "            self._refuse(resource_id, 'REFUSED_STALE_GENERATION')\n\n"
         "        self.generation[resource_id] += 1\n"
         "        self.state[resource_id] = to_state"),
     ["test_another_owners_reply_is_refused",
      "test_a_stale_reply_cannot_release_someone_elses_grant"]),

    ("releasing does not bump the generation",
     "resources",
     lambda s: s.replace(
         "        self.generation[resource_id] += 1\n"
         "        self.state[resource_id] = to_state\n"
         "        self.grant[resource_id] = None",
         "        self.state[resource_id] = to_state  # MUTATION: generation not bumped\n"
         "        self.grant[resource_id] = None"),
     ["test_releasing_bumps_the_generation_so_the_old_grant_is_stale"]),

    ("a reservation becomes occupied without checking the holder",
     "resources",
     lambda s: s.replace(
         "        held = self.grant[resource_id]\n"
         "        if epoch != self.epoch:\n"
         "            self._refuse(resource_id, 'REFUSED_STALE_EPOCH')\n"
         "        if held is None:\n"
         "            self._refuse(resource_id, 'REFUSED_NO_LIVE_GRANT')\n"
         "        if held.owner != owner:\n"
         "            self._refuse(resource_id, 'REFUSED_GRANT_OWNED_BY_OTHER')\n"
         "        if generation != held.generation:\n"
         "            self._refuse(resource_id, 'REFUSED_STALE_GENERATION')\n"
         "        if now_s is not None:",
         "        held = self.grant[resource_id]\n"
         "        if False:  # MUTATION: no identity check on promotion\n"
         "            pass\n"
         "        if now_s is not None:"),
     ["test_promotion_requires_the_right_identity",
      "test_a_stale_reply_cannot_confirm_occupancy_either"]),

    ("promotion ignores the clock (an expired reservation becomes occupancy)",
     "resources",
     lambda s: s.replace(
         "        if now_s is not None:\n"
         "            now_s = S._number_in(now_s, 'now_s', 0.0, S.MAX_DURATION_S)\n"
         "            if now_s > held.expires_at_s:",
         "        if False:  # MUTATION: promotion ignores expiry\n"
         "            if now_s > held.expires_at_s:"),
     ["test_promotion_after_the_lease_lapsed_is_refused"]),

    # ---- transfer.py -----------------------------------------------------
    ("a transfer may skip a stage (the edge map stops being derived)",
     "transfer",
     # ★ The forward map is derived from the contract's own tuple; this replaces the derivation
     # with a constant that jumps to the end, which is exactly the defect a hand-written edge map
     # would have and cannot be seen by reading it.
     lambda s: s.replace(
         "_NEXT = {a: b for a, b in zip(STAGES, STAGES[1:])}",
         "_NEXT = {a: 'RELEASED' for a in STAGES[:-1]}  # MUTATION"),
     ["test_the_forward_edges_are_derived_from_the_stage_list",
      "test_the_eight_stages_are_walked_in_order",
      "test_a_transfer_cannot_skip_a_stage"]),

    ("custody moves at a setup stage (DOCK_VERIFIED claims the tray arrived)",
     "transfer",
     lambda s: s.replace(
         "    'DOCK_VERIFIED': None,",
         "    'DOCK_VERIFIED': 'AT_DESTINATION',  # MUTATION"),
     ["test_custody_moves_only_at_the_two_stages_that_prove_it",
      "test_only_the_stages_that_prove_a_move_change_custody"]),

    ("a stopped transfer stops holding both ends",
     "transfer",
     # ★ This is the mutation that caught a real bug: `holds_both_ends` originally excluded
     # `STOPPING`, so an interrupted transfer released the scene it was protecting.
     lambda s: s.replace(
         "        return self.stage in ('BOTH_READY', 'TRANSFERRING', 'RECEIVER_CONFIRMED', "
         "'STOPPING')",
         "        return self.stage in ('BOTH_READY', 'TRANSFERRING', 'RECEIVER_CONFIRMED')"
         "  # MUTATION"),
     ["test_an_interrupt_touches_no_custody"]),

    ("ABORTED is granted on an unproven stop",
     "transfer",
     # ★ The contract's 仅证明未发生转移时. Without this the tray gets written off.
     lambda s: s.replace(
         "        if non_transfer_proven:",
         "        if True:  # MUTATION: any stop is an abort"),
     ["test_aborted_requires_a_proof_and_needs_attention_does_not",
      "test_the_outcomes_are_counted_apart"]),

    ("an unproven stop hands the tray back to the source",
     "transfer",
     lambda s: s.replace(
         "            record.custody = 'AT_SOURCE'\n"
         "            self._enter(record, 'ABORTED')",
         "            self._enter(record, 'ABORTED')"),
     ["test_aborted_requires_a_proof_and_needs_attention_does_not"]),

    ("an interrupt reassigns custody",
     "transfer",
     lambda s: s.replace(
         "        self._enter(record, 'STOPPING')\n        return record",
         "        record.custody = 'AT_DESTINATION'  # MUTATION\n"
         "        self._enter(record, 'STOPPING')\n        return record"),
     ["test_an_interrupt_touches_no_custody"]),

    ("the receiver confirmation needs no evidence",
     "transfer",
     lambda s: s.replace(
         "            if not evidence:\n"
         "                raise SchemaRefused('REFUSED_MISSING_FIELD',\n"
         "                                    'RECEIVER_CONFIRMED requires evidence', "
         "field='evidence')",
         "            if False:  # MUTATION: any claim will do\n"
         "                raise SchemaRefused('REFUSED_MISSING_FIELD',\n"
         "                                    'RECEIVER_CONFIRMED requires evidence', "
         "field='evidence')"),
     ["test_the_receiver_confirmation_needs_evidence"]),

    ("a false launch precondition is treated as satisfied",
     "transfer",
     # ★ The contract's 任一 UNKNOWN 拒绝启动 -- and a false boolean is the other half of it.
     lambda s: s.replace(
         "            if not value:",
         "            if False:  # MUTATION: an unsatisfied precondition passes"),
     # ★ The gap this row found: nothing was checking a *non-geometry* condition reported
     # False, so this mutation previously survived. The new test closes that hole.
     ["test_a_false_non_geometry_precondition_is_refused",
      "test_a_single_geometry_violation_is_refused_with_its_own_reason"]),

    ("an UNKNOWN precondition is silently defaulted instead of refused",
     "transfer",
     # The staleness check would then always pass: "a default pretending to be a measurement".
     lambda s: s.replace(
         "    if value is None:\n"
         "        return None, 'REFUSED_PRECONDITION_UNKNOWN'",
         "    if value is None:\n"
         "        return (0.0 if low is not None else False), None  # MUTATION"),
     ["test_every_launch_precondition_must_be_known",
      "test_stale_evidence_is_refused"]),

    ("a stale epoch is accepted at launch",
     "transfer",
     lambda s: s.replace(
         "        if epoch != self.epoch:\n"
         "            self._refuse_start('REFUSED_STALE_EPOCH',",
         "        if False:  # MUTATION: any epoch will do\n"
         "            self._refuse_start('REFUSED_STALE_EPOCH',"),
     ["test_a_stale_epoch_is_refused"]),

    ("a failed start leaks the first reservation",
     "transfer",
     lambda s: s.replace(
         "            for resource in reserved:\n"
         "                resources.release(resource, owner=owner,",
         "            for resource in []:  # MUTATION: a failed start keeps what it took\n"
         "                resources.release(resource, owner=owner,"),
     ["test_a_failed_start_leaks_no_reservation"]),

    ("reconcile calls an interrupted transfer ABORTED",
     "transfer",
     # ★ The crash case the contract forbids from being called a proof.
     lambda s: s.replace(
         "                self._enter(record, 'NEEDS_ATTENTION')",
         "                record.custody = 'AT_SOURCE'  # MUTATION\n"
         "                self._enter(record, 'ABORTED')\n"
         "                self.aborted += 1"),
     ["test_reconcile_never_invents_an_aborted_transfer",
      "test_a_bare_reservation_is_also_not_aborted_at_restart"]),

    ("a corrupt record is reconciled anyway (history not cross-checked)",
     "transfer",
     lambda s: s.replace(
         "        if not record.history or record.history[-1] != stage:",
         "        if False:  # MUTATION: the history is not checked against the stage"),
     ["test_a_record_whose_history_disagrees_with_its_stage_is_refused"]),

    ("advancing a stopped transfer becomes legal",
     "transfer",
     lambda s: s.replace(
         "        if record.stage in FAILURE_STAGES:",
         "        if False:  # MUTATION: a stopped transfer can be pushed on"),
     ["test_a_stopped_transfer_cannot_be_advanced"]),

    ("a transfer to the same device is allowed",
     "transfer",
     lambda s: s.replace(
         "        if source == receiver:",
         "        if False:  # MUTATION"),
     ["test_a_transfer_to_the_same_device_is_refused"]),

    ("resolving something that is not stopping becomes legal",
     "transfer",
     lambda s: s.replace(
         "        if record.stage != 'STOPPING':",
         "        if False:  # MUTATION"),
     ["test_resolving_something_that_is_not_stopping_is_refused"]),

    # ---- safety.py : the gate -------------------------------------------
    ("the gate accepts commands before it is bound to an epoch",
     "safety",
     # ★ The one case where "the wheels moved and nobody knows why" is the true story.
     lambda s: s.replace(
         "        if self.epoch is None:\n"
         "            return self._refuse('REFUSED_STALE_EPOCH')\n",
         "        if self.epoch is None:  # MUTATION: the gate starts open\n"
         "            self.epoch = raw.get('epoch') if isinstance(raw, dict) else None\n"),
     ["test_the_gate_outputs_zero_before_it_is_bound_to_an_epoch"]),

    ("a foreign source may steer the gate",
     "safety",
     # ★ "two writers disagreed" must be impossible, not handled.
     lambda s: s.replace(
         "        if raw['source'] != self.source:\n"
         "            return self._refuse('REFUSED_WRONG_SOURCE')",
         "        if False:  # MUTATION: anyone may steer\n"
         "            return self._refuse('REFUSED_WRONG_SOURCE')"),
     ["test_a_command_from_a_different_source_is_refused"]),

    ("a malformed command extends the live lease",
     "safety",
     # ★ A malformed datagram must not be a way to keep a robot driving.
     lambda s: s.replace(
         "        if epoch != self.epoch:\n"
         "            return self._refuse('REFUSED_STALE_EPOCH')",
         "        if epoch != self.epoch:\n"
         "            self.accepted_at_s = now_s  # MUTATION: a refusal re-arms the watchdog\n"
         "            return self._refuse('REFUSED_STALE_EPOCH')"),
     ["test_a_malformed_command_is_refused_without_touching_the_lease",
      "test_a_refused_command_leaves_the_watchdog_alone"]),

    ("an out-of-range velocity is clamped instead of refused",
     "safety",
     lambda s: s.replace(
         "            v = S._number_in(raw['v'], 'v', -self.v_max, self.v_max)",
         "            v = max(-self.v_max, min(self.v_max, raw['v']))  # MUTATION: clamped"),
     ["test_a_malformed_command_is_refused_without_touching_the_lease",
      "test_an_out_of_range_command_is_not_silently_clamped"]),

    ("a command dated in the future is accepted",
     "safety",
     # ★ Clamping it would hide the clock disagreement; refusing it makes it countable.
     lambda s: s.replace(
         "        if issued_s > now_s + 0.05:\n"
         "            return self._refuse('REFUSED_FROM_THE_FUTURE')",
         "        if False:  # MUTATION: the future is fine\n"
         "            return self._refuse('REFUSED_FROM_THE_FUTURE')"),
     ["test_a_command_dated_in_the_future_is_refused_not_clamped"]),

    ("the stored lease is not clamped to the gate's own (sender sets the policy)",
     "safety",
     # ★★ This row used to mutate the guard `now_s - issued_s > min(ttl_s, self.ttl_s)`.
     # That mutation cannot change behaviour: `min()` is applied *twice* -- once in the guard and
     # once at the assignment to `active['ttl_s']` -- and the guard is evaluated at age 0, where
     # the two branches are identical. Removing the guard-side clamp leaves the stored value
     # clamped, so nothing the caller can observe changes. The row was removing redundancy, not
     # a guarantee; that is a dead-guard finding, not a test gap.
     # The clamp that DOES matter is the one at the assignment, and it is what this row removes.
     lambda s: s.replace(
         "        self.active = {'v': v, 'w': w, 'issued_s': issued_s, 'ttl_s': "
         "min(ttl_s, self.ttl_s),",
         "        self.active = {'v': v, 'w': w, 'issued_s': issued_s, 'ttl_s': "
         "ttl_s,  # MUTATION: the sender sets the policy"),
     ["test_the_gate_lease_is_the_tighter_of_the_two",
      "test_the_gate_lease_is_what_is_stored_and_what_expires"]),

    ("a stale generation can revive a stopped gate",
     "safety",
     # ★ 持盘车不能自动让另一台接管货物, read through the gate.
     lambda s: s.replace(
         "        if generation < self.generation:\n"
         "            return self._refuse('REFUSED_STALE_GENERATION')",
         "        if False:  # MUTATION: an old world may steer\n"
         "            return self._refuse('REFUSED_STALE_GENERATION')"),
     ["test_a_stale_generation_cannot_restart_a_stopped_gate",
      "test_a_replayed_sequence_number_is_refused"]),

    ("a replayed sequence number is accepted",
     "safety",
     lambda s: s.replace(
         "        if seq <= self.highest_seq:\n"
         "            return self._refuse('REFUSED_STALE_SEQ')",
         "        if False:  # MUTATION: replays are fine\n"
         "            return self._refuse('REFUSED_STALE_SEQ')"),
     ["test_a_replayed_sequence_number_is_refused"]),

    ("the watchdog is disabled (the last command keeps being honoured)",
     "safety",
     # ★★ 设备失联 — the failure that reads as success in a log.
     lambda s: s.replace(
         "        if self.accepted_at_s is not None and now_s - self.accepted_at_s > "
         "self.silence_s:",
         "        if False:  # MUTATION: silence is not a stop"),
     ["test_silence_falls_back_to_zero",
      "test_the_watchdog_output_is_exactly_zero_from_the_first_fire",
      "test_a_new_command_re_arms_the_gate",
      "test_the_summary_counts_watchdog_fires_apart_from_refusals",
      "test_every_mode_the_gate_can_report_is_declared"]),

    ("the watchdog ramps down instead of zeroing",
     "safety",
     # ★ A decay is a slower version of the same bug; `abs(v) < 0.1` would accept a creep.
     lambda s: s.replace(
         "            return 0.0, 0.0, 'SILENT', 'SILENT_SINCE_LAST_COMMAND'",
         "            return self.active['v'] * 0.01, 0.0, 'SILENT', "
         "'SILENT_SINCE_LAST_COMMAND'  # MUTATION: a creep, not a stop"),
     ["test_the_watchdog_output_is_exactly_zero_from_the_first_fire",
      "test_silence_falls_back_to_zero"]),

    ("the lease never expires",
     "safety",
     lambda s: s.replace(
         "        age = now_s - self.active['issued_s']\n"
         "        if age > self.active['ttl_s']:",
         "        age = now_s - self.active['issued_s']\n"
         "        if False:  # MUTATION: leases are forever"),
     ["test_the_lease_expires_independently_of_silence",
      "test_the_gate_lease_is_the_tighter_of_the_two",
      "test_every_mode_the_gate_can_report_is_declared"]),

    ("a confirmed stop becomes a latch that motion cannot clear",
     "safety",
     # ★ A different bug wearing the shape of safety: the gate could never be commanded again.
     lambda s: s.replace(
         "        self.stopped_confirmed = False\n"
         "        self.highest_seq = seq",
         "        self.highest_seq = seq  # MUTATION: the stop confirmation is a latch"),
     ["test_a_confirmed_stop_is_not_a_latch_forever"]),

    ("an unconfirmed boolean is accepted for the stop confirmation",
     "safety",
     lambda s: s.replace(
         "        if stopped_confirmed is not True and stopped_confirmed is not False:",
         "        if False:  # MUTATION: truthiness is enough"),
     ["test_confirm_stopped_requires_an_explicit_bool"]),

    ("the stop accepts a reason outside the contract list",
     "safety",
     # ★ 不要把所有失败写成 BUDGET_EXHAUSTED is only enforceable if the list is closed.
     lambda s: s.replace(
         "        if reason_code not in S.REASON_CODES:\n"
         "            raise SchemaRefused('REFUSED_BAD_ENUM', repr(reason_code), "
         "field='reason_code')",
         "        if False:  # MUTATION: any phrase will do\n"
         "            raise SchemaRefused('REFUSED_BAD_ENUM', repr(reason_code), "
         "field='reason_code')"),
     ["test_a_stop_with_an_undeclared_reason_is_refused"]),

    ("the audit stops checking that the mode agrees with the gate's own state",
     "safety",
     # ★★ This row used to target `if mode != 'HOLDING' and (v != 0.0 or w != 0.0):` -- a
     # guard that could never fire, because `step` returns a literal `0.0, 0.0` from every
     # non-HOLDING branch. The row survived because it was removing dead code, and that is what
     # sent me to look: a surviving mutation is either a test gap OR a dead guard. The guard has
     # been replaced by `_check_mode_agrees_with_state`, which is reachable and targeted here.
     lambda s: s.replace(
         "        live = self.active is not None\n"
         "        if mode == 'HOLDING' and not live:",
         "        live = self.active is not None\n"
         "        if False:  # MUTATION: the mode is not checked against the state"),
     ["test_the_audit_notices_a_mode_that_disagrees_with_the_gates_own_state"]),

    ("the audit stops checking the stopped-confirmed conflict",
     "safety",
     # ★ This guard moved into `_check_mode_agrees_with_state` when the audit was split so
     # its checks could be reached with a stated mode. The anchor is updated rather than the
     # expectation, because the property is unchanged -- only its address is.
     lambda s: s.replace(
         "        if self.stopped_confirmed and live:\n"
         "            raise SchemaRefused('REFUSED_CONFLICT',",
         "        if False:  # MUTATION\n"
         "            raise SchemaRefused('REFUSED_CONFLICT',"),
     ["test_the_audit_notices_a_mode_that_disagrees_with_the_gates_own_state",
      "test_the_audit_notices_a_mode_that_disagrees_with_the_gates_own_state"]),

    ("a watchdog fire is counted as a refusal",
     "safety",
     # ★ Two different events with different fixes; conflating them is how a healthy gate
     # looks like a misbehaving one.
     lambda s: s.replace(
         "            self.counters['watchdog_fired'] += 1",
         "            self.counters['watchdog_fired'] += 1\n"
         "            self.counters['refused'] += 1  # MUTATION"),
     ["test_the_summary_counts_watchdog_fires_apart_from_refusals"]),

    # ---- safety.py : the verdict ----------------------------------------
    ("a required check nobody ran is dropped instead of UNKNOWN",
     "safety",
     # ★ 缺项 UNKNOWN/NOT_RUN 则不能 ACCEPTED. A missing key would let a report
     # omit exactly the check that would have failed.
     lambda s: s.replace(
         "                if name not in self.checks:\n"
         "                    # \u2605 A required check nobody ran is UNKNOWN, not absent. "
         "Defaulting it to a\n",
         "                if False:  # MUTATION: an unrun check is simply absent\n"
         "                    # \u2605 A required check nobody ran is UNKNOWN, not absent. "
         "Defaulting it to a\n"),
     ["test_a_required_check_that_was_never_run_is_unknown_not_absent"]),

    ("the task outcome is aggregated from the check tri-state",
     "safety",
     # ★★ The exact defect this session fixed: a green run emitted the literal 'PASS'
     # into an outcome field.
     lambda s: s.replace(
         "        derived_outcome = self._aggregate_outcome(task_checks, worst='FAILED')",
         "        derived_outcome = self._aggregate(task_checks, "
         "worst='FAIL')  # MUTATION"),
     ["test_the_task_outcome_is_drawn_from_the_outcome_vocabulary_not_the_tri_state",
      "test_the_five_fields_are_separate",
      "test_accepted_needs_every_field_good_and_evidence_complete",
      "test_an_unknown_check_blocks_acceptance",
      "test_a_required_check_that_was_never_run_is_unknown_not_absent",
      "test_incomplete_evidence_blocks_acceptance_and_is_countable",
      "test_the_five_fields_are_exactly_what_the_validator_declares",
      "test_a_correct_stop_is_a_safety_pass_and_a_task_failure"]),

    ("an all-green task group is not mapped to SUCCEEDED",
     "safety",
     lambda s: s.replace(
         "        if all(v == 'PASS' for v in values):\n"
         "            return 'SUCCEEDED'",
         "        if all(v == 'PASS' for v in values):\n"
         "            return 'FAILED'  # MUTATION"),
     ["test_the_task_outcome_is_drawn_from_the_outcome_vocabulary_not_the_tri_state",
      "test_the_five_fields_are_separate",
      "test_accepted_needs_every_field_good_and_evidence_complete"]),

    ("an UNKNOWN task check is upgraded to a FAILED instead of needing attention",
     "safety",
     lambda s: s.replace(
         "        if any(v == 'UNKNOWN' for v in values):\n"
         "            return 'NEEDS_ATTENTION'\n"
         "        if all(v == 'PASS' for v in values):\n"
         "            return 'SUCCEEDED'\n"
         "        return worst",
         "        if all(v == 'PASS' for v in values):\n"
         "            return 'SUCCEEDED'\n"
         "        return worst  # MUTATION: an unknown is treated as a failure"),
     ["test_the_task_outcome_is_drawn_from_the_outcome_vocabulary_not_the_tri_state",
      "test_a_never_run_task_check_cannot_be_declared_succeeded"]),

    ("a delivery may be declared SUCCEEDED the checks do not support",
     "safety",
     # ★ The delivery is the checks, not an assertion about them. The condition is inverted
     # rather than the branch deleted, so the module still compiles and the *semantics* are what
     # the tests have to notice -- a syntax error would make every row look caught.
     lambda s: s.replace(
         "            if task_outcome == 'SUCCEEDED' and derived_outcome != 'SUCCEEDED':",
         "            if task_outcome == 'SUCCEEDED' and derived_outcome == 'SUCCEEDED':  "
         "# MUTATION: the guard is inverted"),
     ["test_a_fault_scenario_cannot_be_declared_succeeded",
      "test_a_declared_delivery_needs_the_task_checks_to_back_it_whatever_they_say",
      "test_the_task_outcome_is_drawn_from_the_outcome_vocabulary_not_the_tri_state"]),

    ("the five fields collapse into one aggregate",
     "safety",
     # ★ The whole clause: collapsing them is how "stopped correctly" becomes "delivered".
     lambda s: s.replace(
         "            'physical_result': physical,\n"
         "            'safety_result': safety,\n"
         "            'expected_behavior': expected,",
         "            'physical_result': safety,\n"
         "            'safety_result': safety,\n"
         "            'expected_behavior': safety,  # MUTATION"),
     ["test_a_correct_stop_is_a_safety_pass_and_a_task_failure",
      "test_an_unknown_check_blocks_acceptance",
      "test_a_required_check_that_was_never_run_is_unknown_not_absent"]),

    ("a safety_result of UNKNOWN no longer blocks acceptance",
     "safety",
     lambda s: s.replace(
         "        safety = self._aggregate(safety_checks, worst='FAIL')",
         "        safety = self._aggregate(safety_checks, "
         "worst='FAIL').replace('UNKNOWN', 'PASS')  # MUTATION"),
     ["test_an_unknown_check_blocks_acceptance"]),

    ("incomplete evidence is reported as complete",
     "safety",
     # ★ Otherwise "we did not look" and "fine" are the same report.
     lambda s: s.replace(
         "        complete = not self.missing",
         "        complete = True  # MUTATION: missing evidence is not missing"),
     ["test_incomplete_evidence_blocks_acceptance_and_is_countable"]),

    ("a declared gap is no longer listed",
     "safety",
     lambda s: s.replace(
         "            'missing': list(self.missing),",
         "            'missing': [],  # MUTATION"),
     ["test_incomplete_evidence_blocks_acceptance_and_is_countable"]),

    ("a later refusal overwrites the first one",
     "safety",
     # ★ The latest refusal is usually a downstream symptom; the first is the cause.
     lambda s: s.replace(
         "            if self.first_refusal is None:\n"
         "                self.first_refusal = first_refusal",
         "            self.first_refusal = first_refusal  # MUTATION: last wins"),
     ["test_the_three_annotations_are_recorded_and_the_first_refusal_sticks"]),

    ("a reason outside the contract list is accepted on the verdict",
     "safety",
     lambda s: s.replace(
         "        if reason_code is not None:\n"
         "            if reason_code not in S.REASON_CODES:\n"
         "                raise SchemaRefused('REFUSED_BAD_ENUM', repr(reason_code), "
         "field='reason_code')",
         "        if reason_code is not None and False:  # MUTATION\n"
         "            if reason_code not in S.REASON_CODES:\n"
         "                raise SchemaRefused('REFUSED_BAD_ENUM', repr(reason_code), "
         "field='reason_code')"),
     ["test_a_reason_outside_the_contract_list_is_refused"]),

    ("a check result outside the declared tri-state is accepted",
     "safety",
     lambda s: s.replace(
         "        if result not in ('PASS', 'FAIL', 'UNKNOWN', 'NOT_APPLICABLE'):\n"
         "            raise SchemaRefused('REFUSED_BAD_ENUM', repr(result), "
         "field=f'checks.{name}')",
         "        if False:  # MUTATION: any string is a result\n"
         "            raise SchemaRefused('REFUSED_BAD_ENUM', repr(result), "
         "f'checks.{name}')"),
     ["test_a_check_result_must_be_from_the_declared_tri_state"]),

    ("the fault-scenario helper reports the stop as the delivery",
     "safety",
     # ★ 故障测试可以"正确停车"为预期通过，但不能计为配送完成.
     lambda s: s.replace(
         "        task_outcome='FAILED',",
         "        task_outcome='SUCCEEDED',  # MUTATION: a good stop is a delivery"),
     ["test_a_correct_stop_is_a_safety_pass_and_a_task_failure"]),

    ("the builder stops calling validate_evaluation (redundant guard)",
     "safety",
     # ★ This one removes a guard that is redundant *by construction*: `build()` validates
     # `task_outcome` and every result value at the point they enter, and the dict it hands to
     # `validate_evaluation` is built from already-validated locals. So skipping the call cannot
     # be observed from outside, for the same reason the guard-side `min()` above could not.
     # Declared rather than deleted: a row that fails loudly would be a lie, and deleting it
     # would lose the record that the second validation is defence in depth, not load-bearing.
     # `MULTI_OK`-style: this row is expected to SURVIVE, and the harness must say so.
     lambda s: s.replace(
         "        validated = S.validate_evaluation(raw)",
         "        validated = raw  # MUTATION: the second validation is skipped"),
     []),  # <- empty: surviving is the correct result, asserted by REDUNDANT below
]


def parse_failed(out):
    """(number failed, set of test names) from pytest -q output.

    The name extraction is unit-tested below, because the first version of this harness had a
    regex that captured the class name instead of the test name: the suite really was failing
    but the harness reported "nothing matching failed" for every row, which looks exactly like
    "the suite has gaps" and would have sent me hunting in the wrong place.
    """
    m = re.search(r'(\d+) failed', out)
    n = int(m.group(1)) if m else 0
    return n, set(re.findall(r'FAILED tests/\S+::(\w+)', out))


_SAMPLE = (
    "FAILED tests/test_p2_schema.py::TestOrderRefusals::test_unknown_top_level_field - Assertion\n"
    "FAILED tests/test_p2_ledger.py::TestCancellation::test_an_unconfirmed_stop_becomes_attention_not_a_cancellation\n"
    "2 failed, 134 passed in 0.3s\n"
)
_n, _names = parse_failed(_SAMPLE)
assert _n == 2, f'parse_failed missed the count: {_n}'
assert _names == {'test_unknown_top_level_field',
                  'test_an_unconfirmed_stop_becomes_attention_not_a_cancellation'}, \
    f'parse_failed extracted {_names}'
assert 'TestOrderRefusals' not in _names, 'parse_failed captured a class name as a test name'


def run_tests():
    """Run the core suite against whatever is currently on disk.

    `PYTHONDONTWRITEBYTECODE` is set because the harness rewrites the module between runs: a
    stale `.pyc` keyed on mtime+size can be reused, and the mutation then runs against the
    *previous* mutation's bytecode. That is exactly how this harness first misbehaved -- one
    row reported the previous row's failures, which read as "this mutation was not caught".
    """
    env = dict(os.environ)
    env["PYTHONDONTWRITEBYTECODE"] = "1"
    p = subprocess.run(
        [os.path.join(ROOT, ".venv/bin/python"), "-B", "-m", "pytest"] + TEST_FILES +
        ["-q", "--no-header", "-p", "no:cacheprovider"],
        cwd=ROOT, env=env, capture_output=True, text=True)
    return p.stdout + p.stderr


def write_source(path, text):
    """Write and prove it landed: a mutation that did not reach disk proves nothing."""
    with open(path, "w", encoding="utf-8") as handle:
        handle.write(text)
        handle.flush()
        os.fsync(handle.fileno())
    on_disk = open(path, encoding="utf-8").read()
    assert on_disk == text, f'{path} does not match what was written'
    # Drop the bytecode cache for this module, so the next interpreter starts clean.
    cache = os.path.join(os.path.dirname(path), "__pycache__")
    if os.path.isdir(cache):
        stem = os.path.basename(path).split('.')[0]
        for name in os.listdir(cache):
            if name.startswith(stem):
                os.remove(os.path.join(cache, name))


originals = {path: open(path, encoding="utf-8").read()
             for path in (SCHEMA, LEDGER, RESOURCES, TRANSFER, SAFETY)}

n_checked = check_anchors_unique()
print(f"anchor guard: {n_checked} mutation anchors each occur exactly once")
print()

results = []
try:
    for label, which, mutate, expect in MUTATIONS:
        target = _path(which)
        base = originals[target]
        mutated = mutate(base)
        if mutated == base:
            results.append((label, "COULD NOT APPLY", expect, 0, 0))
            continue
        write_source(target, mutated)
        # Prove the mutation is the thing on disk before asking the suite about it.
        assert open(target, encoding="utf-8").read() == mutated, \
            f'{label}: mutation did not survive the write'
        out = run_tests()
        n_failed, names = parse_failed(out)
        hit = sorted(names & set(expect))
        if not expect and not hit and n_failed == 0:
            # ★ An empty expectation is a *declaration* that this mutation cannot change
            # observable behaviour, and the row is required to survive. The alternative --
            # deleting the row -- would lose the record that the guard is redundant rather than
            # load-bearing, which is itself a finding worth keeping. Anything other than a clean
            # survival is reported as a failure, so the declaration cannot be used to hide a gap.
            status = "REDUNDANT (declared)"
        else:
            status = "CAUGHT" if hit else ("NO TEST FAILED" if n_failed == 0 else "WRONG TESTS")
        # A CAUGHT row must not be riding on a leftover mutation: restore, then require the
        # suite to be green before the next row is applied. Without this, a row can report the
        # previous row's failures as its own -- which is how this harness first misled me.
        write_source(target, base)
        check = run_tests()
        clean_n, _ = parse_failed(check)
        results.append((label, status, hit, n_failed, clean_n))
finally:
    for path, text in originals.items():
        write_source(path, text)

print("=" * 80)
print("MUTATION PROOF -- every row must read CAUGHT or REDUNDANT (declared)")
print("=" * 80)
ok = True
for label, status, hit, n_failed, clean_n in results:
    ok_status = status in ("CAUGHT", "REDUNDANT (declared)")
    mark = "ok " if ok_status else "!! "
    print(f"{mark}[{status}] {label}")
    if status == "REDUNDANT (declared)":
        if clean_n != 0:
            print(f"          !! the suite was not clean before this row ({clean_n} failed)")
            ok = False
    elif status == "CAUGHT":
        print(f"          failed {n_failed} test(s), named: {hit}")
        if clean_n != 0:
            # The suite must be green again after restoring, or the next row would be judged
            # against a leftover mutation.
            print(f"          !! suite was NOT clean after restore ({clean_n} failed)")
            ok = False
    else:
        print(f"          expected one of: {hit or 'n/a'}; nothing matching failed")
        ok = False
print("=" * 80)
caught = sum(1 for r in results if r[1] == 'CAUGHT')
redundant = sum(1 for r in results if r[1] == 'REDUNDANT (declared)')
print(f"{caught}/{len(results)} mutations caught, {redundant} declared redundant")
assert caught + redundant == len(results), 'a mutation survived: the suite has a gap'

for path, text in originals.items():
    assert open(path, encoding="utf-8").read() == text, f"{path} was not restored!"
print("all five modules restored byte-for-byte; final suite check:")
print(run_tests().strip().splitlines()[-1])
sys.exit(0 if ok else 1)
