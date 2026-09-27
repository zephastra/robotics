"""Tests for the P2 schema layer.

The rule this file is written to satisfy is hard lesson 2: **a check that cannot be shown to
fail is not a check.** So for every refusal reason `schema.py` can raise, there is an input
here that must produce exactly that reason -- and for the accepting paths, there is an input
that must produce a refusal when one field is perturbed. A validator tested only with valid
input proves nothing about what it rejects.
"""
import json
import pathlib
import sys

import pytest

# Same convention as the rest of the suite: resolve src/ from the project root rather than
# relying on PYTHONPATH, so `pytest tests/` works in a clean shell.
ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src'))

from workcell import schema as S  # noqa: E402


# ---------------------------------------------------------------------------
# helpers
# ---------------------------------------------------------------------------

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
    """Assert `fn` refuses with exactly `reason`, and return the exception."""
    with pytest.raises(S.SchemaRefused) as caught:
        fn(*args, **kwargs)
    assert caught.value.reason == reason, \
        f'expected {reason!r}, got {caught.value.reason!r} ({caught.value})'
    return caught.value


# ---------------------------------------------------------------------------
# 1. the happy path, so the refusals below mean something
# ---------------------------------------------------------------------------

class TestOrderAccepts:

    def test_the_contract_example_validates(self):
        """The example in docs/CONTRACTS.md section 2 is the canonical accepted input."""
        raw = {
            'schema_version': 1,
            'request_id': 'request-001',
            'order_id': 'order-001',
            'destination_id': 'station_b',
            'items': [{'type': 'red_block', 'count': 2},
                      {'type': 'blue_cylinder', 'count': 1}],
            'priority': 10,
        }
        out = S.validate_order(raw)
        assert out['request_id'] == 'request-001'
        assert out['destination_id'] == 'station_b'
        assert out['items'] == [{'type': 'red_block', 'count': 2},
                                {'type': 'blue_cylinder', 'count': 1}]
        assert out['priority'] == 10

    def test_it_accepts_a_json_string_and_bytes(self):
        """The ledger reads from a file or a socket, not always from a dict."""
        text = json.dumps(order())
        assert S.validate_order(text)['order_id'] == 'order-001'
        assert S.validate_order(text.encode())['order_id'] == 'order-001'

    def test_it_copies_rather_than_aliases_the_caller_item_list(self):
        raw = order()
        out = S.validate_order(raw)
        out['items'].append({'type': 'red_block', 'count': 5})
        assert len(raw['items']) == 1, 'the caller\'s payload was mutated through the result'


# ---------------------------------------------------------------------------
# 2. every refusal reason the order path can raise, with an input that raises it
# ---------------------------------------------------------------------------

class TestOrderRefusals:

    def test_unknown_top_level_field(self):
        e = refuses('REFUSED_UNKNOWN_FIELD', S.validate_order, order(urgency='high'))
        assert 'urgency' in e.detail

    def test_missing_field(self):
        raw = order()
        del raw['priority']
        refuses('REFUSED_MISSING_FIELD', S.validate_order, raw)

    def test_duplicate_key_is_not_last_wins(self):
        """`{"count": 2, "count": 1}` has two readings; stdlib silently picks the second."""
        e = refuses('REFUSED_DUPLICATE_KEY', S.validate_order,
                    '{"schema_version":1,"request_id":"request-001","order_id":"order-001",'
                    '"destination_id":"station_b",'
                    '"items":[{"type":"red_block","count":2,"count":1}],"priority":10}')
        assert e.detail == 'count'

    def test_wrong_schema_version(self):
        refuses('REFUSED_SCHEMA_VERSION', S.validate_order, order(schema_version=2))
        refuses('REFUSED_SCHEMA_VERSION', S.validate_order, order(schema_version=True))

    def test_not_json(self):
        refuses('REFUSED_NOT_JSON', S.validate_order, '{not json')

    def test_not_an_object(self):
        refuses('REFUSED_NOT_OBJECT', S.validate_order, '[1, 2, 3]')

    def test_not_utf8(self):
        refuses('REFUSED_NOT_JSON', S.validate_order, b'\xff\xfe\x00')

    def test_bad_id_shape(self):
        refuses('REFUSED_BAD_ID', S.validate_order, order(order_id='has spaces'))
        refuses('REFUSED_BAD_ID', S.validate_order, order(order_id='-leading-dash'))
        refuses('REFUSED_BAD_ID', S.validate_order, order(order_id=''))
        refuses('REFUSED_BAD_ID', S.validate_order, order(request_id='x' * 65))

    def test_destination_must_be_registered(self):
        """A free-form destination is how a typo becomes a station that waits forever."""
        e = refuses('REFUSED_ENTITY_UNKNOWN', S.validate_order,
                    order(destination_id='station_z'))
        assert 'station_z' in e.detail

    def test_items_must_be_a_list(self):
        refuses('REFUSED_BAD_TYPE', S.validate_order, order(items={'red_block': 2}))

    def test_items_must_not_be_empty(self):
        refuses('REFUSED_OUT_OF_RANGE', S.validate_order, order(items=[]))

    def test_a_duplicated_row_is_refused_before_the_line_cap_can_hide_it(self):
        """The line cap and the duplicate rule interact: with only five registered part types,
        MAX_ITEMS_PER_ORDER (16) is unreachable via legal distinct rows. So the enforceable
        statements are the duplicate rule and the total-count budget, and this test says so
        rather than pretending the line cap is exercised."""
        many = [{'type': 'red_block', 'count': S.MAX_ITEM_COUNT} for _ in range(2)]
        refuses('REFUSED_BAD_COUNT', S.validate_order, order(items=many))

    def test_duplicate_type_in_two_lines_is_refused(self):
        """Two lines for one type have two readings: "2 total" or "2+3". Refuse, do not guess."""
        e = refuses('REFUSED_BAD_COUNT', S.validate_order,
                    order(items=[{'type': 'red_block', 'count': 2},
                                 {'type': 'red_block', 'count': 3}]))
        assert 'red_block' in e.detail

    def test_unknown_part_type(self):
        refuses('REFUSED_BAD_ENUM', S.validate_order,
                order(items=[{'type': 'purple_pyramid', 'count': 1}]))

    def test_count_must_be_an_int_in_range(self):
        refuses('REFUSED_BAD_TYPE', S.validate_order,
                order(items=[{'type': 'red_block', 'count': '2'}]))
        refuses('REFUSED_BAD_TYPE', S.validate_order,
                order(items=[{'type': 'red_block', 'count': 2.0}]))
        refuses('REFUSED_OUT_OF_RANGE', S.validate_order,
                order(items=[{'type': 'red_block', 'count': 0}]))
        refuses('REFUSED_OUT_OF_RANGE', S.validate_order,
                order(items=[{'type': 'red_block', 'count': S.MAX_ITEM_COUNT + 1}]))

    def test_true_is_not_one(self):
        """★ Python makes `True == 1`, so `"count": true` would otherwise pass as one item."""
        refuses('REFUSED_BAD_TYPE', S.validate_order,
                order(items=[{'type': 'red_block', 'count': True}]))

    def test_nan_and_infinity(self):
        """★ `json.loads` accepts these by default, and NaN compares false against itself,
        which would make every later `>= 0` guard take the else branch silently.

        The reason must be NON_FINITE and not BAD_TYPE: a NaN priority is a float where an int
        was wanted, so a caller told "expected an int" would go looking for a JSON formatting
        bug rather than a not-a-number.
        """
        refuses('REFUSED_NON_FINITE', S.validate_order,
                order(items=[{'type': 'red_block', 'count': 1}], priority=float('nan')))
        refuses('REFUSED_NON_FINITE', S.validate_order, order(priority=float('inf')))
        refuses('REFUSED_NON_FINITE', S.validate_order,
                '{"schema_version":1,"request_id":"request-001","order_id":"order-001",'
                '"destination_id":"station_b","items":[{"type":"red_block","count":1}],'
                '"priority":NaN}')

    def test_a_non_finite_float_is_never_read_as_a_valid_int(self):
        """The same rule on a different field, so the fix is general and not a special case
        carved out for `priority`."""
        refuses('REFUSED_NON_FINITE', S.validate_skill_request,
                {'schema_version': 1, 'command_id': 'c1', 'order_id': 'order-001',
                 'revision': float('nan'), 'expected_boot_id': 'boot-001', 'skill': 'STOP',
                 'arguments': {}, 'resource_generation': 0, 'deadline_s': 1.0})
        refuses('REFUSED_NON_FINITE', S.validate_device_state,
                {'schema_version': 1, 'device_id': 'amr_1', 'boot_id': 'boot-001',
                 'sequence': float('inf'), 'sim_stamp': 0.0, 'capabilities': ['OBSERVE'],
                 'operating_state': 'IDLE', 'active_command': None,
                 'pose': {'frame': 'map', 'x': 0, 'y': 0, 'z': 0,
                          'quaternion': [0.0, 0.0, 0.0, 1.0]},
                 'observation_age_s': 0.0, 'stopped_confirmed': True,
                 'held_payload_id': None, 'resources': [], 'fault': None, 'battery': None})

    def test_priority_range_and_type(self):
        refuses('REFUSED_OUT_OF_RANGE', S.validate_order, order(priority=-1))
        refuses('REFUSED_OUT_OF_RANGE', S.validate_order, order(priority=S.MAX_PRIORITY + 1))
        refuses('REFUSED_BAD_TYPE', S.validate_order, order(priority='10'))

    def test_item_unknown_field(self):
        refuses('REFUSED_UNKNOWN_FIELD', S.validate_order,
                order(items=[{'type': 'red_block', 'count': 1, 'color': 'red'}]))

    def test_total_part_budget(self):
        """The per-line cap is 99, so the only way to the total cap is more lines."""
        # 5 types x 99 = 495 > 256, so a full house must be refused on the total.
        items = [{'type': t, 'count': S.MAX_ITEM_COUNT} for t in sorted(S.PART_TYPES)]
        refuses('REFUSED_TOO_MANY_ITEMS', S.validate_order, order(items=items))

    def test_the_whole_part_type_vocabulary_is_accepted_one_at_a_time(self):
        """Guards against a typo in PART_TYPES: every declared type must actually validate."""
        for t in sorted(S.PART_TYPES):
            assert S.validate_order(order(items=[{'type': t, 'count': 1}]))['items'][0]['type'] == t


# ---------------------------------------------------------------------------
# 3. the fingerprint, which is what makes "same request, same content" checkable
# ---------------------------------------------------------------------------

class TestContentFingerprint:

    def test_key_order_does_not_change_it(self):
        a = S.validate_order(order())
        b = S.validate_order(json.loads(json.dumps(order())))
        assert S.content_fingerprint(a) == S.content_fingerprint(b)

    def test_a_changed_count_changes_it(self):
        a = S.validate_order(order())
        b = S.validate_order(order(items=[{'type': 'red_block', 'count': 3}]))
        assert S.content_fingerprint(a) != S.content_fingerprint(b)

    def test_a_changed_destination_changes_it(self):
        a = S.validate_order(order())
        b = S.validate_order(order(destination_id='station_c'))
        assert S.content_fingerprint(a) != S.content_fingerprint(b)

    def test_every_validated_field_enters_the_fingerprint(self):
        """★ The structural half of the typed-constant lesson.

        The fingerprint is computed over the whole normalised dict, so a field added to
        `validate_order` later is covered automatically. This test proves the *current* set is
        covered by perturbing each field in turn and requiring a different fingerprint -- if a
        field were forgotten, its perturbation would look like 'same content'.
        """
        base = S.validate_order(order())
        perturbations = {
            'request_id': order(request_id='request-002'),
            'order_id': order(order_id='order-002'),
            'destination_id': order(destination_id='station_c'),
            'items': order(items=[{'type': 'red_block', 'count': 9}]),
            'priority': order(priority=11),
        }
        baseline = S.content_fingerprint(base)
        for field, mutated in perturbations.items():
            other = S.validate_order(mutated)
            assert S.content_fingerprint(other) != baseline, \
                f'changing {field!r} did not change the fingerprint'
        # and the covered set is the whole normalised dict, not a hand-written list
        assert set(base) == {'schema_version', 'request_id', 'order_id',
                             'destination_id', 'items', 'priority'}


# ---------------------------------------------------------------------------
# 4. pose: the quaternion-order trap
# ---------------------------------------------------------------------------

class TestPose:

    def test_a_unit_quaternion_is_accepted_and_the_order_is_declared(self):
        out = S.validate_pose({'frame': 'map', 'x': 1.0, 'y': 2.0, 'z': 0.0,
                               'quaternion': [0.0, 0.0, 0.0, 1.0]})
        assert out['quaternion_order'] == 'xyzw'

    def test_a_non_unit_quaternion_is_refused(self):
        """xyzw vs wxyz is silent: the wrong order is still a unit quaternion."""
        refuses('REFUSED_BAD_QUATERNION', S.validate_pose,
                {'frame': 'map', 'x': 0, 'y': 0, 'z': 0, 'quaternion': [0.0, 0.0, 0.0, 0.5]})
        refuses('REFUSED_BAD_QUATERNION', S.validate_pose,
                {'frame': 'map', 'x': 0, 'y': 0, 'z': 0, 'quaternion': [0.0, 0.0, 0.0]})

    def test_a_bad_norm_is_reported_as_a_bad_quaternion_even_with_a_nan_component(self):
        """★ The third fix this suite forced. A NaN component made the first version report
        NON_FINITE, but the caller's problem is that this field is not a rotation; the more
        specific reason has to win or the message points at the wrong thing."""
        refuses('REFUSED_BAD_QUATERNION', S.validate_pose,
                {'frame': 'map', 'x': 0, 'y': 0, 'z': 0,
                 'quaternion': [0.0, 0.0, 0.0, float('nan')]})
        refuses('REFUSED_BAD_QUATERNION', S.validate_pose,
                {'frame': 'map', 'x': 0, 'y': 0, 'z': 0,
                 'quaternion': [0.0, 0.0, 0.0, float('inf')]})
        refuses('REFUSED_BAD_QUATERNION', S.validate_pose,
                {'frame': 'map', 'x': 0, 'y': 0, 'z': 0,
                 'quaternion': [0.0, 0.0, 0.0, True]})

    def test_unknown_frame_is_refused(self):
        refuses('REFUSED_BAD_ENUM', S.validate_pose,
                {'frame': 'world_map', 'x': 0, 'y': 0, 'z': 0,
                 'quaternion': [0.0, 0.0, 0.0, 1.0]})

    def test_a_non_finite_coordinate_is_refused(self):
        """★ Covers `_finite` directly, on a scalar coordinate.

        Found by mutation testing: with the NaN guard removed from `_finite`, the whole suite
        still passed, because at the time every NaN test happened to reach a *different* guard
        (`_int_in`'s, or the quaternion's). The scalar path through `_number_in` -> `_finite`
        was untested, so disabling it changed nothing observable. This test closes that.
        """
        base = {'frame': 'map', 'x': 0.0, 'y': 0.0, 'z': 0.0,
                'quaternion': [0.0, 0.0, 0.0, 1.0]}
        for axis in ('x', 'y', 'z'):
            for value in (float('nan'), float('inf'), float('-inf')):
                bad = dict(base, **{axis: value})
                refuses('REFUSED_NON_FINITE', S.validate_pose, bad)

    def test_a_coordinate_must_be_a_number_not_a_string_or_bool(self):
        base = {'frame': 'map', 'x': 0.0, 'y': 0.0, 'z': 0.0,
                'quaternion': [0.0, 0.0, 0.0, 1.0]}
        refuses('REFUSED_BAD_TYPE', S.validate_pose, dict(base, x='0.0'))
        refuses('REFUSED_BAD_TYPE', S.validate_pose, dict(base, y=True))

    def test_a_coordinate_beyond_the_sane_bound_is_refused(self):
        base = {'frame': 'map', 'x': 0.0, 'y': 0.0, 'z': 0.0,
                'quaternion': [0.0, 0.0, 0.0, 1.0]}
        refuses('REFUSED_OUT_OF_RANGE', S.validate_pose,
                dict(base, x=S.MAX_DISTANCE_M + 1.0))

    def test_a_pose_cannot_hide_an_extra_field(self):
        refuses('REFUSED_UNKNOWN_FIELD', S.validate_pose,
                {'frame': 'map', 'x': 0, 'y': 0, 'z': 0,
                 'quaternion': [0.0, 0.0, 0.0, 1.0], 'quaternion_order': 'xyzw'})


# ---------------------------------------------------------------------------
# 5. DeviceState: the conditional battery requirement
# ---------------------------------------------------------------------------

class TestDeviceState:

    def state(self, **over):
        base = {
            'schema_version': 1,
            'device_id': 'amr_1',
            'boot_id': 'boot-001',
            'sequence': 42,
            'sim_stamp': 12.5,
            'capabilities': ['OBSERVE', 'MOVE_TO_STATION', 'DOCK'],
            'operating_state': 'IDLE',
            'active_command': None,
            'pose': {'frame': 'map', 'x': 1.0, 'y': 2.0, 'z': 0.0,
                     'quaternion': [0.0, 0.0, 0.0, 1.0]},
            'observation_age_s': 0.05,
            'stopped_confirmed': True,
            'held_payload_id': None,
            'resources': [{'resource_id': 'lane_north', 'state': 'FREE'}],
            'fault': None,
            'battery': None,
        }
        base.update(over)
        return base

    def test_a_vehicle_without_the_capability_needs_no_reading(self):
        out = S.validate_device_state(self.state())
        assert out['battery'] is None

    def test_declaring_the_capability_without_a_reading_is_refused(self):
        """★ 'battery（适用时）' is a conditional requirement. Ignoring the condition accepts a
        vehicle with no battery field, which reads downstream as 'battery fine'."""
        refuses('REFUSED_MISSING_FIELD', S.validate_device_state,
                self.state(capabilities=['OBSERVE', 'battery']))

    def test_declaring_it_wth_a_reading_is_accepted(self):
        out = S.validate_device_state(self.state(capabilities=['OBSERVE', 'battery'],
                                                battery=0.42))
        assert out['battery'] == pytest.approx(0.42)

    def test_battery_out_of_range(self):
        refuses('REFUSED_OUT_OF_RANGE', S.validate_device_state,
                self.state(capabilities=['battery'], battery=1.5))

    def test_a_device_capability_is_not_a_skill(self):
        """★ The fix this test forced: `capabilities` lists what a device *is* as well as what
        it can be *asked to do*, so validating it against SKILLS alone refused every vehicle
        that declared a battery."""
        assert 'battery' not in S.SKILLS
        assert 'battery' in S.CAPABILITIES
        assert S.SKILLS <= S.CAPABILITIES
        out = S.validate_device_state(self.state(capabilities=['battery', 'cargo_hold'],
                                                battery=0.5))
        assert out['battery'] == pytest.approx(0.5)

    def test_a_capability_that_is_neither_skill_nor_resource_is_refused(self):
        refuses('REFUSED_BAD_ENUM', S.validate_device_state,
                self.state(capabilities=['TELEPORT']))

    def test_stopped_confirmed_must_be_a_bool(self):
        refuses('REFUSED_BAD_TYPE', S.validate_device_state,
                self.state(stopped_confirmed=1))

    def test_an_unknown_resource_state_is_refused(self):
        """The default is UNKNOWN and it must be spelled; there is no implied default here."""
        refuses('REFUSED_BAD_ENUM', S.validate_device_state,
                self.state(resources=[{'resource_id': 'lane_north', 'state': 'IDLE'}]))

    def test_an_unlisted_capability_is_refused(self):
        """★ RETURN_TO_CHARGE is reserved and NOT registered, so a device claiming it is
        claiming a capability nobody implemented."""
        refuses('REFUSED_BAD_ENUM', S.validate_device_state,
                self.state(capabilities=['OBSERVE', 'RETURN_TO_CHARGE']))

    def test_a_fault_must_be_a_known_reason(self):
        refuses('REFUSED_BAD_ENUM', S.validate_device_state,
                self.state(fault='SOMETHING_WEIRD'))

    def test_sequence_must_be_a_non_negative_int(self):
        refuses('REFUSED_BAD_TYPE', S.validate_device_state, self.state(sequence=1.0))
        refuses('REFUSED_OUT_OF_RANGE', S.validate_device_state, self.state(sequence=-1))
        refuses('REFUSED_BAD_TYPE', S.validate_device_state, self.state(sequence=True))

    def test_operating_state_is_an_enum(self):
        refuses('REFUSED_BAD_ENUM', S.validate_device_state,
                self.state(operating_state='RUNNING'))


# ---------------------------------------------------------------------------
# 6. SkillRequest: the whitelist and the forbidden-argument wall
# ---------------------------------------------------------------------------

class TestSkillRequest:

    def req(self, skill, arguments, **over):
        base = {
            'schema_version': 1,
            'command_id': 'cmd-0001',
            'order_id': 'order-001',
            'revision': 3,
            'expected_boot_id': 'boot-001',
            'skill': skill,
            'arguments': arguments,
            'resource_generation': 7,
            'deadline_s': 30.0,
        }
        base.update(over)
        return base

    def test_every_whitelisted_skill_validates_with_its_declared_arguments(self):
        """★ Guards the whole table at once: a skill added without REQUIRED_SKILL_ARGUMENTS
        would KeyError here rather than silently accept anything."""
        samples = {
            'OBSERVE': {},
            'PRESENT_TRAY': {'tray_id': 'tray_v1_0'},
            'PICK_PART': {'part_id': 'part_0001'},
            'PLACE_PART': {'part_id': 'part_0001', 'slot_id': 'slot_1'},
            'VERIFY_KIT': {'tray_id': 'tray_v1_0',
                           'expected_bom': [{'type': 'red_block', 'count': 2}]},
            'MOVE_TO_STATION': {'station_id': 'station_b'},
            'DOCK': {'station_id': 'station_b', 'transfer_id': 'xfer-001'},
            'START_TRANSFER': {'transfer_id': 'xfer-001'},
            'VERIFY_TRANSFER': {'transfer_id': 'xfer-001'},
            'UNDOCK': {'station_id': 'station_b'},
            'VERIFY_DELIVERY': {'order_id': 'order-001'},
            'STOP': {},
        }
        assert set(samples) == set(S.SKILLS), 'a whitelisted skill has no sample here'
        assert set(samples) == set(S.REQUIRED_SKILL_ARGUMENTS), \
            'REQUIRED_SKILL_ARGUMENTS and this sample table disagree'
        for skill, args in samples.items():
            out = S.validate_skill_request(self.req(skill, args))
            assert out['skill'] == skill

    def test_reserved_skills_are_refused_with_a_distinct_message(self):
        e = refuses('REFUSED_BAD_ENUM', S.validate_skill_request,
                    self.req('RETURN_TO_CHARGE', {}))
        assert 'reserved' in e.detail

    def test_an_unknown_skill_is_refused(self):
        refuses('REFUSED_BAD_ENUM', S.validate_skill_request, self.req('TELEPORT', {}))

    def test_control_quantities_are_refused_by_name(self):
        """★ The contract says a user may not supply thresholds, joint targets, shell commands
        or simulation coordinates. Here that is enforced by field name, so it cannot be
        forgotten for a newly added skill."""
        for name, value in [('joint_targets', [0.1] * 7), ('shell', 'rm -rf /'),
                            ('thresholds', {'dock': 0.01}), ('world_pose', [1, 2, 3]),
                            ('ctrl', [0.0]), ('exec', 'code')]:
            e = refuses('REFUSED_FORBIDDEN_FIELD', S.validate_skill_request,
                        self.req('MOVE_TO_STATION', {'station_id': 'station_b', name: value}))
            assert name in e.detail

    def test_a_misspelled_optional_argument_is_a_plain_unknown_field(self):
        """Distinct from the forbidden case above: a typo is countable apart from an attempt."""
        refuses('REFUSED_UNKNOWN_FIELD', S.validate_skill_request,
                self.req('MOVE_TO_STATION', {'station_id': 'station_b', 'stationid': 'x'}))

    def test_a_required_argument_missing_is_refused_not_defaulted(self):
        """★ An absent station_id filled in from a default is a command sent somewhere the
        caller never named."""
        refuses('REFUSED_MISSING_FIELD', S.validate_skill_request,
                self.req('MOVE_TO_STATION', {}))
        refuses('REFUSED_MISSING_FIELD', S.validate_skill_request,
                self.req('PLACE_PART', {'part_id': 'part_0001'}))

    def test_a_skill_cannot_carry_another_skills_argument(self):
        refuses('REFUSED_UNKNOWN_FIELD', S.validate_skill_request,
                self.req('PICK_PART', {'part_id': 'part_0001', 'station_id': 'station_b'}))

    def test_station_and_slot_must_be_registered(self):
        refuses('REFUSED_BAD_ENUM', S.validate_skill_request,
                self.req('MOVE_TO_STATION', {'station_id': 'station_z'}))
        refuses('REFUSED_BAD_ENUM', S.validate_skill_request,
                self.req('PLACE_PART', {'part_id': 'p1', 'slot_id': 'slot_9'}))

    def test_expected_bom_rejects_duplicates_and_bad_counts(self):
        refuses('REFUSED_BAD_COUNT', S.validate_skill_request,
                self.req('VERIFY_KIT', {'tray_id': 'tray_v1_0',
                                        'expected_bom': [{'type': 'red_block', 'count': 1},
                                                         {'type': 'red_block', 'count': 2}]}))
        refuses('REFUSED_OUT_OF_RANGE', S.validate_skill_request,
                self.req('VERIFY_KIT', {'tray_id': 'tray_v1_0',
                                        'expected_bom': [{'type': 'red_block', 'count': 0}]}))
        refuses('REFUSED_BAD_TYPE', S.validate_skill_request,
                self.req('VERIFY_KIT', {'tray_id': 'tray_v1_0', 'expected_bom': []}))

    def test_revision_and_generation_are_non_negative_ints(self):
        refuses('REFUSED_OUT_OF_RANGE', S.validate_skill_request,
                self.req('STOP', {}, revision=-1))
        refuses('REFUSED_OUT_OF_RANGE', S.validate_skill_request,
                self.req('STOP', {}, resource_generation=-1))

    def test_no_argument_name_is_required_but_wrong(self):
        """★ Structural guard: every name in SKILL_ARG_KEYS must be classified by the
        arg-checking branch in `_check_arguments`. A name that reaches the else-branch would
        raise at runtime on a legal request."""
        import inspect
        src = inspect.getsource(S._check_arguments)
        handled = {'expected_bom', 'part_id', 'tray_id', 'transfer_id', 'slot_id',
                   'station_id', 'order_id', 'target_id'}
        declared = set()
        for names in S.SKILL_ARG_KEYS.values():
            declared |= set(names)
        assert declared <= handled, \
            f'schema.py declares arguments no branch validates: {sorted(declared - handled)}'


# ---------------------------------------------------------------------------
# 7. SkillResult: the stale-answer rules
# ---------------------------------------------------------------------------

class TestSkillResult:

    def result(self, **over):
        base = {
            'schema_version': 1,
            'command_id': 'cmd-0001',
            'order_id': 'order-001',
            'revision': 3,
            'boot_id': 'boot-001',
            'status': 'SUCCEEDED',
            'reason_code': None,
            'evidence': ['run-001/trace.json'],
            'final_state': None,
            'start_s': 1.0,
            'end_s': 2.5,
        }
        base.update(over)
        return base

    def test_a_success_needs_no_reason(self):
        assert S.validate_skill_result(self.result())['status'] == 'SUCCEEDED'

    def test_a_failure_without_a_reason_is_refused(self):
        """★ A failure with no reason cannot be counted, and an uncountable failure is the
        shape of 'everything becomes BUDGET_EXHAUSTED'."""
        refuses('REFUSED_MISSING_FIELD', S.validate_skill_result,
                self.result(status='FAILED', reason_code=None))
        refuses('REFUSED_MISSING_FIELD', S.validate_skill_result,
                self.result(status='CANCELED', reason_code=None))

    def test_a_failure_with_a_known_reason_is_accepted(self):
        out = S.validate_skill_result(self.result(status='FAILED', reason_code='CONTACT_LOST'))
        assert out['reason_code'] == 'CONTACT_LOST'

    def test_an_unknown_reason_is_refused(self):
        refuses('REFUSED_BAD_ENUM', S.validate_skill_result,
                self.result(status='FAILED', reason_code='IT_BROKE'))

    def test_status_is_an_enum(self):
        refuses('REFUSED_BAD_ENUM', S.validate_skill_result, self.result(status='DONE'))

    def test_end_before_start_is_refused(self):
        refuses('REFUSED_OUT_OF_RANGE', S.validate_skill_result,
                self.result(start_s=2.0, end_s=1.0))

    def test_evidence_must_be_a_list_of_paths(self):
        refuses('REFUSED_BAD_TYPE', S.validate_skill_result, self.result(evidence='trace.json'))
        refuses('REFUSED_BAD_ID', S.validate_skill_result, self.result(evidence=['']))

    def test_an_evidence_reference_is_a_path_not_an_identifier(self):
        """★ The second fix this suite forced: `run-001/trace.json` has a slash, so the
        identifier rule refused every realistic evidence reference."""
        out = S.validate_skill_result(self.result(
            evidence=['p1-gate-07/acceptance.json', 'run-001/trace.json']))
        assert out['evidence'] == ['p1-gate-07/acceptance.json', 'run-001/trace.json']

    def test_evidence_cannot_traverse_out_of_the_report(self):
        """An evidence ref becomes a filename in a report, so it is input reaching the
        filesystem: absolute paths and `..` segments stop here."""
        for bad in ('/etc/passwd', '../../secret', 'a/../../b', 'a\\b', 'a//b', 'a' * 201):
            refuses('REFUSED_BAD_ID', S.validate_skill_result,
                    self.result(evidence=[bad]))

    def test_the_result_carries_both_boot_id_and_revision(self):
        """These are the two fields a late answer is caught by, so they are not optional."""
        for field in ('boot_id', 'revision'):
            raw = self.result()
            del raw[field]
            refuses('REFUSED_MISSING_FIELD', S.validate_skill_result, raw)
            assert field not in raw


# ---------------------------------------------------------------------------
# 8. Evaluation aggregation: the five fields and the ACCEPTED rule
# ---------------------------------------------------------------------------

class TestEvaluation:

    def ev(self, **over):
        base = {
            'schema_version': 1,
            'run_id': 'p2-core-smoke-01',
            'task_outcome': 'SUCCEEDED',
            'physical_result': 'PASS',
            'safety_result': 'PASS',
            'expected_behavior': 'PASS',
            'evidence_complete': True,
            'missing': [],
        }
        base.update(over)
        return base

    def test_a_clean_evaluation_is_accepted(self):
        out = S.validate_evaluation(self.ev())
        ok, blockers = S.top_level_accepted(out)
        assert ok and blockers == []
        assert S.accepted_or_refused(out) is None

    def test_unknown_anywhere_blocks_accepted(self):
        """★ 'we did not look' must not become 'fine'."""
        for field in ('physical_result', 'safety_result', 'expected_behavior'):
            ok, blockers = S.top_level_accepted(
                S.validate_evaluation(self.ev(**{field: 'UNKNOWN'})))
            assert not ok, f'{field}=UNKNOWN was accepted'
            assert any(field.split('_')[0].upper() in b or 'UNKNOWN' in b for b in blockers)

    def test_safety_has_no_not_applicable(self):
        """★ 'we did not check safety' is not a third option, it is UNKNOWN."""
        refuses('REFUSED_BAD_ENUM', S.validate_evaluation,
                self.ev(safety_result='NOT_APPLICABLE'))
        # and NOT_APPLICABLE is genuinely reachable in the field that does allow it, so the
        # refusal above is about the field, not about the token being unknown everywhere
        assert S.validate_evaluation(self.ev(physical_result='NOT_APPLICABLE'))[
            'physical_result'] == 'NOT_APPLICABLE'

    def test_incomplete_evidence_blocks_accepted_even_when_all_fields_pass(self):
        out = S.validate_evaluation(self.ev(evidence_complete=False,
                                            missing=['safety_trace']))
        ok, blockers = S.top_level_accepted(out)
        assert not ok and 'EVIDENCE_INCOMPLETE' in blockers

    def test_incomplete_must_list_what_is_missing(self):
        e = refuses('REFUSED_MISSING_FIELD', S.validate_evaluation,
                    self.ev(evidence_complete=False, missing=[]))
        assert 'missing' in (e.field or '')

    def test_complete_with_a_missing_list_is_a_conflict(self):
        refuses('REFUSED_CONFLICT', S.validate_evaluation,
                self.ev(evidence_complete=True, missing=['safety_trace']))

    def test_evidence_complete_must_be_a_bool(self):
        refuses('REFUSED_BAD_TYPE', S.validate_evaluation,
                self.ev(evidence_complete='yes'))

    def test_a_failed_task_is_not_accepted(self):
        out = S.validate_evaluation(self.ev(task_outcome='FAILED'))
        ok, blockers = S.top_level_accepted(out)
        assert not ok and 'TASK_FAILED' in blockers

    def test_a_correctly_stopped_fault_may_pass_safety_but_is_not_a_delivery(self):
        """★ Contract 7: fault tests may pass on 'stopped correctly' but must not count as a
        delivery. So safety PASS with task_outcome NEEDS_ATTENTION is not ACCEPTED."""
        out = S.validate_evaluation(self.ev(task_outcome='NEEDS_ATTENTION',
                                            physical_result='NOT_APPLICABLE',
                                            safety_result='PASS'))
        ok, blockers = S.top_level_accepted(out)
        assert not ok
        assert 'TASK_NEEDS_ATTENTION' in blockers
        assert 'PHYSICAL_NOT_APPLICABLE' in blockers

    def test_accepted_or_refused_raises_with_the_blockers(self):
        out = S.validate_evaluation(self.ev(safety_result='FAIL'))
        e = refuses('REFUSED_CONFLICT', S.accepted_or_refused, out)
        assert 'SAFETY_FAIL' in e.detail


# ---------------------------------------------------------------------------
# 9. table consistency: the tables must agree with the docs they implement
# ---------------------------------------------------------------------------

class TestTableConsistency:

    def test_forward_path_is_a_subsequence_of_the_state_set(self):
        assert set(S.ORDER_FORWARD_PATH) <= set(S.ORDER_STATES)

    def test_constrained_and_terminal_states_are_disjoint(self):
        assert not (S.CONSTRAINED_ORDER_STATES & S.TERMINAL_ORDER_STATES)

    def test_every_order_state_is_classified(self):
        """★ Catches a state added to one table and not the others.

        This test is what flagged `CANCELLED` when it was first added: it was in ORDER_STATES
        but in none of the three classifications, which is exactly the drift it exists to
        catch. Terminal states belong in the coverage set -- `SUCCEEDED` is both the last stage
        of the forward path and a terminal state, and `CANCELLED` is terminal without being on
        the forward path, so the three sets together must cover ORDER_STATES.
        """
        covered = (set(S.ORDER_FORWARD_PATH) | set(S.CONSTRAINED_ORDER_STATES)
                   | set(S.TERMINAL_ORDER_STATES))
        assert set(S.ORDER_STATES) == covered, \
            f'unclassified order states: {sorted(set(S.ORDER_STATES) - covered)}'
        assert set(S.ORDER_STATES) <= covered, 'an order state is not reachable by any rule'

    def test_every_terminal_state_is_either_the_end_of_the_path_or_a_known_failure_exit(self):
        """★ A terminal state must be reachable by a rule, not merely declared.

        `SUCCEEDED` is reachable because it ends the forward path. `CANCELLED` is reachable
        because it is in TERMINAL_ORDER_STATES *and* is not on the forward path -- i.e. it is
        declared as an exit. A terminal state that is in ORDER_STATES but in no rule at all is
        the "typed constant that cannot notice it expired" shape: a name claiming something
        nothing implements. So the assertion is that the set of declared terminals is exactly
        the set of states that some rule can reach and no rule leaves.
        """
        reachable_exits = (set(S.TERMINAL_ORDER_STATES))
        # SUCCEEDED must be both the path's end and declared terminal
        assert S.ORDER_FORWARD_PATH[-1] in S.TERMINAL_ORDER_STATES
        # every other declared terminal must be off the forward path (a real exit, not a stage)
        for state in sorted(reachable_exits - {S.ORDER_FORWARD_PATH[-1]}):
            assert state not in S.ORDER_FORWARD_PATH, \
                f'{state} is declared terminal but is also a walkable stage'
            assert state not in S.CONSTRAINED_ORDER_STATES, \
                f'{state} is declared terminal but also a constrained (resumable) state'

    def test_every_skill_has_both_an_allowed_and_a_required_entry(self):
        assert set(S.SKILL_ARG_KEYS) == set(S.SKILLS)
        assert set(S.REQUIRED_SKILL_ARGUMENTS) == set(S.SKILLS)

    def test_required_arguments_are_a_subset_of_allowed_ones(self):
        """★ 'may carry' and 'must carry' are different questions; collapsing them would make
        every optional argument mandatory."""
        for skill in sorted(S.SKILLS):
            assert S.REQUIRED_SKILL_ARGUMENTS[skill] <= S.SKILL_ARG_KEYS[skill], skill

    def test_no_skill_declares_a_forbidden_name(self):
        for skill, names in sorted(S.SKILL_ARG_KEYS.items()):
            bad = set(names) & S.FORBIDDEN_ARG_NAMES
            assert not bad, f'{skill} declares forbidden names: {sorted(bad)}'

    def test_reserved_skills_are_not_whitelisted(self):
        assert not (S.RESERVED_SKILLS & S.SKILLS)

    def test_refusal_reasons_are_disjoint_from_business_reasons(self):
        """So a report can count 'the validator said no' apart from 'the workcell said no'."""
        assert not (S.REFUSAL_REASONS & S.REASON_CODES)

    def test_entity_vocabularies_are_non_empty_and_ascii_identifiers(self):
        for name, vocab in [('STATION_IDS', S.STATION_IDS), ('PART_TYPES', S.PART_TYPES),
                            ('TRAY_IDS', S.TRAY_IDS), ('SLOT_IDS', S.SLOT_IDS)]:
            assert vocab, f'{name} is empty'
            for value in vocab:
                assert S.ID_PATTERN.match(value), f'{name} holds a non-identifier {value!r}'

    def test_every_skill_argument_name_is_a_legal_identifier_shape(self):
        for skill, names in S.SKILL_ARG_KEYS.items():
            for name in names:
                assert name.islower() and '_' in name or name.islower(), (skill, name)


# ---------------------------------------------------------------------------
# 10. the validator's own reasons must be countable
# ---------------------------------------------------------------------------

class TestReasonsAreCountable:

    def test_every_refusal_reason_the_module_can_raise_is_declared(self):
        """★ Hard lesson 2, applied to the validator itself.

        Walk the source for `SchemaRefused(` reason literals and require each to be in
        REFUSAL_REASONS. Otherwise a new refusal appears that a report cannot count, and an
        uncountable failure is indistinguishable from a different one.
        """
        import inspect
        import re
        src = inspect.getsource(S)
        found = set(re.findall(r"SchemaRefused\(\s*'(REFUSED_[A-Z_]+)'", src))
        found |= set(re.findall(r"SchemaRefused\(\s*\n\s*'(REFUSED_[A-Z_]+)'", src))
        assert found, 'no refusal literals were found -- the scan itself is broken'
        undeclared = found - S.REFUSAL_REASONS
        assert not undeclared, f'undeclared refusal reasons: {sorted(undeclared)}'

    def test_no_declared_refusal_reason_is_unused(self):
        import inspect
        import re
        src = inspect.getsource(S)
        found = set(re.findall(r"'(REFUSED_[A-Z_]+)'", src))
        unused = S.REFUSAL_REASONS - found
        assert not unused, f'declared but never raised: {sorted(unused)}'

    def test_the_schema_version_constant_is_the_one_the_module_checks(self):
        assert S.SCHEMA_VERSION == 1

    def test_a_refusal_stringifies_with_its_reason_first(self):
        e = S.SchemaRefused('REFUSED_BAD_ID', 'nope', field='order_id')
        assert str(e).startswith('REFUSED_BAD_ID')
        assert 'order_id' in str(e)
        assert e.reason == 'REFUSED_BAD_ID' and e.field == 'order_id'
