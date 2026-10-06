"""A FIXTURE report that lets `judge_h3` be exercised without a two-minute physical run.

★ WHY. `judge_h3` runs LAST, after every physical claim has been measured. Two runs were lost to
defects that only appear there: `XFER` was imported inside `run()` and read by the judge
(`NameError`), and then `tx` was defined next to its first user while a later row needed it earlier
(`UnboundLocalError`). Both cost a full run to discover, and both are invisible to an import check.
So the judge is called here on a report that is internally consistent, for BOTH arms.

The fixture is deliberately small and mostly correct: its job is to prove the judge RUNS and
produces one row per declared check, not to re-assert the physics. The physics is the real run's
job, and `reports/p4-h3-w5-01/report.json` is where it lives.
"""

STAGES = ['REQUESTED', 'BOTH_RESERVED', 'DOCK_VERIFIED', 'BOTH_READY', 'TRANSFERRING',
          'RECEIVER_CONFIRMED', 'COMMITTED', 'RELEASED']
CUSTODY = ['AT_SOURCE', 'AT_SOURCE', 'TRANSFERRING', 'TRANSFERRING', 'AT_DESTINATION',
           'AT_DESTINATION']


def _sample(t, x):
    return {
        'time': t, 'payload': [x, 0.0, 0.8469], 'payload_speed': 0.0,
        'hand_contacts': {'left': 1, 'right': 1} if t < 34.0 else {'left': 0, 'right': 0},
        'arm_dev_rad': 0.001, 'support': {}, 'tray_tilt_deg': 0.003,
        'band_clearance_m': 0.2295, 'band_clearance_live_m': 0.2295,
        'hand_tray_clearance_m': 0.1262, 'tray_support_rows': ['source_band'],
        'hand_tray_contacts': [], 'feet': {'L': {'pitch_deg': 0.5, 'ground_contacts': 8},
                                          'R': {'pitch_deg': 0.5, 'ground_contacts': 8}},
    }


def _transaction(leg, source, receiver, destination_zone):
    return {
        'source': source, 'receiver': receiver, 'refusal': None,
        'final_stage': 'RELEASED', 'final_custody': 'AT_DESTINATION',
        'history': list(STAGES), 'custody_history': list(CUSTODY),
        'stage_evidence_keys': list(STAGES),
        'preconditions': {'source_has_tray': True, 'receiver_empty': True},
        'tray_zone_after': destination_zone, 'support_after': [destination_zone],
        'source_zone': 'source_band' if leg == 'xfer_source_to_deck' else 'deck',
        'destination_zone': destination_zone,
    }


def _chain(negative):
    skills = ['MOVE_TO_STATION', 'DOCK', 'START_TRANSFER', 'VERIFY_TRANSFER', 'UNDOCK',
              'MOVE_TO_STATION', 'DOCK', 'START_TRANSFER', 'VERIFY_TRANSFER', 'UNDOCK',
              'VERIFY_DELIVERY']
    rows = []
    for index, skill in enumerate(skills):
        ok = not (negative and skill in ('START_TRANSFER', 'VERIFY_TRANSFER', 'VERIFY_DELIVERY'))
        rows.append({
            'command_id': 'h3-cmd-%03d' % index, 'skill': skill, 'accepted': True,
            'idempotent': False, 'refusal': None,
            'status': 'SUCCEEDED' if ok else 'FAILED',
            'reason_code': None if ok else 'PAYLOAD_LOST',
            'final_state': {'travelled_m': 0.01, 'stopped_confirmed': True,
                            'held_speed_mps': 0.0, 'drift_m': 0.0,
                            'longitudinal_m': 0.009999, 'lateral_m': 0.0, 'yaw_rad': 0.0},
            'tray_zone': 'receiver_band' if not negative else 'source_band',
            'tray_x_m': 12.5956 if not negative else 4.3687,
            'tray_on_crowns': True, 'tray_support': ['receiver_band'],
            'plant_clock_s': 51.0 + index, 'phase': 'chain',
        })
    return rows


def report(negative=False):
    """A report whose numbers are self-consistent for the arm it describes."""
    samples = [_sample(0.1, 4.3435), _sample(12.4, 4.4484), _sample(20.0, 4.4484),
               _sample(34.398, 4.4485), _sample(50.0, 4.448523)]
    delivered = not negative
    return {
        'status': 'DIAGNOSTIC_COMPLETED', 'world': 'assets/world_w5_h085_loop.xml',
        'world_sha256': 'f964477fb8c2ffdaf383c613a6f9f9f2b40561b703e7fb040a9c6fb11cb95941',
        'samples': samples,
        'shared_world': {'humanoid_model_id': 1, 'plant_model_id': 1, 'humanoid_data_id': 2,
                         'plant_data_id': 2, 'same_model': True, 'same_data': True,
                         'plant_owns_world': False,
                         'plant_qpos_writes_at_construction': {'calibrate': 3}},
        'tray_body_id': 124, 'tray_body_id_at_end': 124, 'tray_mass_kg': 0.098,
        'tray_start': [4.343532, 0.0, 0.85],
        'tray_at_handoff': [4.448523, -0.0068, 0.8469],
        'tray_at_chain_end': [12.5956, 0.0215, 0.8475],
        'support_at_handoff': ['source_band'] if not negative else ['source_band'],
        'support_at_chain_end': ['receiver_band'] if delivered else ['source_band'],
        'start_support': [],
        'grasp_s': 12.398, 'hand_off_since_s': 34.398, 'release_s': 34.398,
        'hand_touch_last_s': 34.298, 'humanoid_phase_end_s': 50.0,
        'chassis_drift_during_humanoid_m': 0.0,
        'chassis_trace_during_humanoid': [[0.0, 4.41372], [50.0, 4.41372]],
        'second_vehicle_trace_during_humanoid': [[0.0, 9.80291], [50.0, 9.80291]],
        'full_H_acceptance': ('FAIL: 2 of 6 stages failed (lift, hold)' if negative
                              else 'PASS: all six stages'),
        'band_clearance_run_min': {
            'all_geoms_m': 0.0228, 'all_geoms_who': 'h_lh_rf_tip',
            'collision_enabled': {'m': 0.0228, 'humanoid': 'h_lh_rf_tip/<unnamed>',
                                  'band': 'c_fixed_roller_0_1/c_fixed_roller_0_1',
                                  'humanoid_xyz': [4.46569, 0.23869, 0.79935],
                                  'band_xyz': [4.49372, 0.0, 0.775], 'beyond_budget': False,
                                  't': 34.398}},
        'abnormal_collision_count': 0, 'abnormal_collisions': [],
        'plant_qpos_writes': {'calibrate': 3}, 'runtime_qpos_writes': 0,
        'init_qpos_writes': 1,
        'equalities_involving_the_tray': [],
        'equalities_involving_the_tray_at_end': [],
        'gate_counters': {'accepted': 11, 'refused': 0, 'watchdog_fired': 0},
        'chain_started_s': 51.286,
        'chain_rows': _chain(negative),
        'chain_transactions': {
            'xfer_source_to_deck': _transaction('xfer_source_to_deck', 'c_source_band',
                                                'c_deck', 'deck'),
            'xfer_deck_to_receiver': _transaction('xfer_deck_to_receiver', 'c_deck',
                                                  'c_receiver_band',
                                                  'receiver_band' if delivered else 'deck'),
        },
        'negative_arm': {'enabled': negative},
    }
