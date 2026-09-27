"""The docking contract: declared geometry, a window, and where every number comes from.

A contract is not a config file with nice names. Three properties make it one:

  * **every threshold carries its source and its version**, and an unfrozen threshold cannot be used
    silently -- `check_window` reports it instead of applying it;
  * **the window is a consequence of several errors**, and the guidance warns against adding them as
    if they were independent random variables. So the contract stores them separately and
    `check_window` sums the WORST CASE, which is conservative and honest, while recording the
    components so a reader can see the budget rather than a single number;
  * **what the vehicle cannot observe is declared**, so a machine that needs an unobservable
    quantity fails to start instead of discovering it in the middle.

The numbers themselves live in `config/docking_contract.json`, whose header records that the
geometry comes from `roller_rig`'s own generator metadata (`D065`: a copy of a generator's number is
one copy too many). This module reads the config; it does not restate it.
"""
import json
import math
from pathlib import Path

CONTRACT_REFUSALS = frozenset({
    'REFUSED_NOT_A_MAPPING', 'REFUSED_MISSING_FIELD', 'REFUSED_UNKNOWN_FIELD',
    'REFUSED_BAD_TYPE', 'REFUSED_BAD_ENUM', 'REFUSED_NEGATIVE',
    'REFUSED_UNFROZEN_THRESHOLD', 'REFUSED_UNSOURCED_THRESHOLD', 'REFUSED_NO_WINDOW',
    'REFUSED_UNOBSERVABLE', 'REFUSED_NOT_INITIALISED',
})

CONTRACT_KEYS = frozenset({'schema_version', 'contract_id', 'version', 'frozen',
                           'stations', 'axes', 'frames', 'tolerance', 'observability',
                           'validity', 'sources',
                           # declared extras: a prose header, and the measured bracket the window
                           # came from. Listed here so an unknown field is still an error.
                           '_comment'})
STATION_KEYS = frozenset({'station_id', 'role', 'frame_id', 'trains_along',
                          'travel_sign', 'rotation_centre_m', 'declared_by'})
TOLERANCE_KEYS = frozenset({'longitudinal_gap_m', 'lateral_error_m', 'height_mismatch_m',
                            'yaw_error_rad', 'window_m', 'window_yaw_rad', 'budget',
                            # the discrete sweep points the window was bracketed from: the boundary
                            # lies BETWEEN the last pass and the first fail, so it is a bracket
                            'measured_bracket'})
VALIDITY_KEYS = frozenset({'max_observation_age_s', 'stopped_speed_mps',
                           'settle_duration_s', 'docked_validity_s'})
OBSERVABILITY_KEYS = frozenset({'observable', 'not_observable', 'sensor_ids', 'method',
                                 # what the gate measured and what it refused to certify
                                 'resolution_m', 'certifies_tight_side', 'verdict_source',
                                 'why_not_certified'})
REQUIRED_AXES = ('+x', '-x', '+y', '-y')


class ContractRefused(ValueError):
    def __init__(self, reason, detail='', *, field=None):
        self.reason = reason
        self.detail = detail
        self.field = field
        super().__init__(f'{reason}: {detail}' if detail else reason)


def _require(raw, keys, allowed, *, where):
    if not isinstance(raw, dict):
        raise ContractRefused('REFUSED_NOT_A_MAPPING', type(raw).__name__, field=where)
    missing = sorted(allowed - set(raw))
    if missing:
        raise ContractRefused('REFUSED_MISSING_FIELD', f'{missing}', field=where)
    unknown = sorted(set(raw) - allowed)
    if unknown:
        raise ContractRefused('REFUSED_UNKNOWN_FIELD', f'{unknown}', field=where)
    return {key: raw[key] for key in keys}


def _positive(value, *, where):
    if not isinstance(value, (int, float)) or isinstance(value, bool):
        raise ContractRefused('REFUSED_BAD_TYPE', repr(value), field=where)
    if value < 0:
        raise ContractRefused('REFUSED_NEGATIVE', repr(value), field=where)
    return float(value)


def validate_contract(raw):
    """Normalise and check a contract. Refuses rather than filling in a default: a default here is a
    number nobody declared."""
    doc = _require(raw, CONTRACT_KEYS, CONTRACT_KEYS, where='contract')
    if doc['frozen'] is not True:
        raise ContractRefused('REFUSED_UNFROZEN_THRESHOLD',
                              'the contract is not marked frozen, so its thresholds may still move',
                              field='frozen')
    sources = doc['sources']
    if not isinstance(sources, dict) or not sources:
        raise ContractRefused('REFUSED_UNSOURCED_THRESHOLD',
                              'no source table: every threshold must say where it came from',
                              field='sources')

    stations = {}
    for station_id, entry in (doc['stations'] or {}).items():
        block = _require(entry, STATION_KEYS, STATION_KEYS, where=f'stations.{station_id}')
        if block['trains_along'] not in REQUIRED_AXES:
            raise ContractRefused('REFUSED_BAD_ENUM', repr(block['trains_along']),
                                  field=f'stations.{station_id}.trains_along')
        for axis, vector in (doc['axes'] or {}).items():
            _require(vector, frozenset({'x', 'y', 'z'}), frozenset({'x', 'y', 'z'}),
                     where=f'axes.{axis}')
        stations[station_id] = block

    tolerance = _require(doc['tolerance'], TOLERANCE_KEYS, TOLERANCE_KEYS, where='tolerance')
    budget = tolerance['budget']
    if not isinstance(budget, dict) or not budget:
        raise ContractRefused('REFUSED_MISSING_FIELD', 'tolerance.budget', field='tolerance.budget')
    for name in budget:
        entry = budget[name]
        if not isinstance(entry, dict) or 'value_m' not in entry or 'source' not in entry:
            raise ContractRefused('REFUSED_UNSOURCED_THRESHOLD',
                                  f'budget entry {name!r} needs a value_m and a source',
                                  field=f'tolerance.budget.{name}')
        _positive(entry['value_m'], where=f'tolerance.budget.{name}.value_m')
        if entry['source'] not in sources:
            raise ContractRefused('REFUSED_UNSOURCED_THRESHOLD',
                                  f'budget entry {name!r} cites {entry["source"]!r}, which is not in '
                                  f'the source table', field=f'tolerance.budget.{name}.source')

    validity = _require(doc['validity'], VALIDITY_KEYS, VALIDITY_KEYS, where='validity')
    for key in VALIDITY_KEYS:
        _positive(validity[key], where=f'validity.{key}')
    observability = _require(doc['observability'], OBSERVABILITY_KEYS, OBSERVABILITY_KEYS,
                             where='observability')
    if not observability['observable']:
        raise ContractRefused('REFUSED_UNOBSERVABLE',
                              'the contract declares nothing observable, so no docking can start',
                              field='observability.observable')
    return {'schema_version': int(doc['schema_version']), 'contract_id': doc['contract_id'],
            'version': int(doc['version']), 'frozen': True,
            'stations': stations, 'axes': doc['axes'], 'frames': doc['frames'],
            'tolerance': {**tolerance, 'budget': budget}, 'observability': observability,
            'validity': validity, 'sources': sources}


def load_contract(path):
    text = Path(path).read_text(encoding='utf-8')
    return validate_contract(json.loads(text))


def window_of(contract):
    """The transfer window, and the price of the declaration: a sum of worst cases.

    The guidance says the budget's components must not be treated as independent random variables
    and summed in quadrature. So this adds them, states that it is adding worst cases, and hands back
    the components so the reader can see what was paid. The result is a LOWER bound on the true
    window, which is the safe direction for a docking decision.
    """
    total = 0.0
    parts = {}
    for name, entry in sorted(contract['tolerance']['budget'].items()):
        parts[name] = {'value_m': float(entry['value_m']), 'source': entry['source']}
        total += float(entry['value_m'])
    declared = float(contract['tolerance']['window_m'])
    return {'declared_window_m': declared, 'worst_case_sum_m': total,
            'components': parts,
            'within_declaration': total <= declared + 1e-12,
            'basis': 'sum of worst cases, not a quadrature sum: the components are not independent '
                     'random variables and the guidance forbids treating them as such. The result '
                     'is therefore a lower bound on the real window.'}


def check_window(contract, *, longitudinal_m, lateral_m, height_m, yaw_rad):
    """Does a measured relative pose fit the transfer window?

    `longitudinal_m` is the MEASURED gap, and the error is its deviation from the nominal gap the
    contract declares. Treating the nominal gap itself as an error made a perfectly docked vehicle
    read as 70 mm out, so the window could never fit -- which is what the first version did.
    """
    tol = contract['tolerance']
    nominal = float(tol['longitudinal_gap_m'])
    deviation = float(longitudinal_m) - nominal
    results = {}
    for name, value, limit in (
            ('longitudinal_gap_m', deviation, tol['window_m']),
            ('lateral_error_m', lateral_m, tol['lateral_error_m']),
            ('height_mismatch_m', height_m, tol['height_mismatch_m']),
            ('yaw_error_rad', yaw_rad, tol['yaw_error_rad'])):
        results[name] = {'value': float(value), 'limit': float(limit),
                         'ok': abs(float(value)) <= float(limit) + 1e-12}
    worst = window_of(contract)
    total = abs(deviation) + abs(float(lateral_m)) + abs(float(height_m))
    results['total_translation_m'] = {'value': total, 'limit': float(tol['window_m']),
                                      'ok': total <= float(tol['window_m']) + 1e-12}
    results['yaw_within_window'] = {'value': abs(float(yaw_rad)),
                                    'limit': float(tol['window_yaw_rad']),
                                    'ok': abs(float(yaw_rad)) <= float(tol['window_yaw_rad']) + 1e-12}
    ok = all(item['ok'] for item in results.values())
    return {'fits': ok, 'components': results,
            'declared_window_m': float(tol['window_m']), 'worst_case_sum_m': worst['worst_case_sum_m'],
            'note': 'the worst-case sum is reported so a reader can see that the declared window is '
                    'not larger than the sum of the parts it is supposed to cover'
                    if worst['within_declaration'] else
                    'the declared window is SMALLER than the sum of its own components, which means '
                    'the contract cannot be met by construction and must be re-declared'}
