"""Controller routing contracts only; physical balance is judged separately."""
import ast
import inspect
import sys
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

root = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(root / 'src'), str(root / 'experiments')]
import humanoid_supply as supply_module


def compose_from_source(mode, runtime, static):
    # Exercise the actual nested callback, without running a physical H sequence.
    source = ast.parse(inspect.getsource(supply_module.supply))
    callback = next(node for node in source.body[0].body
                    if isinstance(node, ast.FunctionDef) and node.name == 'compose')
    namespace = dict(np=np, OPEN=supply_module.OPEN, post_mode=mode,
                     runtime=runtime, default=np.arange(10.), stationary_control=static)
    exec(compile(ast.Module(body=[callback], type_ignores=[]), '<supply callback>', 'exec'), namespace)
    return namespace['compose']


def test_policy_callback_runs_original_controller_every_period_and_keeps_other_roles():
    calls = []
    def control(command, arms, hands, **kwargs):
        calls.append((command.copy(), arms.copy(), hands, kwargs))
        return kwargs['base_control'].copy()
    runtime = SimpleNamespace(tick=100, control=control)
    def forbidden_static(*args):
        raise AssertionError('policy mode must never skip policy history')
    compose = compose_from_source('policy', runtime, forbidden_static)
    base = np.arange(120.)
    for _ in range(3):
        np.testing.assert_array_equal(compose(base), base)
    assert runtime.tick == 103 and len(calls) == 3
    for command, arms, hands, kwargs in calls:
        np.testing.assert_array_equal(command, np.zeros(3))
        assert kwargs['stationary'] is False and kwargs['stopping'] is True
        assert set(hands) == {'left', 'right'}
        for hand in hands.values():
            np.testing.assert_array_equal(hand, supply_module.OPEN)
    np.testing.assert_array_equal(base, np.arange(120.))


def test_legacy_static_callback_remains_explicit_and_advances_tick():
    calls = []
    runtime = SimpleNamespace(tick=100)
    def static(r, default, base):
        calls.append(r.tick)
        return base.copy()
    compose = compose_from_source('static', runtime, static)
    compose(np.zeros(120))
    assert calls == [100] and runtime.tick == 101


@pytest.mark.parametrize('mode', ['', 'walk', None, 'POLICY'])
def test_unknown_mode_refused_before_world_access(mode):
    with pytest.raises(ValueError, match='unknown post-supply mode'):
        supply_module.supply(None, None, None, None, None, None, post_mode=mode)
