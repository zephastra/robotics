import ast
import sys
from pathlib import Path
import pytest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'experiments'))
from probe_candidate_multi_loading import empty_tray_confirmed


@pytest.mark.parametrize('observed',[
    {},dict(status='UNKNOWN',counts=None),dict(status='UNKNOWN',counts={'red':0,'blue':0}),
    dict(status='RESOLVED',counts={'red':1,'blue':0}),dict(status='RESOLVED',counts={'red':0}),
])
def test_nonempty_or_unknown_refuses(observed):
    assert not empty_tray_confirmed(observed)


def test_resolved_independent_empty_is_required():
    assert empty_tray_confirmed(dict(status='RESOLVED',counts={'red':0,'blue':0}))


def test_empty_observation_guard_is_before_any_pick_loop():
    path=Path(__file__).resolve().parents[1]/'experiments/probe_candidate_multi_loading.py'
    tree=ast.parse(path.read_text())
    guard=[n for n in ast.walk(tree) if isinstance(n,ast.If)
           and ast.unparse(n.test)=="not checks['initial_observed_empty']"]
    assert len(guard)==1
    calls=[ast.unparse(n.func) for n in ast.walk(guard[0]) if isinstance(n,ast.Call)]
    assert 'owner.permit.stop' in calls and 'owner.step' in calls
    loops=[n for n in ast.walk(tree) if isinstance(n,ast.For)
           and ast.unparse(n.target)=='(index, cls)']
    assert len(loops)==1 and guard[0].lineno<loops[0].lineno
