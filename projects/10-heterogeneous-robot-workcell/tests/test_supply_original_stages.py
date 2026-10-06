"""Pure aggregation/structural tests, not physical six-stage acceptance."""
import ast
import sys
from pathlib import Path
import pytest
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'experiments'))
from humanoid_supply import six_stage_passes
from humanoid007.tray_task import STAGES


def healthy():
    return dict(stages={name:dict(verdict='PASS') for name in STAGES},failed=[],not_run=[])


def test_human_summary_text_is_not_the_status_enum():
    result=healthy();result['overall']='PASS: all six stages'
    assert six_stage_passes(result)


@pytest.mark.parametrize('verdict',['FAIL','UNKNOWN','NOT_RUN'])
def test_one_missing_or_failed_stage_blocks_supply(verdict):
    result=healthy();result['stages']['hold']['verdict']=verdict
    assert not six_stage_passes(result)


def test_missing_stage_cannot_pass():
    result=healthy();del result['stages']['exit']
    assert not six_stage_passes(result)


def test_original_contact_and_stage_instruments_are_reused():
    source=(ROOT/'experiments/humanoid_supply.py').read_text()
    tree=ast.parse(source)
    calls={ast.unparse(node.func) for node in ast.walk(tree) if isinstance(node,ast.Call)}
    assert {'tray_task.evaluate','H1.foot_and_collision_instruments'}<=calls
