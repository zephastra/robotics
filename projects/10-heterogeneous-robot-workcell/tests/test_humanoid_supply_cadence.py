"""Pure structural tests; not H2 physical acceptance."""
import ast
from pathlib import Path


def test_inverse_kinematics_only_on_controller_consumption_ticks():
    source=Path(__file__).resolve().parents[1]/'experiments/humanoid_supply.py'
    tree=ast.parse(source.read_text())
    supply=next(node for node in tree.body if isinstance(node,ast.FunctionDef) and node.name=='supply')
    guarded=[node for node in ast.walk(supply) if isinstance(node,ast.If)
             and ast.unparse(node.test)=='runtime.tick % 5 == 0']
    assert len(guarded)==1
    calls=[node for node in ast.walk(guarded[0]) if isinstance(node,ast.Call)
           and ast.unparse(node.func)=='H2.stage_driver']
    assert len(calls)==1


def test_supply_has_no_direct_runtime_physical_state_write():
    source=Path(__file__).resolve().parents[1]/'experiments/humanoid_supply.py'
    tree=ast.parse(source.read_text())
    for node in ast.walk(tree):
        if isinstance(node,(ast.Assign,ast.AugAssign)):
            targets=node.targets if isinstance(node,ast.Assign) else [node.target]
            assert not any(token in ast.unparse(target) for target in targets
                           for token in ('.qpos[','.qvel[','.ctrl['))
