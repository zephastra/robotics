"""The permission callback counter cannot be rebound to an orchestration object."""
import ast
from pathlib import Path


def test_permission_counter_has_only_one_assignment():
    path=Path(__file__).resolve().parents[1]/'experiments/probe_candidate_multi_loading.py'
    tree=ast.parse(path.read_text())
    bindings=[node for node in ast.walk(tree) if isinstance(node,ast.Name)
              and node.id=='sequence' and isinstance(node.ctx,ast.Store)]
    assert len(bindings)==1
