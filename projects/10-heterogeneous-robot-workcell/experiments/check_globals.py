"""Find names a function reads as GLOBAL that the module never binds and never imports.

★ WHY THIS EXISTS. `judge_h3` used `XFER.STAGES`, but `XFER` is imported INSIDE `run()`. Every
physical claim in a 6-minute run had already been measured when the judge raised
`NameError: name 'XFER' is not defined`, and the report came back `POST_RUN_ERROR` with no rows.
Static analysis catches that in a second, and it is a recurring shape: the code that imports a
module is not always the code that uses it.

★ THE FIRST VERSION OF THIS CHECKER WAS WRONG IN THE USUAL WAY: it walked each function's body and
collected `Name` nodes, then reported any name it had not seen bound THERE. That flagged **lambda
parameters** (`side`, `s`) and **closure variables** (`prefix` inside a nested `actuators()`) as
unbound globals -- 46 hits, almost all innocent. A checker that cries wolf gets ignored, which is
worse than not having one. So this version uses `symtable`, the standard library's own scope
analysis, which resolves lambdas, comprehensions, nested functions and closures correctly.

A hit means: some function reads a name that is neither a local, a parameter, a free variable, nor
a name the module binds at top level, nor a builtin.
"""
import builtins
import symtable
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def _bound_at_module(table):
    """Names the module table itself binds: assignments, defs, classes, imports."""
    names = set()
    for symbol in table.get_symbols():
        if symbol.is_assigned() or symbol.is_imported() or symbol.is_namespace():
            names.add(symbol.get_name())
    return names


def _walk(table):
    yield table
    for child in table.get_children():
        yield from _walk(child)


def report(path):
    source = path.read_text(encoding='utf-8')
    module = symtable.symtable(source, str(path), 'exec')
    module_names = _bound_at_module(module) | set(dir(builtins))
    problems = []
    for table in _walk(module):
        if table.get_type() != 'function':
            continue
        for symbol in table.get_symbols():
            name = symbol.get_name()
            if not symbol.is_referenced():
                continue
            # `is_global()` is True for a name the compiler resolves in the GLOBAL scope -- which
            # is exactly the case that raises NameError at run time if nothing binds it.
            if not symbol.is_global():
                continue
            if name in module_names:
                continue
            if name in ('__name__', '__file__', '__doc__'):
                continue
            problems.append((path.name, table.get_name(), table.get_lineno() or 0, name))
    return problems


def main(paths):
    total = 0
    for raw in paths:
        for problem in report(ROOT / raw):
            total += 1
            print('  %s: %s() line %s reads %r as a global the module never binds'
                  % problem)
    print('[%s] %d unbound global read(s) across %d file(s)'
          % ('FAIL' if total else 'OK', total, len(paths)))
    return 1 if total else 0


if __name__ == '__main__':
    raise SystemExit(main(sys.argv[1:]))
