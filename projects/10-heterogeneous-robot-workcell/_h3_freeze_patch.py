"""Add `experiments/check_globals.py` to the freeze's CODE list.

Script rather than a tool edit: the Edit tool reports `ModifyBackup failed` on this path.
"""
from pathlib import Path

path = Path("/home/ziling/projects/010_heterogeneous_robot_workcell/experiments/make_p3_freeze.py")
text = path.read_text(encoding="utf-8")

anchor = "    'experiments/probe_h3_w5.py',\n"
addition = (anchor
            + "    #: the scope checker `tests/test_h3_integration.py` runs, and which found the\n"
              "    #: `XFER` defect that cost a whole run (D114)\n"
              "    'experiments/check_globals.py',\n")

if anchor not in text:
    raise SystemExit("[FAIL] anchor not found")
if "check_globals.py" in text:
    raise SystemExit("[FAIL] check_globals.py is already listed")
path.write_text(text.replace(anchor, addition, 1), encoding="utf-8")
print("listed:", "experiments/check_globals.py" in path.read_text(encoding="utf-8"))
