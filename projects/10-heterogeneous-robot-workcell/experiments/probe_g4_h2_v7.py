"""G4 (section 7): H2 six-stage tray supply, same judging machine, world swapped to v7.

The ONLY changed factor is the world. Everything else -- the six-stage contract
(tray_task, frozen thresholds), the nine declared H2 rows, the stage schedule, the
station/place/handle constants -- is `probe_h2_w5` reused by import, not copied.

Why a wrapper and not a copy: the judge must be the same machine for a world swap
to mean anything (the same rule the G4 vision probe applied to the RGB-D judge).
`probe_h2_w5.main()` reads its module-global WORLD at call time, so patching the
global before the call changes exactly one factor.

Provenance in the report: the inherited `probe` label still reads the mother
module's string; the authoritative fields are `world`, `world_sha256` (v7) and
`command` (this wrapper's filename).
"""
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT / 'experiments'), str(ROOT / 'src')]

import probe_h2_w5 as H2  # noqa: E402

V7 = ROOT / 'assets' / 'world_p5_candidate_v7_hinged_retainer.xml'
assert V7.is_file(), 'v7 world missing'
H2.WORLD = V7  # the ONLY changed factor


if __name__ == '__main__':
    raise SystemExit(H2.main())
