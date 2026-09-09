# Validation — 2026-09-08

## Current bounded result

The final approach-gate version completed three full physical tasks: nominal
initial position, +2 mm initial X offset, and -2 mm initial X offset. These are
three test cases, **not a statistical reliability estimate**. Arbitrary Y
offsets, object types, payload masses, and destination changes are unvalidated.

| Initial X offset | Result | Simulation time | Report directory |
| --- | --- | --- | --- |
| +2 mm | Completed | 49.00 s | `20260908T070150496945Z` |
| 0 mm | Completed | 51.30 s | `20260908T070205901725Z` |
| -2 mm | Completed | 49.05 s | `20260908T070226135724Z` |

All three used unchanged final acceptance: sphere on the actual tray floor,
height error <12 mm, horizontal center error <60 mm, summed finger load <0.02 N,
and object speed <0.025 m/s, maintained for one second. Success occurs only
after the hand withdraws. The tray is an explicit physical part of the task.

The automated acceptance/model suite has 12 tests. It checks opposed grasp
contacts, contact redistribution with bounded local slip, false placement
conditions, stability timing, terminal-state behavior, free payload dynamics,
articulated hand joints, and asset hashes. Passing these tests does not replace
full integration runs.

## Earlier experiments and failures

Failed reports are retained under local `reports/` rather than overwritten.

- High workstation approaches collided with the support or destabilized the
  body. The current low workstation is a deliberate scope limitation.
- Altering the thumb grasp posture or enabling experimental grip feedback
  worsened retention; both were reverted/disabled.
- Requiring thumb force continuously falsely classified a retained grasp as
  lost after load redistribution. Retention now also measures hand-local slip;
  establishment still requires opposed thumb-plus-two-finger contact.
- Large lateral arm repositioning after walking broke the grasp. The robot now
  aligns its base first and lowers approximately vertically.
- Opening all fingers at once pushed the sphere off a flat platform. A staged
  release and a collision-enabled receiving tray are used instead.
- The object could remain partly supported by open fingers. A withdrawal phase
  was added before final verification; opening the hand alone is not success.
- A tighter approach gate timed out for +/-2 mm initial X offsets. The approach
  gate was adjusted from 40 to 50 mm; final placement tolerances were not changed.

## GUI and reproduction scope

A real WSLg MuJoCo viewer and Tk control panel opened, advanced simulation, and
closed at the requested two-second timeout. That timeout report is an expected
GUI test outcome, not a successful transport task. The callback smoke script
also passed pause/resume/cancel: physics did not advance while paused, resumed
after the callback, and cancellation exited with a `CANCELLED` report
(`20260908T070445070067Z`). Standing lift-only mode passed at 17.30 simulated
seconds (`20260908T070453098570Z`). Repeat the GUI check with:

```bash
.venv/bin/python scripts/gui_smoke.py
```

Human mouse/keyboard interaction and a fresh-machine installation remain user
acceptance work. The environment here uses its own pinned dependencies; the
previous project environments are not runtime dependencies.

Retained evidence: [nominal](evidence/nominal.json),
[+2 mm X](evidence/x-plus-2mm.json), [-2 mm X](evidence/x-minus-2mm.json).
