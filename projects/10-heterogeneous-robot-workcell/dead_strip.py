"""Does the tray fall into a dead strip at the fixed/deck handoff?

Pure arithmetic on the rig constants plus the stall positions observed in deck-07, so the answer
does not depend on re-running the simulation.
"""
import importlib.util
from pathlib import Path

ROOT = Path('.').resolve()
spec = importlib.util.spec_from_file_location('rig', ROOT / 'experiments' / 'roller_rig.py')
rig = importlib.util.module_from_spec(spec)
spec.loader.exec_module(rig)

tray = rig.tray_measurements(ROOT / 'assets' / 'objects' / 'tray_v1.xml')
half_x = (tray['x_max'] - tray['x_min']) / 2.0

last_fixed_crown = rig.ROLLER_PITCH * (rig.ROLLERS_PER_SECTION * rig.FIXED_SECTIONS - 1)
first_deck_crown = rig.DECK_X0 + rig.DECK_ROLLER_X0
print(f'last fixed crown     x = {last_fixed_crown:.4f}')
print(f'first deck crown     x = {first_deck_crown:.4f}')
print(f'handoff dead strip     = {first_deck_crown - last_fixed_crown:.4f} m '
      f'(DECK_LEAD_GAP = {rig.DECK_LEAD_GAP:.4f})')
print(f'tray half-length     = {half_x:.4f}  -> support window is 2 * {half_x:.4f} = '
      f'{2 * half_x:.4f}')
print()

# A tray resting on the fixed row spans [c - h, c + h] and is supported while its trailing edge
# is still on the last fixed crown. It loses that support when c - h > last_fixed_crown.
print(f'tray can creep to   c = {last_fixed_crown + half_x:.4f} before its trailing edge '
      f'leaves the LAST FIXED roller')
print(f'  ... at which point its leading edge is at c + h = '
      f'{last_fixed_crown + 2 * half_x:.4f}')
print(f'  ... and the first DECK roller is only reached when the leading edge passes '
      f'{first_deck_crown:.4f}')
print()
stall_aligned, stall_gap30 = 1.2337, 1.2256
for tag, c in (('aligned@0.15', stall_aligned), ('gap_30mm@0.25', stall_gap30)):
    lead = c + half_x
    print(f'{tag}: stalls with centre at {c:.4f}; leading edge {lead:.4f} is '
          f'{first_deck_crown - lead:.4f} m short of the first deck roller')
print()
print('Conclusion: the 0.040 m DECK_LEAD_GAP is wider than the tray can bridge, so the tray')
print('nose-dives into it. Raising mu hid this by letting the tray JUMP the gap on momentum.')
