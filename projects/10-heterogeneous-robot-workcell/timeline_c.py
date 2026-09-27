"""Inspect one C run's state timeline. Reads the report the probe already wrote."""
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
run_id = sys.argv[1] if len(sys.argv) > 1 else 'p1-c-deck-06'
runs = json.loads((ROOT / 'reports' / run_id / 'report.json').read_text())['runs']

for x in runs:
    f = x['final'] or {}
    rel = f.get('rel_to_deck', [0, 0, 0])
    print(f"{x['scenario']['name']:>10}@{x['friction']} gd={str(x['guides']):>5}: "
          f"t={f.get('t')} state={f['final_state']:>10} "
          f"rel_x={rel[0]:8.4f} rel_z={rel[2]:7.4f} "
          f"deck={f.get('deck_qpos')} pin={f.get('pin_qpos')} fell={x['fell_at']}")
