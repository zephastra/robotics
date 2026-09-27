"""Dump the state transitions of one (scenario, mu, guides) cell of a C run."""
import json
import sys
from pathlib import Path

ROOT = Path('.').resolve()
run_id = sys.argv[1]
wanted = set(sys.argv[2:])
runs = json.loads((ROOT / 'reports' / run_id / 'report.json').read_text())['runs']

for x in runs:
    tag = f"{x['scenario']['name']}@{x['friction']}gd{int(x['guides'])}"
    if wanted and tag not in wanted:
        continue
    print(f"--- {tag} final={x['final_state']} fell={x['fell_at']} samples={x['n_samples']}")
    prev = None
    for rec in x['records']:
        if rec['state'] == prev:
            continue
        rel = rec['rel_to_deck']
        print(f"    t={rec['t']:8.3f} {rec['state']:<11} rel_x={rel[0]:8.4f} "
              f"rel_z={rel[2]:7.4f} deck={rec['deck_qpos']:.4f} pin={rec['pin_qpos']:.4f} "
              f"A={int(rec['beam_seat_a'])} B={int(rec['beam_seat_b'])} c={rec['contacts']}")
        prev = rec['state']
