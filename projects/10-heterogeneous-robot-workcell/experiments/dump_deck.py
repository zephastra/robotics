import json
import sys

run = sys.argv[1]
report = json.load(open(f'reports/{run}/report.json'))
print('run', run, 'tray', report['tray_asset'], report['tray_asset_sha256'][:12])
print('roller half length', report['roller_half_length_m'], 'handle clearance',
      report['handle_clearance_m'])
print('deck', report['deck'])
print()
for scenario in report['scenarios']:
    name = scenario['scenario']['name']
    records = scenario['records']
    deck_x0 = report['deck']['x0'] + scenario['scenario']['gap']
    got_on = [r for r in records if r['tray'][0] >= deck_x0]
    travel = max(r['tray'][0] for r in records) - records[0]['tray'][0]
    y_drift = max(abs(r['tray'][1]) for r in records)
    z_min = min(r['tray'][2] for r in records)
    print(f'== {name}  status={scenario["status"]} fell_at={scenario["fell_at"]}')
    print(f'   travel_x={travel:.4f}  max_tray_x={max(r["tray"][0] for r in records):.4f}'
          f'  deck_x0={deck_x0:.4f}  samples_on_deck={len(got_on)}/{len(records)}')
    print(f'   max|y|={y_drift:.4f}  min_tray_z={z_min:.4f}  final_rel={records[-1]["rel_to_deck"]}')
    print(f'   final deck_q={records[-1]["deck_qpos"]:.4f}  final pin_q={records[-1]["pin_qpos"]:.4f}')
    print('      t    tray_x   tray_y   tray_z   rel_x    rel_z    deckq    pinq   contacts')
    for r in records:
        if abs(r['t'] - round(r['t'])) < 1e-9 or r['t'] > 21.9:
            print(f'   {r["t"]:6.2f} {r["tray"][0]:8.4f} {r["tray"][1]:8.4f} {r["tray"][2]:8.4f}'
                  f' {r["rel_to_deck"][0]:8.4f} {r["rel_to_deck"][2]:8.4f}'
                  f' {r["deck_qpos"]:8.4f} {r["pin_qpos"]:7.4f} {r["contacts"]:6d}')
    print()
