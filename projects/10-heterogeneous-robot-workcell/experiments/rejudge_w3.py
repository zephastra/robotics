"""Re-judge W3 after fixing a measurement bug -- WITHOUT re-running the sweep.

THE BUG
-------
`probe_w3_dock_mechanism.measure()` reads `geom_xpos` of the rail geoms and the compliance row
compared the resulting number against the deck frame's half-width. But `geom_xpos` is the geom
CENTRE, and the rail is a box `RAIL_THICKNESS_M` thick whose INNER FACE is what constrains the
frame. So the row reported an 11.000 mm clearance where the geometry declares 1.000 mm, and the
yaw row inherited the same 11 mm and printed 2.100 deg instead of 0.191 deg.

Both rows were wrong for the same reason, and both are recomputable from numbers the sweep
ALREADY recorded (`rail_face_posy_y_m`, `rail_face_negy_y_m` are the geom centres). So this script
does not re-run the physics: it copies the original report aside, re-derives the two geometry rows
with the corrected formula, and writes the result under a NEW evidence id. The original
`reports/w3-mechanism-01` is untouched (D042: a judge's run id is both what it reads and what it
writes, so a re-judge needs a new one).
"""
import hashlib
import json
import shutil
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'experiments'))

SOURCE = ROOT / 'reports' / 'w3-mechanism-01'
TARGET = ROOT / 'reports' / 'w3-mechanism-requal-01'


def main():
    import build_w2_logistic_world as builder
    import roller_rig as rig

    original = json.loads((SOURCE / 'report.json').read_text())
    half_y = float(rig.DECK_FRAME_HALF[1])
    half_x = float(rig.DECK_FRAME_HALF[0])
    thickness = builder.RAIL_THICKNESS_M
    declared_residual = builder.RESIDUAL_LATERAL_M

    rows = original['rows']
    if not rows:
        raise SystemExit('the original report has no rows to re-judge')
    posy = rows[0]['rail_face_posy_y_m']
    negy = rows[0]['rail_face_negy_y_m']
    # HALF the thickness, not the whole of it: a box of half-extent `thickness / 2` centred at
    # `posy` has its inner face at `posy - thickness / 2`.
    face_posy, face_negy = posy - thickness / 2.0, negy + thickness / 2.0
    channel_half = (face_posy - face_negy) / 2.0
    clearance = channel_half - half_y
    yaw_bound_deg = __import__('math').degrees(__import__('math').atan(clearance / half_x))

    checks = []
    for check in original['checks']:
        name = check['check']
        if name.startswith('the rails hold the deck frame'):
            ok = abs(channel_half - (half_y + declared_residual)) < 5e-4
            checks.append({
                'check': name, 'status': 'PASS' if ok else 'FAIL',
                'detail': (
                    f'RECOMPUTED. The rail geom CENTRES are {posy:.6f} and {negy:.6f}; the geoms '
                    f'are {thickness * 1000:.1f} mm thick boxes, so the INNER FACES are at '
                    f'{face_posy:.6f} and {face_negy:.6f} and the channel half-width is '
                    f'{channel_half:.6f} m. Against the deck frame half-width {half_y:.4f} plus the '
                    f'declared residual {declared_residual:.4f} that is a clearance of '
                    f'{clearance * 1000:.3f} mm per side, which is what the builder declares. The '
                    f'original row compared the geom CENTRE against the frame half-width and '
                    f'reported {clearance * 1000 + thickness * 1000:.3f} mm -- a measurement bug in '
                    f'the judge, not a geometry error in the world')})
        elif name.startswith('the declared yaw limit'):
            checks.append({
                'check': name, 'status': 'PASS' if yaw_bound_deg < 0.2 else 'FAIL',
                'detail': (
                    f'RECOMPUTED with the corrected clearance {clearance * 1000:.3f} mm: a square '
                    f'frame of half-width {half_y:.2f} m inside that channel cannot be yawed by '
                    f'more than {yaw_bound_deg:.3f} deg and still fit, against C\'s own bracket of '
                    f'0.2 deg passing and 0.5 deg failing. The original row used the uncorrected '
                    f'11.000 mm and printed 2.100 deg, which read as "the mechanism cannot meet the '
                    f'contract" when the mechanism was fine and the arithmetic was not')})
        else:
            checks.append(dict(check))

    verdict = 'FAIL' if any(c['status'] == 'FAIL' for c in checks) else 'PASS'
    TARGET.mkdir(parents=True, exist_ok=True)
    inputs = {}
    for name in ('report.json',):
        source = SOURCE / name
        shutil.copyfile(source, TARGET / f'source_{name}')
        inputs[name] = hashlib.sha256(source.read_bytes()).hexdigest()
    report = dict(original)
    report.update({'run_id': 'w3-mechanism-requal-01',
                   'requalifies': 'w3-mechanism-01',
                   'verdict': verdict, 'checks': checks,
                   'source_sha256': inputs,
                   'correction': (
                       'the two geometry rows used the rail geom CENTRE as if it were the inner '
                       'FACE, so an 11 mm-thick-box offset was counted as clearance. Corrected here '
                       'by subtracting half the rail thickness, using numbers the original sweep '
                       'already recorded. No physics was re-run and the original report is '
                       'unchanged.'),
                   'unchanged_rows': [c['check'] for c in checks
                                      if not c['detail'].startswith('RECOMPUTED')]})
    (TARGET / 'report.json').write_text(json.dumps(report, indent=2) + '\n', encoding='utf-8')
    (TARGET / 'README.md').write_text(
        '# w3-mechanism-requal-01\n\n'
        'A RE-JUDGE of `reports/w3-mechanism-01`, not a new run.\n\n'
        f'* source sha256 (report.json): `{inputs["report.json"]}`\n'
        f'* verdict then: `{original["verdict"]}` -> now: `{verdict}`\n\n'
        '## What changed and why\n\n'
        'Two of the four failing rows were failing because the JUDGE measured the rail geom\'s\n'
        'centre and called it the rail\'s inner face. The rails are 20 mm thick boxes, so the row\n'
        'reported 11.000 mm of clearance where the geometry declares 1.000 mm, and the yaw row\n'
        'inherited that and printed 2.100 deg instead of 0.191 deg.\n\n'
        '## What did NOT change\n\n'
        'The physical rows. The mechanism docks for 13 of the 15 entries inside its declared\n'
        'catch and 1 of the 3 outside it, which means the catch boundary is not clean; three\n'
        'entries JAM instead of docking (two outside the catch, one inside it), and the jam is\n'
        'sign-asymmetric. All of that is in the original sweep and none of it is affected by the\n'
        'measurement fix.\n\n'
        '## What this does not license\n\n'
        'A PASS here would mean the GEOMETRY is what it declares. It would not mean the mechanism\n'
        'works. The rows that say whether it works are the physical ones, and they still fail.\n',
        encoding='utf-8')
    print(json.dumps({'requalifies': original['run_id'], 'verdict_then': original['verdict'],
                      'verdict_now': verdict,
                      'corrected_clearance_mm': round(clearance * 1000, 3),
                      'corrected_yaw_bound_deg': round(yaw_bound_deg, 3),
                      'failed_now': [c['check'] for c in checks if c['status'] == 'FAIL']},
                     indent=2))
    return 0 if verdict == 'PASS' else 1


if __name__ == '__main__':
    sys.exit(main())
