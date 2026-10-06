"""Build candidate B: the W5 logistics world at ONE uniform interface height, with a presentation
station the humanoid can actually reach.

CHANGE 3, AND WHY IT IS NEEDED (measured, `_diag/out_h1_reach.txt`)
--------------------------------------------------------------------
Changes 1 and 2 raise the interface and remove the duplicate tray. That is NOT sufficient to supply
a tray, and the measurement that shows it is:

    the H chain grips with the hand site **0.1976 m** from the handle (its own world,
    `assets/world_tray_v1.xml`). In candidate B the tray sits at x 4.61372, in the MIDDLE of the
    roller band (4.4137 .. 6.2537), so the humanoid must stand outside the band and reach over it.
    Of the whole station band, only three positions are collision-free (4.10 / 4.15 / 4.20), and
    even the closest needs **0.3659 m = 1.85 x** the proven reach. Extending that far shifts the
    centre of mass and the posture-hold stand law tips the robot over: measured BODY_FALL at
    t = 17.5 s in `reports/_h1_smoke`, with the tilt growing from the moment the reach begins.

So a **presentation station** is added at the humanoid's working distance, flush with the band's
near end -- the same device `build_p4_handover_world.py` used for the same reason ("the tray cannot
be lowered past the table's east edge and the band's first roller is at x 4.4137"). Because change
1 raised the band, the table's top is now LEVEL with the band crown, so the tray transfers from the
table onto the band without a step. In the source world the two were 0.325 m apart.

  * the table top is 0.810, the band crown is 0.810  -> level, and that is a consequence of
    change 1, not a coincidence.

WHY A CANDIDATE WORLD AND NOT AN EDIT
-------------------------------------
`assets/world_w2_logistic.xml` is hashed by the freeze and cited by `w2-logistic-05`,
`w3-mechanism-07`, `w4-skills-05` and `w5-loop-07`. Editing it would invalidate four
authoritative evidence sets for a change none of them is about. So this builder READS it and
writes a separate file. The source's sha256 goes into a companion manifest, so "the source was
not touched" is checkable rather than promised. Per the review's own rule, **a candidate world
CANNOT cite the old world's PASS**.

CHANGE 1: ONE UNIFORM dz = +0.325 m
-----------------------------------
Measured, not read off a comment (`_diag/out_b_hw.txt`): every roller in the W5 world -- the
source band `c_fixed_roller_{0,1,2}_*`, the AMT deck `c_deck_roller_*`, and the receiver
`c_recv_roller_*` -- is an identical cylinder, radius 0.035, centre z 0.450, so **crown 0.4850**.
The tray's underside is **exactly 0.4850**: it rests on the crown. So the whole logistics
interface is ALREADY at one consistent height.

  * an earlier AABB scan of mine reported "top_z 0.700" for these rollers. It was wrong: it used
    `max(geom_size)` as a half-extent, which for a cylinder takes the 0.25 half-LENGTH as a
    vertical extent. The contact test is what caught it, and that is why these numbers come from
    contact, not from an AABB.

  46 anchors, ALL interface hardware, ALL moved by the same dz:
    38  world-level roller bodies   c_fixed_roller_{0,1,2}_N (24), c_recv_roller_N (14)
     7  receiver guides             w2_taper_*, w2_chamfer_*, w2_chamfer_post_*, w2_datum_stop
     1  c_deck                      the AMT's deck; the 8 deck rollers are CHILDREN of it
  Uniform, not per-station, because the three stations are already level with each other: one dz
  keeps every relative height intact. Raising only the source would put a 0.325 m step between the
  source and the AMT deck, which is a different problem, not a fix.

CHANGE 2: THE DUPLICATE TRAY GOES
---------------------------------
W5 holds TWO V1 trays -- `h_payload` at the humanoid and `c_payload` at the conveyor. A handover
whose subject is "the same tray entity" cannot start from a world with two candidates, so
`h_payload` is REMOVED. Measured before the fix: it sat at [0.89013, 0, 0.85], 0.098 kg, and it
silently contaminated the clearance sweep -- every geom pair involving it reported distance 0.

  * **THE KEYFRAME MUST FOLLOW, AND MUJOCO VALIDATES IT AT COMPILE TIME.** Removing the body left
    `<key name="home">` claiming 158 values against 151 DOF and the world would not LOAD:
    `keyframe 'home': invalid qpos size, expected 151, got 158`. `build_p4_handover_world.py`
    documents the identical failure. So the keyframe is STRIPPED for an intermediate compile, the
    duplicate's seven values are dropped, and it is re-inserted -- after which the result is
    compiled for real.

CHANGE 3: THE PRESENTATION STATION
----------------------------------
A table at the humanoid's station, flush with the band's near end, its top level with the band
crown, and the tray moved onto it. Anchored on the P4 station (+3.4184 m, `D087` notes the ORIGINAL
justification for that number is stale; the value is what the built world contains).

WHAT THIS BUILDER CHECKS ABOUT ITSELF
--------------------------------------
It requires EVERY differing line to name one of the anchors. A silent extra edit fails the build
and nothing is written. And the finished world is **compiled** before it is written, so a world
that cannot be loaded cannot be produced -- the first version checked only that the text had
changed in the right places, and text that does not load passed that check.
"""
import argparse
import hashlib
import json
import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]

SOURCE = ROOT / 'assets' / 'world_w2_logistic.xml'
OUT = ROOT / 'assets' / 'world_w5_h085.xml'
MANIFEST = ROOT / 'assets' / 'world_w5_h085.manifest.json'

DZ = 0.325
SOURCE_CROWN = 0.485
TARGET_TRAY_Z = 0.850
DUPLICATE_TRAY_BODY = 'h_payload'

#: The presentation station, anchored on the world the P4 handover round already built.
STATION_SHIFT_M = 3.4184
TRAY_STATION_X = 4.343532          # the P4 tray start, flush with the band's near end
TABLE_TOP_Z = 0.810                # == the raised band crown, so the transfer is level
#: The presentation fixture: x [4.245, 4.375], y [-0.06, 0.06], top 0.810. Every dimension
#: is a measured margin, not a taste (`_diag/out_h2_shape.txt`):
#:   +x edge 4.375, not 4.409 -- the tray is PLACED at 4.45372, so its underside starts at
#:     4.38872; an edge at 4.409 would still carry the tray's rear edge and make "received
#:     by the designated rollers" ambiguous. 13.7 mm clear now.
#:   -x edge 4.245, not 4.209 -- this is the face the humanoid's legs approach, and it is
#:     what limited the station to 4.10 (leg gap 12.8 mm). At 4.245 the station 4.13 has
#:     37 mm of leg clearance.
#:   y half-width 0.06, not 0.20 -- the hip/waist meshes reach in to |y| ~0.02-0.06 and
#:     were the next binding obstacle once x was fixed.
#:   the tray's centre of mass is 31.5 mm inside the +x edge at its start pose, so the
#:     fixture still carries the tray before the grasp.
TABLE_GEOM = ('<geom name="w5_h085_station" type="box" pos="4.310 0 0.803" '
              'size="0.065 0.06 0.007" friction="1.0" rgba="0.45 0.45 0.5 1"/>')

#: CHANGE 4. The source band's roller half-length, narrowed from the source's 0.250. Measured
#: in `_diag/out_h2_roller.txt`: the humanoid's hand wraps the tray's handle 50 mm inside the
#: roller's end and needs ~55 mm of clearance, so at 0.250 some hand geom is always inside the
#: first roller (-0.0037 m). 0.200 gives +0.0209 m. Only the SOURCE band; the deck and receiver
#: rollers are untouched, because the humanoid never reaches them and narrowing them would
#: change the tray's support on the deck that the AMT chain verifies.
ROLLER_HALF_LENGTH = 0.20
ROLLER_GEOM_RE = re.compile(r'(<geom name="c_fixed_roller_[^"]*" size="[0-9.]+ )([0-9.]+)(")')

ANCHOR_PREFIXES = ('c_fixed_roller_', 'c_recv_roller_', 'w2_')
ANCHOR_EXACT = ('c_deck', 'c_payload', 'w5_h085_station')
ANCHOR_EXTRA = ('<key name="home"',)

BODY_RE = re.compile(r'^(\s*<body\s+name=")([A-Za-z0-9_]+)("\s+pos=")([^"]+)(".*)$')
KF_RE = re.compile(r'[ \t]*<key name="home"[^\n]*\n')


def is_anchor(name):
    return name.startswith(ANCHOR_PREFIXES) or name in ANCHOR_EXACT


def named_anchor(line):
    return any(p in line for p in ANCHOR_PREFIXES + ANCHOR_EXACT + ANCHOR_EXTRA)


def fmt(v):
    s = ('%.5f' % v).rstrip('0').rstrip('.')
    return s if s else '0'


def strip_body(text, name):
    lines = text.split('\n')
    start = next((i for i, ln in enumerate(lines) if ('<body name="%s"' % name) in ln), None)
    if start is None:
        return text, 0
    depth, end = 0, None
    for j in range(start, len(lines)):
        depth += lines[j].count('<body')
        depth -= lines[j].count('</body>')
        if depth == 0:
            end = j
            break
    if end is None:
        raise RuntimeError('unterminated body %s' % name)
    return '\n'.join(lines[:start] + lines[end + 1:]), (end - start + 1)


def shift_anchors(text):
    out, changed = [], []
    for i, line in enumerate(text.split('\n'), 1):
        m = BODY_RE.match(line)
        if not m or not is_anchor(m.group(2)):
            out.append(line)
            continue
        pre, name, mid, pos, post = m.groups()
        parts = pos.split()
        if len(parts) != 3:
            out.append(line)
            continue
        x, y, z = parts
        out.append('%s%s%s%s %s %s%s' % (pre, name, mid, x, y, fmt(float(z) + DZ), post))
        changed.append((i, name, float(z), float(z) + DZ))
    return '\n'.join(out), changed


def add_station(text):
    """Change 3: a presentation table in the world body, and the tray moved onto it."""
    lines = text.split('\n')
    # insert the table just before the first roller body (any stable world-body anchor)
    i = next(k for k, ln in enumerate(lines) if '<body name="c_fixed_roller_0_0"' in ln)
    lines.insert(i, '    ' + TABLE_GEOM)
    text = '\n'.join(lines)
    # move the tray onto the table
    m = re.search(r'(<body name="c_payload" pos=")([^"]+)(")', text)
    if m is None:
        raise RuntimeError('no c_payload pos to move')
    old = m.group(2).split()
    new = '%s 0 %s' % (fmt(TRAY_STATION_X), fmt(TARGET_TRAY_Z))
    text = text[:m.start(2)] + new + text[m.end(2):]
    return text, (old, new)


def narrow_source_rollers(text):
    """Change 4: shrink the source band's rollers in y so the hand can clear their ends."""
    out, changed = [], []
    for i, line in enumerate(text.split('\n'), 1):
        m = ROLLER_GEOM_RE.search(line)
        if m is None:
            out.append(line)
            continue
        old_len = float(m.group(2))
        out.append(line[:m.start(2)] + fmt(ROLLER_HALF_LENGTH) + line[m.end(2):])
        changed.append((i, old_len, ROLLER_HALF_LENGTH))
    return '\n'.join(out), changed


def rebuild_keyframe(text, tmp):
    import mujoco
    m = KF_RE.search(text)
    if m is None:
        raise RuntimeError('the source has no `home` keyframe to rebuild')
    tmp.write_text(text[:m.start()] + text[m.end():], encoding='utf-8')
    before = mujoco.MjModel.from_xml_path(str(SOURCE))
    after = mujoco.MjModel.from_xml_path(str(tmp))
    assert before.nq - after.nq == 7, '%d -> %d' % (before.nq, after.nq)
    dup_adr = int(before.jnt_qposadr[before.joint(DUPLICATE_TRAY_BODY + '_free').id])
    qpos = [float(v) for v in text[m.start():m.end()].split('qpos="')[1].split('"')[0].split()]
    if len(qpos) != before.nq:
        raise RuntimeError('keyframe claims %d, source has %d' % (len(qpos), before.nq))
    # the tray moved, so its own three position values are re-written from the tray's new pose
    after_adr = int(after.jnt_qposadr[after.joint('c_payload_free').id])
    qpos = qpos[:dup_adr] + qpos[dup_adr + 7:]
    qpos[after_adr + 0] = TRAY_STATION_X
    qpos[after_adr + 1] = 0.0
    qpos[after_adr + 2] = TARGET_TRAY_Z
    if len(qpos) != after.nq:
        raise RuntimeError('trimmed keyframe has %d, world needs %d' % (len(qpos), after.nq))
    span = re.search(r'<key name="home" qpos="([^"]+)"', text).span(1)
    return text[:span[0]] + ' '.join(repr(v) for v in qpos) + text[span[1]:]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--check', action='store_true')
    args = ap.parse_args()
    import mujoco

    src = SOURCE.read_text(encoding='utf-8')
    src_sha = hashlib.sha256(src.encode('utf-8')).hexdigest()

    trimmed, stripped = strip_body(src, DUPLICATE_TRAY_BODY)
    if stripped == 0:
        raise RuntimeError('the duplicate tray %s was not found' % DUPLICATE_TRAY_BODY)

    # change 1, then change 3's tray move, then the keyframe
    shifted, changed = shift_anchors(trimmed)
    new, tray_move = add_station(shifted)
    new, roller_change = narrow_source_rollers(new)
    if not roller_change:
        raise RuntimeError('change 4 found no c_fixed_roller_* geom to narrow')
    new = rebuild_keyframe(new, ROOT / 'assets' / '_w5_h085_tmp.xml')

    # every differing line must name an anchor
    sl, nl = trimmed.split('\n'), new.split('\n')
    if len(sl) + 1 != len(nl):
        raise RuntimeError('line count %d -> %d (expected exactly +1 for the table)'
                           % (len(sl), len(nl)))
    # compare with the inserted line removed, so the shapes match
    nl_cmp = [ln for ln in nl if 'w5_h085_station' not in ln]
    diffs = [(i, a, b) for i, (a, b) in enumerate(zip(sl, nl_cmp), 1) if a != b]
    bad = [(i, b) for i, a, b in diffs if not named_anchor(b)]
    if bad:
        print('[FAIL] %d differing line(s) name no anchor:' % len(bad))
        for i, b in bad[:5]:
            print('   %d: %s' % (i, b.strip()[:110]))
        sys.exit(3)
    changed_names = {n for _i, n, _o, _n in changed}
    # c_payload legitimately appears twice (dz then the move), so allow the extra. Change 4
    # adds one differing line per source roller geom, which is derived, not typed.
    allowed = {len(changed) + 1 + len(roller_change), len(changed) + 2 + len(roller_change)}
    if len(diffs) not in allowed:
        print('[FAIL] %d differing lines vs %d recorded changes (+1 keyframe, +1 tray move, '
              '+%d roller geoms)' % (len(diffs), len(changed), len(roller_change)))
        sys.exit(3)

    tray = [c for c in changed if c[1] == 'c_payload']
    if len(tray) != 1:
        print('[FAIL] tray anchor %r' % tray); sys.exit(3)

    tmp = ROOT / 'assets' / '_w5_h085_tmp.xml'
    tmp.write_text(new, encoding='utf-8')
    final = mujoco.MjModel.from_xml_path(str(tmp))
    tmp.unlink()
    if final.nq != 151:
        print('[FAIL] compiled nq %d, expected 151' % final.nq); sys.exit(3)

    by_prefix = {}
    for _i, name, _o, _n in changed:
        by_prefix[re.sub(r'_?\d+$', '', name)] = by_prefix.get(re.sub(r'_?\d+$', '', name), 0) + 1
    print('source      %s' % SOURCE.name)
    print('source sha  %s' % src_sha[:16])
    print('dz          +%.3f m   (crown %.3f -> %.3f)' % (DZ, SOURCE_CROWN, SOURCE_CROWN + DZ))
    print('anchors     %d bodies changed' % len(changed))
    for k in sorted(by_prefix):
        print('    %-24s %d' % (k, by_prefix[k]))
    print('removed     %s (%d lines) -- the duplicate tray' % (DUPLICATE_TRAY_BODY, stripped))
    print('rollers     %d source rollers narrowed in y: %.3f -> %.3f'
          % (len(roller_change), roller_change[0][1], roller_change[0][2]))
    print('station     table top %.3f (band crown %.3f -> LEVEL), tray x %s'
          % (TABLE_TOP_Z, SOURCE_CROWN + DZ, tray_move[1]))
    print('load check  OK  nq=%d nbody=%d ngeom=%d' % (final.nq, final.nbody, final.ngeom))

    if args.check:
        if not OUT.is_file():
            print('[FAIL] %s does not exist' % OUT); sys.exit(1)
        if OUT.read_text(encoding='utf-8') == new:
            print('[OK] %s matches a fresh build' % OUT.name); sys.exit(0)
        print('[FAIL] %s differs from a fresh build' % OUT.name); sys.exit(1)

    OUT.write_text(new, encoding='utf-8')
    MANIFEST.write_text(json.dumps({
        'output': OUT.name, 'source': SOURCE.name, 'source_sha256': src_sha,
        'dz_m': DZ, 'anchors_changed': len(changed), 'removed_bodies': [DUPLICATE_TRAY_BODY],
        'station_shift_m': STATION_SHIFT_M, 'tray_station_x': TRAY_STATION_X,
        'table_top_z': TABLE_TOP_Z, 'compiled_nq': final.nq,
        'roller_half_length_m': ROLLER_HALF_LENGTH,
        'roller_geoms_narrowed': len(roller_change),
        'roller_scope': ('SOURCE band only (c_fixed_roller_*). The deck rollers and the '
                         'receiver rollers are untouched: the humanoid never reaches them, and '
                         'narrowing them would change the tray support the AMT chain verifies.'),
        'note': ("candidate B: one uniform interface height, duplicate tray removed, presentation "
                 "station added so the tray is within the humanoid's proven reach, and the source "
                 "band's rollers narrowed so the hand clears their ends. A candidate world cannot "
                 "cite the source world's PASS."),
    }, indent=2) + '\n', encoding='utf-8')
    print('wrote %s' % OUT.name)
    print('wrote %s' % MANIFEST.name)
    print('source untouched: %s'
          % (hashlib.sha256(SOURCE.read_bytes()).hexdigest() == src_sha))


if __name__ == '__main__':
    main()
