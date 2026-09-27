"""Build the P4 handover world from the W5 logistics world, with every change declared.

WHY A THIRD WORLD AND NOT AN EDIT OF THE LOGISTICS WORLD
--------------------------------------------------------
`assets/world_w2_logistic.xml` is hashed by the freeze and cited by `reports/w2-logistic-05`,
`w3-mechanism-07`, `w4-skills-05` and `w5-loop-07`. Editing it in place would invalidate four
authoritative evidence sets for a change none of them is about. So this builder READS it and writes
a separate file, exactly as `build_w2_logistic_world.py` did for the cell world. The source's
sha256 is recorded in the output, so "the source was not touched" is checkable rather than promised.

THE THREE CHANGES, AND WHERE EACH NUMBER COMES FROM
----------------------------------------------------
`reports/p4-reach-01` measured the humanoid's arm envelope against the tray's grip interface and
left two constraints it did not choose:

  1. **THE STATION SHIFT, +3.4184 m in x.** The arm needs at least +3.4134 m -- the tray must
     travel 0.13 m (its own length) to clear the presentation table, against 0.140 m of measured
     carry -- and the table permits at most +3.4234 m, because the tray cannot be lowered past the
     table's east edge and the band's first roller is at x 4.4137. The window is 10 mm wide.
  2. **THE TRAY'S START POSE**, on that table, flush with its EAST edge. Flush, not centred: from
     the centre the tray would have to travel 0.100 + 0.065 = 0.165 m, beyond the measured
     0.140 m, so a centred start can never reach the band.
  3. **THE DUPLICATE TRAY IS REMOVED.** Each role imports its own copy of the V1 tray, so the
     merged world holds TWO -- `h_payload` at the humanoid and `c_payload` at the conveyor. A
     handover whose subject is "the same tray entity" cannot start from a world with two
     candidates, so `h_payload` goes and `c_payload` -- the one the vehicle finally carries -- is
     the only tray left.

The crouch (-0.40 m) is deliberately NOT here. It is a runtime posture of the humanoid's own leg
joints, not a property of the layout; writing it into the world would describe a behaviour as
geometry.

WHAT THIS BUILDER CHECKS ABOUT ITSELF
--------------------------------------
It does not claim "nothing else changed". It compares the output against the source line by line
and requires every differing line to contain one of the five anchors this builder is allowed to
touch. A silent extra edit fails the build.
"""
import argparse
import hashlib
import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'experiments'))
sys.path.insert(0, str(ROOT / 'src'))

SOURCE = ROOT / 'assets' / 'world_w2_logistic.xml'
OUT = ROOT / 'assets' / 'world_p4_handover.xml'

#: `reports/p4-reach-01`: the midpoint of the 10 mm window the reach and the table leave.
STATION_SHIFT_M = 3.4184
HUMANOID_BASE_NAME = 'h_LINK_BASE'
TABLE_GEOM_NAME = 'h_source_table'
DUPLICATE_TRAY_BODY = 'h_payload'
TRAY_BODY = 'c_payload'
#: The only lines the builder may alter. A line that changed without matching one of these is a bug.
ALLOWED_ANCHORS = (HUMANOID_BASE_NAME, TABLE_GEOM_NAME, DUPLICATE_TRAY_BODY, TRAY_BODY,
                   '<key name="home"')


def _body_block(text, name):
    """(start, end) of a <body name="...">...</body>, by nesting depth."""
    start = text.index(f'<body name="{name}"')
    depth, i = 0, start
    while True:
        nxt_open = text.find('<body', i)
        nxt_close = text.find('</body>', i)
        if nxt_close < 0:
            raise RuntimeError(f'{name}: unterminated body')
        if 0 <= nxt_open < nxt_close:
            depth += 1
            i = nxt_open + len('<body')
        else:
            depth -= 1
            i = nxt_close + len('</body>')
            if depth == 0:
                return start, i


def _shift_pos(pos_text, dx):
    parts = pos_text.split()
    parts[0] = repr(round(float(parts[0]) + dx, 9))
    return ' '.join(parts)


def build(write=True):
    text = SOURCE.read_text(encoding='utf-8')
    source_sha = hashlib.sha256(text.encode('utf-8')).hexdigest()

    # -- the duplicate tray, captured BEFORE it is removed: its height is reused for the survivor --
    dup_start, dup_end = _body_block(text, DUPLICATE_TRAY_BODY)
    dup_match = re.search(rf'<body name="{DUPLICATE_TRAY_BODY}" pos="([^"]+)"', text)
    duplicate_z = float(dup_match.group(1).split()[2])

    # -- change 1: the humanoid's station ------------------------------------------------------
    tag = f'<body name="{HUMANOID_BASE_NAME}" pos="'
    i = text.index(tag) + len(tag)
    j = text.index('"', i)
    new_base = _shift_pos(text[i:j], STATION_SHIFT_M)
    text = text[:i] + new_base + text[j:]
    assert text.count(f'<body name="{HUMANOID_BASE_NAME}"') == 1
    base_x = float(new_base.split()[0])

    # -- change 1b: the table that belongs to that station -------------------------------------
    geom = re.search(rf'<geom name="{TABLE_GEOM_NAME}"[^>]*size="([^"]+)"[^>]*pos="([^"]+)"', text)
    if geom is None:
        geom = re.search(rf'<geom name="{TABLE_GEOM_NAME}"[^>]*pos="([^"]+)"[^>]*size="([^"]+)"',
                         text)
        table_half_x = float(geom.group(1).split()[0])
        span = geom.span(2)
    else:
        table_half_x = float(geom.group(1).split()[0])
        span = geom.span(2)
    old_table = text[span[0]:span[1]]
    new_table = _shift_pos(old_table, STATION_SHIFT_M)
    text = text[:span[0]] + new_table + text[span[1]:]
    table_x = float(new_table.split()[0])

    # -- change 3: one tray, not two -----------------------------------------------------------
    dup_start, dup_end = _body_block(text, DUPLICATE_TRAY_BODY)
    text = text[:dup_start] + text[dup_end:]
    assert f'name="{DUPLICATE_TRAY_BODY}"' not in text

    # -- change 2: the single tray's start pose ------------------------------------------------
    tag = f'<body name="{TRAY_BODY}" pos="'
    i = text.index(tag) + len(tag)
    j = text.index('"', i)
    floor = re.search(r'<geom name="c_tray_floor"[^>]*size="([^"]+)"', text)
    tray_half_x = float(floor.group(1).split()[0])
    tray_x = (table_x + table_half_x) - tray_half_x
    tray_z = duplicate_z
    text = text[:i] + f'{round(tray_x, 9)} 0 {round(tray_z, 9)}' + text[j:]

    # -- the keyframe, kept consistent ---------------------------------------------------------
    #
    # ★ ORDER. MuJoCo VALIDATES a keyframe's qpos length at COMPILE time, so the intermediate
    # compile cannot even happen while the old keyframe is still there -- the first attempt failed
    # with `keyframe 'home': invalid qpos size, expected 151, got 158`, which is the deleted tray's
    # seven values still being claimed. The addresses are needed to rebuild the keyframe, and the
    # addresses need a compile. So the intermediate compile runs with the keyframe STRIPPED, and
    # the keyframe is composed afterwards from a second, final compile.
    tmp = ROOT / 'assets' / '_p4_build_tmp.xml'
    import mujoco  # noqa: E402

    kf_line = re.search(r'[ \t]*<key name="home"[^\n]*\n', text)
    if kf_line is None:
        raise RuntimeError('the source has no `home` keyframe line to rebuild')
    tmp.write_text(text[:kf_line.start()] + text[kf_line.end():], encoding='utf-8')
    before = mujoco.MjModel.from_xml_path(str(SOURCE))
    after_del = mujoco.MjModel.from_xml_path(str(tmp))

    def qadr(model, joint):
        return int(model.jnt_qposadr[model.joint(joint).id])

    base_adr = qadr(after_del, 'h_base_free')
    tray_adr = qadr(after_del, 'c_payload_free')
    dup_adr = qadr(before, 'h_payload_free')
    assert before.nq - after_del.nq == 7

    qpos = [float(v) for v in kf_line.group(0).split('qpos="')[1].split('"')[0].split()]
    assert len(qpos) == before.nq, (len(qpos), before.nq)
    qpos = qpos[:dup_adr] + qpos[dup_adr + 7:]
    qpos[base_adr + 0] = base_x
    qpos[tray_adr + 0] = tray_x
    qpos[tray_adr + 1] = 0.0
    qpos[tray_adr + 2] = tray_z
    assert len(qpos) == after_del.nq
    kf_qpos_span = re.search(r'<key name="home" qpos="([^"]+)"', text).span(1)
    text = text[:kf_qpos_span[0]] + ' '.join(repr(v) for v in qpos) + text[kf_qpos_span[1]:]

    tmp.write_text(text, encoding='utf-8')
    final = mujoco.MjModel.from_xml_path(str(tmp))
    tmp.unlink()

    # -- self-check: every differing line must be one this builder is allowed to touch ---------
    #
    # ★ The first version compared the source and the output line by line WITHOUT accounting for
    # the deletion, so removing a 12-line body shifted every later line and the check reported
    # line 935 onwards as "changed without an anchor" -- 600 innocent lines. A positional compare
    # is only valid between texts of the same shape, so the deletion is removed from the source
    # first and the two are then the same length.
    src_full = SOURCE.read_text(encoding='utf-8')
    src_start, src_end = _body_block(src_full, DUPLICATE_TRAY_BODY)
    src_without_dup = src_full[:src_start] + src_full[src_end:]
    src_lines = src_without_dup.split('\n')
    out_lines = text.split('\n')
    if len(src_lines) != len(out_lines):
        raise RuntimeError(f'the builder changed the line count: {len(src_lines)} -> '
                           f'{len(out_lines)}')
    offenders = []
    for number, (a, b) in enumerate(zip(src_lines, out_lines), start=1):
        if a == b:
            continue
        if not any(anchor in a or anchor in b for anchor in ALLOWED_ANCHORS):
            offenders.append((number, a.strip()[:80], b.strip()[:80]))
    if offenders:
        for entry in offenders[:5]:
            print('  OFFENDER', entry)
        raise RuntimeError(f'{len(offenders)} line(s) changed without an allowed anchor')

    if write:
        OUT.write_text(text, encoding='utf-8')

    report = {
        'source': str(SOURCE.relative_to(ROOT)),
        'source_sha256': source_sha,
        'source_unchanged': hashlib.sha256(
            SOURCE.read_text(encoding='utf-8').encode('utf-8')).hexdigest() == source_sha,
        'output': str(OUT.relative_to(ROOT)),
        'output_sha256': hashlib.sha256(text.encode('utf-8')).hexdigest(),
        'station_shift_m': STATION_SHIFT_M,
        'humanoid_base_x_m': base_x,
        'table_x_m': table_x,
        'table_half_x_m': table_half_x,
        'table_east_edge_x_m': table_x + table_half_x,
        'tray_start_x_m': tray_x,
        'tray_start_z_m': tray_z,
        'duplicate_tray_removed': DUPLICATE_TRAY_BODY,
        'duplicate_tray_z_reused': duplicate_z,
        'nq_before': before.nq, 'nq_after': after_del.nq,
        'nbody_before': before.nbody, 'nbody_after': final.nbody,
        'changed_lines_carry_an_anchor': True,
    }
    return report


def verify():
    """Load the built world and report the geometry the P4 plant will drive against."""
    import mujoco  # noqa: E402
    import numpy as np  # noqa: E402
    import build_p3_world as bp3  # noqa: E402
    import merge_world as mw  # noqa: E402

    bp3.install()
    model = mujoco.MjModel.from_xml_path(str(OUT))
    data = mujoco.MjData(model)
    data.qpos[:] = np.asarray(mw.merged_home(model)[0], dtype=float)
    mujoco.mj_forward(model, data)
    name = lambda g: mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, g) or ''  # noqa: E731
    body = lambda b: mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, b) or ''  # noqa: E731
    band = sorted(float(data.geom_xpos[g][0]) for g in range(model.ngeom)
                  if name(g).startswith('c_fixed_roller'))
    tray = model.body(TRAY_BODY).id
    table_g = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, TABLE_GEOM_NAME)
    bottom = min(data.geom_xpos[g][2] - model.geom_size[g][2] for g in range(model.ngeom)
                 if int(model.geom_bodyid[g]) == tray)
    return {
        'nbody': model.nbody,
        'humanoid_base_xyz': [round(float(v), 6) for v in data.xpos[model.body(
            HUMANOID_BASE_NAME).id]],
        'table_centre_x_m': round(float(data.geom_xpos[table_g][0]), 6),
        'table_top_z_m': round(float(data.geom_xpos[table_g][2] + model.geom_size[table_g][2]), 6),
        'tray_origin_xyz': [round(float(v), 6) for v in data.xpos[tray]],
        'tray_bottom_z_m': round(float(bottom), 6),
        'tray_rests_on_the_table': abs(float(bottom)
                                      - float(data.geom_xpos[table_g][2]
                                              + model.geom_size[table_g][2])) < 0.002,
        'bodies_named_payload': sorted(body(b) for b in range(model.nbody)
                                       if 'payload' in body(b)),
        'band_first_roller_x': band[0] if band else None,
        'chassis_base_x': round(float(data.xpos[model.body('n_base_link').id][0]), 6),
        'tray_west_edge_x_m': round(float(data.xpos[tray][0]) - 0.065, 6),
    }


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--check', action='store_true', help='rebuild and compare; do not write')
    ap.add_argument('--quiet', action='store_true')
    args = ap.parse_args()
    report = build(write=not args.check)
    if args.check:
        current = OUT.read_text(encoding='utf-8')
        same = hashlib.sha256(current.encode('utf-8')).hexdigest() == report['output_sha256']
        print(f'[{"OK" if same else "FAIL"}] the built world matches what this builder produces')
        return 0 if same else 1
    for key, value in report.items():
        print(f'  {key:30s} {value}')
    print()
    for key, value in verify().items():
        print(f'  {key:30s} {value}')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
