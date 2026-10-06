"""Build the H3 world: candidate B's logistics world WITH THE HUMANOID PLACED IN IT.

WHY THIS FILE EXISTS AT ALL, AND WHAT IT FOUND
-----------------------------------------------
`assets/world_w5_h085.xml` (candidate B) was built to give the humanoid a reachable presentation
station. It raised the whole logistics interface by a uniform +0.325 m and moved the tray onto a
table -- but it **never moved the humanoid**. Measured (`_diag/out_h3_clear.txt`):

    candidate B, keyframe home state:
        h_LINK_BASE   x 0.660132          <- the robot is at the far end of the cell
        w5_h085_station (the table)  x 4.31
        c_payload     x 4.343532
        n_base_link   x 4.41372

H2 got away with this because **H2 wrote the station into the probe** -- an initialisation-only
`qpos[0] = 4.13` (`probe_h2_w5.py`; the write is logged in `reports/p4-h2-w5-03` as
`init_qpos_writes`). That is a legitimate DECLARED STATION for a single-probe run. It is the
wrong shape for H3, where the humanoid's runtime and the W5 `LogisticsPlant` share ONE
`MjModel`/`MjData`: a station that exists only inside a probe is a station the world does not
have, and the next consumer of this world would inherit a robot and a workcell four metres apart.

So this builder puts the station INTO THE WORLD, as the model's own home pose.

HOW THE STATION WAS CHOSEN, AND THE WRONG ANSWER THIS FILE RECORDS
-------------------------------------------------------------------
STAR THE FIRST VERSION OF THIS BUILDER PICKED 4.09, AND IT WAS WRONG. Two measurements are worth
recording because the first one is a trap this project has fallen into before:

1. **`mj_geomDistance` said the humanoid was 15.6 mm INSIDE `n_chassis` and touching the tray.**
   Both are false. The offending geoms are `contype=0 conaffinity=0 group=2` VISUAL meshes --
   physics never pairs them. `mj_geomDistance` reports geometry, not interaction, and on this
   robot it reports **272 self-penetrating pairs at the home pose**, including
   `h_LINK_BASE <-> h_LINK_BASE = -0.16740 m`. That is the same class of artefact as the
   "58.5 mm self-penetration" refuted by `D078`. **A distance between two geoms is not a contact.**

2. **Free-settling the world says the humanoid falls over at every station** (drift +0.050 m at
   4.13, +0.164 m at 4.07, hands on the floor). Also not informative here: the stand law HOLDS
   POSTURE and does not balance, which is the standing P4 fact. A settle test measures the
   absence of a balance controller, not the quality of a station.

**The measurement that IS decisive** is the physics, with the robot's own controller running --
which is exactly how H3 will drive it. At 400 ticks, `Runtime.step(..., stationary=True)`:

    station      drift_x   cross-entity contacts
    0.660132     +0.0041   h_LINK_FOOT_L/R <-> world/cell_ground  (8 + 8)   -- nothing else
    4.13         +0.0041   h_LINK_FOOT_L/R <-> world/cell_ground  (8 + 8)   -- nothing else
    4.09         +0.0041   h_LINK_FOOT_L/R <-> world/cell_ground  (8 + 8)   -- nothing else
    4.20         -0.0104   h_LINK_FOOT_L/R <-> world/cell_ground  (8 + 8)   -- nothing else

The robot stands, and **the only thing it touches anywhere near the workcell is the floor.**
4.20 is the experimental arm: the humanoid base at 4.20 puts its feet under the parked chassis
(`n_chassis` spans 4.22372 .. 4.60372) and physics still reports no chassis contact, because the
feet are below the chassis underside and simply pass under it.

So **THE STATION IS H2's 4.13.** It is inside `D102`'s feasible band [4.0, 4.15], it is the
station H2's measurements were taken at (arm residual 2.5e-4 m), and the contact test says it is
clear. Choosing 4.09 would have re-measured the arm at a station H2 never validated, for a
collision that does not exist. **Change the ruler, not the threshold -- and here the ruler was
wrong in the opposite direction: it was inventing a collision.**

WHAT THIS BUILDER REFUSES TO DO
--------------------------------
* It does not touch `assets/world_w5_h085.xml`. That file is candidate B, hashed and cited; the
  H1/H2 evidence stands as recorded there. A candidate world cannot cite the old world's PASS and
  neither can this one.
* It does not move the tray, the rollers, the deck, the receiver or the chassis. Correcting a
  humanoid-vs-chassis overlap by moving the CHASSIS would change where the W5 loop docks, which is
  the thing H3 integrates with rather than re-decides.
* It does not remove the AMT, so H3 runs against the hardware the loop actually uses.

WHAT THIS BUILDER CHECKS ABOUT ITSELF
--------------------------------------
It does not claim "nothing else changed". It diffs its output against the source line by line and
classifies EVERY differing line as either:

  (a) INHERITED FROM CANDIDATE B -- the source is candidate B, not the W5 source, so a line that
      already differed between `world_w2_logistic.xml` and candidate B must NOT differ again here;
      if it does, this builder changed something candidate B had already settled; or

  (b) INTRODUCED BY THIS STEP -- inside the anchor set this builder may touch.

A line that differs and is in neither class fails the build and nothing is written. And the
finished world is COMPILED before it is written, so a world that cannot be loaded cannot be
produced.
"""
import argparse
import hashlib
import json
import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]

#: candidate B. NOT touched. Stands as the H1/H2 evidence recorded it.
SOURCE = ROOT / 'assets' / 'world_w5_h085.xml'
#: the W5 source, read only to CLASSIFY deltas -- never written, never a build input.
REFERENCE = ROOT / 'assets' / 'world_w2_logistic.xml'
OUT = ROOT / 'assets' / 'world_w5_h085_loop.xml'
MANIFEST = ROOT / 'assets' / 'world_w5_h085_loop.manifest.json'

HUMANOID_BASE_BODY = 'h_LINK_BASE'
HUMANOID_BASE_JOINT = 'h_base_free'
TRAY_BODY = 'c_payload'
CHASSIS_BODY = 'n_base_link'

#: The humanoid's own base height, in every world this project has built. H1/H2 ran at it.
BASE_Z = 1.03

#: H2's station, retained. See the module docstring: the two measurements that suggested moving
#: it were a `mj_geomDistance` reading of non-colliding visual geoms and a free-settle test of a
#: robot that has no balance controller. The runtime-driven contact test -- the one that matches
#: how H3 drives the robot -- shows the humanoid standing at 4.13 touching only the ground.
STATION_X = 4.13
#: Declared tolerances the build enforces on the runtime-driven stand test.
STAND_TICKS = 400
STAND_DRIFT_LIMIT_M = 0.030
STAND_SETTLE_LIMIT_M = 0.030
#: Cross-entity contacts the stand test may produce. Only the ground, by construction: if the
#: robot touches the workcell's hardware while merely standing, H3's later contact accounting
#: cannot be trusted, so the build fails rather than reporting it.
#: The GROUND is the `world` BODY carrying the `cell_ground` geom -- the first version of this
#: tuple named the geom and the check rejected a perfectly good station, which is the useful
#: direction for a wrong label to fail in.
STAND_ALLOWED_CONTACT_BODIES = ('world',)

#: The only lines this builder may alter.
ALLOWED_ANCHORS = (HUMANOID_BASE_BODY, '<key name="home"')

BODY_RE = re.compile(r'^(\s*<body\s+name=")([A-Za-z0-9_]+)("\s+pos=")([^"]+)(".*)$')
KF_RE = re.compile(r'[ \t]*<key name="home"[^\n]*\n')


def _set_pos(text, name, xyz):
    """Rewrite one `<body name="...">`'s pos. Returns (text, old_xyz, count)."""
    pat = re.compile(r'(<body name="%s" pos=")([^"]+)(")' % re.escape(name))
    hits = pat.findall(text)
    if len(hits) != 1:
        raise RuntimeError('%s appears %d times, expected exactly 1' % (name, len(hits)))
    old = [float(v) for v in hits[0][1].split()]
    new = ' '.join(repr(round(v, 9)) for v in xyz)
    return pat.sub(lambda m: m.group(1) + new + m.group(3), text, count=1), old


def stand_test(station_x, quiet=False):
    """Drive the humanoid's OWN controller at `station_x` and report where it actually touches.

    This is the measurement the station is chosen from, and it is a CONTACT test, not a distance
    test -- `mj_geomDistance` on this robot reports visual meshes and self-overlaps that physics
    never sees (module docstring, point 1).
    """
    import mujoco
    import numpy as np

    sys.path.insert(0, str(ROOT / 'experiments'))
    sys.path.insert(0, str(ROOT / 'src'))
    import build_p3_world as bp3
    bp3.install()
    from humanoid007.runtime import Runtime, OPEN

    runtime = Runtime(world=str(SOURCE), prefix='h_', object_body=TRAY_BODY)
    model, data = runtime.m, runtime.d
    name_b = lambda i: mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, i) or ''
    adr = int(model.jnt_qposadr[model.joint(HUMANOID_BASE_JOINT).id])
    data.qpos[adr:adr + 7] = [float(station_x), 0.0, BASE_Z, 1.0, 0.0, 0.0, 0.0]
    mujoco.mj_forward(model, data)
    before = np.array(data.xpos[model.body(HUMANOID_BASE_BODY).id], dtype=float)

    for _ in range(STAND_TICKS):
        runtime.step(np.zeros(3), arms=runtime.arm_target.copy(),
                     hands={side: OPEN for side in runtime.arms}, stationary=True)

    after = np.array(data.xpos[model.body(HUMANOID_BASE_BODY).id], dtype=float)
    delta = after - before
    contacts = {}
    for index in range(data.ncon):
        contact = data.contact[index]
        sides = []
        for geom in (int(contact.geom1), int(contact.geom2)):
            sides.append('%s/%s' % (name_b(int(model.geom_bodyid[geom])),
                                    mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, geom) or '?'))
        contacts[tuple(sides)] = contacts.get(tuple(sides), 0) + 1
    cross = {pair: count for pair, count in contacts.items()
             if (pair[0].startswith('h_') != pair[1].startswith('h_'))}
    return {
        'station_x_m': float(station_x),
        'drift_xyz_m': [round(float(v), 6) for v in delta],
        'drift_x_m': round(float(delta[0]), 6),
        'settle_z_m': round(float(delta[2]), 6),
        'base_z_m': round(float(after[2]), 6),
        'ncon': int(data.ncon),
        'cross_entity_contacts': [{'pair': list(pair), 'count': count}
                                  for pair, count in sorted(cross.items())],
        'cross_entity_bodies': sorted({side.split('/')[0] for pair in cross for side in pair
                                       if not side.startswith('h_')}),
    }


def check_stand(result):
    """The declared tolerances, enforced by the build rather than reported and ignored."""
    problems = []
    if abs(result['drift_x_m']) > STAND_DRIFT_LIMIT_M:
        problems.append('drift x %.4f m exceeds %.4f' % (result['drift_x_m'], STAND_DRIFT_LIMIT_M))
    if abs(result['settle_z_m']) > STAND_SETTLE_LIMIT_M:
        problems.append('settle z %.4f m exceeds %.4f' % (result['settle_z_m'],
                                                          STAND_SETTLE_LIMIT_M))
    unexpected = [name for name in result['cross_entity_bodies']
                  if name not in STAND_ALLOWED_CONTACT_BODIES]
    if unexpected:
        problems.append('the standing humanoid touches %s' % unexpected)
    return problems


def rebuild_keyframe(text, tmp):
    """Recompose `<key name="home">` for the humanoid's new station.

    Same ORDER as `build_p4_handover_world.py`, for the same measured reason: MuJoCo validates a
    keyframe's qpos length AT COMPILE TIME, so an intermediate compile that has to yield joint
    addresses must run with the keyframe stripped. Here `nq` does not change -- only three values
    inside the keyframe do -- so the strip is not strictly forced, but the documented order is
    kept so a future edit that DOES change `nq` cannot silently rely on an address that moved.
    """
    import mujoco

    match = KF_RE.search(text)
    if match is None:
        raise RuntimeError('the source has no `home` keyframe line to rewrite')
    tmp.write_text(text[:match.start()] + text[match.end():], encoding='utf-8')
    model = mujoco.MjModel.from_xml_path(str(tmp))
    adr = int(model.jnt_qposadr[model.joint(HUMANOID_BASE_JOINT).id])
    if adr != 0:
        raise RuntimeError('%s qpos address is %d, not 0: the batch builders assume it is first'
                           % (HUMANOID_BASE_JOINT, adr))

    values = [float(v) for v in match.group(0).split('qpos="')[1].split('"')[0].split()]
    if len(values) != model.nq:
        raise RuntimeError('keyframe claims %d qpos, the world needs %d' % (len(values), model.nq))
    before = values[adr:adr + 7]
    values[adr + 0] = STATION_X
    values[adr + 1] = 0.0
    values[adr + 2] = BASE_Z
    values[adr + 3:adr + 7] = [1.0, 0.0, 0.0, 0.0]
    after = values[adr:adr + 7]
    span = re.search(r'<key name="home" qpos="([^"]+)"', text).span(1)
    text = text[:span[0]] + ' '.join(repr(v) for v in values) + text[span[1]:]
    return text, before, after, model.nq


def classify(source_text, reference_text, out_text):
    """Every differing line, classed as inherited-from-candidate-B or introduced-here.

    A positional compare is only meaningful between texts of the same SHAPE, so the three texts
    must be the same length. They are: candidate B's own builder inserted exactly one line (the
    presentation table) and this builder inserts none.
    """
    src_lines = source_text.split('\n')
    ref_lines = reference_text.split('\n')
    out_lines = out_text.split('\n')
    if len(src_lines) != len(out_lines):
        raise RuntimeError('this builder changed the source line count: %d -> %d'
                           % (len(src_lines), len(out_lines)))

    inherited, introduced, offenders, shared = [], [], [], []
    for number, (before, after) in enumerate(zip(src_lines, out_lines), start=1):
        if before == after:
            continue
        candidate_b_moved_it = (number <= len(ref_lines) and ref_lines[number - 1] != before)
        if not any(anchor in after for anchor in ALLOWED_ANCHORS):
            offenders.append((number, after.strip()[:100]))
            continue
        if candidate_b_moved_it:
            # TWO ANCHORS ARE LEGITIMATELY OWNED BY BOTH BUILDERS.
            #
            # `<key name="home">` is one line holding every free body's qpos, so candidate B had
            # to rewrite it when it removed the duplicate tray (`nq` 158 -> 151) and this builder
            # has to rewrite it again to move the humanoid. A rule of "candidate B touched it, so
            # this builder may not" would forbid the very edit this file exists to make, and
            # cancelling the whole build would be the safe-looking version of a wrong rule. So the
            # overlap is RECORDED rather than rejected -- `shared` names it in the report, and a
            # reader can see that the keyframe is co-owned instead of assuming one owner.
            shared.append((number, after.strip()[:100]))
            continue
        introduced.append((number, after.strip()[:100]))
    return inherited, introduced, offenders, shared


def build(write=True, station_x=STATION_X, quiet=False):
    import mujoco

    source_text = SOURCE.read_text(encoding='utf-8')
    source_sha = hashlib.sha256(source_text.encode('utf-8')).hexdigest()
    reference_text = REFERENCE.read_text(encoding='utf-8')

    # -- the measurement the station is justified by ------------------------------------------
    measured = stand_test(station_x, quiet=quiet)
    problems = check_stand(measured)
    if problems:
        raise RuntimeError('the stand test at %.5f fails: %s' % (station_x, '; '.join(problems)))

    # -- the one edit: the humanoid's station, body and keyframe -------------------------------
    text, old_pos = _set_pos(source_text, HUMANOID_BASE_BODY, (station_x, 0.0, BASE_Z))
    text, before, after, nq = rebuild_keyframe(text, ROOT / 'assets' / '_w5_h085_loop_tmp.xml')

    # -- classify every delta ------------------------------------------------------------------
    _inherited, introduced, offenders, shared = classify(source_text, reference_text, text)
    if offenders:
        print('[FAIL] %d differing line(s) are neither allowed nor inherited:' % len(offenders))
        for number, line in offenders[:5]:
            print('   %d: %s' % (number, line))
        sys.exit(3)
    if not introduced:
        raise RuntimeError('no line was introduced by this step: the build did nothing')

    # -- COMPILE BEFORE WRITING -----------------------------------------------------------------
    # `build_w5_h085_world.py` states the rule: text that does not load must not be writable. Its
    # first version checked only that the right lines had changed, and a world that could not be
    # loaded passed that check.
    tmp = ROOT / 'assets' / '_w5_h085_loop_tmp.xml'
    tmp.write_text(text, encoding='utf-8')
    final = mujoco.MjModel.from_xml_path(str(tmp))
    tmp.unlink()
    if final.nq != nq:
        raise RuntimeError('compiled nq %d, the source declared %d' % (final.nq, nq))

    # -- the compiled world must still satisfy the plant's hard requirements --------------------
    #
    # ASKED AS THE RIGHT OBJECT KIND. `c_station` is a SITE and `w2_datum_face` is a GEOM; asking
    # for either as a body returns -1 and the build reported a missing body that was present. A
    # requirement check that names the wrong kind fails on a good world, which is at least the
    # safe direction -- but it is still the wrong answer, so each is asserted against its own kind.
    required = (
        (mujoco.mjtObj.mjOBJ_ACTUATOR, ('n_wheel_left_motor', 'n_wheel_right_motor')),
        # the plant RAISES unless there are exactly two of each; presence is not enough
        (mujoco.mjtObj.mjOBJ_ACTUATOR, ('c_pusher_lift', 'c_pusher_slide')),
        (mujoco.mjtObj.mjOBJ_BODY, ('c_deck', 'c_payload', 'n_base_link', HUMANOID_BASE_BODY)),
        (mujoco.mjtObj.mjOBJ_SITE, ('c_station',)),
        (mujoco.mjtObj.mjOBJ_GEOM, ('w2_datum_face',)),
        (mujoco.mjtObj.mjOBJ_BODY, (TRAY_BODY,)),
    )
    for kind, names in required:
        missing = [name for name in names
                   if mujoco.mj_name2id(final, kind, name) < 0]
        if missing:
            raise RuntimeError('the W5 loop needs these %s objects: %s'
                               % (mujoco.mjtObj(kind).name, missing))
    for name in ('n_wheel_left_motor', 'n_wheel_right_motor', 'c_pusher_lift', 'c_pusher_slide'):
        joint = int(final.actuator_trnid[mujoco.mj_name2id(
            final, mujoco.mjtObj.mjOBJ_ACTUATOR, name), 0])
        if joint < 0:
            raise RuntimeError('actuator %s is not bound to a joint' % name)

    report = {
        'source': SOURCE.name, 'source_sha256': source_sha,
        'reference': REFERENCE.name,
        'output': OUT.name,
        'output_sha256': hashlib.sha256(text.encode('utf-8')).hexdigest(),
        'station_x_m': float(station_x),
        'station_provenance': ('H2 station, retained after a contact test in THIS world. Two '
                               'earlier measurements suggested moving it and both were '
                               'artefacts: mj_geomDistance reads visual contype=0 geoms (this '
                               'robot reports 272 self-penetrations at home, including '
                               'h_LINK_BASE against itself at -0.1674 m), and a free settle '
                               'measures the absence of a balance controller. The runtime-driven '
                               'stand test -- how H3 actually drives the robot -- shows it '
                               'standing and touching only the ground.'),
        'humanoid_base_pos_before': old_pos,
        'humanoid_base_pos_after': [float(station_x), 0.0, BASE_Z],
        'keyframe_base_qpos_before': [round(float(v), 6) for v in before],
        'keyframe_base_qpos_after': [round(float(v), 6) for v in after],
        'stand_test': measured,
        'stand_tolerances': {'drift_x_m': STAND_DRIFT_LIMIT_M, 'settle_z_m': STAND_SETTLE_LIMIT_M,
                             'allowed_contact_bodies': list(STAND_ALLOWED_CONTACT_BODIES),
                             'ticks': STAND_TICKS},
        'lines_introduced': [n for n, _ in introduced],
        'lines_shared_with_candidate_b': [n for n, _ in shared],
        'nq': final.nq, 'nbody': final.nbody, 'ngeom': final.ngeom,
        'compiled_before_write': True,
        'plant_requirements_present': True,
        'note': ('candidate B with the humanoid PLACED in it. Candidate B raised the interface and '
                 'built the presentation table but left the robot at x 0.660132, so H2 supplied '
                 'its station from inside its own probe. H3 shares one MjData between the '
                 'humanoid runtime and the W5 plant, so the station has to be the world\u2019s own '
                 'home pose. Nothing else is touched: the tray, rollers, deck, receiver and '
                 'chassis stay exactly where candidate B put them, because correcting an overlap '
                 'by moving the CHASSIS would re-decide where the W5 loop docks -- the thing H3 '
                 'integrates with, not re-negotiates.'),
    }
    if write:
        OUT.write_text(text, encoding='utf-8')
        MANIFEST.write_text(json.dumps(report, indent=2) + '\n', encoding='utf-8')
    return report


def audit(stations=()):
    """Run the runtime-driven stand test at several stations, so the number can be re-chosen from
    measurement. Each call constructs a fresh Runtime, so this is slow by design."""
    rows = []
    for x in tuple(stations) or (0.660132, 4.13, 4.20, 4.09):
        print(' station %.6f' % x)
        rows.append(stand_test(x))
        for key in ('drift_x_m', 'settle_z_m', 'ncon', 'cross_entity_bodies'):
            print('    %-22s %s' % (key, rows[-1][key]))
    return rows


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--check', action='store_true',
                    help='rebuild and compare against the file on disk; do not write')
    ap.add_argument('--audit', action='store_true',
                    help='run the runtime-driven stand test at several stations and exit')
    ap.add_argument('--station-x', type=float, default=STATION_X)
    ap.add_argument('--quiet', action='store_true')
    args = ap.parse_args()

    if args.audit:
        audit()
        return 0

    report = build(write=not args.check, station_x=args.station_x, quiet=args.quiet)
    if args.check:
        if not OUT.is_file():
            print('[FAIL] %s does not exist' % OUT)
            return 1
        same = hashlib.sha256(OUT.read_text(encoding='utf-8').encode('utf-8')).hexdigest() \
            == report['output_sha256']
        print('[%s] %s matches a fresh build' % ('OK' if same else 'FAIL', OUT.name))
        return 0 if same else 1

    for key in ('source', 'source_sha256', 'output', 'output_sha256', 'station_x_m',
                'humanoid_base_pos_before', 'humanoid_base_pos_after',
                'keyframe_base_qpos_before', 'keyframe_base_qpos_after',
                'lines_introduced', 'lines_shared_with_candidate_b', 'nq', 'nbody', 'ngeom'):
        print('  %-30s %s' % (key, report[key]))
    m = report['stand_test']
    for key in ('drift_xyz_m', 'settle_z_m', 'ncon', 'cross_entity_bodies'):
        print('  %-30s %s' % (key, m[key]))
    print('  %-30s %s' % ('cross_entity_contacts',
                          [(c['pair'], c['count']) for c in m['cross_entity_contacts']]))
    print('  source untouched: %s'
          % (hashlib.sha256(SOURCE.read_bytes()).hexdigest() == report['source_sha256']))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
