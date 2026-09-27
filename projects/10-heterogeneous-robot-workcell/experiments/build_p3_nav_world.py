"""Build the P3 navigation arena: the cell's own room, one AMR, obstacles derived from the cell.

WHY A SEPARATE WORLD FROM `world_p3_cell.xml`
---------------------------------------------
P3-NAV-02 is a single-vehicle navigation task (MASTER_PLAN, P3 exit gate: "single-vehicle
localisation and navigation, coordinate/time/sensor calibration, and loaded docking
measurement"). The five-instance cell cannot host it, for two measured reasons:

  * `n` is not at its own station. `merge_world`'s assembly parking drives it to C's station so
    that C's deck lands where C's own rig expects it, which leaves the CHASSIS standing inside
    C's fixed-roller row (chassis x 4.41372, C's station 4.41372, the fixture's rollers running
    4.41372 -> 6.25372). Driving that would drag the base through the fixture, so any arrival
    number it produced would be about the clamp, not about navigation.
  * `c_deck` is mounted 1.920000 m AHEAD of the chassis origin and 0.445 m above it (measured:
    `body_pos` local `[1.92, 0, 0.445]`), because the deck kept its own standalone x rather than
    acquiring a real mounting interface. That is the standing D041 debt, and it means the
    assembled vehicle's navigation footprint does not correspond to any physical vehicle. A
    navigation result measured on it could not be quoted.

So this arena is a DECLARED STAND-IN, and every number in it is derived from the judged world
rather than typed:

  * the ROOM (`cell_ground`, the four `cell_wall_*`) is copied verbatim out of the judged
    `world_p3_cell.xml`, so the arena is the cell's real floor plan, including its friction;
  * the ROBOT is the declared differential-drive stand-in from `assets/worlds/world_n_probe.xml`
    (P1-ENV-01: the AMR is not selected yet), with its names left unprefixed so the existing
    ROS bridge and simulator halves address it unchanged;
  * the OBSTACLES are one axis-aligned box per non-AMR instance, from that instance's measured
    world AABB in the judged world, clipped to the floor. C's movable deck and pusher are
    EXCLUDED from C's box, because those bodies ride the AMR.

WHAT THIS ARENA IS NOT
----------------------
Not the cell. It has no H, no A, no C fixture, and no second AMR -- only their FOOTPRINTS, as
obstacles. Its start pose is derived by scanning the centreline for the first position clear of
every obstacle, and is written to a sidecar JSON so the map, the AMCL initial pose and the
reported numbers all come from one place instead of three typed copies.
"""
import argparse
import copy
import hashlib
import json
import pathlib
import sys
import xml.etree.ElementTree as ET

ROOT = pathlib.Path(__file__).resolve().parents[1]

#: The judged artefact. Read, never written.
CELL = ROOT / 'assets' / 'world_p3_cell.xml'
#: The declared chassis stand-in. Read, never written.
PLANT = ROOT / 'assets' / 'worlds' / 'world_n_probe.xml'
#: The V1 tray, mounted as a declared LOAD on the chassis so the docking measurement can be
#: made loaded as well as bare. `assets/objects/tray_v1.xml` is the object itself (a single body
#: with a free joint and 11 geoms); the joint is dropped because a body cannot be a child of the
#: chassis AND free.
TRAY = ROOT / 'assets' / 'objects' / 'tray_v1.xml'
TARGET = ROOT / 'assets' / 'worlds' / 'world_p3_nav.xml'
TARGET_LOADED = ROOT / 'assets' / 'worlds' / 'world_p3_nav_loaded.xml'
LAYOUT = ROOT / 'assets' / 'worlds' / 'world_p3_nav.layout.json'

#: Clearance the start and goal poses need from every obstacle, in metres. Declared, and used by
#: the centreline scan below, so "clear" is a number rather than a judgement at the keyboard.
POSE_CLEARANCE = 0.45
#: And from the walls, so the first pose is not sitting in the costmap's inflation ring. Nav2's
#: `robot_radius` is 0.26 and its `inflation_radius` 0.35, both read from
#: `config/nav2_n_probe.yaml`; 0.70 m leaves the footprint clear of the inscribed zone.
WALL_CLEARANCE = 0.70
#: How far the obstacle column is lifted so a 2D lidar at 0.21 m can see it. Derived from the
#: scan height `make_map_n.py` uses, plus its cell size.
SCAN_HEIGHT_M = 0.21
OBSTACLE_TOP_MARGIN = 0.05

#: The instances that become obstacles, and the bodies that must NOT be counted as theirs
#: because they ride the AMR. Measured on the judged world: the deck and the pusher are the
#: V1 assembly's moving half (9 relocated bodies), and C's box is meant to be the FIXTURE.
INSTANCES = ('h', 'a', 'c')
RIDES_THE_AMR = ('c_deck', 'c_deck_roller_', 'c_pusher_', 'c_pusher')


def _prefix(el):
    return el.tag.split('}')[-1]


def _parse(path):
    """Parse one of this project's world files.

    NOT `ET.parse`: `merge_world` writes a header comment containing `--check` and
    `-- do not hand-edit`, and a `--` inside an XML comment is ILLEGAL. MuJoCo's own reader
    accepts it, so the files compile and every MuJoCo-based tool in this project is happy;
    a standard XML parser is not. Measured: `ET.parse` on world_p3_cell.xml raises
    "not well-formed (invalid token): line 2, column 44", which is the `--` in that comment.
    """
    import re
    text = path.read_text(encoding='utf-8')
    text = re.sub(r'<!--.*?-->', '', text, flags=re.S)
    return ET.fromstring(text)


def read_room():
    """The `cell_*` geoms, copied verbatim from the judged world."""
    root = _parse(CELL)
    wb = root.find('worldbody')
    out = []
    for child in wb:
        name = child.get('name') or ''
        if name.startswith('cell_'):
            out.append(copy.deepcopy(child))
    names = [c.get('name') for c in out]
    assert 'cell_ground' in names, names
    for side in ('north', 'south', 'east', 'west'):
        assert f'cell_wall_{side}' in names, f'missing cell_wall_{side}: {names}'
    return out


def read_plant():
    """The chassis subtree, its actuators, and the plant's own compiler/option/default blocks."""
    root = _parse(PLANT)
    wb = root.find('worldbody')
    furniture = {'floor', 'wall_north', 'wall_south', 'wall_east', 'wall_west',
                 'pillar_a', 'pillar_b', 'pillar_c'}
    bodies = [copy.deepcopy(c) for c in wb
              if _prefix(c) == 'body' and (c.get('name') or '') not in furniture]
    assert len(bodies) == 1, f'expected one chassis body, got {[b.get("name") for b in bodies]}'
    assert bodies[0].get('name') == 'base_link', bodies[0].get('name')
    return (bodies,
            [copy.deepcopy(a) for a in root.find('actuator')],
            copy.deepcopy(root.find('compiler')),
            copy.deepcopy(root.find('option')),
            copy.deepcopy(root.find('default')))


def measure_obstacles():
    """One box per instance, from that instance's measured AABB in the JUDGED world."""
    sys.path.insert(0, str(ROOT / 'experiments'))
    sys.path.insert(0, str(ROOT / 'src'))
    import mujoco
    import numpy as np

    m = mujoco.MjModel.from_xml_path(str(CELL))
    d = mujoco.MjData(m)
    d.qpos[:] = m.key_qpos[0]
    mujoco.mj_forward(m, d)

    def instance_of(name):
        for k in INSTANCES:
            if name.startswith(k + '_'):
                return k
        return None

    boxes = {}
    for i in range(m.nbody):
        name = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, i) or ''
        key = instance_of(name)
        if key is None:
            continue
        if any(name.startswith(p) for p in RIDES_THE_AMR):
            continue                                  # this half leaves with the AMR
        # a body's own geom extents, in the body frame, rotated into the world by mj_forward
        for g in range(m.ngeom):
            if int(m.geom_bodyid[g]) != i:
                continue
            p = np.array(d.geom_xpos[g], float)
            r = float(m.geom_rbound[g])
            lo, hi = p - r, p + r
            cur = boxes.get(key)
            boxes[key] = ((lo, hi) if cur is None
                          else (np.minimum(cur[0], lo), np.maximum(cur[1], hi)))
    out = []
    for key in INSTANCES:
        if key not in boxes:
            raise SystemExit(f'instance {key!r} produced no geoms; the obstacle derivation is '
                             f'not measuring what it thinks')
        lo, hi = boxes[key]
        z_hi = max(float(hi[2]), SCAN_HEIGHT_M + OBSTACLE_TOP_MARGIN)
        z_lo = min(0.0, float(lo[2]))
        out.append({
            'name': f'nav_obstacle_{key}', 'instance': key,
            'centre': [round(float((lo[0] + hi[0]) / 2), 6), round(float((lo[1] + hi[1]) / 2), 6),
                       round((z_lo + z_hi) / 2, 6)],
            'half': [round(float((hi[0] - lo[0]) / 2), 6), round(float((hi[1] - lo[1]) / 2), 6),
                     round((z_hi - z_lo) / 2, 6)],
        })
    return out


def room_bounds(room):
    g = next(c for c in room if c.get('name') == 'cell_ground')
    cx, cy, _ = (float(v) for v in g.get('pos').split())
    hx, hy, _ = (float(v) for v in g.get('size').split())
    return cx - hx, cx + hx, cy - hy, cy + hy


def clear_of(x, y, obstacles, margin):
    for o in obstacles:
        dx = abs(x - o['centre'][0]) - o['half'][0]
        dy = abs(y - o['centre'][1]) - o['half'][1]
        if max(dx, dy) < margin:                 # box distance, conservative
            return False
    return True


def derive_poses(room, obstacles):
    """Start = the FIRST clear centreline x from the west; goal = the LAST from the east.

    Scanned rather than chosen, so the two poses follow the measured footprints. Both are then
    checked again at full resolution; a scan that stepped over an obstacle would otherwise
    produce a start that is clear at the sampled points and not in between.
    """
    import numpy as np
    x0, x1, y0, y1 = room_bounds(room)
    need = POSE_CLEARANCE + 0.05
    xs = np.arange(x0 + WALL_CLEARANCE, x1 - WALL_CLEARANCE, 0.005)
    start = None
    for x in xs:
        if clear_of(x, 0.0, obstacles, need):
            start = float(x)
            break
    goal = None
    for x in reversed(xs):
        if clear_of(x, 0.0, obstacles, need):
            goal = float(x)
            break
    if start is None or goal is None:
        raise SystemExit('no clear centreline pose exists between the walls and the obstacles')
    assert goal > start + 1.0, f'start {start} and goal {goal} are not a drive'
    # ★ THE POSE IS WHAT MUST BE CLEAR, NOT THE LINE BETWEEN THE POSES. The first version
    # asserted every point on the straight segment start->goal, and it fired: start -0.130 and
    # goal 9.790, with instance `h`'s footprint between them. But driving AROUND an obstacle is
    # the entire content of a navigation task -- that assertion would only have been satisfied by
    # an empty arena, and it was measuring "is the corridor empty", not "are the poses usable".
    # What actually has to hold is that each pose is clear (checked here) and that a path exists
    # between them, and the second one is measured rather than asserted: `make_map_n.py` fills
    # free space by flood fill FROM THE START, so a goal the planner cannot reach shows up as an
    # occupied cell in the map, and the nav gate fails on it. Asserting it here would duplicate
    # that with a weaker method (a straight line) and call it proof.
    for label, x in (('start', start), ('goal', goal)):
        if not clear_of(x, 0.0, obstacles, POSE_CLEARANCE):
            raise SystemExit(f'the {label} pose x={x:.3f} is within {POSE_CLEARANCE} m of an '
                             f'obstacle')
    return start, goal


def _geom_z_min(g):
    """The lowest point of one geom along z, in its BODY frame.

    Per type, because the size convention differs (a box's size[2] is a half-extent, a sphere's
    size[0] is a radius, a cylinder's size[1] is a half-length), and because two of the tray's
    eleven geoms carry no `pos` at all and one is declared with `fromto`.

    An earlier version of this used `pos[2] - size[-1]` for everything and raised
    `AttributeError: 'NoneType' object has no attribute 'split'` on the `fromto` capsule -- which
    is the good outcome: the tray would otherwise have been mounted at a height derived from the
    wrong geom.
    """
    size = [float(v) for v in (g.get('size') or '0').split()]
    pos = [float(v) for v in (g.get('pos') or '0 0 0').split()]
    while len(pos) < 3:
        pos.append(0.0)
    ft = g.get('fromto')
    if ft is not None:
        f = [float(v) for v in ft.split()]
        r = size[0] if size else 0.0
        return min(f[2], f[5]) - r
    kind = g.get('type') or 'sphere'
    if kind == 'box':
        return pos[2] - (size[2] if len(size) > 2 else 0.0)
    if kind == 'sphere':
        return pos[2] - (size[0] if size else 0.0)
    if kind == 'capsule':
        return pos[2] - (size[0] + (size[1] if len(size) > 1 else 0.0))
    if kind == 'cylinder':
        return pos[2] - (size[1] if len(size) > 1 else 0.0)
    if kind == 'plane':
        return pos[2]
    return pos[2] - (max(size) if size else 0.0)


#: How far the load's underside must stay below the lidar plane. Declared, and small: the
#: point is that the sensor looks UNDER its own cargo, which is what a low-mounted lidar on
#: a real AMR does. Recorded because the real sensor/load layout is unresolved (P1-ENV-01).
LIDAR_LOAD_CLEARANCE_M = 0.03


def lidar_z(chassis):
    """The lidar site's local z, read from the plant rather than assumed."""
    for s in chassis.iter('site'):
        if s.get('name') == 'lidar_site':
            return float(s.get('pos').split()[2])
    raise SystemExit('the chassis has no `lidar_site`; the load height cannot be derived')


def tray_load(chassis):
    """The tray as a body on the chassis, at a DERIVED height, plus the numbers that justify it.

    The chassis geoms are the N plant's: a box at local z 0.060 with half-height 0.050, so its
    top face is at 0.110. The tray's lowest geom point is its floor at z -0.035 with half-thickness
    0.005, i.e. -0.040 in the tray's own body frame. So the tray body goes at
    0.110 + 0.040 = 0.150 and the floor rests exactly on the chassis. All three numbers are read
    from the two source files here rather than typed into the output.
    """
    ch = None
    for g in chassis.iter('geom'):
        if g.get('name') == 'chassis':
            ch = g
    assert ch is not None, 'the chassis has no geom named `chassis`'
    cx, cy, cz = (float(v) for v in ch.get('pos').split())
    _, _, chz = (float(v) for v in ch.get('size').split())
    top = cz + chz

    tray_root = _parse(TRAY)
    tray_body = None
    for b in tray_root.iter('body'):
        tray_body = b
    assert tray_body is not None and tray_body.get('name') == 'payload', tray_body
    lo = None
    for g in tray_body.iter('geom'):
        z = _geom_z_min(g)
        lo = z if lo is None else min(lo, z)
    assert lo is not None, 'the tray has no geoms to measure'
    # TWO requirements, and the LARGER wins. Resting the tray on the chassis is not enough:
    # the lidar sits above the chassis top, and a tray that covers it turns the arena into a
    # box with walls 0.22 m away -- measured on p3-nav-06, where the chassis then travelled
    # 0.0 m and the round reported GOAL_TIMEOUT. The first version derived the mount from the
    # chassis top alone and never mentioned the sensor.
    lz = lidar_z(chassis)
    resting = top - lo
    clearing = lz + LIDAR_LOAD_CLEARANCE_M - lo
    mount_z = max(resting, clearing)
    mount_reason = ('raised clear of the lidar' if clearing > resting
                    else 'resting on the chassis')

    body = ET.Element('body', {'name': 'nav_load', 'pos': f'0.0 0.0 {mount_z:.6f}'})
    for g in tray_body.iter('geom'):
        body.append(copy.deepcopy(g))
    mass = 0.0
    for g in tray_body.iter('geom'):
        m = g.get('mass')
        mass += float(m) if m is not None else 0.0
    return body, {'mount_z_m': round(mount_z, 6), 'chassis_top_m': round(top, 6),
                  'lidar_site_z_m': round(lz, 6),
                  'lidar_load_clearance_m': LIDAR_LOAD_CLEARANCE_M,
                  'mount_reason': mount_reason,
                  'mount_if_only_resting_m': round(resting, 6),
                  'tray_lowest_local_m': round(lo, 6), 'load_mass_kg': round(mass, 6),
                  'tray_source': str(TRAY.relative_to(ROOT)),
                  'tray_source_sha256': hashlib.sha256(TRAY.read_bytes()).hexdigest()}


def build(loaded=False):
    room = read_room()
    bodies, actuators, compiler, option, default = read_plant()
    obstacles = measure_obstacles()
    start_x, goal_x = derive_poses(room, obstacles)
    bodies[0].set('pos', f'{start_x:.6f} 0.0 0.040000')

    out = ET.Element('mujoco', {'model': 'workcell010_p3_nav'})
    out.append(compiler)
    out.append(option)
    if default is not None:
        out.append(default)
    wb = ET.SubElement(out, 'worldbody')
    for el in room:
        wb.append(el)
    for o in obstacles:
        ET.SubElement(wb, 'geom', {
            'name': o['name'], 'type': 'box',
            'pos': ' '.join(f'{v:.6f}' for v in o['centre']),
            'size': ' '.join(f'{v:.6f}' for v in o['half']),
            # The cell's own wall friction, so the arena does not invent a second surface.
            'friction': '1.0 0.005 0.0001',
        })
    load_meta = None
    for el in bodies:
        if loaded:
            child, load_meta = tray_load(el)
            el.append(child)
        wb.append(el)
    act = ET.SubElement(out, 'actuator')
    for el in actuators:
        act.append(el)

    ET.indent(out, space='  ')
    xml = '<?xml version="1.0" encoding="utf-8"?>\n' + ET.tostring(out, encoding='unicode')
    meta = {'loaded': bool(loaded), 'load': load_meta,
            'start_x': round(start_x, 6), 'goal_x': round(goal_x, 6),
                 'start': [round(start_x, 6), 0.0, 0.0], 'goal': [round(goal_x, 6), 0.0, 0.0],
            'pose_clearance_m': POSE_CLEARANCE, 'obstacles': obstacles,
            'room': list(room_bounds(room))}
    return xml, meta


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--check', action='store_true',
                    help='regenerate in memory and compare with what is on disk')
    args = ap.parse_args()

    bare_xml, bare = build(loaded=False)
    loaded_xml, loaded = build(loaded=True)
    # AN A/B NEEDS ONE VARIABLE. The two worlds must differ ONLY by the load, so the start and
    # goal the builder derives from the obstacles have to come out identical; if the load ever
    # started changing them the comparison would be measuring two different runs.
    for key in ('start_x', 'goal_x', 'start', 'goal', 'pose_clearance_m', 'obstacles', 'room'):
        assert bare[key] == loaded[key], (
            f'{key} differs between the bare and loaded arenas: {bare[key]!r} vs {loaded[key]!r} '
            f'-- then the loaded/unloaded comparison is not one variable')

    common = {
        'cell_world': CELL.name,
        'cell_world_sha256': hashlib.sha256(CELL.read_bytes()).hexdigest(),
        'plant_world': str(PLANT.relative_to(ROOT)),
        'plant_world_sha256': hashlib.sha256(PLANT.read_bytes()).hexdigest(),
    }
    bare.update(common)
    loaded.update(common)
    bare['xml'] = TARGET.name
    bare['xml_sha256'] = hashlib.sha256(bare_xml.encode()).hexdigest()
    loaded['xml'] = TARGET_LOADED.name
    loaded['xml_sha256'] = hashlib.sha256(loaded_xml.encode()).hexdigest()

    if args.check:
        ok = True
        for path, xml in ((TARGET, bare_xml), (TARGET_LOADED, loaded_xml)):
            on_disk = path.read_text(encoding='utf-8') if path.exists() else None
            same = on_disk == xml
            ok = ok and same
            print(f'--check {path.name}: {"IDENTICAL" if same else "DIFFERS"}')
        return 0 if ok else 1

    TARGET.write_text(bare_xml, encoding='utf-8')
    TARGET_LOADED.write_text(loaded_xml, encoding='utf-8')
    LAYOUT.write_text(json.dumps({'bare': bare, 'loaded': loaded}, indent=2) + '\n',
                      encoding='utf-8')

    import mujoco

    def horizon(path, nrays=181, range_max=8.0):
        """The lidar's own rays, cast at the home pose: (min, max) of what it can see."""
        import numpy as _np
        m = mujoco.MjModel.from_xml_path(str(path))
        d = mujoco.MjData(m)
        if m.nkey:
            d.qpos[:] = m.key_qpos[0]
        mujoco.mj_forward(m, d)
        base = m.body('base_link').id
        lid = m.site('lidar_site').id
        ang = _np.radians(_np.linspace(-90.0, 90.0, nrays))
        loc = _np.stack([_np.cos(ang), _np.sin(ang), _np.zeros(nrays)], axis=1)
        vec = loc @ d.xmat[base].reshape(3, 3).T
        gid = _np.zeros(nrays, dtype=_np.int32)
        dist = _np.full(nrays, _np.inf)
        mujoco.mj_multiRay(m, d, d.site_xpos[lid].copy(), vec.ravel(), None, True, base,
                           gid, dist, nrays, range_max + 1.0)
        finite = dist[_np.isfinite(dist)]
        return (float(finite.min()), float(finite.max())) if finite.size else (None, None)

    # THE LOAD MUST NOT BLIND THE SENSOR. Measured on p3-nav-06 before this row existed:
    # the loaded arena's horizon was 0.2248 m against 8.0 m bare, the chassis travelled
    # 0.0 m, and the round reported GOAL_TIMEOUT -- a sensor failure wearing a navigation
    # failure's clothes, 2.5 minutes into a run.
    hb, hl = horizon(TARGET), horizon(TARGET_LOADED)
    assert hl[1] is not None and hl[1] >= 0.5 * hb[1], (
        f'the load BLINDS the lidar: horizon {hl} in the loaded arena against {hb} bare. '
        f'The mount derivation, not the navigation, is what has to change.')
    print(f'  lidar horizon bare {hb[0]:.3f}-{hb[1]:.3f} m | loaded {hl[0]:.3f}-{hl[1]:.3f} m (the load is carried clear of the sensor)')

    for label, path, meta in (('bare  ', TARGET, bare), ('loaded', TARGET_LOADED, loaded)):
        m = mujoco.MjModel.from_xml_path(str(path))
        print(f'{label} {path.name}  sha256 {meta["xml_sha256"][:16]}  '
              f'nq {m.nq} nv {m.nv} nu {m.nu} nbody {m.nbody} ngeom {m.ngeom} '
              f'mass {float(m.body_mass.sum()):.4f} kg')
    print(f'  room x[{bare["room"][0]:.3f},{bare["room"][1]:.3f}] '
          f'y[{bare["room"][2]:.3f},{bare["room"][3]:.3f}]')
    print(f'  START x {bare["start_x"]:.6f} m   GOAL x {bare["goal_x"]:.6f} m   '
          f'(drive {bare["goal_x"] - bare["start_x"]:.3f} m)')
    for o in bare['obstacles']:
        print(f'  obstacle {o["name"]:18s} centre {o["centre"]} half {o["half"]}')
    if loaded['load']:
        L = loaded['load']
        print(f'  load: {L["tray_source"]} mounted at z {L["mount_z_m"]:.6f} m '
              f'(chassis top {L["chassis_top_m"]:.6f} - tray lowest {L["tray_lowest_local_m"]:.6f}), '
              f'mass {L["load_mass_kg"]:.6f} kg')
    print(f'  wrote {LAYOUT.name}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
