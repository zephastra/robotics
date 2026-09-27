"""Build the P1-N static map from the world it is a map of, and cast rays through it.

Why a generated map instead of a hand-drawn one, and why that is not "truth leaking into
localisation":

  * The arena is 010-authored and static. A floor plan of a known cell is an *operator prior*,
    the same object a real deployment loads from a surveyed drawing. It is not a runtime
    measurement, and it is not the movable-payload truth that AGENTS.md forbids feeding to
    localisation, vision or planning.
  * It is *generated from the world file rather than typed*, so the two cannot disagree, and
    `--check` fails if the world changes without the map being regenerated. A hand-drawn PGM
    would silently disagree with the world the moment a pillar moved.
  * What it takes away: N-07 cannot claim SLAM or mapping. The report records
    `map.source = "world_geometry_prior"` and the judge says so explicitly.

Rules that make the rasterisation unambiguous, each of which raises rather than guessing:

  * Only geoms that are direct children of `<worldbody>` are mapped. A geom inside a `<body>`
    belongs to an assembly that moves, so it is not part of a static map. The names of the
    skipped geoms are written into the map metadata, so adding a static geom inside a body
    changes the generated file and `--check` forces a human to look at it.
  * A geom with `quat`, `euler`, `zaxis` or `fromto` is refused: this rasteriser only handles
    axis-aligned boxes and upright cylinders, and silently mis-drawing a rotated obstacle is
    exactly the failure a map must not have.
  * Type `plane` is the floor, not an obstacle. Any other geom type is refused.
  * A geom is included only if its vertical extent contains the scan height. A map for a 2D
    lidar is a list of what the lidar can hit; a 3 cm curb casts no shadow at 0.21 m and would
    make map and scan disagree.

The occupancy grid is 0 (free) / 100 (occupied), with no unknown cell: free space is the
*reachable* space from the start pose by flood fill, so the band outside the walls (which no
path can enter) is written occupied instead of free. That needs no hardcoded wall coordinates.

The grid is laid out so that the start pose sits exactly at the centre of a cell. With
`origin = start - (n + 0.5) * resolution` and `2n + 1` cells, a raycast that samples cell
centres agrees with the continuous geometry instead of being offset by half a cell.
"""
import argparse
import hashlib
import math
import re
import xml.etree.ElementTree as ElementTree
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]

FREE = 0
OCCUPIED = 100

UNSUPPORTED_ATTRS = ('quat', 'euler', 'zaxis', 'fromto')
SUPPORTED_TYPES = ('plane', 'box', 'cylinder')


def read_world_geoms(world_path):
    """Split the world's geoms into static ones (mapped) and movable ones (skipped).

    Comments are stripped before parsing. MuJoCo's own parser accepts `--` inside an XML
    comment; XML does not, so ElementTree rejects the world file as not well-formed at the
    first `--` in the prose. Stripping comments is also the honest thing for a rasteriser:
    the comments are documentation, and nothing in them should influence the map.
    """
    text = Path(world_path).read_text(encoding='utf-8')
    stripped = re.sub(r'<!--.*?-->', '', text, flags=re.S)
    tree = ElementTree.ElementTree(ElementTree.fromstring(stripped))
    worldbody = tree.getroot().find('worldbody')
    if worldbody is None:
        raise SystemExit(f'{world_path}: no <worldbody>')

    static, moving = [], []

    def walk(node, in_body):
        for child in node:
            if child.tag == 'body':
                walk(child, True)
            elif child.tag == 'geom':
                name = child.get('name') or '<unnamed>'
                if in_body:
                    # Belongs to a movable assembly (the robot's wheels carry a quat to lie on
                    # their side). It is skipped, so its attributes are not this file's
                    # business; its NAME is recorded so a new geom inside a body shows up as a
                    # diff in the generated map instead of being silently ignored.
                    moving.append({'name': name})
                    continue
                geom_type = child.get('type', 'sphere')
                if geom_type not in SUPPORTED_TYPES:
                    raise SystemExit(f'{name}: geom type {geom_type!r} is not mapped by this '
                                     f'rasteriser (supported: {SUPPORTED_TYPES})')
                bad = [a for a in UNSUPPORTED_ATTRS if child.get(a) is not None]
                if bad:
                    raise SystemExit(f'{name}: attribute(s) {bad} unsupported; this rasteriser '
                                     f'only handles axis-aligned, upright geoms. Handle it '
                                     f'explicitly rather than mapping it in the wrong place.')
                entry = {'name': name, 'type': geom_type,
                         'pos': [float(v) for v in (child.get('pos') or '0 0 0').split()],
                         'size': [float(v) for v in (child.get('size') or '0').split()]}
                static.append(entry)
            elif child.tag in ('joint', 'freejoint', 'site', 'camera', 'light', 'include'):
                continue
            else:
                walk(child, in_body)

    walk(worldbody, False)
    return static, moving


def geom_z_range(geom):
    """World-space (zmin, zmax) of a static geom."""
    pos_z = geom['pos'][2] if len(geom['pos']) > 2 else 0.0
    size = geom['size']
    if geom['type'] == 'plane':
        return (-math.inf, math.inf)
    if geom['type'] == 'box':
        half = size[2] if len(size) > 2 else 0.0
        return (pos_z - half, pos_z + half)
    if geom['type'] == 'cylinder':
        half = size[1] if len(size) > 1 else 0.0
        return (pos_z - half, pos_z + half)
    raise SystemExit(f"{geom['name']}: no z-range rule for type {geom['type']!r}")


def contains_xy(geom, x, y):
    cx, cy = geom['pos'][0], geom['pos'][1]
    if geom['type'] == 'plane':
        return False                      # the floor carries the robot, it is not an obstacle
    if geom['type'] == 'box':
        return abs(x - cx) <= geom['size'][0] and abs(y - cy) <= geom['size'][1]
    if geom['type'] == 'cylinder':
        return math.hypot(x - cx, y - cy) <= geom['size'][0]
    raise SystemExit(f"{geom['name']}: no footprint rule for type {geom['type']!r}")


def to_cell(x, y, origin, resolution, height):
    """World metres -> (row, col). Row 0 is the TOP of the image, as map_server expects."""
    col = int(math.floor((x - origin[0]) / resolution))
    row = height - 1 - int(math.floor((y - origin[1]) / resolution))
    return row, col


def to_world(row, col, origin, resolution, height):
    """(row, col) -> the world metres at the CENTRE of that cell."""
    x = origin[0] + (col + 0.5) * resolution
    y = origin[1] + (height - 1 - row + 0.5) * resolution
    return x, y


def build_grid(world_path, *, half_extent_m=4.5, resolution=0.05,
               scan_height_m=0.21, start=(0.0, 0.0)):
    """Rasterise the world, then keep only the free space reachable from `start`."""
    n = int(round(half_extent_m / resolution))
    if n < 1:
        raise SystemExit(f'half extent {half_extent_m} is smaller than one cell')
    width = height = 2 * n + 1
    origin = (start[0] - (n + 0.5) * resolution, start[1] - (n + 0.5) * resolution)

    static, moving = read_world_geoms(world_path)
    floor, included, excluded_by_height = [], [], []
    for geom in static:
        if geom['type'] == 'plane':
            floor.append(geom['name'])
            continue
        zmin, zmax = geom_z_range(geom)
        if zmin <= scan_height_m < zmax:
            included.append(geom)
        else:
            excluded_by_height.append(geom['name'])

    grid = bytearray(width * height)
    for row in range(height):
        for col in range(width):
            x, y = to_world(row, col, origin, resolution, height)
            for geom in included:
                if contains_xy(geom, x, y):
                    grid[row * width + col] = OCCUPIED
                    break

    start_row, start_col = to_cell(start[0], start[1], origin, resolution, height)
    if (start_row, start_col) != (n, n):
        raise SystemExit(f'start pose {start} maps to cell ({start_row}, {start_col}), '
                         f'expected the centre cell ({n}, {n}); origin maths is wrong')
    if grid[start_row * width + start_col] != FREE:
        raise SystemExit(f'start pose {start} lands on an occupied cell; the map is wrong')

    # Flood fill: everything not reachable from the start pose becomes occupied too, so the
    # map never offers a path into the band outside the walls.
    reachable = bytearray(width * height)
    stack = [(start_row, start_col)]
    reachable[start_row * width + start_col] = 1
    while stack:
        row, col = stack.pop()
        for drow, dcol in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            nrow, ncol = row + drow, col + dcol
            if not (0 <= nrow < height and 0 <= ncol < width):
                continue
            index = nrow * width + ncol
            if reachable[index] or grid[index] != FREE:
                continue
            reachable[index] = 1
            stack.append((nrow, ncol))
    for index in range(width * height):
        if grid[index] == FREE and not reachable[index]:
            grid[index] = OCCUPIED

    return {'grid': grid, 'width': width, 'height': height, 'resolution': resolution,
            'origin': origin, 'start': list(start), 'scan_height_m': scan_height_m,
            'floor_geoms': floor,
            'included_geoms': [g['name'] for g in included],
            'excluded_by_scan_height': sorted(excluded_by_height),
            'skipped_moving_geoms': sorted(g['name'] for g in moving),
            'free_cells': sum(1 for v in grid if v == FREE),
            'occupied_cells': sum(1 for v in grid if v == OCCUPIED)}


def pgm_bytes(built):
    """P5 PGM: occupied is black (0), free is white (254)."""
    width, height, grid = built['width'], built['height'], built['grid']
    body = bytearray(b'\xfe' * (width * height))
    for index, value in enumerate(grid):
        if value == OCCUPIED:
            body[index] = 0
    return b'P5\n%d %d\n255\n' % (width, height) + bytes(body)


def yaml_text(built, *, stem, world_relpath, world_sha256, generator_sha256):
    xmin, ymin = built['origin']
    lines = [
        '# P1-N static map of the N probe arena. GENERATED -- do not hand-edit.',
        '#',
        '# Regenerate:  .venv/bin/python experiments/make_map_n.py',
        '# Verify:      .venv/bin/python experiments/make_map_n.py --check',
        '#',
        '# It is a floor plan of a world 010 itself authors, i.e. an operator prior, NOT a',
        '# measurement and NOT SLAM output. N-07 therefore validates map -> AMCL -> planner ->',
        '# controller -> command gate; it says nothing about mapping.',
        f'# source world: {world_relpath}',
        f'# source world sha256: {world_sha256}',
        f'# generator sha256: {generator_sha256}',
        f"# start pose used for the reachability fill, cell centre by construction: "
        f"{built['start']}",
        f"# scan height: {built['scan_height_m']} m",
        f"# floor geoms (never obstacles): {', '.join(built['floor_geoms']) or 'none'}",
        f"# geoms mapped: {', '.join(built['included_geoms'])}",
        "# geoms below/above the scan height (skipped): "
        f"{', '.join(built['excluded_by_scan_height']) or 'none'}",
        "# geoms inside a <body> (movable assemblies, skipped): "
        f"{', '.join(built['skipped_moving_geoms']) or 'none'}",
        f"# cells: {built['free_cells']} free, {built['occupied_cells']} occupied, 0 unknown",
        '#',
        f'image: {stem}.pgm',
        f"resolution: {built['resolution']}",
        f'origin: [{xmin}, {ymin}, 0.0]',
        'negate: 0',
        'occupied_thresh: 0.65',
        'free_thresh: 0.196',
        '',
    ]
    return '\n'.join(lines)


def read_map(yaml_path):
    """Load a generated map back, for the judge and for tests."""
    import yaml
    yaml_path = Path(yaml_path)
    meta = yaml.safe_load(yaml_path.read_text(encoding='utf-8'))
    image = yaml_path.parent / meta['image']
    raw = image.read_bytes()
    if not raw.startswith(b'P5'):
        raise SystemExit(f'{image}: not a binary PGM')
    fields, offset = [], 2
    while len(fields) < 3:
        while raw[offset:offset + 1].isspace():
            offset += 1
        start = offset
        while not raw[offset:offset + 1].isspace():
            offset += 1
        fields.append(int(raw[start:offset]))
    offset += 1
    width, height, maxval = fields
    if maxval != 255:
        raise SystemExit(f'{image}: maxval {maxval}, expected 255')
    body = raw[offset:offset + width * height]
    if len(body) != width * height:
        raise SystemExit(f'{image}: {len(body)} bytes of pixel data, expected {width * height}')
    grid = bytearray(width * height)
    for index, value in enumerate(body):
        grid[index] = OCCUPIED if value == 0 else FREE
    origin = meta['origin']
    return {'grid': grid, 'width': width, 'height': height,
            'resolution': float(meta['resolution']), 'origin': (origin[0], origin[1]),
            'meta': meta}


def raycast_grid(built, pose, angles, range_max, step_m=0.01):
    """Ranges from `pose` = (x, y, yaw) out along `angles`, by stepping through the grid.

    This is the independent side of the map/scan consistency check: it looks ONLY at the map
    image, so a wrong origin, a wrong resolution or a flipped image would fail to reproduce
    the simulator's rays through the real scene.
    """
    grid, width, height = built['grid'], built['width'], built['height']
    origin, resolution = built['origin'], built['resolution']
    steps = int(range_max / step_m) + 1
    ranges = []
    for angle in angles:
        heading = pose[2] + angle
        dx, dy = math.cos(heading), math.sin(heading)
        hit = range_max
        for k in range(1, steps + 1):
            distance = k * step_m
            if distance > range_max:
                break
            row, col = to_cell(pose[0] + dx * distance, pose[1] + dy * distance,
                               origin, resolution, height)
            if not (0 <= row < height and 0 <= col < width):
                hit = distance
                break
            if grid[row * width + col] == OCCUPIED:
                hit = distance
                break
        ranges.append(hit)
    return ranges


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--world', default=None)
    parser.add_argument('--out-dir', default=None, help='default <root>/assets/maps')
    parser.add_argument('--stem', default='arena_n_probe')
    parser.add_argument('--resolution', type=float, default=0.05)
    parser.add_argument('--half-extent-m', type=float, default=4.5)
    parser.add_argument('--scan-height-m', type=float, default=0.21)
    parser.add_argument('--start', nargs=2, type=float, default=[0.0, 0.0])
    parser.add_argument('--check', action='store_true',
                        help='regenerate in memory and compare with what is on disk')
    args = parser.parse_args()

    import yaml
    cfg = yaml.safe_load((ROOT / 'config' / 'n_probe.yaml').read_text(encoding='utf-8'))
    world = ROOT / (args.world or cfg['world'])
    out_dir = Path(args.out_dir) if args.out_dir else ROOT / 'assets' / 'maps'

    built = build_grid(world, half_extent_m=args.half_extent_m,
                       resolution=args.resolution, scan_height_m=args.scan_height_m,
                       start=tuple(args.start))
    world_relpath = str(world.relative_to(ROOT))
    new_pgm = pgm_bytes(built)
    new_yaml = yaml_text(built, stem=args.stem, world_relpath=world_relpath,
                         world_sha256=hashlib.sha256(world.read_bytes()).hexdigest(),
                         generator_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest())
    pgm_path = out_dir / f'{args.stem}.pgm'
    yaml_path = out_dir / f'{args.stem}.yaml'

    print(yaml.safe_dump({k: v for k, v in built.items() if k != 'grid'},
                         sort_keys=False, allow_unicode=True), flush=True)

    if args.check:
        problems = []
        for path, expected in ((pgm_path, new_pgm), (yaml_path, new_yaml.encode())):
            if not path.is_file():
                problems.append(f'missing: {path}')
            elif path.read_bytes() != expected:
                problems.append(f'out of date: {path}')
        if problems:
            for problem in problems:
                print(f'[FAIL] {problem}', flush=True)
            print('[FAIL] regenerate without --check, then review the diff', flush=True)
            return 1
        print(f'[OK] {pgm_path.name} and {yaml_path.name} are current '
              f'({built["free_cells"]} free / {built["occupied_cells"]} occupied)', flush=True)
        return 0

    out_dir.mkdir(parents=True, exist_ok=True)
    pgm_path.write_bytes(new_pgm)
    yaml_path.write_text(new_yaml, encoding='utf-8')
    print(f'[OK] wrote {pgm_path} ({len(new_pgm)} bytes) and {yaml_path}', flush=True)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
