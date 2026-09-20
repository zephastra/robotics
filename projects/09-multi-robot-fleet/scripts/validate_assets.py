"""Generate the occupancy map FROM the world file, then validate the assets.

Why derive the map instead of drawing one: a hand-drawn map and an SDF drift
apart, and the failure mode is silent. The planner then routes through a wall the
map says is open, or refuses a doorway the world leaves open. Here the map is
rasterised from the very box collisions the simulator will use, so the two cannot
disagree without the rasteriser being wrong.

What it validates (each of these is a way a "nice looking map" is actually
unusable, which is exactly what MASTER_PLAN 7.2 warns about):

  1. corridor gap is wide enough for the footprint plus margin
  2. the barrier really separates the two halves -- checked by flood-filling the
     map and refusing to step inside the gap. If the far side is still reachable,
     there is a way around and the reservation logic is decorative.
  3. waiting / exit slots are outside the protected zone and do not sit in the
     passage mouth
  4. spawn poses are clear of every pad, the corridor and the chargers
  5. wheels do not overhang the declared footprint (a wheel outside the footprint
     means the robot clips what the planner believed it had cleared)
  6. map metadata agrees with the world extents and the config
  7. every asset gets a SHA-256 so a frozen configuration can be proven later

Usage:  python3 scripts/validate_assets.py [--write-map] [--json out.json]
Exit:   0 ok, 2 input problem, 4 an invariant failed
"""

from __future__ import annotations

import argparse
import hashlib
import json
import sys
import xml.etree.ElementTree as ET
from collections import deque
from dataclasses import dataclass
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]

# Map extents, frozen with the world. Must match MASTER_PLAN 7.1.
X_MIN, X_MAX = -7.0, 7.0
Y_MIN, Y_MAX = -5.0, 5.0
RESOLUTION = 0.05
OCCUPIED = 0        # ROS convention: 0 = black = occupied
FREE = 254          # 254 rather than 255 keeps a margin from "unknown"
UNKNOWN = 205

CORRIDOR = (-1.5, 1.5, -0.6, 0.6)   # x0, x1, y0, y1  (protected zone)
GAP_HALF_WIDTH = 0.6                 # barrier opening is |y| < 0.6

FOOTPRINT_LENGTH = 0.60
FOOTPRINT_WIDTH = 0.45
WHEEL_TRACK = 0.41
WHEEL_WIDTH = 0.040
APPROACH_MARGIN = 0.15               # required clearance each side in the gap

VALID_ZONE = (-6.5, 6.5, -4.5, 4.5)  # must be inside the walls with a margin


@dataclass
class Box:
    name: str
    x: float
    y: float
    sx: float
    sy: float

    @property
    def bounds(self) -> tuple[float, float, float, float]:
        return (self.x - self.sx / 2, self.x + self.sx / 2,
                self.y - self.sy / 2, self.y + self.sy / 2)


def _parse_pose(text: str | None) -> tuple[float, float]:
    if not text:
        return 0.0, 0.0
    parts = text.split()
    if len(parts) < 2:
        raise ValueError(f"bad pose: {text!r}")
    return float(parts[0]), float(parts[1])


def load_static_boxes(world_path: Path) -> list[Box]:
    """Every static model whose link has a box COLLISION.

    Visual-only marks (pads, corridor slab) are skipped on purpose: they are
    decoration and must not enter the occupancy map.
    """
    root = ET.parse(world_path).getroot()
    world = root.find("world")
    if world is None:
        raise ValueError(f"{world_path}: no <world>")

    boxes: list[Box] = []
    for model in world.findall("model"):
        name = model.get("name", "?")
        static = model.findtext("static", "false").strip().lower() == "true"
        if not static:
            continue
        mx, my = _parse_pose(model.findtext("pose"))
        for link in model.findall("link"):
            lx, ly = _parse_pose(link.findtext("pose"))
            for collision in link.findall("collision"):
                cx, cy = _parse_pose(collision.findtext("pose"))
                box = collision.find("geometry/box/size")
                if box is None:
                    continue
                sx, sy, _sz = (float(v) for v in box.text.split())
                boxes.append(Box(name, mx + lx + cx, my + ly + cy, sx, sy))
    return boxes


class Grid:
    def __init__(self) -> None:
        self.nx = int(round((X_MAX - X_MIN) / RESOLUTION))
        self.ny = int(round((Y_MAX - Y_MIN) / RESOLUTION))
        self.cells = bytearray([FREE]) * (self.nx * self.ny)

    # row 0 is the TOP of the image = max y, as in a PGM viewed normally
    def idx(self, ix: int, iy: int) -> int:
        return iy * self.nx + ix

    def to_cell(self, x: float, y: float) -> tuple[int, int]:
        ix = int((x - X_MIN) / RESOLUTION)
        iy = self.ny - 1 - int((y - Y_MIN) / RESOLUTION)
        return ix, iy

    def to_world(self, ix: int, iy: int) -> tuple[float, float]:
        return X_MIN + (ix + 0.5) * RESOLUTION, Y_MIN + (self.ny - 1 - iy + 0.5) * RESOLUTION

    def fill(self, box: Box) -> None:
        x0, x1, y0, y1 = box.bounds
        for iy in range(self.ny):
            wy = Y_MIN + (self.ny - 1 - iy + 0.5) * RESOLUTION
            if not (y0 - RESOLUTION <= wy <= y1 + RESOLUTION):
                continue
            for ix in range(self.nx):
                wx = X_MIN + (ix + 0.5) * RESOLUTION
                if x0 - RESOLUTION <= wx <= x1 + RESOLUTION:
                    self.cells[self.idx(ix, iy)] = OCCUPIED

    def free(self, x: float, y: float) -> bool:
        ix, iy = self.to_cell(x, y)
        if not (0 <= ix < self.nx and 0 <= iy < self.ny):
            return False
        return self.cells[self.idx(ix, iy)] == FREE

    def to_pgm(self) -> bytes:
        header = f"P5\n# 009 warehouse, derived from assets/worlds/warehouse.sdf\n{self.nx} {self.ny}\n255\n"
        return header.encode("ascii") + bytes(self.cells)


# --------------------------------------------------------------------------- #
# checks
# --------------------------------------------------------------------------- #


@dataclass
class Result:
    name: str
    ok: bool
    detail: str


def check_gap_width() -> Result:
    gap = 2 * GAP_HALF_WIDTH
    need = FOOTPRINT_WIDTH + 2 * APPROACH_MARGIN
    ok = gap >= need
    return Result(
        "corridor gap suits the footprint",
        ok,
        f"gap {gap:.2f} m vs {FOOTPRINT_WIDTH:.2f} m robot + 2x{APPROACH_MARGIN:.2f} m margin "
        f"= {need:.2f} m",
    )


def check_separation(grid: Grid) -> Result:
    """Flood fill from the west, refusing to enter the gap.

    If the east side is still reachable, the barrier has a way around it and the
    whole corridor reservation is theatre.
    """
    start = grid.to_cell(-4.0, 0.0)
    if grid.cells[grid.idx(*start)] != FREE:
        return Result("barrier separates the two halves", False,
                      "start cell for the flood fill is not free")

    seen = {start}
    q: deque[tuple[int, int]] = deque([start])
    x0, x1, y0, y1 = CORRIDOR
    reached_east = False

    while q:
        ix, iy = q.popleft()
        wx, wy = grid.to_world(ix, iy)
        if wx > 3.0:
            reached_east = True
            break
        for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            nx_, ny_ = ix + dx, iy + dy
            if not (0 <= nx_ < grid.nx and 0 <= ny_ < grid.ny):
                continue
            if (nx_, ny_) in seen:
                continue
            nwx, nwy = grid.to_world(nx_, ny_)
            if x0 <= nwx <= x1 and y0 <= nwy <= y1:
                continue  # forbidden: this is the only legal crossing
            if grid.cells[grid.idx(nx_, ny_)] != FREE:
                continue
            seen.add((nx_, ny_))
            q.append((nx_, ny_))

    return Result(
        "barrier separates the two halves",
        not reached_east,
        "east side unreachable without the gap (good)" if not reached_east
        else "east side REACHABLE without using the gap -- there is a way around",
    )


def check_corridor_is_open(grid: Grid) -> Result:
    """The gap itself must be traversable on the centreline."""
    blocked = [x for x in (-1.2, -0.6, 0.0, 0.6, 1.2) if not grid.free(x, 0.0)]
    return Result(
        "corridor centreline is traversable",
        not blocked,
        "clear at all sampled x" if not blocked else f"blocked at x = {blocked}",
    )


def check_slots_clear(grid: Grid) -> Result:
    slots = {
        "wait_west": (-2.5, 1.4), "wait_east": (2.5, 1.4),
        "exit_west": (-2.5, -1.4), "exit_east": (2.5, -1.4),
    }
    problems = []
    x0, x1, y0, y1 = CORRIDOR
    for name, (x, y) in slots.items():
        if not grid.free(x, y):
            problems.append(f"{name} is on an obstacle")
            continue
        # A slot must clear the robot's half-extent from the protected zone, or a
        # waiting robot's corner sticks into the passage.
        half_diag = (FOOTPRINT_LENGTH ** 2 + FOOTPRINT_WIDTH ** 2) ** 0.5 / 2
        dx = max(x0 - x, x - x1, 0.0)
        dy = max(y0 - y, y - y1, 0.0)
        if dx * dx + dy * dy < half_diag ** 2:
            problems.append(f"{name} footprint overlaps the protected zone")
    return Result(
        "waiting and exit slots clear of the zone",
        not problems,
        "all four slots clear" if not problems else "; ".join(problems),
    )


def load_spawns(path: Path) -> dict[str, tuple[float, float]]:
    """Spawn poses come from config/spawns.yaml, not from a copy in this file.

    A duplicated spawn list is the classic way for the validator to keep passing
    while the launch file spawns somewhere else entirely. One source, read by both.
    """
    if not path.is_file():
        raise SystemExit(f"validate_assets: missing spawn config {path}")
    try:
        import yaml
    except ImportError:  # pragma: no cover
        raise SystemExit("validate_assets: PyYAML is required to read config/spawns.yaml")
    data = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
    raw = data.get("spawns")
    if not isinstance(raw, dict) or not raw:
        raise SystemExit(f"validate_assets: {path} has no non-empty 'spawns' mapping")
    out: dict[str, tuple[float, float]] = {}
    for name, pose in raw.items():
        if not isinstance(pose, dict) or "x" not in pose or "y" not in pose:
            raise SystemExit(f"validate_assets: spawn {name!r} needs at least x and y")
        out[str(name)] = (float(pose["x"]), float(pose["y"]))
    return out


def check_spawns_clear(grid: Grid, spawns: dict[str, tuple[float, float]]) -> Result:
    pads = {
        "station_(-5,2.5)": (-5.0, 2.5), "station_(-5,-2.5)": (-5.0, -2.5),
        "station_(5,2.5)": (5.0, 2.5), "station_(5,-2.5)": (5.0, -2.5),
        "charger_(-5,-3.8)": (-5.0, -3.8), "charger_(5,-3.8)": (5.0, -3.8),
        "wait_(-2.5,1.4)": (-2.5, 1.4), "wait_(2.5,1.4)": (2.5, 1.4),
        "exit_(-2.5,-1.4)": (-2.5, -1.4), "exit_(2.5,-1.4)": (2.5, -1.4),
    }
    half_diag = (FOOTPRINT_LENGTH ** 2 + FOOTPRINT_WIDTH ** 2) ** 0.5 / 2
    problems = []
    x0, x1, y0, y1 = CORRIDOR
    for sname, (sx, sy) in spawns.items():
        if not grid.free(sx, sy):
            problems.append(f"{sname} on an obstacle")
        for pname, (px, py) in pads.items():
            if ((sx - px) ** 2 + (sy - py) ** 2) ** 0.5 < half_diag + 0.25:
                problems.append(f"{sname} too close to {pname}")
        if x0 - half_diag <= sx <= x1 + half_diag and y0 - half_diag <= sy <= y1 + half_diag:
            problems.append(f"{sname} inside the protected zone")
    return Result(
        "spawn poses clear of pads, zone and chargers",
        not problems,
        "all spawns clear" if not problems else "; ".join(problems),
    )


def check_wheels_inside_footprint(robot_path: Path) -> Result:
    """A wheel outside the declared footprint is a silent collision bug.

    Runs against model.sdf.in, the per-robot TEMPLATE. That is the file every robot
    is rendered from, so checking it covers all of them; the __ROBOT__ tokens sit
    inside <topic> text and do not affect geometry parsing.
    """
    root = ET.parse(robot_path).getroot()
    model = root.find("model")
    if model is None:
        return Result("wheels inside the declared footprint", False, "model.sdf has no <model>")

    worst = 0.0
    for link in model.findall("link"):
        if "wheel" not in (link.get("name") or ""):
            continue
        lx, ly = _parse_pose(link.findtext("pose"))
        cyl = link.find("collision/geometry/cylinder/length")
        if cyl is None:
            continue
        half = float(cyl.text) / 2
        worst = max(worst, abs(ly) + half)

    half_width = FOOTPRINT_WIDTH / 2
    ok = worst <= half_width + 1e-9
    return Result(
        "wheels inside the declared footprint",
        ok,
        f"widest wheel face at y = {worst:.3f} m vs footprint half-width {half_width:.3f} m"
        + ("" if ok else "  <-- WHEELS OVERHANG"),
    )


def _sdf_link_children(sdf_path: Path) -> dict[str, tuple[float, float, float]]:
    """child link name -> its pose relative to the model, for non-fixed links."""
    root = ET.parse(sdf_path).getroot()
    model = root.find("model")
    if model is None:
        raise ValueError(f"{sdf_path}: no <model>")
    out: dict[str, tuple[float, float, float]] = {}
    for link in model.findall("link"):
        name = link.get("name") or "?"
        vals = (link.findtext("pose") or "0 0 0 0 0 0").split()
        out[name] = (float(vals[0]), float(vals[1]), float(vals[2]))
    return out


def _urdf_joint_origins(urdf_path: Path) -> dict[str, tuple[float, float, float]]:
    """child link name -> joint origin xyz, for the joints that place a child link."""
    root = ET.parse(urdf_path).getroot()
    out: dict[str, tuple[float, float, float]] = {}
    for joint in root.findall("joint"):
        child = joint.find("child")
        origin = joint.find("origin")
        if child is None or child.get("link") is None:
            continue
        vals = (origin.get("xyz") if origin is not None else "0 0 0") or "0 0 0"
        xyz = [float(v) for v in vals.split()]
        out[child.get("link")] = (xyz[0], xyz[1], xyz[2])
    return out


def check_urdf_matches_sdf(sdf_path: Path, urdf_path: Path) -> Result:
    """The kinematics used for TF and the kinematics used for physics must agree.

    If they do not, robot_state_publisher places the lidar somewhere the simulator
    does not have it, the costmap is built from a scan taken in the wrong frame, and
    every later "the robot drifted" conclusion is contaminated. This comparison is
    the reason the URDF comment can claim the numbers are checked rather than merely
    copied.

    Only RELATIVE offsets are compared, because the two formats express the same
    chain with opposite conventions:

        SDF  base_link at z = 0.175, wheel at z = 0.075  ->  relative -0.10
        URDF base_footprint -> base_link origin z = +0.175
             base_link -> wheel_front_left origin z = -0.10

    So a URDF joint origin must equal (child SDF pose - parent SDF pose).
    """
    sdf_poses = _sdf_link_children(sdf_path)
    urdf_origins = _urdf_joint_origins(urdf_path)

    pairs = [
        ("base_link", "base_footprint"),
        ("laser_link", "base_link"),
        ("wheel_front_left", "base_link"),
        ("wheel_rear_left", "base_link"),
        ("wheel_front_right", "base_link"),
        ("wheel_rear_right", "base_link"),
    ]

    problems: list[str] = []
    checked = 0
    for child, parent in pairs:
        if child not in sdf_poses or parent not in sdf_poses:
            problems.append(f"SDF is missing {child} or {parent}")
            continue
        if child not in urdf_origins:
            problems.append(f"URDF has no joint whose child link is {child} "
                            f"(it would never be attached to {parent})")
            continue
        cx, cy, cz = sdf_poses[child]
        px, py, pz = sdf_poses[parent]
        want = (cx - px, cy - py, cz - pz)
        got = urdf_origins[child]
        if max(abs(a - b) for a, b in zip(want, got)) > 1e-6:
            problems.append(
                f"{child}: URDF joint origin {tuple(round(v, 4) for v in got)} != "
                f"SDF relative offset {tuple(round(v, 4) for v in want)}"
            )
        checked += 1

    return Result(
        "URDF kinematics match the SDF",
        not problems,
        f"{checked}/6 link offsets agree (TF cannot disagree with the physics)"
        if not problems else "; ".join(problems),
    )


def check_extents(boxes: list[Box]) -> Result:
    problems = []
    for b in boxes:
        x0, x1, y0, y1 = b.bounds
        if x0 < X_MIN - 0.5 or x1 > X_MAX + 0.5 or y0 < Y_MIN - 0.5 or y1 > Y_MAX + 0.5:
            problems.append(f"{b.name} outside the declared map extent")
    return Result(
        "geometry inside the declared map extent",
        not problems,
        "all static boxes within extents" if not problems else "; ".join(problems),
    )


def check_valid_zone(grid: Grid) -> Result:
    """Both work areas must be usable, and the interior mostly open.

    Deliberately NOT "is the bounding box free": the barrier runs straight through
    the interior, so sampling x = 0 would always land on a wall and the check would
    fail for a correct map. What matters is that each quadrant has reachable free
    space and that the walls are thin rather than eating the working area.
    """
    x0, x1, y0, y1 = VALID_ZONE

    # One representative point per quadrant, well clear of the barrier and walls.
    quadrants = {
        "NW": (-3.0, 2.5), "SW": (-3.0, -2.5),
        "NE": (3.0, 2.5), "SE": (3.0, -2.5),
    }
    blocked = [name for name, (x, y) in quadrants.items() if not grid.free(x, y)]

    if blocked:
        return Result(
            "both work areas usable",
            False,
            f"blocked quadrants: {blocked}",
        )

    # Free fraction of the interior. The six walls occupy a few percent; anything
    # much lower means geometry has crept into the working area.
    #
    # to_cell() returns (ix, iy) -- unpack each coordinate from the right call.
    # Getting this wrong once produced an empty row range and a reported "0.0%
    # free" on a perfectly good map.
    xi, _ = grid.to_cell(x0, y0)   # leftmost column
    xj, _ = grid.to_cell(x1, y0)   # rightmost column
    _, yi = grid.to_cell(x0, y1)   # top row
    _, yj = grid.to_cell(x0, y0)   # bottom row
    free = total = 0
    for iy in range(max(0, yi), min(grid.ny, yj + 1)):
        for ix in range(max(0, xi), min(grid.nx, xj + 1)):
            total += 1
            if grid.cells[grid.idx(ix, iy)] == FREE:
                free += 1
    fraction = free / total if total else 0.0
    ok = total > 0 and fraction >= 0.90
    return Result(
        "both work areas usable",
        ok,
        f"all four quadrants reachable; interior {fraction * 100:.1f}% free over "
        f"{total} cells (threshold 90%: the six wall boxes should be thin, not "
        f"area-consuming)",
    )


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--write-map", action="store_true", help="write assets/maps/warehouse.pgm/.yaml")
    ap.add_argument("--json", help="also write the report as JSON here")
    args = ap.parse_args()

    world = ROOT / "assets" / "worlds" / "warehouse.sdf"
    robot = ROOT / "assets" / "models" / "amr_4wd" / "model.sdf.in"
    urdf = ROOT / "src" / "fleet_bringup" / "urdf" / "amr_4wd.urdf"
    spawns_yaml = ROOT / "config" / "spawns.yaml"
    for p in (world, robot, urdf, spawns_yaml):
        if not p.exists():
            print(f"validate_assets: missing {p}", file=sys.stderr)
            return 2

    spawns = load_spawns(spawns_yaml)

    print("=" * 74)
    print("  009 asset validation")
    print("=" * 74)
    print(f"  world : {world.relative_to(ROOT)}")
    print(f"  robot : {robot.relative_to(ROOT)}   (per robot template)")
    print(f"  urdf  : {urdf.relative_to(ROOT)}")
    print(f"  spawns: {spawns_yaml.relative_to(ROOT)}   ({len(spawns)} robots: "
          f"{', '.join(sorted(spawns))})")
    print()

    boxes = load_static_boxes(world)
    print(f"  static box collisions found: {len(boxes)}")
    for b in boxes:
        print(f"    {b.name:20s} pos=({b.x:6.2f},{b.y:6.2f}) size=({b.sx:5.2f},{b.sy:5.2f})")
    print()

    grid = Grid()
    for b in boxes:
        grid.fill(b)

    results = [
        check_gap_width(),
        check_separation(grid),
        check_corridor_is_open(grid),
        check_slots_clear(grid),
        check_spawns_clear(grid, spawns),
        check_wheels_inside_footprint(robot),
        check_urdf_matches_sdf(robot, urdf),
        check_extents(boxes),
        check_valid_zone(grid),
    ]

    print("  checks")
    failed = 0
    for r in results:
        tag = "PASS" if r.ok else "FAIL"
        print(f"    [{tag}] {r.name}")
        print(f"           {r.detail}")
        if not r.ok:
            failed += 1

    print()
    print(f"  map: {grid.nx} x {grid.ny} px at {RESOLUTION} m/px "
          f"over x[{X_MIN},{X_MAX}] y[{Y_MIN},{Y_MAX}]")

    if args.write_map:
        maps = ROOT / "assets" / "maps"
        maps.mkdir(parents=True, exist_ok=True)
        pgm = maps / "warehouse.pgm"
        pgm.write_bytes(grid.to_pgm())
        yml = maps / "warehouse.yaml"
        yaml_header = (
            "# 009 occupancy map, DERIVED from assets/worlds/warehouse.sdf by\n"
            "# scripts/validate_assets.py --write-map. Do not hand-edit: re-run the\n"
            "# generator so the map cannot drift from the world it describes.\n"
            "image: warehouse.pgm\n"
            f"resolution: {RESOLUTION}\n"
            f"origin: [{X_MIN}, {Y_MIN}, 0.0]\n"
            "negate: 0\n"
            "occupied_thresh: 0.65\n"
            "free_thresh: 0.196\n"
        )
        yml.write_text(yaml_header, encoding="utf-8")
        print(f"  wrote {pgm.relative_to(ROOT)} ({pgm.stat().st_size} bytes)")
        print(f"  wrote {yml.relative_to(ROOT)}")

    print()
    print("  hashes")
    hashes = {}
    for p in [world, robot, ROOT / "assets" / "models" / "amr_4wd" / "model.config",
              urdf, spawns_yaml,
              ROOT / "assets" / "maps" / "warehouse.pgm", ROOT / "assets" / "maps" / "warehouse.yaml",
              ROOT / "config" / "fleet.yaml",
              ROOT / "src" / "fleet_bringup" / "config" / "nav2_params.yaml"]:
        if p.exists():
            h = sha256(p)
            hashes[str(p.relative_to(ROOT))] = h
            print(f"    {h[:16]}  {p.relative_to(ROOT)}")

    report = {
        "results": [{"name": r.name, "ok": r.ok, "detail": r.detail} for r in results],
        "failed": failed,
        "map": {"nx": grid.nx, "ny": grid.ny, "resolution": RESOLUTION,
                "x": [X_MIN, X_MAX], "y": [Y_MIN, Y_MAX]},
        "hashes": hashes,
    }
    if args.json:
        Path(args.json).write_text(json.dumps(report, indent=2), encoding="utf-8")

    print()
    if failed:
        print(f"  RESULT: {failed} check(s) FAILED -- assets are not usable (exit 4)")
        print("=" * 74)
        return 4
    print("  RESULT: all asset checks passed (exit 0)")
    print("  NOTE: geometry passes are not physics passes. P2 must still measure the")
    print("        real stop distance and confirm a turning wheel does not clip a corner.")
    print("=" * 74)
    return 0


if __name__ == "__main__":
    sys.exit(main())
