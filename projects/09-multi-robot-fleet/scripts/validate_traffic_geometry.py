#!/usr/bin/env python3
"""Cross-check the P4 traffic config against the shipped world and fleet config.

Read-only. Exit codes: 0 = all checks pass, 4 = a check failed, 3 = input missing.

Why this is a separate script from the unit tests: the unit tests assert the
*logic*, using coordinates from the config. This asserts that the coordinates
themselves agree with three files that are edited independently --

  config/resources.yaml    the rectangles and node poses
  assets/worlds/warehouse.sdf   where the walls and pads physically are
  config/fleet.yaml        the robot footprints the margin is measured against

A pass means the gate's idea of where the corridor is matches the world it is
sitting in. Hand-editing any one of the three without the others is exactly the
failure this catches.
"""

from __future__ import annotations

import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src" / "fleet_core"))

import yaml  # noqa: E402

from fleet_core import validate_fleet_config, validate_traffic_config  # noqa: E402
from fleet_core.geometry import footprint_corners  # noqa: E402

POSITION_TOL_M = 0.01
MODEL_RE = re.compile(r'<model name="([^"]+)">(.*?)</model>', re.S)
POSE_RE = re.compile(r"<pose>([^<]+)</pose>")
SIZE_RE = re.compile(r"<size>([^<]+)</size>")

failures: list[str] = []
checks = 0


def check(ok: bool, message: str) -> None:
    global checks
    checks += 1
    if not ok:
        failures.append(message)


def parse_sdf(path: Path) -> dict[str, dict]:
    out: dict[str, dict] = {}
    text = path.read_text(encoding="utf-8")
    for name, body in MODEL_RE.findall(text):
        pose = POSE_RE.search(body)
        size = SIZE_RE.search(body)
        entry: dict = {"static": "<static>true</static>" in body}
        if pose:
            entry["pose"] = [float(v) for v in pose.group(1).split()]
        if size:
            entry["size"] = [float(v) for v in size.group(1).split()]
        out[name] = entry
    return out


def main() -> int:
    resources_path = ROOT / "config" / "resources.yaml"
    fleet_path = ROOT / "config" / "fleet.yaml"
    world_path = ROOT / "assets" / "worlds" / "warehouse.sdf"
    for p in (resources_path, fleet_path, world_path):
        if not p.is_file():
            print(f"validate_traffic_geometry: missing {p}", file=sys.stderr)
            return 3

    tcfg = validate_traffic_config(yaml.safe_load(resources_path.read_text(encoding="utf-8")))
    fcfg = validate_fleet_config(yaml.safe_load(fleet_path.read_text(encoding="utf-8")))
    world = parse_sdf(world_path)

    # ---------------------------------------------------------------- bars
    for name, expect_y in (("barrier_north", 0.6), ("barrier_south", -0.6)):
        model = world.get(name)
        check(model is not None, f"world: {name} not found")
        if not model or "pose" not in model or "size" not in model:
            continue
        px, py, _, _, _, _ = (model["pose"] + [0.0] * 6)[:6]
        half = model["size"][1] / 2.0
        inner_edge = py - half if expect_y > 0 else py + half
        check(
            abs(inner_edge - expect_y) < POSITION_TOL_M,
            f"world: {name} inner edge is y={inner_edge:.4f}, expected {expect_y} "
            "(the gap the corridor rectangle must cover)",
        )

    # The corridor rectangle must contain the physical gap, not sit inside it.
    gap = tcfg.rect_named("gap")
    marker = world.get("corridor_marker")
    check(gap is not None, "resources: no rectangle named 'gap'")
    check(marker is not None, "world: corridor_marker not found")
    if gap and marker and "pose" in marker and "size" in marker:
        mx, my = marker["pose"][0], marker["pose"][1]
        sx, sy = marker["size"][0], marker["size"][1]
        check(
            gap.x_min <= mx - sx / 2 + POSITION_TOL_M and gap.x_max >= mx + sx / 2 - POSITION_TOL_M,
            f"resources: gap rectangle x[{gap.x_min},{gap.x_max}] does not cover the "
            f"corridor marker x[{mx - sx / 2},{mx + sx / 2}]",
        )
        check(
            gap.y_min <= my - sy / 2 + POSITION_TOL_M and gap.y_max >= my + sy / 2 - POSITION_TOL_M,
            f"resources: gap rectangle y[{gap.y_min},{gap.y_max}] does not cover the "
            f"corridor marker y[{my - sy / 2},{my + sy / 2}]",
        )
        check(
            gap.y_min <= -0.6 and gap.y_max >= 0.6,
            f"resources: gap rectangle y[{gap.y_min},{gap.y_max}] does not span the "
            "physical gap |y| < 0.6",
        )

    # ---------------------------------------------------------------- pads
    # Every exit buffer rectangle must actually cover its pad, and every node
    # that names a pad must be at the pad's centre.
    pad_for_node = {
        "wait_west": "pad_wait_west", "wait_east": "pad_wait_east",
        "exit_west": "pad_exit_west", "exit_east": "pad_exit_east",
        "release_west": "pad_release_west", "release_east": "pad_release_east",
    }
    for node_name, pad_name in pad_for_node.items():
        node = tcfg.nodes.get(node_name)
        pad = world.get(pad_name)
        check(node is not None, f"resources: node {node_name} not declared")
        check(pad is not None, f"world: {pad_name} not found")
        if node and pad and "pose" in pad:
            d = ((node.x - pad["pose"][0]) ** 2 + (node.y - pad["pose"][1]) ** 2) ** 0.5
            check(
                d < POSITION_TOL_M,
                f"resources: node {node_name} is {d:.4f} m from {pad_name}; the config and "
                "the world disagree about where it is",
            )

    # Both exit buffers name their own rectangle "pad", so look them up per resource.
    for res_name, pad_name in (("mid_left", "pad_exit_west"), ("mid_right", "pad_exit_east")):
        spec = tcfg.resources.get(res_name)
        pad = world.get(pad_name)
        check(spec is not None, f"resources: resource {res_name} missing")
        if not spec or not pad or "pose" not in pad or "size" not in pad:
            continue
        px, py = pad["pose"][0], pad["pose"][1]
        sx, sy = pad["size"][0], pad["size"][1]
        rect = spec.rects[0]
        check(
            rect.x_min <= px - sx / 2 + POSITION_TOL_M and rect.x_max >= px + sx / 2 - POSITION_TOL_M,
            f"resources: {res_name} rectangle x[{rect.x_min},{rect.x_max}] does not cover "
            f"{pad_name} x[{px - sx / 2},{px + sx / 2}]",
        )
        check(
            rect.y_min <= py - sy / 2 + POSITION_TOL_M and rect.y_max >= py + sy / 2 - POSITION_TOL_M,
            f"resources: {res_name} rectangle y[{rect.y_min},{rect.y_max}] does not cover "
            f"{pad_name} y[{py - sy / 2},{py + sy / 2}]",
        )

    # ------------------------------------------------------- node placement
    # Re-derived here against the *world-verified* geometry and the real
    # footprints from fleet.yaml, for every configured robot.
    from fleet_core import CrossingManager

    mgr = CrossingManager(tcfg)
    for robot_id, spec_robot in fcfg.robots.items():
        L, W = spec_robot.footprint_length_m, spec_robot.footprint_width_m
        for direction, dspec in tcfg.directions.items():
            names = mgr.bundle(direction)

            wait = tcfg.nodes.get(dspec.wait_node)
            if wait:
                corners = footprint_corners(wait.x, wait.y, wait.yaw, L, W)
                check(
                    mgr.full_region().fully_outside(corners),
                    f"{robot_id}/{direction}: wait node {dspec.wait_node} is inside the crossing "
                    "for this footprint",
                )

            align = tcfg.nodes.get(dspec.align_node)
            if align:
                corners = footprint_corners(align.x, align.y, align.yaw, L, W)
                check(
                    mgr.full_region().fully_outside(corners),
                    f"{robot_id}/{direction}: align node {dspec.align_node} is inside the crossing",
                )

            release = tcfg.nodes.get(dspec.release_node)
            if release:
                corners = footprint_corners(release.x, release.y, release.yaw, L, W)
                check(
                    mgr.region_of(names).fully_outside(corners),
                    f"{robot_id}/{direction}: release node {dspec.release_node} is still inside "
                    f"{list(names)}; a robot parked there can never prove it left",
                )

            exit_node = tcfg.nodes.get(dspec.exit_node)
            if exit_node:
                corners = footprint_corners(exit_node.x, exit_node.y, exit_node.yaw, L, W)
                check(
                    bool(mgr.region_of(names, margin_m=0.0).box_overlap(corners)),
                    f"{robot_id}/{direction}: exit node {dspec.exit_node} is not inside its "
                    "bundle; OCCUPIED could never be observed there",
                )

    # ---------------------------------------------------------------- report
    print(f"validate_traffic_geometry: {checks} checks")
    if failures:
        print(f"FAIL {len(failures)}:")
        for f in failures:
            print(f"  - {f}")
        return 4
    print("PASS: traffic config agrees with warehouse.sdf and fleet.yaml")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
