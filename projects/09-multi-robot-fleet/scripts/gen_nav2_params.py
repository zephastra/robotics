#!/usr/bin/env python3
"""Derive config/nav2_params.yaml for the 009 fleet from the INSTALLED stock Nav2 file.

Why derive instead of writing a config from scratch: the stock file is guaranteed to
match the Nav2 build that is actually on this machine (plugin class names, parameter
names and defaults all move between Nav2 releases). A hand written config that
"looks right" is the classic way to spend a day debugging a typo in
`nav2_mppi_controller::MPPIController`. So the rule is: copy what ships, then change
only the things that are provably wrong for THIS robot and THIS world.

Every edit below is asserted. If a pattern is not found, this exits non zero instead
of quietly producing a config that is missing a fix. A silent no-op here would mean
the robot keeps limits the plant cannot honour, which shows up much later as "Nav2
thinks it is tracking the path but the robot is somewhere else".

Edits, and why each one is necessary rather than cosmetic:

  footprint              stock ships `robot_radius: 0.22`. This robot is 0.60 x 0.45.
                         A radius of 0.22 under covers the corners, so the planner
                         believes it fits through gaps it does not fit through.
  inflation_radius       stock ships 0.70, tuned for open spaces. The protected
                         corridor is a 1.2 m gap. Inflating 0.70 m from each side
                         makes the gap prohibitively expensive and the planner will
                         refuse a route that physically exists.
  velocity limits        stock ships 0.5 m/s / 2.0 rad/s. The DiffDrive plugin in
                         model.sdf.in caps the plant at 0.35 m/s / 0.6 rad/s. If the
                         smoother asks for more, gz silently clamps and Nav2's
                         tracking error looks like a control failure when it is
                         really a configuration mismatch.
  acceleration limits    same argument: stock 2.5 / 3.2 against a plant that does
                         0.5 / 1.0.
  cmd_vel_out_topic      collision_monitor must NOT be the last publisher of
                         cmd_vel: the safety gate owns that topic. Renaming the
                         monitor's output removes the possibility of two publishers
                         on cmd_vel and a feedback loop through the smoother.
  laser_max_range        100.0 m against a 12 m lidar.
  frame prefixes         the stock file uses bare `odom` / `base_link`. With two
                         robots in one TF tree, bare frame names collide, so every
                         frame becomes __NS__/<frame>. The launch file substitutes
                         __NS__; nothing here is left as a bare frame name.
  initial pose           set from the spawn pose so localisation does not depend on
                         a human clicking "2D Pose Estimate" in RViz.
  amcl update_min_d      stock ships 0.25 m, the distance the robot must travel before
                         AMCL republishes `amcl_pose`. The safety gate declares that estimate
                         unusable once the robot has moved more than 0.10 m
                         (`localiser_move_margin_m`) since it arrived, so the two numbers
                         cannot both stand: at 0.25 the gate must refuse partway through
                         every update interval, and only a no-motion refresh clears it.
                         Measured 2026-09-18 (batch_20260918T061032Z): 227-262 refusals per
                         N05 run, 17-18 Nav2 `Failed to make progress` aborts, and a 68 s
                         approach that took 194 s. Lowered to 0.10, which is the largest
                         step that cannot cross the hatch.
  bt server timeout      stock `default_server_timeout: 20` (milliseconds) is shorter
                         than this machine's discovery plus first-call latency, so
                         every behaviour tree action and service call timed out while
                         the planner was computing paths correctly. The visible symptom
                         names the planner, not the timeout.
  keepout / speed zones  disabled: the stock file enables them behind launch
                         substitutions and this deployment has no zone masks yet.
                         Leaving them enabled with an empty mask topic is a
                         lifecycle activation failure.

Usage:
    scripts/gen_nav2_params.py                 # write config/nav2_params.yaml
    scripts/gen_nav2_params.py --check         # verify the committed file is current
    scripts/gen_nav2_params.py --diff          # show a unified diff and write nothing
"""

from __future__ import annotations

import argparse
import difflib
import os
import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "src" / "fleet_bringup" / "config" / "nav2_params.yaml"

STOCK_CANDIDATES = [
    Path(os.environ.get("NAV2_BRINGUP_PARAMS", "")),
    Path("/opt/nav2/nav2_bringup/share/nav2_bringup/params/nav2_params.yaml"),
    Path("/opt/ros/lyrical/share/nav2_bringup/params/nav2_params.yaml"),
]

FOOTPRINT = "[[0.30, 0.225], [0.30, -0.225], [-0.30, -0.225], [-0.30, 0.225]]"

HEADER = """# 009 fleet Nav2 parameters.
#
# GENERATED by scripts/gen_nav2_params.py from the stock Nav2 parameter file that
# ships with this machine's Nav2 build. Do not hand edit: edit the generator so the
# reason for every deviation from stock stays in the repository next to the value.
#
# Source: {source}
#
# The placeholder __NS__ appears in every TF frame name and is substituted with the
# robot namespace by fleet_bringup/launch/robot.launch.py. There must be no bare
# frame name left in this file: two robots share one TF tree, so a bare `odom` or
# `base_link` would put both robots in the same frame.

"""

# (description, pattern, replacement, expected_count, limit)
#   expected_count  number of replacements that MUST happen; None = at least one
#   limit           None = replace all matches, else replace at most this many
#                   (used where the same text appears in blocks we do not launch)
EDITS: list[tuple[str, str, str, int | None, int | None]] = [
    (
        "per robot footprint replaces the stock circular radius in both costmaps",
        r"^(\s*)robot_radius: 0\.22$",
        # SINGLE quotes, deliberately, and no escape backslashes.
        #
        # The first attempt used \" ... \" inside the regex replacement. Python's
        # re.sub leaves an unknown escape such as \" alone, so the generated YAML became
        #
        #     footprint: \"[[0.30, 0.225], ...]\"
        #
        # which YAML reads as a plain scalar CONTAINING backslashes. nav2 then failed
        # with "Error parsing footprint parameter: Numbers at depth other than 2. Char
        # was '\'" and silently fell back to `using radius (0.100000) instead` -- a 10 cm
        # radius for a 60 x 45 cm robot, i.e. the planner would believe it fits through
        # gaps it cannot fit through. A silent safety regression caused by one backslash.
        #
        # Single-quoted YAML scalars do no escape processing, so there is nothing to get
        # wrong. `make_nav2_params_checks` below fails the build if a backslash appears
        # anywhere in the generated file.
        r"\1footprint: '" + FOOTPRINT + r"'",
        2,
        None,
    ),
    (
        # Was 0.35, which nav2 rejected on the first Nav2 bringup:
        #   "The inflation radius (0.350000) is smaller than the circumscribed radius
        #    (0.389005) ... This may significantly slow down planning times!"
        # 0.40 clears the circumscribed radius (sqrt(0.30^2 + 0.225^2) = 0.375 plus the
        # polygon's own rounding) while still being 43% smaller than the stock 0.70.
        #
        # The tension, recorded because it will come up again: the protected corridor is
        # only a 1.2 m gap, and a large inflation makes it expensive enough that the
        # planner may prefer a route that does not exist. Stock 0.70 inflates 0.70 m
        # from each wall in a 1.2 m opening. 0.40 leaves the centreline at zero cost and
        # only raises cost in the outer 0.175 m of each side, so the gap stays usable.
        # Whether a 0.40 inflation actually lets a plan through is P4's business, and
        # until then this number is a reasoned starting point, not a validated one.
        "inflation sized above the circumscribed radius but below stock, for the 1.2 m gap",
        r"^(\s*)inflation_radius: 0\.70?$",
        r"\1inflation_radius: 0.40",
        2,
        None,
    ),
    (
        "smoother linear velocity matched to the plant cap (0.35 m/s)",
        r"^(\s*)max_velocity: \[0\.5, 0\.0, 2\.0\]$",
        r"\1max_velocity: [0.35, 0.0, 0.6]",
        1,
        None,
    ),
    (
        "smoother reverse velocity matched to the plant cap",
        r"^(\s*)min_velocity: \[-0\.5, 0\.0, -2\.0\]$",
        r"\1min_velocity: [-0.35, 0.0, -0.6]",
        1,
        None,
    ),
    (
        "smoother acceleration matched to the plant cap (0.5 m/s2, 1.0 rad/s2)",
        r"^(\s*)max_accel: \[2\.5, 0\.0, 3\.2\]$",
        r"\1max_accel: [0.5, 0.0, 1.0]",
        1,
        None,
    ),
    (
        "smoother deceleration matched to the plant cap",
        r"^(\s*)max_decel: \[-2\.5, 0\.0, -3\.2\]$",
        r"\1max_decel: [-0.5, 0.0, -1.0]",
        1,
        None,
    ),
    (
        "collision_monitor hands cmd_vel to the safety gate instead of owning it",
        r'^(\s*)cmd_vel_out_topic: "cmd_vel"$',
        r'\1cmd_vel_out_topic: "cmd_vel_pre_gate"',
        1,
        None,
    ),
    (
        "lidar max range matched to the 12 m sensor",
        r"^(\s*)laser_max_range: 100\.0$",
        r"\1laser_max_range: 12.0",
        1,
        None,
    ),
    (
        "keepout zones disabled (no mask published yet): local and global costmap",
        r"^(\s*enabled: )KEEPOUT_ZONE_ENABLED$",
        r"\1False",
        2,
        None,
    ),
    (
        "speed zones disabled (no mask published yet)",
        r"^(\s*enabled: )SPEED_ZONE_ENABLED$",
        r"\1False",
        1,
        None,
    ),
    (
        "every base_frame_id (amcl, collision_monitor, docking_server) prefixed",
        r'^(\s*)base_frame_id: "base_footprint"$',
        r'\1base_frame_id: "__NS__/base_footprint"',
        3,
        None,
    ),
    (
        "every odom_frame_id (amcl, collision_monitor, docking_server) prefixed",
        r'^(\s*)odom_frame_id: "odom"$',
        r'\1odom_frame_id: "__NS__/odom"',
        3,
        None,
    ),
    (
        "every robot_base_frame (bt_navigator, both costmaps, behaviour) prefixed",
        r"^(\s*)robot_base_frame: base_link$",
        r"\1robot_base_frame: __NS__/base_link",
        4,
        None,
    ),
    (
        "local costmap rolling frame prefixed per robot",
        r"^(\s*)global_frame: odom$",
        r"\1global_frame: __NS__/odom",
        1,
        None,
    ),
    (
        "behaviour server local frame prefixed per robot",
        r"^(\s*)local_frame: odom$",
        r"\1local_frame: __NS__/odom",
        1,
        None,
    ),
    (
        "behaviour backup accel matched to the plant",
        r"^(\s*)acceleration_limit: 2\.5$",
        r"\1acceleration_limit: 0.5",
        2,
        None,
    ),
    (
        "behaviour backup decel matched to the plant",
        r"^(\s*)deceleration_limit: -2\.5$",
        r"\1deceleration_limit: -0.5",
        2,
        None,
    ),
    (
        "behaviour rotational speed within the plant cap",
        r"^(\s*)max_rotational_vel: 1\.0$",
        r"\1max_rotational_vel: 0.6",
        1,
        None,
    ),
    (
        "behaviour rotational accel within the plant cap",
        r"^(\s*)rotational_acc_lim: 3\.2$",
        r"\1rotational_acc_lim: 1.0",
        1,
        None,
    ),
    (
        # Stock ships 20 ms, tuned for a fast native desktop, and it is the timeout that
        # EVERY behaviour-tree action and service call inherits. On this machine it is
        # too short, and the failure it produces is thoroughly misleading: the planner
        # DOES receive the goal and DOES start computing a path, while the behaviour tree
        # has already given up and reports only
        #
        #     Timed out while waiting for action server to acknowledge goal request for
        #     compute_path_to_pose
        #     Node timed out while executing service call to
        #     global_costmap/clear_entirely_global_costmap
        #     NavigateToPoseNavigator::goalCompleted error 207:
        #     Behavior Tree action client timed out waiting.
        #
        # which reads like a broken planner or a broken map. Every one of those calls
        # timed out in the same burst, which is what points at a shared timeout rather
        # than at any individual server.
        #
        # 2000 ms is generous but bounded. If it is too generous the cost is a slower
        # failure, not a wrong answer, which is the right way round for a safety system.
        "behaviour tree server timeout raised from the stock 20 ms",
        r"^(\s*)default_server_timeout: 20$",
        r"\1default_server_timeout: 2000",
        1,
        None,
    ),
    (
        # LIMIT 1: the same commented line also appears under the keepout and speed
        # mask servers, which this deployment does not launch. Only the FIRST one
        # (map_server) may be filled in; giving the mask servers a real map path would
        # silently turn them into live map publishers.
        "map_server yaml_filename is a placeholder filled in by the launch file "
        "(first occurrence only: the mask servers share the same commented line)",
        r'^(\s*)# yaml_filename: ""$',
        r'\1yaml_filename: "__MAP__"',
        1,
        1,
    ),
    (
        # Stock ships 0.25 m: the distance the robot must travel before AMCL republishes
        # `amcl_pose`. The safety gate's escape hatch admits an estimate while the robot has
        # moved at most `localiser_move_margin_m` (0.10 m) since that estimate arrived, so at
        # the stock value the gate is GUARANTEED to start refusing partway through every
        # update interval -- measured 227-262 refusals in each N05 run of
        # batch_20260918T061032Z. What that costs is not the refusal but Nav2's reaction to
        # it: `SimpleProgressChecker` allows 10 s without 0.5 m of movement, so a stall
        # becomes `Failed to make progress`, the behaviour server clears both costmaps and
        # replans, and a 68 s approach takes 194 s.
        #
        # 0.10 m is not a new tuning preference: it is the largest step that cannot cross the
        # hatch, and `scripts/check_pose_freshness.py` fails the build if the two ever cross
        # again, so this cannot be undone by accident.
        "amcl update_min_d kept at or below the gate's escape hatch",
        r"^(\s*)update_min_d: 0\.25$",
        r"\1update_min_d: 0.10",
        1,
        None,
    ),
]

# Inserted into the amcl block so localisation starts from the spawn pose instead of
# waiting for a human to click a pose estimate.
AMCL_INITIAL_POSE_ANCHOR = "    scan_topic: scan\n"
AMCL_INITIAL_POSE_ADDITION = """    scan_topic: scan
    # Initial pose, supplied by robot.launch.py from the spawn pose in
    # config/spawns.yaml. Without this AMCL stays uninitialised until someone publishes
    # a pose estimate, and the entire stack looks broken while it is only waiting -- a
    # failure mode that costs an afternoon every time.
    #
    # These are PLACEHOLDERS and not zeros on purpose. Hardcoding 0 0 0 puts the initial
    # belief at the map origin, so a robot spawned at (-6, -2) starts localised 6.3 m
    # from where it actually is. AMCL then has to recover from a wrong prior, which is
    # slower and sometimes converges on the wrong hypothesis -- and it presents as
    # "localisation is broken" rather than "the initial pose was never set".
    set_initial_pose: true
    initial_pose:
      x: __INIT_X__
      y: __INIT_Y__
      z: 0.0
      yaw: __INIT_YAW__
"""


def find_stock() -> Path:
    for cand in STOCK_CANDIDATES:
        if str(cand) and cand.is_file():
            return cand
    raise SystemExit(
        "FATAL: stock nav2_params.yaml not found. Source the Nav2 overlay first, or "
        "set NAV2_BRINGUP_PARAMS to its path. Looked in:\n  "
        + "\n  ".join(str(c) for c in STOCK_CANDIDATES if str(c))
    )


def generate(stock: Path) -> str:
    text = stock.read_text(encoding="utf-8")
    problems: list[str] = []

    for label, pattern, replacement, expected, limit in EDITS:
        text, n = re.subn(pattern, replacement, text,
                          count=0 if limit is None else limit, flags=re.MULTILINE)
        if n == 0:
            problems.append(f"NOT APPLIED  {label}\n              pattern: {pattern}")
        elif expected is not None and n != expected:
            problems.append(
                f"COUNT        {label}\n"
                f"              expected {expected} replacement(s), made {n}. "
                "The stock file changed; re-read it before trusting this generator."
            )

    if AMCL_INITIAL_POSE_ANCHOR not in text:
        problems.append(
            "NOT APPLIED  amcl initial pose\n"
            f"              anchor not found: {AMCL_INITIAL_POSE_ANCHOR!r}"
        )
    else:
        text = text.replace(AMCL_INITIAL_POSE_ANCHOR, AMCL_INITIAL_POSE_ADDITION, 1)

    if problems:
        raise SystemExit(
            "FATAL: the generator did not apply cleanly against\n  "
            + str(stock)
            + "\n\n"
            + "\n".join(problems)
            + "\n\nRefusing to write an incomplete config: a silently skipped fix is "
            "worse than no config at all.\n"
        )

    # A bare frame name left behind is exactly the two robots in one frame bug, and
    # it is invisible until both robots move. Prove there is none.
    bare = []
    for frame in ("odom", "base_link", "base_footprint"):
        for m in re.finditer(r"^(\s*)([a-z_]+):\s*\"?" + frame + r"\"?\s*$", text,
                             flags=re.MULTILINE):
            key = m.group(2)
            if key in ("global_frame",) and frame == "odom":
                bare.append(f"line {text[:m.start()].count(chr(10)) + 1}: {key}: {frame}")
            elif key in ("robot_base_frame", "base_frame_id", "odom_frame_id",
                         "local_frame", "global_frame"):
                bare.append(f"line {text[:m.start()].count(chr(10)) + 1}: {key}: {frame}")
    if bare:
        raise SystemExit(
            "FATAL: unprefixed TF frame names survived generation. With two robots in "
            "one TF tree these collide:\n  " + "\n  ".join(bare) + "\n"
        )

    # A backslash in the output means a regex replacement mis-escaped. re.sub leaves an
    # unknown escape such as \" in place, which is how `footprint: \"[[...]]\"` got into
    # the file and cost nav2 its footprint (falling back to a 0.1 m radius). Asserting
    # is cheaper than noticing the misalignment three steps later.
    if "\\" in text:
        offenders = [
            f"line {text[:m.start()].count(chr(10)) + 1}: {m.group(0).strip()}"
            for m in re.finditer(r".*\\.*", text)
        ][:5]
        raise SystemExit(
            "FATAL: backslash(es) in the generated params. A regex replacement "
            "mis-escaped and YAML will read the escape literally:\n  "
            + "\n  ".join(offenders)
            + "\n"
        )

    # The footprint must be present twice (local + global costmap) and quoted the way
    # nav2 expects, because the silent fallback is a dangerous default rather than an
    # error.
    footprints = re.findall(r"^\s*footprint:\s*(.+)$", text, flags=re.MULTILINE)
    if len(footprints) != 2:
        raise SystemExit(
            f"FATAL: expected 2 footprint entries (local + global costmap), "
            f"found {len(footprints)}. nav2 would fall back to a default radius."
        )
    for value in footprints:
        if not (value.startswith("'") and value.endswith("'")):
            raise SystemExit(
                f"FATAL: footprint is not single quoted as nav2 expects: {value!r}"
            )

    return HEADER.format(source=stock) + text


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--check", action="store_true",
                    help="exit 1 if the committed file differs from a fresh render")
    ap.add_argument("--diff", action="store_true", help="print a unified diff")
    args = ap.parse_args(argv)

    stock = find_stock()
    fresh = generate(stock)

    if args.check or args.diff:
        if not OUT.is_file():
            print(f"FATAL: {OUT} does not exist; run without --check", file=sys.stderr)
            return 1
        current = OUT.read_text(encoding="utf-8")
        if current == fresh:
            print(f"[ OK ] {OUT.relative_to(ROOT)} is current (source {stock})")
            return 0
        if args.diff:
            sys.stdout.writelines(difflib.unified_diff(
                current.splitlines(keepends=True),
                fresh.splitlines(keepends=True),
                fromfile=str(OUT.relative_to(ROOT)) + " (committed)",
                tofile=str(OUT.relative_to(ROOT)) + " (fresh)",
            ))
        print("STALE: committed nav2 params differ from a fresh render", file=sys.stderr)
        return 1

    OUT.parent.mkdir(parents=True, exist_ok=True)
    OUT.write_text(fresh, encoding="utf-8")
    print(f"[ OK ] wrote {OUT.relative_to(ROOT)}  ({len(fresh.splitlines())} lines)")
    print(f"       source: {stock}")
    print(f"       edits applied: {len(EDITS) + 1} (incl. amcl initial pose)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
