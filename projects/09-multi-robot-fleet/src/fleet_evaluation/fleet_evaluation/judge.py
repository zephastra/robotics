"""Independent evaluation: judge a recorded run from ground truth, not from claims.

WHY THIS IS A SEPARATE PACKAGE
------------------------------
Every verdict this project has published so far was produced by the same process that
drove, or at least watched, the thing being judged. `acceptance_p4.py` subscribes to each
robot's odometry, calls the task service's own `/fleet/status`, and reads each gate's own
`gate_state` JSON, and then decides whether the run passed. The ledger and the gate are the
*subject* of the test, so their self-reports cannot also be its evidence.

There is a second, subtler version of the same problem. `acceptance_p4.py` decides "was this
robot inside the protected region" by running the geometry on **odometry**. Odometry is the
estimate the controller acts on. An estimate-based test cannot detect a localisation error,
because it asks the robot where it is and then believes the answer.

This package does three things differently:

1. **Truth is the arbiter.** The world's own `dynamic_pose` stream decides where each robot
   actually was. Every safety property is re-derived from truth and reported separately from
   the claim-based version, so a disagreement is visible instead of averaged away.
2. **Claims are recorded as claims.** The ledger, the gate and the driver reports are read
   and *cross-checked*, never used as evidence. Each ends up CORROBORATED, CONTRADICTED or
   UNCHECKED, and UNCHECKED is not a pass.
3. **It is pure Python.** No ROS, no simulator, same rule as `fleet_core`. A judge that needs
   the thing it is judging in order to run is not independent.

WHAT IT DELIBERATELY DOES NOT DO
--------------------------------
It does not re-run anything and it does not fix anything. It reads a recorded run directory.
Recording is `fleet_evaluation.recorder`, which subscribes and never publishes.

TRUTH HEADING, AND WHAT ITS ABSENCE COSTS
-----------------------------------------
The world stream carries a rotation. The P4 sampler recorded only the translation, so for
those recordings containment cannot use the real oriented footprint and falls back to the
bounding square of the circumscribed circle (side = 2 x footprint radius). That direction of
error is the safe one for the two safety properties -- it can call a robot "inside" when it
is marginally outside, never the reverse, so a PASS is a *stronger* claim than usual.

The cost is stated rather than hidden: with a conservative truth test and an exact
odometry test, the two containment answers differ **by construction**, so
`estimate_and_truth_agree_on_containment` is `NOT_RUN` for such a recording. Comparing a
conservative test against an exact one and calling the difference a localisation error would
be a fabricated finding. Newer recordings carry yaw and get the exact test.
"""

from __future__ import annotations

import argparse
import itertools
import json
import math
import pathlib
import statistics
import sys
from typing import Any

from fleet_core import validate_traffic_config
from fleet_core.geometry import Region, footprint_corners, footprint_radius

PASS = "PASS"
FAIL = "FAIL"
NOT_RUN = "NOT_RUN"
HOLD = "HOLD"

L, W = 0.60, 0.45


# --------------------------------------------------------------------------- #
# loading
# --------------------------------------------------------------------------- #

def load_samples(path: pathlib.Path) -> list[dict[str, Any]]:
    """Read a samples-*.jsonl. Unreadable lines are reported, never skipped silently."""
    out: list[dict[str, Any]] = []
    bad: list[str] = []
    for number, line in enumerate(path.read_text(encoding="utf-8").splitlines(), 1):
        line = line.strip()
        if not line:
            continue
        try:
            out.append(json.loads(line))
        except json.JSONDecodeError as exc:
            bad.append(f"line {number}: {exc}")
    if bad:
        raise ValueError(f"{path.name}: {len(bad)} unreadable line(s): {bad[:3]}")
    return out


def load_case(case_dir: pathlib.Path) -> dict[str, Any]:
    """Everything a case directory holds: samples, the drivers' own reports, the verdict."""
    samples_files = sorted(case_dir.glob("samples-*.jsonl"))
    verdict_files = sorted(case_dir.glob("verdict-*.json"))
    drivers = []
    for path in sorted(case_dir.glob("*.json")):
        if path.name.startswith("verdict-"):
            continue
        try:
            body = json.loads(path.read_text(encoding="utf-8"))
        except json.JSONDecodeError:
            continue
        if isinstance(body, dict) and "robot" in body and "direction" in body:
            body["_file"] = path.name
            drivers.append(body)
    published = None
    if verdict_files:
        published = json.loads(verdict_files[0].read_text(encoding="utf-8"))
    return {
        "dir": case_dir,
        "samples": load_samples(samples_files[0]) if samples_files else [],
        "samples_file": samples_files[0].name if samples_files else "",
        "drivers": drivers,
        "published_verdict": published,
        "published_verdict_file": verdict_files[0].name if verdict_files else "",
    }


def load_spawns(path: pathlib.Path) -> dict[str, tuple[float, float, float]]:
    import yaml

    body = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
    out: dict[str, tuple[float, float, float]] = {}
    for rid, pose in (body.get("spawns") or {}).items():
        out[str(rid)] = (float(pose["x"]), float(pose["y"]), float(pose.get("yaw", 0.0)))
    if not out:
        raise ValueError(f"{path}: no spawns")
    return out


def robots_in_recording(samples: list[dict[str, Any]]) -> set[str]:
    """Which robots this recording is actually about.

    Judged from the odometry the recording holds. That is not using a claim as evidence
    about behaviour; it is using it to know who took part. A configuration that has since
    gained a third robot must not make an older two-robot recording unjudgeable -- but the
    difference has to be stated, because "judged 2 of the 3 configured robots" is a fact the
    reader of a verdict needs.
    """
    present: set[str] = set()
    for sample in samples:
        present |= set((sample.get("odom") or {}).keys())
    return present


# --------------------------------------------------------------------------- #
# attributing truth slots to robots
# --------------------------------------------------------------------------- #

def _first_shared(rows: "list[dict[str, Any]]") -> dict:
    """The first sample's region -> robots, or an empty map. For the failure message only.

    A message that says "two robots inside" without naming the region is what made this check's
    scope unnoticed for a round: the reader could not tell a real overlap from two neighbours in
    two different buffers, and both are printed by the same sentence.
    """
    if not rows:
        return {}
    return rows[0].get("shared") or {}


def _truth_slots(sample: dict[str, Any]) -> dict[int, Any]:
    raw = sample.get("truth") or {}
    out: dict[int, tuple[float, ...]] = {}
    for key, value in raw.items():
        try:
            out[int(key)] = tuple(float(v) for v in value)
        except (TypeError, ValueError):
            continue
    return out


SEED_TOLERANCE_M = 1.0
DECISIVE_MARGIN_M = 0.5


def _slot_note(slot: int, track: dict[str, Any]) -> str:
    """One slot's evidence, in words, for a slot that turned out not to be a robot."""
    if track["first"] is None:
        return (f"slot {slot} never reported a position away from the origin "
                f"({track.get('zeros', 0)} sample(s) at (0.000, 0.000))")
    return (f"slot {slot} sits at ({track['first'][0]:.3f}, {track['first'][1]:.3f}) and "
            f"moved {track['span_m']:.3f} m in total")


def _slot_tracks(samples: list[dict[str, Any]]) -> dict[int, dict[str, Any]]:
    """Per slot: where it started, where it ended, and how far it ever moved.

    `first` is the first position the slot reported AWAY FROM THE ORIGIN, and that is a fix
    rather than a refinement. The world widens its pose array as models come up: measured on
    N07 seed 1 (2026-09-18, `batch_20260918T080259Z`) the truth widths are 0 (241 samples),
    **8 (7 samples)** and 24 (12147 samples) -- and during those 7 samples only slot 0 held a
    position while slots 1..7 read as (0, 0). Taking the first sample verbatim therefore fixed
    r02's and r03's starting position at the origin, and `attribute_truth`, whose rule is that
    a slot is a robot only if it STARTS ON THAT ROBOT'S SPAWN, could match neither. Every
    truth-based check then returned NOT_RUN and the case's `safety` was UNKNOWN.

    Two-robot recordings never showed it because their widths are only 0 and 16: there is no
    intermediate array to poison the start.

    Skipping the origin is sound, not convenient: no robot can start there, because
    scripts/validate_assets.py proves every spawn in config/spawns.yaml is clear of the walls,
    the pads and the protected corridor, and (0, 0) is inside the corridor's throat.

    A slot that never reports a non-origin position keeps `first = None`. That is a statement
    about the recording, and it is deliberately NOT the same as "this slot started at the
    origin", which is what the previous version asserted.
    """
    tracks: dict[int, dict[str, Any]] = {}
    for sample in samples:
        for slot, pos in _truth_slots(sample).items():
            track = tracks.setdefault(
                slot, {"first": None, "last": None, "n": 0, "xs": [], "ys": [], "zeros": 0})
            if pos[0] == 0.0 and pos[1] == 0.0:
                track["zeros"] += 1
                continue
            if track["first"] is None:
                track["first"] = pos[:2]
            track["last"] = pos[:2]
            track["n"] += 1
            track["xs"].append(pos[0])
            track["ys"].append(pos[1])
    for track in tracks.values():
        if not track["xs"]:
            track["span_m"] = 0.0
            continue
        track["span_m"] = round(
            max(max(track["xs"]) - min(track["xs"]),
                max(track["ys"]) - min(track["ys"])), 3)
    return tracks


def _placement_index(samples: list[dict[str, Any]]) -> dict[int, int]:
    """Per slot, the first sample index at which it reported a position at all.

    The same fact `_slot_tracks` uses for `first`, kept in the form the containment loops
    need. A slot with no entry here was never placed in this recording.
    """
    placed: dict[int, int] = {}
    for index, sample in enumerate(samples):
        for slot, pos in _truth_slots(sample).items():
            if slot in placed:
                continue
            if pos[0] == 0.0 and pos[1] == 0.0:
                continue
            placed[slot] = index
    return placed


def _is_placed(placed: dict[int, int], slot: int, index: int) -> bool:
    """Has this slot been placed by this sample?

    Before its first placement a slot reads exactly (0.000, 0.000), and the origin is
    inside `mid`, so counting it as a position invents occupancy. After placement the
    origin is a real coordinate and is counted normally.
    """
    return slot in placed and index >= placed[slot]


def attribute_truth(samples: list[dict[str, Any]],
                    spawns: dict[str, tuple[float, float, float]]) -> dict[str, Any]:
    """Slot to robot, decided by geometry, with what that decision rested on.

    The world stream carries no frame names (measured: 2816 of 2816 `child_frame_id` values
    empty) and **it also carries models that are not robots** -- a static barrier or prop
    comes through the same stream. So a slot is only accepted as a robot when it *starts on
    that robot's spawn*; anything starting elsewhere is reported as "not a robot" rather than
    quietly matched to whoever is nearest, which is how a static prop at (0.2, 0.0) ends up
    labelled as a robot and every number afterwards is about the wrong entity.

    Two things must hold before any judgement is made:

    * every robot taking part has a slot that starts on its spawn, and
    * the assignment is decisive: each slot's own spawn is much closer than any other's.

    The margin is the second condition. A thin margin means the evidence does not tell the
    robots apart, and a mislabelled slot silently swaps two trajectories.
    """
    robots = sorted(spawns)
    tracks = _slot_tracks(samples)
    if not tracks:
        return {"ok": False, "mapping": {}, "margin_m": None, "seed_distances_m": {},
                "reason": "the recording contains no truth positions at all"}

    slot_ids = sorted(tracks)
    # A slot that never reported a position away from the origin has no starting point to
    # compare against, so it cannot be matched to anything -- and it must not be handed the
    # origin as a consolation prize, which is exactly what made three-robot cases unjudgeable.
    located = [slot for slot in slot_ids if tracks[slot]["first"] is not None]

    # A slot may only be assigned to a robot whose spawn it started on. Slots that started
    # nowhere near a spawn are not robots and are excluded, not guessed at.
    pairs = [(slot, robot,
              math.hypot(tracks[slot]["first"][0] - spawns[robot][0],
                         tracks[slot]["first"][1] - spawns[robot][1]))
             for slot in located for robot in robots]
    eligible = [p for p in pairs if p[2] <= SEED_TOLERANCE_M]

    best = None
    if len(eligible) >= len(robots):
        for perm in itertools.permutations(eligible, len(robots)):
            if len({p[0] for p in perm}) != len(robots):
                continue
            if len({p[1] for p in perm}) != len(robots):
                continue
            total = sum(p[2] for p in perm)
            if best is None or total < best[0]:
                best = (total, list(perm))

    not_robots = sorted(slot for slot in slot_ids
                        if all(d > SEED_TOLERANCE_M
                               for _s, _r, d in pairs if _s == slot))
    # Every transform index is recorded now, so most slots are a robot's own link offsets.
    # Listing sixteen of them buries the sentence that matters; listing none hides it. So the
    # first few are named exactly as before and the rest are counted. The count is stated, not
    # silently dropped.
    listed = not_robots[:4]
    non_robot_note = "; ".join(_slot_note(slot, tracks[slot]) for slot in listed)
    if len(not_robots) > len(listed):
        extra = len(not_robots) - len(listed)
        # ONE literal on purpose. The first version split this across two adjacent literals, and
        # the assertion written to check it searched the source for the joined phrase and found
        # nothing -- so the check reported a missing message that was there. Code and prose are
        # different layers, and so are a string and its concatenation.
        non_robot_note += (f"; and {extra} further slot(s) that did not start on any spawn")

    if best is None:
        matched = sorted({robot for _s, robot, _d in eligible})
        missing = [r for r in robots if r not in matched]
        reason = (f"no slot starts on the spawn of {missing}: " if missing else
                  "no slot could be matched to a spawn: ")
        reason += (non_robot_note or "no slot started within "
                   f"{SEED_TOLERANCE_M} m of any spawn")
        reason += (". Without truth for a robot taking part, nothing about that robot can be "
                   "judged independently, so the truth-based checks are NOT_RUN rather than "
                   "answered from the estimate.")
        # `slot_tracks` is carried on this branch too. Without it a reader is handed the
        # sentence about the slots and not the slots, which is backwards: an unattributable
        # recording is precisely when someone needs to see the positions that caused it.
        return {"ok": False, "mapping": {}, "margin_m": None,
                "seed_distances_m": {}, "not_robot_slots": not_robots,
                "missing_robots": missing, "reason": reason,
                "slot_tracks": {str(s): {"start": None if t["first"] is None
                                         else [round(v, 3) for v in t["first"]],
                                         "end": None if t["last"] is None
                                         else [round(v, 3) for v in t["last"]],
                                         "span_m": t["span_m"], "n": t["n"],
                                         "zeros": t["zeros"]}
                                for s, t in tracks.items()}}

    _total, chosen = best
    assigned = {slot: robot for slot, robot, _d in chosen}
    distances = {robot: d for _s, robot, d in chosen}

    margins = []
    for slot, robot, d in chosen:
        px, py = tracks[slot]["first"]
        others = [math.hypot(px - spawns[o][0], py - spawns[o][1])
                  for o in robots if o != robot]
        if others:
            margins.append(min(others) - d)

    worst_seed = max(distances.values(), default=0.0)
    smallest_margin = min(margins) if margins else None
    ok = worst_seed <= SEED_TOLERANCE_M and (
        smallest_margin is None or smallest_margin > DECISIVE_MARGIN_M)
    reason = "" if ok else (
        f"attribution is not decisive: worst seed distance {worst_seed:.2f} m, "
        f"smallest margin {smallest_margin:.2f} m")

    # Continuity: a slot must stay closest to the robot it was assigned to. A reordered
    # stream would otherwise swap two trajectories with no other symptom.
    violations = 0
    previous = {robot: (spawns[robot][0], spawns[robot][1]) for robot in robots}
    for sample in samples:
        here = _truth_slots(sample)
        if not all(slot in here for slot in assigned):
            continue
        nearest: dict[str, int] = {}
        for robot, (rx, ry) in previous.items():
            nearest[robot] = min(
                assigned, key=lambda s: math.hypot(here[s][0] - rx, here[s][1] - ry))
        if len(set(nearest.values())) != len(robots):
            violations += 1
        elif any(assigned[slot] != robot for robot, slot in nearest.items()):
            violations += 1
        for slot, robot in assigned.items():
            previous[robot] = here[slot][:2]

    return {
        "ok": ok,
        "reason": reason,
        "mapping": {str(slot): robot for slot, robot in assigned.items()},
        "margin_m": round(smallest_margin, 3) if smallest_margin is not None else None,
        "seed_distances_m": {r: round(d, 3) for r, d in distances.items()},
        "continuity_violations": violations,
        "not_robot_slots": not_robots,
        # `start` is None for a slot that never reported a position away from the origin.
        # Emitting the origin there would be a number where the recording contains none.
        "slot_tracks": {str(s): {"start": None if t["first"] is None
                                 else [round(v, 3) for v in t["first"]],
                                 "end": None if t["last"] is None
                                 else [round(v, 3) for v in t["last"]],
                                 "span_m": t["span_m"], "n": t["n"], "zeros": t["zeros"]}
                        for s, t in tracks.items()},
    }


# --------------------------------------------------------------------------- #
# truth-based geometry
# --------------------------------------------------------------------------- #

def _regions(cfg) -> dict[str, Region]:
    return {
        name: Region(name=name, rects=spec.rects, margin_m=cfg.footprint_margin_m)
        for name, spec in cfg.resources.items()
    }


def truth_has_heading(samples: list[dict[str, Any]]) -> bool:
    """Does this recording's truth carry a yaw for every slot it reports?"""
    seen = False
    for sample in samples:
        for value in _truth_slots(sample).values():
            seen = True
            if len(value) < 3:
                return False
    return seen


def containment_from_truth(pose: tuple[float, ...] | None, regions: dict[str, Region],
                           *, conservative: bool) -> tuple[str, ...]:
    """Which protected regions this TRUTH position is in.

    `conservative=True` substitutes the bounding square of the circumscribed circle
    (side = 2 x footprint radius) for the oriented footprint, and is used when the recording
    has no truth heading. It can report "inside" for a robot marginally outside and never
    "outside" for a robot inside, which is the safe direction for the two properties it
    serves -- but it is not the same test as the exact one, which is why the two are never
    compared against each other.
    """
    if pose is None:
        return ()
    if conservative or len(pose) < 3:
        radius = footprint_radius(L, W)
        corners = footprint_corners(pose[0], pose[1], 0.0, 2.0 * radius, 2.0 * radius)
    else:
        corners = footprint_corners(pose[0], pose[1], pose[2], L, W)
    return tuple(name for name, region in regions.items() if region.box_overlap(corners))


def detect_odom_frame(samples: list[dict[str, Any]], judged: dict[str, Any]) -> dict[str, Any]:
    """Is the recorded `odom` already in the map frame, or still in its spawn frame?

    This has to be decided rather than assumed, because the two sample formats in this
    project disagree. `fleet_recorder` stores `/rXX/odom` as it arrives: origin at the spawn
    pose. The P4 sampler composed each robot's odometry into the map frame *before* storing
    it. Applying the composition to already-composed data shifts every robot by a whole
    spawn offset, which turns a perfect estimate into several metres of error -- and it
    looks like a localisation fault rather than a format mistake.

    The test is the data: at the first sample that carries an odometry for every judged
    robot, compare the distance of those values from the spawns against their distance from
    the origin. "Already in map" means near the spawns; "spawn frame" means near zero. An
    ambiguous answer is returned as `unknown` so the caller can refuse rather than guess.
    """
    for sample in samples:
        odom = sample.get("odom") or {}
        if not all(r in odom and odom[r] is not None for r in judged):
            continue
        to_spawn = max(math.hypot(odom[r][0] - judged[r][0], odom[r][1] - judged[r][1])
                       for r in judged)
        to_origin = max(math.hypot(odom[r][0], odom[r][1]) for r in judged)
        if to_spawn <= 1.0 and to_spawn < to_origin * 0.5:
            return {"frame": "map", "to_spawn_m": round(to_spawn, 3),
                    "to_origin_m": round(to_origin, 3)}
        if to_origin <= 1.0 and to_origin < to_spawn * 0.5:
            return {"frame": "spawn", "to_spawn_m": round(to_spawn, 3),
                    "to_origin_m": round(to_origin, 3)}
        return {"frame": "unknown", "to_spawn_m": round(to_spawn, 3),
                "to_origin_m": round(to_origin, 3)}
    return {"frame": "unknown", "to_spawn_m": None, "to_origin_m": None}


def to_map(claim: tuple[float, float, float],
           spawn: tuple[float, float, float], frame: str) -> tuple[float, float]:
    """Put a recorded odometry reading into map coordinates, once and only once."""
    if frame == "map":
        return (claim[0], claim[1])
    sx, sy, syaw = spawn
    return (sx + claim[0] * math.cos(syaw) - claim[1] * math.sin(syaw),
            sy + claim[0] * math.sin(syaw) + claim[1] * math.cos(syaw))


def containment_from_odom(pose: tuple[float, float, float] | None,
                          spawn: tuple[float, float, float] | None,
                          regions: dict[str, Region], *, frame: str) -> tuple[str, ...]:
    """The same question, asked of the estimate the controller acts on.

    Both the spawn and the frame are REQUIRED, not optional. Odometry that has not been
    composed starts at zero at the spawn pose, so running the geometry on it raw answers a
    question about the map origin instead -- and the map origin is inside the corridor, which
    makes every stationary robot read as an intruder. An earlier version of this function
    omitted the argument and did exactly that; the test that caught it was
    `test_truth_catches_occupancy_the_estimate_hides`, whose whole purpose is to make the two
    containment answers differ. This is the fourth time this project has met the same frame
    mistake, so both parameters are mandatory here and the arithmetic lives in one place.
    """
    if pose is None or spawn is None:
        return ()
    x, y = to_map(pose, spawn, frame)
    corners = footprint_corners(x, y, pose[2], L, W)
    return tuple(name for name, region in regions.items() if region.box_overlap(corners))


def pose_error(samples: list[dict[str, Any]], mapping: dict[str, str],
               spawns: dict[str, tuple[float, float, float]], *,
               frame: str) -> dict[str, Any]:
    """|claim - truth| per robot, from the odometry the fleet acted on."""
    per_robot: dict[str, list[float]] = {}
    for sample in samples:
        odom = sample.get("odom") or {}
        slots = _truth_slots(sample)
        for slot_key, robot in mapping.items():
            slot = int(slot_key)
            claim = odom.get(robot)
            if claim is None or slot not in slots:
                continue
            cx, cy = to_map(claim, spawns[robot], frame)
            per_robot.setdefault(robot, []).append(
                math.hypot(cx - slots[slot][0], cy - slots[slot][1]))
    out: dict[str, Any] = {}
    for robot, errors in per_robot.items():
        ordered = sorted(errors)
        out[robot] = {
            "n": len(ordered),
            "p50_m": round(statistics.median(ordered), 4),
            "p95_m": round(ordered[int(0.95 * (len(ordered) - 1))], 4),
            "max_m": round(ordered[-1], 4),
        }
    return out


# --------------------------------------------------------------------------- #
# the verdict
# --------------------------------------------------------------------------- #

def _check(name: str, verdict: str, detail: str, kind: str = "verdict",
           **numbers: Any) -> dict[str, Any]:
    """One check result.

    `kind="caveat"` marks a check that qualifies the numbers without being a
    judgement about the run. A caveat must not turn a PASS into a NOT_RUN, or the
    summary stops carrying information -- which is what happened when the recorder's
    sampling rate was a HOLD: every case came out NOT_RUN, including clean ones.
    """
    return {"check": name, "verdict": verdict, "detail": detail, "kind": kind,
            **numbers}


def judge_case(case_dir: pathlib.Path, cfg, spawns: dict[str, tuple[float, float, float]],
               *, pose_p95_limit_m: float = 0.35,
               odom_frame: str = "auto") -> dict[str, Any]:
    case = load_case(case_dir)
    samples = case["samples"]
    regions = _regions(cfg)
    checks: list[dict[str, Any]] = []

    if not samples:
        return {
            "case": str(case_dir),
            "samples_file": case["samples_file"],
            "samples": 0,
            "summary": NOT_RUN,
            "failed": [], "not_run": ["samples_present"], "held": [], "caveats": [],
            "checks": [_check("samples_present", NOT_RUN,
                              "no samples-*.jsonl; nothing can be judged")],
        }

    # Judge the robots this recording is about, and say when that is fewer than configured.
    present = robots_in_recording(samples)
    judged = {r: p for r, p in spawns.items() if r in present} if present else dict(spawns)
    absent = sorted(set(spawns) - set(judged))
    scope_note = (f"judging {len(judged)} of {len(spawns)} configured robots"
                  + (f"; absent from this recording: {absent}" if absent else ""))

    # The two sample formats in this project disagree about the odom frame (see
    # detect_odom_frame), so it is established from the data and reported. Applying the
    # spawn composition to already-composed odometry shifts every robot by a whole spawn
    # offset and looks like a localisation fault.
    detection = detect_odom_frame(samples, judged)
    frame = detection["frame"] if odom_frame == "auto" else odom_frame
    checks.append(_check(
        "odom_frame_established",
        PASS if frame in ("map", "spawn") else NOT_RUN,
        (f"the recorded odometry is in the {frame!r} frame "
         f"({detection['to_spawn_m']} m from the spawns, {detection['to_origin_m']} m from "
         f"the origin at the first complete sample)"
         + ("" if frame != "unknown" else
            ". Neither close to the spawns nor close to the origin, so the composition "
            "cannot be decided from the data; pass --odom-frame to say which it is.")),
        frame=frame, detected=detection,
    ))

    exact = truth_has_heading(samples)
    conservative = not exact
    mode_note = ("exact: truth carries a heading, so the oriented footprint is used"
                 if exact else
                 "conservative: truth has no heading in this recording, so containment "
                 "uses the circumscribed circle's bounding square. It can report 'inside' "
                 "for a robot marginally outside, never the reverse, so a PASS here is "
                 "stronger than usual")

    attribution = attribute_truth(samples, judged)
    # If the attribution is not decisive, `mapping` is discarded rather than used. The
    # function still returns its best guess so the reason can quote a margin, but judging
    # with a guessed mapping would produce numbers about the wrong robot, confidently.
    mapping = attribution["mapping"] if attribution["ok"] else {}
    # A slot that has not been placed yet reads exactly (0.000, 0.000), and the origin is
    # inside `mid`, so counting it as a position invents occupancy. See _placement_index.
    placed_from = _placement_index(samples)
    checks.append(_check(
        "truth_attribution",
        PASS if attribution["ok"] else NOT_RUN,
        attribution["reason"] or (
            f"slots {attribution['mapping']} matched to spawns with margin "
            f"{attribution['margin_m']} m; continuity violations "
            f"{attribution.get('continuity_violations')}; {scope_note}"),
        reason=attribution["reason"],
        margin_m=attribution["margin_m"],
        seed_distances_m=attribution["seed_distances_m"],
        continuity_violations=attribution.get("continuity_violations"),
        not_robot_slots=attribution.get("not_robot_slots"),
        slot_tracks=attribution.get("slot_tracks"),
        robots_judged=sorted(judged),
        robots_absent=absent,
    ))

    # --- the two safety properties, derived from TRUTH ---------------------- #
    same_region_truth: list[dict[str, Any]] = []
    cross_region_truth: list[dict[str, Any]] = []
    truth_inside_count = 0
    disagreements = 0
    for index, sample in enumerate(samples):
        slots = _truth_slots(sample)
        odom = sample.get("odom") or {}
        inside: dict[str, list[str]] = {}
        for slot_key, robot in mapping.items():
            slot = int(slot_key)
            if not _is_placed(placed_from, slot, index):
                continue
            t_hits = containment_from_truth(slots.get(slot), regions,
                                            conservative=conservative)
            o_hits = containment_from_odom(odom.get(robot), judged.get(robot), regions,
                                          frame=frame)
            if t_hits:
                inside[robot] = list(t_hits)
                truth_inside_count += 1
            if exact and bool(t_hits) != bool(o_hits):
                disagreements += 1
        if len(inside) > 1:
            # "inside A protected region" means ONE region. Two robots each inside a DIFFERENT
            # region is a different situation, and a legal one: `mid_left` and `mid_right` are
            # separate resources with capacity 1 each, so putting one robot on each has obeyed
            # the reservation. Counting them together made this property fail on correct
            # operation -- measured 2026-09-18, N05 seed 1, both exit buffers occupied after a
            # crossing, reported as a safety violation.
            per_region: dict[str, list[str]] = {}
            for robot_name, hits in inside.items():
                for region in hits:
                    per_region.setdefault(region, []).append(robot_name)
            shared = {region: sorted(robots) for region, robots in per_region.items()
                      if len(robots) > 1}
            if shared:
                same_region_truth.append({"t": sample.get("t"), "shared": shared})
            else:
                cross_region_truth.append({"t": sample.get("t"), "inside": inside})
    if not mapping:
        same_region_truth = []
        cross_region_truth = []

    checks.append(_check(
        "no_simultaneous_occupancy_from_truth",
        # NOT_RUN when there is nothing to test, not PASS: same condition as the detail,
        # so the two cannot say different things.
        NOT_RUN if (not mapping or (not same_region_truth and truth_inside_count == 0))
        else (PASS if not same_region_truth else FAIL),
        (f"no robot's footprint overlapped a protected region anywhere in this recording "
         f"({truth_inside_count} single-robot inside-samples), so this property had nothing "
         f"to test. NOT_RUN rather than PASS: a property that was never exercised cannot be "
         f"evidence that it holds. {mode_note}"
         if not same_region_truth and mapping and truth_inside_count == 0 else
         f"no sample had two robots inside the same protected region, judged from the world's "
         f"own pose stream ({truth_inside_count} single-robot inside-samples); {mode_note}"
         if not same_region_truth and mapping else
         f"{len(same_region_truth)} sample(s) with two robots inside the SAME region, e.g. "
         f"{_first_shared(same_region_truth)}"
         if same_region_truth else attribution["reason"]),
        samples=len(same_region_truth), first=same_region_truth[:2],
        #: Reported and NOT judged: one robot on each exit buffer is legal, so this number must
        #: not change the verdict. Kept because "both buffers occupied" is worth being able to
        #: see without re-running anything.
        cross_region_samples=len(cross_region_truth), cross_region_first=cross_region_truth[:2],
        geometry=mode_note.split(":")[0],
    ))

    unauthorised_truth: list[dict[str, Any]] = []
    #: Robot-samples inside some protected region at all, whoever owns it. A recording with
    #: none of these cannot evidence "no unauthorised entry", for the same reason this judge
    #: reports `driver_claims_corroborated_by_truth` as NOT_RUN when there are no claims:
    #: "nothing was observed" is not the same claim as "nothing was wrong". Needed because
    #: skipping unplaced slots can otherwise turn the old false FAIL into a vacuous PASS.
    truth_in_region_samples = 0
    book_samples = sum(1 for s in samples if s.get("resources"))
    for index, sample in enumerate(samples):
        resources = sample.get("resources")
        if not resources:
            continue
        slots = _truth_slots(sample)
        for slot_key, robot in mapping.items():
            slot = int(slot_key)
            if not _is_placed(placed_from, slot, index):
                continue
            for name in containment_from_truth(slots.get(slot), regions,
                                               conservative=conservative):
                truth_in_region_samples += 1
                entry = resources.get(name) or {}
                if entry.get("owner") is None:
                    unauthorised_truth.append({"t": sample.get("t"), "robot": robot,
                                               "region": name,
                                               "book": str(entry.get("state"))})
    # The book has three states. `traffic.py` sets the INITIAL state to UNKNOWN rather
    # than FREE, and `domain.py` defines UNKNOWN as "no trustworthy occupancy knowledge ->
    # block". The PASS text below promises "a region its own book called unowned", so the
    # two are reported apart even though both remain a failure (CONTRACTS section 7 makes
    # UNKNOWN block entry). Merging them made a count nobody could read: F02's 1949 were
    # 1450 UNKNOWN followed by 499 FREE, one continuous episode, not one uniform fault.
    n_book_free = sum(1 for row in unauthorised_truth if row.get("book") == "FREE")
    n_book_unknown = sum(1 for row in unauthorised_truth if row.get("book") == "UNKNOWN")
    n_book_other = len(unauthorised_truth) - n_book_free - n_book_unknown

    if book_samples == 0:
        checks.append(_check(
            "no_unauthorised_entry_from_truth", NOT_RUN,
            "no sample carried the reservation book, so 'authorised' cannot be judged"))
    elif not mapping:
        checks.append(_check(
            "no_unauthorised_entry_from_truth", NOT_RUN, attribution["reason"]))
    elif not unauthorised_truth and truth_in_region_samples == 0:
        checks.append(_check(
            "no_unauthorised_entry_from_truth", NOT_RUN,
            f"no robot's footprint overlapped a protected region in this recording, so an "
            f"unauthorised entry was impossible to observe ({book_samples} book-carrying "
            f"samples examined). NOT_RUN rather than PASS: 'nothing was observed' is not the "
            f"same claim as 'nothing was wrong'. {mode_note}",
            samples=0, samples_book_free=0, samples_book_unknown=0, samples_book_other=0,
            geometry=mode_note.split(":")[0]))
    else:
        checks.append(_check(
            "no_unauthorised_entry_from_truth",
            PASS if not unauthorised_truth else FAIL,
            (f"no robot was inside a region its own book called unowned, judged from truth "
             f"across {book_samples} book-carrying samples; {mode_note}"
             if not unauthorised_truth else
             f"{n_book_free} sample(s) with a region the book called FREE occupied"
             + (f", plus {n_book_unknown} with the book saying UNKNOWN (CONTRACTS section "
                f"7: the initial state, and it blocks entry, so neither is authorised)"
                if n_book_unknown else "")
             + (f", plus {n_book_other} with the book in another state" if n_book_other
                else "")),
            samples=len(unauthorised_truth), first=unauthorised_truth[:2],
            samples_book_free=n_book_free, samples_book_unknown=n_book_unknown,
            samples_book_other=n_book_other,
            geometry=mode_note.split(":")[0],
        ))

    # --- the estimate, against truth ---------------------------------------- #
    errors = pose_error(samples, mapping, judged, frame=frame) if mapping else {}
    worst = max((e["p95_m"] for e in errors.values()), default=None)
    if worst is None:
        checks.append(_check("pose_estimate_error", NOT_RUN,
                             "no truth to compare the estimate against"))
    else:
        checks.append(_check(
            "pose_estimate_error",
            PASS if worst <= pose_p95_limit_m else FAIL,
            (f"worst p95 |claim-truth| = {worst:.4f} m over "
             f"{ {r: e['n'] for r, e in errors.items()} } samples; "
             f"limit {pose_p95_limit_m} m"),
            per_robot=errors, worst_p95_m=worst, limit_m=pose_p95_limit_m,
        ))

    # Does asking the robot where it was give a different answer from truth?
    if not mapping:
        checks.append(_check("estimate_and_truth_agree_on_containment", NOT_RUN,
                             attribution["reason"]))
    elif conservative:
        checks.append(_check(
            "estimate_and_truth_agree_on_containment", NOT_RUN,
            "cannot be asked of this recording: the truth test is conservative (no heading) "
            "and the odometry test is exact, so they differ by construction. Reporting that "
            "difference as a localisation error would be a fabricated finding. Record with "
            "fleet_evaluation.recorder to get truth headings and ask this question.",
            geometry="conservative",
        ))
    else:
        checks.append(_check(
            "estimate_and_truth_agree_on_containment",
            PASS if disagreements == 0 else FAIL,
            (f"the odometry-based and truth-based containment answers differ in "
             f"{disagreements} of {len(samples) * len(mapping)} robot-samples"
             if disagreements else
             "every sample gave the same containment answer whether asked of the estimate "
             "or of the world"),
            disagreements=disagreements,
        ))

    # --- the claims --------------------------------------------------------- #
    corroborated: list[dict[str, Any]] = []
    contradicted: list[dict[str, Any]] = []
    for driver in case["drivers"]:
        robot = driver.get("robot")
        entry = {"robot": robot, "direction": driver.get("direction") or "",
                 "complete": bool(driver.get("complete"))}
        if not entry["complete"]:
            entry["verdict"] = "NOT_CLAIMED"
            corroborated.append(entry)
            continue
        slot = next((int(k) for k, r in mapping.items() if r == robot), None)
        if slot is None:
            entry["verdict"] = "UNCHECKED"
            entry["why"] = "truth could not be attributed to this robot"
            corroborated.append(entry)
            continue
        first = last = None
        for sample in samples:
            slots = _truth_slots(sample)
            if slot not in slots:
                continue
            if first is None:
                first = slots[slot][:2]
            last = slots[slot][:2]
        if first is None or last is None:
            entry["verdict"] = "UNCHECKED"
            entry["why"] = "no truth positions for this robot"
            corroborated.append(entry)
            continue
        travelled = math.hypot(last[0] - first[0], last[1] - first[1])
        crossed = (first[0] < 0) != (last[0] < 0)
        entry.update({
            "truth_start": [round(v, 3) for v in first],
            "truth_end": [round(v, 3) for v in last],
            "truth_travelled_m": round(travelled, 3),
            "truth_crossed_barrier": crossed,
        })
        if crossed and travelled >= 2.0:
            entry["verdict"] = "CORROBORATED"
            corroborated.append(entry)
        else:
            entry["verdict"] = "CONTRADICTED"
            entry["why"] = ("the driver reported complete=True; truth shows it did not "
                            "cross the barrier" if not crossed else
                            "the driver reported complete=True; truth shows it barely moved")
            contradicted.append(entry)

    checked = [c for c in corroborated if c.get("verdict") != "UNCHECKED"]
    unchecked = [c for c in corroborated if c.get("verdict") == "UNCHECKED"]
    not_claimed = [c for c in corroborated if c.get("verdict") == "NOT_CLAIMED"]
    # UNCHECKED is not corroboration. A detail line reading "consistent with truth"
    # when nothing was checked is the failure this package exists to prevent, so the
    # count is by outcome and an all-unchecked result is NOT_RUN, not PASS.
    #
    # And neither is NOTHING CHECKED a corroboration. `checked == 0` happened on every case of
    # the regression batch -- the runner writes no driver reports -- and the formula below used
    # to answer PASS, with the detail line next to it reading "0 corroborated". A check that
    # looked at nothing has not passed; it has not been performed.
    verdict = FAIL if contradicted else (NOT_RUN if (unchecked or not checked) else PASS)
    checks.append(_check(
        "driver_claims_corroborated_by_truth",
        verdict,
        (f"{len(checked)} claim(s) corroborated by truth, {len(unchecked)} unchecked, "
         f"{len(not_claimed)} not claimed, {len(contradicted)} contradicted"
         + ("" if not unchecked else
            " -- an unchecked claim is not a corroborated one")
         + ("" if checked else
            " -- and a check that was given no claim to corroborate is NOT_RUN, not a pass")),
        corroborated=corroborated, contradicted=contradicted,
        unchecked=unchecked, checked=checked,
    ))

    # --- the recorder's own audit ------------------------------------------- #
    times = [s.get("t") for s in samples if isinstance(s.get("t"), (int, float))]
    span = (times[-1] - times[0]) if len(times) > 1 else 0.0
    gaps = [b - a for a, b in zip(times, times[1:])]
    observed = (len(times) - 1) / span if span > 0 else 0.0
    checks.append(_check(
        "recorder_sampling_audit", HOLD,
        (f"{len(times)} samples over {span:.1f} s = {observed:.2f} Hz; worst gap "
         f"{max(gaps) if gaps else 0:.2f} s. HOLD rather than PASS: a rate this process "
         "achieved says nothing about what it failed to receive."),
        kind="caveat", samples=len(times), observed_hz=round(observed, 2),
        worst_gap_s=round(max(gaps), 3) if gaps else None,
    ))

    blocking = [c for c in checks if c.get("kind") != "caveat"]
    failed = [c["check"] for c in blocking if c["verdict"] == FAIL]
    not_run = [c["check"] for c in blocking if c["verdict"] == NOT_RUN]
    held = [c["check"] for c in blocking if c["verdict"] == HOLD]
    caveats = [c["check"] for c in checks if c.get("kind") == "caveat"]
    summary = FAIL if failed else (NOT_RUN if (not_run or held) else PASS)
    return {
        "case": str(case_dir),
        "samples_file": case["samples_file"],
        "samples": len(samples),
        "robots_judged": sorted(judged),
        "robots_absent": absent,
        "truth_has_heading": exact,
        "published_verdict_file": case["published_verdict_file"],
        "summary": summary,
        "failed": failed,
        "not_run": not_run,
        "held": held,
        "caveats": caveats,
        "checks": checks,
    }


# --------------------------------------------------------------------------- #
# cli
# --------------------------------------------------------------------------- #

def discover_cases(run_dir: pathlib.Path) -> list[pathlib.Path]:
    if any(run_dir.glob("samples-*.jsonl")):
        return [run_dir]
    return sorted(p for p in run_dir.iterdir()
                  if p.is_dir() and list(p.glob("samples-*.jsonl")))


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(
        prog="fleet_eval",
        description="Judge a recorded run from ground truth. Read-only, no ROS.")
    ap.add_argument("--run-dir", required=True,
                    help="a case directory, or a run directory holding case directories")
    ap.add_argument("--traffic-config", default="config/resources.yaml")
    ap.add_argument("--spawns", default="config/spawns.yaml")
    ap.add_argument("--pose-p95-limit-m", type=float, default=0.35)
    ap.add_argument("--odom-frame", choices=["auto", "map", "spawn"], default="auto",
                    help="frame the recorded odom is in; auto decides from the data")
    ap.add_argument("--out", default="")
    args = ap.parse_args(argv)

    import yaml

    cfg = validate_traffic_config(
        yaml.safe_load(pathlib.Path(args.traffic_config).read_text(encoding="utf-8")))
    spawns = load_spawns(pathlib.Path(args.spawns))

    run_dir = pathlib.Path(args.run_dir)
    cases = discover_cases(run_dir)
    if not cases:
        print(f"fleet_eval: no case with samples-*.jsonl under {run_dir}", file=sys.stderr)
        return 3

    results = [judge_case(case, cfg, spawns, pose_p95_limit_m=args.pose_p95_limit_m,
                          odom_frame=args.odom_frame)
               for case in cases]

    for result in results:
        print(f"\n=== {result['case']}")
        print(f"    {result['samples']} samples, {result['samples_file']}, "
              f"robots judged {result['robots_judged']}"
              + (f", absent {result['robots_absent']}" if result["robots_absent"] else "")
              + f", truth heading {'yes' if result['truth_has_heading'] else 'NO'}")
        for check in result["checks"]:
            print(f"  [{check['verdict']:>7}] {check['check']}")
            print(f"            {check['detail']}")
        print(f"  summary: {result['summary']}"
              + (f"   failed={result['failed']}" if result["failed"] else "")
              + (f"   not_run={result['not_run']}" if result["not_run"] else ""))
        for caveat in result.get("caveats", []):
            print(f"  (caveat) {caveat}")

    doc = {"run_dir": str(run_dir), "cases": results,
           "note": ("Judged from the world's own pose stream. The fleet's ledger, the gate's "
                    "verdicts and the driver reports are cross-checked as CLAIMS, not used "
                    "as evidence.")}
    if args.out:
        pathlib.Path(args.out).write_text(json.dumps(doc, indent=2), encoding="utf-8")
        print(f"\nfleet_eval: wrote {args.out}")
    worst = (FAIL if any(r["summary"] == FAIL for r in results)
             else NOT_RUN if any(r["summary"] == NOT_RUN for r in results) else PASS)
    print(f"\nfleet_eval: {worst}")
    return 0 if worst == PASS else (1 if worst == FAIL else 3)


if __name__ == "__main__":
    raise SystemExit(main())
