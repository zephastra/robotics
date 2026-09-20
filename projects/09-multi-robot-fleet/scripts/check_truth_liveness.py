#!/usr/bin/env python3
"""Guard: a recording whose truth stream did not move must SAY so.

Why this exists rather than a comment
-------------------------------------
Round 17 measured three-robot recordings in which the truth stream carried a
well-formed 24-slot message for every sample and yet NO model frame ever moved:
0 of 9141 samples in N07/T103152Z, 0 of 38960 in N08_seed2. The judge read that
stream as the arbiter and returned `safety = PASS/PASS/PASS`.

A frozen arbiter is indistinguishable from a clean run by looking at the verdict.
The only place the difference was visible was a counter (`truth_messages`) compared
against the sample count -- two numbers in two different files, which is not a check.

So the recorder decides liveness itself and writes the verdict into the audit,
and this guard fails a recording that is present but not alive. The distinction it
enforces is:

    truth_liveness == "live"    -> the stream moved; the judge's verdicts mean something
    truth_liveness == "frozen"  -> the stream did not move; NO safety verdict from it
                                   may be read as a pass, and the run must be redone
    truth_liveness == "absent"  -> no truth at all; truth-derived checks are NOT_RUN
    truth_liveness == "partial" -> SOME model body never moved while others did; the
                                   recording is neither clean nor wholly frozen

Run with `--self-test` to check the criterion itself against synthetic
recordings whose verdict is known by construction. See D-P17-14.

`frozen` is deliberately NOT treated as "no data": it is worse. Absent truth makes
every truth check NOT_RUN, which is honest. Frozen truth makes them PASS, which is not.

What is actually measured (2026-09-19, second pass)
--------------------------------------------------
Count the model frames' successive-distinct values. Slot 0 carries a model
frame in world coordinates; slots 5-9 are LINK OFFSETS inside the robot and
must not be read as positions (doing so once produced a false 'the robots had
moved 5.8 m' claim -- see D-P17-08's second correction).

    N07 T103152Z     3 robots  slot0 distinct-consecutive = 1 over  8738 samples
    N08_seed2        3 robots  slot0 distinct-consecutive = 1 over 38679 samples
    N05 T015759Z     2 robots  slot0 distinct-consecutive = 4921, travels 12 m

So the three-robot stream delivers ONE frame and repeats it verbatim. That is
not a slow queue drain -- a queue that drains at all eventually advances. Nor
is it merely a starved publisher, which would deliver nothing rather than the
same frame thousands of times. The arrival-rate gap (0.13-2.75 Hz for three
robots vs 52-57 Hz for two) is a symptom of the same fault.

Meanwhile the same N07 run's odometry shows r01/r02/r03 travelling 37.15 /
31.17 / 26.28 m. The robots drove; the arbiter says they never moved a
millimetre.

CORRECTION (D-P17-20, 2026-09-19): "model m" is NOT "robot m". The stream
carries a fixed three models -- one robot body plus two same-footprint
fixtures -- in every recording, and the count does not follow the case's
declared robot count. So `slot0_distinct` is model 0's count and nothing
more, and a pinned index is the normal state of a fixture. Read the
`roots[...]` line and the case's declared runners together before calling a
pinned index a defect. D-P17-20 correction applied. `scripts/batch.py` therefore has to run three-robot cases again
once the channel works, and NO three-robot safety verdict on record is
quotable. The root cause inside SceneBroadcaster / ros_gz_bridge is NOT yet
attributed -- there is no probe on the gz side of the bridge.
"""

from __future__ import annotations

import json
import pathlib
import sys

#: A MODEL is alive if its ROOT slot advanced at least this many times. 2 means
#: "changed at least once", expressed as a count so the reader sees the judgement
#: is on the root's successive-distinct count and not on a movement fraction.
#:
#: This criterion REPLACED a fraction-of-moved-samples test (`LIVE_FRACTION`).
#: That test was calibrated against whole-stream freezes only -- measured live
#: runs 0.75-0.82, frozen exactly 0.00 -- and so could not see a PER-MODEL
#: freeze. With 24 slots a sample counts as moved if ANY slot advanced, and
#: slots 1-7 carry wheel link offsets driven by /joint_states, a stream
#: independent of the model's world pose. A model whose body pose froze still
#: had spinning wheels. Measured (batch_20260919T022011Z): the old criterion
#: reported all 11 recordings `live` while five of them had slot 0 pinned --
#: model 0's body never moved a millimetre for the whole run. See D-P17-14.
ROOT_ALIVE_MIN_DISTINCT = 2

#: Kept ONLY as the fallback for a recording that exposes no root slot at all,
#: where the per-model test has nothing to read. In that case the old
#: any-slot-moved fraction is still better than inventing a verdict.
LIVE_FRACTION = 0.01

#: Below this many truth-bearing samples, liveness cannot be judged and the verdict is
#: `unknown` rather than a guess.
MIN_SAMPLES = 50

#: Links per model in the world. The truth message is a flat list of links, so the
#: number of bodies is `slots // LINKS_PER_MODEL`. Kept as arithmetic rather than a
#: hardcoded 24 so a different world or robot count cannot make this silently wrong.
LINKS_PER_MODEL = 8


def liveness(path: pathlib.Path) -> tuple[str, dict]:
    """Return (verdict, evidence) for one samples file."""
    n = truth_bearing = moved = 0
    root_distinct = 0
    prev_root: tuple[float, float] | None = None
    first_seen: dict[int, tuple[float, float]] = {}
    last_seen: dict[int, tuple[float, float]] = {}
    max_jump = 0.0
    # Per-model root tracking (D-P17-14). Keyed by MODEL index, not slot index.
    root_distinct_by_model: dict[int, int] = {}
    prev_root_by_model: dict[int, tuple[float, float]] = {}

    with open(path, encoding="utf-8") as fh:
        for line in fh:
            line = line.strip()
            if not line:
                continue
            try:
                s = json.loads(line)
            except json.JSONDecodeError:
                continue
            n += 1
            t = s.get("truth") or {}
            if not t:
                continue
            truth_bearing += 1
            cur = {}
            for k, v in t.items():
                try:
                    cur[int(k)] = (float(v[0]), float(v[1]))
                except (TypeError, ValueError, IndexError):
                    continue
            for slot, pos in cur.items():
                if slot in last_seen:
                    dx = pos[0] - last_seen[slot][0]
                    dy = pos[1] - last_seen[slot][1]
                    d = (dx * dx + dy * dy) ** 0.5
                    if d > 1e-4:
                        moved += 1
                        max_jump = max(max_jump, d)
                        break
            # Slot 0 is a MODEL frame in world coordinates. Its successive-distinct
            # count is the honest measure of whether the stream is advancing:
            # a stream that delivers one frame and repeats it scores 1, a live
            # three-robot run scores in the hundreds. Counting "did any slot
            # move" alone is weaker -- and reading slots 5-9 as positions is
            # simply wrong, they are link offsets inside the robot.
            root = cur.get(0)
            if root is not None:
                if root != prev_root:
                    root_distinct += 1
                    prev_root = root
            # Per-model roots. Model m's body is slot m * LINKS_PER_MODEL, so a
            # three-robot world tracks slots 0, 8 and 16 independently. Without
            # this, one frozen model hides behind its neighbours' rotating wheels.
            for model in range(len(cur) // LINKS_PER_MODEL):
                rpos = cur.get(model * LINKS_PER_MODEL)
                if rpos is None:
                    continue
                if rpos != prev_root_by_model.get(model):
                    root_distinct_by_model[model] = root_distinct_by_model.get(model, 0) + 1
                    prev_root_by_model[model] = rpos
            for slot, pos in cur.items():
                first_seen.setdefault(slot, pos)
                last_seen[slot] = pos

    evidence = {
        "samples": n,
        "truth_bearing_samples": truth_bearing,
        "samples_with_movement": moved,
        "max_jump_m": round(max_jump, 4),
        "slots": len(last_seen),
        "models_in_world": len(last_seen) // LINKS_PER_MODEL,
        "slots_left_over": len(last_seen) % LINKS_PER_MODEL,
        "root_slot_distinct": root_distinct,
    }
    # Per MODEL, from each model's root slot. `LINKS_PER_MODEL` is the stride, so
    # model m's root is slot m * LINKS_PER_MODEL. A model is alive if its root
    # advanced; the recording is frozen only if NO model did, and `partial` when
    # some did and some did not -- partial is neither a clean run nor a wholly
    # frozen one, and folding it into either would hide a defect (D-P17-14).
    alive_models = sorted(m for m, d in root_distinct_by_model.items()
                          if d >= ROOT_ALIVE_MIN_DISTINCT)
    dead_models = sorted(m for m, d in root_distinct_by_model.items()
                         if d < ROOT_ALIVE_MIN_DISTINCT)
    evidence["root_distinct_by_model"] = {str(m): d
                                          for m, d in sorted(root_distinct_by_model.items())}
    evidence["alive_models"] = alive_models
    evidence["dead_models"] = dead_models
    if truth_bearing < MIN_SAMPLES:
        return "unknown", evidence
    if not root_distinct_by_model:
        # No root slot was ever seen. Fall back to the any-slot-moved signal
        # rather than inventing a verdict from nothing.
        if moved / truth_bearing < LIVE_FRACTION:
            return "frozen", evidence
        return "live", evidence
    if not alive_models:
        return "frozen", evidence
    if dead_models:
        return "partial", evidence
    return "live", evidence


def was_recorded_by_fixed_code(sp: pathlib.Path) -> bool:
    """True if this recording's audit declares the deep truth QoS.

    Used to scope the guard to recordings the fix could have affected. A
    recording written before the fix cannot be a regression, so judging it
    would fail the build for history rather than for a defect.
    """
    case_dir = sp.parent
    for audit in case_dir.glob("recorder-*.json"):
        try:
            data = json.loads(audit.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError):
            continue
        if "truth_qos_depth" in data:
            return True
    return False


def robots_expected(sp: pathlib.Path) -> list[str]:
    """The robots the recorder was asked to record.

    This is what the world MUST hold bodies for. An audit that names a robot
    the launch never spawned is the defect this comparison exists to catch, so
    the caller is told which list it used.
    """
    for audit in sp.parent.glob("recorder-*.json"):
        try:
            data = json.loads(audit.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError):
            continue
        robots = data.get("robots")
        if isinstance(robots, list):
            return [str(r) for r in robots]
        if isinstance(robots, str):
            return [r.strip() for r in robots.split(",") if r.strip()]
    return []


# --------------------------------------------------------------------------- #
# self-test (D-P17-14)
# --------------------------------------------------------------------------- #
#: Build recordings whose liveness is known BY CONSTRUCTION and assert the criterion.
#: This exists because the criterion was measured WRONG in the silent direction: it
#: called a per-model freeze `live`, and no amount of green on real data would have
#: shown it -- the real data was the thing being mis-judged. SELFTEST_MARKER_D_P17_14
def self_test() -> int:
    import tempfile

    def build(path, *, models, samples, frozen, wheels, truth=True):
        with open(path, "w", encoding="utf-8") as fh:
            for i in range(samples):
                rec = {"t": round(i * 0.05, 3)}
                if truth:
                    t = {}
                    for m in range(models):
                        base = m * LINKS_PER_MODEL
                        t[str(base)] = ([float(m), 0.0] if m in frozen
                                        else [float(m) + i * 0.01, 0.0])
                        # Link offsets are a DIFFERENT stream (wheel angles come from
                        # /joint_states). They keep changing when the body is pinned,
                        # which is precisely why an any-slot-moved test cannot see it.
                        for j in range(1, LINKS_PER_MODEL):
                            t[str(base + j)] = [float(m), float(j),
                                                (i * 0.1 if wheels else 0.0)]
                    rec["truth"] = t
                fh.write(json.dumps(rec) + "\n")
        return path

    tmp = pathlib.Path(tempfile.mkdtemp(prefix="liveness_selftest_"))
    cases = [
        ("all bodies move",       dict(models=3, samples=200, frozen=set(), wheels=True), "live"),
        ("one body pinned",       dict(models=3, samples=200, frozen={0}, wheels=True), "partial"),
        ("all bodies pinned",     dict(models=3, samples=200, frozen={0, 1, 2}, wheels=True), "frozen"),
        ("all pinned, no wheels", dict(models=3, samples=200, frozen={0, 1, 2}, wheels=False), "frozen"),
        ("no truth at all",       dict(models=3, samples=200, frozen=set(), wheels=True, truth=False), "unknown"),
    ]
    bad = []
    for i, (label, kw, want) in enumerate(cases, 1):
        got, ev = liveness(build(tmp / f"st{i}.jsonl", **kw))
        if got != want:
            bad.append(f"{label}: want {want}, got {got}")
        print(f"  [{'ok  ' if got == want else 'FAIL'}] self-test {i} {label:22s} "
              f"want={want:8s} got={got:8s} roots={ev.get('root_distinct_by_model')}")
    if bad:
        print(f"  self-test FAILED: {'; '.join(bad)}")
        return 1
    print(f"  self-test: {len(cases)} case(s) passed")
    return 0


def main(argv: list[str]) -> int:
    if "--self-test" in argv:
        return self_test()
    args = [a for a in argv[1:] if not a.startswith("--")]
    audit_all = "--all" in argv
    roots = [pathlib.Path(a) for a in args] or [pathlib.Path("reports")]

    rows = []
    skipped = 0
    for root in roots:
        for sp in sorted(root.rglob("samples-*.jsonl")):
            # A directory whose name starts with "_" is not a result set. This is
            # how `reports/_invalid/` -- recordings kept only as evidence of a
            # mistaken run -- stays out of the verdict, without deleting history.
            if any(part.startswith("_") for part in sp.parts):
                continue
            if not audit_all and not was_recorded_by_fixed_code(sp):
                skipped += 1
                continue
            verdict, ev = liveness(sp)
            rows.append((sp, verdict, ev))

    # ONE label, one verdict: a label's CURRENT evidence is its NEWEST recording. An older
    # recording of the same label is history -- it stays on disk and is listed below, but it
    # must not gate the build, for the same reason `--all` does not: failing on it would fail
    # for history rather than for a defect. Measured 2026-09-19: five labels were re-run after
    # their recordings froze, the re-runs came back live, and the guard stayed red because the
    # superseded recordings were still being judged.
    newest: dict = {}
    for sp, verdict, ev in rows:
        label, batch = sp.parent.name, sp.parent.parent.name
        if label not in newest or batch > newest[label][0]:
            newest[label] = (batch, sp, verdict, ev)
    superseded = [(sp, v) for sp, v, _ev in rows if sp != newest[sp.parent.name][1]]
    rows = [(sp, v, ev) for _b, sp, v, ev
            in sorted(newest.values(), key=lambda t: str(t[1]))]

    if not rows:
        print("check_truth_liveness: no post-fix samples-*.jsonl found"
              + (f" ({skipped} pre-fix recording(s) skipped)" if skipped else ""))
        return 0

    n_frozen = n_unknown = n_short = n_partial = 0
    for sp, verdict, ev in rows:
        mark = {"live": "  live ", "frozen": "FROZEN", "unknown": "  ?   ",
                "partial": "PARTIAL"}[verdict]
        expected = robots_expected(sp)
        short = bool(expected) and ev["models_in_world"] < len(expected)
        note = ""
        if short:
            n_short += 1
            note = (f"  <== SHORT: {ev['models_in_world']} model(s) for "
                    f"{len(expected)} robot(s)")
        print(f"  [{mark}] {sp}")
        print(f"           samples={ev['samples']} truth={ev['truth_bearing_samples']} "
              f"moved={ev['samples_with_movement']} slots={ev['slots']} "
              f"models={ev['models_in_world']} max_jump={ev['max_jump_m']}m "
              f"slot0_distinct={ev['root_slot_distinct']}{note}")
        # The per-MODEL roots, in the same line block. Which index is pinned is the
        # whole question -- `slot0_distinct` is only model 0's count, and model 0 is
        # not necessarily "the robot" (D-P17-20). Run the numbers before reading
        # anything into a pinned index.
        if ev["root_distinct_by_model"]:
            roots = " ".join(f"m{m}={d}" for m, d in
                             ev["root_distinct_by_model"].items())
            print(f"           roots[{roots}]")
        if verdict == "frozen":
            n_frozen += 1
        elif verdict == "partial":
            n_partial += 1
        elif verdict == "unknown":
            n_unknown += 1

    print()
    print(f"  recordings: {len(rows)}   live: "
          f"{len(rows) - n_frozen - n_unknown - n_partial}   "
          f"FROZEN: {n_frozen}   PARTIAL: {n_partial}   unknown: {n_unknown}   "
          f"short: {n_short}")

    if n_short:
        print()
        print(f"  {n_short} recording(s) are SHORT: they carry NO model body at all.")
        print("  Measured 2026-09-19 on the two on file: the recorder audit says")
        print("  `truth_messages == 0`, while the launch log shows all three spawn poses")
        print("  ('spawn pose for r01/r02/r03') and three `create-NN` processes, so")
        print("  `robots:=` WAS passed and the world DID hold three bodies.")
        print("  The earlier version of this message blamed a caller that forgot")
        print("  `robots:=`; that explanation is DISPROVED by the evidence above and is")
        print("  kept here only so the wrong cause is not re-derived. `batch.py` builds")
        print("  `robots:=` from the case body's `runners` (guard 11 asserts it).")
        print("  What SHORT actually means: the truth channel delivered nothing, so every")
        print("  truth-derived check is NOT_RUN and no verdict from this recording may be")
        print("  read. A starved channel and a body that was never created cannot be told")
        print("  apart from the sample file -- either way the case must be re-run.")

    if n_partial:
        print()
        print("  A PARTIAL recording has at least one model whose body pose never moved.")
        print("  READ THE INDEX BEFORE READING ANYTHING INTO IT. Measured (D-P17-20): the")
        print("  truth message carries a FIXED THREE models -- a robot body plus two")
        print("  same-footprint fixtures that never travel -- and the count does NOT change")
        print("  with the number of robots the case declares. A single-robot case and a")
        print("  two-robot case both report models=3. So a pinned index is the NORMAL state")
        print("  of a fixture, and PARTIAL on its own is not a defect.")
        print("  What IS a defect: the index that pins changing from case to case on the")
        print("  same world (measured: model 2 pinned in N05, model 0 pinned in N01). That")
        print("  is the signature of a fixture rather than a frozen vehicle. A pinned")
        print("  VEHICLE cannot be told apart from a pinned fixture by this guard -- check")
        print("  the case's declared runners against the roots[...] line above.")
        print("  Also note `slot0_distinct` is model 0's count ONLY; it is not a")
        print("  robot-tracking number. [D-P17-20 correction applied]")

    if n_frozen:
        print()
        print("  A FROZEN recording is not a clean run. The truth stream delivered ONE")
        print("  frame and repeated it for the whole run (slot0_distinct == 1), so the")
        print("  judge's safety verdicts were taken against a world that only moved in")
        print("  the odometry. Do not read those verdicts as passes. Re-run the case.")
        print("  Measured (batch_20260919T022011Z, 2026-09-19): six of 33 recordings")
        print("  scored every model at 1 -- F05 F07 F13 I02 I08 N02. Five of those six")
        print("  are fault-injection cases whose whole point is that the robot KEEPS")
        print("  WORKING. The same batch has live runs (F01/F02 reproduce the robot's")
        print("  world pose ~1026/1236 times over ~1640/1851 samples), so the channel can")
        print("  work; it collapses to one frame for some runs and not others.")
        print("  The gz side was probed separately and the bridge's subscriber there was")
        print("  observed flapping (present in 3 of 14 live polls), so the fault is in")
        print("  the bridge<->gz transport, NOT in this subscription's queue depth.")
        print("  ⛔ STILL NOT ATTRIBUTED, and the distinction matters: a frozen ROBOT body")
        print("  is a broken instrument, a frozen FIXTURE is expected (D-P17-20). For")
        print("  these six, model 0 -- the body that moves in every live run -- is the")
        print("  one pinned, so these are instrument failures, not fixture artefacts.")

    if superseded:
        print()
        print(f"  superseded (an OLDER recording of a label whose newest recording is "
              f"judged above): {len(superseded)}")
        for sp, v in superseded:
            print(f"    [{v}] {sp}")
        print("  Kept, not deleted, and NOT fatal: the label's current evidence is its")
        print("  newest recording, and a re-run is the fix for a frozen one. Quote the")
        print("  newest; do not read a superseded recording as the label's verdict.")

    if skipped:
        print()
        print(f"  skipped (written before the QoS fix, no truth_qos_depth in the audit): {skipped}")
        print("  audit those with: scripts/check_truth_liveness.py --all")

    # Guard mode judges only recordings the fix could have affected, so a
    # non-zero exit is a real regression. --all is a survey of the whole history,
    # most of which predates the fix; failing a build on it would fail for
    # history rather than for a defect. The distinction is deliberate. A SUPERSEDED
    # recording -- an older run of a label whose newest run is judged above -- is the same
    # kind of thing and is counted and printed, never fatal.
    if audit_all:
        return 0
    return 1 if (n_frozen or n_partial or n_short) else 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
