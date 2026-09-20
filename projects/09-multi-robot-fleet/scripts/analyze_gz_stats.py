"""Turn a gz WorldStatistics stream into loop-rate numbers. No ROS involved.

gz publishes its own accounting on /world/<world>/stats: cumulative sim time, wall time
and iteration count. Reading it needs nothing from the ROS side, which is exactly the
point -- the ROS graph looks perfectly healthy while every topic on it collapses to the
same rate, so the ROS graph cannot be the witness for a gz-side problem.

`gz topic -e` prints the message as protobuf text, so this parses by FIELD NAME rather
than by position. If the schema changes, the failure should surface as "no records"
rather than as a plausible-looking wrong number.

Reported per segment:
  steps/sim-s   iterations per second of SIMULATED time. This is the configured
                physics rate (1 / max_step_size) and should be flat at 1000.
  steps/wall-s  iterations per second of WALL time. When this falls below steps/sim-s,
                gz is falling behind real time.
  rtf           gz's own real_time_factor.
  sim/wall      the ratio actually achieved over the segment.

What to look for: if steps/sim-s stays at 1000 while rtf drops, gz is still stepping the
physics correctly and the whole loop is simply slow -- i.e. something per ITERATION is
expensive, and every plugin that publishes in PostUpdate (the lidar, joint states) is
throttled even though the physics is fine.
"""

from __future__ import annotations

import re
import statistics
import sys
from pathlib import Path

BLOCK = re.compile(r"^(sim_time|real_time)\s*\{$")


def parse(text: str) -> list[dict[str, float]]:
    lines = text.splitlines()
    recs: list[dict[str, float]] = []
    cur: dict[str, float] = {}
    i = 0
    while i < len(lines):
        ln = lines[i].strip()
        m = BLOCK.match(ln)
        if m:
            name = m.group(1)
            sec = nsec = None
            j = i + 1
            while j < len(lines) and "}" not in lines[j]:
                mm = re.match(r"^(sec|nsec):\s*(\d+)", lines[j].strip())
                if mm:
                    if mm.group(1) == "sec":
                        sec = int(mm.group(2))
                    else:
                        nsec = int(mm.group(2))
                j += 1
            if sec is not None:
                cur[name] = sec + (nsec or 0) * 1e-9
            i = j
        elif ln.startswith("iterations:"):
            try:
                cur["iterations"] = float(ln.split(":", 1)[1])
            except ValueError:
                pass
        elif ln.startswith("real_time_factor:"):
            try:
                cur["rtf"] = float(ln.split(":", 1)[1])
            except ValueError:
                pass
            if "sim_time" in cur and "iterations" in cur:
                recs.append(cur)
            cur = {}
        i += 1
    return recs


def segment(path: Path, label: str) -> None:
    text = path.read_text(encoding="utf-8", errors="replace")
    recs = parse(text)
    print()
    print(f"  === {label}  ({path.name}) ===")
    if len(recs) < 3:
        print(f"      only {len(recs)} parsable record(s). NOT_RUN.")
        if recs:
            print(f"      first record keys: {sorted(recs[0])}")
        else:
            head = text.splitlines()[:12]
            print("      first lines of the raw stream:")
            for ln in head:
                print(f"        {ln}")
        return

    sim, wall, it, rtf = [], [], [], []
    for a, b in zip(recs, recs[1:]):
        ds = b["sim_time"] - a["sim_time"]
        dw = b["real_time"] - a["real_time"]
        dn = b["iterations"] - a["iterations"]
        if ds <= 0 or dw <= 0 or dn < 0:
            continue
        sim.append(dn / ds)
        wall.append(dn / dw)
        rtf.append(b.get("rtf", float("nan")))

    if not sim:
        print("      records present but no usable intervals. NOT_RUN.")
        return

    sim_span = recs[-1]["sim_time"] - recs[0]["sim_time"]
    wall_span = recs[-1]["real_time"] - recs[0]["real_time"]
    print(f"      records {len(recs)}   intervals {len(sim)}")
    print(f"      sim span {sim_span:8.2f} s   wall span {wall_span:8.2f} s   "
          f"achieved sim/wall {sim_span / wall_span:.3f}")
    print(f"      steps/sim-s   median {statistics.median(sim):9.1f}   "
          f"min {min(sim):9.1f}   max {max(sim):9.1f}")
    print(f"      steps/wall-s  median {statistics.median(wall):9.1f}   "
          f"min {min(wall):9.1f}   max {max(wall):9.1f}")
    print(f"      reported rtf  median {statistics.median(rtf):9.3f}")

    # timeline, so a transition inside a segment is visible rather than averaged away
    bins = 8
    n = len(sim)
    step = max(1, n // bins)
    print("      timeline (per bin):")
    print(f"        {'t_sim':>8}{'t_wall':>9}{'steps/sim':>11}{'steps/wall':>12}{'rtf':>8}")
    t0s, t0w = recs[0]["sim_time"], recs[0]["real_time"]
    for k in range(0, n, step):
        chunk = slice(k, min(k + step, n))
        s = sim[chunk]
        w = wall[chunk]
        r = rtf[chunk]
        mid = recs[min(k + step, len(recs) - 1)]
        print(f"        {mid['sim_time'] - t0s:8.2f}{mid['real_time'] - t0w:9.2f}"
              f"{statistics.median(s):11.1f}{statistics.median(w):12.1f}"
              f"{statistics.median(r):8.3f}")


def main(argv: list[str]) -> int:
    if not argv:
        print(__doc__)
        return 2
    print("=" * 78)
    print("  gz's own loop rate, from /world/<world>/stats  (no ROS in the path)")
    print("=" * 78)
    for spec in argv:
        # accept "label=path" or just "path"
        if "=" in spec and not Path(spec).exists():
            label, _, p = spec.partition("=")
        else:
            label, p = Path(spec).stem, spec
        path = Path(p)
        if not path.is_file():
            print(f"\n  === {label} ===\n      missing: {path}")
            continue
        segment(path, label)
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
