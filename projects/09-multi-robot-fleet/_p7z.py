"""Is the three-robot truth frozen, or is the sampling the problem?

N07/N08_seed2 report 0.000 m of truth displacement for all three robots over 9000-39000
samples. Two-robot cases in the same batches move normally. The difference has to be in the
stream, so look at the raw stream: does the truth array CHANGE at all, and does the odometry?

If odom moves and truth does not, the world stream is reporting a stale pose -- and every
truth-based check in those cases is judging a frozen world.
"""
import json, math, os

CASES = [
    ("N07", "reports/batch_20260918T103152Z/N07/samples-N07.jsonl", ["r01", "r02", "r03"]),
    ("N08_seed2", "reports/batch_20260918T113422Z/N08_seed2/samples-N08_seed2.jsonl",
     ["r01", "r02", "r03"]),
    ("F09", "reports/batch_20260918T110845Z/F09/samples-F09.jsonl", ["r01", "r02"]),
]

for lab, path, robots in CASES:
    if not os.path.exists(path):
        print("MISSING", path)
        continue
    print("=== %s" % lab)
    odom_moved = {r: 0.0 for r in robots}
    truth_moved = {}
    truth_samples = 0
    distinct_truth = set()
    first_odom = {r: None for r in robots}
    for line in open(path):
        line = line.strip()
        if not line:
            continue
        try:
            d = json.loads(line)
        except Exception:
            continue
        od = d.get("odom") or {}
        for r in robots:
            v = od.get(r)
            if v is None:
                continue
            if first_odom[r] is None:
                first_odom[r] = v
            odom_moved[r] = max(odom_moved[r], math.hypot(v[0] - first_odom[r][0],
                                                           v[1] - first_odom[r][1]))
        t = d.get("truth") or {}
        if t:
            truth_samples += 1
            for k, v in t.items():
                ks = str(k)
                truth_moved.setdefault(ks, [v[0], v[1], 0.0])
                truth_moved[ks][2] = max(truth_moved[ks][2],
                                         math.hypot(v[0] - truth_moved[ks][0],
                                                    v[1] - truth_moved[ks][1]))
            if len(distinct_truth) < 5:
                distinct_truth.add(tuple(sorted((k, round(v[0], 3), round(v[1], 3))
                                                for k, v in t.items())[:3]))
    print("    odom max |displacement| per robot: %s"
          % {r: round(v, 3) for r, v in odom_moved.items()})
    print("    truth samples=%d  distinct leading triples seen=%d"
          % (truth_samples, len(distinct_truth)))
    mv = {k: round(v[2], 3) for k, v in sorted(truth_moved.items(), key=lambda x: int(x[0]))
          if int(k) < 3}
    print("    truth max |displacement| for slots 0/1/2: %s" % mv)
    print()
