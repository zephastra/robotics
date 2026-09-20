import json, math, glob, os

MAPPING = {"0": "r01", "1": "r03", "2": "r02"}
PATHS = [
    "reports/batch_20260918T113422Z/N08_seed2/samples-N08_seed2.jsonl",
    "reports/batch_20260918T103152Z/N07/samples-N07.jsonl",
    "reports/batch_20260918T110845Z/F09/samples-F09.jsonl",
]

for p in PATHS:
    if not os.path.exists(p):
        print("MISSING", p)
        continue
    first, last = {}, {}
    n = truth_rows = 0
    for line in open(p):
        line = line.strip()
        if not line:
            continue
        n += 1
        try:
            d = json.loads(line)
        except Exception:
            continue
        t = d.get("truth")
        if not isinstance(t, dict) or not t:
            continue
        truth_rows += 1
        for k, v in t.items():
            ks = str(k)
            if ks not in first:
                first[ks] = v
            last[ks] = v
    print("=== %s" % p)
    print("    rows=%d truth_rows=%d slots_seen=%s" % (n, truth_rows, sorted(first, key=int)))
    for r, slot in (("r01", "0"), ("r03", "1"), ("r02", "2")):
        a = first.get(slot)
        b = last.get(slot)
        if a is None or b is None:
            print("    %s slot %s: absent" % (r, slot))
            continue
        print("    %s slot %s: (%.3f,%.3f) -> (%.3f,%.3f)  moved %.3f"
              % (r, slot, a[0], a[1], b[0], b[1], math.hypot(b[0]-a[0], b[1]-a[1])))
