import json, math, glob, os

MAPPING = {"0": "r01", "1": "r03", "2": "r02"}


def analyse(path):
    first, last = {}, {}
    n = truth_rows = 0
    for line in open(path):
        line = line.strip()
        if not line:
            continue
        n += 1
        try:
            d = json.loads(line)
        except Exception:
            continue
        t = d.get("truth") or {}
        if not t:
            continue
        truth_rows += 1
        for k, v in t.items():
            ks = str(k)
            if ks not in first:
                first[ks] = v
            last[ks] = v
    out = []
    for r, slot in sorted(MAPPING.items()):
        if slot in first and slot in last:
            a, b = first[slot], last[slot]
            out.append("%s=%.3f" % (r, math.hypot(b[0] - a[0], b[1] - a[1])))
    return n, truth_rows, out


PATS = ("reports/batch_20260918T103152Z/N07*/samples-*.jsonl",
        "reports/batch_20260918T105936Z/*/samples-*.jsonl",
        "reports/batch_20260918T110845Z/*/samples-*.jsonl",
        "reports/batch_20260918T113422Z/*/samples-*.jsonl")

print("%-14s %-8s %-9s %s" % ("case", "samples", "truthrow", "truth net displacement (m)"))
for pat in PATS:
    for p in sorted(glob.glob(pat)):
        lab = os.path.basename(os.path.dirname(p))
        try:
            n, tr, out = analyse(p)
        except Exception as e:
            print("%-14s ERROR %r" % (lab, e))
            continue
        print("%-14s %-8d %-9d %s" % (lab, n, tr, "  ".join(out) or "(none)"))
