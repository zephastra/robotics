import json, math, glob, os, yaml, pathlib

spawns = {r: (v["x"], v["y"]) for r, v in
          yaml.safe_load(pathlib.Path("config/spawns.yaml").read_text())["spawns"].items()}

# F09 runners are r01 and r02. Which truth slots carried a MOVING robot?
p = "reports/batch_20260918T110845Z/F09/samples-F09.jsonl"
first, last = {}, {}
rows = 0
widths = {}
for line in open(p):
    line = line.strip()
    if not line:
        continue
    rows += 1
    try:
        d = json.loads(line)
    except Exception:
        continue
    t = d.get("truth") or {}
    widths[len(t)] = widths.get(len(t), 0) + 1
    for k, v in t.items():
        ks = str(k)
        if ks not in first:
            first[ks] = v
        last[ks] = v

print("F09 (runners declared: r01, r02)")
print("rows=%d  truth widths=%s" % (rows, widths))
print()
print("%-5s %-24s %-24s %-9s %s" % ("slot", "first", "last", "moved", "which spawn it started on"))
for k in sorted(first, key=int):
    a, b = first[k], last[k]
    mv = math.hypot(b[0]-a[0], b[1]-a[1])
    best = min(spawns, key=lambda r: math.hypot(a[0]-spawns[r][0], a[1]-spawns[r][1]))
    d = math.hypot(a[0]-spawns[best][0], a[1]-spawns[best][1])
    tag = ""
    if mv > 0.05:
        tag = "   <== MOVED"
    print("%-5s (%+8.3f,%+8.3f)         (%+8.3f,%+8.3f)         %8.3f  %s (%.3f m)%s"
          % (k, a[0], a[1], b[0], b[1], mv, best, d, tag))

print()
print("=== the judge's own mapping for F09 ===")
j = json.load(open("reports/batch_20260918T110845Z/F09/judge-F09.json"))
print("robots_judged =", j.get("robots_judged"))
print("robots_absent =", j.get("robots_absent"))
for c in j.get("checks", []):
    if c["check"] in ("truth_attribution", "driver_claims_corroborated_by_truth"):
        print("%s: %s" % (c["check"], c["verdict"]))
        print("   ", c.get("detail"))
