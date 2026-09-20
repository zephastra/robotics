import json, math, glob, os, yaml, pathlib

root = pathlib.Path(".")
spawns = {r: (v["x"], v["y"]) for r, v in
          yaml.safe_load((root / "config/spawns.yaml").read_text())["spawns"].items()}
print("configured spawns:", {k: (round(a, 2), round(b, 2)) for k, (a, b) in spawns.items()})
print()

# Which robots does each case declare? Read the case manifests.
cases = {}
for p in glob.glob("config/cases/*.yaml") + glob.glob("config/cases/*.json"):
    cases[os.path.basename(p)] = p
print("case definition files:", sorted(cases)[:8])

# Pull the robots each case used from the runner's own manifest if present
for b in ("reports/batch_20260918T105936Z", "reports/batch_20260918T110845Z"):
    m = os.path.join(b, "manifest.json")
    if os.path.exists(m):
        man = json.load(open(m))
        print()
        print("=== %s manifest ===" % b)
        entries = man if isinstance(man, list) else man.get("cases") or man.get("runs") or []
        for e in entries[:14]:
            if isinstance(e, dict):
                print("  %-6s robots=%s seeds=%s" % (e.get("case"), e.get("robots"),
                                                     e.get("seeds")))
