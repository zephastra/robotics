"""Compare the judge's OWN slot mapping against each case's declared runners.

The point: N07's contradicted claims carried truth_start == truth_end == a spawn pose, with
``truth_travelled_m = 0.0``. That can mean two very different things:

  (a) ``_truth_slots`` (used by the claim check) picked the wrong slot, so it compared the
      robot against a bystander, or
  (b) the robot really did not move.

This script decides which, by using the judge's own mapping and the judge's own truth tracks.
"""
import json, math, glob, os, yaml, pathlib

spawns = {r: (v["x"], v["y"]) for r, v in
          yaml.safe_load(pathlib.Path("config/spawns.yaml").read_text())["spawns"].items()}

CASE_RUNNERS = {}
src = pathlib.Path("config/scenarios/regression_v1.yaml").read_text()
cur = None
for line in src.splitlines():
    if line.startswith("  ") and not line.startswith("   ") and line.rstrip().endswith(":"):
        cur = line.strip().rstrip(":")
    if "runners:" in line and cur:
        inside = line.split("runners:")[1].strip().strip("[]")
        CASE_RUNNERS[cur] = [x.strip() for x in inside.split(",") if x.strip()]
print("declared runners:", {k: v for k, v in list(CASE_RUNNERS.items()) if k.startswith(("F", "N"))})
print()


def tracks(path):
    first, last = {}, {}
    for line in open(path):
        line = line.strip()
        if not line:
            continue
        try:
            d = json.loads(line)
        except Exception:
            continue
        t = d.get("truth") or {}
        for k, v in t.items():
            ks = str(k)
            if ks not in first:
                first[ks] = v
            last[ks] = v
    return first, last


def check(judge_path):
    d = json.load(open(judge_path))
    case_dir = os.path.dirname(judge_path)
    samples = glob.glob(os.path.join(case_dir, "samples-*.jsonl"))
    if not samples:
        return None
    first, last = tracks(samples[0])
    att = next((c for c in d.get("checks", []) if c["check"] == "truth_attribution"), None)
    mapping = None
    if att and isinstance(att.get("mapping"), dict):
        mapping = {str(k): v for k, v in att["mapping"].items()}
    if mapping is None:
        # fall back: parse "slots {'0': 'r01', ...}" out of the detail
        import re
        m = re.search(r"slots \{([^}]*)\}", att.get("detail", "") if att else "")
        if m:
            mapping = {}
            for pair in m.group(1).split(","):
                if ":" in pair:
                    k, v = pair.split(":")
                    mapping[k.strip().strip("'\"")] = v.strip().strip("'\"")
    return d, first, last, mapping


for jp in sorted(glob.glob("reports/batch_20260918T10*/N07*/judge-*.json") +
                 glob.glob("reports/batch_20260918T11*/*/judge-*.json")):
    r = check(jp)
    if not r:
        continue
    d, first, last, mapping = r
    lab = os.path.basename(os.path.dirname(jp))
    print("=== %-12s judged=%s absent=%s declared=%s"
          % (lab, d.get("robots_judged"), d.get("robots_absent"),
             CASE_RUNNERS.get(lab.split("_")[0], "?")))
    if not mapping:
        print("    (no mapping recorded)")
        continue
    for slot, robot in sorted(mapping.items()):
        if slot not in first:
            continue
        a, b = first[slot], last[slot]
        mv = math.hypot(b[0] - a[0], b[1] - a[1])
        print("    slot %-3s = %-4s  (%.3f,%.3f) -> (%.3f,%.3f)  moved %.3f m"
              % (slot, robot, a[0], a[1], b[0], b[1], mv))
    print()
