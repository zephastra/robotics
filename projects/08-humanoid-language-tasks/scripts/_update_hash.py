"""One-off: re-hash assets/combined.xml into baseline_manifest.json (section 6.2)."""
import hashlib
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
manifest_path = ROOT / "assets" / "baseline_manifest.json"
manifest = json.loads(manifest_path.read_text(encoding="utf-8"))

changed = 0
for name, entry in manifest.get("files", {}).items():
    path = ROOT / name
    if not path.is_file():
        continue
    actual = hashlib.sha256(path.read_bytes()).hexdigest()
    if entry.get("sha256") != actual:
        entry["sha256"] = actual
        changed += 1
        print(f"updated {name} -> {actual}")

manifest_path.write_text(json.dumps(manifest, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
print(f"done, {changed} hash(es) updated")
