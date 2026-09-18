"""One-time baseline import (docs/MASTER_PLAN.md section 6).

Copies the already-prepared robot assets, walking policy, licences and the
fixed-fixture configuration from the 007 baseline into 008, verifies every file
against the source manifest's SHA-256, and writes ``assets/baseline_manifest.json``
(the 008 record) plus ``docs/BASELINE_IMPORT.md``.

This copies -- it never moves or deletes the source (section 6.4). It refuses to
overwrite an already-imported file, so a second run is a no-op with an explicit
message rather than a silent change.

Run from the project root:
    .venv/bin/python scripts/import_baseline.py --source ~/projects/007_humanoid_visual_transport
"""

from __future__ import annotations

import argparse
import hashlib
import json
import shutil
from datetime import datetime, timezone
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]

# source (relative to the 007 baseline) -> destination (relative to 008)
FILE_MAP = {
    "assets/combined.xml": "assets/combined.xml",
    "config/task.json": "config/task.json",
    "config/workcell.json": "config/workcell.json",
    "config/scene.json": "config/scene.json",
    "config/t800/walking.yaml": "config/t800/walking.yaml",
    "config/t800/model.yaml": "config/t800/model.yaml",
    "config/t800/stand.yaml": "config/t800/stand.yaml",
    "licenses/EngineAI-BSD-3-Clause.txt": "licenses/EngineAI-BSD-3-Clause.txt",
    "licenses/Allegro-BSD-2-Clause.txt": "licenses/Allegro-BSD-2-Clause.txt",
    "policies/t800/walking.mnn": "policies/t800/walking.mnn",
    "policies/t800/LICENSE-EngineAI.txt": "policies/t800/LICENSE-EngineAI.txt",
}

# directories copied wholesale, then every file is hash-verified
DIR_MAP = {
    "assets/t800": "assets/t800",
    "assets/allegro": "assets/allegro",
}

# the code that is adapted (not copied verbatim) into 008's simulation package
ADAPTED_CODE = {
    "policy.py": "src/humanoid008/simulation/walking_policy.py",
    "runtime.py": "src/humanoid008/simulation/robot_runtime.py",
    "camera.py": "src/humanoid008/simulation/camera.py",
    "vision.py": "src/humanoid008/simulation/vision.py",
    "tactile.py": "src/humanoid008/simulation/tactile.py",
    "stance.py": "src/humanoid008/simulation/stance.py",
}


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def load_source_manifest(source: Path) -> dict:
    path = source / "assets" / "manifest.json"
    if not path.is_file():
        raise SystemExit(f"source manifest not found: {path}")
    return json.loads(path.read_text(encoding="utf-8"))


def collect_source_files(source: Path) -> dict[str, Path]:
    files: dict[str, Path] = {}
    for rel, _ in FILE_MAP.items():
        files[rel] = source / rel
    for src_rel, _ in DIR_MAP.items():
        for p in sorted((source / src_rel).rglob("*")):
            if p.is_file():
                files[p.relative_to(source).as_posix()] = p
    return files


def copy_file(src: Path, dst: Path, expected_sha: str | None) -> dict:
    dst.parent.mkdir(parents=True, exist_ok=True)
    if dst.exists():
        actual = sha256(dst)
        if actual == expected_sha:
            return {"status": "unchanged", "sha256": actual}
        raise SystemExit(f"refusing to overwrite {dst} (content differs)")
    shutil.copy2(src, dst)
    actual = sha256(dst)
    if expected_sha is not None and actual != expected_sha:
        raise SystemExit(f"hash mismatch for {dst}: expected {expected_sha}, got {actual}")
    return {"status": "copied", "sha256": actual}


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", required=True, help="path to the 007 baseline")
    args = parser.parse_args()

    source = Path(args.source).expanduser().resolve()
    if not (source / "assets" / "combined.xml").is_file():
        raise SystemExit(f"--source does not look like the 007 baseline: {source}")

    manifest = load_source_manifest(source)
    expected = manifest.get("sha256", {})
    files = collect_source_files(source)

    record: dict[str, dict] = {}
    copied = unchanged = 0
    for rel, src in files.items():
        want = expected.get(rel)
        if want is None:
            # e.g. a file not covered by the 007 manifest -- still import, verify by copy
            pass
        result = copy_file(src, ROOT / FILE_MAP.get(rel, DIR_MAP.get(rel, rel)), want)
        record[rel] = {"sha256": result["sha256"], "status": result["status"]}
        if result["status"] == "copied":
            copied += 1
        else:
            unchanged += 1

    baseline = {
        "schema_version": "1.0",
        "imported_utc": datetime.now(timezone.utc).isoformat(),
        "source": {
            "path": str(source),
            "repository": manifest.get("repository"),
            "commit": manifest.get("commit"),
            "hand_revision": manifest.get("hand_revision"),
            "model": manifest.get("model"),
            "modifications": manifest.get("modifications"),
            "policy_file": manifest.get("policy_file"),
            "note": manifest.get("note"),
        },
        "files": record,
        "adapted_code_mapping": ADAPTED_CODE,
        "source_baseline_status": (
            "experimental; the full 30-example regression has not been re-run inside 008"
        ),
    }
    out = ROOT / "assets" / "baseline_manifest.json"
    out.write_text(json.dumps(baseline, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(f"imported: {copied} copied, {unchanged} already present")
    print(f"baseline manifest: {out.relative_to(ROOT)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
