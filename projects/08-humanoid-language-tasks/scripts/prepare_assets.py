"""Restore missing simulation resources from pinned upstream Git revisions.

No dependency on project 007. Existing files are verified, never overwritten.
The tracked baseline manifest remains authoritative; it is not regenerated.
"""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path, PurePosixPath
import subprocess

ROOT = Path(__file__).resolve().parents[1]
UPSTREAMS = {
    "engineai": ("https://github.com/engineai-robotics/engineai_robotics_native_sdk.git",
                 "335c60e88772c26c7852d0abd6b3c7439037dd8f",
                 ["assets/resource/robot/t800", "assets/resource/environment", "assets/config/t800"]),
    "menagerie": ("https://github.com/google-deepmind/mujoco_menagerie.git",
                  "8161bba264d7fa7c99ca301e91e7fb44737676ad", ["wonik_allegro"]),
}


def upstream_path(relative: str) -> tuple[str, str] | None:
    path = PurePosixPath(relative)
    if path.is_absolute() or ".." in path.parts or "\\" in relative:
        raise ValueError("Unsafe manifest path")
    if relative.startswith("assets/allegro/"):
        return "menagerie", "wonik_allegro/" + relative.removeprefix("assets/allegro/")
    if relative.startswith(("assets/t800/robot/", "assets/t800/environment/")):
        return "engineai", "assets/resource/" + relative.removeprefix("assets/t800/")
    if relative == "assets/t800/scene.xml":
        return "engineai", "assets/resource/t800.xml"
    if relative == "policies/t800/walking.mnn":
        return "engineai", "assets/config/t800/rl_walking_example/policy/t800_260618_165257_30000.mnn"
    if relative in ("assets/t800/LICENSE-EngineAI.txt", "policies/t800/LICENSE-EngineAI.txt"):
        return "engineai", "LICENSE.txt"
    return None  # project-specific XML, configs and licences must be tracked


def verified(data: bytes, expected: str, name: str) -> bytes:
    if hashlib.sha256(data).hexdigest() != expected:
        raise ValueError(f"SHA-256 mismatch: {name}; refusing to replace or bless it")
    return data


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--engineai-repo", type=Path, help="optional existing upstream Git cache (read-only)")
    parser.add_argument("--menagerie-repo", type=Path, help="optional existing upstream Git cache (read-only)")
    args = parser.parse_args()
    supplied = {"engineai": args.engineai_repo, "menagerie": args.menagerie_repo}
    repos: dict[str, Path] = {}
    manifest = json.loads((ROOT / "assets/baseline_manifest.json").read_text())
    checked = restored = 0
    for relative, entry in manifest["files"].items():
        mapping = upstream_path(relative)
        target = ROOT / relative
        if not target.resolve().is_relative_to(ROOT.resolve()):
            raise ValueError(f"Out-of-project asset: {relative}")
        if target.exists():
            verified(target.read_bytes(), entry["sha256"], relative)
            checked += 1
            continue
        if mapping is None:
            raise FileNotFoundError(f"Missing tracked project file: {relative}; restore it from Git")
        key, source_path = mapping
        url, revision, sparse = UPSTREAMS[key]
        if key not in repos:
            repo = supplied[key] or ROOT / ".cache" / key
            if supplied[key] is None and not repo.exists():
                repo.parent.mkdir(parents=True, exist_ok=True)
                subprocess.run(["git", "clone", "--filter=blob:none", "--no-checkout", url, str(repo)], check=True)
                subprocess.run(["git", "-C", str(repo), "sparse-checkout", "set", *sparse], check=True)
                subprocess.run(["git", "-C", str(repo), "checkout", "--detach", revision], check=True)
            subprocess.run(["git", "-C", str(repo), "cat-file", "-e", revision + "^{commit}"], check=True)
            repos[key] = repo
        data = subprocess.check_output(["git", "-C", str(repos[key]), "show", revision + ":" + source_path])
        verified(data, entry["sha256"], relative)
        target.parent.mkdir(parents=True, exist_ok=True)
        with target.open("xb") as stream:
            stream.write(data)
        restored += 1
    print(f"Assets verified: {checked} existing, {restored} restored; no manifest changes")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
