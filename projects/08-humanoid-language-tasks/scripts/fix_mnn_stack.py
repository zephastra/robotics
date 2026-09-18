"""Remove the MNN wheel's executable-stack request, locally and reversibly.

This removes a permission; it does not enable executable stacks or change glibc.
Only MNN extensions inside this project's venv are eligible. Never edit the uv
cache. (Adapted from the 007 baseline; the original keeps a backup + a JSON
record of every patch.)
"""

from __future__ import annotations

import hashlib
import json
import shutil
import subprocess
import sysconfig
import tempfile
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def main() -> int:
    site = Path(sysconfig.get_paths()["platlib"]).resolve()
    venv = (ROOT / ".venv").resolve()
    if not site.is_relative_to(venv):
        raise SystemExit("Refusing to modify a Python environment outside this project's .venv")

    patcher = venv / "bin" / "patchelf"
    if not patcher.exists():
        raise SystemExit("patchelf not found; run: uv pip install patchelf==0.19.1.0")

    backup = ROOT / ".cache" / "mnn-stack-backup"
    patched = 0
    for source in sorted(site.glob("_mnn*.so")):
        state = subprocess.check_output(
            [str(patcher), "--print-execstack", str(source)], text=True
        )
        if "execstack: X" not in state:
            continue
        before = hashlib.sha256(source.read_bytes()).hexdigest()
        backup.mkdir(parents=True, exist_ok=True)
        original = backup / (before + ".so")
        if not original.exists():
            shutil.copy2(source, original)
        with tempfile.TemporaryDirectory(dir=site, prefix="mnn-noexec-") as folder:
            target = Path(folder) / source.name
            shutil.copy2(source, target)
            subprocess.run([str(patcher), "--clear-execstack", str(target)], check=True)
            after = hashlib.sha256(target.read_bytes()).hexdigest()
            target.replace(source)
        record = dict(
            file=str(source.relative_to(ROOT)),
            before_sha256=before,
            after_sha256=after,
            operation="patchelf --clear-execstack",
            backup=str(original.relative_to(ROOT)),
        )
        (backup / (before + ".json")).write_text(
            json.dumps(record, indent=2) + "\n", encoding="utf-8"
        )
        patched += 1
        print("Removed executable-stack request:", source.name)

    print(f"execstack fix complete ({patched} file(s) patched)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
