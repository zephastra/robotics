"""Remove the MNN wheel's executable-stack request, locally and reversibly.

This removes a permission; it does not enable executable stacks or change glibc.
Only MNN extensions inside this project's venv are eligible. Never edit uv cache.
"""
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import sysconfig
import tempfile

ROOT = Path(__file__).resolve().parents[1]


def main():
    site = Path(sysconfig.get_paths()['platlib']).resolve()
    if not site.is_relative_to((ROOT/'.venv').resolve()):
        raise SystemExit('Refusing to modify a Python environment outside project 004')
    patcher = ROOT/'.venv/bin/patchelf'
    backup = ROOT/'.cache/mnn-stack-backup'
    for source in sorted(site.glob('_mnn*.so')):
        state = subprocess.check_output([str(patcher), '--print-execstack', str(source)], text=True)
        if 'execstack: X' not in state:
            continue
        before = hashlib.sha256(source.read_bytes()).hexdigest()
        backup.mkdir(parents=True, exist_ok=True)
        original = backup/(before+'.so')
        if not original.exists():
            shutil.copy2(source, original)
        # Patch a separate file, then replace; uv's cache may use hard links.
        with tempfile.TemporaryDirectory(dir=site, prefix='mnn-noexec-') as folder:
            target = Path(folder)/source.name
            shutil.copy2(source, target)
            subprocess.run([str(patcher), '--clear-execstack', str(target)], check=True)
            after = hashlib.sha256(target.read_bytes()).hexdigest()
            target.replace(source)
        record = dict(file=str(source.relative_to(ROOT)), before_sha256=before,
                      after_sha256=after, operation='patchelf --clear-execstack',
                      backup=str(original.relative_to(ROOT)))
        (backup/(before+'.json')).write_text(json.dumps(record, indent=2)+'\n')
        print('Removed executable-stack request:', source.name)


if __name__ == '__main__':
    main()
