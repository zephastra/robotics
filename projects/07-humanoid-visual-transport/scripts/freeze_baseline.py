"""Save a self-contained source/assets baseline (not the nonportable venv)."""
from datetime import datetime, timezone
from pathlib import Path
import hashlib
import json
import tarfile

ROOT = Path(__file__).resolve().parents[1]
EXCLUDED = {'.venv', '.cache', '.git', '__pycache__', '.pytest_cache', 'reports'}


def main():
    stamp = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S%fZ')
    destination = ROOT.parent / 'releases' / f'007-v1.0-{stamp}'
    destination.mkdir(parents=True, exist_ok=False)
    paths = sorted(p for p in ROOT.rglob('*') if p.is_file()
                   and not EXCLUDED.intersection(p.relative_to(ROOT).parts))
    hashes = {p.relative_to(ROOT).as_posix(): hashlib.sha256(p.read_bytes()).hexdigest()
              for p in paths}
    archive = destination / 'baseline.tar.gz'
    with tarfile.open(archive, 'x:gz') as tar:
        for p in paths:
            tar.add(p, arcname=p.relative_to(ROOT), recursive=False)
    # Verify the archived bytes, not merely the inputs.
    with tarfile.open(archive, 'r:gz') as tar:
        for name, digest in hashes.items():
            assert hashlib.sha256(tar.extractfile(name).read()).hexdigest() == digest, name
    (destination / 'SHA256.json').write_text(json.dumps({
        'archive_sha256': hashlib.sha256(archive.read_bytes()).hexdigest(),
        'files': hashes}, indent=2) + '\n')
    (destination / 'RESTORE.md').write_text(
        '# 007 v1.0 baseline\n\n'
        'Contains source, configuration, model assets, policy weights, licenses and selected evidence.\n'
        'Excludes virtual environments, download caches and transient reports.\n'
        'Extract baseline.tar.gz into a NEW empty directory; do not overwrite ongoing work.\n'
        'Recreate the environment with bash scripts/setup.sh; compare generated assets with SHA256.json.\n'
        'Do not claim fresh-machine reproduction until it has actually been tested.\n')
    print(destination)
    print(f'Archive verified: {len(hashes)} files, {archive.stat().st_size} bytes')


if __name__ == '__main__':
    main()
