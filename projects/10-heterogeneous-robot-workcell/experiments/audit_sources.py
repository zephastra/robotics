"""Record independent copied source/asset provenance, not runtime imports."""
import hashlib
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
SOURCE = Path('/home/ziling/projects/007_humanoid_visual_transport')


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


records = []
for folder in ['assets', 'config/t800', 'policies', 'licenses', 'src/humanoid007']:
    for target in sorted((ROOT/folder).rglob('*')):
        if not target.is_file() or '__pycache__' in target.parts:
            continue
        relative = target.relative_to(ROOT)
        original = SOURCE/relative
        if relative.as_posix() == 'licenses/Integration-Apache-2.0.txt':
            original = SOURCE/'LICENSE'
        records.append(dict(path=relative.as_posix(), sha256=digest(target),
                            source=str(original), source_sha256=digest(original),
                            identical=digest(target)==digest(original)))
assert all(row['identical'] for row in records)
(ROOT/'docs/P1_SOURCE_MANIFEST.json').write_text(json.dumps(records, indent=2)+'\n')
print(f'Verified {len(records)} independent copied files')
