"""Remove executable-stack request only in 010's copied dependency installation."""
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import sysconfig

ROOT = Path(__file__).resolve().parents[1]
site = Path(sysconfig.get_paths()['platlib']).resolve()
assert site.is_relative_to((ROOT/'.venv').resolve())
backup = ROOT/'runtime/mnn_original'
backup.mkdir(parents=True, exist_ok=True)
records = []
for source in sorted(site.glob('_mnn*.so')):
    tool = str(ROOT/'.venv/bin/patchelf')
    before = hashlib.sha256(source.read_bytes()).hexdigest()
    state = subprocess.check_output([tool, '--print-execstack', str(source)], text=True)
    if 'execstack: X' not in state:
        continue
    target = backup/(before+'.so')
    if not target.exists():
        shutil.copy2(source, target)
    subprocess.run([tool, '--clear-execstack', str(source)], check=True)
    records.append(dict(file=str(source.relative_to(ROOT)), original_sha256=before,
                        patched_sha256=hashlib.sha256(source.read_bytes()).hexdigest()))
(backup/'patch.json').write_text(json.dumps(records, indent=2)+'\n')
print(json.dumps(records, indent=2))
