"""Copy pinned third-party assets; generate only a non-colliding lab overlay."""
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
REV = '276801e46c5d433564f24658bac64f254b7d2d4b'
upstream = ROOT / '.cache/unitree_rl_gym'
actual = subprocess.check_output(['git', '-C', str(upstream), 'rev-parse', 'HEAD'], text=True).strip()
if actual != REV:
    raise SystemExit('Unexpected upstream revision; refusing to mix model and policy versions')
subprocess.run(['git','-C',str(upstream),'diff','--exit-code','HEAD','--','resources/robots/g1_description',
                'deploy/pre_train/g1','deploy/deploy_mujoco/configs/g1.yaml','LICENSE'],check=True,stdout=subprocess.DEVNULL)
expected_policy='cf668f75b90d1abf73d2b87612a6e76bccc61ff7e083b63582d3f6aaa3c1759d'
if hashlib.sha256((upstream/'deploy/pre_train/g1/motion.pt').read_bytes()).hexdigest()!=expected_policy:
    raise SystemExit('Pretrained policy checksum mismatch')
source = upstream / 'resources/robots/g1_description'
dest = ROOT / 'assets/g1'
dest.mkdir(parents=True, exist_ok=True)
# Copy precisely the model and the meshes referenced by that model.
shutil.copy2(source / 'g1_12dof.xml', dest / 'g1_12dof.xml')
(dest / 'meshes').mkdir(exist_ok=True)
for node in ET.parse(source / 'g1_12dof.xml').findall('.//asset/mesh'):
    mesh = node.attrib['file']
    shutil.copy2(source / 'meshes' / mesh, dest / 'meshes' / mesh)
shutil.copy2(source / 'scene.xml', dest / 'scene.xml')
shutil.copy2(upstream / 'LICENSE', dest / 'LICENSE-Unitree.txt')
(ROOT / 'policies').mkdir(exist_ok=True)
shutil.copy2(upstream / 'deploy/pre_train/g1/motion.pt', ROOT / 'policies/motion.pt')
shutil.copy2(upstream / 'LICENSE', ROOT / 'policies/LICENSE-Unitree.txt')
(ROOT / 'config').mkdir(exist_ok=True)
shutil.copy2(upstream / 'deploy/deploy_mujoco/configs/g1.yaml', ROOT / 'config/upstream_g1.yaml')
tree = ET.parse(dest / 'scene.xml')
world = tree.getroot().find('worldbody')
for name, x, y, rgba in [('A', 3, 0, '0.1 0.8 0.4 0.6'), ('B', 3, 2, '0.2 0.6 1 0.6'), ('HOME', 0, 0, '0.9 0.3 0.7 0.6')]:
    ET.SubElement(world, 'geom', name='target_' + name, type='cylinder', size='0.3 0.002', pos=f'{x} {y} 0.003', rgba=rgba, contype='0', conaffinity='0')
tree.write(dest / 'lab.xml', encoding='utf-8', xml_declaration=True)
files = sorted(p for p in dest.rglob('*') if p.is_file())
files += [ROOT/'policies/motion.pt', ROOT/'policies/LICENSE-Unitree.txt']
files.append(ROOT / 'config/upstream_g1.yaml')
manifest = {'repository': 'https://github.com/unitreerobotics/unitree_rl_gym', 'commit': REV,
            'note': 'Model and pretrained weights are upstream assets, not trained in project 004.',
            'sha256': {str(p.relative_to(ROOT)): hashlib.sha256(p.read_bytes()).hexdigest() for p in files}}
(ROOT / 'assets/manifest.json').write_text(json.dumps(manifest, indent=2) + '\n')
print(f'Prepared {len(files)} assets from pinned upstream revision {REV}')
