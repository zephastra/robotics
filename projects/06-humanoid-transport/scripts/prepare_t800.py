"""Prepare the BSD-licensed official T800 assets, never the competition bundle."""
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
REV = '335c60e88772c26c7852d0abd6b3c7439037dd8f'
POLICY_SHA = 'cbcb90f86dbb2fde39bdc5a25c8d0530d5c79c7a8f84b1f90863d8c9065b6427'
SOURCE = ROOT / '.cache/engineai_native_sdk'


def main():
    actual = subprocess.check_output(['git', '-C', str(SOURCE), 'rev-parse', 'HEAD'], text=True).strip()
    if actual != REV:
        raise SystemExit('T800 upstream revision mismatch')
    paths = ['assets/resource/robot/t800', 'assets/resource/environment/ground.xml',
             'assets/resource/t800.xml', 'assets/config/t800', 'LICENSE.txt']
    subprocess.run(['git', '-C', str(SOURCE), 'diff', '--exit-code', 'HEAD', '--', *paths], check=True)
    weight = SOURCE/'assets/config/t800/rl_walking_example/policy/t800_260618_165257_30000.mnn'
    if hashlib.sha256(weight.read_bytes()).hexdigest() != POLICY_SHA:
        raise SystemExit('T800 pretrained policy checksum mismatch')
    dest = ROOT/'assets/t800'
    files = []

    def copy(source, target):
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(source, target)
        files.append(target)

    resources = SOURCE/'assets/resource'
    robot = resources/'robot/t800'
    for xml in sorted((robot/'xml').glob('*.xml')):
        copy(xml, dest/xml.relative_to(resources))
    # Only the meshes/textures actually referenced by MJCF, not duplicate DAE/URDF assets.
    for node in ET.parse(robot/'xml/assets.xml').getroot():
        if 'file' in node.attrib:
            source = (robot/'xml'/node.attrib['file']).resolve()
            copy(source, dest/source.relative_to(resources))
    copy(resources/'environment/ground.xml', dest/'environment/ground.xml')
    copy(resources/'t800.xml', dest/'scene.xml')
    copy(weight, ROOT/'policies/t800/walking.mnn')
    for name, relative in [('walking', 'rl_walking_example'), ('stand', 'pd_stand'), ('model', 'model')]:
        copy(SOURCE/f'assets/config/t800/{relative}/default.yaml', ROOT/f'config/t800/{name}.yaml')
    for target in [dest/'LICENSE-EngineAI.txt', ROOT/'policies/t800/LICENSE-EngineAI.txt',
                   ROOT/'licenses/EngineAI-BSD-3-Clause.txt']:
        copy(SOURCE/'LICENSE.txt', target)
    tree = ET.ElementTree(ET.Element('mujoco', model='006_t800_baseline'))
    root = tree.getroot()
    ET.SubElement(root, 'include', file='scene.xml')
    world = ET.SubElement(root, 'worldbody')
    for name, x, y, rgba in [('A', 3, 0, '.1 .8 .4 .6'), ('B', 3, 2, '.2 .6 1 .6'), ('HOME', 0, 0, '.9 .3 .7 .6')]:
        ET.SubElement(world, 'geom', name='target_'+name, type='cylinder', size='.3 .002',
                      pos=f'{x} {y} .003', rgba=rgba, contype='0', conaffinity='0')
    tree.write(dest/'lab.xml', encoding='utf-8', xml_declaration=True)
    files.append(dest/'lab.xml')
    manifest = dict(repository='https://github.com/engineai-robotics/engineai_robotics_native_sdk',
                    commit=REV, policy_file='policies/t800/walking.mnn',
                    note='Official SDK T800 serial 25-joint model; 22-action pretrained walking policy. No competition assets.',
                    sha256={p.relative_to(ROOT).as_posix(): hashlib.sha256(p.read_bytes()).hexdigest() for p in sorted(set(files))})
    (ROOT/'assets/manifest-t800.json').write_text(json.dumps(manifest, indent=2)+'\n')
    print(f'Prepared T800: {len(manifest["sha256"])} verified resources at {REV}')


if __name__ == '__main__':
    main()
