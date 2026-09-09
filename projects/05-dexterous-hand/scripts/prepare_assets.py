"""Build an explicitly instrumented hand fixture from pinned, licensed MJCF."""
from pathlib import Path
import hashlib
import json
import shutil
import subprocess
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
REV = '8161bba264d7fa7c99ca301e91e7fb44737676ad'


def main():
    source = ROOT / '.cache/menagerie'
    assert subprocess.check_output(['git', '-C', str(source), 'rev-parse', 'HEAD'], text=True).strip() == REV
    subprocess.run(['git', '-C', str(source), 'diff', '--exit-code', 'HEAD', '--', 'wonik_allegro'], check=True)
    upstream = source / 'wonik_allegro'
    target = ROOT / 'assets/allegro'
    target.mkdir(parents=True, exist_ok=True)
    shutil.copytree(upstream / 'assets', target / 'assets', dirs_exist_ok=True)
    for name in ['right_hand.xml', 'LICENSE', 'README.md']:
        shutil.copy2(upstream / name, target / name)
    (ROOT / 'licenses').mkdir(exist_ok=True)
    shutil.copy2(upstream / 'LICENSE', ROOT / 'licenses/Allegro-BSD-2-Clause.txt')
    tree = ET.parse(target / 'right_hand.xml')
    root = tree.getroot()
    root.set('model', '005 Allegro Hand - Actuated Wrist Test Fixture')
    root.find('compiler').set('meshdir', 'allegro/assets')
    option = root.find('option')
    option.set('timestep', '0.002')
    option.set('integrator', 'implicitfast')
    world = root.find('worldbody')
    palm = world.find('body')
    world.remove(palm)
    stage = ET.SubElement(world, 'body', name='wrist_fixture', pos='0 0 0.22')
    palm.set('quat', '0 1 0 0')
    for axis, direction in [('y', '0 1 0'), ('z', '0 0 1')]:
        ET.SubElement(stage, 'joint', name='stage_' + axis, type='slide', axis=direction,
                      range='-0.02 0.3', damping='30', armature='.1')
    ET.SubElement(stage, 'geom', type='box', size='.025 .055 .018', pos='-.075 0 -.05',
                  rgba='.3 .35 .4 1', mass='1', contype='0', conaffinity='0')
    stage.append(palm)
    for prefix in ['ff', 'mf', 'rf', 'th']:
        body = palm.find(f".//body[@name='{prefix}_tip']")
        ET.SubElement(body, 'site', name=prefix + '_tip_site', pos='0 0 ' + ('.035' if prefix == 'th' else '.019'),
                      size='.003', rgba='1 .4 .1 1')
    actuators = root.find('actuator')
    for axis in ['y', 'z']:
        ET.SubElement(actuators, 'position', name='stage_' + axis, joint='stage_' + axis,
                      kp='2000', kv='100', ctrlrange='-0.02 0.3', forcerange='-100 100')
    ET.SubElement(world, 'light', pos='0 -.5 1.5', dir='0 .3 -1', directional='true')
    ET.SubElement(world, 'light', pos='.5 .5 1', dir='-.2 -.2 -1')
    ET.SubElement(world, 'geom', name='floor', type='plane', size='1 1 .02', pos='0 0 -.02', rgba='.16 .22 .28 1')
    ET.SubElement(world, 'geom', name='source_pedestal', type='cylinder', size='.018 .085',
                  pos='.06 0 .085', rgba='.25 .45 .6 1')
    ET.SubElement(world, 'geom', name='target_tray', type='box', size='.075 .075 .01',
                  pos='.06 .20 .16', rgba='.2 .65 .35 1', condim='6', friction='1 .005 .001')
    for axis in [0, 1]:
        for sign in [-1, 1]:
            pos = [.06, .20, .185]
            pos[axis] += sign * .078
            size = [.075, .075, .015]
            size[axis] = .003
            ET.SubElement(world, 'geom', name=f'tray_wall_{axis}_{sign}', type='box',
                          size=' '.join(map(str, size)), pos=' '.join(map(str, pos)), rgba='.12 .4 .24 1')
    obj = ET.SubElement(world, 'body', name='object', pos='.06 0 .205')
    ET.SubElement(obj, 'freejoint', name='object_free')
    ET.SubElement(obj, 'geom', name='object_geom', type='sphere', size='.035', mass='.05',
                  rgba='1 .55 .12 1', friction='1 .005 .0001', condim='6')
    visual = ET.SubElement(root, 'visual')
    ET.SubElement(visual, 'global', offwidth='960', offheight='720')
    ET.indent(tree)
    scene = ROOT / 'assets/scene.xml'
    tree.write(scene, encoding='unicode')
    files = [scene, *sorted(target.rglob('*'))]
    manifest = {'source': 'https://github.com/google-deepmind/mujoco_menagerie', 'revision': REV,
                'model': 'wonik_allegro/right_hand.xml',
                'changes': ['Actuated 2-axis wrist fixture', 'Fingertip sites', 'Object, pedestal and tray',
                            'Vertical palm mounting; original finger dynamics and actuators retained',
                            '2ms implicitfast integration',
                            'Sphere 50g radius 35mm, 6D contact; tray rolling friction .001 m (nominal receiving pad)'],
                'sha256': {str(p.relative_to(ROOT)): hashlib.sha256(p.read_bytes()).hexdigest() for p in files if p.is_file()}}
    (ROOT / 'assets/manifest.json').write_text(json.dumps(manifest, indent=2) + '\n')
    print('Prepared pinned Allegro hand and test fixture.')


if __name__ == '__main__':
    main()
