"""Assemble an explicitly experimental T800 + Allegro right-hand configuration."""
from pathlib import Path
import copy
import hashlib
import json
import math
import shutil
import subprocess
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
ASSETS = ROOT/'assets'
HAND_REV = '8161bba264d7fa7c99ca301e91e7fb44737676ad'


def expanded(path):
    root = ET.parse(path).getroot()
    def visit(parent):
        for child in list(parent):
            if child.tag == 'include':
                children = list(expanded(path.parent / child.get('file')))
                index = list(parent).index(child)
                parent.remove(child)
                for offset, node in enumerate(children):
                    parent.insert(index+offset, node)
            else:
                if child.tag in ('mesh', 'texture', 'hfield') and child.get('file'):
                    child.set('file', (path.parent/child.get('file')).resolve().relative_to(ASSETS).as_posix())
                visit(child)
    visit(root)
    return root


def main():
    upstream = ROOT/'.cache/menagerie'
    assert subprocess.check_output(['git', '-C', str(upstream), 'rev-parse', 'HEAD'], text=True).strip() == HAND_REV
    subprocess.run(['git', '-C', str(upstream), 'diff', '--exit-code', 'HEAD', '--', 'wonik_allegro'], check=True)
    dest = ASSETS/'allegro'
    shutil.copytree(upstream/'wonik_allegro', dest, dirs_exist_ok=True)
    shutil.copy2(dest/'LICENSE', ROOT/'licenses/Allegro-BSD-2-Clause.txt')
    root = expanded(ASSETS/'t800/scene.xml')
    root.set('model', '006 Experimental T800 + Allegro RH')
    hand = ET.parse(dest/'right_hand.xml').getroot()
    mission = json.loads((ROOT/'config/mission.json').read_text())
    for mesh in hand.findall('./asset/mesh'):
        if mesh.get('name') is None:
            mesh.set('name', Path(mesh.get('file')).stem)
        mesh.set('file', 'allegro/assets/'+mesh.get('file'))
    for node in hand.iter():
        for key in ('name', 'class', 'childclass', 'mesh', 'material', 'joint', 'body1', 'body2', 'site'):
            if key in node.attrib:
                node.set(key, 'rh_'+node.get(key))
    # Merge hand defaults and assets with prefixed identifiers, preserving parameters.
    for tag in ('default', 'asset', 'actuator', 'contact'):
        section = root.find(tag)
        if section is None:
            section = ET.SubElement(root, tag)
        other = hand.find(tag)
        if other is not None:
            section.extend(list(other))
    wrist = root.find(".//body[@name='LINK_WRIST_END_R']")
    if wrist is None:
        raise RuntimeError('Missing attachment body')
    for geom in list(wrist.findall('geom')):
        wrist.remove(geom)  # Replace only the placeholder wrist-end sphere.
    palm = hand.find('./worldbody/body')
    palm.set('pos', '0 0 -.11')
    angle = mission['mount_pitch']
    palm.set('quat', f'0 {math.cos(angle/2)} 0 {-math.sin(angle/2)}')
    wrist.append(palm)
    ET.SubElement(palm, 'site', name='grasp_center', pos='.06 0 .015', size='.006', rgba='1 .5 .1 1')
    for prefix in ('ff', 'mf', 'rf', 'th'):
        body = palm.find(f".//body[@name='rh_{prefix}_tip']")
        ET.SubElement(body, 'site', name=prefix+'_tip', pos='0 0 '+('.035' if prefix=='th' else '.019'), size='.003')
    # Hand is a rigidly mounted articulated mechanism; no fixture or support constraint.
    world = root.find('worldbody')
    mission = json.loads((ROOT/'config/mission.json').read_text())
    sx, sy, sz = mission['source']
    tx, ty, tz = mission['destination']
    ET.SubElement(world, 'geom', name='destination', type='cylinder', pos='1 0 .003', size='.18 .003',
                  rgba='.1 .8 .3 .5', contype='0', conaffinity='0')
    ET.SubElement(world, 'geom', name='source_support', type='box', pos=f'{sx} {sy-.1} {sz-.039}', size='.015 .115 .004',
                  rgba='.25 .45 .6 1')
    ET.SubElement(world, 'geom', name='source_leg', type='cylinder', pos=f'{sx} {sy-.2} {(sz-.043)/2}', size=f'.015 {(sz-.043)/2}',
                  rgba='.25 .45 .6 1')
    ET.SubElement(world, 'geom', name='receiving_pad', type='box', pos=f'{tx} {ty} {tz-.045}', size='.11 .11 .01',
                  rgba='.15 .65 .3 1', condim='6', friction='1 .005 .001')
    ET.SubElement(world, 'geom', name='receiving_leg', type='cylinder', pos=f'{tx} {ty} {(tz-.055)/2}', size=f'.025 {(tz-.055)/2}',
                  rgba='.2 .3 .3 1')
    # A receiving tray for the spherical payload, with real collision walls.
    # This bounded fixture is not arbitrary flat-table placement.
    for axis in (0,1):
        for sign in (-1,1):
            position = [tx,ty,tz]
            position[axis] += sign*.095
            size = [.105,.105,.035]
            size[axis] = .01
            ET.SubElement(world,'geom',name=f'receiving_rim_{axis}_{sign}',type='box',
                          pos=' '.join(map(str,position)),size=' '.join(map(str,size)),
                          rgba='.15 .65 .3 1',condim='6',friction='1 .005 .001')
    obj = ET.SubElement(world, 'body', name='payload', pos=f'{sx} {sy} {sz}')
    ET.SubElement(obj, 'freejoint', name='payload_free')
    ET.SubElement(obj, 'geom', name='payload_geom', type='sphere', size='.035', mass='.05',
                  rgba='1 .5 .1 1', friction='1 .005 .0001', condim='6')
    ET.indent(root)
    output = ASSETS/'combined.xml'
    ET.ElementTree(root).write(output, encoding='unicode')
    base = json.loads((ASSETS/'manifest-t800.json').read_text())
    files = [output, ROOT/'config/mission.json', *dest.rglob('*')]
    base.update(hand_revision=HAND_REV, model='Experimental T800 + right Allegro hand',
                modifications=['Right wrist placeholder collision sphere replaced by Allegro right hand',
                               f'Experimental angled rigid adapter: translation (0,0,-0.11), pitch {angle} rad before hand flip',
                               'Hand names prefixed; original finger dynamics retained; no base fixation'])
    base['sha256'].update({p.relative_to(ROOT).as_posix():hashlib.sha256(p.read_bytes()).hexdigest()
                          for p in files if p.is_file()})
    (ASSETS/'manifest.json').write_text(json.dumps(base, indent=2)+'\n')
    print('006 combined model prepared; integration experiments required before manipulation claims.')


if __name__ == '__main__':
    main()
