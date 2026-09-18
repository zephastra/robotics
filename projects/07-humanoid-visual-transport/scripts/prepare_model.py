"""Assemble two real articulated hands and a head-mounted RGB-D camera."""
from pathlib import Path
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
                nodes = list(expanded(path.parent/child.get('file')))
                index = list(parent).index(child)
                parent.remove(child)
                for offset,node in enumerate(nodes): parent.insert(index+offset,node)
            else:
                if child.tag in ('mesh','texture','hfield') and child.get('file'):
                    child.set('file',(path.parent/child.get('file')).resolve().relative_to(ASSETS).as_posix())
                visit(child)
    visit(root)
    return root


def main():
    upstream = ROOT/'.cache/menagerie'
    assert subprocess.check_output(['git','-C',str(upstream),'rev-parse','HEAD'],text=True).strip()==HAND_REV
    subprocess.run(['git','-C',str(upstream),'diff','--exit-code','HEAD','--','wonik_allegro'],check=True)
    dest=ASSETS/'allegro'
    shutil.copytree(upstream/'wonik_allegro',dest,dirs_exist_ok=True)
    shutil.copy2(dest/'LICENSE',ROOT/'licenses/Allegro-BSD-2-Clause.txt')
    root=expanded(ASSETS/'t800/scene.xml')
    root.set('model','007 T800 active vision and bilateral Allegro — experimental')
    cfg=json.loads((ROOT/'config/scene.json').read_text())
    for side,letter,prefix in [('left','L','lh_'),('right','R','rh_')]:
        hand=ET.parse(dest/f'{side}_hand.xml').getroot()
        for mesh in hand.findall('./asset/mesh'):
            if mesh.get('name') is None: mesh.set('name',Path(mesh.get('file')).stem)
            mesh.set('file','allegro/assets/'+mesh.get('file'))
        for node in hand.iter():
            for key in ('name','class','childclass','mesh','material','joint','body1','body2','site'):
                if key in node.attrib: node.set(key,prefix+node.get(key))
        for tag in ('default','asset','actuator','contact'):
            section=root.find(tag)
            if section is None: section=ET.SubElement(root,tag)
            other=hand.find(tag)
            if other is not None: section.extend(list(other))
        wrist=root.find(f".//body[@name='LINK_WRIST_END_{letter}']")
        for geom in list(wrist.findall('geom')): wrist.remove(geom)
        palm=hand.find('./worldbody/body')
        angle=cfg['hand_mount_pitch']
        palm.set('pos','0 0 -.11')
        palm.set('quat',f'0 {math.cos(angle/2)} 0 {-math.sin(angle/2)}')
        wrist.append(palm)
        ET.SubElement(palm,'site',name=prefix+'grasp',pos='.06 0 .015',size='.003',rgba='1 .8 0 1')
    head=root.find(".//body[@name='LINK_HEAD_YAW']")
    angle=math.radians(cfg['camera_mount_down_degrees'])
    ET.SubElement(head,'camera',name='eyes',pos='.10 0 .08',
                  xyaxes=f'0 -1 0 {math.sin(angle)} 0 {math.cos(angle)}',fovy='65')
    ET.SubElement(head,'geom',name='camera_housing',type='box',pos='.085 0 .08',
                  size='.012 .035 .012',rgba='.1 .15 .2 1',contype='0',conaffinity='0',mass='0')
    world=root.find('worldbody')
    for name,position in [('source',cfg['source']),('destination',cfg['destination'])]:
        x,y,z=position
        ET.SubElement(world,'geom',name=name+'_table',type='box',pos=f'{x} {y} {z-.047}',
                      size='.10 .20 .007',rgba='.35 .40 .45 1',condim='6',friction='1 .005 .001')
        ET.SubElement(world,'geom',name=name+'_leg',type='cylinder',pos=f'{x+.08} {y} {(z-.054)/2}',
                      size=f'.022 {(z-.054)/2}',rgba='.3 .35 .4 1')
        if name=='destination':
            marker_y=y+cfg['destination_marker_offset_y']
            ET.SubElement(world,'geom',name='destination_marker_plate',type='box',pos=f'{x} {marker_y} {z-.045}',
                          size='.08 .14 .005',rgba='.35 .4 .45 1')
            ET.SubElement(world,'geom',name='destination_marker_post',type='cylinder',
                          pos=f'{x} {marker_y} {(z-.05)/2}',size=f'.015 {(z-.05)/2}',rgba='.35 .4 .45 1')
            ET.SubElement(world,'geom',name='destination_marker',type='box',pos=f'{x} {marker_y} {z-.0395}',
                          size='.06 .12 .0004',rgba='.05 1 .1 1',contype='0',conaffinity='0',mass='0')
    x,y,z=cfg['source']
    box=ET.SubElement(world,'body',name='payload',pos=f'{x} {y} {z}')
    ET.SubElement(box,'freejoint',name='payload_free')
    ET.SubElement(box,'geom',name='payload_box',type='box',size=' '.join(map(str,cfg['box_halfsize'])),
                  mass=str(cfg['box_mass']),rgba='1 .4 .04 1',condim='6',friction='1 .005 .001')
    for sign in (-1,1):
        hy=sign*cfg['handle_y']
        ET.SubElement(box,'geom',name=f'handle_{sign}',type='sphere',pos=f'0 {hy} 0',
                      size=str(cfg['handle_radius']),mass='.025',rgba='1 .4 .04 1',condim='6',friction='1 .005 .001')
        ET.SubElement(box,'geom',name=f'handle_stem_{sign}',type='capsule',
                      fromto=f'0 {sign*.22} 0 0 {hy} 0',size='.009',mass='.005',rgba='1 .4 .04 1')
    ET.SubElement(box,'geom',name='visual_marker',type='box',pos='0 0 .041',size='.045 .18 .001',
                  rgba='1 .02 1 1',contype='0',conaffinity='0',mass='0')
    ET.indent(root)
    output=ASSETS/'combined.xml'
    ET.ElementTree(root).write(output,encoding='unicode')
    manifest=json.loads((ASSETS/'manifest-t800.json').read_text())
    manifest.update(hand_revision=HAND_REV,model='T800 + bilateral Allegro + head RGB-D camera',
                    modifications=['Experimental bilateral rigid hand adapters; original finger dynamics',
                                   'Both wrist placeholder spheres replaced; free base retained',
                                   'Head camera tilted down 35 degrees; no payload constraints'])
    files=[output,ROOT/'config/scene.json',ROOT/'config/task.json',*dest.rglob('*')]
    manifest['sha256'].update({p.relative_to(ROOT).as_posix():hashlib.sha256(p.read_bytes()).hexdigest()
                               for p in files if p.is_file()})
    (ASSETS/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
    print('007 model assembled. Physical validation is required.')


if __name__=='__main__': main()
