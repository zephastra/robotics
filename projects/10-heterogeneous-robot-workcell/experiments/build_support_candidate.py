"""User-approved independent W2 support prototype, NOT an integrated/order world.

Add passive idlers between source-band rollers. Original world, controls, contact
parameters, tray and acceptance thresholds remain unchanged. No weld or cargo lock.
"""
import argparse
import hashlib
import json
import re
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import mujoco
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'experiments'))
from build_p5_candidate_world import compile_text

SOURCE = ROOT / 'assets/world_w2_logistic.xml'
TARGET = ROOT / 'assets/world_w2_support_v1.xml'
IDLER_RADIUS_M = .010  # declared prototype dimension, not a changed criterion


def preserve_keys(original_text, candidate_text):
    """Map old keyframe joint states by body/name, never by shifted DOF indices."""
    old = compile_text(original_text)
    new = compile_text(candidate_text)
    if old.nu != new.nu or old.na != new.na:
        raise ValueError('passive support must not change actuator dimensions')
    if [old.actuator(a).name for a in range(old.nu)] != [new.actuator(a).name for a in range(new.nu)]:
        raise ValueError('actuator order changed')
    root = ET.fromstring(candidate_text)
    keys = ET.SubElement(root, 'keyframe')
    qwidth = {mujoco.mjtJoint.mjJNT_FREE: 7, mujoco.mjtJoint.mjJNT_BALL: 4,
              mujoco.mjtJoint.mjJNT_SLIDE: 1, mujoco.mjtJoint.mjJNT_HINGE: 1}
    vwidth = {mujoco.mjtJoint.mjJNT_FREE: 6, mujoco.mjtJoint.mjJNT_BALL: 3,
              mujoco.mjtJoint.mjJNT_SLIDE: 1, mujoco.mjtJoint.mjJNT_HINGE: 1}
    for k in range(old.nkey):
        qpos, qvel = new.qpos0.copy(), np.zeros(new.nv)
        for j in range(old.njnt):
            old_body = int(old.jnt_bodyid[j])
            new_body = new.body(old.body(old_body).name).id
            target_joint = int(new.body_jntadr[new_body]) + j - int(old.body_jntadr[old_body])
            if (old.joint(j).name != new.joint(target_joint).name
                    or old.jnt_type[j] != new.jnt_type[target_joint]):
                raise ValueError('joint identity changed during key mapping')
            kind = mujoco.mjtJoint(int(old.jnt_type[j]))
            qa, qb = int(old.jnt_qposadr[j]), int(new.jnt_qposadr[target_joint])
            va, vb = int(old.jnt_dofadr[j]), int(new.jnt_dofadr[target_joint])
            qpos[qb:qb+qwidth[kind]] = old.key_qpos[k, qa:qa+qwidth[kind]]
            qvel[vb:vb+vwidth[kind]] = old.key_qvel[k, va:va+vwidth[kind]]
        attrs = dict(time=format(float(old.key_time[k]), '.12g'),
                     qpos=' '.join(format(v, '.17g') for v in qpos),
                     qvel=' '.join(format(v, '.17g') for v in qvel),
                     ctrl=' '.join(format(v, '.17g') for v in old.key_ctrl[k]))
        if old.key(k).name:
            attrs['name'] = old.key(k).name
        if old.na:
            attrs['act'] = ' '.join(format(v, '.17g') for v in old.key_act[k])
        if old.nmocap:
            raise ValueError('mocap key mapping not supported')
        ET.SubElement(keys, 'key', **attrs)
    return ET.tostring(root, encoding='unicode')


def build(text, split=False, roller_pattern=r'c_fixed_roller_\d+_\d+',
          idler_prefix='c_fixed_roller_idler', parent_name=None):
    root = ET.fromstring(re.sub(r'<!--.*?-->', '', text, flags=re.S))
    world = root.find('worldbody')
    if parent_name is not None:
        world = next((node for node in world.iter('body')
                      if node.get('name') == parent_name), None)
        if world is None:
            raise ValueError('declared roller parent missing')
    rollers = sorted([b for b in world.findall('body')
                      if re.fullmatch(roller_pattern, b.get('name', ''))],
                     key=lambda b: float(b.get('pos').split()[0]))
    if len(rollers) < 2:
        raise ValueError('source roller row missing')
    keyframe = root.find('keyframe')
    if keyframe is not None:
        # Diagnostics initialise BY NAME using merged_home, not these old-width vectors.
        root.remove(keyframe)
    pairs = []
    for i, (left, right) in enumerate(zip(rollers, rollers[1:])):
        lp = np.fromstring(left.get('pos'), sep=' ')
        rp = np.fromstring(right.get('pos'), sep=' ')
        geom = left.find('geom')
        size = np.fromstring(geom.get('size'), sep=' ')
        radius, half_length = size[:2]
        if not np.allclose(lp[1:], rp[1:]) or rp[0] <= lp[0]:
            raise ValueError('source rollers must form one aligned row')
        pos = (lp + rp) / 2
        pos[2] += radius - IDLER_RADIUS_M
        gap = float(np.linalg.norm(pos - lp) - radius - IDLER_RADIUS_M)
        if gap <= 0:
            raise ValueError('idler would intersect original roller')
        for side in (-1, 1) if split else (0,):
            local_pos = pos.copy()
            local_pos[1] += side * half_length / 2
            local_half = half_length / 4 if split else half_length
            name = f'{idler_prefix}_{i}' + (f'_side{side}' if split else '')
            body = ET.SubElement(world, 'body', name=name,
                                 pos=' '.join(format(v, '.12g') for v in local_pos))
            ET.SubElement(body, 'joint', name=name+'_joint', axis='0 1 0',
                          damping='0', frictionloss='0')
            attrs = dict(geom.attrib)
            attrs.update(name=name, size=f'{IDLER_RADIUS_M} {local_half}',
                         mass=format(float(geom.get('mass'))*(IDLER_RADIUS_M/radius)**2
                                     *local_half/half_length, '.12g'))
            ET.SubElement(body, 'geom', **attrs)
            pairs.append(dict(name=name, neighbour_clearance_m=gap,
                              support_pitch_m=float((rp[0]-lp[0])/2),
                              crown_z_m=float(pos[2]+IDLER_RADIUS_M)))
    return ET.tostring(root, encoding='unicode'), pairs


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run-id', required=True)
    parser.add_argument('--write', action='store_true')
    parser.add_argument('--split', action='store_true')
    parser.add_argument('--integrated', action='store_true')
    parser.add_argument('--receiver',action='store_true',
                        help='v4: preserve v3 and add split passive idlers to receiver only')
    parser.add_argument('--deck',action='store_true',
                        help='v5: preserve v4 and add split passive idlers on first vehicle deck')
    args = parser.parse_args()
    if args.integrated and not args.split:
        parser.error('integrated candidate requires split supports')
    if args.receiver and not (args.integrated and args.split):
        parser.error('receiver candidate requires --integrated --split')
    if args.deck and (args.receiver or not (args.integrated and args.split)):
        parser.error('deck candidate requires --integrated --split without --receiver')
    source = SOURCE.with_name('world_p5_candidate_v2.xml') if args.integrated else SOURCE
    if args.receiver:source=SOURCE.with_name('world_p5_candidate_v3.xml')
    if args.deck:source=SOURCE.with_name('world_p5_candidate_v4.xml')
    raw = source.read_bytes()
    target = (TARGET.with_name('world_p5_candidate_v3.xml') if args.integrated else
              TARGET.with_name('world_w2_support_v2.xml') if args.split else TARGET)
    if args.receiver:target=TARGET.with_name('world_p5_candidate_v4.xml')
    if args.deck:target=TARGET.with_name('world_p5_candidate_v5.xml')
    text, pairs = build(raw.decode(), split=args.split,
        **(dict(roller_pattern=r'c_recv_roller_\d+',idler_prefix='c_recv_roller_idler')
           if args.receiver else dict(roller_pattern=r'c_deck_roller_\d+',
               idler_prefix='c_deck_roller_idler',parent_name='c_deck') if args.deck else {}))
    if args.integrated:
        text = preserve_keys(re.sub(r'<!--.*?-->', '', raw.decode(), flags=re.S), text)
    model = compile_text(text)
    idlers = [model.body(p['name']).id for p in pairs]
    passive = all(not any(model.actuator_trnid[:, 0] == model.body_jntadr[b]) for b in idlers)
    if not passive:
        raise ValueError('idler unexpectedly actuated')
    if args.write:
        if target.exists():
            raise FileExistsError('never overwrite an existing candidate')
        target.write_text(text+'\n')
    elif not target.exists() or target.read_text() != text+'\n':
        raise ValueError('candidate absent or differs from reconstruction')
    if source.read_bytes() != raw:
        raise RuntimeError('source asset changed')
    report = dict(scope=('INTEGRATED_SOURCE_SUPPORT_STATIC_ONLY' if args.integrated
                         else 'SOURCE_SUPPORT_STATIC_PROTOTYPE_ONLY'),
        source=str(source.relative_to(ROOT)), target=str(target.relative_to(ROOT)),
        source_sha256=hashlib.sha256(raw).hexdigest(),
        candidate_sha256=hashlib.sha256(target.read_bytes()).hexdigest(),
        split_axial_support=args.split,
        support_row='deck' if args.deck else 'receiver' if args.receiver else 'source',
        passive_idlers=len(idlers), pairs=pairs,
        minimum_clearance_m=min(p['neighbour_clearance_m'] for p in pairs),
        maximum_support_pitch_m=max(p['support_pitch_m'] for p in pairs),
        dynamic_stop='NOT_RUN', loaded_transfer='NOT_RUN',
        integration='NOT_RUN', v1_complete=False, diagnostic_result='STATIC_ONLY')
    if args.integrated:
        import build_p5_candidate_world as layout
        _, declarations = layout.build(layout.SOURCE.read_text(), stock_offset_x=.50)
        report['integration_layout_audit'] = layout.audit(text, declarations)
    out = ROOT / 'reports' / args.run_id
    out.mkdir(parents=True, exist_ok=False)
    (out/'report.json').write_text(json.dumps(report, indent=2)+'\n')
    print(json.dumps(report, indent=2))
    return (1 if args.integrated and
            report['integration_layout_audit']['diagnostic_result'] != 'PASS' else 0)


if __name__ == '__main__':
    raise SystemExit(main())
