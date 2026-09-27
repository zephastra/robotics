"""Generate a world file for a chosen object, and extract object fragments.

`assets/combined.xml` is the carried-over 007 snapshot. It is READ, never written: the
plan requires the original model snapshot to be preserved, and this project has been
bitten before by a "fix" that quietly rewrote the thing it was measuring. A test asserts
its hash is unchanged after this script runs.

Jobs:

  --extract                     pull the `payload` body out of combined.xml into
                                assets/objects/payload_007.xml, mechanically, so the
                                fragment is provably the original text, not a retyping.
  --object <name>               write assets/world_<name>.xml: combined.xml with the
                                `payload` body replaced by assets/objects/<name>.xml.
  --object <name> --check       rebuild in memory and compare with the file on disk;
                                exit 1 on any difference, writing nothing. This is how a
                                test can verify the committed world matches its sources
                                without mutating the tree.

The generated world lands in the SAME directory as combined.xml, so every relative mesh
path inside it resolves exactly as before. A world written one directory deeper would
silently lose its meshes.

Every run compares the robot part of the source and the generated world and refuses to
write if they differ. Without that check a generated world could diverge from the
snapshot and nobody would notice until the numbers stopped making sense.
"""
import argparse
import hashlib
import pathlib
import sys

ROOT = pathlib.Path(__file__).resolve().parents[1]
ASSETS = ROOT / 'assets'
COMBINED = ASSETS / 'combined.xml'
OBJECTS = ASSETS / 'objects'
OBJECT_NAME = 'payload'

EXTRACT_HEADER = '''<!--
  Object fragment extracted verbatim from assets/combined.xml by
  experiments/make_world.py --extract. This is the 007 carried-over object.

  Do not hand-edit: re-run the extractor instead, so the fragment cannot drift away from
  the snapshot it is supposed to represent. combined.xml is not modified by this.
-->
'''


def _body_span(shape):
    """Character span of the top-level `<body name="payload">` element."""
    start_tag = f'<body name="{OBJECT_NAME}"'
    start = shape.find(start_tag)
    if start < 0:
        raise ValueError(f'{OBJECT_NAME} body not found')
    end = shape.find('</body>', start)
    if end < 0:
        raise ValueError(f'{OBJECT_NAME} body has no closing tag')
    end += len('</body>')
    block = shape[start:end]
    if block.count('<body') != 1:
        raise ValueError(f'{OBJECT_NAME} block contains nested bodies; the extractor '
                         f'assumes a flat body and would corrupt the file')
    return start, end, block


def fragment_body(path):
    """The bare <body> element of an object fragment, with any header comment stripped."""
    text = path.read_text(encoding='utf-8')
    start = text.find(f'<body name="{OBJECT_NAME}"')
    if start < 0:
        raise ValueError(f'{path}: no <body name="{OBJECT_NAME}"> element')
    end = text.find('</body>', start) + len('</body>')
    block = text[start:end]
    if block.count('<body') != 1:
        raise ValueError(f'{path}: fragment contains nested bodies')
    return block


def robot_fingerprint(path):
    """Structure of everything except the object: the part that must not change.

    The object's own geoms are excluded as well as its body. Counting them would make
    every generated world "differ" purely because the new object has more shapes, which
    is the one difference this comparison must tolerate.
    """
    import mujoco
    model = mujoco.MjModel.from_xml_path(str(path))
    object_id = model.body(OBJECT_NAME).id
    bodies = tuple(sorted(model.body(i).name for i in range(model.nbody)
                          if i != object_id))
    other_geoms = sum(1 for g in range(model.ngeom)
                      if int(model.geom_bodyid[g]) != object_id)
    return {
        'nq': int(model.nq), 'nv': int(model.nv), 'nu': int(model.nu),
        'neq': int(model.neq), 'nbody': int(model.nbody),
        'ngeom_outside_object': other_geoms,
        'nsite': int(model.nsite), 'nmesh': int(model.nmesh),
        'bodies': bodies,
        'robot_mass': round(float(model.body_mass.sum() - model.body_mass[object_id]), 6),
    }


def diff_fingerprints(before, after):
    return {k: (before[k], after[k]) for k in before if before[k] != after[k]}


def extract_text():
    _, _, block = _body_span(COMBINED.read_text(encoding='utf-8'))
    return EXTRACT_HEADER + block + '\n'


def build_text(name):
    """The generated world for `name`, or None when it is the inline default."""
    if name == 'payload_007':
        return None
    fragment = OBJECTS / f'{name}.xml'
    if not fragment.is_file():
        raise FileNotFoundError(f'{fragment.relative_to(ROOT)} does not exist')
    shape = COMBINED.read_text(encoding='utf-8')
    start, end, _ = _body_span(shape)
    return shape[:start] + fragment_body(fragment) + shape[end:]


def _compare(target, text, label):
    if not target.is_file():
        print(f'MISSING {label}: {target.relative_to(ROOT)} has not been generated')
        return 1
    on_disk = target.read_text(encoding='utf-8')
    if on_disk == text:
        print(f'OK {label}: {target.relative_to(ROOT)} matches its sources '
              f'(sha256 {hashlib.sha256(target.read_bytes()).hexdigest()})')
        return 0
    print(f'STALE {label}: {target.relative_to(ROOT)} differs from what its sources '
          f'produce. Re-run without --check.')
    print(f'  on disk {len(on_disk)} chars, rebuilt {len(text)} chars')
    return 1


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--extract', action='store_true')
    parser.add_argument('--object')
    parser.add_argument('--check', action='store_true',
                        help='compare with what is on disk; write nothing')
    args = parser.parse_args()

    if args.extract:
        text = extract_text()
        target = OBJECTS / 'payload_007.xml'
        if args.check:
            return _compare(target, text, 'extracted fragment')
        OBJECTS.mkdir(parents=True, exist_ok=True)
        target.write_text(text, encoding='utf-8')
        print(f'extracted {len(text)} chars -> {target.relative_to(ROOT)}')
        print(f'  sha256 {hashlib.sha256(target.read_bytes()).hexdigest()}')
        return 0

    if args.object:
        name = args.object
        if name == 'payload_007':
            print('payload_007 is the object already inside combined.xml; nothing to build')
            return 0
        try:
            text = build_text(name)
        except FileNotFoundError as exc:
            print(f'ABORT: {exc}')
            return 2
        target = ASSETS / f'world_{name}.xml'
        if args.check:
            return _compare(target, text, f'world for {name}')
        target.write_text(text, encoding='utf-8')
        import mujoco
        source = mujoco.MjModel.from_xml_path(str(COMBINED))
        model = mujoco.MjModel.from_xml_path(str(target))
        drift = diff_fingerprints(robot_fingerprint(COMBINED), robot_fingerprint(target))
        if drift:
            target.unlink()
            print(f'ABORT: generated world differs from the snapshot outside the object '
                  f'body: {drift}')
            return 2
        print(f'wrote {target.relative_to(ROOT)}')
        print(f'  robot part identical to combined.xml')
        print(f'  object mass {float(model.body_mass[model.body(OBJECT_NAME).id]):.3f} kg '
              f'(007 tray {float(source.body_mass[source.body(OBJECT_NAME).id]):.3f} kg)')
        print(f'  world sha256 {hashlib.sha256(target.read_bytes()).hexdigest()}')
        print(f'  fragment     assets/objects/{name}.xml')
        return 0

    parser.error('give --extract or --object <name>')
    return 2


if __name__ == '__main__':
    # The snapshot must not move while this script runs. The hash pair has to live in
    # this scope: the first version read `before` from main()'s locals and died with
    # NameError only after it had already written the file.
    _before = hashlib.sha256(COMBINED.read_bytes()).hexdigest()
    _code = main()
    if _code == 0 and hashlib.sha256(COMBINED.read_bytes()).hexdigest() != _before:
        print('ABORT: combined.xml changed during this run; the snapshot must not move')
        _code = 2
    sys.exit(_code)
