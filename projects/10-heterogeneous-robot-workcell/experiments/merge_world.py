"""Merge the four P1 roles (H, A, C, N) into one MuJoCo world: `assets/world_p1_cell.xml`.

This is the `P1-GATE-07` artifact. It exists to answer three questions and no others:

  1. does one MjModel load all four roles at once,
  2. is every degree of freedom addressable without reaching into another role,
  3. does the joint scene settle instead of exploding.

It deliberately does NOT claim that the four tasks run together, and it does not mount C's
deck on N's chassis -- C's deck is world-welded and N's chassis has no mounting interface yet.
Those are the next steps, not this one.

WHY NOT HAND-WRITTEN XML
-----------------------
Each role already has a generator (`arm_rig.py`, `roller_rig.py`, `make_world.py`), and this
file is the same discipline applied to the merge: nothing is typed that can be derived, and
the artifact is rebuilt and compared rather than edited. `--check` rebuilds in memory and
diffs against the file on disk, so the committed world cannot drift away from its sources.

WHY `MjSpec.attach` RATHER THAN TEXT CONCATENATION
--------------------------------------------------
Concatenating four XML files would require merging four `<default>` trees, four `<asset>`
sections and four `<actuator>` blocks by hand. `MjSpec.attach` carries bodies, geoms, sites,
joints, actuators, sensors, equalities and meshes, and applies the name prefix for us. Two
things it does NOT do, both found by measurement rather than assumed:

  * a carried `<keyframe>` hard-fails: "Keyframe 'home' has invalid qpos size, got 9, should
    be 16", because the keyframe's width is the CHILD's nq and the merged model's is larger.
    So every child's keyframes are deleted before attaching, and the merged world's single
    home keyframe is rebuilt by NAME (see `merged_home`).
  * mesh and texture paths are not rebased. The child's `meshdir` does not travel, so the
    parent's `meshdir` would be applied to the child's bare filenames and every mesh would
    be looked up in the wrong place. Each path is therefore rebased explicitly.

WHY THE STATIONS ARE DERIVED
----------------------------
All four roles sit near the origin in their own worlds, so merging them in place would put
four tables inside each other. Each role is therefore attached at a station, and the station
positions, the ground plane and the cell boundary are all computed from measured footprints
plus a declared clearance -- not typed.
"""
import argparse
import hashlib
import pathlib
import posixpath
import sys

import mujoco
import numpy as np
import yaml

ROOT = pathlib.Path(__file__).resolve().parents[1]
ASSETS = ROOT / 'assets'
TARGET = ASSETS / 'world_p1_cell.xml'

#: Empty floor between two stations' measured footprints.
ROLE_CLEARANCE = 0.30

#: Cell boundary: walls are a boundary marker, not a safety fence, and nothing in this gate
#: depends on their height. Declared here so the number has one owner.
CELL_WALL_HEIGHT = 0.60
CELL_WALL_THICKNESS = 0.05
CELL_WALL_MARGIN = 0.50

#: Integration settings the merged world must use. Not typed: `sources_agree_on_stepping`
#: asserts all four sources already use exactly this, so the gate cannot silently unify two
#: different time bases by picking one. Found by measurement: all four are already
#: timestep 0.002 / implicitfast.
MERGED_TIMESTEP = 0.002
MERGED_INTEGRATOR = 'implicitfast'

#: Ground friction, and the measurement that makes it a non-issue.
#:
#: The four sources disagree about the floor:  H 1.00  A 0.60  C 0.25  N 1.20.
#: One shared ground cannot BE all four -- but it does not have to be. MuJoCo combines two
#: geoms' friction by ELEMENTWISE MAX when their `priority` is equal, measured in situ: N's
#: wheels declare 1.6, this floor declares 0.25, and the resulting contact carried 1.6 on
#: every component. So a floor BELOW a contacting geom's own value is invisible, and a floor
#: ABOVE it takes over and stops that role reproducing its own world.
#:
#: That makes the LOWEST of the four the safest choice, and it makes the admissible set an
#: interval rather than a value. Measured by sweeping this floor and diffing the whole 3 s
#: settle against this value:
#:
#:     floor 0.001 0.10 0.25 0.50 0.89 0.90 0.95 -> deviation 0.0000e+00  (all identical)
#:     floor 1.20                                 -> deviation 5.3850e+00
#:     floor 5.00                                 -> deviation 4.5557e+00  (sensitivity control)
#:
#: The boundary is 1.00: the lowest friction among the geoms that actually touch the floor
#: (role H's, which all declare 1.00). The 5.00 arm exists so that the agreement of the other
#: arms is not the agreement of a dead probe -- without an arm that MUST change something,
#: "identical" proves nothing.
#:
#: `evaluate_gate.py` re-measures that boundary on every run and FAILS if this value ever
#: rises above it, so this is a monitored invariant rather than a comment.
SHARED_GROUND_FRICTION = 0.25
SHARED_GROUND_CONDIM = 3

#: The humanoid's base body carries an UNNAMED free joint in H's source. It is named in the
#: merged world so that nothing downstream has to reach a joint through its parent body.
#: Measured reason: an unnamed joint cannot be addressed by name, so the home assembly could
#: not match it and skipped it -- leaving a ZERO quaternion on the base and the humanoid's feet
#: 1.0165 m inside the ground. `LINK_BASE` is the source's own body name; the merged one is
#: prefixed. `verify_kinematics` still reports the source as unnamed, which is the truth about
#: the source.
HUMANOID_BASE_BODY = 'LINK_BASE'
BASE_FREE_JOINT = 'base_free'

# --- the V1 assembly: C's deck rides on N's chassis (2026-09-23) -------------------------
#
# WHAT CHANGED AND WHY IT IS NOT JUST A MOVE. C's rig was built with the deck as a WORLD body,
# so `RECV_FIRST_CROWN_X = DECK_LAST_CROWN_X + 2r + CROWN_CLEARANCE` asserted a FIXED DISTANCE
# between the deck and the receiving section -- a constant, true by construction.
#
# Once the deck is bolted to the chassis that distance is not a constant any more: it is
# `(where the AMR parked) - (where the receiver is)`. The same expression now describes a
# DOCKING TOLERANCE, which is a different claim needing its own evidence. This module therefore
# does not "convert a number" -- it changes what C's end position is measured against, and it
# records the change so C's standalone evidence stays readable next to the assembled world.
#
# THE ASSEMBLY IS AN INITIAL CONDITION, NOT A DRIVE. The AMR is parked so that the deck's world
# position reproduces its standalone value exactly (`DOCK_OFFSET == 0`). Driving the robot to the
# dock would make C's "the receiver is fixed and the deck travels out to meet it" claim circular,
# so the chassis is placed and then reported, never commanded.
#
# WHAT KEEPS ITS MEANING ACROSS THE ASSEMBLY:
#   * deck-local geometry: all eight `deck_roller_*`, the frame and the whole pusher are already
#     expressed relative to the deck origin, so they are untouched (the pusher constants
#     `PUSHER_*` are deck-relative by construction -- see roller_rig's `PUSHER_CARRIAGE_X`).
#   * `deck_slide` becomes a CHASSIS-relative joint, which is the mechanical intent.
#   * `PLATFORM_CLEAR` is a RELATIVE criterion (`rel_x` against `DECK_LENGTH`), so it survives.
# WHAT DOES NOT:
#   * anything that reads the deck's world x directly. Those are reported, not silently reused.
CHASSIS_MOUNT_BODY = 'base_link'          # N's chassis root, in N's source
DECK_BODY = 'deck'                        # C's deck root, in C's source
DECK_SUBTREE_PREFIX = ('deck',)           # `deck` plus everything attached under it

#: The chassis pose that reproduces the standalone rig. `slide_x` at this value puts the deck
#: back at `DECK_X0` in the world, so `dock_offset` is exactly zero and C's numbers transfer.
#: Derived at build time from the measured geometry, not typed.
def deck_world_x(model):
    """The deck's world x in a compiled model. The single place this is read."""
    bid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, 'c_deck')
    if bid < 0:
        raise RuntimeError('c_deck is missing from the merged model')
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    return float(data.xpos[bid][0])


#: Filled in by `build_spec` and read by the gate and by `--assembly`. A dict rather than a
#: return value because `build_spec`'s signature is used by tests, and because this is
#: diagnostic: nothing decides anything from it.
ASSEMBLY_REPORT = {}


class Role:
    def __init__(self, key, label, source, furniture=(), meshdir=None):
        self.key = key
        self.label = label
        self.source = source          # str path, or None when the role is generated in-process
        self.furniture = tuple(furniture)
        self.meshdir = meshdir

    @property
    def prefix(self):
        return f'{self.key}_'

    def __repr__(self):
        return f'<Role {self.key} {self.label}>'


#: Declaration order, and the order along +x. It is fixed so the merged index layout is
#: deterministic: two builds of the same sources must produce byte-identical XML.
ROLES = (
    Role('h', 'humanoid + V1 tray', 'assets/world_tray_v1.xml', furniture=('floor',)),
    Role('a', 'fixed arm + part + table', 'assets/world_arm_a.xml', furniture=('floor',)),
    Role('c', 'deck + rollers + tray', None, furniture=('floor',)),
    # N's arena is its own single-role fixture: an 8x8 m box with three pillars, used for a
    # navigation test. It is not cell furniture, so the walls and pillars are dropped and the
    # cell boundary is generated from the merged layout instead. Recorded because N's
    # navigation evidence cites `pillar_a`, which therefore does not exist in this world.
    Role('n', 'diff-drive chassis', 'assets/worlds/world_n_probe.xml',
         furniture=('floor', 'wall_north', 'wall_south', 'wall_east', 'wall_west',
                    'pillar_a', 'pillar_b', 'pillar_c')),
)


def role_path(role):
    """Materialise a role's source XML, since C's only exists as a string from its builder.

    C's world has to land in `assets/` rather than a temp directory: MuJoCo resolves relative
    mesh paths against the XML file's own directory, and a world written one directory deeper
    silently loses its meshes.
    """
    if role.source is not None:
        return ROOT / role.source
    sys.path.insert(0, str(ROOT / 'experiments'))
    import roller_rig as rig
    out = ASSETS / '_merge_role_c.xml'
    out.write_text(rig.build_model(ASSETS / 'objects' / 'tray_v1.xml',
                                   receiver=True, guides=True)[0], encoding='utf-8')
    return out


def rebase(path, child_meshdir, parent_meshdir=''):
    """A child's asset path, expressed relative to the merged world's meshdir."""
    joined = posixpath.normpath(posixpath.join(child_meshdir or '', path))
    if parent_meshdir:
        return posixpath.relpath(joined, parent_meshdir)
    return joined


def decompose_c_xml():
    """Split C's built world into (deck subtree, world-fixed remainder, dropped furniture).

    C's world is FLAT: 24 `fixed_roller_*`, the `recv_roller_*` row, the `deck` subtree and the
    cargo `payload` are all siblings under `worldbody`. So the assembly is not a re-parenting of
    a tree -- it is a split of a sibling list, and it has to happen BEFORE `attach`, because
    `MjsBody.parent` is read-only (measured: `AttributeError: property ... has no setter`).

    What goes where:
      * THE DECK SUBTREE rides on the chassis: `deck`, its eight rollers, the pusher carriage
        and the blade -- 11 bodies. `attach` carries the hierarchy under `deck` for free.
      * THE WORLD stays put: the three fixed sections and the receiving row. They are the
        fixture the AMR drives up to, so bolting them to the robot would delete the experiment.
      * THE CARGO stays at world level and keeps its free joint. This is not a workaround for
        `free joint can only be used on top level` -- it is also the physically correct answer:
        the tray is goods being handed over, not a part of the vehicle.

    Returns (deck_root_element, world_fixed_xml_text, report dict).
    """
    import xml.etree.ElementTree as ET
    path = role_path(next(r for r in ROLES if r.key == 'c'))
    root = ET.fromstring(path.read_text(encoding='utf-8'))
    worldbody = root.find('worldbody')

    deck_root = None
    for body in worldbody.findall('body'):
        if body.get('name') == DECK_BODY:
            deck_root = body
            break
    if deck_root is None:
        raise RuntimeError(f"C's world has no body named {DECK_BODY!r}; the assembly has no "
                           f"deck to mount")

    # Everything not under the deck root: fixed rollers, the receiver row, and the cargo.
    world_fixed = ET.Element('mujoco', attrib=dict(root.attrib))
    for tag in ('compiler', 'option', 'default', 'asset', 'actuator', 'sensor', 'equality',
                'contact', 'tendon'):
        found = root.find(tag)
        if found is not None:
            world_fixed.append(found)
    wb = ET.SubElement(world_fixed, 'worldbody')
    moved, kept = [], []
    for child in list(worldbody):
        if child is deck_root:
            continue
        wb.append(child)
        name = child.get('name') or ''
        (moved if name.startswith('payload') else kept).append(name or child.tag)
    return (deck_root, ET.tostring(world_fixed, encoding='unicode'),
            {'deck_bodies': len(list(deck_root.iter('body'))) + 1,
             'world_kept': len(kept), 'cargo': moved})


def load_deck_spec():
    """Only the deck subtree, as a detached spec ready for `attach`.

    `spec.default.name` IS writable in this build (measured: `'main'` -> `'c_main'`), which is
    what keeps two specs derived from C's XML from colliding on `repeated default class name`.
    C's default class is the GLOBAL one, so without this the merged world refuses to compile
    the moment the source is loaded twice.
    """
    import xml.etree.ElementTree as ET
    deck_root, _, report = decompose_c_xml()
    root = ET.Element('mujoco', model='010 C deck only (mounted on the chassis)')
    ET.SubElement(root, 'compiler', angle='radian', autolimits='true')
    ET.SubElement(root, 'option', timestep='0.002', integrator='implicitfast')
    ET.SubElement(root, 'default')
    wb = ET.SubElement(root, 'worldbody')
    wb.append(deck_root)
    acts = ET.SubElement(root, 'actuator')
    # The deck's own actuators travel with it. They are identified from the built world rather
    # than listed by hand, so adding a roller cannot silently leave its actuator behind.
    src = ET.fromstring(role_path(next(r for r in ROLES if r.key == 'c'))
                        .read_text(encoding='utf-8'))
    wanted = set((deck_root.iter('joint')))
    names = {j.get('name') for j in wanted if j.get('name')}
    for act in (src.find('actuator') if src.find('actuator') is not None else []):
        if act.get('joint') in names:
            acts.append(act)
    spec = mujoco.MjSpec.from_string(ET.tostring(root, encoding='unicode'))
    spec.meshdir = ''
    # The default CLASS NAME is what collides when two specs derived from the same XML are
    # attached: MuJoCo refuses `repeated default class name`. `spec.defaults` does not exist in
    # this build, but `spec.default.name` is writable (measured: 'main' -> 'c_deck_main'), so
    # that is where the rename happens -- NOT in the XML, where a top-level `<default name=...>`
    # is a schema violation (measured: 'unrecognized attribute: name').
    spec.default.name = 'c_deck_main'
    return spec, report, sorted(names)


def load_c_world_spec():
    """The part of C that stays on the world: fixed sections + receiving row + cargo.

    Split out of `load_role_spec` so that the deck is attached exactly once, to the chassis.
    Attaching `load_role_spec('c')` as well would put a SECOND deck at C's station and the
    world would compile with two decks -- which is the failure this split exists to prevent,
    so it is asserted rather than assumed (see the gate's deck-count check).
    """
    import xml.etree.ElementTree as ET
    _, world_fixed_xml, _ = decompose_c_xml()
    spec = mujoco.MjSpec.from_string(world_fixed_xml)
    for mesh in spec.meshes:
        mesh.file = rebase(mesh.file, spec.meshdir)
    spec.meshdir = ''
    for geom in list(spec.worldbody.geoms):
        if geom.name in ('floor',):
            spec.delete(geom)
    for key in list(spec.keys):
        spec.delete(key)
    return spec


def load_role_spec(role, *, strip=True):
    """The role as a spec, ready to attach.

    `strip=False` keeps the furniture and the keyframes; that is only used for measuring the
    role's standalone extent, which must include everything it brings.
    """
    path = role_path(role)
    spec = mujoco.MjSpec.from_file(str(path))
    for mesh in spec.meshes:
        mesh.file = rebase(mesh.file, spec.meshdir)
    for texture in spec.textures:
        texture.file = rebase(getattr(texture, 'file', '') or '', spec.meshdir)
    # The child's meshdir must be cleared as well as its paths rebased. `attach` copies the
    # child's meshdir onto the parent, so a child left at `panda/assets` made the merged model
    # look for assets/panda/assets/panda/assets/link0.stl -- the path was rebased and then
    # prefixed again. Measured, not guessed.
    spec.meshdir = ''
    if not strip:
        return spec
    for geom in list(spec.worldbody.geoms):
        if geom.name in role.furniture:
            spec.delete(geom)
    for key in list(spec.keys):
        spec.delete(key)
    return spec


def role_extent(role):
    """Measured (lo, hi) of a role's OWN geoms in its own frame, furniture excluded.

    Taken from a compiled model with a forward pass rather than from the XML, so rotating
    joints and rbound inflation are included: a footprint read off the source text would
    miss the arm's reach.
    """
    path = role_path(role)
    model = mujoco.MjModel.from_xml_path(str(path))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    keep = [g for g in range(model.ngeom)
            if (mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, g) or '')
            not in role.furniture]
    if not keep:
        raise ValueError(f'{role.key}: no role geoms once furniture is dropped')
    lo = np.array([data.geom_xpos[g] - model.geom_rbound[g] for g in keep])
    hi = np.array([data.geom_xpos[g] + model.geom_rbound[g] for g in keep])
    return lo.min(axis=0), hi.max(axis=0)


def station_layout():
    """Station (x, y) per role, derived from measured footprints and ROLE_CLEARANCE.

    Each role is shifted so that its own lowest x lands one clearance to the right of the
    previous role's highest x. All stations share y = 0, so the cell is a corridor whose width
    is set by the widest role.
    """
    layout, cursor = {}, 0.0
    for role in ROLES:
        lo, hi = role_extent(role)
        x = cursor + ROLE_CLEARANCE - float(lo[0])
        layout[role.key] = (x, 0.0)
        cursor = x + float(hi[0])
    layout['_cursor'] = cursor
    return layout


def cell_bounds(layout):
    """The merged cell's rectangle, from the layout plus the declared wall margin."""
    lo_x = 0.0
    hi_x = layout['_cursor']
    half_y = 0.0
    for role in ROLES:
        x, _ = layout[role.key]
        lo, hi = role_extent(role)
        half_y = max(half_y, abs(float(lo[1])), abs(float(hi[1])))
    return (lo_x - CELL_WALL_MARGIN, hi_x + CELL_WALL_MARGIN,
            -(half_y + CELL_WALL_MARGIN), half_y + CELL_WALL_MARGIN)


def ground_friction_sources():
    """The ground friction each source declared, so the choice above is auditable."""
    out = {}
    for role in ROLES:
        spec = mujoco.MjSpec.from_file(str(role_path(role)))
        for geom in spec.worldbody.geoms:
            if geom.name == 'floor':
                out[role.key] = (round(float(geom.friction[0]), 4), int(geom.condim))
    return out


def sources_agree_on_stepping():
    """Every source's timestep and integrator. The gate asserts they already agree."""
    out = {}
    for role in ROLES:
        model = mujoco.MjModel.from_xml_path(str(role_path(role)))
        out[role.key] = (round(float(model.opt.timestep), 6), int(model.opt.integrator))
    return out


def joint_key(model, j, strip=None):
    """Stable identity for a joint across the merge, robust to joints with no name.

    A joint is identified by (body name, joint name). The body name is what makes an UNNAMED
    joint addressable at all: the carried-over 007 snapshot gives the humanoid's floating base
    no joint name, so matching on names alone silently skipped it and left its qpos at zero --
    including a zero quaternion. The index is the joint's position among its own body's joints,
    which `attach` preserves.
    """
    body = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, int(model.jnt_bodyid[j])) or ''
    if strip and body.startswith(strip):
        body = body[len(strip):]
    name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, j)
    if name:
        # The prefix applies to JOINT names as well as body names. Stripping it from the body
        # only left the merged key as ('link0', 'a_joint1') while the source key was
        # ('link0', 'joint1'), so nothing matched and the build reported
        # "0 joint overrides applied" -- a lookup that failed silently into "no declarations".
        if strip and name.startswith(strip):
            name = name[len(strip):]
        return (body, name)
    k = sum(1 for jj in range(j)
            if int(model.jnt_bodyid[jj]) == int(model.jnt_bodyid[j]))
    return (body, f'#k{k}')


#: qpos width per joint type: free 7, ball 4, slide/hinge 1.
QPOS_WIDTH = {
    int(mujoco.mjtJoint.mjJNT_FREE): 7,
    int(mujoco.mjtJoint.mjJNT_BALL): 4,
    int(mujoco.mjtJoint.mjJNT_SLIDE): 1,
    int(mujoco.mjtJoint.mjJNT_HINGE): 1,
}


def stand_law():
    """The humanoid's declared standing law: (joint names, target angles, kp, kd).

    Read from the role's own `config/t800/stand.yaml` and `config/t800/model.yaml`, which is
    what `humanoid007.runtime` reads too. This is needed because the carried-over snapshot's 25
    limb actuators are PURE TORQUE (gain type fixed, bias type none): `ctrl` is a torque, so
    there is no position-servo "hold" for them, and zero torque drops the humanoid 1.58 m in
    three seconds.
    """
    stand = yaml.safe_load((ROOT / 'config' / 't800' / 'stand.yaml').read_text())
    model_cfg = yaml.safe_load((ROOT / 'config' / 't800' / 'model.yaml').read_text())
    names = [j for limb in model_cfg['limbs'] for j in limb['joints']]
    target = np.concatenate(stand['desired_joint_position'])
    kp = np.concatenate(stand['stiffness'])
    kd = np.concatenate(stand['damping'])
    if not len(names) == len(target) == len(kp) == len(kd):
        raise ValueError(f'stand.yaml and model.yaml disagree on the joint count: '
                         f'{len(names)} names, {len(target)} targets, {len(kp)} kp, {len(kd)} kd')
    return names, target, kp, kd


def declared_pose(role):
    """What this role's state is beyond its own XML, keyed by `joint_key`.

    Returns (qpos overrides, per-actuator ctrl, notes). Two declarations exist:
      * a keyframe, which is a full declared state (A has one);
      * for H, the walking policy's default pose on the 25 limb joints and `OPEN` on the 32
        hand joints -- `humanoid007.runtime` does not start from `qpos0`, it writes
        `policy.default` and `OPEN`, so starting the humanoid from `qpos0` would be starting it
        somewhere its own runtime never is.
    """
    model = mujoco.MjModel.from_xml_path(str(role_path(role)))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    over, ctrl, notes = {}, {}, []
    if model.nkey > 0:
        q = np.array(model.key_qpos[0], dtype=float)
        c = np.array(model.key_ctrl[0], dtype=float)
        for j in range(model.njnt):
            adr = int(model.jnt_qposadr[j])
            over[joint_key(model, j)] = q[adr:adr + QPOS_WIDTH[int(model.jnt_type[j])]]
        for a in range(model.nu):
            ctrl[mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, a)] = float(c[a])
        notes.append('keyframe')
    if role.key == 'h':
        sys.path.insert(0, str(ROOT / 'src'))
        from humanoid007 import runtime as h_runtime
        walking = yaml.safe_load((ROOT / 'config' / 't800' / 'walking.yaml').read_text())
        names, target, _, _ = stand_law()
        default = np.concatenate(walking['default_joint_q'])
        if len(names) != len(default):
            raise ValueError(f'policy joint list {len(names)} != default pose {len(default)}')
        for name, value in zip(names, default):
            over[joint_key(model, model.joint(name).id)] = np.array([float(value)])
        hands = [f'{side}_{part}a{i}' for side in ('lh', 'rh')
                 for part in ('ff', 'mf', 'rf', 'th') for i in range(4)]
        for name, value in zip(hands, np.tile(np.asarray(h_runtime.OPEN, dtype=float), 2)):
            act = model.actuator(name).id
            jid = int(model.actuator_trnid[act, 0])
            over[joint_key(model, jid)] = np.array([float(value)])
        notes.append(f'policy default on {len(names)} limb joints + OPEN on {len(hands)} hands, '
                     f'from humanoid007.runtime')
    return over, ctrl, notes


#: model id -> (model, {joint id: [actuator indices]}). The model reference is part of the value
#: so that the id cannot be reused after a collection while a stale entry survives.
_STAND_LAW_ACTUATORS = {}


def _stand_actuator_index(model):
    """Which actuators drive the humanoid's limb joints, found once per model.

    Cached because `home_hold_ctrl` runs every control step and the uncached form rescanned all
    `nu` actuators once per joint. Measured: RTF 0.209 on the logistics world before this, i.e.
    5x slower than real time, which is the difference between a sweep being 4 minutes and 40.
    """
    cached = _STAND_LAW_ACTUATORS.get(id(model))
    if cached is not None and cached[0] is model:
        return cached[1]
    index = {}
    for actuator in range(model.nu):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, actuator) or ''
        if not name.startswith('h_'):
            continue
        index.setdefault(int(model.actuator_trnid[actuator, 0]), []).append(actuator)
    _STAND_LAW_ACTUATORS[id(model)] = (model, index)
    return index


def home_hold_ctrl(model, home_qpos, qpos, qvel):
    """The command that holds each role at its declared home state, for one control step.

    One rule in one place, used both for the merged keyframe's `ctrl` (evaluated at the home
    state) and by the gate's settle test (evaluated every step). It is derived from the
    COMPILED actuator properties, not typed:

      * a position servo (`biastype` affine with `biasprm[1] != 0`, i.e. -kp) on a slide or
        hinge holds the joint at its declared home angle;
      * the humanoid's torque actuators follow its own stand law, because a torque actuator has
        no hold position at all;
      * everything else -- velocity servos, raw motors -- holds zero, which for a roller or a
        wheel means "stopped" and is what the role's own runs command at rest.
    """
    ctrl = np.zeros(model.nu, dtype=float)
    affine = int(mujoco.mjtBias.mjBIAS_AFFINE)
    joint_trn = int(mujoco.mjtTrn.mjTRN_JOINT)
    for a in range(model.nu):
        if int(model.actuator_biastype[a]) != affine:
            continue
        if float(model.actuator_biasprm[a][1]) == 0.0:
            continue                                  # not a position servo
        if int(model.actuator_trntype[a]) != joint_trn:
            continue                                  # tendon servos keep their declared value
        jid = int(model.actuator_trnid[a, 0])
        if int(model.jnt_type[jid]) not in (2, 3):
            continue
        adr = int(model.jnt_qposadr[jid])
        value = float(home_qpos[adr])
        if int(model.actuator_ctrllimited[a]):
            lo, hi = (float(v) for v in model.actuator_ctrlrange[a])
            value = min(max(value, lo), hi)
        ctrl[a] = value

    names, target, kp, kd = stand_law()
    index = _stand_actuator_index(model)
    for k, name in enumerate(names):
        jid = model.joint('h_' + name).id
        if jid < 0:
            continue
        adr, dof = int(model.jnt_qposadr[jid]), int(model.jnt_dofadr[jid])
        torque = (float(target[k]) - float(qpos[adr])) * float(kp[k]) - float(qvel[dof]) * float(kd[k])
        for a in index.get(int(jid), ()):
            ctrl[a] = torque
    return ctrl


def home_pose(role):
    """Deprecated shim kept so nothing imports a name that no longer exists."""
    raise NotImplementedError('use declared_pose() instead')


def with_option_block(xml):
    """Inject the merged world's `<option>` element and return the XML.

    `MjSpec` has no option block in this build, so the stepping settings cannot travel through
    `to_xml()`. Injecting text would be a bad habit on its own; what makes it acceptable here
    is that `build_text` immediately recompiles the result and asserts the compiled model's
    timestep and integrator are exactly `MERGED_TIMESTEP` / `MERGED_INTEGRATOR`, and that the
    compiler really did read the angles as radians.
    """
    marker = '/>'
    start = xml.find('<compiler')
    if start < 0:
        raise ValueError('no <compiler> element to anchor the option block to')
    end = xml.find(marker, start) + len(marker)
    if '<option' in xml:
        return xml
    option = (f'\n  <option timestep="{MERGED_TIMESTEP}" '
              f'integrator="{MERGED_INTEGRATOR}"/>')
    return xml[:end] + option + xml[end:]


def verify_stepping(xml):
    """Compile the emitted XML and confirm the option block and the angle unit took effect.

    This is the check that stops the injected `<option>` from being decoration: if the element
    is dropped, misspelled, or placed where it is ignored, the timestep or the integrator will
    not match and the build fails here rather than producing a world that runs at the wrong dt.

    The scratch file is written BESIDE the merged world rather than into a temp directory:
    mesh paths are relative to the XML file's own directory, so a copy in /tmp cannot be
    compiled at all. It is removed in a `finally`, including on the `--check` path.
    """
    scratch = ASSETS / '_merged_syntax_check.xml'
    scratch.write_text(xml, encoding='utf-8')
    try:
        model = mujoco.MjModel.from_xml_path(str(scratch))
    finally:
        scratch.unlink()
    got = (round(float(model.opt.timestep), 6), int(model.opt.integrator))
    want = (MERGED_TIMESTEP, int(mujoco.mjtIntegrator.mjINT_IMPLICITFAST))
    if got != want:
        raise ValueError(f'merged world steps at {got}, expected {want}: the option block '
                         f'did not take effect')
    if 'angle="radian"' not in xml:
        raise ValueError('the emitted compiler element does not declare radians')
    return got

    # NOTE: an earlier version of this check compared the widest joint range against 2*pi to
    # "detect degrees". It was backwards -- a degree misreading makes the stored ranges SMALLER,
    # not larger -- so it could never have fired. The unit is now checked by comparison against
    # the sources in `verify_kinematics`, which is the check that can actually fail.


def verify_kinematics(xml, layout):
    """Every joint range and every body pose in the merged world, against its own source.

    This is the check that makes "the merge did not change the mechanisms" a measurement rather
    than a hope. It compares, BY NAME and for every role:

      * each hinge/slide joint's `range` and `axis`;
      * each body's local `pos` and `quat` -- identical for nested bodies, and offset by the
        role's station for bodies whose parent is the world.

    It catches three whole classes of silent merge damage at once: an angle unit read as
    degrees (every range would be scaled by pi/180), a `eulerseq` change (every euler-declared
    orientation would rotate differently -- the H source declares `eulerseq="zyx"` and the
    emitted merged compiler element has only `angle="radian"`), and a lost or reordered body.

    Returns (joints_checked, bodies_checked, worst) where `worst` is the largest discrepancy
    seen, so the build can print the margin it actually achieved.
    """
    scratch = ASSETS / '_merged_kin_check.xml'
    scratch.write_text(xml, encoding='utf-8')
    try:
        merged = mujoco.MjModel.from_xml_path(str(scratch))
    finally:
        scratch.unlink()

    def q(m, obj, i):
        return [round(float(v), 12) for v in m.body_quat[i]]

    joints = bodies = 0
    unnamed = []
    relocated = []
    # DERIVED, not typed: the bodies this build moved out of c are `DECK_SUBTREE_PREFIX` under
    # `c_`, resolved from the compiled merged model. An earlier version spelled the set out as
    # `{'deck'} | {f'deck_roller_{i}' ...} | {'pusher_carriage', 'pusher_blade'}`, which silently
    # invented a constant name and made the build un-runnable -- the exact "typed constant that
    # cannot notice it is wrong" failure this project keeps re-finding.
    relocated_c_bodies = set()
    {
        relocated_c_bodies.add(nm[len('c_'):])
        for nm in (
            mujoco.mj_id2name(merged, mujoco.mjtObj.mjOBJ_BODY, b)
            for b in range(merged.nbody)
        )
        if nm and nm.startswith('c_' + DECK_BODY)
    }
    worst, worst_where = 0.0, ''

    def excess(got, want):
        """The part of a discrepancy that is NOT the model's float32 storage floor.

        MuJoCo stores `body_pos`/`jnt_range`/`axis`/`quat` as FLOAT32 in the compiled model, and
        the merged world adds the role's station (up to ~9 m) to C's positions before storing
        them. The round-trip of `4.413720` through float32 is ~3.9e-06, so a plain `abs(got -
        want)` reports 3.94e-06 on bodies that are placed exactly right -- a number that reads
        like damage and is not.

        Subtracting the storage floor keeps this check's output meaning "the merge changed
        something". It is DERIVED from `np.spacing` at the actual magnitudes, not a typed
        epsilon, and it is the same for every row, so a real error of 1e-3 still reports ~1e-3.
        """
        floor = max(float(np.spacing(np.float32(want))),
                    float(np.spacing(np.float32(got))),
                    float(np.spacing(np.float32(want + 1e-9))))
        return max(0.0, abs(got - want) - 4.0 * floor)

    for role in ROLES:
        src = mujoco.MjModel.from_xml_path(str(role_path(role)))
        station = layout[role.key][0]
        for j in range(src.njnt):
            name = mujoco.mj_id2name(src, mujoco.mjtObj.mjOBJ_JOINT, j)
            if name is None:
                # An unnamed joint cannot be addressed BY NAME, which is the only addressing
                # this project allows. H's floating base is one: its free joint has no name in
                # the carried-over 007 snapshot, so the humanoid's base is reachable only by
                # index. Counted and reported rather than skipped silently.
                unnamed.append(f'{role.key}:joint[{j}]')
                continue
            mj = merged.joint(role.prefix + name)
            if mj.id < 0:
                raise ValueError(f'{role.key}: joint {name!r} is missing from the merged world')
            joints += 1
            if int(src.jnt_type[j]) in (2, 3):        # slide / hinge
                for got, want in zip([float(v) for v in mj.range],
                                     [float(v) for v in src.jnt_range[j]]):
                    gap = excess(got, want)
                    if gap > worst:
                        worst, worst_where = gap, f'{role.key}:{name}.range'
            for got, want in zip([float(v) for v in mj.axis],
                                 [float(v) for v in src.jnt_axis[j]]):
                gap = excess(got, want)
                if gap > worst:
                    worst, worst_where = gap, f'{role.key}:{name}.axis'
        for b in range(src.nbody):
            name = mujoco.mj_id2name(src, mujoco.mjtObj.mjOBJ_BODY, b)
            if name is None or b == 0:
                continue
            mb = merged.body(role.prefix + name)
            if mb.id < 0:
                raise ValueError(f'{role.key}: body {name!r} is missing from the merged world')
            bodies += 1
            # V1 ASSEMBLY: bodies the merge deliberately moved under another role are not
            # compared locally -- their local pos was rewritten on purpose. Their WORLD pose is
            # what has to be preserved, and the gate's `check_rig_reproduced` measures that to
            # 0.0 m. Reporting them in this max() would only hide real damage behind a number
            # that has no meaning for them.
            if role.key == 'c' and name in relocated_c_bodies:
                relocated.append(f'{role.key}:{name}')
                continue
            # nested bodies keep their local pose; world children move by the station
            shift = station if int(src.body_parentid[b]) == 0 else 0.0
            want_pos = [float(src.body_pos[b][0]) + shift, float(src.body_pos[b][1]),
                        float(src.body_pos[b][2])]
            for got, want in zip(list(mb.pos), want_pos):
                gap = excess(got, want)
                if gap > worst:
                    worst, worst_where = gap, f'{role.key}:{name}.pos'
            for got, want in zip(q(merged, mujoco.mjtObj.mjOBJ_BODY, mb.id),
                                 q(src, mujoco.mjtObj.mjOBJ_BODY, b)):
                gap = excess(got, want)
                if gap > worst:
                    worst, worst_where = gap, f'{role.key}:{name}.quat'
    # `relocated` is returned so the build can print WHICH bodies it declined to compare,
    # rather than silently checking one fewer thing. A skip nobody can see is a silent skip.
    return joints, bodies, worst, worst_where, unnamed, relocated


def name_base_free_joint(spec):
    """Give the humanoid's base free joint a name. Returns the merged joint's name.

    The joint arrives unnamed (see HUMANOID_BASE_BODY). An unnamed joint cannot be addressed
    by name, and the only handle left is its parent body -- which is exactly the indirection
    that let a real bug hide: home assembly could not match the joint, so it was skipped, and
    the humanoid started at the origin with a zero quaternion.

    Idempotent by design: if a future source names the joint itself, that name is kept.
    """
    role = next(r for r in ROLES if r.key == 'h')
    body_name = f'{role.prefix}{HUMANOID_BASE_BODY}'
    try:
        body = spec.body(body_name)
    except Exception as exc:
        raise RuntimeError(f'the humanoid base body {body_name!r} is missing from the merged '
                           f'spec ({type(exc).__name__})') from exc
    if body is None:
        raise RuntimeError(f'the humanoid base body {body_name!r} is missing from the merged spec')
    free = [j for j in body.joints if j.type == mujoco.mjtJoint.mjJNT_FREE]
    if len(free) != 1:
        raise RuntimeError(f'{body_name} carries {len(free)} free joints, expected exactly one')
    if not free[0].name:
        free[0].name = f'{role.prefix}{BASE_FREE_JOINT}'
    return free[0].name


def build_spec():
    """The merged spec: shared ground + one station per role + the roles attached."""
    layout = station_layout()
    # N's own compiled model, kept for DERIVING the mount offset rather than typing it.
    n_model = mujoco.MjModel.from_xml_path(str(role_path(next(r for r in ROLES if r.key == 'n'))))
    lo_x, hi_x, lo_y, hi_y = cell_bounds(layout)

    spec = mujoco.MjSpec()
    spec.modelname = '010 P1 common world: H humanoid + A fixed arm + C deck + N chassis'
    # `MjsCompiler` in this build exposes `degree`, not `angle`, and `MjSpec` exposes no
    # option block at all -- so the timestep and integrator cannot be set through the spec API
    # and are injected into the serialised XML instead (see `with_option_block`). Both are then
    # read back off the COMPILED model and asserted, because an injected attribute that nothing
    # checks is just text.
    # `meshdir` is not settable on MjsCompiler in this build either. That is fine and is why
    # the merged world is written into `assets/`: mesh paths resolve against the XML file's own
    # directory, so every path rebased to the parent's meshdir ('' here) resolves unchanged.
    spec.compiler.degree = False
    spec.compiler.autolimits = True

    cx, cy = (lo_x + hi_x) / 2.0, (lo_y + hi_y) / 2.0
    ground = spec.worldbody.add_geom(name='cell_ground', type=mujoco.mjtGeom.mjGEOM_PLANE,
                                    pos=[cx, cy, 0.0],
                                    size=[(hi_x - lo_x) / 2.0, (hi_y - lo_y) / 2.0, 0.1],
                                    condim=SHARED_GROUND_CONDIM,
                                    friction=[SHARED_GROUND_FRICTION, 0.005, 0.0001])
    ground.contype = 1
    ground.conaffinity = 1

    # The boundary is generated from the layout, so it grows with the stations rather than
    # being a number that has to be remembered.
    half_x = (hi_x - lo_x) / 2.0
    half_y = (hi_y - lo_y) / 2.0
    # TWO BUGS LIVED HERE UNTIL 2026-09-23, and neither was visible to any check this project
    # had. The four (offset_x, offset_y) pairs were wrong -- north and south were pushed to the
    # +x EDGE and east and west both to the +y EDGE, instead of being centred on their own side
    # -- and the orientation was picked by `ax >= ay`, which compares two POSITION OFFSETS
    # rather than asking which axis the wall runs along.
    #
    # Measured on the artefact that was produced: the WEST side had 0.000 m of wall, the SOUTH
    # 0.000 m, the NORTH 5.544 m of 11.039 m (50.2%), and the east cap was covered twice. The
    # `cell` was a floor slab with a half fence, and the names disagreed with the geometry
    # (`cell_wall_north` was the east cap). Everything else -- naming, TF, clock, isolation,
    # settling, contact -- passed the whole time, because every one of those rows counted the
    # walls or asked what they were called. None of them asked whether the room was CLOSED.
    #
    # `evaluate_p3_world.py` now has that row, with a probe that opens a wall.
    for name, along_x, off in (('cell_wall_north', True, (0.0, half_y)),
                               ('cell_wall_south', True, (0.0, -half_y)),
                               ('cell_wall_east', False, (half_x, 0.0)),
                               ('cell_wall_west', False, (-half_x, 0.0))):
        size = ([half_x, CELL_WALL_THICKNESS / 2.0, CELL_WALL_HEIGHT / 2.0] if along_x
                else [CELL_WALL_THICKNESS / 2.0, half_y, CELL_WALL_HEIGHT / 2.0])
        wall = spec.worldbody.add_geom(name=name, type=mujoco.mjtGeom.mjGEOM_BOX,
                                       pos=[cx + off[0], cy + off[1], CELL_WALL_HEIGHT / 2.0],
                                       size=size)
        wall.contype = 1
        wall.conaffinity = 1

    anchors = {}
    for role in ROLES:
        x, y = layout[role.key]
        anchors[role.key] = spec.worldbody.add_site(
            name=f'{role.key}_station', type=mujoco.mjtGeom.mjGEOM_SPHERE,
            pos=[x, y, 0.0], size=[0.001, 0.001, 0.001])
        if role.key == 'c':
            # C ARRIVES IN TWO PIECES, and that is the assembly.
            #
            # `load_role_spec('c')` would attach the whole rig -- fixture included -- at C's
            # station. That is what this world used to do, and it is why C's deck was
            # world-welded. The assembly replaces it with:
            #   1. the world-fixed remainder (fixed sections + receiving row + cargo) at C's
            #      station, exactly as before;
            #   2. the deck subtree alone, attached to N's chassis root.
            # N is attached first is NOT required -- `attach` lands the deck at its declared
            # ABSOLUTE world pose whatever the parent is (measured: parent `n_base_link`,
            # `body_pos` [1.92, 0, 0.485] unchanged), so the deck's station is still what places
            # it and the chassis under it must be parked to match. That parking is `merged_home`'s
            # job and it is the thing that makes `dock_offset` zero.
            spec.attach(load_c_world_spec(), prefix='c_', site=anchors[role.key])
            continue
        spec.attach(load_role_spec(role), prefix=role.prefix, site=anchors[role.key])

    # --- the deck goes onto the chassis, not onto the world ---------------------------
    deck_spec, deck_report = load_deck_spec()[0], load_deck_spec()[1]
    chassis = spec.body(f'n_{CHASSIS_MOUNT_BODY}')
    if chassis is None:
        raise RuntimeError(f'N has no body {CHASSIS_MOUNT_BODY!r} to mount the deck on; the '
                           f'assembly cannot be skipped silently')
    # THE MOUNT SITE CARRIES THE CHASSIS ROOT'S OWN OFFSET, negated.
    #
    # `attach(child, site=s)` re-expresses the child so the site lands on the parent's origin;
    # the child keeps its body-local geometry. So a site at (0,0,0) on a body that itself sits at
    # z = +0.040 raises the whole deck by 0.040 m. Measured before this fix: the deck's roller
    # crowns sat at 0.525 while the fixture's sat at 0.485 -- exactly one chassis-root offset,
    # enough to break the crown plane the tray transfers along.
    #
    # The offset is READ OFF N's SOURCE, not typed, and it is negated so the deck returns to the
    # world pose it declares. This is the same class of defect as the free-joint absolute pose:
    # a quantity expressed in the parent's frame being read as if it were a world quantity.
    n_data = mujoco.MjData(n_model)
    mujoco.mj_forward(n_model, n_data)
    _root_bid = mujoco.mj_name2id(n_model, mujoco.mjtObj.mjOBJ_BODY, CHASSIS_MOUNT_BODY)
    if _root_bid < 0:
        raise RuntimeError(f'{CHASSIS_MOUNT_BODY!r} is missing from N, so the mount offset '
                           f'cannot be derived and the assembly must not guess it')
    mount_offset = -np.asarray(n_data.xpos[_root_bid], dtype=float)
    ASSEMBLY_REPORT['mount_offset_m'] = [round(float(v), 6) for v in mount_offset]
    mount = chassis.add_site(name='c_deck_mount', type=mujoco.mjtGeom.mjGEOM_SPHERE,
                             pos=[float(v) for v in mount_offset],
                             size=[0.001, 0.001, 0.001])
    # NOTE on `frame=`: the earlier note in this project that `frame=` is unavailable in this
    # build is WRONG and is corrected here -- measured, both `site=` and `frame=` work, and
    # `frame=pos=[...]` offsets the child exactly as written. `site=` is used because the mount
    # is a point on the chassis, and a site is the thing a reader can find by name later.
    spec.attach(deck_spec, prefix='c_', site=mount)
    ASSEMBLY_REPORT.update(deck_report)
    ASSEMBLY_REPORT['mount_parent'] = f'n_{CHASSIS_MOUNT_BODY}'
    name_base_free_joint(spec)
    # Belt and braces: whichever child was attached last may have left its meshdir behind, so
    # the merged spec's is pinned to '' -- the directory the merged world itself lives in.
    spec.meshdir = ''
    return spec, layout


def _deck_standalone_x0():
    """`DECK_X0` as the rig itself declares it. Read from the rig, never typed here.

    A typed copy of a derived constant is the exact defect this project keeps re-finding, so the
    number is imported from the single source of truth and the gate re-reads it on every run.
    """
    sys.path.insert(0, str(ROOT / 'experiments'))
    import roller_rig as rig
    return float(rig.DECK_X0)


def _deck_source_x():
    """The deck root's own x in C's source, i.e. `DECK_X0` before any station shift."""
    return _deck_standalone_x0()


def deck_world_pos_at(model, qpos):
    """The deck's full world position for a given qpos, without disturbing the caller's data.

    Full position, not just x: the mount offset is a 3-vector and a version that only solved x
    would leave a y displacement uncorrected, where it would surface much later as an alignment
    error with no obvious cause.
    """
    data = mujoco.MjData(model)
    data.qpos[:] = np.asarray(qpos, dtype=float)
    mujoco.mj_forward(model, data)
    bid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, 'c_deck')
    if bid < 0:
        raise RuntimeError('c_deck is missing from the merged model; the deck was not assembled')
    return np.array(data.xpos[bid], dtype=float)


def merged_home(model, layout=None):
    """The merged home qpos and ctrl, assembled BY NAME from each role's own declaration.

    The starting point is the MERGED model's own `qpos0`, not each source's. That is the
    correction of a measured defect, not a shortcut: a free joint's qpos is an ABSOLUTE world
    pose, so a source's value is expressed in that source's frame and must be translated by the
    role's station. Copying the sources' values straight in put every payload back at the world
    origin (H's tray at x 0.23, C's at 0.2, A's at 0.0) and, because the humanoid's base free
    joint has NO NAME, skipped it entirely and left it at all zeros -- including a zero
    quaternion. Measured consequence of that first version: 220 contacts at the home state with
    the humanoid's feet 1.0165 m inside the ground, and role A's arm unloading 1.50 rad from a
    servo that was holding perfectly in its own world.
    `attach` has already placed every body at its station, so `qpos0` is already coherent; the
    role declarations are applied on top of it as OVERRIDES.

    Returns (qpos, ctrl, applied, notes).
    """
    layout = layout if layout is not None else station_layout()
    qpos = np.array(model.qpos0, dtype=float)
    declarations, notes, applied, free_disagreements = {}, [], 0, []

    for role in ROLES:
        over, declared_ctrl, note = declared_pose(role)
        declarations[role.key] = declared_ctrl
        notes.append((role.key, ', '.join(note) or 'qpos0 only', len(over)))
        station = layout[role.key]
        for j in range(model.njnt):
            key = joint_key(model, j, strip=role.prefix)
            if key not in over:
                continue
            free = int(model.jnt_type[j]) == int(mujoco.mjtJoint.mjJNT_FREE)
            adr = int(model.jnt_qposadr[j])
            width = QPOS_WIDTH[int(model.jnt_type[j])]
            value = np.array(over[key], dtype=float)
            if free:
                # A FREE JOINT KEEPS THE MODEL'S OWN REFERENCE POSE.
                #
                # Measured reason: A's `home` keyframe is a keyframe of the ARM. It sets
                # `payload_free` to (0, 0, 0, 1, 0, 0, 0) -- a valid unit quaternion at the
                # origin -- while `qpos0` puts the same part on the table at
                # (0.55, -0.13, 0.225). Applying the keyframe therefore buried the part in the
                # ground and inside `link0` (measured: 4 contacts at `world <-> a_payload` of
                # -0.025 m and one `a_payload <-> a_link0` of -0.02502 m, present for the whole
                # run). The A probe never read that value either; it placed the part from its
                # own `PICK_XY`.
                #
                # So for a free body the model's own body/joint definition wins, and a
                # declaration that disagrees is REPORTED rather than obeyed: it means the
                # source keyframe is not a complete state of its own scene.
                reference = np.array(model.qpos0[adr:adr + width], dtype=float)
                shifted = value.copy()
                shifted[0] += float(station[0])
                shifted[1] += float(station[1])
                if not np.allclose(shifted, reference, rtol=0, atol=1e-9):
                    free_disagreements.append(
                        (role.key, key[0], key[1],
                         [round(float(v), 5) for v in shifted[:3]],
                         [round(float(v), 5) for v in reference[:3]]))
                continue
            if free:
                value = value.copy()
            qpos[adr:adr + width] = value
            applied += 1

    # --- the assembly parking, AFTER the declarations ---------------------------------
    #
    # THIS BLOCK MUST COME AFTER THE ROLE-OVERRIDE LOOP. It was written before it first, and the
    # loop then wrote N's own declared `qpos0` (zero) straight back over `n_slide_x` -- so the
    # parking was computed, correct, and erased, and the only symptom was a 4.41372 m
    # `dock_offset` that reads like a derivation error rather than an ordering error.
    # --- the assembly parking ---------------------------------------------------------
    #
    # WHERE THE CHASSIS HAS TO STAND, DERIVED. C's deck and the receiving row are two halves of
    # one rig whose spacing was a DESIGN CONSTANT while the deck was welded to the world
    # (`RECV_FIRST_CROWN_X = DECK_LAST_CROWN_X + 2r + CROWN_CLEARANCE`). Bolting the deck to the
    # chassis turns that constant into `(where the AMR parked) - (where the receiver is)`.
    #
    # This block chooses the parking that makes the two agree, and then REPORTS the residual
    # instead of assuming it is zero. It is a placement, not a drive: commanding the robot to
    # the dock would make C's "the fixture is fixed and the deck travels out to meet it" claim
    # circular, because the thing under test would have produced its own initial condition.
    #
    # The derivation is a difference of two world positions, both read off the compiled model:
    #   want  = where C's own station puts the deck      (`c_station` + deck's source x)
    #   have  = where the chassis currently puts it
    #   shift = want - have, applied as `slide_x` on the chassis root
    # (`slide_x` is a plain slide joint on `base_link`, so adding to its qpos is a rigid world
    # translation of the whole vehicle -- deck, wheels and all.)
    # SOLVED IN THE WORLD FRAME. The first version computed `want` from C's station and `have`
    # from the same expression, so the two agreed by construction and the shift came out ~0 --
    # while the deck was actually 4.41 m from where the rig needs it. Measuring both numbers
    # against the COMPILED world is what makes this a measurement instead of an identity.
    station_c = np.array(layout['c'], dtype=float)
    want = np.array([float(station_c[0]) + _deck_standalone_x0(), float(station_c[1]), 0.0])
    # ITERATED TO A FIXED POINT, and the residual is REPORTED.
    #
    # `deck_world_pos_at` reads the compiled model's kinematics, which is exact to float32 (the
    # model stores its numbers as float32), so a single shift leaves a residual of order 1e-6 m
    # -- measured 3.94e-06 before this loop existed. That is far below any physical tolerance,
    # but it is NOT below the gate's rig-fidelity tolerance, and the honest fix is to converge
    # rather than to loosen the gate: 1e-9 is what makes "the assembled rig reproduces C's own
    # numbers" a statement about the assembly instead of about the solver.
    parking = None
    residual = None
    for _ in range(8):
        have = deck_world_pos_at(model, qpos)
        shift = want - have
        residual = [round(float(v), 12) for v in shift]
        if max(abs(float(shift[0])), abs(float(shift[1]))) <= 1e-12:
            break
        # x AND y. A mount offset that displaces the deck in y would otherwise be left
        # uncorrected and would only show up later as a mysterious lateral error in C.
        for axis, joint in (('x', 'n_slide_x'), ('y', 'n_slide_y')):
            idx = 0 if axis == 'x' else 1
            if abs(float(shift[idx])) <= 1e-12:
                continue
            jid = model.joint(joint).id
            if jid < 0:
                raise RuntimeError(f'the chassis has no {joint!r} joint to park with; the '
                                   f'assembly cannot place the deck and must not pretend it did')
            adr = int(model.jnt_qposadr[jid])
            qpos[adr] = float(qpos[adr]) + float(shift[idx])
            parking = dict(parking or {})
            parking[joint] = float(parking.get(joint, 0.0)) + float(shift[idx])
    ASSEMBLY_REPORT['parking'] = parking
    ASSEMBLY_REPORT['deck_world_pos_want'] = [round(float(v), 6) for v in want]
    ASSEMBLY_REPORT['deck_world_pos_as_built'] = [round(float(v), 9)
                                                  for v in deck_world_pos_at(model, qpos)]
    ASSEMBLY_REPORT['deck_world_pos_residual_m'] = residual
    ASSEMBLY_REPORT['deck_x0_standalone'] = _deck_standalone_x0()

    ctrl = home_hold_ctrl(model, qpos, qpos, np.zeros(model.nv))
    joint_trn = int(mujoco.mjtTrn.mjTRN_JOINT)
    kept = []
    for a in range(model.nu):
        if int(model.actuator_trntype[a]) == joint_trn:
            continue                     # joint servos are governed by the hold law above
        nm = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, a) or ''
        for key, declared in declarations.items():
            short = nm[len(key) + 1:] if nm.startswith(key + '_') else None
            if short is not None and short in declared:
                ctrl[a] = float(declared[short])
                kept.append(nm)
    if kept:
        notes.append(('-', f'declared values kept for non-joint transmissions: {kept}', 0))
    for key, body, joint, declared, reference in free_disagreements:
        notes.append((key, f'DECLARED FREE POSE IGNORED on {body}.{joint}: the source declares '
                           f'{declared}, the model reference is {reference}; the model wins and '
                           f'the disagreement is recorded because it means the source keyframe '
                           f'is not a complete state of its own scene', 0))
    return qpos, ctrl, applied, notes


def build_text():
    spec, layout = build_spec()
    model = spec.compile()
    qpos, ctrl, applied, notes = merged_home(model, layout)
    if spec.add_key(name='home', qpos=qpos, ctrl=ctrl) is None:
        raise RuntimeError('add_key returned None')
    # Recompile: the first compile predates the keyframe, so reporting from it said `nkey=0`
    # for a world that has one. A stale reading of the very artifact being built is exactly the
    # kind of thing this project keeps catching.
    model = spec.compile()
    header = (
        '<!--\n'
        '  GENERATED by experiments/merge_world.py -- do not hand-edit.\n'
        '  Rebuild and verify with:  ./.venv/bin/python experiments/merge_world.py --check\n\n'
        '  P1 common world. Four roles, one MjModel, one time base. Built because the four\n'
        '  component experiments each passed alone and that says nothing about loading them\n'
        '  together. What this artifact does NOT claim:\n'
        '    * that the four tasks run together (they do not; this is a loading gate);\n'
        '    * that C\'s deck is mounted on N\'s chassis (it is not -- C\'s deck is world-welded\n'
        '      and N\'s chassis has no mounting interface);\n'
        '    * that the ground is a faithful stand-in for N\'s arena -- N\'s own floor, its four\n'
        '      walls and its three pillars were dropped, and the cell\'s walls declare 1.00\n'
        '      where N\'s declared 0.90;\n'
        '    * that a shared floor is harmless at ANY value -- it is inert only while it stays\n'
        '      below the friction of every geom that touches it (measured; see\n'
        '      SHARED_GROUND_FRICTION, which the gate re-measures on every run).\n'
        '-->\n')
    xml = with_option_block(spec.to_xml())
    stepping = verify_stepping(xml)
    kinematics = verify_kinematics(xml, layout)
    return header + xml, layout, model, stepping, kinematics, notes, applied


def build(check=False):
    text, layout, model, stepping, kinematics, notes, applied = build_text()
    if check:
        if not TARGET.is_file():
            print(f'MISSING: {TARGET.relative_to(ROOT)} has not been generated')
            return 1
        on_disk = TARGET.read_text(encoding='utf-8')
        if on_disk == text:
            print(f'OK: {TARGET.relative_to(ROOT)} matches its sources '
                  f'(sha256 {hashlib.sha256(TARGET.read_bytes()).hexdigest()})')
            return 0
        print(f'STALE: {TARGET.relative_to(ROOT)} differs from what its sources produce. '
              f'Re-run without --check.')
        print(f'  on disk {len(on_disk)} chars, rebuilt {len(text)} chars')
        return 1
    TARGET.write_text(text, encoding='utf-8')
    print(f'wrote {TARGET.relative_to(ROOT)}  {len(text)} chars')
    print(f'  sha256 {hashlib.sha256(TARGET.read_bytes()).hexdigest()}')
    print(f'  nq={model.nq} nv={model.nv} nu={model.nu} nbody={model.nbody} '
          f'ngeom={model.ngeom} njnt={model.njnt} neq={model.neq} nkey={model.nkey}')
    for role in ROLES:
        x, y = layout[role.key]
        lo, hi = role_extent(role)
        print(f'  {role.key}: station ({x:+.3f}, {y:+.3f})  own extent '
              f'x {lo[0]:+.3f}..{hi[0]:+.3f}  ->  world x {x + lo[0]:+.3f}..{x + hi[0]:+.3f}')
    lo_x, hi_x, lo_y, hi_y = cell_bounds(layout)
    print(f'  cell x {lo_x:+.3f}..{hi_x:+.3f}  y {lo_y:+.3f}..{hi_y:+.3f}')
    joints, bodies, worst, where, unnamed, relocated = kinematics
    print(f'  stepping verified on the compiled world: timestep={stepping[0]} '
          f'integrator={stepping[1]} (implicitfast), angle="radian"')
    print(f'  kinematics verified by name: {joints} joints and {bodies} bodies match their '
          f'sources to {worst:.2e} (worst at {where or "nothing"})')
    if relocated:
        # Named, not silently skipped. The comparison that DOES apply to these is a world-pose
        # comparison, and it lives in the gate (`check_rig_reproduced`, tolerance 1e-9).
        print(f'  relocated by the V1 assembly (local pos rewritten on purpose; world pose '
              f'measured by the gate instead): {len(relocated)} bodies -- {relocated[:3]}'
              f'{" ..." if len(relocated) > 3 else ""}')
    if unnamed:
        # Precise wording. It is the SOURCE that has no name for these, which is why the
        # merged world assigns one; saying the merged world is "not addressable by name"
        # was misleading from the moment name_base_free_joint started fixing exactly that.
        print(f'  unnamed in the SOURCE (the merged world assigns a name): {unnamed}')
    print(f'  home: {applied} joint overrides applied by name on top of the merged qpos0')
    for key, note, n in notes:
        print(f'  home {key}: {n} joint(s) declared -- {note}')
    return 0


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--check', action='store_true',
                        help='compare with what is on disk; write nothing')
    parser.add_argument('--layout', action='store_true')
    parser.add_argument('--surveys', action='store_true')
    args = parser.parse_args()
    if args.layout:
        for k, v in station_layout().items():
            print(f'  {k}: {v}')
        print(f'  bounds: {cell_bounds(station_layout())}')
        return 0
    if args.surveys:
        print(f'  ground friction per source: {ground_friction_sources()}')
        print(f'  stepping per source:        {sources_agree_on_stepping()}')
        return 0
    return build(check=args.check)


if __name__ == '__main__':
    try:
        _code = main()
    finally:
        scratch = ASSETS / '_merge_role_c.xml'
        if scratch.is_file():
            scratch.unlink()
    sys.exit(_code)
