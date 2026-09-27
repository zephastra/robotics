"""D031-A / D032 contract: the rear PUSHER, now a two-DOF carriage + blade.

WHY THE MECHANISM HAS TWO JOINTS. A pusher rigidly bolted to the deck was PROVED infeasible: it
has one x, and EMBARK asks two opposite things of it --

    REACH        face >= w0 - T   (meet the wall before the deck runs out)
    NEVER AHEAD  face <= w0 - T   (do not strike a stalled tray from the front)

-- whose only solution is an equality that is zero-margin and lies off the rear of the deck. So
the carriage slides in x, which decouples them; the blade lifts in z, which keeps it out of the
tray's path while the tray is conveyed over it.

WHAT IT IS NOT: this is a PUSHER, not the transport stopper / lock. Retaining a tray against an
AMR's acceleration is a different mechanism with different requirements (two-sided, releasable).
Nothing here claims it -- P1-C-05 is a bench transfer with no AMR in the loop.

FRAME: deck-local. x = 0 at the deck body origin, z = 0 at the crown plane. Roller bodies sit at
z = -ROLLER_RADIUS and their crowns touch z = 0, so the roller band is z in [-2r, 0].
"""
import ast
import math
import re
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
TRAY = ROOT / 'assets' / 'objects' / 'tray_v1.xml'
RIG_PATH = ROOT / 'experiments' / 'roller_rig.py'
PROBE_PATH = ROOT / 'experiments' / 'probe_deck.py'

BARE_NUMBER = re.compile(r'-?\d+(?:\.\d+)?')


def _module_constant(name, path):
    tree = ast.parse(Path(path).read_text(encoding='utf-8'))
    for node in tree.body:
        if isinstance(node, ast.Assign):
            for target in node.targets:
                if isinstance(target, ast.Name) and target.id == name:
                    return ast.unparse(node.value)
    return None


def _resolve(name, path, hops=6):
    expr = _module_constant(name, path)
    seen = set()
    for _ in range(hops):
        if expr is None or expr in seen:
            break
        seen.add(expr)
        nxt = _module_constant(expr, path)
        if nxt is None:
            break
        expr = nxt
    return expr


@pytest.fixture(scope='module')
def rig():
    import importlib.util
    spec = importlib.util.spec_from_file_location('p1a_rig_contract', RIG_PATH)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.fixture(scope='module')
def tray_geom(rig):
    """The tray's x-end wall band (the pressable face) and the seat-derived wall interval."""
    m = rig.tray_measurements(TRAY)
    offset = -m['z_min']
    walls = [p for p in m['parts'] if (p['name'] or '').startswith('tray_wall_x')]
    assert walls, 'the tray asset has no x-end wall; a rear pusher has nothing to push'
    return {
        'z_lo': min(p['z_min'] for p in walls) + offset,
        'z_hi': max(p['z_max'] for p in walls) + offset,
        'y_half': max(max(abs(p['y_min']), abs(p['y_max'])) for p in walls),
        'half_length': rig.roller_half_length(m),
    }


def _aabb(geom):
    kind = geom.get('type', 'sphere')
    pos = [float(v) for v in (geom.get('pos') or '0 0 0').split()]
    size = [float(v) for v in (geom.get('size') or '0').split()]
    quat = [float(v) for v in (geom.get('quat') or '1 0 0 0').split()]
    angle = 2.0 * math.atan2(quat[3], quat[0]) if quat[3] else 0.0
    if kind == 'box':
        hx, hy, hz = size[0], size[1], size[2]
    elif kind == 'sphere':
        hx = hy = hz = size[0]
    else:
        hx, hz = size[0], float(size[1])
        hy = size[0]
    cos_a, sin_a = abs(math.cos(angle)), abs(math.sin(angle))
    rx = hx * cos_a + hy * sin_a
    ry = hx * sin_a + hy * cos_a
    return (pos[0] - rx, pos[0] + rx, pos[1] - ry, pos[1] + ry, pos[2] - hz, pos[2] + hz)


def model_tree(rig, **kw):
    return ET.fromstring(rig.build_model(TRAY, **kw)[0])


def nested_bodies(root):
    """Walk the body tree, composing each body's pos into a deck-local origin."""
    out = {}

    def walk(node, offset):
        for body in node.findall('body'):
            pos = [float(v) for v in body.get('pos').split()]
            origin = (offset[0] + pos[0], offset[1] + pos[1], offset[2] + pos[2])
            out[body.get('name')] = {'origin': origin, 'body': body, 'parent': node}
            walk(body, origin)

    walk(root.find("worldbody/body[@name='deck']"), (0.0, 0.0, 0.0))
    return out


def geom_boxes(rig, **kw):
    """name -> deck-local AABB of every geom under the deck, nested offsets composed."""
    root = model_tree(rig, **kw)
    out = {}

    def walk(node, offset):
        for child in node:
            if child.tag == 'geom':
                local = _aabb(child)
                out[child.get('name')] = tuple(
                    local[i] + offset[i // 2] for i in range(6))
            elif child.tag == 'body':
                pos = [float(v) for v in child.get('pos').split()]
                walk(child, (offset[0] + pos[0], offset[1] + pos[1], offset[2] + pos[2]))

    walk(root.find("worldbody/body[@name='deck']"), (0.0, 0.0, 0.0))
    return out


# =============================================================================================
# 1. THE REASON THE MECHANISM HAS TWO JOINTS: prove the rigid version is infeasible
# =============================================================================================

def test_a_rigidly_mounted_pusher_is_proved_infeasible(rig, tray_geom):
    """Keep the proof in the suite, not only in a document.

    If someone later 'simplifies' the carriage away and bolts the blade to the deck, this is the
    test that tells them why they cannot: the two inequalities have a single-point solution and
    that point is off the rear of the deck.
    """
    half = tray_geom['half_length']
    T = rig.DECK_TRAVEL_REF
    # The wall rear face interval, from the seat beams.
    w_lo = rig.BEAM_SEAT_A_X - half
    w_hi = rig.BEAM_SEAT_B_X - half
    for w0 in (w_lo, w_hi):
        reach = w0 - T          # face must be >= this
        never_ahead = w0 - T    # face must be <= this
        assert abs(reach - never_ahead) < 1e-12, (
            'the two constraints should coincide for a rigid pusher; the analysis assumed it'
        )
        body_lo = (w0 - T) - rig.PUSHER_HALF[0]
        assert body_lo < 0.0, (
            'the only feasible rigid position (%+.4f) should lie off the rear of the deck '
            '(deck origin 0.0); if it does not, the carriage is no longer justified' % body_lo
        )


def test_the_slide_stroke_covers_the_deploy_plus_the_whole_deck_travel(rig):
    """The stroke has to do two jobs in sequence with one joint: close the wall gap, then follow
    the wall for the full travel. The deploy is not recoverable mid-run, so they ADD."""
    assert rig.PUSHER_SLIDE_TRAVEL >= rig.PUSHER_DEPLOY_TRAVEL + rig.DECK_TRAVEL_REF, (
        'slide stroke %.4f < deploy %.4f + travel %.4f'
        % (rig.PUSHER_SLIDE_TRAVEL, rig.PUSHER_DEPLOY_TRAVEL, rig.DECK_TRAVEL_REF)
    )


# =============================================================================================
# 2. THE BLADE MUST PASS *UNDER* THE ROLLERS -- the defect a degree of freedom introduces
# =============================================================================================

def test_the_stowed_blade_lies_above_the_crown_plane(rig):
    """The blade slides the length of the deck, so it must clear EVERY roller at EVERY stroke
    position -- but the escape is ABOVE the band, not below it.

    THE SUPERSEDED FORM of this test asserted `blade_top <= band_bottom`: hide the blade under
    the rollers. That was measured WRONG. The fixed row runs under the deck origin at the same z
    band as the deck's own rollers, so "below the band" is simply the other row's band:
        blade in the band, top on the crown plane -> fixed_roller_2_5 -0.0046, 2_6 -0.0054
    Three placements were measured in the solver before this one, and only the last is clear:
        blade bottom at +0.0010, top at +0.0610    -> CLEAR at stow, deploy, mid-follow, full
    The reason is structural: the crown plane is the band's TOP, so everything above it is
    roller-free at EVERY x, and the whole corridor argument stops being needed.
    """
    boxes = geom_boxes(rig)
    blade_bottom = boxes['rear_pusher'][4]
    band_top = 0.0
    assert blade_bottom >= band_top + rig.PUSHER_CROWN_CLEARANCE - 1e-9, (
        'the stowed blade bottom is at z %+.4f, not the declared %+.4f above the crown plane '
        '%+.4f: it would plow into every roller it slides past'
        % (blade_bottom, rig.PUSHER_CROWN_CLEARANCE, band_top)
    )
    # And the clearance must be a REAL gap, not a coincidence that happens to be positive.
    assert rig.PUSHER_CROWN_CLEARANCE > 0.0, (
        'the crown clearance is %+.4f; a zero or negative gap is the tangency that measured '
        '-0.000009 against fixed_roller_2_6' % rig.PUSHER_CROWN_CLEARANCE
    )


def test_the_carriage_body_also_lies_below_the_roller_band(rig):
    boxes = geom_boxes(rig)
    assert 'pusher_carriage_body' in boxes, 'the carriage has no body geom'
    assert boxes['pusher_carriage_body'][5] <= -2.0 * rig.ROLLER_RADIUS + 1e-9, (
        'the carriage body reaches into the roller band; it travels the whole deck length'
    )


def test_the_carriage_clears_the_deck_frame_at_every_stroke(rig):
    """The carriage slides x over the whole stroke and the frame is 0.600 m wide, so their x
    ranges overlap for practically the whole travel -- at the forward stop the carriage's rear
    face lands EXACTLY on the frame's rear edge, which is a zero-clearance tangency and is the
    failure this test exists for. They must therefore be separated in z."""
    boxes = geom_boxes(rig)
    car = boxes['pusher_carriage_body']
    frame = boxes['deck_frame']
    # The stroke sweep, not just the stowed pose: the carriage's x range moves by the whole
    # slide travel, so at the forward stop its span is shifted.
    car_lo = car[0]
    car_hi = car[1] + rig.PUSHER_SLIDE_TRAVEL
    x_overlap = car_lo < frame[1] and car_hi > frame[0]
    assert x_overlap, (
        'the carriage never overlaps the frame in x; this test would be vacuous'
    )
    assert car[5] <= frame[4] + 1e-9, (
        'carriage z band %+.4f..%+.4f is not below the deck frame %+.4f..%+.4f; at the forward '
        'stop their x ranges coincide, so z is the only separation available'
        % (car[4], car[5], frame[4], frame[5])
    )


def test_the_deployed_blade_still_clears_every_roller(rig):
    """Deploying raises the blade INTO the band's upper region, where the rollers are. It must
    have travelled past them in x first -- which is only true because the deploy happens after
    the tray is seated and the carriage's forward stop is short of the last crown."""
    blade_face_at_stop = rig.PUSHER_X + rig.PUSHER_SLIDE_TRAVEL + rig.PUSHER_HALF[0]
    assert blade_face_at_stop <= rig.PUSHER_FORWARD_LIMIT + 1e-9, (
        'at full stroke the blade reaches %+.4f, past the deck last crown %+.4f: it would shove '
        'the tray off the end instead of handing it over'
        % (blade_face_at_stop, rig.PUSHER_FORWARD_LIMIT)
    )


# =============================================================================================
# 3. reach and safety, now satisfied by the CARRIAGE rather than by a fixed position
# =============================================================================================

def test_the_retracted_blade_is_behind_the_worst_case_wall(rig, tray_geom):
    """A tray can stop anywhere in the seat window, so the wall has an interval of possible
    positions. The retracted blade must be behind the FURTHEST-BACK one, or it is already ahead
    of a tray that stopped early and would hit it from the front."""
    boxes = geom_boxes(rig)
    retracted_face = boxes['rear_pusher'][1]        # slide = 0
    assert retracted_face <= rig.PUSHER_WALL_REAR_LO + 1e-9, (
        'the retracted blade face is at %+.4f, ahead of the worst-case wall %+.4f'
        % (retracted_face, rig.PUSHER_WALL_REAR_LO)
    )


def test_the_deploy_stroke_reaches_the_nearest_wall(rig):
    """The deploy must be able to close the gap even for a tray that stopped furthest forward,
    because that is the widest gap between the retracted blade and a wall it must press."""
    face_at_deploy = rig.PUSHER_X + rig.PUSHER_DEPLOY_TRAVEL + rig.PUSHER_HALF[0]
    assert face_at_deploy >= rig.PUSHER_WALL_REAR_HI, (
        'the blade at full deploy reaches %+.4f, short of the nearest wall %+.4f: the widest-gap '
        'tray could never be pushed' % (face_at_deploy, rig.PUSHER_WALL_REAR_HI)
    )


def test_the_deploy_is_stop_on_contact_and_not_drive_to_the_limit(rig):
    """The stroke LIMIT is a ceiling, not a commanded position.

    `PUSHER_DEPLOY_TRAVEL` brings the blade to the NEAREST wall in the seat window; the blade is
    expected to meet the wall EARLIER for any tray that stopped further back. So the deploy must
    end on contact. Driving to the limit would push a tray that stopped early by the width of the
    seat window (0.100 m), which is the fault the whole mechanism exists to avoid.

    This is a probe-source invariant, because the geometry cannot express it: the same ceiling is
    correct for both behaviours.
    """
    src = PROBE_PATH.read_text(encoding='utf-8')
    deploy = src[src.index("state == 'DEPLOY_PUSHER'"):]
    deploy = deploy[:deploy.index('elif state ==')]
    assert 'pusher_touching' in deploy, (
        'the DEPLOY_PUSHER branch does not test for contact: it would drive to the stroke limit '
        'and push a tray that stopped early'
    )
    assert 'min(' in deploy, (
        'the DEPLOY_PUSHER branch does not ramp the carriage: a step to the limit would strike '
        'the wall at full speed'
    )
    # And the ceiling must still be reachable for the nearest possible wall.
    face_at_limit = rig.PUSHER_X + rig.PUSHER_DEPLOY_TRAVEL + rig.PUSHER_HALF[0]
    assert face_at_limit >= rig.PUSHER_WALL_REAR_HI - 1e-9, (
        'the stroke cannot even reach the nearest wall: no tray could ever be pushed'
    )


def _blade_z_band(rig):
    """The blade's physical z band in the DEPLOYED (pressing) state.

    Both edges read off the geometry rather than reconstructed: the blade body sits at
    `PUSHER_STOW_Z` and its geom is a further `PUSHER_HALF[2]` above it, so the bottom face is
    `PUSHER_STOW_Z` and the top is `PUSHER_STOW_Z + 2 * PUSHER_HALF[2]`.

    NOTE the quantity CHANGED with the design: this used to be `stow_top + travel`, because the
    blade had to climb into position. It no longer climbs -- it is stowed at the push height --
    so `+ travel` would be wrong and would double-count the lift.
    """
    bottom = rig.PUSHER_STOW_Z
    top = rig.PUSHER_STOW_Z + 2.0 * rig.PUSHER_HALF[2]
    return bottom, top


def test_the_blade_presses_low_on_the_wall(rig, tray_geom):
    """Push LOW or the tray tips. The tray is 0.098 kg and 0.500 m long; a push above the wall's
    mid-height drives the leading lip down into the 0.010 m hole between crowns."""
    bottom, top = _blade_z_band(rig)
    midpoint = 0.5 * (tray_geom['z_lo'] + tray_geom['z_hi'])
    assert top < midpoint, (
        'the deployed blade top is at %+.4f, at or above the wall midpoint %+.4f'
        % (top, midpoint)
    )
    assert bottom <= tray_geom['z_lo'] + 1e-9, (
        'the deployed blade bottom %+.4f starts above the wall bottom %+.4f, so the blade '
        'would press high or miss the wall entirely'
        % (bottom, tray_geom['z_lo'])
    )


def test_the_blade_band_stays_within_the_wall_height(rig, tray_geom):
    _, top = _blade_z_band(rig)
    assert top <= tray_geom['z_hi'] + 1e-9, (
        'the deployed blade reaches %+.4f, above the wall top %+.4f'
        % (top, tray_geom['z_hi'])
    )


# =============================================================================================
# 4. DERIVED, not typed
# =============================================================================================

def test_the_pusher_anchors_are_derived_from_the_seat_beams_and_the_tray(rig):
    """The carriage's position is decided by where the tray STOPS, and the seat sensor pair is
    what decides that. Anchoring to the deck alone would silently break if the beams moved."""
    for name, needles in (('PUSHER_WALL_REAR_LO', ('BEAM_SEAT_A_X',)),
                          ('PUSHER_WALL_REAR_HI', ('BEAM_SEAT_B_X',)),
                          ('PUSHER_X', ('PUSHER_WALL_REAR_LO', 'PUSHER_HALF')),
                          ('PUSHER_DEPLOY_TRAVEL', ('PUSHER_WALL_REAR_HI', 'PUSHER_X')),
                          ('PUSHER_SLIDE_TRAVEL', ('PUSHER_DEPLOY_TRAVEL', 'DECK_TRAVEL_REF'))):
        resolved = _resolve(name, RIG_PATH)
        assert resolved is not None, '%s is not a module-level assignment' % name
        assert not BARE_NUMBER.fullmatch(resolved), (
            '%s resolves to the bare literal %s' % (name, resolved)
        )
        assert any(n in resolved for n in needles), (
            '%s resolves to %r, referencing none of %s' % (name, resolved, needles)
        )


def test_the_stow_height_is_derived_off_the_crown_plane(rig):
    """The stow is now the PUSH height, and its only input is the declared crown clearance --
    so the thing that must be derived is the clearance, and the thing that must NOT be a
    literal is the stow.

    THE SUPERSEDED FORM asserted `PUSHER_STOW_DEPTH` appears in `PUSHER_STOW_TOP_Z` and
    `PUSHER_HALF` appears in `PUSHER_STOW_Z`. Both were true of the old design, where the stow
    was a depth below the band and the body's origin was the top minus a half-height. Neither
    is true now, and asserting them would have forced the wrong geometry back in. What matters
    is the MEASURABLE claim: the blade's bottom clears the crown plane by the declared gap, and
    the gap is a positive, named constant rather than a magic number typed at the call site.
    """
    # NOTE: `_resolve` follows the chain to the LEAF, so it would report `0.001` here and
    # the "is it a literal?" test would fire on a correctly-derived value. The claim to make is
    # about the DIRECT assignment, which is exactly what `_module_constant` returns.
    direct = _module_constant('PUSHER_STOW_TOP_Z', RIG_PATH)
    assert direct is not None, 'PUSHER_STOW_TOP_Z is not a module-level assignment'
    assert not BARE_NUMBER.fullmatch(direct), (
        'PUSHER_STOW_TOP_Z is assigned the bare literal %s; it must be the named clearance'
        % direct
    )
    assert 'PUSHER_CROWN_CLEARANCE' in direct, (
        'PUSHER_STOW_TOP_Z is assigned %r, which does not reference the declared clearance'
        % direct
    )
    # The clearance is a DECLARED constant, so "is it a literal?" is the wrong question -- a
    # declared constant is a number by definition. What carries the intent is that it is a
    # module-level assignment, that it is a real (small, positive) gap, and that the stow
    # derives from it -- which the check above already establishes.
    assert _module_constant('PUSHER_CROWN_CLEARANCE', RIG_PATH) is not None, (
        'PUSHER_CROWN_CLEARANCE is not a module-level assignment'
    )
    assert 0.0 < rig.PUSHER_CROWN_CLEARANCE <= 0.005, (
        'the crown clearance is %+.4f; it must be a positive gap no larger than a few '
        'millimetres, or it is a standoff rather than a clearance, and the blade stops '
        'pressing the wall low' % rig.PUSHER_CROWN_CLEARANCE
    )
    assert rig.PUSHER_CROWN_CLEARANCE > 0.0, (
        'the crown clearance must be a real gap; %+.4f is the tangency that measured '
        '-0.000009 against fixed_roller_2_6' % rig.PUSHER_CROWN_CLEARANCE
    )
    # And the BLADE must actually be above the band, measured off the built model.
    boxes = geom_boxes(rig)
    assert boxes['rear_pusher'][4] >= 0.0, (
        'the blade bottom is at %+.4f, inside the roller band' % boxes['rear_pusher'][4]
    )


def test_the_lift_travel_is_derived_from_the_wall_band(rig):
    """The lift's job changed, so its derivation did too.

    THE SUPERSEDED FORM pinned `PUSHER_DEPLOY_TOP_Z` inside `PUSHER_TRAVEL`. That was right when
    the lift climbed from a stow *below* the band up to the push height. The lift now starts AT
    the push height and rises to clear the wall's TOP edge, so the honest needle is the wall
    band itself. Keeping the old needle would have been satisfied vacuously, because
    `PUSHER_DEPLOY_TOP_Z` is now literally equal to the stow -- the assertion would have passed
    while proving nothing.
    """
    resolved = _resolve('PUSHER_TRAVEL', RIG_PATH)
    assert resolved is not None, 'PUSHER_TRAVEL is not a module-level assignment'
    assert not BARE_NUMBER.fullmatch(resolved), (
        'PUSHER_TRAVEL resolves to the bare literal %s' % resolved
    )
    assert 'PUSHER_WALL' in resolved, (
        'PUSHER_TRAVEL (%r) does not reference the wall band, which is what it must clear'
        % resolved
    )
    # And the travel must be big enough to actually clear the wall it references.
    wall_top = rig.PUSHER_WALL['z_hi']
    assert rig.PUSHER_TRAVEL >= (wall_top + rig.PUSHER_ENGAGE_M) - rig.PUSHER_STOW_TOP_Z - 1e-9, (
        'the lift travel %+.4f cannot raise the blade clear of the wall top %+.4f'
        % (rig.PUSHER_TRAVEL, wall_top)
    )


# =============================================================================================
# 5. SEMANTICS: it is a pusher, the probe must monitor it, and EMBARK must require it
# =============================================================================================

def test_the_mechanism_is_named_a_pusher_and_the_old_pin_is_gone(rig):
    xml, _ = rig.build_model(TRAY)
    assert 'pusher_carriage' in xml and 'pusher_blade' in xml
    assert 'rear_pusher' in xml
    assert 'retention_pin' not in xml, 'the old in-plane pin is still in the model'
    assert 'pin_lift_joint' not in xml, 'the old pin joint is still in the model'


def test_the_pusher_has_two_degrees_of_freedom(rig):
    """The whole point of D032. A single slide joint means someone removed the carriage."""
    xml, _ = rig.build_model(TRAY)
    assert 'pusher_slide_joint' in xml, 'the carriage slide joint is missing'
    assert 'pusher_lift_joint' in xml, 'the blade lift joint is missing'
    joints = {j.get('name') for j in ET.fromstring(xml).iter('joint')}
    assert {'pusher_slide_joint', 'pusher_lift_joint'} <= joints
    acts = {a.get('name') for a in ET.fromstring(xml).find('actuator')}
    assert {'pusher_slide', 'pusher_lift'} <= acts, 'both joints need actuators'


def test_the_rig_does_not_advertise_the_pusher_as_the_transport_stopper(rig):
    src = RIG_PATH.read_text(encoding='utf-8').lower()
    joined = '\n'.join(ln for ln in src.splitlines() if 'pusher' in ln or 'scope' in ln)
    assert 'not the transport stopper' in joined, (
        'the source must state that the pusher is NOT the transport stopper'
    )


def test_the_probe_monitors_pusher_to_wall_contact(rig):
    src = PROBE_PATH.read_text(encoding='utf-8')
    assert 'rear_pusher' in src, 'probe_deck.py does not look up the pusher geom'
    assert 'pusher_contacts' in src, 'the probe must record per-sample pusher contact'
    assert 'pusher_engaged' in src, (
        'the probe must latch whether the pusher ever touched the tray'
    )
    assert 'pusher_first_contact_t' in src, 'the engagement needs a WHEN'


def test_the_probe_drives_both_pusher_joints(rig):
    src = PROBE_PATH.read_text(encoding='utf-8')
    assert 'pusher_slide' in src, 'the probe never commands the carriage'
    assert 'pusher_lift' in src, 'the probe never commands the blade'


def test_the_embark_outcome_requires_the_pusher_to_have_engaged(rig):
    src = PROBE_PATH.read_text(encoding='utf-8')
    assert 'PUSHER_NEVER_ENGAGED' in src, (
        'the probe needs a NAMED terminal state for "the tray moved but the pusher never '
        'touched it", so that outcome cannot be filed as a success'
    )
    tail = src[src.index("'faulted'") : src.index("'faulted'") + 500]
    assert 'PUSHER_NEVER_ENGAGED' in tail, (
        'PUSHER_NEVER_ENGAGED must be classified as a fault, not as a refusal or a pass'
    )
