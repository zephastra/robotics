"""Static geometry contract for the C rig. No simulation, no solver -- pure XML arithmetic.

This file exists because the previous C round shipped five basic geometry errors that only
showed up at runtime. Each one of them would have been caught here in under a second, which is
what `AGENTS.md` means by "先失败复现/契约测试，再修改".
"""
import ast
import importlib.util
import pytest
import re
import math
from pathlib import Path
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
TRAY = ROOT / 'assets' / 'objects' / 'tray_v1.xml'

spec = importlib.util.spec_from_file_location('rig', ROOT / 'experiments' / 'roller_rig.py')
rig = importlib.util.module_from_spec(spec)
spec.loader.exec_module(rig)


def aabb_deck_local(geom, deck_yaw=0.0):
    """Axis-aligned bounds of one deck child, in deck-local coordinates.

    Rotated boxes are bounded conservatively, so an overlap reported here is possible rather
    than certain -- acceptable for a contract that must not miss a collision.
    """
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
        hx = hy = hz = size[0]
        hz = float(size[1])
    cos_a, sin_a = abs(math.cos(angle)), abs(math.sin(angle))
    rx = hx * cos_a + hy * sin_a
    ry = hx * sin_a + hy * cos_a
    return (pos[0] - rx, pos[0] + rx, pos[1] - ry, pos[1] + ry, pos[2] - hz, pos[2] + hz)


def deck_children():
    """AABB (deck-local) of every geom under the deck, keyed by GEOM name.

    REWRITTEN (D034). The previous version walked exactly two levels -- `deck` -> `geom` and
    `deck` -> `body` -> `geom` -- and keyed the nested case by BODY name. That silently lost
    everything below the second level, which is where the pusher blade lives:
        deck / pusher_carriage / pusher_blade / geom `rear_pusher`
    The old function therefore returned `pusher_carriage` (the plate) under a body key and
    never reported the blade at all. A test asking for the stow position would have got the
    carriage's z (-0.1200..-0.1000) and concluded the blade was safely under the deck, which is
    how a swept-volume check would have passed while the blade sat in the tray.

    This now walks the whole tree and accumulates each body's own offset down the chain, so a
    mechanism of any depth is measured where it actually is. Keys are geom names, which are
    unique and are what a reader looks for.
    """
    root = ET.fromstring(rig.build_model(TRAY, guides=True)[0])
    deck = root.find("worldbody/body[@name='deck']")
    out = {}

    def walk(node, offset):
        for child in node:
            if child.tag == 'geom':
                local = aabb_deck_local(child)
                out[child.get('name')] = (local[0] + offset[0], local[1] + offset[0],
                                          local[2] + offset[1], local[3] + offset[1],
                                          local[4] + offset[2], local[5] + offset[2])
            elif child.tag == 'body':
                # A body's `pos` is a 3-vector applied to (x, y, z); the AABB is stored as
                # (x0, x1, y0, y1, z0, z1), so each axis feeds two slots.
                pos = [float(v) for v in (child.get('pos') or '0 0 0').split()]
                shift = (offset[0] + pos[0], offset[1] + pos[1], offset[2] + pos[2])
                walk(child, shift)

    walk(deck, (0.0, 0.0, 0.0))
    return out


def roller_band():
    """Deck-local z band occupied by any roller (fixed rows share it, they sit on the crown)."""
    return (-rig.ROLLER_RADIUS * 2, 0.0)


def _bare_eval(node, env=None):
    """Evaluate one probe expression with NO name resolution. The recursion-free core."""
    return eval(compile(ast.Expression(node), '<probe>', 'eval'), env or {'math': math})


def _eval_literal(node):
    """Evaluate a probe constant that may be a literal, a `math.radians(x)` call, or a table entry.

    The window may legitimately be written as `min(STEP_MAX_BY_FRICTION.values())` so that the
    fallback cannot drift from the per-friction table. Those names are resolved from the probe's
    own text rather than imported, so this file keeps running in 0.04 s with no simulator.

    Resolution is deliberately ONE level deep and uses `_bare_eval` for the table's own entries:
    letting the two helpers call each other would recurse forever, because resolving a dict-valued
    name would re-resolve the same names.
    """
    tables = _declared_module_values({'GAP_MAX_BY_FRICTION', 'STEP_MAX_BY_FRICTION',
                                      'YAW_SENSOR_LIMIT'})
    env = {'math': math, **tables}
    return _bare_eval(node, env)


def declared_align(key):
    """A constant from the probe's `ALIGN` window, read as text.

    Importing `probe_deck` here would drag MuJoCo and a live `import roller_rig` into a test
    whose whole point is to run in 0.04 s with no simulator. Parsing the literal keeps the two
    files in step without coupling them, and the assertion below fails loudly if the entry
    ever stops being a plain literal or a `math.radians(...)` call.
    """
    src = (ROOT / 'experiments' / 'probe_deck.py').read_text(encoding='utf-8')
    tree = ast.parse(src)
    for node in tree.body:
        if isinstance(node, ast.Assign) and any(
                getattr(t, 'id', None) == 'ALIGN' for t in node.targets):
            for k, value in zip(node.value.keys, node.value.values):
                if getattr(k, 'value', None) == key:
                    return float(_eval_literal(value))
    raise AssertionError(f"probe_deck.ALIGN[{key!r}] is no longer a literal; update this test")


def declared_yaw_window():
    return declared_align('yaw_max')


def declared_scenarios():
    """The probe's scenario table as (name, gap, step, yaw) tuples, read as text."""
    src = (ROOT / 'experiments' / 'probe_deck.py').read_text(encoding='utf-8')
    tree = ast.parse(src)
    for node in tree.body:
        if isinstance(node, ast.Assign) and any(
                getattr(t, 'id', None) == 'SCENARIOS' for t in node.targets):
            out = []
            for elt in node.value.elts:
                fields = {getattr(k, 'value'): v for k, v in zip(elt.keys, elt.values)}
                out.append((fields['name'].value, _eval_literal(fields['gap']),
                            _eval_literal(fields['step']), _eval_literal(fields['yaw'])))
            return out
    raise AssertionError('probe_deck.SCENARIOS is no longer a literal; update this test')


def test_every_roller_row_sits_on_the_crown_plane():
    xml, _ = rig.build_model(TRAY)
    root = ET.fromstring(xml)
    for body in root.findall('worldbody/body'):
        if body.get('name').startswith('fixed_roller_'):
            assert abs(float(body.get('pos').split()[2]) - rig.ROLLER_Z) < 1e-9
    deck = root.find("worldbody/body[@name='deck']")
    for body in deck.findall('body'):
        if body.get('name').startswith('deck_roller_'):
            assert abs(float(body.get('pos').split()[2]) + rig.ROLLER_RADIUS) < 1e-9


def test_the_guides_cannot_touch_any_roller():
    lo, hi = roller_band()
    children = deck_children()
    rails = [name for name in children if name.startswith('guide_rail')]
    assert len(rails) == 2, f'expected a straight rail per side, found {rails}'
    for name in rails:
        z0, z1 = children[name][4], children[name][5]
        assert z1 <= lo or z0 >= hi, f'{name} overlaps the roller z band {lo}..{hi}'


def test_the_guides_sit_inside_the_band_the_tray_allows():
    _, meta = rig.build_model(TRAY, guides=True)
    band = meta['guide_band']
    children = deck_children()
    for sign in (-1, 1):
        name = f'guide_rail_{sign:+d}'
        y0, y1 = children[name][2], children[name][3]
        z1 = children[name][5]
        inner = min(abs(y0), abs(y1))
        outer = max(abs(y0), abs(y1))
        assert inner >= band['wall_outer_y'], 'guide would foul the tray side wall'
        assert outer <= band['ball_inner_y'], 'guide would foul the handle ball'
        assert z1 <= band['guide_z_max'], 'guide would foul the handle stem'


def test_the_guide_running_clearance_is_derived_not_just_positive():
    """A positive running clearance is not enough, and the previous revision proved it.

    The old rail had GUIDE_HALF_Y = 0.0125 at GUIDE_Y = 0.2495, so its inner face reached
    0.2620 against a handle ball whose inner face is at 0.2650: 3 mm of air. That reads as
    "clear" and behaves as "fitted", because the tray's rotational slop and the roller ripple
    are both millimetres. The rail must instead clear the ball by a stated multiple of the
    ACTUATED yaw window, which is the only yaw the interlock ever admits.
    """
    _, meta = rig.build_model(TRAY, guides=True)
    band = meta['guide_band']
    yaw_max = declared_yaw_window()
    # Slack against the STATIC ball face, with the ball at its worst (widest) pose.
    static_slack = band['ball_inner_y'] - (rig.GUIDE_Y + rig.GUIDE_HALF_Y)
    assert static_slack >= 0.004, \
        f'only {static_slack * 1000:.1f} mm of clearance to the handle ball; the 3 mm the ' \
        f'previous rail had is what let a rotated ball into it'
    # And the model's own claim must hold at the largest yaw the interlock admits: a rotated
    # tray swings its ball's inner face inward, so assert the margin survives that swing.
    swing = band['ball_inner_y'] * (1.0 - math.cos(yaw_max))
    assert static_slack - swing > 0.0, 'a rotated handle ball would touch the rail'
    assert rig.GUIDE_Y - rig.GUIDE_HALF_Y - band['wall_outer_y'] > 0.0
    assert band['guide_z_max'] - (rig.GUIDE_Z + rig.GUIDE_HALF_Z) > 0.0


def test_the_beams_pass_inside_the_guides_and_through_the_tray_dividers():
    _, meta = rig.build_model(TRAY, guides=True)
    children = deck_children()
    rail_inner = min(min(abs(children[f'guide_rail_{s:+d}'][2]),
                         abs(children[f'guide_rail_{s:+d}'][3])) for s in (-1, 1))
    assert abs(rig.BEAM_Y0) < rail_inner, 'a beam would be blocked by its own guide rail'
    lo, hi = roller_band()
    assert rig.BEAM_Z > hi, 'a beam at this height would be blocked by the rollers'
    divider = meta['guide_band']['divider_z']
    assert divider[0] <= rig.BEAM_Z <= divider[1], \
        'the beam height must pass through a tray divider, otherwise the beam never breaks'


def test_the_stowed_blade_clears_both_roller_rows_and_the_deck_end():
    """The blade must stow without fouling a roller, and without overhanging the deck end.

    REWRITTEN (D034). The old form read the AABB of a body named `retention_pin`, which no
    longer exists: the mechanism is now a two-DOF `pusher_carriage` + `pusher_blade`, and the
    blade's own geom is `rear_pusher`. One assertion survives as written; the other two had to
    be re-based, because they encoded the WRONG model of the mechanism:

      * "stowed pin must not present an edge to the tray" tested `z1 < 0.0`, i.e. the pin
        hides BELOW the crown plane. The blade cannot do that -- it would have to live in the
        roller band, which is the configuration that ploughs the fixed row (see the constants
        note in roller_rig.py, and D032). The blade instead sits just ABOVE the crown plane,
        and the load-bearing property is the one asserted here: it must clear the band.
      * "clear of both roller rows" was checked as an x-interval against the DECK row only.
        The blade actually stows over the FIXED row at deck-local -0.1650, so that assertion
        was measuring the wrong row and could not have failed for the right reason.

    Measured stow pose (deck-local): x -0.1750..-0.1550, z +0.0010..+0.0310.
    """
    children = deck_children()
    assert 'rear_pusher' in children, (
        f'the blade geom `rear_pusher` is not reachable from the deck; children are '
        f'{sorted(children)}')
    blade = children['rear_pusher']
    lo, hi = roller_band()
    assert blade[4] >= hi - 1e-9, (
        f'the blade bottom {blade[4]:.4f} is inside the roller band {lo:.4f}..{hi:.4f}: a blade '
        f'in the band ploughs the rollers (this is the D032 defect)')
    assert blade[5] > blade[4], 'the blade has no height'
    # The roller-free corridor: a body wholly behind BOTH rows' rear tangents is clear of every
    # roller at every height. This is the property that lets the blade live in the z band of the
    # band without hiding below it, and it is why the stow x is what it is.
    assert blade[1] <= rig.PUSHER_CORRIDOR_X + 1e-9, (
        f'the stowed blade reaches deck-local x {blade[1]:.4f}, forward of the roller-free '
        f'corridor limit {rig.PUSHER_CORRIDOR_X:.4f}: it would foul a fixed-row roller'
    )


def test_the_blade_never_reaches_the_forward_limit_that_would_shove_the_tray_off():
    """`PUSHER_FORWARD_LIMIT` is a real constraint and the stroke must respect it.

    REWRITTEN (D034). This is the salvageable half of the old
    `test_the_stowed_pin_is_below_the_surface_and_clear_of_both_roller_rows`, whose remaining
    claim -- "the mechanism must not overhang the deck end" -- is still true and still matters:
    a blade past the last crown shoves the tray off the end instead of handing it over.
    """
    assert rig.PUSHER_FORWARD_LIMIT <= rig.DECK_ROLLER_X0 + rig.ROLLER_PITCH * (rig.DECK_ROLLERS - 1) + 1e-9, (
        'PUSHER_FORWARD_LIMIT is past the deck\'s last crown; it no longer bounds anything')
    # The furthest forward the blade face can ever be: the retracted face plus the whole stroke.
    furthest = rig.PUSHER_X + rig.PUSHER_HALF[0] + rig.PUSHER_SLIDE_TRAVEL
    assert furthest <= rig.PUSHER_FORWARD_LIMIT + 1e-9, (
        f'the blade face can reach deck-local {furthest:.4f}, past the forward limit '
        f'{rig.PUSHER_FORWARD_LIMIT:.4f}: it would push the tray off the end'
    )


def test_the_frame_clears_the_deck_rollers():
    children = deck_children()
    frame_top = children['deck_frame'][5]
    lo, _ = roller_band()
    assert frame_top <= lo


def test_the_interface_friction_is_overridden_on_the_tray_asset():
    xml, meta = rig.build_model(TRAY, friction=0.25)
    assert meta['friction'] == 0.25
    assert meta['friction_overridden_on_asset_geoms'] > 0
    root = ET.fromstring(xml)
    tray = root.find("worldbody/body[@name='payload']")
    for geom in tray.findall('geom'):
        friction = geom.get('friction')
        if friction is None:
            continue
        assert friction.split()[0] == '0.2500', \
            f'{geom.get("name")} still carries the asset declaration {friction}'


def test_the_declared_friction_reaches_the_rollers_too():
    xml, _ = rig.build_model(TRAY, friction=0.15)
    root = ET.fromstring(xml)
    for body in root.findall('worldbody/body'):
        if body.get('name').startswith('fixed_roller_'):
            assert body.find('geom').get('friction').split()[0] == '0.1500'


def test_the_seat_beams_break_together_over_a_window_where_the_tray_is_aboard():
    """A pair of occlusion heads only breaks together inside the overlap of their windows.

    If the heads are spaced more than 2 x tray_half_x apart the overlap is empty, the seated
    transition is unreachable, and the state machine silently falls through to its guard. That
    failure mode is invisible from the geometry alone, which is why it is asserted here.
    """
    _, meta = rig.build_model(TRAY)
    half_x = (meta['tray']['x_max'] - meta['tray']['x_min']) / 2.0

    def window(x_b):
        return (x_b - half_x, x_b + half_x)

    a, b = window(rig.BEAM_SEAT_A_X), window(rig.BEAM_SEAT_B_X)
    lo, hi = max(a[0], b[0]), min(a[1], b[1])
    assert lo < hi, 'seat beams never break together: the seated transition is unreachable'
    assert lo >= half_x, 'the overlap must begin only once the tray is fully aboard'
    assert hi <= meta['deck_length'] - half_x, 'the overlap must end before the tray overhangs'
    assert window(rig.BEAM_FAR_X)[0] > hi, 'the far guard must not fire inside the seating window'


def test_a_laterally_flaring_mouth_is_impossible_with_this_tray():
    """A derived design constraint, asserted so it cannot be quietly undone again.

    A funnel mouth must be wider than the tray's side wall (0.230) at its entrance. But the
    handle ball sits at |y| 0.265..0.335, and although it spans only +-0.035 of x around the
    tray's centre, the tray translates through every x: a mouth at any fixed x is therefore
    swept by a ball. The mouth cannot exceed 0.265, which caps capture at 0.265 - 0.230 =
    35 mm. An earlier revision flared to 0.297 and struck the ball; with guides fitted the tray
    then never completed a transfer.
    """
    _, meta = rig.build_model(TRAY)
    band = meta['guide_band']
    capture = band['ball_inner_y'] - band['wall_outer_y']
    assert capture > 0.0, 'the tray geometry leaves no room for a guide at all'
    assert capture < 0.050, \
        'capture range grew past 50 mm; re-check whether a real funnel has become possible'
    names = [name for name in deck_children() if name.startswith('guide_lead')]
    assert not names, f'a flared lead-in was re-introduced ({names}); it will strike the ball'


def test_a_negative_gap_is_refused():
    try:
        rig.build_model(TRAY, gap=-0.001)
    except SystemExit:
        return
    raise AssertionError('a negative gap overlaps the fixed section and must be refused')


def test_the_fixed_to_deck_handoff_is_bridgeable():
    """The tray must not be able to fall into the strip between the two roller rows.

    This is THE test that was missing, and its absence cost two rounds. The tray is carried by
    rollers; between the last fixed crown and the first deck crown there is a strip with nothing
    under it. The tray bridges that strip only while its trailing edge is still on the last fixed
    roller, i.e. while `centre - half_length <= last_fixed_crown`. Its leading edge must have
    reached the first deck roller by then, i.e. `centre + half_length >= first_deck_crown`.

    Combining: the strip is bridgeable iff
        first_deck_crown <= last_fixed_crown + 2 * half_length
    With FIXED_SECTIONS = 2 that was 1.320 <= 1.200 + 0.130 = 1.330, i.e. bridged by 10 mm on
    paper -- but the tray is not a rigid plank on two point supports, so the real margin was
    negative and every low-friction run stalled with its leading edge about 20-30 mm short of the
    first deck roller. The requirement below therefore demands a REAL margin, not a paper one.
    """
    tray = rig.tray_measurements(TRAY)
    half_length = (tray['x_max'] - tray['x_min']) / 2.0
    last_fixed = rig.LAST_FIXED_CROWN_X
    first_deck = rig.DECK_X0 + rig.DECK_ROLLER_X0
    span = first_deck - last_fixed
    margin = (last_fixed + 2.0 * half_length) - first_deck
    assert span > 0.0, 'the deck overlaps the fixed row'
    # Demand at least a third of the tray's own support window as margin. A tray tipping into a
    # gap is a dynamic event, so a millimetre of paper clearance is not a design.
    assert margin >= (2.0 * half_length) / 3.0, (
        f'handoff strip {span:.4f} m leaves only {margin:.4f} m of bridge against a '
        f'{2.0 * half_length:.4f} m support window; the tray will nose-dive into it')
    # The strip no longer has to house the pin (the pin moved to the deck's outer end), so it is
    # bounded by the rollers themselves: two crowns may be no further apart than two radii,
    # otherwise there is a hole between them instead of a nip.
    assert span < 2.0 * rig.ROLLER_RADIUS + 1e-9, (
        f'the strip is {span:.4f} m, wider than two roller radii '
        f'({2.0 * rig.ROLLER_RADIUS:.4f} m), so there is a hole rather than a nip between the '
        f'two rows')


def _declared_module_values(names):
    """Read several module-level assignments from `probe_deck.py` by name.

    Handles both a bare literal and a subscript of a declared table (e.g. `STEP_MAX_BY_FRICTION[0.15]`),
    so the window may be written either way without silently failing to parse.
    """
    src = (ROOT / 'experiments' / 'probe_deck.py').read_text(encoding='utf-8')
    tree = ast.parse(src)
    found = {}
    for node in tree.body:
        if not isinstance(node, ast.Assign):
            continue
        for target in node.targets:
            if getattr(target, 'id', None) in names:
                if isinstance(node.value, ast.Dict):
                    found[target.id] = {_bare_eval(k): _bare_eval(v)
                                        for k, v in zip(node.value.keys, node.value.values)}
                else:
                    found[target.id] = _bare_eval(node.value)
    return found


def declared_windows():
    """Every declared window: the fallback plus the per-friction ones, as a list of dicts.

    The window is indexed by friction (see `probe_deck.ALIGN_BY_FRICTION`), so a scenario may
    legitimately sit INSIDE one friction's limit and OUTSIDE another's -- that is exactly what
    tests the indexing. What no scenario may do is sit ON a limit, because then the outcome is
    decided by rounding rather than by the interface.
    """
    src = (ROOT / 'experiments' / 'probe_deck.py').read_text(encoding='utf-8')
    tree = ast.parse(src)
    windows = {}
    for node in tree.body:
        if not isinstance(node, ast.Assign):
            continue
        names = [getattr(t, 'id', None) for t in node.targets]
        if 'ALIGN' in names:
            windows['fallback'] = {getattr(k, 'value'): _eval_literal(v)
                                   for k, v in zip(node.value.keys, node.value.values)}
        if 'ALIGN_BY_FRICTION' in names:
            for key, value in zip(node.value.keys, node.value.values):
                # Each entry may be a dict of literals, or reference the per-axis tables by
                # subscript (`GAP_MAX_BY_FRICTION[0.15]`). Resolve both, so the window may be
                # written either way without this helper silently failing to parse it.
                mu = _eval_literal(key)
                entry = {}
                for k, v in zip(value.keys, value.values):
                    field = getattr(k, 'value')
                    if isinstance(v, ast.Subscript):
                        table_name = getattr(v.value, 'id', None)
                        index = _eval_literal(v.slice)
                        entry[field] = _declared_module_values({table_name})[table_name][index]
                    else:
                        entry[field] = _eval_literal(v)
                windows[f'mu={mu}'] = entry
    assert 'fallback' in windows and len(windows) > 1, (
        'probe_deck no longer declares both ALIGN and ALIGN_BY_FRICTION')
    return windows


def declared_coupling():
    """`probe_deck.GAP_FLOOR` as a {friction: [(yaw, gap), ...]} dict.

    The floor is a MEASURED TABLE rather than a proportional law, because the true requirement is
    ZERO below a friction-dependent knee and only then rises. A proportional `k * |yaw|` was tried
    and REFUSED CASES THAT WORK, so the shape is part of the contract and is asserted, not assumed.
    """
    src = (ROOT / 'experiments' / 'probe_deck.py').read_text(encoding='utf-8')
    tree = ast.parse(src)
    for node in tree.body:
        if isinstance(node, ast.Assign) and any(
                getattr(t, 'id', None) == 'GAP_FLOOR' for t in node.targets):
            table = {}
            for key, rows in zip(node.value.keys, node.value.values):
                table[_eval_literal(key)] = [(_eval_literal(pair.elts[0]),
                                              _eval_literal(pair.elts[1]))
                                             for pair in rows.elts]
            return table
    raise AssertionError('probe_deck.GAP_FLOOR is no longer a literal table')


def declared_floor_safety():
    """`probe_deck.GAP_FLOOR_SAFETY` -- the margin added on top of the measured floor."""
    src = (ROOT / 'experiments' / 'probe_deck.py').read_text(encoding='utf-8')
    tree = ast.parse(src)
    for node in tree.body:
        if isinstance(node, ast.Assign) and any(
                getattr(t, 'id', None) == 'GAP_FLOOR_SAFETY' for t in node.targets):
            return float(_eval_literal(node.value))
    raise AssertionError('probe_deck.GAP_FLOOR_SAFETY is no longer a literal')


def floor_at(rows, yaw):
    """Interpolate a (yaw, gap) row the way `probe_deck._interp_rows` does."""
    if yaw <= rows[0][0]:
        return rows[0][1]
    for (y0, g0), (y1, g1) in zip(rows, rows[1:]):
        if yaw <= y1:
            return g0 + (yaw - y0) * (g1 - g0) / (y1 - y0)
    (y0, g0), (y1, g1) = rows[-2], rows[-1]
    return g1 + (yaw - y1) * (g1 - g0) / (y1 - y0)


def floor_for(friction, coupling):
    """The gap floor at this friction, including the safety factor."""
    rows = next((v for mu, v in coupling.items() if abs(mu - friction) < 1e-9), None)
    if rows is None:
        knots = sorted({y for r in coupling.values() for y, _ in r})
        rows = [(y, max(floor_at(r, y) for r in coupling.values())) for y in knots]
    return rows


def window_for(friction, windows):
    """The window that applies at this friction, in the same shape the probe uses."""
    for key, window in windows.items():
        if key.startswith('mu=') and abs(float(key[3:]) - friction) < 1e-9:
            return window
    return windows['fallback']


def test_no_scenario_sits_exactly_on_the_gate_boundary():
    """A scenario placed exactly on a window limit is decided by rounding, not by the interface.

    This is not a theoretical worry. `gap_20mm` (gap == gap_max == 0.020) was refused by the armed
    gate and then completed in all six bypass runs, which reads as "the gate refuses a workable
    case" until the raw arithmetic shows the measured gap was 0.0200000001 against a limit of
    0.020. A scenario that close to the boundary produces an arbitrary verdict, and the bypass arm
    then reports that arbitrary verdict as a tightness failure.

    The coupling floor `|gap| >= gap_floor(mu, |yaw|)` is checked too: a scenario sitting exactly
    on that floor would be just as arbitrary as one sitting on an axis limit.
    """
    offset = declared_align_margin()
    coupling = declared_coupling()
    safety = declared_floor_safety()
    windows = declared_windows()
    offenders = []
    for friction in coupling:
        window = window_for(friction, windows)
        rows = floor_for(friction, coupling)
        for name, gap, step, yaw in declared_scenarios():
            for axis, value in (('gap', gap), ('step', step), ('yaw', yaw)):
                limit = window[f'{axis}_max']
                if abs(abs(value) - limit) < offset:
                    offenders.append(f'{name} {axis}={value!r} on mu={friction} limit {limit!r}')
            floor = floor_at(rows, abs(yaw)) * safety
            # Only meaningful where the floor is a POSITIVE requirement. When yaw is below the
            # knee the floor is 0, and a zero floor is not a boundary anyone can sit on -- it is
            # just "no gap required". Flagging those would flag every aligned scenario in the set.
            if floor > offset and abs(abs(gap) - floor) < offset:
                offenders.append(f'{name} gap={gap!r} sits on the mu={friction} coupling floor '
                                 f'{floor!r}')
    assert not offenders, (
        'scenario(s) sit on a gate boundary and would be decided by rounding: '
        + '; '.join(offenders))


def test_the_scenarios_span_both_sides_of_the_coupling_floor():
    """Every friction must have a scenario the coupling admits and one it refuses.

    The coupling is the constraint that actually binds, so if no scenario straddles it at a given
    friction then neither soundness nor tightness is being tested there. This asserts the scenario
    set keeps that property, so a later edit that quietly drops the coupled axis fails here rather
    than silently reducing the gate to a box again.
    """
    offset = declared_align_margin()
    coupling = declared_coupling()
    safety = declared_floor_safety()
    windows = declared_windows()
    problems = []
    for friction in coupling:
        window = window_for(friction, windows)
        rows = floor_for(friction, coupling)
        admissible, refused = [], []
        for name, gap, step, yaw in declared_scenarios():
            if abs(yaw) > window['yaw_max'] or abs(gap) > window['gap_max']:
                refused.append(name)
                continue
            if floor_at(rows, abs(yaw)) * safety > abs(gap):
                refused.append(name)
            else:
                admissible.append(name)
        if not admissible:
            problems.append(f'mu={friction}: no scenario satisfies the coupling')
        if not refused:
            problems.append(f'mu={friction}: no scenario violates any limit')
        # The specific case the coupling exists to catch: a yawed entry with no gap.
        assert any(abs(yaw) > offset and abs(gap) <= offset for _, gap, _, yaw
                   in declared_scenarios()), (
            'no scenario has yaw with zero gap, so the coupling is never exercised')
    assert not problems, 'scenario set does not exercise the gate: ' + '; '.join(problems)



def declared_align_margin():
    """`ALIGN_BOUNDARY_EPS`, read as text, times ten to give a comfortable placement clearance."""
    src = (ROOT / 'experiments' / 'probe_deck.py').read_text(encoding='utf-8')
    tree = ast.parse(src)
    for node in tree.body:
        if isinstance(node, ast.Assign) and any(
                getattr(t, 'id', None) == 'ALIGN_BOUNDARY_EPS' for t in node.targets):
            return 10.0 * float(_eval_literal(node.value))
    raise AssertionError('probe_deck.ALIGN_BOUNDARY_EPS is no longer a literal')


def test_the_gap_floor_rises_with_friction_and_covers_the_sweep():
    """The gap a yawed tray needs must INCREASE with friction, and cover every swept mu.

    This is the mechanism the coupling encodes, so it is asserted rather than left in a comment.
    A yawed tray has to be straightened by the roller row before it makes contact; grippier rollers
    resist that correction, so MORE gap is needed at higher friction to buy the same yaw tolerance.

    The floor is tabulated, so "rises with friction" means: at every yaw where any friction asks for
    a positive gap, the requirement must not DECREASE with friction, and must strictly increase
    somewhere. It must also be conservative against the corridor that was measured.

    If someone later flattens this to a constant, or drops the coupling, this fails.
    """
    coupling = declared_coupling()
    safety = declared_floor_safety()
    mus = sorted(coupling)
    assert mus == sorted(rig.FRICTION_SWEEP), (
        f'the floor covers {mus} but the sweep is {rig.FRICTION_SWEEP}; an uncovered friction '
        f'would silently fall back to the worst-case floor')
    assert safety >= 1.0, f'GAP_FLOOR_SAFETY {safety} is below 1.0, so the floor has no margin'

    # Compare the frictions on the union of their yaw knots, so a friction cannot look cheap by
    # simply not tabulating the yaw the others are penalised for.
    knots = sorted({y for rows in coupling.values() for y, _ in rows})
    curves = {mu: [floor_at(coupling[mu], y) for y in knots] for mu in mus}
    for lower, upper in zip(mus, mus[1:]):
        for y, a, b in zip(knots, curves[lower], curves[upper]):
            assert a <= b + 1e-12, (
                f'the floor must not DECREASE with friction, but at yaw {y:.4f} rad it is '
                f'{a:.4f} m at mu={lower} and {b:.4f} m at mu={upper}')
    assert any(any(a < b - 1e-12 for a, b in zip(curves[mus[0]], curves[mus[-1]]))
               for _ in [0]), (
        'the floor is identical at the lowest and highest friction, so it is not encoding the '
        'mechanism -- check the derivation comment before flattening it')

    # It must also be conservative with respect to what was measured. Each entry is the smallest
    # gap observed to rescue that yaw, per friction. mu = 0.15 needs nothing up to 4 deg, which is
    # itself part of the shape: a floor that charged it anything would refuse workable cases.
    measured = {
        0.15: {},
        0.25: {math.radians(3.0): 0.002, math.radians(4.0): 0.006,
               math.radians(5.0): 0.010, math.radians(6.0): 0.015},
        0.40: {math.radians(2.0): 0.002, math.radians(2.5): 0.004,
               math.radians(3.0): 0.006, math.radians(4.0): 0.015},
    }
    for mu, points in measured.items():
        rows = floor_for(mu, coupling)
        for yaw, gap in points.items():
            required = floor_at(rows, yaw) * safety
            assert required >= gap - 1e-12, (
                f'mu={mu}: the floor requires {required:.4f} m of gap at {math.degrees(yaw):.1f} deg '
                f'but the rig needed {gap:.4f} m, so the gate would admit a case the rig failed')
    # And the zero-knee must survive: mu = 0.15 to 4 deg, and 0.25 / 0.40 to their own knees.
    for mu, knee in ((0.15, math.radians(4.0)), (0.25, math.radians(2.5)),
                     (0.40, math.radians(1.5))):
        rows = floor_for(mu, coupling)
        assert floor_at(rows, knee) * safety <= 1e-9, (
            f'mu={mu}: the floor demands gap below the measured knee at {math.degrees(knee):.1f} deg, '
            f'which would refuse cases that work with no gap at all')



def test_the_step_ceiling_rises_with_friction_and_stays_below_measurement():
    """The step axis is FRICTION-INDEXED, and a single scalar for it is both unsafe and over-tight.

    This is the defect the interlock-bypass arm exposed. The shipped window used step_max = 0.006
    for every friction, which is ABOVE what mu = 0.15 can do (it completes at 4 mm and fails at 5)
    and FAR BELOW what mu = 0.40 can do (it completes at 13 mm). One number was therefore
    simultaneously unsafe at the low end and over-tight at the high end, and `step_8mm@0.40`
    completed twice under bypass while the gate refused it.

    The declared value must sit strictly BELOW the measured ceiling at every friction, and must
    RISE with friction -- the same mechanism as the gap floor, since grip is what converts roller
    motion into tray motion.
    """
    tables = _declared_module_values({'STEP_MAX_BY_FRICTION', 'GAP_MAX_BY_FRICTION'})
    step = tables['STEP_MAX_BY_FRICTION']
    assert sorted(step) == sorted(rig.FRICTION_SWEEP), (
        f'the step ceiling covers {sorted(step)} but the sweep is {rig.FRICTION_SWEEP}')
    # Measured ceilings: the largest offset observed to complete, per friction.
    measured_ceiling = {0.15: 0.004, 0.25: 0.006, 0.40: 0.013}
    measured_fail = {0.15: 0.005, 0.25: 0.008, 0.40: 0.014}
    mus = sorted(step)
    values = [step[m] for m in mus]
    assert all(a <= b for a, b in zip(values, values[1:])), (
        f'the step ceiling must not DECREASE with friction, but the table gives {values}')
    assert values[-1] > values[0], (
        'the step ceiling is flat across friction, so it is not encoding the mechanism')
    for mu in mus:
        assert step[mu] >= measured_ceiling[mu] - 1e-12, (
            f'mu={mu}: step_max {step[mu]} is BELOW the measured ceiling {measured_ceiling[mu]}, '
            f'so the gate refuses a case the rig demonstrably completes')
        assert step[mu] < measured_fail[mu], (
            f'mu={mu}: step_max {step[mu]} is at or above the first measured FAILURE '
            f'{measured_fail[mu]}, so the gate would admit a case the rig failed')


def test_the_yaw_limit_is_documented_as_a_sensor_limit_not_a_mechanical_one():
    """The yaw ceiling comes from sensor line-of-sight, and the code must say so.

    A direct scan found every yaw from 6 to 12 deg REFUSED -- not because the transfer would fail
    but because the two gap rays MISS the tray, which trips the sensor-miss branch and refuses
    unconditionally (a sensor miss is never bypassed). So above roughly 8 deg the harness cannot
    evaluate the mechanism at all.

    Below that, the envelope is NON-MONOTONE: at mu = 0.15 with 24 mm of gap, 8 deg fails and
    10 deg succeeds, reproducibly over five identical runs, because the rig's own pitch
    (0.080 m) exceeds 2 * ROLLER_RADIUS (0.070 m), leaving a hole between crowns that a yawed
    corner drops into at some angles and not others.

    A non-monotone envelope cannot be represented by any threshold, so the honest claim is the
    narrower one. This test asserts the code carries that caveat and does not present `yaw_max`
    as a mechanical tolerance.
    """
    src = (ROOT / 'experiments' / 'probe_deck.py').read_text(encoding='utf-8')
    assert 'YAW_CEILING_REASON' in src, (
        'the yaw ceiling must carry an explicit reason constant, so a reader cannot mistake it '
        'for a mechanical tolerance')
    reason = _declared_module_values({'YAW_CEILING_REASON'})['YAW_CEILING_REASON']
    assert 'sensor' in reason.lower(), (
        f'the yaw ceiling reason {reason!r} does not mention the sensor, which is what actually '
        f'sets it')
    # The hole is real geometry, and it is the cause: pitch must not silently exceed two radii
    # without the caveat being present.
    if rig.ROLLER_PITCH > 2.0 * rig.ROLLER_RADIUS:
        assert 'NON-MONOTONE' in src or 'non-monotone' in src, (
            'the rig has a hole between roller crowns (pitch > 2r) but the probe does not record '
            'that this makes the yaw envelope non-monotone')
    assert rig.ROLLER_PITCH > 2.0 * rig.ROLLER_RADIUS, (
        'the pitch-vs-2r relationship changed; re-derive the yaw section, because the '
        'non-monotonicity finding assumed a hole between crowns')

# ---------------------------------------------------------------------------------------
# Acceptance-scope contract.
#
# The reviewer's first instruction was "unify the acceptance scope", and the concrete failure
# it removes is a specific one that had already happened in this project: a partial component
# experiment being readable as the complete flow. These tests make the two claims that could
# be silently conflated -- "the receiving-side component works" and "the C transfer works" --
# impossible to merge by accident in the source, because the stage lists are what separate
# them and they must be consistent with each other.
# ---------------------------------------------------------------------------------------

PROBE_SRC = (ROOT / 'experiments' / 'probe_deck.py').read_text(encoding='utf-8')
JUDGE_SRC = (ROOT / 'experiments' / 'evaluate_c_transfer.py').read_text(encoding='utf-8')


def probe_module():
    """Import the probe without running it, with `experiments/` on the path for roller_rig."""
    import sys
    exp = str((ROOT / 'experiments').resolve())
    if exp not in sys.path:
        sys.path.insert(0, exp)
    import probe_deck
    return probe_deck


def test_the_whole_C_claim_is_derived_and_not_hard_coded():
    """`full_C_acceptance` must be computed, because a typed assertion cannot go stale loudly.

    The antecedent is real: this field was the literal string 'NOT_RUN' in the report and then
    hard-coded AGAIN in the judge, which overwrote whatever the probe had said. Two independent
    hand-written copies of one fact is how the artifact ended up showing 12 PASS checks next to
    a value that looked like it was contradicting them, with nothing explaining why.
    """
    assert "full_C_acceptance': _full_c_state()" in PROBE_SRC, (
        'the probe must DERIVE the whole-C claim from its own stage list')
    assert "full_C_acceptance': 'NOT_RUN'" not in PROBE_SRC, (
        'the literal whole-C claim is back in the probe; it cannot notice when it becomes wrong')
    assert "full_C_acceptance': 'NOT_RUN'" not in JUDGE_SRC, (
        'the judge is asserting the whole-C claim again instead of carrying the probe\'s value '
        'through; the judge checks claims, it does not make them')


def test_the_required_stage_list_covers_the_real_transfer_end_to_end():
    """Whole-C needs the source side, the traverse, the transfer AND the unload.

    If a stage is dropped from this list the derivation silently starts returning CLAIMED for a
    partial rig, which is the exact silent upgrade this whole mechanism exists to stop.
    """
    m = probe_module()
    required = set(m.FULL_C_REQUIRED_STAGES)
    for stage in ('CHECK_ALIGN', 'CONVEY', 'RELEASE', 'SETTLE', 'DEPLOY_PUSHER', 'EMBARK'):
        assert stage in required, f'{stage} is part of the transfer but is not required for C'
    for stage in ('UNLOAD', 'RECEIVED', 'PLATFORM_CLEAR'):
        assert stage in required, (
            f'{stage} is the receiving end of the transfer; without it "C complete" would mean '
            f'"the tray was picked up"')
    # The claim must follow the stages, in BOTH directions. This used to assert that some
    # required stage was necessarily absent, which was true while the machine stopped at
    # EMBARK and is false now that UNLOAD/RECEIVED/PLATFORM_CLEAR exist. Asserting a fixed
    # answer here would have frozen the rig at its most incomplete revision; asserting the
    # RELATIONSHIP is what actually protects the claim.
    missing = set(m.FULL_C_REQUIRED_STAGES) - set(m._stages_observed())
    claimed = m._full_c_state()
    assert (claimed == 'CLAIMED') == (not missing), (
        f'full_C_acceptance is {claimed!r} but {len(missing)} required stage(s) are missing '
        f'({sorted(missing) or "none"}); the claim and the stage list disagree')
    if claimed == 'CLAIMED':
        # If the rig now claims the whole transfer, the receiving end must really be in the
        # machine and not merely named in the list.
        for stage in ('UNLOAD', 'RECEIVED', 'PLATFORM_CLEAR'):
            assert stage in m._stages_observed(), (
                f'the whole-C claim is CLAIMED but {stage} is not a real stage in the machine')


def test_the_stage_detector_sees_every_transition_form():
    """The detector must catch BOTH assignment forms, and must not under-report.

    Its first version only matched `state = 'X'` and therefore missed RELEASE, SETTLE,
    DEPLOY_PUSHER and EMBARK -- four of the seven stages this machine actually has. An
    under-reporting detector is worse than none, because it fails QUIETLY in the direction of
    claiming less was done, and the next person reads a "missing stage" that is really a
    missing regex.

    UPDATED (D034). This list used to name `RAISE_PIN`, a stage that the pin-to-pusher
    redesign REMOVED in favour of `DEPLOY_PUSHER`. The rename is exactly the event this test
    exists to catch, and it is worth recording that the test did NOT catch it cleanly: it was
    already red for the stale name, so the rename landed inside an existing failure and read
    as more of the same noise. The stage list is now asserted against the probe's own
    `FULL_C_REQUIRED_STAGES` where possible, so a future rename has to update ONE place.
    """
    m = probe_module()
    observed = m._stages_observed()
    for stage in ('CHECK_ALIGN', 'CONVEY', 'RELEASE', 'SETTLE', 'DEPLOY_PUSHER', 'EMBARK', 'DONE'):
        assert stage in observed, f'{stage} exists in probe_deck.py but the detector missed it'
    assert 'RAISE_PIN' not in observed, (
        'RAISE_PIN is back in the state machine; if the pin was genuinely re-introduced, '
        're-derive the pusher section and this test together rather than renaming one of them')
    for transition in ('REFUSED', 'STALLED', 'WALL_TIMEOUT', 'OVERTRAVEL',
                       'PAYLOAD_LOST'):
        assert transition in observed, (
            f'{transition} is a terminal state and must be visible, so that a failure cannot be '
            f'mistaken for an unmodelled stage')


def test_the_judge_fails_if_the_whole_C_claim_disagrees_with_the_stage_list():
    """A claim and its evidence living in one file must not be allowed to contradict.

    The judge's new check is `consistent`-shaped, i.e. it FAILS in both directions: claiming
    CLAIMED with stages missing, and claiming NOT_RUN with all stages present (the latter would
    be a stale claim, which is how the ambiguity started). This test asserts the check is a
    real two-sided constraint and not a note.

    The scope-text half is asserted by SHAPE rather than by naming a sentence. It used to pin the
    sentence verbatim -- which made the guarantee itself a typed constant, and it went stale
    exactly as typed constants do: once every stage existed, the pinned sentence read "Whether C
    as a whole passes is NOT_RUN" beside a computed "0 stage(s) ... never executed". The
    behavioural proof of both directions now lives in tests/test_p1_c_transfer.py:
    `test_every_stage_present_makes_the_scope_sentence_say_so` and
    `test_a_missing_stage_makes_the_scope_sentence_say_so`.
    """
    tree = ast.parse(JUDGE_SRC)
    names = [n.name for n in ast.walk(tree) if isinstance(n, ast.FunctionDef)]
    assert 'main' in names
    assert 'the whole-C claim is not made without its stages' in JUDGE_SRC, (
        'the whole-C consistency check is missing from the judge')
    assert "(claim == 'NOT_RUN' and missing) or (claim == 'CLAIMED' and not missing)" in JUDGE_SRC, (
        'the whole-C check must be two-sided: it fails on a stale NOT_RUN as well as on an '
        'unearned CLAIMED')
    assert 'scope_note' in JUDGE_SRC, (
        'the whole-C check must build its scope sentence from a DERIVED variable, not a typed one')
    assert 'f"*** {scope_note} ***"' in JUDGE_SRC, (
        'the scope sentence must be interpolated into the check detail, so the number and its '
        'scope cannot be quoted apart')
    assert 'Whether C as a whole passes is NOT_RUN' in JUDGE_SRC, (
        'the absent-stage branch must still say a whole-C verdict cannot be decided')
    assert 'a real-AMR docking is NOT_RUN' in JUDGE_SRC, (
        'the every-stage-present branch must still carry the declared scope')
    assert 'A PASS here is a statement about the receiving-side component ONLY' not in JUDGE_SRC, (
        'the typed sentence must not come back: with every stage present it asserted "C is '
        'NOT_RUN" beside a computed "0 stage(s) ... never executed"')


def test_the_scope_string_says_mechanism_experiment_not_amr_docking():
    """The reviewer's wording requirement, enforced rather than remembered."""
    m = probe_module()
    claim = m.SCOPE_CLAIM.lower()
    assert 'mechanism' in claim, 'the scope must call this a mechanism experiment'
    assert 'not a real-amr docking' in claim or 'not a real amr docking' in claim, (
        'the scope must deny being a real-AMR docking')


# ---------------------------------------------------------------------------------------
# Tightness: three categories, not one verdict.
#
# The reviewer was right that "everything refused must fail when the gate is opened" is the
# wrong criterion, and right for a reason worth writing down: a SAFETY interlock is allowed to
# refuse conservatively. A refusal that would in fact have worked is a cost, not a hazard, and
# collapsing both into FAIL teaches the reader to distrust the word FAIL.
#
# But it is NOT acceptable to simply stop failing. Turning the FAIL into a note would be
# "silently upgrading NOT_RUN to PASS" wearing a different hat: after the change, a report full
# of unjustified refusals would read exactly like a report with none. The categories below are
# therefore deliberately ASYMMETRIC -- excessive conservatism is reported and DOES fail the
# check (because a window that refuses workable cases is an undiscovered defect and the project
# must not ship one), while the residual case the reviewer named, a refusal nobody can decide,
# is reported as NOT_RUN rather than as a pass.
# ---------------------------------------------------------------------------------------

# The three categories that live in the BYPASS report. `admitted_then_failed` is deliberately
# NOT among them and is NOT required to appear: nothing in a bypass set was admitted, so the
# critical class is unobservable there and is owned by the armed arm's soundness check.
BYPASS_CATEGORIES = ('refused_but_completable', 'sensor_unavailable', 'justified')


def test_the_judge_reports_tightness_in_three_categories():
    """The three-way split must exist in the judge, with the reviewer's semantics."""
    for category in BYPASS_CATEGORIES:
        assert category in JUDGE_SRC, f'category {category!r} is missing from the tightness check'
    assert 'would_have_completed' in JUDGE_SRC
    assert 'sensor_unavailable' in JUDGE_SRC, (
        'a refusal caused by the ray missing the tray is undecidable, not a conservatism win')
    # The critical class must be NAMED as belonging to the soundness check, not silently
    # dropped -- dropping it is how the reviewer's first category would disappear.
    assert 'admitted-then-failed' in JUDGE_SRC, (
        'the critical class must still be named in the report, even though a bypass set cannot '
        'measure it, so a reader is told where it is measured instead of assuming it was fine')
    assert 'soundness check on the armed arm' in JUDGE_SRC, (
        'the report must say WHERE the critical class is owned')


def test_a_refusal_nobody_can_decide_is_not_reported_as_a_pass():
    """The residual is NOT_RUN when it is undecidable -- never a discretionary pass.

    The reviewer's third category is the one that must not become a loophole: if the sensor could
    not see the tray, the case tells us nothing about the mechanism. It is excluded from the
    conservatism count, and if that leaves the claim with no decidable evidence at all the check
    must be NOT_RUN, so it cannot be rolled up into a PASS.
    """
    assert "'NOT_RUN' if not decidable" in JUDGE_SRC or 'undecidable' in JUDGE_SRC, (
        'the judge must have an explicit undecidable path for the tightness claim')
    # And the residual must be excluded from the count that drives the verdict.
    assert 'refused_but_completable' in JUDGE_SRC and 'sensor_unavailable' in JUDGE_SRC
    tree = ast.parse(JUDGE_SRC)
    text_nodes = [n.value for n in ast.walk(tree)
                  if isinstance(n, ast.Constant) and isinstance(n.value, str)]
    joined = ' '.join(text_nodes)
    assert 'conservatism' in joined.lower(), (
        'the refused-but-completable category must be NAMED as conservatism in the report text, '
        'so a reader is not left to infer intent from a bare count')


# ---------------------------------------------------------------------------------------------
# D031 contract tests
#
# Both defects below were found in round 3 by reading a failing run's own trace, not by reading
# the source. Each one is written so that a future edit which undoes the fix makes the test
# fail: the constants are checked for being DERIVED, and the physical claims are recomputed.
# ---------------------------------------------------------------------------------------------

def _module_constant(module_name, const_name, source_path):
    """Source text of a module-level assignment, looked up by name. None if absent."""
    tree = ast.parse(source_path.read_text(encoding='utf-8'))
    for node in tree.body:
        if isinstance(node, ast.Assign):
            for tgt in node.targets:
                if isinstance(tgt, ast.Name) and tgt.id == const_name:
                    return ast.unparse(node.value)
    return None


# NOTE: `PROBE_SRC` is already defined above as the probe's SOURCE TEXT. Re-using that
# name for a Path silently shadowed it and broke an earlier test -- so these two are
# named for what they are.
PROBE_PATH = ROOT / 'experiments' / 'probe_deck.py'
RIG_PATH = ROOT / 'experiments' / 'roller_rig.py'
BARE_NUMBER = re.compile(r'-?\d+(?:\.\d+)?')


# ---- D031-a: SUPERSEDED BY D032/D034 -- the retention pin no longer exists ------------------
# The original defect was real: the pin sat at deck-local 0.6700 while the seated tray's
# leading edge was at 0.4994, so `pin_contacts` was 0 in every recorded run. The three tests
# below used to record that defect under `xfail(strict=True)`.
#
# The pin was then replaced by the two-DOF carriage+blade pusher (D032), and D034 measured
# that the blade now has a different, worse problem: it cannot stow anywhere that is both
# clear of the tray and able to reach the wall. The defect has NOT been fixed -- it has MOVED,
# and the tests that should now be failing are the swept-volume ones in
# `test_the_stowed_blade_clears_both_roller_rows_and_the_deck_end` above and the D034 block at
# the end of this file.
#
# So the `PIN_X` / `PIN_HALF` names below are kept as aliases in roller_rig.py, but the object
# they describe is gone, and these tests are DELETED rather than re-marked: an xfail marker on
# a test whose subject no longer exists records nothing. The two markers were already XPASSING
# (i.e. failing the suite) when this was written, which is the signal D031 said to watch for.
#
# What is kept is the one assertion that still has force -- that the mechanism's x is DERIVED
# and not typed in -- re-based onto the constants that now actually determine it.


def test_the_pusher_x_is_derived_from_the_wall_band_and_not_typed():
    """`PUSHER_X` must resolve, through aliases, to an expression over the wall constants.

    REWRITTEN (D034). The old form demanded `PUSHER_X` reference `DECK_ROLLER_X0`,
    `ROLLER_PITCH` and `DECK_ROLLERS`. That encoded the OLD mechanism's placement rule -- "the
    pin sits between the deck's roller rows" -- which is precisely what made the pin
    unreachable. The current placement rule is different and is derived from a different
    quantity: the retracted blade must sit BEHIND the furthest-back possible tray wall, which
    is `PUSHER_WALL_REAR_LO`. Keeping the old name list would have forced a correct file to
    fail, so the names are updated to the ones that now carry the derivation.

    The mechanism is worth stating because it is the whole reason this test exists: if someone
    types a number here, the blade stops tracking the seat-sensor window, and the window is
    what tells the rig where the wall is (0.100 m of uncertainty -- see `BEAM_SEAT_A/B_X`).
    """
    expr = _module_constant('roller_rig', 'PUSHER_X', RIG_PATH)
    assert expr is not None, 'PUSHER_X is not a module-level assignment in roller_rig.py'
    seen = {'PUSHER_X'}
    for _ in range(4):
        if expr in seen:
            break
        seen.add(expr)
        nxt = _module_constant('roller_rig', expr, RIG_PATH)
        if nxt is None:
            break
        expr = nxt
    assert not BARE_NUMBER.fullmatch(expr), f'PUSHER_X resolves to a bare literal: {expr}'
    for name in ('PUSHER_WALL_REAR_LO', 'PUSHER_HALF', 'PUSHER_RETRACT_MARGIN'):
        assert name in expr, (
            f'PUSHER_X resolves to {expr!r}, which does not reference {name}; the stowed blade '
            f'position must be derived from the wall band, not typed in'
        )


def test_the_retracted_blade_clears_the_furthest_forward_tray_wall():
    """The stow position must be behind the worst case, and the deploy must close that gap.

    This is the assertion that replaces D031-a's pair. The pin's failure was one of
    REACHABILITY (it was too far forward to be touched); the blade's is the opposite one --
    it must be far enough BACK to never sit ahead of an early-stopping tray, and then travel
    far enough FORWARD to reach the latest one. Both halves are checked, because satisfying
    only the first is what a hand-placed constant usually does.
    """
    blade_face = rig.PUSHER_X + rig.PUSHER_HALF[0]
    assert blade_face <= rig.PUSHER_WALL_REAR_LO + 1e-9, (
        f'the retracted blade face ({blade_face:.4f}) is ahead of the furthest-back wall '
        f'({rig.PUSHER_WALL_REAR_LO:.4f}): it would block a tray that stopped early'
    )
    deployed_face = blade_face + rig.PUSHER_DEPLOY_TRAVEL
    assert deployed_face >= rig.PUSHER_WALL_REAR_HI - 1e-9, (
        f'fully deployed the blade face reaches {deployed_face:.4f}, short of the furthest-'
        f'forward wall {rig.PUSHER_WALL_REAR_HI:.4f}: it could not push a late-stopping tray'
    )


# ---- D031-b: the EMBARK roller surface must not outrun the deck -------------------------------
# Measured fault: 5.0 rad/s * 0.035 m = 0.1750 m/s against a deck translating at 0.1500 m/s.
# During EMBARK the deck moved 0.2400 m while the tray moved 0.4708 m: 0.2308 m of
# uncompensated forward slide, which is what carried the tray's leading edge past the receiver.
#
# UPDATED (D034). The FIX for this changed shape. D031-b answered "slow the rollers down to a
# fraction of deck speed" (`EMBARK_ROLLER_SPEED`, derived from `DECK_SPEED`). D033 then measured
# that NO roller speed works: the row's drive direction is intrinsically the convey-OUT
# direction, so any nonzero speed ADDS relative slip rather than removing it (sweep: ratio 0.982
# at v=0, rising to 3.053 and a fall at v=1.10). The EMBARK rollers are therefore CUT, and the
# whole retention duty moved to friction drag (`EMBARK_DECK_ROLLER_SPEED = 0.0`, F1, 3/3).
#
# So `EMBARK_ROLLER_SPEED` still exists -- CONVEY and RELEASE legitimately use it to send the
# tray onto the deck -- but it is no longer what governs EMBARK, and the assertion below is
# re-pointed at the constant that does. The original claim is still checked, because a
# regression that put a spinning roller back into EMBARK is exactly what D033 forbids.

def test_the_embark_roller_speed_is_derived_from_the_deck_speed():
    """The CONVEY-side roller speed must stay derived; it still carries the tray aboard."""
    expr = _module_constant('probe_deck', 'EMBARK_ROLLER_SPEED', PROBE_PATH)
    assert expr is not None, 'EMBARK_ROLLER_SPEED is not a module-level assignment'
    assert not BARE_NUMBER.fullmatch(expr), (
        f'EMBARK_ROLLER_SPEED is a bare literal ({expr}); deriving it from DECK_SPEED is what '
        f'stops a later edit from silently defeating the transfer onto the deck'
    )
    assert 'DECK_SPEED' in expr, f'EMBARK_ROLLER_SPEED ({expr!r}) does not use DECK_SPEED'


def test_the_embark_rollers_are_cut_because_no_speed_carries_the_tray():
    """The EMBARK phase must command ZERO roller speed, and the constant must say why.

    This is the D033 finding written as a contract. `EMBARK_DECK_ROLLER_SPEED` is a named
    constant rather than an inline `0.0` precisely so that this test can pin it and so that a
    reader who is about to "restore the rollers" meets the measurement first.

    The swept-ratio evidence is asserted as a comment-carrying requirement: the constant's own
    block must record that the relationship was measured and is monotone in the WRONG direction.
    """
    expr = _module_constant('probe_deck', 'EMBARK_DECK_ROLLER_SPEED', PROBE_PATH)
    assert expr is not None, (
        'EMBARK_DECK_ROLLER_SPEED is not a module-level assignment; the EMBARK roller speed '
        'must be named so this contract can pin it')
    assert float(expr) == 0.0, (
        f'EMBARK_DECK_ROLLER_SPEED is {expr}, not 0.0. Any nonzero value re-introduces the '
        f'slip D033 measured: the deck row drives the conveyor-OUT direction, so a spinning '
        f'roller ADDS relative motion instead of removing it')
    src = (ROOT / 'experiments' / 'probe_deck.py').read_text(encoding='utf-8')
    # The measurement must travel with the constant. It may live either in the probe's own
    # comment block or in DECISIONS.md, but at least one of them must carry the two endpoints of
    # the sweep: a bare `0.0` is indistinguishable from a typo, and "we tried slowing the rollers
    # and it did not work" is not reproducible without the numbers.
    decisions = (ROOT / 'docs' / 'DECISIONS.md').read_text(encoding='utf-8')
    assert 'D033' in src or 'D033' in decisions, (
        'neither probe_deck.py nor DECISIONS.md cites D033; the measurement that justifies '
        'cutting the EMBARK rollers must be recorded somewhere a reader will find it')
    for marker in ('0.982', '3.053'):
        assert marker in decisions, (
            f'the roller-speed sweep endpoint {marker} is no longer recorded; the sweep is the '
            f'evidence, and without it "cut the rollers" is an assertion')


def test_the_embark_roller_surface_does_not_outrun_the_deck():
    """Kept as a guard on the CONVEY-side constant, which still must not outrun the deck."""
    deck_speed = float(_module_constant('probe_deck', 'DECK_SPEED', PROBE_PATH))
    margin = float(_module_constant('probe_deck', 'EMBARK_SPEED_MARGIN', PROBE_PATH))
    surface = (margin * deck_speed / rig.ROLLER_RADIUS) * rig.ROLLER_RADIUS
    assert surface <= deck_speed + 1e-12, (
        f'roller surface {surface:.6f} m/s exceeds deck ground speed {deck_speed:.6f} m/s: '
        f'the tray is thrown forward past the point it is supposed to be held against'
    )


# ---------------------------------------------------------------------------------------------
# D034 contract: the blade must not intersect the tray's swept volume, in ANY state.
#
# This is the test whose absence cost a whole round, and the lesson is worth stating in the
# form it was learned: **"commanded to 0" is not "out of the way".**
#
# The state machine held the carriage retracted and reported `pusher_slide = 0`, which reads as
# "the pusher is stowed". It was: stowed is exactly where the blade sits INSIDE the tray. The
# blade occupies world x 1.7450..1.7650, z 0.4860..0.5160; the tray floor's underside is at
# world z 0.4850. So the blade was 1 mm above the floor plane and 10 mm inside the tray's
# footprint, overlapping the tray body over x 1.7450..1.7650 -- and every subsequent CONVEY
# shoved the tray sideways off the deck (final tray y = 0.5427, i.e. gone off the side).
#
# Measured A/B: pusher present -> the tray stalls at world x 1.6799. Pusher stripped from the
# model entirely -> 2.5032. Difference 0.8232 m. This also retroactively explains the earlier
# `p1-c-rig-38` symptom, where the tray appeared to "freeze" for 45 s: it was not a state-machine
# bug, it was physical jamming.
#
# The assertion is deliberately SWEPT-VOLUME and not point-in-time, because the failure mode is
# precisely the one a point check misses: at t=0 the blade is behind the tray and everything
# looks fine; the collision happens only once the tray has travelled forward into it.
# ---------------------------------------------------------------------------------------------

TRAY_FLOOR_BOTTOM_Z = -0.0400    # tray floor's underside, in deck-local z (crown plane = 0)
TRAY_FLOOR_TOP_Z = -0.0300       # tray floor's top face -- the interior floor surface


def tray_swept_volume_deck_local():
    """Deck-local x span the tray sweeps while it is anywhere on OR APPROACHING the deck.

    THE APPROACH IS THE PART THAT MATTERS. The span has to open BEHIND the deck's lead edge,
    because the tray arrives from behind and the pusher corridor -- the only place on the deck a
    body can hide at every z -- sits in exactly that region.

    This function has now been wrong twice, both times in the UNSAFE direction (under-estimating,
    so the blade looked clear of the tray):
      * version 1 subtracted `half` twice and reported a rear extent of +0.05;
      * version 2 opened the span at the rearmost SEATED centre, `min(A, B) - half` = 0.0500,
        which is FORWARD of the deck lead edge (-0.0100). The seat window bounds where the tray
        may come to REST, not where it may be while it is being conveyed in, so the corridor fell
        outside the span and the intersection check could not see the blade at all.

    Version 2 did not fail loudly. The sanity assertion in the test DID catch the under-estimate,
    and `xfail(strict=True)` then converted that assertion error into an "expected failure", which
    read as a recorded product defect for a whole round while the check tested nothing. AN XFAIL
    HIDES A BUG IN THE TEST AS READILY AS ONE IN THE PRODUCT, and a marker that is always
    tripping cannot report a second fault.

    Both rails are derived rather than chosen:
      * REAR: the tray centre at which its leading face reaches the rear face of the pusher
        corridor, `PUSHER_X - PUSHER_HALF[0] - half` = -0.2400. D034 measured the jam at
        deck-local -0.2401, so this bound reproduces the measured stall position.
      * FRONT: the tray must be able to sit fully on the deck, so its leading edge reaches
        `PUSHER_FORWARD_LIMIT + half`.
    """
    tray = rig.tray_measurements(TRAY)
    half = (tray['x_max'] - tray['x_min']) / 2.0
    front_centre = rig.PUSHER_FORWARD_LIMIT - half
    rear_extent = (rig.PUSHER_X - rig.PUSHER_HALF[0]) - half
    return (rear_extent, front_centre + half)


def test_the_blade_cannot_hide_below_the_crown_plane():
    """The blade's stow height must be provably clear of the roller band, not merely commanded low.

    This half of D034 PASSES today: `PUSHER_STOW_Z = PUSHER_CROWN_CLEARANCE = 0.0010` puts the
    blade's bottom face 1 mm ABOVE the crown plane. Because the crown plane is the top of the
    roller band, the blade cannot plough the fixed row -- which is the D032 defect.

    Note carefully what this test does NOT claim. "Above the crown plane" is a statement about
    the ROLLERS. It says nothing about whether the blade is inside the tray, and in fact the
    blade is entirely inside the tray's hollow interior: the tray floor's underside is at
    deck-local z -0.0400 and the blade occupies +0.0010..+0.0310, i.e. 41 mm ABOVE the floor's
    underside, sitting over the floor. That is the still-broken property, asserted in the marked
    test below. The old pin test conflated the two by asserting `z < 0` "hides the pin", which
    is why it could never fail for the right reason.
    """
    assert rig.PUSHER_STOW_Z >= 0.0, (
        f'the blade stows at z {rig.PUSHER_STOW_Z:.4f}, below the crown plane; it would be '
        f'inside the roller band and would plough the fixed row (the D032 defect)')
    _, band_top = roller_band()
    assert rig.PUSHER_STOW_Z >= band_top - 1e-9, (
        f'the blade stows at z {rig.PUSHER_STOW_Z:.4f}, below the roller band top {band_top:.4f}')
    # And confirm the MODEL actually emits that, not just the constant saying so.
    blade = deck_children()['rear_pusher']
    assert blade[4] >= band_top - 1e-9, (
        f'the emitted blade bottom is {blade[4]:.4f} (deck-local), inside/below the roller band '
        f'top {band_top:.4f}')


def test_the_blade_is_above_the_tray_floor_which_is_why_it_is_inside_the_tray():
    """The measurement that makes D034 unambiguous: the blade sits OVER the tray's floor.

    Recorded as its own passing test because the geometry is counter-intuitive and a reader
    will otherwise assume "above the crown plane" means "below the tray". It does not. The
    crown plane is the crown of the ROLLERS; the tray's own floor hangs 40 mm BELOW its body
    origin, so "just above the crown plane" is 41 mm up inside the tray.

    This is asserted as a positive measurement rather than left to the x-overlap test, so that
    the two failure modes stay separable: one says the blade is in the wrong PLACE in z, the
    other says it is in the right place but the tray still runs into it.
    """
    blade = deck_children()['rear_pusher']
    assert blade[4] > TRAY_FLOOR_BOTTOM_Z, (
        f'the blade bottom ({blade[4]:.4f}) is below the tray floor underside '
        f'({TRAY_FLOOR_BOTTOM_Z:.4f}); this test documents the opposite defect and is stale')
    # The blade bottom sits ABOVE the floor's TOP face, i.e. it is genuinely up inside the tray's
    # hollow, over the floor. If it ever drops BELOW the floor surface the mechanism has changed
    # shape (it would then be scoring the floor rather than standing in the interior), and the
    # D034 account in the docstring no longer describes the rig.
    assert blade[4] >= TRAY_FLOOR_TOP_Z, (
        f'the blade bottom ({blade[4]:.4f}) is below the tray floor TOP face '
        f'({TRAY_FLOOR_TOP_Z:.4f}); the blade is inside the floor slab rather than standing in '
        f'the tray interior, and the D034 mechanism description is stale')
    # And the blade must rise far enough to press a wall: the wall band is [-0.0350, +0.0250].
    assert blade[5] > TRAY_FLOOR_BOTTOM_Z, (
        'the blade does not rise above the tray floor underside, so it could not push a wall')


def test_the_stowed_blade_does_not_intersect_the_trays_swept_volume():
    """KNOWN DEFECT (D034). The blade stows INSIDE the tray's swept footprint.

    This test is marked xfail(strict=True) for the same reason D031's were: it records a
    measured defect that is not yet fixed, and it will fail LOUDLY -- XPASS under strict mode --
    the moment the geometry is changed so the blade no longer overlaps the tray. That XPASS is
    the signal to delete the marker and re-run the transfer for real.

    The defect, measured: with the blade stowed the tray cannot traverse the deck. It stalls at
    world x 1.6799 against 2.5032 with the pusher removed -- a 0.8232 m difference -- and the
    contact pair in the stall trace is
        fixed_roller_2_5|tray_floor ; rear_pusher|tray_floor ; rear_pusher|tray_wall_xp
    The tray is 0.23 m half-width against a blade of 0.12 m, so it is shoved SIDEWAYS and ends at
    y = 0.5427, off the side of the deck.

    WHY IT IS NOT FIXED HERE: the two properties "does not touch the tray" and "can reach the
    wall" are jointly unsatisfiable at this pitch. See DECISIONS.md D034 section four for the
    interval proof, and D032's R1-R4 for the independent first proof. Resolving it is a
    MECHANISM decision (G1: remove the pusher and use a forward-end `Stopper`; G2: change the
    pitch), and it is the user's to make -- not a test's.
    """
    blade = deck_children()['rear_pusher']          # deck-local AABB, (x0,x1,y0,y1,z0,z1)
    sweep_x0, sweep_x1 = tray_swept_volume_deck_local()
    # Sanity: the span must be at least as wide as one tray and must reach behind the deck's
    # lead edge. If this fails the SPAN is wrong, and the overlap verdict below means nothing.
    tray = rig.tray_measurements(TRAY)
    half = (tray['x_max'] - tray['x_min']) / 2.0
    assert (sweep_x1 - sweep_x0) >= 2.0 * half - 1e-9, (
        f'the swept span {(sweep_x1 - sweep_x0):.4f} is narrower than one tray '
        f'{2.0 * half:.4f}: the span is under-estimated and cannot detect an intrusion')
    assert sweep_x0 <= rig.DECK_LEAD_GAP, (
        f'the swept span starts at {sweep_x0:.4f}, forward of the deck lead edge '
        f'{rig.DECK_LEAD_GAP:.4f}: the tray approaches from behind, so the span must include it')
    overlap_x = min(blade[1], sweep_x1) - max(blade[0], sweep_x0)
    # The tray's own band in deck-local z: underside on the crown plane, top at the tray's top.
    overlap_z = min(blade[5], rig.TRAY_TOP_DECK_LOCAL) - max(blade[4], 0.0)

    # Overlap in x is EXPECTED and asserted, so this test cannot quietly go vacuous: the tray
    # sweeps the whole deck, so its x footprint covers the corridor whatever the stow height.
    assert overlap_x > 1e-9, (
        'the stowed blade no longer lies under the tray sweep in x at all -- that is a different '
        'mechanism and this test has gone vacuous. Re-derive the stow instead of deleting a check')
    assert overlap_z <= 1e-9, (
        f'the stowed blade spans deck-local z {blade[4]:.4f}..{blade[5]:.4f} while the tray '
        f'occupies 0.0000..{rig.TRAY_TOP_DECK_LOCAL:.4f}: they overlap over {overlap_z:.4f} m, so '
        f'the blade is back inside the tray. A stow is accepted by SWEPT-VOLUME intersection '
        f'(all three axes), never by an x footprint alone and never by "the command is 0"')


def test_the_probe_does_not_treat_a_zero_command_as_stowed():
    """The code must not infer "out of the way" from `ctrl == 0` anywhere.

    The generalised lesson from D034 (see MEMORY/LESSONS): a stowed position is a geometric
    claim. This test is a text-level guard on the probe so that the reasoning cannot silently
    regress to the command-value form while the swept-volume test above stays xfailed -- which
    is exactly how the defect survived a rewrite.
    """
    src = (ROOT / 'experiments' / 'probe_deck.py').read_text(encoding='utf-8')
    assert 'PUSHER_DEPLOY_NO_CONTACT' in src, (
        'the `PUSHER_DEPLOY_NO_CONTACT` fault vanished; it is deliberately retained in the '
        'vocabulary so that the designed-out contact condition stays visible')
    assert 'D034' in src or 'swept' in src.lower(), (
        'the probe never cites D034 or mentions the tray sweep in relation to the blade; the '
        'geometric argument for the stow position has to live in the source, not only in the '
        'tests')


def test_the_probe_latches_payload_loss_and_ends_the_run():
    """A dropped payload must LATCH and STOP, not be recorded and ignored.

    Measured defect this exists to prevent (`p1-c-f1-01`): `first_fall_t` was assigned and then
    read by NOTHING -- not by the state machine, and not by the judge, which had zero references
    to `fell_at`. The tray hit the floor at 54.808 s and the machine went on commanding the deck,
    the rollers and the pusher for a further 6.0 s, ending with `faulted = False` and
    `final_state = 'EMBARK'`. Payload loss was invisible to everything except a human reading a
    number in a JSON blob.

    The ORDER of the assertions matters as much as their content: if detection moves back below
    the state machine, a state whose premise has been destroyed can still act.
    """
    src = PROBE_SRC
    assert "state = 'PAYLOAD_LOST'" in src, (
        'the payload-loss latch is gone; a dropped tray would be recorded and ignored again')
    assert 'tray_pos[2] < rest_z - FALL_DEPTH' not in src, (
        'the old stride-based detector is back. It sat below the `SAMPLE_EVERY` skip, so how '
        'long a lost payload went unnoticed was set by the LOG RATE rather than the physics')
    assert 'PAYLOAD_LOST' in src.split("'faulted':")[1][:400], (
        'PAYLOAD_LOST is not in the faulted vocabulary, so a dropped tray would report '
        '`faulted = False`')

    i_latch = src.index("state = 'PAYLOAD_LOST'")
    i_machine = src.index("if state == 'CHECK_ALIGN':")
    assert i_latch < i_machine, (
        'the latch is evaluated AFTER the state machine starts; a state whose premise has been '
        'destroyed must not get a turn')
    assert 'break' in src[i_latch:i_latch + 400], (
        'the latch does not end the run -- it would fall through and keep commanding actuators')

    assert 'PAYLOAD_LOST' in set(probe_module()._stages_observed()), (
        'the latch state is not visible as a terminal state of the machine')


def test_the_payload_latch_threshold_uses_the_trays_own_rest_height():
    """The fall threshold must be measured against the resting height of the tray itself.

    A hard-coded world z would be wrong the moment the tray starts anywhere else, and it would
    be wrong QUIETLY: a permissive value means no latch at all, and a strict one kills a
    healthy run on its first tick.
    """
    src = PROBE_SRC
    assert 'rest_z - FALL_DEPTH' in src, (
        'the fall test no longer compares against `rest_z - FALL_DEPTH`')
    assert 'rest_z = meta[' in src, (
        '`rest_z` is no longer read from the model metadata, so the threshold is no longer the '
        'resting height of the tray')
    assert 'FALL_DEPTH = ' in src, 'FALL_DEPTH is no longer a named threshold'


def test_the_two_half_lengths_are_not_interchangeable():
    """`roller_half_length` is LATERAL; `travel_axis_half_length` is ALONG TRAVEL.

    The two differ by 0.1850 m, and because `aboard` is a `>=` test, substituting the lateral
    one is a SILENT TIGHTENING rather than a crash. Measured (`p1-c-g1-*`): the run settled at
    rel_x 0.2367 against a threshold of 0.2400 and reported STALLED while sitting correctly on
    the deck -- 3.3 mm short of a bar that was 0.1850 m too high.

    The roller row is sized against the tray floor's WIDTH, so the lateral number is correct
    for what it was written for; the error is only ever in USING it for a travel-axis decision.
    """
    tray = rig.tray_measurements(rig.TRAY_ASSET_REF)
    lateral = rig.roller_half_length(tray)
    travel = rig.travel_axis_half_length(tray)

    assert lateral == tray['support_half_width'] + rig.ROLLER_OVERHANG, (
        'roller_half_length must stay the row half-length derived from the floor width')
    assert abs(travel - (tray['x_max'] - tray['x_min']) / 2.0) < 1e-12, (
        'travel_axis_half_length must be the tray half-extent along the travel direction')
    assert travel < lateral, (
        'the tray is shorter along the travel direction than the roller row is wide; if this '
        'inverts, the tray has been re-oriented and every travel-axis constant needs rederiving')

    # The error and the two thresholds it produced, as measured.
    assert abs((lateral - travel) - 0.185) < 1e-9, 'the lateral/travel gap has changed'
    assert abs((rig.DECK_ROLLER_X0 + lateral) - 0.240) < 1e-9, (
        'the WRONG threshold is no longer 0.240 -- re-measure before quoting the numbers')
    assert abs((rig.DECK_ROLLER_X0 + travel) - 0.055) < 1e-9, (
        'the RIGHT threshold is no longer 0.055 -- re-measure before quoting the numbers')


def test_the_probe_measures_tray_length_along_the_travel_axis():
    """Both travel-axis decisions in the probe must use the travel-axis length.

    Two call sites, and they are the only two: `aboard` in EMBARK and the trailing edge in
    UNLOAD. A text-level guard rather than a behavioural one because the failure mode is a
    SILENT tightening -- every run simply stops one stage early and reports STALLED, which
    looks exactly like a rig that cannot reach far enough.
    """
    assert 'roller_half_length(meta[' not in PROBE_SRC, (
        'the probe is sizing a TRAVEL-axis decision with the LATERAL half-length of the roller '
        'row again. Because `aboard` is a >= test, this does not crash: it moves the bar '
        '0.1850 m too high and every run stalls one stage early')
    n = PROBE_SRC.count('travel_axis_half_length(meta[')
    assert n == 2, (
        f'expected exactly 2 travel-axis call sites in the probe (EMBARK `aboard` and UNLOAD '
        f'trailing edge), found {n}')


def test_the_contact_trace_records_names_not_counts():
    """A support question needs NAMES.

    `contacts` counts the tray's own 11 geoms against anything, so it reads 8 while the tray is
    0.46 m in the air -- useless for "is it still supported". During the handoff investigation
    the only question that mattered was WHICH crowns were carrying the tray, which needs names.
    This is a text-level guard: it must not silently regress to numeric ids, which would make
    the trace unreadable at exactly the moment it is needed.
    """
    assert "'tray_contacts': tray_contacts" in PROBE_SRC, (
        'the tray contact trace is no longer written into the records')
    assert 'geom_names = [model.geom(i).name' in PROBE_SRC, (
        'the probe no longer builds a geom-name lookup, so a contact trace could only report ids')
    assert ('geom_names[contact.geom2]' in PROBE_SRC
            and 'geom_names[contact.geom1]' in PROBE_SRC), (
        'the contact trace is not resolving geom ids to names')


def test_the_simulated_model_contains_the_receiving_section():
    """The artifact and the simulation must describe the SAME machine.

    `run_scenario` built its model WITHOUT `receiver=True`, while `main()` wrote `model.xml` WITH
    it. So the artifact shown to every reviewer had a receiving section that was not in the model
    the probe actually stepped -- and the consequence was invisible rather than loud: the tray
    simply ran off the end of the deck, `recv_roller_*` registered ZERO contacts in every arm even
    with the tray sitting over where the receiver should have been, and the failure read as a
    hand-off geometry problem. It is a one-keyword bug that invalidated the whole receiving side.

    Two assertions, because either alone is insufficient: the flag must actually change the model,
    and the probe must actually pass it.
    """
    with_receiver = rig.build_model(TRAY, receiver=True)[0].count('name="recv_roller_')
    without = rig.build_model(TRAY, receiver=False)[0].count('name="recv_roller_')
    assert with_receiver > 0, 'receiver=True no longer produces a receiving section at all'
    assert without == 0, (
        'receiver=False now produces receiver rollers as well, so this test can no longer tell '
        'the two models apart and the guard is meaningless')

    i = PROBE_SRC.index('def run_scenario(')
    j = PROBE_SRC.index('mujoco.MjModel.from_xml_string', i)
    call = PROBE_SRC[i:j]
    assert 'build_model(' in call, 'run_scenario no longer builds its own model'
    assert 'receiver=True' in call, (
        'run_scenario is building the model WITHOUT the receiving section again. The probe would '
        'simulate a rig with no receiver while the report still advertises one, and the tray '
        'would run off the end of the deck while every contact trace said "no recv_roller"')

def test_platform_clear_drives_the_tray_clear_instead_of_waiting():
    """`PLATFORM_CLEAR` must CONVEY, not wait.

    `RECEIVED` deliberately stops every roller (a dwell, so "received" means at rest rather than
    merely past a line). `PLATFORM_CLEAR` then inherited that stop and did nothing but poll
    `rel_x` against `DECK_LENGTH + PLATFORM_CLEAR_MARGIN` until its 5 s timeout fired.

    Measured (`p1-c-g1-*` arm G): UNLOAD ends the moment the trailing edge clears the last crown,
    which leaves the tray's centre at rel_x 0.6321 -- and the bar is 0.84. So the state sat waiting
    for a tray with no drive to cover 0.215 m, and reported PLATFORM_NOT_CLEAR. The tray was
    never unable to finish; it was never asked to.
    """
    i = PROBE_SRC.index("elif state == 'PLATFORM_CLEAR':")
    j = PROBE_SRC.index("elif state in ('REFUSED'", i)
    branch = PROBE_SRC[i:j]
    assert 'recv_roller_ids] = UNLOAD_ROLLER_SPEED' in branch, (
        'PLATFORM_CLEAR no longer drives the receiving rollers, so it is back to waiting for an '
        'undriven tray to leave the deck footprint on its own')
    assert 'deck_roller_ids] = UNLOAD_ROLLER_SPEED' in branch, (
        'PLATFORM_CLEAR no longer drives the deck rollers; a tray still touching the last crown '
        'would have nothing pushing it off')
    assert 'DECK_LENGTH + PLATFORM_CLEAR_MARGIN' in branch, (
        'PLATFORM_CLEAR no longer measures the clearance it is supposed to achieve')


def test_the_pusher_is_retired_and_the_reason_is_asserted():
    """The blade cannot both miss the tray and reach the wall. Assert the impossibility.

    The free z-window is zero: the crown plane is the tray underside AND the top of the roller
    band, while the blade is 0.0300 thick. So `REQUIRED_PUSH_TRAVEL` -- the rise needed to bring
    the blade from its stow to the wall's engagement height -- comes out NEGATIVE.

    Kept as a test rather than a comment because it is a TRIPWIRE in both directions: while the
    number stays <= 0 the pusher is correctly retired and no stroke can make it work, and if a
    future geometry makes it positive again then a pusher has become possible and this fails to
    say so, rather than leaving the dead mechanism in place unchallenged.
    """
    assert rig.REQUIRED_PUSH_TRAVEL <= 0.0, (
        f'REQUIRED_PUSH_TRAVEL is {rig.REQUIRED_PUSH_TRAVEL:.4f} > 0: a pusher is geometrically '
        f'possible again. Rebuild the pusher mechanism and re-derive PUSHER_TRAVEL rather than '
        f'leaving a nominal stroke in place')
    assert rig.PUSHER_TRAVEL > 0.0, (
        'PUSHER_TRAVEL must stay positive or the joint range is degenerate and the model will not '
        'load; it is a nominal stroke, not a working one')
    assert rig.PUSHER_STOW_Z >= rig.TRAY_TOP_DECK_LOCAL, (
        f'the blade stows at {rig.PUSHER_STOW_Z:.4f}, below the tray top '
        f'{rig.TRAY_TOP_DECK_LOCAL:.4f}: it is inside the tray again')
    # and the tray top must be the TRAY's, not a placed number
    tray = rig.tray_measurements(rig.TRAY_ASSET_REF)
    assert abs(rig.TRAY_TOP_DECK_LOCAL - (tray['z_max'] - tray['z_min'])) < 1e-12, (
        'TRAY_TOP_DECK_LOCAL is no longer the tray asset top minus its underside')


def test_both_contact_traces_name_the_far_side_through_one_helper():
    """The contact direction must be written exactly once.

    Three consecutive instrument failures, all the same family: the first `tray_contacts` took the
    tray's own geom, the first `deck_contacts` missed the deck's child bodies, the second
    `deck_contacts` had a self-contradictory predicate, and the third took the deck's own geom
    again. Every one returned something plausible -- `['tray_floor']`, or an empty set, or the
    deck's own roller names -- and none crashed.

    A "which side" error answers a DIFFERENT QUESTION rather than failing, and an empty result
    reads like a finding. So the direction is now one helper that both traces call; this asserts
    neither has been inlined again.
    """
    assert 'def other_side_of(' in PROBE_SRC, (
        'the shared contact-direction helper is gone; an inlined copy will eventually be inverted')
    assert PROBE_SRC.count('other_side_of(c,') == 2, (
        f'expected both contact traces to call the helper, found '
        f'{PROBE_SRC.count("other_side_of(c,")} call site(s)')
    assert 'geom_names[contact.geom2] if contact.geom1 in own_geom_ids' in PROBE_SRC, (
        'the helper no longer returns the geom on the FAR side of the contact')
    # and both predicates must be the symmetric exclusion of our own side
    assert PROBE_SRC.count('!= (c.geom2 in tray_geoms)') == 1
    assert PROBE_SRC.count('!= (c.geom2 in deck_geom_ids)') == 1


def test_the_deck_travel_target_is_referenced_to_the_world():
    """A joint-referenced travel breaks the moment the rig's origin moves with the dock.

    The rig is built with the deck at `DECK_X0 + gap` and the two crown rows are exactly tangent
    at full travel (clearance 0.0000), so a joint-referenced target spends the dock gap on
    interpenetration with `recv_roller_0`. Measured: 42 of 58 admitted runs timed out in EMBARK
    with the slide short by exactly the gap.

    This is a text-level guard because the failure is silent at the level of the rig geometry --
    every constant stays correct while the WORLD end position drifts.
    """
    assert 'deck_travel_here = DECK_TRAVEL - float(scenario[' in PROBE_SRC, (
        'the deck travel is no longer corrected for the dock gap, so its WORLD end position will '
        'drift by the gap and the last crown will drive into recv_roller_0')
    assert '>= deck_travel_here - 1e-3' in PROBE_SRC, (
        'EMBARK still demands the uncorrected travel, which a docked part cannot reach')
    assert 'min(deck_travel_here, DECK_SPEED * elapsed)' in PROBE_SRC, (
        'the deck trajectory still ramps to the uncorrected target')
    assert 'data.ctrl[deck_id] = DECK_TRAVEL' not in PROBE_SRC, (
        'a deck command still uses the raw constant; every command site must use the corrected '
        'travel or the deck will push into the receiver in whichever state was missed')
    # The hand-off must have REAL mechanical clearance. This assertion used to require the two
    # rows to be exactly tangent -- a zero-clearance design whose end position is a contact --
    # with the comment "if this ever gains margin, revisit the above". That is exactly what
    # happened, and the deck-travel correction above was re-checked and KEPT: the largest swept
    # dock gap (0.028 m) still exceeds CROWN_CLEARANCE, so the correction is still what keeps
    # the deck out of the receiver. The clearance is margin stacked on top of it, not a
    # replacement for it.
    last = (rig.DECK_X0 + 0.240 + rig.DECK_ROLLER_X0
            + rig.ROLLER_PITCH * (rig.DECK_ROLLERS - 1))
    gap = rig.RECV_FIRST_CROWN_X - last - 2 * rig.ROLLER_RADIUS
    assert gap == pytest.approx(rig.CROWN_CLEARANCE), (
        f'the hand-off clearance is {gap:+.4f}, not CROWN_CLEARANCE '
        f'{rig.CROWN_CLEARANCE:+.4f}')
    assert gap > 0, (
        'the two crown rows must not be tangent: zero clearance means the designed end '
        'position is a contact')
    assert rig.CROWN_CLEARANCE < 0.028, (
        'the clearance has grown past the largest swept dock gap, so the deck-travel '
        'correction is no longer what keeps the deck out of the receiver -- reconsider it '
        'rather than leaving it in place')


def test_the_deck_travel_corrects_for_both_the_gap_and_the_yaw():
    """The deck's end position depends on the dock gap AND on the yaw.

    `DECK_TRAVEL - gap` took DONE from 16 to 34 and left 24 failures, every one a `yaw_*` case.
    The deck's last crown is a LINE of half-width `roller_half_length` at deck-local
    `DECK_ROLLER_X0 + pitch*(n-1)`: yawing the deck swings it, so the row's END reaches further
    forward by `row_half * |sin yaw|` (0.0131 m at 3 deg) while its centre pulls back by
    `crown_x * (1 - cos yaw)`. The skew is what touches `recv_roller_0` first.

    Both terms must be subtracted from the commanded travel, and both are derived from the rig.
    """
    assert "'yaw']" in PROBE_SRC and 'deck_travel_here = DECK_TRAVEL - float(scenario[' in PROBE_SRC
    assert '_forward_reach' in PROBE_SRC, (
        'the yaw term is gone; every yawed dock will drive its crown row into recv_roller_0')
    assert 'meta[' in PROBE_SRC and "'roller_half_length']" in PROBE_SRC, (
        'the row half-width is no longer read from the rig metadata, so the skew term is not '
        'derived from the machine under test')
    # the two terms must be the geometry the rig actually has
    import math
    crown_x = rig.DECK_ROLLER_X0 + rig.ROLLER_PITCH * (rig.DECK_ROLLERS - 1)
    row_half = rig.roller_half_length(rig.tray_measurements(TRAY))
    yaw = math.radians(3.0)
    reach = crown_x * math.cos(yaw) + row_half * abs(math.sin(yaw))
    assert abs((reach - crown_x) - (row_half * math.sin(yaw) - crown_x * (1 - math.cos(yaw)))) < 1e-12
    assert 0.012 < (reach - crown_x) < 0.014, (
        f'the 3 deg skew is {reach - crown_x:.4f} m; re-derive if the row geometry changed')


def test_deploy_pusher_verifies_the_carriage_is_stowed():
    """The retired mechanism's stage must not be a no-op.

    `DEPLOY_PUSHER` commands nothing: the pusher is retired (D034 -- the blade has nowhere to
    stow that both clears the tray's path and reaches the wall). But a carriage that drifted
    forward would put the blade back inside the tray's swept volume and push the tray, which is
    the failure D034 measured at 0.8232 m of travel lost. This stage is the only place that can
    notice, so it must actually check rather than merely hold for its nominal span.

    Asserted by SHAPE and by the fault being latched, not by pinning a wording -- pinning
    wordings is exactly what let the whole-C scope sentence go stale.
    """
    assert 'PUSHER_STOW_TOL' in PROBE_SRC, (
        'the stow tolerance is gone, so the stage cannot tell a stowed carriage from a drifted one')
    assert 'abs(float(data.qpos[pusher_slide_adr])) > PUSHER_STOW_TOL' in PROBE_SRC, (
        'DEPLOY_PUSHER no longer compares the carriage position against the tolerance')
    assert "state = 'STOW_DRIFT'" in PROBE_SRC, (
        'a drifted carriage must latch a fault, not merely be noted')
    # a fault that is not enumerated is not latched: the hold branch must include it, or the
    # deck could re-open under a tray after the carriage has already been found drifting
    assert PROBE_SRC.count("'STOW_DRIFT'") >= 2, (
        'STOW_DRIFT must appear both in the fault vocabulary and in the latched-fault hold branch')
