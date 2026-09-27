"""Single source of truth for project 010's conveyor rig geometry (experiment C).

Every load-bearing dimension is derived from the V1 tray (`assets/objects/tray_v1.xml`) or
from a clearance rule, never chosen by eye:

  roller pitch 0.080 m, radius 0.035 m   -> 10 mm clearance between neighbouring crowns
  roller length = 2 x (tray support half-width + 0.020 m overhang)
  guide band    = the only strip clear of the tray wall, the handle sphere and the handle stem

Frame conventions
  world rows      : roller bodies at z = ROLLER_Z, crown plane at CROWN_Z
  receiving deck  : body origin sits ON the crown plane, so everything inside it is
                    expressed relative to the deck surface (rollers at z = -ROLLER_RADIUS)

Friction is a SWEPT PARAMETER, applied to both sides of every interface so the effective
coefficient is unambiguous. The tray asset declares 1.0 with no material basis anywhere in the
project; `PRECONDITIONS.md` forbids leaning on a strong friction number to mask a bad
mechanism, so the rig overrides it and records what it overrode.
"""
import math
import xml.etree.ElementTree as ET
from pathlib import Path

ROLLER_RADIUS = 0.035
ROLLER_PITCH = 0.080
ROLLER_Z = 0.450
CROWN_Z = ROLLER_Z + ROLLER_RADIUS                      # 0.485
ROLLERS_PER_SECTION = 8
FIXED_SECTIONS = 3
DECK_ROLLERS = 8
ROLLER_OVERHANG = 0.020

# A pitch-wide strip between the rows cannot be closed by simply adding fixed sections: the strip
# moves with the handoff, so FIXED_SECTIONS = 3 relocated it and left it 0.120 m wide. A full
# pitch of clearance was only needed because the retention pin lived in it. So the pin is
# relocated to the deck's OUTER end -- which is where it does real work anyway, since a pin at
# the inner edge only stops the tray rolling back while the deck drives out from under it --
# and the handoff is then closed by the roller-radius rule below.
LAST_FIXED_CROWN_X = ROLLER_PITCH * (ROLLERS_PER_SECTION * FIXED_SECTIONS - 1)
DECK_X0 = LAST_FIXED_CROWN_X + ROLLER_PITCH                          # 1.36
# The deck's local x origin is DECK_X0, and its first roller is placed so that its crown sits
# exactly TWO RADII from the last fixed crown. Any more and there is a HOLE between the rows
# rather than a nip -- two r = 0.035 rollers only touch at 0.070 m, while a full pitch is
# 0.080 m. So the handoff spacing is derived from the roller radius and the pitch, not chosen:
DECK_FIRST_CROWN_X = LAST_FIXED_CROWN_X + 2.0 * ROLLER_RADIUS
DECK_ROLLER_X0 = DECK_FIRST_CROWN_X - DECK_X0                         # -0.010
DECK_LENGTH = ROLLER_PITCH * (DECK_ROLLERS + 1) + 2.0 * ROLLER_RADIUS  # deck body span
# ^ one extra pitch past the last roller, which is where the retention pin lives.
# DECK_LEAD_GAP is the old name for how far the first crown sits past the deck origin. It is now
# NEGATIVE (the first roller is tucked slightly behind the origin), which the geometry tests
# assert rather than assume, because a negative value is what makes the two rows meet cleanly.
DECK_LEAD_GAP = DECK_ROLLER_X0

DECK_FRAME_HALF = (0.300, 0.300, 0.010)

# --- the rear PUSHER (D031-A) --------------------------------------------------------
#
# WHAT THIS REPLACED. A `retention_pin` used to sit IN the roller plane at deck-local
# x = 0.6700, one and a half pitches past the last deck roller. Measured: the seated tray
# spans -0.0100..+0.4900, so the pin was 0.1706 m beyond its leading edge and
# `pin_contacts` was 0 in EVERY recorded run. The tray was never pushed by it; it was
# stopped by running INTO it after its leading edge had already left the last crown --
# which is why the fault looked like a late overrun rather than a mechanism doing nothing.
#
# Moving the pin into the row was proved impossible by measurement. The pin is 0.0240 m
# wide and the clear slot between two roller surfaces is `pitch - 2r` = 0.0100 m, so no
# position in the roller plane both misses every roller and can be reached by a fully
# supported tray. The two ROWS also meet with zero clearance: the deck first crown sits
# exactly 2r past the last fixed crown, so an inner-end placement hits the FIXED row.
# Full sweep in DECISIONS.md D031.
#
# So the mechanism leaves the roller plane. ABOVE the crown plane there are no rollers at
# all, which makes x geometrically free, and what it can reach there is the tray REAR
# WALL -- the face it should have been pushing from the start.
#
# SCOPE: this is a PUSHER, not the transport stopper. It presents a normal-contact face
# during EMBARK and has no job afterwards. Retention against an AMR acceleration is a
# different mechanism with different requirements (two-sided, releasable) and nothing here
# claims it -- P1-C-05 is a bench transfer with no AMR in the loop.
#
# HEIGHT and TRAVEL are derived after `tray_measurements`, because deriving them needs the
# asset. Only the x placement lives here, expressed against the deck first crown so that a
# change to the handoff rule carries the pusher with it instead of stranding it.
# X PLACEMENT: a CARRIAGE, not a fixed post.
#
# A rigidly mounted pusher was measured to be INFEASIBLE, and the proof is worth keeping
# because it is the reason this mechanism has two joints. With the pusher bolted to the deck
# it has exactly one x, and the EMBARK asks two opposite things of it:
#
#     REACH        pusher_front >= w0 - T   (it must meet the wall before the deck runs out)
#     NEVER AHEAD  pusher_front <= w0 - T   (it must not strike a stalled tray from the front)
#
# The only solution is the equality, which is zero-margin AND lies off the rear of the deck,
# because w0 is only ~0.135 m behind the deck origin while the deck travels 0.240 m. So the
# mechanism gets its own x degree of freedom: it retracts, deploys to meet the wall, and then
# follows it. That decouples the two constraints completely -- see docs/DECISIONS.md D032.
#
# RETRACTED POSITION, anchored to the SEAT BEAMS. Where the wall ends up is decided by where
# the tray STOPS, and the seat sensor pair is what decides that: the pair only breaks together
# while the tray centre is inside BEAM_SEAT_A_X..BEAM_SEAT_B_X, so the wall rear face is only
# known to [BEAM_SEAT_A_X - half, BEAM_SEAT_B_X - half]. The retracted blade must be behind
# the FURTHEST-BACK wall of that interval, so the dependency is on the beams and the tray, not
# on the deck: change the beam spacing and the carriage follows.
# The blade's z half-height is set by the PRESS-LOW requirement, not by stiffness. The
# tray is 0.098 kg over 0.500 m, and a push above the wall's mid-height tips its leading lip
# into the 0.010 m hole between crowns. The wall band is deck-local z [+0.0050, +0.0650], so
# the midpoint is +0.0350; with the blade's bottom at the crown clearance (+0.0010) the tallest
# blade that stays below the midpoint is 0.0680 m, i.e. half_z <= 0.0340.
#
# 0.0150 gives a 0.0300 m blade whose top is +0.0310 -- 0.0040 m under the midpoint, pressing
# the wall's lower 43%. The earlier 0.0300 put the top at +0.0610, ABOVE the midpoint, and a
# test caught it. That test was right and the blade was wrong.
#
# Note the blade's x half-width is still ROLLER_PITCH/8; only z changed, and the measured
# roller clearance is a function of the BOTTOM face, which did not move -- a thinner blade is
# further from the rollers, so the CLEAR result is unaffected.
PUSHER_HALF = (ROLLER_PITCH / 8.0, 0.120, 0.0150)
PUSHER_RETRACT_MARGIN = 0.020                 # clearance behind the worst-case wall
# The carriage's own body. It runs the length of the deck, so it is long and thin and lives
# under the roller band like the blade. Its length is set by the stroke it has to carry.
#
# It also must not TOUCH the blade. The first version put the plate's top exactly on the
# blade's stow bottom, and the two solid boxes then shared a face: the lift actuator sat
# saturated at its full 30 N and the joint did not move by 1e-5. Two parts of one mechanism
# that slide relative to each other need a REAL gap, and the gap is declared here rather than
# obtained by switching off their collision.
PUSHER_CARRIAGE_HALF_Y = 0.100
PUSHER_CARRIAGE_HALF_Z = 0.010
# THE CARRIAGE CANNOT BE LONG. It carries the blade the whole deck travel, and the deck FRAME
# hangs below the rollers over the deck's forward 0.600 m. At the stroke's FORWARD STOP the
# carriage's rear face lands exactly on DECK_LEAD_GAP -- a zero-clearance tangency, which is
# what a 0.300 m plate centred on the deck origin produced and why it collided. So the rear
# face is DERIVED to sit one declared width behind the deck's first roller, and the length is
# DERIVED from the rear face and the stroke. Neither is chosen.
PUSHER_CARRIAGE_BACK_CLEAR = 2.0 * PUSHER_HALF[0]      # one blade width, declared margin
# PUSHER_CARRIAGE_HALF / PUSHER_CARRIAGE_X / PUSHER_CARRIAGE_TOP_Z are derived further down:
# they consume PUSHER_SLIDE_TRAVEL (itself derived) and frame geometry, so declaring them here
# is a NameError at import. That is the documented ordering rule in this file.
PUSHER_CARRIAGE_GAP = 0.005
# How deep below the crown plane the stowed blade hides, and how far past the wall bottom
# edge it engages. Declared here; consumed by the derived block further down.
#
# THE STOW DEPTH IS NOT A FREE CHOICE ANY MORE. The old rigid pin could sit anywhere in z,
# because its x never changed. This blade SLIDES along x through the deck roller row, so it
# must clear every roller at EVERY slide position: it has to lie entirely below the roller
# lowest point, which is the roller band bottom `-2 * ROLLER_RADIUS`. Sharing the band over
# any x overlap is a plow. That is the stricter rule a degree of freedom buys you.
PUSHER_STOW_DEPTH = 2.0 * ROLLER_RADIUS
PUSHER_ENGAGE_M = 0.010
# THE BLADE PRESSES ABOVE THE CROWN PLANE, and that single choice is what removes the roller
# constraint on x entirely. The crown plane is the TOP of the roller band (deck-local z = 0),
# so everything above it is roller-free at EVERY x -- measured: a blade occupying z
# [0, +0.0600] leaves only a 9 micrometre tangency with `fixed_roller_2_6`, and that tangency
# does NOT move with the slide, i.e. it is z and not x. Any positive clearance removes it.
#
# This replaces the whole failed family of "hide the blade in the band" designs, all three of
# which were measured in the solver and all three of which plough a roller:
#   * blade in the band at x -0.1650  -> `fixed_roller_2_5` -0.0046, `fixed_roller_2_6` -0.0054
#   * blade in the band at x -0.0350  -> `deck_roller_0` -0.0002, `deck_roller_1..3` -0.0051
#   * blade ABOVE the crown plane     -> `fixed_roller_2_6` -0.000009, and only ever that
# The reason is structural: the seat beams put the push point FORWARD of the only free
# corridor (deck-local x <= -0.0450, behind the fixed row), and inside the band no x forward
# of that corridor clears the deck's own crowns -- which the follow phase sweeps unavoidably.
# Above the plane, none of it applies.
PUSHER_CROWN_CLEARANCE = 0.0010               # blade bottom sits this far above the band top
# The blade also must not foul the deck FRAME, which hangs below the rollers and is 0.600 m
# wide in x about the deck origin. The retracted blade is behind it; only the deployed blade
# could reach it, and the deployed blade is above the crown plane. Asserted in the tests.

# Guide band, DERIVED from the tray's own geoms (resting on the crown plane):
#   side wall   |y| 0.222..0.230 at deck-local z 0.005..0.065
#   handle ball |y| 0.265..0.335 at z 0.005..0.075, but only within +-0.035 of the tray centre
#   handle stem |y| 0.211..0.309 at z 0.031..0.049
# The only strip clear of all three is z 0.005..0.029 at |y| 0.237..0.262. It can be that
# thick only because it sits ABOVE the roller crowns.
#
# There is NO flared lead-in, and that is a derived constraint rather than an omission: a mouth
# has to be wider than the wall (0.230) at its entrance, but any fixed-x mouth is swept by the
# handle ball once the tray's centre passes it, and the ball's inner face is at 0.265. So the
# mouth can never exceed 0.265, giving at most 0.265 - 0.230 = 35 mm of capture. An earlier
# revision flared to 0.297 and struck the ball, which is why `guides=True` never completed.
# The rail diameter is what sets the reach. An earlier revision used GUIDE_HALF_Y = 0.0125 at
# GUIDE_Y = 0.2495, i.e. it reached up to 0.2620 against the handle ball's inner face at 0.2650
# -- a 3 mm gap, which is why any residual yaw put a rotating handle ball into it.
GUIDE_HALF_X = 0.190
GUIDE_HALF_Y = 0.0090
GUIDE_HALF_Z = 0.012
GUIDE_X = 0.270
GUIDE_Y = 0.2495
GUIDE_Z = 0.017
GUIDE_FRICTION = 0.10

# Modelled sensors. None of them reads the tray's pose: the two beams measure OCCLUSION and
# the two proximity rays measure distance to structure, which is what the real heads do.
BEAM_Z = 0.010                    # deck-local height; clears the rollers and the guide rails
BEAM_Y0 = -0.200                  # inside the guide rails, so they cannot block it
# Three heads. A Y-directed head at deck-local x_b is occluded while the tray centre is within
# +-tray_half_x of x_b, so a PAIR only breaks together inside the overlap of their windows.
# An earlier revision spaced them 0.255 m apart, which is wider than 2 x tray_half_x = 0.130 m,
# so "fully aboard" was unreachable and the sequence always fell through to over-travel.
BEAM_SEAT_A_X = 0.115             # window [-0.050, 0.180]; pair overlap [0.150, 0.180]
BEAM_SEAT_B_X = 0.215             # window [ 0.150, 0.280]
BEAM_FAR_X = 0.600                # over-travel guard: broken once the leading edge passes it
# Gap rays start BEHIND the deck origin so they sit outside the last fixed roller, and are
# aimed at the roller's curved surface. Their zero-gap reading is therefore a geometric
# constant, which is what a real proximity head is calibrated for. An earlier revision put the
# origin inside the roller and returned nothing.
GAP_RAY_X = -0.038
GAP_RAY_Y = 0.150
GAP_RAY_Z = -0.060
# The height head looks down at the FLOOR past the deck's own rear edge. Aiming it at the deck
# itself is unreliable: the sample point slides across the roller row as the gap changes, so the
# reading jumps by a roller diameter instead of tracking the step.
STEP_RAY_LOCAL = (0.700, 0.0, 0.050)
STEP_RAY_LENGTH = 1.0

DEFAULT_FRICTION = 0.25
FRICTION_SWEEP = (0.15, 0.25, 0.40)
FRICTION_TRIM = (0.005, 0.001)
SUPPORT_TOLERANCE = 0.001
AXIS_ALIGNED_TYPES = ('box', 'sphere', 'capsule')

# --- derived friction floor -----------------------------------------------------------------
# A driven roller can push the tray along x only if its tangential force beats edge friction,
# i.e. mu * m * g > mu * m * g * (R - g/2) / R ... which cancels, so the honest statement is
# narrower and is the one measured here: a SINGLE driven row has roughly half the tray's weight
# on it, so the acceleration available to a seated tray is mu * g / 2. Over a 0.24 m transfer
# that is a time of sqrt(2 * 0.24 / (mu * 9.81)):
#
#     mu = 0.15  ->  0.57 s   fine
#     mu = 0.10  ->  0.70 s   fine
#
# So edge friction is NOT what stopped the mu = 0.15 runs. What stopped them is that a tray with
# friction mu on BOTH the fixed row and the deck row can stall on the HANDOFF: while its centre
# is in the 0.04 m dead strip between the last fixed crown (x = 1.240) and the first deck crown
# (x = 1.320) it is unsupported, its front edge drops, and the deck's leading roller has to lift
# it. That is a geometric defect of the 0.04 m dead strip, not a friction-threshold problem, and
# it is reported as such rather than papered over by raising mu.
MIN_FRICTION_CITED = 0.15

# The tray must actually be CARRIED by the deck, not left standing on the fixed row while the
# deck drives away. The retention pin is the only thing pushing it, and the tray rides on
# rollers, so this is the single number that separates "transferred" from "abandoned".
EMBARK_FOLLOW_MIN = 0.150


def friction_string(mu):
    return f'{mu:.4f} {FRICTION_TRIM[0]} {FRICTION_TRIM[1]}'


def gap_reading_at_zero(step=0.0, yaw=0.0):
    """Distance a deck-mounted -X gap ray reads when the docking gap is zero.

    The ray meets the last fixed roller's cylindrical surface, so the reading is
    `origin_x - (last_roller_x + sqrt(R^2 - dz^2))` with dz the ray's height below the roller
    axis. This is the calibration constant of a real proximity head -- and it is a FUNCTION of
    the deck's height and attitude, which is the part that is easy to miss:

      * raising the deck by `step` raises the ray by the same amount, so dz falls and the chord
        grows by d(sqrt(R^2 - dz^2))/d(dz) * -step. At the nominal dz = 0.025 m a 4 mm step
        changes the chord by +3.50 mm and an 8 mm step by +6.10 mm.
      * yawing the deck about Z leaves the ray's height alone, but the ray is now oblique to the
        roller's axis, so its `y` offset (0.150 m) becomes an effective `x` offset on the
        cylindrical surface; the chord it cuts grows with theta.

    Both terms were being charged to the gap head in an earlier revision, which is why a
    perfectly docked tray read a 6.1 mm "gap error" at an 8 mm step and a 35 mm one at 8 deg of
    yaw. They are calibration, not measurement error, so they are computed here and subtracted
    by the observer -- exactly as a real head's calibration table would.
    """
    dz = ROLLER_Z - (CROWN_Z + step + GAP_RAY_Z)
    if abs(dz) >= ROLLER_RADIUS:
        raise SystemExit('the gap ray misses the roller it is meant to see')
    half_chord = math.sqrt(ROLLER_RADIUS ** 2 - dz ** 2)
    # Obliquity: the ray is along deck-local -X, so a deck yaw theta turns it into a chord whose
    # effective axial offset is GAP_RAY_Y * tan(theta). The half-chord is shortened by that
    # amount times the local surface slope, which to first order is (GAP_RAY_Y * tan(theta))^2 /
    # (2 * R) -- the same second-order curvature term measured in the sweep.
    obliquity = (GAP_RAY_Y * math.tan(yaw)) ** 2 / (2.0 * ROLLER_RADIUS)
    return (DECK_X0 + GAP_RAY_X) - (LAST_FIXED_CROWN_X + half_chord + obliquity)


def step_reading_at_zero():
    """Distance a deck-mounted -Z height ray reads when the deck sits on the crown plane.

    It looks down at the floor (z = 0) past the deck's own rear edge, so it returns
    `mount_z + CROWN_Z`. Raising the deck by `step` raises the reading by exactly `step`.
    """
    return STEP_RAY_LOCAL[2] + CROWN_Z


def load_tray_body(path):
    text = Path(path).read_text(encoding='utf-8')
    return ET.fromstring(text[text.index('<body'):])


def geom_extents(geom):
    kind = geom.get('type', 'sphere')
    if kind not in AXIS_ALIGNED_TYPES:
        raise SystemExit(f'tray geom {geom.get("name")!r}: type {kind!r} unsupported by this '
                         f'measurement (supported: {AXIS_ALIGNED_TYPES})')
    pos = [float(v) for v in (geom.get('pos') or '0 0 0').split()]
    size = [float(v) for v in (geom.get('size') or '0').split()]
    mass = float(geom.get('mass') or 0.0)
    if kind == 'box':
        half = (size[0], size[1], size[2])
    elif kind == 'sphere':
        half = (size[0], size[0], size[0])
    else:
        a = [float(v) for v in geom.get('fromto').split()]
        r = size[0]
        pos = [(a[0] + a[3]) / 2, (a[1] + a[4]) / 2, (a[2] + a[5]) / 2]
        half = (abs(a[3] - a[0]) / 2 + r, abs(a[4] - a[1]) / 2 + r, abs(a[5] - a[2]) / 2 + r)
    return {'name': geom.get('name'), 'type': kind,
            'x_min': pos[0] - half[0], 'x_max': pos[0] + half[0],
            'y_min': pos[1] - half[1], 'y_max': pos[1] + half[1],
            'z_min': pos[2] - half[2], 'z_max': pos[2] + half[2], 'mass': mass}


def colliding_geoms(body):
    return [g for g in body.findall('geom') if g.get('contype', '1') != '0']


def tray_measurements(path):
    parts = [geom_extents(g) for g in colliding_geoms(load_tray_body(path))]
    if not parts:
        raise SystemExit(f'{path}: no colliding geoms')
    z_lowest = min(p['z_min'] for p in parts)
    support = [p for p in parts if p['z_min'] <= z_lowest + SUPPORT_TOLERANCE]
    return {
        'n_colliding_geoms': len(parts),
        'mass_kg': round(sum(p['mass'] for p in parts), 6),
        'x_min': min(p['x_min'] for p in parts), 'x_max': max(p['x_max'] for p in parts),
        'y_min': min(p['y_min'] for p in parts), 'y_max': max(p['y_max'] for p in parts),
        'z_min': z_lowest, 'z_max': max(p['z_max'] for p in parts),
        'support_half_width': max(max(abs(p['y_min']), abs(p['y_max'])) for p in support),
        'support_names': sorted(p['name'] for p in support),
        'parts': parts,
    }


def roller_half_length(measurements):
    """The half-length of the ROLLER ROW -- a LATERAL (y) dimension.

    It is `support_half_width` (the tray floor's y half-width) plus the roller overhang: how
    far the rollers reach SIDEWAYS so they can carry the floor. The roller is a cylinder with
    `quat = .707 .707 0 0`, i.e. its 0.500 m axis lies along y, which is why this number is a
    y-dimension and why it is 0.2500 while the tray is only 0.0650 long along the travel axis.

    Using it where a travel-axis length belongs inflates the value by 0.1850 m, and in a `>=`
    test that is a SILENT TIGHTENING, not a crash: a measured run settled at rel_x 0.2367
    against an `aboard` threshold of 0.2400 and was reported STALLED while sitting correctly
    on the deck. Use `travel_axis_half_length` for anything measured along the travel axis.
    """
    return measurements['support_half_width'] + ROLLER_OVERHANG


def travel_axis_half_length(measurements):
    """The tray's half-extent ALONG THE TRAVEL DIRECTION (deck-local x).

    The tray is placed with no rotation -- `model.xml` gives the `payload` body no `quat` -- so
    the asset x axis IS the travel axis and this is simply the asset's x half-extent.

    Named separately from `roller_half_length` so the two can never be swapped by accident:

        roller_half_length()      = 0.2500   LATERAL, sizes the roller row against floor width
        travel_axis_half_length() = 0.0650   ALONG TRAVEL, decides what "aboard" means
    """
    return (measurements['x_max'] - measurements['x_min']) / 2.0


def handle_clearance(measurements):
    return (CROWN_Z - measurements['z_min']) - 0.035 - CROWN_Z

# --- the receiving section ------------------------------------------------------------------
# Where the tray ends up after the deck has carried it out and it is pushed off the far end.
#
# ORDER MATTERS IN THIS FILE. Every constant below is derived by CALLING the measurement
# helpers, so this block has to sit AFTER `roller_half_length` is defined. It was first written
# up in the constants section at the top of the module and the module stopped importing -- a
# NameError, not a wrong number. That is the failure mode that is easiest to get right by
# accident and hardest to notice: the file still parses, and the traceback points at a line
# that looks correct in isolation.
#
# The asset this rig is built around. Named here so the receiving section's length can be
# derived from it; `build_model` still takes the path as an argument.
TRAY_ASSET_REF = Path(__file__).resolve().parents[1] / 'assets' / 'objects' / 'tray_v1.xml'

# Mirrors probe_deck.DECK_TRAVEL; a test asserts the two agree rather than trusting this copy.
DECK_TRAVEL_REF = 0.240

# The tray's own length, MEASURED from the real asset so the receiving section cannot be sized
# against a stale number. Taken once at import time.
TRAY_LENGTH_REF = 2.0 * roller_half_length(tray_measurements(TRAY_ASSET_REF))

# The deck's LAST crown, in world coordinates, when the deck is fully out:
#   DECK_X0 + DECK_TRAVEL + (DECK_ROLLER_X0 + pitch * (n - 1))
DECK_LAST_CROWN_X = (DECK_X0 + DECK_TRAVEL_REF + DECK_ROLLER_X0
                     + ROLLER_PITCH * (DECK_ROLLERS - 1))

# The receiving section's first crown sits exactly 2 * ROLLER_RADIUS beyond it, i.e. the two
# crowns TOUCH. This is the same rule the fixed->deck handoff already uses (see
# DECK_FIRST_CROWN_X), and it is deliberately NOT the pitch rule: a pitch of 0.080 m between
# crowns 0.070 m wide is a 0.010 m hole, which is fine inside a row where the tray is supported
# on both sides and fatal at a handoff, where the leading edge drops into it. The first
# placement used one pitch and the tray fell at 16.8 s.
# --- the deck-to-receiver hand-off, and its MECHANICAL clearance -------------------------
# This used to be `DECK_LAST_CROWN_X + 2r`, which places the two crown rows exactly tangent:
# zero clearance, i.e. the design's end position IS a contact. Keeping the deck out of the
# receiver was therefore entirely a control problem -- the corrected `deck_travel_here` does it,
# and it is measured -- but the mechanical design contributed nothing, so any control error
# becomes a collision. Real clearance is the mechanical half of the same fix.
#
# 0.010 m is chosen against what the mechanism can bear, not for roundness:
#   * the tray floor is 0.130 m long along the travel axis, so it still spans the widened
#     hand-off (surface gap 0.010 to 0.023 m) with room to spare;
#   * it stays SMALLER than the largest swept dock gap (0.028 m), so the control correction is
#     still required -- this adds margin, it does not replace the correction;
#   * at the corrected deck end position the surface gap is CROWN_CLEARANCE plus the yaw skew,
#     so it is never below 10 mm.
CROWN_CLEARANCE = 0.010
RECV_FIRST_CROWN_X = DECK_LAST_CROWN_X + 2.0 * ROLLER_RADIUS + CROWN_CLEARANCE

# How long the receiving section has to be. Derived from the object it must hold rather than
# picked: the tray plus a margin at each end, rounded UP to a whole number of pitches so the row
# keeps the same spacing as every other row in the rig.
#
# The first attempt used 8 rollers (0.56 m of span) for a 0.50 m tray and the tray fell: with
# its centre still near the deck, the tray's trailing quarter-metre had nothing under it once
# the deck's surface ended at 2.710 and the receiver did not start until 2.780.
RECV_MARGIN_M = 0.250
RECV_ROLLERS = math.ceil((TRAY_LENGTH_REF + 2.0 * RECV_MARGIN_M) / ROLLER_PITCH) + 1

# _roller_row places crowns at x0 + pitch * i, so this is the first crown, and the row lives in
# the world (not on the deck), so no deck-origin offset applies.
RECV_ROLLER_X0 = RECV_FIRST_CROWN_X


# --- the pusher height, travel and wall band, DERIVED from the tray asset (D031-A) -----
#
# This block sits here rather than in the constants section because every number in it is
# obtained by CALLING `tray_measurements`. Putting it up top was tried first and the module
# stopped importing: a NameError, not a wrong number, which is the failure mode that leaves
# a file which still parses and a traceback pointing at a line that looks correct alone.
_PUSHER_TRAY = tray_measurements(TRAY_ASSET_REF)
_PUSHER_TRAY_HALF = roller_half_length(_PUSHER_TRAY)


def pusher_wall_band(measurements=None):
    """Deck-local z band and y half-width of the tray x-end walls, i.e. the pressable face.

    The tray floor rests on the crown plane, so an asset-relative z becomes deck-local by
    adding `-z_min`. Both x-end walls are included: the pusher cannot know which end faces
    it, so the band is their union, and the tray own geometry decides which one is touched.
    """
    m = measurements if measurements is not None else tray_measurements(TRAY_ASSET_REF)
    offset = -m["z_min"]
    walls = [p for p in m["parts"] if (p["name"] or "").startswith("tray_wall_x")]
    if not walls:
        raise SystemExit("tray asset has no x-end wall; a rear pusher has nothing to push")
    return {"z_lo": min(p["z_min"] for p in walls) + offset,
            "z_hi": max(p["z_max"] for p in walls) + offset,
            "y_half": max(max(abs(p["y_min"]), abs(p["y_max"])) for p in walls),
            "rear_face_x": min(p["x_min"] for p in walls)}


PUSHER_WALL = pusher_wall_band(_PUSHER_TRAY)
# STOWED: the blade sits in the roller band with its TOP exactly ON the crown plane.
#
# THE ROLLER CLEARANCE COMES FROM X, NOT FROM Z. Two attempts to hide the blade BELOW the
# band both failed, and the second is instructive: the blade collided with `fixed_roller_2_6`
# (measured dist -0.000022). The FIXED row's rollers extend under the WHOLE conveyor at the
# same world z band as the deck's own, so 'below the band' is not free space at all -- it is
# simply the other row's band.
#
# What IS free is an x corridor: the fixed row's last crown sits at deck-local -0.080, so its
# cylinder reaches back to -0.115, and the deck's first roller's cylinder reaches back to
# -0.045. Any body wholly at x <= -0.115 is clear of EVERY roller of BOTH rows at EVERY z.
# PUSHER_X is derived to sit in that corridor, so the blade needs no z-based escape at all.
#
# The stow height then only has to keep the blade's top from presenting an edge to a tray
# sliding over the deck: the top sits ON the crown plane. In practice the blade also stays
# stowed until the tray is seated (the probe lifts it in DEPLOY_PUSHER, after SETTLE), so the
# two guards are independent.
# The blade's BOTTOM face is the reference, and it sits just ABOVE the crown plane. The
# earlier form (`top = 0`, `z = top - 2*half_z`) put the blade's TOP on the plane and its body
# 0.0600 m down inside the roller band -- the exact configuration that ploughs the fixed row.
# Inverting it costs nothing on the push side: the wall band is [+0.0050, +0.0650], so a blade
# occupying [+0.0010, +0.0610] still presses the wall's lower three quarters.
# THE FREE WINDOW BETWEEN THE ROLLERS AND THE TRAY IS ZERO.
#
# The crown plane (deck-local 0) IS the tray's underside, and it is also the top of the roller
# band. So a blade must be at z >= crown+clearance to miss the ROLLERS and at z <= 0 to miss the
# TRAY -- and it is 2*PUSHER_HALF[2] = 0.0300 thick. Those two cannot both hold, by 0.0300 m.
#
# D032's R1-R4 and D034's sections four and five each excluded one escape route; this excludes all
# of them at once and is the reason the stow moved ABOVE the tray instead of below it. The blade
# therefore no longer presses the wall at all, which is a MECHANISM change, not a tweak -- see
# `REQUIRED_PUSH_TRAVEL` below, which asserts the impossibility rather than merely describing it.
TRAY_TOP_DECK_LOCAL = 2.0 * (-_PUSHER_TRAY["z_min"]) + _PUSHER_TRAY["z_max"] + _PUSHER_TRAY["z_min"]
# The blade's BOTTOM face clears the tray's TOP face. Derived from the asset so a thicker tray or
# a deeper one moves the stow with it.
PUSHER_STOW_TOP_Z = TRAY_TOP_DECK_LOCAL + PUSHER_CROWN_CLEARANCE
PUSHER_STOW_Z = PUSHER_STOW_TOP_Z
# The carriage plate hangs clear BELOW the deck FRAME, not below the blade. Deriving it off
# the blade stow was the earlier version and it is WRONG: the stow bottom is -0.0600 while the
# frame's underside is -0.0950, so a plate hung off the blade sits in the band the frame
# already occupies. The frame is the ceiling; the plate goes under it.
#
# THE CEILING IS THE FRAME'S *BOTTOM* FACE, and getting that wrong cost an interpenetration.
# `deck_frame` is a box of z half-height `DECK_FRAME_HALF[2]` whose CENTRE sits at
# `frame_top - DECK_FRAME_HALF[2]`, so its band is
#     [frame_top - 2*half_z, frame_top]  =  [-0.0950, -0.0750]
# and anything placed against `frame_top - gap` is inside it. Measured before the fix:
# carriage z band -0.1000..-0.0800 against the frame's -0.0950..-0.0750 -> 0.0150 m overlap,
# and their x ranges coincide exactly at the stroke's forward stop, which is why it bit.
#
# `PUSHER_FRAME_TOP_Z` is duplicated from `build_model` deliberately rather than exported, so a
# test can compare the two and fail if they drift apart. `PUSHER_FRAME_BOTTOM_Z` is the one the
# carriage actually hangs from.
PUSHER_FRAME_TOP_Z = -2.0 * ROLLER_RADIUS - 0.005
PUSHER_FRAME_BOTTOM_Z = PUSHER_FRAME_TOP_Z - 2.0 * DECK_FRAME_HALF[2]
PUSHER_CARRIAGE_TOP_Z = PUSHER_FRAME_BOTTOM_Z - PUSHER_CARRIAGE_GAP
# DEPLOYED: the wall BOTTOM edge plus the declared engagement. Deliberately NOT the wall
# midpoint -- the push must stay low or the 0.098 kg tray pitches nose-down and its leading
# lip drops into the 0.010 m hole between crowns.
# DEPLOYED: the blade's bottom face already sits on the push line, so the deploy height
# IS the stow height. The LIFT has a different job now -- it raises the blade clear of the
# wall's top edge at the end of the push so the tray can leave without snagging -- and it is
# sized from the wall band, not from taste. The blade never climbs out of the roller band,
# because it was never in it.
PUSHER_DEPLOY_TOP_Z = PUSHER_STOW_TOP_Z
# LIFT TRAVEL: the rise needed to bring the blade top from the band bottom to that height.
# Bigger than the old rigid design's, and necessarily so: the blade now has to climb out of
# the roller band before it can touch anything.
# WHAT THE LIFT WOULD HAVE TO DO, AND WHY IT IS NOT A WORKING STROKE.
#
# `REQUIRED_PUSH_TRAVEL` is the rise the blade would need to bring its top from the stow to the
# wall's engagement height. With the stow above the tray it is NEGATIVE: the "retracted" position
# is already higher than the "engaged" one, so no positive stroke can press the wall. That number
# is kept and asserted, because it is the evidence for retiring the pusher -- and it is a tripwire:
# if a future geometry makes it positive again, a pusher becomes possible and the test says so.
REQUIRED_PUSH_TRAVEL = (PUSHER_WALL["z_hi"] + PUSHER_ENGAGE_M) - PUSHER_STOW_TOP_Z
# A joint needs a non-degenerate range (`range="0 0"` is not a range), so the lift keeps a nominal
# stroke. It is NOT a working stroke and nothing depends on it; it exists so the model is valid.
PUSHER_MIN_TRAVEL = 0.0010
PUSHER_TRAVEL = max(PUSHER_MIN_TRAVEL, REQUIRED_PUSH_TRAVEL)

# --- the carriage: where it sits, and how far it can stroke ---------------------------
#
# The tray centre is only bounded to the seat pair overlap, so the wall rear face is an
# INTERVAL. Both ends of it are used, and for different things:
#   * the FURTHEST-BACK end sets the retracted position (the blade must be behind that too,
#     or it would be ahead of a tray that stopped early);
#   * the FURTHEST-FORWARD end sets the DEPLOY stroke, because that is where the gap between
#     the retracted blade and the wall is widest.
PUSHER_WALL_REAR_LO = BEAM_SEAT_A_X - _PUSHER_TRAY_HALF
PUSHER_WALL_REAR_HI = BEAM_SEAT_B_X - _PUSHER_TRAY_HALF
# Retracted: the blade face sits behind the furthest-back wall, with a declared margin.
PUSHER_X = (PUSHER_WALL_REAR_LO - PUSHER_HALF[0]) - PUSHER_RETRACT_MARGIN
# Deploy stroke: close the widest gap between the retracted blade and the wall.
PUSHER_DEPLOY_TRAVEL = PUSHER_WALL_REAR_HI - PUSHER_X
# Slide stroke: the deploy, then the whole deck travel, then a margin for the servo to settle
# on the stop. The deploy is NOT recoverable during the same run, so the two add rather than
# overlap -- that is the whole reason the stroke is longer than DECK_TRAVEL_REF.
PUSHER_SLIDE_MARGIN = 0.020
PUSHER_SLIDE_TRAVEL = PUSHER_DEPLOY_TRAVEL + DECK_TRAVEL_REF + PUSHER_SLIDE_MARGIN
# The deck landmark the forward stop must stay behind: a blade past the last crown would
# shove the tray off the end instead of handing it over.
PUSHER_FORWARD_LIMIT = DECK_ROLLER_X0 + ROLLER_PITCH * (DECK_ROLLERS - 1)
# THE ROLLER-FREE CORRIDOR. The fixed row's last crown is the last roller under the conveyor,
# and the deck's first roller is the first one on the deck. A body wholly behind BOTH of
# their rear tangents is clear of every roller at every height, which is what lets the blade
# live in the roller band without hiding below it.
# THIS CORRIDOR IS A DERIVED FACT, NOT A PLACEMENT RULE. It is measured and TRUE --
# deck-local x <= -0.0450 clears every roller of every row -- but UNUSABLE here: the seat
# beams put the retracted blade at -0.1650, which is over the FIXED row rather than behind the
# deck, so the corridor never contained it. Kept because a later mechanism that does live
# behind the deck can rely on it, and because the test pinning it is the one that caught this.
PUSHER_CORRIDOR_X = (LAST_FIXED_CROWN_X - DECK_X0) - ROLLER_RADIUS

# --- where the carriage itself sits, and how long it is --------------------------------
#
# The carriage needs no roller corridor at all: it lives UNDER the frame, whose underside
# (-0.0850) is below the roller band bottom (-0.0700), so no roller can reach it. What bounds
# it is the frame ABOVE and the floor below.
#
# Its REAR face is pinned by the stroke's forward stop: at slide = PUSHER_SLIDE_TRAVEL the rear
# face is at DECK_LEAD_GAP exactly, i.e. touching the frame's rear edge. That is a real
# constraint, not a coincidence -- the stroke was sized to carry the blade across the deck, and
# the deck's forward edge is DECK_LEAD_GAP + DECK_LENGTH.
PUSHER_CARRIAGE_X = DECK_ROLLER_X0 - PUSHER_CARRIAGE_BACK_CLEAR
# LENGTH is DERIVED from that rear face and the stroke, so the two can never be edited apart.
# Note this is LONGER than the old hand-placed 0.300 m plate, not shorter: the fix was not to
# shrink the carriage but to move it below the frame, where length is free.
PUSHER_CARRIAGE_HALF = ((PUSHER_SLIDE_TRAVEL + 2.0 * PUSHER_CARRIAGE_BACK_CLEAR) / 2.0,
                        PUSHER_CARRIAGE_HALF_Y, PUSHER_CARRIAGE_HALF_Z)

# The blade is stowed by the LIFT joint, not by the slide: at slide = 0 the blade is still
# under the tray's path, so it must be low enough to be invisible. `PUSHER_STOW_Z` above
# already puts its top on the crown plane, which is exactly that.

# Aliases, so the mechanism is also reachable under the names the state machine uses.
PIN_X = PUSHER_X
PIN_HALF = PUSHER_HALF
PIN_STOW_Z = PUSHER_STOW_Z
PIN_TRAVEL = PUSHER_TRAVEL



def tray_clearance_bands(measurements):
    """Deck-local bands of the tray's own features, derived from the asset.

    The tray body sits `-z_min` above the crown plane, so a body-relative z becomes deck-local
    by adding that offset. Getting this frame wrong is precisely the class of error the
    geometry contract exists to catch, so the offset is returned with the bands.
    """
    offset = -measurements['z_min']

    def pick(prefix, exclude=()):
        return [p for p in measurements['parts']
                if (p['name'] or '').startswith(prefix)
                and not any((p['name'] or '').startswith(x) for x in exclude)]

    walls = pick('tray_wall')
    balls = pick('handle_', exclude=('handle_stem',))
    stems = pick('handle_stem')
    dividers = pick('tray_divider')
    floor = pick('tray_floor')
    for group, name in ((walls, 'wall'), (balls, 'ball'), (stems, 'stem'),
                        (dividers, 'divider'), (floor, 'floor')):
        if not group:
            raise SystemExit(f'tray asset has no {name} geom; the clearance band is undefined')
    return {
        'body_to_deck_z': offset,
        'wall_outer_y': max(abs(p['y_max']) for p in walls),
        'ball_inner_y': min(abs(p['y_min']) for p in balls),
        'stem_y': [min(abs(p['y_min']) for p in stems), max(abs(p['y_max']) for p in stems)],
        'stem_z': [min(p['z_min'] for p in stems) + offset,
                   max(p['z_max'] for p in stems) + offset],
        'divider_z': [min(p['z_min'] for p in dividers) + offset,
                      max(p['z_max'] for p in dividers) + offset],
        'floor_z': [min(p['z_min'] for p in floor) + offset,
                    max(p['z_max'] for p in floor) + offset],
        'guide_z_max': round(min(p['z_min'] for p in stems) + offset, 6),
    }


def _roller_row(parent, actuators, prefix, x0, count, half_length, z, mu):
    for index in range(count):
        x = x0 + ROLLER_PITCH * index
        name = f'{prefix}_{index}'
        body = ET.SubElement(parent, 'body', name=name, pos=f'{x} 0 {z}')
        ET.SubElement(body, 'joint', name=f'{name}_joint', type='hinge', axis='0 1 0',
                      damping='.01')
        ET.SubElement(body, 'geom', name=name, type='cylinder',
                      size=f'{ROLLER_RADIUS} {half_length}', quat='.70710678 .70710678 0 0',
                      mass='.4', condim='4', friction=friction_string(mu))
        ET.SubElement(actuators, 'velocity', name=name, joint=f'{name}_joint', kv='2',
                      ctrllimited='true', ctrlrange='-5 5', forcelimited='true',
                      forcerange='-2 2')


def _guide(deck, sign):
    """A straight rail only. See the guide-band comment for why no flared mouth exists."""
    ET.SubElement(deck, 'geom', name=f'guide_rail_{sign:+d}', type='box',
                  size=f'{GUIDE_HALF_X} {GUIDE_HALF_Y} {GUIDE_HALF_Z}',
                  pos=f'{GUIDE_X} {sign * GUIDE_Y} {GUIDE_Z}', mass='.3',
                  condim='4', friction=friction_string(GUIDE_FRICTION))


def apply_friction(body, mu):
    """Override the interface friction on the tray's own geoms; return what was overridden."""
    overridden = {}
    for geom in body.findall('geom'):
        was = geom.get('friction')
        if was is not None:
            overridden[geom.get('name')] = was
            geom.set('friction', friction_string(mu))
    return overridden


def build_model(tray_path, *, gap=0.0, step=0.0, yaw=0.0, tray_start_x=0.20,
                friction=DEFAULT_FRICTION, guides=False, receiver=False):
    if gap < 0:
        raise SystemExit('gap must be >= 0; a negative gap overlaps the fixed section')
    m = tray_measurements(tray_path)
    half_length = roller_half_length(m)
    root = ET.Element('mujoco', model='010 C transfer rig (real V1 tray)')
    ET.SubElement(root, 'option', timestep='.002', integrator='implicitfast')
    default = ET.SubElement(root, 'default')
    ET.SubElement(default, 'geom', friction=friction_string(friction), condim='4')
    world = ET.SubElement(root, 'worldbody')
    ET.SubElement(world, 'geom', name='floor', type='plane', size='5 5 .1',
                  friction=friction_string(friction))
    actuators = ET.SubElement(root, 'actuator')

    for section in range(FIXED_SECTIONS):
        _roller_row(world, actuators, f'fixed_roller_{section}',
                    ROLLER_PITCH * ROLLERS_PER_SECTION * section, ROLLERS_PER_SECTION,
                    half_length, ROLLER_Z, friction)

    # The receiving section is only built when the experiment asks for the receiving end. It
    # continues the SAME crown plane and the SAME pitch (see the module docstring in
    # experiments/recv_section.py for why that is the whole point), and it sits where the deck's
    # last crown lands when the deck is fully out, plus exactly one pitch.
    if receiver:
        _roller_row(world, actuators, 'recv_roller', RECV_ROLLER_X0, RECV_ROLLERS,
                    half_length, ROLLER_Z, friction)

    half_yaw = yaw / 2.0
    deck = ET.SubElement(world, 'body', name='deck',
                         pos=f'{DECK_X0 + gap} 0 {CROWN_Z + step}',
                         quat=f'{math.cos(half_yaw)} 0 0 {math.sin(half_yaw)}')
    ET.SubElement(deck, 'joint', name='deck_slide', type='slide', axis='1 0 0', damping='0')
    # 500 N was not enough. The deck carries 5 kg of frame plus 0.5 kg of pin plus eight rollers,
    # and at the end of EMBARK it drags the ROBOT: the tray's total edge reaction is mu * m_tray *
    # g / R, so a 0.098 kg tray at mu = 0.40 asks for about 11 N, but the deck's own 5.5 kg on the
    # 0.010 m frame skids against mu = 0.40 at roughly 22 N, and the 20000 N/m spring has to hold
    # its 0.15 m target against that. Saturating the actuator is indistinguishable from "the
    # mechanism stalled", which is exactly the wrong thing to leave ambiguous.
    ET.SubElement(actuators, 'position', name='deck_drive', joint='deck_slide',
                  kp='20000', kv='200', ctrllimited='true', ctrlrange='-0.5 1.5',
                  forcelimited='true', forcerange='-3000 3000')
    # The frame must clear the roller UNDERSIDES, which reach -2 * ROLLER_RADIUS. An earlier
    # revision used -ROLLER_RADIUS - 0.005, which puts the frame top inside the rollers.
    frame_top = -2.0 * ROLLER_RADIUS - 0.005
    frame_x = DECK_LEAD_GAP + DECK_FRAME_HALF[0]
    ET.SubElement(deck, 'geom', name='deck_frame', type='box',
                  size='{} {} {}'.format(*DECK_FRAME_HALF),
                  pos=f'{frame_x} 0 {frame_top - DECK_FRAME_HALF[2]}',
                  mass='5', condim='4', friction=friction_string(friction))
    _roller_row(deck, actuators, 'deck_roller', DECK_ROLLER_X0, DECK_ROLLERS, half_length,
                -ROLLER_RADIUS, friction)
    if guides:
        for sign in (-1, 1):
            _guide(deck, sign)

    # The rear pusher, as a TWO-DOF mechanism: a carriage that strokes in x and a blade that
    # lifts in z. A rigidly mounted pusher was measured infeasible (see the constants note and
    # D032): it cannot be both reachable and never-ahead. The carriage resolves that.
    #
    # Order matters in this hierarchy: the carriage's origin is the RETRACTED blade position,
    # and the blade's lift is expressed on top of it, so `pusher_slide` at 0 puts the blade
    # exactly where the constants say it is, and the geometry tests can read one frame.
    #
    # Both servos are position-controlled with the same gains as the deck drive's smaller
    # sibling. The blade actuator is deliberately weak (-30..30 N): the blade must not be able
    # to shove the tray, only to press it, so a saturated push is visible as a saturated push.
    carriage = ET.SubElement(deck, 'body', name='pusher_carriage', pos='0 0 0')
    ET.SubElement(carriage, 'joint', name='pusher_slide_joint', type='slide', axis='1 0 0',
                  damping='0', range=f'0 {PUSHER_SLIDE_TRAVEL}', limited='true')
    ET.SubElement(actuators, 'position', name='pusher_slide', joint='pusher_slide_joint',
                  kp='4000', kv='150', ctrllimited='true',
                  ctrlrange=f'0 {PUSHER_SLIDE_TRAVEL}', forcelimited='true',
                  forcerange='-200 200')
    # The carriage's own body, hung BELOW the deck FRAME. It cannot live in the roller band
    # beside the blade: at the stroke's forward stop it would touch the frame's rear edge
    # (measured, tangency at DECK_LEAD_GAP -> the lift servo saturated and the joint moved
    # 0.0 m). Under the frame there is no roller at all, so the carriage may be as long as
    # the stroke needs. Its rear face is `PUSHER_CARRIAGE_X` and its centre is half a length
    # forward of that -- the origin of `pusher_carriage` is the RETRACTED BLADE, not the
    # carriage's centre, so the two must not be conflated.
    ET.SubElement(carriage, 'geom', name='pusher_carriage_body', type='box',
                  size='{} {} {}'.format(*PUSHER_CARRIAGE_HALF),
                  pos=f'{PUSHER_CARRIAGE_X + PUSHER_CARRIAGE_HALF[0]} 0 '
                      f'{PUSHER_CARRIAGE_TOP_Z - PUSHER_CARRIAGE_HALF[2]}',
                  mass='1', condim='4', friction=friction_string(friction))
    blade = ET.SubElement(carriage, 'body', name='pusher_blade',
                          pos=f'{PUSHER_X} 0 {PUSHER_STOW_Z}')
    ET.SubElement(blade, 'joint', name='pusher_lift_joint', type='slide', axis='0 0 1',
                  damping='0', range=f'0 {PUSHER_TRAVEL}', limited='true')
    ET.SubElement(actuators, 'position', name='pusher_lift', joint='pusher_lift_joint',
                  kp='2000', kv='100', ctrllimited='true', ctrlrange=f'0 {PUSHER_TRAVEL}',
                  forcelimited='true', forcerange='-30 30')
    # `pusher_blade` sits at `PUSHER_STOW_Z`; the geom is a further `+PUSHER_HALF[2]` above
    # it, so the BOTTOM face is exactly at `PUSHER_STOW_Z` and the top at
    # `PUSHER_STOW_Z + 2 * PUSHER_HALF[2]`. The crown clearance and the wall engagement are
    # both expressed against that bottom face, which is why they read as bare z values.
    ET.SubElement(blade, 'geom', name='rear_pusher', type='box',
                  size='{} {} {}'.format(*PUSHER_HALF), pos=f'0 0 {PUSHER_HALF[2]}',
                  mass='.5', condim='4', friction=friction_string(friction))

    tray = load_tray_body(tray_path)
    overridden = apply_friction(tray, friction)
    tray.set('pos', f'{tray_start_x} 0 {CROWN_Z - m["z_min"]}')
    world.append(tray)

    meta = {'tray': {k: v for k, v in m.items() if k != 'parts'},
            'roller_half_length': half_length, 'deck_x0': DECK_X0,
            'deck_length': DECK_LENGTH, 'deck_surface_z': CROWN_Z + step,
            'crown_z': CROWN_Z, 'handle_clearance': handle_clearance(m),
            'handle_overhang': m['y_max'] - half_length,
            'guide_band': tray_clearance_bands(m), 'guides': guides,
            'friction': friction, 'friction_overridden_on_asset_geoms': len(overridden),
            'gap': gap, 'step': step, 'yaw': yaw,
            'tray_start': [tray_start_x, 0.0, CROWN_Z - m['z_min']],
            'receiver': bool(receiver),
            'recv_first_crown_x': RECV_FIRST_CROWN_X,
            'deck_last_crown_x': DECK_LAST_CROWN_X,
            'recv_handoff_gap_m': RECV_FIRST_CROWN_X - DECK_LAST_CROWN_X,
            'recv_rollers': RECV_ROLLERS,
            'roller_pitch': ROLLER_PITCH, 'roller_radius': ROLLER_RADIUS,
            'deck_travel': DECK_TRAVEL_REF,
            'pusher_x': PUSHER_X, 'pusher_half': list(PUSHER_HALF),
            'pusher_stow_z': PUSHER_STOW_Z, 'pusher_lift_travel': PUSHER_TRAVEL,
            'pusher_deploy_travel': PUSHER_DEPLOY_TRAVEL,
            'pusher_slide_travel': PUSHER_SLIDE_TRAVEL,
            'pusher_wall_rear_lo': PUSHER_WALL_REAR_LO,
            'pusher_wall_rear_hi': PUSHER_WALL_REAR_HI,
            'pusher_forward_limit': PUSHER_FORWARD_LIMIT,
            'pusher_carriage_x': PUSHER_CARRIAGE_X,
            'pusher_carriage_half': list(PUSHER_CARRIAGE_HALF),
            'pusher_carriage_top_z': PUSHER_CARRIAGE_TOP_Z,
            'pusher_frame_top_z': PUSHER_FRAME_TOP_Z,
            'pusher_crown_clearance': PUSHER_CROWN_CLEARANCE,
            'pusher_deploy_top_z': PUSHER_DEPLOY_TOP_Z}
    return ET.tostring(root, encoding='unicode'), meta
