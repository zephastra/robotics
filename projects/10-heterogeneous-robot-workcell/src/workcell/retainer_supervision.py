"""Retainer supervision contract: state machine + observation contract + authorization.

G1 (docs/CONTINUATION_TO_V1_COMPLETION.md §4). The v7 hinged retainer was an
*experimental probe*: it judged "was there contact during the closed window"
and then unconditionally wrote ``transport_authorized = False``. That is a
bench diagnostic, not a supervised skill.

This module is the contract layer only:

* It reads measurements. It never steps physics, never writes qpos/qvel, never
  touches a world file. There is no ``mujoco`` import here on purpose, and
  ``tests/test_retainer_supervision.py`` asserts that.
* A single contact boolean is NEVER sufficient for transport authorization.
  Authorization is a *conjunction* over independently-declared evidence.
* UNKNOWN is a first-class outcome. It never becomes PASS, and it never
  authorizes. A missing field is UNKNOWN, not "fine".
* Every observation carries identity (vehicle, mechanism, generation/epoch,
  sequence, run), both joints' position AND speed, per-side contact existence
  plus its support source, contact force with units, and a wall timestamp
  whose age is checked.

Why the state machine exists: mechanical position reaching the closed value
does NOT mean the tray is genuinely held on both sides. ``CLOSED`` (geometry
arrived) and ``CLOSED_VERIFIED`` (measured bilateral support with fresh
evidence) are deliberately DIFFERENT states. Collapsing them is the defect
this module is designed to make impossible.
"""
import math

# ---------------------------------------------------------------------------
# Declared vocabulary. Aligned with the existing repo words where they exist
# (UNKNOWN, NOT_RUN, REFUSED_*) rather than inventing a parallel dialect.
# ---------------------------------------------------------------------------

STATES = (
    'OPEN',              # nothing asserted about retention
    'CLOSING',           # a close command is outstanding, not yet measured
    'CLOSED',            # geometry arrived; support NOT yet verified
    'CLOSED_VERIFIED',   # bilateral support measured, evidence fresh
    'TRANSPORTING',      # motion permitted under a live authorization
    'RELEASING',         # a release command is outstanding
    'OPEN_VERIFIED',     # measured clear of the tray, release complete
    'FAULT',             # a declared, named failure state
    'UNKNOWN',           # evidence missing/invalid; asserts nothing
)

# Terminal-in-this-run failure reasons. Each must be distinguishable.
FAULT_REASONS = (
    'CLOSE_TIMEOUT',
    'CLOSE_NOT_AT_TARGET',
    'RELEASE_TIMEOUT',
    'SINGLE_SIDE_SUPPORT',       # only one shoe bears on the tray
    'WRONG_VEHICLE',             # observation from a different vehicle
    'STALE_EPOCH',               # authorization identity changed
    'STALE_OBSERVATION',         # wall-age exceeded
    'SEQUENCE_REGRESSED',
    'CANCEL_REQUESTED',
    'DUPLICATE_COMMAND',
    'COMMAND_CONFLICT',
    'OBSERVATION_INVALID',
    'INITIAL_VISUAL_UNKNOWN',
    'RELEASE_NOT_CLEARED',
    'RELEASE_NOT_AT_TARGET',
    'RUN_IDENTITY_MISMATCH',
)

# The two joints of the declared v7 mechanism. Order is the declared order;
# callers must key by name, never by integer index (actuator count grew
# 135 -> 137 in v7 and index-based mapping is exactly the bug we refuse).
MECHANISM_SIDES = ('c_retainer_-1', 'c_retainer_1')

# Units are declared, not implied. Force is newtons.
FORCE_UNITS = 'N'
POSITION_UNITS = 'rad'      # HINGE_RAD; v6 SLIDE_M declares 'm'
SPEED_UNITS = 'rad/s'

MIN_FORCE_N = 1e-9          # a "contact" with no measurable normal force is not support


def _finite(value):
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


class ContractRefused(Exception):
    """A malformed declaration. Raised loudly; never downgraded to a PASS."""

    def __init__(self, reason, detail=''):
        super().__init__('%s: %s' % (reason, detail) if detail else reason)
        self.reason = reason
        self.detail = detail


# ---------------------------------------------------------------------------
# Observation contract
# ---------------------------------------------------------------------------

def validate_observation(observation, *, expect_vehicle, expect_epoch, expect_generation,
                         expect_run_id, max_age_s, now_wall_s, last_sequence=-1):
    """Return (ok, reason, detail). A missing field is a refusal, not a default.

    This is deliberately a *strict* reader: it refuses on absence, on
    contradiction (force claimed without contact), and on wrong identity.
    """
    if not isinstance(observation, dict):
        return False, 'OBSERVATION_INVALID', 'not a mapping'
    required = ('vehicle_id', 'mechanism', 'epoch', 'generation', 'sequence',
                'run_id', 'observed_wall_s', 'sides', 'visual')
    for field in required:
        if field not in observation:
            return False, 'OBSERVATION_INVALID', 'missing field %r' % (field,)
    if observation['vehicle_id'] != expect_vehicle:
        return False, 'WRONG_VEHICLE', 'observed %r != %r' % (observation['vehicle_id'], expect_vehicle)
    if observation['run_id'] != expect_run_id:
        return False, 'RUN_IDENTITY_MISMATCH', 'observed %r != %r' % (observation['run_id'], expect_run_id)
    if observation['mechanism'] != 'retainer':
        return False, 'OBSERVATION_INVALID', 'mechanism %r' % (observation['mechanism'],)
    if type(observation['epoch']) is not int or observation['epoch'] != expect_epoch:
        return False, 'STALE_EPOCH', 'epoch %r != %r' % (observation['epoch'], expect_epoch)
    if type(observation['generation']) is not int or observation['generation'] != expect_generation:
        return False, 'STALE_EPOCH', 'generation %r != %r' % (observation['generation'], expect_generation)
    sequence = observation['sequence']
    if type(sequence) is not int or sequence < 0:
        return False, 'OBSERVATION_INVALID', 'sequence %r' % (sequence,)
    if sequence <= last_sequence:
        return False, 'SEQUENCE_REGRESSED', 'sequence %r <= %r' % (sequence, last_sequence)
    stamp = observation['observed_wall_s']
    if not _finite(stamp):
        return False, 'OBSERVATION_INVALID', 'observed_wall_s %r' % (stamp,)
    if not _finite(now_wall_s):
        return False, 'OBSERVATION_INVALID', 'now_wall_s %r' % (now_wall_s,)
    if stamp > now_wall_s:
        return False, 'OBSERVATION_INVALID', 'observation from the future'
    if now_wall_s - stamp > max_age_s:
        return False, 'STALE_OBSERVATION', 'age %.4f > %.4f' % (now_wall_s - stamp, max_age_s)
    sides = observation['sides']
    if not isinstance(sides, dict) or set(sides) != set(MECHANISM_SIDES):
        return False, 'OBSERVATION_INVALID', 'sides must be exactly %r' % (MECHANISM_SIDES,)
    for name in MECHANISM_SIDES:
        side = sides[name]
        if not isinstance(side, dict):
            return False, 'OBSERVATION_INVALID', '%s not a mapping' % (name,)
        for field in ('position', 'speed', 'contact', 'support_source', 'normal_force_n'):
            if field not in side:
                return False, 'OBSERVATION_INVALID', '%s missing %r' % (name, field)
        if not _finite(side['position']):
            return False, 'OBSERVATION_INVALID', '%s position %r' % (name, side['position'])
        if not _finite(side['speed']):
            return False, 'OBSERVATION_INVALID', '%s speed %r' % (name, side['speed'])
        if side['contact'] not in (True, False):
            return False, 'OBSERVATION_INVALID', '%s contact %r' % (name, side['contact'])
        force = side['normal_force_n']
        if not _finite(force):
            return False, 'OBSERVATION_INVALID', '%s normal_force_n %r' % (name, force)
        if force < 0:
            return False, 'OBSERVATION_INVALID', '%s negative normal force' % (name,)
        # Contradiction: force claimed where contact is denied.
        if side['contact'] is False and force > MIN_FORCE_N:
            return False, 'OBSERVATION_INVALID', '%s force without contact' % (name,)
        # Contradiction: contact claimed with no measurable force.
        if side['contact'] is True and force <= MIN_FORCE_N:
            return False, 'OBSERVATION_INVALID', '%s contact without measurable force' % (name,)
        if side['contact'] is True and not isinstance(side['support_source'], str):
            return False, 'OBSERVATION_INVALID', '%s support_source %r' % (name, side['support_source'])
    visual = observation['visual']
    if not isinstance(visual, dict) or 'status' not in visual:
        return False, 'OBSERVATION_INVALID', 'visual contract missing'
    if visual['status'] not in ('RESOLVED', 'UNKNOWN'):
        return False, 'OBSERVATION_INVALID', 'visual status %r' % (visual['status'],)
    return True, None, None


def make_observation(*, vehicle_id, epoch, generation, sequence, run_id, observed_wall_s,
                     sides, visual):
    """Assemble a conforming observation. Raises ContractRefused on bad input.

    ``sides`` maps each mechanism side to position/speed/contact/support_source/
    normal_force_n. Callers should build this from measurement; this helper only
    normalizes shape and identity.
    """
    if not isinstance(sides, dict) or set(sides) != set(MECHANISM_SIDES):
        raise ContractRefused('OBSERVATION_INVALID', 'sides must be exactly %r' % (MECHANISM_SIDES,))
    out = {'vehicle_id': vehicle_id, 'mechanism': 'retainer', 'epoch': epoch,
           'generation': generation, 'sequence': sequence, 'run_id': run_id,
           'observed_wall_s': observed_wall_s, 'sides': {}, 'visual': visual,
           'position_units': POSITION_UNITS, 'speed_units': SPEED_UNITS,
           'force_units': FORCE_UNITS}
    for name in MECHANISM_SIDES:
        side = sides[name]
        out['sides'][name] = {'position': float(side['position']), 'speed': float(side['speed']),
                             'contact': bool(side['contact']),
                             'support_source': side['support_source'],
                             'normal_force_n': float(side['normal_force_n'])}
    return out


# ---------------------------------------------------------------------------
# Combined transport authorization
# ---------------------------------------------------------------------------

def transport_authorization(observation, *, expect_vehicle, expect_epoch, expect_generation,
                            expect_run_id, now_wall_s, max_age_s, last_sequence=-1,
                            close_target, close_tol, release_target, release_tol,
                            state):
    """The conjunction. Returns dict(allowed, reason_code, detail, evidence).

    NO single field is sufficient. In particular:
      * "the joint reached the closed angle" is NOT sufficient;
      * "there is contact on one side" is NOT sufficient;
      * "the camera resolved" is NOT sufficient and is not required for the
        *closed* verdict only -- an unresolved camera forces UNKNOWN, which
        refuses authorization outright.
    """
    ok, reason, detail = validate_observation(
        observation, expect_vehicle=expect_vehicle, expect_epoch=expect_epoch,
        expect_generation=expect_generation, expect_run_id=expect_run_id,
        max_age_s=max_age_s, now_wall_s=now_wall_s, last_sequence=last_sequence)
    if not ok:
        return dict(allowed=False, reason_code=reason, detail=detail, evidence=None)

    sides = observation['sides']
    positions = {n: sides[n]['position'] for n in MECHANISM_SIDES}
    contacts = {n: sides[n]['contact'] for n in MECHANISM_SIDES}
    forces = {n: sides[n]['normal_force_n'] for n in MECHANISM_SIDES}
    at_close = {n: abs(positions[n] - close_target) <= close_tol for n in MECHANISM_SIDES}
    at_release = {n: positions[n] >= release_target - release_tol for n in MECHANISM_SIDES}
    evidence = dict(positions=positions, contacts=contacts, forces=forces,
                    at_close=at_close, at_release=at_release,
                    visual_status=observation['visual']['status'],
                    support_sources={n: sides[n]['support_source'] for n in MECHANISM_SIDES})

    # 1. Camera: an unresolved initial scene is UNKNOWN, never a pass.
    if observation['visual']['status'] != 'RESOLVED':
        return dict(allowed=False, reason_code='INITIAL_VISUAL_UNKNOWN',
                    detail='visual status %r' % (observation['visual']['status'],),
                    evidence=evidence)

    # 2. Geometry must be at the closed target on BOTH sides.
    if not all(at_close.values()):
        # Geometry is not at the close target. Distinct from the declared
        # wall budget expiring: one is a position, the other is a clock.
        return dict(allowed=False, reason_code='CLOSE_NOT_AT_TARGET',
                    detail='not at close target: %r' % (at_close,), evidence=evidence)

    # 3. Bilateral support. One side is a named failure, not a partial pass.
    if not all(contacts.values()):
        missing = sorted(n for n in MECHANISM_SIDES if not contacts[n])
        return dict(allowed=False, reason_code='SINGLE_SIDE_SUPPORT',
                    detail='no contact on %r' % (missing,), evidence=evidence)

    # 4. Contact must name its support source.
    #
    # The "contact must carry measurable force" requirement lives in
    # `validate_observation` (`contact is True and force <= MIN_FORCE_N` is
    # refused as a self-contradiction), and step 0 above always runs it. A
    # duplicate check here was mutation-tested: disabling it changed no test,
    # because it was unreachable. Rather than keep an unfalsifiable branch, the
    # requirement is asserted once, in the validator, where a test can reach it.
    for name in MECHANISM_SIDES:
        if sides[name]['support_source'] != 'tray':
            return dict(allowed=False, reason_code='OBSERVATION_INVALID',
                        detail='%s supported by %r, not the tray'
                               % (name, sides[name]['support_source']), evidence=evidence)

    # 5. State must not be a release/opening phase.
    if state not in ('CLOSED_VERIFIED', 'TRANSPORTING'):
        return dict(allowed=False, reason_code='COMMAND_CONFLICT',
                    detail='state %r does not authorize transport' % (state,), evidence=evidence)

    return dict(allowed=True, reason_code=None, detail='bilateral verified support',
                evidence=evidence)


# ---------------------------------------------------------------------------
# State machine
# ---------------------------------------------------------------------------

# Which states may follow which. Conservative: anything undeclared is refused.
# CANCEL_REQUESTED is a *reason*, not a state: a cancel parks the mechanism in
# FAULT, because "the caller withdrew" tells us nothing about the mechanism.
TRANSITIONS = {
    'OPEN': ('CLOSING', 'FAULT', 'UNKNOWN'),
    'CLOSING': ('CLOSED', 'FAULT', 'UNKNOWN'),
    # CLOSED -> TRANSPORTING is deliberately NOT allowed. Transport requires the
    # CLOSED_VERIFIED rung, which is the whole point of separating the two.
    'CLOSED': ('CLOSED_VERIFIED', 'FAULT', 'UNKNOWN'),
    'CLOSED_VERIFIED': ('TRANSPORTING', 'RELEASING', 'FAULT', 'UNKNOWN'),
    'TRANSPORTING': ('CLOSED_VERIFIED', 'RELEASING', 'FAULT', 'UNKNOWN'),
    'RELEASING': ('OPEN_VERIFIED', 'FAULT', 'UNKNOWN'),
    'OPEN_VERIFIED': ('FAULT', 'UNKNOWN'),
    # FAULT and UNKNOWN are absorbing in this run: recovery needs a NEW run,
    # because "the flag recovered" is not evidence that the mechanism did.
    'FAULT': (),
    'UNKNOWN': (),
}


class RetainerSupervisor:
    """Declared lifecycle for one retainer on one vehicle. Never writes physics.

    Every method takes an observation and returns a decision dict. Nothing here
    mutates the simulation; the caller is the only party that may command an
    actuator, and it may only do so after ``decision['admitted']``.
    """

    def __init__(self, *, vehicle_id, run_id, epoch, generation, max_age_s,
                 close_target, close_tol, release_target, release_tol,
                 close_deadline_s=None, release_deadline_s=None):
        for label, value in (('vehicle_id', vehicle_id), ('run_id', run_id)):
            if not isinstance(value, str) or not value:
                raise ContractRefused('OBSERVATION_INVALID', '%s=%r' % (label, value))
        if type(epoch) is not int or epoch < 0 or type(generation) is not int or generation < 0:
            raise ContractRefused('OBSERVATION_INVALID', 'bad epoch/generation')
        if not _finite(max_age_s) or max_age_s <= 0:
            raise ContractRefused('OBSERVATION_INVALID', 'max_age_s=%r' % (max_age_s,))
        for label, value in (('close_deadline_s', close_deadline_s),
                             ('release_deadline_s', release_deadline_s)):
            if value is not None and (not _finite(value) or value <= 0):
                raise ContractRefused('OBSERVATION_INVALID', '%s=%r' % (label, value))
        self.vehicle_id, self.run_id = vehicle_id, run_id
        self.epoch, self.generation, self.max_age_s = epoch, generation, float(max_age_s)
        self.close_target, self.close_tol = float(close_target), float(close_tol)
        self.release_target, self.release_tol = float(release_target), float(release_tol)
        # Declared wall-clock budgets. Without these, CLOSE_TIMEOUT and
        # RELEASE_TIMEOUT are declared reasons that nothing can ever raise -- the
        # mutation harness proved that by disabling the deadline and seeing no
        # test fail. A deadline is a DECLARATION, not a derived threshold.
        self.close_deadline_s = close_deadline_s
        self.release_deadline_s = release_deadline_s
        self.phase_started_wall_s = None
        self.state = 'OPEN'
        self.last_sequence = -1
        self.last_wall_s = None
        self.latched = None          # reason_code once FAULT/UNKNOWN is entered
        self.history = ['OPEN']
        self.last_evidence = None
        self.pending_command = None  # outstanding close/release
        self.commands_seen = []

    # -- deadlines ---------------------------------------------------------
    def _start_phase(self, now_wall_s):
        self.phase_started_wall_s = now_wall_s

    def _deadline_exceeded(self, now_wall_s):
        # Clock domain: `now_wall_s` is whatever clock the integration
        # stamps; this module only diffs it. The G1 probe stamps sim time
        # (declared in its report as contract_clock_domain).
        """Return the timeout reason if the current phase has run out of budget."""
        if self.phase_started_wall_s is None or not _finite(now_wall_s):
            return None
        elapsed = now_wall_s - self.phase_started_wall_s
        if self.state == 'CLOSING' and self.close_deadline_s is not None:
            if elapsed > self.close_deadline_s:
                return 'CLOSE_TIMEOUT'
        if (self.state in ('RELEASING', 'CLOSED_VERIFIED', 'TRANSPORTING')
                and self.release_deadline_s is not None
                and self.pending_command == 'release'):
            if elapsed > self.release_deadline_s:
                return 'RELEASE_TIMEOUT'
        return None

    # Reasons that yield UNKNOWN rather than FAULT: the mechanism position is not
    # known to be bad, the *evidence* is missing. Conflating the two would let a
    # missing camera be reported as a mechanical failure, or vice versa.
    UNKNOWN_REASONS = ('INITIAL_VISUAL_UNKNOWN', 'OBSERVATION_INVALID',
                       'STALE_OBSERVATION', 'STALE_EPOCH', 'SEQUENCE_REGRESSED',
                       'WRONG_VEHICLE', 'RUN_IDENTITY_MISMATCH')

    # -- internal ----------------------------------------------------------
    def _fault(self, reason, detail=''):
        self.latched = reason
        target = 'UNKNOWN' if reason in self.UNKNOWN_REASONS else 'FAULT'
        self._enter(target)
        return dict(admitted=False, authorized=False, state=self.state,
                    reason_code=reason, detail=detail, evidence=self.last_evidence)

    def _enter(self, state):
        if state not in STATES:
            raise ContractRefused('OBSERVATION_INVALID', 'unknown state %r' % (state,))
        allowed = TRANSITIONS.get(self.state, ())
        if state not in allowed and state != self.state:
            raise ContractRefused('COMMAND_CONFLICT',
                                  'illegal transition %s -> %s' % (self.state, state))
        self.state = state
        self.history.append(state)

    def _latched(self):
        """Once latched, the latch is authoritative: no re-derivation, ever.

        Re-running the conjunction after a fault would let a later, luckier
        sample silently clear a fault that was already recorded. That is exactly
        the "the flag recovered, so the mechanism must have" defect.
        """
        if self.latched is None:
            return None
        return dict(admitted=False, authorized=False, allowed=False, state=self.state,
                    reason_code=self.latched, detail='latched; a new run is required',
                    evidence=self.last_evidence)

    def _read(self, observation, now_wall_s):
        ok, reason, detail = validate_observation(
            observation, expect_vehicle=self.vehicle_id, expect_epoch=self.epoch,
            expect_generation=self.generation, expect_run_id=self.run_id,
            max_age_s=self.max_age_s, now_wall_s=now_wall_s,
            last_sequence=self.last_sequence)
        if not ok:
            return None, reason, detail
        self.last_sequence = observation['sequence']
        self.last_wall_s = now_wall_s
        self.last_evidence = dict(observation=observation, read_wall_s=now_wall_s)
        return observation, None, None

    # -- lifecycle ---------------------------------------------------------
    def command_close(self, name='close', now_wall_s=None):
        """Record an outstanding close. Duplicate commands are a named fault."""
        latched = self._latched()
        if latched is not None:
            return latched
        if self.state != 'OPEN':
            # A second close after the mechanism already moved is a conflict, not a
            # duplicate: "duplicate" means the same request twice while still OPEN.
            reason = 'DUPLICATE_COMMAND' if name in self.commands_seen else 'COMMAND_CONFLICT'
            return self._fault(reason, 'close from %s' % (self.state,))
        if name in self.commands_seen:
            return self._fault('DUPLICATE_COMMAND', 'close already issued')
        self._command_wall_s = now_wall_s
        self.commands_seen.append(name)
        self.pending_command = 'close'
        self._enter('CLOSING')
        # The close budget starts when the command is issued, not at the first
        # observation: a mechanism that never produces an observation at all is
        # exactly the case CLOSE_TIMEOUT must catch.
        if now_wall_s is not None:
            self._start_phase(now_wall_s)
        return dict(admitted=True, authorized=False, state=self.state,
                    reason_code=None, detail='closing', evidence=None)

    def command_release(self, name='release', now_wall_s=None):
        """Declare intent to release, starting the release budget.

        Split from `release()` so the budget starts at the COMMAND, not at the
        first measurement. Without this, a mechanism that produces no further
        observation never times out.
        """
        if self.latched is not None:
            latched = self._latched()
            if latched is not None:
                return latched
        if name in self.commands_seen:
            return self._fault('DUPLICATE_COMMAND', 'release already issued')
        if self.state not in ('TRANSPORTING', 'CLOSED_VERIFIED'):
            return self._fault('COMMAND_CONFLICT', 'release from %s' % (self.state,))
        self.commands_seen.append(name)
        self.pending_command = 'release'
        if now_wall_s is not None:
            self._start_phase(now_wall_s)
        return dict(admitted=True, allowed=True, authorized=False, state=self.state,
                    reason_code=None, detail='release commanded', evidence=None)

    def cancel(self, observation, *, now_wall_s):
        """A cancel is a result, not an exception. It grants nothing.

        Cancel parks the mechanism in FAULT with reason CANCEL_REQUESTED; there is
        no separate CANCEL_REQUESTED state, because withdrawal is a reason and the
        mechanical position is still whatever it was.
        """
        latched = self._latched()
        if latched is not None:
            return latched
        read, reason, detail = self._read(observation, now_wall_s)
        if read is None:
            return self._fault(reason, detail)
        self.pending_command = None
        return self._fault('CANCEL_REQUESTED', 'cancelled by caller')

    def observe_closed(self, observation, *, now_wall_s):
        """Measure the closed state. CLOSED (geometry) vs CLOSED_VERIFIED (support).

        These are two DIFFERENT rungs on purpose, and each needs its own
        observation call. Arriving at CLOSED never grants support verification in
        the same step: a single sample that shows both the angle and a contact is
        still one sample, and one sample is not bilateral proof over time.
        """
        latched = self._latched()
        if latched is not None:
            return latched
        read, reason, detail = self._read(observation, now_wall_s)
        if read is None:
            return self._fault(reason, detail)
        if self.state == 'CLOSING':
            if self.phase_started_wall_s is None:
                self._start_phase(now_wall_s)
            timeout = self._deadline_exceeded(now_wall_s)
            if timeout is not None:
                return self._fault(timeout, 'close budget %.4fs exceeded'
                                   % (self.close_deadline_s,))
        positions = {n: read['sides'][n]['position'] for n in MECHANISM_SIDES}
        if self.state == 'CLOSING':
            if not all(abs(positions[n] - self.close_target) <= self.close_tol
                       for n in MECHANISM_SIDES):
                return dict(admitted=False, authorized=False, state=self.state,
                            reason_code=None, detail='still closing',
                            evidence=dict(positions=positions))
            self._enter('CLOSED')
            return dict(admitted=False, authorized=False, state=self.state, reason_code=None,
                        detail='geometry arrived; support NOT yet verified',
                        evidence=dict(positions=positions))
        if self.state == 'CLOSED':
            contacts = {n: read['sides'][n]['contact'] for n in MECHANISM_SIDES}
            if not all(contacts.values()):
                # Separately: this is the single-side retention counterexample.
                return dict(admitted=False, authorized=False, state=self.state,
                            reason_code=None, detail='support not yet bilateral',
                            evidence=dict(contacts=contacts))
            if read['visual']['status'] != 'RESOLVED':
                return self._fault('INITIAL_VISUAL_UNKNOWN', 'camera unresolved at close')
            self._enter('CLOSED_VERIFIED')
            return dict(admitted=False, authorized=False, state=self.state, reason_code=None,
                        detail='bilateral support verified', evidence=dict(contacts=contacts))
        return dict(admitted=False, authorized=False, state=self.state, reason_code=None,
                    detail='closed observed from %s' % (self.state,),
                    evidence=dict(positions=positions))

    def authorize_transport(self, observation, *, now_wall_s):
        """The only path to motion. Conjunction over the full contract."""
        latched = self._latched()
        if latched is not None:
            return latched
        read, reason, detail = self._read(observation, now_wall_s)
        if read is None:
            return self._fault(reason, detail)
        decision = transport_authorization(
            read, expect_vehicle=self.vehicle_id, expect_epoch=self.epoch,
            expect_generation=self.generation, expect_run_id=self.run_id,
            now_wall_s=now_wall_s, max_age_s=self.max_age_s,
            last_sequence=-1,  # already advanced by _read
            close_target=self.close_target, close_tol=self.close_tol,
            release_target=self.release_target, release_tol=self.release_tol,
            state=self.state)
        if not decision['allowed']:
            return self._fault(decision['reason_code'], decision['detail'])
        if self.state == 'CLOSED_VERIFIED':
            self._enter('TRANSPORTING')
        return dict(admitted=True, allowed=True, authorized=True, state=self.state,
                    reason_code=None, detail=decision['detail'], evidence=decision['evidence'])

    def release(self, observation, *, now_wall_s):
        """Release is only meaningful from TRANSPORTING (or CLOSED_VERIFIED)."""
        latched = self._latched()
        if latched is not None:
            return latched
        read, reason, detail = self._read(observation, now_wall_s)
        if read is None:
            return self._fault(reason, detail)
        if self.state not in ('TRANSPORTING', 'CLOSED_VERIFIED'):
            return self._fault('COMMAND_CONFLICT', 'release from %s' % (self.state,))
        timeout = self._deadline_exceeded(now_wall_s)
        if timeout is not None:
            return self._fault(timeout, 'release budget %.4fs exceeded'
                               % (self.release_deadline_s,))
        positions = {n: read['sides'][n]['position'] for n in MECHANISM_SIDES}
        if not all(positions[n] >= self.release_target - self.release_tol for n in MECHANISM_SIDES):
            # Geometry has not reached the release position. This is NOT the same
            # as exceeding the declared wall budget; sharing one reason code would
            # make two different failures indistinguishable.
            return dict(admitted=False, authorized=False, state=self.state,
                        reason_code='RELEASE_NOT_AT_TARGET',
                        detail='not at release target: %r' % (positions,),
                        evidence=dict(positions=positions))
        contacts = {n: read['sides'][n]['contact'] for n in MECHANISM_SIDES}
        if any(contacts.values()):
            return dict(admitted=False, authorized=False, state=self.state,
                        reason_code='RELEASE_NOT_CLEARED',
                        detail='still in contact: %r' % (contacts,),
                        evidence=dict(contacts=contacts))
        self._enter('RELEASING')
        self._enter('OPEN_VERIFIED')
        self.pending_command = None
        return dict(admitted=True, allowed=True, authorized=False, state='OPEN_VERIFIED', reason_code=None,
                    detail='release verified clear', evidence=dict(positions=positions))

    # -- reporting ---------------------------------------------------------
    def snapshot(self):
        return dict(vehicle_id=self.vehicle_id, run_id=self.run_id, epoch=self.epoch,
                    generation=self.generation, state=self.state, latched=self.latched,
                    history=list(self.history), last_sequence=self.last_sequence,
                    transport_authorized=bool(self.state == 'TRANSPORTING' and self.latched is None),
                    declared_states=list(STATES), fault_reasons=list(FAULT_REASONS))
