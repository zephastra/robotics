"""Resources: who holds what, until when, and whether a late arrival may extend it.

`docs/CONTRACTS.md` section 6:

    资源 FREE/RESERVED/OCCUPIED/CLEARING/UNKNOWN/BLOCKED；默认 UNKNOWN，需要初始化清空证据。
    授权带 owner、generation、epoch、ttl；延迟响应不能重新计满 TTL；已占用时超时不释放。

Every rule in that paragraph is a rule about a *stale answer arriving late*, and that is what
shapes this module. The failure it prevents is not "the resource was taken" -- it is "the
resource was taken, then released, then a reply from before the release arrived and everyone
agreed the resource was free while something was still standing on it."

Three consequences, and they are the whole file:

* **`UNKNOWN` is the only starting state.** Nothing is `FREE` until initialisation says so, with
  evidence. A resource that defaults to `FREE` is a resource whose first grant happens before
  anyone looked.

* **A grant carries a generation and an epoch, and a late answer is refused against them.**
  `generation` counts grants on one resource; `epoch` identifies the coordinator's own run. A
  reply tagged with an older generation is not a renewal, it is history -- so it cannot re-fill
  the TTL. This is the "延迟响应不能重新计满 TTL" rule made structural: without a generation
  there is no way to tell a renewal from a resurrection.

* **Expiry releases only what expiry owns.** A lease that lapses frees a `RESERVED` resource;
  it does not free an `OCCUPIED` one, because `OCCUPIED` means something is physically on it
  and a clock running out is not evidence that it moved. That is "已占用时超时不释放".

The module is pure and clock-injected: every method takes `now_s` rather than calling a clock,
so a test can drive a five-minute lease through its whole life in microseconds, and so the
ledger cannot accidentally consult wall time while the rest of the system runs on sim time.
"""
from . import schema as S
from .schema import SchemaRefused

#: The six states, from the contract. Imported rather than retyped, so adding one to the
#: contract does not leave this module quietly accepting a subset.
STATES = S.RESOURCE_STATES


class Grant:
    """One resource, granted to one owner, valid for one window.

    The identity of a grant is `(resource_id, owner, generation, epoch)` and all four are
    compared on every renewal. Comparing fewer would let a reply from a different owner, or
    from before a restart, look like a continuation of this grant.
    """

    __slots__ = ('resource_id', 'owner', 'generation', 'epoch', 'ttl_s',
                 'granted_at_s', 'state')

    def __init__(self, resource_id, owner, generation, epoch, ttl_s, granted_at_s, state):
        self.resource_id = resource_id
        self.owner = owner
        self.generation = generation
        self.epoch = epoch
        self.ttl_s = float(ttl_s)
        self.granted_at_s = float(granted_at_s)
        self.state = state

    @property
    def expires_at_s(self):
        return self.granted_at_s + self.ttl_s

    def to_dict(self):
        return {
            'resource_id': self.resource_id,
            'owner': self.owner,
            'generation': self.generation,
            'epoch': self.epoch,
            'ttl_s': self.ttl_s,
            'granted_at_s': self.granted_at_s,
            'expires_at_s': self.expires_at_s,
            'state': self.state,
        }

    def __repr__(self):  # pragma: no cover -- debugging aid
        return (f'<Grant {self.resource_id} owner={self.owner} gen={self.generation} '
                f'ep={self.epoch} state={self.state} ttl={self.ttl_s}>')


class ResourceTable:
    """The resource states and the grants over them.

    One instance is one coordinator's view. `epoch` is passed in at construction and is the
    coordinator's own run identifier: after a restart a new table gets a new epoch, so a reply
    that was in flight across the restart is recognisable as belonging to the old run.
    """

    def __init__(self, *, epoch, resources):
        self.epoch = int(epoch)
        self.state = {}
        self.generation = {}
        self.grant = {}
        self.reasons = {}       # resource_id -> last refusal reason (countable)
        self.refusal_count = {}
        for resource_id in resources:
            self._check_id(resource_id)
            if resource_id in self.state:
                raise SchemaRefused('REFUSED_CONFLICT',
                                    f'duplicate resource {resource_id!r}', field='resources')
            # ★ Default UNKNOWN, per the contract. Not FREE: a resource nobody has cleared yet
            # is a resource whose first grant would otherwise happen before anyone looked.
            self.state[resource_id] = 'UNKNOWN'
            self.generation[resource_id] = 0
            self.grant[resource_id] = None

    # -- helpers -----------------------------------------------------------
    @staticmethod
    def _check_id(resource_id):
        if not isinstance(resource_id, str) or not S.ID_PATTERN.match(resource_id):
            raise SchemaRefused('REFUSED_BAD_ID', repr(resource_id), field='resource_id')
        return resource_id

    def _require(self, resource_id):
        self._check_id(resource_id)
        if resource_id not in self.state:
            raise SchemaRefused('REFUSED_ENTITY_UNKNOWN',
                                f'no such resource {resource_id!r}', field='resource_id')
        return resource_id

    def _refuse(self, resource_id, reason):
        self.reasons[resource_id] = reason
        self.refusal_count[reason] = self.refusal_count.get(reason, 0) + 1
        raise SchemaRefused(reason, f'resource {resource_id!r}', field='resource_id')

    # -- initialisation ----------------------------------------------------

    def mark_cleared(self, resource_id, *, evidence, now_s):
        """Move a resource from UNKNOWN to FREE, and only with evidence.

        The contract says "需要初始化清空证据". So `evidence` is required and must be a
        non-empty reference: "I looked and it was empty" is a claim that needs a source, and a
        call that asserts emptiness without one is the shape of a default pretending to be a
        measurement.
        """
        self._require(resource_id)
        if not evidence:
            raise SchemaRefused('REFUSED_MISSING_FIELD',
                                'clearing a resource requires evidence', field='evidence')
        if isinstance(evidence, str):
            raise SchemaRefused('REFUSED_BAD_TYPE', 'evidence must be a list', field='evidence')
        for index, ref in enumerate(evidence):
            S._evidence_ref(ref, f'evidence[{index}]')
        current = self.state[resource_id]
        # `UNKNOWN`, `FREE` and `CLEARING` are the only states a clear may be applied to, and
        # the reason is not the same mistake for each refusal. `OCCUPIED` is refused because
        # declaring empty something that is in use is a claim that something moved, made by the
        # caller, with no measurement behind it -- "you asked me to assert a fact I did not
        # observe", not "two writers disagreed". `RESERVED` is refused because somebody is
        # acting on that promise. `BLOCKED` is a deliberate human hold, which is not the
        # coordinator's to lift. All three get `REFUSED_RESOURCE_NOT_CLEARABLE` so a report can
        # count them, and each is named here so the reason survives the guard being one line.
        #
        # ★ This used to be two guards -- an explicit `OCCUPIED` branch plus this one. They
        # refuse the same set with the same reason, so the explicit branch was a subset of this
        # one and could not be told apart from it by any input: dead code wearing the shape of
        # an extra check. Removed rather than tested, because a check that cannot fail is not
        # a check.
        if current not in ('UNKNOWN', 'FREE', 'CLEARING'):
            self._refuse(resource_id, 'REFUSED_RESOURCE_NOT_CLEARABLE')
        self.state[resource_id] = 'FREE'
        # Clearing bumps the generation: any grant from before the clear is now history, and a
        # late reply carrying the old generation will be refused.
        self.generation[resource_id] += 1
        self.grant[resource_id] = None
        return self.snapshot(resource_id, now_s=now_s)

    def mark_blocked(self, resource_id, *, reason):
        """BLOCKED is a deliberate hold, not a lease. It has no owner and no expiry."""
        self._require(resource_id)
        if reason not in S.REASON_CODES:
            raise SchemaRefused('REFUSED_BAD_ENUM', repr(reason), field='reason')
        self.state[resource_id] = 'BLOCKED'
        self.generation[resource_id] += 1
        self.grant[resource_id] = None

    def initialise_occupied(self, resource_id, *, owner, ttl_s, now_s, evidence):
        """Measured loaded initial state; never claim a loaded source was cleared."""
        self._require(resource_id)
        owner=S._identifier(owner,'owner')
        ttl_s=S._number_in(ttl_s,'ttl_s',0.,S.MAX_DURATION_S)
        now_s=S._number_in(now_s,'now_s',0.,S.MAX_DURATION_S)
        if ttl_s<=0:raise SchemaRefused('REFUSED_OUT_OF_RANGE','positive ttl required')
        if not isinstance(evidence,list) or not evidence:
            raise SchemaRefused('REFUSED_MISSING_FIELD','occupied initialization requires evidence')
        for i,ref in enumerate(evidence):S._evidence_ref(ref,f'evidence[{i}]')
        if self.state[resource_id]!='UNKNOWN' or self.grant[resource_id] is not None:
            self._refuse(resource_id,'REFUSED_CONFLICT')
        self.generation[resource_id]+=1
        self.state[resource_id]='OCCUPIED'
        self.grant[resource_id]=Grant(resource_id,owner,self.generation[resource_id],
                                     self.epoch,ttl_s,now_s,'OCCUPIED')
        return self.snapshot(resource_id,now_s=now_s)

    def handover_occupied(self, resource_id, *, owner, generation, epoch,
                          new_owner, ttl_s, now_s, evidence):
        """Authorized atomic owner change, NEVER a FREE interval or new cargo pose.

        The caller supplies fresh physical launch/receiver evidence and the
        existing owner's grant token. Expired tokens cannot resurrect control.
        """
        self._require(resource_id)
        owner=S._identifier(owner,'owner');new_owner=S._identifier(new_owner,'new_owner')
        generation=S._int_in(generation,'generation',0,2**62)
        epoch=S._int_in(epoch,'epoch',0,2**62)
        ttl_s=S._number_in(ttl_s,'ttl_s',0.,S.MAX_DURATION_S)
        now_s=S._number_in(now_s,'now_s',0.,S.MAX_DURATION_S)
        if ttl_s<=0:raise SchemaRefused('REFUSED_OUT_OF_RANGE','positive ttl required')
        if not isinstance(evidence,list) or not evidence:
            raise SchemaRefused('REFUSED_MISSING_FIELD','handover requires physical evidence')
        for i,ref in enumerate(evidence):S._evidence_ref(ref,f'evidence[{i}]')
        held=self.grant[resource_id]
        if epoch!=self.epoch:self._refuse(resource_id,'REFUSED_STALE_EPOCH')
        if held is None:self._refuse(resource_id,'REFUSED_NO_LIVE_GRANT')
        if held.owner!=owner:self._refuse(resource_id,'REFUSED_GRANT_OWNED_BY_OTHER')
        if held.generation!=generation:self._refuse(resource_id,'REFUSED_STALE_GENERATION')
        if self.state[resource_id]!='OCCUPIED' or held.state!='OCCUPIED':
            self._refuse(resource_id,'REFUSED_RESOURCE_NOT_CLEARABLE')
        if now_s<held.granted_at_s or now_s>held.expires_at_s:
            self._refuse(resource_id,'REFUSED_GRANT_EXPIRED')
        if owner==new_owner:self._refuse(resource_id,'REFUSED_CONFLICT')
        self.generation[resource_id]+=1
        self.grant[resource_id]=Grant(resource_id,new_owner,self.generation[resource_id],
                                     self.epoch,ttl_s,now_s,'OCCUPIED')
        return self.snapshot(resource_id,now_s=now_s)

    # -- reservation -------------------------------------------------------

    def reserve(self, resource_id, *, owner, ttl_s, now_s, want_state='RESERVED'):
        """Atomically take a resource, or refuse with a reason that says why.

        Atomic means: the state check and the grant happen together, and the caller learns
        exactly which of the possible refusals it hit. All the reasons are distinct because
        "the reservation failed" is not actionable -- a test asserting `FAILED` cannot tell
        "someone else has it" from "nobody has cleared it yet", and the two have different
        fixes.
        """
        self._require(resource_id)
        owner = S._identifier(owner, 'owner')
        ttl_s = S._number_in(ttl_s, 'ttl_s', 0.0, S.MAX_DURATION_S)
        now_s = S._number_in(now_s, 'now_s', 0.0, S.MAX_DURATION_S)
        want_state = S._enum(want_state, 'want_state', frozenset({'RESERVED', 'OCCUPIED'}))
        if ttl_s <= 0.0:
            # A zero-length lease would be released by the next call to `expire`, which reads
            # as "the grant succeeded and then vanished".
            raise SchemaRefused('REFUSED_OUT_OF_RANGE', f'ttl_s={ttl_s}', field='ttl_s')

        # Expiry is evaluated before the state check, so a lapsed lease on this resource does
        # not block a new owner. Doing it after would make the resource look busy until some
        # unrelated call happened to sweep it.
        self._expire_one(resource_id, now_s)

        current = self.state[resource_id]
        if current == 'UNKNOWN':
            # ★ The most important refusal in the module: nobody has cleared it yet.
            self._refuse(resource_id, 'REFUSED_RESOURCE_UNKNOWN')
        if current == 'BLOCKED':
            self._refuse(resource_id, 'REFUSED_RESOURCE_BLOCKED')
        if current == 'OCCUPIED':
            self._refuse(resource_id, 'REFUSED_RESOURCE_OCCUPIED')
        if current == 'CLEARING':
            self._refuse(resource_id, 'REFUSED_RESOURCE_CLEARING')
        if current in ('RESERVED',) and self.grant[resource_id] is not None:
            held = self.grant[resource_id]
            if held.owner != owner:
                self._refuse(resource_id, 'REFUSED_RESOURCE_HELD_BY_OTHER')
            # Same owner re-asking for a live reservation is idempotent, not a conflict.
            return self.snapshot(resource_id, now_s=now_s)

        self.generation[resource_id] += 1
        self.state[resource_id] = want_state
        self.grant[resource_id] = Grant(resource_id, owner, self.generation[resource_id],
                                        self.epoch, ttl_s, now_s, want_state)
        return self.snapshot(resource_id, now_s=now_s)

    def renew(self, resource_id, *, owner, generation, epoch, ttl_s, now_s):
        """Extend a live grant -- and refuse anything that is not the live grant.

        ★ This is the "延迟响应不能重新计满 TTL" rule. Three ways a reply can be late or wrong,
        and each gets its own reason so a report can tell them apart:

          * wrong `epoch`   -- the reply predates a coordinator restart;
          * stale `generation` -- the reply predates a release and re-grant;
          * wrong `owner`   -- the reply is someone else's.

        A renewal restarts the TTL from `now_s`, never from the original `granted_at_s`: using
        the original would let a reply arriving just before expiry buy a full fresh window while
        reporting itself as a continuation.
        """
        self._require(resource_id)
        owner = S._identifier(owner, 'owner')
        generation = S._int_in(generation, 'generation', 0, 2 ** 62)
        epoch = S._int_in(epoch, 'epoch', 0, 2 ** 62)
        ttl_s = S._number_in(ttl_s, 'ttl_s', 0.0, S.MAX_DURATION_S)
        now_s = S._number_in(now_s, 'now_s', 0.0, S.MAX_DURATION_S)
        if ttl_s <= 0.0:
            raise SchemaRefused('REFUSED_OUT_OF_RANGE', f'ttl_s={ttl_s}', field='ttl_s')

        if epoch != self.epoch:
            self._refuse(resource_id, 'REFUSED_STALE_EPOCH')
        held = self.grant[resource_id]
        if held is None:
            self._refuse(resource_id, 'REFUSED_NO_LIVE_GRANT')
        # Identity is checked owner-first, then generation. Both are true of a reply from a
        # previous holder, and the two readings want different fixes: "this grant was never
        # yours" is a routing or a bug, "yours but older" is a duplicate delivery. So the order
        # is fixed here and asserted by the tests rather than left to whichever branch ran first.
        if held.owner != owner:
            self._refuse(resource_id, 'REFUSED_GRANT_OWNED_BY_OTHER')
        if generation != held.generation:
            self._refuse(resource_id, 'REFUSED_STALE_GENERATION')
        if now_s > held.expires_at_s:
            # A lapsed grant cannot be renewed back into existence: the resource may already
            # have been handed to someone else, and reviving it would silently steal from them.
            self._expire_one(resource_id, now_s)
            self._refuse(resource_id, 'REFUSED_GRANT_EXPIRED')

        held.ttl_s = ttl_s
        held.granted_at_s = now_s
        return self.snapshot(resource_id, now_s=now_s)

    # -- release -----------------------------------------------------------

    def release(self, resource_id, *, owner, generation, epoch, now_s=None,
                to_state='FREE'):
        """Give a resource back. Refuses a release from anyone but the holder.

        `to_state` is `FREE` for a reservation ending and `CLEARING` for an occupied resource
        being unloaded -- `CLEARING` exists precisely so "the container is being emptied" is not
        the same statement as "the container is empty".
        """
        self._require(resource_id)
        owner = S._identifier(owner, 'owner')
        generation = S._int_in(generation, 'generation', 0, 2 ** 62)
        epoch = S._int_in(epoch, 'epoch', 0, 2 ** 62)
        to_state = S._enum(to_state, 'to_state', frozenset({'FREE', 'CLEARING'}))
        held = self.grant[resource_id]
        if epoch != self.epoch:
            self._refuse(resource_id, 'REFUSED_STALE_EPOCH')
        if held is None:
            self._refuse(resource_id, 'REFUSED_NO_LIVE_GRANT')
        if held.owner != owner:
            self._refuse(resource_id, 'REFUSED_GRANT_OWNED_BY_OTHER')
        if generation != held.generation:
            self._refuse(resource_id, 'REFUSED_STALE_GENERATION')

        self.generation[resource_id] += 1
        self.state[resource_id] = to_state
        self.grant[resource_id] = None
        return self.snapshot(resource_id, now_s=now_s)

    # -- occupancy ---------------------------------------------------------

    def confirm_occupied(self, resource_id, *, owner, generation, epoch, now_s=None):
        """A reservation becomes occupancy: something is physically on the resource now.

        ★ The distinction is load-bearing. `RESERVED` is a promise; `OCCUPIED` is a fact. Only
        an `OCCUPIED` resource is protected from expiry, so a system that never promotes a
        reservation before the load arrives loses it to the clock.
        """
        self._require(resource_id)
        owner = S._identifier(owner, 'owner')
        generation = S._int_in(generation, 'generation', 0, 2 ** 62)
        epoch = S._int_in(epoch, 'epoch', 0, 2 ** 62)
        held = self.grant[resource_id]
        if epoch != self.epoch:
            self._refuse(resource_id, 'REFUSED_STALE_EPOCH')
        if held is None:
            self._refuse(resource_id, 'REFUSED_NO_LIVE_GRANT')
        if held.owner != owner:
            self._refuse(resource_id, 'REFUSED_GRANT_OWNED_BY_OTHER')
        if generation != held.generation:
            self._refuse(resource_id, 'REFUSED_STALE_GENERATION')
        if now_s is not None:
            now_s = S._number_in(now_s, 'now_s', 0.0, S.MAX_DURATION_S)
            if now_s > held.expires_at_s:
                self._expire_one(resource_id, now_s)
                self._refuse(resource_id, 'REFUSED_GRANT_EXPIRED')
        held.state = 'OCCUPIED'
        self.state[resource_id] = 'OCCUPIED'
        return self.snapshot(resource_id, now_s=now_s)

    # -- expiry ------------------------------------------------------------

    def _expire_one(self, resource_id, now_s):
        """Lapse this resource's grant if the clock says so, and only if it may.

        Returns True when something changed. The `OCCUPIED` guard is the "已占用时超时不释放"
        rule: a clock running out is not evidence that the thing on the resource moved.
        """
        held = self.grant[resource_id]
        if held is None:
            return False
        if now_s <= held.expires_at_s:
            return False
        if self.state[resource_id] == 'OCCUPIED':
            # Deliberately left in place and left granted. The caller can see it is overdue via
            # `overdue()`, but the resource is not handed to anyone else.
            return False
        self.state[resource_id] = 'FREE'
        self.grant[resource_id] = None
        self.generation[resource_id] += 1
        return True

    def expire(self, now_s):
        """Sweep every resource. Returns the ids that actually changed."""
        now_s = S._number_in(now_s, 'now_s', 0.0, S.MAX_DURATION_S)
        changed = [rid for rid in sorted(self.state) if self._expire_one(rid, now_s)]
        return changed

    def overdue(self, now_s):
        """Occupied resources whose lease has lapsed -- i.e. what expiry refused to release.

        Contract section 6 leaves these for a human: the loader is still on the resource and
        the coordinator's authority to move it has expired. Reporting them is the difference
        between "the lease system works" and "the lease system hides a stuck loader".
        """
        out = []
        for rid in sorted(self.state):
            held = self.grant[rid]
            if held is not None and self.state[rid] == 'OCCUPIED' and now_s > held.expires_at_s:
                out.append({'resource_id': rid, 'owner': held.owner,
                            'overdue_by_s': now_s - held.expires_at_s,
                            'generation': held.generation})
        return out

    # -- queries -----------------------------------------------------------

    def snapshot(self, resource_id, *, now_s=None):
        self._require(resource_id)
        held = self.grant[resource_id]
        return {
            'resource_id': resource_id,
            'state': self.state[resource_id],
            'generation': self.generation[resource_id],
            'epoch': self.epoch,
            'owner': held.owner if held else None,
            'expires_at_s': held.expires_at_s if held else None,
            'remaining_s': (max(0.0, held.expires_at_s - now_s)
                            if (held and now_s is not None) else None),
        }

    def table(self, *, now_s=None):
        return {rid: self.snapshot(rid, now_s=now_s) for rid in sorted(self.state)}

    def summary(self, *, now_s=None):
        by_state = {}
        for rid in self.state:
            by_state[self.state[rid]] = by_state.get(self.state[rid], 0) + 1
        return {
            'epoch': self.epoch,
            'resources': len(self.state),
            'by_state': dict(sorted(by_state.items())),
            'refusals': dict(sorted(self.refusal_count.items())),
            'overdue': self.overdue(now_s) if now_s is not None else [],
        }


#: Reasons this module refuses with. Declared so a report can count them apart from the
#: business-level reasons in `schema.REASON_CODES` -- the same split the validator uses.
RESOURCE_REFUSALS = frozenset({
    'REFUSED_RESOURCE_UNKNOWN', 'REFUSED_RESOURCE_BLOCKED', 'REFUSED_RESOURCE_OCCUPIED',
    'REFUSED_RESOURCE_CLEARING', 'REFUSED_RESOURCE_HELD_BY_OTHER',
    'REFUSED_RESOURCE_NOT_CLEARABLE', 'REFUSED_STALE_EPOCH',
    'REFUSED_STALE_GENERATION', 'REFUSED_NO_LIVE_GRANT', 'REFUSED_GRANT_OWNED_BY_OTHER',
    'REFUSED_GRANT_EXPIRED',
})

__all__ = ['ResourceTable', 'Grant', 'STATES', 'RESOURCE_REFUSALS']
