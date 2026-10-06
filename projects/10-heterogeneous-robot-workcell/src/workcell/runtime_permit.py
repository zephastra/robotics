"""Latched runtime permission, using monotonic wall time rather than sim time.

Observation/authorization producer is explicit and trusted by the integration.
This is not a sensor, a zone detector, or authentication. It rejects missing,
expired, contradictory evidence and never resumes just because a flag recovers.
"""
import math
import time


class RuntimePermit:
    def __init__(self, observe, *, owner, epoch, generation, max_age_s, clock=time.monotonic):
        if (not isinstance(owner, str) or not owner
                or type(epoch) is not int or epoch < 0
                or type(generation) is not int or generation < 0
                or isinstance(max_age_s, bool) or not math.isfinite(max_age_s) or max_age_s <= 0):
            raise ValueError('invalid runtime permit identity/age')
        self.observe, self.clock = observe, clock
        self.owner, self.epoch, self.generation = owner, epoch, generation
        self.max_age_s = float(max_age_s)
        self.latched = None
        self.last_sequence = -1
        self.last_wall_s = None
        self.checks = 0

    def stop(self, reason, detail):
        if self.latched is None:
            self.latched = {'allowed': False, 'reason_code': reason, 'detail': detail,
                            'generation': self.generation}
        return dict(self.latched)

    def _sample(self, generation, last_sequence):
        try:
            sample = self.observe()
            now = float(self.clock())
        except Exception as exc:
            return None, 'TRANSFER_UNKNOWN', 'observation unavailable: ' + type(exc).__name__
        if (not math.isfinite(now) or (self.last_wall_s is not None and now < self.last_wall_s)):
            return None, 'CLOCK_RESET', 'monotonic clock regressed'
        if not isinstance(sample, dict):
            return None, 'EVIDENCE_MISSING', 'no runtime observation'
        stamp = sample.get('observed_wall_s')
        if (isinstance(stamp, bool) or not isinstance(stamp, (int, float))
                or not math.isfinite(stamp) or stamp > now
                or now-stamp > self.max_age_s):
            return None, 'STALE_OBSERVATION', 'runtime evidence missing/expired/future'
        if (sample.get('owner') != self.owner or type(sample.get('epoch')) is not int
                or sample['epoch'] != self.epoch
                or type(sample.get('generation')) is not int or sample['generation'] != generation):
            return None, 'PERMIT_EXPIRED', 'authorization identity changed'
        sequence = sample.get('sequence')
        if type(sequence) is not int or sequence < 0 or sequence < last_sequence:
            return None, 'STALE_OBSERVATION', 'observation sequence regressed'
        if sample.get('cancel_requested') is True:
            return None, 'REQUEST_CONFLICT', 'cancel requested'
        if sample.get('cancel_requested') is not False:
            return None, 'EVIDENCE_MISSING', 'cancel state unknown'
        if sample.get('zone_clear') is not True:
            return None, 'RESOURCE_UNKNOWN', 'shared zone not proven clear'
        if sample.get('stop_chain_healthy') is not True:
            return None, 'CANCEL_UNCONFIRMED', 'stop chain not proven healthy'
        return (sample, now), None, None

    def check(self):
        self.checks += 1
        if self.latched is not None:
            return dict(self.latched)
        value, reason, detail = self._sample(self.generation, self.last_sequence)
        if reason:
            return self.stop(reason, detail)
        sample, now = value
        self.last_sequence, self.last_wall_s = sample['sequence'], now
        return {'allowed': True, 'reason_code': None, 'generation': self.generation}

    def rearm(self, *, generation, stopped_confirmed):
        """Explicit fresh authorization after a measured stop; failure leaves latch intact."""
        if stopped_confirmed is not True:
            raise ValueError('a measured stop is required before recovery')
        if type(generation) is not int or generation <= self.generation:
            raise ValueError('recovery requires a newer authorization generation')
        value, reason, detail = self._sample(generation, -1)
        if reason:
            raise ValueError(reason + ': ' + detail)
        sample, now = value
        self.generation, self.latched = generation, None
        self.last_sequence, self.last_wall_s = sample['sequence'], now
        return self.check()
