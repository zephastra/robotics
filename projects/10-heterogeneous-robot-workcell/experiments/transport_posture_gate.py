"""Latch transport motion refusal without disabling the existing wheel brake.

No physics writes, threshold changes, or automatic recovery. The caller must
measure the actual stop and retain cargo/resource ownership on refusal.
"""
import copy


class TransportPostureGate:
    def __init__(self):
        self.latched_reason = None
        self.first_failure = None

    def check(self, posture):
        if self.latched_reason is not None:
            return False
        # Unknown/malformed observations cannot authorize motion.
        valid = (isinstance(posture, dict) and posture.get('allowed') is True
                 and posture.get('reason_code') is None
                 and posture.get('failures') == [])
        if not valid:
            self.latched_reason = (posture.get('reason_code') if isinstance(posture, dict) else None) or 'TRANSPORT_POSTURE_UNKNOWN'
            self.first_failure = copy.deepcopy(posture)
        return valid
