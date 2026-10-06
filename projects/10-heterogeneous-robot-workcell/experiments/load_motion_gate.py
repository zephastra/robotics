"""Fresh load evidence permits motion, never disables an available brake.

A refusal while motion is requested is latched. New images cannot silently
resume the interrupted task. This class neither writes actuators nor certifies
physical stopping: the final writer must issue the brake and judge the result.
"""
from loaded_nav_envelope import observed_load_gate


class LoadMotionGate:
    def __init__(self, contract):
        self.contract = contract
        self.latched_reason = None

    def check(self, observation, *, now_sim_s, now_wall_s, motion_requested):
        sensed = observed_load_gate(observation, self.contract,
                                    now_sim_s=now_sim_s, now_wall_s=now_wall_s)
        if motion_requested and not sensed['allowed'] and self.latched_reason is None:
            self.latched_reason = sensed['reason_code']
        allowed = bool(motion_requested and sensed['allowed'] and self.latched_reason is None)
        return dict(motion_allowed=allowed, brake_required=not allowed,
                    latched_reason=self.latched_reason, observed_load=sensed)
