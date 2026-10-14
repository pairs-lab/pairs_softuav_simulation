"""Illustrative transient feedback degradation, NOT a fitted hardware model.

Hover-disturbance footage motivates the qualitative response only; it does
not identify a wall-impact speed curve or an IMU recovery duration.
"""
from collections import deque

from rigid_offset_behavior_sweep import OffsetRigidTrial


class RigidRecoveryTrial(OffsetRigidTrial):
    def __init__(self, speed, cfg, transient_delay_s=.05, transient_duration_s=2.8):
        super().__init__(speed,cfg)
        self.transient_delay_s=transient_delay_s
        self.transient_duration_s=transient_duration_s
        self.active_delay_s=.012

    def controller(self, settling=False):
        elapsed=self.data.time-self.impact_time if self.impact_time is not None else -1.
        delay=self.transient_delay_s if 0<=elapsed<self.transient_duration_s else .012
        steps=round(delay/self.model.opt.timestep)
        if steps!=self.delay_steps:
            self.delay_steps=steps
            self.observations=deque(self.observations,maxlen=steps+1)
        self.active_delay_s=delay
        super().controller(settling)

    def summary(self):
        result=super().summary()
        result.update(assumed_post_contact_delay_s=self.transient_delay_s,
                      assumed_post_contact_delay_duration_s=self.transient_duration_s,
                      behavior_profile='illustrative transient feedback degradation',
                      reference_validation='qualitative hover disturbance only',
                      controller_matched_to_soft=False)
        return result
