#!/usr/bin/env python3
"""Absolute-only hand activity estimator; inherit beta head lifecycle unchanged."""
import legacy_dependencies
import math
import rospy
import head_control_node as shared_head
from head_control_node import _MAX_HAND_SPEED_M_S

class _HandActivity:
    """一只手的时间归一化速度与活动量（m/s），抑制静止噪声。"""

    def __init__(self):
        self.last_pos = None
        self.last_t = None
        self.speed = 0.0
        self.activity = 0.0

    def update(self, pos, now, hold_sec):
        if pos is None:
            self.last_pos = None
            self.last_t = None
            self.speed = 0.0
            self.activity = 0.0
            return 0.0
        if self.last_pos is None or self.last_t is None:
            self.last_pos = list(pos)
            self.last_t = now
            return self.activity
        dt = now - self.last_t
        if dt <= 0.0:
            return self.activity
        if dt > 0.25:
            self.last_pos, self.last_t = list(pos), now
            self.speed = self.activity = 0.0
            return 0.0
        dist = math.sqrt(sum((a - b) * (a - b) for a, b in zip(pos, self.last_pos)))
        inst = dist / dt
        # Reject TF jumps rather than converting them into maximum activity.
        if not math.isfinite(inst) or inst > _MAX_HAND_SPEED_M_S:
            inst = 0.0
        # Ignore feedback jitter below 1 cm/s. Both filters use wall-time
        # constants so a 25/50/100 Hz tracker has the same activity units (m/s).
        inst = max(0.0, inst - 0.01)
        alpha = 1.0 - math.exp(-dt / 0.06)
        self.speed += alpha * (inst - self.speed)
        alpha = 1.0 - math.exp(-dt / hold_sec)
        self.activity += alpha * (self.speed - self.activity)
        self.last_pos = list(pos)
        self.last_t = now
        return self.activity

class HeadTrackHandNode(shared_head.HeadTrackHandNode):
    def __init__(self):
        super().__init__()
        self._left_activity = _HandActivity()
        self._right_activity = _HandActivity()
        self._bias_right_weight = self.hand_bias
        self._bias_last_time = self._bias_last_motion = self._bias_motion_since = None

    def _velocity_biased_point(self, left_pos, right_pos):
        """中点按双手活动量偏移。活动量是平滑速度的泄漏积分，动得勤的手不会马上被忘掉。"""
        baseline = self.hand_bias
        midpoint = [(1.0-baseline)*left + baseline*right
                    for left, right in zip(left_pos, right_pos)]
        if self.velocity_bias <= 0.0 or baseline in (0.0, 1.0):
            return midpoint
        now = rospy.Time.now().to_sec()
        left_activity = self._left_activity.update(left_pos, now, self.velocity_bias_hold_sec)
        right_activity = self._right_activity.update(right_pos, now, self.velocity_bias_hold_sec)
        total = left_activity + right_activity
        dt = 0.0 if self._bias_last_time is None else max(0.0, min(0.1, now-self._bias_last_time))
        self._bias_last_time = now
        moving = max(self._left_activity.speed, self._right_activity.speed) > 0.003
        if moving:
            if self._bias_motion_since is None:
                self._bias_motion_since = now
        else:
            self._bias_motion_since = None
        confirmed = self._bias_motion_since is not None and now-self._bias_motion_since >= 0.25
        if confirmed:
            self._bias_last_motion = now
        if confirmed and total > 1.0e-6:
            # Once sustained motion is confirmed, use relative activity rather
            # than attenuating slow movements by an absolute speed threshold.
            contrast = (right_activity-left_activity) / total
            contrast = math.copysign(max(0.0, abs(contrast)-0.15)/0.85, contrast)
            # Symmetric excursion around the configured baseline. Endpoint
            # settings remain fixed on one hand, even during opposite motion.
            desired = baseline + min(baseline, 1.0-baseline)*self.velocity_bias*contrast
        elif self._bias_last_motion is not None and now-self._bias_last_motion < 1.5:
            desired = self._bias_right_weight
        else:
            desired = baseline
        alpha = 1.0-math.exp(-dt/0.2)
        self._bias_right_weight += alpha*(desired-self._bias_right_weight)
        w = self._bias_right_weight
        return [(1.0-w)*left+w*right for left, right in zip(left_pos, right_pos)]

if __name__ == "__main__":
    try:
        HeadTrackHandNode().spin()
    except rospy.ROSInterruptException:
        pass
