"""Wheel incremental joystick mapping for the absolute-mode input supervisor.

Matches WheelJoyStickHandler and publishAuxiliaryStates: left Y / -left X /
-right X, model JSON deadzone and alpha, right-Y torso gesture suppression.
"""
import math


class WheelBaseCommandMapper:
    def __init__(self, config=None, limits=(0.4, 0.4, 0.25)):
        config = config or {}
        self.threshold = float(config.get("threshold", 0.05))
        self.alpha = float(config.get("alpha", 0.8))
        self.limits = tuple(float(x) for x in limits)
        if not (math.isfinite(self.threshold) and 0 <= self.threshold < 1
                and math.isfinite(self.alpha) and 0 <= self.alpha < 1
                and len(self.limits) == 3
                and all(math.isfinite(x) and x > 0 for x in self.limits)):
            raise ValueError("Invalid wheel joystick filter or speed limits")
        self.reset()

    def reset(self):
        self.filtered = [0.0, 0.0, 0.0]
        self.received = None

    def sample(self, msg, active, now):
        values = (msg.left_y, -msg.left_x, -msg.right_x)
        if (not active or not math.isfinite(now)
                or not all(math.isfinite(x) for x in (*values, msg.right_y))):
            self.reset()
            return (0.0, 0.0, 0.0)
        if self.received is not None and not 0 <= now - self.received < 0.2:
            self.reset()
        self.received = now
        if abs(msg.right_y) > 0.8:
            self.filtered = [0.0, 0.0, 0.0]
        else:
            self.filtered = [self.alpha * old + (1 - self.alpha) * value
                             if abs(value) > self.threshold else 0.0
                             for old, value in zip(self.filtered, values)]
        return tuple(max(-1.0, min(1.0, value)) * limit
                     for value, limit in zip(self.filtered, self.limits))
