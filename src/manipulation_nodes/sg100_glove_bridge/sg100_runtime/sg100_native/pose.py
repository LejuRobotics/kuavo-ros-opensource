"""Continuous five-finger tracking for electromagnetic serial measurements."""

import math

import numpy as np

# Five finger channels, expressed as indices in thumb-to-pinky order.
_WIRE_ORDER = [3, 4, 2, 0, 1]


class InvalidPoseError(ValueError):
    """A serial measurement cannot be used as a hand pose."""


class PoseTracker:
    """Choose continuous position branches before passing poses to the solver."""

    def __init__(self):
        self.reset()

    def reset(self):
        self._positions = np.zeros((5, 3))
        self._velocities = np.zeros((5, 3))
        self._hold_count = np.zeros(5, dtype=int)
        self._reach_count = np.zeros(5, dtype=int)
        self._reach_cooldown = np.zeros(5, dtype=int)
        self._frames = 0
        self._previous_time = None

    def process(self, position, orientation, now):
        raw = np.asarray(position, dtype=float).reshape(5, 3)[_WIRE_ORDER]
        quat = np.asarray(orientation, dtype=float).reshape(5, 4)[_WIRE_ORDER]
        with np.errstate(over="ignore", invalid="ignore"):
            norm_squared = np.sum(quat * quat, axis=1)
        if not np.isfinite(raw).all() or not np.isfinite(quat).all():
            raise InvalidPoseError("pose must contain finite values")
        if not np.isfinite(norm_squared).all() or np.any(norm_squared < 1e-20):
            raise InvalidPoseError("pose quaternions must have a finite, non-zero norm")
        now = float(now)
        if not math.isfinite(now):
            raise InvalidPoseError("pose timestamp must be finite")
        dt = None if self._previous_time is None else now - self._previous_time
        if dt is None or not math.isfinite(dt) or dt <= 0:
            dt = 1 / 60
        else:
            dt = min(max(dt, 1 / 240), 0.1)
        self._previous_time = now
        self._frames += 1
        positions = np.zeros((5, 3))
        quaternions = np.zeros((5, 4))
        held = np.zeros(5, dtype=bool)
        reacquired = np.zeros(5, dtype=bool)
        recovered = np.zeros(5, dtype=bool)

        for i in range(5):
            sample = raw[i]
            if self._frames <= 3:
                positive_score = int(sample[0] >= 0) + int(sample[2] <= 0)
                negative_score = int(sample[0] <= 0) + int(sample[2] >= 0)
                selected = sample.copy() if positive_score >= negative_score else -sample
            else:
                velocity = self._velocities[i].copy()
                speed = np.linalg.norm(velocity)
                if speed > 2:
                    velocity *= 2 / speed
                predicted = self._positions[i] + velocity * dt
                selected = (
                    sample.copy()
                    if np.sum((sample - predicted) ** 2) <= np.sum((-sample - predicted) ** 2)
                    else -sample
                )
                speed = np.linalg.norm(selected - self._positions[i]) / dt
                radius = np.linalg.norm(selected)
                if speed > 6 or radius > 1:
                    self._hold_count[i] += 1
                    if self._hold_count[i] <= 5:
                        selected = predicted
                        held[i] = True
                    else:
                        if radius > 1:
                            selected = selected / max(radius, 1e-12)
                        reacquired[i] = True
                        self._hold_count[i] = 0
                else:
                    self._hold_count[i] = 0
            positions[i] = selected
            sign = -1 if np.dot(selected, sample) < 0 else 1
            quaternions[i] = sign * quat[i]

        if self._frames > 3:
            snapshot = positions.copy()
            for i in range(5):
                if self._reach_cooldown[i] > 0:
                    self._reach_cooldown[i] -= 1
                others = [j for j in range(5) if j != i and not held[j] and not reacquired[j]]
                violation = False
                if not held[i] and not reacquired[i] and others:
                    center = snapshot[others].mean(axis=0)
                    distance = np.sum((snapshot[i] - center) ** 2)
                    opposite_distance = np.sum((-snapshot[i] - center) ** 2)
                    violation = distance > 0.06**2 and opposite_distance < distance * 0.6**2
                self._reach_count[i] = self._reach_count[i] + 1 if violation else 0
                if self._reach_count[i] >= 3 and self._reach_cooldown[i] == 0:
                    positions[i] = -snapshot[i]
                    quaternions[i] = -quaternions[i]
                    recovered[i] = True
                    self._reach_count[i] = 0
                    self._reach_cooldown[i] = 8

        for i in range(5):
            if reacquired[i] or recovered[i]:
                self._velocities[i] = 0
            elif held[i]:
                self._velocities[i] *= 0.5
            elif self._frames > 1 and np.any(self._positions[i] != 0):
                measured = (positions[i] - self._positions[i]) / dt
                velocity = 0.7 * self._velocities[i] + 0.3 * measured
                speed = np.linalg.norm(velocity)
                if speed > 2:
                    velocity *= 2 / speed
                self._velocities[i] = velocity
            self._positions[i] = positions[i]

        quaternions /= np.sqrt(norm_squared)[:, None]
        return positions[_WIRE_ORDER].reshape(-1), quaternions[_WIRE_ORDER].reshape(-1)
