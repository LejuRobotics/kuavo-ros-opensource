"""Admission guard for Quest optical skeleton frames before pose smoothing.

The limits are deliberately generous candidates for this capture, not human
motion limits. Fatal faults latch for the lifetime of this object. With
latch_on_jump=False, isolated motion jumps are rejected without latching.
"""

import math


_POSITION_BONES = (0, 1, 2, 3, 4, 5, 23)
_ORIENTATION_BONES = (4, 5, 23)


class OpticalInputGuard:
    def __init__(self, enabled=True, max_linear_speed=6.0,
                 position_slack=0.020, max_angular_speed=50.0,
                 orientation_slack=0.15, max_gap=0.10,
                 latch_on_jump=True):
        limits = {
            'max_linear_speed': max_linear_speed,
            'position_slack': position_slack,
            'max_angular_speed': max_angular_speed,
            'orientation_slack': orientation_slack,
            'max_gap': max_gap,
        }
        for name, value in limits.items():
            if not isinstance(value, (int, float)) or not math.isfinite(value) or value < 0:
                raise ValueError('{} must be a finite, nonnegative number'.format(name))
        if max_gap == 0:
            raise ValueError('max_gap must be positive')
        self.enabled = bool(enabled)
        self.latch_on_jump = bool(latch_on_jump)
        self.max_linear_speed = max_linear_speed
        self.position_slack = position_slack
        self.max_angular_speed = max_angular_speed
        self.orientation_slack = orientation_slack
        self.max_gap = max_gap
        self.latched = False
        self.reason = ''
        self.details = {}
        self.previous_stamp = None
        self._previous_poses = None
        self._received_stamp = None
        self._candidate_poses = None
        self._candidate_count = 0
        self._recovering = False
        self._raw_poses = None
        self.output_poses = None

    def _latch(self, reason, **details):
        self.latched = True
        self.reason = reason
        self.details = details
        return False

    def _reject_jump(self, reason, **details):
        if self.latch_on_jump:
            return self._latch(reason, **details)
        self.reason = reason
        self.details = details
        return False

    def _discontinuity(self, current, previous, dt):
        for index in _POSITION_BONES:
            position, quaternion = current[index]
            old_position, old_quaternion = previous[index]
            step = math.sqrt(sum((a - b) ** 2 for a, b in zip(position, old_position)))
            limit = self.position_slack + self.max_linear_speed * dt
            if step > limit:
                return 'position_jump', dict(bone=index, step=step, limit=limit, dt=dt)
            if index in _ORIENTATION_BONES:
                dot = abs(sum(a * b for a, b in zip(quaternion, old_quaternion)))
                angle = 2 * math.acos(min(1.0, max(0.0, dot)))
                limit = self.orientation_slack + self.max_angular_speed * dt
                if angle > limit:
                    return 'orientation_jump', dict(bone=index, angle=angle,
                                                    limit=limit, dt=dt)
        return '', {}

    def _blend(self, target, dt):
        """Advance every guarded bone from the last emitted pose, without a step."""
        result = {}
        # These rates are intentionally below the jump detector's permissive
        # limits. The downstream hand smoother still handles ordinary noise.
        position_limit = min(self.max_linear_speed, 0.8) * dt
        angular_limit = min(self.max_angular_speed, 3.0) * dt
        fraction = 1.0
        for index in _POSITION_BONES:
            position, quaternion = target[index]
            previous_position, previous_quaternion = self._previous_poses[index]
            distance = math.sqrt(sum((a - b) ** 2 for a, b in
                                     zip(position, previous_position)))
            if distance:
                fraction = min(fraction, position_limit / distance)
            dot = abs(sum(a * b for a, b in zip(quaternion, previous_quaternion)))
            angle = 2 * math.acos(min(1.0, max(0.0, dot)))
            if angle:
                fraction = min(fraction, angular_limit / angle)
        # Use the same fraction for every bone to avoid distorting the hand
        # skeleton when the optical coordinate frame itself jumps.
        for index in _POSITION_BONES:
            position, quaternion = target[index]
            previous_position, previous_quaternion = self._previous_poses[index]
            delta = tuple(a - b for a, b in zip(position, previous_position))
            output_position = tuple(b + fraction * value for b, value in
                                    zip(previous_position, delta))
            dot = sum(a * b for a, b in zip(quaternion, previous_quaternion))
            if dot < 0:
                quaternion = tuple(-value for value in quaternion)
                dot = -dot
            if dot > 1.0 - 1e-8:
                blended = tuple((1.0 - fraction) * b + fraction * a for a, b in
                                zip(quaternion, previous_quaternion))
                norm = math.sqrt(sum(value * value for value in blended))
                output_quaternion = tuple(value / norm for value in blended)
            else:
                half_angle = math.acos(dot)
                denominator = math.sin(half_angle)
                old_weight = math.sin((1.0 - fraction) * half_angle) / denominator
                new_weight = math.sin(fraction * half_angle) / denominator
                output_quaternion = tuple(old_weight * b + new_weight * a for a, b in
                                          zip(quaternion, previous_quaternion))
            result[index] = output_position, output_quaternion
        return result

    @staticmethod
    def _snapshot(poses):
        try:
            snapshot = {}
            for index in _POSITION_BONES:
                values = poses[index]
                if len(values) != 7:
                    return None, 'invalid_pose', index
                position = tuple(float(value) for value in values[:3])
                if not all(math.isfinite(value) for value in position):
                    return None, 'invalid_pose', index
                quaternion = tuple(float(value) for value in values[3:])
                if not all(math.isfinite(value) for value in quaternion):
                    return None, 'invalid_pose', index
                # Match the mapper's raw quaternion validity check before
                # normalization can turn an invalid sample into a unit pose.
                norm2 = sum(value * value for value in quaternion)
                if not math.isfinite(norm2) or norm2 <= 0.25:
                    return None, 'invalid_pose', index
                magnitude = math.sqrt(norm2)
                quaternion = tuple(value / magnitude for value in quaternion)
                snapshot[index] = (position, quaternion)
        except (KeyError, IndexError):
            return None, 'missing_bone', index
        except (TypeError, ValueError, OverflowError):
            return None, 'invalid_pose', index
        return snapshot, '', None

    def update(self, stamp_seconds, poses, armed=False):
        """Return whether the frame can enter the mapper; never modify poses.

        ``poses`` maps bone index to (x, y, z, qx, qy, qz, qw).
        Duplicate or old timestamps are ignored without poisoning the baseline.
        ``armed=False`` accepts valid frames as a fresh baseline without motion
        checks; once armed, every accepted frame is compared to that baseline.
        """
        if not self.enabled:
            self.output_poses = None
            return True
        if self.latched:
            return False
        try:
            stamp = float(stamp_seconds)
        except (TypeError, ValueError, OverflowError):
            return self._latch('invalid_stamp') if armed else False
        if not math.isfinite(stamp):
            return self._latch('invalid_stamp') if armed else False
        if self.previous_stamp is not None and stamp <= self.previous_stamp:
            return False
        if self._received_stamp is not None and stamp <= self._received_stamp:
            return False
        snapshot, error, index = self._snapshot(poses)
        if error:
            return self._latch(error, bone=index) if armed else False
        if armed and self._received_stamp is not None:
            dt = stamp - self._received_stamp
            if dt > self.max_gap:
                return self._latch('source_gap', dt=dt, limit=self.max_gap)
        else:
            dt = 0.0
        if armed and self._previous_poses is not None and not self.latch_on_jump:
            baseline = self._candidate_poses
            if baseline is None:
                baseline = self._raw_poses if self._recovering else self._previous_poses
            reason, details = self._discontinuity(snapshot, baseline, dt)
            if reason and self._candidate_poses is not None:
                # A lone optical spike followed by the original track should
                # resume immediately, without treating the return as a jump.
                original_reason, _ = self._discontinuity(
                    snapshot, self._raw_poses if self._recovering else self._previous_poses,
                    dt)
                if not original_reason:
                    self._candidate_poses = None
                    self._candidate_count = 0
                    reason = ''
            if reason:
                self._received_stamp = stamp
                self._candidate_poses = snapshot
                self._candidate_count = 1
                return self._reject_jump(reason, **details)
            if self._candidate_poses is not None:
                self._candidate_count += 1
                self._candidate_poses = snapshot
                self._received_stamp = stamp
                if self._candidate_count < 3:
                    return False
                self._recovering = True
                self._candidate_poses = None
            if self._recovering:
                self._raw_poses = snapshot
                # Quarantined frames do not accumulate movement allowance.
                # dt is the interval since the previous received frame.
                emitted = self._blend(snapshot, dt)
                # Continue blending until every guarded bone reaches raw input.
                self._recovering = any(
                    any(abs(a - b) > 1e-8 for a, b in zip(emitted[i][0], snapshot[i][0]))
                    or abs(sum(a * b for a, b in zip(emitted[i][1], snapshot[i][1]))) < 1 - 1e-8
                    for i in _POSITION_BONES)
                snapshot = emitted
            else:
                self._raw_poses = snapshot
        elif armed and self._previous_poses is not None:
            reason, details = self._discontinuity(snapshot, self._previous_poses,
                                                   stamp - self.previous_stamp)
            if reason:
                return self._reject_jump(reason, **details)
        self.previous_stamp = stamp
        self._received_stamp = stamp
        self._previous_poses = snapshot
        self._candidate_poses = None
        self._candidate_count = 0
        if not armed:
            self._recovering = False
            self._raw_poses = snapshot
        self.output_poses = snapshot if self._recovering or not self.latch_on_jump else None
        self.reason = ''
        self.details = {}
        return True
