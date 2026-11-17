"""Per-hand clutch in neutral chest axes. Positions in metres, spatial rates in SI."""
import numpy as np
from .pose_velocity_kalman import exp_so3, log_so3, left_jacobian


class HandClutch:
    def __init__(self, translation_scale, rotation_scale, incremental_rotation, radius, quiet_distance=.002,
                 observation_window=.1, quiet_angle=.003):
        self.scale = np.asarray(translation_scale, dtype=float)
        self.rotation_scale = np.asarray(rotation_scale, dtype=float)
        self.incremental_rotation = incremental_rotation
        self.radius = float(radius)
        self.window = observation_window
        self.quiet_distance = quiet_distance
        self.last_stamp = None
        self.linear_quiet_speed = quiet_distance / observation_window
        self.angular_quiet_speed = quiet_angle / observation_window
        if (not np.isfinite(self.scale).all() or not np.isfinite(self.rotation_scale).all()
                or not np.isfinite(self.radius) or self.radius <= 0):
            raise ValueError("invalid incremental mapping parameters")
        self.key = None
        self.edge = None
        self.elbow = None
        self.frozen_base = None
        self.frozen_chest = None
        self.command_hold_key = None
        self.paused = False
        self.pause_chest = None
        self.source_rotation = None
        from .incremental_controls import RotationTransfer
        self.transfer = RotationTransfer()

    def sample(self, key, edge, engaged, hand, rotation, velocity, omega, measured, shoulder, stamp=None):
        if key != self.key:
            self.key = key
            self.position, self.rotation = (v.copy() for v in measured)
            self.edge = None
        gap = (stamp is not None and self.last_stamp is not None and
               (stamp < self.last_stamp or stamp - self.last_stamp > self.window))
        self.last_stamp = stamp
        if self.edge != edge or not engaged or gap:
            self.quiet_origin = hand.copy()
            self.quiet_since = stamp
            self.previous = hand.copy()
            self.hand_zero = rotation.copy()
            self.robot_zero = self.rotation.copy()
            self.edge = edge
            return self.position, self.rotation, np.zeros(3), np.zeros(3)

        # Integrate only the admitted increment. A saturated displacement
        # cannot accumulate outside the workspace during a sustained push.
        proposed = self.position + self.scale * (hand - self.previous)
        self.previous = hand.copy()
        radius_vector = proposed - shoulder
        length = np.linalg.norm(radius_vector)
        linear = self.scale * velocity
        if length >= self.radius:
            outward = radius_vector / length
            proposed = shoulder + self.radius * outward
            linear = linear - outward * max(0., linear @ outward)
        self.position = proposed
        if self.incremental_rotation:
            if np.all(self.rotation_scale == 1.):
                self.rotation = rotation @ self.hand_zero.T @ self.robot_zero
            else:
                delta = log_so3(rotation @ self.hand_zero.T)
                scaled = self.rotation_scale * delta
                omega = left_jacobian(scaled) @ (
                    self.rotation_scale * np.linalg.solve(left_jacobian(delta), omega))
                self.rotation = exp_so3(scaled) @ self.robot_zero
        else:
            self.rotation = rotation.copy()
        # !4170: output admission must never overwrite the human source state.
        # robot_zero/hand_zero stay fixed until the next explicit clutch edge.
        self.source_rotation = self.rotation.copy()
        # The full-body CV still owns pose and velocity estimation. Across one
        # observer window, motion smaller than the legacy arm movement threshold
        # carries no intentional hand velocity. Only gate the emitted twist.
        linear_speed = np.linalg.norm(velocity)
        angular_speed = np.linalg.norm(omega)
        linear *= self.quiet_gain(linear_speed, self.linear_quiet_speed)
        if stamp is not None:
            if np.linalg.norm(hand - self.quiet_origin) > self.quiet_distance:
                self.quiet_origin = hand.copy()
                self.quiet_since = stamp
            elif stamp - self.quiet_since >= self.window and linear_speed < 2. * self.linear_quiet_speed:
                linear.fill(0.)
        omega = omega * self.quiet_gain(angular_speed, self.angular_quiet_speed)
        return self.position, self.rotation, linear, omega

    @staticmethod
    def quiet_gain(speed, threshold):
        if threshold <= 0. or speed >= 2. * threshold:
            return 1.
        ratio = max(0., speed / threshold - 1.)
        return ratio * ratio * (3. - 2. * ratio)
