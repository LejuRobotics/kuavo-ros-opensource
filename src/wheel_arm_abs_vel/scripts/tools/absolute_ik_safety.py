#!/usr/bin/env python3

"""Pure safety primitives for the legacy absolute position-IK executor."""

from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class HardCandidateDecision:
    accepted: bool
    reason: str
    hold_active: bool
    recovery_success_count: int
    max_trusted_offset: float
    min_joint_limit_margin: float


class AbsoluteIkExecutionSafety:
    """Reject unsafe hard-IK candidates and rate-limit accepted commands.

    Hard-IK target continuity and executed-command rate limiting use separate
    states.  Candidate branches are compared with the previous accepted hard
    target even while the rate-limited command is still catching up.  During a
    fault, recovery candidates are compared with that frozen target rather
    than with rejected candidates, so bounded steps cannot drift to a branch.
    """

    def __init__(
            self, dof, recovery_successes, target_max_step,
            recovery_max_offset,
            recovery_stability_step, max_velocity, max_acceleration,
            joint_lower=None, joint_upper=None, min_joint_limit_margin=0.0):
        self.dof = int(dof)
        self.recovery_successes = max(1, int(recovery_successes))
        self.target_max_step = max(0.0, float(target_max_step))
        self.recovery_max_offset = max(0.0, float(recovery_max_offset))
        self.recovery_stability_step = max(
            0.0, float(recovery_stability_step)
        )
        self.max_velocity = max(0.0, float(max_velocity))
        self.max_acceleration = max(0.0, float(max_acceleration))
        self.min_joint_limit_margin = max(
            0.0, float(min_joint_limit_margin)
        )
        self.joint_lower = self._optional_vector(joint_lower)
        self.joint_upper = self._optional_vector(joint_upper)
        if (self.joint_lower is None) != (self.joint_upper is None):
            raise ValueError("joint lower and upper limits must be supplied together")
        if (self.joint_lower is not None
                and np.any(self.joint_lower >= self.joint_upper)):
            raise ValueError("joint lower limits must be below upper limits")

        self.hold_active = False
        self.recovery_success_count = 0
        self._hold_reference = None
        self._trusted_target = None
        self._previous_recovery_candidate = None
        self._command = None
        self._velocity = np.zeros(self.dof, dtype=float)

    def _optional_vector(self, value):
        if value is None:
            return None
        vector = np.asarray(value, dtype=float).reshape(-1)
        if vector.size != self.dof or not np.all(np.isfinite(vector)):
            raise ValueError("joint vector must be finite and match dof")
        return vector.copy()

    def _vector(self, value):
        vector = np.asarray(value, dtype=float).reshape(-1)
        if vector.size != self.dof or not np.all(np.isfinite(vector)):
            return None
        return vector.copy()

    def reset(self, command):
        command = self._vector(command)
        if command is None:
            raise ValueError("reset command must be finite and match dof")
        self.hold_active = False
        self.recovery_success_count = 0
        self._hold_reference = command.copy()
        self._trusted_target = None
        self._previous_recovery_candidate = None
        self._command = command.copy()
        self._velocity = np.zeros(self.dof, dtype=float)

    @property
    def command(self):
        return None if self._command is None else self._command.copy()

    @property
    def hold_reference(self):
        return (
            None if self._hold_reference is None
            else self._hold_reference.copy()
        )

    @property
    def trusted_target(self):
        return (
            None if self._trusted_target is None
            else self._trusted_target.copy()
        )

    def enter_hold(self, last_safe_command=None):
        reference = self._vector(last_safe_command)
        if reference is None:
            reference = self.command
        if reference is None:
            raise ValueError("hold requires a valid last safe command")
        if not self.hold_active:
            self._hold_reference = reference.copy()
        self.hold_active = True
        self.recovery_success_count = 0
        self._previous_recovery_candidate = None
        self._command = self._hold_reference.copy()
        self._velocity.fill(0.0)
        return self._command.copy()

    def _joint_limit_margin(self, candidate):
        if self.joint_lower is None:
            return float("inf")
        return float(np.min(np.minimum(
            candidate - self.joint_lower,
            self.joint_upper - candidate,
        )))

    def observe_hard_candidate(self, candidate, last_safe_command=None):
        candidate = self._vector(candidate)
        if candidate is None:
            self.enter_hold(last_safe_command)
            return HardCandidateDecision(
                False, "invalid_candidate", True, 0, float("inf"),
                float("-inf"),
            )

        min_margin = self._joint_limit_margin(candidate)
        if min_margin < self.min_joint_limit_margin:
            self.enter_hold(last_safe_command)
            return HardCandidateDecision(
                False, "joint_limit_margin", True, 0, 0.0, min_margin,
            )

        if not self.hold_active:
            if self._trusted_target is None:
                self._trusted_target = candidate.copy()
                return HardCandidateDecision(
                    True, "acquired", False, 0, 0.0, min_margin,
                )
            target_step = float(np.max(np.abs(
                candidate - self._trusted_target
            )))
            if target_step > self.target_max_step:
                self.enter_hold(last_safe_command)
                return HardCandidateDecision(
                    False, "target_discontinuity", True, 0, target_step,
                    min_margin,
                )
            self._trusted_target = candidate.copy()
            return HardCandidateDecision(
                True, "tracking", False, 0, target_step, min_margin,
            )

        recovery_reference = (
            self._trusted_target
            if self._trusted_target is not None
            else self._hold_reference
        )
        trusted_offset = float(np.max(np.abs(
            candidate - recovery_reference
        )))
        if trusted_offset > self.recovery_max_offset:
            self.recovery_success_count = 0
            self._previous_recovery_candidate = candidate
            return HardCandidateDecision(
                False, "recovery_offset", True, 0, trusted_offset,
                min_margin,
            )

        stable = True
        if self._previous_recovery_candidate is not None:
            candidate_step = float(np.max(np.abs(
                candidate - self._previous_recovery_candidate
            )))
            stable = candidate_step <= self.recovery_stability_step
        self._previous_recovery_candidate = candidate
        self.recovery_success_count = (
            self.recovery_success_count + 1 if stable else 1
        )
        if self.recovery_success_count < self.recovery_successes:
            return HardCandidateDecision(
                False, "recovery_validating", True,
                self.recovery_success_count, trusted_offset, min_margin,
            )

        self.hold_active = False
        self._trusted_target = candidate.copy()
        count = self.recovery_success_count
        self.recovery_success_count = 0
        self._previous_recovery_candidate = None
        return HardCandidateDecision(
            True, "recovered", False, count, trusted_offset, min_margin,
        )

    def limit_accepted_candidate(self, candidate, dt):
        candidate = self._vector(candidate)
        if candidate is None:
            raise ValueError("accepted candidate must be finite and match dof")
        if self._command is None:
            self.reset(candidate)
            return candidate
        dt = float(dt)
        if not np.isfinite(dt) or dt <= 0.0:
            raise ValueError("dt must be finite and positive")

        error = candidate - self._command
        # Reserve enough distance to stop with the configured acceleration.
        # Solving v*dt + v^2/(2*a) <= |error| avoids the final target snap that
        # would otherwise violate the acceleration limit.
        if self.max_acceleration > 0.0:
            stop_velocity = (
                -self.max_acceleration * dt
                + np.sqrt(
                    (self.max_acceleration * dt) ** 2
                    + 2.0 * self.max_acceleration * np.abs(error)
                )
            )
        else:
            stop_velocity = np.zeros(self.dof, dtype=float)
        desired_velocity = np.sign(error) * np.minimum(
            self.max_velocity, stop_velocity
        )
        velocity_delta = self.max_acceleration * dt
        next_velocity = np.clip(
            desired_velocity,
            self._velocity - velocity_delta,
            self._velocity + velocity_delta,
        )
        step = next_velocity * dt

        self._command = self._command + step
        self._velocity = next_velocity
        return self._command.copy()
