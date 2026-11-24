"""Correct Quest controller pose discontinuities on the current source stamp."""
import math

import numpy as np

from pose_velocity_kalman import exp_so3, log_so3


class QuestSkeletonJumpCorrection:
    """Apply one coherent correction to each arm's shoulder, elbow and palm."""

    indices = (0, 1, 4, 2, 3, 5)
    palms = (4, 5)
    max_gap = .1
    position_rejoin_speed = .8
    rotation_rejoin_speed = 3.5
    shared_chest_turn_cos = math.cos(math.radians(25.))
    shared_arm_turn_cos = math.cos(math.radians(8.))
    local_arm_turn = math.radians(15.)

    def __init__(self):
        self.correcting = np.zeros(2, dtype=bool)
        self.reset()

    def reset(self):
        self.stamp = None
        self.key = None
        self.world_from_tracking = None
        self.correcting[:] = False
        self.rebased = False
        self.measurements = None

    def update(self, transforms, stamp, key):
        dt = stamp-self.stamp if self.stamp is not None else 0.
        if self.key == key and self.stamp is not None and dt == 0.:
            self.rebased = False
            return self.measurements

        chest = transforms[23]
        chest_rotation = chest[:3, :3]
        chest_position = chest[:3, 3]
        position = np.array([transforms[i][:3, 3] for i in self.indices])
        position = (position - chest_position) @ chest_rotation
        palms = chest_rotation.T @ np.array([transforms[i][:3, :3] for i in self.palms])
        contract_changed = key != self.key or dt < 0. or self.stamp is None
        if contract_changed:
            self.key = key
            self.world_from_tracking = None
        if contract_changed or dt > self.max_gap:
            # Restart local rejoin history in tracking space; the shared frame survives a gap.
            self.correcting[:] = False
            self.rebased = False
            self.stamp, self.chest = stamp, chest
            self.previous_position = self.position = position
            self.previous_palms = self.palm_rotation = palms
            self.measurements = self._in_world_frame(transforms)
            return self.measurements

        delta = position-self.previous_position
        displacement_squared = np.einsum('ij,ij->i', delta, delta).reshape(2, 3).max(axis=1)
        rotation_step_cos = .5*(np.einsum('ijk,ijk->i', palms, self.previous_palms)-1.)
        chest_delta = chest_position-self.chest[:3, 3]
        chest_turn_cos = .5*(np.sum(chest_rotation*self.chest[:3, :3])-1.)
        # A rigid tracking-origin change has a large world-space jump and small
        # chest-relative error in both arms. Keep all filtered bones in one frame.
        self.rebased = (
            (chest_delta @ chest_delta > .25**2 or chest_turn_cos < self.shared_chest_turn_cos)
            and displacement_squared.max() < .025**2
            and rotation_step_cos.min() > self.shared_arm_turn_cos
        )
        if self.rebased:
            next_to_previous = np.eye(4)
            next_to_previous[:3, :3] = self.chest[:3, :3] @ chest_rotation.T
            next_to_previous[:3, 3] = self.chest[:3, 3] - next_to_previous[:3, :3] @ chest_position
            self.world_from_tracking = (next_to_previous if self.world_from_tracking is None
                                        else self.world_from_tracking @ next_to_previous)

        abrupt = ((displacement_squared > max(.025, 4.*dt)**2)
                  | (rotation_step_cos < math.cos(max(self.local_arm_turn, 18.*dt))))
        consider = abrupt | self.correcting
        needs_local_correction = consider.any()
        self.correcting[:] = False
        result = self._in_world_frame(transforms)
        if needs_local_correction and result is transforms:
            result = transforms.copy()

        corrected_position, corrected_palms = position, palms
        if needs_local_correction:
            corrected_position, corrected_palms = position.copy(), palms.copy()
            body = result[23]
            for side in np.flatnonzero(consider):
                group = slice(3*side, 3*(side+1))
                delta = position[group]-self.position[group]
                relative_turn = log_so3(palms[side] @ self.palm_rotation[side].T)
                # These limits are active only during a detected discontinuity
                # and the following continuous rejoin, using measured source dt.
                scale = 1./max(1., np.linalg.norm(delta, axis=1).max()/(self.position_rejoin_speed*dt),
                               np.linalg.norm(relative_turn)/(self.rotation_rejoin_speed*dt))
                if scale == 1.:
                    continue
                self.correcting[side] = True
                corrected_position[group] = self.position[group] + scale*delta
                corrected_palms[side] = exp_so3(scale*relative_turn) @ self.palm_rotation[side]
                for bone, xyz in zip(self.indices[group], corrected_position[group]):
                    pose = result[bone].copy()
                    pose[:3, 3] = body[:3, 3] + body[:3, :3] @ xyz
                    if bone == self.palms[side]:
                        pose[:3, :3] = body[:3, :3] @ corrected_palms[side]
                    result[bone] = pose
        self.stamp, self.chest = stamp, chest
        self.previous_position, self.previous_palms = position, palms
        self.position, self.palm_rotation = corrected_position, corrected_palms
        self.measurements = result
        return result

    def _in_world_frame(self, transforms):
        if self.world_from_tracking is None:
            return transforms
        return {i: self.world_from_tracking @ pose for i, pose in transforms.items()}
