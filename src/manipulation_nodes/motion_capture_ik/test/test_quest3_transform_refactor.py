#!/usr/bin/env python3

import os
import sys
import threading
import unittest
from types import SimpleNamespace
from unittest.mock import MagicMock, patch

import numpy as np


SCRIPTS_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "scripts"))
if SCRIPTS_DIR not in sys.path:
    sys.path.insert(0, SCRIPTS_DIR)

from tools import quest3_utils  # noqa: E402


def make_pose(position, quaternion=(0.0, 0.0, 0.0, 1.0)):
    return SimpleNamespace(
        position=SimpleNamespace(
            x=float(position[0]), y=float(position[1]), z=float(position[2])
        ),
        orientation=SimpleNamespace(
            x=float(quaternion[0]),
            y=float(quaternion[1]),
            z=float(quaternion[2]),
            w=float(quaternion[3]),
        ),
    )


def make_transformer_for_arm_test():
    obj = quest3_utils.Quest3ArmInfoTransformer.__new__(
        quest3_utils.Quest3ArmInfoTransformer
    )
    obj.pose_info_list = None
    obj.init_R_wC = quest3_utils.rpy_to_matrix([np.pi / 2.0, 0.0, np.pi / 2.0])
    obj.init_R_wLS = None
    obj.init_R_wRS = None
    obj.left_shoulder_rpy_in_robot = [0.0, 0.0, 0.0]
    obj.right_shoulder_rpy_in_robot = [0.0, 0.0, 0.0]
    obj.chest_axis_agl = [0.0, 0.0, 0.0]
    obj.head_body_pose = quest3_utils.HeadBodyPose()
    obj.is_hand_tracking = False
    obj.shoulder_width = 0.15
    obj.left_hand_pose = None
    obj.right_hand_pose = None
    obj.left_elbow_pos = None
    obj.right_elbow_pos = None
    obj.debug_pub = False
    obj.vis_pub = False
    obj._debug_publish_period = 0.1
    obj._last_debug_publish_monotonic = float("-inf")
    obj._finger_state_lock = threading.RLock()
    obj.shoulder_angle_puber = None
    obj.chest_axis_puber = None
    obj.marker_pub_chest = None
    # Keep this test focused on coordinate conversion, not arm-length scaling.
    obj.scale_arm_positions = MagicMock(
        side_effect=lambda shoulder, elbow, hand, *_args: (
            np.asarray(elbow).copy(), np.asarray(hand).copy()
        )
    )
    return obj


class Quest3TransformRefactorTest(unittest.TestCase):
    def test_read_msg_keeps_inline_fingers_for_existing_callers(self):
        obj = make_transformer_for_arm_test()
        obj.left_joystick = [0.0, 0.0]
        obj.right_joystick = [0.0, 0.0]
        obj.is_runing = False
        obj.compute_finger_joints = MagicMock(return_value=[0.0] * 6)
        obj.compute_finger_joints_joy = MagicMock(return_value=[0.0] * 6)
        obj.compute_arm_poses = MagicMock(return_value=True)
        obj._update_gesture_state_locked = MagicMock()
        obj.pub_head_body_pose_msg = MagicMock()
        msg = SimpleNamespace(
            poses=[make_pose([0.0, 0.0, 0.0]) for _ in quest3_utils.bone_names],
            timestamp_ms=1,
            is_high_confidence=True,
            is_hand_tracking=True,
        )

        self.assertTrue(obj.read_msg(msg))

        self.assertEqual(obj.compute_finger_joints.call_count, 2)
        obj.compute_finger_joints.assert_any_call(
            "Left", pose_info_list=msg.poses
        )
        obj.compute_finger_joints.assert_any_call(
            "Right", pose_info_list=msg.poses
        )
        obj._update_gesture_state_locked.assert_called_once()

    def test_shoulder_rpy_is_converted_once_per_side(self):
        obj = make_transformer_for_arm_test()
        obj.init_R_wLS = np.eye(3)

        with patch.object(
            quest3_utils, "matrix_to_rpy", return_value=np.array([1.0, 2.0, 3.0])
        ) as convert:
            obj.compute_shoudler_pose(np.eye(3), "Left")

        convert.assert_called_once()
        np.testing.assert_allclose(obj.left_shoulder_rpy_in_robot, [3.0, -1.0, -2.0])

    def test_both_arms_share_one_chest_context_and_preserve_positions(self):
        obj = make_transformer_for_arm_test()
        poses = [make_pose([0.0, 0.0, 0.0]) for _ in quest3_utils.bone_names]
        poses[quest3_utils.bone_name_to_index["Chest"]] = make_pose([0.2, -0.1, 1.3])
        poses[quest3_utils.bone_name_to_index["LeftArmUpper"]] = make_pose([0.25, 0.1, 1.35])
        poses[quest3_utils.bone_name_to_index["LeftArmLower"]] = make_pose([0.45, 0.25, 1.15])
        poses[quest3_utils.bone_name_to_index["LeftHandPalm"]] = make_pose([0.6, 0.35, 1.05])
        poses[quest3_utils.bone_name_to_index["RightArmUpper"]] = make_pose([0.25, -0.3, 1.35])
        poses[quest3_utils.bone_name_to_index["RightArmLower"]] = make_pose([0.45, -0.45, 1.15])
        poses[quest3_utils.bone_name_to_index["RightHandPalm"]] = make_pose([0.6, -0.55, 1.05])

        chest_pose = poses[quest3_utils.bone_name_to_index["Chest"]]
        legacy_chest = obj.pose_info2_transform(chest_pose)
        axis, _ = quest3_utils.matrix_to_axis_angle(
            obj.init_R_wC.T @ legacy_chest[:3, :3]
        )
        compensation = quest3_utils.axis_angle_to_matrix([0.0, 0.0, axis[1]]).T
        chest_position = legacy_chest[:3, 3]

        expected = {}
        for side in ("Left", "Right"):
            hand_pose = poses[quest3_utils.bone_name_to_index[side + "HandPalm"]]
            elbow_pose = poses[quest3_utils.bone_name_to_index[side + "ArmLower"]]
            hand = compensation @ (obj.pose_info2_transform(hand_pose)[:3, 3] - chest_position)
            elbow = compensation @ (obj.pose_info2_transform(elbow_pose)[:3, 3] - chest_position)
            hand += np.asarray(quest3_utils.bias_chest_to_base_link)
            hand[0] = max(0.1, hand[0])
            elbow[0] += quest3_utils.bias_chest_to_base_link[0]
            elbow[2] += quest3_utils.bias_chest_to_base_link[2]
            expected[side] = (hand, elbow)

        with patch.object(
            obj,
            "_build_chest_frame_context",
            wraps=obj._build_chest_frame_context,
        ) as build_context, patch.object(
            quest3_utils,
            "transform_to_pos_rpy",
            side_effect=AssertionError("双臂位置计算不应再转换为 RPY"),
        ) as legacy_conversion:
            self.assertTrue(obj.compute_arm_poses(poses))

        build_context.assert_called_once_with(poses)
        legacy_conversion.assert_not_called()
        np.testing.assert_allclose(obj.left_hand_pose[0], expected["Left"][0], atol=1e-12)
        np.testing.assert_allclose(obj.left_elbow_pos, expected["Left"][1], atol=1e-12)
        np.testing.assert_allclose(obj.right_hand_pose[0], expected["Right"][0], atol=1e-12)
        np.testing.assert_allclose(obj.right_elbow_pos, expected["Right"][1], atol=1e-12)


if __name__ == "__main__":
    unittest.main()
