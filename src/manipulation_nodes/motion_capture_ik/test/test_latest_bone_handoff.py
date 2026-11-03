#!/usr/bin/env python3

import os
import sys
import threading
import time
import unittest
from types import SimpleNamespace
from unittest.mock import MagicMock, patch

import numpy as np


SCRIPTS_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "scripts"))
if SCRIPTS_DIR not in sys.path:
    sys.path.insert(0, SCRIPTS_DIR)

import ik_ros_uni  # noqa: E402
from ik_ros_uni import (  # noqa: E402
    IkRos,
    IkSolutionFrame,
    TRANSFORM_STAGE_DIAGNOSTIC_FIELDS,
    calculate_first_publish_latencies_ms,
    calculate_ik_stage_latencies_ms,
)


class FakeTransformer:
    def __init__(self):
        self.left_shoulder_rpy_in_robot = [0.1, 0.2, 0.3]
        self.right_shoulder_rpy_in_robot = [-0.1, -0.2, -0.3]
        self.is_runing = True
        self.is_hand_tracking = True
        self._message = None

    def read_msg(self, message, collect_timing=False, joystick_snapshot=None,
                 defer_finger_processing=False):
        self._message = message
        if collect_timing:
            return {
                "input_validation_ms": 0.1,
                "finger_compute_ms": float("nan"),
                "chest_context_compute_ms": 0.2,
                "left_hand_compute_ms": 0.3,
                "right_hand_compute_ms": 0.4,
                "arm_debug_publish_ms": 0.05,
                "arm_pair_compute_ms": 0.95,
                "gesture_publish_ms": 0.5,
                "read_msg_total_ms": 1.5,
            }
        return True

    def get_hand_pose(self, side):
        sign = 1.0 if side == "Left" else -1.0
        value = float(self._message.value)
        return (
            (np.array([value, sign, 0.5]), np.array([0.0, 0.0, 0.0, 1.0])),
            np.array([value, sign, 0.25]),
        )

    def get_finger_joints(self, side):
        sign = 1.0 if side == "Left" else -1.0
        return [sign * float(self._message.value)] * 6

    def get_finger_state(self):
        return {
            "left_finger_joints": self.get_finger_joints("Left"),
            "right_finger_joints": self.get_finger_joints("Right"),
            "is_running": self.is_runing,
        }

    def process_finger_frame(self, message, joystick_snapshot):
        self._message = message
        return {
            "left_finger_joints": self.get_finger_joints("Left"),
            "right_finger_joints": self.get_finger_joints("Right"),
            "is_running": self.is_runing,
            "is_hand_tracking": self.is_hand_tracking,
        }

    def check_if_vr_error(self):
        return False


def make_test_ik_ros(enable_diagnostics=False):
    obj = IkRos.__new__(IkRos)
    obj.enable_vr_latency_diagnostics = True
    obj.stop_event = threading.Event()
    obj._bone_condition = threading.Condition()
    obj._pending_bone_frame = None
    obj._bone_input_seq = 0
    obj._bone_overwrite_count = 0
    obj._last_reported_bone_overwrite_count = 0
    obj._target_lock = threading.Lock()
    obj._ik_condition = threading.Condition(obj._target_lock)
    obj._ik_target_generation = 0
    obj._ik_target_ready_time = 0.0
    obj._ik_target_ready_monotonic = 0.0
    obj._ik_solution_lock = threading.Lock()
    obj._latest_ik_solution = None
    obj._last_published_arm_q = None
    obj._joystick_lock = threading.Lock()
    obj._joystick_snapshot = ((0.0, 0.0), (0.0, 0.0))
    obj._finger_condition = threading.Condition()
    obj._pending_finger_input = None
    obj.finger_processing_hz = 30.0
    obj._latest_processed_bone = None
    obj._vr_is_running = False
    obj._vr_is_hand_tracking = False
    obj._vr_error = False
    obj._IkRos__target_pose = (None, None)
    obj._IkRos__target_pose_right = (None, None)
    obj._IkRos__left_elbow_pos = None
    obj._IkRos__right_elbow_pos = None
    obj._IkRos__recieved_new_target_pose = False
    obj.hand_finger_data = None
    obj.robot_type = 0
    obj.quest3_arm_info_transformer = FakeTransformer()
    obj.comm_latency_pub = MagicMock()
    obj.bone_overwrite_count_pub = MagicMock()
    obj.transform_processing_latency_pub = MagicMock()
    obj.transform_pipeline_latency_pub = MagicMock()
    obj.ik_target_commit_latency_pub = MagicMock()
    obj.finger_processing_latency_pub = MagicMock()
    obj.transform_stage_latency_pub = MagicMock() if enable_diagnostics else None
    return obj


class LatestBoneHandoffTest(unittest.TestCase):
    def test_callback_overwrites_pending_frame(self):
        obj = make_test_ik_ros()
        first = SimpleNamespace(timestamp_ms=int(time.time() * 1000), value=1)
        second = SimpleNamespace(timestamp_ms=int(time.time() * 1000), value=2)

        obj.quest_bone_poses_callback(first)
        obj.quest_bone_poses_callback(second)

        self.assertEqual(obj._bone_input_seq, 2)
        self.assertEqual(obj._bone_overwrite_count, 1)
        self.assertIs(obj._pending_bone_frame.message, second)

    def test_callback_skips_latency_publish_when_diagnostics_disabled(self):
        obj = make_test_ik_ros()
        obj.enable_vr_latency_diagnostics = False
        frame = SimpleNamespace(timestamp_ms=int(time.time() * 1000), value=1)

        obj.quest_bone_poses_callback(frame)

        obj.comm_latency_pub.publish.assert_not_called()

    def test_worker_publishes_atomic_snapshot_from_newest_frame(self):
        obj = make_test_ik_ros()
        first = SimpleNamespace(timestamp_ms=int(time.time() * 1000), value=1)
        newest = SimpleNamespace(timestamp_ms=int(time.time() * 1000), value=7)
        obj.quest_bone_poses_callback(first)
        obj.quest_bone_poses_callback(newest)

        worker = threading.Thread(target=obj._bone_processing_loop)
        worker.start()
        deadline = time.time() + 1.0
        while obj._latest_processed_bone is None and time.time() < deadline:
            time.sleep(0.005)

        obj.stop_event.set()
        with obj._bone_condition:
            obj._bone_condition.notify_all()
        worker.join(timeout=1.0)

        self.assertFalse(worker.is_alive())
        snapshot = obj._latest_processed_bone
        self.assertIsNotNone(snapshot)
        self.assertEqual(snapshot.sequence, 2)
        self.assertGreater(snapshot.transform_done_monotonic, 0.0)
        self.assertEqual(snapshot.left_pose[0][0], 7.0)
        self.assertEqual(snapshot.right_pose[0][0], 7.0)
        self.assertTrue(obj._IkRos__recieved_new_target_pose)
        self.assertEqual(obj._ik_target_generation, 1)
        obj.bone_overwrite_count_pub.publish.assert_called_once()
        obj.transform_processing_latency_pub.publish.assert_called_once()
        obj.transform_pipeline_latency_pub.publish.assert_called_once()
        obj.ik_target_commit_latency_pub.publish.assert_called_once()

    def test_calculates_same_frame_ik_stage_latencies(self):
        snapshot = SimpleNamespace(transform_done_monotonic=10.0)

        wait_ms, solve_ms, postprocess_ms = calculate_ik_stage_latencies_ms(
            snapshot,
            solve_start_monotonic=10.004,
            solve_done_monotonic=10.007,
            ik_done_monotonic=10.012,
        )

        self.assertAlmostEqual(wait_ms, 4.0)
        self.assertAlmostEqual(solve_ms, 3.0)
        self.assertAlmostEqual(postprocess_ms, 5.0)

    def test_calculates_first_publish_latencies_for_same_frame(self):
        processed_bone = SimpleNamespace(
            receive_time=20.000,
            source_timestamp_ms=19998,
        )
        solution = IkSolutionFrame(
            target_generation=7,
            processed_bone=processed_bone,
            desired_arm_q=np.zeros(14),
            shoulder_velocity_limit=120.0,
            ready_time=20.006,
            ready_monotonic=10.006,
        )

        solution_wait_ms, callback_ms, source_ms = (
            calculate_first_publish_latencies_ms(
                solution,
                publish_time=20.010,
                publish_monotonic=10.010,
            )
        )

        self.assertAlmostEqual(solution_wait_ms, 4.0)
        self.assertAlmostEqual(callback_ms, 10.0)
        self.assertAlmostEqual(source_ms, 12.0)

    def test_fixed_rate_publisher_uses_latest_solution_mailbox(self):
        obj = IkRos.__new__(IkRos)
        obj.enable_vr_latency_diagnostics = True
        obj.controller_dt = 0.01
        obj.stop_event = threading.Event()
        obj._ik_solution_lock = threading.Lock()
        obj._latest_ik_solution = IkSolutionFrame(
            target_generation=3,
            processed_bone=None,
            desired_arm_q=np.ones(14) * 0.2,
            shoulder_velocity_limit=20.0,
            ready_time=time.time(),
            ready_monotonic=time.perf_counter(),
        )
        obj._last_published_arm_q = np.zeros(14)
        obj._ik_publish_timeout_count = 0
        obj.ik_solution_timeout_s = 0.2
        obj.arm_mode_changing = False
        obj._IkRos__ik_claw_publish_active = False
        obj.limit_angle_by_velocity = MagicMock(
            return_value=np.ones(14) * 0.1
        )
        obj.pub_filtered_joint = MagicMock()
        obj.ik_publish_execution_latency_pub = MagicMock()
        obj.ik_publish_period_pub = MagicMock()
        obj.ik_solution_to_publish_latency_pub = MagicMock()
        obj.published_arm_traj_latency_pub = MagicMock()
        obj.published_end_to_end_latency_pub = MagicMock()
        obj.ik_publish_timeout_pub = MagicMock()

        def publish_once(**_kwargs):
            obj.stop_event.set()
            return True

        obj.publish_joint_states = MagicMock(side_effect=publish_once)

        fake_rate = MagicMock()
        with patch.object(ik_ros_uni.rospy, "Rate", return_value=fake_rate), \
                patch.object(ik_ros_uni.rospy, "is_shutdown", return_value=False):
            obj.ik_fixed_rate_publish_thread()

        obj.limit_angle_by_velocity.assert_called_once()
        obj.publish_joint_states.assert_called_once()
        np.testing.assert_allclose(obj._last_published_arm_q, np.ones(14) * 0.1)
        obj.ik_solution_to_publish_latency_pub.publish.assert_called_once()

    def test_finger_worker_uses_newest_frame_outside_arm_worker(self):
        obj = make_test_ik_ros()
        now_ms = int(time.time() * 1000)
        first = SimpleNamespace(timestamp_ms=now_ms, value=2)
        newest = SimpleNamespace(timestamp_ms=now_ms, value=9)
        obj._queue_latest_finger_input(
            SimpleNamespace(message=first), obj._joystick_snapshot
        )
        obj._queue_latest_finger_input(
            SimpleNamespace(message=newest), obj._joystick_snapshot
        )

        worker = threading.Thread(target=obj._finger_processing_loop)
        worker.start()
        deadline = time.time() + 1.0
        while obj.hand_finger_data is None and time.time() < deadline:
            time.sleep(0.005)

        obj.stop_event.set()
        with obj._finger_condition:
            obj._finger_condition.notify_all()
        worker.join(timeout=1.0)

        self.assertFalse(worker.is_alive())
        self.assertEqual(obj.hand_finger_data[0], [9.0] * 6)
        self.assertEqual(obj.hand_finger_data[1], [-9.0] * 6)
        obj.finger_processing_latency_pub.publish.assert_called_once()

    def test_worker_publishes_self_describing_transform_diagnostics(self):
        obj = make_test_ik_ros(enable_diagnostics=True)
        frame = SimpleNamespace(
            timestamp_ms=int(time.time() * 1000),
            value=3,
        )
        obj.quest_bone_poses_callback(frame)

        worker = threading.Thread(target=obj._bone_processing_loop)
        worker.start()
        deadline = time.time() + 1.0
        while (not obj.transform_stage_latency_pub.publish.called
               and time.time() < deadline):
            time.sleep(0.005)

        obj.stop_event.set()
        with obj._bone_condition:
            obj._bone_condition.notify_all()
        worker.join(timeout=1.0)

        obj.transform_stage_latency_pub.publish.assert_called_once()
        diagnostic = obj.transform_stage_latency_pub.publish.call_args.args[0]
        self.assertEqual(diagnostic.header.seq, 1)
        self.assertEqual(
            diagnostic.data.layout.dim[0].label,
            ",".join(TRANSFORM_STAGE_DIAGNOSTIC_FIELDS),
        )
        self.assertEqual(
            len(diagnostic.data.data), len(TRANSFORM_STAGE_DIAGNOSTIC_FIELDS)
        )
        values = dict(zip(TRANSFORM_STAGE_DIAGNOSTIC_FIELDS, diagnostic.data.data))
        self.assertTrue(np.isnan(values["finger_compute_ms"]))
        self.assertAlmostEqual(values["chest_context_compute_ms"], 0.2, places=5)
        self.assertAlmostEqual(values["left_hand_compute_ms"], 0.3, places=5)
        self.assertGreaterEqual(values["callback_to_worker_ms"], 0.0)
        self.assertGreaterEqual(values["worker_total_ms"], 0.0)


if __name__ == "__main__":
    unittest.main()
