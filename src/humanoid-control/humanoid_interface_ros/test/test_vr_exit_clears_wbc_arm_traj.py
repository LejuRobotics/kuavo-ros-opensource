#!/usr/bin/env python3
"""Offline regression for kuavodevlab#5064.

VR low-latency mode sets /enable_wbc_arm_trajectory_control(1). G12 long-press B
kills the VR tmux session, and monitor_quest3 calls /quest3/reset_vr_torso_state
on the still-running QuestControlFSM. That reset used to clear only torso state,
so use_ros_arm_joint_trajectory_ stayed true in humanoidController. The next
G12 voice action then entered the WBC /kuavo_arm_traj path before fresh action
data was ready, applying a stale VR command (Phase-2 entry spike).

This test does not need ROS or a simulator:
1. Source contract: resetVrTorsoStateCallback always calls
   callEnableWbcArmTrajectorySrv(0), outside the torso-enabled-only branch.
2. Replay of the #5064 stdout sequence under the old vs new policy.
"""

from __future__ import annotations

import unittest
from pathlib import Path

REPO_CPP = (
    Path(__file__).resolve().parents[1] / "src" / "QuestControlFSMNode.cpp"
)

# Key events reconstructed from kuavo5-392 stdout, 2026-11-14, Asia/Shanghai.
# Times are seconds from an arbitrary origin for ordering only.
ISSUE_5064_EVENTS = [
    # 15:55:35 left front trigger + X: low-latency on
    ("15:55:35", "enable_wbc", 1),
    # 15:55:47 G12 long-press B: monitor_quest3 -> reset_vr_torso_state
    # (log: "Torso control already disabled (VR node shutdown reset)")
    ("15:55:47", "vr_shutdown_reset", None),
    # 15:56:22 first RR_A / G12 voice action starts
    ("15:56:22", "action_start", None),
]


def extract_function(source: str, name: str) -> str:
    """Return the body of a simple one-level C++ method named `name`."""
    start = source.find(f"bool {name}(")
    if start < 0:
        raise AssertionError(f"function {name} not found")
    brace = source.find("{", start)
    if brace < 0:
        raise AssertionError(f"opening brace for {name} not found")
    depth = 0
    for i, ch in enumerate(source[brace:], brace):
        if ch == "{":
            depth += 1
        elif ch == "}":
            depth -= 1
            if depth == 0:
                return source[start : i + 1]
    raise AssertionError(f"function {name} is not brace-balanced")


def split_torso_enabled_branch(callback_src: str) -> tuple[str, str]:
    """Split callback into the torso-enabled `if` body and the remainder after it."""
    header = "if (was_torso_control_enabled && robot_type_ != 1)"
    idx = callback_src.find(header)
    if idx < 0:
        raise AssertionError("torso-enabled branch not found")
    brace = callback_src.find("{", idx)
    depth = 0
    end = None
    for i, ch in enumerate(callback_src[brace:], brace):
        if ch == "{":
            depth += 1
        elif ch == "}":
            depth -= 1
            if depth == 0:
                end = i
                break
    if end is None:
        raise AssertionError("torso-enabled branch is not brace-balanced")
    return callback_src[brace : end + 1], callback_src[end + 1 :]


def replay_ros_arm(events, clear_on_shutdown: bool) -> list[tuple[str, int]]:
    """Replay enable/reset/action events; return (time, ros_arm) after each event."""
    ros_arm = 0
    snapshots = []
    for time_str, kind, value in events:
        if kind == "enable_wbc":
            ros_arm = 1 if value else 0
        elif kind == "vr_shutdown_reset":
            if clear_on_shutdown:
                ros_arm = 0
        elif kind == "action_start":
            pass
        else:
            raise AssertionError(f"unknown event kind: {kind}")
        snapshots.append((time_str, kind, ros_arm))
    return snapshots


class VrExitClearsWbcArmTrajTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.cpp_path = REPO_CPP
        if not cls.cpp_path.is_file():
            raise AssertionError(f"missing source file: {cls.cpp_path}")
        cls.source = cls.cpp_path.read_text(encoding="utf-8")
        cls.callback = extract_function(cls.source, "resetVrTorsoStateCallback")

    def test_source_file_exists(self):
        self.assertTrue(self.cpp_path.is_file(), self.cpp_path)

    def test_reset_callback_always_clears_wbc_arm_traj(self):
        inside_if, after_if = split_torso_enabled_branch(self.callback)
        self.assertNotIn(
            "callEnableWbcArmTrajectorySrv(0)",
            inside_if,
            "clearing only inside the torso-enabled branch would miss #5064, "
            "where torso control was already disabled at VR exit",
        )
        self.assertRegex(
            after_if,
            r"callEnableWbcArmTrajectorySrv\s*\(\s*0\s*\)",
            "resetVrTorsoStateCallback must call callEnableWbcArmTrajectorySrv(0) "
            "on every VR shutdown reset, including when torso control is already off",
        )
        clear_call = after_if.find("callEnableWbcArmTrajectorySrv(0)")
        stand_call = after_if.find("callResetArmTrajToStandSrv()")
        self.assertGreater(
            stand_call,
            clear_call,
            "VR shutdown must replace arm_joint_trajectory_.pos with the stand pose "
            "after clearing the low-latency flag",
        )

    def test_controller_resets_cached_traj_to_stand_pose(self):
        controller = (
            Path(__file__).resolve().parents[2]
            / "humanoid_controllers/src/humanoidController.cpp"
        )
        source = controller.read_text(encoding="utf-8")
        start = source.find("bool humanoidController::resetArmTrajToStandCallback")
        self.assertGreater(start, 0)
        callback = source[start : start + 1200]
        self.assertIn("arm_joint_trajectory_.pos = defalutArmPosMPC_", callback)
        self.assertIn("arm_traj_hold_stand_ = true", callback)
        receiver_at = source.find("arm_traj_receiver_.init")
        receiver = source[receiver_at : receiver_at + 2500]
        self.assertIn("if (!arm_traj_hold_stand_)", receiver)

    def test_reset_callback_still_clears_torso_state(self):
        self.assertIn("torso_control_enabled_ = false", self.callback)
        self.assertIn("callVRWaistControlSrv(false)", self.callback)

    def test_issue_5064_old_policy_leaves_ros_arm_enabled(self):
        snaps = replay_ros_arm(ISSUE_5064_EVENTS, clear_on_shutdown=False)
        by_time = {t: (kind, flag) for t, kind, flag in snaps}
        self.assertEqual(by_time["15:55:35"], ("enable_wbc", 1))
        self.assertEqual(by_time["15:55:47"], ("vr_shutdown_reset", 1))
        self.assertEqual(
            by_time["15:56:22"],
            ("action_start", 1),
            "pre-fix policy must reproduce the bug: first G12 action starts with ros_arm=1",
        )

    def test_issue_5064_new_policy_clears_ros_arm_before_action(self):
        snaps = replay_ros_arm(ISSUE_5064_EVENTS, clear_on_shutdown=True)
        by_time = {t: (kind, flag) for t, kind, flag in snaps}
        self.assertEqual(by_time["15:55:35"], ("enable_wbc", 1))
        self.assertEqual(
            by_time["15:55:47"],
            ("vr_shutdown_reset", 0),
            "VR shutdown reset must set ros_arm back to 0",
        )
        self.assertEqual(
            by_time["15:56:22"],
            ("action_start", 0),
            "first G12 action must no longer inherit leftover low-latency ros_arm=1",
        )

    def test_wbc_path_does_not_apply_stale_traj_after_clear(self):
        """Mirrors humanoidController: apply /kuavo_arm_traj only when ros_arm && EXTERN."""
        stale_cmd = -1.117  # rad, joint 2 leftover from #5064 bag _19 at 15:57:51
        current_q = -0.006

        def apply_wbc(ros_arm: int, arm_mode: int, cached_cmd: float, q: float) -> float:
            if ros_arm and arm_mode == 2:
                return cached_cmd
            return q

        # Old policy: exit leaves ros_arm=1; action then sets mode 2 before new traj arrives.
        old = apply_wbc(ros_arm=1, arm_mode=2, cached_cmd=stale_cmd, q=current_q)
        self.assertAlmostEqual(old, stale_cmd)
        self.assertGreater(abs(old - current_q), 1.0)

        # New policy: exit clears ros_arm; same mode switch does not apply the stale command.
        new = apply_wbc(ros_arm=0, arm_mode=2, cached_cmd=stale_cmd, q=current_q)
        self.assertAlmostEqual(new, current_q)


if __name__ == "__main__":
    unittest.main()
