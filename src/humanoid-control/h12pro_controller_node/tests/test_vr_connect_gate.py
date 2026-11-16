#!/usr/bin/env python3
"""#4673 VR connect gate unit tests (no ROS / no H12 hardware)."""
import unittest
from unittest.mock import Mock

from utils.vr_connect_gate import (
    DEFAULT_LAUNCH_VR_REMOTE_CONTROL_CMD,
    QUEST3_CONNECTED_PARAM,
    VR_QUEST_CONNECT_TIMEOUT_SEC,
    capture_tmux_pane_tail,
    describe_quest_connect_timeout,
    reset_quest3_connected_for_new_session,
    resolve_launch_vr_cmd,
    wait_for_quest3_connected,
)


class ResolveLaunchCmdTest(unittest.TestCase):
    def test_none_and_blank_use_default(self):
        cmd, used = resolve_launch_vr_cmd(None)
        self.assertEqual(cmd, DEFAULT_LAUNCH_VR_REMOTE_CONTROL_CMD)
        self.assertTrue(used)
        cmd, used = resolve_launch_vr_cmd("   ")
        self.assertEqual(cmd, DEFAULT_LAUNCH_VR_REMOTE_CONTROL_CMD)
        self.assertTrue(used)

    def test_env_value_preserved(self):
        custom = "bash /ws/platform_roslaunch.sh noitom_hi5_hand_udp_python launch_quest3_ik.launch"
        cmd, used = resolve_launch_vr_cmd("  " + custom + "  ")
        self.assertEqual(cmd, custom)
        self.assertFalse(used)

    def test_default_matches_140_style(self):
        self.assertIn("noitom_hi5_hand_udp_python", DEFAULT_LAUNCH_VR_REMOTE_CONTROL_CMD)
        self.assertIn("launch_quest3_ik.launch", DEFAULT_LAUNCH_VR_REMOTE_CONTROL_CMD)


class WaitConnectedTest(unittest.TestCase):
    def test_returns_true_when_param_becomes_true(self):
        values = [False, False, True]
        has = Mock(return_value=True)
        get = Mock(side_effect=lambda *_a, **_k: values.pop(0) if values else True)
        sleeps = []
        ok = wait_for_quest3_connected(
            timeout_sec=2.0,
            poll_interval_sec=0.01,
            has_param_fn=has,
            get_param_fn=get,
            sleep_fn=lambda s: sleeps.append(s),
            monotonic_fn=Mock(side_effect=[0.0, 0.01, 0.02, 0.03, 0.04]),
        )
        self.assertTrue(ok)
        self.assertTrue(sleeps)  # polled at least once before True on 3rd check... 
        # Actually True on 3rd get after 2 sleeps from first two False
        self.assertGreaterEqual(len(sleeps), 2)

    def test_timeout_when_never_connected(self):
        clock = [0.0]

        def mono():
            return clock[0]

        def sleep(dt):
            clock[0] += dt

        ok = wait_for_quest3_connected(
            timeout_sec=0.05,
            poll_interval_sec=0.02,
            has_param_fn=lambda _p: True,
            get_param_fn=lambda *_a, **_k: False,
            sleep_fn=sleep,
            monotonic_fn=mono,
        )
        self.assertFalse(ok)
        self.assertGreaterEqual(clock[0], 0.05)

    def test_missing_param_counts_as_not_connected(self):
        clock = [0.0]

        def mono():
            return clock[0]

        def sleep(dt):
            clock[0] += dt

        ok = wait_for_quest3_connected(
            timeout_sec=0.03,
            poll_interval_sec=0.02,
            has_param_fn=lambda _p: False,
            get_param_fn=lambda *_a, **_k: True,  # should not be consulted
            sleep_fn=sleep,
            monotonic_fn=mono,
        )
        self.assertFalse(ok)

    def test_shutdown_aborts_wait(self):
        ok = wait_for_quest3_connected(
            timeout_sec=5.0,
            poll_interval_sec=0.01,
            has_param_fn=lambda _p: False,
            get_param_fn=lambda *_a, **_k: False,
            sleep_fn=lambda _s: None,
            monotonic_fn=Mock(side_effect=[0.0, 0.0]),
            is_shutdown_fn=lambda: True,
        )
        self.assertFalse(ok)


class DescribeTimeoutTest(unittest.TestCase):
    def test_param_missing_mentions_monitor_not_started(self):
        msg = describe_quest_connect_timeout(
            timeout_sec=VR_QUEST_CONNECT_TIMEOUT_SEC,
            used_default_launch=True,
            launch_cmd=DEFAULT_LAUNCH_VR_REMOTE_CONTROL_CMD,
            has_param=False,
            connected_value=None,
            tmux_pane_tail="",
        )
        self.assertIn(QUEST3_CONNECTED_PARAM, msg)
        self.assertIn("尚未出现", msg)
        self.assertIn("1.4.0 默认", msg)
        self.assertIn("H3", msg)

    def test_param_false_mentions_handshake(self):
        msg = describe_quest_connect_timeout(
            timeout_sec=12.0,
            used_default_launch=False,
            launch_cmd="custom launch",
            has_param=True,
            connected_value=False,
            tmux_pane_tail="Waiting for Quest3\nline2\n",
        )
        self.assertIn("=False", msg)
        self.assertIn("10019", msg)
        self.assertIn("环境变量", msg)
        self.assertIn("Waiting for Quest3", msg)

    def test_tmux_tail_truncated_to_last_20_lines(self):
        pane = "\n".join(f"L{i}" for i in range(30))
        msg = describe_quest_connect_timeout(
            timeout_sec=1.0,
            used_default_launch=False,
            launch_cmd="x",
            has_param=True,
            connected_value=False,
            tmux_pane_tail=pane,
        )
        self.assertNotIn("| L0", msg)
        self.assertIn("| L29", msg)
        self.assertIn("| L10", msg)


class CaptureTmuxTest(unittest.TestCase):
    def test_success_returns_stdout(self):
        result = Mock(returncode=0, stdout="hello pane\n")
        run = Mock(return_value=result)
        self.assertEqual(capture_tmux_pane_tail("vr_remote_control", run), "hello pane\n")
        run.assert_called_once()

    def test_failure_returns_empty(self):
        run = Mock(side_effect=RuntimeError("no tmux"))
        self.assertEqual(capture_tmux_pane_tail("vr_remote_control", run), "")



class ResetConnectedTest(unittest.TestCase):
    def test_reset_forces_false(self):
        store = {QUEST3_CONNECTED_PARAM: True}
        reset_quest3_connected_for_new_session(
            set_param_fn=lambda k, v: store.__setitem__(k, v)
        )
        self.assertFalse(store[QUEST3_CONNECTED_PARAM])


class FreshnessGateTest(unittest.TestCase):
    """回调级回归：旧值 True + 本次 monitor 未启动时不得切态。"""

    def _clock(self):
        clock = [0.0]

        def mono():
            return clock[0]

        def sleep(dt):
            clock[0] += dt

        return clock, mono, sleep

    def test_stale_true_without_reset_would_pass_but_require_false_blocks(self):
        """历史 True 在 require_false_before_true 下不会立刻放行。"""
        store = {QUEST3_CONNECTED_PARAM: True}  # 残留
        clock, mono, sleep = self._clock()
        ok = wait_for_quest3_connected(
            timeout_sec=0.05,
            poll_interval_sec=0.02,
            has_param_fn=lambda _p: QUEST3_CONNECTED_PARAM in store,
            get_param_fn=lambda _p, default=False: store.get(
                QUEST3_CONNECTED_PARAM, default
            ),
            sleep_fn=sleep,
            monotonic_fn=mono,
            require_false_before_true=True,
        )
        self.assertFalse(ok)
        self.assertGreaterEqual(clock[0], 0.05)

    def test_reset_then_stale_never_refreshed_times_out(self):
        """复位后若本次 monitor 未把参数置 True，不得切态。"""
        store = {QUEST3_CONNECTED_PARAM: True}
        reset_quest3_connected_for_new_session(
            set_param_fn=lambda k, v: store.__setitem__(k, v)
        )
        self.assertFalse(store[QUEST3_CONNECTED_PARAM])
        clock, mono, sleep = self._clock()
        ok = wait_for_quest3_connected(
            timeout_sec=0.05,
            poll_interval_sec=0.02,
            has_param_fn=lambda _p: QUEST3_CONNECTED_PARAM in store,
            get_param_fn=lambda _p, default=False: store.get(
                QUEST3_CONNECTED_PARAM, default
            ),
            sleep_fn=sleep,
            monotonic_fn=mono,
            require_false_before_true=True,
        )
        self.assertFalse(ok)

    def test_reset_then_fresh_true_passes(self):
        """复位后观测到 False，再出现 True，视为本次连通。"""
        store = {QUEST3_CONNECTED_PARAM: True}
        reset_quest3_connected_for_new_session(
            set_param_fn=lambda k, v: store.__setitem__(k, v)
        )
        values = [False, False, True]
        clock, mono, sleep = self._clock()

        def get_param(_p, default=False):
            if values:
                store[QUEST3_CONNECTED_PARAM] = values.pop(0)
            return store.get(QUEST3_CONNECTED_PARAM, default)

        ok = wait_for_quest3_connected(
            timeout_sec=2.0,
            poll_interval_sec=0.01,
            has_param_fn=lambda _p: True,
            get_param_fn=get_param,
            sleep_fn=sleep,
            monotonic_fn=Mock(side_effect=[0.0, 0.01, 0.02, 0.03, 0.04, 0.05]),
            require_false_before_true=True,
        )
        self.assertTrue(ok)

    def test_cpp_monitor_contract_param_missing_times_out(self):
        """C++ monitor 未写 /quest3/connected 时门禁应超时（修复前行为）。"""
        clock, mono, sleep = self._clock()
        ok = wait_for_quest3_connected(
            timeout_sec=0.04,
            poll_interval_sec=0.02,
            has_param_fn=lambda _p: False,
            get_param_fn=lambda *_a, **_k: False,
            sleep_fn=sleep,
            monotonic_fn=mono,
            require_false_before_true=True,
        )
        self.assertFalse(ok)

    def test_cpp_monitor_contract_writes_true_after_handshake(self):
        """C++ monitor 握手成功写 True 后门禁放行（修复后契约）。"""
        store = {QUEST3_CONNECTED_PARAM: False}  # init / reset
        seq = [False, True]  # handshake completes
        clock = [0.0]

        def get_param(_p, default=False):
            if seq:
                store[QUEST3_CONNECTED_PARAM] = seq.pop(0)
            return store[QUEST3_CONNECTED_PARAM]

        ok = wait_for_quest3_connected(
            timeout_sec=2.0,
            poll_interval_sec=0.01,
            has_param_fn=lambda _p: True,
            get_param_fn=get_param,
            sleep_fn=lambda s: clock.__setitem__(0, clock[0] + s),
            monotonic_fn=Mock(side_effect=[0.0, 0.01, 0.02, 0.03]),
            require_false_before_true=True,
        )
        self.assertTrue(ok)

    def test_reset_then_handshake_before_first_poll_passes(self):
        """复位后、首次轮询前握手完成且持续 True，应正常放行（生产启动时序）。

        回调顺序：reset False → 启动 session → sleep(3) → wait。
        若这 3 秒内 monitor 已写 True，等待器首次轮询就看到 True；
        因复位已完成，应 require_false_before_true=False 直接放行。
        """
        store = {QUEST3_CONNECTED_PARAM: True}
        reset_quest3_connected_for_new_session(
            set_param_fn=lambda k, v: store.__setitem__(k, v)
        )
        self.assertFalse(store[QUEST3_CONNECTED_PARAM])
        # 模拟 sleep(3) 期间握手完成
        store[QUEST3_CONNECTED_PARAM] = True
        clock, mono, sleep = self._clock()
        ok = wait_for_quest3_connected(
            timeout_sec=2.0,
            poll_interval_sec=0.01,
            has_param_fn=lambda _p: True,
            get_param_fn=lambda _p, default=False: store.get(
                QUEST3_CONNECTED_PARAM, default
            ),
            sleep_fn=sleep,
            monotonic_fn=Mock(side_effect=[0.0, 0.01, 0.02]),
            require_false_before_true=False,
        )
        self.assertTrue(ok)

    def test_require_false_after_reset_misrejects_pre_poll_true(self):
        """文档化错误用法：复位后仍 require_false，会误拒绝首次轮询前的 True。"""
        store = {QUEST3_CONNECTED_PARAM: True}
        reset_quest3_connected_for_new_session(
            set_param_fn=lambda k, v: store.__setitem__(k, v)
        )
        store[QUEST3_CONNECTED_PARAM] = True  # 握手发生在首次轮询前
        clock, mono, sleep = self._clock()
        ok = wait_for_quest3_connected(
            timeout_sec=0.05,
            poll_interval_sec=0.02,
            has_param_fn=lambda _p: True,
            get_param_fn=lambda _p, default=False: store.get(
                QUEST3_CONNECTED_PARAM, default
            ),
            sleep_fn=sleep,
            monotonic_fn=mono,
            require_false_before_true=True,
        )
        self.assertFalse(ok)

    def test_callback_start_order_reset_then_wait_accepts_pre_poll_true(self):
        """覆盖两处回调启动顺序：kill→reset→launch/sleep→wait(False)。"""
        store = {QUEST3_CONNECTED_PARAM: True}  # 旧残留
        # 1) reset（对应 kill 旧 session 之后）
        reset_quest3_connected_for_new_session(
            set_param_fn=lambda k, v: store.__setitem__(k, v)
        )
        self.assertFalse(store[QUEST3_CONNECTED_PARAM])
        # 2) 启动新 session 的 sleep 窗口内完成本次握手
        store[QUEST3_CONNECTED_PARAM] = True
        # 3) wait：与 ocs2/multi_before_callback 一致，复位后不再要求轮询内 False
        ok = wait_for_quest3_connected(
            timeout_sec=1.0,
            poll_interval_sec=0.01,
            has_param_fn=lambda _p: True,
            get_param_fn=lambda _p, default=False: store.get(
                QUEST3_CONNECTED_PARAM, default
            ),
            sleep_fn=lambda _s: None,
            monotonic_fn=Mock(side_effect=[0.0, 0.01]),
            require_false_before_true=False,
        )
        self.assertTrue(ok)



if __name__ == "__main__":
    unittest.main()
