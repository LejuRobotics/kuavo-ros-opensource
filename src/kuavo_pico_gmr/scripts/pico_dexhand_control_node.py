#!/usr/bin/env python3
# coding: utf-8
# Copyright: lejurobot 2025
"""
PICO 灵巧手控制独立节点

轻量级节点，订阅 /pico/joy 话题，将 PICO 手柄 grip 值映射为灵巧手指令，
发布到 /control_robot_hand_position。

适用场景：
  - GMR 链路 (pico_comm_minimal + pico_realtime_retarget) 下，无需启动
    pico_whole_body_teleop 即可控制灵巧手。
  - 避免 pico_whole_body_teleop 与 pico_comm_minimal 的 UDP 12345 端口冲突。

支持功能：
  - 左/右 grip → 6 指均匀抓握（0~100）
  - Y 按键 → 锁定/解锁手指输出
  - X 长按 → 左拇指张开切换
  - A 长按 → 右拇指张开切换
  - 解锁后回切阈值防突变

配置读取优先级：
  1. ~/.config/lejuconfig/pico_vr_config.yaml
  2. 本包 config/pico_vr_config.yaml 相对路径 (fallback)
  3. 内置默认值

用法：
  rosrun kuavo_pico_gmr pico_dexhand_control_node.py
  或由 pico_streaming.launch 的 enable_dexhand_control 参数自动启动。
"""

import os
import time
import threading
import rospy
import yaml

from kuavo_msgs.msg import JoySticks, robotHandPosition, lejuClawCommand


# ========================== 配置 ==========================

_DEX_HAND_DEFAULTS = {
    "command_min": 0,
    "command_max": 100,
    "smoothing_alpha": 0.35,
    "grip_deadzone": 0.02,
    "thumb_open": {
        "enabled": True,
        "joint_indices": [1],
        "open_value": 100,
        "require_teleop_unlock": True,
    },
    "stable_unlock": {
        "enabled": True,
        "reengage_threshold": 8,
    },
}


def _load_dex_hand_config() -> dict:
    """加载灵巧手配置，优先用户目录，再 fallback 到包内默认。"""
    user_cfg = os.path.expanduser("~/.config/lejuconfig/pico_vr_config.yaml")
    pkg_cfg = os.path.join(os.path.dirname(__file__), "..", "config", "pico_vr_config.yaml")
    # 兼容：pico-body-tracking-server 的 config 目录（从 scripts/ 向上两级到 src/）
    pico_srv_cfg = os.path.join(
        os.path.dirname(__file__), "..", "..",
        "manipulation_nodes", "pico-body-tracking-server", "config", "pico_vr_config.yaml"
    )

    cfg_path = None
    for p in [user_cfg, pkg_cfg, pico_srv_cfg]:
        if os.path.isfile(p):
            cfg_path = p
            break

    if cfg_path is None:
        rospy.logwarn("[DexHandCtrl] pico_vr_config.yaml not found, using built-in defaults")
        return dict(_DEX_HAND_DEFAULTS)

    rospy.loginfo(f"[DexHandCtrl] Loading config from {cfg_path}")
    try:
        with open(cfg_path, "r", encoding="utf-8") as f:
            raw = yaml.safe_load(f) or {}
    except Exception as e:
        rospy.logwarn(f"[DexHandCtrl] Failed to load config: {e}, using defaults")
        return dict(_DEX_HAND_DEFAULTS)

    dex = raw.get("dex_hand", {})
    thumb_cfg = dex.get("thumb_open", {})
    unlock_cfg = dex.get("stable_unlock", {})
    defaults = _DEX_HAND_DEFAULTS

    return {
        "command_min": dex.get("command_min", defaults["command_min"]),
        "command_max": dex.get("command_max", defaults["command_max"]),
        "smoothing_alpha": dex.get("smoothing_alpha", defaults["smoothing_alpha"]),
        "grip_deadzone": dex.get("grip_deadzone", defaults["grip_deadzone"]),
        "thumb_open": {
            "enabled": thumb_cfg.get("enabled", defaults["thumb_open"]["enabled"]),
            "joint_indices": thumb_cfg.get("joint_indices", defaults["thumb_open"]["joint_indices"]),
            "open_value": thumb_cfg.get("open_value", defaults["thumb_open"]["open_value"]),
            "require_teleop_unlock": thumb_cfg.get(
                "require_teleop_unlock", defaults["thumb_open"]["require_teleop_unlock"]
            ),
        },
        "stable_unlock": {
            "enabled": unlock_cfg.get("enabled", defaults["stable_unlock"]["enabled"]),
            "reengage_threshold": unlock_cfg.get(
                "reengage_threshold", defaults["stable_unlock"]["reengage_threshold"]
            ),
        },
    }


# ======================== 按键状态 ========================

LONG_PRESS_THRESHOLD = 0.50  # 秒
TRIGGER_THRESHOLD = 0.5


class _ButtonTracker:
    """极简按键上升沿 / 长按检测（仅跟踪 X, Y, A, B 四键）。"""

    def __init__(self):
        self._last = {"X": False, "Y": False, "A": False, "B": False}
        self._press_time = {"X": 0.0, "Y": 0.0, "A": 0.0, "B": 0.0}
        self._reported_long = {"X": False, "Y": False, "A": False, "B": False}

    def update(self, joy: JoySticks) -> set:
        """返回本帧激活的事件集合，如 {'Y_PRESSED', 'X_LONG_PRESSED', 'LT_IDLE', ...}"""
        btn_map = {
            "X": bool(joy.left_first_button_pressed),
            "Y": bool(joy.left_second_button_pressed),
            "A": bool(joy.right_first_button_pressed),
            "B": bool(joy.right_second_button_pressed),
        }
        now = time.time()
        events = set()

        for name, pressed in btn_map.items():
            was = self._last[name]
            if pressed and not was:
                # 上升沿
                self._press_time[name] = now
                self._reported_long[name] = False
            elif not pressed and was:
                # 下降沿
                dur = now - self._press_time[name]
                if not self._reported_long[name]:
                    if dur >= LONG_PRESS_THRESHOLD:
                        events.add(f"{name}_LONG_PRESSED")
                    else:
                        events.add(f"{name}_PRESSED")
                self._press_time[name] = 0.0
                self._reported_long[name] = False
            elif pressed:
                dur = now - self._press_time[name]
                if dur >= LONG_PRESS_THRESHOLD and not self._reported_long[name]:
                    events.add(f"{name}_LONG_PRESSED")
                    self._reported_long[name] = True
            else:
                events.add(f"{name}_IDLE")
            self._last[name] = pressed

        # trigger / grip 状态
        for tname, val in [
            ("LT", joy.left_trigger),
            ("RT", joy.right_trigger),
            ("LG", joy.left_grip),
            ("RG", joy.right_grip),
        ]:
            if val >= TRIGGER_THRESHOLD:
                events.add(f"{tname}_PRESSED")
            else:
                events.add(f"{tname}_IDLE")

        return events


# ====================== 主节点类 ======================

class PicoDexHandControlNode:
    """
    轻量级灵巧手控制节点。

    仅订阅 /pico/joy，发布 /control_robot_hand_position。
    不占用 UDP 端口，不启动 TCP/UDP 服务器。
    """

    def __init__(self):
        rospy.init_node("pico_dexhand_control", anonymous=False)

        # 配置
        self.cfg = _load_dex_hand_config()
        self.eef_type = rospy.get_param("/end_effector_type", "qiangnao")

        if not (self.eef_type.startswith("qiangnao") or self.eef_type == "lejuclaw"):
            rospy.logwarn(
                f"[DexHandCtrl] end_effector_type='{self.eef_type}', 非 qiangnao/lejuclaw 类型, 节点将空转"
            )

        # 灵巧手抓握状态
        self.filtered_cmd = None  # {left: int, right: int}
        self.last_cmd = None
        self.freeze_finger = False
        self.eef_wait_reengage = False

        # 拇指张开状态
        thumb_cfg = self.cfg.get("thumb_open", {})
        self.thumb_open_enabled = bool(thumb_cfg.get("enabled", True))
        self.thumb_joint_indices = list(thumb_cfg.get("joint_indices", [1]))
        self.thumb_open_value = int(thumb_cfg.get("open_value", 100))
        self.left_thumb_open = False
        self.right_thumb_open = False

        # 按键检测
        self._btn = _ButtonTracker()

        # ROS pub/sub
        if self.eef_type.startswith("qiangnao"):
            self.pub_hand = rospy.Publisher(
                "/control_robot_hand_position", robotHandPosition, queue_size=10
            )
        elif self.eef_type == "lejuclaw":
            self.pub_claw = rospy.Publisher(
                "/leju_claw_command", lejuClawCommand, queue_size=10
            )
        rospy.Subscriber("/pico/joy", JoySticks, self._joy_cb)

        rospy.loginfo("[DexHandCtrl] Node started — listening on /pico/joy")
        rospy.loginfo(f"[DexHandCtrl] eef_type={self.eef_type}, config={self.cfg}")

    # -------------------- joy 回调 --------------------

    def _joy_cb(self, joy: JoySticks):
        events = self._btn.update(joy)

        # 处理组合按键（优先级：RG/LG 组合 > 单键）
        has_rg_lg = any("RG" in e or "LG" in e for e in events if "PRESSED" in e)

        # Y 按键切换冻结手指
        if "Y_PRESSED" in events and "RG_PRESSED" not in events and "RT_PRESSED" not in events:
            self.freeze_finger = not self.freeze_finger
            if self.freeze_finger:
                # 冻结时锁定当前值
                if self.last_cmd is not None and self.filtered_cmd is not None:
                    self.filtered_cmd = dict(self.last_cmd)
                self.eef_wait_reengage = False
                rospy.loginfo("[DexHandCtrl] 手指锁定")
            else:
                stable_cfg = self.cfg.get("stable_unlock", {})
                self.eef_wait_reengage = bool(stable_cfg.get("enabled", True))
                rospy.loginfo("[DexHandCtrl] 手指解锁")

        # X 长按 → 左拇指张开切换
        if "X_LONG_PRESSED" in events and "LT_IDLE" in events and "RT_IDLE" in events:
            if self.thumb_open_enabled:
                self.left_thumb_open = not self.left_thumb_open
                state = "开启" if self.left_thumb_open else "关闭"
                rospy.loginfo(f"[DexHandCtrl] 左拇指张开{state}")

        # A 长按 → 右拇指张开切换
        if "A_LONG_PRESSED" in events and "LT_IDLE" in events and "RT_IDLE" in events:
            if self.thumb_open_enabled:
                self.right_thumb_open = not self.right_thumb_open
                state = "开启" if self.right_thumb_open else "关闭"
                rospy.loginfo(f"[DexHandCtrl] 右拇指张开{state}")

        # 计算 eef_freeze 状态
        eef_freeze = self.freeze_finger

        # 发布灵巧手控制
        self._publish_hand(joy, eef_freeze)

    # -------------------- 灵巧手发布 --------------------

    def _publish_hand(self, joy: JoySticks, eef_freeze: bool):
        if not (self.eef_type.startswith("qiangnao") or self.eef_type == "lejuclaw"):
            return

        cmd_min = int(self.cfg.get("command_min", 0))
        cmd_max = int(self.cfg.get("command_max", 100))
        if cmd_min > cmd_max:
            cmd_min, cmd_max = cmd_max, cmd_min
        cmd_span = max(1, cmd_max - cmd_min)

        grip_deadzone = float(self.cfg.get("grip_deadzone", 0.02))
        smoothing_alpha = float(self.cfg.get("smoothing_alpha", 0.35))
        smoothing_alpha = max(0.0, min(1.0, smoothing_alpha))
        reengage_threshold = int(
            self.cfg.get("stable_unlock", {}).get("reengage_threshold", 8)
        )

        def _grip_to_cmd(grip: float) -> int:
            g = max(0.0, min(1.0, float(grip)))
            if g < grip_deadzone:
                g = 0.0
            return int(round(cmd_min + g * cmd_span))

        raw = {
            "left": _grip_to_cmd(joy.left_grip),
            "right": _grip_to_cmd(joy.right_grip),
        }

        # 一阶低通平滑
        if self.filtered_cmd is None:
            self.filtered_cmd = dict(raw)
        else:
            self.filtered_cmd = {
                "left": int(
                    round(
                        (1.0 - smoothing_alpha) * self.filtered_cmd["left"]
                        + smoothing_alpha * raw["left"]
                    )
                ),
                "right": int(
                    round(
                        (1.0 - smoothing_alpha) * self.filtered_cmd["right"]
                        + smoothing_alpha * raw["right"]
                    )
                ),
            }

        if self.last_cmd is None:
            self.last_cmd = dict(self.filtered_cmd)

        # 锁定 / 回切 / 正常
        if eef_freeze:
            cmd = dict(self.last_cmd)
        elif self.eef_wait_reengage:
            left_diff = abs(self.filtered_cmd["left"] - self.last_cmd["left"])
            right_diff = abs(self.filtered_cmd["right"] - self.last_cmd["right"])
            if left_diff <= reengage_threshold and right_diff <= reengage_threshold:
                self.eef_wait_reengage = False
                self.last_cmd = dict(self.filtered_cmd)
                cmd = dict(self.filtered_cmd)
                rospy.loginfo("[DexHandCtrl] 灵巧手解锁完成：抓握值已平滑回切")
            else:
                cmd = dict(self.last_cmd)
        else:
            self.last_cmd = dict(self.filtered_cmd)
            cmd = dict(self.filtered_cmd)

        # 根据末端类型构造并发布消息
        if self.eef_type.startswith("qiangnao"):
            msg = robotHandPosition()
            msg.header.stamp = rospy.Time.now()
            msg.header.frame_id = "pico_dexhand_control"
            left_pos = [cmd["left"]] * 6
            right_pos = [cmd["right"]] * 6

            # 拇指张开覆盖
            if self.thumb_open_enabled:
                open_val = max(cmd_min, min(cmd_max, self.thumb_open_value))
                for idx in self.thumb_joint_indices:
                    if isinstance(idx, int) and 0 <= idx < 6:
                        if self.left_thumb_open:
                            left_pos[idx] = open_val
                        if self.right_thumb_open:
                            right_pos[idx] = open_val

            msg.left_hand_position = left_pos
            msg.right_hand_position = right_pos
            self.pub_hand.publish(msg)
        elif self.eef_type == "lejuclaw":
            msg = lejuClawCommand()
            msg.header.stamp = rospy.Time.now()
            msg.data.name = ['left_claw', 'right_claw']
            msg.data.position = [float(cmd["left"]), float(cmd["right"])]
            msg.data.velocity = [90.0, 90.0]  # 速度默认 90%
            self.pub_claw.publish(msg)


# ========================== main ==========================

def main():
    try:
        node = PicoDexHandControlNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


if __name__ == "__main__":
    main()
