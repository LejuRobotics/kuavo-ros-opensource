#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""拳师五指空间数据识别 Quest3 虚拟按键，以及双手握拳一次性启动。

每手 Float32MultiArray 长度 35；thumb/index/middle/ring/pinky 每指
[x,y,z,qw,qx,qy,qz]，位置单位米（传感器模块位姿，不是人体指尖 FK）。

位定义：
  bit0 = 1   左手拇指+食指捏合，稳定 press_stable_sec  → X
  bit1 = 2   左手拇指+中指捏合                          → Y
  bit2 = 4   右手拇指+食指捏合                          → A
  bit3 = 8   右手拇指+中指捏合                          → B
  bit4 = 16  双手各自拇指+小指捏合，保持 mode_hold_sec  → 交互模式切换，电平保持 0.5s
  bit5 = 32  双手握拳，稳定 press_stable_sec            → 交互模式内只启动一次，并退出交互模式

同一只手拇指同时捏住食指和中指时，两个位都保持，用来组成 X+Y 关机。
双手握拳或双手拇指+小指期间，不再输出该周期的 X/Y/A/B。
默认使用 launch 阈值；通过个人标定后在启动时加载左右手独立阈值。
"""
from __future__ import annotations

import argparse
import json
import math
import os
import sys
import threading
import time
from typing import Dict, Optional, Sequence

# Catkin's devel-space relay executes this source file, while install space
# places both scripts in the package bin directory.  Import the sibling helper
# explicitly in either layout instead of relying on PYTHONPATH side effects.
_SCRIPT_DIR = os.path.dirname(os.path.realpath(__file__))
if _SCRIPT_DIR not in sys.path:
    sys.path.insert(0, _SCRIPT_DIR)

try:
    import rospy
    from std_msgs.msg import Bool, Float32MultiArray, String, UInt8
except ImportError:
    rospy = None
    Bool = Float32MultiArray = String = UInt8 = None

from gesture_calibration import DEFAULT_PROFILE_PATH, input_contract, load_profile

LEFT, RIGHT = "L", "R"
SIDES = (LEFT, RIGHT)
TOPICS = {LEFT: "/qianshi/glove_left/fingertips",
          RIGHT: "/qianshi/glove_right/fingertips"}
GROUPS = ("thumb_index", "thumb_middle", "thumb_little")
BITS = {LEFT: (0, 1), RIGHT: (2, 3)}
FIST_BIT = 5
MODE_BIT = 4
SIDE_NAMES = {LEFT: "left", RIGHT: "right"}


def positions(data: Sequence[float]):
    if len(data) != 35 or not all(math.isfinite(float(v)) for v in data):
        raise ValueError("expected 35 finite fingertip values")
    return [tuple(float(data[i * 7 + k]) for k in range(3)) for i in range(5)]


def hand_distances(points):
    """拇指到食指、中指、小指的距离。"""
    return tuple(math.dist(points[0], points[i]) for i in (1, 2, 4))


def fist_span(points):
    """食指/中指/无名指/小指之间最大空间距离。"""
    return max(math.dist(points[i], points[j]) for i in range(1, 5)
               for j in range(i + 1, 5))


def fallback_thresholds(press, release, fist_press, fist_release):
    """Convert legacy shared thresholds to the per-side runtime contract."""
    return {
        side: {
            **{key: {"press": float(press[key]), "release": float(release[key])}
               for key in GROUPS},
            "fist": {"press": float(fist_press), "release": float(fist_release)},
        }
        for side in SIDE_NAMES.values()
    }


def resolve_calibration(get_param, legacy_thresholds, profile_loader=None):
    """Resolve one startup-only profile, with fail-closed explicit selection."""
    enabled = bool(get_param("~calibration_enabled", True))
    requested = str(get_param("~calibration_file", DEFAULT_PROFILE_PATH))
    backend = str(get_param("~calibration_backend", "linux_sdk"))
    left_map = get_param("~calibration_left_module_map", [])
    right_map = get_param("~calibration_right_module_map", [])
    left_id = get_param("~calibration_left_id", 12)
    right_id = get_param("~calibration_right_id", 2)
    expected = input_contract(backend, left_map, right_map, left_id, right_id)
    status = {"enabled": enabled, "profile_loaded": False,
              "profile_name": None, "source": "launch_parameters",
              "requested_file": os.path.abspath(os.path.expanduser(requested)),
              "input": expected, "reason": "calibration_disabled" if not enabled else None,
              "thresholds": legacy_thresholds}
    if not enabled:
        return legacy_thresholds, status

    requested_path = status["requested_file"]
    default_path = os.path.abspath(os.path.expanduser(DEFAULT_PROFILE_PATH))
    if profile_loader is None:
        profile_loader = load_profile
    profile = profile_loader(requested_path, expected_input=expected,
                             optional=requested_path == default_path)
    if profile is None:
        status["reason"] = "default_profile_missing"
        return legacy_thresholds, status
    status.update({"profile_loaded": True, "profile_name": profile["profile_name"],
                   "source": requested_path, "reason": None,
                   "thresholds": profile["thresholds"]})
    return profile["thresholds"], status


def windows_fist_pinch_suppression(get_param):
    """Read the opt-in only for the dedicated Windows glove entry point."""
    return (os.environ.get("KUAVO_WINDOWS_GLOVE_ENTRY") == "1"
            and bool(get_param("~suppress_fist_pinches", False)))


class GloveGestureLogic:
    def __init__(self, press, release, mode_hold_sec=2.0, data_timeout=0.5,
                 press_stable_sec=0.1, fist_distance_threshold=0.065,
                 fist_release_threshold=0.085, side_thresholds=None,
                 suppress_fist_pinches=False):
        if any(not 0 < press[k] < release[k] for k in GROUPS):
            raise ValueError("pinch thresholds require 0 < press < release")
        if not 0 < fist_distance_threshold < fist_release_threshold:
            raise ValueError("fist thresholds require 0 < press < release")
        self.press, self.release = press, release
        self.mode_hold_sec, self.data_timeout = mode_hold_sec, data_timeout
        self.press_stable_sec = press_stable_sec
        self.fist_distance_threshold = fist_distance_threshold
        self.fist_release_threshold = fist_release_threshold
        self.suppress_fist_pinches = bool(suppress_fist_pinches)
        if side_thresholds is None:
            side_thresholds = {
                name: {
                    **{key: {"press": press[key], "release": release[key]}
                       for key in GROUPS},
                    "fist": {"press": fist_distance_threshold,
                             "release": fist_release_threshold},
                }
                for name in SIDE_NAMES.values()
            }
        self.side_thresholds = self._validated_side_thresholds(side_thresholds)
        self.hold = {s: {k: False for k in GROUPS} for s in SIDES}
        self.since = {s: {k: None for k in GROUPS} for s in SIDES}
        self.last_rx: Dict[str, Optional[float]] = {s: None for s in SIDES}
        self.fist = {s: False for s in SIDES}
        self.mode_start = self.fist_start = None
        self.mode_pulse_until = None
        self.mode_pulse_sec = 0.5
        self.mode_latched = self.fist_latched = False

    @staticmethod
    def _validated_side_thresholds(values):
        result = {}
        for side in SIDE_NAMES.values():
            side_values = values.get(side) if isinstance(values, dict) else None
            if not isinstance(side_values, dict):
                raise ValueError("gesture thresholds require left and right mappings")
            result[side] = {}
            for key in GROUPS + ("fist",):
                pair = side_values.get(key)
                if not isinstance(pair, dict):
                    raise ValueError("missing %s %s gesture thresholds" % (side, key))
                try:
                    press_value = float(pair["press"])
                    release_value = float(pair["release"])
                except (KeyError, TypeError, ValueError) as exc:
                    raise ValueError("invalid %s %s gesture thresholds" % (side, key)) from exc
                if (not math.isfinite(press_value) or not math.isfinite(release_value)
                        or not 0 < press_value < release_value <= .25):
                    raise ValueError(
                        "%s %s thresholds require finite 0 < press < release <= 0.25"
                        % (side, key))
                result[side][key] = {"press": press_value, "release": release_value}
        return result

    def threshold(self, side, key, state):
        return self.side_thresholds[SIDE_NAMES[side]][key][state]

    @staticmethod
    def hysteresis(previous, value, press, release):
        return value < release if previous else value <= press

    def update_hand(self, side, distances, span, now):
        if len(distances) != 3 or not all(math.isfinite(d) for d in distances) or not math.isfinite(span):
            return
        self.last_rx[side] = now
        for key, distance in zip(GROUPS, distances):
            press = self.threshold(side, key, "press")
            release = self.threshold(side, key, "release")
            held = self.hysteresis(self.hold[side][key], distance,
                                   press, release)
            if held and not self.hold[side][key]:
                self.since[side][key] = now
            elif not held:
                self.since[side][key] = None
            self.hold[side][key] = held
        self.fist[side] = self.hysteresis(
            self.fist[side], span, self.threshold(side, "fist", "press"),
            self.threshold(side, "fist", "release"))

    def _pinch_bits(self, side, now):
        bits = 0
        for key, bit in zip(GROUPS[:2], BITS[side]):
            since = self.since[side][key]
            if self.hold[side][key] and since is not None and now - since >= self.press_stable_sec:
                bits |= 1 << bit
        return bits

    def poll(self, now):
        """返回本周期按钮位。

        捏合在保持期间持续置位。bit5 只输出一个周期。
        bit4 在触发后保持 mode_pulse_sec，避免 FSM 队列深度为 1 时丢掉上升沿。
        断流的手不参与。单手握拳、单手拇指+小指不产生双手手势。
        握拳重新武装要双手都松开；交互模式重新武装只要有一只手松开拇指+小指。
        """
        stale = {s: self.last_rx[s] is None or now - self.last_rx[s] > self.data_timeout
                 for s in SIDES}
        for side in SIDES:
            if stale[side]:
                self.hold[side] = {k: False for k in GROUPS}
                self.since[side] = {k: None for k in GROUPS}
                self.fist[side] = False
        valid = not stale[LEFT] and not stale[RIGHT]
        both_fist = valid and self.fist[LEFT] and self.fist[RIGHT]
        both_little = (valid and not both_fist and self.hold[LEFT]["thumb_little"]
                       and self.hold[RIGHT]["thumb_little"])
        # 拇指+小指只在“正在进入交互模式”时挡住 X/Y/A/B，避免同一次捏合误触发关机或切模式。
        # 脉冲结束后即使小指距离还没松开，也要允许 X+A。
        mode_pulse_active = self.mode_pulse_until is not None and now < self.mode_pulse_until
        block_pinch = both_fist or (both_little and (not self.mode_latched or mode_pulse_active))
        buttons = 0
        for side in SIDES:
            if (stale[side] or block_pinch
                    or (self.suppress_fist_pinches and self.fist[side])):
                continue
            buttons |= self._pinch_bits(side, now)

        if not valid:
            self.mode_start = self.fist_start = None
        if both_fist:
            self.mode_start = None
            if self.fist_start is None:
                self.fist_start = now
            elif not self.fist_latched and now - self.fist_start >= self.press_stable_sec:
                buttons = 1 << FIST_BIT
                self.fist_latched = True
        else:
            self.fist_start = None
            if self.fist_latched and valid and all(not self.fist[s] for s in SIDES):
                self.fist_latched = False
        if both_little and not self.fist_latched:
            if self.mode_start is None:
                self.mode_start = now
            elif not self.mode_latched and now - self.mode_start >= self.mode_hold_sec:
                self.mode_latched = True
                self.mode_pulse_until = now + self.mode_pulse_sec
        else:
            self.mode_start = None
        if self.mode_pulse_until is not None:
            if now < self.mode_pulse_until:
                buttons |= 1 << MODE_BIT
            else:
                self.mode_pulse_until = None
        if self.mode_latched and any(not stale[s] and not self.hold[s]["thumb_little"] for s in SIDES):
            self.mode_latched = False
        return buttons


class GloveGestureNode:
    def __init__(self, args):
        press = {k: args.press_threshold for k in GROUPS}
        release = {k: args.release_threshold for k in GROUPS}
        for key in GROUPS:
            for prefix, values in (("press", press), ("release", release)):
                override = getattr(args, prefix + "_" + key)
                if override is not None:
                    values[key] = override
        legacy_thresholds = fallback_thresholds(
            press, release, args.fist_distance_threshold, args.fist_release_threshold)
        side_thresholds, calibration_status = resolve_calibration(
            rospy.get_param, legacy_thresholds)
        suppress_fist_pinches = windows_fist_pinch_suppression(rospy.get_param)
        self.logic = GloveGestureLogic(press, release, args.mode_hold_sec, args.data_timeout,
                                       args.press_stable_sec, args.fist_distance_threshold,
                                       args.fist_release_threshold, side_thresholds,
                                       suppress_fist_pinches=suppress_fist_pinches)
        self.hz = args.hz
        self._lock = threading.Lock()
        self._state_lock = threading.RLock()
        self._pending = {s: None for s in SIDES}
        self._distances = {s: [float("nan")] * 3 for s in SIDES}
        self._spans = {s: float("nan") for s in SIDES}
        self._last_valid_frame = {s: None for s in SIDES}
        self._interaction = False
        self._mode_bit_prev = False
        self._announced = set()
        self.buttons_pub = rospy.Publisher("/glove_gesture/buttons", UInt8, queue_size=1)
        self.interaction_pub = rospy.Publisher(
            "/glove_gesture/interaction_mode", Bool, queue_size=1, latch=True)
        self.interaction_pub.publish(Bool(data=False))
        self.calibration_pub = rospy.Publisher(
            "/glove_gesture/calibration_status", String, queue_size=1, latch=True)
        self.calibration_pub.publish(String(data=json.dumps(
            calibration_status, ensure_ascii=False, sort_keys=True)))
        if calibration_status["profile_loaded"]:
            rospy.loginfo("[glove_gesture] 已加载个人手势标定 %s: %s",
                          calibration_status["profile_name"], calibration_status["source"])
        elif calibration_status["reason"] == "default_profile_missing":
            rospy.logwarn("[glove_gesture] 默认个人标定不存在，使用 launch 距离阈值: %s",
                          calibration_status["requested_file"])
        else:
            rospy.logwarn("[glove_gesture] 个人标定已关闭，使用 launch 距离阈值")
        self.debug_pub = rospy.Publisher("/glove_gesture/debug/distances", Float32MultiArray,
                                         queue_size=1)
        self.subs = [rospy.Subscriber(TOPICS[s], Float32MultiArray,
                                      lambda msg, side=s: self.on_fingertips(side, msg), queue_size=5)
                     for s in SIDES]
        self.interaction_state_sub = rospy.Subscriber(
            "/glove_gesture/fsm_interaction_mode", Bool,
            self.on_fsm_interaction_mode, queue_size=1)

    def _set_interaction_locked(self, enabled, reason=""):
        if enabled == self._interaction:
            return False
        self._interaction = enabled
        self.interaction_pub.publish(Bool(data=enabled))
        color = "\033[32m" if enabled else "\033[33m"
        state = "开启" if enabled else "关闭"
        suffix = "（%s）" % reason if reason else ""
        rospy.loginfo("%s[glove_gesture] 手套交互模式 %s%s\033[0m",
                      color, state, suffix)
        self._announced = set()
        return True

    def on_fsm_interaction_mode(self, msg):
        """跟随 FSM 权威状态；首次握拳失败时 FSM 不退出，手套端也保持可重试。"""
        with self._state_lock:
            self._set_interaction_locked(bool(msg.data), "FSM同步")

    def _announce(self, buttons):
        with self._state_lock:
            self._announce_locked(buttons)

    def _announce_locked(self, buttons):
        """在本终端打印交互模式和已生效的手势映射。未进交互模式时不报 X/Y/A/B。"""
        mode_now = bool(buttons & (1 << MODE_BIT))
        # FSM owns the state transition. buttons is published before this method
        # runs, so its latched acknowledgement may already have arrived; toggling
        # locally here would then undo the authoritative state.
        self._mode_bit_prev = mode_now
        if not self._interaction:
            return
        chords = (
            ((1 << BITS[LEFT][0]) | (1 << BITS[RIGHT][0]), "X+A", (1 << BITS[LEFT][0]) | (1 << BITS[RIGHT][0])),
            ((1 << BITS[LEFT][0]) | (1 << BITS[LEFT][1]), "X+Y", (1 << BITS[LEFT][0]) | (1 << BITS[LEFT][1])),
            ((1 << BITS[LEFT][0]) | (1 << BITS[RIGHT][1]), "X+B", (1 << BITS[LEFT][0]) | (1 << BITS[RIGHT][1])),
        )
        singles = (
            (1 << BITS[LEFT][0], "X"),
            (1 << BITS[LEFT][1], "Y"),
            (1 << BITS[RIGHT][0], "A"),
            (1 << BITS[RIGHT][1], "B"),
            (1 << FIST_BIT, "双手握拳"),
        )
        active = set()
        covered = 0
        for mask, name, bits in chords:
            if (buttons & mask) == mask:
                active.add(name)
                covered |= bits
        for bit, name in singles:
            if buttons & bit and not (bit & covered):
                active.add(name)
        for name in ("X+A", "X+Y", "X+B", "X", "Y", "A", "B", "双手握拳"):
            if name in active and name not in self._announced:
                rospy.loginfo("[glove_gesture] 检测到 %s", name)
        self._announced = active

    def on_fingertips(self, side, msg):
        with self._lock:
            self._pending[side] = list(msg.data)

    def spin(self):
        rate = rospy.Rate(self.hz)
        while not rospy.is_shutdown():
            now = time.monotonic()
            for side in SIDES:
                with self._lock:
                    pending, self._pending[side] = self._pending[side], None
                if pending is None:
                    continue
                try:
                    points = positions(pending)
                except (ValueError, TypeError):
                    rospy.logwarn_throttle(2.0, "[glove_gesture] invalid %s fingertip frame", side)
                    continue
                distances = hand_distances(points)
                span = fist_span(points)
                self._distances[side] = list(distances)
                self._spans[side] = span
                self._last_valid_frame[side] = now
                self.logic.update_hand(side, distances, span, now)
            buttons = self.logic.poll(now)
            self.buttons_pub.publish(UInt8(data=buttons))
            self._announce(buttons)
            left_little, right_little = self._distances[LEFT][2], self._distances[RIGHT][2]
            left_need = self.logic.threshold(LEFT, "thumb_little", "press")
            right_need = self.logic.threshold(RIGHT, "thumb_little", "press")
            fresh = all(self._last_valid_frame[s] is not None
                        and now - self._last_valid_frame[s] <= self.logic.data_timeout
                        for s in SIDES)
            if (fresh and math.isfinite(left_little) and math.isfinite(right_little)
                    and left_little < left_need + 0.015
                    and right_little < right_need + 0.015
                    and not (buttons & (1 << MODE_BIT))):
                if self.logic.mode_start is None:
                    rospy.loginfo_throttle(
                        2.0,
                        "[glove_gesture] 拇指-小指距离 左 %.3fm/阈值 %.3fm，右 %.3fm/阈值 %.3fm，需同时保持 %.1f 秒",
                        left_little, left_need, right_little, right_need,
                        self.logic.mode_hold_sec)
                elif not self.logic.mode_latched:
                    held = now - self.logic.mode_start
                    rospy.loginfo_throttle(
                        1.0,
                        "[glove_gesture] 拇指+小指已捏住，已保持 %.1f/%.1f 秒",
                        held, self.logic.mode_hold_sec)
            self.debug_pub.publish(Float32MultiArray(
                data=self._distances[LEFT] + self._distances[RIGHT]
                + [self._spans[LEFT], self._spans[RIGHT]]))
            rate.sleep()


def build_arg_parser():
    parser = argparse.ArgumentParser(description="拳师五指手势（阈值待实机标定）")
    for flag, default in (("hz", 50.0), ("mode-hold-sec", 2.0),
                          ("data-timeout", 0.5), ("press-stable-sec", 0.1),
                          ("press-threshold", 0.020), ("release-threshold", 0.035),
                          ("fist-distance-threshold", 0.065), ("fist-release-threshold", 0.085)):
        parser.add_argument("--" + flag, type=float, default=default)
    for finger in ("thumb-index", "thumb-middle", "thumb-little"):
        parser.add_argument("--press-" + finger, type=float, default=None)
        parser.add_argument("--release-" + finger, type=float, default=None)
    return parser


def main():
    argv = rospy.myargv()[1:] if rospy is not None else sys.argv[1:]
    args = build_arg_parser().parse_args(argv)
    if rospy is None:
        print("[glove_gesture] ROS environment unavailable", file=sys.stderr)
        return 1
    rospy.init_node("glove_gesture_node")
    node = GloveGestureNode(args)
    node.spin()
    return 0


if __name__ == "__main__":
    sys.exit(main())
