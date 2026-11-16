#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""物理急停桥接节点

订阅电源板状态 /power_board_status，检测 stop_int（急停中断）置位后发布
/stop_robot，把物理急停按钮接入标准软件急停链路，停止控制器、硬件与 rosbag 录制。
运行期只打按下锚点（按下即退出，等不到释放），退出时补打状态快照供事后分析。
"""

import datetime
import threading
import time

import rospy
from std_msgs.msg import Bool
from kuavo_msgs.msg import PowerBoardStatus

# 采样新鲜度门限。以帧内 timestamp（真实采样时刻）为准，而非消息到达时刻：
# 上游串口失联后会继续重发同一份缓存，到达时刻永远是"刚刚"，无法据此判断。
# 系统状态约 1s 刷新一次，3s 可容忍偶发查询失败与 LED 抢锁。
SAMPLE_STALE_SECONDS = 3.0
# 5 代机本话题有两个发布者共用同一缓存，在途旧帧可能后到，退出快照需确认释放
RELEASE_CONFIRM_SAMPLES = 2


def _now_str():
    """本地墙钟时间，日志锚点用"""
    return datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S')


class PhysicalEstopBridge:
    """把电源板物理急停标志转成 /stop_robot 软件急停事件"""

    def __init__(self):
        rospy.init_node('physical_estop_bridge_node', anonymous=False)
        self._stop_robot_pub = rospy.Publisher('/stop_robot', Bool, queue_size=10)
        # 仅在见过 stop_int==0 之后才允许触发，避免开机瞬态误报急停
        self._armed = False
        self._reported = False
        # 最近一次急停状态缓存，供退出快照使用；None 表示从未收到
        self._last_stop_int = None
        self._last_sample_time = None
        # 日志与快照共用的已确认状态；与触发用的 _reported 分离
        self._confirmed_pressed = None
        # 连续收到的正常帧计数，用于过滤在途旧帧造成的假释放
        self._release_samples = 0
        self._lock = threading.Lock()
        rospy.Subscriber('/power_board_status', PowerBoardStatus, self._on_status, queue_size=10)
        rospy.on_shutdown(self._print_exit_snapshot)
        rospy.loginfo("[ESTOP] %s estop_state=INIT (bridge started, watching power board stop_int)",
                      _now_str())

    def _on_status(self, msg):
        """stop_int 上升沿发布 /stop_robot 并打印 PRESSED 锚点"""
        pressed = bool(msg.stop_int)
        # 0 表示发布端未填采样时刻（旧版本），按未知新鲜度处理
        sample_time = msg.timestamp.to_sec()
        publish = False
        log_pressed = False
        with self._lock:
            # 记录最后收到的缓存值与采样时刻，供退出快照判断新鲜度
            self._last_stop_int = pressed
            self._last_sample_time = sample_time if sample_time > 0 else None
            # 发布沿用原始上升沿语义；释放确认只用于退出快照与抑制重复锚点
            self._release_samples = 0 if pressed else self._release_samples + 1
            if not pressed:
                self._armed = True   # 出现过正常态，后续上升沿才可信
                self._reported = False
                # 释放不打锚点：按下后运控即退出，节点活不到释放时刻
                if self._confirmed_pressed and self._release_samples >= RELEASE_CONFIRM_SAMPLES:
                    self._confirmed_pressed = False
            elif self._armed and not self._reported:
                self._reported = True
                publish = True
                # 释放未获确认的重复按下只补发布，不重复打印锚点
                if not self._confirmed_pressed:
                    self._confirmed_pressed = True
                    log_pressed = True

        if log_pressed:
            rospy.logerr("[ESTOP] %s estop_state=PRESSED (Physical e-stop detected, "
                         "power board stop_int, publish /stop_robot)", _now_str())
        if publish:
            self._publish_stop_robot()

    def _print_exit_snapshot(self):
        """退出快照：只读内存缓存，不访问串口，不阻塞退出"""
        try:
            with self._lock:
                pressed = self._last_stop_int
                confirmed = self._confirmed_pressed
                sample_time = self._last_sample_time
            # 优先用已确认状态，避免把未确认的在途释放帧写进快照
            if confirmed is not None:
                pressed = confirmed
            if pressed is None or sample_time is None:
                state, reason = 'UNKNOWN', 'no_data'
            else:
                state_name = 'PRESSED' if pressed else 'RELEASED'
                age = time.time() - sample_time
                # 采样过旧只说明数据不可信，不改变"最后收到的缓存值"这一事实
                if age > SAMPLE_STALE_SECONDS:
                    rospy.logwarn(
                        "[ESTOP] %s exit snapshot estop_state=UNKNOWN reason=stale "
                        "last_cached_state=%s sample_age=%.1fs "
                        "(last received cache, freshness unknown)",
                        _now_str(), state_name, age)
                    return
                state, reason = state_name, 'ok'
            rospy.logwarn("[ESTOP] %s exit snapshot estop_state=%s reason=%s",
                          _now_str(), state, reason)
        except Exception as e:
            rospy.logwarn("[ESTOP] exit snapshot failed: %s", e)

    def _publish_stop_robot(self, times=5):
        """连发数次，确保各订阅方在自身退出前都能收到急停"""
        for _ in range(times):
            self._stop_robot_pub.publish(Bool(data=True))


if __name__ == '__main__':
    try:
        PhysicalEstopBridge()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
