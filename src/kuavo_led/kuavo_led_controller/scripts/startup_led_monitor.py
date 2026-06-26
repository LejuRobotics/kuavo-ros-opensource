#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
启动 LED 状态监控节点

监听机器人启动流程中的关键事件，驱动 LED 灯带显示状态：
  - 启动中（绿灯闪烁）：/hardware/ready_to_start==0 时触发
  - 可以按'o'(绿灯常亮): /hardware/ready_to_start==1 时触发（关节已移动到准备姿态）
  - 启动完成（白灯常亮）：
      双足(robot_type==2): /bot_stand_up_complete topic data==1 时触发
      轮臂(robot_type==1): /hardware/is_ready==1 时触发（按'o'后直接切白灯）
  - 跌倒（红灯闪烁）：/humanoid_controller/fall_down_state_ topic data==1 时触发

使用 led_state 统一管理状态，避免重复发送 LED 命令。
仅兼容新系统(24 灯,led_strip_service)。
"""

import rospy
from std_msgs.msg import Int8, Float64
from kuavo_msgs.srv import SetLEDMode_free, SetLEDMode_freeRequest
from kuavo_msgs.msg import Color


LED_SERVICE = '/led_strip_set_mode_and_color'
LED_COUNT = 24

# ==================== 颜色常量 ====================
# 修改这里即可更换各阶段的 LED 颜色
# 格式: (R, G, B)，各分量范围 0-255

# --- 预留颜色，方便扩展 ---
BLACK = (0, 0, 0)                # 黑色（关灯）
RED = (255, 0, 0)
ORANGE = (255, 128, 0)
YELLOW = (255, 255, 0)
GREEN = (0, 255, 0)
CYAN = (0, 255, 255)
BLUE = (0, 0, 255)
PURPLE = (128, 0, 255)
PINK = (255, 0, 128)
WHITE = (255, 255, 255)

# --- 启动阶段 ---
COLOR_BOOTING = GREEN      # 启动中：绿色

# --- 站立阶段 ---
COLOR_READY = GREEN        # 准备站立：绿色
COLOR_STANDING = WHITE     # 站立完成：白色

# --- 异常状态 ---
COLOR_FALL = RED         # 跌倒：红色
COLOR_ERROR = RED        # 通用错误：红色

# ==================== LED 模式常量 ====================
# 修改这里即可更换各阶段的 LED 模式
MODE_CONSTANT = 0    # 常亮
MODE_BREATHING = 1   # 呼吸
MODE_FLASH = 2       # 闪烁
MODE_RHYTHM = 3      # 律动


def set_led(mode, color_rgb):
    """
    设置 LED 灯带颜色和模式（新系统 24 灯）

    Args:
        mode: 0=常亮, 1=呼吸, 2=闪烁, 3=律动
        color_rgb: (R, G, B) 元组，应用到全部 24 颗灯
    """
    try:
        rospy.wait_for_service(LED_SERVICE, timeout=3.0)
        srv = rospy.ServiceProxy(LED_SERVICE, SetLEDMode_free)
        req = SetLEDMode_freeRequest()
        req.mode = mode
        req.colors = []
        for _ in range(LED_COUNT):
            c = Color()
            c.r = color_rgb[0]
            c.g = color_rgb[1]
            c.b = color_rgb[2]
            req.colors.append(c)
        resp = srv(req)
        if resp.success:
            rospy.loginfo("[StartupLED] LED 设置成功: mode=%d, color=%s", mode, color_rgb)
        else:
            rospy.logwarn("[StartupLED] LED 设置失败: mode=%d, color=%s", mode, color_rgb)
    except rospy.ServiceException as e:
        rospy.logerr("[StartupLED] LED 服务调用失败: %s", e)
    except rospy.ROSException as e:
        rospy.logerr("[StartupLED] LED 服务不可用: %s", e)


class StartupLEDMonitor:
    """启动 LED 状态监控"""

    def __init__(self):
        rospy.init_node('startup_led_monitor', anonymous=True)

        # 当前 LED 状态: None(初始) / 'booting' / 'ready' / 'standing' / 'fall'
        self.led_state = None

        # 机器人类型：1=轮臂, 2=双足（由 launch 文件设置）
        self.robot_type = rospy.get_param('/robot_type', 2)
        rospy.loginfo("[StartupLED] 机器人类型: %s", "轮臂" if self.robot_type == 1 else "双足")

        # 订阅站立完成 topic（仅双足会发布）
        rospy.Subscriber('/bot_stand_up_complete', Int8, self._stand_complete_cb)

        # 订阅跌倒状态 topic（Float64: 0=STANDING, 1=FALL_DOWN）
        rospy.Subscriber(
            '/humanoid_controller/fall_down_state_',
            Float64,
            self._fall_state_cb
        )

        # 定时器（2Hz）：检测启动中/准备站立/启动完成
        self._poll_timer = rospy.Timer(rospy.Duration(0.5), self._poll_state)

        rospy.loginfo("[StartupLED] 启动 LED 监控节点已启动，等待状态变化...")

    def _set_state(self, state, mode, color_rgb):
        """切换 LED 状态，相同状态不重复发送"""
        if self.led_state == state:
            return
        set_led(mode, color_rgb)
        self.led_state = state

    def _poll_state(self, event):
        """定时轮询：检测启动中/准备站立/启动完成"""
        # 跌倒或站立完成后，定时器不再干预（由跌倒回调管理异常状态）
        if self.led_state in ('fall', 'standing'):
            return

        ready = rospy.get_param('/hardware/ready_to_start', 0)

        # 轮臂：用 /hardware/is_ready==1 判断启动完成（轮臂不发布 /bot_stand_up_complete）
        if self.robot_type == 1:  # 轮臂
            is_ready = rospy.get_param('/hardware/is_ready', 0)
            if is_ready == 1:
                # 按'o'后，启动完成，白灯常亮
                self._set_state('standing', MODE_CONSTANT, COLOR_STANDING)
                return

        # ready_to_start==1：关节已移动到准备姿态，绿灯常亮
        if ready == 1:
            self._set_state('ready', MODE_CONSTANT, COLOR_READY)
        # 都不满足：启动中，绿灯闪烁
        else:
            self._set_state('booting', MODE_FLASH, COLOR_BOOTING)

    def _stand_complete_cb(self, msg):
        """站立完成回调"""
        if msg.data == 1:
            self._set_state('standing', MODE_CONSTANT, COLOR_STANDING)

    def _fall_state_cb(self, msg):
        """跌倒状态回调（0=站立, 1=跌倒）"""
        if int(msg.data) == 1:
            self._set_state('fall', MODE_FLASH, COLOR_FALL)

    def spin(self):
        rospy.spin()


if __name__ == '__main__':
    try:
        monitor = StartupLEDMonitor()
        monitor.spin()
    except rospy.ROSInterruptException:
        pass
