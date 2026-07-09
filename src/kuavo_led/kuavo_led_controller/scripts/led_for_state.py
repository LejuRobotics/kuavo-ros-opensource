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
  - 硬件失能（红灯常亮）：/robot_disabled_flag topic data==True 时触发
  - 急停（红灯常亮）：/stop_robot topic data==True 时触发
    （G12 等急停会触发 controllerNodelet→ros::shutdown() 竞态，/robot_disabled_flag
     随 nodelet_manager 关闭发不出，故同时订阅 /stop_robot 兜底——急停指令由独立
     joy 节点发出，在 nodelet_manager 关闭前即可收到。）
  - 电源板/BMS保护（红灯常亮）：/battery_info_1 或 /battery_info_2 protection_flags!=0 时触发
  - 电池拔出/低电量（红灯闪烁）：/power_board_status 中 bat1_exists/bat2_exists 任意为False
    或 bat1_low_power/bat2_low_power 任意为True 时触发

使用 led_state 统一管理状态，避免重复发送 LED 命令。
仅兼容新系统(24 灯,led_strip_service)。
"""

import rospy
from std_msgs.msg import Int8, Bool
from kuavo_msgs.srv import SetLEDMode_free, SetLEDMode_freeRequest
from kuavo_msgs.msg import BatteryInfo, Color, PowerBoardStatus


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

# ==================== 状态配置 ====================
# 各正常状态对应的 (mode, color)，用于电池警告恢复时回到先前状态
STATE_CONFIG = {
    'booting':  (MODE_FLASH, COLOR_BOOTING),
    'ready':    (MODE_CONSTANT, COLOR_READY),
    'standing': (MODE_CONSTANT, COLOR_STANDING),
}


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

        # 当前 LED 状态: None(初始) / 'booting' / 'ready' / 'standing' / 'stop' / 'power'
        self.led_state = None
        # 双足站立完成标志（由 /bot_stand_up_complete 回调更新）
        # _poll_state 据此判断双足是否应恢复 standing（param 里读不到这个状态）
        self._stand_up_complete = False

        # 机器人类型：1=轮臂, 2=双足（由 launch 文件设置）
        self.robot_type = rospy.get_param('/robot_type', 2)
        rospy.loginfo("[StartupLED] 机器人类型: %s", "轮臂" if self.robot_type == 1 else "双足")

        # 订阅站立完成 topic（仅双足会发布）
        rospy.Subscriber('/bot_stand_up_complete', Int8, self._stand_complete_cb)

        # 订阅硬件失能标志 topic（Bool: True=硬件已失能，收到即红灯常亮）
        rospy.Subscriber(
            '/robot_disabled_flag',
            Bool,
            self._robot_disabled_cb
        )

        # 订阅急停指令 topic（Bool: True=急停，收到即红灯常亮）
        # G12 等急停会触发 controllerNodelet→ros::shutdown() 竞态，/robot_disabled_flag
        # 此时随 nodelet_manager 关闭发不出，故同时订阅 /stop_robot 兜底（急停指令由
        # 独立 joy 节点发出，在 nodelet_manager 关闭前即可收到）。
        rospy.Subscriber(
            '/stop_robot',
            Bool,
            self._stop_robot_cb
        )

        # 订阅电源板/BMS保护状态（protection_flags!=0 表示保护触发，红灯常亮）
        rospy.Subscriber('/battery_info_1', BatteryInfo, self._battery_info_cb)
        rospy.Subscriber('/battery_info_2', BatteryInfo, self._battery_info_cb)

        # 订阅电源板系统状态（0x01 系统状态读取）
        # 电池拔出/低电量 → 红灯闪烁
        rospy.Subscriber('/power_board_status', PowerBoardStatus, self._power_board_status_cb)

        # 定时器（2Hz）：检测启动中/准备站立/启动完成
        self._poll_timer = rospy.Timer(rospy.Duration(0.5), self._poll_state)

        rospy.loginfo("[StartupLED] 启动 LED 监控节点已启动，等待状态变化...")

    def _set_state(self, state, mode, color_rgb):
        """切换 LED 状态，相同状态不重复发送

        优先级: stop(失能/急停) > power(电池警告) > 正常状态(booting/ready/standing)
        """
        # 失能(stop)期间，任何其他状态切换都忽略（最高优先级）
        if self.led_state == 'stop' and state != 'stop':
            return
        # 电池警告(power)期间，正常状态切换直接忽略，恢复时由 _detect_normal_state 重新判断
        if self.led_state == 'power' and state in STATE_CONFIG:
            return
        if self.led_state == state:
            return
        set_led(mode, color_rgb)
        self.led_state = state

    def _poll_state(self, event):
        """定时轮询：检测启动中/准备站立/启动完成"""
        # 失能/电源警告/站立完成后，定时器不再干预（由对应回调管理异常状态）
        if self.led_state in ('stop', 'power', 'standing'):
            return

        ready = rospy.get_param('/hardware/ready_to_start', 0)

        # 轮臂：用 /hardware/is_ready==1 判断启动完成（轮臂不发布 /bot_stand_up_complete）
        if self.robot_type == 1:  # 轮臂
            is_ready = rospy.get_param('/hardware/is_ready', 0)
            if is_ready == 1:
                # 按'o'后，启动完成，白灯常亮
                self._set_state('standing', MODE_CONSTANT, COLOR_STANDING)
                return

        # 双足：站立已完成（由 /bot_stand_up_complete 回调记录）→ 白灯常亮
        if self.robot_type == 2 and self._stand_up_complete:
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
            self._stand_up_complete = True
            self._set_state('standing', MODE_CONSTANT, COLOR_STANDING)

    def _robot_disabled_cb(self, msg):
        """硬件失能标志回调（收到 /robot_disabled_flag data==True 即红灯常亮）"""
        if msg.data:
            self._set_state('stop', MODE_CONSTANT, COLOR_FALL)

    def _stop_robot_cb(self, msg):
        """急停回调（收到 /stop_robot data==True 即红灯常亮）

        覆盖 G12 急停等会触发 controllerNodelet→ros::shutdown() 竞态、
        导致 /robot_disabled_flag 发不出的场景。
        """
        if msg.data:
            self._set_state('stop', MODE_CONSTANT, COLOR_FALL)

    def _battery_info_cb(self, msg):
        """电源板/BMS保护回调（protection_flags!=0 即红灯常亮）"""
        if msg.protection_flags != 0:
            rospy.logwarn("[StartupLED] 电源板/BMS保护触发: protection_flags=%d", msg.protection_flags)
            self._set_state('stop', MODE_CONSTANT, COLOR_ERROR)

    def _power_board_status_cb(self, msg):
        """电源板系统状态回调（0x01 系统状态）

        电池拔出(bat1_exists/bat2_exists 任意 False)或
        两块电池都低电量(bat1_low_power 且 bat2_low_power 同时 True) → 红灯闪烁。
        恢复正常后，由 _poll_state 重新检测当前机器人状态并恢复对应 LED。
        """
        battery_abnormal = (not msg.bat1_exists or not msg.bat2_exists or
                            (msg.bat1_low_power and msg.bat2_low_power))

        if battery_abnormal:
            reason = []
            if not msg.bat1_exists: reason.append("BAT1拔出")
            if not msg.bat2_exists: reason.append("BAT2拔出")
            if msg.bat1_low_power and msg.bat2_low_power:
                reason.append("双电池低电量")
            rospy.logwarn("[StartupLED] 电源板电池状态: %s", " ".join(reason))
            self._set_state('power', MODE_FLASH, COLOR_FALL)
        else:
            # 电池恢复正常：重新检测当前机器人状态并恢复对应 LED
            if self.led_state == 'power':
                rospy.loginfo("[StartupLED] 电源板电池状态恢复正常，重新检测机器人状态")
                self.led_state = None  # 先清掉 power，否则 _set_state 会拦截正常状态切换
                self._poll_state(None)  # 根据当前 ready_to_start/is_ready/stand_up 重新判断

    def spin(self):
        rospy.spin()


if __name__ == '__main__':
    try:
        monitor = StartupLEDMonitor()
        monitor.spin()
    except rospy.ROSInterruptException:
        pass
