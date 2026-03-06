#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
import termios
import tty
import select
import time

import rospy
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool

from arm_keyboard_control import (
    KeyBoardArmController,
    ArmType,
    get_version_parameter,
    set_arm_control_mode,
)

# JoyButton constants
BUTTON_A = 0
BUTTON_B = 1
BUTTON_X = 2
BUTTON_Y = 3
BUTTON_LB = 4
BUTTON_RB = 5
BUTTON_BACK = 6
BUTTON_START = 7

# JoyAxis constants
AXIS_LEFT_STICK_Y = 0
AXIS_LEFT_STICK_X = 1
AXIS_LEFT_LT = 2  # 1 -> (-1)
AXIS_RIGHT_STICK_YAW = 3
AXIS_RIGHT_STICK_Z = 4
AXIS_RIGHT_RT = 5  # 1 -> (-1)
AXIS_LEFT_RIGHT_TRIGGER = 6
AXIS_FORWARD_BACK_TRIGGER = 7

# 手臂控制部分改为从KeyBoardArmController继承，详见同目录下的arm_keyboard_control.py
class KeyBoardRobotController(KeyBoardArmController):
    def __init__(
        self,
        x_gap=0.03, # 此处仅为占位, 具体参数以main函数中传入的为准
        y_gap=0.03,
        z_gap=0.03,
        roll_gap=0.03,
        pitch_gap=0.03,
        yaw_gap=0.03,
        time_gap=0.5,
        robot_version=45,
        arm_control_enabled=True,
        which_hand=ArmType.Right,
    ):

        self.robot_mode_flag = 1  # 1为键盘控制手臂移动  2为键盘控制机器人运动
        self.change_robot_mode_flag = 1  # 切换模式时进行特殊处理
        self.arm_control_enabled = arm_control_enabled

        if self.arm_control_enabled:
            # 手臂相关能力全部继承自基类
            super().__init__(
                x_gap=x_gap,
                y_gap=y_gap,
                z_gap=z_gap,
                roll_gap=roll_gap,
                pitch_gap=pitch_gap,
                yaw_gap=yaw_gap,
                time_gap=time_gap,
                robot_version=robot_version,
                which_hand=which_hand,
            )
        else:
            # arm模式禁用时，不初始化基类中的手臂相关资源
            self.input_buffer = []
            self.old_settings = termios.tcgetattr(sys.stdin)
            self._flag_pose_inited = True

        rospy.Subscriber("/stop_robot", Bool, self.stop_robot_callback)

        self.joy_pub = rospy.Publisher('/joy', Joy, queue_size=10)
        self.joy_msg = Joy()
        self.joy_msg.axes = [0.0] * 8  # Initialize 8 axes
        self.joy_msg.buttons = [0] * 11  # Initialize 11 buttons
        self.old_settings = termios.tcgetattr(sys.stdin)

    def stop_robot_callback(self, msg):
        rospy.signal_shutdown("stop_robot")

    def _read_key(self):
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_settings)
        return key

    # 移动模式用机器人按键逻辑；手臂模式复用基类按键处理
    def getKey(self):
        key = self._read_key()

        if key == 'v':
            if self.arm_control_enabled:
                self.change_robot_mode_flag = 1
            else:
                rospy.logwarn("切换失败：当前 robot_version 无效，已禁止键盘控制手臂移动模式。")
            return ''

        if self.robot_mode_flag == 2:
            return key

        if not key:
            return ''

        # 将按键送入基类输入缓冲，由基类完成手臂键处理流程
        self.input_buffer.append(key)
        return super().getKey()

    # 键盘控制机器人运动 响应函数
    def update_joy(self, key):
        key = key.lower()
        self.joy_msg.buttons = [0] * 11

        if key == 'w':
            self.joy_msg.axes[AXIS_LEFT_STICK_X] = round(min(1.0, self.joy_msg.axes[AXIS_LEFT_STICK_X] + 0.1), 3)
        elif key == 's':
            self.joy_msg.axes[AXIS_LEFT_STICK_X] = round(max(-1.0, self.joy_msg.axes[AXIS_LEFT_STICK_X] - 0.1), 3)
        elif key == 'a':
            self.joy_msg.axes[AXIS_LEFT_STICK_Y] = round(min(1.0, self.joy_msg.axes[AXIS_LEFT_STICK_Y] + 0.4), 3)
        elif key == 'd':
            self.joy_msg.axes[AXIS_LEFT_STICK_Y] = round(max(-1.0, self.joy_msg.axes[AXIS_LEFT_STICK_Y] - 0.4), 3)
        elif key == 'i':
            self.joy_msg.axes[AXIS_RIGHT_STICK_Z] = round(min(1.0, self.joy_msg.axes[AXIS_RIGHT_STICK_Z] + 0.1), 3)
        elif key == 'k':
            self.joy_msg.axes[AXIS_RIGHT_STICK_Z] = round(max(-1.0, self.joy_msg.axes[AXIS_RIGHT_STICK_Z] - 0.1), 3)
        elif key == 'l' or key == 'e':
            self.joy_msg.axes[AXIS_RIGHT_STICK_YAW] = round(max(-1.0, self.joy_msg.axes[AXIS_RIGHT_STICK_YAW] - 0.1), 3)
        elif key == 'j' or key == 'q':
            self.joy_msg.axes[AXIS_RIGHT_STICK_YAW] = round(min(1.0, self.joy_msg.axes[AXIS_RIGHT_STICK_YAW] + 0.1), 3)
        elif key == ' ':  # Space key
            self.joy_msg.axes = [0.0] * 8  # Reset all axes to zero
        elif key == 'r':
            self.joy_msg.buttons[BUTTON_Y] = 1  # 发送walk
        elif key == 'c':
            self.joy_msg.buttons[BUTTON_A] = 1  # 发送stance
            self.joy_msg.axes = [0.0] * 8  # Reset all axes to zero
            
        cmdvel = [self.joy_msg.axes[AXIS_LEFT_STICK_X],self.joy_msg.axes[AXIS_LEFT_STICK_Y], 0, 0, 0, self.joy_msg.axes[AXIS_RIGHT_STICK_YAW]]
        print(f"cmdvel: {[f'{x * 100:.0f}%' for x in cmdvel]}", end='\r')
        # self.joy_pub.publish(self.joy_msg)

    # 机器人程序入口：
    def robot_run(self):
        if self.arm_control_enabled:
            print("waiting for ik server...")
            # 等待初始化结束
            while not self._flag_pose_inited and not rospy.is_shutdown():
                time.sleep(0.2)
            # 初始化为运动模式，下一循环通过切换流程进入手臂模式并打印提示
            self.robot_mode_flag = 2
        else:
            self.robot_mode_flag = 2
            self.change_robot_mode_flag = 0
            print("Use keys to control:")
            print("WASD: Left stick, control forward/backward, left/right")
            print("IKJL/QE: Right stick, up/down, turn left/right")
            print("R: walk, C: stance")
            print("<space>: Reset all axes to zero")
            print("Press V to Switch arm control (disabled)")
            print("Press Ctrl-C to exit")

        try:

            if self.arm_control_enabled:
                set_arm_control_mode(2)

            while not rospy.is_shutdown():
                if self.arm_control_enabled and self.change_robot_mode_flag == 1:
                    self.change_robot_mode_flag = 0

                    # 机器人运动 -> 手臂移动
                    if self.robot_mode_flag == 2:
                        self.robot_mode_flag = 1
                        set_arm_control_mode(2)
                        self.update_joy('c')
                        self.joy_pub.publish(self.joy_msg)
                        print("Use keys to control:")
                        print("WS: position - X")
                        print("AD: position - Y")
                        print("QE: position - Z")
                        print("UO: rotation - X - ROLL")
                        print("IK: rotation - Y - PITCH")
                        print("JL: rotation - Z - YAW")
                        print("Press N to Switch to another hand")
                        print("Press V to Switch move control")
                        print("Press Ctrl-C to exit")

                    # 手臂移动 -> 机器人运动
                    elif self.robot_mode_flag == 1:
                        self.robot_mode_flag = 2
                        set_arm_control_mode(1)
                        self.update_joy('c')
                        self.joy_pub.publish(self.joy_msg)
                        print("Use keys to control:")
                        print("WASD: Left stick, control forward/backward, left/right")
                        print("IKJL/QE: Right stick, up/down, turn left/right")
                        print("R: walk, C: stance")
                        print("<space>: Reset all axes to zero")
                        print("Press V to Switch arm control")
                        print("Press Ctrl-C to exit")

                key = self.getKey()
                if key == '\x03':  # Ctrl-C
                    break
                if key:
                    if self.robot_mode_flag == 1:
                        self.update_response()
                    else :
                        self.update_joy(key)
                        
                if self.robot_mode_flag == 2:
                    self.joy_pub.publish(self.joy_msg)

            if self.arm_control_enabled:
                # 手臂恢复控制模式为一
                set_arm_control_mode(1)
            # 机器人恢复站立
            self.update_joy('c')
            self.joy_pub.publish(self.joy_msg)

        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_settings)


if __name__ == "__main__":
    try:
        rospy.init_node("robot_control_keyboard_node", anonymous=True)
        # 获取机器人版本
        my_robot_version = get_version_parameter()
        arm_control_enabled = my_robot_version is not None
        if not arm_control_enabled:
            rospy.logwarn("robot_version 参数无效或缺失：将禁用键盘控制手臂移动功能，仅保留机器人运动控制。")
            my_robot_version = 45

        # Right Arm
        keyboard_robot_controller = KeyBoardRobotController(x_gap = 0.03, y_gap = 0.03, z_gap = 0.03,
                                                        roll_gap = 0.157, pitch_gap = 0.157, yaw_gap = 0.157, 
                                                        time_gap = 1.5,
                                                        robot_version = my_robot_version,
                                                        arm_control_enabled = arm_control_enabled,
                                                        which_hand=ArmType.Right)
        keyboard_robot_controller.robot_run()
    except rospy.ROSInterruptException:
        pass
