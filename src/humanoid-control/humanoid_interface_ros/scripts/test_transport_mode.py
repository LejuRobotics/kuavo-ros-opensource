#!/usr/bin/env python3
"""搬运模式测试 —— 键盘转 Joy + 躯干吊绳（电动绞盘）控制

按键:
  Y → LB+RB+Y  进入搬运模式（关节插值到默认姿态）
  A → LB+RB+A  退出搬运模式（桩）
  B → LB+RB+B  掉使能倒地（桩）
  X → LB+RB+X  两步起身（桩）
  S → 吊绳 开/关
  - → 绳长缩短 5cm（更紧）
  = → 绳长放长 5cm（更松）
  Q → 退出
"""

import sys
import termios
import tty
import select
import rospy
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool
from geometry_msgs.msg import Vector3

BUTTON_A = 0
BUTTON_B = 1
BUTTON_X = 2
BUTTON_Y = 3
BUTTON_LB = 4
BUTTON_RB = 5

KEY_MAP = {
    'y': (BUTTON_Y, "LB+RB+Y 进入搬运"),
    'a': (BUTTON_A, "LB+RB+A 退出搬运"),
    'b': (BUTTON_B, "LB+RB+B 倒地"),
    'x': (BUTTON_X, "LB+RB+X 起身"),
}


def make_joy(buttons):
    msg = Joy()
    msg.axes = [0.0] * 8
    if len(buttons) < 11:
        buttons = buttons + [0] * (11 - len(buttons))
    msg.buttons = buttons[:11]
    msg.header.stamp = rospy.Time.now()
    return msg


def send_combo(pub, extra_button):
    """脉冲：按下 0.3s → 松开"""
    buttons = [0] * 11
    buttons[BUTTON_LB] = 1
    buttons[BUTTON_RB] = 1
    buttons[extra_button] = 1
    pub.publish(make_joy(buttons))
    rospy.sleep(0.3)
    pub.publish(make_joy([0] * 11))


def main():
    rospy.init_node("test_transport_keyboard")
    joy_pub = rospy.Publisher("/joy", Joy, queue_size=10)
    spring_active_pub = rospy.Publisher("/mujoco/torso_rope/active", Bool, queue_size=10)
    spring_params_pub = rospy.Publisher("/mujoco/torso_rope/params", Vector3, queue_size=10)

    rospy.loginfo("等待 /joy subscriber ...")
    deadline = rospy.Time.now() + rospy.Duration(5)
    while joy_pub.get_num_connections() == 0 and rospy.Time.now() < deadline:
        rospy.sleep(0.1)
    if joy_pub.get_num_connections() == 0:
        rospy.logerr("无 /joy subscriber，请先启动仿真")
        return

    # 初始参数：speed=0.3 m/s, kv=5000, rope_len=1.1；锚点 Z=1.8m 固定
    spring_on = False
    rope_len = 1.1
    spring_params_pub.publish(Vector3(0.3, 5000.0, rope_len))
    rospy.sleep(0.1)

    rospy.loginfo("Y=进入 A=退出 B=倒地 X=起身   S=吊绳  -=绳长缩  ==绳长放   Q=退出")
    rospy.loginfo("rope_len=%.3f", rope_len)

    old = termios.tcgetattr(sys.stdin)
    try:
        tty.setcbreak(sys.stdin.fileno())
        while not rospy.is_shutdown():
            if select.select([sys.stdin], [], [], 0.1)[0]:
                ch = sys.stdin.read(1).lower()
                if ch == 'q':
                    rospy.loginfo("退出")
                    break
                if ch == 's':
                    spring_on = not spring_on
                    spring_active_pub.publish(Bool(spring_on))
                    rospy.loginfo(">>> 吊绳 %s", "ON" if spring_on else "OFF")
                elif ch == '-':
                    rope_len = max(0.05, rope_len - 0.05)
                    spring_params_pub.publish(Vector3(0.3, 5000.0, rope_len))
                    rospy.loginfo(">>> 绳长=%.2fm", rope_len)
                elif ch == '=':
                    rope_len += 0.05
                    spring_params_pub.publish(Vector3(0.3, 5000.0, rope_len))
                    rospy.loginfo(">>> 绳长=%.2fm", rope_len)
                elif ch in KEY_MAP:
                    btn, label = KEY_MAP[ch]
                    rospy.loginfo(">>> %s", label)
                    send_combo(joy_pub, btn)
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old)


if __name__ == "__main__":
    main()
