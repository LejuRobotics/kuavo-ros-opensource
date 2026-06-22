#!/usr/bin/env python3
"""5W 平台单手摇杆控躯干（折叠臂升降 + 前后倾 + 腰旋）独立节点

通过 launch_quest3_ik.launch 加 single_hand_mode:=true 启动。
订阅 /quest_joystick_data，复用 TorsoController 类，发布 /cmd_lb_torso_pose。
"""
import sys
import os
import rospy

# 让本节点能找到 tools 子包
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, SCRIPT_DIR)

from std_msgs.msg import Bool
from noitom_hi5_hand_udp_python.msg import JoySticks
from tools.torso_joystick_controller import create_torso_controller


def main():
    rospy.init_node("torso_joystick_node", anonymous=False)
    rospy.loginfo("[torso_joystick_node] 启动；尝试初始化 TorsoController...")
    try:
        torso = create_torso_controller()
    except Exception as e:
        rospy.logerr(f"[torso_joystick_node] TorsoController 初始化失败: {e}")
        rospy.signal_shutdown("TorsoController init failed")
        return
    rospy.loginfo("[torso_joystick_node] TorsoController 就绪，开始转发 joystick 数据")

    # 通知 QuestControlFSMNode 进入/退出单手躯干模式，让它在期间不发 /cmd_vel
    active_pub = rospy.Publisher("/single_hand_torso_active", Bool, queue_size=1, latch=True)
    active_pub.publish(Bool(data=False))  # 初始 false
    last_active = {"value": False}

    def _on_joy(msg):
        try:
            torso.handle_joystick(msg)
        except Exception as e:
            rospy.logerr_throttle(2.0, f"[torso_joystick_node] handle_joystick error: {e}")
            return
        # 主指手锁定/释放边沿触发，发布激活状态
        now_active = torso.main_hand is not None
        if now_active != last_active["value"]:
            active_pub.publish(Bool(data=now_active))
            last_active["value"] = now_active

    rospy.Subscriber("/quest_joystick_data", JoySticks, _on_joy, queue_size=10)
    rospy.spin()


if __name__ == "__main__":
    main()
