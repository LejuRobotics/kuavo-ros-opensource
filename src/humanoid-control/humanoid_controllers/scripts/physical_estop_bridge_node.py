#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""物理急停桥接节点

订阅电源板状态 /power_board_status，检测 stop_int（急停中断）置位后发布
/stop_robot，把物理急停按钮接入标准软件急停链路，停止控制器、硬件与 rosbag 录制。
"""

import rospy
from std_msgs.msg import Bool
from kuavo_msgs.msg import PowerBoardStatus


class PhysicalEstopBridge:
    """把电源板物理急停标志转成 /stop_robot 软件急停事件"""

    def __init__(self):
        rospy.init_node('physical_estop_bridge_node', anonymous=False)
        self._stop_robot_pub = rospy.Publisher('/stop_robot', Bool, queue_size=10)
        # 仅在见过 stop_int==0 之后才允许触发，避免开机瞬态误报急停
        self._armed = False
        self._reported = False
        rospy.Subscriber('/power_board_status', PowerBoardStatus, self._on_status, queue_size=10)
        rospy.loginfo("[PhysicalEstop] bridge started, watching power board stop_int")

    def _on_status(self, msg):
        """stop_int 由 0 变 1 时上报一次急停"""
        if not msg.stop_int:
            self._armed = True   # 出现过正常态，后续上升沿才可信
            self._reported = False
            return
        if not self._armed or self._reported:
            return
        self._reported = True
        rospy.logerr("[PhysicalEstop] Physical e-stop detected (power board stop_int), publish /stop_robot")
        self._publish_stop_robot()

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
