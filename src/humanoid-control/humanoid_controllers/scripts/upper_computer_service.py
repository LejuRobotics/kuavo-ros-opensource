#!/usr/bin/env python3
import rospy
from std_srvs.srv import Trigger, TriggerResponse
import os
import sys
from ssh_executor import SSHExecutor

SCRIPT_PATH = os.path.dirname(os.path.abspath(__file__))
REMOTE_CONFIG_PATH = os.path.join(SCRIPT_PATH, "remote-config.json")

class UpperComputerServiceNode:
    def __init__(self):
        rospy.init_node('upper_computer_service_node')
        self.ssh_executor = SSHExecutor(REMOTE_CONFIG_PATH)
        if not self.ssh_executor.connect():
            rospy.logerr("❌ 上位机连接失败")
            raise RuntimeError("上位机连接失败")
        rospy.loginfo("上位机已连接")
        self.start_srv = rospy.Service('/kuavo/start_stair_detection', Trigger, self.handle_start_stair_detection)
        self.start_srv_orbbec = rospy.Service('/kuavo/start_stair_detection_orbbec', Trigger, self.handle_start_stair_detection_orbbec)

        self.stop_srv = rospy.Service('/kuavo/stop_stair_detection', Trigger, self.handle_stop_stair_detection)
        rospy.on_shutdown(self.cleanup)
        rospy.loginfo("上位机服务节点已启动，等待服务调用...")

    def handle_start_stair_detection(self, req):
        try:
            self.ssh_executor.start_stair_detection()
            return TriggerResponse(success=True, message="已远程启动楼梯识别脚本")
        except Exception as e:
            return TriggerResponse(success=False, message=f"启动失败: {e}")

    def handle_start_stair_detection_orbbec(self, req):
        try:
            self.ssh_executor.start_stair_detection("orbbec")
            return TriggerResponse(success=True, message="已远程启动楼梯识别脚本")
        except Exception as e:
            return TriggerResponse(success=False, message=f"启动失败: {e}")

    def handle_stop_stair_detection(self, req):
        try:
            self.ssh_executor.stop_stair_detection()
            return TriggerResponse(success=True, message="已远程关闭楼梯识别脚本")
        except Exception as e:
            return TriggerResponse(success=False, message=f"关闭失败: {e}")

    def cleanup(self):
        self.ssh_executor.disconnect()
        rospy.loginfo("上位机已断开连接")

def main():
    node = UpperComputerServiceNode()
    rospy.spin()

if __name__ == '__main__':
    main() 
