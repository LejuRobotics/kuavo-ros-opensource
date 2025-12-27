#!/usr/bin/env python3
# 在下位机提供一个服务,用于一键启动太极脚本
import rospy
from taiji_trigger_node.srv import PerformTaiji, PerformTaijiResponse

import subprocess
import os
import time

current_path = os.path.dirname(os.path.abspath(__file__))
kuavo_resp_root = os.path.join(
    current_path,
    "..",
    "..",
    "..",
    "..",
)
source_path = os.path.join(kuavo_resp_root, "devel", "setup.bash")
ocs2body_demo_root = os.path.join(kuavo_resp_root, "src", "demo", "csv2body_demo")
taiji_exec_path = os.path.join(
    ocs2body_demo_root,
    "step_player_csv_ocs2.py",
)
taiji_action_csv_path = os.path.join(
    ocs2body_demo_root,
    "actions",
    "taiji_wuhan_step_part.csv",
)


class TaijiTriggerNode:
    def __init__(self) -> None:
        rospy.init_node("taiji_trigger_node", anonymous=True)

        self.taiji_service = None
        self.taiji_proc = None
        self.setup_taiji_service()

    def setup_taiji_service(self) -> None:
        self.taiji_service = rospy.Service(
            "/perform_taiji", PerformTaiji, self.handle_perform_taiji
        )

    def handle_perform_taiji(self, msg) -> PerformTaijiResponse:
        if rospy.get_param("/taiji_executing", False):
            rospy.logwarn("太极正在执行中!")
            return PerformTaijiResponse(
                success=False, message="另一个太极脚本正在执行中"
            )

        cmd = f"bash -c 'source {source_path} && python3 {taiji_exec_path} {taiji_action_csv_path}'"
        if not os.path.exists(taiji_exec_path):
            rospy.logerr(f"Taiji exec path not found: {taiji_exec_path}")
            return PerformTaijiResponse(
                success=False, message=f"Taiji exec path not found: {taiji_exec_path}"
            )
        if not os.path.exists(taiji_action_csv_path):
            rospy.logerr(f"Taiji action file not found: {taiji_action_csv_path}")
            return PerformTaijiResponse(
                success=False,
                message=f"Taiji action file not found: {taiji_action_csv_path}",
            )
        # 执行命令（非阻塞）
        print(f"正在启动taiji脚本")
        print(f"开始时间: {time.strftime('%Y-%m-%d %H:%M:%S')}")
        self.taiji_proc = subprocess.Popen(
            cmd, shell=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True
        )
        rospy.loginfo(cmd)
        return PerformTaijiResponse(success=True, message="success")


def main():
    TaijiTriggerNode()
    rospy.spin()


if __name__ == "__main__":
    main()
