"""
期望力测试案例：移动手臂到指定位置 -> 关闭挥空检测 -> 施加期望力 -> 等待 -> 撤销期望力 -> 恢复挥空检测
"""

import math
import time

import py_trees
import rospy
from std_msgs.msg import Bool

from kuavo_humanoid_sdk import KuavoSDK
from kuavo_humanoid_sdk.kuavo_strategy_pytree.nodes.api import TimedCmdAPI
from kuavo_humanoid_sdk.kuavo_strategy_pytree.nodes.nodes import (
    NodeArmForce,
    NodeFuntion,
    NodeWheelMoveTimedCmd,
)

BOX_WEIGHT_KG = 6.0         # 施加手臂末端重量，单位：kg
INTERPOLATION_SPEED = 2000.0   # 单位：N/s， 期望力插值速度
ARM_MOVE_TIME = 2.0           # 手臂运动时间，单位：s

FORCE_EMPTY_DETECT_ENABLE = False   # False: 关闭挥空检测; True: 开启挥空检测

# 单个手臂目标点位（arm_ee_local），姿态使用角度（yaw/pitch/roll）
ARM_EE_POSE_DEG = {
    "left":  [0.5, 0.22, 0.8, 0.0, -90.0, 0.0],   # x y z yaw pitch roll (deg)
    "right": [0.5, -0.22, 0.8, 0.0, -90.0, 0.0],  # x y z yaw pitch roll (deg)
}

def build_tree(timed_cmd_api: TimedCmdAPI) -> py_trees.behaviour.Behaviour:
    pub_enable_force_empty_detact = rospy.Publisher("/enable_force_empty_detact", Bool, queue_size=10)

    def pose_deg_to_keypoint(pose_deg: list) -> list:
        x, y, z, yaw_deg, pitch_deg, roll_deg = pose_deg
        return [x, y, z, math.radians(yaw_deg), math.radians(pitch_deg), math.radians(roll_deg)]

    def set_arm_keypoint():
        bb = py_trees.blackboard.Client(name="set_arm_ee_keypoints")
        bb.register_key("arm_ee_local_keypoints", py_trees.common.Access.WRITE)
        bb.register_key("arm_ee_keypoint_times", py_trees.common.Access.WRITE)
        keypoint = pose_deg_to_keypoint(ARM_EE_POSE_DEG["left"]) + pose_deg_to_keypoint(ARM_EE_POSE_DEG["right"])
        bb.arm_ee_local_keypoints = [keypoint]
        bb.arm_ee_keypoint_times = [ARM_MOVE_TIME]
        rospy.loginfo("已设置手臂末端单点位")
        return True

    def set_force_empty_detect(enabled: bool):
        # False: 关闭挥空检测; True: 开启挥空检测
        rospy.sleep(0.05)  # 给发布器建立连接一点时间
        pub_enable_force_empty_detact.publish(Bool(data=enabled))
        rospy.loginfo(
            f"[ForceEmptyDetect] {'启用' if enabled else '关闭'}挥空检测 (/enable_force_empty_detact={enabled})"
        )
        return True

    root = py_trees.composites.Sequence(name="arm_force_demo", memory=True)
    root.add_children(
        [
            NodeFuntion(name="set_arm_keypoint", fn=set_arm_keypoint),
            NodeWheelMoveTimedCmd(name="move_arm", timed_cmd_api=timed_cmd_api, cmd_type="arm_ee_local"),
            NodeFuntion(name="disable_force_empty_detect", fn=lambda: set_force_empty_detect(FORCE_EMPTY_DETECT_ENABLE)),
            NodeArmForce(
                name="enable_arm_force",
                box_weight_kg=BOX_WEIGHT_KG,
                enable=True,
                transition_time=1.0,
                interpolation_speed=INTERPOLATION_SPEED,
            ),
            NodeFuntion(name="wait_user_input", fn=lambda: (input("期望力已施加，按回车撤销...\n") or True)),
            NodeArmForce(
                name="disable_arm_force",
                box_weight_kg=BOX_WEIGHT_KG,
                enable=False,
                transition_time=1.0,
                interpolation_speed=INTERPOLATION_SPEED,
            ),
            NodeFuntion(name="enable_force_empty_detect", fn=lambda: set_force_empty_detect(True)),
        ]
    )
    return root


def main():
    KuavoSDK.Init(log_level="INFO")

    if not rospy.core.is_initialized():
        rospy.init_node("arm_force_case_node", anonymous=True, disable_signals=True)

    rospy.loginfo(f"期望力: {BOX_WEIGHT_KG} kg, 插值速度: {INTERPOLATION_SPEED}")
    rospy.loginfo(f"手臂运动时间: {ARM_MOVE_TIME} s")

    timed_cmd_api = TimedCmdAPI()
    root = build_tree(timed_cmd_api)
    tree = py_trees.trees.BehaviourTree(root)

    while True:
        tree.tick()
        status = root.status
        if status == py_trees.common.Status.SUCCESS:
            rospy.loginfo("期望力测试完成!")
            break
        if status == py_trees.common.Status.FAILURE:
            rospy.logerr("期望力测试失败.")
            break
        time.sleep(0.002)


if __name__ == "__main__":
    main()
