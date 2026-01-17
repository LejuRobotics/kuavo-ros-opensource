#!/usr/bin/env python3
"""
VMP轨迹列表查询工具

用法:
    rosrun humanoid_controllers vmp_list_trajectories.py
"""

import sys
import rospy
from kuavo_msgs.srv import GetStringList


def get_trajectory_list(controller_name: str) -> list:
    """获取轨迹列表"""
    service_name = f"/humanoid_controllers/{controller_name}/trajectory/list"

    rospy.loginfo(f"等待服务: {service_name}")
    try:
        rospy.wait_for_service(service_name, timeout=5.0)
    except rospy.ROSException:
        rospy.logerr(f"服务 {service_name} 不可用")
        return None

    try:
        get_list = rospy.ServiceProxy(service_name, GetStringList)
        resp = get_list()

        if resp.success:
            return list(resp.data)
        else:
            rospy.logerr(f"获取轨迹列表失败: {resp.message}")
            return None
    except rospy.ServiceException as e:
        rospy.logerr(f"服务调用失败: {e}")
        return None


CONTROLLER_NAME = "vmp_controller"


def main():
    rospy.init_node("vmp_list_trajectories", anonymous=True)

    trajectories = get_trajectory_list(CONTROLLER_NAME)

    if trajectories is None:
        sys.exit(1)

    print("=" * 50)
    print("VMP轨迹列表")
    print("=" * 50)

    if len(trajectories) == 0:
        print("(无可用轨迹)")
    else:
        for i, name in enumerate(trajectories):
            print(f"  [{i}] {name}")

    print("=" * 50)
    print(f"共 {len(trajectories)} 条轨迹")

    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except rospy.ROSInterruptException:
        pass
