#!/usr/bin/env python3
"""
切换到VMP控制器

用法:
    rosrun humanoid_controllers vmp_switch_controller.py
"""

import sys
import rospy
from std_srvs.srv import Trigger


def main():
    rospy.init_node("vmp_switch_controller", anonymous=True)

    service_name = "/humanoid_controller/switch_to_vmp_controller"

    rospy.loginfo(f"等待服务: {service_name}")
    try:
        rospy.wait_for_service(service_name, timeout=5.0)
    except rospy.ROSException:
        rospy.logerr(f"服务 {service_name} 不可用")
        return 1

    try:
        switch = rospy.ServiceProxy(service_name, Trigger)
        resp = switch()

        if resp.success:
            rospy.loginfo(f"切换成功: {resp.message}")
            return 0
        else:
            rospy.logerr(f"切换失败: {resp.message}")
            return 1
    except rospy.ServiceException as e:
        rospy.logerr(f"服务调用失败: {e}")
        return 1


if __name__ == "__main__":
    try:
        sys.exit(main())
    except rospy.ROSInterruptException:
        pass
