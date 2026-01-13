#!/usr/bin/env python3
"""
VMP轨迹执行工具 - 交互式选择并执行轨迹

用法:
    rosrun humanoid_controllers vmp_execute_trajectory.py
"""

import sys
import rospy
from kuavo_msgs.srv import GetStringList, SetString


CONTROLLER_NAME = "vmp_controller"


class VMPTrajectoryExecutor:
    def __init__(self):
        self.service_ns = f"/humanoid_controllers/{CONTROLLER_NAME}"

    def get_trajectory_list(self) -> list:
        """获取轨迹列表"""
        service_name = f"{self.service_ns}/trajectory/list"

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

    def execute_trajectory(self, trajectory_name: str) -> bool:
        """执行指定轨迹"""
        service_name = f"{self.service_ns}/trajectory/execute"

        try:
            rospy.wait_for_service(service_name, timeout=5.0)
        except rospy.ROSException:
            rospy.logerr(f"服务 {service_name} 不可用")
            return False

        try:
            execute = rospy.ServiceProxy(service_name, SetString)
            resp = execute(trajectory_name)

            if resp.success:
                rospy.loginfo(f"轨迹执行成功: {resp.message}")
                return True
            else:
                rospy.logerr(f"轨迹执行失败: {resp.message}")
                return False
        except rospy.ServiceException as e:
            rospy.logerr(f"服务调用失败: {e}")
            return False


def interactive_select(trajectories: list) -> str:
    """交互式选择轨迹"""
    print("\n" + "=" * 50)
    print("可用轨迹列表:")
    print("=" * 50)

    for i, name in enumerate(trajectories):
        print(f"  [{i}] {name}")

    print("-" * 50)
    print("  [q] 退出")
    print("=" * 50)

    while True:
        try:
            user_input = input("\n请选择轨迹编号 (或输入轨迹名称): ").strip()

            if user_input.lower() == 'q':
                return None

            # 尝试解析为索引
            try:
                index = int(user_input)
                if 0 <= index < len(trajectories):
                    return trajectories[index]
                else:
                    print(f"无效的索引: {index} (有效范围: 0-{len(trajectories)-1})")
                    continue
            except ValueError:
                pass

            # 尝试作为轨迹名称
            if user_input in trajectories:
                return user_input

            # 模糊匹配
            matches = [t for t in trajectories if user_input.lower() in t.lower()]
            if len(matches) == 1:
                return matches[0]
            elif len(matches) > 1:
                print(f"多个匹配: {matches}")
                continue
            else:
                print(f"未找到轨迹: {user_input}")
                continue

        except EOFError:
            return None
        except KeyboardInterrupt:
            print("\n已取消")
            return None


def main():
    rospy.init_node("vmp_execute_trajectory", anonymous=True)

    executor = VMPTrajectoryExecutor()

    trajectories = executor.get_trajectory_list()

    if trajectories is None:
        return 1

    if len(trajectories) == 0:
        print("无可用轨迹")
        return 1

    selected = interactive_select(trajectories)

    if selected is None:
        print("已退出")
        return 0

    print(f"\n执行轨迹: {selected}")
    executor.execute_trajectory(selected)

    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except rospy.ROSInterruptException:
        pass
