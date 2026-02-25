import rospy
from std_srvs.srv import Empty
import sys

def main():
    # 初始化ROS节点
    rospy.init_node('behavior_tree_controller_cli', anonymous=True)

    try:
        # 等待服务可用
        rospy.loginfo("等待行为树控制服务...")
        rospy.wait_for_service('/stop_behavior_tree', timeout=5.0)
        rospy.wait_for_service('/restart_behavior_tree', timeout=5.0)

        # 创建服务代理
        stop_service = rospy.ServiceProxy('/stop_behavior_tree', Empty)
        restart_service = rospy.ServiceProxy('/restart_behavior_tree', Empty)

        rospy.loginfo("行为树控制服务已连接成功！")
        rospy.loginfo("命令列表：")
        rospy.loginfo("  stop   - 发送停止行为树请求")
        rospy.loginfo("  restart - 发送重启行为树请求")
        rospy.loginfo("  exit    - 退出程序")

        # 主循环处理用户输入
        while not rospy.is_shutdown():
            command = input("请输入命令 (stop/restart/exit): ").strip().lower()

            if command == 'stop':
                try:
                    rospy.loginfo("正在发送停止行为树请求...")
                    stop_service()
                    rospy.loginfo("停止行为树请求已发送成功！")
                except rospy.ServiceException as e:
                    rospy.logerr(f"调用停止服务失败: {str(e)}")

            elif command == 'restart':
                try:
                    rospy.loginfo("正在发送重启行为树请求...")
                    restart_service()
                    rospy.loginfo("重启行为树请求已发送成功！")
                except rospy.ServiceException as e:
                    rospy.logerr(f"调用重启服务失败: {str(e)}")

            elif command in ['exit', 'quit', 'q']:
                rospy.loginfo("程序已退出！")
                break

            else:
                rospy.logwarn("无效命令！请输入 stop、restart 或 exit")

    except rospy.ROSException as e:
        rospy.logerr(f"无法连接到行为树控制服务: {str(e)}")
        sys.exit(1)

if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        rospy.loginfo("程序已被用户中断")
