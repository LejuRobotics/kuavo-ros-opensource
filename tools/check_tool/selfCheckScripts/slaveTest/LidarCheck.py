import rospy
from sensor_msgs.msg import PointCloud2
import os
from common.common_utils import print_colored_text

lidar_callback_count = 0
lidar_valid_flag = False

def check_and_print_status():
    global lidar_callback_count, lidar_valid_flag
    if lidar_callback_count >= 20:
        if lidar_valid_flag:
            print_colored_text("lidar 信息有效", color="green", bold=True)
        else:
            print_colored_text("lidar 信息无效", color="yellow", bold=True)

        rospy.signal_shutdown("All callbacks have been executed 20 times.")

def lidar_callback(msg):

    global lidar_callback_count
    global lidar_valid_flag

    lidar_callback_count += 1

    num_points = msg.width * msg.height
    if  0 != num_points:
        lidar_valid_flag =True
        # print("cloud size: ", num_points)

    check_and_print_status()

def timer_callback(event):
    global lidar_callback_count
    if lidar_callback_count == 0:
        print_colored_text("没有订阅到雷达点云的msg", color="yellow", bold=True)
        rospy.signal_shutdown("Timer expired.")

def main():
    rospy.init_node('lidar_subscriber', anonymous=True)
    rospy.Subscriber("/lidar", PointCloud2, lidar_callback)

    # 设置一个3秒的定时器
    rospy.Timer(rospy.Duration(3), timer_callback)

    # 保持节点运行
    rospy.spin()

if __name__ == '__main__':
    main()