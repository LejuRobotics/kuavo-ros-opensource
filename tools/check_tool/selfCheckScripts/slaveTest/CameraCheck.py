import rospy
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
import cv2
import os
from common.common_utils import print_colored_text

bridge = CvBridge()
# 初始化颜色图像回调次数计数器
color_callback_count = 0
# 初始化深度图像回调次数计数器
depth_callback_count = 0

color_valid_flag = False
depth_valid_flag = False

def check_and_print_status():
    global color_callback_count, depth_callback_count
    if color_callback_count >= 20 and depth_callback_count >= 20:
        if color_valid_flag:
            print_colored_text("RBG 图片信息有效", color="green", bold=True)
        else:
            print_colored_text("RBG 图片信息无效", color="yellow", bold=True)

        if depth_valid_flag:
            print_colored_text("depth 图片信息有效", color="green", bold=True)
        else:
            print_colored_text("depth 图片信息无效", color="yellow", bold=True)

        rospy.signal_shutdown("All callbacks have been executed 20 times.")

def image_callback(msg):
    global color_callback_count
    global color_valid_flag
    # 若颜色图像回调次数达到 20 次，直接返回
    if color_callback_count >= 20:
        return
    color_callback_count += 1

    try:
        # 将 ROS 图像消息转换为 OpenCV 格式
        cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
        if cv_image.size == 0:
            rospy.logerr("Converted color image has zero size.")
        else:
            color_valid_flag = True
    except CvBridgeError as e:
        rospy.logerr(f"CvBridge Error in color image: {e}")

    check_and_print_status()

# 深度图像回调函数
def depth_callback(msg):
    global depth_callback_count
    global depth_valid_flag
    # 若深度图像回调次数达到 20 次，直接返回
    if depth_callback_count >= 20:
        return
    depth_callback_count += 1

    try:
        # 将 ROS 深度图像消息转换为 OpenCV 格式
        depth_image = bridge.imgmsg_to_cv2(msg, "16UC1")
        if depth_image.size == 0:
            rospy.logerr("Converted depth image has zero size.")
        else:
            depth_valid_flag = True
    except CvBridgeError as e:
        rospy.logerr(f"CvBridge Error in depth image: {e}")

    check_and_print_status()

def timer_callback(event):
    global color_callback_count, depth_callback_count
    if color_callback_count == 0 and depth_callback_count == 0:
        print_colored_text("没有订阅到相机图片的msg", color="yellow", bold=True)
    rospy.signal_shutdown("Timer expired.")

def main():
    rospy.init_node('image_subscriber_downstream', anonymous=True)
    rospy.Subscriber("/camera/color/image_raw", Image, image_callback)
    rospy.Subscriber('/camera/depth/image_rect_raw', Image, depth_callback)

    # 设置一个3秒的定时器
    rospy.Timer(rospy.Duration(3), timer_callback)

    # 保持节点运行
    rospy.spin()

if __name__ == '__main__':
    main()