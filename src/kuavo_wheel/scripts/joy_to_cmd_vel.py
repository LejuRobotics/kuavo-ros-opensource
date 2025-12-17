#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
import os


def clamp(value, min_value, max_value):
    return max(min_value, min(value, max_value))


def on_joy_message(joy_msg):
    global cmd_vel_publisher, linear_axis_index, angular_axis_index, linear_scale, angular_scale, deadzone

    twist = Twist()

    if len(joy_msg.axes) > max(linear_axis_index, angular_axis_index):
        linear_input = joy_msg.axes[linear_axis_index]
        angular_input = joy_msg.axes[angular_axis_index]

        if abs(linear_input) < deadzone:
            linear_input = 0.0
        if abs(angular_input) < deadzone:
            angular_input = 0.0

        twist.linear.x = clamp(linear_input * linear_scale, -linear_scale, linear_scale)

        # 前进的yaw角校正, 临时, 合肥拍摄版本
        angular_offset = 0.0
        # if abs(linear_input) > deadzone: 
        #     if linear_input > 0:
        #         angular_offset = 0.035 * abs(linear_input) 
        #     elif linear_input < 0:
        #         angular_offset = -0.035 * abs(linear_input)
            
        twist.angular.z = clamp(angular_input * angular_scale + angular_offset, -angular_scale, angular_scale)
    cmd_vel_publisher.publish(twist)


def main():
    global cmd_vel_publisher, linear_axis_index, angular_axis_index, linear_scale, angular_scale, deadzone

    rospy.init_node('joy_to_cmd_vel_node')

    # Detect joystick model and set mapping/scales without using ROS params
    # Defaults
    linear_axis_index = 1  # Left stick vertical
    angular_axis_index = 0  # Left stick horizontal
    linear_scale = 0.8  # m/s
    angular_scale = 1.2  # rad/s
    deadzone = 0.05

    try:
        joy_device = '/dev/input/js0'
        js_name = os.path.basename(joy_device)  # e.g., js0
        sys_name_file = f"/sys/class/input/{js_name}/device/name"
        if os.path.isfile(sys_name_file):
            with open(sys_name_file, 'r') as f:
                name = f.read().strip()
            upper_name = name.upper()
            print(upper_name)
            # Example profiles
            if 'BEITONG' in upper_name or 'BFM' in upper_name:
                # BEITONG A1N3: axes[1] forward/back, axes[0] left/right
                linear_axis_index = 1
                angular_axis_index = 2
                linear_scale = 0.5
                angular_scale = 0.4
                deadzone = 0.0
            elif 'XBOX' in upper_name or 'X-BOX' in upper_name:
                linear_axis_index = 4
                angular_axis_index = 3
                linear_scale = 0.8
                angular_scale = 1.2
                deadzone = 0.0

            # else keep defaults
            rospy.loginfo(f"Detected joystick '{name}', using axes (lin={linear_axis_index}, ang={angular_axis_index}), scales (lin={linear_scale}, ang={angular_scale}), deadzone={deadzone}")
        else:
            rospy.logwarn(f"Joystick sysfs not found: {sys_name_file}, using defaults")
    except Exception as e:
        rospy.logwarn(f"Joystick detection failed: {e}, using defaults")


    cmd_vel_topic = rospy.get_param('~cmd_vel_topic', '/cmd_vel')
    joy_topic = rospy.get_param('~joy_topic', '/joy')

    cmd_vel_publisher = rospy.Publisher(cmd_vel_topic, Twist, queue_size=10)
    rospy.Subscriber(joy_topic, Joy, on_joy_message)

    rospy.loginfo('joy_to_cmd_vel_node started: joy=%s -> cmd_vel=%s', joy_topic, cmd_vel_topic)
    # 操作提示（右摇杆控制）
    rospy.loginfo("操作提示: 使用右摇杆控制底盘：上下=前进/后退，左右=原地旋转")
    rospy.loginfo(f"当前映射: linear_axis_index={linear_axis_index}, angular_axis_index={angular_axis_index}, deadzone={deadzone}")

    rospy.spin()


if __name__ == '__main__':
    main() 