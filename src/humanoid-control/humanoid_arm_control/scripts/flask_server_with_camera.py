#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from flask import Flask, request, jsonify
import os
import sys
import rospy
from humanoid_arm_control.srv import armControl, armControlRequest  # 导入你实际的ROS服务
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import base64
from KuavoSDK import kuavo
from BezierWrap import BezierWrap
import threading

app = Flask(__name__)
app.debug = True

# 初始化 ROS 节点
rospy.init_node('flask_ros_server', anonymous=True)

kuavo_instance = kuavo()

# 等待 /arm_control 服务启动
rospy.wait_for_service('/arm_control')
arm_control_service = rospy.ServiceProxy('/arm_control', armControl)


# 将命令映射为 ROS 服务的请求值
command_mapping = {
    "left": 1,
    "right": 2,
    "middle": 0
}

def call_arm_control(req, tagid):
    try:
        response = arm_control_service(armControlRequest(req, tagid))  # 向ROS服务发送请求
        return response.success, response.time
    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")
        return False, 0.0

@app.route('/humanoid_arm_control', methods=['POST'])
def control():
    data = request.json  # 获取POST请求中的JSON数据
    command = data.get("command")
    tagid = data.get("id")  # 获取POST请求中的id

    # 验证 command 和 id 是否存在
    if not command or not tagid:
        return jsonify({"error": "Missing command or id"}), 400
    
    if command not in command_mapping:
        return jsonify({"error": "Invalid command"}), 400

    # 根据 command 映射为相应的 int64 值
    req = command_mapping[command]

    # 调用 ROS 服务
    success, duration = call_arm_control(req, tagid)

    # 如果成功，返回message和duration
    if success:
        return jsonify({"message": "Command executed successfully", "duration": duration}), 200
    # 如果不成功，返回message和501
    else:
        return jsonify({"message": "Failed to execute command"}), 501


# 初始化 CvBridge
bridge = CvBridge()

lock = threading.Lock()

# 用于存储接收到的最新图像
latest_image = None
counter = 0

def image_callback(msg):
    with lock:
        global latest_image
        global counter
        counter = counter + 1
        print("get new image", counter)
        try:
            latest_image = msg

            # 将 JPEG 图像转换为 Base64
            # latest_image = base64.b64encode(buffer).decode('utf-8')

        except Exception as e:
            rospy.logerr(f"Error converting ROS Image to OpenCV: {e}")

# 定义接口
@app.route('/humanoid_get_image', methods=['GET'])
def get_image():
    if latest_image:
        # 返回 Base64 格式的图像
        kuavo_instance.set_head_target(30.0, -20.0)
        rospy.sleep(5)
        
        cv_image = bridge.imgmsg_to_cv2(latest_image, "bgr8")
        _, buffer = cv2.imencode('.jpg', cv_image)
        left_image = base64.b64encode(buffer).decode('utf-8')
        
        kuavo_instance.set_head_target(-30.0, -20.0)
        rospy.sleep(5)
        
        cv_image = bridge.imgmsg_to_cv2(latest_image, "bgr8")
        _, buffer = cv2.imencode('.jpg', cv_image)
        right_image = base64.b64encode(buffer).decode('utf-8')
        
        kuavo_instance.set_head_target(0.0, 0.0)
        return jsonify({'left_image': left_image, 'right_image': right_image}), 200
    else:
        return jsonify({'error': 'No image received yet'}), 404

def get_image_thread():
    pass
    
@app.route('/humanoid_nod', methods=['POST'])
def head_nod():
    # 返回 Base64 格式的图像
    kuavo_instance.set_head_target(0, -20.0)
    rospy.sleep(0.5)
    kuavo_instance.set_head_target(0, 0)
    return jsonify({'success': True}), 200

wave_action_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "actions/wave.tact")

@app.route('/humanoid_wave', methods=['POST'])
def wave_hand():
    bezier_wrap = BezierWrap()
    bezier_wrap.load_pre_bezier_traj(wave_action_path)
    curves = bezier_wrap.get_standard_bezier_line_format()
    curves = curves[0:14]
    points_all = bezier_wrap.get_traj_points_after_interpolate(curves, 100)
    time_list_wave, torque_list_wave = bezier_wrap.get_traj_points_after_seperate_time_q(points_all)
    print("time_list_wave: ")
    print(time_list_wave)
    print("torque_list_wave")
    print(torque_list_wave)
    
    kuavo_instance.move_with_trajactory(time_list_wave, torque_list_wave)
    return jsonify({'success': True}), 200

rospy.Subscriber('/camera/color/image_raw', Image, image_callback, queue_size=1)

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=8031, threaded=True)