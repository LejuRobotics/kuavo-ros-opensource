#!/usr/bin/env python3

import rospy
import cv2
import time
import threading
import numpy as np
import os

from cv_bridge import CvBridge
from ultralytics import YOLO

from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from kuavo_msgs.msg import robotHeadMotionData, sensorsData
# 导入新的消息类型
from kuavo_msgs.msg import FaceBoundingBox

from std_msgs.msg import Header

class PID:
    def __init__(self, kp, ki, kd, output_limits=(None, None)):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.min_out, self.max_out = output_limits

        self._prev_error = 0.0
        self._integral = 0.0
        self._last_time = None

    def reset(self):
        self._prev_error = 0.0
        self._integral = 0.0
        self._last_time = None
    
    def __call__(self, error, now=None):
        """Compute PID output given current error and timestamp."""
        if now is None:
            now = time.time()
        dt = 0.0
        if self._last_time is not None:
            dt = now - self._last_time
        self._last_time = now

        # Proportional term
        p = self.kp * error

        # Integral term
        self._integral += error * dt
        i = self.ki * self._integral

        # Derivative term
        d_error = 0.0
        if dt > 0:
            d_error = (error - self._prev_error) / dt
        d = self.kd * d_error

        self._prev_error = error

        # PID output before limits
        out = p + i + d

        # Clamp to output limits
        if self.min_out is not None:
            out = max(self.min_out, out)
        if self.max_out is not None:
            out = min(self.max_out, out)

        return out
        

class FaceTrack:
    def __init__(self):
        rospy.init_node('face_track_node', anonymous=True)

        # 查询当前脚本所在路径，然后加载模型
        current_dir = os.path.dirname(os.path.abspath(__file__))
        model_path = os.path.join(current_dir, 'yolov8n-face.pt')
        self.model = YOLO(model_path)
        self.bridge = CvBridge()
        rospy.loginfo("人脸识别模型加载完毕....")

        self.face_position_x = 0    # 人脸框中点在图像中的位置
        self.face_position_y = 0
        self.target_point_x = 640    # 宽 1280，目标点 x 坐标
        self.target_point_y = 360    # 高 720，目标点 y 坐标

        self.is_face_detected = False
        self.min_face_area = 5000  # 最小人脸面积阈值（像素）

        # 帧率计算相关变量
        self.prev_time = time.time()
        self.fps = 0

        # CPU资源控制相关变量
        self.last_process_time = time.time()
        self.process_interval = 0.05  # 处理间隔（秒），限制为每秒最多处理20帧
        self.frame_count = 0
        self.process_every_n_frames = 2  # 每处理1帧就跳过2帧

        # yaw 左右转动，pitch 上下转动
        self.yaw_pid = PID(kp=0.015, ki=0.00, kd=0.001, output_limits=(-90, 90))
        self.pitch_pid = PID(kp=0.015, ki=0.00, kd=0.001, output_limits=(-20, 30))
        self.head_yaw = 0.0
        self.head_pitch = 0.0
        self.head_yaw_limit = (-90, 90)
        self.head_pitch_limit = (-20, 30)

        self.motor_lock = threading.Lock()

        # 检测指定话题是否存在
        self.image_sub = None
        self.check_and_subscribe_to_camera()

        self.head_state_sub = rospy.Subscriber("/sensors_data_raw", sensorsData, self.update_head_state)
        self.head_motion_pub = rospy.Publisher("/robot_head_motion_data", robotHeadMotionData, queue_size=10)

        self.image_pub = rospy.Publisher("/camera/detected_face", Image, queue_size=30)
        # 修改发布器，使用新的FaceBoundingBox消息类型
        self.face_position_pub = rospy.Publisher("/face_detection/bounding_box", FaceBoundingBox, queue_size=10)
        
        self.cv_image = None

    def check_and_subscribe_to_camera(self):
        """检查摄像头话题是否存在，并订阅第一个找到的话题"""
        # 获取当前发布的所有话题
        published_topics = rospy.get_published_topics()
        camera_topics = [topic for topic, _ in published_topics if 'image_raw' in topic and ('camera' in topic or 'cam_h' in topic)]
        
        rospy.loginfo("发现的摄像头相关话题: %s", camera_topics)
        
        # 定义优先级话题列表
        priority_topics = ["/camera/color/image_raw", "/cam_h/color/image_raw"]
        
        # 根据优先级订阅第一个可用的话题
        subscribed = False
        for topic in priority_topics:
            if topic in camera_topics:
                self.image_sub = rospy.Subscriber(topic, Image, self.image_callback)
                rospy.loginfo("已订阅话题: %s", topic)
                subscribed = True
                break
        
        # # 如果优先级话题都不存在，但有其他image_raw话题，则订阅第一个
        # if not subscribed and camera_topics:
        #     topic = camera_topics[0]
        #     self.image_sub = rospy.Subscriber(topic, Image, self.image_callback)
        #     # rospy.loginfo("订阅了第一个找到的摄像头话题: %s", topic)
        #     subscribed = True
            
        if not subscribed:
            rospy.logwarn("未找到任何可用的摄像头话题")

    def image_callback(self, msg):
        try:
            # 帧计数器增加
            self.frame_count += 1
            
            # 控制处理频率 - 跳帧处理
            if self.frame_count % self.process_every_n_frames != 0:
                return
            
            # 控制处理间隔 - 时间间隔控制
            current_time = time.time()
            if current_time - self.last_process_time < self.process_interval:
                return
            self.last_process_time = current_time
            
            # 计算帧率
            self.fps = 1.0 / (current_time - self.prev_time)
            self.prev_time = current_time
            
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            self.cv_image = cv_image

            # 在图像上显示帧率
            cv2.putText(cv_image, f"FPS: {self.fps:.2f}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

            results = self.model.predict(source=cv_image, conf=0.3, save=False, verbose=False)
            result = results[0]

            if result.boxes and len(result.boxes) > 0:
                # 检测到人脸
                self.is_face_detected = True
                self.find_face_in_picture(cv_image, result.boxes, msg.header)
            else:
                # 未检测到人脸
                self.is_face_detected = False

            self.image_pub.publish(self.bridge.cv2_to_imgmsg(self.cv_image, "bgr8"))
        except Exception as e:
            rospy.logerr("图像转换失败: %s", e)
            self.is_face_detected = False
    
    def find_face_in_picture(self, cv_image, boxes, header):
        # 转为 NumPy 方便处理
        xyxy = boxes.xyxy.cpu().numpy()
        confs = boxes.conf.cpu().numpy()

        # 计算每个框的面积：(x2 - x1) * (y2 - y1)
        areas = (xyxy[:, 2] - xyxy[:, 0]) * (xyxy[:, 3] - xyxy[:, 1])
        
        # 过滤掉面积小于阈值的人脸
        valid_faces = areas >= self.min_face_area
        if not np.any(valid_faces):
            self.is_face_detected = False
            return
            
        # 在有效人脸中找最大的
        valid_areas = areas[valid_faces]
        valid_xyxy = xyxy[valid_faces]
        valid_confs = confs[valid_faces]
        max_idx = np.argmax(valid_areas)

        # 获取最大人脸框的坐标
        x1, y1, x2, y2 = map(int, valid_xyxy[max_idx])    # x1 y1 为左上角，x2 y2 为右下角
        confidence = valid_confs[max_idx]

        # 计算人脸框中心点
        center_x = (x1 + x2) / 2
        center_y = (y1 + y2) / 2

        # 将当前人脸中心坐标存在类属性中
        self.face_position_x = center_x
        self.face_position_y = center_y

        # 发布人脸检测框位置 (发布x1, y1, x2, y2所有四个坐标)
        face_bbox = FaceBoundingBox()
        face_bbox.header = header  # 添加时间戳信息
        face_bbox.x1 = x1
        face_bbox.y1 = y1
        face_bbox.x2 = x2
        face_bbox.y2 = y2
        face_bbox.confidence = confidence
        self.face_position_pub.publish(face_bbox)

        # 绘制人脸框
        cv2.rectangle(cv_image, (x1, y1), (x2, y2), (0, 255, 0), 2)
        # cv2.putText(cv_image, f"{confidence:.2f}", (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)
        # rospy.loginfo("人脸框位置: %d, %d", center_x, center_y)

    def update_head_state(self, msg):
        # 获取当前头部电机弧度制下的位置，转换为角度制
        # 头部电机为第22、23个关节（索引21、22）
        with self.motor_lock:
            self.head_yaw = msg.joint_data.joint_q[21] * 180 / np.pi
            self.head_pitch = msg.joint_data.joint_q[22] * 180 / np.pi
            # print("当前头部位置: ", self.head_yaw, self.head_pitch)
    
    def reset_head_position(self):
        # 发布头部复位消息
        msg = robotHeadMotionData()
        msg.joint_data = [0.0, 0.0]

        self.head_motion_pub.publish(msg)
    
    def send_head_motion_data(self, target_position):
        # 发布头部运动消息

        # 判断是否超出限位，如果超出的话则修改为限位值
        if target_position[0] < self.head_yaw_limit[0]:
            target_position[0] = self.head_yaw_limit[0]
        if target_position[0] > self.head_yaw_limit[1]:
            target_position[0] = self.head_yaw_limit[1]
        if target_position[1] < self.head_pitch_limit[0]:
            target_position[1] = self.head_pitch_limit[0]
        if target_position[1] > self.head_pitch_limit[1]:
            target_position[1] = self.head_pitch_limit[1]
        
        print("发布头部运动消息: ", target_position)
        msg = robotHeadMotionData()
        msg.joint_data = target_position



        self.head_motion_pub.publish(msg)

    def run(self):
        rate = rospy.Rate(30)  # 降低主循环频率
        while not rospy.is_shutdown():
            if self.is_face_detected:
                next_yaw = self.head_yaw
                next_pitch = self.head_pitch
                
                if self.face_position_x  < 512 or self.face_position_x > 768:
                    # 人脸在图像中水平方向上超出中心点，需要转动头部
                    print("人脸在图像中水平方向上超出中心点，需要转动头部: ", self.face_position_x)
                    cur_error_x = self.target_point_x - self.face_position_x
                    move_size_x = self.yaw_pid(cur_error_x)
                    next_yaw += move_size_x
                else:
                    self.yaw_pid.reset()

                if self.face_position_y < 300 or self.face_position_y > 420:
                    print("人脸在图像中垂直方向上超出中心点，需要转动头部: ", self.face_position_y)
                    cur_error_y = self.face_position_y - self.target_point_y
                    move_size_y = self.pitch_pid(cur_error_y)
                    next_pitch += move_size_y
                else:
                    self.pitch_pid.reset()
                
                self.send_head_motion_data([next_yaw, next_pitch])

            rate.sleep()

if __name__ == '__main__':
    face_tracker = FaceTrack()
    face_tracker.run()