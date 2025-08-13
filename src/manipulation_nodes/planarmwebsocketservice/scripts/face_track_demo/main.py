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
from kuavo_msgs.msg import robotHeadMotionData, sensorsData

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
        self.min_face_area = 10000  # 最小人脸面积阈值（像素）

        # yaw 左右转动，pitch 上下转动
        self.yaw_pid = PID(kp=0.04, ki=0.0, kd=0.018, output_limits=(-90, 90))
        self.pitch_pid = PID(kp=0.0145, ki=0.0, kd=0.0155, output_limits=(-20, 30))
        self.head_yaw = 0.0
        self.head_pitch = 0.0
        self.head_yaw_limit = (-90, 90)
        self.head_pitch_limit = (-20, 30)

        self.motor_lock = threading.Lock()

        self.image_sub = rospy.Subscriber("/camera/color/image_raw", Image, self.image_callback)

        self.head_state_sub = rospy.Subscriber("/sensors_data_raw", sensorsData, self.update_head_state)
        self.head_motion_pub = rospy.Publisher("/robot_head_motion_data", robotHeadMotionData, queue_size=10)
    
    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

             # 人脸检测
            results = self.model.predict(source=cv_image, conf=0.3, save=False)
            result = results[0]

            if result.boxes and len(result.boxes) > 0:
                # 检测到人脸
                self.is_face_detected = True
                self.find_face_in_picture(cv_image, result.boxes)
            else:
                # 未检测到人脸
                self.is_face_detected = False
        except Exception as e:
            rospy.logerr("图像转换失败: %s", e)
            self.is_face_detected = False
    
    def find_face_in_picture(self, cv_image, boxes):
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
        rate = rospy.Rate(50)
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
                
                if self.face_position_y < 180 or self.face_position_y > 540:
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
