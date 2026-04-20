#!/usr/bin/env python3

import rospy
import cv2
import time
import threading
import numpy as np
import os
import math
import xml.etree.ElementTree as ET

from cv_bridge import CvBridge
from ultralytics import YOLO
from openvino.runtime import Core

from sensor_msgs.msg import Image, CompressedImage
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


def get_head_limits_from_urdf():
    """从 ROS 参数服务器的 URDF 中解析头部关节（zhead_1_joint=yaw, zhead_2_joint=pitch）限位。
    URDF 中 limit 单位为弧度，返回 (yaw_limit_deg, pitch_limit_deg)，单位为度。
    若解析失败返回 None。"""
    urdf_param_names = ['/humanoid_description', '/robot_description']
    urdf_str = None
    for name in urdf_param_names:
        try:
            if rospy.has_param(name):
                urdf_str = rospy.get_param(name)
                break
        except (rospy.ROSException, KeyError):
            continue
    if not urdf_str or not isinstance(urdf_str, str):
        return None
    try:
        root = ET.fromstring(urdf_str)
    except ET.ParseError:
        return None
    head_joint_names = ('zhead_1_joint', 'zhead_2_joint')  # yaw, pitch
    limits_rad = []
    for jname in head_joint_names:
        joint_elem = root.find(f".//joint[@name='{jname}']")
        if joint_elem is None:
            return None
        limit_elem = joint_elem.find('limit')
        if limit_elem is None:
            return None
        lower = float(limit_elem.get('lower', str(-math.pi)))
        upper = float(limit_elem.get('upper', str(math.pi)))
        limits_rad.append((lower, upper))
    yaw_deg = (math.degrees(limits_rad[0][0]), math.degrees(limits_rad[0][1]))
    pitch_deg = (math.degrees(limits_rad[1][0]), math.degrees(limits_rad[1][1]))
    return (yaw_deg, pitch_deg)


class FaceTrack:
    def __init__(self):
        rospy.init_node('face_track_node', anonymous=True)

        # 查询当前脚本所在路径，然后加载模型
        current_dir = os.path.dirname(os.path.abspath(__file__))
        model_dir = os.path.join(current_dir, 'yolov8n-face_openvino_model')
        model_path = os.path.join(model_dir, 'yolov8n-face.xml')

        rospy.loginfo("开始加载OpenVINO人脸识别模型: %s", model_path)
        self.ov_core = Core()
        self.device = rospy.get_param("~openvino_device", "CPU")
        
        # 限制OpenVINO线程数以降低CPU占用（必须在编译模型之前设置）
        try:
            num_threads = rospy.get_param("~openvino_num_threads", 2)  # 默认2线程
            self.ov_core.set_property("CPU", {"INFERENCE_NUM_THREADS": num_threads})
            rospy.loginfo("OpenVINO线程数设置为: %d", num_threads)
        except Exception as e:
            rospy.logwarn("无法设置OpenVINO线程数: %s", e)
        
        try:
            self.compiled_model = self.ov_core.compile_model(model_path, self.device)
        except Exception as exc:
            rospy.logerr("OpenVINO模型编译失败: %s", exc)
            raise
        self.infer_request = self.compiled_model.create_infer_request()
        self.input_tensor = self.compiled_model.inputs[0]
        self.output_tensor = self.compiled_model.outputs[0]
        input_shape = self.input_tensor.shape
        self.input_height = int(input_shape[2])
        self.input_width = int(input_shape[3])
        self.conf_threshold = rospy.get_param("~confidence_threshold", 0.6)

        self.bridge = CvBridge()
        rospy.loginfo("OpenVINO人脸识别模型加载完毕")

        self.face_position_x = 0    # 人脸框中点在图像中的位置
        self.face_position_y = 0
        self.target_point_x = 320    # 宽 640，目标点 x 坐标
        self.target_point_y = 240    # 高 480，目标点 y 坐标

        self.is_face_detected = False
        self.min_face_area = 1000  # 最小人脸面积阈值（像素）

        # 帧率计算相关变量
        self.prev_time = time.time()
        self.fps = 0

        # 获取最大处理帧率参数（用于控制CPU占用率）
        self.max_processing_fps = rospy.get_param("~max_processing_fps", 30.0)  # 默认30fps
        if self.max_processing_fps <= 0:
            rospy.logwarn("max_processing_fps参数无效，使用默认值30.0")
            self.max_processing_fps = 30.0
        self.min_processing_interval = 1.0 / self.max_processing_fps  # 最小处理间隔（秒）
        rospy.loginfo("最大处理帧率设置为: %.1f fps (处理间隔: %.3f秒)", 
                     self.max_processing_fps, self.min_processing_interval)

        # 异步处理相关变量 - 使用线程安全的队列实现实时处理
        self.latest_image_lock = threading.Lock()
        self.latest_image = None
        self.latest_image_msg = None  # 保存原始消息，延迟转换
        self.latest_image_header = None
        self.latest_image_timestamp = None
        self.processing_thread = None
        self.stop_processing = False

        self.head_yaw = 0.0
        self.head_pitch = 0.0

        # 从ROS参数服务器获取机器人版本号
        try:
            self.robot_version = rospy.get_param('robot_version', None)
        except rospy.ROSException:
            rospy.logwarn("无法从参数服务器获取robot_version参数")
            self.robot_version = None

        if self.robot_version is not None:
            robot_type = "roban" if (self.robot_version // 10) % 10 == 1 else "kuavo"
            if robot_type == "roban":
                yaw_kp, yaw_ki, yaw_kd = 0.045, 0.00, 0.001
                pitch_kp, pitch_ki, pitch_kd = 0.04, 0.00, 0.001
                default_yaw = (-90, 90)
                default_pitch = (-20, 45)
            else:
                yaw_kp, yaw_ki, yaw_kd = 0.15, 0.00, 0.001
                pitch_kp, pitch_ki, pitch_kd = 0.1, 0.00, 0.001
                default_yaw = (-90, 90)
                default_pitch = (-30, 30)
        else:
            yaw_kp, yaw_ki, yaw_kd = 0.055, 0.00, 0.001
            pitch_kp, pitch_ki, pitch_kd = 0.01, 0.00, 0.001
            default_yaw = (-90, 90)
            default_pitch = (-20, 45)

        # 优先从 URDF 获取头部限位（zhead_1_joint, zhead_2_joint），否则使用上述默认值
        head_limits = get_head_limits_from_urdf()
        if head_limits is not None:
            self.head_yaw_limit = head_limits[0]
            self.head_pitch_limit = head_limits[1]
            rospy.loginfo("头部限位已从 URDF 加载: yaw=%s deg, pitch=%s deg",
                          self.head_yaw_limit, self.head_pitch_limit)
        else:
            self.head_yaw_limit = default_yaw
            self.head_pitch_limit = default_pitch
            rospy.logwarn("无法从 URDF 获取头部限位，使用默认: yaw=%s deg, pitch=%s deg",
                          self.head_yaw_limit, self.head_pitch_limit)

        # 使用获取到的限位值和PID参数初始化PID控制器
        self.yaw_pid = PID(kp=yaw_kp, ki=yaw_ki, kd=yaw_kd, output_limits=self.head_yaw_limit)
        self.pitch_pid = PID(kp=pitch_kp, ki=pitch_ki, kd=pitch_kd, output_limits=self.head_pitch_limit)


        self.motor_lock = threading.Lock()

        # 检测指定话题是否存在
        self.image_sub = None
        self.use_compressed = False
        self.check_and_subscribe_to_camera()

        self.head_state_sub = rospy.Subscriber("/sensors_data_raw", sensorsData, self.update_head_state)
        self.head_motion_pub = rospy.Publisher("/robot_head_motion_data", robotHeadMotionData, queue_size=10)

        self.image_pub = rospy.Publisher("/camera/detected_face", Image, queue_size=1)  # 只发布最新结果
        # 修改发布器，使用新的FaceBoundingBox消息类型
        self.face_position_pub = rospy.Publisher("/face_detection/bounding_box", FaceBoundingBox, queue_size=1)  # 只发布最新结果
        
        self.cv_image = None
        
        # 启动后台处理线程
        self.stop_processing = False
        self.processing_thread = threading.Thread(target=self._processing_loop, daemon=True)
        self.processing_thread.start()
        rospy.loginfo("后台图像处理线程已启动")

    def check_and_subscribe_to_camera(self):
        """检查摄像头话题是否存在，并订阅第一个找到的话题"""
        # 获取当前发布的所有话题
        published_topics = rospy.get_published_topics()
        camera_topics = [topic for topic, _ in published_topics if ('image_raw' in topic or 'compressed' in topic) and ('camera' in topic or 'cam_h' in topic)]
        
        rospy.loginfo("发现的摄像头相关话题: %s", camera_topics)
        
        # 定义优先级话题列表（优先使用压缩话题）
        priority_topics = [
            "/camera/color/image_raw/compressed", 
            "/cam_h/color/image_raw/compressed",
            "/camera/color/image_raw", 
            "/cam_h/color/image_raw"
        ]
        
        # 根据优先级订阅第一个可用的话题
        subscribed = False
        for topic in priority_topics:
            if topic in camera_topics:
                # 判断是否为压缩话题
                if "/compressed" in topic:
                    # 设置 queue_size=1 确保只处理最新消息，丢弃旧消息，提高实时性
                    self.image_sub = rospy.Subscriber(topic, CompressedImage, self.compressed_image_callback, queue_size=1)
                    self.use_compressed = True
                else:
                    # 设置 queue_size=1 确保只处理最新消息，丢弃旧消息，提高实时性
                    self.image_sub = rospy.Subscriber(topic, Image, self.image_callback, queue_size=1)
                    self.use_compressed = False
                rospy.loginfo("已订阅话题: %s (压缩: %s, queue_size=1)", topic, self.use_compressed)
                subscribed = True
                break
        
        if not subscribed:
            rospy.logwarn("未找到任何可用的摄像头话题")

    def image_callback(self, msg):
        """快速回调函数，只保存最新图像，不进行耗时处理"""
        try:
            # 只在锁内进行最小操作，避免长时间持有锁
            with self.latest_image_lock:
                # 直接保存消息，延迟转换以减少回调时间
                self.latest_image_msg = msg
                # 保存原始图像的时间戳
                self.latest_image_header = msg.header
                self.latest_image_timestamp = time.time()
        except Exception as e:
            rospy.logerr("图像回调失败: %s", e)
    
    def compressed_image_callback(self, msg):
        """快速回调函数，只保存最新压缩图像，不进行耗时处理"""
        try:
            # 只在锁内进行最小操作，延迟转换以减少回调时间
            with self.latest_image_lock:
                self.latest_image_msg = msg
                # 压缩图像消息有header字段，使用原始header的时间戳
                # sensor_msgs/CompressedImage 有 header 字段
                self.latest_image_header = msg.header
                self.latest_image_timestamp = time.time()
        except Exception as e:
            rospy.logerr("压缩图像回调失败: %s", e)
    
    def _processing_loop(self):
        """后台处理循环，持续处理最新图像"""
        last_process_time = time.time()
        prev_time = time.time()
        last_image_timestamp = None  # 跟踪最后处理的图像时间戳
        
        while not self.stop_processing and not rospy.is_shutdown():
            try:
                current_time = time.time()
                
                # 控制处理频率：检查是否达到最小处理间隔
                time_since_last_process = current_time - last_process_time
                if time_since_last_process < self.min_processing_interval:
                    # 还没到处理时间，休眠剩余时间（避免CPU空转）
                    sleep_time = self.min_processing_interval - time_since_last_process
                    time.sleep(sleep_time)
                    continue
                
                # 获取最新图像消息（线程安全，快速操作）
                image_msg = None
                header = None
                image_timestamp = None
                with self.latest_image_lock:
                    if self.latest_image_msg is not None:
                        image_timestamp = self.latest_image_timestamp
                        # 只处理新图像，避免重复处理同一帧
                        if image_timestamp != last_image_timestamp:
                            image_msg = self.latest_image_msg
                            header = self.latest_image_header
                            last_image_timestamp = image_timestamp
                            # 清除消息，避免重复处理
                            self.latest_image_msg = None
                
                if image_msg is not None:
                    # 更新最后处理时间
                    last_process_time = time.time()
                    
                    # 确保header有效，如果为None则创建
                    if header is None:
                        header = Header()
                        header.stamp = rospy.Time.now()
                        rospy.logwarn("图像消息没有header，使用当前时间作为时间戳")
                    
                    # 转换图像（在锁外进行，避免长时间持有锁）
                    try:
                        if self.use_compressed:
                            cv_image = self.bridge.compressed_imgmsg_to_cv2(image_msg, desired_encoding="bgr8")
                        else:
                            cv_image = self.bridge.imgmsg_to_cv2(image_msg, desired_encoding="bgr8")
                    except Exception as e:
                        rospy.logerr("图像转换失败: %s", e)
                        time.sleep(self.min_processing_interval)
                        continue
                    
                    # 计算实际处理帧率
                    self.fps = 1.0 / (current_time - prev_time) if (current_time - prev_time) > 0 else 0
                    prev_time = current_time
                    
                    # 记录原图尺寸（用于将检测框坐标映射回原图再发布）
                    orig_h, orig_w = cv_image.shape[:2]
                    # 判断图像大小，不是指定分辨率帧resize的图片
                    if cv_image.shape != (480, 640, 3):
                        cv_image = cv2.resize(cv_image, (640, 480))
                    orig_size = (orig_w, orig_h)  # (width, height)
                    
                    # 保存用于显示（避免不必要的复制）
                    self.cv_image = cv_image
                    
                    # 在图像上显示帧率和最大帧率限制
                    cv2.putText(cv_image, f"FPS: {self.fps:.1f}/{self.max_processing_fps:.1f}", 
                               (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                    
                    # 执行推理
                    detections = self.run_openvino_inference(cv_image)
                    
                    if detections.size > 0:
                        # 检测到人脸
                        self.is_face_detected = True
                        self.find_face_in_picture(cv_image, detections, header, orig_size)
                    else:
                        # 未检测到人脸，发布空的人脸框消息（标记为无效）
                        self.publish_empty_face_bbox(header)
                        self.is_face_detected = False
                    
                    # 发布处理后的图像
                    self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
                else:
                    # 没有新图像，休眠较长时间以减少CPU占用
                    # 休眠时间设为最小处理间隔，确保能及时响应新图像
                    time.sleep(self.min_processing_interval)
                    
            except Exception as e:
                rospy.logerr("后台处理循环错误: %s", e)
                time.sleep(self.min_processing_interval)
    
    def run_openvino_inference(self, image):
        blob, scale, dwdh, original_size = self.preprocess_image(image)
        outputs = self.infer_request.infer({self.input_tensor: blob})
        # OpenVINO导出的YOLO模型通常只有一个输出
        detections = outputs[self.output_tensor]
        if detections.ndim == 3:
            detections = detections[0]
        detections = detections.astype(np.float32)
        return self.postprocess_detections(detections, scale, dwdh, original_size)

    def preprocess_image(self, image):
        h0, w0 = image.shape[:2]
        # 避免不必要的复制，直接使用原图像（resize会创建新图像）
        input_image = image
        r = min(self.input_width / w0, self.input_height / h0)
        new_unpad = (int(round(w0 * r)), int(round(h0 * r)))
        dw = (self.input_width - new_unpad[0]) / 2
        dh = (self.input_height - new_unpad[1]) / 2
        top = int(round(dh - 0.1))
        bottom = int(round(self.input_height - new_unpad[1] - top))
        left = int(round(dw - 0.1))
        right = int(round(self.input_width - new_unpad[0] - left))
        resized = cv2.resize(input_image, new_unpad, interpolation=cv2.INTER_LINEAR)
        padded = cv2.copyMakeBorder(resized, top, bottom, left, right, cv2.BORDER_CONSTANT, value=(114, 114, 114))
        padded = padded[:, :, ::-1]  # BGR to RGB
        padded = np.ascontiguousarray(padded)
        padded = padded.transpose(2, 0, 1)  # HWC to CHW
        padded = np.expand_dims(padded, axis=0).astype(np.float32) / 255.0
        return padded, r, (left, top), (h0, w0)

    def postprocess_detections(self, detections, scale, dwdh, original_size):
        if detections.size == 0:
            return np.empty((0, 6), dtype=np.float32)

        # detections: [num, 6] -> x1,y1,x2,y2,score,class
        x_offset, y_offset = dwdh
        h0, w0 = original_size
        processed = []
        for det in detections:
            if det.shape[0] < 6:
                continue
            score = det[4]
            if score < self.conf_threshold:
                continue
            cls_id = det[5]
            if cls_id != 0:
                continue
            x1, y1, x2, y2 = det[0], det[1], det[2], det[3]
            x1 = (x1 - x_offset) / scale
            y1 = (y1 - y_offset) / scale
            x2 = (x2 - x_offset) / scale
            y2 = (y2 - y_offset) / scale
            x1 = max(0, min(w0 - 1, x1))
            y1 = max(0, min(h0 - 1, y1))
            x2 = max(0, min(w0 - 1, x2))
            y2 = max(0, min(h0 - 1, y2))
            if x2 <= x1 or y2 <= y1:
                continue
            processed.append([x1, y1, x2, y2, score, cls_id])

        if not processed:
            return np.empty((0, 6), dtype=np.float32)
        return np.array(processed, dtype=np.float32)

    def publish_empty_face_bbox(self, header):
        face_bbox = FaceBoundingBox()
        face_bbox.header = header
        face_bbox.x1 = 0
        face_bbox.y1 = 0
        face_bbox.x2 = 0
        face_bbox.y2 = 0
        face_bbox.confidence = 0.0
        self.face_position_pub.publish(face_bbox)

    def publish_face_bbox(self, header, x1, y1, x2, y2, confidence):
        face_bbox = FaceBoundingBox()
        face_bbox.header = header
        face_bbox.x1 = x1
        face_bbox.y1 = y1
        face_bbox.x2 = x2
        face_bbox.y2 = y2
        face_bbox.confidence = confidence
        self.face_position_pub.publish(face_bbox)

    def find_face_in_picture(self, cv_image, detections, header, orig_size=None):
        xyxy = detections[:, :4]
        confs = detections[:, 4]

        # 计算每个框的面积：(x2 - x1) * (y2 - y1)
        areas = (xyxy[:, 2] - xyxy[:, 0]) * (xyxy[:, 3] - xyxy[:, 1])
        
        # 过滤掉面积小于阈值的人脸
        valid_faces = areas >= self.min_face_area
        if not np.any(valid_faces):
            self.is_face_detected = False
            self.publish_empty_face_bbox(header)
            return
            
        # 在有效人脸中找最大的
        valid_areas = areas[valid_faces]
        valid_xyxy = xyxy[valid_faces]
        valid_confs = confs[valid_faces]
        max_idx = np.argmax(valid_areas)

        # 获取最大人脸框的坐标（当前为处理图坐标系，如 640x480）
        x1, y1, x2, y2 = valid_xyxy[max_idx]
        x1, y1, x2, y2 = map(int, [x1, y1, x2, y2])
        confidence = valid_confs[max_idx]

        # 计算人脸框中心点（用于 PID 控制，保持在处理图坐标系）
        center_x = (x1 + x2) / 2
        center_y = (y1 + y2) / 2

        # 将当前人脸中心坐标存在类属性中
        self.face_position_x = center_x
        self.face_position_y = center_y

        # 将坐标映射回原图再发布
        if orig_size is not None:
            orig_w, orig_h = orig_size
            cur_h, cur_w = cv_image.shape[:2]
            scale_x = orig_w / float(cur_w)
            scale_y = orig_h / float(cur_h)
            x1_pub = int(round(x1 * scale_x))
            y1_pub = int(round(y1 * scale_y))
            x2_pub = int(round(x2 * scale_x))
            y2_pub = int(round(y2 * scale_y))
        else:
            x1_pub, y1_pub, x2_pub, y2_pub = x1, y1, x2, y2

        # 发布人脸检测框位置（原图坐标系下的 x1, y1, x2, y2）
        self.publish_face_bbox(header, x1_pub, y1_pub, x2_pub, y2_pub, confidence)

        # 绘制人脸框
        cv2.rectangle(cv_image, (x1, y1), (x2, y2), (0, 255, 0), 2)
        cv2.putText(cv_image, f"{confidence:.2f}", (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)
        rospy.loginfo("人脸框位置: %d, %d", center_x, center_y)

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
        
        # print("发布头部运动消息: ", target_position)
        msg = robotHeadMotionData()
        msg.joint_data = target_position



        self.head_motion_pub.publish(msg)

    def run(self):
        # 头部回中再开始运行
        rospy.loginfo("正在将头部回中...")
        # 持续发送复位命令1秒，确保头部能够到达中心位置
        reset_rate = rospy.Rate(10.0)  # 10Hz频率发送复位命令
        start_time = rospy.Time.now()
        while (rospy.Time.now() - start_time).to_sec() < 1.0:
            self.reset_head_position()
            reset_rate.sleep()
        rospy.loginfo("头部已回中，开始运行面部跟踪")
        
        # 获取主循环频率参数（用于控制CPU占用率）
        main_loop_rate = rospy.get_param("~main_loop_rate", 30.0)  # 默认30Hz
        rate = rospy.Rate(main_loop_rate)
        rospy.loginfo("主循环频率设置为: %.1f Hz", main_loop_rate)
        
        while not rospy.is_shutdown():
            if self.is_face_detected:
                next_yaw = self.head_yaw
                next_pitch = self.head_pitch
                
                if self.face_position_x  < 300 or self.face_position_x > 340:
                    # 人脸在图像中水平方向上超出中心点，需要转动头部
                    print("人脸在图像中水平方向上超出中心点，需要转动头部: ", self.face_position_x)
                    cur_error_x = self.target_point_x - self.face_position_x
                    move_size_x = self.yaw_pid(cur_error_x)
                    next_yaw += move_size_x
                else:
                    self.yaw_pid.reset()

                if self.face_position_y < 200 or self.face_position_y > 280:
                    print("人脸在图像中垂直方向上超出中心点，需要转动头部: ", self.face_position_y)
                    cur_error_y = self.face_position_y - self.target_point_y
                    move_size_y = self.pitch_pid(cur_error_y)
                    next_pitch += move_size_y
                else:
                    self.pitch_pid.reset()
                
                self.send_head_motion_data([next_yaw, next_pitch])
            else:
                # 没有检测到人脸时，降低循环频率以减少CPU占用
                time.sleep(0.03)  # 休眠100ms，降低CPU占用

            rate.sleep()
    
    def __del__(self):
        """析构函数，确保线程正确关闭"""
        self.stop_processing = True
        if self.processing_thread is not None and self.processing_thread.is_alive():
            self.processing_thread.join(timeout=1.0)

if __name__ == '__main__':
    try:
        face_tracker = FaceTrack()
        face_tracker.run()
    except KeyboardInterrupt:
        rospy.loginfo("正在关闭...")
    finally:
        if 'face_tracker' in locals():
            face_tracker.stop_processing = True
            if face_tracker.processing_thread is not None and face_tracker.processing_thread.is_alive():
                face_tracker.processing_thread.join(timeout=1.0)
