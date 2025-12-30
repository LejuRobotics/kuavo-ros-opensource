#!/usr/bin/env python
import rospy
import rospkg
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from kuavo_msgs.msg import yoloOutputData, yoloDetection

import os
import argparse
import cv2
import torch
import torchvision.transforms as transforms 
from ultralytics import YOLO
import numpy
import threading
import subprocess


class YOLO_detection:
    def __init__(self):
        self.bridge = CvBridge()
        self.latest_images = {}
        self.locks = {}
        
        self.cameras = {
            'head': {
                'topic': '/camera/color/image_raw',
                'pub_topic': '/model_output_data',
                'image': None,
                'lock': threading.Lock(),
                'publisher': None
            },
            'chest': {
                'topic': '/chest_cam',
                'pub_topic': '/chest_detection_data',
                'image': None,
                'lock': threading.Lock(),
                'publisher': None
            }
        }

    def load_model(self, model_path):
        try:
            model = YOLO(model_path)
            print(f"Model loaded successfully from {model_path}")
            return model
        except Exception as e:
            print(f"Error loading model: {e}")
            return None

    def head_cam_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            with self.cameras['head']['lock']:
                self.cameras['head']['image'] = cv_image
        except CvBridgeError as e:
            rospy.logerr(f"Head Camera CvBridge Error: {e}")
            return

    def chest_cam_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            with self.cameras['chest']['lock']:
                self.cameras['chest']['image'] = cv_image
        except CvBridgeError as e:
            rospy.logerr(f"Chest Camera CvBridge Error: {e}")
            return

    def init_ros_node(self):
        rospy.init_node('YOLO_detection', anonymous=True)
        
        for camera_name, camera_info in self.cameras.items():
            try:
                if camera_name == 'head':
                    rospy.Subscriber(camera_info['topic'], Image, self.head_cam_callback, queue_size=1)
                elif camera_name == 'chest':
                    rospy.Subscriber(camera_info['topic'], Image, self.chest_cam_callback, queue_size=1)
                print(f"Subscribed to {camera_name} camera topic")
            except Exception as e:
                print(f"Failed to subscribe to {camera_name} camera topic: {e}")
        
        for camera in self.cameras.values():
            camera['publisher'] = rospy.Publisher(
                camera['pub_topic'], 
                yoloDetection, 
                queue_size=1, 
                tcp_nodelay=True
            )

    def get_detections(self, camera, model):
        if camera not in self.cameras:
            rospy.logerr(f"Unknown camera: {camera}")
            return None
            
        with self.cameras[camera]['lock']:
            image = self.cameras[camera]['image']

        if image is not None:
            results = model.predict(image, show=False, verbose=False)
            self.publish_results(results, camera)
            return results
        else:
            rospy.loginfo(f"No {camera} Camera Image...")
            return None

    def node_spin(self):
        rospy.spin()

    def node_is_shutdown(self):
        return rospy.is_shutdown()

    def tensor_to_msg(self, results):
        if not results:
            return None
        else:
            yolo_detections = yoloDetection()
            for result in results:
                boxes = result.boxes.cpu().numpy()

                xywh = boxes.xywh  # center-x(i,0), center-y(i,1), width(i,2), height(i,3)
                class_ids = result.boxes.cls.int().cpu().numpy()
                class_names = [result.names[cls.item()] for cls in result.boxes.cls.int()]
                confs = boxes.conf

                for i in range(len(xywh)):
                    yolo_output_data = yoloOutputData(
                        class_name=class_names[i],
                        class_id=int(class_ids[i]),
                        confidence=confs[i], 
                        x_pos=xywh[i][0], 
                        y_pos=xywh[i][1], 
                        height=xywh[i][2], 
                        width=xywh[i][3]
                    )
                    yolo_detections.data.append(yolo_output_data)
            return yolo_detections

    def show_results(self, results):
        if not results:
            print("No results to show.")
        else:
            yolo_detections = self.tensor_to_msg(results)
            print(yolo_detections)

    def publish_results(self, results, camera):
        if not results:
            rospy.loginfo(f"No results to publish for {camera} camera.")
            return
            
        if camera not in self.cameras:
            rospy.logerr(f"Unknown camera: {camera}")
            return
            
        yolo_detections = self.tensor_to_msg(results)
        if yolo_detections:
            self.cameras[camera]['publisher'].publish(yolo_detections)

    def get_max_area_object(self, results):
        results = self.tensor_to_msg(results)
        if not results or len(results.data) == 0:
            return {'x': 0, 'y': 0, 'w': 0, 'h': 0, 'area': 0, 'class_id': 0}
        
        max_area = 0
        max_obj = None
        
        for detection in results.data:
            area = detection.width * detection.height
            if area > max_area:
                max_area = area
                max_obj = {
                    'x': detection.x_pos,
                    'y': detection.y_pos,
                    'w': detection.width,
                    'h': detection.height,
                    'area': area,
                    'class_id': detection.class_id
                }
        
        return max_obj if max_obj else {'x': 0, 'y': 0, 'w': 0, 'h': 0, 'area': 0, 'class_id': 0}

    def get_min_area_object(self, results):
        results = self.tensor_to_msg(results)
        if not results or len(results.data) == 0:
            return {'x': 0, 'y': 0, 'w': 0, 'h': 0, 'area': 0, 'class_id': 0}
        
        min_area = float('inf')
        min_obj = None
        
        for detection in results.data:
            area = detection.width * detection.height
            if area < min_area:
                min_area = area
                min_obj = {
                    'x': detection.x_pos,
                    'y': detection.y_pos,
                    'w': detection.width,
                    'h': detection.height,
                    'area': area,
                    'class_id': detection.class_id
                }
        
        return min_obj if min_obj else {'x': 0, 'y': 0, 'w': 0, 'h': 0, 'area': 0, 'class_id': 0}
