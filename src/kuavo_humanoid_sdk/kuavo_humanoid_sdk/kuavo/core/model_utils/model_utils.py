#!/usr/bin/env python
import os
import argparse
import cv2
import torch
from ultralytics import YOLO
import numpy

from kuavo_humanoid_sdk.kuavo.core.ros.camera import CameraROSInterface

is_yolo_init = False
yolo_detection = None
model = None

class YOLO_detection:
    def __init__(self):
        self.camera_interface = CameraROSInterface()

        global is_yolo_init
        is_yolo_init = True

    def load_model(self, model_path):
        try:
            model = YOLO(model_path)
            print(f"Model loaded successfully from {model_path}")
            return model
        except Exception as e:
            print(f"Error loading model: {e}")
            return None

    def init_ros_node(self):
        self.camera_interface.init_ros_node()

    def get_detections(self, camera, model):
        image = self.camera_interface.get_camera_image(camera)
        
        if image is not None:
            results = model.predict(image, conf=0.6, show=False, verbose=False)
            self.publish_results(results, camera)
            return results
        else:
            print(f"No {camera} Camera Image...")
            return None

    def node_spin(self):
        self.camera_interface.node_spin()

    def node_is_shutdown(self):
        return self.camera_interface.node_is_shutdown()

    def tensor_to_msg(self, results):
        return self.camera_interface.tensor_to_msg(results)

    def show_results(self, results):
        if not results:
            print("No results to show.")
        else:
            yolo_detections = self.tensor_to_msg(results)
            print(yolo_detections)

    def publish_results(self, results, camera):
        self.camera_interface.publish_results(results, camera)

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

