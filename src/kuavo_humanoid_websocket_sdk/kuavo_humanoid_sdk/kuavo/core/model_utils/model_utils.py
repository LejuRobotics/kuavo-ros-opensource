#!/usr/bin/env python
import os
import argparse
from kuavo_humanoid_sdk.kuavo.core.ros.camera import CameraROSInterface
from kuavo_humanoid_sdk.common.optional_deps import require_optional

is_yolo_init = False
yolo_detection = None
model = None

def _require_vision_deps():
    require_optional(["ultralytics", "torch"], "vision", "Vision")
    from ultralytics import YOLO
    return YOLO

class YOLO_detection:
    def __init__(self):
        self.camera_interface = CameraROSInterface()
        self.conf = 0.6  # default confidence threshold

        global is_yolo_init
        is_yolo_init = True

    def load_model(self, model_path):
        try:
            YOLO = _require_vision_deps()
            model = YOLO(model_path)
            print(f"Model loaded successfully from {model_path}")
            return model
        except Exception as e:
            print(f"Error loading model: {e}")
            return None

    def set_conf(self, conf):
        """
        Set the confidence threshold for detection.

        Args:
            conf (float): Confidence threshold, should be between 0 and 1
        """
        if 0 <= conf <= 1:
            self.conf = conf
            print(f"Confidence threshold set to {conf}")
        else:
            print("Invalid confidence value. Please provide a value between 0 and 1.")

    def init_ros_node(self):
        self.camera_interface.init_ros_node()

    def get_detections(self, camera, model, confidence=0.6):
        image = self.camera_interface.get_camera_image(camera)

        if image is not None:
            results = model.predict(image, conf=confidence, show=False, verbose=False)
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
        data_list = results.get('data', []) if isinstance(results, dict) else (results.data if hasattr(results, 'data') else [])
        if not data_list:
            return {'x': 0, 'y': 0, 'w': 0, 'h': 0, 'area': 0, 'area_ratio': 0.0, 'class_id': 0}

        max_area = 0
        max_obj = None

        for detection in data_list:
            if isinstance(detection, dict):
                w, h = detection.get('width', 0), detection.get('height', 0)
                area = w * h
            else:
                w, h = detection.width, detection.height
                area = w * h
            if area > max_area:
                max_area = area
                if isinstance(detection, dict):
                    max_obj = {
                        'x': detection.get('x_pos', 0),
                        'y': detection.get('y_pos', 0),
                        'w': detection.get('width', 0),
                        'h': detection.get('height', 0),
                        'area': area,
                        'area_ratio': detection.get('area_ratio', 0.0),
                        'class_id': detection.get('class_id', 0)
                    }
                else:
                    max_obj = {
                        'x': detection.x_pos,
                        'y': detection.y_pos,
                        'w': detection.width,
                        'h': detection.height,
                        'area': area,
                        'area_ratio': detection.area_ratio,
                        'class_id': detection.class_id
                    }

        return max_obj if max_obj else {'x': 0, 'y': 0, 'w': 0, 'h': 0, 'area': 0, 'area_ratio': 0.0, 'class_id': 0}

    def get_min_area_object(self, results):
        results = self.tensor_to_msg(results)
        data_list = results.get('data', []) if isinstance(results, dict) else (results.data if hasattr(results, 'data') else [])
        if not data_list:
            return {'x': 0, 'y': 0, 'w': 0, 'h': 0, 'area': 0, 'area_ratio': 0.0, 'class_id': 0}

        min_area = float('inf')
        min_obj = None

        for detection in data_list:
            if isinstance(detection, dict):
                w, h = detection.get('width', 0), detection.get('height', 0)
                area = w * h
            else:
                w, h = detection.width, detection.height
                area = w * h
            if area < min_area:
                min_area = area
                if isinstance(detection, dict):
                    min_obj = {
                        'x': detection.get('x_pos', 0),
                        'y': detection.get('y_pos', 0),
                        'w': detection.get('width', 0),
                        'h': detection.get('height', 0),
                        'area': area,
                        'area_ratio': detection.get('area_ratio', 0.0),
                        'class_id': detection.get('class_id', 0)
                    }
                else:
                    min_obj = {
                        'x': detection.x_pos,
                        'y': detection.y_pos,
                        'w': detection.width,
                        'h': detection.height,
                        'area': area,
                        'area_ratio': detection.area_ratio,
                        'class_id': detection.class_id
                    }

        return min_obj if min_obj else {'x': 0, 'y': 0, 'w': 0, 'h': 0, 'area': 0, 'area_ratio': 0.0, 'class_id': 0}
