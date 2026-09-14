#!/usr/bin/env python3
# coding: utf-8
import threading
from kuavo_humanoid_sdk.common.logger import SDKLogger
from kuavo_humanoid_sdk.common.websocket_kuavo_sdk import WebSocketKuavoSDK
from cv_bridge import CvBridge, CvBridgeError
import roslibpy


class CameraROSInterface:
    def __init__(self):
        self.bridge = CvBridge()
        self._is_shutdown = False

        self.cameras = {
            'head': {
                'topic': '/camera/color/image_raw/compressed',
                'pub_topic': '/model_output_data',
                'publisher': None
            },
            'chest': {
                'topic': '/chest_cam',
                'pub_topic': '/chest_detection_data',
                'publisher': None
            }
        }

        self.cv_image_shape = None

    def init_ros_node(self):
        """Initialize publishers for all cameras."""
        websocket = WebSocketKuavoSDK()
        for camera in self.cameras.values():
            camera['publisher'] = roslibpy.Topic(
                websocket.client, camera['pub_topic'], 'kuavo_msgs/yoloDetection')

    def get_camera_image(self, camera):
        if camera not in self.cameras:
            SDKLogger.error(f"Unknown camera: {camera}")
            return None

        try:
            websocket = WebSocketKuavoSDK()
            msg_type = 'sensor_msgs/CompressedImage' if camera == 'head' else 'sensor_msgs/Image'
            topic = roslibpy.Topic(websocket.client, self.cameras[camera]['topic'], msg_type)
            result = {}
            event = threading.Event()

            def _on_image(msg):
                result['msg'] = msg
                event.set()

            topic.subscribe(_on_image)
            if event.wait(timeout=1.0):
                msg = result['msg']
                topic.unsubscribe()
                cv_image = self.bridge.compressed_imgmsg_to_cv2(msg, "bgr8") if camera == 'head' else self.bridge.imgmsg_to_cv2(msg, "bgr8")
                self.cv_image_shape = cv_image.shape
                return cv_image
            else:
                topic.unsubscribe()
                SDKLogger.warn(f"Timeout waiting for {camera} camera image")
                return None
        except CvBridgeError as e:
            SDKLogger.error(f"{camera.capitalize()} Camera CvBridge Error: {e}")
            return None
        except Exception as e:
            SDKLogger.error(f"Error getting {camera} camera image: {e}")
            return None

    def node_spin(self):
        """No-op in D2 — WebSocket event loop handles concurrency."""
        pass

    def node_is_shutdown(self):
        """Check if shutdown has been requested."""
        return self._is_shutdown

    def shutdown(self):
        """Signal shutdown."""
        self._is_shutdown = True

    def tensor_to_msg(self, results):
        if not results:
            return None
        shape = self.cv_image_shape
        image_area = (shape[0] * shape[1]) if shape is not None and len(shape) >= 2 else 0.0

        data_entries = []
        for result in results:
            boxes = result.boxes.cpu().numpy()
            xywh = boxes.xywh
            class_ids = result.boxes.cls.int().cpu().numpy()
            class_names = [result.names[cls.item()] for cls in result.boxes.cls.int()]
            confs = boxes.conf

            for i in range(len(xywh)):
                box_w, box_h = xywh[i][2], xywh[i][3]
                box_area = box_w * box_h
                area_ratio = (box_area / image_area) if image_area > 0 else 0.0

                entry = {
                    "class_name": class_names[i],
                    "class_id": int(class_ids[i]),
                    "confidence": float(confs[i]),
                    "x_pos": float(xywh[i][0]),
                    "y_pos": float(xywh[i][1]),
                    "height": float(box_h),
                    "width": float(box_w),
                    "area_ratio": float(area_ratio)
                }
                data_entries.append(entry)

        return {"data": data_entries}

    def publish_results(self, results, camera):
        if not results:
            SDKLogger.info(f"No results to publish for {camera} camera.")
            return

        if camera not in self.cameras:
            SDKLogger.error(f"Unknown camera: {camera}")
            return

        yolo_detections = self.tensor_to_msg(results)
        if yolo_detections:
            self.cameras[camera]['publisher'].publish(roslibpy.Message(yolo_detections))
