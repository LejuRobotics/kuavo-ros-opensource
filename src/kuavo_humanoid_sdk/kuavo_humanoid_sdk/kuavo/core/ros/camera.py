#!/usr/bin/env python3
# coding: utf-8
import rospy
from std_msgs.msg import Bool, Int16MultiArray
from kuavo_humanoid_sdk.common.logger import SDKLogger
from kuavo_humanoid_sdk.kuavo.core.core import KuavoRobotCore
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from kuavo_msgs.msg import yoloOutputData, yoloDetection
import threading

class CameraROSInterface:
    def __init__(self):
        self.bridge = CvBridge()
        
        self.cameras = {
            'head': {
                'topic': '/camera/color/image_raw',
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
        if not rospy.get_node_uri():
            rospy.init_node('YOLO_detection', anonymous=True)
        
        for camera in self.cameras.values():
            camera['publisher'] = rospy.Publisher(
                camera['pub_topic'], 
                yoloDetection, 
                queue_size=1, 
                tcp_nodelay=True
            )

    def get_camera_image(self, camera):
        if camera not in self.cameras:
            rospy.logerr(f"Unknown camera: {camera}")
            return None
            
        try:
            # 使用rospy.wait_for_message()获取一帧图像
            msg = rospy.wait_for_message(self.cameras[camera]['topic'], Image, timeout=1.0)
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.cv_image_shape = cv_image.shape
            return cv_image
        except rospy.ROSException as e:
            print(f"Timeout waiting for {camera} camera image: {e}")
            return None
        except CvBridgeError as e:
            rospy.logerr(f"{camera.capitalize()} Camera CvBridge Error: {e}")
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