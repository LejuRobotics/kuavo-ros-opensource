#!/usr/bin/env python3

import rospy
from vision_msgs.msg import Detection2DArray
from collections import defaultdict

class ObjectPositionTracker:
    def __init__(self):
        rospy.init_node('object_position_tracker', anonymous=True)
        
        # 初始化存储结构
        self.object_positions = defaultdict(list)  # 按标签ID存储位置
        self.latest_positions = {}                 # 每个ID的最新位置
        
        # 订阅 YOLOv8 检测结果
        self.sub = rospy.Subscriber(
            '/robot_yolov8_info', 
            Detection2DArray, 
            self.detection_callback
        )
        
        rospy.loginfo("ObjectPositionTracker initialized. Waiting for detections...")
    
    def detection_callback(self, msg):
        """处理检测结果回调"""
        # 清空前一次的结果
        self.object_positions.clear()
        self.latest_positions.clear()
        
        # 处理每个检测结果
        for detection in msg.detections:
            # 确保有检测结果
            if not detection.results:
                continue
                
            # 获取第一个结果（通常只有一个）
            result = detection.results[0]
            obj_id = result.id
            
            # 获取位置信息
            position = result.pose.pose.position
            x, y, z = position.x, position.y, position.z
            
            # 存储位置
            self.object_positions[obj_id].append((x, y, z))
            self.latest_positions[obj_id] = (x, y, z)
        
        # 打印调试信息（可选）
        self.print_positions()
    
    def get_positions_by_id(self, obj_id):
        """获取特定ID的所有位置"""
        return self.object_positions.get(obj_id, [])
    
    def get_latest_position_by_id(self, obj_id):
        """获取特定ID的最新位置"""
        return self.latest_positions.get(obj_id, None)
    
    def get_all_positions(self):
        """获取所有检测到的位置"""
        return dict(self.object_positions)
    
    def get_all_latest_positions(self):
        """获取所有检测的最新位置"""
        return dict(self.latest_positions)
    
    def print_positions(self):
        """打印位置信息（调试用）"""
        if not self.object_positions:
            rospy.loginfo("No objects detected")
            return
            
        rospy.loginfo("=== Detected Objects ===")
        for obj_id, positions in self.object_positions.items():
            rospy.loginfo(f"ID {obj_id}:")
            for i, (x, y, z) in enumerate(positions):
                rospy.loginfo(f"  Object {i+1}: x={x:.3f}, y={y:.3f}, z={z:.3f}")
        rospy.loginfo("========================")

if __name__ == '__main__':
    processor = ObjectPositionTracker()
    
    # 设置要监控的物体ID
    target_ids = [39, 41]  # 瓶子和杯子
    # 主循环
    rate = rospy.Rate(10)  # 10Hz
    while not rospy.is_shutdown():
        cup_position = None
        bottle_position = None
        # 获取特定ID的最新位置
        for obj_id in target_ids:
            position = processor.get_latest_position_by_id(obj_id)
            if position:
                if obj_id == 39:
                    cup_position = position
                    x, y, z = cup_position
                    rospy.loginfo(f"Position of ID {obj_id}: "
                                f"x={x:.3f}, y={y:.3f}, z={z:.3f}")
                elif obj_id == 41:
                    bottle_position = position
                    x, y, z = bottle_position
                    rospy.loginfo(f"Position of ID {obj_id}: "
                                f"x={x:.3f}, y={y:.3f}, z={z:.3f}")
        # 两个标签都识别到,则进行下一步
        # if cup_position != None and bottle_position != None:
        #     break
        rate.sleep()
