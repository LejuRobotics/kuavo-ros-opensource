#!/usr/bin/env python3
# coding: utf-8

import rospy
from kuavo_msgs.msg import gestureInfo
from kuavo_msgs.srv import gestureList, gestureListRequest
from kuavo_msgs.msg import gestureTask
from kuavo_msgs.srv import gestureExecute, gestureExecuteRequest, gestureExecuteState, gestureExecuteStateRequest
import time

def excute_all_gestures_client():
    """请求并执行所有可用手势的服务"""
    # 服务的名称
    service_name = 'gesture/list'
    
    # 等待服务可用
    rospy.wait_for_service(service_name)
    
    try:
        # 创建服务代理
        list_gestures_service = rospy.ServiceProxy(service_name, gestureList)
        
        # 创建请求对象
        request = gestureListRequest()
        
        # 调用服务
        response = list_gestures_service(request)
        
        # 处理响应
        if response.success:
            print(f"Number of gestures: {response.gesture_count}")
            for gesture_info in response.gesture_infos:
                gesture_client(gesture_info.gesture_name, 0)
                time.sleep(3)
        else:
            print(f"Failed to list gestures: {response.message}")
    
    except rospy.ServiceException as e:
        print(f"Service call failed: {e}")

def gesture_client(gesture_name, hand_side):
    """请求执行特定手势的服务"""
    # 服务的名称，需要与服务器端注册的服务名一致
    service_name = 'gesture/execute'
    
    # 等待服务可用
    rospy.wait_for_service(service_name)
    
    try:
        # 创建服务代理
        gesture_service = rospy.ServiceProxy(service_name, gestureExecute)
        
        # 创建请求对象
        request = gestureExecuteRequest()
        gesture1 = gestureTask(gesture_name=gesture_name, hand_side=hand_side)
        request.gestures = [gesture1]
        
        # 调用服务
        response = gesture_service(request)
        
        # 处理响应
        if response.success:
            print(f"Gesture '{gesture_name}' executed successfully.")
        else:
            print(f"Failed to execute gesture '{gesture_name}': {response.message}")
    
    except rospy.ServiceException as e:
        print(f"Service call failed: {e}")


def gesture_state_client()->bool:
    service_name = 'gesture/execute_state'
    
    # 等待服务可用
    rospy.wait_for_service(service_name)
    
    try:
        # 创建服务代理
        gesture_state_service = rospy.ServiceProxy(service_name, gestureExecuteState)
        
        # 创建请求对象
        request = gestureExecuteStateRequest()
        
        response = gesture_state_service(request)
        
        return response.is_executing
    
    except rospy.ServiceException as e:
        print(f"Service call failed: {e}")
    
    return None

if __name__ == "__main__":
    # 初始化节点
    rospy.init_node('list_gestures_client')
    
    # 调用服务
    excute_all_gestures_client()