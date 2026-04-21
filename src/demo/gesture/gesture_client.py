#!/usr/bin/env python3
# coding: utf-8

import sys
import rospy
import time
from kuavo_msgs.msg import gestureTask
from kuavo_msgs.srv import gestureExecute, gestureExecuteRequest, gestureExecuteState, gestureExecuteStateRequest

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
    rospy.init_node('gesture_client')
    
    # 获取手势名称，如果没有提供参数，则使用默认值 "ok"
    gesture_name = sys.argv[1] if len(sys.argv) > 1 else "ok"
    
    # 定义手部位置
    hand_side = 0  # 左手
    
    # 调用服务
    gesture_client(gesture_name, hand_side)
    
    # 循环 20 次，每次 sleep 100ms
    for _ in range(20):
        state = gesture_state_client()
        if state is not None:
            print(f"Current gesture state: {state}")
        time.sleep(0.1)  # 等待 100ms

    # reset hand gesture.    
    # gesture_client("empty", hand_side=2)