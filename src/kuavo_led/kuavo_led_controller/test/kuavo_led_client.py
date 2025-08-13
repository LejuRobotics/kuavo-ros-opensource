#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from kuavo_led_controller.srv import SetLEDMode, SetLEDModeRequest, SetLEDMode_free, SetLEDMode_freeRequest
from std_srvs.srv import Trigger, TriggerRequest
from kuavo_led_controller.msg import Color
def set_led_color(colors):
    """
    调用LED控制服务设置LED颜色
    :param colors: 颜色列表，每个颜色为(R,G,B)元组
    """
    rospy.init_node('led_client', anonymous=True)
    
    # 等待服务可用
    rospy.wait_for_service('control_led')
    
    try:
        # 创建服务客户端
        led_service = rospy.ServiceProxy('control_led', SetLEDMode)
        
        # 创建请求
        request = SetLEDModeRequest()
        request.mode = 0  # 设置模式为1（呼吸模式）
        request.color1 = colors[0]
        request.color2 = colors[1]
        request.color3 = colors[2]
        request.color4 = colors[3]
        request.color5 = colors[4]
        request.color6 = colors[5]
        request.color7 = colors[6]
        request.color8 = colors[7]
        request.color9 = colors[8]
        request.color10 = colors[9]
        print(request)
        
        # 调用服务
        response = led_service(request)
        
        # 输出结果
        if response.success:
            rospy.loginfo("LED设置成功")
        else:
            rospy.logerr("LED设置失败")
            
    except rospy.ServiceException as e:
        rospy.logerr("服务调用失败: %s", e)

def set_led_color_free(colors):
    """
    调用LED控制服务设置LED颜色
    :param colors: 颜色列表，每个颜色为(R,G,B)元组
    """
    rospy.init_node('led_client', anonymous=True)
    
    # 等待服务可用
    rospy.wait_for_service('control_led_free')
    
    try:
        # 创建服务客户端
        led_service_free = rospy.ServiceProxy('control_led_free', SetLEDMode_free)
        
        # 创建请求
        request = SetLEDMode_freeRequest()
        request.mode = 0  # 设置模式为1（呼吸模式） 
        for i in colors:
            color = Color()
            color.r = i[0]
            color.g = i[1]
            color.b = i[2]
            request.colors.append(color)

        # 调用服务
        response = led_service_free(request)
        
        # 输出结果
        if response.success:    
            rospy.loginfo("LED设置成功")
        else:
            rospy.logerr("LED设置失败")
            
    except rospy.ServiceException as e:
        rospy.logerr("服务调用失败: %s", e)
        
def close_led():
    """
    调用关闭LED服务
    """
    rospy.init_node('led_client', anonymous=True)
    
    # 等待服务可用
    rospy.wait_for_service('close_led')
    
    try:
        # 创建服务客户端
        close_led_service = rospy.ServiceProxy('close_led', Trigger)
        
        # 调用服务
        response = close_led_service()
        
        # 输出结果
        if response.success:
            rospy.loginfo("LED关闭成功")
        else:
            rospy.logerr("LED关闭失败")
            
    except rospy.ServiceException as e:
        rospy.logerr("服务调用失败: %s", e)

if __name__ == '__main__':
    # 示例颜色列表，可以根据需要修改
    # 每个颜色为(R,G,B)元组，值范围为0-255
    colors = [
            (255, 0, 0),    # 红
            (255, 127, 0),  # 橙
            (255, 255, 0),  # 黄
            (0, 255, 0),    # 绿
            (0, 0, 255),    # 蓝
            (75, 0, 130),   # 靛
            (148, 0, 211),  # 紫
            (255, 0, 127),  # 粉
            (255, 255, 255),# 白
            (0, 255, 255)   # 青
        ]
    
    set_led_color(colors)

    rospy.sleep(5)

    set_led_color_free(colors)  

    rospy.sleep(5)

    close_led()

    
