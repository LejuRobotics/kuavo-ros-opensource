#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
测试腿部IK服务
"""

import rospy
import numpy as np
from kuavo_msgs.srv import lbBaseLinkPoseCmdSrv, lbBaseLinkPoseCmdSrvRequest
from kuavo_msgs.srv import lbLegControlSrv, lbLegControlSrvRequest

def test_leg_ik_service():
    """测试腿部IK服务"""
    rospy.init_node('test_leg_ik_service', anonymous=True)
    
    # 等待服务可用
    rospy.loginfo("等待腿部IK服务...")
    rospy.wait_for_service('/lb_leg_ik_srv', timeout=10.0)
    
    # 等待腿部控制服务可用
    rospy.loginfo("等待腿部控制服务...")
    rospy.wait_for_service('/lb_leg_control_srv', timeout=10.0)
    
    # 创建服务代理
    ik_service = rospy.ServiceProxy('/lb_leg_ik_srv', lbBaseLinkPoseCmdSrv)
    control_service = rospy.ServiceProxy('/lb_leg_control_srv', lbLegControlSrv)
    
    # 创建测试请求
    request = lbBaseLinkPoseCmdSrvRequest()
    
    # 设置请求参数
    request.with_chassis = True
    request.chassis_info = [0.0, 0.0, 0.0]  # [x, y, yaw]
    request.q_lb = [0.21, -0.375, 0.0202, 0.0]  # [knee, leg, waist_pitch, waist_yaw]
    request.control_type = 0  # 0-底盘优先，1-腰部优先
    
    # 设置目标base_link位姿 [x, y, z, qw, qx, qy, qz]
    request.base_link = [0.22716442, -0.01699162, 1.01857916, 0.8072142642317506, 0.018572800501078318, -0.025435662725308866, -0.5894176870132634]
    
    rospy.loginfo("发送IK请求...")
    rospy.loginfo(f"with_chassis: {request.with_chassis}")
    rospy.loginfo(f"chassis_info: {request.chassis_info}")
    rospy.loginfo(f"q_lb: {request.q_lb}")
    rospy.loginfo(f"control_type: {request.control_type} ({'底盘控制' if request.control_type == 0 else '腰部控制'})")
    rospy.loginfo(f"base_link: {request.base_link}")
    
    try:
        # 调用IK服务
        ik_response = ik_service(request)
        
        if ik_response.success:
            rospy.loginfo("IK求解成功！")
            rospy.loginfo(f"腿部关节角度: {ik_response.lb_leg}")
            rospy.loginfo(f"求解耗时: {ik_response.time_cost:.2f} ms")
            
            # 获得逆解结果后，调用控制服务执行这些关节角度
            rospy.loginfo("开始执行控制服务...")
            
            # 创建控制服务请求
            control_request = lbLegControlSrvRequest()
            control_request.target_joints = ik_response.lb_leg
            
            rospy.loginfo(f"发送控制请求，目标关节角度: {control_request.target_joints}")
            
            # 调用控制服务
            control_response = control_service(control_request)
            
            if control_response.success:
                rospy.loginfo("控制服务调用成功！机器人开始执行目标关节角度")
            else:
                rospy.logwarn("控制服务调用失败")
                
        else:
            rospy.logwarn("IK求解失败")
            rospy.logwarn(f"求解耗时: {ik_response.time_cost:.2f} ms")
            
    except rospy.ServiceException as e:
        rospy.logerr(f"服务调用失败: {e}")

def main():
    try:
        test_leg_ik_service()
    except KeyboardInterrupt:
        print("\n用户中断程序")
    except Exception as e:
        rospy.logerr(f"程序异常: {e}")

if __name__ == '__main__':
    main() 