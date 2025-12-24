#!/usr/bin/env python3
from kuavo_msgs.srv import changeArmCtrlMode, changeArmCtrlModeRequest
import rospy
import os

def change_arm_control_mode(target_mode):
    """
    切换手臂控制模式
    
    Args:
        target_mode: 目标控制模式 (1: 正常模式, 2: 外部控制模式)
    
    Returns:
        bool: 是否成功切换
    """
    try:
        # 等待服务可用
        rospy.wait_for_service('/change_arm_ctrl_mode', timeout=10.0)
        
        # 创建服务代理
        arm_mode_service = rospy.ServiceProxy('/change_arm_ctrl_mode', changeArmCtrlMode)
        
        # 创建请求
        request = changeArmCtrlModeRequest()
        request.control_mode = target_mode
        
        rospy.loginfo(f"切换手臂控制模式到: {target_mode}")
        
        # 调用服务
        response = arm_mode_service(request)
        
        if response.result:
            rospy.loginfo(f"手臂控制模式切换成功: {response.mode}, 消息: {response.message}")
            return True
        else:
            rospy.logwarn(f"手臂控制模式切换失败: {response.message}")
            return False
            
    except rospy.ServiceException as e:
        rospy.logerr(f"手臂控制模式切换服务调用失败: {e}")
        return False
    except Exception as e:
        rospy.logerr(f"手臂控制模式切换过程中发生异常: {e}")
        return False

def main():
    """
    主函数
    """
    change_arm_control_mode(2)
    os.system('rosbag play kuavo_arm_traj.bag --topics /kuavo_arm_traj')


if __name__ == "__main__":
    main() 