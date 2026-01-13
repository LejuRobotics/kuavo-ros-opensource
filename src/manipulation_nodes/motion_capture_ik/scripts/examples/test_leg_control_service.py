#!/usr/bin/env python3
import rospy
from kuavo_msgs.srv import lbLegControlSrv, lbLegControlSrvRequest

def test_leg_control_service():
    rospy.init_node('test_leg_control_service', anonymous=True)
    rospy.wait_for_service('/lb_leg_control_srv', timeout=10.0)
    control_service = rospy.ServiceProxy('/lb_leg_control_srv', lbLegControlSrv)
    
    request = lbLegControlSrvRequest()
    # 奇怪的位置
    # request.target_joints = [0.6929002280736146, -0.7696885772144236, 0.0, -1.0455531607393092]

    # 初始化位置
    # request.target_joints = [0.314, -0.16, -0.157, 0.0]
    
    # 零位
    request.target_joints = [0.0, -0.0, -0.0, -0]
    request.duration= 5.0
    
    try:
        response = control_service(request)
        rospy.loginfo(f"控制设置: {'成功' if response.success else '失败'}")
    except Exception as e:
        rospy.logerr(f"调用失败: {e}")

if __name__ == '__main__':
    try:
        test_leg_control_service()
    except rospy.ROSInterruptException:
        pass 