#!/usr/bin/env python3
import rospy
import numpy as np
from sensor_msgs.msg import JointState
from scipy.interpolate import CubicSpline
from std_srvs.srv import SetBool, SetBoolRequest
from kuavo_msgs.srv import changeLbQuickModeSrv, changeLbQuickModeSrvRequest

class SimpleArmTrajectory:
    def __init__(self):
        rospy.init_node('simple_arm_trajectory')
        self.pub = rospy.Publisher('/kuavo_arm_traj', JointState, queue_size=10)
        self.joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6', 'joint7', 
                            'joint8', 'joint9', 'joint10', 'joint11', 'joint12', 'joint13', 'joint14']
        
    def publish_trajectory(self, time_points, joint_angles_list):
        """
        发布轨迹
        :param time_points: 时间点列表 [t0, t1, t2, ...]
        :param joint_angles_list: 14关节角度列表 [[j1,j2,...,j14], [j1,j2,...,j14], ...]
        """
        # 转换为numpy数组
        times = np.array(time_points)
        angles = np.array(joint_angles_list).T  # 转置以便每行对应一个关节
        
        # 创建插值器
        interpolators = [CubicSpline(times, angles[i]) for i in range(14)]
        
        # 执行轨迹
        rate = rospy.Rate(50)
        start_time = rospy.Time.now().to_sec()
        
        for t in np.arange(times[0], times[-1], 0.1):  # 50Hz
            if rospy.is_shutdown():
                break
                
            elapsed = rospy.Time.now().to_sec() - start_time
            if elapsed > times[-1]:
                break
                
            # 计算当前关节位置
            current_angles = [interp(t) for interp in interpolators]
            
            # 发布消息
            msg = JointState()
            msg.header.stamp = rospy.Time.now()
            msg.name = self.joint_names
            msg.position = [angle for angle in current_angles]  
            
            self.pub.publish(msg)
            rate.sleep()

def set_arm_quick_mode(quickMode):
    """
    设置手臂快速模式
    Args:
        全身快速模式类型: 0-关闭, 1-下肢快, 2-上肢快, 3-上下肢快
    """
    print(f"call set_arm_quick_mode:{quickMode}")
    rospy.wait_for_service('/enable_lb_arm_quick_mode')
    try:
        set_arm_quick_mode_service = rospy.ServiceProxy('/enable_lb_arm_quick_mode', changeLbQuickModeSrv)
        req = changeLbQuickModeSrvRequest()
        req.quickMode = quickMode
        resp = set_arm_quick_mode_service(req)
        if resp.success:
            rospy.loginfo(f"Successfully enabled {quickMode} quick mode")
        else:
            rospy.logwarn(f"Failed to enable {quickMode} quick mode")
    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")

# 使用示例
if __name__ == '__main__':
    try:

        # 启用手臂快速模式
        set_arm_quick_mode(2)

        traj = SimpleArmTrajectory()
        
        # 定义轨迹点
        times = [0.0, 4.0, 8.0, 12.0]  # 时间点列表，单位为秒
        
        # 14个关节的角度轨迹 (单位: 度) - 更大范围的运动
        angles = [
            # 时间点 0.0s - 初始位置 (所有关节为0)
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
             0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            
            # 时间点 3.0s - 展开手臂
            [-30.0, 20.0, 15.0, -45.0, 25.0, 10.0, -35.0, 
             -30.0, -20.0, -15.0, -45.0, -25.0, -10.0, -35.0],
            
            # 时间点 6.0s - 弯曲手臂
            [-20.0, 30.0, -25.0, -20.0, 40.0, -15.0, 25.0, 
             -20.0, -30.0, 25.0, -20.0, -40.0, 15.0, 25.0],
            
            # 时间点 12.0s - 回到初始位置
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
             0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        ]
        
        traj.publish_trajectory(times, angles)
        print("轨迹执行完成")
        
        # 禁用手臂快速模式
        set_arm_quick_mode(0)
        
    except rospy.ROSInterruptException:
        pass