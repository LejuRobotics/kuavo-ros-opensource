#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
磨线主控制器节点 - 简化版本
100Hz检查频率，持续发布start_together状态
- 如果手和脚都ready，持续发布start_together=True
- 如果有一个失败，发布start_together=False
"""

import rospy
from std_msgs.msg import Bool

class BreakinController:
    def __init__(self):
        # 初始化ROS节点
        rospy.init_node('breakin_controller', anonymous=True)
        
        # 状态标志
        self.arm_ready = False
        self.leg_ready = False
        
        # 创建发布者（使用latch确保最新状态可以被订阅者获取）
        self.pub_start = rospy.Publisher('/breakin/start_together', Bool, queue_size=10, latch=True)
        
        # 创建订阅者
        self.sub_arm_ready = rospy.Subscriber('/breakin/arm_ready', Bool, self.arm_ready_cb)
        self.sub_leg_ready = rospy.Subscriber('/breakin/leg_ready', Bool, self.leg_ready_cb)
        
        rospy.loginfo("已订阅 /breakin/arm_ready 和 /breakin/leg_ready")
        
        # 等待发布者和订阅者注册
        rospy.sleep(2.0)
        
        # 检查话题是否存在
        try:
            arm_ready_topics = rospy.get_published_topics('/breakin/arm_ready')
            leg_ready_topics = rospy.get_published_topics('/breakin/leg_ready')
            if arm_ready_topics:
                rospy.loginfo(f"检测到 /breakin/arm_ready 话题已发布")
            if leg_ready_topics:
                rospy.loginfo(f"检测到 /breakin/leg_ready 话题已发布")
        except Exception as e:
            rospy.logwarn(f"检查话题时出错: {e}")
        
        # 初始状态：发布False（因为还没有ready）
        initial_msg = Bool()
        initial_msg.data = False
        self.pub_start.publish(initial_msg)
        
        rospy.loginfo("磨线主控制器节点已启动")
        rospy.loginfo("检查频率: 100Hz")
        rospy.loginfo("等待手臂和腿部准备完成...")
    
    def arm_ready_cb(self, msg):
        """手臂准备状态回调"""
        old_arm_ready = self.arm_ready
        self.arm_ready = msg.data
        
        # 只在状态变化时打印日志（仅打印True，False可能是正常退出）
        if msg.data != old_arm_ready:
            if msg.data:
                rospy.loginfo("收到 arm_ready = True")
            # 不打印False，因为正常退出时也会变为False，不是失败
    
    def leg_ready_cb(self, msg):
        """腿部准备状态回调"""
        old_leg_ready = self.leg_ready
        self.leg_ready = msg.data
        
        # 只在状态变化时打印日志（仅打印True，False可能是正常退出）
        if msg.data != old_leg_ready:
            if msg.data:
                rospy.loginfo("收到 leg_ready = True")
            # 不打印False，因为正常退出时也会变为False，不是失败
    
    def run(self):
        """主循环 - 100Hz检查频率"""
        rate = rospy.Rate(100)  # 100Hz
        
        last_start_together_state = None
        last_status_print_time = rospy.get_time()
        
        while not rospy.is_shutdown():
            current_time = rospy.get_time()
            
            # 计算start_together的状态
            # 只有当两者都为True时，start_together才为True
            start_together_state = self.arm_ready and self.leg_ready
            
            # 如果状态发生变化，打印日志
            if start_together_state != last_start_together_state:
                if start_together_state:
                    rospy.loginfo("=" * 50)
                    rospy.loginfo("手臂和腿部都已准备完成！")
                    rospy.loginfo("开始持续发布 start_together = True (100Hz)")
                    rospy.loginfo("=" * 50)
                else:
                    # 不打印"失败"信息，因为正常退出时也会变为False
                    # rospy.logwarn("开始持续发布 start_together = False (100Hz)")
                    pass
                last_start_together_state = start_together_state
            
            # 持续发布当前状态（100Hz）
            msg = Bool()
            msg.data = start_together_state
            self.pub_start.publish(msg)
            
            # 移除定期状态打印（用户要求不输出）
            
            rate.sleep()

if __name__ == '__main__':
    try:
        controller = BreakinController()
        controller.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("磨线主控制器退出")
