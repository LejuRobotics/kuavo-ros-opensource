#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import sys
import tty
import termios
import select
import time
from geometry_msgs.msg import Twist
from ocs2_msgs.msg import mpc_observation
from std_msgs.msg import Float64

class PoseControlScript:
    def __init__(self):
        # 初始化ROS节点
        rospy.init_node('pose_control_script', anonymous=True)
        
        # 创建发布者和订阅者
        self.pub_cmd_pose_world = rospy.Publisher("/cmd_pose_world", Twist, queue_size=1)
        self.sub_mpc_obs = rospy.Subscriber("/humanoid_mpc_observation", mpc_observation, self.mpc_observation_callback)
        self.sub_terrain_height = rospy.Subscriber("/humanoid/mpc/terrainHeight", Float64, self.terrain_height_callback)
        
        # 当前机器人状态
        self.current_pose = {
            'x': 0.0,
            'y': 0.0, 
            'z': 0.0,
            'roll': 0.0,
            'pitch': 0.0,
            'yaw': 0.0
        }
        
        # 目标增量设置（可根据需要修改）
        self.pose_increment = {
            'x': 0.0,      # x方向增量
            'y': -2.3,      # y方向增量
            'z': 0.0,      # z方向增量
            'roll': 0.0,   # roll增量
            'pitch': 0.0,  # pitch增量
            'yaw': -1.57    # yaw增量（约90度）
        }
        
        # 地形高度
        self.terrain_height = 0.0
        
        # 状态标志
        self.has_received_observation = False
        self.running = True
        
        # 等待一下确保订阅者已注册
        rospy.sleep(1.0)
        rospy.loginfo("位置控制脚本已初始化")
        rospy.loginfo("等待接收机器人状态数据...")
        
    def mpc_observation_callback(self, msg):
        """处理MPC观测数据回调"""
        try:
            # 根据文档，状态向量索引：
            # 6-8: p_base_x, p_base_y, p_base_z (位置)
            # 9-11: theta_base_z, theta_base_y, theta_base_x (姿态，顺序为yaw, pitch, roll)
            if len(msg.state.value) >= 12:
                self.current_pose['x'] = msg.state.value[6]      # p_base_x
                self.current_pose['y'] = msg.state.value[7]      # p_base_y  
                self.current_pose['z'] = msg.state.value[8]      # p_base_z
                self.current_pose['yaw'] = msg.state.value[9]    # theta_base_z
                self.current_pose['pitch'] = msg.state.value[10] # theta_base_y
                self.current_pose['roll'] = msg.state.value[11]  # theta_base_x
                
                if not self.has_received_observation:
                    self.has_received_observation = True
                    rospy.loginfo("已接收到机器人状态数据")
                    # self.print_current_pose()
                    
        except Exception as e:
            rospy.logerr(f"解析MPC观测数据时出错: {e}")
    
    def terrain_height_callback(self, msg):
        """处理地形高度数据回调"""
        try:
            self.terrain_height = msg.data
            rospy.logdebug(f"接收到地形高度: {self.terrain_height:.3f}")
        except Exception as e:
            rospy.logerr(f"解析地形高度数据时出错: {e}")
    
    def print_current_pose(self):
        """打印当前机器人位置和姿态"""
        rospy.loginfo("当前机器人状态:")
        rospy.loginfo(f"  位置: x={self.current_pose['x']:.3f}, y={self.current_pose['y']:.3f}, z={self.current_pose['z']:.3f}")
        rospy.loginfo(f"  姿态: roll={self.current_pose['roll']:.3f}, pitch={self.current_pose['pitch']:.3f}, yaw={self.current_pose['yaw']:.3f}")
    
    def calculate_target_pose(self):
        """计算目标位置"""
        target_pose = {}
        for key in self.current_pose:
            target_pose[key] = self.current_pose[key] + self.pose_increment[key]
        # target_pose['z'] = self.terrain_height
        return target_pose
    
    def publish_target_pose(self, target_pose):
        """发布目标位置到cmd_pose_world话题"""
        twist_msg = Twist()
        twist_msg.linear.x = target_pose['x']
        twist_msg.linear.y = target_pose['y'] 
        twist_msg.linear.z = 0
        twist_msg.angular.x = 0
        twist_msg.angular.y = 0.0
        twist_msg.angular.z = target_pose['yaw']
        
        self.pub_cmd_pose_world.publish(twist_msg)
        print(twist_msg)
        rospy.loginfo("已发布目标位置:")
        rospy.loginfo(f"  目标位置: x={target_pose['x']:.3f}, y={target_pose['y']:.3f}, z={target_pose['z']:.3f}")
        rospy.loginfo(f"  目标姿态: roll={target_pose['roll']:.3f}, pitch={target_pose['pitch']:.3f}, yaw={target_pose['yaw']:.3f}")
        rospy.loginfo(f"  地形高度: {self.terrain_height:.3f}")
    
    def get_char(self):
        """获取单个字符输入（非阻塞）"""
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch
    
    def wait_for_enter_key(self):
        """等待用户按回车键"""
        rospy.loginfo("按回车键发送目标位置指令，按 'q' 退出...")
        
        while self.running and not rospy.is_shutdown():
            try:
                # 检查是否有键盘输入
                if sys.stdin in select.select([sys.stdin], [], [], 0)[0]:
                    key = self.get_char()
                    if key == '\r' or key == '\n':  # 回车键
                        if self.has_received_observation:
                            # 计算并发布目标位置
                            self.print_current_pose()
                            target_pose = self.calculate_target_pose()
                            self.publish_target_pose(target_pose)
                            rospy.loginfo("按回车键继续发送下一个指令，按 'q' 退出...")
                        else:
                            rospy.logwarn("尚未接收到机器人状态数据，请等待...")
                    elif key == 'q' or key == 'Q':
                        rospy.loginfo("用户请求退出...")
                        self.running = False
                        break
                else:
                    # 短暂休眠避免CPU占用过高
                    time.sleep(0.01)
            except KeyboardInterrupt:
                rospy.loginfo("接收到中断信号，正在退出...")
                self.running = False
                break
            except Exception as e:
                rospy.logerr(f"处理用户输入时出错: {e}")
                break
    
    def run(self):
        """运行主循环"""
        try:
            # 等待接收到机器人状态数据
            while not self.has_received_observation and not rospy.is_shutdown():
                rospy.loginfo_throttle(2, "等待机器人状态数据...")
                rospy.sleep(0.1)
            
            if rospy.is_shutdown():
                return
                
            # 显示当前状态和增量设置
            rospy.loginfo("=" * 50)
            self.print_current_pose()
            rospy.loginfo("增量设置:")
            rospy.loginfo(f"  x增量: {self.pose_increment['x']:.3f}")
            rospy.loginfo(f"  y增量: {self.pose_increment['y']:.3f}")
            rospy.loginfo(f"  z增量: {self.pose_increment['z']:.3f}")
            rospy.loginfo(f"  roll增量: {self.pose_increment['roll']:.3f}")
            rospy.loginfo(f"  pitch增量: {self.pose_increment['pitch']:.3f}")
            rospy.loginfo(f"  yaw增量: {self.pose_increment['yaw']:.3f}")
            rospy.loginfo(f"  当前地形高度: {self.terrain_height:.3f}")
            rospy.loginfo("=" * 50)
            
            # 等待用户输入
            self.wait_for_enter_key()
            
        except Exception as e:
            rospy.logerr(f"运行过程中出错: {e}")
        finally:
            rospy.loginfo("位置控制脚本已退出")

def main():
    try:
        # 创建并运行脚本
        script = PoseControlScript()
        script.run()
        
    except ImportError:
        rospy.logerr("缺少必要的模块，请确保在Linux环境下运行")
    except Exception as e:
        rospy.logerr(f"启动脚本时出错: {e}")

if __name__ == '__main__':
    main()
