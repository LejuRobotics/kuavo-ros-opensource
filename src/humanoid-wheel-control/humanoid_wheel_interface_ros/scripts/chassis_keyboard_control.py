#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
底盘键盘控制节点
直接通过底层接口 /move_base/base_cmd_vel 控制底盘运动
不依赖轮臂上层控制器，可用于硬件故障时移动底盘

操作说明:
    w/s : 前进/后退
    a/d : 左移/右移
    j/l : 左转/右转
    空格 : 急停
    +/- : 增加/减少速度
    Esc : 退出
"""

import sys
import select
import termios
import tty
import rospy
from geometry_msgs.msg import Twist

# 控制说明
HELP_MSG = """
============================================
    底盘直接控制 - 键盘模式
============================================
操作说明:
    w/s : 前进/后退 (按一次增加10%速度)
    a/d : 左移/右移 (按一次增加10%速度)
    j/l : 左转/右转 (按一次增加10%速度)
    空格 : 急停
    b/B : 退出程序
    h   : 显示帮助
    Esc/Ctrl+C : 退出

最大速度: 线速度 = {:.2f} m/s, 角速度 = {:.2f} rad/s
============================================
"""

class ChassisKeyboardControl:
    def __init__(self):
        rospy.init_node('chassis_keyboard_control', anonymous=True)
        
        # 底盘底层控制话题
        self.cmd_vel_pub = rospy.Publisher('/move_base/base_cmd_vel', Twist, queue_size=10)
        
        # 速度参数
        self.max_linear_speed = 0.3  # m/s (最大线速度)
        self.max_angular_speed = 0.3  # rad/s (最大角速度)
        self.speed_increment = 0.1    # 每次按键增加10%的速度
        self.speed_step = self.max_linear_speed * self.speed_increment  # 每次按键的速度增量 (0.03 m/s)
        
        # 当前速度
        self.current_vx = 0.0
        self.current_vy = 0.0
        self.current_wz = 0.0
        
        # 发布频率
        self.rate = rospy.Rate(50)  # 50Hz
        
        # 打印计数器（每10次打印一次）
        self.print_counter = 0
        
        # 终端设置
        self.old_settings = None
        
    def get_key(self, timeout=0.1):
        """非阻塞获取键盘输入"""
        if select.select([sys.stdin], [], [], timeout)[0]:
            key = sys.stdin.read(1)
            return key
        return None
        
    def print_help(self):
        """打印帮助信息"""
        print(HELP_MSG.format(self.max_linear_speed, self.max_angular_speed))
        
    def print_status(self):
        """打印当前状态"""
        sys.stdout.write(f"\r当前速度: vx={self.current_vx:.2f}, vy={self.current_vy:.2f}, wz={self.current_wz:.2f} | 按空格急停   ")
        sys.stdout.flush()
        
    def publish_cmd_vel(self):
        """发布速度命令"""
        msg = Twist()
        msg.linear.x = self.current_vx
        msg.linear.y = self.current_vy
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = self.current_wz
        self.cmd_vel_pub.publish(msg)
        
    def stop(self):
        """急停"""
        self.current_vx = 0.0
        self.current_vy = 0.0
        self.current_wz = 0.0
        self.publish_cmd_vel()
        print("\n[急停] 底盘已停止")
        
    def run(self):
        """主循环"""
        try:
            # 保存终端设置
            self.old_settings = termios.tcgetattr(sys.stdin)
            tty.setcbreak(sys.stdin.fileno())
            
            self.print_help()
            print("\n开始控制底盘，按 B 键、Esc 或 Ctrl+C 退出...")
            print("提示: 每次按键增加10%速度，逐步加速到最大速度\n")
            
            while not rospy.is_shutdown():
                key = self.get_key()
                
                if key is not None:
                    # 前进/后退 - 每次按键增加10%速度
                    if key == 'w':
                        # 增加vx（正向，最大0.3）
                        self.current_vx = min(self.current_vx + self.speed_step, self.max_linear_speed)
                    elif key == 's':
                        # 减小vx（负向，最大-0.3）
                        self.current_vx = max(self.current_vx - self.speed_step, -self.max_linear_speed)
                    # 左移/右移 - 每次按键增加10%速度
                    elif key == 'a':
                        # 增加vy（正向，最大0.3）
                        self.current_vy = min(self.current_vy + self.speed_step, self.max_linear_speed)
                    elif key == 'd':
                        # 减小vy（负向，最大-0.3）
                        self.current_vy = max(self.current_vy - self.speed_step, -self.max_linear_speed)
                    # 左转/右转 - 每次按键增加10%速度
                    elif key == 'j':
                        # 增加wz（正向，最大0.3）
                        self.current_wz = min(self.current_wz + self.speed_step, self.max_angular_speed)
                    elif key == 'l':
                        # 减小wz（负向，最大-0.3）
                        self.current_wz = max(self.current_wz - self.speed_step, -self.max_angular_speed)
                    # 急停
                    elif key == ' ':
                        self.stop()
                    # 退出 (B键)
                    elif key in ['b', 'B']:
                        print("\n退出控制...")
                        self.stop()
                        rospy.signal_shutdown("用户按B键退出")
                        break
                    # 帮助
                    elif key == 'h':
                        self.print_help()
                    # 退出 (Esc)
                    elif key == '\x1b':  # Esc
                        print("\n退出控制...")
                        self.stop()
                        break
                else:
                    # 无按键时，速度保持当前值（不自动衰减）
                    pass
                
                # 发布速度命令
                self.publish_cmd_vel()
                
                # 每10次循环打印一次状态
                self.print_counter += 1
                if self.print_counter >= 10:
                    self.print_status()
                    self.print_counter = 0
                
                self.rate.sleep()
                
        except KeyboardInterrupt:
            print("\n\n收到中断信号，停止底盘...")
            self.stop()
        except Exception as e:
            print(f"\n发生错误: {e}")
            self.stop()
        finally:
            # 恢复终端设置
            if self.old_settings:
                termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_settings)
            print("\n底盘键盘控制节点已退出")


def main():
    print("=" * 50)
    print("  底盘直接控制节点 - 键盘模式")
    print("  话题: /move_base/base_cmd_vel")
    print("  注意: 此节点直接控制底盘，不经过上层控制器")
    print("=" * 50)
    
    controller = ChassisKeyboardControl()
    controller.run()


if __name__ == '__main__':
    main()


