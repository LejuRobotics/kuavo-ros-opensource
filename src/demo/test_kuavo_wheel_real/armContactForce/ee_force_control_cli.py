#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""End-Effector Force Control CLI - 终端交互式期望力控制脚本"""

import rospy

from lb_force_ctrl import GRAVITY, LBForceController


class EEForceControlCLI:
    def __init__(self):
        rospy.init_node('ee_force_control_cli', anonymous=True)
        self.force_ctrl = LBForceController(wait_for_connection=True, connection_delay=0.5)
        self.gravity = GRAVITY
        self.max_force_kg = 60.0 / self.gravity  # 最大力值60N转换为kg

        rospy.loginfo("期望力控制CLI已启动")
        print("\n" + "="*50)
        print("  注意：力的方向是以机器人本体作为参考坐标系")
        print("="*50)
    
    def validate_force(self, force_kg):
        """验证力值是否在允许范围内"""
        if abs(force_kg) > self.max_force_kg:
            return False, f"力值超过最大限制 {self.max_force_kg:.2f} kg (60N)，请重新输入"
        return True, None
    
    def publish_force(self, hand, force_x_kg, force_y_kg, force_z_kg):
        """发布单个手的力值（将 kg 转换为 N）"""
        force_kg = (force_x_kg, force_y_kg, force_z_kg)
        self.force_ctrl.set_desired_ee_force_kg(hand, force_kg=force_kg, torque=(0.0, 0.0, 0.0))

        force_x_n, force_y_n, force_z_n = [value * self.gravity for value in force_kg]
        hand_name = {'left': '左手', 'right': '右手', 'both': '双手'}[hand]
        rospy.loginfo(
            f"已发布{hand_name}期望力: "
            f"x={force_x_kg:.2f}kg ({force_x_n:.2f}N), "
            f"y={force_y_kg:.2f}kg ({force_y_n:.2f}N), "
            f"z={force_z_kg:.2f}kg ({force_z_n:.2f}N)"
        )
    
    def input_force_value(self, axis_name):
        """输入单个方向的力值"""
        while True:
            try:
                user_input = input(f"请输入{axis_name}方向的力值(kg，回车默认为0): ").strip()
                if user_input == "":
                    return 0.0
                force_kg = float(user_input)
                is_valid, error_msg = self.validate_force(force_kg)
                if is_valid:
                    return force_kg
                else:
                    print(f"错误: {error_msg}")
            except ValueError:
                print("错误: 请输入有效的数字")
    
    def input_force_option(self):
        """选项1：输入期望力"""
        print("\n=== 输入期望力 ===")
        print("请选择要设置的手：")
        print("1. 左手")
        print("2. 右手")
        print("3. 双手")
        
        while True:
            choice = input("请输入选择(1/2/3): ").strip()
            if choice == '1':
                hand = 'left'
                hand_name = '左手'
                break
            elif choice == '2':
                hand = 'right'
                hand_name = '右手'
                break
            elif choice == '3':
                hand = 'both'
                hand_name = '双手'
                break
            else:
                print("错误: 请输入1、2或3")
        
        print(f"\n正在设置{hand_name}的期望力...")
        force_x = self.input_force_value('x')
        force_y = self.input_force_value('y')
        force_z = self.input_force_value('z')
        
        print(f"\n您输入的{hand_name}期望力为:")
        print(f"  x方向: {force_x:.2f} kg ({force_x * self.gravity:.2f} N)")
        print(f"  y方向: {force_y:.2f} kg ({force_y * self.gravity:.2f} N)")
        print(f"  z方向: {force_z:.2f} kg ({force_z * self.gravity:.2f} N)")
        
        while True:
            confirm = input("\n确认发送? (y/n): ").strip().lower()
            if confirm == 'y':
                self.publish_force(hand, force_x, force_y, force_z)
                print("期望力已发送！")
                break
            elif confirm == 'n':
                print("已取消发送")
                break
            else:
                print("错误: 请输入y或n")
    
    def zero_force_option(self):
        """选项2：期望力归零"""
        print("\n=== 期望力归零 ===")
        print("正在将左右手期望力归零...")
        self.publish_force('left', 0.0, 0.0, 0.0)
        self.publish_force('right', 0.0, 0.0, 0.0)
        print("左右手期望力已归零！")
    
    def quick_start_option(self):
        """选项3：快速启动（左右手z方向各-3kg）"""
        print("\n=== 快速启动 ===")
        force_z = -3.0
        print("将设置左右手期望力为:")
        print("  x方向: 0.00 kg (0.00 N)")
        print("  y方向: 0.00 kg (0.00 N)")
        print(f"  z方向: {force_z:.2f} kg ({force_z * self.gravity:.2f} N)")
        
        while True:
            confirm = input("\n确认发送? (y/n): ").strip().lower()
            if confirm == 'y':
                self.publish_force('left', 0.0, 0.0, force_z)
                self.publish_force('right', 0.0, 0.0, force_z)
                print("快速启动完成！左右手z方向已设置为-3kg (-29.4N)")
                break
            elif confirm == 'n':
                print("已取消发送")
                break
            else:
                print("错误: 请输入y或n")
    
    def show_menu(self):
        """显示主菜单"""
        print("\n" + "="*50)
        print("        期望力控制终端")
        print("="*50)
        print("1. 输入期望力")
        print("2. 期望力归零")
        print("3. 快速启动（双手期望力为-3kg）")
        print("="*50)
    
    def run(self):
        """运行主循环"""
        try:
            while not rospy.is_shutdown():
                self.show_menu()
                choice = input("\n请选择操作(1/2/3，输入q退出): ").strip().lower()
                
                if choice == 'q':
                    print("\n程序已退出")
                    rospy.loginfo("期望力控制CLI已关闭")
                    break
                elif choice == '1':
                    self.input_force_option()
                elif choice == '2':
                    self.zero_force_option()
                elif choice == '3':
                    self.quick_start_option()
                else:
                    print("错误: 请输入1、2、3或q")
        except KeyboardInterrupt:
            print("\n\n程序已退出")
            rospy.loginfo("期望力控制CLI已关闭")


def main():
    try:
        cli = EEForceControlCLI()
        cli.run()
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main()

