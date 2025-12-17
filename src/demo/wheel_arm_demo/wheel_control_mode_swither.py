#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from kuavo_msgs.srv import changeTorsoCtrlMode, changeTorsoCtrlModeRequest

class ControlModeSwitcher:
    def __init__(self):
        # 等待服务可用
        rospy.wait_for_service('/mobile_manipulator_mpc_control')
        try:
            self.control_mode_client = rospy.ServiceProxy('/mobile_manipulator_mpc_control', changeTorsoCtrlMode)
            rospy.loginfo("Connected to control mode service successfully!")
        except rospy.ServiceException as e:
            rospy.logerr("Service connection failed: %s", e)
            return
        
        # 控制模式描述
        self.mode_descriptions = {
            0: "NoControl - no active control",
            1: "ArmOnly - controlling arms only, base fixed", 
            2: "BaseOnly - controlling base only, arms fixed",
            3: "BaseArm - controlling both base and arms",
            4: "ArmEeOnly - controlling arms Ee only"
        }
    
    def print_menu(self):
        """打印控制模式菜单"""
        print("\n" + "="*50)
        print("    Mobile Manipulator MPC Control Mode Switcher")
        print("="*50)
        for mode, desc in sorted(self.mode_descriptions.items()):
            print(f"  {mode}: {desc}")
        print("  q: Quit")
        print("="*50)
    
    def switch_mode(self, mode):
        """切换控制模式"""
        try:
            req = changeTorsoCtrlModeRequest()
            req.control_mode = mode
            resp = self.control_mode_client(req)
            
            if resp.result:
                print(f"✅ Successfully switched to mode {mode}: {resp.message}")
                return True
            else:
                print(f"❌ Failed to switch mode: {resp.message}")
                return False
                
        except rospy.ServiceException as e:
            print(f"❌ Service call failed: {e}")
            return False
    
    def run(self):
        """运行交互式终端"""
        print("Mobile Manipulator Control Mode Switcher Started!")
        print("Press Ctrl+C to exit at any time.")
        
        while not rospy.is_shutdown():
            try:
                self.print_menu()
                user_input = input("\nPlease select control mode (0-4) or 'q' to quit: ").strip()
                
                if user_input.lower() == 'q':
                    print("Exiting control mode switcher. Goodbye!")
                    break
                
                # 验证输入
                if user_input.isdigit():
                    mode = int(user_input)
                    if mode in self.mode_descriptions:
                        self.switch_mode(mode)
                    else:
                        print(f"❌ Invalid mode! Please enter a number between 0 and 4.")
                else:
                    print("❌ Invalid input! Please enter a number (0-4) or 'q' to quit.")
                    
            except KeyboardInterrupt:
                print("\n\nExiting control mode switcher. Goodbye!")
                break
            except Exception as e:
                print(f"❌ An error occurred: {e}")

def main():
    rospy.init_node('control_mode_switcher', anonymous=True)
    switcher = ControlModeSwitcher()
    switcher.run()

if __name__ == '__main__':
    main()