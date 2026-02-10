#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from kuavo_msgs.srv import changeLbMpcObsUpdateModeSrv, changeLbMpcObsUpdateModeSrvRequest

class MpcObsUpdateModeSwitcher:
    def __init__(self):
        """初始化MPC观察更新模式切换器"""
        # 等待服务可用
        rospy.wait_for_service('/change_lb_mpc_obs_update_mode')
        try:
            self.mpc_obs_client = rospy.ServiceProxy('/change_lb_mpc_obs_update_mode', changeLbMpcObsUpdateModeSrv)
            rospy.loginfo("✅ Connected to MPC obs update mode service successfully!")
        except rospy.ServiceException as e:
            rospy.logerr("❌ Service connection failed: %s", e)
            return
        
        # 模式描述字典
        self.mode_descriptions = {
            0: "Full Feedback - 全反馈模式",
            1: "Lower Body Masked - 下肢屏蔽模式", 
            2: "Upper Body Masked - 上肢屏蔽模式",
            3: "Both Limbs Masked - 上下肢都屏蔽模式"
        }
    
    def print_menu(self):
        """打印交互式菜单"""
        print("\n" + "="*60)
        print("    MPC Observation Update Mode Switcher")
        print("="*60)
        for mode, desc in sorted(self.mode_descriptions.items()):
            print(f"  {mode}: {desc}")
        print("  q: Quit")
        print("="*60)
    
    def switch_mode(self, mode):
        """
        切换MPC观察更新模式
        Args:
            mode: 模式编号 (0-3)
        """
        try:
            # 创建请求对象
            req = changeLbMpcObsUpdateModeSrvRequest()
            req.obsUpdateMode = mode
            
            # 调用服务
            resp = self.mpc_obs_client(req)
            
            if resp.success:
                print(f"\n✅ 成功切换到模式 {mode}: {self.mode_descriptions[mode]}")
                return True
            else:
                print(f"\n❌ 切换模式失败: {resp.message if hasattr(resp, 'message') else 'Unknown error'}")
                return False
                
        except rospy.ServiceException as e:
            print(f"\n❌ 服务调用失败: {e}")
            return False
    
    def run(self):
        """运行交互式终端"""
        print("\nMPC Obs Update Mode Switcher Started!")
        print("Press Ctrl+C to exit at any time.")
        
        while not rospy.is_shutdown():
            try:
                self.print_menu()
                user_input = input("\n请输入模式编号 (0-3) 或 'q' 退出: ").strip()
                
                # 退出命令
                if user_input.lower() == 'q':
                    print("\n👋 退出MPC反馈更新模式切换器，再见！")
                    break
                
                # 验证输入
                if user_input.isdigit():
                    mode = int(user_input)
                    if mode in self.mode_descriptions:
                        self.switch_mode(mode)
                    else:
                        print(f"\n⚠️  无效模式! 请输入0到3之间的数字。")
                else:
                    print(f"\n⚠️  无效输入! 请输入数字 (0-3) 或 'q' 退出。")
                    
            except KeyboardInterrupt:
                print("\n\n👋 检测到Ctrl+C，退出程序。再见！")
                break
            except Exception as e:
                print(f"\n❌ 发生错误: {e}")

def main():
    """主函数"""
    rospy.init_node('mpc_obs_update_mode_switcher', anonymous=True)
    switcher = MpcObsUpdateModeSwitcher()
    switcher.run()

if __name__ == '__main__':
    main()