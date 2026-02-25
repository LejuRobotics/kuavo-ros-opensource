#!/usr/bin/env python3
"""
电池监控脚本
监听 /battery 话题，当电量低于15%时设置行为树最大循环次数为1并前往充电，
当电量高于95%时停止充电并重启行为树。
"""

import rospy
from std_srvs.srv import Empty, EmptyRequest, SetBool, SetBoolRequest
from pytrees_actions.srv import SetMaxIterations, SetMaxIterationsRequest
from leju_mobile_base_msgs.msg import Battery


class BatteryMonitor:
    def __init__(self):
        rospy.init_node('battery_monitor', anonymous=True)

        # 电池阈值
        self.low_battery_threshold = 15.0  # 低电量阈值（%）
        self.full_battery_threshold = 95.0  # 满电量阈值（%）

        # 状态标志
        self.is_charging = False  # 是否正在充电
        self.behavior_tree_stopped = False  # 行为树是否已停止

        # 等待服务可用
        rospy.loginfo("[电池监控] 等待服务可用...")

        # 等待设置最大循环次数服务
        rospy.wait_for_service('/set_max_iterations', timeout=10.0)
        self.set_max_iterations_service = rospy.ServiceProxy('/set_max_iterations', SetMaxIterations)

        # 等待重启行为树服务
        rospy.wait_for_service('/restart_behavior_tree', timeout=10.0)
        self.restart_bt_service = rospy.ServiceProxy('/restart_behavior_tree', Empty)

        # 等待充电服务
        rospy.wait_for_service('/5w_charge', timeout=10.0)
        self.charge_service = rospy.ServiceProxy('/5w_charge', SetBool)

        # 等待速度控制服务
        rospy.wait_for_service('/enable_vel_control', timeout=10.0)
        self.enable_vel_control_service = rospy.ServiceProxy('/enable_vel_control', SetBool)

        rospy.loginfo("[电池监控] 所有服务已就绪")

        # 订阅电池话题
        rospy.Subscriber('/battery', Battery, self.battery_callback)

        rospy.loginfo("[电池监控] 电池监控节点已启动")
        rospy.loginfo(f"[电池监控] 低电量阈值: {self.low_battery_threshold}%")
        rospy.loginfo(f"[电池监控] 满电量阈值: {self.full_battery_threshold}%")

    def battery_callback(self, msg):
        """电池话题回调函数"""
        percentage = msg.percentage

        rospy.logdebug(f"[电池监控] 当前电量: {percentage:.1f}%")

        # 检查低电量
        if percentage < self.low_battery_threshold and not self.is_charging:
            rospy.logwarn(f"[电池监控] ⚠️ 电量过低 ({percentage:.1f}%)，开始充电流程")
            self.handle_low_battery()

        # 检查满电量
        elif percentage > self.full_battery_threshold and self.is_charging:
            rospy.loginfo(f"[电池监控] ✅ 电量已满 ({percentage:.1f}%)，停止充电并重启行为树")
            self.handle_full_battery()

    def handle_low_battery(self):
        """处理低电量情况"""
        try:
            # 1. 设置行为树最大循环次数为1，让当前流程执行完成
            if not self.behavior_tree_stopped:
                rospy.loginfo("[电池监控] 正在设置行为树最大循环次数为1...")
                max_iter_req = SetMaxIterationsRequest()
                max_iter_req.max_iterations = 1
                max_iter_resp = self.set_max_iterations_service(max_iter_req)

                if max_iter_resp.success:
                    self.behavior_tree_stopped = True
                    rospy.loginfo("[电池监控] ✅ 行为树最大循环次数已设置为1")
                else:
                    rospy.logerr(f"[电池监控] ❌ 设置最大循环次数失败: {max_iter_resp.message}")
                    return

            # 2. 等待200秒，确保当前完整流程执行完成
            rospy.loginfo("[电池监控] 等待200秒，确保当前完整流程执行完成...")
            rospy.sleep(200.0)
            rospy.loginfo("[电池监控] 等待完成")

            # 3. 调用速度控制服务，设置为false
            rospy.loginfo("[电池监控] 正在设置速度控制为false...")
            vel_control_req = SetBoolRequest()
            vel_control_req.data = False
            vel_control_resp = self.enable_vel_control_service(vel_control_req)

            if vel_control_resp.success:
                rospy.loginfo("[电池监控] ✅ 速度控制已设置为false")
            else:
                rospy.logwarn(f"[电池监控] ⚠️ 速度控制设置失败: {vel_control_resp.message}")

            # 4. 调用充电服务（true = 前往充电）
            rospy.loginfo("[电池监控] 正在调用充电服务，前往充电...")
            charge_req = SetBoolRequest()
            charge_req.data = True
            charge_resp = self.charge_service(charge_req)

            if charge_resp.success:
                self.is_charging = True
                rospy.loginfo("[电池监控] ✅ 充电服务调用成功，机器人正在前往充电")
            else:
                rospy.logerr(f"[电池监控] ❌ 充电服务调用失败: {charge_resp.message}")

        except rospy.ServiceException as e:
            rospy.logerr(f"[电池监控] ❌ 服务调用失败: {e}")
        except Exception as e:
            rospy.logerr(f"[电池监控] ❌ 处理低电量时发生错误: {e}")
            import traceback
            rospy.logerr(traceback.format_exc())

    def handle_full_battery(self):
        """处理满电量情况"""
        try:
            # 1. 停止充电（false = 停止充电）
            rospy.loginfo("[电池监控] 正在停止充电...")
            charge_req = SetBoolRequest()
            charge_req.data = False
            charge_resp = self.charge_service(charge_req)

            if charge_resp.success:
                self.is_charging = False
                rospy.loginfo("[电池监控] ✅ 充电服务停止成功")
            else:
                rospy.logerr(f"[电池监控] ❌ 停止充电服务调用失败: {charge_resp.message}")
                return

            # 2. 等待一小段时间确保充电完全停止
            rospy.sleep(1.0)

            # 3. 设置行为树最大循环次数为-1（无限循环）
            if self.behavior_tree_stopped:
                rospy.loginfo("[电池监控] 正在设置行为树最大循环次数为-1（无限循环）...")
                max_iter_req = SetMaxIterationsRequest()
                max_iter_req.max_iterations = -1
                max_iter_resp = self.set_max_iterations_service(max_iter_req)

                if max_iter_resp.success:
                    rospy.loginfo("[电池监控] ✅ 行为树最大循环次数已设置为-1")
                else:
                    rospy.logerr(f"[电池监控] ❌ 设置最大循环次数失败: {max_iter_resp.message}")
                    return

            # 4. 调用速度控制服务，设置为true
            rospy.loginfo("[电池监控] 正在设置速度控制为true...")
            vel_control_req = SetBoolRequest()
            vel_control_req.data = True
            vel_control_resp = self.enable_vel_control_service(vel_control_req)

            if vel_control_resp.success:
                rospy.loginfo("[电池监控] ✅ 速度控制已设置为true")
            else:
                rospy.logwarn(f"[电池监控] ⚠️ 速度控制设置失败: {vel_control_resp.message}")

            # 5. 重启行为树
            if self.behavior_tree_stopped:
                rospy.loginfo("[电池监控] 正在重启行为树...")
                self.restart_bt_service(EmptyRequest())
                self.behavior_tree_stopped = False
                rospy.loginfo("[电池监控] ✅ 行为树已重启")

        except rospy.ServiceException as e:
            rospy.logerr(f"[电池监控] ❌ 服务调用失败: {e}")
        except Exception as e:
            rospy.logerr(f"[电池监控] ❌ 处理满电量时发生错误: {e}")
            import traceback
            rospy.logerr(traceback.format_exc())

    def run(self):
        """运行监控节点"""
        rospy.spin()


if __name__ == '__main__':
    try:
        monitor = BatteryMonitor()
        monitor.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("[电池监控] 节点已关闭")
    except Exception as e:
        rospy.logerr(f"[电池监控] 节点运行错误: {e}")
        import traceback
        rospy.logerr(traceback.format_exc())

