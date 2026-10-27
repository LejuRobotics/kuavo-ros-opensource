#!/usr/bin/env python3
import rospy
import sys
import time
from std_srvs.srv import SetBool, SetBoolRequest, Trigger, TriggerRequest

class TurboModeServiceTester:
    def __init__(self):
        rospy.init_node('turbo_mode_service_tester', anonymous=True)

        # 服务名称
        self.set_services = {
            'left': '/dexhand/left/set_turbo_mode',
            'right': '/dexhand/right/set_turbo_mode',
            'both': '/dexhand/set_turbo_mode'
        }

        self.get_services = {
            'left': '/dexhand/left/get_turbo_mode',
            'right': '/dexhand/right/get_turbo_mode',
            'both': '/dexhand/get_turbo_mode'
        }

        # 等待服务启动
        self.wait_for_services()

    def wait_for_services(self):
        """等待所有服务可用"""
        rospy.loginfo("等待服务启动...")

        for name, srv in self.set_services.items():
            try:
                rospy.wait_for_service(srv, timeout=5.0)
                rospy.loginfo(f"✓ 服务 {srv} 已就绪")
            except rospy.ROSException:
                rospy.logerr(f"✗ 服务 {srv} 超时未启动")
                sys.exit(1)

        for name, srv in self.get_services.items():
            try:
                rospy.wait_for_service(srv, timeout=5.0)
                rospy.loginfo(f"✓ 服务 {srv} 已就绪")
            except rospy.ROSException:
                rospy.logerr(f"✗ 服务 {srv} 超时未启动")
                sys.exit(1)

        rospy.loginfo("所有服务已就绪，开始测试...\n")

    def call_set_service(self, hand, enable):
        """调用set_turbo_mode服务"""
        try:
            srv_name = self.set_services[hand]
            srv_proxy = rospy.ServiceProxy(srv_name, SetBool)
            req = SetBoolRequest()
            req.data = enable
            resp = srv_proxy(req)
            return resp.success, resp.message
        except rospy.ServiceException as e:
            return False, f"服务调用失败: {str(e)}"

    def call_get_service(self, hand):
        """调用get_turbo_mode服务"""
        try:
            srv_name = self.get_services[hand]
            srv_proxy = rospy.ServiceProxy(srv_name, Trigger)
            req = TriggerRequest()
            resp = srv_proxy(req)
            return resp.success, resp.message
        except rospy.ServiceException as e:
            return False, f"服务调用失败: {str(e)}"

    def test_hand(self, hand_name, display_name):
        """测试单只手或双手的服务"""
        rospy.loginfo(f"{'='*60}")
        rospy.loginfo(f"开始测试 {display_name} 的turbo模式服务")
        rospy.loginfo(f"{'='*60}")

        # 步骤1: 查询初始状态
        rospy.loginfo(f"\n[步骤1] 查询初始turbo模式状态")
        initial_state, initial_msg = self.call_get_service(hand_name)
        rospy.loginfo(f"查询结果: 状态={initial_state}, 消息={initial_msg}")

        # 步骤2: 开启turbo模式
        rospy.loginfo(f"\n[步骤2] 开启turbo模式")
        set_success, set_msg = self.call_set_service(hand_name, True)
        rospy.loginfo(f"设置结果: 成功={set_success}, 消息={set_msg}")
        # 等待硬件响应
        time.sleep(0.5)

        if not set_success:
            rospy.logwarn(f"{display_name} 开启turbo模式失败，跳过后续测试")
            return False

        # 步骤3: 查询确认开启
        rospy.loginfo(f"\n[步骤3] 查询确认turbo模式已开启")
        get_state, get_msg = self.call_get_service(hand_name)
        rospy.loginfo(f"查询结果: 状态={get_state}, 消息={get_msg}")

        if get_state != True:
            rospy.logerr(f"✗ {display_name} turbo模式开启验证失败，期望=True，实际={get_state}")
            return False
        else:
            rospy.loginfo(f"✓ {display_name} turbo模式开启验证成功")

        # 步骤4: 关闭turbo模式
        rospy.loginfo(f"\n[步骤4] 关闭turbo模式")
        set_success, set_msg = self.call_set_service(hand_name, False)
        rospy.loginfo(f"设置结果: 成功={set_success}, 消息={set_msg}")
        # 等待硬件响应
        time.sleep(0.5)

        if not set_success:
            rospy.logwarn(f"{display_name} 关闭turbo模式失败，跳过后续测试")
            return False

        # 步骤5: 查询确认关闭
        rospy.loginfo(f"\n[步骤5] 查询确认turbo模式已关闭")
        get_state, get_msg = self.call_get_service(hand_name)
        rospy.loginfo(f"查询结果: 状态={get_state}, 消息={get_msg}")

        if get_state != False:
            rospy.logerr(f"✗ {display_name} turbo模式关闭验证失败，期望=False，实际={get_state}")
            return False
        else:
            rospy.loginfo(f"✓ {display_name} turbo模式关闭验证成功")

        rospy.loginfo(f"\n✓ {display_name} 所有测试通过\n")
        return True

    def run_all_tests(self):
        """运行所有测试"""
        test_results = []

        # 测试左手
        test_results.append(self.test_hand('left', '左手'))

        # 测试右手
        test_results.append(self.test_hand('right', '右手'))

        # 测试双手
        test_results.append(self.test_hand('both', '双手'))

        # 汇总结果
        rospy.loginfo(f"{'='*60}")
        rospy.loginfo(f"测试汇总")
        rospy.loginfo(f"{'='*60}")

        passed = sum(test_results)
        total = len(test_results)

        rospy.loginfo(f"总测试数: {total}, 通过: {passed}, 失败: {total - passed}")

        if passed == total:
            rospy.loginfo("\n✓ 所有测试通过!")
            return 0
        else:
            rospy.logerr("\n✗ 部分测试失败!")
            return 1

if __name__ == '__main__':
    try:
        tester = TurboModeServiceTester()
        exit_code = tester.run_all_tests()
        sys.exit(exit_code)
    except rospy.ROSInterruptException:
        rospy.loginfo("测试被中断")
        sys.exit(1)
