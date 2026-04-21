import rclpy
from rclpy.node import Node
import sys

import time
from motor_msgs.msg import RuiermanMotordata

sys.path.append("os.path.dirname(os.path.abspath(__file__))")
from SimpleSDK import WHJ30Tools

# 睿尔曼关节电机地址
Left_joint_address = 0x01
Right_joint_address = 0x02

# 电机控制模式地址
mode_position_address = 0x03

class RuiErmanJointControlNode(Node):

    def __init__(self):
        super().__init__('ruierman_joint_control_node')

        self.whj30 = WHJ30Tools()
        self.whj30.open_canbus()

        # 跳过IAP更新操作，传入关节ID 0x01 -- 左手
        self.whj30.skip_iap_update(Left_joint_address)
        # 跳过IAP更新操作，传入关节ID 0x02 -- 右手
        self.whj30.skip_iap_update(Right_joint_address)

        # 等待3s
        time.sleep(3)

        # 启动节点时即使能左右手电机，并设置工作模式为位置模式
        self.whj30.enable_joint(Left_joint_address)  # 左手电机 ID 可能为 0x01，
        self.whj30.set_joint_operate_mode(Left_joint_address, mode_position_address)  # 设置左手电机工作模式为位置模式

        self.whj30.enable_joint(Right_joint_address)  # 右手电机 ID 可能为 0x02，
        self.whj30.set_joint_operate_mode(Right_joint_address, mode_position_address)  # 设置右手电机工作模式为位置模式

        # self.whj30.clear_joint_error(0x01)
        # self.whj30.clear_joint_error(0x02)

        # 获取关节状态
        print("-- now left  joint status is : ", self.whj30.get_joint_state(Left_joint_address))   # 获取左手关节状态
        print("-- now Right joint status is : ", self.whj30.get_joint_state(Right_joint_address))  # 获取右手关节状态

        # 设置范围
        self.whj30.set_joint_min_position(Left_joint_address, -10)
        self.whj30.set_joint_max_position(Left_joint_address, 50)

        self.whj30.set_joint_min_position(Right_joint_address, -10)
        self.whj30.set_joint_max_position(Right_joint_address, 50)

        # 订阅 /ruiermanJoint/motordata 话题
        self.subscription = self.create_subscription(
            RuiermanMotordata,
            '/ruiermanJoint/motordata',
            self.motor_data_callback,
            10  # QoS profile depth
        )

        self.subscription  # 防止未使用的变量警告

    def motor_data_callback(self, msg):
        # 打印接收到的电机数据
        self.get_logger().info(
            f"Received Motor Data - Left: {msg.l_mpos}, Right: {msg.r_mpos}"
        )

        # 设置左手电机位置
        self.whj30.set_joint_position(Left_joint_address, msg.l_mpos)  # 左手电机 ID 可能为 0x01，

        # 设置右手电机位置
        self.whj30.set_joint_position(Right_joint_address, msg.r_mpos)  # 右手电机 ID 可能为 0x02，

        self.whj30.clear_joint_error(0x01)
        self.whj30.clear_joint_error(0x02)
        
    # def run(self, msg):
    #     self.get_joint_state()

def main(args=None):
    rclpy.init(args=args)
    joint_control_node = RuiErmanJointControlNode()
    rclpy.spin(joint_control_node)
    joint_control_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
