#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import sys
import time
import json
import queue
import rospy
import rospkg
import threading

from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from kuavo_msgs.msg import sensorsData, lejuClawState

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../')))
import protos.robot_state_pb2 as robot_state_pb2

from udp_server import UDPServer


def get_package_path(package_name):
    """获取 ROS 包的路径"""
    try:
        rospack = rospkg.RosPack()
        package_path = rospack.get_path(package_name)
        return package_path
    except rospkg.ResourceNotFound:
        return None


def get_end_effector_type():
    """
    获取末端执行器类型
    优先从 ROS 参数服务器获取，如果不存在则从 kuavo.json 配置文件读取
    """
    if rospy.has_param("/end_effector_type"):
        ee_type = rospy.get_param("/end_effector_type")
        print(f"\033[92mrobot_state_server: end_effector_type from rosparam: {ee_type}\033[0m")
        return ee_type
    
    # 从 kuavo.json 配置文件读取
    kuavo_assets_path = get_package_path("kuavo_assets")
    if kuavo_assets_path is None:
        print("\033[91mrobot_state_server: kuavo_assets package not found, using default ee_type: none\033[0m")
        return "none"
    
    robot_version = os.environ.get('ROBOT_VERSION', '40')
    config_file = os.path.join(kuavo_assets_path, f"config/kuavo_v{robot_version}/kuavo.json")
    
    try:
        with open(config_file, 'r') as f:
            config = json.load(f)
            ee_type = config.get("EndEffectorType", ["qiangnao", "qiangnao"])[0]
            print(f"\033[93mrobot_state_server: end_effector_type from kuavo.json: {ee_type}\033[0m")
            return ee_type
    except FileNotFoundError:
        print(f"\033[91mrobot_state_server: Config file not found: {config_file}, using default ee_type: none\033[0m")
        return "none"
    except (json.JSONDecodeError, KeyError, IndexError) as e:
        print(f"\033[91mrobot_state_server: Error reading config file: {e}, using default ee_type: none\033[0m")
        return "none"

class RobotSubscriber:
    """机器人数据订阅器类"""

    # 力反馈归一化常量
    MAX_FORCE_NEWTONS = 19.6  # 最大力值（牛顿），对应2kg重量
    VR_VIBRATION_MAX = 255    # VR振动强度最大值

    def __init__(self, ee_type="none", subscribe_sensor_data=False):
        self.ee_type = ee_type.lower()
        self.subscribe_sensor_data = subscribe_sensor_data

        # 使用队列存储数据
        queue_size = 100
        self.sensor_queue = queue.Queue(maxsize=queue_size)
        self.ee_queue = queue.Queue(maxsize=queue_size)
        self.arm_force_queue = queue.Queue(maxsize=queue_size)

        # 设置订阅
        self._setup_subscription()

    def _setup_subscription(self):
        if self.subscribe_sensor_data:
            self.sensors_sub = rospy.Subscriber("/sensors_data_raw", sensorsData, self._sensors_callback)
            print(f"robot_state_server: Subscribed to /sensors_data_raw (sensor data)")

        # 根据末端执行器类型订阅对应话题
        if self.ee_type == "lejuclaw":
            self.ee_sub = rospy.Subscriber("/leju_claw_state", lejuClawState, self._lejuclaw_callback)
            print(f"robot_state_server: Subscribed to /leju_claw_state (leju claw) for ee_type: {self.ee_type}")
        elif self.ee_type in ["qiangnao", "qiangnao_touch", "revo2"]:
            self.ee_sub = rospy.Subscriber("/dexhand/state", JointState, self._dexhand_callback)
            print(f"robot_state_server: Subscribed to /dexhand/state (dexhand) for ee_type: {self.ee_type}")
        else:
            print(f"robot_state_server: No end effector subscription for ee_type: {self.ee_type}")

        # 订阅手臂接触力反馈数据
        self.arm_force_sub = rospy.Subscriber("/state_estimate/arm_contact_force", Float64MultiArray, self._arm_force_callback)
        print(f"robot_state_server: Subscribed to /state_estimate/arm_contact_force (arm force feedback)")

    def _sensors_callback(self, msg):
        """传感器数据回调函数"""
        try:
            # 创建关节状态
            joint_state = robot_state_pb2.JointState()
            joint_state.position[:] = list(msg.joint_data.joint_q)
            joint_state.velocity[:] = list(msg.joint_data.joint_v)
            joint_state.torque[:] = list(msg.joint_data.joint_torque)

            # 放入队列，如果队列满了则丢弃最旧的数据
            if self.sensor_queue.full():
                # print(f"robot_state_server: Sensor data queue is full, dropping oldest data")
                try:
                    self.sensor_queue.get_nowait()
                except queue.Empty:
                    pass

            self.sensor_queue.put_nowait(joint_state)

        except Exception as e:
            print(f"Error processing sensors data: {e}")

    def _lejuclaw_callback(self, msg):
        """二指夹爪状态回调函数"""
        try:
            # 创建末端执行器状态
            ee_state = robot_state_pb2.EndEffectorState()
            ee_state.position[:] = list(msg.data.position) if msg.data.position else [0.0, 0.0]
            ee_state.velocity[:] = list(msg.data.velocity) if msg.data.velocity else [0.0, 0.0]
            ee_state.effort[:] = list(msg.data.effort) if msg.data.effort else [0.0, 0.0]

            # 放入队列，如果队列满了则丢弃最旧的数据
            if self.ee_queue.full():
                # print(f"robot_state_server: Lejuclaw data queue is full, dropping oldest data")
                try:
                    self.ee_queue.get_nowait()
                except queue.Empty:
                    pass

            self.ee_queue.put_nowait(ee_state)

        except Exception as e:
            print(f"Error processing lejuclaw data: {e}")

    def _dexhand_callback(self, msg):
        """灵巧手状态回调函数"""
        try:
            # 创建末端执行器状态
            ee_state = robot_state_pb2.EndEffectorState()
            ee_state.position[:] = list(msg.position) if msg.position else [0.0] * 12
            ee_state.velocity[:] = list(msg.velocity) if msg.velocity else [0.0] * 12
            ee_state.effort[:] = list(msg.effort) if msg.effort else [0.0] * 12

            # 放入队列，如果队列满了则丢弃最旧的数据
            if self.ee_queue.full():
                # print(f"robot_state_server: Dexhand data queue is full, dropping oldest data")
                try:
                    self.ee_queue.get_nowait()
                except queue.Empty:
                    pass

            self.ee_queue.put_nowait(ee_state)

        except Exception as e:
            print(f"Error processing dexhand data: {e}")

    def _normalize_force(self, value):
        """将力值归一化到 VR 振动强度范围

        Args:
            value: 力值（牛顿）

        Returns:
            float: 归一化后的振动强度 (0-255)
        """
        normalized = (value / self.MAX_FORCE_NEWTONS) * self.VR_VIBRATION_MAX
        return min(self.VR_VIBRATION_MAX, max(0.0, normalized))

    def _arm_force_callback(self, msg):
        """手臂接触力反馈回调函数"""
        try:
            # 数据格式: [左臂Fx, Fy, Fz, 右臂Fx, Fy, Fz]
            data = list(msg.data)

            if len(data) < 6:
                print(f"robot_state_server: Invalid arm force data length: {len(data)}, expected 6")
                return

            # 创建 ArmForceFeedback 消息
            arm_force = robot_state_pb2.ArmForceFeedback()

            # 计算左臂 xyz 中最大的力（取绝对值）并归一化
            left_max_force = max(abs(data[0]), abs(data[1]), abs(data[2]))
            arm_force.max_force.append(self._normalize_force(left_max_force))

            # 计算右臂 xyz 中最大的力（取绝对值）并归一化
            right_max_force = max(abs(data[6]), abs(data[7]), abs(data[8]))
            arm_force.max_force.append(self._normalize_force(right_max_force))

            # 放入队列，如果队列满了则丢弃最旧的数据
            if self.arm_force_queue.full():
                try:
                    self.arm_force_queue.get_nowait()
                except queue.Empty:
                    pass

            self.arm_force_queue.put_nowait(arm_force)

        except Exception as e:
            print(f"Error processing arm force feedback data: {e}")

    def get_joint_state(self):
        """从队列获取最新的关节状态"""
        try:
            # 获取队列中最新的数据
            latest_data = None
            while not self.sensor_queue.empty():
                latest_data = self.sensor_queue.get_nowait()

            if latest_data and len(latest_data.position) > 0:
                return (True, latest_data)
        except queue.Empty:
            pass
        return (False, robot_state_pb2.JointState())

    def get_arm_force_feedback(self):
        """从队列获取最新的手臂力反馈数据"""
        try:
            # 获取队列中最新的数据
            latest_data = None
            while not self.arm_force_queue.empty():
                latest_data = self.arm_force_queue.get_nowait()

            if latest_data:
                return (True, latest_data)
        except queue.Empty:
            pass
        return (False, robot_state_pb2.ArmForceFeedback())

    def get_ee_state(self):
        try:
            # 获取队列中最新的数据
            latest_data = None
            while not self.ee_queue.empty():
                latest_data = self.ee_queue.get_nowait()

            if latest_data and len(latest_data.position) > 0:
                return (True, latest_data)
        except queue.Empty:
            pass
        return (False, robot_state_pb2.EndEffectorState())

class RobotStateServer:
    """机器人状态服务器主类"""

    def __init__(self, ee_type="none", host='0.0.0.0', udp_port=15170, publish_rate=20, subscribe_sensor_data=False, client_timeout=60):
        """初始化机器人状态服务器"""
        self.ee_type = ee_type.lower()
        self.host = host
        self.udp_port = udp_port
        self.publish_rate = publish_rate
        self.client_timeout = client_timeout
        self.running = False
        self.message_id = 0

        # 创建机器人订阅器
        self.robot_subscriber = RobotSubscriber(ee_type=self.ee_type, subscribe_sensor_data=subscribe_sensor_data)
        
        # 物品质量与力响应队列
        self.item_mass_force_response_queue = queue.Queue(maxsize=100)

    def start(self):
        """启动服务器

        Returns:
            bool: 启动是否成功
        """
        try:
            print(f"\033[92mRobot State Server: {self.host}:{self.udp_port}, Rate {self.publish_rate}Hz, ee_type: {self.ee_type}\033[0m")

            # 创建UDP服务器
            self.udp_server = UDPServer(host=self.host, port=self.udp_port, client_timeout=self.client_timeout)

            # 启动UDP服务器
            if not self.udp_server.start():
                print(f"\033[91mFailed to start UDP server on port {self.udp_port}\033[0m")
                return False

            # 启动发布线程
            self.running = True
            self.publish_thread = threading.Thread(target=self._publish_loop, daemon=True)
            self.publish_thread.start()

            print("\033[92mRobot State Server started successfully!\033[0m")
            return True

        except Exception as e:
            print(f"Failed to start server: {e}")
            self.stop()
            return False

    def stop(self):
        """停止服务器"""
        if self.running:
            self.running = False
            print("\033[91mStopping Robot State Server...\033[0m")
            if hasattr(self, 'udp_server'):
                self.udp_server.stop()
                print("\033[92mRobot State Server stopped!\033[0m")
        print("\033[91mRobot State Server stopped!\033[0m")
    
    def add_item_mass_force_response(self, response):
        """添加物品质量与力响应到队列
        
        Args:
            response: hand_wrench_srv_pb2.ItemMassForceResponse 消息
        """
        try:
            if self.item_mass_force_response_queue.full():
                # 如果队列满了，移除最旧的响应
                try:
                    self.item_mass_force_response_queue.get_nowait()
                except queue.Empty:
                    pass
            self.item_mass_force_response_queue.put_nowait(response)
        except Exception as e:
            print(f"Error adding item mass force response: {e}")
    
    def _process_item_mass_force_response(self):
        """从队列中获取物品质量与力响应
        
        Returns:
            hand_wrench_srv_pb2.ItemMassForceResponse 或 None
        """
        try:
            return self.item_mass_force_response_queue.get_nowait()
        except queue.Empty:
            return None
    def _publish_loop(self):
        """发布循环"""
        rate = rospy.Rate(self.publish_rate)

        while self.running and not rospy.is_shutdown():
            try:
                # 获取当前状态
                robot_state = self._get_robot_state()

                if robot_state:
                    # 通过UDP发送状态
                    self.udp_server.broadcast(robot_state)

                rate.sleep()

            except Exception as e:
                print(f"Error in publish loop: {e}")

    def _get_robot_state(self):
        """
        获取当前机器人状态并转换为protobuf格式

        Returns:
            robot_state_pb2.RobotState: 机器人状态protobuf消息
        """
        try:
            # 创建机器人状态消息
            robot_state = robot_state_pb2.RobotState()

            # 设置消息头
            self.message_id += 1
            robot_state.header.id = self.message_id
            robot_state.header.timestamp = int(time.time() * 1e9)  # 纳秒时间戳

            # 获取关节状态
            joint_valid, joint_state = self.robot_subscriber.get_joint_state()
            if joint_valid:
                robot_state.joint_state.CopyFrom(joint_state)
            
            ee_valid, ee_state = self.robot_subscriber.get_ee_state()
            if ee_valid and ee_state:
                robot_state.ee_state.CopyFrom(ee_state)
            
            # 获取物品质量与力响应
            item_mass_force_response = self._process_item_mass_force_response()
            if item_mass_force_response:
                robot_state.item_mass_force_response.CopyFrom(item_mass_force_response)

            # 获取手臂力反馈数据
            arm_force_valid, arm_force = self.robot_subscriber.get_arm_force_feedback()
            if arm_force_valid and arm_force:
                robot_state.arm_force_feedback.CopyFrom(arm_force)

            return robot_state

        except Exception as e:
            print(f"Error creating robot state: {e}")
            return None


def main():
    """主函数"""
    # 初始化ROS节点
    rospy.init_node('robot_state_server', anonymous=False)

    # 获取末端执行器类型（优先从 ROS 参数，否则从 kuavo.json 读取）
    ee_type = get_end_effector_type()

    # 创建并启动服务器
    subscribe_sensor_data = True  # 订阅 sensors_data_raw
    server = RobotStateServer(ee_type=ee_type, udp_port=15170, publish_rate=20, subscribe_sensor_data=subscribe_sensor_data)

    try:
        if server.start():
            rospy.spin()
        else:
            print(f"\033[91mFailed to start Robot State Server\033[0m")
            sys.exit(1)
    except KeyboardInterrupt:
        server.stop()
    except Exception as e:
        print(f"Server error: {e}")
        server.stop()
        sys.exit(1)


if __name__ == "__main__":
    main()