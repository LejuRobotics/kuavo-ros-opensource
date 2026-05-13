#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
VR 动作回放脚本
读取 rosbag 文件，回放 joystick、leju_quest_bones 等话题，并显示躯干信息
"""

import rospy
import rosbag
import sys
import os
import argparse
import glob
import subprocess
import signal
import numpy as np
from collections import defaultdict

# 消息类型
from noitom_hi5_hand_udp_python.msg import PoseInfoList
from kuavo_msgs.msg import JoySticks, headBodyPose, robotWaistControl, sensorsData
from kuavo_msgs.srv import changeArmCtrlMode, changeArmCtrlModeRequest
from geometry_msgs.msg import PoseStamped, Twist
from tf2_msgs.msg import TFMessage
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import JointState


class VRRosbagPlayback:
    """VR Rosbag 回放器"""

    # 需要回放的话题列表
    PLAYBACK_TOPICS = [
        '/quest_joystick_data',           # Joystick 数据
        '/leju_quest_bone_poses',         # VR 骨骼姿态
        '/cmd_torso_pose_vr',             # 躯干姿态命令
        '/cmd_pose',                       # 位姿命令
        '/kuavo_arm_traj',                # 手臂轨迹
        '/robot_waist_motion_data',       # 腰部运动数据
    ]

    # 需要显示信息的话题列表
    DISPLAY_TOPICS = [
        '/kuavo_head_body_orientation_data',  # 躯干方向信息
        '/robot_waist_motion_data',           # 腰部运动数据
        '/quest3_debug/chest_axis',           # 胸部轴信息
        '/quest3_debug/shoulder_angle',       # 肩部角度
    ]

    # 骨骼名称列表
    BONE_NAMES = [
        "LeftArmUpper", "LeftArmLower", "RightArmUpper", "RightArmLower",
        "LeftHandPalm", "RightHandPalm", "LeftHandThumbMetacarpal",
        "LeftHandThumbProximal", "LeftHandThumbDistal", "LeftHandThumbTip",
        "LeftHandIndexTip", "LeftHandMiddleTip", "LeftHandRingTip",
        "LeftHandLittleTip", "RightHandThumbMetacarpal", "RightHandThumbProximal",
        "RightHandThumbDistal", "RightHandThumbTip", "RightHandIndexTip",
        "RightHandMiddleTip", "RightHandRingTip", "RightHandLittleTip",
        "Root", "Chest", "Neck", "Head"
    ]

    def __init__(self):
        rospy.init_node('vr_rosbag_playback', anonymous=True)

        # 发布器
        self.publishers = {}
        self._init_publishers()

        # 统计信息
        self.msg_counts = defaultdict(int)
        self.last_torso_info = None
        self.last_bone_poses = None

        # 回放参数
        self.rate = rospy.Rate(100)  # 100Hz
        self.playback_speed = 1.0
        self.arm_ctrl_mode = 2
        self.skip_arm_mode = False
        self.use_ik = False
        self.ik_process = None

        # 关节状态监控
        self.latest_sensors_data = None
        self.sensors_data_sub = rospy.Subscriber(
            '/sensors_data_raw', sensorsData, self._sensors_data_callback, queue_size=1)

    def _sensors_data_callback(self, msg):
        """传感器数据回调"""
        self.latest_sensors_data = msg

    def _init_publishers(self):
        """初始化发布器"""
        self.publishers['/quest_joystick_data'] = rospy.Publisher(
            '/quest_joystick_data', JoySticks, queue_size=10)
        self.publishers['/leju_quest_bone_poses'] = rospy.Publisher(
            '/leju_quest_bone_poses', PoseInfoList, queue_size=10)
        self.publishers['/cmd_torso_pose_vr'] = rospy.Publisher(
            '/cmd_torso_pose_vr', PoseStamped, queue_size=10)
        self.publishers['/cmd_pose'] = rospy.Publisher(
            '/cmd_pose', Twist, queue_size=10)
        self.publishers['/kuavo_arm_traj'] = rospy.Publisher(
            '/kuavo_arm_traj', JointState, queue_size=10)
        self.publishers['/robot_waist_motion_data'] = rospy.Publisher(
            '/robot_waist_motion_data', robotWaistControl, queue_size=10)

    def change_arm_ctrl_mode(self, control_mode=2):
        """切换手臂控制模式

        按照原始测试脚本的流程：先设置为1，再设置为目标模式
        """
        service_name = '/arm_traj_change_mode'
        rospy.loginfo(f"等待服务 {service_name}...")

        try:
            rospy.wait_for_service(service_name, timeout=5.0)
            change_mode = rospy.ServiceProxy(service_name, changeArmCtrlMode)

            # 先设置为模式 1
            req = changeArmCtrlModeRequest()
            req.control_mode = 1
            rospy.loginfo("先切换手臂控制模式为: 1")
            resp = change_mode(req)
            if resp.result:
                rospy.loginfo(f"模式切换成功: mode={resp.mode}")
            else:
                rospy.logwarn(f"模式切换失败: {resp.message}")

            rospy.sleep(1.0)

            # 再设置为目标模式
            req.control_mode = control_mode
            rospy.loginfo(f"切换手臂控制模式为: {control_mode}")
            resp = change_mode(req)

            if resp.result:
                rospy.loginfo(f"手臂控制模式切换成功: mode={resp.mode}, {resp.message}")
                return True
            else:
                rospy.logwarn(f"手臂控制模式切换失败: {resp.message}")
                return False

        except rospy.ServiceException as e:
            rospy.logerr(f"服务调用失败: {e}")
            return False
        except rospy.ROSException as e:
            rospy.logerr(f"等待服务超时: {e}")
            return False

    def start_ik_node(self):
        """启动 motion_capture_ik 节点"""
        if not self.use_ik:
            return True

        rospy.loginfo("启动 motion_capture_ik 节点...")
        try:
            # 使用 roslaunch 启动 IK 节点
            cmd = [
                "roslaunch", "motion_capture_ik", "ik_ros_uni.launch",
                "version:=4", "ctrl_arm_idx:=2", "ik_type_idx:=0",
                "control_torso:=0"
            ]
            self.ik_process = subprocess.Popen(
                cmd,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                preexec_fn=os.setsid
            )
            rospy.sleep(3.0)  # 等待节点启动
            rospy.loginfo("motion_capture_ik 节点已启动")
            return True
        except Exception as e:
            rospy.logerr(f"启动 IK 节点失败: {e}")
            return False

    def stop_ik_node(self):
        """停止 motion_capture_ik 节点"""
        if self.ik_process is not None:
            rospy.loginfo("停止 motion_capture_ik 节点...")
            try:
                os.killpg(os.getpgid(self.ik_process.pid), signal.SIGTERM)
                self.ik_process.wait(timeout=5)
            except Exception as e:
                rospy.logwarn(f"停止 IK 节点时出错: {e}")
            self.ik_process = None

    def list_bag_files(self, bag_dir):
        """列出目录下的所有 bag 文件"""
        if os.path.isfile(bag_dir) and bag_dir.endswith('.bag'):
            return [bag_dir]

        bag_files = glob.glob(os.path.join(bag_dir, '*.bag'))
        bag_files.sort()
        return bag_files

    def get_bag_info(self, bag_path):
        """获取 bag 文件信息"""
        try:
            bag = rosbag.Bag(bag_path)
            info = bag.get_type_and_topic_info()

            topics_info = {}
            for topic, topic_info in info.topics.items():
                topics_info[topic] = {
                    'msg_type': topic_info.msg_type,
                    'msg_count': topic_info.message_count,
                    'frequency': topic_info.frequency if topic_info.frequency else 'N/A'
                }

            duration = bag.get_end_time() - bag.get_start_time()
            bag.close()

            return {
                'duration': duration,
                'topics': topics_info
            }
        except Exception as e:
            rospy.logerr(f"获取 bag 信息失败: {e}")
            return None

    def print_bag_summary(self, bag_path):
        """打印 bag 文件摘要"""
        info = self.get_bag_info(bag_path)
        if not info:
            return

        print("\n" + "="*60)
        print(f"Bag 文件: {os.path.basename(bag_path)}")
        print(f"时长: {info['duration']:.2f} 秒")
        print("-"*60)
        print("VR 相关话题:")

        vr_topics = self.PLAYBACK_TOPICS + self.DISPLAY_TOPICS
        for topic in vr_topics:
            if topic in info['topics']:
                t_info = info['topics'][topic]
                print(f"  {topic}:")
                print(f"    类型: {t_info['msg_type']}")
                print(f"    消息数: {t_info['msg_count']}")
        print("="*60 + "\n")

    def display_torso_info(self, msg, topic):
        """显示躯干信息"""
        if topic == '/kuavo_head_body_orientation_data':
            # headBodyPose 消息
            if hasattr(msg, 'body_euler'):
                print(f"\r[躯干] Roll: {msg.body_euler.x:7.2f}° "
                      f"Pitch: {msg.body_euler.y:7.2f}° "
                      f"Yaw: {msg.body_euler.z:7.2f}°", end='')
            self.last_torso_info = msg

        elif topic == '/robot_waist_motion_data':
            # robotWaistControl 消息
            if hasattr(msg, 'waist_pitch') and hasattr(msg, 'waist_yaw'):
                print(f"  [腰部] Pitch: {msg.waist_pitch:7.2f}° "
                      f"Yaw: {msg.waist_yaw:7.2f}°", end='')

        elif topic == '/quest3_debug/chest_axis':
            # Float32MultiArray 消息
            if hasattr(msg, 'data') and len(msg.data) >= 3:
                print(f"\n[胸部轴] X: {msg.data[0]:7.3f} "
                      f"Y: {msg.data[1]:7.3f} "
                      f"Z: {msg.data[2]:7.3f}", end='')

        elif topic == '/quest3_debug/shoulder_angle':
            # Float32MultiArray 消息
            if hasattr(msg, 'data') and len(msg.data) >= 2:
                print(f"  [肩部角度] L: {msg.data[0]:7.2f}° "
                      f"R: {msg.data[1]:7.2f}°", end='')

    def display_bone_info(self, msg):
        """显示骨骼信息"""
        if not hasattr(msg, 'pose_info_list') or len(msg.pose_info_list) == 0:
            return

        # 找到 Chest (躯干) 骨骼
        chest_idx = self.BONE_NAMES.index("Chest") if "Chest" in self.BONE_NAMES else -1
        root_idx = self.BONE_NAMES.index("Root") if "Root" in self.BONE_NAMES else -1

        info_str = ""
        for pose_info in msg.pose_info_list:
            if pose_info.bone_id == chest_idx:
                pos = pose_info.position
                rot = pose_info.rotation
                info_str += f"\n[Chest] pos:({pos.x:.3f},{pos.y:.3f},{pos.z:.3f}) "
                info_str += f"rot:({rot.x:.3f},{rot.y:.3f},{rot.z:.3f},{rot.w:.3f})"
            elif pose_info.bone_id == root_idx:
                pos = pose_info.position
                info_str += f"  [Root] pos:({pos.x:.3f},{pos.y:.3f},{pos.z:.3f})"

        if info_str:
            print(info_str, end='')

        self.last_bone_poses = msg

    def display_joystick_info(self, msg):
        """显示 joystick 信息"""
        if hasattr(msg, 'axes') and hasattr(msg, 'buttons'):
            axes_str = ','.join([f"{a:.2f}" for a in msg.axes[:4]]) if len(msg.axes) >= 4 else str(msg.axes)
            print(f"\n[Joystick] axes: [{axes_str}]", end='')

    def display_arm_traj_info(self, msg):
        """显示手臂轨迹信息"""
        if hasattr(msg, 'position') and len(msg.position) > 0:
            # 显示前几个关节位置
            pos_str = ','.join([f"{p:.2f}" for p in msg.position[:6]])
            print(f"\r[ArmTraj] pos: [{pos_str}...]", end='')

    def monitor_joint_angles(self):
        """持续监控并打印当前关节角度，频率 1Hz

        关节索引对照 (sensors_data_raw.joint_data.joint_q):
        - 左腿: 0-6  (leg_l1 ~ leg_l7)
        - 右腿: 7-11 (leg_r2 ~ leg_r6)
        - 左臂: 12-18 (zarm_l1 ~ zarm_l7)
        - 右臂: 19-25 (zarm_r1 ~ zarm_r7)
        - 头部: 26-27 (zhead_1, zhead_2)
        """
        print("\n" + "="*70)
        print("回放完成，开始监控关节角度 (按 Ctrl+C 停止)")
        print("="*70)

        rate = rospy.Rate(1)  # 1Hz
        while not rospy.is_shutdown():
            if self.latest_sensors_data is not None:
                msg = self.latest_sensors_data
                if hasattr(msg, 'joint_data') and hasattr(msg.joint_data, 'joint_q'):
                    positions = list(msg.joint_data.joint_q)
                    num_joints = len(positions)

                    # 打印时间戳
                    print(f"\n[{rospy.Time.now().to_sec():.2f}] 当前关节角度 ({num_joints} 个关节):")
                    print("-" * 70)

                    # 左腿 (关节 0-6)
                    if num_joints >= 7:
                        left_leg = positions[0:7]
                        left_leg_str = ', '.join([f"{p:7.2f}" for p in left_leg])
                        print(f"  左腿 [0-6]:   [{left_leg_str}]")

                    # 右腿 (关节 7-11)
                    if num_joints >= 12:
                        right_leg = positions[7:12]
                        right_leg_str = ', '.join([f"{p:7.2f}" for p in right_leg])
                        print(f"  右腿 [7-11]:  [{right_leg_str}]")

                    # 左臂 (关节 12-18)
                    if num_joints >= 19:
                        left_arm = positions[12:19]
                        left_arm_str = ', '.join([f"{p:7.2f}" for p in left_arm])
                        print(f"  左臂 [12-18]: [{left_arm_str}]")

                    # 右臂 (关节 19-25)
                    if num_joints >= 26:
                        right_arm = positions[19:26]
                        right_arm_str = ', '.join([f"{p:7.2f}" for p in right_arm])
                        print(f"  右臂 [19-25]: [{right_arm_str}]")

                    # 头部 (关节 26-27)
                    if num_joints >= 28:
                        head = positions[26:28]
                        head_str = ', '.join([f"{p:7.2f}" for p in head])
                        print(f"  头部 [26-27]: [{head_str}]")
            else:
                print(f"\n[{rospy.Time.now().to_sec():.2f}] 等待 /sensors_data_raw 数据...")

            rate.sleep()

    def playback_bag(self, bag_path, display_info=True, loop=False):
        """回放单个 bag 文件"""
        if not os.path.exists(bag_path):
            rospy.logerr(f"Bag 文件不存在: {bag_path}")
            return False

        self.print_bag_summary(bag_path)

        # 启动 IK 节点（如果需要）
        self.start_ik_node()

        # 切换手臂控制模式
        if not self.skip_arm_mode:
            if not self.change_arm_ctrl_mode(control_mode=self.arm_ctrl_mode):
                rospy.logwarn("手臂控制模式切换失败，继续回放...")

        try:
            bag = rosbag.Bag(bag_path)
            start_time = bag.get_start_time()
            end_time = bag.get_end_time()
            duration = end_time - start_time

            # 获取 bag 中实际存在的话题
            bag_info = bag.get_type_and_topic_info()
            available_topics = set(bag_info.topics.keys())

            # 过滤出 bag 中存在的话题
            all_topics = self.PLAYBACK_TOPICS + self.DISPLAY_TOPICS
            topics_to_read = [t for t in all_topics if t in available_topics]

            # 记录哪些话题将被播放，哪些不存在
            playback_topics_available = [t for t in self.PLAYBACK_TOPICS if t in available_topics]
            playback_topics_missing = [t for t in self.PLAYBACK_TOPICS if t not in available_topics]

            rospy.loginfo(f"开始回放: {bag_path}")
            rospy.loginfo(f"回放时长: {duration:.2f} 秒")
            if playback_topics_available:
                rospy.loginfo(f"将回放的话题: {playback_topics_available}")
            if playback_topics_missing:
                rospy.logwarn(f"Bag 中不存在的话题 (跳过): {playback_topics_missing}")
            rospy.loginfo("按 Ctrl+C 停止回放")
            print("\n" + "-"*60)

            while not rospy.is_shutdown():
                playback_start = rospy.Time.now()
                msg_start_time = None

                for topic, msg, t in bag.read_messages(topics=topics_to_read):
                    if rospy.is_shutdown():
                        break

                    # 计算时间差并等待
                    if msg_start_time is None:
                        msg_start_time = t.to_sec()

                    elapsed_bag_time = (t.to_sec() - msg_start_time) / self.playback_speed
                    elapsed_real_time = (rospy.Time.now() - playback_start).to_sec()

                    sleep_time = elapsed_bag_time - elapsed_real_time
                    if sleep_time > 0:
                        rospy.sleep(sleep_time)

                    # 发布消息
                    if topic in self.publishers:
                        # 更新时间戳
                        if hasattr(msg, 'header'):
                            msg.header.stamp = rospy.Time.now()
                        self.publishers[topic].publish(msg)
                        self.msg_counts[topic] += 1

                    # 显示信息
                    if display_info:
                        if topic == '/leju_quest_bone_poses':
                            self.display_bone_info(msg)
                        elif topic == '/quest_joystick_data':
                            self.display_joystick_info(msg)
                        elif topic == '/kuavo_arm_traj':
                            self.display_arm_traj_info(msg)
                        elif topic in self.DISPLAY_TOPICS:
                            self.display_torso_info(msg, topic)

                if not loop:
                    break

                rospy.loginfo("\n循环回放，重新开始...")

            bag.close()

            # 打印统计信息
            print("\n\n" + "="*60)
            print("回放统计:")
            for topic, count in self.msg_counts.items():
                print(f"  {topic}: {count} 条消息")
            print("="*60)

            return True

        except Exception as e:
            rospy.logerr(f"回放失败: {e}")
            import traceback
            traceback.print_exc()
            return False
        finally:
            # 停止 IK 节点
            self.stop_ik_node()

    def playback_directory(self, bag_dir, display_info=True, loop=False):
        """回放目录下的所有 bag 文件"""
        bag_files = self.list_bag_files(bag_dir)

        if not bag_files:
            rospy.logerr(f"目录下没有找到 bag 文件: {bag_dir}")
            return False

        print(f"\n找到 {len(bag_files)} 个 bag 文件:")
        for i, f in enumerate(bag_files):
            print(f"  {i+1}. {os.path.basename(f)}")
        print()

        for bag_file in bag_files:
            if rospy.is_shutdown():
                break

            success = self.playback_bag(bag_file, display_info, loop=False)
            if not success:
                rospy.logwarn(f"跳过文件: {bag_file}")

            # 文件之间稍作停顿
            rospy.sleep(1.0)

        return True


def main():
    parser = argparse.ArgumentParser(description='VR 动作回放脚本')
    parser.add_argument('bag_path', type=str,
                        help='Rosbag 文件或目录路径')
    parser.add_argument('--speed', type=float, default=1.0,
                        help='回放速度倍率 (默认: 1.0)')
    parser.add_argument('--loop', action='store_true',
                        help='循环回放')
    parser.add_argument('--no-display', action='store_true',
                        help='不显示详细信息')
    parser.add_argument('--info-only', action='store_true',
                        help='只显示 bag 文件信息，不回放')
    parser.add_argument('--arm-mode', type=int, default=2,
                        help='手臂控制模式 (默认: 2)')
    parser.add_argument('--skip-arm-mode', action='store_true',
                        help='跳过手臂模式切换')
    parser.add_argument('--use-ik', action='store_true',
                        help='启动 motion_capture_ik 节点处理骨骼数据')

    # 解析参数（排除 ROS 参数）
    args, _ = parser.parse_known_args()

    player = VRRosbagPlayback()
    player.playback_speed = args.speed
    player.arm_ctrl_mode = args.arm_mode
    player.skip_arm_mode = args.skip_arm_mode
    player.use_ik = args.use_ik

    bag_path = os.path.expanduser(args.bag_path)

    if args.info_only:
        # 只显示信息
        if os.path.isdir(bag_path):
            bag_files = player.list_bag_files(bag_path)
            for f in bag_files:
                player.print_bag_summary(f)
        else:
            player.print_bag_summary(bag_path)
        return

    # 回放
    display_info = not args.no_display

    if os.path.isdir(bag_path):
        player.playback_directory(bag_path, display_info, args.loop)
    else:
        player.playback_bag(bag_path, display_info, args.loop)

    # 回放完成后持续监控关节角度
    player.monitor_joint_angles()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        print("\n\n用户中断回放")
