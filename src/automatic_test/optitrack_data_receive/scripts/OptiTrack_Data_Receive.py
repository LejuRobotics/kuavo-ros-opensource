#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
from os.path import abspath, join, dirname
sys.path.insert(0, join(abspath(dirname(__file__))))

import sys
import socket
import time
import signal
import threading
import rospy
from geometry_msgs.msg import PoseStamped
from NatNetClient import NatNetClient
import DataDescriptions
import MoCapData

import sys
import os

# 刚体 ID -> 名称映射，由 Motive 下发的 Data Descriptions 自动填充
rigid_body_names = {}
# 刚体 ID -> 话题名（与 Nokov motioncapture 一致，便于同一套标定流程切换 SDK）
rigid_body_id_to_topic = {}

# 与 Nokov SampleClient 一致的话题名：Motive 刚体名称(小写) -> 话题名
# joint_1..joint_7 在 Nokov 中无 _pose 后缀，其余为 xxx_pose
NOKOV_TOPIC_MAP = {
    "car": "car_pose",
    "lefthand": "lefthand_pose",
    "righthand": "righthand_pose",
    "robot": "robot_pose",
    "base": "base_pose",
    "base_pose": "base_pose",
    "tag": "tag_pose",
    "shoulder": "shoulder_pose",
    "effector": "effector_pose",
    "elbow": "elbow_pose",
    "belly": "belly_pose",
    "calimark": "calimark_pose",
    "joint_1": "joint_1",
    "joint_2": "joint_2",
    "joint_3": "joint_3",
    "joint_4": "joint_4",
    "joint_5": "joint_5",
    "joint_6": "joint_6",
    "joint_7": "joint_7",
    "joint_end": "joint_end",
}

# 话题名 -> Publisher（与 Nokov 一致的话题名和类型 geometry_msgs/PoseStamped）
pose_publishers = {}

# 全局变量用于控制程序退出
shutdown_flag = False
streaming_client = None

# 回调函数，连接到NatNet客户端，每帧动作捕捉数据调用一次
def receive_new_frame(data_dict):
    order_list = ["frameNumber", "markerSetCount", "unlabeledMarkersCount",
                  "rigidBodyCount", "skeletonCount", "labeledMarkerCount",
                  "timecode", "timecodeSub", "timestamp", "isRecording",
                  "trackedModelsChanged"]
    dump_args = False  # 设置为True以保留原始帧数据打印
    if dump_args is True:
        print("MoCap Frame Begin")
        print("-----------------")
        
        out_string = f"Frame #:{data_dict.get('frameNumber', 'N/A')}\n"
        out_string += f"  Markerset Count:  {data_dict.get('markerSetCount', 'N/A')}\n"
        
        print(out_string)


def on_data_descriptions(data_descs):
    """收到 Motive 下发的 Data Descriptions 后，按刚体名称建立 ID 映射并创建 ROS 发布者（话题名与 Nokov 一致）"""
    global rigid_body_names, rigid_body_id_to_topic, pose_publishers
    for rb in data_descs.rigid_body_list:
        name = rb.sz_name
        if isinstance(name, bytes):
            name = name.decode("utf-8", errors="replace")
        name = name.strip("\x00").strip()
        if not name:
            continue
        rigid_body_names[rb.id_num] = name
        name_lower = name.lower()
        topic_name = NOKOV_TOPIC_MAP.get(name_lower)
        if topic_name is None:
            topic_name = name + "_pose"
        rigid_body_id_to_topic[rb.id_num] = topic_name
        if topic_name not in pose_publishers:
            pose_publishers[topic_name] = rospy.Publisher(topic_name, PoseStamped, queue_size=10)
        print(f"  刚体: ID={rb.id_num} -> 名称='{name}' 话题 {topic_name}")


# 每帧每个刚体调用一次；话题名、类型、frame_id 与 Nokov 一致
def receive_rigid_body_frame(new_id, position, rotation):
    if new_id not in rigid_body_id_to_topic:
        return
    topic_name = rigid_body_id_to_topic[new_id]
    if topic_name not in pose_publishers:
        return
    pose_msg = PoseStamped()
    pose_msg.header.stamp = rospy.Time.now()
    pose_msg.header.frame_id = "mocap_frame"
    # 与 Nokov 一致的话题/类型/frame_id；位置为毫米（Motive 为米，×1000）
    pose_msg.pose.position.x = position[0] * 1000.0
    pose_msg.pose.position.y = position[1] * 1000.0
    pose_msg.pose.position.z = position[2] * 1000.0
    pose_msg.pose.orientation.x = rotation[0]
    pose_msg.pose.orientation.y = rotation[1]
    pose_msg.pose.orientation.z = rotation[2]
    pose_msg.pose.orientation.w = rotation[3]
    pose_publishers[topic_name].publish(pose_msg)

def add_lists(totals, totals_tmp):
    totals[0] += totals_tmp[0]
    totals[1] += totals_tmp[1]
    totals[2] += totals_tmp[2]
    return totals

def print_configuration(natnet_client):
    natnet_client.refresh_configuration()
    print("Connection Configuration:")
    print("  Client:          %s" % natnet_client.local_ip_address)
    print("  Server:          %s" % natnet_client.server_ip_address)
    print("  Command Port:    %d" % natnet_client.command_port)
    print("  Data Port:       %d" % natnet_client.data_port)

    changeBitstreamString = "  Can Change Bitstream Version = "
    if natnet_client.use_multicast:
        print("  Using Multicast")
        print("  Multicast Group: %s" % natnet_client.multicast_address)
        changeBitstreamString += "false"
    else:
        print("  Using Unicast")
        changeBitstreamString += "true"

    # NatNet Server Info
    application_name = natnet_client.get_application_name()
    nat_net_requested_version = natnet_client.get_nat_net_requested_version()
    nat_net_version_server = natnet_client.get_nat_net_version_server()
    server_version = natnet_client.get_server_version()

    print("  NatNet Server Info")
    print("    Application Name %s" % (application_name))
    print("    MotiveVersion  %d %d %d %d" % (server_version[0], server_version[1], server_version[2], server_version[3]))
    print("    NatNetVersion  %d %d %d %d" % (nat_net_version_server[0], nat_net_version_server[1], nat_net_version_server[2], nat_net_version_server[3]))
    print("  NatNet Bitstream Requested")
    print("    NatNetVersion  %d %d %d %d" % (nat_net_requested_version[0], nat_net_requested_version[1],
                                              nat_net_requested_version[2], nat_net_requested_version[3]))

    print(changeBitstreamString)
    print("  PythonVersion    %s" % (sys.version))

def print_commands(can_change_bitstream):
    outstring = "Commands:\n"
    outstring += "Return Data from Motive\n"
    outstring += "  s  send data descriptions\n"
    outstring += "  r  resume/start frame playback\n"
    outstring += "  p  pause frame playback\n"
    outstring += "     pause may require several seconds\n"
    outstring += "     depending on the frame data size\n"
    outstring += "Change Working Range\n"
    outstring += "  o  reset Working Range to: start/current/end frame 0/0/end of take\n"
    outstring += "  w  set Working Range to: start/current/end frame 1/100/1500\n"
    outstring += "Return Data Display Modes\n"
    outstring += "  j  print_level = 0 supress data description and mocap frame data\n"
    outstring += "  k  print_level = 1 show data description and mocap frame data\n"
    outstring += "  l  print_level = 20 show data description and every 20th mocap frame data\n"
    outstring += "Change NatNet data stream version (Unicast only)\n"
    outstring += "  3  Request NatNet 3.1 data stream (Unicast only)\n"
    outstring += "  4  Request NatNet 4.1 data stream (Unicast only)\n"
    outstring += "General\n"
    outstring += "  t  data structures self test (no motive/server interaction)\n"
    outstring += "  c  print configuration\n"
    outstring += "  h  print commands\n"
    outstring += "  q  quit\n"
    outstring += "\n"
    outstring += "NOTE: Motive frame playback will respond differently in\n"
    outstring += "       Endpoint, Loop, and Bounce playback modes.\n"
    outstring += "\n"
    outstring += "EXAMPLE: PacketClient [serverIP [ clientIP [ Multicast/Unicast]]]\n"
    outstring += "         PacketClient \"192.168.10.14\" \"192.168.10.14\" Multicast\n"
    outstring += "         PacketClient \"127.0.0.1\" \"127.0.0.1\" u\n"
    outstring += "\n"
    print(outstring)

def request_data_descriptions(s_client):
    # 请求模型定义
    s_client.send_request(s_client.command_socket, s_client.NAT_REQUEST_MODELDEF, "", (s_client.server_ip_address, s_client.command_port))

def test_classes():
    totals = [0, 0, 0]
    print("Test Data Description Classes")
    totals_tmp = DataDescriptions.test_all()
    totals = add_lists(totals, totals_tmp)
    print("")
    print("Test MoCap Frame Classes")
    totals_tmp = MoCapData.test_all()
    totals = add_lists(totals, totals_tmp)
    print("")
    print("All Tests totals")
    print("--------------------")
    print("[PASS] Count = %3.1d" % totals[0])
    print("[FAIL] Count = %3.1d" % totals[1])
    print("[SKIP] Count = %3.1d" % totals[2])

def get_local_ip():
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("8.8.8.8", 80))
        local_ip = s.getsockname()[0]
        s.close()
        return local_ip
    except Exception as e:
        print(f"获取本地 IP 地址时出错: {e}")
        return None

def signal_handler(signum, frame):
    """处理Ctrl+C信号的函数（仅设标志并通知 ROS，具体收尾由主线程在 cleanup_and_exit 中执行）"""
    global shutdown_flag
    print("\n接收到中断信号 (Ctrl+C)，正在退出...")
    shutdown_flag = True
    rospy.signal_shutdown("用户中断")

def cleanup_and_exit():
    """清理资源并退出"""
    global streaming_client
    print("正在清理资源...")
    
    if streaming_client:
        streaming_client.shutdown()
    
    print("程序已安全退出")
    sys.exit(0)

if __name__ == "__main__":
    # 注册信号处理器
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    
    # 初始化ROS节点
    rospy.init_node('optitrack_data_publisher')

    optionsDict = {}
    optionsDict["clientAddress"] = get_local_ip()
    if optionsDict["clientAddress"] is None:
        print("无法获取本地 IP 地址，退出程序。")
        sys.exit(1)
    
    # 服务器 IP：优先命令行参数（rospy.myargv 去掉 ROS 重映射），其次环境变量 MOCAP_SERVER_IP，否则交互输入
    default_server_ip = "192.168.8.217"
    server_ip = None
    argv = rospy.myargv()
    if len(argv) > 1 and str(argv[1]).strip():
        server_ip = str(argv[1]).strip()
    elif os.environ.get("MOCAP_SERVER_IP", "").strip():
        server_ip = os.environ.get("MOCAP_SERVER_IP").strip()

    if server_ip:
        optionsDict["serverAddress"] = server_ip
        print(f"使用指定服务器IP: {server_ip}")
    else:
        while True:
            user_input = input(f"是否使用默认服务器IP地址 {default_server_ip} ? (y/n): ").strip().lower()
            if user_input == 'y' or user_input == 'yes':
                optionsDict["serverAddress"] = default_server_ip
                print(f"使用默认服务器IP: {default_server_ip}")
                break
            elif user_input == 'n' or user_input == 'no':
                while True:
                    new_ip = input("请输入新的服务器IP地址: ").strip()
                    if new_ip:
                        optionsDict["serverAddress"] = new_ip
                        print(f"使用自定义服务器IP: {new_ip}")
                        break
                    else:
                        print("IP地址不能为空，请重新输入。")
                break
            else:
                print("请输入 'y' 或 'n'")
    
    optionsDict["use_multicast"] = False
    optionsDict["stream_type"] = "d"

    # 创建新的NatNet客户端（print_level=0 不打印每帧，避免刷屏）
    streaming_client = NatNetClient()
    streaming_client.set_print_level(0)
    streaming_client.set_client_address(optionsDict["clientAddress"])
    streaming_client.set_server_address(optionsDict["serverAddress"])

    # 流媒体客户端配置
    streaming_client.new_frame_listener = receive_new_frame
    # streaming_client.new_frame_with_data_listener = receive_new_frame_with_data
    streaming_client.rigid_body_listener = receive_rigid_body_frame
    streaming_client.data_descriptions_listener = on_data_descriptions

    # 启动流媒体客户端
    streaming_client.set_use_multicast(optionsDict["use_multicast"])
    is_running = streaming_client.run(optionsDict["stream_type"])
    if not is_running:
        print("ERROR: Could not start streaming client.")
        sys.exit(1)

    time.sleep(1)
    if streaming_client.connected() is False:
        print("ERROR: Could not connect properly.  Check that Motive streaming is on.")
        sys.exit(2)

    print_configuration(streaming_client)
    print("\n")
    print_commands(streaming_client.can_change_bitstream_version())

    request_data_descriptions(streaming_client)
    
    print("程序已启动，按 Ctrl+C 退出...")

    try:
        # 使用更短的睡眠时间，并检查shutdown_flag
        while not rospy.is_shutdown() and not shutdown_flag:
            time.sleep(0.01)  # 减少睡眠时间，提高响应性
    except KeyboardInterrupt:
        print("\n程序被用户中断 (Ctrl+C)")
        cleanup_and_exit()
    except Exception as e:
        print(f"程序运行时出错: {e}")
        cleanup_and_exit()
    finally:
        cleanup_and_exit()
