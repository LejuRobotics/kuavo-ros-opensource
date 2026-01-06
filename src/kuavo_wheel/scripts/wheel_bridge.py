#!/usr/bin/env python3
import os
import time
import multiprocessing
import rospy
from nav_msgs.msg import Odometry
import signal
import sys
from geometry_msgs.msg import Twist
import copy
import argparse

STOP_THRESHOLD = 0.5


def kuavo_process(master_uri, shared_data, kuavo_slave_ip):
    """kuavo进程 - 订阅cmd_vel，发布odom，连接到 ROS Master A"""
    try:
        # 彻底清理和重新设置环境变量
        os.environ.clear()
        # 重新设置必要的环境变量
        os.environ['ROS_MASTER_URI'] = master_uri
        os.environ['ROS_ROOT'] = '/opt/ros/noetic/share/ros'
        os.environ['ROS_IP'] = kuavo_slave_ip
        os.environ['ROS_PACKAGE_PATH'] = '/opt/ros/noetic/share'
        os.environ['ROS_ETC_DIR'] = '/opt/ros/noetic/etc/ros'
        os.environ['ROS_DISTRO'] = 'noetic'
        
        print(f"[Kuavo Process] Connecting to ROS Master: {master_uri}")
        print(f"[Kuavo Process] Current ROS_MASTER_URI: {os.environ.get('ROS_MASTER_URI')}")
        print(f"[Kuavo Process] Using ROS_IP: {os.environ.get('ROS_IP')}")
        
        rospy.init_node('kuavo_bridge_node', anonymous=True)
        print(f"[Kuavo Process] Node initialized: {rospy.get_name()}")
        
        # 验证连接到的 ROS Master
        try:
            master_uri_actual = rospy.get_master().getUri()
            print(f"[Kuavo Process] Actually connected to: {master_uri_actual}")
        except Exception as e:
            print(f"[Kuavo Process] Error getting master URI: {e}")
        
        # 创建odom发布者
        odom_pub = rospy.Publisher('/odom', Odometry, queue_size=10)
        print("[Kuavo Process] Publisher created for /odom")

        def cmd_vel_callback(msg):
            with shared_data['cmd_vel_lock']:
                shared_data['cmd_vel_data'] = copy.deepcopy(msg)
                shared_data['cmd_vel_time'] = rospy.Time.now().to_sec()
        rospy.Subscriber('/move_base/base_cmd_vel', Twist, cmd_vel_callback)
        print("[Kuavo Process] Subscribed to /move_base/base_cmd_vel")
        
        none_data_cnt = 0
        # 发布odom消息
        rate = rospy.Rate(50)
        while not rospy.is_shutdown():
            try:
                # 从共享数据获取odom消息并发布
                with shared_data['odom_lock']:
                    if shared_data['odom_data'] is not None:
                        odom_pub.publish(shared_data['odom_data'])
                        shared_data['odom_data'] = None
                        # print(f"[Kuavo Process] Sent odom to /odom")
                    else: none_data_cnt = none_data_cnt + 1
                if none_data_cnt > 100:
                    print(f"[Kuavo Process] No odom data received for {none_data_cnt} times")
                    none_data_cnt = 0
            except Exception as e:
                # 队列为空或其他错误，继续循环
                pass
            rate.sleep()
        
    except Exception as e:
        print(f"[Kuavo Process] Error: {e}")

def wheel_process(master_uri, shared_data, wheel_slave_ip):
    """wheel进程 - 订阅odom，发布cmd_vel，连接到 ROS Master B"""
    try:
        # 彻底清理和重新设置环境变量
        os.environ.clear()
        # 重新设置必要的环境变量
        os.environ['ROS_MASTER_URI'] = master_uri
        os.environ['ROS_ROOT'] = '/opt/ros/noetic/share/ros'
        os.environ['ROS_IP'] = wheel_slave_ip
        os.environ['ROS_PACKAGE_PATH'] = '/opt/ros/noetic/share'
        os.environ['ROS_ETC_DIR'] = '/opt/ros/noetic/etc/ros'
        os.environ['ROS_DISTRO'] = 'noetic'
        
        print(f"[Wheel Process] Connecting to ROS Master: {master_uri}")
        print(f"[Wheel Process] Current ROS_MASTER_URI: {os.environ.get('ROS_MASTER_URI')}")
        print(f"[Wheel Process] Using ROS_IP: {os.environ.get('ROS_IP')}")
        
        rospy.init_node('wheel_bridge_node', anonymous=True)
        print(f"[Wheel Process] Node initialized: {rospy.get_name()}")
        
        # 验证连接到的 ROS Master
        try:
            master_uri_actual = rospy.get_master(os.environ).getUri()
            print(f"[Wheel Process] Actually connected to: {master_uri_actual}")
        except Exception as e:
            print(f"[Wheel Process] Error getting master URI: {e}")
        
        # 创建cmd_vel发布者
        cmd_vel_pub = rospy.Publisher('/base_cmd_vel', Twist, queue_size=10)
        print("[Wheel Process] Publisher created for /base_cmd_vel")
        
        def odom_callback(msg):
        
            with shared_data['odom_lock']:
                shared_data['odom_data'] = copy.deepcopy(msg)
        rospy.Subscriber('/odom', Odometry, odom_callback)
        print("[Wheel Process] Subscribed to /odom")
        
        def check_timestemp(msg):
            now = rospy.Time.now()
            return (now.to_sec() - shared_data['cmd_vel_time']) <= STOP_THRESHOLD
        stopped = False

        # 发布cmd_vel消息
        rate = rospy.Rate(50)
        while not rospy.is_shutdown():
            try:
                msg_to_publish = None
                with shared_data['cmd_vel_lock']:
                    if shared_data['cmd_vel_data'] is not None:
                        if check_timestemp(shared_data['cmd_vel_data']):
                            msg_to_publish = shared_data['cmd_vel_data']
                            stopped = False
                        else:
                            if not stopped:
                                rospy.logwarn("[Wheel Process] cmd_vel timestamp is too old")
                            msg_to_publish = Twist()  # stop
                            stopped = True
                if msg_to_publish is not None:
                    cmd_vel_pub.publish(msg_to_publish)
            except Exception as e:
                rospy.logerr(f"[Wheel Process] Error in loop: {e}")
            rate.sleep()        
    except Exception as e:
        print(f"[Wheel Process] Error: {e}")

def main():
    # 创建参数解析器
    parser = argparse.ArgumentParser(description='ROS Bridge for Kuavo and Wheel communication')
    parser.add_argument('--kuavo-master', default='http://169.254.128.130:11311', 
                       help='Kuavo ROS Master URI (default: http://169.254.128.130:11311)')
    parser.add_argument('--wheel-master', default='http://169.254.128.2:11311', 
                       help='Wheel ROS Master URI (default: http://169.254.128.2:11311)')
    parser.add_argument('--kuavo-slave-ip', default='169.254.128.130', 
                       help='Kuavo slave IP address (default: 169.254.128.130)')
    parser.add_argument('--wheel-slave-ip', default='169.254.128.130', 
                       help='Wheel slave IP address (default: 169.254.128.130)')
    
    args = parser.parse_args()
    
    # 从命令行参数读取配置
    kuavo_master = args.kuavo_master
    wheel_master = args.wheel_master
    kuavo_slave_ip = args.kuavo_slave_ip
    wheel_slave_ip = args.wheel_slave_ip

    print(f"Starting ROS Bridge...")
    print(f"  Kuavo Master: {kuavo_master}")
    print(f"  Wheel Master: {wheel_master}")
    print(f"  Kuavo Slave IP: {kuavo_slave_ip}")
    print(f"  Wheel Slave IP: {wheel_slave_ip}")
    print("")

    manager = multiprocessing.Manager()
    shared_data = manager.dict()
    shared_data['cmd_vel_data'] = None
    shared_data['odom_data'] = None
    shared_data['cmd_vel_lock'] = manager.Lock()
    shared_data['odom_lock'] = manager.Lock()
    shared_data['cmd_vel_time'] = None
    # 创建两个进程
    # Kuavo进程: 订阅cmd_vel，发布odom
    kuavo_proc = multiprocessing.Process(target=kuavo_process, args=(kuavo_master, shared_data, kuavo_slave_ip))
    
    # Wheel进程: 订阅odom，发布cmd_vel
    wheel_proc = multiprocessing.Process(target=wheel_process, args=(wheel_master, shared_data, wheel_slave_ip))

    try:
        # 启动两个进程
        kuavo_proc.start()
        print("[Main] Kuavo process started")
        time.sleep(2)  # 等kuavo进程先启动
        
        wheel_proc.start()
        print("[Main] Wheel process started")

        print("[Main] Bridge started. Press Ctrl+C to stop.")

        # 等待两个进程
        kuavo_proc.join()
        wheel_proc.join()

    except KeyboardInterrupt:
        print("\n[Main] Interrupted by user")
        kuavo_proc.terminate()
        wheel_proc.terminate()
        
        kuavo_proc.join()
        wheel_proc.join()

if __name__ == '__main__':
    main() 
