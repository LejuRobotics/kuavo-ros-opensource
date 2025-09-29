#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import subprocess
import time
import os
from std_msgs.msg import String
from sensor_msgs.msg import Image


class CameraManagementNode:
    def __init__(self):
        # 初始化节点
        rospy.init_node('camera_management_node', anonymous=True)
        
        # # 创建订阅者
        # self.image_sub = rospy.Subscriber('/camera/image_raw', Image, self.image_callback)
        #
        # # 创建发布者
        # self.processed_image_pub = rospy.Publisher('/camera/image_processed', Image, queue_size=10)
        # self.status_pub = rospy.Publisher('/camera/status', String, queue_size=10)
        # self.camera_status_pub = rospy.Publisher('/camera/detection_status', String, queue_size=10)
        #
        # 参数服务器
        self.camera_id = rospy.get_param('~camera_id', 'default_camera')
        self.enable_processing = rospy.get_param('~enable_processing', True)
        self.remote_host = rospy.get_param('~remote_host', '192.168.26.12')  # 远程主机地址
        self.remote_user = rospy.get_param('~remote_user', 'leju_kuavo')  # 远程主机用户名
        self.remote_camera_launch = rospy.get_param('~remote_camera_launch', 'roslaunch realsense2_camera rs_camera.launch')  # 远程启动相机的命令
        self.local_camera_launch = rospy.get_param('~local_camera_launch', 'roslaunch realsense2_camera rs_camera.launch')  # 本地启动相机的命令
        
        # # 发布节点状态
        # self.status_pub.publish("Camera Management Node initialized with camera_id: {}".format(self.camera_id))
        #
        rospy.loginfo("Camera Management Node started with camera_id: %s", self.camera_id)

    def check_camera_devices(self):
        """检查系统中是否存在Intel RealSense相机设备，返回布尔值"""
        try:
            # 先检查本地Intel RealSense相机设备
            local_detected = self.check_local_realsense_devices()
            
            # 如果本地检测到相机，则启动本地相机
            if local_detected:
                self.start_local_camera()
                return True
            
            # 如果本地没有检测到相机且配置了远程主机，则通过SSH检查远程设备上的相机
            if not local_detected and self.remote_host:
                remote_detected = self.check_remote_realsense_devices()
                # 如果在远程设备上检测到相机，则启动远程相机
                if remote_detected:
                    self.start_remote_camera()
                return remote_detected
            else:
                return local_detected
                
        except Exception as e:
            rospy.logerr("Error when checking RealSense camera devices: %s", str(e))
            # self.camera_status_pub.publish(f"RealSense camera detected: Error - {str(e)}")
            return False

    def check_local_realsense_devices(self):
        """检查本地系统中的Intel RealSense相机设备"""
        try:
            # 方法1: 使用rs-enumerate-devices检查连接的RealSense设备
            result = subprocess.run(['rs-enumerate-devices'], capture_output=True, text=True, timeout=10)
            if result.returncode == 0:
                output = result.stdout
                if 'Intel RealSense' in output or 'RealSense' in output:
                    rospy.loginfo("Intel RealSense camera detected locally")
                    # self.camera_status_pub.publish("Intel RealSense camera detected: True")
                    return True
                else:
                    rospy.logdebug("No Intel RealSense camera detected locally")
                    # self.camera_status_pub.publish("Intel RealSense camera detected: False")
                    return False
            else:
                # 方法2: 使用lsusb检查USB设备
                result = subprocess.run(['lsusb'], capture_output=True, text=True, timeout=10)
                if result.returncode == 0:
                    usb_devices = result.stdout
                    # Intel RealSense设备的Vendor ID通常是8086 (Intel)
                    if '8086:0b07' in usb_devices or '8086:0b3a' in usb_devices or '8086:0b5c' in usb_devices:
                        rospy.loginfo("Intel RealSense camera detected via lsusb")
                        # self.camera_status_pub.publish("Intel RealSense camera detected: True")
                        return True
                    
                    # 或者通过设备名称查找
                    if 'RealSense' in usb_devices:
                        rospy.loginfo("Intel RealSense camera detected via lsusb (by name)")
                        # self.camera_status_pub.publish("Intel RealSense camera detected: True")
                        return True
                
                rospy.logdebug("No Intel RealSense camera detected locally")
                # self.camera_status_pub.publish("Intel RealSense camera detected: False")
                return False
                
        except subprocess.TimeoutExpired:
            rospy.logerr("Timeout when trying to check local RealSense camera devices")
            # self.camera_status_pub.publish("Intel RealSense camera detected: Error - Timeout")
            return False
        except Exception as e:
            rospy.logerr("Error when checking local RealSense camera devices: %s", str(e))
            # self.camera_status_pub.publish(f"Intel RealSense camera detected: Error - {str(e)}")
            return False

    def check_remote_realsense_devices(self):
        """通过SSH检查远程设备上的Intel RealSense相机设备"""
        try:
            # 构建SSH命令来检查远程设备上的RealSense相机
            ssh_command = [
                'ssh', 
                '-o', 'ConnectTimeout=10',
                '-o', 'BatchMode=yes',
                f'{self.remote_user}@{self.remote_host}',
                'rs-enumerate-devices 2>/dev/null | grep -i realsense || lsusb | grep -i realsense || lsusb | grep "8086:0b07\|8086:0b3a\|8086:0b5c"'
            ]
            
            rospy.loginfo("Checking Intel RealSense camera devices on remote host: %s@%s", self.remote_user, self.remote_host)
            
            result = subprocess.run(ssh_command, capture_output=True, text=True, timeout=15)
            
            if result.returncode == 0 and result.stdout.strip():
                rospy.loginfo("Intel RealSense camera device detected on %s@%s", self.remote_user, self.remote_host)
                # self.camera_status_pub.publish(f"Intel RealSense camera detected on {self.remote_host}: True")
                return True
            else:
                rospy.loginfo("No Intel RealSense camera devices detected on remote host %s@%s", self.remote_user, self.remote_host)
                # self.camera_status_pub.publish(f"Intel RealSense camera detected on {self.remote_host}: False")
                return False
                
        except subprocess.TimeoutExpired:
            rospy.logerr("Timeout when trying to check remote RealSense camera devices on %s@%s", self.remote_user, self.remote_host)
            # self.camera_status_pub.publish(f"Intel RealSense camera detected on {self.remote_host}: Error - Timeout")
            return False
        except Exception as e:
            rospy.logerr("Error when checking remote RealSense camera devices on %s@%s: %s", self.remote_user, self.remote_host, str(e))
            # self.camera_status_pub.publish(f"Intel RealSense camera detected on {self.remote_host}: Error - {str(e)}")
            return False

    def start_local_camera(self):
        """启动本地相机"""
        try:
            if not self.local_camera_launch:
                rospy.logdebug("Local camera launch command not specified")
                return False
                
            # 启动本地相机
            rospy.loginfo("Starting local camera with command: %s", self.local_camera_launch)
            
            # 在后台运行启动命令
            subprocess.Popen(self.local_camera_launch, shell=True)
            
            rospy.loginfo("Local camera start command issued")
            # self.camera_status_pub.publish("Local camera start: Command issued")
            return True
            
        except Exception as e:
            rospy.logerr("Error when starting local camera: %s", str(e))
            # self.camera_status_pub.publish(f"Local camera start: Error - {str(e)}")
            return False

    def start_remote_camera(self):
        """在远程设备上启动相机"""
        try:
            if not self.remote_camera_launch:
                rospy.logwarn("Remote camera launch command not specified")
                return False
                
            # 构建SSH命令来启动远程相机
            ssh_command = [
                'ssh', 
                '-o', 'ConnectTimeout=10',
                '-o', 'BatchMode=yes',
                f'{self.remote_user}@{self.remote_host}',
                self.remote_camera_launch
            ]
            
            rospy.loginfo("Starting camera on remote host: %s@%s", self.remote_user, self.remote_host)
            
            result = subprocess.run(ssh_command, capture_output=True, text=True, timeout=20)
            
            if result.returncode == 0:
                rospy.loginfo("Remote camera started successfully on %s@%s", self.remote_user, self.remote_host)
                # self.camera_status_pub.publish(f"Remote camera started on {self.remote_host}: Success")
                return True
            else:
                rospy.logerr("Failed to start remote camera on %s@%s: %s", self.remote_user, self.remote_host, result.stderr)
                # self.camera_status_pub.publish(f"Remote camera started on {self.remote_host}: Failed")
                return False
                
        except subprocess.TimeoutExpired:
            rospy.logerr("Timeout when trying to start remote camera on %s@%s", self.remote_user, self.remote_host)
            # self.camera_status_pub.publish(f"Remote camera started on {self.remote_host}: Timeout")
            return False
        except Exception as e:
            rospy.logerr("Error when starting remote camera on %s@%s: %s", self.remote_user, self.remote_host, str(e))
            # self.camera_status_pub.publish(f"Remote camera started on {self.remote_host}: Error - {str(e)}")
            return False


    def run(self):
        """运行节点"""
        # # 定期检查相机设备
        # rate = rospy.Rate(0.2)  # 每5秒检查一次
        # while not rospy.is_shutdown():
        #     camera_detected = self.check_camera_devices()
        #     rospy.loginfo("RealSense camera detection result: %s", camera_detected)
        #     rate.sleep()

        self.check_camera_devices()


if __name__ == '__main__':
    try:
        node = CameraManagementNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down camera management node")