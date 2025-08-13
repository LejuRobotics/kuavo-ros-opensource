# -*- coding: utf-8 -*-
import socket
import threading
import json
import time
import logging
import rospy
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
from kuavo_msgs.msg import lejuClawState
from datetime import datetime
import queue
import os

class RobotDataServer:
    def __init__(self, eef_type=None, target_client_ip=None, target_client_port=None, rate=20):
        self.robot_name = os.getenv("ROBOT_NAME", "KUAVO")
        self.server_socket = None
        self.eef_type = eef_type
        self.target_client_addr = (target_client_ip, target_client_port) if target_client_ip and target_client_port else None
        self.running = False
        self.last_sensor_time = 0
        self.last_hand_time = 0
        self.last_claw_time = 0
        self.rate = rate
        self.downsample_interval = 1 / self.rate  # 50ms = 20Hz

    def start(self):
        try:
            self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

            self.sensor_queue = queue.Queue(maxsize=1000)
            self.hand_queue = queue.Queue(maxsize=1000)
            self.claw_queue = queue.Queue(maxsize=1000)
            self.hand_queue.put([0.0] * 12)
            self.claw_queue.put([0.0] * 2)
            self.running = True

            if not rospy.core.is_initialized():
                rospy.init_node('socket_server')
            self.init_topic()

            threading.Thread(target=self._periodic_push, daemon=True).start()
            threading.Thread(target=rospy.spin, daemon=True).start()

            while not rospy.is_shutdown() and self.running:
                time.sleep(1)

        except KeyboardInterrupt:
            rospy.loginfo("收到 Ctrl+C，正在关闭服务器...")
        except Exception as e:
            rospy.logerr(f"服务器启动失败: {e}")
        finally:
            self.stop()

    def stop(self):
        self.running = False
        if self.server_socket:
            try:
                self.server_socket.close()
            except:
                pass
        rospy.loginfo("服务器已停止")

    def init_topic(self):
        rospy.Subscriber("/sensor_data_motor/motor_cur", Float64MultiArray, self.sensor_data_cur_callback)
        rospy.Subscriber("dexhand/state", JointState, self.hand_data_cur_callback)
        rospy.Subscriber("/leju_claw_state", lejuClawState, self.claw_data_cur_callback)

    def send_to_client(self, message):
        if not self.target_client_addr:
            rospy.logwarn("未指定目标客户端地址")
            return False

        try:
            if isinstance(message, str):
                message = message.encode('utf-8')
            self.server_socket.sendto(message, self.target_client_addr)
            return True
        except Exception as e:
            rospy.logerr(f"发送数据到 {self.target_client_addr} 失败: {e}")
            return False

    def sensor_data_cur_callback(self, data):
        now = time.time()
        if now - self.last_sensor_time >= self.downsample_interval:
            self.sensor_queue.put(data.data)
            self.last_sensor_time = now


    def hand_data_cur_callback(self, data):
        if self.eef_type == "qiangnao":
            now = time.time()
            if now - self.last_hand_time >= self.downsample_interval:
                hand_data = [x * 6 / 1000 for x in data.effort]  # 将0-100映射到0~0.6A
                self.hand_queue.put(hand_data)
                self.last_hand_time = now


    def claw_data_cur_callback(self, data):
        if self.eef_type == "lejuclaw":
            now = time.time()
            if now - self.last_claw_time >= self.downsample_interval:
                self.claw_queue.put(data.data.effort)
                self.last_claw_time = now

    def _periodic_push(self):
        last_eef_cur = None
        last_motor_cur = [0.0] * 28
        while self.running:
            if not self.target_client_addr:
                rospy.logwarn("未指定目标客户端地址，等待中...")
                time.sleep(1)  # 阻塞等待 1 秒再检查
            time1 = time.time()
            try:
                try:
                    motor_cur = self.sensor_queue.get(timeout=0.002)
                    last_motor_cur = motor_cur
                except queue.Empty:
                    motor_cur = last_motor_cur

                if self.eef_type == "qiangnao":
                    try:
                        eef_cur = self.hand_queue.get_nowait()
                        last_eef_cur = eef_cur
                    except queue.Empty:
                        eef_cur = last_eef_cur
                elif self.eef_type == "lejuclaw":
                    try:
                        eef_cur = self.claw_queue.get_nowait()
                        last_eef_cur = eef_cur
                    except queue.Empty:
                        eef_cur = last_eef_cur
                else:
                    eef_cur = []

                push_data = {
                    "motor_cur": motor_cur,
                    "eef_cur": eef_cur,
                }
                self.send_to_client(json.dumps(push_data))

            except queue.Empty:
                rospy.logwarn("传感器数据获取超时，motor_cur 队列为空")
            except Exception as e:
                rospy.logerr(f"定期推送线程出错: {e}")

            time2 = time.time()
            time.sleep(max(0, 1/self.rate - (time2 - time1)))


def main():
    target_ip = "192.168.1.100"  # <-- 替换为目标客户端 IP
    target_port = 9000           # <-- 替换为目标客户端端口
    server = RobotDataServer(eef_type="qiangnao", target_client_ip=target_ip, target_client_port=target_port)
    try:
        server.start()
    except KeyboardInterrupt:
        rospy.loginfo("收到中断信号，正在关闭服务器...")
        server.stop()

if __name__ == "__main__":
    main()
