# -*- coding: utf-8 -*-
import queue
import os
import socket
import threading
import time
import rospy
from typing import Dict, Optional
from std_msgs.msg import Float64MultiArray,Float32
from diagnostic_msgs.msg import DiagnosticStatus
from sensor_msgs.msg import JointState

from kuavo_msgs.msg import lejuClawState

from google.protobuf.timestamp_pb2 import Timestamp
from . import body_tracking_extended_pb2 as proto
from . import delayed_diagnosis_pb2
from . import hand_wrench_srv_pb2
from ..config.pico_vr_config import PicoVrConfig, HandWrenchConfig


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
        self.only_push_half_body_data = True
        self.control_mode = "mobile_mpc"
        self.id = 0
        
        # 初始化配置管理器
        self.config_manager = PicoVrConfig()

    def start(self):
        try:
            self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

            self.sensor_queue = queue.Queue(maxsize=1000)
            self.hand_queue = queue.Queue(maxsize=1000)
            self.claw_queue = queue.Queue(maxsize=1000)

            # 延迟诊断队列
            self.delayed_diagnosis_result_queue = queue.Queue(maxsize=100)
            self.delayed_diagnosis_progress_queue = queue.Queue(maxsize=100)

            # 物品质量与力队列
            self.item_mass_force_response_queue = queue.Queue(maxsize=100)

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

        # 延迟诊断
        rospy.Subscriber("/pico/system_identification/result", DiagnosticStatus, self.delayed_diagnosis_result_callback)
        rospy.Subscriber("/pico/system_identification/progress", Float32, self.delayed_diagnosis_progress_callback)
    def send_to_client(self, message):
        if not self.target_client_addr:
            # rospy.logwarn("未指定目标客户端地址")
            return False

        try:
            self.server_socket.sendto(message, self.target_client_addr)
            return True
        except Exception as e:
            rospy.logerr(f"发送数据到 {self.target_client_addr} 失败: {e}")
            return False

    def sensor_data_cur_callback(self, data):
        now = time.time()
        if now - self.last_sensor_time >= self.downsample_interval:
            if self.only_push_half_body_data:
                data.data = [abs(x) for x in data.data[12:26]]
                self.sensor_queue.put(data.data)
            else:
                data.data = [abs(x) for x in data.data]
                self.sensor_queue.put(data.data)
            self.last_sensor_time = now


    def hand_data_cur_callback(self, data):
        if self.eef_type == "qiangnao":
            now = time.time()
            if now - self.last_hand_time >= self.downsample_interval:
                hand_data = [x * 6 / 1000 for x in data.effort]  # 将0-100映射到0~0.6A
                hand_data = [abs(x) for x in hand_data]
                self.hand_queue.put(hand_data)
                self.last_hand_time = now


    def claw_data_cur_callback(self, data):
        if self.eef_type == "lejuclaw":
            now = time.time()
            effort = data.data.effort
            if len(effort) == 2:
                # 每个值后面补5个0
                padded_effort = [effort[0]] + [0.0]*5 + [effort[1]] + [0.0]*5
                padded_effort = [abs(x) for x in padded_effort]
                self.claw_queue.put(padded_effort)
                self.last_claw_time = now
            else:
                rospy.logwarn(f"收到的 claw effort 长度不为2: {effort}")

    def delayed_diagnosis_result_callback(self, data):
        """延迟诊断结果回调"""
        self.delayed_diagnosis_result_queue.put(data)
        
    def delayed_diagnosis_progress_callback(self, data):
        """延迟诊断进度回调"""
        self.delayed_diagnosis_progress_queue.put(data)

    def _process_delayed_diagnosis_result(self):
        # 延迟诊断进度处理
        def get_result():
            try:
                return self.delayed_diagnosis_result_queue.get_nowait()
            except queue.Empty:
                return None
        def get_progress():
            try:
                return self.delayed_diagnosis_progress_queue.get_nowait()
            except queue.Empty:
                return None
            
        result = get_result()
        progress = get_progress()
        if result or progress: 
            proto_msg = delayed_diagnosis_pb2.DelayedDiagnosisResult()
            if result:
                proto_msg.progress = 100
                proto_msg.message = result.message
                if result.level == DiagnosticStatus.OK:
                    # 成功
                    proto_msg.status = delayed_diagnosis_pb2.Status.SUCCESS
                elif result.level == DiagnosticStatus.WARN:
                    # 强制停止
                    proto_msg.status = delayed_diagnosis_pb2.Status.FORCE_KILLED
                else:
                    # 失败
                    proto_msg.status = delayed_diagnosis_pb2.Status.FAILED
                # 赋值结果
                for kv in result.values:
                    try:
                        # 直接设置字段值，不使用HasField检查
                        setattr(proto_msg.result, kv.key, float(kv.value))
                    except Exception as e:
                        # rospy.logwarn(f"设置字段 {kv.key} 失败: {e}")
                        continue
                # 清空进度队列
                while not self.delayed_diagnosis_progress_queue.empty():
                    try:
                        self.delayed_diagnosis_progress_queue.get_nowait()
                    except queue.Empty:
                        break
                print(f"延迟诊断结果: {proto_msg}")   
            elif progress:
                # 更新诊断进度
                proto_msg.progress = progress.data
                proto_msg.status = delayed_diagnosis_pb2.Status.IN_PROGRESS
                proto_msg.message = "延迟诊断进行中"
            return proto_msg
        else:
            return None

    def _process_item_mass_force_response(self):
        # 物品质量与力响应处理
        try:
            return self.item_mass_force_response_queue.get_nowait()
        except queue.Empty:
            return None
        return None

    def _periodic_push(self):
        last_eef_cur = None
        last_motor_cur = [0.0] * 28
        while self.running:
            if not self.target_client_addr:
                # rospy.logwarn("未指定目标客户端地址，等待中...")
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

                message = proto.VRData()

                header = proto.Header()
                timestamp = Timestamp()
                timestamp.GetCurrentTime()
                header.timestamp.CopyFrom(timestamp)
                header.id = self.id
                message.header.CopyFrom(header)
        
                robot_data = proto.RobotData()
                robot_data.motor_cur.extend(motor_cur)
                robot_data.eef_cur.extend(eef_cur)
                
                # 延迟诊断结果处理
                try:
                    delayed_diagnosis_result = self._process_delayed_diagnosis_result()
                    if delayed_diagnosis_result:
                        robot_data.delayed_diagnosis_result.CopyFrom(delayed_diagnosis_result)
                        # print(f"延迟诊断结果: {delayed_diagnosis_result}")
                except Exception as e:
                    pass
                    # print(f"延迟诊断结果处理出错: {e}")

                # 物品重量/末端力响应
                imf_res = self._process_item_mass_force_response()
                if imf_res:
                    # print(f"imf_res: {imf_res.item_mass_forces}")
                    robot_data.item_mass_force_response.CopyFrom(imf_res)
                    # print(f"robot_data: {robot_data}")
                control_mode_msg = proto.ControlMode()
                control_mode_msg.control_mode = self.control_mode
                robot_data.control_mode.CopyFrom(control_mode_msg)
                message.robot_data.CopyFrom(robot_data)

                self.send_to_client(message.SerializeToString())
                self.id += 1
            except queue.Empty:
                rospy.logwarn("传感器数据获取超时，motor_cur 队列为空")
            except Exception as e:
                rospy.logerr(f"定期推送线程出错: {e}")

            time2 = time.time()
            time.sleep(max(0, 1/self.rate - (time2 - time1)))
    
    def push_item_mass_force_config(self):
        """推送手部力矩配置"""
        res = hand_wrench_srv_pb2.ItemMassForceResponse()
        res.operation = hand_wrench_srv_pb2.ItemMassForceOperation.GET
        success, errmsg, hand_wrench_cases = self.config_manager.get_all_hand_wrench_cases()
        if not success:
            res.status = hand_wrench_srv_pb2.ItemMassForceResponse.OperationStatus.ERROR
            res.description = errmsg
        else:
            for case_name, config in hand_wrench_cases.items():
                # print(f"case_name: {config}")
                item_mass_force = hand_wrench_srv_pb2.ItemMassForce()
                item_mass_force.case_name = case_name
                item_mass_force.description = config.description
                item_mass_force.item_mass = config.itemMass
                item_mass_force.lforce_x = config.lforceX
                item_mass_force.lforce_y = config.lforceY
                item_mass_force.lforce_z = config.lforceZ
                res.item_mass_forces.append(item_mass_force)
            res.status = hand_wrench_srv_pb2.ItemMassForceResponse.OperationStatus.SUCCESS
            res.description = "success"    
        self.add_item_mass_force_response(res)

    def add_item_mass_force_response(self, response: hand_wrench_srv_pb2.ItemMassForceResponse):
        # print(f"add_item_mass_force_response: {response}")
        self.item_mass_force_response_queue.put(response)
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
