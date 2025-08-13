#!/usr/bin/env python3
# coding: utf-8
# Copyright (c) Lejurobot 2025

import os
import time
import json
import rospy
import datetime
import threading
import subprocess

from std_msgs.msg import Float32
from diagnostic_msgs.msg import DiagnosticStatus,KeyValue
from std_srvs.srv import Empty, EmptyResponse
from kuavo_msgs.srv import (
                       changeArmCtrlMode, changeArmCtrlModeRequest,
                        changeTorsoCtrlMode, changeTorsoCtrlModeRequest)

"""
执行系统辨识，延迟诊断脚本
"""
class SystemIdentification:
    def __init__(self):
        self.process = None
        self._running = False
        self.process_lock = threading.Lock()
        self.running_lock = threading.Lock()
        self.timeout = 60*5 # 5mins
        self.estimated_duration = 2*60 # 2mins 正常执行成功需要时间，用来计算进度
        self.is_timeout = False
        self.is_force_killed = False
        self.OUTPUT_DIR_PREFIX='~/.ros/plots/pico-vr'
        self.output_dir = None
        self._init_ros()

    def start(self):
        """开启一个线程执行系统辨识命令"""
        with self.running_lock:
            if self._running:
                print("System identification is already running")
                return False
             
            self._running = True
            
            # 创建并启动执行线程
            execute_thread = threading.Thread(target=self._system_identification_func, daemon=True)
            execute_thread.start()
            
            # 启动超时检查线程
            timeout_thread = threading.Thread(target=self._timeout_check_func, daemon=True)
            timeout_thread.start()

            print("System identification started")
            return True
    def is_running(self)->bool:
        """检查执行是否完成"""
        with self.running_lock:
            return self._running

    def stop(self):
        """停止执行"""
        with self.running_lock:
            if not self._running:
                print("System identification is not running")
                self._publish_result(DiagnosticStatus.OK, "延迟诊断工具未执行, 无需停止", {})
                return
        with self.process_lock:
            self.is_force_killed = True
            if self.process and self.process.poll() is None:
                try:
                    self.process.terminate()
                    # 等待一段时间后强制杀死
                    time.sleep(2)
                    if self.process.poll() is None:
                        self.process.kill()
                    print("System identification process stopped")
                except Exception as e:
                    print(f"Error stopping system identification: {e}")

    def _execute_finished(self, retcode):
        # 重置手臂
        self._arm_reset()
        
        # 超时
        if self.is_timeout:
            print(f"process timeout, output dir: {self.output_dir}")
            self._publish_result(DiagnosticStatus.STALE, "延迟诊断工具执行超时, 耗费时间: " + str(self.timeout/60) + "分钟", {})
            return
        
        if self.is_force_killed:
            print(f"process force killed, output dir: {self.output_dir}")
            self._publish_result(DiagnosticStatus.WARN, "延迟诊断工具被强制停止", {})
            return
        
        # 诊断进程异常退出
        if retcode != 0:
            print(f"process return code: {retcode}, output dir: {self.output_dir}")
            self._publish_result(DiagnosticStatus.ERROR, "延迟诊断工具执行过程中出现错误, 错误码: " + str(retcode), {})
            return

        print(f"process return code: {retcode}, output dir: {self.output_dir}")

        # 发布完成进度
        if hasattr(self, 'progress_pub'):
            progress_msg = Float32()
            progress_msg.data = 100.0
            for _ in range(5):
                self.progress_pub.publish(progress_msg)
                time.sleep(0.1)

        # 检查输出目录和结果文件
        if not self.output_dir:
            self._publish_result(DiagnosticStatus.ERROR, "延迟诊断工具执行成功，但结果输出异常", {})
            return

        result_file = os.path.join(self.output_dir, 'system_identification_results.json')
        if not os.path.exists(result_file):
            self._publish_result(DiagnosticStatus.ERROR, "延迟诊断工具执行成功，但结果输出异常", {})
            return

        # 读取并发布结果
        try:
            with open(result_file, 'r') as f:
                result = json.load(f)
            # 判断是否有 success 字段
            print(result)
            if 'success' in result and not result['success']:
                if 'errmsg' in result:
                    self._publish_result(DiagnosticStatus.ERROR, result['errmsg'], result)
                else:
                    self._publish_result(DiagnosticStatus.ERROR, "延迟诊断工具执行失败", result)
            else:
                self._publish_result(DiagnosticStatus.OK, "成功", result)
        except Exception as e:
            print(f"Error reading result file: {e}")
            self._publish_result(DiagnosticStatus.ERROR, "延迟诊断工具执行成功，但结果输出异常", {})
    def _cleanup(self):
        if self.output_dir:
            try:
                import shutil
                shutil.rmtree(self.output_dir)
            except Exception as e:
                print(f"Error removing output directory {self.output_dir}: {e}")
        self.output_dir = None 

    def _publish_result(self, status, err_msg:str, result:dict):
        self._cleanup()
        msg = DiagnosticStatus()
        msg.level = status
        msg.message = err_msg
        for k,v in result.items():
            msg.values.append(KeyValue(key=k, value=str(v)))
        for _ in range(5):    
            self.result_pub.publish(msg)
            time.sleep(0.05)
    
    def _init_ros(self):
        self.result_pub = rospy.Publisher("/pico/system_identification/result", DiagnosticStatus, queue_size=1)
        self.progress_pub = rospy.Publisher("/pico/system_identification/progress", Float32, queue_size=5)
        self.service = rospy.Service("/pico/system_identification/start", Empty, self._system_identification_start_callback)
        self.service = rospy.Service("/pico/system_identification/stop", Empty, self._system_identification_stop_callback)

    def _system_identification_start_callback(self, req):
        self.start()
        return EmptyResponse()

    def _system_identification_stop_callback(self, req):
        self.stop()
        return EmptyResponse()

    def srv_change_manipulation_mpc_ctrl_mode(self, ctrl_mode:int)->bool:
        try:
            service_name = '/mobile_manipulator_mpc_control'
            rospy.wait_for_service(service_name, timeout=2.0)
            set_mode_srv = rospy.ServiceProxy(service_name, changeTorsoCtrlMode)
            
            req = changeTorsoCtrlModeRequest()
            req.control_mode = ctrl_mode
            
            resp = set_mode_srv(req)
            if not resp.result:
                print(f"Failed to change manipulation mpc control mode to {ctrl_mode}: {resp.message}")
            return resp.result
        except rospy.ServiceException as e:
            print(f"Service call to {service_name} failed: {e}")
        except rospy.ROSException as e: # For timeout from wait_for_service
            print(f"Failed to connect to service {service_name}: {e}")
        except Exception as e:
            print(f"Failed to change manipulation mpc control mode: {e}")
        return False

    def srv_change_manipulation_mpc_control_flow(self, ctrl_flow: int)-> bool:
        try:
            service_name = '/enable_mm_wbc_arm_trajectory_control'
            rospy.wait_for_service(service_name, timeout=2.0)
            set_mode_srv = rospy.ServiceProxy(service_name, changeArmCtrlMode)

            req = changeArmCtrlModeRequest()
            req.control_mode = ctrl_flow

            resp = set_mode_srv(req)
            if not resp.result:
                print(f"Failed to change manipulation mpc wbc arm trajectory control to {ctrl_flow}: {resp.message}")
            return resp.result
        except rospy.ServiceException as e:
            print(f"Service call to {service_name} failed: {e}")
        except rospy.ROSException as e:  # For timeout from wait_for_service
            print(f"Failed to connect to service {service_name}: {e}")
        except Exception as e:
            print(f"Failed to change manipulation mpc control flow: {e}")
        return False

    def srv_change_arm_ctrl_mode(self, mode: int)->bool:
        try:
            rospy.wait_for_service('/humanoid_change_arm_ctrl_mode', timeout=2.0)
            change_arm_ctrl_mode_srv = rospy.ServiceProxy('/humanoid_change_arm_ctrl_mode', changeArmCtrlMode)
            req = changeArmCtrlModeRequest()
            req.control_mode = mode
            resp = change_arm_ctrl_mode_srv(req)
            return resp.result
        except rospy.ServiceException as e:
            print(f"Service call failed: {e}")
        except Exception as e:
            print(f"[Error] change arm ctrl mode: {e}")
        return False

    def _arm_reset(self):
        self.srv_change_manipulation_mpc_ctrl_mode(0)
        self.srv_change_manipulation_mpc_control_flow(0)
        self.srv_change_arm_ctrl_mode(1)

    """ Thread Functions """
    def _timeout_check_func(self):
        """检查是否超时并停止执行"""
        start_time = time.time()
        rate = rospy.Rate(20)
        self.is_timeout = False
        self.is_force_killed = False
        while not rospy.is_shutdown():
            rate.sleep()
            with self.running_lock:
                if not self._running:
                    break
                
                elapsed_time = time.time() - start_time
                # 计算进度百分比，最大到 99 %
                progress_percentage = min(99.00, round(float((elapsed_time / self.estimated_duration) * 100), 2))
                
                # 发布进度
                if hasattr(self, 'progress_pub'):
                    progress_msg = Float32()
                    progress_msg.data = progress_percentage
                    self.progress_pub.publish(progress_msg)
                
                if elapsed_time >= self.timeout:
                    with self.process_lock:
                        if self.process and self.process.poll() is None:
                            print(f"System identification timeout after {self.timeout/60:.1f} minutes, stopping...")
                            try:
                                self.process.terminate()
                                time.sleep(2)
                                if self.process.poll() is None:
                                    self.process.kill()
                                print("System identification process stopped due to timeout")
                            except Exception as e:
                                print(f"Error stopping system identification due to timeout: {e}")
                    self._running = False
                    self.is_timeout = True
                    break

    def _system_identification_func(self):
        """在线程中执行系统辨识命令"""
        try:            
            # 获取当前脚本所在目录
            current_dir = os.path.dirname(os.path.abspath(__file__))
            project_dir = os.path.join(current_dir, "..", "..", "..", "..", "..", "..")
            # 构建系统辨识脚本的路径
            script_path = os.path.join(project_dir, "src", "manipulation_nodes", "handcontrollerdemorosnode", "scripts", "system_identification.py")
            setup_bash_file = os.path.join(project_dir, "devel", "setup.bash")
            setup_zsh_file = os.path.join(project_dir, "devel", "setup.zsh")
            if os.path.exists(setup_bash_file):
                setup_file = setup_bash_file
            elif os.path.exists(setup_zsh_file):
                setup_file = setup_zsh_file
            else:   
                # TODO 
                print("Setup file not found")
                return

            # 构建完整的shell命令
            # 获取当前时间戳生成目录
            timestamp = datetime.datetime.now().strftime("%Y-%m-%d_%H%M%S")
            self.output_dir = os.path.expanduser(f"{self.OUTPUT_DIR_PREFIX}/{timestamp}")
            os.makedirs(self.output_dir, exist_ok=True)
            cmd = f"source ~/.bashrc &&source {setup_file} && python3 {script_path} --no-plot --save-data --plots-dir {self.output_dir}"
            
            print(f"Executing command: {cmd}")
            
            # 执行命令，使用shell=True来支持source命令
            with self.process_lock:
                self.process = subprocess.Popen(
                    cmd,
                    stdout=subprocess.PIPE,
                    stderr=subprocess.PIPE,
                    text=True,
                    shell=True,
                    executable='/bin/bash'
                )
            
            # 等待命令完成
            stdout, stderr = self.process.communicate()
            
            # Create log directory
            log_dir = os.path.expanduser(f"~/.ros/stdout/pico-vr/{timestamp}")
            os.makedirs(log_dir, exist_ok=True)
            
            # Write stdout to log file
            if stdout:
                stdout_log_path = os.path.join(log_dir, "system_ident_stdout.log")
                with open(stdout_log_path, 'w') as f:
                    f.write(stdout)
                print(f"Stdout written to: {stdout_log_path}")
            
            # Write stderr to log file
            if stderr:
                stderr_log_path = os.path.join(log_dir, "stderr.log")
                with open(stderr_log_path, 'w') as f:
                    f.write(stderr)
                print(f"Stderr written to: {stderr_log_path}")
            
            if self.process.returncode == 0:
                print("System identification completed successfully")
            else:
                print(f"System identification failed with return code: {self.process.returncode}")
        except Exception as e:
            print(f"Error running system identification: {e}")
        finally:
            with self.running_lock:
                self._running = False
            with self.process_lock:
                returncode = -1
                if self.process:
                   returncode =  self.process.returncode
                self._execute_finished(returncode)
                self.process = None

if __name__ == "__main__":
    rospy.init_node("SystemIdentificationTestNode")
    system_identification = SystemIdentification()
    # system_identification.start()
    while not rospy.is_shutdown():
        time.sleep(1)
    print("System identification finished ...")