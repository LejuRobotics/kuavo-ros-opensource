#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import subprocess
import rospy
import sys
import os
import time
import signal
import argparse
from kuavo_msgs.srv import CreatePath, CreatePathRequest
from rosmsg_dict_converter.converter import a_dict_to_ros_message
from std_msgs.msg import Bool, String

def parse_args():
    parser = argparse.ArgumentParser(description="Trace path demo")
    parser.add_argument("--path_type", choices=["circle", "square", "line", "triangle", "scurve"], type=str, default="scurve",
                       help="Path type for custom demo")
    parser.add_argument("--v_max_linear", type=float, default=0.2, help="Maximum linear velocity")
    parser.add_argument("--length", type=float, default=4.0, help="Length parameter")
    parser.add_argument("--radius", type=float, default=2.0, help="Radius for circle")
    parser.add_argument("--amplitude", type=float, default=0.5, help="Amplitude for S-curve")
    parser.add_argument("--half_scurve", type=bool, default=False, help="Half S-curve")
    return parser.parse_args()

def call_create_path_service(path_data):
  """Call the create_path service to generate a path"""
  try:
      rospy.wait_for_service('/mpc_path_tracer_node/create_path', timeout=1.0)
      create_path_service = rospy.ServiceProxy('/mpc_path_tracer_node/create_path', CreatePath)
      
      # Create request object
      req = CreatePathRequest()
      req = a_dict_to_ros_message(req, path_data)
      
      response = create_path_service(req)
      return response.success
  except rospy.ServiceException as e:
      rospy.logerr(f"Service call failed: {e}")
      return False
  except Exception as e:
      rospy.logerr(f"Error calling create_path service: {e}")
      return False

def call_start_path_service():
    """Call the start service to begin path following"""
    try:
        # Start service is a topic publisher, not a service
        start_pub = rospy.Publisher('/mpc_path_tracer_node/start', Bool, queue_size=10)
        rospy.sleep(0.1)  # Wait for publisher to connect
        
        start_msg = Bool()
        start_msg.data = True
        start_pub.publish(start_msg)
        return True, "Path following started"
    except Exception as e:
        rospy.logerr(f"Error starting path: {e}")
        return False, f"Error: {e}"

mpc_path_tracer_process = None

def switch_to_pose(pose_name):
  """切换到步态，支持 'walk' 和 'stance'"""
  try:
      # 验证步态名称
      if pose_name not in ['walk', 'stance']:
          rospy.logwarn(f"不支持的步态名称: {pose_name}，仅支持 'walk' 和 'stance'")
          return False
      
      # 创建话题发布者
      gait_pub = rospy.Publisher('/humanoid_mpc_gait_change', String, queue_size=10)
      rospy.sleep(0.1)  # 等待发布者连接
      
      # 发布步态切换消息
      gait_msg = String()
      gait_msg.data = pose_name
      gait_pub.publish(gait_msg)
      
      # rospy.loginfo(f"成功发布步态切换指令: {pose_name}")
      return True
  except Exception as e:
      rospy.logerr(f"切换到步态时出错: {e}")
      return False

def signal_handler(signum, frame):
  rospy.loginfo("Stopping trace path demo...")
  if mpc_path_tracer_process:
    os.kill(mpc_path_tracer_process.pid, signal.SIGTERM)

  try:
      # rospy.loginfo("切换到姿态以确保双脚对齐...")
      switch_to_pose("walk")
      rospy.sleep(2)
      switch_to_pose("stance")

      subprocess.run([
          "rostopic", "pub", "--once", "/cmd_vel", "geometry_msgs/Twist",
          "linear:\n  x: 0.0\n  y: 0.0\n  z: 0.0\nangular:\n  x: 0.0\n  y: 0.0\n  z: 0.0"
      ], text=True, check=True, capture_output=True)
      rospy.loginfo("Published zero velocity to /cmd_vel")
  except Exception as e:
      rospy.logerr(f"Failed to publish zero velocity to /cmd_vel: {e}")

  sys.exit(0)

def start_trace_path_demo(args):
  rospy.init_node("trace_path_demo")
  rospy.loginfo("Starting trace path demo...")
  global mpc_path_tracer_process

  # 复制当前环境变量并添加drake路径
  env_new = os.environ.copy()
  
  # 设置PYTHONPATH以包含drake库
  python_version = f"{sys.version_info.major}.{sys.version_info.minor}"
  drake_python_path = f"/opt/drake/lib/python{python_version}/site-packages"
  
  if "PYTHONPATH" in env_new:
      env_new["PYTHONPATH"] = f"{drake_python_path}:{env_new['PYTHONPATH']}"
  else:
      env_new["PYTHONPATH"] = drake_python_path

  rospy.logdebug(f"Environment PYTHONPATH: {env_new.get('PYTHONPATH', '')}")

  cmd = ['rosrun', 'trace_path', 'mpc_path_tracer.py']
  mpc_path_tracer_process = subprocess.Popen(cmd, env=env_new, stdout=subprocess.PIPE, stderr=subprocess.PIPE)

  rospy.loginfo(f"Started mpc_path_tracer process with PID: {mpc_path_tracer_process.pid}")


  try:
    rospy.wait_for_service('/mpc_path_tracer_node/create_path', timeout=10)
  except rospy.ROSException as e:
    rospy.logerr(f"Service not available: {e}")
    # 输出子进程的错误信息帮助调试
    stdout, stderr = mpc_path_tracer_process.communicate(timeout=1)
    rospy.logerr(f"mpc_path_tracer process stdout: {stdout.decode()}")
    rospy.logerr(f"mpc_path_tracer process stderr: {stderr.decode()}")
    return False
  
  rospy.logdebug("mpc_path_tracer_node is available")
  path_data = {
      "path_type": args.path_type,
      "v_max_linear": args.v_max_linear,
      "length": args.length,
      "radius": args.radius,
      "amplitude": args.amplitude,
      "half_scurve": args.half_scurve
  }

  if call_create_path_service(path_data):
    print("Path created successfully")

    call_start_path_service()
  else:
    rospy.logerr("Failed to create path")
    return False
  

  while True:
    try:
      time.sleep(1)
    except KeyboardInterrupt:
      rospy.loginfo("Stopping trace path demo...")
      # 停止时也切换到站立姿态
      if mpc_path_tracer_process:
        os.kill(mpc_path_tracer_process.pid, signal.SIGTERM)
      
      try:
          # rospy.loginfo("切换到姿态以确保双脚对齐...")
          switch_to_pose("walk")
          rospy.sleep(2)
          switch_to_pose("stance")

          subprocess.run([
              "rostopic", "pub", "--once", "/cmd_vel", "geometry_msgs/Twist",
              "linear:\n  x: 0.0\n  y: 0.0\n  z: 0.0\nangular:\n  x: 0.0\n  y: 0.0\n  z: 0.0"
          ], text=True, check=True, capture_output=True)
          rospy.loginfo("Published zero velocity to /cmd_vel")
      except Exception as e:
          rospy.logerr(f"Failed to publish zero velocity to /cmd_vel: {e}")
      
      break


if __name__ == "__main__":
  args = parse_args()
  signal.signal(signal.SIGINT, signal_handler)
  signal.signal(signal.SIGTERM, signal_handler)
  start_trace_path_demo(args)
