#!/usr/bin/env python3

import subprocess
import rospy
import sys
import os
import time
import signal
import argparse
from kuavo_msgs.srv import CreatePath, CreatePathRequest
from rosmsg_dict_converter.converter import a_dict_to_ros_message
from std_msgs.msg import Bool

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

def signal_handler(signum, frame):
  rospy.loginfo("Stopping trace path demo...")
  if mpc_path_tracer_process:
    os.kill(mpc_path_tracer_process.pid, signal.SIGTERM)
  sys.exit(0)

def start_trace_path_demo(args):
  rospy.loginfo("Starting trace path demo...")
  global mpc_path_tracer_process
  cmd = ['rosrun', 'trace_path', 'mpc_path_tracer.py']
  mpc_path_tracer_process = subprocess.Popen(cmd,env=os.environ,stdout=subprocess.PIPE,stderr=subprocess.PIPE)

  rospy.init_node("trace_path_demo")
  try:
    rospy.wait_for_service('/mpc_path_tracer_node/create_path', timeout=10)
  except rospy.ROSException as e:
    rospy.logerr(f"Service not available: {e}")
    return False
  
  rospy.loginfo("mpc_path_tracer_node is available")
  path_data = {
      "path_type": args.path_type,
      "v_max_linear": args.v_max_linear,
      "length": args.length,
      "radius": args.radius,
      "amplitude": args.amplitude,
      "half_scurve": args.half_scurve
  }

  if call_create_path_service(path_data):
    call_start_path_service()
  else:
    rospy.logerr("Failed to create path")
    return False
  

  while True:
    try:
      time.sleep(1)
    except KeyboardInterrupt:
      rospy.loginfo("Stopping trace path demo...")
  

if __name__ == "__main__":
  args = parse_args()
  signal.signal(signal.SIGINT, signal_handler)
  signal.signal(signal.SIGTERM, signal_handler)
  start_trace_path_demo(args)
