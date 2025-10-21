#! /usr/bin/env python3
import os
import time
import rospy
import argparse
import subprocess
from typing import Tuple
from abc import ABC, abstractmethod
from std_msgs.msg import Bool
from std_srvs.srv import Trigger, TriggerRequest,TriggerResponse
from geometry_msgs.msg import Twist

ROS_MASTER_URI = os.getenv("ROS_MASTER_URI")
ROS_IP = os.getenv("ROS_IP")
ROS_HOSTNAME = os.getenv("ROS_HOSTNAME")
KUAVO_ROS_CONTROL_WS_PATH = os.getenv("KUAVO_ROS_CONTROL_WS_PATH")
ROBOT_VERSION = os.getenv('ROBOT_VERSION')
WEBSOCKET_HUMANOID_ROBOT_SESSION_NAME = "websocket_humanoid_robot"

def check_real_kuavo():
    try:
        # optimize: 简单通过检查零点文件来判断是否为实物, 可优化判断条件
        offset_file = os.path.expanduser("~/.config/lejuconfig/offset.csv")
        config_file = os.path.expanduser("~/.config/lejuconfig/config.yaml")
        
        offset_file_exists = os.path.exists(offset_file)    
        config_file_exists = os.path.exists(config_file)
        print(f"offset_file: {offset_file}, exists: {offset_file_exists}")
        print(f"config_file: {config_file}, exists: {config_file_exists}")
        return offset_file_exists and config_file_exists
    except Exception as e:
        return False

def tmux_run_cmd(session_name:str, cmd:str, sudo:bool=False)->Tuple[bool, str]:
    launch_cmd = cmd
        
    print(f"launch_cmd: {launch_cmd}")
    
    try:
        subprocess.run(["tmux", "kill-session", "-t", session_name], 
                        stderr=subprocess.DEVNULL) 
    except Exception as e:
        print(f"Failed to kill session: {e}")
        return False, f"Failed to kill session: {e}"

    print(f"If you want to check the session, please run 'tmux attach -t {session_name}'")
    tmux_cmd = [
        "tmux", "new-session",
        "-s", session_name, 
        "-d",  
        f"source ~/.bashrc && \
            source {KUAVO_ROS_CONTROL_WS_PATH}/devel/setup.bash && \
            export ROS_MASTER_URI={ROS_MASTER_URI} && \
            export ROS_IP={ROS_IP} && \
            export ROS_HOSTNAME={ROS_HOSTNAME} &&\
            export ROBOT_VERSION={ROBOT_VERSION} && \
            {launch_cmd}; exec bash"
    ]
    if sudo:
        tmux_cmd.insert(0, "sudo")
    
    print(f"tmux_cmd: {tmux_cmd}")

    process = subprocess.Popen(
        tmux_cmd,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE
    )

    time.sleep(5.0)

    result = subprocess.run(["tmux", "has-session", "-t", session_name], 
                            capture_output=True)
    ret = False
    if result.returncode == 0:
        ret = True
        msg = f"Started {session_name} in tmux session: {session_name}"
    else:
        msg = f"Failed to start {session_name}"
    return ret, msg

def check_rosnode_exists(node_name:str)->bool:
    try:
        nodes = subprocess.check_output(['rosnode', 'list']).decode('utf-8').split('\n')
        return node_name in nodes
    except subprocess.CalledProcessError as e:
       print(f"Error checking if node {node_name} exists: {e}")
       return False
    except Exception as e:
        print(f"Error checking if node {node_name} exists: {e}")
        return False

class KuavoRobot(ABC):
    def __init__(self):
        pass

    @abstractmethod
    def start_robot(self)->Tuple[bool, str]:
        raise NotImplementedError("start_robot is not implemented")

    @abstractmethod
    def stop_robot(self)->Tuple[bool, str]:
        raise NotImplementedError("stop_robot is not implemented")

    @abstractmethod
    def stand_robot(self)->Tuple[bool, str]:
        raise NotImplementedError("stand_robot is not implemented")

    @abstractmethod
    def get_robot_launch_status(self)->Tuple[bool, str]:
        raise NotImplementedError("get_robot_launch_status is not implemented")

class KuavoRobotReal(KuavoRobot):
    def __init__(self, debug=False):
        super().__init__()
        self.debug = debug
        self.stop_pub = rospy.Publisher('/stop_robot', Bool, queue_size=10)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    def start_robot(self)->Tuple[bool, str]:
        global WEBSOCKET_HUMANOID_ROBOT_SESSION_NAME
        global KUAVO_ROS_CONTROL_WS_PATH
        global ROS_MASTER_URI
        global ROS_IP
        global ROS_HOSTNAME

        if not KUAVO_ROS_CONTROL_WS_PATH:
            KUAVO_ROS_CONTROL_WS_PATH = os.path.abspath(os.path.join(os.path.dirname(__file__), "../../../../.."))
        
        if self.debug:
            launch_cmd = "roslaunch humanoid_controllers load_kuavo_real.launch joystick_type:=bt2pro"
            if not ROS_MASTER_URI or not ROS_IP or not ROS_HOSTNAME:
                ROS_MASTER_URI = "http://kuavo_master:11311"
                ROS_IP = "kuavo_master" 
                ROS_HOSTNAME = "kuavo_master"
        else:
            launch_cmd = "roslaunch humanoid_controllers load_kuavo_real.launch joystick_type:=h12 start_way:=auto"

        return tmux_run_cmd(WEBSOCKET_HUMANOID_ROBOT_SESSION_NAME, launch_cmd, sudo=True)
    
    def stop_robot(self)->Tuple[bool, str]:
        try:
            success, status = self.get_robot_launch_status()
            # 已经启动则下蹲再停止
            if success and status == "launched":
                msg = Twist()
                msg.linear.x = 0.0
                msg.linear.y = 0.0
                msg.angular.z = 0.0
                msg.linear.z = -0.05
                for i in range(10):
                    self.cmd_vel_pub.publish(msg)
                    time.sleep(0.1)
            self.stop_pub.publish(True)
            self.stop_pub.publish(True)
            self.stop_pub.publish(True)
            return True, "success"
        except Exception as e:
            print(f"Failed to stop robot: {e}")
            return False, f"Failed to stop robot: {e}"
    def stand_robot(self)->Tuple[bool, str]:
        try:
            client = rospy.ServiceProxy('/humanoid_controller/real_initial_start', Trigger)
            req = TriggerRequest()
            client.wait_for_service(timeout=2.0)
            # Call the service
            if client.call(req):
                print(f"RealInitializeSrv service call successful")
                return True, "Success"
            else:
                print(f"Failed to callRealInitializeSrv service")
                return False, "Failed to callRealInitializeSrv service"
        except rospy.ServiceException as e:
            print(f"Service call failed: {e}")
            return False, f"Service call failed: {e}" 

    def get_robot_launch_status(self)->Tuple[bool, str]:
        client = rospy.ServiceProxy('/humanoid_controller/real_launch_status', Trigger)
        try:
            client.wait_for_service(timeout=2.0)
        except rospy.ROSException as e:
            # 等待服务超时（服务不存在）
            print(f"Service does not exist: {e}")
            return True, "unknown"  # 关键修改：服务不存在时返回(True, "unknown")
    
        try:
            req = TriggerRequest()
            client.wait_for_service(timeout=1.5)
            # Call the service
            response = client.call(req)
            if response.success:
                print(f"RealInitializeSrv service call successful")
                return True, response.message
            else:
                print(f"Failed to callRealInitializeSrv service")
                return False, "unknown"    
        except rospy.ServiceException as e:
            print(f"Service call failed: {e}")
            return False, f"unknown" 

class KuavoRobotSim(KuavoRobot):
    def __init__(self, debug=False):
        super().__init__()
        self.debug = debug
        self.stop_pub = rospy.Publisher('/stop_robot', Bool, queue_size=10)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    def start_robot(self)->Tuple[bool, str]:
        global WEBSOCKET_HUMANOID_ROBOT_SESSION_NAME
        global KUAVO_ROS_CONTROL_WS_PATH
        global ROS_MASTER_URI
        global ROS_IP
        global ROS_HOSTNAME

        if not KUAVO_ROS_CONTROL_WS_PATH:
            KUAVO_ROS_CONTROL_WS_PATH = os.path.abspath(os.path.join(os.path.dirname(__file__), "../../../../.."))
        if not ROS_MASTER_URI or not ROS_IP or not ROS_HOSTNAME:
            ROS_MASTER_URI = "http://localhost:11311"
            ROS_IP = "localhost" 
            ROS_HOSTNAME = "localhost"

        launch_cmd = "roslaunch humanoid_controllers load_kuavo_mujoco_sim.launch && export DISPLAY=:1"
        return tmux_run_cmd(WEBSOCKET_HUMANOID_ROBOT_SESSION_NAME, launch_cmd, sudo=False)
    
    def stop_robot(self)->Tuple[bool, str]:
        try:
            msg = Twist()
            msg.linear.x = 0.0
            msg.linear.y = 0.0
            msg.angular.z = 0.0
            msg.linear.z = -0.05
            for i in range(50):
                self.cmd_vel_pub.publish(msg)
                time.sleep(0.1)
            self.stop_pub.publish(True)
            self.stop_pub.publish(True)
            self.stop_pub.publish(True)
            return True, "success"
        except Exception as e:
            print(f"Failed to stop robot: {e}")
            return False, f"Failed to stop robot: {e}"
    
    def stand_robot(self)->Tuple[bool, str]:
        return self.get_robot_launch_status()
    
    def get_robot_launch_status(self)->Tuple[bool, str]:

        if check_rosnode_exists("/humanoid_sqp_mpc") and check_rosnode_exists("/nodelet_controller"):
            return True, "launched"
        else:
            return True, "unlaunch"

class WebSocketSdkStartNode:
    """
        Kuavo Humanoid Websocket SDK 服务节点，为 SDK 提供启动，站立和启动状态等接口
    """
    def __init__(self, debug=False):
        # 检查实物机器人或仿真环境
        if check_real_kuavo():
            self.robot = KuavoRobotReal(debug)
        else:
            self.robot = KuavoRobotSim(debug)
        # 启动机器人
        self.start_robot_service = rospy.Service(
            '/websocket_sdk_srv/start_robot', Trigger, self._start_robot_callback
        )
        # 停止机器人
        self.stop_robot_service = rospy.Service(
            '/websocket_sdk_srv/stop_robot', Trigger, self._stop_robot_callback
        )
        # 站立机器人
        self.stop_robot_service = rospy.Service(
            '/websocket_sdk_srv/stand_robot', Trigger, self._stand_robot_callback
        )
        # 获取机器人启动状态: unlaunch, initing, ready_to_squat, ready_to_stand, launched ...
        self.get_robot_launch_status_service = rospy.Service(
            '/websocket_sdk_srv/get_robot_launch_status', Trigger, self._get_robot_launch_status_callback
        )

    def _get_robot_launch_status_callback(self, req):
        result, status = self.robot.get_robot_launch_status()
        print(f"Get robot launch status: {status}")
        if not result:
            print(f"Get robot launch status failed: {status}")
            status = "unknown"
        return TriggerResponse(success=result, message=status)

    def _stand_robot_callback(self, req):
        result, msg = self.robot.stand_robot()
        if not result:
            print(f"Stand robot failed: {msg}")
        return TriggerResponse(success=result, message=msg)

    def _start_robot_callback(self, req):
        result, msg = self.robot.start_robot()
        if not result:
            print(f"Start robot success: {msg}")
        return TriggerResponse(success=result, message=msg)

    def _stop_robot_callback(self, req):
        result, msg = self.robot.stop_robot()
        if not result:
            print(f"Stop robot failed: {msg}")
        return TriggerResponse(success=result, message=msg)

def main():
    parser = argparse.ArgumentParser(description='WebSocket SDK Start Node')
    parser.add_argument('--debug', type=bool, default=False, help='Enable debug mode')
    # Add this line to ignore unknown arguments from roslaunch
    args, unknown = parser.parse_known_args()

    rospy.init_node("websocket_sdk_start_node")
    print(f"Debug mode: {args.debug}")
    WebSocketSdkStartNode(args.debug)
    rospy.spin()

if __name__ == "__main__":
    main()