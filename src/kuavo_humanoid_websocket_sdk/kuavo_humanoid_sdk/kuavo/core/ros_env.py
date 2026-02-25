#!/usr/bin/env python3
# coding: utf-8

import os
import subprocess
import atexit
from kuavo_humanoid_sdk.common.logger import SDKLogger
from kuavo_humanoid_sdk.common.websocket_kuavo_sdk import WebSocketKuavoSDK
import roslibpy


class KuavoROSEnvWebsocket:
    _instance = None
    _processes = []  # Store all subprocess instances

    def __new__(cls):
        if cls._instance is None:
            cls._instance = super(KuavoROSEnvWebsocket, cls).__new__(cls)
            cls._instance._initialized = False
            cls._instance._init_success = False  # Add flag to track successful Init() call
            # Register cleanup handler
            atexit.register(cls._cleanup_processes)
        return cls._instance

    def __init__(self):
        if not self._initialized:
            self._initialized = True
            self.websocket = None

    @classmethod
    def _cleanup_processes(cls):
        """Cleanup all registered processes on exit"""
        for process in cls._processes:
            if process.poll() is None:  # Process is still running
                process.terminate()
                try:
                    process.wait(timeout=3)  # Wait for process to terminate
                except subprocess.TimeoutExpired:
                    process.kill()  # Force kill if not terminated
        cls._processes.clear()

    def _get_kuavo_ws_root(self) -> str:
        # For WebSocket version, we'll use environment variable instead of ROS param
        model_path = os.environ.get('KUAVO_MODEL_PATH')
        if not model_path:
            raise Exception("KUAVO_MODEL_PATH environment variable not found")
        
        if not os.path.exists(model_path):
            raise Exception(f"Model path {model_path} not found")
            
        # ws 
        return model_path.replace('/src/kuavo_assets/models', '')
    
    def Init(self) -> bool:
        """
        Initialize the WebSocket environment.
        Raises:
            Exception: If the KUAVO_MODEL_PATH environment variable is not found or the path is not valid.
        """
        # if generate docs, skip init.
        if 'GEN_KUAVO_HUMANOID_SDK_DOCS' in os.environ:
            return True

        # Return directly if already initialized successfully
        if self._init_success:
            return True
        
        # Check if WebSocket server is running
        try:
            
            self.websocket = WebSocketKuavoSDK()
            if not self.websocket.client.is_connected:
                print(f"\033[31m\nError: Can't connect to WebSocket server. Please ensure the server is running.\033[0m"
                      "\nMaybe manually launch the app first?"
                      "\n - for example(sim): roslaunch humanoid_controller load_kuavo_mujoco_sim.launch, "
                      "\n - for example(real): roslaunch humanoid_controller load_kuavo_real.launch\n")
                exit(1)
        except Exception as e:
            print(f"\033[31m\nError: Failed to connect to WebSocket server: {e}\033[0m")
            exit(1)
        
        # Only check nodes exist when Init SDK, if not, tips user manually launch nodes.
        deps_nodes = ['/humanoid_gait_switch_by_name']
        for node in deps_nodes:
            if not self.check_rosnode_exists(node):
                print(f"\033[31m\nError: Node {node} not found. Please launch it manually.\033[0m")
                exit(1)
                
        self._init_success = True  # Set flag after successful initialization

        return True
    
    def _launch_ros_node(self, node_name, launch_cmd, log_name):
        """Launch a ROS node with the given command and log the output.
        
        Args:
            node_name (str): Name of the node to launch
            launch_cmd (str): Full launch command including source and roslaunch
            log_name (str): Name for the log file
            
        Raises:
            Exception: If node launch fails
        """
        # Launch in background and check if successful
        try:
            os.makedirs('/var/log/kuavo_humanoid_sdk', exist_ok=True)
            log_path = f'/var/log/kuavo_humanoid_sdk/{log_name}.log'
        except PermissionError:
            os.makedirs('log/kuavo_humanoid_sdk', exist_ok=True)
            log_path = f'log/kuavo_humanoid_sdk/{log_name}.log'
            
        with open(log_path, 'w') as log_file:
            process = subprocess.Popen(launch_cmd, shell=True, executable='/bin/bash', stdout=log_file, stderr=log_file)
            self._processes.append(process)  # Add process to tracking list
            
        if process.returncode is not None and process.returncode != 0:
            raise Exception(f"Failed to launch {node_name}, return code: {process.returncode}")

        SDKLogger.info(f"{node_name} launched successfully")

    def _get_setup_file(self, ws_root=None):
        """Get the appropriate ROS setup file path based on shell type.
        
        Args:
            ws_root (str, optional): ROS workspace root path. If None, uses ROS_WORKSPACE.
            
        Returns:
            str: Path to the setup file
            
        Raises:
            Exception: If setup file not found
        """
        is_zsh = 'zsh' in os.environ.get('SHELL', '')
        
        if ws_root is None:
            ws_root = os.environ['ROS_WORKSPACE']
            
        setup_files = {
            'zsh': os.path.join(ws_root, 'devel/setup.zsh'),
            'bash': os.path.join(ws_root, 'devel/setup.bash')
        }
        
        setup_file = setup_files['zsh'] if is_zsh else setup_files['bash']
        if not os.path.exists(setup_file):
            setup_file = setup_file.replace('devel', 'install')
            if not os.path.exists(setup_file):
                raise Exception(f"Setup file not found in either devel or install: {setup_file}")
                
        return setup_file

    def launch_ik_node(self):
        # nodes: /arms_ik_node
        # services: /ik/two_arm_hand_pose_cmd_srv, /ik/fk_srv
        try:
            if not self.websocket or not self.websocket.client.is_connected:
                raise Exception("WebSocket server is not running")
        except Exception as e:
            raise Exception(f"WebSocket server is not running: {e}")

        # Check if IK node and services exist
        try:
            # Check if arms_ik_node is running using roslibpy
            service = roslibpy.Service(self.websocket.client, '/rosnode/list', 'rosapi/Nodes')
            response = service.call({})
            nodes = response.get('nodes', [])
            
            if '/arms_ik_node' not in nodes:
                # Launch IK node if not running
                kuavo_ws_root = self._get_kuavo_ws_root()
                setup_file = self._get_setup_file(kuavo_ws_root)
                source_cmd = f"source {setup_file}"
                
                # Get robot version from environment variable
                robot_version = os.environ.get('ROBOT_VERSION')
                if robot_version is None:
                    raise Exception("Failed to get ROBOT_VERSION from environment variables")
                
                # Launch IK node
                launch_cmd = f"roslaunch motion_capture_ik ik_node.launch robot_version:={robot_version}"
                full_cmd = f"{source_cmd} && {launch_cmd}"
                
                self._launch_ros_node('IK node', full_cmd, 'launch_ik')

            return True

        except Exception as e:
            raise Exception(f"Failed to verify IK node and services: {e}")
        

    def launch_gait_switch_node(self)-> bool:
        """Verify that the gait switch node is running, launch if not."""
        try:
            # Check if node exists using roslibpy
            service = roslibpy.Service(self.websocket.client, '/rosnode/list', 'rosapi/Nodes')
            response = service.call({})
            nodes = response.get('nodes', [])
            
            if '/humanoid_gait_switch_by_name' not in nodes:
                kuavo_ws_root = self._get_kuavo_ws_root()
                setup_file = self._get_setup_file(kuavo_ws_root)
                source_cmd = f"source {setup_file}"
    
                # Launch gait switch node
                launch_cmd = "roslaunch humanoid_interface_ros humanoid_switch_gait.launch"
                full_cmd = f"{source_cmd} && {launch_cmd}"
                
                self._launch_ros_node('Gait switch node', full_cmd, 'launch_gait_switch')

        except Exception as e:
            raise Exception(f"Failed to launch gait_switch node: {e}")
        
    @staticmethod
    def check_rosnode_exists(node_name):
        """Check if a ROS node is running using roslibpy.
        
        Args:
            node_name (str): Name of the node to check
            
        Returns:
            bool: True if node is running, False otherwise
        """
        websocket = WebSocketKuavoSDK()
        try:
            if not websocket or not websocket.client.is_connected:
                return False
                
            service = roslibpy.Service(websocket.client, '/rosapi/nodes', 'rosapi/Nodes')
            response = service.call({})
            nodes = response.get('nodes', [])
            return node_name in nodes
        except Exception as e:
            SDKLogger.error(f"Error checking if node {node_name} exists: {e}")
            return False
