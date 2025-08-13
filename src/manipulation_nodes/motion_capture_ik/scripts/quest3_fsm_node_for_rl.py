#!/usr/bin/env python3

import rospy
from kuavo_msgs.msg import JoySticks
from std_srvs.srv import SetBool, SetBoolRequest
from kuavo_msgs.srv import changeTorsoCtrlMode, changeTorsoCtrlModeRequest
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
import numpy as np
import os
import re
from typing import Tuple, Optional

class Quest3FSMNode:
    def __init__(self):
        """
        Initialize Quest3 FSM node for reinforcement learning control
        """
        # Initialize ROS node
        rospy.init_node('quest3_fsm_node_for_rl', anonymous=True)
        
        # Initialize joystick data structures
        self.joystick_data = JoySticks()
        self.joystick_data_prev = JoySticks()
        
        # Node state management
        self.node_active = True
        self.first_data_received = False
        
        # Finite State Machine (FSM) implementation
        self.current_state = "stand"  # Default state is stand
        self.previous_state = "stand"
        
        # Joystick control settings
        self.joystick_deadzone = 0.05  # Deadzone threshold for joystick input detection
        
        # Create subscriber for joystick data
        self.joystick_sub = rospy.Subscriber(
            "/quest_joystick_data", 
            JoySticks, 
            self.joystick_callback, 
            queue_size=1
        )
        
        # Create RL control service client
        self.walkenable_service_name = "/humanoid_controller/walkenable"
        self.walkenable_client = rospy.ServiceProxy(self.walkenable_service_name, SetBool)
        
        # Create mobile manipulator MPC control service client
        self.mpc_control_service_name = "/mobile_manipulator_mpc_control"
        self.mpc_control_client = rospy.ServiceProxy(self.mpc_control_service_name, changeTorsoCtrlMode)
        
        # Create velocity command publisher
        self.cmd_vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        
        # Create stop robot publisher
        self.stop_pub = rospy.Publisher("/stop_robot", Bool, queue_size=10)
        
        # 从 RL 仓库的配置文件中读取
        self.max_linear_x, self.max_linear_y, self.max_linear_z, self.max_angular_z = self.set_velocity_limits()
        rospy.loginfo(f"Velocity limits: x={self.max_linear_x}, y={self.max_linear_y}, z={self.max_linear_z}, yaw={self.max_angular_z}")
        
        rospy.loginfo("Quest3 FSM Node for RL initialized")
        rospy.loginfo("Subscribing to /quest_joystick_data")
        rospy.loginfo("Publishing to /cmd_vel")
        rospy.loginfo(f"Initial FSM state: {self.current_state}")
        rospy.loginfo("Control modes:")
        rospy.loginfo("  - Button mappings: Right Button 1 -> STAND, Right Button 2 -> WALK")
        rospy.loginfo(f"  - Joystick auto control: deadzone = {self.joystick_deadzone}")
        rospy.loginfo("  - Push joystick -> WALK state + walkenable=true + mpc_control_mode=0")
        rospy.loginfo("  - Release joystick -> STAND state + walkenable=false")
        rospy.loginfo("  - Press A button (Right Button 1) -> STAND state + mpc_control_mode=1")
        
        # Initialize RL control service to disabled state - wait until service is available
        rospy.loginfo("Initializing walkenable service to disabled state...")
        self.call_walkenable_service(False, is_initialization=True)
        
    def set_velocity_limits(self, default_limits: Tuple[float, float, float, float] = (2.0, 0.7, 0.3, 0.6)) -> Tuple[float, float, float, float]:
        kuavo_rl_ws_path = os.environ.get('KUAVO_RL_WS_PATH', "/home/lab/kuavo-RL/kuavo-robot-deploy")
        robot_version = rospy.get_param("/robot_version", 46)
        if robot_version != 46:
            rospy.logwarn(f"当前机器人版本{robot_version}不支持设置速度限制，使用默认值")
            return default_limits

        config_path = os.path.join(
            kuavo_rl_ws_path,
            f"src/humanoid-control/humanoid_controllers/config/kuavo_v{str(robot_version)}/rl/skw_rl_param.info"
        )
        # 检查文件是否存在
        if not os.path.exists(config_path):
            rospy.logwarn(f"配置文件 {config_path} 不存在，使用默认值")
            return default_limits
        
        try:
            with open(config_path, 'r', encoding='utf-8') as f:
                content = f.read()
            
            # 查找velocityLimits块
            pattern = r'velocityLimits\s*\{([^}]+)\}'
            match = re.search(pattern, content, re.DOTALL)
            
            if not match:
                rospy.logwarn(f"在文件 {config_path} 中未找到velocityLimits配置块，使用默认值")
                return default_limits
            
            velocity_block = match.group(1)
            
            # 匹配 (i,j) value 格式的参数
            param_pattern = r'\((\d+),(\d+)\)\s+([0-9.]+)'
            matches = re.findall(param_pattern, velocity_block)
            
            if not matches:
                rospy.logwarn(f"在velocityLimits块中未找到有效参数，使用默认值")
                return default_limits
            
            # 按索引排序并提取值
            params = [(int(i), int(j), float(value)) for i, j, value in matches]
            params.sort(key=lambda x: (x[0], x[1]))  # 按(i,j)排序
            
            # 创建结果数组，初始化为默认值
            result = list(default_limits)
            
            # 填充找到的参数
            for i, j, value in params:
                if i < len(result):
                    result[i] = value
                else:
                    rospy.logwarn(f"索引 ({i},{j}) 超出范围，忽略该参数")
            
            # 检查是否有缺失的参数
            missing_count = 0
            for i, (found_val, default_val) in enumerate(zip(result, default_limits)):
                if found_val == default_val:
                    # 检查是否真的找到了这个索引的参数
                    found_in_file = any(idx == i for idx, _, _ in params)
                    if not found_in_file:
                        missing_count += 1
                        param_names = ['cmdVelLineX', 'cmdVelLineY', 'cmdVelLineZ', 'cmdVelAngularZ']
                        rospy.logwarn(f"参数 {param_names[i]} 缺失，使用默认值 {default_val}")
            
            if missing_count > 0:
                rospy.loginfo(f"共缺失 {missing_count} 个参数，已使用默认值补充")
            
            return tuple(result)
            
        except Exception as e:
            rospy.logerr(f"解析配置文件 {config_path} 时发生异常: {e}")
            rospy.logwarn("使用默认值")
            return default_limits

    def joystick_callback(self, msg):
        """
        Joystick data callback function for real-time input processing
        
        Args:
            msg (JoySticks): Incoming joystick data message
        """
        # Store previous frame data for edge detection
        if self.first_data_received:
            self.joystick_data_prev = self.joystick_data
        
        # Update current data buffer
        self.joystick_data = msg
        
        if not self.first_data_received:
            self.first_data_received = True
            rospy.loginfo("First joystick data received!")
            
        # Process incoming joystick data
        self.process_joystick_data()
        
    def process_joystick_data(self):
        """
        Main logic for processing joystick data and extracting control signals
        """
        # Extract joystick axis data
        left_stick = {
            'x': self.joystick_data.left_x,
            'y': self.joystick_data.left_y
        }
        
        right_stick = {
            'x': self.joystick_data.right_x,
            'y': self.joystick_data.right_y
        }
        
        # Extract trigger and grip sensor data
        triggers = {
            'left_trigger': self.joystick_data.left_trigger,
            'right_trigger': self.joystick_data.right_trigger,
            'left_grip': self.joystick_data.left_grip,
            'right_grip': self.joystick_data.right_grip
        }
        
        # Extract button state information
        buttons = {
            'left_first_pressed': self.joystick_data.left_first_button_pressed,
            'left_second_pressed': self.joystick_data.left_second_button_pressed,
            'left_first_touched': self.joystick_data.left_first_button_touched,
            'left_second_touched': self.joystick_data.left_second_button_touched,
            'right_first_pressed': self.joystick_data.right_first_button_pressed,
            'right_second_pressed': self.joystick_data.right_second_button_pressed,
            'right_first_touched': self.joystick_data.right_first_button_touched,
            'right_second_touched': self.joystick_data.right_second_button_touched
        }
        
        # Detect button state changes using edge detection
        button_changes = self.detect_button_changes()
        
        # Check if both left joystick buttons are pressed simultaneously
        self.check_stop_condition(buttons)
        
        # Check joystick movement and automatically manage walkenable service
        self.handle_joystick_walkenable(left_stick, right_stick)
        
        # Update finite state machine based on input
        self.update_fsm_state(left_stick, right_stick, triggers, buttons, button_changes)
        
    def detect_button_changes(self):
        """
        Detect button state transitions using edge detection algorithm
        
        Returns:
            dict: Dictionary containing button state change information
        """
        if not self.first_data_received:
            return {}
            
        changes = {}
        
        # Detect button press events (transition from not pressed to pressed)
        changes['left_first_pressed_edge'] = (
            not self.joystick_data_prev.left_first_button_pressed and 
            self.joystick_data.left_first_button_pressed
        )
        changes['left_second_pressed_edge'] = (
            not self.joystick_data_prev.left_second_button_pressed and 
            self.joystick_data.left_second_button_pressed
        )
        changes['right_first_pressed_edge'] = (
            not self.joystick_data_prev.right_first_button_pressed and 
            self.joystick_data.right_first_button_pressed
        )
        changes['right_second_pressed_edge'] = (
            not self.joystick_data_prev.right_second_button_pressed and 
            self.joystick_data.right_second_button_pressed
        )
        
        # Detect button release events (transition from pressed to not pressed)
        changes['left_first_released_edge'] = (
            self.joystick_data_prev.left_first_button_pressed and 
            not self.joystick_data.left_first_button_pressed
        )
        changes['left_second_released_edge'] = (
            self.joystick_data_prev.left_second_button_pressed and 
            not self.joystick_data.left_second_button_pressed
        )
        changes['right_first_released_edge'] = (
            self.joystick_data_prev.right_first_button_pressed and 
            not self.joystick_data.right_first_button_pressed
        )
        changes['right_second_released_edge'] = (
            self.joystick_data_prev.right_second_button_pressed and 
            not self.joystick_data.right_second_button_pressed
        )
        
        return changes
        
    def check_stop_condition(self, buttons):
        """
        Check if both left joystick buttons are pressed simultaneously and publish stop signal
        
        Args:
            buttons (dict): Button state information
        """
        # Check if both left buttons are pressed simultaneously
        if (buttons['left_first_pressed'] and buttons['left_second_pressed']):
            stop_msg = Bool()
            stop_msg.data = True
            self.stop_pub.publish(stop_msg)
            rospy.loginfo("Both left buttons pressed - sending stop signal")
        
    def handle_joystick_walkenable(self, left_stick, right_stick):
        """
        Automatically control state machine and walkenable service based on joystick movement
        
        Args:
            left_stick (dict): Left joystick axis data
            right_stick (dict): Right joystick axis data
        """
        # Calculate joystick magnitude for movement detection
        left_magnitude = np.sqrt(left_stick['x']**2 + left_stick['y']**2)
        right_magnitude = np.sqrt(right_stick['x']**2 + right_stick['y']**2)
        
        # Determine if any joystick is being actively used
        joystick_active = (left_magnitude > self.joystick_deadzone or 
                          right_magnitude > self.joystick_deadzone)
        
        # Control state machine based on joystick activity
        if joystick_active and self.current_state != "walk":
            # Joystick pushed - transition to walk state
            rospy.loginfo(f"Joystick pushed - transitioning to WALK state")
            self.previous_state = self.current_state
            self.current_state = "walk"
            
            # Call mobile manipulator MPC control service (control_mode: 0)
            self.call_mpc_control_service(0, is_initialization=False)
            
            # Call walkenable service and update state
            self.call_walkenable_service(True, is_initialization=False)
            rospy.loginfo(f"State changed: {self.previous_state} -> {self.current_state}")
            
        elif not joystick_active and self.current_state != "stand":
            # Joystick released - transition to stand state
            rospy.loginfo(f"Joystick released - transitioning to STAND state")
            self.previous_state = self.current_state
            self.current_state = "stand"
            
            # Call walkenable service and update state
            self.call_walkenable_service(False, is_initialization=False)
            rospy.loginfo(f"State changed: {self.previous_state} -> {self.current_state}")
            
            # Immediately publish zero velocity when transitioning to stand
            cmd_vel = Twist()
            self.cmd_vel_pub.publish(cmd_vel)
        
    def update_fsm_state(self, left_stick, right_stick, triggers, buttons, button_changes):
        """
        Update finite state machine state based on input conditions
        
        Args:
            left_stick (dict): Left joystick axis data
            right_stick (dict): Right joystick axis data  
            triggers (dict): Trigger and grip sensor data
            buttons (dict): Current button states
            button_changes (dict): Button state transition events
        """
        # Store previous state for transition detection
        self.previous_state = self.current_state
        
        # State transition logic implementation
        if button_changes.get('right_first_pressed_edge', False):
            # Right controller first button pressed - transition to stand state
            self.current_state = "stand"
            
            # Call mobile manipulator MPC control service (control_mode: 1) when A button pressed
            self.call_mpc_control_service(1, is_initialization=False)
            
        elif button_changes.get('right_second_pressed_edge', False):
            # Right controller second button pressed - transition to walk state
            self.current_state = "walk"
        
        # Detect state changes and log transitions
        if self.current_state != self.previous_state:
            rospy.loginfo(f"State changed: {self.previous_state} -> {self.current_state}")
            self.on_state_changed()
        
        # Execute behavior for current state
        self.execute_current_state(left_stick, right_stick, triggers, buttons)
    
    def on_state_changed(self):
        """
        Callback function executed when FSM state transitions occur
        Note: State transitions are now primarily handled by joystick input
        """
        if self.current_state == "stand":
            rospy.loginfo("Entering STAND state")
            # Immediately publish zero velocity command to ensure immediate stop
            cmd_vel = Twist()
            self.cmd_vel_pub.publish(cmd_vel)
            # Call walkenable service to disable RL control
            self.call_walkenable_service(False)
            
        elif self.current_state == "walk":
            rospy.loginfo("Entering WALK state")
            # Enable RL control when entering WALK state
            self.call_walkenable_service(True)
    
    def call_walkenable_service(self, enable_value, is_initialization=False):
        """
        Invoke reinforcement learning control service
        
        Args:
            enable_value (bool): Service request value - True enables RL control, False disables
            is_initialization (bool): If True, wait indefinitely for service; if False, use timeout
        """
        try:
            # Check service availability
            rospy.loginfo(f"Waiting for service {self.walkenable_service_name}...")
            
            if is_initialization:
                # During initialization, wait indefinitely until service becomes available
                rospy.wait_for_service(self.walkenable_service_name)
                rospy.loginfo(f"Service {self.walkenable_service_name} is now available")
            else:
                # During runtime, use timeout to avoid blocking
                rospy.wait_for_service(self.walkenable_service_name, timeout=5.0)
                rospy.loginfo(f"Service {self.walkenable_service_name} is available")
            
            # Create service request message
            request = SetBoolRequest()
            request.data = enable_value
            
            # Execute service call
            rospy.loginfo(f"Calling walkenable service with value: {enable_value}")
            response = self.walkenable_client(request)
            
            if response.success:
                rospy.loginfo(f"Walkenable service call successful: {response.message}")
            else:
                rospy.logwarn(f"Walkenable service call failed: {response.message}")
                
        except rospy.ROSException as e:
            if is_initialization:
                rospy.logerr(f"Failed to wait for service {self.walkenable_service_name} during initialization: {e}")
                raise  # Re-raise exception during initialization to prevent node from starting
            else:
                rospy.logwarn(f"Service {self.walkenable_service_name} not available, skipping call")
        except rospy.ServiceException as e:
            rospy.logerr(f"Failed to call walkenable service: {e}")
        except Exception as e:
            rospy.logerr(f"Unexpected error calling walkenable service: {e}")
    
    def call_mpc_control_service(self, control_mode, is_initialization=False):
        """
        Invoke mobile manipulator MPC control service
        
        Args:
            control_mode (int): Control mode - 0: None, 1: ArmOnly, 2: other modes
            is_initialization (bool): If True, wait indefinitely for service; if False, use timeout
        """
        try:
            # Check service availability
            rospy.loginfo(f"Waiting for service {self.mpc_control_service_name}...")
            
            if is_initialization:
                # During initialization, wait indefinitely until service becomes available
                rospy.wait_for_service(self.mpc_control_service_name)
                rospy.loginfo(f"Service {self.mpc_control_service_name} is now available")
            else:
                # During runtime, use timeout to avoid blocking
                rospy.wait_for_service(self.mpc_control_service_name, timeout=5.0)
                rospy.loginfo(f"Service {self.mpc_control_service_name} is available")
            
            # Create service request message
            request = changeTorsoCtrlModeRequest()
            request.control_mode = control_mode
            
            # Execute service call
            rospy.loginfo(f"Calling mobile_manipulator_mpc_control service with control_mode: {control_mode}")
            response = self.mpc_control_client(request)
            
            if response.result:
                rospy.loginfo(f"MPC control service call successful: {response.message}")
            else:
                rospy.logwarn(f"MPC control service call failed: {response.message}")
                
        except rospy.ROSException as e:
            if is_initialization:
                rospy.logerr(f"Failed to wait for service {self.mpc_control_service_name} during initialization: {e}")
                raise  # Re-raise exception during initialization to prevent node from starting
            else:
                rospy.logwarn(f"Service {self.mpc_control_service_name} not available, skipping call")
        except rospy.ServiceException as e:
            rospy.logerr(f"Failed to call mobile_manipulator_mpc_control service: {e}")
        except Exception as e:
            rospy.logerr(f"Unexpected error calling mobile_manipulator_mpc_control service: {e}")
    
    def execute_current_state(self, left_stick, right_stick, triggers, buttons):
        """
        Execute behavior associated with current FSM state
        
        Args:
            left_stick (dict): Left joystick axis data
            right_stick (dict): Right joystick axis data
            triggers (dict): Trigger sensor data
            buttons (dict): Button state information
        """
        if self.current_state == "stand":
            self.execute_stand_state(left_stick, right_stick, triggers, buttons)
            
        elif self.current_state == "walk":
            self.execute_walk_state(left_stick, right_stick, triggers, buttons)
    
    def execute_stand_state(self, left_stick, right_stick, triggers, buttons):
        """
        Execute standing state behavior - maintain stationary position
        
        Args:
            left_stick (dict): Left joystick axis data
            right_stick (dict): Right joystick axis data
            triggers (dict): Trigger sensor data
            buttons (dict): Button state information
        """
        # Publish zero velocity command in standing state to ensure robot immobility
        cmd_vel = Twist()
        cmd_vel.linear.x = 0.0
        cmd_vel.linear.y = 0.0
        cmd_vel.linear.z = 0.0
        cmd_vel.angular.x = 0.0
        cmd_vel.angular.y = 0.0
        cmd_vel.angular.z = 0.0
        
        self.cmd_vel_pub.publish(cmd_vel)
        
        # Additional standing state logic implementation
        # Can detect joystick input for posture adjustment (but does not control locomotion)
        left_magnitude = np.sqrt(left_stick['x']**2 + left_stick['y']**2)
        if left_magnitude > 0.1:  # Apply deadzone threshold
            rospy.loginfo_throttle(2.0, f"STAND state - Left stick input detected but movement disabled: x={left_stick['x']:.2f}, y={left_stick['y']:.2f}")
    
    def execute_walk_state(self, left_stick, right_stick, triggers, buttons):
        """
        Execute walking state behavior - process locomotion commands
        
        Args:
            left_stick (dict): Left joystick axis data
            right_stick (dict): Right joystick axis data
            triggers (dict): Trigger sensor data
            buttons (dict): Button state information
        """
        # Create Twist message for velocity control
        cmd_vel = Twist()
        
        # Map joystick data to velocity commands using coordinate transformation
        # Left joystick y-axis -> linear.x (forward/backward motion, forward is positive)
        linear_x = left_stick['y'] * self.max_linear_x  # Joystick forward produces positive value
        
        # Apply velocity reduction for backward motion (safety consideration)
        if linear_x < 0:
            linear_x = linear_x / 2  # Reduce backward speed by half
            
        cmd_vel.linear.x = linear_x
        
        # Left joystick x-axis -> linear.y (lateral motion)
        cmd_vel.linear.y = left_stick['x'] * self.max_linear_y
        
        # Disable vertical motion for ground-based locomotion
        cmd_vel.linear.z = 0.0
        
        # Right joystick x-axis -> angular.z (yaw rotation, counter-clockwise positive)
        cmd_vel.angular.z = -right_stick['x'] * self.max_angular_z  # Invert for intuitive control mapping
        
        # Disable roll and pitch rotations for stability
        cmd_vel.angular.x = 0.0
        cmd_vel.angular.y = 0.0
        
        # Publish velocity command to robot controller
        self.cmd_vel_pub.publish(cmd_vel)
        
        # Throttled logging to prevent console spam
        left_magnitude = np.sqrt(left_stick['x']**2 + left_stick['y']**2)
        right_magnitude = np.sqrt(right_stick['x']**2 + right_stick['y']**2)
        
        if left_magnitude > 0.1 or right_magnitude > 0.1:  # Apply deadzone threshold
            rospy.loginfo_throttle(2.0, 
                f"WALK state - cmd_vel: linear_x={cmd_vel.linear.x:.2f}, "
                f"linear_y={cmd_vel.linear.y:.2f}, angular_z={cmd_vel.angular.z:.2f}")

    def run(self):
        """
        Execute main node control loop at specified frequency
        """
        rate = rospy.Rate(100)  # 100Hz control frequency
        
        rospy.loginfo("Quest3 FSM Node is running...")
        
        while not rospy.is_shutdown() and self.node_active:
            try:
                # Additional periodic logic can be implemented here
                rate.sleep()
                
            except rospy.ROSInterruptException:
                break
                
        rospy.loginfo("Quest3 FSM Node shutting down...")
    
    def shutdown(self):
        """
        Gracefully shutdown node and cleanup resources
        """
        self.node_active = False
        rospy.loginfo("Quest3 FSM Node shutdown requested")


def main():
    """
    Main entry point for Quest3 FSM node execution
    """
    try:
        # Instantiate node object
        quest3_fsm = Quest3FSMNode()
        
        # Execute node main loop
        quest3_fsm.run()
        
    except rospy.ROSInterruptException:
        rospy.loginfo("Quest3 FSM Node interrupted")
    except Exception as e:
        rospy.logerr(f"Quest3 FSM Node error: {e}")
    finally:
        rospy.loginfo("Quest3 FSM Node terminated")


if __name__ == '__main__':
    main()
