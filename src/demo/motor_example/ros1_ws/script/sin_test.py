#!/usr/bin/env python3

import rospy
import numpy as np
from sensor_msgs.msg import JointState
import time
import signal
import sys

class ArmRuiwoMotorTest:
    def __init__(self):
        rospy.init_node('ruiwo_motor_arm_test', anonymous=True)
        self.joint_pub = rospy.Publisher('/ruiwo_motor/command', JointState, queue_size=10)

        # ===== CONFIGURABLE PARAMETERS =====
        self.rate = rospy.Rate(200)  # 200Hz

        # Get arm type from rosparameter, default to 'left'
        self.arm_type = rospy.get_param('arm_type', 'left')
        if self.arm_type not in ['left', 'right']:
            rospy.logwarn(f"Invalid arm_type '{self.arm_type}', defaulting to 'left'")
            self.arm_type = 'left'
        rospy.loginfo(f"Using {self.arm_type} arm")

        # Define left and right arm joint names separately
        self.left_arm_joint_names = [
            'zarm_l2_joint', 'zarm_l3_joint', 'zarm_l4_joint', 'zarm_l5_joint', 'zarm_l6_joint', 'zarm_l7_joint'
        ]
        self.right_arm_joint_names = [
            'zarm_r2_joint', 'zarm_r3_joint', 'zarm_r4_joint', 'zarm_r5_joint', 'zarm_r6_joint', 'zarm_r7_joint'
        ]

        # Define left arm joint limits
        self.left_arm_joint_limits = [
            (10, 90),    # zarm_l2
            (-45, 45),   # zarm_l3
            (-45, 0),    # zarm_l4
            (-45, 45),   # zarm_l5
            (-20, 20),   # zarm_l6
            (-45, 30)    # zarm_l7
        ]

        # Define right arm joint limits
        self.right_arm_joint_limits = [
            (-90, -10),  # zarm_r2
            (-45, 45),   # zarm_r3
            (-45, 0),    # zarm_r4
            (-45, 45),   # zarm_r5
            (-20, 20),   # zarm_r6
            (-30, 45)    # zarm_r7
        ]

        # Phase offsets for left arm joints
        self.left_arm_phase_offsets = [
            0.0, np.pi/6, np.pi/3, np.pi/2, 2*np.pi/3, 5*np.pi/6
        ]

        # Phase offsets for right arm joints
        self.right_arm_phase_offsets = [
            np.pi, 7*np.pi/6, 4*np.pi/3, 3*np.pi/2, 5*np.pi/3, 11*np.pi/6
        ]

        # Select joint names, limits, and phase offsets based on arm_type
        if self.arm_type == 'left':
            self.joint_names = self.left_arm_joint_names
            self.joint_limits = self.left_arm_joint_limits
            self.phase_offsets = self.left_arm_phase_offsets
        else:
            self.joint_names = self.right_arm_joint_names
            self.joint_limits = self.right_arm_joint_limits
            self.phase_offsets = self.right_arm_phase_offsets

        # Speed multiplier to adjust movement speed
        self.speed_multiplier = 2.0  # Default to double speed
        self.frequencies = [0.3*self.speed_multiplier] * len(self.joint_names) # Base frequencies for arm joints 0.3Hz
        # ===== END OF CONFIGURABLE PARAMETERS =====

        rospy.loginfo(f"Initialized for {self.arm_type} arm with {len(self.joint_names)} joints")

        signal.signal(signal.SIGINT, self.signal_handler)
        self.start_time = time.time()
        self.running = True
            
    def signal_handler(self, sig, frame):
        """Handle Ctrl+C to stop the node gracefully"""
        print("\nStopping joint movement and shutting down...")
        self.running = False
        self.move_to_zero_position()
        sys.exit(0)
    
    def move_to_zero_position(self):
        """Gradually move all joints to zero position"""
        print("Moving to zero position...")
        # Get current positions
        current_positions = self.calculate_joint_positions(time.time() - self.start_time)
        target_positions = [0.0] * len(self.joint_names)
        
        # Gradually move to zero position
        steps = 400  # Number of steps for smooth transition
        
        for step in range(steps + 1):
            # Calculate intermediate positions with easing
            t = step / steps  # Normalized time from 0 to 1
            easing_factor = (1 - np.cos(t * np.pi)) / 2  # Sinusoidal easing
            
            intermediate_positions = [
                current + (target - current) * easing_factor
                for current, target in zip(current_positions, target_positions)
            ]
            
            self.publish_joint_command(intermediate_positions)
            self.rate.sleep()  # Use the defined rate instead of fixed delay
        
        print("Reached zero position")
    
    def move_to_start_position(self):
        """Gradually move all joints to the starting position of the sinusoidal movement"""
        print("Moving to sinusoidal starting position...")
        # Get current positions
        current_positions = [0.0] * len(self.joint_names)  # Assume we're at zero
        
        # Calculate the starting positions (t=0 in the sinusoidal movement)
        start_positions = self.calculate_joint_positions(0)
        
        # Gradually move to start position
        steps = 400  # Number of steps for smooth transition
        
        for step in range(steps + 1):
            # Calculate intermediate positions with easing
            t = step / steps  # Normalized time from 0 to 1
            easing_factor = (1 - np.cos(t * np.pi)) / 2  # Sinusoidal easing
            
            intermediate_positions = [
                current + (target - current) * easing_factor
                for current, target in zip(current_positions, start_positions)
            ]
            
            self.publish_joint_command(intermediate_positions)
            self.rate.sleep()  # Use the defined rate instead of fixed delay
        
        print("Reached sinusoidal starting position")

    def calculate_joint_positions(self, t):
        """Calculate joint positions using sinusoidal functions"""
        positions = []

        # Calculate positions for selected arm joints
        for i in range(len(self.joint_names)):
            min_val, max_val = self.joint_limits[i]
            # Convert angle limits from degrees to radians if needed
            min_val = min_val * np.pi / 180.0
            max_val = max_val * np.pi / 180.0
            mid_val = (max_val + min_val) / 2
            amplitude = (max_val - min_val) / 2

            pos = mid_val + amplitude * np.sin(self.frequencies[i] * t + self.phase_offsets[i])
            positions.append(pos)

        return positions
    
    def publish_joint_command(self, positions):
        """Create and publish joint state message"""
        # Check if positions length matches joint_names length
        if len(positions) != len(self.joint_names):
            rospy.logwarn(f"Position array length ({len(positions)}) does not match joint names length ({len(self.joint_names)})")
            return
        
        # Also publish to joint_states topic
        joint_cmd = JointState()
        joint_cmd.header.stamp = rospy.Time.now()
        joint_cmd.name = self.joint_names
        joint_cmd.position = positions

        # radians
        self.joint_pub.publish(joint_cmd)
        
    def run(self):
        """Main loop to publish joint positions"""
        print(f"Starting joint movement with speed multiplier {self.speed_multiplier}x. Press Ctrl+C to stop.")
                
        self.move_to_start_position()
        
        # Main control loop
        self.start_time = time.time()
        while not rospy.is_shutdown() and self.running:
            t = time.time() - self.start_time
            positions = self.calculate_joint_positions(t)
            self.publish_joint_command(positions)
            self.rate.sleep()

if __name__ == '__main__':
    try:
        controller = ArmRuiwoMotorTest()
        controller.run()
    except rospy.ROSInterruptException:
        pass
