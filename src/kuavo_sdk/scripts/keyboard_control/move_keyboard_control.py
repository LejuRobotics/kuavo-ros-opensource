#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import Joy
import sys
import termios
import tty
import select
from std_msgs.msg import Bool

# JoyButton constants
BUTTON_A = 0
BUTTON_B = 1
BUTTON_X = 2
BUTTON_Y = 3
BUTTON_LB = 4
BUTTON_RB = 5
BUTTON_BACK = 6
BUTTON_START = 7

# JoyAxis constants
AXIS_LEFT_STICK_Y = 0
AXIS_LEFT_STICK_X = 1
AXIS_LEFT_LT = 2  # 1 -> (-1)
AXIS_RIGHT_STICK_YAW = 3
AXIS_RIGHT_STICK_Z = 4
AXIS_RIGHT_RT = 5  # 1 -> (-1)
AXIS_LEFT_RIGHT_TRIGGER = 6
AXIS_FORWARD_BACK_TRIGGER = 7

class SimulatedJoystick:
    def __init__(self):
        rospy.init_node('simulated_joystick')
        rospy.Subscriber("/stop_robot", Bool, self.stop_robot_callback)

        self.joy_pub = rospy.Publisher('/joy', Joy, queue_size=10)
        self.joy_msg = Joy()
        self.joy_msg.axes = [0.0] * 8  # Initialize 8 axes
        self.joy_msg.buttons = [0] * 11  # Initialize 11 buttons
        self.old_settings = termios.tcgetattr(sys.stdin)

    def stop_robot_callback(self, msg):
        rospy.signal_shutdown("stop_robot")
        
    def getKey(self):
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_settings)
        return key

    def update_joy(self, key):
        key = key.lower()
        # Reset all buttons
        self.joy_msg.buttons = [0] * 11

        # Gradual change for axes
        if key == 'w':
            self.joy_msg.axes[AXIS_LEFT_STICK_X] = round(min(1.0, self.joy_msg.axes[AXIS_LEFT_STICK_X] + 0.1), 3)
        elif key == 's':
            self.joy_msg.axes[AXIS_LEFT_STICK_X] = round(max(-1.0, self.joy_msg.axes[AXIS_LEFT_STICK_X] - 0.1), 3)
        elif key == 'a':
            self.joy_msg.axes[AXIS_LEFT_STICK_Y] = round(min(1.0, self.joy_msg.axes[AXIS_LEFT_STICK_Y] + 0.1), 3)
        elif key == 'd':
            self.joy_msg.axes[AXIS_LEFT_STICK_Y] = round(max(-1.0, self.joy_msg.axes[AXIS_LEFT_STICK_Y] - 0.1), 3)
        elif key == 'i':
            self.joy_msg.axes[AXIS_RIGHT_STICK_Z] = round(min(1.0, self.joy_msg.axes[AXIS_RIGHT_STICK_Z] + 0.1), 3)
        elif key == 'k':
            self.joy_msg.axes[AXIS_RIGHT_STICK_Z] = round(max(-1.0, self.joy_msg.axes[AXIS_RIGHT_STICK_Z] - 0.1), 3)
        elif key == 'l' or key == 'e':
            self.joy_msg.axes[AXIS_RIGHT_STICK_YAW] = round(max(-1.0, self.joy_msg.axes[AXIS_RIGHT_STICK_YAW] - 0.1), 3)
        elif key == 'j' or key == 'q':
            self.joy_msg.axes[AXIS_RIGHT_STICK_YAW] = round(min(1.0, self.joy_msg.axes[AXIS_RIGHT_STICK_YAW] + 0.1), 3)
        elif key == ' ':  # Space key
            self.joy_msg.axes = [0.0] * 8  # Reset all axes to zero
        elif key == 'r':
            self.joy_msg.buttons[BUTTON_Y] = 1  # 发送walk
        elif key == 'c':
            self.joy_msg.buttons[BUTTON_A] = 1  # 发送stance
            
        cmdvel = [self.joy_msg.axes[AXIS_LEFT_STICK_X],self.joy_msg.axes[AXIS_LEFT_STICK_Y], 0, 0, 0, self.joy_msg.axes[AXIS_RIGHT_STICK_YAW]]
        print(f"cmdvel: {[f'{x * 100:.0f}%' for x in cmdvel]}", end='\r')
        # self.joy_pub.publish(self.joy_msg)

    def run(self):
        try:
            print("Use keys to control:")
            print("WASD: Left stick, control forward/backward, left/right")
            print("IKJL/QE: Right stick, up/down, turn left/right")
            print("R: walk, C: stance")
            print("<space>: Reset all axes to zero")
            print("Press Ctrl-C to exit")
            
            while not rospy.is_shutdown():
                key = self.getKey()
                if key:
                    self.update_joy(key)
                if (key == '\x03'):  # Ctrl-C
                    break
                self.joy_pub.publish(self.joy_msg)
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_settings)

if __name__ == '__main__':
    try:
        simulated_joystick = SimulatedJoystick()
        simulated_joystick.run()
    except rospy.ROSInterruptException:
        pass
