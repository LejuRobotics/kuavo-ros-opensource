#!/usr/bin/env python3
"""测试 /enable_control 和 /enable_vel_control 的交互行为。

  底盘 (经 CDM→RM→MPC 门控管线, disable 时被拦截):
    w/s      发 /cmd_vel 前进/后退 (开关, 互斥)
    a/d      发 /cmd_vel 左转/右转 (开关, 互斥)

  底盘 (直发 /move_base/base_cmd_vel, 绕过门控):
    i/o      直发 /move_base/base_cmd_vel 前进/后退 (开关, 互斥)

  enable 服务:
    e        调用 /enable_control (切换)
    v        调用 /enable_vel_control (切换)

  身体 (躯干 /cmd_lb_torso_pose + 手臂 /kuavo_arm_traj):
    b        发送躯干+手臂动作指令
    h        发送躯干+手臂归位指令

    space    停止所有底盘移动 (速度归零)
    p        打印当前各状态量
    q        退出
"""

import sys, select, termios, tty
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from std_srvs.srv import SetBool
import lb_ctrl_api as ct

LINEAR_SPEED = 0.3
ANGULAR_SPEED = 0.5

ARM_MOVE = [-30, 20, 15, -45, 25, 10, -35, -30, -20, -15, -45, -25, -10, -35]
ARM_HOME = [0.0] * 14
ARM_NAMES = [f'joint{i}' for i in range(1, 15)]


def getch():
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        r, _, _ = select.select([sys.stdin], [], [], 0.05)
        return sys.stdin.read(1) if r else ''
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)


class Test:
    def __init__(self):
        rospy.init_node('manual_test', anonymous=True)

        self._pub_cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self._pub_base_cmd_vel = rospy.Publisher('/move_base/base_cmd_vel', Twist, queue_size=10)
        self._pub_arm_traj = rospy.Publisher('/kuavo_arm_traj', JointState, queue_size=10)
        self._pub_torso_pose = rospy.Publisher('/cmd_lb_torso_pose', Twist, queue_size=10)

        self._cmd_vel_vx = 0.0       # /cmd_vel 当前线速度
        self._cmd_vel_wz = 0.0       # /cmd_vel 当前角速度
        self._base_cmd_vel_vx = 0.0  # /move_base/base_cmd_vel 当前线速度
        self._running = True

        self._enable_control = True
        self._enable_vel_control = True
        self._home_torso_xyz = None
        self._init()

    def _init(self):
        print('[init] ...')
        ct.reset_torso_to_initial(); rospy.sleep(1.5)
        ct.set_arm_control_mode(1);  rospy.sleep(1.5)
        ct.set_arm_control_mode(2)
        ct.set_focus_ee(True); ct.set_focus_z(False)
        ct.set_wheel_control_enable(True)          # /enable_control true
        self._call_enable_vel_control(True)
        rospy.sleep(3.0)
        ok, pose = ct.get_torso_initial_pose(True)
        if ok and pose:
            p = pose['position']; self._home_torso_xyz = [p[0], 0.0, p[2], 0.0, 0.0, 0.0]
        self._send_arm_traj(ARM_HOME)
        rospy.sleep(2.0)
        print('[init] ok')
        print('w/s=/cmd_vel前进后退 a/d=/cmd_vel转向 i/o=/move_base/base_cmd_vel直驱 space=停 e=/enable_control v=/enable_vel_control b=身体动作 h=身体归位 p=状态 q=退出')

    def _call_enable_vel_control(self, enable):
        try:
            rospy.wait_for_service('/enable_vel_control', timeout=3.0)
            rospy.ServiceProxy('/enable_vel_control', SetBool)(enable)
            self._enable_vel_control = enable
        except: pass

    def _pub_twist(self, pub, vx=0.0, wz=0.0):
        tw = Twist(); tw.linear.x = vx; tw.angular.z = wz
        pub.publish(tw)

    def _send_arm_traj(self, deg):
        m = JointState(); m.header.stamp = rospy.Time.now()
        m.name = ARM_NAMES; m.position = deg
        m.velocity = [0.0]*14; m.effort = [0.0]*14
        self._pub_arm_traj.publish(m)

    def _send_torso_pose(self, xyz_ypr):
        tw = Twist()
        tw.linear.x, tw.linear.y, tw.linear.z = xyz_ypr[0], xyz_ypr[1], xyz_ypr[2]
        tw.angular.z, tw.angular.y = xyz_ypr[3], xyz_ypr[4]
        self._pub_torso_pose.publish(tw)

    def _body_action(self):
        self._send_torso_pose([0, 0, 0.15, 0.25, -0.08, 0])
        self._send_arm_traj(ARM_MOVE)

    def _body_home(self):
        if self._home_torso_xyz:
            self._send_torso_pose(self._home_torso_xyz)
        self._send_arm_traj(ARM_HOME)

    def _publish_all(self):
        """同时发布 /cmd_vel 和 /move_base/base_cmd_vel 的当前速度"""
        self._pub_twist(self._pub_cmd_vel, vx=self._cmd_vel_vx, wz=self._cmd_vel_wz)
        self._pub_twist(self._pub_base_cmd_vel, vx=self._base_cmd_vel_vx)

    def _stop_all(self):
        self._cmd_vel_vx = 0.0; self._cmd_vel_wz = 0.0; self._base_cmd_vel_vx = 0.0

    def run(self):
        rate = rospy.Rate(50)
        while self._running and not rospy.is_shutdown():
            c = getch()
            while c:
                if c == 'q':
                    self._stop_all(); self._running = False; break
                elif c == ' ':
                    self._stop_all()
                    print('  /cmd_vel=0 /move_base/base_cmd_vel=0')
                elif c == 'w':
                    self._cmd_vel_vx = LINEAR_SPEED if self._cmd_vel_vx <= 0 else 0.0
                    if self._cmd_vel_vx != 0: self._base_cmd_vel_vx = 0.0
                    print(f'  /cmd_vel vx={self._cmd_vel_vx}')
                elif c == 's':
                    self._cmd_vel_vx = -LINEAR_SPEED if self._cmd_vel_vx >= 0 else 0.0
                    if self._cmd_vel_vx != 0: self._base_cmd_vel_vx = 0.0
                    print(f'  /cmd_vel vx={self._cmd_vel_vx}')
                elif c == 'a':
                    self._cmd_vel_wz = ANGULAR_SPEED if self._cmd_vel_wz <= 0 else 0.0
                    if self._cmd_vel_wz != 0: self._base_cmd_vel_vx = 0.0
                    print(f'  /cmd_vel wz={self._cmd_vel_wz}')
                elif c == 'd':
                    self._cmd_vel_wz = -ANGULAR_SPEED if self._cmd_vel_wz >= 0 else 0.0
                    if self._cmd_vel_wz != 0: self._base_cmd_vel_vx = 0.0
                    print(f'  /cmd_vel wz={self._cmd_vel_wz}')
                elif c == 'i':
                    self._base_cmd_vel_vx = LINEAR_SPEED if self._base_cmd_vel_vx <= 0 else 0.0
                    if self._base_cmd_vel_vx != 0: self._cmd_vel_vx = 0.0; self._cmd_vel_wz = 0.0
                    print(f'  /move_base/base_cmd_vel vx={self._base_cmd_vel_vx}')
                elif c == 'o':
                    self._base_cmd_vel_vx = -LINEAR_SPEED if self._base_cmd_vel_vx >= 0 else 0.0
                    if self._base_cmd_vel_vx != 0: self._cmd_vel_vx = 0.0; self._cmd_vel_wz = 0.0
                    print(f'  /move_base/base_cmd_vel vx={self._base_cmd_vel_vx}')
                elif c == 'e':
                    self._enable_control = not self._enable_control
                    ct.set_wheel_control_enable(self._enable_control)
                    print(f'  /enable_control -> {self._enable_control}')
                elif c == 'v':
                    self._enable_vel_control = not self._enable_vel_control
                    self._call_enable_vel_control(self._enable_vel_control)
                    print(f'  /enable_vel_control -> {self._enable_vel_control}')
                elif c == 'b':
                    self._body_action()
                    print('  /cmd_lb_torso_pose + /kuavo_arm_traj -> action')
                elif c == 'h':
                    self._body_home()
                    print('  /cmd_lb_torso_pose + /kuavo_arm_traj -> home')
                elif c == 'p':
                    print(f'  /enable_control={self._enable_control}'
                          f'  /enable_vel_control={self._enable_vel_control}'
                          f'  /cmd_vel(vx={self._cmd_vel_vx}, wz={self._cmd_vel_wz})'
                          f'  /move_base/base_cmd_vel(vx={self._base_cmd_vel_vx})')

                c = getch()

            self._publish_all()
            rate.sleep()

        self._stop_all(); self._publish_all()
        ct.set_wheel_control_enable(True)
        self._call_enable_vel_control(True)


if __name__ == '__main__':
    try: Test().run()
    except rospy.ROSInterruptException: pass
    except KeyboardInterrupt: print('\ninterrupted')
