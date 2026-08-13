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

  身体 (躯干 + 双臂联动, Ruckig 平滑同步):
    b        send_timed_multi_commands(is_sync=True): 躯干(planner=2) + 左臂(6) + 右臂(7)
    h        躯干复位 + 手臂复位 (/mobile_manipulator_reset_torso + arm_ctrl_mode=1)

    space    停止所有底盘移动 (速度归零)
    p        打印当前各状态量
    q        退出
"""

import math, sys, select, termios, tty
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from std_srvs.srv import SetBool
import lb_ctrl_api as ct

LINEAR_SPEED = 0.3
ANGULAR_SPEED = 0.5

# ---------- 躯干 + 双臂联动轨迹 ----------
# 躯干: planner_index=2, cmdVec=[x, z, yaw, pitch] (绝对, 局部系)
# 左臂: planner_index=6, cmdVec=[x, y, z, roll, pitch, yaw] (deg, 局部系 ← RPM 内部转弧度)
# 右臂: planner_index=7, cmdVec=[x, y, z, roll, pitch, yaw] (deg, 局部系)
#
# 数值参考 cmd_torso_ee_local_test.py，按需调整
# 躯干 workspace 限位（对齐 BT2 MobileManipulatorJoyCommandNode.cpp:104-108 + getTorsoMaxX）：
#   x: 0~0.25 (z联动: z=0.22→x≤0.123)
#   z: 0~0.32  pitch: 0~0.5235  yaw: ±0.5235
ARM_EE_POSE_DEG = {
    "torso_rel": (0.06, 0.22, 0.0, 0.0),    # (dx, dz, yaw, pitch): 前移6cm, 升高22cm
    "left":      (0.50, 0.30, 1.15, 0.0, -60.0, 0.0),  # (x, y, z, roll, pitch, yaw) deg
    "right":     (0.50, -0.30, 1.15, 0.0, -60.0, 0.0),
}
DESIRE_TIME = 4.0   # 动作时长 (秒)，越大越缓慢


def getch():
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        r, _, _ = select.select([sys.stdin], [], [], 0.05)
        return sys.stdin.read(1) if r else ''
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)


def _deg_to_rad(deg_vec):
    """前 3 维 (位置 m) 不动，后 3 维角度 deg → rad"""
    return [deg_vec[0], deg_vec[1], deg_vec[2],
            math.radians(deg_vec[3]), math.radians(deg_vec[4]), math.radians(deg_vec[5])]


class Test:
    def __init__(self):
        rospy.init_node('manual_test', anonymous=True)

        self._pub_cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self._pub_base_cmd_vel = rospy.Publisher('/move_base/base_cmd_vel', Twist, queue_size=10)

        self._cmd_vel_vx = 0.0
        self._cmd_vel_wz = 0.0
        self._base_cmd_vel_vx = 0.0
        self._running = True

        self._enable_control = True
        self._enable_vel_control = True
        self._torso_initial = None   # 躯干初始绝对位姿 (x, y, z, roll, pitch, yaw)
        # 订阅 latched 使能话题（发布值 = 状态机衍生，仅 IDLE 为 true）：
        # /enable_control 受理矩阵：过渡态（PAUSING/RESUMING）拒绝一切；稳定态只受理
        # 有效切换（IDLE 受理 disable、PAUSED 受理 enable），重复请求也被拒（预期）。
        # 以话题值为准同步本地状态，避免脚本内部状态与 controller 错位。
        self._enable_state_sub = rospy.Subscriber('/enable_control_state', Bool,
                                                  self._on_enable_control_state, queue_size=1)
        self._init()

    def _on_enable_control_state(self, msg):
        self._enable_control = msg.data

    def _init(self):
        print('[init] ...')
        ct.reset_torso_to_initial(); rospy.sleep(1.5)
        ct.set_arm_control_mode(1);  rospy.sleep(1.5)
        ct.set_arm_control_mode(2)
        ct.set_focus_ee(True); ct.set_focus_z(False)
        ct.set_wheel_control_enable(True)
        self._call_enable_vel_control(True)
        rospy.sleep(3.0)

        # 记录初始躯干位姿，用于联动轨迹的绝对坐标计算
        ok, pose = ct.get_torso_initial_pose(True)
        if ok and pose:
            self._torso_initial = pose['position']  # [x, y, z, roll, pitch, yaw]
            print(f'[init] torso initial: x={self._torso_initial[0]:.3f} z={self._torso_initial[2]:.3f}')
        else:
            print('[init] WARN: failed to get torso initial pose')

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

    def _body_action(self):
        """躯干 + 双臂联动，Ruckig 同步规划"""
        if self._torso_initial is None:
            print('  no torso initial pose, skip')
            return

        ct.set_arm_control_mode(2)   # 切到外部控制，确保手臂能收 timed command

        t = ARM_EE_POSE_DEG
        dx, dz, yaw, pitch = t["torso_rel"]
        torso_cmd = [self._torso_initial[0] + dx,
                     self._torso_initial[2] + dz,
                     yaw,
                     pitch]

        timed_cmd_vec = [
            {'planner_index': 2, 'desire_time': DESIRE_TIME, 'cmd_vec': torso_cmd},
            {'planner_index': 6, 'desire_time': DESIRE_TIME, 'cmd_vec': _deg_to_rad(t["left"])},
            {'planner_index': 7, 'desire_time': DESIRE_TIME, 'cmd_vec': _deg_to_rad(t["right"])},
        ]

        success, actual_time, msg = ct.send_timed_multi_commands(timed_cmd_vec, is_sync=True)
        if success:
            print(f'  ✓ 躯干+手臂联动完成 ({actual_time:.1f}s)')
        elif ct._is_soft_pause_rejection(msg):
            print(f'  \033[92m✓ 躯干+手臂联动被拒（预期：软暂停窗口内）\033[0m')
        else:
            print(f'  ⚠️ 躯干+手臂联动失败: {msg}')

    def _body_home(self):
        """躯干复位 + 手臂复位"""
        ct.reset_torso_to_initial()
        ct.set_arm_control_mode(1)   # 手臂回初始站姿
        print('  torso + arm reset done')

    def _publish_all(self):
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
                    target = not self._enable_control   # 基于话题真实值取反
                    if ct.set_wheel_control_enable(target):
                        self._enable_control = target
                        print(f'  /enable_control -> {self._enable_control}')
                    else:
                        print(f'  \033[92m✓ /enable_control 被拒（预期：过渡窗口内 或 已处于目标态），保持 {self._enable_control}\033[0m')
                elif c == 'v':
                    self._enable_vel_control = not self._enable_vel_control
                    self._call_enable_vel_control(self._enable_vel_control)
                    print(f'  /enable_vel_control -> {self._enable_vel_control}')
                elif c == 'b':
                    self._body_action()
                    print(f'  torso+arms sync ({DESIRE_TIME}s)')
                elif c == 'h':
                    self._body_home()
                    print('  torso + arm reset')
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
