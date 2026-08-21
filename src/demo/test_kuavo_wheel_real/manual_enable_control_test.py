#!/usr/bin/env python3
"""测试 /enable_control 和 /enable_vel_control 的交互行为（软暂停测试入口）。

默认模式（不加 --simulate-joy）：
  仅暴露软暂停相关功能，不发送任何底盘速度指令，避免与手柄/G12 冲突。
    e        调用 /enable_control (切换)
    v        调用 /enable_vel_control (切换)
    p        打印当前各状态量
    q        退出

带 --simulate-joy 参数时额外开启（模拟软遥控，可能与 G12 冲突）：
  底盘速度由 w/s/a/d 控制，根据 /enable_vel_control 状态自动选择通路：
    v=ON (enable_vel_control=true)   -> w/s/a/d 发 /cmd_vel（经 CDM→RM→MPC 门控管线）
    v=OFF (enable_vel_control=false) -> w/s/a/d 直发 /move_base/base_cmd_vel（绕过门控）

  身体 (躯干 + 双臂联动, Ruckig 平滑同步):
    b        send_timed_multi_commands(is_sync=True): 躯干(planner=2) + 左臂(6) + 右臂(7)
    h        躯干复位 + 手臂复位 (/mobile_manipulator_reset_torso + arm_ctrl_mode=1)

    space    停止所有底盘移动 (速度归零, 仅发一次, 不会持续发零速)

用法：
    python3 manual_enable_control_test.py              # 仅软暂停测试
    python3 manual_enable_control_test.py --simulate-joy  # 同时模拟遥控发速度
"""

import argparse
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
    def __init__(self, simulate_joy=False):
        rospy.init_node('manual_test', anonymous=True)

        self._simulate_joy = simulate_joy

        self._pub_cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self._pub_base_cmd_vel = rospy.Publisher('/move_base/base_cmd_vel', Twist, queue_size=10)

        self._cmd_vel_vx = 0.0
        self._cmd_vel_wz = 0.0
        self._base_cmd_vel_vx = 0.0
        self._base_cmd_vel_wz = 0.0
        # 记录上次实际发布的速度，用于“零速不持续发指令”
        self._last_pub_cmd_vel_vx = 0.0
        self._last_pub_cmd_vel_wz = 0.0
        self._last_pub_base_cmd_vel_vx = 0.0
        self._last_pub_base_cmd_vel_wz = 0.0
        # 切换 v 后强制持续发零速的截止时间
        self._vel_switch_stop_until = rospy.Time(0)

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
        if self._simulate_joy:
            print('w/s/a/d=底盘速度  e=/enable_control  v=/enable_vel_control(决定走 /cmd_vel 还是 /move_base/base_cmd_vel)'
                  '  space=停  b=身体动作  h=身体归位  p=状态  q=退出')
        else:
            print('当前为软暂停测试模式，不发速度指令。e=/enable_control v=/enable_vel_control p=状态 q=退出')
            print('如需发速度模拟遥控，请带 --simulate-joy 参数启动。')

    def _call_enable_vel_control(self, enable):
        try:
            rospy.wait_for_service('/enable_vel_control', timeout=3.0)
            rospy.ServiceProxy('/enable_vel_control', SetBool)(enable)
            self._enable_vel_control = enable
        except: pass

    def _pub_twist(self, pub, vx=0.0, wz=0.0):
        tw = Twist(); tw.linear.x = vx; tw.angular.z = wz
        pub.publish(tw)

    @staticmethod
    def _toggle_speed(current, pos_speed, neg_speed, key):
        """开关式速度设置：w/a 发正向，s/d 发负向；已在目标方向则归零。"""
        if key in ('w', 'a'):
            return pos_speed if current <= 0 else 0.0
        return neg_speed if current >= 0 else 0.0

    def _handle_motion_key(self, key):
        """根据 v 状态把 w/s/a/d 映射到 /cmd_vel 或 /move_base/base_cmd_vel。"""
        if key in ('w', 's'):
            if self._enable_vel_control:
                self._cmd_vel_vx = self._toggle_speed(self._cmd_vel_vx, LINEAR_SPEED, -LINEAR_SPEED, key)
                if self._cmd_vel_vx != 0:
                    self._base_cmd_vel_vx = 0.0
                    self._base_cmd_vel_wz = 0.0
                print(f'  [v=ON] /cmd_vel vx={self._cmd_vel_vx}')
            else:
                self._base_cmd_vel_vx = self._toggle_speed(self._base_cmd_vel_vx, LINEAR_SPEED, -LINEAR_SPEED, key)
                if self._base_cmd_vel_vx != 0:
                    self._cmd_vel_vx = 0.0
                    self._cmd_vel_wz = 0.0
                print(f'  [v=OFF] /move_base/base_cmd_vel vx={self._base_cmd_vel_vx}')
        else:  # a, d
            if self._enable_vel_control:
                self._cmd_vel_wz = self._toggle_speed(self._cmd_vel_wz, ANGULAR_SPEED, -ANGULAR_SPEED, key)
                if self._cmd_vel_wz != 0:
                    self._base_cmd_vel_vx = 0.0
                    self._base_cmd_vel_wz = 0.0
                print(f'  [v=ON] /cmd_vel wz={self._cmd_vel_wz}')
            else:
                self._base_cmd_vel_wz = self._toggle_speed(self._base_cmd_vel_wz, ANGULAR_SPEED, -ANGULAR_SPEED, key)
                if self._base_cmd_vel_wz != 0:
                    self._cmd_vel_vx = 0.0
                    self._cmd_vel_wz = 0.0
                print(f'  [v=OFF] /move_base/base_cmd_vel wz={self._base_cmd_vel_wz}')

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
        """发布速度指令：非零时持续发（维持运动和超时刷新），零速只发一次。
        切换 v 后 0.5s 内两路都持续发零速，确保实际停住。
        避免与 G12/控制器在 topic 上持续竞争。"""
        if rospy.Time.now() < self._vel_switch_stop_until:
            self._pub_twist(self._pub_cmd_vel, vx=0.0, wz=0.0)
            self._pub_twist(self._pub_base_cmd_vel, vx=0.0, wz=0.0)
            self._last_pub_cmd_vel_vx = 0.0
            self._last_pub_cmd_vel_wz = 0.0
            self._last_pub_base_cmd_vel_vx = 0.0
            self._last_pub_base_cmd_vel_wz = 0.0
            return

        cmd_vel_active = (abs(self._cmd_vel_vx) > 1e-6 or abs(self._cmd_vel_wz) > 1e-6)
        cmd_vel_zero_transition = (self._last_pub_cmd_vel_vx != 0.0 or self._last_pub_cmd_vel_wz != 0.0)
        if cmd_vel_active or cmd_vel_zero_transition:
            self._pub_twist(self._pub_cmd_vel, vx=self._cmd_vel_vx, wz=self._cmd_vel_wz)
            self._last_pub_cmd_vel_vx = self._cmd_vel_vx
            self._last_pub_cmd_vel_wz = self._cmd_vel_wz

        base_cmd_vel_active = (abs(self._base_cmd_vel_vx) > 1e-6 or abs(self._base_cmd_vel_wz) > 1e-6)
        base_cmd_vel_zero_transition = (self._last_pub_base_cmd_vel_vx != 0.0 or
                                        self._last_pub_base_cmd_vel_wz != 0.0)
        if base_cmd_vel_active or base_cmd_vel_zero_transition:
            self._pub_twist(self._pub_base_cmd_vel, vx=self._base_cmd_vel_vx, wz=self._base_cmd_vel_wz)
            self._last_pub_base_cmd_vel_vx = self._base_cmd_vel_vx
            self._last_pub_base_cmd_vel_wz = self._base_cmd_vel_wz

    def _stop_all(self):
        self._cmd_vel_vx = 0.0
        self._cmd_vel_wz = 0.0
        self._base_cmd_vel_vx = 0.0
        self._base_cmd_vel_wz = 0.0

    def run(self):
        rate = rospy.Rate(50)
        while self._running and not rospy.is_shutdown():
            c = getch()
            while c:
                if c == 'q':
                    self._stop_all(); self._running = False; break
                elif c == ' ':
                    if not self._simulate_joy:
                        print('  [软暂停模式] space 无效，请加 --simulate-joy 使用速度控制')
                    else:
                        self._stop_all()
                        print('  /cmd_vel=0 /move_base/base_cmd_vel=0')
                elif c in ('w', 's', 'a', 'd'):
                    if not self._simulate_joy:
                        print(f'  [软暂停模式] {c} 无效，请加 --simulate-joy 使用速度控制')
                    else:
                        self._handle_motion_key(c)
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
                    # 切换速度控制模式时清零两边速度，并在 0.5s 内向两路持续发零速，确保实际停住
                    self._stop_all()
                    self._vel_switch_stop_until = rospy.Time.now() + rospy.Duration(0.5)
                    print(f'  /enable_vel_control -> {self._enable_vel_control}  (速度已清零，0.5s 内持续发零速)')
                elif c == 'b':
                    if not self._simulate_joy:
                        print('  [软暂停模式] b 无效，请加 --simulate-joy 使用身体动作')
                    else:
                        self._body_action()
                        print(f'  torso+arms sync ({DESIRE_TIME}s)')
                elif c == 'h':
                    if not self._simulate_joy:
                        print('  [软暂停模式] h 无效，请加 --simulate-joy 使用身体归位')
                    else:
                        self._body_home()
                        print('  torso + arm reset')
                elif c == 'p':
                    mode_str = '模拟遥控' if self._simulate_joy else '软暂停测试'
                    vel_path = '/cmd_vel' if self._enable_vel_control else '/move_base/base_cmd_vel'
                    print(f'  模式={mode_str}  当前速度通路={vel_path}'
                          f'  /enable_control={self._enable_control}'
                          f'  /enable_vel_control={self._enable_vel_control}'
                          f'  /cmd_vel(vx={self._cmd_vel_vx}, wz={self._cmd_vel_wz})'
                          f'  /move_base/base_cmd_vel(vx={self._base_cmd_vel_vx}, wz={self._base_cmd_vel_wz})')

                c = getch()

            self._publish_all()
            rate.sleep()

        self._stop_all(); self._publish_all()
        ct.set_wheel_control_enable(True)
        self._call_enable_vel_control(True)


def main():
    parser = argparse.ArgumentParser(
        description='测试 /enable_control 和 /enable_vel_control 的交互行为（软暂停测试）。')
    parser.add_argument('--simulate-joy', action='store_true',
                        help='额外开启速度/身体控制键，模拟软遥控（可能与 G12 冲突）')
    args = parser.parse_args()

    try:
        Test(simulate_joy=args.simulate_joy).run()
    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        print('\ninterrupted')


if __name__ == '__main__':
    main()
