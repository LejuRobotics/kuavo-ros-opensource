#!/usr/bin/env python3
"""
enable control 连续型指令测试（摇杆式）

持续发送 /cmd_vel（前进 + 右转），循环 10 次 disable/enable，
记录 /odom 轨迹与速度，最后绘图输出。不做 pass/fail 判断。
"""
import argparse
import math
import sys
import threading

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from tf.transformations import euler_from_quaternion

import lb_ctrl_api as ct

# ── 参数 ──
N_CYCLES = 10
CMD_LINEAR_X = 0.4          # 前进速度 (m/s)
CMD_ANGULAR_Z = -0.1        # 右转角速度 (rad/s)
CMD_HZ = 50.0               # 连续发布 cmd_vel 的频率
SAMPLE_HZ = 20.0            # odom 采样频率
ENABLE_DURATION = 1.0       # 每个 enable 阶段持续时间 (s)
DISABLE_DURATION = 1.0      # 每个 disable 阶段持续时间 (s)
DEFAULT_OUT = '/home/wujing/kuavo-ros-control/.private/issue-390/continuous_odom.png'

RST = '\033[0m'; BLD = '\033[1m'
CYN = '\033[36m'; YEL = '\033[33m'; GRN = '\033[32m'


class Test:
    def __init__(self):
        rospy.init_node('enable_control_continuous_odom', anonymous=True)

        self._cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self._stop_pub = threading.Event()

        self.records = []   # [{t, x, y, yaw, vx, vy, wz}, ...]
        self.events = []    # [(t, label, x, y), ...]
        self.t0 = None

        print(f'{CYN}[init]{RST} 等待 /odom ...')
        self.odom = rospy.wait_for_message('/odom', Odometry, timeout=10.0)
        rospy.Subscriber('/odom', Odometry, self._odom_cb)

        self._init_robot()

    def _odom_cb(self, msg):
        self.odom = msg

    def _init_robot(self):
        print(f'{CYN}[init]{RST} 初始化机器人 ...')
        ct.reset_torso_to_initial()
        rospy.sleep(1.5)
        ct.set_arm_control_mode(1)
        rospy.sleep(1.5)
        ct.set_arm_control_mode(2)
        ct.set_focus_ee(True)
        ct.set_focus_z(False)
        ct.set_wheel_control_enable(True)
        rospy.sleep(1.0)

        print(f'{CYN}[init]{RST} 预留时间让机器人稳定回 HOME ...')
        rospy.sleep(3.0)           # 预留稳定时间
        self._wait_stable(timeout=5.0)
        print(f'{CYN}[init]{RST} ok\n')

    def _is_odom_stable(self):
        if self.odom is None:
            return False
        v = self.odom.twist.twist
        return (abs(v.linear.x) < 0.01 and
                abs(v.linear.y) < 0.01 and
                abs(v.angular.z) < 0.02)

    def _wait_stable(self, timeout=5.0):
        """等待 /odom 速度连续 0.5s 低于阈值；若超时则继续。"""
        stable_start = None
        t_start = rospy.Time.now().to_sec()
        rate = rospy.Rate(20.0)
        while rospy.Time.now().to_sec() - t_start < timeout:
            if self._is_odom_stable():
                now = rospy.Time.now().to_sec()
                if stable_start is None:
                    stable_start = now
                if now - stable_start >= 0.5:
                    return
            else:
                stable_start = None
            rate.sleep()
        print(f'  {YEL}警告{RST}: 未能在 {timeout}s 内完全稳定，继续测试')

    def _sample_odom(self):
        if self.odom is None:
            return None
        p = self.odom.pose.pose.position
        q = self.odom.pose.pose.orientation
        yaw = euler_from_quaternion([q.x, q.y, q.z, q.w], axes='szyx')[0]
        v = self.odom.twist.twist
        return {
            't': rospy.Time.now().to_sec() - self.t0,
            'x': p.x,
            'y': p.y,
            'yaw': yaw,
            'vx': v.linear.x,
            'vy': v.linear.y,
            'wz': v.angular.z,
        }

    def _cmd_vel_thread(self, twist):
        """持续发布 cmd_vel，直到收到停止信号。"""
        rate = rospy.Rate(CMD_HZ)
        while not self._stop_pub.is_set() and not rospy.is_shutdown():
            self._cmd_pub.publish(twist)
            rate.sleep()
        # 结束后发送若干次零速度
        zero = Twist()
        for _ in range(int(CMD_HZ * 0.4)):
            self._cmd_pub.publish(zero)
            rospy.sleep(1.0 / CMD_HZ)

    def _record_phase(self, duration):
        """按 SAMPLE_HZ 采样 odom，持续 duration 秒。"""
        rate = rospy.Rate(SAMPLE_HZ)
        t_start = rospy.Time.now().to_sec()
        while rospy.Time.now().to_sec() - t_start < duration:
            r = self._sample_odom()
            if r is not None:
                self.records.append(r)
            rate.sleep()

    def _record_event(self, label):
        """记录当前事件的时间和 odom 位置。"""
        t_event = rospy.Time.now().to_sec() - self.t0
        p = self.odom.pose.pose.position if self.odom else None
        self.events.append((
            t_event, label,
            p.x if p else None, p.y if p else None,
        ))
        return t_event

    def run(self, out_path):
        """主流程：持续 cmd_vel + 10 次 disable/enable。"""
        # 启动连续速度指令线程
        twist = Twist()
        twist.linear.x = CMD_LINEAR_X
        twist.angular.z = CMD_ANGULAR_Z

        cmd_thread = threading.Thread(target=self._cmd_vel_thread, args=(twist,))
        cmd_thread.start()

        self.t0 = rospy.Time.now().to_sec()
        self._record_event('start')
        print(f'{BLD}开始：持续发送 cmd_vel (vx={CMD_LINEAR_X}, wz={CMD_ANGULAR_Z}){RST}')
        print(f'{BLD}循环 {N_CYCLES} 次 disable/enable{RST}\n')

        for i in range(1, N_CYCLES + 1):
            if rospy.is_shutdown():
                break

            # enable 阶段：命令应该被响应，底盘运动
            ct.set_wheel_control_enable(True)
            t_event = self._record_event(f'enable #{i}')
            print(f'  {GRN}▶ enable #{i}{RST}  t={t_event:.2f}s')
            self._record_phase(ENABLE_DURATION)

            # disable 阶段：命令应被丢弃，底盘冻结
            ct.set_wheel_control_enable(False)
            t_event = self._record_event(f'disable #{i}')
            print(f'  {YEL}⏸ disable #{i}{RST} t={t_event:.2f}s')
            self._record_phase(DISABLE_DURATION)

        # 最后一次 enable，让机器人恢复正常可控状态
        ct.set_wheel_control_enable(True)
        t_event = self._record_event('final enable')
        print(f'\n  {GRN}▶ final enable{RST} t={t_event:.2f}s')

        # 停止命令线程
        self._stop_pub.set()
        cmd_thread.join()
        rospy.sleep(0.5)

        print(f'\n  共记录 {len(self.records)} 条 odom 数据')
        self._plot(out_path)
        return True

    def _plot(self, out_path):
        if not self.records:
            print('  无数据可绘图')
            return

        t = [r['t'] for r in self.records]
        x = [r['x'] for r in self.records]
        y = [r['y'] for r in self.records]

        fig, axes = plt.subplots(1, 2, figsize=(16, 7))

        # ── 左图：二维打点图 ──
        ax0 = axes[0]
        ax0.scatter(x, y, c='lightgray', s=3, alpha=0.5, label='odom')
        ax0.scatter(x[0], y[0], color='green', s=80, alpha=0.8, zorder=5, label='start')
        ax0.scatter(x[-1], y[-1], color='red', s=80, alpha=0.8, zorder=5, label='end')
        for et, label, ex, ey in self.events:
            if ex is None or ey is None:
                continue
            if 'disable' in label:
                ax0.scatter(ex, ey, color='red', s=60, marker='o', alpha=0.8,
                            edgecolors='darkred', linewidths=0.5, zorder=6)
            elif 'enable' in label or label == 'start':
                ax0.scatter(ex, ey, color='green', s=60, marker='o', alpha=0.8,
                            edgecolors='darkgreen', linewidths=0.5, zorder=6)

        xmin, xmax = min(x), max(x)
        ymin, ymax = min(y), max(y)
        span = max(xmax - xmin, ymax - ymin)
        pad = span * 0.1 if span > 1e-6 else 1.0
        cx = (xmin + xmax) / 2.0
        cy = (ymin + ymax) / 2.0
        half = span / 2.0 + pad
        ax0.set_xlim(cx - half, cx + half)
        ax0.set_ylim(cy - half, cy + half)
        ax0.set_aspect('equal', adjustable='box')
        ax0.set_xlabel('x (m)')
        ax0.set_ylabel('y (m)')
        ax0.set_title('odom x-y displacement (dot plot)')
        ax0.grid(True, alpha=0.3)
        ax0.legend()

        # ── 右图：时间-位移图 ──
        ax1 = axes[1]
        ax1.plot(t, x, color='blue', linewidth=1.2, label='x')
        ax1.plot(t, y, color='orange', linewidth=1.2, label='y')
        for et, label, _, _ in self.events:
            if 'disable' in label:
                ax1.axvline(et, color='red', linestyle=':', alpha=0.5)
            elif 'enable' in label or label == 'start':
                ax1.axvline(et, color='green', linestyle=':', alpha=0.5)
        ax1.plot([], [], color='red', linestyle=':', label='disable')
        ax1.plot([], [], color='green', linestyle=':', label='enable/start')
        ax1.set_xlabel('time (s)')
        ax1.set_ylabel('displacement (m)')
        ax1.set_title('x / y displacement vs time')
        ax1.grid(True, alpha=0.3)
        ax1.legend()

        plt.tight_layout()
        plt.savefig(out_path, dpi=150)
        print(f'  综合图已保存: {out_path}')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='enable control 连续 cmd_vel 测试')
    parser.add_argument('--output', '-o', type=str, default=DEFAULT_OUT,
                        help='输出图片的绝对路径')
    args = parser.parse_args()

    try:
        ok = Test().run(args.output)
        sys.exit(0 if ok else 1)
    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        print('\ninterrupted')
