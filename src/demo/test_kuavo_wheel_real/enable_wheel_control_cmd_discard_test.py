#!/usr/bin/env python3
"""
enable control 运动控制验证 —— 三种 enable/disable case 对比。

核心：验证 disable 瞬间冻结当前状态，disable 期间丢弃所有指令，
      enable 瞬间无 oneshot 恢复。

所有动作从 HOME 开始；只记录关节位置；三个 case 画成 3x3 子图。
"""
import argparse
import sys

import rospy
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
from kuavo_msgs.msg import sensorsData, robotHeadMotionData
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

import lb_ctrl_api as ct

# ── 颜色与输出 ──
RST, BLD = "\033[0m", "\033[1m"
CYN, YEL = "\033[36m", "\033[33m"

# ── 关节名称 ──
ARM_JOINT_NAMES = [f'joint{i}' for i in range(1, 15)]

# ── 时间参数 ──
T_SAMPLE = 0.1          # 采样周期：0.1s，让 0.5s 的 disable 能在采样点精确触发
T_CAPTURE = 3.5         # 每个 case 记录时长
T_RESET = 6.0           # 回 HOME 等待时间

# ── 运动指令相对时间 ──
# 以每个 case 的 t=0 为基准：0.0 身体，0.2 手，0.4 头，0.5 停止
TORSO_START = 0.0
ARM_START = 0.2
HEAD_START = 0.4
STOP_TIME = 0.5

MOVE_PROFILE = [
    (TORSO_START, 'torso_move'),
    (ARM_START,   'arm_move'),
    (HEAD_START,  'head_move'),
]

# ── 三个 case：只需指定 disable/enable 的相对时刻 ──
# BASELINE:    无 stop，完整执行 MOVE_PROFILE
# DISABLE_FIRST: 0s stop，指令组整体延迟 0.6s（0.6/0.8/1.0），验证 disable 期间指令被丢弃
# PAUSE_MID:   0.5s stop，指令组 0.0/0.2/0.4，验证中途冻结
CASES = [
    # (name, cmd_delay, disable_time, enable_time)
    ('BASELINE',                 0.0, None,      None),
    ('DISABLE_FIRST',            0.6, 0.0,       1.2),
    ('PAUSE_MID (0.5s disable)', 0.0, STOP_TIME, 1.0),
]

DEFAULT_OUT = '/home/wujing/kuavo-ros-control/.private/issue-390/three_case_position.png'

# ── 目标指令（写死） ──
ARM_HOME_DEG = [0.0] * 14
ARM_MOVE_DEG = [-30, 20, 15, -45, 25, 10, -35,
                -30, -20, -15, -45, -25, -10, -35]

TORSO_DZ = 0.15
TORSO_DPITCH = -0.08
TORSO_DYAW = 0.25

HEAD_MOVE_DEG = [20.0, -15.0]

EVENT_COLORS = {
    'torso_move': 'blue',
    'arm_move':   'orange',
    'head_move':  'purple',
    'disable':    'red',
    'enable':     'green',
}


class Test:
    def __init__(self):
        rospy.init_node('enable_control_motion_check', anonymous=True)
        self.sensors = rospy.wait_for_message('/sensors_data_raw', sensorsData, timeout=10.0)
        rospy.Subscriber('/sensors_data_raw', sensorsData, self._sensors_cb)

        self._arm_pub = rospy.Publisher('/kuavo_arm_traj', JointState, queue_size=10)
        self._torso_pub = rospy.Publisher('/cmd_lb_torso_pose', Twist, queue_size=10)
        self._head_pub = rospy.Publisher('/robot_head_motion_data', robotHeadMotionData, queue_size=10)
        rospy.sleep(0.5)

        self._init_robot()

    def _sensors_cb(self, msg):
        self.sensors = msg

    # ── 传感器读取 ──
    def _arm(self):
        if self.sensors is None:
            return None
        return self.sensors.joint_data.joint_q[4:18]

    def _lb(self):
        if self.sensors is None:
            return None
        return self.sensors.joint_data.joint_q[0:4]

    def _head(self):
        if self.sensors is None:
            return None
        return self.sensors.joint_data.joint_q[18:20]

    # ── 指令发送 ──
    def _send_arm(self, deg_list):
        m = JointState()
        m.header.stamp = rospy.Time.now()
        m.name = ARM_JOINT_NAMES
        m.position = deg_list
        m.velocity = [0.0] * 14
        m.effort = [0.0] * 14
        self._arm_pub.publish(m)

    def _send_torso(self, xyz_ypr):
        tw = Twist()
        tw.linear.x, tw.linear.y, tw.linear.z = xyz_ypr[:3]
        tw.angular.x = xyz_ypr[5]  # roll（无效）
        tw.angular.y = xyz_ypr[4]  # pitch
        tw.angular.z = xyz_ypr[3]  # yaw
        self._torso_pub.publish(tw)

    def _send_head(self, deg_list):
        m = robotHeadMotionData()
        m.joint_data = deg_list
        self._head_pub.publish(m)

    def _set_enable(self, enable):
        ok = ct.set_wheel_control_enable(enable)
        status = "ENABLE" if enable else "DISABLE"
        print(f"  {YEL}[{status}] {'ok' if ok else 'FAIL'}{RST}")
        return ok

    # ── 初始化 ──
    def _init_robot(self):
        print(f"{CYN}[init]{RST} ...")
        ct.reset_torso_to_initial()
        rospy.sleep(1.5)
        ct.set_arm_control_mode(2)
        ct.set_focus_z(False)
        self._set_enable(True)
        rospy.sleep(0.5)

        success, torso_pose = ct.get_torso_initial_pose(True)
        if not success or torso_pose is None:
            raise RuntimeError("无法获取躯干初始位姿")
        init_xyz = torso_pose['position']
        self.home_torso = [init_xyz[0], 0.0, init_xyz[2], 0.0, 0.0, 0.0]
        self.move_torso = self.home_torso.copy()
        self.move_torso[2] += TORSO_DZ
        self.move_torso[4] += TORSO_DPITCH
        self.move_torso[5] += TORSO_DYAW

        self._send_arm(ARM_HOME_DEG)
        self._send_torso(self.home_torso)
        self._send_head([0.0, 0.0])
        rospy.sleep(3.0)
        print(f"{CYN}[init]{RST} ok\n")

    def _reset_home(self):
        """回 HOME，按写死时间等待"""
        self._set_enable(True)
        self._send_arm(ARM_HOME_DEG)
        self._send_torso(self.home_torso)
        self._send_head([0.0, 0.0])
        print(f"  {CYN}→ 等待 HOME {T_RESET}s{RST}")
        rospy.sleep(T_RESET)

    def _build_events(self, cmd_delay, disable_time, enable_time):
        """把统一的运动 profile（可选延迟）与当前 case 的 disable/enable 合并成事件列表。"""
        events = [(t + cmd_delay, a) for t, a in MOVE_PROFILE]
        if disable_time is not None:
            events.append((disable_time, 'disable'))
        if enable_time is not None:
            events.append((enable_time, 'enable'))
        # 按时间排序；同一时刻命令在 enable/disable 之前（虽然 disable 会丢弃它们）
        events.sort(key=lambda x: (x[0], x[1] in ('disable', 'enable')))
        return events

    def _dispatch_action(self, name, t, action):
        """根据 action 发送对应指令。"""
        if action == 'torso_move':
            self._send_torso(self.move_torso)
        elif action == 'arm_move':
            self._send_arm(ARM_MOVE_DEG)
        elif action == 'head_move':
            self._send_head(HEAD_MOVE_DEG)
        elif action == 'disable':
            self._set_enable(False)
        elif action == 'enable':
            self._set_enable(True)
        print(f"    [{name}] t={t:.2f}s {action}")

    def _run_case(self, name, cmd_delay, disable_time, enable_time):
        """运行一个 case，返回位置记录与实际触发事件列表。"""
        events = self._build_events(cmd_delay, disable_time, enable_time)
        print(f"\n{BLD}CASE: {name}{RST}")
        print(f"  events: {events}")

        self._reset_home()

        rate = rospy.Rate(1.0 / T_SAMPLE)
        t0 = rospy.Time.now().to_sec()
        records = []
        actual_events = []
        next_event = 0

        print(f"  {YEL}→ 开始记录{RST}")
        while rospy.Time.now().to_sec() - t0 < T_CAPTURE:
            t = rospy.Time.now().to_sec() - t0

            while next_event < len(events) and events[next_event][0] <= t:
                et, action = events[next_event]
                self._dispatch_action(name, et, action)
                actual_events.append((et, action))
                next_event += 1

            arm = self._arm()
            lb = self._lb()
            head = self._head()
            if arm is None or lb is None or head is None:
                continue

            records.append({
                't': t,
                'arm': arm,
                'lb': lb,
                'head': head,
            })
            rate.sleep()

        if records:
            final = records[-1]
            print(f"  {CYN}→ case 结束 t={final['t']:.2f}s{RST}")
            print(f"      arm =[" + " ".join(f"{x:+.2f}" for x in final['arm']) + "]")
            print(f"      lb  =[" + " ".join(f"{x:+.3f}" for x in final['lb']) + "]")
            print(f"      head=[" + " ".join(f"{x:+.3f}" for x in final['head']) + "]")

        return records, actual_events

    # ── 绘图 ──
    @staticmethod
    def _slice(records, key, idx):
        return [r[key][idx] for r in records]

    def _plot_joint_groups(self, ax, records, key, n_joints, colors, label_prefix):
        """在 ax 上绘制一组关节位置曲线。"""
        t = [r['t'] for r in records]
        for ji in range(n_joints):
            ax.plot(t, self._slice(records, key, ji), color=colors[ji], linewidth=1.2,
                    alpha=0.8, label=f'{label_prefix}{ji}')
        ax.set_ylabel('joint position (rad)')
        ax.set_ylim(-1.0, 1.0)
        ax.grid(True, alpha=0.3)

    def _plot_three_cases(self, cases, out_path):
        """绘制 3x3 子图：每行一个 case，每列一个部位（手/身体/头）。
        事件线时间与代码中实际触发时间完全一致。"""
        if out_path is None:
            return

        fig, axes = plt.subplots(3, 3, figsize=(15, 10), sharex='col')

        arm_colors = plt.cm.tab20([i / 20.0 for i in range(14)])
        lb_colors = ['black', 'dimgray', 'gray', 'lightgray']
        head_colors = ['red', 'blue']

        col_titles = ['hand (arm)', 'body (lb)', 'head']

        for row, (name, recs, events) in enumerate(cases):
            # hand
            self._plot_joint_groups(axes[row, 0], recs, 'arm', 14, arm_colors, 'j')
            axes[row, 0].set_title(f'{name}: {col_titles[0]}')

            # body
            self._plot_joint_groups(axes[row, 1], recs, 'lb', 4, lb_colors, 'lb')
            axes[row, 1].set_title(f'{name}: {col_titles[1]}')

            # head
            self._plot_joint_groups(axes[row, 2], recs, 'head', 2, head_colors, 'head')
            axes[row, 2].set_title(f'{name}: {col_titles[2]}')

            # 事件标注
            self._add_event_lines(axes[row, 0], events)
            self._add_event_lines(axes[row, 1], events)
            self._add_event_lines(axes[row, 2], events)

        for col in range(3):
            axes[-1, col].set_xlabel('time (s)')

        # 统一图例
        handles = [plt.Line2D([0], [0], color=c, linestyle=':', label=a)
                   for a, c in EVENT_COLORS.items()]
        fig.legend(handles=handles, loc='upper center', ncol=5,
                   bbox_to_anchor=(0.5, 0.98), fontsize=9)

        plt.tight_layout(rect=[0, 0, 1, 0.96])
        plt.savefig(out_path, dpi=150)
        print(f"\n  图片已保存: {out_path}")

    @staticmethod
    def _add_event_lines(ax, events):
        """在 ax 上为所有事件画竖线。"""
        for et, action in events:
            ax.axvline(et, color=EVENT_COLORS[action], linestyle=':', alpha=0.7)

    # ── 主流程 ──
    def run(self, out_path):
        all_records = []
        for name, cmd_delay, disable_time, enable_time in CASES:
            recs, events = self._run_case(name, cmd_delay, disable_time, enable_time)
            all_records.append((name, recs, events))

        self._plot_three_cases(all_records, out_path)

        self._reset_home()
        ct.set_focus_z(True)
        print(f"\n{BLD}完成{RST}")
        return True


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='enable control 运动控制验证')
    parser.add_argument('--output', '-o', type=str, default=DEFAULT_OUT,
                        help='输出图片的绝对路径')
    args = parser.parse_args()

    try:
        ok = Test().run(args.output)
        sys.exit(0 if ok else 1)
    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        print("\ninterrupted")
