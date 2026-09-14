#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
MoRE / RL 仿真键盘遥控：发布 /cmd_vel（行走）、/cmd_pose（姿态）、motion_style、步态名，并切换 more_controller。

  /cmd_vel   : linear.x/y, angular.z（行走）
  /cmd_pose  : linear.z 下蹲, angular.y 弯腰 -> RlGaitReceiver -> MoRE posture_commands
  /more_motion_style_cmd : Int32 index 0/1/2 → 风格1/2/3 → [1,0,0]/[0,1,0]/[0,0,1]

参考 humanoid_interface_ros/scripts/joystickSimulator.py（键盘 + termios）
与 humanoid_controllers/scripts/vmp_switch_controller.py（切换控制器服务）。

用法:
  rosrun humanoid_interface_ros more_keyboard_control.py
  rosrun humanoid_interface_ros more_keyboard_control.py _auto_switch_more:=true

前提:
  - kuavo_v54 且 rl_controllers.yaml 中 more_controller enabled: true
  - 已启动 load_kuavo_mujoco_sim.launch（robot_version:=54）
"""

from __future__ import print_function

import select
import sys
import termios
import tty

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32, String

from kuavo_msgs.srv import getControllerList, switchController


# Int32 index → 风格 → motion_style_weights one-hot
MOTION_STYLE_LABELS = {
    0: ("风格1", [1, 0, 0], "pose"),
    1: ("风格2", [0, 1, 0], "walk_policy_arm"),
    2: ("风格3", [0, 0, 1], "walk_external_arm"),
}


class MoREKeyboardControl(object):
    """键盘发布速度 / 步态，并调用 switch_controller 切到 MoRE。"""

    def __init__(self):
        rospy.init_node("more_keyboard_control", anonymous=False)

        self.rate_hz = rospy.get_param("~rate", 50.0)
        self.lin_step = rospy.get_param("~lin_step", 0.05)
        self.ang_step = rospy.get_param("~ang_step", 0.08)
        self.max_lin_x = rospy.get_param("~max_lin_x", 1.25)
        self.max_lin_y = rospy.get_param("~max_lin_y", 0.5)
        self.max_ang_z = rospy.get_param("~max_ang_z", 1.0)
        self.height_step = rospy.get_param("~height_step", 0.02)
        self.pitch_step = rospy.get_param("~pitch_step", 0.03)
        self.min_height_z = rospy.get_param("~min_height_z", -0.30)
        self.max_height_z = rospy.get_param("~max_height_z", 0.0)
        self.min_pitch_y = rospy.get_param("~min_pitch_y", 0.0)
        self.max_pitch_y = rospy.get_param("~max_pitch_y", 0.5)
        self.walk_lin_x = rospy.get_param("~walk_lin_x", 0.35)
        self.more_controller_name = rospy.get_param("~more_controller_name", "more_controller")
        self.auto_switch_more = rospy.get_param("~auto_switch_more", False)
        self.style_command_topic = rospy.get_param(
            "~style_command_topic", "/more_motion_style_cmd"
        )
        self.cmd_pose_topic = rospy.get_param("~cmd_pose_topic", "/cmd_pose")

        self.cmd_vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        self.cmd_pose_pub = rospy.Publisher(self.cmd_pose_topic, Twist, queue_size=1)
        self.gait_pub = rospy.Publisher(
            "/humanoid_mpc_gait_name_request", String, queue_size=1, latch=True
        )
        self.style_pub = rospy.Publisher(
            self.style_command_topic, Int32, queue_size=1, latch=True
        )

        rospy.wait_for_service("/humanoid_controller/switch_controller", timeout=10.0)
        self._switch_controller = rospy.ServiceProxy(
            "/humanoid_controller/switch_controller", switchController
        )
        try:
            rospy.wait_for_service("/humanoid_controller/get_controller_list", timeout=2.0)
            self._get_controller_list = rospy.ServiceProxy(
                "/humanoid_controller/get_controller_list", getControllerList
            )
        except rospy.ROSException:
            self._get_controller_list = None

        self.vx = 0.0
        self.vy = 0.0
        self.vz = 0.0
        self.wz = 0.0
        self.pitch_y = 0.0
        self._gait_name = "stance"
        self._style_idx = 0
        self._fd = sys.stdin.fileno()
        self._old_term = termios.tcgetattr(self._fd)

        self._print_help()
        self.publish_motion_style(0)
        rospy.loginfo(
            "[more_keyboard] publish /cmd_vel + %s + %s + gait",
            self.cmd_pose_topic,
            self.style_command_topic,
        )
        if self.auto_switch_more:
            self.switch_controller(self.more_controller_name)

    def _print_help(self):
        print("\n======== MoRE 键盘遥控 ========")
        print("移动 (按住连续发 /cmd_vel，约 {} Hz):".format(self.rate_hz))
        print("  W / S     : 前 / 后  (linear.x)")
        print("  A / D     : 左 / 右  (linear.y)")
        print("  Q / E     : 左转 / 右转 (angular.z)")
        print("姿态 posture_commands（话题 {}）:".format(self.cmd_pose_topic))
        print("  I / K     : 起身 / 下蹲 (linear.z, {:.2f}~{:.2f} m, step={:.2f})".format(
            self.min_height_z, self.max_height_z, self.height_step))
        print("  U / O     : 抬头 / 弯腰 (angular.y, {:.2f}~{:.2f} rad, step={:.2f})".format(
            self.min_pitch_y, self.max_pitch_y, self.pitch_step))
        print("  空格      : 速度 + 姿态 清零")
        print("  Z         : 仅姿态清零（行走速度保留）")
        print("步态 (/humanoid_mpc_gait_name_request):")
        print("  C         : stance 站立")
        print("  R         : walk   行走")
        print("  T         : trot   小跑")
        print("MoRE motion_style ({}):".format(self.style_command_topic))
        print("  1         : 风格1 [1,0,0] pose + stance")
        print("  2         : 风格2 [0,1,0] walk + policy arm")
        print("  3         : 风格3 [0,0,1] walk + external arm")
        print("  R         : walk 步态 + 自动切 风格2 [0,1,0]")
        print("  G / H     : 清零前进速度 / 仅设 walk vx={:.2f}".format(
            self.walk_lin_x))
        print("控制器切换 (/humanoid_controller/switch_controller):")
        print("  M         : -> {}".format(self.more_controller_name))
        print("  P         : -> mpc")
        print("  Y         : -> amp_controller")
        print("  L         : 列出可用控制器")
        print("  Ctrl+C    : 退出\n")

    def _clamp(self, val, limit):
        return max(-limit, min(limit, val))

    def _clamp_height_z(self, val):
        return max(self.min_height_z, min(self.max_height_z, val))

    def _clamp_pitch_y(self, val):
        return max(self.min_pitch_y, min(self.max_pitch_y, val))

    def publish_cmd_vel(self):
        vel = Twist()
        vel.linear.x = self.vx
        vel.linear.y = self.vy
        vel.angular.z = self.wz
        self.cmd_vel_pub.publish(vel)

    def publish_cmd_pose(self):
        pose = Twist()
        pose.linear.z = self.vz
        pose.angular.y = self.pitch_y
        self.cmd_pose_pub.publish(pose)

    def publish_gait(self, gait_name):
        self._gait_name = gait_name
        msg = String()
        msg.data = gait_name
        self.gait_pub.publish(msg)
        rospy.loginfo("[more_keyboard] gait -> %s", gait_name)

    def publish_motion_style(self, style_idx):
        """发布 MoRE gate: index 0/1/2 → 风格1/2/3 → [1,0,0]/[0,1,0]/[0,0,1]."""
        style_idx = int(max(0, min(style_idx, 2)))
        self._style_idx = style_idx
        msg = Int32()
        msg.data = style_idx
        self.style_pub.publish(msg)
        label, one_hot, _ = MOTION_STYLE_LABELS[style_idx]
        rospy.loginfo("[more_keyboard] motion_style index=%d %s %s", style_idx, label, one_hot)

    def switch_controller(self, name):
        try:
            resp = self._switch_controller(controller_name=name)
            if resp.success:
                rospy.loginfo("[more_keyboard] switch_controller '%s' OK: %s", name, resp.message)
            else:
                rospy.logwarn("[more_keyboard] switch_controller '%s' failed: %s", name, resp.message)
            return resp.success
        except rospy.ServiceException as exc:
            rospy.logerr("[more_keyboard] switch_controller service error: %s", exc)
            return False

    def list_controllers(self):
        if self._get_controller_list is None:
            rospy.logwarn("get_controller_list service not available")
            return
        try:
            resp = self._get_controller_list()
            if not resp.success:
                rospy.logwarn("get_controller_list: %s", resp.message)
                return
            rospy.loginfo(
                "controllers (%d), current [%d] '%s':",
                resp.count,
                resp.current_index,
                resp.current_controller,
            )
            for i, n in enumerate(resp.controller_names):
                mark = " <--" if n == resp.current_controller else ""
                print("  [{}] {}{}".format(i, n, mark))
        except rospy.ServiceException as exc:
            rospy.logerr("get_controller_list: %s", exc)

    def _read_key(self, timeout):
        tty.setraw(self._fd)
        rlist, _, _ = select.select([sys.stdin], [], [], timeout)
        key = sys.stdin.read(1) if rlist else ""
        termios.tcsetattr(self._fd, termios.TCSADRAIN, self._old_term)
        return key

    def handle_key(self, key):
        if not key:
            return True
        if key == "\x03":
            return False

        k = key.lower()
        if k == "w":
            self.vx = self._clamp(self.vx + self.lin_step, self.max_lin_x)
        elif k == "s":
            self.vx = self._clamp(self.vx - self.lin_step, self.max_lin_x)
        elif k == "a":
            self.vy = self._clamp(self.vy + self.lin_step, self.max_lin_y)
        elif k == "d":
            self.vy = self._clamp(self.vy - self.lin_step, self.max_lin_y)
        elif k in ("q", "j"):
            self.wz = self._clamp(self.wz + self.ang_step, self.max_ang_z)
        elif k == "e":
            self.wz = self._clamp(self.wz - self.ang_step, self.max_ang_z)
        elif k == "i":
            # 起身：linear.z 增大（趋向 0）
            self.vz = self._clamp_height_z(self.vz + self.height_step)
        elif k == "k":
            # 下蹲：linear.z 减小（约 [-0.3, 0] m）
            self.vz = self._clamp_height_z(self.vz - self.height_step)
        elif k == "u":
            # 抬头：俯仰减小
            self.pitch_y = self._clamp_pitch_y(self.pitch_y - self.pitch_step)
        elif k == "o":
            # 弯腰：俯仰增大（约 [0, 0.5] rad）
            self.pitch_y = self._clamp_pitch_y(self.pitch_y + self.pitch_step)
        elif k == "z":
            self.vz = 0.0
            self.pitch_y = 0.0
        elif key == " ":
            self.vx = self.vy = self.vz = self.wz = 0.0
            self.pitch_y = 0.0
        elif k == "c":
            self.vx = self.vy = self.vz = self.wz = 0.0
            self.pitch_y = 0.0
            self.publish_gait("stance")
        elif k == "r":
            self.publish_motion_style(1)
            self.publish_gait("walk")
        elif k == "t":
            self.publish_gait("trot")
        elif k == "1":
            self.publish_motion_style(0)
            self.publish_gait("stance")
            self.vx = self.vy = self.wz = 0.0
        elif k == "2":
            self.publish_motion_style(1)
            self.publish_gait("walk")
        elif k == "3":
            self.publish_motion_style(2)
            self.publish_gait("walk")
        elif k == "g":
            self.vx = 0.0
        elif k == "h":
            self.publish_motion_style(1)
            self.publish_gait("walk")
            self.vx = self.walk_lin_x
        elif k == "m":
            self.switch_controller(self.more_controller_name)
        elif k == "p":
            self.switch_controller("mpc")
        elif k == "y":
            self.switch_controller("amp_controller")
        elif key == "L":
            self.list_controllers()
        else:
            return True

        label, one_hot, short = MOTION_STYLE_LABELS.get(self._style_idx, ("?", [0, 0, 0], "?"))
        print(
            "cmd [gait={} {} {}] vx={:+.2f} vy={:+.2f} vz={:+.2f} pitch={:+.2f} wz={:+.2f}".format(
                self._gait_name, label, one_hot, self.vx, self.vy, self.vz, self.pitch_y, self.wz
            ),
            end="\r",
        )
        sys.stdout.flush()
        return True

    def run(self):
        rate = rospy.Rate(self.rate_hz)
        try:
            while not rospy.is_shutdown():
                key = self._read_key(1.0 / self.rate_hz)
                if not self.handle_key(key):
                    break
                self.publish_cmd_vel()
                self.publish_cmd_pose()
                rate.sleep()
        finally:
            termios.tcsetattr(self._fd, termios.TCSADRAIN, self._old_term)
            zero = Twist()
            self.cmd_vel_pub.publish(zero)
            self.cmd_pose_pub.publish(zero)
            print("\n[more_keyboard] exit, cmd_vel and cmd_pose zeroed.")


def main():
    try:
        node = MoREKeyboardControl()
        node.run()
    except rospy.ROSInterruptException:
        pass


if __name__ == "__main__":
    main()
