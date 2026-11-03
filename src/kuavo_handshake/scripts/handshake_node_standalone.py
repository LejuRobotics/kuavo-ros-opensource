#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
下位机握手节点 — 独立版，零 SDK 依赖

启动:
  source /home/lab/kuavo-ros-control/devel/setup.bash
  roslaunch kuavo_handshake handshake_standalone.launch

测试:
  rostopic pub /handshake/target kuavo_msgs/HandshakeTarget \
    "{header: {stamp: now, frame_id: 'base_link'}, hand: 'right', wrist_x: 0.6, wrist_y: -0.25, wrist_z: 0.15}"

直接调用 ROS 服务/话题:
  /humanoid_change_arm_ctrl_mode → 切手臂控制模式
  /ik/two_arm_hand_pose_cmd_srv   → IK 求解
  /ik/fk_srv                      → FK 求解 (获取非主动手末端位姿)
  /sensor_data_motor/motor_pos    → 读当前关节角
  /kuavo_arm_traj                 → 下发关节轨迹

话题:
  /handshake/target (HandshakeTarget) — 上位机发布握手目标
  /kuavo_arm_traj (JointState) — 手臂轨迹发布
"""

import time
import math
import random
import threading
import os
import numpy as np
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, String
from kuavo_msgs.msg import HandshakeTarget, RobotActionState, twoArmHandPoseCmd
from kuavo_msgs.srv import twoArmHandPoseCmdSrv
from kuavo_msgs.srv import fkSrv
from kuavo_msgs.srv import changeArmCtrlMode, changeArmCtrlModeRequest
from scipy.spatial.transform import Rotation as R

# =============================================================================
# 常量
# =============================================================================

CONTROL_RATE = 50.0
APPROACH_DURATION = 1.5
RETRACT_DURATION = 1.5
ZERO_DURATION = 1.0
SHAKE_HOLD_DURATION = 2.0
TARGET_TIMEOUT = 1.0
ZERO_THRESHOLD_DEG = 10.0  # 双臂各关节偏离零位 ±10° 以内认为"在零位附近"
RL_CONTROLLER_SWITCH_TOPIC = "/humanoid_controller/nav_switch_rl_controller_by_name"
ARM_PLANNING_CONTROLLER = "amp_controller"
CONTROLLER_SWITCH_SETTLE_TIME = 0.5
ROBOT_ACTION_STATE_TOPIC = "/robot_action_state"
ROBOT_ACTION_FAILED_STATE = 0
ROBOT_ACTION_RUNNING_STATE = 1
ROBOT_ACTION_SUCCESS_STATE = 2
ROBOT_ACTION_STATE_PUBLISH_RATE_HZ = 10.0

DEFAULT_JOINTS_DEG = [
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
]

CLAMP_Y_RIGHT = -0.20
CLAMP_Y_LEFT  =  0.20
CLAMP_X_MIN = 0.35
CLAMP_X_MAX = 0.85

SHOULDER_POS_L = np.array([0.0, 0.29, 0.35])
SHOULDER_POS_R = np.array([0.0, -0.29, 0.35])


class _State:
    IDLE = 0
    APPROACHING = 1
    SHAKING = 2
    RETRACTING = 3


def _interpolate_bezier(start, end, num_points):
    start = np.array(start, dtype=float)
    end = np.array(end, dtype=float)
    t = np.linspace(0, 1, num_points)
    mid = (start + end) / 2
    offset = (end - start) * 0.1
    control = mid + offset
    result = []
    for i in range(num_points):
        pt = ((1 - t[i])**2 * start +
              2 * (1 - t[i]) * t[i] * control +
              t[i]**2 * end)
        result.append(pt.tolist())
    return result


class HandshakeNode:
    def __init__(self):
        rospy.init_node("handshake_node", log_level=rospy.INFO)

        # —— 机器人版本 → 手臂关节起始索引 ——
        try:
            raw = rospy.get_param("robot_version")
        except KeyError:
            raw = os.environ.get("ROBOT_VERSION", "46")
        robot_version = int(raw)
        self._arm_start_idx = 13 if 50 <= robot_version < 60 else 12
        rospy.loginfo("[Handshake] robot_version=%d arm_start_idx=%d",
                      robot_version, self._arm_start_idx)

        # —— 关节角 (从 motor_pos 订阅) ——
        self._joints_lock = threading.Lock()
        self._current_joints_deg = [0.0] * 14
        rospy.Subscriber("/sensor_data_motor/motor_pos", Float64MultiArray,
                         self._motor_cb, queue_size=1)

        # —— Arm 模式切换服务 ——
        rospy.wait_for_service("/humanoid_change_arm_ctrl_mode", timeout=5.0)

        # —— IK 服务 ——
        rospy.wait_for_service("/ik/two_arm_hand_pose_cmd_srv", timeout=5.0)
        self._ik_srv = rospy.ServiceProxy("/ik/two_arm_hand_pose_cmd_srv",
                                          twoArmHandPoseCmdSrv)

        # —— FK 服务 ——
        rospy.wait_for_service("/ik/fk_srv", timeout=5.0)
        self._fk_srv = rospy.ServiceProxy("/ik/fk_srv", fkSrv)

        # —— Arm 轨迹发布 ——
        self._arm_pub = rospy.Publisher("/kuavo_arm_traj", JointState, queue_size=10)

        # —— RL 控制器切换（后续手臂规划需使用 amp_controller）——
        self._controller_switch_pub = rospy.Publisher(
            RL_CONTROLLER_SWITCH_TOPIC, String, queue_size=1, latch=True
        )

        # —— 状态机 ——
        self._lock = threading.Lock()
        self._current_target: HandshakeTarget | None = None
        self._last_target_time: float = 0.0
        self._state: int = _State.IDLE
        self._state_start_time: float = 0.0
        self._active_hand: str = "right"
        self._target_shake_joints: list | None = None
        self._inactive_snapshot: list | None = None
        self._cooldown_until: float = 0.0
        self._robot_action_state: int = 0
        self._handshake_action_active: bool = False
        self._action_state_thread: threading.Thread | None = None
        self._action_state_pub = rospy.Publisher(
            ROBOT_ACTION_STATE_TOPIC, RobotActionState, queue_size=2
        )

        rospy.Subscriber(ROBOT_ACTION_STATE_TOPIC, RobotActionState,
                         self._robot_action_state_cb, queue_size=2)
        rospy.loginfo("[Handshake] Subscribed %s", ROBOT_ACTION_STATE_TOPIC)

        self._sub = rospy.Subscriber(
            "/handshake/target", HandshakeTarget, self._on_target, queue_size=1
        )

        rospy.loginfo("[Handshake] 初始化完成, 等待 /handshake/target ...")

    # =========================================================================
    # 手臂控制
    # =========================================================================

    def _set_arm_mode(self, mode):
        client = rospy.ServiceProxy("/humanoid_change_arm_ctrl_mode",
                                    changeArmCtrlMode)
        req = changeArmCtrlModeRequest()
        req.control_mode = mode
        resp = client(req)
        rospy.loginfo("[Handshake] Set Arm Mode %d: %s", mode, resp.result)
        return bool(resp.result)

    def _switch_to_arm_planning_controller(self):
        """切换到支持手臂规划的 amp_controller。"""
        self._controller_switch_pub.publish(String(data=ARM_PLANNING_CONTROLLER))
        rospy.loginfo("[Handshake] 切换控制器到 %s, 准备后续手臂规划",
                      ARM_PLANNING_CONTROLLER)
        rospy.sleep(CONTROLLER_SWITCH_SETTLE_TIME)

    def _motor_cb(self, msg: Float64MultiArray):
        data = list(msg.data)
        if len(data) < self._arm_start_idx + 14:
            return
        arm_rad = data[self._arm_start_idx:self._arm_start_idx + 14]
        with self._joints_lock:
            self._current_joints_deg = [math.degrees(a) for a in arm_rad]

    def _read_current_joints(self):
        with self._joints_lock:
            return list(self._current_joints_deg)

    def _robot_action_state_cb(self, msg):
        with self._lock:
            self._robot_action_state = int(getattr(msg, "state", 0) or 0)

    def _get_robot_action_state(self) -> int:
        with self._lock:
            return self._robot_action_state

    def _is_robot_action_running(self) -> bool:
        with self._lock:
            if self._handshake_action_active:
                return False
        return self._get_robot_action_state() == ROBOT_ACTION_RUNNING_STATE

    def _clear_current_target(self):
        with self._lock:
            self._current_target = None
            self._last_target_time = 0.0

    def _publish_robot_action_state_once(self, state: int):
        msg = RobotActionState()
        msg.state = state
        self._action_state_pub.publish(msg)

    def _start_handshake_action_state(self):
        with self._lock:
            if self._handshake_action_active:
                return
            self._handshake_action_active = True

        self._publish_robot_action_state_once(ROBOT_ACTION_RUNNING_STATE)
        self._action_state_thread = threading.Thread(
            target=self._publish_running_action_state,
            daemon=True,
        )
        self._action_state_thread.start()

    def _finish_handshake_action_state(self, state: int):
        with self._lock:
            was_active = self._handshake_action_active
            self._handshake_action_active = False

        if was_active:
            self._publish_robot_action_state_once(state)

    def _publish_running_action_state(self):
        rate = rospy.Rate(ROBOT_ACTION_STATE_PUBLISH_RATE_HZ)
        while not rospy.is_shutdown():
            with self._lock:
                active = self._handshake_action_active
            if not active:
                break
            self._publish_robot_action_state_once(ROBOT_ACTION_RUNNING_STATE)
            rate.sleep()

    # =========================================================================
    # 回调
    # =========================================================================

    def _on_target(self, msg: HandshakeTarget):
        msg_age = (rospy.Time.now() - msg.header.stamp).to_sec()
        if msg_age > 3.0:
            rospy.logwarn_throttle(2.0, "[Handshake] 丢弃过期消息 (%.1fs 前)", msg_age)
            return
        with self._lock:
            self._current_target = msg
            self._last_target_time = time.time()

    # =========================================================================
    # 主循环
    # =========================================================================

    def run(self):
        rate = rospy.Rate(CONTROL_RATE)
        while not rospy.is_shutdown():
            with self._lock:
                target = self._current_target
                last_time = self._last_target_time
            now = time.time()
            target_active = (target is not None
                             and (now - last_time <= TARGET_TIMEOUT)
                             and now >= self._cooldown_until)

            if self._state == _State.IDLE:
                if target_active and self._is_robot_action_running():
                    self._target_shake_joints = None
                    self._clear_current_target()
                    rospy.loginfo_throttle(
                        1.0,
                        "[Handshake] Robot action state=%d, skip arm planning",
                        self._get_robot_action_state(),
                    )
                    rate.sleep()
                    continue
                if target_active and self._target_shake_joints is None:
                    # —— 双臂不在零位附近时，切换控制器后直接执行握手规划 ——
                    if not self._arms_near_zero():
                        rospy.loginfo("[Handshake] 双臂不在零位附近, 切换控制器后直接握手")
                        self._switch_to_arm_planning_controller()
                    self._start_handshake_action_state()
                    if not self._update_target_joints(target):
                        self._finish_handshake_action_state(ROBOT_ACTION_FAILED_STATE)
                self._handle_idle(target_active, now)
            elif self._state == _State.APPROACHING:
                self._handle_approaching(now, target_active)
            elif self._state == _State.SHAKING:
                self._handle_shaking(now, target_active)
            elif self._state == _State.RETRACTING:
                self._handle_retracting(now, target_active)

            rate.sleep()

    # =========================================================================
    # 状态机
    # =========================================================================

    def _handle_idle(self, target_active: bool, now: float):
        if not target_active or self._target_shake_joints is None:
            return
        if self._is_robot_action_running():
            self._target_shake_joints = None
            self._clear_current_target()
            self._finish_handshake_action_state(ROBOT_ACTION_FAILED_STATE)
            rospy.loginfo_throttle(
                1.0,
                "[Handshake] Robot action state=%d before arm control, skip handshake",
                self._get_robot_action_state(),
            )
            return
        if not self._set_arm_mode(2):
            rospy.logwarn("[Handshake] 无法进入外部控制模式, 放弃本次握手")
            self._target_shake_joints = None
            self._clear_current_target()
            self._finish_handshake_action_state(ROBOT_ACTION_FAILED_STATE)
            return
        self._state = _State.APPROACHING
        self._state_start_time = now
        cd = self._read_current_joints()
        self._inactive_snapshot = cd[:7] if self._active_hand == "right" else cd[7:]
        rospy.loginfo("[Handshake] 开始接近 -> %s", self._active_hand)

    def _handle_approaching(self, now: float, target_active: bool):
        if self._target_shake_joints is None:
            return

        q_start = self._apply_inactive_snapshot(self._read_current_joints())
        q_target = self._apply_inactive_snapshot(self._target_shake_joints)

        n_pts = max(2, int(APPROACH_DURATION * CONTROL_RATE))
        traj = _interpolate_bezier(q_start, q_target, n_pts)
        for _ in range(5):
            traj.append(traj[-1][:])

        rospy.loginfo("[Handshake] 接近轨迹 %d 点, %.1fs", len(traj), APPROACH_DURATION)
        self._execute_traj(traj, APPROACH_DURATION)

        self._state = _State.SHAKING
        self._state_start_time = time.time()
        rospy.loginfo("[Handshake] 接近完成, 开始握持")

    def _handle_shaking(self, now: float, target_active: bool):
        if self._target_shake_joints is None:
            return

        hold_joints = self._apply_inactive_snapshot(self._target_shake_joints)
        n_pts = max(2, int(SHAKE_HOLD_DURATION * CONTROL_RATE))

        rospy.loginfo("[Handshake] 握持 %.1fs (%d 点 @ %.0fHz)",
                      SHAKE_HOLD_DURATION, n_pts, CONTROL_RATE)
        hold_rate = rospy.Rate(CONTROL_RATE)
        hold_start = time.time()
        for _ in range(n_pts):
            if rospy.is_shutdown():
                break
            self._publish_joints(hold_joints)
            hold_rate.sleep()
        rospy.loginfo("[Handshake] 握持完成, 实际耗时 %.1fs", time.time() - hold_start)

        self._start_retract(time.time())

    def _handle_retracting(self, now: float, target_active: bool):
        q_current = self._read_current_joints()
        q_default = self._apply_inactive_snapshot(DEFAULT_JOINTS_DEG)
        q_now_snapped = self._apply_inactive_snapshot(q_current)

        n_pts = max(2, int(RETRACT_DURATION * CONTROL_RATE))
        traj = _interpolate_bezier(q_now_snapped, q_default, n_pts)
        for _ in range(5):
            traj.append(traj[-1][:])

        rospy.loginfo("[Handshake] 撤回轨迹 %d 点, %.1fs", len(traj), RETRACT_DURATION)
        self._execute_traj(traj, RETRACT_DURATION)

        # 双臂回零, 再退出外部控制模式
        self._zero_arms()
        arm_mode_restored = self._set_arm_mode(1)

        self._state = _State.IDLE
        self._inactive_snapshot = None
        self._target_shake_joints = None
        self._cooldown_until = time.time() + 2.0
        self._finish_handshake_action_state(
            ROBOT_ACTION_SUCCESS_STATE if arm_mode_restored else ROBOT_ACTION_FAILED_STATE
        )
        rospy.loginfo("[Handshake] 收回完成, 冷却 2s")

    def _start_retract(self, now: float):
        self._state = _State.RETRACTING
        self._state_start_time = now
        rospy.loginfo("[Handshake] 开始收回")

    # =========================================================================
    # IK 求解 (ROS 服务)
    # =========================================================================

    def _update_target_joints(self, target: HandshakeTarget) -> bool:
        if self._is_robot_action_running():
            self._target_shake_joints = None
            self._clear_current_target()
            rospy.loginfo_throttle(
                1.0,
                "[Handshake] Robot action state=%d, skip IK planning",
                self._get_robot_action_state(),
            )
            return False

        robot_hand = target.hand
        self._active_hand = robot_hand

        base_pt = np.array([target.wrist_x, target.wrist_y, target.wrist_z])

        rospy.loginfo_throttle(1.0, "[Handshake] 目标 base_link=(%.3f %.3f %.3f) hand=%s",
                               base_pt[0], base_pt[1], base_pt[2], robot_hand)

        # —— 工作空间限位 ——
        y_was_clamped = False
        if robot_hand == "right":
            if base_pt[1] > CLAMP_Y_RIGHT:
                rospy.loginfo_throttle(1.0,
                    "[Handshake] 右手 Y=%.3f 跨中线, Y 定死=%.1f, X/Z 对齐人手",
                    base_pt[1], CLAMP_Y_RIGHT)
                base_pt[1] = CLAMP_Y_RIGHT
                y_was_clamped = True
        else:
            if base_pt[1] < CLAMP_Y_LEFT:
                rospy.loginfo_throttle(1.0,
                    "[Handshake] 左手 Y=%.3f 跨中线, Y 定死=%.1f, X/Z 对齐人手",
                    base_pt[1], CLAMP_Y_LEFT)
                base_pt[1] = CLAMP_Y_LEFT
                y_was_clamped = True

        base_pt[0] = np.clip(base_pt[0], CLAMP_X_MIN, CLAMP_X_MAX)

        # —— 握手姿态: pitch=-90°, twist=±15° ——
        WRIST_INWARD_DEG = 15.0
        PITCH_DEG = -90.0
        tw = WRIST_INWARD_DEG if robot_hand == "right" else -WRIST_INWARD_DEG
        base_rot = (R.from_euler("z", 0.0, degrees=True) *
                    R.from_euler("y", PITCH_DEG, degrees=True))
        twist = R.from_euler("z", tw, degrees=True)
        shake_rot = twist * base_rot
        ik_quat = shake_rot.as_quat().tolist()

        rospy.loginfo_throttle(1.0,
            "[Handshake] IK目标 pos=(%.3f %.3f %.3f) quat=[%.3f %.3f %.3f %.3f] hand=%s",
            base_pt[0], base_pt[1], base_pt[2],
            ik_quat[0], ik_quat[1], ik_quat[2], ik_quat[3], robot_hand)

        # —— 当前关节角作为 IK 种子 + FK 获取非主动手位姿 ——
        cd = self._read_current_joints()
        q0_l_rad_orig = [math.radians(a) for a in cd[:7]]
        q0_r_rad_orig = [math.radians(a) for a in cd[7:]]
        inactive_pos, inactive_quat = self._get_inactive_hand_pose(robot_hand, q0_l_rad_orig, q0_r_rad_orig)

        # —— 保存 clamp 后的原始目标位置，重试时基于此加扰动 ——
        base_pt_original = base_pt.copy()

        # —— 构建 IK 请求 (不变部分) ——
        req = twoArmHandPoseCmd()
        req.joint_angles_as_q0 = True
        req.ik_param.oritation_constraint_tol = 0.01
        req.ik_param.major_optimality_tol = 1e-4
        req.frame = 0

        # 填充非主动手 (固定不变)
        if robot_hand == "left":
            req.hand_poses.right_pose.pos_xyz = inactive_pos
            req.hand_poses.right_pose.quat_xyzw = inactive_quat
            req.hand_poses.left_pose.quat_xyzw = ik_quat
        else:
            req.hand_poses.left_pose.pos_xyz = inactive_pos
            req.hand_poses.left_pose.quat_xyzw = inactive_quat
            req.hand_poses.right_pose.quat_xyzw = ik_quat

        # —— 重试循环, 每次用不同扰动 ——
        MAX_IK_RETRIES = 5
        for attempt in range(1, MAX_IK_RETRIES + 1):
            if attempt == 1:
                # 第一次: 用原始值
                pert_pos = base_pt_original.tolist()
                pert_pos_np = base_pt_original
                q0_l = q0_l_rad_orig
                q0_r = q0_r_rad_orig
                req.ik_param.pos_constraint_tol = 1e-3
                req.ik_param.major_iterations_limit = 200
            else:
                # 重试: 限位和非限位策略不同
                if y_was_clamped:
                    # 限位: 偶数次变 X, 奇数(≥3)次变 Y, 幅度递增
                    step = attempt // 2
                    if attempt % 2 == 0:
                        x_offset = step * 0.02
                        y_offset = 0.0
                    else:
                        x_offset = 0.0
                        y_offset = -step * 0.02 if robot_hand == "right" else step * 0.02
                else:
                    # 非限位: 每次 X/Y 都变, 幅度递增
                    offset = (attempt - 1) * 0.02
                    x_offset = offset
                    y_offset = -offset if robot_hand == "right" else offset

                pert_pos_np = base_pt_original + np.array([x_offset, y_offset, 0.0])
                pert_pos_np[0] = np.clip(pert_pos_np[0], CLAMP_X_MIN, CLAMP_X_MAX)
                if robot_hand == "right":
                    if pert_pos_np[1] > CLAMP_Y_RIGHT:
                        pert_pos_np[1] = CLAMP_Y_RIGHT
                else:
                    if pert_pos_np[1] < CLAMP_Y_LEFT:
                        pert_pos_np[1] = CLAMP_Y_LEFT
                pert_pos = pert_pos_np.tolist()

                q0_l = [a + random.gauss(0, 0.009) for a in q0_l_rad_orig]
                q0_r = [a + random.gauss(0, 0.009) for a in q0_r_rad_orig]

                req.ik_param.pos_constraint_tol = 5e-3 if attempt == 2 else 1e-2
                req.ik_param.major_iterations_limit = 300 if attempt == 2 else 500

                rospy.loginfo("[Handshake] IK retry %d/%d pos=(%.3f %.3f %.3f) "
                              "tol=%.0e iters=%d",
                              attempt, MAX_IK_RETRIES,
                              pert_pos[0], pert_pos[1], pert_pos[2],
                              req.ik_param.pos_constraint_tol,
                              req.ik_param.major_iterations_limit)

            # 更新主动手可变字段
            if robot_hand == "left":
                req.hand_poses.left_pose.pos_xyz = pert_pos
                req.hand_poses.left_pose.elbow_pos_xyz = self._compute_elbow_target(pert_pos_np, "left")
                req.hand_poses.left_pose.joint_angles = q0_l
                req.hand_poses.right_pose.joint_angles = q0_r
            else:
                req.hand_poses.right_pose.pos_xyz = pert_pos
                req.hand_poses.right_pose.elbow_pos_xyz = self._compute_elbow_target(pert_pos_np, "right")
                req.hand_poses.right_pose.joint_angles = q0_r
                req.hand_poses.left_pose.joint_angles = q0_l

            try:
                resp = self._ik_srv(req)
                if resp.success:
                    if self._is_robot_action_running():
                        self._target_shake_joints = None
                        self._clear_current_target()
                        rospy.loginfo_throttle(
                            1.0,
                            "[Handshake] Robot action state=%d after IK, drop planning result",
                            self._get_robot_action_state(),
                        )
                        return False
                    jl = [math.degrees(a) for a in resp.hand_poses.left_pose.joint_angles]
                    jr = [math.degrees(a) for a in resp.hand_poses.right_pose.joint_angles]
                    if len(jl) == 7 and len(jr) == 7:
                        self._target_shake_joints = jl + jr
                        rospy.loginfo("[Handshake] IK 求解成功 hand=%s time=%.1fms attempt=%d/%d",
                                      robot_hand, resp.time_cost, attempt, MAX_IK_RETRIES)
                        return True
                    else:
                        rospy.logwarn("[Handshake] IK 返回关节数异常: L=%d R=%d (attempt=%d/%d)",
                                      len(jl), len(jr), attempt, MAX_IK_RETRIES)
                else:
                    rospy.logwarn("[Handshake] IK 求解失败: %s (attempt=%d/%d)",
                                  resp.error_reason, attempt, MAX_IK_RETRIES)
            except rospy.ServiceException as e:
                rospy.logerr("[Handshake] IK 服务调用失败: %s (attempt=%d/%d)",
                             e, attempt, MAX_IK_RETRIES)

            if attempt < MAX_IK_RETRIES:
                rospy.sleep(0.1)

        rospy.logerr("[Handshake] IK 求解失败, 已重试 %d 次, 放弃本次握手", MAX_IK_RETRIES)
        self._target_shake_joints = None
        self._clear_current_target()
        return False

    # =========================================================================
    # 工具函数
    # =========================================================================

    def _compute_elbow_target(self, hand_pos, side):
        """计算物理合理的肘部目标位置 (球面投影到大臂长度)"""
        hand_pos_np = np.array(hand_pos)
        if side == "left":
            shoulder = SHOULDER_POS_L
            offset = np.array([-0.15, 0.25, -0.20])
        else:
            shoulder = SHOULDER_POS_R
            offset = np.array([-0.15, -0.25, -0.20])

        mid = (shoulder + hand_pos_np) / 2.0
        rough = mid + offset
        vec = rough - shoulder
        dist = np.linalg.norm(vec)
        if dist < 1e-3:
            return shoulder.tolist()
        elbow = shoulder + (vec / dist) * 0.30
        return elbow.tolist()

    def _get_inactive_hand_pose(self, robot_hand, q0_l_rad, q0_r_rad):
        """调用 FK 服务获取非主动手当前末端位姿"""
        full_rad = q0_l_rad + q0_r_rad
        try:
            resp = self._fk_srv(full_rad)
            if resp.success:
                if robot_hand == "left":
                    pos = list(resp.hand_poses.right_pose.pos_xyz)
                    quat = list(resp.hand_poses.right_pose.quat_xyzw)
                else:
                    pos = list(resp.hand_poses.left_pose.pos_xyz)
                    quat = list(resp.hand_poses.left_pose.quat_xyzw)
                rospy.loginfo_throttle(5.0,
                    "[Handshake] FK inactive %s_hand pos=(%.3f %.3f %.3f)",
                    "left" if robot_hand == "right" else "right",
                    pos[0], pos[1], pos[2])
                return (pos, quat)
            else:
                rospy.logwarn("[Handshake] FK 求解失败: success=False")
        except rospy.ServiceException as e:
            rospy.logwarn("[Handshake] FK 服务调用异常: %s", e)
        return ([0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0])

    def _arms_near_zero(self) -> bool:
        """检查双臂所有关节是否在全零附近 (阈值 ZERO_THRESHOLD_DEG)"""
        current = self._read_current_joints()
        if len(current) < 14:
            rospy.logwarn("[Handshake] 关节数据不足14个, 认为不在零位")
            return False
        for i, val in enumerate(current):
            if abs(val) > ZERO_THRESHOLD_DEG:
                rospy.loginfo("[Handshake] 关节%d=%.1f° 超出阈值 ±%.0f°, 不在零位",
                              i, val, ZERO_THRESHOLD_DEG)
                return False
        return True

    def _zero_arms(self):
        """双臂回零: 从当前关节角余弦插值到默认零位"""
        current = self._read_current_joints()
        rospy.loginfo("[Handshake] 双臂回零 %s -> %s",
                       [round(v, 1) for v in current],
                       [round(v, 1) for v in DEFAULT_JOINTS_DEG])

        steps = max(1, int(ZERO_DURATION * CONTROL_RATE))
        rate = rospy.Rate(CONTROL_RATE)
        start_np = np.array(current, dtype=float)
        target_np = np.array(DEFAULT_JOINTS_DEG, dtype=float)

        for i in range(steps + 1):
            if rospy.is_shutdown():
                break
            t = i / float(steps)
            smooth_t = (1.0 - math.cos(math.pi * t)) / 2.0
            cmd = start_np + (target_np - start_np) * smooth_t
            self._publish_joints(cmd.tolist())
            rate.sleep()

        for _ in range(10):
            self._publish_joints(DEFAULT_JOINTS_DEG[:])
            rate.sleep()

        rospy.loginfo("[Handshake] 双臂回零完成")

    def _apply_inactive_snapshot(self, joints_14):
        if self._inactive_snapshot is None:
            return list(joints_14)
        j = list(joints_14)
        if self._active_hand == "right":
            return self._inactive_snapshot + j[7:]
        else:
            return j[:7] + self._inactive_snapshot

    def _publish_joints(self, joints_14):
        msg = JointState()
        msg.header.stamp = rospy.Time.now()
        msg.name = [f"j{i}" for i in range(1, 15)]
        msg.position = list(joints_14)
        msg.velocity = [0.0] * 14
        self._arm_pub.publish(msg)

    def _execute_traj(self, traj, duration):
        n = len(traj)
        if n < 2:
            if n == 1:
                self._publish_joints(traj[0])
            return
        dt = duration / n
        traj_rate = rospy.Rate(1.0 / dt)
        for pt in traj:
            if rospy.is_shutdown():
                break
            self._publish_joints(pt)
            traj_rate.sleep()

    def shutdown(self):
        rospy.loginfo("[Handshake] 节点关闭")


if __name__ == "__main__":
    node = HandshakeNode()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
