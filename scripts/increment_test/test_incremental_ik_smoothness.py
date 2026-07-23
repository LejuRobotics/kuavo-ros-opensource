#!/usr/bin/env python3
"""
增量 VR 遥操平滑性测试

轨迹（三段）：
  1) 直线进入工作空间
  2) YOZ 大圆
  3) YOZ 小圆
然后复位 → 自动出报告：
  - report.html            交互报告（SHM traj + 全链路 + timing）
  - arm_cmp/*.png          SHM / sensor / cmd + pipeline 静态图
  - arm_cmp/pipeline.md    增量→ArmTraj SHM 流程诊断

用法:
    python3 scripts/increment_test/test_incremental_ik_smoothness.py
    python3 scripts/increment_test/test_incremental_ik_smoothness.py --laps 20
    python3 scripts/increment_test/test_incremental_ik_smoothness.py --no-launch

监控: ArmTraj SHM（ctypes 只读），不订阅 /kuavo_arm_traj，避免 TCPROS 阻塞。
启动时不抢先 set TRANSPORT_SHM；由 IK 在进/出 mode2 时显式切链。

按 ROBOT_VERSION / --version 自动选启动文件:
  非 6x（#3095）: wheel_ik 关, control_torso:=false
  6x（#3198）  : wheel_ik:=true, control_torso:=false（完整 wheel_ik，非 arm_only）
  姿态（合成 bone）: use_incremental_hand_orientation:=false
    + VR 中性 quat → 指尖朝 +X、掌心朝 +Y
  结果: scripts/tmp 最多保留 KEEP_RUNS 次；arm_cmp.npz 录制侧 ≤ NPZ_MAX_MB 截断旧数据
"""

import argparse, json, math, os, shutil, signal, subprocess, sys, time, threading
from collections import defaultdict, deque
from datetime import datetime
import numpy as np

# ═══════════════════════════════════════════════════════════════════
# 可调参数（关节级 + 轨迹）—— 改这里即可
# ═══════════════════════════════════════════════════════════════════

# --- 轨迹 ---
CONFIG = {
    # 起点（VR bone 手掌位置，近似人体系相对胸）
    "start_left":  [0.25,  0.20, -0.50],
    "start_right": [0.25, -0.20, -0.50],
    # ① 零点→圆心；圆心→大圆出发点→大圆→回圆心；圆心→小圆出发点→小圆
    "workspace_delta": [0.20, 0.0, 0.20],
    "line_dur": 2.0,          # 各直线段时长 (s)
    # YOZ 平面画圆（圆心 = start + workspace_delta，x 固定）
    "r_large": 0.10,          # 大圆半径 (m)
    "r_small": 0.03,          # 小圆半径 (m)
    "circle_laps": 3,         # 大圆/小圆各自圈数（长测用 --laps 加大）
    "sec_per_lap": 2.5,       # 每圈时长 (s)；段时长 = laps * sec_per_lap
    "pause": 1.0,             # 回圆心后、小圆前停顿 (s)；保持 grip，不断开增量
}

# 结果保留
KEEP_RUNS = 3                 # scripts/tmp 下最多保留几次测试目录
NPZ_MAX_MB = 500              # arm_cmp 录制缓冲截断上限 (MB)，超了丢最旧样本

# --- 关节级 / IK 滤波与限幅（写入 /ik_ros_uni_cpp_node/quest3/*）---
JOINT_PARAMS = {
    "fhan_r": 3000.0,
    "fhan_kh0": 4.0,
    "fhan_r_joint": 3000.0,
    "fhan_kh0_joint": 4.0,
    "max_joint_velocity": 300.0,
    "delta_scale_x": 1.26,
    "delta_scale_y": 1.26,
    "delta_scale_z": 1.26,
    "delta_scale_roll": 1.0,
    "delta_scale_pitch": 1.0,
    "delta_scale_yaw": 1.0,
    "max_pos_diff": 0.45,
    "arm_move_threshold": 0.002,
    "task_space_acc_limit": 500.0,
    "task_space_jerk_limit": 3000.0,
    "joint_space_acc_limit": 500.0,
    "joint_space_jerk_limit": 3000.0,
    "pos_vel_limit": 30.0,
    "sphere_radius_limit": 0.6,
    "min_reachable_distance": 0.08,
    "elbow_min_distance": 0.16,
    "elbow_max_distance": 0.24,
    "chest_offset_y_ax": 0.0,
    # False：Abs 姿态；配合 _HAND_VR_QUAT → 指尖+X / 掌心±Y，只测位置增量。
    "use_incremental_hand_orientation": False,
    "hand_changing_mode_threshold": 1.2,
    "enable_wbc_arm_trajectory": True,
}

MODE_2_TIMEOUT = 5.5  # Quest3IkIncrementalROS mode2 超时 + margin

# ═══════════════════════════════════════════════════════════════════

try:
    import rospy
    from geometry_msgs.msg import Point, PoseStamped, Quaternion
    from kuavo_msgs.srv import changeArmCtrlMode, changeArmCtrlModeRequest
    from noitom_hi5_hand_udp_python.msg import JoySticks, PoseInfo, PoseInfoList
    from std_msgs.msg import Float64MultiArray, Int32
except ImportError as e:
    sys.exit(f"[FATAL] ROS 未就绪: {e}\nsource devel/setup.bash")

BONE_NAMES = [
    "LeftArmUpper", "LeftArmLower", "RightArmUpper", "RightArmLower",
    "LeftHandPalm", "RightHandPalm",
    "LeftHandThumbMetacarpal", "LeftHandThumbProximal", "LeftHandThumbDistal",
    "LeftHandThumbTip", "LeftHandIndexTip", "LeftHandMiddleTip",
    "LeftHandRingTip", "LeftHandLittleTip",
    "RightHandThumbMetacarpal", "RightHandThumbProximal", "RightHandThumbDistal",
    "RightHandThumbTip", "RightHandIndexTip", "RightHandMiddleTip",
    "RightHandRingTip", "RightHandLittleTip",
    "Root", "Chest",
]
IDX_L_SHOULDER, IDX_L_ELBOW = 0, 1
IDX_R_SHOULDER, IDX_R_ELBOW = 2, 3
IDX_L_HAND, IDX_R_HAND = 4, 5
IDX_CHEST = 23

# 机器人系「观测」目标：指尖朝 +X，掌心朝 +Y（左右同向；右手曾用 -Y 会反）。
# 实测 wheel Abs FK ≈ Rz(+90)·R_cmd，故下发 R_cmd = Rz(-90)·R_des。
_HAND_VR_QUAT = {
    "left":  (0.70105738, 0.09229596, -0.70105738, 0.09229596),
    "right": (0.09229596, -0.70105738, 0.09229596, 0.70105738),
}

_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
# 仓库根：脚本在 <ws>/scripts/；可用 KUAVO_WS 覆盖
_WS = os.environ.get("KUAVO_WS") or os.path.abspath(os.path.join(_SCRIPT_DIR, ".."))
_IK_LIB = os.path.join(_WS, "devel/lib/motion_capture_ik")


def resolve_robot_version(cli_version=None):
    if cli_version is not None:
        return int(cli_version)
    env = os.environ.get("ROBOT_VERSION")
    if env:
        return int(env)
    try:
        return int(rospy.get_param("/robot_version"))
    except Exception:
        pass
    return 45


def resolve_platform(robot_version, arm_only=False):
    """对齐 launch_quest3_ik.launch：按版本选 IK launch / 切模式服务 / 推荐仿真 launch。"""
    ver = int(robot_version)
    wheel_ik = (ver // 10) == 6
    if arm_only and ver != 62:
        print(f"[WARN] --arm-only 资产目前主要配齐 v62；当前 v{ver}")

    if arm_only:
        kind = "arm_only"
        binary_name = "wheel_arm_only_ik_ros_uni_cpp_node"
    elif wheel_ik:
        kind = "wheel_ik"
        binary_name = "wheel_ik_ros_uni_cpp_node"
    else:
        kind = "biped"
        binary_name = "ik_ros_uni_cpp_node"

    mpc_arm_svc = (
        "/wheel_arm_change_arm_ctrl_mode" if (wheel_ik or arm_only) else
        "/humanoid_change_arm_ctrl_mode"
    )

    # 合成 bone：Abs 姿态 → 指尖朝 +X、掌心朝左右 Y；位置仍增量；control_torso:=false。
    use_ori = False
    ik_launch_args = [
        "use_cpp_incremental_ik:=true",
        f"robot_version:={ver}",
        "use_incremental_hand_orientation:=false",
        "control_torso:=false",
    ]
    if wheel_ik and not arm_only:
        ik_launch_args.append("wheel_ik:=true")
    if arm_only:
        ik_launch_args.append("use_arm_only_ik:=true")

    if wheel_ik:
        sim_launch = "load_kuavo_mujoco_sim_wheel.launch"
        vr_launch = "load_kuavo_mujoco_sim_wheel_vr.launch"
    else:
        sim_launch = "load_kuavo_mujoco_sim.launch"
        vr_launch = "load_kuavo_mujoco_sim_with_vr.launch"

    return {
        "robot_version": ver,
        "wheel_ik": bool(wheel_ik),
        "arm_only": bool(arm_only),
        "kind": kind,
        "binary_name": binary_name,
        "binary": os.path.join(_IK_LIB, binary_name),
        "mpc_arm_svc": mpc_arm_svc,
        "vr_arm_svc": "/change_arm_ctrl_mode",
        "use_incremental_hand_orientation": use_ori,
        "control_torso": False,
        # 正常启动：roslaunch（与工单复现一致）
        "ik_package": "noitom_hi5_hand_udp_python",
        "ik_launch_file": "launch_quest3_ik.launch",
        "ik_launch_args": ik_launch_args,
        "run_cmd": [
            "roslaunch", "noitom_hi5_hand_udp_python", "launch_quest3_ik.launch",
            *ik_launch_args,
        ],
        "sim_package": "humanoid_controllers",
        "sim_launch_file": sim_launch,
        "vr_launch_file": vr_launch,
        "sim_run_cmd": [
            "roslaunch", "humanoid_controllers", sim_launch,
            f"robot_version:={ver}",
        ],
    }


class IncrementalIKTester:
    TOPICS = ["/ik_fk_result/left_ee_pose", "/ik_fk_result/right_ee_pose",
              "/ik_fk_result/left_ee_pose_filter"]

    def __init__(self, config, output_dir, platform, launch_ik=True, launch_sim=False):
        self.config = config
        self.output_dir = output_dir
        self.platform = platform
        self._launch_ik = launch_ik
        self._launch_sim = launch_sim
        self._ik_proc = None
        self._sim_proc = None
        os.makedirs(output_dir, exist_ok=True)

        self.records = defaultdict(list)
        self._lock = threading.Lock()
        self._t0 = None
        self._arm_mode = (None, None)   # MPC: biped [cur,req]；轮臂单元素扩成 (m,m)
        self._triger_mode = None        # /quest3/triger_arm_mode —— IK forceActivate 真正读这个
        self._stop_bg = threading.Event()
        self._bg_thread = None
        self._bg_left = self._bg_right = self._bg_joy = None
        self.arm_mode_pub = None
        self._shm_link_verified = False
        self._shm_link_active = False

        rospy.init_node("test_incremental_ik", anonymous=True, disable_signals=True)

    def startup(self):
        self._load_params()
        p = self.platform
        print(f"[SETUP] 平台 v{p['robot_version']} kind={p['kind']} "
              f"wheel_ik={p['wheel_ik']} arm_only={p['arm_only']}")
        print(f"[SETUP]   IK launch: {p['ik_package']}/{p['ik_launch_file']} {' '.join(p['ik_launch_args'])}")
        print(f"[SETUP]   推荐仿真: {p['sim_package']}/{p['sim_launch_file']}")
        print(f"[SETUP]   mpc_arm_svc={p['mpc_arm_svc']}  vr_arm_svc={p['vr_arm_svc']}")

        if self._launch_sim:
            print(f"[SETUP] 启动仿真: {' '.join(p['sim_run_cmd'])}")
            self._sim_proc = subprocess.Popen(
                p["sim_run_cmd"], stdout=subprocess.DEVNULL, stderr=subprocess.STDOUT,
                preexec_fn=os.setsid)
            time.sleep(15)
            if self._sim_proc.poll() is not None:
                sys.exit(f"[FATAL] 仿真 launch 立即退出: {p['sim_launch_file']}")
            print("[SETUP] 仿真已启动 (pid=%d)" % self._sim_proc.pid)

        if self._launch_ik:
            print(f"[SETUP] 启动 IK: {' '.join(p['run_cmd'])}")
            self._ik_proc = subprocess.Popen(
                p["run_cmd"], stdout=subprocess.DEVNULL, stderr=subprocess.STDOUT,
                preexec_fn=os.setsid)
            time.sleep(10)
            if self._ik_proc.poll() is not None:
                sys.exit(f"[FATAL] IK launch 立即退出: {p['ik_launch_file']}")
            print("[SETUP] IK launch 已启动 (pid=%d)" % self._ik_proc.pid)
        else:
            print("[SETUP] --no-launch：使用已有 IK；姿态参数需重启 IK 才生效")
        self._init_ros()
        if not self._wait_ready():
            sys.exit("[FATAL] IK 话题未就绪")

    def _kill_proc(self, proc, label):
        if not proc:
            return
        print(f"[CLEANUP] 关闭 {label} ...")
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGINT)
        except ProcessLookupError:
            pass
        try:
            proc.wait(timeout=12)
        except subprocess.TimeoutExpired:
            try:
                os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
            except ProcessLookupError:
                pass
            proc.wait()
        print(f"[CLEANUP] {label} 已释放")

    def shutdown(self):
        self._stop_bg_pub()
        self._kill_proc(self._ik_proc, "IK launch")
        self._ik_proc = None
        self._kill_proc(self._sim_proc, "仿真 launch")
        self._sim_proc = None

    def _load_params(self):
        p = self.platform
        rospy.set_param("/use_cpp_incremental_ik", True)
        rospy.set_param("/wheel_ik", bool(p["wheel_ik"]))
        rospy.set_param("/control_torso", bool(p.get("control_torso", False)))
        # ArmTraj SHM：WBC 可读；IK Writer（轮臂已合入）写 SHM
        rospy.set_param("/vr_ik/enable_incremental_arm_traj_link", True)
        rospy.set_param("/vr_ik/arm_traj_shm_stale_timeout_sec", 0.3)
        if self._is_wheel():
            rospy.set_param("/vr_ik/enable_solve_loop_timing_log", True)
        try:
            rospy.set_param("/robot_version", int(p["robot_version"]))
        except Exception:
            pass
        params = dict(JOINT_PARAMS)
        # 轮臂与 #3198 一致：关手部姿态增量；双足保持 JOINT_PARAMS 默认 true
        params["use_incremental_hand_orientation"] = bool(
            p.get("use_incremental_hand_orientation", True))
        for k, v in params.items():
            rospy.set_param("/quest3/%s" % k, v)
            rospy.set_param("/ik_ros_uni_cpp_node/quest3/%s" % k, v)
        print(f"[SETUP]   orientation={params['use_incremental_hand_orientation']} "
              f"control_torso={p.get('control_torso', False)}  shm_link=mode2_owned")

    def _is_wheel(self):
        p = self.platform
        return bool(p.get("wheel_ik") or p.get("arm_only") or p.get("kind") in ("wheel_ik", "arm_only"))

    def _set_arm_mode(self, mode):
        """MPC 切模式 + 发布 /quest3/triger_arm_mode（与 QuestControlFSM::callSetArmModeSrv 对齐）。"""
        p = self.platform
        mode = int(mode)
        ok_mpc = self._call_svc(p["mpc_arm_svc"], mode)
        ok_vr = self._call_svc(p["vr_arm_svc"], mode)
        # IK（人形/轮臂）订阅的是 Int32 /quest3/triger_arm_mode，不是 Float64MultiArray
        if self.arm_mode_pub is not None:
            self.arm_mode_pub.publish(Int32(data=mode))
        self._triger_mode = mode
        if self._is_wheel():
            # 轮臂 MPC 话题只发 1 元；服务成功即以 mode 作为本地状态
            if ok_mpc or ok_vr:
                self._arm_mode = (mode, mode)
        print(f"[TEST]   triger_arm_mode <- {mode}  (mpc_ok={ok_mpc} vr_ok={ok_vr})")
        rospy.sleep(0.2)
        return ok_mpc or ok_vr

    def _init_ros(self):
        self.bone_pub = rospy.Publisher(
            "/leju_quest_bone_poses", PoseInfoList, queue_size=10, latch=False)
        self.joy_pub = rospy.Publisher(
            "/quest_joystick_data", JoySticks, queue_size=10, latch=False)
        self.arm_mode_pub = rospy.Publisher(
            "/quest3/triger_arm_mode", Int32, queue_size=1, latch=True)
        rospy.Subscriber("/humanoid/mpc/arm_control_mode", Float64MultiArray,
                         self._on_arm_mode)
        rospy.Subscriber("/quest3/triger_arm_mode", Int32, self._on_triger_mode)
        for t in self.TOPICS:
            rospy.Subscriber(t, PoseStamped, self._recorder(t.rsplit("/", 1)[-1]))
        rospy.sleep(0.5)

    def _on_arm_mode(self, msg):
        # 人形: [current, requested]；轮臂 MobileManipulatorReferenceManager 只发 [current]
        if len(msg.data) >= 2:
            self._arm_mode = (int(msg.data[0]), int(msg.data[1]))
        elif len(msg.data) == 1:
            m = int(msg.data[0])
            self._arm_mode = (m, m)

    def _on_triger_mode(self, msg):
        self._triger_mode = int(msg.data)

    def _recorder(self, name):
        def cb(msg):
            with self._lock:
                t = rospy.Time.now().to_sec()
                if self._t0 is None:
                    self._t0 = t
                self.records[name].append({
                    "t": t - self._t0,
                    "x": msg.pose.position.x,
                    "y": msg.pose.position.y,
                    "z": msg.pose.position.z,
                    "qx": msg.pose.orientation.x,
                    "qy": msg.pose.orientation.y,
                    "qz": msg.pose.orientation.z,
                    "qw": msg.pose.orientation.w,
                })
        return cb

    def _wait_ready(self, timeout=40):
        print("[SETUP] 等待 IK 话题就绪 ...")
        topic = self.TOPICS[0]
        deadline = time.time() + timeout
        while time.time() < deadline and not rospy.is_shutdown():
            try:
                out = subprocess.check_output(
                    ["rostopic", "info", topic], stderr=subprocess.STDOUT,
                    timeout=3).decode()
                if "Publishers:" in out and " * /" in out:
                    print(f"[SETUP]   {topic} ✓")
                    return True
            except Exception:
                pass
            sys.stdout.write("."); sys.stdout.flush()
            rospy.sleep(0.5)
        print(f"\n[FATAL] {topic} 未就绪")
        return False

    # ── bone / joy ──

    def _pose(self, p, q=None):
        pi = PoseInfo()
        pi.position = Point(x=float(p[0]), y=float(p[1]), z=float(p[2]))
        if q is None:
            q = (0, 0, 0, 1)
        pi.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
        return pi

    def _bone(self, left_hand, right_hand):
        lh = np.asarray(left_hand, dtype=float)
        rh = np.asarray(right_hand, dtype=float)
        l_sh = np.array([0.0, 0.20, 0.15])
        r_sh = np.array([0.0, -0.20, 0.15])
        l_el = 0.5 * (l_sh + lh) + np.array([-0.05, 0.0, 0.0])
        r_el = 0.5 * (r_sh + rh) + np.array([-0.05, 0.0, 0.0])
        poses = [self._pose([0, 0, 0]) for _ in BONE_NAMES]
        poses[IDX_L_SHOULDER] = self._pose(l_sh)
        poses[IDX_L_ELBOW] = self._pose(l_el)
        poses[IDX_R_SHOULDER] = self._pose(r_sh)
        poses[IDX_R_ELBOW] = self._pose(r_el)
        poses[IDX_L_HAND] = self._pose(lh, _HAND_VR_QUAT["left"])
        poses[IDX_R_HAND] = self._pose(rh, _HAND_VR_QUAT["right"])
        poses[IDX_CHEST] = self._pose([0.0, 0.0, 0.0])
        poses[22] = self._pose([0.0, 0.0, -0.3])
        m = PoseInfoList()
        m.timestamp_ms = int(time.time() * 1000)
        m.is_high_confidence = True
        m.is_hand_tracking = True
        m.poses = poses
        return m

    def _joy(self, grip=False, x=False, a=False):
        m = JoySticks()
        if grip:
            m.left_grip = m.right_grip = 0.8
        m.left_first_button_pressed = bool(x)
        m.right_first_button_pressed = bool(a)
        return m

    def _pub(self, left, right, joy=None):
        self.bone_pub.publish(self._bone(left, right))
        self.joy_pub.publish(joy if joy is not None else self._joy())

    def _start(self, side):
        c = self.config
        return np.array(c["start_left"] if side == "left" else c["start_right"], dtype=float)

    def _workspace(self, side):
        return self._start(side) + np.asarray(self.config["workspace_delta"], dtype=float)

    def _lerp(self, a, b, t):
        t = max(0.0, min(1.0, t))
        return a + (b - a) * t

    def _yoz_circle(self, center, radius, progress, laps=2):
        """progress∈[0,1] 对应 laps 圈。"""
        a = progress * float(laps) * 2.0 * math.pi
        return center + np.array([0.0, radius * math.cos(a), radius * math.sin(a)])

    # ── 后台推送 ──

    def _start_bg_pub(self, left, right, joy=None):
        self._stop_bg_pub()
        self._bg_left = np.asarray(left, dtype=float)
        self._bg_right = np.asarray(right, dtype=float)
        self._bg_joy = joy if joy is not None else self._joy()
        self._stop_bg.clear()

        def loop():
            rate = rospy.Rate(50)
            while not self._stop_bg.is_set() and not rospy.is_shutdown():
                self._pub(self._bg_left, self._bg_right, self._bg_joy)
                rate.sleep()

        self._bg_thread = threading.Thread(target=loop, daemon=True)
        self._bg_thread.start()

    def _update_bg(self, left=None, right=None, joy=None):
        if left is not None:
            self._bg_left = np.asarray(left, dtype=float)
        if right is not None:
            self._bg_right = np.asarray(right, dtype=float)
        if joy is not None:
            self._bg_joy = joy

    def _stop_bg_pub(self):
        self._stop_bg.set()
        if self._bg_thread and self._bg_thread.is_alive():
            self._bg_thread.join(timeout=2)
        self._bg_thread = None

    # ── 模式切换 ──

    def _call_svc(self, name, mode, timeout=10):
        try:
            rospy.wait_for_service(name, timeout=timeout)
            resp = rospy.ServiceProxy(name, changeArmCtrlMode)(
                changeArmCtrlModeRequest(control_mode=mode))
            print(f"[TEST]   {name}({mode}) -> ok={resp.result}")
            return resp.result
        except Exception as e:
            print(f"[WARN]   {name}: {e}")
            return False

    def _wait_arm_mode(self, current, requested, timeout=30):
        """等待模式到位。轮臂看 MPC 单元素 + triger_arm_mode；人形看 [cur,req]。"""
        print(f"[TEST]   等待 arm_mode current={current} requested={requested} "
              f"(wheel={self._is_wheel()}) ...")
        deadline = time.time() + timeout
        while time.time() < deadline and not rospy.is_shutdown():
            cur = self._arm_mode
            trig = self._triger_mode
            if self._is_wheel():
                ok = (cur[0] == requested) and (trig == requested)
            else:
                ok = (cur == (current, requested)) and (trig in (None, requested) or trig == requested)
            if ok:
                print(f"[TEST]   arm_control_mode={cur} triger={trig} ✓")
                return True
            sys.stdout.write(f"\r  mpc={cur} triger={trig}  "); sys.stdout.flush()
            rospy.sleep(0.1)
        print(f"\n[WARN]   超时，mpc={self._arm_mode} triger={self._triger_mode}")
        return False

    def _reset_to_mode1(self, left, right):
        print("[TEST] Phase 0a: 复位 arm mode → 1")
        self._start_bg_pub(left, right, self._joy())
        self._set_arm_mode(1)
        ok = self._wait_arm_mode(1, 1, timeout=20)
        if not ok:
            self._set_arm_mode(1)
            ok = self._wait_arm_mode(1, 1, timeout=20)
        return ok

    def _press_xa(self, left, right):
        print("[TEST] 摇杆 X+A")
        self._stop_bg_pub()
        for _ in range(15):
            self._pub(left, right, self._joy(x=True, a=False)); rospy.sleep(0.02)
        for _ in range(15):
            self._pub(left, right, self._joy(x=True, a=True)); rospy.sleep(0.02)
        for _ in range(15):
            self._pub(left, right, self._joy()); rospy.sleep(0.02)
        self._start_bg_pub(left, right, self._joy())

    def _seg(self, name):
        if callable(getattr(self, "segment_marker", None)):
            self.segment_marker(name)

    # ── 主流程：三段轨迹 ──

    def run(self):
        c = self.config
        ls, rs = self._start("left"), self._start("right")
        wl, wr = self._workspace("left"), self._workspace("right")
        d = np.asarray(c["workspace_delta"], dtype=float)

        # ── 0a: 验证 SHM 链路 service 存在 ──
        self._verify_shm_link()

        print("[TEST] Phase 0: 预热 bone poses ...")
        self._seg("预热/进模式")
        for _ in range(50):
            self._pub(ls, rs, self._joy()); rospy.sleep(0.02)

        if not self._reset_to_mode1(ls, rs):
            print("[FATAL] 无法复位到 mode 1"); return

        self._press_xa(ls, rs)
        # 轮臂仿真可能没有 FSM 消费摇杆；显式服务+triger 进 mode2（与真机 X+A 效果对齐）
        if self._is_wheel():
            print("[TEST] 轮臂：显式切 mode 2（MPC 服务 + /quest3/triger_arm_mode）")
            self._set_arm_mode(2)
        print("[TEST] 等待 MPC/IK mode 2 ...")
        if not self._wait_arm_mode(2, 2, timeout=40):
            print("[FATAL] 未进入 mode 2"); return

        # ── 0b: mode2 后激活 SHM 链路 ──
        self._activate_shm_link()

        print(f"[TEST] 等待 mode2 超时 ({MODE_2_TIMEOUT:.1f}s) forceActivate ...")
        t0 = time.time()
        while time.time() - t0 < MODE_2_TIMEOUT and not rospy.is_shutdown():
            rospy.sleep(0.1)

        print("[TEST] grip ON")
        self._update_bg(joy=self._joy(grip=True))
        rospy.sleep(0.5)

        # ① 零点 → 圆心（大圆/小圆共用圆心）
        print(f"[TEST] ① 零点 → 圆心 Δ={d.tolist()} "
              f"L{wl.round(3).tolist()} R{wr.round(3).tolist()} ({c['line_dur']:.1f}s)")
        self._seg("①零点→圆心")
        self._drive_line(ls, rs, wl, wr, c["line_dur"], "零点→圆心")

        laps = int(c.get("circle_laps", 2))
        sec = float(c.get("sec_per_lap", 2.5))
        entry_l = self._yoz_circle(wl, c["r_large"], 0.0, laps=1)
        entry_r = self._yoz_circle(wr, c["r_large"], 0.0, laps=1)

        # 圆心 → 大圆出发点，再画圆（增量从上一段终点接着走）
        print(f"[TEST] ①→② 圆心 → 大圆出发点 ({c['line_dur']:.1f}s)")
        self._seg("①→②出大圆半径")
        self._drive_line(wl, wr, entry_l, entry_r, c["line_dur"], "圆心→大圆出发点")

        print(f"[TEST] ② YOZ 大圆 r={c['r_large']*1000:.0f}mm ×{laps}圈 ({laps*sec:.1f}s)")
        self._seg("②大圆YOZ")
        self._drive_yoz_circle(wl, wr, c["r_large"], laps, sec, "大圆YOZ")

        # 大圆终点(=出发点) → 圆心，再画小圆
        print(f"[TEST] ②→③ 大圆终点 → 圆心 ({c['line_dur']:.1f}s)")
        self._seg("②→③回圆心")
        self._drive_line(entry_l, entry_r, wl, wr, c["line_dur"], "回圆心")

        # 回圆心后短暂停顿（保持 grip，避免增量锚点重置导致后续小圆不动）
        if c["pause"] > 0:
            print(f"[TEST] 圆心停留 {c['pause']:.0f}s（grip 保持）...")
            self._seg("圆心停留")
            self._update_bg(left=wl, right=wr, joy=self._joy(grip=True))
            rospy.sleep(c["pause"])

        entry_s_l = self._yoz_circle(wl, c["r_small"], 0.0, laps=1)
        entry_s_r = self._yoz_circle(wr, c["r_small"], 0.0, laps=1)
        print(f"[TEST] ③a 圆心 → 小圆出发点 ({c['line_dur']:.1f}s)")
        self._seg("③a出小圆半径")
        self._drive_line(wl, wr, entry_s_l, entry_s_r, c["line_dur"], "圆心→小圆出发点")

        print(f"[TEST] ③ YOZ 小圆 r={c['r_small']*1000:.0f}mm ×{laps}圈 ({laps*sec:.1f}s)")
        self._seg("③小圆YOZ")
        self._drive_yoz_circle(wl, wr, c["r_small"], laps, sec, "小圆YOZ")

        self._seg("复位")
        self._reset_before_exit(entry_s_l, entry_s_r)
        if callable(getattr(self, "segment_ender", None)):
            self.segment_ender()

    def _drive_line(self, ol, or_, tl, tr, dur, label):
        ol, or_ = np.asarray(ol, float), np.asarray(or_, float)
        tl, tr = np.asarray(tl, float), np.asarray(tr, float)
        joy = self._joy(grip=True)
        t0 = time.time()
        while time.time() - t0 < dur and not rospy.is_shutdown():
            p = (time.time() - t0) / dur
            if p >= 1:
                break
            l, r = self._lerp(ol, tl, p), self._lerp(or_, tr, p)
            self._update_bg(left=l, right=r, joy=joy)
            self._pub(l, r, joy)
            if int(p * 100) % 20 == 0:
                sys.stdout.write(f"\r  [{label}] {p*100:.0f}%"); sys.stdout.flush()
            rospy.sleep(0.01)
        self._update_bg(left=tl, right=tr, joy=joy)
        self._pub(tl, tr, joy)
        print("")

    def _drive_yoz_circle(self, center_l, center_r, radius, laps, sec_per_lap, label):
        joy = self._joy(grip=True)
        laps = max(1, int(laps))
        dur = float(laps) * float(sec_per_lap)
        t0 = time.time()
        while time.time() - t0 < dur and not rospy.is_shutdown():
            p = (time.time() - t0) / dur
            if p >= 1:
                break
            l = self._yoz_circle(center_l, radius, p, laps=laps)
            r = self._yoz_circle(center_r, radius, p, laps=laps)
            self._update_bg(left=l, right=r, joy=joy)
            self._pub(l, r, joy)
            if int(p * 100) % 20 == 0:
                sys.stdout.write(f"\r  [{label}] {p*100:.0f}% ({p*laps:.1f}/{laps}圈)"); sys.stdout.flush()
            rospy.sleep(0.01)
        # 整数圈落回出发点，便于下一段从确定位置接着走
        l = self._yoz_circle(center_l, radius, 0.0, laps=1)
        r = self._yoz_circle(center_r, radius, 0.0, laps=1)
        self._update_bg(left=l, right=r, joy=joy)
        self._pub(l, r, joy)
        print("")

    # ── SHM 链路验证 ──

    def _resolve_shm_svc(self):
        """人形 /humanoid_controller/...  轮臂 /humanoid_wheel/..."""
        if self._is_wheel():
            return "/humanoid_wheel/set_incremental_arm_traj_link"
        return "/humanoid_controller/set_incremental_arm_traj_link"

    def _shm_svc_call(self, transport, timeout=5):
        """调用 SetIncrementalArmTrajLink，返回 (ok, msg, raw)。"""
        import subprocess, shlex
        ws = os.environ.get("KUAVO_WS") or os.path.abspath(
            os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
        setup = os.path.join(ws, "devel", "setup.bash")
        svc = self._resolve_shm_svc()
        cmd = (f"bash -c 'source {shlex.quote(setup)} && "
               f"rosservice call {shlex.quote(svc)} \"transport: {transport}\" 2>&1'")
        try:
            out = subprocess.check_output(cmd, shell=True, stderr=subprocess.STDOUT,
                                         timeout=timeout, cwd=ws).decode()
            ok = "success: True" in out
            msg = ""
            for line in out.splitlines():
                if "message:" in line:
                    msg = line.split("message:", 1)[-1].strip().strip('"')
            return ok, msg, out
        except Exception as e:
            return False, str(e), ""

    def _verify_shm_link(self):
        """验证 SHM 链路 service 存在且可调用。"""
        svc = self._resolve_shm_svc()
        print(f"[TEST] SHM 链路验证: {svc}")
        try:
            rospy.wait_for_service(svc, timeout=5)
            print(f"[TEST]   {svc} ✓ 已注册")
        except Exception as e:
            print(f"[WARN]   {svc} 未就绪: {e}")
            return

        # 确保默认状态
        ok, msg, _ = self._shm_svc_call(2)  # TRANSPORT_KUAVO_ARM_TRAJ
        print(f"[TEST]   默认 TRANSPORT_KUAVO_ARM_TRAJ: ok={ok} msg={msg}")

        # 切 SHM 再恢复，验证链路完整
        ok, msg, _ = self._shm_svc_call(1)  # TRANSPORT_SHM
        print(f"[TEST]   TRANSPORT_SHM: ok={ok} msg={msg}")
        rospy.sleep(0.3)
        ok, msg, _ = self._shm_svc_call(2)  # 恢复
        print(f"[TEST]   恢复 TRANSPORT_KUAVO_ARM_TRAJ: ok={ok} msg={msg}")
        self._shm_link_verified = True

    def _activate_shm_link(self):
        """进入 mode2 后切 SHM 传输。"""
        svc = self._resolve_shm_svc()
        ok, msg, _ = self._shm_svc_call(1)
        print(f"[TEST] SHM link ACTIVATE ({svc}): ok={ok} msg={msg}")
        self._shm_link_active = ok

    def _deactivate_shm_link(self):
        """退出增量前恢复 ROS topic。"""
        if not getattr(self, "_shm_link_active", False):
            return
        svc = self._resolve_shm_svc()
        ok, msg, _ = self._shm_svc_call(2)
        print(f"[TEST] SHM link DEACTIVATE ({svc}): ok={ok} msg={msg}")
        self._shm_link_active = False

    def _reset_before_exit(self, workspace_l, workspace_r):
        """直线退回起点 → 停 SHM 链路 → 松 grip → X+A → mode 1。"""
        c = self.config
        ls, rs = self._start("left"), self._start("right")
        print("[TEST] 复位：直线退回起点")
        self._drive_line(workspace_l, workspace_r, ls, rs, c["line_dur"], "直线复位")
        self._update_bg(left=ls, right=rs, joy=self._joy())
        rospy.sleep(0.5)

        # 退出增量前先停 SHM 链路，恢复 /kuavo_arm_traj ROS 订阅
        self._deactivate_shm_link()

        print("[TEST] X+A 退出 → mode 1")
        self._press_xa(ls, rs)
        if self._is_wheel():
            self._set_arm_mode(1)
        if not self._wait_arm_mode(1, 1, timeout=30):
            self._set_arm_mode(1)
            self._wait_arm_mode(1, 1, timeout=20)
        print("[TEST] 复位完成")
        self._update_bg(joy=self._joy())
        rospy.sleep(0.3)
        self._stop_bg_pub()

    # ── 分析 ──

    def analyze(self):
        print(f"\n{'='*60}\n[ANALYZE] 增量 IK 输出平滑性\n{'='*60}")
        for k in ["left_ee_pose", "left_ee_pose_filter", "right_ee_pose"]:
            recs = self.records.get(k, [])
            if len(recs) < 10:
                print(f"\n  {k}: 数据不足 (n={len(recs)})"); continue
            self._analyze(k, recs)
        self._save()

    def _analyze(self, name, recs):
        t = np.array([r["t"] for r in recs])
        x = np.array([r["x"] for r in recs])
        y = np.array([r["y"] for r in recs])
        z = np.array([r["z"] for r in recs])
        dt = np.diff(t); dt[dt == 0] = 1e-9
        v = np.sqrt(np.diff(x)**2 + np.diff(y)**2 + np.diff(z)**2) / dt * 1000
        dv = np.abs(np.diff(v))
        th = np.percentile(dv, 95) * 1.5
        step = np.sum(dv > th) / max(len(dv), 1) * 100
        zero = np.sum(v < 0.5) / max(len(v), 1) * 100
        issues = []
        if step > 5:
            issues.append(f"阶梯{step:.1f}%")
        if zero > 15:
            issues.append(f"停顿{zero:.1f}%")
        flag = "⚠ " + "; ".join(issues) if issues else "✓ 平滑"
        print(f"\n  {name}: n={len(recs)} t={t[-1]-t[0]:.1f}s "
              f"v={np.mean(v):.0f}±{np.std(v):.0f}mm/s "
              f"step={step:.1f}% zero={zero:.1f}% → {flag}")

    def _save(self):
        data = {}
        for k, recs in self.records.items():
            if recs:
                data[f"{k}_t"] = np.array([r["t"] for r in recs])
                data[f"{k}_x"] = np.array([r["x"] for r in recs])
                data[f"{k}_y"] = np.array([r["y"] for r in recs])
                data[f"{k}_z"] = np.array([r["z"] for r in recs])
        np.savez(os.path.join(self.output_dir, "ik_output.npz"), **data)
        jp = dict(JOINT_PARAMS)
        jp["use_incremental_hand_orientation"] = bool(
            self.platform.get("use_incremental_hand_orientation", True))
        jp["control_torso"] = bool(self.platform.get("control_torso", False))
        json.dump({"ts": datetime.now().isoformat(), "config": self.config,
                   "joint_params": jp,
                   "platform": {k: self.platform[k] for k in (
                       "robot_version", "wheel_ik", "arm_only", "kind",
                       "use_incremental_hand_orientation", "control_torso",
                       "ik_launch_args") if k in self.platform},
                   "arm_mode_final": list(self._arm_mode) if self._arm_mode[0] is not None else None,
                   "n": {k: len(v) for k, v in self.records.items()}},
                  open(os.path.join(self.output_dir, "meta.json"), "w"), indent=2)
        print(f"\n数据: {self.output_dir}/ik_output.npz")


def prune_run_dirs(output_root, keep=KEEP_RUNS):
    """scripts/tmp 下只保留最近 keep 个测试目录（按 mtime）。"""
    keep = max(1, int(keep))
    if not os.path.isdir(output_root):
        return
    dirs = []
    for name in os.listdir(output_root):
        path = os.path.join(output_root, name)
        if os.path.isdir(path):
            dirs.append(path)
    dirs.sort(key=lambda p: os.path.getmtime(p), reverse=True)
    for old in dirs[keep:]:
        try:
            shutil.rmtree(old)
            print(f"[CLEAN] 删除旧结果: {old}")
        except OSError as e:
            print(f"[WARN] 无法删除 {old}: {e}")


def main():
    p = argparse.ArgumentParser()
    p.add_argument("-o", "--output", default=os.path.join(os.path.dirname(os.path.abspath(__file__)), "tmp"),
                   help="输出根目录 (默认 scripts/tmp)")
    p.add_argument("--label", default=None)
    p.add_argument("--no-launch", action="store_true")
    p.add_argument("--version", type=int, default=None,
                   help="ROBOT_VERSION，默认读环境变量 /robot_version")
    p.add_argument("--arm-only", action="store_true",
                   help="非工单路径：use_arm_only_ik:=true（v62 半身）。#3198 默认是 wheel_ik，勿用此项")
    p.add_argument("--launch-sim", action="store_true",
                   help="按版本自动 roslaunch 仿真（非6x mujoco_sim / 6x mujoco_sim_wheel）")
    p.add_argument("--laps", type=int, default=None,
                   help="大圆/小圆各自圈数（覆盖 CONFIG.circle_laps，长测例如 50）")
    p.add_argument("--sec-per-lap", type=float, default=None,
                   help="每圈秒数（覆盖 CONFIG.sec_per_lap）")
    p.add_argument("--keep-runs", type=int, default=KEEP_RUNS,
                   help=f"最多保留几次测试结果目录（默认 {KEEP_RUNS}）")
    p.add_argument("--npz-max-mb", type=float, default=NPZ_MAX_MB,
                   help=f"录制缓冲截断上限 MB（默认 {NPZ_MAX_MB}）")
    a = p.parse_args()

    c = dict(CONFIG)
    if a.laps is not None:
        c["circle_laps"] = max(1, int(a.laps))
    if a.sec_per_lap is not None:
        c["sec_per_lap"] = max(0.1, float(a.sec_per_lap))
    ver = resolve_robot_version(a.version)
    platform = resolve_platform(ver, arm_only=a.arm_only)
    label = a.label or datetime.now().strftime("%Y%m%d_%H%M%S")
    out = os.path.join(a.output, f"{label}_v{ver}_{platform['kind']}_r{int(c['r_small']*1000)}mm")

    tester = IncrementalIKTester(
        c, out, platform, launch_ik=not a.no_launch, launch_sim=a.launch_sim)
    plot_rec = plot_meta = None

    def _s(*_):
        tester.shutdown(); sys.exit(1)

    signal.signal(signal.SIGINT, _s)
    signal.signal(signal.SIGTERM, _s)
    try:
        tester.startup()
        # 录制：ArmTraj SHM + 增量中间链路（不订阅 /kuavo_arm_traj）
        sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
        import plot_arm_traj_sensor_cmd as arm_plot
        n_arm, arm_offset, _, plot_meta = arm_plot.load_layout(ver)
        plot_meta["platform"] = {k: platform[k] for k in (
            "robot_version", "wheel_ik", "arm_only", "kind", "binary_name",
            "mpc_arm_svc", "vr_arm_svc")}
        plot_meta["circle_laps"] = c["circle_laps"]
        plot_meta["sec_per_lap"] = c["sec_per_lap"]
        plot_meta["traj_source"] = "ArmTraj SHM"
        plot_meta["shm_svc_name"] = ("/humanoid_wheel/set_incremental_arm_traj_link"
                                     if tester._is_wheel()
                                     else "/humanoid_controller/set_incremental_arm_traj_link")
        plot_rec = arm_plot.ArmCmpRecorder(
            n_arm, arm_offset, max_bytes=int(float(a.npz_max_mb) * 1024 * 1024))
        plot_rec.subscribe_all()
        # SHM 由 IK 在进/出 mode2 时显式切链（方案 A），测试侧只监控 SHM，不抢先 setLink
        print("[PLOT] SHM link owned by IK mode2 enter/exit (test does not set TRANSPORT_SHM)")
        tester.segment_marker = plot_rec.mark_segment
        tester.segment_ender = plot_rec.end_segment
        print(f"[PLOT] 录制中 arm_n={n_arm} offset={arm_offset} "
              f"({'wheel-like' if plot_meta.get('wheel_like') else 'biped'}) "
              f"npz_cap={a.npz_max_mb:.0f}MB  source=SHM")
        print(f"[TEST] circle_laps={c['circle_laps']} sec_per_lap={c['sec_per_lap']} "
              f"→ 每段 {c['circle_laps']*c['sec_per_lap']:.1f}s")
        tester.run()
    finally:
        if plot_rec is not None:
            try:
                plot_rec.stop()
            except Exception:
                pass
        tester.shutdown()
    tester.analyze()

    if plot_rec is not None:
        data = plot_rec.snapshot()
        counts = plot_rec.pipeline_counts()
        segments = plot_rec.get_segments()
        plot_meta["segments"] = segments
        plot_meta["shm_stats"] = getattr(plot_rec, "shm_stats", {})
        plot_meta["shm_link_verified"] = getattr(tester, "_shm_link_verified", False)
        plot_meta["shm_link_active"] = getattr(tester, "_shm_link_active", False)
        plot_dir = os.path.join(out, "arm_cmp")
        arm_plot.save_npz(os.path.join(plot_dir, "arm_cmp.npz"), data, plot_meta)
        arm_plot.plot_compare(data, plot_meta, plot_dir)
        report = arm_plot.write_html_report(
            data, plot_meta, out, arm_cmp_rel="arm_cmp", counts=counts,
            shm_stats=plot_meta.get("shm_stats"))
        print(f"[PLOT] 对比图+pipeline: {plot_dir}")
        print(f"[PLOT] pipeline counts: {counts}")
        print(f"[PLOT] shm_stats: {plot_meta.get('shm_stats')}")
        print(f"[PLOT] segments: {segments}")
        print(f"[PLOT] HTML 报告: {report}")

    prune_run_dirs(a.output, keep=a.keep_runs)


if __name__ == "__main__":
    main()
