#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""SG100 拳师手套 -> 内置 Linux SG100 SDK -> ROS 轻量桥（每手一个实例）。

本节点不含任何手套数据处理算法。串口解码、PoseTracker 姿态消歧、
板卡路由和 SG100 IK 全部由包内 sg100_runtime/ 的 Linux SDK
(Python 3.12 + Pinocchio 4.0.0) 完成。

左右手套可各运行一个本节点实例，各自持有独立串口接收器与 SDK 子进程
（side=left / side=right），互不共享串口；也可在单接收器同时收到左右
双手数据时运行一个实例（side=both，SDK 以 --hand both 启动，
左右板卡 ID 经 --left-id / --right-id 显式配置）。

节点职责：
  1. 读取 ROS 参数 (~side 支持 left/right/both; 自动模式先排除系统
     保留串口，再验证目标板卡 ID，失败时不启动 SDK);
  2. 解析 Python 3.12 解释器 (~sdk_python > 环境变量 SG100_PYTHON >
     包内 sg100_runtime/.venv_sg100_native);
  3. subprocess 启动 sg100_runtime/run_sg100_native.sh (exec 内置
     sg100_live_native.py)，清理 ROS 注入的 PYTHONPATH/PYTHONHOME/
     LD_LIBRARY_PATH，避免 Pinocchio 导入冲突;
  4. 读取 SDK 子进程 stdout:
     - J 行: 单手 "J seq q0..q10" (13 字段), 双手 (side=both)
       "J hand board_id seq q0..q10" (15 字段, SDK --hand both);
       校验 11 个有限浮点数后发布仿真关节话题, std_msgs/
       Float32MultiArray, rad, 直接透传 SDK 输出, 不叠加 EMA/deadband;
       右手发 ~command_topic, 左手发 ~left_command_topic
       （side=both 时按 hand 字段分发，互不写入对方话题）;
     - P 行 ("P hand board_id seq <35 数>", 需 SDK --emit-gesture-poses)
       -> /qianshi/glove_{left,right}/fingertips, 35 维五指位姿
       (thumb/index/middle/ring/pinky x [x,y,z,qw,qx,qy,qz], 位置 m,
       四元数 WXYZ)。注意: 这是五指电磁传感器模块经 PoseTracker 消歧后
       的跟踪位姿, 不是人体指尖 FK 结果; side=both 时按 hand 字段
       分发到左右两个话题;
  5. 消费 stderr 诊断并转发到 ROS 日志;
  6. SDK 子进程异常退出时重新验证串口; ROS shutdown 时终止并回收子进程。

关节顺序 (与 SDK JOINT_ORDER 一致)：
  thumb_j1 thumb_j2 thumb_j3
  index_j1 index_j2 index_j3
  middle_j1 middle_j2
  little_j1 little_j2 little_j3
"""

import json
import math
import os
import re
import shlex
import stat
import subprocess
import threading

import rospy
import rospkg
from std_msgs.msg import Bool, Float32MultiArray

try:
    from kuavo_msgs.msg import SG100HandCommand
except ImportError:
    SG100HandCommand = None

# 单手行协议: "J seq q0 ... q10" -> 1 + 1 + 11 = 13 个字段
JOINT_COUNT = 11
FIELDS_PER_LINE = 2 + JOINT_COUNT
# 双手行协议 (side=both, SDK --hand both):
# "J hand board_id seq q0 ... q10" -> 1 + 3 + 11 = 15 个字段
BOTH_FIELDS_PER_LINE = 4 + JOINT_COUNT

# P 行 (五指位姿): "P hand board_id seq <35 数>" -> 4 + 35 = 39 个字段
POSE_COUNT = 35
POSE_FIELDS_PER_LINE = 4 + POSE_COUNT

SUPPORTED_SIDES = ("left", "right", "both")

_RECEIVER_SCAN_SCRIPT = r"""
import json
import sys

sys.path.insert(0, sys.argv[1])
from sg100_native.connection import scan_boards

baud = int(sys.argv[2])
seconds = float(sys.argv[3])
wanted = {int(item) for item in sys.argv[4].split(",") if item}
preferred = sys.argv[5]
devices = sys.argv[6:]

matches = []
reports = []
for device in devices:
    try:
        rows = scan_boards(device, baud, seconds)["boards"]
        ids = [int(row["board_id"]) for row in rows]
        error = ""
    except Exception as exc:
        ids = []
        error = str(exc)
    reports.append({"device": device, "ids": ids, "error": error})
    if wanted and wanted.issubset(set(ids)):
        matches.append(device)
        if device == preferred:
            break

chosen = matches[0] if len(matches) == 1 else None
print(json.dumps({"chosen": chosen, "matches": matches, "reports": reports}))
"""

_DEFAULT_RESERVED_PATHS = (
    "/dev/lefthand", "/dev/righthand", "/dev/usb_servo",
    "/dev/power_board", "/dev/usb_remote",
)

_KNOWN_HAND_SERIALS = {"A10RA5IC", "A10RA5IA", "A10RABMP"}


def _clean_env():
    env = dict(os.environ)
    for key in ("PYTHONPATH", "PYTHONHOME", "LD_LIBRARY_PATH"):
        env.pop(key, None)
    return env


def _permission_denied(text):
    text = text or ""
    return "Permission denied" in text or "Errno 13" in text


def _board_id(value, param):
    if (type(value) not in (int, str) or not re.fullmatch(r"[0-9]{1,10}", str(value))
            or int(value) > 0xFFFFFFFF):
        raise ValueError("%s 必须是无符号 32 位整数板卡 ID" % param)
    return int(value)


def _dialout_argv(argv):
    return ["sg", "dialout", "-c", " ".join(shlex.quote(str(item)) for item in argv)]


def _port_identity(port):
    """Read only sysfs identity; never open a serial device during enumeration."""
    name = os.path.basename(os.path.realpath(port))
    if not re.fullmatch(r"tty(?:USB|ACM)\d+", name):
        return {}
    try:
        if not stat.S_ISCHR(os.stat(port).st_mode):
            return {}
        device = os.path.realpath("/sys/class/tty/%s/device" % name)
        usb = next(
            directory for directory in [device] + list(_parent_paths(device))
            if os.path.isfile(os.path.join(directory, "idVendor"))
            and os.path.isfile(os.path.join(directory, "idProduct"))
        )
        def read(key):
            try:
                with open(os.path.join(usb, key), encoding="utf-8") as stream:
                    return stream.read().strip()
            except FileNotFoundError:
                return ""
        return {"vid": int(read("idVendor"), 16), "pid": int(read("idProduct"), 16),
                "serial": read("serial"), "product": read("product")}
    except (OSError, StopIteration, ValueError):
        return {}


def _parent_paths(path):
    while path and path != "/":
        yield path
        path = os.path.dirname(path)


def _reserved_devices():
    """Resolve only identities that are genuinely unsafe to probe."""
    reserved = set()
    for path in _DEFAULT_RESERVED_PATHS:
        if os.path.exists(path):
            reserved.add(os.path.realpath(path))
    # Also catch known devices when a broad udev rule points an alias elsewhere.
    for name in os.listdir("/sys/class/tty"):
        if not name.startswith(("ttyUSB", "ttyACM")):
            continue
        path = "/dev/" + name
        info = _port_identity(path)
        if (info.get("serial") in _KNOWN_HAND_SERIALS
                or info.get("vid") == 0x16D0
                or info.get("product", "").casefold() in ("ljremote",)):
            reserved.add(path)
    return reserved


def _candidate_ports():
    """Discover CH340 candidates on any machine, independent of USB location."""
    reserved = _reserved_devices()
    devices = []
    for name in os.listdir("/sys/class/tty"):
        if not name.startswith("ttyUSB"):
            continue
        path = "/dev/" + name
        info = _port_identity(path)
        if (path not in reserved
                and (info.get("vid"), info.get("pid")) == (0x1A86, 0x7523)):
            devices.append(path)
    preferred = os.path.realpath("/dev/sg100_glove_receiver")
    return sorted(devices, key=lambda path: (path != preferred, path))


def _select_receiver(python, runtime_dir, requested, baud, wanted_ids,
                     allowed_ports="", scan_seconds=2.0, use_dialout=False,
                     fixed_port=False):
    """Return a verified receiver or None; never fall back to an invalid port."""
    preferred = os.path.realpath(requested) if requested not in (None, "", "auto") else ""
    allowed = {
        os.path.realpath(path.strip()) for path in allowed_ports.split(",")
        if path.strip()
    }
    devices = _candidate_ports()
    if allowed:
        devices = [port for port in devices if port in allowed]
    if fixed_port:
        devices = [port for port in devices if port == preferred]
    if preferred in devices:
        devices.sort(key=lambda path: path != preferred)
    if not devices:
        raise ValueError("没有可安全探测的 CH340 串口；请检查接收器、串口限制和系统设备别名")
    # wanted_ids 可为空：扫描只记录每个口实际出现的板卡，供现场核对 ID。
    argv = [
        python, "-c", _RECEIVER_SCAN_SCRIPT, runtime_dir, str(int(baud)),
        str(float(scan_seconds)), ",".join(str(int(board_id)) for board_id in sorted(wanted_ids)),
        preferred, *devices,
    ]
    if use_dialout:
        argv = _dialout_argv(argv)
    proc = subprocess.run(
        argv, env=_clean_env(), capture_output=True, text=True,
        timeout=15 + scan_seconds * len(devices), check=False)
    if proc.returncode != 0 or not proc.stdout.strip():
        detail = (proc.stderr or proc.stdout or "").strip()
        raise RuntimeError(detail or "receiver scan produced no output")
    return json.loads(proc.stdout)

# 真机 /sg100_hand_command 的 11 关节使能掩码（bit 0-10）。
REAL_HAND_ENABLE_MASK = 0x07FF

# 五指位姿输出话题 (与 motion_capture_ik/glove_gesture_node.py 现有输入一致)
FINGERTIP_TOPICS = {
    "left": "/qianshi/glove_left/fingertips",
    "right": "/qianshi/glove_right/fingertips",
}


def _resolve_runtime_dir():
    """返回包内 sg100_runtime 绝对路径 (源码目录与安装目录均适用)。"""
    pkg_share = rospkg.RosPack().get_path("sg100_glove_bridge")
    runtime_dir = os.path.join(pkg_share, "sg100_runtime")
    if not os.path.isdir(runtime_dir):
        raise RuntimeError(
            "找不到 sg100_runtime 目录: %s (确认 sg100_glove_bridge 已编译安装)" % runtime_dir)
    return runtime_dir


def _resolve_sdk_python(runtime_dir):
    """按优先级解析 Python 3.12 解释器并验证可执行。

    优先级: ROS 参数 ~sdk_python > 环境变量 SG100_PYTHON >
    包内 sg100_runtime/.venv_sg100_native/bin/python。
    """
    candidates = []
    param_python = rospy.get_param("~sdk_python", "")
    if param_python:
        candidates.append(("~sdk_python", param_python))
    env_python = os.environ.get("SG100_PYTHON", "")
    if env_python:
        candidates.append(("环境变量 SG100_PYTHON", env_python))
    venv_python = os.path.join(
        runtime_dir, ".venv_sg100_native", "bin", "python")
    candidates.append(("包内虚拟环境", venv_python))

    for source, path in candidates:
        if os.path.isfile(path) and os.access(path, os.X_OK):
            rospy.loginfo("[sg100_glove] SDK Python (%s): %s", source, path)
            return path
    raise RuntimeError(
        "无可用的 SDK Python 解释器。已尝试: %s。"
        "请先运行 sg100_runtime/setup_sg100_native.sh 创建虚拟环境，"
        "或通过 ~sdk_python / SG100_PYTHON 指定已有 Python 3.12 解释器。"
        % "; ".join("%s=%s" % (s, p) for s, p in candidates))


class SG100GloveNode:
    def __init__(self):
        self.side = rospy.get_param("~side", "right")
        if self.side not in SUPPORTED_SIDES:
            raise ValueError("~side 必须是 left、right 或 both（both 为单接收器双手套模式）")

        self.auto_detect = bool(rospy.get_param("~auto_detect", False))
        self.port = rospy.get_param("~serial_port", "auto")
        if self.port == "auto":
            self.port = ""
        # Keep user restrictions separate from the receiver selected at runtime.
        # Automatic rescans must not become pinned to a previous ttyUSB number.
        self.requested_port = self.port
        self.allowed_ports = rospy.get_param("~allowed_ports", "")
        if not self.auto_detect and not self.port:
            raise ValueError("~auto_detect=false 时必须指定 ~serial_port")
        self.baud = int(rospy.get_param("~baud", 921600))
        self.right_id = _board_id(rospy.get_param("~right_id", 2), "~right_id")
        self.left_id = rospy.get_param("~left_id", "")
        # side=left / side=both 必须显式给出 ~left_id（左手板卡 ID 需现场确认，不设默认值）
        if self.side in ("left", "both"):
            if self.left_id == "":
                raise ValueError(
                    "~side=%s 需要同时设置 ~left_id" % self.side)
            self.left_id = _board_id(self.left_id, "~left_id")
        if self.side == "both":
            if self.left_id == self.right_id:
                raise ValueError("~side=both 要求 ~left_id 与 ~right_id 不同")
        else:
            self.board_id = self.left_id if self.side == "left" else self.right_id
        # 数据身份表: hand -> 该 hand 合法的 board ID（P/J 行按此校验，防止左右误分发）
        if self.side == "both":
            self.hand_board_ids = {"left": self.left_id, "right": self.right_id}
        else:
            self.hand_board_ids = {self.side: self.board_id}

        # 右手仿真话题保持原名；左手单独一条，避免 11 维数组无法区分左右。
        self.command_topic = rospy.get_param(
            "~command_topic", "/sg100_sim_hand_command")
        self.left_command_topic = rospy.get_param(
            "~left_command_topic", "/sg100_sim_left_hand_command")
        self.publish_real = bool(rospy.get_param("~publish_real", True))
        self.real_command_topic = rospy.get_param(
            "~real_command_topic", "/sg100_hand_command")
        # 是否透传 SDK --emit-gesture-poses 的 P 行 (五指位姿, 手势识别用)。
        self.emit_gesture_poses = bool(
            rospy.get_param("~emit_gesture_poses", True))

        self.runtime_dir = _resolve_runtime_dir()
        self.sdk_python = _resolve_sdk_python(self.runtime_dir)

        self.launcher = os.path.join(self.runtime_dir, "run_sg100_native.sh")
        if not os.path.isfile(self.launcher):
            raise RuntimeError("缺少 SDK 启动脚本: %s" % self.launcher)

        self.joint_publishers = {}
        if self.side in ("right", "both"):
            self.joint_publishers["right"] = rospy.Publisher(
                self.command_topic, Float32MultiArray, queue_size=1)
        if self.side in ("left", "both"):
            self.joint_publishers["left"] = rospy.Publisher(
                self.left_command_topic, Float32MultiArray, queue_size=1)
        # 真机指令：左右手合成一条 SG100HandCommand，未收到数据的一侧禁用。
        self.real_positions = {"left": None, "right": None}
        self.real_publisher = None
        if self.publish_real:
            if SG100HandCommand is None:
                rospy.logerr(
                    "[sg100_glove] 无法导入 kuavo_msgs/SG100HandCommand，"
                    "不发布真机话题 %s", self.real_command_topic)
            else:
                self.real_publisher = rospy.Publisher(
                    self.real_command_topic, SG100HandCommand, queue_size=1)
                rospy.logwarn(
                    "[sg100_glove] 真机指令已接入 %s "
                    "(MODE_JOINT_POSITION, 11 维, rad)",
                    self.real_command_topic)
        # 五指位姿话题 (P 行): 仅在开启 emit_gesture_poses 时创建。
        # side=both 时左右手各一个话题; 单手模式只有一个。
        self.pose_publishers = {}
        if self.emit_gesture_poses:
            pose_sides = ("left", "right") if self.side == "both" else (self.side,)
            for pose_side in pose_sides:
                self.pose_publishers[pose_side] = rospy.Publisher(
                    FINGERTIP_TOPICS[pose_side], Float32MultiArray, queue_size=1)

        self.proc = None
        self._sdk_needs_dialout = False
        self._shutdown_lock = threading.Lock()
        self._shutdown_started = False
        self._frames = 0
        self._pose_frames = 0
        self._bad_lines = 0
        self._joint_lock = threading.Lock()
        self._interaction_mode = False
        self._freeze_q = {}
        self._interaction_sub = rospy.Subscriber(
            "/glove_gesture/interaction_mode", Bool, self._on_interaction_mode, queue_size=1)

    def _on_interaction_mode(self, msg):
        """交互模式中不再把新的手指关节写到 SG100，保持进入前的最后一帧。"""
        enabled = bool(msg.data)
        with self._joint_lock:
            if enabled == self._interaction_mode:
                return
            self._interaction_mode = enabled
            if not enabled:
                self._freeze_q = {}
        if enabled:
            rospy.loginfo("[sg100_glove] 交互模式开启，手指停止映射到 SG100")
        else:
            rospy.loginfo("[sg100_glove] 交互模式关闭，手指恢复映射到 SG100")

    # ---- 子进程环境 ----

    def _subprocess_env(self):
        """SDK 子进程环境: 剥离 ROS 注入的 Python/库路径, 固定线程数。

        SG100_PYTHON 传给 run_sg100_native.sh, 由其 exec 选定解释器。
        OPENBLAS/OMP 单线程与 SDK 启动脚本一致, 保证数值确定性。
        """
        env = dict(os.environ)
        for key in ("PYTHONPATH", "PYTHONHOME", "LD_LIBRARY_PATH"):
            env.pop(key, None)
        env["SG100_PYTHON"] = self.sdk_python
        env["OPENBLAS_NUM_THREADS"] = "1"
        env["OMP_NUM_THREADS"] = "1"
        return env

    def _sdk_argv(self):
        argv = [
            "bash", self.launcher,
            "--serial", str(self.port),
            "--baud", str(self.baud),
            "--hand", self.side,
        ]
        if self.side == "both":
            # 单接收器双手: SDK 同时路由左右两个板卡 ID
            argv.extend(["--left-id", str(self.left_id),
                         "--right-id", str(self.right_id)])
        else:
            argv.extend(["--%s-id" % self.side, str(self.board_id)])
        # Reconnect through the bridge so every new open requires a fresh ID scan.
        argv.extend(["--no-viz", "--no-reconnect"])
        if self.emit_gesture_poses:
            argv.append("--emit-gesture-poses")
        return argv

    def _use_receiver_with_target_boards(self):
        """验证目标板卡后才返回串口；失败时不打开长期 SDK。"""
        wanted = set(self.hand_board_ids.values())
        options = {"allowed_ports": self.allowed_ports, "fixed_port": not self.auto_detect}
        try:
            payload = _select_receiver(
                self.sdk_python, self.runtime_dir, self.requested_port, self.baud, wanted,
                **options)
            if (not payload.get("chosen") and any(
                    _permission_denied(report.get("error"))
                    for report in payload.get("reports", []))):
                rospy.logwarn_throttle(5.0, "[sg100_glove] 串口权限不足，改用 dialout 再探测")
                payload = _select_receiver(
                    self.sdk_python, self.runtime_dir, self.requested_port, self.baud, wanted,
                    use_dialout=True, **options)
                self._sdk_needs_dialout = bool(payload.get("chosen"))
            else:
                self._sdk_needs_dialout = False
        except (OSError, ValueError, RuntimeError, subprocess.TimeoutExpired) as exc:
            rospy.logerr_throttle(5.0, "[sg100_glove] 串口探测失败，未启动 SDK: %s", exc)
            return False
        chosen = payload.get("chosen")
        if not chosen:
            if len(payload.get("matches", [])) > 1:
                rospy.logerr_throttle(
                    5.0, "[sg100_glove] 多个串口均收到目标板卡 %s（%s）；请指定首选串口",
                    sorted(wanted), ", ".join(payload["matches"]))
                return False
            summary = ", ".join(
                "%s: %s" % (report.get("device"),
                            report.get("ids") or report.get("error") or "无帧")
                for report in payload.get("reports", []))
            rospy.logerr_throttle(
                5.0, "[sg100_glove] 未收到目标板卡 %s，未启动 SDK（%s）",
                sorted(wanted), summary or "无可用串口")
            return False
        if chosen != self.port:
            rospy.logwarn("[sg100_glove] 已验证目标板卡 %s，使用 %s", sorted(wanted), chosen)
            self.port = chosen
        return True

    # ---- 启动与线程 ----

    def start(self):
        rospy.on_shutdown(self.shutdown)
        threading.Thread(
            target=self._run_sdk, name="sdk-monitor", daemon=True).start()

    def _spawn_sdk(self):
        argv = self._sdk_argv()
        if self._sdk_needs_dialout:
            argv = _dialout_argv(argv)
        proc = subprocess.Popen(
            argv,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            env=self._subprocess_env(),
            bufsize=1,
            universal_newlines=True,
        )
        self.proc = proc
        rospy.loginfo(
            "[sg100_glove] SDK 子进程已启动 (pid %d): %s",
            proc.pid, " ".join(argv))
        if self.side == "both":
            rospy.loginfo(
                "[sg100_glove] 双手套(单接收器) %s @ %d, 左手 board ID %d -> %s, 右手 board ID %d -> %s",
                self.port, self.baud, self.left_id, self.left_command_topic,
                self.right_id, self.command_topic)
        else:
            joint_topic = (
                self.command_topic if self.side == "right"
                else self.left_command_topic)
            rospy.loginfo(
                "[sg100_glove] %s 手套 %s @ %d, board ID %d -> 位姿话题: %s"
                " (关节话题: %s, 11 维, rad)",
                self.side, self.port, self.baud, self.board_id,
                FINGERTIP_TOPICS[self.side], joint_topic)
        threading.Thread(
            target=self._stdout_loop, args=(proc,), name="sdk-stdout", daemon=True).start()
        threading.Thread(
            target=self._stderr_loop, args=(proc,), name="sdk-stderr", daemon=True).start()
        return proc

    def _run_sdk(self):
        """没找到手套就继续探测。SDK 因无帧退出时换口重开，不把整个 ROS 节点关掉。"""
        while not rospy.is_shutdown() and not self._shutdown_started:
            if not self._use_receiver_with_target_boards():
                rospy.sleep(1.0)
                continue
            if rospy.is_shutdown() or self._shutdown_started:
                return
            proc = self._spawn_sdk()
            code = proc.wait()
            if rospy.is_shutdown() or self._shutdown_started:
                return
            rospy.logerr(
                "[sg100_glove] SDK 子进程退出 (code=%d)，重新探测串口", code)
            rospy.sleep(0.5)

    # ---- stdout: 数据通道 ----

    def _stdout_loop(self, proc):
        """逐行读取 SDK stdout, 解析 J 行 (关节) 与 P 行 (五指位姿) 并发布。

        单手 J: "J seq q0..q10" (13 字段); 双手 J (side=both):
        "J hand board_id seq q0..q10" (15 字段); P 行: "P hand board_id
        seq <35 数>" (39 字段)。
        每帧直接发布最新合法数据, 不缓存历史; 单帧解析失败只跳过, 不退出;
        J 与 P 互不覆盖 (独立话题)。
        """
        if proc.stdout is None:
            return
        for line in proc.stdout:
            if rospy.is_shutdown():
                break
            fields = line.split()
            if not fields:
                continue
            kind = fields[0]
            if kind == "J":
                self._handle_joint_line(fields)
            elif kind == "P":
                self._handle_pose_line(fields)

    def _handle_joint_line(self, fields):
        if self.side == "both":
            # 双手 J: "J hand board_id seq q0..q10"，按 hand 分发到左右仿真话题
            if len(fields) != BOTH_FIELDS_PER_LINE:
                self._bad_lines += 1
                if self._bad_lines % 100 == 1:
                    rospy.logwarn(
                        "[sg100_glove] J 行字段数异常: 期望 %d, 实际 %d (累计 %d)",
                        BOTH_FIELDS_PER_LINE, len(fields), self._bad_lines)
                return
            hand, board_id, q = fields[1], fields[2], fields[4:]
            if not self._check_hand_identity(hand, board_id):
                return
        else:
            if len(fields) != FIELDS_PER_LINE:
                self._bad_lines += 1
                if self._bad_lines % 100 == 1:
                    rospy.logwarn(
                        "[sg100_glove] J 行字段数异常: 期望 %d, 实际 %d (累计 %d)",
                        FIELDS_PER_LINE, len(fields), self._bad_lines)
                return
            hand = self.side
            q = fields[2:]
        try:
            q = [float(x) for x in q]
        except ValueError:
            self._bad_lines += 1
            return
        if len(q) != JOINT_COUNT or not all(math.isfinite(x) for x in q):
            self._bad_lines += 1
            return
        with self._joint_lock:
            if self._interaction_mode:
                frozen = self._freeze_q.get(hand)
                if frozen is None:
                    frozen = list(q)
                    self._freeze_q[hand] = frozen
                q = list(frozen)
            else:
                self._freeze_q[hand] = list(q)
        publisher = self.joint_publishers.get(hand)
        if publisher is None:
            self._frames += 1
            return
        msg = Float32MultiArray()
        msg.data = q
        publisher.publish(msg)
        self._publish_real_command(hand, q)
        self._frames += 1
        if self._frames == 1:
            rospy.loginfo("[sg100_glove] 开始发布五指关节数据")

    def _publish_real_command(self, hand, q):
        """把当前手的 11 维弧度并入 /sg100_hand_command。

        关节顺序与 SDK JOINT_ORDER、真机驱动一致：
        thumb_j1..3, index_j1..3, middle_j1..2, little_j1..3。
        刚度、阻尼和输出限幅留空，由 sg100_hand_driver 的参数决定。
        """
        if self.real_publisher is None or SG100HandCommand is None:
            return
        if hand not in self.real_positions:
            return
        self.real_positions[hand] = list(q)
        cmd = SG100HandCommand()
        cmd.header.stamp = rospy.Time.now()
        cmd.control_mode = SG100HandCommand.MODE_JOINT_POSITION
        left = self.real_positions["left"]
        right = self.real_positions["right"]
        # The driver validates both arrays before considering enable masks.
        # The all-zero placeholder for an unseen hand is never enabled.
        cmd.left_hand_positions = left if left is not None else [0.0] * JOINT_COUNT
        cmd.right_hand_positions = right if right is not None else [0.0] * JOINT_COUNT
        cmd.source_id = 1
        cmd.left_enable_mask = REAL_HAND_ENABLE_MASK if left is not None else 0
        cmd.right_enable_mask = REAL_HAND_ENABLE_MASK if right is not None else 0
        self.real_publisher.publish(cmd)

    def _check_hand_identity(self, hand, board_id_text):
        """校验带身份的行 (P 行 / 双手 J 行) 的 hand 与 board_id 是否匹配。

        防止单接收器双手数据左右误分发; 不匹配的帧丢弃并计数。
        """
        expected = self.hand_board_ids.get(hand)
        if expected is None:
            self._bad_lines += 1
            return False
        try:
            board_id = int(board_id_text)
        except ValueError:
            self._bad_lines += 1
            return False
        if board_id != expected:
            self._bad_lines += 1
            if self._bad_lines % 100 == 1:
                rospy.logwarn(
                    "[sg100_glove] %s 手 board_id=%d 与配置 %d 不符, 丢弃该帧 (累计 %d)",
                    hand, board_id, expected, self._bad_lines)
            return False
        return True

    def _handle_pose_line(self, fields):
        """P 行: 五指电磁传感器模块的跟踪位姿 (非人体指尖 FK)。

        35 数 = 5 指 x [x,y,z,qw,qx,qy,qz], 顺序
        thumb/index/middle/ring/pinky, 位置 m, 四元数 WXYZ。
        hand 与 board_id 必须与本节点配置匹配; 不合法帧直接丢弃,
        不影响同帧 J 数据。
        """
        if len(fields) != POSE_FIELDS_PER_LINE:
            self._bad_lines += 1
            if self._bad_lines % 100 == 1:
                rospy.logwarn(
                    "[sg100_glove] P 行字段数异常: 期望 %d, 实际 %d (累计 %d)",
                    POSE_FIELDS_PER_LINE, len(fields), self._bad_lines)
            return
        hand = fields[1]
        if not self._check_hand_identity(hand, fields[2]):
            return
        try:
            pose = [float(x) for x in fields[4:]]
        except ValueError:
            self._bad_lines += 1
            return
        if len(pose) != POSE_COUNT or not all(math.isfinite(x) for x in pose):
            self._bad_lines += 1
            return
        publisher = self.pose_publishers.get(hand)
        if publisher is None:
            return
        msg = Float32MultiArray()
        msg.data = pose
        publisher.publish(msg)
        self._pose_frames += 1
        if self._pose_frames == 1:
            rospy.loginfo("[sg100_glove] 开始发布五指位姿")

    # ---- stderr: 诊断通道 ----

    def _stderr_loop(self, proc):
        """及时消费 stderr, 避免子进程因缓冲区写满而阻塞。"""
        if proc.stderr is None:
            return
        for line in proc.stderr:
            text = line.rstrip()
            if text:
                rospy.loginfo("[sg100_sdk] %s", text)

    # ---- 生命周期 ----

    def shutdown(self):
        """ROS shutdown 回调: 终止并回收 SDK 子进程, 释放串口。"""
        with self._shutdown_lock:
            if self._shutdown_started:
                return
            self._shutdown_started = True
        if self.proc is None:
            return
        if self.proc.poll() is None:
            self.proc.terminate()
            try:
                self.proc.wait(timeout=5.0)
                rospy.loginfo(
                    "[sg100_glove] SDK 子进程已退出 (关节 %d 帧, 位姿 %d 帧)",
                    self._frames, self._pose_frames)
            except subprocess.TimeoutExpired:
                self.proc.kill()
                self.proc.wait()
                rospy.logwarn(
                    "[sg100_glove] SDK 子进程被强杀 (关节 %d 帧, 位姿 %d 帧)",
                    self._frames, self._pose_frames)


def main():
    rospy.init_node("sg100_glove_bridge")
    node = SG100GloveNode()
    node.start()
    rospy.spin()


if __name__ == "__main__":
    main()
