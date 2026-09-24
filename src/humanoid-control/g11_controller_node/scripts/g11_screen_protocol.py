#!/usr/bin/env python3
"""G11 遥控器屏幕指令协议 (屏幕端: 虚拟通道 CH14 页面码 / CH15 功能码脉冲)

纯 Python 模块, 不依赖 ROS / 不依赖 h12pro_controller_node。

码值约定(均为"接收端收到值", 屏幕发送值经 G11 MCU 换算: 接收=(发送-1050)*1.6+282):
  - 空闲/复位 = 1002
  - CH14 页面码(持续保持): 二级菜单与三级页
      首页/未选类型 1002 (IDLE)
      人型二级菜单 1010 (HUMANOID) / 轮臂二级菜单 1018 (WHEEL)
      常用功能页(人型) / 动作相关页(轮臂) 1162 (COMMON)
      特殊功能页(人型) 1170 (SPECIAL)
      下肢三级页 1178 (LEG) / VR 三级页 1186 (VR)
  - CH15 功能码(脉冲, 屏幕 ~800ms 后自动复位 1002): 按页分区
      下肢 362~402 / VR 522~546 / 常用功能 682~722 / 特殊功能 842~898 / 硬件启动 282

解析策略:
  - CH15 上升沿(空闲1002 -> 非空闲码)触发一次
  - 页面二次校验: 功能码所属页面必须与 CH14 当前页面一致才执行
    (硬件启动码 282 允许在人型/轮臂二级菜单 1010/1018 下执行)
"""

# ---------- 页面码(接收值) ----------
PAGE_IDLE_RECV = 1002

# G11 全通道(含实体 SBUS 与虚拟 CH14~16)接收值域硬边界:
#   任何通道不可能出现 < 282 或 > 1722 的数值; 超范围视为噪声。
CHANNEL_MIN_RECV = 282
CHANNEL_MAX_RECV = 1722

PAGE_HUMANOID_RECV = 1010   # 人型 二级菜单
PAGE_WHEEL_RECV = 1018      # 轮臂 二级菜单
PAGE_COMMON_RECV = 1162     # 常用功能页(人型) / 动作相关页(轮臂)
PAGE_SPECIAL_RECV = 1170    # 特殊功能页(人型)
PAGE_LEG_RECV = 1178        # 下肢三级页(轮臂)
PAGE_VR_RECV = 1186         # VR 三级页
# 兼容旧名(1162/1170 槽位语义已随屏幕菜单重构变化)
PAGE_POSTURE_RECV = PAGE_COMMON_RECV
PAGE_ACTION_RECV = PAGE_SPECIAL_RECV

SCREEN_PAGE_RECV = {
    1002: "IDLE",          # 首页/未选类型(空闲)
    1010: "HUMANOID",      # 人型二级菜单
    1018: "WHEEL",         # 轮臂二级菜单
    1162: "COMMON",        # 常用功能页(人型) / 动作相关页(轮臂)
    1170: "SPECIAL",       # 特殊功能页(人型)
    1178: "LEG",           # 下肢三级页
    1186: "VR",            # VR 三级页
}

# ---------- CH16 设置位通道(接收值, 持续保持) ----------
# 屏幕端遥控设置页 4 档组合编码(发送基值1600 → 接收基值1162, 步进按位叠加):
#   force 位 = 接收值相对 1162 偏移 +8  (1170/1186 → 启动前必须选择机器人类型)
#   claw  位 = 接收值相对 1162 偏移 +16 (1178/1186 → 旋钮控制夹爪末端跟随)
# 四档: 1162(都关) / 1170(force) / 1178(claw) / 1186(都开)
# 注: 1170 现在表示 force=1+claw=0, 不再需要按单值判"强制档"。
CFG_RECV_BASE = 1162
CFG_FORCE_BIT_OFFSET = 8      # force 位偏移(1170/1186)
CFG_CLAW_BIT_OFFSET = 16      # claw  位偏移(1178/1186)

def decode_cfg(cfg: int):
    """解码 CH16 设置位 → (force: bool, claw: bool)。非本值域(不在4档内)返回 None。"""
    offset = cfg - CFG_RECV_BASE
    if offset not in (0, CFG_FORCE_BIT_OFFSET, CFG_CLAW_BIT_OFFSET,
                      CFG_FORCE_BIT_OFFSET + CFG_CLAW_BIT_OFFSET):
        return None
    force = bool(offset & CFG_FORCE_BIT_OFFSET)
    claw = bool(offset & CFG_CLAW_BIT_OFFSET)
    return force, claw

# ---------- 功能码(接收值) -> (功能名, 类型, 目标) ----------
#   type: "trigger" = 调状态机 trigger(getattr robot_state_machine)
#         "head"    = 翻转头部控制模式(特殊处理)
#         "action"  = 自定义动作(常用功能页 1~3 -> customize_action_*)
#         "boot"    = 硬件启动(人型/轮臂二级菜单下发, 目标 initial_pre)
#         "skip"    = 暂不处理(仅日志)
#
# 屏幕菜单重构(2026-09): CH15 显式表, 不再用算术公式推导。
#   接收值 = (发送值 - 1050) * 1.6 + 282
SCREEN_CMD_MAP = {
    # --- 硬件启动 (人型/轮臂二级菜单 CH14=1010/1018) ---
    282: ("硬件启动",      "boot",    "initial_pre"),
    # --- 常用功能页 (CH14=1162) ---
    #   人型: 踏步/站立/切换MPC-AMP + 自定义动作1~3
    #   轮臂: 仅自定义动作1~3 (踏步/站立 状态机侧由 is_not_wheel_robot 拦下)
    682: ("踏步",         "trigger", "walk"),
    690: ("站立",         "trigger", "stance"),
    698: ("切换MPC/AMP",  "trigger", "switch_controller"),
    706: ("自定义动作1",   "action",  "customize_action_RR_A"),
    714: ("自定义动作2",   "action",  "customize_action_RR_B"),
    722: ("自定义动作3",   "action",  "customize_action_RR_C"),
    # --- 特殊功能页 (CH14=1170, 仅人型) ---
    # 屏幕端 SPECIAL_SHOW_ALL=0, 页内只创建「航空箱起身/坐下」2 个按钮,
    # 前 6 项不发码 → 机器人端注释保留(不删), 恢复时屏幕端置 1、此处取消注释即可。
    # 842: ("进入VMP",      "trigger", "vmp_controller"),
    # 850: ("退出VMP",      "trigger", "exit_vmp_controller"),
    # 858: ("进入舞蹈",     "trigger", "dance_controller"),
    # 866: ("退出舞蹈",     "trigger", "exit_dance_controller"),
    # 874: ("深度步态",     "trigger", "depth_loco_switch"),
    # 882: ("头部控制",     "head",    "toggle_head_control"),
    890: ("航空箱起身",   "trigger", "sit_to_stand"),
    898: ("航空箱坐下",   "trigger", "sit_down"),
    # --- VR 页 (CH14=1186) ---
    522: ("进入VR",      "trigger", "start_vr_remote_control"),
    530: ("退出VR",      "trigger", "stop_vr_remote_control"),
    538: ("开始录制",    "trigger", "record_vr_rosbag"),
    546: ("停止录制",    "trigger", "stop_record_vr_rosbag"),
    # --- 下肢页 (CH14=1178) 轮臂控制 ---
    #   type: "wheel_leg" = 轮臂下肢模式切换/躯干组选择(目标为 C++ 模式名),
    #         joy_node 端据此驱动 mobile_manipulator_joy_command_node
    362: ("本地坐标系",     "wheel_leg", "cmd_vel"),
    370: ("世界坐标系",     "wheel_leg", "cmd_vel_world"),
    378: ("切换到躯干控制", "wheel_leg", "torso_control"),
    386: ("躯干复位",       "wheel_leg", "torso_reset"),
    394: ("躯干组1-平移升降", "wheel_leg", "torso_group_xz"),     # 左杆上下vx/右杆上下vz
    402: ("躯干组2-旋转俯仰", "wheel_leg", "torso_group_yawpitch"),  # 右杆左右vyaw/左杆上下vpitch
}

# 常用功能页自定义动作(1~3): 抱拳 / 打招呼(挥手) / 点赞
ACTION_TRIGGERS = [
    "customize_action_RR_A", "customize_action_RR_B",
    "customize_action_RR_C",
]

# 屏幕端 SPECIAL_SHOW_ALL=0 时不会发出的"已知但未启用"功能码(特殊功能页前 6 项:
# 进入/退出VMP、进入/退出舞蹈、深度步态、头部控制)。单独计数而不静默丢弃, 避免
# 屏幕端把开关改回 1 后机器人端无任何线索(按钮点了没反应、两端都没日志)。
HIDDEN_CMDS = frozenset({842, 850, 858, 866, 874, 882})


def page_of_cmd(cmd: int):
    """由功能码反查其所属页面的接收值(区间判断)。

    硬件启动码(282)不归三级页, 返回 None(由调用方按 boot 页面规则校验)。
    """
    if cmd == 282:
        return None  # 硬件启动: 允许二级菜单 1010/1018, 单独校验
    if 362 <= cmd <= 410:
        return PAGE_LEG_RECV       # 下肢 1178 (362本地系/370世界系/378躯干/386复位/394组1/402组2)
    if 522 <= cmd <= 546:
        return PAGE_VR_RECV        # VR 1186
    if 682 <= cmd <= 722:
        return PAGE_COMMON_RECV    # 常用功能 1162 (682踏步/690站立/698切换MPCAMP/706~722自定义动作1~3)
    if 890 <= cmd <= 898:
        return PAGE_SPECIAL_RECV   # 特殊功能 1170 (890航空箱起身/898航空箱坐下;
                                   #   842~882 前6项屏幕已隐藏, 暂不注册/不校验)
    return None


class ScreenCmdParser:
    """G11 屏幕虚拟通道解析器(纯逻辑, 状态由调用方持有)。

    输入一帧的虚拟通道值(与物理通道号一致, 均为接收值):
      - CH14: 页面码(持续保持)
      - CH15: 功能码(脉冲, 上升沿触发)
      - CH16: 设置位(持续保持): 双 bit 组合编码
              force位(1170/1186)=启动前必须选类型; claw位(1178/1186)=旋钮控制夹爪
    """

    def __init__(self):
        self._prev_cmd = PAGE_IDLE_RECV
        self._prev_valid = False
        self.boot_force = False   # CH16 force 位: 启动前必须选择机器人类型
        self.claw_mode = False    # CH16 claw  位: 旋钮控制夹爪末端跟随
        # 状态型通道(CH14 页面码 / CH16 设置位)的最后有效值
        self.effective_page = PAGE_IDLE_RECV
        self.effective_cfg = None
        # 诊断计数(只累加, 不随 reset 清零): 供调用方节流上报,
        # 便于现场排查"偶尔不响应/丢一次指令"(屏幕端 README 要求越界帧丢弃并计数)
        self.noise_frames = 0     # CH14~16 越界帧数合计(= cmd_drops + state_holds)
        self.cmd_drops = 0        # CH15(功能码/脉冲型)越界 -> 整帧丢弃, 命令可能丢失
        self.state_holds = 0      # CH14/CH16(状态型)越界 -> 沿用最后有效值, 不丢帧
        # 按通道统计异常值 {通道号: 次数}
        self.ch_abnormal = {14: 0, 15: 0, 16: 0}
        self.last_abnormal = None  # (通道号, 异常值) 最近一次异常
        self.unknown_cmds = 0     # 未知功能码次数
        self.hidden_cmds = 0      # 命中"已知但未启用"码(842~882)的次数

    def reset(self):
        self._prev_cmd = PAGE_IDLE_RECV
        self._prev_valid = False
        self.boot_force = False
        self.claw_mode = False
        # 不清 effective_page/effective_cfg

    def _note_abnormal(self, ch: int, value) -> None:
        """记录某条通道出现异常(越界)值, 供调用方上报定位是哪条通道。"""
        self.ch_abnormal[ch] = self.ch_abnormal.get(ch, 0) + 1
        self.last_abnormal = (ch, value)

    def update(self, page_val: int, cmd_val: int, cfg: int = None):
        """喂入一帧 CH14/CH15(/CH16) 接收值。

        Args:
            page_val: CH14 页面码(接收值)
            cmd_val:  CH15 功能码(接收值)
            cfg:      CH16 设置位(接收值, 可选); 语义由协议表统一解码

        Returns:
            dict | None: 命中时返回 {"func_name","type","target","cmd","page"}
            未触发(非上升沿/未知码/页面不匹配/CH15 越界)返回 None。
        """
        held = False
        # --- CH14 页面码: 越界沿用最后有效值 ---
        if CHANNEL_MIN_RECV <= page_val <= CHANNEL_MAX_RECV:
            self.effective_page = page_val
        else:
            self._note_abnormal(14, page_val)
            page_val = self.effective_page
            held = True
        # --- CH16 设置位: 越界沿用最后有效值 ---
        if cfg is not None:
            if CHANNEL_MIN_RECV <= cfg <= CHANNEL_MAX_RECV:
                self.effective_cfg = cfg
            else:
                self._note_abnormal(16, cfg)
                cfg = self.effective_cfg   # 可能为 None
                held = True
        # --- CH15 功能码: 越界整帧跳过 ---
        if not (CHANNEL_MIN_RECV <= cmd_val <= CHANNEL_MAX_RECV):
            self._note_abnormal(15, cmd_val)
            self.noise_frames += 1
            self.cmd_drops += 1
            return None
        if held:
            self.noise_frames += 1
            self.state_holds += 1

        # CH16 设置位解码(独立于 CH15 上升沿, 持续保持值双 bit 解码)
        if cfg is not None:
            decoded = decode_cfg(cfg)
            if decoded is not None:
                force, claw = decoded
                self.boot_force = force
                self.claw_mode = claw

        rising = (cmd_val != PAGE_IDLE_RECV and
                  (not self._prev_valid or self._prev_cmd == PAGE_IDLE_RECV))
        self._prev_cmd = cmd_val
        self._prev_valid = True

        if not rising:
            return None
        if cmd_val not in SCREEN_CMD_MAP:
            if cmd_val in HIDDEN_CMDS:
                # 屏幕端 SPECIAL_SHOW_ALL=0 时不会发这些码; 若收到说明屏幕已改回 1,
                # 而机器人端这 6 项仍在注释保留状态 -> 计数上报, 不要静默
                self.hidden_cmds += 1
            else:
                self.unknown_cmds += 1
            return None

        func_name, ftype, target = SCREEN_CMD_MAP[cmd_val]

        # 页面二次校验
        expect_page = page_of_cmd(cmd_val)
        if ftype == "boot":
            # 硬件启动码: 允许在人型(1010)/轮臂(1018) 二级菜单下发
            if page_val not in (PAGE_HUMANOID_RECV, PAGE_WHEEL_RECV):
                return {
                    "func_name": func_name, "type": ftype, "target": target,
                    "cmd": cmd_val, "page": page_val,
                    "rejected": True,
                    "expect_page": f"{PAGE_HUMANOID_RECV}/{PAGE_WHEEL_RECV}",
                }
        elif expect_page is not None and page_val != expect_page:
            return {
                "func_name": func_name, "type": ftype, "target": target,
                "cmd": cmd_val, "page": page_val,
                "rejected": True, "expect_page": expect_page,
            }

        return {
            "func_name": func_name, "type": ftype, "target": target,
            "cmd": cmd_val, "page": page_val, "rejected": False,
        }
