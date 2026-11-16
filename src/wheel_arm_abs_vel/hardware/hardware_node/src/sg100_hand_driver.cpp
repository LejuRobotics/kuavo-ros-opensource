/**
 * @file sg100_hand_driver.cpp
 * @brief SG100 灵巧手驱动实现(声明见 include/hardware_node/sg100_hand_driver.h)
 *
 * 本文件内容:
 * 1. SG100HandROSNode 类实现 —— ROS 与 SG100 手之间的桥:
 *    - 订阅 /sg100_hand_command(SG100HandCommand)接收目标
 *    - 按 sg100_control_rate(默认 100Hz)发布 /sg100_hand_state(SG100HandState)反馈
 *    - 有实际位置下发时按 50 Hz 发布 /sg100_hand_filtered_command(SG100HandFilteredCommand),
 *      内容是低通滤波后真正下发给 SDK 的值
 *    - 经 C++ SDK(libsg100_sdk.so)驱动手;话题与 SDK 同为 11 维(JOINT_COUNT=11)
 * 2. loadSg100HardcodedDefaults() —— 11 个命令关节名与左右手 rest pose 的硬编码默认值
 * 3. 文件内工具(匿名命名空间)—— 配置目录查找、SDK 状态/模式等待、SDK 回调
 *
 * roslaunch 示例:
 *   roslaunch motion_capture_ik sg100_hand_control.launch
 *     sg100_left_port:=/dev/lefthand sg100_right_port:=/dev/righthand
 *     sg100_baudrate:=1000000 sg100_control_rate:=100.0 sg100_cmd_filter_alpha:=0.2
 *
 * 命令位置在话题上为弧度,下发 SDK 时换算为度(rad * 180/pi → deg);
 * 收到的位置数组必须是 JOINT_COUNT(11)个元素。
 *
 * 控制模式:SG100HandCommand 的枚举与 SDK CtrlMode_e(0-9)一致,解析出的 mode
 * 直接写入 Joint_Control_t.mode。支持 7(位置)、9(阻抗)、0(NULL)。
 * 其他模式导致整条命令拒绝。
 *
 * 上游:
 * - 本节点内的 SG100HandBridge 按手势库发布 /sg100_hand_command(不依赖 VR)
 * - VR 只发布 /sg100/trigger,并用 /sg100/step_gesture 切手势
 * - 仿真同一套桥在 dexhand_mujoco_node 里启动
 */

#include "hardware_node/sg100_hand_driver.h"
#include "hardware_node/sg100_command_encoding.h"

#include "wheel_arm_abs_vel/sg100/SG100HandBridge.h"

#include <ros/ros.h>
#include <ros/package.h>
#include <kuavo_msgs/SG100HandCommand.h>
#include <kuavo_msgs/SG100HandFilteredCommand.h>
#include <kuavo_msgs/SG100HandState.h>
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

// SG100 SDK 头(接口声明在此,实现位于 libsg100_sdk.so)
#include "sg100_device.h"
#include "sg100_device_manager.h"
#include "sg100_types.h"

namespace {

// JOINT_COUNT 由 sg100_types.h 提供(11)
constexpr double DEFAULT_CONTROL_HZ = 100.0;
constexpr double kReconnectRetryIntervalSec = 0.2;
constexpr uint32_t kReconnectReadinessWaitMs = 150;
constexpr double kPi = 3.14159265358979323846;

std::string getSg100ConfigSearchDir() {
    try {
        const std::string package_path = ros::package::getPath("hardware_node");
        if (!package_path.empty()) {
            // 配置随包存放：源码/devel 用包内 config/sg100，install 后由 CMakeLists
            // 装到 <share>/hardware_node/config/sg100 —— 两侧同一相对路径，无需区分环境
            return package_path + "/config/sg100";
        }
    } catch (const std::runtime_error&) {
    }
    return "";
}

// ==== SG100 SDK 回调与工具函数(不访问类成员)====

// Wait until status traffic is observed (same idea as SDK realtime_joint_control_example).
bool waitForRealtimeTraffic(SG100Device* hand, uint32_t timeout_ms) {
    if (!hand) {
        return false;
    }
    const uint32_t baseline = hand->GetStatusRecvCount();
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);
    while (std::chrono::steady_clock::now() < deadline) {
        if (hand->GetStatusRecvCount() != baseline) {
            return true;

        }
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    return hand->GetStatusRecvCount() != baseline;
}

bool queryPalmRuntimeMode(SG100Device* hand, Palm_Runtime_Mode_Status_t& status,
                          uint32_t wait_ms = 250, uint32_t retries = 3) {
    if (!hand) {
        return false;
    }
    for (uint32_t attempt = 0; attempt < retries; ++attempt) {
        if (!hand->PalmRuntimeModeGetStatus()) {
            continue;
        }
        const auto deadline =
            std::chrono::steady_clock::now() + std::chrono::milliseconds(wait_ms);
        while (std::chrono::steady_clock::now() < deadline) {
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
            if (hand->GetPalmRuntimeModeStatus(status)) {
                return true;
            }
        }
        if (hand->GetPalmRuntimeModeStatus(status)) {
            return true;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    return false;
}

bool ensureRealtimeMode(SG100Device* hand, uint32_t retries = 3) {
    if (!hand) {
        return false;
    }
    Palm_Runtime_Mode_Status_t status{};
    if (queryPalmRuntimeMode(hand, status) &&
        status.realtime_enabled && status.mode == PALM_RUNTIME_MODE_REALTIME) {
        return true;
    }
    for (uint32_t attempt = 1; attempt <= retries; ++attempt) {
        if (hand->SwitchToRealtimeMode()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            return true;
        }
        if (hand->GetWorkMode() == WORK_MODE_REALTIME) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            return true;
        }
        ROS_WARN("SG100 realtime switch/verify attempt %u/%u failed",
                 static_cast<unsigned>(attempt),
                 static_cast<unsigned>(retries));
        std::this_thread::sleep_for(std::chrono::milliseconds(150));
    }
    return false;
}

bool hasRawSnapshotPayload(const RawSnapshotFrame_t& raw) {
    return raw.adc_in1 != 0 || raw.adc_in2 != 0 || raw.adc_in3 != 0 ||
           raw.hall_value != 0 || raw.ia_offset != 0 || raw.ib_offset != 0 ||
           raw.ic_offset != 0 || raw.ia != 0.0f || raw.ib != 0.0f ||
           raw.ic != 0.0f || raw.id != 0.0f || raw.iq != 0.0f ||
           raw.encoder_angle != 0.0f || raw.joint_angle != 0.0f ||
           raw.elec_angle != 0.0f || raw.logic_angle != 0.0f ||
           raw.motor_speed != 0.0f;
}

bool switchToPassthroughWithRetry(SG100Device* hand, uint32_t retries = 3) {
    if (!hand) {
        return false;
    }
    for (uint32_t attempt = 1; attempt <= retries; ++attempt) {
        if (hand->SwitchToPassthroughMode()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            return true;
        }
        if (hand->GetWorkMode() == WORK_MODE_PASSTHROUGH) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            return true;
        }
        ROS_WARN("SG100 passthrough switch attempt %u/%u failed",
                 static_cast<unsigned>(attempt),
                 static_cast<unsigned>(retries));
        std::this_thread::sleep_for(std::chrono::milliseconds(150));
    }
    return false;
}

void fillNullFrame(Hand_Control_t& frame) {
    std::memset(&frame, 0, sizeof(frame));
    for (int i = 0; i < JOINT_COUNT; ++i) {
        frame.joints[i].id = static_cast<uint8_t>(i + 1);
        frame.joints[i].mode = MODE_NULL;
    }
}

void onHandError(uint8_t joint_id, uint8_t error_code) {
    ROS_ERROR("SG100 Hand Error: Joint %d, Code %d", (int)joint_id, (int)error_code);
}

}  // namespace

/**
 * @brief SG100 命令关节名与默认 rest pose(硬编码,不解析 URDF)
 *
 * 关节顺序与 kuavo_msgs/SG100HandCommand 一致(11 DOF)。
 * 命令关节名与 URDF 关节名是两套命名(URDF 侧如 l_thumb_j1),不可互推,故此处写死;
 * URDF 只用于关节限位(见 motion_capture_ik 的 SG100JointLimits)。
 *
 * @param joint_names [out] 11 个命令关节名(SDK/msg 顺序)
 * @param left_rest   [out] 左手 11 个 rest 位置,弧度(初始化目标,滤波从硬件姿态插值过去)
 * @param right_rest  [out] 右手 11 个 rest 位置,弧度
 * @return 固定 true
 */
bool loadSg100HardcodedDefaults(std::vector<std::string>& joint_names,
                                std::vector<float>& left_rest,
                                std::vector<float>& right_rest) {
    // 命令关节名写死:URDF 用另一套命名(如 l_thumb_j1),两者不可互推
    joint_names = {
        "TH_CMC_ABD", "TH_MCP_FLEX", "TH_IP_FLEX",
        "IF_MCP_ABD", "IF_MCP_FLEX", "IF_PIP_FLEX",
        "MF_MCP_FLEX", "MF_PIP_FLEX",
        "LF_MCP_ABD", "LF_MCP_FLEX", "LF_PIP_FLEX",
    };

    // rest pose 同样写死(URDF 不提供该信息)，11 维，与 MuJoCo HeimanHand::kLeftRestPose / kRightRestPose 一致
    left_rest  = {0.00, -1.0, 1.57, 0.0, 1.57, 1.57, 1.57, 1.57, 0.0, 1.57, 1.57};
    right_rest = {0.00, -1.0, 1.57, 0.0, 1.57, 1.57, 1.57, 1.57, 0.0, 1.57, 1.57};

    ROS_INFO("SG100 hand config loaded (joint_names + rest from hardcoded defaults)");
    return true;

}

// ==== SG100HandROSNode 类方法实现(声明见 include/hardware_node/sg100_hand_driver.h)====
SG100HandROSNode::SG100HandROSNode(ros::NodeHandle& nh, ros::NodeHandle& pnh)
    : nh_(nh), pnh_(pnh), is_running_(false),
      left_hand_(nullptr), right_hand_(nullptr), sdk_initialized_(false),
      left_connected_(false), right_connected_(false),
      left_feedback_ready_(false), right_feedback_ready_(false),
      control_hz_(DEFAULT_CONTROL_HZ), left_frame_count_(0), right_frame_count_(0),
      last_left_status_count_(0), last_right_status_count_(0), sg100_debug_state_(false) {}

SG100HandROSNode::~SG100HandROSNode() {
    shutdown();
}

bool SG100HandROSNode::initialize() {
    // Load parameters
    pnh_.param<std::string>("sg100_left_port", left_port_, "/dev/lefthand");
    pnh_.param<std::string>("sg100_right_port", right_port_, "/dev/righthand");
    pnh_.param<int>("sg100_baudrate", baudrate_, 1000000);
    // SG100 参数统一加 sg100_ 前缀：避免 /control_rate、/position_kp 这类泛名与其它模块撞车。
    // 驱动只在 end_effector_type == heiman 时被创建（见 hardware_node.cc），故这些参数仅黑漫机型生效。
    pnh_.param<double>("sg100_control_rate", control_hz_, DEFAULT_CONTROL_HZ);
    pnh_.param<std::string>("sg100_left_hand_name", left_hand_name_, "LEFT_HAND");
    pnh_.param<std::string>("sg100_right_hand_name", right_hand_name_, "RIGHT_HAND");
    // Low-pass filter coefficient for position commands: 0 < alpha <= 1.
    // alpha = 1  → no filtering (pass-through)
    // alpha → 0  → heavy smoothing (slow response)
    pnh_.param<double>("sg100_cmd_filter_alpha", cmd_filter_alpha_, 0.2);
    if (!std::isfinite(control_hz_) || control_hz_ < 50 || control_hz_ > 100 ||
        !std::isfinite(cmd_filter_alpha_) || cmd_filter_alpha_ <= 0 || cmd_filter_alpha_ > 1) {
        ROS_ERROR("SG100 invalid rate (50..100 Hz) or filter alpha (0..1]");
        return false;
    }
    const HighlyDynamic::SG100CommandParameters defaults;
    pnh_.param<float>("sg100_output_limit", output_limit_, defaults.output_limit);
    pnh_.param<float>("sg100_kp", pid_kp_, defaults.kp);
    pnh_.param<float>("sg100_ki", pid_ki_, 0.0f);
    pnh_.param<float>("sg100_kd", pid_kd_, defaults.kd);
    pnh_.param<float>("sg100_imp_kp", imp_kp_, 0.005f);
    pnh_.param<float>("sg100_imp_kd", imp_kd_, 0.005f);
    const auto validParam = [](float value, float maximum) {
        return std::isfinite(value) && value >= 0 && value <= maximum;
    };
    if (!validParam(pid_kp_, 30) || !validParam(imp_kp_, 30) ||
        !validParam(pid_kd_, 10) || !validParam(imp_kd_, 10) ||
        !validParam(pid_ki_, 10) || !validParam(output_limit_, 1)) {
        ROS_ERROR("SG100 invalid gain/output parameter");
        return false;
    }
    if (pid_kp_ > 10 || imp_kp_ > 10) ROS_WARN("SG100 kp capped to SDK maximum 10");
    pid_kp_ = std::min(pid_kp_, 10.0f);
    imp_kp_ = std::min(imp_kp_, 10.0f);
    pnh_.param<bool>("sg100_debug_state", sg100_debug_state_, false);
    ROS_INFO("SG100 Hand ROS Node initializing...");
    ROS_INFO("  Left hand: %s on %s", left_hand_name_.c_str(), left_port_.c_str());
    ROS_INFO("  Right hand: %s on %s", right_hand_name_.c_str(), right_port_.c_str());
    ROS_INFO("  Baudrate: %d", baudrate_);
    ROS_INFO("  Control rate: %.1f Hz", control_hz_);
    ROS_INFO("  Output limit SDK: %.3f (direct encoding)  PID kp/ki/kd: %.4f / %.4f / %.4f",
             output_limit_,
             pid_kp_, pid_ki_, pid_kd_);
    ROS_INFO("  Impedance kp/kd: %.4f / %.4f", imp_kp_, imp_kd_);

    // Legacy rest defaults remain available to tools, but are never startup targets.
    std::vector<float> unused_left, unused_right;
    loadSg100HardcodedDefaults(joint_names_, unused_left, unused_right);
    current_cmd_.left_hand_positions.assign(JOINT_COUNT, 0);
    current_cmd_.right_hand_positions.assign(JOINT_COUNT, 0);
    current_cmd_.left_enable_mask = current_cmd_.right_enable_mask = 0;
    current_cmd_.left_control_mode = kuavo_msgs::SG100HandCommand::MODE_NULL;
    current_cmd_.right_control_mode = kuavo_msgs::SG100HandCommand::MODE_NULL;
    left_filtered_pos_.assign(JOINT_COUNT, 0);
    right_filtered_pos_.assign(JOINT_COUNT, 0);
    left_feedback_ready_ = false;
    right_feedback_ready_ = false;
    last_left_status_count_ = 0;
    last_right_status_count_ = 0;

    ROS_INFO("  Cmd filter alpha: %.3f", cmd_filter_alpha_);

    // Pre-allocate state vectors to avoid out-of-bounds access in readHandState()
    current_state_.left_hand_positions.resize(JOINT_COUNT, 0.0f);
    current_state_.right_hand_positions.resize(JOINT_COUNT, 0.0f);
    current_state_.left_hand_positions_deg.resize(JOINT_COUNT, 0.0f);
    current_state_.right_hand_positions_deg.resize(JOINT_COUNT, 0.0f);
    current_state_.left_hand_velocities.resize(JOINT_COUNT, 0.0f);
    current_state_.right_hand_velocities.resize(JOINT_COUNT, 0.0f);
    current_state_.left_hand_currents.resize(JOINT_COUNT, 0.0f);
    current_state_.right_hand_currents.resize(JOINT_COUNT, 0.0f);
    current_state_.left_hand_currents_d.resize(JOINT_COUNT, 0.0f);
    current_state_.right_hand_currents_d.resize(JOINT_COUNT, 0.0f);
    current_state_.left_hand_modes.resize(JOINT_COUNT, MODE_JOINT_POSITION);
    current_state_.right_hand_modes.resize(JOINT_COUNT, MODE_JOINT_POSITION);
    current_state_.left_hand_connected = false;
    current_state_.right_hand_connected = false;
    current_state_.left_error_code = 0;
    current_state_.right_error_code = 0;

    // Setup ROS subscribers and publishers
    setupROSInterfaces();

    // Initialize SG100 SDK
    if (!initializeSDK()) {
        ROS_ERROR("SG100 SDK initialization failed!");
        shutdownSDK();
        return false;
    }

    readHandState();

    is_running_ = true;

    // Start control thread
    control_thread_ = std::thread(&SG100HandROSNode::controlLoop, this);

    // 手势服务跟手驱动一起注册。VR 未启动时前端也能同步手势库。
    // 同进程直送 current_cmd_,不依赖 /sg100_hand_command 订阅何时被 spin。
    gesture_ = std::make_unique<HighlyDynamic::SG100HandBridge>(nh_, &pnh_);
    gesture_->setConfiguredHands(!left_port_.empty(), !right_port_.empty());
    gesture_->setInputResumeHandler([this] {
        vr_filter_reseed_after_.store(monotonicNow());
    });
    gesture_->setCommandHandler([this](const kuavo_msgs::SG100HandCommand& cmd) {
        handCommandCallback(
            boost::make_shared<kuavo_msgs::SG100HandCommand>(cmd));
    });
    gesture_->start();

    ROS_INFO("SG100 Hand ROS Node initialized successfully!");
    return true;


}

int SG100HandROSNode::getHandJointsNum() const {
    const int configured = (left_port_.empty() ? 0 : 1) +
                           (right_port_.empty() ? 0 : 1);
    return JOINT_COUNT * configured;
}

void SG100HandROSNode::shutdown() {
    gesture_.reset();
    if (!is_running_ && !sdk_initialized_) return;

    ROS_INFO("Shutting down SG100 Hand ROS Node...");
    is_running_ = false;

    if (control_thread_.joinable()) {
        control_thread_.join();
    }

    // Reconnect work owns the corresponding SDK device while it is running.
    // Wait before sending the final release frame or clearing the SDK manager.
    for (auto& future : reconnect_futures_) {
        if (future.valid()) {
            try {
                future.get();
            } catch (const std::exception& e) {
                ROS_ERROR("SG100 reconnect worker failed during shutdown: %s", e.what());
            }
        }
    }

    // Release every motor before closing the serial devices. A zero ROS mask is
    // represented on the SDK wire by MODE_NULL for every joint.
    releaseAllJointControl();

    // Shutdown SDK
    shutdownSDK();

    ROS_INFO("SG100 Hand ROS Node shutdown complete.");
}

void SG100HandROSNode::setupROSInterfaces() {
    // Subscribe to hand commands
    cmd_sub_ = nh_.subscribe("/sg100_hand_command", 256,
                             &SG100HandROSNode::handCommandCallback, this);

    // Publish hand state
    state_pub_ = nh_.advertise<kuavo_msgs::SG100HandState>("/sg100_hand_state", 10);

    // Publish filtered command (the exact values sent to SDK after low-pass filtering)
    // on a dedicated topic using a dedicated message type, so /sg100_hand_command and
    // /sg100_hand_state stay free of filtered data.
    filtered_cmd_pub_ = nh_.advertise<kuavo_msgs::SG100HandFilteredCommand>(
        "/sg100_hand_filtered_command", 10);
}

double SG100HandROSNode::monotonicNow() {
    return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}

void SG100HandROSNode::handCommandCallback(const kuavo_msgs::SG100HandCommand::ConstPtr& msg) {
    if (!sg100_safety::validate(*msg)) {
        ROS_WARN_THROTTLE(1.0, "Reject entire SG100 command: invalid size/value/mode/mask/range");
        return;
    }
    // The control thread alone owns commands, state, SDK handles and safety state.
    std::lock_guard<std::mutex> lock(cmd_mutex_);
    const double received = monotonicNow();
    constexpr size_t kMaxPendingSources = 64;
    const auto protectedSource = [&](uint32_t id) {
        return (mailbox_owned_[0] && mailbox_owners_[0] == id) ||
               (mailbox_owned_[1] && mailbox_owners_[1] == id);
    };
    if (!pending_commands_.count(msg->source_id) && pending_commands_.size() >= kMaxPendingSources) {
        // Keep bounded memory, with reserved admission for existing owners.
        if (!protectedSource(msg->source_id)) {
            ROS_WARN_THROTTLE(1.0, "SG100 pending source limit reached");
            return;
        }
        for (auto it = pending_commands_.begin(); it != pending_commands_.end(); ++it) {
            if (!protectedSource(it->first)) { pending_commands_.erase(it); break; }
        }
    }
    auto& pending = pending_commands_[msg->source_id];
    if (!pending.message) pending.first_received = received;
    pending.message = msg;
    pending.received = received;
}

void SG100HandROSNode::resetHandCommand(unsigned side) {
    auto& modes = side ? current_cmd_.right_hand_control_mode : current_cmd_.left_hand_control_mode;
    auto& positions = side ? current_cmd_.right_hand_positions : current_cmd_.left_hand_positions;
    auto& mask = side ? current_cmd_.right_enable_mask : current_cmd_.left_enable_mask;
    auto& global_mode = side ? current_cmd_.right_control_mode : current_cmd_.left_control_mode;
    mask = 0;
    global_mode = kuavo_msgs::SG100HandCommand::MODE_NULL;
    modes.assign(JOINT_COUNT, 0);
    positions.assign(JOINT_COUNT, 0);
    safety_[side].filter_ready = false;
    safety_[side].sent_position = false;
    safety_[side].force_null_once = false;
    safety_[side].stall.timing.fill(false);
}

void SG100HandROSNode::applyPendingCommand() {
    std::map<uint32_t, PendingCommand> pending;
    {
        std::lock_guard<std::mutex> lock(cmd_mutex_);
        pending.swap(pending_commands_);
    }
    const double now = monotonicNow();
    std::vector<PendingCommand> ordered;
    for (const auto& item : pending) ordered.push_back(item.second);
    std::sort(ordered.begin(), ordered.end(), [](const PendingCommand& a, const PendingCommand& b) {
        return a.first_received < b.first_received;
    });
    for (const auto& entry : ordered) {
        const auto& msg = entry.message;
        if (now - entry.received >= sg100_safety::kCommandTimeoutSec) {
            ROS_WARN_THROTTLE(1.0, "Reject stale SG100 command");
            continue;
        }
        const uint16_t active[2] = {
            sg100_safety::activeMask(msg->control_mode, msg->left_enable_mask, msg->left_hand_control_mode),
            sg100_safety::activeMask(msg->control_mode, msg->right_enable_mask, msg->right_hand_control_mode)};
        bool applied = false;
        for (unsigned side = 0; side < 2; ++side) {
            auto& safe = safety_[side];
            if (!active[side]) {
                // A source may stop its own hand; the reserved debug source may
                // stop any hand. An inactive field from another source must not
                // disturb that hand's owner or retained target.
                if (!safe.owner.owned || safe.owner.source == msg->source_id || msg->source_id == 0) {
                    safe.owner.owned = false;
                    resetHandCommand(side);
                    applied = true;
                }
                continue;
            }
            const bool ready = side ? right_feedback_ready_ : left_feedback_ready_;
            if (!sg100_safety::canApplyActiveCommand(
                    safe.owner, msg->source_id, entry.received, safe.accept_after,
                    ready, safe.timeout_latched, safe.reconnecting)) {
                if (side == 0) {
                    ROS_WARN_THROTTLE(1.0, "Skip SG100 left command: unavailable hand or owner conflict (source=%u)", msg->source_id);
                } else {
                    ROS_WARN_THROTTLE(1.0, "Skip SG100 right command: unavailable hand or owner conflict (source=%u)", msg->source_id);
                }
                continue;
            }
            const bool resuming_vr = safe.owner.resumingVr(msg->source_id, entry.received);
            const double resume_after = vr_filter_reseed_after_.load();
            const bool reseed_requested = sg100_safety::needsVrFilterReseed(
                msg->source_id, entry.received, resume_after, safe.vr_filter_reseed_applied);
            const bool handover = safe.owner.claim(msg->source_id, entry.received);
            if (handover) safe.force_null_once = true;
            // Resume from measured pose, never from the pre-disconnect filter.
            // This does not clear stall latches or change the retained target.
            if (resuming_vr || reseed_requested || handover) safe.filter_ready = false;
            if (reseed_requested) safe.vr_filter_reseed_applied = resume_after;
            safe.use_filter = !msg->skip_filter;
            applied = true;
            auto positions = side ? msg->right_hand_positions : msg->left_hand_positions;
            for (int i = 0; i < JOINT_COUNT; ++i) {
                const float limited = std::max(safe.lower[i], std::min(safe.upper[i], positions[i]));
                if (limited != positions[i]) ROS_WARN_THROTTLE(1.0, "SG100 target clamped to joint_angle_map.json");
                positions[i] = limited;
            }
            auto kp = side ? msg->right_hand_kp : msg->left_hand_kp;
            for (auto& gain : kp) {
                if (gain > 10) ROS_WARN_THROTTLE(1.0, "SG100 kp > 10 capped before SDK encoding");
                gain = std::min(gain, 10.0f);
            }
            if (side == 0) {
                if (sg100_debug_state_) {
                    ROS_INFO_THROTTLE(1.0,
                        "SG100 apply cmd: source=%u mode=%d left_mask=0x%04X force_null=%d",
                        msg->source_id, static_cast<int>(msg->control_mode),
                        static_cast<unsigned>(msg->left_enable_mask),
                        safe.force_null_once ? 1 : 0);
                }
                current_cmd_.left_hand_positions = positions;
                current_cmd_.left_control_mode = msg->control_mode;
                current_cmd_.left_enable_mask = msg->left_enable_mask;
                current_cmd_.left_hand_control_mode = msg->left_hand_control_mode;
                current_cmd_.left_hand_kp = kp;
                current_cmd_.left_hand_kd = msg->left_hand_kd;
                current_cmd_.left_hand_torque_ff = msg->left_hand_torque_ff;
                current_cmd_.left_hand_output_limit = msg->left_hand_output_limit;
            } else {
                current_cmd_.right_hand_positions = positions;
                current_cmd_.right_control_mode = msg->control_mode;
                current_cmd_.right_enable_mask = msg->right_enable_mask;
                current_cmd_.right_hand_control_mode = msg->right_hand_control_mode;
                current_cmd_.right_hand_kp = kp;
                current_cmd_.right_hand_kd = msg->right_hand_kd;
                current_cmd_.right_hand_torque_ff = msg->right_hand_torque_ff;
                current_cmd_.right_hand_output_limit = msg->right_hand_output_limit;
            }
        }
        if (applied) current_cmd_.header = msg->header;
    }
    // Callback reads only this owner snapshot, never mutable SDK/safety state.
    std::lock_guard<std::mutex> lock(cmd_mutex_);
    for (unsigned side = 0; side < 2; ++side) {
        mailbox_owned_[side] = safety_[side].owner.owned;
        mailbox_owners_[side] = safety_[side].owner.source;
    }
}

bool SG100HandROSNode::initializeSDK() {
    const std::string base = getSg100ConfigSearchDir();
    if (base.empty()) { ROS_ERROR("SG100 config package unavailable"); return false; }
    left_config_dir_ = base + "/left";
    right_config_dir_ = base + "/right";
    auto& manager = SG100DeviceManager::Instance();
    sdk_initialized_ = true; // Cleanup also covers partially initialized devices.
    for (unsigned side = 0; side < 2; ++side) {
        const auto& dir = side ? right_config_dir_ : left_config_dir_;
        const auto& port = side ? right_port_ : left_port_;
        if (port.empty()) continue;
        SG100AngleMapParams map;
        const auto result = LoadJointAngleMapFile(dir + "/joint_angle_map.json",
            side ? SG100AngleMapHand::RIGHT : SG100AngleMapHand::LEFT, map);
        // SDK prioritizes v3 when present: fail rather than silently use another mapping.
        if (!result.success || std::ifstream(dir + "/joint_angle_map_v3.0.json").good()) {
            ROS_ERROR("SG100 requires valid legacy joint_angle_map.json without a v3 override: %s", dir.c_str());
            return false;
        }
        for (int i = 0; i < JOINT_COUNT; ++i) {
            safety_[side].map_a[i] = map[i].a;
            safety_[side].map_b[i] = map[i].b;
            safety_[side].lower[i] = map[i].min_angle * static_cast<float>(kPi / 180.0);
            safety_[side].upper[i] = map[i].max_angle * static_cast<float>(kPi / 180.0);
        }
        const auto& name = side ? right_hand_name_ : left_hand_name_;
        if (!manager.AddDeviceByParams(name, port, baudrate_)) return false;
        SG100Device*& hand = side ? right_hand_ : left_hand_;
        hand = manager.GetDevice(name);
        if (!hand) return false;
        hand->SetConfigSearchDir(dir);
        const SG100AngleMapHand angle_hand =
            side ? SG100AngleMapHand::RIGHT : SG100AngleMapHand::LEFT;
        if (!hand->SetAngleMapHand(angle_hand) || !hand->ValidateAngleMap()) {
            const SG100AngleMapStatus status = hand->GetAngleMapStatus();
            ROS_ERROR("SG100 %s angle map activation failed: %s",
                      side ? "right" : "left", status.error_message.c_str());
            return false;
        }
        hand->RegisterErrorCallback(onHandError);
        // Reconnection is asynchronous so one broken hand cannot stall the other's 100 Hz loop.
        safety_[side].timeout_latched = true;
    }
    return true;
}

void SG100HandROSNode::updateProtection() {
    const double now = monotonicNow();
    for (unsigned side = 0; side < 2; ++side) {
        auto& safe = safety_[side];
        SG100Device* hand = side ? right_hand_ : left_hand_;
        bool& connected = side ? right_connected_ : left_connected_;
        bool& ready = side ? right_feedback_ready_ : left_feedback_ready_;

        if (safe.owner.owned && now - safe.owner.last_command >= sg100_safety::kCommandTimeoutSec) {
            if (side == 0) {
                ROS_WARN_THROTTLE(1.0,
                    "SG100 left command timeout: holding the last target (source=%u, age=%.3fs)",
                    safe.owner.source, now - safe.owner.last_command);
            } else {
                ROS_WARN_THROTTLE(1.0,
                    "SG100 right command timeout: holding the last target (source=%u, age=%.3fs)",
                    safe.owner.source, now - safe.owner.last_command);
            }
        }

        if (safe.owner.expire(now)) {
            ROS_WARN("SG100 %s VR owner expired after %.1f s; last target retained, other sources may take over",
                     side ? "right" : "left", sg100_safety::kVrOwnerTimeoutSec);
        }

        if (safe.reconnecting) {
            auto& future = reconnect_futures_[side];
            if (!future.valid() ||
                future.wait_for(std::chrono::milliseconds(0)) != std::future_status::ready) {
                continue;
            }
            bool ok = false;
            try {
                ok = future.get();
            } catch (const std::exception& e) {
                ROS_ERROR("SG100 %s reconnect worker exception: %s",
                          side ? "right" : "left", e.what());
            }
            safe.last_reconnect = monotonicNow();
            safe.reconnecting = false;
            connected = ok;
            ready = false;
            if (ok) {
                safe.awaiting_recovery = true;
                (side ? last_right_status_count_ : last_left_status_count_) = 0;
                // Timeout stays latched until readHandState observes a fresh,
                // finite realtime frame. The retained target is then resumed.
                ROS_INFO("SG100 %s reconnected in MODE_NULL; retaining last target",
                         side ? "right" : "left");
            } else {
                safe.awaiting_recovery = false;
                safe.timeout_latched = true;
                ROS_ERROR("SG100 %s reconnect failed; will retry",
                          side ? "right" : "left");
            }
            continue;
        }

        if (!hand && !safe.timeout_latched) continue;
        if (hand && connected && !ready && !safe.timeout_latched) {
            safe.timeout_latched = true;
            safe.awaiting_recovery = false;
            safe.transport_reopen_required = true;
            safe.realtime_anchor_valid = false;
            safe.physical_position_valid.fill(false);
            safe.filter_ready = false;
            safe.sent_position = false;
            safe.last_reconnect = now;
            ROS_ERROR("SG100 %s feedback timeout: holding last target and starting recovery",
                      side ? "right" : "left");
        }
        if (safe.timeout_latched && now - safe.last_reconnect >= kReconnectRetryIntervalSec) {
            safe.last_reconnect = now;
            safe.reconnecting = true;
            safe.realtime_anchor_valid = false;
            safe.physical_position_valid.fill(false);
            safe.filter_ready = false;
            safe.sent_position = false;
            connected = ready = false;
            try {
                reconnect_futures_[side] = std::async(std::launch::async,
                    [this, side]() { return reconnectHand(side); });
            } catch (const std::exception& e) {
                safe.reconnecting = false;
                ROS_ERROR("SG100 %s failed to start reconnect worker: %s",
                          side ? "right" : "left", e.what());
            }
        }
    }
}

bool SG100HandROSNode::reconnectHand(unsigned side) {
    SG100Device*& hand = side ? right_hand_ : left_hand_;
    const std::string& port = side ? right_port_ : left_port_;
    const std::string& dir = side ? right_config_dir_ : left_config_dir_;
    if (port.empty() || !hand) {
        return false;
    }

    auto& safe = safety_[side];
    const bool link_open = !safe.transport_reopen_required &&
                           hand->IsRunning() && hand->IsSerialOpen();
    if (!link_open) {
        ROS_WARN_THROTTLE(1.0, "SG100 %s reopening serial transport on %s",
                          side ? "right" : "left", port.c_str());
        hand->Stop();
        hand->SetConfigSearchDir(dir);
        const SG100AngleMapHand angle_hand =
            side ? SG100AngleMapHand::RIGHT : SG100AngleMapHand::LEFT;
        if (!hand->SetAngleMapHand(angle_hand) || !hand->ValidateAngleMap()) {
            const SG100AngleMapStatus status = hand->GetAngleMapStatus();
            ROS_ERROR("SG100 %s reconnect angle map activation failed: %s",
                      side ? "right" : "left", status.error_message.c_str());
            return false;
        }
        hand->RegisterErrorCallback(onHandError);

        if (!hand->Init() || !hand->Start()) {
            ROS_WARN_THROTTLE(1.0, "SG100 %s serial reopen failed on %s",
                              side ? "right" : "left", port.c_str());
            return false;
        }
        safe.transport_reopen_required = false;
    }

    // Fast readiness probe: do not enter the 3-second passthrough timeout while
    // the palm is still booting or disconnected.
    Palm_Runtime_Mode_Status_t readiness{};
    if (!queryPalmRuntimeMode(hand, readiness, kReconnectReadinessWaitMs, 1)) {
        ROS_WARN_THROTTLE(1.0, "SG100 %s not ready for reconnect", side ? "right" : "left");
        return false;
    }

    Hand_Control_t frame;
    fillNullFrame(frame);
    // dcl 启动/重连流程：先透传读取 11 个 raw.joint_angle，
    // 再切回 realtime 并建立 physical/logic anchor。
    // The outer protection loop owns retry timing. Do not block this reconnect
    // round with three consecutive SDK timeouts.
    if (!switchToPassthroughWithRetry(hand, 1)) {
        return false;
    }
    std::array<float, 11> physical_deg{};
    if (!readPhysicalUserAngles(side, physical_deg)) {
        return false;
    }
    if (!ensureRealtimeMode(hand)) {
        return false;
    }
    // DCL confirms the palm mode after switching back. A successful SDK switch
    // return alone is not enough: the palm can still report passthrough and
    // then accept state traffic while ignoring position commands.
    Palm_Runtime_Mode_Status_t mode_status{};
    if (!queryPalmRuntimeMode(hand, mode_status, 250, 2)) {
        ROS_WARN("SG100 %s realtime mode query failed after switch", side ? "right" : "left");
        return false;
    }
    if (!(mode_status.realtime_enabled &&
          mode_status.mode == PALM_RUNTIME_MODE_REALTIME)) {
        ROS_WARN("SG100 %s realtime confirmation failed: mode=%u realtime=%u",
                 side ? "right" : "left",
                 static_cast<unsigned>(mode_status.mode),
                 static_cast<unsigned>(mode_status.realtime_enabled));
        return false;
    }
    if (!hand->SetHandControl(frame)) {
        return false;
    }
    if (!waitForRealtimeTraffic(hand, 1000)) {
        return false;
    }
    return establishRealtimeAnchor(side, physical_deg);
}

bool SG100HandROSNode::readPhysicalUserAngles(unsigned side,
                                              std::array<float, 11>& out_user_deg) {
    SG100Device* hand = side ? right_hand_ : left_hand_;
    if (!hand) {
        return false;
    }
    auto& safe = safety_[side];
    constexpr float kRadToDeg = 180.0f / static_cast<float>(kPi);
    for (int i = 0; i < JOINT_COUNT; ++i) {
        const uint8_t joint_id = static_cast<uint8_t>(i + 1);
        if (!hand->GetRawSnapshot(joint_id)) {
            ROS_ERROR("SG100 %s J%d raw snapshot request failed",
                      side ? "right" : "left", i + 1);
            return false;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
        RawSnapshotFrame_t raw{};
        bool received = false;
        const auto deadline = std::chrono::steady_clock::now() +
                              std::chrono::milliseconds(500);
        while (std::chrono::steady_clock::now() < deadline) {
            if (hand->GetJointRawSnapshot(joint_id, raw) &&
                hasRawSnapshotPayload(raw)) {
                received = true;
                break;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        if (!received || !std::isfinite(raw.joint_angle)) {
            ROS_ERROR("SG100 %s J%d raw snapshot missing or invalid",
                      side ? "right" : "left", i + 1);
            return false;
        }
        if (!std::isfinite(safe.map_a[i]) || !std::isfinite(safe.map_b[i]) ||
            std::fabs(safe.map_a[i]) < 1.0e-6f) {
            ROS_ERROR("SG100 %s J%d invalid legacy angle map",
                      side ? "right" : "left", i + 1);
            return false;
        }
        const float actual_deg = raw.joint_angle * kRadToDeg;
        const float user_deg = (actual_deg - safe.map_b[i]) / safe.map_a[i];
        if (!std::isfinite(user_deg)) {
            ROS_ERROR("SG100 %s J%d physical angle is NaN/Inf",
                      side ? "right" : "left", i + 1);
            return false;
        }
        out_user_deg[i] = user_deg;
    }
    return true;
}

bool SG100HandROSNode::establishRealtimeAnchor(
    unsigned side, const std::array<float, 11>& physical_deg) {
    SG100Device* hand = side ? right_hand_ : left_hand_;
    if (!hand) {
        return false;
    }
    auto& safe = safety_[side];
    const Hand_Status_t status = hand->GetCachedHandStatus();
    for (int i = 0; i < JOINT_COUNT; ++i) {
        const uint8_t mode = status.joints[i].mode;
        if (!std::isfinite(physical_deg[i]) ||
            !std::isfinite(status.joints[i].position) ||
            (mode != MODE_NULL && mode != MODE_JOINT_POSITION &&
             mode != MODE_JOINT_IMPEDANCE)) {
            return false;
        }
    }
    safe.physical_anchor_deg = physical_deg;
    safe.physical_position_deg = physical_deg;
    for (int i = 0; i < JOINT_COUNT; ++i) {
        safe.logic_anchor_deg[i] = status.joints[i].position;
        safe.previous_control_mode[i] = status.joints[i].mode;
        safe.physical_position_valid[i] = true;
    }
    safe.realtime_anchor_valid = true;
    return true;
}

void SG100HandROSNode::updatePhysicalPosition(unsigned side) {
    auto& safe = safety_[side];
    SG100Device* hand = side ? right_hand_ : left_hand_;
    if (!hand || !safe.realtime_anchor_valid) {
        return;
    }
    const Hand_Status_t status = hand->GetCachedHandStatus();
    for (int i = 0; i < JOINT_COUNT; ++i) {
        const uint8_t mode = status.joints[i].mode;
        const float logic_deg = status.joints[i].position;
        if (!std::isfinite(logic_deg)) {
            safe.physical_position_valid[i] = false;
            safe.previous_control_mode[i] = mode;
            continue;
        }

        if (mode == MODE_JOINT_POSITION || mode == MODE_JOINT_IMPEDANCE) {
            safe.physical_position_deg[i] = logic_deg;
            safe.physical_anchor_deg[i] = logic_deg;
            safe.logic_anchor_deg[i] = logic_deg;
            safe.physical_position_valid[i] = true;
        } else if (mode == MODE_NULL) {
            if (safe.previous_control_mode[i] != MODE_NULL) {
                safe.logic_anchor_deg[i] = logic_deg;
                safe.physical_anchor_deg[i] = safe.physical_position_deg[i];
            }
            safe.physical_position_deg[i] =
                safe.physical_anchor_deg[i] +
                (logic_deg - safe.logic_anchor_deg[i]) / 250.0f;
            safe.physical_position_valid[i] = true;
        } else {
            safe.physical_position_valid[i] = false;
        }
        safe.previous_control_mode[i] = mode;
    }
}

void SG100HandROSNode::releaseAllJointControl() {
    if (!sdk_initialized_) {
        return;
    }

    {
        std::lock_guard<std::mutex> lock(cmd_mutex_);
        current_cmd_.left_enable_mask = 0;
        current_cmd_.right_enable_mask = 0;
        current_cmd_.left_control_mode = kuavo_msgs::SG100HandCommand::MODE_NULL;
        current_cmd_.right_control_mode = kuavo_msgs::SG100HandCommand::MODE_NULL;
        current_cmd_.left_hand_control_mode.assign(
            JOINT_COUNT, kuavo_msgs::SG100HandCommand::MODE_NULL);
        current_cmd_.right_hand_control_mode.assign(
            JOINT_COUNT, kuavo_msgs::SG100HandCommand::MODE_NULL);
    }

    Hand_Control_t left_ctrl;
    Hand_Control_t right_ctrl;
    std::memset(&left_ctrl, 0, sizeof(left_ctrl));
    std::memset(&right_ctrl, 0, sizeof(right_ctrl));
    for (int i = 0; i < JOINT_COUNT; ++i) {
        left_ctrl.joints[i].id = static_cast<uint8_t>(i + 1);
        right_ctrl.joints[i].id = static_cast<uint8_t>(i + 1);
        left_ctrl.joints[i].mode = kuavo_msgs::SG100HandCommand::MODE_NULL;
        right_ctrl.joints[i].mode = kuavo_msgs::SG100HandCommand::MODE_NULL;
    }

    ROS_INFO("Releasing all SG100 joints (left_mask=0x0000, right_mask=0x0000)...");
    constexpr int kReleaseSendCount = 3;
    for (int attempt = 0; attempt < kReleaseSendCount; ++attempt) {
        if (left_hand_ && !left_hand_->SetHandControl(left_ctrl)) {
            ROS_WARN("Failed to send release command to left hand (attempt %d/%d)",
                     attempt + 1, kReleaseSendCount);
        }
        if (right_hand_ && !right_hand_->SetHandControl(right_ctrl)) {
            ROS_WARN("Failed to send release command to right hand (attempt %d/%d)",
                     attempt + 1, kReleaseSendCount);
        }
        if (attempt + 1 < kReleaseSendCount) {
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }
    }
}

void SG100HandROSNode::shutdownSDK() {
    ROS_INFO("Shutting down SG100 SDK...");

    if (sdk_initialized_) {
        try {
            auto& manager = SG100DeviceManager::Instance();
            if (left_hand_) {
                left_hand_->Stop();
            }
            if (right_hand_) {
                right_hand_->Stop();
            }
            manager.Clear();
        } catch (const std::exception& e) {
            ROS_ERROR("Exception during SDK shutdown: %s", e.what());
        }
    }

    left_hand_ = nullptr;
    right_hand_ = nullptr;
    left_connected_ = false;
    right_connected_ = false;
    left_feedback_ready_ = false;
    right_feedback_ready_ = false;
    sdk_initialized_ = false;

    ROS_INFO("SG100 SDK shutdown complete");
}

void SG100HandROSNode::sendCommandToHands() {
    if (!sdk_initialized_) {
        return;
    }

    // Prepare control structures (control thread owns command/state storage)
    Hand_Control_t left_ctrl;
    Hand_Control_t right_ctrl;
    std::memset(&left_ctrl, 0, sizeof(left_ctrl));
    std::memset(&right_ctrl, 0, sizeof(right_ctrl));
    std::array<bool, 2> force_null{};
    std::array<uint16_t, 2> active_masks{};

    for (unsigned side = 0; side < 2; ++side) {
        auto& safe = safety_[side];
        const bool ready = side ? right_feedback_ready_ : left_feedback_ready_;
        auto& filtered = side ? right_filtered_pos_ : left_filtered_pos_;
        const auto& target = side ? current_cmd_.right_hand_positions : current_cmd_.left_hand_positions;
        const auto& feedback = side ? current_state_.right_hand_positions : current_state_.left_hand_positions;
        const auto& velocity = side ? current_state_.right_hand_velocities : current_state_.left_hand_velocities;
        const auto& current = side ? current_state_.right_hand_currents : current_state_.left_hand_currents;
        const auto& modes = side ? current_cmd_.right_hand_control_mode : current_cmd_.left_hand_control_mode;
        const uint16_t mask = side ? current_cmd_.right_enable_mask : current_cmd_.left_enable_mask;
        const int8_t global_mode = side ? current_cmd_.right_control_mode : current_cmd_.left_control_mode;
        const uint16_t active = sg100_safety::activeMask(global_mode, mask, modes);
        active_masks[side] = active;
        safe.sent_position = false;
        if (!ready || safe.timeout_latched || safe.reconnecting || !active) continue;
        if (safe.force_null_once) {
            force_null[side] = true;
            continue;
        }
        if (!safe.filter_ready) { filtered = feedback; safe.filter_ready = true; }
        const float alpha = safe.use_filter ? static_cast<float>(cmd_filter_alpha_) : 1.0f;
        for (int i = 0; i < JOINT_COUNT; ++i) {
            filtered[i] = std::max(safe.lower[i], std::min(safe.upper[i],
                alpha * target[i] + (1.0f - alpha) * filtered[i]));
            const auto event = safe.stall.update(
                i, monotonicNow(),
                (active & (1u << i)) && !(safe.stall.latched & (1u << i)),
                current[i], velocity[i], feedback[i], filtered[i]);
            if (event == sg100_safety::ProtectionEvent::STALL_TRIP) {
                ROS_ERROR("SG100 %s J%d stall: current=%.3f A, velocity=%.3f rad/s, error=%.3f rad for %.1f s; latched until driver restart",
                          side ? "right" : "left", i + 1,
                          static_cast<double>(std::abs(current[i])),
                          static_cast<double>(std::abs(velocity[i])),
                          static_cast<double>(std::abs(filtered[i] - feedback[i])),
                          sg100_safety::kStallDurationSec);
            }
        }
    }

    // Cached global defaults (encoded once)
    const uint16_t pid_p     = SG100_Quant::encodePid(pid_kp_);
    const uint16_t pid_i     = SG100_Quant::encodePid(pid_ki_);
    const uint16_t pid_d     = SG100_Quant::encodePid(pid_kd_);
    const uint16_t out_lim   = encodeSg100OutputLimit(output_limit_);
    const uint16_t imp_kp_enc = SG100_Quant::encodePid(imp_kp_);
    const uint16_t imp_kd_enc = SG100_Quant::encodePid(imp_kd_);
    const uint16_t imp_out_lim = encodeSg100OutputLimit(output_limit_);

    // Lambda: resolve per-joint mode value — per-joint array wins, else global fallback.
    auto resolveJointMode = [](const std::vector<int8_t>& per_joint_modes,
                               int idx, int8_t global_mode) -> int8_t {
        if (static_cast<int>(per_joint_modes.size()) == JOINT_COUNT) {
            return per_joint_modes[idx];
        }
        return global_mode;
    };

    // Modes that use the position frame fields (MODE_JOINT_POSITION=7).
    auto isPositionMode = [](int8_t mode) -> bool {
        return mode == kuavo_msgs::SG100HandCommand::MODE_JOINT_POSITION;
    };

    // Modes that use the impedance frame fields (MODE_JOINT_IMPEDANCE=9).
    auto isImpedanceMode = [](int8_t mode) -> bool {
        return mode == kuavo_msgs::SG100HandCommand::MODE_JOINT_IMPEDANCE;
    };

    // Lambda: fill one joint's control fields.
    auto fillJoint = [&](Hand_Control_t& ctrl, int i, bool enabled, float pos_deg,
                          int8_t global_mode,
                          const std::vector<int8_t>& per_joint_modes,
                          const std::vector<float>& per_joint_kp,
                          const std::vector<float>& per_joint_kd,
                          const std::vector<float>& per_joint_torque_ff,
                          const std::vector<float>& per_joint_output_limit) {
        ctrl.joints[i].id = static_cast<uint8_t>(i + 1);
        int8_t mode = enabled ? resolveJointMode(per_joint_modes, i, global_mode)
                              : kuavo_msgs::SG100HandCommand::MODE_NULL;

        const bool use_imp = isImpedanceMode(mode);
        const bool use_pos = isPositionMode(mode);
        if (mode != kuavo_msgs::SG100HandCommand::MODE_NULL && !use_imp && !use_pos) {
            ROS_WARN_THROTTLE(1.0,
                "SG100 J%d: unsupported control mode %d (only 7 position, 9 impedance, 0 NULL) — holding joint",
                i + 1, static_cast<int>(mode));
            mode = kuavo_msgs::SG100HandCommand::MODE_NULL;
        }

        // Resolve per-joint gains (fall back to global ros params / global control_mode)
        const uint16_t kp_enc = (static_cast<int>(per_joint_kp.size()) == JOINT_COUNT)
            ? SG100_Quant::encodePid(per_joint_kp[i])
            : (use_imp ? imp_kp_enc : pid_p);
        const uint16_t kd_enc = (static_cast<int>(per_joint_kd.size()) == JOINT_COUNT)
            ? SG100_Quant::encodePid(per_joint_kd[i])
            : (use_imp ? imp_kd_enc : pid_d);
        const uint16_t olim   = (static_cast<int>(per_joint_output_limit.size()) == JOINT_COUNT)
            ? encodeSg100OutputLimit(per_joint_output_limit[i])
            : (use_imp ? imp_out_lim : out_lim);
        const uint16_t tff    = (static_cast<int>(per_joint_torque_ff.size()) == JOINT_COUNT)
            ? SG100_Quant::encodeOutLim(per_joint_torque_ff[i])
            : SG100_Quant::encodeOutLim(0.0f);

        // Mode value passes through unchanged (msg enums == SDK CtrlMode_e).
        ctrl.joints[i].mode = static_cast<uint8_t>(mode);
        if (use_imp) {
            ctrl.joints[i].imp_position    = pos_deg;
            ctrl.joints[i].imp_velocity    = SG100_Quant::encodeVel(0.0f);
            ctrl.joints[i].imp_kp          = kp_enc;
            ctrl.joints[i].imp_kd          = kd_enc;
            ctrl.joints[i].imp_torque_ff   = tff;
            ctrl.joints[i].imp_output_limit = olim;
        } else if (use_pos) {
            ctrl.joints[i].position      = pos_deg;
            ctrl.joints[i].p             = kp_enc;
            ctrl.joints[i].i             = pid_i;  // integral gain is always global
            ctrl.joints[i].d             = kd_enc;
            ctrl.joints[i].output_limit  = olim;
        }
    };

    // Fill left hand (rad → deg for SDK)
    constexpr float kRad2Deg = static_cast<float>(180.0 / kPi);
    for (int i = 0; i < JOINT_COUNT; ++i) {
        const bool enabled = !force_null[0] && left_feedback_ready_ && !safety_[0].timeout_latched &&
            !safety_[0].reconnecting && current_cmd_.left_control_mode != 0 &&
            !(safety_[0].stall.latched & (1u << i)) && (current_cmd_.left_enable_mask & (1u << i));
        const float pos_deg = left_filtered_pos_[i] * kRad2Deg;
        fillJoint(left_ctrl, i, enabled, pos_deg, current_cmd_.left_control_mode,
                  current_cmd_.left_hand_control_mode,
                  current_cmd_.left_hand_kp,
                  current_cmd_.left_hand_kd,
                  current_cmd_.left_hand_torque_ff,
                  current_cmd_.left_hand_output_limit);
    }

    // Fill right hand (rad → deg for SDK)
    for (int i = 0; i < JOINT_COUNT; ++i) {
        const bool enabled = !force_null[1] && right_feedback_ready_ && !safety_[1].timeout_latched &&
            !safety_[1].reconnecting && current_cmd_.right_control_mode != 0 &&
            !(safety_[1].stall.latched & (1u << i)) && (current_cmd_.right_enable_mask & (1u << i));
        const float pos_deg = right_filtered_pos_[i] * kRad2Deg;
        fillJoint(right_ctrl, i, enabled, pos_deg, current_cmd_.right_control_mode,
                  current_cmd_.right_hand_control_mode,
                  current_cmd_.right_hand_kp,
                  current_cmd_.right_hand_kd,
                  current_cmd_.right_hand_torque_ff,
                  current_cmd_.right_hand_output_limit);
    }

    if (sg100_debug_state_) {
        ROS_INFO_THROTTLE(1.0,
            "SG100 send L: force_null=%d mode0=%u pos0=%.3f p0=%u d0=%u out0=%u work=%d mode_fb=%u ready=%d timeout=%d reconnecting=%d mask=0x%04X",
            force_null[0] ? 1 : 0,
            static_cast<unsigned>(left_ctrl.joints[0].mode),
            static_cast<double>(left_ctrl.joints[0].position),
            static_cast<unsigned>(left_ctrl.joints[0].p),
            static_cast<unsigned>(left_ctrl.joints[0].d),
            static_cast<unsigned>(left_ctrl.joints[0].output_limit),
            left_hand_ ? static_cast<int>(left_hand_->GetWorkMode()) : -1,
            current_state_.left_hand_modes.empty()
                ? 255u : static_cast<unsigned>(current_state_.left_hand_modes[0]),
            left_feedback_ready_ ? 1 : 0,
            safety_[0].timeout_latched ? 1 : 0,
            safety_[0].reconnecting ? 1 : 0,
            static_cast<unsigned>(current_cmd_.left_enable_mask));
    }

    // A stale link must receive no ordinary frame at all. Sending the zeroed
    // structure here would be an implicit MODE_NULL and violate hold-last-target.
    if (left_hand_ && left_connected_ && left_feedback_ready_ &&
        !safety_[0].timeout_latched && !safety_[0].reconnecting) {
        const bool sent = left_hand_->SetHandControl(left_ctrl);
        if (sent && force_null[0]) safety_[0].force_null_once = false;
        safety_[0].sent_position = sent && safety_[0].filter_ready &&
            !force_null[0] && active_masks[0] != 0;
        if (!sent) ROS_ERROR_THROTTLE(1.0, "Failed to send SG100 left command");
    }
    if (right_hand_ && right_connected_ && right_feedback_ready_ &&
        !safety_[1].timeout_latched && !safety_[1].reconnecting) {
        const bool sent = right_hand_->SetHandControl(right_ctrl);
        if (sent && force_null[1]) safety_[1].force_null_once = false;
        safety_[1].sent_position = sent && safety_[1].filter_ready &&
            !force_null[1] && active_masks[1] != 0;
        if (!sent) ROS_ERROR_THROTTLE(1.0, "Failed to send SG100 right command");
    }
}

void SG100HandROSNode::readHandState() {
    current_state_.header.stamp = ros::Time::now();
    current_state_.header.frame_id = "sg100_hands";
    const double now = monotonicNow();
    for (unsigned side = 0; side < 2; ++side) {
        auto& safe = safety_[side];
        SG100Device* hand = side ? right_hand_ : left_hand_;
        const bool connected = side ? right_connected_ : left_connected_;
        bool& ready = side ? right_feedback_ready_ : left_feedback_ready_;
        uint32_t& last = side ? last_right_status_count_ : last_left_status_count_;
        uint32_t& count = side ? right_frame_count_ : left_frame_count_;
        auto& positions = side ? current_state_.right_hand_positions : current_state_.left_hand_positions;
        auto& velocities = side ? current_state_.right_hand_velocities : current_state_.left_hand_velocities;
        auto& currents = side ? current_state_.right_hand_currents : current_state_.left_hand_currents;
        auto& currents_d = side ? current_state_.right_hand_currents_d : current_state_.left_hand_currents_d;
        auto& modes = side ? current_state_.right_hand_modes : current_state_.left_hand_modes;
        if (hand && connected && !safe.reconnecting) {
            count = hand->GetStatusRecvCount();
            if (count != last && count != 0) {
                const Hand_Status_t status = hand->GetCachedHandStatus();
                bool finite = true;
                for (int i = 0; i < JOINT_COUNT; ++i)
                    finite = finite && std::isfinite(status.joints[i].position);
                last = count;
                if (finite) {
                    safe.feedback_time = now;
                    if (safe.timeout_latched && safe.awaiting_recovery &&
                        safe.realtime_anchor_valid) {
                        safe.timeout_latched = false;
                        safe.awaiting_recovery = false;
                        safe.accept_after = now;
                        // Seed the command filter from this recovered physical
                        // pose before converging to the retained target.
                        safe.filter_ready = false;
                    }
                    constexpr float d2r = static_cast<float>(kPi / 180.0);
                    updatePhysicalPosition(side);
                    for (int i = 0; i < JOINT_COUNT; ++i) {
                        positions[i] = safe.physical_position_valid[i]
                            ? safe.physical_position_deg[i] * d2r
                            : status.joints[i].position * d2r;
                        velocities[i] = SG100_Quant::decodeVel(status.joints[i].velocity) * d2r;
                        currents[i] = SG100_Quant::decodeCurQ(status.joints[i].cur_q);
                        currents_d[i] = SG100_Quant::decodeCurD(status.joints[i].cur_d);
                        modes[i] = status.joints[i].mode;
                    }
                }
            }
        }
        ready = connected && !safe.reconnecting && !safe.timeout_latched &&
            safe.feedback_time >= 0 && now - safe.feedback_time <= 0.5;
        (side ? current_state_.right_hand_connected : current_state_.left_hand_connected) = connected;
        (side ? current_state_.right_error_code : current_state_.left_error_code) =
            !connected ? 2 : !ready ? 1 : safe.stall.latched ? 3 : 0;
    }
    current_state_.left_frame_count = left_frame_count_;
    current_state_.right_frame_count = right_frame_count_;
}

void SG100HandROSNode::publishState() {
    // Publish SG100HandState message (11-dim, direct copy — no PALM_ROT to skip)
    kuavo_msgs::SG100HandState state_msg;
    state_msg.header = current_state_.header;
    state_msg.left_hand_connected = current_state_.left_hand_connected;
    state_msg.right_hand_connected = current_state_.right_hand_connected;
    state_msg.left_hand_connected = left_connected_;
    state_msg.right_hand_connected = right_connected_;
    state_msg.left_error_code = !left_connected_ ? 2 : !left_feedback_ready_ ? 1 : safety_[0].stall.latched ? 3 : 0;
    state_msg.right_error_code = !right_connected_ ? 2 : !right_feedback_ready_ ? 1 : safety_[1].stall.latched ? 3 : 0;
    state_msg.left_hand_positions = current_state_.left_hand_positions;
    state_msg.right_hand_positions = current_state_.right_hand_positions;
    state_msg.left_hand_velocities = current_state_.left_hand_velocities;
    state_msg.right_hand_velocities = current_state_.right_hand_velocities;
    state_msg.left_hand_currents = current_state_.left_hand_currents;
    state_msg.right_hand_currents = current_state_.right_hand_currents;
    state_msg.left_hand_currents_d = current_state_.left_hand_currents_d;
    state_msg.right_hand_currents_d = current_state_.right_hand_currents_d;
    state_msg.left_hand_modes = current_state_.left_hand_modes;
    state_msg.right_hand_modes = current_state_.right_hand_modes;
    state_msg.left_frame_count = current_state_.left_frame_count;
    state_msg.right_frame_count = current_state_.right_frame_count;

    state_msg.left_feedback_valid = left_feedback_ready_ && !safety_[0].timeout_latched;
    state_msg.right_feedback_valid = right_feedback_ready_ && !safety_[1].timeout_latched;
    const double now = monotonicNow();
    state_msg.left_feedback_age = safety_[0].feedback_time < 0 ? std::numeric_limits<float>::infinity() : now - safety_[0].feedback_time;
    state_msg.right_feedback_age = safety_[1].feedback_time < 0 ? std::numeric_limits<float>::infinity() : now - safety_[1].feedback_time;
    if (sg100_debug_state_) {
        ROS_INFO_THROTTLE(0.5, "SG100 feedback valid L/R=%d/%d age=%.3f/%.3f stall=0x%03x/0x%03x",
            state_msg.left_feedback_valid, state_msg.right_feedback_valid,
            state_msg.left_feedback_age, state_msg.right_feedback_age,
            safety_[0].stall.latched, safety_[1].stall.latched);

        // Debug diagnostics only: these thresholds never change control mode,
        // latch an error, or send MODE_NULL. Current is the Q-axis feedback
        // reported by each joint board; position limits come from the active
        // legacy joint_angle_map.json loaded for that hand.
        constexpr float kDebugCurrentWarnA = 0.2f;
        constexpr float kDebugPositionLimitToleranceRad = 0.1f;
        std::ostringstream current_values;
        std::ostringstream current_warnings;
        std::ostringstream position_warnings;
        current_values << std::fixed << std::setprecision(3);
        current_warnings << std::fixed << std::setprecision(3);
        position_warnings << std::fixed << std::setprecision(3);
        bool have_current_values = false;
        bool have_current_warning = false;
        bool have_position_warning = false;

        for (unsigned side = 0; side < 2; ++side) {
            const bool feedback_valid = side
                ? state_msg.right_feedback_valid : state_msg.left_feedback_valid;
            if (!feedback_valid) continue;

            const auto& currents = side
                ? state_msg.right_hand_currents : state_msg.left_hand_currents;
            const auto& positions = side
                ? state_msg.right_hand_positions : state_msg.left_hand_positions;
            if (currents.size() != JOINT_COUNT || positions.size() != JOINT_COUNT) continue;

            if (have_current_values) current_values << " ";
            current_values << (side ? "R[" : "L[");
            for (int i = 0; i < JOINT_COUNT; ++i) {
                if (i != 0) current_values << " ";
                current_values << "J" << (i + 1) << "=" << currents[i];

                if (std::isfinite(currents[i]) &&
                    std::abs(currents[i]) > kDebugCurrentWarnA) {
                    if (have_current_warning) current_warnings << ", ";
                    current_warnings << (side ? "R " : "L ")
                                     << "J" << (i + 1) << "=" << currents[i] << "A";
                    have_current_warning = true;
                }

                const float position = positions[i];
                const float lower = safety_[side].lower[i];
                const float upper = safety_[side].upper[i];
                if (std::isfinite(position) &&
                    (position < lower - kDebugPositionLimitToleranceRad ||
                     position > upper + kDebugPositionLimitToleranceRad)) {
                    if (have_position_warning) position_warnings << ", ";
                    position_warnings << (side ? "R " : "L ")
                                      << "J" << (i + 1) << "=" << position
                                      << "rad outside [" << lower << "," << upper << "]rad";
                    have_position_warning = true;
                }
            }
            current_values << "]";
            have_current_values = true;
        }

        if (have_current_values) {
            const std::string values = current_values.str();
            ROS_INFO_THROTTLE(0.5, "SG100 Q-axis currents(A): %s", values.c_str());
        }
        if (have_current_warning) {
            const std::string warnings = current_warnings.str();
            ROS_WARN_THROTTLE(1.0, "SG100 current warning (|Iq| > 0.200A): %s",
                              warnings.c_str());
        }
        if (have_position_warning) {
            const std::string warnings = position_warnings.str();
            ROS_WARN_THROTTLE(1.0, "SG100 joint angle outside JSON limit (+/-0.1rad tolerance): %s",
                              warnings.c_str());
        }
    }
    state_pub_.publish(state_msg);

    // Publish filtered command (the exact values sent to SDK this cycle, after low-pass filtering)
    publishFilteredCommand();
}

void SG100HandROSNode::publishFilteredCommand() {
    const double now = monotonicNow();
    if (now - last_filtered_publish_ < 0.019) return;
    last_filtered_publish_ = now;
    if (!safety_[0].sent_position && !safety_[1].sent_position) return;
    kuavo_msgs::SG100HandFilteredCommand msg;
    msg.header.stamp = ros::Time::now();
    // NaN explicitly marks disabled/faulted/unconfigured joints in this fixed 11+11 interface.
    msg.left_hand_filtered_positions.assign(JOINT_COUNT, std::numeric_limits<float>::quiet_NaN());
    msg.right_hand_filtered_positions.assign(JOINT_COUNT, std::numeric_limits<float>::quiet_NaN());
    for (unsigned side = 0; side < 2; ++side) {
        const auto& safe = safety_[side];
        if (!safe.sent_position) continue;
        auto& out = side ? msg.right_hand_filtered_positions : msg.left_hand_filtered_positions;
        const auto& filtered = side ? right_filtered_pos_ : left_filtered_pos_;
        const auto& modes = side ? current_cmd_.right_hand_control_mode : current_cmd_.left_hand_control_mode;
        const auto mask = side ? current_cmd_.right_enable_mask : current_cmd_.left_enable_mask;
        const int8_t global_mode = side ? current_cmd_.right_control_mode : current_cmd_.left_control_mode;
        const auto active = sg100_safety::activeMask(global_mode, mask, modes) & ~safe.stall.latched;
        for (int i = 0; i < JOINT_COUNT; ++i) if (active & (1u << i)) out[i] = filtered[i];
    }
    filtered_cmd_pub_.publish(msg);
}

void SG100HandROSNode::controlLoop() {
    ros::WallRate rate(control_hz_);

    ROS_INFO("SG100 control loop started at %.1f Hz", control_hz_);

    while (is_running_ && ros::ok()) {
        // The SDK is read only here; ROS callbacks only enqueue validated messages.
        readHandState();
        updateProtection();
        applyPendingCommand();

        // Send commands to hands (after state update so filter can seed from real hardware pose)
        sendCommandToHands();

        // Publish state
        publishState();

        rate.sleep();
    }
}
