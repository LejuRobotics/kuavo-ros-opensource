#include "lejuclaw_can_customed.h"
#include <sstream>

namespace lejuclaw_can {

static constexpr uint8_t kEnterMotorStateFrameLocal[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC};
static constexpr uint8_t kEnterResetStateFrameLocal[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD};
static constexpr uint8_t kMultiTurnZeroFrameLocal[8]   = {0x67, 0x06, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x76};

// 日志宏定义
#define LEJUCLAW_LOG_ERROR(fmt, ...) printf("\033[31m[LeJuClawCan] ERROR: " fmt "\033[0m\n", ##__VA_ARGS__)
#define LEJUCLAW_LOG_WARN(fmt, ...) printf("\033[33m[LeJuClawCan] WARN: " fmt "\033[0m\n", ##__VA_ARGS__)
#define LEJUCLAW_LOG_INFO(fmt, ...) printf("[LeJuClawCan] INFO: " fmt "\n", ##__VA_ARGS__)

using motorevo::MotorId;

LeJuClawCan::LeJuClawCan() = default;
LeJuClawCan::~LeJuClawCan() {
    if (thread_running || control_thread_.joinable()) {
        close();
    }
}

void LeJuClawCan::set_debug_callback(DebugCallback callback) {
    debug_callback_ = std::move(callback);
}

void LeJuClawCan::set_tx_callback(TxCallback callback) {
    std::lock_guard<std::mutex> lock(tx_callback_mutex_);
    tx_callback_ = std::move(callback);
}

void LeJuClawCan::emit_debug(const std::string& json) const {
    if (debug_callback_) {
        debug_callback_(json);
    }
}

// 更新当前电机的状态
void LeJuClawCan::internalMessageCallback(canbus_sdk::CanMessageFrame* frame, const canbus_sdk::CallbackContext* context) {
    if(!frame) {
        return;
    }

    if (!context || !context->userdata) {
        if (frame) canbus_sdk::freeCanMessageFrame(frame);
        return;
    }

    // 从context获取类实例指针
    LeJuClawCan* controller = static_cast<LeJuClawCan*>(context->userdata);
    if (!controller) {
        canbus_sdk::freeCanMessageFrame(frame);
        return;
    }

    // The feedback decoder consumes all eight bytes, including status/temperature.
    if (canbus_sdk::dlc_to_payload_length(frame->ctrl.rx.dlc) < 8) {
        canbus_sdk::freeCanMessageFrame(frame);
        return;
    }

    // 创建反馈帧并解析
    uint8_t payload[8];
    memcpy(payload, frame->payload, 8);
    motorevo::FeedbackFrame feedback_frame(payload);
    // !!! 重要！释放CAN帧
    canbus_sdk::freeCanMessageFrame(frame);

    // 查找对应的电机控制数据
    MotorId motor_id = feedback_frame.motor_id();
    auto data_it = controller->motor_ctrl_datas_.find(motor_id);
    if (data_it == controller->motor_ctrl_datas_.end()) {
        printf("Received CAN message for unknown motor ID: 0x%02X\n", motor_id);
        return;
    }

    // 更新电机状态、更新电机控制状态为收到反馈
    data_it->second.motor.receiveFeedback(feedback_frame);
    data_it->second.feedback_health.received();
    data_it->second.feedback_received = true;

    // printf("Received feedback from motor %d: pos=%.4f, vel=%.4f, torque=%.4f\n",
    //          motor_id, feedback_frame.position(), feedback_frame.velocity(), feedback_frame.torque());
}

// 得知数据什么时候发出去，因为 controlPTM 里面的 sendCanMessage 是写入 BM 的消息队列，BM 使用后会返回使用这个回调
void LeJuClawCan::internalTefEventCallback(canbus_sdk::CanMessageFrame* frame, const canbus_sdk::CallbackContext* context) {
    if(!frame) {
        return;
    }

    if (!context || !context->userdata) {
        if (frame) canbus_sdk::freeCanMessageFrame(frame);
        return;
    }

    // 从context获取类实例指针
    LeJuClawCan* controller = static_cast<LeJuClawCan*>(context->userdata);
    if (!controller) {
        canbus_sdk::freeCanMessageFrame(frame);
        return;
    }

    MotorId motor_id = static_cast<MotorId>(frame->id.SID); // CAN_ID ---> MotorId

    // 拷贝 payload
    uint8_t payload[8];
    memcpy(payload, frame->payload, 8);

    // 释放CAN帧
    canbus_sdk::freeCanMessageFrame(frame);

    // 查找电机控制数据
    auto data_it = controller->motor_ctrl_datas_.find(motor_id);
    if (data_it == controller->motor_ctrl_datas_.end()) {
        printf("TEF callback for unknown motor ID: 0x%02X\n", motor_id);
        return;
    }

    data_it->second.feedback_health.tef();

    if (data_it->second.operation_status != OperationStatus::PENDING) {
        // 减少 memcmp 次数
        return;
    }
    
    // 根据payload判断是不是控制指令
    if (memcmp(payload, kEnterMotorStateFrameLocal, 8) == 0) {
        // enable指令发送成功
        if (data_it->second.operation == Operation::ENABLE &&
            data_it->second.operation_status == OperationStatus::PENDING) {
            data_it->second.operation_status = OperationStatus::SUCCESS;
            printf("Motor 0x%02X enable command sent successfully\n", motor_id);
        }
    } else if (memcmp(payload, kEnterResetStateFrameLocal, 8) == 0) {
        // disable指令发送成功
        if (data_it->second.operation == Operation::DISABLE &&
            data_it->second.operation_status == OperationStatus::PENDING) {
            data_it->second.operation_status = OperationStatus::SUCCESS;
            printf("Motor 0x%02X disable command sent successfully\n", motor_id);
        }
    } else if (memcmp(payload, kMultiTurnZeroFrameLocal, 8) == 0) {
        // multi-turn zero指令发送成功
        if (data_it->second.operation == Operation::MULTI_TURN_ZERO &&
            data_it->second.operation_status == OperationStatus::PENDING) {
            data_it->second.operation_status = OperationStatus::SUCCESS;
            printf("Motor 0x%02X multi-turn zero command sent successfully\n", motor_id);
        }
    }
}

bool LeJuClawCan::Connect(const canbus_sdk::DeviceConfig& config) {

    using namespace canbus_sdk;
    if(config.device_type != DeviceType::LEJUCLAW) {
        LEJUCLAW_LOG_ERROR("Device type is not LEJUCLAW");
        return false;
    }

    auto &bus_name = config.canbus_name;
    auto& canbus_controller = canbus_sdk::CanBusController::getInstance();
    // 需要外部已经打开总线 这里不负责打开，只负责注册
    // 先通过canbus_name获取bus_id，检查总线是否存在
    auto bus_id_result = canbus_controller.getBusIdByName(bus_name);
    if (!bus_id_result.has_value()) {
        LEJUCLAW_LOG_ERROR("Cannot find bus_id for canbus_name '%s'", bus_name.c_str());
        return false;
    }
    canbus_sdk::BusId bus_id = bus_id_result.value();
    RWLOG_I("Using CAN bus %s with bus_id: %d", bus_name.c_str(), bus_id);

    // 初始化回调上下文
    msg_callback_context_.userdata = this;
    tef_callback_context_.userdata = this;

    // 总线存在，则创建设备句柄
    canbus_sdk::CallbackParams callback_params{
        .msg_callback = internalMessageCallback,
        .msg_cb_ctx = &msg_callback_context_,
        .tef_callback = internalTefEventCallback,
        .tef_cb_ctx = &tef_callback_context_
    };

    // 注册设备到CAN总线
    auto register_result = canbus_controller.registerDevice(canbus_sdk::DeviceInfo(config.name, config.device_type, config.device_id, nullptr), bus_name, callback_params);

    if (!register_result.has_value()) {
        auto error = register_result.error();
        if (error == static_cast<canbus_sdk::ErrorType>(canbus_sdk::CanBusError::ERROR_DEVICE_ALREADY_EXISTS)) {
            LEJUCLAW_LOG_WARN("Device already exists for device_id %d on bus '%s'", config.device_id, bus_name.c_str());
        } else {
            LEJUCLAW_LOG_ERROR("Failed to register device for device_id %d, error: %d", config.device_id, static_cast<int>(error));
            return false;
        }   
    }

    // DeviceConfig -> RevoMotorConfig_t
    motorevo::MotorId motor_id = static_cast<motorevo::MotorId>(config.device_id);
    // 初始化电机参数
    motorevo::RevoMotorConfig_t motor_config;
    motor_config.id = motor_id;
    motor_config.name = config.name;
    motor_config.negtive = config.negtive;
    motor_config.ignore = config.ignore;
    motor_config.zero_offset = 0.0f;
    motor_config.ratio = config.ratio;
    motor_config.default_params.vel = config.motor_params[0];
    motor_config.default_params.kp_pos = config.motor_params[1];
    motor_config.default_params.kd_pos = config.motor_params[2];
    motor_config.default_params.tor = config.motor_params[3];
    motor_config.default_params.kp_vel = config.motor_params[4];
    motor_config.default_params.kd_vel = config.motor_params[5];
    motor_config.default_params.ki_vel = config.motor_params[6];

    motorevo::RevoMotor motor(bus_id, motor_id, motorevo::MotorMode::TorquePositionMixControlMode);

    motor_ctrl_datas_.emplace(
        std::piecewise_construct,
        std::forward_as_tuple(motor_id),
        std::forward_as_tuple(false, std::move(motor), motor_config)
    );

    return true;
}

bool LeJuClawCan::init_lejuclaw_can_customed()
{
    using namespace canbus_sdk;
    auto config_file = ConfigParser::getDefaultConfigFilePath();

    /** 配置文件读取与解析 */
    canbus_sdk::ConfigParser parser;
    if (!parser.parseFromFile(config_file)) { // 会抛异常(无所谓), 配置文件都解析失败,程序就没法玩了
        printf("\033[31m[LejuClawController] ERROR: Failed to parse config file: %s\033[0m\n", config_file.c_str());
        return false;
    }

    /** 获取CAN总线配置, 没有就返回失败 */
    auto canbus_configs = parser.getCanBusConfigs();
    if (canbus_configs.empty()) {
        printf("\033[31m[LejuClawController] ERROR: No CAN bus configurations found in config file\033[0m\n");
        return false;
    }

    canbus_sdk::CanBusController::getInstance().init(); // 重复初始化无所谓
    std::vector<DeviceConfig> lejuclaw_devices;
    for (const auto &canbus_config : canbus_configs) { 
        auto devices = parser.getDevices(canbus_config.name, canbus_sdk::DeviceType::LEJUCLAW);
        if(devices.empty()) {
            continue;
        }

        // 找到夹爪设备配置在CAN总线上，那么就可以后续的初始化操作
        auto result = canbus_sdk::CanBusController::getInstance().openCanBus(
            canbus_config.name.c_str(), canbus_config.type, canbus_config.bitrate);
        if (result.has_value()) {
            // 总线打开成功，准备把这些设备后续注册到总线
            for (auto &d : devices) {
                lejuclaw_devices.push_back(d);
            }
        }
        else {
            printf("\033[31m[LejuClawController] ERROR: 打开CAN总线 %s 失败: %s\033[0m\n", canbus_config.name.c_str(),canbus_sdk::errorToString(result.error()));
        }
    }

    if(lejuclaw_devices.empty()) {
        printf("\033[31m[LejuClawController] ERROR: 没有找到夹爪设备\033[0m\n");
        return false;
    }

    // 初始化设备!
    // 定义设备初始化函数
    auto init_device = [this, &lejuclaw_devices](const std::string& device_name, const char* hand_desc) -> bool {
        auto it = std::find_if(lejuclaw_devices.begin(), lejuclaw_devices.end(),
            [&](const canbus_sdk::DeviceConfig& device) { return device.name == device_name; });

        if (it != lejuclaw_devices.end()) {
            // 检查设备是否被忽略
            if (it->ignore) {
                printf("\033[33m[LejuClawController] WARN: %s LeJuClawCan 设备被忽略，跳过初始化\033[0m\n", hand_desc);
                return false;
            }

            bool lejuclaw = Connect(*it);
            if (lejuclaw) {
                // 记录 device_id -> canbus_name 映射
                MotorId id = static_cast<MotorId>(it->device_id);
                device_canbus_map_[id] = it->canbus_name;
            }
            printf("%s[LejuClawController] %s%s LeJuClawCan 设备%s\033[0m\n",
                   lejuclaw ? "\033[32m" : "\033[31m", lejuclaw ? "成功初始化" : "初始化", hand_desc, lejuclaw ? "" : "失败");
            return lejuclaw;
        }
        return false;
    };

    bool left = init_device("Llejuclaw", "左夹爪");
    bool right = init_device("Rlejuclaw", "右夹爪");
    if (!left && !right) {
        printf("\033[31m[LejuClawController] ERROR: 未找到任何可用的夹爪设备\033[0m\n");
        return false;
    } else if (!left) {
        printf("\033[31m[LejuClawController] ERROR: 未找到左夹爪设备\033[0m\n");
        return false;
    } else if (!right) {
        printf("\033[31m[LejuClawController] ERROR: 未找到右夹爪设备\033[0m\n");
        return false;
    }

    return true;
}

bool LeJuClawCan::waitForAllMotorsFeedback(int timeout_ms) {
    auto start_time = std::chrono::steady_clock::now();

    while (true) {
        // 检查是否超时
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - start_time);
        if (elapsed.count() >= timeout_ms) {
            RWLOG_W("Timeout waiting for feedback from all motors after %d ms", timeout_ms);
            return false;
        }

        // 检查是否所有非忽略电机都收到了反馈
        bool all_feedback_received = true;
        for (const auto& [id, data] : motor_ctrl_datas_) {
            if (!data.config.ignore && !data.feedback_received) {
                all_feedback_received = false;
            }
        }

        if (all_feedback_received) {
            RWLOG_SUCCESS("All %d motors have received feedback", motor_ctrl_datas_.size());
            return true;
        }

        // 10ms检查一次
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}

void LeJuClawCan::init_status_variables() {
    // 使用 PTM，target_cmd 以 MotorId 为键，命令为值
    target_cmd.clear();
    old_target_cmd.clear();
    current_cmd.clear();
    non_vr_hold_mode_.clear();
    non_vr_hold_target_rad_.clear();
    non_vr_hold_current_.clear();
    last_move_paw_target_percent_.clear();
    // 触发值集合在 load_leju_params 中初始化，此处仅确保存在时默认值
    if (non_vr_hold_trigger_set_.empty()) {
        non_vr_hold_trigger_set_.push_back(100.0f);
    }
    // RevoMotorCmd_t 原意是指控制命令，这里我意为当前电机的状态
    for (const auto& motor : motor_ctrl_datas_) {
        MotorId id = motor.first;
        const auto& data = motor.second;
        motorevo::RevoMotorCmd_t cmd{};
        cmd.pos = data.motor.position();
        cmd.vel = data.motor.velocity();
        cmd.torque = data.motor.torque();
        // 从默认参数初始化 KP/KD
        cmd.kp = data.config.default_params.kp_pos;
        cmd.kd = data.config.default_params.kd_pos;
        target_cmd[id] = cmd;
        old_target_cmd[id] = cmd;
        current_cmd[id] = cmd;
        non_vr_hold_mode_[id] = false;
        non_vr_hold_target_rad_[id] = 0.0f;
        non_vr_hold_current_[id] = 0.0f;
        last_move_paw_target_percent_[id] = -1.0;
    }
}

std::unordered_map<MotorId, FeedbackSnapshot> LeJuClawCan::get_feedback_health() const {
    std::unordered_map<MotorId, FeedbackSnapshot> result;
    const auto now = FeedbackHealth::nowNs();
    for (const auto& entry : motor_ctrl_datas_) {
        if (!entry.second.config.ignore)
            result.emplace(entry.first, entry.second.feedback_health.snapshot(now));
    }
    return result;
}

void LeJuClawCan::control_thread() {
    // 基于 init_status_variables 初始化的三个映射：
    // - target_cmd: 期望命令（写入）
    // - old_target_cmd: 上一周期命令（用于变化检测/插值）
    // - current_cmd: 当前反馈（读出）
    // 线程职责：
    // 1) 定周期将 target_cmd 下发给各电机（通过 data.motor.controlPTM），并做方向/零点处理在 motor_ctrl write 前置；
    // 2) 采样最新反馈到 current_cmd（轻量加锁，避免频繁锁争用）；
    // 3) 简单的变化边沿判定（若 target 与 old_target 差异较大，可插值/限幅）。

    auto last_refresh = std::chrono::steady_clock::time_point{};
    auto last_health_report = std::chrono::steady_clock::time_point{};
    std::unordered_map<MotorId, bool> was_fresh;
    while (thread_running) {
        std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(CONTROL_LOOP_DT * 1000)));
        const auto now = std::chrono::steady_clock::now();
        const bool report_due = now - last_health_report >= std::chrono::seconds(1);
        for (const auto& entry : get_feedback_health()) {
            const auto id = entry.first;
            const auto& health = entry.second;
            if (report_due || !was_fresh.count(id) || was_fresh[id] != health.fresh) {
                std::ostringstream out;
                out << "{\"source\":\"can_sdk\",\"event\":\"feedback_health\","
                    << "\"motor_id\":" << static_cast<int>(id)
                    << ",\"rx_count\":" << health.rx_count
                    << ",\"rx_age_ms\":" << health.age_ms
                    << ",\"fresh\":" << (health.fresh ? "true" : "false")
                    << ",\"timeout_ms\":" << FeedbackHealth::kTimeoutMs
                    << ",\"tx_attempts\":" << health.tx_attempts
                    << ",\"tx_enqueue_failures\":" << health.tx_enqueue_failures
                    << ",\"tef_count\":" << health.tef_count << "}";
                emit_debug(out.str());
                was_fresh[id] = health.fresh;
            }
        }
        if (report_due) last_health_report = now;

        // 更新 current_cmd：只需读电机原始数据，做零点/方向处理后写入（供上层使用）
        {
            std::lock_guard<std::mutex> lock(state_lock);
            for (auto& [id, data] : motor_ctrl_datas_) {
                motorevo::RevoMotorCmd_t cur{};
                float pos = data.motor.position();
                float vel = data.motor.velocity();
                float tor = data.motor.torque();

                if (data.config.negtive) {
                    pos = -pos;
                    vel = -vel;
                    tor = -tor;
                }

                cur.pos = pos;
                cur.vel = vel;
                cur.torque = tor;
                cur.kp = data.config.default_params.kp_pos;
                cur.kd = data.config.default_params.kd_pos;
                current_cmd[id] = cur;
            }
        }

        // ===== 断流看门狗：高频控制流断流 → 权威下发安全 PTM（kp=0 防堵转） =====
        bool stream_watchdog_active = false;
        bool stream_watchdog_just_expired = false;
        {
            std::lock_guard<std::mutex> lock(stream_watchdog_mutex_);
            if (stream_watchdog_armed_ && !stream_watchdog_expired_) {
                auto time_since_last_cmd = std::chrono::duration_cast<std::chrono::milliseconds>(
                    std::chrono::steady_clock::now() - last_stream_command_time_).count();
                if (time_since_last_cmd >= cfg_STREAM_WATCHDOG_TIMEOUT_MS) {
                    stream_watchdog_expired_ = true;
                    stream_watchdog_just_expired = true;
                }
            }
            stream_watchdog_active = stream_watchdog_armed_ && stream_watchdog_expired_;
        }
        if (stream_watchdog_just_expired) {
            LEJUCLAW_LOG_WARN("[LeJuClawCan]:stream watchdog expired（高频控制流断流 %d ms），安全 PTM：kp=0、力矩清零、位置保持最后目标",
                              cfg_STREAM_WATCHDOG_TIMEOUT_MS);
            emit_debug("{\"source\":\"can_sdk\",\"phase\":\"move\",\"event\":\"stream_watchdog_expired\","
                       "\"timeout_ms\":" + std::to_string(cfg_STREAM_WATCHDOG_TIMEOUT_MS) + "}");
        }
        if (stream_watchdog_active) {
            // 构建安全命令缓存：pos = 最后一次指令的目标位置（target_cmd[id].pos）、vel/torque/kp 清零、kd 保留
            std::lock_guard<std::mutex> lock(target_lock);
            safe_command_cache_.clear();
            for (const auto& [id, data] : motor_ctrl_datas_) {
                if (data.config.ignore || !data.isEnabled()) continue;
                motorevo::RevoMotorCmd_t safe{};
                auto it = target_cmd.find(id);
                if (it != target_cmd.end()) {
                    safe.pos = it->second.pos;
                    safe.kd = it->second.kd;
                }
                safe.vel = 0.0;
                safe.torque = 0.0;
                safe.kp = 0.0;
                safe_command_cache_[id] = safe;
            }
            target_updated_ = true;   // 强制每周期走发送路径，权威下发安全 PTM
        }

        // Consume before the snapshot. An update arriving during TX remains set.
        const bool new_target = target_updated_.exchange(false);
        if (!new_target)
        {
            bool any_hold = false;
            for (auto& [id, data] : motor_ctrl_datas_) {
                if (data.config.ignore || !data.isEnabled()) continue;
                if (non_vr_hold_mode_.count(id) && non_vr_hold_mode_[id]) { any_hold = true; break; }
            }
            // PTM feedback is solicited by commands. Refresh the existing target
            // at 20 Hz while idle so a stationary claw still has measurable RX age.
            if (!any_hold && now - last_refresh < std::chrono::milliseconds(50)) { continue; }
        }

        last_refresh = now;

        // 读取期望命令：加锁一次，取快照，降低细粒度锁开销
        std::unordered_map<MotorId, motorevo::RevoMotorCmd_t> targets_snapshot;
        {
            std::lock_guard<std::mutex> lock(target_lock);
            targets_snapshot = target_cmd;
            // 记录已下发命令为 old_target，用于外部需要做差分/插值时参考
            old_target_cmd = targets_snapshot;
        }

        // 下发控制
        for (auto& [id, data] : motor_ctrl_datas_) {
            if (data.config.ignore || !data.isEnabled()) {
                continue;
            }

            // ========== 非VR保持模式（断流看门狗过期时，由下方安全 PTM 权威接管） ==========
            if (!stream_watchdog_active && cfg_ENABLE_NON_VR_HOLD && non_vr_hold_mode_.count(id) && non_vr_hold_mode_[id]) {
                const float logical_hold_current = non_vr_hold_current_[id];
                const float logical_hold_pos = non_vr_hold_target_rad_[id];
                float hold_current = logical_hold_current;
                float hold_pos = logical_hold_pos;
                if (data.config.negtive) {
                    hold_current = -hold_current;
                    hold_pos = -hold_pos;
                }

                TxCallback tx_cb;
                {
                    std::lock_guard<std::mutex> lock(tx_callback_mutex_);
                    tx_cb = tx_callback_;
                }
                if (tx_cb) {
                    tx_cb(static_cast<uint8_t>(id),
                          std::max(5.0f, std::min(95.0f,
                              mapLogicalRadToPercent(id, logical_hold_pos))),
                          0.0f, logical_hold_current, 0.0f, 0.0f, false);
                }
                data.feedback_health.transmitted(
                    data.motor.controlPTM(hold_pos, 0.0f, hold_current, 0.0f, 0.0f));
                continue;
            }

            // 断流看门狗过期时，权威下发安全命令缓存（忽略 target_cmd），防止旧目标持续施力堵转
            const auto& cmd_source = stream_watchdog_active ? safe_command_cache_ : targets_snapshot;
            auto it = cmd_source.find(id);
            if (it == cmd_source.end()) {
                continue;
            }

            // 获取目标命令
            motorevo::RevoMotorCmd_t cmd = it->second;

            // 方向/零点处理：与 LeJuClawCan::write 中一致
            double raw_target_position = cmd.pos;
            double raw_target_velocity = cmd.vel;
            double raw_target_torque = cmd.torque;
            float kp = static_cast<float>(cmd.kp);
            float kd = static_cast<float>(cmd.kd);

            if(data.config.negtive) {
                raw_target_position = -raw_target_position;
                raw_target_velocity = -raw_target_velocity;
                raw_target_torque = -raw_target_torque;
            }


            // std::cout<<"数据输出"<<raw_target_position<<"数据输出"<<raw_target_velocity<<"数据输出"<<raw_target_torque<<"数据输出"<<kp<<"数据输出"<<kd<<std::endl;
            // 观察口：把最终真实下发参数回报上层（在 controlPTM 之前，看到的就是发到电机的值）
            // 位置在驱动内换算为行程百分比（逻辑坐标→%，negtive 翻转前的用户坐标系），vel/torque 同报逻辑值
            TxCallback tx_cb;
            {
                std::lock_guard<std::mutex> lock(tx_callback_mutex_);
                tx_cb = tx_callback_;
            }
            if (tx_cb) {
                tx_cb(static_cast<uint8_t>(id),
                      std::max(5.0f, std::min(95.0f,
                          mapLogicalRadToPercent(id, static_cast<float>(cmd.pos)))),
                      static_cast<float>(cmd.vel),
                      static_cast<float>(cmd.torque),
                      kp, kd, stream_watchdog_active);
            }
            // 下发 PTM
            data.feedback_health.transmitted(data.motor.controlPTM(
                static_cast<float>(raw_target_position),
                static_cast<float>(raw_target_velocity),
                static_cast<float>(raw_target_torque),
                kp,
                kd
            ));
        }
        // Never clear a producer notification after sending.
    }
}

void LeJuClawCan::set_joint_state(const std::unordered_map<MotorId, motorevo::RevoMotorCmd_t>& cmd)
{
    if (!thread_running)
    {
        return;
    }

    std::lock_guard<std::mutex> lock(state_lock);
    current_cmd = cmd;
}

std::unordered_map<MotorId, motorevo::RevoMotorCmd_t> LeJuClawCan::get_joint_state()
{
    if (!thread_running)
    {
        return std::unordered_map<MotorId, motorevo::RevoMotorCmd_t>();
    }
    std::lock_guard<std::mutex> lock(state_lock);
    return current_cmd;
}

bool LeJuClawCan::measure_range()
{
    // 寻找开爪限位（负速度）
    std::cout << "[LEJU claw]:开始寻找开爪限位，超时时间设置为: " << cfg_ZERO_FIND_TIMEOUT_MS << " ms..." << std::endl;
    emit_debug("{\"source\":\"can_sdk\",\"phase\":\"init_open\",\"event\":\"start\"}");

    // 记录初始位置
    std::unordered_map<MotorId, float> open_start_positions;
    
    std::unordered_map<MotorId, motorevo::RevoMotorCmd_t> state = get_joint_state();
    for (auto& [id, data] : motor_ctrl_datas_) {
        if (data.config.ignore || !data.isEnabled()) {
            continue;
        }
        open_start_positions[id] = state[id].pos;
        std::cout << "[LEJU claw]:电机 " << id << " 开爪开始位置: " << open_start_positions[id] << std::endl;
        std::ostringstream oss;
        oss << "{\"source\":\"can_sdk\",\"phase\":\"init_open\",\"event\":\"start_position\","
            << "\"motor_id\":" << static_cast<int>(id) << ","
            << "\"position\":" << open_start_positions[id] << "}";
        emit_debug(oss.str());
    }

    std::unordered_map<MotorId, bool> no_reverse_impact;
    bool open_limit_found = find_claw_limit_velocity_control(true, cfg_ZERO_CONTROL_KP, cfg_ZERO_CONTROL_KD, cfg_ZERO_CONTROL_ALPHA, cfg_ZERO_CONTROL_MAX_CURRENT,
                                                        cfg_STALL_CURRENT_THRESHOLD, cfg_STALL_VELOCITY_THRESHOLD, cfg_ZERO_CONTROL_DT, cfg_ZERO_FIND_TIMEOUT_MS, no_reverse_impact);
    if (!open_limit_found)
    {
        std::cout << "[LEJU claw]:开爪限位寻找超时，认为当前位置为开爪限位，继续寻找关爪限位" << std::endl;
        emit_debug("{\"source\":\"can_sdk\",\"phase\":\"init_open\",\"event\":\"timeout\"}");
    }

    std::unordered_map<MotorId, float> open_limit_positions;
    state = get_joint_state();
    for (auto& [id, data] : motor_ctrl_datas_) {
        if (data.config.ignore || !data.isEnabled()) {
            continue;
        }
        open_limit_positions[id] = state[id].pos;
        float position_change = open_limit_positions[id] - open_start_positions[id];
        std::cout << "[LEJU claw]:电机 " << id << " 开爪限位位置: " << open_limit_positions[id] << std::endl;
        std::cout << "[LEJU claw]:电机 " << id << " 开爪位置变化: " << position_change << " rad" << std::endl;
        std::ostringstream oss;
        oss << "{\"source\":\"can_sdk\",\"phase\":\"init_open\",\"event\":\"limit_position\","
            << "\"motor_id\":" << static_cast<int>(id) << ","
            << "\"position\":" << open_limit_positions[id] << ","
            << "\"position_change\":" << position_change << "}";
        emit_debug(oss.str());
    }

    // 等待一段时间后开始关爪
    std::this_thread::sleep_for(std::chrono::milliseconds(cfg_ZERO_WAIT_MS));
    
    // 检查开爪位置变化情况，记录是否已经明确发生开爪动作。
    // 只有开爪阶段确实动过，关爪阶段才允许自动反向冲击；否则双向不动时停机报警，交给人工确认方向。
    std::unordered_map<MotorId, bool> open_motion_seen_by_id;
    std::unordered_map<MotorId, bool> allow_close_reverse_impact;
    for (auto& [id, data] : motor_ctrl_datas_) {
        if (data.config.ignore || !data.isEnabled()) {
            continue;
        }
        float open_position_change = std::abs(open_limit_positions[id] - open_start_positions[id]);
        bool open_motion_seen = open_position_change > cfg_OPEN_POSITION_CHANGE_THRESHOLD;
        open_motion_seen_by_id[id] = open_motion_seen;
        allow_close_reverse_impact[id] = open_motion_seen;
        if (open_motion_seen) {
            std::cout << "[LEJU claw]:电机 " << id << " 开爪运动正常，位置变化(" << open_position_change << " rad) > " << cfg_OPEN_POSITION_CHANGE_THRESHOLD << " rad" << std::endl;
        } else {
            std::cout << "[LEJU claw]:电机 " << id << " 开爪位置变化较小(" << open_position_change << " rad) < " << cfg_OPEN_POSITION_CHANGE_THRESHOLD << " rad，将继续低速寻找关爪限位；若关爪也不动，将判定为双向卡死" << std::endl;
        }
        std::ostringstream oss;
        oss << "{\"source\":\"can_sdk\",\"phase\":\"init_open\",\"event\":\"impact_permission\","
            << "\"motor_id\":" << static_cast<int>(id) << ","
            << "\"open_position_change\":" << open_position_change << ","
            << "\"open_motion_seen\":" << (open_motion_seen ? "true" : "false") << ","
            << "\"allow_close_reverse_impact\":" << (allow_close_reverse_impact[id] ? "true" : "false") << "}";
        emit_debug(oss.str());
    }

    // 寻找关爪限位（正速度）
    std::cout << "[LEJU claw]:开始寻找关爪限位，超时时间设置为: " << cfg_ZERO_FIND_TIMEOUT_MS << " ms..." << std::endl;
    emit_debug("{\"source\":\"can_sdk\",\"phase\":\"init_close\",\"event\":\"start\"}");
    bool close_limit_found = find_claw_limit_velocity_control(false, cfg_ZERO_CONTROL_KP, cfg_ZERO_CONTROL_KD, cfg_ZERO_CONTROL_ALPHA, cfg_ZERO_CONTROL_MAX_CURRENT,
                                                            cfg_STALL_CURRENT_THRESHOLD, cfg_STALL_VELOCITY_THRESHOLD, cfg_ZERO_CONTROL_DT, cfg_ZERO_FIND_TIMEOUT_MS,
                                                            allow_close_reverse_impact);
    if (!close_limit_found)
    {
        std::cout << "[LEJU claw]:关爪限位寻找超时，认为当前位置为关爪限位，完成夹爪初始化" << std::endl;
        emit_debug("{\"source\":\"can_sdk\",\"phase\":\"init_close\",\"event\":\"timeout\"}");
    }

    // 记录关爪限位位置
    std::unordered_map<MotorId, float> close_limit_positions;
    state = get_joint_state();
    for (auto& [id, data] : motor_ctrl_datas_) {
        if (data.config.ignore || !data.isEnabled()) {
            continue;
        }
        close_limit_positions[id] = state[id].pos;
        float position_change = close_limit_positions[id] - open_start_positions[id];
        float close_position_change = std::abs(close_limit_positions[id] - open_limit_positions[id]);
        std::cout << "[LEJU claw]:电机 " << id << " 关爪限位位置: " << close_limit_positions[id] << std::endl;
        std::cout << "[LEJU claw]:电机 " << id << " 关爪位置变化: " << position_change << " rad" << std::endl;
        std::ostringstream oss;
        oss << "{\"source\":\"can_sdk\",\"phase\":\"init_close\",\"event\":\"limit_position\","
            << "\"motor_id\":" << static_cast<int>(id) << ","
            << "\"position\":" << close_limit_positions[id] << ","
            << "\"position_change\":" << position_change << ","
            << "\"close_position_change\":" << close_position_change << "}";
        emit_debug(oss.str());

        bool open_motion_seen = open_motion_seen_by_id.count(id) ? open_motion_seen_by_id[id] : false;
        bool close_motion_seen = close_position_change > cfg_ZERO_STUCK_POSITION_THRESHOLD;
        if (!close_motion_seen) {
            clear_all_torque();
            std::ostringstream msg;
            InitFault fault = InitFault::CLOSE_NO_MOTION;
            const char* event = "close_no_motion";
            if (!open_motion_seen) {
                fault = InitFault::BIDIRECTION_STUCK;
                event = "bidirection_stuck";
                msg << "电机 " << static_cast<int>(id)
                    << " 初始化开爪和关爪两个方向位置变化都过小，疑似双向卡死；已停止电流，等待人工确认恢复方向";
            } else {
                msg << "电机 " << static_cast<int>(id)
                    << " 开爪阶段有运动，但关爪阶段位置变化过小，禁止使用错误行程继续初始化";
            }
            std::cout << "[LEJU claw]:" << msg.str() << std::endl;
            set_init_fault(fault, msg.str());
            std::ostringstream fault_oss;
            fault_oss << "{\"source\":\"can_sdk\",\"phase\":\"initialize\",\"event\":\"" << event << "\","
                      << "\"motor_id\":" << static_cast<int>(id) << ","
                      << "\"open_motion_seen\":" << (open_motion_seen ? "true" : "false") << ","
                      << "\"close_motion_seen\":false,"
                      << "\"close_position_change\":" << close_position_change << "}";
            emit_debug(fault_oss.str());
            return false;
        }
    }

    // 停止电机
    clear_all_torque();

    // 调整行程范围
    adjust_range(open_limit_positions, close_limit_positions);

    // 等待系统稳定
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    return true;
}

bool LeJuClawCan::find_claw_limit_velocity_control(bool is_open_direction, float kp, float kd, float alpha,
                                               float max_current, float stall_current_threshold, 
                                               float stall_velocity_threshold, float dt, 
                                               int timeout_ms,
                                               const std::unordered_map<MotorId, bool>& allow_reverse_impact)
{
    // 期望速度（rad/s）
    float velocity_abs = std::abs(cfg_TARGET_VELOCITY);
    float velocity_limit = std::max(0.1f, cfg_ZERO_TARGET_VELOCITY_MAX);
    velocity_abs = std::min(velocity_abs, velocity_limit);
    const float target_velocity = is_open_direction ? -velocity_abs : velocity_abs;
    const float stuck_position_threshold = std::max(0.001f, cfg_ZERO_STUCK_POSITION_THRESHOLD);
    {
        std::ostringstream oss;
        oss << "{\"source\":\"can_sdk\",\"phase\":\""
            << (is_open_direction ? "init_open" : "init_close")
            << "\",\"event\":\"find_limit_start\","
            << "\"configured_target_velocity\":" << cfg_TARGET_VELOCITY << ","
            << "\"target_velocity\":" << target_velocity << ","
            << "\"target_velocity_limit\":" << velocity_limit << ","
            << "\"max_current\":" << max_current << ","
            << "\"stall_current_threshold\":" << stall_current_threshold << ","
            << "\"stall_velocity_threshold\":" << stall_velocity_threshold << ","
            << "\"stuck_position_threshold\":" << stuck_position_threshold << "}";
        emit_debug(oss.str());
    }

    // 基于 MotorId 的状态表
    std::unordered_map<MotorId, float> last_positions;
    std::unordered_map<MotorId, float> prev_torque;
    std::unordered_map<MotorId, bool>  joint_limit_found;
    // 关爪专用
    std::unordered_map<MotorId, float> close_start_positions;   // 关爪开始位置
    std::unordered_map<MotorId, bool>  close_attempt_started;   // 是否开始关爪尝试
    std::unordered_map<MotorId, int>   close_attempt_count;     // 关爪尝试次数
    std::unordered_map<MotorId, std::chrono::steady_clock::time_point> close_attempt_start_times;
    std::unordered_map<MotorId, bool>  close_high_current_mode; // 是否处于高电流模式
    std::unordered_map<MotorId, bool>  close_impact_active;     // 是否处于冲击阶段
    std::unordered_map<MotorId, bool>  close_stuck_detection_mode;  // 是否处于关爪卡死检测模式
    std::unordered_map<MotorId, bool>  close_normal_limit_detection_enabled;    // 是否启用正常限位检测
    std::unordered_map<MotorId, std::chrono::steady_clock::time_point> close_startup_start_times;   // 关爪启动开始时间

    auto state = get_joint_state();
    for (auto& [id, data] : motor_ctrl_datas_) {
        if (data.config.ignore || !data.isEnabled()) continue;
        float pos = state.count(id) ? static_cast<float>(state[id].pos) : data.motor.position();
        last_positions[id] = pos;
        prev_torque[id] = 0.0f;
        joint_limit_found[id] = false;
        close_start_positions[id] = pos;
        close_startup_start_times[id] = std::chrono::steady_clock::now();
        close_stuck_detection_mode[id] = !is_open_direction;
        close_normal_limit_detection_enabled[id] = is_open_direction;
        close_attempt_started[id] = false;
        close_attempt_count[id] = 0;
        close_high_current_mode[id] = false;
        close_impact_active[id] = false;
    }

    auto start_time = std::chrono::steady_clock::now();
    bool all_joints_found_limit = false;

    while (!all_joints_found_limit && thread_running)
    {
        // 超时检查
        auto now = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - start_time).count();
        if (elapsed > timeout_ms)
        {
            std::cerr << "[LEJU claw]:限位寻找超时 (" << (is_open_direction ? "开爪" : "关爪") << ")，超时时间: " << timeout_ms << " ms，实际运行时间: " << elapsed << " ms" << std::endl;
            std::ostringstream oss;
            oss << "{\"source\":\"can_sdk\",\"phase\":\""
                << (is_open_direction ? "init_open" : "init_close")
                << "\",\"event\":\"find_limit_timeout\","
                << "\"timeout_ms\":" << timeout_ms << ","
                << "\"elapsed_ms\":" << elapsed << "}";
            emit_debug(oss.str());
            return false;
        }

        // 读取当前状态
        state = get_joint_state();
        std::unordered_map<MotorId, float> current_torques;
        
        // 计算当前速度和电流控制输出
        for (auto& [id, data] : motor_ctrl_datas_) 
        {
            if (data.config.ignore || !data.isEnabled() || joint_limit_found[id]) continue;

            float current_pos = state[id].pos;
            float current_velocity = (current_pos - last_positions[id]) / dt;
            float current_torque_feedback = state[id].torque;  // 当前电流反馈
                
            // 初始化关爪卡死保护：只有开爪阶段确认动过，关爪阶段才自动向开爪方向短时冲击。
            if (close_stuck_detection_mode[id] && !close_high_current_mode[id])
            {
                float position_change = std::abs(current_pos - close_start_positions[id]);
                float current_torque_feedback = state[id].torque;  // 当前电流反馈
                
                // 检查启动延时是否已过
                auto startup_elapsed = std::chrono::steady_clock::now() - close_startup_start_times[id];
                long long startup_ms = std::chrono::duration_cast<std::chrono::milliseconds>(startup_elapsed).count();
                
                // 只有在启动延时过后才开始卡死检测
                if (startup_ms >= cfg_CLOSE_STARTUP_DELAY_MS)
                {
                    bool stuck_current_reached = current_torque_feedback >= cfg_CLOSE_STUCK_CURRENT_THRESHOLD;

                    // 检查是否满足卡死条件：电流达到阈值且位置变化小于阈值
                    if (stuck_current_reached && position_change < stuck_position_threshold)
                    {
                        bool can_impact = allow_reverse_impact.count(id) ? allow_reverse_impact.at(id) : false;
                        if (can_impact) {
                            // 满足卡死条件且前面开爪确实动过，开始向开爪方向冲击松脱。
                            close_attempt_started[id] = true;
                            close_attempt_count[id] = 0;
                            close_high_current_mode[id] = true;
                            close_attempt_start_times[id] = std::chrono::steady_clock::now();
                            close_impact_active[id] = true;  // 开始冲击阶段
                            float impact_current = -cfg_CLOSE_IMPACT_CURRENT;
                            std::cout << "[LEJU claw] 夹爪 " << id << " 寻找关爪限位时检测到卡死（电流:" << current_torque_feedback
                                      << "A, 位置变化:" << position_change << "），开始向开爪方向" << impact_current << "A冲击模式" << std::endl;
                            std::ostringstream oss;
                            oss << "{\"source\":\"can_sdk\",\"phase\":\"init_close\",\"event\":\"stuck_detected\","
                                << "\"motor_id\":" << static_cast<int>(id) << ","
                                << "\"feedback_torque\":" << current_torque_feedback << ","
                                << "\"position_change\":" << position_change << ","
                                << "\"impact_current\":" << impact_current << "}";
                            emit_debug(oss.str());
                        } else {
                            close_stuck_detection_mode[id] = false;
                            close_normal_limit_detection_enabled[id] = true;
                            std::cout << "[LEJU claw] 夹爪 " << id << " 关爪方向疑似卡住，但开爪阶段未确认运动，禁用自动冲击，等待双向卡死判定" << std::endl;
                            std::ostringstream oss;
                            oss << "{\"source\":\"can_sdk\",\"phase\":\"init_close\",\"event\":\"stuck_detected_no_auto_impact\","
                                << "\"motor_id\":" << static_cast<int>(id) << ","
                                << "\"feedback_torque\":" << current_torque_feedback << ","
                                << "\"position_change\":" << position_change << "}";
                            emit_debug(oss.str());
                        }
                    }
                    // 如果位置变化大于阈值，说明正常运动，退出卡死检测模式，启用正常限位检测
                    else if (position_change >= stuck_position_threshold)
                    {
                        close_stuck_detection_mode[id] = false;
                        close_high_current_mode[id] = false;  // 确保退出冲击模式
                        close_attempt_started[id] = false;
                        close_impact_active[id] = false;
                        close_normal_limit_detection_enabled[id] = true;
                        // 重置滤波器状态，确保平滑过渡到正常速度控制
                        prev_torque[id] = 0.0f;
                        std::cout << "[LEJU claw] 夹爪 " << id << " 位置变化正常，位置从 " << close_start_positions[id] << " 变化到 " << current_pos << "，总变化量:" << position_change << " rad，退出卡死检测模式，启用正常限位检测" << std::endl;
                        std::ostringstream oss;
                        oss << "{\"source\":\"can_sdk\",\"phase\":\"" << (is_open_direction ? "init_open" : "init_close")
                            << "\",\"event\":\"normal_motion\","
                            << "\"motor_id\":" << static_cast<int>(id) << ","
                            << "\"position\":" << current_pos << ","
                            << "\"position_change\":" << position_change << "}";
                        emit_debug(oss.str());
                    }
                }
                else
                {
                    // // 启动延时内，输出调试信息
                    // if (startup_ms % 100 == 0) // 定期输出调试信息
                    // {
                    //     std::cout << "[LEJU claw] 夹爪 " << motor_ctrl_datas_[id] << " 启动卡死检测延时中，剩余 " << (cfg_CLOSE_STARTUP_DELAY_MS - startup_ms) << " ms，位置变化:" << position_change << " rad" << std::endl;
                    // }
                }
            }
            
            // 高电流尝试模式
            if (close_high_current_mode[id])
            {
                auto attempt_elapsed = std::chrono::steady_clock::now() - close_attempt_start_times[id];
                long long attempt_ms = std::chrono::duration_cast<std::chrono::milliseconds>(attempt_elapsed).count();
                float impact_current = -cfg_CLOSE_IMPACT_CURRENT;
                float impact_current_abs = std::abs(impact_current);
                
                // 冲击和间隔的循环逻辑
                if (close_impact_active[id])
                {
                    // 冲击阶段
                    if (attempt_ms < cfg_CLOSE_IMPACT_DURATION_MS)
                    {
                        current_torques[id] = impact_current;  // 反向冲击电流
                        // 只在冲击开始时输出一次日志
                        if (attempt_ms == 0)
                        {
                            three_amp_current_usage_count++;
                            std::cout << "[LEJU claw] 夹爪 " << id << " 进入反向" << impact_current << "A冲击模式 - 第" << three_amp_current_usage_count << "次" << std::endl;
                            std::ostringstream oss;
                            oss << "{\"source\":\"can_sdk\",\"phase\":\"" << (is_open_direction ? "init_open" : "init_close")
                                << "\",\"event\":\"impact_start\","
                                << "\"motor_id\":" << static_cast<int>(id) << ","
                                << "\"impact_current\":" << impact_current << ","
                                << "\"impact_count\":" << three_amp_current_usage_count << "}";
                            emit_debug(oss.str());
                        }
                    }
                    else
                    {
                        // 冲击结束，进入间隔阶段
                        close_impact_active[id] = false;
                        current_torques[id] = 0.0f;
                        close_attempt_start_times[id] = std::chrono::steady_clock::now();  // 重置计时器
                    }
                }
                else
                {
                    // 间隔阶段
                    if (attempt_ms < cfg_CLOSE_IMPACT_INTERVAL_MS)
                    {
                        current_torques[id] = 0.0f;  // 零电流间隔
                    }
                    else
                    {
                        // 间隔结束，检查位置变化
                        float position_change = std::abs(current_pos - close_start_positions[id]);
                        std::cout << "[LEJU claw] 夹爪 " << id << " 第" << close_attempt_count[id] << "次冲击间隔结束，位置变化:" << position_change << " rad" << std::endl;
                        {
                            std::ostringstream oss;
                            oss << "{\"source\":\"can_sdk\",\"phase\":\"" << (is_open_direction ? "init_open" : "init_close")
                                << "\",\"event\":\"impact_interval_done\","
                                << "\"motor_id\":" << static_cast<int>(id) << ","
                                << "\"attempt_count\":" << close_attempt_count[id] << ","
                                << "\"position_change\":" << position_change << "}";
                            emit_debug(oss.str());
                        }
                        if (position_change > stuck_position_threshold)
                        {
                            // 位置变化超过阈值，退出高电流模式，启用正常限位检测
                            close_high_current_mode[id] = false;
                            close_attempt_started[id] = false;
                            close_stuck_detection_mode[id] = false;
                            close_normal_limit_detection_enabled[id] = true;
                            // 重置滤波器状态，确保平滑过渡到正常速度控制
                            prev_torque[id] = 0.0f;
                            std::cout << "[LEJU claw] 夹爪 " << id << " 冲击成功，位置从 " << close_start_positions[id] << " 变化到 " << current_pos << "，总变化量:" << position_change << " rad，退出冲击模式，启用正常限位检测" << std::endl;
                            std::ostringstream oss;
                            oss << "{\"source\":\"can_sdk\",\"phase\":\"" << (is_open_direction ? "init_open" : "init_close")
                                << "\",\"event\":\"impact_success\","
                                << "\"motor_id\":" << static_cast<int>(id) << ","
                                << "\"position\":" << current_pos << ","
                                << "\"position_change\":" << position_change << "}";
                            emit_debug(oss.str());
                        }
                        else
                        {
                            // 位置变化不大，开始下一次冲击
                            close_attempt_count[id]++;
                            if (close_attempt_count[id] >= cfg_CLOSE_MAX_ATTEMPTS)
                            {
                                // 最大尝试次数完成，认为到达限位
                                joint_limit_found[id] = true;
                                std::ostringstream oss;
                                oss << "{\"source\":\"can_sdk\",\"phase\":\"" << (is_open_direction ? "init_open" : "init_close")
                                    << "\",\"event\":\"impact_max_attempts\","
                                    << "\"motor_id\":" << static_cast<int>(id) << ","
                                    << "\"attempt_count\":" << close_attempt_count[id] << ","
                                    << "\"impact_current_abs\":" << impact_current_abs << "}";
                                emit_debug(oss.str());
                            }
                            else
                            {
                                // 开始下一次冲击
                                close_impact_active[id] = true;
                                close_attempt_start_times[id] = std::chrono::steady_clock::now();
                            }
                        }
                    }
                }
            }
            else
            {
                // 正常速度控制模式
                float velocity_error = target_velocity - current_velocity;
                float torque_out = kd * velocity_error;
                
                // 双向限流，避免异常速度反馈导致反向大电流。
                torque_out = std::max(-max_current, std::min(max_current, torque_out));
                
                // 滤波处理
                current_torques[id] = lowPassFilter(torque_out, prev_torque[id], alpha);
                current_torques[id] = std::max(-max_current, std::min(max_current, current_torques[id]));
                prev_torque[id] = current_torques[id];
                
                // 开爪时的限位检测
                if (is_open_direction && current_torque_feedback < -stall_current_threshold && std::abs(current_velocity) < stall_velocity_threshold)
                {
                    joint_limit_found[id] = true;
                    std::ostringstream oss;
                    oss << "{\"source\":\"can_sdk\",\"phase\":\"init_open\",\"event\":\"normal_limit_detected\","
                        << "\"motor_id\":" << static_cast<int>(id) << ","
                        << "\"feedback_torque\":" << current_torque_feedback << ","
                        << "\"velocity\":" << current_velocity << "}";
                    emit_debug(oss.str());
                }
                
                // 关爪时的限位检测 - 只有在启用正常限位检测且不在冲击模式时才执行
                if (!is_open_direction && close_normal_limit_detection_enabled[id] && !close_high_current_mode[id] && current_torque_feedback > stall_current_threshold && std::abs(current_velocity) < stall_velocity_threshold)
                {
                    joint_limit_found[id] = true;
                    std::cout << "[LEJU claw] 夹爪 " << id << " 正常限位检测触发" << std::endl;
                    std::ostringstream oss;
                    oss << "{\"source\":\"can_sdk\",\"phase\":\"init_close\",\"event\":\"normal_limit_detected\","
                        << "\"motor_id\":" << static_cast<int>(id) << ","
                        << "\"feedback_torque\":" << current_torque_feedback << ","
                        << "\"velocity\":" << current_velocity << "}";
                    emit_debug(oss.str());
                }
            }
            
            last_positions[id] = current_pos;
        
        }
        
        // 发送电流指令
        for (auto& [id, data] : motor_ctrl_datas_)
        {
            if (data.config.ignore || !data.isEnabled()) continue;
            // 如果关节已经找到限位，发送0电流停止运动
            if (joint_limit_found[id])
                data.motor.controlPTM(0, 0, 0, 0, 0);
            else {
                float raw_torque = data.config.negtive ? -current_torques[id] : current_torques[id];
                data.motor.controlPTM(0, 0, raw_torque, 0, 0);
            }
        }
        
        // 检查是否所有关节都找到限位
        all_joints_found_limit = true;
        for (auto& [id, data] : motor_ctrl_datas_) {
            if (data.config.ignore || !data.isEnabled()) continue;
            if (!joint_limit_found[id])
            {
                all_joints_found_limit = false;
                break;
            }
        }
        
        // 如果所有关节都找到限位，立即退出循环
        if (all_joints_found_limit)
        {
            std::cout << "[LEJU claw]:所有关节都到达" << (is_open_direction ? "开爪" : "关爪") << "限位，退出寻找过程" << std::endl;
            std::ostringstream oss;
            oss << "{\"source\":\"can_sdk\",\"phase\":\""
                << (is_open_direction ? "init_open" : "init_close")
                << "\",\"event\":\"all_limit_found\"}";
            emit_debug(oss.str());
            break;
        }
        
        std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(dt * 1000)));
    }
    
    // 超时退出
    if (!all_joints_found_limit)
    {
        std::cout << "[LEJU claw]:" << (is_open_direction ? "开爪" : "关爪") << "限位寻找超时，认为当前位置为限位" << std::endl;
        std::ostringstream oss;
        oss << "{\"source\":\"can_sdk\",\"phase\":\""
            << (is_open_direction ? "init_open" : "init_close")
            << "\",\"event\":\"finish_without_all_limit\"}";
        emit_debug(oss.str());
    }
    
    return all_joints_found_limit;
}



void LeJuClawCan::adjust_range(std::unordered_map<MotorId, float> open_limit_positions, std::unordered_map<MotorId, float> close_limit_positions)
{
    // 设置关节位置映射范围
    joint_start_positions_.clear();
    joint_end_positions_.clear();
    for (auto& [id, data] : motor_ctrl_datas_) {
        if (data.config.ignore || !data.isEnabled()) {
            continue;
        }
        float original_open_position = open_limit_positions[id];
        float original_close_position = close_limit_positions[id];
        float total_travel_range = std::abs(original_close_position - original_open_position);
        
        // 计算调整量（基于总行程范围的百分比）
        float open_adjustment = total_travel_range * (cfg_OPEN_LIMIT_ADJUSTMENT / 100.0f);
        float close_adjustment = total_travel_range * (cfg_CLOSE_LIMIT_ADJUSTMENT / 100.0f);
        
        float actual_start_position, actual_end_position;
        
        if (original_close_position > original_open_position) {
            // 关爪位置大于开爪位置（正向运动）
            actual_start_position = original_open_position - open_adjustment;
            actual_end_position = original_close_position + close_adjustment;
        } else {
            // 关爪位置小于开爪位置（负向运动）
            actual_start_position = original_open_position + open_adjustment;
            actual_end_position = original_close_position - close_adjustment;
        }
        
        // 存储实际使用的限位位置
        joint_start_positions_[id] = actual_start_position; 
        joint_end_positions_[id] = actual_end_position;   
        
        float actual_travel_range = std::abs(actual_end_position - actual_start_position);
        std::cout << "[LEJU claw]:电机 " << id << " 行程范围: " << actual_travel_range << " rad" << std::endl;
    }
    
    std::cout << "[LEJU claw]:夹爪限位寻找完成，位置映射设置完成" << std::endl;
}

int LeJuClawCan::initialize()
{
    emit_debug("{\"source\":\"can_sdk\",\"phase\":\"initialize\",\"event\":\"start\"}");
    if (!init_lejuclaw_can_customed()) {
        emit_debug("{\"source\":\"can_sdk\",\"phase\":\"initialize\",\"event\":\"can_init_failed\"}");
        return -1;
    }

    // 加载可选的夹爪运行参数文件（不存在则忽略，保持默认值）
    std::string home_path = get_home_path();
    if (!home_path.empty()) {
        std::string leju_param_file = "lejuclaw_config.yaml";
        std::string leju_param_path = home_path + "/" + leju_param_file;
        
        std::filesystem::path leju_yaml_path(leju_param_path);
        if (std::filesystem::exists(leju_yaml_path)) {
            LEJUCLAW_LOG_INFO("找到配置文件: %s", leju_param_path.c_str());
            emit_debug("{\"source\":\"can_sdk\",\"phase\":\"initialize\",\"event\":\"config_found\",\"config_source\":\"yaml\"}");
            load_leju_params(leju_param_path);
        } else {
            LEJUCLAW_LOG_INFO("配置文件 %s 未找到，使用头文件默认参数", leju_param_path.c_str());
            emit_debug("{\"source\":\"can_sdk\",\"phase\":\"initialize\",\"event\":\"config_missing\",\"config_source\":\"default\"}");
        }
    } else {
        LEJUCLAW_LOG_WARN("无法获取用户主目录路径，使用头文件默认参数");
        emit_debug("{\"source\":\"can_sdk\",\"phase\":\"initialize\",\"event\":\"home_path_missing\",\"config_source\":\"default\"}");
    }

    init_status_variables();

    // 清除故障码，否则使能后会退回 Rest State
    if(!disableAll()) {
        RWLOG_FAILURE("失能所有电机失败--->无法正常初始运行");
        emit_debug("{\"source\":\"can_sdk\",\"phase\":\"initialize\",\"event\":\"disable_all_failed\"}");
        return -1;
    }

    // 使能所有电机
    if(!enableAll()) {
        RWLOG_FAILURE("使能所有电机失败--->无法正常初始运行");
        emit_debug("{\"source\":\"can_sdk\",\"phase\":\"initialize\",\"event\":\"enable_all_failed\"}");
        return -1;
    }

    // 等待所有电机第一个反馈到达
    if (!waitForAllMotorsFeedback(1500)) {
        RWLOG_W("Some motors did not receive initial feedback, continuing anyway");
        emit_debug("{\"source\":\"can_sdk\",\"phase\":\"initialize\",\"event\":\"initial_feedback_timeout\"}");
        for (const auto& [id, data] : motor_ctrl_datas_) {
            if (!data.config.ignore && !data.feedback_received) {
                RWLOG_WARNING("电机 ID %u (%s) 没有收到初始反馈", id, data.config.name.c_str());
            }
        }
    }

    // 开启控制数据接收线程
    thread_running = true;
    control_thread_ = std::thread(&LeJuClawCan::control_thread, this);
    // 等待控制线程启动
    std::this_thread::sleep_for(std::chrono::milliseconds(200));

    // 量程测量
    set_init_fault(InitFault::NONE, "");
    if (!measure_range()) {
        clear_all_torque();
        emit_debug("{\"source\":\"can_sdk\",\"phase\":\"initialize\",\"event\":\"measure_range_failed\"}");
        return -1;
    }
    emit_debug("{\"source\":\"can_sdk\",\"phase\":\"initialize\",\"event\":\"measure_range_done\"}");

    // 创建目标位置向量
    std::vector<double> target_positions(motor_ctrl_datas_.size(), 50.0);
    std::vector<double> target_velocity(motor_ctrl_datas_.size(), 0.0);
    std::vector<double> target_torque(motor_ctrl_datas_.size(), 0.0);
    auto result = move_paw(target_positions, target_velocity, target_torque);
    
    if (result == PawMoveState::LEFT_REACHED_RIGHT_REACHED) {
        std::cout << "[LEJU claw]:夹爪已成功运动到中间位置" << std::endl;
        emit_debug("{\"source\":\"can_sdk\",\"phase\":\"initialize\",\"event\":\"move_to_mid_success\"}");
    } else {
        std::cout << "[LEJU claw]:夹爪运动到中间位置失败" << std::endl;
        emit_debug("{\"source\":\"can_sdk\",\"phase\":\"initialize\",\"event\":\"move_to_mid_failed\"}");
    }

    emit_debug("{\"source\":\"can_sdk\",\"phase\":\"initialize\",\"event\":\"done\"}");

    // 初始化完成：解除断流看门狗武装（等高频命令到达时再武装）
    disarm_stream_watchdog();
    return 0;
}

int LeJuClawCan::initialize_recovery_mode()
{
    disarm_stream_watchdog();
    emit_debug("{\"source\":\"can_sdk\",\"phase\":\"recovery_initialize\",\"event\":\"start\"}");
    if (!init_lejuclaw_can_customed()) {
        emit_debug("{\"source\":\"can_sdk\",\"phase\":\"recovery_initialize\",\"event\":\"can_init_failed\"}");
        return -1;
    }

    std::string home_path = get_home_path();
    if (!home_path.empty()) {
        std::string leju_param_path = home_path + "/lejuclaw_config.yaml";
        std::filesystem::path leju_yaml_path(leju_param_path);
        if (std::filesystem::exists(leju_yaml_path)) {
            LEJUCLAW_LOG_INFO("找到配置文件: %s", leju_param_path.c_str());
            load_leju_params(leju_param_path);
        } else {
            LEJUCLAW_LOG_INFO("配置文件 %s 未找到，使用头文件默认参数", leju_param_path.c_str());
        }
    }

    init_status_variables();
    set_init_fault(InitFault::NONE, "");

    if (!disableAll()) {
        RWLOG_FAILURE("恢复模式失能所有电机失败");
        emit_debug("{\"source\":\"can_sdk\",\"phase\":\"recovery_initialize\",\"event\":\"disable_all_failed\"}");
        return -1;
    }

    if (!enableAll()) {
        RWLOG_FAILURE("恢复模式使能所有电机失败");
        emit_debug("{\"source\":\"can_sdk\",\"phase\":\"recovery_initialize\",\"event\":\"enable_all_failed\"}");
        return -1;
    }

    if (!waitForAllMotorsFeedback(1500)) {
        RWLOG_W("Recovery mode: some motors did not receive initial feedback, continuing anyway");
        emit_debug("{\"source\":\"can_sdk\",\"phase\":\"recovery_initialize\",\"event\":\"initial_feedback_timeout\"}");
    }

    thread_running = true;
    control_thread_ = std::thread(&LeJuClawCan::control_thread, this);
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    stop_all_current(5, 10);

    emit_debug("{\"source\":\"can_sdk\",\"phase\":\"recovery_initialize\",\"event\":\"done\"}");
    return 0;
}

void LeJuClawCan::close()
{
    disarm_stream_watchdog();
    stop_all_current(5, 10);
    thread_running = false;
    if (control_thread_.joinable()) {
        control_thread_.join();
    }

    stop_all_current(3, 10);
    disableAll();

    // 使用 device_canbus_map_ 反注册
    auto& canbus_controller = canbus_sdk::CanBusController::getInstance();
    for (const auto& kv : device_canbus_map_) {
        auto bus_id_result = canbus_controller.getBusIdByName(kv.second);
        if (bus_id_result.has_value()) {
            canbus_controller.unregisterDevice(canbus_sdk::DeviceType::LEJUCLAW, kv.first, kv.second);
            RWLOG_SUCCESS("Unregistered device id %d (%s)", kv.first, kv.second.c_str());
        }
    }
}

float LeJuClawCan::mapLogicalRadToPercent(MotorId id, float logical_rad) {
    auto it_s = joint_start_positions_.find(id);
    auto it_e = joint_end_positions_.find(id);
    if (it_s == joint_start_positions_.end() || it_e == joint_end_positions_.end()) {
        return 0.0f;
    }
    float s = it_s->second;
    float e = it_e->second;
    if (std::abs(e - s) < 1e-6f) {
        return 0.0f;
    }
    float percent = (logical_rad - s) * 100.0f / (e - s);
    if (percent < 0.0f) percent = 0.0f;
    if (percent > 100.0f) percent = 100.0f;
    return percent;
}

float LeJuClawCan::mapPercentToLogicalRad(MotorId id, float percent) {
    percent = std::max(0.0f, std::min(100.0f, percent));
    auto it_s = joint_start_positions_.find(id);
    auto it_e = joint_end_positions_.find(id);
    if (it_s == joint_start_positions_.end() || it_e == joint_end_positions_.end()) {
        return 0.0f;
    }
    float s = it_s->second;
    float e = it_e->second;
    float logical = s + (percent / 100.0f) * (e - s);
    return logical;
}

bool LeJuClawCan::waitForOperationStatus(MotorId id, Operation expected_op, OperationStatus expected_status, int timeout_ms, const char* operation_name) {
    auto start_time = std::chrono::steady_clock::now();
    while (true) {
        // 检查是否超时
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - start_time);
        if (elapsed.count() >= timeout_ms) {
            RWLOG_W("%s motor 0x%02X timeout after %d ms", operation_name, id, timeout_ms);
            auto data_it = motor_ctrl_datas_.find(id);
            if (data_it != motor_ctrl_datas_.end()) {
                data_it->second.operation_status = OperationStatus::TIMEOUT;
            }
            return false;
        }

        // 检查状态是否已更新为成功
        auto data_it = motor_ctrl_datas_.find(id);
        if (data_it != motor_ctrl_datas_.end() &&
            data_it->second.operation == expected_op &&
            data_it->second.operation_status == expected_status) {
            // 多圈清理没有反馈帧
            if(expected_op == Operation::MULTI_TURN_ZERO) {
                RWLOG_I("Motor 0x%02X %s completed successfully", id, operation_name);
                return true;
            }

            if(data_it->second.feedback_received == true) {
                RWLOG_I("Motor 0x%02X %s completed successfully", id, operation_name);
                return true;
            }
        }

        // 10ms检查一次
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}

bool LeJuClawCan::enableMotor(MotorId id, int timeout_ms) {
    // 查找电机控制数据
    auto data_it = motor_ctrl_datas_.find(id);
    if (data_it == motor_ctrl_datas_.end()) {
        RWLOG_W("Cannot enable unknown motor ID: 0x%02X", id);
        return false;
    }

    // 设置状态为发送中
    data_it->second.operation = Operation::ENABLE;
    data_it->second.operation_status = OperationStatus::PENDING;
    data_it->second.feedback_received = false;
    // 发送enable指令
    if (!data_it->second.motor.enterMotorState()) {
        RWLOG_E("Failed to send enable command to motor 0x%02X", id);
        data_it->second.operation_status = OperationStatus::FAILED;
        return false;
    }

    // RWLOG_I("Enable command sent to motor 0x%02X, waiting for confirmation...", id);

    return waitForOperationStatus(id, Operation::ENABLE, OperationStatus::SUCCESS, timeout_ms, "Enable");
}

bool LeJuClawCan::enableAll()
{
    for (auto& [id, data] : motor_ctrl_datas_) {
        // 电机在配置文件中被标记为 <ignore> 则跳过
        if(data.config.ignore) {
            continue;
        }
        enableMotor(id, 500); // 500ms
    }

    // 检查状态: enable 使能指令是否发送成功
    int ignore_count = 0, success_count = 0;
    for (auto& [id, data] : motor_ctrl_datas_) {
        // 电机在配置文件中被标记为 <ignore> 则跳过
        if(data.config.ignore) {
            RWLOG_WARNING("MOTOR ID:%u Enable: [Ignored]", id);
            ignore_count++;
            continue;
        }
        if (!data.isEnabled()) {
            RWLOG_FAILURE("MOTOR ID:%u Enable: [Failed]", id);
        } else {
            success_count++;
            RWLOG_SUCCESS("MOTOR ID:%u Enable: [Success]", id);
        }
    }

    if(ignore_count == motor_ctrl_datas_.size()) {
        // 全部忽略那就是成功
        return true;
    }
    if(success_count == 0) {
        // 一个enable成功的都没有
        return false;
    }
    return true;
}

bool LeJuClawCan::disableMotor(MotorId id, int timeout_ms) {
    // 查找电机控制数据
    auto data_it = motor_ctrl_datas_.find(id);
    if (data_it == motor_ctrl_datas_.end()) {
        RWLOG_W("Cannot disable unknown motor ID: 0x%02X", id);
        return false;
    }

    // 设置状态为发送中
    data_it->second.operation = Operation::DISABLE;
    data_it->second.operation_status = OperationStatus::PENDING;

    // 发送disable指令
    if (!data_it->second.motor.enterRestState()) {
        RWLOG_E("Failed to send disable command to motor 0x%02X", id);
        data_it->second.operation_status = OperationStatus::FAILED;
        return false;
    }

    // RWLOG_I("Disable command sent to motor 0x%02X, waiting for confirmation...", id);

    return waitForOperationStatus(id, Operation::DISABLE, OperationStatus::SUCCESS, timeout_ms, "Disable");
}

bool LeJuClawCan::disableAll()
{
    std::unordered_map<MotorId, int> results;
    for (auto& [id, data] : motor_ctrl_datas_) {
        if(data.config.ignore) {
            results.insert(std::make_pair(id, -1));
            continue;
        }
        bool ret = disableMotor(id);
        results.insert(std::make_pair(id, ret?0:1));
    }
    int ignore_count = 0, success_count = 0;
    for (auto& [id, result] : results) {
        if (result == 0) {
            success_count++;
            RWLOG_SUCCESS("MOTOR ID:%u Disable: [Success]", id);
        }
        else if(result == -1) {
            ignore_count++;
            RWLOG_SUCCESS("MOTOR ID:%u Disable: [Ignored]", id);
        }
        else {
            RWLOG_FAILURE("MOTOR ID:%u Disable: [Failed]", id);
        }
    }

    if(ignore_count == motor_ctrl_datas_.size()) {
        // 全部忽略那就是成功
        return true;
    }
    if(success_count == 0) {
        // 一个成功的都没有
        return false;
    }
    return true;
}

float LeJuClawCan::lowPassFilter(float input, float prevOutput, float alpha) {
    float safe_alpha = std::max(0.0f, std::min(1.0f, alpha));
    return safe_alpha * input + (1.0f - safe_alpha) * prevOutput;
}

void LeJuClawCan::clear_all_torque() {
    for (auto& [id, data] : motor_ctrl_datas_) {
        if (data.config.ignore || !data.isEnabled()) {
            continue;
        }
        // 直接清零力矩输出
        data.motor.controlPTM(0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
    }
}

uint64_t LeJuClawCan::refresh_stream_watchdog(std::chrono::steady_clock::time_point received) {
    std::lock_guard<std::mutex> lock(stream_watchdog_mutex_);
    if (std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - received).count() >= cfg_STREAM_WATCHDOG_TIMEOUT_MS) return 0;
    stream_watchdog_armed_ = true;
    stream_watchdog_expired_ = false;
    last_stream_command_time_ = received;
    return ++stream_generation_;
}

void LeJuClawCan::disarm_stream_watchdog() {
    std::lock_guard<std::mutex> lock(stream_watchdog_mutex_);
    stream_watchdog_armed_ = false;
    stream_watchdog_expired_ = false;
}

bool LeJuClawCan::stream_watchdog_expired() {
    std::lock_guard<std::mutex> lock(stream_watchdog_mutex_);
    return stream_watchdog_armed_ && stream_watchdog_expired_;
}

void LeJuClawCan::stop_all_current(int repeat_count, int interval_ms) {
    repeat_count = std::max(1, repeat_count);
    interval_ms = std::max(0, interval_ms);

    {
        std::lock_guard<std::mutex> lock(target_lock);
        for (auto& [id, data] : motor_ctrl_datas_) {
            if (data.config.ignore || !data.isEnabled()) {
                continue;
            }
            motorevo::RevoMotorCmd_t zero_cmd{};
            zero_cmd.pos = data.config.negtive ? -data.motor.position() : data.motor.position();
            zero_cmd.vel = 0.0;
            zero_cmd.torque = 0.0;
            zero_cmd.kp = 0.0;
            zero_cmd.kd = 0.0;
            target_cmd[id] = zero_cmd;
            old_target_cmd[id] = zero_cmd;
        }
        target_updated_ = true;
    }

    for (int i = 0; i < repeat_count; ++i) {
        clear_all_torque();
        if (interval_ms > 0) {
            std::this_thread::sleep_for(std::chrono::milliseconds(interval_ms));
        }
    }
}

void LeJuClawCan::cancel_motion()
{
    motion_cancel_requested_ = true;
    stop_all_current(5, 10);
    emit_debug("{\"source\":\"can_sdk\",\"phase\":\"move\",\"event\":\"cancel_requested\"}");
}

std::vector<double> LeJuClawCan::get_positions()
{
    std::vector<double> out;
    out.reserve(motor_ctrl_datas_.size());
    if (!thread_running)
    {
        return out;
    }
    std::lock_guard<std::mutex> lock(state_lock);
    // 收集 (id, percent) 并按 id 升序返回
    std::vector<std::pair<MotorId, double>> id_percent;
    id_percent.reserve(motor_ctrl_datas_.size());
    for (const auto &kv : motor_ctrl_datas_) {
        MotorId id = kv.first;
        float pos_rad = current_cmd.count(id) ? static_cast<float>(current_cmd[id].pos) : kv.second.motor.position();
        float percent = mapLogicalRadToPercent(id, pos_rad);
        id_percent.emplace_back(id, static_cast<double>(percent));
    }
    std::sort(id_percent.begin(), id_percent.end(), [](const auto &a, const auto &b){
        return static_cast<unsigned int>(a.first) < static_cast<unsigned int>(b.first);
    });
    for (const auto &p : id_percent) {
        out.push_back(p.second);
    }
    return out;
}

std::vector<double> LeJuClawCan::get_torque()
{
    std::vector<double> out;
    out.reserve(motor_ctrl_datas_.size());
    if (!thread_running)
    {
        return out;
    }
    std::lock_guard<std::mutex> lock(state_lock);
    // 收集 (id, torque) 并按 id 升序返回
    std::vector<std::pair<MotorId, double>> id_val;
    id_val.reserve(motor_ctrl_datas_.size());
    for (const auto &kv : motor_ctrl_datas_) {
        MotorId id = kv.first;
        float tor = current_cmd.count(id) ? static_cast<float>(current_cmd[id].torque) : kv.second.motor.torque();
        id_val.emplace_back(id, static_cast<double>(tor));
    }
    std::sort(id_val.begin(), id_val.end(), [](const auto &a, const auto &b){
        return static_cast<unsigned int>(a.first) < static_cast<unsigned int>(b.first);
    });
    for (const auto &p : id_val) {
        out.push_back(p.second);
    }
    return out;
}

std::vector<double> LeJuClawCan::get_velocity()
{
    std::vector<double> out;
    out.reserve(motor_ctrl_datas_.size());
    if (!thread_running)
    {
        return out;
    }
    std::lock_guard<std::mutex> lock(state_lock);
    // 收集 (id, velocity) 并按 id 升序返回
    std::vector<std::pair<MotorId, double>> id_val;
    id_val.reserve(motor_ctrl_datas_.size());
    for (const auto &kv : motor_ctrl_datas_) {
        MotorId id = kv.first;
        float vel = current_cmd.count(id) ? static_cast<float>(current_cmd[id].vel) : kv.second.motor.velocity();
        id_val.emplace_back(id, static_cast<double>(vel));
    }
    std::sort(id_val.begin(), id_val.end(), [](const auto &a, const auto &b){
        return static_cast<unsigned int>(a.first) < static_cast<unsigned int>(b.first);
    });
    for (const auto &p : id_val) {
        out.push_back(p.second);
    }
    return out;
}

LeJuClawCan::InitFault LeJuClawCan::get_init_fault() const
{
    return last_init_fault_;
}

std::string LeJuClawCan::get_init_fault_message() const
{
    return last_init_fault_message_;
}

void LeJuClawCan::set_init_fault(InitFault fault, const std::string& message)
{
    last_init_fault_ = fault;
    last_init_fault_message_ = message;
}

bool LeJuClawCan::apply_recovery_current(RecoveryDirection direction, float current, int duration_ms)
{
    disarm_stream_watchdog();
    constexpr float kManualRecoveryMaxCurrent = 3.0f;
    constexpr int kManualRecoveryMaxDurationMs = 500;

    if (!thread_running) {
        std::cerr << "[LEJU claw]:恢复冲击失败，控制线程未运行" << std::endl;
        return false;
    }
    if (duration_ms <= 0) {
        std::cerr << "[LEJU claw]:恢复冲击失败，持续时间无效" << std::endl;
        return false;
    }

    float current_abs = std::abs(current);
    if (current_abs < 0.01f) {
        std::cerr << "[LEJU claw]:恢复冲击失败，电流过小" << std::endl;
        return false;
    }
    if (current_abs > kManualRecoveryMaxCurrent) {
        std::cout << "[LEJU claw]:恢复冲击电流限制为 " << kManualRecoveryMaxCurrent << "A" << std::endl;
        current_abs = kManualRecoveryMaxCurrent;
    }
    if (duration_ms > kManualRecoveryMaxDurationMs) {
        std::cout << "[LEJU claw]:恢复冲击持续时间限制为 " << kManualRecoveryMaxDurationMs << "ms" << std::endl;
        duration_ms = kManualRecoveryMaxDurationMs;
    }

    float signed_current = (direction == RecoveryDirection::TOWARD_OPEN) ? -current_abs : current_abs;
    std::cout << "[LEJU claw]:人工恢复冲击，方向="
              << (direction == RecoveryDirection::TOWARD_OPEN ? "开爪" : "关爪")
              << ", current=" << signed_current << "A, duration=" << duration_ms << "ms" << std::endl;
    {
        std::ostringstream oss;
        oss << "{\"source\":\"can_sdk\",\"phase\":\"manual_recovery\",\"event\":\"start\","
            << "\"direction\":\"" << (direction == RecoveryDirection::TOWARD_OPEN ? "open" : "close") << "\","
            << "\"current\":" << signed_current << ","
            << "\"duration_ms\":" << duration_ms << "}";
        emit_debug(oss.str());
    }

    auto start = std::chrono::steady_clock::now();
    while (thread_running) {
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - start).count();
        if (elapsed >= duration_ms) {
            break;
        }

        for (auto& [id, data] : motor_ctrl_datas_) {
            if (data.config.ignore || !data.isEnabled()) {
                continue;
            }
            float raw_current = data.config.negtive ? -signed_current : signed_current;
            data.motor.controlPTM(0, 0, raw_current, 0, 0);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(std::max(1, static_cast<int>(cfg_ZERO_CONTROL_DT * 1000))));
    }

    clear_all_torque();
    emit_debug("{\"source\":\"can_sdk\",\"phase\":\"manual_recovery\",\"event\":\"done\"}");
    return true;
}

void LeJuClawCan::updateMotorCmd(MotorId id, double pos, double vel, double torque, double kp, double kd)
{
    if (motor_ctrl_datas_.find(id) == motor_ctrl_datas_.end()) {
        RWLOG_W("Motor %u not found in updateMotorCmd", id);
        return;
    }

    auto& data = motor_ctrl_datas_.at(id);

    // 更新 target_cmd
    std::lock_guard<std::mutex> lock(target_lock);
    target_cmd[id] = motorevo::RevoMotorCmd_t{
        .pos = pos,
        .vel = vel,
        .torque = torque,
        .kp = kp,
        .kd = kd
    };

    // RWLOG_D("Updated motor %u command: pos=%.3f, vel=%.3f, torque=%.3f, kp=%.1f, kd=%.1f",
    //          id, pos, vel, torque, kp, kd);

    target_updated_ = true;
}

LeJuClawCan::PawMoveState LeJuClawCan::move_paw(const std::vector<double>& positions, const std::vector<double>& velocity, const std::vector<double>& torque)
{
    return move_paw(positions, velocity, torque, false);
}

LeJuClawCan::PawMoveState LeJuClawCan::move_paw(const std::vector<double>& positions,
                                                const std::vector<double>& velocity,
                                                const std::vector<double>& torque,
                                                bool is_high_freq, TargetUpdate update)
{
    // // 输出当前使用的模式
    // if (is_high_freq) {
    //     std::cout << "[LEJU claw] 调用 move_paw: 使用 VR 模式 (超时时间: " << VR_CONTROL_TIMEOUT_MS << " ms)" << std::endl;
    // } else {
    //     std::cout << "[LEJU claw] 调用 move_paw: 使用非 VR 模式 (等待动作完成)" << std::endl;
    // }

    if (!thread_running)
    {
        return PawMoveState::ERROR;
    }

    // 输入参数安全检查
    if (positions.size() != motor_ctrl_datas_.size() ||
        velocity.size() != motor_ctrl_datas_.size() ||
        torque.size() != motor_ctrl_datas_.size())
    {
        std::cerr << "[LEJU claw]:Input vector sizes do not match motor_ctrl_datas_ size." << std::endl;
        return PawMoveState::ERROR;
    }

    std::vector<double> safe_positions = positions;
    for (size_t i = 0; i < safe_positions.size(); ++i)
    {
        // 软限位 5~95%，防止夹爪撞到硬限位
        safe_positions[i] = std::max(5.0, std::min(95.0, safe_positions[i]));
    }

    // 将 vector 参数按 MotorId 升序映射为 map，再转调 map 版本
    std::vector<MotorId> ids = get_sorted_motor_ids();
    std::unordered_map<MotorId, double> pos_map, vel_map, tor_map;
    if (ids.size() != safe_positions.size()) {
        return PawMoveState::ERROR;
    }
    for (size_t i = 0; i < ids.size(); ++i) {
        pos_map[ids[i]] = safe_positions[i];
        vel_map[ids[i]] = (i < velocity.size() ? velocity[i] : 0.0);
        tor_map[ids[i]] = (i < torque.size() ? torque[i] : 0.0);
    }
    return move_paw(pos_map, vel_map, tor_map, is_high_freq, update);
}

// map 版本的 move_paw（基于目标位置附近范围调整kp/kd的PD控制）
LeJuClawCan::PawMoveState LeJuClawCan::move_paw(
    const std::unordered_map<MotorId, double>& positions_percent,
    const std::unordered_map<MotorId, double>& velocity,
    const std::unordered_map<MotorId, double>& torque,
    bool is_high_freq, TargetUpdate update)
{
    if (!thread_running) {
        emit_debug("{\"source\":\"can_sdk\",\"phase\":\"move\",\"event\":\"thread_not_running\"}");
        return PawMoveState::ERROR;
    }

    emit_debug(std::string("{\"source\":\"can_sdk\",\"phase\":\"move\",\"event\":\"start\",\"is_high_freq\":") +
               (is_high_freq ? "true" : "false") + "}");
    motion_cancel_requested_ = false;

    // 目标位置映射到逻辑坐标(rad)，并准备状态变量
    std::unordered_map<MotorId, float> target_positions_rad;
    std::unordered_map<MotorId, float> prev_torque;
    std::unordered_map<MotorId, float> last_positions;
    std::unordered_map<MotorId, float> last_velocities;

    // 稳定检测（非VR模式）
    std::unordered_map<MotorId, std::chrono::steady_clock::time_point> stable_detection_start_times;
    std::unordered_map<MotorId, bool> stable_detection_enabled;
    std::unordered_map<MotorId, bool> is_stable;
    std::unordered_map<MotorId, float> stable_check_positions;

    // 卡死/冲击检测
    std::unordered_map<MotorId, std::chrono::steady_clock::time_point> stuck_start_times;
    std::unordered_map<MotorId, std::chrono::steady_clock::time_point> stuck_detection_start_times;
    std::unordered_map<MotorId, bool> stuck_detected;
    std::unordered_map<MotorId, bool> in_impact_mode;
    std::unordered_map<MotorId, std::chrono::steady_clock::time_point> impact_start_times;
    std::unordered_map<MotorId, bool> impact_active;
    std::unordered_map<MotorId, float> stuck_check_positions;
    std::unordered_map<MotorId, bool> stuck_detection_enabled;

    // VR 卡死检测
    std::unordered_map<MotorId, int> vr_stuck_cycle_count;
    std::unordered_map<MotorId, float> vr_stuck_check_positions;

    // 初始状态快照
    auto state = get_joint_state();
    for (const auto &kv : motor_ctrl_datas_) {
        MotorId id = kv.first;
        if (kv.second.config.ignore || !kv.second.isEnabled()) continue;
        double pct = 0.0;
        auto itp = positions_percent.find(id);
        // 软限位 5~95%，防止夹爪撞到硬限位
        if (itp != positions_percent.end()) pct = std::max(5.0, std::min(95.0, itp->second));
        target_positions_rad[id] = mapPercentToLogicalRad(id, static_cast<float>(pct));
        {
            std::ostringstream oss;
            oss << "{\"source\":\"can_sdk\",\"phase\":\"move\",\"event\":\"target\","
                << "\"motor_id\":" << static_cast<int>(id) << ","
                << "\"target_percent\":" << pct << ","
                << "\"target_rad\":" << target_positions_rad[id] << "}";
            emit_debug(oss.str());
        }
        last_positions[id] = state.count(id) ? static_cast<float>(state[id].pos) : kv.second.motor.position();
        last_velocities[id] = 0.0f;
        prev_torque[id] = 0.0f;
        
        // 初始化稳定检测
        stable_detection_enabled[id] = false;
        is_stable[id] = false;
        stable_check_positions[id] = last_positions[id];
        
        // 初始化卡死检测
        stuck_detected[id] = false;
        in_impact_mode[id] = false;
        impact_active[id] = false;
        stuck_detection_enabled[id] = false;
        vr_stuck_cycle_count[id] = 0;
        vr_stuck_check_positions[id] = last_positions[id];
        stuck_detection_start_times[id] = std::chrono::steady_clock::now();
        stable_detection_start_times[id] = std::chrono::steady_clock::now();
    }

    // 非VR保持模式入口跳过：已在保持 + 目标不变/更大 → 直接退出
    if (cfg_ENABLE_NON_VR_HOLD && !is_high_freq) {
        bool all_skip = true;
        for (const auto &kv : motor_ctrl_datas_) {
            MotorId id = kv.first;
            if (kv.second.config.ignore || !kv.second.isEnabled()) continue;
            auto itp = positions_percent.find(id);
            if (itp == positions_percent.end()) { all_skip = false; break; }
            double pct = itp->second;
            bool is_hold_target = false;
            for (float trigger : non_vr_hold_trigger_set_) {
                if (std::abs(static_cast<float>(pct) - trigger) < 0.01f) { is_hold_target = true; break; }
            }
            bool already_holding = (non_vr_hold_mode_.count(id) && non_vr_hold_mode_[id]);
            if (!already_holding || !is_hold_target) { all_skip = false; break; }
            double last_pct = last_move_paw_target_percent_.count(id) ? last_move_paw_target_percent_[id] : -1.0;
            if (last_pct >= 0.0 && pct < last_pct - 1.0) { all_skip = false; break; }
        }
        if (all_skip) {
            for (const auto &kv : motor_ctrl_datas_) {
                MotorId id = kv.first;
                if (kv.second.config.ignore || !kv.second.isEnabled()) continue;
                last_move_paw_target_percent_[id] = positions_percent.find(id)->second;
            }
            target_updated_ = true;
            return PawMoveState::LEFT_REACHED_RIGHT_REACHED;
        }
    }

    // 非VR保持模式：目标位置命中触发值时激活保持电流
    if (cfg_ENABLE_NON_VR_HOLD && !is_high_freq) {
        for (const auto &kv : motor_ctrl_datas_) {
            MotorId id = kv.first;
            if (kv.second.config.ignore || !kv.second.isEnabled()) continue;
            auto itp = positions_percent.find(id);
            if (itp != positions_percent.end()) {
                float pct = static_cast<float>(itp->second);
                bool is_hold_target = false;
                for (float trigger : non_vr_hold_trigger_set_) {
                    if (std::abs(pct - trigger) < 0.01f) { is_hold_target = true; break; }
                }
                if (is_hold_target) {
                    non_vr_hold_mode_[id] = true;
                    non_vr_hold_current_[id] = cfg_NON_VR_HOLD_CURRENT;
                } else {
                    non_vr_hold_mode_[id] = false;
                    non_vr_hold_current_[id] = 0.0f;
                }
            }
        }
    }

    // VR: 直接设置目标位置（持续由控制线程下发），并启用超时
    auto movement_start_time = std::chrono::steady_clock::now();
    bool movement_timeout_enabled = false;
    if (is_high_freq) {
        std::lock_guard<std::mutex> lock(target_lock);
        for (const auto &kv : motor_ctrl_datas_) {
            MotorId id = kv.first;
            if (kv.second.config.ignore || !kv.second.isEnabled()) continue;
            motorevo::RevoMotorCmd_t cmd = target_cmd[id];
            cmd.pos = target_positions_rad[id];
            cmd.kp = kv.second.config.default_params.kp_pos;
            cmd.kd = kv.second.config.default_params.kd_pos;
            target_cmd[id] = cmd;
        }
        target_updated_ = true;
        movement_timeout_enabled = true;
        movement_start_time = std::chrono::steady_clock::now();
    }

    bool all_reached = false;
    while (!all_reached && thread_running) {
        // 断流看门狗：高频控制流断流 → 立即中止（不再写入 target_cmd，防堵转；安全 PTM 由 control_thread 权威下发）
        if (stream_watchdog_expired()) {
            emit_debug("{\"source\":\"can_sdk\",\"phase\":\"move\",\"event\":\"stream_expired_abort\"}");
            return PawMoveState::STREAM_EXPIRED;
        }

        if (motion_cancel_requested_) {
            stop_all_current(5, 10);
            motion_cancel_requested_ = false;
            emit_debug("{\"source\":\"can_sdk\",\"phase\":\"move\",\"event\":\"cancelled\"}");
            return PawMoveState::ERROR;
        }

        // VR 模式超时
        if (movement_timeout_enabled && is_high_freq) {
            auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::steady_clock::now() - movement_start_time).count();
            if (elapsed > cfg_VR_CONTROL_TIMEOUT_MS) {
                break; // VR 模式：超时即可退出
            }
        }

        // 低频 Topic 动作中消费最新目标。只替换当前运动循环的目标，不重启 worker 或看门狗。
        std::vector<double> next_positions, next_velocity, next_torque;
        if (update && !is_high_freq && update(next_positions, next_velocity, next_torque)) {
            const auto ids = get_sorted_motor_ids();
            const bool valid_update =
                next_positions.size() == ids.size() &&
                next_velocity.size() == ids.size() &&
                next_torque.size() == ids.size() &&
                std::all_of(next_positions.begin(), next_positions.end(),
                            [](double value) { return std::isfinite(value); }) &&
                std::all_of(next_velocity.begin(), next_velocity.end(),
                            [](double value) { return std::isfinite(value); }) &&
                std::all_of(next_torque.begin(), next_torque.end(),
                            [](double value) { return std::isfinite(value); });
            if (!valid_update) {
                emit_debug("{\"source\":\"can_sdk\",\"phase\":\"move\",\"event\":\"target_update_invalid\"}");
                stop_all_current(5, 10);
                return PawMoveState::ERROR;
            }

            const auto feedback = get_joint_state();
            const auto now = std::chrono::steady_clock::now();
            for (size_t i = 0; i < ids.size(); ++i) {
                const auto id = ids[i];
                const auto& motor = motor_ctrl_datas_.at(id);
                if (motor.config.ignore || !motor.isEnabled()) continue;

                const double pct = std::max(5.0, std::min(95.0, next_positions[i]));
                const float next_target = mapPercentToLogicalRad(id, static_cast<float>(pct));
                const float previous_target = target_positions_rad[id];
                if (std::abs(next_target - previous_target) <= 1e-6f) continue;

                const float current_pos = feedback.count(id)
                    ? static_cast<float>(feedback.at(id).pos)
                    : last_positions[id];
                const float previous_error = previous_target - current_pos;
                const int previous_direction =
                    previous_error > 1e-6f ? 1 : (previous_error < -1e-6f ? -1 : 0);
                const float next_error = next_target - current_pos;
                const int next_direction =
                    next_error > 1e-6f ? 1 : (next_error < -1e-6f ? -1 : 0);
                const bool direction_reversed =
                    next_direction != 0 && next_direction != previous_direction;

                target_positions_rad[id] = next_target;
                stable_detection_enabled[id] = false;
                stable_check_positions[id] = current_pos;
                stable_detection_start_times[id] = now;
                is_stable[id] = false;

                if (direction_reversed) {
                    // 只有目标方向改变时才废弃旧方向的卡死和反冲状态；
                    // 同方向小幅更新继续累积检测时间，避免持续压制真实限位反冲。
                    in_impact_mode[id] = false;
                    impact_active[id] = false;
                    stuck_detected[id] = false;
                    stuck_detection_enabled[id] = false;
                    vr_stuck_cycle_count[id] = 0;
                    stuck_check_positions[id] = current_pos;
                    vr_stuck_check_positions[id] = current_pos;
                    stuck_detection_start_times[id] = now;
                    stuck_start_times[id] = now;
                    impact_start_times[id] = now;
                    last_positions[id] = current_pos;
                    prev_torque[id] = 0.0f;
                }

                std::ostringstream oss;
                oss << "{\"source\":\"can_sdk\",\"phase\":\"move\",\"event\":\"target_replaced\","
                    << "\"motor_id\":" << static_cast<int>(id) << ","
                    << "\"target_percent\":" << pct << ","
                    << "\"direction_reversed\":" << (direction_reversed ? "true" : "false") << "}";
                emit_debug(oss.str());
            }
        }

        // 刷新状态
        state = get_joint_state();

        // 计算速度与 PD 输出/冲击输出
        std::unordered_map<MotorId, float> new_torque;
        all_reached = true;

        for (const auto &kv : motor_ctrl_datas_) {
            MotorId id = kv.first;
            if (kv.second.config.ignore || !kv.second.isEnabled()) continue;

            float current_pos = state.count(id) ? static_cast<float>(state[id].pos) : kv.second.motor.position();
            float current_torque_feedback = state.count(id) ? static_cast<float>(state[id].torque) : kv.second.motor.torque();
            float current_velocity = (current_pos - last_positions[id]) / DEFAULT_DT;
            
            // 限位范围判定
            float s = joint_start_positions_[id];
            float e = joint_end_positions_[id];
            float total_range = std::abs(e - s);
            float error_threshold = total_range * (cfg_MIN_ERROR_PERCENT / 100.0f);
            
            // 使用目标位置
            float error = target_positions_rad[id] - current_pos;
            bool is_closing_direction = (error > 0);
            
            // 更新位置记录
            last_positions[id] = current_pos;
            
            // 限位范围判定（用于冲击检测）
            float limit_range_rad = total_range * (cfg_LIMIT_RANGE_PERCENT / 100.0f);
            float dist_to_open = std::abs(current_pos - s);
            float dist_to_close = std::abs(current_pos - e);
            bool in_impact_range = is_closing_direction ? (dist_to_open < limit_range_rad)
                                                        : (dist_to_close < limit_range_rad);
            
            // ========== 到位判据 ==========
            bool is_reached = std::abs(error) <= error_threshold;
            if (is_reached) {
                new_torque[id] = 0.0f;
                stuck_detected[id] = false;
                in_impact_mode[id] = false;
                impact_active[id] = false;
                stuck_detection_enabled[id] = false;
                vr_stuck_cycle_count[id] = 0;
                vr_stuck_check_positions[id] = current_pos;
                is_stable[id] = true;
                continue;
            }

            // 稳定检测（非 VR，且不在冲击范围内）
            if (!is_high_freq && !is_stable[id] && !in_impact_range) {
                float position_change = std::abs(current_pos - stable_check_positions[id]);
                float velocity_mag = std::abs(current_velocity);
                if (position_change < cfg_STABLE_POSITION_THRESHOLD && velocity_mag < cfg_STABLE_VELOCITY_THRESHOLD) {
                    if (!stable_detection_enabled[id]) {
                        stable_detection_enabled[id] = true;
                        stable_detection_start_times[id] = std::chrono::steady_clock::now();
                    } else {
                        auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                            std::chrono::steady_clock::now() - stable_detection_start_times[id]).count();
                        if (static_cast<float>(ms) >= cfg_STABLE_DETECTION_TIME_MS) {
                            is_stable[id] = true;
                        }
                    }
                } else {
                    stable_detection_enabled[id] = false;
                    stable_check_positions[id] = current_pos;
                }
            } else if (in_impact_range || is_high_freq) {
                stable_detection_enabled[id] = false;
                stable_check_positions[id] = current_pos;
            }

            if (!is_high_freq && is_stable[id]) {
                new_torque[id] = 0.0f;
                continue;
            }

            // ========== 控制逻辑 ==========
            if (!in_impact_mode[id]) {
                float adjusted_kp = cfg_KP;
                float adjusted_kd = cfg_KD;
                
                // 计算位置百分比，用于目标位置两端减速范围检测
                float target_percent = 0.0f;
                float current_percent = 0.0f;
                if (total_range > 1e-6f) {
                    target_percent = ((target_positions_rad[id] - s) / (e - s)) * 100.0f;
                    target_percent = std::max(0.0f, std::min(100.0f, target_percent));
                    current_percent = ((current_pos - s) / (e - s)) * 100.0f;
                    current_percent = std::max(0.0f, std::min(100.0f, current_percent));
                }
                float percent_diff = std::abs(current_percent - target_percent);
                
                // 在目标位置cfg_PRECISION_RANGE_PERCENT范围内调整kp/kd
                bool in_precision_range = (percent_diff <= cfg_PRECISION_RANGE_PERCENT);
                if (in_precision_range) {
                    adjusted_kp = adjusted_kp * cfg_PRECISION_KP_SCALE;
                    adjusted_kd = adjusted_kd * cfg_PRECISION_KD_SCALE;
                }
                
                float velocity_error = 0.0f - current_velocity;
                float pd_output = adjusted_kp * error + adjusted_kd * velocity_error;
                float torque_value = std::max(-cfg_MAX_CURRENT, std::min(cfg_MAX_CURRENT, pd_output));
                new_torque[id] = lowPassFilter(torque_value, prev_torque[id], cfg_ALPHA);
                prev_torque[id] = new_torque[id];
            }

            // VR 模式卡死检测：基于多周期小位移
            if (is_high_freq && !stuck_detected[id] && !in_impact_mode[id]) {
                // 首先检查目标位置与当前位置的差值
                float target_error = std::abs(target_positions_rad[id] - current_pos);
                // 只有当目标位置与当前位置差值大于阈值时，才进行卡死检测
                if (target_error > cfg_VR_TARGET_POSITION_THRESHOLD) {
                    float position_change = std::abs(current_pos - vr_stuck_check_positions[id]);
                    // 如果位置变化小于阈值，增加卡死计数
                    if (position_change < cfg_VR_STUCK_POSITION_THRESHOLD) {
                        vr_stuck_cycle_count[id]++;
                    } else {
                        // 位置有变化，重置计数和检测位置
                        vr_stuck_cycle_count[id] = 0;
                        vr_stuck_check_positions[id] = current_pos;
                    }
                    // 如果连续多个周期位置变化都很小，判定为卡死
                    if (vr_stuck_cycle_count[id] >= cfg_VR_STUCK_DETECTION_CYCLES) {
                        // VR模式下，只有在cfg_LIMIT_RANGE_PERCENT范围内才允许反冲
                        if (in_impact_range) {
                            // 在限位范围内，开始反冲
                            stuck_detected[id] = true;
                            in_impact_mode[id] = true;
                            impact_start_times[id] = std::chrono::steady_clock::now();
                            impact_active[id] = true;
                            std::cout << "[LEJU claw] 夹爪 " << id << " VR模式检测到卡死(" << (is_closing_direction ? "关闭方向" : "打开方向") << ")！目标位置差值:" << target_error << " rad，在限位范围内，开始" << (is_closing_direction ? "+" : "-") << cfg_IMPACT_CURRENT << "A冲击模式" << std::endl;
                            std::ostringstream oss;
                            oss << "{\"source\":\"can_sdk\",\"phase\":\"move\",\"event\":\"vr_stuck_detected\","
                                << "\"motor_id\":" << static_cast<int>(id) << ","
                                << "\"direction\":\"" << (is_closing_direction ? "close" : "open") << "\","
                                << "\"target_error\":" << target_error << ","
                                << "\"impact_current\":" << cfg_IMPACT_CURRENT << "}";
                            emit_debug(oss.str());
                        } else {
                            // 在限位范围外，不进行反冲，重置卡死检测状态
                            vr_stuck_cycle_count[id] = 0;
                            vr_stuck_check_positions[id] = current_pos;
                            std::cout << "[LEJU claw] 夹爪 " << id << " VR模式检测到卡死(" << (is_closing_direction ? "关闭方向" : "打开方向") << ")！目标位置差值:" << target_error << " rad，但不在限位范围内，不进行反冲" << std::endl;
                            std::ostringstream oss;
                            oss << "{\"source\":\"can_sdk\",\"phase\":\"move\",\"event\":\"vr_stuck_no_impact\","
                                << "\"motor_id\":" << static_cast<int>(id) << ","
                                << "\"direction\":\"" << (is_closing_direction ? "close" : "open") << "\","
                                << "\"target_error\":" << target_error << "}";
                            emit_debug(oss.str());
                        }
                    }
                } else {
                    // 目标位置与当前位置差值很小，重置卡死检测状态
                    vr_stuck_cycle_count[id] = 0;
                    vr_stuck_check_positions[id] = current_pos;
                }
            }

            // 非 VR 卡死检测（只在对应方向的限位范围内才允许卡死检测和反冲）
            if (in_impact_range && !is_high_freq) {
                auto ms_delay = std::chrono::duration_cast<std::chrono::milliseconds>(
                    std::chrono::steady_clock::now() - stuck_detection_start_times[id]).count();
                if (ms_delay >= STUCK_DETECTION_DELAY_MS && !stuck_detection_enabled[id]) {
                    stuck_detection_enabled[id] = true;
                    stuck_check_positions[id] = current_pos;
                    stuck_start_times[id] = std::chrono::steady_clock::now();
                }
                if (stuck_detection_enabled[id] && !stuck_detected[id] && !in_impact_mode[id]) {
                    float position_change = std::abs(current_pos - stuck_check_positions[id]);
                    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                        std::chrono::steady_clock::now() - stuck_start_times[id]).count();
                    // 只有在位置变化很小且时间足够长时才判定为卡死
                    if (position_change < cfg_STUCK_POSITION_THRESHOLD && ms > cfg_STUCK_DETECTION_TIME_MS) {
                        stuck_detected[id] = true;
                        in_impact_mode[id] = true;
                        impact_start_times[id] = std::chrono::steady_clock::now();
                        impact_active[id] = true;
                        std::cout << "[LEJU claw] 夹爪 " << id << " 检测到卡死(" << (is_closing_direction ? "关闭方向" : "打开方向") << ")！位置变化:" << position_change << " rad，开始" << (is_closing_direction ? "+" : "-") << cfg_IMPACT_CURRENT << "A冲击模式" << std::endl;
                        std::ostringstream oss;
                        oss << "{\"source\":\"can_sdk\",\"phase\":\"move\",\"event\":\"stuck_detected\","
                            << "\"motor_id\":" << static_cast<int>(id) << ","
                            << "\"direction\":\"" << (is_closing_direction ? "close" : "open") << "\","
                            << "\"position_change\":" << position_change << ","
                            << "\"impact_current\":" << cfg_IMPACT_CURRENT << "}";
                        emit_debug(oss.str());
                    } else if (position_change >= cfg_STUCK_POSITION_THRESHOLD) {
                        // 位置有变化，重置卡死检测状态
                        stuck_check_positions[id] = current_pos;
                        stuck_start_times[id] = std::chrono::steady_clock::now();
                    }
                }
            }

            // 冲击模式（只在对应方向的限位范围内才允许反冲）
            if (in_impact_mode[id] && in_impact_range) {
                auto impact_elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
                    std::chrono::steady_clock::now() - impact_start_times[id]).count();
                if (impact_active[id]) {
                    if (impact_elapsed < cfg_IMPACT_DURATION_MS) {
                        // 根据运动方向确定冲击电流方向。关闭方向：在开限位附近，使用+冲击电流；打开方向：在关限位附近，使用-冲击电流
                        new_torque[id] = is_closing_direction ? +cfg_IMPACT_CURRENT : -cfg_IMPACT_CURRENT;
                        if (impact_elapsed == 0) {
                            three_amp_current_usage_count++;
                            std::cout << "[LEJU claw] 夹爪 " << id << " 进入" << (is_closing_direction ? "+" : "-") << cfg_IMPACT_CURRENT << "A冲击模式 - 第" << three_amp_current_usage_count << "次" << std::endl;
                            std::ostringstream oss;
                            oss << "{\"source\":\"can_sdk\",\"phase\":\"move\",\"event\":\"impact_start\","
                                << "\"motor_id\":" << static_cast<int>(id) << ","
                                << "\"direction\":\"" << (is_closing_direction ? "close" : "open") << "\","
                                << "\"impact_current\":" << cfg_IMPACT_CURRENT << ","
                                << "\"impact_count\":" << three_amp_current_usage_count << "}";
                            emit_debug(oss.str());
                        }
                    } else {
                        impact_active[id] = false;
                        new_torque[id] = 0.0f;
                        impact_start_times[id] = std::chrono::steady_clock::now();
                    }
                } else {
                    if (impact_elapsed < cfg_IMPACT_INTERVAL_MS) {
                        new_torque[id] = 0.0f;
                    } else {
                        // 检查位置是否有变化，如果有变化则退出冲击模式
                        float position_change = std::abs(current_pos - stuck_check_positions[id]);
                        if (position_change > cfg_STUCK_POSITION_THRESHOLD) {
                            // 位置有变化，退出冲击模式，回到正常控制
                            in_impact_mode[id] = false;
                            stuck_detected[id] = false;
                            stuck_detection_enabled[id] = false;
                            vr_stuck_cycle_count[id] = 0;  // 重置VR卡死检测计数
                            vr_stuck_check_positions[id] = current_pos;  // 重置VR卡死检测位置
                            std::cout << "[LEJU claw] 夹爪 " << id << " 冲击成功，退出冲击模式，重置VR卡死检测状态" << std::endl;
                            std::ostringstream oss;
                            oss << "{\"source\":\"can_sdk\",\"phase\":\"move\",\"event\":\"impact_success\","
                                << "\"motor_id\":" << static_cast<int>(id) << ","
                                << "\"position_change\":" << position_change << "}";
                            emit_debug(oss.str());
                        } else {
                            // 位置无变化，继续冲击
                            impact_active[id] = true;
                            impact_start_times[id] = std::chrono::steady_clock::now();
                            std::cout << "[LEJU claw] 夹爪 " << id << " 冲击模式继续，位置无变化，继续冲击" << std::endl;
                            std::ostringstream oss;
                            oss << "{\"source\":\"can_sdk\",\"phase\":\"move\",\"event\":\"impact_retry\","
                                << "\"motor_id\":" << static_cast<int>(id) << ","
                                << "\"position_change\":" << position_change << "}";
                            emit_debug(oss.str());
                        }
                    }
                }
            }
        }

        // 到位/稳定判定
        all_reached = true;
        for (const auto &kv : motor_ctrl_datas_) {
            MotorId id = kv.first;
            if (kv.second.config.ignore || !kv.second.isEnabled()) continue;
            
            // VR模式下，检查到位误差（基于行程百分比）
            if (is_high_freq) {
                float current_pos = state.count(id) ? static_cast<float>(state[id].pos) : kv.second.motor.position();
                float error = target_positions_rad[id] - current_pos;
                float s = joint_start_positions_[id];
                float e = joint_end_positions_[id];
                float total_range = std::abs(e - s);
                float error_threshold = total_range * (cfg_MIN_ERROR_PERCENT / 100.0f);
                if (std::abs(error) > error_threshold) { all_reached = false; break; }
            } else {
                // 非VR模式下，检查稳定状态
                if (!is_stable[id]) { all_reached = false; break; }
            }
        }

        // 持续下发：更新位置和torque
        for (const auto &kv : motor_ctrl_datas_) {
            MotorId id = kv.first;
            if (kv.second.config.ignore || !kv.second.isEnabled()) continue;
            motorevo::RevoMotorCmd_t cmd = target_cmd[id];
            cmd.pos = target_positions_rad[id];
            cmd.vel = 0.0;
            cmd.torque = new_torque.count(id) ? new_torque[id] : 0.0f;
            cmd.kp = kv.second.config.default_params.kp_pos;
            cmd.kd = kv.second.config.default_params.kd_pos;
            updateMotorCmd(id, cmd.pos, cmd.vel, cmd.torque, cmd.kp, cmd.kd);
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(DEFAULT_DT * 1000)));
    }

    // 退出前电流清零（跳过非VR保持模式的电机）
    for (const auto &kv : motor_ctrl_datas_) {
        MotorId id = kv.first;
        if (kv.second.config.ignore || !kv.second.isEnabled()) continue;
        if (cfg_ENABLE_NON_VR_HOLD && non_vr_hold_mode_.count(id) && non_vr_hold_mode_[id]) {
            // 保持模式：更新 non_vr_hold_target_rad_ 为当前目标位置
            auto itp = positions_percent.find(id);
            if (itp != positions_percent.end()) {
                non_vr_hold_target_rad_[id] = target_positions_rad[id];
            }
            continue;
        }
        motorevo::RevoMotorCmd_t cmd = target_cmd[id];
        cmd.torque = 0.0f;
        updateMotorCmd(id, cmd.pos, cmd.vel, cmd.torque, cmd.kp, cmd.kd);
    }

    // 有保持模式时保持 target_updated_ 为 true
    if (cfg_ENABLE_NON_VR_HOLD) {
        for (const auto &kv : motor_ctrl_datas_) {
            if (non_vr_hold_mode_.count(kv.first) && non_vr_hold_mode_[kv.first]) {
                target_updated_ = true;
                break;
            }
        }
    }

    // 记录本次目标位置（供下次入口跳过检测）
    if (cfg_ENABLE_NON_VR_HOLD) {
        for (const auto &kv : motor_ctrl_datas_) {
            MotorId id = kv.first;
            auto pit = positions_percent.find(id);
            if (pit != positions_percent.end()) {
                last_move_paw_target_percent_[id] = pit->second;
            }
        }
    }

    emit_debug("{\"source\":\"can_sdk\",\"phase\":\"move\",\"event\":\"done\"}");
    return PawMoveState::LEFT_REACHED_RIGHT_REACHED;
}


// 返回升序的 MotorId 列表
std::vector<MotorId> LeJuClawCan::get_sorted_motor_ids() const {
    std::vector<MotorId> ids;
    ids.reserve(motor_ctrl_datas_.size());
    for (const auto &kv : motor_ctrl_datas_) ids.push_back(kv.first);
    std::sort(ids.begin(), ids.end(), [](MotorId a, MotorId b){
        return static_cast<unsigned int>(a) < static_cast<unsigned int>(b);
    });
    return ids;
}

std::string LeJuClawCan::get_home_path()
{
    const char *sudo_user = getenv("SUDO_USER");
    if (sudo_user != nullptr)
    {
        passwd *pw = getpwnam(sudo_user);
        if (pw != nullptr)
        {
            std::string path = pw->pw_dir;
            path += "/.config/lejuconfig";
            return path;
        }
    }
    else
    {
        uid_t uid = getuid();
        passwd *pw = getpwuid(uid);
        if (pw != nullptr)
        {
            std::string path = pw->pw_dir;
            path += "/.config/lejuconfig";
            return path;
        }
    }
    return "";
}

void LeJuClawCan::load_leju_params(const std::string& config_file)
{
    try {
        YAML::Node cfg = YAML::LoadFile(config_file);
        // 可采用平铺或lejuclaw命名空间两种写法
        YAML::Node root = cfg["lejuclaw"].IsDefined() ? cfg["lejuclaw"] : cfg;

        auto set_float = [&](const char* key, float &dst){ if (root[key].IsDefined()) dst = root[key].as<float>(); };
        auto set_int   = [&](const char* key, int   &dst){ if (root[key].IsDefined()) dst = root[key].as<int>(); };
        auto set_bool  = [&](const char* key, bool  &dst){ if (root[key].IsDefined()) dst = root[key].as<bool>(); };

        set_float("kp", cfg_KP);
        set_float("kd", cfg_KD);
        set_float("alpha", cfg_ALPHA);
        set_float("max_current", cfg_MAX_CURRENT);
        set_float("min_error_percent", cfg_MIN_ERROR_PERCENT);
        set_float("dt", cfg_DT);

        set_float("limit_range_percent", cfg_LIMIT_RANGE_PERCENT);
        set_float("impact_current", cfg_IMPACT_CURRENT);
        set_float("impact_duration_ms", cfg_IMPACT_DURATION_MS);
        set_float("impact_interval_ms", cfg_IMPACT_INTERVAL_MS);

        set_float("stuck_detection_delay_ms", cfg_STUCK_DETECTION_DELAY_MS);
        set_float("stuck_detection_time_ms", cfg_STUCK_DETECTION_TIME_MS);
        set_float("stuck_position_threshold", cfg_STUCK_POSITION_THRESHOLD);

        set_float("stable_position_threshold", cfg_STABLE_POSITION_THRESHOLD);
        set_float("stable_velocity_threshold", cfg_STABLE_VELOCITY_THRESHOLD);
        set_float("stable_detection_time_ms", cfg_STABLE_DETECTION_TIME_MS);

        set_float("precision_range_percent", cfg_PRECISION_RANGE_PERCENT);
        set_float("precision_kp_scale", cfg_PRECISION_KP_SCALE);
        set_float("precision_kd_scale", cfg_PRECISION_KD_SCALE);

        set_int("vr_control_timeout_ms", cfg_VR_CONTROL_TIMEOUT_MS);
        set_int("vr_stuck_detection_cycles", cfg_VR_STUCK_DETECTION_CYCLES);
        set_float("vr_stuck_position_threshold", cfg_VR_STUCK_POSITION_THRESHOLD);
        set_float("vr_target_position_threshold", cfg_VR_TARGET_POSITION_THRESHOLD);

        set_bool("enable_non_vr_hold", cfg_ENABLE_NON_VR_HOLD);
        set_float("non_vr_hold_current", cfg_NON_VR_HOLD_CURRENT);
        if (root["non_vr_hold_trigger_percents"].IsDefined()) {
            cfg_NON_VR_HOLD_TRIGGER_PERCENTS = root["non_vr_hold_trigger_percents"].as<std::string>();
        }
        // 解析逗号分隔的触发值为列表
        non_vr_hold_trigger_set_.clear();
        {
            std::string str = cfg_NON_VR_HOLD_TRIGGER_PERCENTS;
            size_t pos = 0;
            while (pos < str.size()) {
                size_t next = str.find(',', pos);
                std::string token = (next == std::string::npos) ? str.substr(pos) : str.substr(pos, next - pos);
                pos = (next == std::string::npos) ? str.size() : next + 1;
                // 去除前后空格
                while (!token.empty() && token.front() == ' ') token.erase(token.begin());
                while (!token.empty() && token.back() == ' ') token.pop_back();
                if (!token.empty()) {
                    try { non_vr_hold_trigger_set_.push_back(std::stof(token)); }
                    catch (...) {}
                }
            }
        }

        set_float("zero_control_kp", cfg_ZERO_CONTROL_KP);
        set_float("zero_control_kd", cfg_ZERO_CONTROL_KD);
        set_float("zero_control_alpha", cfg_ZERO_CONTROL_ALPHA);
        set_float("zero_control_max_current", cfg_ZERO_CONTROL_MAX_CURRENT);
        set_float("stall_current_threshold", cfg_STALL_CURRENT_THRESHOLD);
        set_float("stall_velocity_threshold", cfg_STALL_VELOCITY_THRESHOLD);
        set_float("zero_control_dt", cfg_ZERO_CONTROL_DT);
        set_int("zero_find_timeout_ms", cfg_ZERO_FIND_TIMEOUT_MS);
        set_int("zero_wait_ms", cfg_ZERO_WAIT_MS);
        set_float("open_limit_adjustment", cfg_OPEN_LIMIT_ADJUSTMENT);
        set_float("close_limit_adjustment", cfg_CLOSE_LIMIT_ADJUSTMENT);
        set_float("target_velocity", cfg_TARGET_VELOCITY);
        set_float("zero_target_velocity_max", cfg_ZERO_TARGET_VELOCITY_MAX);
        set_float("zero_stuck_position_threshold", cfg_ZERO_STUCK_POSITION_THRESHOLD);
        set_float("open_position_change_threshold", cfg_OPEN_POSITION_CHANGE_THRESHOLD);
        set_float("close_stuck_current_threshold", cfg_CLOSE_STUCK_CURRENT_THRESHOLD);
        set_float("close_stuck_detection_threshold", cfg_CLOSE_STUCK_DETECTION_THRESHOLD);
        set_int("close_startup_delay_ms", cfg_CLOSE_STARTUP_DELAY_MS);
        set_float("close_impact_current", cfg_CLOSE_IMPACT_CURRENT);
        set_int("close_impact_duration_ms", cfg_CLOSE_IMPACT_DURATION_MS);
        set_int("close_impact_interval_ms", cfg_CLOSE_IMPACT_INTERVAL_MS);
        set_int("close_max_attempts", cfg_CLOSE_MAX_ATTEMPTS);

        // 断流看门狗
        set_int("stream_watchdog_timeout_ms", cfg_STREAM_WATCHDOG_TIMEOUT_MS);
        if (cfg_STREAM_WATCHDOG_TIMEOUT_MS <= 0) {
            cfg_STREAM_WATCHDOG_TIMEOUT_MS = STREAM_WATCHDOG_TIMEOUT_MS;
        }

        LEJUCLAW_LOG_INFO("成功加载配置文件: %s", config_file.c_str());
    } catch (const YAML::Exception &e) {
        LEJUCLAW_LOG_WARN("读取配置文件 %s 发生YAML异常: %s，使用默认参数", config_file.c_str(), e.what());
    } catch (const std::exception &e) {
        LEJUCLAW_LOG_WARN("读取配置文件 %s 发生异常: %s，使用默认参数", config_file.c_str(), e.what());
    }
}

} // namespace lejuclaw_can
