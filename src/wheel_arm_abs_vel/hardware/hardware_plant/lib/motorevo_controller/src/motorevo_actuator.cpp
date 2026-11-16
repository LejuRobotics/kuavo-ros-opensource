#include "motorevo/motorevo_actuator.h"
#include "motorevo/motor_ctrl.h"
#include "motorevo/motor_log.h"
#include "canbus_sdk/canbus_sdk.h"
#include "canbus_sdk/config_parser.h"

#include <iostream>
#include <unistd.h>
#include <pwd.h>
#include <fstream>
#include <iomanip>
#include <algorithm>
#include <filesystem>
#include <yaml-cpp/yaml.h>
#include <cmath>
#include <thread>

namespace {

// 从文件加载零点偏移
std::vector<float> _LoadZeroOffsets() {
    std::vector<float> zero_offsets;
    std::string zero_file_path = motorevo::MotorevoActuator::getDefaultZeroFilePath();

    // 先判断零点文件是否存在
    if (!std::filesystem::exists(zero_file_path)) {
        RWLOG_WARNING("Zero file does not exist: %s", zero_file_path.c_str());
        return zero_offsets;
    }

    try {
        YAML::Node zero_config = YAML::LoadFile(zero_file_path);
        if (zero_config["arms_zero_position"]) {
            // cppcheck-suppress useStlAlgorithm
            for (const auto& zero_node : zero_config["arms_zero_position"]) {
                zero_offsets.push_back(zero_node.as<float>());
            }
            RWLOG_I("Loaded %zu zero offsets from file: %s", zero_offsets.size(), zero_file_path.c_str());
        } else {
            RWLOG_WARNING("No arms_zero_position found in zero file: %s", zero_file_path.c_str());
        }
    } catch (const YAML::Exception& e) {
        RWLOG_WARNING("Failed to load zero file %s: %s", zero_file_path.c_str(), e.what());
    } catch (...) {
        RWLOG_WARNING("Failed to load zero file: %s", zero_file_path.c_str());
    }

    return zero_offsets;
}

}

namespace motorevo {

MotorevoActuator::MotorevoActuator(const std::string& config_file, bool cali, int control_frequency)
    : config_file_(config_file)
    , cali_(cali)
    , running_(false)
    , control_frequency_(control_frequency)
    , rt_enabled_(true) {
    // RWLOG_I("MotorevoActuator constructed with config file: %s, cali: %s, control frequency: %d Hz",
    //          config_file_.c_str(), cali_ ? "True" : "False", control_frequency);
}

void MotorevoActuator::setRtEnabled(bool enable) {
    rt_enabled_.store(enable, std::memory_order_release);
    RWLOG_I("MotorevoActuator RT direct-send mode %s", enable ? "enabled" : "disabled");
}

bool MotorevoActuator::isRtEnabled() const {
    return rt_enabled_.load(std::memory_order_acquire);
}

MotorevoActuator::~MotorevoActuator() {
    reset();
    RWLOG_I("MotorevoActuator destructor called");
}

int MotorevoActuator::init() {
    RWLOG_SUCCESS(
        "\n"
        "===============================================\n"
        "Motorevo C++ SDK\n"
        "Calibration Mode: %s\n"
        "===============================================",
        cali_ ? "True" : "False"
    );

    /** 配置文件读取与解析 */
    canbus_sdk::ConfigParser parser;
    if (!parser.parseFromFile(config_file_)) { // 会抛异常(无所谓), 配置文件都解析失败,程序就没法玩了
        RWLOG_FAILURE("Init, Failed to parse config file: %s", config_file_.c_str());
        return 1;
    }

    /** 获取CAN总线配置, 没有就返回失败 */
    auto canbus_configs = parser.getCanBusConfigs();
    if (canbus_configs.empty()) {
        RWLOG_FAILURE("Init, No CAN bus configurations found in config file");
        return 1;
    }
    
    /** 循环打开CAN总线，失败则返回 */
    RWLOG_SUCCESS("============================== CAN 总线初始化开始 ==============================");
    canbus_sdk::CanBusController::getInstance().init(); // 重复初始化无所谓
    for (const auto& bus_config : canbus_configs) {
        // 暂不实现enable标记功能
        // if (!bus_config.enable) {
        //     RWLOG_I("CAN CAN总线 %s is disabled, skipping", bus_config.name.c_str());
        //     continue;
        // }

        // 打开CAN总线
        auto result = canbus_sdk::CanBusController::getInstance().openCanBus(
            bus_config.name.c_str(), bus_config.type, bus_config.bitrate);
        if (!result.has_value()) {
            RWLOG_FAILURE("打开CAN总线 %s 失败: %s",
                bus_config.name.c_str(),canbus_sdk::errorToString(result.error()));
            return 2;
        }

        uint8_t bus_id = result.value();
        RWLOG_SUCCESS("CAN总线 %s 打开成功, bus_id: %d", bus_config.name.c_str(), bus_id);
    }
    RWLOG_SUCCESS("============================== CAN 总线初始化完成 ==============================");

    /** 设备信息转电机配置,并绑定总线 */
    RWLOG_SUCCESS("=========================== 开始向CAN总线绑定电机设备 ===========================");
    std::vector<MotorControlRef> motor_temp_refs;
    std::unordered_map<motorevo::MotorId, motorevo::RevoMotorConfig_t> revo_motor_configs;
    size_t bus_groups_index = 0;
    for (const auto& canbus_config : canbus_configs) {
        RWLOG_I("Processing CAN bus: %s", canbus_config.name.c_str());

        // 获取该CAN总线上的电机设备
        auto motor_devices = parser.getDevices(canbus_config.name, canbus_sdk::DeviceType::MOTOR);
        if (motor_devices.empty()) {
            RWLOG_WARNING("No motor devices found on CAN bus: %s", canbus_config.name.c_str());
            continue; // 没有电机设备, 跳过
        }

        RWLOG_I("Found %zu motor devices on CAN bus '%s'", motor_devices.size(), canbus_config.name.c_str());

        // 转换设备配置为RevoMotorConfig_t
        for (const auto& device : motor_devices) {
            if (device.motor_params.size() != 7) {
                RWLOG_FAILURE("电机设备 '%s' 的 motor_params 参数数量不正确 (%zu 个元素，期望 7 个), 请检查件'~/.config/lejuconfig/'下的配置文件",
                              device.name.c_str(), device.motor_params.size());
                return 1;
            }

            motorevo::MotorId id = static_cast<motorevo::MotorId>(device.device_id);
            // 初始化电机参数
            motorevo::RevoMotorConfig_t config;
            config.id = id;
            config.name = device.name;
            config.negtive = device.negtive;
            config.ignore = device.ignore;
            config.zero_offset = 0.0f;      // 这里赋值只是暂时的后面加载零点文件会赋值
            config.ratio = device.ratio;     // 减速比
            config.default_params.vel = device.motor_params[0];
            config.default_params.kp_pos = device.motor_params[1];
            config.default_params.kd_pos = device.motor_params[2];
            config.default_params.tor = device.motor_params[3];
            config.default_params.kp_vel = device.motor_params[4];
            config.default_params.kd_vel = device.motor_params[5];
            config.default_params.ki_vel = device.motor_params[6];
            motor_temp_refs.push_back({bus_groups_index, id, device.name, device.ignore, config.default_params});
            revo_motor_configs.insert(std::make_pair(id, config));
        }

        // 保存CAN总线组信息
        CanBusGroup can_group;
        can_group.name = canbus_config.name;
        can_group.motor_control = std::make_shared<motorevo::RevoMotorControl>(canbus_config.name);
        can_bus_groups_.push_back(can_group);
        bus_groups_index++;
    }
    RWLOG_SUCCESS("=========================== 所有电机设备绑定总线完毕 ===========================");

    if (can_bus_groups_.empty()) {
        RWLOG_FAILURE("No enabled CAN buses with motor devices found");
        return 2;
    }

    /** 索引映射构建:索引与电机ID映射 */
    buildIndexMappings(motor_temp_refs);

    /** 零点文件读取 */
    RWLOG_I("=============================== 零点文件读取开始 ===============================");
    std::vector<float> zero_offsets = _LoadZeroOffsets();


    if(!zero_offsets.empty()) {
        std::cout << "从配置文件加载零点:\n";
        for (size_t i = 0; i < motor_refs_.size(); ++i) {            
            auto &ref = motor_refs_[i];
            float offset = 0.0f;         // 设置零点偏移
            if (zero_offsets.size() > i) {
                offset = zero_offsets[i]; // 正式从零点文件更新零点偏移
            } else {
                RWLOG_WARNING("No zero offset found for motor ID %d at index %zu, using current position as zero point with magic number identifier", ref.id, i);
                // 获取当前电机位置作为零点
                // 注意：这里需要在电机控制器初始化后才能获取当前位置
                // 由于此时电机控制器还未初始化，先设置magic number，稍后在控制器初始化后更新
                offset = kOFFSET_MAGIC_NUMBER;
            }
            revo_motor_configs[ref.id].zero_offset = offset; // 从配置文件更新零点偏移
            std::cout << " " << std::fixed << std::setprecision(6) << offset;
        }
        std::cout << std::endl;
    }
    RWLOG_I("=============================== 零点文件读取%s ===============================", zero_offsets.empty() ? "失败" : "成功");

    /** 控制线程启动: 必须要在 init之前 */
    RWLOG_SUCCESS("=============================== 控制线程开始启动 ===============================");
    if (!startControlThread()) {
        RWLOG_FAILURE("Failed to start control thread");
        return 3;
    }
    setControlThreadAffinity(7);
    RWLOG_SUCCESS("=============================== 控制线程启动成功 ===============================");

    // 在循环开始前复制一次pending数据，让所有CAN总线组都能获取到
    std::map<MotorId, float> positions_to_apply;
    {
        std::lock_guard<std::mutex> lock(pending_target_positions_mutex_);
        if (!pending_initial_target_positions_.empty()) {
            positions_to_apply = pending_initial_target_positions_;
            // 清空pending数据，避免重复应用
            pending_initial_target_positions_.clear();
        }
    }

    // A later bus failure/exception must also disable already enabled buses.
    struct BusInitRollback {
        decltype(can_bus_groups_)& groups;
        bool complete{false};
        ~BusInitRollback() {
            if (complete) return;
            for (const auto& group : groups) {
                try {
                    if (group.motor_control) group.motor_control->disableAll();
                } catch (...) {
                    RWLOG_FAILURE("Exception while disabling CAN bus: %s", group.name.c_str());
                }
            }
        }
    } rollback{can_bus_groups_};

    // 初始化电机控制器
    for (size_t i = 0; i < can_bus_groups_.size(); ++i) {
        const auto& group = can_bus_groups_[i];

        // 从motor_refs_中找出属于当前CAN总线的电机配置
        std::vector<motorevo::RevoMotorConfig_t> bus_motor_configs;
        for (const auto& ref : motor_refs_) {
            if (ref.bus_group_index == i) {
                bus_motor_configs.push_back(revo_motor_configs[ref.id]);
            }
        }

        // 应用待设置的目标初始位置（如果之前通过 setInitialTargetPositions() 设置了）
        // 在锁外应用目标位置，避免在持有锁时调用可能较慢的操作
        if (!positions_to_apply.empty()) {
            group.motor_control->setInitialTargetPositions(positions_to_apply);
            RWLOG_I("Applied pending initial target positions to CAN bus: %s", group.name.c_str());
        }

        // 初始化电机控制器
        // 注意：电机回零时的目标初始位置应该由外部通过 setInitialTargetPositions() 接口设置
        // 不要在模块内部硬编码电机ID和角度值
        if (!group.motor_control->init(bus_motor_configs, cali_, skipBootMoveToZero())) {
            RWLOG_FAILURE("Failed to initialize motor controller for CAN bus: %s", group.name.c_str());
            return 3;
        }
        RWLOG_SUCCESS("Successfully initialized CAN bus: %s with %zu motors",
                    group.name.c_str(), bus_motor_configs.size());
    }

    /** 零点文件保存 */
    if(cali_ || zero_offsets.empty()) {
        if(zero_offsets.empty()) {
            RWLOG_WARNING("No zero offsets found, writing new zero offsets to file");
        }
        saveZeroToFile();
    }

    /** RT 直发模式：将电机控制帧注册为 RT 源（绕过 ring 队列，由 sender 线程直发） */
    if (rt_enabled_.load(std::memory_order_acquire)) {
        RWLOG_SUCCESS("=============================== RT 直发模式启动 ===============================");
        // 交接顺序（重要）：
        // 1) 先置 rt_active_=true 让控制线程停止 write() 泵帧；
        // 2) quiesce 握手：等控制线程 loop_epoch_ 前进 >=2 轮。epoch 在每轮 write() 返回后才递增，
        //    前进 2 轮说明最近一整轮是在 rt_active_=true 可见后完整跑完的（该轮已跳过 write()），
        //    即不再有在途 write()。否则 registerRtSource 内 setRtSend(true) 会让残留的
        //    controlPTM 走 sendMessageRtUnlocked 不持锁直发，与 sender 线程竞争；
        // 3) 再逐总线注册 RT 源，由 sender 线程接管发送。失败则注销已注册的总线并整体回滚。
        rt_active_.store(true, std::memory_order_seq_cst);
        {
            const uint64_t epoch_start = loop_epoch_.load(std::memory_order_acquire);
            const auto period = std::chrono::microseconds(1000000 / control_frequency_.load());
            const auto deadline = std::chrono::steady_clock::now() + 4 * period + std::chrono::milliseconds(5);
            while (loop_epoch_.load(std::memory_order_acquire) < epoch_start + 2) {
                if (std::chrono::steady_clock::now() >= deadline) {
                    RWLOG_WARNING("RT handover: quiesce handshake timeout (control thread stalled?), proceeding after deadline");
                    break;
                }
                std::this_thread::sleep_for(std::chrono::microseconds(200));
            }
        }
        std::vector<motorevo::RevoMotorControl*> rt_registered;
        for (auto& group : can_bus_groups_) {
            if (group.motor_control) {
                if (!group.motor_control->registerRtSource()) {
                    RWLOG_FAILURE("Failed to register RT source for bus: %s", group.name.c_str());
                    // 回滚：先注销已成功注册的总线（内部会 setRtSend(false)+注销 sender 回调），
                    // 再恢复控制线程 write() 泵帧。否则已注册总线 rt_send_ 仍为 true，
                    // 控制线程恢复后 controlPTM 会走不持锁直发，与 sender 线程竞争。
                    for (auto* mc : rt_registered) {
                        mc->unregisterRtSource();
                    }
                    rt_active_.store(false, std::memory_order_release);
                    return 3;
                }
                rt_registered.push_back(group.motor_control.get());
                RWLOG_SUCCESS("RT 源注册成功: CAN bus %s", group.name.c_str());
            }
        }
        RWLOG_SUCCESS("=============================== RT 直发模式启动完成 ===============================");
    }

    /** 完成初始化 */
    RWLOG_SUCCESS("MotorevoActuator 初始化完毕，共 %zu 个CAN总线，%zu 个电机设备", can_bus_groups_.size(), motor_refs_.size());

    // 打印设备信息表格
    std::cout << "\n=== 设备详细信息 ===\n";
    std::cout << "Index\tID\tName\tCAN Bus\tNegtive\tIgnore\tZero Offset\tParams" << std::endl;
    std::cout << std::string(132, '-') << std::endl;

    for (size_t i = 0; i < motor_refs_.size(); ++i) {
        const auto& motor_ref = motor_refs_[i];
        const auto& group = can_bus_groups_[motor_ref.bus_group_index];

        // 从motor_control获取最新的配置
        float zero_offset = 0.0f;
        bool negtive = false;
        if (group.motor_control) {
            auto config = group.motor_control->getMotorConfig(motor_ref.id);
            zero_offset = config.zero_offset;
            negtive = config.negtive;
        }

        const auto& params = motor_ref.rt_params;

        std::cout << i << "\t"
                  << static_cast<int>(motor_ref.id) << "\t"
                  << motor_ref.name << "\t"
                  << group.name << "\t"
                  << (negtive ? "\033[33mYes\033[0m" : "No") << "\t"
                  << (motor_ref.ignore ? "\033[32mYes\033[0m" : "No") << "\t"
                  << std::fixed << std::setprecision(3) << zero_offset << "\t"
                  << std::fixed << std::setprecision(2) << params.vel << ","
                  << params.kp_pos << "," << params.kd_pos << "," << params.tor << ","
                  << params.kp_vel << "," << params.kd_vel << "," << params.ki_vel << std::endl;
    }
    std::cout << std::string(132, '=') << std::endl;

    rollback.complete = true;
    return 0;
}

std::vector<double> MotorevoActuator::getPositions() {
    if (!running_){
        return {};
    }
    std::vector<double> all_positions(motor_refs_.size(), 0.0);

    for (const auto& group : can_bus_groups_) {
        auto positions = group.motor_control->getPositions();
        for (const auto& [id, pos] : positions) {
            all_positions[id2indexs_[id]] = static_cast<double>(pos);
        }
    }

    return all_positions;
}

std::vector<double> MotorevoActuator::getVelocities() {
    if (!running_){
        return {};
    }
    std::vector<double> all_velocities(motor_refs_.size(), 0.0);

    for (const auto& group : can_bus_groups_) {
        auto velocities = group.motor_control->getVelocities();
        for (const auto& [id, vel] : velocities) {
            all_velocities[id2indexs_[id]] = static_cast<double>(vel);
        }
    }

    return all_velocities;
}

std::vector<double> MotorevoActuator::getTorques() {
    if (!running_){
        return {};
    }
    std::vector<double> all_torques(motor_refs_.size(), 0.0);

    for (const auto& group : can_bus_groups_) {
        auto torques = group.motor_control->getTorques();
        for (const auto& [id, torque] : torques) {
            all_torques[id2indexs_[id]] = static_cast<double>(torque);
        }
    }

    return all_torques;
}

void MotorevoActuator::setTargets(const std::vector<RevoMotorCmd_t>& targets) {
    if (!running_){
        return;
    }
    // 按CAN总线分组目标命令
    std::vector<std::map<motorevo::MotorId, motorevo::RevoMotorCmd_t>> bus_targets(can_bus_groups_.size());

    // 按索引分配目标命令，使用motor_refs_缓存进行O(1)访问
    for (size_t i = 0; i < targets.size() && i < motor_refs_.size(); ++i) {
        const auto& cmd = targets[i];
        const auto& motor_ref = motor_refs_[i];

        // 跳过被忽略的设备
        if (motor_ref.ignore) {
            continue;
        }

        motorevo::MotorId motor_id = motor_ref.id;

        // 直接使用motor_ref中的信息，无需查找
        motorevo::RevoMotorCmd_t revo_cmd;
        revo_cmd.pos = static_cast<float>(cmd.pos);
        revo_cmd.vel = static_cast<float>(cmd.vel);

        // 限制torque在±10范围内
        revo_cmd.torque = std::clamp(static_cast<float>(cmd.torque), -10.0f, 10.0f);
        revo_cmd.scalar_before10 = static_cast<float>(cmd.torque);
        revo_cmd.kp = static_cast<float>(cmd.kp);
        revo_cmd.kd = static_cast<float>(cmd.kd);

        // 直接使用bus_group_index获取对应的CanBusGroup
        bus_targets[motor_ref.bus_group_index][motor_id] = revo_cmd;
    }

    // 直接设置目标命令到控制器
    for (size_t i = 0; i < bus_targets.size(); ++i) {
        if (!bus_targets[i].empty()) {
            can_bus_groups_[i].motor_control->setTargetPositions(bus_targets[i]);
        }
    }
}


static std::string _GetConfigRootPath() {
    const char* sudo_user = getenv("SUDO_USER");
    passwd* pw = nullptr;
    
    if (sudo_user != nullptr) {
        pw = getpwnam(sudo_user);
    } else {
        pw = getpwuid(getuid());
    }
    
    if (pw != nullptr) {
        return std::string(pw->pw_dir) + "/.config/lejuconfig";
    }
    
    return "";
}

std::string MotorevoActuator::getDefaultZeroFilePath() {
    std::string config_root = _GetConfigRootPath();
    if (config_root.empty()) {
        throw std::runtime_error("Failed to get config root path");
    }
    return config_root + "/arms_zero.yaml";
}

bool MotorevoActuator::startControlThread() {
    if (running_) {
        RWLOG_WARNING("Control thread is already running");
        return false;
    }

    if (control_frequency_ <= 0) {
        RWLOG_FAILURE("Control frequency must be positive");
        return false;
    }

    running_ = true;

    control_thread_ = std::thread(&MotorevoActuator::controlThreadFunc, this);

    RWLOG_SUCCESS("Control thread started with frequency %d Hz", control_frequency_.load());
    return true;
}

bool MotorevoActuator::setControlThreadAffinity(int cpu_core) {
    if (!running_ || !control_thread_.joinable()) {
        RWLOG_FAILURE("设置 CPU 亲和性失败, 控制线程未运行");
        return false;
    }

    // 1. 设置控制线程CPU亲和性
    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    CPU_SET(cpu_core, &cpuset);

    int rc = pthread_setaffinity_np(control_thread_.native_handle(), sizeof(cpu_set_t), &cpuset);
    if (rc != 0) {
        RWLOG_FAILURE("设置控制线程CPU亲和性到核心%d失败，错误：%s",
                     cpu_core, strerror(rc));
        return false;
    }

    RWLOG_SUCCESS("控制线程已绑定到CPU核心%d", cpu_core);

    // 2. 设置CAN总线发送线程CPU亲和性（绑定到同一个核心）
    if (canbus_sdk::CanBusController::getInstance().setSenderThreadAffinity(cpu_core)) {
        RWLOG_SUCCESS("CAN总线发送线程已绑定到CPU核心%d", cpu_core);
    } else {
        RWLOG_WARNING("设置CAN总线发送线程CPU亲和性到核心%d失败", cpu_core);
    }

    // 3. 设置每个CAN总线接收线程CPU亲和性（绑定到同一个核心）
    for (size_t i = 0; i < can_bus_groups_.size(); ++i) {
        const std::string& bus_name = can_bus_groups_[i].name;

        if (canbus_sdk::CanBusController::getInstance().setRecvThreadAffinity(bus_name, cpu_core)) {
            RWLOG_SUCCESS("CAN总线%s接收线程已绑定到CPU核心%d", bus_name.c_str(), cpu_core);
        } else {
            RWLOG_WARNING("设置CAN总线%s接收线程CPU亲和性到核心%d失败", bus_name.c_str(), cpu_core);
        }
    }

    RWLOG_SUCCESS("所有线程已绑定到CPU核心%d", cpu_core);
    return true;
}

void MotorevoActuator::controlThreadFunc() {
    RWLOG_I("MotorevoActuator: Control thread started");

    auto period = std::chrono::microseconds(1000000 / control_frequency_.load());

    // RT 直发模式下，电机控制帧由 sender 线程的 RT slot 直接下发（绕过 ring 队列），
    // control_thread 不再周期 write() 发帧，只保留节拍。
    // 注意：RT 源要等 init()（含 moveToZero 插值）完成后才注册（rt_active_=true）。
    // 注册之前必须继续由本线程 write() 泵帧（此时 rt_send_=false，帧走 ring，行为与旧版一致），
    // 否则 moveToZero 的插值命令无人发送，RT 接管瞬间会直接跳到最终目标位置（手臂跳变）。
    bool rt_handover_logged = false;

    while (running_) {
        auto start_time = std::chrono::steady_clock::now();

        // 向每个CAN总线发送write命令（RT 源接管后跳过，发送由 RT slot 接管）
        const bool rt_active = rt_active_.load(std::memory_order_acquire);
        if (!rt_active) {
            for (const auto& group : can_bus_groups_) {
                group.motor_control->write();
            }
        } else if (!rt_handover_logged) {
            rt_handover_logged = true;
            RWLOG_I("MotorevoActuator: RT direct-send active, control thread skip write()");
        }

        auto end_time = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);

        // 一轮完整结束（write() 已返回），推进 epoch 供 RT 交接握手确认
        loop_epoch_.fetch_add(1, std::memory_order_release);

        if (elapsed < period) {
            std::this_thread::sleep_for(period - elapsed);
        }

        if (!running_) {
            break;
        }
    }

    RWLOG_SUCCESS("MotorevoActuator: Control thread exited");
}

bool MotorevoActuator::enableAll() {
    if (!running_){
        RWLOG_FAILURE("enableAll failed: actuator not running");
        return false;
    }
    RWLOG_I("Enabling all motors...");

    std::vector<bool> result;
    for (size_t i = 0; i < can_bus_groups_.size(); ++i) {
        const auto& group = can_bus_groups_[i];
        bool ret = group.motor_control->enableAll();
        result.push_back(ret);
    }

    // 如果全失败返回false
    return std::all_of(result.begin(), result.end(), [](bool success) { return success; });
}

bool MotorevoActuator::disableAll() {
    if (!running_){
        RWLOG_FAILURE("disableAll failed: actuator not running");
        return false;
    }
    RWLOG_I("Disabling all motors...");

    std::vector<bool> result;
    for (size_t i = 0; i < can_bus_groups_.size(); ++i) {
        const auto& group = can_bus_groups_[i];
        bool ret = group.motor_control->disableAll();
        result.push_back(ret);
    }

    // 如果全失败返回false
    return std::all_of(result.begin(), result.end(), [](bool success) { return success; });
}

void MotorevoActuator::applyInitialTargetPositionsToAllGroups(const std::map<MotorId, float>& target_positions) {
    for (auto& group : can_bus_groups_) {
        if (group.motor_control) {
            group.motor_control->setInitialTargetPositions(target_positions);
        }
    }
}

void MotorevoActuator::setInitialTargetPositions(const std::map<MotorId, float>& target_positions) {
    {
        std::lock_guard<std::mutex> lock(pending_target_positions_mutex_);
        pending_initial_target_positions_ = target_positions;
    }

    // 如果CAN总线组已经初始化，立即设置；否则在init()中设置
    if (!can_bus_groups_.empty()) {
        applyInitialTargetPositionsToAllGroups(target_positions);
        RWLOG_SUCCESS("Initial target positions set for %zu motors across %zu CAN bus groups",
                     target_positions.size(), can_bus_groups_.size());
    } else {
        RWLOG_I("Initial target positions will be applied during init() for %zu motors", target_positions.size());
    }
}

void MotorevoActuator::setZeroOffsetAdjustments(const std::map<size_t, double>& zero_offset_adjustments) {
    std::lock_guard<std::mutex> lock(zero_offset_adjustments_mutex_);
    zero_offset_adjustments_ = zero_offset_adjustments;
    RWLOG_I("Zero offset adjustments set for %zu motors", zero_offset_adjustments.size());
}

void MotorevoActuator::applyZeroOffsetAdjustments() {
    std::lock_guard<std::mutex> lock(zero_offset_adjustments_mutex_);
    
    if (zero_offset_adjustments_.empty()) {
        // RWLOG_I("No zero offset adjustments to apply");
        return;
    }
    
    // RWLOG_I("Applying zero offset adjustments to runtime motor configs (triggered at stand-up)");
    
    // 收集当前所有电机的零点偏移值
    std::vector<float> zero_offsets(motor_refs_.size(), 0.0f);
    for (const auto& group : can_bus_groups_) {
        if (group.motor_control) {
            auto offsets = group.motor_control->getZeroOffsets();
            for (const auto& [id, offset] : offsets) {
                zero_offsets[id2indexs_[id]] = offset;
            }
        }
    }
    
    // 平滑过渡参数
    constexpr int num_steps = 20;  // 分20步平滑过渡
    constexpr float step_duration_ms = 50.0f;  // 每步50ms
    constexpr float total_duration_ms = num_steps * step_duration_ms;
    
    // RWLOG_I("Smoothly applying zero offset adjustments over %.1f seconds (%d steps)", 
    //         total_duration_ms / 1000.0f, num_steps);
    
    // 应用调整并平滑更新运行时零点值
    for (const auto& [index, adjustment] : zero_offset_adjustments_) {
        if (index < motor_refs_.size()) {
            float old_offset = zero_offsets[index];
            float new_offset = old_offset + static_cast<float>(adjustment);
            float adjustment_per_step = static_cast<float>(adjustment) / num_steps;
            
            auto& motor_ref = motor_refs_[index];
            auto& can_group = can_bus_groups_[motor_ref.bus_group_index];
            if (can_group.motor_control) {
                // RWLOG_I("Smoothly adjusting motor %zu (ID: 0x%02X): %.6f -> %.6f (adjustment: %.6f, %.1f seconds)",
                //         index, motor_ref.id, old_offset, new_offset, static_cast<float>(adjustment), 
                //         total_duration_ms / 1000.0f);
                
                // 逐步更新零点偏移值
                float current_offset = old_offset;
                for (int step = 1; step <= num_steps; ++step) {
                    current_offset = old_offset + adjustment_per_step * step;
                    
                    auto config = can_group.motor_control->getMotorConfig(motor_ref.id);
                    config.zero_offset = current_offset;
                    can_group.motor_control->setMotorConfig(motor_ref.id, config);
                    
                    // 等待一段时间让系统适应
                    std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(step_duration_ms)));
                }
                
                // RWLOG_I("Completed smooth adjustment for motor %zu (ID: 0x%02X): final offset = %.6f",
                //         index, motor_ref.id, new_offset);
            }
        }
    }
    
    // RWLOG_SUCCESS("Zero offset adjustments smoothly applied to runtime motor configs");
}

void MotorevoActuator::buildIndexMappings(const std::vector<MotorControlRef>& motor_refs) {
    // 清空现有映射
    motor_refs_.clear();

    // 初始化id2indexs_数组，填充无效值
    for (size_t i = 0; i <= MAX_MOTOR_ID; ++i) {
        id2indexs_[i] = static_cast<size_t>(-1); // 使用-1表示无效索引
    }

    // 按电机ID排序以确保一致性  ??? 也许可以不用排序 ??? 直接根据配置文件定义的顺序
    std::vector<MotorControlRef> sorted_refs = motor_refs;
    std::sort(sorted_refs.begin(), sorted_refs.end(),
              [](const MotorControlRef& a, const MotorControlRef& b) {
                  return a.id < b.id;
              });

    // 构建映射关系
    for (size_t i = 0; i < sorted_refs.size(); ++i) {
        auto& ref = sorted_refs[i];
        id2indexs_[ref.id] = i;
        motor_refs_.push_back(ref);
    }
}

void MotorevoActuator::saveZeroToFile() {
    std::string config_path = getDefaultZeroFilePath();
    std::string backup_path = config_path + ".bak";

    // 收集所有电机的零点偏移值，按索引顺序
    std::vector<float> zero_offsets(motor_refs_.size(), 0.0f);

    for (const auto& group : can_bus_groups_) {
        auto offsets = group.motor_control->getZeroOffsets();
        for (const auto& [id, offset] : offsets) {
            // 根据电机ID找到对应的索引
            zero_offsets[id2indexs_[id]] = offset;
        }
    }

    // 获取零点偏移调整参数
    std::map<size_t, double> adjustments;
    {
        std::lock_guard<std::mutex> lock(zero_offset_adjustments_mutex_);
        adjustments = zero_offset_adjustments_;
    }

    // 按索引顺序保存零点偏移值
    YAML::Node config;
    YAML::Node ym_zero_position;
    for (size_t i = 0; i < zero_offsets.size(); ++i) {
        float offset = zero_offsets[i];

        // 应用传入的偏移调整参数（仅保存到文件，运行时更新延迟到按下'o'开始站立时）
        auto it = adjustments.find(i);
        if (it != adjustments.end()) {
            float old_offset = offset;
            offset += static_cast<float>(it->second);  // 应用调整
            // RWLOG_I("Applying zero offset adjustment to motor %zu: %.6f -> %.6f (adjustment: %.6f) - will apply at stand-up",
            //         i, old_offset, offset, static_cast<float>(it->second));
            // 注意：运行时零点值的更新将延迟到按下'o'开始站立时通过 applyZeroOffsetAdjustments() 执行
        }

        ym_zero_position.push_back(offset);
    }
    config["arms_zero_position"] = ym_zero_position;

    // 备份配置文件
    if (std::filesystem::exists(config_path)) {
        std::filesystem::copy_file(config_path, backup_path, std::filesystem::copy_options::overwrite_existing);
        // RWLOG_I("Backup zero file to %s", backup_path.c_str());
    }

    // 保存配置文件
    std::ofstream file(config_path);
    if (file.is_open()) {
        file << config;
        file.close();
        // RWLOG_SUCCESS("Zero position saved to %s", config_path.c_str());
    } else {
        RWLOG_FAILURE("Failed to open zero file for writing: %s", config_path.c_str());
    }
}

void MotorevoActuator::reset() {
    RWLOG_I("Closing MotorevoActuator...");

    // 停止控制线程
    if (!running_) {
        return;
    }

    // 关键：先失能所有总线上的所有电机（趁控制线程仍在运行、CAN 接收线程仍存活）。
    // 之前依赖 RevoMotorControl 析构时逐个失能，若某条总线 teardown 被中断/异常，
    // 后续总线（如右臂 canbus1）就会被漏掉，导致关闭后手臂仍使能。
    disableAll();

    running_ = false;

    if (control_thread_.joinable()) {
        control_thread_.join();
    }

    RWLOG_SUCCESS("Control thread stopped");

    // 清空CAN总线组，手动释放motor_control
    for (auto& group : can_bus_groups_) {
        if (group.motor_control) {
            try {
                group.motor_control.reset();
            } catch (const std::exception& e) {
                RWLOG_FAILURE("释放 CAN 总线 %s 的 motor_control 时异常: %s", group.name.c_str(), e.what());
            } catch (...) {
                RWLOG_FAILURE("释放 CAN 总线 %s 的 motor_control 时未知异常", group.name.c_str());
            }
            group.motor_control = nullptr;
        }
    }
    can_bus_groups_.clear();
    motor_refs_.clear();
    rt_active_.store(false, std::memory_order_release);  // RT 源已随 motor_control 析构注销，复位接管标志以便重新 init
    for (size_t i = 0; i <= MAX_MOTOR_ID; ++i) {
        id2indexs_[i] = static_cast<size_t>(-1);
    }

    RWLOG_SUCCESS("RuiwoActuator closed successfully");
}

/******************************************************************************/
/*                          基类接口兼容 (MotorActuatorBase)                  */
/******************************************************************************/

int MotorevoActuator::initialize() {
    return init();
}

int MotorevoActuator::enable() {
    if(!enableAll()) {
        return 1;
    }
    return 0;
}

int MotorevoActuator::disable() {
    if(!disableAll()) {
        return 1;
    }
    return 0;
}

void MotorevoActuator::close() {
    reset();
}

bool MotorevoActuator::disableMotor(int motorIndex) {
    // 检查索引是否有效
    if (motorIndex < 0 || static_cast<size_t>(motorIndex) >= motor_refs_.size()) {
        RWLOG_W("disableMotor Invalid motor index: %d (range: 0-%zu)", motorIndex, motor_refs_.size() - 1);
        return false;
    }

    // 直接从motor_refs_获取对应的电机信息
    const auto& motor_ref = motor_refs_[motorIndex];
    if (motor_ref.ignore) {
        RWLOG_W("Motor at index %d is ignored, cannot disable", motorIndex);
        return true;
    }

    // 获取对应的电机ID和CanBusGroup
    motorevo::MotorId motor_id = motor_ref.id;
    const auto& group = can_bus_groups_[motor_ref.bus_group_index];

    RWLOG_I("Disabling motor at index %d (ID: %d) on CAN总线 %s", motorIndex, motor_id, group.name.c_str());

    int result = group.motor_control->disableMotor(motor_id);
    if (result == 0) {
        RWLOG_SUCCESS("Motor %d disabled successfully", motor_id);
        return true;
    } else {
        RWLOG_FAILURE("Motor %d disable failed with ret code: %d", motor_id, result);
    }
    
    return false;
}

std::vector<double> MotorevoActuator::get_positions() {
    return getPositions();
}

std::vector<double> MotorevoActuator::get_torque() {
    return getTorques();
}

std::vector<double> MotorevoActuator::get_velocity() {
    return getVelocities();
}

void MotorevoActuator::saveAsZeroPosition() {
    return saveZeroToFile();
}

void MotorevoActuator::saveZeroPosition() {
    return saveZeroToFile();
}

void MotorevoActuator::set_teach_pendant_mode(int mode) {
    // 设置0-torque模式
    bool enable = (mode == 1);  // 1：示教模式，0：非示教模式

    // 通知所有CAN总线组设置0-torque模式
    for (auto& bus_group : can_bus_groups_) {
        if (bus_group.motor_control) {
            bus_group.motor_control->setZeroTorqueMode(enable);
        }
    }

    RWLOG_I("Teach pendant mode %s (0-torque mode %s)",
            enable ? "enabled" : "disabled",
            enable ? "enabled" : "disabled");
}

void MotorevoActuator::changeEncoderZeroRound(int index, double direction) {
    if (!running_) {
        RWLOG_FAILURE("changeEncoderZeroRound failed: actuator not running");
        return;
    }

    // 检查关节索引是否有效
    if (index < 0 || static_cast<size_t>(index) >= motor_refs_.size()) {
        RWLOG_W("changeEncoderZeroRound Invalid joint index: %d (range: 0-%zu)", index, motor_refs_.size() - 1);
        return;
    }

    // 获取电机引用
    auto& motor_ref = motor_refs_[index];
    auto& can_group = can_bus_groups_[motor_ref.bus_group_index];

    if (!can_group.motor_control) {
        RWLOG_W("Motor control not available for joint %d", index);
        return;
    }

    // 从motor_control获取当前配置
    auto config = can_group.motor_control->getMotorConfig(motor_ref.id);

    // 计算编码器零点修正值
    float round = 360.0f / (config.ratio) * M_PI / 180.0f;

    // 根据方向调整零点偏移
    if (direction > 0) {
        config.zero_offset += round;
    } else {
        config.zero_offset -= round;
    }

    // 更新电机控制器中的配置
    can_group.motor_control->setMotorConfig(motor_ref.id, config);

    RWLOG_I("ChangeEncoderZeroRound joint %d, direction %.1f, ratio %.1f, round %.3f, new offset: %.3f",
            index, direction, config.ratio, round, config.zero_offset);
}

void MotorevoActuator::adjustZeroPosition(int index, double offset) {
    if (!running_) {
        RWLOG_FAILURE("adjustZeroPosition failed: actuator not running");
        return;
    }
    // 检查关节索引是否有效
    if (index < 0 || static_cast<size_t>(index) >= motor_refs_.size()) {
        RWLOG_W("adjustZeroPosition Invalid joint index: %d (range: 0-%zu)", index, motor_refs_.size() - 1);
        return;
    }

    // 获取电机引用
    auto& motor_ref = motor_refs_[index];
    auto& can_group = can_bus_groups_[motor_ref.bus_group_index];

    if (!can_group.motor_control) {
        RWLOG_W("Motor control not available for joint %d", index);
        return;
    }

    // 从motor_control获取当前配置
    auto config = can_group.motor_control->getMotorConfig(motor_ref.id);

    // 调整零点偏移
    config.zero_offset += static_cast<float>(offset);

    // 更新电机控制器中的配置
    can_group.motor_control->setMotorConfig(motor_ref.id, config);

    RWLOG_I("Adjusted zero position for joint %d by %.3f, new offset: %.3f",
            index, offset, config.zero_offset);
}

std::vector<double> MotorevoActuator::getMotorZeroPoints() {
    std::vector<double> zero_points(motor_refs_.size(), 0.0);

    // 从motor_control获取零点偏移值
    for (const auto& group : can_bus_groups_) {
        if (group.motor_control) {
            auto configs = group.motor_control->getZeroOffsets();
            for (const auto& [id, offset] : configs) {
                // 根据电机ID找到对应的索引
                size_t index = id2indexs_[id];
                zero_points[index] = static_cast<double>(offset);
            }
        }
    }

    return zero_points;
}

RuiwoActuatorBase::MotorStateDataVec MotorevoActuator::get_motor_state() {
    MotorStateDataVec motor_states(motor_refs_.size());

    // 初始化，所有电机默认为忽略状态
    for (size_t i = 0; i < motor_refs_.size(); ++i) {
        motor_states[i] = {static_cast<uint8_t>(motor_refs_[i].id), State::Ignored};
    }

    // 从motor_control获取电机状态
    for (const auto& group : can_bus_groups_) {
        auto revo_motor_states = group.motor_control->getMotorStates();
        for (const auto& [id, motor_state] : revo_motor_states) {
            // 根据电机ID找到对应的索引
            size_t index = id2indexs_[id];

            // 将MotorState映射到State
            State state = (motor_state == motorevo::MotorState::MotorSate) ? State::Enabled : State::Disabled;

            // 如果电机被标记为忽略，则保持忽略状态
            if (!motor_refs_[index].ignore) {
                motor_states[index] = {static_cast<uint8_t>(id), state};
            }
        }
    }

    return motor_states;
}

void MotorevoActuator::set_positions(const std::vector<uint8_t> &index,
                                   const std::vector<double> &positions,
                                   const std::vector<double> &torque,
                                   const std::vector<double> &velocity,
                                   const std::vector<double> &kp,
                                   const std::vector<double> &kd) {
    // 检查是否在运行状态
    if (!running_) {
        // printf("Actuator is not running, cannot set positions\n");
        return;
    }

    // 检查参数维度是否一致
    if (index.size() != positions.size() || index.size() != torque.size() ||
        index.size() != velocity.size()) {
        printf("参数维度不一致: index(%zu), positions(%zu), torque(%zu), velocity(%zu)",
                      index.size(), positions.size(), torque.size(), velocity.size());
        return;
    }

    // 创建目标命令数组，初始化为默认值
    std::vector<RevoMotorCmd_t> targets(motor_refs_.size(), {0.0, 0.0, 0.0, 0.0, 0.0});

    // 按索引设置目标值
    for (size_t i = 0; i < index.size(); ++i) {
        uint8_t idx = index[i];

        // 检查索引有效性
        if (idx >= motor_refs_.size()) {
            printf("set_positions Invalid index %u at position %zu, skipping", idx, i);
            continue;
        }

        targets[idx].pos = positions[i] * M_PI / 180.0;  // 角度转弧度
        targets[idx].vel = velocity[i] * M_PI / 180.0;  // 角度/秒 转 弧度/秒
        targets[idx].torque = torque[i];

        // 优先使用从 joint_cmd 实时传入的 kp/kd，否则回退到缓存的 rt_params
        if (!kp.empty() && i < kp.size() && !kd.empty() && i < kd.size()) {
            targets[idx].kp = static_cast<float>(kp[i]);
            targets[idx].kd = static_cast<float>(kd[i]);
        } else {
            const auto& motor_ref = motor_refs_[idx];
            targets[idx].kp = motor_ref.rt_params.kp_pos;
            targets[idx].kd = motor_ref.rt_params.kd_pos;
        }
    }

    // 调用现有接口设置目标
    setTargets(targets);
}

void MotorevoActuator::set_torque(const std::vector<uint8_t> &index,
                                 const std::vector<double> &torque) {
    // 检查是否在运行状态
    if (!running_) {
        printf("Actuator is not running, cannot set torque\n");
        return;
    }

    // 检查输入维度是否一致
    if (index.size() != torque.size()) {
        RWLOG_FAILURE("Parameter size mismatch: index(%zu), torque(%zu)",
                      index.size(), torque.size());
        return;
    }

    // TODO 实现具体的扭矩设置逻辑
    RWLOG_W("set_torque not fully implemented yet");
}

void MotorevoActuator::set_velocity(const std::vector<uint8_t> &index,
                                   const std::vector<double> &velocity) {
    // 检查是否在运行状态
    if (!running_) {
        printf("Actuator is not running, cannot set velocity\n");
        return;
    }

    // 检查输入维度是否一致
    if (index.size() != velocity.size()) {
        RWLOG_FAILURE("Parameter size mismatch: index(%zu), velocity(%zu)",
                      index.size(), velocity.size());
        return;
    }

    // TODO 实现具体的速度设置逻辑
    RWLOG_W("set_velocity not fully implemented yet");
}

void MotorevoActuator::set_joint_gains(const std::vector<int> &joint_indices,
                                     const std::vector<double> &kp_pos,
                                     const std::vector<double> &kd_pos) {
    // 检查参数维度是否一致
    if (kp_pos.size() != joint_indices.size() ||
        kd_pos.size() != joint_indices.size()) {
        RWLOG_WARNING("Parameter size mismatch: joint_indices(%zu), kp_pos(%zu), kd_pos(%zu)",
                      joint_indices.size(), kp_pos.size(), kd_pos.size());
        return;
    }

    // 按关节索引设置增益参数
    for (size_t i = 0; i < joint_indices.size(); ++i) {
        int joint_idx = joint_indices[i];

        // 检查关节索引是否有效
        if (joint_idx < 0 || static_cast<size_t>(joint_idx) >= motor_refs_.size()) {
            RWLOG_W("set_joint_gains Invalid joint index: %d (range: 0-%zu)", joint_idx, motor_refs_.size() - 1);
            continue;
        }

        // 更新增益参数
        auto& motor_ref = motor_refs_[joint_idx];
        motor_ref.rt_params.kp_pos = static_cast<float>(kp_pos[i]);
        motor_ref.rt_params.kd_pos = static_cast<float>(kd_pos[i]);
        RWLOG_I("Set joint %d kp_pos to %.2f", joint_idx, kp_pos[i]);
        RWLOG_I("Set joint %d kd_pos to %.2f", joint_idx, kd_pos[i]);
    }
}

std::vector<std::vector<double>> MotorevoActuator::get_joint_gains(const std::vector<int> &joint_indices) {
    std::vector<double> kp_pos_values;
    std::vector<double> kd_pos_values;

    // 确定要获取的关节索引
    std::vector<int> indices_to_get;
    if (joint_indices.empty()) {
        // 获取所有关节的增益参数
        indices_to_get.resize(motor_refs_.size());
        for (size_t i = 0; i < motor_refs_.size(); ++i) {
            indices_to_get[i] = static_cast<int>(i);
        }
    } else {
        indices_to_get = joint_indices;
    }

    // 获取指定关节的增益参数
    for (int joint_idx : indices_to_get) {
        // 检查关节索引是否有效
        if (joint_idx < 0 || static_cast<size_t>(joint_idx) >= motor_refs_.size()) {
            RWLOG_W("get_joint_gains Invalid joint index: %d (range: 0-%zu)", joint_idx, motor_refs_.size() - 1);
            continue;
        }

        // 获取电机控制引用
        const auto& motor_ref = motor_refs_[joint_idx];
        kp_pos_values.push_back(static_cast<double>(motor_ref.rt_params.kp_pos));
        kd_pos_values.push_back(static_cast<double>(motor_ref.rt_params.kd_pos));
    }

    return {kp_pos_values, kd_pos_values};
}

} // namespace motorevo
