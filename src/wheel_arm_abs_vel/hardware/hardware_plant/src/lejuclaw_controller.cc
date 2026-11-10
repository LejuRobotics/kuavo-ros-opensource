#include <kuavo_common/common/BackgroundThread.h>
#include "lejuclaw_controller.h"
#include <vector>
#include <filesystem>
#include <yaml-cpp/yaml.h>
#include <chrono>
#include <algorithm>
#include <cctype>
#include <iostream>
#include <sstream>
#include <thread>

namespace eef_controller {

static bool CreateDirectoriesWithPermissions(const std::filesystem::path &dir_path) {
    try {
        if(std::filesystem::create_directories(dir_path)) {
            auto perms_rwx_xr_r = 
                std::filesystem::perms::owner_all  | 
                std::filesystem::perms::group_exec | std::filesystem::perms::group_read |
                std::filesystem::perms::others_read;
            
            // 给其他用户组和其他人添加目录的可执行和读权限
            std::filesystem::permissions(dir_path, perms_rwx_xr_r, std::filesystem::perm_options::add);
            
            return true;
        }
    } catch (const std::filesystem::filesystem_error& ex) {
        std::cerr << "[LejuClawController] CreateDirectoriesWithPermissions exception:" << ex.what() << '\n';
    }

    return false;   
}

static bool CopyFileWithPermissions(const std::filesystem::path &src_path, 
                        const std::filesystem::path &dst_path, 
                        std::filesystem::copy_options opt = std::filesystem::copy_options::skip_existing) 
{
    try {
        if(std::filesystem::copy_file(src_path, dst_path, opt)) {
            auto perms_rw_rw_rw = 
                std::filesystem::perms::owner_write | std::filesystem::perms::owner_read |
                std::filesystem::perms::group_write | std::filesystem::perms::group_read |
                std::filesystem::perms::others_write | std::filesystem::perms::others_read;
            
            // 给其他用户组和其他人添加读写权限
            std::filesystem::permissions(dst_path, perms_rw_rw_rw, std::filesystem::perm_options::add);

            return true;
        }
    } catch (const std::filesystem::filesystem_error& ex) {
        std::cerr << "[LejuClawController] CreateDirectoriesWithPermissions exception:" << ex.what() << '\n';
    }
    return false;  
}

std::string GetHomePath()
{
    const char *sudo_user = getenv("SUDO_USER");
    if (sudo_user != nullptr) {
        passwd *pw = getpwnam(sudo_user);
        if (pw != nullptr){
            std::string path = pw->pw_dir;
            std::cout << "sudo user home path: " << path << std::endl;
            return path;
        }
    }
    else
    {
        uid_t uid = getuid();
        passwd *pw = getpwuid(uid);
        if (pw != nullptr){
            std::string path = pw->pw_dir;
            return path;
        }
    }
    return "/home/lab";
}

bool CopyOrUpdateLejuClawConfigFile(const std::string &src_file)
{   
    auto config_file_path = std::filesystem::path(GetHomePath()) / ".config/lejuconfig/config.yaml";
    if(!std::filesystem::exists(config_file_path)) {
        std::cout << "[LejuClawController] CopyOrUpdateLejuClawConfigFile config.yaml not exist:" << config_file_path.string() << std::endl;
        /* config.yaml not exist, copy src_file to ~/.config/lejuconfig/config.yaml  */
        if(CreateDirectoriesWithPermissions(config_file_path.parent_path())) {
            return CopyFileWithPermissions(src_file, config_file_path);
        }
        else {
            std::cerr << "[LejuClawController] CopyOrUpdateLejuClawConfigFile CreateDirectoriesWithPermissions failed" << std::endl;
            return false;
        }
    }

    /* config.yaml exist, then check params, */
    bool need_updated = false;
    try {
        std::cout << " [LejuClawController]  old config path: " << config_file_path << std::endl;
        std::cout << " [LejuClawController]  new config path: " << src_file << std::endl;

        YAML::Node old_config = YAML::LoadFile(config_file_path.string());
        YAML::Node new_config = YAML::LoadFile(src_file);

        std::vector<std::string> item_names{"address", "online", "parameter"};
        std::vector<std::string> claw_names{"Claw_joint_left", "Claw_joint_right"};

        for(auto &item_name : item_names) {
            if(!old_config[item_name]) {
                std::cerr << "[LejuClawController] CopyOrUpdateLejuClawConfigFile config.yaml `" << item_name << "` not exist, updated it." << std::endl;
                old_config[item_name] = new_config[item_name];
                need_updated = true;
                continue;
            }

            for(auto &claw_name : claw_names) {
                if(!old_config[item_name][claw_name]) {
                    need_updated = true;
                    old_config[item_name][claw_name] = new_config[item_name][claw_name];
                    std::cout << "[LejuClawController] CopyOrUpdateLejuClawConfigFile config.yaml `" << claw_name << "` not exist, updated it." << std::endl;
                }
            }
        }

        /* checkouut ratio item */
        if (!old_config["ratio"]) {
            old_config["ratio"] = new_config["ratio"];
            need_updated = true;
            std::cout << "[LejuClawController] CopyOrUpdateLejuClawConfigFile config.yaml `ratio` not exist, updated it." << std::endl;
        }

        /* need to update config.yaml */
        if (need_updated) {
            /*  backup the config.yaml to config.yaml.for_lejuclaw.bak and update the config.yaml */
            auto bak_file_path = config_file_path.parent_path() / "config.yaml.for_lejuclaw.bak";
            std::cout << "[LejuClawController] CopyOrUpdateLejuClawConfigFile backup config.yaml to " << bak_file_path.string() << std::endl;
            if(!CopyFileWithPermissions(config_file_path, bak_file_path)) {
                std::cerr << "[LejuClawController] CopyOrUpdateLejuClawConfigFile backup config.yaml failed \n";
            }
            
            /* update the config.yaml */ 
            std::ofstream fout(config_file_path);
            fout << old_config;
        }
        return true;
    }
    catch (const YAML::Exception &e) {
        std::cout << "[LejuClawController] YAML Exception: " << e.what() << std::endl;
        return false;
    }
    catch (const std::exception& e) {
        std::cout <<  "[LejuClawController] Error: " <<  e.what()  << std::endl;
        return false;
    }
    
    return false;
}

ClawState GetClawState(const LejuClawControllerPtr &controller_ptr)
{
    ClawState state;
    if(controller_ptr) {
        auto pos = controller_ptr->get_positions();
        auto tor = controller_ptr->get_torque();
        auto vel = controller_ptr->get_velocity();
        auto claw_state = controller_ptr->get_state();

        state.data.name = {kLeftGripperName, kRightGripperName};
        state.data.position = pos.empty() ? std::vector<double>{0.0, 0.0} : pos;
        state.data.velocity = vel.empty() ? std::vector<double>{0.0, 0.0} : vel;
        state.data.effort = tor.empty() ? std::vector<double>{0.0, 0.0} : tor;
        state.state = {static_cast<int8_t>(claw_state[0]), static_cast<int8_t>(claw_state[1])};
        state.is_high_freq = controller_ptr->get_is_high_freq();
    }
    return state;
}

bool LejuClawController::controlGripper(ControlClawRequest &req, ControlClawResponse &res, bool is_high_freq)
{
    res.success = false;

    if (this->execute(req.data, is_high_freq, res.message)) {
        res.success = true;
    }
    return res.success;
}

void  LejuClawController::command(const lejuClawCommand &msg, bool is_high_freq)
{
    // 高频/低频标志由 hardware_node（ROS 边界）根据 /lejuclaw_high_freq_switch 与
    // /quest_joystick_data 数据流激活计算后逐命令透传，控制器不再做时间间隔检测。
    std::string err_msg;
    this->execute(msg, is_high_freq, err_msg, true);
}

void LejuClawController::setDebugCallback(LejuClawDebugCallback callback)
{
    debug_callback_ = std::move(callback);
    if (claw_can_ptr_ != nullptr) {
        claw_can_ptr_->set_debug_callback(debug_callback_);
    }
}

void LejuClawController::setTargetCallback(LejuClawTargetCallback callback)
{
    {
        std::lock_guard<std::mutex> lock(target_callback_mutex_);
        target_callback_ = std::move(callback);
    }
    installTargetCallback();
}

void LejuClawController::installTargetCallback()
{
    LejuClawTargetCallback callback;
    {
        std::lock_guard<std::mutex> lock(target_callback_mutex_);
        callback = target_callback_;
    }
    if (callback == nullptr) {
        return;
    }
    if (is_can_protocol_) {
        if (claw_can_ptr_ != nullptr) {
            // CAN 逐电机回调：升序 MotorId ↔ {left_claw, right_claw}（与 GetClawState 命名一致）
            auto sorted_ids = claw_can_ptr_->get_sorted_motor_ids();
            claw_can_ptr_->set_tx_callback(
                [callback, sorted_ids](uint8_t id, float pos_percent, float vel, float torque,
                                   float kp, float kd, bool zero_kp) {
                    std::vector<std::string> name;
                    std::vector<double> position, velocity, effort;
                    std::vector<int> kps, kds;
                    // 定位当前电机在升序列表中的槽位，取不到时按右爪兜底
                    size_t slot = sorted_ids.size() > 0 ? sorted_ids.size() - 1 : 0;
                    for (size_t i = 0; i < sorted_ids.size(); ++i) {
                        if (static_cast<uint8_t>(sorted_ids[i]) == id) { slot = i; break; }
                    }
                    name.push_back(slot == 0 ? kLeftGripperName : kRightGripperName);
                    position.push_back(pos_percent);
                    velocity.push_back(vel);
                    effort.push_back(torque);
                    kps.push_back(static_cast<int>(kp));
                    kds.push_back(static_cast<int>(kd));
                    callback(name, position, velocity, effort, kps, kds, zero_kp);
                });
        }
    }
    else {
        if (claw_ptr_ != nullptr) {
            // 串口一次回全量数组：index 0/1 ↔ {left_claw, right_claw}
            claw_ptr_->set_tx_callback(
                [callback](const std::vector<int>& index, const std::vector<float>& pos,
                       const std::vector<float>& velocity, const std::vector<float>& torque,
                       const std::vector<int>& kp, const std::vector<int>& kd, bool zero_kp) {
                    std::vector<std::string> name;
                    std::vector<double> position(pos.begin(), pos.end());
                    std::vector<double> vel(velocity.begin(), velocity.end());
                    std::vector<double> effort(torque.begin(), torque.end());
                    for (size_t i = 0; i < index.size(); ++i) {
                        name.push_back(index[i] == 0 ? kLeftGripperName : kRightGripperName);
                    }
                    // 安全 PTM 时 kp 数组里是原始增益，消费侧按 zero_kp 标志置 0
                    callback(name, position, vel, effort, kp, kd, zero_kp);
                });
        }
    }
}

bool LejuClawController::recover(const std::string& direction, float current, int duration_ms, int repeat, std::string& err_msg)
{
    constexpr int kMaxRecoveryRepeat = 3;

    if (!is_can_protocol_ || claw_can_ptr_ == nullptr) {
        err_msg = "leju claw recovery is only supported by CAN claw";
        return false;
    }

    std::string mode = direction;
    std::transform(mode.begin(), mode.end(), mode.begin(), [](unsigned char ch) {
        return static_cast<char>(std::tolower(ch));
    });
    std::replace(mode.begin(), mode.end(), '-', '_');

    bool stop_only = mode == "stop";
    lejuclaw_can::LeJuClawCan::RecoveryDirection recovery_direction =
        lejuclaw_can::LeJuClawCan::RecoveryDirection::TOWARD_OPEN;

    if (mode == "toward_open" || mode == "open") {
        recovery_direction = lejuclaw_can::LeJuClawCan::RecoveryDirection::TOWARD_OPEN;
    } else if (mode == "toward_close" || mode == "close") {
        recovery_direction = lejuclaw_can::LeJuClawCan::RecoveryDirection::TOWARD_CLOSE;
    } else if (mode == "stuck_open") {
        recovery_direction = lejuclaw_can::LeJuClawCan::RecoveryDirection::TOWARD_CLOSE;
    } else if (mode == "stuck_close") {
        recovery_direction = lejuclaw_can::LeJuClawCan::RecoveryDirection::TOWARD_OPEN;
    } else if (!stop_only) {
        err_msg = "invalid direction, use toward_open/toward_close/stuck_open/stuck_close/stop";
        return false;
    }

    if (recovery_in_progress_.exchange(true)) {
        err_msg = "leju claw recovery is already in progress";
        return false;
    }
    struct RecoveryGuard {
        std::atomic_bool& flag;
        ~RecoveryGuard() { flag.store(false); }
    } guard{recovery_in_progress_};

    {
        std::lock_guard<std::mutex> lock(task_mutex_);
        // 恢复与命令接受共用 task_mutex_ 串行化：恢复一旦取得接管权，后续命令不能重新武装看门狗。
        disarmStreamWatchdog();
        current_task_.reset();
    }

    claw_can_ptr_->cancel_motion();
    auto cancel_start = std::chrono::steady_clock::now();
    while (true) {
        {
            std::lock_guard<std::mutex> lock(task_mutex_);
            if (!worker_active_ && !claw_is_executing_) break;
        }
        auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - cancel_start).count();
        if (elapsed_ms > 1000) {
            claw_can_ptr_->stop_all_current(10, 10);
            err_msg = "current claw motion did not stop within 1000ms";
            return false;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    claw_can_ptr_->stop_all_current(10, 10);
    if (stop_only) {
        err_msg = "zero current sent";
        return true;
    }

    int safe_repeat = std::max(1, std::min(kMaxRecoveryRepeat, repeat));
    for (int i = 0; i < safe_repeat; ++i) {
        if (!claw_can_ptr_->apply_recovery_current(recovery_direction, current, duration_ms)) {
            std::ostringstream oss;
            oss << "recovery pulse failed at " << (i + 1) << "/" << safe_repeat;
            err_msg = oss.str();
            claw_can_ptr_->stop_all_current(10, 10);
            return false;
        }
        claw_can_ptr_->stop_all_current(5, 10);
        if (i + 1 < safe_repeat) {
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
        }
    }

    if (repeat > kMaxRecoveryRepeat) {
        std::ostringstream oss;
        oss << "recovery pulse complete, repeat clamped to " << kMaxRecoveryRepeat;
        err_msg = oss.str();
    } else {
        err_msg = "recovery pulse complete";
    }
    return true;
}

bool LejuClawController::execute(const lejuClawCommand &data, bool is_high_freq, std::string &err_msg,
                                 bool from_topic)
{
    if (recovery_in_progress_.load()) {
        err_msg = "claw recovery is in progress";
        return false;
    }
    
    // Check params
    if (data.position.empty()||data.name.empty() || data.position.size() != data.name.size()) {
        err_msg = "Invaild Data: position.size() and name.size() must be equal!";
        std::cout << err_msg << std::endl;
        return false;
    }

    // Check name
    int right_claw_idx = -1, left_claw_idx = -1;
    for (size_t i = 0; i < data.name.size(); i++) {
        if (data.name[i] != kRightGripperName && data.name[i] != kLeftGripperName) {
            err_msg = "Invaild Data: name must be `left_claw` or `right_claw`!";
            return false;
        }
        else {
            if (data.name[i] == kRightGripperName) {
                right_claw_idx = i;
            }
            else {
                left_claw_idx = i;
            }
        }
    }

    const double kDefaultTorque = 1.0;    // 1.0A
    const double kDefaultVelocity = 50;   // 50%
    std::vector<double> pos{0.0, 0.0}, vel{kDefaultVelocity, kDefaultVelocity}, tor{kDefaultTorque, kDefaultTorque};

    // 合成完整目标、检查任务所有权和替换 pending 槽使用同一把锁，避免并发命令互相覆盖。
    std::unique_lock<std::mutex> lock(task_mutex_);
    {
        if (stop_worker_ || recovery_in_progress_.load()) {
            err_msg = "claw is stopping or recovering";
            return false;
        }
        if (!command_positions_initialized_) {
            err_msg = "Claw target positions are not initialized.";
            return false;
        }
        pos[0] = last_command_positions_[0];
        pos[1] = last_command_positions_[1];
    }

    for(int i = 0; i<2; i++) {
        if(i == right_claw_idx) {
            pos[1] = data.position[i];
            if(data.velocity.size() > i) {
                vel[1] = data.velocity[i] == 0.0 ? kDefaultVelocity : data.velocity[i]; // if velocity is 0, use default velocity
            }
            if (data.effort.size() > i) {
                tor[1] = data.effort[i] == 0.0 ? kDefaultTorque : data.effort[i];
            }
        }
        else if(i == left_claw_idx){
            pos[0] = data.position[i];
            if(data.velocity.size() > i) {
                vel[0] = data.velocity[i] == 0.0 ? kDefaultVelocity : data.velocity[i]; // if velocity is 0, use default velocity
            }
            if (data.effort.size() > i) {
                tor[0] = data.effort[i] == 0.0 ? kDefaultTorque : data.effort[i];
            }
        }
    }

    const bool busy = worker_active_ || current_task_.has_value();
    // 仅 CAN、低频、Topic 来源且当前任务也是同模式 Topic 时，允许替换 latest target。
    // Service、串口和跨模式命令继续沿用原有拒绝规则。
    const bool can_replace_topic =
        from_topic && is_can_protocol_ && !is_high_freq &&
        (!worker_active_ || (active_from_topic_ && !active_high_freq_)) &&
        (!current_task_.has_value() ||
         (current_task_->from_topic && !current_task_->is_high_freq));
    if (busy && !is_high_freq && !can_replace_topic) {
        err_msg = " claw is executing, please send again later.";
        return false;
    }

    // 高频模式下允许覆盖正在执行的任务（新任务入队，worker 在当前 move_paw 结束后执行）
    // qa: https://www.lejuhub.com/highlydynamic/kuavodevlab/-/issues/3505

    // add task
    if (is_high_freq && !armStreamWatchdog(data.received_at)) {
        err_msg = "claw stream command expired before dispatch";
        return false;
    }
    current_task_ = MotionTask{pos, vel, tor, is_high_freq, from_topic};

    // 只有请求成功入队后才更新缓存，保证“上一次目标”确实是上一次被接受的目标。
    last_command_positions_[0] = pos[0];
    last_command_positions_[1] = pos[1];

    // Preserve admission/recovery serialization and the receipt-time lease.
    if (!is_high_freq) {
        disarmStreamWatchdog();
    }
    current_is_high_freq_ = is_high_freq;
    lock.unlock();

    // 命令已经入队并记录模式，先唤醒 worker，调试上报不阻塞运动启动。
    condition_.notify_all();

    if (debug_callback_) {
        std::ostringstream oss;
        oss << "{\"source\":\"controller\",\"event\":\"command_mapped\","
            << "\"is_high_freq\":" << (is_high_freq ? "true" : "false") << ","
            << "\"position\":[" << pos[0] << "," << pos[1] << "],"
            << "\"velocity\":[" << vel[0] << "," << vel[1] << "],"
            << "\"effort\":[" << tor[0] << "," << tor[1] << "]}";
        debug_callback_(oss.str());
    }

    return true;
}


std::array<LejuClawController::State, 2> LejuClawController::get_state() const {
    std::array<State, 2> state;
    {
        std::lock_guard<std::mutex> lock(gripper_state_mutex_);
        state = gripper_state_;
    }
    if (is_can_protocol_ && claw_can_ptr_) {
        const auto ids = claw_can_ptr_->get_sorted_motor_ids();
        const auto health = claw_can_ptr_->get_feedback_health();
        for (size_t i = 0; i < state.size(); ++i) {
            if (i >= ids.size() || !health.count(ids[i]) || !health.at(ids[i]).fresh)
                state[i] = State::kError;
        }
    }
    return state;
}

void LejuClawController::updateState(std::array<State, 2> new_state) {
    std::lock_guard<std::mutex> lock(gripper_state_mutex_);
    gripper_state_ = new_state;
}

LejuClawController::~LejuClawController(){
    this->close();
}

bool LejuClawController::armStreamWatchdog(std::chrono::steady_clock::time_point received)
{
    if (received == std::chrono::steady_clock::time_point{}) received = std::chrono::steady_clock::now();
    if (is_can_protocol_) return claw_can_ptr_ && claw_can_ptr_->refresh_stream_watchdog(received) != 0;
    return claw_ptr_ && claw_ptr_->refresh_stream_watchdog(received) != 0;
}

void LejuClawController::disarmStreamWatchdog()
{
    if (is_can_protocol_) {
        if (claw_can_ptr_ != nullptr) {
            claw_can_ptr_->disarm_stream_watchdog();
        }
    }
    else {
        if (claw_ptr_ != nullptr) {
            claw_ptr_->disarm_stream_watchdog();
        }
    }
}

void LejuClawController::workerThread()
{
    kuavo_thread::background("claw_io");
    while (true) {
        MotionTask task;
        {
            std::unique_lock<std::mutex> lock(task_mutex_);
            condition_.wait(lock, [this] { return current_task_.has_value() || stop_worker_; });
            if (stop_worker_) return;

            task = std::move(*current_task_);
            current_task_.reset();
            worker_active_ = true;
            active_from_topic_ = task.from_topic;
            active_high_freq_ = task.is_high_freq;
        }

        move_paw(task.positions, task.velocity, task.torque,
                 task.is_high_freq, task.from_topic && is_can_protocol_ && !task.is_high_freq);

        {
            std::lock_guard<std::mutex> lock(task_mutex_);
            worker_active_ = false;
            // 安全退出时不执行残留目标；普通动作结束后的新目标仍由下一轮 worker 领取。
            if (stop_worker_ || recovery_in_progress_.load()) {
                current_task_.reset();
            }
        }
        condition_.notify_all();
    }
}

bool LejuClawController::initialize(bool init_bmapilib)
{
    kuavo_thread::BackgroundScope sdk_inheritance;
    claw_is_executing_ = false;

    int ret = 1;
    if (is_can_protocol_) {
        claw_can_ptr_ = new lejuclaw_can::LeJuClawCan();
        if (debug_callback_) {
            claw_can_ptr_->set_debug_callback(debug_callback_);
        }
        ret = claw_can_ptr_->initialize();
    }
    else {
        claw_ptr_ = new LeJuClaw();
        ret = claw_ptr_->initialize(init_bmapilib);
    }
    if (ret != 0) {
        std::cout << "[LejuClawController]: claw init failed, retcode:" << ret << std::endl;
        return false;
    }

    // 第一个服务请求到来前，以当前反馈位置初始化“最近一次目标”缓存。
    const std::vector<double> current_positions = get_positions();
    if (current_positions.size() >= 2) {
        last_command_positions_[0] = current_positions[0];
        last_command_positions_[1] = current_positions[1];
        command_positions_initialized_ = true;
    }

    // 驱动已创建：补装真实下发指令回调（setTargetCallback 可能早于 initialize 被调用）
    installTargetCallback();

    // 初始化完成：解除断流看门狗武装（等高频命令到达时再武装）
    disarmStreamWatchdog();

    worker_ = std::thread(&LejuClawController::workerThread, this);

    return true;
}

void LejuClawController::close()
{
    {
        std::unique_lock<std::mutex> lock(task_mutex_);
        stop_worker_ = true;
        current_task_.reset();
    }
    condition_.notify_all();
    if (worker_.joinable()) {
        worker_.join();
    }

    // 关闭前解除断流看门狗武装
    disarmStreamWatchdog();

    if (is_can_protocol_) {
        if (claw_can_ptr_ != nullptr) {
            std::cout << "[LejuClawController]: claw close" << std::endl;
            claw_can_ptr_->close();
            delete claw_can_ptr_;
            claw_can_ptr_ = nullptr;
        }
    }
    else {
        if (claw_ptr_ != nullptr) {
            std::cout << "[LejuClawController]: claw close" << std::endl;
            claw_ptr_->close();
            delete claw_ptr_;
            claw_ptr_ = nullptr;
        }
    }
}

std::vector<double> LejuClawController::get_positions()
{
    if (is_can_protocol_) {
        return claw_can_ptr_->get_positions();
    }
    else {
        return claw_ptr_->get_positions();
    }
}

std::vector<double> LejuClawController::get_torque()
{
    if (is_can_protocol_) {
        return claw_can_ptr_->get_torque();
    }
    else {
        return claw_ptr_->get_torque();
    }
}

std::vector<double> LejuClawController::get_velocity()
{
    if (is_can_protocol_) {
        return claw_can_ptr_->get_velocity();
    }
    else {
        return claw_ptr_->get_velocity();
    }
}

std::array<LejuClawController::State, 2> LejuClawController::move_paw(const std::vector<double> &positions,
const std::vector<double> &velocity,
const std::vector<double> &torque, bool is_high_freq, bool follow_topic)
{
    // update moving state
    this->updateState({State::kMoving, State::kMoving});
    
    // mark claw is executing
    claw_is_executing_.store(true);


    // 软限位：由于夹爪存在 bug，目标位置范围限制为 5.0 ~ 95.0，防止撞到硬限位
    // qa: https://www.lejuhub.com/zhanglongbo/leju_claw_driver/-/issues/1
    auto temp_pos = positions;
    temp_pos[0] = std::min(95.0, std::max(temp_pos[0], 5.0));
    temp_pos[1] = std::min(95.0, std::max(temp_pos[1], 5.0));
    // 
    // std::cout << "[LejuClawController] 调用底层move_paw，高频模式: " << (is_high_freq ? "是" : "否") << std::endl;
    LeJuClaw::PawMoveState ret_state;
    if (is_can_protocol_) {
        lejuclaw_can::LeJuClawCan::TargetUpdate update;
        if (follow_topic) {
            update = [this, is_high_freq](std::vector<double>& next_positions,
                                          std::vector<double>& next_velocity,
                                          std::vector<double>& next_torque) {
                std::lock_guard<std::mutex> lock(task_mutex_);
                if (recovery_in_progress_.load() || !current_task_.has_value() ||
                    !current_task_->from_topic || current_task_->is_high_freq != is_high_freq) {
                    return false;
                }
                next_positions = current_task_->positions;
                next_velocity = current_task_->velocity;
                next_torque = current_task_->torque;
                current_task_.reset();
                return true;
            };
        }
        auto can_ret = claw_can_ptr_->move_paw(temp_pos, velocity, torque, is_high_freq, update);
        ret_state = static_cast<LeJuClaw::PawMoveState>(static_cast<int8_t>(can_ret));
    }
    else {
        ret_state = claw_ptr_->move_paw(temp_pos, velocity, torque, is_high_freq);
    }

    std::array<State, 2> new_state{State::kUnknown, State::kUnknown};

    static const std::map<LeJuClaw::PawMoveState, std::array<State, 2>> kStateMap = {
        {LeJuClaw::PawMoveState::ERROR, {State::kError, State::kError}},
        {LeJuClaw::PawMoveState::LEFT_GRABBED_RIGHT_GRABBED, {State::kGrabbed, State::kGrabbed}},
        {LeJuClaw::PawMoveState::LEFT_GRABBED_RIGHT_REACHED, {State::kGrabbed, State::kReached}},
        {LeJuClaw::PawMoveState::LEFT_REACHED_RIGHT_GRABBED, {State::kReached, State::kGrabbed}},
        {LeJuClaw::PawMoveState::LEFT_REACHED_RIGHT_REACHED, {State::kReached, State::kReached}}
        };

    if (kStateMap.find(ret_state) != kStateMap.end()) {
        new_state = kStateMap.at(ret_state);
    }

    // 断流看门狗中止：高频控制流断流导致 move_paw 返回 STREAM_EXPIRED。
    // 不覆盖 ROS 状态（保持 kMoving，不伪装 Reached/Grabbed，也不落到 kUnknown），
    // 仅清执行标记；新高频命令到达后自动恢复并正常更新状态。
    if (ret_state != LeJuClaw::PawMoveState::STREAM_EXPIRED) {
        // update state
        this->updateState(new_state);
    }
    // mark claw is executing
    claw_is_executing_.store(false);

    return new_state;
}

} // namespace eef_controller    
