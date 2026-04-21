#ifndef LEJUCLAW_CAN_CUSTOMED_H
#define LEJUCLAW_CAN_CUSTOMED_H

#include <vector>
#include <memory>
#include <string>
#include <thread>
#include <atomic>
#include <unordered_map>
#include <algorithm>
#include <chrono>
#include <iostream>
#include <iterator>
#include <cmath>
#include <cstring>
#include "motorevo/motor_ctrl.h"
#include "motorevo/motor_log.h"
#include "canbus_sdk/canbus_sdk.h"
#include "canbus_sdk/config_parser.h"
// 设备名宏（左右夹爪）
#define RLEJUCLAW_NAME "Rlejuclaw"
#define LLEJUCLAW_NAME "Llejuclaw"

namespace lejuclaw_can {

using motorevo::MotorId;
using motorevo::RevoMotor;
using motorevo::RevoMotorCmd_t;
using motorevo::RevoMotorConfig_t;

struct DeviceHandler; // 前置声明，避免额外包含


class LeJuClawCan {
public:
    enum class PawMoveState  : int8_t {
    ERROR = -1,                       // 出现错误
    LEFT_REACHED_RIGHT_REACHED = 0,   // 所有夹爪到位
    LEFT_REACHED_RIGHT_GRABBED = 1,   // 左夹爪到位，右夹爪抓取到物品
    LEFT_GRABBED_RIGHT_REACHED = 2,   // 右夹爪到位，左夹爪抓取到物品
    LEFT_GRABBED_RIGHT_GRABBED = 3,   // 所有夹爪均夹取
    };

    // 将内部操作枚举设为public，便于类外部定义中使用
    enum class Operation {
        IDLE,
        ENABLE,
        DISABLE,
        MULTI_TURN_ZERO
    };
    enum class OperationStatus {
        PENDING,
        SUCCESS,
        TIMEOUT,
        FAILED
    };

    enum class State {
    None,
    Enabled,
    Disabled
    };

    struct MotorStateData {
    uint8_t id;
    State   state;
    MotorStateData():id(0x0), state(State::None) {}
    MotorStateData(uint8_t id_, State state_):id(id_), state(state_) {}
    };    
    using MotorStateDataVec = std::vector<MotorStateData>;

    LeJuClawCan();
    ~LeJuClawCan();
    int initialize();
    void close();

    bool enableMotor(MotorId id, int timeout_ms = 100);
    bool enableAll();
    bool disableMotor(MotorId id, int timeout_ms = 100);
    bool disableAll();

    PawMoveState move_paw(const std::vector<double> &positions, const std::vector<double> &velocity, const std::vector<double> &torque);
    PawMoveState move_paw(const std::vector<double> &positions, const std::vector<double> &velocity, const std::vector<double> &torque, bool is_vr_mode);
    // 基于 unordered_map 的控制接口（百分比位置 → 物理映射并下发），键为 MotorId
    PawMoveState move_paw(const std::unordered_map<MotorId, double>& positions_percent,
                          const std::unordered_map<MotorId, double>& velocity,
                          const std::unordered_map<MotorId, double>& torque,
                          bool is_vr_mode);
    void set_joint_state(const std::unordered_map<MotorId, motorevo::RevoMotorCmd_t>& cmd);
    std::unordered_map<MotorId, motorevo::RevoMotorCmd_t> get_joint_state();
    std::vector<double> get_positions();
    std::vector<double> get_torque();
    std::vector<double> get_velocity();

private:
    bool init_lejuclaw_can_customed();
    bool Connect(const canbus_sdk::DeviceConfig& config);

    static void internalMessageCallback(canbus_sdk::CanMessageFrame* frame, const canbus_sdk::CallbackContext* context);
    static void internalTefEventCallback(canbus_sdk::CanMessageFrame* frame, const canbus_sdk::CallbackContext* context);

    // 控制参数常量（完全对齐串口版口径）
    static constexpr float DEFAULT_KP = 18.00f;                    // 比例增益系数
    static constexpr float DEFAULT_KD = 3.20f;                     // 微分增益系数
    static constexpr float DEFAULT_ALPHA = 0.20f;                  // 低通滤波器系数
    static constexpr float DEFAULT_MAX_CURRENT = 2.50f;            // 最大电流限制 A
    static constexpr float DEFAULT_MIN_ERROR = 0.05f;              // 到位误差阈值 rad
    static constexpr float DEFAULT_DT = 0.002f;                    // 控制周期 s

    // 卡死检测参数
    static constexpr float STUCK_DETECTION_DELAY_MS = 50.0f;       // 卡死检测延迟 ms
    static constexpr float STUCK_DETECTION_TIME_MS = 50.0f;        // 卡死检测判定时间 ms
    static constexpr float STUCK_POSITION_THRESHOLD = 0.002f;      // 卡死位置阈值 rad

    // 冲击参数（通用，非零点测程用）
    static constexpr float IMPACT_CURRENT = 3.0f;                  // 冲击电流 A
    static constexpr float IMPACT_DURATION_MS = 200.0f;            // 冲击持续 ms
    static constexpr float IMPACT_INTERVAL_MS = 800.0f;            // 冲击间隔 ms
    static constexpr float LIMIT_RANGE_PERCENT = 5.0f;             // 限位范围 %

    // 稳定检测（非 VR 模式）
    static constexpr float STABLE_POSITION_THRESHOLD = 0.005f;     // 稳定位置阈值 rad
    static constexpr float STABLE_VELOCITY_THRESHOLD = 0.1f;       // 稳定速度阈值 rad/s
    static constexpr float STABLE_DETECTION_TIME_MS = 50.0f;       // 稳定检测时间 ms

    // VR 控制检测参数
    static constexpr int VR_CONTROL_TIMEOUT_MS = 60;               // VR 控制超时 ms
    static constexpr int VR_STUCK_DETECTION_CYCLES = 50;           // VR 卡死检测周期数
    static constexpr float VR_STUCK_POSITION_THRESHOLD = 0.01f;    // VR 卡死位置阈值 rad
    static constexpr float VR_TARGET_POSITION_THRESHOLD = 0.5f;    // VR 目标差阈值 rad

    // 刹车参数
    static constexpr float BRAKE_RANGE_PERCENT = 35.0f;            // 两端刹车范围 %
    static constexpr float BRAKE_MIN_SPEED_FACTOR = 0.01f;         // 最小速度系数
    static constexpr float BRAKE_CURVE_EXPONENT = 40.0f;           // 刹车曲线指数
    static constexpr float TARGET_PROXIMITY_BRAKE_PERCENT = 10.0f; // 目标附近刹车范围 %
    static constexpr float TARGET_PROXIMITY_MIN_FACTOR = 0.05f;    // 目标附近最小速度系数

    // 统计高电流冲击使用次数（调试/记录）
    int three_amp_current_usage_count = 0;

    // 量程测量与映射
    // 初始化寻找零点参数
    static constexpr float ZERO_CONTROL_KP = 0.0f;                  // 零点控制比例增益，零点寻找时使用
    static constexpr float ZERO_CONTROL_KD = 1.0f;                  // 零点控制微分增益，零点寻找时使用
    static constexpr float ZERO_CONTROL_ALPHA = 1.50f;              // 零点控制低通滤波系数
    static constexpr float ZERO_CONTROL_MAX_CURRENT = 1.80f;        // 零点控制最大电流限制，单位 A，仅作最大电流限制保护使用
    static constexpr float STALL_CURRENT_THRESHOLD = 1.50f;         // 堵转电流阈值，超过阈值后认为到达限位，单位 A
    static constexpr float STALL_VELOCITY_THRESHOLD = 0.10f;        // 堵转速度阈值，小于阈值后认为到达限位，单位 rad/s
    static constexpr float ZERO_CONTROL_DT = 0.001f;                // 零点控制周期，单位 s
    static constexpr int ZERO_FIND_TIMEOUT_MS = 20000;              // 零点寻找超时时间，单位 ms
    static constexpr int ZERO_WAIT_MS = 500;                        // 零点等待时间，单位 ms
    static constexpr float OPEN_LIMIT_ADJUSTMENT = -10.0f;          // 开限位调整值，百分比，单位 %（正数往行程外扩展，负数往行程内收缩）
    static constexpr float CLOSE_LIMIT_ADJUSTMENT = 0.0f;           // 关限位调整值，百分比，单位 %（正数往行程外扩展，负数往行程内收缩）
    static constexpr float TARGET_VELOCITY = 65.0f;                 // 限位寻找目标速度，单位 rad/s
    // 关爪限位寻找参数
    static constexpr float OPEN_POSITION_CHANGE_THRESHOLD = 0.03f;  // 开爪位置变化阈值，开爪过程位置变化大于该值，明确有开爪动作，才可进行高电流冲关爪，避免错方向，单位 rad
    static constexpr float CLOSE_STUCK_CURRENT_THRESHOLD = 1.0f;    // 关爪卡死检测电流阈值，超过阈值后可判断为卡死状态，单位 A
    static constexpr float CLOSE_STUCK_DETECTION_THRESHOLD = 1.0f;  // 关爪卡死检测位置阈值，自开始关爪动作开始，小于阈值可判断为卡死状态，单位 rad
    static constexpr int CLOSE_STARTUP_DELAY_MS = 1000;             // 关爪启动卡死检测延时，避免电机启动初期运动位置过小的误判，单位 ms
    static constexpr float CLOSE_IMPACT_CURRENT = 3.0f;             // 关爪冲击电流，单位 A
    static constexpr int CLOSE_IMPACT_DURATION_MS = 200;            // 关爪冲击持续时间，单位 ms
    static constexpr int CLOSE_IMPACT_INTERVAL_MS = 800;            // 关爪冲击间隔时间，单位 ms
    static constexpr int CLOSE_MAX_ATTEMPTS = 10;                   // 关爪最大尝试次数，达到次数后认为到达关爪限位

    void measure_range();
    bool find_claw_limit_velocity_control(bool is_open_direction, float kp, float kd, float alpha,
        float max_current, float stall_current_threshold, 
        float stall_velocity_threshold, float dt, 
        int timeout_ms,
        const std::unordered_map<MotorId, bool>& can_perform_3a_impact);
    void adjust_range(std::unordered_map<MotorId, float> open_limit_positions, std::unordered_map<MotorId, float> close_limit_positions);

    // 量程映射：百分比 <-> 逻辑坐标(rad)
    // 逻辑坐标定义与 current_cmd/target_cmd 一致：已做方向(negtive)与零点(zero_offset)校正
    float mapPercentToLogicalRad(MotorId id, float percent);
    float mapLogicalRadToPercent(MotorId id, float logical_rad);

    void updateMotorCmd(MotorId id, double pos, double vel, double torque, double kp, double kd);
    float lowPassFilter(float input, float prevOutput, float alpha);
    void clear_all_torque();

    void control_thread();
    bool waitForAllMotorsFeedback(int timeout_ms);
    void init_status_variables();
    bool waitForOperationStatus(MotorId id, Operation expected_op, OperationStatus expected_status, int timeout_ms, const char* operation_name);

    // 获取按 MotorId 升序排列的电机 ID 列表（用于 vector<->map 的稳定映射）
    std::vector<MotorId> get_sorted_motor_ids() const;

    // 刹车相关函数：计算速度系数（对齐串口版）
    float calculate_brake_speed_factor(float current_position, float target_position,
                                       float start_position, float end_position);

    // 回调上下文
    canbus_sdk::CallbackContext msg_callback_context_;
    canbus_sdk::CallbackContext tef_callback_context_;

    struct MotorCtrlData {
        Operation operation;              // 当前操作
        OperationStatus operation_status; // 操作状态
        bool  feedback_received;          // 是否收到反馈
        ////////////////////////////////////////////////
        RevoMotor motor;                  // 电机实例
        RevoMotorConfig_t config;         // 电机配置
        RevoMotorCmd_t cmd;               // 电机控制命令
        mutable std::mutex cmd_mutex;     // 保护cmd字段的互斥锁

        MotorCtrlData(bool fr, RevoMotor&& m, const RevoMotorConfig_t& c)
            : operation(Operation::IDLE), operation_status(OperationStatus::SUCCESS),
              feedback_received(fr), motor(std::move(m)), config(c),
              cmd{0.0, 0.0, 0.0, 0.0, 0.0}, cmd_mutex() {}

        bool isEnabled() const {
            return operation == Operation::ENABLE 
                && feedback_received
                && (operation_status == OperationStatus::SUCCESS || operation_status == OperationStatus::PENDING) ;
        }
        // 线程安全地获取电机命令
        RevoMotorCmd_t getCmd() const {
            std::lock_guard<std::mutex> lock(cmd_mutex);
            return cmd;
        }

        // 线程安全地设置电机命令
        void setCmd(const RevoMotorCmd_t& new_cmd) {
            std::lock_guard<std::mutex> lock(cmd_mutex);
            cmd = new_cmd;
        }
    };

    std::mutex state_lock;
    std::mutex target_lock;
    std::unordered_map<MotorId, motorevo::RevoMotorCmd_t> target_cmd; // 对应原来的8个并列的数组，舍弃 vel_kpid
    std::unordered_map<MotorId, motorevo::RevoMotorCmd_t> old_target_cmd;
    std::unordered_map<MotorId, motorevo::RevoMotorCmd_t> current_cmd;

    // 量程测量与映射（按电机ID存储）
    std::unordered_map<MotorId, float> open_limit_positions_;   // 开爪限位（rad）
    std::unordered_map<MotorId, float> close_limit_positions_;  // 关爪限位（rad）
    std::unordered_map<MotorId, float> joint_start_positions_;        // 实际使用起点（rad）
    std::unordered_map<MotorId, float> joint_end_positions_;          // 实际使用终点（rad）

    // 设备ID -> CAN总线名称 映射（用于反注册等操作）
    std::unordered_map<MotorId, std::string> device_canbus_map_;
    std::unordered_map<MotorId, MotorCtrlData> motor_ctrl_datas_;

    // 控制线程
    std::atomic<bool> thread_running{false};
    std::thread control_thread_;
    std::atomic<bool> target_updated_{false};
};

}

#endif // LEJUCLAW_CAN_CUSTOMED_H