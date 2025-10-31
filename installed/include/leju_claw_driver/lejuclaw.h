#ifndef LEJUCLAW_CPP_IMPL_H
#define LEJUCLAW_CPP_IMPL_H
#define POSITION_TOLERANCE 1
#include <vector>
#include <string>
#include <thread>
#include <mutex>
#include <algorithm>
#include <variant>
#include <yaml-cpp/yaml.h>
#include <cmath>
#include <fstream>
#include <pwd.h>
#include <unistd.h>
#include <iostream>
#include <iterator>
#include <time.h>
#include <chrono>
#include "ruiwoSDK.h"
#include <algorithm>

class LeJuClaw
{
//#define ENABLE_JOINT_DEBUG  // 取消注释此行以启用关节调试打印
 public:
    
     enum class PawMoveState  : int8_t {
        ERROR = -1,                       // 出现错误
        LEFT_REACHED_RIGHT_REACHED = 0,   // 所有夹爪到位
        LEFT_REACHED_RIGHT_GRABBED = 1,   // 左夹爪到位，右夹爪抓取到物品
        LEFT_GRABBED_RIGHT_REACHED = 2,   // 右夹爪到位，左夹爪抓取到物品
        LEFT_GRABBED_RIGHT_GRABBED = 3,   // 所有夹爪均夹取
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
public:
    LeJuClaw(std::string unused = "");
    ~LeJuClaw();
    int initialize(bool init_bmlib);
    void enable();
    void disable();
    void set_zero();
    void go_to_zero();
    void go_to_zero_with_current_control();
    bool find_claw_limit_velocity_control(bool is_open_direction, float kp, float kd, float alpha,
                            float max_current, float stall_current_threshold, 
                            float stall_velocity_threshold, float dt, 
                            int timeout_ms,
                            const std::vector<bool>& can_perform_3a_impact = std::vector<bool>());
    void set_positions(const std::vector<uint8_t> &index, const std::vector<double> &positions, const std::vector<double> &torque, const std::vector<double> &velocity);
    void set_torque(const std::vector<uint8_t> &index, const std::vector<double> &torque);
    void set_velocity(const std::vector<uint8_t> &index, const std::vector<double> &velocity);
    void set_joint_state(int index, const std::vector<float> &state);
    PawMoveState move_paw(const std::vector<double> &positions, const std::vector<double> &velocity,const std::vector<double> &torque);
    PawMoveState move_paw(const std::vector<double> &positions, const std::vector<double> &velocity,const std::vector<double> &torque, bool is_vr_mode);
    std::vector<std::vector<float>> get_joint_state();
    std::vector<double> get_positions();
    std::vector<double> get_torque();
    std::vector<double> get_velocity();
    void close();
    MotorStateDataVec get_motor_state();
    std::vector<int> disable_joint_ids;
    
    void set_claw_kpkd(int kp, int kd);
    float lowPassFilter(float input, float prevOutput, float alpha);
    void clear_all_torque();

private:
    // 控制参数常量
    static constexpr float DEFAULT_KP = 18.00f;                    // 比例增益系数，用于位置控制响应速度
    static constexpr float DEFAULT_KD = 3.0f;                      // 微分增益系数，用于阻尼控制
    static constexpr float DEFAULT_ALPHA = 0.20f;                  // 低通滤波器系数，用于信号平滑
    static constexpr float DEFAULT_MAX_CURRENT = 2.50f;            // 最大电流限制，防止过载保护，单位 A
    static constexpr float DEFAULT_MIN_ERROR = 0.02f;              // 最小误差阈值，用于判断到位精度，单位 rad
    static constexpr float DEFAULT_DT = 0.002f;                    // 控制周期，控制循环时间间隔，单位 s
    // 卡死检测参数
    static constexpr float STUCK_DETECTION_DELAY_MS = 50.0f;       // 卡死检测延迟时间，单位 ms
    static constexpr float STUCK_DETECTION_TIME_MS = 50.0f;        // 卡死检测持续时间，单位 ms
    static constexpr float STUCK_POSITION_THRESHOLD = 0.002f;      // 卡死位置阈值，小于此值认为卡死，单位 rad
    static constexpr float IMPACT_CURRENT = 3.0f;                  // 冲击电流阈值，单位 A
    static constexpr float IMPACT_DURATION_MS = 200.0f;            // 冲击持续时间，单位 ms
    static constexpr float IMPACT_INTERVAL_MS = 800.0f;            // 冲击间隔时间，单位 ms
    static constexpr float LIMIT_RANGE_PERCENT = 5.0f;             // 限位范围百分比，包括VR与非VR模式，在行程两端此范围内才执行反冲，单位 %
    // 稳定检测参数（用于若未到达目标位置，但处于稳定状态，为 ros 服务反馈已到达位置）
    static constexpr float STABLE_POSITION_THRESHOLD = 0.005f;     // 稳定位置阈值，单位 rad
    static constexpr float STABLE_VELOCITY_THRESHOLD = 0.1f;       // 稳定速度阈值，单位 rad/s
    static constexpr float STABLE_DETECTION_TIME_MS = 50.0f;       // 稳定检测时间，单位 ms
    // VR控制检测参数
    static constexpr int VR_CONTROL_TIMEOUT_MS = 60;               // VR模式下的超时时间，单位 ms
    static constexpr int VR_STUCK_DETECTION_CYCLES = 50;           // VR模式下卡死检测需要的周期个数，每个周期时长为DEFAULT_DT，超过次数后不动，判断为卡死
    static constexpr float VR_STUCK_POSITION_THRESHOLD = 0.01f;    // VR模式下卡死位置阈值，连续VR_STUCK_DETECTION_CYCLES个周期内，位置变化都小于该弧度，认为夹爪卡死，单位 rad
    static constexpr float VR_TARGET_POSITION_THRESHOLD = 0.5f;    // VR模式下目标位置差值阈值，当目标位置与当前位置差值大于此值时才进行卡死检测，单位 rad
    // 刹车参数
    static constexpr float BRAKE_RANGE_PERCENT = 35.0f;             // 刹车范围百分比，在行程两端刹车范围内减速，单位 %
    static constexpr float BRAKE_MIN_SPEED_FACTOR = 0.01f;          // 刹车最小速度系数，在限位处速度降至最小值，单位 %
    static constexpr float BRAKE_CURVE_EXPONENT = 30.0f;            // 刹车曲线指数，控制减速曲线形状
    
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
    
    std::vector<float> create_zero_vector(size_t size);
    void send_torque_with_lock(const std::vector<float>& torques);
    void send_claw_torque_only(const std::vector<float>& torques);  // 发送夹爪电流，左右独立控
    void send_torque_direct(const std::vector<float>& torques);     // 直接发送电流指令，使用run_torque_mode
    void send_velocity_direct(const std::vector<float>& velocities); // 直接发送速度指令，使用run_vel_mode
    
    // 刹车相关函数
    float calculate_brake_speed_factor(float current_position_percent, float target_position_percent, 
                                      float start_position, float end_position); // 计算刹车速度系数
    
    private:
    void control_thread();
    // void get_parameter();
    void get_config(const std::string &config_file);
    std::string get_home_path();
    void interpolate_move(const std::vector<float> &start_positions, const std::vector<float> &target_positions, const std::vector<float> &speeds, float dt);
    std::vector<std::vector<float>> interpolate_positions_with_speed(const std::vector<float> &a, const std::vector<float> &b, const std::vector<float> &speeds, float dt);
    void send_positions(const std::vector<int> &index, const std::vector<float> &pos, const std::vector<float> &torque, const std::vector<float> &velocity);
    std::vector<int> get_joint_addresses(const YAML::Node &config, const std::string &joint_type, int count);
    std::vector<bool> get_joint_online_status(const YAML::Node &config, const std::string &joint_type, int count);
    std::vector<std::vector<int>> get_joint_parameters(const YAML::Node &config, const std::string &joint_type, int count);
    // 关节ID
    std::vector<int> Claw_joint_address = {0x0F, 0x10};
    
    static inline const std::vector<std::vector<int>> Claw_parameter = {{0, 10, 2, 0, 0, 0, 0},
                                                                        {0, 10, 2, 0, 0, 0, 0}};

                                                                           
    // 反转电机ID 可能会用上?
    static inline  std::vector<int> Negtive_joint_address_list = {};
    
    // // ptm:力控模式 servo:伺服模式
    static inline  std::string Control_mode = "ptm";
    
    // 高电流使用统计
    static inline int three_amp_current_usage_count = 0;

    
    std::vector<bool> Claw_joint_online;

    std::vector<std::vector<int>> Joint_parameter_list;
    std::vector<int> Joint_address_list;
    std::vector<bool> Joint_online_list;

    // RUIWOTools ruiwo;
    bool thread_running;
    bool thread_end;
    std::thread control_thread_;
    std::mutex sendpos_lock;
    std::mutex recvpos_lock;
    std::mutex sendvel_lock;
    std::mutex recvvel_lock;
    std::mutex sendtor_lock;
    std::mutex recvtor_lock;
    std::mutex state_lock;
    std::mutex update_lock;
    std::mutex can_lock;

    bool target_update;
    std::vector<float> target_positions;
    std::vector<float> target_velocity;
    std::vector<float> target_torque;
    std::vector<int> target_pos_kp;
    std::vector<int> target_pos_kd;
    std::vector<int> target_vel_kp;
    std::vector<int> target_vel_kd;
    std::vector<int> target_vel_ki;

    std::vector<float> old_target_positions;
    std::vector<float> current_positions;
    std::vector<float> current_torque;
    std::vector<float> current_velocity;
    std::vector<std::vector<float>> joint_status;
    std::vector<float> joint_start_positions;  // 行程起点位置
    std::vector<float> joint_end_positions;    // 行程终点位置
    
    // VR控制相关变量
    std::chrono::steady_clock::time_point last_target_update_time;  // 上次目标位置更新时间
    bool is_vr_control_mode;                                        // 是否为VR控制模式
    std::chrono::steady_clock::time_point movement_start_time;      // 运动开始时间
    bool movement_timeout_enabled;                                  // 是否启用运动超时机制
};

#endif // LEJUCLAW_CPP_H