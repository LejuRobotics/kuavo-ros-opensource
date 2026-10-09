#ifndef RUIWO_ACTUATOR_CPP_H
#define RUIWO_ACTUATOR_CPP_H
#include <cassert>
#include <vector>
#include <string>
#include <thread>
#include <mutex>
#include <iterator>
#include <iostream> 
#include <map>
#include "ruiwo_actuator_base.h"
#include <iostream>

/**
 * Structure representing motor parameters for Ruiwo actuators
 * Contains control parameters for position and velocity control
 */
struct RuiwoMotorParam_t {
    // 关节参数[vel, kp_pos, kd_pos, tor, kp_vel, kd_vel, ki_vel] vel的pid只有在servo模式下才起作用
    double vel;
    double kp_pos;
    double kd_pos;
    double tor;
    double kp_vel;
    double kd_vel;
    double ki_vel;

    RuiwoMotorParam_t() : vel(0.0), kp_pos(0.0), kd_pos(0.0), tor(0.0), kp_vel(0.0), kd_vel(0.0), ki_vel(0.0) {}

    RuiwoMotorParam_t(double vel_, double kp_pos_, double kd_pos_, double tor_, double kp_vel_, double kd_vel_, double ki_vel_) :
        vel(vel_), kp_pos(kp_pos_), kd_pos(kd_pos_), tor(tor_), kp_vel(kp_vel_), kd_vel(kd_vel_), ki_vel(ki_vel_) {}

    RuiwoMotorParam_t(const std::vector<double> & motor_parameters) {
        assert(motor_parameters.size() == 7 && "Invalid motor parameters size");
        vel = motor_parameters[0];
        kp_pos = motor_parameters[1];
        kd_pos = motor_parameters[2];
        tor = motor_parameters[3];
        kp_vel = motor_parameters[4];
        kd_vel = motor_parameters[5];
        ki_vel = motor_parameters[6];
    }

    // 兼容性构造函数，支持从std::vector<int>构造
    RuiwoMotorParam_t(const std::vector<int> & motor_parameters) {
        assert(motor_parameters.size() == 7 && "Invalid motor parameters size");
        vel = static_cast<double>(motor_parameters[0]);
        kp_pos = static_cast<double>(motor_parameters[1]);
        kd_pos = static_cast<double>(motor_parameters[2]);
        tor = static_cast<double>(motor_parameters[3]);
        kp_vel = static_cast<double>(motor_parameters[4]);
        kd_vel = static_cast<double>(motor_parameters[5]);
        ki_vel = static_cast<double>(motor_parameters[6]);
    }
};

struct RuiwoMotorConfig_t {
    bool negtive;
    bool online;
    int address;
    double zero_offset;
    std::string joint_name;
    RuiwoMotorParam_t parameter;

    RuiwoMotorConfig_t() : negtive(false), online(false), address(0), zero_offset(0.0), joint_name(""), parameter() {}

    bool operator<(const RuiwoMotorConfig_t& other) const {
        // 即 Left_joint_arm_* < Right_joint_arm_* < Head_joint_*
        auto getJointTypePriority = [](const std::string& name) -> int {
            if (name.find("Left_joint_arm") == 0) return 1;
            if (name.find("Right_joint_arm") == 0) return 2;
            if (name.find("Head_joint") == 0) return 3;
            return 4;
        };
        
        // 获取关节类型优先级
        int thisPriority = getJointTypePriority(joint_name);
        int otherPriority = getJointTypePriority(other.joint_name);
        
        // 如果优先级不同，按优先级排序
        if (thisPriority != otherPriority) {
            return thisPriority < otherPriority;
        }
        
        // 如果优先级相同，按关节名称排序
        return joint_name < other.joint_name;
    }

    bool operator==(const RuiwoMotorConfig_t& other) const {
        return (joint_name == other.joint_name);
    }
    friend std::ostream& operator<<(std::ostream& os, const RuiwoMotorConfig_t& config) {
        os << "[" << config.joint_name << "]: address: 0x" << std::hex << config.address << std::dec
        << ", online: " << (config.online ? "True" : "False")
        << ", negtive: " << (config.negtive ? "True" : "False")
        << ", offset: " << config.zero_offset
        << "  params: [" 
        << config.parameter.vel << ", " 
        << config.parameter.kp_pos << ", " 
        << config.parameter.kd_pos << ", " 
        << config.parameter.tor << ", " 
        << config.parameter.kp_vel << ", " 
        << config.parameter.kd_vel << ", " 
        << config.parameter.ki_vel << "]";
        return os;
    }
};

class RuiWoActuator : public RuiwoActuatorBase
{
public:
    RuiWoActuator(std::string unused = "", bool is_cali = false);
    ~RuiWoActuator();
    
    /**
     * @brief 模块初始化, 若该函数调用未返回成功，后续其他接口不可用
     * 
     * @return int return 0 if success, otherwise return error
     *  0: success
     *  1: config error, e.g. config file not exist, parse error...
     *  2: canbus error, e.g. canbus init fail... 
     *  3: motor error, e.g. 电机使能、失能过程中出现不可清除的故障码...
     */
    int initialize() override;
        
    /**
     * @brief 使能所有电机，对全部电机执行 Enter Motor State 进入 Motor State 运行模式
     * 
     * @return int 成功返回 0，失败返回其他错误码
     * 1: 通信问题，包括：CAN 消息发送/接收失败或者超时
     * 2: 严重错误!!! 严重错误!!! 返回 2 时说明有电机存在严重故障码(RuiwoErrorCode 中大于 128 的故障码)
     */
    int enable() override;

    /**
     * @brief 对全部电机执行 Enter Reset State 进入 Reset State 运行模式
     * 
     * @note: enter reset state 可用于清除故障码（大于 128 的故障码无法清除）
     * @return int 成功返回 0，失败返回其他错误码
     * 1: 通信问题，包括：CAN 消息发送/接收失败或者超时
     * 2: 严重错误!!! 严重错误!!! 返回 2 时说明有电机存在严重故障码(RuiwoErrorCode 中大于 128 的故障码)
     */
    int disable() override;

    /**
     * @brief 对指定 index 的电机执行 Enter Reset State 进入 Reset State 运行模式，即失能电机
     * 
     * @param motorIndex 
     * @return true 
     * @return false 
     */
    bool disableMotor(int motorIndex) override;

    /**
     * @brief 关闭并释放所有资源
     * 
     */
    void close() override;
    void saveAsZeroPosition() override;
    /**
     * @brief 将当前零点位置输出到零点文件
     * 
     */
    void saveZeroPosition() override;
    void set_teach_pendant_mode(int mode) override;
    void changeEncoderZeroRound(int index, double direction) override;
    void adjustZeroPosition(int index, double offset) override;
    std::vector<double> getMotorZeroPoints() override;
    void setZeroOffsetAdjustments(const std::map<size_t, double>& zero_offset_adjustments) override;
    
    /**
     * @brief 设置电机目标位置
     *
     * @param index 关节索引 [0,1,2,3,...]
     * @param positions 目标位置（角度）注意单位是角度
     * @param torque 力矩值
     * @param velocity 速度值（角度/秒）
     */
    void set_positions(const std::vector<uint8_t> &index,
        const std::vector<double> &positions,
        const std::vector<double> &torque,
        const std::vector<double> &velocity,
        const std::vector<double> &kp = {},
        const std::vector<double> &kd = {}) override;

    /**
     * @brief Set the torque object
     * 
     * @param index [0,1,2,3,...]
     * @param torque 
     */
    void set_torque(const std::vector<uint8_t> &index, const std::vector<double> &torque) override;

    /**
     * @brief 设置电机目标速度
     * 
     * @param index [0,1,2,3,...]
     * @param velocity 单位弧度/秒
     */
    void set_velocity(const std::vector<uint8_t> &index, const std::vector<double> &velocity) override;
    
    /**
     * @brief 获取所有关节当前位置
     *
     * @return std::vector<double> 位置（弧度）
     */
    std::vector<double> get_positions() override;
    std::vector<double> get_torque() override;
    std::vector<double> get_velocity() override;

    /**
     * @brief 获取电机状态
     * 
     * notes: std::vector<float> 长度为6: 电机ID、位置、速度、扭矩、温度、故障码
     * 
     * @return std::vector<std::vector<float>> 
     */
    std::vector<std::vector<float>> get_joint_state();

    /**
     * @brief 获取所有电机状态
     * 
     * @return MotorStateDataVec 
     */
    MotorStateDataVec get_motor_state() override;

    /**
     * @brief 设置指定关节的kp_pos和kd_pos参数
     * 
     * @param joint_indices 关节索引列表 (0-based)
     * @param kp_pos kp_pos值列表，如果为空则不修改
     * @param kd_pos kd_pos值列表，如果为空则不修改
     */
    void set_joint_gains(const std::vector<int> &joint_indices, const std::vector<double> &kp_pos, const std::vector<double> &kd_pos) override;

    /**
     * @brief 获取指定关节的kp_pos和kd_pos参数
     * 
     * @param joint_indices 关节索引列表，如果为空则返回所有关节
     * @return std::vector<std::vector<double>> 第一个vector是kp_pos，第二个是kd_pos
     */
    std::vector<std::vector<double>> get_joint_gains(const std::vector<int> &joint_indices = {}) override;

    void setInitPosition(std::vector<float> set_positions) override
    {
        size_t joint_size = ruiwo_mtr_config_.size();
        if(joint_size != set_positions.size())
        {
            std::cout << "set init pos failed, ruiwo_mtr_config_.size() != set_positions.size()" << std::endl;
            init_positions.assign(joint_size, 0);
            return;
        }
        init_positions = set_positions;
        std::cout << "set ruiwo motor init pos[ ";
        for (float pos : init_positions) // 使用基于范围的 for 循环，更简洁
        {
            std::cout << pos << " ";
        }
        std::cout << "]" << std::endl;
        is_set_init_pos_ = true;
    }

private:
    void go_to_zero();
    void set_zero();
    void multi_turn_zeroing(const std::vector<int>& dev_ids);
    std::vector<int> get_all_joint_addresses();
    std::vector<std::vector<float>> get_joint_origin_state();

    void control_thread();
    void recv_thread();
    void recv_message();
    bool parse_config_file(const std::string &config_file);

    float measure_head_torque(float position);
    void update_status();
    void interpolate_move(const std::vector<float> &start_positions, const std::vector<float> &target_positions, float speed, float dt);
    std::vector<std::vector<float>> interpolate_positions_with_speed(const std::vector<float> &a, const std::vector<float> &b, float speed, float dt);
   
    void set_joint_state(int index, const std::vector<float> &state);

    /**
     * @brief Sends positions to the specified motor indices, with dynamic kp/kd parameters.
     * 
     * @param index The indices of the motors to send positions to.
     * @param pos The positions to send to the motors.
     * @param torque The torques to send to the motors.
     * @param velocity The velocities to send to the motors.
     * @param current_kp The kp parameters for each motor, if nullptr uses default kp.
     * @param current_kd The kd parameters for each motor, if nullptr uses default kd.
     */
    void send_positions(const std::vector<int> &index, 
        const std::vector<float> &pos, 
        const std::vector<float> &torque, 
        const std::vector<float> &velocity, 
        const std::vector<float> *current_kp = nullptr, 
        const std::vector<float> *current_kd = nullptr);
    
    /**
     * @brief Sends positions to the specified motor indices without waiting for a response, with dynamic kp/kd parameters.
     * 
     * @param index The indices of the motors to send positions to.
     * @param pos The positions to send to the motors.
     * @param torque The torques to send to the motors.
     * @param velocity The velocities to send to the motors.
     * @param current_kp The kp parameters for each motor, if nullptr uses default kp.
     * @param current_kd The kd parameters for each motor, if nullptr uses default kd.
     */
    void send_positions_No_response(const std::vector<int> &index, 
        const std::vector<float> &pos, 
        const std::vector<float> &torque, 
        const std::vector<float> &velocity, 
        const std::vector<float> *current_kp = nullptr, 
        const std::vector<float> *current_kd = nullptr);

private:
    // 关节ID
    std::vector<int> Head_joint_address = {0x0D, 0x0E};
    // 小臂电机ID
    static inline  std::vector<int> Unnecessary_go_zero_list = {0x04, 0x05, 0x06, 0x0A, 0x0B, 0x0C};
    // ptm:力控模式 servo:伺服模式
    static inline  std::string Control_mode_ = "ptm";

    bool parse_config_flag_{false};
    bool is_set_init_pos_{false};
    bool thread_running{false};
    bool thread_end{true};
    std::thread control_thread_;
    std::thread recv_thread_;
    std::mutex sendpos_lock;
    std::mutex recvpos_lock;
    std::mutex sendvel_lock;
    std::mutex recvvel_lock;
    std::mutex sendtor_lock;
    std::mutex recvtor_lock;
    std::mutex state_lock;
    std::mutex update_lock;

    bool target_update;
    std::vector<float> init_positions;
    std::vector<float> target_positions;
    std::vector<float> target_velocity;
    std::vector<float> target_torque;

    std::vector<float> current_positions;
    std::vector<float> current_torque;
    std::vector<float> current_velocity;
    std::vector<std::vector<float>> joint_status;
    std::vector<std::vector<float>> origin_joint_status;

    float head_low_torque;
    float head_high_torque;

    bool multi_turn_encoder_mode = false;
    int teach_pendant_mode = 0;
    bool is_cali_ = false;

    std::vector<int> ratio = {36, 36, 36, 10, 10, 10, 36, 36, 36, 10, 10, 10, 36, 36};

    /** Data */
    std::vector<RuiwoMotorConfig_t> ruiwo_mtr_config_;
    
    // 保存零点时的偏移调整参数（电机索引到偏移量的映射，弧度）
    std::map<size_t, double> zero_offset_adjustments_;
    mutable std::mutex zero_offset_adjustments_mutex_;
};

/**
 * @brief 将RuiwoErrCode枚举值转换为可读的字符串描述
 *
 * @param[in] errcode RuiwoErrCode枚举值，表示具体的电机故障码
 * @return std::string 故障码的中文描述
 */
std::string RuiwoErrCode2string(RuiwoErrCode errcode);

#endif // RUIWO_ACTUATOR_CPP_H
