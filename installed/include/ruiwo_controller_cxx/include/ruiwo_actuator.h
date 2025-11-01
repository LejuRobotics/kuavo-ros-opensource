#ifndef RUIWO_ACTUATOR_CPP_H
#define RUIWO_ACTUATOR_CPP_H
#include <cassert>
#include <vector>
#include <string>
#include <thread>
#include <mutex>
#include <iterator>

/// @brief Revo电机故障码
/// 一旦驱动板检测到故障，将会从 Motor State 自动切回 Rest State 以保护驱动器和电机
/// 在排除异常情况后，发送 Enter Rest State 命令清除故障码，
// 再发送 Enter Motor State 命令让电机重新恢复运行
// See Details:《Motorevo Driver User Guide v0.2.3》- 6.5.1 反馈帧格式 - 故障码
enum class RuiwoErrCode:uint8_t {
    NO_FAULT = 0x00,                       // 无故障
    DC_BUS_OVER_VOLTAGE = 0x01,            // 直流母线电压过压
    DC_BUS_UNDER_VOLTAGE = 0x02,           // 直流母线电压欠压
    ENCODER_ANGLE_FAULT = 0x03,            // 编码器电角度故障
    DRV_DRIVER_FAULT = 0x04,               // DRV 驱动器故障
    DC_BUS_CURRENT_OVERLOAD = 0x05,        // 直流母线电流过流
    MOTOR_A_PHASE_CURRENT_OVERLOAD = 0x06, // 电机 A 相电流过载
    MOTOR_B_PHASE_CURRENT_OVERLOAD = 0x07, // 电机 B 相电流过载
    MOTOR_C_PHASE_CURRENT_OVERLOAD = 0x08, // 电机 C 相电流过载
    DRIVER_BOARD_OVERHEAT = 0x09,          // 驱动板温度过高
    MOTOR_WINDING_OVERHEAT = 0x0A,         // 电机线圈过温
    ENCODER_FAILURE = 0x0B,                // 编码器故障
    CURRENT_SENSOR_FAILURE = 0x0C,         // 电流传感器故障
    OUTPUT_ANGLE_OUT_OF_RANGE = 0x0D,      // 输出轴实际角度超过通信范围：CAN COM Theta MIN ~ CAN COM Theta MAX
    OUTPUT_SPEED_OUT_OF_RANGE = 0x0E,      // 输出轴速度超过通信范围 CAN COM Velocity MIN ~ CAN COM Velocity MAX
    STUCK_PROTECTION = 0x0F,               // 堵转保护：电机电枢电流(Iq)大于 Stuck Current，同时电机速度小于 Stuck Velocity，持续时间超过 Stuck Time 后触发
    CAN_COMMUNICATION_LOSS = 0x10,         // CAN 通讯丢失：超过 CAN COM TIMEOUT 时间没有收到 CAN 数据帧时触发

    // WARNING: 大于 128 的故障码为开机自检检测出的故障码，无法用该方法清除
    //
    ABS_ENCODER_OFFSET_VERIFICATION_FAILURE = 0x81, // 离轴/对心多圈绝对值编码器接口帧头校验失败
    ABSOLUTE_ENCODER_MULTI_TURN_FAILURE = 0x82,     // 对心多圈绝对值编编码器多圈接口故障
    ABSOLUTE_ENCODER_EXTERNAL_INPUT_FAILURE = 0x83, // 对心多圈绝对值编码器外部输入故障
    ABSOLUTE_ENCODER_SYSTEM_ANOMALY = 0x84,         // 对心多圈绝对值编码器读值故障
    ERR_OFFS = 0x85,                                // 对心多圈绝对值编码器ERR_OFFS
    ERR_CFG = 0x86,                                 // 对心多圈绝对值编码器ERR_CFG
    ILLEGAL_FIRMWARE_DETECTED = 0x88,               // 检测到非法固件
    INTEGRATED_STATOR_DRIVER_DAMAGED = 0x89,        // 集成式栅极驱动器初始化失败
 };

/**
 * Structure representing motor parameters for Ruiwo actuators
 * Contains control parameters for position and velocity control
 */
struct RuiwoMotorParam_t {
    // 关节参数[vel, kp_pos, kd_pos, tor, kp_vel, kd_vel, ki_vel] vel的pid只有在servo模式下才起作用
    int vel;   
    int kp_pos;
    int kd_pos;
    int tor;   
    int kp_vel;
    int kd_vel;
    int ki_vel;
    
    RuiwoMotorParam_t() : vel(0), kp_pos(0), kd_pos(0), tor(0), kp_vel(0), kd_vel(0), ki_vel(0) {}
    
    RuiwoMotorParam_t(int vel_, int kp_pos_, int kd_pos_, int tor_, int kp_vel_, int kd_vel_, int ki_vel_) :
        vel(vel_), kp_pos(kp_pos_), kd_pos(kd_pos_), tor(tor_), kp_vel(kp_vel_), kd_vel(kd_vel_), ki_vel(ki_vel_) {}

    RuiwoMotorParam_t(const std::vector<int> & motor_parameters) {
        assert(motor_parameters.size() == 7 && "Invalid motor parameters size");
        vel = motor_parameters[0];
        kp_pos = motor_parameters[1];
        kd_pos = motor_parameters[2];
        tor = motor_parameters[3];
        kp_vel = motor_parameters[4];
        kd_vel = motor_parameters[5];
        ki_vel = motor_parameters[6];
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

class RuiWoActuator
{
 public:
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
    int initialize();
        
    /**
     * @brief 使能所有电机，对全部电机执行 Enter Motor State 进入 Motor State 运行模式
     * 
     * @return int 成功返回 0，失败返回其他错误码
     * 1: 通信问题，包括：CAN 消息发送/接收失败或者超时
     * 2: 严重错误!!! 严重错误!!! 返回 2 时说明有电机存在严重故障码(RuiwoErrorCode 中大于 128 的故障码)
     */
    int enable();

    /**
     * @brief 对全部电机执行 Enter Reset State 进入 Reset State 运行模式
     * 
     * @note: enter reset state 可用于清除故障码（大于 128 的故障码无法清除）
     * @return int 成功返回 0，失败返回其他错误码
     * 1: 通信问题，包括：CAN 消息发送/接收失败或者超时
     * 2: 严重错误!!! 严重错误!!! 返回 2 时说明有电机存在严重故障码(RuiwoErrorCode 中大于 128 的故障码)
     */
    int disable();

    bool disableMotor(int motorIndex);
    void close();

    void go_to_zero();
    void set_zero();
    void saveAsZeroPosition();
    void saveZeroPosition();
    void set_teach_pendant_mode(int mode);
    void changeEncoderZeroRound(int index, double direction);
    void multi_turn_zeroing(const std::vector<int>& dev_ids);
    std::vector<int> get_all_joint_addresses();
    
    /**
     * @brief Set the positions object
     * 
     * @param index     [0,1,2,3,...]
     * @param positions  单位为角度
     * @param torque     
     * @param velocity  单位为角度/s
     */
    void set_positions(const std::vector<uint8_t> &index, 
        const std::vector<double> &positions, 
        const std::vector<double> &torque, 
        const std::vector<double> &velocity);

    /**
     * @brief Set the torque object
     * 
     * @param index [0,1,2,3,...]
     * @param torque 
     */
    void set_torque(const std::vector<uint8_t> &index, const std::vector<double> &torque);

    /**
     * @brief Set the velocity object
     * 
     * @param index [0,1,2,3,...]
     * @param velocity 
     */
    void set_velocity(const std::vector<uint8_t> &index, const std::vector<double> &velocity);
    
    /**
     * @brief Get the positions object
     * 
     * @return std::vector<double> radians
     */
    std::vector<double> get_positions();
    std::vector<double> get_torque();
    std::vector<double> get_velocity();

    std::vector<std::vector<float>> get_joint_state();

    MotorStateDataVec get_motor_state();
    std::vector<std::vector<float>> get_joint_origin_state();

    /**
     * @brief 设置指定关节的kp_pos和kd_pos参数
     * 
     * @param joint_indices 关节索引列表 (0-based)
     * @param kp_pos kp_pos值列表，如果为空则不修改
     * @param kd_pos kd_pos值列表，如果为空则不修改
     */
    void set_joint_gains(const std::vector<int> &joint_indices, const std::vector<double> &kp_pos, const std::vector<double> &kd_pos);

    /**
     * @brief 获取指定关节的kp_pos和kd_pos参数
     * 
     * @param joint_indices 关节索引列表，如果为空则返回所有关节
     * @return std::vector<std::vector<double>> 第一个vector是kp_pos，第二个是kd_pos
     */
    std::vector<std::vector<double>> get_joint_gains(const std::vector<int> &joint_indices = {});    

private:
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
};

/**
 * @brief 将RuiwoErrCode枚举值转换为可读的字符串描述
 *
 * @param[in] errcode RuiwoErrCode枚举值，表示具体的电机故障码
 * @return std::string 故障码的中文描述
 */
std::string RuiwoErrCode2string(RuiwoErrCode errcode);

#endif // RUIWO_ACTUATOR_CPP_H
