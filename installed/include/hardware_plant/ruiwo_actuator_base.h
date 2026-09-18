#ifndef RUIWO_ACTUATOR_BASE_H
#define RUIWO_ACTUATOR_BASE_H

#include <vector>
#include <cstdint>
#include <map>

/// @brief Revo电机故障码
/// 一旦驱动板检测到故障，将会从 Motor State 自动切回 Rest State 以保护驱动器和电机
/// 在排除异常情况后，发送 Enter Rest State 命令清除故障码，
/// 再发送 Enter Motor State 命令让电机重新恢复运行
/// See Details:《Motorevo Driver User Guide v0.2.3》- 6.5.1 反馈帧格式 - 故障码
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
 * @brief Ruiwo电机执行器抽象基类
 *
 **/
class RuiwoActuatorBase {
public:
    enum class State {
        None,
        Enabled,
        Disabled,
        Ignored,
    };
    struct MotorStateData {
        uint8_t id;
        State   state;
        MotorStateData():id(0x0), state(State::None) {}
        MotorStateData(uint8_t id_, State state_):id(id_), state(state_) {}
    };    
    using MotorStateDataVec = std::vector<MotorStateData>;   
public:
    /**
     * @brief 电机执行器基类构造函数
     *
     * @param unused 未使用参数（用于兼容性）
     * @param is_cali 是否处于标定模式
     */
    RuiwoActuatorBase() {}

    virtual ~RuiwoActuatorBase() = default;

    /**
     * @brief 初始化电机执行器
     *
     * @return int 返回0表示成功，否则返回错误码
     *  0: 成功
     *  1: 配置错误，如配置文件不存在、解析错误等
     *  2: CAN总线错误，如CAN总线初始化失败等
     *  3: motor error, e.g. 电机使能、失能过程中出现不可清除的故障码...
     *  ...
     */
    virtual int initialize() = 0;

    /**
     * @brief 使能所有电机，对全部电机执行 Enter Motor State 进入 Motor State 运行模式
     * 
     * @return int 成功返回 0，失败返回其他错误码
     * 1: 通信问题，包括：CAN 消息发送/接收失败或者超时
     * 2: 严重错误!!! 严重错误!!! 返回 2 时说明有电机存在严重故障码(RuiwoErrorCode 中大于 128 的故障码)
     */
    virtual int enable() = 0;

    /**
     * @brief 对全部电机执行 Enter Reset State 进入 Reset State 运行模式
     * 
     * @note: enter reset state 可用于清除故障码（大于 128 的故障码无法清除）
     * @return int 成功返回 0，失败返回其他错误码
     * 1: 通信问题，包括：CAN 消息发送/接收失败或者超时
     * 2: 严重错误!!! 严重错误!!! 返回 2 时说明有电机存在严重故障码(RuiwoErrorCode 中大于 128 的故障码)
     */
    virtual int disable() = 0;

    /**
     * @brief 失能指定电机，执行 Enter Reset State
     *
     * @param motorIndex 要禁用的电机索引
     * @return true 成功，false 失败
     */
    virtual bool disableMotor(int motorIndex) = 0;

    /**
     * @brief 关闭电机执行器并释放资源
     */
    virtual void close() = 0;

    /**
     * @brief 保存当前位置为零位
     */
    virtual void saveAsZeroPosition() = 0;

    /**
     * @brief 保存零位到持久存储
     */
    virtual void saveZeroPosition() = 0;

    /**
     * @brief 设置示教模式
     *
     * @param mode 示教模式值 1：示教模式 0：非示教模式
     */
    virtual void set_teach_pendant_mode(int mode) = 0;

    /**
     * @brief 改变指定关节编码器零位圈数
     *
     * @param index 关节索引
     * @param direction 方向值
     */
    virtual void changeEncoderZeroRound(int index, double direction) = 0;

    /**
     * @brief 调整指定关节零位位置
     *
     * @param index 关节索引
     * @param offset 偏移量
     */
    virtual void adjustZeroPosition(int index, double offset) = 0;

    /**
     * @brief 获取电机零点位置
     *
     * @return std::vector<double> 电机零点位置列表
     */
    virtual std::vector<double> getMotorZeroPoints() = 0;

    /**
     * @brief 获取所有电机的状态信息
     *
     * @return MotorStateDataVec 电机状态数据向量，包含每个电机的ID和状态
     */
    virtual MotorStateDataVec get_motor_state() = 0;;

    /**
     * @brief 设置多个关节位置
     *
     * @param index 关节索引 [0,1,2,3,...]
     * @param positions 目标位置（角度）
     * @param torque 力矩值
     * @param velocity 速度值（角度/秒）
     * @param kp 位置比例增益（为空时使用缓存的 rt_params.kp_pos）
     * @param kd 位置微分增益（为空时使用缓存的 rt_params.kd_pos）
     */
    virtual void set_positions(const std::vector<uint8_t> &index,
                             const std::vector<double> &positions,
                             const std::vector<double> &torque,
                             const std::vector<double> &velocity,
                             const std::vector<double> &kp = {},
                             const std::vector<double> &kd = {}) = 0;

    /**
     * @brief 设置多个关节力矩
     *
     * @param index 关节索引 [0,1,2,3,...]
     * @param torque 力矩值
     */
    virtual void set_torque(const std::vector<uint8_t> &index,
                           const std::vector<double> &torque) = 0;

    /**
     * @brief 设置多个关节速度
     *
     * @param index 关节索引 [0,1,2,3,...]
     * @param velocity 速度值
     */
    virtual void set_velocity(const std::vector<uint8_t> &index,
                             const std::vector<double> &velocity) = 0;

    /**
     * @brief 设置指定关节的kp_pos和kd_pos参数
     *
     * @param joint_indices 关节索引列表 (0-based)
     * @param kp_pos kp_pos值列表，如果为空则不修改
     * @param kd_pos kd_pos值列表，如果为空则不修改
     */
    virtual void set_joint_gains(const std::vector<int> &joint_indices,
                               const std::vector<double> &kp_pos,
                               const std::vector<double> &kd_pos) = 0;

    /**
     * @brief 获取指定关节的kp_pos和kd_pos参数
     *
     * @param joint_indices 关节索引列表，如果为空则返回所有关节
     * @return std::vector<std::vector<double>> 第一个vector是kp_pos，第二个是kd_pos
     */
    virtual std::vector<std::vector<double>> get_joint_gains(const std::vector<int> &joint_indices = {}) = 0;

    /**
     * @brief 获取所有关节当前位置
     *
     * @return std::vector<double> 位置（弧度）
     */
    virtual std::vector<double> get_positions() = 0;

    /**
     * @brief 获取所有关节当前力矩值
     *
     * @return std::vector<double> 力矩值
     */
    virtual std::vector<double> get_torque() = 0;

    /**
     * @brief 获取所有关节当前速度值
     *
     * @return std::vector<double> 速度值
     */
    virtual std::vector<double> get_velocity() = 0;

    /**
     * @brief 设置初始位置（可选接口，非必须实现）
     *
     * @param set_positions 初始位置向量
     */
    virtual void setInitPosition(std::vector<float> set_positions) 
    {
        std::cout << "setInitPosition interface not implemented" << std::endl;
        return;
    }

    /**
     * @brief 设置保存零点时的偏移调整参数
     * @param zero_offset_adjustments 电机索引到偏移量的映射（弧度），在保存零点时会应用到对应电机
     * @note 此接口用于在保存零点时对特定电机应用额外的偏移调整，例如v15版本机器人的特殊处理
     */
    virtual void setZeroOffsetAdjustments(const std::map<size_t, double>& zero_offset_adjustments) = 0;
};

#endif // RUIWO_ACTUATOR_BASE_H