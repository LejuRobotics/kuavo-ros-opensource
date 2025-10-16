#ifndef RUIWO_ACTUATOR_BASE_H
#define RUIWO_ACTUATOR_BASE_H

#include <vector>
#include <cstdint>

/**
 * @brief Ruiwo电机执行器抽象基类
 *
 **/
class RuiwoActuatorBase {
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
     *  ...
     */
    virtual int initialize() = 0;

    /**
     * @brief 使能所有电机
     */
    virtual void enable() = 0;

    /**
     * @brief 禁用所有电机
     */
    virtual void disable() = 0;

    /**
     * @brief 禁用指定电机
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
     * @param mode 示教模式值
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
     */
    virtual void set_positions(const std::vector<uint8_t> &index,
                             const std::vector<double> &positions,
                             const std::vector<double> &torque,
                             const std::vector<double> &velocity) = 0;

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
};

#endif // RUIWO_ACTUATOR_BASE_H