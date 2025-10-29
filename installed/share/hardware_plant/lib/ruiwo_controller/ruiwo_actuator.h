#ifndef RUIWO_ACTUATOR_H
#define RUIWO_ACTUATOR_H

#include <Python.h>
#include <vector>
#include <thread>
#include <iostream>
#include <fstream>
#include <yaml-cpp/yaml.h>
#include <string>
#include <cstdlib>
static PyObject *RuiWo_pJoinMethod;// 用于将python线程移动到c++线程中,避免GIL占用

/// @brief Revo电机故障码
/// 一旦驱动板检测到故障，将会从 Motor State 自动切回 Rest State 以保护驱动器和电机
/// 在排除异常情况后，发送 Enter Rest State 命令清除故障码，
/// 再发送 Enter Motor State 命令让电机重新恢复运行
/// See Details:《Motorevo Driver User Guide v0.2.2》- 6.5.1 反馈帧格式 - 故障码
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
 * @brief 将RuiwoErrCode枚举值转换为可读的字符串描述
 *
 * @param[in] errcode RuiwoErrCode枚举值，表示具体的电机故障码
 * @return std::string 故障码的中文描述
 */
std::string RuiwoErrCode2string(RuiwoErrCode errcode);

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
private:
    PyObject *pModule;
    PyObject *RuiWoActuatorClass;
    PyObject *ActuatorInstance;

    PyObject *pEnableMethod;
    PyObject *pCloseMethod;
    PyObject *pDisableMethod;
    PyObject *pSetPositionMethod;
    PyObject *pSetTorqueMethod;
    PyObject *pSetVelocityMethod;
    PyObject *pGetPositionMethod;
    PyObject *pGetTorqueMethod;
    PyObject *pGetVelocityMethod;
    PyObject *pGetJointStateMethod;
    PyObject *pCheckStateMethod;
    PyObject *pSaveZerosMethod{nullptr};
    PyObject *pSetZeroMethod{nullptr};
    PyObject *pChangEncoderMethod{nullptr};
    PyObject *pAdjustZeroMethod{nullptr};
    PyObject *pGetZeroPointsMethod{nullptr};
    PyObject *pSetTeachPendantModeMethod{nullptr};
    PyObject *pSetJointGainsMethod{nullptr};
    PyObject *pGetJointGainsMethod{nullptr};
    PyObject *pJoint_online_list;
    std::string pymodule_path;
    PyGILState_STATE gstate;
    std::thread pythonThread;
    bool is_multi_encode_mode;

public:
    // 需要传入python模块所在路径
    RuiWoActuator(std::string pymodule_path = "", bool is_cali = false);

    ~RuiWoActuator();
    bool is_cali_{false};
    int initialize();

    /**
     * @brief 使能所有电机，对全部电机执行 Enter Motor State 进入 Motor State 运行模式
     *
     * @return int 成功返回 0，失败返回其他错误码
     * 1: 通信问题，包括：CAN 消息发送/接收失败或者超时
     * 2: 严重错误!!! 严重错误!!! 返回 2 时说明有电机存在严重故障码(RuiwoErrorCode 中大于 128 的故障码)
     * -1: Python接口错误，包括：Python方法不存在或调用失败
     */
    int enable();

    /**
     * @brief 对全部电机执行 Enter Reset State 进入 Reset State 运行模式
     *
     * @note: enter reset state 可用于清除故障码（大于 128 的故障码无法清除）
     * @return int 成功返回 0，失败返回其他错误码
     * 1: 通信问题，包括：CAN 消息发送/接收失败或者超时
     * 2: 严重错误!!! 严重错误!!! 返回 2 时说明有电机存在严重故障码(RuiwoErrorCode 中大于 128 的故障码)
     * -1: Python接口错误，包括：Python方法不存在或调用失败
     */
    int disable();

    bool disableMotor(int motorIndex);
    void close();
    void join();
    // void set_positions(const std::vector<uint8_t> &ids, const std::vector<double> &positions ,std::vector<double> vel ,std::vector<double> pos_kp ,std::vector<double> pos_kd,std::vector<double> torque);
    void set_positions(const std::vector<uint8_t> &ids, const std::vector<double> &positions,const std::vector<double> &torque,const std::vector<double> &velocity);
    void set_torque(const std::vector<uint8_t> &ids, const std::vector<double> &torque);
    void set_velocity(const std::vector<uint8_t> &ids, const std::vector<double> &velocity);
    void saveZeroPosition();
    void saveAsZeroPosition();
    void changeEncoderZeroRound(int index, double direction);
    void adjustZeroPosition(int index, double offset);
    std::vector<double> getMotorZeroPoints();
    void set_teach_pendant_mode(int mode_);
    bool check_motor_list_state();
    void set_joint_gains(const std::vector<int> &joint_indices, const std::vector<double> &kp_pos, const std::vector<double> &kd_pos);
    std::vector<std::vector<double>> get_joint_gains(const std::vector<int> &joint_indices);
    std::vector<double> get_positions();
    std::vector<double> get_torque();
    std::vector<double> get_velocity();
    std::vector<std::vector<double>> get_joint_state();
    std::vector<int> disable_joint_ids;
    MotorStateDataVec get_motor_state();
    bool getMultModeConfig(const YAML::Node &config);
    std::string getHomePath();
};

#endif // RUIWO_ACTUATOR_H
