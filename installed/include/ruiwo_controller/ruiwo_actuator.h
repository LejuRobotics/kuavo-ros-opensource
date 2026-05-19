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
#include "ruiwo_actuator_base.h"
static PyObject *RuiWo_pJoinMethod;// 用于将python线程移动到c++线程中,避免GIL占用


/**
 * @brief 将RuiwoErrCode枚举值转换为可读的字符串描述
 *
 * @param[in] errcode RuiwoErrCode枚举值，表示具体的电机故障码
 * @return std::string 故障码的中文描述
 */
std::string RuiwoErrCode2string(RuiwoErrCode errcode);

class RuiWoActuator : public RuiwoActuatorBase{
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
    PyObject *pSetZeroOffsetAdjustmentsMethod{nullptr};
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
    int initialize() override;

    /**
     * @brief 使能所有电机，对全部电机执行 Enter Motor State 进入 Motor State 运行模式
     *
     * @return int 成功返回 0，失败返回其他错误码
     * 1: 通信问题，包括：CAN 消息发送/接收失败或者超时
     * 2: 严重错误!!! 严重错误!!! 返回 2 时说明有电机存在严重故障码(RuiwoErrorCode 中大于 128 的故障码)
     * -1: Python接口错误，包括：Python方法不存在或调用失败
     */
    int enable() override;

    /**
     * @brief 对全部电机执行 Enter Reset State 进入 Reset State 运行模式
     *
     * @note: enter reset state 可用于清除故障码（大于 128 的故障码无法清除）
     * @return int 成功返回 0，失败返回其他错误码
     * 1: 通信问题，包括：CAN 消息发送/接收失败或者超时
     * 2: 严重错误!!! 严重错误!!! 返回 2 时说明有电机存在严重故障码(RuiwoErrorCode 中大于 128 的故障码)
     * -1: Python接口错误，包括：Python方法不存在或调用失败
     */
    int disable() override;

    bool disableMotor(int motorIndex) override;
    void close() override;
    void join();
    // void set_positions(const std::vector<uint8_t> &ids, const std::vector<double> &positions ,std::vector<double> vel ,std::vector<double> pos_kp ,std::vector<double> pos_kd,std::vector<double> torque);
    void set_positions(const std::vector<uint8_t> &ids, const std::vector<double> &positions,const std::vector<double> &torque,const std::vector<double> &velocity,
                       const std::vector<double> &kp = {}, const std::vector<double> &kd = {}) override;
    void set_torque(const std::vector<uint8_t> &ids, const std::vector<double> &torque) override;
    void set_velocity(const std::vector<uint8_t> &ids, const std::vector<double> &velocity) override;
    void saveZeroPosition() override;
    void saveAsZeroPosition() override;
    void changeEncoderZeroRound(int index, double direction) override;
    void adjustZeroPosition(int index, double offset) override;
    std::vector<double> getMotorZeroPoints() override;
    void setZeroOffsetAdjustments(const std::map<size_t, double>& zero_offset_adjustments) override;
    void set_teach_pendant_mode(int mode_) override;
    bool check_motor_list_state();
    void set_joint_gains(const std::vector<int> &joint_indices, const std::vector<double> &kp_pos, const std::vector<double> &kd_pos) override;
    std::vector<std::vector<double>> get_joint_gains(const std::vector<int> &joint_indices) override;
    std::vector<double> get_positions() override;
    std::vector<double> get_torque() override;
    std::vector<double> get_velocity() override;
    std::vector<std::vector<double>> get_joint_state();
    std::vector<int> disable_joint_ids;
    MotorStateDataVec get_motor_state() override;
    bool getMultModeConfig(const YAML::Node &config);
    std::string getHomePath();
};

#endif // RUIWO_ACTUATOR_H
