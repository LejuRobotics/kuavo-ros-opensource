#include "motorevo/motor_def.h"
#include <algorithm>

namespace motorevo {

template<uint8_t bit_num>
uint32_t float_to_int(float float_num, float min_num, float max_num) {
    float clamped_float = std::clamp(float_num, min_num, max_num);
    uint32_t max_int_value = (1 << bit_num) - 1;
    return (uint32_t)((clamped_float - min_num) / (max_num - min_num) * max_int_value);
}

template<uint8_t bit_num>
float int_to_float(uint32_t int_num, float min_num, float max_num) {
    uint32_t max_int_value = (1 << bit_num) - 1;
    uint32_t clamped_int = std::clamp(int_num, 0u, max_int_value);
    return ((float)clamped_int / max_int_value) * (max_num - min_num) + min_num;
}

// 显式实例化常用位数的模板
template uint32_t float_to_int<8>(float float_num, float min_num, float max_num);
template uint32_t float_to_int<12>(float float_num, float min_num, float max_num);
template uint32_t float_to_int<16>(float float_num, float min_num, float max_num);
template float int_to_float<8>(uint32_t int_num, float min_num, float max_num);
template float int_to_float<12>(uint32_t int_num, float min_num, float max_num);
template float int_to_float<16>(uint32_t int_num, float min_num, float max_num);


const char* to_string(MotorMode mode)
{
    switch (mode)
    {
    case MotorMode::None :
        return "None";
    case MotorMode::ServoMode :
    return "ServoMode";
    case MotorMode::Torque4Mode :
    return "Torque4Mode";
    case MotorMode::TorqueMode :
    return "TorqueMode";
    case MotorMode::TorquePositionMixControlMode :
    return "TorquePositionMixControlMode";
    case MotorMode::VelocityMode :
    return "VelocityMode";
    default:
        return "Unknown";
    }

    return "Unknown";
}

MotorMode  to_motor_mode(const std::string& mode_str)
{
    if( mode_str.empty()) return MotorMode::None;

    std::string str = mode_str;
    // tolower
    std::transform(str.begin(), str.end(), str.begin(),
                   [](unsigned char c){ return std::tolower(c); });

    if(str == "ptm") return MotorMode::TorquePositionMixControlMode;
    else if (str == "servo") return MotorMode::ServoMode;
    else if (str == "torque4") return MotorMode::Torque4Mode;
    else if (str== "torque") return MotorMode::TorqueMode;
    else if (str == "velocity") return MotorMode::VelocityMode;
    return MotorMode::None;
}

std::string MotorErrCode2string(MotorErrCode errcode) {
    switch (errcode) {
        case MotorErrCode::NO_FAULT:
            return "无故障";
        case MotorErrCode::DC_BUS_OVER_VOLTAGE:
            return "直流母线电压过压";
        case MotorErrCode::DC_BUS_UNDER_VOLTAGE:
            return "直流母线电压欠压";
        case MotorErrCode::ENCODER_ANGLE_FAULT:
            return "编码器电角度故障";
        case MotorErrCode::DRV_DRIVER_FAULT:
            return "DRV驱动器故障";
        case MotorErrCode::DC_BUS_CURRENT_OVERLOAD:
            return "直流母线电流过流";
        case MotorErrCode::MOTOR_A_PHASE_CURRENT_OVERLOAD:
            return "电机A相电流过载";
        case MotorErrCode::MOTOR_B_PHASE_CURRENT_OVERLOAD:
            return "电机B相电流过载";
        case MotorErrCode::MOTOR_C_PHASE_CURRENT_OVERLOAD:
            return "电机C相电流过载";
        case MotorErrCode::DRIVER_BOARD_OVERHEAT:
            return "驱劝板温度过高";
        case MotorErrCode::MOTOR_WINDING_OVERHEAT:
            return "电机线圈过温";
        case MotorErrCode::ENCODER_FAILURE:
            return "编码器故障";
        case MotorErrCode::CURRENT_SENSOR_FAILURE:
            return "电流传感器故障";
        case MotorErrCode::OUTPUT_ANGLE_OUT_OF_RANGE:
            return "输出轴实际角度超过通信范围";
        case MotorErrCode::OUTPUT_SPEED_OUT_OF_RANGE:
            return "输出轴速度超过通信范围";
        case MotorErrCode::STUCK_PROTECTION:
            return "堵转保护：电机电枢电流(Iq)大于 Stuck Current，同时电机速度小于 StuckVelocity，持续时间超过 Stuck Time 后触发";
        case MotorErrCode::CAN_COMMUNICATION_LOSS:
            return "CAN通讯丢失：超过CAN通信超时时间未收到数据帧";
        case MotorErrCode::ABS_ENCODER_OFFSET_VERIFICATION_FAILURE:
            return "离轴/对心多圈绝对值编码器接口帧头校验失败，重启，若还失败则请联系售后工程师";
        case MotorErrCode::ABSOLUTE_ENCODER_MULTI_TURN_FAILURE:
            return "对心多圈绝对值编码器多圈接口故障，重启，若还失败则请联系售后工程师";
        case MotorErrCode::ABSOLUTE_ENCODER_EXTERNAL_INPUT_FAILURE:
            return "对心多圈绝对值编码器外部输入故障，重启，若还失败则请联系售后工程师";
        case MotorErrCode::ABSOLUTE_ENCODER_SYSTEM_ANOMALY:
            return "对心多圈绝对值编码器读值故障，重启，若还失败则请联系售后工程师";
        case MotorErrCode::ERR_OFFS:
            return "对心多圈绝对值编码器ERR_OFFS，重启，若还失败则请联系售后工程师";
        case MotorErrCode::ERR_CFG:
            return "对心多圈绝对值编码器ERR_CFG，重启，若还失败则请联系售后工程师";
        case MotorErrCode::ILLEGAL_FIRMWARE_DETECTED:
            return "检测到非法固件，重启，若还失败则请联系售后工程师";
        case MotorErrCode::INTEGRATED_STATOR_DRIVER_DAMAGED:
            return "集成式栅极驱动器初始化失败，重启，若还失败则请联系售后工程师";
        default:
            return "未知故障码: " + std::to_string(static_cast<int>(errcode));
    }
}

} // namespace motorevo