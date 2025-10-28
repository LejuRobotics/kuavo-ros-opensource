// leg_breakin.cpp
#include <cstdint>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/gil.h>
#include <iostream>
#include <vector>
#include <string>
#include "EcDemoApp.h"
#include "EcSdo.h"
#include "EcDemoPlatform.h"
#include "EcLogFile.h"
#include "EcMotor.h"
#include "UserAppWorkpd.h"
#include "util.h"


namespace py = pybind11;

// 全局变量
static std::vector<std::string> g_command_args;

static void py_set_command_args(const std::vector<std::string>& args) {
    g_command_args = args;
}

static int init_ec_master(const std::vector<std::string>& args) {
    std::vector<char*> argv;
    argv.reserve(args.size());
    for (const auto& arg : args) {
        argv.push_back(const_cast<char*>(arg.c_str()));
    }

    py::gil_scoped_release release;
    return ::Ec_Master_init(argv.size(), argv.data());
}

/**
 * 读取单个SDO
 * @return 返回std::pair(success, value)
 */
static std::pair<bool, int32_t> py_readSingleSdo(uint8_t slaveId, uint16_t obIndex, uint16_t subIndex)
{
    setSdoStatus(SDO_STATUS_READ_SINGLE);
    SdoRead.SlaveId = slaveId;
    SdoRead.ObIndex = obIndex;
    SdoRead.SubIndex = subIndex;

    int res = init_ec_master(g_command_args);
    if (res != 0) {
        std::cerr << "EC Master initialization failed with error code: " << res << std::endl;
        return std::make_pair(0, 0);
    }

    return std::make_pair(Sdo_RW_Res, SdoRead.read_data);
}

/**
 * 写入单个SDO
 * @return 成功返回true，失败返回false
 */
static bool py_writeSingleSdo(uint8_t slaveId, uint16_t obIndex, uint16_t subIndex, int32_t writeData, bool save)
{
    setSdoStatus(SDO_STATUS_WRITE_SINGLE);
    SdoWrite.SlaveId = slaveId;
    SdoWrite.ObIndex = obIndex;
    SdoWrite.SubIndex = subIndex;
    SdoWrite.write_data = writeData;
    SdoWrite.save = save;

    int res = init_ec_master(g_command_args);
    if (res != 0) {
        std::cerr << "EC Master initialization failed with error code: " << res << std::endl;
        return false;
    }

    return ::Sdo_RW_Res;
}


static bool py_robotSetAllConfig(const char *conJsonPath) {
    setSdoStatus(SDO_STATUS_WRITE_ALL);

    g_conJsonPath = conJsonPath;

    int res = init_ec_master(g_command_args);
    if (res != 0) {
        std::cerr << "EC Master initialization failed with error code: " << res << std::endl;
        return false;
    }

    return ::Sdo_RW_Res;
}

static bool py_checkAllDriveConfig(const char *conJsonPath)
{
    setSdoStatus(SDO_STATUS_READ_ALL);

    int res = init_ec_master(g_command_args);
    if (res != 0) {
        std::cerr << "EC Master initialization failed with error code: " << res << std::endl;
        return false;
    }

    if (real_config_parameter.empty())
    {
        return false;
    }

    // printf("\n=== get real config 读取配置参数 ===\n");
    // size_t real_size = real_config_parameter.size();
    // printf("real_config_parameter.size() = %ld\n",real_size);
    // for (size_t i = 0; i < real_size; i++) {
    //     const auto& param = real_config_parameter[i];
    //     printf("\n驱动器 %zu (%s):\n", i + 1, param.joint_name.c_str());
    //     printf("  Product Code: 0x%x\n", param.product_code);
    //     printf("  Motor Parameter Code: %d\n", param.motorParameterCode);
    //     printf("  Torque Mode Velocity Limit: %u\n", param.TorqueModeLimitOfVelocity);
    //     printf("  Driver Rating Count: %u A\n", param.driverRatingCount);
    //     printf("  Driver Work Mode: %u\n", param.driverWorkMode);
    //     printf("  Joint CSP KP: %.1f\n", param.joint_csp_kp / 10.0f);
    //     printf("  Joint CSV KP: %.1f\n", param.joint_csv_kp / 1000.0f);
    //     printf("  Joint CSV KI: %u\n", param.joint_csv_ki);
    //     printf("  Joint CSP Offset: %u\n", param.joint_csp_offset);
    //     printf("  Joint CSP Command Filter: %u\n", param.joint_csp_command_filter);
    //     printf("  Encoder Feedback Mode: %u\n", param.EncoderFeedbackMode);
    // }
    // printf("=== 配置打印完成 ===\n\n");

    // 读取驱动参数并保存
    const char *relPath = "./config/sdo_config/Driver_Real.json";
    saveStructToJson(real_config_parameter, relPath);

    // 读取json配置
    std::vector<EC_DriversConfigParameter_t> json_config_parameter = getJsonDriverConfigParameter(conJsonPath);

    // 对比json配置
    std::vector<ConfigMismatch> mismatches = validateDriverConfigurations(json_config_parameter, real_config_parameter);
    printValidationResults(mismatches);

    return true;
}

// 电机自学习
static bool py_motorSelfLearning(uint8_t slave_id, int32_t learn_mode)
{
    setSdoStatus(SDO_STATUS_SELF_LEARN);
    SdoWrite.SlaveId = slave_id;
    SdoWrite.write_data = learn_mode;

    int res = init_ec_master(g_command_args);
    if (res != 0) {
        std::cerr << "EC Master initialization failed with error code: " << res << std::endl;
        return false;
    }

    return ::Sdo_RW_Res;
}

static bool py_checkAllErrorCode(void)
{
    setSdoStatus(SDO_STATUS_READ_ERROR_CODE);

    int res = init_ec_master(g_command_args);
    if (res != 0) {
        std::cerr << "EC Master initialization failed with error code: " << res << std::endl;
        return false;
    }

    return ::Sdo_RW_Res;
}

static bool py_readErrorFromLog(void)
{
    std::string logFilePath = "/home/lab/Gx/ec_master_tools/EC_log/EC_log_err0.000.log";
    std::vector<EC_T_WKCERR_DESC> error_list;
    readWkcErrorFromLog(logFilePath, error_list);
    readRedundancyLineFromLog(logFilePath);

    return true;
}

static double get_dt_from_args(const std::vector<std::string>& args) {
    for (size_t i = 0; i + 1 < args.size(); ++i) {
        if (args[i] == "-auxclk") {
            int auxclk_us = std::stoi(args[i + 1]);
            return auxclk_us / 1e6;
        }
    }
    return 0.0005;
}

// CSP正弦运动 - 支持单电机控制
static bool py_MotorCspSin(uint32_t num, double A, double T, double time_total)
{
    MotorCspSinParam_t param;
    param.A = A;
    param.T = T;
    param.dt = get_dt_from_args(g_command_args);
    param.time_total = time_total;

    // 根据num参数决定控制哪些电机
    if (num == 1) {
        // 只控制腰部电机 (ID 1)
        uint16_t waist_id[1] = {1};
        addMotorSinTask(waist_id, 1, param);
    } else if (num == 6) {
        // 控制左脚电机 (ID 1-6) - 保持原有功能
        uint16_t left_leg_ids[6] = {1, 2, 3, 4, 5, 6};
        addMotorSinTask(left_leg_ids, 6, param);
    } else {
        std::cerr << "Error: num must be 1 (waist only) or 6 (left leg)" << std::endl;
        return false;
    }

    int res = init_ec_master(g_command_args);
    if (res != 0) {
        std::cerr << "EC Master initialization failed with error code: " << res << std::endl;
        return false;
    }
    return true;
}

// 单电机动作帧控制 - 基于初始位置的偏移值
static bool py_MotorActionFrame(uint16_t motor_id, const std::vector<double>& actions, double motion_duration, double total_time)
{
    if (actions.empty()) {
        std::cerr << "Error: actions vector is empty" << std::endl;
        return false;
    }

    if (actions.size() > 32) {
        std::cerr << "Error: too many actions, maximum 32 allowed" << std::endl;
        return false;
    }

    MotorActionFrameParam_t param;
    param.action_count = static_cast<uint32_t>(actions.size());
    param.motion_duration = motion_duration;
    param.total_time = total_time;
    param.dt = get_dt_from_args(g_command_args);

    // 复制动作数据（偏移值）
    for (size_t i = 0; i < actions.size(); i++) {
        param.actions[i] = actions[i];
    }

    // 只控制指定的单个电机
    uint16_t motor_ids[1] = {motor_id};
    addMotorActionFrameTask(motor_ids, 1, param);

    int res = init_ec_master(g_command_args);
    if (res != 0) {
        std::cerr << "EC Master initialization failed with error code: " << res << std::endl;
        return false;
    }
    return true;
}

// 通用多电机动作帧控制 - 基于初始位置的偏移值
static bool py_MotorMultiActionFrame(const std::vector<uint16_t>& motor_ids, const std::vector<std::vector<double>>& actions_list, double motion_duration, double total_time)
{
    if (motor_ids.empty()) {
        std::cerr << "Error: motor_ids is empty" << std::endl;
        return false;
    }

    if (motor_ids.size() > 13) {
        std::cerr << "Error: maximum 13 motors supported" << std::endl;
        return false;
    }

    if (actions_list.size() != motor_ids.size()) {
        std::cerr << "Error: actions_list size must match motor_ids size" << std::endl;
        return false;
    }

    // 检查每个动作序列
    for (size_t i = 0; i < actions_list.size(); i++) {
        if (actions_list[i].empty()) {
            std::cerr << "Error: actions_list[" << i << "] is empty" << std::endl;
            return false;
        }
        if (actions_list[i].size() > 32) {
            std::cerr << "Error: actions_list[" << i << "] has too many actions, maximum 32 allowed" << std::endl;
            return false;
        }
    }

    // 检查动作序列长度是否一致
    size_t action_count = actions_list[0].size();
    for (size_t i = 1; i < actions_list.size(); i++) {
        if (actions_list[i].size() != action_count) {
            std::cerr << "Error: all action sequences must have the same length" << std::endl;
            return false;
        }
    }

    MotorMultiActionFrameParam_t param;
    param.motor_count = static_cast<uint32_t>(motor_ids.size());
    param.action_count = static_cast<uint32_t>(action_count);
    param.motion_duration = motion_duration;
    param.total_time = total_time;
    param.dt = get_dt_from_args(g_command_args);

    // 复制电机ID
    for (size_t i = 0; i < motor_ids.size(); i++) {
        param.motor_ids[i] = motor_ids[i];
    }

    // 复制动作数据（偏移值）
    for (size_t i = 0; i < motor_ids.size(); i++) {
        for (size_t j = 0; j < action_count; j++) {
            param.actions[i][j] = actions_list[i][j];
        }
    }

    // 控制多个电机
    addMotorMultiActionFrameTask(&param);

    int res = init_ec_master(g_command_args);
    if (res != 0) {
        std::cerr << "EC Master initialization failed with error code: " << res << std::endl;
        return false;
    }
    return true;
}

static bool py_MotorMoveRel(uint16_t id, double deltaPos, double totalTime)
{
    addMotorInterpTask(id, driver_type, deltaPos, totalTime, get_dt_from_args(g_command_args));
    
    int res = init_ec_master(g_command_args);
    if (res != 0) {
        std::cerr << "EC Master initialization failed with error code: " << res << std::endl;
        return false;
    }
    return true;
}

// 左脚电机位置控制 - 根据位置列表运动并往复递增
static bool py_MotorPositionControl(const std::vector<double>& positions, double increment, double holdTime, double moveTime, double totalTime)
{
    if (positions.size() != 6) {
        std::cerr << "Error: positions vector must contain exactly 6 values for left leg motors" << std::endl;
        return false;
    }

    MotorPositionControlParam_t param;
    for (int i = 0; i < 6; i++) {
        param.positions[i] = positions[i];
    }
    param.increment = increment;
    param.hold_time = holdTime;
    param.move_time = moveTime;
    param.total_time = totalTime;
    param.dt = get_dt_from_args(g_command_args);

    // 只控制左脚电机 (ID 1-6)
    uint16_t left_leg_ids[6] = {1, 2, 3, 4, 5, 6};
    addMotorPositionControlTask(left_leg_ids, 6, param);

    int res = init_ec_master(g_command_args);
    if (res != 0) {
        std::cerr << "EC Master initialization failed with error code: " << res << std::endl;
        return false;
    }
    return true;
}

// 左脚电机动作序列控制 - 按动作序列运动
static bool py_MotorActionSequence(const std::vector<std::vector<double>>& actions, double motionDuration, double totalTime)
{
    if (actions.empty()) {
        std::cerr << "Error: actions vector is empty" << std::endl;
        return false;
    }

    if (actions.size() > 32) {
        std::cerr << "Error: too many actions, maximum 32 allowed" << std::endl;
        return false;
    }

    for (size_t i = 0; i < actions.size(); i++) {
        if (actions[i].size() != 6) {
            std::cerr << "Error: action " << i << " must contain exactly 6 values for left leg motors" << std::endl;
            return false;
        }
    }

    MotorActionSequenceParam_t param;
    param.action_count = static_cast<uint32_t>(actions.size());
    param.motion_duration = motionDuration;
    param.total_time = totalTime;
    param.dt = get_dt_from_args(g_command_args);

    // 复制动作数据
    for (size_t i = 0; i < actions.size(); i++) {
        for (int j = 0; j < 6; j++) {
            param.actions[i][j] = actions[i][j];
        }
    }

    // 只控制左脚电机 (ID 1-6)
    uint16_t left_leg_ids[6] = {1, 2, 3, 4, 5, 6};
    addMotorActionSequenceTask(left_leg_ids, 6, param);

    int res = init_ec_master(g_command_args);
    if (res != 0) {
        std::cerr << "EC Master initialization failed with error code: " << res << std::endl;
        return false;
    }
    return true;
}

// 双脚电机动作序列控制 - 支持对称运动
static bool py_MotorDualLegActionSequence(const std::vector<std::vector<double>>& actions, double motionDuration, double totalTime, bool symmetricMotion)
{
    if (actions.empty()) {
        std::cerr << "Error: actions vector is empty" << std::endl;
        return false;
    }

    if (actions.size() > 32) {
        std::cerr << "Error: too many actions, maximum 32 allowed" << std::endl;
        return false;
    }

    for (size_t i = 0; i < actions.size(); i++) {
        if (actions[i].size() != 7) {
            std::cerr << "Error: action " << i << " must contain exactly 7 values [waist, left_leg1, left_leg2, left_leg3, left_leg4, left_leg5, left_leg6]" << std::endl;
            return false;
        }
    }

    MotorDualLegActionSequenceParam_t param;
    param.action_count = static_cast<uint32_t>(actions.size());
    param.motion_duration = motionDuration;
    param.total_time = totalTime;
    param.symmetric_motion = symmetricMotion;
    param.dt = get_dt_from_args(g_command_args);

    // 复制动作数据（7个值：腰部1个 + 左腿6个）
    for (size_t i = 0; i < actions.size(); i++) {
        for (int j = 0; j < 7; j++) {
            param.actions[i][j] = actions[i][j];
        }
    }
    
    // 左腿电机ID: 2-7, 右腿电机ID: 8-13 (腰部电机ID: 1)
    uint16_t left_leg_ids[6] = {2, 3, 4, 5, 6, 7};
    uint16_t right_leg_ids[6] = {8, 9, 10, 11, 12, 13};
    
    addMotorDualLegActionSequenceTask(left_leg_ids, right_leg_ids, 6, param);
    
    int res = init_ec_master(g_command_args);
    if (res != 0) {
        std::cerr << "EC Master initialization failed with error code: " << res << std::endl;
        return false;
    }
    
    return true;
}

// 加载电机零点值
static bool py_loadMotorOffset()
{
    bool success = loadOffset();
    if (success) {
        std::cout << "Successfully loaded motor offset values from CSV file" << std::endl;
        // 打印所有13个电机的零点值（腰部1个 + 左腿6个 + 右腿6个）
        std::cout << "Motor 1 offset: " << pos_offset[0] << std::endl;  // 腰部
        for (int i = 1; i <= 6; i++) {
            std::cout << "Motor " << (i+1) << " offset: " << pos_offset[i] << std::endl;  // 左腿
        }
        for (int i = 7; i <= 12; i++) {
            std::cout << "Motor " << (i+1) << " offset: " << pos_offset[i] << std::endl;  // 右腿
        }
    } else {
        std::cerr << "Failed to load motor offset values from CSV file" << std::endl;
    }
    return success;
}

// 获取电机零点值
static std::vector<double> py_getMotorOffsets()
{
    std::vector<double> offsets;
    for (int i = 0; i < 30; i++) {  // 最多30个电机（NUM_SLAVE_MAX）
        offsets.push_back(pos_offset[i]);
    }
    return offsets;
}

/**
 * 初始化EC Master
 * @return 初始化结果代码
 */
static int py_Ec_Master_init(const std::vector<std::string>& args)
{
    try {
        int argc = static_cast<int>(args.size());
        std::vector<char*> argv(argc);
        for (int i = 0; i < argc; ++i) {
            argv[i] = const_cast<char*>(args[i].c_str());
        }
        py::gil_scoped_release release;
        return ::Ec_Master_init(argc, argv.data());
    } catch (const std::exception& e) {
        throw py::error_already_set();
    }
}

/**
 * 停止EC Master
 */
static void py_EcMasterStop()
{
    try {
        ::EcMasterStop();
    } catch (const std::exception& e) {
        throw py::error_already_set();
    }
}


static bool py_isEcMasterExit()
{
    return ::isEcMasterExit();
}

static void py_setEncoderRange(uint16_t id, uint32_t encoderRange)
{
    setEncoderRange(id, encoderRange);
}

static uint32_t py_getEncoderRange(uint16_t id)
{
    return getEncoderRange(id);
}



// static void py_test()
// {
//     while (true)
//     {
//         std::cout << "test" << std::endl;
//         sleep(1);
//     }
// }

/**
 * EC Master Python绑定模块
 * 提供对EC Master C++库的Python访问接口
 */
PYBIND11_MODULE(leg_breakin, m) {
    m.doc() = "EC Master Python接口 - 提供EtherCAT主站功能";

    // Set command arguments
    m.def("set_command_args", &py_set_command_args,
          py::arg("args"),
          "设置EC Master命令行参数");

    // 读取单个SDO，返回元组 (success, value)
    m.def("readSingleSdo", &py_readSingleSdo,
          py::arg("slaveId"), py::arg("obIndex"), py::arg("subIndex"),
          "读取指定从站的SDO参数: (success, value) = readSingleSdo(slaveId, obIndex, subIndex)");

    // 写入单个SDO，返回成功或失败
    m.def("writeSingleSdo", &py_writeSingleSdo,
          py::arg("slaveId"), py::arg("obIndex"), py::arg("subIndex"), 
          py::arg("writeData"), py::arg("save"),
          "写入指定从站的SDO参数: success = writeSingleSdo(slaveId, obIndex, subIndex, writeData, save)");

    m.def("robotSetAllConfig", &py_robotSetAllConfig, py::arg("conJsonPath"),
          "根据配置文件写入所有驱动器参数: success = robotSetAllConfig(conJsonPath)");

    m.def("checkAllDriveConfig", &py_checkAllDriveConfig, py::arg("conJsonPath"),
          "检查所有驱动器配置参数: success = checkAllDriveConfig(conJsonPath)");

    m.def("motorSelfLearning", &py_motorSelfLearning,
          py::arg("slave_id"),py::arg("learn_mode"),
          "电机自学习: success = motorSelfLearning(slave_id, learn_mode)");

    m.def("checkAllErrorCode", &py_checkAllErrorCode,
          "检查所有驱动器error code: success = checkAllErrorCode()");

    m.def("readErrorFromLog", &py_readErrorFromLog,
          "从log文件读取cyclic command: working counter error 和 redundancy line: success = readErrorFromLog()");

    m.def("MotorCspSin", &py_MotorCspSin,
          py::arg("num"), py::arg("A"), py::arg("T"), py::arg("time_total"),
          "CSP正弦运动: success = MotorCspSin(num, A, T, time_total)");

    m.def("MotorActionFrame", &py_MotorActionFrame,
          py::arg("motor_id"), py::arg("actions"), py::arg("motion_duration"), py::arg("total_time"),
          "单电机动作帧控制: success = MotorActionFrame(motor_id, actions, motion_duration, total_time)");

    m.def("MotorMultiActionFrame", &py_MotorMultiActionFrame,
          py::arg("motor_ids"), py::arg("actions_list"), py::arg("motion_duration"), py::arg("total_time"),
          "多电机动作帧控制: success = MotorMultiActionFrame(motor_ids, actions_list, motion_duration, total_time)");

    m.def("MotorMoveRel", &py_MotorMoveRel,
          py::arg("id"), py::arg("deltaPos"), py::arg("totalTime"),
          "电机转动相对角度: success = MotorMoveRel(id, deltaPos, totalTime)");

    m.def("MotorPositionControl", &py_MotorPositionControl,
          py::arg("positions"), py::arg("increment"), py::arg("holdTime"), py::arg("moveTime"), py::arg("totalTime"),
          "左脚电机位置控制: success = MotorPositionControl(positions, increment, holdTime, moveTime, totalTime)");

    m.def("MotorActionSequence", &py_MotorActionSequence,
          py::arg("actions"), py::arg("motionDuration"), py::arg("totalTime"),
          "左脚电机动作序列控制: success = MotorActionSequence(actions, motionDuration, totalTime)");

    m.def("MotorDualLegActionSequence", &py_MotorDualLegActionSequence,
          py::arg("actions"), py::arg("motionDuration"), py::arg("totalTime"), py::arg("symmetricMotion"),
          "双脚电机动作序列控制: success = MotorDualLegActionSequence(actions, motionDuration, totalTime, symmetricMotion)");

    m.def("loadMotorOffset", &py_loadMotorOffset,
          "加载电机零点值: success = loadMotorOffset()");

    m.def("getMotorOffsets", &py_getMotorOffsets,
          "获取电机零点值: offsets = getMotorOffsets()");

    m.def("setEncoderRange", &py_setEncoderRange,
          py::arg("id"), py::arg("encoderRange"),
          "设置编码器范围: setEncoderRange(id, encoderRange)");

    m.def("getEncoderRange", &py_getEncoderRange,
          py::arg("id"),
          "获取编码器范围: getEncoderRange(id)");

    // m.def("test", &py_test,
    //       "测试函数");

    // // 初始化EC Master
    // m.def("Ec_Master_init", &py_Ec_Master_init,
    //       py::arg("args"),
    //       "初始化EC Master: result = Ec_Master_init(args)");

    // // 停止EC Master
    // m.def("EcMasterStop", &py_EcMasterStop,
    //       "停止EC Master");

    // m.def("isEcMasterExit", &py_isEcMasterExit,
    //       "检查EC Master是否退出");

    py::enum_<EcMasterType>(m, "EcMasterType")
        .value("ELMO", ELMO)
        .value("YD", YD)
        .value("LEJU", LEJU);
}

