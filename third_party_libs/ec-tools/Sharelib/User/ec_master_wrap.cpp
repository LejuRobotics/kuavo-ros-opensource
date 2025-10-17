// ec_master_wrap.cpp
#include <cstdint>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>  // 自动转换STL容器
#include <pybind11/gil.h>  // 添加GIL相关头文件
#include <iostream>
#include <vector>
#include <string>
#include "EcDemoApp.h"
#include "EcSdo.h"  // 包含函数声明及相关结构体
#include "EcDemoPlatform.h"
#include "EcLogFile.h"
#include "EcMotor.h"
#include "UserAppWorkpd.h"


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
        std::cerr << "EC 启动失败 " << std::endl;
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
        std::cerr << "EC 启动失败 " << std::endl;
        return false;
    }

    return ::Sdo_RW_Res;
}


static bool py_robotSetAllConfig(const char *conJsonPath) {
    setSdoStatus(SDO_STATUS_WRITE_ALL);

    g_conJsonPath = conJsonPath;

    int res = init_ec_master(g_command_args);
    if (res != 0) {
        std::cerr << "EC 启动失败 " << std::endl;
        return false;
    }

    return ::Sdo_RW_Res;
}

static bool py_checkAllDriveConfig(const char *conJsonPath)
{
    setSdoStatus(SDO_STATUS_READ_ALL);

    int res = init_ec_master(g_command_args);
    if (res != 0) {
        std::cerr << "EC 启动失败 " << std::endl;
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
        std::cerr << "EC 启动失败 " << std::endl;
        return false;
    }

    return ::Sdo_RW_Res;
}

static bool py_checkAllErrorCode(void)
{
    setSdoStatus(SDO_STATUS_READ_ERROR_CODE);

    int res = init_ec_master(g_command_args);
    if (res != 0) {
        std::cerr << "EC 启动失败 " << std::endl;
        return false;
    }

    return ::Sdo_RW_Res;
}

static bool py_readErrorFromLog(void)
{
    std::string logFilePath = "./EC_log/EC_log_err0.000.log";
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

// CSP正弦运动
static bool py_MotorCspSin(uint32_t num, double A, double T, double time_total)
{
    MotorCspSinParam_t param;
    param.A = A;
    param.T = T;
    param.dt = get_dt_from_args(g_command_args);
    param.time_total = time_total;

    addMotorSinTask(motor_data::ids, num, param);

    int res = init_ec_master(g_command_args);
    if (res != 0) {
        std::cerr << "EC 启动失败 " << std::endl;
        return false;
    }
    return true;
}

static bool py_MotorMoveRel(uint16_t jointId, double deltaPos, double totalTime)
{
    addMotorInterpTask(jointId, deltaPos, totalTime, get_dt_from_args(g_command_args));
    
    int res = init_ec_master(g_command_args);
    if (res != 0) {
        std::cerr << "EC 启动失败 " << std::endl;
        return false;
    }
    return true;
}

// 多电机动作序列函数
static bool py_MotorMultiAction(const std::vector<uint16_t>& motor_ids, 
                                const std::vector<std::vector<double>>& motor_actions,
                                double motion_duration, 
                                double time_total)
{
    MotorMultiActionParam_t param;
    param.motor_actions = motor_actions;
    param.motion_duration = motion_duration;
    param.time_total = time_total;
    param.dt = get_dt_from_args(g_command_args);
    param.action_sequence_length = motor_actions.empty() ? 0 : motor_actions[0].size();

    addMotorMultiActionTask(motor_ids.data(), motor_ids.size(), param);

    int res = init_ec_master(g_command_args);
    if (res != 0) {
        std::cerr << "EC 启动失败 " << std::endl;
        return false;
    }
    return true;
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

static void py_setSlave2Joint(uint8_t SlaveId, uint8_t JointId)
{
    setSlave2Joint(SlaveId, JointId);
}

static uint8_t py_getSlave2Joint(uint8_t SlaveId)
{
    return getSlave2Joint(SlaveId);
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
PYBIND11_MODULE(ec_master_wrap, m) {
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

    m.def("MotorMoveRel", &py_MotorMoveRel,
          py::arg("jointId"), py::arg("deltaPos"), py::arg("totalTime"),
          "电机转动相对角度: success = MotorMoveRel(jointId, deltaPos, totalTime)");

    m.def("MotorMultiAction", &py_MotorMultiAction,
          py::arg("motor_ids"), py::arg("motor_actions"), py::arg("motion_duration"), py::arg("time_total"),
          "多电机动作序列: success = MotorMultiAction(motor_ids, motor_actions, motion_duration, time_total)");

    m.def("setEncoderRange", &py_setEncoderRange,
          py::arg("id"), py::arg("encoderRange"),
          "设置编码器范围: setEncoderRange(id, encoderRange)");

    m.def("getEncoderRange", &py_getEncoderRange,
          py::arg("id"),
          "获取编码器范围: getEncoderRange(id)");

    m.def("setSlave2Joint", &py_setSlave2Joint,
          py::arg("SlaveId"), py::arg("JointId"),
          "设置从站到关节的映射: setSlave2Joint(SlaveId, JointId)");

    m.def("getSlave2Joint", &py_getSlave2Joint,
          py::arg("SlaveId"),
          "获取从站到关节的映射: getSlave2Joint(SlaveId)");

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

