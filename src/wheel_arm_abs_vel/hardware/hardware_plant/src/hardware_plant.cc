#include "hardware_plant.h"
#include "wheel_arm_abs_vel/hardware/end_effector_settings.h"
#include <algorithm>
#include <iomanip>
#include <unistd.h>
#include <fstream>
#include <mutex>
#include <limits>
#include <sstream>
#include <deque>
#include <numeric>
#include <map>
#include <set>
#include "utils.h"
#include <csignal>
#include <chrono>
#include <sys/ioctl.h>
#include <cstdio>
#include <cmath>

#include "imu_receiver.h"
#include "revo2_hand_controller.h"
#include "lejuclaw_controller.h"
#include "EcDemoApp.h"
#include "ec_firmware_param_check.h"
#include "joint_test_poses.h"
#include "stance_leg_pos_def.h"
#include "ruiwo_actuator.h"
#include "motorevo/motorevo_actuator.h"
#include "canbus_sdk/config_parser.h"
#include <yaml-cpp/yaml.h>
#include <pwd.h>
#define IS_IN_RANGE(value, lower, upper) (((value) >= (lower)) && ((value) <= (upper)))

using namespace eef_controller;
namespace HighlyDynamic
{
#define MOTOR_L_ARM_LONG 17
#define MOTOR_L_ARM_SHORT 18
#define MOTOR_R_ARM_LONG 24
#define MOTOR_R_ARM_SHORT 25
#define JOINT_L_ARM_ROLL 17
#define JOINT_L_ARM_PITCH 18
#define JOINT_R_ARM_ROLL 24
#define JOINT_R_ARM_PITCH 25

// v15版本机器人手臂零点调整相关常量
constexpr double ARM_ZERO_OFFSET_ADJUSTMENT_DEG = 10.0;  // 零点偏移调整角度（度）
constexpr size_t LEFT_ARM_2_JOINT_INDEX = 1;      // Left_joint_arm_2 对应的关节索引
constexpr size_t RIGHT_ARM_2_JOINT_INDEX = 5;     // Right_joint_arm_2 对应的关节索引
constexpr int LEFT_ARM_2_MOTOR_ADDRESS = 0x02;   // Left_joint_arm_2 对应的电机地址
constexpr int RIGHT_ARM_2_MOTOR_ADDRESS = 0x06;  // Right_joint_arm_2 对应的电机地址

// Roban1系列电机回零时的初始目标位置调整相关常量
constexpr float ROBAN1_INITIAL_TARGET_ADJUSTMENT_DEG = 10.0f;  // Roban1系列初始目标位置调整角度（度）
constexpr motorevo::MotorId LEFT_ARM_2_MOTOR_ID = 2;   // 左臂第2号电机ID
constexpr motorevo::MotorId RIGHT_ARM_2_MOTOR_ID = 6;   // 右臂第2号电机ID

const int kEcMasterNilId = 254; // Mask for does not exist ecmaster Id
const int kStatusNotInAnyState = -3; // Robot status that does not belong to any state

namespace
{
// 标定堵转保护参数
constexpr double kCalibrationStallPositionErrorDeg = 2.0;  // 目标与反馈的位置误差阈值 单位度
constexpr double kCalibrationStallPositionChangeDeg = 0.5; // 观察窗口内允许的最大位置变化 单位度
constexpr double kCalibrationStallDurationSec = 0.2;       // 堵转观察窗口 单位秒

// 单个关节的堵转观察状态
struct CalibrationStallState
{
    bool active = false;
    std::chrono::steady_clock::time_point start_time;
    double start_position = 0.0;
};

// 更新堵转观察窗口并返回是否检测到堵转
bool updateCalibrationStallState(CalibrationStallState& state,
                                 const std::chrono::steady_clock::time_point& current_time,
                                 double current_position,
                                 double position_error,
                                 double torque_nm,
                                 double torque_threshold,
                                 double& position_change)
{
    const bool high_load_tracking_error =
        position_error > kCalibrationStallPositionErrorDeg && torque_nm > torque_threshold;
    if (!high_load_tracking_error)
    {
        state.active = false;
        return false;
    }

    if (!state.active)
    {
        state.active = true;
        state.start_time = current_time;
        state.start_position = current_position;
        return false;
    }

    if (std::chrono::duration<double>(current_time - state.start_time).count() <
        kCalibrationStallDurationSec)
    {
        return false;
    }

    position_change = std::abs(current_position - state.start_position);
    if (position_change > kCalibrationStallPositionChangeDeg)
    {
        // 关节仍在产生有效位移时重新开始下一观察窗口
        state.start_time = current_time;
        state.start_position = current_position;
        return false;
    }

    return true;
}
} // namespace

    bool imu_invert{false};
    uint32_t num_joint, num_arm_joints;
    uint32_t num_head_joints = 2, num_waist_joints = 0;
    bool isParallelArm{false};
    Eigen::Vector4d ankle_qv_old;
    uint32_t hand_ankle_nq, hand_ankle_nv;
    Eigen::VectorXd r_hand_ankle_qv, r_hand_ankle_qv_in_left_hand, r_hand_ankle_qv_old; // output qv hand_roll, hand_pitch, r_motor, r_link(2), l_motor, l_link(2)
    Eigen::VectorXd l_hand_ankle_qv, l_hand_ankle_qv_old, defalutJointPos_;
    bool swing_arm_ = false;
    double swing_arm_gain_ = 0.0;
    static ActuatorsInterface_t actuators;
    static Revo2HandControllerPtr revo2_hand_controller = nullptr;
    static RuiwoActuatorBase *ruiwo_actuator = nullptr;
    static LejuClawControllerPtr leju_claw_actuator = nullptr;
    static kuavo_solver::AnkleSolver ankleSolver;
    static bool imu_is_recv;
    static std::mutex mtx_imu;

    std::vector<std::unique_ptr<Kalman>> joint_kf;

    double hard_dt = 1e-3;

    static uint8_t _th_running;
    bool hardware_ready_ = false;

    Eigen::Vector3d pre_accel, pre_gyrosco;
    Eigen::Quaterniond pre_quaternion;
    int8_t use_pre_imu_times;

    ImuType imu_type_ = IMU_TYPE_XSENS;

    inline void convert_uint8_to_uint16(const uint8_t *input, uint16_t *output, size_t length)
    {
        for (size_t i = 0; i < length; i++)
        {
            // 将每个 uint8_t 类型的元素转换为 uint16_t 类型
            output[i] = static_cast<uint16_t>(input[i]);
        }
    }

    bool HardwarePlant::disableMotor(int motorIndex, const std::string& reason)
    {
        // 电机disable
        if (motorIndex <= 12 || 13 == motorIndex || 20 == motorIndex)
        {
            // EC
            // TODO 当前ecmaster disable接口有问题，无法立刻让电机失能，采用0力控方式
            std::cout << "ecmaster motor index, current disable use zero CST mode. " << std::endl; 
        }
        else if (motorIndex > 13 && motorIndex < 20)
        {
            // ruiwo
            //TODO 当前这个接口只在c++ sdk起作用
            std::cout << "diable motor " << motorIndex << std::endl;
            auto ruiwoIndex = motorIndex - 14;
            auto isDisableFlag = ruiwo_actuator->disableMotor(ruiwoIndex);
            if (!isDisableFlag)
            {
                std::cerr << "diable motor " << motorIndex << " failed!" << std::endl;
                return false;
            }
        }
        else if (motorIndex > 20 && motorIndex < 28)
        {
            // ruiwo
            std::cout << "diable motor " << motorIndex << std::endl;
            auto ruiwoIndex = motorIndex - 15;
            auto isDisableFlag = ruiwo_actuator->disableMotor(ruiwoIndex);
            if (!isDisableFlag)
            {
                std::cerr << "diable motor " << motorIndex << "failed!" << std::endl;
                return false;
            }
        }
        else
        {
            std::cerr << "invalue motor index: " << motorIndex << std::endl;
            return false;
        }

        // 统一在这里做电机状态管理
        markJointAsDisabled(motorIndex, reason);

        return true;
    }

    void HardwarePlant::setEcmasterDriverType(std::string type)
    {
        if (type != "elmo" && type != "youda" && type != "lunbi" && type != "youda1" && type != "leju" && type != "youda3")
        {
            std::cerr << "Unsupported ecmaster driver type: " << type << std::endl;
            exit(1);
        }
        std::cout << "set ecmaster driver type to: " << type << std::endl;
        size_t driver_type_length = sizeof(driver_type) / sizeof(driver_type[0]);
        for (size_t i = 0; i < driver_type_length; i++)
        {
            if (type == "elmo")
            {
                driver_type[i] = EcMasterType::ELMO;
            }
            else if (type == "lunbi")
            {
                if(i < 4)
                {
                    driver_type[i] = EcMasterType::ELMO;
                }
                else
                {
                    driver_type[i] = EcMasterType::YD;
                }
            }
            
            else if (type == "youda3" || type == "youda" || type == "youda1")
            {
                driver_type[i] = EcMasterType::YD;
            }
            else if (type == "leju")
            {
                driver_type[i] = EcMasterType::LEJU;
            }
        }
    }

    void HardwarePlant::GetMotorData(const std::vector<uint8_t> &joint_ids, std::vector<MotorParam_t> &motor_data)
    {
        // Classify IDs based on their types
        std::vector<uint8_t> ecMasterIds;
        std::vector<uint8_t> ecExistIds;
        std::vector<uint8_t> ruiwoIds;
        std::vector<MotorParam_t> EC_joint_data;
        RuiWoJointData rw_joint_data;
        // Add more vectors for other types if needed...

        for (size_t i = 0; i < joint_ids.size(); ++i)
        {
            size_t index = joint_ids[i] - 1;
            // Check the driver type and add to the appropriate vector
            if (motor_info.driver[index] == EC_MASTER)
            {
                ecMasterIds.push_back(ec_index_map_[index]);
                // EC_joint_data.push_back(motor_data.at(i));
                if (motor_info.motors_exist[i]) {
                    ecExistIds.push_back(ec_index_map_[index]);
                }
            }
            else if (motor_info.driver[index] == RUIWO)
            {
                ruiwoIds.push_back(ruiwoIds.size());
                rw_joint_data.pos.push_back(motor_data.at(i).position);
                rw_joint_data.vel.push_back(motor_data.at(i).velocity);
                rw_joint_data.torque.push_back(motor_data.at(i).torque);
            }

            // Add more conditions for other types if needed...
        }
        if (ecMasterIds.size())
        {
            EC_joint_data.resize(ecMasterIds.size());
            std::vector<JointParam_t> exsit_ec_joint_data(ecExistIds.size());
            std::vector<uint16_t> ecMasterIds16(ecExistIds.size());
            convert_uint8_to_uint16(ecExistIds.data(), ecMasterIds16.data(), ecExistIds.size());
            actuators.getJointData(ecMasterIds16.data(), driver_type, ecExistIds.size(), exsit_ec_joint_data.data());
            
            int ec_exist_index = 0;


            for (size_t i = 0; i < EC_joint_data.size(); i++)
            {
                if (ecMasterIds[i] != kEcMasterNilId)
                {
                    EC_joint_data[i] = exsit_ec_joint_data.at(ec_exist_index++);
                    /* only-half-up-body mode: always keep stance position. */
                    if (stance_leg_joint_pos_ && i < stance_leg_joint_pos_->size() && motor_info.motors_disable[i]) {
                        EC_joint_data[i].position = stance_leg_joint_pos_->at(i);
                    }
                }
                else {
                    // !!! Motor does not exist, set all values default.
                    EC_joint_data[i] = JointParam_t();
                    /* only-half-up-body mode: always keep stance position. */
                    if(stance_leg_joint_pos_ && i < stance_leg_joint_pos_->size()) { 
                        EC_joint_data[i].position = stance_leg_joint_pos_->at(i);
                    }
                }
            }

            // convert_uint8_to_uint16(ecMasterIds.data(), ecMasterIds16.data(), ecMasterIds.size());
            // actuators.getJointData(ecMasterIds16.data(), driver_type, ecMasterIds.size(), EC_joint_data.data());
        }
        if (ruiwoIds.size())
        {
            if(nullptr != ruiwo_actuator){
                
            // 获取到弧度值
            rw_joint_data.pos = ruiwo_actuator->get_positions();
            rw_joint_data.vel = ruiwo_actuator->get_velocity();
            rw_joint_data.torque = ruiwo_actuator->get_torque();
            for (size_t i = 0; i < rw_joint_data.pos.size(); i++)
            {
                rw_joint_data.pos.at(i) = (rw_joint_data.pos.at(i) * 180) / M_PI;
                rw_joint_data.vel.at(i) = (rw_joint_data.vel.at(i) * 180) / M_PI;
                rw_joint_data.torque.at(i) = rw_joint_data.torque.at(i);
            }
            }
        }

        uint8_t num_ec_data = 0;
        uint8_t num_rw_data = 0;
        for (size_t i = 0; i < joint_ids.size(); ++i)
        {
            size_t index = joint_ids[i] - 1;
            // Check the driver type and add to the appropriate vector
            if (motor_info.driver[index] == EC_MASTER)
            {
                // std::cout << ":i:" << i << " EC_joint_data[num_ec_data]:" << EC_joint_data[num_ec_data].position << std::endl;
                motor_data.at(i) = EC_joint_data.at(num_ec_data);
                num_ec_data++;
            }
            else if (motor_info.driver[index] == RUIWO)
            {
                if (rw_joint_data.pos.size() <= num_rw_data) 
                {
                    std::cerr << "[HardwarePlant::GetMotorData] rw_joint_data.pos.size() " << rw_joint_data.pos.size() << " <= num_rw_data " << num_rw_data << std::endl;
                    continue;
                }
                motor_data.at(i).position = rw_joint_data.pos.at(num_rw_data);
                motor_data.at(i).velocity = rw_joint_data.vel.at(num_rw_data);
                motor_data.at(i).torque = rw_joint_data.torque.at(num_rw_data);
                ++num_rw_data;
            }
            // Add more conditions for other types if needed...
        }
    }

    void HardwarePlant::setPrepEcLegGains(const std::vector<double>& kp, const std::vector<double>& kd)
    {
        if (kp.size() == kd.size() && kp.size() == 12) {
            prep_ec_leg_kp_ = kp;
            prep_ec_leg_kd_ = kd;
        } else {
            prep_ec_leg_kp_.clear();
            prep_ec_leg_kd_.clear();
        }
    }

    void HardwarePlant::clearPrepEcLegGains()
    {
        prep_ec_leg_kp_.clear();
        prep_ec_leg_kd_.clear();
    }

    void HardwarePlant::latchCurrentJointsCsp()
    {
        GetMotorData(joint_ids, joint_data);
        for (uint32_t i = 0; i < num_joint && i < joint_cmd.size() && i < joint_data.size(); ++i)
        {
            joint_cmd[i].position = joint_data[i].position;
            joint_cmd[i].maxTorque = motor_info.max_current[i];
        }
        setDefaultKpKd(joint_cmd, joint_ids);
        SetMotorPosition(joint_ids, joint_cmd);
    }

    // 辅助函数：为 EC_MASTER 电机设置默认的 kp/kd（用于 CSP 模式）
    inline void HardwarePlant::setDefaultKpKd(std::vector<MotorParam_t> &motor_data, const std::vector<uint8_t> &joint_ids)
    {
        const auto &running_settings = kuavo_settings_.running_settings;
        const bool use_sit_boot_leg_ec_gains =
            prep_ec_leg_kp_.size() == prep_ec_leg_kd_.size() && prep_ec_leg_kp_.size() == 12;
        const bool use_vr_joint_gains = !use_sit_boot_leg_ec_gains && running_settings.use_vr_arm_kpkd &&
                                        !running_settings.vr_joint_kp.empty() &&
                                        !running_settings.vr_joint_kd.empty() &&
                                        running_settings.vr_joint_kp.size() == running_settings.vr_joint_kd.size();
        const bool use_vr_ruiwo_gains = running_settings.use_vr_arm_kpkd &&
                                        !running_settings.vr_ruiwo_kp.empty() &&
                                        !running_settings.vr_ruiwo_kd.empty() &&
                                        running_settings.vr_ruiwo_kp.size() == running_settings.vr_ruiwo_kd.size();
        const auto &default_joint_kp =
            use_vr_joint_gains ? running_settings.vr_joint_kp : running_settings.joint_kp;
        const auto &default_joint_kd =
            use_vr_joint_gains ? running_settings.vr_joint_kd : running_settings.joint_kd;
        const auto &default_ruiwo_kp = use_vr_ruiwo_gains ? running_settings.vr_ruiwo_kp : running_settings.ruiwo_kp;
        const auto &default_ruiwo_kd = use_vr_ruiwo_gains ? running_settings.vr_ruiwo_kd : running_settings.ruiwo_kd;

        int ec_count = 0;
        int ruiwo_count = 0;

        for (size_t i = 0; i < joint_ids.size() && i < motor_data.size(); ++i)
        {
            size_t index = joint_ids[i] - 1;
            if (index >= motor_info.driver.size()) continue;

            if (motor_info.driver[index] == EC_MASTER && motor_info.motors_exist[index])
            {
                if (use_sit_boot_leg_ec_gains && ec_count < static_cast<int>(prep_ec_leg_kp_.size()))
                {
                    motor_data[i].kp = prep_ec_leg_kp_[ec_count];
                    motor_data[i].kd = prep_ec_leg_kd_[ec_count];
                }
                else if (ec_count < static_cast<int>(default_joint_kp.size()) &&
                         ec_count < static_cast<int>(default_joint_kd.size()))
                {
                    motor_data[i].kp = static_cast<double>(default_joint_kp[ec_count]);
                    motor_data[i].kd = static_cast<double>(default_joint_kd[ec_count]);
                }
                ec_count++;
            }
            else if (motor_info.driver[index] == RUIWO)
            {
                if (ruiwo_count < static_cast<int>(default_ruiwo_kp.size()) &&
                    ruiwo_count < static_cast<int>(default_ruiwo_kd.size()))
                {
                    motor_data[i].kp = default_ruiwo_kp[ruiwo_count];
                    motor_data[i].kd = default_ruiwo_kd[ruiwo_count];
                }
                ruiwo_count++;
            }
        }
    }

    void HardwarePlant::SetMotorPosition(const std::vector<uint8_t> &joint_ids, std::vector<MotorParam_t> &motor_data)
    {
        // Classify IDs based on their types
        std::vector<uint8_t> ecMasterIds;
        std::vector<uint8_t> ruiwoIds;
        std::vector<MotorParam_t> EC_joint_data;
        RuiWoJointData rw_joint_data;
        // Add more vectors for other types if needed...
        for (size_t i = 0; i < joint_ids.size(); ++i)
        {
            size_t index = joint_ids[i] - 1;
            // Check the driver type and add to the appropriate vector
            if (motor_info.driver[index] == EC_MASTER)
            {
                // Only add the motor that exist 
                if (motor_info.motors_exist[index]) {
                    EC_joint_data.push_back(motor_data.at(i));
                    ecMasterIds.push_back(ec_index_map_[index]);
                }
            }
            else if (motor_info.driver[index] == RUIWO)
            {
                ruiwoIds.push_back(ruiwoIds.size());
                rw_joint_data.pos.push_back(motor_data.at(i).position);
                rw_joint_data.vel.push_back(motor_data.at(i).velocityOffset);
                rw_joint_data.torque.push_back(motor_data.at(i).torqueOffset);
                rw_joint_data.kp.push_back(motor_data.at(i).kp);
                rw_joint_data.kd.push_back(motor_data.at(i).kd);
            }
        }
        if (ecMasterIds.size())
        {
            std::vector<uint16_t> ecMasterIds16(ecMasterIds.size());
            convert_uint8_to_uint16(ecMasterIds.data(), ecMasterIds16.data(), ecMasterIds.size());
            actuators.setJointPosition(ecMasterIds16.data(), driver_type, ecMasterIds.size(), EC_joint_data.data());
        }
        if (ruiwoIds.size())
        {
            // 接口传入的是角度, c++sdk中转为弧度; kp/kd 从 joint_cmd 中实时获取
            ruiwo_actuator->set_positions(ruiwoIds, rw_joint_data.pos, rw_joint_data.torque, rw_joint_data.vel, rw_joint_data.kp, rw_joint_data.kd);
        }
    }

    void HardwarePlant::SetMotorTorque(const std::vector<uint8_t> &joint_ids, std::vector<MotorParam_t> &motor_data)
    {
        // Classify IDs based on their types
        std::vector<uint8_t> ecMasterIds;
        std::vector<MotorParam_t> EC_joint_data;
        // Add more vectors for other types if needed...

        for (size_t i = 0; i < joint_ids.size(); ++i)
        {
            size_t index = joint_ids[i] - 1;
            // Check the driver type and add to the appropriate vector
            if (motor_info.driver[index] == EC_MASTER)
            {
                // Only add the motor that exist 
                if (motor_info.motors_exist[index]) {
                    ecMasterIds.push_back(ec_index_map_[index]);
                    EC_joint_data.push_back(motor_data.at(i));
                }
            }
        }
        if (ecMasterIds.size())
        {   
            std::vector<uint16_t> ecMasterIds16(ecMasterIds.size());
            convert_uint8_to_uint16(ecMasterIds.data(), ecMasterIds16.data(), ecMasterIds.size());
            actuators.setJointTorque(ecMasterIds16.data(), driver_type, ecMasterIds.size(), EC_joint_data.data());
        }
    }

    inline void HardwarePlant::SetMotorVelocity(const std::vector<uint8_t> &joint_ids, std::vector<MotorParam_t> &motor_data)
    {
        // Classify IDs based on their types
        std::vector<uint8_t> ecMasterIds;
        std::vector<MotorParam_t> EC_joint_data;
        // Add more vectors for other types if needed...

        for (size_t i = 0; i < joint_ids.size(); ++i)
        {
            size_t index = joint_ids[i] - 1;
            // Check the driver type and add to the appropriate vector
            if (motor_info.driver[index] == EC_MASTER)
            {
                // if (i>11)
                // std::cout <<"set ec:"<<i <<" |" <<ecMasterIds.size() + 1<<"|"<< joint_data.at(i).position<<std::endl;
                
                // Only add the motor that exist 
                if (motor_info.motors_exist[index]) {
                    ecMasterIds.push_back(ec_index_map_[index]);
                    EC_joint_data.push_back(motor_data.at(i));
                }
            }
        }
        if (ecMasterIds.size())
        {
            std::vector<uint16_t> ecMasterIds16(ecMasterIds.size());
            convert_uint8_to_uint16(ecMasterIds.data(), ecMasterIds16.data(), ecMasterIds.size());
            actuators.setJointVelocity(ecMasterIds16.data(), driver_type, ecMasterIds.size(), EC_joint_data.data());
        }
    }

    static double calcCos(double start, double stop, double T, double t)
    {
        double A = (stop - start) / 2.0;
        return A * -cos(M_PI / T * t) + start + A;
    }

    Eigen::Vector3d removeGravity(const Eigen::Vector3d &rawAccel, const Eigen::Quaterniond &orientation)
    {
        // 设置重力向量在全局坐标系中的方向，假设重力向量的方向为 (0, 0, -9.81)
        Eigen::Vector3d gravity(0.0, 0.0, 9.785); // TODO: 安装方向

        // 将重力向量转换到局部坐标系中
        Eigen::Vector3d localGravity = orientation.conjugate() * gravity;

        // 计算去除重力影响的加速度
        Eigen::Vector3d accelNoGravity = rawAccel - localGravity;

        return accelNoGravity;
    }

    bool HardwarePlant::jointMoveTo(std::vector<double> goal_pos, double speed, double dt, double current_limit)
    {
        if (goal_pos.size() == num_joint - num_head_joints)// 适配旧的服务接口, 补全头部自由度
        {
            std::cout << "[HardwarePlant::jointMoveTo] Warning: The goal position size is " << goal_pos.size() << ", number of joints is " << num_joint << std::endl;
            for (int i = 0; i < num_head_joints; i++)
                goal_pos.push_back(0);
        }

        Eigen::VectorXd cmd_s(goal_pos.size());
        for (size_t i = 0; i < goal_pos.size(); i++)
        {
            cmd_s(i) = goal_pos[i] * TO_RADIAN;
        }
        
        // 注意：ankleSolver 是“脚踝解算器”，会对某些索引（例如 joint_q[3] 当作膝关节）做非负截断。
        // 轮臂模式下 na_foot_ == 4（ID1-4里包含腰部电机），不应走脚踝解算/限幅，否则会把4号电机负值截断成0。
        const bool should_use_ankle_solver = (na_foot_ >= 12);
        if (should_use_ankle_solver)
        {
            Eigen::VectorXd motor_position(na_foot_);

            motor_position = ankleSolver.joint_to_motor_position(cmd_s.head(na_foot_));

            for (size_t  i = 0; i < na_foot_; i++)
            {
                goal_pos[i] = motor_position[i] * TO_DEGREE;
            }
        }
        else
        {
            std::cout << "[DEBUG jointMoveTo] Skip ankleSolver conversion (na_foot_=" << na_foot_
                      << "). Wheel-arm mode keeps raw goal_pos for joints 1-" << na_foot_ << "." << std::endl;
        }

        // 检查并限制目标位置到关节限位范围内
        for (size_t i = 0; i < goal_pos.size() && i < num_joint; i++)
        {
            double original_goal = goal_pos[i];
            bool clamped = false;

            // 检查最大限位
            if (goal_pos[i] > max_joint_position_limits[i])
            {
                goal_pos[i] = max_joint_position_limits[i];
                clamped = true;
            }
            // 检查最小限位
            else if (goal_pos[i] < min_joint_position_limits[i])
            {
                goal_pos[i] = min_joint_position_limits[i];
                clamped = true;
            }

            if (clamped)
            {
                std::cout << "[HardwarePlant::jointMoveTo] 警告：关节 " << i + 1
                          << " 目标位置从 " << original_goal << "° 被限制到 "
                          << goal_pos[i] << "° (限位范围: "
                          << min_joint_position_limits[i] << "° ~ "
                          << max_joint_position_limits[i] << "°)" << std::endl;
            }
        }

        GetMotorData(joint_ids, joint_data);
        usleep(10000);
        std::vector<double> start_pos(num_joint, 0);
        std::vector<double> T(num_joint, 0);
        for (uint32_t i = 0; i < num_joint; i++)
        {
            start_pos[i] = joint_data[i].position;
            T[i] = fabs(goal_pos[i] - start_pos[i]) / speed;
            printf("Joint %d from %f to %f\n", i + 1, start_pos[i], goal_pos[i]);
        }
        double max_T = *max_element(T.begin(), T.end());
        if (max_T < 0.5)
        {
            max_T = 0.5;
        }
        printf("Duration %f\n", max_T);
        uint32_t total_cnt = ceil(max_T / dt);
        uint32_t count = 0;
        // 标定模式下记录手臂关节持续堵转状态
        const int arm_start_idx = num_joint - num_head_joints - num_arm_joints; // 手臂在全身关节数组中的起始索引
        std::vector<CalibrationStallState> stall_states(num_joint);
        struct timespec next_time;
        clock_gettime(CLOCK_MONOTONIC, &next_time);
        while (1)
        {
            int disable_id = getDisableMotorId();
            if(-1 != disable_id)
            {
                disableMotor(disable_id, "jointMoveTo");
            }

            for (uint32_t i = 0; i < num_joint; i++)
            {
                joint_cmd[i].position = calcCos(start_pos[i], goal_pos[i], total_cnt, count);
                if (current_limit > 0)
                {
                    joint_cmd[i].maxTorque = current_limit;
                }
                else
                {
                    joint_cmd[i].maxTorque = motor_info.max_current[i];
                }
            }

            // 检查腿部电机是否被disable
            bool leg_disabled = false;
            for (const auto& jointStatus : getAllJointsStatus()) 
            {
                int joint_id = jointStatus.first;
                auto status = jointStatus.second;
                if (joint_id >= 1 && joint_id <= 12) 
                {
                    if (status == MotorStatus::DISABLED || status == MotorStatus::ERROR) 
                    {
                        leg_disabled = true;
                        std::cout << "joint_id: " << joint_id << ", status: " << (int)status << std::endl;
                        break;
                    }
                }
            }

            if (!leg_disabled) 
            {
                // 在 CSP 模式下，为 EC_MASTER 和 RUIWO 电机设置默认的 kp/kd
                setDefaultKpKd(joint_cmd, joint_ids);
                SetMotorPosition(joint_ids, joint_cmd);

                // 仅在标定模式监测手臂关节
                if (is_cali_)
                {
                    // 每个控制周期刷新位置 速度和力矩反馈
                    GetMotorData(joint_ids, joint_data);
                    const auto current_time = std::chrono::steady_clock::now();
                    for (int i = arm_start_idx; i < arm_start_idx + static_cast<int>(num_arm_joints); ++i)
                    {
                        if (i < 0 || i >= static_cast<int>(num_joint) ||
                            (motor_info.driver[i] != RUIWO && motor_info.driver[i] != EC_MASTER))
                        {
                            continue;
                        }

                        const double position_error = std::abs(joint_cmd[i].position - joint_data[i].position);
                        const double raw_torque = joint_data[i].torque;
                        const double torque_nm = motorFeedbackToJointTorqueNm(i, raw_torque);
                        const double torque_threshold = getCalibrationStallTorqueThreshold(i);
                        double position_change = 0.0;
                        if (updateCalibrationStallState(stall_states[i], current_time,
                                                        joint_data[i].position, position_error,
                                                        torque_nm, torque_threshold, position_change))
                        {
                            std::cerr << "\n\033[31m[CalibrationStallGuard] 关节" << i + 1
                                      << "在标定模式运动时疑似堵转"
                                      << " (target=" << joint_cmd[i].position
                                      << "deg, position=" << joint_data[i].position
                                      << "deg, velocity=" << joint_data[i].velocity
                                      << "deg/s, position_change=" << position_change
                                      << "deg/" << kCalibrationStallDurationSec
                                      << "s, torque=" << torque_nm
                                      << "Nm, threshold=" << torque_threshold
                                      << "Nm, raw_feedback=" << raw_torque
                                      << ")\033[0m" << std::endl;
                            // 立即停止所有手臂关节并终止本次运动
                            if (!stopArmForCalibrationFailure())
                            {
                                std::cerr << "\033[31m[CalibrationStallGuard] 无法确认手臂已安全停止，正在触发紧急停机\033[0m"
                                          << std::endl;
                                std::raise(SIGINT);
                            }
                            else
                            {
                                std::cerr << "\033[31m[CalibrationStallGuard] 手臂已停止\033[0m" << std::endl;
                            }
                            return false;
                        }
                    }
                }
            } 
            else 
            {
                //TODO 因为ec sdk有问题，掉使能并不是每次起作用
                // std::vector<uint8_t> leg_joint_ids;
                // std::vector<JointParam_t> leg_joint_cmd;
                // for (int i = 0; i < num_joint; ++i) 
                // {
                //     int joint_id = joint_ids[i];
                //     if (joint_id >= 1 && joint_id <= 12) 
                //     {
                //         joint_cmd[i].torque = 0;
                //         leg_joint_ids.push_back(joint_id);
                //         leg_joint_cmd.push_back(joint_cmd[i]);
                //     }
                // }
                // SetMotorTorque(leg_joint_ids, leg_joint_cmd);
                std::cerr << "jointMoveTo trigger joint protect." << std::endl;
                std::raise(SIGINT);
            }

            count++;
            if (count > total_cnt)
            {
                OsSleep(4);
                GetMotorData(joint_ids, joint_data);
                break;
            }

            next_time.tv_sec += (next_time.tv_nsec + dt * 1e9) / 1e9;
            next_time.tv_nsec = (int)(next_time.tv_nsec + dt * 1e9) % (int)1e9;
            clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next_time, NULL);
        }
        return true;
    }

    bool HardwarePlant::checkJointSafety(const std::vector<JointParam_t> &joint_data, std::vector<uint8_t> ids, std::string &msg)
    {
        if (hardware_status_ < 0)
        {
            return true;
        }

        if(!motor_status_manager_)
        {
            return true;
        }

        std::string err_msg = "";
        // 使用电机状态管理器进行全面的关节安全检查（包括状态和位置限制）
        bool result = motor_status_manager_->checkAllJointsStatus(joint_data, ids, err_msg);
        msg = err_msg;
        
        // 检查是否需要触发急停
        if (motor_status_manager_->shouldTriggerEmergencyStop()) 
        {
            std::cerr << "[HardwarePlant] Emergency stop triggered due to critical motor errors!" << std::endl;
            std::raise(SIGINT);
        }
        
        // 每10秒输出一次状态摘要
        std::string summary = motor_status_manager_->getStatusSummary(30);
        if (!summary.empty()) 
        {
            std::cout << summary << std::endl;
        }
        
        return result;
    }

    bool HardwarePlant::checkJointPos(JointParam_t *joint_data, std::vector<uint8_t> ids, std::string *msg)
    {
        if (hardware_status_ == -2)
            return true;
        bool result = true;
        std::string err_msg = "";
        for (uint32_t i = 0; i < ids.size(); i++)
        {
            if(ids[i] > 14) continue;
            uint8_t index = ids[i] - 1;
            if (joint_data[i].position < min_joint_position_limits[index] ? true : false)
            {
                err_msg += "Error: Joint " + std::to_string(index + 1) + " position minimum value exceeded, actual value " + std::to_string(joint_data[i].position) + "\n";
                result = false;
            }
            if (joint_data[i].position > max_joint_position_limits[index] ? true : false)
            {
                err_msg += "Error: Joint " + std::to_string(index + 1) + " position maxmum value exceeded, actual value " + std::to_string(joint_data[i].position) + "\n";
                result = false;
            }
            if (std::abs(joint_data[i].velocity) > joint_velocity_limits_[index])
            {
                err_msg += "Error: Joint " + std::to_string(index + 1) + " velocity value exceeded, actual value " + std::to_string(joint_data[i].velocity) + "\n";
                result = false;
            }
            // std::cout << joint_data[i].status << std::endl;
            if (joint_data[i].status != 0)
            {
                err_msg += "Error: Joint " + std::to_string(index + 1) + " joint status error!" + std::to_string(joint_data[i].status) + " \n";
                result = false;
            }
        }
        *msg = err_msg;
        // if (!result)
        //     std::raise(SIGINT);
        // if (!result && KuavoErrorManager.CurrentRobotState() == ROBOT_OK)
        //     KuavoErrorManager.trigger(ERROR_MAJOR, err_msg);

        return result;
    }

    void HardwarePlant::jointFiltering(std::vector<JointParam_t> &joint_data, double dt)
    {
        Eigen::VectorXd measure(3);
        Eigen::VectorXd estimate(3);
        Eigen::MatrixXd F(3, 3);
        F << 1.0, dt, dt * dt / 2.0,
            0.0, 1.0, dt,
            0.0, 0.0, 1.0;
        Eigen::MatrixXd H = Eigen::Matrix3d::Identity();

        for (uint32_t i = 0; i < num_joint; i++)
        {
            measure << joint_data[i].position, joint_data[i].velocity, 0;
            joint_kf[i]->update_F_H(F, H);
            estimate = joint_kf[i]->updateData(measure);
            joint_data[i].position = estimate[0];
            joint_data[i].velocity = estimate[1];
            joint_data[i].acceleration = estimate[2];
        }
    }

    static bool readImu(bool flag_half_up_body, Eigen::Vector3d &gyro, Eigen::Vector3d &acc, Eigen::Vector4d &quat, Eigen::Vector3d &free_acc)
    {
        Eigen::Vector3d accel, gyrosco;
        Eigen::Quaterniond quaternion;

        struct timespec t0_sol, t1_sol;
        clock_gettime(CLOCK_MONOTONIC, &t0_sol);
        bool readflag = false;
        if (imu_type_ == ImuType::IMU_TYPE_HIPNUC){
        if(flag_half_up_body) { // Only Use Half Up Body.
            accel<<0,0,0;
            gyrosco<<0,0,0;
            quaternion = Eigen::Quaterniond(1,0,0,0);
                readflag = true;
            }
            else{
                readflag = HIPNUC_IMU::getImuDataFrame(accel, gyrosco, quaternion);
            }
        }
        else {
            if(flag_half_up_body) { // Only Use Half Up Body.
                accel<<0,0,0;
                gyrosco<<0,0,0;
                quaternion = Eigen::Quaterniond(1,0,0,0);
                readflag = true;
            }
            else{
                readflag = xsens_IMU::getImuDataFrame(accel, gyrosco, quaternion);
        }
        }
        if (!readflag)
        {

            std::cout << "getImuData failed! using old data " << static_cast<int>(use_pre_imu_times) << std::endl;
            use_pre_imu_times++;
            if (use_pre_imu_times > 30)
            {
                // KuavoErrorManager.trigger(ERROR_CRITICAL, "readIMU failed");
                return false;
            }
            accel = pre_accel;
            gyrosco = pre_gyrosco;
            quaternion = pre_quaternion;
        }
        else
        {
            use_pre_imu_times = 0;
            pre_accel = accel;
            pre_gyrosco = gyrosco;
            pre_quaternion = quaternion;
        }
        clock_gettime(CLOCK_MONOTONIC, &t1_sol);
        if (TIME_DIFF(t0_sol, t1_sol) * 1000 > 0.1)
        {
            printf("getImuDataFrame timeout err: %f\n",
                   TIME_DIFF(t0_sol, t1_sol) * 1000);
        }

        static int readpacket_count;

        static struct timespec t_ini, t_end;
        if (readpacket_count == 0)
        {
            clock_gettime(CLOCK_MONOTONIC, &t_ini);
        }
        readpacket_count++;
        clock_gettime(CLOCK_MONOTONIC, &t_end);
        // printf("readpacket_count: %d, %f\n\n", readpacket_count, TIME_DIFF(t_ini, t_end) * 1000);

        acc = accel;
        gyro = gyrosco;
        quat = Eigen::Vector4d(quaternion.w(), quaternion.x(), quaternion.y(), quaternion.z());
        free_acc = removeGravity(acc, quaternion);
        return true;
    }

    /**
     * @brief 获取 imu（角速度， 加速度， 四元数） 和 关节位置，关节速度，关节加速度，关节力矩 数值
     *
     * @param qvt
     * @param sensor_data
     * @param free_acc
     * @warning 因为加速度是根据固定的 dt 除 速度差，一个周期只能调用一次
     */
    bool HardwarePlant::readSensor(SensorData_t &sensor_data)
    {
        bool result = true;

        uint32_t na_r = num_joint;
        std::vector<JointParam_t> filter_data(na_r);
        for (uint32_t i = 0; i < na_r; i++)
        {
            joint_data_old[i] = joint_data[i];
        }
        GetMotorData(joint_ids, joint_data);
        for (uint32_t i = 0; i < na_r; i++)
        {
            joint_data[i].acceleration = (joint_data[i].velocity - joint_data_old[i].velocity) / hard_dt;
            filter_data[i] = joint_data[i];
        }
        Eigen::VectorXd tmp_torque, tmp_state;
        tmp_torque.setZero(num_joint);
        tmp_state.setZero(num_joint);
        for (int i = 0; i < num_joint; i++)
        {
            tmp_torque(i) = joint_data[i].torque_demand_trans;
            tmp_state(i) = joint_data[i].status;
        }

        std::string err_msg;
        result = result && checkJointSafety(filter_data, joint_ids, err_msg);
        if (!result)
        {
            std::cerr << err_msg << std::endl;
            // HWPlantDeInit();
            std::raise(SIGINT);
            return false;
        }

        // jointPublish(lcm_ptr,"raw/joint", filter_data,na_r);

        jointFiltering(filter_data, hard_dt);
        // jointPublish(lcm_ptr, "sensors/joint", filter_data, na_r);

        // Eigen::VectorXd q_r(na_r);
        // Eigen::VectorXd v_r(na_r);
        // Eigen::VectorXd vd_r(na_r);
        // Eigen::VectorXd tau_r(na_r);
        sensor_data.joint_q.resize(na_r);
        sensor_data.joint_v.resize(na_r);
        sensor_data.joint_vd.resize(na_r);
        sensor_data.joint_current.resize(na_r);
        sensor_data.joint_torque_demand.resize(na_r);
        sensor_data.joint_igbt_temperature.resize(na_r);
        sensor_data.joint_ntc_temperature.resize(na_r);
        sensor_data.joint_status_word.resize(na_r);
        sensor_data.joint_error_code.resize(na_r);

        for (uint32_t i = 0; i < na_r; i++)
        {
            sensor_data.joint_q[i] = filter_data[i].position * (M_PI / 180.0);
            sensor_data.joint_v[i] = filter_data[i].velocity * (M_PI / 180.0);
            sensor_data.joint_vd[i] = filter_data[i].acceleration * (M_PI / 180.0);
            sensor_data.joint_current[i] = filter_data[i].torque;
            sensor_data.joint_torque_demand[i] = filter_data[i].torque_demand_trans;
            sensor_data.joint_igbt_temperature[i] = filter_data[i].igbt_temperature;
            sensor_data.joint_ntc_temperature[i] = filter_data[i].ntc_temperature;
            sensor_data.joint_status_word[i] = filter_data[i].status_word;
            sensor_data.joint_error_code[i] = filter_data[i].error_code;
        }

        if (has_end_effectors)
        {
            std::vector<double> claw_position_data;
            sensor_data.end_effectors_data.clear();
            for (uint32_t i = 0; i < 2; i++)
            {
                EndEffectorData ee_data;
                ee_data.type = EndEffectorType::jodell;
                ee_data.position = Eigen::VectorXd::Zero(1);
                ee_data.velocity = Eigen::VectorXd::Zero(1);
                ee_data.torque = Eigen::VectorXd::Zero(1);
                sensor_data.end_effectors_data.push_back(ee_data);
            }
            // sensor_data.end_effectors_data = {
            //     {EndEffectorType::jodell, Eigen::VectorXd::Zero(1), Eigen::VectorXd::Zero(1), Eigen::VectorXd::Zero(1)},
            //     {EndEffectorType::jodell, Eigen::VectorXd::Zero(1), Eigen::VectorXd::Zero(1), Eigen::VectorXd::Zero(1)}};
            // TODO:add position\velocity\torque
        }

        if(isLunbiModule(robot_module_)) 
        {
            //轮臂无imu提前返回
            return result;
        }

        bool flag_only_half_up_body = kuavo_settings_.running_settings.only_half_up_body;
        result = result && readImu(flag_only_half_up_body, sensor_data.gyro, sensor_data.acc, sensor_data.quat, sensor_data.free_acc);
        if (redundant_imu_)
        {
            Eigen::Vector3d accel, gyrosco;
            Eigen::Quaterniond quaternion;
            bool readflag = HIPNUC_IMU::getImuDataFrame(accel, gyrosco, quaternion);
            if (!readflag)
            {
                std::cerr << "read redundant imu failed" << std::endl;
            }
        }

        if (imu_invert)
        {
            Invt_imudate(sensor_data);
        }

        // lcmPublishVector(&lc_instance, "sensors/imu/free_acc", sensor_data.free_acc);
        // lcmPublishVector(&lc_instance, "sensors/imu/acc", sensor_data.acc);
        // lcmPublishVector(&lc_instance, "sensors/imu/gyro", sensor_data.gyro);
        // lcmPublishVector(&lc_instance, "sensors/imu/quat", sensor_data.quat);

        if (has_end_effectors)
        {
            std::vector<double> claw_position_data;
            // claw_position_data = claw_actuator->getPositions();
            sensor_data.end_effectors_data.clear();
            for (uint32_t i = 0; i < 2; i++)
            {
                EndEffectorData ee_data;
                ee_data.type = EndEffectorType::jodell;
                ee_data.position = Eigen::VectorXd::Zero(1);
                ee_data.velocity = Eigen::VectorXd::Zero(1);
                ee_data.torque = Eigen::VectorXd::Zero(1);
                sensor_data.end_effectors_data.push_back(ee_data);
            }
            // sensor_data.end_effectors_data = {
            //     {EndEffectorType::jodell, Eigen::VectorXd::Zero(1), Eigen::VectorXd::Zero(1), Eigen::VectorXd::Zero(1)},
            //     {EndEffectorType::jodell, Eigen::VectorXd::Zero(1), Eigen::VectorXd::Zero(1), Eigen::VectorXd::Zero(1)}};
            // TODO:add position\velocity\torque
        }
        return result;
    }

    /**
     * @brief 获取电机电流力矩转换系数
     *
     * @param ret_c2t_coeff
     */
    void HardwarePlant::GetC2Tcoeff(double *ret_c2t_coeff, size_t size)
    {
        auto len_c2t_coeff = c2t_coeff.size();
        auto  len = size > len_c2t_coeff ? len_c2t_coeff : size;
        std::cout << "ret_c2t_coeff size: " << (int)size << std::endl;
        std::cout << "len_c2t_coeff size: " << (int)len_c2t_coeff << std::endl;
        std::cout << "c2t size: " << (int)len << std::endl;
        for (auto i = 0; i < len; i++)
        {
            ret_c2t_coeff[i] = c2t_coeff[i];
        }
    }
    Eigen::VectorXd HardwarePlant::GetC2Tcoeff(Eigen::VectorXd &motor_cur)
    {
        Eigen::VectorXd motor_c2t_ = Eigen::VectorXd::Zero(motor_cur.size());
        for (int i = 0; i < motor_cul.size(); i++) {
            int index = motor_cul[i].size(); // 默认超出最大值
            for (int j = 1; j < motor_cul[i].size(); j++) {
                if (abs(motor_cur[i]) < motor_cul[i][j]) {
                    index = j - 1; // 找到区间
                    break;
                }
            }
            if (abs(motor_cur[i]) < motor_cul[i][0]) {
                index = -1; // 小于最小值
            }

            if (index == motor_cul[i].size()) {
                motor_c2t_[i] = motor_coeff[i].back(); // 超出最大值
            } else if (index == -1) {
                motor_c2t_[i] = motor_coeff[i][0]; // 小于最小值
            } else {
                double min_cul = motor_cul[i][index];
                double max_cul = motor_cul[i][index + 1];
                double min_c2t = motor_coeff[i][index];
                double max_c2t = motor_coeff[i][index + 1];
                motor_c2t_[i] = (max_c2t - min_c2t) * (abs(motor_cur[i]) - min_cul) / (max_cul - min_cul) + min_c2t;
            }
        }
        return motor_c2t_;
    }

    double HardwarePlant::motorFeedbackToJointTorqueNm(size_t joint_idx, double motor_feedback) const
    {
        const double abs_feedback = std::abs(motor_feedback);
        if (joint_idx >= motor_cul.size() || joint_idx >= motor_coeff.size() ||
            motor_cul[joint_idx].empty() || motor_coeff[joint_idx].empty() ||
            motor_cul[joint_idx].size() != motor_coeff[joint_idx].size())
        {
            // 分段配置缺失时使用默认 c2t 系数
            const double fallback_coefficient =
                joint_idx < c2t_coeff.size() && c2t_coeff[joint_idx] > 0.0
                    ? c2t_coeff[joint_idx]
                    : 1.0;
            return abs_feedback * fallback_coefficient;
        }

        const auto& current_points = motor_cul[joint_idx];
        const auto& torque_coeffs = motor_coeff[joint_idx];
        double coefficient = torque_coeffs.back();

        if (abs_feedback < current_points.front())
        {
            coefficient = torque_coeffs.front();
        }
        else
        {
            for (size_t i = 1; i < current_points.size(); ++i)
            {
                if (abs_feedback < current_points[i])
                {
                    const double ratio = (abs_feedback - current_points[i - 1]) /
                                         (current_points[i] - current_points[i - 1]);
                    coefficient = torque_coeffs[i - 1] +
                                  ratio * (torque_coeffs[i] - torque_coeffs[i - 1]);
                    break;
                }
            }
        }

        return abs_feedback * coefficient;
    }

    double HardwarePlant::getCalibrationStallTorqueThreshold(size_t joint_idx) const
    {
        if (joint_idx >= joint_peak_torque_limits_.size())
        {
            return 5.0;
        }

        // 优先使用当前机器人配置的峰值力矩比例
        const double ratio = calibration_stall_torque_ratio_ > 0.0
                                 ? calibration_stall_torque_ratio_
                                 : 0.3;
        return std::max(1.0, joint_peak_torque_limits_[joint_idx] * ratio);
    }

    bool HardwarePlant::stopArmForCalibrationFailure()
    {
        const int arm_start_idx = num_joint - num_head_joints - num_arm_joints;
        std::vector<uint8_t> ec_arm_ids;
        bool has_ruiwo_arm = false;
        bool stop_confirmed = true;

        for (int i = arm_start_idx; i < arm_start_idx + static_cast<int>(num_arm_joints); ++i)
        {
            if (i < 0 || i >= static_cast<int>(joint_ids.size()) ||
                i >= static_cast<int>(motor_info.driver.size()) ||
                i >= static_cast<int>(motor_info.motors_exist.size()))
            {
                continue;
            }

            if (motor_info.driver[i] == EC_MASTER && motor_info.motors_exist[i])
            {
                ec_arm_ids.push_back(joint_ids[i]);
            }
            else if (motor_info.driver[i] == RUIWO)
            {
                has_ruiwo_arm = true;
            }
        }

        // EC 驱动使用工程现有的零力矩方式停止
        // 当前接口没有执行结果反馈 因此下发后仍按停止未确认处理
        if (!ec_arm_ids.empty())
        {
            setZeroTorqueForJoints(ec_arm_ids);
            std::cerr << "[CalibrationStallGuard] 已向 EC 手臂关节下发零力矩命令，但无法确认执行结果"
                      << std::endl;
            stop_confirmed = false;
        }

        if (has_ruiwo_arm)
        {
            if (ruiwo_actuator == nullptr)
            {
                std::cerr << "[CalibrationStallGuard] RUIWO 驱动未初始化" << std::endl;
                return false;
            }

            // 标定专用接口通过具体驱动调用，保持公共基类的虚函数表兼容旧二进制
            // C++ Ruiwo 使用严格失能；Python Ruiwo 和 Motorevo 保持原有失能行为
            auto* ruiwo_driver = dynamic_cast<RuiWoActuator*>(ruiwo_actuator);
            const int disable_result = ruiwo_driver != nullptr
                                           ? ruiwo_driver->disableForCalibration()
                                           : ruiwo_actuator->disable();
            if (disable_result != 0)
            {
                std::cerr << "[CalibrationStallGuard] RUIWO 手臂失能失败 code="
                          << disable_result << std::endl;
                return false;
            }
        }

        return has_ruiwo_arm && stop_confirmed;
    }

    Eigen::VectorXd HardwarePlant::GetC2Tcoeff_Torque(Eigen::VectorXd &joint_torque)
    {
        Eigen::VectorXd torque_c2t = Eigen::VectorXd::Zero(joint_torque.size());
        for (int i = 0; i < motor_cul.size(); i++) {
            int index = motor_cul[i].size(); // 默认超出最大值
            for (int j = 1; j < motor_cul[i].size(); j++) {
                if (abs(joint_torque[i]) < motor_cul[i][j] * motor_coeff[i][j]) {
                    index = j - 1; // 找到区间
                    break;
                }
            }
            if (abs(joint_torque[i]) < motor_cul[i][0] * motor_coeff[i][0]) {
                index = -1; // 小于最小值
            }

            if (index == motor_cul[i].size()) {
                torque_c2t[i] = motor_coeff[i].back(); // 超出最大值
            } else if (index == -1) {
                torque_c2t[i] = motor_coeff[i][0]; // 小于最小值
            } else {
                double min_tau = motor_cul[i][index] * motor_coeff[i][index];
                double max_tau = motor_cul[i][index + 1] * motor_coeff[i][index + 1];
                double min_c2t = motor_coeff[i][index];
                double max_c2t = motor_coeff[i][index + 1];
                torque_c2t[i] = (max_c2t - min_c2t) * (abs(joint_torque[i]) - min_tau) / (max_tau - min_tau) + min_c2t;
            }
        }
        return torque_c2t;
    }

    void HardwarePlant::endEffectorCommand(std::vector<EndEffectorData> &end_effector_cmd)
    {
        std::vector<double> dexhand_position_data;
        std::vector<double> claw_position_data;
        for (auto &ee_cmd : end_effector_cmd)
        {
            if (ee_cmd.type == EndEffectorType::jodell)
            {
                claw_position_data.push_back(ee_cmd.position[0]);
            }
            if (ee_cmd.type == EndEffectorType::qiangnao || ee_cmd.type == EndEffectorType::qiangnao_touch || ee_cmd.type == EndEffectorType::linker_hand)
            {
                for (size_t i = 0; i < ee_cmd.position.size(); i++)
                {
                    dexhand_position_data.push_back(ee_cmd.position(i));
                }
            }
        }
        
        if (dexhand_position_data.size() == 12)
        {
            eef_controller::UnsignedDualHandsArray positions;
            auto single_hand_position_size = dexhand_position_data.size()/2;
            for(int i = 0; i < single_hand_position_size; i++) {
                positions[0][i] = std::min<uint16_t>(100, std::max<uint16_t>(0, dexhand_position_data[i]));
                positions[1][i] = std::min<uint16_t>(100, std::max<uint16_t>(0, dexhand_position_data[i+single_hand_position_size]));
            }
            if(dexhand_actuator != nullptr) {

                dexhand_actuator->send_position(positions);
            }
        }

    }

    bool HardwarePlant::checkLejuClawInitialized() {
        return leju_claw_actuator != nullptr;
    }

    void HardwarePlant::setLejuClawDebugCallback(eef_controller::LejuClawDebugCallback callback) {
        leju_claw_debug_callback_ = std::move(callback);
        if (leju_claw_actuator) {
            leju_claw_actuator->setDebugCallback(leju_claw_debug_callback_);
        }
    }

    void HardwarePlant::setLejuClawTargetCallback(eef_controller::LejuClawTargetCallback callback) {
        leju_claw_target_callback_ = std::move(callback);
        if (leju_claw_actuator) {
            leju_claw_actuator->setTargetCallback(leju_claw_target_callback_);
        }
    }

    bool HardwarePlant::controlLejuClaw(eef_controller::ControlClawRequest& req, eef_controller::ControlClawResponse& res, bool is_high_freq) {
        if (!leju_claw_actuator) {
            res.success = false;
            res.message = "leju claw actuator not initialized";
            return false;
        }
        return leju_claw_actuator->controlGripper(req, res, is_high_freq);
    }

    bool HardwarePlant::controlLejuClaw(eef_controller::lejuClawCommand& command, bool is_high_freq) {
        if (!leju_claw_actuator) {
            return false;
        }
        leju_claw_actuator->command(command, is_high_freq);
        return true;
    }

    bool HardwarePlant::recoverLejuClaw(const std::string& direction, float current, int duration_ms, int repeat, std::string& message) {
        if (!leju_claw_actuator) {
            message = "leju claw actuator not initialized";
            return false;
        }
        return leju_claw_actuator->recover(direction, current, duration_ms, repeat, message);
    }

    eef_controller::ClawState HardwarePlant::getLejuClawState() {
        if (!leju_claw_actuator) {
            return eef_controller::ClawState(); // 返回空状态
        }
        return eef_controller::GetClawState(leju_claw_actuator);
    }
    void HardwarePlant::writeCommand(Eigen::VectorXd cmd_r, uint32_t na_r, std::vector<int> control_modes, Eigen::VectorXd &joint_kp, Eigen::VectorXd &joint_kd)
    {
        for (uint32_t i = 0; i < num_joint; i++)
        {
            joint_cmd_old[i] = joint_cmd[i];
        }

        // uint32_t na_r = num_joint;
        // Eigen::VectorXd cmd_r(na_r * 4);
        // cmds2Cmdr(cmd, na, cmd_r, na_r);
        std::string err_msg;
        bool result = true;
        Eigen::VectorXd tau_ratio(na_r);
        tau_ratio << cmd_r.segment(na_r * 4, na_r);

        std::vector<uint8_t> joint_tau_ids;
        std::vector<uint8_t> joint_vel_ids;
        std::vector<uint8_t> joint_pos_ids;
        std::vector<JointParam_t> joint_tau_cmd;
        std::vector<JointParam_t> joint_vel_cmd;
        std::vector<JointParam_t> joint_pos_cmd;

        uint8_t num_tau = 0, num_vel = 0, num_pos = 0;
        SensorData_t sensor_data_motor;
        SensorData_t sensor_data_joint;
        getState(sensor_data_motor,sensor_data_joint);
        Eigen::VectorXd joint_c2t(na_foot_ + num_arm_joints + num_head_joints + num_waist_joints);
        Eigen::VectorXd cmd_tau = cmd_r.segment(na_r * 2, na_r);
        joint_c2t = GetC2Tcoeff_Torque(cmd_tau);

        for (uint32_t i = 0; i < na_r; i++)
        {
            JointParam_t joint_tau_temp;
            joint_tau_temp.status = 0;
            double tmp_tau{0.0};
            switch (control_modes[i])
            {
            case MOTOR_CONTROL_MODE_TORQUE:
                // RUIWO 驱动层不支持 CST 模式，改走 CSP 通路并将 kp/kd 置零来模拟 CST 效果
                if (motor_info.driver[joint_ids[i] - 1] == RUIWO)
                {
                    joint_tau_temp.positionOffset = 0;
                    joint_tau_temp.position = cmd_r[i] * (180.0 / M_PI);
                    joint_tau_temp.velocityOffset = cmd_r[na_r + i] * (180.0 / M_PI);
                    joint_tau_temp.torqueOffset = cmd_r[na_r * 2 + i] / joint_c2t[i];
                    joint_tau_temp.maxTorque = cmd_r[na_r * 3 + i];
                    joint_tau_temp.kp = 0.0;// kpkd设置成0模拟cst模式
                    joint_tau_temp.kd = 0.0;
                    joint_pos_cmd.push_back(joint_tau_temp);
                    joint_pos_ids.push_back(joint_ids[i]);
                    num_pos++;
                }
                else
                {
                    joint_tau_temp.torqueOffset = 0;
                    joint_tau_temp.torque = cmd_r[na_r * 2 + i] / joint_c2t[i]; //c2t_coeff[i];
                    joint_tau_temp.maxTorque = cmd_r[na_r * 3 + i];

                    // for feedback control
                    joint_tau_temp.position = cmd_r[i] * (180.0 / M_PI);
                    joint_tau_temp.velocityOffset = cmd_r[na_r + i] * (180.0 / M_PI);
                    joint_tau_temp.kp = joint_kp[i];
                    joint_tau_temp.kd = joint_kd[i];
                    
                    //
                    joint_tau_cmd.push_back(joint_tau_temp);
                    joint_tau_ids.push_back(joint_ids[i]);
                    num_tau++;
                }
                break;
            case MOTOR_CONTROL_MODE_VELOCITY:

                joint_tau_temp.velocityOffset = 0;
                joint_tau_temp.velocity = cmd_r[na_r + i] * (180.0 / M_PI);
                joint_tau_temp.torqueOffset = cmd_r[na_r * 2 + i] / joint_c2t[i]; //c2t_coeff[i];
                joint_tau_temp.maxTorque = cmd_r[na_r * 3 + i];
                joint_vel_cmd.push_back(joint_tau_temp);
                joint_vel_ids.push_back(joint_ids[i]);
                num_vel++;
                break;
            case MOTOR_CONTROL_MODE_POSITION:
                joint_tau_temp.positionOffset = 0;
                joint_tau_temp.position = cmd_r[i] * (180.0 / M_PI);
                joint_tau_temp.velocityOffset = cmd_r[na_r + i] * (180.0 / M_PI);
                // joint_tau_temp.torqueOffset = cmd_r[na_r * 2 + i] / joint_c2t[i] * tau_ratio[i];
                joint_tau_temp.maxTorque = cmd_r[na_r * 3 + i];
                joint_tau_temp.kp = joint_kp[i];
                joint_tau_temp.kd = joint_kd[i];

                tmp_tau = cmd_r[na_r * 2 + i] / joint_c2t[i] * tau_ratio[i];
                if (tmp_tau > joint_tau_temp.maxTorque)
                {
                    joint_tau_temp.torqueOffset = joint_tau_temp.maxTorque;
                }
                else if (tmp_tau < -joint_tau_temp.maxTorque)
                {
                    joint_tau_temp.torqueOffset = -joint_tau_temp.maxTorque;
                }
                else
                {
                    joint_tau_temp.torqueOffset = tmp_tau;
                }

                joint_pos_cmd.push_back(joint_tau_temp);
                joint_pos_ids.push_back(joint_ids[i]);
                num_pos++;
                break;
            default:
                break;
            }
            joint_cmd[i] = joint_tau_temp;
        }
        result = result && checkJointSafety(joint_pos_cmd, joint_pos_ids, err_msg);
        if (!result)
        {
            std::cerr << "writeCommand ERROR: \n"
                      << err_msg << std::endl;
            std::raise(SIGINT);
        }
        // std::cout<<std::endl<<std::endl;

        if (num_tau > 0)
            SetMotorTorque(joint_tau_ids, joint_tau_cmd);
        if (num_vel > 0)
        {
            SetMotorVelocity(joint_vel_ids, joint_vel_cmd);
        }
        if (num_pos > 0)
        {
            SetMotorPosition(joint_pos_ids, joint_pos_cmd);
        }
    }
    void HardwarePlant::calibrateArmJoints()
    {
        auto has_ruiwo = std::find(motor_info.driver.begin(), motor_info.driver.end(), MotorDriveType::RUIWO);
        if (has_ruiwo == motor_info.driver.end())
        {
            std::cerr << "仅支持 ruiwo 电机的手臂关节校准。" << std::endl;
            return;
        }
        if (ruiwo_actuator == nullptr)
        {
            std::cerr << "ruiwo_actuator 为 nullptr，请确保已经加载 ruiwo 驱动。" << std::endl;
            return;
        }
        std::vector<double> moving_pos(num_joint, 0);
        initial_input_cmd_ = '\0';
        int left_right = 0;
        int motor_id = 2;
        std::ostringstream cali_tips_oss;
        cali_tips_oss << "\033[32m*******校准手臂电机编码器偏移*******\n"
                      << "1. 输入 'l' 或 'r' 选择左侧或右侧手臂电机进行校准。\n"
                      << "2. 输入 2-7 选择要校准的电机。\n"
                      << "3. 输入 'w' 增加选中电机编码器的轮数，'s' 减少选中电机编码器的轮数（注意：电机会移动！）\n"
                      << "4. 输入 'e' 启用电机，'d' 禁用电机(禁用之后可以手动掰电机到期望位置), 'f' 将当前的位置用作零位置\n"
                      << "5. 输入 'c' 保存校准结果到文件\n"
                      << "6. 输入 '<Enter>' 显示当前选择，'h' 再次显示此提示，'q' 退出校准。\033[0m\n";
        std::cout << cali_tips_oss.str() << std::endl;
        auto print_selection = [&]()
        {
            std::cout << "选中" << ((left_right == 0) ? "左臂" : "右臂") << std::to_string(motor_id) + "号电机." << std::endl;
        };
        while (1)
        {
            if (kbhit())
            {
                initial_input_cmd_ = '\0';
                initial_input_cmd_ = getchar();
            }
            if (initial_input_cmd_ == 'h')
            {
                std::cout << cali_tips_oss.str() << std::endl;
                initial_input_cmd_ = '\0';
            }
            else if (initial_input_cmd_ == 'e')
            {
                std::cout << "enable the motor " << std::endl;
                ruiwo_actuator->enable();
            }
            else if (initial_input_cmd_ == 'd')
            {
                std::cout << "disable the motor " << std::endl;
                ruiwo_actuator->disable();
            }
            else if (initial_input_cmd_ == 'l')
            {
                left_right = 0;
                print_selection();
            }
            else if (initial_input_cmd_ == 'r')
            {
                left_right = 1;
                print_selection();
            }
            else if (initial_input_cmd_ >= '2' && initial_input_cmd_ <= '7')
            {
                motor_id = initial_input_cmd_ - '0';
                print_selection();
            }
            else if (initial_input_cmd_ == 'w')
            {
                std::cout << "增加" << ((left_right == 0) ? "左臂" : "右臂") << std::to_string(motor_id) + "号电机的轮数." << std::endl;
                int real_id = left_right * 6 + motor_id - 2;
                std::cout << "real_id: " << real_id << std::endl;
                ruiwo_actuator->changeEncoderZeroRound(real_id, 1);
                std::fill(moving_pos.begin(), moving_pos.end(), 0.0);
                this->jointMoveTo(moving_pos, 60.0, dt_);
            }
            else if (initial_input_cmd_ == 's')
            {
                std::cout << "减少" << ((left_right == 0) ? "左臂" : "右臂") << std::to_string(motor_id) + "号电机的轮数." << std::endl;
                int real_id = left_right * 6 + motor_id - 2;
                std::cout << "real_id: " << real_id << std::endl;
                ruiwo_actuator->changeEncoderZeroRound(real_id, -1);
                std::fill(moving_pos.begin(), moving_pos.end(), 0.0);
                this->jointMoveTo(moving_pos, 60.0, dt_);
            }
            else if (initial_input_cmd_ == 'c')
            {
                std::cout << "保存校准结果" << std::endl;
                ruiwo_actuator->saveZeroPosition();
                initial_input_cmd_ = '\0';
            }
            else if (initial_input_cmd_ == 'f')
            {
                std::cout << "将当前的位置保存为零位置." << std::endl;
                ruiwo_actuator->saveAsZeroPosition();
                initial_input_cmd_ = '\0';
            }
            else if (initial_input_cmd_ == 'q')
            {
                std::cout << "退出校准." << std::endl;
                break;
            }
            else if (initial_input_cmd_ == '\n')
            {
                print_selection();
            }
            initial_input_cmd_ = '\0';
            usleep(2000);
        }
    }

    bool HardwarePlant::calibrateArmJointsAtLimit(bool auto_mode, bool calibrate_head, bool head_only,
                                                   bool calibrate_leg, bool leg_only)
    {

        std::cout << "\033[32m======== 开始限位校准 ========\033[0m" << std::endl;
        
        // 模式提示
        if (leg_only)
        {
            std::cout << "校准模式: \033[33m仅校准腿\033[0m - 只校准腿关节（前4个关节）" << std::endl;
        }
        else if (head_only)
        {
            std::cout << "校准模式: \033[33m仅校准头部\033[0m - 只校准头部关节" << std::endl;
        }
        else if (auto_mode)
        {
            std::cout << "校准模式: \033[33m自动校准\033[0m - 所有关节自动依次校准" << std::endl;
        }
        else
        {
            std::cout << "校准模式: \033[33m逐个校准\033[0m - 每组关节需要手动确认" << std::endl;
            std::cout << "提示: 每组校准前按 Enter 开始，校准后按 Enter 继续下一组" << std::endl;
        }
        
        // 检查是否有 ruiwo 电机
        auto has_ruiwo = std::find(motor_info.driver.begin(), motor_info.driver.end(), MotorDriveType::RUIWO);
        if (has_ruiwo == motor_info.driver.end())
        {
            std::cerr << "警告：没有找到 ruiwo 电机" << std::endl;
        }
        else if (ruiwo_actuator == nullptr)
        {
            std::cerr << "错误：ruiwo_actuator 为 nullptr，请确保已经加载 ruiwo 驱动。" << std::endl;
            return false;
        }

        // 获取 EC 电机的当前零点偏移
        std::vector<double> ec_motor_offsets;
        getMotorPositionOffset(ec_motor_offsets);
        std::cout << "获取到 " << ec_motor_offsets.size() << " 个 EC 电机的当前零点偏移" << std::endl;
        
        // 备份原始EC电机零点偏移，用于放弃保存时恢复
        std::vector<double> original_ec_motor_offsets = ec_motor_offsets;
        
        // 用于跟踪哪些零点被修改了
        std::map<int, double> modified_ec_offsets;    // EC index -> new offset
        std::map<int, double> modified_ruiwo_offsets; // Ruiwo arm index -> offset

        // 统一恢复本次限位标定产生的运行时零点修改
        auto rollback_calibration_changes = [&]()
        {
            if (!modified_ec_offsets.empty())
            {
                setMotorPositionOffset(original_ec_motor_offsets);
                std::cout << "已恢复 EC 电机零点到标定前数值" << std::endl;
            }

            if (!modified_ruiwo_offsets.empty() && ruiwo_actuator != nullptr)
            {
                for (const auto& [ruiwo_idx, delta_deg] : modified_ruiwo_offsets)
                {
                    ruiwo_actuator->adjustZeroPosition(ruiwo_idx, -delta_deg * M_PI / 180.0);
                    std::cout << "  已回滚 ruiwo 电机[" << ruiwo_idx << "] 零点增量: "
                              << delta_deg << " 度" << std::endl;
                }
                std::cout << "已恢复 RUIWO 电机零点到标定前数值" << std::endl;
            }

            modified_ec_offsets.clear();
            modified_ruiwo_offsets.clear();
        };

        // 从配置文件读取校准参数
        const double CALIBRATION_VELOCITY = kuavo_settings_.predefined_arm_pose.arm_calibration_velocity;        // 校准移动速度 (度/秒)
        const double TIMEOUT = kuavo_settings_.predefined_arm_pose.arm_calibration_timeout;                      // 超时时间 (秒)
        const double POSITION_VARIANCE_TIME = kuavo_settings_.predefined_arm_pose.arm_calibration_position_variance_time;      // 位置方差检测时间窗口 (秒)
        const double POSITION_VARIANCE_THRESHOLD = kuavo_settings_.predefined_arm_pose.arm_calibration_position_variance_threshold; // 位置方差阈值 (度^2)
        
        // 固定的校准参数
        const double CURRENT_THRESHOLD = 5.0;           // 电流阈值 (单位取决于传感器)
        const double VELOCITY_THRESHOLD = 1.5;          // 速度阈值 (度/秒)
        const double VELOCITY_STABLE_TIME = 0.3;        // 速度稳定时间 (秒)
        const double CONTROL_FREQUENCY = 500.0;         // 控制频率 (Hz)
        const double POSITION_STEP = CALIBRATION_VELOCITY / CONTROL_FREQUENCY;  // 根据速度和频率计算步长 (度)
        const int VELOCITY_STABLE_SAMPLES = static_cast<int>(VELOCITY_STABLE_TIME * CONTROL_FREQUENCY);
        const int POSITION_HISTORY_SAMPLES = static_cast<int>(POSITION_VARIANCE_TIME * CONTROL_FREQUENCY);
        // 防止启动瞬间/目标尚未追上实际位置时，被“位置不变/速度过低”误判为限位
        const double MIN_LIMIT_DETECT_TIME = 1.0;       // 开始检测前的最小运动时间 (秒)
        const double MIN_LIMIT_TRAVEL_DEG = 1.0;        // 开始检测前的最小行程 (度)
        
        std::cout << "校准参数: 速度=" << CALIBRATION_VELOCITY << "度/秒, 步长=" << POSITION_STEP << "度, 频率=" << CONTROL_FREQUENCY << "Hz" << std::endl;
        std::cout << "            超时=" << TIMEOUT << "秒, 方差时间窗口=" << POSITION_VARIANCE_TIME << "秒, 方差阈值=" << POSITION_VARIANCE_THRESHOLD << std::endl;
        std::cout << "            限位检测保护: 最小时间=" << MIN_LIMIT_DETECT_TIME
                  << "秒, 最小行程=" << MIN_LIMIT_TRAVEL_DEG << "度" << std::endl;

        // 计算各关节段在 all_joint_data / jointMoveTo 的索引范围
        //
        // 该工程的 joint 数组段顺序可从 motor2joint() 的 segment 拼接看出：
        //   leg(na_foot_) -> waist(num_waist_joints) -> arm+head(num_arm_joints+num_head_joints)
        //
        // 因此这里必须按：leg -> waist -> arm -> head 计算起始索引。
        // 若仍用“腿->手->头”（忽略腰）会把腰误当成手臂第一个关节，导致回零/安全姿态时腰部被错误驱动（例如在 52 机型上出现回腰旋转）。
        int num_leg_joints = num_joint - num_arm_joints - num_head_joints - num_waist_joints;
        int leg_start_idx = 0;
        int waist_start_idx = leg_start_idx + num_leg_joints;
        int arm_start_idx = waist_start_idx + static_cast<int>(num_waist_joints);
        int num_calibration_joints = num_arm_joints + num_head_joints;  // 手臂+头部（不含腰）
        
        std::cout << "腰关节数量: " << num_waist_joints << std::endl;
        std::cout << "腿关节数量: " << num_leg_joints << std::endl;
        std::cout << "手臂关节数量: " << num_arm_joints << std::endl;
        std::cout << "头部关节数量: " << num_head_joints << std::endl;
        std::cout << "腿关节起始索引: " << leg_start_idx << std::endl;
        std::cout << "腰关节起始索引: " << waist_start_idx << std::endl;
        std::cout << "手臂关节起始索引: " << arm_start_idx << std::endl;
        std::cout << "总共需要校准的关节数: " << num_calibration_joints + num_leg_joints << std::endl;

        // 从 kuavo_settings_ 读取校准安全姿态 (calibration_safe_pose)
        std::vector<double> calibration_safe_pose(num_joint, 0.0);
        if (kuavo_settings_.predefined_arm_pose.calibration_safe_pose.size() >= num_calibration_joints)
        {
            // 读取手臂和头部的安全姿态
            for (size_t i = 0; i < num_calibration_joints; ++i)
            {
                calibration_safe_pose[arm_start_idx + i] = kuavo_settings_.predefined_arm_pose.calibration_safe_pose[i];
            }
            std::cout << "从配置读取到校准安全姿态 (calibration_safe_pose) - 手臂和头部" << std::endl;
        }
        else
        {
            std::cerr << "警告：无法从配置读取 calibration_safe_pose，使用零位作为安全姿态" << std::endl;
        }

        // 从 kuavo_settings_ 读取校准限位 (arm_calibration_limits) - 包含手臂和头部
        std::vector<double> arm_calibration_limits;
        if (kuavo_settings_.predefined_arm_pose.arm_calibration_limits.size() >= num_calibration_joints)
        {
            for (size_t i = 0; i < num_calibration_joints; ++i)
            {
                arm_calibration_limits.push_back(kuavo_settings_.predefined_arm_pose.arm_calibration_limits[i]);
            }
            std::cout << "从配置读取到校准限位 (arm_calibration_limits) - 手臂和头部" << std::endl;
        }
        else
        {
            std::cerr << "错误：无法从配置读取 arm_calibration_limits" << std::endl;
            return false;
        }

        // 从 kuavo_settings_ 读取校准运动方向 (arm_calibration_directions) - 包含手臂和头部
        std::vector<int> arm_calibration_directions;
        if (kuavo_settings_.predefined_arm_pose.arm_calibration_directions.size() >= num_calibration_joints)
        {
            for (size_t i = 0; i < num_calibration_joints; ++i)
            {
                arm_calibration_directions.push_back(static_cast<int>(kuavo_settings_.predefined_arm_pose.arm_calibration_directions[i]));
            }
            std::cout << "从配置读取到校准运动方向 (arm_calibration_directions) - 手臂和头部" << std::endl;
        }
        else
        {
            std::cerr << "错误：无法从配置读取 arm_calibration_directions" << std::endl;
            return false;
        }

        // 读取腿部校准参数（如果包含腿部校准）
        std::vector<double> leg_calibration_safe_pose;
        std::vector<double> leg_calibration_limits;
        std::vector<int> leg_calibration_directions;
        
        if (calibrate_leg || leg_only)
        {
            if (num_leg_joints > 0)
            {
                // 读取安全姿态
                if (kuavo_settings_.predefined_arm_pose.leg_calibration_safe_pose.size() >= num_leg_joints)
                {
                    for (size_t i = 0; i < num_leg_joints; ++i)
                    {
                        leg_calibration_safe_pose.push_back(
                            kuavo_settings_.predefined_arm_pose.leg_calibration_safe_pose[i]);
                    }
                    std::cout << "从配置读取到腿部校准安全姿态 (leg_calibration_safe_pose)" << std::endl;
                }
                else
                {
                    std::cerr << "警告：无法从配置读取 leg_calibration_safe_pose，使用零位作为安全姿态" << std::endl;
                }
                
                // 读取限位位置
                if (kuavo_settings_.predefined_arm_pose.leg_calibration_limits.size() >= num_leg_joints)
                {
                    for (size_t i = 0; i < num_leg_joints; ++i)
                    {
                        leg_calibration_limits.push_back(
                            kuavo_settings_.predefined_arm_pose.leg_calibration_limits[i]);
                    }
                    std::cout << "从配置读取到腿部校准限位 (leg_calibration_limits)" << std::endl;
                }
                else
                {
                    std::cerr << "错误：无法从配置读取 leg_calibration_limits" << std::endl;
                    return false;
                }
                
                // 读取运动方向
                if (kuavo_settings_.predefined_arm_pose.leg_calibration_directions.size() >= num_leg_joints)
                {
                    for (size_t i = 0; i < num_leg_joints; ++i)
                    {
                        leg_calibration_directions.push_back(
                            static_cast<int>(kuavo_settings_.predefined_arm_pose.leg_calibration_directions[i]));
                    }
                    std::cout << "从配置读取到腿部校准运动方向 (leg_calibration_directions)" << std::endl;
                }
                else
                {
                    std::cerr << "错误：无法从配置读取 leg_calibration_directions" << std::endl;
                    return false;
                }
            }
            else
            {
                std::cerr << "错误：要求校准腿部，但 num_leg_joints = 0" << std::endl;
                return false;
            }
        }

        // 动态生成校准分组：腿部每个关节独立一组（放在最前），头部每个关节独立一组，手臂逐对校准（左右对称）
        std::vector<std::vector<int>> calibration_groups;
        int joints_per_arm = num_arm_joints / 2;  // 每条手臂的关节数
        
        // 动态生成校准分组
        int leg_groups_count = 0;
        int head_groups_count = 0;
        
        if (leg_only)
        {
            // 只校准腿关节
            for (int i = 0; i < num_leg_joints; ++i)
            {
                std::vector<int> leg_group = { leg_start_idx + i };
                calibration_groups.push_back(leg_group);
                leg_groups_count++;
            }
            std::cout << "仅校准腿关节，共 " << num_leg_joints << " 个独立关节组" << std::endl;
        }
        else if (head_only)
        {
            // 只校准头部关节
            if (num_head_joints > 0)
            {
                int head_start_idx = arm_start_idx + num_arm_joints;
                for (int i = 0; i < num_head_joints; ++i)
                {
                    std::vector<int> head_group = { head_start_idx + i };
                    calibration_groups.push_back(head_group);
                    head_groups_count++;
                }
                std::cout << "仅校准头部关节，共 " << num_head_joints << " 个独立关节组" << std::endl;
            }
            else
            {
                std::cerr << "错误：没有头部关节需要校准" << std::endl;
                return false;
            }
        }
        else
        {
            // 添加腿关节（如果包含）
            if (calibrate_leg)
            {
                for (int i = 0; i < num_leg_joints; ++i)
                {
                    std::vector<int> leg_group = { leg_start_idx + i };
                    calibration_groups.push_back(leg_group);
                    leg_groups_count++;
                }
                std::cout << "已添加腿关节组，共 " << num_leg_joints << " 个独立关节组" << std::endl;
            }
            // 正常模式：头部+手臂
            if (calibrate_head && num_head_joints > 0)
            {
                int head_start_idx = arm_start_idx + num_arm_joints;
                for (int i = 0; i < num_head_joints; ++i)
                {
                    std::vector<int> head_group = { head_start_idx + i };
                    calibration_groups.push_back(head_group);
                    head_groups_count++;
                }
                std::cout << "已添加头部关节组，共 " << num_head_joints << " 个独立关节组（最后校准）" << std::endl;
            }
            else if (num_head_joints > 0)
            {
                std::cout << "跳过头部关节校准" << std::endl;
            }
            
            // 手臂关节：从第一个关节开始，包括 EC master 和 ruiwo 电机
            for (int i = 0; i < joints_per_arm; ++i)
            {
                std::vector<int> group = {
                    arm_start_idx + i,                    // 左臂关节
                    arm_start_idx + joints_per_arm + i    // 右臂对应关节
                };
                calibration_groups.push_back(group);
            }
            

        }
        
        if (leg_only)
        {
            std::cout << "生成了 " << calibration_groups.size() << " 组校准关节（仅腿部 " << leg_groups_count << " 组）" << std::endl;
        }
        else if (head_only)
        {
            std::cout << "生成了 " << calibration_groups.size() << " 组校准关节（仅头部 " << head_groups_count << " 组）" << std::endl;
        }
        else
        {
            std::cout << "生成了 " << calibration_groups.size() << " 组校准关节（头部 " << head_groups_count << " 组 + 手臂 " << joints_per_arm << " 组）" << std::endl;
        }

        // 步骤1: 准备全身关节的初始位置
        // 读取当前所有关节的位置（腿部位置）
        std::vector<JointParam_t> all_joint_data(num_joint);
        GetMotorData(joint_ids, all_joint_data);
        
        // 保存腿部当前位置，设置手臂为校准安全姿态
        std::vector<double> calibration_start_pose(num_joint);
        for (size_t i = 0; i < num_joint; ++i)
        {
            calibration_start_pose[i] = all_joint_data[i].position;
        }
        
        // 根据校准模式设置安全姿态
        if (leg_only)
        {
            // 只校准腿时，设置腿关节和手臂的安全姿态（确保手臂在安全位置）
            // 设置手臂安全姿态
            for (size_t i = 0; i < num_arm_joints; ++i)
            {
                calibration_start_pose[arm_start_idx + i] = calibration_safe_pose[arm_start_idx + i];
            }
            // 设置腿关节的安全姿态
            for (size_t i = 0; i < num_leg_joints; ++i)
            {
                calibration_start_pose[leg_start_idx + i] = leg_calibration_safe_pose[i];
            }
            std::cout << "\n步骤 1: 移动到校准初始位置（手臂和腿移动到安全姿态）..." << std::endl;
        }
        else if (head_only)
        {
            // 只校准头部时，只设置头部关节的安全姿态
            int head_start_idx = arm_start_idx + num_arm_joints;
            for (size_t i = 0; i < num_head_joints; ++i)
            {
                calibration_start_pose[head_start_idx + i] = calibration_safe_pose[head_start_idx + i];
            }
            std::cout << "\n步骤 1: 移动到校准初始位置（腿部保持当前位置，头部移动到安全姿态）..." << std::endl;
        }
        else
        {
            // 正常模式：设置手臂、头部和（可选）腿部的安全姿态
            // 设置手臂和头部
            for (size_t i = 0; i < num_arm_joints + num_head_joints; ++i)
            {
                calibration_start_pose[arm_start_idx + i] = calibration_safe_pose[arm_start_idx + i];
            }
            
            // 如果包含腿部，设置腿部安全姿态
            if (calibrate_leg)
            {
                for (size_t i = 0; i < num_leg_joints; ++i)
                {
                    calibration_start_pose[leg_start_idx + i] = leg_calibration_safe_pose[i];
                }
                std::cout << "\n步骤 1: 移动到校准初始位置（手臂、头部和腿移动到安全姿态）..." << std::endl;
            }
            else
            {
                std::cout << "\n步骤 1: 移动到校准初始位置（手臂和头部移动到安全姿态）..." << std::endl;
            }
        }
        if (!jointMoveTo(calibration_start_pose, 30.0, dt_))
        {
            // 安全姿态移动失败后不允许继续进入限位搜索
            std::cerr << "\033[31m移动到校准安全姿态失败，本次标定已中止\033[0m" << std::endl;
            return false;
        }
        usleep(100000); // 等待500ms
        
        // 读取所有关节的当前状态，用于后续控制
        std::vector<JointParam_t> current_all_joints(num_joint);
        std::vector<JointParam_t> target_all_joints(num_joint);
        GetMotorData(joint_ids, current_all_joints);
        
        // 获取默认的 kp/kd 值（用于 CSP 模式）
        const auto &running_settings = kuavo_settings_.running_settings;
        const bool use_vr_joint_gains = running_settings.use_vr_arm_kpkd &&
                                        !running_settings.vr_joint_kp.empty() &&
                                        !running_settings.vr_joint_kd.empty() &&
                                        running_settings.vr_joint_kp.size() == running_settings.vr_joint_kd.size();
        const auto &default_joint_kp = use_vr_joint_gains ? running_settings.vr_joint_kp : running_settings.joint_kp;
        const auto &default_joint_kd = use_vr_joint_gains ? running_settings.vr_joint_kd : running_settings.joint_kd;
        
        // 初始化目标位置为当前位置
        for (size_t i = 0; i < num_joint; ++i)
        {
            target_all_joints[i].position = current_all_joints[i].position;
            target_all_joints[i].velocity = 0.0;
            target_all_joints[i].torque = 0.0;
            
            // 对于 CSP 模式（位置控制模式），使用默认的 kp/kd 值
            // 只有 EC_MASTER 电机在 CSP 模式下需要设置 kp/kd
            if (motor_info.driver[i] == EC_MASTER && motor_info.motors_exist[i])
            {
                // 找到对应的 EC_MASTER 索引（在所有 EC_MASTER 电机中的索引）
                int ec_master_index = -1;
                int ec_count = 0;
                for (int j = 0; j <= static_cast<int>(i); ++j)
                {
                    if (motor_info.driver[j] == EC_MASTER && motor_info.motors_exist[j])
                    {
                        if (j == static_cast<int>(i))
                        {
                            ec_master_index = ec_count;
                            break;
                        }
                        ec_count++;
                    }
                }
                
                // 如果找到了对应的索引，且默认值数组大小足够，则使用默认值
                if (ec_master_index >= 0 && 
                    ec_master_index < static_cast<int>(default_joint_kp.size()) &&
                    ec_master_index < static_cast<int>(default_joint_kd.size()))
                {
                    target_all_joints[i].kp = static_cast<double>(default_joint_kp[ec_master_index]);
                    target_all_joints[i].kd = static_cast<double>(default_joint_kd[ec_master_index]);
                }
                else
                {
                    // 如果找不到对应的默认值，使用 0（保持原有行为）
                    target_all_joints[i].kp = 0.0;
                    target_all_joints[i].kd = 0.0;
                }
            }
            else
            {
                // 非 EC_MASTER 电机或电机不存在，设置为 0
                target_all_joints[i].kp = 0.0;
                target_all_joints[i].kd = 0.0;
            }
        }
        
        std::cout << "已获取全身关节初始状态" << std::endl;
        
        // 步骤2: 逐组校准（从最后一组开始，逆序执行）
        for (int group_idx = calibration_groups.size() - 1; group_idx >= 0; --group_idx)
        {
            const auto& joint_group = calibration_groups[group_idx];
            std::cout << "\n======== 校准第 " << (group_idx + 1) << " 组关节 (从末端向近端) ========" << std::endl;
            std::cout << "关节索引: ";
            for (int joint_idx : joint_group) {
                std::cout << joint_idx << " ";
            }
            std::cout << std::endl;

            // 逐个模式：等待用户确认
            if (!auto_mode)
            {
                std::cout << "\n\033[33m按 Enter 开始校准第 " << (group_idx + 1) << " 组关节...\033[0m" << std::endl;
                std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
            }

            // 准备组内所有有效关节
            std::vector<int> valid_joints;
            std::vector<int> valid_arm_joint_indices;
            std::vector<double> target_limits;
            std::vector<int> directions;
            
            for (int joint_idx : joint_group)
            {
                // 检查是否是腿关节
                if (joint_idx >= leg_start_idx && joint_idx < leg_start_idx + num_leg_joints)
                {
                    int leg_joint_idx = joint_idx - leg_start_idx;
                    valid_joints.push_back(joint_idx);
                    valid_arm_joint_indices.push_back(-1);  // 标记为腿关节（不在arm_joint_idx范围内）
                    target_limits.push_back(leg_calibration_limits[leg_joint_idx]);
                    directions.push_back(leg_calibration_directions[leg_joint_idx]);
                    continue;
                }
                
                // 检查是否是需要校准的关节（手臂或头部）
                if (joint_idx >= arm_start_idx && joint_idx < arm_start_idx + num_arm_joints + num_head_joints)
                {
                    int arm_joint_idx = joint_idx - arm_start_idx;  // 在配置数组中的索引
                    int calibration_direction = arm_calibration_directions[arm_joint_idx];
                    
                    valid_joints.push_back(joint_idx);
                    valid_arm_joint_indices.push_back(arm_joint_idx);
                    target_limits.push_back(arm_calibration_limits[arm_joint_idx]);
                    directions.push_back(calibration_direction);
                    continue;
                }
            }
            
            if (valid_joints.empty())
            {
                std::cout << "本组没有需要校准的关节，跳过" << std::endl;
                continue;
            }
            
            std::cout << "\n本组共 " << valid_joints.size() << " 个关节需要校准" << std::endl;
            for (size_t i = 0; i < valid_joints.size(); ++i)
            {
                std::cout << "  关节 " << valid_joints[i] << ": 目标限位=" << target_limits[i] 
                          << "度, 方向=" << (directions[i] > 0 ? "正" : "负") << std::endl;
            }
            
            // 读取全身关节当前状态
            GetMotorData(joint_ids, current_all_joints);
            
            // 记录本组关节的起始位置。
            // 必须用“当前实际位置”作为插值起点：若仍用理论 calibration_start_pose，
            // 而实际关节还停在几度偏差处，目标要先花数百毫秒追上实际位置，
            // 期间位置几乎不变，0.4s 方差检测会误判限位并写入错误零点。
            std::vector<double> start_positions(valid_joints.size());
            std::vector<double> current_target_positions(valid_joints.size());
            std::vector<bool> joint_limit_reached(valid_joints.size(), false);
            std::vector<double> limit_positions(valid_joints.size(), 0.0);  // 记录到达限位时的位置
            std::vector<int> velocity_low_counts(valid_joints.size(), 0);
            // 防止错误逻辑零位导致关节从机械限位继续施力
            std::vector<CalibrationStallState> stall_states(valid_joints.size());
            
            // 用于位置方差检测的历史记录（循环队列）
            std::vector<std::deque<double>> position_history(valid_joints.size());
            
            for (size_t i = 0; i < valid_joints.size(); ++i)
            {
                int joint_idx = valid_joints[i];
                start_positions[i] = current_all_joints[joint_idx].position;
                current_target_positions[i] = start_positions[i];
                // 同步快照，保证后续回程/改零补偿与真实起点一致
                calibration_start_pose[joint_idx] = start_positions[i];
                
                std::cout << "  关节 " << joint_idx << " 起始位置(实际): " << start_positions[i] << " 度" << std::endl;
            }
            
            // 全身保持当前位置，避免把未校准关节强拉到过期快照
            for (size_t i = 0; i < num_joint; ++i)
            {
                target_all_joints[i].position = current_all_joints[i].position;
            }

            // 开始同时移动所有关节到限位
            auto start_time = std::chrono::steady_clock::now();
            bool all_limits_reached = false;
            std::vector<std::string> stop_reasons(valid_joints.size(), "未知");

            std::cout << "\n开始同时移动所有关节到限位..." << std::endl;

            while (!all_limits_reached)
            {
                // 读取全身关节的当前传感器数据
                GetMotorData(joint_ids, current_all_joints);
                
                auto current_time = std::chrono::steady_clock::now();
                double elapsed_time = std::chrono::duration<double>(current_time - start_time).count();
                
                // 构建实时状态打印字符串
                std::stringstream status_line;
                status_line << "\r[校准中 " << std::fixed << std::setprecision(1) << elapsed_time << "s] ";
                
                // 检查每个关节是否到达限位
                bool any_active = false;
                for (size_t i = 0; i < valid_joints.size(); ++i)
                {
                    int joint_idx = valid_joints[i];
                    
                    double current_pos = current_all_joints[joint_idx].position;
                    double current_vel = current_all_joints[joint_idx].velocity;
                    double raw_torque = current_all_joints[joint_idx].torque;
                    double current_torque = motorFeedbackToJointTorqueNm(joint_idx, raw_torque);
                    
                    if (joint_limit_reached[i])
                    {
                        // 已到达限位的关节显示为完成状态，并保持在限位位置
                        status_line << "J" << joint_idx << ":✓ ";
                        target_all_joints[joint_idx].position = limit_positions[i];
                        continue;
                    }
                    
                    // 记录位置历史用于方差计算
                    position_history[i].push_back(current_pos);
                    if (position_history[i].size() > POSITION_HISTORY_SAMPLES)
                    {
                        position_history[i].pop_front();
                    }
                    
                    // 添加当前关节的状态信息到打印字符串
                    status_line << "J" << joint_idx << ":[" 
                                << std::fixed << std::setprecision(1) 
                                << current_pos << "°, " 
                                << current_vel << "°/s, " 
                                << std::setprecision(2)
                                << current_torque << "Nm] Target:"
                                << current_target_positions[i] << "°,"  ;
                    
                    any_active = true;
                    
                    // 检查停止条件
                    bool limit_detected = false;
                    std::string detected_reason = "";
                    const double traveled = std::abs(current_pos - start_positions[i]);
                    const bool allow_motion_based_detect =
                        (elapsed_time >= MIN_LIMIT_DETECT_TIME) && (traveled >= MIN_LIMIT_TRAVEL_DEG);

                    const bool is_arm_joint =
                        joint_idx >= arm_start_idx &&
                        joint_idx < arm_start_idx + static_cast<int>(num_arm_joints);
                    // 限位搜索开始不足一秒或实际行程不足一度时启用快速保护
                    const bool in_startup_guard_window =
                        elapsed_time < MIN_LIMIT_DETECT_TIME || traveled < MIN_LIMIT_TRAVEL_DEG;
                    const bool supports_stall_guard =
                        motor_info.driver[joint_idx] == RUIWO ||
                        motor_info.driver[joint_idx] == EC_MASTER;
                    if (is_cali_ && is_arm_joint && supports_stall_guard &&
                        in_startup_guard_window)
                    {
                        const double position_error =
                            std::abs(current_target_positions[i] - current_pos);
                        const double torque_threshold = getCalibrationStallTorqueThreshold(joint_idx);
                        double position_change = 0.0;
                        if (updateCalibrationStallState(stall_states[i], current_time, current_pos,
                                                        position_error, current_torque,
                                                        torque_threshold, position_change))
                        {
                            std::ostringstream reason;
                            reason << "关节" << joint_idx + 1
                                   << "未产生有效位移且持续高力矩，疑似起点位于机械限位"
                                   << " (target=" << current_target_positions[i]
                                   << "deg, position=" << current_pos
                                   << "deg, velocity=" << current_vel
                                   << "deg/s, position_change=" << position_change
                                   << "deg/" << kCalibrationStallDurationSec
                                   << "s, torque=" << current_torque
                                   << "Nm, threshold=" << torque_threshold
                                   << "Nm, raw_feedback=" << raw_torque << ")";
                            std::cerr << "\n\033[31m[CalibrationStallGuard] " << reason.str()
                                      << "，本次标定禁止保存\033[0m" << std::endl;
                            if (!stopArmForCalibrationFailure())
                            {
                                std::cerr << "\033[31m[CalibrationStallGuard] 无法确认手臂已安全停止，正在触发紧急停机\033[0m"
                                          << std::endl;
                                std::raise(SIGINT);
                            }
                            else
                            {
                                std::cerr << "\033[31m[CalibrationStallGuard] 手臂已停止\033[0m" << std::endl;
                            }
                            // 退出前恢复前面分组已经调整的运行时零点
                            rollback_calibration_changes();
                            return false;
                        }
                    }
                    else
                    {
                        // 离开启动保护范围后丢弃未完成的观察窗口
                        stall_states[i].active = false;
                    }
                    
                    // 条件1: 电流超出阈值
                    // if (current_torque > CURRENT_THRESHOLD)
                    // {
                    //     limit_detected = true;
                    //     detected_reason = "电流超出阈值";
                    // }
                    // 条件2/3: 仅在“已经真正跑起来”后才允许用速度/方差判定限位，
                    // 避免启动阶段目标爬行、跟随滞后造成的假限位。
                    if (allow_motion_based_detect)
                    {
                        // 条件2: 速度持续低于阈值
                        if (std::abs(current_vel) < VELOCITY_THRESHOLD)
                        {
                            velocity_low_counts[i]++;
                            if (velocity_low_counts[i] >= VELOCITY_STABLE_SAMPLES)
                            {
                                limit_detected = true;
                                detected_reason = "速度持续低于阈值";
                            }
                        }
                        else
                        {
                            velocity_low_counts[i] = 0;
                        }
                        
                        // 条件3: 位置方差很小（0.4s内位置几乎不变）
                        if (!limit_detected && position_history[i].size() >= POSITION_HISTORY_SAMPLES)
                        {
                            // 计算位置方差
                            double mean = std::accumulate(position_history[i].begin(), position_history[i].end(), 0.0) / position_history[i].size();
                            double variance = 0.0;
                            for (double pos : position_history[i])
                            {
                                variance += (pos - mean) * (pos - mean);
                            }
                            variance /= position_history[i].size();
                            
                            if (variance < POSITION_VARIANCE_THRESHOLD)
                            {
                                limit_detected = true;
                                detected_reason = "检测到位置基本不变";
                            }
                        }
                    }
                    else
                    {
                        // 保护窗口内不累计低速计数，避免窗口结束瞬间被旧计数误触发
                        velocity_low_counts[i] = 0;
                    }
                    
                    // 条件4: 超时
                    if (!limit_detected && elapsed_time > TIMEOUT)
                    {
                        std::cerr << "\n\033[31m[CalibrationTimeout] 关节" << joint_idx + 1
                                  << "限位搜索超时"
                                  << " (position=" << current_pos
                                  << "deg, traveled=" << traveled
                                  << "deg, velocity=" << current_vel
                                  << "deg/s, torque=" << current_torque
                                  << "Nm)，本次标定失败\033[0m" << std::endl;

                        // 手臂限位搜索超时与堵转共用同一停止策略
                        if (is_arm_joint)
                        {
                            if (!stopArmForCalibrationFailure())
                            {
                                std::cerr << "\033[31m[CalibrationTimeout] 无法确认手臂已安全停止，正在触发紧急停机\033[0m"
                                          << std::endl;
                                std::raise(SIGINT);
                            }
                            else
                            {
                                std::cerr << "\033[31m[CalibrationTimeout] 手臂已停止\033[0m" << std::endl;
                            }
                        }
                        // 保留非手臂 Ruiwo 关节原有的超时处理
                        else if (motor_info.driver[joint_idx] == RUIWO)
                        {
                            int disable_result = -1;
                            if (ruiwo_actuator != nullptr)
                            {
                                disable_result = ruiwo_actuator->disable();
                            }
                            if (disable_result != 0)
                            {
                                std::cerr << "\033[31m[CalibrationTimeout] 手臂失能失败 code="
                                          << disable_result << "，正在触发紧急停机\033[0m" << std::endl;
                                std::raise(SIGINT);
                            }
                            else
                            {
                                std::cerr << "\033[31m[CalibrationTimeout] 手臂已失能\033[0m" << std::endl;
                            }
                        }

                        // 超时属于标定失败 退出前恢复本次运行时零点修改
                        rollback_calibration_changes();
                        return false;
                    }
                    
                    // 如果检测到限位，立即记录当前位置
                    if (limit_detected)
                    {
                        joint_limit_reached[i] = true;
                        limit_positions[i] = current_pos;  // 立即记录限位位置
                        stop_reasons[i] = detected_reason;
                        std::cout << "\n关节 " << joint_idx << " 到达限位 (" << detected_reason 
                                  << "), 位置: " << current_pos << " 度" << std::endl;
                    }
                    
                    // 如果还没到达限位，继续位置插值
                    if (!joint_limit_reached[i])
                    {
                        current_target_positions[i] += directions[i] * POSITION_STEP;
                        
                        // 更新全身关节中该关节的目标位置
                        target_all_joints[joint_idx].position = current_target_positions[i];
                    }
                }
                
                // 发送全身关节位置命令
                // 在 CSP 模式下，确保 EC_MASTER 和 RUIWO 电机有默认的 kp/kd
                setDefaultKpKd(target_all_joints, joint_ids);
                SetMotorPosition(joint_ids, target_all_joints);
                
                all_limits_reached = !any_active;
                
                // 打印实时状态（同一行刷新）
                std::cout << status_line.str() << std::flush;
                static int print_count = 0;
                if (print_count++ % 150 == 0)
                    std::cout << std::endl;
                // 控制频率
                usleep(1000000 / CONTROL_FREQUENCY);
            }
            
            std::cout << "\n所有关节已到达限位" << std::endl;
            
            // 使用记录的限位位置计算零点偏移
            for (size_t i = 0; i < valid_joints.size(); ++i)
            {
                int joint_idx = valid_joints[i];
                int arm_joint_idx = valid_arm_joint_indices[i];
                double final_position = limit_positions[i];  // 使用到达限位时立即记录的位置
                
                std::cout << "\n关节 " << joint_idx << " 校准结果:" << std::endl;
                std::cout << "  停止原因: " << stop_reasons[i] << std::endl;
                std::cout << "  实际检测到的限位位置: " << final_position << " 度" << std::endl;
                
                // 计算零点偏移
                double zero_offset = final_position - target_limits[i];
                std::cout << "  零点偏移 = " << final_position << " - " << target_limits[i] 
                          << " = " << zero_offset << " 度" << std::endl;
                
                // 判断是腿关节还是手臂/头部关节
                bool is_leg_joint = (joint_idx >= leg_start_idx && joint_idx < leg_start_idx + num_leg_joints);
                
                // 根据电机类型调整零点
                MotorDriveType motor_type = motor_info.driver[joint_idx];
                
                if (motor_type == MotorDriveType::EC_MASTER)
                {
                    // EC 电机：找到对应的EC索引
                    int ec_idx = -1;
                    int ec_count = 0;
                    for (int j = 0; j <= joint_idx; ++j)
                    {
                        if (motor_info.driver[j] == MotorDriveType::EC_MASTER)
                        {
                            if (j == joint_idx)
                            {
                                ec_idx = ec_count;
                                break;
                            }
                            ec_count++;
                        }
                    }
                    
                    if (ec_idx >= 0 && ec_idx < ec_motor_offsets.size())
                    {
                        double old_offset = ec_motor_offsets[ec_idx];
                        double new_ec_offset = old_offset + zero_offset;
                        ec_motor_offsets[ec_idx] = new_ec_offset;
                        modified_ec_offsets[ec_idx] = new_ec_offset;
                        
                        // 立即更新EC电机零点
                        setMotorPositionOffset(ec_motor_offsets);

                        // 改零后不要用“旧起始角 - zero_offset”做回程目标：
                        // 该值经常超出软件限位（如头部 yaw ±30），被 jointMoveTo clamp 后
                        // 关节停在错误逻辑角，下一轮会表现为方向正反交替、假限位。
                        // 统一回到配置安全姿态（新逻辑坐标系下）。
                        if (is_leg_joint)
                        {
                            int leg_joint_idx = joint_idx - leg_start_idx;
                            if (leg_joint_idx >= 0 && leg_joint_idx < static_cast<int>(leg_calibration_safe_pose.size()))
                            {
                                calibration_start_pose[joint_idx] = leg_calibration_safe_pose[leg_joint_idx];
                            }
                        }
                        else
                        {
                            calibration_start_pose[joint_idx] = calibration_safe_pose[joint_idx];
                        }
                        
                        if (is_leg_joint)
                        {
                            std::cout << "  已更新腿部 EC 电机[" << ec_idx << "] 零点偏移: " 
                                      << old_offset << " -> " << new_ec_offset << " 度" << std::endl;
                        }
                        else
                        {
                            std::cout << "  已更新 EC 电机[" << ec_idx << "] 零点偏移: " 
                                      << old_offset << " -> " << new_ec_offset << " 度" << std::endl;
                        }
                        std::cout << "  回程目标改为安全姿态: " << calibration_start_pose[joint_idx] << " 度" << std::endl;
                    }
                    else
                    {
                        std::cerr << "  错误：无法找到 EC 电机索引" << std::endl;
                    }
                }
                else if (motor_type == MotorDriveType::RUIWO)
                {
                    if (ruiwo_actuator != nullptr)
                    {
                        // RUIWO 电机：找到对应的RUIWO索引（在所有RUIWO电机中的索引）
                        int ruiwo_idx = -1;
                        int ruiwo_count = 0;
                        for (int j = 0; j <= joint_idx; ++j)
                        {
                            if (motor_info.driver[j] == MotorDriveType::RUIWO)
                            {
                                if (j == joint_idx)
                                {
                                    ruiwo_idx = ruiwo_count;
                                    break;
                                }
                                ruiwo_count++;
                            }
                        }
                        
                        if (ruiwo_idx >= 0)
                        {
                            ruiwo_actuator->adjustZeroPosition(ruiwo_idx, zero_offset * M_PI / 180.0);
                            // 累计增量，便于放弃保存时回滚（注意是 +=，不是 =）
                            modified_ruiwo_offsets[ruiwo_idx] += zero_offset;
                            // 改零后回到配置安全姿态（新逻辑坐标系），避免
                            // start-zero_offset 超出软件限位被 clamp，污染下一轮方向/起点
                            if (is_leg_joint)
                            {
                                int leg_joint_idx = joint_idx - leg_start_idx;
                                if (leg_joint_idx >= 0 && leg_joint_idx < static_cast<int>(leg_calibration_safe_pose.size()))
                                {
                                    calibration_start_pose[joint_idx] = leg_calibration_safe_pose[leg_joint_idx];
                                }
                            }
                            else
                            {
                                calibration_start_pose[joint_idx] = calibration_safe_pose[joint_idx];
                            }
                            std::cout << "  已调整 ruiwo 电机[" << ruiwo_idx << "] (关节 " << joint_idx 
                                      << ") 的零点偏移: " << zero_offset << " 度" << std::endl;
                            std::cout << "  回程目标改为安全姿态: " << calibration_start_pose[joint_idx] << " 度" << std::endl;
                        }
                        else
                        {
                            std::cerr << "  错误：无法找到 RUIWO 电机索引" << std::endl;
                        }
                    }
                }
                else
                {
                    std::cout << "  警告：关节 " << joint_idx << " 的电机类型未知，跳过零点调整" << std::endl;
                }
            }
            
            
            // 每组校准完成后，也移回到校准初始姿态
            std::cout << "\n第 " << (group_idx + 1) << " 组关节校准完成，移回到初始姿态..." << std::endl;
            
            // 检查当前组是否包含腿部前四个关节（索引0-3），如果是则使用较慢的速度
            bool contains_leg_first_four = false;
            for (int joint_idx : joint_group)
            {
                if (joint_idx >= leg_start_idx && joint_idx < leg_start_idx + 4)
                {
                    contains_leg_first_four = true;
                    break;
                }
            }
            
            double return_speed = contains_leg_first_four ? 15.0 : 30.0;  // 前四个腿部关节使用较慢速度
            if (contains_leg_first_four)
            {
                std::cout << "检测到腿部前四个关节，使用较慢速度（15度/秒）..." << std::endl;
            }
            if (!jointMoveTo(calibration_start_pose, return_speed, dt_))
            {
                std::cerr << "\033[31m回到校准安全姿态失败，本次标定已中止\033[0m" << std::endl;
                rollback_calibration_changes();
                return false;
            }
            usleep(100000); // 等待500ms
        }

        // 步骤3: 移动校准关节到安全姿态或零位
        if (leg_only)
        {
            std::cout << "\n======== 所有关节校准完成，移动腿部到安全姿态 ========" << std::endl;
            // 基于calibration_start_pose生成safe_pose，将腿部设为安全姿态
            std::vector<double> safe_pose = calibration_start_pose;
            for (size_t i = 0; i < num_leg_joints; ++i)
            {
                safe_pose[leg_start_idx + i] = leg_calibration_safe_pose[i];
            }
            
            std::cout << "移动腿部到安全姿态..." << std::endl;
            if (!jointMoveTo(safe_pose, 15.0, dt_))
            {
                std::cerr << "\033[31m移动腿部到安全姿态失败，本次标定已中止\033[0m" << std::endl;
                rollback_calibration_changes();
                return false;
            }
            std::cout << "腿部已移动到安全姿态" << std::endl;
        }
        else if (head_only)
        {
            std::cout << "\n======== 所有关节校准完成，移动头部到零位 ========" << std::endl;
            // 基于calibration_start_pose生成zero_pose，将头部设为0
            std::vector<double> zero_pose = calibration_start_pose;
            int head_start_idx = arm_start_idx + num_arm_joints;
            for (size_t i = 0; i < num_head_joints; ++i)
            {
                zero_pose[head_start_idx + i] = 0.0;
            }
            
            std::cout << "移动头部到零位..." << std::endl;
            if (!jointMoveTo(zero_pose, 30.0, dt_))
            {
                std::cerr << "\033[31m移动头部到零位失败，本次标定已中止\033[0m" << std::endl;
                rollback_calibration_changes();
                return false;
            }
            std::cout << "头部已移动到零位" << std::endl;
        }
        else
        {
            std::cout << "\n======== 所有关节校准完成，移动手臂、头部和（可选）腿部到零位 ========" << std::endl;
            // 基于calibration_start_pose生成zero_pose，将手臂和头部设为0
            std::vector<double> zero_pose = calibration_start_pose;
            for (size_t i = 0; i < num_arm_joints + num_head_joints; ++i)
            {
                zero_pose[arm_start_idx + i] = 0.0;
            }
            
            // 如果包含腿部，将腿部设为安全姿态
            if (calibrate_leg)
            {
                for (size_t i = 0; i < num_leg_joints; ++i)
                {
                    zero_pose[leg_start_idx + i] = leg_calibration_safe_pose[i];
                }
                std::cout << "移动手臂、头部到零位，腿部到安全姿态..." << std::endl;
            }
            else
            {
                std::cout << "移动手臂和头部到零位..." << std::endl;
            }
            
            if (!jointMoveTo(zero_pose, 30.0, dt_))
            {
                std::cerr << "\033[31m移动到零位失败，本次标定已中止\033[0m" << std::endl;
                rollback_calibration_changes();
                return false;
            }
            std::cout << "已移动到零位" << std::endl;
        }

        // 步骤4: 保存确认
        std::cout << "\n======== 校准完成，等待保存确认 ========" << std::endl;
        std::cout << "已修改 " << modified_ec_offsets.size() << " 个 EC 电机零点" << std::endl;
        std::cout << "已修改 " << modified_ruiwo_offsets.size() << " 个 ruiwo 电机零点" << std::endl;
        
        std::cout << "\n\033[33m按 's' 保存校准结果，按其他键放弃保存...\033[0m" << std::endl;
        
        char save_choice;
        std::cin >> save_choice;
        std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
        
        bool calibration_saved = false;
        if (save_choice == 's' || save_choice == 'S')
        {
            std::cout << "\n正在保存校准结果..." << std::endl;
            
            // 保存 EC 电机零点
            if (!modified_ec_offsets.empty())
            {
                // EC电机零点已在校准时实时更新，这里只需保存到文件
                saveOffset();  // 保存EC电机零点到文件
                std::cout << "✓ EC 电机零点已保存 (" << modified_ec_offsets.size() << " 个)" << std::endl;
            }
            
            // 保存 ruiwo 电机零点
            if (!modified_ruiwo_offsets.empty() && ruiwo_actuator != nullptr)
            {
                ruiwo_actuator->saveZeroPosition();
                std::cout << "✓ ruiwo 电机零点已保存 (" << modified_ruiwo_offsets.size() << " 个)" << std::endl;
            }
            
            std::cout << "\n\033[32m======== 手臂限位校准完成并已保存 ========\033[0m" << std::endl;
            calibration_saved = true;
        }
        else
        {
            std::cout << "\n\033[31m放弃保存，正在恢复原始零点...\033[0m" << std::endl;
            rollback_calibration_changes();
            
            std::cout << "\n\033[32m======== 手臂限位校准已取消 ========\033[0m" << std::endl;
            calibration_saved = false;
        }

        // 下肢标定后，允许手动重新标定腰部电机（下肢第4个关节，即关节4）
        constexpr int kWaistLegJointOffset = 3;  // 下肢第4个关节（0-based 索引3）
        bool waist_calibrated = true;
        if ((calibrate_leg || leg_only) && isLunbiModule(robot_module_) && num_leg_joints > kWaistLegJointOffset)
        {
            std::cout << "\n\033[33m腰部电机（关节4）的限位标定，是否准确？是否需要手动重新标定？(y/n)\033[0m" << std::endl;
            char waist_choice = '\0';
            std::cin >> waist_choice;
            std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
            if (waist_choice == 'y' || waist_choice == 'Y')
            {
                if (!manualCalibrateSingleLegJoint(leg_start_idx + kWaistLegJointOffset))
                {
                    waist_calibrated = false;
                    std::cout << "\033[31m[HardwarePlant] 腰部电机手动标定失败，请重试\033[0m" << std::endl;
                }
            }
        }

        return calibration_saved && waist_calibrated;
    }

    void HardwarePlant::calibrateBipedLoop()
    {
        std::vector<double> moving_pos(num_joint, 0);
        double cali_tau_limit = 3;
        this->jointMoveTo(moving_pos, 60.0, dt_, cali_tau_limit);
        initial_input_cmd_ = '\0';
        int left_right = 0;
        int motor_id = 1;
        std::ostringstream cali_tips_oss;
        cali_tips_oss << "\033[32m*******校准腿部电机编码器偏移*******\n"
                      << "1. 输入 'l' 或 'r' 选择要校准的是左腿或右腿。\n"
                      << "2. 输入 1-6 选择要校准的电机。\n"
                      << "3. 输入 'w' 增加电机编码器的轮数，输入 's' 减少电机编码器的轮数（注意：电机会移动！）\n"
                      << "4. 输入 'c' 保存校准结果。\n"
                      << "5. 输入 '<Enter>' 显示当前选择，输入 'h' 再次显示此提示，输入 'q' 退出校准。\033[0m\n";
        std::cout << cali_tips_oss.str() << std::endl;
        auto print_selection = [&]()
        {
            std::cout << "选中" << ((left_right == 0) ? "左腿" : "右腿") << std::to_string(motor_id) + "号电机." << std::endl;
        };
        while (1)
        {
            if (kbhit())
            {
                initial_input_cmd_ = '\0';
                initial_input_cmd_ = getchar();
            }
            if (initial_input_cmd_ == 'h')
            {
                std::cout << cali_tips_oss.str() << std::endl;
                initial_input_cmd_ = '\0';
            }
            else if (initial_input_cmd_ == 'l')
            {
                left_right = 0;
                print_selection();
            }
            else if (initial_input_cmd_ == 'r')
            {
                left_right = 1;
                print_selection();
            }
            else if (initial_input_cmd_ >= '1' && initial_input_cmd_ <= '6')
            {
                motor_id = initial_input_cmd_ - '0';
                print_selection();
            }
            if (initial_input_cmd_ == 'w')
            {
                std::cout << "增加 " << ((left_right == 0) ? "左腿" : "右腿") << std::to_string(motor_id) + "号 " + "电机的编码器轮数." << std::endl;
                this->calibrateMotor(left_right * 6 + motor_id - 1, 1);
                std::fill(moving_pos.begin(), moving_pos.end(), 0.0);
                this->jointMoveTo(moving_pos, 60.0, dt_, cali_tau_limit);
            }
            else if (initial_input_cmd_ == 's')
            {
                std::cout << "减少 " << ((left_right == 0) ? "左腿" : "右腿") << std::to_string(motor_id) + "号 " + "电机的编码器轮数." << std::endl;
                this->calibrateMotor(left_right * 6 + motor_id - 1, -1);
                std::fill(moving_pos.begin(), moving_pos.end(), 0.0);
                this->jointMoveTo(moving_pos, 60.0, dt_, cali_tau_limit);
            }
            else if (initial_input_cmd_ == 'c')
            {
                std::cout << "保存校准结果." << std::endl;
                this->calibrateMotor(left_right * 6 + motor_id - 1, 0, true);
                initial_input_cmd_ = '\0';
            }
            else if (initial_input_cmd_ == 'q')
            {
                std::cout << "退出校准." << std::endl;
                break;
            }
            else if (initial_input_cmd_ == '\n')
            {
                print_selection();
            }
            initial_input_cmd_ = '\0';
            usleep(2000);
        }
    }

        void HardwarePlant::calibrateWheelLoop()
    {
        std::vector<double> moving_pos(num_joint, 0);
        double cali_tau_limit = 3;
        this->jointMoveTo(moving_pos, 60.0, dt_, cali_tau_limit);
        initial_input_cmd_ = '\0';
        int motor_id = 1;

        std::ostringstream cali_tips_oss;
        cali_tips_oss << "\033[32m*******校准腿部电机编码器偏移*******\n"
                      << "1. 输入 1-4 选择要校准的电机。\n"
                      << "2. 输入 'w' 增加电机编码器的轮数，输入 's' 减少电机编码器的轮数（注意：电机会移动！）\n"
                      << "3. 输入 'c' 保存校准结果。\n"
                      << "4. 输入 '<Enter>' 显示当前选择，输入 'h' 再次显示此提示，输入 'q' 退出校准。\033[0m\n";
        std::cout << cali_tips_oss.str() << std::endl;

        auto print_selection = [&]()
        {
            std::cout << "选中" << std::to_string(motor_id) + "号电机." << std::endl;
        };

        auto get_motor_id = [&]() -> int
        {
            return motor_id - 1;
        };

        auto print_operation = [&](const std::string& operation)
        {
            std::cout << operation << std::to_string(motor_id) + "号电机的编码器轮数." << std::endl;
        };

        while (true)
        {
            if (kbhit())
            {
                initial_input_cmd_ = '\0';
                initial_input_cmd_ = getchar();
            }
            if (initial_input_cmd_ == 'h')
            {
                std::cout << cali_tips_oss.str() << std::endl;
                initial_input_cmd_ = '\0';
            }
            else if (initial_input_cmd_ >= '1' && initial_input_cmd_ <= '4')
            {
                motor_id = initial_input_cmd_ - '0';
                print_selection();
            }
            else if (initial_input_cmd_ == 'w')
            {
                print_operation("增加 ");
                this->calibrateMotor(get_motor_id(), 1);
                std::fill(moving_pos.begin(), moving_pos.end(), 0.0);
                this->jointMoveTo(moving_pos, 60.0, dt_, cali_tau_limit);
            }
            else if (initial_input_cmd_ == 's')
            {
                print_operation("减少 ");
                this->calibrateMotor(get_motor_id(), -1);
                std::fill(moving_pos.begin(), moving_pos.end(), 0.0);
                this->jointMoveTo(moving_pos, 60.0, dt_, cali_tau_limit);
            }
            else if (initial_input_cmd_ == 'c')
            {
                std::cout << "保存校准结果." << std::endl;
                this->calibrateMotor(get_motor_id(), 0, true);
                initial_input_cmd_ = '\0';
            }
            else if (initial_input_cmd_ == 'q')
            {
                std::cout << "退出校准." << std::endl;
                break;
            }
            else if (initial_input_cmd_ == '\n')
            {
                print_selection();
            }
            initial_input_cmd_ = '\0';
            usleep(2000);
        }
    }

    // 辅助函数：从配置文件读取反转电机地址列表
    // 双CAN: 从canbus_device_cofig.yaml读取每个电机的negtive字段
    // 单CAN: 从config.yaml读取negtive_address列表
    std::set<int> HardwarePlant::loadNegativeMotorAddresses(HighlyDynamic::CanbusWiringType canbus_mode) const
    {
        std::set<int> negtive_addresses;

        // 双CAN总线：从canbus_device_cofig.yaml读取每个电机的negtive字段
        if (canbus_mode == HighlyDynamic::CanbusWiringType::DUAL_BUS) {
            try {
                std::string config_path = canbus_sdk::ConfigParser::getDefaultConfigFilePath();
                canbus_sdk::ConfigParser parser;
                if (parser.parseFromFile(config_path)) {
                    // 遍历所有CAN总线，收集negtive=true的电机ID
                    for (const auto& canbus_config : parser.getCanBusConfigs()) {
                        auto motor_devices = parser.getDevices(canbus_config.name, canbus_sdk::DeviceType::MOTOR);
                        for (const auto& device : motor_devices) {
                            if (device.negtive) {
                                negtive_addresses.insert(device.device_id);
                            }
                        }
                    }
                    std::cout << "[HWPlantInit] Loaded " << negtive_addresses.size()
                              << " negative motor addresses from canbus_device_cofig.yaml (dual bus)" << std::endl;
                } else {
                    std::cout << "[HWPlantInit] Warning: Failed to parse canbus_device_cofig.yaml: "
                              << config_path << std::endl;
                }
            } catch (const std::exception& e) {
                std::cout << "[HWPlantInit] Warning: Failed to load canbus_device_cofig.yaml: "
                          << e.what() << ", using default adjustment direction" << std::endl;
            }
            return negtive_addresses;
        }

        // 单CAN总线：从config.yaml读取negtive_address列表
        try {
            const char* sudo_user = getenv("SUDO_USER");
            std::string home_dir;
            if (sudo_user != nullptr) {
                struct passwd* pw = getpwnam(sudo_user);
                if (pw != nullptr) {
                    home_dir = pw->pw_dir;
                }
            }
            if (home_dir.empty()) {
                home_dir = getenv("HOME");
            }

            if (!home_dir.empty()) {
                std::string config_path = home_dir + "/.config/lejuconfig/config.yaml";
                YAML::Node motor_config = YAML::LoadFile(config_path);
                const YAML::Node& negtive_address = motor_config["negtive_address"];
                if (negtive_address.size()) {
                    const YAML::Node& negtive_motor_list = negtive_address[0];
                    for (size_t j = 0; j < negtive_motor_list.size(); ++j) {
                        int address = negtive_motor_list[j].as<int>();
                        negtive_addresses.insert(address);
                    }
                }
                std::cout << "[HWPlantInit] Loaded " << negtive_addresses.size()
                          << " negative motor addresses from config.yaml (single bus)" << std::endl;
            }
        } catch (const std::exception& e) {
            std::cout << "[HWPlantInit] Warning: Failed to load negtive_address from config: "
                      << e.what() << ", using default adjustment direction" << std::endl;
        }
        return negtive_addresses;
    }

    int8_t HardwarePlant::HWPlantInit()
    {
        std::cout << "[HWPlantInit] Starting initialization..." << std::endl;
        
        is_cali_ = hardware_param_.cali;
        hardware_status_ = kStatusNotInAnyState;
        
        if (hardware_param_.cali_leg)
        {
            hardware_status_ = -2;
        }
        
        std::cout << "[HWPlantInit] Checking for Ruiwo motors..." << std::endl;
        auto has_ruiwo = std::find(motor_info.driver.begin(), motor_info.driver.end(), MotorDriveType::RUIWO);
        if (has_ruiwo != motor_info.driver.end())
        {
            bool cali_arm = hardware_param_.cali_arm;
            if (cali_arm) {
                // hardware_status_ 由 hardware_node 中 jointMoveToPrepGoal 之后统一设置，此处不再提前赋值
            }
            std::cout << "[HWPlantInit] cali_arm: " << cali_arm << std::endl;
            
            std::string ruiwo_path = GetAbsolutePathHW("lib/ruiwo_controller");
            std::cout << "[HWPlantInit] Ruiwo controller path: " << ruiwo_path << std::endl;
            
            try {
                HighlyDynamic::CanbusWiringType canbus_mode = motor_info.getCanbusWiringType(rb_version_);
                if(canbus_mode == HighlyDynamic::CanbusWiringType::DUAL_BUS
                  || canbus_mode == HighlyDynamic::CanbusWiringType::SINGLE_BUS) {
                    // dual bus
                    std::cout << "[HWPlantInit] Creating RuiWoActuator instance with DUAL_BUS" << std::endl;
                    auto config_file = canbus_sdk::ConfigParser::getDefaultConfigFilePath();
                    std::cout << "[HWPlantInit] MotorevoActuator config file: " << config_file << std::endl;
                    int control_frequency = 250;
                    if(rb_version_.start_with(1) && canbus_mode == HighlyDynamic::CanbusWiringType::DUAL_BUS) {
                        printf("\033[33m Roban2 双 Can 总线, 手臂电机控制频率 500hz \033[0m\n");
                        control_frequency = 500;
                    }
                    ruiwo_actuator = new motorevo::MotorevoActuator(config_file, cali_arm, control_frequency);
                    
                    // 检查是否为气泵末端类型（ROBOT_MODULE为LUNBI_V62，且末端类型包含qibeng）
                    bool qibeng_enabled = robot_module_ == "LUNBI_V62" &&
                                         std::find(motor_info.end_effector_type.begin(), motor_info.end_effector_type.end(), EndEffectorType::qibeng) != motor_info.end_effector_type.end();
                    if (qibeng_enabled) {
                        std::cout << "[HWPlantInit] qibeng end effector detected, enabling arm offset" << std::endl;
                    }
                    
                    // 设置电机回零时的目标初始位置。必须在 initialize() 之前设置，因为其内部会调用 moveToZero()。
                    // use_custom_arm_prepare 时整臂去 yaml；否则保持原有 Roban1/气泵偏置，未列出的电机回 0。
                    auto* motorevo_actuator = dynamic_cast<motorevo::MotorevoActuator*>(ruiwo_actuator);
                    const bool customBootArm =
                        motorevo_actuator != nullptr && !cali_arm &&
                        static_cast<int>(boot_arm_target_rad_.size()) == num_arm_joints &&
                        num_arm_joints == 14 &&
                        std::all_of(boot_arm_target_rad_.begin(), boot_arm_target_rad_.end(),
                                    [](double q) { return std::isfinite(q); });
                    if (customBootArm) {
                        std::map<motorevo::MotorId, float> target_positions;
                        for (int i = 0; i < num_arm_joints; ++i) {
                            // 电机 1-7 左臂，8-14 右臂，与 can 配置 Larm/Rarm_joint 一致。头保持 0。
                            target_positions[static_cast<motorevo::MotorId>(i + 1)] =
                                static_cast<float>(boot_arm_target_rad_[i]);
                        }
                        motorevo_actuator->setInitialTargetPositions(target_positions);
                        std::cout << "[HWPlantInit] use_custom_arm_prepare: moveToZero arm target from yaml, "
                                  << num_arm_joints << " joints" << std::endl;
                    } else if (motorevo_actuator != nullptr && rb_version_.start_with(1) && rb_version_.minor() <= 6 && !cali_arm) {
                        std::map<motorevo::MotorId, float> target_positions;

                        // 设置电机初始位置：0x02加十度，0x06减十度
                        constexpr float adjustment_rad = ROBAN1_INITIAL_TARGET_ADJUSTMENT_DEG * M_PI / 180.0f;  // 角度转换为弧度
                        target_positions[LEFT_ARM_2_MOTOR_ID] = adjustment_rad;      // 0x02电机加十度
                        target_positions[RIGHT_ARM_2_MOTOR_ID] = -adjustment_rad;    // 0x06电机减十度

                        motorevo_actuator->setInitialTargetPositions(target_positions);
                        std::cout << "[HWPlantInit] Roban1 series detected (non-calibration mode), set initial target positions: motor 0x02 -> +"
                                  << ROBAN1_INITIAL_TARGET_ADJUSTMENT_DEG << " deg, motor 0x06 -> -" << ROBAN1_INITIAL_TARGET_ADJUSTMENT_DEG << " deg" << std::endl;
                    } else if (cali_arm) {
                        std::cout << "[HWPlantInit] Calibration mode enabled, motors will return to 0 degrees (no angle adjustment)" << std::endl;
                    }

                    // 如果末端类型为qibeng，为5号和12号电机添加90度偏移，避免与气泵末端干涉
                    // 启动时候：5号比标准零点-90度偏置，12号比标准零点+90度偏置
                    if (motorevo_actuator != nullptr && qibeng_enabled && !cali_arm && !customBootArm) {
                        std::map<motorevo::MotorId, float> target_positions;
                        constexpr float offset_deg_rad = 90.0f * M_PI / 180.0f;
                        target_positions[5] = -offset_deg_rad;
                        target_positions[12] = offset_deg_rad;
                        motorevo_actuator->setInitialTargetPositions(target_positions);
                        std::cout << "[HWPlantInit] qibeng enabled, set initial target positions: motor 0x05 -> -90 deg, motor 0x0C -> +90 deg" << std::endl;
                    }
                }
                else {
                    // unknown --> 使用旧的 kuavo4pro 上的控制方式
                    std::cout << "[HWPlantInit] Creating RuiWoActuator instance..." << std::endl;
                    ruiwo_actuator = new RuiWoActuator(GetAbsolutePathHW("lib/ruiwo_controller"), cali_arm);
                    std::cout << "[HWPlantInit] RuiWoActuator instance created" << std::endl;

                    if ((isLunbiModule(robot_module_)) && !cali_arm)
                    {
                        std::vector<float> init_arm_pos = {0.0f, 0.0f, -0.5236f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, -0.5236f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
                        if (static_cast<int>(boot_arm_target_rad_.size()) == num_arm_joints && num_arm_joints == 14 &&
                            std::all_of(boot_arm_target_rad_.begin(), boot_arm_target_rad_.end(),
                                        [](double q) { return std::isfinite(q); }))
                        {
                            for (int i = 0; i < num_arm_joints; ++i) {
                                init_arm_pos[i] = static_cast<float>(boot_arm_target_rad_[i]);
                            }
                            std::cout << "[HWPlantInit] use_custom_arm_prepare: legacy ruiwo init position from yaml" << std::endl;
                        }
                        ruiwo_actuator->setInitPosition(init_arm_pos);
                    }
                }
            
                ruiwo_actuator->setSkipBootMoveToZero(skip_boot_move_to_zero_);
                if (skip_boot_move_to_zero_) {
                    std::cout << "[HWPlantInit] skip_boot_move_to_zero enabled (use_sit_init)" << std::endl;
                }
                std::cout << "[HWPlantInit] Initializing RuiWoActuator..." << std::endl;
                auto ret = ruiwo_actuator->initialize();
                if (ret != 0)
                {
                        std::cerr << "[HWPlantInit] RuiWoActuator initialization failed with code: " << ret << std::endl;
                    exit(1);
                }
                std::cout << "[HWPlantInit] RuiWoActuator initialized successfully" << std::endl;
                
                // 检查是否为气泵末端类型（ROBOT_MODULE为LUNBI_V62，且末端类型包含qibeng）
                bool qibeng_enabled = robot_module_ == "LUNBI_V62" &&
                                     std::find(motor_info.end_effector_type.begin(), motor_info.end_effector_type.end(), EndEffectorType::qibeng) != motor_info.end_effector_type.end();
                if (qibeng_enabled) {
                    std::cout << "[HWPlantInit] qibeng end effector detected, enabling arm offset" << std::endl;
                }
                
                // 设置v15和v16版本机器人手臂零点偏移调整参数(仅在校准模式下)
                if ((rb_version_.start_with(1, 5) || rb_version_.start_with(1, 6)) && cali_arm) {

                    std::map<size_t, double> zero_offset_adjustments;
                    constexpr double adjustment_rad = ARM_ZERO_OFFSET_ADJUSTMENT_DEG * M_PI / 180.0;  // 角度转换为弧度

                    // 读取配置文件获取反转电机地址列表（传入canbus_mode参数）
                    HighlyDynamic::CanbusWiringType canbus_mode = motor_info.getCanbusWiringType(rb_version_);
                    std::set<int> negtive_addresses = loadNegativeMotorAddresses(canbus_mode);
                    
                    // 检查电机是否在反转列表中
                    bool is_negtive_left = negtive_addresses.find(LEFT_ARM_2_MOTOR_ADDRESS) != negtive_addresses.end();
                    bool is_negtive_right = negtive_addresses.find(RIGHT_ARM_2_MOTOR_ADDRESS) != negtive_addresses.end();

                    // 根据电机是否反转来调整零点偏移方向
                    zero_offset_adjustments[LEFT_ARM_2_JOINT_INDEX] = is_negtive_left ? adjustment_rad : -adjustment_rad;
                    zero_offset_adjustments[RIGHT_ARM_2_JOINT_INDEX] = is_negtive_right ? -adjustment_rad : adjustment_rad;

                    // 如果末端类型为qibeng，为5号和12号电机添加90度零点偏移调整，避免与气泵末端干涉
                    // 标定时候：5号读到的值+90度，12号-90度作为电机零点
                    if (qibeng_enabled) {
                        constexpr double offset_deg_rad = 90.0f * M_PI / 180.0f;
                        zero_offset_adjustments[4] = offset_deg_rad;
                        zero_offset_adjustments[11] = -offset_deg_rad;
                        std::cout << "[HWPlantInit] qibeng enabled, set zero offset adjustments: joint 4 -> +90 deg, joint 11 -> -90 deg" << std::endl;
                    }
                    
                    ruiwo_actuator->setZeroOffsetAdjustments(zero_offset_adjustments);
                    
                    // 调用底层方法将处理后的零点值写入到配置文件
                    std::cout << "[HWPlantInit] Saving adjusted zero points to file via RuiWoActuator::saveZeroPosition()..." << std::endl;
                    ruiwo_actuator->saveZeroPosition();
 
                }
                
                // 如果末端类型为qibeng且cali_arm为true，为5号和12号电机添加90度零点偏移调整
                // 标定时候：5号读到的值+90度，12号-90度作为电机零点
                if (qibeng_enabled && cali_arm) {
                    std::map<size_t, double> zero_offset_adjustments;
                    constexpr double offset_deg_rad = 90.0f * M_PI / 180.0f;
                    zero_offset_adjustments[4] = offset_deg_rad;
                    zero_offset_adjustments[11] = -offset_deg_rad;
                    ruiwo_actuator->setZeroOffsetAdjustments(zero_offset_adjustments);
                    std::cout << "[HWPlantInit] qibeng enabled, set zero offset adjustments for all versions: joint 4 -> +90 deg, joint 11 -> -90 deg" << std::endl;
                    
                    // 调用底层方法将处理后的零点值写入到配置文件
                    std::cout << "[HWPlantInit] Saving adjusted zero points to file via RuiWoActuator::saveZeroPosition()..." << std::endl;
                    ruiwo_actuator->saveZeroPosition();
                }
                
                // 设置ruiwo电机的kp和kd参数
                const auto &running_settings = kuavo_settings_.running_settings;
                const bool use_vr_ruiwo_gains = running_settings.use_vr_arm_kpkd &&
                                                !running_settings.vr_ruiwo_kp.empty() &&
                                                !running_settings.vr_ruiwo_kd.empty() &&
                                                running_settings.vr_ruiwo_kp.size() == running_settings.vr_ruiwo_kd.size();
                const auto &active_ruiwo_kp = use_vr_ruiwo_gains ? running_settings.vr_ruiwo_kp : running_settings.ruiwo_kp;
                const auto &active_ruiwo_kd = use_vr_ruiwo_gains ? running_settings.vr_ruiwo_kd : running_settings.ruiwo_kd;
                if (!active_ruiwo_kp.empty() && !active_ruiwo_kd.empty() && active_ruiwo_kp.size() == active_ruiwo_kd.size()) {
                    
                    std::vector<int> joint_indices;
                    std::vector<double> kp_values, kd_values;
                    
                    // 创建关节索引列表 (0-based)
                    for (size_t i = 0; i < active_ruiwo_kp.size(); ++i) {
                        joint_indices.push_back(i);
                        kp_values.push_back(active_ruiwo_kp[i]);
                        kd_values.push_back(active_ruiwo_kd[i]);
                    }
                    
                    std::cout << "[HWPlantInit] Setting RuiWo joint gains from "
                              << (use_vr_ruiwo_gains ? "vr_ruiwo_kp/kd" : "ruiwo_kp/kd")
                              << "..." << std::endl;
                    ruiwo_actuator->set_joint_gains(joint_indices, kp_values, kd_values);
                    std::cout << "[HWPlantInit] RuiWo joint gains set successfully" << std::endl;
                } else {
                    std::cout << "[HWPlantInit] Warning: active RuiWo gains not found in config, using default values" << std::endl;
                }
                
                if(1 == hardware_param_.teach_pendant_){
                    ruiwo_actuator->set_teach_pendant_mode(1);
                }
            } catch (const std::exception& e) {
                std::cerr << "[HWPlantInit] Exception during RuiWoActuator setup: " << e.what() << std::endl;
                exit(1);
            } catch (...) {
                std::cerr << "[HWPlantInit] Unknown exception during RuiWoActuator setup" << std::endl;
                exit(1);
            }
        }

        std::cout << "[HWPlantInit] Initializing end effectors..." << std::endl;
        initEndEffector();

        actuatorsInterfaceSetup("real", &actuators);
        // set ec_master encoder range
        std::vector<uint32_t> ecMasterEncoderRange;
        std::vector<uint16_t> ec_master_nil_ids{};
        countECMasters = 0;

        for (size_t i = 0; i < motor_info.driver.size(); ++i)
        {
            if (motor_info.driver[i] == EC_MASTER)
            {
                int id = ec_index_map_[i];
                if (motor_info.motors_exist[i]) {
                    ecMasterEncoderRange.push_back(motor_info.encoder_range[i]);
                    countECMasters ++;
                }
                if(motor_info.motors_disable[i]) {
                    ec_master_nil_ids.push_back(id);
                }
            }
        }

        // 定义对应的拓扑映射
        initRobotModule();

        // add ignore ids for ec master.
        actuators.addIgnore(ec_master_nil_ids.data(), ec_master_nil_ids.size());       
        actuators.setEncoderRange(ecMasterEncoderRange.data(), ecMasterEncoderRange.size());

        const auto &running_settings = kuavo_settings_.running_settings;
        const bool use_vr_joint_gains = running_settings.use_vr_arm_kpkd &&
                                        !running_settings.vr_joint_kp.empty() &&
                                        !running_settings.vr_joint_kd.empty() &&
                                        running_settings.vr_joint_kp.size() == running_settings.vr_joint_kd.size();
        std::vector<int32_t> robot_running_mode_joint_kp =
            use_vr_joint_gains ? running_settings.vr_joint_kp : running_settings.joint_kp;
        std::vector<int32_t> robot_running_mode_joint_kd =
            use_vr_joint_gains ? running_settings.vr_joint_kd : running_settings.joint_kd;
        std::cout << "[HWPlantInit] Using "
                  << (use_vr_joint_gains ? "VR" : "legacy")
                  << " EC joint gains for actuator init" << std::endl;
        actuators.setJointKp(robot_running_mode_joint_kp);
        actuators.setJointKd(robot_running_mode_joint_kd);
        
        // only half up body.
        bool flag_half_up_body = kuavo_settings_.running_settings.only_half_up_body;
        int num_leg_ec_motors = motor_info.num_joints - motor_info.num_arm_joints - motor_info.num_head_joints - motor_info.num_waist_joints;
        int num_arm_ec_motors = 2;
        int num_waist_ec_motors = motor_info.num_waist_joints;
        if(flag_half_up_body && (num_leg_ec_motors + num_arm_ec_motors + num_waist_ec_motors) == ecMasterEncoderRange.size()) { // Only Half Up Body
            std::cout  << "-----------------------------------------------------\n";
            std::cout  << "!!!!!!! Only Half Up Body !!!!!!!\n";
            std::cout  << "-----------------------------------------------------\n";
            std::vector<uint16_t> ignore_motor_ids{};
            std::cout << "Half Up Body Ignore Ecmaster Id:";
            for(int id = 1; id <= num_leg_ec_motors; ++id) {
                motor_info.motors_disable[id-1] = true;
                // 传入 Joint Index (logical_id, 0-based) 而不是 EC Master ID
                ignore_motor_ids.push_back(static_cast<uint16_t>(id - 1));
                std::cout << (id - 1) << " ";
            }
            std::cout << std::endl;
            // 设置忽略以下 Id EC_MASTER 电机
            actuators.addIgnore(ignore_motor_ids.data(), ignore_motor_ids.size());
        }
        std::cout << "actuatorsInterfaceSetup DONE \n";
        std::cout << "actuatorsInterfaceSetup countECMasters: " << countECMasters << "\n";

        EcActuatorParams params;
        params.num_actuators = countECMasters;
        params.ec_type = ecmaster_type_;
        params.robot_module = robot_module_;
        params.robot_version_int = rb_version_.version_number();
        if (actuators.init(params) != 0)
        {
            std::cout << "actuators init failed !\n";
            return -2;
        }

        // 启动阶段写入YD驱动器温度限幅值（对象索引0x3F0D）
        // 配置来自 kuavo.json 的 "temperature_limit" 字段
        {
            const auto &temp_limits = kuavo_settings_.running_settings.temperature_limit;
            if (!temp_limits.empty())
            {
                // 仅当EC驱动器类型为YD时才写入温度限幅
                bool has_yd_driver = false;
                for (size_t i = 0; i < motor_info.driver.size(); ++i)
                {
                    if (motor_info.driver[i] == EC_MASTER && motor_info.motors_exist[i] &&
                        driver_type[i] == EcMasterType::YD)
                    {
                        has_yd_driver = true;
                        break;
                    }
                }

                if (has_yd_driver)
                {
                    // 使用 ec_index_map_ 获取实际的EC从站ID
                    // 配置值按 ec_count（EC从站计数，与 setDefaultKpKd 中 joint_kp/joint_kd 索引方式一致）
                    std::vector<uint16_t> ec_exist_ids;
                    std::vector<int32_t> values;
                    int ec_count = 0;
                    for (size_t i = 0; i < motor_info.driver.size(); ++i)
                    {
                        if (motor_info.driver[i] == EC_MASTER && motor_info.motors_exist[i] &&
                            driver_type[i] == EcMasterType::YD)
                        {
                            // ec_index_map_[i] 是EC从站物理ID（1-based）
                            ec_exist_ids.push_back(static_cast<uint16_t>(ec_index_map_[i]));
                            // 配置值按 ec_count（EC从站计数）索引，与 setDefaultKpKd 中 joint_kp/joint_kd 一致
                            if (ec_count < static_cast<int>(temp_limits.size()))
                            {
                                values.push_back(temp_limits[ec_count]);
                            }
                            else
                            {
                                values.push_back(temp_limits.back());
                            }
                            ec_count++;
                        }
                    }

                    if (!ec_exist_ids.empty())
                    {
                        std::cout << "[HWPlantInit] Writing YD temperature limits to "
                                  << ec_exist_ids.size() << " YD motors..." << std::endl;
                        int ret = actuators.writeTemperatureLimit(ec_exist_ids, EcMasterType::YD, values);
                        if (ret != 0)
                        {
                            std::cerr << "[HWPlantInit] Write temperature limit failed, ret=" << ret << std::endl;
                        }
                        else
                        {
                            std::cout << "[HWPlantInit] Write temperature limit success" << std::endl;
                        }
                    }
                    else
                    {
                        std::cout << "[HWPlantInit] No YD motors found, skip temperature limit writing" << std::endl;
                    }
                }
                else
                {
                    std::cout << "[HWPlantInit] No YD driver detected, skip temperature limit writing" << std::endl;
                }
            }
            else
            {
                std::cout << "[HWPlantInit] No temperature_limit configured, skip writing" << std::endl;
            }
        }

#if 0
    exit(0);
    actuators.deInit();
#endif

        // ===== EC 驱动器固件与参数校验 =====
        // 在 EC master 起来、电机使能前，复用 live master 的 SDO upload 读取
        // youda 驱动器固件版本与全量参数，与标准定义 JSON 逐字段比对。
        // 严格模式(默认)下任何 mismatch 即中止启动。
        if (hardware_param_.check_firmware_param_)
        {
            std::cout << "[HWPlantInit] === EC 固件/参数校验开始 ===" << std::endl;
            bool check_ok = highlydynamic::EcFirmwareParamCheck::run(
                rb_version_.version_number(),
                ecmaster_type_,
                hardware_param_.allow_mismatch_);
            if (!check_ok)
            {
                std::cerr << "[HWPlantInit] EC 固件/参数校验失败，中止启动" << std::endl;
                exit(1);
            }
            std::cout << "[HWPlantInit] === EC 固件/参数校验结束 ===" << std::endl;
        }
        else
        {
            std::cout << "[HWPlantInit] check_firmware_param=false, 跳过 EC 固件/参数校验" << std::endl;
        }

        double joint_Q[] = {0.2, 1, 1};
        double joint_R[] = {0.2, 2, 4};
        for (uint32_t i = 0; i < num_joint; i++)
        {
            joint_kf.push_back(std::make_unique<Kalman>(3, 3));
            joint_kf[i]->setQR(joint_Q, joint_R);
        }
        sensorsInitHW();

        // 设置电机状态管理器的硬件配置
        setMotorStatusHardwareSettings();

        // 初始化电机状态
        if (motor_status_manager_) 
        {
            GetMotorData(joint_ids, joint_data);
            motor_status_manager_->initMotorStatus(joint_data, joint_ids);
        }

        return 0;
    }
    // bool HardwarePlant::deintialCallback(std_srvs::Trigger::Request &req,
    //                                     std_srvs::Trigger::Response &res)
    // {
    //     std::cout << "HWPlantDeInit Service called!\n";
    //     HWPlantDeInit();
    //     res.success = true;                          // 设置响应字段的值
    //     res.message = "Service called successfully"; // 可选的响应消息
    //     return true;
    // }
    // void HardwarePlant::stopCallback(const std_msgs::Bool::ConstPtr &msg)
    // {
    //     if (msg->data)
    //     {
    //         std::cout << "/stop_robot received!\n";
    //         HWPlantDeInit();
    //     }
    // }
    // bool HardwarePlant::setMotorEncoderOffsetCallback(kuavo_msgs::setMotorEncoderRoundServiceRequest &req, kuavo_msgs::setMotorEncoderRoundServiceResponse &res)
    // {
    //     if (hardware_ready_)
    //     {
    //         res.success = false;
    //         res.message = "hardware is running, setMotorEncoderOffset is prohibited, please stop first!";
    //         return false;
    //     }

    //     bool ret = this->calibrateMotor(req.motor_id, req.direction, req.save_offset);
    //     res.success = ret;
    //     res.message = "";
    //     return true;
    // }
    bool HardwarePlant::setCurrentPositionAsOffset()
    {
        if (hardware_ready_)
        {
            std::cout << "[HardwarePlant]hardware is running, setMotorEncoderOffset is prohibited, please stop first!";
            return false;
        }
        
        std::vector<double> offset;
        getMotorPositionOffset(offset);
        
        GetMotorData(joint_ids, joint_data);

        std::vector<double> ec_positons;
        for (size_t i = 0; i < joint_data.size(); ++i)
        {
            if (motor_info.driver[i] == EC_MASTER)
            {
                ec_positons.push_back(joint_data[i].position);
            }
        }

        std::vector<double> current_pos_remove_offset(num_joint, 0);
        for (size_t i = 0; i < ec_positons.size(); ++i)
        {
            current_pos_remove_offset[i] = ec_positons[i] + offset[i];
            std::cout << "motor_pos_remove_offset[" << i << "]: " << current_pos_remove_offset[i] << std::endl;
            offset[i] = current_pos_remove_offset[i];
        }
        std::string tips_msg = "\033[32m***************正在设置当前位置为编码器零点，请确认当前位置是零点位置***************\n \
        \n输入'c'保存当前位置为零点; \
        \n输入'q'退出; \
        \n输入'p'打印当前位置; \
        \033[0m";
        std::cout << tips_msg << std::endl;
        while (1)
        {
            if (kbhit())
            {
                initial_input_cmd_ = '\0';
                initial_input_cmd_ = getchar();
                if (initial_input_cmd_ == 'q')
                {
                    std::cout << "退出校准\n";
                    return false;
                }
                else if (initial_input_cmd_ == 'c')
                {
                    std::cout << "save offset to file\n";
                    setMotorPositionOffset(offset);
                    saveOffset();
                    return true;
                }
                else if (initial_input_cmd_ == 'p')
                {
                    std::cout << "current position: \n";
                    for (size_t i = 0; i < ec_positons.size(); ++i)
                    {
                        std::cout << current_pos_remove_offset[i] << "\n";
                    }
                    std::cout << std::endl;
                }
                else if (initial_input_cmd_ == '\n' || initial_input_cmd_ == 'h')
                {
                    std::cout << tips_msg << std::endl;
                }
            }
            usleep(100000);
        }

        exit(0);
        // setMotorPositionOffset(offset);
        return false;
    }

    // /* gesture execute service. */
    // bool HardwarePlant::gestureExecuteCallback(kuavo_msgs::gestureExecuteRequest &req,
    //                       kuavo_msgs::gestureExecuteResponse &res)
    // {
    //     if (!hand_controller) {
    //         res.success = false;
    //         res.message = "hand controller not initialized.";
    //         return false;
    //     }

    //     std::string err_msg;
    //     GestureExecuteInfoVec gesture_tasks;
    //     for(const auto & gs : req.gestures) {
    //         gesture_tasks.push_back(GestureExecuteInfo{static_cast<HandSide>(gs.hand_side), gs.gesture_name});
    //     }
        
    //     if(gesture_tasks.size() == 0) {
    //         res.success = false;
    //         res.message = "No gesture to execute.";
    //         return false;
    //     }

    //     /* execute gestures */
    //     bool exec_ret = hand_controller-> execute_gestures(gesture_tasks, err_msg);
    //     if(exec_ret) {
    //         res.message = "success.";
    //         res.success = true;
    //     }    
    //     else {
    //         res.message = err_msg;
    //         res.success = false;
    //     }
    //     return true;
    // }

    // /* gesture list service. */
    // bool HardwarePlant::gestureListCallback(kuavo_msgs::gestureListRequest &req,
    //                       kuavo_msgs::gestureListResponse &res)
    // {
    //     if (!hand_controller) {
    //         res.gesture_infos.clear();
    //         res.gesture_count  = 0;
    //         res.success = false;
    //         res.message = "hand controller not initialized.";
    //         return false;
    //     }

    //     res.gesture_infos = hand_controller->list_gestures();        
    //     res.success = true;
    //     res.message = "success";
    //     res.gesture_count = res.gesture_infos.size();
    //     return true;
    // }

    // /* gesture execute state service. */
    // bool HardwarePlant::gestureExecuteStateCallback(kuavo_msgs::gestureExecuteStateRequest &req,
    //                       kuavo_msgs::gestureExecuteStateResponse &res)
    // {
    //     if (!hand_controller) {
    //         res.is_executing = false;
    //         res.message = "hand controller not initialized.";
    //         return false;
    //     }

    //     res.is_executing = hand_controller->is_gesture_executing();        
    //     res.message = "success";
    //     return true;
    // }

    bool HardwarePlant::calibrateMotor(int motor_id, int direction, bool save_offset)
    {
        if (hardware_ready_)
        {
            std::cout << "[HardwarePlant]hardware is running, setMotorEncoderOffset is prohibited, please stop first!";
            return false;
        }
        std::vector<double> offset;
        getMotorPositionOffset(offset);
        if (motor_id >= motor_info.encoder_range.size())
        {
            std::cout << "[HardwarePlant]motor_id " + std::to_string(motor_id) + " out of range "+ std::to_string(motor_info.encoder_range.size()) + " !";
            return false;
        }
        if (save_offset)
        {
            std::cout << "[HardwarePlant]save offset to file\n";
            saveOffset();
            return true;
        }
        
        double degree_per_round = 360.0 / (motor_info.encoder_range[motor_id] / BIT_17);
        std::cout << "motor_id: " << motor_id << " direction: " << direction << " degree_per_round: " << degree_per_round << std::endl;
        std::cout << "offset[" + std::to_string(motor_id) + "]: " << offset[motor_id];
        offset[motor_id] += (direction > 0) ? degree_per_round : -degree_per_round;
        std::cout << " -> " << offset[motor_id] << std::endl;
        setMotorPositionOffset(offset);
        return true;
    }

    // bool HardwarePlant::setHwIntialStateCallback(kuavo_msgs::setHwIntialStateRequest &req, kuavo_msgs::setHwIntialStateResponse &res)
    // {
    //     std::cout << "setHwIntialStateCallback called\n";
    //     if (hardware_ready_)
    //     {
    //         res.success = false;
    //         res.message = "hardware is running, setHwIntialState is prohibited, please stop first!";
    //         return false;
    //     }
    //     // TODO: only control legs
    //     if (req.q_intial.size() == 12)
    //     {
    //         std::vector<double> goal_pos(num_joint, 0);
    //         for (int i; i < 12; i++)
    //         {
    //             goal_pos[i] = req.q_intial[i];
    //         }
    //         jointMoveTo(goal_pos, 45.0, dt_);
    //         res.success = true;
    //         res.message = "success";
    //         hardware_ready_ = true;
    //         joint_sub_ = nh_.subscribe("joint_cmd", 10, &HardwarePlant::jointCmdCallback, this);
    //         return true;
    //     }
    //     if (req.q_intial.size() != num_joint)
    //     {
    //         res.success = false;
    //         res.message = "q_intial size " + std::to_string(req.q_intial.size()) + " not match " + std::to_string(num_joint) + "!";
    //         return false;
    //     }
    //     jointMoveTo(req.q_intial, 45, dt_);
    //     res.success = true;
    //     res.message = "success";
    //     this->readyToRecvCmd();
    //     return true;
    // }
    // void HardwarePlant::readyToRecvCmd()
    // {
    //     hardware_ready_ = true;
    //     joint_sub_ = nh_.subscribe("joint_cmd", 10, &HardwarePlant::jointCmdCallback, this);
    //     ros::param::set("/hardware/is_ready", 1);
    //     hardware_status_ = 1; 

    // }
    // bool HardwarePlant::jointMoveToServiceCallback(kuavo_msgs::jointMoveToRequest &req, kuavo_msgs::jointMoveToResponse &res)
    // {
    //     std::cout << "jointMoveToServiceCallback called\n";
    //     if (hardware_ready_)
    //     {
    //         res.success = false;
    //         res.message = "hardware is running, jointMoveTo is prohibited, please stop first!";
    //         return false;
    //     }
    //     // TODO: only control legs
    //     if (req.goal_position.size() == 12)
    //     {
    //         std::vector<double> goal_pos(num_joint, 0);
    //         for (int i; i < 12; i++)
    //         {
    //             goal_pos[i] = req.goal_position[i];
    //         }
    //         jointMoveTo(goal_pos, req.speed, req.dt);
    //         res.success = true;
    //         res.message = "success";
    //         return true;
    //     }
    //     if (req.goal_position.size() != num_joint)
    //     {
    //         res.success = false;
    //         res.message = "goal_position size " + std::to_string(req.goal_position.size()) + " not match " + std::to_string(num_joint) + "!";
    //         return false;
    //     }

    //     jointMoveTo(req.goal_position, req.speed, req.dt);
    //     res.success = true;
    //     res.message = "success";
    //     return true;
    // }

    // void HardwarePlant::jointCmdCallback(const kuavo_msgs::jointCmd::ConstPtr &msg)
    // {
    //     // std::cout << "jointCmdCallback called\n" << std::endl;
    //     size_t leg_joints = num_joint; // leg + arm + head
    //     auto is_match_size = [&](size_t size)
    //     {
    //         if (msg->joint_q.size() != size || msg->joint_v.size() != size ||
    //             msg->tau.size() != size || msg->tau_ratio.size() != size ||
    //             msg->control_modes.size() != size || msg->tau_max.size() != size)
    //         {
    //             return false;
    //         }
    //         return true;
    //     };
    //     bool match_leg_joints_size = is_match_size(leg_joints);
    //     if (!match_leg_joints_size)
    //     {
    //         std::cerr << "jointCmdCallback Error: joint_q, joint_v, tau, tau_ratio, control_modes size not match!" << std::endl;
    //         std::cerr << "desire size:" << leg_joints << std::endl;
    //         std::cerr << "joint_q size:" << msg->joint_q.size() << std::endl;
    //         std::cerr << "joint_v size:" << msg->joint_v.size() << std::endl;
    //         std::cerr << "tau size:" << msg->tau.size() << std::endl;
    //         std::cerr << "tau_ratio size:" << msg->tau_ratio.size() << std::endl;
    //         std::cerr << "control_modes size:" << msg->control_modes.size() << std::endl;
    //         std::cerr << "tau_max size:" << msg->tau_max.size() << std::endl;
    //         return;
    //     }
    //     Eigen::VectorXd cmd, cmd_out,joint_kp(num_joint),joint_kd(num_joint); // q,v,tau,tau_max,tau_ratio
    //     cmd.resize(num_joint * 5);
    //     cmd_out.resize(num_joint * 5);
    //     cmd_out.setZero();
    //     joint_kp.setZero();
    //     joint_kd.setZero();
        
    //     std::vector<int> control_mode_vec(num_joint, 2);
    //     for (uint32_t i = 0; i < num_joint; i++)
    //     {
    //         if (i < leg_joints)
    //         {
    //             cmd[num_joint * 0 + i] = msg->joint_q[i];
    //             cmd[num_joint * 1 + i] = msg->joint_v[i];
    //             cmd[num_joint * 2 + i] = msg->tau[i];
    //             cmd[num_joint * 3 + i] = msg->tau_max[i];
    //             cmd[num_joint * 4 + i] = msg->tau_ratio[i];
    //             control_mode_vec[i] = msg->control_modes[i];
    //         }
    //         joint_kp[i] = msg->joint_kp[i];
    //         joint_kd[i] = msg->joint_kd[i];
    //         // else // TODO: only leg controll
    //         // {
    //         //     cmd[num_joint * 0 + i] = 0;
    //         //     cmd[num_joint * 1 + i] = 0;
    //         //     cmd[num_joint * 2 + i] = 0;
    //         //     cmd[num_joint * 3 + i] = 50;
    //         //     cmd[num_joint * 4 + i] = 1;
    //         //     control_mode_vec[i] = 2;
    //         // }
    //     }

    //     cmds2Cmdr(cmd, num_joint, cmd_out, num_joint);
    //     logger_ptr_->publishVector("motor_cmd/motor_pos", cmd_out.segment(num_joint * 0, num_joint));
    //     logger_ptr_->publishVector("motor_cmd/motor_vel", cmd_out.segment(num_joint * 1, num_joint));
    //     logger_ptr_->publishVector("motor_cmd/motor_cur", cmd_out.segment(num_joint * 2, num_joint));
    //     if (cmd_out.hasNaN())
    //     {
    //         std::cout << "cmd_out.hasNaN \ncmd_out:" << cmd_out.segment(num_joint * 2, num_joint).segment(10, 2).transpose() << std::endl;
    //         std::cout << "cmd" << cmd.transpose() << std::endl;
    //         std::cout << "cmd_out:" << cmd_out.transpose() << std::endl;
    //         cmd_out.setZero();
    //     }

    //     writeCommand(cmd_out, num_joint, control_mode_vec, joint_kp, joint_kd);
    // }

    // void HardwarePlant::handCmdCallback(const kuavo_msgs::robotHandPosition::ConstPtr &msg)
    // {
    //     if (msg->left_hand_position.size() != 6 || msg->right_hand_position.size() != 6) 
    //     {
    //     ROS_WARN("Received desired positions vector of incorrect size");
    //     return;
    //     }
    //     // 目前通过直接调用hand_sdk
    //     int8_t left_end_effc_position[6] = {0};
    //     int8_t right_end_effc_position[6] = {0};

    //     // left_hand_position 的值
    //     for (size_t i = 0; i < msg->left_hand_position.size() && i < 6; ++i) {
    //         left_end_effc_position[i] = static_cast<int8_t>(msg->left_hand_position[i]);
    //     }

    //     // right_hand_position 的值
    //     for (size_t i = 0; i < msg->right_hand_position.size() && i < 6; ++i) {
    //         right_end_effc_position[i] = static_cast<int8_t>(msg->right_hand_position[i]);
    //     }

    //     // 接口调用
    //     if(hand_controller != nullptr) {
    //         hand_controller->send_position(HandSide::LEFT, left_end_effc_position);
    //         hand_controller->send_position(HandSide::RIGHT, right_end_effc_position);
    //     }
    // }
    

    void HardwarePlant::genArmSwingTrajectory(Eigen::VectorXd &cmd)
    {
        double q_hip_pitch = cmd[2] - cmd[8];
        // logger_ptr_->publishValue("cmd/arm/q_hip_pitch", q_hip_pitch);
        double alpha = 0.05;
        q_hip_pitch = alpha * q_hip_pitch + (1 - alpha) * q_hip_pitch_prev_;
        q_hip_pitch_prev_ = q_hip_pitch;
        Eigen::Vector2d v_hip_pitch(cmd[num_joint + 2], cmd[num_joint + 8]);
        int l_arm_id = 12;
        int r_arm_id = 12 + num_arm_joints / 2;
        cmd[l_arm_id] = -q_hip_pitch * swing_arm_gain_;
        cmd[r_arm_id] = q_hip_pitch * swing_arm_gain_;
        // logger_ptr_->publishVector("cmd/arm/q", Eigen::Vector2d(cmd[l_arm_id], cmd[r_arm_id]));
    }
    SensorData_t HardwarePlant::sensorsInitHW()
    {
        Eigen::Vector3d acc, gyro;
        Eigen::Quaterniond quat;
        std::cout << "sensor init start!!\r\n";

        //  bool flag_only_half_up_body = hardware_param_.only_half_up_body;
        bool flag_only_half_up_body = kuavo_settings_.running_settings.only_half_up_body;
        if(flag_only_half_up_body || isLunbiModule(robot_module_)) { // Only Use Half Up Body.
            acc << 0,0,0;
            gyro << 0,0,0;
            quat = Eigen::Quaterniond(1,0,0,0);
        }
        else 
        {
            int ret = false;
            if (redundant_imu_)// 初始化两个imu
            {
                ret = HIPNUC_IMU::imu_init();
                ret = xsens_IMU::imu_init();
            }
            else if (imu_type_ == ImuType::IMU_TYPE_HIPNUC)
            {
                ret = HIPNUC_IMU::imu_init(); // TODO: better imu init
            }
            else
            {
                ret = xsens_IMU::imu_init();
            }
            if (ret != 0)
            {
                std::cerr << "imu init failed !\n";
                exit(1);
            }
            std::cout << "imu init DONE!!\r\n";
            bool readflag = false;
            if (imu_type_ == ImuType::IMU_TYPE_HIPNUC)
            {
                readflag = HIPNUC_IMU::getImuDataFrame(acc, gyro, quat);
            }
            else
            {
                readflag = xsens_IMU::getImuDataFrame(acc, gyro, quat);
            }
            std::cout << "acc: " << readflag << " " << acc.transpose() << std::endl;
            if (readflag)
            {
                std::cout << "imu init success!!\r\n";
            }
            else
            {
                std::cerr << "get imu data failed !\n";
                exit(1);
            }
        }
        SensorData_t sensor_data_motor;
        for (uint8_t i = 0; i < 5; i++)
        {
            readSensor(sensor_data_motor);
            usleep(dt_ * 1e5);
        }

        return sensor_data_motor;
    }

    void HardwarePlant::setState(SensorData_t &sensor_data_motor,SensorData_t &sensor_data_joint)
    {
        std::lock_guard<std::mutex> lock(motor_joint_data_mtx_);
        sensor_data_motor_last = sensor_data_motor;
        sensor_data_joint_last = sensor_data_joint;
    }
    void HardwarePlant::getState(SensorData_t &sensor_data_motor,SensorData_t &sensor_data_joint)
    {
        std::lock_guard<std::mutex> lock(motor_joint_data_mtx_);
        sensor_data_motor = sensor_data_motor_last;
        sensor_data_joint = sensor_data_joint_last;
    }


    void HardwarePlant::HWPlantDeInit()
    {
        std::cerr << "HardwarePlant deinit start!!\n";
        th_running_ = false;
        hardware_ready_ = false;
        is_deinitialized_ = true;  // 设置析构标志位
        
        // std::vector<double> goal_pos(num_joint, 0);
        actuators.deInit();
        // Close dexhands while CAN is still open. Ruiwo close() tears down canbus0/1.
        if(dexhand_actuator != nullptr) {
            std::cout << "dexhand_actuator deinit\n";
            dexhand_actuator->close();
            dexhand_actuator = nullptr;
        }
        if(revo2_actuator != nullptr) {
            std::cout << "revo2_actuator deinit\n";
            revo2_actuator->close();
            revo2_actuator = nullptr;
        }
        if(leju_claw_actuator != nullptr) { 
            std::cout << "leju_claw_actuator deinit\n";
            leju_claw_actuator->close();
            leju_claw_actuator = nullptr;
        }
        if (ruiwo_actuator != nullptr)
        {
            std::cout << "ruiwo_actuator deinit\n";
            ruiwo_actuator->close();
            ruiwo_actuator = nullptr;
        }
        std::cout << "imu deinit" << std::endl;
        if (redundant_imu_)
        {
            HIPNUC_IMU::imu_stop();
            xsens_IMU::imu_stop();
        }
        else if (imu_type_ == ImuType::IMU_TYPE_HIPNUC)
            HIPNUC_IMU::imu_stop();
        else
            xsens_IMU::imu_stop();
        std::cout << "HardwarePlant deinit DONE!!\n";
    }

    void HardwarePlant::setDefaultJointPos(std::vector<double> default_joint_pos)
    {
        hardware_param_.default_joint_pos = default_joint_pos;
        defalutJointPos_.resize(hardware_param_.default_joint_pos.size());
        for (int i = 0; i < hardware_param_.default_joint_pos.size(); i++) {
            defalutJointPos_[i] = hardware_param_.default_joint_pos[i];
        }
        vector_t stance_motor_position = ankleSolver.joint_to_motor_position(defalutJointPos_.head(na_foot_));
        std::cout << ">>>>>>>>>>>>>>>>>> stance_motor_position: " << stance_motor_position.transpose()*TO_DEGREE << std::endl;

        std::array<double, 12> leg_pos_array;
        for (int i = 0; i < 12; i++) {
            leg_pos_array[i] = stance_motor_position[i]*TO_DEGREE;
        }
        if (stance_leg_joint_pos_ == nullptr)
        {
            stance_leg_joint_pos_ = std::make_unique<std::array<double, 12>>(leg_pos_array);
        }
        else
        {
            for (int i = 0; i < leg_pos_array.size(); i++) {
                stance_leg_joint_pos_->at(i) = leg_pos_array[i];
            }
        }
    }

    HardwarePlant::HardwarePlant(double dt, HardwareParam hardware_param, const std::string & hardware_abs_path, uint8_t control_mode, uint16_t num_actuated, uint16_t nq_f, uint16_t nv_f): 
                                        dt_(dt), 
                                        control_mode_{control_mode},
                                        num_actuated_{num_actuated},
                                        nq_f_{nq_f},
                                        nv_f_{nv_f},
                                        rb_version_(4,2)
    {
        hardware_param_ = hardware_param;
        hard_dt = dt_;
        // Get robot version from hardware params
        rb_version_ = hardware_param_.robot_version;

        setHardwarePlantPath(hardware_abs_path);
        
        kuavo_common_ptr_ = KuavoCommon::getInstancePtr(rb_version_, hardware_param_.kuavo_assets_path);
        kuavo_settings_ = kuavo_common_ptr_->getKuavoSettings();
        wheel_arm_abs_vel::applyEndEffectorContract(kuavo_settings_, rb_version_.version_number());
        JSONConfigReader *robot_config = kuavo_common_ptr_->getRobotConfig();
        calibration_stall_torque_ratio_ =
            robot_config->getValueOrDefault<double>("joint_limit_torque_ratio", 0.3);
        swing_arm_ = robot_config->getValue<bool>("swing_arm");
        swing_arm_gain_ = robot_config->getValue<double>("swing_arm_gain");
        const std::string ankleSolverTypeToken = robot_config->getValue<std::string>("ankle_solver_type");
        ankleSolver.getconfig(ankleSolverTypeToken);
        motor_info = kuavo_settings_.hardware_settings;
        isParallelArm = kuavo_settings_.model_settings.is_parallel_arm;
        imu_invert = motor_info.imu_invert;
        num_joint = kuavo_settings_.hardware_settings.num_joints;
        robot_module_ = kuavo_settings_.hardware_settings.robot_module;
        num_arm_joints = kuavo_settings_.hardware_settings.num_arm_joints;
        num_waist_joints = kuavo_settings_.hardware_settings.num_waist_joints;
        num_head_joints = kuavo_settings_.hardware_settings.num_head_joints;
        ankle_motor_offset = kuavo_settings_.model_settings.ankle_motor_offset_degree * TO_RADIAN;
        if (isParallelArm)
        {
            arm_ankle_motor_offset_ = kuavo_settings_.model_settings.arm_ankle_motor_offset_degree * TO_RADIAN;
        }

        std::cout << " get num_joint: " << num_joint << std::endl;
        std::cout << " get num_arm_joints: " << num_arm_joints << std::endl;
        std::cout << " get num_waist_joints: " << num_waist_joints << std::endl;
        std::cout << " get num_head_joints: " << num_head_joints << std::endl;
        na_foot_ = num_joint - num_arm_joints - num_waist_joints - num_head_joints;
        std::cout << "get na_foot num: " << na_foot_ << std::endl;

        // 计算头部电机在 ruiwo 数组中的起始索引
        head_ruiwo_idx_start_ = 0;
        int head_joint_start = na_foot_ + num_waist_joints + num_arm_joints;
        for (int j = 0; j < head_joint_start; ++j)
        {
            if (motor_info.driver[j] == MotorDriveType::RUIWO)
                head_ruiwo_idx_start_++;
        }
        std::cout << " head_ruiwo_idx_start: " << head_ruiwo_idx_start_ << std::endl;
        nq_ = nq_f_ + na_foot_;
        nv_ = nv_f_ + na_foot_;
        joint_data.resize(num_joint);
        joint_data_old.resize(num_joint);
        joint_cmd.resize(num_joint);
        joint_cmd_old.resize(num_joint);
        joint_ids = motor_info.joint_ids;
        c2t_coeff = motor_info.c2t_coeff_default;
        motor_cul = motor_info.c2t_cul;
        motor_coeff = motor_info.c2t_coeff;
        max_joint_position_limits = motor_info.max_joint_position_limits;
        min_joint_position_limits = motor_info.min_joint_position_limits;
        joint_velocity_limits_ = motor_info.joint_velocity_limits;
        joint_peak_velocity_limits_ = motor_info.joint_peak_velocity_limits;
        joint_lock_rotor_limits_ = motor_info.joint_lock_rotor_limits;
        joint_peak_torque_limits_ = motor_info.joint_peak_limits;
        peakTimeWin_ = motor_info.peak_timeWin;
        lockRotorTimeWin_ = motor_info.lock_rotor_timeWin;
        speedTimeWin_ = motor_info.speed_timeWin;
        std::cout << "current limit[";
        for (int i = 0; i < num_joint; i++)
        {
            std::cout << motor_info.max_current[i] << " ";
        }
        std::cout << "]" <<std::endl;
        std::cout << "c2t [";
        for (const auto& coeff_row : motor_info.c2t_coeff)
        {
            std::cout << "[";
            for (size_t j = 0; j < coeff_row.size(); ++j)
            {
                std::cout << coeff_row[j];
                if (j + 1 < coeff_row.size())
                {
                    std::cout << " ";
                }
            }
            std::cout << "] ";
        }
        std::cout << "]" << std::endl;
        std::cout << "Load protect time window, peak: " << peakTimeWin_ << "| lock rotor: " << lockRotorTimeWin_ << "| speed: " << speedTimeWin_ << std::endl;
        int ec_count = 0;

        // 禁用电机
        if(1 == hardware_param_.teach_pendant_){

            // ROS_INFO("current is teach_pendant mode");
            std::cout << "current is teach_pendant mode" << std::endl;
            this->kuavo_settings_.running_settings.only_half_up_body = true;
            std::vector<std::string> MOTORS_TYPE = robot_config->getValue<std::vector<std::string>>("MOTORS_TYPE");
            
            for (uint8_t i = 0; i < motor_info.num_joints; i++)
            {
                std::string motor_type_name = MOTORS_TYPE[i];

                if(motor_type_name.size() >= 5){

                    if(motor_type_name.substr(0, 5) != "ruiwo"){

                        if(motor_type_name.substr(motor_type_name.size() - 5, 5) != "_none"){
                            motor_info.motors_exist[i] = true;
                            motor_info.motors_disable[i] = true;
                            // ROS_INFO_STREAM("Disable motor " << motor_type_name);
                        }
                    }
                }
                else{

                    motor_info.motors_exist[i] = true;
                    motor_info.motors_disable[i] = true;
                    // ROS_INFO_STREAM("Disable motor " << motor_type_name);
                }
            }
        }

        for (int i = 0; i < motor_info.driver.size(); ++i)
        {
            if (motor_info.driver[i] == EC_MASTER)
            {
                if(motor_info.motors_exist[i]) {
                    ec_count++;
                     ec_index_map_[i] = ec_count; // EC master not exist
                }
                else {
                    ec_index_map_[i] = kEcMasterNilId; // EC master not exist
                }
            }
        }

        /* only for halfup_body mode */
        if (kuavo_settings_.running_settings.only_half_up_body)
        {
            if (hardware_param_.default_joint_pos.size() == 0)
            {
                stance_leg_joint_pos_ = std::make_unique<std::array<double, 12>>(std::array<double, 12>{});
                std::cout << "no defalut joint pos provide !!!" << std::endl;
            }
            else{
                defalutJointPos_.resize(hardware_param_.default_joint_pos.size());
                for (int i = 0; i < hardware_param_.default_joint_pos.size(); i++) {
                    defalutJointPos_[i] = hardware_param_.default_joint_pos[i];
                }
                vector_t stance_motor_position = ankleSolver.joint_to_motor_position(defalutJointPos_.head(na_foot_));
                std::cout << ">>>>>>>>>>>>>>>>>> stance_motor_position: " << stance_motor_position.transpose()*TO_DEGREE << std::endl;

                std::array<double, 12> leg_pos_array;
                for (int i = 0; i < 12; i++) {
                    leg_pos_array[i] = stance_motor_position[i]*TO_DEGREE;
                }
                stance_leg_joint_pos_ = std::make_unique<std::array<double, 12>>(leg_pos_array);
            }
        }
        /* end */

        if (std::count(motor_info.end_effector_type.begin(), motor_info.end_effector_type.end(), EndEffectorType::jodell) == 2)
            has_end_effectors = true;
        ankle_qv_old.setZero();
        ecmaster_type_ = motor_info.getEcmasterType(rb_version_);
        std::string imu_type_str = motor_info.getIMUType(rb_version_);
        std::map<std::string, ImuType> imu_type_map = {{"xsens", ImuType::IMU_TYPE_XSENS}, {"hipnuc", ImuType::IMU_TYPE_HIPNUC}, {"none", ImuType::IMU_TYPE_NONE}};
        if (imu_type_map.find(imu_type_str) == imu_type_map.end())
        {
            std::cerr << "IMU type not supported: " << imu_type_str << std::endl;
            exit(1);
        }
        imu_type_ = imu_type_map[imu_type_str];
        if (redundant_imu_)
        {
            imu_type_ = ImuType::IMU_TYPE_XSENS;
        }
        this->setEcmasterDriverType(ecmaster_type_);

        const auto &eef_types = motor_info.end_effector_type;

        std::map<EndEffectorType, std::string> eef_name_maps {
            {EndEffectorType::jodell, "jodell"},
            {EndEffectorType::qiangnao, "qiangnao"},
            {EndEffectorType::qiangnao_touch, "qiangnao_touch"},
            {EndEffectorType::lejuclaw, "lejuclaw"},
            {EndEffectorType::revo2, "revo2"},
            {EndEffectorType::heiman, "heiman"},
            {EndEffectorType::linker_hand, "linker_hand"},
            {EndEffectorType::qibeng, "qibeng"},
            {EndEffectorType::none, "none"}
        };

        end_effector_type_ = eef_name_maps[eef_types[0]];

        // 初始化电机状态管理器
        motor_status_manager_ = std::make_unique<MotorStatusManager>(num_joint, 10, 1000);
        th_running_ = true;
        
    }

    //  bool HardwarePlant::adjustZeroPointCallback(kuavo_msgs::adjustZeroPointRequest &req,
    //                                           kuavo_msgs::adjustZeroPointResponse &res)
    // {
    //     if (hardware_ready_)
    //     {         
    //         ROS_INFO("[HardwarePlant] adjustZeroPoint require robot at ready pose before push 'o', now robot pose is after push 'o', please restart robot!");
    //         res.success = false;
    //         res.message = "adjustZeroPoint require robot at ready pose before push 'o', now robot pose is after push 'o', please restart robot!";
    //         return false;
    //     }

    //     int motor_index = req.motor_index;
    //     double adjust_pos = req.offset;  // degree

    //     if (motor_index >= motor_info.driver.size() || motor_index < 0) {
    //         ROS_INFO("[HardwarePlant] require motor_index < %d, while recv motor_index is %d !, value mismatch!", motor_info.driver.size(), motor_index);
    //         res.success = false;
    //         res.message = "require motor_index < " + std::to_string(motor_info.driver.size()) + ", while recv motor_index is " + std::to_string(motor_index) + " !, value mismatch!";
    //         return false;
    //     }

    //     int ruiwo_motor_index = 0;
    //     if (motor_info.driver[motor_index] == EC_MASTER){

    //         std::vector<double> offset;
    //         getMotorPositionOffset(offset);
    //         offset[motor_index] = adjust_pos;
    //         setMotorPositionOffset(offset);
    //     }
    //     else if (motor_info.driver[motor_index] == RUIWO){
            
    //         if(nullptr != ruiwo_actuator){

    //             // 将电机索引转换为RUIWO电机索引
    //             for(int i = 0; i < motor_index; i++) {
    //                 if(motor_info.driver[i] == RUIWO){
    //                     ruiwo_motor_index++;
    //                 }
    //             }

    //             adjust_pos = adjust_pos * TO_RADIAN;
    //             ruiwo_actuator->adjustZeroPosition(ruiwo_motor_index, adjust_pos);
    //         }
    //     }
    //     else {
    //         ROS_INFO("[HardwarePlant] unsupport motor driver type: %d ", motor_info.driver[motor_index]);
    //         res.success = false;
    //         res.message = "unsupport motor driver type: " + std::to_string(motor_info.driver[motor_index]);
    //         return false;   
    //     }


    //     std::vector<double> moving_pos(num_joint, 0);
    //     double cali_tau_limit = 3;

    //     std::fill(moving_pos.begin(), moving_pos.end(), 0.0);
    //     if(is_cali_set_zero_){
    //         int num_leg = num_joint - num_arm_joints - num_waist_joints - num_head_joints;
    //         moving_pos[num_waist_joints + num_leg] = 90;
    //         moving_pos[num_waist_joints + num_leg + num_arm_joints / 2] = -90;
    //     }
    //     this->jointMoveTo(moving_pos, 60.0, dt_, cali_tau_limit);

    //     res.success = true;
    //     res.message = "adjust zero point success";

    //     return true;
    // }

    size_t HardwarePlant::get_num_actuated() const
    {
        return num_joint;
    }

    void HardwarePlant::cmds2Cmdr(const Eigen::VectorXd &cmd_s, uint32_t na_src, Eigen::VectorXd &cmd_r, uint32_t na_r)
    {
        cmd_r = cmd_s;

        if(!isLunbiModule(robot_module_))
        {
            // foot-->ankle_motor
            Eigen::VectorXd motor_position(na_foot_);
            Eigen::VectorXd motor_velocity(na_foot_);
            Eigen::VectorXd motor_current(na_foot_);
            SensorData_t sensor_data_motor;
            SensorData_t sensor_data_joint;
            getState(sensor_data_motor,sensor_data_joint);
            motor_position = ankleSolver.joint_to_motor_position(cmd_s.segment(0, na_foot_));
            motor_velocity = ankleSolver.joint_to_motor_velocity(sensor_data_joint.joint_q.segment(0, na_foot_),sensor_data_motor.joint_q.segment(0, na_foot_),cmd_s.segment(na_src, na_foot_));
            motor_current = ankleSolver.joint_to_motor_current(sensor_data_joint.joint_q.segment(0, na_foot_),sensor_data_motor.joint_q.segment(0, na_foot_),cmd_s.segment(na_src * 2,na_foot_));
            
            cmd_r.segment(0, na_foot_)<< motor_position;
            cmd_r.segment(na_r, na_foot_) << motor_velocity;
            cmd_r.segment(na_r * 2, na_foot_) << motor_current;
        }

        // tau_max
        cmd_r.segment(na_r * 3, na_r) << cmd_s.segment(na_src * 3, na_src);
        // tau ratio
        cmd_r.segment(na_r * 4, na_r) << cmd_s.segment(na_src * 4, na_src);
        if (!isParallelArm)
            return;
    }

    void HardwarePlant::motor2joint(SensorData_t sensor_data_motor, SensorData_t &sensor_data_joint)
    {
        Eigen::VectorXd q_r = sensor_data_motor.joint_q;
        Eigen::VectorXd v_r = sensor_data_motor.joint_v;
        Eigen::VectorXd vd_r = sensor_data_motor.joint_vd;
        Eigen::VectorXd cur_r = sensor_data_motor.joint_current;
        Eigen::VectorXd q_joint(na_foot_);
        Eigen::VectorXd v_joint(na_foot_);
        Eigen::VectorXd tau_joint(na_foot_);
        Eigen::VectorXd joint_c2t(num_joint);
        joint_c2t = GetC2Tcoeff(sensor_data_motor.joint_current);
        sensor_data_joint.acc = sensor_data_motor.acc;
        sensor_data_joint.free_acc = sensor_data_motor.free_acc;
        sensor_data_joint.quat = sensor_data_motor.quat;
        sensor_data_joint.gyro = sensor_data_motor.gyro;
        for (size_t i = 0; i < num_joint; i++)
        {
            cur_r(i) = cur_r(i) * joint_c2t[i];//c2t_coeff[i];
        }
        // num_waist_joints 表示腰部自由度, 电机排布上在下肢电机前面, 临时做法, 腰部不参与数据反馈
        q_joint = ankleSolver.motor_to_joint_position(q_r.head(na_foot_));
        v_joint = ankleSolver.motor_to_joint_velocity(q_joint,q_r.head(na_foot_),v_r.head(na_foot_));
        tau_joint = ankleSolver.motor_to_joint_torque(q_joint,q_r.head(na_foot_),cur_r.head(na_foot_));
        vd_r[4] = (v_joint(4) - ankle_qv_old(0)) / hard_dt;
        vd_r[5] = (v_joint(5) - ankle_qv_old(1)) / hard_dt;
        vd_r[10] = (v_joint(10) - ankle_qv_old(2)) / hard_dt;
        vd_r[11] = (v_joint(11) - ankle_qv_old(3)) / hard_dt;
        ankle_qv_old << v_joint(4), v_joint(5), v_joint(10), v_joint(11);
        // foot
        sensor_data_joint.joint_q.segment(0, na_foot_) << q_joint;
        sensor_data_joint.joint_v.segment(0, na_foot_) << v_joint;
        sensor_data_joint.joint_vd.segment(0, na_foot_) << vd_r.segment(0, na_foot_);
        sensor_data_joint.joint_current.segment(0, na_foot_) << tau_joint;
        // waist
        sensor_data_joint.joint_q.segment(na_foot_, num_waist_joints) << q_r.segment(na_foot_, num_waist_joints);
        sensor_data_joint.joint_v.segment(na_foot_, num_waist_joints) << v_r.segment(na_foot_, num_waist_joints);
        sensor_data_joint.joint_vd.segment(na_foot_, num_waist_joints) << vd_r.segment(na_foot_, num_waist_joints);
        sensor_data_joint.joint_current.segment(na_foot_, num_waist_joints) << cur_r.segment(na_foot_, num_waist_joints);
        // ARM
        sensor_data_joint.joint_q.segment(na_foot_+num_waist_joints, num_arm_joints + num_head_joints) << sensor_data_motor.joint_q.segment(na_foot_+num_waist_joints, num_arm_joints + num_head_joints);
        sensor_data_joint.joint_v.segment(na_foot_+num_waist_joints, num_arm_joints + num_head_joints) << sensor_data_motor.joint_v.segment(na_foot_+num_waist_joints, num_arm_joints + num_head_joints);
        sensor_data_joint.joint_vd.segment(na_foot_+num_waist_joints, num_arm_joints + num_head_joints) << sensor_data_motor.joint_vd.segment(na_foot_+num_waist_joints, num_arm_joints + num_head_joints);
        sensor_data_joint.joint_current.segment(na_foot_+num_waist_joints, num_arm_joints + num_head_joints) << cur_r.segment(na_foot_+num_waist_joints, num_arm_joints + num_head_joints);

        if (!isParallelArm)
            return;
    }

    void HardwarePlant::performJointSymmetryCheck()
    {
        std::vector<std::vector<double>> pos_list = joint_test_poses::test_pos_list();
        // 轮臂是 4 下肢 + 14 臂 + 2 头，不能直接套用双足 28 关节测试姿态。
        // 双足姿态下标 3 是左膝（约 109°），下标 7 是右腿，在轮臂上分别落到腰（关节4）和左手 j4。
        if (isLunbiModule(robot_module_))
        {
            const int leg_n = static_cast<int>(num_joint) - static_cast<int>(num_arm_joints)
                              - static_cast<int>(num_head_joints) - static_cast<int>(num_waist_joints);
            const int arm_start = leg_n + static_cast<int>(num_waist_joints);
            const int head_start = arm_start + static_cast<int>(num_arm_joints);
            constexpr int kBipedArmStart = 12;
            constexpr int kBipedHeadStart = 26;
            const auto biped = pos_list;

            pos_list.clear();
            std::vector<double> lower(num_joint, 0.0);
            const auto &safe = kuavo_settings_.predefined_arm_pose.leg_calibration_safe_pose;
            for (int i = 0; i < leg_n && i < static_cast<int>(num_joint); ++i)
            {
                if (i < static_cast<int>(safe.size()))
                    lower[static_cast<size_t>(i)] = safe[i];
            }
            pos_list.push_back(lower);

            std::vector<double> arm(num_joint, 0.0);
            if (biped.size() > 1)
            {
                const auto &src = biped[1];
                for (uint32_t j = 0; j < num_arm_joints; ++j)
                {
                    const int src_i = kBipedArmStart + static_cast<int>(j);
                    const int dst_i = arm_start + static_cast<int>(j);
                    if (src_i >= 0 && src_i < static_cast<int>(src.size())
                        && dst_i >= 0 && dst_i < static_cast<int>(num_joint))
                        arm[static_cast<size_t>(dst_i)] = src[static_cast<size_t>(src_i)];
                }
            }
            pos_list.push_back(arm);

            std::vector<double> head(num_joint, 0.0);
            if (biped.size() > 2)
            {
                const auto &src = biped[2];
                for (uint32_t j = 0; j < num_head_joints; ++j)
                {
                    const int src_i = kBipedHeadStart + static_cast<int>(j);
                    const int dst_i = head_start + static_cast<int>(j);
                    if (src_i >= 0 && src_i < static_cast<int>(src.size())
                        && dst_i >= 0 && dst_i < static_cast<int>(num_joint))
                        head[static_cast<size_t>(dst_i)] = src[static_cast<size_t>(src_i)];
                }
            }
            pos_list.push_back(head);
        }

        std::cout << "按回车执行下一个动作，按'q'退出检查\n" << std::endl;

        // 菜单里 getchar 只读走 's'，回车仍留在行缓冲里。先丢掉这个回车，避免刚进入自检就执行第一组动作。
        {
            int pending = 0;
            if (ioctl(STDIN_FILENO, FIONREAD, &pending) == 0 && pending > 0)
            {
                const int c = getchar();
                if (c != '\n' && c != '\r' && c != EOF)
                    std::ungetc(c, stdin);
            }
        }

        int i = 0;
        while (true) {
            initial_input_cmd_ = '\0';
            initial_input_cmd_ = getchar();
            if (initial_input_cmd_ == '\n' ) {
                this->jointMoveTo(std::vector<double>(num_joint, 0), 60, hard_dt);
                usleep(1000000);
                std::cout << "执行第 " << i + 1 << " 组动作..." << std::endl;
                this->jointMoveTo(pos_list[i], 50, hard_dt);
                i++;
                i = i % pos_list.size();
            } else if (initial_input_cmd_ == 'q') {
                std::cout << "退出电机自检" << std::endl;
                break;
            }
        }

        std::cout << "所有动作检查完成！" << std::endl;
        // return to the initial position
        std::cout << "恢复初始姿势..." << std::endl;
        this->jointMoveTo(std::vector<double>(num_joint, 0), 60, hard_dt);
        return;
    }

    
    // Refactor: 把读取 IMU 的操作变成是从函数的参数传入，在调用函数之前准备好实物或者仿真的传感器数据。
    void HardwarePlant::Update(RobotState_t state_des, Eigen::VectorXd actuation)
    {

        state_des_ = state_des;

        // //
        // std::cout << "state_des.tau_max : " << state_des.tau_max ;
        // std::cout << std::endl<<std::endl;
        // //

        Eigen::VectorXd cmd;
        uint32_t na_r = num_joint;
        // joint2motor(state_des_, actuation, cmd);
        // writeCommand(cmd, na_r, control_mode_);
        if (has_end_effectors)
            endEffectorCommand(state_des.end_effectors);

#if 0
    clock_gettime(CLOCK_MONOTONIC, &t1);
    printf("State update time: %f\n", TIME_DIFF(t0, t1) * 1000);
    printf("Realtime period: %f\n\n", TIME_DIFF(last_time, t1) * 1000);
    last_time = t1;
#endif
    }
    /**
     * @brief 翻转imu, 解决imu倒装的问题。让值绕imu自身x轴旋转180度
     *
     * @param acc 原加速度
     * @param gyro 原角速度
     * @param quat 原四元数 [w, x, y, z]
     * @param roll_acc
     * @param roll_gyro
     * @param roll_quat [w, x, y, z]
     *
     * @note roll_quat 赋值是四元数绕当前x轴旋转的一个特解, 没有使用矩阵乘法。
     */
    void Invt_imudate(SensorData_t &sensor_data)
    {
        auto roll_offset = Eigen::AngleAxisd(-M_PI, Eigen::Vector3d::UnitX()).toRotationMatrix();
        sensor_data.gyro = roll_offset * sensor_data.gyro;
        sensor_data.acc = roll_offset * sensor_data.acc;
        sensor_data.free_acc = roll_offset * sensor_data.free_acc;
        sensor_data.quat = {-sensor_data.quat[1], sensor_data.quat[0], sensor_data.quat[3], -sensor_data.quat[2]}; // [w, x, y, z]
    }

    void HardwarePlant::initRobotModule()
    {
        if (robot_module_ == "KUAVO") {
            actuators.setRobotMoudle(KUAVO);
        } else if (robot_module_ == "ROBAN2") {
            actuators.setRobotMoudle(ROBAN2);
        }
        else if (robot_module_ == "KUAVO5") {
            actuators.setRobotMoudle(KUAVO5);
        }
        else if (robot_module_ == "LUNBI") {
            actuators.setRobotMoudle(LUNBI);
        }
        else if (robot_module_ == "LUNBI_V62") {
            actuators.setRobotMoudle(LUNBI_V62);
        }
        else {
            std::cerr << "Unsupported robot module: " << robot_module_ << std::endl;
            exit(1);
        }
    }

    void HardwarePlant::initEndEffector()
    {
        const auto &eef_types = motor_info.end_effector_type;
        
        // ROS params
        {
            // std::map<EndEffectorType, std::string> eef_name_maps {
            //     {EndEffectorType::jodell, "jodell"},
            //     {EndEffectorType::qiangnao, "qiangnao"},
            //     {EndEffectorType::lejuclaw, "lejuclaw"},
            //     {EndEffectorType::revo2, "revo2"},
            //     {EndEffectorType::none, "none"}
            // };
            
            // /* // rosparam  /end_effector_type */
            // for (size_t i = 0; i < eef_types.size(); ++i) {
            //     ros::param::set("/end_effector_type", eef_name_maps[eef_types[i]]);
            // }
            
            // FIXME: rosparm set  left/right eef.
            // // rosparam  /end_effector/left/type
            // // rosparam  /end_effector/right/type
            // for (size_t i = 0; i < eef_types.size() && i < 2; ++i) {
            //     std::string side = (i == 0) ? "left" : "right";
            //     ros::param::set("/end_effector/" + side + "/type", eef_name_maps[eef_types[i]]);
            // }

            // // Set remaining sides to "none"
            // for (size_t i = eef_types.size(); i < 2; ++i) {
            //     std::string side = (i == 0) ? "left" : "right";
            //     ros::param::set("/end_effector/" + side + "/type", "none");
            // }
        }
        
        auto has_jodell = std::find(eef_types.begin(), eef_types.end(), EndEffectorType::jodell);
        auto has_qiangnao = std::find(eef_types.begin(), eef_types.end(),  EndEffectorType::qiangnao);
        auto has_lejuclaw = std::find(eef_types.begin(), eef_types.end(),  EndEffectorType::lejuclaw);
        auto has_qiangnao_touch = std::find(eef_types.begin(), eef_types.end(),  EndEffectorType::qiangnao_touch);
        auto has_revo2 = std::find(eef_types.begin(), eef_types.end(),  EndEffectorType::revo2);
        auto has_heiman = std::find(eef_types.begin(), eef_types.end(), EndEffectorType::heiman);
        auto has_linker_hand = std::find(eef_types.begin(), eef_types.end(),  EndEffectorType::linker_hand);
        std::vector<bool> eef_init_ok;

        HighlyDynamic::CanbusWiringType canbus_mode = motor_info.getCanbusWiringType(rb_version_);
        HighlyDynamic::HandProtocolType hand_protocol_type = motor_info.getHandProtocolType();
        bool is_dual_bus = canbus_mode == HighlyDynamic::CanbusWiringType::DUAL_BUS;
        bool is_can_protocol = hand_protocol_type == HighlyDynamic::HandProtocolType::PROTO_CAN;

        // Init Qiangnao DexterousHand (linker_hand 复用 qiangnao 协议)
        if (has_qiangnao != eef_types.end() || has_linker_hand != eef_types.end()) {
            std::cout << " =======> 末端执行器类型: "
                      << (has_linker_hand != eef_types.end() ? "linker_hand" : has_heiman != eef_types.end() ? "heiman" : "qiangnao")
                      << "<============" << std::endl;
            gesture_filepath_ = kuavo_settings_.kuavo_assets_path + "/config/gesture/preset_gestures.json";
            dexhand_actuator = eef_controller::DexhandController::Create(gesture_filepath_, false, is_can_protocol);
            if (!dexhand_actuator->init()) {
                printf("[HardwarePlant] initEndEffector StarkDexhand init failed \n");
                eef_init_ok.push_back(false);
            }
            else {
                std::cout << "[HardwarePlant] initEndEffector StarkDexhand init success!! \r\n";
            }
        }

        // Init Leju Claw
        if (has_lejuclaw != eef_types.end()) {
            std::cout << " =======> 末端执行器类型: lejuclaw<============" << std::endl;
            // update config.yaml
            std::string src_path = kuavo_settings_.kuavo_assets_path + "/config/config.yaml";
            if(!CopyOrUpdateLejuClawConfigFile(src_path)) {
                printf("[HardwarePlant] initEndEffector Leju Claw config file update failed \n");
            }

            leju_claw_actuator = LejuClawController::Create(is_dual_bus);
            leju_claw_actuator->setDebugCallback(leju_claw_debug_callback_);
            leju_claw_actuator->setTargetCallback(leju_claw_target_callback_);
            /* Because the ruiwocontroller was called `BM_Init` , so we can't init it again.*/
            /* See details of `void BM_Init()` in  bmapi.h */
            bool init_bmapilib = false;
            if(!leju_claw_actuator->initialize(init_bmapilib)) {
                printf("[HardwarePlant] initEndEffector Leju Claw init failed \n");
                eef_init_ok.push_back(false);
            }
        }

        // FIXME: 当前不支持触觉手和非触觉手同时使用!!!!
        if(has_qiangnao_touch != eef_types.end()) {
            std::cout << " =======> 末端执行器类型: qiangnao_touch<============" << std::endl;
            if(has_qiangnao != eef_types.end() || has_linker_hand != eef_types.end()) {
                // FIXME: 当前不支持触觉手和非触觉手同时使用!!!!
                std::cerr << "\033[33m当前不支持触觉手和非触觉手同时使用, 放弃初始化触觉灵巧手！\033[0m" << std::endl;
            }
            else {
                gesture_filepath_ = kuavo_settings_.kuavo_assets_path + "/config/gesture/preset_gestures.json";
                // TODO: qiangnao_touch 触觉手暂时固定使用 485 协议 (is_can_protocol = false)
                dexhand_actuator = eef_controller::DexhandController::Create(gesture_filepath_, true, false);

                if (!dexhand_actuator->init()) {
                    printf("[HardwarePlant] initEndEffector TouchDexhand init failed \n");
                    eef_init_ok.push_back(false);
                }
                else {
                    std::cout << "[HardwarePlant] initEndEffector TouchDexhand init success!! \r\n";
                }
            }
            
        }
        // Init Revo2
        if (has_revo2 != eef_types.end()) {
            std::cout << " =======> 末端执行器类型: revo2<============" << std::endl;
            do{
                // if(!rb_version_.start_with(1)) {
                //     printf("\033[33m************************** !!! WARNING 警告 WARNING !!!! **************************\033[0m\n");
                //     printf("\033[33m[WARN] 当前 revo2 类型的末端执行器和您的机器人版本不匹配，reov2 仅支持 Roban2 机器人 \033[0m\n");
                //     printf("\033[33m[WARN] 当前 revo2 类型的末端执行器和您的机器人版本不匹配，reov2 仅支持 Roban2 机器人 \033[0m\n");
                //     break;
                // }
                // 初始化
                std::string gesture_filepath = kuavo_settings_.kuavo_assets_path + "/config/gesture/preset_gestures.json";
                revo2_actuator = eef_controller::Revo2HandController::Create(gesture_filepath, is_can_protocol);
                if(!revo2_actuator->init()) {
                    printf("[HardwarePlant] initEndEffector Revo2 DexterousHand init failed \n");
                    eef_init_ok.push_back(false);
                }
                else {
                    std::cout << "[HardwarePlant] initEndEffector Revo2 DexterousHand init success!! \r\n";
                }
            } while(false);
        }   

        // WARNNING and TIPS, wait for user input 'enter' or 'Ctrl+C' to continue.
        if(!eef_init_ok.empty()) {
            printf("\033[33m************************** !!! WARNING 警告 WARNING !!!! **************************\033[0m\n");
            printf("\033[33m[WARN] 机器人末端执行器初始化失败 \033[0m\n");
        }

        std::cout << "[HardwarePlant] init end-effector end......\n";
    }

    // bool HardwarePlant::controlLejuClawCallback(kuavo_msgs::controlLejuClawRequest &req,
    //                       kuavo_msgs::controlLejuClawResponse &res)
    //                       {
    //                         return leju_claw_actuator->controlGripperCallback(req, res);
    // }

    // void HardwarePlant::pubLejuClawStateTimerCallback(const ros::TimerEvent& event)
    // {
    //     return ROSPublishLejuClawState(leju_claw_actuator, leju_claw_state_pub_);
    // }

    // void HardwarePlant::lejuClawCommandCallback(const kuavo_msgs::lejuClawCommand::ConstPtr &msg)
    // {
    //     if (leju_claw_actuator != nullptr)  {
    //         leju_claw_actuator->commandCallback(msg);
    //     }
    // }

    // bool HardwarePlant::getHardwareReadyCallback(std_srvs::Trigger::Request &req,
    //                             std_srvs::Trigger::Response &res)
    // {
    //     res.success = cali_set_zero_status == 0;
    //     res.message = cali_set_zero_status == 0 ? "Hardware is ready" : "Hardware is not ready";
    //     return true;
    // }

    // bool HardwarePlant::getMotorZeroPointsCallback(kuavo_msgs::getMotorZeroPointsRequest &req,
    //                                         kuavo_msgs::getMotorZeroPointsResponse &res)
    // {
    //     std::vector<double> offset;
    //     getMotorPositionOffset(offset);

    //     offset.resize(countECMasters);

    //     std::vector<double> zero_points;
    //     zero_points = ruiwo_actuator->getMotorZeroPoints();
    //     // Convert radians to degrees for RUIWO motors
    //     std::transform(zero_points.begin(), zero_points.end(), zero_points.begin(),
    //                   [](double rad) { return rad * TO_DEGREE; });
    //     offset.insert(offset.end(), zero_points.begin(), zero_points.end());
        
    //     res.zero_points = offset;
    //     res.success = true;
    //     res.message = "Successfully retrieved motor zero points.";
    //     return true;
    // }

    bool HardwarePlant::changeMotorParam(const std::vector<MotorParam> &motor_params, std::string &err_msg)
    {
        std::vector<uint16_t> ec_ids;
        std::vector<int32_t> ec_kps;
        std::vector<int32_t> ec_kds;

        // 检查 ID 范围且是否存在
        for(const auto &param : motor_params) {
            // 检查电机ID是否在joint_ids中存在
            if(std::find(joint_ids.begin(), joint_ids.end(), param.id) == joint_ids.end()) {
                err_msg = "Motor ID " + std::to_string(param.id) + " not found in joint_ids";
                return false;
            }
            
            int index = param.id - 1; // 判断 ID 对应的驱动类型
            if(motor_info.driver[index] == EC_MASTER) {
                if(motor_info.motors_exist[index]) {
                    ec_ids.push_back(ec_index_map_[index]); // 找到实际的 ECMASTER ID
                    ec_kps.push_back(static_cast<int32_t>(param.Kp));
                    ec_kds.push_back(static_cast<int32_t>(param.Kd));
                }
                else {
                    std::cerr << "[HardwarePlant]:changeMotorParam Motor ID " << param.id << " not found in ecmaster\n";
                }
            }
            else {
                // TODO 其他类型电机更改Kp, Kd
            }
        }

        if(!ec_ids.empty()) {
            if(driver_type[0] != EcMasterType::YD) {
                err_msg = "Only support Youda motor";
                return false;
            }

            std::cout << "[HardwarePlant] Writing motor parameters:" << std::endl;
            for(size_t i = 0; i < ec_ids.size(); i++) {
                std::cout << "  Motor ID: " << ec_ids[i] 
                         << ", Kp: " << ec_kps[i]
                         << ", Kd: " << ec_kds[i] << std::endl;
            }

            if(0 != actuators.writeJointKd(ec_ids, driver_type[0], ec_kds)) {
                err_msg = "EcDriver writeJointKd failed";
                return false;
            }
            if(0 != actuators.writeJointKp(ec_ids, driver_type[0], ec_kps)) {
                err_msg = "EcDriver writeJointKp failed";
                return false;
            }
        }


        return true;
    }

    bool HardwarePlant::getMotorParam(std::vector<MotorParam> &motor_params, std::string &err_msg)
    {
        // Classify IDs based on their types
        std::vector<uint16_t> ecMasterIds;

        for (size_t i = 0; i < joint_ids.size(); ++i)
        {
            size_t index = joint_ids[i] - 1;
            // Check the driver type and add to the appropriate vector
            if (motor_info.driver[index] == EC_MASTER)
            {                
                // Only add the motor that exist 
                if (motor_info.motors_exist[index]) {
                    ecMasterIds.push_back(ec_index_map_[index]);
                }
            }
        }

        motor_params.clear();
        for (size_t i = 0; i < joint_ids.size(); ++i)
        {
            motor_params.push_back(MotorParam{joint_ids[i], 0, 0});
        }
        
        // int motor_id = 1;
        if(!ecMasterIds.empty()) {
            if(driver_type[0] != EcMasterType::YD) {
                err_msg = "Only support Youda motor";
                return false;
            }
            std::vector<int32_t> joint_kp;
            std::vector<int32_t> joint_kd;
            if(0 != actuators.readJointKp(ecMasterIds, driver_type[0], joint_kp)) {
                err_msg = "EcDriver readJointKp failed";
                return false;
            }
            if(0 != actuators.readJointKd(ecMasterIds, driver_type[0], joint_kd)) {
                err_msg = "EcDriver readJointKd failed";
                return false;
            }

            for(size_t i = 0; i < ecMasterIds.size(); ++i) {
            // ECMASTER ID --> index
            auto it = std::find_if(ec_index_map_.begin(), ec_index_map_.end(),
                [&](const std::pair<int, int>& pair) { return pair.second == ecMasterIds[i]; });

            // index --> joint id    
            if (it != ec_index_map_.end()) {
                auto id = it->first + 1;
                auto param_it = std::find_if(motor_params.begin(), motor_params.end(),
                    [&](const MotorParam& param) { return param.id == id; });
                // Found joint id
                if(param_it != motor_params.end()) {
                    param_it->Kp = static_cast<double>(joint_kp[i]);
                    param_it->Kd = static_cast<double>(joint_kd[i]);
                }
                else {
                    err_msg = "MotorParam not found";
                    return false;
                }
            }
            }
        }
        
        // TODO: 先不考虑 Ruiwo 电机

        return true;
    }

    bool HardwarePlant::changeRuiwoMotorParam(const std::string &param_name, std::string &err_msg)
    {
        if (ruiwo_actuator == nullptr) {
            err_msg = "Ruiwo actuator is not initialized";
            return false;
        }

        JSONConfigReader *robot_config = kuavo_common_ptr_->getRobotConfig();
        if (robot_config == nullptr) {
            err_msg = "Robot config is not initialized";
            return false;
        }

        auto nested_obj = (*robot_config)[param_name];
        if (!nested_obj.contains("ruiwo_kp") || !nested_obj.contains("ruiwo_kd")) {
            err_msg = "Nested object '" + param_name + "' does not contain ruiwo_kp or ruiwo_kd";
            return false;
        }
        std::vector<double> kp_values = nested_obj["ruiwo_kp"].get<std::vector<double>>();
        std::vector<double> kd_values = nested_obj["ruiwo_kd"].get<std::vector<double>>();

        if (kp_values.empty() || kd_values.empty()) {
            err_msg = "Failed to load ruiwo_kp or ruiwo_kd from '" + param_name + "'";
            return false;
        }

        if (kp_values.size() != kd_values.size()) {
            err_msg = "ruiwo_kp and ruiwo_kd size mismatch";
            return false;
        }

        // 设置参数
        std::vector<int> joint_indices;

        for (size_t i = 0; i < kp_values.size(); ++i) {
            joint_indices.push_back(i);
        }
        
        std::cout << "[HardwarePlant] Setting RuiWo joint gains from config (param: " << param_name << ")..." << std::endl;
        ruiwo_actuator->set_joint_gains(joint_indices, kp_values, kd_values);
        std::cout << "[HardwarePlant] RuiWo joint gains set successfully" << std::endl;
        
        return true;
    }

    eef_controller::FingerStatusArray HardwarePlant::getHandControllerStatus() {
        if(dexhand_actuator) {
            return dexhand_actuator->get_finger_status();
        }
        else {
            return eef_controller::FingerStatusArray();
        }
    }

    void HardwarePlant::setMotorStatusHardwareSettings() 
    {
        if (motor_status_manager_) 
        {
            motor_status_manager_->setHardwareSettings(motor_info);
        }
    }

    void HardwarePlant::markJointAsDisabled(int joint_id, const std::string& reason)
    {
        if (motor_status_manager_) {
            motor_status_manager_->setJointStatus(joint_id, 
                MotorStatus::DISABLED, reason);
        }
    }

    bool HardwarePlant::isJointMarkedAsDisabled(int joint_id) const
    {
        if (!motor_status_manager_) {
            return false;
        }
        
        auto status_map = motor_status_manager_->getAllJointsStatus();
        auto it = status_map.find(joint_id);
        return (it != status_map.end() && 
               (it->second == MotorStatus::DISABLED || 
                it->second == MotorStatus::ERROR));
    }

    std::map<int, MotorStatus> HardwarePlant::getAllJointsStatus() const
    {
        if (motor_status_manager_) {
            return motor_status_manager_->getAllJointsStatus();
        }
        return std::map<int, MotorStatus>();
    }

    bool HardwarePlant::setZeroTorqueForLegECMotors()
    {
        // 从joint_ids中选择腿部关节ID（EC_MASTER）
        std::vector<uint8_t> leg_joint_ids;
        for (size_t i = 0; i < joint_ids.size(); ++i)
        {
            size_t index = joint_ids[i] - 1;
            if (motor_info.driver[index] == EC_MASTER)
            {
                leg_joint_ids.push_back(joint_ids[i]);
            }
        }

        int set_zero_torque_ec_motor_num = static_cast<int>(leg_joint_ids.size());
        int expected_ec_motor_num = 6;
        if (robot_module_ == "LUNBI_V62")
            expected_ec_motor_num = 4;
        if (expected_ec_motor_num != set_zero_torque_ec_motor_num)
        {
            std::cout << "current ec motor size is not equal to expected " << expected_ec_motor_num 
                    << ", actual size: " << set_zero_torque_ec_motor_num << std::endl;
            return false;
        }

        setZeroTorqueForJoints(leg_joint_ids);
        return true;
    }

    bool HardwarePlant::exitZeroTorqueMode()
    {
        // 确定腿部关节数量（根据宏定义判断）
        int leg_joint_count = na_foot_;
        std::cout << "[HardwarePlant] Exiting zero torque mode for leg joints 1-" << na_foot_ << std::endl;

        // 从joint_ids中选择前 leg_joint_count 个关节（腿部关节）
        std::vector<uint8_t> leg_joint_ids;
        for (int i = 0; i < leg_joint_count && i < static_cast<int>(joint_ids.size()); i++)
        {
            leg_joint_ids.push_back(joint_ids[i]);
        }

        exitZeroTorqueForJoints(leg_joint_ids);
        std::cout << "[HardwarePlant] Normal control mode restored for leg joints" << std::endl;
        return true;
    }

    // 内部复用实现：对给定 EC 关节 id 列表下发 0 扭矩
    void HardwarePlant::setZeroTorqueForJoints(const std::vector<uint8_t>& ec_ids)
    {
        if (ec_ids.empty())
        {
            return;
        }
        std::vector<JointParam_t> zero_torque_cmd(ec_ids.size());
        for (auto& cmd : zero_torque_cmd)
        {
            cmd.position = 0.0;  // 位置设为0，实际会保持当前位置
            cmd.velocity = 0.0;  // 速度为0
            cmd.torque = 0.0;    // 扭矩为0
            cmd.maxTorque = 0.0; // 最大扭矩为0
        }
        SetMotorTorque(ec_ids, zero_torque_cmd);
    }

    // 内部复用实现：对给定 EC 关节 id 列表恢复位置控制（保持当前位置）
    void HardwarePlant::exitZeroTorqueForJoints(const std::vector<uint8_t>& ec_ids)
    {
        if (ec_ids.empty())
        {
            return;
        }
        GetMotorData(joint_ids, joint_data);

        std::vector<JointParam_t> normal_cmd;
        normal_cmd.reserve(ec_ids.size());
        for (uint8_t id : ec_ids)
        {
            size_t index = id - 1;
            JointParam_t item;
            item.position = joint_data[index].position;  // 保持当前位置
            item.velocity = 0.0;
            item.torque = 0.0;
            item.maxTorque = motor_info.max_current[index];  // 恢复最大扭矩限制
            normal_cmd.push_back(item);
        }

        setDefaultKpKd(normal_cmd, ec_ids);
        SetMotorPosition(ec_ids, normal_cmd);
    }

    bool HardwarePlant::setZeroTorqueForSingleLegJoint(int joint_idx)
    {
        if (joint_idx < 0 || joint_idx >= static_cast<int>(joint_ids.size()))
        {
            std::cout << "[HardwarePlant] 关节索引 " << joint_idx << " 超出范围" << std::endl;
            return false;
        }
        if (motor_info.driver[joint_idx] != EC_MASTER)
        {
            std::cout << "[HardwarePlant] 关节 " << joint_idx + 1 << " 不是 EC_MASTER 电机，无法设置0扭矩" << std::endl;
            return false;
        }

        std::vector<uint8_t> ids = { joint_ids[joint_idx] };
        setZeroTorqueForJoints(ids);
        return true;
    }

    bool HardwarePlant::exitZeroTorqueForSingleLegJoint(int joint_idx)
    {
        if (joint_idx < 0 || joint_idx >= static_cast<int>(joint_ids.size()))
        {
            std::cout << "[HardwarePlant] 关节索引 " << joint_idx << " 超出范围" << std::endl;
            return false;
        }

        std::vector<uint8_t> ids = { joint_ids[joint_idx] };
        exitZeroTorqueForJoints(ids);
        return true;
    }

    bool HardwarePlant::setCurrentPositionAsOffsetForJoint(int joint_idx)
    {
        if (joint_idx < 0 || joint_idx >= static_cast<int>(joint_ids.size()))
        {
            std::cout << "[HardwarePlant] 关节索引 " << joint_idx << " 超出范围" << std::endl;
            return false;
        }
        if (motor_info.driver[joint_idx] != EC_MASTER)
        {
            std::cout << "[HardwarePlant] 关节 " << joint_idx + 1 << " 不是 EC_MASTER 电机，无法锁定零点" << std::endl;
            return false;
        }

        // 找到该关节对应的 EC 索引（在所有 EC_MASTER 电机中的索引）
        int ec_idx = -1;
        int ec_count = 0;
        for (int j = 0; j <= joint_idx; ++j)
        {
            if (motor_info.driver[j] == EC_MASTER)
            {
                if (j == joint_idx)
                {
                    ec_idx = ec_count;
                    break;
                }
                ec_count++;
            }
        }
        if (ec_idx < 0)
        {
            std::cout << "[HardwarePlant] 无法找到关节 " << joint_idx + 1 << " 的 EC 索引" << std::endl;
            return false;
        }

        std::vector<double> offset;
        getMotorPositionOffset(offset);
        if (ec_idx >= static_cast<int>(offset.size()))
        {
            std::cout << "[HardwarePlant] EC 索引 " << ec_idx << " 超出 offset 范围" << std::endl;
            return false;
        }

        GetMotorData(joint_ids, joint_data);
        double current_pos = joint_data[joint_idx].position;
        double old_offset = offset[ec_idx];
        double new_offset = current_pos + old_offset;  // 与 setCurrentPositionAsOffset 一致
        offset[ec_idx] = new_offset;
        setMotorPositionOffset(offset);
        saveOffset();
        std::cout << "[HardwarePlant] 关节 " << joint_idx + 1 << " (EC[" << ec_idx
                  << "]) 零点偏移: " << old_offset << " -> " << new_offset << " 度，已保存" << std::endl;
        return true;
    }

    bool HardwarePlant::manualCalibrateSingleLegJoint(int joint_idx)
    {
        if (hardware_ready_)
        {
            std::cout << "[HardwarePlant] 硬件正在运行，禁止手动标定，请先停止！" << std::endl;
            return false;
        }
        if (!setZeroTorqueForSingleLegJoint(joint_idx))
        {
            return false;
        }

        std::cout << "\033[32m已进入关节 " << joint_idx + 1 << " 的 0 扭矩模式，现在可以手动摆正该关节...\033[0m" << std::endl;
        std::cout << "\033[32m摆正完成后，输入 'x' 锁定当前位为该关节零点并退出 0 扭矩模式\033[0m" << std::endl;
        std::cout << "\033[32m输入 'q' 取消（退出 0 扭矩，不保存零点）\033[0m" << std::endl;

        char exit_zero_torque = '\0';
        while (exit_zero_torque != 'x' && exit_zero_torque != 'q' && exit_zero_torque != 'Q')
        {
            // 持续发送0扭矩命令，保持0扭矩状态
            setZeroTorqueForSingleLegJoint(joint_idx);

            if (kbhit())
            {
                exit_zero_torque = getchar();
                if (exit_zero_torque == '\n')
                {
                    continue; // 忽略回车键
                }
            }
            usleep(100000);
        }

        // 取消：退出 0 扭矩但不保存零点
        if (exit_zero_torque == 'q' || exit_zero_torque == 'Q')
        {
            exitZeroTorqueForSingleLegJoint(joint_idx);
            std::cout << "\033[33m已取消腰部标定，恢复正常控制\033[0m" << std::endl;
            return false;
        }

        if (!exitZeroTorqueForSingleLegJoint(joint_idx))
        {
            std::cout << "\033[31m退出 0 扭矩模式失败\033[0m" << std::endl;
            return false;
        }
        std::cout << "\033[32m已退出 0 扭矩模式，恢复正常控制\033[0m" << std::endl;

        if (!setCurrentPositionAsOffsetForJoint(joint_idx))
        {
            std::cout << "\033[31m锁定零点失败\033[0m" << std::endl;
            return false;
        }
        std::cout << "\033[32m关节 " << joint_idx + 1 << " 零点已锁定并保存\033[0m" << std::endl;
        return true;
    }

    void HardwarePlant::adjustZeroPosition(int motor_index, double offset) {
        if (getRuiwoActuator() != nullptr) {
            // 将电机索引转换为RUIWO电机索引
            int ruiwo_motor_index = 0;
            for(int i = 0; i < motor_index; i++) {
                if(motor_info.driver[i] == RUIWO){
                    ruiwo_motor_index++;
                }
            }
            
            offset = offset * TO_RADIAN;
            getRuiwoActuator()->adjustZeroPosition(ruiwo_motor_index, offset);
            //ruiwo_actuator->adjustZeroPosition(ruiwo_motor_index, offset);
        }
    }

    std::vector<double> HardwarePlant::getMotorZeroPoints() {
        std::vector<double> zero_points;
        if (getRuiwoActuator() != nullptr) {
            zero_points = getRuiwoActuator()->getMotorZeroPoints();
            //zero_points = ruiwo_actuator->adjustZeroPosition(ruiwo_motor_index, adjust_pos)
            // Convert radians to degrees for RUIWO motors
            std::transform(zero_points.begin(), zero_points.end(), zero_points.begin(),
                        [](double rad) { return rad * TO_DEGREE; });
        }
        return zero_points;
    }

    RuiwoActuatorBase* HardwarePlant::getRuiwoActuator() 
    {
        return ruiwo_actuator;
    }

    void HardwarePlant::writeHardwareJointCommand(Eigen::VectorXd cmd, uint32_t na_r, std::vector<int> control_modes, Eigen::VectorXd &joint_kp, Eigen::VectorXd &joint_kd)
    {
        Eigen::VectorXd cmd_out(na_r*5);
        cmds2Cmdr(cmd, na_r, cmd_out, na_r);
        if (cmd_out.hasNaN())
        {
            std::cout << "cmd_out.hasNaN \ncmd_out:" << cmd_out.segment(na_r * 2, na_r).segment(10, 2).transpose() << std::endl;
            std::cout << "cmd" << cmd.transpose() << std::endl;
            std::cout << "cmd_out:" << cmd_out.transpose() << std::endl;
            cmd_out.setZero();
        }
        writeHardwareCommand(cmd_out, na_r, control_modes, joint_kp, joint_kd);
    }

    void HardwarePlant::readHardwareJointState(SensorData_t &sensor_data_joint)
    {
        SensorData_t sensor_data_motor;
        if (readHardwareState(sensor_data_motor))
        {
            motor2joint(sensor_data_motor, sensor_data_joint);
            setState(sensor_data_motor, sensor_data_joint);
        }
    }
}; // namespace HighlyDynamic
