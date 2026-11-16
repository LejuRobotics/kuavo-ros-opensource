#pragma once

#include <iostream>
#include <string>
#include <vector>
#include <Eigen/Core>
#include "kuavo_common/common/robot_state.h"
#include "kuavo_common/common/json_config_reader.hpp"
#include "kuavo_common/common/common.h"
namespace HighlyDynamic
{

#define MOTOR_CONTROL_MODE_TORQUE 0
#define MOTOR_CONTROL_MODE_VELOCITY 1
#define MOTOR_CONTROL_MODE_POSITION 2
#define BIT_17 (1 << 17)
#define BIT_17_9 (BIT_17 * 9)
#define BIT_17_10 (BIT_17 * 10)
#define BIT_17_16 (BIT_17 * 16)
#define BIT_17_18 (BIT_17 * 18)
#define BIT_17_20 (BIT_17 * 20)
#define BIT_17_25 (BIT_17 * 25)
//电机减速比25.775，乘2e17后为3378380
#define BIT_17_251 (3378380) 
#define BIT_17_36 (BIT_17 * 36)
#define BIT_17_120 (BIT_17 * 120)

// ---------------- 电流 MC 宏（命名规则：MOTORS_TYPE 中的电机全名 + _MC） ----------------
// 说明：电流（A），命名与 MOTORS_TYPE 电机全名一一对应
#define AK10_9_MC (40)
#define AK70_10_MC (26.1) // 手册是 23.2
#define CK_MC (18)
#define PA100_10_MC (70)
#define PA100_20_MC (70)
#define PA100_20_18_KV60_MC (70)
#define PA43_10_25_KV70_MC (8)
#define PA60_36_MC (70)
#define PA72_10_36_KV100_MC (15)
#define PA76_15_25_KV45_ZHK_MC (18)
#define PA76_15_18_KV45_MC (31.5)     // PA76-15-18-KV45（v51）
#define PA76_15_18_KV70_MC (31.5)     // PA76-15-18-KV70（v52+）
#define PA81_10_MC (60)
#define PA81_18_25_KV60_ZHK_MC (18)
#define PA81_25_KV60_ZHK_MC (40)
#define PA105_18_DS_MC (70)
#define PA105_27_18_KV35_MC (70)
#define HA115_20_120_MC (40)
#define HA115_10_120_ZHK_MC (40)
#define PA4315_36_MC (22.5)

// ruiwo 系列（驱动器为 RUIWO，型号名带 ruiwo 前缀）
#define ruiwoPA81_18_25_KV60_ZHK_MC (18)
#define ruiwoPA72_10_36_KV50_MC (18)
#define ruiwoPA60_13_36_KV50_MC (18)
#define ruiwoPA43_10_25_KV70_MC (18)
#define ruiwoPA43_10_25_KV70_PREV_MC (18)
#define ruiwoPA4315_36_MC (18)
#define ruiwoPA60_10_16_ZHK_KV35_MC (18)

// 占位/方位变体
#define dynamixel_MC CK_MC
#define realman_MC CK_MC
#define ruiwo_MC CK_MC
#define PA72_10_36_KV100_L_MC PA72_10_36_KV100_MC
#define PA72_10_36_KV100_R_MC PA72_10_36_KV100_MC

// ---------------- C2T 标定宏（命名规则：MOTORS_TYPE 中的电机全名 + _C2T） ----------------
// 说明：kuavo.json 的 MOTOR_C2T 分段表优先，此处为兜底值；键名与 motor_name_map 一一对应
#define AK10_9_C2T (1.26)
#define AK70_10_C2T (1.23)
#define CK_C2T (2.1) // 1.4
#define PA100_10_C2T (1.2) // 1.2
#define PA100_20_C2T (2.4)
#define PA100_20_18_KV60_C2T (2.08)
#define PA43_10_25_KV70_C2T (4.7)
#define PA60_36_C2T (2.0)
#define PA72_10_36_KV100_C2T (4.8)
#define PA76_15_25_KV45_ZHK_C2T (4.2)
#define PA76_15_18_KV45_C2T (3.2)      // PA76-15-18-KV45（v51）
#define PA76_15_18_KV70_C2T (2.19)     // PA76-15-18-KV70（v52+）
#define PA81_10_C2T (2.55)
#define PA81_18_25_KV60_ZHK_C2T (3.13)
#define PA81_25_KV60_ZHK_C2T (2.9)
#define PA105_18_DS_C2T (2.8)
#define PA105_27_18_KV35_C2T (4.13)
#define HA115_20_120_C2T (15.76)
#define HA115_10_120_ZHK_C2T (14.58)
#define PA4315_36_C2T (4.7)

// ruiwo 系列（驱动器为 RUIWO，型号名带 ruiwo 前缀）
#define ruiwoPA81_18_25_KV60_ZHK_C2T (2.55)
#define ruiwoPA72_10_36_KV50_C2T (3.6)
#define ruiwoPA60_13_36_KV50_C2T (2.0)
#define ruiwoPA43_10_25_KV70_C2T (2.189438) // 当前规范标定：PA4310-25 实测扭矩系数均值；有 MOTOR_C2T 分段表时以表为准
#define ruiwoPA43_10_25_KV70_PREV_C2T (4.7)  // 旧标定（仅 ruiwoPA43_10_25_KV70_PREV 使用）
#define ruiwoPA4315_36_C2T (4.7)
#define ruiwoPA60_10_16_ZHK_KV35_C2T (2.0)

// 占位/方位变体
#define dynamixel_C2T CK_C2T
#define realman_C2T CK_C2T
#define ruiwo_C2T CK_C2T
#define PA72_10_36_KV100_L_C2T PA72_10_36_KV100_C2T
#define PA72_10_36_KV100_R_C2T PA72_10_36_KV100_C2T

#define LEG_DOF 6
#define LEGS_TOTEL_JOINT 12

    enum class CanbusWiringType {
        SINGLE_BUS,  // 单总线
        DUAL_BUS,    // 双总线 左右手臂各接一个CAN模块
        UNKNOWN      // 未知
    };
    enum class HandProtocolType {
        PROTO_BUF,  // 485协议
        PROTO_CAN,  // CAN协议
        UNKNOWN  // 未知
    };
    inline auto vectorToEigen(std::vector<double> v) -> Eigen::VectorXd
    {
        return Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(v.data(), v.size());
    }

    struct PID
    {
        Eigen::VectorXd kp;
        Eigen::VectorXd ki;
        Eigen::VectorXd kd;
    };
    struct PidParams
    {
        PID pos;
        PID pos_sim;
        PID arm;
        PID pbc;
    };
    struct PredefinedArmPose
    {
        Eigen::VectorXd init_arm_pos;
        Eigen::VectorXd walk_arm_pose;
        Eigen::VectorXd calibration_safe_pose;
        Eigen::VectorXd arm_calibration_limits;
        Eigen::VectorXd arm_calibration_directions;
        double arm_calibration_velocity;
        double arm_calibration_timeout;
        double arm_calibration_position_variance_time;
        double arm_calibration_position_variance_threshold;
        std::vector<Eigen::VectorXd> arm_poses;
        // 腿部校准参数
        Eigen::VectorXd leg_calibration_safe_pose;      // 安全姿态
        Eigen::VectorXd leg_calibration_limits;        // 目标限位位置
        Eigen::VectorXd leg_calibration_directions;    // 运动方向
    };
    struct ModelSettings
    {
        // urdf path
        std::string model_path;
        std::string model_with_arm_path;
        std::string left_leg_urdf;
        std::string right_leg_urdf;
        std::string left_arm_urdf;
        std::string right_arm_urdf;
        std::string arm_urdf;
        //
        Eigen::VectorXd ankle_motor_offset_degree;
        Eigen::VectorXd arm_ankle_motor_offset_degree;
        uint8_t NUM_JOINT;
        uint8_t NUM_ARM_JOINT;
        uint8_t NUM_HEAD_JOINT;
        uint8_t NUM_WAIST_JOINT;
        bool is_parallel_arm;
        // frames
        std::vector<std::string> end_frames_name;
        std::vector<std::string> contact_frames_name;
        std::vector<std::string> left_foot_ankle_link_joint_frames;
        std::vector<std::string> right_foot_ankle_link_joint_frames;
        std::vector<std::string> left_arm_ankle_link_joint_frames;
        std::vector<std::string> right_arm_ankle_link_joint_frames;
    };

    struct RunningSettings
    {
        double torso_pitch;
        double torso_yaw;
        double step_height;
        double com_z;
        double com_z_jump;
        double step_with;
        double step_duration;
        Eigen::VectorXd velocity_limit;
        Eigen::VectorXd cmd_vel_step;
        uint8_t walk_stablizer_count;
        double walk_stablizer_threshold;
        double v_takeoff;
        bool swing_arm;
        bool only_half_up_body = false;
        bool use_vr_arm_kpkd = false;
        std::vector<int32_t> joint_kp;
        std::vector<int32_t> joint_kd;
        std::vector<double> ruiwo_kp;
        std::vector<double> ruiwo_kd;
        std::vector<int32_t> vr_joint_kp;
        std::vector<int32_t> vr_joint_kd;
        std::vector<double> vr_ruiwo_kp;
        std::vector<double> vr_ruiwo_kd;
        // EC/YD驱动器温度限幅值（对象索引0x3F0D，单位：℃）
        // 为空则不写入；元素个数应与EC电机数一致，按EC电机顺序对应
        std::vector<int32_t> temperature_limit;
    };

    struct HardwareSettings
    {
        int inner_size = 10;
        bool imu_invert;
        uint8_t num_joints;
        uint8_t num_arm_joints;
        uint8_t num_head_joints;
        double peak_timeWin;
        double speed_timeWin;
        double lock_rotor_timeWin;
        uint8_t num_waist_joints = 0;
        std::string robot_module;
        Eigen::VectorXd imu_in_torso;
        std::vector<std::string> motors_type;
        std::vector<uint8_t> joint_ids;
        std::vector<MotorDriveType> driver;
        std::vector<uint32_t> encoder_range;
        std::vector<double> max_current;
        std::vector<double> c2t_coeff_default;
        std::vector<std::vector<double>> c2t_coeff;
        std::vector<std::vector<double>> c2t_cul;
        std::vector<double> min_joint_position_limits;
        std::vector<double> max_joint_position_limits;
        std::vector<double> joint_velocity_limits;
        std::vector<double> joint_peak_velocity_limits;
        std::vector<double> joint_lock_rotor_limits;
        std::vector<double> joint_peak_limits;
        std::vector<EndEffectorType> end_effector_type;
        double joint_limit_torque_ratio = 1.0;  // 到达关节限位时触发保护所需的扭矩阈值倍率（相对于peak torque limit），默认1.0倍

        std::vector<bool> motors_exist;
        std::vector<bool> motors_disable;
        void resizeMotor(uint8_t num_joints)
        {
            joint_ids.resize(num_joints);
            motors_type.resize(num_joints);
            driver.resize(num_joints);
            encoder_range.resize(num_joints);
            max_current.resize(num_joints);
            c2t_coeff_default.resize(num_joints);
            c2t_coeff.resize(num_joints); // 设置外层向量大小
            for (auto& inner_vector : c2t_coeff) {
                inner_vector.resize(inner_size); // 设置每个内层向量的大小
            }
            c2t_cul.resize(num_joints); // 设置外层向量大小
            for (auto& inner_vector : c2t_cul) {
                inner_vector.resize(inner_size); // 设置每个内层向量的大小
            }
            min_joint_position_limits.resize(num_joints);
            max_joint_position_limits.resize(num_joints);
            joint_velocity_limits.resize(num_joints);
            joint_peak_velocity_limits.resize(num_joints);
            joint_lock_rotor_limits.resize(num_joints);
            joint_peak_limits.resize(num_joints);
            motors_exist.resize(num_joints);
            motors_disable.resize(num_joints);
        }
        std::string getEcmasterType(RobotVersion rb_version = RobotVersion(4, 0));
        std::string getIMUType(RobotVersion rb_version = RobotVersion(4, 0));
        /**
         * @brief 获取CanBus接线方式:单总线、双总线
         * 
         * @return CanbusWiringType 
         */
        CanbusWiringType getCanbusWiringType(RobotVersion rb_version);

        /**
         * @brief 获取手部协议类型: Protobuf、CAN
         * 
         * @return HandProtocolType 
         */
        HandProtocolType getHandProtocolType();
    };
    struct MotorC2TSettings
    {
        int inner_size = 10;
        std::vector<std::vector<double>> c2t_cul;
        std::vector<std::vector<double>> c2t_coeff;
        void resizeMotor(uint8_t num_joints)
        {
            c2t_coeff.resize(num_joints); // 设置外层向量大小
            for (auto& inner_vector : c2t_coeff) {
                inner_vector.resize(inner_size); // 设置每个内层向量的大小
            }
            c2t_cul.resize(num_joints); // 设置外层向量大小
            for (auto& inner_vector : c2t_cul) {
                inner_vector.resize(inner_size); // 设置每个内层向量的大小
            }
        }
    };

    struct FilterSettings
    {
        Eigen::VectorXd base_Q;
        Eigen::VectorXd base_R;
        Eigen::VectorXd com_Q;
        Eigen::VectorXd com_R;
        Eigen::VectorXd joint_vel_Q;
        Eigen::VectorXd joint_vel_R;
        Eigen::VectorXd joint_pos_Q;
        Eigen::VectorXd joint_pos_R;
        double l_joint_defor_K;
        double r_joint_defor_K;
    };
    struct KuavoSettings
    {

        // params
        PidParams pid_params;
        PredefinedArmPose predefined_arm_pose;
        ModelSettings model_settings;
        RunningSettings running_settings;
        HardwareSettings hardware_settings;
        MotorC2TSettings motor_c2t_settings;
        FilterSettings filter_settings;
        std::string kuavo_assets_path;

        // functions
        void printInfo();
        void loadKuavoSettings(JSONConfigReader &robot_config);

    private:
        void loadPidParams(JSONConfigReader &robot_config);
        void loadPredefinedArmPose(JSONConfigReader &robot_config);
        void loadModelSettings(JSONConfigReader &robot_config);
        void loadRunningSettings(JSONConfigReader &robot_config);
        void loadHardwareSettings(JSONConfigReader &robot_config);
        void loadFilterSettings(JSONConfigReader &robot_config);
    };
}
