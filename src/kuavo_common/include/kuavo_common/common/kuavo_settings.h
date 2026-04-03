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

#define AK10_9_MC (40)
#define AK70_10_MC (26.1) // 手册是 23.2
#define PA81_MC (60)
#define PA100_MC (70)
#define PA76_25_MC (18)
#define PA81_18_25_MC (18)
#define CK_MC (18)
#define PA4310_25_MC (8)
#define PA72_36_MC (15)
#define PA76_18_MC (31.5)
#define PA105_18_MC (70)
#define PA115_MC (40)
#define PA81_25_MC (40)
#define PA4315_36_MC (22.5)


#define AK10_9_C2T (1.26)
#define AK70_10_C2T (1.23)
#define PA81_C2T (1.25)
#define PA100_C2T (1.2) // 1.2
#define PA100_18_C2T (2.0)
#define PA100_20_C2T (2.4)
#define PA81_25_C2T (2.9)


#define CK_C2T (2.1) // 1.4
#define PA72_C2T (3.6)
#define PA60_C2T (2.0)
#define PA43_C2T (4.7)

#define PA76_25_C2T (4.2)
#define PA76_18_C2T (2.0)
#define PA105_18_C2T (4.1)
#define PA72_36_C2T (4.8)
#define PA81_18_25_C2T (2.9)
#define PA115_C2T (11.5)
#define PA60_16_C2T (2.0)
#define PA4310_25_C2T (4.7)
#define PA4315_36_C2T (4.7)

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
        std::vector<int32_t> ruiwo_kp;
        std::vector<int32_t> ruiwo_kd;
        std::vector<int32_t> vr_joint_kp;
        std::vector<int32_t> vr_joint_kd;
        std::vector<int32_t> vr_ruiwo_kp;
        std::vector<int32_t> vr_ruiwo_kd;
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
