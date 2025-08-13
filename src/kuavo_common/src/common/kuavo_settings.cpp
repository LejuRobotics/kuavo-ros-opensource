#include "kuavo_common/common/kuavo_settings.h"
#include "kuavo_common/common/utils.h"

namespace HighlyDynamic
{
    struct motor_config
    {
        uint32_t encoder_range;
        double max_current;
        double c2t_coeff;
        MotorDriveType driver;
    };
    void KuavoSettings::printInfo()
    {
        std::cout << ">>>>>>>>>>>>>>>>>>>>>>>>>>> KuavoSettings INFO <<<<<<<<<<<<<<<<<<<<<<<<<<<\n";
    }
    void KuavoSettings::loadKuavoSettings(JSONConfigReader &robot_config)
    {
        loadModelSettings(robot_config);
        loadRunningSettings(robot_config);
        loadHardwareSettings(robot_config);
        loadFilterSettings(robot_config);
        loadPidParams(robot_config);
        loadPredefinedArmPose(robot_config);
    }
    void KuavoSettings::loadPidParams(JSONConfigReader &robot_config)
    {
        pid_params.pos.kp = robot_config.getValue<Eigen::VectorXd>("pos_kp");
        pid_params.pos.ki = robot_config.getValue<Eigen::VectorXd>("pos_ki");
        pid_params.pos.kd = robot_config.getValue<Eigen::VectorXd>("pos_kd");

        pid_params.pos_sim.ki = robot_config.getValue<Eigen::VectorXd>("pos_ki_sim");
        pid_params.pos_sim.kp = robot_config.getValue<Eigen::VectorXd>("pos_kp_sim");
        pid_params.pos_sim.kd = robot_config.getValue<Eigen::VectorXd>("pos_kd_sim");

        pid_params.arm.kp = robot_config.getValue<Eigen::VectorXd>("arm_kp");
        pid_params.arm.ki = robot_config.getValue<Eigen::VectorXd>("arm_ki");
        pid_params.arm.kd = robot_config.getValue<Eigen::VectorXd>("arm_kd");

        pid_params.pbc.kp = robot_config.getValue<Eigen::VectorXd>("pbc_kp");
        pid_params.pbc.ki = Eigen::VectorXd::Zero(pid_params.pbc.kp.size());
        pid_params.pbc.kd = robot_config.getValue<Eigen::VectorXd>("pbc_kd");
    }

    void KuavoSettings::loadPredefinedArmPose(JSONConfigReader &robot_config)
    {
        predefined_arm_pose.init_arm_pos = robot_config.getValue<Eigen::VectorXd>("init_arm_pos");
        predefined_arm_pose.walk_arm_pose = robot_config.getValue<Eigen::VectorXd>("walk_arm_pose");
        predefined_arm_pose.arm_poses.push_back(robot_config.getValue<Eigen::VectorXd>("arm_pose0"));
        predefined_arm_pose.arm_poses.push_back(robot_config.getValue<Eigen::VectorXd>("arm_pose1"));
    }

    void KuavoSettings::loadModelSettings(JSONConfigReader &robot_config)
    {
        std::string path_prefix = kuavo_assets_path + "/models/";
        model_settings.model_path = path_prefix + robot_config.getValue<std::string>("model_path");
        model_settings.model_with_arm_path = path_prefix + robot_config.getValue<std::string>("model_with_arm_path");
        model_settings.left_leg_urdf = path_prefix + robot_config.getValue<std::string>("Lleg_urdf");
        model_settings.right_leg_urdf = path_prefix + robot_config.getValue<std::string>("Rleg_urdf");
        model_settings.left_arm_urdf = path_prefix + robot_config.getValue<std::string>("Larm_urdf");
        model_settings.right_arm_urdf = path_prefix + robot_config.getValue<std::string>("Rarm_urdf");
        model_settings.arm_urdf = path_prefix + robot_config.getValue<std::string>("arm_urdf");

        model_settings.ankle_motor_offset_degree = robot_config.getValue<Eigen::VectorXd>("ankle_motor_offset_degree");
        model_settings.arm_ankle_motor_offset_degree = robot_config.getValue<Eigen::VectorXd>("arm_ankle_motor_offset_degree");
        model_settings.NUM_JOINT = robot_config.getValue<uint8_t>("NUM_JOINT");
        model_settings.NUM_ARM_JOINT = robot_config.getValue<uint8_t>("NUM_ARM_JOINT");
        model_settings.NUM_HEAD_JOINT = robot_config.getValue<uint8_t>("NUM_HEAD_JOINT");
        model_settings.is_parallel_arm = robot_config.getValue<bool>("isParallelArm");

        model_settings.end_frames_name = robot_config.getValue<std::vector<std::string>>("end_frames_name");
        model_settings.contact_frames_name = robot_config.getValue<std::vector<std::string>>("contact_frames_name");
        model_settings.left_foot_ankle_link_joint_frames = robot_config.getValue<std::vector<std::string>>("left_foot_ankle_link_joint_frames");
        model_settings.right_foot_ankle_link_joint_frames = robot_config.getValue<std::vector<std::string>>("right_foot_ankle_link_joint_frames");
        model_settings.left_arm_ankle_link_joint_frames = robot_config.getValue<std::vector<std::string>>("left_arm_ankle_link_joint_frames");
        model_settings.right_arm_ankle_link_joint_frames = robot_config.getValue<std::vector<std::string>>("right_arm_ankle_link_joint_frames");
    }

    void KuavoSettings::loadRunningSettings(JSONConfigReader &robot_config)
    {
        running_settings.torso_pitch = robot_config.getValue<double>("torsoP");
        running_settings.torso_yaw = robot_config.getValue<double>("torsoY");
        running_settings.step_height = robot_config.getValue<double>("StepH");
        running_settings.com_z = robot_config.getValue<double>("com_z");
        running_settings.com_z_jump = robot_config.getValue<double>("com_z_jump");
        running_settings.step_with = robot_config.getValue<double>("StepWith");
        running_settings.step_duration = robot_config.getValue<double>("StepDuration");
        running_settings.velocity_limit = robot_config.getValue<Eigen::VectorXd>("VelocityLimit");
        running_settings.cmd_vel_step = robot_config.getValue<Eigen::VectorXd>("cmd_vel_step");
        running_settings.walk_stablizer_count = robot_config.getValue<uint8_t>("walk_stablizer_count");
        running_settings.walk_stablizer_threshold = robot_config.getValue<double>("walk_stablizer_threshold");
        running_settings.v_takeoff = robot_config.getValue<double>("V_takeoff");
        running_settings.swing_arm = robot_config.getValue<bool>("swing_arm");
        running_settings.only_half_up_body = robot_config.getValue<bool>("only_half_up_body");
        running_settings.joint_kp = robot_config.getValue<std::vector<int32_t>>("joint_kp");
        running_settings.joint_kd = robot_config.getValue<std::vector<int32_t>>("joint_kd");
    }

    std::string HardwareSettings::getEcmasterType(RobotVersion rb_version) {

        std::string filePath = getUserHomeDirectory() + "/.config/lejuconfig/EcMasterType.ini";
        std::ifstream file(filePath);
        std::string ecMasterType;

        if (!file.is_open()) {
            std::cerr << "\033[33mwarning: " << filePath << " 文件不存在, 未指定EcMasterType, 使用默认值 'elmo' 驱动器类型\033[0m" << std::endl;
            return "elmo";  
        }

        getline(file, ecMasterType);
        file.close();

        if (ecMasterType != "elmo" && ecMasterType != "youda") {
            std::cerr << "\033[33mwarning: ecmaster_type :" << ecMasterType 
                  << " error, 使用默认值 'elmo' 驱动器类型\033[0m" << std::endl;
            return "elmo"; 
        }

        return ecMasterType;
    }
    std::string HardwareSettings::getIMUType(RobotVersion rb_version) {

        std::string filePath = getUserHomeDirectory() + "/.config/lejuconfig/ImuType.ini";
        std::ifstream file(filePath);
        std::string imuType;

        if (!file.is_open()) {
            std::cerr << "\033[33mwarning: " << filePath << " 文件不存在, 未指定imuType, 使用默认值 'xsens' IMU类型\033[0m" << std::endl;
            return "xsens";  
        }

        getline(file, imuType);
        file.close();

        if (imuType != "xsens" && imuType != "hipnuc") {
            std::cerr << "\033[33mwarning: ecmaster_type :" << imuType 
                  << " error, 使用默认值 'xsens' IMU类型\033[0m" << std::endl;
            return "xsens"; 
        }

        return imuType;
    }

    void KuavoSettings::loadHardwareSettings(JSONConfigReader &robot_config)
    {
        std::map<std::string, motor_config>
            motor_name_map = {
                {"PA100", {BIT_17_10, PA100_MC, PA100_C2T, EC_MASTER}},
                {"PA81", {BIT_17_10, PA81_MC, PA81_C2T, EC_MASTER}},
                {"AK10_9", {BIT_17_9, AK10_9_MC, AK10_9_C2T, EC_MASTER}},
                {"CK", {BIT_17_36, CK_MC, CK_C2T, EC_MASTER}},
                {"dynamixel", {BIT_17_36, CK_MC, CK_C2T, DYNAMIXEL}},
                {"realman", {BIT_17_36, CK_MC, CK_C2T, REALMAN}},
                {"ruiwo", {BIT_17_36, CK_MC, CK_C2T, RUIWO}},
                {"ruiwoPA72", {BIT_17_36, CK_MC, PA72_C2T, RUIWO}},
                {"ruiwoPA60", {BIT_17_36, CK_MC, PA60_C2T, RUIWO}},
                {"ruiwoPA43", {BIT_17_10, CK_MC, PA43_C2T, RUIWO}},
                {"PA100_18", {BIT_17_18, PA100_MC, PA100_18_C2T, EC_MASTER}},
                {"PA100_20", {BIT_17_20, PA100_MC, PA100_20_C2T, EC_MASTER}}};
        hardware_settings.num_joints = robot_config.getValue<uint8_t>("NUM_JOINT");
        hardware_settings.num_arm_joints = robot_config.getValue<uint8_t>("NUM_ARM_JOINT");
        hardware_settings.num_head_joints = robot_config.getValue<uint8_t>("NUM_HEAD_JOINT");
        hardware_settings.imu_invert = robot_config.getValue<bool>("imu_invert");
        hardware_settings.imu_in_torso = robot_config.getValue<Eigen::VectorXd>("imu_in_torso");

        hardware_settings.peak_timeWin = robot_config.getValue<double>("peak_protection");
        hardware_settings.speed_timeWin = robot_config.getValue<double>("speed_protection");
        hardware_settings.lock_rotor_timeWin = robot_config.getValue<double>("locked_rotor_protection");

        hardware_settings.resizeMotor(hardware_settings.num_joints);
        std::vector<std::string> MOTORS_TYPE = robot_config.getValue<std::vector<std::string>>("MOTORS_TYPE");
        std::vector<double> min_limits = robot_config.getValue<std::vector<double>>("min_joint_position_limits");
        std::vector<double> max_limits = robot_config.getValue<std::vector<double>>("max_joint_position_limits");
        std::vector<double> vel_limits = robot_config.getValue<std::vector<double>>("joint_velocity_limits");
        std::vector<double> peak_vel_limits = robot_config.getValue<std::vector<double>>("joint_peak_velocity_limits");
        std::vector<double> lock_rotor_limits = robot_config.getValue<std::vector<double>>("joint_torque_limits");
        std::vector<double> peak_torque_limits = robot_config.getValue<std::vector<double>>("joint_peak_torque_limits");


        auto endsWith = [](std::string& s, const std::string& suffix) -> bool {
            if (suffix.size() > s.size()) return false;
            return std::equal(suffix.rbegin(), suffix.rend(), s.rbegin());
        };
        
        auto removeSuffix = [](std::string& s, const std::string& suffix) -> std::string& {
            if (s.size() >= suffix.size() && s.compare(s.size() - suffix.size(), suffix.size(), suffix) == 0) {
                s.erase(s.size() - suffix.size());
            }
            return s;
        };

        for (uint8_t i = 0; i < hardware_settings.num_joints; i++)
        {
            // mark motor exist or disable!
            std::string motor_type_name = MOTORS_TYPE[i];

            hardware_settings.motors_exist[i] = true;
            hardware_settings.motors_disable[i] = false;
            if (endsWith(motor_type_name, "_none")) {
                hardware_settings.motors_exist[i] = false;
                motor_type_name = removeSuffix(motor_type_name, "_none");
            }
            else if (endsWith(motor_type_name, "_disable")) {
                hardware_settings.motors_exist[i] = true;
                hardware_settings.motors_disable[i] = true;
                motor_type_name = removeSuffix(motor_type_name, "_disable");
            }

            // check motor type
            if (motor_name_map.find(motor_type_name) == motor_name_map.end()) {
                std::cerr << "\033[31m\nERROR: `" << motor_type_name << "` motor type not found, \nPlease check config file:`src/kuavo_assets/config/kuavo_v$ROBOT_VERSION/kuavo.json`\n\033[0m" << std::endl;
                exit(-1);
            }

            motor_config motor = motor_name_map[motor_type_name];
            hardware_settings.joint_ids[i] = i + 1;
            hardware_settings.motors_type[i] = motor_type_name;
            hardware_settings.driver[i] = motor.driver;
            hardware_settings.encoder_range[i] = motor.encoder_range;
            hardware_settings.max_current[i] = motor.max_current;
            hardware_settings.c2t_coeff[i] = motor.c2t_coeff;
            hardware_settings.min_joint_position_limits[i] = min_limits[i];
            hardware_settings.max_joint_position_limits[i] = max_limits[i];
            hardware_settings.joint_velocity_limits[i] = vel_limits[i];
            hardware_settings.joint_peak_velocity_limits[i] = peak_vel_limits[i];
            hardware_settings.joint_lock_rotor_limits[i] = lock_rotor_limits[i];
             hardware_settings.joint_peak_limits[i] = peak_torque_limits[i];
        }

        std::vector<std::string> end_effector_type = robot_config.getValue<std::vector<std::string>>("EndEffectorType");
        std::map<std::string, EndEffectorType> end_effector_type_map = {{"none", EndEffectorType::none},
                                                                        {"jodell", EndEffectorType::jodell},
                                                                        {"qiangnao", EndEffectorType::qiangnao},
                                                                        {"lejuclaw", EndEffectorType::lejuclaw},
                                                                        {"qiangnao_touch", EndEffectorType::qiangnao_touch}};
        for (auto &name : end_effector_type)
        {
            // std::cout << "EndEffectorType: " << name << std::endl;
            hardware_settings.end_effector_type.push_back(end_effector_type_map[name]);
        }
    }
    void KuavoSettings::loadFilterSettings(JSONConfigReader &robot_config)
    {
        filter_settings.base_Q = robot_config.getValue<Eigen::VectorXd>("base_Q");
        filter_settings.base_R = robot_config.getValue<Eigen::VectorXd>("base_R");
        filter_settings.com_Q = robot_config.getValue<Eigen::VectorXd>("com_Q");
        filter_settings.com_R = robot_config.getValue<Eigen::VectorXd>("com_R");
        auto joint_vel_Q = robot_config.getValue<Eigen::VectorXd>("joint_vel_Q");
        filter_settings.joint_vel_Q.resize(joint_vel_Q.size() * 2);
        filter_settings.joint_vel_Q << joint_vel_Q, joint_vel_Q;

        auto joint_vel_R = robot_config.getValue<Eigen::VectorXd>("joint_vel_R");
        filter_settings.joint_vel_R.resize(joint_vel_R.size() * 2);
        filter_settings.joint_vel_R << joint_vel_R, joint_vel_R;
        auto joint_pos_Q = robot_config.getValue<Eigen::VectorXd>("joint_pos_Q");
        filter_settings.joint_pos_Q.resize(joint_pos_Q.size() * 2);
        filter_settings.joint_pos_Q << joint_pos_Q, joint_pos_Q;
        auto joint_pos_R = robot_config.getValue<Eigen::VectorXd>("joint_pos_R");
        filter_settings.joint_pos_R.resize(joint_pos_R.size() * 2);
        filter_settings.joint_pos_R << joint_pos_R, joint_pos_R;

        filter_settings.l_joint_defor_K = robot_config.getValue<double>("l_joint_defor_K");
        filter_settings.r_joint_defor_K = robot_config.getValue<double>("r_joint_defor_K");
    }

}
