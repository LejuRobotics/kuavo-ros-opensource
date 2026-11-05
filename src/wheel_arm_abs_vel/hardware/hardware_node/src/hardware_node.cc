#include <poll.h>
#include <unistd.h>
#include <cerrno>
#include "hardware_node.h"
#include "hardware_node/ActuatorDynamics.hpp"
#include "kuavo_assets/include/package_path.h"
#include "motorevo/motorevo_actuator.h"

#include <set>
#include <algorithm>
#include <chrono>
#include <functional>
#include <limits>
#include <iomanip>
#include <fstream>
#include <sstream>
#include <yaml-cpp/yaml.h>
#include <pwd.h>
#include <cmath>

namespace {

constexpr char kHardwareStateParam[] = "/nodelet_manager/hardware_state";
constexpr int kHardwareRunning = 0;
constexpr int kHardwareDeinitializing = 1;
constexpr int kHardwareDeinitialized = 2;

std::string getConfigRootPath() {
    const char* sudo_user = getenv("SUDO_USER");
    passwd* pw = nullptr;
    if (sudo_user != nullptr) {
        pw = getpwnam(sudo_user);
    } else {
        pw = getpwuid(getuid());
    }
    if (pw != nullptr) {
        return std::string(pw->pw_dir) + "/.config/lejuconfig";
    }
    return "";
}

std::vector<double> readArmsZeroFromFile() {
    std::vector<double> result;
    std::string path = getConfigRootPath() + "/arms_zero.yaml";
    try {
        YAML::Node config = YAML::LoadFile(path);
        if (config["arms_zero_position"]) {
            result = config["arms_zero_position"].as<std::vector<double>>();
        }
    } catch (const std::exception& e) {
        ROS_ERROR("Failed to read arms_zero.yaml: %s", e.what());
    }
    return result;
}

std::vector<double> readOffsetFromFile(size_t count) {
    std::vector<double> result;
    std::string path = getConfigRootPath() + "/offset.csv";
    std::ifstream file(path);
    if (!file.is_open()) {
        ROS_ERROR("Failed to open offset.csv: %s", path.c_str());
        return result;
    }
    std::string line;
    while (std::getline(file, line) && result.size() < count) {
        std::istringstream iss(line);
        double val;
        if (iss >> val) {
            result.push_back(val);
        }
    }
    return result;
}

}  // namespace

namespace HighlyDynamic
{
    void HardwareNode::applyArmActuatorDynamicsCompensation(const kuavo_msgs::jointCmd::ConstPtr &msg, Eigen::VectorXd &cmd)
    {
        constexpr int kArmCompensationDof = 14;
        if (!actuatorDynamicsCompensatorPtr_ || armJointIndices_.size() != kArmCompensationDof)
        {
            return;
        }

        Eigen::VectorXd tauCmd = Eigen::VectorXd::Zero(kArmCompensationDof);
        Eigen::VectorXd dqCmd = Eigen::VectorXd::Zero(kArmCompensationDof);
        Eigen::VectorXd dqMeas = Eigen::VectorXd::Zero(kArmCompensationDof);
        const Eigen::VectorXd ddq = Eigen::VectorXd::Zero(kArmCompensationDof);

        {
            std::lock_guard<std::mutex> lk(enable_mpc_mtx_);
            if (curVel_.size() < static_cast<size_t>(num_joint))
            {
                return;
            }
            for (int i = 0; i < kArmCompensationDof; ++i)
            {
                const int jointIndex = armJointIndices_[i];
                tauCmd[i] = msg->tau.at(jointIndex);
                dqCmd[i] = msg->joint_v.at(jointIndex);
                dqMeas[i] = curVel_[jointIndex];
            }
        }

        const Eigen::VectorXd compensatedTau = actuatorDynamicsCompensatorPtr_->compute(tauCmd, ddq, dqCmd, dqMeas);
        if (compensatedTau.size() != kArmCompensationDof)
        {
            return;
        }

        for (int i = 0; i < kArmCompensationDof; ++i)
        {
            const int jointIndex = armJointIndices_[i];
            cmd[num_joint * 2 + jointIndex] = compensatedTau[i];
        }
    }

    void HardwareNode::jointMoveToPrepGoal(const std::vector<double>& goal_pos, int speed_index)
    {
        std::vector<double> prep_ec_kp;
        std::vector<double> prep_ec_kd;
        if (nh_.getParam("/hardware_prep_ec_joint_kp", prep_ec_kp) &&
            nh_.getParam("/hardware_prep_ec_joint_kd", prep_ec_kd) &&
            prep_ec_kp.size() == prep_ec_kd.size() && prep_ec_kp.size() == 12)
        {
            hardware_plant_->setPrepEcLegGains(prep_ec_kp, prep_ec_kd);
        }

        std::vector<double> prep_moves;
        if (!nh_.getParam("/hardware_prep_moves", prep_moves) || prep_moves.empty() ||
            speed_index < 0 || speed_index >= static_cast<int>(prep_moves.size()) ||
            hardware_param_.cali || hardware_plant_->is_cali_set_zero_)
        {
            jointMoveTo(goal_pos, jointMoveSpeed_);
            hardware_plant_->clearPrepEcLegGains();
            return;
        }

        const double speed = prep_moves[static_cast<size_t>(speed_index)];
        jointMoveTo(goal_pos, speed, dt_);
        hardware_plant_->clearPrepEcLegGains();
    }

    void HardwareNode::zeroTorqueCalibrateLegs()
    {
        std::cout << "\033[32m是否执行0扭矩控制EC关节?(y/n): \033[0m" << std::endl;
        char zero_torque_choice = '\0';
        while (ros::ok() && !shutdownRequested() && (zero_torque_choice != 'y' && zero_torque_choice != 'n'))
        {
            if (kbhit())
            {
                zero_torque_choice = getchar();
                if (zero_torque_choice == '\n')
                {
                    continue; // 忽略回车键
                }
            }
            usleep(100000);
        }

        if (shutdownRequested() || !ros::ok()) return;
        if (zero_torque_choice == 'y')
        {
            std::cout << "\033[32m进入0扭矩模式，现在可以手动移动关节...\033[0m" << std::endl;
            std::cout << "\033[32m操作完成后，输入 'x' 退出0扭矩模式并继续执行后续代码\033[0m" << std::endl;

            // 进入0扭矩模式
            if (hardware_plant_->setZeroTorqueForLegECMotors())
            {
                std::cout << "\033[32m0扭矩模式已启动\033[0m" << std::endl;

                // 等待用户操作完成并退出0扭矩模式
                char exit_zero_torque = '\0';
                while (ros::ok() && !shutdownRequested() && (exit_zero_torque != 'x'))
                {
                    // 持续发送0扭矩命令，保持0扭矩状态
                    hardware_plant_->setZeroTorqueForLegECMotors();

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

                if (shutdownRequested() || !ros::ok()) return;
                // 退出0扭矩模式，恢复正常控制
                if (hardware_plant_->exitZeroTorqueMode())
                {
                    std::cout << "\033[32m0扭矩模式已退出，恢复正常控制\033[0m" << std::endl;
                }
                else
                {
                    std::cout << "\033[31m退出0扭矩模式失败\033[0m" << std::endl;
                }

                std::cout << "\033[32m退出0扭矩模式，继续执行后续代码...\033[0m" << std::endl;
                std::cout << "\033[32m校准腿部电机编码器偏移...\033[0m" << std::endl;
                hardware_plant_->setCurrentPositionAsOffset();
            }
            else
            {
                std::cout << "\033[31m0扭矩模式启动失败\033[0m" << std::endl;
            }
        }
        else
        {
            std::cout << "\033[32m跳过0扭矩控制EC关节\033[0m" << std::endl;
        }
    }

    void HardwareNode::real_init_wait()
    {
        std::lock_guard<std::mutex> lifecycle_lock(hardware_lifecycle_mutex_);
        if (shutdownRequested() || !ros::ok()) return;
        
        ros::param::set("/hardware/is_ready", 0);
        ros::param::set("/hardware/ready_to_start", 0);
        std::cout << "[HardwareNode]正在等待 cppad 构建完成..." << std::endl;
        int build_cppad_status_ = 0;
        while (ros::ok() && !shutdownRequested() && (build_cppad_status_ != 2))
        {
            if (nh_.hasParam("build_cppad_state"))
            {
                nh_.getParam("build_cppad_state", build_cppad_status_);
            }
            usleep(100000);
        }

        if (shutdownRequested() || !ros::ok()) return;
        auto real_initial_start_service_ = nh_.advertiseService("/humanoid_controller/real_initial_start", &HardwareNode::realIntialStartCallback, this);
        
        std::string robot_module;
        robot_module = hardware_plant_->getRobotModule();
        if (hardware_param_.cali_leg)
        {
            bool cali_leg = false;
            nh_.getParam("cali_leg", cali_leg);
            if (cali_leg)
            {
                hardware_plant_->hardware_status_ = -2;
                if(isLunbiModule(robot_module))
                {
                    zeroTorqueCalibrateLegs();
                }
                else {
                    std::cout << "\033[32m校准腿部电机编码器偏移...\033[0m" << std::endl;
                    hardware_plant_->setCurrentPositionAsOffset();
                }

                std::cout << "\033[32m按'o'继续使用, 按'q'退出...\033[0m" << std::endl;
                while (ros::ok() && !shutdownRequested())
                {
                    if (kbhit())
                    {
                        hardware_plant_->initial_input_cmd_ = '\0';
                        hardware_plant_->initial_input_cmd_ = getchar();
                    }
                    if (hardware_plant_->initial_input_cmd_ == 'o')
                    {
                        hardware_plant_->initial_input_cmd_ = '\0';
                        break;
                    }
                    else if (hardware_plant_->initial_input_cmd_ == 'q')
                    {
                        std::cerr << "exit..." << std::endl;
                        requestShutdown();
                        ros::shutdown();
                        return;
                    }
                    usleep(100000);
                }
            }
        }
        
        std::vector<double> default_joint_pos;
        std::vector<double> initial_state_vector;
        std::vector<double> squat_state_vector;
        while (ros::ok() && !shutdownRequested() && (!nh_.getParam("/initial_state", initial_state_vector))) {
            ROS_INFO("Waiting for 'initial_state' parameter to be set...");
            ros::WallDuration(0.2).sleep();
        }
        Eigen::VectorXd initial_state_(initial_state_vector.size());
        for (size_t i = 0; i < initial_state_vector.size(); ++i)
        {
            initial_state_(i) = initial_state_vector[i];
        }

        while (ros::ok() && !shutdownRequested() && (!nh_.getParam("/squat_initial_state", squat_state_vector)))
        {
            ROS_INFO("Waiting for 'squat_initial_state' parameter to be set...");
            ros::WallDuration(0.2).sleep(); // 等待1秒后再次尝试
        }
        Eigen::VectorXd squat_initial_state_(squat_state_vector.size());
        for (size_t i = 0; i < squat_state_vector.size(); ++i)
        {
            squat_initial_state_(i) = squat_state_vector[i];
        }

        while (ros::ok() && !shutdownRequested() && (!nh_.getParam("/default_joint_pos", default_joint_pos))) {
            ROS_INFO("Waiting for 'default_joint_pos' parameter to be set...");
            ros::WallDuration(0.2).sleep();
        }

        // use_sit_init：必须等 HC configureLaunchBoot 写完 /hardware_prep_*。
        // 否则 build_cppad=2 后 HW 可能读到旧 squat_initial_state、错过 prep（首次像“参数没加载”）。
        bool use_sit_init = false;
        nh_.param("/use_sit_init", use_sit_init, false);
        if (use_sit_init) {
            bool prep_ready = false;
            std::cout << "[HardwareNode] /use_sit_init=true: waiting for /hardware_sit_prep_ready ..."
                      << std::endl;
            while (ros::ok() && !shutdownRequested()) {
                if (nh_.getParam("/hardware_sit_prep_ready", prep_ready) && prep_ready) {
                    std::cout << "[HardwareNode] /hardware_sit_prep_ready=true, continue sit prep."
                              << std::endl;
                    break;
                }
                ROS_INFO_THROTTLE(2.0, "[HardwareNode] Waiting for /hardware_sit_prep_ready ...");
                ros::WallDuration(0.1).sleep();
            }
            if (!ros::ok() || shutdownRequested()) {
                return;
            }
        }

        if (shutdownRequested() || !ros::ok()) return;
        hardware_plant_->setDefaultJointPos(default_joint_pos);

        int arm_joints_num =  hardware_plant_->num_arm_joints;
        int head_joints_num = hardware_plant_->num_head_joints;
        int waist_joints_num = hardware_plant_->num_waist_joints;
        int leg_joint_num = num_joint - arm_joints_num - head_joints_num - waist_joints_num;
        vector_t intial_state = initial_state_;
        vector_t squat_intial_state = squat_initial_state_;
        
        vector_t ready_joint_pos_ = vector_t::Zero(num_joint);
        vector_t squat_joint_pos_ = vector_t::Zero(num_joint);
        if(isLunbiModule(robot_module))
        {
            //轮臂默认初始化姿态一样，state的构成浮动基 7 + 全向轮 8 + 底盘下肢电机 4 + 双臂 7*2 + 头部 
            std::cout << "设置轮臂初始化状态."<<std::endl;
            ready_joint_pos_.head(leg_joint_num + arm_joints_num + head_joints_num) = intial_state.segment(7, leg_joint_num + arm_joints_num + head_joints_num) * 180 / M_PI;
            squat_joint_pos_.head(leg_joint_num + arm_joints_num + head_joints_num) = squat_intial_state.segment(7, leg_joint_num + arm_joints_num + head_joints_num) * 180 / M_PI;
        }
        else
        {
            std::cout << "设置人形初始化状态."<<std::endl;
            /********************站立姿态*********************/ 
            ready_joint_pos_.head(leg_joint_num) = intial_state.segment(12, leg_joint_num) * 180 / M_PI;
            /********************蹲下准备姿态*********************/
            squat_joint_pos_.head(leg_joint_num) = squat_intial_state.segment(12, leg_joint_num) * 180 / M_PI;
        }
        std::vector<double> prep_joint_pos_deg;
        if (nh_.getParam("/hardware_prep_joint_pos_deg", prep_joint_pos_deg) &&
            prep_joint_pos_deg.size() == static_cast<size_t>(num_joint))
        {
            // 与 hardware_plant 一致：腿 | 腰 | 臂 | 头
            for (int i = 0; i < leg_joint_num; ++i) {
                squat_joint_pos_[i] = prep_joint_pos_deg[static_cast<size_t>(i)];
            }
            for (int i = 0; i < waist_joints_num; ++i) {
                squat_joint_pos_[leg_joint_num + i] = prep_joint_pos_deg[static_cast<size_t>(leg_joint_num + i)];
            }
            const int arm_head_dof = arm_joints_num + head_joints_num;
            for (int i = 0; i < arm_head_dof; ++i) {
                const int idx = leg_joint_num + waist_joints_num + i;
                squat_joint_pos_[idx] = prep_joint_pos_deg[static_cast<size_t>(idx)];
            }
            std::cout << "[HardwareNode] prep pose from /hardware_prep_joint_pos_deg (leg|waist|arm|head)" << std::endl;
        }
        std::cout << "ready_joint_pos_:"<<ready_joint_pos_.transpose()<<std::endl;
        std::cout << "squat_joint_pos_:"<<squat_joint_pos_.transpose()<<std::endl;
        
        /***************手臂展开的校准姿态****************** */
        std::vector<double> ready_pos_set_zero(num_joint, 0);
        std::fill(ready_pos_set_zero.begin(), ready_pos_set_zero.end(), 0.0);
        ready_pos_set_zero[waist_joints_num + leg_joint_num + 1 ] = 90;
        ready_pos_set_zero[waist_joints_num + leg_joint_num + 1 + arm_joints_num / 2] = -90;
        /***********************************************/
        std::vector<double> moving_pos(hardware_plant_->num_joint, 0);
        std::vector<double> ready_inital_pos(hardware_plant_->num_joint, 0); // 进入反馈的初始位置
        std::vector<double> ready_pos(hardware_plant_->num_joint, 0); // 挂起的动作
        // squat/ready 布局为 leg|waist|arm|head，必须拷满 num_joint（旧代码用 leg+arm+head 会漏腰/头）
        for (int i = 0; i < num_joint; i++)
        {
            ready_inital_pos[i] = ready_joint_pos_[i];
            ready_pos[i] = squat_joint_pos_[i];
        }

        if (!hardware_param_.cali)
        {
            ready_to_feedback_ = true;
            moving_pos = ready_pos;
            std::cout << "移动到准备姿态 ..." << std::endl;
            // 座椅启动(use_sit_init)多段 prep 较慢：过早置 0 会一直上报 ready_stance（issue 4012），
            // 置位延迟到 prep 完成后的就绪段（ready_to_start=1 同处）再置 0
            if (!use_sit_init) {
                hardware_plant_->hardware_status_ = 0;
            }
        }
        else
        {

            std::fill(moving_pos.begin(), moving_pos.end(), 0.0);
            if(isLunbiModule(robot_module))
            {
                // 设置底盘下肢的初始位置（单位：度）
                moving_pos[0] = 14.27;    // 0.249 rad
                moving_pos[1] = -25.27;   // -0.441 rad
                moving_pos[2] = 11.80;    // 0.206 rad
                moving_pos[3] = 0.0;
            }
            
            std::cout << "移动到校准姿态 ..." << std::endl;
        }

        if(hardware_plant_->is_cali_set_zero_)
        {
            cali_zero_point_mode = true;
            moving_pos = ready_pos_set_zero;
            hardware_plant_->cali_set_zero_status = 1;
            hardware_plant_->hardware_status_ = -3;

            std::cout << "移动到手臂张开的校准姿态 ..." << std::endl;
        }

        bool prep_two_phase = false;
        std::vector<double> prep_offset_joint_pos_deg;
        if (nh_.getParam("/hardware_prep_two_phase", prep_two_phase) && prep_two_phase &&
            nh_.getParam("/hardware_prep_joint_pos_offset_deg", prep_offset_joint_pos_deg) &&
            prep_offset_joint_pos_deg.size() == static_cast<size_t>(num_joint))
        {
            bool reverse_leg_first = false;
            nh_.getParam("/hardware_prep_reverse_leg_first", reverse_leg_first);
            std::vector<double> prep_leg_first_deg;
            const bool has_leg_first = reverse_leg_first &&
                                       nh_.getParam("/hardware_prep_joint_pos_leg_first_deg", prep_leg_first_deg) &&
                                       prep_leg_first_deg.size() == static_cast<size_t>(num_joint);
            std::cout << "[HardwareNode] sit-init prep phase1: sit_joint_pos + seat_offset ..." << std::endl;
            jointMoveToPrepGoal(prep_offset_joint_pos_deg, 0);  // move_speed
            if (has_leg_first) {
                // 与座椅段0一致：腿/腰回 sit 用 move_speed，臂回 sit 用 settle_speed
                std::cout << "[HardwareNode] sit-init prep phase2: reverse offset legs/waist (move_speed) ..."
                          << std::endl;
                jointMoveToPrepGoal(prep_leg_first_deg, 0);
                std::cout << "[HardwareNode] sit-init prep phase3: reverse offset arms (settle_speed) ..."
                          << std::endl;
            } else {
                std::cout << "[HardwareNode] sit-init prep phase2: sit_joint_pos (reverse offset, settle_speed) ..."
                          << std::endl;
            }
            // 收尾回到 sit（含低头）；勿用 ready_pos/moving_pos 以免头维被置 0
            std::vector<double> prep_sit_deg;
            if (nh_.getParam("/hardware_prep_joint_pos_deg", prep_sit_deg) &&
                prep_sit_deg.size() == static_cast<size_t>(num_joint))
                jointMoveToPrepGoal(prep_sit_deg, 1);
            else
                jointMoveToPrepGoal(moving_pos, 1);
        }
        else
        {
            jointMoveToPrepGoal(moving_pos);
        }
        // 移动到校准姿态完成：HWPlantInit 将 hardware_status_ 初始化为 kStatusNotInAnyState(-3)，
        // jointMoveToPrepGoal 返回即表示腿已绷直到位，状态切换为 calibrate
        if (hardware_param_.cali && !hardware_plant_->is_cali_set_zero_) {
            hardware_plant_->hardware_status_ = -1;
        }
        std::ostringstream tips_oss;
        bool sit_init_auto_start = false;  // await_start_after_p5=false 时跳过等 'o' 循环
        if (hardware_param_.cali)
        {
            tips_oss << "\033[32m*******检查你的机器人的状态并进行校准*******"
                     << "\n1. 输入 'c' 校准腿部电机编码器偏移."
                     << "\n2. 输入 'v' 校准手臂电机."
                     << "\n3. 输入 'a' 自动限位校准手臂电机(通过限位检测)."
                     << "\n4. 输入 's' 自检手臂腿部电机"
                     << "\n5. 输入 'o' 移动到准备姿态，(准备姿态下)输入 'q' 返回校准姿态，或直接输入 'ctrl+c' 退出.."
                     << "\n6. 输入 'h' 再次显示此提示。\033[0m\n";
        }
        else
        {
            hardware_plant_->hardware_status_ = 0;
            ros::param::set("/hardware/ready_to_start", 1);
            bool await_start = true;
            nh_.param("/seat/await_start_after_p5", await_start, true);
            if (!await_start && use_sit_init) {
              // 实机起立仍需按 'o'（StandUpLoop 仅凭 'o' 置 is_ready），提示与普通蹲姿一致，避免误导（issue 3883）
              tips_oss << "\033[32m检查你的机器人的状态:\n1. 确认无误后, 扶住机器人, 输入 'o'（机器人将站起来进入反馈!):\033[0m";
              std::cout << tips_oss.str() << std::endl;
              if (!(hardware_param_.cali_arm && hardware_param_.robot_version.start_with(1))) {
                auto* ruiwo_actuator = hardware_plant_->getRuiwoActuator();
                if (ruiwo_actuator != nullptr) {
                  auto* motorevo_actuator = dynamic_cast<motorevo::MotorevoActuator*>(ruiwo_actuator);
                  if (motorevo_actuator != nullptr)
                    motorevo_actuator->applyZeroOffsetAdjustments();
                }
              }
              StandUpLoop();
              printf("feedback start!!! \r\n");
              // StandUpLoop 返回后跳过等 'o' 循环：设标志让下方 while(1) 不进入
              sit_init_auto_start = true;
            }
            tips_oss << "\033[32m检查你的机器人的状态:\n1. 确认无误后, 扶住机器人, 输入 'o'（机器人将站起来进入反馈!):\033[0m";
        }

        if (!sit_init_auto_start) {
        // 当cali_arm=true且机器人为1系列时，不打印常规提示信息
        if (!(hardware_param_.cali_arm && hardware_param_.robot_version.start_with(1))) {
            std::cout << tips_oss.str() << std::endl;
        }

        // 当cali_arm=true且机器人为1系列时，获取机器人手臂零点并等待用户确认
        if (hardware_param_.cali_arm && hardware_param_.robot_version.start_with(1)) {
            std::cout << "\n\033[33m" << std::string(60, '=') << "\033[0m" << std::endl;
            std::cout << "\033[33m======== 1系列机器人手臂零点校准 ========\033[0m" << std::endl;
            std::cout << "\033[33m" << std::string(60, '=') << "\033[0m" << std::endl;
            
            auto* ruiwo_actuator = hardware_plant_->getRuiwoActuator();
            if (ruiwo_actuator != nullptr) {
                // 获取机器人手臂零点
                std::vector<double> arm_zero_points = ruiwo_actuator->getMotorZeroPoints();
                std::cout << "\n\033[36m[步骤 1/3] 已获取机器人手臂零点:\033[0m" << std::endl;
                for (size_t i = 0; i < arm_zero_points.size(); ++i) {
                    std::cout << "  电机 " << i << ": " << std::fixed << std::setprecision(4) << arm_zero_points[i] 
                              << " rad (" << arm_zero_points[i] * 180.0 / M_PI << " deg)" << std::endl;
                }
      
                // // 清空输入缓冲区，避免残留的换行符影响后续输入
                // std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
                
                // 等待用户输入y应用零点
                std::string user_input;
                bool zero_applied = false;
                
                while (ros::ok() && !shutdownRequested() && (!zero_applied)) {

                    std::cout << "\n\033[32m" << std::string(60, '-') << "\033[0m" << std::endl;
                    std::cout << "\033[32m[重要提示]\033[0m" << std::endl;
                    std::cout << "\033[32m请确认已拔掉工装！\033[0m" << std::endl;
                    std::cout << "\033[32m确认无误后，输入 'y' 应用零点\033[0m" << std::endl;
                    std::cout << "\033[32m" << std::string(60, '-') << "\033[0m" << std::endl;

                    if (!readStartupLine(user_input)) return;
                    
                    // 去除首尾空格
                    if (!user_input.empty()) {
                        user_input.erase(0, user_input.find_first_not_of(" \t"));
                        user_input.erase(user_input.find_last_not_of(" \t") + 1);
                    }
                    
                    if (user_input == "y" || user_input == "Y") {
                        std::cout << "\n\033[33m[步骤 1/3] 正在应用零点...\033[0m" << std::endl;

                        // 保存零点到文件
                        ruiwo_actuator->saveZeroPosition();

                        // 应用零点偏移调整到运行时电机配置
                        auto* motorevo_actuator = dynamic_cast<motorevo::MotorevoActuator*>(ruiwo_actuator);
                        if (motorevo_actuator != nullptr) {
                            // std::cout << "[HardwareNode] 正在应用零点偏移调整到运行时配置..." << std::endl;
                            motorevo_actuator->applyZeroOffsetAdjustments();
                        }

                        std::cout << "\033[32m✓ [步骤 1/3] 零点已应用并保存成功！\033[0m" << std::endl;

                        // 应用零点后，根据 cali 参数决定是否缩腿
                        if (hardware_param_.cali) {
                            // cali=true：启动时是腿绷直，需要缩腿
                            std::cout << "\n\033[33m[步骤 2/3] 正在移动到准备姿态（缩腿）...\033[0m" << std::endl;
                            std::vector<double> squat_pos(num_joint, 0);
                            for (int i = 0; i < leg_joint_num + arm_joints_num + head_joints_num; i++) {
                                squat_pos[i] = squat_joint_pos_[i];
                            }
                            jointMoveTo(squat_pos, jointMoveSpeed_);
                            std::cout << "\033[32m✓ [步骤 2/3] 已移动到准备姿态\033[0m" << std::endl;
                        } else {
                            // cali=false：启动时已经缩腿，跳过
                            std::cout << "\n\033[32m✓ 机器人已在准备姿态，跳过移动\033[0m" << std::endl;
                        }

                        zero_applied = true;
                        ready_to_feedback_ = true;
                    } else {
                        std::cout << "\033[31m[提示] 无效输入，请按 'y' 应用零点\033[0m" << std::endl;
                    }
                }
                

                
                bool stand_requested = false;
                while (ros::ok() && !shutdownRequested() && (!stand_requested)) {
                    // 缩腿完成后，提示用户按下o进行站立
                    std::cout << "\n\033[32m" << std::string(60, '-') << "\033[0m" << std::endl;
                    std::cout << "\033[32m[步骤 3/3] 准备姿态完成！\033[0m" << std::endl;
                    std::cout << "\033[32m现在可以输入 'o' 让机器人进行站立\033[0m" << std::endl;
                    std::cout << "\033[32m" << std::string(60, '-') << "\033[0m" << std::endl;

                    if (!readStartupLine(user_input)) return;

                    // 去除首尾空格
                    if (!user_input.empty()) {
                        user_input.erase(0, user_input.find_first_not_of(" \t"));
                        user_input.erase(user_input.find_last_not_of(" \t") + 1);
                    }

                    if (user_input == "o" || user_input == "O") {
                        std::cout << "\n\033[33m[步骤 3/3] 准备进行站立...\033[0m" << std::endl;
                        // 设置输入命令，让主循环执行站立逻辑
                        hardware_plant_->initial_input_cmd_ = 'o';
                        // ready_to_feedback 已在按'y'后设置为true，主循环将直接站立
                        stand_requested = true;
                    } else {
                        std::cout << "\033[31m[提示] 无效输入，请按 'o' 进行站立\033[0m" << std::endl;
                    }
                }
                
                std::cout << "\033[32m" << std::string(60, '=') << "\033[0m" << std::endl;
                std::cout << "\033[32m======== 已准备站立，进入主循环 ========\033[0m" << std::endl;
                std::cout << "\033[32m" << std::string(60, '=') << "\033[0m" << std::endl;
            } else {
                std::cerr << "\033[31m[HardwareNode] 错误：无法获取RuiWoActuator实例\033[0m" << std::endl;
            }
        }

        while (ros::ok() && !shutdownRequested())
        {
            if (kbhit()) 
            {
                hardware_plant_->initial_input_cmd_ = '\0';
                hardware_plant_->initial_input_cmd_ = getchar();
            }
            if (hardware_plant_->initial_input_cmd_ == 'h' || hardware_plant_->initial_input_cmd_ == '\n' )
            {
                std::cout << tips_oss.str() << std::endl;
                hardware_plant_->initial_input_cmd_ = '\0';
            }
            if (hardware_plant_->initial_input_cmd_ == 'o')
            {
                stand_up_cmd_ = hardware_plant_->initial_input_cmd_;
                hardware_plant_->initial_input_cmd_ = '\0';
                if (!ready_to_feedback_)
                {
                    moving_pos = ready_pos;
                    std::cout << "移动到准备姿态..." << std::endl;
                    jointMoveTo(moving_pos, jointMoveSpeed_);
                    std::cout << "\033[32m输入 'q' 返回校准状态，或再次输入 'o' 机器人将站起来!\033[0m" << std::endl;
                    ready_to_feedback_ = true;
                    hardware_plant_->hardware_status_ = 0;
                    hardware_plant_->cali_set_zero_status = 0;
                    ros::param::set("/hardware/ready_to_start", 1);
                    continue;
                }
                // 应用零点偏移调整（在机器人开始站立前）
                // 注意：如果是在 cali_arm=true 且1系列的情况下，零点偏移调整已在按下 'y' 时应用，这里不再重复应用
                // 但对于其他情况（非1系列或非cali_arm模式），仍需要在这里应用
                if (!(hardware_param_.cali_arm && hardware_param_.robot_version.start_with(1))) {
                    auto* ruiwo_actuator = hardware_plant_->getRuiwoActuator();
                    if (ruiwo_actuator != nullptr) {
                        auto* motorevo_actuator = dynamic_cast<motorevo::MotorevoActuator*>(ruiwo_actuator);
                        if (motorevo_actuator != nullptr) {
                            std::cout << "[HardwareNode] Applying zero offset adjustments before stand-up..." << std::endl;
                            motorevo_actuator->applyZeroOffsetAdjustments();
                        }
                    }
                }
                std::string robot_module = hardware_plant_->getRobotModule();
                StandUpLoop();

                printf("feedback start!!! \r\n");
                break;
            }
            if (hardware_plant_->initial_input_cmd_ == 'c')
            { // calibrate the motors encoders offsets
                if(isLunbiModule(robot_module))
                {
                    hardware_plant_->calibrateWheelLoop();
                }
                else
                {
                    hardware_plant_->calibrateBipedLoop();
                }
                hardware_plant_->initial_input_cmd_ = '\0';
                std::cout << tips_oss.str() << std::endl;
                hardware_plant_->cali_set_zero_status = 0;
                continue;
            }
            else if (hardware_plant_->initial_input_cmd_ == 'v')
            {  // calibrate the motors encoders offsets of arm joints
                hardware_plant_->calibrateArmJoints();
                hardware_plant_->initial_input_cmd_ = '\0';
                std::cout << tips_oss.str() << std::endl;
                hardware_plant_->cali_set_zero_status = 0;
                continue;
            }
            else if (hardware_plant_->initial_input_cmd_ == 'a')
            {  // automatic calibration of arm joints using limit detection
                // 清空输入缓冲区，避免残留的换行符影响后续输入
                std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
                
                std::cout << "\033[33m=== 手臂和头部限位校准 ===\033[0m" << std::endl;
                std::cout << "请选择校准模式：" << std::endl;
                std::cout << "  1 - 自动校准（所有关节自动依次校准）[默认]" << std::endl;
                std::cout << "  2 - 逐个校准（每组关节需要手动确认）" << std::endl;
                std::cout << "请输入选项 ([1]/2): ";
                
                std::string mode_input;
                std::getline(std::cin, mode_input);
                
                // 默认选择1（自动校准）
                char mode_choice = '1';
                if (!mode_input.empty())
                {
                    mode_choice = mode_input[0];
                }
                
                bool auto_mode = (mode_choice == '1');
                
                if (mode_choice == '1' || mode_choice == '2')
                {
                    // 询问校准范围
                    std::cout << "\n\033[33m请选择校准范围：\033[0m" << std::endl;
                    std::cout << "\033[31m警告：请确认关节能运动到限位位置且无遮挡！\033[0m" << std::endl;
                    std::cout << "  1 - 仅校准手臂 [默认]" << std::endl;
                    std::cout << "  2 - 校准手臂和头部" << std::endl;
                    std::cout << "  3 - 仅校准头部" << std::endl;
                    if (isLunbiModule(robot_module))
                    {
                        std::cout << "  轮臂如果需要校准腿，请选择以下选项" << std::endl;
                        std::cout << "  4 - 校准手臂和腿" << std::endl;
                        std::cout << "  5 - 校准手臂、头部和腿" << std::endl;
                        std::cout << "  6 - 仅校准腿" << std::endl;
                        std::cout << "请输入选项 ([1]/2/3/4/5/6): ";
                    }
                    else
                    {
                        std::cout << "请输入选项 ([1]/2/3): ";
                    }
                    
                    std::string range_input;
                    std::getline(std::cin, range_input);
                    
                    // 默认仅校准手臂
                    char range_choice = '1';
                    if (!range_input.empty())
                    {
                        range_choice = range_input[0];
                    }
                    
                    bool calibrate_head = false;
                    bool head_only = false;
                    bool calibrate_leg = false;
                    bool leg_only = false;
                    
                    if (range_choice == '2')
                    {
                        calibrate_head = true;
                        head_only = false;
                        calibrate_leg = false;
                        leg_only = false;
                        std::cout << (auto_mode ? "\033[33m开始自动校准模式（手臂+头部）...\033[0m" : "\033[33m开始逐个校准模式（手臂+头部）...\033[0m") << std::endl;
                    }
                    else if (range_choice == '3')
                    {
                        calibrate_head = true;
                        head_only = true;
                        calibrate_leg = false;
                        leg_only = false;
                        std::cout << (auto_mode ? "\033[33m开始自动校准模式（仅头部）...\033[0m" : "\033[33m开始逐个校准模式（仅头部）...\033[0m") << std::endl;
                    }
                    else if ((isLunbiModule(robot_module)) && range_choice == '4')
                    {
                        calibrate_head = false;
                        head_only = false;
                        calibrate_leg = true;
                        leg_only = false;
                        std::cout << (auto_mode ? "\033[33m开始自动校准模式（手臂+腿）...\033[0m" : "\033[33m开始逐个校准模式（手臂+腿）...\033[0m") << std::endl;
                    }
                    else if ((isLunbiModule(robot_module)) && range_choice == '5')
                    {
                        calibrate_head = true;
                        head_only = false;
                        calibrate_leg = true;
                        leg_only = false;
                        std::cout << (auto_mode ? "\033[33m开始自动校准模式（手臂+头部+腿）...\033[0m" : "\033[33m开始逐个校准模式（手臂+头部+腿）...\033[0m") << std::endl;
                    }
                    else if ((isLunbiModule(robot_module)) && range_choice == '6')
                    {
                        calibrate_head = false;
                        head_only = false;
                        calibrate_leg = true;
                        leg_only = true;
                        std::cout << (auto_mode ? "\033[33m开始自动校准模式（仅腿）...\033[0m" : "\033[33m开始逐个校准模式（仅腿）...\033[0m") << std::endl;
                    }
                    else  // 默认选项1
                    {
                        calibrate_head = false;
                        head_only = false;
                        calibrate_leg = false;
                        leg_only = false;
                        std::cout << (auto_mode ? "\033[33m开始自动校准模式（仅手臂）...\033[0m" : "\033[33m开始逐个校准模式（仅手臂）...\033[0m") << std::endl;
                    }
                    
                    bool calibration_success = hardware_plant_->calibrateArmJointsAtLimit(auto_mode, calibrate_head, head_only, calibrate_leg, leg_only);
                    
                    if (calibration_success)
                    {
                        std::cout << "\033[32m限位校准成功完成！\033[0m" << std::endl;
                    }
                    else
                    {
                        std::cout << "\033[31m限位校准失败或已取消。\033[0m" << std::endl;
                    }
                }
                else
                {
                    std::cout << "\033[31m无效的选项，取消校准。\033[0m" << std::endl;
                }
                
                hardware_plant_->initial_input_cmd_ = '\0';
                std::cout << tips_oss.str() << std::endl;
                hardware_plant_->cali_set_zero_status = 0;
                continue;
            }
   
            
            else if (hardware_plant_->initial_input_cmd_ == 's')
            {
                if (!hardware_param_.cali)
                {
                    hardware_plant_->initial_input_cmd_ = '\0';
                    continue;
                }
                // TODO: do some poses
                std::cout << "请观察各电机运动是否正常，对称..." << std::endl;
                hardware_plant_->performJointSymmetryCheck();
                hardware_plant_->initial_input_cmd_ = '\0';
                std::cout << tips_oss.str() << std::endl;
                hardware_plant_->cali_set_zero_status = 0;
                continue;
            }
            else if (hardware_plant_->initial_input_cmd_ == 'q')
            {
                hardware_plant_->initial_input_cmd_ = '\0';
                if (ready_to_feedback_)
                {
                    std::fill(moving_pos.begin(), moving_pos.end(), 0.0);
                    std::cout << "移动到校准姿态..." << std::endl;

                    if(hardware_plant_->is_cali_set_zero_){
                        moving_pos = ready_pos_set_zero;
                        hardware_plant_->cali_set_zero_status = 1;
                        
                        std::cout << "移动到手臂张开的校准姿态 ..." << std::endl;
                    }

                    jointMoveTo(moving_pos, jointMoveSpeed_);
                    std::cout << "\033[32m输入 'o' 进入准备状态\033[0m" << std::endl;
                    
                    ready_to_feedback_ = false;

                    if(hardware_plant_->is_cali_set_zero_){
                        hardware_plant_->hardware_status_ = -3;
                    }else{
                        hardware_plant_->hardware_status_ = -1;
                    }
                    continue;
                }

                
                requestShutdown();
                ros::shutdown();
                return;
            }
            else if (zero_point_status_swith && hardware_plant_->initial_input_cmd_ == 'z')
            {
                hardware_plant_->initial_input_cmd_ = '\0';
                zero_point_status_swith = false;

                std::fill(moving_pos.begin(), moving_pos.end(), 0.0);
                std::cout << "esc:移动到校准姿态..." << std::endl;

                moving_pos = ready_pos_set_zero;
                hardware_plant_->cali_set_zero_status = 1;
                std::cout << "移动到手臂张开的校准姿态 ..." << std::endl;
          

                jointMoveTo(moving_pos, jointMoveSpeed_);
                std::cout << "\033[32m输入 'o' 进入准备状态\033[0m" << std::endl;
                hardware_plant_->hardware_status_ = -3;

                ready_to_feedback_ = false;

                continue;
                    
            }else if (zero_point_status_swith && hardware_plant_->initial_input_cmd_ == 'e')
            {
                hardware_plant_->initial_input_cmd_ = '\0';
                zero_point_status_swith = false;
                std::fill(moving_pos.begin(), moving_pos.end(), 0.0);
                std::cout << "esc:移动到校准姿态..." << std::endl;
                jointMoveTo(moving_pos, jointMoveSpeed_);
                std::cout << "\033[32m输入 'o' 进入准备状态\033[0m" << std::endl;
                hardware_plant_->hardware_status_ = -3;
                ready_to_feedback_ = false;
                continue;
                 
                    
            }
            usleep(1000);
        }

        }  // !sit_init_auto_start

        if (shutdownRequested() || !ros::ok()) return;
        std::cout << "hardware init completed." << std::endl;
        hardware_plant_->hardware_ready_ = true;
        hardware_plant_->hardware_status_ = 1;
        hardware_plant_->cali_set_zero_status = 0;

        // 发布手臂零点偏置 - 直接读取 arms_zero.yaml
        {
            std::vector<double> arm_zeros = readArmsZeroFromFile();
            std_msgs::Float64MultiArray arm_zeros_msg;
            arm_zeros_msg.data = arm_zeros;
            arm_zeros_pub_.publish(arm_zeros_msg);
            ROS_INFO("[HardwareNode] Published /kuavo/arm_zeros with %zu values", arm_zeros.size());
        }

        // 发布腿部电机零点偏置 - 直接读取 offset.csv
        {
            size_t ec_count = hardware_plant_->getCountECMasters();
            std::vector<double> leg_offset = readOffsetFromFile(ec_count);
            leg_offset.resize(ec_count);
            std_msgs::Float64MultiArray leg_offset_msg;
            leg_offset_msg.data = leg_offset;
            leg_offset_pub_.publish(leg_offset_msg);
            ROS_INFO("[HardwareNode] Published /robot/offset with %zu values", leg_offset.size());
        }
    }

    void HardwareNode::StandUpLoop()
    {
        if (shutdownRequested() || !ros::ok()) return;
#if !defined(USE_DDS) && !defined(USE_LEJU_DDS)
        // Only subscribe to ROS joint commands when DDS is not used
        std::cout << "Register and subscribe to the topic jointCmd." << std::endl;
        joint_sub_ = nh_.subscribe("joint_cmd", 10, &HardwareNode::jointCmdCallback, this);
#endif
        std::string robot_module = hardware_plant_->getRobotModule();
        if(isLunbiModule(robot_module))
        {
            //轮臂不需要站立
            std::cout << "The wheel arm does not need to stand up." << std::endl;
            ros::param::set("/hardware/is_ready", 1);
            return;
        }

        int stand_up_complete{0};
        while (ros::ok() && !shutdownRequested() && (1 != stand_up_complete))
        {
            stand_up_complete = bot_stand_up_complete_;

            if (1 == stand_up_complete)
            {
                break;
            }
            else if (-1 == stand_up_complete)
            {
                bot_stand_up_complete_ = 0;
                hardware_plant_->hardware_status_ = 0;
                ros::param::set("/hardware/ready_to_start", 1);
                std::ostringstream tips_oss;
                tips_oss << "\033[32m检查你的机器人的状态:\n1. 确认无误后, 扶住机器人, 输入 'o'（机器人将站起来进入反馈!):\033[0m";
                std::cout << tips_oss.str() << std::endl;
            }
            else
            {
                if(kbhit())
                {
                    stand_up_cmd_ = getchar();
                }
                if( 'o' == stand_up_cmd_)
                {
                    stand_up_cmd_ = '\0';
                    ros::param::set("/hardware/is_ready", 1);
                }
            }
            usleep(2000);
        }
    }


    
    bool HardwareNode::deintialCallback(std_srvs::Trigger::Request &req,
                                        std_srvs::Trigger::Response &res)
    {
        std::cout << "HWPlantDeInit Service called!\n";
        stopHardware();
        res.success = true;                          // 设置响应字段的值
        res.message = "Service called successfully"; // 可选的响应消息
        return true;
    }
    void HardwareNode::stopCallback(const std_msgs::Bool::ConstPtr &msg)
    {
        if (msg->data)
        {
            std::cout << "/stop_robot received!\n";
            stopHardware();
        }
    }
    bool HardwareNode::setMotorEncoderOffsetCallback(kuavo_msgs::setMotorEncoderRoundServiceRequest &req, kuavo_msgs::setMotorEncoderRoundServiceResponse &res)
    {
        auto command_lease = command_gate_.enter();
        if (!command_lease.owns_lock()) return false;
        if (hardware_plant_->hardware_ready_)
        {
            res.success = false;
            res.message = "hardware is running, setMotorEncoderOffset is prohibited, please stop first!";
            return false;
        }

        bool ret = hardware_plant_->calibrateMotor(req.motor_id, req.direction, req.save_offset);
        res.success = ret;
        res.message = "";
        return true;
    }

    bool HardwareNode::setHwIntialStateCallback(kuavo_msgs::setHwIntialStateRequest &req, kuavo_msgs::setHwIntialStateResponse &res)
    {
        auto command_lease = command_gate_.enter();
        if (!command_lease.owns_lock()) return false;
        std::cout << "setHwIntialStateCallback called\n";
        if (hardware_plant_->hardware_ready_)

        {
            res.success = false;
            res.message = "hardware is running, setHwIntialState is prohibited, please stop first!";
            return false;
        }
        // TODO: only control legs
        if (req.q_intial.size() == 12)
        {
            std::vector<double> goal_pos(hardware_plant_->num_joint, 0);
            for (int i = 0; i < 12; i++)
            {
                goal_pos[i] = req.q_intial[i];
            }
            jointMoveTo(goal_pos, 45.0);
            res.success = true;
            res.message = "success";
            hardware_plant_->hardware_ready_ = true;
#if !defined(USE_DDS) && !defined(USE_LEJU_DDS)
            // Only subscribe to ROS joint commands when DDS is not used
            joint_sub_ = nh_.subscribe("joint_cmd", 10, &HardwareNode::jointCmdCallback, this);
#endif
            return true;
        }
        if (req.q_intial.size() != hardware_plant_->num_joint)
        {
            res.success = false;
            res.message = "q_intial size " + std::to_string(req.q_intial.size()) + " not match " + std::to_string(hardware_plant_->num_joint) + "!";
            return false;
        }
        jointMoveTo(req.q_intial, 45);
        res.success = true;
        res.message = "success";
        this->readyToRecvCmd();
        return true;
    }
    void HardwareNode::readyToRecvCmd()
    {
        if (shutdownRequested()) return;
        hardware_plant_->hardware_ready_ = true;
#if !defined(USE_DDS) && !defined(USE_LEJU_DDS)
        // Only subscribe to ROS joint commands when DDS is not used
        joint_sub_ = nh_.subscribe("joint_cmd", 10, &HardwareNode::jointCmdCallback, this);
#endif
        ros::param::set("/hardware/is_ready", 1);
        hardware_plant_->hardware_status_ = 1;

        // 发布手臂零点偏置 - 直接读取 arms_zero.yaml
        {
            std::vector<double> arm_zeros = readArmsZeroFromFile();
            std_msgs::Float64MultiArray arm_zeros_msg;
            arm_zeros_msg.data = arm_zeros;
            arm_zeros_pub_.publish(arm_zeros_msg);
            ROS_INFO("[HardwareNode] Published /kuavo/arm_zeros with %zu values", arm_zeros.size());
        }

        // 发布腿部电机零点偏置 - 直接读取 offset.csv
        {
            size_t ec_count = hardware_plant_->getCountECMasters();
            std::vector<double> leg_offset = readOffsetFromFile(ec_count);
            leg_offset.resize(ec_count);
            std_msgs::Float64MultiArray leg_offset_msg;
            leg_offset_msg.data = leg_offset;
            leg_offset_pub_.publish(leg_offset_msg);
            ROS_INFO("[HardwareNode] Published /robot/offset with %zu values", leg_offset.size());
        }
    }
    bool HardwareNode::jointMoveToServiceCallback(kuavo_msgs::jointMoveToRequest &req, kuavo_msgs::jointMoveToResponse &res)
    {
        auto command_lease = command_gate_.enter();
        if (!command_lease.owns_lock()) return false;
        std::cout << "jointMoveToServiceCallback called\n";
        if (hardware_plant_->hardware_ready_)
        {
            res.success = false;
            res.message = "hardware is running, jointMoveTo is prohibited, please stop first!";
            return false;
        }
        // TODO: only control legs
        if (req.goal_position.size() == 12)
        {
            std::vector<double> goal_pos(hardware_plant_->num_joint, 0);
            for (int i = 0; i < 12; i++)
            {
                goal_pos[i] = req.goal_position[i];
            }
            jointMoveTo(goal_pos, req.speed, req.dt);
            res.success = true;
            res.message = "success";
            return true;
        }
        if (req.goal_position.size() != hardware_plant_->num_joint)
        {
            res.success = false;
            res.message = "goal_position size " + std::to_string(req.goal_position.size()) + " not match " + std::to_string(hardware_plant_->num_joint) + "!";
            return false;
        }

        jointMoveTo(req.goal_position, req.speed, req.dt);
        res.success = true;
        res.message = "success";
        return true;
    }

    bool HardwareNode::seatReturnReversePrepCallback(std_srvs::Trigger::Request &req,
                                                     std_srvs::Trigger::Response &res)
    {
        auto command_lease = command_gate_.enter();
        if (!command_lease.owns_lock()) return false;
        (void)req;
        if (seat_reverse_prep_active_.exchange(true)) {
            res.success = false;
            res.message = "seat_return_reverse_prep already running";
            return true;
        }

        // RAII-ish：任意 return 路径都清 active + 交接 param，避免永久 mute / 误桥接
        struct PrepGuard {
            std::atomic<bool>& flag;
            ~PrepGuard() {
                ros::param::set("/hardware/seat_reverse_prep_active", false);
                flag.store(false);
            }
        } guard{seat_reverse_prep_active_};

        // 立刻对外宣告 mute，供 HC 停止 CSP 桥接；并马上锁当前编码器位，避免 jointMoveTo 启动前空窗
        ros::param::set("/hardware/seat_reverse_prep_active", true);
        {
            std::vector<double> prep_ec_kp;
            std::vector<double> prep_ec_kd;
            if (nh_.getParam("/hardware_prep_ec_joint_kp", prep_ec_kp) &&
                nh_.getParam("/hardware_prep_ec_joint_kd", prep_ec_kd) &&
                prep_ec_kp.size() == prep_ec_kd.size() && prep_ec_kp.size() == 12) {
                hardware_plant_->setPrepEcLegGains(prep_ec_kp, prep_ec_kd);
            }
            hardware_plant_->latchCurrentJointsCsp();
            std::cout << "[HardwareNode] seat_return_reverse_prep: latched current CSP under HW ownership"
                      << std::endl;
        }

        std::vector<double> prep_sit_deg;
        if (!nh_.getParam("/hardware_prep_joint_pos_deg", prep_sit_deg) ||
            prep_sit_deg.size() != static_cast<size_t>(num_joint)) {
            res.success = false;
            res.message = "missing/invalid /hardware_prep_joint_pos_deg (need sit_joint_pos deg, size=" +
                          std::to_string(num_joint) + ")";
            return true;
        }

        bool reverse_leg_first = false;
        nh_.getParam("/hardware_prep_reverse_leg_first", reverse_leg_first);
        std::vector<double> prep_leg_first_deg;
        const bool has_leg_first =
            reverse_leg_first &&
            nh_.getParam("/hardware_prep_joint_pos_leg_first_deg", prep_leg_first_deg) &&
            prep_leg_first_deg.size() == static_cast<size_t>(num_joint);

        bool reverse_arm_clear = false;
        nh_.getParam("/hardware_prep_reverse_arm_clear", reverse_arm_clear);
        std::vector<double> prep_arm_clear_deg;
        const bool has_arm_clear =
            reverse_arm_clear &&
            nh_.getParam("/hardware_prep_joint_pos_arm_clear_deg", prep_arm_clear_deg) &&
            prep_arm_clear_deg.size() == static_cast<size_t>(num_joint);
        if (reverse_arm_clear && !has_arm_clear) {
            res.success = false;
            res.message = "reverse_arm_clear set but /hardware_prep_joint_pos_arm_clear_deg missing/invalid";
            return true;
        }
        if (has_arm_clear && !has_leg_first) {
            res.success = false;
            res.message = "arm_clear requires reverse_leg_first mid targets";
            return true;
        }

        std::cout << "[HardwareNode] seat_return_reverse_prep: from current -> sit "
                  << (has_leg_first ? "(legs/waist then arms" : "(all joints")
                  << (has_arm_clear ? " via arm_clear)" : ")") << std::endl;

        try {
            if (has_leg_first) {
                std::cout << "[HardwareNode] seat reverse phase2a: legs/waist->sit (move_speed) ..."
                          << std::endl;
                jointMoveToPrepGoal(prep_leg_first_deg, 0);
                if (has_arm_clear) {
                    std::cout << "[HardwareNode] seat reverse phase2b: arms->clear (settle_speed) ..."
                              << std::endl;
                    jointMoveToPrepGoal(prep_arm_clear_deg, 1);
                    std::cout << "[HardwareNode] seat reverse phase2c: arms->sit (settle_speed) ..."
                              << std::endl;
                } else {
                    std::cout << "[HardwareNode] seat reverse phase2b: arms->sit (settle_speed) ..."
                              << std::endl;
                }
                jointMoveToPrepGoal(prep_sit_deg, 1);
            } else {
                std::cout << "[HardwareNode] seat reverse: all joints->sit (settle_speed) ..."
                          << std::endl;
                jointMoveToPrepGoal(prep_sit_deg, 1);
            }
        } catch (const std::exception& e) {
            res.success = false;
            res.message = std::string("jointMoveToPrepGoal failed: ") + e.what();
            return true;
        }

        // 对齐 use_sit_init prep 完成：可按 o / real_initial_start
        // 结束前再锁一次 sit 位（重注 prep 增益：jointMoveToPrepGoal 末尾会 clear）
        {
            std::vector<double> prep_ec_kp;
            std::vector<double> prep_ec_kd;
            if (nh_.getParam("/hardware_prep_ec_joint_kp", prep_ec_kp) &&
                nh_.getParam("/hardware_prep_ec_joint_kd", prep_ec_kd) &&
                prep_ec_kp.size() == prep_ec_kd.size() && prep_ec_kp.size() == 12) {
                hardware_plant_->setPrepEcLegGains(prep_ec_kp, prep_ec_kd);
            }
            hardware_plant_->latchCurrentJointsCsp();
            hardware_plant_->clearPrepEcLegGains();
        }
        ros::param::set("/hardware/is_ready", 0);
        ros::param::set("/hardware/ready_to_start", 1);
        res.success = true;
        res.message = "seat reverse prep to sit_joint_pos complete; ready_to_start=1";
        std::cout << "[HardwareNode] seat_return_reverse_prep done; ready_to_start=1 is_ready=0"
                  << std::endl;
        return true;
    }

    void HardwareNode::jointCmdCallback(const kuavo_msgs::jointCmd::ConstPtr &msg)
    {
        auto command_lease = command_gate_.enter();
        if (!command_lease.owns_lock()) return;
        if (seat_reverse_prep_active_.load()) {
            return;  // 座椅段0 HW reverse prep 占用电机，忽略 HC /joint_cmd
        }
        // std::cout << "jointCmdCallback called\n" << std::endl;
        size_t leg_joints = num_joint; // leg + arm + head
        auto is_match_size = [&](size_t size)
        {
            if (msg->joint_q.size() != size || msg->joint_v.size() != size ||
                msg->tau.size() != size || msg->tau_ratio.size() != size ||
                msg->control_modes.size() != size || msg->tau_max.size() != size ||
                msg->joint_kd.size() != size || msg->joint_kp.size() != size)
            {
                return false;
            }
            return true;
        };

        bool match_leg_joints_size = is_match_size(leg_joints);
        if (!match_leg_joints_size)
        {
            std::cerr << "jointCmdCallback Error: joint_q, joint_v, tau, tau_ratio, control_modes, joint_kp, joint_kd size not match!" << std::endl;
            std::cerr << "desire size:" << leg_joints << std::endl;
            std::cerr << "joint_q size:" << msg->joint_q.size() << std::endl;
            std::cerr << "joint_v size:" << msg->joint_v.size() << std::endl;
            std::cerr << "tau size:" << msg->tau.size() << std::endl;
            std::cerr << "tau_ratio size:" << msg->tau_ratio.size() << std::endl;
            std::cerr << "control_modes size:" << msg->control_modes.size() << std::endl;
            std::cerr << "tau_max size:" << msg->tau_max.size() << std::endl;
            std::cerr << "joint_kp size:" << msg->joint_kp.size() << std::endl;
            std::cerr << "joint_kd size:" << msg->joint_kd.size() << std::endl;
            return;
        }
        Eigen::VectorXd cmd, cmd_out,joint_kp(num_joint),joint_kd(num_joint); // q,v,tau,tau_max,tau_ratio
        cmd.resize(num_joint * 5);
        cmd_out.resize(num_joint * 5);
        cmd_out.setZero();
        joint_kp.setZero();
        joint_kd.setZero();

        std::vector<int> control_mode_vec(num_joint, 2);
        for (uint32_t i = 0; i < num_joint; i++)
        {
            joint_kp[i] = msg->joint_kp[i];
            joint_kd[i] = msg->joint_kd[i];

            cmd[num_joint * 0 + i] = msg->joint_q.at(i);
            cmd[num_joint * 1 + i] = msg->joint_v.at(i);
            cmd[num_joint * 2 + i] = msg->tau.at(i);
            cmd[num_joint * 3 + i] = msg->tau_max.at(i);
            cmd[num_joint * 4 + i] = msg->tau_ratio.at(i);
            control_mode_vec[i] = msg->control_modes.at(i);
        }

        applyArmActuatorDynamicsCompensation(msg, cmd);

        hardware_plant_->cmds2Cmdr(cmd, num_joint, cmd_out, num_joint);

        // 将输出cmd_out在电机阈值内截断
        cmdTruncation(cmd_out);

        // 触发真实值保护时的action - 在截断后执行，确保失能操作不被覆盖
        JointProtectionAction(cmd_out, control_mode_vec);

        logger_ptr_->publishVector("motor_cmd/motor_pos", cmd_out.segment(num_joint * 0, num_joint));
        logger_ptr_->publishVector("motor_cmd/motor_vel", cmd_out.segment(num_joint * 1, num_joint));
        logger_ptr_->publishVector("motor_cmd/motor_cur", cmd_out.segment(num_joint * 2, num_joint));
        logger_ptr_->publishVector("motor_cmd/motor_max_cur", cmd_out.segment(num_joint * 3, num_joint));
        logger_ptr_->publishVector("motor_cmd/motor_cur_ratio", cmd_out.segment(num_joint * 4, num_joint));
        if (cmd_out.hasNaN())
        {
            std::cout << "cmd_out.hasNaN \ncmd_out:" << cmd_out.segment(num_joint * 2, num_joint).segment(10, 2).transpose() << std::endl;
            std::cout << "cmd" << cmd.transpose() << std::endl;
            std::cout << "cmd_out:" << cmd_out.transpose() << std::endl;
            cmd_out = cmd_out_prev_;
        }
        cmd_out_prev_ = cmd_out;

        hardware_plant_->writeCommand(cmd_out, num_joint, control_mode_vec, joint_kp, joint_kd);
    }

    void HardwareNode::handCmdCallback(const kuavo_msgs::robotHandPosition::ConstPtr &msg)
    {
        auto command_lease = command_gate_.enter();
        if (!command_lease.owns_lock()) return;
        if (msg->left_hand_position.size() != 6 || msg->right_hand_position.size() != 6) 
        {
        ROS_WARN("Received desired positions vector of incorrect size");
        return;
        }
        // 目前通过直接调用hand_sdk
        Eigen::VectorXd left_end_effc_position(6);
        Eigen::VectorXd right_end_effc_position(6);
        left_end_effc_position.setZero();
        right_end_effc_position.setZero();

        // left_hand_position 的值
        for (size_t i = 0; i < msg->left_hand_position.size() && i < 6; ++i) {
            left_end_effc_position[i] = static_cast<int8_t>(msg->left_hand_position[i]);
        }

        // right_hand_position 的值
        for (size_t i = 0; i < msg->right_hand_position.size() && i < 6; ++i) {
            right_end_effc_position[i] = static_cast<int8_t>(msg->right_hand_position[i]);
        }
        std::vector<EndEffectorData> end_effector_cmd;
        end_effector_cmd.push_back(EndEffectorData{EndEffectorType::qiangnao, left_end_effc_position, Eigen::VectorXd::Zero(6), Eigen::VectorXd::Zero(6)});
        end_effector_cmd.push_back(EndEffectorData{EndEffectorType::qiangnao, right_end_effc_position, Eigen::VectorXd::Zero(6), Eigen::VectorXd::Zero(6)});
        hardware_plant_->endEffectorCommand(end_effector_cmd);
    }

    void HardwareNode::cmdTruncation(Eigen::VectorXd &cmd_out)
    {
        if(!cmd_truncation_enable_)
        {
            return;
        }

        if(1 != hardware_plant_->hardware_status_)
        {
            return;
        }

        // 使用电机的物理限制值
        auto motor_info = hardware_plant_->get_motor_info();
        
        // 电机数据通以弧度为单位
        for (uint32_t i = 0; i < num_joint; i++)
        {
            // 位置截断阈值 （度为单位)
            auto pos = cmd_out[i];
            double pos_min = motor_info.min_joint_position_limits[i];  
            double pos_max = motor_info.max_joint_position_limits[i];
            
            // 转换为弧度进行比较
            double pos_min_rad = pos_min * M_PI / 180.0;
            double pos_max_rad = pos_max * M_PI / 180.0;
            
            if (pos < pos_min_rad) 
            {
                pos = pos_min_rad;
            } 
            else if (pos > pos_max_rad) 
            {
                pos = pos_max_rad;
            }
            cmd_out[i] = pos;

            // 电机速度截断 (rad/sec)
            auto vel = cmd_out[num_joint + i];
            double vel_limit = motor_info.joint_velocity_limits[i];  // rad/sec
            
            if (vel < -vel_limit) 
            {
                vel = -vel_limit;
            } 
            else if (vel > vel_limit) 
            {
                vel = vel_limit;
            }
            cmd_out[num_joint + i] = vel;

            // 扭矩截断 (cmd_out[2*num_joint+i] 存储的是扭矩)
            auto torque = cmd_out[2 * num_joint + i];
            double peak_torque_limit = motor_info.joint_peak_limits[i];  // 峰值扭矩限制
            
            // 与峰值扭矩进行比较截断
            if (torque < -peak_torque_limit) 
            {
                torque = -peak_torque_limit;
            } 
            else if (torque > peak_torque_limit) 
            {
                torque = peak_torque_limit;
            }
            cmd_out[2 * num_joint + i] = torque;
            
            // 最大扭矩截断
            auto max_torque = cmd_out[3 * num_joint + i];
            double max_current_limit = peak_torque_limit;
            if (max_torque > max_current_limit) 
            {
                max_torque = max_current_limit;
            } 
            else if (max_torque < 0) 
            {
                max_torque = 0;
            }
            cmd_out[3 * num_joint + i] = max_torque;
        }
    }

    void HardwareNode::JointProtectionAction(Eigen::VectorXd &cmd_out, std::vector<int> &control_mode)
    {

        if(!joint_protect_enable_)
        {
            return;
        }

        // 获取所有关节的状态（用于已失能的EC电机继续0扭矩控制）
        auto allJointsStatus = hardware_plant_->getAllJointsStatus();
        
        // 将之前失能的EC电机继续0扭矩控制
        for (const auto& jointStatus : allJointsStatus)
        {
            int joint_id = jointStatus.first;
            auto status = jointStatus.second;
            
            // 修复逻辑错误：EC电机是 <= 12 或 = 13 或 = 20
            bool isECMotor = (joint_id <= 12 || joint_id == 13 || joint_id == 20);
            if (!isECMotor)
            {
                continue;
            }
            
            // 如果电机已禁用或错误，设置0扭矩控制
            if (status == MotorStatus::DISABLED || status == MotorStatus::ERROR)
            {
                int index = joint_id - 1;  // 转换为0-based索引
                control_mode[index] = 0;
                cmd_out[2 * num_joint + index] = 0;  // 在电机空间设置0电流
                cmd_out[3 * num_joint + index] = 0;
            }
        }

        // 最多尝试处理一个有效的失能请求（跳过已禁用的电机）
        for (int attempt = 0; attempt < 3; ++attempt)  // 最多尝试3次，避免无限循环
        {
            auto motorIndex = hardware_plant_->getDisableMotorId();
            if (motorIndex < 0 || motorIndex > 28)
            {
                return;  // 队列为空或ID无效，结束处理
            }

            int index = motorIndex - 1;
            
            // 使用电机状态管理器检查电机是否已经禁用
            if (hardware_plant_->isJointMarkedAsDisabled(motorIndex))
            {
                std::cout << "####### motor " << motorIndex << " already disabled, trying next #######" << std::endl;
                continue;  // 尝试下一个ID
            }
            
            // 找到有效的失能请求，执行处理
            // 判断是否为EC电机
            bool isECMotor = (motorIndex <= 12 || motorIndex == 13 || motorIndex == 20);
            if (isECMotor)
            {
                // 使用电机状态管理接口记录状态（不控制硬件）
                std::string reason = "Joint protection triggered";
                hardware_plant_->disableMotor(motorIndex, reason);
                
                std::cout << "####### set motor " << motorIndex << " disable #######" << std::endl;
                
                // 对于EC电机，由于disable接口有问题，继续使用0扭矩控制
                control_mode[index] = 0;
                cmd_out[2 * num_joint + index] = 0;  // 在电机空间设置0电流
                cmd_out[3 * num_joint + index] = 0;
                
                // 如果启用了腿部全失能模式，且失能的是腿部关节(1-12)，则失能所有腿部关节
                if (leg_disable_mode_ && motorIndex >= 1 && motorIndex <= 12) 
                {
                    std::cout << "####### leg disable mode enabled: disabling all leg joints (1-12) #######" << std::endl;
                    for (int leg_joint = 1; leg_joint <= 12; ++leg_joint) 
                    {
                        int leg_index = leg_joint - 1;
                        hardware_plant_->disableMotor(leg_joint, "Leg cascade disable triggered by joint " + std::to_string(motorIndex));
                        control_mode[leg_index] = 0;
                        cmd_out[2 * num_joint + leg_index] = 0;  // 在电机空间设置0电流
                        cmd_out[3 * num_joint + leg_index] = 0;
                    }
                }
            }
            else
            {
                // 其他电机：调用硬件接口
                std::string reason = "Joint protection triggered";
                auto disableFlag = hardware_plant_->disableMotor(motorIndex, reason);
                if (!disableFlag)
                {
                    ROS_WARN_STREAM("Failed to disable motor " << motorIndex);
                    return;  // 失能失败，结束处理
                }
                std::cout << "####### set motor " << motorIndex << " disable #######" << std::endl;
            }
            
            break;  // 成功处理一个失能请求，退出循环
        }
    }

    int HardwareNode::JointProtectionTrigger(PROTEST_MODE mode, std::vector<double> &values)
    {
        if(!joint_protect_enable_)
        {
            return -1;
        }

        // 判断是否需要启用保护逻辑
        bool enable_protection = true;
        if (!hardware_plant_->hardware_ready_) 
        {
            // 堵转和峰值保护始终启用，其它保护不启用
            if (mode != PROTEST_MODE::ENUM_PEAR_PROTECTION && mode != PROTEST_MODE::ENUM_LOCKED_ROTOR_PROTECTION) 
            {
                enable_protection = false;
            }
        }

        if (!enable_protection)
        {
            //TODO
            ros::Time current_time = ros::Time::now();
            for (int i = 0; i < num_joint; i++) 
            {
                pre_vel_over_time_[i] = current_time;
            }
            return -1;
        }


        std::string name = "";
        double timeout = 0.0;
        std::vector<double> thresh_limit(28);

        switch (mode)
        {
        case PROTEST_MODE::ENUM_PEAR_PROTECTION:
        {
            name = "peak protection";
            timeout = hardware_plant_->peakTimeWin_;
            thresh_limit = hardware_plant_->joint_peak_torque_limits_;
            break;
        }
        case PROTEST_MODE::ENUM_LOCKED_ROTOR_PROTECTION:
        {
            name = "lock rotor protection";
            timeout = hardware_plant_->lockRotorTimeWin_;
            thresh_limit = hardware_plant_->joint_lock_rotor_limits_;
            break;
        }
        case PROTEST_MODE::ENUM_SPEED_PROTECTION:
        {
            name = "speed protection";
            timeout = hardware_plant_->speedTimeWin_;
            thresh_limit = hardware_plant_->joint_velocity_limits_;
            break;
        }
        default:
        {
            std::cerr << "cur protect mode err: " << mode << ", protect is not enforced." << std::endl;
            return -1;
        }
        }

        for (int i = 0; i < num_joint; i++)
        {
            // 使用电机状态管理器检查电机是否已经禁用
            if (hardware_plant_->isJointMarkedAsDisabled(i + 1))  // 注意：i是0-based，joint_id是1-based
            {
                continue;
            }

            ros::Time pre_time;
            pre_time = ros::Time::now();
            if (fabs(values[i]) < thresh_limit[i])
            {
                if (PROTEST_MODE::ENUM_PEAR_PROTECTION == mode)
                {
                    pre_peak_over_time_[i] = pre_time;
                }
                else if (PROTEST_MODE::ENUM_LOCKED_ROTOR_PROTECTION == mode)
                {
                    pre_cur_over_time_[i] = pre_time;
                }
                else
                {
                    pre_vel_over_time_[i] = pre_time;
                }
                pre_time = ros::Time::now();
            }
            else
            {
                if (PROTEST_MODE::ENUM_PEAR_PROTECTION == mode)
                {
                    pre_time = pre_peak_over_time_[i];  // 修复：峰值保护应该获取峰值时间戳
                }
                else if (PROTEST_MODE::ENUM_LOCKED_ROTOR_PROTECTION == mode)
                {
                    pre_time = pre_cur_over_time_[i];   // 堵转保护获取电流时间戳
                }
                else
                {
                    pre_time = pre_vel_over_time_[i];   // 修复：速度保护应该获取速度时间戳
                }
            }

            auto diff = ros::Time::now() - pre_time;
            if (diff > ros::Duration(timeout))
            {
                auto id = i + 1;
                if (hardware_plant_->addDisableMotorId(id))
                {
                    std::cerr << "################# Trigger " << name << " #################" << std::endl;
                    std::cerr << "joint[" << id << "] cur pos=" << curPos_[i] << std::endl;
                    std::cerr << "value " << values[i] << " is over limit[" << thresh_limit[i] << "], time window " << diff.toSec() << std::endl;
                    std::cerr << "add disable motor id: " << id << ", The current number of ids to be processed " << hardware_plant_->getDisableMotorSize() << std::endl;
                    std::cerr << "###########################  ###########################" << std::endl;
                    if (!hardware_plant_->hardware_ready_) 
                    {
                         std::cerr << "The current hardware is not ready. Terminate the program." << std::endl;
                        requestShutdown();
                        std_msgs::Bool stop;
                        stop.data = true;
                        stop_pub_.publish(stop);
                    }
                    return id;
                }
            }
        }

        return -1;
    }

    void HardwareNode::sensorThreadFunc(ros::Publisher &sensor_data_pub)
    {
        sched_thread(30);
        SensorData_t sensor_data_motor;
        SensorData_t sensor_data_joint;
        sensor_data_joint.resizeJoint(num_joint);
        ros::WallRate loop_rate(1 / dt_);
        std::cout << "loop_rate:" << 1 / dt_ << std::endl;
        ros::Time start_time = ros::Time::now();
        std::cout << "th_running_: " << hardware_plant_->th_running_ << std::endl;
        while (ros::ok() && sensor_thread_running_ && !shutdownRequested() && hardware_plant_->th_running_)
        {
            bool ret = hardware_plant_->readSensor(sensor_data_motor);
            if (!ret)
            {
                loop_rate.sleep();
                continue;
            }
            // std::cout << "sensor_data_motor.joint_q:" << sensor_data_motor.joint_q.segment(0, 12).transpose() << std::endl;
            logger_ptr_->publishVector("sensor_data_motor/motor_pos", sensor_data_motor.joint_q);
            logger_ptr_->publishVector("sensor_data_motor/motor_vel", sensor_data_motor.joint_v);
            logger_ptr_->publishVector("sensor_data_motor/motor_cur", sensor_data_motor.joint_current);
            logger_ptr_->publishVector("sensor_data_motor/motor_torque_demand", sensor_data_motor.joint_torque_demand);
            logger_ptr_->publishVector("sensor_data_motor/motor_igbt_temperature", sensor_data_motor.joint_igbt_temperature);
            logger_ptr_->publishVector("sensor_data_motor/motor_ntc_temperature", sensor_data_motor.joint_ntc_temperature);
            logger_ptr_->publishVector("sensor_data_motor/motor_status_word", sensor_data_motor.joint_status_word);
            logger_ptr_->publishVector("sensor_data_motor/motor_error_code", sensor_data_motor.joint_error_code);
            hardware_plant_->motor2joint(sensor_data_motor, sensor_data_joint);
            hardware_plant_->setState(sensor_data_motor,sensor_data_joint);


            // 检测从传感器实时的vel 和 current是否超出limit
            //  vel limit

            std::vector<double> velVec(sensor_data_joint.joint_v.data(), sensor_data_joint.joint_v.data() + sensor_data_joint.joint_v.size());
            std::vector<double> posVec(sensor_data_joint.joint_q.data(), sensor_data_joint.joint_q.data() + sensor_data_joint.joint_q.size());
            
            // 获取原始电机数据用于堵转和峰值保护
            std::vector<double> motorCurVec(sensor_data_motor.joint_current.data(), sensor_data_motor.joint_current.data() + sensor_data_motor.joint_current.size());
            
            {
                std::lock_guard<std::mutex> lk(enable_mpc_mtx_);
                curVel_ = velVec;
                curPos_ = posVec;

                // 为电流保护创建转换后的扭矩数据
                std::vector<double> motorTorqueVec;
                motorTorqueVec.reserve(motorCurVec.size());
                for (size_t i = 0; i < motorCurVec.size(); ++i)
                {
                    motorTorqueVec.push_back(motorCurVec[i] * c2t_coeff_[i]);
                }

                //实际值峰值保护 - 使用转换后的扭矩数据
                JointProtectionTrigger(ENUM_PEAR_PROTECTION, motorTorqueVec);

                //实际值堵转保护 - 使用转换后的扭矩数据
                JointProtectionTrigger(ENUM_LOCKED_ROTOR_PROTECTION, motorTorqueVec);

                //实际值飞车保护 - 使用虚拟关节速度数据（关注关节空间运动安全，而非电机转速）
                JointProtectionTrigger(ENUM_SPEED_PROTECTION, velVec);
            }

            if (shutdownRequested()) break;
            // std::cout << "sensor_data_joint.joint_q:" << sensor_data_joint.joint_q.segment(0, 12).transpose() << std::endl;
            kuavo_msgs::sensorsDataPtr msg_ptr(new kuavo_msgs::sensorsData); // 使用nodelet时使用pt类型的消息可以无拷贝复制
            msg_ptr->header.stamp = ros::Time::now();
            msg_ptr->sensor_time = start_time;
            msg_ptr->joint_data.joint_q = eigenToStdVector(sensor_data_joint.joint_q);
            msg_ptr->joint_data.joint_v = eigenToStdVector(sensor_data_joint.joint_v);
            msg_ptr->joint_data.joint_vd = eigenToStdVector(sensor_data_joint.joint_vd);
            msg_ptr->joint_data.joint_torque = eigenToStdVector(sensor_data_joint.joint_current);
            msg_ptr->imu_data.acc.x = sensor_data_joint.acc.x();
            msg_ptr->imu_data.acc.y = sensor_data_joint.acc.y();
            msg_ptr->imu_data.acc.z = sensor_data_joint.acc.z();
            msg_ptr->imu_data.gyro.x = sensor_data_joint.gyro.x();
            msg_ptr->imu_data.gyro.y = sensor_data_joint.gyro.y();
            msg_ptr->imu_data.gyro.z = sensor_data_joint.gyro.z();
            msg_ptr->imu_data.free_acc.x = sensor_data_joint.free_acc.x();
            msg_ptr->imu_data.free_acc.y = sensor_data_joint.free_acc.y();
            msg_ptr->imu_data.free_acc.z = sensor_data_joint.free_acc.z();
            msg_ptr->imu_data.quat.x = sensor_data_joint.quat[1];
            msg_ptr->imu_data.quat.y = sensor_data_joint.quat[2];
            msg_ptr->imu_data.quat.z = sensor_data_joint.quat[3];
            msg_ptr->imu_data.quat.w = sensor_data_joint.quat[0];

            // msg_ptr->eno_effectors_data = sensor_data_joint.end_effectors_data;
#if defined(USE_DDS) || defined(USE_LEJU_DDS)
            // Publish sensor data via DDS when DDS is enabled
            // Pass start_time's seconds and nanoseconds as timestamp
            uint32_t timestamp_sec = static_cast<uint32_t>(start_time.sec);
            uint32_t timestamp_nsec = static_cast<uint32_t>(start_time.nsec);
            hardware_plant_->publishStateViaDDS(sensor_data_joint, timestamp_sec, timestamp_nsec);

#else

            sensor_data_pub.publish(msg_ptr);
#endif
            loop_rate.sleep();
            start_time += ros::Duration(dt_);
        }

        std::cout << "sensorThread exit" << std::endl;
    }

    void HardwareNode::stopEndEffectorRosNodes()
    {
        // Also reached directly by motor protection. Cancel hand recovery before
        // joining, and serialize concurrent end-effector shutdown callers.
        sg100_shutdown_requested_ = true;
        std::lock_guard<std::mutex> hand_lock(end_effector_lifecycle_mutex_);
        sg100_init_retry_.stop();
        if (sg100_driver_) {
            sg100_driver_.reset();
            if (ros::ok()) nh_.setParam("end_effector_joints_num", 0);
        }
        if (dexhand_ros_actuator_) {
            dexhand_ros_actuator_->stop();
        }
        if (revo2_dexhand_ros_actuator_) {
            revo2_dexhand_ros_actuator_->stop();
        }
    }

    void HardwareNode::deInitHardwarePlant()
    {
        if (!hardware_plant_ || hardware_plant_->is_deinitialized_) {
            return;
        }
        stopEndEffectorRosNodes();
        hardware_plant_->HWPlantDeInit();
        publishRobotDisabledFlag();
    }

    void HardwareNode::stopHardware()
    {
        requestShutdown();
        std::lock_guard<std::mutex> lifecycle_lock(hardware_lifecycle_mutex_);
        sensor_thread_running_ = false;
        if (sensor_thread_.joinable()) sensor_thread_.join();
        {
            auto drained = command_gate_.drain();
            (void)drained;  // Keep the RAII lease alive until driver teardown completes.
            if (ros::ok()) ros::param::set(kHardwareStateParam, kHardwareDeinitializing);
            deInitHardwarePlant();
            if (ros::ok()) {
                ros::param::set(kHardwareStateParam, kHardwareDeinitialized);
                ros::param::set("/hardware/is_ready", 0);
                ros::param::set("/hardware/ready_to_start", 0);
            }
        }
        joint_sub_.shutdown();
    }

    HardwareNode::~HardwareNode()
    {
        stopHardware();
        if (logger_ptr_)
        {
            delete logger_ptr_;
            logger_ptr_ = nullptr;
        }
    }

    bool HardwareNode::readStartupLine(std::string& line)
    {
        line.clear();
        while (ros::ok() && !shutdownRequested()) {
            pollfd input{STDIN_FILENO, POLLIN, 0};
            const int result = poll(&input, 1, 100);
            if (result < 0) { if (errno == EINTR) continue; return false; }
            if (result == 0) continue;
            if (!(input.revents & POLLIN)) return false;
            char c;
            if (::read(STDIN_FILENO, &c, 1) != 1) return false;
            if (c == '\n') return true;
            if (c != '\r') line.push_back(c);
        }
        return false;
    }

    HardwareNode::HardwareNode(ros::NodeHandle &nh, double dt) : nh_(nh), dt_(dt)
    {
        logger_ptr_ = new ocs2::humanoid::TopicLogger(nh_);

        // 初始化三个新服务
        adjust_zero_point_srv_ = nh_.advertiseService("hardware/adjust_zero_point", &HardwareNode::adjustZeroPointCallback, this);
        get_hardware_ready_srv_ = nh_.advertiseService("hardware/get_hardware_ready", &HardwareNode::getHardwareReadyCallback, this);
        get_motor_zero_points_srv_ = nh_.advertiseService("hardware/get_motor_zero_points", &HardwareNode::getMotorZeroPointsCallback, this);
        switch_robot_state_srv_ = nh_.advertiseService("hardware/switch_robot_state", &HardwareNode::switchRobotStateCallback, this); // 新增的服务


        joint_move_to_srv_ = nh_.advertiseService("hardware/joint_move_to", &HardwareNode::jointMoveToServiceCallback, this);
        seat_return_reverse_prep_srv_ = nh_.advertiseService(
            "hardware/seat_return_reverse_prep", &HardwareNode::seatReturnReversePrepCallback, this);
        set_intial_state_srv_ = nh_.advertiseService("hardware/set_intial_state", &HardwareNode::setHwIntialStateCallback, this);
        modify_motor_encoder_offset_srv_ = nh_.advertiseService("hardware/modify_motor_encoder_offset", &HardwareNode::setMotorEncoderOffsetCallback, this);

        change_motor_param_srv_ = nh_.advertiseService("hardware/change_motor_param", &HardwareNode::changeMotorParamCallback, this);
        get_motor_param_srv_ = nh_.advertiseService("hardware/get_motor_param", &HardwareNode::getMotorParamCallback, this);

        real_launch_status_srv_ = nh_.advertiseService("humanoid_controller/real_launch_status", &HardwareNode::realLaunchStatusCallback, this);

        deintial_srv_ = nh_.advertiseService("hardware/deinitial", &HardwareNode::deintialCallback, this);
        stop_sub_ = nh_.subscribe<std_msgs::Bool>("/stop_robot", 10, &HardwareNode::stopCallback, this);
        stop_pub_ = nh_.advertise<std_msgs::Bool>("/stop_robot", 10);
        robot_disabled_flag_pub_ = nh_.advertise<std_msgs::Bool>("/robot_disabled_flag", 1, true);
        ros::param::set(kHardwareStateParam, kHardwareRunning);
        publishRobotDisabledFlag(false);
        re_start_sub_ = nh_.subscribe<std_msgs::Bool>("/re_start_robot", 10, &HardwareNode::reStartCallback, this);
        bot_stand_up_sub_ = nh_.subscribe<std_msgs::Int8>("/bot_stand_up_complete", 10, &HardwareNode::getBotStandUpCompleteCallback, this);

        enable_mpc_pub_ = nh_.advertise<std_msgs::Bool>("/enable_mpc_flag", 10);
        enable_wbc_pub_ = nh_.advertise<std_msgs::Bool>("/enable_wbc_flag", 10);
        arm_zeros_pub_ = nh_.advertise<std_msgs::Float64MultiArray>("/robot/arm_zeros", 1, true);
        leg_offset_pub_ = nh_.advertise<std_msgs::Float64MultiArray>("/robot/offset", 1, true);

        
        hardware_param_ = HardwareParam();
        if (nh_.hasParam("/robot_version")) {
            int rb_version_int;
            nh_.getParam("/robot_version", rb_version_int);
            hardware_param_.robot_version = RobotVersion::create(rb_version_int);
        }
        if (nh_.hasParam("cali_leg")) {
            nh_.getParam("cali_leg", hardware_param_.cali_leg);
        }
        if (nh_.hasParam("cali_arm")) {
            nh_.getParam("cali_arm", hardware_param_.cali_arm);
        }
        if (nh_.hasParam("cali")) {
            nh_.getParam("cali", hardware_param_.cali);
        }
        if (nh_.hasParam("teach_pendant")) {
            nh_.getParam("teach_pendant", hardware_param_.teach_pendant_);
        }
        if (nh_.hasParam("check_firmware_param")) {
            nh_.getParam("check_firmware_param", hardware_param_.check_firmware_param_);
        }
        if (nh_.hasParam("allow_mismatch")) {
            nh_.getParam("allow_mismatch", hardware_param_.allow_mismatch_);
        }
        if (nh_.hasParam("default_joint_pos"))
        {
            nh_.getParam("default_joint_pos", hardware_param_.default_joint_pos);
        }
        if (nh_.hasParam("joint_protect_enable"))
        {
            nh_.getParam("joint_protect_enable", joint_protect_enable_);  
        }
        std::cout << "joint protect enable status: " << joint_protect_enable_ << std::endl;
        if (nh_.hasParam("cmd_truncation_enable"))
        {
            nh_.getParam("cmd_truncation_enable", cmd_truncation_enable_);
        }
        std::cout << "cmd truncation enable status: " << cmd_truncation_enable_ << std::endl;
        
        bool is_cali_set_zero_temp = false;
        if(nh_.hasParam("cali_set_zero"))
        {
            nh_.getParam("cali_set_zero", is_cali_set_zero_temp);
        }
        
        if (nh_.hasParam("leg_disable_mode"))
        {
            nh_.getParam("leg_disable_mode", leg_disable_mode_);
        }
        std::cout << "leg disable mode: " << (leg_disable_mode_ ? "ALL_LEGS" : "INDIVIDUAL") 
                  << " (when leg joint fails, " << (leg_disable_mode_ ? "all leg joints disabled" : "only failed joint disabled") << ")" << std::endl;

        const std::string hardware_node_path = ros::package::getPath("hardware_node");
        const std::string hardware_plant_path = std::filesystem::canonical(hardware_node_path).parent_path().string() + "/hardware_plant";
        const std::string kuavo_assets_path = ocs2::kuavo_assets::getPath();
        hardware_param_.kuavo_assets_path = kuavo_assets_path;

#ifdef USE_DDS
        hardware_plant_ = std::make_unique<HardwareDdsPlant<unitree_hg::msg::dds_::LowCmd_, unitree_hg::msg::dds_::LowState_>>(dt_, hardware_param_, hardware_plant_path);
#elif defined(USE_LEJU_DDS)
        hardware_plant_ = std::make_unique<HardwareDdsPlant<leju::msgs::JointCmd, leju::msgs::SensorsData>>(dt_, hardware_param_, hardware_plant_path);
#else
        hardware_plant_ = std::make_unique<HardwarePlant>(dt_, hardware_param_, hardware_plant_path);
#endif
        leju_claw_debug_pub_ = nh_.advertise<std_msgs::String>("leju_claw_debug", 100);
        leju_claw_target_pub_ = nh_.advertise<kuavo_msgs::lejuClawTarget>("leju_claw_target", 100);
        hardware_plant_->setLejuClawDebugCallback(
            [this](const std::string& debug_msg) {
                this->publishLejuClawDebug(debug_msg);
            });
        hardware_plant_->setLejuClawTargetCallback(
            [this](const std::vector<std::string>& name,
                   const std::vector<double>& position,
                   const std::vector<double>& velocity,
                   const std::vector<double>& effort,
                   const std::vector<int>& kp,
                   const std::vector<int>& kd,
                   bool zero_kp) {
                this->publishLejuClawTarget(name, position, velocity, effort, kp, kd, zero_kp);
            });
        
        // 在hardware_plant_初始化后设置参数值
        hardware_plant_->is_cali_set_zero_ = is_cali_set_zero_temp;    
        num_joint = get_num_actuated();
        his_cmd_.resize(num_joint * 5);
        curVel_.resize(num_joint);
        curPos_.resize(num_joint);
        actuatorDynamicsCompensatorPtr_ = std::make_unique<leju_utils::ActuatorDynamicsCompensator>();

        constexpr int kArmCompensationDof = 14;
        const int armJointNum = hardware_plant_->num_arm_joints;
        const int headJointNum = hardware_plant_->num_head_joints;
        const int armStartIndex = num_joint - headJointNum - armJointNum;
        if (armJointNum == kArmCompensationDof && armStartIndex >= 0 &&
            armStartIndex + kArmCompensationDof <= num_joint)
        {
            armJointIndices_.reserve(kArmCompensationDof);
            for (int i = 0; i < kArmCompensationDof; ++i)
            {
                armJointIndices_.push_back(armStartIndex + i);
            }
            ROS_INFO("[HardwareNode] Actuator dynamics compensation is attached to arm joints [%d, %d].",
                     armJointIndices_.front(), armJointIndices_.back());
        }
        else
        {
            ROS_WARN("[HardwareNode] Arm actuator dynamics compensation is skipped: expected arm dof=14, got %d.", armJointNum);
        }

        ros::Time current_time = ros::Time::now();
        pre_vel_over_time_.resize(num_joint,current_time);
        pre_cur_over_time_.resize(num_joint,current_time);
        pre_peak_over_time_.resize(num_joint,current_time);
        
        hardware_plant_->GetC2Tcoeff(c2t_coeff_, 28);
        std::cout << "c2tcoeff[";
        for (int i = 0; i < 28; i++)
        {
            std::cout << c2t_coeff_[i] << " ";
        }
        std::cout << "]" << std::endl;
        std::string robot_module = hardware_plant_->getRobotModule();
        jointMoveSpeed_ = (isLunbiModule(robot_module)) ? 12.0 : 60.0;
        std::cout << "Set jointMove speed: " << jointMoveSpeed_ << std::endl;
        // motorStatusMap_ = {{0, true}, {1, true}, {2, true}, {3, true}, {4, true}, {5, true}, {6, true}, {7, true}, {8, true}, {9, true}, {10, true}, {11, true}, {12, true}, {19, true}};
    }

    bool HardwareNode::tryInitializeSg100()
    {
        if (shutdownRequested() || sg100_shutdown_requested_.load() || !ros::ok()) return false;
        if (sg100_driver_) return true;
        try {
            // Each failed attempt destroys all partial SDK/ROS state before the
            // next attempt. Only this owner opens the hand serial ports.
            auto candidate = std::make_shared<SG100HandROSNode>(nh_, nh_);
            if (!candidate->initialize()) {
                ROS_ERROR("[HardwareNode] SG100 initialization failed; "
                          "end_effector_joints_num remains 0");
                return false;
            }
            if (shutdownRequested() || sg100_shutdown_requested_.load() || !ros::ok()) return false;
            nh_.setParam("end_effector_joints_num", candidate->getHandJointsNum());
            sg100_driver_ = std::move(candidate);
            ROS_INFO("[HardwareNode] SG100 initialized; hand services are available");
            return true;
        } catch (const std::exception& error) {
            ROS_ERROR("[HardwareNode] SG100 initialization exception: %s", error.what());
        } catch (...) {
            ROS_ERROR("[HardwareNode] SG100 initialization raised an unknown exception");
        }
        return false;
    }

    void HardwareNode::init()
    {
        std::lock_guard<std::mutex> lifecycle_lock(hardware_lifecycle_mutex_);
        if (shutdownRequested()) return;
        bool use_sit_init = false;
        nh_.param("/use_sit_init", use_sit_init, false);
        hardware_plant_->setSkipBootMoveToZero(use_sit_init);
        if (use_sit_init) {
            ROS_INFO("[HardwareNode] /use_sit_init=true: skip boot moveToZero for arm actuators");
        }

        bool use_custom_arm_prepare = false;
        nh_.param("/use_custom_arm_prepare", use_custom_arm_prepare, false);
        if (use_custom_arm_prepare && !hardware_param_.cali && !hardware_param_.cali_arm) {
            std::vector<double> arm_pos;
            const int arm_dof = hardware_plant_->num_arm_joints;
            if (nh_.getParam("/arm_prepare/joint_pos", arm_pos) &&
                static_cast<int>(arm_pos.size()) == arm_dof && arm_dof == 14 &&
                std::all_of(arm_pos.begin(), arm_pos.end(), [](double q) { return std::isfinite(q); })) {
                hardware_plant_->setBootArmTargetRad(arm_pos);
                ROS_INFO("[HardwareNode] use_custom_arm_prepare: boot moveToZero uses /arm_prepare/joint_pos");
            } else {
                ROS_ERROR("[HardwareNode] use_custom_arm_prepare is true, but /arm_prepare/joint_pos is missing or size %zu != %d. Boot moveToZero stays at 0.",
                          arm_pos.size(), arm_dof);
            }
        }
        const auto initialization_result = hardware_plant_->HWPlantInit();
        if (initialization_result != 0) {
            requestShutdown();
            throw std::runtime_error("Hardware plant initialization failed: " +
                                     std::to_string(initialization_result));
        }
        if (shutdownRequested()) return;
        if (hardware_plant_ != nullptr)
        {
            hardware_plant_->th_running_ = true;
            ROS_INFO("[HardwareNode] Hardware plant initialized successfully");
        }

        nh_.setParam("end_effector_type", hardware_plant_->end_effector_type_);
        
        bool end_effector_joints_num_set = false;

        auto motor_info = hardware_plant_->get_motor_info();
        HighlyDynamic::HandProtocolType hand_protocol_type = motor_info.getHandProtocolType();
        bool is_can_protocol = hand_protocol_type == HighlyDynamic::HandProtocolType::PROTO_CAN;

        if (hardware_plant_->end_effector_type_ == "heiman") {
            // Discard stale readiness from a previous launch before opening ports.
            nh_.setParam("end_effector_joints_num", 0);
            end_effector_joints_num_set = tryInitializeSg100();
        }

        if (hardware_plant_->end_effector_type_ == "qiangnao" || hardware_plant_->end_effector_type_ == "linker_hand") {

            const bool is_linker_hand = (hardware_plant_->end_effector_type_ == "linker_hand");
            auto gesture_filepath = hardware_plant_->gesture_filepath_;
            dexhand_ros_actuator_ = std::make_shared<eef_controller::DexhandRosNode>();
            if(hardware_plant_->dexhand_actuator && !dexhand_ros_actuator_->init(nh_, gesture_filepath, kDexhandStatePublishFrequency, hardware_plant_->dexhand_actuator, false, is_can_protocol, is_linker_hand)) {
                std::cerr << "[HardwareNode] Failed to initialize." << std::endl;
            }
            nh_.setParam("end_effector_joints_num", dexhand_ros_actuator_->get_hand_joints_num());
            end_effector_joints_num_set = true;
        }

        if (hardware_plant_->end_effector_type_ == "lejuclaw") {
            leju_claw_ctrl_srv_ = nh_.advertiseService("control_robot_leju_claw", &HardwareNode::controlLejuClawCallback, this);
            leju_claw_recovery_srv_ = nh_.advertiseService("control_robot_leju_claw_recovery", &HardwareNode::recoverLejuClawCallback, this);
            leju_claw_state_pub_ = nh_.advertise<kuavo_msgs::lejuClawState>("leju_claw_state", 10);
            leju_claw_command_sub_ = nh_.subscribe("leju_claw_command", 10, &HardwareNode::lejuClawCommandCallback, this);
            leju_claw_joy_sub_ = nh_.subscribe("/quest_joystick_data", 10, &HardwareNode::lejuClawJoystickCallback, this);
            leju_claw_state_timer_ = nh_.createTimer(ros::Duration(dt_),
                &HardwareNode::pubLejuClawStateTimerCallback, this);
            auto lejuclaw_state = hardware_plant_->getLejuClawState();
            nh_.setParam("end_effector_joints_num", static_cast<int>(lejuclaw_state.state.size()));
            end_effector_joints_num_set = true;
        }

        if (hardware_plant_->end_effector_type_ == "qiangnao_touch") {
            auto gesture_filepath = hardware_plant_->gesture_filepath_;
            dexhand_ros_actuator_ = std::make_shared<eef_controller::DexhandRosNode>();
            // TODO: qiangnao_touch 触觉手暂时固定使用 485 协议 (is_can_protocol = false)
            if(hardware_plant_->dexhand_actuator && !dexhand_ros_actuator_->init(nh_, gesture_filepath, kDexhandStatePublishFrequency, hardware_plant_->dexhand_actuator, true, false)) {
                std::cerr << "[HardwareNode] Failed to initialize." << std::endl;
            }
            nh_.setParam("end_effector_joints_num", dexhand_ros_actuator_->get_hand_joints_num());
            end_effector_joints_num_set = true;
        }

        if (hardware_plant_->end_effector_type_ == "revo2") {
            auto gesture_filepath = hardware_plant_->gesture_filepath_;
            revo2_dexhand_ros_actuator_ = std::make_shared<eef_controller::Revo2DexhandRosNode>();
            if(hardware_plant_->revo2_actuator && !revo2_dexhand_ros_actuator_->init(nh_, gesture_filepath, kDexhandStatePublishFrequency, hardware_plant_->revo2_actuator, is_can_protocol)) {
                std::cerr << "[HardwareNode] Failed to initialize Revo2DexhandRosNode." << std::endl;
            }
            nh_.setParam("end_effector_joints_num", revo2_dexhand_ros_actuator_->get_hand_joints_num());
            end_effector_joints_num_set = true;
        }

        sensor_thread_running_ = true;
        sensor_data_pub_ = nh_.advertise<kuavo_msgs::sensorsData>("/sensors_data_raw", 10);
        sensor_thread_ = std::thread(&HardwareNode::sensorThreadFunc, this, std::ref(sensor_data_pub_));
        if (!sensor_thread_.joinable())
        {
            std::cout << "sensor_thread_ joinable failed!!\r\n";
            exit(1);
        }
        logger_ptr_->startThread();
        set_logger_callback(std::bind(&ocs2::humanoid::TopicLogger::publishVectorCallback, logger_ptr_, std::placeholders::_1, std::placeholders::_2));
        set_EClogger_callback(std::bind(&ocs2::humanoid::TopicLogger::publishStdVectorCallback, logger_ptr_, std::placeholders::_1, std::placeholders::_2));

        if (!end_effector_joints_num_set) {
            nh_.setParam("end_effector_joints_num", 0);
        }

        if (hardware_plant_->end_effector_type_ == "heiman" &&
            !end_effector_joints_num_set && !shutdownRequested()) {
            bool retry = true;
            double interval = 5.0;
            nh_.param("sg100_init_retry", retry, true);
            nh_.param("sg100_init_retry_interval", interval, 5.0);
            if (!std::isfinite(interval) || interval < 0.1) {
                ROS_WARN("[HardwareNode] invalid sg100_init_retry_interval; using 5 seconds");
                interval = 5.0;
            }
            if (retry) {
                ROS_WARN("[HardwareNode] SG100 initialization will retry every %.1f s; "
                         "arm/chassis initialization is not restarted", interval);
                sg100_init_retry_.start(
                    std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                        std::chrono::duration<double>(interval)),
                    [this] { return tryInitializeSg100(); },
                    [this] { return shutdownRequested() ||
                                    sg100_shutdown_requested_.load() || !ros::ok(); });
            }
        }
    }

    void HardwareNode::publishLejuClawDebug(const std::string& debug_msg)
    {
        if (!leju_claw_debug_pub_) {
            return;
        }
        std_msgs::String msg;
        msg.data = debug_msg;
        leju_claw_debug_pub_.publish(msg);
    }

    void HardwareNode::publishLejuClawTarget(const std::vector<std::string>& name,
                                              const std::vector<double>& position,
                                              const std::vector<double>& velocity,
                                              const std::vector<double>& effort,
                                              const std::vector<int>& kp,
                                              const std::vector<int>& kd,
                                              bool zero_kp)
    {
        if (!leju_claw_target_pub_) {
            return;
        }

        kuavo_msgs::lejuClawTarget msg;
        msg.header.stamp = ros::Time::now();
        msg.header.frame_id = "leju_claw";
        msg.data.name = name;
        msg.data.position = position;
        msg.data.velocity = velocity;
        msg.data.effort = effort;
        msg.kp.reserve(kp.size());
        for (int value : kp) {
            msg.kp.push_back(zero_kp ? 0 : value);
        }
        msg.kd.reserve(kd.size());
        for (int value : kd) {
            msg.kd.push_back(value);
        }
        msg.zero_kp = zero_kp;
        leju_claw_target_pub_.publish(msg);
    }

    void HardwareNode::lejuClawJoystickCallback(const kuavo_msgs::JoySticks::ConstPtr&)
    {
        auto command_lease = command_gate_.enter();
        if (!command_lease.owns_lock()) return;
        const auto now = std::chrono::steady_clock::now().time_since_epoch();
        last_vr_joystick_us_.store(
            std::chrono::duration_cast<std::chrono::microseconds>(now).count());
    }

    bool HardwareNode::getLejuClawHighFreq()
    {
        bool manual_switch = false;
        ros::param::param("/lejuclaw_high_freq_switch", manual_switch, false);
        if (manual_switch) {
            return true;
        }

        int timeout_ms = kLejuClawVrStreamTimeoutMs;
        ros::param::param("/lejuclaw_vr_stream_timeout_ms", timeout_ms,
                          static_cast<int>(kLejuClawVrStreamTimeoutMs));
        if (timeout_ms <= 0) {
            timeout_ms = kLejuClawVrStreamTimeoutMs;
        }

        const int64_t last_us = last_vr_joystick_us_.load();
        if (last_us == 0) {
            return false;
        }
        const auto now = std::chrono::steady_clock::now().time_since_epoch();
        const int64_t now_us =
            std::chrono::duration_cast<std::chrono::microseconds>(now).count();
        return now_us - last_us <= static_cast<int64_t>(timeout_ms) * 1000;
    }

    bool HardwareNode::notifyLejuClawModeSwitch(bool high_freq)
    {
        const bool previous = last_high_freq_.exchange(high_freq);
        if (previous == high_freq) {
            return false;
        }
        ROS_WARN("[LejuClaw] control mode switched to %s frequency%s.",
                 high_freq ? "high" : "low",
                 high_freq ? "" : " (VR joystick stream inactive or timed out)");
        return true;
    }

    void HardwareNode::reStartCallback(const std_msgs::Bool::ConstPtr &msg)
    {
        auto command_lease = command_gate_.enter();
        if (!command_lease.owns_lock()) return;
        // if (1 != hardware_status_)
        // {
        //     std::cout << "first start cmd use real_init_wait fun." << std::endl;
        //     return;
        // }
        if (msg->data)
        {
            std::cout << "/re_start_robot received!\n";
            ros::param::set(kHardwareStateParam, kHardwareRunning);
            ros::param::set("/hardware/is_ready", 1);
            publishRobotDisabledFlag(false);
        }
    }

    void HardwareNode::getBotStandUpCompleteCallback(const std_msgs::Int8::ConstPtr &msg)
    {
        bot_stand_up_complete_ = msg->data;
        std::cout << "recvice bot stand up status msg: " << static_cast<int>(bot_stand_up_complete_) << std::endl;
    }

    
    bool HardwareNode::controlLejuClawCallback(kuavo_msgs::controlLejuClawRequest &req,
                          kuavo_msgs::controlLejuClawResponse &res)
    {
        auto command_lease = command_gate_.enter();
        if (!command_lease.owns_lock()) return false;
        if (!hardware_plant_->checkLejuClawInitialized()) {
            res.success = false;
            res.message = "leju claw actuator not initialized.";
            return false;
        }

        // 转换ROS请求到底层控制器请求
        eef_controller::ControlClawRequest controller_req;
        eef_controller::ControlClawResponse controller_res;
        
        // 复制数据
        controller_req.data.name = req.data.name;
        controller_req.data.position = req.data.position;
        controller_req.data.velocity = req.data.velocity;
        controller_req.data.effort = req.data.effort;

        {
            std::stringstream ss;
            ss << "{\"source\":\"hardware_node\",\"event\":\"service_command\","
               << "\"name_count\":" << req.data.name.size() << ","
               << "\"position_count\":" << req.data.position.size() << ","
               << "\"velocity_count\":" << req.data.velocity.size() << ","
               << "\"effort_count\":" << req.data.effort.size() << "}";
            publishLejuClawDebug(ss.str());
        }

        // 调用底层控制器
        const bool is_high_freq = getLejuClawHighFreq();
        notifyLejuClawModeSwitch(is_high_freq);
        bool result = hardware_plant_->controlLejuClaw(
            controller_req, controller_res, is_high_freq);
        
        // 转换响应
        res.success = controller_res.success;
        res.message = controller_res.message;
        
        return result;
    }

    static std::string escapeJsonString(const std::string& input)
    {
        std::ostringstream oss;
        for (char ch : input) {
            switch (ch) {
                case '\\':
                    oss << "\\\\";
                    break;
                case '"':
                    oss << "\\\"";
                    break;
                case '\n':
                    oss << "\\n";
                    break;
                case '\r':
                    oss << "\\r";
                    break;
                case '\t':
                    oss << "\\t";
                    break;
                default:
                    oss << ch;
                    break;
            }
        }
        return oss.str();
    }

    bool HardwareNode::recoverLejuClawCallback(kuavo_msgs::lejuClawRecoveryRequest &req,
                          kuavo_msgs::lejuClawRecoveryResponse &res)
    {
        auto command_lease = command_gate_.enter();
        if (!command_lease.owns_lock()) return false;
        if (!hardware_plant_->checkLejuClawInitialized()) {
            res.success = false;
            res.message = "leju claw actuator not initialized.";
            return false;
        }

        {
            std::stringstream ss;
            ss << "{\"source\":\"hardware_node\",\"event\":\"recovery_request\","
               << "\"direction\":\"" << escapeJsonString(req.direction) << "\","
               << "\"current\":" << req.current << ","
               << "\"duration_ms\":" << req.duration_ms << ","
               << "\"repeat\":" << req.repeat << "}";
            publishLejuClawDebug(ss.str());
        }

        std::string message;
        bool success = hardware_plant_->recoverLejuClaw(
            req.direction, req.current, req.duration_ms, req.repeat, message);
        res.success = success;
        res.message = message;

        {
            std::stringstream ss;
            ss << "{\"source\":\"hardware_node\",\"event\":\"recovery_response\","
               << "\"success\":" << (success ? "true" : "false") << ","
               << "\"message\":\"" << escapeJsonString(message) << "\"}";
            publishLejuClawDebug(ss.str());
        }

        return true;
    }

    void HardwareNode::pubLejuClawStateTimerCallback(const ros::TimerEvent& event)
    {
        auto command_lease = command_gate_.enter();
        if (!command_lease.owns_lock()) return;
        if (!hardware_plant_->checkLejuClawInitialized()) {
            return;
        }

        const bool effective_high_freq = getLejuClawHighFreq();
        notifyLejuClawModeSwitch(effective_high_freq);

        // 获取夹爪状态
        auto state = hardware_plant_->getLejuClawState();
        
        // 创建并发布ROS消息
        kuavo_msgs::lejuClawState msg;
        msg.header.stamp = ros::Time::now();
        msg.header.frame_id = "leju_claw";
        msg.data.name = state.data.name;
        msg.data.position = state.data.position;
        msg.data.velocity = state.data.velocity;
        msg.data.effort = state.data.effort;
        msg.state = state.state;
        msg.is_high_freq = state.is_high_freq;

        leju_claw_state_pub_.publish(msg);
    }

    void HardwareNode::lejuClawCommandCallback(const kuavo_msgs::lejuClawCommand::ConstPtr &msg)
    {
        auto command_lease = command_gate_.enter();
        if (!command_lease.owns_lock()) return;
        eef_controller::lejuClawCommand command;
        command.name = msg->data.name;
        command.position = msg->data.position;
        command.velocity = msg->data.velocity;
        command.effort = msg->data.effort;
        {
            std::stringstream ss;
            ss << "{\"source\":\"hardware_node\",\"event\":\"topic_command\","
               << "\"name_count\":" << msg->data.name.size() << ","
               << "\"position_count\":" << msg->data.position.size() << ","
               << "\"velocity_count\":" << msg->data.velocity.size() << ","
               << "\"effort_count\":" << msg->data.effort.size() << "}";
            publishLejuClawDebug(ss.str());
        }
        const bool is_high_freq = getLejuClawHighFreq();
        notifyLejuClawModeSwitch(is_high_freq);
        hardware_plant_->controlLejuClaw(command, is_high_freq);
    }

    bool HardwareNode::changeMotorParamCallback(kuavo_msgs::changeMotorParamRequest &req, kuavo_msgs::changeMotorParamResponse &res)
    {
        auto command_lease = command_gate_.enter();
        if (!command_lease.owns_lock()) return false;
        std::vector<MotorParam> motor_params;
        for(auto &param : req.data) {
            if(std::find_if(motor_params.begin(), motor_params.end(), 
                [&param](const MotorParam& m) { return m.id == param.id; }) == motor_params.end()) {
                motor_params.push_back(MotorParam{static_cast<uint16_t>(param.id), param.Kp, param.Kd});
            } else {
                // 参数中不要有重复的电机ID
                res.message = "Duplicate motor ID " + std::to_string(param.id) + " found in parameters";
                res.success = true;
                ROS_WARN("%s", res.message.c_str());
                return true;
            }
        }
        
        if(motor_params.empty()) {
            res.message = "no motor param to change";
            res.success = true;
                ROS_WARN("%s", res.message.c_str());
            return true;
        }

        std::string err_msg;
        bool result = hardware_plant_->changeMotorParam(motor_params, err_msg);
        if(!result) {
            res.message = err_msg;
            ROS_ERROR("change motor param failed: %s", err_msg.c_str());
        } else {
            res.message = "success";
        }
        res.success = result;
        return true;
    }

    bool HardwareNode::getMotorParamCallback(kuavo_msgs::getMotorParamRequest &req, kuavo_msgs::getMotorParamResponse &res)
    {
        auto command_lease = command_gate_.enter();
        if (!command_lease.owns_lock()) return false;
        std::vector<MotorParam> motor_params;
        std::string err_msg;
        bool result = hardware_plant_->getMotorParam(motor_params, err_msg);
        
        if(!result) {
            res.message = err_msg;
            ROS_ERROR("get motor param failed: %s", err_msg.c_str());
        } else {
            for(auto &param : motor_params) {
                kuavo_msgs::motorParam motor_param;
                motor_param.id = param.id;
                motor_param.Kp = param.Kp;
                motor_param.Kd = param.Kd;
                res.data.push_back(motor_param);
            }
            res.message = "success";
        }

        res.success = result;
        return true;
    }

    bool HardwareNode::realLaunchStatusCallback(std_srvs::Trigger::Request &request, std_srvs::Trigger::Response &response)
    {
        auto command_lease = command_gate_.enter();
        if (!command_lease.owns_lock()) return false;
        // 站立成功
        if(bot_stand_up_complete_ == 1) {
            response.message = "launched"; // 站立成功
            response.success = true;
            return true;
        }

        // -3 zero_point_calibration
        // -2 cali_leg
        // -1 calibrate
        // 0  ready_stance
        // 1 站立
        // 零点标定模式

        // cali_set_zero=true时，始终机器人始终为zero_point_cali
        if (cali_zero_point_mode){
            response.message = "zero_point_cali";
            response.success = true;
            return true;
        }


        // -3 (kStatusNotInAnyState) 表示尚未就绪，走兜底返回 "initing"；
        // 真正的零点校准场景由上方 cali_zero_point_mode 检查覆盖

        // 准备姿态:缩腿准备站立
        if (hardware_plant_->hardware_status_ == 0) {
            response.message = "ready_stance";
            response.success = true;
            return true;
        }

        //  校准状态:腿绷直准备下蹲
        if (hardware_plant_->hardware_status_ == -1) {
            response.message = "calibrate";
            response.success = true;
            return true;
        }

        // 初始化中
        response.message = "initing";
        response.success = true;
        return true;
    }

bool HardwareNode::switchRobotStateCallback(kuavo_msgs::robotSwitchPose::Request &req, kuavo_msgs::robotSwitchPose::Response &res)
    {
        auto command_lease = command_gate_.enter();
        if (!command_lease.owns_lock()) return false;
        auto robot_status = req.status;

        if (robot_status == "cali_zero") {

            // 零点标定模式
            std::cout << "[HardwareNode] Moving to cali_zero position..." << std::endl;

            zero_point_status_swith = true;
            hardware_plant_->is_cali_set_zero_ = true;

            hardware_plant_->initial_input_cmd_ = 'z';
            hardware_plant_->cali_set_zero_status = 1;
            hardware_plant_->hardware_status_ = -3;

            res.success = true;
            res.message = "Robot cali zero up (z command)";
            
        } else if (robot_status == "ready") {
            // 'o' 命令效果 - 移动到准备姿态
            std::cout << "[HardwareNode] Moving to ready position..." << std::endl;
            zero_point_status_swith = true;
            hardware_plant_->is_cali_set_zero_ = false;

            hardware_plant_->cali_set_zero_status = 0;
            hardware_plant_->initial_input_cmd_ = 'o';

            res.success = true;
            res.message = "Robot standing up (o command)";
            
        } else if (robot_status == "stand") {
            // 'q' 命令效果 - 返回校准姿态
            std::cout << "[HardwareNode] Moving to calibration position..." << std::endl;
            zero_point_status_swith = true;

            hardware_plant_->is_cali_set_zero_ = false;
            hardware_plant_->cali_set_zero_status = 1;
            hardware_plant_->initial_input_cmd_ = 'e';
            
            res.success = true;
            res.message = "Robot standing up (q command)";

        } else if  (robot_status == "exit"){

            std::cout << "[HardwareNode] Exit calibration mode..." << std::endl;
            zero_point_status_swith = true;
            cali_zero_point_mode = false;
            hardware_plant_->initial_input_cmd_ = 'e';

            res.success = true;
            res.message = "Robot standing up (exit command)";

        }
        else {
            // 其他状态不支持切换
            res.success = false;
            res.message = "Unsupported robot status: " + robot_status + 
                         ". Supported statuses: ready, calibrate_leg, calibrate_arm, self_check, stand, help";
        }
        
        return true;
    }

    bool HardwareNode::adjustZeroPointCallback(kuavo_msgs::adjustZeroPoint::Request &req,
                                             kuavo_msgs::adjustZeroPoint::Response &res)
    {
        auto command_lease = command_gate_.enter();
        if (!command_lease.owns_lock()) return false;
        if (hardware_plant_->hardware_ready_)
        {         
            ROS_INFO("[HardwareNode] adjustZeroPoint require robot at ready pose before push 'o', now robot pose is after push 'o', please restart robot!");
            res.success = false;
            res.message = "adjustZeroPoint require robot at ready pose before push 'o', now robot pose is after push 'o', please restart robot!";
            return false;
        }

        int motor_index = req.motor_index;
        double adjust_pos = req.offset;  // degree

        if (motor_index >= hardware_plant_->get_motor_info().driver.size() || motor_index < 0) {
            ROS_INFO("[HardwareNode] require motor_index < %d, while recv motor_index is %d !, value mismatch!", 
                    (int)hardware_plant_->get_motor_info().driver.size(), motor_index);
            res.success = false;
            res.message = "require motor_index < " + std::to_string(hardware_plant_->get_motor_info().driver.size()) + 
                         ", while recv motor_index is " + std::to_string(motor_index) + " !, value mismatch!";
            return false;
        }

        int ruiwo_motor_index = 0;
        auto motor_info = hardware_plant_->get_motor_info();
        if (motor_info.driver[motor_index] == EC_MASTER){
            std::vector<double> offset;
            getMotorPositionOffset(offset);
            offset[motor_index] = adjust_pos;
            setMotorPositionOffset(offset);
        }
        else if (motor_info.driver[motor_index] == RUIWO){
            if(nullptr != hardware_plant_->getRuiwoActuator()){

                // 将电机索引转换为RUIWO电机索引
                for(int i = 0; i < motor_index; i++) {
                    if(motor_info.driver[i] == RUIWO){
                        ruiwo_motor_index++;
                    }
                }

                adjust_pos = adjust_pos * TO_RADIAN;
                hardware_plant_->getRuiwoActuator()->adjustZeroPosition(ruiwo_motor_index, adjust_pos);
            }
        }
        else {
            ROS_INFO("[HardwareNode] unsupport motor driver type: %d ", motor_info.driver[motor_index]);
            res.success = false;
            res.message = "unsupport motor driver type: " + std::to_string(motor_info.driver[motor_index]);
            return false;   
        }

        std::vector<double> moving_pos(num_joint, 0);
        double cali_tau_limit = 3;

        std::fill(moving_pos.begin(), moving_pos.end(), 0.0);
        if(hardware_plant_->is_cali_set_zero_){
            int num_leg = num_joint - hardware_plant_->num_arm_joints - hardware_plant_->num_waist_joints - hardware_plant_->num_head_joints;
            moving_pos[hardware_plant_->num_waist_joints + num_leg] = 90;
            moving_pos[hardware_plant_->num_waist_joints + num_leg + hardware_plant_->num_arm_joints / 2] = -90;
        }
        jointMoveTo(moving_pos, jointMoveSpeed_, dt_, cali_tau_limit);

        res.success = true;
        res.message = "adjust zero point success";

        return true;
    }

    bool HardwareNode::getHardwareReadyCallback(std_srvs::Trigger::Request &req,
                                std_srvs::Trigger::Response &res)
    {
        auto command_lease = command_gate_.enter();
        if (!command_lease.owns_lock()) return false;
        res.success = hardware_plant_->cali_set_zero_status == 0;
        res.message = hardware_plant_->cali_set_zero_status == 0 ? "Hardware is ready" : "Hardware is not ready";
        return true;
    }

    bool HardwareNode::getMotorZeroPointsCallback(kuavo_msgs::getMotorZeroPoints::Request &req,
                                            kuavo_msgs::getMotorZeroPoints::Response &res)
    {
        auto command_lease = command_gate_.enter();
        if (!command_lease.owns_lock()) return false;
        // 腿部 offset - 直接读取 offset.csv
        size_t ec_count = hardware_plant_->getCountECMasters();
        std::vector<double> offset = readOffsetFromFile(ec_count);
        offset.resize(ec_count);

        // 手臂零点 - 直接读取 arms_zero.yaml
        std::vector<double> zero_points = readArmsZeroFromFile();
        offset.insert(offset.end(), zero_points.begin(), zero_points.end());

        res.zero_points = offset;
        res.success = true;
        res.message = "Successfully retrieved motor zero points from config files.";
        return true;
    }

}; // namespace HighlyDynamic
