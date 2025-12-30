#include <ros/ros.h>
#include <std_srvs/Trigger.h>
#include <string>
#include <vector>
#include "kuavo_msgs/questJoySticks.h"
#include <string>

#include <ros/init.h>
#include <ros/package.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>
#include <geometry_msgs/Twist.h>

#include <ocs2_core/Types.h>
#include <ocs2_core/misc/LoadData.h>
#include <ocs2_msgs/mpc_observation.h>

#include <ocs2_ros_interfaces/command/TargetTrajectoriesRosPublisher.h>
#include <humanoid_interface/gait/ModeSequenceTemplate.h>
#include "humanoid_interface_ros/gait/ModeSequenceTemplateRos.h"
#include "std_srvs/Trigger.h"
#include <std_msgs/Bool.h>
#include "humanoid_interface_drake/humanoid_interface_drake.h"

#include <kuavo_msgs/changeArmCtrlMode.h>

namespace ocs2
{
    enum ArmTarget
    {
    TARGET_NONE = 0,
    TARGET_SQUAT = 1,
    TARGET_STAND = 2,
    TARGET_DEFAULT = 3,
    };



    class QuestControlFSM 
    {
    public:
        QuestControlFSM(ros::NodeHandle &nodeHandle, const std::string &robotName, bool verbose = false) :
            state_("STAND"),
            arm_control_enabled_(false),
            arm_control_previous_(false),
            mode_changed_(false),
            last_execution_time_(ros::Time(0)),
            last_height_change_time_(ros::Time(0)),
            update_interval_(0.1),
            last_update_time_(ros::Time::now()),
            targetPoseCommand_(nodeHandle, robotName) 
        {
            cmdVel_.linear.x = 0;
            cmdVel_.linear.y = 0;
            cmdVel_.linear.z = 0;
            cmdVel_.angular.x = 0;
            cmdVel_.angular.y = 0;
            cmdVel_.angular.z = 0;
            // Get node parameters
            std::string referenceFile;
            nodeHandle.getParam("/referenceFile", referenceFile);

            // loadData::loadCppDataType(referenceFile, "comHeight", com_height_);
            RobotVersion robot_version(3, 4);
            if (nodeHandle.hasParam("/robot_version"))
            {
                int version_num;
                nodeHandle.getParam("/robot_version", version_num);
                int major = version_num / 10;
                int minor = version_num % 10;
                robot_version = RobotVersion(major, minor);
            }

            auto drake_interface_ = HighlyDynamic::HumanoidInterfaceDrake::getInstancePtr(robot_version, true, 2e-3);
           
            default_joint_state_ = drake_interface_->getDefaultJointState();
            com_height_ = drake_interface_->getIntialHeight();
            only_half_up_body_ = drake_interface_->getKuavoSettings().running_settings.only_half_up_body;

            loadData::loadCppDataType(referenceFile, "targetRotationVelocity", target_rotation_velocity_);
            loadData::loadCppDataType(referenceFile, "targetDisplacementVelocity", target_displacement_velocity_);
            loadData::loadCppDataType(referenceFile, "cmdvelLinearXLimit", c_relative_base_limit_[0]);
            loadData::loadCppDataType(referenceFile, "cmdvelAngularYAWLimit", c_relative_base_limit_[3]);

            loadData::loadEigenMatrix(referenceFile, "standBaseState", stand_base_state_);
            loadData::loadEigenMatrix(referenceFile, "standJointState", stand_arm_state_);

            loadData::loadEigenMatrix(referenceFile, "squatBaseState", squat_base_state_);
            loadData::loadEigenMatrix(referenceFile, "squatJointState", squat_arm_state_);
            loadData::loadCppDataType(referenceFile, "armMode", armMode_);

            // gait
            std::string gaitCommandFile;
            nodeHandle.getParam("/gaitCommandFile", gaitCommandFile);
            ROS_INFO_STREAM(robotName + "_mpc_mode_schedule node is setting up ...");
            std::vector<std::string> gaitList;
            loadData::loadStdVector(gaitCommandFile, "list", gaitList, verbose);
            gait_map_.clear();
            for (const auto &gaitName : gaitList)
            {
                gait_map_.insert({gaitName, humanoid::loadModeSequenceTemplate(gaitCommandFile, gaitName, verbose)});
            }

            mode_sequence_template_publisher_ = nodeHandle.advertise<ocs2_msgs::mode_schedule>(robotName + "_mpc_mode_schedule", 10, true);
            mode_scale_publisher_ = nodeHandle.advertise<std_msgs::Float32>(robotName + "_mpc_mode_scale", 10, true);

            joystick_sub_ = nodeHandle_.subscribe("/quest_joystick_data", 1, &QuestControlFSM::joystickCallback, this);
            observation_sub_ = nodeHandle_.subscribe(robotName + "_mpc_observation", 10, &QuestControlFSM::observationCallback, this);
            stop_pub_ = nodeHandle_.advertise<std_msgs::Bool>("/stop_robot", 10);
            step_num_stop_pub_ = nodeHandle_.advertise<std_msgs::Int32>(robotName + "_mpc_stop_step_num", 10, true);
            vel_control_pub_ = nodeHandle_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);

            change_arm_mode_service_client_ = nodeHandle_.serviceClient<kuavo_msgs::changeArmCtrlMode>("/humanoid_change_arm_ctrl_mode");
            change_arm_mode_service_VR_client_ = nodeHandle_.serviceClient<kuavo_msgs::changeArmCtrlMode>("/change_arm_ctrl_mode");
            
            get_arm_mode_service_client_ = nodeHandle_.serviceClient<kuavo_msgs::changeArmCtrlMode>("/humanoid_get_arm_ctrl_mode");
            command_height_ = 0.0;
            command_add_height_pre_ = 0.0;

            arm_mode_pub_ = nodeHandle_.advertise<std_msgs::Int32>("/quest3/triger_arm_mode", 1);
        }

        void run()
        {
            ros::Rate rate(100);
            while (ros::ok())
            {
                ros::spinOnce();
                rate.sleep();
                if (!get_observation_)
                {
                // ROS_INFO_STREAM("Waiting for observation message...");
                continue;
                }
                // checkAndPublishCommandLine(joystick_origin_axis_);
            }
            return;
        }

        void callRealInitializeSrv()
        {
            ros::ServiceClient client = nodeHandle_.serviceClient<std_srvs::Trigger>("/humanoid_controller/real_initial_start");
            std_srvs::Trigger srv;

            // 调用服务
            if (client.call(srv))
            {
                ROS_INFO("RealInitializeSrv call successful");
            }
            else
            {
                ROS_ERROR("Failed to call RealInitializeSrv");
            }
        }
        int callGetArmModeSrv()
        {
            kuavo_msgs::changeArmCtrlMode srv;
            srv.request.control_mode = 0;

            // 调用服务
            if (get_arm_mode_service_client_.call(srv))
            {
                ROS_INFO("callGetArmModeSrv call successful");
                return srv.response.mode;
            }
            else
            {
                ROS_ERROR("Failed to call callGetArmModeSrv");
            } 
            return -1;
            
        }
        void callSetArmModeSrv(int32_t mode)
        {
            kuavo_msgs::changeArmCtrlMode srv;
            srv.request.control_mode = mode;

            // 调用服务
            if (change_arm_mode_service_client_.call(srv))
            {
                ROS_INFO("SetArmModeSrv call successful");
                // 发布当前手臂模式
                std_msgs::Int32 arm_mode_msg;
                arm_mode_msg.data = mode;
                arm_mode_pub_.publish(arm_mode_msg);
            }
            else
            {
                ROS_ERROR("Failed to call SetArmModeSrv");
            }
        }

        void callVRSetArmModeSrv(int32_t mode)
        {
            kuavo_msgs::changeArmCtrlMode srv;
            srv.request.control_mode = mode;

            // 调用服务
            if (change_arm_mode_service_VR_client_.call(srv))
            {
                ROS_INFO("SetArmModeSrv call successful");
                // 发布当前手臂模式
                std_msgs::Int32 arm_mode_msg;
                arm_mode_msg.data = mode;
                arm_mode_pub_.publish(arm_mode_msg);
            }
            else
            {
                ROS_ERROR("Failed to call SetArmModeSrv");
            }
        }

        void callTerminateSrv()
        {
        std::cout << "tigger callTerminateSrv" << std::endl;
        for (int i = 0; i < 5; i++)
        {
            std_msgs::Bool msg;
            msg.data = true;
            stop_pub_.publish(msg);
            ::ros::Duration(0.1).sleep();
        }
        }

    private:
        void joystickCallback(const kuavo_msgs::questJoySticks::ConstPtr& msg) 
        {
            joystick_data_ = *msg;
            updateState();
            joystick_data_prev_ = joystick_data_;

        }

        void observationCallback(const ocs2_msgs::mpc_observation::ConstPtr &observation_msg)
        {
        observation_ = ros_msg_conversions::readObservationMsg(*observation_msg);
        get_observation_ = true;
        }

        void updateState()
        {
            if (!rec_joystick_data_)
            {
                joystick_data_prev_ = joystick_data_;
                rec_joystick_data_ = true;
                return;
            }

            if (!get_observation_ && !joystick_data_prev_.right_first_button_pressed && joystick_data_.right_first_button_pressed)
            {
                callRealInitializeSrv();
                return;
            }
            
            if (joystick_data_.left_first_button_pressed && joystick_data_.left_second_button_pressed) // 左边第一二个按钮同时按下，关闭机器人
            {
                  callTerminateSrv();
                  return;
            }
            if (joystick_data_.left_first_button_pressed) // 左边第一个按钮按下了，切换模式
            {
                if (!joystick_data_prev_.right_second_button_pressed && joystick_data_.right_second_button_pressed) // 关闭手臂控制、自动摆手
                {
                    callSetArmModeSrv(0);
                    current_arm_mode_ = 0;
                }
                else if (!joystick_data_prev_.right_first_button_pressed && joystick_data_.right_first_button_pressed) // 启用手臂控制
                {
                    current_arm_mode_ = (current_arm_mode_!=1) ? 1 : 2;
                    std::cout << "[QuestControlFSM] change arm mode to :" << current_arm_mode_ << std::endl;
                    if (only_half_up_body_) {
                        callVRSetArmModeSrv(current_arm_mode_);
                    }
                    else {
                        callSetArmModeSrv(current_arm_mode_);
                    }
                }

                return;
            }
            
            if (!only_half_up_body_) {
                // 全身控制时才支持步态控制
                checkGaitSwitchCommand(joystick_data_);
                updateCommandLine();
            }
            return;
            std::string new_state = state_;
            if (state_ == "STAND") {
                if (joystick_data_.right_second_button_pressed) new_state = "WALK";
                else if (joystick_data_.left_second_button_pressed) new_state = "WALK_STEP_MODE";
            } else if (state_ == "WALK") {
                if (joystick_data_.right_first_button_pressed) new_state = "STAND";
                else if (joystick_data_.left_second_button_pressed) new_state = "WALK_SPEED_MODE";
            } else if (state_ == "WALK_SPEED_MODE") {
                if (joystick_data_.right_first_button_pressed) new_state = "STAND";
                else if (joystick_data_.left_first_button_pressed) new_state = "WALK_POSITION_MODE";
            } else if (state_ == "WALK_POSITION_MODE") {
                if (joystick_data_.right_first_button_pressed) new_state = "STAND";
                else if (joystick_data_.left_second_button_pressed) new_state = "WALK_SPEED_MODE";
            } else if (state_ == "WALK_STEP_MODE") {
                if (joystick_data_.right_first_button_pressed || joystick_data_.left_first_button_pressed) new_state = "STAND";
            }

            if (new_state != state_) {
                state_ = new_state;
                mode_changed_ = false; // 重置模式切换标志
                last_execution_time_ = ros::Time(0); // 重置最后执行时间
                last_height_change_time_ = ros::Time(0);
            }

            executeStateActions();
            joystick_data_prev_ = joystick_data_;
        }

        void publish_zero_spd()
        {
            geometry_msgs::Twist cmdVel;
            cmdVel.linear.x = 0.0;
            cmdVel.linear.y = 0.0;
            cmdVel.linear.z = 0.0;
            cmdVel.angular.z = 0.0;
            vel_control_pub_.publish(cmdVel);
        }

        void updateCommandLine()
        {
            const std::vector<float> deadzone = {0.02f, 0.02f, 0.02f, 0.02f};
            auto joystick_vector = getJoystickVector(deadzone);
            if (joystick_vector[0] < 0.0)
            {
                joystick_vector[0] *= 0.5;// 后退灵敏度减弱50%
            }
            bool cmd_close_to_zero = (joystick_vector.norm() <= 1e-3);
            if (cmd_close_to_zero)
            {
                if(!last_cmd_close_to_zero_)
                {
                    publish_zero_spd();
                    last_cmd_close_to_zero_ = true;
                }
                return;
            }
            last_cmd_close_to_zero_ = cmd_close_to_zero;

            Eigen::VectorXd select_vector(4);
            select_vector<<1,0,0,1;
            if (joystick_data_.left_second_button_touched && joystick_data_.left_first_button_touched)
            {
                select_vector << 0, 0, 1, 0;
            }
            if (joystick_data_.right_second_button_touched && joystick_data_.right_first_button_touched)
            {
                select_vector << 0, 1, 0, 0;
            }
            // std::cout << "joycmd: " << joystick_vector.transpose() << std::endl;
            Eigen::VectorXd limit_vector(4);
            limit_vector << c_relative_base_limit_[0], c_relative_base_limit_[1], c_relative_base_limit_[2], c_relative_base_limit_[3];

            commad_line_target_.head(4) = joystick_vector.cwiseProduct(limit_vector).cwiseProduct(select_vector);

            cmdVel_.linear.x = commad_line_target_(0);
            cmdVel_.linear.y = commad_line_target_(1);
            cmdVel_.linear.z = commad_line_target_(2);
            cmdVel_.angular.z = commad_line_target_(3);
            vel_control_pub_.publish(cmdVel_);
        }

        void checkGaitSwitchCommand(const kuavo_msgs::questJoySticks &joy_msg)
        {
            // 检查是否有gait切换指令
            if (!joystick_data_prev_.right_first_button_pressed && joy_msg.right_first_button_pressed)
            {
                publish_mode_sequence_temlate("stance");
                publish_zero_spd();
            }

            else if (!joystick_data_prev_.right_second_button_pressed && joy_msg.right_second_button_pressed)
            {
                publish_mode_sequence_temlate("walk");
            }
            else
            {
                return;
            }

            std::cout << "joycmd switch to: " << current_desired_gait_ << std::endl;
            std::cout << "turn " << (current_desired_gait_ == "stance" ? "on " : "off ") << " auto stance mode" << std::endl;
        }

        void executeStateActions() 
        {
            ros::Time current_time = ros::Time::now();
            if (current_time - last_update_time_ >= update_interval_) {
                if (state_ == "WALK") {
                    walk();
                } else if (state_ == "STAND") {
                    stand();
                } else if (state_ == "WALK_SPEED_MODE") {
                    walkSpeedMode();
                } else if (state_ == "WALK_POSITION_MODE") {
                    walkPositionMode();
                } else if (state_ == "WALK_STEP_MODE") {
                    walkStepMode();
                } else {
                    ROS_WARN("Unknown state: %s", state_.c_str());
                    return;
                }
                last_update_time_ = current_time;
            }
            if (state_ != "STAND")
            {
                heightCommand();
            }
        }

        void walk() {
            if (!mode_changed_) {
                mode_changed_ = true;
                arm_target_nums_ = 2; // 站立
                publish_mode_sequence_temlate("trot2");   // "walk"
                sendWalkCommand(1, {0.0, 0.0, 0.0});
                ROS_INFO("Mode changed to: %s", state_.c_str());
            }
        }

        void stand() {
            if (!mode_changed_) {
                mode_changed_ = true;
                arm_target_nums_ = 2; // 站立
                publish_mode_sequence_temlate("stance");    // 
                ROS_INFO("Mode changed to: %s", state_.c_str());
            }
        }

        void walkSpeedMode() {
            if (!mode_changed_) {
                mode_changed_ = true;
                publish_mode_sequence_temlate("walk");   // "walk"

                ROS_INFO("Mode changed to: %s", state_.c_str());
            }
            auto values_n = joystickNorm();
            auto values = getWalkValue(values_n, {0.2f, 0.1f, 8.0f});
            sendWalkCommand(1, values);
            previous_value_m_ = values_n;
        }

        void walkPositionMode() {
            if (!mode_changed_) {
                mode_changed_ = true;
                publish_mode_sequence_temlate("walk");   // "walk"

                ROS_INFO("Mode changed to: %s", state_.c_str());
            }
            auto values_n = joystickNorm();
            if (isValueChanged(values_n) || canExecuteCommand()) {
                auto values = getWalkValue(values_n, {0.2f, 0.1f, 8.0f});
                if (std::all_of(values.begin(), values.end(), [](float v) { return v == 0.0f; })) return;
                sendWalkCommand(0, values);
                last_execution_time_ = ros::Time::now();
                previous_value_m_ = values_n;
            }
        }

        void walkStepMode() {
            if (!mode_changed_) {
                mode_changed_ = true;
                publish_mode_sequence_temlate("stance");   // "step walk command"

                ROS_INFO("Mode changed to: %s", state_.c_str());
            }
            auto values_n = joystickNorm();
            if (isValueChanged(values_n) || canExecuteCommand()) {
                auto values = getWalkValue(values_n, {0.2f, 0.1f, 8.0f});
                if (std::all_of(values.begin(), values.end(), [](float v) { return v == 0.0f; })) return;
                sendWalkCommand(2, {values[0], values[1], values[2]});
                last_execution_time_ = ros::Time::now();
                previous_value_m_ = values_n;
            }
        }

        void heightCommand(){
            double command_add_height_ = 0.0;
            if(joystick_data_.left_y >= 0.8) 
            {
                command_add_height_ = 0.05;
            } 
            else if (joystick_data_.left_y <= -0.8)
            {
                command_add_height_ = -0.05;
            }
            // 判断是否变化，动作执行时间是否足够

            if (canHeightChangeCommand() || command_add_height_!= command_add_height_pre_) 
            {
                command_add_height_pre_ = command_add_height_;
                command_height_ += command_add_height_;
                ROS_INFO("Height Change!  Comand_height : %.2f", command_height_);
                last_height_change_time_ = ros::Time::now();
            }

            return;
        }

        bool canHeightChangeCommand(double interval = 1.5) {
            if (last_height_change_time_.isZero()) return false;
            ros::Time current_time = ros::Time::now();
            if ((current_time - last_height_change_time_).toSec() >= interval) {
                last_height_change_time_ = current_time;
                ROS_INFO("Execution interval reached, applying command.");
                return true;
            }
            return false;
        }


        void sendWalkCommand(int control_mode, const std::vector<float>& values) {
            ROS_INFO("Walk command sent: mode=%d, values=%.2f, %.2f, %.2f", control_mode, values[0], values[1], values[2]);

            const vector_t currentPose = observation_.state.segment<6>(6);
            const vector_t currentArmPose = observation_.state.segment<14>(12+12);
            TargetTrajectories target_traj;
            double dx = values[0] * cos(currentPose(3)) - values[1] * sin(currentPose(3));
            double dy = values[0] * sin(currentPose(3)) + values[1] * cos(currentPose(3));
            current_target_(0) = currentPose(0) + dx;
            current_target_(1) = currentPose(1) + dy;
            current_target_(2) = com_height_ + command_height_;
            current_target_(3) = currentPose(3) + values[2] * M_PI / 180.0;
            current_target_(4) = 6 * M_PI / 180.0; // fixed value，因为存在静差
            current_target_(5) = 0.0;
            const vector_t targetPose = current_target_;
            const vector_t targetPoseForArm = [&]()
            {
                vector_t target = targetPose;
                if(armMode_){
                    switch(arm_target_nums_){
                        case TARGET_SQUAT: 
                            target(0) += squat_base_state_(0);
                            target(1) += squat_base_state_(1);
                            target(2) = squat_base_state_(2);
                            break;
                        case TARGET_STAND:
                            target(0) += stand_base_state_(0);
                            target(1) += stand_base_state_(1);
                            target(2) = stand_base_state_(2);
                        case TARGET_NONE:
                            target(2) = com_height_;
                            break;
                    }
                }
                return target;
            }();
            // target reaching duration
            const scalar_t targetReachingTime = observation_.time + estimateTimeToTarget(targetPose - currentPose);

            // desired time trajectory
            const scalar_array_t timeTrajectory{observation_.time, targetReachingTime, targetReachingTime + 1.0};

            // desired state trajectory
            vector_array_t stateTrajectory(3, vector_t::Zero(observation_.state.size() + 1));
            stateTrajectory[0] << vector_t::Zero(6), currentPose, default_joint_state_, currentArmPose, armMode_;
            stateTrajectory[1] << vector_t::Zero(6), targetPose, default_joint_state_, currentArmPose, armMode_;
            stateTrajectory[2] << vector_t::Zero(6), targetPoseForArm, default_joint_state_, currentArmPose, armMode_;

            switch (arm_target_nums_){
                case TARGET_SQUAT: stateTrajectory[2] << vector_t::Zero(6), targetPoseForArm, default_joint_state_, squat_arm_state_, armMode_; break;
                case TARGET_STAND: stateTrajectory[2] << vector_t::Zero(6), targetPoseForArm, default_joint_state_, stand_arm_state_, armMode_; break;
                case TARGET_DEFAULT: stateTrajectory[2] << vector_t::Zero(6), targetPoseForArm, default_joint_state_, vector_t::Zero(14), armMode_; break;
            }
            // desired input trajectory (just right dimensions, they are not used)
            const vector_array_t inputTrajectory(3, vector_t::Zero(observation_.input.size()));
            target_traj = {timeTrajectory, stateTrajectory, inputTrajectory};

            // 0 位置  1 速度  2  单步
            // This function will later be implemented with the proper service call
            if (control_mode == 0)
            {
                targetPoseCommand_.publishTargetTrajectories(target_traj);
            }
            else if (control_mode == 1)
            {
                geometry_msgs::Twist vel;
                vel.linear.x = values[0];
                vel.linear.y = values[1];
                vel.linear.z = command_height_;
                vel.angular.z = 3.14 * values[2] / 180.0;
                vel_control_pub_.publish(vel);
            }
            else if (control_mode == 2)
            {
                std_msgs::Int32 stop_step_num;
                stop_step_num.data = 3;
                step_num_stop_pub_.publish(stop_step_num);
                // targetPoseCommand_.publishTargetTrajectories(target_traj);
                geometry_msgs::Twist vel;
                vel.linear.x = values[0];
                vel.linear.y = values[1];
                vel.linear.z = command_height_;
                vel.angular.z = 3.14 * values[2] / 180.0;
                vel_control_pub_.publish(vel);
                publish_mode_sequence_temlate("walk");   // "walk"

            }
            else 
            {
                ROS_INFO("Control mode %d  is a wrong Number mode!", control_mode);
            }
        }
        vector_t getJoystickVector(const std::vector<float> &deadzone = {0.02f, 0.2f, 0.2f, 0.02f})
        {
            vector_t values_norm = vector_t::Zero(4);
            std::vector<float> values_raw{joystick_data_.left_y, -joystick_data_.left_x, joystick_data_.right_y, -joystick_data_.right_x};
            
            for (int i = 0; i < 4; ++i)
            {
                float value = values_raw[i];
                if (std::abs(value) > deadzone[i])
                {
                    values_norm[i] = std::copysign(1, value) * (std::abs(value) - deadzone[i]) / (1.0 - deadzone[i]);
                }
            }

            return values_norm;
        }
        std::vector<int> joystickNorm()
        {
            std::vector<int> values_norm(3, 0);
            if (joystick_data_.right_y >= 0.8) values_norm[0] = 1;
            else if (joystick_data_.right_y <= -0.8) values_norm[0] = -1;
            if (joystick_data_.right_x >= 0.8) values_norm[1] = -1;
            else if (joystick_data_.right_x <= -0.8) values_norm[1] = 1;
            if (joystick_data_.left_x >= 0.8) values_norm[2] = -1;
            else if (joystick_data_.left_x <= -0.8) values_norm[2] = 1;
            return values_norm;
        }

        std::vector<float> getWalkValue(const std::vector<int>& values_norm, const std::vector<float>& w) {
            std::vector<float> value(values_norm.size());
            for (size_t i = 0; i < values_norm.size(); ++i) {
                value[i] = w[i] * values_norm[i];
            }
            return value;
        }

        bool isValueChanged(const std::vector<int>& values_norm) {
            if (previous_value_m_.empty()) return true;
            for (size_t i = 0; i < values_norm.size(); ++i) {
                if (previous_value_m_[i] != values_norm[i]) return true;
            }
            return false;
        }

        bool canExecuteCommand(double interval = 1.5) {
            if (last_execution_time_.isZero()) return false;
            ros::Time current_time = ros::Time::now();
            if ((current_time - last_execution_time_).toSec() >= interval) {
                last_execution_time_ = current_time;
                ROS_INFO("Execution interval reached, applying command.");
                return true;
            }
            return false;
        }

        void publish_mode_sequence_temlate(const std::string &gaitName)
        {   //切换步态
            humanoid::ModeSequenceTemplate modeSequenceTemplate = gait_map_.at(gaitName);
            mode_sequence_template_publisher_.publish(createModeSequenceTemplateMsg(modeSequenceTemplate));
            current_desired_gait_ = gaitName;            
        }

        scalar_t estimateTimeToTarget(const vector_t &desiredBaseDisplacement)
        {
        const scalar_t &dx = desiredBaseDisplacement(0);
        const scalar_t &dy = desiredBaseDisplacement(1);
        const scalar_t &dz = desiredBaseDisplacement(2);
        const scalar_t &dyaw = desiredBaseDisplacement(3);
        const scalar_t rotationTime = std::abs(dyaw) / target_rotation_velocity_;
        const scalar_t displacement = std::sqrt(dx * dx + dy * dy + dz * dz);
        const scalar_t displacementTime = displacement / target_displacement_velocity_;
        return std::max(rotationTime, displacementTime);
        }

        ros::NodeHandle nodeHandle_;
        TargetTrajectoriesRosPublisher targetPoseCommand_;
        ros::Subscriber observation_sub_;
        bool get_observation_ = false;
        vector_t current_target_ = vector_t::Zero(6);
        scalar_t target_displacement_velocity_;
        scalar_t target_rotation_velocity_;
        scalar_t com_height_;
        vector_t default_joint_state_ = vector_t::Zero(12);
        vector_t commad_line_target_ = vector_t::Zero(6);
        vector_t stand_base_state_ = vector_t::Zero(6);
        vector_t stand_arm_state_ = vector_t::Zero(14);

        vector_t squat_base_state_ = vector_t::Zero(6);
        vector_t squat_arm_state_ = vector_t::Zero(14);

        int arm_target_nums_ = 0;
        bool arm_target_flag_ = false;
        bool armMode_ = true;

        std::string current_desired_gait_;
        ocs2::scalar_array_t c_relative_base_limit_{0.4, 0.2, 0.3, 0.4};
        ocs2::SystemObservation observation_;
        ros::Publisher mode_sequence_template_publisher_;
        ros::Publisher mode_scale_publisher_;
        ros::Publisher stop_pub_;
        ros::Publisher step_num_stop_pub_;
        ros::Publisher vel_control_pub_;

        geometry_msgs::Twist cmdVel_;

        ros::ServiceClient change_arm_mode_service_client_;
        ros::ServiceClient change_arm_mode_service_VR_client_;
        ros::ServiceClient get_arm_mode_service_client_;

        int current_arm_mode_{2};

        float total_mode_scale_{1.0};

        std::map<std::string, humanoid::ModeSequenceTemplate> gait_map_;

        ros::Subscriber joystick_sub_;
        std::string state_;
        kuavo_msgs::questJoySticks joystick_data_;
        kuavo_msgs::questJoySticks joystick_data_prev_;
        bool mode_changed_;
        ros::Duration update_interval_;
        ros::Time last_update_time_;
        std::vector<int> previous_value_m_;
        bool arm_control_enabled_;
        bool arm_control_previous_;
        ros::Time trigger_start_time_;
        ros::Time grip_start_time_;
        ros::Time last_execution_time_;
        ros::Time last_height_change_time_;
        bool is_velControl;
        bool rec_joystick_data_{false};

        double command_height_; // 高度单独控制
        double command_add_height_pre_;

        bool last_cmd_close_to_zero_{true};
        bool only_half_up_body_{false};

        ros::Publisher arm_mode_pub_;
    };
}

int main(int argc, char** argv) {
    const std::string robotName = "humanoid";

    // Initialize ros node
    ::ros::init(argc, argv, robotName + "_quest_command_node");
    ::ros::NodeHandle nodeHandle;
    ocs2::QuestControlFSM quest_control_fsm(nodeHandle, robotName);
    quest_control_fsm.run();
    return 0;
}
