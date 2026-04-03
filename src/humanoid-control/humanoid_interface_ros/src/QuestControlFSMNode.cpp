#include <ros/ros.h>
#include <std_srvs/Trigger.h>
#include <string>
#include <vector>
#include "kuavo_msgs/JoySticks.h"
#include <string>
#include <std_msgs/String.h>

#include <ros/init.h>
#include <ros/package.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64MultiArray.h>

#include <ocs2_core/Types.h>
#include <ocs2_core/misc/LoadData.h>
#include <ocs2_msgs/mpc_observation.h>

#include <ocs2_ros_interfaces/command/TargetTrajectoriesRosPublisher.h>
#include <humanoid_interface/gait/ModeSequenceTemplate.h>
#include "humanoid_interface_ros/gait/ModeSequenceTemplateRos.h"
#include "std_srvs/Trigger.h"
#include <std_srvs/SetBool.h>
#include <std_msgs/Bool.h>
#include "humanoid_interface_drake/humanoid_interface_drake.h"

#include <kuavo_msgs/changeArmCtrlMode.h>
#include <kuavo_msgs/changeTorsoCtrlMode.h>
#include <kuavo_msgs/robotWaistControl.h>
#include <kuavo_msgs/headBodyPose.h>
#include <kuavo_msgs/footPose.h>
#include <kuavo_msgs/footPoseTargetTrajectories.h>
#include <kuavo_msgs/getCurrentGaitName.h>
#include <std_srvs/Trigger.h>
#include "utils/singleStepControl.hpp"
#include <kuavo_msgs/switchToNextController.h>
#include <kuavo_msgs/headBodyPose.h>
#include <geometry_msgs/PoseStamped.h>

namespace ocs2
{
    enum ArmTarget
    {
    TARGET_NONE = 0,
    TARGET_SQUAT = 1,
    TARGET_STAND = 2,
    TARGET_DEFAULT = 3,
    };

    // 创建足部轨迹消息的辅助函数（使用HumanoidControl单步控制工具）
    // body_pose： [x(m), y(m), z(m), yaw(deg)]
    kuavo_msgs::footPoseTargetTrajectories CreateFootPoseTrajectory(const std::vector<Eigen::Vector4d>& body_poses) {
        // 使用HumanoidControl的get_multiple_steps_msg函数生成轨迹
        // 参数：身体姿态序列，时间步长，脚步间距，碰撞检测
        return HumanoidControl::get_multiple_steps_msg(body_poses, 0.4, 0.15, true);
    }

    // 单步转向区间定义
    struct TurnStepZone {
        float min_value;     // 摇杆区间最小值
        float max_value;     // 摇杆区间最大值
        kuavo_msgs::footPoseTargetTrajectories trajectory;  // 足部轨迹
    };

    // 60度轨迹数据（已移除，但保留注释）
    //         {0.75f, 1.0f, CreateFootPoseTrajectory(0.117f, 0.08f, -1.047f, -0.057f, -0.02f, -1.047f)}, // 60度右转
    //         {0.75f, 1.0f, CreateFootPoseTrajectory(-0.057f, 0.08f, 1.047f, 0.117f, -0.02f, 1.047f)}  // 60度左转

    class QuestControlFSM 
    {
    public:
        QuestControlFSM(ros::NodeHandle &nodeHandle, const std::string &robotName, bool verbose = false) :
            state_("STAND"),
            arm_control_enabled_(false),
            arm_control_previous_(false),
            mode_changed_(false),
            vr_torso_arm_locked_(false),
            last_execution_time_(ros::Time(0)),
            last_height_change_time_(ros::Time(0)),
            update_interval_(0.1),
            last_update_time_(ros::Time::now()),
            targetPoseCommand_(nodeHandle, robotName),
            torso_control_enabled_(false),
            torso_yaw_zero_(0.0),
            body_height_zero_(0.0),
            torso_control_start_time_(ros::Time(0)),
            hand_wrench_enabled_(false),
            current_hand_wrench_item_mass_(0.0),
            current_hand_wrench_left_force_{0.0, 0.0, 0.0},
            current_hand_wrench_right_force_{0.0, 0.0, 0.0}
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
            std::cout << "get referenceFile: " << referenceFile << std::endl;

            // loadData::loadCppDataType(referenceFile, "comHeight", com_height_);
            RobotVersion rb_version(3, 4);
            if (nodeHandle.hasParam("/robot_version"))
            {
                int rb_version_int;
                nodeHandle.getParam("/robot_version", rb_version_int);
                rb_version = RobotVersion::create(rb_version_int);
                robot_version_int_ = rb_version_int;  // 保存版本号
            }
            
            // 获取机器人类型
            if (nodeHandle.hasParam("/robot_type"))
            {
                nodeHandle.getParam("/robot_type", robot_type_);
                std::cout << "Robot type: " << robot_type_ << " (1 for wheel robot, 0 for biped)" << std::endl;
                
                if(1 == robot_type_)
                {
                    get_observation_ = true;
                }
            }
            nodeHandle.param("/wheel_ik", wheel_ik_, false);
            ROS_INFO_STREAM("[QuestControlFSM] wheel_ik: " << (wheel_ik_ ? "true" : "false"));
            auto drake_interface_ = HighlyDynamic::HumanoidInterfaceDrake::getInstancePtr(rb_version, true, 2e-3);
            auto kuavo_settings = drake_interface_->getKuavoSettings();
            waist_dof_ = kuavo_settings.hardware_settings.num_waist_joints;
            default_joint_state_ = drake_interface_->getDefaultJointState();
            if (useLegacyWheelVr()) {
                // 兼容旧版轮臂逻辑：避免无腿关节模型调用 getIntialHeight() 引发异常
                try {
                    loadData::loadCppDataType(referenceFile, "comHeight", com_height_);
                } catch (...) {
                    com_height_ = 0.0;
                }
            } else {
                com_height_ = drake_interface_->getIntialHeight();
            }
            only_half_up_body_ = drake_interface_->getKuavoSettings().running_settings.only_half_up_body;
            std::cout << "only_half_up_body: " << only_half_up_body_ << std::endl;
            if(nodeHandle.hasParam("/only_half_up_body"))
            {
                nodeHandle.getParam("/only_half_up_body", only_half_up_body_);
            }

            loadData::loadCppDataType(referenceFile, "targetRotationVelocity", target_rotation_velocity_);
            loadData::loadCppDataType(referenceFile, "targetDisplacementVelocity", target_displacement_velocity_);
            loadData::loadCppDataType(referenceFile, "cmdvelLinearXLimit", c_relative_base_limit_[0]);
            loadData::loadCppDataType(referenceFile, "cmdvelLinearYLimit", c_relative_base_limit_[1]);
            loadData::loadCppDataType(referenceFile, "cmdvelAngularYAWLimit", c_relative_base_limit_[3]);

            // Load VR control limits
            loadData::loadCppDataType(referenceFile, "vrSquatMinPitchDeg", vr_squat_min_pitch_deg_);
            loadData::loadCppDataType(referenceFile, "vrSquatMaxPitchDeg", vr_squat_max_pitch_deg_);
            loadData::loadCppDataType(referenceFile, "vrSquatHeightMin", vr_squat_height_min_);
            loadData::loadCppDataType(referenceFile, "vrSquatHeightMax", vr_squat_height_max_);
            
            // 加载腰部最大旋转角度
            try {
                loadData::loadCppDataType(referenceFile, "waist_yaw_max", waist_yaw_max_angle_deg_);
            } catch (const std::exception &e) {
                ROS_WARN_STREAM("waist_yaw_max not found, using default: " << waist_yaw_max_angle_deg_);
            }
            
            loadData::loadEigenMatrix(referenceFile, "standBaseState", stand_base_state_);
            loadData::loadEigenMatrix(referenceFile, "standJointState", stand_arm_state_);

            loadData::loadEigenMatrix(referenceFile, "squatBaseState", squat_base_state_);
            loadData::loadEigenMatrix(referenceFile, "squatJointState", squat_arm_state_);
            loadData::loadCppDataType(referenceFile, "armMode", armMode_);

            // 加载转向区间配置
            loadTurnZones(nodeHandle);

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
            
            mode_sequence_template_publisher_ = nodeHandle_.advertise<ocs2_msgs::mode_schedule>(robotName + "_mpc_mode_schedule", 10, true);
            mode_scale_publisher_ = nodeHandle_.advertise<std_msgs::Float32>(robotName + "_mpc_mode_scale", 10, true);
            gait_name_publisher_ = nodeHandle_.advertise<std_msgs::String>("/humanoid_mpc_gait_name_request", 10, true);

            // 从主控制器实时订阅当前手臂控制模式
            arm_ctrl_mode_vr_sub_ = nodeHandle_.subscribe<std_msgs::Float64MultiArray>(
            "/humanoid/mpc/arm_control_mode", 1, &QuestControlFSM::armCtrlModeCallback, this); 

            joystick_sub_ = nodeHandle_.subscribe("/quest_joystick_data", 1, &QuestControlFSM::joystickCallback, this);
            observation_sub_ = nodeHandle_.subscribe(robotName + "_mpc_observation", 10, &QuestControlFSM::observationCallback, this);
            stop_pub_ = nodeHandle_.advertise<std_msgs::Bool>("/stop_robot", 10);
            step_num_stop_pub_ = nodeHandle_.advertise<std_msgs::Int32>(robotName + "_mpc_stop_step_num", 10, true);
            vel_control_pub_ = nodeHandle_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);

            std::string change_arm_mode_service_name = robot_type_ == 1 ? "/wheel_arm_change_arm_ctrl_mode" : "/humanoid_change_arm_ctrl_mode";
            change_arm_mode_service_client_ = nodeHandle_.serviceClient<kuavo_msgs::changeArmCtrlMode>(change_arm_mode_service_name);
           
            change_arm_mode_service_VR_client_ = nodeHandle_.serviceClient<kuavo_msgs::changeArmCtrlMode>("/change_arm_ctrl_mode");
            
            get_arm_mode_service_client_ = nodeHandle_.serviceClient<kuavo_msgs::changeArmCtrlMode>("/humanoid_get_arm_ctrl_mode");
            whole_torso_ctrl_pub_ = nodeHandle_.advertise<std_msgs::Bool>("/vr_whole_torso_ctrl", 1);

            cmd_torso_pose_pub_ = nodeHandle_.advertise<geometry_msgs::PoseStamped>("/cmd_torso_pose_vr", 1);
            
            // 添加 enable_wbc_arm_trajectory_control 服务客户端
            enable_wbc_arm_trajectory_control_client_ = nodeHandle_.serviceClient<kuavo_msgs::changeArmCtrlMode>("/enable_wbc_arm_trajectory_control");
            
            // VR腰部控制动态Q矩阵服务客户端
            vr_waist_control_service_client_ = nodeHandle_.serviceClient<std_srvs::SetBool>("/humanoid/mpc/vr_waist_control");
            
            // GaitReceiver自动步态模式服务客户端
            auto_gait_mode_service_client_ = nodeHandle_.serviceClient<std_srvs::SetBool>(robotName + "_auto_gait");

            // MPC控制模式切换服务客户端
            control_mode_client_ = nodeHandle_.serviceClient<kuavo_msgs::changeTorsoCtrlMode>("/mobile_manipulator_mpc_control");
            
            // 腰部控制相关的订阅者和发布者
            head_body_pose_sub_ = nodeHandle_.subscribe("/kuavo_head_body_orientation_data", 1, &QuestControlFSM::headBodyPoseCallback, this);
            waist_motion_pub_ = nodeHandle_.advertise<kuavo_msgs::robotWaistControl>("/robot_waist_motion_data", 1);
            cmd_pose_pub_ = nodeHandle_.advertise<geometry_msgs::Twist>("/cmd_pose", 1);
            
            // 末端力配置订阅器和发布器
            hand_wrench_config_sub_ = nodeHandle_.subscribe("/quest3/hand_wrench_config", 1, &QuestControlFSM::handWrenchConfigCallback, this);
            hand_wrench_cmd_pub_ = nodeHandle_.advertise<std_msgs::Float64MultiArray>("/hand_wrench_cmd", 1);
            
            command_height_ = 0.0;
            command_add_height_pre_ = 0.0;

            arm_mode_pub_ = nodeHandle_.advertise<std_msgs::Int32>("/quest3/triger_arm_mode", 1);

            // 添加足部轨迹发布者
            foot_pose_target_pub_ = nodeHandle_.advertise<kuavo_msgs::footPoseTargetTrajectories>("/humanoid_mpc_foot_pose_target_trajectories", 10);

            // 添加查询当前步态服务客户端
            get_current_gait_service_client_ = nodeHandle_.serviceClient<std_srvs::SetBool>("/humanoid_get_current_gait");
            get_current_gait_name_service_client_ = nodeHandle_.serviceClient<kuavo_msgs::getCurrentGaitName>("/humanoid_get_current_gait_name");

            // 添加arm_collision_control服务
            arm_collision_control_service_ = nodeHandle_.advertiseService("/quest3/set_arm_collision_control", &QuestControlFSM::armCollisionControlCallback, this);
            
            // 添加切换控制器服务客户端
            switch_to_next_controller_client_ = nodeHandle_.serviceClient<kuavo_msgs::switchToNextController>("/humanoid_controller/switch_to_next_controller");
            
            // 订阅RL控制器状态话题
            is_rl_controller_sub_ = nodeHandle_.subscribe<std_msgs::Float64>("/humanoid_controller/is_rl_controller_", 1, &QuestControlFSM::isRlControllerCallback, this);
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

                        // 调用服务
            if (change_arm_mode_service_VR_client_.call(srv))
            {
                ROS_INFO("/change_arm_ctrl_mode call successful");
                // 发布当前手臂模式
                std_msgs::Int32 arm_mode_msg;
                arm_mode_msg.data = mode;
                arm_mode_pub_.publish(arm_mode_msg);
            }
            else
            {
                ROS_ERROR("Failed to call /change_arm_ctrl_mode");
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

        void callEnableWbcArmTrajectorySrv(int32_t enable)
        {
            kuavo_msgs::changeArmCtrlMode srv;
            srv.request.control_mode = enable;

            // 调用服务
            if (enable_wbc_arm_trajectory_control_client_.call(srv))
            {
                ROS_INFO("EnableWbcArmTrajectorySrv call successful, enabled: %s", enable ? "true" : "false");
            }
            else
            {
                ROS_ERROR("Failed to call EnableWbcArmTrajectorySrv");
            }
        }

        void callVRWaistControlSrv(bool enable)
        {
            std_srvs::SetBool srv;
            srv.request.data = enable;

            // 等待服务可用
            if (!vr_waist_control_service_client_.waitForExistence(ros::Duration(2.0)))
            {
                ROS_WARN("VR waist control service not available, skipping call");
                return;
            }

            // 调用服务
            if (vr_waist_control_service_client_.call(srv))
            {
                if (srv.response.success)
                {
                    ROS_INFO("VRWaistControlSrv call successful: %s, response: %s", 
                             enable ? "enabled" : "disabled", srv.response.message.c_str());
                }
                else
                {
                    ROS_WARN("VRWaistControlSrv returned failure: %s", srv.response.message.c_str());
                }
            }
            else
            {
                ROS_ERROR("Failed to call VRWaistControlSrv");
            }
        }

        void callWheelMpcControlMode(int control_mode)
        {
            kuavo_msgs::changeTorsoCtrlMode srv;
            srv.request.control_mode = control_mode;

            // 等待服务可用
            if (!control_mode_client_.waitForExistence(ros::Duration(2.0)))
            {
                ROS_WARN("MPC control mode service not available, skipping call");
                return;
            }

            // 调用服务
            if (control_mode_client_.call(srv))
            {
                if (srv.response.result)
                {
                    ROS_INFO("WheelMpcControlMode call successful: control_mode=%d, mode=%d, response: %s", 
                             control_mode, srv.response.mode, srv.response.message.c_str());
                }
                else
                {
                    ROS_WARN("WheelMpcControlMode returned failure: control_mode=%d, response: %s", 
                             control_mode, srv.response.message.c_str());
                }
            }
            else
            {
                ROS_ERROR("Failed to call WheelMpcControlMode with control_mode=%d", control_mode);
            }
        }

        void callAutoGaitModeSrv(bool enable)
        {
            std_srvs::SetBool srv;
            srv.request.data = enable;

            // 等待服务可用
            if (!auto_gait_mode_service_client_.waitForExistence(ros::Duration(2.0)))
            {
                ROS_WARN("Auto gait mode service not available, skipping call");
                return;
            }

            // 调用服务
            if (auto_gait_mode_service_client_.call(srv))
            {
                if (srv.response.success)
                {
                    ROS_INFO("AutoGaitModeSrv call successful: %s, response: %s", 
                             enable ? "enabled" : "disabled", srv.response.message.c_str());
                }
                else
                {
                    ROS_WARN("AutoGaitModeSrv returned failure: %s", srv.response.message.c_str());
                }
            }
            else
            {
                ROS_ERROR("Failed to call AutoGaitModeSrv");
            }
        }

        void callSwitchToNextControllerSrv()
        {
            kuavo_msgs::switchToNextController srv;
            
            // 等待服务可用
            if (!switch_to_next_controller_client_.waitForExistence(ros::Duration(2.0)))
            {
                ROS_WARN("Switch to next controller service not available, skipping call");
                return;
            }

            if (useLegacyWheelVr()) {
                if (switch_to_next_controller_client_.call(srv))
                {
                    if (srv.response.success)
                    {
                        ROS_INFO("Switch to next controller successful: %s", srv.response.message.c_str());
                        ROS_INFO("Switched from %s (index: %d) to %s (index: %d)", 
                                 srv.response.current_controller.c_str(), srv.response.current_index,
                                 srv.response.next_controller.c_str(), srv.response.next_index);
                    }
                    else
                    {
                        ROS_WARN("Switch to next controller failed: %s", srv.response.message.c_str());
                    }
                }
                else
                {
                    ROS_ERROR("Failed to call switch to next controller service");
                }
                return;
            }

            // 设置控制器切换保护标志
            controller_switching_ = true;
            controller_switch_start_time_ = ros::Time::now();
            ROS_WARN("[QuestControlFSM] Controller switching started. VR joystick and button inputs will be disabled for %.1f seconds.", 
                     CONTROLLER_SWITCH_PROTECTION_TIME);

            // 调用服务
            if (switch_to_next_controller_client_.call(srv))
            {
                if (srv.response.success)
                {
                    ROS_INFO("Switch to next controller successful: %s", srv.response.message.c_str());
                    ROS_INFO("Switched from %s (index: %d) to %s (index: %d)", 
                             srv.response.current_controller.c_str(), srv.response.current_index,
                             srv.response.next_controller.c_str(), srv.response.next_index);
                }
                else
                {
                    ROS_WARN("Switch to next controller failed: %s", srv.response.message.c_str());
                }
            }
            else
            {
                ROS_ERROR("Failed to call switch to next controller service");
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

        bool armCollisionControlCallback(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res) {
            arm_collision_control_ = req.data;
            res.success = true;
            if (req.data) {
                callSetArmModeSrv(0);
                arm_ctrl_mode_ = 0;
            }
            res.message = "Arm collision control set to " + std::string(req.data ? "true" : "false");
            ROS_INFO("Arm collision control set to %s", req.data ? "true" : "false");
            return true;
        }

    private:
        // 从YAML文件加载转向区间配置
        void loadTurnZones(ros::NodeHandle& nodeHandle) {
            turn_zones_.clear();
            
            // 获取YAML文件路径
            std::string pkg_path = ros::package::getPath("humanoid_interface_ros");
            std::string yaml_file = pkg_path + "/config/step_turn.yaml";
            
            std::cout << "Loading turn zones from: " << yaml_file << std::endl;
            
            // 加载YAML文件到参数服务器
            std::string command = "rosparam load " + yaml_file + " /quest_turn_config";
            int result = system(command.c_str());
            if (result != 0) {
                ROS_WARN("Failed to load turn zones YAML file, using default configuration");
                loadDefaultTurnZones();
                return;
            }
            
            // 创建临时NodeHandle用于读取配置
            ros::NodeHandle config_nh("/quest_turn_config");
            
            // 读取转向区间数量
            int num_turn_zones = 0;
            if (!config_nh.getParam("turn_zones_count", num_turn_zones) || num_turn_zones <= 0) {
                ROS_WARN("Invalid or missing turn_zones_count, using default configuration");
                loadDefaultTurnZones();
                return;
            }
            
            std::cout << "Loading " << num_turn_zones << " turn zones from YAML file..." << std::endl;
            
            // 用于存储左转区间的临时数据
            struct LeftTurnZoneConfig {
                float min_value;
                float max_value;
                double body_x;
                double body_y;
                double body_z;
                double body_yaw;
            };
            std::vector<LeftTurnZoneConfig> left_turn_configs;
            
            // 读取每个左转区间的配置
            XmlRpc::XmlRpcValue left_turn_zones;
            if (!config_nh.getParam("left_turn_zones", left_turn_zones)) {
                ROS_WARN("Failed to read left_turn_zones, using default configuration");
                loadDefaultTurnZones();
                return;
            }
            
            if (left_turn_zones.getType() != XmlRpc::XmlRpcValue::TypeArray) {
                ROS_WARN("left_turn_zones is not an array, using default configuration");
                loadDefaultTurnZones();
                return;
            }
            
            for (int i = 0; i < left_turn_zones.size(); ++i) {
                LeftTurnZoneConfig config;

                XmlRpc::XmlRpcValue& zone = left_turn_zones[i];

                // 读取区间参数
                config.min_value = static_cast<double>(zone["min_value"]);
                config.max_value = static_cast<double>(zone["max_value"]);

                // 只读取 body_yaw，body_x 和 body_y 直接设为 0
                config.body_yaw = static_cast<double>(zone["body_yaw"]);
                config.body_x = 0.0;  // 直接设为0
                config.body_y = 0.0;  // 直接设为0
                config.body_z = 0.0;

                left_turn_configs.push_back(config);
                
                // std::cout << "Loaded left turn zone " << i << ": min=" << config.min_value 
                //           << ", max=" << config.max_value 
                //           << ", pose=[" << config.body_x << ", " << config.body_y 
                //           << ", " << config.body_z << ", " << config.body_yaw << "]" << std::endl;
            }
            
            // 为左转和右转预留空间
            turn_zones_.reserve(left_turn_configs.size() * 2);
            
            // 根据左转配置生成右转区间（正值范围）
            // 规则：左转的值取反，y和yaw值取反
            for (int i = left_turn_configs.size() - 1; i >= 0; --i) {
                const auto& left_config = left_turn_configs[i];
                
                TurnStepZone right_zone;
                // 右转区间：将左转的负值范围映射到正值范围
                right_zone.min_value = -left_config.max_value;
                right_zone.max_value = -left_config.min_value;
                
                // 右转姿态：x保持不变，y和yaw取反
                right_zone.trajectory = CreateFootPoseTrajectory({
                    Eigen::Vector4d(left_config.body_x, -left_config.body_y, 
                                   left_config.body_z, -left_config.body_yaw)
                });
                
                turn_zones_.push_back(right_zone);
                
                // std::cout << "Generated right turn zone " << (left_turn_configs.size() - 1 - i) 
                //           << ": min=" << right_zone.min_value 
                //           << ", max=" << right_zone.max_value 
                //           << ", pose=[" << left_config.body_x << ", " << -left_config.body_y 
                //           << ", " << left_config.body_z << ", " << -left_config.body_yaw << "]" << std::endl;
            }
            
            // 添加左转区间（负值范围）
            for (const auto& left_config : left_turn_configs) {
                TurnStepZone left_zone;
                left_zone.min_value = left_config.min_value;
                left_zone.max_value = left_config.max_value;
                
                // 左转姿态：直接使用配置值
                left_zone.trajectory = CreateFootPoseTrajectory({
                    Eigen::Vector4d(left_config.body_x, left_config.body_y, 
                                   left_config.body_z, left_config.body_yaw)
                });
                
                turn_zones_.push_back(left_zone);
            }
            
            std::cout << "Total turn zones loaded: " << turn_zones_.size() << std::endl;
        }
        
        // 加载默认转向区间配置（备用方案）
        void loadDefaultTurnZones() {
            turn_zones_.clear();
            
            std::cout << "Loading default turn zones configuration..." << std::endl;
            
            // 右转区间 (0.0~1.0)
            turn_zones_.push_back({0.0f, 0.15f, CreateFootPoseTrajectory({
                Eigen::Vector4d(-0.01, -0.01, 0.0, -10.0)
            })});
            turn_zones_.push_back({0.151f, 0.45f, CreateFootPoseTrajectory({
                Eigen::Vector4d(-0.03, -0.03, 0.0, -20.0)
            })});
            turn_zones_.push_back({0.451f, 0.75f, CreateFootPoseTrajectory({
                Eigen::Vector4d(-0.04, -0.04, 0.0, -30.0)
            })});
            turn_zones_.push_back({0.751f, 1.0f, CreateFootPoseTrajectory({
                Eigen::Vector4d(-0.06, -0.06, 0.0, -45.0)
            })});
            
            // 左转区间 (-1.0~0.0)
            turn_zones_.push_back({-0.15f, 0.0f, CreateFootPoseTrajectory({
                Eigen::Vector4d(-0.01, 0.01, 0.0, 10.0)
            })});
            turn_zones_.push_back({-0.45f, -0.15f, CreateFootPoseTrajectory({
                Eigen::Vector4d(-0.03, 0.03, 0.0, 20.0)
            })});
            turn_zones_.push_back({-0.75f, -0.451f, CreateFootPoseTrajectory({
                Eigen::Vector4d(-0.04, 0.04, 0.0, 30.0)
            })});
            turn_zones_.push_back({-1.0f, -0.751f, CreateFootPoseTrajectory({
                Eigen::Vector4d(-0.06, 0.06, 0.0, 45.0)
            })});
            
            std::cout << "Default turn zones loaded: " << turn_zones_.size() << std::endl;
        }

        void armCtrlModeCallback(const std_msgs::Float64MultiArray::ConstPtr &mode_msg)
        {
            if(mode_msg->data.size() >= 2)
            {
                arm_ctrl_mode_ = static_cast<int>(mode_msg->data[1]); // 兼容MPC发布 [current, desired]
                return;
            }
            if(mode_msg->data.size() == 1)
            {
                arm_ctrl_mode_ = static_cast<int>(mode_msg->data[0]); // 兼容轮臂发布 [current]
                return;
            }
            ROS_WARN_THROTTLE(2.0, "[QuestControlFSM] arm_control_mode is empty");
        }

        void joystickCallback(const kuavo_msgs::JoySticks::ConstPtr& msg) 
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

        void headBodyPoseCallback(const kuavo_msgs::headBodyPose::ConstPtr& msg)
        {
            // Apply pitch limits from config
            current_head_body_pose_ = *msg;
            current_head_body_pose_.body_pitch = std::max(vr_squat_min_pitch_deg_*M_PI/180.0, std::min(current_head_body_pose_.body_pitch, vr_squat_max_pitch_deg_*M_PI/180.0));

            // 在腰部控制模式下且没有XY按键摇杆控制时，发布VR腰部控制指令
            if (torso_control_enabled_)
            {
                // 计算相对于零点的腰部位置
                double current_yaw = current_head_body_pose_.body_yaw;
                
                // === 角度连续性处理：防止180°附近的跳变 ===
                double yaw_diff = current_yaw - last_body_yaw_;
                if (yaw_diff > M_PI) {
                    // 从负值跳到正值（例如从-π跳到+π），说明实际是继续向负方向转
                    accumulated_yaw_offset_ -= 2 * M_PI;
                } else if (yaw_diff < -M_PI) {
                    // 从正值跳到负值（例如从+π跳到-π），说明实际是继续向正方向转
                    accumulated_yaw_offset_ += 2 * M_PI;
                }
                last_body_yaw_ = current_yaw;
                
                // 计算连续的yaw值（加上累计偏移）
                double continuous_yaw = current_yaw + accumulated_yaw_offset_;
                double relative_yaw = continuous_yaw - torso_yaw_zero_;
                double current_height = current_head_body_pose_.body_height;
                double relative_height = current_height - body_height_zero_;  // 计算相对于零点的高度

                //轮臂躯干控制
                auto lb_body_pitch_diff = std::max(-15*M_PI/180.0, std::min((current_head_body_pose_.body_pitch - torso_pitch_zero_), 30*M_PI/180.0));
                Eigen::Quaterniond quat;
                quat = Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitX()) *  // roll = 0
                Eigen::AngleAxisd(lb_body_pitch_diff, Eigen::Vector3d::UnitY()) *
                Eigen::AngleAxisd(relative_yaw, Eigen::Vector3d::UnitZ());

                double current_x = current_head_body_pose_.body_x;
                double lb_relative_height = current_height - body_height_zero_;  // 计算相对于零点的高度
                lb_relative_height = std::max(-0.20, std::min(lb_relative_height, 0.3));  // 计算相对于零点的高度
                double lb_relative_x = 0.6 * std::max(-0.15, std::min((current_x - body_x_zero_), 0.5));  // 计算相对于零点的x
                geometry_msgs::PoseStamped cmd_torso_pose;
                cmd_torso_pose.pose.position.x = lb_relative_x;
                cmd_torso_pose.pose.position.y = 0.0;
                cmd_torso_pose.pose.position.z = lb_relative_height;
                cmd_torso_pose.pose.orientation.x = quat.x();
                cmd_torso_pose.pose.orientation.y = quat.y();
                cmd_torso_pose.pose.orientation.z = quat.z();
                cmd_torso_pose.pose.orientation.w = quat.w();
                cmd_torso_pose.header.stamp = ros::Time::now();
                cmd_torso_pose.header.frame_id = "torso_link";
                cmd_torso_pose_pub_.publish(cmd_torso_pose);

                if(1 == robot_type_) // 轮臂机器人
                {
                    return; // 轮臂机器人不进行VR腰部控制
                }

                // 腰部yaw控制（如果支持腰部自由度）
                if (waist_dof_ > 0)
                {
                    // 发布腰部控制指令
                    controlWaist(relative_yaw * 180.0 / M_PI); // 转换为角度
                }
                
                // Height control - apply limits from config
                relative_height = std::max(vr_squat_height_min_, std::min(relative_height, vr_squat_height_max_));
                geometry_msgs::Twist cmd_pose;
                cmd_pose.linear.x = 0.0;  // 基于当前位置的 x 方向值 (m)
                cmd_pose.linear.y = 0.0;  // 基于当前位置的 y 方向值 (m)
                cmd_pose.linear.z = relative_height;  // 相对高度
                cmd_pose.angular.x = 0.0;  // roll
                cmd_pose.angular.z = 0.0;  // # 基于当前位置旋转（偏航）的角度，单位为弧度 (radian)
                cmd_pose.angular.y = current_head_body_pose_.body_pitch;  // pitch

                cmd_pose_pub_.publish(cmd_pose);
                
                // 记录最后一次的相对高度和body_pitch
                last_relative_height_ = relative_height;
                last_body_pitch_ = current_head_body_pose_.body_pitch;
                
                // 根据msg的pose值设置base的高度参考，通过/cmd_pose发布
            }
        }

        void updateState()
        {
            // 检查并清除控制器切换保护标志
            if (controller_switching_)
            {
                double elapsed_time = (ros::Time::now() - controller_switch_start_time_).toSec();
                if (elapsed_time >= CONTROLLER_SWITCH_PROTECTION_TIME)
                {
                    controller_switching_ = false;
                    ROS_INFO("[QuestControlFSM] Controller switching protection ended. VR joystick and button inputs are now enabled.");
                }
            }
            
            // 动态读取control_torso参数，支持后续启动的launch文件设置参数
            if(nodeHandle_.hasParam("/control_torso"))
            {
                nodeHandle_.getParam("/control_torso", control_torso_);
            }

            if (!rec_joystick_data_)
            {
                joystick_data_prev_ = joystick_data_;
                rec_joystick_data_ = true;
                return;
            }

            if (useLegacyWheelVr()) {
                updateStateLegacyWheelVr();
                return;
            }
            
            // 如果正在切换控制器，禁用所有按键输入，仅允许按X和Y关闭机器人
            if (controller_switching_)
            {
                // 只允许安全相关的操作（如关闭机器人），其他操作都被禁用
                if (joystick_data_.left_first_button_pressed && joystick_data_.left_second_button_pressed) // 左边第一二个按钮同时按下，关闭机器人
                {
                    callTerminateSrv();
                    return;
                }
                // 其他所有按键和摇杆输入都被忽略
                return;
            }

            // 检测末端力控制按键组合
            if (joystick_data_.left_second_button_pressed) { // Y 按钮按下
                // Y + A: 施加末端力
                if (!joystick_data_prev_.right_first_button_pressed && joystick_data_.right_first_button_pressed) {
                    applyHandWrench();
                    joystick_data_prev_ = joystick_data_;
                    return;
                }
                // Y + B: 释放末端力
                if (!joystick_data_prev_.right_second_button_pressed && joystick_data_.right_second_button_pressed) {
                    releaseHandWrench();
                    joystick_data_prev_ = joystick_data_;
                    return;
                }
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
            if (joystick_data_.left_trigger > 0.5)
            {
                if (!joystick_data_prev_.left_first_button_pressed && joystick_data_.left_first_button_pressed)
                {
                    // 使能 WBC 手臂轨迹控制
                    callEnableWbcArmTrajectorySrv(1);
                    return;
                }
                // if (!joystick_data_prev_.right_first_button_pressed && joystick_data_.right_first_button_pressed)
                // {
                //     callSwitchToNextControllerSrv();
                //     return;
                // }
            }
            if (joystick_data_.left_grip > 0.5)
            {
                if (!joystick_data_prev_.left_first_button_pressed && joystick_data_.left_first_button_pressed)
                {
                    // 禁用 WBC 手臂轨迹控制
                    callEnableWbcArmTrajectorySrv(0);
                    return;
                }
            }

            if (joystick_data_.left_first_button_pressed) // 左边第一个按钮按下了，切换模式
            {
                if (!joystick_data_prev_.right_second_button_pressed && joystick_data_.right_second_button_pressed) // 关闭手臂控制、自动摆手
                {
                    if (robot_type_ == 2) // 人形机器人
                    {
                        auto new_arm_mode = (arm_ctrl_mode_!=0) ? 0 : 1;
                        callSetArmModeSrv(new_arm_mode);
                    }
                    else if (robot_type_ == 1) // 轮臂机器人,通过奇偶按键次数锁定/解锁
                    {
                        if (!vr_torso_arm_locked_) {
                            // 第一次按：锁定手臂
                            callSetArmModeSrv(0);
                            vr_torso_arm_locked_ = true;
                            ROS_INFO("[QuestControlFSM] VR torso mode: arm locked");
                        } else {
                            // 第二次按：解锁手臂，切换到BaseArm模式，允许底盘移动
                            callSetArmModeSrv(2);
                            vr_torso_arm_locked_ = false;
                            ROS_INFO("[QuestControlFSM] VR torso mode: arm unlocked");
                        }
                    }
                }
                else if (!joystick_data_prev_.right_first_button_pressed && joystick_data_.right_first_button_pressed) // 启用手臂控制
                {
                    // 如果手臂碰撞控制中，手臂正在回归，回归完成会切换到手臂 KEEP 模式，此时再按 XA 继续手臂跟踪 
                    if (arm_collision_control_) {
                        arm_collision_control_ = false;
                        return;
                    }
                    auto new_arm_mode = (arm_ctrl_mode_!=2) ? 2 : 1;
                    std::cout << "[QuestControlFSM] change arm mode to :" << new_arm_mode << std::endl;
                    if (only_half_up_body_) {
                        callVRSetArmModeSrv(new_arm_mode);
                    }
                    else {
                        callSetArmModeSrv(new_arm_mode);
                    }

                    if(1 == robot_type_) // 轮臂机器人
                    {
                        callWheelMpcControlMode(3);  // BaseArm mode
                    }
                }

                return;
            }
            
            
            // 腰部控制逻辑
            // if (joystick_data_.left_trigger > 0.5) // 左边扳机按下，进入腰部控制模式
            if ((joystick_data_.left_first_button_touched && joystick_data_.left_second_button_touched))
            {
                if (!joystick_data_prev_.right_second_button_pressed && joystick_data_.right_second_button_pressed) // 右边第二个按钮按下，切换腰部控制模式
                {
                    if (!torso_control_enabled_ && control_torso_)
                    {
                        // 启用腰部控制模式
                        torso_control_enabled_ = true;
                        torso_pitch_zero_= current_head_body_pose_.body_pitch;
                        torso_yaw_zero_ = current_head_body_pose_.body_yaw; // 记录当前腰部位置作为零点
                        torso_roll_zero_ = current_head_body_pose_.body_roll;

                        body_height_zero_ = current_head_body_pose_.body_height; // 记录当前高度作为零点
                        body_x_zero_ = current_head_body_pose_.body_x; // 记录当前x作为零点
                        torso_control_start_time_ = ros::Time::now();
                        
                        // 初始化角度连续性处理变量
                        last_body_yaw_ = current_head_body_pose_.body_yaw;
                        accumulated_yaw_offset_ = 0.0;
                        
                        std::cout << "腰部控制模式已启用，腰部零点yaw: " << torso_yaw_zero_  << ", 腰部零点pitch: " << torso_pitch_zero_ 
                                << "，高度零点: " << body_height_zero_ << ", x零点: " << body_x_zero_ << std::endl;
                        std_msgs::Bool whole_torso_ctrl_msg;
                        whole_torso_ctrl_msg.data = true;
                        whole_torso_ctrl_pub_.publish(whole_torso_ctrl_msg);

                        if(1 != robot_type_)
                        {
                            // 失能GaitReceiver的自动步态模式
                            callAutoGaitModeSrv(false);
                            // 调用VR腰部控制服务，启用VR腰部控制动态Q矩阵
                            callVRWaistControlSrv(true);
                        }

                    }
                    else
                    {
                        // 关闭腰部控制模式
                        torso_control_enabled_ = false;

                        std_msgs::Bool whole_torso_ctrl_msg;
                        whole_torso_ctrl_msg.data = false;
                        whole_torso_ctrl_pub_.publish(whole_torso_ctrl_msg);
                        std::cout << "腰部控制模式已关闭" << std::endl;

                        if(1 != robot_type_)
                        {
                            // 发送最后一帧，使用记录的relative_height和body_pitch
                            geometry_msgs::Twist cmd_pose;
                            cmd_pose.linear.x = 0.0;
                            cmd_pose.linear.y = 0.0;
                            cmd_pose.linear.z = last_relative_height_;  // 使用记录的相对高度
                            cmd_pose.angular.x = 0.0;
                            cmd_pose.angular.y = last_body_pitch_;      // 使用记录的body_pitch
                            cmd_pose.angular.z = 0.0;  // 基于当前位置旋转（偏航）的角度，单位为弧度 (radian)
                            cmd_pose_pub_.publish(cmd_pose);

                            
                            // 使能GaitReceiver的自动步态模式
                            callAutoGaitModeSrv(true);
                            // 调用VR腰部控制服务，禁用VR腰部控制动态Q矩阵
                            callVRWaistControlSrv(false);
                        }

                        std::cout << "腰部控制模式已关闭，发送最后一帧 - 相对高度: " << last_relative_height_ 
                                  << ", body_pitch: " << last_body_pitch_ << std::endl;
                    }
                    return;
                }
            }
            
            // 接触时实时腰部控制：当手只放在左边第二个按钮时，右边摇杆变为腰部控制指令
            if ((joystick_data_.left_second_button_touched && !joystick_data_.left_first_button_touched) && !torso_control_enabled_)
            {
                updateTorsoControl();
                return;
            }

            // 控制器切换：当手放在左边两个按钮上时，按下右边第一个按钮切换到下一个控制器
            if ((joystick_data_.left_first_button_touched && joystick_data_.left_second_button_touched))
            {
                if (!joystick_data_prev_.right_first_button_pressed && joystick_data_.right_first_button_pressed)
                {
                    callSwitchToNextControllerSrv();
                    return;
                }
            }
            
            if (!only_half_up_body_) {
                // 全身控制时才支持步态控制
                checkGaitSwitchCommand(joystick_data_);

                // 动态检查单步转向参数是否存在
                bool step_turning_enabled = false;
                if (nodeHandle_.hasParam("/quest3/use_step_turning")) {
                    nodeHandle_.getParam("/quest3/use_step_turning", step_turning_enabled);
                }

                // 添加单步转向控制
                // VR模式下，如果为RL控制器，禁止执行单步
                if (step_turning_enabled && !is_rl_controller_) {
                    updateSingleStepTurning();
                }
                else {
                    updateCommandLine();
                }
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

        void updateStateLegacyWheelVr()
        {
            if (!get_observation_ && !joystick_data_prev_.right_first_button_pressed && joystick_data_.right_first_button_pressed)
            {
                callRealInitializeSrv();
                return;
            }

            if (joystick_data_.left_first_button_pressed && joystick_data_.left_second_button_pressed)
            {
                callTerminateSrv();
                return;
            }

            if (joystick_data_.left_trigger > 0.5)
            {
                if (!joystick_data_prev_.left_first_button_pressed && joystick_data_.left_first_button_pressed)
                {
                    callEnableWbcArmTrajectorySrv(1);
                    return;
                }
                if (!joystick_data_prev_.right_first_button_pressed && joystick_data_.right_first_button_pressed)
                {
                    callSwitchToNextControllerSrv();
                    return;
                }
            }

            if (joystick_data_.left_grip > 0.5)
            {
                if (!joystick_data_prev_.left_first_button_pressed && joystick_data_.left_first_button_pressed)
                {
                    callEnableWbcArmTrajectorySrv(0);
                    return;
                }
            }

            if (joystick_data_.left_first_button_pressed)
            {
                if (!joystick_data_prev_.right_second_button_pressed && joystick_data_.right_second_button_pressed)
                {
                    callSetArmModeSrv(0);
                    arm_ctrl_mode_ = 0;
                }
                else if (!joystick_data_prev_.right_first_button_pressed && joystick_data_.right_first_button_pressed)
                {
                    if (arm_collision_control_) {
                        arm_ctrl_mode_ = 2;
                        arm_collision_control_ = false;
                    }
                    else arm_ctrl_mode_ = (arm_ctrl_mode_!=1) ? 1 : 2;
                    std::cout << "[QuestControlFSM] change arm mode to :" << arm_ctrl_mode_ << std::endl;
                    if (only_half_up_body_) {
                        callVRSetArmModeSrv(arm_ctrl_mode_);
                    }
                    else {
                        callSetArmModeSrv(arm_ctrl_mode_);
                    }

                    if(1 == robot_type_)
                    {
                        callWheelMpcControlMode(3);
                    }
                }
                return;
            }

            if (joystick_data_.left_trigger > 0.5)
            {
                if (!joystick_data_prev_.right_second_button_pressed && joystick_data_.right_second_button_pressed)
                {
                    if (!torso_control_enabled_)
                    {
                        torso_control_enabled_ = true;
                        torso_pitch_zero_= current_head_body_pose_.body_pitch;
                        torso_yaw_zero_ = current_head_body_pose_.body_yaw;
                        torso_roll_zero_ = current_head_body_pose_.body_roll;
                        body_height_zero_ = current_head_body_pose_.body_height;
                        body_x_zero_ = current_head_body_pose_.body_x;
                        torso_control_start_time_ = ros::Time::now();
                        last_body_yaw_ = current_head_body_pose_.body_yaw;
                        accumulated_yaw_offset_ = 0.0;

                        std_msgs::Bool whole_torso_ctrl_msg;
                        whole_torso_ctrl_msg.data = true;
                        whole_torso_ctrl_pub_.publish(whole_torso_ctrl_msg);

                        if(1 != robot_type_)
                        {
                            callAutoGaitModeSrv(false);
                            callVRWaistControlSrv(true);
                        }
                        if(1 == robot_type_)
                        {
                            callWheelMpcControlMode(3);
                        }
                    }
                    else
                    {
                        torso_control_enabled_ = false;
                        std_msgs::Bool whole_torso_ctrl_msg;
                        whole_torso_ctrl_msg.data = false;
                        whole_torso_ctrl_pub_.publish(whole_torso_ctrl_msg);

                        if(1 != robot_type_)
                        {
                            geometry_msgs::Twist cmd_pose;
                            cmd_pose.linear.x = 0.0;
                            cmd_pose.linear.y = 0.0;
                            cmd_pose.linear.z = last_relative_height_;
                            cmd_pose.angular.x = 0.0;
                            cmd_pose.angular.y = last_body_pitch_;
                            cmd_pose.angular.z = 0.0;
                            cmd_pose_pub_.publish(cmd_pose);

                            callAutoGaitModeSrv(true);
                            callVRWaistControlSrv(false);
                        }
                        if(1 == robot_type_)
                        {
                            callWheelMpcControlMode(3);
                        }
                    }
                    return;
                }
            }

            if ((joystick_data_.left_second_button_touched && !joystick_data_.left_first_button_touched) && !torso_control_enabled_)
            {
                updateTorsoControl();
                return;
            }

            if (!only_half_up_body_) {
                checkGaitSwitchCommand(joystick_data_);

                bool step_turning_enabled = false;
                if (nodeHandle_.hasParam("/quest3/use_step_turning")) {
                    nodeHandle_.getParam("/quest3/use_step_turning", step_turning_enabled);
                }

                if (step_turning_enabled) {
                    updateSingleStepTurning();
                }
                else {
                    updateCommandLine();
                }
            }
        }

        void updateTorsoControl()
        {
            if (waist_dof_ == 0) return;
            // 使用右边摇杆控制腰部
            const float deadzone = 0.1f;
            float right_x = joystick_data_.right_x;
            float right_y = joystick_data_.right_y;
            
            // 应用死区
            if (std::abs(right_x) < deadzone) right_x = 0.0f;
            if (std::abs(right_y) < deadzone) right_y = 0.0f;
            
            // 控制腰部yaw（左右转动）
            float yaw_sensitivity = static_cast<float>(waist_yaw_max_angle_deg_); // 灵敏度，从配置文件读取
            float target_yaw = -1 * right_x * yaw_sensitivity;
            std::cout << "controling torso_yaw: " << target_yaw << std::endl;
            controlWaist(target_yaw);
        }

        void controlWaist(double waist_yaw)
        {
            double max_angle = waist_yaw_max_angle_deg_; // 从配置文件读取
            waist_yaw = std::max(-max_angle, std::min(waist_yaw, max_angle));
            kuavo_msgs::robotWaistControl msg;
            msg.header.stamp = ros::Time::now();
            msg.data.data.resize(1);
            msg.data.data[0] =  waist_yaw;
            std::cout << "waist_yaw:" << waist_yaw <<std::endl;
            waist_motion_pub_.publish(msg);
        }
        
        void handWrenchConfigCallback(const std_msgs::Float64MultiArray::ConstPtr& msg)
        {
            if (msg->data.size() != 7) {
                ROS_WARN("Invalid hand wrench config message size: %lu (expected 7)", msg->data.size());
                return;
            }
            
            current_hand_wrench_item_mass_ = msg->data[0];
            current_hand_wrench_left_force_ = {msg->data[1], msg->data[2], msg->data[3]};
            current_hand_wrench_right_force_ = {msg->data[4], msg->data[5], msg->data[6]};
            
            ROS_INFO("Received hand wrench config: item_mass=%.2f kg, left_force=[%.2f, %.2f, %.2f] N, right_force=[%.2f, %.2f, %.2f] N",
                     current_hand_wrench_item_mass_,
                     current_hand_wrench_left_force_[0], current_hand_wrench_left_force_[1], current_hand_wrench_left_force_[2],
                     current_hand_wrench_right_force_[0], current_hand_wrench_right_force_[1], current_hand_wrench_right_force_[2]);
        }
        
        void isRlControllerCallback(const std_msgs::Float64::ConstPtr& msg)
        {
            is_rl_controller_ = (std::abs(msg->data) > 0.5);
        }
        
        bool useLegacyWheelVr() const
        {
            return robot_type_ == 1 && wheel_ik_;
        }
        
        void publishHandWrenchCmd(double item_mass, const std::vector<double>& left_force, const std::vector<double>& right_force)
        {
            std_msgs::Float64MultiArray msg;
            msg.data.resize(12);  // 左手 6 维 + 右手 6 维
            
            // 左手力 (力 + 力矩)
            msg.data[0] = left_force[0];   // fx
            msg.data[1] = left_force[1];   // fy
            msg.data[2] = left_force[2];   // fz
            msg.data[3] = 0.0;              // mx (力矩)
            msg.data[4] = 0.0;              // my
            msg.data[5] = 0.0;              // mz
            
            // 右手力 (力 + 力矩)
            msg.data[6] = right_force[0];   // fx
            msg.data[7] = right_force[1];   // fy
            msg.data[8] = right_force[2];   // fz
            msg.data[9] = 0.0;              // mx (力矩)
            msg.data[10] = 0.0;             // my
            msg.data[11] = 0.0;             // mz
            
            hand_wrench_cmd_pub_.publish(msg);
            
            ROS_INFO("Published hand wrench command: item_mass=%.2f kg, left_force=[%.2f, %.2f, %.2f] N, right_force=[%.2f, %.2f, %.2f] N",
                     item_mass, left_force[0], left_force[1], left_force[2], 
                     right_force[0], right_force[1], right_force[2]);
        }
        
        void applyHandWrench()
        {
            std::cout << "\033[92m========================================\033[0m" << std::endl;
            std::cout << "\033[92m    施加末端力 (Y+A)\033[0m" << std::endl;
            std::cout << "\033[92m    质量: " << current_hand_wrench_item_mass_ << " kg\033[0m" << std::endl;
            std::cout << "\033[92m    左手力: [" << current_hand_wrench_left_force_[0] << ", " 
                      << current_hand_wrench_left_force_[1] << ", " << current_hand_wrench_left_force_[2] << "] N\033[0m" << std::endl;
            std::cout << "\033[92m    右手力: [" << current_hand_wrench_right_force_[0] << ", " 
                      << current_hand_wrench_right_force_[1] << ", " << current_hand_wrench_right_force_[2] << "] N\033[0m" << std::endl;
            std::cout << "\033[92m========================================\033[0m" << std::endl;
            
            publishHandWrenchCmd(current_hand_wrench_item_mass_, current_hand_wrench_left_force_, current_hand_wrench_right_force_);
            hand_wrench_enabled_ = true;
        }
        
        void releaseHandWrench()
        {
            std::vector<double> zero_force = {0.0, 0.0, 0.0};
            
            std::cout << "\033[93m========================================\033[0m" << std::endl;
            std::cout << "\033[93m    释放末端力 (Y+B)\033[0m" << std::endl;
            std::cout << "\033[93m========================================\033[0m" << std::endl;
            
            publishHandWrenchCmd(0.0, zero_force, zero_force);
            hand_wrench_enabled_ = false;
        }

        // 获取当前步态名称
        std::string getCurrentGaitName()
        {
            kuavo_msgs::getCurrentGaitName srv;
            if (get_current_gait_name_service_client_.call(srv)) {
                if (srv.response.success) {
                    return srv.response.gait_name;
                } else {
                    ROS_WARN("Failed to get current gait name - service returned success: false");
                    return "";
                }
            } else {
                ROS_ERROR("Failed to call /humanoid_get_current_gait_name service");
                return "";
            }
        }

        void updateSingleStepTurning()
        {

            // 时间控制参数
            constexpr double kStableThreshold = 0.40;    // 稳定阈值 X 秒
            constexpr float kDeadzone = 0.05f;            // 死区
            constexpr double kDeadzoneTimeThreshold = 0.5; // 死区时间阈值（秒）- 需要在死区内持续这么久才退出

            // 获取摇杆值
            float right_x = joystick_data_.right_x;
            float right_y = joystick_data_.right_y;
            float left_x = joystick_data_.left_x;
            float left_y = joystick_data_.left_y;

            // 检查左摇杆是否在死区
            bool left_joystick_in_deadzone = (std::abs(left_x) < kDeadzone && std::abs(left_y) < kDeadzone);

            // 检查右摇杆是否有输入（用于转向）
            bool right_joystick_has_input = std::abs(right_x) >= kDeadzone;

            // 如果是walk状态，且右摇杆有输入，则执行正常运动控制（支持cmd_vel转向）
            std::string current_gait = getCurrentGaitName();
            if (current_gait == "walk" && right_joystick_has_input) {
                updateCommandLine();
                return;
            }

            // 单步转向开关已关，且左摇杆有输入，则执行正常运动控制（RL控制器在stance模式下不支持单步转向）
            if (!turn_step_single_step_switch_ && !left_joystick_in_deadzone) {
                updateCommandLine();
                return;
            }

            // 检查是否在死区内
            bool in_deadzone = torso_control_enabled_||std::abs(right_x) < kDeadzone || (std::abs(left_x) >= kDeadzone || std::abs(left_y) >= kDeadzone)||std::abs(right_y) > (std::abs(right_x) + kDeadzone);
            
            if (in_deadzone) {
                // 进入死区
                if (!turn_step_in_deadzone_) {
                    // 第一次进入死区，记录时间
                    turn_step_in_deadzone_ = true;
                    turn_step_deadzone_enter_time_ = ros::Time::now();
                    return; // 第一次检测到死区，不立即退出
                } else {
                    // 已经在死区内，检查持续时间
                    double time_in_deadzone = (ros::Time::now() - turn_step_deadzone_enter_time_).toSec();
                    if (time_in_deadzone >= kDeadzoneTimeThreshold) {
                        // 在死区内持续超过阈值时间，退出单步转向模式
                        turn_step_current_zone_ = -1;
                        turn_step_zone_published_ = false;
                        turn_step_in_deadzone_ = false;
                        // 有其他摇杆输入，执行正常运动控制
                        updateCommandLine();
                        return;
                    } else {
                        // 还未达到时间阈值，继续等待
                        return;
                    }
                }
            } else {
                // 不在死区内，重置死区状态
                turn_step_in_deadzone_ = false;
            }

            // 检测当前所在区间
            int target_zone = -1;

            // 直接检测摇杆值所在的区间
            for (size_t i = 0; i < turn_zones_.size(); ++i) {
                if (right_x >= turn_zones_[i].min_value && right_x < turn_zones_[i].max_value) {
                    target_zone = i;
                    break;
                }
            }

            // 如果不在任何区间，重置状态
            if (target_zone == -1) {
                turn_step_current_zone_ = -1;
                turn_step_zone_published_ = false;
                return;
            }

            // 安全的区间切换逻辑：只能在相邻区间内逐级改变
            int current_zone = turn_step_current_zone_;
            if (current_zone != -1 && current_zone != target_zone) {
                // 检查是否是相邻区间（防止跳变）
                if (std::abs(current_zone - target_zone) != 1) {
                    // 不允许跳变，重置状态
                    turn_step_current_zone_ = -1;
                    turn_step_zone_stable_ = false;
                    ROS_WARN("Zone change blocked: current=%d, target=%d (not adjacent). Only adjacent zone changes allowed for safety.",
                             current_zone, target_zone);
                    return;
                }
            }

            // 如果区间发生变化，重置时间
            if (target_zone != turn_step_current_zone_) {
                turn_step_current_zone_ = target_zone;
                turn_step_zone_enter_time_ = ros::Time::now();
                turn_step_zone_stable_ = false;
                turn_step_zone_published_ = false;  // 新区间，重置发布标志
                return;
            }

            ros::Time current_time = ros::Time::now();

            // 检查是否在区间内稳定超过阈值时间
            if (!turn_step_zone_stable_) {
                if ((current_time - turn_step_zone_enter_time_).toSec() >= kStableThreshold) {
                    turn_step_zone_stable_ = true;
                } else {
                    return; // 还未稳定，继续等待
                }
            }
    
            // 如果当前区间已经发布过，检查是否需要重新稳定后再次发布
            if (turn_step_zone_published_) {
                // 如果距离上次发布已经过了稳定时间阈值，允许重新发布
                if ((current_time - turn_step_last_execute_time_).toSec() >= kStableThreshold) {
                    turn_step_zone_published_ = false;  // 重置发布标志，允许再次发布
                    ROS_INFO("Zone %d ready for re-publish after %.2f seconds", turn_step_current_zone_, kStableThreshold);
                } else {
                    return;  // 还未到重新发布的时间，继续等待
                }
            }
    
            // 根据区间选择不同的控制方式
            if (target_zone >= 0 && target_zone < static_cast<int>(turn_zones_.size()) &&
                !turn_zones_[target_zone].trajectory.footPoseTrajectory.empty()) {

                // 调用服务检查当前是否是Custom-Gait模式
                std_srvs::SetBool gait_srv;
                gait_srv.request.data = false;

                bool service_call_success = get_current_gait_service_client_.call(gait_srv);

                if (service_call_success && !gait_srv.response.success) {
                    // 服务返回success: False，表示当前不是Custom-Gait模式，继续检查步态名称
                    std::string current_gait = getCurrentGaitName();
                    bool is_stance = (current_gait == "stance");
                    if (is_stance) {
                        foot_pose_target_pub_.publish(turn_zones_[target_zone].trajectory);
                        turn_step_zone_published_ = true;  // 标记已发布
                        turn_step_last_execute_time_ = ros::Time::now();  // 记录发布时间
                        ROS_WARN("Zone %d trajectory published - not Custom-Gait and current gait is stance", target_zone);
                    } else if(current_gait == "walk") {
                        // 先站立再单步
                        publish_mode_sequence_temlate("stance");
                        publish_zero_spd();
                        ROS_WARN("===================> Current gait is walk, switching to stance first failed");
                    }
                    else {
                        // ROS_WARN("Zone %d trajectory blocked - current gait is '%s' (not stance)", target_zone, current_gait.c_str());
                    }
                }
                else {
                    if(!service_call_success)
                        ROS_WARN("get_current_gait_service_client_ 调用失败");
                    else {
                        // ROS_WARN("get_current_gait_service_client_ gait_srv.response.success 是false");
                    }
                }
            }

            // ROS_INFO("Single step turn executed: zone=%d", target_zone);
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
            Eigen::VectorXd limit_vector_negative(4);
            limit_vector_negative << c_relative_base_limit_[0], c_relative_base_limit_[1], 
                                     std::fabs(vr_squat_height_min_), c_relative_base_limit_[3];
            Eigen::VectorXd limit_vector_positive(4);
            limit_vector_positive << c_relative_base_limit_[0], c_relative_base_limit_[1], 
                                     std::fabs(vr_squat_height_max_), c_relative_base_limit_[3];

            for (int i = 0; i < 4; i++) {
              if (joystick_vector[i] >= 0) {
                  commad_line_target_[i] = joystick_vector[i] * limit_vector_positive[i] * select_vector[i];
              } else {
                  commad_line_target_[i] = joystick_vector[i] * limit_vector_negative[i] * select_vector[i];
              }
            }

            cmdVel_.linear.x = commad_line_target_(0);
            cmdVel_.linear.y = commad_line_target_(1);
            cmdVel_.linear.z = commad_line_target_(2);
            cmdVel_.angular.z = commad_line_target_(3);
            if(!torso_control_enabled_ || robot_type_ == 1)  // 轮臂支持动腰和行走同时下发
                vel_control_pub_.publish(cmdVel_);
        }

        void checkGaitSwitchCommand(const kuavo_msgs::JoySticks &joy_msg)
        {
            // 检查是否有gait切换指令
            if (!joystick_data_prev_.right_first_button_pressed && joy_msg.right_first_button_pressed && joy_msg.left_trigger < 0.5 && !torso_control_enabled_)
            {
                publish_mode_sequence_temlate("stance");
                publish_zero_spd();
                turn_step_single_step_switch_ = false;
            }

            else if (!joystick_data_prev_.right_second_button_pressed && joy_msg.right_second_button_pressed && joy_msg.left_trigger < 0.5 && !torso_control_enabled_)
            {
                publish_mode_sequence_temlate("walk");
                turn_step_single_step_switch_ = true;
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

        void walkSpeedMode() 
        {
            if (!mode_changed_) 
            {
                mode_changed_ = true;
                publish_mode_sequence_temlate("walk");   // "walk"

                ROS_INFO("Mode changed to: %s", state_.c_str());
            }
            auto values_n = joystickNorm();
            auto values = getWalkValue(values_n, {0.2f, 0.1f, 8.0f});
            // 如果是轮式机器人(s60)，使用专门的移动命令
            if (1 == robot_type_) 
            {  // 假设robot_type=1表示轮式机器人

                sendWheelMoveCommand(values);
                previous_value_m_ = values_n;
            } 
            else 
            {
                sendWalkCommand(1, values);
                previous_value_m_ = values_n;
            }
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


        void sendWheelMoveCommand(const std::vector<float>& values) 
        {
            // 直接发布速度指令
            geometry_msgs::Twist vel;
            vel.linear.x = values[0];
            vel.linear.y = values[1];
            vel.linear.z = command_height_;
            vel.angular.z = 3.14 * values[2] / 180.0;
            vel_control_pub_.publish(vel);
            ROS_INFO("Wheel move command sent: values=%.2f, %.2f, %.2f", values[0], values[1], values[2]);
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
            // 同步发布 gait_name
            std_msgs::String gait_name_msg;
            gait_name_msg.data = gaitName;
            gait_name_publisher_.publish(gait_name_msg);
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
        ros::Publisher gait_name_publisher_;
        ros::Publisher stop_pub_;
        ros::Publisher step_num_stop_pub_;
        ros::Publisher vel_control_pub_;
        ros::Publisher whole_torso_ctrl_pub_;
        geometry_msgs::Twist cmdVel_;

        ros::ServiceClient change_arm_mode_service_client_;
        ros::ServiceClient change_arm_mode_service_VR_client_;
        ros::ServiceClient get_arm_mode_service_client_;

        ros::ServiceClient enable_wbc_arm_trajectory_control_client_;
        ros::ServiceClient vr_waist_control_service_client_;  // VR腰部控制动态Q矩阵服务客户端
        ros::ServiceClient auto_gait_mode_service_client_;    // GaitReceiver自动步态模式服务客户端
        ros::ServiceClient control_mode_client_;              // MPC控制模式切换服务客户端
        ros::ServiceClient switch_to_next_controller_client_; // 切换控制器服务客户端
        ros::ServiceServer arm_collision_control_service_;

        // 腰部控制相关的订阅者和发布者
        ros::Subscriber head_body_pose_sub_;
        ros::Publisher waist_motion_pub_;
        ros::Publisher cmd_pose_pub_;  // 用于发布高度和位置控制指令
        ros::Publisher cmd_torso_pose_pub_;


        float total_mode_scale_{1.0};

        std::map<std::string, humanoid::ModeSequenceTemplate> gait_map_;

        ros::Subscriber arm_ctrl_mode_vr_sub_; // 从主控制器获取手臂控制模式
        int arm_ctrl_mode_{2};

        ros::Subscriber joystick_sub_;
        std::string state_;
        kuavo_msgs::JoySticks joystick_data_;
        kuavo_msgs::JoySticks joystick_data_prev_;
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
        int robot_type_{0};  // 0: biped, 1: wheel robot
        int robot_version_int_{0};  // 机器人版本号
        bool wheel_ik_{false}; // 轮臂增量VR兼容模式开关

        // VR control limits (loaded from reference.info)
        double vr_squat_min_pitch_deg_{3.0};    // min pitch (deg)
        double vr_squat_max_pitch_deg_{35.0};   // max pitch (deg)
        double vr_squat_height_min_{-0.25};     // min height (m)
        double vr_squat_height_max_{0.1};       // max height (m)

        // 腰部控制相关变量
        bool torso_control_enabled_;
        bool control_torso_{false};  // 是否允许启动腰部控制
        
        // 末端力控制相关变量
        bool hand_wrench_enabled_;  // 末端力施加状态
        double current_hand_wrench_item_mass_;
        std::vector<double> current_hand_wrench_left_force_;
        std::vector<double> current_hand_wrench_right_force_;
        
        int waist_dof_{0};
        double waist_yaw_max_angle_deg_{0.0};  // 腰部最大旋转角度（度），从配置文件加载
        double torso_pitch_zero_;
        double torso_yaw_zero_;
        double body_height_zero_;  // 记录进入控制模式时的高度零点
        double body_x_zero_; // 记录进入控制模式时的x零点

        double torso_roll_zero_;
        ros::Time torso_control_start_time_;
        double last_relative_height_{0.0};  // 记录最后一次的相对高度
        double last_body_pitch_{0.0};       // 记录最后一次的body_pitch
        double last_body_yaw_{0.0};         // 记录上一次的body_yaw，用于角度连续性处理
        double accumulated_yaw_offset_{0.0}; // 累计的yaw偏移，用于处理180°跳变

        kuavo_msgs::headBodyPose current_head_body_pose_;
        // 手臂碰撞控制，当前是否处于发生碰撞，手臂回归控制中
        bool arm_collision_control_{false};

        // 单步转向控制状态变量
        bool turn_step_single_step_switch_{0};               // 当前单步转向步数
        int turn_step_current_zone_{-1};                 // 当前所在区间 (-1表示不在任何区间)
        ros::Time turn_step_zone_enter_time_;            // 进入当前区间时间
        bool turn_step_zone_stable_{false};              // 是否在区间内稳定超过阈值时间
        bool turn_step_zone_published_{false};           // 当前区间是否已发布轨迹
        ros::Time turn_step_last_execute_time_;          // 上次执行时间
        ros::Time turn_step_deadzone_enter_time_;        // 进入死区的时间
        bool turn_step_in_deadzone_{false};              // 是否在死区内
        std::vector<TurnStepZone> turn_zones_;           // 转向区间配置（从YAML文件加载）

        ros::Publisher arm_mode_pub_;
        ros::Publisher foot_pose_target_pub_;
        
        // 末端力控制相关
        ros::Subscriber hand_wrench_config_sub_;  // 末端力配置订阅器
        ros::Publisher hand_wrench_cmd_pub_;      // 末端力命令发布器
        
        ros::ServiceClient get_current_gait_service_client_;
        ros::ServiceClient get_current_gait_name_service_client_;
        
        // 控制器切换保护相关变量
        bool controller_switching_{false};              // 标志控制器是否正在切换
        ros::Time controller_switch_start_time_;        // 切换开始时间
        const double CONTROLLER_SWITCH_PROTECTION_TIME = 2.5;  // 保护时间窗口（秒）
        
        // RL控制器状态相关变量
        ros::Subscriber is_rl_controller_sub_;          // RL控制器状态订阅者
        bool is_rl_controller_{false};                 // 当前是否为RL控制器
        
        bool vr_torso_arm_locked_{false};              // 手臂是否被锁定（用于X+B奇偶判断）
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
