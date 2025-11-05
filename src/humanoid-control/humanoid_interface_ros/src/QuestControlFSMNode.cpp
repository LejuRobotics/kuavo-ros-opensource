#include <ros/ros.h>
#include <std_srvs/Trigger.h>
#include <string>
#include <vector>
#include "kuavo_msgs/JoySticks.h"
#include <string>

#include <ros/init.h>
#include <ros/package.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Float32.h>
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
#include <kuavo_msgs/headBodyPose.h>
#include <kuavo_msgs/footPose.h>
#include <kuavo_msgs/footPoseTargetTrajectories.h>
#include <kuavo_msgs/getCurrentGaitName.h>
#include <std_srvs/Trigger.h>
#include "utils/singleStepControl.hpp"

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
        return HumanoidControl::get_multiple_steps_msg(body_poses, 0.4, 0.1, true);
    }

    // 单步转向区间定义
    struct TurnStepZone {
        float min_value;     // 摇杆区间最小值
        float max_value;     // 摇杆区间最大值
        kuavo_msgs::footPoseTargetTrajectories trajectory;  // 足部轨迹
    };

    // 定义转向区间
    static const std::vector<TurnStepZone> kTurnZones = {
        // 右转区间 (0.0~1.0)
        {0.0f, 0.15f, CreateFootPoseTrajectory({
            Eigen::Vector4d(-0.01, -0.01, 0.0, -15.0)
        })},  // 小角度右转
         {0.151f, 0.45f, CreateFootPoseTrajectory({
            Eigen::Vector4d(-0.03, -0.03, 0.0, -30.0)
        })},  // 小角度右转
        {0.451f, 0.75f, CreateFootPoseTrajectory({
            Eigen::Vector4d(-0.04, -0.04, 0.0, -45.0)
        })}, // 中角度右转
        {0.751f, 1.0f, CreateFootPoseTrajectory({
            Eigen::Vector4d(-0.06, -0.06, 0.0, -60.0)
        })}, // 大角度右转
        // 左转区间 (-1.0~0.0)
         {-0.15f, 0.0f, CreateFootPoseTrajectory({
            Eigen::Vector4d(-0.01, 0.01, 0.0, 15.0)
        })}, // 小角度左转
        {-0.45f, -0.15f, CreateFootPoseTrajectory({
            Eigen::Vector4d(-0.03, 0.03, 0.0, 30.0)
        })}, // 小角度左转
        {-0.75f, -0.451f, CreateFootPoseTrajectory({
            Eigen::Vector4d(-0.04, 0.04, 0.0, 45.0)
        })}, // 中角度左转
        {-1.0f, -0.751f, CreateFootPoseTrajectory({
            Eigen::Vector4d(-0.06, 0.06, 0.0, 60.0)
        })} // 大角度左转
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
            last_execution_time_(ros::Time(0)),
            last_height_change_time_(ros::Time(0)),
            update_interval_(0.1),
            last_update_time_(ros::Time::now()),
            targetPoseCommand_(nodeHandle, robotName),
            torso_control_enabled_(false),
            torso_yaw_zero_(0.0),
            body_height_zero_(0.0),
            torso_control_start_time_(ros::Time(0))
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
            RobotVersion rb_version(3, 4);
            if (nodeHandle.hasParam("/robot_version"))
            {
                int rb_version_int;
                nodeHandle.getParam("/robot_version", rb_version_int);
                rb_version = RobotVersion::create(rb_version_int);
            }
            
            auto drake_interface_ = HighlyDynamic::HumanoidInterfaceDrake::getInstancePtr(rb_version, true, 2e-3);
            auto kuavo_settings = drake_interface_->getKuavoSettings();
            waist_dof_ = kuavo_settings.hardware_settings.num_waist_joints;
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

            mode_sequence_template_publisher_ = nodeHandle_.advertise<ocs2_msgs::mode_schedule>(robotName + "_mpc_mode_schedule", 10, true);
            mode_scale_publisher_ = nodeHandle_.advertise<std_msgs::Float32>(robotName + "_mpc_mode_scale", 10, true);
            gait_name_publisher_ = nodeHandle_.advertise<std_msgs::String>("/humanoid_mpc_gait_name_request", 10, true);

            joystick_sub_ = nodeHandle_.subscribe("/quest_joystick_data", 1, &QuestControlFSM::joystickCallback, this);
            observation_sub_ = nodeHandle_.subscribe(robotName + "_mpc_observation", 10, &QuestControlFSM::observationCallback, this);
            stop_pub_ = nodeHandle_.advertise<std_msgs::Bool>("/stop_robot", 10);
            step_num_stop_pub_ = nodeHandle_.advertise<std_msgs::Int32>(robotName + "_mpc_stop_step_num", 10, true);
            vel_control_pub_ = nodeHandle_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);

            change_arm_mode_service_client_ = nodeHandle_.serviceClient<kuavo_msgs::changeArmCtrlMode>("/humanoid_change_arm_ctrl_mode");
            change_arm_mode_service_VR_client_ = nodeHandle_.serviceClient<kuavo_msgs::changeArmCtrlMode>("/change_arm_ctrl_mode");
            
            get_arm_mode_service_client_ = nodeHandle_.serviceClient<kuavo_msgs::changeArmCtrlMode>("/humanoid_get_arm_ctrl_mode");
            
            // 添加 enable_wbc_arm_trajectory_control 服务客户端
            enable_wbc_arm_trajectory_control_client_ = nodeHandle_.serviceClient<kuavo_msgs::changeArmCtrlMode>("/enable_wbc_arm_trajectory_control");
            
            // VR腰部控制动态Q矩阵服务客户端
            vr_waist_control_service_client_ = nodeHandle_.serviceClient<std_srvs::SetBool>("/humanoid/mpc/vr_waist_control");
            
            // GaitReceiver自动步态模式服务客户端
            auto_gait_mode_service_client_ = nodeHandle_.serviceClient<std_srvs::SetBool>(robotName + "_auto_gait");
            
            // 腰部控制相关的订阅者和发布者
            head_body_pose_sub_ = nodeHandle_.subscribe("/kuavo_head_body_orientation_data", 1, &QuestControlFSM::headBodyPoseCallback, this);
            waist_motion_pub_ = nodeHandle_.advertise<std_msgs::Float64MultiArray>("/robot_waist_motion_data", 1);
            cmd_pose_pub_ = nodeHandle_.advertise<geometry_msgs::Twist>("/cmd_pose", 1);
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
                current_arm_mode_ = 0;
            }
            res.message = "Arm collision control set to " + std::string(req.data ? "true" : "false");
            ROS_INFO("Arm collision control set to %s", req.data ? "true" : "false");
            return true;
        }

    private:
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
            current_head_body_pose_ = *msg;
            current_head_body_pose_.body_pitch = std::max(3*M_PI/180.0, std::min(current_head_body_pose_.body_pitch, 35*M_PI/180.0));

            // 检查是否正在进行XY按键摇杆控制（高优先级），如果是则跳过VR腰部控制
            bool joystick_torso_control_active = (joystick_data_.left_second_button_touched && joystick_data_.left_first_button_touched);
            
            // 在腰部控制模式下且没有XY按键摇杆控制时，发布VR腰部控制指令
            if (torso_control_enabled_ && !joystick_torso_control_active)
            {
                // 腰部yaw控制（如果支持腰部自由度）
                if (waist_dof_ > 0)
                {
                    // 计算相对于零点的腰部位置
                    double current_yaw = current_head_body_pose_.body_yaw;
                    double relative_yaw = current_yaw - torso_yaw_zero_;
                    
                    // 发布腰部控制指令
                    controlWaist(relative_yaw * 180.0 / M_PI); // 转换为角度
                }
                
                //计算相对于0点的roll,yaw位置
                double current_roll = current_head_body_pose_.body_roll;
                double relative_roll = current_roll - torso_roll_zero_;
                double current_yaw_torso = current_head_body_pose_.body_yaw;
                double relative_yaw_torso = current_yaw_torso - torso_yaw_zero_;

                //std::cout << "相对roll: " << relative_roll << std::endl;
                if(waist_dof_ == 0)
                {
                    // std::cout << "相对roll: " << relative_roll * 180.0 / M_PI << std::endl;
                    // 对relative_roll限幅5度
                    relative_roll = std::max(-15.0*M_PI/180.0, std::min(relative_roll, 15.0*M_PI/180.0));
                    relative_yaw_torso = std::max(-30.0*M_PI/180.0, std::min(relative_yaw_torso, 30.0*M_PI/180.0));
                    //std::cout << "相对roll: " << relative_roll * 180.0 / M_PI << std::endl;
                    //std::cout << "相对yaw: " << relative_yaw_torso * 180.0 / M_PI << std::endl;
                }
                else
                {
                    // 对relative_roll限幅0度
                    relative_roll = 0;
                    relative_yaw_torso = 0;
                }
                
                // 高度控制
                // 根据msg中的pose高度发布高度指令（使用相对高度）
                double current_height = current_head_body_pose_.body_height;
                double relative_height = current_height - body_height_zero_;  // 计算相对于零点的高度
                //std::cout << "相对高度: " << relative_height << std::endl;
                //限制相对高度在[-0.4,0.1]之间
                relative_height = std::max(-0.35, std::min(relative_height, 0.1));
                geometry_msgs::Twist cmd_pose;
                cmd_pose.linear.x = 0.0;  // 基于当前位置的 x 方向值 (m)
                cmd_pose.linear.y = 0.0;  // 基于当前位置的 y 方向值 (m)
                cmd_pose.linear.z = relative_height;  // 相对高度
                cmd_pose.angular.x = relative_roll;  // roll
                cmd_pose.angular.z = relative_yaw_torso;  // # 基于当前位置旋转（偏航）的角度，单位为弧度 (radian)
                cmd_pose.angular.y = current_head_body_pose_.body_pitch;  // pitch

                cmd_pose_pub_.publish(cmd_pose);
                // 根据msg的pose值设置base的高度参考，通过/cmd_pose发布
            }
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
            if (joystick_data_.left_trigger > 0.5)
            {
                if (!joystick_data_prev_.left_first_button_pressed && joystick_data_.left_first_button_pressed)
                {
                    // 使能 WBC 手臂轨迹控制
                    callEnableWbcArmTrajectorySrv(1);
                    return;
                }
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
                    callSetArmModeSrv(0);
                    current_arm_mode_ = 0;
                }
                else if (!joystick_data_prev_.right_first_button_pressed && joystick_data_.right_first_button_pressed) // 启用手臂控制
                {
                    // 如果手臂碰撞控制中，手臂正在回归，回归完成会切换到手臂 KEEP 模式，此时再按 XA 继续手臂跟踪 
                    if (arm_collision_control_) {
                        current_arm_mode_ = 2;
                        arm_collision_control_ = false;
                    }
                    else current_arm_mode_ = (current_arm_mode_!=1) ? 1 : 2;
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
            
            
            // 腰部控制逻辑
            if (joystick_data_.left_trigger > 0.5)
            {
                if (!joystick_data_prev_.right_second_button_pressed && joystick_data_.right_second_button_pressed) // 左边第二个按钮按下，切换腰部控制模式
                {
                    if (!torso_control_enabled_)
                    {
                        // 启用腰部控制模式
                        torso_control_enabled_ = true;
                        torso_yaw_zero_ = current_head_body_pose_.body_yaw; // 记录当前腰部位置作为零点
                        body_height_zero_ = current_head_body_pose_.body_height; // 记录当前高度作为零点
                        torso_roll_zero_ = current_head_body_pose_.body_roll;
                        torso_control_start_time_ = ros::Time::now();
                        
                        if(waist_dof_ == 0)
                        {
                            // 失能GaitReceiver的自动步态模式
                            callAutoGaitModeSrv(false);
                            // 调用VR腰部控制服务，启用VR腰部控制动态Q矩阵
                            callVRWaistControlSrv(true);
                        }
                        
                        std::cout << "腰部控制模式已启用，腰部零点: " << torso_yaw_zero_ 
                                << "，高度零点: " << body_height_zero_ << std::endl;
                    }
                    else
                    {
                        // 关闭腰部控制模式
                        torso_control_enabled_ = false;
                        if(waist_dof_ == 0)
                        {
                            geometry_msgs::Twist cmd_pose;
                            cmd_pose.angular.z = 0;  // # 基于当前位置旋转（偏航）的角度，单位为弧度 (radian)
                            cmd_pose_pub_.publish(cmd_pose);

                            
                            // 使能GaitReceiver的自动步态模式
                            callAutoGaitModeSrv(true);
                            // 调用VR腰部控制服务，禁用VR腰部控制动态Q矩阵
                            callVRWaistControlSrv(false);
                        }
                        std::cout << "腰部控制模式已关闭" << std::endl;
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
            
            if (!only_half_up_body_) {
                // 全身控制时才支持步态控制
                checkGaitSwitchCommand(joystick_data_);

                // 动态检查单步转向参数是否存在
                bool step_turning_enabled = false;
                if (nodeHandle_.hasParam("/quest3/use_step_turning")) {
                    nodeHandle_.getParam("/quest3/use_step_turning", step_turning_enabled);
                }

                // 添加单步转向控制
                if (step_turning_enabled) {
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
            float yaw_sensitivity = 110.0f; // 灵敏度，与遥控器节点保持一致
            float target_yaw = -1 * right_x * yaw_sensitivity;
            std::cout << "controling torso_yaw: " << target_yaw << std::endl;
            controlWaist(target_yaw);
        }

        void controlWaist(double waist_yaw)
        {
            double max_angle = 110.0;
            waist_yaw = std::max(-max_angle, std::min(waist_yaw, max_angle));
            std_msgs::Float64MultiArray msg;
            msg.data.resize(1);
            msg.data[0] =  -waist_yaw;
            std::cout << "waist_yaw" << waist_yaw <<std::endl;
            waist_motion_pub_.publish(msg);
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

            // 获取摇杆值
            float right_x = joystick_data_.right_x;
            float left_x = joystick_data_.left_x;
            float left_y = joystick_data_.left_y;

            // 应用死区 - 如果右摇杆X轴在死区内，但其他轴有输入，则执行正常运动控制
            if (std::abs(right_x) < kDeadzone || (std::abs(left_x) >= kDeadzone ||std::abs(left_y) >= kDeadzone)) {
                turn_step_current_zone_ = -1;
                turn_step_zone_published_ = false;
                // 有其他摇杆输入，执行正常运动控制
                updateCommandLine();
                return;
            }

            // 检测当前所在区间
            int target_zone = -1;

            // 直接检测摇杆值所在的区间
            for (size_t i = 0; i < kTurnZones.size(); ++i) {
                if (right_x >= kTurnZones[i].min_value && right_x < kTurnZones[i].max_value) {
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
            if (target_zone >= 0 && target_zone < static_cast<int>(kTurnZones.size()) &&
                !kTurnZones[target_zone].trajectory.footPoseTrajectory.empty()) {

                // 调用服务检查当前是否是Custom-Gait模式
                std_srvs::SetBool gait_srv;
                gait_srv.request.data = false;

                bool service_call_success = get_current_gait_service_client_.call(gait_srv);

                if (service_call_success && !gait_srv.response.success) {
                    // 服务返回success: False，表示当前不是Custom-Gait模式，继续检查步态名称
                    std::string current_gait = getCurrentGaitName();
                    bool is_stance = (current_gait == "stance");
                    if (is_stance) {
                        foot_pose_target_pub_.publish(kTurnZones[target_zone].trajectory);
                        turn_step_zone_published_ = true;  // 标记已发布
                        turn_step_last_execute_time_ = ros::Time::now();  // 记录发布时间
                        ROS_WARN("Zone %d trajectory published - not Custom-Gait and current gait is stance", target_zone);
                    } else if(current_gait == "walk") {
                        // 先站立再单步
                        publish_mode_sequence_temlate("stance");
                        publish_zero_spd();
                        ROS_WARN("===================> 当前是walk, 先 stance 调用失败");
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
            Eigen::VectorXd limit_vector(4);
            limit_vector << c_relative_base_limit_[0], c_relative_base_limit_[1], c_relative_base_limit_[2], c_relative_base_limit_[3];

            commad_line_target_.head(4) = joystick_vector.cwiseProduct(limit_vector).cwiseProduct(select_vector);

            cmdVel_.linear.x = commad_line_target_(0);
            cmdVel_.linear.y = commad_line_target_(1);
            cmdVel_.linear.z = commad_line_target_(2);
            cmdVel_.angular.z = commad_line_target_(3);
            vel_control_pub_.publish(cmdVel_);
        }

        void checkGaitSwitchCommand(const kuavo_msgs::JoySticks &joy_msg)
        {
            // 检查是否有gait切换指令
            if (!joystick_data_prev_.right_first_button_pressed && joy_msg.right_first_button_pressed && joy_msg.left_trigger < 0.5)
            {
                publish_mode_sequence_temlate("stance");
                publish_zero_spd();
            }

            else if (!joystick_data_prev_.right_second_button_pressed && joy_msg.right_second_button_pressed && joy_msg.left_trigger < 0.5)
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

        geometry_msgs::Twist cmdVel_;

        ros::ServiceClient change_arm_mode_service_client_;
        ros::ServiceClient change_arm_mode_service_VR_client_;
        ros::ServiceClient get_arm_mode_service_client_;
        ros::ServiceClient enable_wbc_arm_trajectory_control_client_;
        ros::ServiceClient vr_waist_control_service_client_;  // VR腰部控制动态Q矩阵服务客户端
        ros::ServiceClient auto_gait_mode_service_client_;    // GaitReceiver自动步态模式服务客户端
        ros::ServiceServer arm_collision_control_service_;

        // 腰部控制相关的订阅者和发布者
        ros::Subscriber head_body_pose_sub_;
        ros::Publisher waist_motion_pub_;
        ros::Publisher cmd_pose_pub_;  // 用于发布高度和位置控制指令

        int current_arm_mode_{2};

        float total_mode_scale_{1.0};

        std::map<std::string, humanoid::ModeSequenceTemplate> gait_map_;

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

        // 腰部控制相关变量
        bool torso_control_enabled_;
        int waist_dof_{0};
        double torso_yaw_zero_;
        double body_height_zero_;  // 记录进入控制模式时的高度零点
        double torso_roll_zero_;
        ros::Time torso_control_start_time_;

        kuavo_msgs::headBodyPose current_head_body_pose_;
        // 手臂碰撞控制，当前是否处于发生碰撞，手臂回归控制中
        bool arm_collision_control_{false};

        // 单步转向控制状态变量
        int turn_step_current_zone_{-1};                 // 当前所在区间 (-1表示不在任何区间)
        ros::Time turn_step_zone_enter_time_;            // 进入当前区间时间
        bool turn_step_zone_stable_{false};              // 是否在区间内稳定超过阈值时间
        bool turn_step_zone_published_{false};           // 当前区间是否已发布轨迹
        ros::Time turn_step_last_execute_time_;          // 上次执行时间

        ros::Publisher arm_mode_pub_;
        ros::Publisher foot_pose_target_pub_;
        ros::ServiceClient get_current_gait_service_client_;
        ros::ServiceClient get_current_gait_name_service_client_;
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
