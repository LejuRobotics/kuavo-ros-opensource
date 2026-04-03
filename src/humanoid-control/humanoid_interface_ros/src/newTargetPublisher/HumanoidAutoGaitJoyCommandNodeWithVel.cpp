/******************************************************************************
Copyright (c) 2021, Farbod Farshidian. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

 * Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

 * Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

 * Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
******************************************************************************/

#include <string>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/joystick.h> 

#include <ros/init.h>
#include <ros/package.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <geometry_msgs/Twist.h>
#include "kuavo_msgs/SetJoyTopic.h"
#include "kuavo_msgs/switchController.h"
#include "kuavo_msgs/getControllerList.h"
#include "kuavo_msgs/switchToNextController.h"
#include <kuavo_msgs/robotWaistControl.h>
#include <std_srvs/SetBool.h>

#include <ocs2_core/Types.h>
#include <ocs2_core/misc/LoadData.h>
#include <ocs2_msgs/mpc_observation.h>

#include <ocs2_ros_interfaces/command/TargetTrajectoriesRosPublisher.h>
#include <humanoid_interface/gait/ModeSequenceTemplate.h>
#include "humanoid_interface_ros/gait/ModeSequenceTemplateRos.h"
#include "humanoid_interface_ros/newTargetPublisher/LowPassFilter.h"
#include "humanoid_interface_ros/newTargetPublisher/LowPassFilter5thOrder.h"
#include "std_srvs/Trigger.h"
#include <std_msgs/Bool.h>
#include <ocs2_robotic_tools/common/RotationTransforms.h>

#include "humanoid_interface_drake/humanoid_interface_drake.h"
#include <humanoid_interface/gait/MotionPhaseDefinition.h>
#include "kuavo_msgs/gaitTimeName.h"
#include <ocs2_msgs/mpc_flattened_controller.h>
#include <kuavo_common/common/json.hpp>
#include <map>
#include <kuavo_msgs/changeArmCtrlMode.h>
#include "kuavo_msgs/robotHeadMotionData.h"
#include "kuavo_msgs/robotHandPosition.h"
#include <std_srvs/SetBool.h>
#include <kuavo_msgs/ExecuteArmAction.h>
#include <humanoid_plan_arm_trajectory/RobotActionState.h>

// 命令执行相关头文件
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <array>
#include <thread>
#include <future>

namespace ocs2
{
  using namespace humanoid;
  std::map<std::string, int> joyButtonMap = {
      {"BUTTON_STANCE", 0},
      {"BUTTON_TROT", 1},
      {"BUTTON_RL", 2},
      {"BUTTON_WALK", 3},
      {"BUTTON_LB", 4},
      {"BUTTON_RB", 5},
      {"BUTTON_BACK", 6},
      {"BUTTON_START", 7},
      {"BUTTON_GUIDE", 8},
      {"BUTTON_M1", 9},
      {"BUTTON_M2", 10}
  };

  std::map<std::string, int> joyAxisMap = {
      {"AXIS_LEFT_STICK_Y", 0},
      {"AXIS_LEFT_STICK_X", 1},
      {"AXIS_LEFT_LT", 2},
      {"AXIS_RIGHT_STICK_YAW", 3},
      {"AXIS_RIGHT_STICK_Z", 4},
      {"AXIS_RIGHT_RT", 5},
      {"AXIS_LEFT_RIGHT_TRIGGER", 6},
      {"AXIS_FORWARD_BACK_TRIGGER", 7}
  };

    std::map<std::string, int> joyButtonMap_backup = {
      {"BUTTON_STANCE", 0},
      {"BUTTON_TROT", 1},
      {"BUTTON_RL", 3},
      {"BUTTON_WALK", 4},
      {"BUTTON_LB", 6},
      {"BUTTON_RB", 7},
      {"BUTTON_BACK", 10},
      {"BUTTON_START", 11},
      {"BUTTON_GUIDE", 12},
      {"BUTTON_M1", 13},
      {"BUTTON_M2", 14}
  };

  std::map<std::string, int> joyAxisMap_backup = {
      {"AXIS_LEFT_STICK_Y", 0},
      {"AXIS_LEFT_STICK_X", 1},
      {"AXIS_LEFT_LT", 5},
      {"AXIS_RIGHT_STICK_YAW", 2},
      {"AXIS_RIGHT_STICK_Z", 3},
      {"AXIS_RIGHT_RT", 4},
      {"AXIS_LEFT_RIGHT_TRIGGER", 6},
      {"AXIS_FORWARD_BACK_TRIGGER", 7}
  };


  struct gaitTimeName_t
  {
    std::string name;
    double startTime;
  };

  // 命令结构体
  struct Command_t
  {
    std::string name;
    std::string type;
    std::string value;
    std::string description;
  };

#define DEAD_ZONE 0.05
#define TARGET_REACHED_THRESHOLD 0.1
#define TARGET_REACHED_THRESHOLD_YAW 0.1
#define TARGET_REACHED_FEET_THRESHOLD 0.08
#define MAX_JOYSTICK_NAME_LEN 256
#define JOYSTICK_XBOX_MAP_JSON "bt2"
#define JOYSTICK_XBOX_BUTTON_NUM 11
#define JOYSTICK_BEITONG_MAP_JSON "bt2pro"
#define JOYSTICK_BEITONG_BUTTON_NUM 16
#define JOYSTICK_AXIS_NUM 8
#define WAIST_YAW_MAX_ANGLE_DEG 180.0  // 腰部最大旋转角度（度），±180度

  class JoyControl
  {
  public:
    JoyControl(ros::NodeHandle &nodeHandle, const std::string &robotName, bool verbose = false)
        : nodeHandle_(nodeHandle),
          targetPoseCommand_(nodeHandle, robotName)
    {
      if (nodeHandle.hasParam("/robot_type"))
      {
        nodeHandle.getParam("/robot_type", robot_type_);
        ROS_INFO_STREAM("[JoyControl] Robot type: " << robot_type_ << " (1=wheel, 2=humanoid)");
      }
      else
      {
        ROS_INFO_STREAM("[JoyControl] Robot type not specified, defaulting to humanoid mode (2)");
      }

      // 获取repo_root_path
      if (!nodeHandle.getParam("repo_root_path", repo_root_path_))
      {
        ROS_WARN_STREAM("No repo_root_path parameter found, using current directory.");
        repo_root_path_ = ".";
      }
      ROS_INFO_STREAM("Repository root path: " << repo_root_path_);

      if (nodeHandle.hasParam("/robot_version"))
      {
        int rb_version_int;
        nodeHandle.getParam("/robot_version", rb_version_int);
        rb_version_ = RobotVersion::create(rb_version_int);
      }

      if(rb_version_.major() <= 2)  // 判断小于等于2的主版本号为鲁班
      {
        controller_switch_time = roban_controller_switch_time;
        std::cout << "[JoyControl] controller_switch_time change to roban's time !!!" << std::endl;
      }

      if (nodeHandle.hasParam("/joy_node/dev"))
      {
        std::string joystick_device;
        nodeHandle.getParam("/joy_node/dev", joystick_device);

        int fd = -1;
        const char *device_path = joystick_device.c_str();

        fd = open(device_path, O_RDONLY);
        if (fd < 0) {
          ROS_ERROR("[JoyControl] Error opening joystick device: %s", device_path);
        } 
        else {
          char name[MAX_JOYSTICK_NAME_LEN] = {0};
          if (ioctl(fd, JSIOCGNAME(MAX_JOYSTICK_NAME_LEN), name) < 0) {
              ROS_ERROR("[JoyControl] Error getting joystick name via ioctl");
          } 
          else {
            std::string joystick_name(name);
            
            if (joystick_name.find("BEITONG") != std::string::npos) {
              nodeHandle.setParam("joystick_type", JOYSTICK_BEITONG_MAP_JSON);
              std::string channel_map_path = ros::package::getPath("humanoid_controllers") + "/launch/joy/" + JOYSTICK_BEITONG_MAP_JSON + ".json";
              nodeHandle.setParam("channel_map_path", channel_map_path);
              ROS_INFO("[JoyControl] Setting joystick type to bt2pro");
              loadJoyJsonConfig(ros::package::getPath("humanoid_controllers") + "/launch/joy/" + JOYSTICK_XBOX_MAP_JSON + ".json", joyButtonMap_backup, joyAxisMap_backup);
            } else if (joystick_name.find("X-Box") != std::string::npos) {
              nodeHandle.setParam("joystick_type", JOYSTICK_XBOX_MAP_JSON);
              std::string channel_map_path = ros::package::getPath("humanoid_controllers") + "/launch/joy/" + JOYSTICK_XBOX_MAP_JSON + ".json";
              nodeHandle.setParam("channel_map_path", channel_map_path);
              ROS_INFO("[JoyControl] Setting joystick type to bt2");
              
              loadJoyJsonConfig(ros::package::getPath("humanoid_controllers") + "/launch/joy/" + JOYSTICK_BEITONG_MAP_JSON + ".json", joyButtonMap_backup, joyAxisMap_backup);
            }
          }
          close(fd);
        }
      }

      std::string channel_map_path_loaded;
      if (nodeHandle.hasParam("channel_map_path"))
      {
        nodeHandle.getParam("channel_map_path", channel_map_path_loaded);
        ROS_INFO_STREAM("Loading joystick mapping from " << channel_map_path_loaded);
        loadJoyJsonConfig(channel_map_path_loaded, joyButtonMap, joyAxisMap);
        if (joyButtonMap_backup.empty()) {
          if (channel_map_path_loaded.find(JOYSTICK_BEITONG_MAP_JSON) != std::string::npos)
            loadJoyJsonConfig(ros::package::getPath("humanoid_controllers") + "/launch/joy/" + JOYSTICK_XBOX_MAP_JSON + ".json", joyButtonMap_backup, joyAxisMap_backup);
          else
            loadJoyJsonConfig(ros::package::getPath("humanoid_controllers") + "/launch/joy/" + JOYSTICK_BEITONG_MAP_JSON + ".json", joyButtonMap_backup, joyAxisMap_backup);
        }
      }
      else
      {
        ROS_WARN_STREAM("No channel_map_path parameter found, using default joystick mapping.");
      }
      if (nodeHandle.hasParam("joystick_sensitivity"))
      {
        nodeHandle.getParam("joystick_sensitivity", joystickSensitivity);
        ROS_INFO_STREAM("Loading joystick sensitivity: " << joystickSensitivity);
      }
      else
      {
        ROS_WARN_STREAM("No input sensitivity parameter found, using default joystick sensitivity.");
      }

      if (nodeHandle.hasParam("joy_execute_action"))
      {
        nodeHandle.getParam("joy_execute_action", joy_execute_action_);
        // kuavo 不受 roban 按键需求影响
        if(rb_version_.major() != 1)
        {
          joy_execute_action_ = false;
          nodeHandle.setParam("joy_execute_action", false);
        }
        ROS_INFO_STREAM("Loading joy_execute_action_ value: " << joy_execute_action_);
      }
      else
      {
        ROS_WARN_STREAM("No joy_execute_action parameter found, using default joy_execute_action.");
      }

      Eigen::Vector4d joystickFilterCutoffFreq_(joystickSensitivity, joystickSensitivity, 
                                                  joystickSensitivity, joystickSensitivity);
      joystickFilter_.setParams(0.01,joystickFilterCutoffFreq_);
      old_joy_msg_.axes = std::vector<float>(JOYSTICK_AXIS_NUM, 0.0);
      if (!channel_map_path_loaded.empty() && channel_map_path_loaded.find(JOYSTICK_BEITONG_MAP_JSON) != std::string::npos)
        old_joy_msg_.buttons = std::vector<int32_t>(JOYSTICK_BEITONG_BUTTON_NUM, 0);
      else
        old_joy_msg_.buttons = std::vector<int32_t>(JOYSTICK_XBOX_BUTTON_NUM, 0);
      // Get node parameters
      std::string referenceFile;
      nodeHandle.getParam("/referenceFile", referenceFile);

      if (nodeHandle.hasParam("/real"))
      {
        nodeHandle.getParam("/real", real_);
      }
      else
      {
        ROS_WARN_STREAM("No real parameter found, using default real.");
      }

      if (robot_type_ == 2)
      {
        // loadData::loadCppDataType(referenceFile, "comHeight", com_height_);
        auto drake_interface_ = HighlyDynamic::HumanoidInterfaceDrake::getInstancePtr(rb_version_, true, 2e-3);
        default_joint_state_ = drake_interface_->getDefaultJointState();
        com_height_ = drake_interface_->getIntialHeight();
        auto kuavo_settings = drake_interface_->getKuavoSettings();
        waist_dof_ = kuavo_settings.hardware_settings.num_waist_joints;
        loadData::loadCppDataType(referenceFile, "targetRotationVelocity", target_rotation_velocity_);
        loadData::loadCppDataType(referenceFile, "targetDisplacementVelocity", target_displacement_velocity_);
        loadData::loadCppDataType(referenceFile, "cmdvelLinearXLimit", c_relative_base_limit_[0]);
        try {
          loadData::loadCppDataType(referenceFile, "cmdvelLinearYLimit", c_relative_base_limit_[1]);
        } catch (const std::exception &e) {
          ROS_WARN_STREAM("cmdvelLinearYLimit not found, using default: " << c_relative_base_limit_[1]);
        }
        loadData::loadCppDataType(referenceFile, "vrSquatHeightMin", squatHeightMin_);
        loadData::loadCppDataType(referenceFile, "vrSquatHeightMax", squatHeightMax_);
        try {
          loadData::loadCppDataType(referenceFile, "cmdvelLinearZLimit", c_relative_base_limit_[2]);
        } catch (const std::exception &e) {
          ROS_WARN_STREAM("cmdvelLinearZLimit not found, using default: " << c_relative_base_limit_[2]);
        }
        std::cout << "cmdvelLinearZLimit:" << c_relative_base_limit_[2] << std::endl;

        loadData::loadCppDataType(referenceFile, "cmdvelAngularYAWLimit", c_relative_base_limit_[3]);
        
        // 加载腰部最大旋转角度
        try {
          loadData::loadCppDataType(referenceFile, "waist_yaw_max", waist_yaw_max_angle_deg_);
        } catch (const std::exception &e) {
          ROS_WARN_STREAM("waist_yaw_max not found, using default: " << waist_yaw_max_angle_deg_);
        }

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
        gait_name_publisher_ = nodeHandle.advertise<std_msgs::String>("/humanoid_mpc_gait_name_request", 10, true);
      }
      else
      {
        // Wheel mode: initialize with default values
        ROS_INFO_STREAM("[JoyControl] Wheel mode detected, skipping gait loading");
        com_height_ = 0.0;
        target_rotation_velocity_ = 1.0;
        target_displacement_velocity_ = 1.0;
        default_joint_state_ = vector_t::Zero(12);
        // Use default limits for wheel mode
        c_relative_base_limit_[0] = 0.8;  // linear x
        c_relative_base_limit_[1] = 0.8;  // linear y
        c_relative_base_limit_[2] = 0.0; // linear z (not used in wheel mode)
        c_relative_base_limit_[3] = 1.2; // angular z
      }

      // 保存MPC默认速度限制）
      mpc_default_velocity_limits_ = c_relative_base_limit_;

      cmd_vel_publisher_ = nodeHandle.advertise<geometry_msgs::Twist>("/cmd_vel", 10, true);
      
      current_joy_topic_ = "/joy";  // 默认话题
      joy_topic_service_ = nodeHandle_.advertiseService("/set_joy_topic", &JoyControl::setJoyTopicCallback, this);
      switch_controller_client_ = nodeHandle_.serviceClient<kuavo_msgs::switchController>("/humanoid_controller/switch_controller");
      get_controller_list_client_ = nodeHandle_.serviceClient<kuavo_msgs::getControllerList>("/humanoid_controller/get_controller_list");
      switch_to_next_controller_client_ = nodeHandle_.serviceClient<kuavo_msgs::switchToNextController>("/humanoid_controller/switch_to_next_controller");
      switch_to_previous_controller_client_ = nodeHandle_.serviceClient<kuavo_msgs::switchToNextController>("/humanoid_controller/switch_to_previous_controller");
      auto_gait_change_client_ = nodeHandle_.serviceClient<std_srvs::SetBool>("/humanoid_auto_gait");
      joy_sub_ = nodeHandle_.subscribe(current_joy_topic_, 10, &JoyControl::joyCallback, this);
      
      // 轮臂模式下初始化observation_（避免访问observation.state时发生段错误）
      if (robot_type_ == 1)
      {
        // 为轮臂模式初始化一个虚拟的observation
        // observation.state至少需要12个元素（6个base状态 + 6个pose状态）
        observation_.state = vector_t::Zero(12);
        observation_.input = vector_t::Zero(6);
        observation_.time = 0.0;
        observation_.mode = 0;
        get_observation_ = true;  // 标记为已初始化，避免等待observation
        ROS_INFO("[JoyControl] 轮臂模式: 已初始化虚拟observation");
      }
      
      if (robot_type_ == 2)
      {
        feet_sub_ = nodeHandle_.subscribe("/humanoid_controller/swing_leg/pos_measured", 2, &JoyControl::feetCallback, this);
        observation_sub_ = nodeHandle_.subscribe(robotName + "_mpc_observation", 10, &JoyControl::observationCallback, this);
        gait_scheduler_sub_ = nodeHandle_.subscribe<kuavo_msgs::gaitTimeName>(robotName + "_mpc_gait_time_name", 10, [this](const kuavo_msgs::gaitTimeName::ConstPtr &msg)
                                                                              {
                                                                                last_gait_rec_ = current_gait_rec_;
                                                                                current_gait_rec_.name = msg->gait_name;
                                                                                current_gait_rec_.startTime = msg->start_time; });
        policy_sub_ = nodeHandle_.subscribe<ocs2_msgs::mpc_flattened_controller>(
            robotName + "_mpc_policy",                            // topic name
            1,                                                    // queue length
            boost::bind(&JoyControl::mpcPolicyCallback, this, _1) // callback
        );
        gait_change_sub_ = nodeHandle_.subscribe<std_msgs::String>(
        "/humanoid_mpc_gait_change", 1, &JoyControl::gaitChangeCallback, this);
      }
      is_rl_controller_sub_ = nodeHandle_.subscribe<std_msgs::Float64>("/humanoid_controller/is_rl_controller_", 1, [this](const std_msgs::Float64::ConstPtr &msg) 
      {
        bool new_is_rl = (msg->data > 0.5);
        // 检测控制器状态变化，设置切换标志
        if (is_rl_controller_ != new_is_rl)
        {
          is_rl_controller_ = new_is_rl;
          last_controller_switch_time_ = ros::Time::now();
          controller_switching_ = true;
          ROS_WARN_STREAM("[JoyControl] Controller switched to " << (is_rl_controller_ ? "RL" : "MPC")
                          << ", disable joystick input for " << controller_switch_time << "s");
          
          // 根据控制器类型更新速度限制
          if (is_rl_controller_)
          {
            // RL控制器：从rosparam读取速度限制
            updateVelocityLimitsFromParam(true);
          }
          else
          {
            // MPC控制器：恢复MPC默认速度限制
            c_relative_base_limit_ = mpc_default_velocity_limits_;
            ROS_INFO("[JoyControl] Restored MPC default velocity limits: [%.2f, %.2f, %.2f, %.2f]",
                     c_relative_base_limit_[0], c_relative_base_limit_[1], 
                     c_relative_base_limit_[2], c_relative_base_limit_[3]);
          }
        }
      });
      // 订阅动作执行状态话题，用于检测是否有动作正在执行
      robot_action_state_sub_ = nodeHandle_.subscribe<humanoid_plan_arm_trajectory::RobotActionState>(
      "/robot_action_state", 1, &JoyControl::robotActionStateCallback, this);
      // 从主控制器实时订阅当前手臂控制模式
      arm_ctrl_mode_sub_ = nodeHandle_.subscribe<std_msgs::Float64MultiArray>(
      "/humanoid/mpc/arm_control_mode", 1, &JoyControl::armCtrlModeCallback, this); 

      stop_pub_ = nodeHandle_.advertise<std_msgs::Bool>("/stop_robot", 10);
      re_start_pub_ = nodeHandle_.advertise<std_msgs::Bool>("/re_start_robot", 10);
      head_motion_pub_ = nodeHandle_.advertise<kuavo_msgs::robotHeadMotionData>("/robot_head_motion_data", 10);
      waist_motion_pub_ = nodeHandle_.advertise<kuavo_msgs::robotWaistControl>("/robot_waist_motion_data", 10);
      enable_waist_control_pub_ = nodeHandle_.advertise<std_msgs::Bool>("/humanoid_controller/enable_waist_control", 10);
      slope_planning_pub_ = nodeHandle_.advertise<std_msgs::Bool>("/humanoid/mpc/enable_slope_planning", 10);
      hand_position_pub_ = nodeHandle_.advertise<kuavo_msgs::robotHandPosition>("/control_robot_hand_position", 10);

      // Service clients
      execute_arm_action_client_ = nodeHandle_.serviceClient<kuavo_msgs::ExecuteArmAction>("/execute_arm_action");
      // Launch status client (rate-limited checks in joy callback)
      real_launch_status_client_ = nodeHandle_.serviceClient<std_srvs::Trigger>("/humanoid_controller/real_launch_status");
      // Fall stand up trigger client
      trigger_fall_stand_up_client_ = nodeHandle_.serviceClient<std_srvs::Trigger>("/humanoid_controller/trigger_fall_stand_up");
      // Fall down state client
      set_fall_down_state_client_ = nodeHandle_.serviceClient<std_srvs::SetBool>("/humanoid_controller/set_fall_down_state");
      // Dance controller trigger client
      trigger_dance_client_ = nodeHandle_.serviceClient<std_srvs::Trigger>("/humanoid_controller/switch_to_dance_controller");
      last_status_check_time_ = ros::Time(0);

      // 加载命令配置
      loadCommandsConfig();
    }
    void loadJoyJsonConfig(const std::string &config_file, std::map<std::string, int>& button_map, std::map<std::string, int>& axis_map)
    {
      nlohmann::json data_;
      std::ifstream ifs(config_file);
      ifs >> data_;
      for (auto &item : data_["JoyButton"].items())
      {
        std::cout << "button:" << item.key() << " item.value():" << item.value() << std::endl;
        if (button_map.find(item.key())!= button_map.end())
        {
          button_map[item.key()] = item.value();
        }
        else
          button_map.insert({item.key(), item.value()});
      }
      for (auto &item : data_["JoyAxis"].items())
      {
        if (axis_map.find(item.key())!= axis_map.end())
        {
          axis_map[item.key()] = item.value();
        }
        else
          axis_map.insert({item.key(), item.value()});
      }
    
    }
    void loadCommandsConfig()
    {
      std::string commands_config_path = repo_root_path_ + "/src/humanoid-control/humanoid_controllers/config/commands.yaml";
      ROS_INFO_STREAM("Loading commands config from: " << commands_config_path);
      
      try
      {
        std::ifstream config_file(commands_config_path);
        if (!config_file.is_open())
        {
          ROS_ERROR_STREAM("Failed to open commands config file: " << commands_config_path);
          return;
        }
        
        // 使用简单的文本解析方法
        std::string line;
        std::string current_command;
        Command_t current_cmd;
        bool in_commands_section = false;
        
        while (std::getline(config_file, line))
        {
          // 去除前导空格
          line.erase(0, line.find_first_not_of(" \t"));
          
          if (line.empty() || line[0] == '#')
            continue;
          
          // 检查是否进入commands部分
          if (line == "commands:")
          {
            in_commands_section = true;
            continue;
          }
          
          if (!in_commands_section)
            continue;
          
          // 检查是否是新的命令（以冒号结尾且不包含空格）
          if (line.find(":") == line.length() - 1 && line.find(" ") == std::string::npos)
          {
            if (!current_command.empty())
            {
              // 保存前一个命令
              commands_map_[current_cmd.name] = current_cmd;
            }
            current_command = line.substr(0, line.length() - 1);
            current_cmd = Command_t(); // 重置
            current_cmd.name = current_command;
            continue;
          }
          
          // 解析键值对
          if (line.find(":") != std::string::npos)
          {
            size_t colon_pos = line.find(":");
            std::string key = line.substr(0, colon_pos);
            std::string value = line.substr(colon_pos + 1);
            
            // 去除前导空格
            value.erase(0, value.find_first_not_of(" \t"));
            
            // 去除引号
            if (!value.empty() && value[0] == '"')
            {
              value = value.substr(1);
              if (!value.empty() && value.back() == '"')
                value.pop_back();
            }
            
            if (key == "type")
            {
              current_cmd.type = value;
            }
            else if (key == "value")
            {
              current_cmd.value = value;
            }
            else if (key == "description")
            {
              current_cmd.description = value;
            }
          }
        }
        
        // 保存最后一个命令
        if (!current_command.empty())
        {
          commands_map_[current_cmd.name] = current_cmd;
        }
        
        // std::cout << "Loaded " << commands_map_.size() << " commands" <<std::endl;
        // for (const auto& cmd : commands_map_)
        // {
        //   std::cout << " - " << cmd.first << ": "<< cmd.second.type << " " << cmd.second.value << " " << cmd.second.description << std::endl;
        // }
      }
      catch (const std::exception& e)
      {
        ROS_ERROR_STREAM("Error loading commands config: " << e.what());
      }
    }
    void mpcPolicyCallback(const ocs2_msgs::mpc_flattened_controller::ConstPtr &msg)
    {
      const auto targetTrajSize = msg->planTargetTrajectories.stateTrajectory.size();
      auto planTargetTrajectory = msg->planTargetTrajectories.stateTrajectory[targetTrajSize - 1].value;
      std::vector<double> planTargetTrajectoryDouble(planTargetTrajectory.begin(), planTargetTrajectory.end());
      auto last_target = Eigen::Map<const vector_t>(planTargetTrajectoryDouble.data(), planTargetTrajectoryDouble.size());
      current_target_ = last_target.segment<6>(6);
    }

    void run()
    {
      ros::Rate rate(100);
      while (ros::ok())
      {
        ros::spinOnce();
        
        // 检查异步命令执行结果
        if (command_future_.valid() && command_future_.wait_for(std::chrono::milliseconds(0)) == std::future_status::ready)
        {
          try
          {
            bool result = command_future_.get();
            if (result)
            {
              // std::cout << "[JoyControl] 异步命令执行成功" << std::endl;
            }
            else
            {
              std::cerr << "[JoyControl] 异步命令执行失败" << std::endl;
            }
          }
          catch (const std::exception& e)
          {
            std::cerr << "[JoyControl] 异步命令执行异常: " << e.what() << std::endl;
          }
        }
        
        rate.sleep();
        if (robot_type_ == 1)
        {
          checkAndPublishCommandLine(joystick_origin_axis_);
        }
        else
        {
          if (!get_observation_)
          {
            // ROS_INFO_STREAM("Waiting for observation message...");
            continue;
          }
          checkAndPublishCommandLine(joystick_origin_axis_);
        }
      }
      return;
    }
    bool checkTargetReached()
    {
      const vector_t currentPose = observation_.state.segment<6>(6);
      double xy_error = (currentPose.head(2) - current_target_.head(2)).norm();
      double yaw_error = std::abs(currentPose(3) - current_target_(3));
      return xy_error < TARGET_REACHED_THRESHOLD && yaw_error < TARGET_REACHED_THRESHOLD_YAW;
    }

    bool checkFeetContactPos()
    {
      if (current_desired_gait_ == "walk")
      {
        vector3_t lf_pos_w = vector3_t::Zero();
        vector3_t rf_pos_w = vector3_t::Zero();

        for (int i = 0; i < 4; i++)
        {
          lf_pos_w.head(2) += feet_pos_measured_.segment<2>(i * 3) / 4;
          rf_pos_w.head(2) += feet_pos_measured_.segment<2>(i * 3 + 12) / 4;
        }

        Eigen::Matrix<scalar_t, 3, 1> zyx;
        zyx << -observation_.state.segment<6>(6).tail(3)[0], 0, 0;
        vector3_t lf = getRotationMatrixFromZyxEulerAngles(zyx) * lf_pos_w;
        vector3_t rf = getRotationMatrixFromZyxEulerAngles(zyx) * rf_pos_w;
        vector3_t current_target = getRotationMatrixFromZyxEulerAngles(zyx) * current_target_.head(3);
        if (observation_.mode == ModeNumber::SS && std::abs(lf(0) - rf(0)) < TARGET_REACHED_FEET_THRESHOLD)
          return true;
        return false;
      }
      return true;
    }

    /**
     * @brief 从rosparam读取速度限制并更新c_relative_base_limit_
     * rosparam格式: [linear_x, linear_y, linear_z, angular_x, angular_y, angular_z] (6维)
     * c_relative_base_limit_格式: [linear_x, linear_y, linear_z, angular_z] (4维)
     */
    void updateVelocityLimitsFromParam(bool log_changes = true)
    {
      std::vector<double> velocity_limits;
      if (nodeHandle_.getParam("/velocity_limits", velocity_limits) && velocity_limits.size() == 6)
      {
        // 检查值是否有变化，避免重复设置和打印日志
        bool changed = (std::abs(c_relative_base_limit_[0] - velocity_limits[0]) > 1e-6) ||
                       (std::abs(c_relative_base_limit_[1] - velocity_limits[1]) > 1e-6) ||
                       (std::abs(c_relative_base_limit_[2] - velocity_limits[2]) > 1e-6) ||
                       (std::abs(c_relative_base_limit_[3] - velocity_limits[5]) > 1e-6);
        
        // 将6维速度限制转换为4维格式
        c_relative_base_limit_[0] = velocity_limits[0];  // linear_x
        c_relative_base_limit_[1] = velocity_limits[1];  // linear_y
        c_relative_base_limit_[2] = velocity_limits[2];  // linear_z
        c_relative_base_limit_[3] = velocity_limits[5];  // angular_z
        
        if (changed && log_changes)
        {
          ROS_INFO("[JoyControl] Updated velocity limits from rosparam: [%.2f, %.2f, %.2f, %.2f]",
                   c_relative_base_limit_[0], c_relative_base_limit_[1], 
                   c_relative_base_limit_[2], c_relative_base_limit_[3]);
        }
      }
    }
    

    void checkAndPublishCommandLine(const vector_t &joystick_origin_axis)
    {
      std::lock_guard<std::mutex> lock(target_mutex_);
      geometry_msgs::Twist cmdVel_;
      cmdVel_.linear.x = 0;
      cmdVel_.linear.y = 0;
      cmdVel_.linear.z = 0;
      cmdVel_.angular.x = 0;
      cmdVel_.angular.y = 0;
      cmdVel_.angular.z = 0;
      static bool send_zero_twist = false;

      auto updated = commandLineToTargetTrajectories(joystick_origin_axis, observation_, cmdVel_);
      if (!std::any_of(updated.begin(), updated.end(), [](bool x)
                       { return x; })) // no command line detected
      {
        if (!send_zero_twist)
        {
          std::cout << "[JoyControl] send zero twist" << std::endl;
          cmd_vel_publisher_.publish(cmdVel_);
          send_zero_twist = true;
        }
        return;
      }
      send_zero_twist = false;
      cmd_vel_publisher_.publish(cmdVel_);
    }

    void reloadJoystickMapping(int axis_num, int button_num) {
        sensor_msgs::Joy old_joy_msg_backup;
        old_joy_msg_backup.axes = std::vector<float>(axis_num, 0.0);
        old_joy_msg_backup.buttons = std::vector<int32_t>(button_num, 0);

        for (const auto& button : joyButtonMap_backup) {
            auto it = std::find_if(joyButtonMap.begin(), joyButtonMap.end(),
                [&button](const auto& pair) { return pair.first == button.first; });
            if (it != joyButtonMap.end()) {
                int old_idx = it->second;
                int new_idx = button.second;
                if (old_idx >= 0 && old_idx < static_cast<int>(old_joy_msg_.buttons.size()) &&
                    new_idx >= 0 && new_idx < static_cast<int>(old_joy_msg_backup.buttons.size())) {
                    old_joy_msg_backup.buttons[new_idx] = old_joy_msg_.buttons[old_idx];
                }
            }
        }
        for (const auto& axis : joyAxisMap_backup) {
            auto it = std::find_if(joyAxisMap.begin(), joyAxisMap.end(),
                [&axis](const auto& pair) { return pair.first == axis.first; });
            if (it != joyAxisMap.end()) {
                int old_ax = it->second;
                int new_ax = axis.second;
                if (old_ax >= 0 && old_ax < static_cast<int>(old_joy_msg_.axes.size()) &&
                    new_ax >= 0 && new_ax < static_cast<int>(old_joy_msg_backup.axes.size())) {
                    old_joy_msg_backup.axes[new_ax] = old_joy_msg_.axes[old_ax];
                }
            }
        }

        old_joy_msg_.buttons = old_joy_msg_backup.buttons;
        old_joy_msg_.axes = old_joy_msg_backup.axes;

        std::map<std::string, int> tempButtonMap = joyButtonMap;
        std::map<std::string, int> tempAxisMap = joyAxisMap;
        joyButtonMap = joyButtonMap_backup;
        joyAxisMap = joyAxisMap_backup;
        joyButtonMap_backup = tempButtonMap;
        joyAxisMap_backup = tempAxisMap;
    }

    bool setJoyTopicCallback(kuavo_msgs::SetJoyTopic::Request &req,
                           kuavo_msgs::SetJoyTopic::Response &res)
    {
      try {
        // 取消当前的订阅
        joy_sub_.shutdown();
        
        // 更新话题名并重新订阅
        current_joy_topic_ = req.topic_name;
        joy_sub_ = nodeHandle_.subscribe(current_joy_topic_, 10, &JoyControl::joyCallback, this);
        
        ROS_INFO_STREAM("成功切换Joy话题到: " << current_joy_topic_);
        res.success = true;
        res.message = "成功切换Joy话题";
        return true;
      } catch (const std::exception& e) {
        ROS_ERROR_STREAM("切换Joy话题失败: " << e.what());
        res.success = false;
        res.message = std::string("切换Joy话题失败: ") + e.what();
        return false;
      }
    }

    bool executeCommand(const std::string& command_name)
    {
      auto it = commands_map_.find(command_name);
      if (it == commands_map_.end())
      {
        ROS_ERROR_STREAM("[JoyControl] Command not found: " << command_name);
        return false;
      }
      
      const Command_t& cmd = it->second;
      std::cout << "[JoyControl] Executing command: " << cmd.name << " (" << cmd.description << ")" << std::endl;
      
      if (cmd.type.find("shell") != std::string::npos)
      {
        // 使用异步线程执行shell命令，避免阻塞ROS回调
        if (command_future_.valid() && command_future_.wait_for(std::chrono::milliseconds(0)) == std::future_status::timeout)
        {
          std::cout << "[JoyControl] Previous command is still running, skipping: " << command_name << std::endl;
          return false;
        }
        
        command_future_ = std::async(std::launch::async, [this, cmd]() {
          return executeShellCommand(cmd.value);
        });
        
        return true;
      }
      else
      {
        std::cerr << "[JoyControl] Unknown command type: " << cmd.type << std::endl;
        return false;
      }
    }

    bool executeShellCommand(const std::string& shell_command)
    {
      
      // 直接执行shell命令，先切换到repo_root_path目录
      std::string command = "cd " + repo_root_path_ + " && " + shell_command;
      std::cout << "execute command: " << command << std::endl;
      int result = system(command.c_str());
      
      if (result == 0)
      {
        std::cout << "[JoyControl] 命令执行成功: " << shell_command << std::endl;
        return true;
      }
      else
      {
        std::cerr << "[JoyControl] 命令执行失败，返回值 " << result << ": " << shell_command << std::endl;
        return false;
      }
    }

   

  private:
    void feetCallback(const std_msgs::Float64MultiArray::ConstPtr &feet_msg)
    {
      feet_pos_measured_ = Eigen::Map<const Eigen::VectorXd>(feet_msg->data.data(), feet_msg->data.size());
    }
    void armCtrlModeCallback(const std_msgs::Float64MultiArray::ConstPtr &mode_msg)
    {
      if(mode_msg->data.size() == 2)
      {
        arm_ctrl_mode_ = static_cast<int>(mode_msg->data[1]); //获取手臂控制模式
      }
    }
    void robotActionStateCallback(const humanoid_plan_arm_trajectory::RobotActionState::ConstPtr &msg)
    {
      // state: 0=失败/未执行, 1=执行中/成功
      // 当state为1时，表示有动作正在执行
      robot_action_executing_ = (msg->state == 1);
    }
    bool isControllerSwitching()
    {
      bool switching = controller_switching_ && !last_controller_switch_time_.isZero();
      if (switching)
      {
        const double elapsed = (ros::Time::now() - last_controller_switch_time_).toSec();
        if (elapsed > controller_switch_time)
        {
          controller_switching_ = false;
          switching = false;
        }
      }
      return switching;
    }
    void joyCallback(const sensor_msgs::Joy::ConstPtr &joy_msg)
    {
      vector_t joystickOriginAxisFilter_ = vector_t::Zero(6);
      vector_t joystickOriginAxisTemp_ = vector_t::Zero(6);
      double alpha_ = joystickSensitivity / 1000;

      if (joy_msg->axes.size() != JOYSTICK_AXIS_NUM || (joy_msg->buttons.size() != JOYSTICK_BEITONG_BUTTON_NUM && joy_msg->buttons.size() != JOYSTICK_XBOX_BUTTON_NUM)) {
        return;
      }
      if (std::any_of(joy_msg->buttons.begin(), joy_msg->buttons.end(), [](float button) {
              return std::abs(button) > 1;
          }))
      {
        std::cout << "invalide joy msg"<<std::endl;
        return;
      }

      // if (old_joy_msg_.buttons.size() == JOYSTICK_BEITONG_BUTTON_NUM && joy_msg->buttons.size() == JOYSTICK_XBOX_BUTTON_NUM)
      // {
      //   nodeHandle_.setParam("joystick_type", JOYSTICK_XBOX_MAP_JSON);
      //   std::string channel_map_path = ros::package::getPath("humanoid_controllers") + "/launch/joy/" + JOYSTICK_XBOX_MAP_JSON + ".json";
      //   nodeHandle_.setParam("channel_map_path", channel_map_path);
      //   ROS_WARN("[JoyController]: Joystick data mapping has changed from BEITONG to X-Box");
      //   reloadJoystickMapping(JOYSTICK_AXIS_NUM, JOYSTICK_XBOX_BUTTON_NUM);
      // }
      // if(old_joy_msg_.buttons.size() == JOYSTICK_XBOX_BUTTON_NUM && joy_msg->buttons.size() == JOYSTICK_BEITONG_BUTTON_NUM)
      // {
      //   nodeHandle_.setParam("joystick_type", JOYSTICK_BEITONG_MAP_JSON);
      //   std::string channel_map_path = ros::package::getPath("humanoid_controllers") + "/launch/joy/" + JOYSTICK_BEITONG_MAP_JSON + ".json";
      //   nodeHandle_.setParam("channel_map_path", channel_map_path);
      //   ROS_WARN("[JoyController]: Joystick data mapping has changed from X-Box to BEITONG");
      //   reloadJoystickMapping(JOYSTICK_AXIS_NUM, JOYSTICK_BEITONG_BUTTON_NUM);
      //   // 更新old_joy_msg并跳过本次处理，避免状态混乱
      //   old_joy_msg_ = *joy_msg;
      //   return;
      // }

      if (rb_version_.major() != 1)
      {
        int back_idx = joyButtonMap["BUTTON_BACK"];
        int start_idx = joyButtonMap["BUTTON_START"];

        bool back_pressed = false;
        bool start_pressed = false;

        if (back_idx < joy_msg->buttons.size()) {
            back_pressed = joy_msg->buttons[back_idx];
        }

        if (start_idx < joy_msg->buttons.size()) {
            start_pressed = joy_msg->buttons[start_idx];
        }

        if (back_pressed){
          callTerminateSrv();
          old_joy_msg_ = *joy_msg;  // 更新旧状态，避免重复触发
          return;
        }

        bool old_start_pressed = false;
        if (start_idx < old_joy_msg_.buttons.size()) {
            old_start_pressed = old_joy_msg_.buttons[start_idx];
        }

        if (!old_start_pressed && start_pressed){
          callRealInitializeSrv();
          old_joy_msg_ = *joy_msg;  // 更新旧状态，避免重复触发
          return;
        }
      }
      else
      {
        if(joy_msg->buttons[joyButtonMap["BUTTON_M1"]] || joy_msg->buttons[joyButtonMap["BUTTON_M2"]])
        {
          return;
        }
      }

      // Rate-limited check: only allow operations after robot is launched       
      if (!robot_launched_ && real_)
      {
        ros::Time now = ros::Time::now();
        if ((now - last_status_check_time_).toSec() >= 1.0)
        {
          last_status_check_time_ = now;
          if (real_launch_status_client_.exists())
          {
            std_srvs::Trigger srv;
            if (real_launch_status_client_.call(srv))
            {
              robot_launched_ = (srv.response.message == "launched");
            }
          }
        }
        old_joy_msg_ = *joy_msg;
        return;
      }


      if(joy_msg->axes[joyAxisMap["AXIS_RIGHT_RT"]] < -0.5)
      {
        head_control_active_ = true;  // 标记进入头部控制模式
        // 组合键控制头部，使用通用平滑控制函数
        double head_yaw_raw = joy_msg->axes[joyAxisMap["AXIS_RIGHT_STICK_YAW"]];
        double head_pitch_raw = joy_msg->axes[joyAxisMap["AXIS_RIGHT_STICK_Z"]];
        
        // 使用通用平滑控制函数处理yaw和pitch（各自独立的时间戳）
        smoothAngleControl(head_yaw_raw, current_head_yaw_, 80.0, 
                          HEAD_MAX_VELOCITY_DEG_PER_SEC_, HEAD_DEAD_ZONE_, 
                          last_head_yaw_control_time_);
        smoothAngleControl(-head_pitch_raw, current_head_pitch_, 25.0, 
                          HEAD_MAX_VELOCITY_DEG_PER_SEC_, HEAD_DEAD_ZONE_, 
                          last_head_pitch_control_time_);
        
        // 发布头部控制命令
        controlHead(current_head_yaw_, current_head_pitch_);
        // return;

        // RT + X: 切换到上一个控制器
        if (!old_joy_msg_.buttons[joyButtonMap["BUTTON_RL"]] && joy_msg->buttons[joyButtonMap["BUTTON_RL"]])
        {
          ROS_INFO("[JoyControl] RT+BUTTON_RL: switch to previous controller");
          switchToPreviousController();
          old_joy_msg_ = *joy_msg;
          return;
        }

        if(!joy_execute_action_)
        {
        joystickOriginAxisTemp_.head(4) << joy_msg->axes[joyAxisMap["AXIS_LEFT_STICK_X"]], joy_msg->axes[joyAxisMap["AXIS_LEFT_STICK_Y"]], joystick_origin_axis_[2], joystick_origin_axis_[3];
        // 行为树控制
        if(joy_msg->buttons[joyButtonMap["BUTTON_STANCE"]])
        {
          ROS_INFO("Start grab box demo");
          enableGrabBoxDemo(true);
        }
        if(joy_msg->buttons[joyButtonMap["BUTTON_TROT"]])
        {
          ROS_INFO("Stop grab box demo");
          enableGrabBoxDemo(false);
        }
        if(joy_msg->buttons[joyButtonMap["BUTTON_RL"]])
        {
          ROS_INFO("Reset grab box demo");
          resetGrabBoxDemo(true);
        }
        if (!old_joy_msg_.buttons[joyButtonMap["BUTTON_WALK"]] && joy_msg->buttons[joyButtonMap["BUTTON_WALK"]])
        {
          if (stair_detection_enabled_)
          {
            executeCommand("stop_stair_detect");
          }
          else{
            executeCommand("start_stair_detect");
          }
        }
        return;
        }
        old_joy_msg_ = *joy_msg;
        return;
      }

      // 检测退出头部控制状态（从头部控制状态退出到非头部控制状态），平滑回正
      if (head_control_active_ && joy_msg->axes[joyAxisMap["AXIS_RIGHT_RT"]] >= -0.5)
      {
        // 使用通用平滑控制函数回正到0度（输入为0表示回正，各自独立的时间戳）
        smoothAngleControl(0.0, current_head_yaw_, 80.0, 
                          HEAD_MAX_VELOCITY_DEG_PER_SEC_, HEAD_DEAD_ZONE_, 
                          last_head_yaw_control_time_);
        smoothAngleControl(0.0, current_head_pitch_, 25.0, 
                          HEAD_MAX_VELOCITY_DEG_PER_SEC_, HEAD_DEAD_ZONE_, 
                          last_head_pitch_control_time_);
        
        // 如果已经接近0度（小于1度），直接设为0
        if (std::abs(current_head_yaw_) < 1.0 && std::abs(current_head_pitch_) < 1.0)
        {
          current_head_yaw_ = 0.0;
          current_head_pitch_ = 0.0;
          head_control_active_ = false;
        }
        else
        {
          // 继续发布回正命令
          controlHead(current_head_yaw_, current_head_pitch_);
        }
      }
      else if (joy_msg->axes[joyAxisMap["AXIS_RIGHT_RT"]] < -0.5)
      {
        head_control_active_ = true;
      }

      // 检测退出腰部控制状态（从腰部控制状态退出到非腰部控制状态）
      if (waist_control_active_ && joy_msg->axes[joyAxisMap["AXIS_LEFT_LT"]] >= -0.5)
      {
        // 发布 false，禁用腰部控制（只有腰部dof>0时才发布）
        if (waist_dof_ > 0)
        {
          std_msgs::Bool enable_msg;
          enable_msg.data = false;
          // enable_waist_control_pub_.publish(enable_msg);
          waist_control_active_ = false;
          ROS_INFO("[JoyControl] Exited waist control mode, disabled waist control");
        }
      }
      
      if(joy_msg->axes[joyAxisMap["AXIS_LEFT_LT"]] < -0.5)
      {        
        if(!joy_execute_action_)
        {
        if (!old_joy_msg_.buttons[joyButtonMap["BUTTON_STANCE"]] && joy_msg->buttons[joyButtonMap["BUTTON_STANCE"]])
        {
          pubSlopePlanning(false);
        }
        else if (!old_joy_msg_.buttons[joyButtonMap["BUTTON_WALK"]] && joy_msg->buttons[joyButtonMap["BUTTON_WALK"]])
        {
          pubSlopePlanning(true);
        }
        else if (!old_joy_msg_.buttons[joyButtonMap["BUTTON_TROT"]] && joy_msg->buttons[joyButtonMap["BUTTON_TROT"]])
        {
          executeCommand("stairclimb");
        }
        }

        // 组合键控制腰部
        // 检查是否有动作正在执行，如果有则禁用转腰控制以避免冲突
        bool action_executing = false;
        
        // 检查ROS参数标志（用于某些动作执行场景，如太极动作）
        if (nodeHandle_.hasParam("/taiji_executing"))
        {
          nodeHandle_.getParam("/taiji_executing", action_executing);
        }
        
        // 检查动作状态话题（用于通过/execute_arm_action服务执行的动作）
        if (!action_executing)
        {
          action_executing = robot_action_executing_;
        }
        
        // 只有在没有动作执行时才允许转腰控制，且只有腰部dof>0时才进行腰部控制
        if (!action_executing && waist_dof_ > 0)
        {
          // 检测进入腰部控制状态（从非腰部控制状态进入）
          if (!waist_control_active_)
          {
            // 发布 true，启用腰部控制
            std_msgs::Bool enable_msg;
            enable_msg.data = true;
            // enable_waist_control_pub_.publish(enable_msg);
            waist_control_active_ = true;
            ROS_INFO("[JoyControl] Entered waist control mode, enabled waist control");
          }
          
          double waist_yaw = joy_msg->axes[joyAxisMap["AXIS_RIGHT_STICK_YAW"]];
          waist_yaw = waist_yaw_max_angle_deg_ * waist_yaw;   // +- waist_yaw_max_angle_deg_ deg
          // std::cout << "waist_yaw: " << waist_yaw << std::endl;
          controlWaist(waist_yaw);
        }
        
        old_joy_msg_ = *joy_msg;
        return;
      }
    
      // 非辅助模式下才可控行走
      if(joy_msg->axes[joyAxisMap["AXIS_RIGHT_RT"]] > -0.5 && joy_msg->axes[joyAxisMap["AXIS_LEFT_LT"]] > -0.5 && axes_input_enabled_)
        joystickOriginAxisTemp_.head(4) << joy_msg->axes[joyAxisMap["AXIS_LEFT_STICK_X"]], joy_msg->axes[joyAxisMap["AXIS_LEFT_STICK_Y"]], joy_msg->axes[joyAxisMap["AXIS_RIGHT_STICK_Z"]], joy_msg->axes[joyAxisMap["AXIS_RIGHT_STICK_YAW"]];
    
      joystickOriginAxisFilter_ = joystickOriginAxisTemp_;
      // for(int i=0;i<4;i++)
      // {
      //   joystickOriginAxisFilter_[i] = alpha_ * joystickOriginAxisTemp_[i] + (1 - alpha_) * joystick_origin_axis_[i];
      // }
      joystickOriginAxisFilter_.head(4) = joystickFilter_.update(joystickOriginAxisTemp_.head(4));
      for (size_t i = 0; i < 4; i++)
      {
        joystickOriginAxisFilter_(i) = std::max(-1.0, std::min(1.0, joystickOriginAxisFilter_(i)));
      }
      joystick_origin_axis_ = joystickOriginAxisFilter_;
      // joystick_origin_axis_.head(4) << joy_msg->axes[joyAxisMap["AXIS_LEFT_STICK_X"]], joy_msg->axes[joyAxisMap["AXIS_LEFT_STICK_Y"]], joy_msg->axes[joyAxisMap["AXIS_RIGHT_STICK_Z"]], joy_msg->axes[joyAxisMap["AXIS_RIGHT_STICK_YAW"]];
      
      if (joy_msg->buttons[joyButtonMap["BUTTON_LB"]])// 按下左侧侧键，切换模式
      {
        if (!old_joy_msg_.buttons[joyButtonMap["BUTTON_STANCE"]] && joy_msg->buttons[joyButtonMap["BUTTON_STANCE"]])
        pubModeGaitScale(0.9);
        else if (!old_joy_msg_.buttons[joyButtonMap["BUTTON_WALK"]] && joy_msg->buttons[joyButtonMap["BUTTON_WALK"]])
        pubModeGaitScale(1.1);
        else if (!old_joy_msg_.buttons[joyButtonMap["BUTTON_TROT"]] && joy_msg->buttons[joyButtonMap["BUTTON_TROT"]])
        {
          arm_ctrl_mode_ = (arm_ctrl_mode_ > 0)? 0 : 1;
          callArmControlService(arm_ctrl_mode_);
        }
        else if (!old_joy_msg_.buttons[joyButtonMap["BUTTON_RL"]] && joy_msg->buttons[joyButtonMap["BUTTON_RL"]])
        {
          kuavo_msgs::robotHandPosition msg;
          const int fingers = 6;
          msg.left_hand_position.resize(fingers);
          msg.right_hand_position.resize(fingers);
          if (!hand_closed_)
          {
            std::fill(msg.left_hand_position.begin(), msg.left_hand_position.end(), 100);
            std::fill(msg.right_hand_position.begin(), msg.right_hand_position.end(), 100);
          }
          else
          {
            std::fill(msg.left_hand_position.begin(), msg.left_hand_position.end(), 0);
            std::fill(msg.right_hand_position.begin(), msg.right_hand_position.end(), 0);
          }
          ROS_INFO("publish hand position: %s", hand_closed_ ? "close" : "open");
          hand_closed_ = !hand_closed_;
          hand_position_pub_.publish(msg);
        }
      }
      else if (joy_msg->buttons[joyButtonMap["BUTTON_RB"]])// 按下右侧侧键，切换模式
      {
        if (!old_joy_msg_.buttons[joyButtonMap["BUTTON_STANCE"]] && joy_msg->buttons[joyButtonMap["BUTTON_STANCE"]])
        {
          c_relative_base_limit_[0] -= (c_relative_base_limit_[0] > 0.1) ? 0.05 : 0.0;
          c_relative_base_limit_[3] -= (c_relative_base_limit_[3] > 0.1) ? 0.05 : 0.0;
          std::cout << "cmdvelLinearXLimit: " << c_relative_base_limit_[0] << "\n"
                    << "cmdvelAngularYAWLimit: " << c_relative_base_limit_[3] << std::endl;
        }
        else if (!old_joy_msg_.buttons[joyButtonMap["BUTTON_WALK"]] && joy_msg->buttons[joyButtonMap["BUTTON_WALK"]])
        {
          c_relative_base_limit_[0] += 0.05;
          c_relative_base_limit_[3] += 0.05;
          std::cout << "cmdvelLinearXLimit: " << c_relative_base_limit_[0] << "\n"
                    << "cmdvelAngularYAWLimit: " << c_relative_base_limit_[3] << std::endl;
        }
        // RB + BUTTON_RL(X): 起身
        if (!old_joy_msg_.buttons[joyButtonMap["BUTTON_RL"]] && joy_msg->buttons[joyButtonMap["BUTTON_RL"]])
        {
          callTriggerFallStandUpSrv();
          old_joy_msg_ = *joy_msg;
          return;
        }
        // RB + BUTTON_WALK(Y): 切换到 DanceController
        if (!old_joy_msg_.buttons[joyButtonMap["BUTTON_WALK"]] && joy_msg->buttons[joyButtonMap["BUTTON_WALK"]])
        {
          ROS_INFO("Switching to DanceController via RB+Y");
          callTriggerDanceSrv();
          old_joy_msg_ = *joy_msg;
          return;
        }
        // RB + BUTTON_TROT(B): 触发倒地逻辑
        if (!old_joy_msg_.buttons[joyButtonMap["BUTTON_TROT"]] && joy_msg->buttons[joyButtonMap["BUTTON_TROT"]])
        {
          callSetFallDownStateSrv();
          old_joy_msg_ = *joy_msg;
          return;
        }
      }
      else
      {
        if (robot_type_ == 2)
        {
          checkGaitSwitchCommand(joy_msg);
        }
      }

      vector_t button_trigger_axis = vector_t::Zero(6);
      if (axes_input_enabled_)
      {
        if (joy_msg->axes[joyAxisMap["AXIS_FORWARD_BACK_TRIGGER"]])
        {
          button_trigger_axis[0] = joy_msg->axes[joyAxisMap["AXIS_FORWARD_BACK_TRIGGER"]];
          checkAndPublishCommandLine(button_trigger_axis);
          joystick_origin_axis_ = button_trigger_axis;
        }
        else if (joy_msg->axes[joyAxisMap["AXIS_LEFT_RIGHT_TRIGGER"]])
        {
          button_trigger_axis[1] = joy_msg->axes[joyAxisMap["AXIS_LEFT_RIGHT_TRIGGER"]];
          checkAndPublishCommandLine(button_trigger_axis);
          joystick_origin_axis_ = button_trigger_axis;
        }
      }
      old_joy_msg_ = *joy_msg;

      // 控制器切换阶段，禁用摇杆输入和除了BUTTON_RL外的button
      if (isControllerSwitching())
      {
        joystick_origin_axis_.setZero();

        sensor_msgs::Joy modified_joy_msg = *joy_msg;
        int button_rl_index = joyButtonMap["BUTTON_RL"];
        for (size_t i = 0; i < modified_joy_msg.buttons.size(); i++)
        {
          if (i != static_cast<size_t>(button_rl_index))
          {
            modified_joy_msg.buttons[i] = 0;
          }
        }
        old_joy_msg_ = modified_joy_msg;
      }
    }

    void checkGaitSwitchCommand(const sensor_msgs::Joy::ConstPtr &joy_msg)
    {
      // 控制器切换阶段，只允许BUTTON_RL用于切换控制器，其他按钮禁用
      if (isControllerSwitching())
      {
        // 只检查BUTTON_RL用于切换控制器
        // if (!old_joy_msg_.buttons[joyButtonMap["BUTTON_RL"]] && joy_msg->buttons[joyButtonMap["BUTTON_RL"]])
        // {
        //   ROS_INFO("[JoyControl] switch to next controller");
        //   switchToNextController();
        // }
        return;
      }

      // 有摇杆数据不可以步态切换
      if (
        std::abs(joy_msg->axes[joyAxisMap["AXIS_LEFT_STICK_Y"]]) > DEAD_ZONE ||
        std::abs(joy_msg->axes[joyAxisMap["AXIS_LEFT_STICK_X"]]) > DEAD_ZONE ||
        std::abs(joy_msg->axes[joyAxisMap["AXIS_RIGHT_STICK_YAW"]]) > DEAD_ZONE ||
        std::abs(joy_msg->axes[joyAxisMap["AXIS_RIGHT_STICK_Z"]]) > DEAD_ZONE ||
        std::abs(joy_msg->axes[joyAxisMap["AXIS_LEFT_RIGHT_TRIGGER"]]) > DEAD_ZONE ||
        std::abs(joy_msg->axes[joyAxisMap["AXIS_FORWARD_BACK_TRIGGER"]]) > DEAD_ZONE
      ) {
        return;
      }
      // 检查是否有gait切换指令
      if (!old_joy_msg_.buttons[joyButtonMap["BUTTON_STANCE"]] && joy_msg->buttons[joyButtonMap["BUTTON_STANCE"]])
      {
        publishGaitTemplate("stance");
      }
      else if (!old_joy_msg_.buttons[joyButtonMap["BUTTON_TROT"]] && joy_msg->buttons[joyButtonMap["BUTTON_TROT"]])
      {
        if (!joy_execute_action_)
        {
          publishGaitTemplate("trot");
        }
        else
        {
          // 使用 TROT 作为“遥感/方向键输入”开关
          axes_input_enabled_ = !axes_input_enabled_;
          ROS_WARN_STREAM("[JoyControl] Axes input toggled: " << (axes_input_enabled_ ? "ENABLED" : "DISABLED"));
          if (!axes_input_enabled_)
          {
            // 关闭时立即发布零速度，确保立刻停止
            vector_t zero_axis = vector_t::Zero(6);
            joystick_origin_axis_.setZero();
            checkAndPublishCommandLine(zero_axis);
          }
          return;
        }
      }
      else if (!old_joy_msg_.buttons[joyButtonMap["BUTTON_RL"]] && joy_msg->buttons[joyButtonMap["BUTTON_RL"]])
      {
        ROS_INFO("[JoyControl] switch to next controller");
        // Get controller list and switch to next
        switchToNextController();
        return;
      }
      else if (!old_joy_msg_.buttons[joyButtonMap["BUTTON_WALK"]] && joy_msg->buttons[joyButtonMap["BUTTON_WALK"]])
      {
        // RL控制器下，ROBAN2禁用按Y进入踏步
        if (is_rl_controller_ && rb_version_.major() == 1)
        {
          ROS_WARN("[JoyControl] Walk gait is disabled for ROBAN2 under RL controller");
          return;
        }
        publishGaitTemplate("walk");
      }
      else
      {
        return;
      }

      std::cout << "joycmd switch to: " << current_desired_gait_ << std::endl;
      std::cout << "turn " << (current_desired_gait_ == "stance" ? "on " : "off ") << " auto stance mode" << std::endl;
      auto_stance_mode_ = (current_desired_gait_ == "stance");
    }

    void publishGaitTemplate(const std::string &gaitName)
    {
      // Skip gait publishing in wheel mode
      if (robot_type_ == 1)
      {
        ROS_WARN_STREAM("[JoyControl] Gait publishing not supported in wheel mode, ignoring gait: " << gaitName);
        return;
      }
      
      // 发布对应的gait模板
      humanoid::ModeSequenceTemplate modeSequenceTemplate = gait_map_.at(gaitName);
      mode_sequence_template_publisher_.publish(createModeSequenceTemplateMsg(modeSequenceTemplate));
      current_desired_gait_ = gaitName;
      
      // 发布gait名字请求
      std_msgs::String gait_name_msg;
      gait_name_msg.data = gaitName;
      gait_name_publisher_.publish(gait_name_msg);
      ROS_INFO_STREAM("[JoyControl] Published gait name request: " << gaitName);
      cur_gait_name_ = gaitName;
    }

    void gaitChangeCallback(const std_msgs::String::ConstPtr& msg) 
    {
      // Skip gait change in wheel mode
      if (robot_type_ == 1)
      {
        ROS_WARN_STREAM("[JoyControl] Gait change not supported in wheel mode, ignoring request: " << msg->data);
        return;
      }
      
      const std::string &req = msg->data;
      ROS_INFO_STREAM("[JoyControl] Gait change request: " << req);

      if (req != "walk" && req != "stance")
      {
        ROS_WARN_STREAM("[JoyControl] Invalid gait '" << req 
                        << "'. Only 'walk' or 'stance' allowed.");
        return;
      }

      if (req == current_desired_gait_)
      {
        ROS_INFO_STREAM("[JoyControl] Already in '" << req << "', no change.");
        return;
      }

      ROS_INFO_STREAM("[JoyControl] Switching gait from '" 
                      << current_desired_gait_ << "' to '" << req << "'");
      publishGaitTemplate(req);
    }

    void observationCallback(const ocs2_msgs::mpc_observation::ConstPtr &observation_msg)
    {
      observation_ = ros_msg_conversions::readObservationMsg(*observation_msg);
      get_observation_ = true;
    }

    scalar_t estimateTimeToTarget(const vector_t &desiredBaseDisplacement)
    {
      const scalar_t &dx = desiredBaseDisplacement(0);
      const scalar_t &dy = desiredBaseDisplacement(1);
      const scalar_t &dyaw = desiredBaseDisplacement(3);
      const scalar_t rotationTime = std::abs(dyaw) / target_rotation_velocity_;
      const scalar_t displacement = std::sqrt(dx * dx + dy * dy);
      const scalar_t displacementTime = displacement / target_displacement_velocity_;
      return std::max(rotationTime, displacementTime);
    }

    /**
     * Converts command line to TargetTrajectories.
     * @param [in] commad_line_target_ : [deltaX, deltaY, deltaZ, deltaYaw]
     * @param [in] observation : the current observation
     */
    std::vector<bool> commandLineToTargetTrajectories(const vector_t &joystick_origin_axis, const SystemObservation &observation, geometry_msgs::Twist &cmdVel)
    {
      std::vector<bool> updated(6, false);
      Eigen::VectorXd limit_vector_negative(4);
      limit_vector_negative << c_relative_base_limit_[0], c_relative_base_limit_[1], 
                               std::fabs(squatHeightMin_), c_relative_base_limit_[3];
      Eigen::VectorXd limit_vector_positive(4);
      limit_vector_positive << c_relative_base_limit_[0], c_relative_base_limit_[1], 
                               std::fabs(squatHeightMax_), c_relative_base_limit_[3];
      if (joystick_origin_axis.cwiseAbs().maxCoeff() < DEAD_ZONE)
        return updated; // command line is zero, do nothing

      for (int i = 0; i < 4; i++) {
        if (joystick_origin_axis[i] >= 0) {
            commad_line_target_[i] = joystick_origin_axis[i] * limit_vector_positive[i];
        } else {
            commad_line_target_[i] = joystick_origin_axis[i] * limit_vector_negative[i];
        }
      }

      const vector_t currentPose = observation.state.segment<6>(6);
      // vector_t target(6);
      if (joystick_origin_axis.head(2).cwiseAbs().maxCoeff() > DEAD_ZONE)
      { // base p_x, p_y are relative to current state
        // double dx = commad_line_target_(0) * cos(currentPose(3)) - commad_line_target_(1) * sin(currentPose(3));
        // double dy = commad_line_target_(0) * sin(currentPose(3)) + commad_line_target_(1) * cos(currentPose(3));
        // current_target_(0) = currentPose(0) + dx;
        // current_target_(1) = currentPose(1) + dy;
        cmdVel.linear.x = commad_line_target_(0);
        cmdVel.linear.y = commad_line_target_(1);
        updated[0] = true;
        updated[1] = true;
        // std::cout << "base displacement: " << dx << ", " << dy << std::endl;
      }
      // base z relative to the default height
      if (std::abs(joystick_origin_axis(2)) > DEAD_ZONE)
      {
        updated[2] = true;
        // current_target_(2) = com_height_ + commad_line_target_(2);
        cmdVel.linear.z = commad_line_target_(2);
        ROS_INFO_THROTTLE(1.0, "base height: %f", current_target_(2));
      }
      else
      {
        // current_target_(2) = com_height_;
        cmdVel.linear.z = 0.0;
      }

      /*******************下蹲超过阈值则不允许踏步***********************/
      static bool last_auto_gait_state = true;
      const double height_diff_max = 0.1;  // 10厘米

      if (std::fabs(cmdVel.linear.z) > height_diff_max && cur_gait_name_ == "stance")
      {
        if(last_auto_gait_state == true)
        {
          changeAutoGaitStatus(false);
          last_auto_gait_state = false;
        }
      }
      else
      {
        if(last_auto_gait_state == false)
        {
          changeAutoGaitStatus(true);
          last_auto_gait_state = true;
        }
      }
      /**************************************************************/

      // theta_z relative to current
      if (std::abs(joystick_origin_axis(3)) > DEAD_ZONE)
      {
        updated[3] = true;
        // current_target_(3) = currentPose(3) + commad_line_target_(3) * M_PI / 180.0;
        cmdVel.angular.z = commad_line_target_(3);
      }

      return updated;
    }

    inline void pubModeGaitScale(float scale)
    {
      // Skip gait scale publishing in wheel mode
      if (robot_type_ == 1)
      {
        ROS_WARN_STREAM("[JoyControl] Gait scale publishing not supported in wheel mode, ignoring scale: " << scale);
        return;
      }
      total_mode_scale_ *= scale;
      ROS_INFO_STREAM("[JoyControl] Publish scale: " << scale << ", Total mode scale: " << total_mode_scale_);
      std_msgs::Float32 msg;
      msg.data = scale;
      mode_scale_publisher_.publish(msg);
    }

    inline void pubSlopePlanning(bool enable)
    {
      ROS_INFO_STREAM("[JoyControl] Publish slope planning: " << (enable ? "enable" : "disable"));
      std_msgs::Bool msg;
      msg.data = enable;
      slope_planning_pub_.publish(msg);
    }

    bool callArmControlService(int mode)
    {
      ros::ServiceClient client = nodeHandle_.serviceClient<kuavo_msgs::changeArmCtrlMode>("humanoid_change_arm_ctrl_mode");
      kuavo_msgs::changeArmCtrlMode srv;
      srv.request.control_mode = mode; 

      if (client.call(srv))
      {
        ROS_INFO("changeArmCtrlMode call succeeded, received response: %s", srv.response.result ? "Success" : "Failure");
        return srv.response.result; 
      }
      else
      {
        ROS_ERROR("Failed to call service change_arm_ctrl_mode");
        return false;
      }
    }

    void callRealInitializeSrv()
    {
      ros::ServiceClient client = nodeHandle_.serviceClient<std_srvs::Trigger>("/humanoid_controller/real_initial_start");
      std_srvs::Trigger srv;

      // 调用服务
      if (client.call(srv))
      {
        ROS_INFO("[JoyControl] Real initialize service call SUCCESS");
      }
      else
      {
        ROS_ERROR("[JoyControl] Real initialize service FAILED, using topic /re_start_robot");
        std_msgs::Bool msg;
        msg.data = true;
        re_start_pub_.publish(msg);
      }
    }
    void callTerminateSrv()
    {
      for (int i = 0; i < 5; i++)
      {
        std_msgs::Bool msg;
        msg.data = true;
        stop_pub_.publish(msg);
        ::ros::Duration(0.1).sleep();
      }
    }
    void callTriggerFallStandUpSrv()
    {
      std::cout << "trigger callTriggerFallStandUpSrv" << std::endl;
      std_srvs::Trigger srv;

      // 调用服务
      if (trigger_fall_stand_up_client_.call(srv))
      {
        ROS_INFO("[JoyControl] Trigger fall stand up service call successful: %s", srv.response.message.c_str());
      }
      else
      {
        ROS_ERROR("[JoyControl] Failed to call trigger_fall_stand_up service");
      }
    }
    
    void callSetFallDownStateSrv()
    {
      std::cout << "trigger callSetFallDownStateSrv" << std::endl;
      std_srvs::SetBool srv;
      srv.request.data = true;  // true = FALL_DOWN, false = STANDING

      // 调用服务
      if (set_fall_down_state_client_.call(srv))
      {
        ROS_INFO("[JoyControl] Set fall down state service call successful: %s", srv.response.message.c_str());
      }
      else
      {
        ROS_ERROR("[JoyControl] Failed to call set_fall_down_state service");
      }
    }

    void callTriggerDanceSrv()
    {
      std::cout << "trigger callTriggerDanceSrv" << std::endl;
      
      // 首先获取当前控制器，再决定切换方向
      kuavo_msgs::getControllerList get_list_srv;
      if (!get_controller_list_client_.call(get_list_srv) || !get_list_srv.response.success)
      {
        ROS_ERROR("[JoyControl] Failed to get current controller");
        return;
      }
      
      std::string current_controller = get_list_srv.response.current_controller;
      ROS_INFO("[JoyControl] Current controller: %s", current_controller.c_str());
      
      // 如果当前已经在dance_controller，切换到amp_controller
      if (current_controller == "dance_controller")
      {
        ROS_INFO("[JoyControl] Switching from DanceController to AmpWalkController");
        if (callSwitchControllerService("amp_controller"))
        {
          ROS_INFO("[JoyControl] Successfully switched to AmpWalkController");
        }
        else
        {
          ROS_ERROR("[JoyControl] Failed to switch to AmpWalkController");
        }
      }
      else
      {
        // 否则切换到dance_controller（通过专门的trigger_dance服务）
        ROS_INFO("[JoyControl] Switching to DanceController");
        std_srvs::Trigger srv;
        if (trigger_dance_client_.call(srv) && srv.response.success)
        {
          ROS_INFO("[JoyControl] Successfully switched to DanceController: %s", srv.response.message.c_str());
        }
        else
        {
          ROS_ERROR("[JoyControl] Failed to switch to DanceController");
        }
      }
    }

    /**
     * @brief 通用的平滑角度控制函数，包含死区处理、速度限制和平滑更新
     * @param raw_input 原始摇杆输入值 [-1.0, 1.0]
     * @param current_angle 当前角度（度），会被更新
     * @param max_angle 最大角度范围（度）
     * @param max_velocity 最大角速度（度/秒）
     * @param dead_zone 死区阈值
     * @param last_control_time 上次控制时间，会被更新
     * @return 更新后的角度（度）
     */
    double smoothAngleControl(double raw_input, double& current_angle, 
                              double max_angle, double max_velocity, 
                              double dead_zone, ros::Time& last_control_time)
    {
      // 计算目标角度
      double target_angle = max_angle * raw_input;
      
      // 应用死区处理
      if (std::abs(raw_input) < dead_zone)
      {
        target_angle = 0.0;  // 回正
      }
      
      // 速度限制和平滑处理
      ros::Time current_time = ros::Time::now();
      double dt = 0.01;  // 默认时间步长（100Hz）
      if (!last_control_time.isZero())
      {
        dt = (current_time - last_control_time).toSec();
        dt = std::max(0.001, std::min(0.1, dt));  // 限制dt在合理范围内
      }
      last_control_time = current_time;
      
      // 计算最大允许变化量（度）
      double max_delta = max_velocity * dt;
      
      // 平滑更新角度
      double angle_delta = target_angle - current_angle;
      if (std::abs(angle_delta) > max_delta)
      {
        angle_delta = (angle_delta > 0) ? max_delta : -max_delta;
      }
      current_angle += angle_delta;
      
      return current_angle;
    }

    void controlHead(double head_yaw, double head_pitch)
    {
      kuavo_msgs::robotHeadMotionData msg;
      msg.joint_data.resize(2);
      msg.joint_data[0] = head_yaw;
      msg.joint_data[1] = head_pitch;
      head_motion_pub_.publish(msg);
    }

    void controlWaist(double waist_yaw)
    {
      kuavo_msgs::robotWaistControl msg;
      msg.header.stamp = ros::Time::now();
      msg.data.data.resize(1);
      msg.data.data[0] = waist_yaw;
      waist_motion_pub_.publish(msg);
    }


    bool enableGrabBoxDemo(bool enable)
    {
      const std::string service_name = "/grab_box_demo/control_bt";
      ros::NodeHandle nh;

      // 等待服务可用
      if (!ros::service::waitForService(service_name, ros::Duration(1))) {
        ROS_ERROR("Service %s not available", service_name.c_str());
        return false;
      }

      // 创建服务代理
      ros::ServiceClient client = nh.serviceClient<std_srvs::SetBool>(service_name);
      std_srvs::SetBool srv;
      srv.request.data = enable;

      // 调用服务
      if (client.call(srv)) {
        ROS_INFO("controlBtTree call succeeded, received response: %s", srv.response.success ? "Success" : "Failure");
        return srv.response.success; // 服务调用成功
      } else {
        ROS_ERROR("Failed to call service %s", service_name.c_str());
        return false; // 服务调用失败
      }
    }

    bool resetGrabBoxDemo(bool reset)
    {
      const std::string service_name = "/grab_box_demo/reset_bt";
      ros::NodeHandle nh;

      // 等待服务可用
      if (!ros::service::waitForService(service_name, ros::Duration(1))) {
        ROS_ERROR("Service %s not available", service_name.c_str());
        return false;
      }

      // 创建服务代理
      ros::ServiceClient client = nh.serviceClient<std_srvs::SetBool>(service_name);
      std_srvs::SetBool srv;
      srv.request.data = reset;

      // 调用服务
      if (client.call(srv)) {
        ROS_INFO("resetGrabBoxDemo call succeeded, received response: %s", srv.response.success ? "Success" : "Failure");
        return srv.response.success; // 服务调用成功
      } else {
        ROS_ERROR("Failed to call service %s", service_name.c_str());
        return false; // 服务调用失败
      }
    }

    bool callExecuteArmAction(const std::string &action_name)
    {
      // 检查是否有动作正在执行，如果有则不允许触发新的手臂动作
      bool action_executing = false;
      
      // 检查ROS参数标志（用于某些动作执行场景，如太极动作）
      if (nodeHandle_.hasParam("/taiji_executing"))
      {
        nodeHandle_.getParam("/taiji_executing", action_executing);
      }
      
      // 检查动作状态话题（用于通过/execute_arm_action服务执行的动作）
      if (!action_executing)
      {
        action_executing = robot_action_executing_;
      }
      
      if (action_executing)
      {
        ROS_WARN("[JoyControl] Cannot execute arm action '%s': another action is currently executing", action_name.c_str());
        return false;
      }
      
      kuavo_msgs::ExecuteArmAction srv;
      srv.request.action_name = action_name;
      const std::string service_name = "/execute_arm_action";

      if (!execute_arm_action_client_.exists())
      {
        ros::service::waitForService(service_name, ros::Duration(1.0));
      }

      if (execute_arm_action_client_.call(srv))
      {
        ROS_INFO("[JoyControl] ExecuteArmAction('%s') -> %s: %s",
                 action_name.c_str(), srv.response.success ? "Success" : "Failure",
                 srv.response.message.c_str());
        return srv.response.success;
      }
      else
      {
        ROS_ERROR("[JoyControl] Failed to call service %s", service_name.c_str());
        return false;
      }
    }
    bool callSwitchControllerService(const std::string& controller_name)
    {
      kuavo_msgs::switchController srv;
      srv.request.controller_name = controller_name;
      
      if (switch_controller_client_.call(srv)) {
        ROS_INFO("Controller switch service call successful, result: %s", srv.response.message.c_str());
        return srv.response.success;
      } else {
        ROS_ERROR("Controller switch service call failed: %s", controller_name.c_str());
        return false;
      }
    }

    bool getControllerList(std::vector<std::string>& controller_list)
    {
      kuavo_msgs::getControllerList srv;
      
      if (get_controller_list_client_.call(srv)) {
        if (srv.response.success) {
          controller_list = srv.response.controller_names;
          ROS_INFO("Get controller list successful: total %d controllers", srv.response.count);
          for (size_t i = 0; i < controller_list.size(); ++i) {
            ROS_INFO("  [%zu] %s", i, controller_list[i].c_str());
          }
          return true;
        } else {
          ROS_ERROR("Get controller list failed: %s", srv.response.message.c_str());
          return false;
        }
      } else {
        ROS_ERROR("Get controller list service call failed");
        return false;
      }
    }

    bool switchToNextController()
    {
      kuavo_msgs::switchToNextController srv;

      if (switch_to_next_controller_client_.call(srv)) {
        if (srv.response.success) {
          ROS_INFO("Switch to next controller successful: %s", srv.response.message.c_str());
          ROS_INFO("Switched from %s (index: %d) to %s (index: %d)", 
                   srv.response.current_controller.c_str(), srv.response.current_index,
                   srv.response.next_controller.c_str(), srv.response.next_index);
          return true;
        } else {
          ROS_ERROR("Switch to next controller failed: %s", srv.response.message.c_str());
          return false;
        }
      } else {
        ROS_ERROR("Switch to next controller service call failed");
        return false;
      }
    }

    bool switchToPreviousController()
    {
      kuavo_msgs::switchToNextController srv;
      
      if (switch_to_previous_controller_client_.call(srv)) {
        if (srv.response.success) {
          ROS_INFO("Switch to previous controller successful: %s", srv.response.message.c_str());
          ROS_INFO("Switched from %s (index: %d) to %s (index: %d)", 
                   srv.response.current_controller.c_str(), srv.response.current_index,
                   srv.response.next_controller.c_str(), srv.response.next_index);
          return true;
        } else {
          ROS_ERROR("Switch to previous controller failed: %s", srv.response.message.c_str());
          return false;
        }
      } else {
        ROS_ERROR("Switch to previous controller service call failed");
        return false;
      }
    }

    bool changeAutoGaitStatus(bool flag)
    {
      std_srvs::SetBool srv;

      srv.request.data = flag;  // true开启，false关闭

      if (auto_gait_change_client_.call(srv))
      {
          return true;
      }
      else
      {
          ROS_ERROR("触发失败");
          return false;
      }

      return true;

    }

  private:
    ros::NodeHandle nodeHandle_;
    TargetTrajectoriesRosPublisher targetPoseCommand_;
    ros::Subscriber joy_sub_;
    ros::Subscriber feet_sub_;
    ros::Subscriber observation_sub_;
    ros::Subscriber gait_scheduler_sub_;
    ros::Subscriber policy_sub_;
    ros::Subscriber gait_change_sub_;
    ros::Subscriber is_rl_controller_sub_;
    ros::Subscriber arm_ctrl_mode_sub_;
    int arm_ctrl_mode_;
    bool is_rl_controller_{false};  // 当前是否为RL控制器
    ocs2::scalar_array_t mpc_default_velocity_limits_{0.4, 0.2, 0.3, 0.4};  // 保存MPC默认速度限制
    bool get_observation_ = false;
    vector_t current_target_ = vector_t::Zero(6);
    std::string current_desired_gait_ = "stance";
    gaitTimeName_t current_gait_rec_{"stance", 0.0}, last_gait_rec_{"stance", 0.0};
    bool auto_stance_mode_ = true;
    bool reached_target_ = false;
    scalar_t target_displacement_velocity_;
    scalar_t target_rotation_velocity_;
    scalar_t com_height_;
    vector_t default_joint_state_ = vector_t::Zero(12);
    vector_t commad_line_target_ = vector_t::Zero(6);
    vector_t joystick_origin_axis_ = vector_t::Zero(6);
    sensor_msgs::Joy old_joy_msg_;
    double joystickSensitivity = 100;
    bool joy_execute_action_ = true;
    LowPassFilter5thOrder joystickFilter_;

    ocs2::scalar_array_t c_relative_base_limit_{0.4, 0.2, 0.3, 0.4};
    double squatHeightMin_ = 0.0, squatHeightMax_ = 0.0;
    ocs2::SystemObservation observation_;
    ros::Publisher mode_sequence_template_publisher_;
    ros::Publisher mode_scale_publisher_;
    ros::Publisher cmd_vel_publisher_;
    ros::Publisher gait_name_publisher_;
    ros::Publisher stop_pub_;
    ros::Publisher re_start_pub_;
    ros::Publisher head_motion_pub_;
    ros::Publisher hand_position_pub_;
    ros::Publisher waist_motion_pub_;
    ros::Publisher enable_waist_control_pub_;
    ros::Publisher slope_planning_pub_;
    float total_mode_scale_{1.0};
    bool button_start_released_{true};
    // TargetTrajectories current_target_traj_;
    std::mutex target_mutex_;
    vector_t feet_pos_measured_ = vector_t::Zero(24);

    std::string cur_gait_name_{"stance"};

    std::map<std::string, humanoid::ModeSequenceTemplate> gait_map_;
    ros::ServiceServer joy_topic_service_;
    std::string current_joy_topic_;
    ros::ServiceClient switch_controller_client_;
    ros::ServiceClient get_controller_list_client_;
    ros::ServiceClient switch_to_next_controller_client_;
    ros::ServiceClient switch_to_previous_controller_client_;
    ros::ServiceClient auto_gait_change_client_;
    
    // 楼梯检测相关
    bool stair_detection_enabled_ = false;
    
    // 遥感/方向键轴输入开关（默认允许）
    bool axes_input_enabled_{true};
    // 手抓开合状态（默认张开 -> false）
    bool hand_closed_{false};
    
    // 命令执行相关
    std::map<std::string, Command_t> commands_map_;
    std::string repo_root_path_;
    std::future<bool> command_future_;
    
    // Arm execute action service
    ros::ServiceClient execute_arm_action_client_;
    // Launch status
    ros::ServiceClient real_launch_status_client_;
    // Fall stand up trigger service
    ros::ServiceClient trigger_fall_stand_up_client_;
    // Fall down state service (SetBool)
    ros::ServiceClient set_fall_down_state_client_;
    // Dance controller trigger service
    ros::ServiceClient trigger_dance_client_;
    bool robot_launched_{false};
    ros::Time last_status_check_time_;
    bool real_{false};
    double controller_switch_time{2.5};
    const double roban_controller_switch_time{1.0};
    ros::Time last_controller_switch_time_{0};
    bool controller_switching_{false};
    
    // Robot type: 1 = wheel mode, 2 = humanoid mode
    int robot_type_;
    RobotVersion rb_version_{3, 4};
    
    // 动作执行状态相关
    ros::Subscriber robot_action_state_sub_;
    bool robot_action_executing_{false};  // 标记是否有动作正在执行
    
    // 腰部控制状态跟踪
    bool waist_control_active_{false};  // 标记是否正在控制腰部（模式2）
    int waist_dof_{0};  // 腰部自由度，只有>0时才进行腰部控制
    double waist_yaw_max_angle_deg_{0.0};  // 腰部最大旋转角度（度），从配置文件加载
    
    // 头部控制状态跟踪
    double current_head_yaw_{0.0};      // 当前头部yaw角度
    double current_head_pitch_{0.0};    // 当前头部pitch角度
    ros::Time last_head_yaw_control_time_{0};   // 上次头部yaw控制时间
    ros::Time last_head_pitch_control_time_{0}; // 上次头部pitch控制时间
    bool head_control_active_{false};   // 标记是否正在控制头部
    const double HEAD_MAX_VELOCITY_DEG_PER_SEC_{150.0};  // 头部最大角速度（度/秒）
    const double HEAD_DEAD_ZONE_{0.15};  // 头部控制死区（摇杆值）

  };
}

int main(int argc, char *argv[])
{
  const std::string robotName = "humanoid";

  // Initialize ros node
  ::ros::init(argc, argv, robotName + "_joy_command_node");
  ::ros::NodeHandle nodeHandle;

  ocs2::JoyControl joyControl(nodeHandle, robotName);
  joyControl.run();

  return 0;
}
