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
#include <std_msgs/Float64MultiArray.h>
#include <geometry_msgs/Twist.h>
#include "kuavo_msgs/SetJoyTopic.h"

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
#include <std_srvs/SetBool.h>

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
      {"BUTTON_JUMP", 2},
      {"BUTTON_WALK", 3},
      {"BUTTON_LB", 4},
      {"BUTTON_RB", 5},
      {"BUTTON_BACK", 6},
      {"BUTTON_START", 7}
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
      {"BUTTON_JUMP", 3},
      {"BUTTON_WALK", 4},
      {"BUTTON_LB", 6},
      {"BUTTON_RB", 7},
      {"BUTTON_BACK", 10},
      {"BUTTON_START", 11}
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

  class JoyControl
  {
  public:
    JoyControl(ros::NodeHandle &nodeHandle, const std::string &robotName, bool verbose = false)
        : nodeHandle_(nodeHandle),
          targetPoseCommand_(nodeHandle, robotName)
    {
      // 获取repo_root_path
      if (!nodeHandle.getParam("repo_root_path", repo_root_path_))
      {
        ROS_WARN_STREAM("No repo_root_path parameter found, using current directory.");
        repo_root_path_ = ".";
      }
      ROS_INFO_STREAM("Repository root path: " << repo_root_path_);
      

      if (nodeHandle.hasParam("joy_node/dev"))
      {
        std::string joystick_device;
        nodeHandle.getParam("joy_node/dev", joystick_device);

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

      if (nodeHandle.hasParam("channel_map_path"))
      {
        std::string channel_map_path;
        nodeHandle.getParam("channel_map_path", channel_map_path);
        ROS_INFO_STREAM("Loading joystick mapping from " << channel_map_path);
        loadJoyJsonConfig(channel_map_path, joyButtonMap, joyAxisMap);
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
      Eigen::Vector4d joystickFilterCutoffFreq_(joystickSensitivity, joystickSensitivity, 
                                                  joystickSensitivity, joystickSensitivity);
      joystickFilter_.setParams(0.01,joystickFilterCutoffFreq_);
      old_joy_msg_.axes = std::vector<float>(8, 0.0);     // 假设有 8 个轴，默认值为 0.0
      old_joy_msg_.buttons = std::vector<int32_t>(12, 0);
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
      default_joint_state_ = drake_interface_->getDefaultJointState();
      com_height_ = drake_interface_->getIntialHeight();
      loadData::loadCppDataType(referenceFile, "targetRotationVelocity", target_rotation_velocity_);
      loadData::loadCppDataType(referenceFile, "targetDisplacementVelocity", target_displacement_velocity_);
      loadData::loadCppDataType(referenceFile, "cmdvelLinearXLimit", c_relative_base_limit_[0]);
      loadData::loadCppDataType(referenceFile, "cmdvelAngularYAWLimit", c_relative_base_limit_[3]);


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
      cmd_vel_publisher_ = nodeHandle.advertise<geometry_msgs::Twist>("/cmd_vel", 10, true);
      current_joy_topic_ = "/joy";  // 默认话题
      joy_topic_service_ = nodeHandle_.advertiseService("/set_joy_topic", &JoyControl::setJoyTopicCallback, this);
      joy_sub_ = nodeHandle_.subscribe(current_joy_topic_, 10, &JoyControl::joyCallback, this);
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

      stop_pub_ = nodeHandle_.advertise<std_msgs::Bool>("/stop_robot", 10);
      re_start_pub_ = nodeHandle_.advertise<std_msgs::Bool>("/re_start_robot", 10);
      head_motion_pub_ = nodeHandle_.advertise<kuavo_msgs::robotHeadMotionData>("/robot_head_motion_data", 10);
      waist_motion_pub_ = nodeHandle_.advertise<std_msgs::Float64MultiArray>("/robot_waist_motion_data", 10);
      slope_planning_pub_ = nodeHandle_.advertise<std_msgs::Bool>("/humanoid/mpc/enable_slope_planning", 10);

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
        
        std::cout << "Loaded " << commands_map_.size() << " commands" <<std::endl;
        for (const auto& cmd : commands_map_)
        {
          std::cout << " - " << cmd.first << ": "<< cmd.second.type << " " << cmd.second.value << " " << cmd.second.description << std::endl;
        }
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
        if (!get_observation_)
        {
          // ROS_INFO_STREAM("Waiting for observation message...");
          continue;
        }
        checkAndPublishCommandLine(joystick_origin_axis_);
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
                old_joy_msg_backup.buttons[button.second] = old_joy_msg_.buttons[it->second];
            }
        }
        for (const auto& axis : joyAxisMap_backup) {
            auto it = std::find_if(joyAxisMap.begin(), joyAxisMap.end(),
                [&axis](const auto& pair) { return pair.first == axis.first; });
            if (it != joyAxisMap.end()) {
                old_joy_msg_backup.axes[axis.second] = old_joy_msg_.axes[it->second];
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
    void joyCallback(const sensor_msgs::Joy::ConstPtr &joy_msg)
    {
      vector_t joystickOriginAxisFilter_ = vector_t::Zero(6);
      vector_t joystickOriginAxisTemp_ = vector_t::Zero(6);
      double alpha_ = joystickSensitivity / 1000;

      if (std::any_of(joy_msg->buttons.begin(), joy_msg->buttons.end(), [](float button) {
              return std::abs(button) > 1;
          }))
      {
        std::cout << "invalide joy msg"<<std::endl;
        return;
      }

      if (old_joy_msg_.buttons.size() == JOYSTICK_BEITONG_BUTTON_NUM && joy_msg->buttons.size() == JOYSTICK_XBOX_BUTTON_NUM)
      {
        nodeHandle_.setParam("joystick_type", JOYSTICK_XBOX_MAP_JSON);
        std::string channel_map_path = ros::package::getPath("humanoid_controllers") + "/launch/joy/" + JOYSTICK_XBOX_MAP_JSON + ".json";
        nodeHandle_.setParam("channel_map_path", channel_map_path);
        ROS_WARN("[JoyController]: Joystick data mapping has changed from BEITONG to X-Box");
        reloadJoystickMapping(JOYSTICK_AXIS_NUM, JOYSTICK_XBOX_BUTTON_NUM);
      }
      if(old_joy_msg_.buttons.size() == JOYSTICK_XBOX_BUTTON_NUM && joy_msg->buttons.size() == JOYSTICK_BEITONG_BUTTON_NUM)
      {
        nodeHandle_.setParam("joystick_type", JOYSTICK_BEITONG_MAP_JSON);
        std::string channel_map_path = ros::package::getPath("humanoid_controllers") + "/launch/joy/" + JOYSTICK_BEITONG_MAP_JSON + ".json";
        nodeHandle_.setParam("channel_map_path", channel_map_path);
        ROS_WARN("[JoyController]: Joystick data mapping has changed from X-Box to BEITONG");
        reloadJoystickMapping(JOYSTICK_AXIS_NUM, JOYSTICK_BEITONG_BUTTON_NUM);
      }

      if(joy_msg->axes[joyAxisMap["AXIS_RIGHT_RT"]] < -0.5)
      {
        // 组合键
        double head_yaw = joy_msg->axes[joyAxisMap["AXIS_RIGHT_STICK_YAW"]];
        double head_pitch = joy_msg->axes[joyAxisMap["AXIS_RIGHT_STICK_Z"]];
        head_yaw = 80.0 * head_yaw;     // +- 60deg
        head_pitch = -25.0 * head_pitch;// +- 25deg 
        // std::cout << "head_yaw: " << head_yaw << " head_pitch: " << head_pitch << std::endl;
        controlHead(head_yaw, head_pitch);
        // return;
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
        if(joy_msg->buttons[joyButtonMap["BUTTON_JUMP"]])
        {
          ROS_INFO("Reset grab box demo");
          resetGrabBoxDemo(true);
        }

      }
      else
      {
        joystickOriginAxisTemp_.head(4) << joy_msg->axes[joyAxisMap["AXIS_LEFT_STICK_X"]], joy_msg->axes[joyAxisMap["AXIS_LEFT_STICK_Y"]], joy_msg->axes[joyAxisMap["AXIS_RIGHT_STICK_Z"]], joy_msg->axes[joyAxisMap["AXIS_RIGHT_STICK_YAW"]];
      }

      if(joy_msg->axes[joyAxisMap["AXIS_LEFT_LT"]] < -0.5)
      {
        if (!old_joy_msg_.buttons[joyButtonMap["BUTTON_STANCE"]] && joy_msg->buttons[joyButtonMap["BUTTON_STANCE"]])
        {
          pubSlopePlanning(false);
        }
        else if (!old_joy_msg_.buttons[joyButtonMap["BUTTON_WALK"]] && joy_msg->buttons[joyButtonMap["BUTTON_WALK"]])
        {
          pubSlopePlanning(true);
        }
        else if (!old_joy_msg_.buttons[joyButtonMap["BUTTON_JUMP"]] && joy_msg->buttons[joyButtonMap["BUTTON_JUMP"]])
        {
          if (stair_detection_enabled_)
          {
            executeCommand("stop_stair_detect");
          }
          else{
            executeCommand("start_stair_detect");
          }
        }
        else if (!old_joy_msg_.buttons[joyButtonMap["BUTTON_TROT"]] && joy_msg->buttons[joyButtonMap["BUTTON_TROT"]])
        {
          executeCommand("stairclimb");
        }
        else
        {
           // 组合键控制腰部
          double waist_yaw = joy_msg->axes[joyAxisMap["AXIS_RIGHT_STICK_YAW"]];
          waist_yaw = 120.0 * waist_yaw;   // +- 120deg
          // std::cout << "waist_yaw: " << waist_yaw << std::endl;
          controlWaist(waist_yaw);

        }
        old_joy_msg_ = *joy_msg;
        return;
      }


      // 非辅助模式下才可控行走
      if(joy_msg->axes[joyAxisMap["AXIS_RIGHT_RT"]] > -0.5 && joy_msg->axes[joyAxisMap["AXIS_LEFT_LT"]] > -0.5)
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

      vector_t button_trigger_axis = vector_t::Zero(6); 
      if (!old_joy_msg_.buttons[joyButtonMap["BUTTON_START"]] && joy_msg->buttons[joyButtonMap["BUTTON_START"]])
      {
        callRealInitializeSrv();
      }

      if (joy_msg->buttons[joyButtonMap["BUTTON_LB"]])// 按下左侧侧键，切换模式
      {
        if (!old_joy_msg_.buttons[joyButtonMap["BUTTON_STANCE"]] && joy_msg->buttons[joyButtonMap["BUTTON_STANCE"]])
        pubModeGaitScale(0.9);
        else if (!old_joy_msg_.buttons[joyButtonMap["BUTTON_WALK"]] && joy_msg->buttons[joyButtonMap["BUTTON_WALK"]])
        pubModeGaitScale(1.1);
        else if (!old_joy_msg_.buttons[joyButtonMap["BUTTON_TROT"]] && joy_msg->buttons[joyButtonMap["BUTTON_TROT"]])
        {
          current_arm_mode_ = (current_arm_mode_ > 0)? 0 : 1;
          callArmControlService(current_arm_mode_);
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
      }
      else
        checkGaitSwitchCommand(joy_msg);


      if (joy_msg->buttons[joyButtonMap["BUTTON_BACK"]])
        callTerminateSrv();
      else if (joy_msg->axes[joyAxisMap["AXIS_FORWARD_BACK_TRIGGER"]])
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
      old_joy_msg_ = *joy_msg;
    }

    void checkGaitSwitchCommand(const sensor_msgs::Joy::ConstPtr &joy_msg)
    {
      // 检查是否有gait切换指令
      if (!old_joy_msg_.buttons[joyButtonMap["BUTTON_STANCE"]] && joy_msg->buttons[joyButtonMap["BUTTON_STANCE"]])
      {
        publishGaitTemplate("stance");
      }
      else if (!old_joy_msg_.buttons[joyButtonMap["BUTTON_TROT"]] && joy_msg->buttons[joyButtonMap["BUTTON_TROT"]])
      {
        publishGaitTemplate("trot");
      }
      else if (!old_joy_msg_.buttons[joyButtonMap["BUTTON_JUMP"]] && joy_msg->buttons[joyButtonMap["BUTTON_JUMP"]])
      {
        // publishGaitTemplate("jump");
      }
      else if (!old_joy_msg_.buttons[joyButtonMap["BUTTON_WALK"]] && joy_msg->buttons[joyButtonMap["BUTTON_WALK"]])
      {
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
      // 发布对应的gait模板
      humanoid::ModeSequenceTemplate modeSequenceTemplate = gait_map_.at(gaitName);
      mode_sequence_template_publisher_.publish(createModeSequenceTemplateMsg(modeSequenceTemplate));
      current_desired_gait_ = gaitName;
    }

    void gaitChangeCallback(const std_msgs::String::ConstPtr& msg) 
    {
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
      Eigen::VectorXd limit_vector(4);
      limit_vector << c_relative_base_limit_[0], c_relative_base_limit_[1], c_relative_base_limit_[2], c_relative_base_limit_[3];
      if (joystick_origin_axis.cwiseAbs().maxCoeff() < DEAD_ZONE)
        return updated; // command line is zero, do nothing

      commad_line_target_.head(4) = joystick_origin_axis.head(4).cwiseProduct(limit_vector);

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
        std::cout << "base height: " << current_target_(2) << std::endl;
      }
      else
      {
        // current_target_(2) = com_height_;
        cmdVel.linear.z = 0.0;
      }

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
        ROS_INFO("[JoyControl] Service call successful");
      }
      else
      {
        ROS_ERROR("Failed to callRealInitializeSrv service, use publish topic.");
        std_msgs::Bool msg;
        msg.data = true;
        re_start_pub_.publish(msg);
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
      std_msgs::Float64MultiArray msg;
      msg.data.resize(1);
      msg.data[0] = -waist_yaw;
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

    

  private:
    ros::NodeHandle nodeHandle_;
    TargetTrajectoriesRosPublisher targetPoseCommand_;
    ros::Subscriber joy_sub_;
    ros::Subscriber feet_sub_;
    ros::Subscriber observation_sub_;
    ros::Subscriber gait_scheduler_sub_;
    ros::Subscriber policy_sub_;
    ros::Subscriber gait_change_sub_;
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
    int current_arm_mode_{1};
    double joystickSensitivity = 100;
    LowPassFilter5thOrder joystickFilter_;

    ocs2::scalar_array_t c_relative_base_limit_{0.4, 0.2, 0.3, 0.4};
    ocs2::SystemObservation observation_;
    ros::Publisher mode_sequence_template_publisher_;
    ros::Publisher mode_scale_publisher_;
    ros::Publisher cmd_vel_publisher_;
    ros::Publisher stop_pub_;
    ros::Publisher re_start_pub_;
    ros::Publisher head_motion_pub_;
    ros::Publisher waist_motion_pub_;
    ros::Publisher slope_planning_pub_;
    float total_mode_scale_{1.0};
    bool button_start_released_{true};
    // TargetTrajectories current_target_traj_;
    std::mutex target_mutex_;
    vector_t feet_pos_measured_ = vector_t::Zero(24);

    std::map<std::string, humanoid::ModeSequenceTemplate> gait_map_;
    ros::ServiceServer joy_topic_service_;
    std::string current_joy_topic_;
    
    // 楼梯检测相关
    bool stair_detection_enabled_ = false;
    
    // 命令执行相关
    std::map<std::string, Command_t> commands_map_;
    std::string repo_root_path_;
    std::future<bool> command_future_;
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
