#include <string>
#include <fstream>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/joystick.h>
#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Bool.h>
#include <kuavo_common/common/json.hpp>
#include <kuavo_msgs/changeTorsoCtrlMode.h>
#include <map>
#include <algorithm>
#include <cmath>
#include <iostream>

#define DEAD_ZONE 0.05
#define MAX_JOYSTICK_NAME_LEN 256
#define JOYSTICK_XBOX_MAP_JSON "bt2"
#define JOYSTICK_BEITONG_MAP_JSON "bt2pro"

namespace mobile_manipulator
{
  // 控制模式枚举
  enum class ControlMode
  {
    CMD_VEL = 0,        // cmd_vel 模式（默认）
    CMD_VEL_WORLD = 1,  // cmd_vel_world 模式
    TORSO_CONTROL = 2   // 躯干控制模式
  };

  // 手柄按钮映射表
  std::map<std::string, int> joyButtonMap = {
      {"BUTTON_STANCE", 0},
      {"BUTTON_TROT", 1},
      {"BUTTON_RL", 2},
      {"BUTTON_WALK", 3},
      {"BUTTON_LB", 4},
      {"BUTTON_RB", 5},
      {"BUTTON_BACK", 6},
      {"BUTTON_START", 7},
      {"BUTTON_M1", 9},
      {"BUTTON_M2", 10}
  };

  // 手柄摇杆轴映射表
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

  class MobileManipulatorJoyControl
  {
  public:
    MobileManipulatorJoyControl(ros::NodeHandle &nodeHandle)
        : nodeHandle_(nodeHandle)
    {
      // 初始化速度限制参数（参考 joy_to_cmd_vel_xy.py）
      linear_scale_x_ = 0.8;  // m/s
      linear_scale_y_ = 0.8;  // m/s (用于cmd_vel和cmd_vel_world模式)
      linear_scale_z_ = 0.4;  // m/s (用于躯干控制)
      angular_scale_z_ = 0.5;   // rad/s
      angular_scale_y_ = 0.4; // rad/s (用于躯干控制)
      deadzone_ = 0.05;

      int robotVersion_ = 60;
      if(nodeHandle_.hasParam("robot_version"))
      {
        nodeHandle_.getParam("robot_version", robotVersion_);
      }
      // 躯干初始化位置xyz
      if(robotVersion_ == 60)
      {
        initialTorsoPose_x_ = 0.196123;
        initialTorsoPose_y_ = 0.0005;
        initialTorsoPose_z_ = 0.789919;
      }
      else if(robotVersion_ == 61 || robotVersion_ == 62 || robotVersion_ == 63)
      {
        initialTorsoPose_x_ = 0.11575;
        initialTorsoPose_y_ = 0.0;
        initialTorsoPose_z_ = 0.923803;
      }

      // 躯干笛卡尔运动限幅
      torsoMax_x_ = 0.13; torsoMin_x_ = 0.2;
      torsoMax_z_ = 0.32; torsoMin_z_ =  0.0;
      torsoMax_yaw_ = 0.5235; torsoMin_yaw_ = 0.5235;
      torsoMax_pitch_ = 0.5235; torsoMin_pitch_ = 0.314;

      // 检测手柄类型并设置映射
      detectJoystickType();

      // 加载手柄映射配置
      if (nodeHandle.hasParam("channel_map_path"))
      {
        std::string channel_map_path;
        nodeHandle.getParam("channel_map_path", channel_map_path);
        ROS_INFO_STREAM("加载手柄映射配置: " << channel_map_path);
        loadJoyJsonConfig(channel_map_path);
      }
      else
      {
        ROS_WARN_STREAM("未找到 channel_map_path 参数，使用默认手柄映射");
      }

      // 从 joyAxisMap 获取轴索引（支持JSON配置）
      // 根据 joy_to_cmd_vel_xy.py: axes[1]->linear.x, axes[0]->linear.y, axes[2]->angular.z
      linear_axis_index_x_ = joyAxisMap["AXIS_LEFT_STICK_X"];   // axes[1] -> 前进/后退
      linear_axis_index_y_ = joyAxisMap["AXIS_LEFT_STICK_Y"];   // axes[0] -> 左右移动
      angular_axis_index_ = joyAxisMap["AXIS_RIGHT_STICK_YAW"];  // 旋转
      linear_z_axis_index_ = joyAxisMap["AXIS_RIGHT_STICK_Z"];  // 用于躯干控制 linear.z
      angular_y_axis_index_ = joyAxisMap["AXIS_LEFT_STICK_X"]; // 用于躯干控制 angular.y (RB+此轴)
      rt_axis_index_ = joyAxisMap["AXIS_RIGHT_RT"];  // RT触发器轴索引（用于复位）

      // 创建发布者和订阅者
      cmd_vel_publisher_ = nodeHandle_.advertise<geometry_msgs::Twist>("/cmd_vel", 10, true);
      cmd_vel_world_publisher_ = nodeHandle_.advertise<geometry_msgs::Twist>("/cmd_vel_world", 10, true);
      cmd_lb_torso_publisher_ = nodeHandle_.advertise<geometry_msgs::Twist>("/cmd_lb_torso_pose", 10, true);
      stop_pub_ = nodeHandle_.advertise<std_msgs::Bool>("/stop_robot", 10);
      joy_sub_ = nodeHandle_.subscribe("/joy", 10, &MobileManipulatorJoyControl::joyCallback, this);
      
      // 初始化MPC模式切换服务客户端
      mpc_control_client_ = nodeHandle_.serviceClient<kuavo_msgs::changeTorsoCtrlMode>("/mobile_manipulator_mpc_control");
      previous_mpc_mode_ = 0;  // 默认模式: No control

      // 初始化控制模式（默认为 cmd_vel 模式）
      current_mode_ = ControlMode::CMD_VEL;
      previous_mode_ = ControlMode::CMD_VEL;

      // 初始化旧的手柄消息（用于检测按钮按下事件）
      old_joy_msg_.axes = std::vector<float>(8, 0.0);
      old_joy_msg_.buttons = std::vector<int32_t>(12, 0);
      
      // 初始化上一次的躯干控制摇杆输入值
      last_linear_x_input_ = 0.0;
      last_linear_z_input_ = 0.0;
      last_angular_y_input_ = 0.0;
      last_angular_z_input_ = 0.0;
      
      // 初始化积分状态变量（从初始位置开始）
      integrated_linear_x_ = 0.0;
      integrated_linear_z_ = 0.0;
      integrated_angular_y_ = 0.0;
      integrated_angular_z_ = 0.0;
      
      // 初始化积分增益参数（可根据需要调整）
      integral_gain_linear_x_ = 0.01;   // 每次回调的积分增益
      integral_gain_linear_z_ = 0.01;
      integral_gain_angular_y_ = 0.01;
      integral_gain_angular_z_ = 0.01;
      
      // 初始化控制量为零的时间跟踪
      zero_control_start_time_ = ros::Time::now();
      is_control_zero_ = true;  // 初始状态认为控制量为零

      ROS_INFO("轮臂手柄控制节点已启动");
      ROS_INFO("操作提示: 使用左摇杆控制底盘移动，右摇杆控制旋转");
      ROS_INFO("模式切换: LB+A -> cmd_vel, LB+Y -> cmd_vel_world, LB+B -> 躯干控制");
      ROS_INFO("当前映射: linear_x=%d, linear_y=%d, angular=%d, deadzone=%.2f",
               linear_axis_index_x_, linear_axis_index_y_, angular_axis_index_, deadzone_);
      ROS_INFO("当前模式: cmd_vel (默认)");
    }

    void run()
    {
      ros::spin();
    }

  private:

    ros::NodeHandle nodeHandle_;
    ros::Publisher cmd_vel_publisher_;
    ros::Publisher cmd_vel_world_publisher_;
    ros::Publisher cmd_lb_torso_publisher_;
    ros::Publisher stop_pub_;
    ros::Subscriber joy_sub_;
    ros::ServiceClient mpc_control_client_;

    // 控制模式
    ControlMode current_mode_;
    ControlMode previous_mode_;

    // 速度缩放参数
    double linear_scale_x_;
    double linear_scale_y_;
    double linear_scale_z_;      // 用于躯干控制
    double angular_scale_z_;
    double angular_scale_y_;      // 用于躯干控制
    double deadzone_;

    // 躯干初始位置
    double initialTorsoPose_x_;
    double initialTorsoPose_y_;
    double initialTorsoPose_z_;

    // 躯干各轴限幅大小
    double torsoMax_x_, torsoMin_x_;
    double torsoMax_z_, torsoMin_z_;
    double torsoMax_yaw_, torsoMin_yaw_;
    double torsoMax_pitch_, torsoMin_pitch_;

    // 摇杆轴索引
    int linear_axis_index_x_;
    int linear_axis_index_y_;
    int angular_axis_index_;
    int linear_z_axis_index_;     // 用于躯干控制
    int angular_y_axis_index_;     // 用于躯干控制
    int rt_axis_index_;           // RT触发器轴索引

    // MPC模式记录
    int previous_mpc_mode_;

    // 保存上一次的手柄消息（用于检测按钮按下事件）
    sensor_msgs::Joy old_joy_msg_;
    
      // 保存上一次的躯干控制摇杆输入值（用于LB松开时保持值）
      double last_linear_x_input_;
      double last_linear_z_input_;
      double last_angular_y_input_;  // 用于RT松开时保持angular.y值
      double last_angular_z_input_;
      
      // 积分状态变量（用于躯干控制的积分控制）
      double integrated_linear_x_;
      double integrated_linear_z_;
      double integrated_angular_y_;
      double integrated_angular_z_;
      
      // 积分增益参数
      double integral_gain_linear_x_;
      double integral_gain_linear_z_;
      double integral_gain_angular_y_;
      double integral_gain_angular_z_;
      
      // 控制量为零的时间跟踪（用于模式切换检查）
      ros::Time zero_control_start_time_;
      bool is_control_zero_;
      const double MIN_ZERO_DURATION_ = 2.0;  // 控制量为零的最小持续时间（秒）
      
    // 检测手柄类型（BEITONG/XBOX）并自动加载对应配置
    void detectJoystickType()
    {
      if (nodeHandle_.hasParam("joy_node/dev"))
      {
        std::string joystick_device;
        nodeHandle_.getParam("joy_node/dev", joystick_device);

        int fd = open(joystick_device.c_str(), O_RDONLY);
        if (fd < 0)
        {
          ROS_ERROR("无法打开手柄设备: %s", joystick_device.c_str());
          return;
        }

        char name[MAX_JOYSTICK_NAME_LEN] = {0};
        if (ioctl(fd, JSIOCGNAME(MAX_JOYSTICK_NAME_LEN), name) >= 0)
        {
          std::string joystick_name(name);
          
          if (joystick_name.find("BEITONG") != std::string::npos || joystick_name.find("BFM") != std::string::npos)
          {
            nodeHandle_.setParam("joystick_type", JOYSTICK_BEITONG_MAP_JSON);
            std::string channel_map_path = ros::package::getPath("humanoid_controllers") + "/launch/joy/" + JOYSTICK_BEITONG_MAP_JSON + ".json";
            nodeHandle_.setParam("channel_map_path", channel_map_path);
            ROS_INFO("检测到手柄类型: BEITONG，自动加载配置: %s", channel_map_path.c_str());
            loadJoyJsonConfig(channel_map_path);
          }
          else if (joystick_name.find("X-Box") != std::string::npos || joystick_name.find("XBOX") != std::string::npos)
          {
            nodeHandle_.setParam("joystick_type", JOYSTICK_XBOX_MAP_JSON);
            std::string channel_map_path = ros::package::getPath("humanoid_controllers") + "/launch/joy/" + JOYSTICK_XBOX_MAP_JSON + ".json";
            nodeHandle_.setParam("channel_map_path", channel_map_path);
            ROS_INFO("检测到手柄类型: XBOX，自动加载配置: %s", channel_map_path.c_str());
            loadJoyJsonConfig(channel_map_path);
          }
          else
          {
            ROS_INFO("未识别的手柄类型: %s，使用默认参数", joystick_name.c_str());
          }
        }
        else
        {
          ROS_ERROR("无法获取手柄名称");
        }

        close(fd);
      }
    }

    // 加载手柄映射JSON配置文件（同时加载按钮和轴映射）
    void loadJoyJsonConfig(const std::string &config_file)
    {
      try
      {
        nlohmann::json data_;
        std::ifstream ifs(config_file);
        if (!ifs.is_open())
        {
          ROS_WARN("无法打开配置文件: %s", config_file.c_str());
          return;
        }

        ifs >> data_;

        // 加载按钮映射
        if (data_.contains("JoyButton"))
        {
          for (auto &item : data_["JoyButton"].items())
          {
            if (joyButtonMap.find(item.key()) != joyButtonMap.end())
            {
              joyButtonMap[item.key()] = item.value();
            }
            else
            {
              joyButtonMap.insert({item.key(), item.value()});
            }
          }
        }

        // 加载摇杆轴映射
        if (data_.contains("JoyAxis"))
        {
          for (auto &item : data_["JoyAxis"].items())
          {
            if (joyAxisMap.find(item.key()) != joyAxisMap.end())
            {
              joyAxisMap[item.key()] = item.value();
            }
            else
            {
              joyAxisMap.insert({item.key(), item.value()});
            }
          }
        }

        ROS_INFO("成功加载手柄映射配置（按钮和轴）");
      }
      catch (const std::exception &e)
      {
        ROS_WARN("加载手柄映射配置失败: %s", e.what());
      }
    }

    // 限制数值在指定范围内
    double clamp(double value, double min_value, double max_value)
    {
      return std::max(min_value, std::min(value, max_value));
    }

    // 调用终止服务（发布停止信号）
    void callTerminateSrv()
    {
      std::cout << "触发 callTerminateSrv" << std::endl;
      for (int i = 0; i < 5; i++)
      {
        std_msgs::Bool msg;
        msg.data = true;
        stop_pub_.publish(msg);
        ros::Duration(0.1).sleep();
      }
    }

    // 查询当前MPC模式
    int getCurrentMpcMode()
    {
      kuavo_msgs::changeTorsoCtrlMode srv;
      srv.request.control_mode = -1;  // 无效模式，用于查询
      if (mpc_control_client_.call(srv))
      {
        return srv.response.mode;
      }
      return 0;
    }

    // 调用MPC模式切换服务
    bool callChangeMpcMode(int mode)
    {
      kuavo_msgs::changeTorsoCtrlMode srv;
      srv.request.control_mode = mode;
      if (mpc_control_client_.call(srv) && srv.response.result)
      {
        return true;
      }
      return false;
    }

    // 检查是否可以切换模式（控制量为零且持续时间超过2秒）
    bool canSwitchMode(const sensor_msgs::Joy::ConstPtr &joy_msg)
    {
      ros::Time now = ros::Time::now();
      bool is_zero = false;
      
      // 检查控制量是否为零
      if (current_mode_ == ControlMode::TORSO_CONTROL)
      {
        // 躯干模式：检查积分值
        is_zero = (std::abs(integrated_linear_x_) < 1e-6 && std::abs(integrated_linear_z_) < 1e-6 &&
                   std::abs(integrated_angular_y_) < 1e-6 && std::abs(integrated_angular_z_) < 1e-6);
      }
      else
      {
        // cmd_vel模式：检查摇杆输入（应用死区）
        double x = std::abs(joy_msg->axes[linear_axis_index_x_]) < deadzone_ ? 0.0 : joy_msg->axes[linear_axis_index_x_];
        double y = std::abs(joy_msg->axes[linear_axis_index_y_]) < deadzone_ ? 0.0 : joy_msg->axes[linear_axis_index_y_];
        double z = std::abs(joy_msg->axes[angular_axis_index_]) < deadzone_ ? 0.0 : joy_msg->axes[angular_axis_index_];
        is_zero = (std::abs(x) < 1e-6 && std::abs(y) < 1e-6 && std::abs(z) < 1e-6);
      }
      
      // 更新状态和时间
      if (is_zero)
      {
        if (!is_control_zero_)
        {
          zero_control_start_time_ = now;
          is_control_zero_ = true;
        }
        return (now - zero_control_start_time_).toSec() >= MIN_ZERO_DURATION_;
      }
      else
      {
        is_control_zero_ = false;
        return false;
      }
    }

    // 切换控制模式
    void switchControlMode(ControlMode new_mode)
    {
      if (new_mode != current_mode_)
      {
        // 切换到躯干控制模式时，记录当前MPC模式并切换到ArmOnly
        if (new_mode == ControlMode::TORSO_CONTROL)
        {
          previous_mpc_mode_ = getCurrentMpcMode();
          if (!callChangeMpcMode(1))  // 切换到ArmOnly模式
          {
            ROS_ERROR("切换到ArmOnly模式失败");
          }
          // 重置积分值和上一次输入值到初始位置
          integrated_linear_x_ = 0.0;
          integrated_linear_z_ = 0.0;
          integrated_angular_y_ = 0.0;
          integrated_angular_z_ = 0.0;
          last_linear_x_input_ = 0.0;
          last_linear_z_input_ = 0.0;
          last_angular_y_input_ = 0.0;
          last_angular_z_input_ = 0.0;
        }
        // 从躯干控制模式切换到其他模式时，恢复之前的MPC模式
        else if (current_mode_ == ControlMode::TORSO_CONTROL)
        {
          if (!callChangeMpcMode(previous_mpc_mode_))
          {
            ROS_ERROR("恢复MPC模式失败，模式: %d", previous_mpc_mode_);
          }
        }

        previous_mode_ = current_mode_;
        current_mode_ = new_mode;
        
        // 输出模式切换信息
        std::string prev_mode_str, curr_mode_str;
        if (previous_mode_ == ControlMode::CMD_VEL)
          prev_mode_str = "cmd_vel";
        else if (previous_mode_ == ControlMode::CMD_VEL_WORLD)
          prev_mode_str = "cmd_vel_world";
        else if (previous_mode_ == ControlMode::TORSO_CONTROL)
          prev_mode_str = "torso_control";
        
        if (current_mode_ == ControlMode::CMD_VEL)
          curr_mode_str = "cmd_vel";
        else if (current_mode_ == ControlMode::CMD_VEL_WORLD)
          curr_mode_str = "cmd_vel_world";
        else if (current_mode_ == ControlMode::TORSO_CONTROL)
          curr_mode_str = "torso_control";
        
        std::cout << "========== 控制模式切换 ==========" << std::endl;
        std::cout << "上一个模式: " << prev_mode_str << std::endl;
        std::cout << "当前模式: " << curr_mode_str << std::endl;
        std::cout << "==================================" << std::endl;
      }
    }

    // 手柄回调函数
    void joyCallback(const sensor_msgs::Joy::ConstPtr &joy_msg)
    {
      // 检查消息有效性
      if (std::any_of(joy_msg->buttons.begin(), joy_msg->buttons.end(), [](float button) {
            return std::abs(button) > 1;
          }))
      {
        ROS_WARN_THROTTLE(1.0, "接收到无效的手柄消息");
        return;
      }

      // 检测模式切换组合键
      bool lb_pressed = (joy_msg->buttons[joyButtonMap["BUTTON_LB"]] == 1);
      
      // 尝试切换模式的辅助函数
      auto trySwitchMode = [this, &joy_msg](ControlMode new_mode) {
        if (canSwitchMode(joy_msg))
        {
          switchControlMode(new_mode);
          return true;
        }
        else
        {
          if (current_mode_ == ControlMode::TORSO_CONTROL)
            std::cout << "按RT键复位后，才可切换模式" << std::endl;
          else
            std::cout << "控制量不为零，无法切换模式（需要控制量为零且持续2秒以上）" << std::endl;
          return false;
        }
      };

      // 检测 BUTTON_STANCE 按下事件（切换到 cmd_vel 模式）
      if (lb_pressed && !old_joy_msg_.buttons[joyButtonMap["BUTTON_STANCE"]] && 
          joy_msg->buttons[joyButtonMap["BUTTON_STANCE"]])
      {
        trySwitchMode(ControlMode::CMD_VEL);
      }

      // 检测 BUTTON_WALK 按下事件（切换到 cmd_vel_world 模式）
      if (lb_pressed && !old_joy_msg_.buttons[joyButtonMap["BUTTON_WALK"]] && 
          joy_msg->buttons[joyButtonMap["BUTTON_WALK"]])
      {
        trySwitchMode(ControlMode::CMD_VEL_WORLD);
      }

      // 检测 BUTTON_TROT 按下事件（切换到躯干控制模式）
      if (lb_pressed && !old_joy_msg_.buttons[joyButtonMap["BUTTON_TROT"]] && 
          joy_msg->buttons[joyButtonMap["BUTTON_TROT"]])
      {
        // LB+B 切换到躯干控制模式
        if (current_mode_ != ControlMode::TORSO_CONTROL)
        {
          trySwitchMode(ControlMode::TORSO_CONTROL);
        }
      }

      // 检测 BUTTON_BACK 按下事件（从未按下到按下）
      if (!old_joy_msg_.buttons[joyButtonMap["BUTTON_BACK"]] && 
          joy_msg->buttons[joyButtonMap["BUTTON_BACK"]])
      {
        callTerminateSrv();
      }

      // 根据当前模式处理不同的控制逻辑
      if (current_mode_ == ControlMode::TORSO_CONTROL)
      {
        // ========== 躯干控制模式 ==========

        // 检测RB按钮是否按下（用于angular.z和angular.y控制）
        bool rb_pressed = (joy_msg->buttons[joyButtonMap["BUTTON_RB"]] == 1);

        // 检测RT轴是否按下（用于复位功能，RT轴值通常小于-0.5表示按下）
        bool rt_pressed = (joy_msg->axes[rt_axis_index_] < -0.5);
        
        // 检测RT按下事件（用于复位功能）
        bool old_rt_pressed = (old_joy_msg_.axes[rt_axis_index_] < -0.5);
        bool rt_just_pressed = rt_pressed && !old_rt_pressed;
        
        // RT按下时复位所有积分值和上一次输入值、并直接将躯干恢复为初始状态
        if (rt_just_pressed)
        {
          integrated_linear_x_ = 0.0;
          integrated_linear_z_ = 0.0;
          integrated_angular_y_ = 0.0;
          integrated_angular_z_ = 0.0;
          last_linear_x_input_ = 0.0;
          last_linear_z_input_ = 0.0;
          last_angular_y_input_ = 0.0;
          last_angular_z_input_ = 0.0;
          // 重置控制量为零的时间跟踪，复位后可以立即切换模式（如果满足2秒条件）
          zero_control_start_time_ = ros::Time::now();
          is_control_zero_ = true;
          
          // 复位时立即发布初始位置
          geometry_msgs::Twist reset_cmd;
          reset_cmd.linear.x = initialTorsoPose_x_;
          reset_cmd.linear.y = initialTorsoPose_y_;
          reset_cmd.linear.z = initialTorsoPose_z_;
          reset_cmd.angular.x = 0.0;
          reset_cmd.angular.y = 0.0;
          reset_cmd.angular.z = 0.0;
          cmd_lb_torso_publisher_.publish(reset_cmd);
          
          ROS_INFO("Torso control reset completed");
        }

        // 读取摇杆输入值（LB按下时更新，LB松开时使用0值避免积分持续累加）
        double linear_x_input, linear_z_input;
        if (lb_pressed)
        {
          // LB按下时，读取当前摇杆值
          linear_x_input = joy_msg->axes[linear_axis_index_x_];
          linear_z_input = joy_msg->axes[linear_z_axis_index_];
          
          // 应用死区后保存（避免保存死区内的值）
          if (std::abs(linear_x_input) < deadzone_)
            linear_x_input = 0.0;
          if (std::abs(linear_z_input) < deadzone_)
            linear_z_input = 0.0;
          
          // 更新保存的值（已应用死区）
          last_linear_x_input_ = linear_x_input;
          last_linear_z_input_ = linear_z_input;
        }
        else
        {
          // LB松开时，使用0值避免积分持续累加
          linear_x_input = 0.0;
          linear_z_input = 0.0;
        }
        
        // angular.y 仅在RB按下时读取，RB松开时使用0值避免积分持续累加
        double angular_y_input;
        if (rb_pressed)
        {
          // RB按下时，读取当前摇杆值
          angular_y_input = joy_msg->axes[angular_y_axis_index_];
          
          // 应用死区后保存
          if (std::abs(angular_y_input) < deadzone_)
            angular_y_input = 0.0;
          
          last_angular_y_input_ = angular_y_input;
        }
        else
        {
          // RB松开时，使用0值避免积分持续累加
          angular_y_input = 0.0;
        }
        
        // angular.z 仅在RB按下时读取，RB松开时使用0值避免积分持续累加
        double angular_z_input;
        if (rb_pressed)
        {
          // RB按下时，读取当前摇杆值
          angular_z_input = joy_msg->axes[angular_axis_index_];
          
          // 应用死区后保存
          if (std::abs(angular_z_input) < deadzone_)
            angular_z_input = 0.0;
          
          last_angular_z_input_ = angular_z_input;
        }
        else
        {
          // RB松开时，使用0值避免积分持续累加
          angular_z_input = 0.0;
        }

        // 积分方式：摇杆为正则增加，为0则保持不变，为负则减小
        integrated_linear_x_ += linear_x_input * integral_gain_linear_x_;
        integrated_linear_z_ += linear_z_input * integral_gain_linear_z_;
        integrated_angular_y_ += angular_y_input * integral_gain_angular_y_;
        integrated_angular_z_ += angular_z_input * integral_gain_angular_z_;

        // 限制积分值在合理范围内（-1到1）
        integrated_linear_x_ = clamp(integrated_linear_x_, -1.0, 1.0);
        integrated_linear_z_ = clamp(integrated_linear_z_, -1.0, 1.0);
        integrated_angular_y_ = clamp(integrated_angular_y_, -1.0, 1.0);
        integrated_angular_z_ = clamp(integrated_angular_z_, -1.0, 1.0);

        // 辅助函数：根据输入值选择正向或负向scale
        auto calculateOutput = [](double input, double scale_pos, double scale_neg) -> double {
            if (input >= 0) {
                return input * scale_pos;
            } else {
                return input * scale_neg;
            }
        };

        // 创建躯干控制消息（使用积分值）
        geometry_msgs::Twist torso_cmd;
        torso_cmd.linear.x = clamp(calculateOutput(integrated_linear_x_, torsoMax_x_, torsoMin_x_), 
                                   -torsoMin_x_, torsoMax_x_) + initialTorsoPose_x_;
        torso_cmd.linear.z = clamp(calculateOutput(integrated_linear_z_, torsoMax_z_, torsoMin_z_), 
                                   -torsoMin_z_, torsoMax_z_) + initialTorsoPose_z_;
        torso_cmd.angular.y = clamp(calculateOutput(integrated_angular_y_, torsoMax_pitch_, torsoMin_pitch_), 
                                    -torsoMin_pitch_, torsoMax_pitch_);
        torso_cmd.angular.z = clamp(calculateOutput(integrated_angular_z_, torsoMax_yaw_, torsoMin_yaw_), 
                                    -torsoMin_yaw_, torsoMax_yaw_);
        // 判断是否发布：如果积分值非零或命令值不在初始位置，则发布（允许回到初始位置）
        bool shouldPublish = (std::abs(integrated_linear_x_) >= 1e-6) || (std::abs(integrated_linear_z_) >= 1e-6) ||
                             (std::abs(integrated_angular_y_) >= 1e-6) || (std::abs(integrated_angular_z_) >= 1e-6) ||
                             (std::abs(torso_cmd.linear.x - initialTorsoPose_x_) >= deadzone_) ||
                             (std::abs(torso_cmd.linear.z - initialTorsoPose_z_) >= deadzone_) ||
                             (std::abs(torso_cmd.angular.y) >= deadzone_) ||
                             (std::abs(torso_cmd.angular.z) >= deadzone_);
        // 发布到躯干控制话题
        if(shouldPublish)
        {
          cmd_lb_torso_publisher_.publish(torso_cmd);
        }
      }
      else
      {
        // ========== cmd_vel 和 cmd_vel_world 模式 ==========
        // 读取摇杆输入值
        double linear_x_input = joy_msg->axes[linear_axis_index_x_];
        double linear_y_input = joy_msg->axes[linear_axis_index_y_];
        double angular_input = joy_msg->axes[angular_axis_index_];

        // 应用死区
        if (std::abs(linear_x_input) < deadzone_)
          linear_x_input = 0.0;
        if (std::abs(linear_y_input) < deadzone_)
          linear_y_input = 0.0;
        if (std::abs(angular_input) < deadzone_)
          angular_input = 0.0;

        // 创建 cmd_vel 消息
        geometry_msgs::Twist cmd_vel;
        cmd_vel.linear.x = clamp(linear_x_input * linear_scale_x_, -linear_scale_x_, linear_scale_x_);
        cmd_vel.linear.y = clamp(linear_y_input * linear_scale_y_, -linear_scale_y_, linear_scale_y_);
        cmd_vel.linear.z = 0.0;
        cmd_vel.angular.x = 0.0;
        cmd_vel.angular.y = 0.0;
        cmd_vel.angular.z = clamp(angular_input * angular_scale_z_, -angular_scale_z_, angular_scale_z_);

        // 判断是否接近0（使用死区阈值）
        bool isNearZero = (std::abs(cmd_vel.linear.x) < deadzone_ && 
                           std::abs(cmd_vel.linear.y) < deadzone_ && 
                           std::abs(cmd_vel.angular.z) < deadzone_);

        // 根据当前模式发布到相应的话题
        if(!isNearZero)
        {
          if (current_mode_ == ControlMode::CMD_VEL)
          {
            cmd_vel_publisher_.publish(cmd_vel);
          }
          else if (current_mode_ == ControlMode::CMD_VEL_WORLD)
          {
            cmd_vel_world_publisher_.publish(cmd_vel);
          }
        }
      }

      // 保存当前手柄消息状态
      old_joy_msg_ = *joy_msg;
    }
  };
}

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "mobile_manipulator_joy_command_node");
  ros::NodeHandle nodeHandle;

  mobile_manipulator::MobileManipulatorJoyControl joyControl(nodeHandle);
  joyControl.run();

  return 0;
}

