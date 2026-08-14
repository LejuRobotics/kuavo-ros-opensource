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
#include <kuavo_common/common/common.h>
#include <kuavo_msgs/changeTorsoCtrlMode.h>
#include <kuavo_msgs/SetJoyTopic.h>
#include <kuavo_msgs/getLbTorsoInitialPose.h>
#include <std_srvs/SetBool.h>
#include <std_srvs/Trigger.h>
#include <ocs2_msgs/mpc_observation.h>
#include <map>
#include <algorithm>
#include <cmath>
#include <iostream>
#include <cstdlib>
#include <Eigen/Core>

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

      // 与 VR 增量节点共用：可由全局参数 /mobile_manipulator_joy/* 覆盖上述默认（绝对路径与 nodeHandle 命名空间无关）
      nodeHandle_.param("/mobile_manipulator_joy/linear_scale_x", linear_scale_x_, linear_scale_x_);
      nodeHandle_.param("/mobile_manipulator_joy/linear_scale_y", linear_scale_y_, linear_scale_y_);
      nodeHandle_.param("/mobile_manipulator_joy/linear_scale_z", linear_scale_z_, linear_scale_z_);
      nodeHandle_.param("/mobile_manipulator_joy/angular_scale_z", angular_scale_z_, angular_scale_z_);
      nodeHandle_.param("/mobile_manipulator_joy/angular_scale_y", angular_scale_y_, angular_scale_y_);
      nodeHandle_.param("/mobile_manipulator_joy/deadzone", deadzone_, deadzone_);
      ROS_INFO_STREAM("[mobile_manipulator_joy] scales (m/s, rad/s): lin_x=" << linear_scale_x_
                      << " lin_y=" << linear_scale_y_ << " lin_z=" << linear_scale_z_
                      << " ang_z=" << angular_scale_z_ << " ang_y=" << angular_scale_y_
                      << " deadzone=" << deadzone_);

      int robotVersion_ = 60;
      if(nodeHandle_.hasParam("robot_version"))
      {
        int raw_version = 0;
        nodeHandle_.getParam("robot_version", raw_version);
        robotVersion_ = RobotVersion::create(raw_version).version_number();
      }
      // 躯干初始化位置xyz
      loadTorsoInitialPoseFromServer(nodeHandle_, robotVersion_);

      // 躯干 workspace 钳制限位（相对 initialTorsoPose，沿用 beta）
      torsoMax_x_ = 0.25; torsoMin_x_ = 0.0;
      torsoMax_z_ = 0.32; torsoMin_z_ =  0.0;
      torsoMax_pitch_ = 0.5235; torsoMin_pitch_ = 0.0;
      torsoMax_yaw_ = 0.5235; torsoMin_yaw_ = -0.5235;

      // 先加载 launch 指定的映射配置（sim.json / bt2.json 等）
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

      // 仿真键盘模式不检测物理手柄，避免 bt2pro 映射(angular=axis2) 覆盖 sim.json(angular=axis3)
      std::string joystick_type_early;
      nodeHandle_.getParam("joystick_type", joystick_type_early);
      if (joystick_type_early != "sim")
      {
        detectJoystickType();
      }

      // 从 joyAxisMap 获取轴索引（sim: axes[3]=右摇杆Yaw -> angular.z）
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
      cmd_torso_vel_publisher_ = nodeHandle_.advertise<geometry_msgs::Twist>("/cmd_torso_vel", 10, false);
      stop_pub_ = nodeHandle_.advertise<std_msgs::Bool>("/stop_robot", 10);
      joy_sub_ = nodeHandle_.subscribe("/joy", 10, &MobileManipulatorJoyControl::joyCallback, this);

      // 订阅MPC observation话题，用于判断是否已经完成初始化
      observation_sub_ = nodeHandle_.subscribe("/mobile_manipulator_mpc_observation", 10, &MobileManipulatorJoyControl::observationCallback, this);
      open_loop_torso_state_sub_ = nodeHandle_.subscribe<geometry_msgs::Twist>(
          "/torso_open_loop_state", 1,
          [this](const geometry_msgs::Twist::ConstPtr& msg) {
            open_loop_torso_x_ = msg->linear.x;
            open_loop_torso_z_ = msg->linear.z;
            open_loop_torso_yaw_ = msg->angular.z;
            open_loop_torso_pitch_ = msg->angular.y;
            has_open_loop_torso_state_ = true;
          });
      
      // 初始化MPC模式切换服务客户端
      mpc_control_client_ = nodeHandle_.serviceClient<kuavo_msgs::changeTorsoCtrlMode>("/mobile_manipulator_mpc_control");
      previous_mpc_mode_ = 0;  // 默认模式: No control

      // 初始化set_joy_topic服务客户端（用于躯干模式禁用auto_gait节点）
      set_joy_topic_client_ = nodeHandle_.serviceClient<kuavo_msgs::SetJoyTopic>("/set_joy_topic");

      // 初始化real_initial_start服务客户端（用于从准备状态切换到运行状态）
      real_initialize_client_ = nodeHandle_.serviceClient<std_srvs::Trigger>("/humanoid_controller/real_initial_start");

      // 初始化控制模式（默认为 cmd_vel 模式）
      current_mode_ = ControlMode::CMD_VEL;
      previous_mode_ = ControlMode::CMD_VEL;

      // 初始化旧的手柄消息（用于检测按钮按下事件）
      old_joy_msg_.axes = std::vector<float>(8, 0.0);
      old_joy_msg_.buttons = std::vector<int32_t>(12, 0);
      
      // 躯干积分增益（沿用旧参数名，内部 ×kTorsoVelCalibHz 转换为 vel_scale）
      // BT2/BT2Pro/Xbox 默认值（可 ROS param 覆盖，与 G12 同形式）
      // 体感对齐：假定遥控约 500Hz（旧 per-msg 积分等效速度 = gain × f）
      // z 默认值 ×0.2 对齐 G12：G12 右摇杆在 Python 层被 SCALE_RIGHT_STICK_Z=0.2 预缩放，
      // BT2 /joy 不预缩放，故等价于把 BT2 的 vel_scale_z 乘 0.2（0.004×0.2=0.0008 → 0.40 m/s）。
      bt2_wheelarm_integral_gain_linear_x_ = 0.001;   // ×500 = 0.50 m/s
      bt2_wheelarm_integral_gain_linear_z_ = 0.0008;  // ×500 = 0.40 m/s（对齐 G12 满杆 0.4）
      bt2_wheelarm_integral_gain_angular_y_ = 0.003;  // ×500 = 1.50 rad/s
      bt2_wheelarm_integral_gain_angular_z_ = 0.005;  // ×500 = 2.50 rad/s
      nodeHandle_.param("bt2_wheelarm_integral_gain_linear_x", bt2_wheelarm_integral_gain_linear_x_, bt2_wheelarm_integral_gain_linear_x_);
      nodeHandle_.param("bt2_wheelarm_integral_gain_linear_z", bt2_wheelarm_integral_gain_linear_z_, bt2_wheelarm_integral_gain_linear_z_);
      nodeHandle_.param("bt2_wheelarm_integral_gain_angular_y", bt2_wheelarm_integral_gain_angular_y_, bt2_wheelarm_integral_gain_angular_y_);
      nodeHandle_.param("bt2_wheelarm_integral_gain_angular_z", bt2_wheelarm_integral_gain_angular_z_, bt2_wheelarm_integral_gain_angular_z_);

      // G12 轮臂默认值（同 BT2，可 ROS param 覆盖，沿用旧名）
      g12_wheelarm_integral_gain_linear_x_ = 0.001;
      g12_wheelarm_integral_gain_linear_z_ = 0.004;
      g12_wheelarm_integral_gain_angular_y_ = 0.003;
      g12_wheelarm_integral_gain_angular_z_ = 0.005;
      nodeHandle_.param("g12_wheelarm_integral_gain_linear_x", g12_wheelarm_integral_gain_linear_x_, g12_wheelarm_integral_gain_linear_x_);
      nodeHandle_.param("g12_wheelarm_integral_gain_linear_z", g12_wheelarm_integral_gain_linear_z_, g12_wheelarm_integral_gain_linear_z_);
      nodeHandle_.param("g12_wheelarm_integral_gain_angular_y", g12_wheelarm_integral_gain_angular_y_, g12_wheelarm_integral_gain_angular_y_);
      nodeHandle_.param("g12_wheelarm_integral_gain_angular_z", g12_wheelarm_integral_gain_angular_z_, g12_wheelarm_integral_gain_angular_z_);

      // 内部转换：gain → vel_scale = gain × kTorsoVelCalibHz
      // 体感标定 500Hz（用户手感：旧实现更快；bag 中位 ~300 偏慢）
      static constexpr double kTorsoVelCalibHz = 500.0;
      torso_vel_scale_x_ = bt2_wheelarm_integral_gain_linear_x_ * kTorsoVelCalibHz;
      torso_vel_scale_z_ = bt2_wheelarm_integral_gain_linear_z_ * kTorsoVelCalibHz;
      torso_vel_scale_pitch_ = bt2_wheelarm_integral_gain_angular_y_ * kTorsoVelCalibHz;
      torso_vel_scale_yaw_ = bt2_wheelarm_integral_gain_angular_z_ * kTorsoVelCalibHz;

      g12_torso_vel_scale_x_ = g12_wheelarm_integral_gain_linear_x_ * kTorsoVelCalibHz;
      g12_torso_vel_scale_z_ = g12_wheelarm_integral_gain_linear_z_ * kTorsoVelCalibHz;
      g12_torso_vel_scale_pitch_ = g12_wheelarm_integral_gain_angular_y_ * kTorsoVelCalibHz;
      g12_torso_vel_scale_yaw_ = g12_wheelarm_integral_gain_angular_z_ * kTorsoVelCalibHz;
      
      // 初始化控制量为零的时间跟踪
      zero_control_start_time_ = ros::Time::now();
      is_control_zero_ = true;  // 初始状态认为控制量为零

      // G12轮臂模式: 需同时满足 is_wheel(ROBOT_VERSION major==6) 和 joystick_type==h12
      // 版本号格式: PPPPMMMMN, major = (version / 10) % 10000
      is_wheel_ = false;
      use_g12_ = false;
      const char* env_robot_version = std::getenv("ROBOT_VERSION");
      if (env_robot_version != nullptr)
      {
        int env_version = std::atoi(env_robot_version);
        int major = (env_version / 10) % 10000;
        is_wheel_ = (major == 6);
      }
      std::string joystick_type;
      if (nodeHandle_.getParam("joystick_type", joystick_type) && joystick_type == "h12")
      {
        if (is_wheel_)
        {
          use_g12_ = true;
          ROS_INFO("G12轮臂模式已启用 (ROBOT_VERSION=%s)", env_robot_version ? env_robot_version : "unknown");
        }
      }

      ROS_INFO("轮臂手柄控制节点已启动");
      ROS_INFO("操作提示: 使用左摇杆控制底盘移动，右摇杆控制旋转");
      ROS_INFO("模式切换: LB+A -> cmd_vel, LB+Y -> cmd_vel_world, LB+B -> 躯干控制");
      ROS_INFO("BACK: chassis E-STOP + call terminate (publish /stop_robot)");
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
    ros::Publisher cmd_torso_vel_publisher_;
    ros::Publisher stop_pub_;
    ros::Subscriber joy_sub_;
    ros::ServiceClient mpc_control_client_;
    ros::ServiceClient set_joy_topic_client_;
    ros::ServiceClient real_initialize_client_;

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

    // 躯干各轴限幅大小（沿用 beta，yaw 修正为对称 ±0.5235）
    double torsoMax_x_, torsoMin_x_;
    double torsoMax_z_, torsoMin_z_;
    double torsoMax_pitch_, torsoMin_pitch_;
    double torsoMax_yaw_, torsoMin_yaw_;
    static constexpr double kTorsoClampEps = 1e-5;

    // open-loop torso state from /torso_open_loop_state (published by ReferenceManager)
    double open_loop_torso_x_{0.0}, open_loop_torso_z_{0.0}, open_loop_torso_yaw_{0.0}, open_loop_torso_pitch_{0.0};
    bool has_open_loop_torso_state_{false};
    ros::Subscriber open_loop_torso_state_sub_;


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

    // Torso integral gains (BT2/BT2Pro, ROS-param overridable, aligned with G12)
    double bt2_wheelarm_integral_gain_linear_x_;
    double bt2_wheelarm_integral_gain_linear_z_;
    double bt2_wheelarm_integral_gain_angular_y_;
    double bt2_wheelarm_integral_gain_angular_z_;

    // G12 wheel-arm integral gains (defaults same as BT2, can be overridden via ROS param)
    double g12_wheelarm_integral_gain_linear_x_;
    double g12_wheelarm_integral_gain_linear_z_;
    double g12_wheelarm_integral_gain_angular_y_;
    double g12_wheelarm_integral_gain_angular_z_;

    // Internal vel_scale = bt2_wheelarm_integral_gain_* × kTorsoVelCalibHz (BT2/BT2Pro)
    double torso_vel_scale_x_;
    double torso_vel_scale_z_;
    double torso_vel_scale_pitch_;
    double torso_vel_scale_yaw_;

    // Internal vel_scale for G12 (= g12_wheelarm_integral_gain_* × kTorsoVelCalibHz)
    double g12_torso_vel_scale_x_;
    double g12_torso_vel_scale_z_;
    double g12_torso_vel_scale_pitch_;
    double g12_torso_vel_scale_yaw_;

    // 控制量为零的时间跟踪（用于模式切换检查）
    ros::Time zero_control_start_time_;
    bool is_control_zero_;
    const double MIN_ZERO_DURATION_ = 2.0;  // 控制量为零的最小持续时间（秒）

    // 底盘急停：按 BACK 时置位，持续发布零速度并每 10 次打印一次
    bool emergency_stop_chassis_{false};

    // G12轮臂模式标志: is_wheel_(ROBOT_VERSION>=60) 且 joystick_type==h12
    bool is_wheel_;
    bool use_g12_;
    // G+H 复位边沿保护：避免按住期间反复调用 reset 服务导致轨迹重规划震荡
    bool g12_torso_reset_latched_{false};
    ros::Time g12_torso_reset_cooldown_until_;

    // MPC observation相关标志，用于判断是否已经完成初始化
    bool get_observation_ = false;
    ros::Subscriber observation_sub_;
      
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

    // 获取躯干的初始位姿
    void loadTorsoInitialPoseFromServer(ros::NodeHandle& nh, int robotVersion) 
    {
      Eigen::VectorXd torsoPose = Eigen::VectorXd::Zero(6);
      bool isGetTorsoPose = false;

      while (ros::ok()) 
      {
          isGetTorsoPose = getTorsoInitialPose(nh, torsoPose);

          if (isGetTorsoPose) {
              // 检查位姿是否有效（非零）
              if (std::abs(torsoPose[0]) > 1e-3 || 
                  std::abs(torsoPose[1]) > 1e-3 || 
                  std::abs(torsoPose[2]) > 1e-3) 
              {
                  initialTorsoPose_x_ = torsoPose[0];
                  initialTorsoPose_y_ = torsoPose[1];
                  initialTorsoPose_z_ = torsoPose[2];
                  std::cout << "成功获取躯干初始位姿: " << torsoPose.transpose() << std::endl;
                  break;  // 获取成功，退出循环
              } else {
                  std::cerr << "\033[31m获取到零位姿, 继续重试...\033[0m"  << std::endl;
                  isGetTorsoPose = false;
              }
          } 
          else 
          {
              std::cerr << "\033[31m无法获取躯干初始位姿, 1秒后重试...\033[0m" << std::endl;
          }

          ros::Duration(1.0).sleep();  // 等待1秒
      }

      if (std::abs(initialTorsoPose_x_) > 1e-3 || 
          std::abs(initialTorsoPose_y_) > 1e-3 || 
          std::abs(initialTorsoPose_z_) > 1e-3)
      {
      }
      else
      {
        std::cout << "\033[31m无法正确加载初始位姿, 载入默认初始数值\033[0m" << std::endl;
        if(robotVersion == 60)
        {
          initialTorsoPose_x_ = 0.196123;
          initialTorsoPose_y_ = 0.0005;
          initialTorsoPose_z_ = 0.789919;
        }
        else if(robotVersion == 61 || robotVersion == 62 || robotVersion == 63 || robotVersion == 200062 || robotVersion == 300062)
        {
          initialTorsoPose_x_ = 0.11575;
          initialTorsoPose_y_ = 0.0;
          initialTorsoPose_z_ = 0.923803;
        }
      }
    }

    // 获取躯干的初始位姿
    bool getTorsoInitialPose(ros::NodeHandle& nh, Eigen::VectorXd& pose_data) 
    {
      // 等待服务可用，超时2秒
      if (!ros::service::waitForService("/mobile_manipulator_get_torso_initial_pose", ros::Duration(10.0))) {
          std::cerr << "服务不可用: /mobile_manipulator_get_torso_initial_pose (超时10秒)" << std::endl;
          return false;
      }

      ros::ServiceClient client = nh.serviceClient<kuavo_msgs::getLbTorsoInitialPose>(
          "/mobile_manipulator_get_torso_initial_pose");
      
      kuavo_msgs::getLbTorsoInitialPose srv;
      srv.request.isNeed = true;
      
      if (client.call(srv) && srv.response.result) 
      {
        pose_data << srv.response.linear.x, srv.response.linear.y, srv.response.linear.z, 
                     srv.response.angular.z, srv.response.angular.y, srv.response.angular.x;
        std::cout << "获取躯干初始位姿成功, torsoPose is: \n" << pose_data.transpose() << std::endl;
        return true;
      }
      else
      {
        std::cerr << "获取躯干初始位姿失败" << std::endl;
      }
      
      return false;
    }

    bool resetTorsoToInitialAsync(ros::NodeHandle& nh)
    {
      if (!ros::service::waitForService("/mobile_manipulator_reset_torso", ros::Duration(10.0))) {
          ROS_ERROR("Service not available");
          return false;
      }

      ros::ServiceClient client = nh.serviceClient<std_srvs::SetBool>(
          "/mobile_manipulator_reset_torso");

      std_srvs::SetBool srv;
      srv.request.data = true;

      if (client.call(srv))
      {
        if (srv.response.success)
        {
          ROS_INFO("Torso reset command sent successfully: %s", srv.response.message.c_str());
          return true;
        }
        ROS_WARN("Torso reset rejected by service: %s", srv.response.message.c_str());
        return false;
      }

      ROS_ERROR("Failed to call torso reset service");
      return false;
    }

    // 限制数值在指定范围内
    double clamp(double value, double min_value, double max_value)
    {
      return std::max(min_value, std::min(value, max_value));
    }

    // axes with deadzone → 0; used by mode-switch zero check and G12 stick reads
    double axisWithDeadzone(const sensor_msgs::Joy::ConstPtr &joy_msg, int axis_index) const
    {
      if (axis_index < 0 || static_cast<size_t>(axis_index) >= joy_msg->axes.size()) {
        return 0.0;
      }
      const double v = joy_msg->axes[axis_index];
      return std::abs(v) < deadzone_ ? 0.0 : v;
    }

    /// z-coupled x-envelope: 根据当前 z 增量返回允许的 x 正向增量幅值（0.05..0.25）。
    /// z_increment = 当前绝对 z - initialTorsoPos_.z
    double getTorsoMaxX(double z_increment) const
    {
      if (z_increment <= 0.0) {
        return 0.05;
      } else if (z_increment <= 0.3) {
        return 0.05 + (z_increment / 0.3) * 0.1;  // 0.05 → 0.15
      } else if (z_increment <= 0.5) {
        return 0.15 + ((z_increment - 0.3) / 0.2) * 0.1;  // 0.15 → 0.25
      } else {
        return 0.25;
      }
    }

    /// Read /torso_open_loop_state and zero velocity components that would exceed limits.
    void clampTorsoVelAgainstOpenLoopState(geometry_msgs::Twist& vel) const
    {
      if (!has_open_loop_torso_state_) return;

      // --- z ---
      const double z_abs_lo = initialTorsoPose_z_ + torsoMin_z_;
      const double z_abs_hi = initialTorsoPose_z_ + torsoMax_z_;

      if      (vel.linear.z > 0.0 && open_loop_torso_z_ >= z_abs_hi - kTorsoClampEps)  vel.linear.z = 0.0;
      else if (vel.linear.z < 0.0 && open_loop_torso_z_ <= z_abs_lo + kTorsoClampEps)  vel.linear.z = 0.0;

      // --- x (z-coupled envelope, 同旧代码 getTorsoMaxX) ---
      const double z_increment = open_loop_torso_z_ - initialTorsoPose_z_;
      const double x_increment = open_loop_torso_x_ - initialTorsoPose_x_;
      const double current_torso_max_x = getTorsoMaxX(z_increment);

      if      (vel.linear.x > 0.0 && x_increment >= current_torso_max_x - kTorsoClampEps) vel.linear.x = 0.0;
      else if (vel.linear.x < 0.0 && x_increment <= torsoMin_x_ + kTorsoClampEps)          vel.linear.x = 0.0;

      // --- pitch ---
      if      (vel.angular.y > 0.0 && open_loop_torso_pitch_ >= torsoMax_pitch_ - kTorsoClampEps) vel.angular.y = 0.0;
      else if (vel.angular.y < 0.0 && open_loop_torso_pitch_ <= torsoMin_pitch_ + kTorsoClampEps) vel.angular.y = 0.0;

      // --- yaw ---
      // kuavodevlab#3801: block waist yaw until torso z raised 5cm (same z gate as getTorsoMaxX at z=0)
      constexpr double kWaistYawMinZIncrement = 0.05;
      if (z_increment < kWaistYawMinZIncrement - kTorsoClampEps) {
        vel.angular.z = 0.0;
      } else if (vel.angular.z > 0.0 && open_loop_torso_yaw_ >= torsoMax_yaw_ - kTorsoClampEps) {
        vel.angular.z = 0.0;
      } else if (vel.angular.z < 0.0 && open_loop_torso_yaw_ <= torsoMin_yaw_ + kTorsoClampEps) {
        vel.angular.z = 0.0;
      }
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

    // 调用real_initial_start服务（从准备状态切换到运行状态）
    void callRealInitializeSrv()
    {
      std::cout << "callRealInitializeSrv triggered, switching from ready state to running state" << std::endl;
      std_srvs::Trigger srv;
      if (real_initialize_client_.call(srv))
      {
        if (srv.response.success)
        {
          ROS_INFO("State switch succeeded: %s", srv.response.message.c_str());
        }
        else
        {
          ROS_ERROR("State switch failed: %s", srv.response.message.c_str());
        }
      }
      else
      {
        ROS_ERROR("Failed to call service /humanoid_controller/real_initial_start");
      }
    }

    // MPC observation回调函数，收到第一个observation表示初始化完成
    void observationCallback(const ocs2_msgs::mpc_observation::ConstPtr &observation_msg)
    {
      (void)observation_msg; // 避免未使用参数警告
      if (!get_observation_)
      {
        get_observation_ = true;
        ROS_INFO("Received MPC observation, robot initialization completed");
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
        // All joysticks: torso is instantaneous velocity — zero when sticks released
        const double lx = axisWithDeadzone(joy_msg, linear_axis_index_x_);
        const double lz = axisWithDeadzone(joy_msg, linear_z_axis_index_);
        const double ay = axisWithDeadzone(joy_msg, angular_y_axis_index_);
        const double az = axisWithDeadzone(joy_msg, angular_axis_index_);
        is_zero = (std::abs(lx) < 1e-6 && std::abs(lz) < 1e-6 &&
                   std::abs(ay) < 1e-6 && std::abs(az) < 1e-6);
      }
      else
      {
        // cmd_vel模式：检查摇杆输入（应用死区）
        const double x = axisWithDeadzone(joy_msg, linear_axis_index_x_);
        const double y = axisWithDeadzone(joy_msg, linear_axis_index_y_);
        const double z = axisWithDeadzone(joy_msg, angular_axis_index_);
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
          // No local open-loop pose to reset; ReferenceManager holds the final target.
          // G12模式: 禁用humanoid_joy_control_auto_gait节点，防止底盘移动
          if (use_g12_)
          {
            kuavo_msgs::SetJoyTopic srv;
            srv.request.topic_name = "/joy_disabled";
            if (set_joy_topic_client_.call(srv))
            {
              std::cout << "G12: auto_gait node disabled (joy topic switched to /joystick_disabled)" << std::endl;
            }
            else
            {
              std::cout << "G12: failed to disable auto_gait node" << std::endl;
            }
          }
        }
        // 从躯干控制模式切换到其他模式时，恢复之前的MPC模式
        else if (current_mode_ == ControlMode::TORSO_CONTROL)
        {
          if (!callChangeMpcMode(previous_mpc_mode_))
          {
            ROS_ERROR("恢复MPC模式失败，模式: %d", previous_mpc_mode_);
          }

          // G12模式: 恢复humanoid_joy_control_auto_gait_with_vel节点
          if (use_g12_)
          {
            kuavo_msgs::SetJoyTopic srv;
            srv.request.topic_name = "/joy";
            if (set_joy_topic_client_.call(srv))
            {
              std::cout << "G12: auto_gait node resumed (joy topic switched to /joystick)" << std::endl;
            }
            else
            {
              std::cout << "G12: failed to resume auto_gait node" << std::endl;
            }
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

      // ========== G12轮臂模式（仅当 is_wheel_且joystick_type==h12 时启用） ==========
      if (use_g12_)
      {
        // G12按钮索引: Python层映射 GUIDE=8, M1=9, M2=10
        const int G12_BTN_GUIDE = 8;
        const int G12_BTN_M1 = 9;
        const int G12_BTN_M2 = 10;

        bool guide_pressed = (joy_msg->buttons.size() > (size_t)G12_BTN_GUIDE && joy_msg->buttons[G12_BTN_GUIDE] == 1);
        bool m1_pressed = (joy_msg->buttons.size() > (size_t)G12_BTN_M1 && joy_msg->buttons[G12_BTN_M1] == 1);
        bool m2_pressed = (joy_msg->buttons.size() > (size_t)G12_BTN_M2 && joy_msg->buttons[G12_BTN_M2] == 1);
        bool old_m2_pressed = (old_joy_msg_.buttons.size() > (size_t)G12_BTN_M2 && old_joy_msg_.buttons[G12_BTN_M2] == 1);
        // G+H 同时处于极值时，禁止摇杆继续发 torso 速度，避免与复位服务抢轨迹
        const bool gh_combo_active = guide_pressed && m1_pressed;
        const ros::Time now = ros::Time::now();
        const bool torso_reset_cooldown_active =
            (!g12_torso_reset_cooldown_until_.isZero() && now < g12_torso_reset_cooldown_until_);

        // G+H同时极值2秒 -> Python层BUTTON_M2=1 -> 躯干复位
        // 必须边沿触发：按住期间若持续 call reset 服务，会反复 resetAllMpcTrajAndTarget 导致震荡/撞限位
        if (m2_pressed && !old_m2_pressed && !g12_torso_reset_latched_)
        {
          // G12 torso has no open-loop integrated pose; only re-arm mode-switch zero timer
          zero_control_start_time_ = now;
          is_control_zero_ = true;
          g12_torso_reset_latched_ = true;
          // 给底层 Ruckig 复位留出执行窗口，期间发零速度避免 sticky /cmd_torso_vel
          g12_torso_reset_cooldown_until_ = now + ros::Duration(3.0);

          resetTorsoToInitialAsync(nodeHandle_);
          std::cout << "G+H torso reset triggered (edge)" << std::endl;
        }
        if (!m2_pressed && !gh_combo_active)
        {
          g12_torso_reset_latched_ = false;
        }

        // G极值 + A/B/C 模式切换
        // Python层G12按钮映射: A->buttons[3](Y), B->buttons[1](B), C->buttons[2](X)
        // 复位组合键期间禁止切模式，避免与 reset 冲突
        if (guide_pressed && !gh_combo_active && !torso_reset_cooldown_active)
        {
          const int G12_BTN_A = 3;  // Python层: channel 7(A) -> BUTTON_Y(3)
          const int G12_BTN_B = 1;  // Python层: channel 8(B) -> BUTTON_B(1)
          const int G12_BTN_C = 2;  // Python层: channel 9(C) -> BUTTON_X(2)

          bool a_just = (joy_msg->buttons.size() > (size_t)G12_BTN_A && joy_msg->buttons[G12_BTN_A] == 1 &&
                         old_joy_msg_.buttons.size() > (size_t)G12_BTN_A && old_joy_msg_.buttons[G12_BTN_A] == 0);
          bool b_just = (joy_msg->buttons.size() > (size_t)G12_BTN_B && joy_msg->buttons[G12_BTN_B] == 1 &&
                         old_joy_msg_.buttons.size() > (size_t)G12_BTN_B && old_joy_msg_.buttons[G12_BTN_B] == 0);
          bool c_just = (joy_msg->buttons.size() > (size_t)G12_BTN_C && joy_msg->buttons[G12_BTN_C] == 1 &&
                         old_joy_msg_.buttons.size() > (size_t)G12_BTN_C && old_joy_msg_.buttons[G12_BTN_C] == 0);

          if (a_just) switchControlMode(ControlMode::CMD_VEL);
          if (b_just) switchControlMode(ControlMode::CMD_VEL_WORLD);
          if (c_just) switchControlMode(ControlMode::TORSO_CONTROL);
        }

        // C+D长按急停
        int btn_back = joyButtonMap["BUTTON_BACK"];
        bool back_just = (joy_msg->buttons.size() > (size_t)btn_back && joy_msg->buttons[btn_back] == 1 &&
                          old_joy_msg_.buttons.size() > (size_t)btn_back && old_joy_msg_.buttons[btn_back] == 0);
        if (back_just)
        {
          callTerminateSrv();
        }

        // 根据模式分发控制
        if (current_mode_ == ControlMode::TORSO_CONTROL)
        {
          // G12 躯干速度接口（无开环绝对位姿状态）
          // G极值: 左杆上下->vx, 右杆上下->vz
          // H极值: 右杆左右->vyaw, 左杆上下->vpitch
          // Workspace limits enforced here via velocity gate; no residual state across soft-pause.
          double lx = 0.0, lz = 0.0, ay = 0.0, az = 0.0;
          if (!gh_combo_active && !torso_reset_cooldown_active)
          {
            if (guide_pressed) {  // G极值: 左杆上下->vx, 右杆上下->vz
              lx = axisWithDeadzone(joy_msg, linear_axis_index_x_);
              lz = axisWithDeadzone(joy_msg, linear_z_axis_index_);
            }
            if (m1_pressed) {  // H极值: 右杆左右->vyaw, 左杆上下->vpitch
              az = axisWithDeadzone(joy_msg, angular_axis_index_);
              ay = axisWithDeadzone(joy_msg, linear_axis_index_x_);
            }
          }

          geometry_msgs::Twist torso_vel;
          torso_vel.linear.x = lx * g12_torso_vel_scale_x_;
          torso_vel.linear.z = lz * g12_torso_vel_scale_z_;
          torso_vel.angular.y = ay * g12_torso_vel_scale_pitch_;
          torso_vel.angular.z = az * g12_torso_vel_scale_yaw_;
          clampTorsoVelAgainstOpenLoopState(torso_vel);
          cmd_torso_vel_publisher_.publish(torso_vel);
        }
        else
        {
          // cmd_vel / cmd_vel_world 模式: 摇杆控制底盘
          double lx = joy_msg->axes[linear_axis_index_x_];
          double ly = joy_msg->axes[linear_axis_index_y_];
          double az = joy_msg->axes[angular_axis_index_];
          if (std::abs(lx) < deadzone_) lx = 0.0;
          if (std::abs(ly) < deadzone_) ly = 0.0;
          if (std::abs(az) < deadzone_) az = 0.0;

          geometry_msgs::Twist cmd;
          cmd.linear.x = clamp(lx * linear_scale_x_, -linear_scale_x_, linear_scale_x_);
          cmd.linear.y = clamp(ly * linear_scale_y_, -linear_scale_y_, linear_scale_y_);
          cmd.linear.z = 0.0;
          cmd.angular.x = 0.0;
          cmd.angular.y = 0.0;
          cmd.angular.z = clamp(az * angular_scale_z_, -angular_scale_z_, angular_scale_z_);

          if (std::abs(cmd.linear.x) >= deadzone_ || std::abs(cmd.linear.y) >= deadzone_ || std::abs(cmd.angular.z) >= deadzone_)
          {
            if (current_mode_ == ControlMode::CMD_VEL)
              cmd_vel_publisher_.publish(cmd);
            else if (current_mode_ == ControlMode::CMD_VEL_WORLD)
              cmd_vel_world_publisher_.publish(cmd);
          }
        }

        old_joy_msg_ = *joy_msg;
        return;  // G12模式处理完毕
      }

      // ========== BT2Pro / BT2 / Xbox：底盘逻辑同前；躯干改为 /cmd_torso_vel ==========

      // 检测RT轴是否按下（用于复位功能，RT轴值通常小于-0.5表示按下）
      // 此复位逻辑在BT2/Xbox分支顶层处理，与G12的M2复位一致，确保在任何模式下都能复位躯干。
      bool rt_pressed = (joy_msg->axes[rt_axis_index_] < -0.5);
      bool old_rt_pressed = (old_joy_msg_.axes.size() > static_cast<size_t>(rt_axis_index_) &&
                             old_joy_msg_.axes[rt_axis_index_] < -0.5);
      bool rt_just_pressed = rt_pressed && !old_rt_pressed;

      if (rt_just_pressed)
      {
        zero_control_start_time_ = ros::Time::now();
        is_control_zero_ = true;
        resetTorsoToInitialAsync(nodeHandle_);
        ROS_INFO("Torso reset command sent from BT2/Xbox");
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

      // BACK：按下时底盘急停并调用退出程序（/stop_robot），按住期间持续发布零速度
      bool back_pressed = (joy_msg->buttons.size() > static_cast<size_t>(joyButtonMap["BUTTON_BACK"]) &&
                                joy_msg->buttons[joyButtonMap["BUTTON_BACK"]]);
      if (back_pressed && !old_joy_msg_.buttons[joyButtonMap["BUTTON_BACK"]])
        callTerminateSrv();
      emergency_stop_chassis_ = back_pressed;

      // START：按下时调用real_initial_start服务，从准备状态切换到运行状态
      // 只有当未收到MPC observation（未初始化）时才允许调用，避免重复初始化
      bool start_pressed = (joy_msg->buttons.size() > static_cast<size_t>(joyButtonMap["BUTTON_START"]) &&
                                joy_msg->buttons[joyButtonMap["BUTTON_START"]]);
      if (!get_observation_ && start_pressed && !old_joy_msg_.buttons[joyButtonMap["BUTTON_START"]])
        callRealInitializeSrv();

      // 根据当前模式处理不同的控制逻辑
      if (current_mode_ == ControlMode::TORSO_CONTROL)
      {
        // ========== 躯干控制模式（速度接口，无上层开环绝对位姿）==========
        // 与旧 BT2 相同的修饰键语义：LB 管 x/z，RB 管 pitch/yaw；RT 复位已在分支顶层处理。
        // Workspace limits enforced here via velocity gate; open-loop integration in ReferenceManager (cmdTorsoPose_).

        bool rb_pressed = (joy_msg->buttons[joyButtonMap["BUTTON_RB"]] == 1);

        double lx = 0.0, lz = 0.0, ay = 0.0, az = 0.0;
        if (lb_pressed)
        {
          lx = axisWithDeadzone(joy_msg, linear_axis_index_x_);
          lz = axisWithDeadzone(joy_msg, linear_z_axis_index_);
        }
        if (rb_pressed)
        {
          ay = axisWithDeadzone(joy_msg, angular_y_axis_index_);
          az = axisWithDeadzone(joy_msg, angular_axis_index_);
        }

        geometry_msgs::Twist torso_vel;
        torso_vel.linear.x = lx * torso_vel_scale_x_;
        torso_vel.linear.z = lz * torso_vel_scale_z_;
        torso_vel.angular.y = ay * torso_vel_scale_pitch_;
        torso_vel.angular.z = az * torso_vel_scale_yaw_;
        clampTorsoVelAgainstOpenLoopState(torso_vel);
        cmd_torso_vel_publisher_.publish(torso_vel);
      }
      else
      {
        // ========== cmd_vel 和 cmd_vel_world 模式 ==========
        if (emergency_stop_chassis_)
        {
          // 底盘急停：向两个话题发布零速度，每 10 次打印一次
          geometry_msgs::Twist zero_cmd;
          zero_cmd.linear.x = zero_cmd.linear.y = zero_cmd.linear.z = 0.0;
          zero_cmd.angular.x = zero_cmd.angular.y = zero_cmd.angular.z = 0.0;
          cmd_vel_publisher_.publish(zero_cmd);
          cmd_vel_world_publisher_.publish(zero_cmd);
          
            ROS_INFO("Chassis Emergency Stop");
        }
        else
        {
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
