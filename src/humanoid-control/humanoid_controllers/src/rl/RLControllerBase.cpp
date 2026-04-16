// Pinocchio must be included before Boost headers
#include <pinocchio/fwd.hpp>

#include "humanoid_controllers/rl/RLControllerBase.h"
#include "humanoid_controllers/rl/armController.h"
#include <ros/ros.h>
#include <sstream>
#include <thread>
#include <vector>
#include <ocs2_core/misc/LoadData.h>

namespace humanoid_controller
{

using namespace ocs2;

  RLControllerBase::RLControllerBase(const std::string& name, RLControllerType type, const std::string& config_file,
                                     ros::NodeHandle& nh, 
                                     ocs2::humanoid::TopicLogger* ros_logger)
    : name_(name)
    , type_(type)
    , state_(ControllerState::INITIALIZING)
    , initialized_(false)
    , config_file_(config_file)
    , nh_(nh)
    , ros_logger_(ros_logger)
  {
    // 自动初始化服务
    initializeServices();
    // 初始化RL相关变量
    initializeRLVariables();
  }

  void RLControllerBase::initializeServices()
  {
    // 服务命名空间为 /humanoid_controllers/{controller_name}
    std::string service_ns = "/humanoid_controllers/" + name_;
    
    reload_srv_ = nh_.advertiseService(service_ns + "/reload", 
                                        &RLControllerBase::reloadServiceCallback, this);
    is_active_srv_ = nh_.advertiseService(service_ns + "/isActive", 
                                          &RLControllerBase::isActiveServiceCallback, this);
    get_state_srv_ = nh_.advertiseService(service_ns + "/getState", 
                                           &RLControllerBase::getStateServiceCallback, this);
    get_type_srv_ = nh_.advertiseService(service_ns + "/getType", 
                                          &RLControllerBase::getTypeServiceCallback, this);
    reset_srv_ = nh_.advertiseService(service_ns + "/reset", 
                                      &RLControllerBase::resetServiceCallback, this);
    
    ROS_INFO("[%s] Services initialized at namespace: %s", name_.c_str(), service_ns.c_str());
  }
  
  void RLControllerBase::initializeRLVariables()
  {
    // 从ROS参数获取关节数量（humanoidController会在init中设置这些参数）
    if (!nh_.hasParam("/legRealDof"))
    {
      ROS_ERROR("[%s] ROS parameter /legRealDof not found", name_.c_str());
      return;
    }
    if (!nh_.hasParam("/armRealDof"))
    {
      ROS_ERROR("[%s] ROS parameter /armRealDof not found", name_.c_str());
      return;
    }
    if (!nh_.hasParam("/waistRealDof"))
    {
      ROS_ERROR("[%s] ROS parameter /waistRealDof not found", name_.c_str());
      return;
    }
    if (!nh_.hasParam("/headRealDof"))
    {
      ROS_ERROR("[%s] ROS parameter /headRealDof not found", name_.c_str());
      return;
    }
    
    nh_.param("/legRealDof", jointNum_, 12);
    nh_.param("/armRealDof", jointArmNum_, 8);
    nh_.param("/waistRealDof", waistNum_, 1);
    nh_.param("/headRealDof", headNum_, 2);
    
    // 计算动作维度
    num_actions_ = jointNum_ + jointArmNum_ + waistNum_;
    
    // 初始化RL相关向量（维度基于关节数量，与humanoidController一致）
    int total_joints = jointNum_ + jointArmNum_ + waistNum_;
    defalutJointPosRL_.resize(total_joints);
    defaultBaseStateRL_.resize(12);
    JointControlModeRL_.resize(total_joints);
    JointPDModeRL_.resize(total_joints);
    jointKpRL_.resize(total_joints);
    jointKdRL_.resize(total_joints);
    torqueLimitsRL_.resize(total_joints);
    actionScaleTestRL_.resize(total_joints);
    motorPdoKp_.resize(total_joints);
    motorPdoKd_.resize(total_joints);
    
    // 初始化为零
    defalutJointPosRL_.setZero();
    defaultBaseStateRL_.setZero();
    JointControlModeRL_.setZero();
    JointPDModeRL_.setZero();
    jointKpRL_.setZero();
    jointKdRL_.setZero();
    torqueLimitsRL_.setZero();
    actionScaleTestRL_.setZero();
    motorPdoKp_.setZero();
    motorPdoKd_.setZero();
    
    ROS_INFO("[%s] RL variables initialized: jointNum_=%d, jointArmNum_=%d, waistNum_=%d, headNum_=%d, num_actions_=%d", 
             name_.c_str(), jointNum_, jointArmNum_, waistNum_, headNum_, num_actions_);
  }

  bool RLControllerBase::reloadServiceCallback(std_srvs::Trigger::Request& req, 
                                                std_srvs::Trigger::Response& res)
  {
    if (state_ == ControllerState::RUNNING)
    {
      res.success = false;
      res.message = "Controller is running, stop or pause before reload";
      return true;
    }

    bool success = reload();
    res.success = success;
    res.message = success ? "Config reloaded successfully" : "Failed to reload config";
    return true;
  }

  bool RLControllerBase::isActiveServiceCallback(std_srvs::Trigger::Request& req, 
                                                  std_srvs::Trigger::Response& res)
  {
    res.success = isActive();
    res.message = res.success ? "Controller is active" : "Controller is not active";
    return true;
  }

  bool RLControllerBase::getStateServiceCallback(std_srvs::Trigger::Request& req, 
                                                 std_srvs::Trigger::Response& res)
  {
    res.success = true;
    std::stringstream ss;
    ss << static_cast<int>(state_);
    res.message = ss.str();
    return true;
  }

  bool RLControllerBase::getTypeServiceCallback(std_srvs::Trigger::Request& req, 
                                                std_srvs::Trigger::Response& res)
  {
    res.success = true;
    std::stringstream ss;
    ss << static_cast<int>(type_);
    res.message = ss.str();
    return true;
  }

  bool RLControllerBase::resetServiceCallback(std_srvs::Trigger::Request& req, 
                                               std_srvs::Trigger::Response& res)
  {
    if (state_ == ControllerState::RUNNING)
    {
      res.success = false;
      res.message = "Controller is running, stop or pause before reset";
      return true;
    }

    reset();
    res.success = true;
    res.message = "Controller reset successfully";
    return true;
  }

  void RLControllerBase::clip(Eigen::VectorXd &a, double limit)
  {
    a = a.cwiseMax(-limit).cwiseMin(limit);
  }

  bool RLControllerBase::update(const ros::Time& time, 
                                 const SensorData& sensor_data,
                                 const Eigen::VectorXd& measuredRbdState,
                                 kuavo_msgs::jointCmd& joint_cmd)
  {
    // 在基类中统一进行传感器预处理（例如 RL 相关的 IMU 滤波）
    SensorData processed_sensor_data = sensor_data;
    preprocessSensorData(processed_sensor_data);

    // 存储传感器数据和状态（线程安全）
    {
      std::lock_guard<std::recursive_mutex> lock(sensor_data_mtx_);
      setRobotState(measuredRbdState);
      setRobotSensorData(processed_sensor_data);
      sensor_data_updated_ = true;
    }
    
    // 调用派生类的实现
    bool success = updateImpl(time, processed_sensor_data, measuredRbdState, joint_cmd);
        
    // 如果启用了手臂指令替换，调用updateArmCommand来替换手臂部分
    if (success && arm_command_replacement_enabled_)
    {
      updateArmCommand(time, processed_sensor_data, joint_cmd);
    }
    
    // 如果启用了腰部指令替换，调用updateWaistCommand来替换腰部部分
    if (success && waist_command_replacement_enabled_)
    {
      updateWaistCommand(time, processed_sensor_data, joint_cmd);
    }
    
    // 如果 use_default_motor_csp_kpkd_ 为 false，使用从 info 文件加载的 motorPdoKp_ 和 motorPdoKd_ 替换所有 control_modes==2 的关节
    if (success && !use_default_motor_csp_kpkd_)
    {
      const int total_joints = jointNum_ + jointArmNum_ + waistNum_;
      const int cmd_size = static_cast<int>(joint_cmd.control_modes.size());
      
      // 确保 motorPdoKp_ 和 motorPdoKd_ 已正确初始化且大小匹配
      if (motorPdoKp_.size() == total_joints && 
          motorPdoKd_.size() == total_joints &&
          cmd_size >= total_joints)
      {
        for (int i = 0; i < total_joints && i < cmd_size; ++i)
        {
          if (joint_cmd.control_modes[i] == 2)
          {
            // 确保 joint_kp 和 joint_kd 向量有足够的空间
            if (i < static_cast<int>(joint_cmd.joint_kp.size()) && 
                i < static_cast<int>(joint_cmd.joint_kd.size()))
            {
              joint_cmd.joint_kp[i] = motorPdoKp_[i];
              joint_cmd.joint_kd[i] = motorPdoKd_[i];
            }
          }
        }
      }
    }
    
    return success;
  }

  bool RLControllerBase::loadRLFilterParams(const std::string& config_file)
  {
    try
    {
      // 从 RL 配置文件中加载滤波相关参数（与 humanoidController 中保持一致）
      loadData::loadEigenMatrix(config_file, "accFilterCutoffFreq", accFilterCutoffFreqRL_);
      loadData::loadEigenMatrix(config_file, "freeAccFilterCutoffFreq", freeAccFilterCutoffFreqRL_);
      loadData::loadEigenMatrix(config_file, "gyroFilterCutoffFreq", gyroFilterCutoffFreqRL_);
      loadData::loadEigenMatrix(config_file, "accFilterState", accFilterStateRL_);
      loadData::loadEigenMatrix(config_file, "freeAccFilterState", freeAccFilterStateRL_);
      loadData::loadEigenMatrix(config_file, "gyroFilterState", gyroFilterStateRL_);

      // 控制频率：优先从配置文件加载 controlFrequency，否则使用 /wbc_frequency ROS 参数
      // 控制频率同时决定滤波器的采样时间 dt
      double wbc_frequency = 500.0;
      if (!nh_.getParam("/wbc_frequency", wbc_frequency))
      {
        ROS_WARN("[%s] /wbc_frequency not found in ROS params, using default: %.1f Hz",
                 name_.c_str(), wbc_frequency);
      }
      if (wbc_frequency <= 0.0)
      {
        ROS_WARN("[%s] Invalid /wbc_frequency (%.3f), fallback to 500 Hz", name_.c_str(), wbc_frequency);
        wbc_frequency = 500.0;
      }

      // 尝试从配置文件加载 controlFrequency，如果没有则使用 wbc_frequency
      try
      {
        loadData::loadCppDataType(config_file, "controlFrequency", control_frequency_);
        ROS_INFO("[%s] Loaded controlFrequency from config: %.1f Hz", name_.c_str(), control_frequency_);
      }
      catch (const std::exception& e)
      {
        // 配置文件中没有 controlFrequency，使用 /wbc_frequency
        control_frequency_ = wbc_frequency;
        ROS_INFO("[%s] controlFrequency not in config, using /wbc_frequency: %.1f Hz", name_.c_str(), control_frequency_);
      }

      // 初始化控制频率 Rate 对象
      control_rate_ = std::make_unique<ros::Rate>(control_frequency_);

      // 使用控制频率计算滤波器采样时间 dt
      double dt = 1.0 / control_frequency_;

      accFilterRL_.setParams(dt, accFilterCutoffFreqRL_);
      freeAccFilterRL_.setParams(dt, freeAccFilterCutoffFreqRL_);
      gyroFilterRL_.setParams(dt, gyroFilterCutoffFreqRL_);

      rl_filter_initialized_ = true;
      ROS_INFO("[%s] Control frequency: %.1f Hz, dt: %.6f s", name_.c_str(), control_frequency_, dt);
      
      // 加载切换相关参数
      // use_interploate_from_mpc: 从MPC切换到这个模型时是否使用插值过渡
      // true: 使用插值过渡，false: 直接切换
      try
      {
        loadData::loadCppDataType(config_file, "use_interploate_from_mpc", use_interpolate_from_mpc_);
      }
      catch (const std::exception& e)
      {
        ROS_WARN("[%s] Failed to load use_interploate_from_mpc from %s: %s, using default (direct switch)",
                 name_.c_str(), config_file.c_str(), e.what());
        // 使用默认值 false（直接切换）
        use_interpolate_from_mpc_ = false;
      }

      // 加载 use_default_motor_csp_kpkd 参数
      // true: 使用kuavo.json中的默认kp/kd，false: 使用info文件中的motor_kp/motor_kd
      try
      {
        loadData::loadCppDataType(config_file, "use_default_motor_csp_kpkd", use_default_motor_csp_kpkd_);
      }
      catch (const std::exception& e)
      {
        ROS_WARN("[%s] Failed to load use_default_motor_csp_kpkd from %s: %s, using default (true)",
                 name_.c_str(), config_file.c_str(), e.what());
        // 使用默认值 true（使用默认kp/kd）
        use_default_motor_csp_kpkd_ = true;
      }

      // 如果 use_default_motor_csp_kpkd_ 为 false，尝试加载 motor_pdo_kp 和 motor_pdo_kd
      if (!use_default_motor_csp_kpkd_)
      {
        try
        {
          loadData::loadEigenMatrix(config_file, "motor_pdo_kp", motorPdoKp_);
          loadData::loadEigenMatrix(config_file, "motor_pdo_kd", motorPdoKd_);
          ROS_INFO("[%s] Loaded motor_pdo_kp and motor_pdo_kd from config file", name_.c_str());
        }
        catch (const std::exception& e)
        {
          ROS_WARN("[%s] Failed to load motor_pdo_kp/motor_pdo_kd from %s: %s, will use zero values",
                   name_.c_str(), config_file.c_str(), e.what());
          motorPdoKp_.setZero();
          motorPdoKd_.setZero();
        }
      }
      
      return true;
    }
    catch (const std::exception& e)
    {
      ROS_WARN("[%s] Failed to load RL IMU filter params from %s: %s",
               name_.c_str(), config_file.c_str(), e.what());
      rl_filter_initialized_ = false;
      return false;
    }
  }

  void RLControllerBase::preprocessSensorData(SensorData& sensor_data)
  {
    // 如果没有正确初始化 RL 滤波器，则不做任何处理
    if (!rl_filter_initialized_)
    {
      ROS_WARN("[%s] RL IMU filters not initialized, skipping filtering", name_.c_str());
      return;
    }

    // 与 humanoidController.cpp 中 RL 部分保持一致的 IMU 滤波逻辑
    Eigen::Vector3d acc_filtered = accFilterRL_.update(sensor_data.linearAccel_);
    Eigen::Vector3d free_acc_filtered = freeAccFilterRL_.update(sensor_data.freeLinearAccel_);
    Eigen::Vector3d gyro_filtered = gyroFilterRL_.update(sensor_data.angularVel_);

    for (int i = 0; i < 3; ++i)
    {
      sensor_data.linearAccel_(i) =
        accFilterStateRL_(i) * acc_filtered(i) +
        (1.0 - accFilterStateRL_(i)) * sensor_data.linearAccel_(i);

      sensor_data.freeLinearAccel_(i) =
        freeAccFilterStateRL_(i) * free_acc_filtered(i) +
        (1.0 - freeAccFilterStateRL_(i)) * sensor_data.freeLinearAccel_(i);

      sensor_data.angularVel_(i) =
        gyroFilterStateRL_(i) * gyro_filtered(i) +
        (1.0 - gyroFilterStateRL_(i)) * sensor_data.angularVel_(i);
    }
  }

  SensorData RLControllerBase::getRobotSensorData() const
  {
    std::lock_guard<std::recursive_mutex> lock(sensor_data_mtx_);
    return stored_sensor_data_;
  }
  void RLControllerBase::setRobotSensorData(const SensorData &sensor_data)
  {
    std::lock_guard<std::recursive_mutex> lock(sensor_data_mtx_);
    stored_sensor_data_ = sensor_data;
  }
  void RLControllerBase::setRobotState(const Eigen::VectorXd &state)
  {
    std::lock_guard<std::recursive_mutex> lock(sensor_data_mtx_);
    stored_measured_state_ = state;
  }

  Eigen::VectorXd RLControllerBase::getRobotState() const
  {
    std::lock_guard<std::recursive_mutex> lock(sensor_data_mtx_);
    return stored_measured_state_;
  }

  Eigen::VectorXd RLControllerBase::getCurrentAction() const
  {
    std::lock_guard<std::mutex> lock(action_mtx_);
    return actions_;
  }

  void RLControllerBase::setCurrentAction(Eigen::VectorXd action)
  {
    std::lock_guard<std::mutex> lock(action_mtx_);
    actions_ = action;
  }
  void RLControllerBase::start()
  {
    if (!initialized_)
    {
      ROS_WARN("[%s] Cannot start controller: not initialized", name_.c_str());
      return;
    }

    // 创建推理线程（如果尚未创建）
    if (!inference_thread_created_)
    {
      inference_thread_created_ = true;
      inference_thread_ = std::thread(&RLControllerBase::inferenceThreadFunc, this);
      ROS_INFO("[%s] Inference thread created", name_.c_str());
    }
    state_ = ControllerState::PAUSED;
   
  }

  void RLControllerBase::stop()
  {
    // 停止推理线程
    // 设置状态为 INITIALIZING，让推理线程退出循环
    state_ = ControllerState::STOPPED;
    if (inference_thread_created_)
    {
      if (inference_thread_.joinable())
      {
        inference_thread_.join();
      }
      inference_thread_created_ = false;
      ROS_INFO("[%s] Inference thread stopped and destroyed", name_.c_str());
    }
  }

  void RLControllerBase::pause()
  {
    if (state_ == ControllerState::RUNNING)
    {
      state_ = ControllerState::PAUSED;
      ROS_INFO("[%s] Controller paused (inference thread continues but will skip execution)", name_.c_str());
    }
    else
    {
      ROS_WARN("[%s] Cannot pause controller: current state=%d", 
               name_.c_str(), static_cast<int>(state_));
    }
    // 暂停之后需要等待新的传感器数据
    sensor_data_updated_ = false;
  }

  void RLControllerBase::resume()
  {
    if (state_ != ControllerState::STOPPED)
    {
      state_ = ControllerState::RUNNING;
      ROS_INFO("[%s] Controller resumed (inference thread will execute)", name_.c_str());
    }
    else
    {
      ROS_WARN("[%s] Cannot resume controller: current state=%d (expected PAUSED)",
               name_.c_str(), static_cast<int>(state_));
    }
  }

  void RLControllerBase::waitForNextCycle()
  {
    control_rate_->sleep();
  }

  bool RLControllerBase::inference(const Eigen::VectorXd& observation, Eigen::VectorXd& action)
  {
    // 基础实现：直接返回零动作
    // 派生类应该重写此方法以实现具体的推理逻辑
    // 注意：此方法只负责计算并返回action，不更新成员变量
    // 成员变量的更新由推理线程（inferenceThreadFunc）负责
    action = Eigen::VectorXd::Zero(num_actions_);
    ROS_WARN_THROTTLE(1.0, "[%s] Using base inference (returns zero action). Override in derived class.", name_.c_str());
    return true;
  }

  void RLControllerBase::updateObservation(const Eigen::VectorXd& state_est, const SensorData& sensor_data)
  {
    // 基础实现：空实现
    // 派生类应该重写此方法以实现具体的观测更新逻辑
  }

  void RLControllerBase::actionToJointCmd(const Eigen::VectorXd& actuation, 
                                           const Eigen::VectorXd& measuredRbdState,
                                           kuavo_msgs::jointCmd& joint_cmd)
  {
    // 基础实现：空实现
    // 派生类应该重写此方法以实现具体的转换逻辑
    // 注意：此方法需要派生类提供必要的成员变量（如is_real_, JointControlModeRL_等）
    ROS_WARN_THROTTLE(1.0, "[%s] Using base actionToJointCmd (empty implementation). Override in derived class.", name_.c_str());
  }

  bool RLControllerBase::updateArmCommand(const ros::Time& time,
                                          const SensorData& sensor_data,
                                          kuavo_msgs::jointCmd& joint_cmd)
  {
    // 基础实现：空实现，返回false表示未使用外部手臂指令替换
    // 派生类可以重写此方法以实现自定义的手臂控制逻辑
    return false;
  }

  bool RLControllerBase::updateWaistCommand(const ros::Time& time,
                                           const SensorData& sensor_data,
                                           kuavo_msgs::jointCmd& joint_cmd)
  {
    // 基础实现：空实现，返回false表示未使用外部腰部指令替换
    // 派生类可以重写此方法以实现自定义的腰部控制逻辑
    return false;
  }

  bool RLControllerBase::reload()
  {
    // 自动调用派生类的loadConfig方法重新加载配置文件
    if (!config_file_.empty())
    {
      ROS_INFO("[%s] Reloading config from: %s", name_.c_str(), config_file_.c_str());
      bool success = loadConfig(config_file_);
      if (success)
      {
        ROS_INFO("[%s] Config reloaded successfully", name_.c_str());
        return true;
      }
      else
      {
        ROS_ERROR("[%s] Failed to reload config from: %s", name_.c_str(), config_file_.c_str());
        return false;
      }
    }
    else
    {
      ROS_WARN("[%s] Cannot reload: config_file_ is empty", name_.c_str());
      return false;
    }
  }

  bool RLControllerBase::shouldRunInference() const
  {
    // 基础实现：只有在RUNNING状态时才执行推理
    return state_ == ControllerState::RUNNING;
  }

  void RLControllerBase::inferenceThreadFunc()
  {
    ros::Rate inference_rate(inference_frequency_);
    
    // 线程持续运行，直到控制器被停止（状态变为 INITIALIZING）
    while (ros::ok() && state_ != ControllerState::STOPPED)
    {
      // 检查是否应该执行推理（派生类可以重写shouldRunInference添加特定条件）
      if (!shouldRunInference() || !sensor_data_updated_)
      {
        inference_rate.sleep();
        continue;
      }
      
      // 从存储的成员变量获取传感器数据和状态
      SensorData sensors_data = getRobotSensorData();
      Eigen::VectorXd measuredRbdState = getRobotState();
      sensor_data_updated_ = false;

      // 更新观测（派生类实现具体逻辑，会更新networkInputDataRL_）
      updateObservation(measuredRbdState, sensors_data);
      // 执行推理（派生类实现具体逻辑，使用networkInputDataRL_作为输入）
      Eigen::VectorXd action;
      if (inference(networkInputDataRL_, action))
      {

        // 推理成功，更新actions_成员变量（线程安全）
        setCurrentAction(action);
        
        // 发布动作数据（如果ros_logger_可用）
        if (ros_logger_)
        {
          ros_logger_->publishVector("/rl_controller/actions", action);
        }
      }
      
      inference_rate.sleep();
    }
    
    ROS_INFO("[%s] Inference thread exiting", name_.c_str());
  }

  void RLControllerBase::updateVelocityLimitsParam(ros::NodeHandle& nh)
  {
    // 初始化默认值: [linear_x, linear_y, linear_z, angular_x, angular_y, angular_z]
    Eigen::VectorXd mpc_limits(6);
    mpc_limits << 0.4, 0.2, 0.3, 0.0, 0.0, 0.4;
    
    // 从referenceFile配置文件读取，如果存在则覆盖默认值
    std::string referenceFile;
    if (nh.getParam("/referenceFile", referenceFile))
    {
      try
      {
        loadData::loadCppDataType(referenceFile, "cmdvelLinearXLimit", mpc_limits(0));
        loadData::loadCppDataType(referenceFile, "cmdvelLinearYLimit", mpc_limits(1));
        loadData::loadCppDataType(referenceFile, "cmdvelLinearZLimit", mpc_limits(2));
        loadData::loadCppDataType(referenceFile, "cmdvelAngularYAWLimit", mpc_limits(5));
      }
      catch (const std::exception& e)
      {
        // 读取失败时使用默认值，不打印警告（某些参数可能不存在是正常的）
      }
    }
    
    // 设置到rosparam
    std::vector<double> limits_vec(mpc_limits.data(), mpc_limits.data() + mpc_limits.size());
    nh.setParam("/velocity_limits", limits_vec);
    
    ROS_INFO("[%s] Updated /velocity_limits with MPC default: [%.2f, %.2f, %.2f, %.2f, %.2f, %.2f]",
             name_.c_str(),
             mpc_limits(0), mpc_limits(1), mpc_limits(2),
             mpc_limits(3), mpc_limits(4), mpc_limits(5));
  }

} // namespace humanoid_controller







