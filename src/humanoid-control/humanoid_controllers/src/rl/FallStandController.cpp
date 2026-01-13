#include "humanoid_controllers/rl/FallStandController.h"
#include <ros/ros.h>
#include <fstream>
#include <sstream>
#include <filesystem>
#include <ocs2_core/misc/LoadData.h>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/info_parser.hpp>
#include <atomic>
#include <thread>
#include <std_srvs/Trigger.h>
#include <std_srvs/SetBool.h>
#include <cmath>

namespace humanoid_controller
{

  FallStandController::FallStandController(const std::string& name, const std::string& config_file,
                                           ros::NodeHandle& nh,
                                           ocs2::humanoid::TopicLogger* ros_logger)
    : RLControllerBase(name, RLControllerType::FALL_STAND_CONTROLLER, config_file, nh, ros_logger)
  {
  }

  FallStandController::~FallStandController()
  {
    // 基类的stop()会自动停止推理线程
    stop();
  }

  bool FallStandController::initialize()
  {
    // 设置ROS服务（使用基类的nh_引用）
    trigger_fall_stand_up_srv_ = nh_.advertiseService("/humanoid_controller/trigger_fall_stand_up", 
                                                      &FallStandController::triggerFallStandUpCallback, this);

    
    if (!loadConfig(config_file_))
    {
      ROS_ERROR("[%s] Failed to load config file: %s", name_.c_str(), config_file_.c_str());
      exit(1);
      return false;
    }

    // 加载两个模型（如果配置了双模型模式）
    try
    {
      // 加载趴着模型
      compiled_model_prone_ = core_.compile_model(network_model_file_prone_, "CPU");
      infer_request_prone_ = compiled_model_prone_.create_infer_request();
      model_loaded_prone_ = true;
      ROS_INFO("[%s] Prone model loaded successfully: %s", name_.c_str(), network_model_file_prone_.c_str());
      
      // 加载躺着模型（如果路径不同）
      if (network_model_file_supine_ != network_model_file_prone_)
      {
        compiled_model_supine_ = core_.compile_model(network_model_file_supine_, "CPU");
        infer_request_supine_ = compiled_model_supine_.create_infer_request();
        model_loaded_supine_ = true;
        ROS_INFO("[%s] Supine model loaded successfully: %s", name_.c_str(), network_model_file_supine_.c_str());
      }
      else
      {
        // 如果路径相同，复用同一个模型
        compiled_model_supine_ = compiled_model_prone_;
        infer_request_supine_ = compiled_model_supine_.create_infer_request();
        model_loaded_supine_ = true;
        ROS_INFO("[%s] Supine model reusing prone model (same path)", name_.c_str());
      }
      
      // 默认使用趴着模型
      compiled_model_ = compiled_model_prone_;
      infer_request_ = compiled_model_.create_infer_request();
      current_model_type_ = FallStandModelType::PRONE;
    }
    catch (const std::exception& e)
    {
      ROS_ERROR("[%s] Failed to load model: %s", name_.c_str(), e.what());
      exit(1);
      return false;
    }

    // 加载两个轨迹（如果配置了双轨迹模式）
    std::filesystem::path configPath(config_file_);
    
    // 加载趴着轨迹
    if (!trajectory_file_prone_.empty())
    {
      std::string trajectoryFilePathProne = (configPath.parent_path() / trajectory_file_prone_).string();
      trajectory_loaded_prone_ = loadMotionTrajectory(trajectoryFilePathProne, motion_trajectory_prone_);
      if (!trajectory_loaded_prone_)
      {
        ROS_ERROR("[%s] Failed to load prone trajectory file: %s", name_.c_str(), trajectoryFilePathProne.c_str());
        exit(1);
      }
      else
      {
        ROS_INFO("[%s] Prone trajectory loaded successfully: %s", name_.c_str(), trajectoryFilePathProne.c_str());
        
        // 初始化fall_stand_init_joints_prone_
        Eigen::VectorXd init_joints_prone = getTrajectoryCommand(motion_trajectory_prone_).head(jointNum_ + jointArmNum_ + waistNum_);
        fall_stand_init_joints_prone_ = init_joints_prone;
        moveVectorEntry(fall_stand_init_joints_prone_, 0, 12);
        fall_stand_init_joints_prone_[12] = -fall_stand_init_joints_prone_[12];
      }
    }
    
    // 加载躺着轨迹
    if (!trajectory_file_supine_.empty())
    {
      std::string trajectoryFilePathSupine = (configPath.parent_path() / trajectory_file_supine_).string();
      trajectory_loaded_supine_ = loadMotionTrajectory(trajectoryFilePathSupine, motion_trajectory_supine_);
      if (!trajectory_loaded_supine_)
      {
        ROS_ERROR("[%s] Failed to load supine trajectory file: %s", name_.c_str(), trajectoryFilePathSupine.c_str());
        exit(1);
      }
      else
      {
        ROS_INFO("[%s] Supine trajectory loaded successfully: %s", name_.c_str(), trajectoryFilePathSupine.c_str());
        
        // 初始化fall_stand_init_joints_supine_
        Eigen::VectorXd init_joints_supine = getTrajectoryCommand(motion_trajectory_supine_).head(jointNum_ + jointArmNum_ + waistNum_);
        fall_stand_init_joints_supine_ = init_joints_supine;
        moveVectorEntry(fall_stand_init_joints_supine_, 0, 12);
        fall_stand_init_joints_supine_[12] = -fall_stand_init_joints_supine_[12];
      }
    }
    
    // 默认使用趴着轨迹
    motion_trajectory_ = motion_trajectory_prone_;
    trajectory_loaded_ = trajectory_loaded_prone_;
    fall_stand_init_joints_ = fall_stand_init_joints_prone_;

    // 初始化ankleSolver（从ROS参数获取，如果不存在则使用默认值）
    int ankle_solver_type = 0; // 默认值
    // 尝试从ROS参数获取ankle_solver_type
    if (!nh_.getParam("/ankle_solver_type", ankle_solver_type))
    {
      ROS_WARN("[%s] ankle_solver_type not found in ROS params, using default: %d", name_.c_str(), ankle_solver_type);
    }
    else
    {
      ROS_INFO("[%s] AnkleSolver type loaded from ROS params: %d", name_.c_str(), ankle_solver_type);
    }
    ankleSolver_.getconfig(ankle_solver_type);
    ROS_INFO("[%s] AnkleSolver initialized with type: %d", name_.c_str(), ankle_solver_type);

    // 初始化动作和观测数据（与humanoidController一致）
    singleInputDataRL_.resize(numSingleObsRL_);
    networkInputDataRL_.resize(numSingleObsRL_ * frameStackRL_);
    actions_.resize(num_actions_);  // 基类的actions_
    singleInputDataRL_.setZero();
    networkInputDataRL_.setZero();
    actions_.setZero();  // 基类的actions_
    
    // 初始化input_deque（与humanoidController一致）
    for (int i = 0; i < frameStackRL_; i++)
    {
      input_deque.push_back(singleInputDataRL_);
    }

    Eigen::VectorXd init_root_pos = getTrajectoryAnchorPos();
    Eigen::Quaterniond init_root_quat = getTrajectoryAnchorQuat();
    // 将姿态角转换为四元数
    double roll = initialStateRL_(9);  // angular_x
    double pitch = initialStateRL_(10); // angular_y  
    double yaw = initialStateRL_(11);    // angular_z
    
    // 计算四元数 (ZYX顺序) !!!!
    double cy = cos(yaw * 0.5);
    double sy = sin(yaw * 0.5);
    double cp = cos(pitch * 0.5);
    double sp = sin(pitch * 0.5);
    double cr = cos(roll * 0.5);
    double sr = sin(roll * 0.5);
    
    double qw = cr * cp * cy + sr * sp * sy;
    double qx = sr * cp * cy - cr * sp * sy;
    double qy = cr * sp * cy + sr * cp * sy;
    double qz = cr * cp * sy - sr * sp * cy;

    int num_joints = initialStateRL_.size() - 12;  // 关节数量
    vector_t mujoco_q = vector_t::Zero(7 + num_joints);
    mujoco_q << init_root_pos,init_root_quat.w(),init_root_quat.x(),init_root_quat.y(),init_root_quat.z(), fall_stand_init_joints_;
    std::cout << "fall_stand_init_joints_:"<<fall_stand_init_joints_.transpose()<<std::endl;
    std::vector<double> mujoco_init_state;
    std::vector<double> squat_initial_state_vector(12 + num_joints, 0);
    for (int i = 0; i < mujoco_q.size(); i++)
    {
      mujoco_init_state.push_back(mujoco_q(i));
    }
    for(int i=0;i<fall_stand_init_joints_.size();i++)
    {
        squat_initial_state_vector[12+i] = fall_stand_init_joints_(i);
    }

    bool init_fall_down_state = false;
    if (!nh_.getParam("/init_fall_down_state", init_fall_down_state))
    {
      ROS_ERROR("[%s] Failed to get init_fall_down_state from ROS params", name_.c_str());
    }
    if (init_fall_down_state) {
      ros::param::set("robot_init_state_param", mujoco_init_state);
      ros::param::set("/squat_initial_state", squat_initial_state_vector);
      ROS_INFO("[%s] init_fall_down_state is true, set robot_init_state_param and squat_initial_state", name_.c_str());
    }

    initialized_ = true;
    // 初始化时自动启动推理线程
    start();
    
    ROS_INFO("[%s] Controller initialized successfully", name_.c_str());
    return true;
  }

  bool FallStandController::loadConfig(const std::string& config_file)
  {
    ROS_INFO("[%s] Loading config from: %s", name_.c_str(), config_file.c_str());
    
    try
    {
      boost::property_tree::ptree pt;
      boost::property_tree::read_info(config_file, pt);
      
      // 加载模型文件路径（与humanoidController对齐：先从ROS参数服务器获取）
      std::string network_model_root_path,network_model_file_name;
      nh_.getParam("/network_model_file", network_model_root_path);
      
      // 尝试加载两个模型的路径（如果存在），否则使用向后兼容的单模型路径
      std::string network_model_file_name_prone, network_model_file_name_supine;
      bool has_prone_model = (pt.find("networkModelFileProne") != pt.not_found());
      bool has_supine_model = (pt.find("networkModelFileSupine") != pt.not_found());
      
      if (has_prone_model && has_supine_model)
      {
        // 双模型模式
        loadData::loadPtreeValue(pt, network_model_file_name_prone, "networkModelFileProne", false);
        loadData::loadPtreeValue(pt, network_model_file_name_supine, "networkModelFileSupine", false);
        network_model_file_prone_ = network_model_root_path + network_model_file_name_prone;
        network_model_file_supine_ = network_model_root_path + network_model_file_name_supine;
        network_model_file_ = network_model_file_prone_; // 默认使用趴着模型
        ROS_INFO("[%s] Dual model mode: prone=%s, supine=%s", 
                 name_.c_str(), network_model_file_prone_.c_str(), network_model_file_supine_.c_str());
      }
      else
      {
        // 向后兼容：单模型模式
        loadData::loadCppDataType(config_file, "networkModelFile", network_model_file_name);
        network_model_file_ = network_model_root_path + network_model_file_name;
        network_model_file_prone_ = network_model_file_; // 默认作为趴着模型
        network_model_file_supine_ = network_model_file_; // 如果没有指定，使用同一个模型
        ROS_INFO("[%s] Single model mode (backward compatible): %s", 
                 name_.c_str(), network_model_file_.c_str());
      }
      
      // 加载轨迹文件路径
      std::string trajectory_file_name_prone, trajectory_file_name_supine;
      bool has_prone_traj = (pt.find("trajectoryFileProne") != pt.not_found());
      bool has_supine_traj = (pt.find("trajectoryFileSupine") != pt.not_found());
      
      if (has_prone_traj && has_supine_traj)
      {
        // 双轨迹模式
        loadData::loadPtreeValue(pt, trajectory_file_name_prone, "trajectoryFileProne", false);
        loadData::loadPtreeValue(pt, trajectory_file_name_supine, "trajectoryFileSupine", false);
        trajectory_file_prone_ = trajectory_file_name_prone;
        trajectory_file_supine_ = trajectory_file_name_supine;
        trajectory_file_ = trajectory_file_prone_; // 默认使用趴着轨迹
        ROS_INFO("[%s] Dual trajectory mode: prone=%s, supine=%s", 
                 name_.c_str(), trajectory_file_prone_.c_str(), trajectory_file_supine_.c_str());
      }
      else
      {
        // 向后兼容：单轨迹模式
        loadData::loadCppDataType(config_file, "trajectoryFile", trajectory_file_);
        trajectory_file_prone_ = trajectory_file_; // 默认作为趴着轨迹
        trajectory_file_supine_ = trajectory_file_; // 如果没有指定，使用同一个轨迹
        ROS_INFO("[%s] Single trajectory mode (backward compatible): %s", 
                 name_.c_str(), trajectory_file_.c_str());
      }
      double inference_freq = 0.0;
      loadData::loadCppDataType(config_file, "inferenceFrequency", inference_freq);
      // 设置基类的推理频率
      inference_frequency_ = inference_freq;
      loadData::loadCppDataType(config_file, "numSingleObs", numSingleObsRL_);
      loadData::loadCppDataType(config_file, "frameStack", frameStackRL_);
      
      // 加载插值相关参数
      if (pt.find("fallStandMaxJointVelocity") != pt.not_found())
      {
        loadData::loadCppDataType(config_file, "fallStandMaxJointVelocity", fall_stand_max_joint_velocity_);
      }
      
      // 加载RL控制相关参数
      loadData::loadEigenMatrix(config_file, "defaultJointState", defalutJointPosRL_);
      loadData::loadEigenMatrix(config_file, "defaultBaseState", defaultBaseStateRL_);
      loadData::loadEigenMatrix(config_file, "JointControlMode", JointControlModeRL_);
      loadData::loadEigenMatrix(config_file, "JointPDMode", JointPDModeRL_);
      loadData::loadEigenMatrix(config_file, "jointKp", jointKpRL_);
      loadData::loadEigenMatrix(config_file, "jointKd", jointKdRL_);
      loadData::loadEigenMatrix(config_file, "torqueLimits", torqueLimitsRL_);
      loadData::loadEigenMatrix(config_file, "actionScaleTest", actionScaleTestRL_);
      
      // 设置 initialStateRL_ = [defaultBaseStateRL_(12) + defalutJointPosRL_]
      initialStateRL_.resize(12 + defalutJointPosRL_.size());
      initialStateRL_ << defaultBaseStateRL_, defalutJointPosRL_;
      loadData::loadCppDataType(config_file, "actionScale", actionScaleRL_);
      loadData::loadCppDataType(config_file, "clipActions", clipActionsRL_);
      loadData::loadCppDataType(config_file, "clipObservations", clipObservationsRL_);
      loadData::loadCppDataType(config_file, "residualAction", residualAction_);
      loadData::loadCppDataType(config_file, "withArm", withArmRL_);
      
      // 从ROS参数获取is_real_和is_roban_（如果未设置则给出提示，并使用默认值）
      if (!nh_.getParam("/is_real", is_real_))
      {
        ROS_WARN("[%s] /is_real not found in ROS params, using default: %d", name_.c_str(), static_cast<int>(is_real_));
      }
      if (!nh_.getParam("/is_roban", is_roban_))
      {
        ROS_WARN("[%s] /is_roban not found in ROS params, using default: %d", name_.c_str(), static_cast<int>(is_roban_));
      }
      
      // 在基类中初始化 RL 相关 IMU 滤波器（如果配置中提供了参数）
      loadRLFilterParams(config_file);
      
      // 加载单输入数据配置（与humanoidController::loadRLSettings一致）
      const std::string prefixSingleInputData_ = "singleInputData";
      double startIdx_ = 0, mumIdx_ = 0, obsScale_ = 0;
      int num_ = 0;
      singleInputDataRLKeys.clear();
      singleInputDataRLID_.clear();
      
      for (const auto &pair : pt)
      {
        if (pair.first == prefixSingleInputData_)
        {
          for (const auto &pair2 : pair.second)
          {
            singleInputDataRLKeys.push_back(pair2.first);
            loadData::loadPtreeValue(pt, startIdx_, prefixSingleInputData_ + "." + pair2.first + ".startIdx", false);
            loadData::loadPtreeValue(pt, mumIdx_, prefixSingleInputData_ + "." + pair2.first + ".numIdx", false);
            loadData::loadPtreeValue(pt, obsScale_, prefixSingleInputData_ + "." + pair2.first + ".obsScales", false);
            num_ += static_cast<int>(mumIdx_);
            singleInputDataRLID_[pair2.first] = {startIdx_, mumIdx_, obsScale_};
          }
        }
      }
      
      // 验证singleInputData的总数是否与numSingleObsRL_一致（与humanoidController一致）
      if (num_ > 0 && num_ != numSingleObsRL_)
      {
        ROS_ERROR("[%s] Error: singleInputData number (%d) is not equal to 'numSingleObsRL_' (%d)", 
                  name_.c_str(), num_, numSingleObsRL_);
        return false;
      }
      
      ROS_INFO("[%s] Config loaded: network_model=%s, trajectory=%s, inference_freq=%.1f, num_obs=%d, num_actions=%d",
               name_.c_str(), network_model_file_.c_str(), trajectory_file_.c_str(), 
               inference_freq, num_obs_, num_actions_);
      ROS_INFO("[%s] Joint numbers: leg=%d, arm=%d, waist=%d, head=%d",
               name_.c_str(), jointNum_, jointArmNum_, waistNum_, headNum_);
      if (!singleInputDataRLKeys.empty())
      {
        ROS_INFO("[%s] Loaded %zu singleInputData keys", name_.c_str(), singleInputDataRLKeys.size());
      }
      
      return true;
    }
    catch (const std::exception& e)
    {
      ROS_ERROR("[%s] Failed to load config file: %s", name_.c_str(), e.what());
      return false;
    }
  }

  bool FallStandController::updateImpl(const ros::Time& time, 
                                        const SensorData& sensor_data,
                                        const Eigen::VectorXd& measuredRbdState,
                                        kuavo_msgs::jointCmd& joint_cmd)
  {
    ros_logger_->publishValue("/humanoid_controller/FallStandController/fall_stand_state_", static_cast<int>(fall_stand_state_));

    if (!initialized_)
    {
      ROS_ERROR("[%s] Controller not initialized", name_.c_str());
      return false;
    }

    // 根据状态处理不同的逻辑
    if (fall_stand_state_ == FallStandState::FALL_DOWN)
    {
      // 倒地状态：发送零指令
      int total_joints = jointNum_ + jointArmNum_ + waistNum_ + headNum_;
      joint_cmd.header.stamp = time;
      joint_cmd.joint_q.assign(total_joints, 0.0);
      joint_cmd.joint_v.assign(total_joints, 0.0);
      joint_cmd.tau.assign(total_joints, 0.0);
      joint_cmd.tau_ratio.assign(total_joints, 1.0);
      joint_cmd.tau_max.assign(total_joints, 10.0);
      joint_cmd.joint_kp.assign(total_joints, 0.0);
      joint_cmd.joint_kd.assign(total_joints, 0.0);
      joint_cmd.control_modes.assign(total_joints, 0);
    }
    else if (fall_stand_state_ == FallStandState::READY_FOR_STAND_UP)
    {
      // READY阶段：执行关节空间插值
      updateFallStandInterpolation(time, sensor_data, measuredRbdState, joint_cmd);
      if (request_for_stand_up_ && is_fall_stand_interpolating_complete_)
      {
        // 根据当前机体姿态自动判断并切换模型
        autoSelectAndSwitchModel();
        
        // 触发倒地启动过程
        motion_trajectory_.resetTimeStep();
        SensorData sensor_data = getRobotSensorData();
        auto mat = sensor_data.quat_.toRotationMatrix();
        // 计算yaw偏移
        double current_yaw = std::atan2(mat(1, 2), mat(0, 2)); 
        my_yaw_offset_ = current_yaw - motion_trajectory_.reference_yaw;
        std::cout << "getRobotSensorData().quat_: [" << sensor_data.quat_.w() << ", " 
                  << sensor_data.quat_.x() << ", " << sensor_data.quat_.y() << ", " << sensor_data.quat_.z() << "]" << std::endl;
        std::cout << "Current yaw: " << current_yaw << std::endl;
        std::cout << "Reference yaw: " << motion_trajectory_.reference_yaw << std::endl;
        // 归一化到[-π, π]范围
        while (my_yaw_offset_ > M_PI) my_yaw_offset_ -= 2 * M_PI;
        while (my_yaw_offset_ < -M_PI) my_yaw_offset_ += 2 * M_PI;
        
        actions_.setZero();
        fall_stand_state_ = FallStandState::STAND_UP;
        request_for_stand_up_ = false;
      }
    }
    else // (fall_stand_state_ == FallStandState::STAND_UP)
    {
      // STAND_UP阶段：使用轨迹或RL控制
      Eigen::VectorXd actuation = updateRLcmd(measuredRbdState);
      
      // 转换为关节命令（使用与humanoidController一致的逻辑）
      actionToJointCmd(actuation, measuredRbdState, joint_cmd);
      joint_cmd.header.stamp = time;
    }
    return true;
  }

  void FallStandController::preprocessSensorData(SensorData& sensor_data)
  {
    // 先执行基类中的通用滤波逻辑（RL IMU 滤波）
    RLControllerBase::preprocessSensorData(sensor_data);
    
    if (is_roban_)
    {
      // 将腰部关节数据从index 12移动到index 0
      moveVectorEntry(sensor_data.jointPos_, 12, 0);
      moveVectorEntry(sensor_data.jointVel_, 12, 0);
      moveVectorEntry(sensor_data.jointAcc_, 12, 0);
      moveVectorEntry(sensor_data.jointCurrent_, 12, 0);
      sensor_data.jointPos_[0] = -sensor_data.jointPos_[0];
      sensor_data.jointVel_[0] = -sensor_data.jointVel_[0];
      sensor_data.jointAcc_[0] = -sensor_data.jointAcc_[0];
      sensor_data.jointCurrent_[0] = -sensor_data.jointCurrent_[0];
    }
  }

  void FallStandController::reset()
  {
    motion_trajectory_.resetTimeStep();
    fall_stand_state_ = FallStandState::FALL_DOWN;
    is_fall_stand_interpolating_ = false;
    is_fall_stand_interpolating_complete_ = false;
    actions_.setZero();
    ROS_INFO("[%s] Controller reset", name_.c_str());
  }

  void FallStandController::resume()
  {
    RLControllerBase::resume();
    ROS_INFO("[%s] Controller resumed, reset state", name_.c_str());
    reset();
  }

  bool FallStandController::isReadyToExit() const
  {
    // 当控制器处于 STANDING 状态时，表示已完成起身任务，可以退出
    return fall_stand_state_ == FallStandState::STANDING;
  }

  bool FallStandController::inference(const Eigen::VectorXd& observation, Eigen::VectorXd& action)
  {
    // 注意：参数observation保留用于接口兼容，但实际使用networkInputDataRL_（与humanoidController一致）
    try
    {
      // 根据当前模型类型选择对应的模型和推理请求
      ov::CompiledModel* current_compiled_model = nullptr;
      ov::InferRequest* current_infer_request = nullptr;
      
      if (current_model_type_ == FallStandModelType::PRONE)
      {
        current_compiled_model = &compiled_model_prone_;
        current_infer_request = &infer_request_prone_;
      }
      else // SUPINE
      {
        current_compiled_model = &compiled_model_supine_;
        current_infer_request = &infer_request_supine_;
      }
      
      // 与humanoidController一致：每次推理都创建新的infer_request_
      *current_infer_request = current_compiled_model->create_infer_request();
      
      const auto input_port = current_compiled_model->input();
      // 检查输入维度是否匹配（与humanoidController一致）
      const auto expected_input_shape = input_port.get_shape();
      const size_t expected_input_length = expected_input_shape[1]; // 假设形状为 [batch_size, input_dim]
      const size_t actual_input_length = networkInputDataRL_.size();
      
      if (actual_input_length != expected_input_length)
      {
        ROS_ERROR_THROTTLE(1.0, "[%s] networkInputDataRL_ size mismatch: actual=%ld vs expected=%ld", 
                           name_.c_str(), actual_input_length, expected_input_length);
        action = Eigen::VectorXd::Zero(num_actions_);
        return false;
      }

      // 准备输入数据（与humanoidController一致，使用networkInputDataRL_）
      Eigen::VectorXf float_network_input = networkInputDataRL_.cast<float>();
      ov::Tensor input_tensor(input_port.get_element_type(), input_port.get_shape(), float_network_input.data());
      current_infer_request->set_input_tensor(input_tensor);
      
      // 执行推理（与humanoidController一致）
      current_infer_request->start_async();
      current_infer_request->wait();
      
      // 获取输出
      const auto output_tensor = current_infer_request->get_output_tensor();
      const size_t output_buf_length = output_tensor.get_size();
      // 注意：data<float>() 在 OpenVINO 2026.0 中将返回 const T*，这里使用 const_cast 消除废弃警告
      // 由于我们只是读取数据，使用 const float* 是安全的
      const float* output_buf = const_cast<const float*>(output_tensor.data<float>());
      
      // 检查输出维度是否匹配（与humanoidController一致，考虑withArmRL_）
      const size_t expected_output_length = withArmRL_ ? jointNum_ + waistNum_ + jointArmNum_ : jointNum_ + waistNum_;
      if (output_buf_length != expected_output_length)
      {
        ROS_ERROR_THROTTLE(1.0, "[%s] Output size mismatch: actual=%ld vs expected=%ld (withArmRL_=%d)", 
                           name_.c_str(), output_buf_length, expected_output_length, withArmRL_);
        action = Eigen::VectorXd::Zero(num_actions_);
        return false;
      }
      
      // 复制输出到action参数（不更新成员变量，由推理线程负责更新）
      action.resize(output_buf_length);
      for (int i = 0; i < output_buf_length; ++i)
      {
        action[i] = output_buf[i];
      }
      
      // 使用基类的clip函数裁剪动作
      clip(action, clipActionsRL_);
      
      return true;
    }
    catch (const std::exception& e)
    {
      ROS_ERROR_THROTTLE(1.0, "[%s] Inference failed: %s", name_.c_str(), e.what());
      action = Eigen::VectorXd::Zero(num_actions_);
      return false;
    }
  }




  Eigen::VectorXd FallStandController::updateRLcmd(const Eigen::VectorXd& state)
  {
    // 获取传感器数据
    SensorData sensor_data = getRobotSensorData();
    Eigen::VectorXd jointPos_ = sensor_data.jointPos_;
    Eigen::VectorXd jointVel_ = sensor_data.jointVel_;
    Eigen::VectorXd jointKpLocal = jointKpRL_;
    Eigen::VectorXd jointKdLocal = jointKdRL_;
    Eigen::VectorXd torqueLimitsLocal = torqueLimitsRL_;

    Eigen::VectorXd actuation(jointNum_ + jointArmNum_ + waistNum_);
    Eigen::VectorXd cmd(jointNum_ + jointArmNum_ + waistNum_);
    Eigen::VectorXd torque(jointNum_ + jointArmNum_ + waistNum_);
    
    Eigen::VectorXd motorPos_ = jointPos_;
    Eigen::VectorXd motorVel_ = jointVel_;
    // 使用基类的线程安全方法获取当前动作
    Eigen::VectorXd local_action = getCurrentAction();
    
    /*****************************************倒地起身*****************************************************************/ 
    Eigen::VectorXd temp;
    if (residualAction_ == true) {
      temp = defalutJointPosRL_;
      defalutJointPosRL_ = getTrajectoryCommand().head(jointNum_ + jointArmNum_ + waistNum_);
    }    
    /*****************************************倒地起身*****************************************************************/ 
    
    if (!withArmRL_)
    {
      local_action.tail(jointArmNum_ + waistNum_).setZero();
    }
    
    // 计算关节扭矩（包含ankleSolver处理，与humanoidController一致）
    Eigen::VectorXd jointTor_(jointNum_ + jointArmNum_ + waistNum_);
    if(is_roban_) {
      motorPos_.segment(waistNum_, jointNum_) = ankleSolver_.joint_to_motor_position(jointPos_.segment(waistNum_, jointNum_));
      motorVel_.segment(waistNum_, jointNum_) = ankleSolver_.joint_to_motor_velocity(jointPos_.segment(waistNum_, jointNum_), motorPos_.segment(waistNum_, jointNum_), jointVel_.segment(waistNum_, jointNum_));
      jointTor_ = -(jointKdLocal.cwiseProduct(motorVel_));
      jointTor_.segment(waistNum_, jointNum_) = ankleSolver_.motor_to_joint_torque(jointPos_.segment(waistNum_, jointNum_), motorPos_.segment(waistNum_, jointNum_), jointTor_.segment(waistNum_, jointNum_));
    } else {
      motorPos_.head(jointNum_) = ankleSolver_.joint_to_motor_position(jointPos_.head(jointNum_));
      motorVel_.head(jointNum_) = ankleSolver_.joint_to_motor_velocity(jointPos_.head(jointNum_), motorPos_.head(jointNum_), jointVel_.head(jointNum_));
      jointTor_ = -(jointKdLocal.cwiseProduct(motorVel_));
      jointTor_.head(jointNum_) = ankleSolver_.motor_to_joint_torque(jointPos_.head(jointNum_), motorPos_.head(jointNum_), jointTor_.head(jointNum_));
    }
    
    for (int i = 0; i < jointNum_ + jointArmNum_ + waistNum_; i++)
    {
      jointTor_(i) = jointTor_(i) + jointKpLocal(i) * (local_action[i] * actionScaleRL_ * actionScaleTestRL_[i] - jointPos_[i] + defalutJointPosRL_[i]);
    }
    
    if (is_real_)
    {
      for (int i = 0; i < jointNum_ + jointArmNum_ + waistNum_; i++)
      {
        if (i < JointControlModeRL_.size() && JointControlModeRL_(i) == 0)
        {
          if (i < JointPDModeRL_.size() && JointPDModeRL_(i) == 0)
          {
            cmd[i] = jointKpLocal[i] * (local_action[i] * actionScaleRL_ * actionScaleTestRL_[i] - jointPos_[i] + defalutJointPosRL_[i]) - jointKdLocal[i] * jointVel_[i];
            cmd[i] = std::clamp(cmd[i], -torqueLimitsLocal[i], torqueLimitsLocal[i]);
            torque[i] = cmd[i];
          }
          else
          {
            cmd[i] = (local_action[i] * actionScaleRL_ * actionScaleTestRL_[i] + defalutJointPosRL_[i]);
            torque[i] = jointKpLocal[i] * (local_action[i] * actionScaleRL_ * actionScaleTestRL_[i] - jointPos_[i] + defalutJointPosRL_[i]) - jointKdLocal[i] * jointVel_[i];
          }
        }
        else if (i < JointControlModeRL_.size() && JointControlModeRL_(i) == 2)
        {
          cmd[i] = jointKpLocal[i] * (local_action[i] * actionScaleRL_ * actionScaleTestRL_[i] - jointPos_[i] + defalutJointPosRL_[i]);
          torque[i] = jointTor_[i];
        }
        else
        {
          cmd[i] = 0.0;
          torque[i] = 0.0;
        }
      }
    }
    else
    {
      for (int i = 0; i < jointNum_ + jointArmNum_ + waistNum_; i++)
      {
        if (i < JointControlModeRL_.size() && JointControlModeRL_(i) == 0)
        {
          cmd[i] = jointKpLocal[i] * (local_action[i] * actionScaleRL_ * actionScaleTestRL_[i] - jointPos_[i] + defalutJointPosRL_[i]) - jointKdLocal[i] * jointVel_[i];
        }
        else if (i < JointControlModeRL_.size() && JointControlModeRL_(i) == 2)
        {
          cmd[i] = jointTor_[i];
        }
        else
        {
          cmd[i] = 0.0;
        }
        cmd[i] = std::clamp(cmd[i], -torqueLimitsLocal[i], torqueLimitsLocal[i]);
        torque[i] = cmd[i];
      }
    }
    
    /*****************************************倒地起身*****************************************************************/ 
    if (residualAction_ == true) {
      defalutJointPosRL_ = temp;
    }    
    /*****************************************倒地起身*****************************************************************/ 

    actuation = cmd;

    return actuation;
  }

  void FallStandController::actionToJointCmd(const Eigen::VectorXd& actuation, 
                                             const Eigen::VectorXd& measuredRbdState,
                                             kuavo_msgs::jointCmd& joint_cmd)
  {
    // 按照humanoidController.cpp (2880-2978)的逻辑处理
    // 获取当前关节位置和速度（从measuredRbdState，与humanoidController一致）
    int total_body_joints = jointNum_ + jointArmNum_ + waistNum_;
    Eigen::VectorXd current_jointPos, current_jointVel;

    {
      // 如果state结构不同，尝试从传感器数据获取
      SensorData sensor_data = getRobotSensorData();
      current_jointPos = sensor_data.jointPos_.head(total_body_joints);
      current_jointVel = sensor_data.jointVel_.head(total_body_joints);
    }
    
    // 确保joint_cmd有足够的空间
    joint_cmd.joint_q.clear();
    joint_cmd.joint_v.clear();
    joint_cmd.tau.clear();
    joint_cmd.tau_ratio.clear();
    joint_cmd.tau_max.clear();
    joint_cmd.joint_kp.clear();
    joint_cmd.joint_kd.clear();
    joint_cmd.control_modes.clear();
    
    // 按照humanoidController.cpp (2923-2979)的逻辑处理
    if (!is_real_)
    {
      // 仿真环境
      for (int i1 = 0; i1 < total_body_joints; ++i1)
      {
        joint_cmd.joint_q.push_back(0.0);
        joint_cmd.joint_v.push_back(0.0);
        joint_cmd.joint_kp.push_back(jointKpRL_[i1]);
        joint_cmd.joint_kd.push_back(jointKdRL_[i1]);
        joint_cmd.tau.push_back(actuation(i1));
        joint_cmd.tau_ratio.push_back(1);
        joint_cmd.tau_max.push_back(torqueLimitsRL_[i1]);
        joint_cmd.control_modes.push_back(JointControlModeRL_(i1));
      }
    }
    else
    {
      // 真实机器人
      for (int i1 = 0; i1 < total_body_joints; ++i1)
      {
        if (JointControlModeRL_(i1) == 0)
        {
          if (JointPDModeRL_(i1) == 0)
          {
            joint_cmd.joint_q.push_back(0.0);
            joint_cmd.joint_v.push_back(0.0);
            joint_cmd.joint_kp.push_back(0);
            joint_cmd.joint_kd.push_back(0);
            joint_cmd.tau.push_back(actuation(i1));
            joint_cmd.tau_ratio.push_back(1);
            joint_cmd.tau_max.push_back(torqueLimitsRL_[i1]);
            joint_cmd.control_modes.push_back(JointControlModeRL_(i1));
          }
          else
          {
            joint_cmd.joint_q.push_back(actuation(i1));
            joint_cmd.joint_v.push_back(0.0);
            joint_cmd.joint_kp.push_back(jointKpRL_[i1]);
            joint_cmd.joint_kd.push_back(jointKdRL_[i1]);
            joint_cmd.tau.push_back(0.0);
            joint_cmd.tau_ratio.push_back(1);
            joint_cmd.tau_max.push_back(torqueLimitsRL_[i1]);
            joint_cmd.control_modes.push_back(JointControlModeRL_(i1));
          }
        }
        else
        {
          joint_cmd.joint_q.push_back(current_jointPos(i1));
          joint_cmd.joint_v.push_back(0.0);
          joint_cmd.joint_kp.push_back(jointKpRL_[i1]);
          joint_cmd.joint_kd.push_back(jointKdRL_[i1]);
          joint_cmd.tau.push_back(actuation(i1));
          joint_cmd.tau_ratio.push_back(1);
          joint_cmd.tau_max.push_back(torqueLimitsRL_[i1]);
          joint_cmd.control_modes.push_back(JointControlModeRL_(i1));
        }
      }
    }
    
    // 设置头部关节（保持零位）
    for (int i = 0; i < headNum_; ++i)
    {
      joint_cmd.joint_q.push_back(0.0);
      joint_cmd.joint_v.push_back(0.0);
      joint_cmd.tau.push_back(0.0);
      joint_cmd.tau_ratio.push_back(1.0);
      joint_cmd.tau_max.push_back(10.0);
      joint_cmd.joint_kp.push_back(0.0);
      joint_cmd.joint_kd.push_back(0.0);
      joint_cmd.control_modes.push_back(0);
    }

    // 将腰部关节命令从index 0移动到index 12
    joint_cmd.joint_q[0] = -joint_cmd.joint_q[0];
    joint_cmd.joint_v[0] = -joint_cmd.joint_v[0];
    joint_cmd.tau[0] = -joint_cmd.tau[0];

    moveStdVectorEntry(joint_cmd.joint_q, 0, 12);
    moveStdVectorEntry(joint_cmd.joint_v, 0, 12);
    moveStdVectorEntry(joint_cmd.joint_kp, 0, 12);
    moveStdVectorEntry(joint_cmd.joint_kd, 0, 12);
    moveStdVectorEntry(joint_cmd.tau, 0, 12);
    moveStdVectorEntry(joint_cmd.tau_ratio, 0, 12);
    moveStdVectorEntry(joint_cmd.tau_max, 0, 12);
    moveStdVectorEntry(joint_cmd.control_modes, 0, 12);

  }



  bool FallStandController::shouldRunInference() const
  {
    // 重写基类方法，添加fall_stand_state_的检查
    return RLControllerBase::shouldRunInference() && 
           fall_stand_state_ == FallStandState::STAND_UP;
  }

  

  void FallStandController::updateObservation(const Eigen::VectorXd& state_est, const SensorData& sensor_data)
  {
    // 在STAND_UP状态下更新轨迹时间步（与humanoidController一致）
    if (fall_stand_state_ == FallStandState::STAND_UP)
    {
      motion_trajectory_.updateTimeStep();
      if (motion_trajectory_.isFinish())
      {
        ROS_INFO("[%s] Motion trajectory finished", name_.c_str());
        fall_stand_state_ = FallStandState::STANDING;
      }
    }
    
    // 提取状态数据（与humanoidController一致）
    const Eigen::Vector3d baseEuler(state_est(2), state_est(1), state_est(0));
    const Eigen::Vector3d baseAngVel(state_est(6 + jointNum_ + waistNum_ + jointArmNum_),
                                      state_est(6 + jointNum_ + waistNum_ + jointArmNum_ + 1),
                                      state_est(6 + jointNum_ + waistNum_ + jointArmNum_ + 2));
    const Eigen::Vector3d baseLineVel = state_est.segment(9 + jointNum_ + waistNum_ + jointArmNum_, 3);
    
    // 提取和处理传感器数据
    Eigen::VectorXd jointPos = sensor_data.jointPos_;
    Eigen::VectorXd jointVel = sensor_data.jointVel_;
    Eigen::VectorXd jointTorque = sensor_data.jointCurrent_;
    jointPos = jointPos - defalutJointPosRL_;

    const Eigen::Vector3d bodyAngVel = sensor_data.angularVel_;
    const Eigen::Vector3d bodyLineAcc = sensor_data.linearAccel_;
    const Eigen::Vector3d bodyLineFreeAcc = sensor_data.freeLinearAccel_;
    
    // 归一化关节扭矩
    Eigen::VectorXd torqueLimitsLocal = torqueLimitsRL_;
    for (int i = 0; i < jointNum_ + waistNum_ + jointArmNum_; ++i)
    {
      jointTorque[i] /= torqueLimitsLocal[i];
    }
    
    // 变换基座线速度
    auto quat_offset_ = Eigen::AngleAxisd(-my_yaw_offset_, Eigen::Vector3d::UnitZ()) * sensor_data.quat_;
    const Eigen::Matrix3d R = quat_offset_.matrix();
    const Eigen::Vector3d bodyLineVel = R.transpose() * baseLineVel;
    
    Eigen::Quaterniond currentBaseQuat = Eigen::Quaterniond(quat_offset_.w(),
                                                           quat_offset_.x(),
                                                           quat_offset_.y(),
                                                           quat_offset_.z());
    Eigen::VectorXd trajectoryCommand = getTrajectoryCommand();
    Eigen::VectorXd motionAnchorOriB = getMotionAnchorOriB(currentBaseQuat);
    const Eigen::Vector3d gravity_world(0, 0, -1);
    const Eigen::Vector3d projected_gravity = R.transpose() * gravity_world;
    
    // 获取本地动作（使用基类的线程安全方法）
    Eigen::VectorXd local_action = getCurrentAction();
    
    // 构建命令数据（倒地起身时命令为零）
    Eigen::VectorXd tempCommand_(4);
    tempCommand_ << 0.0, 0.0, 0.0, 1.0; // cmdVelLineX, cmdVelLineY, cmdVelAngularZ, cmdStance
    
    // 发布tempCommand数据
    if (ros_logger_)
    {
      ros_logger_->publishVector("/rl_controller/tempCommand", tempCommand_);
    }

    Eigen::Vector3d rawTrajectoryPos = motion_trajectory_.getTargetAnchorPos();
    Eigen::Quaterniond rawTrajectoryQuat = motion_trajectory_.getTargetAnchorQuat();
    Eigen::VectorXd motion_root_ori(4);
    motion_root_ori << rawTrajectoryQuat.w(),
                      rawTrajectoryQuat.x(),
                      rawTrajectoryQuat.y(),
                      rawTrajectoryQuat.z();
    Eigen::VectorXd motion_target_pos(1);
    motion_target_pos << rawTrajectoryPos[2];
    
    // 构建观测数据映射（与humanoidController一致）
    const std::map<std::string, Eigen::VectorXd> singleInputDataMap_ = {
        {"motion_command", trajectoryCommand},
        {"motion_target_pos", motion_target_pos},
        {"motion_anchor_ori_b", motionAnchorOriB},
        {"projected_gravity", projected_gravity},
        {"base_ang_vel", bodyAngVel},
        {"joint_pos", jointPos},
        {"joint_vel", jointVel},
        {"actions", local_action}};

    // Fill singleInputData（与humanoidController一致）
    if (!singleInputDataRLKeys.empty())
    {
      // 如果配置了singleInputData，使用配置的方式填充（与humanoidController一致）
      int index = 0;
      for (const auto &key : singleInputDataRLKeys)
      {
        const auto &value = singleInputDataRLID_[key];
        singleInputDataRL_.segment(index, value[1]) = singleInputDataMap_.at(key).segment(value[0], value[1]) * value[2];
        index += value[1];
        if (ros_logger_)
        {
          ros_logger_->publishVector("/rl_controller/InputData/" + key, singleInputDataMap_.at(key).segment(value[0], value[1]) * value[2]);
        }
      }
    }
    else
    {
      ROS_ERROR_THROTTLE(1.0, "[%s] singleInputDataRLKeys is empty, cannot build observation", name_.c_str());
      singleInputDataRL_.setZero();
    }
    
    // Clip and update input_deque（与humanoidController一致）
    // clip(singleInputDataRL_, clipObservationsRL_);
    input_deque.push_back(singleInputDataRL_);
    input_deque.pop_front();
    
    // Update networkInputData_（与humanoidController一致）
    for (int i = 0; i < frameStackRL_; ++i)
    {
      networkInputDataRL_.segment(i * numSingleObsRL_, numSingleObsRL_) = input_deque[i];
    }
    
    // 发布观测数据（如果ros_logger_可用）
    if (ros_logger_)
    {
      ros_logger_->publishVector("/rl_controller/singleInputData", singleInputDataRL_);
    }
  }

  void FallStandController::updateFallStandInterpolation(const ros::Time& time, 
                                                          const SensorData& sensor_data,
                                                          const Eigen::VectorXd& measuredRbdState,
                                                          kuavo_msgs::jointCmd& joint_cmd)
  {
    int total_body_joints = jointNum_ + jointArmNum_ + waistNum_;
    int total_joints = total_body_joints + headNum_;

    // 如果还没有开始插值，启动插值
    if (!is_fall_stand_interpolating_)
    {
      startFallStandInterpolation(time, sensor_data);
      if (!is_fall_stand_interpolating_)
      {
        // 无法启动插值，返回零指令
        joint_cmd.header.stamp = time;
        joint_cmd.joint_q.assign(total_joints, 0.0);
        joint_cmd.joint_v.assign(total_joints, 0.0);
        joint_cmd.tau.assign(total_joints, 0.0);
        joint_cmd.tau_ratio.assign(total_joints, 1.0);
        joint_cmd.tau_max.assign(total_joints, 0.0);
        joint_cmd.joint_kp.assign(total_joints, 0.0);
        joint_cmd.joint_kd.assign(total_joints, 0.0);
        joint_cmd.control_modes.assign(total_joints, 0);
        ROS_WARN("[%s] Failed to start fall stand interpolation", name_.c_str());
        return;
      }
    }

    double elapsed = time.toSec() - fall_stand_interp_start_time_;
    if (elapsed < 0.0)
      elapsed = 0.0;

    double duration = fall_stand_required_time_;
    if (duration <= 0.0)
    {
      duration = 0.0;
    }

    double alpha = 1.0;
    if (duration > 0.0)
    {
      alpha = elapsed / duration;
      if (alpha > 1.0)
        alpha = 1.0;
    }

    double s = alpha; // 线性插值

    joint_cmd.header.stamp = time;
    joint_cmd.joint_q.resize(total_joints);
    joint_cmd.joint_v.assign(total_joints, 0.0);
    joint_cmd.tau.assign(total_joints, 0.0);
    joint_cmd.tau_ratio.assign(total_joints, 1.0);
    joint_cmd.tau_max.resize(total_joints);
    joint_cmd.joint_kp.resize(total_joints);
    joint_cmd.joint_kd.resize(total_joints);
    joint_cmd.control_modes.assign(total_joints, 2); // 位置控制模式

    for (int i = 0; i < total_body_joints; ++i)
    {
      double start_pos = (i < fall_stand_start_pos_.size()) ? fall_stand_start_pos_[i] : 0.0;
      double goal_pos = (i < fall_stand_init_joints_.size()) ? fall_stand_init_joints_[i] : start_pos;
      double pos = start_pos + (goal_pos - start_pos) * s;

      joint_cmd.joint_q[i] = pos;
      joint_cmd.joint_kp[i] = 0.0;  // 默认PD参数
      joint_cmd.joint_kd[i] = 0.0;
      joint_cmd.tau_max[i] = 10.0;
    }

    // 头部关节：保持期望头部位置
    for (int i = 0; i < headNum_; ++i)
    {
      int idx = total_body_joints + i;
      if (idx >= total_joints)
        break;
      joint_cmd.joint_q[idx] = 0.0;

      joint_cmd.joint_kp[idx] = 0.0;
      joint_cmd.joint_kd[idx] = 0.0;
      joint_cmd.tau_max[idx] = 10.0;
      joint_cmd.control_modes[idx] = 2;
    }

    // 在仿真中根据实际关节位置/速度反馈计算期望扭矩（简单PD）
    if (!is_real_)
    {
      auto actuatedDofNum = jointNum_ + jointArmNum_ + waistNum_;
      if (measuredRbdState.size() >= 6 + actuatedDofNum)
      {
        Eigen::VectorXd current_jointPos = measuredRbdState.segment(6, actuatedDofNum);
        Eigen::VectorXd current_jointVel = measuredRbdState.segment(12 + jointNum_ + jointArmNum_ + waistNum_, actuatedDofNum);
        for (int i = 0; i < total_body_joints && i < current_jointPos.size() && i < current_jointVel.size(); ++i)
        {
          double q_meas = current_jointPos[i];
          double v_meas = current_jointVel[i];
          double q_des  = joint_cmd.joint_q[i];
          double v_des  = joint_cmd.joint_v[i]; // 目前为0

          double kp = 50;
          double kd = 10;

          joint_cmd.tau[i] = kp * (q_des - q_meas) + kd * (v_des - v_meas);
        }
      }
    }

    ros_logger_->publishValue("/humanoid_controller/FallStandController/fall_stand_interpolation_alpha_", alpha);
    if (alpha >= 1.0)
    {
      is_fall_stand_interpolating_complete_ = true;
    }
  }

  void FallStandController::startFallStandInterpolation(const ros::Time& time, const SensorData& sensor_data)
  {
    int total_body_joints = jointNum_ + jointArmNum_ + waistNum_;
    
    if (fall_stand_init_joints_.size() < total_body_joints)
    {
      ROS_WARN("[%s] fall_stand_init_joints_ size(%ld) < total_body_joints(%d), skip interpolation.",
               name_.c_str(), fall_stand_init_joints_.size(), total_body_joints);
      is_fall_stand_interpolating_ = false;
      is_fall_stand_interpolating_complete_ = false;
      return;
    }

    if (sensor_data.jointPos_.size() < total_body_joints)
    {
      ROS_WARN("[%s] sensor_data.jointPos_ size(%ld) < total_body_joints(%d), skip interpolation.",
               name_.c_str(), sensor_data.jointPos_.size(), total_body_joints);
      is_fall_stand_interpolating_ = false;
      is_fall_stand_interpolating_complete_ = false;
      return;
    }

    fall_stand_start_pos_ = sensor_data.jointPos_.head(total_body_joints);
    moveVectorEntry(fall_stand_start_pos_, 0, 12);
    fall_stand_start_pos_[12] = -fall_stand_start_pos_[12];

    // 计算插值所需时间
    Eigen::VectorXd delta = fall_stand_init_joints_.head(total_body_joints) - fall_stand_start_pos_;
    double max_delta = 0.0;
    if (delta.size() > 0)
      max_delta = delta.cwiseAbs().maxCoeff();

    if (fall_stand_max_joint_velocity_ > 0.0 && max_delta > 1e-6)
    {
      fall_stand_required_time_ = max_delta / fall_stand_max_joint_velocity_;
    }
    else
    {
      fall_stand_required_time_ = 0.0;
    }

    fall_stand_interp_start_time_ = time.toSec();
    is_fall_stand_interpolating_ = true;
    is_fall_stand_interpolating_complete_ = false;
    
    std::cout << "[FallStandInterpolation] start, required_time: " << fall_stand_required_time_
              << " s, joints: " << total_body_joints << " fall_stand_start_pos_: " << fall_stand_start_pos_.transpose() 
              << " fall_stand_init_joints_: " << fall_stand_init_joints_.transpose() << std::endl;
  }

  bool FallStandController::loadTrajectory(const std::string& trajectory_file)
  {
    return loadMotionTrajectory(trajectory_file, motion_trajectory_);
  }

  /*****************************************倒地起身*****************************************************************/
  bool FallStandController::loadMotionTrajectory(const std::string &trajectoryFile, MotionTrajectoryData& trajectory) {
    std::ifstream file(trajectoryFile);
    if (!file.is_open()) {
      std::cerr << "Failed to open trajectory file: " << trajectoryFile
                << std::endl;
      return false;
    }

    std::vector<std::vector<double>> csvData;
    std::string line;
    bool firstLine = true;
    int lineNum = 0;
    const int expectedCols = 3 + 4 + (jointNum_ + jointArmNum_ + waistNum_) * 2;

    while (std::getline(file, line)) {
      lineNum++;
      if (line.empty())
        continue;

      std::vector<double> row;
      std::stringstream ss(line);
      std::string value;
      bool parseError = false;

      while (std::getline(ss, value, '\t')) {
        if (value.empty())
          continue;
        try {
          row.push_back(std::stod(value));
        } catch (const std::exception &e) {
          // 解析失败，可能是表头
          parseError = true;
          break;
        }
      }
      // 如果是第一行且解析失败，认为是表头，跳过
      if (firstLine && parseError) {
        std::cout << "Detected header line, skipping: "
                  << line.substr(0, std::min(80, (int)line.size())) << "..."
                  << std::endl;
        firstLine = false;
        continue;
      }
      firstLine = false;

      if (row.size() == expectedCols) {
        csvData.push_back(row);
      } else if (row.size() > 0 && !parseError) {
        std::cerr << "Warning: Line " << lineNum << " has " << row.size()
                  << " columns, expected " << expectedCols << " (49)"
                  << std::endl;
      }
    }
    file.close();

    int timeSteps = csvData.size();
    if (timeSteps == 0) {
      std::cerr << "No valid data found in CSV file" << std::endl;
      return false;
    }

    const int numJoints = jointNum_ + jointArmNum_ + waistNum_;
    trajectory.time_step_total = timeSteps;
    trajectory.current_time_step = 0;

    trajectory.body_pos_w.resize(timeSteps, 3);
    trajectory.body_quat_w.resize(timeSteps, 4);
    trajectory.joint_pos.resize(timeSteps, numJoints);
    trajectory.joint_vel.resize(timeSteps, numJoints);

    for (int i = 0; i < timeSteps; ++i) {
      const auto &row = csvData[i];

      trajectory.body_pos_w(i, 0) = row[0];
      trajectory.body_pos_w(i, 1) = row[1];
      trajectory.body_pos_w(i, 2) = row[2];

      trajectory.body_quat_w(i, 0) = row[3]; // w
      trajectory.body_quat_w(i, 1) = row[4]; // x
      trajectory.body_quat_w(i, 2) = row[5]; // y
      trajectory.body_quat_w(i, 3) = row[6]; // z

      for (int j = 0; j < numJoints; ++j) {
        trajectory.joint_pos(i, j) = row[7 + j];
      }
      for (int j = 0; j < numJoints; ++j) {
        trajectory.joint_vel(i, j) = row[7 + numJoints + j];
      }
    }
    if (timeSteps > 0) {
      Eigen::Quaterniond first_quat(trajectory.body_quat_w(0, 0), // w
                                    trajectory.body_quat_w(0, 1), // x
                                    trajectory.body_quat_w(0, 2), // y
                                    trajectory.body_quat_w(0, 3)  // z
      );

      Eigen::Matrix3d ref_mat = first_quat.toRotationMatrix();
      trajectory.reference_yaw =
          std::atan2(ref_mat(1, 2), ref_mat(0, 2));
    }
    return true;
  }

  Eigen::VectorXd FallStandController::getTrajectoryCommand() {
    if (!trajectory_loaded_) {
      ROS_ERROR("[%s] trajectory_loaded_ is false, return zero command", name_.c_str());
      return Eigen::VectorXd::Zero((jointNum_ + jointArmNum_ + waistNum_) * 2);
    }
    return getTrajectoryCommand(motion_trajectory_);
  }
  
  Eigen::VectorXd FallStandController::getTrajectoryCommand(const MotionTrajectoryData& trajectory) {

    return trajectory.getCurrentCommand();
  }
  
  Eigen::Vector3d FallStandController::getTrajectoryAnchorPos() {
    return motion_trajectory_.getTargetAnchorPos();
  }
  Eigen::Quaterniond FallStandController::getTrajectoryAnchorQuat() {
    return motion_trajectory_.getTargetAnchorQuat();
  }

  // Get motion anchor position difference in body frame
  Eigen::Vector3d FallStandController::getMotionAnchorPosB(
      const Eigen::Vector3d &currentBasePos,
      const Eigen::Quaterniond &currentBaseQuat) {
    if (!trajectory_loaded_) {
      return Eigen::Vector3d::Zero();
    }

    // Get target anchor position and orientation from trajectory
    Eigen::Vector3d targetAnchorPos = motion_trajectory_.getTargetAnchorPos();

    // Calculate position difference in body frame using
    // subtract_frame_transforms logic T_12 = T_01^(-1) * T_02, where T_01 is
    // current robot pose, T_02 is target pose
    Eigen::Quaterniond currentQuatInv = currentBaseQuat.inverse();
    Eigen::Vector3d positionDiff =
        currentQuatInv * (targetAnchorPos - currentBasePos);
    return positionDiff;
  }

  // Get motion anchor orientation difference in body frame
  Eigen::VectorXd FallStandController::getMotionAnchorOriB(
      const Eigen::Quaterniond &currentBaseQuat) {
    if (!trajectory_loaded_) {
      return Eigen::VectorXd::Zero(6);
    }

    // Get target anchor orientation from trajectory
    Eigen::Quaterniond targetAnchorQuat =
        motion_trajectory_.getTargetAnchorQuat();
    // q12 = q10 * q02, where q10 = q01^(-1)
    Eigen::Quaterniond currentQuatInv = currentBaseQuat.inverse();
    Eigen::Quaterniond quatDiff = currentQuatInv * targetAnchorQuat;
    Eigen::Matrix3d rotMat = quatDiff.toRotationMatrix();
    // Extract first two columns and reshape row-wise (mat[...,
    // :2].reshape(mat.shape[0], -1))
    Eigen::VectorXd oriDiff(6);
    oriDiff << rotMat(0, 0), rotMat(0, 1), rotMat(1, 0), rotMat(1, 1),
        rotMat(2, 0), rotMat(2, 1);
    return oriDiff;
  }

  /*****************************************倒地起身*****************************************************************/

  bool FallStandController::switchModel(FallStandModelType model_type)
  {
    if (model_type == current_model_type_)
    {
      ROS_INFO("[%s] Model type already set to %d", name_.c_str(), static_cast<int>(model_type));
      return true;
    }
    
    // 检查对应的模型和轨迹是否已加载
    if (model_type == FallStandModelType::PRONE)
    {
      if (!model_loaded_prone_)
      {
        ROS_ERROR("[%s] Prone model not loaded, cannot switch", name_.c_str());
        return false;
      }
      compiled_model_ = compiled_model_prone_;
      infer_request_ = compiled_model_.create_infer_request();
      motion_trajectory_ = motion_trajectory_prone_;
      trajectory_loaded_ = trajectory_loaded_prone_;
      fall_stand_init_joints_ = fall_stand_init_joints_prone_;
      current_model_type_ = FallStandModelType::PRONE;
      ROS_INFO("[%s] Switched to PRONE model", name_.c_str());
    }
    else // SUPINE
    {
      if (!model_loaded_supine_)
      {
        ROS_ERROR("[%s] Supine model not loaded, cannot switch", name_.c_str());
        return false;
      }
      compiled_model_ = compiled_model_supine_;
      infer_request_ = compiled_model_.create_infer_request();
      motion_trajectory_ = motion_trajectory_supine_;
      trajectory_loaded_ = trajectory_loaded_supine_;
      fall_stand_init_joints_ = fall_stand_init_joints_supine_;
      current_model_type_ = FallStandModelType::SUPINE;
      ROS_INFO("[%s] Switched to SUPINE model", name_.c_str());
    }
    
    return true;
  }

  bool FallStandController::autoSelectAndSwitchModel()
  {
    SensorData sensor_data = getRobotSensorData();
    Eigen::Matrix3d mat = sensor_data.quat_.toRotationMatrix();
    // 机体 x 轴（身体前方方向）
    Eigen::Vector3d body_x = mat.col(0);
    // 世界坐标系重力方向 (0, 0, -1)
    Eigen::Vector3d gravity_world(0.0, 0.0, -1.0);
    double cos_angle = body_x.dot(gravity_world);

    // 判断逻辑：
    // - 趴着：x轴指向地面（与重力方向同向），cos_angle > 0
    // - 仰着躺：x轴指向天花板（与重力方向反向），cos_angle < 0
    FallStandModelType desired_model =
        (cos_angle > 0.0) ? FallStandModelType::PRONE : FallStandModelType::SUPINE;

    if (!switchModel(desired_model))
    {
      ROS_WARN("[%s] Failed to switch fall-stand model automatically (desired=%d), keep current model.",
               name_.c_str(), static_cast<int>(desired_model));
      return false;
    }
    else
    {
      ROS_INFO("[%s] Automatically switched fall-stand model to %s based on current posture (body_x·gravity=%.3f).",
               name_.c_str(),
               (desired_model == FallStandModelType::PRONE ? "PRONE" : "SUPINE"),
               cos_angle);
      return true;
    }
  }

  bool FallStandController::triggerFallStandUpCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
  {
    if (state_ != ControllerState::RUNNING)
    {
      res.success = false;
      res.message = "FallStandController is not running, start or resume before trigger fall stand up";
      return true;
    }
    // 检查当前是否处于倒地状态
    if (fall_stand_state_ == FallStandState::FALL_DOWN)
    {
      // ------------------------------------------------------------------
      // 1. 根据当前机体姿态自动选择趴着/躺着模型
      // ------------------------------------------------------------------
      autoSelectAndSwitchModel();



      // 触发倒地启动过程
      motion_trajectory_.resetTimeStep();
      SensorData sensor_data = getRobotSensorData();
      auto mat = sensor_data.quat_.toRotationMatrix();
      // 计算yaw偏移
      double current_yaw = std::atan2(mat(1, 2), mat(0, 2)); 
      my_yaw_offset_ = current_yaw - motion_trajectory_.reference_yaw;
      std::cout << "getRobotSensorData().quat_: [" << sensor_data.quat_.w() << ", " 
                << sensor_data.quat_.x() << ", " << sensor_data.quat_.y() << ", " << sensor_data.quat_.z() << "]" << std::endl;
      std::cout << "Current yaw: " << current_yaw << std::endl;
      std::cout << "Reference yaw: " << motion_trajectory_.reference_yaw << std::endl;
      // 归一化到[-π, π]范围
      while (my_yaw_offset_ > M_PI) my_yaw_offset_ -= 2 * M_PI;
      while (my_yaw_offset_ < -M_PI) my_yaw_offset_ += 2 * M_PI;

      fall_stand_state_ = FallStandState::READY_FOR_STAND_UP;
      res.success = true;
      res.message = "Fall stand up process triggered successfully";
      ROS_INFO("[%s] Fall stand up process triggered", name_.c_str());
    }
    else if (fall_stand_state_ == FallStandState::READY_FOR_STAND_UP)
    {
      // 请求起身，当准备好时(插值完)在update中触发
      request_for_stand_up_ = true;
      
      res.success = false;
      res.message = "Stand up process is already in progress";
      ROS_WARN("[%s] Ready for stand up process is already in progress", name_.c_str());
    }
    else if (fall_stand_state_ == FallStandState::STAND_UP)
    {
      reset();
      res.success = false;
      res.message = "Stand up process is already in progress, stop stand up process";
      ROS_WARN("[%s] Stand up process is already in progress, stop stand up process", name_.c_str());
    }
    else
    {
      res.success = false;
      res.message = "Robot is not in fall down state, current state: " + std::to_string(static_cast<int>(fall_stand_state_));
      ROS_WARN("[%s] Robot is not in fall down state, current state: %d", name_.c_str(), static_cast<int>(fall_stand_state_));
    }
    return true;
  }


} // namespace humanoid_controller







