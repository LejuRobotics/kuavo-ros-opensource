#include "humanoid_controllers/rl/AmpWalkController.h"
#include <ocs2_core/misc/LoadData.h>
#include <ocs2_robotic_tools/common/RotationTransforms.h>
#include <angles/angles.h>
#include "kuavo_common/common/common.h"
#include <cmath>

namespace humanoid_controller
{
  using namespace ocs2;
  using namespace ocs2::humanoid;

  AmpWalkController::AmpWalkController(const std::string& name,
                                       const std::string& config_file,
                                       ros::NodeHandle& nh,
                                       TopicLogger* ros_logger)
    : RLControllerBase(name, RLControllerType::AMP_CONTROLLER, config_file, nh, ros_logger)
  {
    // 构造函数里 RLControllerBase 已经调用 initializeServices() 和 initializeRLVariables()
  }

  bool AmpWalkController::initialize()
  {
    if (!loadConfig(config_file_))
    {
      ROS_ERROR("[%s] loadConfig failed", name_.c_str());
      return false;
    }

    // 初始化 RL IMU 滤波（内部用 accFilterRL_ 等）
    loadRLFilterParams(config_file_);

    // 从 ROS 参数获取控制周期 dt_
    double wbc_frequency = 500.0;
    if (!nh_.getParam("/wbc_frequency", wbc_frequency))
    {
      ROS_WARN("[%s] /wbc_frequency not found in ROS params, using default: %.1f Hz", name_.c_str(), wbc_frequency);
    }
    if (wbc_frequency <= 0.0)
    {
      ROS_WARN("[%s] Invalid /wbc_frequency (%.3f), fallback to 500 Hz", name_.c_str(), wbc_frequency);
      wbc_frequency = 500.0;
    }
    dt_ = 1.0 / wbc_frequency;

    // gait 指令来源：使用 RL gait receiver，等价于原来的 CommandData + joystick/cmd_vel
    initial_cmd_.cmdStance_ = 1;
    gait_receiver_ = std::make_unique<RlGaitReceiver>(nh_, &initial_cmd_);

    // 读取实物/机型参数（与 FallStandController / humanoidController 保持一致）
    if (!nh_.getParam("/is_real", is_real_))
    {
      ROS_WARN("[%s] /is_real not found in ROS params, using default: %d", name_.c_str(), static_cast<int>(is_real_));
    }
    if (!nh_.getParam("/is_roban", is_roban_))
    {
      ROS_WARN("[%s] /is_roban not found in ROS params, using default: %d", name_.c_str(), static_cast<int>(is_roban_));
    }

    // 初始化ankleSolver（从ROS参数获取，如果不存在则使用默认值）
    int ankle_solver_type = 0; // 默认值
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

    initialized_ = true;

    ROS_INFO("[%s] AmpWalkController initialized", name_.c_str());
    return true;
  }

  bool AmpWalkController::loadConfig(const std::string& rlParamFile)
  {
    bool verbose = false;
    boost::property_tree::ptree pt;
    boost::property_tree::read_info(rlParamFile, pt);

    // 下面这段基本是从 humanoidController_rl.cpp::loadSettings 拷贝过来，
    // 只是变量名加了 RL 后缀，对应 RLControllerBase 中的 *_RL_ 成员。

    auto loadEigenMatrix = [&](const std::string &key, auto &matrix)
    {
      loadData::loadEigenMatrix(rlParamFile, key, matrix);
    };

    loadEigenMatrix("defaultJointState", defalutJointPosRL_);
    loadEigenMatrix("defaultBaseState", defaultBaseStateRL_);
    loadEigenMatrix("JointControlMode", JointControlModeRL_);
    loadEigenMatrix("JointPDMode", JointPDModeRL_);
    loadEigenMatrix("jointKp", jointKpRL_);
    loadEigenMatrix("jointKd", jointKdRL_);
    loadEigenMatrix("torqueLimits", torqueLimitsRL_);
    loadEigenMatrix("actionScaleTest", actionScaleTestRL_);
    loadEigenMatrix("velocityLimits", velocityLimits_); // 4 维
    
    // 设置 initialStateRL_ = [defaultBaseStateRL_(12) + defalutJointPosRL_]
    initialStateRL_.resize(12 + defalutJointPosRL_.size());
    initialStateRL_ << defaultBaseStateRL_, defalutJointPosRL_;

    loadData::loadCppDataType(rlParamFile, "actionScale", actionScale_);
    loadData::loadCppDataType(rlParamFile, "frameStack", frameStack_);
    loadData::loadCppDataType(rlParamFile, "numSingleObs", numSingleObs_);
    loadData::loadCppDataType(rlParamFile, "cycleTime", cycleTime_);
    loadData::loadCppDataType(rlParamFile, "cycleTime_short", cycleTime_short_);
    loadData::loadCppDataType(rlParamFile, "switch_ratio", switch_ratio_);
    loadData::loadCppDataType(rlParamFile, "phase", phase_);
    loadData::loadCppDataType(rlParamFile, "episodeLength", episodeLength_);
    loadData::loadCppDataType(rlParamFile, "clipActions", clipActions_);
    loadData::loadCppDataType(rlParamFile, "withArm", withArm_);
    loadData::loadCppDataType(rlParamFile, "inferenceFrequency", inference_frequency_);
    loadData::loadCppDataType(rlParamFile, "defaultBaseHeightControl", defaultBaseHeightControl_);

    std::string networkModelFile;
    loadData::loadCppDataType(rlParamFile, "networkModelFile", networkModelFile);
    // RLControllerBase 的 actions_ 是在 inferenceThreadFunc 里用的，这里只配置 compiled_model_
    nh_.getParam("/network_model_file", networkModelPath_); // 或者直接在 rlParamFile 里拼路径
    networkModelPath_ = networkModelPath_ + networkModelFile;

    // singleInputData 部分：key -> [startIdx, numIdx, obsScale]
    if (verbose)
    {
      std::cerr << "\n #### singleInputData:";
      std::cerr << "\n #### =============================================================================\n";
    }

    numSingleObs_ = 0;
    for (const auto &pair : pt)
    {
      if (pair.first == "singleInputData")
      {
        for (const auto &pair2 : pair.second)
        {
          singleInputDataKeys_.push_back(pair2.first);
          double startIdx = 0, numIdx = 0, obsScale = 0;
          loadData::loadPtreeValue(pt, startIdx, "singleInputData." + pair2.first + ".startIdx", verbose);
          loadData::loadPtreeValue(pt, numIdx, "singleInputData." + pair2.first + ".numIdx", verbose);
          loadData::loadPtreeValue(pt, obsScale, "singleInputData." + pair2.first + ".obsScales", verbose);
          numSingleObs_ += static_cast<int>(numIdx);
          singleInputDataID_[pair2.first] = Eigen::Vector3d(startIdx, numIdx, obsScale);
        }
      }
    }
    

    if (numSingleObs_ <= 0)
    {
      ROS_ERROR("[%s] numSingleObs_ invalid", name_.c_str());
      return false;
    }

    // 分配观测缓存
    singleInputData_.resize(numSingleObs_);
    singleInputData_.setZero();
    networkInputDataRL_.resize(numSingleObs_ * frameStack_);
    networkInputDataRL_.setZero();
    for (int i = 0; i < frameStack_; i++)
    {
      inputDeque_.push_back(singleInputData_);
    }

    // OpenVINO 模型
    compiled_model_ = core_.compile_model(networkModelPath_, "CPU");

    // 动作维度
    num_actions_ = jointNum_ + jointArmNum_ + waistNum_;
    setCurrentAction(Eigen::VectorXd::Zero(num_actions_));

    const std::string prefixCommandData_ = "commandData";
    const std::vector<std::pair<std::string, double CommandDataRL::*>> cmdInitalList = {
        {"cmdVelLineX", &CommandDataRL::cmdVelLineX_},
        {"cmdVelLineY", &CommandDataRL::cmdVelLineY_},
        {"cmdVelLineZ", &CommandDataRL::cmdVelLineZ_},
        {"cmdVelAngularX", &CommandDataRL::cmdVelAngularX_},
        {"cmdVelAngularY", &CommandDataRL::cmdVelAngularY_},
        {"cmdVelAngularZ", &CommandDataRL::cmdVelAngularZ_},
        {"cmdStance", &CommandDataRL::cmdStance_},
    };
    const std::vector<std::pair<std::string, double CommandDataRL::*>> cmdScaleList = {
        {"cmdVelLineX", &CommandDataRL::cmdVelScaleLineX_},
        {"cmdVelLineY", &CommandDataRL::cmdVelScaleLineY_},
        {"cmdVelLineZ", &CommandDataRL::cmdVelScaleLineZ_},
        {"cmdVelAngularX", &CommandDataRL::cmdVelScaleAngularX_},
        {"cmdVelAngularY", &CommandDataRL::cmdVelScaleAngularY_},
        {"cmdVelAngularZ", &CommandDataRL::cmdVelScaleAngularZ_},
        {"cmdStance", &CommandDataRL::cmdScaleStance_}};
    for (const auto &[cmdName, cmdMember] : cmdInitalList)
    {
      loadData::loadPtreeValue(pt, initial_cmd_.*cmdMember, prefixCommandData_ + ".inital." + cmdName, false);
    }
    for (const auto &[cmdName, cmdMember] : cmdScaleList)
    {
      loadData::loadPtreeValue(pt, initial_cmd_.*cmdMember, prefixCommandData_ + ".scale." + cmdName, false);
    }


    ROS_INFO("[%s] loadConfig done. num_actions_=%d, numSingleObs_=%d, frameStack_=%d",
             name_.c_str(), num_actions_, numSingleObs_, frameStack_);
    return true;
  }

  void AmpWalkController::reset()
  {
    phase_ = 0.0;
    episodeLength_ = 0;
    currentCycleTime_ = cycleTime_;
    actions_.setZero();
    // networkInputDataRL_ 和 singleInputData_ 清零
    networkInputDataRL_.setZero();
    singleInputData_.setZero();
    ROS_INFO("[%s] reset", name_.c_str());
    sensor_data_updated_ = false;
  }
  void AmpWalkController::pause()
  {
    RLControllerBase::pause();
    if (gait_receiver_)
    {
      gait_receiver_->setEnabled(false);
    }
  }
  void AmpWalkController::resume()
  {
    RLControllerBase::resume();
    if (gait_receiver_)
    {
      gait_receiver_->setEnabled(true);
    }
    ROS_INFO("[%s] Controller resumed, reset state", name_.c_str());
    reset();
  }

  bool AmpWalkController::isReadyToExit() const
  {
    if (!sensor_data_updated_)
    {
      return false;
    }
    // 获取当前机器人状态（线程安全）
    Eigen::VectorXd state = getRobotState();
    
    // 检查状态是否有效（至少需要12维：位置3 + 姿态3 + 速度3 + 角速度3）
    if (state.size() < 12)
    {
      // 状态数据不足，无法判断，返回false
      return false;
    }
    
    // 从状态中提取姿态角（欧拉角）
    // state格式: [yaw(0), pitch(1), roll(2), ...]
    double roll = state(2);   // angular_x
    double pitch = state(1);  // angular_y
    
    // 转换为度数以便判断
    const double roll_deg = std::abs(roll) * 180.0 / M_PI;
    const double pitch_deg = std::abs(pitch) * 180.0 / M_PI;
    
    // 如果 roll 或 pitch 的绝对值大于60度，判断为倒地
    const double fall_threshold_deg = 60.0;
    bool is_fallen = (roll_deg > fall_threshold_deg) || (pitch_deg > fall_threshold_deg);
    
    if (is_fallen)
    {
      ROS_WARN_THROTTLE(1.0, "[%s] Detected fall: roll=%.2f deg, pitch=%.2f deg, requesting exit", 
                       name_.c_str(), roll_deg, pitch_deg);
    }
    
    return is_fallen;
  }

  bool AmpWalkController::shouldRunInference() const
  {
    if (state_ != ControllerState::RUNNING)
      return false;

    if (!gait_receiver_)
      return false;
    return RLControllerBase::shouldRunInference();
  }

  void AmpWalkController::updatePhase(const CommandDataRL& cmd)
  {
    // 基本照 humanoidController_rl.cpp::updatePhase
    double ratio = cmd.cmdVelLineX_ / velocityLimits_(0);
    double targetCycleTime = (ratio > switch_ratio_) ? cycleTime_short_ : cycleTime_;
    if (targetCycleTime != currentCycleTime_)
    {
      double temp_phase = episodeLength_ * dt_ / currentCycleTime_;
      episodeLength_ = int((targetCycleTime / dt_) * (temp_phase - int(temp_phase)));
    }
    double alpha = 1.0;
    currentCycleTime_ = (1.0 - alpha) * currentCycleTime_ + alpha * targetCycleTime;

    phase_ = cmd.cmdStance_ == 1.0 ? 0.0 : episodeLength_ * dt_ / currentCycleTime_;

    commandPhase_(0) = std::sin(2 * M_PI * phase_);
    commandPhase_(1) = std::cos(2 * M_PI * phase_);
    rl_plannedMode_ = (commandPhase_(0) > 0) ? ModeNumber::SF
                    : (commandPhase_(0) < 0) ? ModeNumber::FS
                                             : ModeNumber::SS;
  }

  void AmpWalkController::updateObservation(const Eigen::VectorXd& state_est,
                                            const SensorData& sensor_data)
  {
    // === 1. 从 gait receiver 获取 CommandDataRL 并更新 phase ===
    CommandDataRL cmd = gait_receiver_->getCurrentCommand();
    
    updatePhase(cmd);
    // 初始化 my_yaw_offset_（仅在第一次调用时，与 humanoidController_rl.cpp 一致）
    static bool yaw_offset_initialized = false;
    if (!yaw_offset_initialized)
    {
      auto mat = sensor_data.quat_.toRotationMatrix();
      double current_yaw = std::atan2(mat(1, 2), mat(0, 2));
      my_yaw_offset_ = 0.0 - current_yaw;
      // 归一化到[-π, π]范围
      while (my_yaw_offset_ > M_PI) my_yaw_offset_ -= 2 * M_PI;
      while (my_yaw_offset_ < -M_PI) my_yaw_offset_ += 2 * M_PI;
      yaw_offset_initialized = true;
      ROS_INFO("[%s] Initialized yaw_offset: %.6f (current_yaw: %.6f)", name_.c_str(), my_yaw_offset_, current_yaw);
    }

    // 速度命令 [vx, vy, omega_z]
    cmd.scale();
    Eigen::Vector3d velocity_commands;
    velocity_commands << cmd.cmdVelLineX_,
                         cmd.cmdVelLineY_,
                         cmd.cmdVelAngularZ_;
    Eigen::VectorXd tempCommand_ = cmd.getCommandRL();


    // === 2. 状态、IMU、关节等数据，与 humanoidController_rl.cpp 一致 ===
    const Eigen::Vector3d baseEuler(state_est(2), state_est(1), state_est(0));
    const Eigen::Vector3d baseAngVel(state_est(6 + waistNum_ + jointNum_ + jointArmNum_),
                                     state_est(6 + waistNum_ + jointNum_ + jointArmNum_ + 1),
                                     state_est(6 + waistNum_ + jointNum_ + jointArmNum_ + 2));
    const Eigen::Vector3d baseLineVel = state_est.segment(9 + waistNum_ + jointNum_ + jointArmNum_, 3);
    const Eigen::Vector3d basePos = state_est.segment(3, 3);

    Eigen::VectorXd jointPos = sensor_data.jointPos_ - defalutJointPosRL_;
    Eigen::VectorXd jointVel = sensor_data.jointVel_;
    Eigen::VectorXd jointTorque = sensor_data.jointCurrent_;
    Eigen::Vector3d bodyAngVel = sensor_data.angularVel_;
    const Eigen::Vector3d &bodyLineAcc = sensor_data.linearAccel_;
    const Eigen::Vector3d &bodyLineFreeAcc = sensor_data.freeLinearAccel_;

    // 归一化 torque
    for (int i = 0; i < jointNum_ + jointArmNum_ + waistNum_; ++i)
      jointTorque[i] /= torqueLimitsRL_[i];

    // yaw 对齐
    auto quat_offset = Eigen::AngleAxisd(-my_yaw_offset_, Eigen::Vector3d::UnitZ()) * sensor_data.quat_;
    const Eigen::Matrix3d R = quat_offset.matrix();
    const Eigen::Vector3d bodyLineVel = R.transpose() * baseLineVel;

    const Eigen::Vector3d gravity_world(0, 0, -1);
    const Eigen::Vector3d projected_gravity = R.transpose() * gravity_world;

    Eigen::VectorXd local_action = getCurrentAction();

    // === 3. 填充 singleInputData / networkInputDataRL_ ===
    std::map<std::string, Eigen::VectorXd> singleInputDataMap = {
        // old name:
        {"base_ang_vel", bodyAngVel},
        {"projected_gravity", projected_gravity},
        {"velocity_commands", velocity_commands},
        {"joint_pos", jointPos},
        {"joint_vel", jointVel},
        {"actions", local_action},

        // new name:
        {"gravity_body", projected_gravity},
        {"baseEuler", baseEuler},
        {"baseAngVel", baseAngVel},
        {"baseLineVel", baseLineVel},
        {"basePos", basePos},
        {"jointPos", jointPos},
        {"jointVel", jointVel},
        {"jointTorque", jointTorque},
        {"bodyAngVel", bodyAngVel},
        {"bodyLineAcc", bodyLineAcc},
        {"bodyLineFreeAcc", bodyLineFreeAcc},
        {"bodyLineVel", bodyLineVel},
        {"commandPhase", commandPhase_},
        {"command", tempCommand_},
        {"action", local_action}
    };

    // Fill singleInputData（与humanoidController一致）
    if (!singleInputDataKeys_.empty())
    {
      // 如果配置了singleInputData，使用配置的方式填充（与humanoidController一致）
      int index = 0;
      for (const auto &key : singleInputDataKeys_)
      {
        const auto &value = singleInputDataID_[key];
        singleInputData_.segment(index, value[1]) = singleInputDataMap.at(key).segment(value[0], value[1]) * value[2];
        index += value[1];
        if (ros_logger_)
        {
          ros_logger_->publishVector("/rl_controller/InputData/" + key, singleInputDataMap.at(key).segment(value[0], value[1]) * value[2]);
        }
      }
    }
    else
    {
      ROS_ERROR_THROTTLE(1.0, "[%s] singleInputDataKeys_ is empty, cannot build observation", name_.c_str());
      singleInputData_.setZero();
    }
    
    // Clip and update inputDeque_（与humanoidController一致）
    inputDeque_.push_back(singleInputData_);
    inputDeque_.pop_front();
    
    // Update networkInputData_（与humanoidController一致）
    for (int i = 0; i < frameStack_; ++i)
    {
      networkInputDataRL_.segment(i * numSingleObs_, numSingleObs_) = inputDeque_[i];
    }
    
    // 发布观测数据（如果ros_logger_可用）
    if (ros_logger_)
    {
      ros_logger_->publishVector("/rl_controller/singleInputData", singleInputData_);
    }
  }

  bool AmpWalkController::inference(const Eigen::VectorXd& observation,
                                    Eigen::VectorXd& action)
  {
    try
    {
      infer_request_ = compiled_model_.create_infer_request();
      const auto input_port = compiled_model_.input();

      const auto expected_input_shape = input_port.get_shape();
      const size_t expected_input_length = expected_input_shape[1];
      const size_t actual_input_length = networkInputDataRL_.size();

      if (actual_input_length != expected_input_length)
      {
        ROS_ERROR_THROTTLE(1.0,
                           "[%s] networkInputDataRL_ size mismatch: actual=%ld vs expected=%ld",
                           name_.c_str(), actual_input_length, expected_input_length);
        action = Eigen::VectorXd::Zero(num_actions_);
        return false;
      }

      Eigen::VectorXf float_network_input = networkInputDataRL_.cast<float>();
      ov::Tensor input_tensor(input_port.get_element_type(),
                              input_port.get_shape(),
                              float_network_input.data());
      infer_request_.set_input_tensor(input_tensor);
      infer_request_.start_async();
      infer_request_.wait();

      const auto output_tensor = infer_request_.get_output_tensor();
      const size_t output_buf_length = output_tensor.get_size();
      const auto output_buf = output_tensor.data<float>();
      const size_t expected_output_length =
          withArm_ ? jointNum_ + jointArmNum_ + waistNum_ : jointNum_ + waistNum_;

      if (output_buf_length != expected_output_length)
      {
        ROS_ERROR_THROTTLE(1.0,
                           "[%s] Output size mismatch: actual=%ld vs expected=%ld (withArm_=%d)",
                           name_.c_str(), output_buf_length, expected_output_length, withArm_);
        action = Eigen::VectorXd::Zero(num_actions_);
        return false;
      }

      action.resize(output_buf_length);
      for (int i = 0; i < static_cast<int>(output_buf_length); ++i)
        action[i] = output_buf[i];

      clip(action, clipActions_);

      return true;
    }
    catch (const std::exception& e)
    {
      ROS_ERROR_THROTTLE(1.0, "[%s] Inference failed: %s", name_.c_str(), e.what());
      action = Eigen::VectorXd::Zero(num_actions_);
      return false;
    }
  }

  Eigen::VectorXd AmpWalkController::updateRLcmd(const Eigen::VectorXd& measuredRbdState)
  {
    // 基本照 humanoidController_rl.cpp::updateRLcmd，把 jointPos_/Vel_ 换成 getRobotSensorData()
    SensorData sensor_data = getRobotSensorData();
    Eigen::VectorXd jointPos = sensor_data.jointPos_;
    Eigen::VectorXd jointVel = sensor_data.jointVel_;

    Eigen::VectorXd motorPos = jointPos;
    Eigen::VectorXd motorVel = jointVel;

    Eigen::VectorXd local_action = getCurrentAction();

    if (!withArm_)
    {
      local_action.tail(jointArmNum_ + waistNum_).setZero();
    }

    Eigen::VectorXd jointTor(jointNum_ + jointArmNum_ + waistNum_);

    // 使用 is_roban_ 判断机型（与 FallStandController 一致）
    if (is_roban_)
    {
      // roban 机型：腰部在前，腿部从 waistNum_ 开始
      motorPos.segment(waistNum_, jointNum_) =
          ankleSolver_.joint_to_motor_position(jointPos.segment(waistNum_, jointNum_));
      motorVel.segment(waistNum_, jointNum_) =
          ankleSolver_.joint_to_motor_velocity(jointPos.segment(waistNum_, jointNum_),
                                               motorPos.segment(waistNum_, jointNum_),
                                               jointVel.segment(waistNum_, jointNum_));
      jointTor = -(jointKdRL_.cwiseProduct(motorVel));
      jointTor.segment(waistNum_, jointNum_) =
          ankleSolver_.motor_to_joint_torque(jointPos.segment(waistNum_, jointNum_),
                                             motorPos.segment(waistNum_, jointNum_),
                                             jointTor.segment(waistNum_, jointNum_));
    }
    else
    {
      // 其他机型：腿部从 0 开始
      motorPos.head(jointNum_) =
          ankleSolver_.joint_to_motor_position(jointPos.head(jointNum_));
      motorVel.head(jointNum_) =
          ankleSolver_.joint_to_motor_velocity(jointPos.head(jointNum_),
                                               motorPos.head(jointNum_),
                                               jointVel.head(jointNum_));
      jointTor = -(jointKdRL_.cwiseProduct(motorVel));
      jointTor.head(jointNum_) =
          ankleSolver_.motor_to_joint_torque(jointPos.head(jointNum_),
                                             motorPos.head(jointNum_),
                                             jointTor.head(jointNum_));
    }

    Eigen::VectorXd cmd(jointNum_ + jointArmNum_ + waistNum_);
    Eigen::VectorXd torque(jointNum_ + jointArmNum_ + waistNum_);
    for (int i = 0; i < jointNum_ + jointArmNum_ + waistNum_; i++)
    {
      jointTor(i) = jointTor(i) + jointKpRL_(i) * (local_action[i] * actionScale_ * actionScaleTestRL_[i] - jointPos[i] + defalutJointPosRL_[i]);
    }
    if (is_real_)
    {
      for (int i = 0; i < jointNum_ + jointArmNum_ + waistNum_; i++)
      {
        if (JointControlModeRL_(i) == 0)
        {
          if (JointPDModeRL_(i) == 0)
          {
            cmd[i] = jointKpRL_[i] * (local_action[i] * actionScale_ * actionScaleTestRL_[i] - jointPos[i] + defalutJointPosRL_[i]) - jointKdRL_[i] * jointVel[i];
            cmd[i] = std::clamp(cmd[i], -torqueLimitsRL_[i], torqueLimitsRL_[i]);
            torque[i] = cmd[i];
          }
          else
          {
            cmd[i] = (local_action[i] * actionScale_ * actionScaleTestRL_[i] + defalutJointPosRL_[i]);
            torque[i] = jointKpRL_[i] * (local_action[i] * actionScale_ * actionScaleTestRL_[i] - jointPos[i] + defalutJointPosRL_[i]) - jointKdRL_[i] * jointVel[i];
          }
        }
        else if (JointControlModeRL_(i) == 2)
        {
          cmd[i] = jointKpRL_[i] * (local_action[i] * actionScale_ * actionScaleTestRL_[i] - jointPos[i] + defalutJointPosRL_[i]);
          // cmd[i] = local_action[i] + defalutJointPosRL_[i];
          // std::cout << "cmd[" << i << "] = " << cmd[i] << "jointKpRL_:" << jointKpRL_[i] << std::endl;
          // cmd[i] = defalutJointPosRL_[i];
          torque[i] = jointTor[i];
        }
      }
      // std::cout << "local_action: " << local_action.transpose() << std::endl;
    }
    else
    {
      for (int i = 0; i < jointNum_ + jointArmNum_ + waistNum_; i++)
      {
        if (JointControlModeRL_(i) == 0)
        {
          cmd[i] = jointKpRL_[i] * (local_action[i] * actionScale_ * actionScaleTestRL_[i] - jointPos[i] + defalutJointPosRL_[i]) - jointKdRL_[i] * jointVel[i];
        }
        else if (JointControlModeRL_(i) == 2)
        {
          cmd[i] = jointTor[i];
        }
        cmd[i] = std::clamp(cmd[i], -torqueLimitsRL_[i], torqueLimitsRL_[i]);
      }

    }

    Eigen::VectorXd actuation = cmd;
    
    // 与 humanoidController_rl.cpp 保持一致：更新 episodeLength_ 并发布日志
    episodeLength_++;
    if (ros_logger_)
    {
      ros_logger_->publishVector("/rl_controller/torque", torque);
      ros_logger_->publishVector("/rl_controller/cmd", cmd);
    }


    return actuation;
  }

  void AmpWalkController::actionToJointCmd(const Eigen::VectorXd& actuation,
                                           const Eigen::VectorXd& measuredRbdState,
                                           kuavo_msgs::jointCmd& joint_cmd)
  {
    int total_joints = jointNum_ + jointArmNum_ + waistNum_;

    joint_cmd.joint_q.clear();
    joint_cmd.joint_v.clear();
    joint_cmd.joint_kp.clear();
    joint_cmd.joint_kd.clear();
    joint_cmd.tau.clear();
    joint_cmd.tau_ratio.clear();
    joint_cmd.tau_max.clear();
    joint_cmd.control_modes.clear();

    if (!is_real_)
    {
      for (int i = 0; i < total_joints; ++i)
      {
        joint_cmd.joint_q.push_back(0.0);
        joint_cmd.joint_v.push_back(0.0);
        joint_cmd.joint_kp.push_back(jointKpRL_[i]);
        joint_cmd.joint_kd.push_back(jointKdRL_[i]);
        joint_cmd.tau.push_back(actuation(i));
        joint_cmd.tau_ratio.push_back(1.0);
        joint_cmd.tau_max.push_back(torqueLimitsRL_[i]);
        joint_cmd.control_modes.push_back(JointControlModeRL_(i));
      }
    }
    else
    {
      // 真实机器人：与 FallStandController / humanoidController 中对实物的分支一致
      int total_body_joints = jointNum_ + jointArmNum_ + waistNum_;
      Eigen::VectorXd current_jointPos, current_jointVel;
      

      {
        // 如果state结构不同，尝试从传感器数据获取
        SensorData sensor_data = getRobotSensorData();
        current_jointPos = sensor_data.jointPos_.head(total_body_joints);
        current_jointVel = sensor_data.jointVel_.head(total_body_joints);
      }
      
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

  bool AmpWalkController::updateImpl(const ros::Time& time,
                                     const SensorData& sensor_data,
                                     const Eigen::VectorXd& measuredRbdState,
                                     kuavo_msgs::jointCmd& joint_cmd)
  {
    gait_receiver_->update(time, baseStateRL_, feetPositionsRL_);
    // 这里只做「用当前 actions_ 计算 actuation，再映射到 joint_cmd」
    Eigen::VectorXd actuation = updateRLcmd(measuredRbdState);
    actionToJointCmd(actuation, measuredRbdState, joint_cmd);
    joint_cmd.header.stamp = time;
    return true;
  }

  void AmpWalkController::preprocessSensorData(SensorData& sensor_data)
  {
    // 先执行基类中的通用滤波逻辑（RL IMU 滤波）
    RLControllerBase::preprocessSensorData(sensor_data);
    
    if (is_roban_)// roban模型使用旧顺序训练
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

} // namespace humanoid_controller
