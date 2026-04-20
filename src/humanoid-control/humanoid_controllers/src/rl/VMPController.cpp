#include "humanoid_controllers/rl/VMPController.h"
#include "vmp/vmp_rotation_utils.hpp"
#include "vmp/vmp_trajectory_loader.hpp"
#include "kuavo_common/common/common.h"

#include <ros/ros.h>
#include <sstream>
#include <fstream>
#include <iostream>
#include <filesystem>
#include <algorithm>
#include <numeric>
#include <chrono>

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/info_parser.hpp>
#include <ocs2_core/misc/LoadData.h>

namespace humanoid_controller
{

VMPController::VMPController(const std::string& name,
                             const std::string& config_file,
                             ros::NodeHandle& nh,
                             ocs2::humanoid::TopicLogger* ros_logger)
    : RLControllerBase(name, RLControllerType::VMP_CONTROLLER, config_file, nh, ros_logger)
{
}

bool VMPController::initialize()
{
  ROS_INFO("[VMPController] Initializing VMPController: %s", name_.c_str());

  // 检查机器人版本是否支持 VMP 控制器
  int robot_version_int;
  if (!nh_.getParam("/robot_version", robot_version_int))
  {
    ROS_ERROR("[VMPController] /robot_version not found in ROS params! Cannot initialize VMPController.");
    return false;
  }
  RobotVersion robot_version = RobotVersion::create(robot_version_int);

  // 当前仅支持 Kuavo4 Pro 的版本
  if (!IS_KUAVO4PRO_LEGGED(robot_version))
  {
    ROS_ERROR("[VMPController] 机器人版本 %s 不支持! VMPController 仅支持 Kuavo4 Pro (v45+).",
              robot_version.to_string().c_str());
    ROS_ERROR("[VMPController] 支持的版本: 45, 46, 47, 48, 49 及其补丁版本 (如 100045, 100046, ...)");
    return false;
  }
  ROS_INFO("[VMPController] 机器人版本 %s 支持.", robot_version.to_string().c_str());

  // 从 ROS 参数获取是否为真实机器人
  if (!nh_.getParam("/is_real", is_real_))
  {
    ROS_WARN("[%s] /is_real not found in ROS params, using default: %d", name_.c_str(), static_cast<int>(is_real_));
  }
  ROS_INFO("[%s] is_real: %s", name_.c_str(), is_real_ ? "true" : "false");

  // 初始化 ankleSolver（从ROS参数获取类型）
  int ankle_solver_type = 0;
  if (nh_.getParam("/ankle_solver_type", ankle_solver_type))
  {
    ROS_INFO("[%s] Using ankle_solver_type from ROS param: %d", name_.c_str(), ankle_solver_type);
  }
  else
  {
    ROS_WARN("[%s] /ankle_solver_type not found, using default: 0", name_.c_str());
  }
  ankleSolver_.getconfig(ankle_solver_type);

  // 从 ROS 参数获取是否包含手臂
  nh_.param<bool>("/with_arm", withArm_, true);
  ROS_INFO("[%s] withArm: %s", name_.c_str(), withArm_ ? "true" : "false");

  // 初始化 RL IMU 滤波（内部用 accFilterRL_ 等），同时加载 control_frequency_
  loadRLFilterParams(config_file_);

  // 使用配置文件中的控制频率设置 dt_
  dt_ = 1.0 / control_frequency_;

  try {
    // 加载配置文件
    if (!loadConfig(config_file_)) 
    {
      ROS_ERROR("[VMPController] Failed to load config file: %s", config_file_.c_str());
      return false;
    }

    initialStateRL_.resize(12 + defalutJointPosRL_.size());
    initialStateRL_ << defaultBaseStateRL_, defalutJointPosRL_;

    // 加载 VMP 模型
    setupVMPModels();

    // 加载 VMP 轨迹文件
    vmp_ref_motion_buffer_.clear();
    loadVMPRefData();

    initialized_ = true;

    // 打印轨迹状态信息
    printMultiTrajectoryStatus();

    // 初始化 ROS 接口
    initROSServices();

    ROS_INFO("[VMPController] VMPController initialized successfully");
  }
  catch (const std::exception& e) {
    ROS_ERROR("[VMPController] Failed to initialize VMPController: %s", e.what());
    return false;
  }
    
  return true;
}

bool VMPController::loadConfig(const std::string& config_file)
{
  ROS_INFO("[VMPController] Loading config from: %s", config_file.c_str());
  
  bool verbose = false;
  boost::property_tree::ptree pt;
  boost::property_tree::read_info(config_file, pt);

  Eigen::VectorXd jointCmdFilterCutoffFreq(jointNum_ + jointArmNum_ + waistNum_);

  loadData::loadEigenMatrix(config_file, "jointControlMode", JointControlModeRL_);
  loadData::loadEigenMatrix(config_file, "jointPDMode", JointPDModeRL_);
  loadData::loadEigenMatrix(config_file, "jointKp", jointKpRL_);
  loadData::loadEigenMatrix(config_file, "jointKd", jointKdRL_);
  loadData::loadEigenMatrix(config_file, "torqueLimits", torqueLimitsRL_);
  loadData::loadEigenMatrix(config_file, "actionScaleTest", actionScaleTestRL_);
  loadData::loadEigenMatrix(config_file, "defaultJointState", defalutJointPosRL_);
  loadData::loadEigenMatrix(config_file, "defaultBaseState", defaultBaseStateRL_);
  loadData::loadEigenMatrix(config_file, "jointCmdFilterCutoffFreq", jointCmdFilterCutoffFreq);

  jointCmdFilter_.setParams(dt_, jointCmdFilterCutoffFreq);
  jointCmdFilterState_.resize(jointCmdFilterCutoffFreq.size());
  jointCmdFilterState_.setZero();
  loadData::loadEigenMatrix(config_file, "jointCmdFilterState", jointCmdFilterState_);

  // 设置 initialStateRL_ = [defaultBaseStateRL_(12) + defalutJointPosRL_]
  initialStateRL_.resize(12 + defalutJointPosRL_.size());
  initialStateRL_ << defaultBaseStateRL_, defalutJointPosRL_;
  defaultBaseHeightControl_ = defaultBaseStateRL_(8);  // pos_z

  loadData::loadCppDataType(config_file, "inferenceFrequency", inference_frequency_);
  loadData::loadCppDataType(config_file, "enableThetaNormalization", vmp_enable_theta_normalization_);
  loadData::loadCppDataType(config_file, "actionScale", actionScale_);
  loadData::loadCppDataType(config_file, "frameStack", frameStackRL_);
  loadData::loadCppDataType(config_file, "numSingleObs", numSingleObsRL_);
  loadData::loadCppDataType(config_file, "clipObservations", clipObservationsRL_);
  loadData::loadCppDataType(config_file, "clipActions", clipActions_);

  //==========================================================================
  // 2. VMP环境参数
  //==========================================================================
  loadData::loadCppDataType(config_file, "numRefMotionObs", numRefMotionObs_);
  loadData::loadCppDataType(config_file, "numEncoderObs", numEncoderObs_);
  loadData::loadCppDataType(config_file, "numActions", num_actions_);
  loadData::loadCppDataType(config_file, "episodeLengthS", episodeLengthS_);
  loadData::loadCppDataType(config_file, "decimation", decimation_);

  setCurrentAction(Eigen::VectorXd::Zero(num_actions_));

  numSingleObsRL_ = 0;
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
        numSingleObsRL_ += static_cast<int>(numIdx);
        singleInputDataID_[pair2.first] = Eigen::Vector3d(startIdx, numIdx, obsScale);
      }
    }
  }
  
  if (numSingleObsRL_ <= 0)
  {
    ROS_ERROR("[%s] numSingleObsRL_ invalid", name_.c_str());
    return false;
  }

  singleInputDataRL_.resize(numSingleObsRL_);
  singleInputDataRL_.setZero();
  networkInputDataRL_.resize(numSingleObsRL_ * frameStackRL_);
  networkInputDataRL_.setZero();
  for (int i = 0; i < frameStackRL_; i++)
  {
    inputDeque_.push_back(singleInputDataRL_);
  }

  // 打印 singleInputDataKeys_ 和 singleInputDataID_
  std::cout << "[VMPController] singleInputData configuration:" << std::endl;
  for (const auto& key : singleInputDataKeys_) {
    const auto& value = singleInputDataID_[key];
    std::cout << "  - " << key << ": startIdx=" << value[0]
              << ", numIdx=" << value[1] << ", obsScale=" << value[2] << std::endl;
  }

  //==========================================================================
  // 3. 模型路径配置
  //==========================================================================
  std::string vmpPolicyModelFile, vmpEncoderModelFile;
  loadData::loadCppDataType(config_file, "vmpPolicyModelFile", vmpPolicyModelFile);
  loadData::loadCppDataType(config_file, "vmpEncoderModelFile", vmpEncoderModelFile);
  loadData::loadCppDataType(config_file, "vmpRefDataDir", vmpRefDataDir_);

  // 从 ROS 参数获取模型基础路径，与 AmpWalkController 保持一致
  std::string network_model_dir;
  nh_.getParam("/network_model_file", network_model_dir);
  vmpModelPath_ = network_model_dir + "/vmp_models/" + vmpPolicyModelFile;
  vmpEncoderPath_ = network_model_dir + "/vmp_models/" + vmpEncoderModelFile;

  // 获取 model_root_dir = $network_model_dir/../
  std::filesystem::path model_root_dir = std::filesystem::path(network_model_dir).parent_path();
  vmpRefDataDir_ = (model_root_dir / "vmp_ref_data").string();

  //==========================================================================
  // 4. 轨迹配置
  //==========================================================================

  // 解析轨迹配置列表
  trajectories_.clear();
  size_t idx = 0;
  while (true) {
    std::string prefix = "trajectories.[" + std::to_string(idx) + "]";
    vmp::TrajectoryData traj;
    try {
      loadData::loadCppDataType(config_file, prefix + ".name", traj.name);
      loadData::loadCppDataType(config_file, prefix + ".data_file", traj.data_file);
      trajectories_.push_back(traj);
      idx++;
    } catch (const std::exception&) {
      break;  // 没有更多配置项
    }
  }

  if (trajectories_.empty()) {
    ROS_ERROR("[VMPController] No trajectory configs found, please configure trajectoryConfigs in config file");
    return false;
  }

  ROS_INFO("[VMPController] Loaded %zu trajectory config(s)", trajectories_.size());
  for (size_t i = 0; i < trajectories_.size(); ++i) {
    std::cout << "  [" << i << "] " << trajectories_[i].name
              << " (data: " << trajectories_[i].data_file << ")" << std::endl;
  }

  //==========================================================================
  // 5. VAE模型配置 (vmp_config_)
  //==========================================================================
  vmp_config_.loadConfig(config_file);

  //==========================================================================
  // 6. 速度限制 (VMP独有)
  //==========================================================================
  velocityLimits_.resize(4);
  loadData::loadEigenMatrix(config_file, "velocityLimits", velocityLimits_);

  //==========================================================================
  // 11. 静止帧配置
  //==========================================================================
  loadData::loadCppDataType(config_file, "preStandingFrames", vmp_pre_standing_frames_);
  loadData::loadCppDataType(config_file, "postStandingFrames", vmp_post_standing_frames_);
  double vmp_standing_height = 0.9;
  loadData::loadCppDataType(config_file, "standingHeight", vmp_standing_height);

  // 插值配置
  loadData::loadCppDataType(config_file, "preInterpolationFrames", vmp_pre_interpolation_frames_);
  loadData::loadCppDataType(config_file, "postInterpolationFrames", vmp_post_interpolation_frames_);

  // 加载静止帧关节位置
  vmp_standing_joint_pos_.resize(jointNum_ + jointArmNum_);
  loadData::loadEigenMatrix(config_file, "standingJointPos", vmp_standing_joint_pos_);

  // 构建静止帧
  vmp_standing_frame_.resize(vmp_config_.in_c);
  std::fill(vmp_standing_frame_.begin(), vmp_standing_frame_.end(), 0.0f);

  // 设置高度
  vmp_standing_frame_[vmp_config_.h_start_id] = static_cast<float>(vmp_standing_height);

  // 设置单位旋转矩阵 (theta)
  vmp_standing_frame_[vmp_config_.theta_start_id] = 1.0f;     // R[0,0]
  vmp_standing_frame_[vmp_config_.theta_start_id + 1] = 0.0f; // R[0,1]
  vmp_standing_frame_[vmp_config_.theta_start_id + 2] = 0.0f; // R[0,2]
  vmp_standing_frame_[vmp_config_.theta_start_id + 3] = 0.0f; // R[1,0]
  vmp_standing_frame_[vmp_config_.theta_start_id + 4] = 1.0f; // R[1,1]
  vmp_standing_frame_[vmp_config_.theta_start_id + 5] = 0.0f; // R[1,2]

  // 设置关节位置
  for (int i = 0; i < vmp_config_.q_end_id - vmp_config_.q_start_id; ++i) {
    if (i < static_cast<int>(vmp_standing_joint_pos_.size())) {
      vmp_standing_frame_[vmp_config_.q_start_id + i] = static_cast<float>(vmp_standing_joint_pos_[i]);
    }
  }

  // 设置默认位置 (p)
  float p_vals[12] = {-0.0174999f,  0.2927f,  0.0353f,
                      -0.0174999f, -0.2927f,  0.0353f,
                      -0.03448717f, 0.087f,  -0.80041755f,
                      -0.03448717f, -0.087f, -0.80041755f};
  for (int i = 0; i < 12 && i < vmp_config_.p_end_id - vmp_config_.p_start_id; ++i) {
    vmp_standing_frame_[vmp_config_.p_start_id + i] = p_vals[i];
  }

  ROS_INFO("[VMPController] Standing frame config - preFrames: %d, postFrames: %d, height: %.2f",
           vmp_pre_standing_frames_, vmp_post_standing_frames_, vmp_standing_height);

  ROS_INFO("[VMPController] Config loaded successfully");
  return true;
}

void VMPController::reset()
{
  ROS_INFO("[VMPController] Resetting VMPController: %s", name_.c_str());

  // 重置轨迹进度（等待命令触发播放，不自动开始）
  trajectoryFrameCounter_ = 0;
  trajectoryPlaybackCompleted_ = true;
  currentTrajectoryIndex_ = 0;

  // 重置关节命令滤波器（与 kuavo-rl 保持一致，避免滤波器状态导致的不稳定）
  jointCmdFilter_.reset();

  // 重置观测历史
  if (!inputDeque_.empty()) {
    for (auto& obs : inputDeque_) {
      obs.setZero();
    }
  }
  singleInputDataRL_.setZero();
  networkInputDataRL_.setZero();

  // 重置动作输出
  setCurrentAction(Eigen::VectorXd::Zero(num_actions_));

  // 重新初始化参考运动缓冲区
  vmp_ref_motion_buffer_.clear();
  if (!vmp_task_data_.empty() && vmp_task_data_.size() >= static_cast<size_t>(vmp_config_.in_c)) {
    Eigen::VectorXd first_frame(vmp_config_.in_c);
    for (int i = 0; i < vmp_config_.in_c; ++i) {
      first_frame[i] = static_cast<double>(vmp_task_data_[i]);
    }
    // 应用 yaw 归一化（如果启用），确保与 updateVMPReferenceMotion 中的处理一致
    if (vmp_enable_theta_normalization_) {
      vmp::rotation_utils::normalize_ref_motion_yaw(first_frame, vmp_config_.theta_start_id, vmp_config_.theta_end_id);
    }
    for (int i = 0; i < vmp_config_.window_l; i++) {
      vmp_ref_motion_buffer_.push_back(first_frame);
    }
  }
}

void VMPController::pause()
{
  RLControllerBase::pause();
  // 停止轨迹播放
  playback_state_ = TrajectoryPlaybackState::STOPPED;

  // Ruiwo 手臂增益通过 joint_cmd 实时下发，切换后由 replaceDefaultEcMotorPdoGait 恢复默认增益
  ROS_INFO("[VMPController] Controller paused, trajectory playback stopped");
}

void VMPController::resume()
{
  RLControllerBase::resume();

  // Ruiwo 手臂增益通过 motor_pdo_kp/kd（VMP info 文件）在 joint_cmd 中实时下发
  ROS_INFO("[VMPController] Controller resumed, resetting state");
  reset();
}


bool VMPController::isReadyToExit() const
{
  return RLControllerBase::isReadyToExit();
}

bool VMPController::updateImpl(const ros::Time& time,
                                const SensorData& sensor_data,
                                const Eigen::VectorXd& measuredRbdState,
                                kuavo_msgs::jointCmd& joint_cmd)
{
  Eigen::VectorXd actuation = updateRLcmd(measuredRbdState);
  actionToJointCmd(actuation, measuredRbdState, joint_cmd);
  joint_cmd.header.stamp = time;
  return true;
}

bool VMPController::inference(const Eigen::VectorXd& observation,
                               Eigen::VectorXd& action)
{
  try {
    // 1. 计算 VMP 动作
    action = computeVMPAction(observation);

    // 2. 更新参考运动
    updateVMPReferenceMotion();

    return true;
  } catch (const std::exception& e) {
    ROS_ERROR("[VMPController] Inference error: %s", e.what());
    action = Eigen::VectorXd::Zero(num_actions_);
    return false;
  }
}

void VMPController::updateObservation(const Eigen::VectorXd& state_est,
                                       const SensorData& sensor_data)
{
  size_t total_joints = jointNum_ + waistNum_ + jointArmNum_;
  // 提取状态数据（与humanoidController一致）
  const Eigen::Vector3d baseEuler(state_est(2), state_est(1), state_est(0));
  const Eigen::Vector3d baseAngVel(state_est(6 + total_joints),
                                   state_est(6 + total_joints + 1),
                                   state_est(6 + total_joints + 2));

  // 提取和处理传感器数据
  Eigen::VectorXd jointPos = sensor_data.jointPos_;
  Eigen::VectorXd jointVel = sensor_data.jointVel_;
  jointPos = jointPos - defalutJointPosRL_;

  Eigen::Vector3d bodyAngVel = sensor_data.angularVel_;

  // 获取本地动作（使用基类的线程安全方法）
  Eigen::VectorXd local_action = getCurrentAction();

  const std::map<std::string, Eigen::VectorXd> singleInputDataMap = {
      {"baseEuler", baseEuler},
      {"bodyAngVel", bodyAngVel},
      {"jointPos", jointPos},
      {"jointVel", jointVel},
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
      singleInputDataRL_.segment(index, value[1]) = singleInputDataMap.at(key).segment(value[0], value[1]) * value[2];
      index += value[1];
      if (ros_logger_)
      {
        ros_logger_->publishVector("/rl_controller/InputData/" + key, singleInputDataMap.at(key).segment(value[0], value[1]) * value[2]);
      }
    }
  }
  else
  {
    ROS_ERROR("[%s] singleInputDataKeys_ is empty, cannot build observation", name_.c_str());
    singleInputDataRL_.setZero();
  }

   // Clip and update inputDeque_（与humanoidController一致）
    clip(singleInputDataRL_, clipObservationsRL_);
    inputDeque_.push_back(singleInputDataRL_);
    inputDeque_.pop_front();
    
    // Update networkInputData_（与humanoidController一致）
    for (int i = 0; i < frameStackRL_; ++i)
    {
      networkInputDataRL_.segment(i * numSingleObsRL_, numSingleObsRL_) = inputDeque_[i];
    }
    
    // 发布观测数据（如果ros_logger_可用）
    if (ros_logger_)
    {
      ros_logger_->publishVector("/rl_controller/singleInputData", singleInputDataRL_);
    }
}

void VMPController::actionToJointCmd(const Eigen::VectorXd& actuation,
                                      const Eigen::VectorXd& measuredRbdState,
                                      kuavo_msgs::jointCmd& joint_cmd)
{
  // 先清空 joint_cmd
  joint_cmd.joint_q.clear();
  joint_cmd.joint_v.clear();
  joint_cmd.joint_kp.clear();
  joint_cmd.joint_kd.clear();
  joint_cmd.tau.clear();
  joint_cmd.tau_ratio.clear();
  joint_cmd.tau_max.clear();
  joint_cmd.control_modes.clear();

  // 关节总数: 维度不包含头部
  int total_joints = jointNum_ + jointArmNum_ + waistNum_;

  // 按照 humanoidController 中 is_vmp_controller_ 模式的实现
  // SensorData sensor_data = getRobotSensorData();
  // auto current_jointPos = sensor_data.jointPos_.head(total_joints);
  auto current_jointPos = measuredRbdState.segment(6, total_joints);

  if (!is_real_) {
    // 仿真环境
    for (int i = 0; i < total_joints; ++i) {
      joint_cmd.joint_q.push_back(0.0);
      joint_cmd.joint_v.push_back(0.0);
      joint_cmd.joint_kp.push_back(jointKpRL_[i]);
      joint_cmd.joint_kd.push_back(jointKdRL_[i]);
      joint_cmd.tau.push_back(actuation(i));
      joint_cmd.tau_ratio.push_back(1);
      joint_cmd.tau_max.push_back(torqueLimitsRL_[i]);
      joint_cmd.control_modes.push_back(JointControlModeRL_(i));
    }
  } else {
    // 真实机器人
    for (int i = 0; i < total_joints; ++i) {
      if (JointControlModeRL_(i) == 0) {
        if (JointPDModeRL_(i) == 0) {
          // CST 模式 (力矩控制)
          joint_cmd.joint_q.push_back(0.0);
          joint_cmd.joint_v.push_back(0.0);
          joint_cmd.joint_kp.push_back(0);
          joint_cmd.joint_kd.push_back(0);
          joint_cmd.tau.push_back(actuation(i));
          joint_cmd.tau_ratio.push_back(1);
          joint_cmd.tau_max.push_back(torqueLimitsRL_[i]);
          joint_cmd.control_modes.push_back(JointControlModeRL_(i));
        } else {
          // 位置控制模式
          joint_cmd.joint_q.push_back(actuation(i));
          joint_cmd.joint_v.push_back(0.0);
          joint_cmd.joint_kp.push_back(jointKpRL_[i]);
          joint_cmd.joint_kd.push_back(jointKdRL_[i]);
          joint_cmd.tau.push_back(0.0);
          joint_cmd.tau_ratio.push_back(1);
          joint_cmd.tau_max.push_back(torqueLimitsRL_[i]);
          joint_cmd.control_modes.push_back(JointControlModeRL_(i));
        }
      } else if (JointControlModeRL_(i) == 2) {
        // CSP 模式
        joint_cmd.joint_q.push_back(current_jointPos(i));
        joint_cmd.joint_v.push_back(0.0);
        joint_cmd.joint_kp.push_back(jointKpRL_[i]);
        joint_cmd.joint_kd.push_back(jointKdRL_[i]);
        joint_cmd.tau.push_back(actuation(i));
        joint_cmd.tau_ratio.push_back(1);
        joint_cmd.tau_max.push_back(torqueLimitsRL_[i]);
        joint_cmd.control_modes.push_back(JointControlModeRL_(i));
      }
    }
  }

  // 设置头部关节（保持零位，与 AmpWalkController 一致）
  for (int i = 0; i < headNum_; ++i)
  {
    joint_cmd.joint_q.push_back(0.0);
    joint_cmd.joint_v.push_back(0.0);
    joint_cmd.tau.push_back(0.0);
    joint_cmd.tau_ratio.push_back(1.0);
    joint_cmd.tau_max.push_back(10.0);
    joint_cmd.joint_kp.push_back(0.0);
    joint_cmd.joint_kd.push_back(0.0);
    joint_cmd.control_modes.push_back(2);   // CSP 模式（与 kuavo-rl 一致）
  }
}

Eigen::VectorXd VMPController::updateRLcmd(const Eigen::VectorXd& state)
{
  // 获取传感器数据 原来是从state获取的，现在改为从sensor_data获取
  SensorData sensor_data = getRobotSensorData();
  const Eigen::VectorXd jointPos = sensor_data.jointPos_;
  const Eigen::VectorXd jointVel = sensor_data.jointVel_;

  Eigen::VectorXd motorPos = jointPos;
  Eigen::VectorXd motorVel = jointVel;

  // 获取当前动作（线程安全）
  Eigen::VectorXd local_action = getCurrentAction();

  // 如果不启用手臂控制，将手臂动作置零
  if (!withArm_) {
    local_action.tail(jointArmNum_ + waistNum_).setZero();
  }

  // 初始化输出向量
  Eigen::VectorXd actuation(jointNum_ + jointArmNum_ + waistNum_);
  Eigen::VectorXd cmd_out(jointNum_ + jointArmNum_ + waistNum_);
  Eigen::VectorXd cmd(jointNum_ + jointArmNum_ + waistNum_);
  Eigen::VectorXd cmd_filter(jointNum_ + jointArmNum_ + waistNum_);
  Eigen::VectorXd torque(jointNum_ + jointArmNum_ + waistNum_);

  // 使用 ankleSolver 进行关节到电机的转换
  motorPos.head(jointNum_) = ankleSolver_.joint_to_motor_position(jointPos.head(jointNum_));
  motorVel.head(jointNum_) = ankleSolver_.joint_to_motor_velocity(
      jointPos.head(jointNum_), motorPos.head(jointNum_), jointVel.head(jointNum_));

  // 计算关节扭矩
  Eigen::VectorXd jointTor = -(jointKdRL_.cwiseProduct(motorVel));
  jointTor.head(jointNum_) = ankleSolver_.motor_to_joint_torque(
      jointPos.head(jointNum_), motorPos.head(jointNum_), jointTor.head(jointNum_));

  // 添加 PD 控制项
  for (int i = 0; i < jointNum_ + jointArmNum_ + waistNum_; i++) {
    jointTor(i) = jointTor(i) + jointKpRL_(i) *
        (local_action[i] * actionScale_ * actionScaleTestRL_[i] - jointPos[i] + defalutJointPosRL_[i]);
  }

  // 根据控制模式计算命令
  if (is_real_) {
    for (int i = 0; i < jointNum_ + jointArmNum_ + waistNum_; i++) {
      if (JointControlModeRL_(i) == 0) {
        if (JointPDModeRL_(i) == 0) {
          // CST 模式 (力矩控制)
          cmd[i] = jointKpRL_[i] * (local_action[i] * actionScale_ * actionScaleTestRL_[i]
                   - jointPos[i] + defalutJointPosRL_[i]) - jointKdRL_[i] * jointVel[i];
          cmd[i] = std::clamp(cmd[i], -torqueLimitsRL_[i], torqueLimitsRL_[i]);
          torque[i] = cmd[i];
        } else {
          // 位置控制模式
          cmd[i] = local_action[i] * actionScale_ * actionScaleTestRL_[i] + defalutJointPosRL_[i];
          torque[i] = jointKpRL_[i] * (local_action[i] * actionScale_ * actionScaleTestRL_[i]
                      - jointPos[i] + defalutJointPosRL_[i]) - jointKdRL_[i] * jointVel[i];
        }
      } else if (JointControlModeRL_(i) == 2) {
        // CSP 模式 (位置控制 + 力矩前馈)
        cmd[i] = jointKpRL_[i] * (local_action[i] * actionScale_ * actionScaleTestRL_[i]
                 - jointPos[i] + defalutJointPosRL_[i]);
        torque[i] = cmd[i] - jointKdRL_[i] * jointVel[i];
      }
    }
  } else {
    // 仿真环境
    for (int i = 0; i < jointNum_ + jointArmNum_ + waistNum_; i++) {
      if (JointControlModeRL_(i) == 0) {
        cmd[i] = jointKpRL_[i] * (local_action[i] * actionScale_ * actionScaleTestRL_[i]
                 - jointPos[i] + defalutJointPosRL_[i]) - jointKdRL_[i] * jointVel[i];
      } else if (JointControlModeRL_(i) == 2) {
        cmd[i] = jointTor[i];
      }
      cmd[i] = std::clamp(cmd[i], -torqueLimitsRL_[i], torqueLimitsRL_[i]);
      torque[i] = cmd[i];
    }
  }

  // 应用关节命令滤波器
  cmd_filter = jointCmdFilter_.update(cmd);
  cmd_out = cmd_filter.cwiseProduct(jointCmdFilterState_) +
      cmd.cwiseProduct(Eigen::VectorXd::Ones(jointNum_ + jointArmNum_ + waistNum_) - jointCmdFilterState_);
  actuation = cmd_out;

  // 发布调试数据
  if (ros_logger_) {
    ros_logger_->publishVector("/vmp_controller/cmd_actuation", cmd_out);
    ros_logger_->publishVector("/vmp_controller/target_torque", torque);
    ros_logger_->publishVector("/vmp_controller/cmd_filter", cmd_filter);
    ros_logger_->publishVector("/vmp_controller/target_cmd", cmd);
    ros_logger_->publishVector("/vmp_controller/joint_torque_with_ankle_solver", jointTor);
  }

  return actuation;
}

bool VMPController::shouldRunInference() const
{
  return RLControllerBase::shouldRunInference();
}

void VMPController::preprocessSensorData(SensorData& sensor_data)
{
  RLControllerBase::preprocessSensorData(sensor_data);

  // 对 sensors_data 进行其他预处理
}

//==========================================================================
// Model Setup Functions
//==========================================================================

void VMPController::setupVMPModels()
{
  ROS_INFO("[VMPController] Loading VMP policy model from: %s", vmpModelPath_.c_str());
  ROS_INFO("[VMPController] Loading VAE encoder model from: %s", vmpEncoderPath_.c_str());

  try {
    vmp_policy_model_ = core_.compile_model(vmpModelPath_, "CPU");
    ROS_INFO("[VMPController] Policy model compiled successfully");

    vmp_encoder_model_ = core_.compile_model(vmpEncoderPath_, "CPU");
    ROS_INFO("[VMPController] Encoder model compiled successfully");

    vmp_policy_request_ = vmp_policy_model_.create_infer_request();
    ROS_INFO("[VMPController] Policy inference request created");

    vmp_encoder_request_ = vmp_encoder_model_.create_infer_request();
    ROS_INFO("[VMPController] Encoder inference request created");

    auto policy_input = vmp_policy_model_.input();
    auto policy_output = vmp_policy_model_.output();

    std::cout << "[VMP] Policy input element type: " << policy_input.get_element_type() << std::endl;
    std::cout << "[VMP] Policy input partial shape: " << policy_input.get_partial_shape() << std::endl;
    std::cout << "[VMP] Policy output element type: " << policy_output.get_element_type() << std::endl;
    std::cout << "[VMP] Policy output partial shape: " << policy_output.get_partial_shape() << std::endl;
        
    warmupVMPModels();

  } catch (const std::exception& e) {
    ROS_ERROR("[VMPController] Error during model setup: %s", e.what());
    throw;
  }

  ROS_INFO("[VMPController] VMP models loaded successfully");
}

void VMPController::warmupVMPModels()
{
  try {
    const int warmup_iterations = 3;

    ROS_INFO("[VMP Warmup] Starting encoder warm-up (%d iterations)...", warmup_iterations);

    // Warmup encoder
    ov::Shape encoder_input_shape = {1, static_cast<size_t>(vmp_config_.in_c), static_cast<size_t>(vmp_config_.window_l)};
    ov::Tensor encoder_dummy_tensor(ov::element::f32, encoder_input_shape);
    float* encoder_dummy_data = encoder_dummy_tensor.data<float>();

    for (size_t i = 0; i < encoder_dummy_tensor.get_size(); ++i) {
      encoder_dummy_data[i] = 0.0f;
    }

    auto encoder_warmup_start = std::chrono::high_resolution_clock::now();
    for (int i = 0; i < warmup_iterations; ++i) {
      vmp_encoder_request_.set_input_tensor(0, encoder_dummy_tensor);
      vmp_encoder_request_.infer();
    }
    auto encoder_warmup_end = std::chrono::high_resolution_clock::now();
    double encoder_warmup_time = std::chrono::duration<double, std::milli>(encoder_warmup_end - encoder_warmup_start).count();
    ROS_INFO("[VMP Warmup] Encoder warm-up completed, total time: %.2f ms, avg: %.2f ms per inference",
             encoder_warmup_time, encoder_warmup_time / warmup_iterations);

    // Warmup policy
    ROS_INFO("[VMP Warmup] Starting policy warm-up (%d iterations)...", warmup_iterations);

    int policy_input_size = numSingleObsRL_ + numRefMotionObs_ + vmp_config_.latent_d; // 83 + 77 + 512 = 672
    ov::Shape policy_input_shape = {1, static_cast<size_t>(policy_input_size)};
    ov::Tensor policy_dummy_tensor(ov::element::f32, policy_input_shape);
    float* policy_dummy_data = policy_dummy_tensor.data<float>();

    for (size_t i = 0; i < policy_dummy_tensor.get_size(); ++i) {
      policy_dummy_data[i] = 0.0f;
    }

    auto policy_warmup_start = std::chrono::high_resolution_clock::now();
    for (int i = 0; i < warmup_iterations; ++i) {
      vmp_policy_request_.set_input_tensor(0, policy_dummy_tensor);
      vmp_policy_request_.infer();
    }
    auto policy_warmup_end = std::chrono::high_resolution_clock::now();
    double policy_warmup_time = std::chrono::duration<double, std::milli>(policy_warmup_end - policy_warmup_start).count();
    ROS_INFO("[VMP Warmup] Policy warm-up completed, total time: %.2f ms, avg: %.2f ms per inference",
             policy_warmup_time, policy_warmup_time / warmup_iterations);

  } catch (const std::exception& e) {
    ROS_WARN("[VMPController] Warm-up failed: %s", e.what());
  }
}

//==========================================================================
// Trajectory Loading Functions
//==========================================================================

void VMPController::loadVMPRefData()
{
  try {
    // 初始化播放状态（等待命令触发播放，不自动开始）
    trajectoryPlaybackCompleted_ = true;
    trajectoryFrameCounter_ = 0;
    currentTrajectoryIndex_ = 0;

    // 加载所有轨迹
    if (loadAllTrajectories()) {
      ROS_INFO("[VMP] All trajectories loaded successfully");
      switchToTrajectory(0);
    } else {
      ROS_ERROR("[VMP] Failed to load trajectories");
    }
  } catch (const std::exception& e) {
    ROS_ERROR("[VMP] Error in loadVMPRefData: %s", e.what());
    throw;
  }
}

void VMPController::appendStandingFramesToTaskData()
{
  if (vmp_task_data_.empty()) {
    ROS_WARN("[VMPController] Cannot append standing frames to empty task data");
    return;
  }

  int original_total_frames = vmp_task_data_.size() / vmp_config_.in_c;
  int new_total_frames = original_total_frames + vmp_pre_standing_frames_ + vmp_post_standing_frames_;

  ROS_INFO("[VMPController] Appending standing frames: %d pre + %d original + %d post = %d total",
           vmp_pre_standing_frames_, original_total_frames, vmp_post_standing_frames_, new_total_frames);

  std::vector<float> start_real(vmp_config_.in_c);
  for (int j = 0; j < vmp_config_.in_c; ++j) {
    start_real[j] = vmp_task_data_[j];
  }

  std::vector<float> new_task_data(new_total_frames * vmp_config_.in_c);

  // Pre-standing frames with interpolation
  for (int i = 0; i < vmp_pre_standing_frames_; ++i) {
    int start_idx = i * vmp_config_.in_c;
    for (int j = 0; j < vmp_config_.in_c; ++j) {
      new_task_data[start_idx + j] = vmp_standing_frame_[j];
    }

    if (i >= vmp_pre_standing_frames_ - vmp_pre_interpolation_frames_) {
      float alpha = float(i - (vmp_pre_standing_frames_ - vmp_pre_interpolation_frames_) + 1) /
                    float(vmp_pre_interpolation_frames_ + 1);
      for (int j = vmp_config_.q_start_id; j < vmp_config_.q_end_id; ++j) {
        new_task_data[start_idx + j] = (1.0f - alpha) * vmp_standing_frame_[j] + alpha * start_real[j];
      }
    }
  }

  // Original data
  int original_start_idx = vmp_pre_standing_frames_ * vmp_config_.in_c;
  for (size_t i = 0; i < vmp_task_data_.size(); ++i) {
    new_task_data[original_start_idx + i] = vmp_task_data_[i];
  }

  // Post-standing frames with interpolation
  std::vector<float> end_real(vmp_config_.in_c);
  int last_frame_idx = (original_total_frames - 1) * vmp_config_.in_c;
  for (int j = 0; j < vmp_config_.in_c; ++j) {
    end_real[j] = vmp_task_data_[last_frame_idx + j];
  }

  int post_start_idx = (vmp_pre_standing_frames_ + original_total_frames) * vmp_config_.in_c;
  for (int i = 0; i < vmp_post_standing_frames_; ++i) {
    int start_idx = post_start_idx + i * vmp_config_.in_c;
    for (int j = 0; j < vmp_config_.in_c; ++j) {
      new_task_data[start_idx + j] = vmp_standing_frame_[j];
    }

    if (i < vmp_post_interpolation_frames_) {
      float alpha = float(vmp_post_interpolation_frames_ - i) / float(vmp_post_interpolation_frames_ + 1);
      for (int j = vmp_config_.q_start_id; j < vmp_config_.q_end_id; ++j) {
        new_task_data[start_idx + j] = alpha * end_real[j] + (1.0f - alpha) * vmp_standing_frame_[j];
      }
    }
  }

  vmp_task_data_ = std::move(new_task_data);
  ROS_INFO("[VMPController] Task data updated with standing frames: %zu floats", vmp_task_data_.size());
}

//==========================================================================
// Reference Motion Update
//==========================================================================

void VMPController::updateVMPReferenceMotion()
{
  try {
    if (vmp_task_data_.empty()) {
      return;
    }

    int total_frames = vmp_task_data_.size() / vmp_config_.in_c;
    int current_frame = trajectoryFrameCounter_;

    // 检查是否到达轨迹末尾
    if (total_frames > 0 && current_frame >= total_frames) {
      // 播放完成，静止在最后一帧（通过外部接口切换轨迹）
      trajectoryPlaybackCompleted_ = true;
      current_frame = total_frames - 1;
    }

    // 递增帧计数器
    if (!trajectoryPlaybackCompleted_) {
      trajectoryFrameCounter_++;
    }

    Eigen::VectorXd current_ref_frame(vmp_config_.in_c);
    int start_idx = current_frame * vmp_config_.in_c;

    for (int i = 0; i < vmp_config_.in_c; ++i) {
      int data_idx = start_idx + i;
      current_ref_frame[i] = (data_idx < static_cast<int>(vmp_task_data_.size())) ?
                             static_cast<double>(vmp_task_data_[data_idx]) : 0.0;
    }

    if (vmp_enable_theta_normalization_) {
      vmp::rotation_utils::normalize_ref_motion_yaw(current_ref_frame, vmp_config_.theta_start_id, vmp_config_.theta_end_id);
    }

    if (static_cast<int>(vmp_ref_motion_buffer_.size()) >= vmp_config_.window_l) {
      vmp_ref_motion_buffer_.erase(vmp_ref_motion_buffer_.begin());
    }
    vmp_ref_motion_buffer_.push_back(current_ref_frame);

    // 发布参考运动数据用于调试
    if (ros_logger_) {
      ros_logger_->publishVector("/vmp_controller/motion_ref_data", current_ref_frame);
      Eigen::VectorXd trajectory_info(2);
      trajectory_info << static_cast<double>(current_frame), static_cast<double>(total_frames);
      ros_logger_->publishVector("/vmp_controller/trajectory_frame", trajectory_info);
    }

  } catch (const std::exception& e) {
    ROS_ERROR("[VMPController] Error updating reference motion: %s", e.what());
  }
}

//==========================================================================
// VMP Inference
//==========================================================================

void VMPController::applyTemporalNormalization(float* input_data, size_t data_size)
{
  // Apply temporal normalization to the input data
  // This includes: yaw rotation removal, velocity/position rotation, and z-score normalization

  int window_length = vmp_config_.window_l;
  int feature_dim = vmp_config_.in_c;
  int window_r = window_length / 2;  // center frame index

  if (data_size != static_cast<size_t>(window_length * feature_dim)) {
    ROS_WARN("[VMP] Data size mismatch in temporal normalization: %zu vs expected %d",
             data_size, window_length * feature_dim);
    return;
  }

  // Step 1: Extract theta (6D rotation) from center frame
  float cfr_data[6];
  for (int i = 0; i < 6; i++) {
    int tensor_idx = window_r * feature_dim + (vmp_config_.theta_start_id + i);
    cfr_data[i] = input_data[tensor_idx];
  }

  // Step 2: Orthogonalize to get rotation matrix (Gram-Schmidt)
  float c0[3] = {cfr_data[0], cfr_data[1], cfr_data[2]};
  float c1[3] = {cfr_data[3], cfr_data[4], cfr_data[5]};

  // Normalize c0
  float c0_norm = std::sqrt(c0[0]*c0[0] + c0[1]*c0[1] + c0[2]*c0[2]);
  if (c0_norm > 1e-6f) {
    c0[0] /= c0_norm; c0[1] /= c0_norm; c0[2] /= c0_norm;
  }

  // Orthogonalize c1 against c0
  float dot_product = c0[0]*c1[0] + c0[1]*c1[1] + c0[2]*c1[2];
  float proj[3] = {dot_product * c0[0], dot_product * c0[1], dot_product * c0[2]};
  c1[0] -= proj[0]; c1[1] -= proj[1]; c1[2] -= proj[2];

  // Normalize c1
  float c1_norm = std::sqrt(c1[0]*c1[0] + c1[1]*c1[1] + c1[2]*c1[2]);
  if (c1_norm > 1e-6f) {
    c1[0] /= c1_norm; c1[1] /= c1_norm; c1[2] /= c1_norm;
  }

  // Compute c2 = c0 x c1 (cross product, not used but completes the rotation matrix)
  // float c2[3] = {c0[1]*c1[2] - c0[2]*c1[1], c0[2]*c1[0] - c0[0]*c1[2], c0[0]*c1[1] - c0[1]*c1[0]};

  // Step 3: Compute yaw angle and build inverse rotation matrix
  float yaw = std::atan2(c0[1], c0[0]);
  float cos_yaw = std::cos(yaw);
  float sin_yaw = std::sin(yaw);

  float inv_rot[3][3] = {
    { cos_yaw,  sin_yaw, 0.0f},
    {-sin_yaw,  cos_yaw, 0.0f},
    { 0.0f,     0.0f,    1.0f}
  };

  // Step 4: Apply inverse rotation to velocity data (v_start_id to v_end_id)
  int v_dim = vmp_config_.v_end_id - vmp_config_.v_start_id;  // should be 6 (2 vectors x 3)
  for (int t = 0; t < window_length; t++) {
    // Rotate first velocity vector (indices 0-2 within v range)
    if (v_dim >= 3) {
      float vec1[3];
      for (int i = 0; i < 3; i++) {
        vec1[i] = input_data[t * feature_dim + vmp_config_.v_start_id + i];
      }
      float rot_vec1[3];
      for (int i = 0; i < 3; i++) {
        rot_vec1[i] = inv_rot[i][0] * vec1[0] + inv_rot[i][1] * vec1[1] + inv_rot[i][2] * vec1[2];
      }
      for (int i = 0; i < 3; i++) {
        input_data[t * feature_dim + vmp_config_.v_start_id + i] = rot_vec1[i];
      }
    }

    // Rotate second velocity vector (indices 3-5 within v range)
    if (v_dim >= 6) {
      float vec2[3];
      for (int i = 0; i < 3; i++) {
        vec2[i] = input_data[t * feature_dim + vmp_config_.v_start_id + 3 + i];
      }
      float rot_vec2[3];
      for (int i = 0; i < 3; i++) {
        rot_vec2[i] = inv_rot[i][0] * vec2[0] + inv_rot[i][1] * vec2[1] + inv_rot[i][2] * vec2[2];
      }
      for (int i = 0; i < 3; i++) {
        input_data[t * feature_dim + vmp_config_.v_start_id + 3 + i] = rot_vec2[i];
      }
    }
  }

  // Step 5: Apply inverse rotation to position data (p_start_id to p_end_id)
  int p_dim = vmp_config_.p_end_id - vmp_config_.p_start_id;  // should be 12 (4 vectors x 3)
  int num_p_vectors = p_dim / 3;
  for (int t = 0; t < window_length; t++) {
    for (int v = 0; v < num_p_vectors; v++) {
      float vec[3];
      for (int i = 0; i < 3; i++) {
        vec[i] = input_data[t * feature_dim + vmp_config_.p_start_id + v * 3 + i];
      }
      float rot_vec[3];
      for (int i = 0; i < 3; i++) {
        rot_vec[i] = inv_rot[i][0] * vec[0] + inv_rot[i][1] * vec[1] + inv_rot[i][2] * vec[2];
      }
      for (int i = 0; i < 3; i++) {
        input_data[t * feature_dim + vmp_config_.p_start_id + v * 3 + i] = rot_vec[i];
      }
    }
  }

  // Step 6: Z-score normalization (exclude theta features)
  std::vector<bool> norm_mask(feature_dim, true);
  for (int i = vmp_config_.theta_start_id; i < vmp_config_.theta_end_id; i++) {
    norm_mask[i] = false;
  }

  // Compute mean for each feature
  std::vector<float> feature_means(feature_dim, 0.0f);
  for (int f = 0; f < feature_dim; f++) {
    if (norm_mask[f]) {
      double sum = 0.0;
      for (int t = 0; t < window_length; t++) {
        sum += static_cast<double>(input_data[t * feature_dim + f]);
      }
      feature_means[f] = static_cast<float>(sum / window_length);
    }
  }

  // Compute std for each feature
  std::vector<float> feature_stds(feature_dim, 0.0f);
  for (int f = 0; f < feature_dim; f++) {
    if (norm_mask[f]) {
      double sum_sq_diff = 0.0;
      for (int t = 0; t < window_length; t++) {
        double diff = static_cast<double>(input_data[t * feature_dim + f]) -
                      static_cast<double>(feature_means[f]);
        sum_sq_diff += diff * diff;
      }
      feature_stds[f] = static_cast<float>(std::sqrt(sum_sq_diff / window_length));
    }
  }

  // Build padded mean/std (theta features use 0/1)
  std::vector<float> pad_mu(feature_dim);
  std::vector<float> pad_std(feature_dim);
  for (int f = 0; f < feature_dim; f++) {
    if (f >= vmp_config_.theta_start_id && f < vmp_config_.theta_end_id) {
      pad_mu[f] = 0.0f;
      pad_std[f] = 1.0f;
    } else {
      pad_mu[f] = feature_means[f];
      pad_std[f] = feature_stds[f];
    }
  }

  // Apply z-score normalization
  for (int t = 0; t < window_length; t++) {
    for (int f = 0; f < feature_dim; f++) {
      float normalized_value = (input_data[t * feature_dim + f] - pad_mu[f]) /
                              (pad_std[f] + 1e-8f);
      input_data[t * feature_dim + f] = normalized_value;
    }
  }
}

Eigen::VectorXd VMPController::computeVMPAction(const Eigen::VectorXd& observation)
{
  try {
    // 准备编码器输入张量
    ov::Shape encoder_input_shape = {1, static_cast<size_t>(vmp_config_.in_c), static_cast<size_t>(vmp_config_.window_l)};
    ov::Tensor encoder_input_tensor(ov::element::f32, encoder_input_shape);
    vmp_encoder_request_.set_input_tensor(0, encoder_input_tensor);

    float* input_data = encoder_input_tensor.data<float>();

    // 统一使用 trajectoryFrameCounter_（由 updateVMPReferenceMotion 维护）
    int current_frame = trajectoryFrameCounter_;
    int window_r = vmp_config_.window_l / 2;  // r = 15
    int total_frames = vmp_task_data_.size() / vmp_config_.in_c;

    // 边界处理
    if (total_frames > 0) {
      if (current_frame >= total_frames) {
        current_frame = total_frames - 1;
      }
    } else {
      current_frame = 0;
    }

    // 构建滑动窗口采样数据
    std::vector<float> sample_batch(vmp_config_.window_l * vmp_config_.in_c);

    for (int t = 0; t < vmp_config_.window_l; ++t) {
      int frame_idx = current_frame + (t - window_r);
      frame_idx = std::min(std::max(frame_idx, 0), total_frames - 1);
      int sample_base_idx = t * vmp_config_.in_c;
      int task_base_idx = frame_idx * vmp_config_.in_c;

      for (int f = 0; f < vmp_config_.in_c; ++f) {
        sample_batch[sample_base_idx + f] = vmp_task_data_[task_base_idx + f];
      }
    }

    // 应用时序归一化（yaw角归一化）
    applyTemporalNormalization(sample_batch.data(), sample_batch.size());

    // 转置数据: (window_l, in_c) -> (in_c, window_l)，适配编码器输入格式
    for (int f = 0; f < vmp_config_.in_c; ++f) {
      for (int t = 0; t < vmp_config_.window_l; ++t) {
        int src_idx = 0 * (vmp_config_.window_l * vmp_config_.in_c) + t * vmp_config_.in_c + f;
        int dst_idx = 0 * (vmp_config_.in_c * vmp_config_.window_l) + f * vmp_config_.window_l + t;
        input_data[dst_idx] = sample_batch[src_idx];
      }
    }

    // 执行编码器推理
    vmp_encoder_request_.infer();

    auto encoder_output = vmp_encoder_request_.get_output_tensor(0);
    const float* encoder_data = encoder_output.data<float>();

    // 准备策略网络输入: [observation, ref_motion, latent_code]
    int expected_policy_input_size = observation.size() + numRefMotionObs_ + vmp_config_.latent_d;
    Eigen::VectorXd policy_input(expected_policy_input_size);

    // 1. 复制观测数据
    int idx = 0;
    policy_input.segment(idx, observation.size()) = observation;
    idx += observation.size();

    // 2. 复制参考运动数据
    if (vmp_ref_motion_buffer_.size() > 0) {
      const auto& current_ref_motion = vmp_ref_motion_buffer_.back();
      int copy_size = std::min(numRefMotionObs_, static_cast<int>(current_ref_motion.size()));
      policy_input.segment(idx, copy_size) = current_ref_motion.head(copy_size);
      if (copy_size < numRefMotionObs_) {
        policy_input.segment(idx + copy_size, numRefMotionObs_ - copy_size).setZero();
      }
    } else {
      policy_input.segment(idx, numRefMotionObs_).setZero();
    }
    idx += numRefMotionObs_;

    // 发布参考运动数据用于调试
    if (ros_logger_ && !vmp_ref_motion_buffer_.empty()) {
      ros_logger_->publishVector("vmp_ref_motion", vmp_ref_motion_buffer_.back());
    }

    // 3. 复制编码器潜在向量
    for (int i = 0; i < vmp_config_.latent_d && i < static_cast<int>(encoder_output.get_size()); ++i) {
      policy_input[idx + i] = static_cast<double>(encoder_data[i]);
    }
    idx += vmp_config_.latent_d;

    // 大小检查
    if (policy_input.size() != expected_policy_input_size) {
      ROS_WARN("[VMP] Warning: Policy input size mismatch! Expected %d, got %ld",
               expected_policy_input_size, policy_input.size());
    }

    // 转换为 OpenVINO Tensor
    ov::Shape policy_input_shape = {1, static_cast<size_t>(policy_input.size())};
    ov::Tensor policy_input_tensor(ov::element::f32, policy_input_shape);
    float* policy_input_data = policy_input_tensor.data<float>();

    for (int i = 0; i < policy_input.size() && i < static_cast<int>(policy_input_tensor.get_size()); ++i) {
      policy_input_data[i] = static_cast<float>(policy_input[i]);
    }

    // 执行策略网络推理
    vmp_policy_request_.set_input_tensor(0, policy_input_tensor);
    vmp_policy_request_.infer();

    auto policy_output = vmp_policy_request_.get_output_tensor(0);
    const float* output_data = policy_output.data<float>();

    // 提取动作输出
    Eigen::VectorXd action(num_actions_);
    for (int i = 0; i < num_actions_; ++i) {
      action[i] = static_cast<double>(output_data[i]);
    }

    // 裁剪动作
    clip(action, clipActions_);

    // 帧计数由 updateVMPReferenceMotion() 统一管理 trajectoryFrameCounter_
    return action;

  } catch (const std::exception& e) {
    ROS_ERROR("[VMPController] Error in computeVMPAction: %s", e.what());
    throw;
  }
}

//==========================================================================
// Trajectory Loading Functions
//==========================================================================

bool VMPController::loadAllTrajectories()
{
  if (trajectories_.empty()) {
    ROS_INFO("[VMP] No trajectories to load");
    return false;
  }

  ROS_INFO("[VMP] Starting to load %zu trajectories...", trajectories_.size());

  bool allSuccess = true;
  size_t totalMemory = 0;

  for (size_t i = 0; i < trajectories_.size(); ++i) {
    ROS_INFO("[VMP] Loading trajectory %zu/%zu: %s",
             i + 1, trajectories_.size(), trajectories_[i].name.c_str());

    if (loadTrajectory(i)) {
      // 处理轨迹（添加静止帧）
      processTrajectory(i);

      size_t trajectoryMemory = trajectories_[i].processed_data.size() * sizeof(float);
      totalMemory += trajectoryMemory;

      ROS_INFO("[VMP] Trajectory %zu loaded: %zu original frames, %zu total frames, %zu KB",
               i + 1, trajectories_[i].original_frames,
               trajectories_[i].total_frames, trajectoryMemory / 1024);
    } else {
      ROS_ERROR("[VMP] Failed to load trajectory %zu", i + 1);
      allSuccess = false;
    }
  }

  if (allSuccess) {
    ROS_INFO("[VMP] All trajectories loaded successfully!");
    ROS_INFO("[VMP] Total memory usage: %zu MB", totalMemory / 1024 / 1024);
  } else {
    ROS_WARN("[VMP] Some trajectories failed to load");
  }

  return allSuccess;
}

bool VMPController::loadTrajectory(size_t index)
{
  if (index >= trajectories_.size()) {
    ROS_ERROR("[VMP] Invalid trajectory index: %zu", index);
    return false;
  }

  auto& trajectory = trajectories_[index];
  std::string task_data_file = vmpRefDataDir_ + "/" + trajectory.data_file;

  // Load task data (total_frames calculated from file size)
  int total_frames = 0;
  if (!vmp::loadTaskData(task_data_file, vmp_config_.in_c, trajectory.task_data, total_frames)) {
    return false;
  }

  trajectory.original_frames = total_frames;
  trajectory.is_loaded = true;
  ROS_INFO("[VMP] Loaded trajectory %zu: %d frames", index, total_frames);
  return true;
}

void VMPController::processTrajectory(size_t index)
{
  if (index >= trajectories_.size() || !trajectories_[index].is_loaded) {
    return;
  }

  auto& trajectory = trajectories_[index];

  if (vmp_pre_standing_frames_ == 0 && vmp_post_standing_frames_ == 0) {
    // No standing frames needed, just copy
    trajectory.processed_data = trajectory.task_data;
    trajectory.total_frames = trajectory.original_frames;
    trajectory.is_processed = true;
    return;
  }

  int original_total_frames = trajectory.original_frames;
  int new_total_frames = original_total_frames + vmp_pre_standing_frames_ + vmp_post_standing_frames_;

  trajectory.processed_data.resize(new_total_frames * vmp_config_.in_c);

  // Get first and last frames for interpolation
  std::vector<float> start_real(vmp_config_.in_c);
  std::vector<float> end_real(vmp_config_.in_c);

  for (int j = 0; j < vmp_config_.in_c; ++j) {
    start_real[j] = trajectory.task_data[j];
    end_real[j] = trajectory.task_data[(original_total_frames - 1) * vmp_config_.in_c + j];
  }

  // Pre-standing frames with interpolation
  for (int i = 0; i < vmp_pre_standing_frames_; ++i) {
    int start_idx = i * vmp_config_.in_c;
    for (int j = 0; j < vmp_config_.in_c; ++j) {
      trajectory.processed_data[start_idx + j] = vmp_standing_frame_[j];
    }

    if (i >= vmp_pre_standing_frames_ - vmp_pre_interpolation_frames_) {
      float alpha = float(i - (vmp_pre_standing_frames_ - vmp_pre_interpolation_frames_) + 1) /
                    float(vmp_pre_interpolation_frames_ + 1);
      for (int j = vmp_config_.q_start_id; j < vmp_config_.q_end_id; ++j) {
        trajectory.processed_data[start_idx + j] =
            (1.0f - alpha) * vmp_standing_frame_[j] + alpha * start_real[j];
      }
    }
  }

  // Copy original data
  int original_start_idx = vmp_pre_standing_frames_ * vmp_config_.in_c;
  std::copy(trajectory.task_data.begin(), trajectory.task_data.end(),
            trajectory.processed_data.begin() + original_start_idx);

  // Post-standing frames with interpolation
  int post_start_idx = (vmp_pre_standing_frames_ + original_total_frames) * vmp_config_.in_c;
  for (int i = 0; i < vmp_post_standing_frames_; ++i) {
    int start_idx = post_start_idx + i * vmp_config_.in_c;
    for (int j = 0; j < vmp_config_.in_c; ++j) {
      trajectory.processed_data[start_idx + j] = vmp_standing_frame_[j];
    }

    if (i < vmp_post_interpolation_frames_) {
      float alpha = float(vmp_post_interpolation_frames_ - i) / float(vmp_post_interpolation_frames_ + 1);
      for (int j = vmp_config_.q_start_id; j < vmp_config_.q_end_id; ++j) {
        trajectory.processed_data[start_idx + j] =
            alpha * end_real[j] + (1.0f - alpha) * vmp_standing_frame_[j];
      }
    }
  }

  trajectory.total_frames = new_total_frames;
  trajectory.is_processed = true;
}

bool VMPController::switchToTrajectory(size_t index)
{
  if (index >= trajectories_.size()) {
    ROS_ERROR("[VMP] Invalid trajectory index: %zu", index);
    return false;
  }

  const auto& trajectory = trajectories_[index];

  if (!trajectory.is_loaded || !trajectory.is_processed) {
    ROS_ERROR("[VMP] Trajectory %zu not ready (loaded=%d, processed=%d)",
              index, trajectory.is_loaded, trajectory.is_processed);
    return false;
  }

  // Copy processed data to active task data
  vmp_task_data_ = trajectory.processed_data;
  currentTrajectoryIndex_ = index;

  // Reset frame counter（保持 trajectoryPlaybackCompleted_ 不变，由调用者决定是否开始播放）
  trajectoryFrameCounter_ = 0;
  // 注意：不在这里修改 trajectoryPlaybackCompleted_，由 executeTrajectoryServiceCallback 控制

  // 重置关节命令滤波器（与 kuavo-rl 保持一致，避免滤波器状态导致的不稳定）
  jointCmdFilter_.reset();

  // Reinitialize reference motion buffer
  vmp_ref_motion_buffer_.clear();
  if (!vmp_task_data_.empty()) {
    Eigen::VectorXd first_frame(vmp_config_.in_c);
    for (int i = 0; i < vmp_config_.in_c; ++i) {
      first_frame[i] = static_cast<double>(vmp_task_data_[i]);
    }
    // 应用 yaw 归一化（如果启用），确保与 updateVMPReferenceMotion 中的处理一致
    if (vmp_enable_theta_normalization_) {
      vmp::rotation_utils::normalize_ref_motion_yaw(first_frame, vmp_config_.theta_start_id, vmp_config_.theta_end_id);
    }
    for (int i = 0; i < vmp_config_.window_l; ++i) {
      vmp_ref_motion_buffer_.push_back(first_frame);
    }
  }

  // 打印轨迹菜单
  printTrajectoryMenu();

  return true;
}

size_t VMPController::getMemoryUsage() const
{
  size_t total = 0;
  for (const auto& trajectory : trajectories_) {
    total += trajectory.task_data.size() * sizeof(float);
    total += trajectory.processed_data.size() * sizeof(float);
  }
  return total;
}

void VMPController::printTrajectoryMenu() const
{
  // ANSI 颜色码
  const char* COLOR_GREEN = "\033[32m";
  const char* COLOR_BOLD_GREEN = "\033[1;32m";
  const char* COLOR_RESET = "\033[0m";

  std::cout << "-----------------------------------------------------\n"
    << "[VMP] Trajectory Menu:" << std::endl;

  // 从预加载数据获取帧数
  for (size_t i = 0; i < trajectories_.size(); ++i) {
    const auto& traj = trajectories_[i];
    bool is_current = (static_cast<int>(i) == currentTrajectoryIndex_);

    if (is_current) {
      std::cout << COLOR_BOLD_GREEN << "  " << (i + 1) << ". " << traj.name
                << " [" << (traj.is_loaded ? "L" : "-") << (traj.is_processed ? "P" : "-") << "]"
                << " (" << traj.total_frames << " frames) <- CURRENT"
                << COLOR_RESET << std::endl;
    } else {
      std::cout << "  " << (i + 1) << ". " << traj.name
                << " [" << (traj.is_loaded ? "L" : "-") << (traj.is_processed ? "P" : "-") << "]"
                << " (" << traj.total_frames << " frames)" << std::endl;
    }
  }
  std::cout << "-----------------------------------------------------" << std::endl;
}

void VMPController::printMultiTrajectoryStatus() const
{
  ROS_INFO("========== Trajectory Status ==========");
  ROS_INFO("[VMP] Trajectory count: %zu", trajectories_.size());
  ROS_INFO("[VMP] Current trajectory: %d/%zu", currentTrajectoryIndex_ + 1, trajectories_.size());
  ROS_INFO("[VMP] Current trajectory: %s",
           currentTrajectoryIndex_ < static_cast<int>(trajectories_.size()) ?
           trajectories_[currentTrajectoryIndex_].name.c_str() : "N/A");
  ROS_INFO("[VMP] Frame counter: %d", trajectoryFrameCounter_);

  // 打印轨迹菜单
  printTrajectoryMenu();

  size_t memoryUsage = getMemoryUsage();
  ROS_INFO("[VMP] Memory usage: %zu MB", memoryUsage / 1024 / 1024);
  ROS_INFO("=============================================");
}

//==========================================================================
// ROS Interface Implementation
//==========================================================================

void VMPController::initROSServices()
{
  // 服务命名空间为 /humanoid_controllers/{controller_name}
  std::string service_ns = "/humanoid_controllers/" + name_;

  // 初始化服务
  srv_get_trajectory_list_ = nh_.advertiseService(
      service_ns + "/trajectory/list",
      &VMPController::getTrajectoryListServiceCallback, this);

  srv_execute_trajectory_ = nh_.advertiseService(
      service_ns + "/trajectory/execute",
      &VMPController::executeTrajectoryServiceCallback, this);

  srv_stop_trajectory_ = nh_.advertiseService(
      service_ns + "/trajectory/stop",
      &VMPController::stopTrajectoryServiceCallback, this);

  // 初始化发布者
  pub_trajectory_state_ = nh_.advertise<kuavo_msgs::VMPTrajectoryState>(
      service_ns + "/trajectory/state", 10);

  // 创建定时器: 50Hz = 0.02秒
  trajectory_state_timer_ = nh_.createTimer(
      ros::Duration(0.02),
      &VMPController::trajectoryStateTimerCallback,
      this);


  ROS_INFO("[%s] ROS services initialized:", name_.c_str());
  ROS_INFO("   - %s/trajectory/list", service_ns.c_str());
  ROS_INFO("   - %s/trajectory/execute", service_ns.c_str());
  ROS_INFO("   - %s/trajectory/stop", service_ns.c_str());
  ROS_INFO("   - %s/trajectory/state (50Hz timer)", service_ns.c_str());
}

void VMPController::trajectoryStateTimerCallback(const ros::TimerEvent& event)
{
  if (!pub_trajectory_state_.getNumSubscribers()) {
    return;
  }

  kuavo_msgs::VMPTrajectoryState msg;
  msg.header.stamp = event.current_real;

  // 当前轨迹索引
  msg.current_index = currentTrajectoryIndex_;

  // 当前轨迹名称
  if (currentTrajectoryIndex_ >= 0 &&
      currentTrajectoryIndex_ < static_cast<int>(trajectories_.size())) {
    msg.current_name = trajectories_[currentTrajectoryIndex_].name;
  } else {
    msg.current_name = "";
  }

  // 帧信息
  int total_frames = static_cast<int>(vmp_task_data_.size() / vmp_config_.in_c);
  msg.current_frame = trajectoryFrameCounter_;
  msg.total_frames = total_frames;

  // 进度百分比
  if (total_frames > 0) {
    msg.progress = static_cast<float>(trajectoryFrameCounter_) / static_cast<float>(total_frames);
  } else {
    msg.progress = 0.0f;
  }

  // 播放完成状态
  msg.completed = trajectoryPlaybackCompleted_;

  pub_trajectory_state_.publish(msg);
}

bool VMPController::getTrajectoryListServiceCallback(
    kuavo_msgs::GetStringList::Request& req,
    kuavo_msgs::GetStringList::Response& res)
{
  res.data.clear();

  for (const auto& config : trajectories_) {
    res.data.push_back(config.name);
  }

  res.success = true;
  res.message = "Retrieved " + std::to_string(trajectories_.size()) + " trajectory/trajectories";

  ROS_INFO("[%s] Get trajectory list: %zu trajectories", name_.c_str(), trajectories_.size());
  return true;
}

bool VMPController::executeTrajectoryServiceCallback(
    kuavo_msgs::SetString::Request& req,
    kuavo_msgs::SetString::Response& res)
{
  const std::string& target_name = req.data;

  // 检查控制器是否在运行中
  if (state_ != ControllerState::RUNNING) {
    res.success = false;
    res.message = "Controller not running (state=" + std::to_string(static_cast<int>(state_)) + ")";
    ROS_WARN("[%s] Execute trajectory failed: %s", name_.c_str(), res.message.c_str());
    return true;
  }

  // 检查当前轨迹是否还在执行中
  if (!trajectoryPlaybackCompleted_) {
    res.success = false;
    res.message = "Current trajectory '" + trajectories_[currentTrajectoryIndex_].name + "' is still executing";
    ROS_WARN("[%s] Execute trajectory failed: %s", name_.c_str(), res.message.c_str());
    return true;
  }

  // 查找匹配的轨迹
  int target_index = -1;
  for (size_t i = 0; i < trajectories_.size(); ++i) {
    if (trajectories_[i].name == target_name) {
      target_index = static_cast<int>(i);
      break;
    }
  }

  if (target_index < 0) {
    res.success = false;
    res.message = "Trajectory '" + target_name + "' not found";
    ROS_WARN("[%s] Execute trajectory failed: %s", name_.c_str(), res.message.c_str());
    return true;
  }

  // 执行目标轨迹
  try {
    if (!switchToTrajectory(target_index)) {
      res.success = false;
      res.message = "Failed to switch to trajectory '" + target_name + "'";
      return true;
    }

    // 重置播放状态
    playback_state_ = TrajectoryPlaybackState::PLAYING;
    trajectoryPlaybackCompleted_ = false;

    res.success = true;
    res.message = "Executing trajectory '" + target_name + "'";

    ROS_INFO("[%s] %s", name_.c_str(), res.message.c_str());

  } catch (const std::exception& e) {
    res.success = false;
    res.message = "Failed to execute trajectory: " + std::string(e.what());
    ROS_ERROR("[%s] %s", name_.c_str(), res.message.c_str());
  }

  return true;
}

bool VMPController::stopTrajectoryServiceCallback(
    std_srvs::Trigger::Request& req,
    std_srvs::Trigger::Response& res)
{
  // TODO: 实现停止轨迹的逻辑
  res.success = false;
  res.message = "Not implemented";
  return true;
}

} // namespace humanoid_controller
