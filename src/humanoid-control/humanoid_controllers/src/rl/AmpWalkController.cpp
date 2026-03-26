// Pinocchio must be included before Boost headers
#include <pinocchio/fwd.hpp>

#include "humanoid_controllers/rl/AmpWalkController.h"
#include <ocs2_core/misc/LoadData.h>
#include <ocs2_robotic_tools/common/RotationTransforms.h>
#include <angles/angles.h>
#include "kuavo_common/common/common.h"
#include <ros/package.h>
#include <cmath>
#include <thread>

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
    
    // 初始化 RL IMU 滤波（内部用 accFilterRL_ 等）
    loadRLFilterParams(config_file_);

    if (!loadConfig(config_file_))
    {
      ROS_ERROR("[%s] loadConfig failed", name_.c_str());
      return false;
    }

    // gait 指令来源：使用 RL gait receiver，等价于原来的 CommandData + joystick/cmd_vel
    initial_cmd_.cmdStance_ = 1;
    gait_receiver_ = std::make_unique<RlGaitReceiver>(nh_, &initial_cmd_);
    
    // 加载原地踏步速度配置
    gait_receiver_->loadInPlaceStepConfig(config_file_, false);

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

    // 初始化手臂控制（可选功能）
    // 获取URDF路径
    std::string urdf_path;
    if (nh_.getParam("/urdfFile", urdf_path))
    {
      ROS_INFO("[%s] Using URDF path from ROS param /urdfFile: %s", name_.c_str(), urdf_path.c_str());
    }
    else
    {
      // 如果ROS参数中没有，尝试从robot_version构造
      int robot_version_int = 45; // 默认版本
      nh_.param("/robot_version", robot_version_int, 45);
      int major = robot_version_int / 10;
      int minor = robot_version_int % 10;
      std::string package_path = ros::package::getPath("kuavo_assets");
      std::string version_str = "biped_s" + std::to_string(major) + std::to_string(minor);
      urdf_path = package_path + "/models/" + version_str + "/urdf/" + version_str + ".urdf";
      ROS_INFO("[%s] Constructed URDF path from robot_version: %s", name_.c_str(), urdf_path.c_str());
    }
    
    initArmControl(urdf_path);
    initWaistControl();

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
    Eigen::VectorXd jointCmdFilterCutoffFreq_(jointNum_ + jointArmNum_ + waistNum_);

    loadEigenMatrix("defaultJointState", defalutJointPosRL_);
    loadEigenMatrix("defaultBaseState", defaultBaseStateRL_);
    loadEigenMatrix("JointControlMode", JointControlModeRL_);
    loadEigenMatrix("JointPDMode", JointPDModeRL_);
    loadEigenMatrix("jointKp", jointKpRL_);
    loadEigenMatrix("jointKd", jointKdRL_);
    loadEigenMatrix("torqueLimits", torqueLimitsRL_);
    loadEigenMatrix("actionScaleTest", actionScaleTestRL_);
    // 速度限制（8 维：正负方向独立设置）
    // 格式：[linear_x_pos, linear_x_neg, linear_y_pos, linear_y_neg, 
    //        linear_z_pos, linear_z_neg, angular_z_pos, angular_z_neg]
    loadEigenMatrix("velocityLimits", velocityLimits_);
    loadEigenMatrix("jointCmdFilterCutoffFreq", jointCmdFilterCutoffFreq_);

    jointCmdFilter_.setParams(dt_, jointCmdFilterCutoffFreq_);
    jointCmdFilterState_.resize(jointCmdFilterCutoffFreq_.size());
    jointCmdFilterState_.setZero();
    loadEigenMatrix("jointCmdFilterState", jointCmdFilterState_);

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
    // 可选: RL 站立时 base 在 x 方向相对于足端中心(0)的偏移, 若配置文件未提供则保持默认 0
    try
    {
      loadData::loadCppDataType(rlParamFile, "defaultBaseXOffsetControl", defaultBaseXOffsetControl_);
    }
    catch (const std::exception& e)
    {
      ROS_WARN("[%s] defaultBaseXOffsetControl not found in ROS params, using default: %f", name_.c_str(), defaultBaseXOffsetControl_);
    }

    // 是否使用关节指令滤波（对应 skw_rl_param.info 中 use_jointcmd_filter）
    loadData::loadPtreeValue(pt, use_jointcmd_filter_, "use_jointcmd_filter", true);


    // 是否启用手臂指令替换功能（对应 skw_rl_param.info 中 setArmCommandReplacementEnabled）
    bool arm_command_replacement_enabled = false;
    loadData::loadPtreeValue(pt, arm_command_replacement_enabled, "use_external_arm_controller", false);
    use_external_arm_controller(arm_command_replacement_enabled);
    ROS_INFO("[%s] Arm command replacement enabled: %s", name_.c_str(), arm_command_replacement_enabled ? "true" : "false");

    // 加载手臂控制参数（用于 ArmController）
    if (arm_command_replacement_enabled && jointArmNum_ > 0)
    {
      loadData::loadPtreeValue(pt, arm_max_tracking_velocity_, "armVelocityLimit.maxTrackingVelocity", false);
      loadData::loadPtreeValue(pt, arm_tracking_error_threshold_, "armVelocityLimit.trackingErrorThreshold", false);
      loadData::loadPtreeValue(pt, arm_mode_interpolation_velocity_, "armVelocityLimit.modeInterpolationVelocity", false);
      
      ROS_INFO("[%s] Arm control parameters loaded: max_velocity=%.3f rad/s, error_threshold=%.3f rad, mode_interpolation_velocity=%.3f rad/s",
               name_.c_str(), arm_max_tracking_velocity_, arm_tracking_error_threshold_, arm_mode_interpolation_velocity_);
    }

    // 是否启用腰部控制覆盖功能（对应 skw_rl_param.info 中 use_external_waist_controller）
    bool waist_command_replacement_enabled = false;
    loadData::loadPtreeValue(pt, waist_command_replacement_enabled, "use_external_waist_controller", false);
    use_external_waist_controller(waist_command_replacement_enabled);
    ROS_INFO("[%s] Waist command replacement enabled: %s", name_.c_str(), waist_command_replacement_enabled ? "true" : "false");
    
    // 加载腰部控制参数（用于 WaistController）
    if (waist_command_replacement_enabled && waistNum_ > 0)
    {
      loadData::loadPtreeValue(pt, waist_mode_interpolation_velocity_, "waistControllerParam.modeInterpolationVelocity", false);
      loadData::loadPtreeValue(pt, waist_mode2_cutoff_freq_, "waistControllerParam.mode2CutoffFreq", false);
      
      // 读取 kp 和 kd，如果未指定则使用默认值
      double waist_kp_default = 10.0;
      double waist_kd_default = 2.0;
      loadData::loadPtreeValue(pt, waist_kp_default, "waistControllerParam.kp", false);
      loadData::loadPtreeValue(pt, waist_kd_default, "waistControllerParam.kd", false);
      
      // 将标量值转换为向量（所有腰部关节使用相同的 kp 和 kd）
      waist_kp_from_config_ = Eigen::VectorXd::Constant(waistNum_, waist_kp_default);
      waist_kd_from_config_ = Eigen::VectorXd::Constant(waistNum_, waist_kd_default);
      
      ROS_INFO("[%s] Waist control parameters loaded: mode_interpolation_velocity=%.3f rad/s, mode2_cutoff_freq=%.1f Hz, kp=%.1f, kd=%.1f",
               name_.c_str(), waist_mode_interpolation_velocity_, waist_mode2_cutoff_freq_, waist_kp_default, waist_kd_default);
    }
    
    // 是否启用行走时腰部0位跟踪功能（忽略RL输出的腰部action，直接跟踪默认位置）
    loadData::loadPtreeValue(pt, waist_zero_tracking_enabled_, "waistZeroTrackingEnabled", false);
    ROS_INFO("[%s] Waist zero tracking in walking enabled: %s", name_.c_str(), waist_zero_tracking_enabled_ ? "true" : "false");

    // 加载站立切换到行走时的支撑腿髋关节roll偏置参数
    loadData::loadPtreeValue(pt, stanceToWalkHipRollBias_, "stanceToWalkHipRollBias", false);
    loadData::loadPtreeValue(pt, stanceToWalkBiasDuration_, "stanceToWalkBiasDuration", false);
    
    ROS_INFO("[%s] Stance-to-walk hip roll bias parameters: bias=%.4f rad, duration=%.4f s",
             name_.c_str(), stanceToWalkHipRollBias_, stanceToWalkBiasDuration_);

    // 加载YAW补偿参数
    if (pt.find("yawCompensation") != pt.not_found()) {
      loadData::loadPtreeValue(pt, yaw_compensation_enabled_, "yawCompensation.enabled", false);
      loadData::loadPtreeValue(pt, yaw_compensation_x_bias_, "yawCompensation.xBias", false);
      loadData::loadPtreeValue(pt, yaw_compensation_threshold_, "yawCompensation.threshold", false);
      loadData::loadPtreeValue(pt, yaw_compensation_x_velocity_threshold_, "yawCompensation.xVelocityThreshold", false);
      loadData::loadPtreeValue(pt, yaw_compensation_separate_enabled_, "yawCompensation.enableSeparateCompensation", false);
      loadData::loadPtreeValue(pt, yaw_compensation_x_bias_clockwise_, "yawCompensation.xBiasClockwise", false);
      loadData::loadPtreeValue(pt, yaw_compensation_x_bias_counterclockwise_, "yawCompensation.xBiasCounterclockwise", false);
      
      ROS_INFO("[%s] YAW compensation loaded: enabled=%s, xBias=%.4f, threshold=%.4f, xVelThreshold=%.4f, separate=%s",
               name_.c_str(), 
               yaw_compensation_enabled_ ? "true" : "false",
               yaw_compensation_x_bias_,
               yaw_compensation_threshold_,
               yaw_compensation_x_velocity_threshold_,
               yaw_compensation_separate_enabled_ ? "true" : "false");
      if (yaw_compensation_enabled_ && yaw_compensation_separate_enabled_) {
        ROS_INFO("[%s] YAW separate compensation: clockwise=%.4f, counterclockwise=%.4f",
                 name_.c_str(), yaw_compensation_x_bias_clockwise_, yaw_compensation_x_bias_counterclockwise_);
      }
    } else {
      ROS_INFO("[%s] YAW compensation not found in config, using defaults (disabled)", name_.c_str());
    }

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

    // 初始化髋关节pitch角度索引
    // 对于非roban机型：leg_l3_joint=2, leg_r3_joint=8
    // 对于roban机型：leg_l3_joint=waistNum_+2, leg_r3_joint=waistNum_+8
    leftHipPitchIdx_ = is_roban_ ? (waistNum_ + 2) : 2;
    rightHipPitchIdx_ = is_roban_ ? (waistNum_ + 8) : 8;

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

    // 加载 X 负向单独缩放系数（用于不对称速度限制）
    loadData::loadPtreeValue(pt, cmdVelLineXNegScale_, "commandData.scale.cmdVelLineXNegScale", false);

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
    
    // 重置手臂控制器状态
    if (arm_controller_)
    {
      arm_controller_->reset();
    }
    
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

    // Ruiwo 手臂增益已通过 joint_cmd 实时下发，无需单独调用 ROS 服务切换
  }
  void AmpWalkController::resume()
  {
    RLControllerBase::resume();
    if (gait_receiver_)
    {
      gait_receiver_->setEnabled(true);
    }

    // Ruiwo 手臂增益已通过 motor_pdo_kp/kd（skw_rl_param.info）在 joint_cmd 中实时下发

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

  bool AmpWalkController::isInStanceMode() const
  {
    if (!gait_receiver_)
    {
      // 没有 gait_receiver_，默认返回 true（stance）
      return true;
    }
    
    auto cmd = gait_receiver_->getCurrentCommand();
    // cmdStance_ == 1 表示 stance 模式，== 0 表示行走模式
    return cmd.cmdStance_ >= 0.5;  // 使用 0.5 作为阈值，兼容浮点数比较
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
    // 根据速度方向选择正负限制：速度>0 用正向限制，速度<0 用负向限制
    double velLimitX = (cmd.cmdVelLineX_ >= 0) ? velocityLimits_(0) : velocityLimits_(1);
    double ratio = cmd.cmdVelLineX_ / velLimitX;
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
      double current_yaw = std::atan2(mat(1, 0), mat(0, 0));
      my_yaw_offset_ = 0.0 - current_yaw;
      // 归一化到[-π, π]范围
      while (my_yaw_offset_ > M_PI) my_yaw_offset_ -= 2 * M_PI;
      while (my_yaw_offset_ < -M_PI) my_yaw_offset_ += 2 * M_PI;
      yaw_offset_initialized = true;
      ROS_INFO("[%s] Initialized yaw_offset: %.6f (current_yaw: %.6f)", name_.c_str(), my_yaw_offset_, current_yaw);
    }

    // 速度命令 [vx, vy, omega_z]
    cmd.scale();
    
    // 应用 X 负向单独缩放系数（实现不对称速度限制）
    if (cmd.cmdVelLineX_ < 0.0) {
      cmd.cmdVelLineX_ *= cmdVelLineXNegScale_;
    }
    
    Eigen::Vector3d velocity_commands;
    velocity_commands << cmd.cmdVelLineX_,
                         cmd.cmdVelLineY_,
                         cmd.cmdVelAngularZ_;
        
    // 应用 YAW 补偿（当旋转时给 X 方向速度添加偏置）
    if (yaw_compensation_enabled_) {
      double angular_z = velocity_commands(2);  // YAW角速度
      double linear_x = velocity_commands(0);   // X方向线速度
      
      // 检查是否满足补偿条件：|角速度| > 阈值 且 |线速度| < 阈值
      if (std::abs(angular_z) > yaw_compensation_threshold_ && 
          std::abs(linear_x) < yaw_compensation_x_velocity_threshold_) {
        
        double x_bias = 0.0;
        if (yaw_compensation_separate_enabled_) {
          // 根据旋转方向使用不同的偏置值
          // angular_z > 0: 逆时针旋转（CCW），angular_z < 0: 顺时针旋转（CW）
          if (angular_z > 0) {
            x_bias = yaw_compensation_x_bias_counterclockwise_;
          } else {
            x_bias = yaw_compensation_x_bias_clockwise_;
          }
        } else {
          // 使用通用偏置值
          x_bias = yaw_compensation_x_bias_;
        }
        
        // 应用偏置到X方向速度（同时修改velocity_commands和cmd，确保一致性）
        velocity_commands(0) += x_bias;
        cmd.cmdVelLineX_ += x_bias;  // 关键：修改cmd对象，确保getCommandRL()返回补偿后的值
        
        // std::cout << "[" << name_ << "] YAW compensation applied: angular_z=" << angular_z
        //           << ", linear_x=" << linear_x << " -> " << velocity_commands(0)
        //           << " (bias=" << x_bias << ")" << std::endl;
      }
    }
    
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
      ROS_ERROR("[%s] singleInputDataKeys_ is empty, cannot build observation", name_.c_str());
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

      // ==================== 站立切换到行走时的支撑腿髋关节roll偏置 ====================
      // 计算并应用支撑腿髋关节roll偏置
      if (isStanceToWalkBiasActive_)
      {
        double elapsed = (ros::Time::now() - stanceToWalkBiasStartTime_).toSec();
        if (elapsed >= stanceToWalkBiasDuration_)
        {
          isStanceToWalkBiasActive_ = false;
        }
        else
        {
          double decay_factor = 1.0 - (elapsed / stanceToWalkBiasDuration_);
          double current_bias = stanceToWalkHipRollBias_ * decay_factor;
          
          // 左腿支撑(-1)：施加正偏置到左腿髋关节roll(索引0)
          // 右腿支撑(1)：施加负偏置到右腿髋关节roll(索引6)
          if (stanceToWalkBiasSupportLeg_ == -1)
          {
            action[0] += current_bias;
          }
          else if (stanceToWalkBiasSupportLeg_ == 1)
          {
            action[6] -= current_bias;
          }
        }
      }
      
      // 获取当前命令数据判断是否从站立切换到行走
      CommandDataRL currentCmdData = gait_receiver_->getCurrentCommand();
      bool is_standing = (currentCmdData.cmdStance_ >= 1.0);
      
      // 当从站立切换到行走时（站立->行走），记录初始髋关节pitch角速度并开始数据收集
      if (lastStanceState_ && !is_standing)
      {
        leftHipPitchVelIntegral_ = 0.0;
        rightHipPitchVelIntegral_ = 0.0;
        stanceToWalkHipPitchCollectionStartTime_ = ros::Time::now();
        isHipPitchDataCollected_ = false;
      }
            
      // 在站立切换到行走后的累积时间窗口内收集髋关节pitch角速度数据并进行积分比较
      if (!lastStanceState_ && !is_standing && !isHipPitchDataCollected_)
      {
        SensorData current_sensor_data = getRobotSensorData();
        double elapsed = (ros::Time::now() - stanceToWalkHipPitchCollectionStartTime_).toSec();
        double currentLeftHipPitchVel = current_sensor_data.jointVel_[leftHipPitchIdx_];
        double currentRightHipPitchVel = current_sensor_data.jointVel_[rightHipPitchIdx_];
              
        if (elapsed < kHipPitchCollectionDuration_)
        {
          // 积分累加：累加pitch角速度
          leftHipPitchVelIntegral_ += currentLeftHipPitchVel;
          rightHipPitchVelIntegral_ += currentRightHipPitchVel;
        }
        else
        {
          // 累积时间结束，比较积分结果判断支撑腿
          // 计算左右髋pitch角速度积分差值：左腿-右腿
          // 差值 > 0：左腿角速度积分更正（抬得少/踩得多）→ 右腿抬起 → 右腿支撑
          // 差值 < 0：右腿角速度积分更负（抬得多/踩得少）→ 左腿抬起 → 左腿支撑
          double hipPitchVelIntegralDiff = - leftHipPitchVelIntegral_ + rightHipPitchVelIntegral_;
                
          ROS_INFO("[SupportLegBias] Hip pitch velocity integral: L=%.6f, R=%.6f, diff=%.6f",
                   leftHipPitchVelIntegral_, rightHipPitchVelIntegral_, hipPitchVelIntegralDiff);
                
          if (hipPitchVelIntegralDiff > 0)
          {
            // 左腿角速度积分 > 右腿 → 右腿抬起 → 右腿支撑
            ROS_INFO("[SupportLegBias] -> Right leg lifting, using RIGHT support");
            stanceToWalkBiasStartTime_ = ros::Time::now();
            isStanceToWalkBiasActive_ = true;
            stanceToWalkBiasSupportLeg_ = 1;
          }
          else if (hipPitchVelIntegralDiff < 0)
          {
            // 右腿角速度积分 < 左腿 → 左腿抬起 → 左腿支撑
            ROS_INFO("[SupportLegBias] -> Left leg lifting, using LEFT support");
            stanceToWalkBiasStartTime_ = ros::Time::now();
            isStanceToWalkBiasActive_ = true;
            stanceToWalkBiasSupportLeg_ = -1;
          }
          else
          {
            // 积分相等 → 默认右腿支撑
            ROS_INFO("[SupportLegBias] -> Equal integral, using default RIGHT support");
            stanceToWalkBiasStartTime_ = ros::Time::now();
            isStanceToWalkBiasActive_ = true;
            stanceToWalkBiasSupportLeg_ = 1;
          }
                
          isHipPitchDataCollected_ = true;
        }
      }
      
      // 更新上一帧状态
      lastStanceState_ = is_standing;

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
    ros_logger_->publishVector("/rl_controller/cmd", cmd);

    // for 4pro AMP 
    Eigen::VectorXd actuation;
    if (use_jointcmd_filter_)
    {
      Eigen::VectorXd cmd_filter = jointCmdFilter_.update(cmd);
      // 将滤波状态向量扩展到完整大小（包括腰部关节）
      Eigen::VectorXd filterState_full = Eigen::VectorXd::Zero(jointNum_ + jointArmNum_ + waistNum_);
      filterState_full.head(jointCmdFilterState_.size()) = jointCmdFilterState_;
      Eigen::VectorXd cmd_out = cmd_filter.cwiseProduct(filterState_full) +
                                cmd.cwiseProduct(Eigen::VectorXd::Ones(jointNum_ + jointArmNum_ + waistNum_) - filterState_full);
      actuation = cmd_out;
    }
    else
    {
      actuation = cmd;
    }
    
    // 与 humanoidController_rl.cpp 保持一致：更新 episodeLength_ 并发布日志
    episodeLength_++;
    if (ros_logger_)
    {
      // ros_logger_->publishVector("/rl_controller/torque", torque);
      ros_logger_->publishVector("/rl_controller/actuation", actuation);
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

    if (is_roban_)
    {
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

  void AmpWalkController::initArmControl(const std::string& urdf_path)
  {
    // 初始化手臂控制器（如果启用了手臂指令替换功能）
    if (arm_command_replacement_enabled_ && jointArmNum_ > 0)
    {
      try
      {
        // 创建 ArmController 实例（关节排序：腿 + 腰 + 手）
        // 注意：AmpWalkController 中 jointNum_ 是腿部关节数，waistNum_ 是腰部关节数，jointArmNum_ 是手臂关节数
        arm_controller_ = std::make_unique<ArmController>(
          nh_, 
          jointNum_,      // 腿部关节数量
          waistNum_,      // 腰部关节数量
          jointArmNum_,   // 手臂关节数量
          ros_logger_     // ROS日志发布器
        );
        
        // 初始化 ArmController
        // 提取手臂部分的 kp 和 kd 参数
        Eigen::VectorXd arm_kp, arm_kd;
        if (is_roban_)
        {
          // roban 机型：手臂在 waistNum_ + jointNum_ 开始
          arm_kp = jointKpRL_.segment(waistNum_ + jointNum_, jointArmNum_);
          arm_kd = jointKdRL_.segment(waistNum_ + jointNum_, jointArmNum_);
        }
        else
        {
          // 非 roban 机型：手臂在 jointNum_ + waistNum_ 开始
          arm_kp = jointKpRL_.segment(jointNum_ + waistNum_, jointArmNum_);
          arm_kd = jointKdRL_.segment(jointNum_ + waistNum_, jointArmNum_);
        }
        
        if (!arm_controller_->initialize(urdf_path, arm_kp, arm_kd))
        {
          ROS_ERROR("[%s] Failed to initialize arm controller", name_.c_str());
          arm_command_replacement_enabled_ = false;
          arm_controller_.reset();
          return;
        }
        
        // 获取默认手臂位置
        Eigen::VectorXd default_arm_pos;
        if (defalutJointPosRL_.size() >= jointNum_ + waistNum_ + jointArmNum_)
        {
          default_arm_pos = defalutJointPosRL_.segment(jointNum_ + waistNum_, jointArmNum_);
        }
        else
        {
          default_arm_pos = Eigen::VectorXd::Zero(jointArmNum_);
          ROS_WARN("[%s] Cannot get default arm position, using zero vector", name_.c_str());
        }
        
        // 加载配置参数（从成员变量中获取，这些参数在 loadConfig 中已加载）
        arm_controller_->loadSettings(arm_max_tracking_velocity_, arm_tracking_error_threshold_,
                                     arm_mode_interpolation_velocity_, default_arm_pos);
        
        ROS_INFO("[%s] Arm controller initialized (arm_command_replacement_enabled=true, urdf_path=%s)", 
                 name_.c_str(), urdf_path.c_str());
      }
      catch (const std::exception& e)
      {
        ROS_ERROR("[%s] Failed to initialize arm controller: %s", name_.c_str(), e.what());
        arm_command_replacement_enabled_ = false;
        arm_controller_.reset();
      }
    }
    else
    {
      ROS_INFO("[%s] Arm command replacement disabled or no arm joints", name_.c_str());
    }
  }

  bool AmpWalkController::updateArmCommand(const ros::Time& time,
                                           const SensorData& sensor_data,
                                           kuavo_msgs::jointCmd& joint_cmd)
  {
    // 如果未启用手臂指令替换或没有手臂关节，直接返回false
    if (!arm_command_replacement_enabled_ || jointArmNum_ == 0 || !arm_controller_)
    {
      return false;
    }

    // 获取控制周期
    double dt = dt_;
    if (dt <= 0.0 || dt > 0.1) dt = 0.002;  // 默认2ms

    // 获取当前命令数据
    CommandDataRL cmdData;
    if (gait_receiver_)
    {
      cmdData = gait_receiver_->getCurrentCommand();
    }

    // 构建完整的关节位置和速度向量（腿 + 腰 + 手）
    // 注意：ArmController 期望的顺序是：腿 + 腰 + 手
    // 对于 roban 机型，preprocessSensorData 已将顺序调整为：腰 + 腿 + 手
    // 所以需要重新排列为：腿 + 腰 + 手
    Eigen::VectorXd full_joint_pos(jointNum_ + waistNum_ + jointArmNum_);
    Eigen::VectorXd full_joint_vel(jointNum_ + waistNum_ + jointArmNum_);

    if (is_roban_)
    {
      // roban 机型：sensor_data 顺序是 腰 + 腿 + 手，需要调整为 腿 + 腰 + 手
      // 腰：索引 0 到 waistNum_-1
      // 腿：索引 waistNum_ 到 waistNum_+jointNum_-1
      // 手：索引 waistNum_+jointNum_ 到 waistNum_+jointNum_+jointArmNum_-1
      
      // 提取各部分
      Eigen::VectorXd waist_pos = sensor_data.jointPos_.segment(0, waistNum_);
      Eigen::VectorXd leg_pos = sensor_data.jointPos_.segment(waistNum_, jointNum_);
      Eigen::VectorXd arm_pos = sensor_data.jointPos_.segment(waistNum_ + jointNum_, jointArmNum_);
      
      Eigen::VectorXd waist_vel = sensor_data.jointVel_.segment(0, waistNum_);
      Eigen::VectorXd leg_vel = sensor_data.jointVel_.segment(waistNum_, jointNum_);
      Eigen::VectorXd arm_vel = sensor_data.jointVel_.segment(waistNum_ + jointNum_, jointArmNum_);
      
      // 重新排列为：腿 + 腰 + 手
      full_joint_pos << leg_pos, waist_pos, arm_pos;
      full_joint_vel << leg_vel, waist_vel, arm_vel;
    }
    else
    {
      // 非 roban 机型：顺序已经是 腿 + 腰 + 手
      full_joint_pos = sensor_data.jointPos_.head(jointNum_ + waistNum_ + jointArmNum_);
      full_joint_vel = sensor_data.jointVel_.head(jointNum_ + waistNum_ + jointArmNum_);
    }

    // 调用 ArmController::update 进行统一的手臂控制
    // 注意：ArmController 内部会根据模式自动处理，模式1（RL控制）不会更新命令消息
    arm_controller_->update(
      time,
      dt,
      full_joint_pos,
      full_joint_vel,
      static_cast<int>(cmdData.cmdStance_),  // cmd_stance: 0=行走, 1=站立
      joint_cmd
    );

    // 检查当前模式，如果模式1（RL控制）则返回false，否则返回true
    if (arm_controller_->getMode() == 1)
    {
      // 模式1：RL控制，不替换手臂指令，返回false表示未使用外部手臂指令替换
      return false;
    }

    // 模式0或2：已使用外部手臂指令替换，返回true
    return true;
  }

  void AmpWalkController::initWaistControl()
  {
    // 初始化腰部控制器（如果启用了腰部控制功能）
    if (waist_command_replacement_enabled_ && waistNum_ > 0)
    {
      try
      {
        // 创建 WaistController 实例
        waist_controller_ = std::make_unique<WaistController>(
          nh_,
          waistNum_,
          ros_logger_,
          is_real_
        );
        
        // 使用从配置文件读取的 kp 和 kd 参数（如果已加载），否则使用默认值
        Eigen::VectorXd waist_kp, waist_kd;
        if (waist_kp_from_config_.size() == waistNum_ && waist_kd_from_config_.size() == waistNum_)
        {
          // 使用从配置文件读取的参数
          waist_kp = waist_kp_from_config_;
          waist_kd = waist_kd_from_config_;
        }
        else
        {
          // 如果未从配置文件加载，使用默认值
          waist_kp = Eigen::VectorXd::Constant(waistNum_, 10.0);
          waist_kd = Eigen::VectorXd::Constant(waistNum_, 2.0);
          ROS_WARN("[%s] Waist kp/kd not loaded from config, using default values (kp=10.0, kd=2.0)", name_.c_str());
        }
        
        // 获取默认腰部位置
        Eigen::VectorXd default_waist_pos;
        if (defalutJointPosRL_.size() >= jointNum_ + waistNum_)
        {
          if (is_roban_)
          {
            default_waist_pos = defalutJointPosRL_.segment(0, waistNum_);
          }
          else
          {
            default_waist_pos = defalutJointPosRL_.segment(jointNum_, waistNum_);
          }
        }
        else
        {
          default_waist_pos = Eigen::VectorXd::Zero(waistNum_);
          ROS_WARN("[%s] Cannot get default waist position, using zero vector", name_.c_str());
        }
        
        // 加载配置参数
        waist_controller_->loadSettings(
          waist_kp,
          waist_kd,
          default_waist_pos,
          waist_mode_interpolation_velocity_,
          waist_mode2_cutoff_freq_
        );
        
        // 腰部控制模式切换通过 /humanoid_controller/enable_waist_control 服务进行
        waist_controller_->enable(true);
        
        ROS_INFO("[%s] Waist controller initialized (default mode=1 RL control, waist_joints=%zu)", 
                 name_.c_str(), waistNum_);
      }
      catch (const std::exception& e)
      {
        ROS_ERROR("[%s] Failed to initialize waist controller: %s", name_.c_str(), e.what());
        waist_command_replacement_enabled_ = false;
        waist_controller_.reset();
      }
    }
    else
    {
      ROS_INFO("[%s] Waist command replacement disabled or no waist joints", name_.c_str());
    }
  }


  // 更新腰部指令（可选功能，用于替换jointCmdMsg中的腰部部分）
  bool AmpWalkController::updateWaistCommand(const ros::Time& time,
                                             const SensorData& sensor_data,
                                             kuavo_msgs::jointCmd& joint_cmd)
  {
    // 如果未启用腰部控制或没有腰部关节，直接返回false
    if (!waist_command_replacement_enabled_ || waistNum_ == 0 || !waist_controller_)
    {
      ROS_WARN_THROTTLE(1.0, "[%s] updateWaistCommand: disabled or no controller (enabled=%d, waistNum_=%zu, controller=%p)",
                          name_.c_str(), waist_command_replacement_enabled_, waistNum_, waist_controller_.get());
      return false;
    }
    
    // 获取控制周期
    double dt = dt_;
    if (dt <= 0.0 || dt > 0.1) dt = 0.002;  // 默认2ms

    // 获取当前命令数据
    CommandDataRL cmdData;
    if (gait_receiver_)
    {
      cmdData = gait_receiver_->getCurrentCommand();
    }

    // 构建完整的关节位置和速度向量（腿 + 腰 + 手）
    // 注意：WaistController 期望的顺序是：腿 + 腰 + 手
    // 对于 roban 机型，preprocessSensorData 已将顺序调整为：腰 + 腿 + 手
    // 所以需要重新排列为：腿 + 腰 + 手
    Eigen::VectorXd full_joint_pos(jointNum_ + waistNum_ + jointArmNum_);
    Eigen::VectorXd full_joint_vel(jointNum_ + waistNum_ + jointArmNum_);

    if (is_roban_)
    {
      // roban 机型：sensor_data 顺序是 腰 + 腿 + 手，需要调整为 腿 + 腰 + 手
      Eigen::VectorXd waist_pos = sensor_data.jointPos_.segment(0, waistNum_);
      Eigen::VectorXd leg_pos = sensor_data.jointPos_.segment(waistNum_, jointNum_);
      Eigen::VectorXd arm_pos = sensor_data.jointPos_.segment(waistNum_ + jointNum_, jointArmNum_);
      
      Eigen::VectorXd waist_vel = sensor_data.jointVel_.segment(0, waistNum_);
      Eigen::VectorXd leg_vel = sensor_data.jointVel_.segment(waistNum_, jointNum_);
      Eigen::VectorXd arm_vel = sensor_data.jointVel_.segment(waistNum_ + jointNum_, jointArmNum_);
      
      // 重新排列为：腿 + 腰 + 手
      full_joint_pos << leg_pos, waist_pos, arm_pos;
      full_joint_vel << leg_vel, waist_vel, arm_vel;
    }
    else
    {
      // 非 roban 机型：顺序已经是 腿 + 腰 + 手
      full_joint_pos = sensor_data.jointPos_.head(jointNum_ + waistNum_ + jointArmNum_);
      full_joint_vel = sensor_data.jointVel_.head(jointNum_ + waistNum_ + jointArmNum_);
    }

    // 调用 WaistController::update 进行腰部控制
    // 注意：WaistController 内部会根据模式自动处理，模式1（RL控制）不会更新命令消息
    waist_controller_->update(
      time,
      dt,
      full_joint_pos,
      full_joint_vel,
      static_cast<int>(cmdData.cmdStance_),  // cmd_stance: 0=行走, 1=站立
      joint_cmd,
      jointNum_  // 腰部在joint_cmd中的起始索引（即腿部关节数）
    );

    // 检查当前模式，如果模式1（RL控制）则返回false，否则返回true
    if (waist_controller_->getMode() == 1)
    {
      // 模式1：RL控制，不替换腰部指令，返回false表示未使用外部腰部指令替换
      return false;
    }

    // 模式0或2：已使用外部腰部指令替换，返回true
    return true;
  }

  void AmpWalkController::updateVelocityLimitsParam(ros::NodeHandle& nh)
  {
    // 将 4 维 velocityLimits_转换为 6 维 rosparam 格式
    // velocityLimits_格式：[linear_x, linear_y, linear_z, angular_z] （统一上限）
    // rosparam 格式：[linear_x, linear_y, linear_z, angular_x, angular_y, angular_z]
    std::vector<double> limits_vec(6);
    limits_vec[0] = velocityLimits_(0);  // linear_x (positive limit)
    limits_vec[1] = velocityLimits_(1);  // linear_y
    limits_vec[2] = velocityLimits_(2);  // linear_z
    limits_vec[3] = 0.0;                 // angular_x (通常为 0)
    limits_vec[4] = 0.0;                 // angular_y (通常为 0)
    limits_vec[5] = velocityLimits_(3);  // angular_z
    
    nh.setParam("/velocity_limits", limits_vec);
    
    ROS_INFO("[%s] Updated /velocity_limits from controller config: [%.2f, %.2f, %.2f, %.2f, %.2f, %.2f]",
             name_.c_str(),
             limits_vec[0], limits_vec[1], limits_vec[2],
             limits_vec[3], limits_vec[4], limits_vec[5]);
  }

} // namespace humanoid_controller
